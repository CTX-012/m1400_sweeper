#!/usr/bin/env python3
"""
arduino_driver  -  drives the REAL JetBot from /cmd_vel.

/cmd_vel (geometry_msgs/Twist)  ->  "left,right\\n" PWM (-255..255)  ->  Arduino Uno -> L298N

The same /cmd_vel also drives the Gazebo robot, so sim and hardware move together.

Behaviour
  * Re-sends the latest command at `rate` Hz (keeps the Arduino's 400 ms failsafe fed,
    and matches Gazebo, which keeps executing the last command it received).
  * If `cmd_timeout` > 0, the robot stops when no new /cmd_vel arrives for that long.
  * If this node or the PC dies, sending stops and the Arduino failsafe stops the motors.
  * Reconnects automatically if the USB cable is unplugged and plugged back in.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


def twist_to_pwm(v, w, track, vmax, min_pwm, max_pwm, fwd_sign=1, turn_sign=1):
    """Convert a body velocity (m/s, rad/s) into left/right PWM (-max_pwm..max_pwm)."""
    v *= fwd_sign
    w *= turn_sign
    vl = v - w * track / 2.0
    vr = v + w * track / 2.0
    # if either wheel is asked for more than the motors can do, scale BOTH
    # down together so the robot still follows the same curve
    peak = max(abs(vl), abs(vr))
    if peak > vmax:
        vl *= vmax / peak
        vr *= vmax / peak

    def to_pwm(s):
        if abs(s) < 1e-3:
            return 0
        # min_pwm = the PWM where the motors just start turning (deadband)
        p = min_pwm + (max_pwm - min_pwm) * abs(s) / vmax
        return int(math.copysign(min(round(p), max_pwm), s))

    return to_pwm(vl), to_pwm(vr)


class ArduinoDriver(Node):
    def __init__(self):
        super().__init__('arduino_driver')
        self.declare_parameter('port', '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_separation', 0.20)   # m, must match the URDF
        self.declare_parameter('max_wheel_speed', 0.60)    # m/s at PWM 255 (calibrate!)
        self.declare_parameter('min_pwm', 70)              # PWM where wheels start moving
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('forward_sign', 1)          # -1 if forward drives backward
        self.declare_parameter('turn_sign', -1)            # -1 = the left/right fix from your teleop node
        self.declare_parameter('rate', 20.0)               # Hz, re-send rate
        self.declare_parameter('cmd_timeout', 0.0)         # s, 0 = hold last command like Gazebo
        self.declare_parameter('boot_wait', 2.0)           # s, Uno resets when the port opens

        g = lambda n: self.get_parameter(n).value
        self.port = g('port')
        self.baud = int(g('baud'))
        self.track = float(g('wheel_separation'))
        self.vmax = float(g('max_wheel_speed'))
        self.min_pwm = int(g('min_pwm'))
        self.max_pwm = int(g('max_pwm'))
        self.fwd_sign = int(g('forward_sign'))
        self.turn_sign = int(g('turn_sign'))
        self.cmd_timeout = float(g('cmd_timeout'))
        self.boot_wait = float(g('boot_wait'))

        self.ser = None
        self.ready_at = 0.0
        self.next_retry = 0.0
        self.v = 0.0
        self.w = 0.0
        self.t_cmd = None
        self.last_sent = (None, None)
        self._n = 0

        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_timer(1.0 / float(g('rate')), self.tick)
        self.get_logger().info(
            f'arduino_driver: port={self.port} track={self.track} m '
            f'vmax={self.vmax} m/s min_pwm={self.min_pwm} turn_sign={self.turn_sign}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.t_cmd = self.now()

    def connect(self):
        t = self.now()
        if t < self.next_retry:
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05, write_timeout=0.1)
            self.ready_at = t + self.boot_wait
            self.get_logger().info(f'opened {self.port}, waiting {self.boot_wait:.1f}s for the Uno to boot')
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.next_retry = t + 2.0
            self.get_logger().warn(f'cannot open {self.port}: {e} (retrying)', throttle_duration_sec=5.0)

    def tick(self):
        if self.ser is None:
            self.connect()
            return
        t = self.now()
        if t < self.ready_at:
            return
        v, w = self.v, self.w
        if self.cmd_timeout > 0 and (self.t_cmd is None or t - self.t_cmd > self.cmd_timeout):
            v, w = 0.0, 0.0
        left, right = twist_to_pwm(v, w, self.track, self.vmax, self.min_pwm, self.max_pwm,
                                   self.fwd_sign, self.turn_sign)
        try:
            self.ser.write(f'{left},{right}\n'.encode())
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'serial write failed: {e}; reconnecting')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.next_retry = t + 1.0
            return
        self._n += 1
        if (left, right) != self.last_sent or self._n % 40 == 0:
            self.get_logger().info(f'cmd v={v:+.2f} m/s w={w:+.2f} rad/s  ->  L={left:+4d} R={right:+4d}')
            self.last_sent = (left, right)

    def stop_motors(self):
        if self.ser is not None:
            try:
                self.ser.write(b'0,0\n')
                self.ser.flush()
                self.ser.close()
            except Exception:
                pass


def main():
    rclpy.init()
    node = ArduinoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
