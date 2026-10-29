#!/usr/bin/env python3
"""
obstacle_guard (v3)  -  LiDAR + camera safety stop for the JetBot (sim + real together).

  gamepad / Gazebo panel -> /cmd_vel_raw -> [obstacle_guard] -> /cmd_vel -> Gazebo robot
                                                                          -> arduino_driver -> real robot
  real RPLIDAR   -> /real/scan   (obstacles ~6 in and taller)  \
  floor_camera   -> /camera/scan (low obstacles on the floor)   >-> one obstacle map
  -> /scan (LiDAR for RViz), /obstacle_markers, distance logs

How it decides
  * Robot = circle (robot_radius + safety_margin). For the command you give right now
    (straight, curving or reversing) it follows the robot's path through BOTH sensors'
    obstacles and measures the free path left ("path" in the log, with the sensor that saw it).
  * Braking: speed is limited so the robot, with its reaction delay (latency) and braking
    (decel), comes to rest with exactly stop_distance of free path left.
  * After every stop it checks with the sensors where the robot really ended up and corrects
    itself (stop_correction), saved in learn_file so it keeps improving between runs.
  * Turning on the spot and moving away from things are always allowed.
  * ignore_sectors hides the robot's own parts / USB cables from the LiDAR.
  * Obstacle vanishing inside a sensor's blind zone -> HOLD until you let go of the stick.
  * No LiDAR data -> stop. Camera stops sending -> continue with LiDAR only (warning).
"""
import math
import os
import time as _time

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

LIDAR, CAMERA = 'lidar', 'camera'


# ---------------------------------------------------------------- geometry helpers
def ang_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def masked(bearing, r, ignore):
    """ignore = flat list [centre_deg, half_width_deg, max_range_m, ...]."""
    for i in range(0, len(ignore) - 2, 3):
        c, hw, mr = math.radians(ignore[i]), math.radians(ignore[i + 1]), ignore[i + 2]
        if r <= mr and abs(ang_diff(bearing, c)) <= hw:
            return True
    return False


def scan_points(ranges, angle_min, angle_inc, yaw, min_r, max_r, ignore, src=LIDAR):
    """Valid readings as (x, y, range, bearing, source) in the robot frame."""
    pts = []
    a = angle_min + yaw
    for r in ranges:
        ang = a
        a += angle_inc
        if not math.isfinite(r) or r < min_r or r > max_r:
            continue
        b = math.atan2(math.sin(ang), math.cos(ang))
        if ignore and masked(b, r, ignore):
            continue
        pts.append((r * math.cos(b), r * math.sin(b), r, b, src))
    return pts


def path_pose(s, v_sign, kappa):
    """Robot centre after driving arc length s with curvature kappa (= w/v)."""
    if abs(kappa) < 1e-6:
        return v_sign * s, 0.0
    a = kappa * v_sign * s
    return math.sin(a) / kappa, (1.0 - math.cos(a)) / kappa


def free_path(pts, v_sign, kappa, radius, horizon, step, k):
    """How far (m) the robot can drive along the arc before >= k readings come within `radius`
    of its centre while getting closer. Returns (distance or inf, contact (x, y, src), path)."""
    n = int(horizon / step) + 1
    if abs(kappa) < 1e-6:                       # straight: exact
        hits = []
        for q in pts:
            x, y = q[0], q[1]
            fx = v_sign * x
            if abs(y) < radius and fx > 0:
                hits.append((max(0.0, fx - math.sqrt(radius * radius - y * y)), x, y, q[4]))
        hits.sort()
        if len(hits) >= k and hits[k - 1][0] <= horizon:
            s = hits[k - 1][0]
            path = [path_pose(i * step, v_sign, 0.0) for i in range(int(s / step) + 1)]
            return s, hits[k - 1][1:], path
        return math.inf, None, [path_pose(i * step, v_sign, 0.0) for i in range(n)]
    rho = 1.0 / kappa                           # curve: step along it, round toward safe
    cand = [q for q in pts if abs(math.hypot(q[0], q[1] - rho) - abs(rho)) < radius]
    d0 = [q[0] * q[0] + q[1] * q[1] for q in cand]
    r2 = radius * radius
    path = []
    for i in range(n):
        s = i * step
        cx, cy = path_pose(s, v_sign, kappa)
        path.append((cx, cy))
        if not cand:
            continue
        inside = []
        for q, start in zip(cand, d0):
            d = (q[0] - cx) ** 2 + (q[1] - cy) ** 2
            if d < r2 and d < start - 1e-9:
                inside.append((d, q[0], q[1], q[4]))
        if len(inside) >= k:
            hit = min(inside)
            return max(0.0, s - step), hit[1:], path
    return math.inf, None, path


def brake_speed(free_left, decel, latency):
    """Highest speed that still stops within free_left (m), given reaction delay and braking."""
    if free_left <= 0.0:
        return 0.0
    al = decel * latency
    return -al + math.sqrt(al * al + 2.0 * decel * free_left)


def fmt(d):
    return '  -- ' if not math.isfinite(d) else f'{d:4.2f}m'


# ---------------------------------------------------------------- node
class ObstacleGuard(Node):
    def __init__(self):
        super().__init__('obstacle_guard')
        p = self.declare_parameter
        p('robot_radius', 0.12)          # m: centre -> outer edge of the wheels
        p('safety_margin', 0.03)         # m: extra space kept around the robot
        p('stop_distance', 0.12)         # m of free path to keep when stopped (target)
        p('decel', 0.6)                  # m/s^2 the robot slows down at (coasting)
        p('latency', 0.15)               # s from sensor reading to the motors reacting
        p('release_margin', 0.03)
        p('clear_scans', 2)
        p('min_points', 2)
        p('min_range', 0.15)             # RPLIDAR A1 blind zone
        p('max_range', 6.0)
        p('horizon', 1.2)                # m of path checked ahead
        p('ignore_sectors', [180.0, 35.0, 0.25])
        p('lidar_yaw_offset', 0.0)
        p('lidar_frame', 'lidar_link')
        p('use_camera', True)
        p('camera_timeout', 0.5)
        p('blind_hold', True)
        p('blind_band', 0.07)
        p('learn_stops', True)           # correct the braking point after every stop
        p('learn_gain', 0.5)
        p('learn_file', '~/.ros/jetbot_guard_learned.yaml')
        p('scan_timeout', 0.5)
        p('require_scan', True)
        p('cmd_timeout', 0.0)
        p('path_step', 0.01)
        p('scan_stamp_delay', 0.1)
        p('rate', 20.0)
        p('log_period', 0.5)
        p('self_check_scans', 30)

        g = lambda n: self.get_parameter(n).value
        self.body = float(g('robot_radius'))
        self.radius = self.body + float(g('safety_margin'))
        self.stop = float(g('stop_distance'))
        self.decel = float(g('decel'))
        self.latency = float(g('latency'))
        self.release = float(g('release_margin'))
        self.clear_scans = max(1, int(g('clear_scans')))
        self.k = max(1, int(g('min_points')))
        self.min_range = float(g('min_range'))
        self.max_range = float(g('max_range'))
        self.horizon = float(g('horizon'))
        self.ignore = [float(x) for x in g('ignore_sectors')]
        self.yaw = float(g('lidar_yaw_offset'))
        self.frame = g('lidar_frame')
        self.use_camera = bool(g('use_camera'))
        self.camera_timeout = float(g('camera_timeout'))
        self.blind_hold = bool(g('blind_hold'))
        self.blind_band = float(g('blind_band'))
        self.learn = bool(g('learn_stops'))
        self.gain = float(g('learn_gain'))
        self.learn_file = os.path.expanduser(g('learn_file'))
        self.scan_timeout = float(g('scan_timeout'))
        self.require_scan = bool(g('require_scan'))
        self.cmd_timeout = float(g('cmd_timeout'))
        self.step = float(g('path_step'))
        self.stamp_delay = Duration(seconds=float(g('scan_stamp_delay')))
        self.log_period = float(g('log_period'))

        self.comp = self.load_correction()
        self.lidar_pts, self.cam_pts = [], []
        self.t_scan = self.t_cam = None
        self.cam_near = 0.0
        self.cam_warned = False
        self.scan_seq = 0
        self.cmd = Twist()
        self.t_cmd = None
        self.stopped = {1: False, -1: False}
        self.clear_count = {1: 0, -1: 0}
        self.counted_seq = {1: -1, -1: -1}
        self.hold = {1: False, -1: False}
        self.close_prev = {(LIDAR, 1): 0, (LIDAR, -1): 0, (CAMERA, 1): 0}
        self.event = None                 # stop being measured for learning
        self.state = ''
        self.t_log = 0.0
        self.last_path, self.last_hit = [], None
        self.view = {'ahead': math.inf, 'low': math.inf, 'behind': math.inf,
                     'nearest': math.inf, 'nearest_b': 0.0}
        self.sc_n = int(g('self_check_scans'))
        self.sc_bins, self.sc_count = {}, 0

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_ahead = self.create_publisher(Float32, '/obstacle/ahead', 10)
        self.pub_mk = self.create_publisher(MarkerArray, '/obstacle_markers', 10)
        self.create_subscription(LaserScan, '/real/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/camera/scan', self.on_camera, 10)
        self.create_subscription(Twist, '/cmd_vel_raw', self.on_cmd, 10)
        self.create_timer(1.0 / float(g('rate')), self.tick,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f'obstacle_guard v3: stops with {self.stop:.2f} m free path '
            f'(= {100 * (self.stop + self.radius - self.body):.0f} cm from the robot edge), '
            f'decel {self.decel} m/s^2, latency {self.latency} s, learned correction '
            f'{self.comp:+.3f} m, camera {"ON" if self.use_camera else "off"}, '
            f'ignoring {self.ignore}')

    # ---------------------------------------------------------- learning store
    def load_correction(self):
        try:
            with open(self.learn_file) as f:
                return float((yaml.safe_load(f) or {}).get('stop_correction', 0.0))
        except (OSError, ValueError, yaml.YAMLError):
            return 0.0

    def save_correction(self):
        try:
            os.makedirs(os.path.dirname(self.learn_file), exist_ok=True)
            with open(self.learn_file, 'w') as f:
                yaml.safe_dump({'stop_correction': round(self.comp, 4)}, f)
        except OSError as e:
            self.get_logger().warn(f'could not save {self.learn_file}: {e}')

    # ---------------------------------------------------------- inputs
    def on_cmd(self, msg):
        self.cmd = msg
        self.t_cmd = _time.monotonic()

    def on_scan(self, s):
        self.t_scan = _time.monotonic()
        self.scan_seq += 1
        self.self_check(s)
        self.lidar_pts = scan_points(s.ranges, s.angle_min, s.angle_increment, self.yaw,
                                     self.min_range, self.max_range, self.ignore, LIDAR)
        self.check_vanish(LIDAR, self.lidar_pts, self.min_range, (1, -1))
        now = self.get_clock().now()
        if now.nanoseconds > self.stamp_delay.nanoseconds:
            now = now - self.stamp_delay
        s.header.stamp = now.to_msg()
        s.header.frame_id = self.frame
        s.angle_min += self.yaw
        s.angle_max += self.yaw
        self.pub_scan.publish(s)

    def on_camera(self, s):
        if not self.use_camera:
            return
        self.t_cam = _time.monotonic()
        self.cam_near = float(s.range_min)
        self.cam_pts = scan_points(s.ranges, s.angle_min, s.angle_increment, 0.0, 0.0,
                                   float(s.range_max), [], CAMERA)
        self.check_vanish(CAMERA, self.cam_pts, self.cam_near, (1,))
        if self.cam_warned:
            self.get_logger().info('camera is back: low-obstacle detection ON')
            self.cam_warned = False

    def camera_ok(self, now):
        if not self.use_camera:
            return False
        ok = self.t_cam is not None and now - self.t_cam <= self.camera_timeout
        if not ok and not self.cam_warned and self.t_scan is not None:
            self.get_logger().warn('no camera data: LOW obstacles are NOT detected (LiDAR only)')
            self.cam_warned = True
        return ok

    def check_vanish(self, src, pts, blind, dirs):
        """Seen just outside a sensor's blind zone, then gone with nothing further out in that
        direction -> probably INSIDE the blind zone. Hold that direction."""
        edge = blind + self.blind_band
        for d in dirs:
            half = [q for q in pts if d * q[0] > 0]
            close = sum(1 for q in half if q[2] < edge)
            near_out = sum(1 for q in half if q[2] < edge + 0.10)
            if (self.blind_hold and self.close_prev[(src, d)] >= self.k and close == 0
                    and near_out == 0 and not self.hold[d]):
                self.hold[d] = True
                self.get_logger().warn(
                    f'HOLD: obstacle {"ahead" if d > 0 else "behind"} vanished from the {src} '
                    f'(too close for it to see?). Let go of the stick, check, then drive again.')
            self.close_prev[(src, d)] = close

    def self_check(self, s):
        if self.sc_count >= self.sc_n:
            return
        self.sc_count += 1
        seen = {}
        a = s.angle_min + self.yaw
        for r in s.ranges:
            ang = a
            a += s.angle_increment
            if math.isfinite(r) and self.min_range <= r < 0.35:
                b = int(round(math.degrees(math.atan2(math.sin(ang), math.cos(ang))) / 10.0)) * 10
                b = 180 if b == -180 else b
                seen[b] = min(r, seen.get(b, 9.0))
        for b, r in seen.items():
            self.sc_bins.setdefault(b, []).append(r)
        if self.sc_count == self.sc_n:
            found = sorted((b, sorted(v)[len(v) // 2]) for b, v in self.sc_bins.items()
                           if len(v) >= 0.8 * self.sc_n)
            if not found:
                self.get_logger().info('self-check: nothing permanently close to the robot. Good.')
            for b, r in found:
                hid = 'already ignored' if masked(math.radians(b), r, self.ignore) else \
                    'NOT ignored - add it to ignore_sectors if nothing is really there'
                self.get_logger().warn(
                    f'self-check: something stays at {r:.2f} m, {b:+d} deg the whole time '
                    f'(robot part or USB cable?) -> {hid}')

    # ---------------------------------------------------------- main loop
    def tick(self):
        now = _time.monotonic()
        v, w = self.cmd.linear.x, self.cmd.angular.z
        if self.cmd_timeout > 0 and (self.t_cmd is None or now - self.t_cmd > self.cmd_timeout):
            v, w = 0.0, 0.0
        v_in = v
        cam = self.camera_ok(now)
        pts = self.lidar_pts + (self.cam_pts if cam else [])
        fp = lambda P, d: free_path(P, d, 0.0, self.radius, self.horizon, self.step, self.k)[0]
        view = self.view
        view['ahead'] = fp(self.lidar_pts, 1)
        view['low'] = fp(self.cam_pts, 1) if cam else math.inf
        view['behind'] = fp(self.lidar_pts, -1)
        near = min(pts, key=lambda q: q[2]) if pts else None
        view['nearest'], view['nearest_b'] = (near[2], near[3]) if near else (math.inf, 0.0)
        fwd = min(view['ahead'], view['low'])
        target = self.stop + self.comp
        path_free, src = math.inf, ''
        self.last_path, self.last_hit = [], None

        stale = self.t_scan is None or now - self.t_scan > self.scan_timeout
        if not self.require_scan:
            state = 'NO GUARD (LiDAR off)'
        elif stale:
            v, w = 0.0, 0.0
            state = 'STOP - NO LIDAR DATA'
        elif abs(v) < 0.005:
            v = 0.0
            self.hold = {1: False, -1: False}            # stick released = operator confirmed
            for d, free in ((1, fwd), (-1, view['behind'])):
                self.try_release(d, free, target)
            blocked = [n for d, n in ((1, 'ahead'), (-1, 'behind')) if self.stopped[d]]
            state = 'BLOCKED ' + '+'.join(blocked) + ' (turn or drive away)' if blocked else 'CLEAR'
        else:
            d = 1 if v > 0 else -1
            side = 'ahead' if d > 0 else 'behind'
            path_free, hit, path = free_path(pts, d, w / v, self.radius, self.horizon,
                                             self.step, self.k)
            src = hit[2] if hit else ''
            self.last_path, self.last_hit = path, hit
            was_stopped = self.stopped[d]
            if self.hold[d]:
                v, state = 0.0, f'HOLD - obstacle vanished {side} (let go of the stick)'
            elif path_free <= target or (was_stopped and path_free < target + self.release):
                v = 0.0
                self.stopped[d] = True
                self.clear_count[d] = 0
                state = f'STOP - {src} obstacle {path_free:.2f} m {side} on the path'
                if not was_stopped and abs(v_in) >= 0.08 and abs(w) <= 0.3:
                    self.event = {'d': d, 't': now, 'vals': [], 'seq': -1, 'src': src}
            else:
                self.try_release(d, path_free, target)
                if self.stopped[d]:
                    v, state = 0.0, f'STOP - checking the path {side} is really clear'
                else:
                    v_lim = brake_speed(path_free - target, self.decel, self.latency)
                    if abs(v) > v_lim:
                        v, state = d * v_lim, f'SLOW ({src})'
                    else:
                        state = 'CLEAR'
            self.try_release(-d, view['behind'] if d > 0 else fwd, target)

        self.measure_stop(now, v, fwd)

        out = Twist()
        out.linear.x = v
        out.angular.z = w
        self.pub_cmd.publish(out)
        self.pub_ahead.publish(Float32(data=float(fwd) if math.isfinite(fwd) else -1.0))

        line = (f'lidar {fmt(view["ahead"])} | camera {fmt(view["low"]) if cam else " off "} | '
                f'behind {fmt(view["behind"])} | path {fmt(path_free)}{"(" + src[0] + ")" if src else "   "} | '
                f'cmd {v_in:+.2f} -> {v:+.2f} m/s | {state}')
        changed = state.split(' ')[0] != self.state.split(' ')[0]
        if changed and state.startswith(('STOP', 'HOLD')):
            self.get_logger().warn(line)
            self.t_log = now
        elif changed or now - self.t_log >= self.log_period:
            self.get_logger().info(line)
            self.t_log = now
        self.state = state
        self.publish_markers(state, target, v_in)

    def try_release(self, d, free, target):
        if not self.stopped[d]:
            return
        if free <= target + self.release:
            self.clear_count[d] = 0
            return
        if self.counted_seq[d] != self.scan_seq:
            self.counted_seq[d] = self.scan_seq
            self.clear_count[d] += 1
        if self.clear_count[d] >= self.clear_scans:
            self.stopped[d] = False
            self.clear_count[d] = 0

    def measure_stop(self, now, v_out, fwd):
        """After a stop: wait until the robot has settled, read how much free path is really
        left, and nudge the braking point so the next stop lands on stop_distance."""
        ev = self.event
        if ev is None or not self.learn:
            return
        age = now - ev['t']
        if v_out != 0.0 or age > 2.5:                    # driven again / obstacle kept moving
            self.event = None
            return
        if age < 0.6 or ev['seq'] == self.scan_seq:
            return
        ev['seq'] = self.scan_seq
        free = fwd if ev['d'] > 0 else self.view['behind']
        if not math.isfinite(free):
            self.event = None
            return
        ev['vals'].append(free)
        last = ev['vals'][-3:]
        if len(last) == 3 and max(last) - min(last) <= 0.01:
            final = sorted(last)[1]
            err = self.stop - final                      # > 0: stopped too close
            old = self.comp
            self.comp = min(0.20, max(-0.05, self.comp + self.gain * err))
            self.save_correction()
            how = 'too close' if err > 0 else 'too early'
            self.get_logger().info(
                f'stop check ({ev["src"]}): wanted {self.stop:.2f} m free, stopped at {final:.2f} m '
                f'({abs(err) * 100:.1f} cm {how}) -> braking point correction '
                f'{old:+.3f} -> {self.comp:+.3f} m (saved)')
            self.event = None

    # ---------------------------------------------------------- RViz
    def publish_markers(self, state, target, v_cmd):
        if state.startswith(('STOP', 'BLOCKED', 'HOLD')):
            rgba = (1.0, 0.1, 0.1, 0.45)
        elif state.startswith('SLOW'):
            rgba = (1.0, 0.8, 0.0, 0.40)
        else:
            rgba = (0.1, 0.9, 0.2, 0.25)

        def mk(mid, mtype, frame='base_footprint'):
            m = Marker()
            m.header.frame_id = frame          # stamp 0 = latest transform
            m.ns, m.id, m.type, m.action = 'guard', mid, mtype, Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        v = abs(v_cmd)
        brake = v * self.latency + v * v / (2 * self.decel)
        zone_len = max(0.05, target + brake)
        zone = mk(0, Marker.CUBE)
        zone.pose.position.x = self.radius + zone_len / 2.0
        zone.pose.position.z = 0.003
        zone.scale.x, zone.scale.y, zone.scale.z = zone_len, 2 * self.radius, 0.004
        zone.color.r, zone.color.g, zone.color.b, zone.color.a = rgba

        stop_line = mk(1, Marker.CUBE)
        stop_line.pose.position.x = self.radius + target
        stop_line.pose.position.z = 0.005
        stop_line.scale.x, stop_line.scale.y, stop_line.scale.z = 0.01, 2 * self.radius, 0.006
        stop_line.color.r, stop_line.color.a = 1.0, 0.9

        hit = mk(2, Marker.SPHERE, self.frame)
        if self.last_hit is not None:
            hit.pose.position.x, hit.pose.position.y = self.last_hit[0], self.last_hit[1]
            hit.scale.x = hit.scale.y = hit.scale.z = 0.05
            if self.last_hit[2] == CAMERA:
                hit.color.r, hit.color.g, hit.color.a = 1.0, 1.0, 1.0    # yellow = camera
            else:
                hit.color.r, hit.color.b, hit.color.a = 1.0, 1.0, 1.0    # magenta = LiDAR
        else:
            hit.action = Marker.DELETE

        path = mk(3, Marker.LINE_STRIP)
        if self.last_path:
            for x, y in self.last_path:
                pt = Point()
                pt.x, pt.y, pt.z = float(x), float(y), 0.01
                path.points.append(pt)
            path.scale.x = 2 * self.radius
            path.color.r, path.color.g, path.color.b, path.color.a = rgba[0], rgba[1], rgba[2], 0.25
        else:
            path.action = Marker.DELETE

        text = mk(4, Marker.TEXT_VIEW_FACING)
        text.pose.position.z = 0.35
        text.scale.z = 0.06
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        f = min(self.view['ahead'], self.view['low'])
        text.text = f'ahead {fmt(f).strip()}  {state.split(" -")[0].split(" (")[0]}'

        arr = MarkerArray()
        arr.markers = [zone, stop_line, hit, path, text]
        self.pub_mk.publish(arr)


def main():
    rclpy.init()
    node = ObstacleGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub_cmd.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
