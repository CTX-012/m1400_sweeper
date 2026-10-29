#!/usr/bin/env python3
"""
floor_camera  -  finds LOW obstacles (below the LiDAR beam) with the front camera.

How it works (no AI model, works for any object):
  1. At start-up it learns what your floor looks like (colour + brightness) from a small
     patch of floor just in front of the robot. Keep that patch clear for ~2 seconds.
  2. In every frame, anything in the robot's way that does NOT look like that floor is
     an obstacle.
  3. The camera height and tilt are known (camera_calibrate), so the image row where an
     obstacle touches the floor gives its real distance on the floor.
Publishes
  /camera/scan                LaserScan in base_footprint (robot centre) -> obstacle_guard
  /camera/obstacles/image     camera view with the floor patch (green) and obstacles (red)
Limits: works best on plain floors; patterned / glossy floors, strong shadows and objects
the same colour as the floor reduce accuracy. It cannot see the floor closer than the
bottom edge of the image (printed at start-up as "camera sees the floor from ...").
"""
import math
import os
import time

import numpy as np
import cv2
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Image

DEFAULT_CALIB = {        # used until you run camera_calibrate
    'cam_height': 0.089,     # m, lens centre above the floor (3.5 in)
    'cam_x': 0.088,          # m, lens ahead of the robot centre
    'pitch': 0.15,           # rad, tilted down
    'focal_px': 554.0,       # px at full resolution (about 60 deg wide)
    'image_width': 640,
    'image_height': 480,
}


def load_calib(path):
    c = dict(DEFAULT_CALIB)
    p = os.path.expanduser(path)
    if os.path.isfile(p):
        with open(p) as f:
            c.update(yaml.safe_load(f) or {})
        return c, True
    return c, False


class FloorGeometry:
    """Maps every pixel of the (small) working image to a point on the floor, robot frame."""

    def __init__(self, calib, pw, ph):
        sx = pw / float(calib['image_width'])
        f = calib['focal_px'] * sx
        cx, cy = pw / 2.0, ph / 2.0
        th, h = calib['pitch'], calib['cam_height']
        u = np.arange(pw) + 0.5
        v = np.arange(ph) + 0.5
        U, V = np.meshgrid(u, v)
        xc, yc = (U - cx) / f, (V - cy) / f
        denom = math.sin(th) + yc * math.cos(th)            # ray points down at the floor?
        self.valid = denom > 1e-3
        t = np.where(self.valid, h / np.where(self.valid, denom, 1.0), np.inf)
        self.X = t * (math.cos(th) - yc * math.sin(th)) + calib['cam_x']
        self.Y = -t * xc
        self.X[~self.valid] = np.inf
        self.Y[~self.valid] = 0.0
        self.f, self.cx, self.cy, self.th, self.h, self.camx = f, cx, cy, th, h, calib['cam_x']
        self.near = float(self.X[-1, pw // 2])               # closest floor the camera sees

    def ground(self, u, v):
        """Floor point (robot frame) for a continuous pixel position (u, v)."""
        xc, yc = (u - self.cx) / self.f, (v - self.cy) / self.f
        d = math.sin(self.th) + yc * math.cos(self.th)
        if d <= 1e-3:
            return math.inf, 0.0
        t = self.h / d
        return t * (math.cos(self.th) - yc * math.sin(self.th)) + self.camx, -t * xc

    def to_pixel(self, X, Y):
        dx = X - self.camx
        zc = dx * math.cos(self.th) + self.h * math.sin(self.th)
        yd = -dx * math.sin(self.th) + self.h * math.cos(self.th)
        return self.cx + self.f * (-Y) / zc, self.cy + self.f * yd / zc


class FloorDetector:
    def __init__(self, geom, ref_len=0.10, ref_half_width=0.08, search_half_width=0.35,
                 max_range=1.2, s_min=40, h_bins=30, s_bins=32, hist_thresh=0.0015,
                 v_low=0.6, v_high=1.3, min_run=4, learn_alpha=0.02):
        self.g = geom
        X, Y, ok = geom.X, geom.Y, geom.valid
        near = geom.near
        self.ref = ok & (X >= near + 0.02) & (X <= near + 0.02 + ref_len) & (np.abs(Y) <= ref_half_width)
        self.search = ok & (X >= near) & (X <= max_range) & (np.abs(Y) <= search_half_width)
        self.s_min, self.hb, self.sb = s_min, h_bins, s_bins
        self.thr, self.v_low, self.v_high = hist_thresh, v_low, v_high
        self.min_run, self.alpha = min_run, learn_alpha
        self.hist = None
        self.grey_frac = 0.0
        self.v_lo = self.v_hi = None
        self.frames = 0
        self.kernel = np.ones((3, 3), np.uint8)

    @staticmethod
    def prep(bgr, pw, ph):
        small = cv2.resize(bgr, (pw, ph), interpolation=cv2.INTER_AREA)
        # (no blur: it shifts the obstacle edge; the morphology below removes noise)
        return cv2.cvtColor(small, cv2.COLOR_BGR2HSV)

    def _hist(self, hsv, mask):
        H = hsv[..., 0][mask].astype(np.int32) * self.hb // 180
        S = hsv[..., 1][mask].astype(np.int32) * self.sb // 256
        hist = np.zeros((self.hb, self.sb), np.float32)
        np.add.at(hist, (np.clip(H, 0, self.hb - 1), np.clip(S, 0, self.sb - 1)), 1.0)
        hist = cv2.GaussianBlur(hist, (3, 3), 0)               # tolerate small colour changes
        return hist / max(hist.sum(), 1.0)

    def learn(self, hsv, mask=None, alpha=None):
        mask = self.ref if mask is None else mask
        if mask.sum() < 20:
            return
        a = (1.0 / (self.frames + 1)) if alpha is None else alpha
        h = self._hist(hsv, mask)
        V = hsv[..., 2][mask].astype(np.float32)
        lo, hi = np.percentile(V, 2), np.percentile(V, 98)
        grey = float((hsv[..., 1][mask] < self.s_min).mean())
        if self.hist is None:
            self.hist, self.v_lo, self.v_hi, self.grey_frac = h, lo, hi, grey
        else:
            self.hist = (1 - a) * self.hist + a * h
            self.v_lo = (1 - a) * self.v_lo + a * lo
            self.v_hi = (1 - a) * self.v_hi + a * hi
            self.grey_frac = (1 - a) * self.grey_frac + a * grey
        self.frames += 1

    def floor_mask(self, hsv):
        H = hsv[..., 0].astype(np.int32) * self.hb // 180
        S = hsv[..., 1].astype(np.int32)
        V = hsv[..., 2].astype(np.float32)
        p = self.hist[np.clip(H, 0, self.hb - 1), np.clip(S * self.sb // 256, 0, self.sb - 1)]
        colour_ok = np.where(S >= self.s_min, p >= self.thr, self.grey_frac >= 0.05)
        v_ok = (V >= self.v_lo * self.v_low) & (V <= self.v_hi * self.v_high + 15)
        return colour_ok & v_ok

    def detect(self, hsv):
        """Returns (obstacle points [(X, Y)], obstacle mask, floor fraction of the ref patch)."""
        floor = self.floor_mask(hsv)
        obst = (~floor & self.search).astype(np.uint8)
        obst = cv2.morphologyEx(obst, cv2.MORPH_OPEN, self.kernel)
        obst = cv2.morphologyEx(obst, cv2.MORPH_CLOSE, self.kernel).astype(bool) & self.search
        ph, pw = obst.shape
        k = self.min_run
        mb = obst[::-1]                                      # row 0 = bottom of the image
        run = mb[:ph - k + 1].copy()
        for i in range(1, k):
            run &= mb[i:ph - k + 1 + i]
        has = run.any(axis=0)
        first = np.argmax(run, axis=0)                       # lowest obstacle row per column
        cols = np.where(has)[0]
        keep = [u for u in cols if (u - 1 in cols) or (u + 1 in cols)]   # >= 2 columns wide
        pts = []
        for u in keep:
            v = ph - 1 - int(first[u])
            X, Y = self.g.ground(u + 0.5, v + 1.0)       # where the obstacle meets the floor
            pts.append((X, Y, u, v))
        ref_floor = float(floor[self.ref].mean()) if self.ref.any() else 0.0
        return pts, obst, ref_floor, floor


def points_to_scan(pts, max_range, n_bins=241, span=1.2):
    """Floor points (robot frame) -> LaserScan ranges around the robot centre."""
    inc = 2 * span / (n_bins - 1)
    ranges = [math.inf] * n_bins
    for X, Y, *_ in pts:
        a = math.atan2(Y, X)
        i = int(round((a + span) / inc))
        if 0 <= i < n_bins:
            ranges[i] = min(ranges[i], math.hypot(X, Y))
    return -span, span, inc, ranges


class FloorCamera(Node):
    def __init__(self):
        super().__init__('floor_camera')
        p = self.declare_parameter
        p('device', 0)                   # /dev/videoN of the real camera
        p('image_topic', '')             # set to use an image topic instead (e.g. Gazebo camera)
        p('calib_file', '~/.ros/jetbot_camera_floor.yaml')
        p('work_width', 320)             # processing resolution (accuracy vs speed)
        p('fps', 15.0)
        p('max_range', 1.2)              # m from the robot centre
        p('search_half_width', 0.35)     # m either side of the robot centre line
        p('learn_frames', 15)            # frames used to learn the floor at start-up
        p('hist_thresh', 0.0015)
        p('min_run', 4)
        p('show_window', False)
        p('debug_rate', 5.0)

        g = lambda n: self.get_parameter(n).value
        self.calib, found = load_calib(g('calib_file'))
        self.pw = int(g('work_width'))
        self.ph = int(round(self.pw * self.calib['image_height'] / self.calib['image_width']))
        self.geom = FloorGeometry(self.calib, self.pw, self.ph)
        self.det = FloorDetector(self.geom, search_half_width=float(g('search_half_width')),
                                 max_range=float(g('max_range')), hist_thresh=float(g('hist_thresh')),
                                 min_run=int(g('min_run')))
        self.max_range = float(g('max_range'))
        self.learn_frames = int(g('learn_frames'))
        self.show = bool(g('show_window'))
        self.debug_period = 1.0 / float(g('debug_rate'))
        self.t_debug = 0.0
        self.t_log = 0.0
        self.bad_ref_since = None

        self.pub_scan = self.create_publisher(LaserScan, '/camera/scan', 10)
        self.pub_img = self.create_publisher(Image, '/camera/obstacles/image', 2)
        self.cap = None
        self.device = int(g('device'))
        topic = g('image_topic')
        if topic:
            self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)
            src = f'topic {topic}'
        else:
            self.create_timer(1.0 / float(g('fps')), self.grab)
            src = f'/dev/video{self.device}'
        self.get_logger().info(
            f'floor_camera on {src}. Calibration: '
            f'{"loaded" if found else "NOT FOUND - using defaults, run camera_calibrate"}. '
            f'camera sees the floor from {self.geom.near:.2f} m (robot centre) out to {self.max_range:.2f} m. '
            f'Learning the floor: keep the area just in front of the robot clear for 2 s.')

    # ------------------------------------------------------------ input
    def grab(self):
        if self.cap is None or not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.device)
            if not self.cap.isOpened():
                self.get_logger().warn(f'cannot open /dev/video{self.device} (retrying)',
                                       throttle_duration_sec=5.0)
                self.cap = None
                return
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.calib['image_width'])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.calib['image_height'])
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('camera frame grab failed', throttle_duration_sec=5.0)
            self.cap.release()
            self.cap = None
            return
        self.process(frame)

    def on_image(self, msg):
        arr = np.frombuffer(bytes(msg.data), np.uint8)
        if msg.encoding in ('rgb8', 'bgr8'):
            img = arr.reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            self.process(img)

    # ------------------------------------------------------------ main
    def process(self, frame):
        hsv = FloorDetector.prep(frame, self.pw, self.ph)
        if self.det.frames < self.learn_frames:
            self.det.learn(hsv)
            if self.det.frames == self.learn_frames:
                self.get_logger().info('floor learned. Camera obstacle detection is on.')
            return
        pts, obst, ref_floor, floor = self.det.detect(hsv)
        # keep adapting to light changes, using only patch pixels that still look like floor
        if ref_floor > 0.6:
            self.det.learn(hsv, mask=self.det.ref & floor, alpha=self.det.alpha)
            self.bad_ref_since = None
        else:
            now = time.monotonic()
            self.bad_ref_since = self.bad_ref_since or now
            if now - self.bad_ref_since > 3.0:
                self.get_logger().warn(
                    'the floor patch in front of the robot no longer looks like floor '
                    '(something parked there, or the floor/lighting changed).',
                    throttle_duration_sec=10.0)

        amin, amax, inc, ranges = points_to_scan(pts, self.max_range)
        s = LaserScan()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = 'base_footprint'
        s.angle_min, s.angle_max, s.angle_increment = amin, amax, inc
        s.range_min = float(self.geom.near)         # the guard uses this as the camera's blind edge
        s.range_max = float(self.max_range + 0.5)
        s.ranges = [float(r) for r in ranges]
        self.pub_scan.publish(s)

        now = time.monotonic()
        if now - self.t_log > 1.0:
            self.t_log = now
            ahead = [(X, Y) for X, Y, *_ in pts if abs(Y) < 0.17 and X > 0]
            if ahead:
                X, Y = min(ahead)
                self.get_logger().info(
                    f'camera: low obstacle {X:.2f} m from robot centre, {math.degrees(math.atan2(Y, X)):+.0f} deg '
                    f'({len(pts)} columns) | floor patch {ref_floor * 100:.0f}% floor')
            else:
                self.get_logger().info(f'camera: path clear | floor patch {ref_floor * 100:.0f}% floor')
        if now - self.t_debug >= self.debug_period:
            self.t_debug = now
            self.publish_debug(frame, obst, pts)

    def publish_debug(self, frame, obst, pts):
        h, w = 240, 320
        img = cv2.resize(frame, (w, h))
        sx, sy = w / self.pw, h / self.ph
        red = cv2.resize(obst.astype(np.uint8) * 255, (w, h), interpolation=cv2.INTER_NEAREST) > 0
        img[red] = (0.5 * img[red] + [0, 0, 127]).astype(np.uint8)
        ref = cv2.resize(self.det.ref.astype(np.uint8) * 255, (w, h), interpolation=cv2.INTER_NEAREST)
        cnts, _ = cv2.findContours(ref, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, cnts, -1, (0, 255, 0), 1)
        for X, Y, u, v in pts:
            cv2.circle(img, (int((u + 0.5) * sx), int((v + 0.5) * sy)), 2, (0, 255, 255), -1)
        ahead = [X for X, Y, *_ in pts if abs(Y) < 0.17 and X > 0]
        txt = f'low obstacle {min(ahead):.2f} m' if ahead else 'path clear'
        cv2.putText(img, txt, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height, msg.width, msg.encoding, msg.step = h, w, 'bgr8', w * 3
        msg.data = img.tobytes()
        self.pub_img.publish(msg)
        if self.show:
            cv2.imshow('floor_camera (green = learned floor, red = obstacle)', img)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        if self.show:
            cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = FloorCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
