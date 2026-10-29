#!/usr/bin/env python3
"""
camera_calibrate  -  measures how your camera looks at the floor, so floor_camera can
turn "obstacle at image row N" into a real distance.

You need: a tape measure and some tape (or a ruler / sheet of paper with a straight edge).
  1. Robot on the floor, camera mounted as it will be used.
  2. For each distance asked (default 20, 40, 70 cm from the robot's FRONT edge), put a
     strip of tape across the floor at that distance, click the tape's edge that is
     nearest the robot in the camera window, then press SPACE.
  3. The tool fits the camera tilt and lens, prints the check, saves the result, and then
     shows distance lines on the live image so you can verify with the tape measure.
Run:  ros2 run jetbot camera_calibrate            (options: --help)
"""
import argparse
import math
import os

import numpy as np
import cv2
import yaml


def floor_x(rows, pitch, focal, cam_h, img_h):
    """Distance along the floor from the camera, for image rows (full resolution)."""
    yc = (np.asarray(rows, float) + 0.5 - img_h / 2.0) / focal
    d = np.sin(pitch) + yc * np.cos(pitch)
    with np.errstate(divide='ignore', invalid='ignore'):
        return np.where(d > 1e-3, cam_h * (np.cos(pitch) - yc * np.sin(pitch)) / d, np.inf)


def fit(rows, dist_cam, cam_h, img_h):
    """Least-squares camera pitch (rad) and focal length (px) from clicked rows."""
    rows = np.asarray(rows, float)[:, None, None]
    target = np.asarray(dist_cam, float)[:, None, None]
    best = None
    for p_lo, p_hi, p_n, f_lo, f_hi, f_n in ((-0.1, 1.2, 651, 120, 2500, 600),):
        P = np.linspace(p_lo, p_hi, p_n)[None, :, None]
        F = np.linspace(f_lo, f_hi, f_n)[None, None, :]
        err = ((floor_x(rows, P, F, cam_h, img_h) - target) ** 2).sum(axis=0)
        i, j = np.unravel_index(np.nanargmin(err), err.shape)
        best = (P[0, i, 0], F[0, 0, j])
    p0, f0 = best
    for span_p, span_f in ((0.01, 10.0), (0.001, 1.0)):     # refine twice
        P = np.linspace(p0 - span_p, p0 + span_p, 81)[None, :, None]
        F = np.linspace(f0 - span_f, f0 + span_f, 81)[None, None, :]
        err = ((floor_x(rows, P, F, cam_h, img_h) - target) ** 2).sum(axis=0)
        i, j = np.unravel_index(np.nanargmin(err), err.shape)
        p0, f0 = P[0, i, 0], F[0, 0, j]
    return float(p0), float(f0)


def row_of(dist_cam, pitch, focal, cam_h, img_h):
    """Image row where the floor is `dist_cam` metres from the camera."""
    zc = dist_cam * math.cos(pitch) + cam_h * math.sin(pitch)
    yd = -dist_cam * math.sin(pitch) + cam_h * math.cos(pitch)
    return img_h / 2.0 + focal * yd / zc - 0.5


def main():
    ap = argparse.ArgumentParser(description='Calibrate the JetBot floor camera.')
    ap.add_argument('--device', type=int, default=0, help='camera /dev/videoN')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--cam-height', type=float, default=0.089,
                    help='lens centre above the floor in metres (default 0.089 = 3.5 in)')
    ap.add_argument('--cam-x', type=float, default=0.088, help='lens ahead of the robot centre (m)')
    ap.add_argument('--front-edge', type=float, default=0.12, help='robot centre -> front edge (m)')
    ap.add_argument('--marks', type=float, nargs='+', default=[20, 40, 70],
                    help='tape distances in cm from the robot FRONT edge (default 20 40 70)')
    ap.add_argument('--out', default='~/.ros/jetbot_camera_floor.yaml')
    a = ap.parse_args()

    cap = cv2.VideoCapture(a.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    ok, frame = cap.read()
    if not ok:
        print(f'Cannot read /dev/video{a.device}. Is another program (floor_camera, yolo) using it?')
        return
    H, W = frame.shape[:2]
    print(f'Camera {W}x{H}. Lens height {a.cam_height:.3f} m - measure it and pass --cam-height if different!')
    win = 'camera_calibrate'
    cv2.namedWindow(win)
    state = {'y': None, 'click': None}

    def on_mouse(event, x, y, flags, param):
        state['y'] = y
        if event == cv2.EVENT_LBUTTONDOWN:
            state['click'] = (x, y)
    cv2.setMouseCallback(win, on_mouse)

    rows, dists = [], []
    for cm in a.marks:
        state['click'] = None
        print(f'\n>> Put tape across the floor {cm:.0f} cm in front of the robot front edge.'
              f'\n   Click the tape edge nearest the robot, then press SPACE (s = skip, ESC = stop).')
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            img = frame.copy()
            if state['y'] is not None:
                cv2.line(img, (0, state['y']), (W, state['y']), (0, 255, 255), 1)
            if state['click'] is not None:
                cv2.line(img, (0, state['click'][1]), (W, state['click'][1]), (0, 0, 255), 2)
            cv2.putText(img, f'tape at {cm:.0f} cm: click its near edge, SPACE to confirm',
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow(win, img)
            k = cv2.waitKey(30) & 0xFF
            if k == 27:
                break
            if k == ord('s'):
                break
            if k == 32 and state['click'] is not None:
                rows.append(state['click'][1])
                dists.append(cm / 100.0 + a.front_edge - a.cam_x)     # metres from the lens
                print(f'   recorded: row {state["click"][1]} for {cm:.0f} cm')
                break
        if k == 27:
            break

    if len(rows) < 2:
        print('Need at least 2 marks. Nothing saved.')
        cap.release()
        cv2.destroyAllWindows()
        return

    pitch, focal = fit(rows, dists, a.cam_height, H)
    pred = floor_x(rows, pitch, focal, a.cam_height, H)
    print(f'\nResult: camera tilted down {math.degrees(pitch):.1f} deg, lens {focal:.0f} px '
          f'(~{math.degrees(2 * math.atan(W / 2 / focal)):.0f} deg wide)')
    worst = 0.0
    for r, d, p in zip(rows, dists, pred):
        e = (p - d) * 100
        worst = max(worst, abs(e))
        print(f'   mark {100 * (d - a.front_edge + a.cam_x):5.1f} cm -> model says {100 * (p - a.front_edge + a.cam_x):5.1f} cm '
              f'(error {e:+.1f} cm)')
    if len(rows) >= 3 and worst > 2.0:
        print('   ! errors above 2 cm: check the lens height (--cam-height) and click the tape edges carefully.')
    near = float(floor_x([H - 1], pitch, focal, a.cam_height, H)[0]) + a.cam_x - a.front_edge
    print(f'   camera sees the floor from {near * 100:.0f} cm in front of the robot.')
    if near > 0.15:
        print('   ! tilt the camera down a little: low obstacles closer than that are invisible to it.')

    out = os.path.expanduser(a.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    data = {'cam_height': a.cam_height, 'cam_x': a.cam_x, 'pitch': round(pitch, 5),
            'focal_px': round(focal, 2), 'image_width': W, 'image_height': H}
    with open(out, 'w') as f:
        yaml.safe_dump(data, f)
    print(f'\nSaved {out}. Restart the launch to use it.')
    print('Now showing distance lines - check them with the tape measure. ESC to quit.')

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        for cm in (10, 20, 30, 50, 75, 100):
            d = cm / 100.0 + a.front_edge - a.cam_x
            y = int(round(row_of(d, pitch, focal, a.cam_height, H)))
            if 0 <= y < H:
                cv2.line(frame, (0, y), (W, y), (0, 255, 0), 1)
                cv2.putText(frame, f'{cm} cm', (6, y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (0, 255, 0), 1, cv2.LINE_AA)
        cv2.imshow(win, frame)
        if (cv2.waitKey(30) & 0xFF) == 27:
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
