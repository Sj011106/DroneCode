# =====================================================================================
# tools/degrade_video.py  -  make test images/video HARDER on purpose
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   We can't crash-test on the real drone every time we change a line of code. So this
#   file does two things:
#   1. render(): a tiny camera simulator. Give it where the drone is, how it's tilted,
#      and how long the rolling-shutter scan takes, and it draws what a downward camera
#      would see of a textured ground. Because WE chose the motion, the tests know the
#      true answer and can measure our error exactly.
#   2. Command line: take a REAL recorded video and add jello wobble (rolling shutter
#      under vibration), blur, darkness, noise or full blackouts, to see how the
#      tracker and the health checks cope.
#
# HOW IT WORKS (technical)
#   render(): ray-casting a flat ground plane. For each pixel (u, v), the camera ray
#   K^-1 [u, v, 1] is rotated into North/East/Down with the pose AT THAT ROW'S TIME
#   (t_mid + (v/H - 0.5) * readout), intersected with the plane z = 0, and the ground
#   texture is sampled there (cv2.remap). readout = 0 gives a global-shutter image.
#   Row timing matches vision/timing.py, pose convention matches fusion/tilt.py.
#
# HOW TO RUN (from the repo root)
#   python -m tools.degrade_video in.mp4 out.mp4 --wobble 6 12   # 6 px jello at 12 Hz
#   python -m tools.degrade_video in.mp4 out.mp4 --blur 2 --noise 5
#   python -m tools.degrade_video in.mp4 out.mp4 --blackout 3 2 --blackout 10 10
#                                                  # black from 3 s for 2 s, and 10 s for 10 s
#   python -m tools.degrade_video in.mp4 out.mp4 --dark 0.3
#
# REFERENCES
#   - Hartley & Zisserman, "Multiple View Geometry", 2nd ed., ch. 6 (pinhole model).
#   - Meingast, Geyer, Sastry, "Geometric Models of Rolling-Shutter Cameras", OMNIVIS
#     2005 (row-time camera model used here).
#   - OpenCV cv2.remap: https://docs.opencv.org/4.x/da/d54/group__imgproc__transform.html
# =====================================================================================

import argparse
import math

import cv2
import numpy as np

from fusion.tilt import R_BODY_CAM, body_to_ned

M_PER_TEXEL = 0.01      # ground texture: one texel = 1 cm


def make_ground(size=2000, seed=0):
    """Random gravel/grass-like texture with some blocks (same recipe as test_tracker)."""
    rng = np.random.default_rng(seed)
    g = rng.integers(0, 256, (size, size)).astype(np.uint8)
    g = cv2.GaussianBlur(g, (0, 0), 3)
    g = cv2.normalize(g, None, 0, 255, cv2.NORM_MINMAX)
    for _ in range(size * size // 17000):
        x, y = rng.integers(0, size, 2)
        cv2.rectangle(g, (int(x), int(y)), (int(x + rng.integers(10, 60)), int(y + rng.integers(10, 60))),
                      int(rng.integers(0, 256)), -1)
    return g


def render(ground, K, size, pose_fn, t_mid, readout_s=0.0, noise=0.0, rng=None,
           m_per_texel=M_PER_TEXEL):
    """What a nadir camera sees.
      pose_fn(t) -> (north_m, east_m, height_m, roll, pitch, yaw)   (radians)
      t_mid      : time of the middle row; row v is captured at t_mid + (v/H - 0.5)*readout_s
    Ground texture centre = north 0, east 0."""
    W, H = size
    rows = np.arange(H)
    poses = np.array([pose_fn(t) for t in t_mid + (rows / H - 0.5) * readout_s])   # (H, 6)
    R = body_to_ned(poses[:, 3], poses[:, 4], poses[:, 5]) @ R_BODY_CAM            # (H, 3, 3)
    u, v = np.meshgrid(np.arange(W, dtype=float), rows.astype(float))
    rays = np.stack([u, v, np.ones_like(u)], -1) @ np.linalg.inv(K).T              # (H, W, 3)
    d = np.einsum("hij,hwj->hwi", R, rays)                                          # rays in NED
    s = poses[:, 2][:, None] / d[..., 2]                                            # distance to ground
    north = poses[:, 0][:, None] + s * d[..., 0]
    east = poses[:, 1][:, None] + s * d[..., 1]
    gh, gw = ground.shape
    map_x = (gw / 2 + east / m_per_texel).astype(np.float32)
    map_y = (gh / 2 - north / m_per_texel).astype(np.float32)                       # north = up
    img = cv2.remap(ground, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
    if noise:
        rng = rng or np.random.default_rng()
        img = np.clip(img + rng.normal(0, noise, img.shape), 0, 255).astype(np.uint8)
    return img


# ---------------- Effects for real video ----------------
def wobble(frame, t, amp_px, freq_hz, readout_s=0.025):
    """Rolling-shutter jello: each row shifted sideways by a vibration sampled at its row time."""
    h, w = frame.shape[:2]
    row_t = t + (np.arange(h) / h - 0.5) * readout_s
    shift = amp_px * np.sin(2 * math.pi * freq_hz * row_t)
    map_x = (np.arange(w)[None, :] + shift[:, None]).astype(np.float32)
    map_y = np.repeat(np.arange(h, dtype=np.float32)[:, None], w, 1)
    return cv2.remap(frame, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)


def main():
    ap = argparse.ArgumentParser(description="Add rolling-shutter wobble, blur, noise, darkness, blackouts")
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--wobble", nargs=2, type=float, metavar=("AMP_PX", "FREQ_HZ"))
    ap.add_argument("--readout", type=float, default=0.025, help="seconds, for --wobble")
    ap.add_argument("--blur", type=float, default=0, help="Gaussian sigma in px")
    ap.add_argument("--noise", type=float, default=0, help="noise sigma in grey levels")
    ap.add_argument("--dark", type=float, default=1.0, help="brightness factor, e.g. 0.3")
    ap.add_argument("--blackout", nargs=2, type=float, action="append", default=[],
                    metavar=("START_S", "DURATION_S"))
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.input)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    out = cv2.VideoWriter(args.output, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    rng = np.random.default_rng(0)
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t = i / fps
        if args.wobble:
            frame = wobble(frame, t, *args.wobble, readout_s=args.readout)
        if args.blur:
            frame = cv2.GaussianBlur(frame, (0, 0), args.blur)
        if args.dark != 1.0:
            frame = cv2.convertScaleAbs(frame, alpha=args.dark)
        if args.noise:
            frame = np.clip(frame + rng.normal(0, args.noise, frame.shape), 0, 255).astype(np.uint8)
        if any(s <= t < s + d for s, d in args.blackout):
            frame[:] = 0
        out.write(frame)
        i += 1
    cap.release()
    out.release()
    print(f"Wrote {i} frames to {args.output}")


if __name__ == "__main__":
    main()
