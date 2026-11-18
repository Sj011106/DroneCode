# calibration/auto_calibrate.py
# Hands-free camera calibration: the automatic version of calibrate.py.
# No 'c' or 'q' keys:
#   - it takes a snapshot BY ITSELF whenever the checkerboard is fully visible,
#     sharp, held still, and seen from a NEW position or angle;
#   - it finishes BY ITSELF after TARGET_VIEWS snapshots (or when time runs out).
#
# How to run:
#   python -m calibration.auto_calibrate               -> camera 0
#   python -m calibration.auto_calibrate 2             -> camera 2
#   python -m calibration.auto_calibrate /dev/video2   -> the Justice D on the APB (CSI1)
#   python -m calibration.auto_calibrate video.mp4     -> a video you recorded of the board
#   add  --no-window  on a computer without a screen (e.g. the drone)
#   add  --square 0.031  to give the measured square size (metres) without editing the file
#
# Justice D: 1280x720, 114 deg wide. Calibrate at the camera's FULL size (vision_hold.py
# scales K down to the 640x360 it tracks at). If the normal lens model fits badly (wide
# lenses bend a lot near the edges), it automatically tries OpenCV's rational model
# (8 distortion numbers) and keeps whichever fits better; tracker.py needs no change.
#
# What YOU do: slowly move the checkerboard (or the camera) around:
#   near and far, tilted left/right/up/down, and into every corner of the picture.
#   Pause for a moment in each new spot. The program does the rest.

# -------------------------------------------------------------------------------------
# IN PLAIN WORDS (for non-technical readers)
#   Same as calibrate.py (measures the lens), but takes the checkerboard photos by itself
#   when the board is sharp, still and in a new position, so it can run on the drone.
# REFERENCES
#   - Zhang, "A Flexible New Technique for Camera Calibration", IEEE TPAMI 2000.
#   - OpenCV tutorial: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
# -------------------------------------------------------------------------------------

import argparse
import time

import cv2
import numpy as np

# ---------------- Settings (check these match YOUR checkerboard!) ----------------
BOARD = (8, 5)           # INNER corners of the checkerboard (columns, rows)
SQUARE_M = 0.025         # size of one square in metres (2.5 cm). Measure yours!
WIDTH, HEIGHT = 1280, 720  # Justice D's fixed output; asked for on webcams (the Justice D ignores it)
TRY_WIDE_MODEL_ABOVE_PX = 0.5   # normal-model error above this -> also try the rational model
TARGET_VIEWS = 20        # stop automatically after this many snapshots
MIN_VIEWS = 10           # fewest snapshots that still give a usable result
MAX_SECONDS = 300        # give up waiting after 5 minutes (uses what it has)
MIN_GAP_S = 0.7          # at least this long between two snapshots
STILL_PX = 1.5           # board must move less than this (pixels) between frames = held still
MIN_BOARD_SHARPNESS = 30.0   # blur check on the board area (Laplacian variance)
GOOD_ERROR_PX = 1.0      # final error must be below this to save calib.npz

# A snapshot counts as "new" if it differs from EVERY earlier snapshot in at least one way:
NEW_POSITION = 0.12      # board centre moved 12% of the image width/height
NEW_SIZE = 0.25          # board looks 25% bigger or smaller (nearer / further)
NEW_TILT = 0.08          # board is tilted differently


def board_outline(corners):
    """The 4 outer corners of the board: top-left, top-right, bottom-right, bottom-left."""
    cols, rows = BOARD
    c = corners.reshape(-1, 2)
    return c[0], c[cols - 1], c[-1], c[(rows - 1) * cols]


def describe_view(corners, w, h):
    """Five numbers that describe WHERE the board is and HOW it is tilted."""
    tl, tr, br, bl = board_outline(corners)
    quad = np.array([tl, tr, br, bl], np.float32)
    centre = quad.mean(axis=0)
    area = cv2.contourArea(quad) / (w * h)                 # how much of the picture it fills
    left, right = np.linalg.norm(bl - tl), np.linalg.norm(br - tr)
    top_edge, bottom_edge = np.linalg.norm(tr - tl), np.linalg.norm(br - bl)
    tilt_lr = (left - right) / (left + right)              # turned left/right
    tilt_ud = (top_edge - bottom_edge) / (top_edge + bottom_edge)   # tipped up/down
    return dict(x=centre[0] / w, y=centre[1] / h, area=area, tilt_lr=tilt_lr, tilt_ud=tilt_ud)


def is_new_view(view, saved_views):
    """True if this view is clearly different from every snapshot we already have."""
    for old in saved_views:
        moved = math_hypot(view["x"] - old["x"], view["y"] - old["y"]) > NEW_POSITION
        resized = abs(view["area"] - old["area"]) / max(old["area"], 1e-6) > NEW_SIZE
        tilted = (abs(view["tilt_lr"] - old["tilt_lr"]) > NEW_TILT or
                  abs(view["tilt_ud"] - old["tilt_ud"]) > NEW_TILT)
        if not (moved or resized or tilted):
            return False          # too similar to this old snapshot
    return True


def math_hypot(a, b):
    return float(np.hypot(a, b))


def board_sharpness(gray, corners):
    """Blur score of just the board area (so a blurry background doesn't matter)."""
    x, y, bw, bh = cv2.boundingRect(corners.reshape(-1, 2).astype(np.float32))
    patch = gray[max(y, 0):y + bh, max(x, 0):x + bw]
    return cv2.Laplacian(patch, cv2.CV_64F).var() if patch.size else 0.0


def missing_areas(saved_views):
    """Which parts of the picture has the board NOT visited yet? (3x3 grid)"""
    names = [["top-left", "top", "top-right"], ["left", "centre", "right"],
             ["bottom-left", "bottom", "bottom-right"]]
    seen = {(min(int(v["y"] * 3), 2), min(int(v["x"] * 3), 2)) for v in saved_views}
    return [names[r][c] for r in range(3) for c in range(3) if (r, c) not in seen]


def fit_lens(board_pts, image_pts, image_size):
    """Fit the normal lens model; if it fits badly (wide lens), also try the rational
    model and keep the better one. Returns (error_px, K, dist, model name)."""
    error, K, dist, _, _ = cv2.calibrateCamera(board_pts, image_pts, image_size, None, None)
    if error <= TRY_WIDE_MODEL_ABOVE_PX:
        return error, K, dist, "normal"
    e2, K2, d2, _, _ = cv2.calibrateCamera(board_pts, image_pts, image_size, None, None,
                                           flags=cv2.CALIB_RATIONAL_MODEL)
    print(f"Normal lens model: {error:.3f} px, rational (wide-angle) model: {e2:.3f} px")
    if e2 < error:
        return e2, K2, d2, "rational"
    return error, K, dist, "normal"


def main():
    parser = argparse.ArgumentParser(description="Automatic camera calibration")
    parser.add_argument("source", nargs="?", default="0",
                        help="camera number, /dev/videoN, or video file")
    parser.add_argument("--no-window", action="store_true", help="don't open a preview window")
    parser.add_argument("--square", type=float, default=SQUARE_M,
                        help="measured size of one square in metres (default %(default)s)")
    args = parser.parse_args()

    is_camera = args.source.isdigit() or args.source.startswith("/dev/video")
    cap = cv2.VideoCapture(int(args.source) if args.source.isdigit() else args.source)
    if is_camera:   # ask the camera for the flying resolution (fixed-mode cameras ignore it)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

    # The real-world 3D corner positions of a flat board (z = 0)
    board_points = np.zeros((BOARD[0] * BOARD[1], 3), np.float32)
    board_points[:, :2] = np.mgrid[0:BOARD[0], 0:BOARD[1]].T.reshape(-1, 2) * args.square

    all_board_points, all_image_points, saved_views = [], [], []
    prev_corners, last_snap, image_size = None, 0.0, None
    start = time.time()
    print(f"Auto-calibration: show the {BOARD[0]}x{BOARD[1]} board from many places. "
          f"Stops after {TARGET_VIEWS} snapshots.")

    while len(all_image_points) < TARGET_VIEWS and time.time() - start < MAX_SECONDS:
        ok, frame = cap.read()
        if not ok:
            break                                    # video ended / camera unplugged
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        image_size = (w, h)

        found, corners = cv2.findChessboardCornersSB(gray, BOARD)
        status = "looking for the board..."
        if found:
            # 1. Held still? (compare with where the corners were in the last frame)
            still = (prev_corners is not None and
                     np.abs(corners - prev_corners).reshape(-1, 2).mean() < STILL_PX)
            # 2. Sharp enough? 3. New place or angle? 4. Not too soon after the last one?
            sharp = board_sharpness(gray, corners) > MIN_BOARD_SHARPNESS
            view = describe_view(corners, w, h)
            new = is_new_view(view, saved_views)
            waited = time.time() - last_snap > MIN_GAP_S

            if still and sharp and new and waited:
                # ---- SNAPSHOT (this is what pressing 'c' used to do) ----
                crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
                all_board_points.append(board_points)
                all_image_points.append(corners)
                saved_views.append(view)
                last_snap = time.time()
                todo = missing_areas(saved_views)
                hint = f"  still to visit: {', '.join(todo)}" if todo else "  all areas covered"
                print(f"Snapshot {len(all_image_points)}/{TARGET_VIEWS}.{hint}")
            status = ("move to a NEW spot or angle" if not new else
                      "hold still..." if not still else
                      "too blurry" if not sharp else "ok")
            prev_corners = corners
        else:
            prev_corners = None

        if not args.no_window:
            if found:
                cv2.drawChessboardCorners(frame, BOARD, corners, found)
            cv2.putText(frame, f"{len(all_image_points)}/{TARGET_VIEWS}  {status}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("auto calibrate", frame)
            cv2.waitKey(1)

    cap.release()
    cv2.destroyAllWindows()

    # ---- FINISH (this is what pressing 'q' used to do) ----
    n = len(all_image_points)
    if n < MIN_VIEWS:
        print(f"Only {n} snapshots (need {MIN_VIEWS}). Move the board around more and run again.")
        return
    error, K, dist, model = fit_lens(all_board_points, all_image_points, image_size)
    print(f"\nUsed {n} snapshots at {image_size[0]}x{image_size[1]} ({model} lens model).")
    if image_size != (WIDTH, HEIGHT):
        print(f"Note: the camera gave {image_size[0]}x{image_size[1]}, not {WIDTH}x{HEIGHT}. "
              f"Fine for a webcam test; on the drone it must be the Justice D's full size.")
    print(f"Average error: {error:.3f} px (below 0.5 = great, below {GOOD_ERROR_PX} = fine)")
    print(f"Focal length: {K[0, 0]:.1f} x {K[1, 1]:.1f} px,  centre: ({K[0, 2]:.1f}, {K[1, 2]:.1f})")
    if error < GOOD_ERROR_PX:
        np.savez("calib.npz", K=K, dist=dist, size=np.array(image_size))
        print("Saved calib.npz")
    else:
        np.savez("calib_rejected.npz", K=K, dist=dist, size=np.array(image_size))
        print("Error too high: saved as calib_rejected.npz instead. Run again with more "
              "varied, steadier views (or tell the team: the lens may need the fisheye model).")


if __name__ == "__main__":
    main()
