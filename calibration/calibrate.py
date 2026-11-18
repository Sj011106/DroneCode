# calibration/calibrate.py
# Step 2: measure your camera's lens so pixels can be turned into real distances.
#
# What you need: a printed checkerboard (search "OpenCV checkerboard 9x6 pdf"),
# taped flat on a book or cardboard.
#
# How to run (from the repo root, so calib.npz lands there):  python -m calibration.calibrate
#   python -m calibration.calibrate 2            -> camera 2 (or /dev/video2, or a video file)
# Keys:  c = capture this view (do ~15 from different angles/distances)
#        q = finish and calculate

# -------------------------------------------------------------------------------------
# IN PLAIN WORDS (for non-technical readers)
#   Every lens bends the picture a little and has its own "zoom" (focal length). Showing
#   it a checkerboard of known size from many angles measures both, so pixels can be
#   turned into real distances. Result: calib.npz (K = lens matrix, dist = lens bending).
# REFERENCES
#   - Zhang, "A Flexible New Technique for Camera Calibration", IEEE TPAMI 2000.
#   - OpenCV tutorial: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
# -------------------------------------------------------------------------------------

import sys

import cv2
import numpy as np

BOARD = (8, 5)        # INNER corners of the checkerboard (columns, rows)
SQUARE_M = 0.025      # size of one square in metres (measure your printout!)

# The real-world 3D positions of the corners (flat board, so z = 0)
board_points = np.zeros((BOARD[0] * BOARD[1], 3), np.float32)
board_points[:, :2] = np.mgrid[0:BOARD[0], 0:BOARD[1]].T.reshape(-1, 2) * SQUARE_M

all_board_points = []   # 3D points, one set per captured photo
all_image_points = []   # where those corners appeared in each photo (pixels)

source = sys.argv[1] if len(sys.argv) > 1 else "0"
cap = cv2.VideoCapture(int(source) if source.isdigit() else source)
image_size = None

while True:
    ok, frame = cap.read()
    if not ok:
        break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    image_size = gray.shape[::-1]   # (width, height)

    # Try to find the checkerboard corners in this frame
    found, corners = cv2.findChessboardCornersSB(gray, BOARD)
    if found:
        cv2.drawChessboardCorners(frame, BOARD, corners, found)

    cv2.putText(frame, f"captured: {len(all_image_points)}  (c=capture, q=done)",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.imshow("calibrate", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("c") and found:
        # Refine corner positions to sub-pixel accuracy, then store them
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        all_board_points.append(board_points)
        all_image_points.append(corners)
        print(f"Captured view {len(all_image_points)}")
    elif key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()

if len(all_image_points) < 8:
    print("Need at least 8 good views. Run again.")
else:
    # The actual calibration: finds the camera matrix K and lens distortion
    error, K, dist, _, _ = cv2.calibrateCamera(all_board_points, all_image_points,
                                               image_size, None, None)
    print("Average error (pixels, smaller is better, aim < 1):", round(error, 3))
    print("Camera matrix K:\n", K)
    print("Distortion:", dist.ravel())
    # Save the image size too: vision_hold.py / tracker.load_calibration use it to scale K
    # to the size we track at, and to refuse a lens file from a different camera.
    np.savez("calib.npz", K=K, dist=dist, size=np.array(image_size))
    print(f"Saved to calib.npz ({image_size[0]}x{image_size[1]})")