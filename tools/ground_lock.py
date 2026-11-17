# tools/ground_lock.py  (old ORB version, kept for reference; vision/tracker.py replaced it)
# Step 3: the "ground lock" — measure how far we've drifted from the hold point.
#
# How to run:
#   python -m tools.ground_lock            -> uses your webcam (point it at the floor/table)
#   python -m tools.ground_lock video.mp4  -> uses a video you filmed with your phone
#
# Keys:  r = save reference (start the hold)   q = quit

# -------------------------------------------------------------------------------------
# IN PLAIN WORDS (for non-technical readers)
#   The team's FIRST hold-point measurer (ORB feature matching against a saved picture).
#   vision/tracker.py replaced it for flying; its matching idea lives on in recovery.py.
# REFERENCES
#   - Rublee et al., "ORB: an efficient alternative to SIFT or SURF", ICCV 2011.
# -------------------------------------------------------------------------------------

import sys
import os
import cv2
import numpy as np

# ---------- Settings you can change ----------
HEIGHT_M = 0.5          # how high the camera is above the floor, in metres (measure it!)
MIN_INLIERS = 25        # fewer good matches than this = "unhealthy"
MIN_SHARPNESS = 50.0    # lower than this = image is too blurry
DEFAULT_FOCAL_PX = 600  # rough guess, only used if you haven't calibrated yet

# ---------- Tools we create once ----------
orb = cv2.ORB_create(nfeatures=1000)                       # finds up to 1000 corner-like points
matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)  # matches points between 2 images


def load_focal_length():
    """Read the focal length (in pixels) from calib.npz if calibrate.py was run."""
    if os.path.exists("calib.npz"):
        data = np.load("calib.npz")
        K = data["K"]                      # camera matrix from calibration
        return (K[0, 0] + K[1, 1]) / 2     # average of fx and fy
    return DEFAULT_FOCAL_PX


def sharpness(gray):
    """Blur score: variance of the Laplacian. Low number = blurry image."""
    return cv2.Laplacian(gray, cv2.CV_64F).var()


def compute_offset(ref_kp, ref_des, gray, height_m, focal_px):
    """
    Compare the current frame to the saved reference frame.
    Returns (dx_m, dy_m, inliers): how far the CAMERA moved from the hold point,
    in metres, plus how many good matches we found.
    Returns (None, None, inliers) if it couldn't work it out.
    """
    kp, des = orb.detectAndCompute(gray, None)
    if des is None or ref_des is None or len(kp) < 10:
        return None, None, 0

    # 1. Match points between the reference image and the current image
    matches = matcher.match(ref_des, des)
    if len(matches) < 10:
        return None, None, len(matches)

    src = np.float32([ref_kp[m.queryIdx].pt for m in matches])  # points in reference
    dst = np.float32([kp[m.trainIdx].pt for m in matches])      # same points now

    # 2. Homography = how the ground moved in the image. RANSAC ignores bad matches.
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
    if H is None:
        return None, None, 0
    inliers = int(mask.sum())

    # 3. Where did the hold point go? The hold point = the ground that was in the
    #    CENTRE of the reference image. Push that centre through H.
    h, w = gray.shape
    centre = np.array([[[w / 2, h / 2]]], dtype=np.float32)
    moved = cv2.perspectiveTransform(centre, H)[0][0]
    shift_px_x = moved[0] - w / 2
    shift_px_y = moved[1] - h / 2

    # 4. If the ground moved LEFT in the image, the camera moved RIGHT -> flip the sign.
    #    Pixels -> metres: metres = pixels * height / focal_length
    dx_m = -shift_px_x * height_m / focal_px
    dy_m = -shift_px_y * height_m / focal_px
    return dx_m, dy_m, inliers


def main():
    source = sys.argv[1] if len(sys.argv) > 1 else 0   # webcam 0 or a video file
    cap = cv2.VideoCapture(source)
    focal_px = load_focal_length()
    print(f"Using focal length {focal_px:.0f} px, height {HEIGHT_M} m")

    ref_kp, ref_des = None, None   # nothing saved yet

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        sharp = sharpness(gray)

        if ref_des is None:
            text, colour = "Press r to save the hold point", (255, 255, 255)
        else:
            dx, dy, inliers = compute_offset(ref_kp, ref_des, gray, HEIGHT_M, focal_px)

            # ---- Health check ----
            healthy = dx is not None and inliers >= MIN_INLIERS and sharp >= MIN_SHARPNESS
            if healthy:
                dist_cm = 100 * (dx**2 + dy**2) ** 0.5
                text = f"HEALTHY  dx={dx*100:+.1f}cm dy={dy*100:+.1f}cm  drift={dist_cm:.1f}cm  inliers={inliers}"
                colour = (0, 255, 0)
                # Arrow from centre showing which way to fly to get back to the hold point
                h, w = gray.shape
                c = (w // 2, h // 2)
                scale = focal_px / HEIGHT_M          # metres back to pixels for drawing
                tip = (int(c[0] - dx * scale), int(c[1] - dy * scale))
                cv2.arrowedLine(frame, c, tip, (0, 255, 0), 3)
            else:
                text = f"UNHEALTHY  inliers={inliers}  sharpness={sharp:.0f}"
                colour = (0, 0, 255)

        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
        cv2.imshow("ground lock", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("r"):                       # save reference = start the hold
            ref_kp, ref_des = orb.detectAndCompute(gray, None)
            print(f"Reference saved with {len(ref_kp)} points")
        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()