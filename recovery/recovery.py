# =====================================================================================
# recovery/recovery.py  -  re-find home after vision is LOST       BRIEF ITEM 3
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Saumya's tracker follows the ground frame by frame: it predicts where home is now
#   from where it was a moment ago. That works great until the picture disappears (a
#   blackout, a dust cloud, a big gust). While blind, the drone keeps drifting, so when
#   the picture comes back home is somewhere else and the frame-by-frame tracker can't
#   find it again. health.py correctly says LOST... forever.
#   Recovery fixes this: it remembers what home LOOKED like (a set of distinctive
#   fingerprints), searches the whole new picture for them, and if enough match in a
#   consistent way, it tells the tracker "home is HERE now". The normal tracker then
#   takes over and health.py rebuilds trust over a few good frames.
#
# HOW IT WORKS (technical)
#   - At home: ORB keypoints + binary descriptors of the home image.
#   - While LOST (and the current frame is actually failing), every Nth frame:
#     ORB on the current frame -> brute-force Hamming matching with cross-check ->
#     RANSAC similarity (cv2.estimateAffinePartial2D). Needs RECOVERY_MIN_INLIERS.
#   - Success: tracker.relock(M) sets the home -> now prediction (raw pixel coordinates,
#     the same frame Saumya's pre-alignment warp uses). LK + RANSAC then refine it.
#   ORB is global (no motion prior), so it can re-find home after any drift as long as
#   part of the home area is still in view. It's slower than LK, so it runs only while LOST.
#
# WHERE IT PLUGS IN
#   vision_hold.py, after health.check(). Uses tracker.relock(), which Saumya already
#   wrote for exactly this. Proof: tests/test_recovery.py (2 s and 10 s blackouts).
#
# REFERENCES
#   - Rublee, Rabaud, Konolige, Bradski, "ORB: an efficient alternative to SIFT or SURF",
#     ICCV 2011.
#   - Williams, Klein, Reid, "Real-Time SLAM Relocalisation", ICCV 2007.
#   - Mur-Artal, Montiel, Tardos, "ORB-SLAM: a Versatile and Accurate Monocular SLAM
#     System", IEEE T-RO 2015 (ORB-based relocalisation after tracking loss).
#   - Fischler & Bolles, "Random Sample Consensus", Comm. ACM 1981 (RANSAC).
# =====================================================================================

import cv2
import numpy as np

import config


class Recovery:
    def __init__(self, tracker, every_n=config.RECOVERY_EVERY_N_FRAMES,
                 min_inliers=config.RECOVERY_MIN_INLIERS, n_features=config.RECOVERY_ORB_FEATURES):
        self.tracker = tracker
        self.every_n, self.min_inliers = every_n, min_inliers
        self.orb = cv2.ORB_create(nfeatures=n_features)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.home_pts = self.home_des = None
        self.lost_frames = 0
        self.relocks = 0             # how many times we re-found home (for the log / slides)

    def set_home(self, gray):
        """Call when HOME is saved: remember its ORB fingerprints."""
        kp, self.home_des = self.orb.detectAndCompute(gray, None)
        self.home_pts = np.float32([k.pt for k in kp])
        self.lost_frames = 0

    def update(self, gray, failing):
        """Call every frame. failing = health says LOST and this frame is bad.
        Returns True on the frame where home was re-found."""
        if not failing:
            self.lost_frames = 0
            return False
        self.lost_frames += 1
        if self.home_des is None or self.lost_frames % self.every_n:
            return False
        M, _ = self.find_home(gray)
        if M is None:
            return False
        self.tracker.relock(M)
        self.relocks += 1
        return True

    def find_home(self, gray):
        """Search the whole frame for home. Returns (home->now 2x3 matrix or None, inliers)."""
        kp, des = self.orb.detectAndCompute(gray, None)
        if des is None or len(kp) < self.min_inliers:
            return None, 0
        matches = self.matcher.match(self.home_des, des)
        if len(matches) < self.min_inliers:
            return None, 0
        src = self.home_pts[[m.queryIdx for m in matches]]
        dst = np.float32([kp[m.trainIdx].pt for m in matches])
        M, inl = cv2.estimateAffinePartial2D(src, dst, method=cv2.RANSAC, ransacReprojThreshold=3.0)
        n = int(inl.sum()) if inl is not None else 0
        return (M, n) if M is not None and n >= self.min_inliers else (None, n)
