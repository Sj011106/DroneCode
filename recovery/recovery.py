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
#   - Stage 2 (bench loop + recovery bonus): the FIRST home picture's fingerprints are kept
#     as well (set_home(origin=True)). find_origin() looks for it; vision_hold.py calls that
#     now and then while the reference is a later picture, and switches back when found.
#     Known limit: a floor of identical tiles can match one tile off. BETAFLIGHT CHANGE: matches must
#     pass Lowe's ratio test (config.RECOVERY_RATIO): a fingerprint that fits two places about
#     equally well (identical tiles) is not used. Lowe, IJCV 2004, section 7.1.
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
    def __init__(self, tracker, every_n=None, min_inliers=None, n_features=None):   # None = config.py
        self.tracker = tracker
        self.every_n = config.RECOVERY_EVERY_N_FRAMES if every_n is None else every_n
        self.min_inliers = config.RECOVERY_MIN_INLIERS if min_inliers is None else min_inliers
        self.orb = cv2.ORB_create(nfeatures=config.RECOVERY_ORB_FEATURES if n_features is None else n_features)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.home_pts = self.home_des = None
        self.origin = None           # (points, fingerprints) of the FIRST home picture
        self.lost_frames = 0
        self.relocks = 0             # how many times we re-found home (for the log / slides)

    def set_home(self, gray, origin=False):
        """Call when HOME is saved: remember its ORB fingerprints.
        origin=True: a brand-new home (not a re-home): also keep it for find_origin()."""
        kp, self.home_des = self.orb.detectAndCompute(gray, None)
        self.home_pts = np.float32([k.pt for k in kp])
        self.lost_frames = 0
        if origin:
            self.origin = (self.home_pts, self.home_des)

    def use_origin(self):
        """The tracker's reference is the first home picture again."""
        self.home_pts, self.home_des = self.origin
        self.lost_frames = 0

    def find_origin(self, gray):
        """First-home -> now matrix if the first home picture is in view at a similar height, else None."""
        if self.origin is None or self.origin[1] is None:
            return None
        M, _ = self.find_home(gray, *self.origin)
        if M is None or not config.REHOME_SCALE_MIN <= float(np.hypot(M[0, 0], M[1, 0])) <= config.REHOME_SCALE_MAX:
            return None                  # other height: the tracker could not hold on to it anyway
        return M

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

    def find_home(self, gray, home_pts=None, home_des=None):
        """Search the whole frame for home. Returns (home->now 2x3 matrix or None, inliers)."""
        if home_des is None:
            home_pts, home_des = self.home_pts, self.home_des
        kp, des = self.orb.detectAndCompute(gray, None)
        if des is None or len(kp) < self.min_inliers:
            return None, 0
        ratio = getattr(config, "RECOVERY_RATIO", 0)
        if ratio:                                    # BETAFLIGHT CHANGE: Lowe ratio test (identical floor tiles)
            if not hasattr(self, "_knn"):
                self._knn = cv2.BFMatcher(cv2.NORM_HAMMING)
            pairs = self._knn.knnMatch(home_des, des, k=2)
            matches = [p[0] for p in pairs if len(p) == 2 and p[0].distance < ratio * p[1].distance]
        else:
            matches = self.matcher.match(home_des, des)
        if len(matches) < self.min_inliers:
            return None, 0
        src = home_pts[[m.queryIdx for m in matches]]
        dst = np.float32([kp[m.trainIdx].pt for m in matches])
        M, inl = cv2.estimateAffinePartial2D(src, dst, method=cv2.RANSAC, ransacReprojThreshold=3.0)
        n = int(inl.sum()) if inl is not None else 0
        return (M, n) if M is not None and n >= self.min_inliers else (None, n)
