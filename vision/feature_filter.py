# =====================================================================================
# vision/feature_filter.py   (OWNER: Saumya - left for her to write)    BRIEF ITEM 1
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Not every corner on the ground is a good landmark. A leaf in the wind, a moving
#   shadow, sun glare on water, or the drone's own legs in the picture all MOVE
#   differently from the ground and can fool the tracker. This filter classifies the
#   corners and drops the bad ones before RANSAC, so the tracker sees cleaner points.
#
# SUGGESTED INTERFACE (so it plugs in like rolling_shutter.py, without touching tracker.py)
#   def keep_mask(gray, pts) -> np.ndarray[bool]
#       gray : current grey image;  pts : (N, 2) raw pixel corners
#       True = keep the point. Called on home corners in set_reference (e.g. a mask for
#       the drone's legs / props in view) and/or on tracked points before RANSAC.
#
# IDEAS + REFERENCES
#   - Static mask of the airframe in view (legs, props): simplest, biggest win.
#   - Glare / saturation: drop points on pixels near 255 (specular water, metal).
#   - Repeated texture (tiles, crops in rows): ambiguity test, e.g. ratio of best to
#     second-best match. Lowe, "Distinctive Image Features from Scale-Invariant
#     Keypoints", IJCV 2004 (ratio test).
#   - Moving objects: points whose residual after RANSAC stays high over several frames.
#     Torr & Zisserman, "MLESAC", CVIU 2000.
# =====================================================================================
