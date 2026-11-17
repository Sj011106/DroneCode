# =====================================================================================
# compute/budget.py  -  always work on the NEWEST picture            BRIEF ITEM 2
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Cameras keep a small queue of pictures. If one frame takes us a bit too long, the
#   next picture we read is already old, and the delay never goes away: the autopilot
#   then corrects for where the drone WAS, not where it IS, and starts to oscillate.
#   Two fixes, cheapest first:
#     1. Ask the camera driver to keep only 1 picture in its queue.
#     2. If we fell behind anyway, throw away the stale pictures and read the fresh one.
#   Old position data is worse than none, so dropping a frame is the right call.
#
# HOW IT WORKS (technical)
#   - cv2.CAP_PROP_BUFFERSIZE = 1 (honoured by V4L2 on Linux; some backends ignore it).
#   - FrameBudget: if the last loop took k frame periods, call cap.grab() k-1 times
#     (grab() fetches without decoding, so it's cheap) before the next read().
#   Only for live cameras: a video file has no queue, it just plays slower.
#
# WHERE IT PLUGS IN
#   vision_hold.py: make_low_latency(cap) after opening a camera, and
#   budget.frames_to_drop(loop seconds) at the end of each frame.
#
# REFERENCES
#   - OpenCV VideoCapture properties: https://docs.opencv.org/4.x/d4/d15/group__videoio__flags__base.html
#   - Linux V4L2 streaming I/O (driver buffer queue): https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/mmap.html
# =====================================================================================

import cv2

import config


def make_low_latency(cap):
    """Ask the driver for a 1-frame queue. Returns True if the backend accepted it."""
    return bool(cap.set(cv2.CAP_PROP_BUFFERSIZE, 1))


class FrameBudget:
    def __init__(self, fps=config.TARGET_FPS, max_drop=3):
        self.period = 1.0 / fps
        self.max_drop = max_drop
        self.dropped = 0              # total frames thrown away (for the log / slides)

    def frames_to_drop(self, loop_seconds):
        """How many queued frames are stale after a loop that took loop_seconds."""
        n = min(int(loop_seconds / self.period), self.max_drop)
        self.dropped += n
        return n
