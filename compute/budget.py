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
#   - FrameBudget: decides from the frame's AGE after processing (now - when read() gave
#     it to us) plus whether read() had to WAIT. Every camera period of age = one newer
#     frame arrived; if read() returned instantly, the queue was already backed up (assume
#     full). The driver queue holds at most queue_len frames, so grab() away (waiting - 1)
#     and the next read() returns the newest. With a 1-frame queue that is always 0: the
#     one waiting frame IS the newest; dropping it would only make us wait for the next.
#     Simulated (tests/test_budget.py, 60 fps camera, 20 ms work, 4-frame queue): data age
#     76 ms -> 22 ms; 1-frame queue: identical to never dropping.
#     NOT from loop time: the loop includes the wait inside read(), so loop time is always
#     about one period and would throw away every other frame (measured: 30 -> 15 fps).
#     The period is the CAMERA's (Justice D: 60 fps), read from the driver.
#   Only for live cameras: a video file has no queue, it just plays slower.
#
# WHERE IT PLUGS IN
#   vision_hold.py: make_low_latency(cap) after opening a camera, and
#   budget.frames_to_drop(now - t_capture, read wait) at the end of each frame. Proof: tests/test_budget.py.
#
# REFERENCES
#   - OpenCV VideoCapture properties: https://docs.opencv.org/4.x/d4/d15/group__videoio__flags__base.html
#   - Linux V4L2 streaming I/O (driver buffer queue): https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/mmap.html
# =====================================================================================

import sys
import threading
import time

import cv2
import numpy as np

import config


def open_camera(source, raw=False):
    """STAGE 2: open a camera AND ask for the Justice D's real format. Without asking, the APB's /dev/video2
    gives 640x480 (measured 21 Nov 2026). source: "2", "/dev/video2" or a video file name.
    Only a camera on Linux is asked for a format; a laptop webcam or a video file is opened as before."""
    source = str(source)
    is_cam = source.isdigit() or source.startswith("/dev/video")
    if not (is_cam and sys.platform.startswith("linux")):
        return cv2.VideoCapture(int(source) if source.isdigit() else source)
    cap = cv2.VideoCapture(int(source) if source.isdigit() else source, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config.CAMERA_FOURCC))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_SIZE[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_SIZE[1])
    cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)
    if raw:                                    # hand us the camera's own YUYV bytes: see to_gray()
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    return cap


def to_gray(frame):
    """A grey picture from whatever the camera handed over:
    colour (H, W, 3) -> converted; raw YUYV (H, W, 2) -> its first channel IS the grey picture (free);
    raw YUYV as one flat row -> reshaped first; already grey -> unchanged."""
    w, h = config.CAMERA_SIZE
    if frame.ndim == 3 and frame.shape[2] == 3:
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if frame.ndim == 3 and frame.shape[2] == 2:
        return np.ascontiguousarray(frame[:, :, 0])
    if frame.size == w * h * 2 and frame.shape[:2] != (h, w):
        return np.ascontiguousarray(frame.reshape(h, w, 2)[:, :, 0])
    return frame if frame.ndim == 2 else frame[:, :, 0]


def camera_fps(cap, default=None):
    """The frame rate the driver reports, or `default` when it reports nothing useful (the APB says -1)."""
    fps = cap.get(cv2.CAP_PROP_FPS)
    return fps if fps and fps > 0 else default


class LatestFrame:
    """STAGE 2: a camera whose read() never waits for the driver. A side thread grabs all the time; read()
    hands over the newest frame not yet seen (waiting only if there is none yet) and `t` is the moment that
    frame arrived. Everything else (get, set, release...) goes to the real camera."""

    def __init__(self, cap, clock=time.time):
        self.cap, self.clock, self.t = cap, clock, 0.0
        self._new, self._item, self._alive = threading.Event(), (False, None, 0.0), True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self._alive:
            ok, frame = self.cap.read()
            self._item = (ok, frame, self.clock())
            self._new.set()
            if not ok:
                time.sleep(0.02)

    def read(self):
        if not self._new.wait(1.0):                  # camera silent for a second: same as a failed read
            self.t = self.clock()
            return False, None
        self._new.clear()
        ok, frame, self.t = self._item
        return ok, frame

    def release(self):
        self._alive = False
        time.sleep(0.05)
        self.cap.release()

    def __getattr__(self, name):
        return getattr(self.cap, name)


def make_low_latency(cap):
    """Ask the driver for a 1-frame queue. Returns True if the backend accepted it."""
    return bool(cap.set(cv2.CAP_PROP_BUFFERSIZE, 1))


class FrameBudget:
    def __init__(self, camera_fps=None, queue_len=4):
        self.period = 1.0 / (camera_fps or config.TARGET_FPS)   # the camera's frame period
        self.queue_len = queue_len    # 1 if make_low_latency() worked, else the driver's (V4L2 ~4)
        self.dropped = 0              # total frames thrown away (for the log / slides)

    def frames_to_drop(self, frame_age_s, read_wait_s):
        """Stale frames to grab() away so the next read() returns the newest one.
        frame_age_s = now - when read() returned; read_wait_s = how long read() blocked."""
        backed_up = self.queue_len if read_wait_s < self.period / 4 else 0
        waiting = min(int(frame_age_s / self.period) + backed_up, self.queue_len)
        n = max(0, waiting - 1)
        self.dropped += n
        return n
