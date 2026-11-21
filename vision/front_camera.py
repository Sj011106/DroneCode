# =====================================================================================
# vision/front_camera.py  -  the forward camera (Justice A) on /dev/video3      STAGE 2
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   The aircraft has two cameras. The Justice D looks down and does the position hold.
#   The Justice A looks forward. It is an ANALOGUE (PAL) camera: its signal goes through
#   an ADV7282 video decoder chip on the APB, which turns it into /dev/video3
#   (Stage 2 package section 02: 720 x 576, about 50 fps, 1-3 % partial frames).
#   Our down-camera code cannot simply be pointed at it, for three reasons:
#     1. it asks for 1920 x 1080; the decoder can only give 720 x 576,
#     2. the decoder sends UYVY, not YUYV: the grey value is the SECOND byte of each
#        pair, not the first (taking the first gives a washed-out colour channel),
#     3. PAL is interlaced: every picture is two half-pictures (even rows, odd rows)
#        taken 20 ms apart, so anything moving looks "combed". Using one half-picture
#        (a "field") removes that.
#   This file reads it the right way, on its own thread, and throws away pictures that
#   are blank (no signal: the decoder then sends a flat blue/black screen) or only
#   half-filled (the "partial frames" the package warns about).
#
#   The front camera is OPTIONAL in Stage 2 and is NOT used by the position hold. Use it
#   for the operator's view (tools/front_cam.py --serve), for evidence recordings, and
#   later perhaps for a heading check (far-away things only move when the drone turns).
#   Keep it out of the control loop and at low priority (deploy/front_view.service).
#
# INTERFACE
#   cam = FrontCamera("/dev/video3")      opens it and starts the reading thread
#   cam.latest() -> (field, frame, t, n)  newest good picture or None; never waits
#                                         field = one half-picture (no combing, 288 rows)
#                                         frame = the full 576-row picture
#                                         t = capture time (time.monotonic seconds)
#                                         n = picture number (changes when a new one came)
#   cam.good / cam.bad / cam.no_signal    counters
#   cam.close()
#   gray_from_raw(), frame_problem(), comb_score()   pure helpers (tests/test_cameras.py)
#
# TEST ON THE APB:   python -m tools.front_cam            (see docs/CAMERAS.md)
#
# REFERENCES
#   - Stage 2 package Rev 2.0 section 02 (cameras, /dev/video3, partial frames).
#   - Linux adv7180 driver (covers ADV7282): outputs MEDIA_BUS_FMT_UYVY8_2X8.
#   - V4L2 field order / interlacing: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/field-order.html
# =====================================================================================
import threading
import time

import cv2
import numpy as np

DEFAULT_DEVICE = "/dev/video3"
PAL_SIZE = (720, 576)
FIELD_TIME_S = 0.020                 # one PAL field; the middle of a picture is about half of it earlier


def fourcc_name(value):
    v = int(value)
    return "".join(chr((v >> 8 * i) & 0xFF) for i in range(4))


def gray_from_raw(frame, w, h, y_at):
    """Grey picture (h, w) from what OpenCV handed over, or None if it does not fit.
    Raw 4:2:2 (any shape with w*h*2 bytes): every second byte from y_at is the grey value
    (UYVY -> y_at=1, YUYV -> y_at=0). Colour (h, w, 3) -> converted. Grey (h, w) -> as is."""
    if frame is None:
        return None
    if frame.ndim == 2 and frame.shape == (h, w):
        return frame
    if frame.ndim == 3 and frame.shape[2] == 3:
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    buf = frame.reshape(-1)
    if buf.size == w * h * 2:
        return np.ascontiguousarray(buf.reshape(h, w * 2)[:, y_at::2])
    return None


def frame_problem(gray):
    """None = usable. "no_signal" = flat picture (decoder free-run screen, camera unplugged or
    unpowered). "partial" = a band that is flat AND unlike the rest (part of the buffer not filled)."""
    if float(gray.std()) < 3.0:
        return "no_signal"
    mean = float(gray.mean())
    band = max(8, gray.shape[0] // 32)
    for y in range(0, gray.shape[0] - band + 1, band):
        b = gray[y:y + band]
        if float(b.std()) < 1.0 and abs(float(b.mean()) - mean) > 15:
            return "partial"
    return None


def comb_score(gray):
    """Neighbouring rows vs rows two apart. About 1 or less = progressive or still;
    clearly above 1.2 = the two fields disagree (interlace combing): use the field."""
    g = gray.astype(np.float32)
    near = float(np.abs(np.diff(g, axis=0)).mean())
    far = float(np.abs(g[2:] - g[:-2]).mean()) + 1e-6
    return near / far


class FrontCamera:
    def __init__(self, device=DEFAULT_DEVICE, fmt="UYVY", size=PAL_SIZE, buffers=4):
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open {device}: wrong node, or busy (check: fuser -v {device})")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fmt))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)        # raw bytes: the grey value is free
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, buffers)   # the thread drains the queue, so this is never stale
        self.device = device
        self.fmt = fourcc_name(self.cap.get(cv2.CAP_PROP_FOURCC))
        self.w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or size[0]
        self.h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or size[1]
        self.y_at = 1 if self.fmt == "UYVY" else 0
        self.fields_only = self.h <= 300                 # driver already hands over single fields (720 x 288)
        self.good = self.bad = self.no_signal = 0
        self.age_s = None                                # how old the newest picture was when we got it
        self._latest, self._n, self._alive = None, 0, True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._alive:
            ok, raw = self.cap.read()
            t_now = time.monotonic()
            if not ok or raw is None:
                self.bad += 1
                time.sleep(0.01)
                continue
            gray = gray_from_raw(raw, self.w, self.h, self.y_at)
            why = "bad" if gray is None else frame_problem(gray)
            if why == "no_signal":
                self.no_signal += 1
                continue
            if why:
                self.bad += 1
                continue
            t_drv = self.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0   # V4L2 buffer time (end of the picture)
            age = t_now - t_drv
            if 0.0 <= age < 0.5:                         # the driver's stamp is on our clock: use it
                self.age_s, t = age, t_drv
            else:
                t = t_now
            field = gray if self.fields_only else gray[0::2]
            self._n += 1
            self.good += 1
            self._latest = (field, gray, t - FIELD_TIME_S / 2, self._n)

    def latest(self):
        return self._latest

    def close(self):
        self._alive = False
        self._thread.join(0.5)
        self.cap.release()
