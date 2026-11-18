# =====================================================================================
# tools/replay.py  -  record a flight, replay it through the SAME code later
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Flying is slow, expensive and a bit risky. So during a flight we RECORD everything
#   the vision program saw: every camera picture with its exact time, every tilt
#   reading from the autopilot, every height reading, and the moment home was saved.
#   Back on the ground, vision_hold.py can REPLAY that recording through exactly the
#   same code, as fast as the computer allows, as often as we like:
#     - try new settings without flying again,
#     - produce the "with vs without" slides from ONE flight
#       (e.g. --set USE_TILT=False, --set USE_ROLLING_SHUTTER=False),
#     - re-run after recalibrating the camera timing.
#
# HOW IT WORKS (technical)
#   Recording (vision_hold.py --record DIR, or just --record = auto folder):
#     DIR/video.avi      GREY frames at the processing size (config.PROCESS_WIDTH), codec
#                        config.RECORD_CODEC: FFV1 lossless (replay sees identical pixels),
#                        or MJPG (~3x smaller, lossy). Size: see config.py / docs/hardware.md.
#     DIR/frames.csv     index, t_capture, home_fired (trigger edge), height_m (raw link.height),
#                        att_upto (newest IMU sample time this frame was processed with)
#     DIR/attitude.csv   t, roll, pitch, yaw - every ATTITUDE sample, stamped on arrival
#     DIR/meta.csv       the camera's ORIGINAL frame size (replay scales the lens K from it)
#   The APB has only 4 GB eMMC: the auto folder goes to /sdcard when a card is mounted, and
#   recording stops (flight continues) when free space < config.RECORD_MIN_FREE_MB.
#   Replay (vision_hold.py --replay DIR):
#     ReplayLink is Saumya's DroneLink with the radio removed: attitude_at() and
#     send_odometry() are her real methods; the attitude buffer is filled with the samples
#     that had ARRIVED by each frame's t_capture, so rolling-shutter and tilt see the same
#     data as in flight. The program's clock is the recorded t_capture, so timers
#     (failsafe, health) behave as in flight. No real-time pacing.
#   Exact: vision_hold takes ONE attitude snapshot per frame and the recorder stores the
#   newest sample time of that snapshot (frames.csv att_upto), so replay hands every frame
#   exactly the samples it had in flight. (Older recordings without att_upto: samples up
#   to t_capture.)
#
# WHERE IT PLUGS IN
#   vision_hold.py: --record DIR, --replay DIR, --set NAME=VALUE, --log PATH.
#   Proof: tests/test_replay.py.
#
# REFERENCES
#   - FFV1 lossless video codec, IETF RFC 9043 (2021).
#   - The same record/replay idea as ROS bags: https://wiki.ros.org/rosbag
#   - ArduPilot's own EKF replay tool (same idea, autopilot side):
#     https://ardupilot.org/dev/docs/testing-with-replay.html
# =====================================================================================

import csv
import os
import shutil
import threading
import time
from collections import deque

import cv2

import config
from link.drone_link import DroneLink


def auto_folder():
    """/sdcard/rec_<time> when the APB's SD card is mounted, else logs/rec_<time>."""
    base = "/sdcard" if os.path.ismount("/sdcard") else "logs"
    return os.path.join(base, time.strftime("rec_%Y%m%d_%H%M%S"))


class Recorder:
    def __init__(self, folder, fps=30):
        os.makedirs(folder, exist_ok=True)
        self.folder, self.fps = folder, fps
        self.video = None
        self.stopped = False                                # True once the disk got too full
        self.frames_file = open(os.path.join(folder, "frames.csv"), "w", newline="")
        self.att_file = open(os.path.join(folder, "attitude.csv"), "w", newline="")
        self.frames = csv.writer(self.frames_file)
        self.att = csv.writer(self.att_file)
        self.frames.writerow(["index", "t_capture", "home_fired", "height_m", "att_upto"])
        self.att.writerow(["t", "roll", "pitch", "yaw"])
        self.n = 0
        self.last_att_t = float("-inf")

    def _open_video(self, frame):
        h, w = frame.shape[:2]
        for fourcc in dict.fromkeys((config.RECORD_CODEC, "FFV1", "MJPG")):   # wanted codec first
            v = cv2.VideoWriter(os.path.join(self.folder, "video.avi"),
                                cv2.VideoWriter_fourcc(*fourcc), self.fps, (w, h), isColor=False)
            if v.isOpened():
                if fourcc != config.RECORD_CODEC:
                    print(f"Recorder: {config.RECORD_CODEC} not available, using {fourcc}")
                return v
        raise RuntimeError("Recorder: could not open a video writer")

    def add(self, frame, t_capture, home_fired, height, attitudes, orig_size=None):
        """Call once per camera frame. frame = grey (or BGR) picture to store; orig_size =
        (w, h) the camera really delivered (for K scaling in replay); attitudes = the
        attitude snapshot THIS frame was processed with (new samples are copied, and the
        newest timestamp is stored so replay gives the frame exactly the same samples)."""
        if self.stopped:
            return
        if frame.ndim == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.n % 30 == 0 and shutil.disk_usage(self.folder).free < config.RECORD_MIN_FREE_MB * 1e6:
            print(f"Recorder: less than {config.RECORD_MIN_FREE_MB} MB free - recording STOPPED "
                  f"(flight continues) after {self.n} frames")
            self.stopped = True
            return
        if self.video is None:
            self.video = self._open_video(frame)
            w, h = orig_size or (frame.shape[1], frame.shape[0])
            with open(os.path.join(self.folder, "meta.csv"), "w", newline="") as f:
                csv.writer(f).writerows([["orig_width", "orig_height"], [w, h]])
        self.video.write(frame)
        attitudes = list(attitudes)
        self.frames.writerow([self.n, repr(float(t_capture)), int(bool(home_fired)),
                              "" if height is None else repr(float(height)),   # float(): plain digits
                              repr(float(attitudes[-1][0])) if attitudes else ""])
        for s in attitudes:
            if s[0] > self.last_att_t:
                self.att.writerow([repr(float(v)) for v in s])
                self.last_att_t = s[0]
        self.n += 1

    def close(self):
        if self.video is not None:
            self.video.release()
        self.frames_file.close()
        self.att_file.close()


class _ReplayConn:
    """Stands in for the pymavlink connection: remembers what we would have sent."""
    def __init__(self):
        self.flightmode, self.address = "REPLAY", "replay"
        self.mav = self
        self.target_system = self.target_component = 1
        self.odometry, self.modes = [], []

    def odometry_send(self, *args):
        self.odometry.append(args)

    def set_mode(self, mode):
        self.modes.append(mode)

    def __getattr__(self, name):                           # any other *_send: ignore
        if name.endswith("_send"):
            return lambda *a, **k: None
        raise AttributeError(name)


class ReplayLink(DroneLink):
    """Saumya's DroneLink without the radio: same attitude_at() and send_odometry()."""

    def __init__(self, folder):                            # no super(): no connection to open
        self.conn = _ReplayConn()
        self.send_lock = threading.Lock()
        self.attitudes = deque(maxlen=200)                 # same size as DroneLink's
        self.rangefinder = self.local_z = None
        self.rc, self.params = {}, {}
        self.origin_set, self.running = True, True
        self.acked_source = self.pending_source = None
        with open(os.path.join(folder, "attitude.csv")) as f:
            self._att = [tuple(float(x) for x in row) for row in list(csv.reader(f))[1:]]
        self._next_att = 0
        self.heights = {}

    def start(self):
        pass

    def check_params(self):
        pass

    def command(self, *args):
        pass

    def send_heartbeat(self):
        pass

    def set_origin(self):
        pass

    def set_source(self, n):
        self.pending_source = self.acked_source = n

    def mode_age(self):
        return 0.0                                         # replay: the mode is always fresh

    def feed_until(self, t):
        """Deliver the attitude samples that had arrived by time t."""
        while self._next_att < len(self._att) and self._att[self._next_att][0] <= t:
            self.attitudes.append(self._att[self._next_att])
            self._next_att += 1

    def height(self, now):
        return self.heights.get(now)


class Replay:
    """Everything vision_hold.py needs to run on a recording: .link, .cap, .clock, .trigger."""

    def __init__(self, folder):
        self.link = ReplayLink(folder)
        with open(os.path.join(folder, "frames.csv")) as f:
            rows = list(csv.reader(f))[1:]
        # (t_capture, home_fired, height, newest attitude time the frame saw; old recordings: t)
        self.frames = [(float(r[1]), bool(int(r[2])), float(r[3]) if r[3] else None,
                        float(r[4]) if len(r) > 4 and r[4] else float(r[1])) for r in rows]
        self.video = cv2.VideoCapture(os.path.join(folder, "video.avi"))
        meta = os.path.join(folder, "meta.csv")             # older recordings: no meta = full size
        self.orig_width = None
        if os.path.exists(meta):
            with open(meta) as f:
                self.orig_width = int(list(csv.reader(f))[1][0])
        self.i = -1
        self.t = self.frames[0][0] if self.frames else 0.0
        self.cap, self.trigger = self, self               # one object plays all three roles

    # ----- camera -----
    def isOpened(self):
        return self.video.isOpened()

    def get(self, prop):
        return self.video.get(prop)

    def set(self, prop, value):
        return False

    def read(self):
        ok, frame = self.video.read()
        if not ok or self.i + 1 >= len(self.frames):
            return False, None
        self.i += 1
        self.t, _, height, att_upto = self.frames[self.i]
        self.link.heights[self.t] = height
        self.link.feed_until(att_upto)
        return True, frame

    def grab(self):
        return False

    def release(self):
        self.video.release()

    # ----- clock + home trigger -----
    def clock(self):
        return self.t

    def fired(self, link):
        return 0 <= self.i < len(self.frames) and self.frames[self.i][1]
