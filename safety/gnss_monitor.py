# =====================================================================================
# safety/gnss_monitor.py  -  is the GPS jammed or lying?  (WATCH ONLY: never navigates)
#                                                                     (OWNER: Ayush, task A5)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   The challenge exists because GNSS (satellite navigation: GPS, GLONASS, ...) is
#   jammed (no signal) or spoofed (fake signal that walks the drone away) on the modern
#   battlefield. The M2-AI has a GPS/GLONASS receiver (M2-AI datasheet p.3). We never fly
#   on it in the scored hold (safety/trial_mode.py), but we can still WATCH it, like a
#   pilot glancing at a second instrument, and say when it is not to be trusted:
#     JAMMED           no 3D fix, too few satellites, a poor HDOP (horizontal dilution of
#                      precision: how badly the satellites are spread), or no reports at all
#     SPOOF SUSPECTED  GPS says we moved while vision (cross-checked by the IMU) says we
#                      are holding still, or the GPS position jumps faster than a drone flies
#     OK               none of the above
#     NO DATA          the autopilot sends no GPS reports (receiver off or not fitted)
#   It writes this to logs/flight_<time>_safety.csv and prints a line when it changes.
#   That is evidence for judging area E (reliability) and a talking point: our drone
#   KNOWS when the satellite signal is under attack, and keeps holding anyway.
#
# THE ONE RULE: monitor only. Nothing from here ever reaches the position we send, the
# EKF source, or a flight mode. tests/test_gnss_monitor.py proves the messages sent to
# ArduPilot are identical with this ON and OFF, even while it shouts SPOOF.
#
# HOW IT WORKS (technical)
#   - Data: MAVLink GPS_RAW_INT (fix_type, satellites_visible, eph = HDOP x 100, lat, lon in
#     1e-7 deg). pymavlink keeps the newest copy of every message in link.conn.messages,
#     filled by Saumya's reader thread, so link/drone_link.py needs NO change. A new
#     sample = a new time_usec. At start we ask the autopilot to stream it at
#     GNSS_REQUEST_HZ with MAV_CMD_SET_MESSAGE_INTERVAL (a read request).
#   - Jump test: two fixes further apart than GNSS_JUMP_MIN_M AND faster than
#     GNSS_JUMP_SPEED_MPS (no hovering quadcopter does that) -> SPOOF SUSPECTED.
#   - Drift test: once vision is trusted after a new home, the GPS "anchor" is the MEDIAN
#     of GNSS_ANCHOR_S of fixes (a median ignores single bad fixes) and the vision anchor
#     the median of vision positions over the same time. Then: if |GPS displacement -
#     vision displacement| > max(GNSS_DISAGREE_M, 4 x the receiver's own accuracy estimate
#     (h_acc; HDOP if missing)) for GNSS_DISAGREE_S while vision is trusted -> SPOOF
#     SUSPECTED. Why 10 m / 3 s: honest GPS error drifts slowly (metres over minutes); with
#     5 m / 2 s our 10-minute honest-GPS test raised 3 false alarms, with 10 m / 3 s none.
#     A new home (reset_counter change) re-anchors.
#   - SPOOF stays shown for GNSS_ALARM_HOLD_S after the last evidence (no flicker).
#
# WHERE IT PLUGS IN
#   safety/side_monitors.py calls update() once per frame (after the send).
#   Switch: config.GNSS_MONITOR. Settings: GNSS_* in config.py.
#
# REFERENCES
#   - MAVLink GPS_RAW_INT: https://mavlink.io/en/messages/common.html#GPS_RAW_INT
#   - MAV_CMD_SET_MESSAGE_INTERVAL: https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL
#   - M2-AI Datasheet v1.0 p.3 (GPS / GLONASS); MRM2-10AI manual p.5, p.20.
#   - Grand Final package Stage 1 Rev 1.3 p.2 (no GNSS aiding in the scored hold).
#   - Spoofing detection by cross-checking GNSS against independent sensors: Psiaki &
#     Humphreys, "GNSS Spoofing and Detection", Proc. IEEE 104(6), 2016.
# =====================================================================================

import math
from collections import deque

import numpy as np

import config

MAV_CMD_SET_MESSAGE_INTERVAL = 511
GPS_RAW_INT_ID = 24
OK, JAMMED, SPOOF, NO_DATA = "OK", "JAMMED", "SPOOF SUSPECTED", "NO DATA"
EARTH_R = 6378137.0


class GnssMonitor:
    COLUMNS = ["gnss_state", "gnss_fix", "gnss_sats", "gnss_hdop", "gnss_gps_disp_m",
               "gnss_vision_disp_m", "gnss_reason"]

    def __init__(self, source=None):
        self.source = source                     # object with .messages (pymavlink conn), or None
        self.state, self.reason = NO_DATA, "no GPS report yet"
        self.last_key = None
        self.last_fix = None                     # (t, n, e) of the previous good fix
        self.last_report_t = None
        self.lat0 = self.lon0 = None
        self.fix = self.sats = self.hdop = self.h_acc = None
        self.gps_ne = None
        self.anchor_buf, self.anchor = [], None  # [(gps_n, gps_e, vis_n, vis_e)], (gn, ge, vn, ve)
        self.anchor_start = None
        self.reset_counter = None
        self.disagree_since = None
        self.spoof_until = -math.inf
        self.spoof_reason = ""
        self.gps_disp = self.vis_disp = None
        self.transitions = []                    # (t, state, reason) for the log / tests

    @classmethod
    def attach(cls, link):
        """Read from this link's message cache; ask the autopilot to stream GPS_RAW_INT."""
        conn = getattr(link, "conn", None)
        mon = cls(conn if hasattr(conn, "messages") else None)
        if config.GNSS_REQUEST_HZ and hasattr(link, "command"):
            try:
                link.command(MAV_CMD_SET_MESSAGE_INTERVAL, GPS_RAW_INT_ID, int(1e6 / config.GNSS_REQUEST_HZ))
            except Exception:
                pass
        return mon

    # ----- input -----
    def _latest(self):
        if self.source is None:
            return None
        try:
            return self.source.messages.get("GPS_RAW_INT")
        except Exception:
            return None

    def _to_ne(self, lat, lon):
        if self.lat0 is None:
            self.lat0, self.lon0 = lat, lon
        n = math.radians(lat - self.lat0) * EARTH_R
        e = math.radians(lon - self.lon0) * EARTH_R * math.cos(math.radians(self.lat0))
        return n, e

    def feed(self, t, fix_type, sats, eph, lat_e7, lon_e7, h_acc_mm=None):
        """One GPS_RAW_INT sample (also used directly by the tests)."""
        self.last_report_t = t
        self.fix, self.sats = int(fix_type), int(sats)
        self.hdop = None if eph in (None, 65535) else eph / 100.0
        self.h_acc = h_acc_mm / 1000.0 if h_acc_mm else None      # receiver's own accuracy (m), MAVLink 2
        good = self.fix >= 3 and lat_e7 not in (None, 0)
        self.gps_ne = self._to_ne(lat_e7 / 1e7, lon_e7 / 1e7) if good else None
        if self.gps_ne is None:
            return
        n, e = self.gps_ne
        if self.last_fix is not None:
            dt = t - self.last_fix[0]
            jump = math.hypot(n - self.last_fix[1], e - self.last_fix[2])
            if dt > 0 and jump > config.GNSS_JUMP_MIN_M and jump / dt > config.GNSS_JUMP_SPEED_MPS:
                self._spoof(t, f"GPS position jumped {jump:.0f} m in {dt:.2f} s ({jump / dt:.0f} m/s)")
        self.last_fix = (t, n, e)

    def _spoof(self, t, why):
        self.spoof_until = t + config.GNSS_ALARM_HOLD_S
        self.spoof_reason = why

    # ----- per frame -----
    def update(self, t, north, east, quality, status, reset_counter):
        msg = self._latest()
        if msg is not None:
            key = getattr(msg, "time_usec", None) or getattr(msg, "_timestamp", None)
            if key != self.last_key:
                self.last_key = key
                self.feed(t, msg.fix_type, msg.satellites_visible, msg.eph, msg.lat, msg.lon,
                          getattr(msg, "h_acc", None))
        trusted = quality is not None and quality > 0 and status == "OK" and np.isfinite(north) and np.isfinite(east)
        self._drift_test(t, north, east, trusted, reset_counter)
        self._decide(t)
        return self.row()

    def _drift_test(self, t, north, east, trusted, reset_counter):
        if reset_counter != self.reset_counter:            # new home: vision restarts at 0 -> re-anchor
            self.reset_counter = reset_counter
            self.anchor_buf, self.anchor, self.anchor_start, self.disagree_since = [], None, None, None
        if not trusted or self.gps_ne is None or self._jammed_reason(t):
            self.disagree_since = None
            return
        gn, ge = self.gps_ne
        if self.anchor is None:
            if self.anchor_start is None:
                self.anchor_start = t
            self.anchor_buf.append((gn, ge, north, east))
            if t - self.anchor_start >= config.GNSS_ANCHOR_S and len(self.anchor_buf) >= 5:
                self.anchor = tuple(np.median(np.array(self.anchor_buf), axis=0))
            return
        a = self.anchor
        g = np.array([gn - a[0], ge - a[1]])
        v = np.array([north - a[2], east - a[3]])
        self.gps_disp, self.vis_disp = float(np.hypot(*g)), float(np.hypot(*v))
        gap = float(np.hypot(*(g - v)))
        own = self.h_acc if self.h_acc else (self.hdop or 1.0)    # receiver's accuracy, else HDOP as metres
        limit = max(config.GNSS_DISAGREE_M, 4 * own)
        if gap > limit:
            if self.disagree_since is None:
                self.disagree_since = t
            if t - self.disagree_since >= config.GNSS_DISAGREE_S:
                self._spoof(t, f"GPS says {self.gps_disp:.1f} m from the hold point, vision says "
                               f"{self.vis_disp:.1f} m (gap {gap:.1f} m > {limit:.1f} m for "
                               f"{t - self.disagree_since:.1f} s)")
        else:
            self.disagree_since = None

    def _jammed_reason(self, t):
        if self.last_report_t is not None and t - self.last_report_t > config.GNSS_TIMEOUT_S:
            return f"no GPS report for {t - self.last_report_t:.1f} s"
        if self.fix is None:
            return None
        why = []
        if self.fix < 3:
            why.append(f"no 3D fix (fix type {self.fix})")
        if self.sats < config.GNSS_MIN_SATS:
            why.append(f"{self.sats} satellites < {config.GNSS_MIN_SATS}")
        if self.hdop is not None and self.hdop > config.GNSS_MAX_HDOP:
            why.append(f"HDOP {self.hdop:.1f} > {config.GNSS_MAX_HDOP:g}")
        return ", ".join(why) or None

    def _decide(self, t):
        jam = self._jammed_reason(t)
        if t < self.spoof_until:
            state, reason = SPOOF, self.spoof_reason
        elif jam:
            state, reason = JAMMED, jam
        elif self.last_report_t is None:
            state, reason = NO_DATA, "no GPS report (receiver off, not streamed, or replay)"
        else:
            state, reason = OK, f"3D fix, {self.sats} sats" + (f", HDOP {self.hdop:.1f}" if self.hdop else "")
        if state != self.state:
            self.transitions.append((t, state, reason))
        self.state, self.reason = state, reason

    # ----- output -----
    def row(self):
        f = lambda x, d=1: "" if x is None else f"{x:.{d}f}"
        return dict(zip(self.COLUMNS, [self.state, "" if self.fix is None else self.fix,
                                       "" if self.sats is None else self.sats, f(self.hdop, 2),
                                       f(self.gps_disp), f(self.vis_disp), self.reason]))

    def message(self):
        tail = "" if self.state in (OK, NO_DATA) else "  (monitor only: the hold is unaffected)"
        return f"GNSS: {self.state} - {self.reason}{tail}"
