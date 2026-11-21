# =====================================================================================
# safety/side_monitors.py  -  the advisors that WATCH but never steer   (OWNER: Ayush)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   One small object that vision_hold.py calls once per frame, AFTER the position has
#   already been sent to ArduPilot. It runs the "watch-only" helpers:
#     - the Altitude Ceiling Advisor (safety/ceiling.py, task A4)
#     - the GNSS integrity monitor   (safety/gnss_monitor.py, task A5)
#   and writes what they think to a separate file next to the flight log
#   (logs/flight_<time>_safety.csv), so the flight log format the team's tools read is
#   unchanged. It prints a line only when the advice changes (or every 10 s), so the
#   console stays readable.
#
# SAFETY RULES (why it can't hurt the hold)
#   - It is called after send_odometry() and returns nothing; vision_hold ignores it.
#   - Every call is wrapped in try/except: a bug here prints once and is switched off,
#     it never stops the hold loop.
#   - It sends nothing that steers the drone. (The GNSS monitor may ask the autopilot to
#     STREAM GPS reports once at start: a read request, not navigation.)
#   - Its time per frame is measured by compute/profiler.py as stage "safety".
#
# WHERE IT PLUGS IN
#   vision_hold.py: monitors = SideMonitors(link, log_path)   before the loop
#                   monitors.frame(...)                        after the send
#                   monitors.close()                           in the safe shutdown
#   Switches: config.CEILING_ADVISOR, config.GNSS_MONITOR.
# =====================================================================================

import csv
import os
import time

import config


class SideMonitors:
    def __init__(self, link, log_path, clock=time.time):
        self.link, self.clock = link, clock
        self.monitors = []
        self.broken = False
        self._last_msgs, self._last_print = {}, {}
        if config.CEILING_ADVISOR:
            from safety.ceiling import CeilingAdvisor
            self.monitors.append(("ceiling", CeilingAdvisor()))
        if getattr(config, "GNSS_MONITOR", False):
            try:
                from safety.gnss_monitor import GnssMonitor
                self.monitors.append(("gnss", GnssMonitor.attach(link)))
            except ImportError:
                pass
        self.path, self.file, self.writer = None, None, None
        if self.monitors:
            self.path = os.path.splitext(log_path)[0] + "_safety.csv"
            self.file = open(self.path, "w", newline="")
            self.writer = csv.writer(self.file)
            cols = ["t_capture"]
            for _, m in self.monitors:
                cols += m.COLUMNS
            self.writer.writerow(cols)
            print(f"Side monitors ({', '.join(n for n, _ in self.monitors)}) -> {self.path}")

    def frame(self, t, height, focal_px, quality, north, east, status, reset_counter):
        if not self.monitors or self.broken:
            return
        try:
            row = [f"{t:.3f}"]
            for name, m in self.monitors:
                if name == "ceiling":
                    out = m.update(height, focal_px, quality)
                else:
                    out = m.update(t, north, east, quality, status, reset_counter)
                row += [out.get(c, "") for c in m.COLUMNS]
                self._maybe_print(name, m)
            self.writer.writerow(row)
        except Exception as e:                         # never let a watcher stop the hold
            self.broken = True
            print(f"Side monitors switched off after an error (the hold continues): {e!r}")

    def _maybe_print(self, name, m):
        msg, now = m.message(), time.time()
        key = msg.split("->")[-1] if "->" in msg else msg
        if key != self._last_msgs.get(name) or now - self._last_print.get(name, 0) > 10.0:
            print(msg)
            self._last_msgs[name], self._last_print[name] = key, now

    def close(self):
        if self.file:
            self.file.close()
            for name, m in self.monitors:
                print(m.message())
            print(f"Side monitor log: {self.path}")
