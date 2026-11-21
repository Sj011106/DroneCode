# =====================================================================================
# compute/thermal.py  -  do less work when the board gets hot              BETAFLIGHT CHANGE
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   The APB's fan holds about 62 C and the board switches itself off at 95 C (APB manual
#   p.11). Our vision program is the biggest load (~1 of the 4 cores at ~13 frames a
#   second). If the board ever gets hot (sun on the field, long bench session), we would
#   rather track fewer frames than lose the board in the air. Above THERMAL_SLOW_C we use
#   every 2nd frame; above THERMAL_HOT_C every 3rd and skip the expensive ORB searches.
#   Prototype simulation: the hold still qualified at every level with vision at 10 frames
#   a second, because the controller damps between frames. The stick sender is a separate
#   program and is never slowed down.
#
# HOW IT WORKS: reads /sys/class/thermal/thermal_zone0/temp (milli-degrees C) at most once a
#   second. No file (laptop) -> always 1 (full rate). Proof: tests/test_thermal.py.
# =====================================================================================
import time

import config


class Thermal:
    def __init__(self, path=None, clock=time.monotonic):
        self.path = path or config.THERMAL_FILE
        self.clock = clock
        self.t = -1e9
        self.temp_c = None
        self.every = 1

    def read(self):
        try:
            with open(self.path) as f:
                v = float(f.read().strip())
            return v / 1000.0 if v > 1000 else v
        except (OSError, ValueError):
            return None

    def frame_every(self):
        """Process every Nth frame: 1 (normal), 2 (hot), 3 (very hot)."""
        now = self.clock()
        if now - self.t >= 1.0:
            self.t = now
            self.temp_c = self.read()
            c = self.temp_c
            self.every = 1 if c is None or c < config.THERMAL_SLOW_C else (2 if c < config.THERMAL_HOT_C else 3)
        return self.every

    def allow_orb(self):
        return self.every < 3
