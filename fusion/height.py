# =====================================================================================
# fusion/height.py  -  steady height for the pixel -> metre scale   BRIEF ITEM 1
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   The camera measures movement in PIXELS. To turn pixels into metres we need the
#   height: from 2 m up one pixel is ~0.5 cm, from 40 m up it's ~10 cm. So if the
#   height reading jumps (barometer gust, rangefinder seeing a bush), our position
#   jumps too, and the drone twitches. This filter smooths the height and ignores
#   impossible jumps, but still follows a REAL climb or descent.
#
# HOW IT WORKS (technical)
#   - Gate: a new reading is rejected if it differs from the filtered value by more
#     than HEIGHT_MAX_RATE_MPS * dt + 5 cm (physically impossible climb rate).
#     After HEIGHT_MAX_REJECTS rejections in a row the filter re-seeds (the step is real,
#     e.g. the rangefinder took over from the barometer).
#   - Smooth: exponential moving average, value += alpha * (reading - value).
#   The height source itself (rangefinder if fresh, else barometer) is still chosen by
#   Saumya's DroneLink.height(); this only cleans its output.
#
# WHERE IT PLUGS IN
#   vision_hold.py: height = filter.update(t, link.height(t)) (skipped with --fixed-height).
#
# REFERENCES
#   - Scaramuzza & Fraundorfer, "Visual Odometry: Part I", IEEE Robotics & Automation
#     Magazine 2011 (monocular VO needs an external scale; here: height).
#   - ArduPilot rangefinder / barometer height: https://ardupilot.org/copter/docs/common-rangefinder-landingpage.html
# =====================================================================================

import config


class HeightFilter:
    def __init__(self, max_rate=config.HEIGHT_MAX_RATE_MPS, alpha=config.HEIGHT_SMOOTH,
                 max_rejects=config.HEIGHT_MAX_REJECTS):
        self.max_rate, self.alpha, self.max_rejects = max_rate, alpha, max_rejects
        self.value = None       # filtered height, metres
        self.t = None           # time of the last ACCEPTED reading
        self.rejects = 0

    def update(self, t, height):
        """Feed one reading (None = no reading). Returns the filtered height or None."""
        if height is None:
            return None                      # same as before: no height -> no position
        if self.value is None or self.rejects >= self.max_rejects:
            self.value, self.t, self.rejects = height, t, 0
            return self.value
        dt = max(t - self.t, 1e-3)
        if abs(height - self.value) > self.max_rate * dt + 0.05:
            self.rejects += 1                # spike: keep the old value
            return self.value
        self.rejects = 0
        self.value += self.alpha * (height - self.value)
        self.t = t
        return self.value
