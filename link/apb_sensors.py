# =====================================================================================
# link/apb_sensors.py  -  the onboard computer's OWN barometer and motion sensor
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   The APB (the drone's onboard computer) has its own air-pressure sensor (DPS310) and
#   its own motion sensor (ICM-42605), separate from the flight controller's. Linux shows
#   them as small text files. We read:
#     - the pressure, and turn its change into "metres higher/lower than the ground".
#       This is our height on the Betaflight route. It is pure air pressure: no GPS is
#       mixed in (the flight controller's own altitude may blend GPS, which Stage 2
#       forbids during a hold, so we do not use that one).
#     - the turn rates (gyro), shown on the status line: bench step 1 wants camera AND
#       IMU live in our own process.
#
# HOW IT WORKS (technical)
#   Linux IIO sysfs: /sys/bus/iio/devices/iio:deviceN/name identifies the chip.
#   Pressure: in_pressure_input (kPa, the IIO unit), else in_pressure_raw * in_pressure_scale.
#   Gyro: in_anglvel_{x,y,z}_raw * in_anglvel_scale (rad/s).
#   Height from pressure (international barometric formula, good to a few % near the ground):
#       h = 44330 * (1 - (p / p0) ^ 0.1903)     p0 = pressure on the ground
#   Everything returns None when the sensor or file is missing (laptop, other board): the
#   callers then run without it. Proof: tests/test_betaflight.py (a pretend sysfs folder).
#   NOT CHECKED ON THE REAL BOARD YET: the exact file names. `python -m link.apb_sensors`
#   prints what it finds; run it first on the APB.
#
# REFERENCES
#   - Linux IIO sysfs ABI: https://www.kernel.org/doc/Documentation/ABI/testing/sysfs-bus-iio
#   - Stage 2 package section 02 (ICM-42605 and DPS310 through Linux IIO, CLOCK_MONOTONIC).
#   - NOAA, "A Quick Derivation relating altitude to air pressure" (barometric formula).
# =====================================================================================
import glob
import os
import threading
import time

import config

IIO_ROOT = "/sys/bus/iio/devices"


def _read(path):
    try:
        with open(path) as f:
            return float(f.read().strip())
    except (OSError, ValueError):
        return None


def find_device(name_part, root=IIO_ROOT):
    """Folder of the first IIO device whose name contains name_part (e.g. 'dps310'), or None."""
    for d in sorted(glob.glob(os.path.join(root, "iio?device*"))):
        try:
            with open(os.path.join(d, "name")) as f:
                if name_part in f.read().lower():
                    return d
        except OSError:
            pass
    return None


def pressure_to_height(p, p0):
    """Metres above the place where the pressure was p0 (same unit for both)."""
    return 44330.0 * (1.0 - (p / p0) ** 0.1903)


class Baro:
    """The APB's DPS310. height() = metres above the last zero() point, or None."""

    def __init__(self, root=IIO_ROOT):
        self.dir = find_device("dps310", root)
        self.p0 = None
        self._bg, self._p = False, None

    def start(self):
        """STAGE 2 (measured): one read blocks for 1-2 s on the APB, so anything that must not wait calls
        this once. A side thread then keeps reading and pressure() returns the newest value at once."""
        if self.dir is None or self._bg:
            return
        for name in ("in_pressure_sampling_frequency", "in_temp_sampling_frequency"):
            try:                                     # a sensor setting, not GPIO / device tree; gone at reboot
                if config.BARO_SAMPLING_HZ:
                    with open(os.path.join(self.dir, name), "w") as f:
                        f.write(str(config.BARO_SAMPLING_HZ))
            except OSError:
                pass

        def loop():
            while True:
                self._p = self._measure()
                time.sleep(0.005)
        self._bg = True
        threading.Thread(target=loop, daemon=True).start()

    def pressure(self):
        return self._p if self._bg else self._measure()

    def _measure(self):
        if self.dir is None:
            return None
        p = _read(os.path.join(self.dir, "in_pressure_input"))
        if p is None:
            raw, scale = _read(os.path.join(self.dir, "in_pressure_raw")), _read(os.path.join(self.dir, "in_pressure_scale"))
            p = raw * scale if raw is not None and scale is not None else None
        return p if p and p > 0 else None

    def zero(self):
        """Call on the ground: 'here is height 0'. Smoothed, so one noisy reading can't shift it."""
        p = self.pressure()
        if p is not None:
            self.p0 = p if self.p0 is None else 0.98 * self.p0 + 0.02 * p

    def height(self):
        p = self.pressure()
        return None if p is None or self.p0 is None else pressure_to_height(p, self.p0)


class Gyro:
    """The APB's ICM-42605 turn rates. rates() = (x, y, z) in rad/s, or None."""

    def __init__(self, root=IIO_ROOT):
        self.dir = find_device("icm426", root)

    def rates(self):
        if self.dir is None:
            return None
        scale = _read(os.path.join(self.dir, "in_anglvel_scale")) or 1.0
        raw = [_read(os.path.join(self.dir, f"in_anglvel_{a}_raw")) for a in "xyz"]
        return None if None in raw else tuple(r * scale for r in raw)


if __name__ == "__main__":                       # on the APB: what is there, and does it move?
    import time
    for d in sorted(glob.glob(os.path.join(IIO_ROOT, "iio?device*"))):
        print(d, open(os.path.join(d, "name")).read().strip(), sorted(os.listdir(d)))
    baro, gyro = Baro(), Gyro()
    for _ in range(20):
        baro.zero() if baro.p0 is None else None
        print(f"pressure {baro.pressure()}  height {baro.height()}  gyro {gyro.rates()}")
        time.sleep(0.25)
