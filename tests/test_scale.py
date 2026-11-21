# =====================================================================================
# tests/test_scale.py  -  bench height from motion (BETAFLIGHT CHANGE, fusion/scale.py)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   A pretend judge carries the aircraft at a height we do NOT tell the code (0.7 to 1.3 m),
#   walking with a natural sway. The accelerometer (with a tilt-error bias, noise, and its
#   axes in a different order and sign than the camera's) and the camera (pixel noise) are
#   simulated. The estimator must find the height within 10%, and say "don't know" when the
#   aircraft is simply held still.
# RUN:  python -m tests.test_scale   (~2 s)
# =====================================================================================
import math

import numpy as np

from fusion.scale import ImuIntegrator, ScaleEstimator, fuse

F = 208.0
rng = np.random.default_rng(5)


def carry(h, walk=True, seconds=12.0):
    fs = 200
    t = np.arange(0, seconds, 1 / fs)
    s = np.clip((t - 2) / 6, 0, 1)
    env = np.sin(np.pi * s) ** 2 if walk else 0 * t
    x = (1.2 * (s - np.sin(2 * np.pi * s) / (2 * np.pi)) if walk else 0 * t) + 0.018 * env * np.sin(2 * np.pi * 1.8 * t)
    y = 0.012 * env * np.sin(np.pi * 1.8 * t + 1)
    z = h + 0.01 * env * np.sin(2 * np.pi * 1.8 * t + 0.5)
    return t, np.c_[x, y, z]


def estimate(h, walk=True):
    t, P = carry(h, walk)
    dt = t[1] - t[0]
    acc = np.gradient(np.gradient(P, dt, axis=0), dt, axis=0)
    acc[:, 2] += 9.80665                                  # gravity on the vertical axis
    acc += np.r_[np.radians(1.0) * 9.81, -np.radians(0.7) * 9.81, 0.0]   # tilt-error bias
    acc += rng.normal(0, 0.03, acc.shape)
    sensor = np.c_[-acc[:, 1], acc[:, 2], acc[:, 0]]      # sensor axes: other order and signs than the camera's
    integ, est = ImuIntegrator(), ScaleEstimator()
    for k in range(0, len(t), 8):                         # Betaflight raw IMU at 25 Hz
        integ.add(t[k], sensor[k])
        if k % 16 == 0:                                   # camera ~12 fps
            u = P[k, :2] / P[k, 2] + rng.normal(0, 0.15 / F, 2)
            est.update(t[k], integ.at(t[k]), (u[0], u[1], math.log(P[k, 2]) + rng.normal(0, 0.002)))
    return est.estimate()


print("Test 1: walking carry, unknown height, scrambled accelerometer axes")
worst = 0.0
for h in (0.7, 0.85, 1.0, 1.15, 1.3):
    est, sig = estimate(h)
    err = abs(est - h) / h
    worst = max(worst, err)
    print(f"  true {h:.2f} m -> estimated {est:.2f} +- {sig:.2f} m ({err:.1%})")
    assert est is not None and err < 0.10
print(f"  worst error {worst:.1%}")

print("\nTest 2: held still -> 'don't know' (the bench guess stays)")
est, sig = estimate(1.0, walk=False)
print(f"  estimate {est}, sigma {sig}")
assert est is None and math.isinf(sig)
assert fuse(1.0, 0.25, None, math.inf) == 1.0 and abs(fuse(1.0, 0.25, 1.2, 0.02) - 1.2) < 0.01
print("\nALL SCALE TESTS PASSED")
