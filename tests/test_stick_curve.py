# =====================================================================================
# tests/test_stick_curve.py  -  Betaflight's real stick -> lean curve (BETAFLIGHT CHANGE)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Checks link/stick_curve.py against the numbers Betaflight itself documents, and that the
#   inverse is exact, so "lean 5 degrees" really becomes the stick value that leans 5 degrees.
#     1. centre sensitivity: default Actual rates 6.3 deg, default Betaflight rates 18 deg
#        (Betaflight Configurator, angle_limit 60)
#     2. the inverse is exact (to 0.001 deg) for several rate settings, and the curve only rises
#     3. deadband: inside it the drone stays level; the inverse steps over it
#     4. how wrong the old straight line was (printed for the slides)
# RUN:  python -m tests.test_stick_curve   (< 1 s)
# =====================================================================================
from link.stick_curve import Rates, angle_from_us, centre_sensitivity, us_from_angle

print("Test 1: centre sensitivity matches Betaflight's documentation")
act, bfr = Rates(), Rates("BETAFLIGHT", 100, 70, 0)
ca, cb = centre_sensitivity(act), centre_sensitivity(bfr)
print(f"  default Actual rates {ca:.1f} deg (docs 6.3), default Betaflight rates {cb:.1f} deg (docs 18)")
assert abs(ca - 6.3) < 0.2 and abs(cb - 18.0) < 0.2

print("\nTest 2: exact inverse, rising curve, full stick = angle_limit")
for r in (act, bfr, Rates(expo=50), Rates("BETAFLIGHT", 120, 75, 20), Rates(angle_limit=45)):
    worst = max(abs(angle_from_us(us_from_angle(d / 10, r), r) - d / 10) for d in range(-400, 401))
    seq = [angle_from_us(u, r) for u in range(1000, 2001, 5)]
    assert worst < 1e-3 and all(b >= a for a, b in zip(seq, seq[1:])), r
    assert abs(angle_from_us(2000, r) - r.angle_limit) < 1e-6 and abs(angle_from_us(1000, r) + r.angle_limit) < 1e-6
print("  5 rate settings: inverse within 0.001 deg, curve never falls, full stick = angle_limit")

print("\nTest 3: deadband")
rd = Rates(deadband=5)
assert angle_from_us(1504, rd) == 0.0 and angle_from_us(1496, rd) == 0.0
assert us_from_angle(1.0, rd) > 1505 and abs(angle_from_us(us_from_angle(1.0, rd), rd) - 1.0) < 1e-3
print("  +-5 us around 1500 = level; 1 deg lean steps over the deadband")

print("\nTest 4: the old straight line vs Betaflight (default Actual rates, angle_limit 60)")
for deg in (2, 5, 8, 15):
    old = 1500 + 500 * deg / 60
    print(f"  want {deg:2d} deg: old sent {old - 1500:5.1f} us -> Betaflight leans {angle_from_us(old, act):4.1f} deg;"
          f" now sends {us_from_angle(deg, act) - 1500:5.1f} us")
assert angle_from_us(1500 + 500 * 8 / 60, act) < 3.0            # the old 8 deg cap came out under 3 deg
print("\nALL STICK CURVE TESTS PASSED")
