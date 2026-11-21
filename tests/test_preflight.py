# =====================================================================================
# tests/test_preflight.py  -  every GO / WARN / NO-GO line, with fakes   (OWNER: Ayush, task A2)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   No drone, no APB needed: each check gets a fake world (a fake /proc with a process
#   holding the camera, a fake thermometer file, a fake SD card, a fake camera that runs at
#   a known frame rate, a fake check_link result) and must give the right verdict with a
#   reason. The repo's own calib.npz (from a 640x480 webcam) must be NO-GO, as must
#   trial mode switched off. Finally the real preflight.py runs on this computer in --bench
#   mode: it must say NO-GO (no Justice D here, webcam lens file) and save its report.
#
# Run (from the repo root):  python -m tests.test_preflight      (~3 s)
# REFERENCES: preflight.py
# =====================================================================================

import os
import subprocess
import tempfile
import time
from collections import namedtuple

import numpy as np

import config
import preflight as pf
from calibration.check_calib import write_spec
from vision.timing import save_timing

tmp = tempfile.mkdtemp(prefix="dronecode_preflight_")
results = []


def expect(check, level, words=""):
    check = check[0] if isinstance(check, tuple) else check
    print(f"  {check.level:5s} {check.name:20s} {check.reason}")
    assert check.level == level, f"{check.name}: expected {level}, got {check.level} ({check.reason})"
    assert words.lower() in check.reason.lower(), f"reason should mention '{words}': {check.reason}"
    results.append(check.level)


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)
    return path


print("1. trial mode")
ok_service = write(os.path.join(tmp, "ok.service"), "[Service]\nExecStart=/x/python vision_hold.py --camera 2\n")
bad_service = write(os.path.join(tmp, "bad.service"), "[Service]\nExecStart=/x/python vision_hold.py --practice\n")
expect(pf.check_trial_mode([ok_service], trial_on=True), pf.GO, "ON")
expect(pf.check_trial_mode([ok_service], trial_on=False), pf.NOGO, "TRIAL_MODE is False")
expect(pf.check_trial_mode([bad_service], trial_on=True), pf.NOGO, "--practice")
expect(pf.check_trial_mode(), pf.GO, "ON")                                     # the repo as shipped

print("2. busy devices")
proc = os.path.join(tmp, "proc")
os.makedirs(os.path.join(proc, "4242", "fd"))
os.symlink("/dev/video2", os.path.join(proc, "4242", "fd", "7"))
write(os.path.join(proc, "4242", "cmdline"), "python\0vision_hold.py\0--camera\0002")
os.makedirs(os.path.join(proc, "99", "fd"))
os.symlink("/dev/null", os.path.join(proc, "99", "fd", "0"))
expect(pf.check_busy(["/dev/video2", "/dev/ttymxc2"], proc, systemctl=lambda: "inactive"), pf.NOGO, "pid 4242")
expect(pf.check_busy(["/dev/video3", "/dev/ttymxc2"], proc, systemctl=lambda: "inactive"), pf.GO, "free")
expect(pf.check_busy(["/dev/video3"], proc, systemctl=lambda: "active"), pf.NOGO, "systemctl stop")

print("3. CPU temperature (fan ~62 C, shutdown 95 C)")
for milli, level in ((62000, pf.GO), (78000, pf.WARN), (91000, pf.NOGO)):
    expect(pf.check_temperature(write(os.path.join(tmp, f"t{milli}"), f"{milli}\n")), level, f"{milli // 1000}")
expect(pf.check_temperature(os.path.join(tmp, "missing")), pf.WARN, "can't read")

print("4. storage")
Usage = namedtuple("Usage", "total used free")
sd = os.path.join(tmp, "sdcard")
os.makedirs(sd)
mounts = write(os.path.join(tmp, "mounts"), f"/dev/sda1 / ext4 rw 0 0\n/dev/mmcblk1p1 {sd} vfat rw 0 0\n")
mounts_ex = write(os.path.join(tmp, "mounts_ex"), f"/dev/mmcblk1p1 {sd} exfat rw 0 0\n")
expect(pf.check_storage(os.path.join(tmp, "nosd")), pf.WARN, "no ")
expect(pf.check_storage(sd, mounts_ex, lambda p: Usage(0, 0, 100e6)), pf.NOGO, "100 MB")
expect(pf.check_storage(sd, mounts_ex, lambda p: Usage(0, 0, 1000e6)), pf.WARN, "~4 min")
expect(pf.check_storage(sd, mounts, lambda p: Usage(0, 0, 30e9)), pf.WARN, "FAT32")
expect(pf.check_storage(sd, mounts_ex, lambda p: Usage(0, 0, 30e9)), pf.GO, "exfat")

print("5. calibration (Saumya's check_calib + size == camera)")
expect(pf.check_calibration("calib.npz"), pf.GO, "1920x1080")                # the repo file: datasheet values
spec = os.path.join(tmp, "calib_spec.npz")
write_spec(spec, force=True)
expect(pf.check_calibration(spec, camera_size=(1920, 1080)), pf.GO, "1920x1080")
expect(pf.check_calibration(spec, camera_size=(640, 480)), pf.NOGO, "recalibrate")
expect(pf.check_calibration(os.path.join(tmp, "none.npz")), pf.NOGO, "no ")

print("6. timing")
expect(pf.check_timing(os.path.join(tmp, "no_timing.npz")), pf.NOGO, "GUESSED")
save_timing(0.0142, -0.004, os.path.join(tmp, "good.npz"))
expect(pf.check_timing(os.path.join(tmp, "good.npz")), pf.GO, "14.2 ms")
save_timing(0.025, 0.0, os.path.join(tmp, "slow.npz"))
expect(pf.check_timing(os.path.join(tmp, "slow.npz")), pf.NOGO, "impossible")
save_timing(0.014, 0.25, os.path.join(tmp, "late.npz"))
expect(pf.check_timing(os.path.join(tmp, "late.npz")), pf.WARN, "offset")


class FakeCam:
    """A camera with its own clock: each read() advances it by 1/fps (no real sleeping, so a busy
    computer can't change the measured frame rate)."""
    def __init__(self, opened=True, size=(1920, 1080), fps=60.0, picture=True):
        self.opened, self.size, self.dt, self.picture = opened, size, 1.0 / fps, picture
        self.t = 0.0

    def isOpened(self):
        return self.opened

    def read(self):
        self.t += self.dt
        if not self.picture:
            return False, None
        return True, np.zeros((self.size[1], self.size[0], 3), np.uint8)

    def release(self):
        pass


print("7. camera (fake camera with its own clock)")


def cam_check(**kw):
    cam = FakeCam(**kw)
    return pf.check_camera("2", lambda s: cam, clock=lambda: cam.t)


expect(cam_check(), pf.GO, "1920x1080 at 60 fps")
expect(cam_check(size=(640, 480)), pf.NOGO, "640x480")
expect(cam_check(fps=30), pf.WARN, "30 fps")
expect(cam_check(opened=False), pf.NOGO, "can't open")
expect(cam_check(picture=False), pf.NOGO, "no picture")

print("8. autopilot link (link/check_link.py as a subprocess)")
Run = namedtuple("Run", "returncode stdout stderr")
expect(pf.check_link("/dev/ttymxc2", runner=lambda *a, **k: Run(0, "...\nPASS\n", "")), pf.GO, "PASS")
fail_text = "Firmware : ARDUPILOT 4.4.0\n\nFIX BEFORE FLIGHT:\n  - attitude only 12 Hz (need >= 40)\n  - no height\n"
expect(pf.check_link("/dev/ttymxc2", runner=lambda *a, **k: Run(1, fail_text, "")), pf.NOGO, "attitude only 12 Hz")


def timeout_runner(*a, **k):
    raise subprocess.TimeoutExpired("check_link", 60)


expect(pf.check_link("/dev/ttymxc2", runner=timeout_runner), pf.NOGO, "no answer")

print("9. autopilot safety parameters")
good = {"FLTMODE_CH": 9, "EK3_SRC_OPTIONS": 0, "RC5_OPTION": 153, "RC7_OPTION": 0}
for c in pf.check_autopilot_safety(good):
    expect(c, pf.GO)
c_switch, c_vel, c_mode = pf.check_autopilot_safety({"FLTMODE_CH": 5, "EK3_SRC_OPTIONS": 1, "RC8_OPTION": 90})
expect(c_switch, pf.NOGO, "RC8_OPTION")
expect(c_vel, pf.NOGO, "GPS velocity")
expect(c_mode, pf.WARN, "CH5")
expect(pf.check_autopilot_safety({})[2], pf.WARN, "could not read")

print("10. verdict and report")
checks = [pf.Check("a", pf.GO, "x"), pf.Check("b", pf.WARN, "y")]
assert pf.verdict(checks) == pf.GO and pf.verdict(checks, strict=True) == pf.NOGO
assert pf.verdict(checks + [pf.Check("c", pf.NOGO, "z")]) == pf.NOGO
text = pf.report(checks + [pf.Check("c", pf.NOGO, "z")])
assert "VERDICT: NO-GO" in text and "Do not fly" in text and all(f" {n} " in text for n in "abc")

print("\n11. the real preflight.py on THIS computer (--bench: no autopilot)")
code = pf.main(["--bench", "--camera", os.path.join(tmp, "no_camera.mp4")])
print(f"  exit code {code}")
assert code == 1, "this computer has no Justice D and a webcam lens file: must be NO-GO"

n = {lvl: results.count(lvl) for lvl in (pf.GO, pf.WARN, pf.NOGO)}
print(f"\n{len(results)} checks verified: {n[pf.GO]} GO, {n[pf.WARN]} WARN, {n[pf.NOGO]} NO-GO, each with its reason")
print("\nALL PREFLIGHT TESTS PASSED")
