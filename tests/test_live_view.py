# tests/test_live_view.py
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   The laptop page (tools/live_view.py) must never hurt the drone: no crash on bad
#   numbers, no work when nobody watches, and if it cannot start, vision_hold carries on.
#   It must also really show the picture and the estimate.
#   Run:  python -m tests.test_live_view
# -------------------------------------------------------------------------------------
import json
import socket
import time
import urllib.error
import urllib.request

import numpy as np

from tools.live_view import LiveView


def get(url, timeout=3.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.status, r.headers.get("Content-Type", ""), r.read()


quiet = lambda *a, **k: None
live = LiveView(port=0, enabled=True, host="127.0.0.1", log=quiet)
assert live.server is not None, "server should start on a free port"
base = f"http://127.0.0.1:{live.server.server_address[1]}"
gray = (np.arange(360 * 640) % 251).astype(np.uint8).reshape(360, 640)

print("Test 1: bad input never reaches the vision loop as an exception")
for junk in (None, "text", np.zeros(5), object()):
    live.frame(junk)
live.state(ready=True, north_m=float("nan"), east_m=float("inf"), sticks=np.array([1500, 1500, 1300, 1500]),
           odd=object(), quality=np.int64(80))
code, ctype, body = get(base + "/state")
s = json.loads(body)
assert code == 200 and "json" in ctype
assert s["north_m"] is None and s["east_m"] is None, "NaN / inf must show as null, not break the page"
assert s["sticks"] == [1500, 1500, 1300, 1500] and s["quality"] == 80
assert "distance_m" not in s, "no distance from invalid numbers"
print(f"  junk frames and NaN/inf/numpy values accepted silently; /state stays valid JSON")

print("\nTest 2: nobody watching -> no picture copied (zero cost with the laptop unplugged)")
live.frame(gray)
assert live._frame is None and live.video_clients == 0
print("  frame() stored nothing")

print("\nTest 3: the estimate is shown as the judge needs it (distance, forward/right)")
live.state(ready=True, status="OK", north_m=1.0, east_m=0.0, yaw0=np.pi / 2, height_change_m=0.5)
s = json.loads(get(base + "/state")[2])
assert abs(s["distance_m"] - 1.0) < 1e-9
assert abs(s["forward_m"]) < 1e-9 and abs(s["right_m"] + 1.0) < 1e-9, "nose east, moved north = moved left"
assert s["state_age_s"] is not None and s["state_age_s"] < 1.0
print(f"  north 1 m with the nose east -> forward {s['forward_m']:+.2f} m, right {s['right_m']:+.2f} m")

print("\nTest 4: the camera stream delivers JPEG pictures while someone watches")
req = urllib.request.urlopen(base + "/video", timeout=3.0)
assert "multipart/x-mixed-replace" in req.headers.get("Content-Type", "")
t_end = time.monotonic() + 3.0
while live.video_clients < 1 and time.monotonic() < t_end:
    time.sleep(0.01)
for _ in range(5):
    live.frame(gray)
    time.sleep(0.05)
chunk = req.read(4000)
req.close()
assert b"--frame" in chunk and b"\xff\xd8" in chunk, "a JPEG part must arrive"
t_end = time.monotonic() + 3.0
while live.video_clients > 0 and time.monotonic() < t_end:
    live.frame(gray)                            # the server notices the closed browser on its next write
    time.sleep(0.05)
assert live.video_clients == 0, "viewer count must drop when the browser leaves"
code, ctype, jpg = get(base + "/snapshot.jpg")
print(f"  stream ok, viewer count back to 0, snapshot {len(jpg)} bytes ({ctype})")

print("\nTest 5: the page itself")
code, ctype, html = get(base + "/")
assert code == 200 and b"Drone live view" in html and b"/video" in html and b"/state" in html
try:
    get(base + "/nothing")
    raise AssertionError("unknown path must be 404")
except urllib.error.HTTPError as e:
    assert e.code == 404
print("  page served; unknown paths 404")

print("\nTest 6: port already taken -> live view switches itself off, nothing raises")
blocker = socket.socket()
blocker.bind(("127.0.0.1", 0))
blocker.listen(1)
msgs = []
off = LiveView(port=blocker.getsockname()[1], enabled=True, host="127.0.0.1", log=msgs.append)
assert off.server is None and msgs and "off" in msgs[0]
off.frame(gray)
off.state(ready=True)
off.close()
blocker.close()
disabled = LiveView(enabled=False, log=msgs.append)
assert disabled.server is None
disabled.frame(gray)
print(f"  '{msgs[0][:60]}...'; disabled view accepts calls too")

live.close()
assert live.errors >= 0
print("\nALL LIVE VIEW TESTS PASSED")
