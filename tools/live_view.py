# tools/live_view.py
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Stage 2 bench (section 05): "Your laptop shows your live estimate." This file is that
#   screen. It runs INSIDE vision_hold.py on the drone as a small background web server.
#   On the laptop you only open a browser:   http://192.168.75.1:8088
#   The page shows the camera picture, READY / NOT READY, the distance from home, forward /
#   right, the height change, the engage switch, our stick values, and a top-down track
#   of where the drone has been since home (1 m grid) - what the judge needs for steps 2-4.
#
# SAFETY (the drone must work without the laptop)
#   - Nothing here can stop or slow the vision loop: frame() and state() only store a
#     reference / a small dict and NEVER raise. All the work (JPEG, network) happens in
#     the server's own threads.
#   - No browser connected (laptop unplugged, bench step 5) = no picture copied, no JPEG
#     made: zero cost.
#   - Port busy or anything else wrong at start-up: the live view switches itself off with
#     one message. vision_hold.py carries on exactly as before.
#   - It only shows numbers. It never sends anything to the flight controller and it has
#     no buttons that change the estimate (the judge sees the raw values).
#
# HOW TO USE
#   In vision_hold.py (4 lines, see docs/LIVE_VIEW.md):
#       from tools.live_view import LiveView
#       live = LiveView()                 # once, at start-up
#       live.frame(gray)                  # every frame, right after the picture is grabbed
#       live.state(ready=..., ...)        # every frame, once the estimate is known
#   Try it on a laptop without the drone:
#       python -m tools.live_view --demo                 # moving synthetic floor
#       python -m tools.live_view --demo --camera 0      # your webcam
#   then open http://localhost:8088
#
# HOW IT WORKS (technical)
#   ThreadingHTTPServer (stdlib) on a daemon thread. Routes: "/" page, "/state" JSON,
#   "/video" MJPEG (multipart/x-mixed-replace, works in every browser's <img>),
#   "/snapshot.jpg" one picture (handy as evidence). One JPEG per new frame is shared by
#   all viewers; at most MAX_CLIENTS video viewers; video capped at VIDEO_FPS.
#   cv2.imencode releases the GIL, so encoding does not stall the vision thread.
# -------------------------------------------------------------------------------------

import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import cv2
    import numpy as np
except Exception:                      # the page still works without OpenCV (no picture)
    cv2 = np = None

try:
    import config as _config
except Exception:
    _config = None


def _cfg(name, default):
    return getattr(_config, name, default) if _config is not None else default


PORT = _cfg("LIVE_VIEW_PORT", 8088)          # laptop: http://192.168.75.1:8088
ENABLED = _cfg("LIVE_VIEW", True)            # config.LIVE_VIEW = False switches it off completely
VIDEO_FPS = _cfg("LIVE_VIEW_FPS", 8)         # pictures per second sent to the browser
JPEG_QUALITY = _cfg("LIVE_VIEW_JPEG_QUALITY", 70)
MAX_CLIENTS = 3                              # video viewers at once (bounds the APB's work)
STALE_S = 1.0                                # older than this = shown as NO DATA / NO CAMERA


def _clean(v):
    """JSON-safe copy: NaN / inf -> None, numpy numbers -> plain numbers, tuples -> lists."""
    if isinstance(v, bool) or v is None or isinstance(v, str):
        return v
    if isinstance(v, (int, float)):
        return v if math.isfinite(v) else None
    if isinstance(v, dict):
        return {str(k): _clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_clean(x) for x in v]
    try:                                     # numpy scalars and small arrays
        if hasattr(v, "tolist"):
            return _clean(v.tolist())
        return _clean(float(v))
    except Exception:
        return str(v)


class LiveView:
    """Background web page with the live picture and estimate. Never raises into the caller."""

    def __init__(self, port=None, enabled=None, host="0.0.0.0", log=print):
        self._lock = threading.Lock()
        self._frame = None                   # latest picture (a copy, only while someone watches)
        self._frame_id = 0
        self._frame_t = None
        self._points = None
        self._jpeg_id, self._jpeg = -1, None
        self._state, self._state_t = {}, None
        self._fps, self._fps_n, self._fps_t = 0.0, 0, time.monotonic()
        self.video_clients = 0
        self.errors = 0
        self.server = None
        self.url = None
        enabled = ENABLED if enabled is None else enabled
        port = PORT if port is None else port
        if not enabled:
            return
        try:
            self.server = ThreadingHTTPServer((host, port), _Handler)
            self.server.daemon_threads = True
            self.server.live = self
            threading.Thread(target=self.server.serve_forever, name="live_view", daemon=True).start()
            real_port = self.server.server_address[1]
            self.url = f"http://192.168.75.1:{real_port}"
            log(f"Live view: open {self.url} on the laptop (http://localhost:{real_port} on this computer)")
        except Exception as e:               # port busy, no network...: the drone does not need this
            self.server = None
            log(f"Live view off ({e}). The drone does not need it.")

    # ---------------- called by vision_hold.py (cheap, never raises) ----------------
    def frame(self, gray, points=None):
        """The picture we just processed (grey, any size). points = optional [(x, y), ...] to mark."""
        try:
            now = time.monotonic()
            self._fps_n += 1
            if now - self._fps_t >= 1.0:
                self._fps, self._fps_n, self._fps_t = self._fps_n / (now - self._fps_t), 0, now
            if self.server is None or self.video_clients <= 0:
                return                       # nobody watching: no copy, no work
            with self._lock:
                self._frame = gray.copy()    # vision_hold may reuse its buffers: keep our own
                self._points = None if points is None else [tuple(map(float, p[:2])) for p in points][:300]
                self._frame_id += 1
                self._frame_t = now
        except Exception:
            self.errors += 1

    def state(self, **fields):
        """The estimate right now. Known keys (all optional): ready, status, reason, quality, north_m,
        east_m, yaw0 (rad: heading at home, to show forward/right), height_m, height_change_m, engaged,
        sticks (roll, pitch, throttle, yaw), relocks, rehomes. Any other key is shown under 'more'."""
        try:
            if self.server is None:
                return
            s = dict(fields)
            n, e, yaw0 = s.get("north_m"), s.get("east_m"), s.pop("yaw0", None)
            if _finite(n) and _finite(e):
                s["distance_m"] = math.hypot(n, e)
                if _finite(yaw0):
                    s["forward_m"] = n * math.cos(yaw0) + e * math.sin(yaw0)
                    s["right_m"] = -n * math.sin(yaw0) + e * math.cos(yaw0)
            s = _clean(s)
            with self._lock:
                self._state, self._state_t = s, time.monotonic()
        except Exception:
            self.errors += 1

    def _viewers(self, delta):
        with self._lock:
            self.video_clients += delta
            return self.video_clients

    def close(self):
        try:
            if self.server is not None:
                self.server.shutdown()
                self.server.server_close()
        except Exception:
            pass
        self.server = None

    # ---------------- used by the web server threads ----------------
    def snapshot_json(self):
        now = time.monotonic()
        with self._lock:
            s = dict(self._state)
            st, ft = self._state_t, self._frame_t
        s["state_age_s"] = None if st is None else round(now - st, 3)
        s["frame_age_s"] = None if ft is None else round(now - ft, 3)
        s["fps"] = round(self._fps, 1)
        s["video_clients"] = self.video_clients
        s["stale_after_s"] = STALE_S
        return json.dumps(_clean(s)).encode()

    def jpeg(self):
        """JPEG of the newest picture (made once per picture, shared by all viewers), or None."""
        with self._lock:
            fid, img, pts = self._frame_id, self._frame, self._points
            if fid == self._jpeg_id:
                return fid, self._jpeg
        if img is None or cv2 is None:
            return fid, None
        try:
            if img.ndim == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            h, w = img.shape[:2]
            cx, cy, r = w // 2, h // 2, max(8, w // 40)
            cv2.line(img, (cx - r, cy), (cx + r, cy), (0, 200, 255), 1)       # straight-down point
            cv2.line(img, (cx, cy - r), (cx, cy + r), (0, 200, 255), 1)
            cv2.putText(img, "nose", (cx - 14, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)
            for x, y in pts or ():
                cv2.circle(img, (int(x), int(y)), 2, (60, 220, 60), -1)
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, int(JPEG_QUALITY)])
            data = buf.tobytes() if ok else None
        except Exception:
            self.errors += 1
            data = None
        with self._lock:
            if fid >= self._jpeg_id:
                self._jpeg_id, self._jpeg = fid, data
        return fid, data


def _finite(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


class _Handler(BaseHTTPRequestHandler):
    server_version = "LiveView/1"

    def log_message(self, *args):            # keep the drone's journal clean
        pass

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        live = self.server.live
        path = self.path.split("?")[0]
        try:
            if path == "/":
                self._send(200, "text/html; charset=utf-8", PAGE.encode())
            elif path == "/state":
                self._send(200, "application/json", live.snapshot_json())
            elif path == "/snapshot.jpg":
                live._viewers(+1)                # make frame() copy the next picture
                try:
                    end = time.monotonic() + 2.0
                    data = None
                    while data is None and time.monotonic() < end:
                        _, data = live.jpeg()
                        if data is None:
                            time.sleep(0.05)
                finally:
                    live._viewers(-1)
                if data is None:
                    self._send(503, "text/plain", b"no picture yet")
                else:
                    self._send(200, "image/jpeg", data)
            elif path == "/video":
                self._video(live)
            else:
                self._send(404, "text/plain", b"not found")
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass                                 # the browser went away
        except Exception:
            live.errors += 1

    def _video(self, live):
        if live._viewers(+1) > MAX_CLIENTS:
            live._viewers(-1)
            self._send(503, "text/plain", b"too many viewers")
            return
        try:
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last, gap = -1, 1.0 / max(1, VIDEO_FPS)
            while True:
                t0 = time.monotonic()
                fid, data = live.jpeg()
                if data is not None and fid != last:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n")
                    self.wfile.flush()
                    last = fid
                time.sleep(max(0.01, gap - (time.monotonic() - t0)))
        finally:
            live._viewers(-1)


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Drone live view</title>
<style>
:root{--bg:#0f1216;--panel:#181d23;--line:#2a323c;--text:#e8edf2;--dim:#8d99a6;--ok:#1fa35a;--bad:#c9353f;--warn:#d08b16;--accent:#2f8fd8}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:15px/1.4 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
header{display:flex;align-items:center;gap:12px;padding:10px 16px;border-bottom:1px solid var(--line);flex-wrap:wrap}
h1{font-size:16px;margin:0;font-weight:600;color:var(--dim)}
.badge{padding:6px 14px;border-radius:6px;font-weight:700;font-size:20px;letter-spacing:.5px}
.ready{background:var(--ok)}.notready{background:var(--bad)}.nodata{background:#555}
.chip{padding:4px 10px;border-radius:12px;border:1px solid var(--line);font-size:13px;color:var(--dim)}
.chip.on{background:var(--warn);color:#111;border-color:var(--warn);font-weight:700}
main{display:grid;grid-template-columns:minmax(0,1.4fr) minmax(0,1fr);gap:16px;padding:16px}
@media (max-width:900px){main{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:12px}
.cam{position:relative;background:#000;border-radius:6px;overflow:hidden;aspect-ratio:16/9;display:flex;align-items:center;justify-content:center}
.cam img{width:100%;height:100%;object-fit:contain;display:block}
.cam .msg{position:absolute;color:var(--dim)}
.big{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px;margin-top:12px}
.num{background:var(--bg);border:1px solid var(--line);border-radius:6px;padding:8px 10px}
.num .k{font-size:12px;color:var(--dim);text-transform:uppercase;letter-spacing:.4px}
.num .v{font-size:30px;font-weight:700;font-variant-numeric:tabular-nums}
.num .u{font-size:14px;color:var(--dim);margin-left:4px}
.reason{margin-top:10px;color:var(--dim);min-height:1.4em}
canvas{width:100%;aspect-ratio:1;background:var(--bg);border:1px solid var(--line);border-radius:6px;display:block}
.row{display:flex;justify-content:space-between;align-items:center;margin-bottom:8px;gap:8px}
button{background:var(--line);color:var(--text);border:0;border-radius:6px;padding:6px 10px;cursor:pointer}
table{width:100%;border-collapse:collapse;font-size:13px;margin-top:10px}td{padding:3px 4px;border-bottom:1px solid var(--line)}td:first-child{color:var(--dim)}
.lost{display:none;background:var(--bad);color:#fff;padding:8px 16px;font-weight:600}
</style></head><body>
<div class="lost" id="lost">No data from the drone: check the USB-C cable / that vision_hold is running.</div>
<header>
  <span class="badge nodata" id="badge">NO DATA</span>
  <span class="chip" id="engaged">switch off</span>
  <span class="chip" id="status">-</span>
  <span class="chip" id="fps">- fps</span>
  <h1>Drone live view (raw estimate, nothing on this page changes it)</h1>
</header>
<main>
  <section class="card">
    <div class="cam"><img id="cam" alt="camera"><span class="msg" id="cammsg">connecting to camera...</span></div>
    <div class="reason" id="reason"></div>
    <div class="big">
      <div class="num"><div class="k">Distance from home</div><div class="v"><span id="dist">-</span><span class="u">m</span></div></div>
      <div class="num"><div class="k">Height change</div><div class="v"><span id="dh">-</span><span class="u">m</span></div></div>
      <div class="num"><div class="k">Forward (+) / back (-)</div><div class="v"><span id="fwd">-</span><span class="u">m</span></div></div>
      <div class="num"><div class="k">Right (+) / left (-)</div><div class="v"><span id="right">-</span><span class="u">m</span></div></div>
    </div>
  </section>
  <section class="card">
    <div class="row"><strong>Track since home (top view, 1 m grid, nose = up)</strong><button id="clear">Clear trail</button></div>
    <canvas id="map" width="600" height="600"></canvas>
    <table id="more"></table>
  </section>
</main>
<script>
const $=id=>document.getElementById(id);
const fmt=(v,d=2)=>(v===null||v===undefined)?'-':(v>=0&&d>0?'+':'')+Number(v).toFixed(d);
let trail=[], camOk=false;
$('clear').onclick=()=>{trail=[]};
const cam=$('cam');
function startCam(){cam.src='/video?'+Date.now();}
cam.onload=()=>{camOk=true;$('cammsg').style.display='none'};
cam.onerror=()=>{camOk=false;$('cammsg').textContent='no camera picture (retrying)';$('cammsg').style.display='';setTimeout(startCam,2000)};
startCam();
function draw(s){
  const c=$('map'),g=c.getContext('2d'),W=c.width,H=c.height;
  g.clearRect(0,0,W,H);
  let span=4;                                   // metres across, grows to fit the trail
  for(const p of trail){span=Math.max(span,2*Math.abs(p[0])+1,2*Math.abs(p[1])+1)}
  span=Math.ceil(span);
  const px=W/span, cx=W/2, cy=H/2;
  g.strokeStyle='#2a323c';g.lineWidth=1;
  for(let m=-Math.floor(span/2);m<=Math.floor(span/2);m++){
    g.beginPath();g.moveTo(cx+m*px,0);g.lineTo(cx+m*px,H);g.stroke();
    g.beginPath();g.moveTo(0,cy+m*px);g.lineTo(W,cy+m*px);g.stroke();}
  g.strokeStyle='#8d99a6';g.beginPath();g.arc(cx,cy,0.3*px,0,7);g.stroke();     // 0.3 m circle (bench step 3)
  g.fillStyle='#8d99a6';g.font='13px system-ui';g.fillText('home',cx+6,cy-6);g.fillText(span+' m across',8,H-8);
  g.strokeStyle='#2f8fd8';g.lineWidth=2;g.beginPath();
  trail.forEach((p,i)=>{const x=cx+p[1]*px,y=cy-p[0]*px;i?g.lineTo(x,y):g.moveTo(x,y)});g.stroke();
  if(trail.length){const p=trail[trail.length-1];g.fillStyle=s&&s.ready?'#1fa35a':'#c9353f';
    g.beginPath();g.arc(cx+p[1]*px,cy-p[0]*px,7,0,7);g.fill();}
}
const KNOWN=['ready','status','reason','quality','north_m','east_m','distance_m','forward_m','right_m','height_change_m','engaged','sticks','state_age_s','frame_age_s','fps','video_clients','stale_after_s'];
async function tick(){
  let s=null;
  try{const r=await fetch('/state',{cache:'no-store'});s=await r.json();}catch(e){}
  const fresh=s&&s.state_age_s!==null&&s.state_age_s<(s.stale_after_s||1);
  $('lost').style.display=s?'none':'block';
  const b=$('badge');
  if(!fresh){b.className='badge nodata';b.textContent=s?'NO DATA':'NO CONNECTION';}
  else if(s.ready){b.className='badge ready';b.textContent='READY';}
  else{b.className='badge notready';b.textContent='NOT READY';}
  if(s){
    $('engaged').textContent=s.engaged?'ENGAGED (CH10 on)':'switch off';$('engaged').className='chip'+(s.engaged?' on':'');
    $('status').textContent=(s.status||'-')+(s.quality!==undefined&&s.quality!==null?'  q='+s.quality:'');
    $('fps').textContent=(s.fps||0).toFixed(1)+' fps';
    $('reason').textContent=s.ready?'':(s.reason||'');
    const ok=fresh&&s.ready;
    $('dist').textContent=ok?Number(s.distance_m).toFixed(2):'-';
    $('fwd').textContent=ok&&s.forward_m!==undefined?fmt(s.forward_m):'-';
    $('right').textContent=ok&&s.right_m!==undefined?fmt(s.right_m):'-';
    $('dh').textContent=ok&&s.height_change_m!==null&&s.height_change_m!==undefined?fmt(s.height_change_m):'-';
    if(ok&&s.forward_m!==undefined&&s.right_m!==undefined){trail.push([s.forward_m,s.right_m]);if(trail.length>3000)trail.shift();}
    const rows=[];
    const show=v=>typeof v==='number'?(Number.isInteger(v)?String(v):v.toFixed(3)):JSON.stringify(v);
    if(s.sticks){const names=['roll','pitch','throttle','yaw'].slice(0,s.sticks.length);
      rows.push(['our stick request ('+names.join(', ')+')',s.sticks.join(', ')]);}
    for(const k of Object.keys(s))if(!KNOWN.includes(k))rows.push([k.replace(/_/g,' '),show(s[k])]);
    rows.push(['picture age / estimate age',(s.frame_age_s??'-')+' s / '+(s.state_age_s??'-')+' s']);
    $('more').innerHTML=rows.map(r=>'<tr><td>'+r[0]+'</td><td>'+r[1]+'</td></tr>').join('');
  }
  draw(s);
}
setInterval(tick,150);tick();
</script></body></html>
"""


# ---------------- demo on a laptop (no drone needed) ----------------
def _demo(args):
    if cv2 is None:
        raise SystemExit("the demo needs OpenCV (pip install opencv-python-headless)")
    live = LiveView(port=args.port)
    if live.server is None:
        raise SystemExit("could not start the live view")
    print(f"Demo running. Open http://localhost:{live.server.server_address[1]}   (Ctrl+C to stop)")
    cap = None
    if args.camera is not None:
        cap = cv2.VideoCapture(int(args.camera) if str(args.camera).isdigit() else args.camera)
    rng = np.random.default_rng(1)
    floor = cv2.GaussianBlur(rng.normal(128, 60, (1400, 1800)).astype(np.float32), (0, 0), 3)
    floor = np.clip(floor, 0, 255).astype(np.uint8)
    t0 = time.monotonic()
    try:
        while True:
            t = time.monotonic() - t0
            # a 3 m square loop at 1 m height, 12 s per lap (like bench step 3), 2 s "covered" every lap
            lap = (t % 12.0) / 12.0
            side, f = int(lap * 4), (lap * 4) % 1.0
            fwd, right = [(f * 1.5, 0), (1.5, f * 0.75), (1.5 - f * 1.5, 0.75), (0, 0.75 - f * 0.75)][side]
            covered = 7.0 <= t % 12.0 < 9.0
            if cap is not None:
                ok, img = cap.read()
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if ok else np.zeros((360, 640), np.uint8)
                gray = cv2.resize(gray, (640, 360))
            else:
                x, y = int(500 + right * 200), int(400 - fwd * 200)
                gray = cv2.resize(floor[y:y + 540, x:x + 960], (640, 360))
            if covered:
                gray = (gray // 12).astype(np.uint8)
            live.frame(gray)
            live.state(ready=not covered, status="LOST" if covered else "OK",
                       reason="picture too dark (camera covered)" if covered else "",
                       quality=0 if covered else 80, north_m=fwd, east_m=right, yaw0=0.0,
                       height_change_m=0.0, engaged=False, sticks=[1500, 1500, 1300, 1500], demo=True)
            time.sleep(1 / 30)
    except KeyboardInterrupt:
        pass
    finally:
        live.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Live view page (runs inside vision_hold.py; --demo to try it alone)")
    ap.add_argument("--demo", action="store_true", help="show a moving synthetic floor (or --camera) without the drone")
    ap.add_argument("--camera", default=None, help="demo with a webcam number or a video file instead")
    ap.add_argument("--port", type=int, default=PORT)
    a = ap.parse_args()
    if a.demo:
        _demo(a)
    else:
        ap.print_help()
