#!/usr/bin/env python3
"""Local bench daemon: one process owns the rig connection (USB serial or WiFi TCP), serves the live viewer, and runs
bounded experiments. The CLI subcommands talk to the daemon over http://127.0.0.1:8765.

    python tools/bench_local.py daemon --serial COM5          # or: --tcp motor-logger.local
    python tools/bench_local.py status
    python tools/bench_local.py cmd "get ki"
    python tools/bench_local.py experiment --tag step20 --sp 20 --minutes 60 --set ki=0.6 usedead=0 --pre-sp 24 --pre-minutes 20
    python tools/bench_local.py wait            # blocks until the running experiment finishes, prints the result
    python tools/bench_local.py abort           # stops the experiment and the Peltier

Safety, on top of the firmware's own guards:
  - whitelisted commands and parameters with bounds; setpoint 15..40 C; experiments max 180 min
  - the rig's dead-man timer (5 min) is armed for every experiment and refreshed every minute, so the Peltier stops
    if this laptop sleeps, crashes or loses the link (needs firmware 2.0.3 or newer; older firmware is refused)
  - abort and stop if the water leaves [tmin, tmax], a FAULT appears, or live data goes quiet for 2 minutes
  - the rig is stopped at the end unless --leave is given
The HTTP server only listens on 127.0.0.1.
"""
import argparse, collections, csv, datetime as dt, json, math, os, socket, sys, threading, time, urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "sim"))
from metrics import run_metrics, count_cycles, score  # noqa: E402

PORT = 8765
SP_RANGE = (15.0, 40.0)
MIN_FW = (2, 0, 3)
PARAM_BOUNDS = {
    "kp": (0, 500), "ki": (0, 5), "kd": (0, 1000), "alpha": (0, 0.99), "mincool": (0, 120), "minheat": (0, 120),
    "maxcool": (0, 220), "maxheat": (0, 100), "starterr": (0.05, 2), "deadband": (0, 1), "usedead": (0, 1),
    "dwell": (0, 120), "spslew": (0.05, 2), "dfilt": (0, 0.95), "resetflip": (0, 1), "idledecay": (0, 1), "wdrift": (0.5, 10),
}
SAFE_CMDS = {"status", "guard", "pid", "params", "get", "set", "save", "defaults", "sp", "a", "s", "clear", "hb",
             "deadman", "tag", "ver", "ip", "time"}


def f(v):
    try:
        x = float(v)
        return None if math.isnan(x) else x
    except (TypeError, ValueError):
        return None


def parse_live(line):
    """'Time:...,PWM_pct:10.2,Direction:-1,Temp1_C:nan,...,Phase:HOLD' -> dict, or None."""
    if not line.startswith("Time:") and not line.startswith("PWM_pct:"):
        return None
    d = {}
    for tok in line.split(","):
        if ":" in tok:
            k, v = tok.split(":", 1); d[k.strip()] = v.strip()
    if "PWM_pct" not in d: return None
    return dict(time=d.get("Time"), pwm=f(d.get("PWM_pct")), dir=int(f(d.get("Direction")) or 0), t1=f(d.get("Temp1_C")),
                t2=f(d.get("Temp2_C")), t3=f(d.get("Temp3_C")), t3f=f(d.get("Temp3f_C")) if "Temp3f_C" in d else f(d.get("Temp3_C")),
                sp=f(d.get("Setpoint_C")), ph=d.get("Phase"))


class SerialTransport:
    def __init__(self, port, baud=115200):
        import serial
        self.s = serial.Serial(); self.s.port = port; self.s.baudrate = baud; self.s.timeout = 0.3
        self.s.dtr = False; self.s.rts = False        # opening the port must not reset the board
        self.s.open(); self.name = "USB " + port
    def readline(self):
        b = self.s.readline()
        return b.decode("utf-8", "replace").strip() if b else None
    def write(self, text): self.s.write((text + "\n").encode())
    def close(self): self.s.close()


class TcpTransport:
    def __init__(self, host, port=3333):
        self.s = socket.create_connection((host, port), timeout=8); self.s.settimeout(0.3)
        self.buf = b""; self.name = "WiFi %s:%d" % (host, port)
    def readline(self):
        end = time.time() + 0.3
        while b"\n" not in self.buf and time.time() < end:
            try:
                d = self.s.recv(4096)
                if not d: raise ConnectionError("closed")
                self.buf += d
            except socket.timeout: break
        if b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            return line.decode("utf-8", "replace").strip()
        return None
    def write(self, text): self.s.sendall((text + "\n").encode())
    def close(self): self.s.close()


class Rig:
    def __init__(self, serial_port=None, tcp_host=None):
        self.serial_port, self.tcp_host = serial_port, tcp_host
        self.t = None; self.connected = False; self.name = "-"
        self.tele = None; self.tele_at = 0.0
        self.points = collections.deque(maxlen=3 * 3600)      # 1 Hz history for the viewer
        self.events = collections.deque(maxlen=80)
        self.replies = collections.deque(maxlen=200); self.reply_at = 0.0
        self.params = {}; self.fw = None; self.lock = threading.Lock(); self.stop = False
        self.last_point = 0.0
        threading.Thread(target=self._loop, daemon=True).start()

    def _open(self):
        return SerialTransport(self.serial_port) if self.serial_port else TcpTransport(self.tcp_host)

    def _loop(self):
        while not self.stop:
            try:
                self.t = self._open(); self.connected = True; self.name = self.t.name
                self.events.append((time.time(), "# connected via " + self.name))
                threading.Thread(target=self._hello, daemon=True).start()
                while not self.stop:
                    line = self.t.readline()
                    if line: self._handle(line)
            except Exception as e:
                self.events.append((time.time(), "# link lost: %s" % e))
            self.connected = False
            try:
                if self.t: self.t.close()
            except Exception: pass
            time.sleep(5)

    def _hello(self):
        time.sleep(3)
        if time.time() - self.tele_at > 3: self.send("q")           # live data was switched off: toggle it on
        self.cmd("ver", 3); self.cmd("params", 3)

    def _handle(self, line):
        if line.startswith("#"):
            self.replies.append(line); self.reply_at = time.time()
            if not line.startswith("# Status:"): self.events.append((time.time(), line))
            if line.startswith("# fw "): self.fw = line.split()[2]
            if line.startswith("# params:"):
                for kv in line[len("# params:"):].split():
                    if "=" in kv:
                        k, v = kv.split("=", 1); self.params[k] = f(v)
            return
        d = parse_live(line)
        if d:
            now = time.time(); self.tele = d; self.tele_at = now
            if now - self.last_point >= 1.0:
                self.last_point = now
                self.points.append((now, d["t3f"], d["sp"], d["pwm"], d["dir"], d["ph"]))

    def send(self, text):
        with self.lock:
            if not self.connected: raise RuntimeError("rig not connected")
            self.t.write(text)

    def cmd(self, text, wait=6.0):
        head = text.split()[0].lower()
        if head not in SAFE_CMDS: raise ValueError("command not allowed: %s" % head)
        if head == "sp":
            v = float(text.split()[1])
            if not SP_RANGE[0] <= v <= SP_RANGE[1]: raise ValueError("setpoint %.1f outside %s" % (v, SP_RANGE))
        if head == "set":
            _, name, val = text.split(); lo, hi = PARAM_BOUNDS[name.lower()]
            if not lo <= float(val) <= hi: raise ValueError("%s=%s outside %s..%s" % (name, val, lo, hi))
        start = len(self.replies); self.replies.clear(); self.reply_at = 0.0
        self.send(text)
        t0 = time.time()
        while time.time() - t0 < wait:
            time.sleep(0.15)
            if self.reply_at and time.time() - self.reply_at > 1.0: break
        out = [l for l in list(self.replies) if not l.startswith("# Status:")]
        return out


def fw_ok(fw):
    try:
        v = tuple(int(x) for x in fw.split("-")[0].split(".")); return v >= MIN_FW
    except Exception:
        return False


class Experiments:
    def __init__(self, rig):
        self.rig = rig; self.running = False; self.abort_flag = False
        self.status = {"running": False}; self.last = None

    def start(self, a):
        if self.running: raise RuntimeError("an experiment is already running")
        if not self.rig.connected: raise RuntimeError("rig not connected")
        if not SP_RANGE[0] <= a["sp"] <= SP_RANGE[1]: raise ValueError("setpoint out of range")
        if a["minutes"] > 180: raise ValueError("experiments are limited to 180 min")
        for k, v in a["set"].items():
            if k not in PARAM_BOUNDS or not PARAM_BOUNDS[k][0] <= float(v) <= PARAM_BOUNDS[k][1]: raise ValueError("bad parameter %s=%s" % (k, v))
        if not fw_ok(self.rig.fw or ""): raise RuntimeError("firmware %s is older than 2.0.3, so the dead-man timer cannot be used; flash first" % self.rig.fw)
        self.running = True; self.abort_flag = False
        threading.Thread(target=self._run, args=(a,), daemon=True).start()

    def abort(self):
        self.abort_flag = True
        try: self.rig.cmd("s", 3)
        except Exception: pass

    def _phase(self, label, sp, minutes, tmin, tmax, rows):
        rig = self.rig
        rig.cmd("sp %.1f" % sp)
        t0 = time.time(); last_hb = t0; last_row = 0.0; fault_since = None
        while time.time() - t0 < minutes * 60:
            time.sleep(1.0)
            now = time.time()
            self.status.update(phase=label, elapsed_min=round((now - t0) / 60, 1), of_min=minutes, sp=sp)
            if self.abort_flag: return "aborted by user"
            if not rig.connected or now - rig.tele_at > 120: return "live data silent for 2 min"
            tel = rig.tele
            if tel and rig.tele_at > last_row:
                last_row = rig.tele_at
                rows.append(dict(phase_label=label, t_min=round((now - t0) / 60, 4), time=tel["time"], t3=tel["t3"], t3f=tel["t3f"],
                                 sp=tel["sp"], pwm=tel["pwm"], dir=tel["dir"], ph=tel["ph"]))
                v = tel["t3f"]
                if v is not None and not tmin <= v <= tmax: return "water %.1f C left the safe band %.0f..%.0f" % (v, tmin, tmax)
                if tel["ph"] == "FAULT": return "rig reported FAULT"
                if tel["ph"] == "SENSOR_FAULT":
                    fault_since = fault_since or now
                    if now - fault_since > 60: return "water sensor fault for 60 s"
                else: fault_since = None
            if now - last_hb >= 60:
                try: rig.cmd("hb", 3)
                except Exception: pass
                last_hb = now
        return None

    def _run(self, a):
        rig = self.rig; rows = []; abort = None
        exp_id = dt.datetime.now().strftime("%Y%m%dT%H%M%S") + "-" + a["tag"]
        self.status = {"running": True, "id": exp_id, "tag": a["tag"], "phase": "setup"}
        try:
            rig.cmd("tag %s" % a["tag"][:14]); rig.cmd("deadman 5")
            if a.get("pre_sp") is not None:
                for k, v in (a.get("pre_set") or {}).items(): rig.cmd("set %s %s" % (k, v))
                abort = self._phase("pre", a["pre_sp"], a["pre_minutes"], a["tmin"], a["tmax"], rows)
            if not abort:
                for k, v in a["set"].items(): rig.cmd("set %s %s" % (k, v))
                abort = self._phase("main", a["sp"], a["minutes"], a["tmin"], a["tmax"], rows)
        except Exception as e:
            abort = "error: %s" % e
        finally:
            try:
                if abort or not a.get("leave"): rig.cmd("s", 3)
                rig.cmd("deadman 0", 3)
            except Exception: pass
        main = [r for r in rows if r["phase_label"] == "main" and r["t3f"] is not None]
        res = {"id": exp_id, "tag": a["tag"], "when": dt.datetime.now().isoformat(timespec="seconds"), "fw": rig.fw, "link": rig.name,
               "sp": a["sp"], "minutes": a["minutes"], "set": a["set"], "aborted": abort}
        if main:
            t = [r["t_min"] for r in main]; temp = [r["t3f"] for r in main]
            m = run_metrics(t, temp, a["sp"])
            if m.get("samples", 0) >= 10:
                m["cycles"] = count_cycles(t, [(-1 if r["dir"] == 1 else 1) * (r["pwm"] or 0) for r in main])
                m["start_c"] = round(temp[0], 2); m["score"] = None if abort else score(m, a["minutes"])
            res["metrics"] = m
        os.makedirs(os.path.join(ROOT, "data", "runs"), exist_ok=True); os.makedirs(os.path.join(ROOT, "ledger"), exist_ok=True)
        if rows:
            with open(os.path.join(ROOT, "data", "runs", exp_id + ".csv"), "w", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
            res["csv"] = "data/runs/%s.csv" % exp_id
        res["events"] = [e[1] for e in list(rig.events)[-15:]]
        with open(os.path.join(ROOT, "ledger", "experiments.jsonl"), "a") as fh: fh.write(json.dumps(res) + "\n")
        self.last = res; self.running = False; self.status = {"running": False, "last": exp_id}


def make_handler(rig, exps):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def _json(self, obj, code=200):
            b = json.dumps(obj).encode(); self.send_response(code)
            self.send_header("Content-Type", "application/json"); self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)
        def _file(self, path, ctype):
            try: b = open(path, "rb").read()
            except OSError: return self._json({"error": "not found"}, 404)
            self.send_response(200); self.send_header("Content-Type", ctype); self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)
        def do_GET(self):
            p = self.path.split("?")[0]
            if p == "/api/state":
                now = time.time()
                return self._json({"connected": rig.connected, "link": rig.name, "fw": rig.fw, "params": rig.params, "tele": rig.tele,
                                   "age": None if not rig.tele_at else round(now - rig.tele_at, 1),
                                   "points": [[round(x[0]), x[1], x[2], x[3], x[4]] for x in list(rig.points)[-2400:]],
                                   "events": [[round(e[0]), e[1]] for e in list(rig.events)[-40:]], "exp": exps.status, "last": exps.last})
            if p in ("/", "/index.html"): return self._file(os.path.join(ROOT, "viewer", "local.html"), "text/html; charset=utf-8")
            if p == "/ledger/experiments.jsonl": return self._file(os.path.join(ROOT, "ledger", "experiments.jsonl"), "text/plain; charset=utf-8")
            if p == "/ledger/NOTES.md": return self._file(os.path.join(ROOT, "ledger", "NOTES.md"), "text/plain; charset=utf-8")
            self._json({"error": "not found"}, 404)
        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try: body = json.loads(self.rfile.read(n) or b"{}")
            except ValueError: return self._json({"error": "bad json"}, 400)
            try:
                if self.path == "/api/cmd": return self._json({"lines": rig.cmd(body["text"])})
                if self.path == "/api/experiment":
                    a = dict(tag="".join(c for c in body["tag"] if c.isalnum() or c in "-_")[:40] or "run", sp=float(body["sp"]),
                             minutes=float(body.get("minutes", 60)), set={k.lower(): v for k, v in (body.get("set") or {}).items()},
                             pre_sp=None if body.get("pre_sp") in (None, "") else float(body["pre_sp"]), pre_minutes=float(body.get("pre_minutes", 20)),
                             pre_set=body.get("pre_set") or {}, tmin=float(body.get("tmin", 10)), tmax=float(body.get("tmax", 42)), leave=bool(body.get("leave")))
                    exps.start(a); return self._json({"started": True})
                if self.path == "/api/abort": exps.abort(); return self._json({"aborted": True})
            except (ValueError, RuntimeError, KeyError) as e:
                return self._json({"error": str(e)}, 400)
            self._json({"error": "not found"}, 404)
    return H


def api(path, body=None):
    req = urllib.request.Request("http://127.0.0.1:%d%s" % (PORT, path), data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=30) as r: return json.loads(r.read())
    except urllib.error.HTTPError as e:
        return json.loads(e.read())
    except OSError:
        sys.exit("the bench daemon is not running; start it with: python tools/bench_local.py daemon --serial COMx")


def kv(items):
    out = {}
    for it in items or []:
        k, v = it.split("=", 1); out[k.lower()] = float(v)
    return out


def main():
    ap = argparse.ArgumentParser(); sub = ap.add_subparsers(dest="mode", required=True)
    d = sub.add_parser("daemon"); g = d.add_mutually_exclusive_group(required=True)
    g.add_argument("--serial"); g.add_argument("--tcp")
    sub.add_parser("status"); sub.add_parser("abort"); sub.add_parser("wait")
    c = sub.add_parser("cmd"); c.add_argument("text")
    e = sub.add_parser("experiment")
    e.add_argument("--tag", required=True); e.add_argument("--sp", type=float, required=True); e.add_argument("--minutes", type=float, default=60)
    e.add_argument("--set", nargs="*"); e.add_argument("--pre-sp", type=float); e.add_argument("--pre-minutes", type=float, default=20)
    e.add_argument("--pre-set", nargs="*"); e.add_argument("--tmin", type=float, default=10); e.add_argument("--tmax", type=float, default=42)
    e.add_argument("--leave", action="store_true")
    a = ap.parse_args()
    if a.mode == "daemon":
        rig = Rig(serial_port=a.serial, tcp_host=a.tcp); exps = Experiments(rig)
        srv = ThreadingHTTPServer(("127.0.0.1", PORT), make_handler(rig, exps))
        print("viewer: http://127.0.0.1:%d   rig link: %s" % (PORT, a.serial or a.tcp), flush=True)
        try: srv.serve_forever()
        except KeyboardInterrupt: pass
        return
    if a.mode == "status":
        s = api("/api/state"); s.pop("points", None); s["events"] = s["events"][-8:]; print(json.dumps(s, indent=1))
    elif a.mode == "cmd":
        r = api("/api/cmd", {"text": a.text}); print("\n".join(r.get("lines", [])) if "lines" in r else r)
    elif a.mode == "abort": print(api("/api/abort", {}))
    elif a.mode == "experiment":
        print(api("/api/experiment", dict(tag=a.tag, sp=a.sp, minutes=a.minutes, set=kv(a.set), pre_sp=a.pre_sp, pre_minutes=a.pre_minutes,
                                          pre_set=kv(a.pre_set), tmin=a.tmin, tmax=a.tmax, leave=a.leave)))
    elif a.mode == "wait":
        while True:
            s = api("/api/state")
            if not s["exp"].get("running"): print(json.dumps(s.get("last"), indent=1)); break
            time.sleep(10)


if __name__ == "__main__":
    main()
