#!/usr/bin/env python3
"""Talk to the rig over MQTT. Used by the experiment workflow and by the agent.

    python tools/bench.py status
    python tools/bench.py cmd "get ki"
    python tools/bench.py watch --seconds 30
    python tools/bench.py experiment --tag step20 --sp 20 --minutes 60 --set ki=0.6 kd=20 \
        [--pre-sp 24 --pre-minutes 20] [--tmin 10 --tmax 42]

Environment: MQTT_HOST, MQTT_PORT (8883), MQTT_USER, MQTT_PASS, MQTT_BASE.

Safety, enforced here on top of the firmware's own guards:
  - only whitelisted commands and parameters, with bounds
  - setpoint limited to 15..40 C
  - a 5-minute dead-man timer on the rig, refreshed every minute, so the Peltier stops if this process dies
  - abort (and send 's') if the water leaves [tmin, tmax], a FAULT appears, or telemetry goes quiet for 2 minutes
  - the rig is stopped at the end unless --leave is given
"""
import argparse, csv, datetime as dt, json, os, queue, ssl, sys, threading, time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
from metrics import run_metrics, count_cycles, score  # noqa: E402

import paho.mqtt.client as mqtt  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")
SP_RANGE = (15.0, 40.0)
PARAM_BOUNDS = {  # mirrors the firmware table; the firmware enforces it again
    "kp": (0, 500), "ki": (0, 5), "kd": (0, 1000), "alpha": (0, 0.99), "mincool": (0, 120), "minheat": (0, 120),
    "maxcool": (0, 220), "maxheat": (0, 100), "starterr": (0.05, 2), "deadband": (0, 1), "usedead": (0, 1),
    "dwell": (0, 120), "spslew": (0.05, 2), "dfilt": (0, 0.95), "resetflip": (0, 1), "idledecay": (0, 1), "wdrift": (0.5, 10),
}
SAFE_CMDS = {"status", "guard", "pid", "params", "get", "set", "save", "defaults", "sp", "a", "s", "clear", "hb",
             "deadman", "tag", "ver", "ip", "time"}


class Rig:
    def __init__(self):
        self.base = os.environ.get("MQTT_BASE", "peltier/rig")
        self.tele = None; self.tele_at = 0.0; self.online = None
        self.resp = queue.Queue(); self.events = []
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="bench-%d" % os.getpid())
        c.tls_set(cert_reqs=ssl.CERT_REQUIRED)
        c.username_pw_set(os.environ["MQTT_USER"], os.environ["MQTT_PASS"])
        c.on_connect = self._on_connect; c.on_message = self._on_message
        c.connect(os.environ["MQTT_HOST"], int(os.environ.get("MQTT_PORT", "8883")), keepalive=30)
        c.loop_start(); self.c = c

    def _on_connect(self, c, u, f, rc, props=None):
        for t in ("telemetry", "resp", "events", "status"):
            c.subscribe("%s/%s" % (self.base, t), qos=1)

    def _on_message(self, c, u, m):
        name = m.topic.rsplit("/", 1)[-1]; text = m.payload.decode("utf-8", "replace")
        if name == "telemetry":
            try: self.tele = json.loads(text); self.tele_at = time.time()
            except ValueError: pass
        elif name == "resp": self.resp.put(text)
        elif name == "events": self.events.append((time.time(), text))
        elif name == "status": self.online = text == "1"

    def cmd(self, text, wait=8.0):
        head = text.split()[0].lower()
        if head not in SAFE_CMDS: raise SystemExit("command not allowed from the bench tool: %s" % head)
        if head == "sp":
            v = float(text.split()[1])
            if not SP_RANGE[0] <= v <= SP_RANGE[1]: raise SystemExit("setpoint %.1f outside %s" % (v, SP_RANGE))
        if head == "set":
            _, name, val = text.split(); lo, hi = PARAM_BOUNDS[name.lower()]
            if not lo <= float(val) <= hi: raise SystemExit("%s=%s outside %s..%s" % (name, val, lo, hi))
        while not self.resp.empty(): self.resp.get_nowait()
        self.c.publish("%s/cmd" % self.base, text, qos=1)
        lines, end = [], time.time() + wait
        while time.time() < end:
            try: lines.append(self.resp.get(timeout=0.5)); end = min(end, time.time() + 1.5)
            except queue.Empty:
                if lines: break
        return lines

    def wait_telemetry(self, timeout=45):
        end = time.time() + timeout
        while time.time() < end:
            if self.tele and time.time() - self.tele_at < 10: return self.tele
            time.sleep(0.5)
        raise SystemExit("no live telemetry from the rig (is it online? status=%s)" % self.online)

    def close(self): self.c.loop_stop(); self.c.disconnect()


def parse_sets(items):
    out = []
    for it in items or []:
        k, v = it.split("=", 1); k = k.lower()
        if k not in PARAM_BOUNDS: raise SystemExit("unknown parameter %s" % k)
        lo, hi = PARAM_BOUNDS[k]
        if not lo <= float(v) <= hi: raise SystemExit("%s=%s outside %s..%s" % (k, v, lo, hi))
        out.append((k, v))
    return out


def run_phase(rig, label, sp, minutes, tmin, tmax, rows, hb_every=60):
    """Hold a setpoint for `minutes`, recording 1 Hz telemetry. Returns an abort reason or None."""
    print("[%s] sp=%.1f for %.0f min" % (label, sp, minutes), flush=True)
    for l in rig.cmd("sp %.1f" % sp): print("  ", l, flush=True)
    t0 = time.time(); last_hb = t0; last_row = 0.0; fault_since = None
    while time.time() - t0 < minutes * 60:
        time.sleep(1.0)
        now = time.time()
        if now - rig.tele_at > 120: return "telemetry silent for 2 min"
        tel = rig.tele
        if tel and rig.tele_at > last_row:
            last_row = rig.tele_at
            rows.append(dict(phase_label=label, t_min=(now - t0) / 60, time=tel.get("time"), t3=tel.get("t3"), t3f=tel.get("t3f"),
                             sp=tel.get("sp"), pwm=tel.get("pwm"), dir=tel.get("dir"), ph=tel.get("ph"),
                             p=tel.get("p"), i=tel.get("i"), d=tel.get("d"), flt=tel.get("flt")))
            v = tel.get("t3f")
            if v is not None and not tmin <= v <= tmax: return "water %.1f C left the safe band %.0f..%.0f" % (v, tmin, tmax)
            if tel.get("ph") == "FAULT": return "rig reported FAULT"
            if tel.get("flt"):
                fault_since = fault_since or now
                if now - fault_since > 90: return "guard active for more than 90 s"
            else: fault_since = None
        if now - last_hb >= hb_every:
            rig.cmd("hb", wait=3); last_hb = now
    return None


def experiment(a):
    rig = Rig()
    try:
        tel = rig.wait_telemetry()
        print("rig online: fw=%s phase=%s t3f=%s sp=%s" % (tel.get("fw"), tel.get("ph"), tel.get("t3f"), tel.get("sp")))
        if tel.get("flt"): raise SystemExit("rig has an active fault; clear it by hand first")
        sets = parse_sets(a.set)
        exp_id = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S") + "-" + a.tag
        rig.cmd("tag %s" % a.tag[:14]); rig.cmd("deadman 5")
        rows = []; abort = None
        if a.pre_sp is not None:
            for k, v in parse_sets(a.pre_set): rig.cmd("set %s %s" % (k, v))
            abort = run_phase(rig, "pre", a.pre_sp, a.pre_minutes, a.tmin, a.tmax, rows)
        if not abort:
            for k, v in sets: print("  set", k, v, rig.cmd("set %s %s" % (k, v)))
            start_t = rig.tele.get("t3f") if rig.tele else None
            abort = run_phase(rig, "main", a.sp, a.minutes, a.tmin, a.tmax, rows)
        if abort or not a.leave:
            rig.cmd("s")
        rig.cmd("deadman 0")
        main = [r for r in rows if r["phase_label"] == "main" and r["t3f"] is not None]
        res = {"id": exp_id, "tag": a.tag, "utc": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds"),
               "fw": tel.get("fw"), "sp": a.sp, "minutes": a.minutes, "set": dict(sets), "aborted": abort}
        if main:
            t = [r["t_min"] for r in main]; temp = [r["t3f"] for r in main]
            m = run_metrics(t, temp, a.sp)
            if m.get("samples", 0) >= 10:
                drive = [(-1 if r["dir"] == 1 else 1) * (r["pwm"] or 0) for r in main]
                m["cycles"] = count_cycles(t, drive); m["start_c"] = temp[0]
                m["score"] = score(m, a.minutes) if not abort else None
            res["metrics"] = m
        os.makedirs(os.path.join(ROOT, "data", "runs"), exist_ok=True)
        path = os.path.join(ROOT, "data", "runs", exp_id + ".csv")
        if rows:
            with open(path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
            res["csv"] = "data/runs/%s.csv" % exp_id
        res["events"] = [e[1] for e in rig.events][-20:]
        os.makedirs(os.path.join(ROOT, "ledger"), exist_ok=True)
        with open(os.path.join(ROOT, "ledger", "experiments.jsonl"), "a") as f: f.write(json.dumps(res) + "\n")
        print(json.dumps(res, indent=1))
        return 2 if abort else 0
    finally:
        try: rig.cmd("deadman 0", wait=2)
        except Exception: pass
        rig.close()


def main():
    ap = argparse.ArgumentParser(); sub = ap.add_subparsers(dest="mode", required=True)
    sub.add_parser("status")
    p = sub.add_parser("cmd"); p.add_argument("text")
    p = sub.add_parser("watch"); p.add_argument("--seconds", type=int, default=20)
    p = sub.add_parser("experiment")
    p.add_argument("--tag", required=True); p.add_argument("--sp", type=float, required=True)
    p.add_argument("--minutes", type=float, default=60); p.add_argument("--set", nargs="*")
    p.add_argument("--pre-sp", type=float); p.add_argument("--pre-minutes", type=float, default=20); p.add_argument("--pre-set", nargs="*")
    p.add_argument("--tmin", type=float, default=10.0); p.add_argument("--tmax", type=float, default=42.0)
    p.add_argument("--leave", action="store_true", help="keep regulating at the end instead of stopping the rig")
    a = ap.parse_args()
    if a.mode == "experiment":
        if not SP_RANGE[0] <= a.sp <= SP_RANGE[1]: sys.exit("setpoint out of range")
        if a.minutes > 180: sys.exit("experiments are limited to 180 min")
        sys.exit(experiment(a))
    rig = Rig()
    try:
        if a.mode == "status":
            print(json.dumps(rig.wait_telemetry(), indent=1)); print("online:", rig.online)
        elif a.mode == "cmd":
            for l in rig.cmd(a.text): print(l)
        elif a.mode == "watch":
            end = time.time() + a.seconds
            while time.time() < end:
                time.sleep(1)
                if rig.tele: print(rig.tele.get("time"), rig.tele.get("t3f"), rig.tele.get("sp"), rig.tele.get("pwm"), rig.tele.get("ph"))
    finally:
        rig.close()


if __name__ == "__main__":
    main()
