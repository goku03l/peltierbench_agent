#!/usr/bin/env python3
"""Offline simulator: the v2 firmware control logic against the fitted thermal model.

    python sim/bench_sim.py --sp 20 --start 24.5
    python sim/bench_sim.py --sp 20 --start 24.5 --set ki=1.0 kd=10 usedead=0
    python sim/bench_sim.py --sp 20 --start 24.5 --sweep ki=0.3,0.6,1.2 --sweep kd=0,20

Parameter names match the firmware ('params' command). Output is JSON, one object per run.
Model limits: no hot-side warm-up and (by default) no sensor lag, so real runs will look worse than simulated ones.
"""
import argparse, itertools, json, math, os, random, sys

sys.path.insert(0, os.path.dirname(__file__))
from metrics import run_metrics, count_cycles, score  # noqa: E402

HERE = os.path.dirname(__file__)
HARD_COOL, HARD_HEAT = 220.0, 100.0

DEFAULTS = dict(kp=60.0, ki=0.6, kd=20.0, alpha=0.95, mincool=40, minheat=35, maxcool=220, maxheat=100,
                starterr=0.30, deadband=0.10, usedead=1, dwell=30, spslew=0.5, dfilt=0.70,
                resetflip=0, idledecay=0, wdrift=2.0)
V1 = dict(DEFAULTS, ki=0.15, kd=150.0, alpha=0.0, mincool=60, minheat=50, resetflip=1, idledecay=1)


def load_plant():
    with open(os.path.join(HERE, "plant.json")) as f:
        return json.load(f)["b"]


class Controller:
    """Line-by-line port of computeZoneControl / polarityInterlock / runPID's fast slew (single zone)."""

    def __init__(self, p):
        self.p = p
        self.I = 0.0; self.spr = None; self.pvf = None
        self.active = not p["usedead"]; self.since = 0.0; self.es = 0
        self.u = 0.0; self.target = 0; self.lastdir = 0; self.zero = 0.0; self.pidlast = 0.0; self.pvsm = None

    def _decide(self, sp, pv, dt, now):
        p = self.p
        if self.spr is None: self.spr = sp
        if abs(sp - self.spr) > 1: self.spr = sp
        else: self.spr = min(self.spr + p["spslew"] * dt, sp) if sp > self.spr else max(self.spr - p["spslew"] * dt, sp)
        e = self.spr - pv; ae = abs(e)
        if self.pvf is None: self.pvf = pv
        prev = self.pvf; self.pvf = p["dfilt"] * self.pvf + (1 - p["dfilt"]) * pv; dm = (self.pvf - prev) / dt
        es = 1 if e > 0 else -1
        if p["resetflip"] and self.es != 0 and es != self.es: self.I = 0.0
        self.es = es
        if p["usedead"]:
            inst = now - self.since; far = ae >= 1
            if not self.active:
                if ae > p["starterr"] and (inst >= p["dwell"] or far): self.active = True; self.since = now
            elif ae < p["deadband"] and inst >= p["dwell"]:
                self.active = False; self.since = now
        else:
            self.active = True
        if not self.active:
            if p["idledecay"]: self.I *= 0.98
            return 0
        P = p["kp"] * e; D = -p["kd"] * dm
        omin, omax = -min(p["maxcool"], HARD_COOL), min(p["maxheat"], HARD_HEAT)
        u = max(omin, min(omax, P + self.I + D))
        sath = u >= omax - 1e-6 and e > 0; satl = u <= omin + 1e-6 and e < 0
        if not sath and not satl: self.I = max(-200.0, min(200.0, self.I + p["ki"] * e * dt))
        elif p["idledecay"]: self.I *= 0.98
        if e <= -0.1 and u > 0: u = 0
        if e >= 0.1 and u < 0: u = 0
        if u < 0: return -int(min(255, max(int(p["mincool"]), -u)))
        if u > 0: return int(min(255, max(int(p["minheat"]), u)))
        return 0

    def step(self, sp, raw, now):
        pv = raw
        a = self.p["alpha"]
        if a > 0:
            self.pvsm = raw if self.pvsm is None else a * self.pvsm + (1 - a) * raw
            pv = self.pvsm
        if self.pidlast == 0: self.pidlast = now or 1e-6
        elif now - self.pidlast >= 1.0:
            dt = min(now - self.pidlast, 3.0); self.pidlast = now
            t = self._decide(sp, pv, dt, now)
            if self.u != 0: self.lastdir = -1 if self.u < 0 else 1; self.zero = 0
            elif self.zero == 0: self.zero = now or 1e-3
            w = -1 if t < 0 else (1 if t > 0 else 0)
            if not (w == 0 or self.lastdir == 0 or w == self.lastdir):
                if self.u == 0 and now - self.zero >= 10: self.lastdir = w
                else: self.I = 0.0; t = 0
            self.target = t
        mx = 25.0 * 0.1
        self.u = min(self.u + mx, self.target) if self.target > self.u else max(self.u - mx, self.target)
        return self.u


def simulate(p, sp, start, minutes=120, room=26.7, cool_k=1.0, lag=0.0, sigma=0.16, spike=0.01, seed=7):
    b = load_plant()
    rng = random.Random(seed); c = Controller(p)
    T = S = start; n = int(minutes * 600)
    t, temp, drive = [], [], []
    for k in range(n):
        now = k * 0.1
        S = S + (T - S) * 0.1 / lag if lag > 0 else T
        meas = S + rng.gauss(0, sigma) + (rng.choice([-1.2, 1.2]) if rng.random() < spike else 0)
        u = c.step(sp, meas, now)
        cool = -u if u < 0 else 0; heat = u if u > 0 else 0
        ce = min(b[2] + b[4] * T, -1e-6) * cool_k
        T += (-b[1] * room + b[1] * T + cool * ce + heat * b[3]) * 0.1
        if k % 10 == 0: t.append(now / 60); temp.append(T); drive.append(u)
    return t, temp, drive


def evaluate(p, sp, start, minutes, **kw):
    t, temp, drive = simulate(p, sp, start, minutes, **kw)
    m = run_metrics(t, temp, sp)
    m["cycles"] = count_cycles(t, drive)
    m["score"] = score(m, minutes)
    return m


def parse_kv(items):
    out = {}
    for it in items or []:
        k, v = it.split("=", 1)
        out[k.strip().lower()] = float(v)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sp", type=float, default=20); ap.add_argument("--start", type=float, default=24.5)
    ap.add_argument("--minutes", type=float, default=120); ap.add_argument("--preset", choices=["v2", "v1"], default="v2")
    ap.add_argument("--set", nargs="*", help="param=value overrides"); ap.add_argument("--sweep", action="append", help="param=a,b,c")
    ap.add_argument("--lag", type=float, default=0); ap.add_argument("--sigma", type=float, default=0.16)
    ap.add_argument("--room", type=float, default=26.7); ap.add_argument("--seeds", type=int, default=1)
    a = ap.parse_args()
    base = dict(V1 if a.preset == "v1" else DEFAULTS); base.update(parse_kv(a.set))
    unknown = set(base) - set(DEFAULTS)
    if unknown: sys.exit("unknown parameters: %s" % sorted(unknown))
    sweeps = [(s.split("=")[0].lower(), [float(x) for x in s.split("=")[1].split(",")]) for s in (a.sweep or [])]
    combos = itertools.product(*[v for _, v in sweeps]) if sweeps else [()]
    for combo in combos:
        p = dict(base); p.update({k: v for (k, _), v in zip(sweeps, combo)})
        ms = [evaluate(p, a.sp, a.start, a.minutes, lag=a.lag, sigma=a.sigma, room=a.room, seed=7 + s) for s in range(a.seeds)]
        m = ms[0] if len(ms) == 1 else {k: round(sum(x[k] for x in ms if x.get(k) is not None) / len(ms), 3) for k in ms[0] if isinstance(ms[0][k], (int, float))}
        print(json.dumps({"params": {k: p[k] for k in p if p[k] != DEFAULTS[k]}, "sp": a.sp, "start": a.start, "metrics": m}))


if __name__ == "__main__":
    main()
