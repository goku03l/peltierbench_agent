#!/usr/bin/env python3
"""A stand-in for the ESP32 on TCP port 3333, for testing the daemon and the viewer without the real rig.
It speaks the same live-line and command format as the v2 firmware, and uses the simulator's plant and controller.

    python tools/fake_rig.py [--speed 10]      # then: python tools/bench_local.py daemon --tcp 127.0.0.1
"""
import argparse, os, random, socket, sys, threading, time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
import bench_sim as bs  # noqa: E402


class Fake:
    def __init__(self, speed):
        self.p = dict(bs.DEFAULTS); self.b = bs.load_plant(); self.c = bs.Controller(self.p)
        self.T = 24.5; self.sp = 22.0; self.on = True; self.rng = random.Random(1); self.speed = speed
        self.k = 0; self.tag = ""; self.dead_at = None; self.last_cmd = time.time(); self.live = True
        self.sock = None

    def step(self):
        now = self.k * 0.1; self.k += 1
        meas = self.T + self.rng.gauss(0, 0.16)
        u = self.c.step(self.sp, meas, now) if self.on else 0.0
        if not self.on: self.c.u = 0.0; self.c.target = 0
        cool = -u if u < 0 else 0; heat = u if u > 0 else 0
        ce = min(self.b[2] + self.b[4] * self.T, -1e-6)
        self.T += (-self.b[1] * 26.7 + self.b[1] * self.T + cool * ce + heat * self.b[3]) * 0.1
        return meas, u

    def phase(self):
        if not self.on: return "STOPPED"
        return "HOLD" if abs(self.T - self.sp) <= 0.5 else "RAMPING"

    def handle(self, text):
        self.last_cmd = time.time()
        t = text.strip(); lo = t.lower(); h = lo.split()[0] if lo else ""
        if h == "ver": return ["# fw 2.0.3-fake | last reset: power-on"]
        if h == "params": return ["# params: " + " ".join("%s=%g" % (k, v) for k, v in self.p.items())]
        if h == "hb": return ["# hb ok"]
        if h == "tag": self.tag = t[4:]; return ["# Tag: " + self.tag]
        if h == "deadman":
            m = int(lo.split()[1]); self.dead_at = None if m == 0 else m * 60; return ["# Dead-man timer " + ("off" if m == 0 else "armed")]
        if h == "sp": self.sp = float(lo.split()[1]); self.on = True; return ["# PID: SP=%.2f" % self.sp]
        if h == "s": self.on = False; return ["# Motor stopped, PID off"]
        if h == "a": self.on = True; return ["# Profile started fresh"]
        if h == "set":
            _, n, v = lo.split()
            if n in self.p: self.p[n] = float(v); self.c.p = self.p; return ["# %s=%g" % (n, float(v))]
            return ["# Unknown parameter (see 'params')"]
        if h == "q": self.live = not self.live; return ["# Live data %s" % ("ON" if self.live else "OFF")]
        return ["# Unknown command"]


def serve(fake, conn):
    buf = b""
    conn.sendall(b"# Connected to fake ESP32\n")
    def reader():
        nonlocal buf
        while True:
            try: d = conn.recv(1024)
            except OSError: return
            if not d: return
            buf += d
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                for r in fake.handle(line.decode()): conn.sendall((r + "\n").encode())
    threading.Thread(target=reader, daemon=True).start()
    try:
        while True:
            meas, u = fake.step()
            if fake.dead_at and time.time() - fake.last_cmd > fake.dead_at / fake.speed:
                fake.on = False; fake.dead_at = None; conn.sendall(b"# DEAD-MAN: no command for too long - Peltier OFF\n")
            if fake.live:
                d = 0 if u == 0 else (1 if u < 0 else -1)
                line = "Time:2026-10-29 12:00:00.000,PWM_pct:%.1f,Direction:%d,Temp1_C:nan,Temp2_C:nan,Temp3_C:%.1f,Temp3f_C:%.2f,Setpoint_C:%.1f,Phase:%s\n" % (
                    abs(u) / 2.55, d, fake.T, fake.T, fake.sp, fake.phase())
                conn.sendall(line.encode())
            time.sleep(0.1 / fake.speed)
    except OSError:
        pass


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--speed", type=float, default=1.0); a = ap.parse_args()
    s = socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1); s.bind(("127.0.0.1", 3333)); s.listen(1)
    print("fake rig on 127.0.0.1:3333, speed x%g" % a.speed, flush=True)
    fake = Fake(a.speed)
    while True:
        conn, _ = s.accept(); serve(fake, conn)


if __name__ == "__main__":
    main()
