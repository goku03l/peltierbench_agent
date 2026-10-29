#!/usr/bin/env python3
"""Refuse to flash firmware whose safety constants differ from the human-approved baseline.

    python3 pi/guard_check.py check     # used by pi/flash.sh; exit 1 on any difference
    python3 pi/guard_check.py lock      # HUMAN ONLY: re-record the baseline after you reviewed a safety change

Two checks: (1) the protected constant lines must be identical (comments and spacing ignored);
(2) safety symbols must still appear at least as often as in the baseline, so a guard cannot be deleted quietly.
This is a tripwire, not a proof of safety: it cannot tell whether the logic around a guard was changed.
"""
import json, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
INO = os.path.join(HERE, "..", "firmware", "peltier_logger_v2", "peltier_logger_v2.ino")
LOCK = os.path.join(HERE, "protected.lock")

CONSTANTS = ["HARD_CAP_COOL", "HARD_CAP_HEAT", "OTA_URL_PREFIX", "WATER_MIN_C", "WATER_MIN_CLEAR_C", "WATER_MAX_C",
             "WATER_MAX_CLEAR_C", "WATER_HARD_MAX_C", "SP_MIN_C", "SENSOR_MIN_C", "SENSOR_MAX_C", "SENSOR_SPIKE_C",
             "SENSOR_SPIKE_N", "SENSOR_FAULT_MS", "SENSOR_OK_MS", "CW_RANGE_LO_C", "CW_RANGE_HI_C", "CW_MIN_DROP_C_PER_MIN",
             "CW_WINDOW_MS", "CW_COOLDOWN_MS", "CW_MAX_TRIPS", "WRONGDIR_COOL_MS", "WRONGDIR_HEAT_MS", "WRONGDIR_COOLDOWN_MS",
             "WRONGDIR_MAX_TRIPS", "MANUAL_MAX_MS", "PELTIER_SLEW_PER_S", "REVERSAL_MIN_OFF_MS"]
SYMBOLS = ["latchFault", "tripWrongDirection", "conditionWater", "faultLatched", "blockedUntil", "deadmanMs",
           "HARD_CAP_COOL", "HARD_CAP_HEAT", "WATER_MIN_C", "WATER_MAX_C", "WATER_HARD_MAX_C", "OTA_URL_PREFIX", "capCool", "capHeat"]

DECL = r"^[ ]*(?:#define[ ]+NAME[ ]+|(?:static[ ]+)?const[^=;]*?\bNAME[ ]*=[ ]*)(.*)$"
COMMENT = re.compile(r'("(?:[^"\\]|\\.)*")|//.*')      # keeps string literals, so the // in a URL survives


def snapshot():
    src = open(INO, encoding="utf-8").read()
    lines = {}
    for name in CONSTANTS:
        m = re.search(DECL.replace("NAME", name), src, re.M)
        if not m: sys.exit("guard_check: cannot find protected constant %s in the sketch" % name)
        rest = COMMENT.sub(lambda g: g.group(1) or "", m.group(1)).strip().rstrip(";")
        lines[name] = re.sub(r"\s+", "", rest)
    return {"constants": lines, "symbols": {s: len(re.findall(r"\b%s\b" % s, src)) for s in SYMBOLS}}


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "check"
    cur = snapshot()
    if mode == "lock":
        json.dump(cur, open(LOCK, "w"), indent=1, sort_keys=True); print("baseline written to", LOCK); return
    if not os.path.exists(LOCK): sys.exit("guard_check: no baseline; a human must run 'python3 pi/guard_check.py lock' first")
    base = json.load(open(LOCK)); bad = []
    for k, v in base["constants"].items():
        if cur["constants"].get(k) != v: bad.append("constant %s changed: %s -> %s" % (k, v, cur["constants"].get(k)))
    for k, n in base["symbols"].items():
        if cur["symbols"].get(k, 0) < n: bad.append("symbol %s used %d times, baseline %d" % (k, cur["symbols"].get(k, 0), n))
    if bad: print("\n".join("GUARD: " + b for b in bad)); sys.exit(1)
    print("guard_check: protected constants and safety symbols match the baseline")


if __name__ == "__main__":
    main()
