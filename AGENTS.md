# Agent instructions: Peltier bench

## Goal
Find the PID tuning and control logic that gives the best performance for this Peltier water bath: reach the setpoint
quickly, little overshoot, settle tight (small offset and spread), no on/off cycling, across 17.5 to 40 C.
`sim/metrics.py:score` is the objective (lower is better). Real runs and simulated runs use the same function.

## What you can do
| Action | How |
|---|---|
| Try ideas offline | `python sim/bench_sim.py --sp 20 --start 24.5 --set ki=1 usedead=0` (add `--sweep k=a,b,c`, `--lag 20`, `--seeds 5`) |
| Read what happened | `ledger/experiments.jsonl` (one JSON per hardware run), `data/runs/*.csv`, `ledger/NOTES.md` |
| Run a hardware experiment | `gh workflow run experiment.yml -f tag=... -f sp=... -f minutes=... -f params="ki=1 usedead=0"` |
| Change the control logic | Edit `firmware/peltier_logger_v2/peltier_logger_v2.ino` and open a **pull request**. CI compiles it; a human merges it |
| Record conclusions | Append to `ledger/NOTES.md`: what you tried, what the data showed, what you will try next |

You do not talk to the rig directly. Only the `experiment` workflow does, through `tools/bench.py`.

## Rules you must not break
1. Never edit `tools/bench.py` safety limits, `.github/workflows/*`, `HARD_CAP_COOL`, `HARD_CAP_HEAT`, the water temperature
   limits, the guard thresholds, or `OTA_URL_PREFIX`. Change proposals for those go in `ledger/NOTES.md` for the human.
2. Never put credentials in the repo. Never print secrets.
3. One experiment at a time. Check `gh run list --workflow experiment.yml` before dispatching.
4. Keep every experiment bounded: at most 90 minutes unless the human says otherwise, setpoint 15 to 40 C, and always
   set `--tmin/--tmax` sensibly around the setpoint.
5. Firmware changes go through a pull request, never straight to `main`. Say in the PR what evidence supports it and what
   could go wrong. A bad firmware can only be recovered by USB, so keep changes small and reviewable.
6. If a run aborts or the rig reports a FAULT, stop experimenting, write what happened in `ledger/NOTES.md`, and wait.

## How to work
1. Read `ledger/NOTES.md` and the last ledger entries first.
2. Prefer the simulator for broad searches (it is cheap). Use the rig to confirm the best two or three candidates and to
   measure how far the model is from reality. When real and simulated results disagree, say so and refit
   (`sim/plant.json` is a least-squares fit; the process is described in `ledger/NOTES.md`).
3. Change one thing at a time on hardware, so a result can be attributed.
4. Compare like with like: same setpoint, same start condition (use `--pre-sp` to bring the bath to a known start).
5. Write down negative results too.

## What is already known (from the 28-29 Oct 2026 logs)
- v1 problems: integral reset on every error sign flip, slow Ki, noisy D term, noisy input, false wrong-direction
  trips, schedule advancing by timer. v2 addresses these.
- Cooling bottoms out near 16 to 19 C, probably a hot-side limit; there is no hot-side sensor yet. Do not try to tune
  your way past a hardware limit.
- Heating is capped at duty 100 (39 percent). At 37.5 C and above the heater saturates.
- In simulation the deadband with 30 s dwell causes small on/off cycling; `usedead=0` removes it. Confirm on the rig.
- The simulator has no hot-side warm-up and no sensor lag by default, so it is optimistic.

## Pi mode (agent runs unattended on the Raspberry Pi; authorised by the human on 2026-10-29)
This section amends rules 3 and 5 for sessions started by `pi/agent_session.sh`. Everything else still applies.
- Talk to the rig only through `python3 tools/bench_local.py` (never open the serial port yourself). Start an experiment and
  end the session; the next session reads the result in `ledger/experiments.jsonl`. Do not use `wait`.
- Firmware: you may flash with `bash pi/flash.sh` after `bash pi/flash.sh --compile` passes. You must bump `FW_VERSION`, keep
  the change small, work on a branch `agent/<topic>`, and open a pull request with the evidence and the risk. You never push
  to `main`; the human merges. If the human rejects the PR, the rig is reflashed from `main`.
- `pi/flash.sh` refuses changes to protected constants and guards (`pi/guard_check.py`). If it refuses, do not try to work
  around it: write a proposal in `ledger/NOTES.md` for the human.
- You cannot edit `tools/`, `.github/`, `AGENTS.md`, `pi/` (enforced by `pi/agent-settings.json`).
- Rule 6 is automatic: after a FAULT, an aborted run or a failed flash the runner creates `~/peltier-state/PAUSE` and no
  session starts until the human removes it. If you notice something unsafe, `touch ~/peltier-state/PAUSE` yourself.
