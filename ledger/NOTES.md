# Notes

Newest at the bottom. Add what you tried, what the data showed, and what you will try next.

## 2026-10-29 baseline (from the v1 logs, before any hardware run of v2)
- Merged v1 logs: 783k rows over 22.5 h, 100 ms samples. Condensed copy: `data/logged_run_2026-10-28.json` (10 s bins).
- Plant model fitted by least squares on 5-minute averages: `sim/plant.json` (R2 0.69, room 26.7 C). To refit, regress dT/dt on
  [1, T, cool, heat, cool*T] using signed duty from the logs.
- v1 at a 20 C setpoint stalls about 1.2 C high and shows a PWM sawtooth (reproduced in `sim/sim_v1.py`). v2 settles in simulation.
- Open questions for the first hardware runs: (1) does `usedead=0` remove the cycling on the real bath? (2) how large is the sim-to-real gap?
  (3) what is the real sensor lag? Run a step with `--lag` values in the simulator and compare with the logged step.
