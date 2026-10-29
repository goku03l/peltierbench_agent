# peltier_logger_v2: what changed and why

Your v1 sketch is untouched. This is a copy with the changes below. **It has not been compiled yet** (no toolchain on this PC); the first build will run in GitHub Actions or in your Arduino IDE.

## Fixes, with the evidence from the 28-29 Oct logs

| # | Problem in v1 | Evidence | Change in v2 |
|---|---|---|---|
| 1 | Integral wiped on every error sign flip | Sim of v1 reproduces the 21-22 C plateau at a 20 C setpoint and the PWM sawtooth seen in the log | `resetflip` parameter, default 0 |
| 2 | Integral too slow, decays while idle | Ki 0.15 = about 9 duty/min at 1 C error | Ki 0.6, `idledecay` default 0 |
| 3 | D term amplifies sensor noise | Kd 150, `pid_d` reached +-66 | Kd 20 |
| 4 | Noisy input into PID and guards | 1,460 heating samples jump 1 C or more between 100 ms samples | Low-pass `alpha` 0.95 on NTC3; guards and PID use the filtered value. The CSV gets an extra `temp3f_c` column |
| 5 | Wrong-direction guard false trips | Water rose 31.9 to 34.0 C while the guard said "water is COOLING while heating" | Guard compares start and end of the window on the filtered reading (`wdrift` parameter) |
| 6 | Open NTC1/NTC2 read -33.7 C | 96-98% of rows | Readings at or above 3050 mV are NaN |
| 7 | Schedule advanced by timer regardless of the water | Profile "completed" at 40 C with the water at 33 C | Every step waits for the water to reach the target (90 min timeout), then holds 1 hour |
| 8 | HOLD/RAMPING flicker in the log | 21,705 phase flips in one file | Hysteresis on the label |
| 9 | Minimum duty floors too high | 60/50 forced the output from 0 to 23% | 40/35 (`mincool`, `minheat`) |

## New for remote operation
- **MQTT over TLS in its own task**, so a slow connection can never stall the control loop. Topics under `MQTT_BASE`: `telemetry` (1 Hz JSON), `events` (every `# ...` message), `resp` (command replies), `cmd` (commands in), `status` (online/offline).
- **Runtime parameters**: `params`, `get <name>`, `set <name> <value>`, `save`, `defaults`. All values are range-checked.
- **Dead-man timer**: `deadman <minutes>` arms it. If no MQTT command arrives in time, the Peltier stops and the PID is switched off.
- **`tag <text>`** labels an experiment in the telemetry.
- **`ota <url>`** downloads firmware, only from a URL starting with `OTA_URL_PREFIX`.
- **Over MQTT these are refused**: manual `f`/`r` drive, all SD commands, setting the clock.
- **Hard caps stay in the code**: `HARD_CAP_COOL` 220 and `HARD_CAP_HEAT` 100. Runtime parameters can go lower, never higher.

## Known limits (please read)
1. **Not compiled or tested on hardware.** Expect a fix or two on first build.
2. **TLS does not verify the broker certificate** (`setInsecure()`): the link is encrypted, but a man-in-the-middle on your network could impersonate the broker. Pinning the CA certificate is a to-do.
3. **OTA has no automatic rollback.** The Arduino ESP32 core does not enable it by default. A firmware that crashes at boot needs a USB reflash. The outputs are off at boot (enable pin low, plus your 10k pull-downs), so a dead board is safe, just offline.
4. **`ota` stops the motor and saves "stopped" state**, so after an update the agent must restart the run (`sp <C>` or `a`).
5. **Heating cap is still 100.** Raising it is a hardware decision (supply and driver). Change `HARD_CAP_HEAT` yourself when you've checked.
6. **The deadband default is on** (as in the "planned fixes" simulation). The simulation showed it causes small on/off cycling; try `set usedead 0` once the rig is running.
7. **NTC1/NTC2 are still not connected**, so there is no hot-side reading yet.

## Before the first flash
1. Install the PubSubClient library (CI does this itself).
2. Flash once over USB. After that, updates can go over the air.
3. Set the credentials once in the serial monitor. They are stored in flash, not in the firmware image:
   `cfg wifi <ssid>;<password>`, `cfg mqtt <host>;<port>;<user>;<password>;<topic base>`, then `reboot`.
   `cfg` only works over USB, never over MQTT or the TCP port.
