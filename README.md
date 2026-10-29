# peltierbench_agent

Hosted, agent-driven tuning for the ESP32 Peltier water bath. Nothing here needs your PC to stay on.

```
ESP32 --TLS--> MQTT broker (HiveMQ Cloud free) <--- tools/bench.py <--- GitHub Actions "experiment" (bounded script)
   ^                  |                                                         ^
   |                  +--> viewer/index.html (GitHub Pages, read-only)           | dispatches
   +-- OTA from GitHub Releases <-- "build-firmware" workflow <-- PR <-- "agent" workflow (Claude, short sessions)
```

| Path | What it is |
|---|---|
| `firmware/peltier_logger_v2/` | ESP32 sketch (see `CHANGES.md`) |
| `sim/` | Offline simulator and the scoring function |
| `tools/bench.py` | The only code that talks to the rig, with its own safety limits |
| `viewer/` | Live dashboard and experiment ledger (GitHub Pages) |
| `ledger/`, `data/` | Experiment history, written by the workflows and the agent |
| `AGENTS.md` | The agent's instructions and rules |

## One-time setup

### 1. Broker (about 5 minutes)
1. Create a free HiveMQ Cloud Serverless cluster. Note the host name. TLS port is 8883; WebSocket port is 8884.
2. Under Access Management add three credentials and give each its topic permissions (use your own base, e.g. `remster/peltier-rig`):

| User | Publish | Subscribe |
|---|---|---|
| `rig` | `<base>/telemetry`, `<base>/events`, `<base>/resp`, `<base>/status` | `<base>/cmd` |
| `agent` | `<base>/cmd` | `<base>/#` |
| `viewer` | nothing | `<base>/#` |

### 2. GitHub secrets (Settings, Secrets and variables, Actions)
`MQTT_HOST`, `MQTT_PORT` (8883), `MQTT_BASE`, `MQTT_AGENT_USER`, `MQTT_AGENT_PASS`.

### 3. Flash the first firmware over USB (the only manual step)
1. Push to `main`; the **build-firmware** workflow compiles and attaches `firmware.bin` to a release.
2. Flash that image (or compile `firmware/peltier_logger_v2` in the Arduino IDE with the ESP32 core 3.0.x and PubSubClient, partition scheme "Minimal SPIFFS").
3. In the serial monitor (115200, Newline):
   ```
   cfg wifi <ssid>;<password>
   cfg mqtt <host>;8883;rig;<rig password>;<base>
   reboot
   ```
   The credentials live in the ESP32's flash, not in the firmware image or in this repo.

### 4. Viewer
Settings, Pages, deploy from `main` / root. Open `https://<user>.github.io/<repo>/viewer/`, enter
`wss://<host>:8884/mqtt`, the `viewer` account and the base.

### 5. Turn on the agent (optional, later)
Add `CLAUDE_CODE_OAUTH_TOKEN` as a secret and uncomment the cron in `.github/workflows/agent.yml`.
Until then you can run the agent session by hand with "Run workflow".

## Safety
- The firmware keeps its own hard limits (55 C hard stop, freeze guard, wrong-direction guard, sensor checks) and compile-time duty caps.
- Over MQTT the rig refuses manual drive, SD access, clock changes and credential changes.
- Experiments arm a 5-minute dead-man timer on the rig and refresh it every minute: if the workflow dies, the Peltier stops.
- The agent cannot touch the safety constants or the workflows without a human merging the pull request. Protect `main` and
  require a review (the `CODEOWNERS` file asks for yours).
- OTA only accepts URLs under this repository's releases. OTA has no automatic rollback; a firmware that crashes at boot needs a USB reflash.
