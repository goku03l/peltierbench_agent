# Running the agent on a Raspberry Pi

```
ESP32 --USB--> Pi: peltier-daemon (tools/bench_local.py, owns the serial port, enforces rig limits)
                   peltier-agent.timer -> pi/agent_session.sh -> claude -p (one bounded session per tick)
                   pi/flash.sh (guard check -> compile -> flash -> verify -> rollback)
```

## One-time setup (Raspberry Pi OS 64-bit, any Pi 4/5 with network)
```
sudo adduser --disabled-password peltier && sudo usermod -aG dialout peltier
sudo -iu peltier
git clone https://github.com/goku03l/peltierbench_agent bench_repo && cd bench_repo
pip3 install --break-system-packages -r requirements.txt pyserial
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
arduino-cli core install esp32:esp32 && arduino-cli lib install PubSubClient     # same libs the sketch needs
npm i -g @anthropic-ai/claude-code && gh auth login                               # gh: PR rights on a fork or branch-only token
exit
```
Credentials (root-only, never in the repo):
```
sudo install -d -m 0750 -o root -g peltier /etc/peltier
echo 'CLAUDE_CODE_OAUTH_TOKEN=...' | sudo tee /etc/peltier/agent.env >/dev/null     # or ANTHROPIC_API_KEY=...
sudo chmod 0640 /etc/peltier/agent.env && sudo chgrp peltier /etc/peltier/agent.env
```
Services:
```
sudo cp pi/systemd/peltier-*.service pi/systemd/peltier-agent.timer /etc/systemd/system/
sudo install -m 0440 pi/systemd/peltier-sudoers /etc/sudoers.d/peltier
sudo systemctl daemon-reload && sudo systemctl enable --now peltier-daemon
```
Check the port name (`ls /dev/ttyUSB* /dev/ttyACM*`) and edit `peltier-daemon.service` if it is not `/dev/ttyUSB0`.

## First flash and go-live (a human does these)
1. `python3 pi/guard_check.py lock` is already committed; re-run it only after you review a deliberate safety change.
2. `bash pi/flash.sh --first` flashes the reviewed sketch and records it as the known-good image.
3. `python3 tools/bench_local.py status` shows `connected: true` and the firmware version.
4. Dry run: `bash pi/agent_session.sh` once by hand and read `~/peltier-state/logs/`.
5. `sudo systemctl enable --now peltier-agent.timer`

## Operating it
| Want to | Do |
|---|---|
| Watch live | `ssh -L 8765:127.0.0.1:8765 peltier@pi`, then open http://127.0.0.1:8765 (the daemon only listens on localhost) |
| Pause the agent | `touch ~/peltier-state/PAUSE` (the agent also writes this after a FAULT, an aborted run or a failed flash) |
| Resume | read `ledger/NOTES.md`, fix the cause, `rm ~/peltier-state/PAUSE` |
| Stop the rig now | `python3 tools/bench_local.py abort`, or `sudo systemctl stop peltier-agent.timer` |
| Logs | `~/peltier-state/logs/{agent,flash,session-DATE}.log`, `journalctl -u peltier-daemon` |
| Spend cap | `MAX_SESSIONS_PER_DAY` (default 40) and `MAX_TURNS` (default 40) in `pi/agent_session.sh` |

## What protects the rig, in layers
1. ESP32 firmware guards and dead-man timer (5 min; the daemon refreshes it every minute during an experiment).
2. `bench_local.py`: whitelisted commands, parameter bounds, setpoint 15 to 40 C, 180 min cap, abort on out-of-band water or silent data.
3. `pi/guard_check.py`: flashing is refused if a protected constant changed or a guard symbol disappeared.
4. `pi/flash.sh`: refuses when protected paths differ from `origin/main`, needs a version bump, verifies after flashing, rolls back on failure, then pauses the agent.
5. `pi/agent-settings.json`: the agent cannot edit `tools/`, `.github/`, `AGENTS.md`, `pi/`, cannot push to `main`, and has no `sudo`, `rm`, `curl` or `ssh`.
6. Branch protection on `main` (set this in GitHub: require a PR and your review) so even a leaked token cannot change protected files.

## Known limits
- The guard check is a tripwire. It cannot prove that new control logic is safe, only that the numeric limits and guard symbols are intact. Review the PRs the agent opens, even though it flashes before you merge.
- The flash happens before the merge, so the rig can run firmware that `main` does not have yet. If you reject the PR, run `git checkout main && bash pi/flash.sh`.
- The Pi is a single point of failure for the dead-man refresh: if it crashes mid-run the rig stops itself within 5 minutes.
- Nothing here has run on a real Pi yet. The scripts are syntax-checked and the guard check is tested, but `flash.sh` and the services need a first supervised run.
