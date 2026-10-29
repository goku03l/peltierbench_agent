#!/usr/bin/env bash
# Guarded compile -> flash -> verify -> rollback for the ESP32 on this Pi.
#   bash pi/flash.sh            flash the sketch in the working tree (needs a version bump and a known-good image)
#   bash pi/flash.sh --first    HUMAN, once: flash the sketch and seed the known-good image
#   bash pi/flash.sh --compile  compile and run the guards only
# A bad flash is rolled back to the last image that booted and answered 'ver'. If the rollback fails too, a
# PAUSE file stops the agent and the rig needs a human with a USB cable.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
STATE="${PELTIER_STATE:-$HOME/peltier-state}"
PORT="${PELTIER_PORT:-/dev/ttyUSB0}"
CLI="${ARDUINO_CLI:-arduino-cli}"
FQBN="esp32:esp32:esp32:PartitionScheme=min_spiffs"
SKETCH="$REPO/firmware/peltier_logger_v2"
NEW="$STATE/build/new"; GOOD="$STATE/known_good"
BENCH=(python3 "$REPO/tools/bench_local.py")
MODE="${1:-flash}"
mkdir -p "$STATE/logs" "$NEW" "$GOOD"
log() { echo "[$(date -Is)] flash: $*" | tee -a "$STATE/logs/flash.log"; }
die() { log "REFUSED/FAILED: $*"; exit 1; }

exec 8>"$STATE/flash.lock"; flock -n 8 || die "another flash is running"
[ ! -e "$STATE/PAUSE" ] || [ "$MODE" = "--first" ] || die "agent is paused (PAUSE file); a human must clear it"

# 1. guards: protected paths untouched relative to origin/main, protected constants unchanged
cd "$REPO"
git fetch -q origin main || die "cannot reach origin to check protected paths"
CHANGED="$(git diff --name-only origin/main -- tools .github AGENTS.md pi; git diff --name-only -- tools .github AGENTS.md pi)"
[ -z "$CHANGED" ] || die "protected paths differ from origin/main: $(echo $CHANGED | tr '\n' ' ')"
python3 pi/guard_check.py check || die "safety constants or guards changed; this needs a human"

NEWVER="$(sed -n 's/^#define FW_VERSION "\(.*\)"/\1/p' "$SKETCH/peltier_logger_v2.ino")"
[ -n "$NEWVER" ] || die "cannot read FW_VERSION"
if [ "$MODE" != "--first" ]; then
  [ -f "$GOOD/version" ] || die "no known-good image yet; a human must run: bash pi/flash.sh --first"
  [ "$NEWVER" != "$(cat "$GOOD/version")" ] || die "FW_VERSION $NEWVER is unchanged; bump it so the flash can be verified"
fi

# 2. compile (rig untouched so far)
log "compiling $NEWVER"
rm -rf "$NEW"; mkdir -p "$NEW"
"$CLI" compile --fqbn "$FQBN" --warnings default --output-dir "$NEW" "$SKETCH" >"$STATE/logs/compile.log" 2>&1 || { tail -20 "$STATE/logs/compile.log"; die "compile failed"; }
[ "$MODE" != "--compile" ] || { log "compile OK"; exit 0; }

# 3. rig must be idle, then stop the heat/cool and free the serial port
S="$("${BENCH[@]}" status)" || die "bench daemon not running"
echo "$S" | python3 -c 'import json,sys; s=json.load(sys.stdin); sys.exit(0 if s["connected"] and not s["exp"].get("running") else 1)' \
  || die "rig disconnected or an experiment is running"
"${BENCH[@]}" cmd s >/dev/null || true
sudo -n systemctl stop peltier-daemon

upload() { "$CLI" upload --fqbn "$FQBN" --port "$PORT" --input-dir "$1" "$SKETCH" >>"$STATE/logs/upload.log" 2>&1; }
verify() {   # daemon back up, link connected, firmware reports the expected version, no FAULT phase
  sudo -n systemctl start peltier-daemon
  for _ in $(seq 1 40); do
    sleep 3
    "${BENCH[@]}" status 2>/dev/null | python3 -c '
import json,sys
s=json.load(sys.stdin); t=s.get("tele") or {}
sys.exit(0 if s["connected"] and s.get("fw")==sys.argv[1] and t.get("ph") not in ("FAULT","SENSOR_FAULT") else 1)' "$1" && return 0
  done
  return 1
}

# 4. flash, verify, roll back on failure
log "flashing $NEWVER to $PORT"
if upload "$NEW" && verify "$NEWVER"; then
  "${BENCH[@]}" cmd s >/dev/null || true
  rm -rf "$GOOD"; mkdir -p "$GOOD"; cp -r "$NEW" "$GOOD/bin"; echo "$NEWVER" >"$GOOD/version"
  python3 -c 'import json,sys,datetime as d; print(json.dumps({"when":d.datetime.now().isoformat(timespec="seconds"),"fw":sys.argv[1],"result":"ok","sha":sys.argv[2]}))' \
    "$NEWVER" "$(git rev-parse --short HEAD)" >>"$REPO/ledger/flashes.jsonl"
  log "OK: rig runs $NEWVER, Peltier stopped, image saved as known-good"
  exit 0
fi

log "new firmware did not come up healthy"
if [ -d "$GOOD/bin" ]; then
  sudo -n systemctl stop peltier-daemon
  if upload "$GOOD/bin" && verify "$(cat "$GOOD/version")"; then
    "${BENCH[@]}" cmd s >/dev/null || true
    python3 -c 'import json,sys,datetime as d; print(json.dumps({"when":d.datetime.now().isoformat(timespec="seconds"),"fw":sys.argv[1],"result":"rolled-back"}))' "$NEWVER" >>"$REPO/ledger/flashes.jsonl"
    touch "$STATE/PAUSE"; die "rolled back to $(cat "$GOOD/version"); agent paused so a human can look at logs/compile.log and logs/upload.log"
  fi
fi
touch "$STATE/PAUSE"
die "ROLLBACK FAILED: reflash over USB by hand (bash pi/flash.sh --first after fixing); agent paused"
