#!/usr/bin/env bash
# One bounded agent session. Run by peltier-agent.timer. Costs nothing (no API call) when the rig is busy,
# paused, faulted or the daily session budget is used up.
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
STATE="${PELTIER_STATE:-$HOME/peltier-state}"
MAX_PER_DAY="${MAX_SESSIONS_PER_DAY:-40}"
MAX_TURNS="${MAX_TURNS:-40}"
mkdir -p "$STATE/logs"
log() { echo "[$(date -Is)] agent: $*" >>"$STATE/logs/agent.log"; }

exec 9>"$STATE/agent.lock"; flock -n 9 || exit 0
[ ! -e "$STATE/PAUSE" ] || { log "paused"; exit 0; }
cd "$REPO"

BENCH=(python3 tools/bench_local.py)
S="$("${BENCH[@]}" status 2>/dev/null)" || { log "bench daemon not running"; exit 0; }
STATE_JSON="$S" python3 - <<'PY' || exit 0
import json, os, sys
s = json.loads(os.environ["STATE_JSON"])
if not s["connected"]: print("rig not connected", file=sys.stderr); sys.exit(1)
if s["exp"].get("running"): sys.exit(1)           # experiment in progress: nothing to decide yet
PY

# Rule 6: a FAULT or an aborted run stops everything until a human clears PAUSE
LAST="$(tail -n 1 ledger/experiments.jsonl 2>/dev/null || true)"
FAULT_NOW="$(STATE_JSON="$S" python3 -c 'import json,os; s=json.loads(os.environ["STATE_JSON"]); t=s.get("tele") or {}; print(int(t.get("ph") in ("FAULT","SENSOR_FAULT")))')"
ABORTED="$(LAST_JSON="$LAST" python3 -c 'import json,os; l=os.environ["LAST_JSON"]; print(json.loads(l).get("aborted") or "" if l else "")' 2>/dev/null || true)"
if [ "$FAULT_NOW" = 1 ] || { [ -n "$ABORTED" ] && [ "$(cat "$STATE/acked_abort" 2>/dev/null)" != "$LAST" ]; }; then
  echo "$LAST" >"$STATE/acked_abort"; touch "$STATE/PAUSE"
  printf '\n## %s STOPPED by the Pi runner (AGENTS.md rule 6)\n- Reason: %s\n- Waiting for a human to inspect the rig and delete %s/PAUSE.\n' \
    "$(date -I)" "${ABORTED:-rig reported FAULT}" "$STATE" >>ledger/NOTES.md
  log "stopped: ${ABORTED:-FAULT}"; exit 0
fi

# daily budget
DAY="$STATE/sessions-$(date +%F)"; N=$(cat "$DAY" 2>/dev/null || echo 0)
[ "$N" -lt "$MAX_PER_DAY" ] || { log "daily session budget ($MAX_PER_DAY) used"; exit 0; }
echo $((N+1)) >"$DAY"

# credentials: root-only file with ANTHROPIC_API_KEY or CLAUDE_CODE_OAUTH_TOKEN
set -a; . "${PELTIER_ENV:-/etc/peltier/agent.env}"; set +a

git pull -q --ff-only origin main || log "git pull failed; continuing on the local checkout"

PROMPT="You are the Peltier bench agent running unattended on the Raspberry Pi next to the rig. Follow AGENTS.md, including its 'Pi mode' section.
Read ledger/NOTES.md and the last entries of ledger/experiments.jsonl, then do exactly one of:
 (a) start ONE hardware experiment with 'python3 tools/bench_local.py experiment ...' and end the session (do not wait for it: the next session reads the result),
 (b) change the firmware: edit the sketch, bump FW_VERSION, run 'bash pi/flash.sh --compile', then 'bash pi/flash.sh', then commit on a branch agent/<topic> and open a PR describing the evidence and the risk,
 (c) update ledger/NOTES.md with conclusions and the next plan.
Use the simulator first for anything broad. Always end by appending to ledger/NOTES.md what you did, what you saw, and what you will do next.
If anything looks unsafe or surprising, run 'touch ~/peltier-state/PAUSE', write it in ledger/NOTES.md, and stop."

log "session start ($((N+1))/$MAX_PER_DAY)"
timeout 40m claude -p "$PROMPT" --settings pi/agent-settings.json --max-turns "$MAX_TURNS" --output-format text \
  >>"$STATE/logs/session-$(date +%F).log" 2>&1
log "session end (exit $?)"
