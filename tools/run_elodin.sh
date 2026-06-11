#!/bin/bash
# Run one headless Elodin race and exit cleanly.
#
#   tools/run_elodin.sh [course] [logfile] [timeout_s]
#
# Needed because `elodin run` never exits after the sim completes (the
# render-server keeps it alive). This waits for the [RACE] summary line,
# prints it, and kills the stack. Always cleans up stale processes first —
# leftover sim/render-server processes starve the 1000 Hz lockstep.

set -u
COURSE_NAME="${1:-easy}"
LOG="${2:-/tmp/elodin_${COURSE_NAME}.log}"
TIMEOUT_S="${3:-180}"
SOLVER="${4:-elodin_solver}"

ELODIN_DIR="$HOME/code/AIGP/elodin"
DGP_DIR="$HOME/code/AIGP/Drone_Grand_Prix"

cleanup() {
    pkill -f "sim/main.py" 2>/dev/null
    pkill -f "render-server" 2>/dev/null
    pkill -f "betaflight_SITL" 2>/dev/null
}

cd "$ELODIN_DIR" || exit 1
source .venv/bin/activate
source "$HOME/.cargo/env"
# Strip anaconda: its broken pyodbc dist-info crashes elodin's env scanner.
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | paste -sd: -)

cleanup; sleep 3  # let UDP ports release or Betaflight fails to bind and every tick eats a 100ms bridge timeout

COURSE="$COURSE_NAME" PYTHONPATH="$DGP_DIR" RACE_SOLVER="$SOLVER" \
    elodin run sim/main.py > "$LOG" 2>&1 &

for _ in $(seq "$TIMEOUT_S"); do
    if grep -q "\[RACE\]" "$LOG" 2>/dev/null; then
        break
    fi
    sleep 1
done

echo "=== $COURSE_NAME ($SOLVER) ==="
grep -E "\[RACE\]|\[PROBE\]|Traceback|Error" "$LOG" | head -25
cleanup
exit 0
