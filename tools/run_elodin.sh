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

# Elodin writes a per-run telemetry database (betaflight_dbNNN) into the sim
# directory: 2.6-3.7 GB EACH, and nothing ever removes them. Fifty runs in one
# session accumulated 163 GB and filled a 926 GB disk, which cost hours and had
# us deleting caches, a Windows ISO and frame archives while the real consumer
# sat in the repo the runs were launched from. Clean up before and after.
rm -rf "$ELODIN_DIR"/betaflight_db* 2>/dev/null

# Filter at the source. Unfiltered, Elodin's per-tick output runs to GIGABYTES
# for a 200 s sim; the lines we actually read are a few hundred. Note the comment
# must sit ABOVE the whole command: put between the env assignments and `elodin`
# it orphans them, and the sim silently runs the DEFAULT course (caught by a run
# reporting course=easy when vq2turn was asked for).
COURSE="$COURSE_NAME" PYTHONPATH="$DGP_DIR" RACE_SOLVER="$SOLVER" elodin run sim/main.py 2>&1 \
    | grep -E --line-buffered "SERVO|PHASE|ALIGN|RACE|GATE|BRINGUP|Error|error|Traceback" \
    > "$LOG" &

cleanup_dbs() { rm -rf "$ELODIN_DIR"/betaflight_db* 2>/dev/null; }
trap cleanup_dbs EXIT

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
