#!/usr/bin/env bash
# Start ArduPilot SITL with this project's precision-landing parameters loaded.
# Works from any machine, any username, any clone location -- no path editing needed.
#
#   ./sim/run_sitl.sh
#
# Optional overrides (env vars):
#   ARDUPILOT_DIR   where ArduPilot is cloned+built (default: ~/ardupilot, then
#                   ~/Documents/gitClone/ardupilot, whichever exists first)
#   OUT_PORT        UDP port your ground station listens on (default: 14550)
#
# Any extra arguments are passed straight through to sim_vehicle.py, e.g.:
#   ./sim/run_sitl.sh --speedup 2
set -euo pipefail

# This script's own directory -- makes --add-param-file correct regardless of
# where this repo was cloned or who's running it. This is the actual fix for
# the "edit this path for your machine" problem: don't ask, don't hardcode,
# derive it.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARM_FILE="$HERE/precision_landing.parm"

# ArduPilot's clone location is genuinely per-person (Mac vs Linux, different
# usernames, different chosen directories -- confirmed different across the
# team already). Same resolution order as the old Webots-era scripts this
# project used before: explicit override first, then the two locations we've
# actually seen in practice, first match wins.
if [ -n "${ARDUPILOT_DIR:-}" ]; then
    AP="$ARDUPILOT_DIR"
elif [ -d "$HOME/ardupilot" ]; then
    AP="$HOME/ardupilot"
elif [ -d "$HOME/Documents/gitClone/ardupilot" ]; then
    AP="$HOME/Documents/gitClone/ardupilot"
else
    echo "Can't find an ArduPilot clone. Either:" >&2
    echo "  export ARDUPILOT_DIR=/path/to/your/ardupilot" >&2
    echo "  or clone it to ~/ardupilot or ~/Documents/gitClone/ardupilot" >&2
    echo "See sim/README.md section 1 for the build steps." >&2
    exit 1
fi

BIN="$AP/build/sitl/bin/arducopter"
if [ ! -x "$BIN" ]; then
    echo "ArduPilot found at $AP but SITL isn't built yet. Run:" >&2
    echo "  cd $AP && ./waf configure --board sitl && ./waf copter" >&2
    exit 1
fi

OUT_PORT="${OUT_PORT:-14550}"

echo "ArduPilot: $AP"
echo "Params:    $PARM_FILE"
echo "GCS port:  UDP $OUT_PORT (point QGroundControl or Mission Planner here)"
echo ""

cd "$AP"
exec Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild \
    --out="udp:127.0.0.1:${OUT_PORT}" \
    --add-param-file="$PARM_FILE" \
    "$@"
