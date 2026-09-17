#!/usr/bin/env bash
# Start ArduPilot SITL with this project's precision-landing parameters loaded.
# Works from any machine, any username, any clone location -- no path editing needed.
#
#   ./sim/run_sitl.sh
#
# Optional overrides (env vars):
#   ARDUPILOT_DIR    where ArduPilot is cloned+built (default: ~/ardupilot, then
#                    ~/Documents/gitClone/ardupilot, whichever exists first)
#   OUT_PORT         UDP port your ground station listens on (default: 14550)
#   COMPANION_PORT   UDP port src/mavlink_out.py (and anything else acting as a
#                    companion computer) connects to (default: 14540). Separate from
#                    OUT_PORT because ArduPilot's SITL TCP serial driver only tracks
#                    ONE client per port (AP_HAL_SITL/UARTDriver.cpp accept() into a
#                    single _fd) -- confirmed 2026-09-15 when a second client on
#                    tcp:5760 connected but silently never received a heartbeat while
#                    MAVProxy already held that port. This port is a second MAVProxy
#                    --out link instead, same pattern already proven with QGC/Mission
#                    Planner on OUT_PORT, just not sharing their port.
#
# Any extra arguments are passed straight through to sim_vehicle.py, e.g.:
#   ./sim/run_sitl.sh --speedup 2
#
# Defaults SITL's start location to Amrita Vishwa Vidyapeetham, Bengaluru
# (12.895444637941674, 77.67580386901876) -- matches sim/precision_landing.parm's
# SIM_PLD_LAT/LON, decided 2026-09-15. Altitude 909m: found by first trying 0 and
# reading ArduPilot's own "Terrain: clamping offset -909 to -30" warning at boot,
# which is SITL's terrain database reporting the real elevation for this exact
# lat/lon -- more reliable than any public source for Kasavanahalli specifically.
# Pass your own -L/-l/--location/--custom-location to override (e.g. back to
# ArduPilot's CMAC default for an unrelated test).
set -euo pipefail

DEFAULT_LOCATION="12.895444637941674,77.67580386901876,909,0"
location_arg=()
has_location_flag=false
for arg in "$@"; do
    case "$arg" in
        -L|--location|-L=*|--location=*|-l|--custom-location|-l=*|--custom-location=*)
            has_location_flag=true
            ;;
    esac
done
if [ "$has_location_flag" = false ]; then
    location_arg=(--custom-location="$DEFAULT_LOCATION")
fi

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
COMPANION_PORT="${COMPANION_PORT:-14540}"

echo "ArduPilot:      $AP"
echo "Params:         $PARM_FILE"
echo "GCS port:       UDP $OUT_PORT (point QGroundControl or Mission Planner here)"
echo "Companion port: UDP $COMPANION_PORT (src/mavlink_out.py connects here)"
echo ""

cd "$AP"
exec Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild \
    --out="udp:127.0.0.1:${OUT_PORT}" \
    --out="udp:127.0.0.1:${COMPANION_PORT}" \
    --add-param-file="$PARM_FILE" \
    "${location_arg[@]}" \
    "$@"
