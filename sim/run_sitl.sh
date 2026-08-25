#!/usr/bin/env bash
# Start ArduPilot SITL with our precision-landing parameters already applied.
#
#   ./sim/run_sitl.sh
#
# Leaves the autopilot on:
#   TCP 5760  SERIAL0 - ground station (MAVProxy / QGroundControl)
#   TCP 5762  SERIAL1 - companion computer (our ROS 2 bridge)
set -euo pipefail

AP="${ARDUPILOT_DIR:-$HOME/Documents/gitClone/ardupilot}"
BIN="$AP/build/sitl/bin/arducopter"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ ! -x "$BIN" ]; then
    echo "SITL not built. Run:" >&2
    echo "  cd $AP && ./waf configure --board sitl && ./waf copter" >&2
    exit 1
fi

# Keep eeprom/logs out of the source tree.
RUNDIR="$HERE/.sitl_run"
mkdir -p "$RUNDIR"
cd "$RUNDIR"

echo "SITL running.  GCS -> tcp:127.0.0.1:5760   companion -> tcp:127.0.0.1:5762"
exec "$BIN" \
    --model quad \
    --speedup 1 \
    --defaults "$AP/Tools/autotest/default_params/copter.parm,$HERE/precision_landing.parm"
