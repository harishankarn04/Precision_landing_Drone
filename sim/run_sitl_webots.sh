#!/usr/bin/env bash
# Start ArduPilot SITL wired to Webots instead of its own internal physics.
#
#   1. Open Webots and load:
#        ~/Documents/gitClone/ardupilot/libraries/SITL/examples/Webots_Python/worlds/iris_camera.wbt
#      (or run ./sim/run_webots.sh) and press the ▶ Play button.
#   2. Then run this script.
#
# With --model webots-python, Webots computes the physics and ArduPilot just
# flies. The vehicle's sensors come from Webots over UDP 9002/9003, and the
# downward camera is streamed as raw grayscale on TCP 5599.
#
# Ports:
#   UDP 9002/9003  SITL <-> Webots controller
#   TCP 5760       SERIAL0 - ground station
#   TCP 5762       SERIAL1 - companion computer (our ROS 2 bridge)
#   TCP 5599       camera images out of Webots
set -euo pipefail

AP="${ARDUPILOT_DIR:-$HOME/Documents/gitClone/ardupilot}"
BIN="$AP/build/sitl/bin/arducopter"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEBOTS_PARAMS="$AP/libraries/SITL/examples/Webots_Python/params/iris.parm"

if [ ! -x "$BIN" ]; then
    echo "SITL not built. Run:" >&2
    echo "  cd $AP && ./waf configure --board sitl && ./waf copter" >&2
    exit 1
fi

RUNDIR="$HERE/.sitl_webots_run"
mkdir -p "$RUNDIR"
cd "$RUNDIR"

echo "SITL -> Webots.  GCS tcp:127.0.0.1:5760   companion tcp:127.0.0.1:5762"
echo "Waiting for the Webots controller on UDP 9002 — press ▶ in Webots if you have not."
exec "$BIN" \
    --model webots-python \
    --defaults "$AP/Tools/autotest/default_params/copter.parm,$WEBOTS_PARAMS,$HERE/precision_landing.parm"
