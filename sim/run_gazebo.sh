#!/usr/bin/env bash
# Start Gazebo + the ardupilot_gazebo plugin + ArduPilot SITL, with MAVLink sent to a
# remote QGroundControl over the LAN -- e.g. QGC running on Hari's Mac while this runs on
# the Mac Mini or a teammate's Linux machine. Works from any machine, any username, any
# clone location, same portability approach as sim/run_sitl.sh.
#
#   GCS_IP=<your GCS machine's LAN IP> ./sim/run_gazebo.sh
#
# GCS_IP is REQUIRED (no silent default) -- MAVLink's UDP --out is a "send to this
# address" mechanism, not "listen for anyone"; picking a wrong default silently drops
# every packet with no error anywhere, so we insist on it explicitly instead. Find your
# GCS machine's IP with `ifconfig`/`ip addr` on that machine.
#
# Optional overrides (env vars):
#   ARDUPILOT_DIR          where ArduPilot is cloned+built (default: ~/ardupilot, then
#                          ~/Documents/gitClone/ardupilot, whichever exists first --
#                          same resolution as sim/run_sitl.sh)
#   ARDUPILOT_GAZEBO_DIR   where the ardupilot_gazebo plugin is cloned+built
#                          (default: ~/ardupilot_gazebo)
#   ARDUPILOT_VENV         the venv holding mavproxy/pymavlink/empy (default:
#                          ~/.venvs/ardupilot). Activated automatically -- without this,
#                          sim_vehicle.py fails with "[Errno 2] No such file or
#                          directory: 'mavproxy.py'" since mavproxy only exists inside
#                          this venv, not on the system PATH (found 2026-09-22).
#   GZ_WORLD               which world to launch (default: iris_runway.sdf -- the
#                          plugin's own stock world; swap for our own board-in-Gazebo
#                          world once that exists, per docs/04-roadmap-checklist.md)
#   OUT_PORT               UDP port the GCS listens on (default: 14550, QGC's own default)
#
# Any extra arguments are passed straight through to sim_vehicle.py.
set -euo pipefail

if [ -z "${GCS_IP:-}" ]; then
    echo "GCS_IP is required -- set it to the IP of the machine running QGroundControl." >&2
    echo "Find it there with: ifconfig | grep 'inet ' (macOS) or ip addr (Linux)" >&2
    echo "Example: GCS_IP=192.168.29.238 ./sim/run_gazebo.sh" >&2
    exit 1
fi

if [ -z "${DISPLAY:-}" ]; then
    echo "DISPLAY is not set -- this needs to run from a real graphical terminal session" >&2
    echo "on this machine (not a bare non-interactive SSH command), so Gazebo's GUI has" >&2
    echo "somewhere to render. Open a terminal on this machine's own desktop and retry." >&2
    exit 1
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Same resolution order as sim/run_sitl.sh -- explicit override first, then the two
# locations actually seen in practice across the team's machines.
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
    exit 1
fi

AP_GZ="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
if [ ! -d "$AP_GZ/build" ]; then
    echo "Can't find a built ardupilot_gazebo plugin at $AP_GZ/build" >&2
    echo "  export ARDUPILOT_GAZEBO_DIR=/path/to/your/ardupilot_gazebo" >&2
    echo "  or clone+build it: https://github.com/ArduPilot/ardupilot_gazebo" >&2
    exit 1
fi

BIN="$AP/build/sitl/bin/arducopter"
if [ ! -x "$BIN" ]; then
    echo "ArduPilot found at $AP but SITL isn't built yet. Run:" >&2
    echo "  cd $AP && ./waf configure --board sitl && ./waf copter" >&2
    exit 1
fi

VENV="${ARDUPILOT_VENV:-$HOME/.venvs/ardupilot}"
if [ ! -f "$VENV/bin/activate" ]; then
    echo "Can't find a Python venv with mavproxy/pymavlink at $VENV" >&2
    echo "  export ARDUPILOT_VENV=/path/to/your/venv" >&2
    echo "  or create one: python3 -m venv $VENV && source $VENV/bin/activate &&" >&2
    echo "    pip install empy==3.3.4 pexpect future pyyaml pymavlink MAVProxy" >&2
    exit 1
fi
# venv activate scripts reference $PS1, which is unset in a non-interactive script --
# temporarily relax -u around it rather than have that kill the whole script.
set +u
# shellcheck disable=SC1091
source "$VENV/bin/activate"
set -u

export GZ_SIM_SYSTEM_PLUGIN_PATH="$AP_GZ/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$AP_GZ/models:$AP_GZ/worlds:${GZ_SIM_RESOURCE_PATH:-}"

GZ_WORLD="${GZ_WORLD:-iris_runway.sdf}"
OUT_PORT="${OUT_PORT:-14550}"

echo "ArduPilot:       $AP"
echo "ardupilot_gazebo: $AP_GZ"
echo "World:           $GZ_WORLD"
echo "GCS (remote):    udp:${GCS_IP}:${OUT_PORT} (e.g. QGroundControl on Hari's Mac)"
echo "GCS (local):     udp:127.0.0.1:${OUT_PORT} (QGroundControl or MAVProxy's own"
echo "                 console/map, running on this same machine)"
echo ""

echo "Starting Gazebo ($GZ_WORLD) ..."
gz sim -v4 -r "$GZ_WORLD" &
GZ_PID=$!
trap 'kill $GZ_PID 2>/dev/null || true' EXIT

# Give Gazebo a few seconds to come up before SITL tries to connect to it.
sleep 5
if ! kill -0 "$GZ_PID" 2>/dev/null; then
    echo "Gazebo exited immediately -- check the output above for the actual error." >&2
    exit 1
fi

echo "Starting ArduPilot SITL (gazebo-iris, JSON model) ..."
cd "$AP"

# Second --out to localhost so a GCS (QGroundControl, or MAVProxy's own console/map,
# already passed below) running on THIS machine also gets telemetry, alongside the
# required remote GCS_IP -- both at once, not one instead of the other. Skipped if
# GCS_IP is already 127.0.0.1, so we don't send the same stream to the same place twice.
LOCAL_OUT_ARGS=()
if [ "$GCS_IP" != "127.0.0.1" ]; then
    LOCAL_OUT_ARGS=(--out="udp:127.0.0.1:${OUT_PORT}")
fi

# --add-param-file is passed explicitly for both files sim_vehicle.py's own frame
# resolution SAYS it should load (copter.parm, gazebo-iris.parm -- confirmed via its own
# printed frame_infos dict) but does NOT actually forward to the arducopter binary for
# "external" (JSON/Gazebo) frames -- a real gap in this ArduPilot version's sim_vehicle.py,
# found and worked around 2026-09-22. Without this, FRAME_CLASS/FRAME_TYPE stay unset and
# arming fails with "PreArm: Motors: Check frame class and type" / "Frame: UNSUPPORTED".
exec Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    --add-param-file="$AP/Tools/autotest/default_params/copter.parm" \
    --add-param-file="$AP/Tools/autotest/default_params/gazebo-iris.parm" \
    --out="udp:${GCS_IP}:${OUT_PORT}" \
    "${LOCAL_OUT_ARGS[@]}" \
    --console \
    "$@"
