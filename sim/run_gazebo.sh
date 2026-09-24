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

# On a machine with a real desktop (e.g. the Mac Mini), run with the GUI on the real
# GPU as before. On a headless box with no DISPLAY (e.g. the Parallels Ubuntu VM), fall
# back to a virtual X server (Xvfb) + forced software rendering + server-only mode (-s,
# no GUI) instead of hard-refusing -- this VM's Ogre2 + virtio-gpu/virgl combo actually
# CRASHES (Ogre::UnimplementedException in GL3PlusTextureGpu::copyTo, hit generating
# hardware mipmaps for ANY textured camera sensor -- confirmed 2026-09-24 not specific to
# our own board model, the plugin's own stock iris_runway.sdf crashes the same way) when
# Ogre2 falls back to its EGL headless-device path on its own; giving it a real (virtual)
# GLX context via Xvfb avoids that code path entirely and camera sensors render
# correctly. LIBGL_ALWAYS_SOFTWARE=1 alone, without Xvfb, is NOT sufficient by itself --
# docs/07-sim-setup.md's earlier note that it was "the fix" was verified only against
# physics, never against an actual camera sensor.
HEADLESS_GAZEBO_ARGS=()
if [ -z "${DISPLAY:-}" ]; then
    if command -v xvfb-run >/dev/null 2>&1; then
        echo "DISPLAY is not set -- no GUI available here, using Xvfb + forced software" >&2
        echo "rendering + server-only mode instead (found 2026-09-24 on the Parallels VM)." >&2
        export LIBGL_ALWAYS_SOFTWARE=1
        HEADLESS_GAZEBO_ARGS=(-s)
        USE_XVFB=1
    else
        echo "DISPLAY is not set and xvfb-run isn't installed -- either run this from a" >&2
        echo "real graphical terminal session on this machine, or:" >&2
        echo "  sudo apt-get install -y xvfb" >&2
        exit 1
    fi
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
# This repo's own models/worlds (sim/gazebo_models, sim/gazebo_worlds) come first, so
# our own precision_landing_board / iris_downward_camera are found ahead of anything
# same-named in the plugin's own resource dirs.
export GZ_SIM_RESOURCE_PATH="$HERE/gazebo_models:$HERE/gazebo_worlds:$AP_GZ/models:$AP_GZ/worlds:${GZ_SIM_RESOURCE_PATH:-}"

# Headless (Xvfb) machines default to the lightweight VM world (trimmed rendering
# cost only -- physics/mass/collisions unchanged, see that file's own header) instead of
# the full-quality one, unless GZ_WORLD is explicitly set to override this.
if [ "${USE_XVFB:-0}" = "1" ]; then
    GZ_WORLD="${GZ_WORLD:-precision_landing_world_vm.sdf}"
else
    GZ_WORLD="${GZ_WORLD:-precision_landing_world.sdf}"
fi
OUT_PORT="${OUT_PORT:-14550}"
COMPANION_PORT="${COMPANION_PORT:-14540}"

echo "ArduPilot:       $AP"
echo "ardupilot_gazebo: $AP_GZ"
echo "World:           $GZ_WORLD"
echo "GCS (remote):    udp:${GCS_IP}:${OUT_PORT} (e.g. QGroundControl on Hari's Mac)"
echo "GCS (local):     udp:127.0.0.1:${OUT_PORT} (QGroundControl or MAVProxy's own"
echo "                 console/map, running on this same machine)"
echo "Companion port:  UDP $COMPANION_PORT (sim/run_landing.py connects here)"
echo ""

echo "Starting Gazebo ($GZ_WORLD) ..."
if [ "${USE_XVFB:-0}" = "1" ]; then
    xvfb-run -a --server-args='-screen 0 1024x768x24' gz sim -v4 -r "${HEADLESS_GAZEBO_ARGS[@]}" "$GZ_WORLD" &
else
    gz sim -v4 -r "${HEADLESS_GAZEBO_ARGS[@]}" "$GZ_WORLD" &
fi
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
#
# sim/precision_landing.parm is ALSO required here, and was missing entirely until
# 2026-09-24 -- PLND_ENABLED/PLND_TYPE (which turn precision landing on at all) only
# exist in that file, not in ArduPilot's own default_params/. Without it, every Gazebo
# mission run so far had precision landing fully DISABLED regardless of whether
# LANDING_TARGET messages were well-formed: the vehicle detects the tag, "corrects"
# nothing, and lands wherever the mission's own GPS waypoint said to -- exactly the
# symptom hit testing sim/test_mission_precision.waypoints. Loaded LAST so it can't be
# shadowed by the two files above (later --add-param-file wins on a shared key; none of
# copter.parm/gazebo-iris.parm actually set PLND_* anyway, so this is defensive, not
# currently load-bearing).
# --console opens MAVProxy's own Tk/wx GUI status window -- skip it under Xvfb/headless
# (same "no display" problem gz-sim itself hit above). The actual MAVProxy command
# prompt (wp load, mode LAND, etc.) is on this terminal's stdin/stdout regardless of
# --console, so nothing interactive is lost by skipping it.
MAVPROXY_CONSOLE_ARGS=(--console)
if [ "${USE_XVFB:-0}" = "1" ]; then
    MAVPROXY_CONSOLE_ARGS=()
fi

exec Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    --add-param-file="$AP/Tools/autotest/default_params/copter.parm" \
    --add-param-file="$AP/Tools/autotest/default_params/gazebo-iris.parm" \
    --add-param-file="$HERE/precision_landing.parm" \
    --out="udp:${GCS_IP}:${OUT_PORT}" \
    --out="udp:127.0.0.1:${COMPANION_PORT}" \
    "${LOCAL_OUT_ARGS[@]}" \
    "${MAVPROXY_CONSOLE_ARGS[@]}" \
    "$@"
