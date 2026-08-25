#!/usr/bin/env bash
# Launch Webots with ArduPilot's camera-equipped iris quadrotor.
#
#   ./sim/run_webots.sh
#
# Webots opens showing the drone. Press the ▶ Play button, then start
# ./sim/run_sitl_webots.sh in another terminal.
set -euo pipefail

AP="${ARDUPILOT_DIR:-$HOME/Documents/gitClone/ardupilot}"
WORLD="${1:-$AP/libraries/SITL/examples/Webots_Python/worlds/iris_camera.wbt}"
WEBOTS="/Applications/Webots.app/Contents/MacOS/webots"

if [ ! -x "$WEBOTS" ]; then
    echo "Webots not found at $WEBOTS" >&2
    echo "Install it from https://cyberbotics.com/ then re-run." >&2
    exit 1
fi

if [ ! -f "$WORLD" ]; then
    echo "World not found: $WORLD" >&2
    exit 1
fi

echo "opening $WORLD"
exec "$WEBOTS" "$WORLD"
