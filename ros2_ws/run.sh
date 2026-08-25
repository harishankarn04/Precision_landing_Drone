#!/usr/bin/env bash
# Open a shell in the ROS 2 container with this workspace mounted.
#
#   ./ros2_ws/run.sh              -> interactive shell
#   ./ros2_ws/run.sh <command>    -> run one command and exit
#
# Inside, build and run with:
#   colcon build --symlink-install && source install/setup.bash
#   ros2 launch precision_landing bridge.launch.py
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

docker run --rm -it \
    -v "${WS_DIR}:/ws" \
    -w /ws \
    --add-host=host.docker.internal:host-gateway \
    precision-landing-ros2 \
    "${@:-bash}"
