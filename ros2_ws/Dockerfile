# ROS 2 Jazzy for this project.
# Runs on Hari's M1 (arm64) and on the teammate's Linux Mint box (amd64) —
# same image definition, Docker picks the right base.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        ros-jazzy-launch-ros \
    && rm -rf /var/lib/apt/lists/*

# pymavlink is what the bridge node uses to talk to ArduPilot.
RUN pip3 install --no-cache-dir --break-system-packages pymavlink

WORKDIR /ws
CMD ["bash"]
