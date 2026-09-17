# PX4 LoRa Precision Autoland

This project enables a PX4 drone to perform a fully autonomous precision landing in Gazebo Harmonic using a custom simulated LoRa ranging network (EKF) for long-distance homing and a downward-facing camera with AprilTag/YOLO detection for terminal precision landing.

## Prerequisites
- **ROS 2 Jazzy**
- **PX4-Autopilot** (main branch configured for Gazebo Harmonic/SITL)
- **MAVROS** (`ros-jazzy-mavros`)
- Python Dependencies: `cv2`, `apriltag`, `cv_bridge`
  ```bash
  pip3 install opencv-python apriltag --break-system-packages
  ```

## Setup & Installation

1. **Copy the Source Code**
   Copy the four folders inside `src/` to your ROS 2 workspace `src` directory:
   ```bash
   cp -r src/* ~/ros2_ws/src/
   ```

2. **Copy the Enlarged AprilTag Model to PX4**
   To ensure Gazebo rendering works perfectly, copy the custom `apriltag_pad` model directly into PX4's internal models directory:
   ```bash
   cp -r ~/ros2_ws/src/roland_lora_gazebo/models/apriltag_pad ~/PX4-Autopilot/Tools/simulation/gz/models/
   ```

3. **Build the Workspace**
   ```bash
   cd ~/ros2_ws
   source /opt/ros/jazzy/setup.bash
   colcon build
   ```

## Running the Simulation

You will need 4 separate terminals. Run these commands in order:

### Terminal 1: PX4 & Gazebo Simulator
```bash
unset VIRTUAL_ENV
export PX4_GZ_WORLD=landing_world
export PX4_GZ_Z=1.0
export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/src/roland_lora_gazebo/models:$HOME/ros2_ws/src/roland_lora_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ros2_ws/install/gz_lora_range_plugin/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
cd ~/PX4-Autopilot
make px4_sitl gz_x500_lora
```

### Terminal 2: MAVROS
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

### Terminal 3: ROS-Gazebo Bridges (Camera & LoRa)
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image /lora/ranging@std_msgs/msg/String[gz.msgs.StringMsg --ros-args -r /camera/image_raw:=/camera/image_raw -r /lora/ranging:=/gz/lora/ranging
```

### Terminal 4: Autoland Logic Nodes
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run roland_lora lora_bridge.py &
ros2 run roland_lora ekf_node &
ros2 run roland_lora vision_node.py &
sleep 2
ros2 run roland_lora autoland_node
```

## How It Works
1. **Takeoff & Patrol:** The drone automatically arms in Offboard mode, takes off to 2.5m, and visits several GPS waypoints.
2. **LoRa Homing:** Once the patrol is finished, the custom Gazebo LoRa plugin simulates noisy ranges to 4 ground anchors. The `ekf_node` filters this noise and guides the drone to the target vicinity.
3. **Vision Landing:** Once the drone is within 1 meter of the landing zone, the `vision_node` detects the 2x2 meter AprilTag. `autoland_node` uses this high-precision data to aggressively center the drone and plunge toward the pad.
4. **Touchdown:** At 30cm altitude, the node triggers PX4's native `AUTO.LAND` mode, safely cutting the motors upon touchdown.
