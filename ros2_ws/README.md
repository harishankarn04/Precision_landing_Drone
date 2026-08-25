# ros2_ws — ROS 2 side of the project

**Status: working.** A ROS 2 node commands a precision landing in ArduPilot SITL.
Verified 2026-08-20: started 2.03 m from the pad, touched down **0.01 m** from it.

```
   ROS 2 (Docker, arm64)                  macOS host
  ┌───────────────────────┐            ┌──────────────────┐
  │ fake_target           │            │ ArduPilot SITL   │
  │   └─ /landing_target ─┼──┐         │  (ArduCopter)    │
  │ mavlink_bridge        │  │ MAVLink │                  │
  │   └─ /drone/pose   ◄──┼──┴─────────┤ SERIAL1 TCP 5762 │
  └───────────────────────┘            └──────────────────┘
```

`mavlink_bridge` is the only node that speaks MAVLink. Everything else is plain
ROS 2. On the real aircraft this same boundary exists — the Raspberry Pi talks
MAVLink over a UART to the flight controller — so the perception nodes we write
later will not change when we move to hardware.

`fake_target` stands in for the camera. It is thrown away once the AprilTag
detector exists; nothing downstream changes.

---

## Run it

**1. Build the image** (once):
```bash
cd ros2_ws && docker build -t precision-landing-ros2 .
```

**2. Build the workspace** (once, and after adding nodes):
```bash
./ros2_ws/run.sh bash -lc "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"
```
`--symlink-install` means editing a `.py` file takes effect on restart, no rebuild.

**3. Start SITL** (terminal 1). This loads `sim/precision_landing.parm` at boot,
so `PLND_ENABLED` is set before the backend is built:
```bash
./sim/run_sitl.sh
```

**4. Start the ROS 2 nodes** (terminal 2):
```bash
./ros2_ws/run.sh bash -lc "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch precision_landing bridge.launch.py pad_north:=2.0"
```

**5. Fly it** (terminal 3):
```bash
.venv/bin/python sim/demo_flight.py --pad-north 2.0 --alt 6
```
It waits for the autopilot to become armable, takes off, switches to LAND, and
prints the start and final error.

To fly by hand instead, point MAVProxy or QGroundControl at TCP 5760 and use
`mode GUIDED` → `arm throttle` → `takeoff 6` → `mode LAND`.

---

## Four things that cost real debugging time

Each of these fails **silently** — no error, the drone just lands in the wrong place.

### 1. `PLND_ENABLED` needs a reboot
It is an enable-flag: the backend is constructed at boot. Setting it on a running
autopilot changes the parameter but never creates the driver, so `LANDING_TARGET`
messages are received and quietly discarded. Set it, restart, then fly.

### 2. A new MAVLink link is silent until you ask
ArduPilot streams telemetry **per link**. A ground station requesting data on
SERIAL0 does nothing for a companion computer on SERIAL1 — that link gets a
heartbeat and nothing else. The bridge sends `MAV_CMD_SET_MESSAGE_INTERVAL` for
`LOCAL_POSITION_NED` on connect. Same applies to the Pi's UART on real hardware.

### 3. The frame must be `MAV_FRAME_BODY_FRD`
From `libraries/AC_PrecLand/AC_PrecLand_MAVLink.cpp`:
```c
if (packet.frame != MAV_FRAME_BODY_FRD && packet.frame != MAV_FRAME_LOCAL_FRD) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Plnd: Frame not supported");
    return;
}
```
⚠️ **The M.Tech thesis specifies `MAV_FRAME_BODY_NED`.** Current ArduPilot rejects
that outright. Either the thesis targeted an older release or it is imprecise —
either way, **use `BODY_FRD`.** This was the single cause of "targets sent, nothing
happens."

### 4. Prefer `position_valid=1`
With `position_valid=0`, ArduPilot rebuilds the direction from the angle fields as
`(-tan(angle_y), tan(angle_x), 1)` — so `angle_x` encodes **right** and `angle_y`
encodes **negated forward**. Easy to get backwards, and a sign error flies the
aircraft *away* from the pad. Setting `position_valid=1` and sending the offset
vector `(forward, right, down)` directly avoids the trap. FRD is already our
convention, so no conversion is needed.

### 5. SERIAL1 doesn't exist until a GCS connects to SERIAL0
SITL binds TCP 5762 only *after* something connects to TCP 5760, so a bridge
started first gets `ConnectionRefusedError`. The bridge now retries on a loop,
which makes launch order irrelevant.

### 7. GUIDED and armed constrain each other from opposite directions
A cold SITL **rejects mode changes** for ~30 s while the EKF settles — so setting
GUIDED first silently fails, the vehicle arms in the wrong mode, and TAKEOFF is
refused (looks like a broken takeoff). But a vehicle left in **LAND** by a previous
run **refuses to arm at all** — so arming first fails too. There is no correct
order: push both every cycle until both stick. `demo_flight.py` does this.

### 6. A fresh SITL is not armable for ~30 s
The EKF has to settle and GPS has to lock. Arming before then fails and the
takeoff silently does nothing. `demo_flight.py` polls until arming succeeds.

---

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/drone/pose` | `geometry_msgs/PoseStamped` | ArduPilot → ROS 2 (NED, from `LOCAL_POSITION_NED`) |
| `/landing_target` | `geometry_msgs/PointStamped` | ROS 2 → ArduPilot (body FRD, metres) |

`/landing_target` is `x` forward, `y` right, `z` down — `z` is the height above the pad.

---

## Next

Replace `fake_target` with a real detector node that renders the AprilTag board
from the simulated camera and publishes the same `/landing_target` message.
Nothing else changes. See `../docs/07-sim-setup.md`.
