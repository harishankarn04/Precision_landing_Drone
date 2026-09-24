"""
Direct GUIDED-mode precision-landing controller, bypassing ArduPilot's own PLND_TYPE/
LANDING_TARGET machinery entirely.

Built 2026-09-24 after PLND_TYPE=1 repeatedly hit "PrecLand: Init Failed" against real
Gazebo detections -- AC_PrecLand.cpp requires a LANDING_TARGET update at least every
EKF_INIT_SENSOR_MIN_UPDATE_MS (500ms) for a full EKF_INIT_TIME_MS (2000ms) window before
it will ever use the data, and that kept failing even with sim/run_landing.py's --no-gui
(so it wasn't just render/GUI overhead on our side) -- alongside an unrelated
"ArduPilotPlugin controller has reset" seen on the same runs, independent of PLND. Rather
than keep chasing an opaque internal ArduPilot timing requirement we don't fully
control, this owns the correction directly.

Architecture ported from a teammate's proven ROS2/MAVROS autoland_node.cpp
(~/Documents/gitClone/lora-precision-landing/roland_lora_ardupilot/src/autoland_node.cpp)
-- same shape: GUIDED velocity setpoints, clamped to a conservative max speed, switching
to ArduPilot's own real LAND mode only once centered and near the ground (for the actual
controlled disarm, not for any correction). Two differences from that reference: plain
pymavlink here instead of MAVROS/ROS2 (matching this project's own ROS2-free default,
docs/06-open-questions.md Q11), and driven by our own metric solvePnP-based fused
body_x/body_y (metres, src/fusion.py) instead of their raw pixel error -- a strictly
better error signal since it's already in real-world units, not pixels.
"""

import time

from pymavlink import mavutil

# MAV_FRAME_BODY_OFFSET_NED: velocities expressed forward/right/down relative to the
# vehicle's own current heading -- matches src/fusion.py's body_x (forward)/body_y
# (right) convention directly, no extra rotation needed here.
_FRAME_BODY_VELOCITY = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED

# type_mask for SET_POSITION_TARGET_LOCAL_NED, "velocity only": ignore position
# (bits 0-2), ignore acceleration (bits 6-8), ignore yaw/yaw_rate (bits 10-11).
# Standard value used across ArduPilot GUIDED-velocity-control references.
_TYPE_MASK_VELOCITY_ONLY = 0x0DC7

MAX_LATERAL_SPEED_MS = 0.4       # clamp -- matches the teammate's proven conservative limit
DESCENT_SPEED_MS = 0.4           # constant descend rate once tracking, +down (FRD)
SEARCH_DESCENT_SPEED_MS = 0.15   # slower, cautious descent while still hunting for the tag
LATERAL_GAIN = 0.8               # m/s per metre of body-frame error, clamped above
TARGET_LOST_TIMEOUT_S = 1.0      # below this, treat a brief dropout as noise, not "lost"
LAND_ALTITUDE_M = 0.3            # switch to real LAND mode below this AGL for final disarm


def _clamp(value, limit):
    return max(-limit, min(limit, value))


class GuidedLander:
    """
    Call send_correction(body_x_m, body_y_m) once per vision-loop iteration when a
    target is detected (src/fusion.py's fused['body_x']/['body_y']), or on_target_lost()
    when it isn't. Handles its own GUIDED velocity setpoints and the GUIDED -> LAND mode
    switch near the ground -- the caller doesn't need to touch flight modes itself.

    Vehicle must already be armed and in GUIDED mode, airborne, before calling either
    method -- this class only sends velocity setpoints and the one GUIDED->LAND
    transition, it doesn't arm/takeoff/mission-manage.
    """

    def __init__(self, master):
        self.master = master
        self._last_target_t = 0.0
        self._switched_to_land = False

    def _current_altitude_m(self):
        """
        Positive-up altitude above home, from LOCAL_POSITION_NED.z (NED: down-positive,
        so altitude = -z). Relies on that message already streaming on this link --
        already the case by default (src/frame_source.py's pose_hint has used this same
        message type since Stage 2).
        """
        msg = self.master.messages.get("LOCAL_POSITION_NED")
        if msg is None:
            return None
        return -msg.z

    def _send_velocity(self, vx, vy, vz):
        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms -- unused by ArduPilot for velocity-only setpoints
            self.master.target_system,
            self.master.target_component,
            _FRAME_BODY_VELOCITY,
            _TYPE_MASK_VELOCITY_ONLY,
            0, 0, 0,        # position (ignored)
            vx, vy, vz,     # velocity, body FRD (forward/right/down)
            0, 0, 0,        # acceleration (ignored)
            0, 0,           # yaw, yaw_rate (ignored)
        )

    def _maybe_switch_to_land(self):
        if self._switched_to_land:
            return
        alt = self._current_altitude_m()
        if alt is not None and alt < LAND_ALTITUDE_M:
            self.master.set_mode_apm("LAND")
            self._switched_to_land = True
            print(f"GuidedLander: altitude {alt:.2f}m < {LAND_ALTITUDE_M}m -- "
                  f"switching to LAND for final disarm")

    def send_correction(self, body_x_m, body_y_m):
        """Target detected this frame -- lateral correction + constant descent."""
        self._last_target_t = time.time()
        if self._switched_to_land:
            return  # ArduPilot owns the descent now, don't fight it with GUIDED setpoints
        vx = _clamp(LATERAL_GAIN * body_x_m, MAX_LATERAL_SPEED_MS)
        vy = _clamp(LATERAL_GAIN * body_y_m, MAX_LATERAL_SPEED_MS)
        self._send_velocity(vx, vy, DESCENT_SPEED_MS)
        self._maybe_switch_to_land()

    def on_target_lost(self):
        """No detection this frame -- hover laterally, keep a cautious slow descent."""
        if self._switched_to_land:
            return
        if time.time() - self._last_target_t > TARGET_LOST_TIMEOUT_S:
            self._send_velocity(0.0, 0.0, SEARCH_DESCENT_SPEED_MS)
        self._maybe_switch_to_land()
