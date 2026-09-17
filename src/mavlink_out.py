"""
Sends LANDING_TARGET to ArduPilot over MAVLink.

Built and tested BEFORE any detection/fusion code exists (see the Stage 2 plan) so the
MAVLink/frame/param wiring is proven independently of whether the vision pipeline works.
Feed it hardcoded fake positions first (sim/demo_mavlink_sweep.py) against live SITL with
PLND_TYPE=1, confirm the drone corrects the same way it did against SIM_PLD in Stage 1,
*then* wire in real detection.

Frame: MAV_FRAME_BODY_FRD. CLAUDE.md section 7 item 0 and the MAVLink docs
(https://mavlink.io/en/services/landing_target.html) both confirm ArduPilot rejects
MAV_FRAME_BODY_NED outright -- this is the exact bug flagged in
Documentation/dinesh_reference_code/drone_test_landing.py, not repeated here.

Mode: angle-based (angle_x, angle_y, distance; position_valid left unset/0), matching
Dinesh's reference script and the Landmark Landing reference implementation -- the
better-tested path in the ArduPilot ecosystem versus full xyz position mode.
"""

import time

import numpy as np
from pymavlink import mavutil


class LandingTargetSender:
    def __init__(self, connection="udpin:0.0.0.0:14540", own_connection=True):
        """
        connection: either a connection string, or an existing mavutil connection object
        to reuse (pass one in and set own_connection=False when something else, e.g.
        a frame source reading vehicle telemetry, already holds a connection on the same
        port -- a single MAVLink link is bidirectional, and two separate sockets cannot
        both bind the same local port in the same process).

        Default listens on the dedicated companion port sim/run_sitl.sh sets up via a
        second MAVProxy --out link (COMPANION_PORT, default 14540) -- NOT SITL's raw
        tcp:5760. ArduPilot's SITL TCP serial driver only tracks one client connection
        per port (confirmed 2026-09-15 by tracing AP_HAL_SITL/UARTDriver.cpp's accept()
        call into a single _fd) -- a second client there connects at the TCP layer but
        never receives a heartbeat, since the driver only services whichever client
        connected first (MAVProxy, in the normal startup order). This port instead
        mirrors the already-working QGroundControl/Mission Planner pattern (their own
        dedicated UDP link on OUT_PORT), just not sharing their port.
        On real hardware this becomes the serial port instead (e.g. '/dev/serial0').
        """
        if isinstance(connection, str):
            self.master = mavutil.mavlink_connection(
                connection,
                source_system=1,
                source_component=mavutil.mavlink.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY,
            )
            self.own_connection = True
        else:
            self.master = connection
            self.own_connection = own_connection
        self.boot_time = time.time()

    def wait_ready(self, timeout=15):
        hb = self.master.wait_heartbeat(timeout=timeout)
        return hb is not None

    def send(self, body_x_m, body_y_m, dist_m, tag_size_m=0.24):
        """
        body_x_m: forward offset of target from vehicle, metres (FRD)
        body_y_m: right offset of target from vehicle, metres (FRD)
        dist_m:   distance to target, metres
        tag_size_m: physical size of the tag used for angular-size estimation
        """
        if dist_m > 0.1:
            angle_x = float(np.arctan2(body_x_m, dist_m))
            angle_y = float(np.arctan2(body_y_m, dist_m))
        else:
            angle_x = 0.0
            angle_y = 0.0

        # Angular size must be non-zero or ArduPilot's LANDING_TARGET rejection logic
        # discards the message outright (CLAUDE.md section 7 item 2).
        size = float(2.0 * np.arctan2(tag_size_m / 2.0, max(dist_m, 0.1)))

        time_usec = int((time.time() - self.boot_time) * 1e6)

        self.master.mav.landing_target_send(
            time_usec,
            0,                                          # target_num
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            angle_x,
            angle_y,
            float(dist_m),
            size,
            size,
        )

    def close(self):
        if self.own_connection:
            self.master.close()
