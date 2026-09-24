"""
Reads the real Gazebo-rendered downward camera feed via gz-transport directly (no ROS 2,
per the standing default in docs/06-open-questions.md Q11) -- the actual Stage 3 swap
sim/run_landing.py exists for: same detector/fusion/mavlink_out code, only the frame
source changes (per src/frame_source.py's BaseFrameSource contract).

Camera comes from sim/gazebo_models/iris_downward_camera/model.sdf's downward_camera
sensor, publishing on the fixed topic name below (set explicitly in that SDF, not
gz-sim's auto-generated naming, so this file can hardcode it).

Packaging note (confirmed 2026-09-23, Mac Mini): Gazebo Harmonic's Python bindings
(python3-gz-transport13, python3-gz-msgs10) are apt packages installed into the SYSTEM
python3's dist-packages, NOT visible inside a venv created without
--system-site-packages. Rather than requiring every machine's venv to be recreated with
that flag, this file adds /usr/lib/python3/dist-packages to sys.path directly -- same
kind of targeted sys.path fix sim/synthetic_camera.py already uses for src/sim imports,
just for this one packaging quirk.
"""

import sys
import time
import math
from typing import Any, Dict, Optional, Tuple

import numpy as np
from pymavlink import mavutil

from src.frame_source import BaseFrameSource

_SYSTEM_DIST_PACKAGES = "/usr/lib/python3/dist-packages"
if _SYSTEM_DIST_PACKAGES not in sys.path:
    sys.path.append(_SYSTEM_DIST_PACKAGES)

import gz.transport13 as gz_transport
import gz.msgs10.image_pb2 as gz_image_pb2

CAMERA_TOPIC = "precision_landing/camera/image_raw"

# Must match sim/gazebo_models/iris_downward_camera/model.sdf's <camera> block exactly,
# so camera_matrix reflects the geometry actually rendered. 1.1490 rad (~65.9deg) matches
# the senior's real calibrated Pi Camera V2 (fx=497.88 at 640x480,
# docs/01-inherited-system.md:99), not an arbitrary 90deg -- changed 2026-09-23 after a
# live mission test showed the 90deg placeholder made the 0.6m board too few pixels wide
# to detect except very close to the ground.
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_HORIZONTAL_FOV_RAD = 1.1490


def _camera_matrix_from_fov(width, height, horizontal_fov_rad):
    """
    fx from horizontal_fov + width (pinhole model: fov = 2*atan(width / (2*fx)));
    fy assumed equal to fx (square pixels, standard for a synthetic/simulated camera
    with no real lens distortion) -- NOT SyntheticSource's arbitrary fx=fy=500, this is
    computed from the real rendered geometry so solvePnP is correct.
    """
    fx = width / (2.0 * math.tan(horizontal_fov_rad / 2.0))
    fy = fx
    cx = width / 2.0
    cy = height / 2.0
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=float)


class GazeboSource(BaseFrameSource):
    def __init__(self, connection="udpin:0.0.0.0:14540", target_pos_ned=(0, 0, 0), own_connection=True):
        """
        connection: same MAVLink connection contract as SyntheticSource -- either a
        connection string or an existing mavutil connection object to share (see
        src/mavlink_out.py's own docstring for why sharing matters: two sockets cannot
        both bind the same local port in one process).
        """
        if isinstance(connection, str):
            self.master = mavutil.mavlink_connection(connection)
            self.own_connection = True
        else:
            self.master = connection
            self.own_connection = own_connection

        self.target_pos_ned = np.array(target_pos_ned, dtype=float)

        self.width = CAMERA_WIDTH
        self.height = CAMERA_HEIGHT
        self.camera_matrix = _camera_matrix_from_fov(
            self.width, self.height, CAMERA_HORIZONTAL_FOV_RAD
        )
        self.dist_coeffs = np.zeros(5, dtype=float)

        self.pos_ned = np.array([0.0, 0.0, 0.0])
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self._latest_frame = None

        self.node = gz_transport.Node()
        self.node.subscribe(gz_image_pb2.Image, CAMERA_TOPIC, self._on_image)

    def _on_image(self, msg):
        # RGB_INT8 is gz-sim's default camera pixel format when <format> isn't set in
        # the SDF (we didn't set one) -- confirmed against the actual message at
        # runtime is the first thing to check if colors ever look wrong/swapped.
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            frame_rgb = arr.reshape((msg.height, msg.width, 3))
        except ValueError:
            # Unexpected size/format -- drop this frame rather than crash the
            # subscriber thread.
            return
        self._latest_frame = frame_rgb[:, :, ::-1].copy()  # RGB -> BGR

    def _update_pose_from_mavlink(self):
        while True:
            msg = self.master.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE"], blocking=False)
            if not msg:
                break
            if msg.get_type() == "LOCAL_POSITION_NED":
                self.pos_ned = np.array([msg.x, msg.y, msg.z])
            elif msg.get_type() == "ATTITUDE":
                self.roll = msg.roll
                self.pitch = msg.pitch
                self.yaw = msg.yaw

    def get_frame(self) -> Tuple[Optional[np.ndarray], Optional[Dict[str, Any]]]:
        self._update_pose_from_mavlink()

        if self._latest_frame is None:
            return None, None

        pose_hint = {
            "pos_ned": self.pos_ned.copy(),
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
        }
        # .copy(), not the cached array itself -- sim/run_landing.py draws detection
        # overlays (drawDetectedMarkers, drawFrameAxes) directly onto whatever it gets
        # back, in place. The main loop runs faster than this camera's 30Hz update
        # rate, so get_frame() is often called twice between real Gazebo frames; without
        # this copy, the second call handed back the FIRST call's frame with debug
        # overlay lines already drawn straight over the tag pattern, breaking detection
        # on every other call -- the exact 1:1 Sent/lost alternation seen in a live
        # mission test 2026-09-24, at every altitude, not just near the ground.
        return self._latest_frame.copy(), pose_hint

    def close(self):
        if self.own_connection:
            self.master.close()


if __name__ == "__main__":
    # Standalone check: subscribe, wait for a real frame, save it and print its shape --
    # confirms the gz-transport subscription actually works before wiring into the full
    # detection loop. Doesn't need MAVLink for this, just the camera topic.
    import cv2

    node = gz_transport.Node()
    received = {}

    def _cb(msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        received["frame"] = arr.reshape((msg.height, msg.width, 3))[:, :, ::-1].copy()

    node.subscribe(gz_image_pb2.Image, CAMERA_TOPIC, _cb)

    print(f"Subscribed to {CAMERA_TOPIC}, waiting up to 10s for a frame...")
    start = time.time()
    while "frame" not in received and time.time() - start < 10:
        time.sleep(0.1)

    if "frame" in received:
        cv2.imwrite("/tmp/gazebo_camera_test.png", received["frame"])
        print(f"Got frame, shape={received['frame'].shape}, saved to /tmp/gazebo_camera_test.png")
    else:
        print("No frame received in 10s -- is Gazebo running with the camera-equipped world?")
