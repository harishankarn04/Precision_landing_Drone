import time
import math
import cv2
import numpy as np
from typing import Tuple, Optional, Dict, Any
from pymavlink import mavutil
import os
import sys

# Add the parent directory to sys.path to allow importing from src and sim
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.frame_source import BaseFrameSource
from sim.board import BOARD_SIZE_M, render_board_texture

class SyntheticSource(BaseFrameSource):
    def __init__(self, connection="udpin:0.0.0.0:14540", target_pos_ned=(0, 0, 0), own_connection=True):
        """
        connection: either a connection string, or an existing mavutil connection object
        to reuse (pass one in and set own_connection=False when something else, e.g. a
        LandingTargetSender, already holds a connection on the same port -- a single
        MAVLink link is bidirectional, and two separate sockets cannot both bind the same
        local port in the same process). Default port is 14540 (COMPANION_PORT in
        sim/run_sitl.sh), NOT 14550 -- that's the GCS port QGroundControl/Mission Planner
        already occupies; binding there either steals their connection or fails outright.
        target_pos_ned: The NED position of the target board in the world.
                        Usually (0, 0, 0) or the SITL home position.
        """
        if isinstance(connection, str):
            self.master = mavutil.mavlink_connection(connection)
            self.own_connection = True
        else:
            self.master = connection
            self.own_connection = own_connection
        self.target_pos_ned = np.array(target_pos_ned, dtype=float)
        
        # Camera intrinsics
        self.width = 640
        self.height = 480
        self.fx = 500.0
        self.fy = 500.0
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0
        self.camera_matrix = np.array([
            [self.fx, 0,       self.cx],
            [0,       self.fy, self.cy],
            [0,       0,       1]
        ], dtype=float)
        self.dist_coeffs = np.zeros(5, dtype=float)
        
        # Latest pose
        self.pos_ned = np.array([0.0, 0.0, -10.0]) # Default 10m high
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_pose_time = time.time()
        
        # Generate the 2D texture of the board
        self._build_board_texture()
        
    def _build_board_texture(self):
        """Builds a high-res 2D image of the board. 1 pixel = 1 mm, board is 600x600 mm.

        Rendering itself lives in sim/board.py's render_board_texture() -- the single
        source of truth also used to generate the Gazebo board model's texture
        (sim/gazebo_models/precision_landing_board/), so tag placement math exists in
        exactly one place.
        """
        self.tex_size = int(BOARD_SIZE_M * 1000)
        self.texture = render_board_texture(px_per_meter=1000)

        # The corners of the board in the WORLD frame (assuming board is at target_pos_ned)
        # Board +X is right (East), +Y is forward (North)
        # Let's say the board is flat on the ground.
        half = BOARD_SIZE_M / 2.0
        # Top-left (North-West)
        self.board_corners_3d = np.array([
            [self.target_pos_ned[0] + half, self.target_pos_ned[1] - half, self.target_pos_ned[2]], # TL (NW)
            [self.target_pos_ned[0] + half, self.target_pos_ned[1] + half, self.target_pos_ned[2]], # TR (NE)
            [self.target_pos_ned[0] - half, self.target_pos_ned[1] + half, self.target_pos_ned[2]], # BR (SE)
            [self.target_pos_ned[0] - half, self.target_pos_ned[1] - half, self.target_pos_ned[2]]  # BL (SW)
        ], dtype=float)
        
        # The corresponding corners in the texture image
        self.board_corners_2d = np.array([
            [0, 0],
            [self.tex_size, 0],
            [self.tex_size, self.tex_size],
            [0, self.tex_size]
        ], dtype=np.float32)

    def _update_pose_from_mavlink(self):
        """Polls MAVLink for LOCAL_POSITION_NED and ATTITUDE."""
        while True:
            msg = self.master.recv_match(type=['LOCAL_POSITION_NED', 'ATTITUDE'], blocking=False)
            if not msg:
                break
            
            if msg.get_type() == 'LOCAL_POSITION_NED':
                self.pos_ned = np.array([msg.x, msg.y, msg.z])
                self.last_pose_time = time.time()
            elif msg.get_type() == 'ATTITUDE':
                self.roll = msg.roll
                self.pitch = msg.pitch
                self.yaw = msg.yaw
                self.last_pose_time = time.time()

    def get_frame(self) -> Tuple[Optional[np.ndarray], Optional[Dict[str, Any]]]:
        self._update_pose_from_mavlink()
        
        # Calculate R_W2B (World to Body)
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        
        # Z-Y-X rotation
        R_z = np.array([[cy, sy, 0], [-sy, cy, 0], [0, 0, 1]])
        R_y = np.array([[cp, 0, -sp], [0, 1, 0], [sp, 0, cp]])
        R_x = np.array([[1, 0, 0], [0, cr, sr], [0, -sr, cr]])
        R_W2B = R_x @ R_y @ R_z
        
        # R_B2C (Body to Camera). Camera is looking down.
        # Body +X (forward) -> Camera -Y (up in image)
        # Body +Y (right) -> Camera +X (right in image)
        # Body +Z (down) -> Camera +Z (optical axis)
        R_B2C = np.array([
            [0, 1, 0],
            [-1, 0, 0],
            [0, 0, 1]
        ], dtype=float)
        
        R_W2C = R_B2C @ R_W2B
        t_W2B = self.pos_ned
        
        # tvec = - R_W2C * t_W2B
        tvec = - (R_W2C @ t_W2B)
        rvec, _ = cv2.Rodrigues(R_W2C)
        
        # Project board corners
        imgpts, _ = cv2.projectPoints(self.board_corners_3d, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        imgpts = np.int32(imgpts).reshape(-1, 2)
        
        # Check if the board is behind the camera (Z < 0 in camera frame)
        corners_c = []
        for p in self.board_corners_3d:
            pc = R_W2C @ (p - t_W2B)
            corners_c.append(pc)
        
        # Create empty image (ground color, e.g., dark gray)
        frame = np.ones((self.height, self.width, 3), dtype=np.uint8) * 100
        
        # Only draw if all corners are in front of the camera
        if all(c[2] > 0 for c in corners_c):
            # Compute perspective transform from texture to image
            imgpts_f = np.float32(imgpts)
            M = cv2.getPerspectiveTransform(self.board_corners_2d, imgpts_f)
            warped_board = cv2.warpPerspective(self.texture, M, (self.width, self.height))
            
            # Mask and composite
            mask = np.zeros((self.height, self.width), dtype=np.uint8)
            cv2.fillConvexPoly(mask, imgpts, 255)
            mask_inv = cv2.bitwise_not(mask)
            
            bg = cv2.bitwise_and(frame, frame, mask=mask_inv)
            fg = cv2.bitwise_and(warped_board, warped_board, mask=mask)
            frame = cv2.add(bg, fg)
            
        pose_hint = {
            'pos_ned': self.pos_ned.copy(),
            'roll': self.roll,
            'pitch': self.pitch,
            'yaw': self.yaw,
            'rvec': rvec,
            'tvec': tvec
        }
        
        return frame, pose_hint

    def close(self):
        if self.own_connection:
            self.master.close()

if __name__ == "__main__":
    # Test the synthetic camera standalone
    source = SyntheticSource("udpin:0.0.0.0:14540")
    # Mock a pose
    source.pos_ned = np.array([0, 0, -5.0]) # 5m above the board
    source.roll = 0.1 # slight roll
    frame, hint = source.get_frame()
    cv2.imwrite("synthetic_test.png", frame)
    print("Wrote synthetic_test.png")
