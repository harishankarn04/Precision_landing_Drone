#!/usr/bin/env python3

"""
DRONE PRECISION LANDING - COMPLETE PRODUCTION SCRIPT
=====================================================

For static and dynamic platform landing using nested AprilTag markers.

HARDWARE:
  - Raspberry Pi 4B 8GB
  - Pi Camera V2 (mounted on belly, pointing down, ribbon toward TAIL)
  - Matek H743-SLIM V3 flight controller (ArduPilot)
  - 5-inch iFlight XL5 frame

MARKER LAYOUT (nested, on 60cm board):
  TAG 0 (24cm) at center
  TAG 1 (8cm) at top-left:     offset (-22cm forward, -22cm right) wait...
  
  Let me clarify the offset coordinate system:
    - In MARKER frame (looking at the board from above):
      X = right when looking at marker
      Y = up/forward when looking at marker
    - When mounted on ground for LANDING:
      MARKER X = body Y (right of drone)
      MARKER Y = body X (forward of drone)
  
  Marker positions on board (in marker frame, meters):
    TAG 0: ( 0.00,  0.00) - center
    TAG 1: (-0.22, +0.22) - top-left
    TAG 2: (+0.22, +0.22) - top-right  
    TAG 3: (-0.22, -0.22) - bottom-left
    TAG 4: (+0.22, -0.22) - bottom-right

FEATURES:
  - AprilTag tag36h11 detection (more robust than ArUco)
  - Multi-marker fusion: uses whichever tag(s) give best pose
  - Automatic transition from large to small markers as drone descends
  - Weighted least-squares velocity estimation with outlier rejection
  - Position prediction for moving platforms
  - Comprehensive MAVLink integration with ArduPilot

DEPENDENCIES:
  pip install pupil-apriltags numpy opencv-python pymavlink picamera2
  
  If pupil-apriltags fails to install on Pi:
    sudo apt install python3-apriltag
    (then change the import below from pupil_apriltags to apriltag)

USAGE:
  python3 drone_precision_landing.py
  
MONITORING:
  tail -f ~/precision_landing/drone.log
"""

import cv2
import numpy as np
import time
import logging
import os
import sys
import traceback
from collections import deque
from threading import Thread, Lock
from datetime import datetime
from pymavlink import mavutil
from picamera2 import Picamera2

# AprilTag detection - try pupil-apriltags first, fall back to apriltag
try:
    from pupil_apriltags import Detector as AprilTagDetector
    APRILTAG_LIB = 'pupil_apriltags'
except ImportError:
    try:
        import apriltag
        APRILTAG_LIB = 'apriltag'
    except ImportError:
        print("ERROR: No AprilTag library installed!")
        print("  Try: pip install pupil-apriltags")
        print("  Or:  sudo apt install python3-apriltag")
        sys.exit(1)

# ============================================================
#                      CONFIGURATION
# ============================================================

# --- Flight Controller ---
FC_CONNECTION = '/dev/serial0'
BAUD_RATE     = 921600

# --- Camera ---
CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480
CAMERA_FPS    = 30

# --- Camera mounting offset from CG (metres, body FRD frame) ---
CAM_OFFSET_X = 0.0     # forward positive
CAM_OFFSET_Y = 0.0     # right positive
CAM_OFFSET_Z = 0.05    # down positive (camera below CG)

# --- Marker layout (AprilTag tag36h11) ---
TAG_FAMILY = 'tag36h11'

# Physical sizes of each tag (the BLACK SQUARE only, not white border) in METERS
# CRITICAL: Measure with ruler after printing - update if different!
MARKER_SIZES = {
    0: 0.24,    # Center tag - 24cm
    1: 0.08,    # Corner tags - 8cm each
    2: 0.08,
    3: 0.08,
    4: 0.08,
}

# Position offset of each marker's CENTER from the BOARD CENTER (landing target)
# in METERS, measured in marker plane frame (X=right when looking down at board,
# Y=forward when looking down at board with the drone's nose pointing forward)
MARKER_OFFSETS = {
    0: ( 0.00,  0.00),   # center
    1: (-0.22, +0.22),   # top-left  (X=-22cm right, Y=+22cm forward)
    2: (+0.22, +0.22),   # top-right
    3: (-0.22, -0.22),   # bottom-left
    4: (+0.22, -0.22),   # bottom-right
}

# --- Detection limits ---
MIN_DETECT_DIST   = 0.15    # m - below this, can't reliably estimate pose
MAX_DETECT_DIST   = 15.0    # m - above this, marker too small
MAX_DETECTION_AGE = 1.0     # seconds before "target lost"

# Detection quality thresholds (AprilTag-specific)
MIN_DECISION_MARGIN = 25.0  # AprilTag confidence (0-100, higher is better)

# Reprojection sanity check: after solvePnP, re-project the tag corners with
# the solved pose and compare to the detected corners. A good pose fits within
# ~1px; a failed/ambiguous solve produces large error. Reject anything above
# this. Cheap insurance against bad poses reaching the FC.
MAX_REPROJ_ERR_PX = 4.0

# Edge buffer - reject detections too close to image edge (unstable pose)
EDGE_MARGIN_PX = 15

# --- Velocity estimation ---
VELOCITY_HISTORY_SIZE  = 20    # frames (~0.67s at 30Hz)
VELOCITY_MIN_POINTS    = 6
VELOCITY_WEIGHT_TAU    = 0.3   # seconds - exponential weight time constant
VELOCITY_OUTLIER_SIGMA = 3.0
MAX_REASONABLE_SPEED   = 5.0   # m/s

# --- Safety limits ---
MAX_SAFE_VELOCITY = 1.0     # m/s - warn if platform faster
MAX_SAFE_ROLL     = 50.0    # degrees
MAX_SAFE_PITCH    = 50.0    # degrees

# --- Prediction (lag compensation) ---
# IMPORTANT: For a STATIC pad, keep this OFF. A stationary target has zero
# real velocity, so any velocity the estimator reports is pure pose jitter.
# With prediction ON, that jitter (often 0.5-2.7 m/s of noise) gets multiplied
# by SYSTEM_LAG and added to the commanded target, telling ArduPilot the pad is
# 10-70cm away from where it actually is - in a random direction. The drone then
# chases the phantom and walks off the pad. Only enable for a genuinely MOVING
# platform, and only once the velocity estimate is trustworthy (quality high,
# speed sustained well above the noise floor).
ENABLE_PREDICTION        = True
SYSTEM_LAG               = 0.30   # seconds - extended for demo so prediction "leads"
                                  # visibly on the overlay; revert to ~0.05-0.1 for
                                  # actual moving-platform landing tuning.
MIN_SPEED_FOR_PREDICTION = 0.10   # m/s - lowered for demo so prediction engages
                                  # at slow walking pace (10cm/s).

# --- Outlier rejection (drops noisy frames before sending to FC) ---
# When the drone vibrates in flight, AprilTag corner detection becomes
# noisy and individual frames can report positions 30-100cm off from
# reality. If we forward those to ArduPilot, it tilts the drone hard to
# chase the bad measurement, which makes vibration worse, which produces
# more bad measurements - a positive feedback loop that swings the drone
# off the pad. These limits drop frames whose position is implausibly
# far from the previous good frame.
ENABLE_JUMP_FILTER      = True
MAX_FRAME_JUMP_M        = 0.30   # max XY distance change between frames (meters)
MAX_FRAME_ALT_JUMP_M    = 0.40   # max altitude change between frames (meters)
JUMP_FILTER_TIMEOUT_S   = 0.5    # if last good frame is older than this, accept anyway
                                 # (avoids being stuck rejecting after a real lost-target)

# --- Video recording (overlay burned in) ---
# Software MJPG encoder via cv2.VideoWriter. Each annotated detection frame
# (with tag boxes, position, altitude) is written to the file. Result:
# the recording shows EXACTLY what the system saw and decided.
ENABLE_RECORDING        = True
RECORD_FPS              = 15      # output video FPS - keep below avg detection FPS
RECORD_FOURCC           = 'mp4v'  # 'mp4v' for MP4, 'MJPG' for AVI (more compat but bigger)
RECORD_EXT              = '.mp4'  # match the fourcc
RECORD_ALWAYS           = False   # True = record always; False = only while target seen
RECORD_POST_TIMEOUT_S   = 5.0     # if RECORD_ALWAYS=False, stop video N sec after last detect

# --- Logging ---
LOG_DIR        = os.path.expanduser('~/precision_landing')
LOG_FILE       = os.path.join(LOG_DIR, 'drone.log')
SNAPSHOT_DIR   = os.path.join(LOG_DIR, 'snapshots')
VIDEO_DIR      = os.path.join(LOG_DIR, 'videos')
CSV_DIR        = os.path.join(LOG_DIR, 'csv')   # flight data for plotting
SNAPSHOT_EVERY = 50   # frames

# Save a CSV row per detected frame for post-flight plotting.
# Lightweight (~100 bytes per row), creates one file per script run.
ENABLE_CSV_LOG = True

# --- Camera calibration (chessboard, 640x480, full-FOV binned mode) ---
# Pi Camera V2 / IMX219, calibrated with cv2.calibrateCamera at RMS = 0.255 px.
# Valid ONLY for: 640x480 capture stream + camera config with
# raw={"size":(1640,1232)} (locks the binned full-FOV sensor mode).
# If you change the resolution or sensor mode, recalibrate.
CAMERA_MATRIX = np.array([
    [497.8761,   0.0,    323.7928],
    [  0.0,    496.6770, 239.3209],
    [  0.0,      0.0,      1.0   ]
], dtype=np.float32)

DIST_COEFFS = np.array([0.13967871, -0.26623053, 0.0, 0.0, 0.0], dtype=np.float32)


# ============================================================
#                  VELOCITY ESTIMATOR CLASS
# ============================================================

class VelocityEstimator:
    """
    Weighted least-squares quadratic fit with outlier rejection.
    Returns position, velocity, and acceleration estimates.
    """
    
    def __init__(self, history_size=20, min_points=6,
                 weight_tau=0.3, outlier_sigma=3.0, max_speed=5.0):
        self.history = deque(maxlen=history_size)
        self.min_points = min_points
        self.weight_tau = weight_tau
        self.outlier_sigma = outlier_sigma
        self.max_speed = max_speed
        
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.speed = 0.0
        self.quality = 0.0
    
    def reset(self):
        self.history.clear()
        self.vel_x = self.vel_y = 0.0
        self.acc_x = self.acc_y = 0.0
        self.speed = 0.0
        self.quality = 0.0
    
    def add_measurement(self, x, y, timestamp):
        self.history.append((x, y, timestamp))
        self.pos_x = x
        self.pos_y = y
        self._update_estimate()
    
    def _update_estimate(self):
        if len(self.history) < self.min_points:
            self.vel_x = self.vel_y = 0.0
            self.acc_x = self.acc_y = 0.0
            self.speed = 0.0
            self.quality = 0.0
            return
        
        data = np.array(self.history)
        t_now = data[-1, 2]
        t = data[:, 2] - t_now  # all <= 0
        x_raw = data[:, 0]
        y_raw = data[:, 1]
        
        weights = np.exp(t / self.weight_tau)
        
        try:
            vx, ax, qx = self._weighted_fit(t, x_raw, weights)
            vy, ay, qy = self._weighted_fit(t, y_raw, weights)
            
            vx, ax, qx = self._refit_with_outlier_rejection(t, x_raw, weights, vx, ax, qx)
            vy, ay, qy = self._refit_with_outlier_rejection(t, y_raw, weights, vy, ay, qy)
            
            speed = np.sqrt(vx**2 + vy**2)
            if speed > self.max_speed:
                self.quality = 0.0
                return
            
            self.vel_x = float(vx)
            self.vel_y = float(vy)
            self.acc_x = float(2.0 * ax)
            self.acc_y = float(2.0 * ay)
            self.speed = float(speed)
            self.quality = float(min(qx, qy))
            
        except (np.linalg.LinAlgError, ValueError):
            self.quality = 0.0
    
    def _weighted_fit(self, t, values, weights):
        A = np.column_stack([t**2, t, np.ones_like(t)])
        W = np.diag(weights)
        AtWA = A.T @ W @ A
        AtWy = A.T @ W @ values
        coeffs = np.linalg.solve(AtWA, AtWy)
        a, b, c = coeffs
        
        predicted = A @ coeffs
        residuals = values - predicted
        wss_res = np.sum(weights * residuals**2)
        mean_val = np.sum(weights * values) / np.sum(weights)
        wss_tot = np.sum(weights * (values - mean_val)**2)
        
        if wss_tot > 1e-9:
            r2 = 1.0 - wss_res / wss_tot
            r2 = max(0.0, min(1.0, r2))
        else:
            r2 = 0.0
        
        return b, a, r2
    
    def _refit_with_outlier_rejection(self, t, values, weights,
                                       vel_initial, half_acc_initial, qual_initial):
        baseline = values - half_acc_initial * t**2 - vel_initial * t
        offset = np.sum(weights * baseline) / np.sum(weights)
        predicted = half_acc_initial * t**2 + vel_initial * t + offset
        residuals = values - predicted
        
        wm_res = np.sum(weights * residuals) / np.sum(weights)
        wvar = np.sum(weights * (residuals - wm_res)**2) / np.sum(weights)
        sigma = np.sqrt(wvar)
        
        if sigma < 0.01:
            return vel_initial, half_acc_initial, qual_initial
        
        inlier_mask = np.abs(residuals - wm_res) < self.outlier_sigma * sigma
        n_inliers = np.sum(inlier_mask)
        
        if n_inliers < self.min_points or n_inliers == len(values):
            return vel_initial, half_acc_initial, qual_initial
        
        try:
            return self._weighted_fit(
                t[inlier_mask], values[inlier_mask], weights[inlier_mask]
            )
        except (np.linalg.LinAlgError, ValueError):
            return vel_initial, half_acc_initial, qual_initial
    
    def predict_position(self, lag_seconds):
        px = self.pos_x + self.vel_x * lag_seconds + 0.5 * self.acc_x * lag_seconds**2
        py = self.pos_y + self.vel_y * lag_seconds + 0.5 * self.acc_y * lag_seconds**2
        return px, py


# ============================================================
#                MAIN PRECISION LANDING CLASS
# ============================================================

class DronePrecisionLanding:
    
    def __init__(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        os.makedirs(SNAPSHOT_DIR, exist_ok=True)
        os.makedirs(VIDEO_DIR, exist_ok=True)
        os.makedirs(CSV_DIR, exist_ok=True)
        
        self._setup_logging()
        self.logger.info("=" * 70)
        self.logger.info("DRONE PRECISION LANDING - PRODUCTION VERSION")
        self.logger.info("=" * 70)
        self.logger.info(f"AprilTag library: {APRILTAG_LIB}")
        self.logger.info(f"Tag family: {TAG_FAMILY}")
        self.logger.info(f"Markers configured: {sorted(MARKER_SIZES.keys())}")
        for tid in sorted(MARKER_SIZES.keys()):
            sz = MARKER_SIZES[tid]
            ox, oy = MARKER_OFFSETS[tid]
            self.logger.info(f"  Tag {tid}: size={sz*100:.1f}cm offset=({ox*100:+.0f},{oy*100:+.0f})cm")
        self.logger.info(f"Camera: {CAMERA_WIDTH}x{CAMERA_HEIGHT} @ {CAMERA_FPS}fps")
        self.logger.info(f"Camera mount: ribbon-toward-tail (body_x=-cam_y, body_y=+cam_x)")
        self.logger.info(f"Cam offset from CG: X={CAM_OFFSET_X} Y={CAM_OFFSET_Y} Z={CAM_OFFSET_Z}")
        self.logger.info(f"Velocity estimator: weighted LSQ, tau={VELOCITY_WEIGHT_TAU}s")
        self.logger.info(f"Prediction: {'ON' if ENABLE_PREDICTION else 'OFF'} lag={SYSTEM_LAG}s")
        self.logger.info(f"Jump filter: {'ON' if ENABLE_JUMP_FILTER else 'OFF'} "
                         f"max_xy={MAX_FRAME_JUMP_M*100:.0f}cm "
                         f"max_z={MAX_FRAME_ALT_JUMP_M*100:.0f}cm "
                         f"timeout={JUMP_FILTER_TIMEOUT_S}s")
        self.logger.info(f"Recording: {'ON' if ENABLE_RECORDING else 'OFF'} "
                         f"{RECORD_FPS}fps {RECORD_FOURCC} "
                         f"mode={'always' if RECORD_ALWAYS else 'on-detect'}")
        self.logger.info("=" * 70)
        
        self._setup_apriltag()
        
        # State
        self.lock = Lock()
        self.master = None
        self.picam2 = None
        self.running = True
        self.boot_time = time.time()
        
        # Detection state
        self.target_detected = False
        self.last_detection_time = 0.0
        self.body_x = 0.0
        self.body_y = 0.0
        self.dist = 0.0
        self.primary_tag_id = -1
        self.tags_visible = []
        
        # Jump filter state - last GOOD measurement (one accepted by filter)
        self.last_good_x = 0.0
        self.last_good_y = 0.0
        self.last_good_z = 0.0
        self.last_good_time = 0.0
        self.jump_reject_count = 0    # how many frames we dropped due to jumps
        
        # Tilt
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.tilt_safe = True
        self.velocity_safe = True
        
        # Velocity estimator (body frame)
        self.vel_estimator = VelocityEstimator(
            history_size=VELOCITY_HISTORY_SIZE,
            min_points=VELOCITY_MIN_POINTS,
            weight_tau=VELOCITY_WEIGHT_TAU,
            outlier_sigma=VELOCITY_OUTLIER_SIGMA,
            max_speed=MAX_REASONABLE_SPEED
        )
        
        # Statistics
        self.frame_count = 0
        self.detection_count = 0
        self.mavlink_sent_count = 0
        self.snap_count = 0
        self.tilt_warn_count = 0
        self.vel_warn_count = 0
        self.start_time = time.time()
        
        # Video recording state
        self.video_writer = None      # cv2.VideoWriter when active, None when not
        self.video_filename = None
        self.video_frame_count = 0    # frames written to current video
        self.video_count = 0          # number of videos created this session
        self.last_record_activity = 0.0  # time of last detection (for on-detect mode)
        
        # CSV flight data - opened in run() so file gets a per-session timestamp
        self.csv_file = None
        self.csv_filename = None
        self.csv_row_count = 0
        
        # FPS
        self.fps = 0.0
        self.fps_start_time = time.time()
    
    # --------------------------------------------------------
    # SETUP
    # --------------------------------------------------------
    
    def _setup_logging(self):
        fmt = '%(asctime)s [%(levelname)s] %(message)s'
        logging.basicConfig(
            level=logging.INFO, format=fmt,
            handlers=[
                logging.FileHandler(LOG_FILE),
                logging.StreamHandler(sys.stdout)
            ],
            force=True
        )
        self.logger = logging.getLogger(__name__)
    
    def _setup_apriltag(self):
        """Initialize AprilTag detector based on which library is available"""
        if APRILTAG_LIB == 'pupil_apriltags':
            self.detector = AprilTagDetector(
                families=TAG_FAMILY,
                nthreads=2,           # use 2 of Pi 4's 4 cores
                quad_decimate=1.0,    # 1.0 = full resolution (most accurate)
                quad_sigma=0.0,       # no input image blur
                refine_edges=1,       # edge refinement (improves pose accuracy)
                decode_sharpening=0.25,
                debug=0
            )
            self.logger.info("Using pupil_apriltags detector")
        else:  # apriltag library
            options = apriltag.DetectorOptions(
                families=TAG_FAMILY,
                border=1,
                nthreads=2,
                quad_decimate=1.0,
                quad_sigma=0.0,
                refine_edges=True,
                refine_decode=False,
                refine_pose=True
            )
            self.detector = apriltag.Detector(options)
            self.logger.info("Using apriltag detector")
    
    def connect_fc(self):
        self.logger.info(f"Connecting FC {FC_CONNECTION} @{BAUD_RATE}...")
        try:
            self.master = mavutil.mavlink_connection(
                FC_CONNECTION,
                baud=BAUD_RATE,
                source_system=1,
                source_component=mavutil.mavlink.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
            )
            self.logger.info("Waiting for heartbeat (timeout 15s)...")
            hb = self.master.wait_heartbeat(timeout=15)
            if not hb:
                self.logger.error("No heartbeat - check wiring/baud rate")
                return False
            self.logger.info("FC connected!")
            self.logger.info(f"  System:{self.master.target_system} "
                             f"Component:{self.master.target_component} "
                             f"Autopilot:{hb.autopilot} Type:{hb.type}")
            return True
        except Exception as e:
            self.logger.error(f"FC connection failed: {e}")
            return False
    
    def init_camera(self):
        """
        Initialize Pi Camera with a single RGB888 preview stream at 640x480
        in the full-FOV binned sensor mode (raw 1640x1232).
        """
        self.logger.info("Initialising camera...")
        try:
            self.picam2 = Picamera2()
            
            cfg = self.picam2.create_preview_configuration(
                main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "RGB888"},
                raw={"size": (1640, 1232)},                # full-FOV binned sensor mode
                controls={"FrameRate": CAMERA_FPS},
                buffer_count=4
            )
            
            self.picam2.configure(cfg)
            
            # Build undistortion lookup maps once. cv2.remap with these on a 640x480
            # grayscale frame is ~0.5 ms on a Pi 4 - negligible cost per frame.
            self.undist_mapx, self.undist_mapy = cv2.initUndistortRectifyMap(
                CAMERA_MATRIX, DIST_COEFFS, None, CAMERA_MATRIX,
                (CAMERA_WIDTH, CAMERA_HEIGHT), cv2.CV_16SC2
            )
            
            # FOV sanity check - log this so we have positive confirmation the sensor
            # mode locked correctly. Should read ~65 x 51 deg (full-FOV). If it shows
            # ~42 x 33 deg, the raw={"size":(1640,1232)} hint did not take effect.
            fx, fy = CAMERA_MATRIX[0, 0], CAMERA_MATRIX[1, 1]
            fov_x = 2 * np.degrees(np.arctan(CAMERA_WIDTH  / (2 * fx)))
            fov_y = 2 * np.degrees(np.arctan(CAMERA_HEIGHT / (2 * fy)))
            self.logger.info(f"Camera FOV: {fov_x:.1f}deg x {fov_y:.1f}deg (target ~65 x 51)")
            
            self.picam2.start()
            time.sleep(2)
            
            # Verify
            test = self.picam2.capture_array()
            if test is None or test.size == 0:
                raise RuntimeError("Empty test frame")
            if len(test.shape) != 3 or test.shape[2] != 3:
                self.logger.error(
                    f"Detection stream returned shape {test.shape} - expected (H, W, 3)."
                )
                return False
            self.logger.info(f"Camera ready: detection stream {test.shape}")
            
            return True
        except Exception as e:
            self.logger.error(f"Camera init failed: {e}\n{traceback.format_exc()}")
            return False
    
    # --------------------------------------------------------
    # COORDINATE FRAME MAPPING
    # --------------------------------------------------------
    
    def _camera_to_body_frame(self, cam_x, cam_y, cam_z):
        """
        Convert OpenCV camera frame to drone body NED frame.
        
        Mounting: camera on belly, lens pointing down, ribbon toward TAIL.
        
        OpenCV camera frame:
          +cam_x = right in image
          +cam_y = down in image
          +cam_z = forward (out of lens) = altitude above marker
        
        With ribbon toward tail:
          Top of image points toward drone NOSE
          So marker forward of drone -> appears at top -> cam_y negative
          Marker right of drone -> appears right in image -> cam_x positive
        
        Body NED (FRD):
          body_x = forward = -cam_y
          body_y = right   = +cam_x
          body_z = down    = -cam_z (handled separately as 'dist')
        
        Then apply camera offset from CG.
        """
        body_x = -cam_y - CAM_OFFSET_X
        body_y =  cam_x - CAM_OFFSET_Y
        dist   =  cam_z
        return body_x, body_y, dist
    
    # --------------------------------------------------------
    # APRILTAG DETECTION
    # --------------------------------------------------------
    
    def _detect_tags(self, gray):
        """
        Detect all configured AprilTags in the image.
        Returns list of detection dicts with all relevant info.
        """
        # Camera intrinsics for AprilTag pose estimation
        cam_params = (CAMERA_MATRIX[0, 0], CAMERA_MATRIX[1, 1],
                      CAMERA_MATRIX[0, 2], CAMERA_MATRIX[1, 2])
        
        detections = []
        
        if APRILTAG_LIB == 'pupil_apriltags':
            # pupil_apriltags can do pose estimation if we pass camera params,
            # but we want to use our own per-tag size. So we detect first, then
            # estimate pose per-tag with correct size.
            results = self.detector.detect(
                gray,
                estimate_tag_pose=False,  # we'll do pose per-tag with correct size
                camera_params=None,
                tag_size=None
            )
            for det in results:
                tag_id = det.tag_id
                if tag_id not in MARKER_SIZES:
                    continue
                if det.decision_margin < MIN_DECISION_MARGIN:
                    continue
                
                # Estimate pose with the correct tag size
                tag_size = MARKER_SIZES[tag_id]
                # det.corners is already (4, 2) array of corner pixel coords
                corners = det.corners.astype(np.float32)
                
                # Build object points for solvePnP (tag corners in tag frame)
                # Order matches AprilTag corner ordering
                half = tag_size / 2.0
                obj_pts = np.array([
                    [-half, -half, 0.0],   # bottom-left
                    [+half, -half, 0.0],   # bottom-right
                    [+half, +half, 0.0],   # top-right
                    [-half, +half, 0.0],   # top-left
                ], dtype=np.float32)
                
                ok, rvec, tvec = cv2.solvePnP(
                    obj_pts, corners,
                    CAMERA_MATRIX, None,                # frame already undistorted via cv2.remap
                    flags=cv2.SOLVEPNP_ITERATIVE
                )
                
                if not ok:
                    continue
                
                # Reprojection sanity check - reject poses that don't fit the
                # observed corners (catches solver failures and ambiguous flips
                # before they ever reach the flight controller).
                proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, CAMERA_MATRIX, None)
                reproj_err = float(np.linalg.norm(
                    proj.reshape(-1, 2) - corners, axis=1).mean())
                if reproj_err > MAX_REPROJ_ERR_PX:
                    continue
                
                # Build detection dict
                detections.append({
                    'id': int(tag_id),
                    'tvec': tvec.flatten(),  # (3,) - [x, y, z] in camera frame
                    'rvec': rvec,
                    'corners': corners,
                    'pixel_area': cv2.contourArea(corners),
                    'decision_margin': float(det.decision_margin),
                    'center': det.center,  # (2,) pixel center
                })
        
        else:  # apriltag library
            results = self.detector.detect(gray)
            for det in results:
                tag_id = det.tag_id
                if tag_id not in MARKER_SIZES:
                    continue
                if det.decision_margin < MIN_DECISION_MARGIN:
                    continue
                
                tag_size = MARKER_SIZES[tag_id]
                corners = det.corners.astype(np.float32)
                
                half = tag_size / 2.0
                obj_pts = np.array([
                    [-half, -half, 0.0],
                    [+half, -half, 0.0],
                    [+half, +half, 0.0],
                    [-half, +half, 0.0],
                ], dtype=np.float32)
                
                ok, rvec, tvec = cv2.solvePnP(
                    obj_pts, corners,
                    CAMERA_MATRIX, None,                # frame already undistorted via cv2.remap
                    flags=cv2.SOLVEPNP_ITERATIVE
                )
                
                if not ok:
                    continue
                
                # Reprojection sanity check - reject poses that don't fit the
                # observed corners (catches solver failures and ambiguous flips
                # before they ever reach the flight controller).
                proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, CAMERA_MATRIX, None)
                reproj_err = float(np.linalg.norm(
                    proj.reshape(-1, 2) - corners, axis=1).mean())
                if reproj_err > MAX_REPROJ_ERR_PX:
                    continue
                
                detections.append({
                    'id': int(tag_id),
                    'tvec': tvec.flatten(),
                    'rvec': rvec,
                    'corners': corners,
                    'pixel_area': cv2.contourArea(corners),
                    'decision_margin': float(det.decision_margin),
                    'center': np.array(det.center, dtype=np.float32),
                })
        
        return detections
    
    def _select_best_detection(self, detections, image_shape):
        """
        From all visible tag detections, compute the LANDING CENTER position
        as a weighted average across valid tags.
        
        Returns dict with: body_x, body_y, dist, primary_tag_id, all_tags
        Or None if no usable detection.
        """
        if not detections:
            return None
        
        h, w = image_shape[:2]
        margin = EDGE_MARGIN_PX
        
        # Filter and weight detections
        valid = []
        for d in detections:
            cam_x, cam_y, cam_z = d['tvec']
            
            # Range check
            if cam_z < MIN_DETECT_DIST or cam_z > MAX_DETECT_DIST:
                continue
            
            # Edge check - reject if any corner too close to image edge
            corners = d['corners']
            x_min, y_min = corners[:, 0].min(), corners[:, 1].min()
            x_max, y_max = corners[:, 0].max(), corners[:, 1].max()
            
            edge_distance = min(x_min, y_min, w - x_max, h - y_max)
            edge_safe = edge_distance > margin
            
            # Compute landing center position from this tag.
            #
            # Each tag is glued flat on the board. MARKER_OFFSETS gives the
            # vector from the LANDING CENTER to the TAG CENTER, expressed in
            # the BOARD PLANE:
            #     +X_board = right when looking down at board
            #     +Y_board = forward (away from drone tail) when looking down
            #
            # To find the landing center in CAMERA frame, we need to:
            #   1. Express the offset vector in the TAG's local frame (which
            #      is the same as the board plane, just centered on the tag).
            #   2. Rotate it into the camera frame using the tag's rvec.
            #   3. Subtract from the tag's camera position (since offset goes
            #      FROM landing center TO tag, we subtract to go the other way).
            #
            # pupil_apriltags / OpenCV tag local frame convention:
            #     +X_tag = right edge of tag (matches +X_board)
            #     +Y_tag = top edge of tag (matches +Y_board)
            #     +Z_tag = out of tag face (toward camera when flat)
            # Empirically verified with pose_diagnostic.py:
            #     Y FLIPPED  -> 42 cm disagreement between tags
            #     Y NOT FLIPPED -> 0.3 cm disagreement (correct)
            
            offset_x_board, offset_y_board = MARKER_OFFSETS[d['id']]
            offset_in_tag_frame = np.array(
                [offset_x_board, offset_y_board, 0.0], dtype=np.float64
            )
            
            # Rotate offset into camera frame using this tag's pose
            R_tag, _ = cv2.Rodrigues(d['rvec'])
            offset_in_cam = R_tag @ offset_in_tag_frame
            
            # Landing center = tag position - offset (offset points center->tag)
            landing_cam_x = cam_x - offset_in_cam[0]
            landing_cam_y = cam_y - offset_in_cam[1]
            landing_cam_z = cam_z - offset_in_cam[2]
            
            # Convert landing center to body frame
            body_x, body_y, dist = self._camera_to_body_frame(
                landing_cam_x, landing_cam_y, landing_cam_z
            )
            
            # Quality weight based on:
            # - pixel area (larger = more accurate pose)
            # - decision margin (higher = more confident detection)
            # - edge safety (corners not at edge)
            area_weight = np.sqrt(d['pixel_area'])  # sqrt to avoid huge tags dominating
            margin_weight = d['decision_margin'] / 100.0
            edge_weight = 1.0 if edge_safe else 0.3
            
            weight = area_weight * margin_weight * edge_weight
            
            valid.append({
                'id': d['id'],
                'body_x': body_x,
                'body_y': body_y,
                'dist': dist,
                'weight': weight,
                'edge_safe': edge_safe,
                'pixel_area': d['pixel_area'],
                'decision_margin': d['decision_margin'],
                'rvec': d['rvec'],
                'corners': d['corners'],
            })
        
        if not valid:
            return None
        
        # Pick primary tag = highest weighted, fully in frame
        primary = max(valid, key=lambda v: v['weight'])
        
        # Weighted average across all valid tags
        total_weight = sum(v['weight'] for v in valid)
        if total_weight > 0:
            avg_body_x = sum(v['body_x'] * v['weight'] for v in valid) / total_weight
            avg_body_y = sum(v['body_y'] * v['weight'] for v in valid) / total_weight
            avg_dist   = sum(v['dist']   * v['weight'] for v in valid) / total_weight
        else:
            avg_body_x = primary['body_x']
            avg_body_y = primary['body_y']
            avg_dist   = primary['dist']
        
        # Per-tag agreement diagnostic: when 2+ tags visible, compute the
        # max distance between any tag's landing-center estimate and the
        # weighted mean. Large disagreement = MARKER_OFFSETS or layout wrong.
        disagreement = 0.0
        if len(valid) > 1:
            for v in valid:
                dx = v['body_x'] - avg_body_x
                dy = v['body_y'] - avg_body_y
                d_err = float(np.sqrt(dx*dx + dy*dy))
                if d_err > disagreement:
                    disagreement = d_err
        
        return {
            'body_x': avg_body_x,
            'body_y': avg_body_y,
            'dist': avg_dist,
            'primary_tag_id': primary['id'],
            'primary_rvec': primary['rvec'],
            'all_tags': valid,
            'n_tags': len(valid),
            'disagreement': disagreement,
        }
    
    # --------------------------------------------------------
    # TILT ESTIMATION (rotation-matrix based, yaw-invariant)
    # --------------------------------------------------------
    
    def _estimate_tilt(self, rvec):
        """
        Extract marker tilt using rotation matrix Z-axis directly.
        Yaw-independent, robust to marker rotation in plane.
        """
        try:
            rot_mat, _ = cv2.Rodrigues(rvec)
            
            # Marker's Z-axis (up from marker) expressed in camera frame
            mz = rot_mat[:, 2].copy()
            
            # For flat marker seen from above: mz = [0, 0, -1]
            # (marker up = camera -Z because camera looks down)
            #
            # solvePnP can return ambiguous solutions where the entire mz
            # vector flips sign (mz -> -mz) - same projection, opposite
            # rotation. We disambiguate by FORCING mz to point toward the
            # camera (negative Z component), which is the physically valid
            # orientation for a tag lying flat with the camera above it.
            # This must flip the WHOLE vector, not just abs() the denominator,
            # otherwise the X/Y components disagree with Z and tilt sign flips.
            if mz[2] > 0:
                mz = -mz
            
            denom = -mz[2]   # now guaranteed positive
            if denom < 0.1:
                # Marker nearly edge-on - tilt is genuinely unmeasurable from
                # this view. Return safe=True (unknown != unsafe) so we don't
                # spam warnings; downstream code can treat this as "no info".
                return 0.0, 0.0, 0.0, True
            
            tilt_cam_x = np.degrees(np.arctan2(mz[0], denom))
            tilt_cam_y = np.degrees(np.arctan2(mz[1], denom))
            
            # Convert to body frame (same mapping as position)
            roll_body  =  tilt_cam_x
            pitch_body = -tilt_cam_y
            
            # Yaw: rotation of marker's X-axis in image plane.
            # If we flipped mz above, we should also use the flipped X-axis
            # to keep the rotation matrix self-consistent.
            mx = rot_mat[:, 0].copy()
            if rot_mat[2, 2] > 0:
                mx = -mx
            yaw = np.degrees(np.arctan2(mx[1], mx[0]))
            
            safe = (abs(roll_body) <= MAX_SAFE_ROLL and 
                    abs(pitch_body) <= MAX_SAFE_PITCH)
            
            return float(roll_body), float(pitch_body), float(yaw), safe
            
        except Exception as e:
            self.logger.error(f"Tilt estimation error: {e}")
            # Unknown tilt is not the same as unsafe tilt
            return 0.0, 0.0, 0.0, True
    
    # --------------------------------------------------------
    # MAVLINK LANDING_TARGET
    # --------------------------------------------------------
    
    def _send_landing_target(self, body_x, body_y, dist):
        """Send LANDING_TARGET to ArduPilot with prediction"""
        try:
            # Apply prediction if platform is moving
            if (ENABLE_PREDICTION 
                and self.vel_estimator.speed >= MIN_SPEED_FOR_PREDICTION
                and self.vel_estimator.quality > 0.3):
                pred_x, pred_y = self.vel_estimator.predict_position(SYSTEM_LAG)
                body_x_send = pred_x
                body_y_send = pred_y
            else:
                body_x_send = body_x
                body_y_send = body_y
            
            # Angular offsets (radians)
            if dist > 0.1:
                angle_x = float(np.arctan2(body_x_send, dist))   # forward angle
                angle_y = float(np.arctan2(body_y_send, dist))   # right angle
            else:
                angle_x = 0.0
                angle_y = 0.0
            
            # Angular size (use largest visible tag for estimation)
            primary_size = MARKER_SIZES.get(self.primary_tag_id, 0.24)
            size = float(2.0 * np.arctan2(primary_size / 2.0, dist))
            
            # Boot-relative microseconds
            time_usec = int((time.time() - self.boot_time) * 1e6)
            
            self.master.mav.landing_target_send(
                time_usec,
                0,                                       # target_num
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                angle_x,                                 # forward angular offset
                angle_y,                                 # right angular offset
                float(dist),                             # distance (m)
                size,                                    # size_x (rad)
                size                                     # size_y (rad)
            )
            self.mavlink_sent_count += 1
            
        except Exception as e:
            self.logger.error(f"MAVLink send error: {e}")
    
    # --------------------------------------------------------
    # VIDEO RECORDING (overlay burned in via cv2.VideoWriter)
    # --------------------------------------------------------
    
    def _open_video_writer(self):
        """Open a new video file for writing. Returns True on success."""
        if self.video_writer is not None:
            return True   # already open
        try:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            fname = os.path.join(VIDEO_DIR, f'landing_{ts}{RECORD_EXT}')
            fourcc = cv2.VideoWriter_fourcc(*RECORD_FOURCC)
            writer = cv2.VideoWriter(
                fname, fourcc, RECORD_FPS,
                (CAMERA_WIDTH, CAMERA_HEIGHT)
            )
            if not writer.isOpened():
                self.logger.error(f"Could not open video file: {fname}")
                return False
            self.video_writer = writer
            self.video_filename = fname
            self.video_frame_count = 0
            self.video_count += 1
            self.logger.info(f"Recording STARTED: {fname}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to open video writer: {e}")
            return False
    
    def _close_video_writer(self):
        """Finalize and close the current video file."""
        if self.video_writer is None:
            return
        try:
            self.video_writer.release()
            self.logger.info(
                f"Recording STOPPED: {self.video_filename} "
                f"({self.video_frame_count} frames @ {RECORD_FPS}fps "
                f"= {self.video_frame_count/RECORD_FPS:.1f}s)"
            )
        except Exception as e:
            self.logger.error(f"Error closing video: {e}")
        finally:
            self.video_writer = None
            self.video_filename = None
            self.video_frame_count = 0
    
    def _write_video_frame(self, annotated_bgr):
        """Write a frame (already BGR, already annotated) to the active video."""
        if self.video_writer is None:
            return
        try:
            # Make sure dimensions match what writer was opened with
            h, w = annotated_bgr.shape[:2]
            if w != CAMERA_WIDTH or h != CAMERA_HEIGHT:
                annotated_bgr = cv2.resize(annotated_bgr, (CAMERA_WIDTH, CAMERA_HEIGHT))
            self.video_writer.write(annotated_bgr)
            self.video_frame_count += 1
        except Exception as e:
            self.logger.error(f"Video write error: {e}")
    
    def _update_video_state(self, target_seen, now):
        """
        Manage recording lifecycle.
        - RECORD_ALWAYS=True: open on first frame, stay open until shutdown.
        - RECORD_ALWAYS=False: open when target seen, close N seconds after lost.
        """
        if not ENABLE_RECORDING:
            return
        
        if RECORD_ALWAYS:
            if self.video_writer is None:
                self._open_video_writer()
            return
        
        # On-detect mode
        if target_seen:
            self.last_record_activity = now
            if self.video_writer is None:
                self._open_video_writer()
        else:
            if self.video_writer is not None:
                idle = now - self.last_record_activity
                if idle > RECORD_POST_TIMEOUT_S:
                    self._close_video_writer()
    
    # --------------------------------------------------------
    # CSV FLIGHT DATA (for post-flight plotting)
    # --------------------------------------------------------
    
    def _open_csv(self):
        """Open the CSV flight-data file with a timestamped name."""
        if not ENABLE_CSV_LOG or self.csv_file is not None:
            return
        try:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.csv_filename = os.path.join(CSV_DIR, f'flight_{ts}.csv')
            self.csv_file = open(self.csv_filename, 'w', buffering=1)  # line-buffered
            # Header row - keep these names stable; the plotter relies on them
            self.csv_file.write(
                't,frame,detected,n_tags,primary_tag,'
                'body_x,body_y,alt,'
                'vel_x,vel_y,speed,vel_quality,'
                'acc_x,acc_y,'
                'pred_x,pred_y,'
                'roll,pitch,yaw,'
                'mav_sent,jump_rejected\n'
            )
            self.logger.info(f"CSV opened: {self.csv_filename}")
        except Exception as e:
            self.logger.error(f"CSV open failed: {e}")
            self.csv_file = None
    
    def _close_csv(self):
        """Flush and close the CSV file."""
        if self.csv_file is None:
            return
        try:
            self.csv_file.flush()
            self.csv_file.close()
            self.logger.info(
                f"CSV closed: {self.csv_filename} ({self.csv_row_count} rows)"
            )
        except Exception as e:
            self.logger.error(f"CSV close error: {e}")
        finally:
            self.csv_file = None
    
    def _write_csv_row(self, now, result, jump_rejected_this_frame):
        """
        Write one row to the CSV. Called every frame so we get a complete
        timeline including frames where the target wasn't detected.
        """
        if self.csv_file is None:
            return
        try:
            vel = self.vel_estimator
            if result is not None:
                detected = 1
                n_tags = result['n_tags']
                primary_tag = result['primary_tag_id']
                bx = result['body_x']
                by = result['body_y']
                alt = result['dist']
            else:
                detected = 0
                n_tags = 0
                primary_tag = -1
                bx = 0.0
                by = 0.0
                alt = 0.0
            
            # Prediction (compute regardless of threshold so we have data
            # in the CSV; the plotter can decide whether to show it)
            pred_x = vel.pos_x + vel.vel_x * SYSTEM_LAG + 0.5 * vel.acc_x * SYSTEM_LAG**2
            pred_y = vel.pos_y + vel.vel_y * SYSTEM_LAG + 0.5 * vel.acc_y * SYSTEM_LAG**2
            
            self.csv_file.write(
                f"{now:.4f},{self.frame_count},{detected},{n_tags},{primary_tag},"
                f"{bx:.4f},{by:.4f},{alt:.4f},"
                f"{vel.vel_x:.4f},{vel.vel_y:.4f},{vel.speed:.4f},{vel.quality:.3f},"
                f"{vel.acc_x:.4f},{vel.acc_y:.4f},"
                f"{pred_x:.4f},{pred_y:.4f},"
                f"{self.roll:.2f},{self.pitch:.2f},{self.yaw:.2f},"
                f"{self.mavlink_sent_count},{int(jump_rejected_this_frame)}\n"
            )
            self.csv_row_count += 1
        except Exception as e:
            self.logger.error(f"CSV write error: {e}")
    
    # --------------------------------------------------------
    # SNAPSHOTS (annotated still frames for debugging)
    # --------------------------------------------------------
    
    def _save_snapshot(self, frame, detection_result):
        try:
            if len(frame.shape) == 2:
                # Grayscale (Y plane from lores YUV) - replicate to 3 channels for color overlay
                snap = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                snap = cv2.cvtColor(frame.copy(), cv2.COLOR_RGB2BGR)
            self._draw_overlay(snap, detection_result, full_overlay=True)
            
            fname = os.path.join(SNAPSHOT_DIR, f"snap_{self.snap_count:05d}.jpg")
            cv2.imwrite(fname, snap)
            self.snap_count += 1
        except Exception as e:
            self.logger.error(f"Snapshot error: {e}")
    
    def _draw_overlay(self, frame_bgr, detection_result, full_overlay=False):
        """Draw status overlay on a BGR frame"""
        h, w = frame_bgr.shape[:2]
        
        # Top status bar
        bar_h = 200 if full_overlay else 90
        overlay = frame_bgr.copy()
        cv2.rectangle(overlay, (0, 0), (w, bar_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame_bgr, 0.4, 0, frame_bgr)
        
        def put(text, yp, color=(255, 255, 255), scale=0.5):
            cv2.putText(frame_bgr, text, (10, yp),
                        cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2)
        
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        yp = 22
        put(f"PRECISION LANDING  {timestamp}", yp, (255, 255, 255), 0.55)
        yp += 22
        
        # REC indicator (top-right) when recording is active
        if self.video_writer is not None:
            cv2.circle(frame_bgr, (w - 35, 18), 6, (0, 0, 255), -1)
            cv2.putText(frame_bgr, "REC", (w - 78, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        if detection_result:
            bx = detection_result['body_x']
            by = detection_result['body_y']
            dist = detection_result['dist']
            ntags = detection_result['n_tags']
            primary = detection_result['primary_tag_id']
            disagree = detection_result.get('disagreement', 0.0)
            
            dir_x = "FWD" if bx > 0 else "BACK"
            dir_y = "RIGHT" if by > 0 else "LEFT"
            
            # Tag header line; color-coded by agreement (only meaningful with 2+ tags)
            if ntags > 1:
                if disagree > 0.10:
                    header_color = (0, 0, 255)        # red - bad
                elif disagree > 0.04:
                    header_color = (0, 200, 255)      # yellow - marginal
                else:
                    header_color = (0, 255, 0)        # green - good
                header_txt = (f"TARGET LOCKED  Tag{primary}  ({ntags} tags visible) "
                              f"  disagree:{disagree*100:.1f}cm")
            else:
                header_color = (0, 255, 0)
                header_txt = f"TARGET LOCKED  Tag{primary}  ({ntags} tags visible)"
            
            put(header_txt, yp, header_color, 0.6)
            yp += 22
            put(f"X:{bx:+.2f}m({dir_x}) Y:{by:+.2f}m({dir_y}) Alt:{dist:.2f}m",
                yp, (0, 255, 0), 0.55)
            yp += 22
            
            if full_overlay:
                vel = self.vel_estimator
                vc = (0, 255, 0) if self.velocity_safe else (0, 0, 255)
                put(f"Vel:{vel.speed:.2f}m/s "
                    f"Vx:{vel.vel_x:+.2f} Vy:{vel.vel_y:+.2f} "
                    f"Q:{vel.quality:.2f}", yp, vc, 0.5)
                yp += 22
                
                tc = (0, 255, 0) if self.tilt_safe else (0, 0, 255)
                put(f"Roll:{self.roll:+.1f} Pitch:{self.pitch:+.1f} "
                    f"Yaw:{self.yaw:+.1f}", yp, tc, 0.5)
                yp += 22
                
                if ENABLE_PREDICTION and vel.speed > MIN_SPEED_FOR_PREDICTION:
                    px, py = vel.predict_position(SYSTEM_LAG)
                    put(f"Predicted+{SYSTEM_LAG}s: X:{px:+.2f} Y:{py:+.2f}",
                        yp, (0, 255, 255), 0.5)
                yp += 22
                
                put(f"FPS:{self.fps:.1f} Frame:{self.frame_count} "
                    f"MAV:{self.mavlink_sent_count}", yp, (200, 200, 200), 0.45)
            
            # Draw detected tags
            for tag in detection_result.get('all_tags', []):
                corners = tag['corners'].astype(int)
                color = (0, 255, 0) if tag['edge_safe'] else (0, 165, 255)
                if tag['id'] == primary:
                    color = (0, 255, 255)  # primary in cyan
                
                for i in range(4):
                    pt1 = tuple(corners[i])
                    pt2 = tuple(corners[(i + 1) % 4])
                    cv2.line(frame_bgr, pt1, pt2, color, 2)
                
                # Label
                center = corners.mean(axis=0).astype(int)
                cv2.putText(frame_bgr, f"T{tag['id']}", tuple(center),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        else:
            put("SEARCHING FOR TARGET...", yp, (0, 100, 255), 0.6)
    
    # --------------------------------------------------------
    # FRAME PROCESSING
    # --------------------------------------------------------
    
    def _process_frame(self, frame):
        try:
            self.frame_count += 1
            
            # FPS update
            if self.frame_count % 30 == 0:
                elapsed = time.time() - self.fps_start_time
                self.fps = 30.0 / elapsed if elapsed > 0 else 0.0
                self.fps_start_time = time.time()
            
            # Convert to grayscale for AprilTag detection
            if len(frame.shape) == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            else:
                gray = frame
            
            # Detect tags
            detections = self._detect_tags(gray)
            
            # Select best detection / fuse multiple
            result = self._select_best_detection(detections, gray.shape)
            
            frame_jump_rejected = False
            
            with self.lock:
                if result is not None:
                    now = time.time()
                    
                    # ---- JUMP FILTER ----
                    # Reject this frame if the position changed too much from
                    # the previous good frame. Vibration during flight produces
                    # large single-frame outliers; if we forward those to the
                    # FC it tilts hard to chase them, causing the swing-out
                    # behavior. After JUMP_FILTER_TIMEOUT_S without a good
                    # frame we accept anyway, so we don't get stuck after a
                    # genuine target loss.
                    accepted = True
                    if ENABLE_JUMP_FILTER and self.last_good_time > 0:
                        gap = now - self.last_good_time
                        if gap < JUMP_FILTER_TIMEOUT_S:
                            dx = result['body_x'] - self.last_good_x
                            dy = result['body_y'] - self.last_good_y
                            dz = abs(result['dist'] - self.last_good_z)
                            jump_xy = float(np.sqrt(dx*dx + dy*dy))
                            if jump_xy > MAX_FRAME_JUMP_M or dz > MAX_FRAME_ALT_JUMP_M:
                                accepted = False
                                frame_jump_rejected = True
                                self.jump_reject_count += 1
                                # Log occasionally so we can see if filter
                                # is firing too often (indicates real vibration
                                # problem, not just transient outliers)
                                if self.jump_reject_count % 20 == 1:
                                    self.logger.warning(
                                        f"JUMP REJECT: dxy={jump_xy*100:.0f}cm "
                                        f"dz={dz*100:.0f}cm "
                                        f"(total rejected: {self.jump_reject_count})")
                    
                    if accepted:
                        self.target_detected = True
                        self.last_detection_time = now
                        self.body_x = result['body_x']
                        self.body_y = result['body_y']
                        self.dist = result['dist']
                        self.primary_tag_id = result['primary_tag_id']
                        self.tags_visible = [t['id'] for t in result['all_tags']]
                        self.detection_count += 1
                        
                        # Update last-good state for next frame's filter check
                        self.last_good_x = result['body_x']
                        self.last_good_y = result['body_y']
                        self.last_good_z = result['dist']
                        self.last_good_time = now
                        
                        # Velocity update (only with accepted frames)
                        self.vel_estimator.add_measurement(
                            result['body_x'], result['body_y'], now
                        )
                        self.velocity_safe = (self.vel_estimator.speed <= MAX_SAFE_VELOCITY)
                        
                        # Tilt
                        self.roll, self.pitch, self.yaw, self.tilt_safe = \
                            self._estimate_tilt(result['primary_rvec'])
                        
                        # Safety warnings (rate limited)
                        if not self.velocity_safe:
                            self.vel_warn_count += 1
                            if self.vel_warn_count % 30 == 1:
                                self.logger.warning(
                                    f"PLATFORM TOO FAST: {self.vel_estimator.speed:.2f}m/s")
                        
                        if not self.tilt_safe:
                            self.tilt_warn_count += 1
                            if self.tilt_warn_count % 30 == 1:
                                self.logger.warning(
                                    f"UNSAFE TILT: Roll:{self.roll:+.1f} "
                                    f"Pitch:{self.pitch:+.1f}")
                        
                        # Send to FC
                        self._send_landing_target(
                            result['body_x'], result['body_y'], result['dist']
                        )
                    
                    # Periodic log
                    if self.frame_count % 30 == 0:
                        offset = float(np.sqrt(result['body_x']**2 + result['body_y']**2))
                        vel = self.vel_estimator
                        disagree_str = ""
                        if result['n_tags'] > 1:
                            disagree_str = f" Disagree:{result['disagreement']*100:.1f}cm"
                        self.logger.info(
                            f"Tag{self.primary_tag_id}({len(self.tags_visible)}vis) "
                            f"X:{result['body_x']:+.2f} Y:{result['body_y']:+.2f} "
                            f"Alt:{result['dist']:.2f} Off:{offset:.2f} "
                            f"Vel:{vel.speed:.2f}m/s Q:{vel.quality:.2f} "
                            f"R:{self.roll:+.1f} P:{self.pitch:+.1f} "
                            f"MAV:{self.mavlink_sent_count}{disagree_str}")
                else:
                    if time.time() - self.last_detection_time > MAX_DETECTION_AGE:
                        if self.target_detected:
                            self.logger.warning("Target LOST")
                            self.target_detected = False
                            self.tags_visible = []
                            self.vel_estimator.reset()
                            # Reset jump filter so first detection after target
                            # loss is always accepted (we have no recent
                            # reference to compare against).
                            self.last_good_time = 0.0
            
            # ---- Video recording: write every frame with overlay burned in ----
            # We annotate a copy of the frame here regardless of snapshot timing,
            # so the video shows continuous live detection state. The overlay
            # function is the same one snapshots use, so the video and snapshots
            # match visually.
            if ENABLE_RECORDING:
                target_seen = (result is not None)
                self._update_video_state(target_seen, time.time())
                if self.video_writer is not None:
                    # Build BGR frame for video
                    if len(frame.shape) == 2:
                        vid_frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    else:
                        vid_frame = cv2.cvtColor(frame.copy(), cv2.COLOR_RGB2BGR)
                    self._draw_overlay(vid_frame, result, full_overlay=True)
                    self._write_video_frame(vid_frame)
            
            # ---- CSV flight data: one row per frame for post-flight plotting ----
            if ENABLE_CSV_LOG:
                self._write_csv_row(time.time() - self.start_time,
                                    result, frame_jump_rejected)
            
            # Snapshot (still frames at slower cadence)
            if self.frame_count % SNAPSHOT_EVERY == 0:
                self._save_snapshot(frame, result)
                
        except Exception as e:
            self.logger.error(f"process_frame error: {e}\n{traceback.format_exc()}")
    
    # --------------------------------------------------------
    # STATUS THREAD
    # --------------------------------------------------------
    
    def _status_thread(self):
        while self.running:
            try:
                time.sleep(10)
                runtime = time.time() - self.start_time
                fps_avg = self.frame_count / runtime if runtime > 0 else 0
                det_pct = (self.detection_count / self.frame_count * 100
                           if self.frame_count > 0 else 0)
                
                with self.lock:
                    self.logger.info("=" * 70)
                    state = 'DETECTED' if self.target_detected else 'SEARCHING'
                    self.logger.info(
                        f"STATUS: {state} runtime:{runtime:.0f}s "
                        f"fps:{fps_avg:.1f} det:{self.detection_count}({det_pct:.0f}%) "
                        f"MAV:{self.mavlink_sent_count}")
                    
                    if self.target_detected:
                        vel = self.vel_estimator
                        self.logger.info(
                            f"  Pos: X={self.body_x:+.3f} Y={self.body_y:+.3f} "
                            f"Alt={self.dist:.3f}  Tags:{self.tags_visible}")
                        self.logger.info(
                            f"  Vel: Vx={vel.vel_x:+.3f} Vy={vel.vel_y:+.3f} "
                            f"Speed={vel.speed:.3f} Q={vel.quality:.2f}")
                        self.logger.info(
                            f"  Tilt: R={self.roll:+.1f} P={self.pitch:+.1f} "
                            f"Y={self.yaw:+.1f}")
                    
                    self.logger.info(
                        f"  Warnings: vel={self.vel_warn_count} "
                        f"tilt={self.tilt_warn_count} snaps={self.snap_count}")
                    self.logger.info("=" * 70)
            except Exception as e:
                self.logger.error(f"Status thread error: {e}")
    
    # --------------------------------------------------------
    # MAIN LOOP
    # --------------------------------------------------------
    
    def run(self):
        if not self.connect_fc():
            self.logger.error("Exiting - no FC connection")
            return
        
        if not self.init_camera():
            self.logger.error("Exiting - camera failed")
            return
        
        Thread(target=self._status_thread, daemon=True).start()
        
        self.logger.info("=" * 70)
        self.logger.info("SYSTEM READY - Detection starting")
        self.logger.info(f"  Snapshots: {SNAPSHOT_DIR}")
        self.logger.info(f"  Videos:    {VIDEO_DIR}")
        self.logger.info(f"  CSV:       {CSV_DIR}")
        self.logger.info(f"  Log file:  {LOG_FILE}")
        self.logger.info(f"  Monitor:   tail -f {LOG_FILE}")
        self.logger.info("=" * 70)
        
        self.start_time = time.time()
        self.boot_time = time.time()
        
        # Open CSV after start_time is set (header logged in _open_csv)
        self._open_csv()
        
        try:
            while self.running:
                try:
                    # Single RGB888 preview stream at 640x480 (full-FOV mode).
                    frame = self.picam2.capture_array()
                    
                    if frame is not None and frame.size > 0:
                        # Undistort BEFORE detection. After remap, the frame
                        # represents a perfect pinhole projection (no lens
                        # distortion), so AprilTag corner detection and the
                        # pose-estimation solvePnP that follows are both
                        # geometrically consistent with the CAMERA_MATRIX
                        # intrinsics.
                        frame = cv2.remap(frame, self.undist_mapx, self.undist_mapy,
                                          interpolation=cv2.INTER_LINEAR)
                        self._process_frame(frame)
                    else:
                        self.logger.warning("Empty frame received")
                        time.sleep(0.1)
                    
                    time.sleep(0.005)
                    
                except Exception as e:
                    self.logger.error(f"Main loop error: {e}\n{traceback.format_exc()}")
                    time.sleep(0.5)
                    
        except KeyboardInterrupt:
            self.logger.info("Shutdown requested (Ctrl+C)")
        finally:
            self.running = False
            self._cleanup()
    
    def _cleanup(self):
        self.logger.info("=" * 70)
        self.logger.info("SHUTTING DOWN")
        
        # Close active video file FIRST so the file is finalized correctly
        try:
            if self.video_writer is not None:
                self._close_video_writer()
        except Exception as e:
            self.logger.error(f"Video close error: {e}")
        
        # Close CSV next so all rows are flushed to disk
        try:
            self._close_csv()
        except Exception as e:
            self.logger.error(f"CSV close error: {e}")
        
        try:
            if self.picam2:
                self.picam2.stop()
                self.logger.info("Camera stopped")
        except Exception as e:
            self.logger.error(f"Camera stop error: {e}")
        
        try:
            if self.master:
                self.master.close()
                self.logger.info("MAVLink closed")
        except Exception as e:
            self.logger.error(f"MAVLink close error: {e}")
        
        runtime = time.time() - self.start_time
        self.logger.info("SESSION SUMMARY")
        self.logger.info(f"  Runtime:           {runtime:.1f}s")
        self.logger.info(f"  Frames processed:  {self.frame_count}")
        self.logger.info(f"  Detections:        {self.detection_count}")
        self.logger.info(f"  MAVLink sent:      {self.mavlink_sent_count}")
        self.logger.info(f"  Velocity warnings: {self.vel_warn_count}")
        self.logger.info(f"  Tilt warnings:     {self.tilt_warn_count}")
        self.logger.info(f"  Jump rejects:      {self.jump_reject_count}")
        self.logger.info(f"  Snapshots:         {self.snap_count}")
        self.logger.info(f"  Videos recorded:   {self.video_count}")
        self.logger.info(f"  CSV rows:          {self.csv_row_count}")
        if self.frame_count > 0 and runtime > 0:
            self.logger.info(f"  Avg FPS:           {self.frame_count/runtime:.1f}")
            self.logger.info(f"  Detection rate:    "
                             f"{self.detection_count/self.frame_count*100:.1f}%")
        self.logger.info("=" * 70)
        self.logger.info("Shutdown complete")


# ============================================================
if __name__ == "__main__":
    try:
        DronePrecisionLanding().run()
    except Exception as e:
        logging.error(f"Fatal: {e}\n{traceback.format_exc()}")
        sys.exit(1)