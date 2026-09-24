import cv2
import numpy as np
from typing import Dict, Any, List

class ArucoDetector:
    # Substitute for the senior's "reject decision margin < 25" spec (CLAUDE.md section
    # 3) -- cv2.aruco doesn't expose a continuous decision-margin score the way
    # pupil-apriltags does (confirmed by a teammate's independent reference project,
    # github.com/format37/courierquad, which uses pupil-apriltags specifically for that
    # metric). solvePnP's own reprojection error serves the same purpose with what this
    # library actually gives us: a marginal/garbage corner detection produces a poor
    # pose fit, which shows up directly as high reprojection error. Found 2026-09-24
    # after live Gazebo tests showed detection flickering frame-to-frame with no
    # filtering at all -- some fraction of those "detections" were likely exactly this
    # kind of weak/marginal corner fit feeding bad data downstream.
    MAX_REPROJECTION_ERROR_PX = 5.0

    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray, tag_dict: Dict[int, Any]):
        """
        tag_dict is typically the TAGS dict from sim.board
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.tag_dict = tag_dict

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG

        # In OpenCV 4.7+, we use ArucoDetector.
        try:
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        except AttributeError:
            # Fallback for older OpenCV
            self.detector = None

        # Precompute object points for each tag
        # We need to import tag_corners_board_frame inside to avoid circular import if used differently,
        # but we can just import it at top level.
        from sim.board import tag_corners_board_frame
        self.obj_pts = {}
        for tag_id, tag in self.tag_dict.items():
            pts_3d = tag_corners_board_frame(tag)
            self.obj_pts[tag_id] = np.array(pts_3d, dtype=np.float32)

    def detect_and_solve(self, frame: np.ndarray):
        """
        Detect tags and solve PnP for each.
        Returns a dictionary mapping tag_id -> (rvec, tvec)
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)

        results = {}

        if ids is not None:
            for i, tag_id in enumerate(ids.flatten()):
                if tag_id in self.obj_pts:
                    # In CLAUDE.md: "Do not pass distortion coefficients to solvePnP after already undistorting the frame"
                    # If frame is not undistorted, we pass dist_coeffs.
                    # Since our synthetic camera generates perfect pinhole images with 0 distortion,
                    # passing self.dist_coeffs (which are zeros) is fine.
                    # If using real hardware, either undistort first or pass dist_coeffs here.

                    # solvePnP ITERATIVE requires minimum 4 points, which we have.
                    # solvePnP expects shape (N, 3) for object points and (N, 2) for image points.
                    img_pts = corners[i].reshape(4, 2)
                    success, rvec, tvec = cv2.solvePnP(
                        self.obj_pts[tag_id],
                        img_pts,
                        self.camera_matrix,
                        self.dist_coeffs,
                        flags=cv2.SOLVEPNP_ITERATIVE
                    )

                    if success:
                        reprojected, _ = cv2.projectPoints(
                            self.obj_pts[tag_id], rvec, tvec,
                            self.camera_matrix, self.dist_coeffs
                        )
                        reproj_error_px = float(np.mean(
                            np.linalg.norm(reprojected.reshape(4, 2) - img_pts, axis=1)
                        ))
                        if reproj_error_px <= self.MAX_REPROJECTION_ERROR_PX:
                            results[tag_id] = (rvec, tvec)
                        # else: marginal/garbage corner fit, silently dropped -- same
                        # tag_id simply won't appear in results this frame.

        return results, corners, ids

if __name__ == "__main__":
    # Standalone corner-order sanity check.
    #
    # An earlier version of this test only called cv2.projectPoints() to make fake
    # "detected" corners directly from board.py's own object points, then solved PnP
    # against those same object points -- trivially self-consistent by construction,
    # since it never touched the real cv2.aruco detector at all. It could not have
    # caught a genuine corner-order mismatch (cv2.aruco's ArUco-based AprilTag detector
    # is known to return corners in a different order than the upstream AprilTag
    # library's own convention: https://github.com/opencv/opencv-python/issues/1195).
    #
    # This version renders an ACTUAL AprilTag image (sim/synthetic_camera.py's real
    # rendering path) at several known, non-symmetric poses, runs it through the REAL
    # ArucoDetector.detect_and_solve(), and checks the recovered pose against ground
    # truth -- this is what would actually catch a corner-order mismatch if one exists.
    # Confirmed 2026-09-17: passes across straight-down, yawed, and tilted/off-center
    # poses -- board.py's tag_corners_board_frame() ordering matches what cv2.aruco
    # actually returns, at least for DICT_APRILTAG_36h11 on this OpenCV build.
    import numpy as np
    from sim.board import TAGS
    from sim.synthetic_camera import SyntheticSource

    def make_source():
        src = SyntheticSource.__new__(SyntheticSource)
        src.target_pos_ned = np.array([0.0, 0.0, 0.0])
        src.width, src.height = 640, 480
        src.fx = src.fy = 500.0
        src.cx, src.cy = 320.0, 240.0
        src.camera_matrix = np.array(
            [[src.fx, 0, src.cx], [0, src.fy, src.cy], [0, 0, 1]], dtype=float
        )
        src.dist_coeffs = np.zeros(5, dtype=float)
        src._build_board_texture()
        src._update_pose_from_mavlink = lambda: None
        src.master = None
        return src

    cam_mat = make_source().camera_matrix
    dist = np.zeros(5, dtype=float)
    det = ArucoDetector(cam_mat, dist, TAGS)

    test_cases = [
        ("straight down, centered",   (0.0, 0.0, -3.0), 0.0,  0.0,  0.0),
        ("straight down + yaw 30deg", (0.0, 0.0, -3.0), 0.0,  0.0,  0.5236),
        ("off-center + roll/pitch",   (0.5, -0.3, -4.0), 0.15, -0.1, 0.3),
        ("closer, more tilt",         (0.2, 0.1, -2.0), 0.25,  0.2, -0.4),
    ]

    all_passed = True
    for name, pos, roll, pitch, yaw in test_cases:
        src = make_source()
        src.pos_ned = np.array(pos)
        src.roll, src.pitch, src.yaw = roll, pitch, yaw
        frame, hint = src.get_frame()
        results, corners, ids = det.detect_and_solve(frame)

        if 0 not in results:
            print(f"[{name}] FAILED: tag 0 not detected at all")
            all_passed = False
            continue

        _, est_tvec = results[0]
        true_tvec_cam = hint['tvec']
        diff = float(np.linalg.norm(est_tvec.flatten() - true_tvec_cam))
        ok = diff < 0.05
        all_passed &= ok
        print(f"[{name}] true={true_tvec_cam} est={est_tvec.flatten()} "
              f"diff={diff:.4f}m -> {'OK' if ok else 'MISMATCH'}")

    print("\nSANITY CHECK", "PASSED" if all_passed else "FAILED",
          "-- corner order" if all_passed else "-- CORNER ORDER MISMATCH, do not trust detector.py")
