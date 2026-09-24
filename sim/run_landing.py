import time
import cv2
import sys
import os
import argparse
from pymavlink import mavutil

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from sim.synthetic_camera import SyntheticSource
from src.detector import ArucoDetector
from src.fusion import TagFusion
from src.mavlink_out import LandingTargetSender
from sim.board import TAGS, CENTER_TAG_SIZE_M

def main():
    parser = argparse.ArgumentParser(description="Precision Landing Vision Loop")
    parser.add_argument("--companion-port", type=str, default="udpin:0.0.0.0:14540",
                         help="MAVLink companion link (sim/run_sitl.sh's COMPANION_PORT) "
                              "-- used for BOTH reading vehicle pose and sending "
                              "LANDING_TARGET over ONE shared connection. NOT the GCS "
                              "port (14550) -- that's QGroundControl/Mission Planner's.")
    parser.add_argument("--source", choices=["synthetic", "gazebo"], default="synthetic",
                         help="frame source: 'synthetic' (sim/synthetic_camera.py, no "
                              "Gazebo needed) or 'gazebo' (src/gazebo_source.py, reads "
                              "the real rendered camera via gz-transport -- needs "
                              "sim/run_gazebo.sh running with the camera-equipped world)")
    args = parser.parse_args()

    print(f"Connecting to SITL companion link on {args.companion_port}...")
    # One shared MAVLink connection for both reading pose (SyntheticSource) and sending
    # LANDING_TARGET (LandingTargetSender) -- a MAVLink link is bidirectional, and two
    # separate sockets cannot both bind the same local port in the same process (this
    # is exactly the bug that caused the stuck/never-connecting first version of this
    # script; confirmed 2026-09-17).
    master = mavutil.mavlink_connection(args.companion_port)
    if not master.wait_heartbeat(timeout=30):
        print("Failed to connect to SITL within timeout. Check if it's running.")
        return
    print("MAVLink ready.")

    # target_pos_ned=(0,0,0) means the board is exactly at the drone's takeoff point --
    # true for both sources: SyntheticSource's own board render, and the real
    # precision_landing_board included at the world origin in
    # sim/gazebo_worlds/precision_landing_world.sdf.
    if args.source == "gazebo":
        print("Initializing Gazebo Source (real rendered camera via gz-transport)...")
        from src.gazebo_source import GazeboSource
        source = GazeboSource(master, target_pos_ned=(0, 0, 0), own_connection=False)
    else:
        print("Initializing Synthetic Camera...")
        source = SyntheticSource(master, target_pos_ned=(0, 0, 0), own_connection=False)

    print("Initializing Aruco Detector...")
    detector = ArucoDetector(source.camera_matrix, source.dist_coeffs, TAGS)

    print("Initializing Fusion...")
    fusion = TagFusion()

    sender = LandingTargetSender(master, own_connection=True)
    print("Starting vision loop...")

    cv2.namedWindow("Synthetic Camera", cv2.WINDOW_NORMAL)

    # Print on state transitions (target acquired/lost) and periodically while locked,
    # not every single frame -- "No valid target" at 30Hz drowned out everything else
    # (found 2026-09-24, live Gazebo mission test).
    had_target = False
    last_print_t = 0.0

    try:
        while True:
            start_t = time.time()

            frame, pose_hint = source.get_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            results, corners, ids = detector.detect_and_solve(frame)

            # Draw detections for visual feedback
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for tag_id, (rvec, tvec) in results.items():
                    cv2.drawFrameAxes(frame, source.camera_matrix, source.dist_coeffs, rvec, tvec, 0.1)

            fused = fusion.fuse_tags(results, corners, ids, frame.shape)

            if fused:
                sender.send(
                    fused['body_x'],
                    fused['body_y'],
                    fused['dist'],
                    tag_size_m=CENTER_TAG_SIZE_M
                )
                if not had_target or start_t - last_print_t > 0.5:
                    print(f"Sent TARGET: X={fused['body_x']:.2f}, Y={fused['body_y']:.2f}, Z={fused['dist']:.2f}, tags={fused['tags_used']}")
                    last_print_t = start_t
                had_target = True
            else:
                if had_target:
                    print("Target lost")
                had_target = False

            fps = 1.0 / (time.time() - start_t)
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
            cv2.imshow("Synthetic Camera", frame)
            if cv2.waitKey(1) & 0xFF == 27: # ESC
                break
                
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        source.close()
        sender.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
