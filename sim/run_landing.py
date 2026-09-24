import time
import csv
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
from src.guided_lander import GuidedLander
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
    parser.add_argument("--no-gui", action="store_true",
                         help="skip cv2.imshow/waitKey and the detection-overlay drawing "
                              "-- real per-frame CPU cost that isn't needed for an actual "
                              "landing run, only for watching it live. Try this first if "
                              "ArduPilot's PrecLand keeps hitting 'Init Failed' (needs a "
                              "LANDING_TARGET update at least every 500ms during its 2s "
                              "init window -- AC_PrecLand.cpp's EKF_INIT_SENSOR_MIN_UPDATE_MS "
                              "-- and this loop being CPU-bound can blow that budget even "
                              "when detection itself is working fine).")
    parser.add_argument("--control", choices=["plnd", "guided"], default="plnd",
                         help="'plnd' (default): send LANDING_TARGET, let ArduPilot's own "
                              "PLND_TYPE=1 correct during LAND mode -- needs sim/"
                              "precision_landing.parm loaded and a continuous detection "
                              "rate (AC_PrecLand's EKF init needs an update at least every "
                              "500ms). 'guided': bypass PLND entirely -- src/guided_lander.py "
                              "sends GUIDED-mode velocity setpoints directly from our own "
                              "fused body_x/body_y, switching to real LAND only once "
                              "centered and near the ground for the final disarm. Vehicle "
                              "must already be armed and in GUIDED mode, airborne, before "
                              "starting this script in --control guided.")
    parser.add_argument("--log-file", type=str, default=None,
                         help="CSV path to log every frame's outcome (elapsed_s, event, "
                              "body_x, body_y, dist, tags_used) -- this is the ONLY record "
                              "of what the vision pipeline actually saw once the terminal "
                              "closes; the console print above is throttled to 2/sec for "
                              "readability and doesn't survive the session. Default: "
                              "logs/vision_<timestamp>.csv (found needed 2026-09-24, "
                              "diagnosing a run after the terminal was already gone --"
                              "had to reconstruct events from ArduPilot's own dataflash log "
                              "instead of this script's own data).")
    args = parser.parse_args()

    log_path = args.log_file or os.path.join(
        os.path.dirname(__file__), "..", "logs",
        f"vision_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    )
    os.makedirs(os.path.dirname(os.path.abspath(log_path)), exist_ok=True)
    log_file = open(log_path, "w", newline="", buffering=1)  # line-buffered -- survives a
    # crash/kill instead of losing whatever's still sitting in an OS-level write buffer
    log_writer = csv.writer(log_file)
    log_writer.writerow(["elapsed_s", "event", "body_x", "body_y", "dist", "tags_used"])
    log_start_t = time.time()
    print(f"Logging every frame to {log_path}")

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

    if args.control == "guided":
        print("Initializing GuidedLander (direct GUIDED-mode control, no PLND)...")
        actuator = GuidedLander(master)
    else:
        actuator = LandingTargetSender(master, own_connection=True)
    print("Starting vision loop..." + (" (--no-gui: no display window)" if args.no_gui else "")
          + f" [control={args.control}]")

    if not args.no_gui:
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

            # Draw detections for visual feedback -- skipped under --no-gui, real
            # per-frame CPU cost with no effect on the actual landing.
            if not args.no_gui and ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for tag_id, (rvec, tvec) in results.items():
                    cv2.drawFrameAxes(frame, source.camera_matrix, source.dist_coeffs, rvec, tvec, 0.1)

            fused = fusion.fuse_tags(results, corners, ids, frame.shape)

            if fused:
                if args.control == "guided":
                    actuator.send_correction(fused['body_x'], fused['body_y'])
                else:
                    actuator.send(
                        fused['body_x'],
                        fused['body_y'],
                        fused['dist'],
                        tag_size_m=CENTER_TAG_SIZE_M
                    )
                if not had_target or start_t - last_print_t > 0.5:
                    print(f"Sent TARGET: X={fused['body_x']:.2f}, Y={fused['body_y']:.2f}, Z={fused['dist']:.2f}, tags={fused['tags_used']}")
                    last_print_t = start_t
                had_target = True
                log_writer.writerow([f"{start_t - log_start_t:.3f}", "target",
                                      f"{fused['body_x']:.3f}", f"{fused['body_y']:.3f}",
                                      f"{fused['dist']:.3f}", fused['tags_used']])
            else:
                if args.control == "guided":
                    actuator.on_target_lost()
                if had_target:
                    print("Target lost")
                had_target = False
                log_writer.writerow([f"{start_t - log_start_t:.3f}", "lost", "", "", "", ""])

            if not args.no_gui:
                fps = 1.0 / (time.time() - start_t)
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow("Synthetic Camera", frame)
                if cv2.waitKey(1) & 0xFF == 27: # ESC
                    break

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        source.close()
        if args.control != "guided":
            actuator.close()
        if not args.no_gui:
            cv2.destroyAllWindows()
        log_file.close()
        print(f"Log written to {log_path}")

if __name__ == "__main__":
    main()
