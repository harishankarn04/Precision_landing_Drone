#!/usr/bin/env python3
"""
Show the drone's downward camera, and detect any AprilTags in it.

    .venv/bin/python sim/camera_view.py            # live window
    .venv/bin/python sim/camera_view.py --headless # print detections only

Webots streams the camera as raw grayscale on TCP 5599: a 4-byte header
(width, height as unsigned shorts) followed by width*height bytes of pixels.

This is the first real step of Stage 2 — a genuine image, from a rendered 3D
world, with a real detector run on it. No perfect coordinates.
"""

import argparse
import socket
import struct
import sys
import time

import cv2
import numpy as np

HEADER = "=HH"
HEADER_SIZE = struct.calcsize(HEADER)


def recv_exactly(sock, n):
    """Read exactly n bytes, or return None if the stream closes."""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(min(n - len(buf), 8192))
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)


def frames(host, port):
    """Yield grayscale frames from the Webots camera stream."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"connecting to Webots camera at {host}:{port} ...")
    sock.connect((host, port))
    print("connected")
    try:
        while True:
            header = recv_exactly(sock, HEADER_SIZE)
            if header is None:
                return
            width, height = struct.unpack(HEADER, header)
            payload = recv_exactly(sock, width * height)
            if payload is None:
                return
            yield np.frombuffer(payload, np.uint8).reshape((height, width))
    finally:
        sock.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5599)
    ap.add_argument("--headless", action="store_true",
                    help="no window; just report detections")
    args = ap.parse_args()

    # tag36h11 is the family the thesis specifies. OpenCV's ArUco module can
    # decode AprilTag families directly, so pupil-apriltags is not needed —
    # which also keeps every coordinate in OpenCV's y-down convention and
    # sidesteps the sign trap the thesis documents.
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    detector = cv2.aruco.ArucoDetector(dictionary, params)

    count = 0
    last_report = time.time()
    try:
        for gray in frames(args.host, args.port):
            count += 1
            corners, ids, _ = detector.detectMarkers(gray)

            if time.time() - last_report > 1.0:
                h, w = gray.shape
                found = [] if ids is None else sorted(int(i) for i in ids.flatten())
                print(f"frame {count:5d}  {w}x{h}  tags={found}")
                last_report = time.time()

            if not args.headless:
                view = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(view, corners, ids)
                cv2.imshow("drone camera (q to quit)", view)
                if cv2.waitKey(1) == ord("q"):
                    break
    except ConnectionRefusedError:
        print(f"nothing listening on {args.host}:{args.port}", file=sys.stderr)
        print("Is Webots running with the world loaded and ▶ pressed?", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        if not args.headless:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
