#!/usr/bin/env python3
"""
Stage 2, step 2 test: prove src/mavlink_out.py's LANDING_TARGET wiring works against
live SITL BEFORE any detection/fusion code exists to blame if it doesn't.

Sends a small hardcoded circular sweep of fake body_x/body_y positions at a fixed
distance -- no camera, no detector, just a known synthetic signal. If this is wired
correctly (PLND_TYPE=1 in sim/precision_landing.parm, correct MAV_FRAME_BODY_FRD,
non-zero angular size), the drone in LAND mode should visibly chase the sweeping
point the same way it chased SIM_PLD's simulated target in Stage 1.

Usage (run against an already-running ./sim/run_sitl.sh, with the vehicle armed,
airborne, and in LAND mode -- same test ladder as Stage 1):
    .venv/bin/python sim/demo_mavlink_sweep.py
    .venv/bin/python sim/demo_mavlink_sweep.py --radius 2.0 --dist 8.0 --period 20
"""

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.mavlink_out import LandingTargetSender


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--radius", type=float, default=1.5,
                     help="sweep radius in metres (default: 1.5)")
    ap.add_argument("--dist", type=float, default=8.0,
                     help="fixed distance-to-target in metres (default: 8.0)")
    ap.add_argument("--period", type=float, default=15.0,
                     help="seconds per full circle (default: 15.0)")
    ap.add_argument("--rate-hz", type=float, default=10.0,
                     help="send rate in Hz (default: 10.0)")
    ap.add_argument("--connection", default="udpin:0.0.0.0:14540",
                     help="MAVLink connection string (default: udpin:0.0.0.0:14540, "
                          "matching sim/run_sitl.sh's COMPANION_PORT)")
    args = ap.parse_args()

    sender = LandingTargetSender(args.connection)
    print(f"Connecting to {args.connection} ...")
    if not sender.wait_ready():
        print("ERROR: no heartbeat -- is SITL running? (./sim/run_sitl.sh)")
        sys.exit(1)
    print("Connected. Sending fake LANDING_TARGET sweep -- Ctrl+C to stop.")
    print(f"  radius={args.radius}m dist={args.dist}m period={args.period}s "
          f"rate={args.rate_hz}Hz")

    start = time.time()
    dt = 1.0 / args.rate_hz
    try:
        while True:
            t = time.time() - start
            theta = 2.0 * math.pi * (t / args.period)
            body_x = args.radius * math.cos(theta)
            body_y = args.radius * math.sin(theta)
            sender.send(body_x, body_y, args.dist)
            print(f"\r  t={t:6.1f}s  body_x={body_x:+.2f}  body_y={body_y:+.2f}",
                  end="", flush=True)
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sender.close()


if __name__ == "__main__":
    main()
