#!/usr/bin/env python3
"""
Fly one automated precision landing and report the result.

    .venv/bin/python sim/demo_flight.py [--pad-north 2.0] [--alt 6]

Expects SITL (sim/run_sitl.sh) and the ROS 2 bridge to already be running.
The pad offset here must match the pad_north/pad_east given to the launch file.
"""

import argparse
import math
import time

from pymavlink import mavutil


def drain(m, texts):
    """Return the NEWEST position, collecting any status texts on the way.

    Two traps here, both of which produce convincing-looking wrong output:
      * Telemetry queues faster than a slow loop reads it, so a plain
        recv_match() returns ever-staler data — indistinguishable from a
        vehicle that has stopped moving.
      * recv_match(type=...) DISCARDS messages that don't match, so a separate
        STATUSTEXT poll silently never fires. Drain once, sort afterwards.
    """
    pos = None
    t0 = time.time()
    while time.time() - t0 < 1.0:
        msg = m.recv_match(blocking=False)
        if msg is None:
            if pos is not None:
                break
            time.sleep(0.02)
            continue
        kind = msg.get_type()
        if kind == "LOCAL_POSITION_NED":
            pos = msg
        elif kind == "STATUSTEXT":
            texts.append(msg.text)
        elif kind == "HEARTBEAT":
            drain.armed = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            drain.mode = msg.custom_mode
    return (pos.x, pos.y, -pos.z) if pos else (None, None, None)


# Latest known armed state and flight mode, refreshed by every drain() call.
# Reading these rather than calling recv_match('HEARTBEAT') avoids acting on a
# heartbeat that has been sitting in the socket buffer for tens of seconds.
drain.armed = False
drain.mode = None


def set_mode(m, texts, name, timeout=30):
    """Command a flight mode and CONFIRM it took.

    A mode change requested before the EKF has settled is rejected silently.
    Assuming it worked leaves the vehicle in the wrong mode, where arming still
    succeeds but TAKEOFF is refused — which looks like a broken takeoff.
    """
    want = m.mode_mapping()[name]
    t0 = time.time()
    while time.time() - t0 < timeout:
        m.set_mode_apm(want)
        time.sleep(1.0)
        drain(m, texts)
        if drain.mode == want:
            return True
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pad-north", type=float, default=2.0)
    ap.add_argument("--pad-east", type=float, default=0.0)
    ap.add_argument("--alt", type=float, default=6.0)
    ap.add_argument("--connect", default="tcp:127.0.0.1:5760")
    args = ap.parse_args()

    texts = []

    m = mavutil.mavlink_connection(args.connect)
    print(f"connecting to {args.connect} ...")
    m.wait_heartbeat(timeout=30)
    print("connected")

    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 100000, 0, 0, 0, 0, 0,
    )

    # Getting airborne needs GUIDED *and* armed at the same time, and the two
    # constrain each other from opposite directions:
    #   * a cold SITL rejects mode changes for ~30 s while the EKF settles
    #   * a vehicle left in LAND by a previous run refuses to arm at all
    # So push both every cycle until both stick, rather than assuming an order.
    guided = m.mode_mapping()["GUIDED"]
    print("\nwaiting for GUIDED + armed ...")
    t0 = time.time()
    while time.time() - t0 < 120:
        if drain.mode != guided:
            m.set_mode_apm(guided)
        if not drain.armed:
            m.arducopter_arm()
        time.sleep(2)
        drain(m, texts)
        if drain.armed and drain.mode == guided:
            break
        print(f"  mode={drain.mode} armed={drain.armed} "
              f"({time.time()-t0:.0f}s) ...")
        for t in texts[-3:]:
            if "PreArm" in t or "Arm" in t:
                print(f"    {t}")
    if not (drain.armed and drain.mode == guided):
        print("  FAILED to reach GUIDED+armed")
        for t in texts[-8:]:
            print(f"    {t}")
        return
    print("  armed, in GUIDED")

    # ArduPilot auto-disarms after ~10 s sitting armed on the ground, so the
    # takeoff has to be re-issued if that race is lost. Keep asking until the
    # vehicle is actually off the ground.
    print(f"takeoff to {args.alt:.0f} m ...")
    t0 = time.time()
    last_cmd = 0.0
    while time.time() - t0 < 90:
        _, _, alt = drain(m, texts)
        if alt is not None and alt > args.alt - 0.5:
            break
        if not drain.armed:
            m.arducopter_arm()
        if time.time() - last_cmd > 3.0:
            m.mav.command_long_send(
                m.target_system, m.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, args.alt,
            )
            last_cmd = time.time()
        time.sleep(0.5)
    else:
        print("  takeoff did not reach altitude — is the vehicle in GUIDED?")

    n0, e0, alt0 = drain(m, texts)
    start_err = math.hypot(n0 - args.pad_north, e0 - args.pad_east)
    print(f"  at N={n0:+.2f} E={e0:+.2f} alt={alt0:.2f}")
    print(f"  pad at N={args.pad_north:+.2f} E={args.pad_east:+.2f}"
          f"  -> {start_err:.2f} m away\n")

    print("LAND — corrections come from the ROS 2 node")
    set_mode(m, texts, "LAND")

    seen = set()
    t0 = time.time()
    while time.time() - t0 < 150:
        n, e, alt = drain(m, texts)
        for t in texts:
            if "PrecLand" in t and t not in seen:
                seen.add(t)
                print(f"  >>> {t}")
        if n is None:
            continue
        err = math.hypot(n - args.pad_north, e - args.pad_east)
        print(f"  t={time.time()-t0:5.1f}s  N={n:+6.2f} E={e:+6.2f} "
              f"alt={alt:5.2f}  err={err:5.2f} m")
        if alt < 0.20:
            break
        time.sleep(2)

    n, e, _ = drain(m, texts)
    final_err = math.hypot(n - args.pad_north, e - args.pad_east)
    acquired = any("PrecLand" in t and "Target Found" in t for t in texts)
    print("\n" + "=" * 46)
    print(f"  target acquired : {'yes' if acquired else 'NO — check the bridge'}")
    print(f"  start error     : {start_err:.2f} m")
    print(f"  final error     : {final_err:.2f} m")
    print("=" * 46)


if __name__ == "__main__":
    main()
