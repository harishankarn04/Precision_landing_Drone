"""
mavlink_bridge — the ROS 2 <-> ArduPilot boundary.

This is the ONLY node that speaks MAVLink. Everything else in this project is a
plain ROS 2 node that publishes/subscribes ordinary messages, which keeps the
perception code free of autopilot details.

Two directions:

  ArduPilot --> ROS 2   vehicle pose on  /drone/pose   (geometry_msgs/PoseStamped)
  ROS 2 --> ArduPilot   landing target from /landing_target (geometry_msgs/PointStamped)
                        forwarded as a MAVLink LANDING_TARGET message

The /landing_target point is in the drone's BODY frame, in metres:
    x = forward, y = right, z = down   (so z is the height above the pad)

Why a separate node: on the real aircraft this exact boundary exists too — the
Raspberry Pi talks MAVLink over a UART to the flight controller. Here it is TCP
to SITL instead. Same architecture, so the perception nodes never change.
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from pymavlink import mavutil


class MavlinkBridge(Node):
    def __init__(self):
        super().__init__("mavlink_bridge")

        # SITL exposes SERIAL1 on TCP 5762 (SERIAL0/5760 is left for MAVProxy).
        # From inside Docker the Mac host is reachable as host.docker.internal.
        self.declare_parameter("connection", "tcp:host.docker.internal:5762")
        conn_str = self.get_parameter("connection").value

        # Retry rather than dying. SITL does not open SERIAL1 (TCP 5762) until a
        # ground station has connected to SERIAL0 first, so on a cold start this
        # port genuinely does not exist yet. Waiting removes any dependency on
        # the order the three processes get launched in.
        self.get_logger().info(f"connecting to autopilot at {conn_str} ...")
        self.mav = None
        while self.mav is None:
            try:
                self.mav = mavutil.mavlink_connection(
                    conn_str, source_system=1, source_component=191
                )
            except Exception as exc:            # noqa: BLE001 - report and retry
                self.get_logger().info(f"  autopilot not up yet ({exc}); retrying in 2 s")
                time.sleep(2.0)

        self.mav.wait_heartbeat()
        self.get_logger().info(
            f"heartbeat received from system {self.mav.target_system} "
            f"component {self.mav.target_component}"
        )

        # A fresh MAVLink link is SILENT until you ask for data. ArduPilot sends
        # telemetry per-link, so the ground station's stream request on another
        # port does nothing for us. Ask for LOCAL_POSITION_NED at 20 Hz.
        # This bites on real hardware too: the Pi's UART is its own link.
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            50000,  # microseconds between messages = 20 Hz
            0, 0, 0, 0, 0,
        )
        self.get_logger().info("requested LOCAL_POSITION_NED at 20 Hz")

        # ArduPilot rejects LANDING_TARGET messages whose timestamp is Unix epoch;
        # it wants microseconds since *its* boot. We track our own boot reference
        # and send relative time, which is what the autopilot actually checks.
        self._boot_ref = self.get_clock().now().nanoseconds

        self.pose_pub = self.create_publisher(PoseStamped, "/drone/pose", 10)
        self.create_subscription(PointStamped, "/landing_target", self.on_landing_target, 10)

        # Pump MAVLink at 20 Hz. Cheap, and well above the ~15 fps the real
        # perception pipeline manages on a Raspberry Pi.
        self.create_timer(0.05, self.pump)

        self.target_count = 0

    def _boot_time_us(self) -> int:
        """Microseconds since this node started — the format ArduPilot expects."""
        return int((self.get_clock().now().nanoseconds - self._boot_ref) / 1000)

    def pump(self):
        """Drain inbound MAVLink and republish what we care about."""
        while True:
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                return
            if msg.get_type() == "LOCAL_POSITION_NED":
                p = PoseStamped()
                p.header.stamp = self.get_clock().now().to_msg()
                p.header.frame_id = "map"
                p.pose.position.x = float(msg.x)
                p.pose.position.y = float(msg.y)
                p.pose.position.z = float(msg.z)
                p.pose.orientation.w = 1.0
                self.pose_pub.publish(p)

    def on_landing_target(self, msg: PointStamped):
        """Forward a body-frame landing-target offset to ArduPilot."""
        x, y, z = msg.point.x, msg.point.y, msg.point.z

        # Height above the pad. Guard against a zero/negative value, which would
        # blow up the angle calculation below.
        if z <= 0.01:
            self.get_logger().warn(f"ignoring landing target with z={z:.3f} m")
            return

        distance = math.sqrt(x * x + y * y + z * z)

        # Apparent angular size of the 60 cm board at this range. Must be
        # non-zero — ArduPilot's sanity check drops the message otherwise.
        size = 2.0 * math.atan2(0.30, distance)

        # Two things ArduPilot is strict about, both learned the hard way:
        #
        # 1. FRAME. AC_PrecLand_MAVLink::handle_msg accepts ONLY
        #    MAV_FRAME_BODY_FRD or MAV_FRAME_LOCAL_FRD. Anything else -- and
        #    MAV_FRAME_BODY_NED in particular -- is discarded outright.
        #
        # 2. position_valid=1 lets us hand over the offset vector directly.
        #    With position_valid=0 ArduPilot instead reconstructs the direction
        #    from the angle fields as (-tan(angle_y), tan(angle_x), 1), i.e.
        #    angle_x encodes RIGHT and angle_y encodes negated FORWARD. That is
        #    easy to get backwards, and a sign error here flies the aircraft
        #    away from the pad rather than toward it. Sending the vector avoids
        #    the trap completely.
        #
        # FRD is exactly our convention already: x forward, y right, z down.
        self.mav.mav.landing_target_send(
            self._boot_time_us(),
            0,                                          # target_num
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            0.0,                                        # angle_x  (unused)
            0.0,                                        # angle_y  (unused)
            distance,
            size,                                       # size_x
            size,                                       # size_y
            x, y, z,                                    # offset vector, FRD
            (1.0, 0.0, 0.0, 0.0),                       # q (unused)
            mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
            1,                                          # position_valid
        )

        self.target_count += 1
        if self.target_count % 20 == 0:
            self.get_logger().info(
                f"sent {self.target_count} targets | "
                f"x={x:+.2f} y={y:+.2f} alt={z:.2f} dist={distance:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
