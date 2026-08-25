"""
fake_target — a stand-in for the camera, so the bridge can be tested end to end.

Publishes a landing-target offset on /landing_target as if a perception node had
seen the marker board. The pad sits at a fixed spot underneath the drone and the
reported offset shrinks as the drone descends.

This exists ONLY to prove the ROS 2 -> MAVLink path works. It is replaced by the
real AprilTag detector later; nothing downstream changes when that happens.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped


class FakeTarget(Node):
    def __init__(self):
        super().__init__("fake_target")

        # Where the pad is, relative to where the drone started, in metres.
        self.declare_parameter("pad_north", 0.0)
        self.declare_parameter("pad_east", 0.0)
        self.pad_n = self.get_parameter("pad_north").value
        self.pad_e = self.get_parameter("pad_east").value

        self.pub = self.create_publisher(PointStamped, "/landing_target", 10)
        self.create_subscription(PoseStamped, "/drone/pose", self.on_pose, 10)

        self.get_logger().info(
            f"pretending a pad sits at N={self.pad_n:.2f} E={self.pad_e:.2f}"
        )

    def on_pose(self, msg: PoseStamped):
        # /drone/pose is LOCAL_POSITION_NED: x=north, y=east, z=down (negative up).
        north, east, down = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        altitude = -down

        if altitude <= 0.05:
            return  # on the ground, nothing to report

        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "body"
        # Offset from drone to pad. Yaw is ignored here — a real detector gets
        # this from the marker's pose, but for a bridge test it does not matter.
        out.point.x = self.pad_n - north      # forward
        out.point.y = self.pad_e - east       # right
        out.point.z = altitude                # height above the pad
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FakeTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
