import argparse

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class OdomToPath(Node):
    def __init__(self):
        super().__init__("odom_to_path")
        self.declare_parameter("odom_topic", "/gnss/odom")
        self.declare_parameter("path_topic", "/rtk_path")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("max_poses", 20000)
        self.declare_parameter("min_distance", 0.05)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.path_topic = self.get_parameter("path_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.max_poses = int(self.get_parameter("max_poses").value)
        self.min_distance = float(self.get_parameter("min_distance").value)

        self.path = Path()
        self.last_pose = None
        self.publisher = self.create_publisher(Path, self.path_topic, 10)
        self.subscription = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)
        self.get_logger().info(f"Building path: {self.odom_topic} -> {self.path_topic}")

    def on_odom(self, msg):
        pose = msg.pose.pose
        if self.last_pose is not None:
            dx = pose.position.x - self.last_pose.position.x
            dy = pose.position.y - self.last_pose.position.y
            dz = pose.position.z - self.last_pose.position.z
            if (dx * dx + dy * dy + dz * dz) ** 0.5 < self.min_distance:
                return

        stamped = PoseStamped()
        stamped.header = msg.header
        if self.frame_id:
            stamped.header.frame_id = self.frame_id
        stamped.pose = pose

        self.path.header = stamped.header
        self.path.poses.append(stamped)
        if self.max_poses > 0 and len(self.path.poses) > self.max_poses:
            self.path.poses = self.path.poses[-self.max_poses :]

        self.last_pose = pose
        self.publisher.publish(self.path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros-args", nargs="*")
    parser.parse_known_args()

    rclpy.init()
    node = OdomToPath()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
