import argparse
from pathlib import Path

from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions


def odom_to_tum_line(msg, stamp):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return f"{stamp:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n"


def export_topic(bag_path, topic, output_path, stamp_source):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    topics = {info.name: info.type for info in reader.get_all_topics_and_types()}
    if topic not in topics:
        available = "\n".join(sorted(topics.keys()))
        raise RuntimeError(f"Topic {topic} not found. Available topics:\n{available}")
    if topics[topic] != "nav_msgs/msg/Odometry":
        raise RuntimeError(f"Topic {topic} has type {topics[topic]}, expected nav_msgs/msg/Odometry")

    count = 0
    first_stamp = None
    last_stamp = None
    first_bag_stamp = None
    last_bag_stamp = None
    with open(output_path, "w", encoding="utf-8") as f:
        while reader.has_next():
            topic_name, data, bag_time_ns = reader.read_next()
            if topic_name != topic:
                continue
            msg = deserialize_message(data, Odometry)
            bag_stamp = bag_time_ns * 1e-9
            if stamp_source == "bag":
                stamp = bag_stamp
            else:
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            f.write(odom_to_tum_line(msg, stamp))
            if first_stamp is None:
                first_stamp = stamp
                first_bag_stamp = bag_stamp
            last_stamp = stamp
            last_bag_stamp = bag_stamp
            count += 1
    return count, first_stamp, last_stamp, first_bag_stamp, last_bag_stamp


def main():
    parser = argparse.ArgumentParser(description="Export a nav_msgs/Odometry topic from a ROS 2 bag to TUM trajectory format.")
    parser.add_argument("--bag", required=True, help="ROS 2 bag directory.")
    parser.add_argument("--topic", required=True, help="Odometry topic to export.")
    parser.add_argument("--out", required=True, help="Output text file.")
    parser.add_argument(
        "--stamp-source",
        choices=["header", "bag"],
        default="header",
        help="Use message header stamp or rosbag receive timestamp.",
    )
    args = parser.parse_args()

    bag_path = Path(args.bag).expanduser().resolve()
    output_path = Path(args.out).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    count, first_stamp, last_stamp, first_bag_stamp, last_bag_stamp = export_topic(
        bag_path, args.topic, output_path, args.stamp_source
    )
    print(f"Exported {count} odometry messages from {args.topic} to {output_path}")
    if count:
        print(f"tum_stamp_range: {first_stamp:.9f} -> {last_stamp:.9f} ({last_stamp - first_stamp:.3f} s)")
        print(
            f"bag_time_range: {first_bag_stamp:.9f} -> {last_bag_stamp:.9f} "
            f"({last_bag_stamp - first_bag_stamp:.3f} s)"
        )


if __name__ == "__main__":
    main()
