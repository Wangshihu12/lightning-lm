#!/usr/bin/env python3
"""
Merge multiple ROS1 bag files into a single bag.
Usage:
    python3 merge_bags.py /path/to/bag_dir [output.bag]
"""
import os
import sys
from pathlib import Path

import rosbag


def find_bags(root_dir):
    root = Path(root_dir)
    if not root.exists():
        raise FileNotFoundError(f"{root} does not exist")
    return sorted(str(p) for p in root.rglob("*.bag"))


def merge_bags(bag_paths, output_path):
    if not bag_paths:
        print("No bag files found, nothing to do.")
        return

    output_path = Path(output_path)
    if output_path.exists():
        output_path.unlink()

    total_msgs = 0
    with rosbag.Bag(str(output_path), "w") as outbag:
        for bag_path in bag_paths:
            print(f"Merging {bag_path} ...")
            with rosbag.Bag(bag_path, "r") as inbag:
                for topic, msg, t in inbag.read_messages():
                    outbag.write(topic, msg, t)
                    total_msgs += 1

    print(f"Done. Wrote {total_msgs} messages from {len(bag_paths)} bags to {output_path}.")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    source_dir = sys.argv[1]
    bags = find_bags(source_dir)
    if len(sys.argv) > 2:
        output = sys.argv[2]
    else:
        output = os.path.join(source_dir, "merged.bag")

    merge_bags(bags, output)


if __name__ == "__main__":
    main()
