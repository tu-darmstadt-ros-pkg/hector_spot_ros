#!/usr/bin/python

import rosbag
import argparse
import os
import sensor_msgs.msg


def main(args):
    input_bag_path = args.bag
    splitext = os.path.splitext(input_bag_path)
    output_bag_path = splitext[0] + "_converted" + splitext[1]

    with rosbag.Bag(output_bag_path, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(input_bag_path).read_messages():
            if msg._type == "sensor_msgs/Image" and msg.encoding == "8UC1":
                msg.encoding = "mono8"
            outbag.write(topic, msg, t)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert image encodings')
    parser.add_argument('bag', type=str, help="Input bagfile")
    args = parser.parse_args()
    main(args)