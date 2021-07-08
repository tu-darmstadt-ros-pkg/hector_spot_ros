#!/usr/bin/python3

import rospy

from hector_spot_ros.spot_ros_interface import SpotROSInterface

if __name__ == "__main__":
    # ROS setup
    rospy.init_node('spot_ros_interface')

    with SpotROSInterface():
        rospy.spin()
