#!/usr/bin/python3

import rospy
import actionlib
import move_base_lite_msgs.msg
import geometry_msgs.msg


def create_goal(speed=1):
    goal = move_base_lite_msgs.msg.FollowPathGoal()
    goal.target_path.header.frame_id = "world"
    goal.target_path.poses.append(create_pose([0, -1, 0]))
    goal.target_path.poses.append(create_pose([0, 1, 0]))
    goal.follow_path_options.desired_speed = speed
    return goal


def create_pose(position):
    msg = geometry_msgs.msg.PoseStamped()
    msg.header.frame_id = "world"
    msg.pose.position.x = position[0]
    msg.pose.position.y = position[1]
    msg.pose.position.z = position[2]
    msg.pose.orientation.w = 1
    return msg


class FollowPathClient:
    def __init__(self):
        self._client = actionlib.SimpleActionClient("/controller/follow_path", move_base_lite_msgs.msg.FollowPathAction)
        rospy.loginfo("Waiting for server")
        self._client.wait_for_server()

    def send_goal(self):
        rospy.loginfo("Sending goal")
        goal = create_goal()
        self._client.send_goal(goal)
        rospy.loginfo("Waiting for result")
        self._client.wait_for_result()
        result = self._client.get_result()
        return result


def main():
    rospy.init_node("follow_path_test")
    follow_path_client = FollowPathClient()
    result = follow_path_client.send_goal()
    error_code = result.result.val
    if error_code == move_base_lite_msgs.msg.ErrorCodes.SUCCESS:
        print_msg = "SUCCESS"
    else:
        print_msg = "Received error " + str(error_code)
    rospy.loginfo("Result received: " + print_msg)


if __name__ == "__main__":
    main()
