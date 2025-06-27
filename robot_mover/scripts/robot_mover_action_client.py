#!/usr/bin/env python3

import rospy
import actionlib
import math

from geometry_msgs.msg import PoseStamped, Point, Quaternion

import tf_conversions

from welding_robot_msgs.msg import WeldingPathAction 
from welding_robot_msgs.msg import WeldingPathFeedback
from welding_robot_msgs.msg import WeldingPathGoal
from welding_robot_msgs.msg import WeldingPathResult


def get_pose(x, y, z):
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position = Point(x, y, z)
    quat = tf_conversions.transformations.quaternion_from_euler(math.pi, 0, 0)
    pose.pose.orientation = Quaternion(*quat)
    return pose


def test_server():
    rospy.init_node("robot_mover_action_client")
    client = actionlib.SimpleActionClient("robot_mover_action_server", WeldingPathAction)

    rospy.loginfo("Waiting for server...")
    client.wait_for_server()

    poses = [
        get_pose(0.3, 0.3, 0.3),
        get_pose(0.3, 0.3, 0.15),
        get_pose(0.3, 0.3, 0.2),
        get_pose(0.2, 0.2, 0.2),
        get_pose(0.2, 0, 0.25)
    ]

    goal = WeldingPathGoal(poses=poses, goal_description="This is my funny goal Description")
    client.send_goal(goal)

    rospy.loginfo("Goal sent, waiting for result...")
    client.wait_for_result()

    result = client.get_result()

    if result is not None:
        rospy.loginfo("Result received")
    else:
        rospy.logwarn("No result received")

if __name__ == "__main__":
    test_server()

