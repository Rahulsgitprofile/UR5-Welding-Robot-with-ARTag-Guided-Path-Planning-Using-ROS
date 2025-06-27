#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import actionlib

from welding_robot_msgs.msg import WeldingPathAction 
from welding_robot_msgs.msg import WeldingPathFeedback
from welding_robot_msgs.msg import WeldingPathGoal
from welding_robot_msgs.msg import WeldingPathResult
from geometry_msgs.msg import PoseStamped

class RobotMoverActionServer():

    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander("manipulator")

    def __init__(self) -> None:
        self.action_server = actionlib.SimpleActionServer("robot_mover_action_server", WeldingPathAction, execute_cb=self.action_callback, auto_start=False)
        self.action_server.start()

    def action_callback(self, goal):
        rospy.loginfo("Received goal: %s", goal)

        # Extract poses from the goal
        #poses = [pose.pose for pose in goal.poses]
        #print("poses received:\n", poses)
        for pose in goal.poses:
            # Plan the movements for all poses
            self.move_group.set_pose_target(pose)
            plan = self.move_group.go(wait=True)
        
            # Check if the plan was successful
            if not plan:
                rospy.logwarn("Failed to plan movement")
                self.action_server.set_aborted()
                return

        # If all movements are successful, set the result as succeeded
        result = WeldingPathResult()
        self.action_server.set_succeeded(result)

if __name__ == "__main__":
    rospy.init_node("robot_mover_action_server")
    moveit_commander.roscpp_initialize(sys.argv)
    server = RobotMoverActionServer()
    rospy.spin()

