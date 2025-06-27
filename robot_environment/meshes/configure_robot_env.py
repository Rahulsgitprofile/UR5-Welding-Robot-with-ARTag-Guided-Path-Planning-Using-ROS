#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Point, PointStamped, Pose, Quaternion
import sys

import rospkg


import moveit_commander

def shutdown():
    """When node is shut down, remove the previous added objects from the planning scene."""
    scene.remove_world_object()
    scene.remove_attached_object("tool0", "welding_gun")


def initialize_planning_obstacles():
    """Fill this method for adding obstacles"""

    # Configure pose of obstacle
    box_pose = PoseStamped()
    
    # add it to the planning scene
    #scene.add_XYZ()



def initialize_nozzle():
    """Fill this method for adding the nozzle to the planning scene.
    It's important, otherwise the nozzle can collide with the robot/ the environment."""

    # path to nozzle
    rospack = rospkg.RosPack()
    nozzle_path = rospack.get_path("whats_the_name_of_this_package?") + "/meshes/" + "nozzle"

    # pose of nozzle relative to frame tool0
    nozzle_pose = PoseStamped()
    nozzle_pose.header.frame_id = "tool0"
    nozzle_pose.pose.orientation = ...
    nozzle_pose.pose.position = ...

    #scene.attach_XYZ()



if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    scene = ...

    rospy.init_node("my_awesome_node_name")
    
    initialize_planning_obstacles()
    initialize_nozzle()

    rospy.on_shutdown(shutdown)
    
    while not rospy.is_shutdown():
        rospy.rostime.wallsleep(0.5)





    
    
