#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Point, PointStamped, Pose, Quaternion
import sys
import math
import tf_conversions
import rospkg


import moveit_commander

def shutdown():
    """When node is shut down, remove the previous added objects from the planning scene."""
    scene.remove_world_object()
    scene.remove_attached_object("tool0", "welding_gun")


def initialize_planning_obstacles(scene):
    """Fill this method for adding obstacles"""

    # Configure pose of obstacle
    box_pose = PoseStamped()
    
    # add it to the planning scene
    #scene.add_XYZ()
    
    # Add ground plane obstacle
    ground_plane_pose = PoseStamped()
    ground_plane_pose.header.frame_id = "base_link"  
    
    
    ground_plane_pose.pose.position.x = 0.0
    ground_plane_pose.pose.position.y = 0.0
    ground_plane_pose.pose.position.z = -0.02
    
    ground_plane_pose.pose.orientation.w = 1.0
    
    scene.add_box("ground_plane", ground_plane_pose, size=(0.6, 0.6, 0.03))
    
    ground_plane1_pose = PoseStamped()
    ground_plane1_pose.header.frame_id = "base_link"  
    
    
    ground_plane1_pose.pose.position.x = -0.45
    ground_plane1_pose.pose.position.y = 0.0
    ground_plane1_pose.pose.position.z = -0.02 
    
    ground_plane1_pose.pose.orientation.w = 1.0
    
    scene.add_box("ground_plane1", ground_plane1_pose, size=(2.0, 1.5, 0.007))
    
    
    
    
    camera_box_pose = PoseStamped()
    camera_box_pose.header.frame_id = "base_link"  
    
    
    camera_box_pose.pose.position.x = -0.6
    camera_box_pose.pose.position.y = 0.4
    camera_box_pose.pose.position.z = 0.05 
    
    camera_box_pose.pose.orientation.w = 1.0
    camera_box_pose.pose.orientation.z = 1.0
    
   
    
    scene.add_box("camera_box", camera_box_pose, size=(0.1, 0.5, 1.2))

    
    camera_stand_pose = PoseStamped()
    camera_stand_pose.header.frame_id = "base_link"  
    
    
    camera_stand_pose.pose.position.x = -0.6
    camera_stand_pose.pose.position.y = 0.2
    camera_stand_pose.pose.position.z = 0.5
    
    camera_stand_pose.pose.orientation.w = 1.0
    
    
    scene.add_box("camera_stand_box", camera_stand_pose, size=(0.1, 0.6, 0.1))

    camera_pose = PoseStamped()
    camera_pose.header.frame_id = "base_link"  
    
    
    camera_pose.pose.position.x = -0.52
    camera_pose.pose.position.y = 0.05
    camera_pose.pose.position.z = 0.5
    
    camera_pose.pose.orientation.w = 1.0
   
    
    scene.add_box("camera", camera_pose, size=(0.08, 0.18, 0.07))
    
    

def initialize_nozzle():
    """Fill this method for adding the nozzle to the planning scene.
    It's important, otherwise the nozzle can collide with the robot/ the environment."""

    # path to nozzle
    rospack = rospkg.RosPack()
    nozzle_path = rospack.get_path("robot_environment") + "/meshes/" + "welding_nozzle.stl"

    # pose of nozzle relative to frame tool0
    nozzle_pose = PoseStamped()
    nozzle_pose.header.frame_id = "tool0"
    
    
    nozzle_pose.pose.position.x = 0
    nozzle_pose.pose.position.y = 0
    nozzle_pose.pose.position.z = 0.001
    
    q_update = tf_conversions.transformations.quaternion_from_euler(1.57,0,3.14)
    q = Quaternion(*q_update) #4 values

    nozzle_pose.pose.orientation = q

    #scene.attach_XYZ()
    
    scene.attach_mesh(
    name="welding_gun",
    pose=nozzle_pose,
    filename=nozzle_path,
    link="tool0",
    size=(0.01,0.01,0.01)
    )



if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    scene = moveit_commander.PlanningSceneInterface()

    rospy.init_node("obstacle_node")
    
    initialize_planning_obstacles(scene)
    initialize_nozzle()

    rospy.on_shutdown(shutdown)
    
    while not rospy.is_shutdown():
        rospy.rostime.wallsleep(0.5)





    
    
