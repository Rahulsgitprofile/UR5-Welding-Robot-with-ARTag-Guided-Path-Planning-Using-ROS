#!/usr/bin/env python3

import rospy
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import tf_conversions
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
import actionlib
import math

from welding_robot_msgs.msg import WeldingPathAction 
from welding_robot_msgs.msg import WeldingPathActionFeedback
from welding_robot_msgs.msg import WeldingPathActionGoal
from welding_robot_msgs.msg import WeldingPathActionResult
from welding_robot_msgs.msg import WeldingPathFeedback
from welding_robot_msgs.msg import WeldingPathGoal
from welding_robot_msgs.msg import WeldingPathResult

pose = None	
pose_list = []

client = actionlib.SimpleActionClient("robot_mover_action_server", WeldingPathAction)

def ar_tag_estimate_callback(msg):
    """Callback from topic subscriber."""
    
    global pose
    
    markers = msg.markers
   
    try:
        pose = msg.markers[0].pose

    except:
        print("AR not found")
       
def transform_marker(msg):
    tfbuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfbuffer)
    
    target_frame = "base_link"
    
    try:
        trans = tfbuffer.lookup_transform(target_frame, msg.header.frame_id, rospy.Time(0))
    except: (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
        rospy.logwarn(f"Transsform lookup failed: {e}")
        return
        
    pose_transform = tf2_geometry_msgs.do_transform_pose(msg, trans)
    
    rospy.loginfo(f"Transformed pose in {target_frame} : {pose_transform}")


def main():

    global pose_list, pose
    #rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_tag_estimate_callback)

    while True:
        n = input("press e to exit & s to save pos : ")
        if n == 's':
            p = pose
            
            p.pose.position.z = 0.54
            p.pose.orientation.x = 0
            p.pose.orientation.y = 0
            p.pose.orientation.z = 0
            p.pose.orientation.w = 1
            p.header.frame_id = "usb_cam"
            
            
            transform_marker(p)
            
            pose_list.append(p)
            print(p)
        elif n == 'e':
            break
            
def test_client():
    client = actionlib.SimpleActionClient("robot_mover_action_server", WeldingPathAction)
    rospy.loginfo("waiting for server...")
    client.wait_for_server()
    
    goal = WeldingPathGoal(poses=pose_list)
    client.send_goal(goal)
    
    rospy.loginfo("Goal sent, waiting for result...")
    client.wait_for_result()
    result = client.get_result()
    
    if result is not None:
        rospy.loginfo("Result Received")
    else:
        rospy.loginfo("Result not received")
   
    
    
if __name__ == "__main__":
    rospy.init_node("teaching_node")

    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_tag_estimate_callback)
    main()
    test_client()
    
    

