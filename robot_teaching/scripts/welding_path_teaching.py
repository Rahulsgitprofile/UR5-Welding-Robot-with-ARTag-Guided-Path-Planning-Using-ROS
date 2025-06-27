#!/usr/bin/env python3

import rospy
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import tf_conversions
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
import actionlib
import math

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, TransformStamped


from welding_robot_msgs.msg import WeldingPathAction 
from welding_robot_msgs.msg import WeldingPathActionFeedback
from welding_robot_msgs.msg import WeldingPathActionGoal
from welding_robot_msgs.msg import WeldingPathActionResult
from welding_robot_msgs.msg import WeldingPathFeedback
from welding_robot_msgs.msg import WeldingPathGoal
from welding_robot_msgs.msg import WeldingPathResult

tfbuffer = None
listener = None

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
        
'''
def transform_marker(msg):
    global tfbuffer
    
    target_frame = "base_link"
    
    try:
        trans = tfbuffer.can_transform(target_frame, msg.header.frame_id, rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
        rospy.logwarn(f"Transsform lookup failed: {e}")
        #return 
    #pose = msg.markers[0].pose
    #pose.header.frame_id = "usb_cam"
    msg.header.stamp = rospy.Time(0)
    
    try:
        pose_transformed = tfbuffer.transform(msg, target_frame, rospy.Duration(1))
        rospy.loginfo(f"Transformed pose in {target_frame} : {pose_transformed}")
    except Exception as e:
        print(e)
    return pose_transformed
'''
def main():

    global pose_list, pose, tfbuffer, listener
    tfbuffer = tf2_ros.Buffer(rospy.Duration(2.0))
    listener = tf2_ros.TransformListener(tfbuffer)
    tfbroadcast = tf2_ros.TransformBroadcaster()

    #nozzle_transform = tfbuffer.lookup_transform("tool0", "nozzle_link", rospy.Time(0), rospy.Duration(0.5))
    #nozzle_transform.header.frame_id = "ar_marker_13a"
    #nozzle_transform.child_frame_id = "tcp_goal"
  

    while True:

        #tfbroadcast.sendTransform(nozzle_transform)
        n = input("press e to exit & s to save pos : ")
        if n == 's':
        

            print("base coordinates----")
            print(pose)
            #t = tfbuffer.lookup_transform("base_link", "ar_marker_13a", rospy.Time(0), rospy.Duration(0.5))
            t = tfbuffer.lookup_transform("base_link", "tcp_goal", rospy.Time(0), rospy.Duration(0.5))

            pose = PoseStamped()
            pose.header = t.header
            pose.pose.orientation = t.transform.rotation
            pose.pose.position = t.transform.translation  
            pose.pose.position.z += .03
            #pose.pose.orientation.x = 0
            #pose.pose.orientation.y = 0

        

            print("Transformed pose-------")
            print(pose)

            #pose.pose.position.z = pose.pose.position.z -0.098  #nozzle z offset



            #print("Updated pose--------------")
            #print(pose)

            # offset_pose = tf2_geometry_msgs.do_transform_pose(pose, nozzle_transform)
            # print("----")
            # print(offset_pose)

            #nozzle_t = TransformStamped()
            #nozzle_t.transform.translation.z = 0.1
            
            #pose_updated = transform_marker(p)
            
            pose_list.append(pose)
    
        elif n == 'e':
            print(pose_list)
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
    rospy.init_node("teaching_transform_node")

    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_tag_estimate_callback)
    main()
    test_client()
    
    

