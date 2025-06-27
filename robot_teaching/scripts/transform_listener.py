#!/usr/bin/env python3

import rospy
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import tf_conversions
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
import actionlib
import math
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

pose = None	
tfbuffer = None
listener = None
       
def transform_marker(msg):
    global tfbuffer
    
    target_frame = "base_link"
    '''
    try:
        trans = tfbuffer.can_transform(target_frame, msg.header.frame_id, rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
        rospy.logwarn(f"Transsform lookup failed: {e}")
        #return
    '''    
    pose = msg.markers[0].pose
    pose.header.frame_id = "usb_cam"
    pose.header.stamp = rospy.Time(0)
    
    try:
        pose_transformed = tfbuffer.transform(pose, target_frame, rospy.Duration(1))
        rospy.loginfo(f"Transformed pose in {target_frame} : {pose_transformed}")
    except Exception as e:
        print(e)


def main():

    global pose_list, pose, tfbuffer, listener
    rospy.init_node("transform_node")
    
    tfbuffer = tf2_ros.Buffer(rospy.Duration(2.0))
    listener = tf2_ros.TransformListener(tfbuffer)
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, transform_marker)
  
    rospy.spin()
     
if __name__ == "__main__":

    main()

    
    

