#! /usr/bin/env python3

import rospy
import tf2_ros
import time
import tf_conversions
import tf2_geometry_msgs


from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped


def create_pose_stamped() -> PoseStamped:
    pose = Pose()

    pose.position.x = 0.1
    pose.position.y = 0.1
    pose.position.z = 0.1

    q_from_euler = tf_conversions.transformations.quaternion_from_euler(3.1415, 0, 0)
    q = Quaternion(*q_from_euler)

    pose.orientation = q

    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = "map"
    pose_stamped.header.stamp = rospy.Time(0)
    
    return pose_stamped


def main():
    ...
    pose_stamped = create_pose_stamped()

    print("=== current pose_stamped:")
    print(pose_stamped)
    print("---"*4)

    time.sleep(1.0)


    pose_stamped_transform = tf_buffer.transform(pose_stamped, "base_link",rospy.Duration(0.1))
    print("=== transformed pose_stamped:")
    print(pose_stamped_transform)
    print("---"*4)


if __name__ == "__main__":
    rospy.init_node("pose_transform_node")
    tf_buffer = tf2_ros.Buffer(rospy.Duration(5))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    time.sleep(1.0)
    main()
