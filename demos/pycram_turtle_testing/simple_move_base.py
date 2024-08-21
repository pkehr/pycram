from math import sin, cos

import rospy
import tf2_ros
from geometry_msgs.msg import Vector3, Quaternion, PoseStamped
from move_base_msgs.msg import MoveBaseGoal
from tf2_geometry_msgs import tf2_geometry_msgs

from pycram.external_interfaces.move_base import create_nav_action_client, query_pose_nav

rate = rospy.Rate(10)

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
client = create_nav_action_client()


def transform_pose(input_pose, from_frame, to_frame):
    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time.now()

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise


def rotate(angle):
    x, y = 0, 0
    z = sin(angle / 2)
    w = cos(angle / 2)
    return Quaternion(x, y, z, w)


starting_orientation = PoseStamped()
starting_orientation.header.frame_id = "base_footprint"
starting_orientation.pose.position = Vector3(0.3, 0.0, 0.0)
orientation_in_map: PoseStamped = tf_buffer.transform(starting_orientation, "map")

pose_1 = PoseStamped()
pose_1.header.frame_id = 'map'
pose_1.header.stamp = rospy.Time.now()
pose_1.pose.position = Vector3(2.0, 3.0, 0.0)
pose_1.pose.orientation = orientation_in_map.pose.orientation
# t_pose: PoseStamped = tf_buffer.transform(pose_1, 'map', rospy.Duration(1))

pose_table = PoseStamped()
pose_table.header.frame_id = 'map'
pose_table.pose.position = Vector3(2.0, 5.0, 0.0)
pose_table.pose.orientation = orientation_in_map.pose.orientation  # Quaternion(0, 0, 0.5, 0)
# g_pose: PoseStamped = tf_buffer.transform(pose_2, 'map', rospy.Duration(1))

starting_pose = PoseStamped()
starting_pose.header.frame_id = 'map'
starting_pose.header.stamp = rospy.Time.now()
starting_pose.pose.position = Vector3(3.5, 2.8, 0.0)
starting_pose.pose.orientation = orientation_in_map.pose.orientation
# g_pose: PoseStamped = tf_buffer.transform(pose_2, 'map', rospy.Duration(1))

rotated_pose = PoseStamped()
rotated_pose.header.frame_id = "base_footprint"
rotated_pose.pose.orientation = rotate(-90)


if __name__ == '__main__':

    try:
        print("starting first pose")
        query_pose_nav(pose_1)
        rospy.sleep(1)
        print("starting table pose")
        rotated_in_map: PoseStamped = tf_buffer.transform(rotated_pose, "map")
        rotated_in_map.header.stamp = rospy.Time.now()
        query_pose_nav(rotated_pose)
        rospy.sleep(1)
        print("starting home pose")
        current_rotation = PoseStamped()
        current_rotation.header.frame_id = "base_footprint"
        c_r: PoseStamped = tf_buffer.transform(current_rotation, "map")
        pose_table.pose.orientation = c_r.pose.orientation
        pose_table.header.stamp = rospy.Time.now()
        query_pose_nav(pose_table)


    except (tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.LookupException,
            rospy.ROSInterruptException):
        pass
