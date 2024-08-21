import rospy
import tf2_ros
from geometry_msgs.msg import Vector3, PoseStamped
from demos.pycram_turtle_testing.util import rotate, send_move_base
from pycram.external_interfaces.move_base import create_nav_action_client

rate = rospy.Rate(10)
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
client = create_nav_action_client()

starting_orientation = PoseStamped()
starting_orientation.header.frame_id = "base_footprint"
starting_orientation.pose.position = Vector3(0.3, 0.0, 0.0)
orientation_in_map: PoseStamped = tf_buffer.transform(starting_orientation, "map")

pose_1 = PoseStamped()
pose_1.header.frame_id = 'map'
pose_1.pose.position = Vector3(2.0, 3.0, 0.0)
pose_1.pose.orientation = orientation_in_map.pose.orientation

pose_table = PoseStamped()
pose_table.header.frame_id = 'map'
pose_table.pose.position = Vector3(2.0, 5.0, 0.0)

starting_pose = PoseStamped()
starting_pose.header.frame_id = 'map'
starting_pose.pose.position = Vector3(3.5, 2.8, 0.0)

rotated_pose = PoseStamped()
rotated_pose.header.frame_id = "base_footprint"
rotated_pose.pose.orientation = rotate(-90)


def test_demo():
    send_move_base(pose_1, tf_buffer)

    send_move_base(rotated_pose, tf_buffer)

    # Get current rotation
    current_rotation = PoseStamped()
    current_rotation.header.frame_id = "base_footprint"
    c_r: PoseStamped = tf_buffer.transform(current_rotation, "map")
    pose_table.pose.orientation = c_r.pose.orientation
    send_move_base(pose_table, tf_buffer)


if __name__ == '__main__':

    try:
        test_demo()

    except (tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.LookupException,
            rospy.ROSInterruptException):
        pass
