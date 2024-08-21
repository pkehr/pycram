from math import sin, cos, pi

import rospy
from geometry_msgs.msg import Quaternion, PoseStamped

from pycram.external_interfaces.move_base import query_pose_nav


def rotate(angle):
    """
    Returns the quaternion that is rotated around the z-axis by given degree.
    Turn direction is right.
    """
    radians = (angle * pi) / 180

    x, y = 0, 0
    z = sin(-radians / 2)
    w = cos(-radians / 2)
    return Quaternion(x, y, z, w)


def send_move_base(pose: PoseStamped, tf_buffer, use_sleep=True):
    if pose.header.frame_id != 'map':
        pose: PoseStamped = tf_buffer.transform(pose, 'map')

    pose.header.stamp = rospy.Time.now()
    query_pose_nav(pose)

    if use_sleep:
        rospy.sleep(1)
