from math import sin, cos

import rospy
import tf2_ros
from geometry_msgs.msg import Vector3, Quaternion, PoseStamped
from move_base_msgs.msg import MoveBaseGoal
from tf2_geometry_msgs import tf2_geometry_msgs

from demos.utils.enums import ENVIRONMENTS
from demos.utils.object_spawner import set_environment
from pycram.datastructures.enums import WorldMode
from pycram.external_interfaces.move_base import create_nav_action_client, query_pose_nav
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld

mode = WorldMode.DIRECT

world = BulletWorld(mode)
viz = VizMarkerPublisher() if mode == WorldMode.DIRECT else None

current_environment = set_environment(ENVIRONMENTS.SUTURO)
