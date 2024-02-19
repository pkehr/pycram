import rospy
from pycram.designators.action_designator import DetectAction, NavigateAction
from pycram.designators.motion_designator import TalkingMotion
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from demos.pycram_receptionist_demo.utils.misc import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from std_msgs.msg import String, Bool
from demos.pycram_receptionist_demo.deprecated import talk_actions
from pycram.helper import axis_angle_to_quaternion

world = BulletWorld("DIRECT")
# /pycram/viz_marker topic bei Marker Array
v = VizMarkerPublisher()

world.set_gravity([0, 0, -9.8])
robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])
kitchen = Object("kitchen", "environment", "kitchen.urdf")
robot.set_joint_state(robot_description.torso_joint, 0.24)
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
milk = Object("Milkpack", "milk", "milk.stl", pose=Pose([-2.7, 2.3, 0.43]), color=[1, 0, 0, 1])

giskardpy.init_giskard_interface()
# giskardpy.sync_worlds()
# RobotStateUpdater("/tf", "/joint_states")




with real_robot:
    # Variables
    robot_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
    pose_kitchen_to_couch = Pose([4.2, 3, 0], robot_orientation)
    robot_orientation_couch = axis_angle_to_quaternion([0, 0, 1], 0)
    pose_couch = Pose([3, 5, 0], robot_orientation_couch)

    # Perception
    DetectAction(technique='human', state='start').resolve().perform()
    rospy.sleep(2)
    rospy.loginfo("human detected")

    # NLP
    pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)
    TalkingMotion("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?").resolve().perform()
    rospy.sleep(2)
    # signal to start listening
    pub_nlp.publish("start listening")

    # Manipulation
    # keep looking at detected human
    giskardpy.move_head_to_human()

    # failure handling
    rospy.Subscriber("nlp_feedback", Bool, talk_error)

    # receives name and drink via topic
    # TODO: test if NLP changes somthing or if this still works
    rospy.Subscriber("nlp_out", String, talk_request)

    TalkingMotion("please follow me into the living room").resolve.perform()

    # Lead human to living room after introduction
    giskardpy.stop_looking()

    NavigateAction([pose_kitchen_to_couch]).resolve().perform()
    NavigateAction([pose_couch]).resolve().perform()

    TalkingMotion("take a seat, it was nice meeting you")

    rospy.loginfo("End of Demo")
