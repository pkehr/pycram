import math
import threading
import time
from collections import OrderedDict
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv_bridge
from robokudo_msgs.msg import QueryGoal, QueryAction

import pycram.external_interfaces.giskard as giskardpy

import pycram
import tf.transformations
from tf.transformations import *
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry

# from pycram.demos.pycram_restaurant_demo.restaurant import human_pose
from pycram.designators.motion_designator import *
from demos.pycram_hsrb_real_test_demos.utils.startup import startup
from pycram.datastructures.enums import Arms, ImageEnum
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import ParkArmsAction, DetectAction, LookAtAction, MoveTorsoAction, \
    NavigateAction
from pycram.designators.motion_designator import TalkingMotion, MoveJointsMotion
from pycram.designators.object_designator import CustomerDescription
from pycram.external_interfaces.robokudo import get_used_annotator_list
from pycram.failures import HumanNotFoundCondition
from pycram.language import Code, Monitor
from pycram.process_module import real_robot
from pycram.robot_description import RobotDescription
from pycram.failures import SensorMonitoringCondition, HumanNotFoundCondition
from pycram.ros.action_lib import create_action_client
from pycram.ros_utils.force_torque_sensor import ForceTorqueSensor

from pycram.utilities.robocup_utils import pakerino, TextToImagePublisher, ImageSendPublisher
tf_listener, marker, world, v, text_to_speech_publisher, image_switch_publisher, move, robot, kitchen = startup()

fts = ForceTorqueSensor(robot_name='hsrb')

#start demo point
start_pose = Pose([6.26, 3.07, 0])

# Room Look Around Poses
kitchen_pose = Pose([8.81, 1.08, 0])
kitchen_pose_2 = Pose([8.49, 1.3, 0], [0, 0, 0, 1])
kitchen_pose_3 = Pose([7.79, -0.11, 0], [0, 0, 0, -1])
sub_office_kitchen_pose = Pose([6.71,0.22,0])
office_pose = Pose([3.65, 0.87, 0])
office_pose_1 = Pose([3.99, 0.51, 0], [0, 0, 0, 1])
office_pose_2 = Pose([2.25, 0.39, 0], [0, 0, 0, -1])
living_room_pose = Pose([5.81, 2.69, 0])
living_room_middle_pose = Pose([7.02, 3.21,0])
sub_kitchen_living_room_pose = Pose([7.6, 2.83, 0])
bedroom_pose = Pose([2.42, 1.96, 0])
bedroom_door_pose = Pose([2.67, 3.01,0])
bedroom_middle_pose = Pose([2.55, 4.41])
sub_living_room_bedroom_pose = Pose([4.34, 2.65, 0])

# Lol variables
notFound = False
human_pose = None
humanInRoom = False
cooperating = False
foundObj = []
litteringHuman = None
overallTries = 0
rkclient = create_action_client('robokudo/query', QueryAction)
rospy.loginfo("Waiting for action server")
rkclient.wait_for_server()
rospy.loginfo("You can start your demo now")

nlp_pub = rospy.Publisher('/startListener', String, queue_size=16)

global sub_nlp
global human_pose_sub


response = [""]
callback = False
timeout = 10

def find_nearest_room_to_offender(robotPose: Pose)-> Pose:
    tmpPose = np.array([robotPose.position.x, robotPose.position.y, robotPose.position.z])

    candidate_poses =[
        np.array([kitchen_pose.position.x, kitchen_pose.position.y, kitchen_pose.position.z]),
        np.array([office_pose.position.x, office_pose.position.y, office_pose.position.z]),
        np.array([living_room_pose.position.x, living_room_pose.position.y, living_room_pose.position.z]),
        np.array([bedroom_pose.position.x, bedroom_pose.position.y, bedroom_pose.position.z]),
        np.array([sub_living_room_bedroom_pose.position.x, sub_living_room_bedroom_pose.position.y,sub_living_room_bedroom_pose.position.z]),
        np.array([sub_office_kitchen_pose.position.x, sub_office_kitchen_pose.position.y, sub_office_kitchen_pose.position.z]),
        np.array([sub_kitchen_living_room_pose.position.x, sub_kitchen_living_room_pose.position.y, sub_kitchen_living_room_pose.position.z])
    ]

    distances = [np.linalg.norm(tmpPose - pose) for pose in candidate_poses]
    nearest_index = np.argmin(distances)

    nearest_pose = candidate_poses[nearest_index]
    return nearest_pose


def data_cb(self, data):
    """
    function to receive data from nlp via /nlp_out topic
    """
    global response
    response = data.data.split(",")
    for ele in self.response:
        ele.strip()
    response.append("None")
    print(response)
    self.callback = True

def look_down_around(increase:float, start_pose:PoseStamped, _tilt:float):
    MoveJointsMotion(["head_tilt_joint"], [_tilt]).perform()
    global foundObj
    foundObj = None
    x = -0.5
    while x <=1:
        MoveJointsMotion(["head_oan_joint"], [x]).perform()
        try:
            foundObj = DetectAction(technique='all').resolve().perform()
        except pycram.failures.PerceptionObjectNotFound:
            foundObj = []
        if len(foundObj) > 0:
            break
        x += increase
def look_around(increase: float, star_pose: PoseStamped):
    """
    Function to make Toya look continuous from left to right. It stops if Toya perceives a human.
    :param: increase: The increments in which Toya should look around.
    """

    global human_pose
    human_pose = None
    tmp_x = star_pose.pose.position.x
    tmp_y = star_pose.pose.position.y
    tmp_z = star_pose.pose.position.z
    x = -0.5
    while x <= 1:
        #notFound = False
        MoveJointsMotion(["head_pan_joint"], [x]).perform()
        try:
            human_pose = DetectAction(technique='human_forbidden', state='start').resolve().perform()
        except pycram.failures.PerceptionObjectNotFound:
            print("oh no, no waving human was found")
        if human_pose : # TODO: Einfügen, dass breaked wird wenn human in wrong room
            break

        x += increase



def change_orientation_given_angle(startPose: Pose, angle: int) -> Pose:
    q_orig = [startPose.orientation.x, startPose.orientation.y, startPose.orientation.z, startPose.orientation.w]

    theta = np.radians(angle)
    q_rot = quaternion_from_euler(0, 0, theta)
    q_new = quaternion_multiply(q_rot, q_orig)

    changedPose = Pose([startPose.pose.position.x, startPose.pose.position.y, startPose.pose.position.z], [q_new.x, q_new.y, q_new.z, q_new.w])
    return changedPose

def monitor_func():
    """
    monitors force torque sensor of robot and throws
    Condition if a significant force is detected (e.g. the gripper is pushed down)
    """
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        rospy.logwarn("sensor exception")
        return SensorMonitoringCondition

    return False

def human_cb( HumanPoseMsg):
    """
    Callback function for human_pose Subscriber.
    Sets the attribute human_pose when someone (e.g. Perception/Robokudo) publishes on the topic.
    :param HumanPoseMsg: received message
    """
    global humanInRoom
    humanInRoom = True

def transform_camera_to_x(pose, frame_x):
    """
    transforms the pose with given frame_x, orientation will be head ori and z is minus 1.3
    """
    pose.pose.position.z -= 1.3

    pose.header.frame_id = "hsrb/" + frame_x
    tPm = tf_listener.transform_pose(pose=pose, target_frame="/map")
    tPm.pose.position.z = 0
    pan_pose = robot.get_link_pose("head_pan_link")
    pan_pose.header.frame_id = "/map"
    tPm.pose.orientation = pan_pose.pose.orientation

    return tPm

def detect_littering_offender(start: Pose):
    global litteringHuman
    litteringHuman = None
    x = -0.5
    while x <= 1:
        MoveJointsMotion(["head_pan_joint"], [x]).perform()
        try:
            litteringHuman = DetectAction(technique='human', state='start').resolve().perform()
        except pycram.failures.PerceptionObjectNotFound:
            print("oh no, no  human was found")
        if litteringHuman : # TODO: Einfügen, dass breaked wird wenn human in wrong room
            break
        x += 0.5

def talk_littering_offender():
    TalkingMotion("Please pick up your trash because you broke the No Littering rule.").perform()
    rospy.sleep(2)

def lead_offender_to():
    move.pub_now(navpose=bedroom_door_pose)
    rospy.sleep(2)
    move.pub_now(navpose=living_room_middle_pose)
    TalkingMotion("We arrived. Please have fun at the party").perform()
    rospy.sleep(2)

def talk_to_offender_forbidden():
    TalkingMotion("Please follow me, you broke the forbidden room rule.").perform()
    rospy.sleep(2)
    giskardpy.turning_around()
    rospy.sleep(1)
    lead_offender_to()
    rospy.sleep(2)

# TODO: Check only in forbidden room for human anpassen
def search_for_person_in_forbidden_room(robotPose:Pose, step: int, forbiddenRoom:str):
    global notFound, human_pose, humanInRoom
    look_around(0.5, robotPose)
    if notFound:
        move_left = change_orientation_given_angle(robotPose, 45)
        NavigateAction([move_left]).resolve().perform()
        notFound = False
        look_around(0.5, robotPose)
        if notFound:
            # move now to next room
            demo(step)
    elif human_pose is not None:
        drive_pose = transform_camera_to_x(human_pose, "head_rgbd_sensor_link")
        marker.publish(Pose.from_pose_stamped(drive_pose), color=[1, 1, 0, 1], name="offender_pose")
        move.pub_now(navpose=drive_pose)
        tries = 0
        while humanInRoom and tries <= 2:
            talk_to_offender_forbidden(forbiddenRoom, tries)
            tries += 1


def send_and_process_query():
    goal_msg = QueryGoal()
    goal_msg.obj.location = "bedroom"
    x = rkclient.send_goal(goal_msg)
    print(type(x))
    print(x)
    y = x.res[0].pose
    print(y)


def demo(step: int):
    global human_pose, litteringHuman, overallTries
    global notFound, humanInRoom
    with real_robot:

        TalkingMotion("start stickler for the rules demo").perform()
        rospy.sleep(2)
        TalkingMotion("Please push down my gripper to start the demo ").perform()
        image_switch_publisher.pub_now(ImageEnum.PUSHBUTTONS.value)

        try:
            plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
            plan.perform()
        except SensorMonitoringCondition:
            image_switch_publisher.pub_now(ImageEnum.HI.value)

        if step <= 0:
            rospy.sleep(2)
            move.pub_now(navpose=start_pose)
            rospy.sleep(1)
            move.pub_now(navpose=bedroom_door_pose)
            look_around(0.5, robot.get_pose())
            if human_pose is not None:
                drive_pose = transform_camera_to_x(human_pose, "head_rgbd_sensor_link")
                move.pub_now(navpose=drive_pose)
                rospy.sleep(2)
                talk_to_offender_forbidden()
            elif human_pose is None:
                move.pub_now(navpose=bedroom_middle_pose)
                rospy.sleep(1)
                look_around(0.5, robot.get_pose())
                if human_pose is not None:
                    drive_pose = transform_camera_to_x(human_pose, "head_rgbd_sensor_link")
                    move.pub_now(navpose=drive_pose)
                    rospy.sleep(2)
                    talk_to_offender_forbidden()
        if step <= 1:
            move.pub_now(navpose=bedroom_door_pose)
            rospy.sleep(1)
            move.pub_now(navpose=living_room_middle_pose)
            change_Pose_living= change_orientation_given_angle(robot.get_pose(), 45)
            NavigateAction([change_Pose_living]).resolve().perform()
            look_down_around(0.5, robot.get_pose(), -0.2)
            if len(foundObj) != 0:
                detect_littering_offender(robot.get_pose())
                if litteringHuman is not None:
                    drive_pose = transform_camera_to_x(litteringHuman, "head_rgbd_sensor_link")
                    move.pub_now(navpose=drive_pose)
                    talk_to_offender_forbidden()
                else:
                    TalkingMotion("I can not find the offender of the broken No Littering rule").perform()
                    rospy.sleep(2)




        if step <= 2:
            rospy.sleep(1)
            move.pub_now(navpose=kitchen_pose_2)
            if len(foundObj) != 0:
                detect_littering_offender(robot.get_pose())
                if litteringHuman is not None:
                    drive_pose = transform_camera_to_x(litteringHuman, "head_rgbd_sensor_link")
                    move.pub_now(navpose=drive_pose)
                    talk_to_offender_forbidden()
                else:
                    TalkingMotion("I can not find the offender of the broken No Littering rule").perform()
                    rospy.sleep(2)
            rospy.sleep(1)
            move.pub_now(navpose=kitchen_pose_3)
            if len(foundObj) != 0:
                detect_littering_offender(robot.get_pose())
                if litteringHuman is not None:
                    drive_pose = transform_camera_to_x(litteringHuman, "head_rgbd_sensor_link")
                    move.pub_now(navpose=drive_pose)
                    talk_to_offender_forbidden()
                else:
                    TalkingMotion("I can not find the offender of the broken No Littering rule").perform()
                    rospy.sleep(2)
        if step <= 3:
            rospy.sleep(1)
            move.pub_now(navpose=office_pose_1)
            if len(foundObj) != 0:
                detect_littering_offender(robot.get_pose())
                if litteringHuman is not None:
                    drive_pose = transform_camera_to_x(litteringHuman, "head_rgbd_sensor_link")
                    move.pub_now(navpose=drive_pose)
                    talk_to_offender_forbidden()
                else:
                    TalkingMotion("I can not find the offender of the broken No Littering rule").perform()
                    rospy.sleep(2)
            rospy.sleep(1)
            move.pub_now(navpose=office_pose_2)
            overallTries += 1
            if len(foundObj) != 0:
                detect_littering_offender(robot.get_pose())
                if litteringHuman is not None:
                    drive_pose = transform_camera_to_x(litteringHuman, "head_rgbd_sensor_link")
                    move.pub_now(navpose=drive_pose)
                    talk_to_offender_forbidden()
                else:
                    TalkingMotion("I can not find the offender of the broken No Littering rule").perform()
                    rospy.sleep(2)
                while overallTries > 4:
                    demo(0)









demo(0)





