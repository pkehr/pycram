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

# Room Look Around Poses
kitchen_pose = Pose([8.81, 1.08, 0])
sub_office_kitchen_pose = Pose([6.71,0.22,0])
office_pose = Pose([3.65, 0.87, 0])
living_room_pose = Pose([5.81, 2.69, 0])
sub_kitchen_living_room_pose = Pose([7.6, 2.83, 0])
bedroom_pose = Pose([2.42, 1.96, 0])
sub_living_room_bedroom_pose = Pose([4.34, 2.65, 0])

# Lol variables
notFound = False
human_pose = None
humanInRoom = False
cooperating = False

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
        notFound = False
        MoveJointsMotion(["head_pan_joint"], [x]).perform()
        try:
            human_pose = DetectAction(technique='human', state='start').resolve().perform()
        except pycram.failures.PerceptionObjectNotFound:
            print("oh no, no waving human was found")
        if human_pose : # TODO: Einfügen, dass breaked wird wenn human in wrong room
            break

        x += increase
        if x == 1:
            notFound = True


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

def confirm_offender_follows():
    global callback, timeout, cooperating
    HeadFollowMotion(state='start').perform()
    rospy.sleep(2)
    TalkingMotion("Confirm that you will follow me, after my display changes").perform()
    rospy.sleep(2)
    rospy.loginfo("nlp start")
    nlp_pub.publish("start listening")
    rospy.sleep(2)
    image_switch_publisher.pub_now(ImageEnum.TALK.value)

    start_time = time.time()
    while not callback:
        rospy.sleep(1)
        if int(time.time()) - start_time == timeout:
            rospy.logwarn("Guest needs to repeat")
            image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
    callback = False
    if response[0] == "<CONFIRM>":
        HeadFollowMotion(state='stop').perform()
        TalkingMotion("Thank you, please follow me now").perform()
        rospy.sleep(2)
        cooperating =  True
    elif response[0] == "<DENY>":
        cooperating = False
    else:
        tries = 0
        while tries <= 2:
            rospy.sleep(2.3)

            nlp_pub.publish("start")
            image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

            start_time_rep = time.time()
            while not callback:
                rospy.sleep(1)
                if int(time.time() - start_time_rep) == timeout:
                    rospy.logwarn("guest needs to repeat")
                    image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                    rospy.sleep(2)
            callback = False
            if response[0] == "<CONFIRM>":
                HeadFollowMotion(state='stop').perform()
                cooperating=  True
            elif response[0] == "<DENY>":
                cooperating= False
            else:
                tries += 1

def lead_offender_to(goal: Pose):
    move.pub_now(navpose=goal).perform()

def talk_to_offender(forbiddenRoom:str, triesToCorrect:int):
    if triesToCorrect <= 1:
        TalkingMotion(f"Dear human, you are in {forbiddenRoom}, which is the forbidden room. Please leave immediately ").perform()
        rospy.sleep(2)
    else:
        TalkingMotion(f"Please leave {forbiddenRoom}").perform()
        rospy.sleep(2)
    TalkingMotion("Please follow me to the other guests. ").perform()
    rospy.sleep(2)
    confirm_offender_follows()
    if cooperating:
        closest_pose = find_nearest_room_to_offender(robot.get_pose)
        lead_offender_to(closest_pose)
    rospy.sleep(2)


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
            talk_to_offender(forbiddenRoom, tries)
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
    send_and_process_query()
    rospy.sleep(9)
    global sub_nlp
    human_pose_sub = rospy.Subscriber("/human_pose", PointStamped, human_cb)
    sub_nlp = rospy.Subscriber("nlp_out", String, data_cb)
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
         # Office Space
            move.pub_now(navpose=office_pose)
            giskardpy.turning_around()

            # look around
            search_for_person_in_forbidden_room(robot.get_pose(), 1, "bedroom")



        if step <= 1:
            # Kitchen
            move.pub_now(navpose=sub_office_kitchen_pose)
            rospy.sleep(1)
            move.pub_now(navpose=kitchen_pose)
            giskardpy.turning_around()

            search_for_person_in_forbidden_room(robot.get_pose(), 2, "bedroom")



        if step <= 2:
            # Living Room
            move.pub_now(navpose=sub_kitchen_living_room_pose)
            rospy.sleep(1)
            move.pub_now(navpose=living_room_pose)
            giskardpy.turning_around()

            search_for_person_in_forbidden_room(robot.get_pose(), 3, "bedroom")


        if step <= 3:
            #bedroom
            move.pub_now(navpose=sub_living_room_bedroom_pose)
            rospy.sleep(1)
            move.pub_now(navpose=bedroom_pose)
            giskardpy.turning_around()


            search_for_person_in_forbidden_room(robot.get_pose(), 0, "bedroom")



demo(0)





