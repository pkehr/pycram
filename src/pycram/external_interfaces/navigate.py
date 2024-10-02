import math

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal

from pycram.fluent import Fluent


import pycram.external_interfaces.giskard as giskardpy


class PoseNavigator():
    def __init__(self, namespace: str=None):
        self.move_base_name = ""

        if namespace is None:
            self.move_base_name = '/move_base/move'
            self.initial_pose_name = "/initialpose"
            self.amcl_pose_name = '/amcl_pose'
        elif namespace == 'turtle':
            self.move_base_name = '/turtle/move_base/move'
            self.initial_pose_name = "/turtle/initialpose"
            self.amcl_pose_name = '/turtle/amcl_pose'

        rospy.loginfo("move_base init")
        global move_client

        self.client = actionlib.SimpleActionClient(self.move_base_name, MoveBaseAction)
        rospy.loginfo("Waiting for move_base ActionServer at: " + self.move_base_name)
        if self.client.wait_for_server():
            rospy.loginfo("Done")
        #self.pub = rospy.Publisher('goal', PoseStamped, queue_size=10, latch=True)
        self.toya_pose = None
        self.goal_pose = None
        self.pose_pub = rospy.Publisher(self.initial_pose_name, PoseWithCovarianceStamped, queue_size=100)

        self.pose_sub = rospy.Subscriber(self.amcl_pose_name, PoseWithCovarianceStamped, self.toya_pose_cb)
        rospy.loginfo("move_base init construct done")

    def pub_fake_pose(self, fake_pose: PoseStamped):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose.position = fake_pose.pose.position
        msg.pose.pose.orientation = fake_pose.pose.orientation
        msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
        self.pose_pub.publish(msg)

    def toya_pose_cb(self, msg):
        self.toya_pose = msg.pose.pose.position
        rospy.sleep(0.1)

    def interrupt(self):
        print("interrupting hehe")
        self.client.cancel_all_goals()

    def pub_now(self, navpose: PoseStamped, interrupt_bool: bool = True) -> bool:
        self.goal_pose = navpose
        goal = MoveBaseGoal()
        goal.target_pose.header.seq = 0
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose = navpose.pose

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            return False

        rospy.loginfo(f"Publishing navigation pose")
        rospy.loginfo("Waiting for subscribers to connect...")
        self.client.send_goal(goal)

        while not rospy.is_shutdown():
            near_goal = False
            rospy.loginfo("Pose was published")
            if self.toya_pose is not None:
                dis = math.sqrt((self.goal_pose.pose.position.x - self.toya_pose.x) ** 2 +
                                (self.goal_pose.pose.position.y - self.toya_pose.y) ** 2)
                rospy.loginfo("Distance to goal: " + str(dis))
                if dis < 0.04 and interrupt_bool:
                    rospy.logwarn("Near Pose")
                    self.interrupt()
                    return True
                else:
                    self.client.wait_for_result()
                    rospy.logerr("robot needs more time")
                    return True
            else:
                rospy.logerr("something is wrong with navigation")
                return False


