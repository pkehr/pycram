import dataclasses
import itertools
import time
import rospy
from typing import List, Optional, Any, Tuple
import numpy as np
import sqlalchemy.orm
import math
from .location_designator import CostmapLocation
from .motion_designator import *
from .object_designator import ObjectDesignatorDescription, BelieveObject, ObjectPart
from ..orm.action_designator import (ParkArmsAction as ORMParkArmsAction, NavigateAction as ORMNavigateAction,
                                     PickUpAction as ORMPickUpAction, PlaceAction as ORMPlaceAction,
                                     MoveTorsoAction as ORMMoveTorsoAction, SetGripperAction as ORMSetGripperAction,
                                     Action as ORMAction)
from ..orm.base import Quaternion, Position, Base, RobotState, MetaData
from ..plan_failures import ObjectUnfetchable, ReachabilityFailure
from ..robot_descriptions import robot_description
from ..task import with_tree
from ..enums import Arms
from ..designator import ActionDesignatorDescription
from ..bullet_world import BulletWorld
from ..pose import Pose, Transform
import pycram.helper as helper
from src.pycram.local_transformer import LocalTransformer


class MoveTorsoAction(ActionDesignatorDescription):
    """
    Action Designator for Moving the torso of the robot up and down
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        """
        Performable Move Torso Action designator.
        """

        position: float
        """
        Target position of the torso joint
        """

        @with_tree
        def perform(self) -> None:
            MoveJointsMotion([robot_description.torso_joint], [self.position]).resolve().perform()

        def to_sql(self):
            return ORMMoveTorsoAction(self.position)

        def insert(self, session: sqlalchemy.orm.session.Session, **kwargs):
            action = super().insert(session)
            session.add(action)
            session.commit()
            return action

    def __init__(self, positions: List[float], resolver=None):
        """
        Create a designator description to move the torso of the robot up and down.

        :param positions: List of possible positions of the robots torso, possible position is a float of height in metres
        :param resolver: An optional resolver that returns a performable designator for a designator description.
        """
        super().__init__(resolver)
        self.positions: List[float] = positions

    def ground(self) -> Action:
        """
        Creates a performable action designator with the first element from the list of possible torso heights.

        :return: A performable action designator
        """
        return self.Action(self.positions[0])

    def __iter__(self):
        """
        Iterates over all possible values for this designator and returns a performable action designator with the value.

        :return: A performable action designator
        """
        for position in self.positions:
            yield self.Action(position)


class SetGripperAction(ActionDesignatorDescription):
    """
    Set the gripper state of the robot
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        gripper: str
        """
        The gripper that should be set 
        """
        motion: str
        """
        The motion that should be set on the gripper
        """

        @with_tree
        def perform(self) -> None:
            MoveGripperMotion(gripper=self.gripper, motion=self.motion).resolve().perform()

        def to_sql(self) -> Base:
            return ORMSetGripperAction(self.gripper, self.motion)

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
            action = super().insert(session)
            session.add(action)
            session.commit()
            return action

    def __init__(self, grippers: List[str], motions: List[str], resolver=None):
        """
        Sets the gripper state, the desired state is given with the motion. Motion can either be 'open' or 'close'.

        :param grippers: A list of possible grippers
        :param motions: A list of possible motions
        :param resolver: An alternative resolver that returns a performable designator for a designator description
        """
        super(SetGripperAction, self).__init__(resolver)
        self.grippers: List[str] = grippers
        self.motions: List[str] = motions

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first element in the grippers and motions list.

        :return: A performable designator
        """
        return self.Action(self.grippers[0], self.motions[0])

    def __iter__(self):
        """
        Iterates over all possible combinations of grippers and motions

        :return: A performable designator with a combination of gripper and motion
        """
        for parameter_combination in itertools.product(self.grippers, self.motions):
            yield self.Action(*parameter_combination)


class ReleaseAction(ActionDesignatorDescription):
    """
    Releases an Object from the robot.

    Note: This action can not be used yet.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        gripper: str
        object_designator: ObjectDesignatorDescription.Object

        @with_tree
        def perform(self) -> Any:
            raise NotImplementedError()

        def to_sql(self) -> Base:
            raise NotImplementedError()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
            raise NotImplementedError()

    def __init__(self, grippers: List[str], object_designator_description: ObjectDesignatorDescription,
                 resolver=None):
        super().__init__(resolver)
        self.grippers: List[str] = grippers
        self.object_designator_description = object_designator_description

    def ground(self) -> Action:
        return self.Action(self.grippers[0], self.object_designator_description.ground())


class GripAction(ActionDesignatorDescription):
    """
    Grip an object with the robot.

    :ivar grippers: The grippers to consider
    :ivar object_designator_description: The description of objects to consider
    :ivar efforts: The efforts to consider

    Note: This action can not be used yet.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        gripper: str
        object_designator: ObjectDesignatorDescription.Object
        effort: float

        @with_tree
        def perform(self) -> Any:
            raise NotImplementedError()

        def to_sql(self) -> Base:
            raise NotImplementedError()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
            raise NotImplementedError()

    def __init__(self, grippers: List[str], object_designator_description: ObjectDesignatorDescription,
                 efforts: List[float], resolver=None):
        super(GripAction, self).__init__(resolver)
        self.grippers: List[str] = grippers
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.efforts: List[float] = efforts

    def ground(self) -> Action:
        return self.Action(self.grippers[0], self.object_designator_description.ground(), self.efforts[0])


class ParkArmsAction(ActionDesignatorDescription):
    """
    Park the arms of the robot.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):

        arm: Arms
        """
        Entry from the enum for which arm should be parked
        """

        @with_tree
        def perform(self) -> None:
            # create the keyword arguments
            kwargs = dict()

            # add park left arm if wanted
            if self.arm in [Arms.LEFT, Arms.BOTH]:
                kwargs["left_arm_config"] = "park"

            # add park right arm if wanted
            if self.arm in [Arms.RIGHT, Arms.BOTH]:
                kwargs["right_arm_config"] = "park"
            MoveArmJointsMotion(**kwargs).resolve().perform()

        def to_sql(self) -> ORMParkArmsAction:
            return ORMParkArmsAction(self.arm.name)

        def insert(self, session: sqlalchemy.orm.session.Session, **kwargs) -> ORMParkArmsAction:
            action = super().insert(session)
            session.add(action)
            session.commit()
            return action

    def __init__(self, arms: List[Arms], resolver=None):
        """
        Moves the arms in the pre-defined parking position. Arms are taken from pycram.enum.Arms

        :param arms: A list of possible arms, that could be used
        :param resolver: An optional resolver that returns a performable designator from the designator description
        """
        super(ParkArmsAction, self).__init__(resolver)
        self.arms: List[Arms] = arms

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first element of the list of possible arms

        :return: A performable designator
        """
        return self.Action(self.arms[0])

class PickUpAction(ActionDesignatorDescription):
    """
    Designator to let the robot pick up an object.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):

        object_designator: ObjectDesignatorDescription.Object
        """
        Object designator describing the object that should be picked up
        """

        arm: str
        """
        The arm that should be used for pick up
        """

        grasp: str
        """
        The grasp that should be used. For example, 'left' or 'right'
        """

        object_at_execution: Optional[ObjectDesignatorDescription.Object] = dataclasses.field(init=False)
        """
        The object at the time this Action got created. It is used to be a static, information holding entity. It is
        not updated when the BulletWorld object is changed.
        """

        @with_tree
        def perform(self) -> None:
            # Store the object's data copy at execution
            self.object_at_execution = self.object_designator.data_copy()
            robot = BulletWorld.robot
            # Retrieve object and robot from designators
            object = self.object_designator.bullet_world_object
            # Get grasp orientation and target pose
            grasp = robot_description.grasps.get_orientation_for_grasp(self.grasp)

            # oTm = Object Pose in Frame map
            oTm = object.get_pose()
            # Transform the object pose to the map frame
            mTo = object.local_transformer.transform_to_object_frame(oTm, object)
            # Adjust the pose according to the special knowledge of the object designator
            adjusted_pose = self.object_designator.special_knowledge_adjustment_pose(self.grasp, mTo)
            # Transform the adjusted pose to the map frame
            adjusted_oTm = object.local_transformer.transform_pose(adjusted_pose, "map")
            # multiplying the orientation therefore "rotating" it, to get the correct orientation of the gripper
            ori = helper.multiply_quaternions([adjusted_oTm.orientation.x, adjusted_oTm.orientation.y,
                                        adjusted_oTm.orientation.z, adjusted_oTm.orientation.w],
                                       grasp)

            # Set the orientation of the object pose by grasp in MAP
            adjusted_oTm.orientation.x = ori[0]
            adjusted_oTm.orientation.y = ori[1]
            adjusted_oTm.orientation.z = ori[2]
            adjusted_oTm.orientation.w = ori[3]

            # prepose depending on the gripper
            gripper_frame = "pr2_1/l_gripper_tool_frame" if self.arm == "left" else "pr2_1/r_gripper_tool_frame"

            # First rotate the gripper, so the further calculations maakes sense
            tmp_for_rotate_pose = object.local_transformer.transform_pose(adjusted_oTm, gripper_frame)
            tmp_for_rotate_pose.pose.position.x = 0
            tmp_for_rotate_pose.pose.position.y = 0
            tmp_for_rotate_pose.pose.position.z = -0.1
            gripper_rotate_pose = object.local_transformer.transform_pose(tmp_for_rotate_pose, "map")

            #Perform Gripper Rotate
            BulletWorld.current_bullet_world.add_vis_axis(gripper_rotate_pose)
            MoveTCPMotion(gripper_rotate_pose, self.arm).resolve().perform()

            oTg = object.local_transformer.transform_pose(adjusted_oTm, gripper_frame)
            oTg.pose.position.x -= 0.1 # in x since this is how the gripper is oriented
            prepose = object.local_transformer.transform_pose(oTg, "map")

            # Perform the motion with the prepose and open gripper
            BulletWorld.current_bullet_world.add_vis_axis(prepose)
            MoveTCPMotion(prepose, self.arm).resolve().perform()
            MoveGripperMotion(motion="open", gripper=self.arm).resolve().perform()

            # Perform the motion with the adjusted pose -> actual grasp and close gripper
            BulletWorld.current_bullet_world.add_vis_axis(adjusted_oTm)
            MoveTCPMotion(adjusted_oTm, self.arm).resolve().perform()
            adjusted_oTm.pose.position.z += 0.03
            MoveGripperMotion(motion="close", gripper=self.arm).resolve().perform()
            tool_frame = robot_description.get_tool_frame(self.arm)
            robot.attach(object, tool_frame)

            # Lift object
            BulletWorld.current_bullet_world.add_vis_axis(adjusted_oTm)
            MoveTCPMotion(adjusted_oTm, self.arm).resolve().perform()

            # Remove the vis axis from the world
            BulletWorld.current_bullet_world.remove_vis_axis()


        def to_sql(self) -> ORMPickUpAction:
            return ORMPickUpAction(self.arm, self.grasp)

        def insert(self, session: sqlalchemy.orm.session.Session, **kwargs):
            action = super().insert(session)
            # try to create the object designator
            if self.object_at_execution:
                od = self.object_at_execution.insert(session, )
                action.object = od.id
            else:
                action.object = None

            session.add(action)
            session.commit()

            return action

    def __init__(self, object_designator_description: ObjectDesignatorDescription, arms: List[str],
                 grasps: List[str], resolver=None):
        """
        Lets the robot pick up an object. The description needs an object designator describing the object that should be
        picked up, an arm that should be used as well as the grasp from which side the object should be picked up.

        :param object_designator_description: List of possible object designator
        :param arms: List of possible arms that could be used
        :param grasps: List of possible grasps for the object
        :param resolver: An optional resolver that returns a performable designator with elements from the lists of possible paramter
        """
        super(PickUpAction, self).__init__(resolver)
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.arms: List[str] = arms
        self.grasps: List[str] = grasps

    def ground(self) -> Action:
        """
        Default resolver, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.ground(), self.arms[0], self.grasps[0])


class PlaceAction(ActionDesignatorDescription):
    """
    Places an Object at a position using an arm.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):

        object_designator: ObjectDesignatorDescription.Object
        """
        Object designator describing the object that should be place
        """
        arm: str
        """
        Arm that is currently holding the object
        """
        target_location: Pose
        """
        Pose in the world at which the object should be placed
        """

        @with_tree
        def perform(self) -> None:
            PlaceMotion(object_desig=self.object_designator, arm=self.arm, target=self.target_location).resolve(). \
                perform()

        def to_sql(self) -> ORMPlaceAction:
            return ORMPlaceAction(self.arm)

        def insert(self, session, *args, **kwargs) -> ORMPlaceAction:
            action = super().insert(session)

            if self.object_designator:
                od = self.object_designator.insert(session, )
                action.object = od.id
            else:
                action.object = None

            if self.target_location:
                position = Position(*self.target_location.position_as_list())
                orientation = Quaternion(*self.target_location.orientation_as_list())
                session.add(position)
                session.add(orientation)
                session.commit()
                action.position = position.id
                action.orientation = orientation.id
            else:
                action.position = None
                action.orientation = None

            session.add(action)
            session.commit()
            return action

    def __init__(self, object_designator_description: ObjectDesignatorDescription,
                 target_locations: List[Pose],
                 arms: List[str], resolver=None):
        """
        Create an Action Description to place an object

        :param object_designator_description: Description of object to place.
        :param target_locations: List of possible positions/orientations to place the object
        :param arms: List of possible arms to use
        :param resolver: Grounding method to resolve this designator
        """
        super(PlaceAction, self).__init__(resolver)
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.target_locations: List[Pose] = target_locations
        self.arms: List[str] = arms

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first entries from the list of possible entries.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.ground(), self.arms[0],
                           self.target_locations[0])


class NavigateAction(ActionDesignatorDescription):
    """
    Navigates the Robot to a position.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        target_location: Pose
        """
        Location to which the robot should be navigated
        """

        @with_tree
        def perform(self) -> None:
            MoveMotion(self.target_location).resolve().perform()

        def to_sql(self) -> ORMNavigateAction:
            return ORMNavigateAction()

        def insert(self, session, *args, **kwargs) -> ORMNavigateAction:
            # initialize position and orientation
            position = Position(*self.target_location.position_as_list())
            orientation = Quaternion(*self.target_location.orientation_as_list())

            # add those to the database and get the primary keys
            session.add(position)
            session.add(orientation)
            session.commit()

            # create the navigate action orm object
            action = super().insert(session)

            # set foreign keys
            action.position = position.id
            action.orientation = orientation.id

            # add it to the db
            session.add(action)
            session.commit()

            return action

    def __init__(self, target_locations: List[Pose], resolver=None):
        """
        Navigates the robot to a location.

        :param target_locations: A list of possible target locations for the navigation.
        :param resolver: An alternative resolver that creates a performable designator from the list of possible parameter
        """
        super(NavigateAction, self).__init__(resolver)
        self.target_locations: List[Pose] = target_locations

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first entry of possible target locations.

        :return: A performable designator
        """
        return self.Action(self.target_locations[0])


class TransportAction(ActionDesignatorDescription):
    """
    Transports an object to a position using an arm
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        object_designator: ObjectDesignatorDescription.Object
        """
        Object designator describing the object that should be transported.
        """
        arm: str
        """
        Arm that should be used
        """
        target_location: Pose
        """
        Target Location to which the object should be transported
        """

        @with_tree
        def perform(self) -> None:
            robot_desig = BelieveObject(names=[robot_description.name])
            ParkArmsAction.Action(Arms.BOTH).perform()
            pickup_loc = CostmapLocation(target=self.object_designator, reachable_for=robot_desig.resolve())
            # Tries to find a pick-up posotion for the robot that uses the given arm
            pickup_pose = None
            for pose in pickup_loc:
                if self.arm in pose.reachable_arms:
                    pickup_pose = pose
                    break
            if not pickup_pose:
                raise ObjectUnfetchable(
                    f"Found no pose for the robot to grasp the object: {self.object_designator} with arm: {self.arm}")

            NavigateAction([pickup_pose.pose]).resolve().perform()
            PickUpAction.Action(self.object_designator, self.arm, "front").perform()
            ParkArmsAction.Action(Arms.BOTH).perform()
            try:
                place_loc = CostmapLocation(target=self.target_location, reachable_for=robot_desig.resolve()).resolve()
            except StopIteration:
                raise ReachabilityFailure(
                    f"No location found from where the robot can reach the target location: {self.target_location}")
            NavigateAction([place_loc.pose]).resolve().perform()
            PlaceAction.Action(self.object_designator, self.arm, self.target_location).perform()
            ParkArmsAction.Action(Arms.BOTH).perform()

        def to_sql(self) -> Base:
            raise NotImplementedError()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
            raise NotImplementedError()

    def __init__(self, object_designator_description: ObjectDesignatorDescription, arms: List[str],
                 target_locations: List[Pose], resolver=None):
        """
        Designator representing a pick and place plan.

        :param object_designator_description: Object designator describing the object that should be transported
        :param arms: A List of possible arms that could be used for transporting
        :param target_locations: A list of possible target locations for the object to be placed
        :param resolver: An alternative resolver that returns a performable designator for the list of possible parameter
        """
        super(TransportAction, self).__init__(resolver)
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.arms: List[str] = arms
        self.target_locations: List[Pose] = target_locations

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.ground(),
                           self.arms[0],
                           self.target_locations[0])


class LookAtAction(ActionDesignatorDescription):
    """
    Lets the robot look at a position.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        target: Pose
        """
        Position at which the robot should look, given as 6D pose
        """

        @with_tree
        def perform(self) -> None:
            LookingMotion(target=self.target).resolve().perform()

        def to_sql(self) -> Base:
            raise NotImplementedError()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
            raise NotImplementedError()

    def __init__(self, targets: List[Pose], resolver=None):
        """
        Moves the head of the robot such that it points towards the given target location.

        :param targets: A list of possible locations to look at
        :param resolver: An alternative resolver that returns a performable designator for a list of possible target locations
        """
        super(LookAtAction, self).__init__(resolver)
        self.targets: List[Pose] = targets

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first entry in the list of possible targets

        :return: A performable designator
        """
        return self.Action(self.targets[0])


class DetectAction(ActionDesignatorDescription):
    """
    Detects an object that fits the object description and returns an object designator describing the object.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        object_designator: ObjectDesignatorDescription.Object
        """
        Object designator loosely describing the object, e.g. only type. 
        """

        @with_tree
        def perform(self) -> ObjectDesignatorDescription.Object:
            return DetectingMotion(object_type=self.object_designator.type).resolve().perform()

        def to_sql(self) -> Base:
            raise NotImplementedError()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
            raise NotImplementedError()

    def __init__(self, object_designator_description: ObjectDesignatorDescription, resolver=None):
        """
        Tries to detect an object in the field of view (FOV) of the robot.

        :param object_designator_description: Object designator describing the object
        :param resolver: An alternative resolver
        """
        super(DetectAction, self).__init__(resolver)
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the resolved object description.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.resolve())


class OpenAction(ActionDesignatorDescription):
    """
    Opens a container like object

    Can currently not be used
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        object_designator: ObjectPart.Object
        """
        Object designator describing the object that should be opened
        """
        arm: str
        """
        Arm that should be used for opening the container
        """

        @with_tree
        def perform(self) -> Any:
            MoveTCPMotion(self.object_designator.part_pose, self.arm).resolve().perform()
            OpeningMotion(self.object_designator, self.arm).resolve().perform()

        def to_sql(self) -> Base:
            raise NotImplementedError()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
            raise NotImplementedError()

    def __init__(self, object_designator_description: ObjectPart, arms: List[str], resolver=None):
        """
        Moves the arm of the robot to open a container.

        :param object_designator_description: Object designator describing the handle that should be used to open
        :param arms: A list of possible arms that should be used
        :param resolver: A alternative resolver that returns a performable designator for the lists of possible parameter.
        """
        super(OpenAction, self).__init__(resolver)
        self.object_designator_description: ObjectPart = object_designator_description
        self.arms: List[str] = arms

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the resolved object description and the first entries
        from the lists of possible parameter.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.resolve(), self.arms[0])


class CloseAction(ActionDesignatorDescription):
    """
    Closes a container like object.

    Can currently not be used
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        object_designator: ObjectPart.Object
        """
        Object designator describing the object that should be closed
        """
        arm: str
        """
        Arm that should be used for closing
        """

        def perform(self) -> Any:
            MoveTCPMotion(self.object_designator.part_pose, self.arm).resolve().perform()
            ClosingMotion(self.object_designator, self.arm).resolve().perform()

        def to_sql(self) -> Base:
            raise NotImplementedError()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
            raise NotImplementedError()

    def __init__(self, object_designator_description: ObjectPart, arms: List[str],
                 resolver=None):
        """
        Attempts to close an open container

        :param object_designator_description: Object designator description of the handle that should be used
        :param arms: A list of possible arms to use
        :param resolver: An alternative resolver that returns a performable designator for the list of possible parameter
        """
        super(CloseAction, self).__init__(resolver)
        self.object_designator_description: ObjectPart = object_designator_description
        self.arms: List[str] = arms

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the resolved object designator and the first entry from
        the list of possible arms.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.resolve(), self.arms[0])


class PourAction(ActionDesignatorDescription):
    """
    Designator to let the robot pour onto an object.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        object_designator: ObjectDesignatorDescription.Object
        """
        Object designator describing the object that should be tilted
        """
        arm: str
        """
        Arm that is currently holding the object
        """
        pouring_location: Pose
        """
        Pose in the world at which the object should be tilted
        """
        revert_location: Pose
        """
        Pose in the world at which the object should be re-tilted
        """
        wait_duration: int
        """
        Wait duration in seconds for which the object should be tilted
        """

        @with_tree
        def perform(self) -> None:
            MoveTCPMotion(target=self.pouring_location, arm=self.arm).resolve(). \
                perform()
            # sleep for some seconds
            time.sleep(self.wait_duration)  # Sleep for wait_duration seconds
            MoveTCPMotion(target=self.revert_location, arm=self.arm).resolve(). \
                perform()
        #
        # def to_sql(self) -> ORMPlaceAction:
        #     return ORMPlaceAction(self.arm)
        #
        # def insert(self, session, *args, **kwargs) -> ORMPlaceAction:
        #     action = super().insert(session)
        #
        #     if self.object_designator:
        #         od = self.object_designator.insert(session, )
        #         action.object = od.id
        #     else:
        #         action.object = None
        #
        #     session.add(action)
        #     session.commit()
        #     return action

    def __init__(self, object_designator_description: ObjectDesignatorDescription,
                 pouring_location: Pose,
                 revert_location: Pose,
                 arms: List[str], wait_duration, resolver=None):
        """
        Create an Action Description to place an object

        :param object_designator_description: Description of object to place.
        :param target_locations: List of possible positions/orientations to place the object
        :param arms: List of possible arms to use
        :param resolver: Grounding method to resolve this designator
        """
        super(PourAction, self).__init__(resolver)
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.pouring_location: Pose = pouring_location
        self.revert_location: Pose = revert_location
        self.arms: List[str] = arms
        self.wait_duration = wait_duration

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first entries from the list of possible entries.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.ground(), self.arms[0],
                           self.pouring_location,
                           self.revert_location,
                           self.wait_duration)

    class HoldAction(ActionDesignatorDescription):
        """
        Designator to let the robot pick up an object.
        """

        @dataclasses.dataclass
        class Action(ActionDesignatorDescription.Action):

            object_designator: ObjectDesignatorDescription.Object
            """
            Object designator describing the object that should be picked up
            """

            arm: str
            """
            The arm that should be used for pick up
            """

            grasp: str
            """
            The grasp that should be used. For example, 'left' or 'right'
            """

            object_at_execution: Optional[ObjectDesignatorDescription.Object] = dataclasses.field(init=False)
            """
            The object at the time this Action got created. It is used to be a static, information holding entity. It is
            not updated when the BulletWorld object is changed.
            """

            @with_tree
            def perform(self) -> None:
                # Store the object's data copy at execution
                self.object_at_execution = self.object_designator.data_copy()
                robot = BulletWorld.robot
                # Retrieve object and robot from designators
                object = self.object_designator.bullet_world_object
                # Get grasp orientation and target pose
                grasp = robot_description.grasps.get_orientation_for_grasp(self.grasp)

                # oTm = Object Pose in Frame map
                oTm = object.get_pose()
                # Transform the object pose to the map frame
                mTo = object.local_transformer.transform_to_object_frame(oTm, object)
                # Adjust the pose according to the special knowledge of the object designator
                adjusted_pose = self.object_designator.special_knowledge_adjustment_pose(self.grasp, mTo)
                # Transform the adjusted pose to the map frame
                adjusted_oTm = object.local_transformer.transform_pose(adjusted_pose, "map")
                # multiplying the orientation therefore "rotating" it, to get the correct orientation of the gripper
                ori = helper.multiply_quaternions([adjusted_oTm.orientation.x, adjusted_oTm.orientation.y,
                                            adjusted_oTm.orientation.z, adjusted_oTm.orientation.w],
                                           grasp)

                # Set the orientation of the object pose by grasp in MAP
                adjusted_oTm.orientation.x = ori[0]
                adjusted_oTm.orientation.y = ori[1]
                adjusted_oTm.orientation.z = ori[2]
                adjusted_oTm.orientation.w = ori[3]

                # prepose depending on the gripper
                gripper_frame = "pr2_1/l_gripper_tool_frame" if self.arm == "left" else "pr2_1/r_gripper_tool_frame"

                # First rotate the gripper, so the further calculations makes sense
                tmp_for_rotate_pose = object.local_transformer.transform_pose(adjusted_oTm, gripper_frame)
                tmp_for_rotate_pose.pose.position.x = 0
                tmp_for_rotate_pose.pose.position.y = 0
                tmp_for_rotate_pose.pose.position.z = -0.1
                gripper_rotate_pose = object.local_transformer.transform_pose(tmp_for_rotate_pose, "map")

                # Perform Gripper Rotate
                BulletWorld.current_bullet_world.add_vis_axis(gripper_rotate_pose)
                MoveTCPMotion(gripper_rotate_pose, self.arm).resolve().perform()

                oTg = object.local_transformer.transform_pose(adjusted_oTm, gripper_frame)
                oTg.pose.position.x -= 0.1  # in x since this is how the gripper is oriented
                prepose = object.local_transformer.transform_pose(oTg, "map")

                # Perform the motion with the prepose and open gripper
                BulletWorld.current_bullet_world.add_vis_axis(prepose)
                MoveTCPMotion(prepose, self.arm).resolve().perform()
                MoveGripperMotion(motion="open", gripper=self.arm).resolve().perform()

                # Perform the motion with the adjusted pose -> actual grasp and close gripper
                BulletWorld.current_bullet_world.add_vis_axis(adjusted_oTm)
                MoveTCPMotion(adjusted_oTm, self.arm).resolve().perform()
                adjusted_oTm.pose.position.z += 0.03
                MoveGripperMotion(motion="close", gripper=self.arm).resolve().perform()


            def to_sql(self) -> ORMPickUpAction:
                return ORMPickUpAction(self.arm, self.grasp)

            def insert(self, session: sqlalchemy.orm.session.Session, **kwargs):
                action = super().insert(session)
                # try to create the object designator
                if self.object_at_execution:
                    od = self.object_at_execution.insert(session, )
                    action.object = od.id
                else:
                    action.object = None

                session.add(action)
                session.commit()

                return action

        def __init__(self, object_designator_description: ObjectDesignatorDescription, arms: List[str],
                     grasps: List[str], resolver=None):
            """
            Lets the robot pick up an object. The description needs an object designator describing the object that should be
            picked up, an arm that should be used as well as the grasp from which side the object should be picked up.

            :param object_designator_description: List of possible object designator
            :param arms: List of possible arms that could be used
            :param grasps: List of possible grasps for the object
            :param resolver: An optional resolver that returns a performable designator with elements from the lists of possible paramter
            """
            super(PickUpAction, self).__init__(resolver)
            self.object_designator_description: ObjectDesignatorDescription = object_designator_description
            self.arms: List[str] = arms
            self.grasps: List[str] = grasps


class CuttingAction(ActionDesignatorDescription):
    """
    Designator to let the robot pick up an object.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):

        object_designator: ObjectDesignatorDescription.Object
        """
        Object designator describing the object that should be cut
        """

        arm: str
        """
        The arm that should be used for pick up
        """

        grasp: str
        """
        The grasp that should be used. For example, 'left' or 'right'
        """

        slice_thickness: float
        """
        The upper bound thickness of the slices
        """

        tool: str
        """
        The tool to cut with
        """

        technique: str
        """
        Technique used to cut the object.
        """

        object_at_execution: Optional[ObjectDesignatorDescription.Object] = dataclasses.field(init=False, repr=False)
        """
        The object at the time this Action got created. It is used to be a static, information holding entity. It is
        not updated when the BulletWorld object is changed.
        """


        @with_tree
        def perform(self) -> None:
            # Store the object's data copy at execution
            self.object_at_execution = self.object_designator.data_copy()

            # Retrieve object and robot from designators
            object = self.object_designator.bullet_world_object
            # Get grasp orientation and target pose
            grasp = robot_description.grasps.get_orientation_for_grasp(self.grasp)

            obj_dim = object.get_Object_Dimensions()

            dim = [max(obj_dim[0], obj_dim[1]), min(obj_dim[0], obj_dim[1]), obj_dim[2]]
            oTm = object.get_pose()
            object_pose = object.local_transformer.transform_to_object_frame(oTm, object)

            # from bread_dim calculate def a calculation that gets me the highest number from the first 2 entries

            # Given slice thickness is 3 cm or 0.03 meters
            slice_thickness = self.slice_thickness
            # Calculate slices and transform them to the map frame with orientation
            obj_length = dim[0]
            obj_width = dim[1]
            obj_height = dim[2]



            # Calculate the starting Y-coordinate offset (half the width minus half a slice thickness)
            if self.technique == 'halving':
                start_offset = 0
                num_slices = 1
            else:
                num_slices = 1
                    #int(obj_length // slice_thickness))
                start_offset = 0
                        #-obj_length / 2 + slice_thickness / 2)

            # Calculate slice coordinates
            slice_coordinates = [start_offset + i * slice_thickness for i in range(num_slices)]

            # Transform slice coordinates to map frame with orientation
            slice_poses = []
            for x in slice_coordinates:
                tmp_pose = object_pose.copy()
                tmp_pose.pose.position.y -= 3 * obj_width
                tmp_pose.pose.position.x = x
                sTm = object.local_transformer.transform_pose(tmp_pose, "map")
                slice_poses.append(sTm)

            for slice_pose in slice_poses:
                # rotate the slice_pose by grasp
                ori = helper.multiply_quaternions([slice_pose.orientation.x, slice_pose.orientation.y,
                                                   slice_pose.orientation.z, slice_pose.orientation.w],
                                                  grasp)

                oriR = helper.axis_angle_to_quaternion([0, 0, 1], 90)
                oriM = helper.multiply_quaternions([oriR[0], oriR[1], oriR[2], oriR[3]],
                                                   [ori[0], ori[1], ori[2], ori[3]])

                adjusted_slice_pose = slice_pose.copy()

                # Set the orientation of the object pose by grasp in MAP
                adjusted_slice_pose.orientation.x = oriM[0]
                adjusted_slice_pose.orientation.y = oriM[1]
                adjusted_slice_pose.orientation.z = oriM[2]
                adjusted_slice_pose.orientation.w = oriM[3]

                # Adjust the position of the object pose by grasp in MAP
                lift_pose = adjusted_slice_pose.copy()
                lift_pose.pose.position.z += 2 * obj_height
                # Perform the motion for lifting the tool
                BulletWorld.current_bullet_world.add_vis_axis(lift_pose)
                MoveTCPMotion(lift_pose, self.arm).resolve().perform()
                # Perform the motion for cutting the object
                BulletWorld.current_bullet_world.add_vis_axis(adjusted_slice_pose)
                MoveTCPMotion(adjusted_slice_pose, self.arm).resolve().perform()
                # Perform the motion for lifting the tool
                BulletWorld.current_bullet_world.add_vis_axis(lift_pose)
                MoveTCPMotion(lift_pose, self.arm).resolve().perform()

    def __init__(self, object_designator_description: ObjectDesignatorDescription, arms: List[str],
                 grasps: List[str], resolver=None):
        """
        Lets the robot pick up an object. The description needs an object designator describing the object that should be
        picked up, an arm that should be used as well as the grasp from which side the object should be picked up.

        :param object_designator_description: List of possible object designator
        :param arms: List of possible arms that could be used
        :param grasps: List of possible grasps for the object
        :param resolver: An optional resolver that returns a performable designator with elements from the lists of possible paramter
        """
        super(CuttingAction, self).__init__(resolver)
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.arms: List[str] = arms
        self.grasps: List[str] = grasps

    def __iter__(self):
        for object_, grasp, arm in itertools.product(iter(self.object_designator_description),
                                                                            self.grasps, self.arms):
            yield self.Action(object_, arm, grasp,
                              slice_thickness=0.05, tool="big_knife", technique="slicing")

    def ground(self) -> Action:
        """
        Default resolver, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        return next(iter(self))



class MixingAction(ActionDesignatorDescription):
    """
    Designator to let the robot pick up an object.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):

        object_designator: ObjectDesignatorDescription.Object
        """
        Object designator describing the object that should be picked up
        """

        object_tool_designator: ObjectDesignatorDescription.Object
        """
        Object designator describing the object that should be picked up
        """

        arm: str
        """
        The arm that should be used for pick up
        """

        grasp: str
        """
        The grasp that should be used. For example, 'left' or 'right'
        """

        object_at_execution: Optional[ObjectDesignatorDescription.Object] = dataclasses.field(init=False)
        """
        The object at the time this Action got created. It is used to be a static, information holding entity. It is
        not updated when the BulletWorld object is changed.
        """

        @with_tree
        def perform(self) -> None:
            # Store the object's data copy at execution
            self.object_at_execution = self.object_designator.data_copy()
            robot = BulletWorld.robot
            # Retrieve object and robot from designators
            object = self.object_designator.bullet_world_object
            # Get grasp orientation and target pose
            grasp = robot_description.grasps.get_orientation_for_grasp(self.grasp)

            obj_dim = object.get_Object_Dimensions()

            dim = [max(obj_dim[0], obj_dim[1]), min(obj_dim[0], obj_dim[1]), obj_dim[2]]
            obj_height = dim[2]
            oTm = object.get_pose()
            object_pose = object.local_transformer.transform_to_object_frame(oTm, object)

            def generate_spiral(pose, upward_increment, radial_increment, angle_increment, steps):
                x_start, y_start, z_start = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
                spiral_poses = []

                for t in range(2*steps):
                    tmp_pose = pose.copy()

                    r = radial_increment * t
                    a = angle_increment * t
                    h = upward_increment * t

                    x = x_start + r * math.cos(a)
                    y = y_start + r * math.sin(a)
                    z = z_start + h

                    tmp_pose.pose.position.x += x
                    tmp_pose.pose.position.y += y
                    tmp_pose.pose.position.z += z

                    spiralTm = object.local_transformer.transform_pose(tmp_pose, "map")
                    spiral_poses.append(spiralTm)
                    BulletWorld.current_bullet_world.add_vis_axis(spiralTm)

                return spiral_poses

            # Transform slice coordinates to map frame with orientation

            #this is a very good one but takes ages
            # spiral_poses = generate_spiral(object_pose, 0.0004, 0.0008, math.radians(10), 100)
            spiral_poses = generate_spiral(object_pose, 0.001, 0.0035, math.radians(30), 10)
            #
            #

            BulletWorld.current_bullet_world.remove_vis_axis()
            for spiral_pose in spiral_poses:
                # rotate the slice_pose by grasp
                # ori = helper.multiply_quaternions([slice_pose.orientation.x, slice_pose.orientation.y,
                #                                    slice_pose.orientation.z, slice_pose.orientation.w],
                #                                   [0,0,0,1])
                #                                   # helper.axis_angle_to_quaternion([0, 0, 1], 180))
                #
                oriR = helper.axis_angle_to_quaternion([1, 0, 0], 180)
                ori = helper.multiply_quaternions([spiral_pose.orientation.x, spiral_pose.orientation.y,
                                                   spiral_pose.orientation.z, spiral_pose.orientation.w],
                                                  oriR)
                # oriM = helper.multiply_quaternions([oriR[0], oriR[1], oriR[2], oriR[3]],
                #                                    [ori[0], ori[1], ori[2], ori[3]])
                #
                adjusted_slice_pose = spiral_pose.copy()
                #
                # # Set the orientation of the object pose by grasp in MAP
                adjusted_slice_pose.orientation.x = ori[0]
                adjusted_slice_pose.orientation.y = ori[1]
                adjusted_slice_pose.orientation.z = ori[2]
                adjusted_slice_pose.orientation.w = ori[3]

                # Adjust the position of the object pose by grasp in MAP
                lift_pose = adjusted_slice_pose.copy()
                lift_pose.pose.position.z += (obj_height + 0.08)
                # Perform the motion for lifting the tool
                #BulletWorld.current_bullet_world.add_vis_axis(lift_pose)
                MoveTCPMotion(lift_pose, self.arm).resolve().perform()






    def __init__(self, object_designator_description: ObjectDesignatorDescription,
                     object_tool_designator_description: ObjectDesignatorDescription, arms: List[str],
                 grasps: List[str], resolver=None):
        """
        Lets the robot pick up an object. The description needs an object designator describing the object that should be
        picked up, an arm that should be used as well as the grasp from which side the object should be picked up.

        :param object_designator_description: List of possible object designator
        :param arms: List of possible arms that could be used
        :param grasps: List of possible grasps for the object
        :param resolver: An optional resolver that returns a performable designator with elements from the lists of possible paramter
        """
        super(MixingAction, self).__init__(resolver)
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.object_tool_designator_description: ObjectDesignatorDescription = object_designator_description
        self.arms: List[str] = arms
        self.grasps: List[str] = grasps

    def ground(self) -> Action:
        """
        Default resolver, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.ground(),  self.object_designator_description.ground(),
                           self.arms[0], self.grasps[0])


