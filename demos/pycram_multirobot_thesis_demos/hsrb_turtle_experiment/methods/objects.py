from enum import Enum, auto
from typing import List

from .utils import rotated_quaternion
from pycram.datastructures.enums import ObjectType
from pycram.datastructures.pose import Pose
from pycram.designator import ObjectDesignatorDescription
from pycram.world_concepts.world_object import Object


class ObjectOptions(Enum):
    MILK = auto()
    COFFEE = auto()
    CHIPS = auto()


class ObjectManager:
    def __init__(self):
        self.objects = {}
        self.desigs = {}
        self.placing_pose_on_turtle = {}
        self.placing_pose_on_table = {}

        self.set_objects()

    def set_objects(self):
        milk_position = [2.03, 4.8, 0.85]
        milk_pickup_orientation = rotated_quaternion(angle=90)

        milk_place_position_on_turtle = [1.87, 5.24, 0.45]
        milk_place_orientation_on_turtle = rotated_quaternion(angle=180)

        # TODO: Adjust placing pose for table
        milk_place_position_on_table = [3.6, 4.5, 0.9]
        milk_place_orientation_on_table = rotated_quaternion(angle=180)

        self.create_object("milk", ObjectOptions.MILK, ObjectType.MILK,
                           starting_position=milk_position, starting_orientation=milk_pickup_orientation,
                           placing_position_on_turtle=milk_place_position_on_turtle,
                           placing_orientation_on_turtle=milk_place_orientation_on_turtle,
                           placing_position_on_table=milk_place_position_on_table,
                           placing_orientation_on_table=milk_place_orientation_on_table)

        coffee_position = [2.19, 4.8, 0.85]
        coffee_pickup_orientation = rotated_quaternion(angle=90)

        coffee_place_position_on_turtle = [1.87, 5.24, 0.45]
        coffee_place_orientation_on_turtle = rotated_quaternion(angle=180)

        # TODO: Adjust placing pose for table
        coffee_place_position_on_table = [3.6, 4.65, 0.9]
        coffee_place_orientation_on_table = rotated_quaternion(angle=180)

        self.create_object("coffee", ObjectOptions.COFFEE, ObjectType.MILK,
                           starting_position=coffee_position, starting_orientation=coffee_pickup_orientation,
                           placing_position_on_turtle=coffee_place_position_on_turtle,
                           placing_orientation_on_turtle=coffee_place_orientation_on_turtle,
                           placing_position_on_table=coffee_place_position_on_table,
                           placing_orientation_on_table=coffee_place_orientation_on_table)

        chips_position = [2.45, 4.8, 0.85]
        chips_pickup_orientation = rotated_quaternion(angle=90)

        chips_place_position_on_turtle = [2.7, 2.7, 0.7]
        chips_place_orientation_on_turtle = rotated_quaternion(angle=-90)

        # TODO: Adjust placing pose for table
        chips_place_position_on_table = [3.6, 4.8, 0.9]
        chips_place_orientation_on_table = rotated_quaternion(angle=180)

        self.create_object("chips", ObjectOptions.CHIPS, ObjectType.MILK,
                           starting_position=chips_position, starting_orientation=chips_pickup_orientation,
                           placing_position_on_turtle=chips_place_position_on_turtle,
                           placing_orientation_on_turtle=chips_place_orientation_on_turtle,
                           placing_position_on_table=chips_place_position_on_table,
                           placing_orientation_on_table=chips_place_orientation_on_table)

    def create_object(self, name: str,
                      object_option: ObjectOptions,
                      object_type: ObjectType,
                      starting_position: List[float],
                      starting_orientation: List[float],
                      placing_position_on_turtle: List[float],
                      placing_orientation_on_turtle: List[float],
                      placing_position_on_table: List[float],
                      placing_orientation_on_table: List[float],
                      path: str = "milk.stl"):
        obj_starting_pose = Pose(position=starting_position, orientation=starting_orientation)
        obj_object = Object(name, object_type, path=path, pose=obj_starting_pose)

        obj_desig = ObjectDesignatorDescription.Object(obj_object.name, ObjectType.MILK, obj_object)

        obj_placing_pose_on_turtle = Pose(position=placing_position_on_turtle,
                                          orientation=placing_orientation_on_turtle)
        obj_placing_pose_on_table = Pose(position=placing_position_on_table, orientation=placing_orientation_on_table)

        self.objects[object_option] = obj_object
        self.desigs[object_option] = obj_desig
        self.placing_pose_on_turtle[object_option] = obj_placing_pose_on_turtle
        self.placing_pose_on_table[object_option] = obj_placing_pose_on_table
