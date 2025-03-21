from enum import Enum, auto

from demos.pycram_multirobot_real_demo.utils import rotated_quaternion
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
        self.placing_poses = {}

        self.set_objects()

    def set_objects(self):
        milk_position = [2.61, 4.8, 0.85]
        milk_pickup_orientation = rotated_quaternion(angle=90)

        milk_place_position = [1.87, 5.24, 0.45]
        milk_place_orientation = rotated_quaternion(angle=180)

        self.create_object("milk", ObjectOptions.MILK, ObjectType.MILK,
                           starting_position=milk_position, starting_orientation=milk_pickup_orientation,
                           placing_position=milk_place_position, placing_orientation=milk_place_orientation)

        coffee_position = [2.21, 4.8, 0.85]
        coffee_pickup_orientation = rotated_quaternion(angle=90)

        coffee_place_position = [1.87, 5.24, 0.45]
        coffee_place_orientation = rotated_quaternion(angle=180)

        self.create_object("coffee", ObjectOptions.COFFEE, ObjectType.MILK,
                           starting_position=coffee_position, starting_orientation=coffee_pickup_orientation,
                           placing_position=coffee_place_position, placing_orientation=coffee_place_orientation)

        chips_position = [2.45, 4.8, 0.85]
        chips_pickup_orientation = rotated_quaternion(angle=90)

        chips_place_position = [2.7, 2.7, 0.7]
        chips_place_orientation = rotated_quaternion(angle=-90)

        self.create_object("chips", ObjectOptions.CHIPS, ObjectType.MILK,
                           starting_position=chips_position, starting_orientation=chips_pickup_orientation,
                           placing_position=chips_place_position, placing_orientation=chips_place_orientation)

    def create_object(self, name, object_option, object_type, starting_position, starting_orientation, placing_position,
                      placing_orientation, path="milk.stl"):
        obj_starting_pose = Pose(position=starting_position, orientation=starting_orientation)
        obj_object = Object(name, object_type, path=path, pose=obj_starting_pose)

        obj_desig = ObjectDesignatorDescription.Object(obj_object.name, ObjectType.MILK, obj_object)

        obj_placing_pose = Pose(position=placing_position, orientation=placing_orientation)

        self.objects[object_option] = obj_object
        self.desigs[object_option] = obj_desig
        self.placing_poses[object_option] = obj_placing_pose
