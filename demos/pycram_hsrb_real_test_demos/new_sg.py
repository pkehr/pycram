from geometry_msgs.msg import Quaternion, Vector3

from pycram.datastructures.enums import WorldMode
from pycram.designators.motion_designator import DoorOpenMotion
from pycram.external_interfaces import giskard

# try:
#     from demos.pycram_carry_my_luggage.utils.cml_helper import fts
# except ModuleNotFoundError:
#     pass
from pycram import helper
from pycram.designators.location_designator import find_placeable_pose
# from pycram.external_interfaces.giskard import sync_worlds
from pycram.failures import PerceptionObjectNotFound
from pycram.language import Code

from pycram.designators.action_designator import *
from pycram.process_module import real_robot, simulated_robot
# import pycram.external_interfaces.giskard_new as giskardpy
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.designators.object_designator import *
from pycram.ros_utils.robot_state_updater import KitchenStateUpdater, RobotStateUpdater
from pycram.utilities.robocup_utils import TextToSpeechPublisher, HSRBMoveGripperReal, pakerino, GraspListener, \
    ImageSwitchPublisher, StartSignalWaiter

from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld

demo_mode = simulated_robot
demo_mode = real_robot

world = BulletWorld(WorldMode.GUI)
# v = VizMarkerPublisher()
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2024_1.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

lt = LocalTransformer()
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/" + "hsrb" + ".urdf")
# robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"])
KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")

start_with_open_shelf_doors = True
shelf_door_handle = "shelf_billy:shelf_billy:shelf_door_left:handle"


if demo_mode == real_robot:
    print("Real Robot")
    grasp_listener = GraspListener()
    img_swap = ImageSwitchPublisher()
    start_signal_waiter = StartSignalWaiter()
    fts = ForceTorqueSensor(robot_name='hsrb')
    move = PoseNavigator()
    # RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
    gripper = HSRBMoveGripperReal()
else:
    # milk1 = Object("milk1", ObjectType.MILK, "milk.stl", pose=Pose([5.4, 3.8, 0.63]))
    cereal1 = Object("cereal1", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([5.4, 4.1, 1.16]))  # Pose([5.4, 4.1, 1.16])
    bowl = Object("bowl1", ObjectType.BOWL, "bowl.stl", pose=Pose([5.4, 3.8, 0.57]))  # Pose([5.4, 3.8, 0.57])
    # [2.5, 5.717920690528091, 0.715]
    milk2 = Object("milk2", ObjectType.MILK, "milk.stl", pose=Pose([2.6, 4.8, 0.81], [0, 0, 1, 1]))
    cereal2 = Object("cereal2", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl",
                     pose=Pose([2, 4.8, 0.82], [0, 0, 1, 1]))
    bowl2 = Object("bowl2", ObjectType.BOWL, "bowl.stl", pose=Pose([2.3, 4.8, 0.75], [0, 0, 1, 1]))


def navigate_to(nav_pose, interrupt_bool=True):
    if demo_mode == real_robot:
        move.pub_now(nav_pose, interrupt_bool=interrupt_bool)
        robot.set_pose(nav_pose)
    elif demo_mode == simulated_robot:
        NavigateAction([nav_pose]).resolve().perform()
        giskard.sync_worlds()


def park_arms():
    if demo_mode == real_robot:
        return pakerino()
    elif demo_mode == simulated_robot:
        ParkArmsAction([Arms.LEFT]).resolve().perform()

        return True

def park_arms_top():
    config = {'arm_flex_joint': -1.1, 'arm_lift_joint': 0, 'arm_roll_joint': 0,
              'wrist_flex_joint': -1.9, 'wrist_roll_joint': 0}
    if demo_mode == real_robot:
        return pakerino(config=config)
    elif demo_mode == simulated_robot:
        MoveJointsMotion(list(config.keys()), list(config.values())).perform()
        return True


def set_joint_config(z_torso=None, config=None):
    if demo_mode == real_robot:
        return pakerino(z_torso, config)
    elif demo_mode == simulated_robot:
        return MoveJointsMotion(list(config.keys()), list(config.values())).perform()


def talk_pub(sentence=Any, talk_bool=True, wait_bool=True):
    if demo_mode == real_robot:
        return talk.pub_now(sentence, talk_bool, wait_bool)
    elif demo_mode == simulated_robot:
        return print(sentence)


def move_head(look_pose):
    if demo_mode == real_robot:
        return giskard.move_head_to_pose(look_pose)
    elif demo_mode == simulated_robot:
        return LookAtAction([look_pose]).resolve().perform()

def gripper_motion(motion):
    if demo_mode == real_robot:
        gripper_state = GripperState.OPEN if motion == "open" else GripperState.CLOSE
        MoveGripperMotion(gripper_state, Arms.LEFT).perform()
    elif demo_mode == simulated_robot:
        giskard.set_gripper_state(motion)

shelf_floor_0 = 'shelf_billy:shelf_billy:shelf_floor_0'
shelf_floor_1 = 'shelf_billy:shelf_billy:shelf_floor_1'
shelf_floor_2 = 'shelf_billy:shelf_billy:shelf_floor_2'
pick_table_link = "popcorn_table:p_table:table_center"

talk = TextToSpeechPublisher()

giskard.clear()

shelf_pose = Pose([4.3, 3.8, 0.0], [0.0, 0.0, 0, 1])
rotated_shelf_pose = Pose([4.3, 3.8, 0.0],
                          [0.0, 0.0, 0.707, 0.707])
rotated_shelf_pose_to_table = Pose([4.3, 3.8, 0.0], [0.0, 0.0, 0.-707, 0.707])
table_pose = Pose([2, 4.2, 0.0], [0.0, 0.0, 0.707, 0.707])
table_pose_pre = Pose([2, 3.9, 0.0], [0.0, 0.0, 0.707, 0.707])
table_pose_to_shelf = Pose([2, 3.9, 0.0], [0.0, 0.0, -0.707, 0.707])
giskard.sync_worlds()

if start_with_open_shelf_doors:
    kitchen.set_joint_position("shelf_billy:shelf_billy:shelf_door_left:joint", -1.7)
    if demo_mode == simulated_robot:
        giskard.set_joint_positions(kitchen, {"shelf_billy:shelf_billy:shelf_door_left:joint": -1.7})

def multiply_quaternions(q1, q2):
    """
    Multiply two quaternions.

    Parameters:
    q1 (tuple): First quaternion (x1, y1, z1, w1).
    q2 (tuple): Second quaternion (x2, y2, z2, w2).

    Returns:
    tuple: The product of the two quaternions.
    """

    x1, y1, z1, w1 = q1.x, q1.y, q1.z, q1.w
    x2, y2, z2, w2 = q2.x, q2.y, q2.z, q2.w

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (x, y, z, w)


# List of objects
objects = ["Fork", "Pitcher", "Bleachcleanserbottle", "Crackerbox", "Minisoccerball", "Baseball", "Mustardbottle",
           "Jellochocolatepuddingbox", "Wineglass", "Orange", "Coffeepack", "Softball", "Metalplate",
           "Pringleschipscan", "Strawberry", "Glasscleanerspraybottle", "Tennisball", "Spoon", "Metalmug",
           "Abrasivesponge", "Jellobox", "Dishwashertab", "Knife", "Cerealbox", "Metalbowl", "Sugarbox", "Coffeecan",
           "Milkpackja", "Apple", "Tomatosoupcan", "Tunafishcan", "Gelatinebox", "Pear", "Lemon", "Banana",
           "Pottedmeatcan", "Peach", "Plum", "Rubikscube", "Mueslibox", "Cupblue", "Cupgreen", "Largemarker",
           "Masterchefcan", "Scissors", "Scrubcleaner", "Grapes", "Cup_small", "screwdriver", "clamp", "hammer",
           "wooden_block", "Cornybox*", 'breakfast_cereal']

# Group objects by similarity
groups = {"Kitchen Utensils and Tools": ["Fork", "Spoon", "Knife"],
          "Containers and Drinkware": ["Pitcher", "Metalplate", "Metalbowl", "Metalmug", "Wineglass", "Cupblue",
                                       "Cupgreen", "Cup_small"],
          "Cleaning Supplies": ["Bleachcleanserbottle", "Glasscleanerspraybottle", "Abrasivesponge", "Scrubcleaner"],
          "Packaged Food and Beverages": ["Crackerbox", "Mustardbottle", "Jellochocolatepuddingbox", "Coffeepack",
                                          "Pringleschipscan", "Jellobox", "Dishwashertab", "Cerealbox", "Sugarbox",
                                          "Coffeecan", "Milkpackja", "Tomatosoupcan", "Tunafishcan", "Gelatinebox",
                                          "Mueslibox", "Cornybox*", "Masterchefcan", "Pottedmeatcan",
                                          'breakfast_cereal'],
          "Fruits": ["Orange", "Strawberry", "Apple", "Pear", "Lemon", "Banana", "Peach", "Plum", "Grapes"],
          "Sports Equipment": ["Minisoccerball", "Baseball", "Softball", "Tennisball"],
          "Miscellaneous": ["Rubikscube", "Largemarker", "Scissors", "screwdriver", "clamp", "hammer", "wooden_block"],
          "Unknown": ["Unknown"]}

cereal_types = ["Muesli", "Cereal"]
# 'shelf:shelf:shelf_floor_0',
links_from_shelf = [shelf_floor_1, shelf_floor_2]

popcorn_frame = "popcorn_table:p_table:table_front_edge_center"


def find_group(obj_type):
    obj_lower = obj_type.lower()
    for group, items in groups.items():
        for item in items:
            if obj_lower in item.lower() or item.lower() in obj_lower:
                return group
    return None


def get_closet_link_to_pose(obj_pose):
    position_distance = 30
    nearest_link = None
    for link in links_from_shelf:
        link_pose = kitchen.get_link_pose(link)
        posez = obj_pose.position.z

        dis = abs(link_pose.pose.position.z - posez)
        if dis <= position_distance:
            position_distance = dis
            nearest_link = link
    return nearest_link


def get_closest_pose(obj_pose, pose_list):
    position_distance = float('inf')
    nearest_pose = None

    for pose in pose_list:
        dis = ((obj_pose.pose.position.x - pose.position.x) ** 2 + (obj_pose.pose.position.y - pose.position.y) ** 2 + (
                obj_pose.pose.position.z - pose.position.z) ** 2) ** 0.5
        if dis < position_distance:
            position_distance = dis
            nearest_pose = pose

    return nearest_pose


def calculate_z_offsets(links_from_shelf):
    """
    Calculate the Z offsets between each link in links_from_shelf and return a dictionary mapping each link
    to the Z height difference to the next link.
    """
    z_offsets = {}

    # Get the pose (including Z position) for each link
    link_poses = {link: kitchen.get_link_pose(link) for link in links_from_shelf}

    # Iterate through the links and calculate Z offsets
    for i, link in enumerate(links_from_shelf[:-1]):  # Skip the last link
        current_pose = link_poses[link]
        next_pose = link_poses[links_from_shelf[i + 1]]
        current_z = current_pose.position.z  # Adjust according to the structure of Pose
        next_z = next_pose.position.z  # Adjust according to the structure of Pose
        z_offset = next_z - current_z
        z_offsets[link] = z_offset

    return z_offsets


def get_z_height_to_next_link(link, z_offsets):
    """
    Return the Z height difference from the given link to the next link.
    """
    return z_offsets.get(link, None)


# noteme if you want to be able to run this in simulation you will have to do get.aabb without bullet world obj
def find_pose_in_shelf(group, object, groups_in_shelf):
    link = None
    nearest_pose_to_group = None
    try:
        link = groups_in_shelf[group][1]
        group_pose = groups_in_shelf[group][0]
        place_poses = find_placeable_pose(link, kitchen_desig.resolve(), robot_desig.resolve(), "left", world, 25,
                                          object_desig=object, clearance_radius=0.19)
        nearest_pose_to_group = get_closest_pose(group_pose, place_poses)

    except (TypeError, KeyError):
        place_poses = []
        for link in links_from_shelf:
            place_poses.append(
                (find_placeable_pose(link, kitchen_desig.resolve(), robot_desig.resolve(), "left", world, 25,
                                     object_desig=object), link)
            )

        # in this case it's the biggest group since why not? i assume most poses are then free
        longest_group = max(place_poses, key=lambda x: len(x[0]))
        nearest_pose_to_group = longest_group[0][0]
        link = longest_group[1]
        if group not in groups_in_shelf:
            groups_in_shelf[group] = [nearest_pose_to_group, longest_group[1]]

    if nearest_pose_to_group:
        z_offsets = calculate_z_offsets(links_from_shelf)
        z_height_to_next = get_z_height_to_next_link(link, z_offsets)

        pose_in_shelf = lt.transform_pose(nearest_pose_to_group, kitchen.get_link_tf_frame(link))
        # pose_in_shelf.pose.position.x = -0.10
        # if z_height_to_next:
        #     pose_in_shelf.pose.position.z = (z_height_to_next / 2) - 0.01
        # else:
        pose_in_shelf.pose.position.z += 0.0
        adjusted_pose_in_map = lt.transform_pose(pose_in_shelf, "map")
        # world.current_world.add_vis_axis(adjusted_pose_in_map)

        return adjusted_pose_in_map, link


previous_value = None


def monitor_func_place():
    global previous_value
    der = fts.get_last_value()
    current_value = fts.get_last_value()

    prev_force_x = previous_value.wrench.force.x
    curr_force_x = current_value.wrench.force.x
    change_in_force_x = abs(curr_force_x - prev_force_x)
    print(f"Current Force X: {curr_force_x}, Previous Force X: {prev_force_x}, Change: {change_in_force_x}")

    def calculate_dynamic_threshold(previous_force_x):
        # Placeholder for a dynamic threshold calculation based on previous values
        # This function can be enhanced to calculate a threshold based on the history of values or other logic
        return max(0.1 * abs(previous_force_x), 1.5)  # Example: 10% of the previous value or a minimum of 1.5

    if change_in_force_x >= calculate_dynamic_threshold(previous_force_x=prev_force_x):
        print("Significant change detected")

        return SensorMonitoringCondition

    return False


def placeorpark(object_name, object, grasp, talk_bool, target_location, link, pick_up_bool):
    global previous_value
    oTm = target_location
    if demo_mode == real_robot:
        oTm.pose.position.z += 0.03

    grasp_rotation = RobotDescription.current_robot_description.grasps[grasp]
    # oTb = lt.transform_pose(oTm, kitchen.get_link_tf_frame(link))

    base_frame = RobotDescription.current_robot_description.name + "/" + RobotDescription.current_robot_description.base_link
    oTb = lt.transform_pose(oTm, base_frame)

    if grasp == Grasp.TOP:
        oTb.pose.position.z += 0.03

    oTb.orientation = grasp_rotation

    # if grasp != Grasp.TOP:
    #     oTb.orientation = grasp_rotation

    oTb_prepose = oTb.copy()
    oTb_prepose.set_position([oTb.position.x - 0.2, oTb.position.y, oTb.position.z + 0.05])
    oTmG_prepose = lt.transform_pose(oTb_prepose, "map")

    oTmG = lt.transform_pose(oTb, "map")
    # marker = AxisMarkerPublisher(frame_id="map")
    # marker.publish([oTmG], 30, length=0.1)

    # BulletWorld.current_world.add_vis_axis(oTmG)

    # todome this mshoudl be depending on the height of the shelf tbh

    # if grasp == Grasp.FRONT:
    #     config_for_placing = {'arm_lift_joint': 0.20, 'arm_flex_joint': -0.16, 'arm_roll_joint': -0.0145,
    #                           'wrist_flex_joint': -1.417, 'wrist_roll_joint': 0.0}
    # else:
    if object.obj_type == ObjectType.BOWL:
        MoveTorsoAction([0.5]).resolve().perform()
    else:
        config_for_placing = {'arm_flex_joint': 0.20, 'arm_lift_joint': 0.5, 'arm_roll_joint': 0,
                              'wrist_flex_joint': -1.6, 'wrist_roll_joint': 0, }

        set_joint_config(config=config_for_placing)
    if pick_up_bool:
        talk_pub(f"Pick Up now! {object_name.split('_')[0]} from: {grasp}")
    else:
        talk_pub(f"Placing now! {object_name.split('_')[0]} from: {grasp}")
    giskard_return = giskard.achieve_sequence_pick_up([oTmG_prepose], demo_mode)
    while not giskard_return:
        rospy.sleep(0.1)
    giskard.update_from_giskard(robot, giskard_return)

    giskard_return = giskard.achieve_sequence_pick_up([oTmG], demo_mode)
    while not giskard_return:
        rospy.sleep(0.1)
    giskard.update_from_giskard(robot, giskard_return)
    config_after_place = {'arm_lift_joint': 0.0}
    if demo_mode == real_robot:
        previous_value = fts.get_last_value()
        try:
            # talk_pub("tracking placing now")
            # plan = Code(lambda: giskard.test(config_after_place)) >> Monitor(monitor_func_place)
            # return_plan = plan.perform()
            place_pose = oTmG.copy()
            print("SKipping FTS Placing for now, until manip is here to help")
            # place_pose.set_position([oTmG.position.x, oTmG.position.y, oTmG.position.z-0.1])
            # giskard_return = giskard.achieve_place_w_fts(place_pose, RobotDescription.current_robot_description.get_arm_chain(
            #     Arms.LEFT).get_tool_frame(), 'map', GiskardStateFTS.PLACE)
            # talk_pub("Object released")
            # giskard.update_from_giskard(robot, giskard_return)

        except Exception as e:
            print(f"Exception type: {type(e).__name__}")
    giskard.achieve_detached(object, demo_mode)
    BulletWorld.robot.detach(object)
    gripper_motion("open")

    giskard_return = giskard.achieve_sequence_pick_up([oTmG_prepose], demo_mode)
    while not giskard_return:
        rospy.sleep(0.1)
    giskard.update_from_giskard(robot, giskard_return)

    gripper_motion("close")
    giskard.avoid_all_collisions()
    park = park_arms()
    while not park:
        print("waiting for park")
        rospy.sleep(0.1)
    giskard.update_from_giskard(robot, park)


# noteme this is with driving! return grasped_bool, grasp, group, object, obj_id, groups_on_table
def process_pick_up_objects(talk_bool):
    # drive
    talk_pub("driving", True)
    navigate_to(table_pose)
    # look
    locationtoplace = Pose([2.5, 5.717920690528091, 0.715])
    look_pose = kitchen.get_link_pose(pick_table_link)
    # park
    perceive_conf = {'arm_lift_joint': 0.25, 'wrist_flex_joint': 1.8, 'arm_roll_joint': -1, }
    set_joint_config(config=perceive_conf)
    # look
    move_head(look_pose)

    talk_pub("perceiving", True)
    groups_on_table = {}

    # noteme detect on table
    try:
        table_obj = DetectAction(technique='all').resolve().perform()
        giskard.sync_worlds()
        dictionary = table_obj
        for value in dictionary.values():
            try:
                group = find_group(value.obj_type)
                if group == "Containers and Drinkware" and not handled_cereal:
                    continue
                if any(cereal_type.lower() in value.obj_type.lower() for cereal_type in cereal_types) and handled_cereal:
                    continue
                groups_on_table[value.name] = [value, group]
            except AttributeError:
                pass
        giskard.sync_worlds()
    except PerceptionObjectNotFound:
        talk_pub("I am Done, I hope I did good!")
        return

    # noteme if groups were found
    if groups_on_table:
        groups_on_table_w_table_frame = {}
        for key, obj in groups_on_table.items():
            obj_pose = obj[0].world_object.pose
            object_raw = obj[0]
            tf_link = kitchen.get_link_tf_frame(popcorn_frame)
            base_frame = RobotDescription.current_robot_description.name + "/" + RobotDescription.current_robot_description.base_link
            oTb = lt.transform_pose(obj_pose, base_frame)

            # Previous version used tf_link of furniture to calculate poses. but since we may not know from which
            # direction we approach the table, this may not be good. so I am using the base_link instead for now
            # oTb = lt.transform_pose(obj_pose, tf_link)


            grasp_set = None
            # if to far behind the front face were robots look on the table
            if object_raw.obj_type == "Metalbowl":
                grasp_set = Grasp.TOP

            # noteme this saves the group
            try:
                obj[1]
            except KeyError:
                obj[1] = "Unknown"

            groups_on_table_w_table_frame[key] = (obj[0], oTb, grasp_set, obj[1])

        # noteme sorts objects on the table with x so i can pick up the "most nearest"
        sorted_groups = sorted(groups_on_table_w_table_frame.items(), key=lambda item: item[1][1].pose.position.x)
        sorted_dict = dict(sorted_groups)

        # noteme get first items since we sorted it already
        first_key, first_value = next(iter(sorted_dict.items()))
        first_object = first_value[0]
        oTb = first_value[1]
        original_pose = lt.transform_pose(first_value[1], "map")
        grasp_set = first_value[2]
        group = first_value[3]
        print(group)
        obj_id = first_object.world_object.id
        object_name = first_object.world_object.name
        object = first_object.world_object

        angle = helper.quaternion_to_angle(
            (oTb.pose.orientation.x, oTb.pose.orientation.y, oTb.pose.orientation.z, oTb.pose.orientation.w))
        object_dim = object.get_object_dimensions()
        print(f"obj dim von {object_name} {object_dim}")

        if grasp_set:
            grasp = Grasp.TOP
        else:
            if object_dim[2] < 0.055:
                rospy.logwarn(f"{object_name} grasp is set to top, angle: {angle}")
                rospy.logwarn(f"{object_name} and height {object_dim[2]}")
                rospy.logwarn(f"{object_name} and width {object_dim[0]}")
                grasp = Grasp.TOP
            elif object_dim[2] < 0.065 or angle > 40 and (object_dim[0] > 0.075 and object_dim[1] > 0.075):
                rospy.logwarn(f"{object_name} grasp is set to top, angle: {angle}")
                rospy.logwarn(f"{object_name} and height {object_dim[2]}")
                rospy.logwarn(f"{object_name} and width {object_dim[0]}")
                grasp = Grasp.TOP
            else:
                rospy.logwarn(f"{object_name} grasp is set to front, angle: {angle}")
                rospy.logwarn(f"{object_name} and height {object_dim[2]}")
                rospy.logwarn(f"{object_name} and width {object_dim[0]}")
                grasp = Grasp.FRONT

            if grasp == Grasp.TOP:
                print("pose adjusted with z")
                oTb.pose.position.z += (object_dim[2] / 10)
                if object_dim[2] < 0.02:
                    rospy.logwarn(f"I am not able to grasp the object: {object_name} please help me!")
                    oTb.pose.position.z = 0.011
            else:
                oTb.pose.position.x += 0.03

        if first_object.obj_type == ObjectType.BOWL:
            oTb.pose.position.z += 0.02
            oTb.pose.position.y += 0.05

        grasp_rotation = RobotDescription.current_robot_description.grasps[grasp]
        if grasp == Grasp.TOP:
            grasp_q = Quaternion(grasp_rotation[0], grasp_rotation[1], grasp_rotation[2], grasp_rotation[3])
            oTb.orientation = multiply_quaternions(oTb.pose.orientation, grasp_q)
        else:
            oTb.orientation = grasp_rotation
        oTb_prepose = oTb.copy()
        oTb_prepose.set_position([oTb.position.x-0.2, oTb.position.y, oTb.position.z+0.05])
        oTmG_prepose = lt.transform_pose(oTb_prepose, "map")

        oTmG = lt.transform_pose(oTb, "map")

        oTb_lift = oTb.copy()
        oTb_lift.set_position([oTb.position.x, oTb.position.y, oTb.position.z + 0.05])
        oTmG_lift = lt.transform_pose(oTb_lift, "map")
        # after_pose = oTmG.copy()
        # after_pose.pose.position.z += 0.02

        z_color = [1, 0, 1, 1]
        # BulletWorld.current_world.add_vis_axis(after_pose)
        # BulletWorld.current_world.add_vis_axis(oTmG)
        navigate_to(table_pose_pre)

        if object.obj_type == ObjectType.BOWL:
            config_for_placing = {'arm_lift_joint': -1, 'arm_flex_joint': -0.16, 'arm_roll_joint': -0.0145,
                                  'wrist_flex_joint': -1.417, 'wrist_roll_joint': 0.0}
        else:
            config_for_placing = {'arm_flex_joint': -1.1, 'arm_lift_joint': 1.15, 'arm_roll_joint': 0,
                                  'wrist_flex_joint': -1.6, 'wrist_roll_joint': 0, }

        set_joint_config(config=config_for_placing)

        # @debug1
        ##########
        # giskard_return = giskard.achieve_sequence_pick_up([oTmG_prepose], demo_mode)
        # while not giskard_return:
        #     rospy.sleep(0.1)
        # giskard.update_from_giskard(robot, giskard_return)
        #
        # gripper_motion("open")
        # talk_pub(f"Pick Up now! {object_name.split('_')[0]} from: {grasp}")
        # giskard_return = giskard.achieve_sequence_pick_up([oTmG], demo_mode)
        # while not giskard_return:
        #     rospy.sleep(0.1)
        ##########
        gripper_motion("open")
        talk_pub(f"Pick Up now! {object_name.split('_')[0]} from: {grasp}")
        giskard_return = giskard.achieve_sequence_pick_up([oTmG_prepose, oTmG], demo_mode)
        while not giskard_return:
            rospy.sleep(0.1)
        ##########

        giskard.update_from_giskard(robot, giskard_return)
        giskard.achieve_attached(object, demo_mode=demo_mode)
        tip_link = 'hand_gripper_tool_frame'
        BulletWorld.robot.attach(child_object=object, parent_link=tip_link)
        gripper_motion("close")

        print()
        # @debug2
        ##########
        # giskard_return = giskard.achieve_sequence_pick_up([oTmG_lift], demo_mode)
        # while not giskard_return:
        #     rospy.sleep(0.1)
        # giskard.update_from_giskard(robot, giskard_return)
        #
        # giskard_return = giskard.achieve_sequence_pick_up([oTmG_prepose], demo_mode)
        # while not giskard_return:
        #     rospy.sleep(0.1)
        # giskard.update_from_giskard(robot, giskard_return)
        ##########
        giskard_return = giskard.achieve_sequence_pick_up([oTmG_lift, oTmG_prepose], demo_mode)
        while not giskard_return:
            rospy.sleep(0.1)
        ##########
        giskard.update_from_giskard(robot, giskard_return)

        giskard.avoid_all_collisions()
        if grasp == Grasp.TOP:
            # navigate_to(table_pose_pre)
            park = park_arms()
        else:
            park = park_arms()
        while not park:
            print("waiting for park")
            rospy.sleep(0.1)
        giskard.update_from_giskard(robot, park)
        if demo_mode == real_robot:
            if grasp_listener.check_grasp():
                talk_pub("Grasped a object")
                grasped_bool = True
            else:
                talk_pub("I was not able to grasped a object")
                grasped_bool = False
        else:
            grasped_bool = True

        groups_on_table.__delitem__(object_name)
        return grasped_bool, grasp, group, object, obj_id, first_object, groups_on_table, original_pose
    else:
        talk_pub("I think am Done, I hope I did good!")
        raise TypeError


# noteme  with look  return groups in shelf
def process_objects_in_shelf(talk_bool):
    # perceive shelf
    groups_in_shelf = {
        "Kitchen Utensils and Tools": None,
        "Containers and Drinkware": None,
        "Cleaning Supplies": None,
        "Packaged Food and Beverages": None,
        "Fruits": None,
        "Sports Equipment": None,
        "Miscellaneous": None,
        "Unknown": None
    }
    look_pose = kitchen.get_link_pose(shelf_floor_0)
    perceive_conf = {'arm_lift_joint': 0.25, 'wrist_flex_joint': 1.8, 'arm_roll_joint': -1, }
    set_joint_config(config=perceive_conf)
    # look
    move_head(look_pose)

    talk_pub("perceiving", True)

    shelf_obj = DetectAction(technique='all').resolve().perform()
    giskard.sync_worlds()
    dictionary = shelf_obj
    for value in dictionary.values():

        try:
            group = find_group(value.obj_type)
            link = get_closet_link_to_pose(value.pose)
            groups_in_shelf[group] = [value.pose, link, value]

        except AttributeError:
            pass

    look_pose = kitchen.get_link_pose(shelf_floor_2)
    move_head(look_pose)

    talk_pub("perceiving", True)

    shelf_obj = DetectAction(technique='all').resolve().perform()
    giskard.sync_worlds()
    dictionary = shelf_obj
    for value in dictionary.values():

        try:
            group = find_group(value.obj_type)
            link = get_closet_link_to_pose(value.pose)
            groups_in_shelf[group] = [value.pose, link, value]

        except AttributeError:
            pass

    return groups_in_shelf


groups_in_shelf = {}
handled_cereal = False

def demo(step):
    global groups_in_shelf, handled_cereal
    with ((((demo_mode)))):
        object_name = None,
        object = None,
        grasp = None,
        talk_bool = None,
        target_location = None,
        link = None
        talk_bool = True
        gripper_motion("close")
        park = park_arms()
        while not park:
            print("waiting for park")
            rospy.sleep(0.1)
        giskard.update_from_giskard(robot, park)

        if step <= 1:
            talk_pub("driving", True)
            navigate_to(rotated_shelf_pose, interrupt_bool=False)
            navigate_to(shelf_pose, interrupt_bool=False)
            if not start_with_open_shelf_doors:
                offset = Vector3()
                # offset.z = -0.025
                # offset.y = -0.039
                MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
                giskard_return = giskard.grasp_doorhandle(shelf_door_handle, offset) # , offset)
                giskard.update_from_giskard(robot, giskard_return)
                MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()
                giskard_return = giskard.open_doorhandle(shelf_door_handle)
                giskard.update_from_giskard(robot, giskard_return)
                MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()

            groups_in_shelf = process_objects_in_shelf(talk_bool)
            navigate_to(rotated_shelf_pose_to_table, interrupt_bool=False)

        if step <= 2:
            try:
                grasped_bool, grasp, group, object, obj_id, object_raw, groups_on_table, original_pose = process_pick_up_objects(
                    talk_bool)
                print(grasped_bool, grasp, group, object, obj_id, object_raw, groups_on_table)
            except TypeError as e:
                print(f"done (caught error {e})")
                return

        if step <= 3:
            bowl = None
            if any(cereal_type.lower() in object_raw.obj_type.lower() for cereal_type in cereal_types):
                navigate_to(table_pose)
                if demo_mode == simulated_robot:
                    obj_type = ObjectType.BOWL
                else:
                    obj_type = "Metalbowl"
                table_obj = DetectAction(technique='all').resolve().perform()
                bowl = next((value for value in table_obj.values() if value.obj_type == obj_type), None)
            if bowl:
                handled_cereal = True
                angle = 115
                if robot.get_pose().pose.position.x > bowl.pose.position.x:
                    direction = "right"
                else:
                    direction = "left"
                PouringAction([bowl.pose], [Arms.LEFT], [direction], [angle]).resolve().perform()
                park = park_arms()
                while not park:
                    print("waiting for park")
                    rospy.sleep(0.1)
                giskard.update_from_giskard(robot, park)
                navigate_to(table_pose_pre)
                placeorpark(object.name, object, grasp, talk_bool, original_pose, pick_table_link, False)

            else:
                navigate_to(table_pose_to_shelf, interrupt_bool=False)
                navigate_to(rotated_shelf_pose, interrupt_bool=False)
                navigate_to(shelf_pose, interrupt_bool=False)
                giskard.sync_worlds()
                place_pose, link = find_pose_in_shelf(group, object_raw, groups_in_shelf)
                placeorpark(object.name, object, grasp, talk_bool, place_pose, link, False)
            demo(2)

        print()


# previous_value = fts.get_last_value()
#
# monitor_func_place()
demo(2)

