from demos.pycram_turtle_testing.util import rotate
from demos.utils.enums import ENVIRONMENTS, ROBOTS
from demos.utils.object_spawner import set_environment, create_robot
from pycram.datastructures.enums import WorldMode
from pycram.datastructures.pose import Pose
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld

mode = WorldMode.DIRECT

world = BulletWorld(mode)
viz = VizMarkerPublisher() if mode == WorldMode.DIRECT else None

current_environment = set_environment(ENVIRONMENTS.SUTURO)

given_orientation = rotate(0)
turtle_pose = Pose(position=[3.5, 2.5, 0], orientation=given_orientation)
turtle = create_robot(ROBOTS.TURTLE, pose=turtle_pose)

print("done")
