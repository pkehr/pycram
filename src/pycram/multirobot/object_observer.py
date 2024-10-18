import threading
import time

import rospy


class ObjectObserver:
    """
    Class to observe the state of objects that are currently in use and shouldn't be accessed by another robot
    """

    blocked_objects = {}
    """
    Variable that stores the array of objects
    """

    def __init__(self):
        """
        Currently does nothing
        """

    def block_object(self, object_desig, robot_name):
        """
        Add an object to the observer list
        """

        obj = {"robot": robot_name,
               "name": object_desig.name}

        self.blocked_objects[object_desig.id] = obj

    def release_object(self, object_desig):
        """
        Remove an object from the observer list
        """

        self.blocked_objects.pop(object_desig.id)

    def is_object_blocked(self, object_desig) -> bool:
        """
        State if a given object is blocked,

        :param object_desig: designator of given object
        """
        all_ids = list(self.blocked_objects.keys())

        if object_desig.id in all_ids:
            return True

        return False
