from typing import Union

import rospy
from pycram_msgs.msg import ObjectIdentifierArray, ObjectIdentifier


class ObjectObserver:
    """
    Publishes an Array of visualization marker which represent the situation in the World
    """

    blocked_objects = ObjectIdentifierArray()

    def __init__(self, topic_name="/pycram/multirobot/blocked_objects", interval=0.1):
        """
        The Publisher creates an Array of Visualization marker with a Marker for each link of each Object in the
        World. This Array is published with a rate of interval.

        :param topic_name: The name of the topic to which the Visualization Marker should be published.
        :param interval: The interval at which the visualization marker should be published, in seconds.
        """
        self.topic_name = topic_name
        self.interval = interval

        self.sub = rospy.Subscriber(self.topic_name, ObjectIdentifierArray, queue_size=10, callback=self._cb)
        self.pub = rospy.Publisher(self.topic_name, ObjectIdentifierArray, queue_size=10)

    def block_object(self, object_desig):
        """
        Add an object to the observer list
        """
        obj = ObjectIdentifier()
        obj.name = object_desig.world_object.name
        obj.id = object_desig.world_object.id

        obj_array = ObjectIdentifierArray()
        obj_array.objects = self.blocked_objects.objects
        obj_array.objects.append(obj)

        # TODO: publish is not correct?
        self.pub.publish(obj_array)

    def release_object(self, object_desig):
        """
        Remove an object from the observer List
        """
        blocked: ObjectIdentifierArray = self.blocked_objects

        new_objects = [item for item in blocked.objects if item.id != object_desig.world_object.id]
        blocked.objects = new_objects

        self.blocked_objects = blocked
        self.pub.publish(blocked)

    def is_object_blocked(self, object_desig) -> bool:
        """
        State if a given object is blocked,
        """
        all_ids = [obj.id for obj in self.blocked_objects.objects]

        if object_desig.world_object.id in all_ids:
            return True

        return False

    def _cb(self, data: ObjectIdentifierArray) -> None:
        """
        Update list of blocked objects by given data
        """
        self.blocked_objects = data
