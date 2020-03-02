#!/usr/bin/env python
"""Provides all the functionality needed for the Behavior Listener."""

from __future__ import unicode_literals

from enum import Enum

import rospy
from architecture_msgs.msg import Behavior  # pylint: disable=import-error

from bpmn import CamundaRESTInteraction


class Status(Enum):
    """An enum that describes the status of a Task."""

    NEW = 1
    CANCELED = 2
    LOCKED = 3


def dict_subtract(dict_a, dict_b):
    """Subtract b from a based on the keys."""
    result = {k: v for k, v in dict_a.items() if k not in dict_b}
    return result


class BehaviorListener(CamundaRESTInteraction):
    """Provides the non-ROS functionality of the Behavior Listener."""

    # BASE_URI = "http://localhost:8080/engine-rest/"

    def __init__(self, base_uri=CamundaRESTInteraction.BASE_URI,
                 encoding='utf-8'):
        """Initialize BehaviorListener."""
        super(BehaviorListener, self).__init__(base_uri, encoding)
        self._prev_tasks = {}           # {id:(status, task)}
        self._curr_tasks = {}           # {id:(status, task)}
        self._curr_unlocked_tasks = {}  # {id:task}
        self._curr_locked_tasks = {}    # {id:task}
        self._curr_canceled_tasks = []
        self._prev_canceled_tasks = []
        self._curr_new_tasks = []

    @property
    def canceled_tasks(self):
        """Create a property to make curr_canceled_tasks private."""
        return self._curr_canceled_tasks

    @property
    def new_tasks(self):
        """Get the list of tasks that are new."""
        return self._curr_new_tasks

    def _update_unlocked_tasks(self):
        """Get the list of tasks that are unlocked."""
        body = {"notLocked": True}
        tasks = self.request_post(body, request="external-task")
        self._curr_unlocked_tasks = {task[u'id']: task for task in tasks}

    def _update_locked_tasks(self):
        """Get the list of tasks that are locked."""
        body = {"locked": True}
        tasks = self.request_post(body, request="external-task")
        self._curr_locked_tasks = {task[u'id']: task for task in tasks}

    def update_tasks(self):
        """Update the internally stored tasks."""
        self._update_unlocked_tasks()
        self._update_locked_tasks()

        self._update_new_tasks()
        self._update_canceled_tasks()

        self._prev_tasks = self._curr_tasks

    def _update_canceled_tasks(self):
        """Update the list of tasks(and _curr_tasks) that are canceled."""
        self._curr_canceled_tasks = []

        for pcanceled_task in self._prev_canceled_tasks:
            self._prev_tasks.pop(pcanceled_task[u'id'])

        for prev_task_id, status_task in self._prev_tasks.iteritems():
            if not (prev_task_id in self._curr_locked_tasks
                    or prev_task_id in self._curr_unlocked_tasks):
                self._curr_tasks[prev_task_id] = (u'canceled', status_task[1])
                self._curr_canceled_tasks.append(status_task[1])

        self._prev_canceled_tasks = self._curr_canceled_tasks

    def _update_new_tasks(self):
        """Update the list of tasks(and _curr_tasks) that are new."""
        self._curr_new_tasks = []

        for utask_id, utask in self._curr_unlocked_tasks.iteritems():
            if utask_id not in self._prev_tasks:
                self._curr_tasks[utask_id] = (u'new', utask)
                self._curr_new_tasks.append(utask)
            else:
                self._curr_tasks[utask_id] = (u'unlocked', utask)

        for ltask_id, ltask in self._curr_locked_tasks.iteritems():
            if ltask_id not in self._prev_tasks:
                self._curr_tasks[ltask_id] = (u'new', ltask)
                self._curr_new_tasks.append(ltask)
            else:
                self._curr_tasks[ltask_id] = (u'locked', ltask)


class BehaviorListenerNode(object):
    """ROS node to convey the Behavior Listener information."""

    def __init__(self, rate=10):
        """Initialize BehaviorListenerNode."""
        self.listener = BehaviorListener()
        rospy.init_node('behavior_listener')
        self.behavior_pub = rospy.Publisher("behaviors",
                                            Behavior,
                                            queue_size=10,
                                            latch=True)
        self.rate = rospy.Rate(rate)

    def run(self):
        """Perform the logic of the Behavior Listener."""
        rospy.sleep(1.5)
        while not rospy.is_shutdown():
            self.listener.update_tasks()

            for task in self.listener.new_tasks:
                self.behavior_pub.publish(topic=task[u'topicName'],
                                          name=task[u'activityId'],
                                          instance_id=task[u'id'].replace('-',
                                                                          '_'),
                                          status=Behavior.NEW)

            for task in self.listener.canceled_tasks:
                self.behavior_pub.publish(topic=task[u'topicName'],
                                          name=task[u'activityId'],
                                          instance_id=task[u'id'].replace('-',
                                                                          '_'),
                                          status=Behavior.CANCELED)

            self.rate.sleep()


if __name__ == "__main__":
    BehaviorListenerNode(10).run()
