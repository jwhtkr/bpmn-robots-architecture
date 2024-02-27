"""
Config file for a general robot

By including this module, or running it, you should have access to the
config variables/data for a robot.

A resource group config file should give mapping values between a general term
for data that needs to be accessed to specific robot-specific topics, services,
etc. to access that data.
e.g.:
"position" -> "odom"

Because its a python script it could even return a dict of function handles
that, if called (e.g. dict["position"]()), return the value(s) desired.
"""
from pdb import set_trace as pause

import rospy
from tf import TransformListener
from tf import Exception as TFException
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, PoseStamped

from resource_manager.agent_base import Agent, AgentInformationError  # pylint: disable=import-error

# This particular file should, for now, just return a position from a
# specified simulated robot.


class RobotAgent(Agent):
    """
    Class to represent a general robot.
    """

    def __init__(self, name, resources, namespace):
        """Update get_dict to include correct identifier:function pairs."""
        super(RobotAgent, self).__init__(name, resources, namespace)
        self.get_dict.update({
            "position": self.get_position,
            "position_stamped": self.get_position_stamped
            })
        self.capacity = 3
        self._odom = None
        self._position_stamped = None
        rospy.Subscriber(self.namespace + "/odom",
                         Odometry,
                         self.odom_callback,
                         queue_size=1)
        self.path_pub = rospy.Publisher("/robot_paths",
                                        Marker,
                                        queue_size=10)
        self.tf_listener = TransformListener()

    def freeze_info(self):
        """Retrieve and freeze the info available for the agent."""
        # rospy.logdebug("%s.freeze_info()", self.namespace)
        odom = self.get_curr_odom()
        if not odom:
            raise AgentInformationError
        position = PointStamped()
        position.header = odom.header
        position.point = odom.pose.pose.position
        try:
            self.tf_listener.waitForTransform("/map",
                                              position.header.frame_id,
                                              position.header.stamp,
                                              rospy.Duration(1))
            self._position_stamped = self.tf_listener.transformPoint("/map",
                                                                     position)
        except TFException as err:
            rospy.logwarn(err.message)
            raise AgentInformationError(err.message)

    def get_curr_odom(self):
        """Get the current odom value."""
        if not self._odom:
            self._odom = rospy.wait_for_message(self.namespace + "/odom",
                                                Odometry,
                                                timeout=0.1)
        return self._odom

    def odom_callback(self, msg):
        """Save odom message."""
        self._odom = msg

    def get_position(self):
        """Return the position of the robot."""
        return self._position_stamped.point

    def get_position_stamped(self):
        """
        Return the position of the robot with header information.
        """
        return self._position_stamped

    def publish_path(self):
        """Publish the assigned path of this robot."""
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time()
        marker.ns = self.namespace
        marker.id = 0
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.color.a = 1.
        marker.color.g = 1.
        marker.pose.orientation.w = 1.
        marker.scale.x = 0.01
        self.freeze_info()
        marker.points.append(self.get_position())
        for role in self.path:
            goal = role.behavior.goal
            marker.points.append(goal.point)
        self.path_pub.publish(marker)


class TurtlebotAgent(RobotAgent):
    """Represent a (simple) Turtlebot."""
    def __init__(self, name, resources, namespace):
        super(TurtlebotAgent, self).__init__(name, resources, namespace)
        self.get_dict.update({"max_speed":self.get_max_speed})
        self._max_speed = 1.0

    def get_max_speed(self):  # pylint: disable=no-self-use
        """Return robot's max speed"""
        # rospy.logdebug("Getting %s's max speed %s",
        #                self.namespace,
        #                self._max_speed)
        return self._max_speed


class TurtlebotAgentFast(TurtlebotAgent):
    """Fast Turtlebot."""
    pass


class TurtlebotAgentSlow(TurtlebotAgent):
    """Slow Turtlebot."""
    def __init__(self, name, resources, namespace):
        super(TurtlebotAgentSlow, self).__init__(name, resources, namespace)
        self._max_speed = 0.5


class QuadcopterAgent(RobotAgent):
    """Represent a (simple) quadcopter."""
    def __init__(self, name, resources, namespace):
        super(QuadcopterAgent, self).__init__(name, resources, namespace)
        self.get_dict.update({"max_speed":self.get_max_speed})

    def get_max_speed(self):  # pylint: disable=no-self-use
        """Return robot's max speed"""
        return 5
