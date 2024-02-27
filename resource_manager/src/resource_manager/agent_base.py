"""Agent base class module"""
import rospy
from resource_manager.resource_helpers import ResourceGroup


class AgentInformationError(AttributeError):
    """Represent an error with getting the information related to an agent."""
    pass

class Agent(ResourceGroup):
    """Represent an agent and its allocation info and state."""

    def __init__(self, name, resources, namespace):
        super(Agent, self).__init__(name,
                                    {},
                                    agent_type=self.__class__.__name__)
        self.resources = resources
        try:
            rospy.init_node('agent', anonymous=True)
        except rospy.ROSException:
            pass  # this class is being instantiated where a node already exists
        self.namespace = namespace
        self.batch = []
        self.path = []
        self.get_dict = {}


    def __repr__(self):
        return "{}({})".format(self.__class__.__name__,
                               super(Agent, self).__repr__())

    def freeze_info(self):
        """Retrieve and freeze the info available for the agent."""
        pass

    def get(self, identifier):
        """
        Get the value(s) associated with the given identifier.

        @param identifier: a string value that specifies what value(s) to return
        @return the value(s) specified by the identifier.
        e.g. identifier="position" might return [x,y,z]

        Child class must add identifier:function pairs to the get_dict member!!
        """
        if identifier in self.get_dict:
            return self.get_dict[identifier]()
        raise NotImplementedError
