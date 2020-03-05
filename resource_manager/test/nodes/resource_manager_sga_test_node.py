#! /usr/bin/env python
from resource_manager.resource_manager_sga import ResourceManagerSGA    # pylint: disable=import-error, no-name-in-module
from resource_manager.resource_manager_node import ResourceManagerNode  # pylint: disable=import-error, no-name-in-module
from resource_manager.resource_helpers import ResourceLock              # pylint: disable=import-error, no-name-in-module
from architecture_msgs.srv import ClearManager
import rospy
from rospkg import RosPack
class ResourceManagerTestNode(ResourceManagerNode):
    """Subclass that adds reset functionality."""
    def __init__(self, rate=10, manager=None):
        super(ResourceManagerTestNode, self).__init__(rate, manager)
        
        rospy.Service('clear_manager', ClearManager,
                      self.clear_manager)
        self.rm_dir = RosPack().get_path("resource_manager")
        self.default_path = self.rm_dir + "/src/resource_manager/resources.yaml"
        self.file_path = rospy.get_param('~resources_path', self.default_path)

        
    def clear_manager(self, req):
        self.manager._roles = []
        self.manager._resources_behaviors = {}
        self.manager.resources_ready = True
        self.manager._request_count = 0
        self.manager._request_dict = {}
        self.request_queue = []
        return []



if __name__ == "__main__":
    ResourceManagerTestNode(manager=ResourceManagerSGA(infix='', index=1)).run()