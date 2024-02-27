#! /usr/bin/env python
"""The ros node for the resource manager."""

from threading import Event, Condition, RLock  # , Lock, Semaphore
from os import walk
from os.path import isfile, join, splitext
# from runpy import run_path
# from types import TypeType
# import imp

import rospy
from rospkg import RosPack

from architecture_msgs.msg import Role as RoleMessage   # pylint: disable=import-error
from architecture_msgs.srv import ResourceRequest, ResourceRequestRequest  # pylint: disable=import-error

# from resource_manager.agent_base import Agent
from resource_manager.resource_manager_base import ResourceManagerError, \
                                                   ResourceManagerBase
# from pdb import set_trace as pause

class ResourceManagerNode(object):
    """ROS node that acts as the Resource Manager."""
    def __init__(self, rate=10, manager=None):
        rospy.init_node("resource_manager", log_level=rospy.DEBUG)

        if manager:
            self.manager = manager
        else:
            self.manager = ResourceManagerBase()

        rm_dir = RosPack().get_path("resource_manager")
        default_path = rm_dir + "/src/resource_manager/resources.yaml"
        file_path = rospy.get_param('~resources_path', default_path)

        self.manager.initialize(self, file_path)
        self.manager.check_ready()

        self.queue_available = Condition()
        self.prelims_done = Event()
        self.postlims_done = Event()
        self.lock = RLock()

        self.update_topic = rospy.get_param('~get_resources_topic',
                                            'update')
        rospy.Service(
            rospy.get_param('~request_resources_topic', 'request_resources'),
            ResourceRequest,
            self.allocate_resources
        )
        rospy.Service(
            rospy.get_param("~resource_return_topic", "resource_return"),
            ResourceRequest,
            self.deallocate_resources
        )

        self.rate = rospy.Rate(rate)
        self.request_queue = []

    @staticmethod
    def load_agents():
        """Load the agents specified in the ros param server to a dict."""
        rm_path = RosPack().get_path("resource_manager")
        config_path = rospy.get_param("~agents_path", rm_path + "/config")
        try:
            _, _, file_names = next(walk(config_path))
        except StopIteration as error:
            raise ResourceManagerError("Agent config filepath was not usable. "
                                       + "Failed with StopIteration error "
                                       + "message \"{}\" and filepath \"{}\""
                                       .format(error.message, config_path))
        modules = dict()
        modules['__builtins__'] = globals()['__builtins__']
        for file_name in file_names:
            if splitext(file_name)[1] == ".py":
                try:
                    execfile(join(config_path, file_name), modules)
                except SyntaxError:
                    pass
                modules.update(modules)
        # to_del = []
        # for key, value in modules.iteritems():
        #     if not type(value) is TypeType or not issubclass(value, Agent):
        #         to_del.append(key)
        # for key in to_del:
        #     del modules[key]
        return modules

    def find_behaviors(self):
        """Find all the default and specified behavior configurations."""
        only_specified = rospy.get_param("~only_specified", False)
        paths = self._get_specified_paths()

        behaviors = self._find_specified_behaviors(paths)
        if not only_specified:
            behaviors.extend(self._find_default_behaviors())

        return behaviors

    @staticmethod
    def _get_specified_paths():
        """Retrieve the paths specified in the ROS param server."""
        # TODO: Change expected value to dictionary of lists of paths
        #       Then the param server can be added to in multiple launch files
        rm_path = RosPack().get_path("resource_manager")
        return rospy.get_param("~behavior_paths", default=[rm_path + "/config"])

    def _find_specified_behaviors(self, paths):
        """Find the specified behavior configurations at each path in paths."""
        behavior_modules = []
        for path in paths:
            if isfile(path):
                behavior_modules.append(self._get_behavior_config(path))
            else:
                behavior_modules.extend(self._get_behavior_configs(path))

        return behavior_modules

    def _find_default_behaviors(self):
        """Find the default behavior configurations."""
        rpk = RosPack()
        behavior_modules = []

        exception_packages = rospy.get_param('~exception_packages',
                                             default=['resource_manager'])
        packages = rpk.list()

        for package in packages:
            if package not in exception_packages:
                path = rpk.get_path(package) + '/config'
                behavior_modules.extend(self._get_behavior_configs(path))

        return behavior_modules

    def _get_behavior_configs(self, path):
        """Extract the behavior configuration modules from the files at path."""
        behavior_modules = []
        try:
            _, _, file_names = next(walk(path))
        except StopIteration:
            file_names = []

        for file_name in file_names:
            if splitext(file_name)[1] == ".config":
                file_path = join(path, file_name)
                behavior_modules.append(self._get_behavior_config(file_path))

        return behavior_modules

    @staticmethod
    def _get_behavior_config(path):
        """Extract the behavior configuration module from the file at path."""
        module = dict()
        module['__builtins__'] = globals()['__builtins__']
        try:
            # TODO: Switch this to use runpy.run_path instead of execfile, maybe
            execfile(path, module)
        except SyntaxError:
            pass
        return module

    def update_resources(self, behavior_name, roles):
        """Call the ROS service to update the resources of "behavior_name"."""
        service_name = behavior_name + '/' + self.update_topic
        request = ResourceRequestRequest()
        request.behavior_id = behavior_name
        request.roles = []
        for role_name, resources in roles.iteritems():
            role_message = RoleMessage(name=role_name,
                                       required=False,
                                       priority=0)
            for _, resource in resources.iteritems():
                role_message.resources.append(resource.to_msg())
            request.roles.append(role_message)

        rospy.logdebug("Updating %s with new roles:\n%s",
                       behavior_name,
                       request.roles)
        try:
            service = rospy.ServiceProxy(service_name, ResourceRequest)
            service.wait_for_service(timeout=1)
            return service(request)
        except rospy.ServiceException as err:
            rospy.logwarn(err.message)
        return None

    def allocate_resources(self, request):
        """ROS Srv handler for allocating resources."""
        # info("allocate request:\n%s", request)[{"name":
        return self._resources_request(request, True)

    def deallocate_resources(self, request):
        """ROS Srv handler for deallocating resources."""
        # info("deallocate_resources(node):\n%s", request)
        return self._resources_request(request, False)

    def _resources_request(self, request, allocate):
        """Either allocates or deallocates depending on allocate flag."""
        success = True
        with self.queue_available:
            if request.behavior_id and request.roles:
                self.request_queue.append((request, allocate))
            else:
                return {'success': False, 'roles': []}

        self.prelims_done.wait()
        if allocate:
            try:
                assigned_roles = self.manager.get_assignment(request,
                                                             allocate=allocate)
            except ResourceManagerError:
                success = False
            except Exception as exception:
                self.manager.postlims(request, allocate=allocate)
                raise exception
            else:
                if assigned_roles:
                    for req_role in request.roles:
                        for req_resource in req_role.resources:
                            assigned_resource = \
                                assigned_roles[req_role.name][req_resource.name]
                            req_resource.group_name = \
                                assigned_resource.group_name
                else:
                    request.roles = []

        self.manager.postlims(request, allocate=allocate)
        return {'success':success, 'roles':request.roles if success else []}

    def run(self):  # pylint: disable=no-self-use
        """Main function for the node."""
        while not rospy.is_shutdown():
            self.manager.run()
            self.rate.sleep()
