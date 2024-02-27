#!/usr/bin/env python
"""This is a module with the base class for resource managers"""

from threading import RLock

import yaml
import rospy
# from rospy import logdebug as debug
from rospy import loginfo as info

from architecture_msgs.srv import ResourceRequest  # pylint: disable=import-error

from resource_helpers import ResourceGroup, ResourceLock


class ResourceManagerError(Exception):
    """Exception indicating an error in the ResourceManager"""
    pass


class ResourceManagerBase(object):
    """Base class for all resource managers. Simplistic, but functional."""
    def __init__(self, infix='_', index=0):
        """Reads in resources, and their groupings from a YAML file"""
        self._resources_file = ""
        self._resources_groups = []
        self._resources_behaviors = {}
        self._lock = RLock()
        self.ready = False
        self.infix = str(infix)
        self.index = int(index)
        if self.index < 0:
            raise ValueError("index must be non-negative")

    def __repr__(self):
        return self._resources_groups.__repr__()

    def set_file_path(self, file_path):
        """Sets the resources file to file_path and builds the resources"""
        self._resources_file = file(file_path, 'r')
        info("Getting resources from %s", file_path)
        self._build_resources()

    def _build_resources(self):
        """Builds the internal list of resources from the file"""
        resources_dict = yaml.load(self._resources_file)
        if not isinstance(resources_dict, dict):
            raise TypeError("{} is improperly formated"
                            .format(self._resources_file.name))
        for resource_group_name, resource_group in resources_dict.iteritems():
            resources = resource_group['resources']
            for i in xrange(resource_group['multiplicity']):
                name = resource_group_name + self.infix + str(i + self.index)
                self._resources_groups.append(ResourceGroup(name, resources))
        self.ready = True

    def allocate_resources(self, roles, name, priority):
        """Given a list of roles, allocates appropriate resources to them"""
        if not self.ready:
            raise ResourceManagerError("The resource manager is not yet ready. "
                                       + "Make sure to set the file path before"
                                       + " using.")
        # info("allocate_resources:\n%s,\n%s,\n%s", roles, name, priority)
        info("Allocating Resources")
        self._lock.acquire()
        resources_roles = {}
        for role in roles:
            resources_roles[role.name] = self._request_resources(role,
                                                                 name,
                                                                 priority)
            info("\tallocating to %s: %s",
                 role.name,
                 [str(rsrc)
                  for rsrc, _ in resources_roles[role.name].itervalues()])

        # debug("resources_dict: %s", resources)
        # debug("roles filled? %s", self._roles_filled(roles, resources))

        if self._roles_filled(roles, resources_roles):
            info("Roles filled, locking resources")
            self._lock_resources(resources_roles)
            self._resources_behaviors[name] = resources_roles
            self._lock.release()
            return resources_roles
        else:
            info("Roles not filled, releasing resources")
            for role, resource_dict in resources_roles.iteritems():
                # release resources if can't fill all
                self._un_request_resources(resource_dict.itervalues())
            self._lock.release()
            raise ResourceManagerError("Roles were not all filled.")

    def _request_resources(self, role, name, priority):
        """Given a role, with a list of resources needed, allocate resources"""
        # debug("_request_resources %s, %s, %s", role, name, priority)

        resources = {}
        for group in self._resources_groups:
            # info("checking group: %s", group)
            for resource in role.resources:
                # info("for resource:\n%s", resource)
                request = ResourceLock(name,
                                       priority,
                                       role.name,
                                       role.priority,
                                       resource.priority,
                                       resource.required)
                rsrc = self._request_resource(resource, request, group)
                if rsrc:
                    # if the resource was able to be filled/requested
                    # info("resource found")
                    if resource.name in resources:
                        self._un_request_resources(resources.itervalues())
                        err_str = "Resource names are not unique in role: {}."
                        raise ResourceManagerError(err_str.format(role))
                    resources[resource.name] = (rsrc, request)
                else:
                    # info("resource not found, about to unrequest:\n%s",
                    #      resources)
                    self._un_request_resources(resources.itervalues())
                    resources = {}
                    break
            else:
                break
        # debug("resources: %s", resources)
        return resources

    def _request_resource(self, requested_resource, request, group):
        """Find a resource to fill the request, otherwise return None"""
        # debug("_request_resource: %s, %s, %s",
        #       requested_resource,
        #       request,
        #       group)

        resources = group.get_resources(requested_resource.category,
                                        requested_resource.type,
                                        True)
        for resource in resources:
            self._lock.acquire()
            if resource.request(request):
                self._lock.release()
                return resource
            self._lock.release()
        return None

    def _roles_filled(self, roles, resources):
        """Checks if the roles are filled and returns a boolean"""
        # debug("_roles_filled:\n%s,\n%s", roles, resources)

        success = True
        for role in roles:
            success = success and self._role_filled(role, resources[role.name])
        return success

    def _role_filled(self, role, resources):
        """Checks if the role is properly filled and returns a boolean"""
        # debug("_role_filled:\n%s,\n%s", role, resources)

        success = True
        for resource in role.resources:
            success = success and self._has_resource(resource, resources)
        return success


    def _has_resource(self, resource, resources):  # pylint: disable=no-self-use
        """Determines whether or not the resources list contains resource"""
        # info("_has_resource:\n%s,\n%s", resource, resources)
        if resource.name in resources:
            rsrc, _ = resources[resource.name]
            if (rsrc.category == resource.category
                    and rsrc.type == resource.type):
                return True

        return False

    def _lock_resources(self, resources_roles):
        """Locks the resources given. Throws an exception if unsuccessful"""
        self._lock.acquire()
        for resources in resources_roles.itervalues():
            for resource, request in resources.itervalues():
                if not resource.lock(request):
                    # undo the locks that were just performed
                    for rsrc, _ in resources:
                        if rsrc is resource:
                            break
                        rsrc.un_lock(request)
                    self._lock.release()
                    raise ResourceManagerError("Unable to lock all resources")
        self._lock.release()

    def deallocate_resources(self, name, roles):
        """Deallocates the resources from the provided roles"""
        if not self.ready:
            raise ResourceManagerError("The resource manager is not yet ready. "
                                       + "Make sure to set the file path before"
                                       + " using.")
        # debug("deallocate_resources:\n%s\n%s", name, roles)
        info("Deallocating Resources")
        self._lock.acquire()
        resources = self._resources_behaviors[name]
        for role in roles:
            for resource in role.resources:
                _rsrc = resources[role.name].pop(resource.name)
                info("\tdeallocating: %s", _rsrc[0])
                self._un_lock_resources([_rsrc])
                if not resources[role.name]:
                    resources.pop(role.name)
        if not resources:
            self._resources_behaviors.pop(name)
        self._lock.release()
        # debug("%s", self._resources_behaviors)

    def _un_request_resources(self, resource_list):  # pylint: disable=no-self-use
        """Un-requests the resources in the list of (resource, lock) tuples"""
        self._lock.acquire()
        # info("_un_request_resources:\n%s", [rsrc for rsrc in resource_list])
        for resource, lock in resource_list:
            try:
                resource.un_request(lock)
            except ValueError:
                pass
        self._lock.release()

    def _un_lock_resources(self, resource_list):  # pylint: disable=no-self-use
        """Unlocks the resources in the list of (resource, lock) tuples"""
        self._lock.acquire()
        try:
            for resource, lock in resource_list:
                resource.un_lock(lock)
        except ValueError:
            raise ResourceManagerError("There was an error unlocking resources")
        self._lock.release()

class ResourceManagerNode(object):
    """ROS node that acts as the Resource Manager"""
    def __init__(self, rate=10, manager=None):
        rospy.init_node("resource_manager")
        default_path = '/home/justin/camunda_ws/src/resource_manager/src'
        file_path = rospy.get_param('~resources_path', default_path)

        if manager:
            self.manager = manager
        else:
            self.manager = ResourceManagerBase()

        self.manager.set_file_path(file_path)

        rospy.Service(rospy.get_param('~request_resources_topic',
                                      'request_resources'),
                      ResourceRequest,
                      self.allocate_resources)
        rospy.Service(rospy.get_param("~resource_return_topic",
                                      "resource_return"),
                      ResourceRequest,
                      self.deallocate_resources)

        self.rate = rospy.Rate(rate)

    def allocate_resources(self, request):
        """ROS Srv handler for allocating resources"""
        # info("allocate request:\n%s", request)
        try:
            resources = self.manager.allocate_resources(request.roles,
                                                        request.behavior_id,
                                                        request.priority)
            for role in request.roles:
                for return_resource in role.resources:
                    _rsrc = resources[role.name][return_resource.name][0]
                    return_resource.group_name = _rsrc.group_name
                    # return_rsrc.name = _rsrc.name
            # info("Returning: \n%s", request.roles)
            return {'success':True, 'roles':request.roles}
        except ResourceManagerError:
            return {'success':False, 'roles':[]}


    def deallocate_resources(self, request):
        """ROS Srv handler for deallocating resources"""
        # debug("deallocate_resources(node):\n%s", request)

        try:
            self.manager.deallocate_resources(request.behavior_id,
                                              request.roles)
            return {'success':True, 'roles':request.roles}
        except ResourceManagerError:
            return {'success':False, 'roles':request.roles}

    def run(self):  # pylint: disable=no-self-use
        """Main function for the node"""
        rospy.spin()

if __name__ == "__main__":
    # pylint: disable=line-too-long
    ResourceManagerNode(manager=ResourceManagerBase(infix='', index=1)).run()
