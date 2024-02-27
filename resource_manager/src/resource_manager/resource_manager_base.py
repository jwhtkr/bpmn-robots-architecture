#!/usr/bin/env python
"""The base class for resource managers and a simple implementation."""

import yaml
# import time
# import threading
from rospy import loginfo as info

from resource_manager.resource_helpers import ResourceGroup, ResourceLock


class ResourceManagerError(StandardError):
    """Exception indicating an error in the ResourceManager."""
    pass


class ResourceManagerBase(object):
    """Base class for all resource managers.."""
    def __init__(self, infix='_', index=0):
        """Reads in resources, and their groupings from a YAML file."""
        self._resources_groups = []
        self._resources_behaviors = {}
        self.resources_ready = False
        self.context_ready = False
        self.infix = str(infix)
        self.index = int(index)
        if self.index < 0:
            raise ValueError("index must be non-negative")
        self.context = None
        self._request_count = 0
        self._request_dict = {}
        self.postlims_just_done = False

    def __repr__(self):
        return self._resources_groups.__repr__()

    def check_ready(self):
        """Raises exception if there's no resources or context assigned."""
        if not(self.context_ready and self.resources_ready):
            raise ResourceManagerError("The resource manager is not yet ready. "
                                       + "Make sure to set the file path and "
                                       + "context before using.")

    def initialize(self, context, resources_path):
        """Initialize any needed information. Child classes may extend this."""
        self.context = context
        self.load_resources(resources_path)
        self.context_ready = True

    def load_resources(self, file_path):
        """Sets the resources file to file_path and builds the resources."""
        resources_file = file(file_path, 'r')
        info("Getting resources from %s", file_path)
        self._build_resources(resources_file)

    def _build_resources(self, resources_file):
        """Builds the internal list of resources from the file."""
        resources_dict = yaml.safe_load(resources_file)
        if not isinstance(resources_dict, dict):
            raise TypeError(
                "{} is improperly formated".format(resources_file.name))

        for resource_group_name, resource_group in resources_dict.iteritems():
            resources = resource_group['resources']
            agent_type = resource_group['agent_type']

            for i in xrange(resource_group['multiplicity']):
                name = resource_group_name + self.infix + str(i + self.index)
                self._resources_groups.append(
                    ResourceGroup(name, resources, agent_type=agent_type))

        self.resources_ready = True

    def prelims(self, request, **kwargs):     # pylint: disable=unused-argument
        """Do any needed preliminary work specific to one request."""
        info("prelims: %s", request.behavior_id)
        self._request_count += 1
        self._request_dict[request.behavior_id] = \
            {"roles":request.roles, "priority":request.priority}

    def postlims(self, request, **kwargs):     # pylint: disable=unused-argument
        """Do any needed post-liminary work specific to one request."""
        info("postlims: %s", request.behavior_id)
        self._request_count -= 1
        if request.behavior_id in self._request_dict:
            del self._request_dict[request.behavior_id]
        if self._request_count == 0:
            self.context.prelims_done.clear()
            self.context.postlims_done.set()

    def update(self, **kwargs):
        """Update the allocation/assignment based on the current state/info."""
        raise NotImplementedError

    def get_assignment(self, request, **kwargs):
        """
        Get the assignment state for the specified behavior.

        The resource manager node expects a return value that is a nested
        dictionary that can be accessed by [role_name][resource_name] to give
        a resource-like object, the important thing being that it has the
        correct resource group name. What the manager stores in
        self._resources_behaviors might look something like:
        {"behavior_id":
            {"role_name":
                {"resource_name":resource_object,
                .
                .
                .
                "resource_name":resource_object},
            .
            .
            .
            "role_name":
                {"resource_name":resource_object,
                .
                .
                .
                "resource_name":resource_object}
            },
            .
            .
            .
         "behavior_id":
            {"role_name":
                {"resource_name":resource_object,
                .
                .
                .
                "resource_name":resource_object},
            .
            .
            .
            "role_name":
                {"resource_name":resource_object,
                .
                .
                .
                "resource_name":resource_object}
            },
        }
        with this function returning the dictionary resulting from a call like:
        self._resources_behaviors["behavior_id"]
        """
        raise NotImplementedError

    def run(self, is_sequential=True):
        """Run the support structure and the algorithm once."""
        with self.context.queue_available:
            if self.context.request_queue:
                for request, allocate in self.context.request_queue:
                    self.prelims(request, allocate=allocate)

                self.context.request_queue = []
                self.context.prelims_done.set()
                self.context.postlims_done.wait()
                self.context.postlims_done.clear()
                if is_sequential:
                    self.context.queue_available.notify()
                else:
                    self.context.queue_available.notify_all()
                self.postlims_just_done = True
                return
        if self.postlims_just_done:
            self.update()
            self.postlims_just_done = False





class ResourceManagerSimple(ResourceManagerBase):
    """Simplistic, but functional resource manager."""

    def prelims(self, request, **kwargs):
        super(ResourceManagerSimple, self).prelims(request, **kwargs)
        if not kwargs["allocate"]:
            self.deallocate_resources(request.behavior_id, request.roles)
            self._request_dict[request.behavior_id]["roles"] = \
                {role.name:
                 {rsrc.name:rsrc
                  for rsrc in role.resources}
                 for role in request.roles}

    def update(self, **kwargs):
        pass

    def get_assignment(self, request, **kwargs):
        for (m_request_behavior_id, _) in self._request_dict.iteritems():
            if m_request_behavior_id == request.behavior_id:
                break
        else:
            raise ResourceManagerError
        try:
            roles = self.allocate_resources(request.roles,
                                            request.behavior_id,
                                            request.priority)
        except ResourceManagerError as exception:
            super(ResourceManagerSimple, self).prelims(request, **kwargs)
            raise exception
        roles = {role_name:
                 {rsrc_name:rsrc[0]
                  for rsrc_name, rsrc in role.iteritems()}
                 for role_name, role in roles.iteritems()}
        with self.context.lock:
            self._request_dict[request.behavior_id]["roles"] = roles
            info("get_assignment")
            if request.behavior_id in self._request_dict:
                return self._request_dict[request.behavior_id]["roles"]
            else:
                raise ResourceManagerError

    def run(self):
        super(ResourceManagerSimple, self).run(is_sequential=True)

    def allocate_resources(self, roles, name, priority):
        """Given a list of roles, allocates appropriate resources to them."""
        self.check_ready()
        # info("allocate_resources:\n%s,\n%s,\n%s", roles, name, priority)
        info("Allocating Resources")
        self.context.lock.acquire()
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
            self.context.lock.release()
            return resources_roles
        else:
            info("Roles not filled, releasing resources")
            for role, resource_dict in resources_roles.iteritems():
                # release resources if can't fill all
                self._un_request_resources(resource_dict.itervalues())
            self.context.lock.release()
            raise ResourceManagerError("Roles were not all filled.")

    def _request_resources(self, role, name, priority):
        """Given a role, with a list of resources needed, allocate resources."""
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
        """Find a resource to fill the request, otherwise return None."""
        # debug("_request_resource: %s, %s, %s",
        #       requested_resource,
        #       request,
        #       group)

        resources = group.get_resources(requested_resource.category,
                                        requested_resource.type,
                                        True)
        for resource in resources:
            self.context.lock.acquire()
            if resource.request(request):
                self.context.lock.release()
                return resource
            self.context.lock.release()
        return None

    def _roles_filled(self, roles, resources):
        """Checks if the roles are filled and returns a boolean."""
        # debug("_roles_filled:\n%s,\n%s", roles, resources)

        success = True
        for role in roles:
            success = success and self._role_filled(role, resources[role.name])
        return success

    def _role_filled(self, role, resources):
        """Checks if the role is properly filled and returns a boolean."""
        # debug("_role_filled:\n%s,\n%s", role, resources)

        success = True
        for resource in role.resources:
            success = success and self._has_resource(resource, resources)
        return success


    def _has_resource(self, resource, resources):  # pylint: disable=no-self-use
        """Determines whether or not the resources list contains resource."""
        # info("_has_resource:\n%s,\n%s", resource, resources)
        if resource.name in resources:
            rsrc, _ = resources[resource.name]
            if (rsrc.category == resource.category
                    and rsrc.type == resource.type):
                return True

        return False

    def _lock_resources(self, resources_roles):
        """Locks the resources given. Throws an exception if unsuccessful."""
        self.context.lock.acquire()
        for resources in resources_roles.itervalues():
            for resource, request in resources.itervalues():
                if not resource.lock(request):
                    # undo the locks that were just performed
                    for rsrc, _ in resources:
                        if rsrc is resource:
                            break
                        rsrc.un_lock(request)
                    self.context.lock.release()
                    raise ResourceManagerError("Unable to lock all resources")
        self.context.lock.release()

    def deallocate_resources(self, name, roles):
        """Deallocates the resources from the provided roles."""
        self.check_ready()
        # debug("deallocate_resources:\n%s\n%s", name, roles)
        info("Deallocating Resources")
        self.context.lock.acquire()
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
        self.context.lock.release()
        # debug("%s", self._resources_behaviors)

    def _un_request_resources(self, resource_list):  # pylint: disable=no-self-use
        """Un-requests the resources in the list of (resource, lock) tuples."""
        self.context.lock.acquire()
        # info("_un_request_resources:\n%s", [rsrc for rsrc in resource_list])
        for resource, lock in resource_list:
            try:
                resource.un_request(lock)
            except ValueError:
                pass
        self.context.lock.release()

    def _un_lock_resources(self, resource_list):  # pylint: disable=no-self-use
        """Unlocks the resources in the list of (resource, lock) tuples."""
        self.context.lock.acquire()
        try:
            for resource, lock in resource_list:
                resource.un_lock(lock)
        except ValueError:
            raise ResourceManagerError("There was an error unlocking resources")
        self.context.lock.release()



if __name__ == "__main__":
    # pylint: disable=line-too-long
    from resource_manager.resource_manager_node import ResourceManagerNode
    ResourceManagerNode(manager=ResourceManagerSimple(infix='', index=1)).run()
