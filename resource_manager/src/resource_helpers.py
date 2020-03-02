"""Helper classes for the Resource Manager"""


class ResourceLock(object):
    """Represents the important info from the lock/request of a Resource"""
    def __init__(self,
                 behavior_name,
                 behavior_priority,
                 role_name,
                 role_priority,
                 resource_priority,
                 required):
        self.behavior_name = behavior_name
        self.behavior_priority = behavior_priority
        self.role_name = role_name
        self.role_priority = role_priority
        self.resource_priority = resource_priority
        self.required = required

    def __repr__(self):
        string = ("ResourceLock(b_name={!r}, r_name={!r},"
                  + " priority={!r}, required={!r})")
        return string.format(self.behavior_name,
                             self.role_name,
                             (self.behavior_priority,
                              self.role_priority,
                              self.resource_priority),
                             self.required)

    def __str__(self):
        return "{!s}:{!s} {!s}, {!s}".format(self.behavior_name,
                                             self.role_name,
                                             (self.behavior_priority,
                                              self.role_priority,
                                              self.resource_priority),
                                             self.required)

    def __eq__(self, other):
        """Override equality function"""
        if isinstance(self, type(other)):
            return (self.behavior_name == other.behavior_name
                    and self.role_name == other.role_name)
        return NotImplemented

    def __ne__(self, other):
        """Override non-equality function"""
        x = self.__eq__(other)
        if x is not NotImplemented:
            return not x
        return NotImplemented


    @property
    def priority(self):
        """The combined priority based on the 3 internal priorities"""
        return self.combine_priorities()

    def combine_priorities(self):
        """combines the priorities in a simple, unweighted linear fashion"""
        return (self.behavior_priority
                + self.role_priority
                + self.resource_priority)


class Resource(object):
    """The representation of a resource"""
    def __init__(self, group_name, name, category, resource_type):
        self.group_name = group_name
        self.name = name
        self.category = category
        self.type = resource_type
        self._request = None    # ResourceLock if requested, None otherwise
        self._lock = None       # ResourceLock if locked, None otherwise

    def __repr__(self):
        string = "Resource(g_name={!r}, name={!r}, category={!r}, "\
                 + "type={!r}, request={!r}, lock={!r})"
        return string.format(self.group_name, self.name, self.category,
                             self.type, self._request, self._lock)

    def __str__(self):
        return "{!s}: {{{!s}:{!s}}}".format(self.name,
                                            self.category,
                                            self.type)

    def __eq__(self, other):
        """Override equality function"""
        if isinstance(self, type(other)):
            return (self.name == other.name
                    and self.category == other.category
                    and self.type == other.type)
        return NotImplemented

    def __ne__(self, other):
        """Override non-equality function"""
        x = self.__eq__(other)
        if x is not NotImplemented:
            return not x
        return NotImplemented

    @property
    def is_requested(self):
        """Returns a boolean representing the request state"""
        return bool(self._request)

    @property
    def is_locked(self):
        """Returns a boolean representing the lock state"""
        return bool(self._lock)

    @property
    def priority(self):
        """Returns the request/lock priority or -1 if not requested/locked"""
        # requested priority takes precedence because if it's both requested and
        # locked the requested priority is always greater than the locked
        # priority.
        if self.is_requested:
            return self._request.priority
        elif self.is_locked:
            return self._lock.priority
        return -1

    def request(self, request):
        """Requests the resource if possible, and returns a success boolean"""
        if not(self.is_locked or self.is_requested):
            self._request = request
            return True
        # TODO: Update logic to include optionality and priority!
        return False

    def lock(self, lock):
        """Locks the resource if appropriate and returns a success boolean"""
        if self._request == lock:
            self._lock = lock
            self._request = None
            return True
        return False

    def un_request(self, request):
        """If the requests match un-request the resource, else exception"""
        if self._request == request:
            self._request = None
        else:
            raise ValueError("The request obj requesting a release of request"
                             + " does not match the held request obj")

    def un_lock(self, lock):
        """If the locks match unlock the resource, else raise an exception"""
        if self._lock == lock:
            self._lock = None
        else:
            raise ValueError("The lock obj requesting an unlock of a resource"
                             + " does not match the held lock obj")


class ResourceGroup(object):
    """Encapsulates resources that should be grouped together for any reason"""
    def __init__(self, name, resources):
        self.name = name
        self._resources = {}

        # resources stored as {resource_name: Resource}
        if not isinstance(resources, dict):
            raise TypeError("resources must be passed in as a dict type")
        self._build_resources(resources)

        self._category_type_resources = {}   # {category: {type: [Resources]}}
        # self._type_resources = {}       # {resource_type: [Resources]}

        self._build_category_type_resources()
        # self._build_type_resources()

    def __repr__(self):
        strings = []
        for resource in self._resources.itervalues():
            strings.append(resource.__repr__())
        resources = "[" + ", ".join(strings) + "]"
        return "ResourceGroup(name={!r}, resources={!r})".format(self.name,
                                                                 resources)

    def __str__(self):
        strings = []
        for resource in self._resources.itervalues():
            strings.append(resource.__str__())
        resources = "[" + ", ".join(strings) + "]"
        return "{!s}:{!s}".format(self.name, resources)

    @property
    def resources(self):
        """Resources property to return the list of resources in the group"""
        return self._resources.values()

    def get_resource(self, resource_name):
        """Gets and individual resource by name"""
        return self._resources[resource_name]

    def get_resources(self, resource_category, resource_type, sort=False):
        """Gets a list of resources belonging to a certain category and type"""
        resources = self._category_type_resources.get(resource_category, {})
        resources = resources.get(resource_type, [])
        if sort:
            return sorted(resources, key=lambda resource: resource.priority)
        return resources

    # def get_resources_type(self, resource_type, sort=False):
    #     """Gets a list of resources that have a certain type"""
    #     resources = self._type_resources[resource_type]
    #     if sort:
    #         return sorted(resources, key=lambda resource: resource.priority)
    #     return resources

    def get_resources_behavior(self, behavior_name):
        """Gets a list of the resources that are locked by a behavior"""
        resources = []
        for resource in self._resources.itervalues():
            if (resource.is_locked
                    and behavior_name == resource.lock.behavior_name):
                resources.append(resource)

    def _build_category_type_resources(self):
        """Sorts the resources by category, then type, and stores them"""
        for resource in self._resources.itervalues():
            if resource.category not in self._category_type_resources:
                self._category_type_resources[resource.category] = \
                    {resource.type: [resource]}
            else:
                category_dict = self._category_type_resources[resource.category]
                if resource.type not in category_dict:
                    category_dict[resource.type] = [resource]
                else:
                    category_dict[resource.type].append(resource)

    # def _build_type_resources(self):
    #     """Sorts the resources by type and stores them"""
    #     for resource in self._resources.itervalues():
    #         type_list = self._type_resources.get(resource.resource_type, [])
    #         type_list.append(resource)

    def _build_resources(self, resources):
        """Createst the resources dict (with Resource objects) from a dict"""
        for resource_name, resource in resources.iteritems():
            self._resources[resource_name] = Resource(self.name,
                                                      resource_name,
                                                      resource['category'],
                                                      resource['type'])
