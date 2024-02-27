"""Role and Behavior Base Classes"""

from pdb import set_trace as pause

from resource_manager.resource_helpers import ResourceGroup
# from resource_manager.resource_manager_base import ResourceManagerError


def build_behavior(behavior_name, behavior_priority, roles):    # pylint: disable=unused-argument
    """
    Build a behavior based on the behavior name, priority, and roles.

    @param behavior_name: the unique identifier of the behavior instance
    @param behavior_priority: the priority for the instance of the behavior
    @param roles: the list of role messages from the request

    @return Behavior class instance corresponding to the behavior_name

    This function will be overridden in each config module. If the behavior_name
    corresponds to one of the behaviors in the config file it will attempt to
    instantiate an instance of that behavior object and return it. Else it will
    raise a ValueError exception
    """
    raise NotImplementedError


class Behavior(object):
    """Represent a Behavior and its allocation info."""
    def __init__(self, behaivor_id, behavior_priority, roles):
        if not self.is_instance(behaivor_id):
            raise TypeError
        self.behavior_id = behaivor_id
        self.priority = behavior_priority
        self.roles = self.to_dict(roles)

    @staticmethod
    def is_instance(behavior_id):
        """Check that the behaivor_id is an instance of this behavior type."""
        raise NotImplementedError

    def to_dict(self, roles):   # pylint: disable=no-self-use
        """Convert a list of role messages to a dict of role objects."""
        role_dict = {role.name:self.build_role(role) for role in roles}
        return role_dict

    def build_role(self, role_msg):
        """Build the proper role from the role message."""
        raise NotImplementedError

    def score(self, role, agent, batch, path):
        """
        Score the role, agent, batch/path combination, ie c_{i,j}[b/p].

        This is where any general behavior wide scores are calculated. I.e. if a
        score, or portion of a score, doesn't depend on which specific role is
        being filled, this function calculates that portion of the score.

        @param role: the role being scored
        @param agent the agent being scored
        @param batch: the "batch" of roles the agent is already assigned in the
                      order of assignment
        @param path: the roles the agent is already assigned in the order of
                     execution

        @return behavior level score
        """

        # will probably take a form like this:
        #
        # start = agent.get([state_identifiers])
        # _, marginal_score = role.transform_state(start)
        # tot_score = some_start_value + marginal_score
        # for prev_role in batch/path:
        #     end, score = prev_role.transform_state(start)
        #     tot_score += score
        #     start = end
        # score = do stuff with tot_score and/or marginal score
        # return score

        raise NotImplementedError

# Services the behavior provides for scoring purposes
# What "topics" to request info from a resource (robot) on


class Role(ResourceGroup):
    """Represent a Role and its allocation info and state."""
    def __init__(self,
                 behavior,
                 role_name=None,
                 role_priority=None,
                 role_required=False,
                 resources=None):
        if not self.is_instance(role_name):
            raise TypeError
        self.behavior = behavior
        self.priority = role_priority
        self.required = role_required
        self._resources_msgs = resources
        resources_dict = self.to_dict(resources)
        super(Role, self).__init__(role_name, resources_dict)

    def __repr__(self):
        return "{}({})".format(self.__class__.__name__,
                               super(Role, self).__repr__())

    @staticmethod
    def is_instance(role_name):
        """Check that the role_name is consistent with the represented role."""
        raise NotImplementedError

    def to_dict(self, resources):   # pylint: disable=no-self-use
        """Convert from a resources list to a resources dict."""
        resource_dict = {resource.name:{"category":resource.category,
                                        "group_name":resource.group_name,
                                        "name":resource.name,
                                        "priority":resource.priority,
                                        "required":resource.required,
                                        "type":resource.type}
                         for resource in resources}
        return resource_dict

    def freeze_info(self):
        """Retrieve and freeze the info needed for the score."""
        raise NotImplementedError

    def score(self, agent, batch, path):
        """
        Combine behavior level score with role level score.

        @param agent: the agent being scored
        @param batch: the roles the agent is already assigned in order of
                      assignment
        @param path: the roles the agent is already assigned in order of
                     execution

        @return the combined behavior and role score, and the corresponding
                index in the path
        """
        # Should always contain this call to the parent class
        # beh_score = self.behavior_context.score(self, agent, batch, path)

        # then any role-specific score modifications can be made
        # probably similar form to what's in the behavior score function ^^

        #return a final score, and index in path for that final score
        raise NotImplementedError

    def from_msg(self, msg):
        """Convert from a role message (or message_like) to this role."""
        self.name = msg.name
        self.priority = msg.priority
        self.required = msg.required
        resources = self.to_dict(msg.resources)
        self._build_resources(resources)
        self._build_category_type_resources()

    def to_msg(self):
        """Convert to a format compatible with the role message."""
        msg = {}
        msg['name'] = self.name
        msg['priority'] = self.priority
        msg['required'] = self.required
        for resource_msg in self._resources_msgs:
            group_name = self._resources[resource_msg.name].group_name
            resource_msg.group_name = group_name
        msg['resources'] = self._resources_msgs
        return msg

# What "topics" to request info from a resource (robot) on
