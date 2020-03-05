#! /usr/bin/env python
"""Uses the Sequential Greedy Algorithm to provide the resource allocation."""
# from pdb import set_trace as pause
from copy import copy
from pdb import set_trace as pause

import numpy as np

from rospy import logdebug as debug
from rospy import loginfo as info
from rospy import logwarn as warn
from rospy import ServiceException as ROSServiceException

from resource_manager.resource_manager_base import ResourceManagerBase, \
                                                   ResourceManagerError
from resource_manager.resource_manager_node import ResourceManagerNode
from resource_manager.resource_helpers import ResourceLock
from resource_manager.agent_base import AgentInformationError

class ResourceManagerSGA(ResourceManagerBase):
    """Uses SGA to allocate resources."""
    def __init__(self, infix='_', index=0):
        super(ResourceManagerSGA, self).__init__(infix=infix, index=index)
        self._roles = []
        self._prev_resources_behaviors = {}
        self._agents = []
        self._behavior_modules = []
        self._capacity = 0

    def initialize(self, context, resources_path):
        super(ResourceManagerSGA, self).initialize(context, resources_path)
        self._agents = self.load_agents()
        self._behavior_modules = self.context.find_behaviors()
        self._capacity = self._calc_capacity()

    def load_agents(self):
        """Load the available agents to a list based on the config files."""
        agents = []
        agent_classes = self.context.load_agents()
        for resource_group in self._resources_groups:
            try:
                agent_class = agent_classes[resource_group.agent_type]
            except KeyError:
                raise ResourceManagerError("{} is not an available agent class"
                                           .format(resource_group.agent_type))
            agent = agent_class(resource_group.name,
                                resource_group.resources,
                                resource_group.name)
            agents.append(agent)
        return agents

    def run(self):
        super(ResourceManagerSGA, self).run(is_sequential=False)
        for agent in self._agents:
            try:
                agent.publish_path()
            except AttributeError:
                pass

    def prelims(self, request, **kwargs):
        super(ResourceManagerSGA, self).prelims(request, **kwargs)
        if kwargs.get("allocate"):
            try:
                self._add_roles(request.behavior_id,
                                request.priority,
                                request.roles)
            except ResourceManagerError:
                pass
        else:
            self._remove_roles(request.behavior_id,
                               request.priority,
                               request.roles)

    def update(self, **kwargs):
        if self._needs_update():
            info("Updating assignment")
            self._clear_assignment()
            self._clear_locks()
            self._get_info()
            self.sequential_greedy()
            self._update_behaviors()
        else:
            self._adjust_behavior_assignments()

    def _needs_update(self):    # pylint: disable=no-self-use
        """Determine if the assignment needs to be updated or not."""
        # for role in self._roles:
        #     if role not in self._prev_roles:
        #         return True

        # for role in self._roles:
        #     for agent in self._agents:
        #         if role in agent.path:
        #             break
        #     else:
        #         for agent in self._agents:
        #             if len(agent.path) < agent.capacity:
        #                 if role.subset_of(agent):
        #                     return True

        return True     # for now the assignment should always be updated

    def _adjust_behavior_assignments(self):
        """Change assignments to match completion statuses if needed."""
        pass    # not worrying about this for now

    def _clear_assignment(self):
        """Clear the currrently stored assignment"""
        self._prev_resources_behaviors.clear()
        self._prev_resources_behaviors.update(self._resources_behaviors)
        self._resources_behaviors.clear()
        for agent in self._agents:
            agent.batch = []
            agent.path = []

    def _clear_locks(self):
        """Clear the locks of the resources."""
        for agent in self._agents:
            for resource in agent.resources:
                resource._lock = None
                resource._request = None

    def _get_info(self):
        """Signal all the agents and roles to get up-to-date info."""
        # debug("SGA._get_info()")
        for role in self._roles:
            try:
                role.freeze_info()
            except ROSServiceException as err:
                warn(err.message)
                self._remove_roles(role.behavior.behavior_id,
                                   role.behavior.priority,
                                   [role])
        for agent in self._agents:
            try:
                agent.freeze_info()
            except AgentInformationError:
                self._agents.remove(agent)

    def sequential_greedy(self):
        """Perform the Sequential Greedy Algorithm from the CBBA paper."""
        debug("Doing SGA for %s",
              [role.behavior.behavior_id + "/" + role.name
               for role in self._roles])
        n_assgn = self._calc_n_assgn()
        assgn = []
        scores = np.zeros((n_assgn+1, len(self._agents), len(self._roles)))
        self._generate_scores(scores[0])

        for n in range(n_assgn):  # pylint: disable=invalid-name
            i_star, j_star = np.unravel_index(np.argmax(scores[n]),
                                              scores[n].shape)
            if scores[n, i_star, j_star] > 0:
                assgn.append((i_star, j_star))
            else:
                break

            self._update_path(i_star, j_star)
            self._update_batch(i_star, j_star)
            self._update_scores(scores, i_star, j_star, n)

        self._update_assignment()

    def _calc_n_assgn(self):
        """Calculate the maximum number of assignments that could be made."""
        return min(self._capacity, len(self._roles))

    def _calc_capacity(self):
        """Calculate the total capacity of roles the agents could fulfill."""
        capacity = 0
        for agent in self._agents:
            capacity += agent.capacity
        return capacity

    def _generate_scores(self, scores):
        """Calculate the starting scores for the agents and roles."""
        for i, agent in enumerate(self._agents):
            for j, role in enumerate(self._roles):
                if role.subset_of(agent):
                    _, scores[i, j] = self._marginal_score(role, agent)
                else:
                    scores[i, j] = 0

    def _update_batch(self, i_star, j_star):
        """Add the j_star role to the end of the batch of the i_star agent."""
        self._agents[i_star].batch.append(self._roles[j_star])

    def _update_path(self, i_star, j_star):
        """Optimally insert the j_star role in the path of the i_star agent."""
        agent = self._agents[i_star]
        role = self._roles[j_star]
        index, _ = self._marginal_score(role, agent)
        agent.path.insert(index, role)

    def _update_scores(self, scores, i_star, j_star, iteration):
        """Update the scores for the next iteration of assignment."""
        scores[iteration+1] = np.copy(scores[iteration])
        agent = self._agents[i_star]
        for j in range(scores[iteration+1].shape[1]):
            role = self._roles[j]
            if role.subset_of(agent):
                _, score = self._marginal_score(role, agent)
                scores[iteration+1, i_star, j] = np.min((score,
                                                         scores[iteration,
                                                                i_star,
                                                                j]))
            else:
                scores[iteration+1, i_star, j] = 0

        scores[iteration+1, :, j_star] = 0

    def _marginal_score(self, role, agent):
        """Calculate a marginal score of the agent performing the role"""
        # Check capacity first
        if len(agent.path) == agent.capacity:
            # If the agent's path is at full capacity
            return None, 0
        elif len(agent.path) > agent.capacity:
            raise ResourceManagerError("{}'s capacity was exceeded"
                                       .format(agent.namespace))

        # debug("Calculating marginal score of %s performing %s/%s",
        #      agent.namespace,
        #      role.behavior.behavior_id,
        #      role.name)
        marginal_scores = []
        path = agent.path
        batch = agent.batch
        candidate_batch = copy(batch)
        candidate_batch.append(role)
        path_scores = self._score_path(agent, path, batch)

        for i in range(len(path)+1):
            candidate_path = copy(path)
            candidate_path.insert(i, role)
            candidate_path_scores = self._score_path(agent,
                                                     candidate_path,
                                                     candidate_batch)
            # debug("%s scores: %s",
            #      [role.name
            #       for role in candidate_path] if candidate_path else [],
            #      candidate_path_scores)
            marginal_score = sum(candidate_path_scores) - sum(path_scores)
            marginal_scores.append(marginal_score)

        return max(enumerate(marginal_scores), key=lambda x: x[1])

    @staticmethod
    def _score_path(agent, path, batch):
        """Score the elements of the path (and batch). Returns a score list."""
        scores = []
        if path:
            for role in path:
                scores.append(role.score(agent, batch, path))
        return scores
        # return [role.score(agent, batch, path)
        #         for role in path] if path else []


    def _update_assignment(self):
        """Update the internal assignment to reflect the results of SGA."""
        for agent in self._agents:
            try:
                str_path = str([role.name for role in agent.path])
                debug("%s's path is %s", agent.namespace, str_path)

                role = agent.path[0]
                roles = self._resources_behaviors.get(role.behavior.behavior_id,
                                                      {})
                roles[role.name] = self._match_resources(agent, role)
                self._resources_behaviors[role.behavior.behavior_id] = roles

                debug("%s/%s is assigned to %s",
                      role.behavior.behavior_id, role.name, agent.namespace)
            except IndexError:
                pass
        # self._update_behaviors()

    def _match_resources(self, agent, role):
        """Match resources for the assignment."""
        resources = {}
        for rsrc in role.resources:
            compatible_rsrc_list = agent.get_resources(rsrc.category,
                                                       rsrc.type)
            resources[rsrc.name] = self._match_resource(compatible_rsrc_list,
                                                        rsrc,
                                                        role)
        return resources

    def _match_resource(self, compatible_resources, req_resource, role):    # pylint: disable=no-self-use
        """Match an individual resource for assignment."""
        for compatible_rsrc in compatible_resources:
            if not compatible_rsrc.is_locked:
                request = ResourceLock(role.behavior.behavior_id,
                                       role.behavior.priority,
                                       role.name,
                                       role.priority,
                                       req_resource.priority,
                                       role.required)
                compatible_rsrc.request(request)
                compatible_rsrc.lock(request)
                req_resource.group_name = compatible_rsrc.group_name
                return req_resource
        return None

    def _update_behaviors(self):
        """Call the behavior services to update their assignments."""
        # pause()
        # Find all the roles that were assigned that are now not assigned.
        # These should call the service to update with empty.
        were_assigned = \
            self._assignment_differences(self._resources_behaviors,
                                         self._prev_resources_behaviors)
        for behavior_id in were_assigned:
            self.context.update_resources(behavior_id, {})

        # Find all the roles that are now assigned something that were not
        # previously assigned. These should call the service to update
        updated_behaviors = \
            self._assignment_differences(self._prev_resources_behaviors,
                                         self._resources_behaviors)
        for behavior_id, roles in updated_behaviors.iteritems():
            self.context.update_resources(behavior_id, roles)

    @staticmethod
    def _assignment_differences(ref, diff):
        """Find the entries in diff that are not in ref."""
        differences = {}
        for behavior_id, roles in diff.iteritems():
            ref_roles = ref.get(behavior_id, {})

            for role_name, resources in roles.iteritems():
                ref_resources = ref_roles.get(role_name, {})

                if resources != ref_resources:
                    differences[behavior_id] = {role_name:resources}
        return differences

    def get_assignment(self, request, **kwargs):
        assgn = {}
        try:
            assgn = self._resources_behaviors[request.behavior_id]
            assgn = {role.name:assgn[role.name] for role in request.roles}
        except KeyError:
            pass
            # raise ResourceManagerError
        return assgn

    def _add_roles(self, behavior_name, behavior_priority, roles):
        """Create and add the roles to the roles list."""
        debug("Adding %s %s", behavior_name, [role.name for role in roles])
        behavior = self._build_behavior(behavior_name,
                                        behavior_priority,
                                        roles)
        if behavior is not None:
            for role in behavior.roles.itervalues():
                for m_role in self._roles:
                    if m_role.behavior.behavior_id == role.behavior.behavior_id\
                        and m_role.name == role.name:

                        break
                else:
                    self._roles.append(role)

    def _build_behavior(self, beh_name, beh_priority, roles):
        """Construct a behavior with the given name, priority, and roles."""
        behavior = None
        for beh_mod in self._behavior_modules:
            try:
                build_behavior = beh_mod['build_behavior']
                behavior = build_behavior(beh_name, beh_priority, roles)
                break
            except (KeyError, ValueError, TypeError):
                pass
        else:
            raise ResourceManagerError("There was no loaded behavior "
                                       + "configuration module that matched the"
                                       + " calling behavior.")
        return behavior

    def _remove_roles(self, behavior_name, behavior_priority, roles):   # pylint: disable=unused-argument
        """Remove the roles from the roles list."""
        debug("Removing %s %s", behavior_name, [role.name for role in roles])
        for del_role in roles:
            del_ind = None
            for i, stored_role in enumerate(self._roles):
                if stored_role.behavior.behavior_id == behavior_name \
                    and del_role.name == stored_role.name:

                    del_ind = i
                    break
            self._roles.pop(del_ind)
            self._resources_behaviors[behavior_name].pop(del_role.name)


if __name__ == "__main__":
    ResourceManagerNode(manager=ResourceManagerSGA(infix='', index=1)).run()
