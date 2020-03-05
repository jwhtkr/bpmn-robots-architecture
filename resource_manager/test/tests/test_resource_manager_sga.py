#!/usr/bin/env python
from resource_manager.resource_manager_sga import ResourceManagerSGA  # pylint: disable=import-error, no-name-in-module
from resource_manager.resource_manager_base import ResourceManagerBase, ResourceManagerError  # pylint: disable=import-error, no-name-in-module
from resource_manager.resource_manager_node import ResourceManagerNode  # pylint: disable=import-error, no-name-in-module
from architecture_msgs.srv import ResourceRequest, ResourceRequestResponse
from architecture_msgs.msg import Role, Resource
from rospkg import RosPack
import rostest
import rospy
import unittest
import sys
import copy
PKG = "resource_manager"


class ResourceManagerSGATests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Start the resource manager in the setup to prevent topics from being
        # redefined. This will require additional setup in the tests though.
        cls.node = ResourceManagerNode(
            manager=ResourceManagerSGA(infix="", index=1))
        # Create a ResourceRequest service request for testing
        cls.test_request = ResourceRequest()
        cls.test_request.behavior_id = "behavior_1"
        cls.test_request.priority = 10
        cls.test_request.roles = []
        cls.test_request.roles.append(Role("role_1",
                                           True,
                                           10,
                                           [Resource("",
                                                     "resource_1",
                                                     "mobility",
                                                     "2d_unicycle",
                                                     True,
                                                     10),
                                            Resource("",
                                                     "resource_2",
                                                     "sensors",
                                                     "infrared_camera",
                                                     True,
                                                     10)]))
        cls.test_request.roles.append(Role("role_2",
                                           True,
                                           10,
                                           [Resource("",
                                                     "resource_1",
                                                     "mobility",
                                                     "3d_quad_rotor",
                                                     True,
                                                     10),
                                            Resource("",
                                                     "resource_2",
                                                     "sensors",
                                                     "gps",
                                                     True,
                                                     10)]))
        rospy.Service('/behavior_1/update', ResourceRequest,
                      cls.update_resources_response)
        rospy.Service('/behavior_2/update', ResourceRequest,
                      cls.update_resources_response)

    def setUp(self):
        """
        Ran before every test, makes sure there is nothing left over from 
        previous tests.
        """
        self.node.manager._roles = []
        self.node.manager._agents = []
        self.node.manager._behavior_modules = []
        self.node.manager._resources_groups = []
        self.node.manager._resources_behaviors = {}
        self.node.manager.resources_ready = False
        self.node.manager._request_count = 0
        self.node.manager._request_dict = {}
        if rospy.has_param("~behavior_paths"):
            rospy.delete_param("~behavior_paths")

    @classmethod
    def update_resources_response(self, req):
        res = ResourceRequestResponse()
        res.success = True
        res.roles = req.roles
        return res

    def test_load_agents(self):
        """
        Load the config files manually in the test and compare them with the
        output of load_agents.
        """
        # Specify the resource manager config folder
        path = RosPack().get_path("resource_manager")
        rospy.set_param("~agents_path", path + "/config")
        # Load the config files into a dict
        modules = dict()
        modules["__builtins__"] = globals()["__builtins__"]
        execfile(path + "/config" + "/robot.py", modules)
        execfile(path + "/config" + "/server.py", modules)

        test_modules = self.node.load_agents()

        # Compare the size of the two dicts
        self.assertEqual(len(modules), len(test_modules))
        # Compare the items in each of the dicts
        self.assertTrue(self.compare_dictionaries(
            modules, test_modules), 'Dictionaries do not match')

    def test_get_specified_paths(self):
        """
        Confirm that get_specified_paths is finding the path in the param server.
        """
        # Add the parameter required
        path = RosPack().get_path("resource_manager")
        rospy.set_param("~behavior_paths", path)
        # Confirm that the function found the right path
        self.assertEqual(self.node._get_specified_paths(), path)

    def test_find_specified_behaviors(self):
        """
        Test that the behaviors can be loaded from a path from the param server.
        """
        path = RosPack().get_path("resource_manager") + "/config"
        # Load the behaviors in a dict
        modules = dict()
        modules["__builtins__"] = globals()["__builtins__"]
        execfile(path + "/test_behaviors.config", modules)
        # Run the function
        test_modules = self.node._find_specified_behaviors(
            self.node._get_specified_paths())
        # Compare the size of the two dicts
        self.assertEqual(len(modules), len(test_modules[0]))
        # Compare the items in each of the dicts
        self.assertTrue(self.compare_dictionaries(
            modules, test_modules[0]), 'Dictionaries do not match')

    def test_find_default_behaviors(self):
        """
        Test that unspecified behaviors can be loaded from default paths.
        """
        # Allow the test files to be found
        rospy.set_param("~exception_packages", "")
        path = RosPack().get_path("resource_manager") + \
            "/config" + "/test_behaviors.config"
        # Load the behaviors into a dict
        modules = dict()
        modules["__builtins__"] = globals()["__builtins__"]
        execfile(path, modules)
        # Run the function
        test_modules = self.node._find_default_behaviors()

        # Compare the size of the two dicts
        self.assertEqual(len(test_modules[0]), len(modules),)
        # Compare the items in each of the dicts
        self.assertTrue(self.compare_dictionaries(
            modules, test_modules[0]), 'Dictionaries do not match')

    def test_find_default_behaviors_empty(self):
        """
        Show that the function correctly does not load the test files.
        """
        # Reset the parameter
        if rospy.has_param("~exception_packages"):
            rospy.delete_param("~exception_packages")
        # Run the function
        test_modules = self.node._find_default_behaviors()
        # Check for an empty list
        self.assertEqual(test_modules, [])

    def test_get_behavior_configs(self):
        """
        Confirm that the overarching find behavior function(that uses the others) is working as intended.
        """
        # Load the two expected modules into a dictionary
        path = RosPack().get_path("resource_manager") + "/test/behaviors"
        modules = [{}, {}]
        modules[0]["__builtins__"] = globals()["__builtins__"]
        modules[1]["__builtins__"] = globals()["__builtins__"]
        execfile(path + "/test_behaviors.config", modules[0])
        execfile(path + "/test_behaviors2.config", modules[1])
        # Run the function
        test_modules = self.node._get_behavior_configs(path)
        # Check that the two expected modules were loaded correctly
        for dict1, dict2 in zip(modules, test_modules):
            self.assertTrue(self.compare_dictionaries(
                dict1, dict2), 'Dictionaries do not match')

    def test_get_behavior_config(self):
        """
        Check that behaviors are being correctly loaded from a file path.
        """
        # Set up the filepath
        path = RosPack().get_path("resource_manager") + \
            "/config" + "/test_behaviors.config"
        # Load the behavior files
        modules = dict()
        modules["__builtins__"] = globals()["__builtins__"]
        execfile(path, modules)
        # Run the function
        test_modules = self.node._get_behavior_config(path)
        # Verify the behaviors were loaded correctly
        self.assertEqual(len(test_modules), len(modules))
        self.assertTrue(self.compare_dictionaries(
            modules, test_modules), 'Dictionaries do not match')

    def test_load_agents_sga(self):
        """
        Verify that agents can be loaded correctly.
        """
        # Reset the param
        if rospy.has_param("~agents_path"):
            rospy.delete_param("~agents_path")
        # Set the param
        path = RosPack().get_path("resource_manager")
        rospy.set_param("~agents_path", path + "/config")
        self.node.manager.load_resources(
            path+'/src/resource_manager/resources.yaml')
        # Run the function
        agents = self.node.manager.load_agents()
        # Check rigorously (lol) that the agents were loaded
        self.assertEqual(len(agents), 16,
                         "Failed to load agents or the resource file changed.")

    def test_build_behavior(self):
        """
        Test that behaviors taken from ResourceRequest service calls 
        are successfully made into Behavior objects under ideal conditions. 
        """
        # Load the default behaviors
        self.node.manager._behavior_modules = self.node.find_behaviors()
        # Run the function with the service request defined in SetUpClass
        test_behavior = self.node.manager._build_behavior(self.test_request.behavior_id,
                                                          self.test_request.priority,
                                                          self.test_request.roles)
        # Check each behavior to assure they are equal
        self.assertEqual(test_behavior.behavior_id,
                         self.test_request.behavior_id)
        self.assertEqual(test_behavior.priority, self.test_request.priority)
        self.assertEqual(sorted(test_behavior.roles.keys())[
                         0], self.test_request.roles[0].name)
        self.assertEqual(sorted(test_behavior.roles.keys())[
                         1], self.test_request.roles[1].name)

    def test_add_roles(self):
        """
        Test that roles from the Resource Request service call can be added to the list
        within the SGA class under ideal conditions.
        """
        # Load the default behaviors
        self.node.manager._behavior_modules = self.node.find_behaviors()
        # Reset the roles then add them from the service request
        self.node.manager._roles = []
        self.node.manager._add_roles(self.test_request.behavior_id,
                                     self.test_request.priority,
                                     self.test_request.roles)
        # Check each role to ensure they are equal length because
        # order is not consistant so comparing them is difficult
        self.assertEqual(len(self.node.manager._roles),
                         len(self.test_request.roles))

    def test_remove_roles(self):
        """
        Confirm that removing roles actually removes them from the manager.
        """
        # Load default behaviors
        self.node.manager._behavior_modules = self.node.find_behaviors()
        self.node.manager._roles = []
        # Add the roles so they can be deleted
        self.node.manager._add_roles(self.test_request.behavior_id,
                                     self.test_request.priority,
                                     self.test_request.roles)
        # Verify the roles were added
        self.assertEqual(len(self.node.manager._roles),
                         len(self.test_request.roles))
        # Delete the roles
        self.node.manager._remove_roles(self.test_request.behavior_id,
                                        self.test_request.priority,
                                        self.test_request.roles)
        # Verify they were deleted
        self.assertEqual(len(self.node.manager._roles), 0)

    def test_prelims_allocate(self):
        """
        Test to see if prelims can add a role and increase the request count.
        """
        # Load default behaviors
        self.node.manager._behavior_modules = self.node.find_behaviors()
        self.node.manager._roles = []
        # Add the roles
        self.node.manager.prelims(self.test_request, allocate=True)
        # Check each role to ensure they are equal length because
        # order is not consistant so comparing them is difficult
        self.assertEqual(len(self.node.manager._roles),
                         len(self.test_request.roles))
        self.assertEqual(self.node.manager._request_count, 1)
        self.assertEqual(len(self.node.manager._request_dict), 1)
        # Also check that the request was successfully added to the list
        self.assertEqual(self.node.manager._request_count, 1)
        self.assertTrue(self.node.manager._request_dict.get(
            self.test_request.behavior_id))

    def test_prelims_deallocate(self):
        """
        Test to see if prelims can remove roles and decrease the request count.
        """
        # Load default behaviors
        self.node.manager._behavior_modules = self.node.find_behaviors()
        self.node.manager._roles = []
        # Add the roles so they can be deleted
        self.node.manager.prelims(self.test_request, allocate=True)
        # Delete the roles
        self.node.manager.prelims(self.test_request, allocate=False)
        # Verify they were deleted
        self.assertEqual(len(self.node.manager._roles), 0)

    def test_postlims(self):
        """
        Check to see if postlims correctly removes the request from the dictionary
        and also decrements the count.
        """
        # Load default behaviors
        self.node.manager._behavior_modules = self.node.find_behaviors()
        self.node.manager._roles = []
        # Add the roles
        self.node.manager.prelims(self.test_request, allocate=True)
        # Run postlims
        self.node.manager.postlims(self.test_request, allocate=True)
        self.assertEqual(len(self.node.manager._request_dict), 0)
        self.assertEqual(self.node.manager._request_count, 0)

    def test_SGA_single_request_one_role(self):
        """
        Check to see if the Sequential Greedy Algorithm works with a single request and one role.
        """
        # Set up resource manager
        path = RosPack().get_path("resource_manager")
        self.node.manager.initialize(
            self.node, path+"/src/resource_manager/resources.yaml")
        # Make the request
        single_request = copy.deepcopy(self.test_request)
        single_request.roles = [self.test_request.roles[0]]
        self.node.manager.prelims(single_request, allocate=True)
        self.node.manager.update()
        assigned_roles = self.node.manager.get_assignment(single_request,
                                                          allocate=True)
        # Verify the correct assignment
        self.assertEqual(assigned_roles['role_1']['resource_1'].group_name,
                         "robot_11")

    def test_SGA_single_request_two_roles(self):
        """
        Check to see if the Sequential Greedy Algorithm works with two roles in a behavior.
        """
        # Set up resource manager
        path = RosPack().get_path("resource_manager")
        self.node.manager.initialize(
            self.node, path+"/src/resource_manager/resources.yaml")
        # Make the request
        assigned_roles = self.resource_request(
            self.test_request, allocate=True)
        # Verify that the assignment was made correctly
        self.assertEqual(assigned_roles['role_1']['resource_1'].group_name,
                         "robot_11")
        self.assertEqual(assigned_roles['role_2']['resource_1'].group_name,
                         "drone_11")

    def test_SGA_double_request_same_resources(self):
        """
        Test to ensure resources are not being assigned to different simultaneous roles.
        """
        # Set up resource manager
        path = RosPack().get_path("resource_manager")
        self.node.manager.initialize(
            self.node, path+"/src/resource_manager/resources.yaml")
        # Make the first request
        request = copy.deepcopy(self.test_request)
        first_assigned_roles = self.resource_request(request, allocate=True)
        # Make the second request
        request.behavior_id = "behavior_2"
        second_assigned_roles = self.resource_request(request, allocate=True)
        # Verify that the assignments are not the same. This tests for double assignment of a resource
        self.assertNotEqual(first_assigned_roles['role_1']['resource_1'].group_name,
                            second_assigned_roles['role_1']['resource_1'].group_name,
                            "Double allocation of resources")
        self.assertNotEqual(first_assigned_roles['role_2']['resource_1'].group_name,
                            second_assigned_roles['role_2']['resource_1'].group_name,
                            "Double allocation of resources")

    def test_SGA_return_all_roles(self):
        """
        Verify that returning resources allows those resources to be assigned again.
        """
        # Set up resource manager
        path = RosPack().get_path("resource_manager")
        self.node.manager.initialize(
            self.node, path+"/src/resource_manager/resources.yaml")
        # Make the first allocation request
        first_assigned_roles = self.resource_request(
            self.test_request, allocate=True)
        # Make the deallocation request
        self.resource_request(self.test_request, allocate=False)
        # Make the second identical allocation request
        second_assigned_roles = self.resource_request(
            self.test_request, allocate=True)
        # Verify that the results are the same
        self.assertEqual(first_assigned_roles['role_1']['resource_1'].group_name,
                         second_assigned_roles['role_1']['resource_1'].group_name)

    def test_SGA_return_partial_roles_same_behavior(self):
        """
        Verify that when partial resources are returned they can be requested again by the same behavior.
        The manager should return all the requested resources.
        """
        # Set up resource manager
        path = RosPack().get_path("resource_manager")
        self.node.manager.initialize(
            self.node, path+"/src/resource_manager/resources.yaml")
        # Make the first allocation request
        first_assigned_roles = self.resource_request(
            self.test_request, allocate=True)
        # Make the deallocation request, copy the original request so it only requests
        # one role to be deallocated
        request = copy.deepcopy(self.test_request)
        request.roles.pop()
        self.resource_request(request, allocate=False)
        # Make second allocation request,
        second_assigned_roles = self.resource_request(
            self.test_request, allocate=True)
        # The manager should return the same resources for duplicate assignments for the same
        # behavior. This checks for this.
        self.assertEqual(first_assigned_roles['role_1']['resource_1'].group_name,
                         second_assigned_roles['role_1']['resource_1'].group_name)
        self.assertEqual(first_assigned_roles['role_2']['resource_1'].group_name,
                         first_assigned_roles['role_2']['resource_1'].group_name)

    def test_SGA_return_partial_roles_different_behavior(self):
        """
        Verify that when partial resources are returned that a different behavior
        requesting resources only receieves the resources that were returned.
        """
        # Set up resource manager
        path = RosPack().get_path("resource_manager")
        self.node.manager.initialize(
            self.node, path+"/src/resource_manager/resources.yaml")
        # Make the first allocation request
        behavior1_roles = self.resource_request(
            self.test_request, allocate=True)
        # Make the deallocation request, copy the original request so it only requests
        # one role to be deallocated
        pop_request = copy.deepcopy(self.test_request)
        pop_request.roles.pop()
        self.resource_request(pop_request, allocate=False)
        # Make second allocation request with a 'different' behavior
        request = copy.deepcopy(self.test_request)
        request.behavior_id = 'behavior_2'
        behavior2_roles = self.resource_request(
            request, allocate=True)
        # Test to see if the resources associated with role_1 were successfully deallocated.
        # If they were deallocated, they should have been reallocated to behavior2
        self.assertEqual(behavior1_roles['role_1']['resource_1'].group_name,
                         behavior2_roles['role_1']['resource_1'].group_name)
        # Test to see if the resources that were not request to be deallocated were
        # not deallocated.
        self.assertNotEqual(behavior1_roles['role_2']['resource_1'].group_name,
                            behavior2_roles['role_2']['resource_1'].group_name)

    def test_multiple_requests_same_behavior(self):
        """
        Make sure more resources are not being assigned to the same behavior.
        """
        # Set up resource manager
        path = RosPack().get_path("resource_manager")
        self.node.manager.initialize(
            self.node, path+"/src/resource_manager/resources.yaml")
        
        responses = []

        for _ in range(5):
            response = self.resource_request(self.test_request, allocate=True)
            responses.append(response)
        group_name = responses[0]['role_1']['resource_1'].group_name
        for response in responses:
            self.assertEqual(group_name, response['role_1']['resource_1'].group_name)



    def resource_request(self, request, allocate):
        """
        Helper method to submit a request to the manager.
        """
        assigned_roles = {}
        self.node.manager.prelims(request, allocate=allocate)
        self.node.manager.update()
        if allocate:
            assigned_roles = self.node.manager.get_assignment(request,
                                                              allocate=allocate)
        self.node.manager.postlims(request, allocate=allocate)

        return assigned_roles

    def compare_dictionaries(self, modules, loaded_modules):
        is_equal = True
        for item1, item2 in zip(modules, loaded_modules):
            if item1 != item2:
                is_equal = False
        return is_equal


if __name__ == "__main__":
    rostest.rosrun(PKG, "ResourceManagerSGATests", ResourceManagerSGATests)
