#!/usr/bin/env python
from resource_manager.resource_manager_base import ResourceManagerSimple, ResourceManagerError # pylint: disable=import-error, no-name-in-module
from resource_manager.resource_manager_node import ResourceManagerNode # pylint: disable=import-error, no-name-in-module
from architecture_msgs.srv import ResourceRequest, ResourceRequestResponse
from architecture_msgs.msg import Role, Resource
from copy import deepcopy
import rostest
import rospy
import unittest

PKG = "resource_manager"

class ResourceManagerSimpleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node = ResourceManagerNode(
            manager=ResourceManagerSimple(infix="", index=1))
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

    @classmethod
    def update_resources_response(self, req):
        res = ResourceRequestResponse()
        res.success = True
        res.roles = req.roles
        return res

    def setUp(self):
        """
        Ran before every test, makes sure there is nothing left over from 
        previous tests.
        """
        self.node.manager._resources_behaviors = {}
        self.node.manager._request_count = 0
        self.node.manager._request_dict = {}
        for group in self.node.manager._resources_groups:
            for resource in group.resources:
                resource.un_lock(resource._lock)


    def test_prelims_correct(self):
        """
        Check to see if prelims are successfully adding the roles to be processed.
        """
        self.node.manager.prelims(self.test_request, allocate=True)
        self.assertTrue(self.node.manager._request_dict['behavior_1'])
        

    def test_prelims_multiple_assignments(self):
        """
        Ensure multiple prelims are added to the request dictionary
        """
        for i in range(3):
            request = deepcopy(self.test_request)
            request.behavior_id = self.test_request.behavior_id + str(i)
            self.node.manager.prelims(request, allocate=True)
            temp = self.node.manager.get_assignment(request, allocate=True)
        
        self.assertTrue(self.node.manager._request_dict, 3)


    def test_get_assignment_success(self):
        """
        Tests to see if an assignment can be recieved after successful prelims.
        """
        self.node.manager.prelims(self.test_request, allocate=True)
        roles = self.node.manager.get_assignment(self.test_request, allocate=True)
        self.assertEqual(self.node.manager._resources_behaviors['behavior_1']['role_1']['resource_1'][0].group_name,
                         roles['role_1']['resource_1'].group_name)

    def test_get_assignment_none_made(self):
        """
        Test whether or not the get_assignment call raises an exception when called without
        first making an assignment.
        """
        self.assertRaises(ResourceManagerError,self.node.manager.get_assignment, self.test_request, allocate=True)


if __name__ == "__main__":
    rostest.rosrun(PKG, "ResourceManagerSimpleTests", ResourceManagerSimpleTests)