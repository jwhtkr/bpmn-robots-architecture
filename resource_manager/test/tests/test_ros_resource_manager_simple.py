#!/usr/bin/env python
from resource_manager.resource_manager_sga import ResourceManagerSGA  # pylint: disable=import-error, no-name-in-module
from resource_manager.resource_manager_base import ResourceManagerBase, ResourceManagerError  # pylint: disable=import-error, no-name-in-module
from resource_manager.resource_manager_node import ResourceManagerNode  # pylint: disable=import-error, no-name-in-module
from architecture_msgs.srv import ResourceRequest, ResourceRequestResponse, ClearManager  # pylint: disable=import-error, no-name-in-module
from architecture_msgs.msg import Role, Resource
from rospkg import RosPack
from multiprocessing import Process, Queue
from time import sleep
import rostest
import roslaunch
import rospy
import unittest
import copy
import time
PKG = "resource_manager"


class ResourceManagerROSTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Start the resource manager in the setup to prevent topics from being
        # redefined. This will require additional setup in the tests though.
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
        request_service_name = rospy.get_param("request_resources_topic",
                                               "request_resources")
        return_service_name = rospy.get_param("resource_return_topic",
                                              "resource_return")
        clear_service_name = rospy.get_param("clear_manager_topic",
                                             "clear_manager")
        rospy.wait_for_service(return_service_name)
        rospy.wait_for_service(request_service_name)
        rospy.wait_for_service(clear_service_name)
        rospy.Service('/behavior_1/update', ResourceRequest,
                      cls.update_resources_response)
        rospy.Service('/behavior_2/update', ResourceRequest,
                      cls.update_resources_response)
        cls.request_resources = rospy.ServiceProxy(
            request_service_name, ResourceRequest)  # pylint: disable=line-too-long
        cls.resource_return = rospy.ServiceProxy(
            return_service_name, ResourceRequest)
        cls.clear_manager = rospy.ServiceProxy(
            clear_service_name, ClearManager)
        cls.queue = Queue()

    def setUp(self):
        self.clear_manager()

    @classmethod
    def update_resources_response(self, req):
        res = ResourceRequestResponse()
        res.success = True
        res.roles = req.roles
        return res

    def test_request_resources_partial_roles(self):
        """
        Test to see if the resources in the requested role are given a group_name.
        This is to ensure it got assigned.
        """
        response = self.request_resources(
            "behavior_1", 10, [self.test_request.roles[0]])
        self.assertEqual(len(response.roles), 1)

    def test_request_resources_all_roles(self):
        """
        Test to see if the resources in the requested roles are given a group_name.

        This is to ensure they got assigned. The testing is minimal because the algorithm
        gets tested in "test_resource_manager.py". 
        """
        response = self.request_resources(
            "behavior_1", 10, self.test_request.roles)
        for role in response.roles:
            for resource in role.resources:
                self.assertNotEqual(resource.group_name, '')

    def test_return_resources_all_roles(self):
        """
        Test to see if resources are reassigned after being deallocated.
        """
        response1 = self.request_resources(
            "behavior_1", 10, self.test_request.roles)
        self.resource_return("behavior_1", 10, self.test_request.roles)
        response2 = self.request_resources(
            "behavior_2", 10, self.test_request.roles)
        for role1, role2 in zip(response1.roles, response2.roles):
            for resource1, resource2 in zip(role1.resources, role2.resources):
                self.assertEqual(resource1, resource2)

    def test_return_resources_partial_roles(self):
        """
        Verify that resources are being assigned even if the entire behavior did not get de-allocated
        """
        response1 = self.request_resources(
            "behavior_1", 10, self.test_request.roles)
        self.resource_return("behavior_1", 10, [self.test_request.roles[0]])
        response2 = self.request_resources(
            "behavior_2", 10, self.test_request.roles)
        for resource1, resource2 in zip(response1.roles[0].resources, response2.roles[0].resources):
            self.assertEqual(resource1, resource2)
        for resource1, resource2 in zip(response1.roles[1].resources, response2.roles[1].resources):
            self.assertNotEqual(resource1, resource2)

    def test_request_no_roles_given(self):
        """
        Verify that a resource request fails when no roles are given.
        """
        response1 = self.request_resources(
                "behavior_1", 10, [])
        self.assertFalse(response1.success)

    def test_request_bad_behavior_no_roles(self):
        """
        Verify that a resource request fails when a behavior that is not loaded
        requests a resource and there are no roles.
        """
        response1 = self.request_resources(
            "behavior_-1", 10, [])
        self.assertFalse(response1.success)

    # def test_simultaneous_service_calls(self):
    #     """
    #     Verify that the manager does not crash when recieving multiple resource requests 
    #     at the same time.
    #     """
    #     processes = []
    #     num_processes = 3
    #     for _ in range(num_processes):
    #         request_process = Process(target=self.multiprocess_request_resources, 
    #                                   args=("behavior_1",10,self.test_request.roles,self.queue,))
    #         request_process.start()
    #         processes.append(request_process)

    #     for process in processes:
    #         process.join()
    #     responses = []
    #     for _ in range(num_processes):
    #         responses.append(self.queue.get())

    #     self.assertEqual(len(responses), 3)
    #     for response in responses:
    #         self.assertTrue(response.success)

    # def multiprocess_request_resources(self, beh_name, priority, roles, queue):
    #     """
    #     Helper method for simultaneous requests.
    #     """
    #     response = self.request_resources(beh_name, priority, roles)
    #     queue.put(response)
        
        


if __name__ == "__main__":
    rospy.init_node("resource_manager_test")
    rostest.rosrun(PKG, "ResourceManagerTests", ResourceManagerROSTests)
