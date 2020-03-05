#!/usr/bin/env python
"""This module contains a node for testing the resource manager node"""

from pdb import set_trace as pause

import yaml

import rospy
from rospkg import RosPack as RP

from architecture_msgs.srv import ResourceRequest  # pylint: disable=import-error
from architecture_msgs.msg import Role, Resource   # pylint: disable=import-error
from architecture_msgs.srv import ResourceRequest, ResourceRequestResponse

if __name__ == "__main__":
    # pylint: disable=invalid-name

    def update_resources_response(req):
        res = ResourceRequestResponse()
        res.success = True
        res.roles   = req.roles
        return res

    rospy.init_node('resource_manager_test')

    request_service_name = rospy.get_param("request_resources_topic",
                                           "request_resources")
    return_service_name = rospy.get_param("resource_return_topic",
                                          "resource_return")
    rospy.wait_for_service(return_service_name)
    rospy.wait_for_service(request_service_name)
    rospy.Service('/behavior_1/update',ResourceRequest,update_resources_response)
    rospy.Service('/behavior_2/update',ResourceRequest,update_resources_response)
    request_resources = rospy.ServiceProxy(request_service_name, ResourceRequest)  # pylint: disable=line-too-long
    resource_return = rospy.ServiceProxy(return_service_name, ResourceRequest)

    rm_dir = RP().get_path('resource_manager')
    x = yaml.safe_load(file(rm_dir + '/src/resource_manager/resources.yaml'))
    roles = []
    roles.append(Role("role_1",
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
    roles.append(Role("role_2",
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

    # pause()
    resp1 = request_resources("behavior_1", 10, roles)
    print resp1.success

    resp2 = request_resources("behavior_2", 10, roles)
    print resp2.success

    resp3 = resource_return("behavior_1", 10, [resp1.roles[0]])
    print resp3.success
    resp4 = resource_return("behavior_1", 10, [resp1.roles[1]])
    print resp4.success

    resp5 = resource_return("behavior_2", 10, [resp2.roles[0]])
    print resp5.success
    resp6 = resource_return("behavior_2", 10, [resp2.roles[1]])
    print resp6.success

    rospy.spin()

