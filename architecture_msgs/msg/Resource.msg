##
# @File: Resource.msg
#
# @brief
# Describes a specific resource, IE a robot for the resource manager to be aware of.
#
# @group_name
# Group identifier (usually corresponds to a Robot). Can be blank if this is a request.
#
# @name
# Resource's identifier. Can be left blank if this message is part of a request for resources.
#
# @category
# Category of resource: mobility, sensor, actuator, computation, etc.
#
# @type
# Type of resource within category: wheeled, camera, 3-axis, server, etc.
#
# @required
# Whether or not this resource is essential to it's role.
#
# @priority
# How important this resource is to it's Role.
##

string group_name
string name
string category
string type
bool   required
uint8  priority

## Resource.msg ##
