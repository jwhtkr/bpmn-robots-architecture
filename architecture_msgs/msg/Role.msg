##
# @File: Role.msg
#
# @brief
# Describes a specific role and its resource's within a behavior.
#
# @name
# Name of the role.
#
# @required
# Whether or not this role is essential to it's behavior.
#
# @priority
# How important this role is to its behavior.
#
# @resources
# The resources that this role uses.
##

string name
bool   required
uint8  priority
architecture_msgs/Resource[] resources

## Role.msg ##
