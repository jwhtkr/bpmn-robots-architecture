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
# @info
# Role level information needed for Resource Manager to calculate scores.
# Formatted as a YAML or JSON string.
#
# @resources
# The resources that this role uses.
##

string name
bool   required
uint8  priority
string info
architecture_msgs/Resource[] resources

## Role.msg ##
