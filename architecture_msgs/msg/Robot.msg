##
# @File: Robot.msg
#
# @brief
# Describes one robot in terms that the ThreadPool master will understand.
#
# @name
# Name of this robot.
#
# @start
# The names of all the resources that this behavior needs on that robot.
#
# @end
# The names of all the resources that need to be ended on the robot.
#
# @remap
# List of topics to be remapped
##

string name
string[] start
string[] end
architecture_msgs/TopicRemap[] remap

## Robot.msg ##
