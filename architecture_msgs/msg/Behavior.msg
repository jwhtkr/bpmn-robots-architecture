##
# @File: Behavior.msg
#
# @brief
# Describes a BPMN task and the behavior that will be or has been executed.
#
# @topic
# The Camunda topic associated with the task.
#
# @name
# Name of the task and behavior.
#
# @instance_id
# A Id that is unique to each instance of all behaviors and tasks.
#
# @status
# What happened to the task or behavior and what should happen to its counterpart.
##

##
# @These are predefined values that should be used to populate the status field.
#
# @NEW
# A new task has arrived in the BPMN.
#
# @CANCELED
# A BPMN task and been ended at the BPMN level of execution.
#
# @NULL
# Explicitly denotes no information.
##

uint8 NEW = 2
uint8 CANCELED = 1
uint8 NULL_STATUS = 0

string topic
string name
string instance_id
uint8  status

## Behavior.msg ##
