# Behavior_Listener

## Purpose
The Behavior Listener primarily reduces the network load in calling the Camunda
Engine REST API.

Because external tasks have a "poll" mechanism for discovering new tasks, it is
beneficial to have only one program performing this regular polling on behalf of
all of the others in order to reduce network load. The Behavior Listener keeps 
track of all of the external tasks and notifies all of the Behavior Pool 
Managers of the creation, or cancellation, of external tasks.

## Functionality
The Behavior Listener achieves this by utilizing the "external-task" function of
the REST API. This function returns a list of tasks, and the Behavior Listener
specifies whether to include only unlocked, or only locked tasks. These induce
the possible states for each external task of previously unlocked, previously 
locked, currently unlocked, and currently locked. Lists of external tasks, each
list corresponding to one of the possible states, is stored every update cycle. 
Note that a task cannot be in both "previous" or both "current" lists at the 
same time. There are several cases that must be considered. These cases and how 
the Behavior Listener handles each case are presented below:

| Case | Implication | Action |
| ------ | ------ | ------ |
| A task is not in either "previous" list, but is in the "current unlocked" list | This means this task is new | Notification is sent to the Behavior Pool Managers |
| A task is in the "previous locked" list and in the "current unlocked" list | This means this task failed, but should be retried | This is treated as if it is new and notification is sent to the Behavior Pool Managers | 
| A task is in the "previous unlocked" list and in the "current unlocked" list | The task is not new and has not yet been claimed by a Behavior Pool Manager | No action is taken |
| A task is in the "previous unlocked" list and in the "current locked" list | The task is not new, and has just been claimed by a Behavior Pool Manager | No action is taken |
| A task is in the "previous locked" list and in the "current locked" list | The task is not new and is claimed by a Behavior Pool Manager | No action is taken |
| A task is in either "previous" list but is not in either "current" list | The task has either been completed successfully or cancelled by the Camunda Engine | Notification of cancellation is sent to the Behavior Pool Managers |
| A task is not in either "previous" list, but is in the "current locked" list | This is only possible if another entity is polling for external tasks and claims them in between Behavior Listener updates | No action is taken |

Note that due to the polling nature of the Behavior Listener and the one-way 
communication to the Behavior Pool Managers, there is no way to distinguish 
between a successful completion and a cancellation of a task. Thus it is always 
treated as a cancellation, and the Behavior Pool Manager distinguishes between 
the two. Also note that it is assumed that the Behavior Listener is the only 
entity polling the Camunda Engine, and so it is impossible for a task to appear 
in the "current locked" list without also appearing in the "previous unlocked" 
list.

The notifications to the Behavior Pool Managers are sent as ROS messages via 
ROS publishing. The message definition is [here](https://gitlab.com/droge-robotics/camunda/architecture_msgs/blob/master/msg/Behavior.msg).


## Use
The Behavior Listener can be started with the rest of the architecture as is. 
Currently the Behavior Listener only recognizes tasks that are started after it 
is initialized, so it must be started before a mission or other process is 
started. It runs as a ROS node for ROS publishing capability and so can be 
started in typical ROS manners via rosrun, roslaunch, etc.

## Dependancies
The Behavior Listener is dependant on the Camunda Engine, ROS, and the httplib2 
and json python libraries.