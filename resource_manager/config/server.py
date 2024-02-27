"""
Config file for a general server

By including this module, or running it, you should have access to the
config variables/data for the server.

A resource group config file should give mapping values between a general term
for data that needs to be accessed to robot-specific topics, services,etc. to
access that data.
e.g.:
"position" -> "odom"

Because its a python script it could even return a dict of function handles
that, if called (e.g. dict["position"]()), return the value(s) desired.
"""

from resource_manager.agent_base import Agent   # pylint: disable=import-error

# For now I don't think the server config needs anything


class ServerAgent(Agent):
    """
    Class to represent a server.
    """

    def __init__(self, name, resources, namespace):
        super(ServerAgent, self).__init__(name, resources, namespace)
        self.capacity = 100
