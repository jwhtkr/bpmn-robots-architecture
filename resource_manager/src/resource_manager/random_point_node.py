#! /usr/bin/env python
"""Contains a ROS node for generating points and creating a BPMN signal."""

from __future__ import print_function
import json
from httplib2 import Http
import numpy as np
import rospy

class CamundaRESTInteraction(object):
    """A base class for interacting with the Camunda Engine through REST."""

    BASE_URI = "http://localhost:8080/engine-rest/"
    JSON_HEADER = {"content-type": "application/json"}
    NAME = PASSWORD = "demo"

    def __init__(self, base_uri=BASE_URI,
                 encoding='utf-8',
                 name=NAME,
                 password=PASSWORD):
        """Initialize CamundaRESTInteraction."""
        self._connection = Http()
        self._base_uri = base_uri
        self._encoding = encoding
        self._decoder = json.JSONDecoder(encoding=self._encoding)
        self._encoder = json.JSONEncoder(encoding=self._encoding)
        self.certify(name, password)

    def certify(self, name=NAME, password=PASSWORD):
        """Add certification with name and password."""
        self._connection.add_credentials(name, password)

        return self

    def request_get(self, request=""):
        """Make a simple GET request."""
        response = self._connection.request(self._base_uri + request)

        try:
            data = self._decoder.decode(response)
        except ValueError:
            return None
        else:
            return data

    def request_get_body(self, query_body, request=""):
        """Make a GET request with body content as a dict."""
        query_list = ["=".join([key, query_body[key]]) for key in query_body]
        query = "&".join(query_list)
        query = "?" + query
        response = self._connection.request(self._base_uri + request + query)
        try:
            data = self._decoder.decode(response[1])
        except ValueError:
            return None
        else:
            return data

    def request_post(self, body, request=""):
        """Make a POST request with the body as a dict."""
        response = self._connection.request(self._base_uri + request,
                                            method="POST",
                                            body=self._encoder.encode(body),
                                            headers=self.JSON_HEADER)
        try:
            data = self._decoder.decode(response[1])
        except ValueError:
            return None
        else:
            return data


class Signal(CamundaRESTInteraction):
    """Class for sending signals to the BPMN engine."""
    def send_signal(self, name, variables):
        """Send the named BPMN signal with the given input variables."""
        body = {"name":name,
                "variables":variables}
        self.request_post(body, request="signal")


def generate_random_points(n_points, seed=None):
    """Generate random 3D points."""
    if seed:
        np.random.seed(seed)
    return np.random.uniform(-5, 5, (n_points, 3))


class RandomPointsNode(object):
    """Generate Random Points and send to BPMN"""
    def __init__(self, n_points=10, seed=None, signal_name=""):
        rospy.init_node("random_point_node")
        self.n_points = n_points
        self.seed = seed
        self.signal_name = signal_name
        self.signal = Signal()
        self.random_points = generate_random_points(n_points, seed)
        self.send_points()

    def send_points(self):
        """Send a signal for each point."""
        for i in range(self.n_points):
            point = self.random_points[i]
            self.signal.send_signal(self.signal_name,
                                    self.point_to_dict(point))

    @staticmethod
    def point_to_dict(point):
        """Convert a point from list-like to dict."""
        return {"x":{"value":point[0], "type":"string"},
                "y":{"value":point[1], "type":"string"},
                "z":{"value":point[2], "type":"string"}}



if __name__ == "__main__":
    RandomPointsNode(n_points=6, signal_name="InvestigatePointSignal")
