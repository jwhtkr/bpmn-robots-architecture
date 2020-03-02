#!/usr/bin/env python
"""This module helps to interface with the Camunda bpmn engine."""

from __future__ import print_function
import json
from httplib2 import Http


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


if __name__ == "__main__":
    print(str(CamundaRESTInteraction().certify().request_get("engine")))
