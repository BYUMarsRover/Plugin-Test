#!/usr/bin/env python

from plugin_base import Plugin_Base
from rover_tasks.src.path_follow import Path_Follow


class Waypoint_Plugin(Plugin_Base):

    __latitude: str
    __longitude: str


    def __init__(self):
        __latitude = ""
        __longitude = ""


    def get_config(self):
        self.__latitude = self._request_input("Latitude:")
        self.__longitude = self._request_input("Longitude:")


    def execute(self):
        path_task = Path_Follow()
        self._run_synchronous_task(path_task)
