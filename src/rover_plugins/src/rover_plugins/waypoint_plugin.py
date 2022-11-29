#!/usr/bin/env python

from rover_plugins.plugin_base import Plugin_Base
#from rover_tasks.src.path_follow import Path_Follow


class Waypoint_Plugin():
    __metaclass__ = Plugin_Base


    def __init__(self):
        __latitude = ""
        __longitude = ""


    def get_config_form(self):
        return ''


    #def execute(self):
        #path_task = Path_Follow("Follow Path", 20, [])
        #self._run_synchronous_task(path_task)
