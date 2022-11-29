#!/usr/bin/env python

from abc import ABCMeta, abstractmethod
#from rover_tasks.src.task import Task


class Plugin_Base():
    __metaclass__ = ABCMeta

    @staticmethod
    @abstractmethod
    def get_config_form():
        pass


    @abstractmethod
    def execute(self):
        pass


    def _request_input(self, prompt, required = True):
        # type: (Plugin_Base, str, bool) -> None
        while True:
            value = input(prompt)
            if value != '' or required == False:
                return value

    
    @staticmethod
    def _run_synchronous_task(task):
        # type: (Task) -> None
        task.run_task()
