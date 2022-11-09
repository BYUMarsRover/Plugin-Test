#!/usr/bin/env python

from abc import ABC, abstractmethod
from rover_tasks.src.task import Task


class Plugin_Base(ABC):

    @abstractmethod
    def get_config(self):
        pass


    @abstractmethod
    def execute(self):
        pass


    def _request_input(self, prompt: str, required: bool = True):
        while True:
            input = input(prompt)
            if input != '' or required == False:
                return input

    
    def _run_synchronous_task(task: Task):
        task.run_task()
