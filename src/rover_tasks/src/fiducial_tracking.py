#!/usr/bin/python3.8
# Adam Welker       MARS ROVER 22   OCTOBER 2022
#
# fiducial_tracking.py -- a Task state machine & ROS node that 
# commands the turtle to spin until it sees a waypoint, then stops, and moves forward by 1 unit on the screen
#
# INHERITS: task.py*
#
# SUBSCRIPTIONS: - *turtle1/pose  (The location and velocity of the turtle bot)
#                - *manager/kill  (A bool message to the task signaling the destruction of the node)
#                - *fiducials/tag_present (A bool message saying if a fiducial is detected by the camera)
#
#
# PUBLICATIONS:  - *turtle1/cmd_vel (Command velocities to the turtle bot)


import rospy
from task import Task
from enum import Enum

class Fiducial_Tracking_Node(Task):

    gate_distance = 2.0 # Distance the turtle travels forward upon seeing the fiducial

    class _STATE(Enum):

        INIT = 0
        CORRECTING_ANGLE = 1
        DRIVING_FORWARD = 2
        FINISH = 4
        KILL = 5


    def __init__(self, name, rate) -> None:

        super().__init__(name, rate)