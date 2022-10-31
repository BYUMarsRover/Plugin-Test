# Adam Welker       MARS ROVER 22   OCTOBER 2022
#
# path_follow.py -- a Task state machine & ROS node that 
# commands the turtle sim to follow a list of waypoints
#
# INHERITS: task.py*
#
# SUBSCRIPTIONS: - *turtle1/pose  (The location and velocity of the turtle bot)
#                - *manager/kill  (A bool message to the task signaling the destruction of the node)
#
#
# PUBLICATIONS:  - *turtle1/cmd_vel (Command velocities to the turtle bot)
#                - *task/status     (A Bool indicating whether the task has been completed)
import task

from enum import Enum


class Path_Follow(task):

    class _STATE(Enum):

        INIT = 0
        CORRECTING_ANGLE = 1
        DRIVING_FORWARD = 2
        ON_WAYPOINT = 3
        FINISH = 4

        waypoint_radius_tolerance = 0.5 # the acceptable radius tolerance for landing on a waypoint

        angle_tolerance = 1.0 # +/- angle tolerance when persuring a waypoint


    # Class constructor
    def __init__(self, name ,rate, waypoint_list) -> None:
        
        super().__init__(name, rate) # Call the parent constructor

        self.waypoints = waypoint_list #Copy the list of waypoints into the self object

        self.curr_point = None # set the current waypoint as a none

        self._machine_state = 0 # initialize the state machine position

    # Overide of the run_task method. 
    # This is a state machine that takes the turtle to each waypoint
    def run_task(self):

        while(self._machine_state != self._STATE.FINISH):

            if self._machine_state == self._STATE.INIT:

                pass 

            elif self._machine_state == self._STATE.CORRECTING_ANGLE:

                pass
            
            elif self._machine_state == self._STATE.DRIVING_FORWARD:

                pass

            elif self._machine_state == self._STATE.ON_WAYPOINT:

                pass
            
            elif self._machine_state == self._STATE.FINISH:

                pass



        