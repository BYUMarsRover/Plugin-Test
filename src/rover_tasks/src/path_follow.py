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
from typing import List
import task
from math import asin

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

    foward_cmd_speed = 5.0 # command speed for forward commands in pix/s

    rotational_cmd_vel = 1.0 # command rotational velocity in rad/s

    _curr_Twist = [0,0] # currend command velocities in format [fwd, angular]

    # Class constructor
    def __init__(self, name ,rate, waypoint_list) -> None:
        
        super().__init__(name, rate) # Call the parent constructor

        assert type(waypoint_list) == List

        self.waypoints = waypoint_list #Copy the list of waypoints into the self object

        self.curr_point = None # set the current waypoint as a none

        self._machine_state = 0 # initialize the state machine position

    # Overide of the run_task method. 
    # This is a state machine that takes the turtle to each waypoint
    def run_task(self):

        while(self._machine_state != self._STATE.FINISH):

            if self._machine_state == self._STATE.INIT:

                if len(self.waypoints == 0):

                    self._machine_state = self._STATE.FINISH # If there's no waypoints, we're gtg
                
                else:

                    self.curr_point = self.waypoints.pop(0)

                    if  not self.check_waypoint_tolerance(self.curr_point): # If we're not on the waypoint, begin spinning
                        
                        self._curr_Twist = [0,self.rotational_cmd_vel]

                        self._machine_state = self._STATE.CORRECTING_ANGLE 

                    else:

                        pass


            elif self._machine_state == self._STATE.CORRECTING_ANGLE:

                pass
            
            elif self._machine_state == self._STATE.DRIVING_FORWARD:

                pass
            
            elif self._machine_state == self._STATE.FINISH:

                pass

    
    # Checks to see if the turtle bot is within the acceptable radial 
    # tolerance of a waypoint
    def check_waypoint_tolerance(self, waypoint):

        diff_x = waypoint[0] - self._turtle_state[0]
        diff_y = waypoint[1] - self._turtle_state[1]

        # Use pythagorean theorom to find if we're within tolerance
        if (diff_x**2 + diff_y**2)**0.5 <= self.waypoint_radius_tolerance:

            return True

        return False

    # Checks to see if the turtle bot is within acceptable
    # angular tolerance ofthe current best path to the next waypoint
    def check_angle_tolerance(self, waypoint):

        diff_x = waypoint[0] - self._turtle_state[0]
        diff_y = waypoint[1] - self._turtle_state[1]

        des_angle = asin(diff_y/diff_x)

        if abs(des_angle - self._turtle_state[2]) <= self.angle_tolerance:

            return True

        return 
        


        