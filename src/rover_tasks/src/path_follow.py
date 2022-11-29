#!/usr/bin/python3.8
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

import rospy
from task import Task
from math import atan2, pi

from enum import Enum


class Path_Follow(Task):

    class _STATE(Enum):

        INIT = 0
        CORRECTING_ANGLE = 1
        DRIVING_FORWARD = 2
        ON_WAYPOINT = 3
        FINISH = 4
        KILL = 5

    waypoint_radius_tolerance = 0.05 # the acceptable radius tolerance for landing on a waypoint

    angle_tolerance = 0.15 # +/- angle tolerance when persuring a waypoint

    foward_cmd_speed = 0.5 # command speed for forward commands in pix/s

    rotational_cmd_vel = 1.25 # command rotational velocity in rad/s

    _curr_Twist = [0,0] # currend command velocities in format [fwd, angular]

    # Class constructor
    def __init__(self, name ,rate, waypoint_list) -> None:
        
        
        self.waypoints = waypoint_list #Copy the list of waypoints into the self object

        self.curr_point = None # set the current waypoint as a none

        self._machine_state = self._STATE.INIT # initialize the state machine position

        super().__init__(name, rate) # Call the parent constructor

        self._manager_listener.name = 'path_follow/kill'

        self._task_status_publisher.name = "path_follow/status"


    # Overide of the run_task method. 
    # This is a state machine that takes the turtle to each waypoint
    def run_task(self):

        while(self._machine_state != self._STATE.KILL):

           
            if self._machine_state == self._STATE.INIT: ################################################# INIT STATE
                

                if len(self.waypoints) == 0:

                    self._machine_state = self._STATE.FINISH # If there's no waypoints, we're gtg
                
                else:
                    self.waypoints.pop(0)
                    self.curr_point = self.waypoints[0]
                    rospy.loginfo('Rover has reached waypoint')
                    rospy.loginfo("Current Path Plan" + str(self.curr_point))
                    
                    # If we're not on the waypoint, check if we need to spin
                    if  not self.check_waypoint_tolerance(self.curr_point):
                        
                        
                        if not self.check_angle_tolerance(self.curr_point):

                            self._curr_Twist = [0,self.get_rotational_vel(self.curr_point,self.rotational_cmd_vel)]
                            self._machine_state = self._STATE.CORRECTING_ANGLE 

                            
                        else: 

                            self._curr_Twist = [self.foward_cmd_speed,0]
                            self._machine_state = self._STATE.DRIVING_FORWARD


                    else: # If we're on the waypoint, then just change state
                        
                        self._curr_Twist = [0,0]
                        self._machine_state = self._STATE.ON_WAYPOINT


            elif self._machine_state == self._STATE.CORRECTING_ANGLE: ################################# SPIN STATE

                # If we're not on the waypoint, check if we need to spin
                    if  not self.check_waypoint_tolerance(self.curr_point):
                        
                        
                        if not self.check_angle_tolerance(self.curr_point): # Yes we need to spin to get to the waypoint

                            self._curr_Twist = [0,self.get_rotational_vel(self.curr_point,self.rotational_cmd_vel)]
                            self._machine_state = self._STATE.CORRECTING_ANGLE 

                             
                        else: # No, we don't need to spin to get to the waypoint

                            self._curr_Twist = [self.foward_cmd_speed,0]
                            self._machine_state = self._STATE.DRIVING_FORWARD


                    else: # If we're on the waypoint, then just change state
                        
                        self._curr_Twist = [0,0]
                        self._machine_state = self._STATE.ON_WAYPOINT
            
            elif self._machine_state == self._STATE.DRIVING_FORWARD: ################################## Driving Forward

                # If we're not on the waypoint, check if we need to spin
                    if  not self.check_waypoint_tolerance(self.curr_point):
                        
                        
                        if not self.check_angle_tolerance(self.curr_point):

                            self._curr_Twist = [0,self.get_rotational_vel(self.curr_point,self.rotational_cmd_vel)]
                            self._machine_state = self._STATE.CORRECTING_ANGLE 

                            
                        else: 

                            self._curr_Twist = [self.foward_cmd_speed,0]
                            self._machine_state = self._STATE.DRIVING_FORWARD


                    else: # If we're on the waypoint, then just change state
                        
                        self._curr_Twist = [0,0]
                        self._machine_state = self._STATE.ON_WAYPOINT
            
            elif self._machine_state == self._STATE.ON_WAYPOINT: ########################################### ON WAYPOINT STATE

                if len(self.waypoints) == 0:

                    self._machine_state = self._STATE.FINISH # If there's no waypoints, we're gtg
                
                else:

                    self.waypoints.pop(0)
                    self.curr_point = self.waypoints[0]
                    rospy.loginfo('Rover has reached waypoint')
                    rospy.loginfo("Current Path Plan" + str(self.curr_point))
                    
                    # If we're not on the waypoint, check if we need to spin
                    if  not self.check_waypoint_tolerance(self.curr_point):
                        
                        
                        if not self.check_angle_tolerance(self.curr_point):

                            self._curr_Twist = self._curr_Twist = [0,self.get_rotational_vel(self.curr_point,self.rotational_cmd_vel)]
                            self._machine_state = self._STATE.CORRECTING_ANGLE 

                            
                        else: 

                            self._curr_Twist = [self.foward_cmd_speed,0]
                            self._machine_state = self._STATE.DRIVING_FORWARD

            elif self._machine_state == self._STATE.FINISH: ########################################### ON FINISHED STATE
                
                self._task_status = True
                self._curr_Twist = [0,0] # Stop all motion


            self.publish_cmd_vel(self._curr_Twist)
            self.publish_task_state(self._task_status)
            self._rate.sleep()

            if rospy.is_shutdown():

                break

    
    # Checks to see if the turtle bot is within the acceptable radial 
    # tolerance of a waypoint
    def check_waypoint_tolerance(self, waypoint) -> bool:

        diff_x = waypoint[0] - self._turtle_state[0]
        diff_y = waypoint[1] - self._turtle_state[1]

        # Use pythagorean theorom to find if we're within tolerance
        if (diff_x**2 + diff_y**2)**0.5 <= self.waypoint_radius_tolerance:

            return True

        return False

    # Checks to see if the turtle bot is within acceptable
    # angular tolerance ofthe current best path to the next waypoint
    def check_angle_tolerance(self, waypoint) -> bool:

        diff_x = waypoint[0] - self._turtle_state[0]
        diff_y = waypoint[1] - self._turtle_state[1]

        des_angle = atan2(diff_y,diff_x)

        if abs(des_angle - self._turtle_state[2]) <= self.angle_tolerance:

            return True

        return False

    # Checks to see if the turtle bot is within acceptable
    # angular tolerance ofthe current best path to the next waypoint
    def get_rotational_vel(self, waypoint, curr_vel) -> float:

        diff_x = waypoint[0] - self._turtle_state[0]
        diff_y = waypoint[1] - self._turtle_state[1]

        des_angle = atan2(diff_y,diff_x)

        multiplier1 = (des_angle - self._turtle_state[2]) / abs(des_angle - self._turtle_state[2])

        multiplier2 = 1

        if abs(des_angle - self._turtle_state[2]) >= pi:

            multiplier2 = -1

        return curr_vel*multiplier1*multiplier2
        


## =====  Main Method =======

if __name__ == '__main__':

    waypoints = [(0,0), (2,0), (2,2), (0,2), (0,0), [2,3.5]]

    curr_task = Path_Follow('Path_Planner', 20, waypoints)

    curr_task.run_task()

    waypoints = [(0,0), (2,0), (2,2), (0,2), (0,0), [2,3.5]]

    curr1_task = Path_Follow('Path_Planner', 20, waypoints)

    curr1_task.run_task()