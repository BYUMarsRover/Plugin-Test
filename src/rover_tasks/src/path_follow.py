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

import roslib
roslib.load_manifest('rover_tasks')
import rospy
import actionlib

from rover_tasks.msg import WayPointsAction

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
    def __init__(self, name ,rate, waypoint_list = []) -> None:
        
        super().__init__(name, rate) # Call the parent constructor

        ## Make the internal variables 
        
        self.waypoints = waypoint_list #Copy the list of waypoints into the self object

        self.curr_point = None # set the current waypoint as a none

        self._machine_state = self._STATE.INIT # initialize the state machine position

        ## Define Action Server

        self.action_server = actionlib.SimpleActionServer('Path_Follow', WayPointsAction, self.execute , auto_start=False)
        self.action_server.start()

        rospy.logwarn("Started Path Follow Action Server!")
        


    
    def execute(self,goal):

        self.add_waypoints(goal)

        self.run_task()

        self.action_server.set_succeeded()


    # Overide of the run_task method. 
    # This is a state machine that takes the turtle to each waypoint
    def run_task(self):

        while(not rospy.is_shutdown()):

           
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
                    if len(self.waypoints) > 0:
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
                break

            if self._machine_state == self._STATE.KILL:
                
                break


            self.publish_cmd_vel(self._curr_Twist)
            self.publish_task_state(self._task_status)
            self._rate.sleep()


        self._curr_Twist = [0,0] # Stop all motion
        self.publish_cmd_vel(self._curr_Twist)
        self.publish_task_state(self._task_status)
        self._machine_state = self._STATE.INIT

    

    
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

    #Transfers goal attributes to internally stored waypoints

    def add_waypoints(self,goal):

        poses = goal.waypoints

        for point in poses:

            x = point.position.x
            y = point.position.y

            coordinate = (x,y)

            self.waypoints.append(coordinate)       


## =====  Main Method =======

if __name__ == '__main__':

    path_Follower = Path_Follow('Path_Planner', 20)
    server = path_Follower.action_server

    rospy.spin()