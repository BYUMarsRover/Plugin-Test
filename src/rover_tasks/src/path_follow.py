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

class Path_Follow(task):

    # Class constructor
    def __init__(self, name ,rate, waypoint_list) -> None:
        
        super().__init__(name, rate) # Call the parent constructor

        self.waypoints = waypoint_list #Copy the list of waypoints into the self object

    # Overide of the run_task method. 
    # This is a state machine that takes the turtle to each waypoint
    def run_task(self):



    