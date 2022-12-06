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


import roslib
roslib.load_manifest('rover_tasks')
import rospy
import actionlib

from rover_tasks.msg import GateTraversalAction, GateTraversalActionResult


from task import Task
from enum import Enum
from std_msgs.msg import Bool
from copy import deepcopy

class Fiducial_Tracking_Node(Task):

    gate_distance = 2.0 # Distance the turtle travels forward upon seeing the fiducial
    fwd_vel = 2.0
    rot_vel = 2.5

    class _STATE(Enum):

        INIT = 0
        CORRECTING_ANGLE = 1
        DRIVING_FORWARD = 2
        FINISH = 4
        KILL = 5


    def __init__(self, name, rate) -> None:

        super().__init__(name, rate)

        self._manager_listener.name = "fiducial_tracking/kill"

        self._task_status_publisher.name = "fiducial_tracking/status"

        self.see_tag = False

        self.state_machine_state = self._STATE.INIT

        self.curr_Twist = [0,0]

        self.tag_subscriber = rospy.Subscriber('fiducials/tag_present', Bool, self.update_tag_status, queue_size=1)

        self.action_server = actionlib.SimpleActionServer('Gate_Traversal', GateTraversalAction, self.execute, auto_start=False)
        self.action_server.start()
        rospy.logwarn("Started gate traversal action server!")

    # method that executes the state machine action

    def execute(self, goal):
        

        rospy.logwarn("Performing Gate Traversal")
        self.run_task()
        rospy.logwarn("Gate Traversed")
        self.action_server.set_succeeded()



    # updates whether we're seeing a tag or not
    def update_tag_status(self, msg) -> None:

        # only record if we see a tag. That way we don't stop and then start spinning again
        if msg.data == True:
            self.see_tag = msg.data


    def get_distance_from_start(self, curr_state):

        diffx = curr_state[0] - self.og_state[0]
        diffy = curr_state[1] - self.og_state[1]

        return (diffx**2 + diffy**2)**0.5


    # Override of the super.run_task method
    def run_task(self):

        rospy.loginfo("Rover is searching for a ficudial tag")

        self.og_state = deepcopy(self._turtle_state)

        while(self.state_machine_state != self._STATE.KILL and not rospy.is_shutdown()):

            if self.state_machine_state == self._STATE.INIT:

                if self.see_tag:

                    self._curr_Twist = [self.fwd_vel, 0]
                    self.state_machine_state = self._STATE.DRIVING_FORWARD
                    rospy.loginfo('Rover has found tag and is traversing gate!')

                else:
                    
                    self._curr_Twist = [0, self.rot_vel]
                    self.state_machine_state = self._STATE.CORRECTING_ANGLE



            elif self.state_machine_state == self._STATE.CORRECTING_ANGLE:

                if self.see_tag:

                    self._curr_Twist = [self.fwd_vel, 0]
                    self.state_machine_state = self._STATE.DRIVING_FORWARD
                    rospy.loginfo('Rover has found tag and is traversing gate!')

            elif self.state_machine_state == self._STATE.DRIVING_FORWARD:

                if self.get_distance_from_start(self._turtle_state) >= self.gate_distance:

                    self._curr_Twist = [0,0]
                    self.state_machine_state = self._STATE.FINISH
                    rospy.loginfo("The rover has finished traversing the gate")

            elif self.state_machine_state == self._STATE.FINISH:

                self._task_status = True

            elif self.state_machine_state == self._STATE.KILL: 

                break

            self.publish_cmd_vel(self._curr_Twist)
            self.publish_task_state(self._task_status)
            self._rate.sleep()


if __name__ == '__main__':

    fiducial_tracker = Fiducial_Tracking_Node('fiducial_tracker',20)