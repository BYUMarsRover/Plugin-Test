#!/usr/bin/python3.8

# Adam Welker       BYU MARS ROVER      December 2022
#
# state_machine_task_manager.py -- this file acts as a
# task manager to various state machines implemented as ros actions
# This file should contain two ros action clients. THIS IS A PROTOTYPE


import rospy
import roslib
roslib.load_manifest('rover_tasks')
import rospy
import actionlib


from geometry_msgs.msg import Pose
from rover_tasks.msg import WayPointsAction, WayPointsGoal, GateTraversalAction, GateTraversalGoal


class StateMachineTaskManager:


    def __init__(self, rate = 20) -> None:

        rospy.init_node("State_Machine_Task_Manager")
        self.rate = rospy.Rate(rate)
        
        # Define action clients
        self.waypoint_client = actionlib.SimpleActionClient('Path_Follow', WayPointsAction)
        self.gatetraversal_client = actionlib.SimpleActionClient('Gate_Traversal', GateTraversalAction)


        #Wait for all action servers
        rospy.loginfo("Waiting for waypoint action server")
        self.waypoint_client.wait_for_server()
        rospy.loginfo("Connected to waypoint action server")

        rospy.loginfo("Waiting for gate traversal action server")
        self.gatetraversal_client.wait_for_server()
        rospy.loginfo("Connected to gate traversal action server")

    

    def perform_waypoint_action(self, waypoints):

        # transfer waypoints in to pose messages
        pose_array = []

        for point in waypoints:

            new_pose_point = Pose()

            new_pose_point.position.x = point[0]
            new_pose_point.position.y = point[1]

            pose_array.append(new_pose_point)

        # make the waypoints goal

        path_goal = WayPointsGoal()
        path_goal.waypoints = pose_array

        # Send the waypoints goal

        self.waypoint_client.send_goal(path_goal)
        rospy.logwarn("Waypoint Request Sent")

        # wait for a response

        self.waypoint_client.wait_for_result(rospy.Duration.from_sec(60.0))
       
        if self.waypoint_client.get_state() == 0:

            rospy.logwarn("Task not completed 0_o")
        else:

            rospy.logwarn("Task Done")


        
    def perform_gate_action(self):

        action_request = GateTraversalGoal()

        action_request.task_id = 2
        
        self.gatetraversal_client.send_goal(action_request)

        rospy.logwarn("Sent Request for Traversal Task!")
        
        self.gatetraversal_client.wait_for_result()
        
        rospy.logwarn("Traversal Task Complete!")
        
        


# ------------------------
#       main method
# ------------------------
if __name__ == '__main__':

    waypoints = [(0,0), (2,0), (2,2), (0,2), (0,0), [2,3.5]]

    task_manager = StateMachineTaskManager()

    task_manager.perform_waypoint_action(waypoints)

    task_manager.perform_gate_action()

    rospy.spin()