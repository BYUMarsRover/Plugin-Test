#!/usr/bin/python3.8

# Adam Welker       BYU Mars Rover      November 2022
#
# task_manager.py -- a ros node that acts as a state machine manager.
# the node is capable of managing a queue of tasks that need to 
# be run on the rover. The manager opens and runs, and then kills task
# state machine. THIS IS A PROTOTYPE



import rospy
from std_msgs.msg import Bool
import actionlib


from path_follow import Path_Follow
from fiducial_tracking import Fiducial_Tracking_Node
from task import Task



class Task_Manager():

    task_queue = []

    task_kill = False


    def __init__(self) -> None:

        rospy.init_node('Task_Manager',anonymous=True)

        self.rate = rospy.Rate(20)
        
        self.kill_publisher = rospy.Publisher('manager/kill',Bool,queue_size=1)
        self.status_subscriber = rospy.Subscriber('task/status',Bool,self.status_listener,queue_size=1)
    

    def add_task(self, task):

        assert issubclass(task, Task)

        self.task_queue.append(Task)

    def status_listener(self, msg):

        if msg.data == True:

            self.task_kill = True

    def run(self):


        while len(self.task_queue) > 0 and not rospy.is_shutdown():

            curr_task = self.task_queue.pop(0) # define the current task

            curr_task.run_task()

            self.task_kill == False

            self.rate.sleep()




if __name__ == '__main__':

    waypoints = [(0,0), (2,0), (2,2), (0,2), (0,0), [2,3.5]]

    path_task = Path_Follow('Path_Planner', 20, waypoints)

    fiducial_tracker = Fiducial_Tracking_Node('fiducial_tracker',20)

    task_manager = Task_Manager

    rospy.spin()






