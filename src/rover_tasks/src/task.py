# Adam Welker       MARS ROVER 2022         October 2022
#
# task.py -- an interface class for state machine tasks

# A class that represents a state machine that will perform some task.
# This class is meant to be inherited. 
#
# SUBSCRIPTIONS: - turtle1/pose  (The location and velocity of the turtle bot)
#                - manager/kill  (A bool message to the task signaling the destruction of the node)
#
#
# PUBLICATIONS:  - turtle1/cmd_vel (Command velocities to the turtle bot)
#                - task/status     (A Bool indicating whether the task has been completed)
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np

# A parent class to each of the state machine tasks. Has atributes common publisher and subscribers for interfacing
# with the task manager + turtle bot. Also contains a run_task() method that must be overwritten. 
class Task():

    # class constructor
    def __init__(self, name, rate):
    
        self._node_name = str(name) # The name visible in the ros node 

        # Publishers
        self._turtle_listener = rospy.Subscriber("turtle1/pose", Twist, self.state_callback, queue_size = 1)
        self._manager_listener = rospy.Subscriber("manager/kill", Bool, self.kill_callback, queue_size = 1)

        # Subscribers
        self._turtle_publisher = rospy.Publisher("turtle1/cmd_vel", Pose, queue_size = 1)
        self._task_status_publisher = rospy.Publisher("task/status", Bool, queue_size =- 1)

        # Internal Variables
        self._turtle_state = np.array([[0,0,0,0,0]]).T # state in matrix format of [x,y,theta,v,w].T

        self._task_status = False # False if running, true if complete

        rospy.init_node(self._node_name, rate)

    # class destructor
    def __del__(self):

        rospy.logwarn(self._node_name + " has completed its task and is shutting down!")

        rospy.signal_shutdown("The task is complete!")


    def state_callback(self, msg):

        # Collect and format the new state
        new_state = np.array([[msg.x, msg.y, msg.theta, msg.linear_velocity, msg.angular_velocity]]).T

        self._turtle_state = new_state # Make the old state the new state


    def kill_callback(self, msg):

        if msg.data == True: # if the node receives the kill signal

            self.__del__()


    # Runs the task state machine. For this parent class
    # the method should throw an error
    
    def run_task(self):

        assert False # Throw an error


# ===== MAIN METHOD ======
# This shouldn't ever be run, but is a template for how the other node main
# methods should run

if __name__ == '__main__':

    curr_task = Task("Generic Task")







    