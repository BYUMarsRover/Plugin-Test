#!/usr/bin/python3
# Adam Welker       BYU Mars Rover      Nov. 2022
#
# fiducial node.py -- a ros wrapper for the fiducial recognition task.
# basically just outputs the current sensor data with a 
# bool whether a fiducial tag is present in the image


import rospy
from std_msgs.msg import Bool
from fiducial_recognition import Fiducial_Tracker
import cv2 as cv


class Fiducial_Node():

    

    def __init__(self) -> None:
        
        self.fiducial_tracker = Fiducial_Tracker()

        rospy.init_node("Fiducial_Tracker")
        self.rate = rospy.Rate(20)

        self.tag_detection_publisher = rospy.Publisher("fiducials/tag_present", Bool, queue_size=1)



    def get_tag_state(self) -> None:

        return self.fiducial_tracker.detect_tag()

    def run(self)->None:

        tag_msg = Bool()

        while not rospy.is_shutdown():

            isTag = self.get_tag_state()

            tag_msg.data = isTag

            self.tag_detection_publisher.publish(tag_msg)

            if isTag:

                rospy.loginfo('QR Code is Present')

            else: 

                rospy.loginfo('No Tag Present')

            self.rate.sleep()


if __name__ == '__main__':

    fiducial_node = Fiducial_Node()
    fiducial_node.run()



    