# Adam Welker       BYU Mars Rover      Nov. 2022
#
# fiducial_recognition.py -- a class that can process  the current view on a webcam and determine a 
# a fiducial AR tag is present


import numpy as np
import cv2 as cv

TIMED = False

from time import time


class Fiducial_Tracker():
    
    CAM_PORT = 0

    def __init__(self) -> None:
        
        self.camera = cv.VideoCapture(self.CAM_PORT)

        self.qr_scanner = cv.QRCodeDetector()

        self.raw = None

        self.tag_present = False



    def __del__(self) -> None:

        self.camera.release()

    def capture_image(self) -> None:

        _,self.raw = self.camera.read()

    def isTagPresent(self) -> bool:

        data,bbox,_ = self.qr_scanner.detectAndDecode(self.raw)

        if data:

            self.tag_present = True
            return True

        self.tag_present = False
        return False

    
    def detect_tag(self) -> bool:

        self.capture_image()
        cv.imshow("Raw Image", self.raw)
        self.isTagPresent()

        return self.tag_present

        



if __name__ == '__main__':

    camera = Fiducial_Tracker()
    camera.capture_image()
    

    while True:
        if TIMED:
            start = time()

        camera.capture_image()
        cv.imshow("Raw Image",camera.raw)
        camera.isTagPresent()
        
        if TIMED:
            stop = time()

        print(camera.tag_present)

        if TIMED:
            print(str(stop - start))
            print(str(1 / (stop - start)) + ' Hz')


        if cv.waitKey(1) == ord('q'):
            break

    cv.destroyAllWindows()

