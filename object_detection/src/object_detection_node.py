#!/usr/bin/env python
#Subscribe to /camera/image_raw MSG: sensor_msg/camera_info
import rospy
import argparse
import imutils
import cv2
import numpy as np 
import time
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ObjectDetection:

    def __init__(self):
        rospy.init_node('object_detection_node', anonymous=True)
        #rospy.Subscriber("tf",TFMessage, self.callback)
        
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.callback, queue_size=1)
        self.bridge = CvBridge()
        self.img = None
        print("bussy")
    
        self.test = 0
        #check for space or esc
        #while not rospy.is_shutdown() and not cv2.waitKey(1):
        while not rospy.is_shutdown():
            if self.img is not None:
                cv2.imshow("Direct", self.img)
                k = cv2.waitKey(3)
                if k != -1:
                    self.object_detection()
            print("reached")
            time.sleep(0.01)
            
    def object_detection(self):
        #statics:
        print("done")
        #Threshold for drawing contours
        MIN_THRESH = 50 #100
        #Dimension test square to measure mm/pixel ratio
        l_square = 40 #mm
        w_square = 40 #mm
        #calcutaing mm/pixel ratio from measurements:
        l_per_pix = l_square/73.9158935547 #in mm per pix
        w_per_pix = w_square/74.7047805786 #in mm per pix
        #MidPoint in pixels
        x_mid = 320 # in pixels
        y_mid = 240 # in pixels
        
        img = self.img 
        blurred = cv2.bilateralFilter(self.img ,6,75,75)#9,75,75
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 0, 200) #90,120
        #draw midpoint circle
        cv2.circle(img, (x_mid, y_mid), 7, (0, 255, 0), -1)    
        # find contours in the thresholded image
        cnts = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # loop over the contours
        for c in cnts:
            if cv2.contourArea(c) > MIN_THRESH:
                # compute the center of the contour
                M = cv2.moments(c)
                if(M["m00"] == 0): 
                    M["m00"]=1
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                print("Center: cX = {} cY = {}".format(cX,cY))
                # draw the contour and center of the shape on the image
                cv2.drawContours(img, [c], -1, (255, 0, 0), 1)
                cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)
                #cv2.putText(img, "center", (cX - 20, cY - 20),
                    #cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                rect = cv2.minAreaRect(c)
                #print("square format: {}".format(rect))
                dimen = rect[1]
                l_pix = dimen[0]
                w_pix = dimen[1]
                #printin real lenght of object
                real_length = l_pix * l_per_pix
                real_width = w_pix * w_per_pix
                print("Object lenght = {} mm, Object width = {} mm".format(real_length,real_width))
                #calculate coordinate from midpoint
                x_orientation = cX - x_mid  #in pixels
                y_orientation = y_mid - cY#in pixels
                x_orientation = x_orientation*l_per_pix
                y_orientation = y_orientation*w_per_pix
                print("X orientation = {} mm, Y orientation = {} mm".format(x_orientation,y_orientation))
                degrees = rect[2]
                radians = ((degrees*math.pi)/180)
                print("Theta in radians: {}.".format(radians))
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(img,[box],0,(0,0,255),2)
        cv2.imshow("img", img)
  
    def callback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.test = self.test + 1
        except CvBridgeError as e:
            print(e)
        (rows,cols,channels) = self.cv_image.shape
        self.img = self.cv_image

if __name__ == '__main__':
    try:
        ObjectDetection()
    except rospy.ROSInterruptException: pass