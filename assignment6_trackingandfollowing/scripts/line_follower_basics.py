#!/usr/bin/env python3
import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from matplotlib import pyplot as plt
import math

class LineFollower(object):
    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback,queue_size=1)
    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        ##cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

        #converted to grayscale
        grayscale = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)

        #edge detection using Canny
        edges = cv2.Canny(grayscale, 50, 150, apertureSize=3)
        #plt.figure()
        #plt.title('Lines in the actual image')
        #plt.imshow(edges, cmap = 'gray')

        #hough transform
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 150, None, 0, 0)
        if lines is not None:
            plt.figure()
            plt.title('Lines in the image space')
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                cv2.line(cv_image, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
                plt.plot(pt1,pt2)


def main():
    
    line_follower_object = LineFollower()
    rospy.init_node('line_following_node', anonymous=True)
    rate = rospy.Rate(1)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()