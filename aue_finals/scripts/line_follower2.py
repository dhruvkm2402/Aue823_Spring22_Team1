#!/usr/bin/env python3
import rospy
import time
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
stop_detected = 0

class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.detect_line_publisher = rospy.Publisher('/detect_line', Int16, queue_size=10)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.stop_sign_detect = rospy.Subscriber("/detect_stop",Int16,self.stop_detection)

    def stop_detection(self,msg):
        global stop_detected
        stop_detected = msg.data
    
    def detect_shape(self, c):
        shape = ""
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)

        # Hexagon
        if len(approx) == 6:
            shape = "hexagon"

        # Octagon 
        if len(approx) == 8:
            shape = "octagon"

        return shape      

    def camera_callback(self, data):
        global stop_detected

        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
        descenter = 160
        rows_to_watch = 200 

        height, width, channels = cv_image.shape
        crop_img = cv_image[int((height)/2)+descenter:int((height)/2)+descenter+rows_to_watch][1:width]
       

       
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Yellow colour detection thresholds
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        m = cv2.moments(mask, False)
        global line_detection
        line_detection=0
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            line_detection=1
            self.detect_line_publisher.publish(line_detection)          
        except ZeroDivisionError:
            cx, cy = height/2, width/2
            self.detect_line_publisher.publish(line_detection)            

        res = cv2.bitwise_and(crop_img,crop_img,mask=mask)

        
        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

        vel_msg = Twist()

        err = cx - height/2

        vel_msg.angular.z = -float(err) /740
        vel_msg.linear.x = 0.08
        
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
        cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        for c in cnts:
        # Identify shape
            shape = self.detect_shape(c)
            if shape == 'hexagon' or shape == 'octagon':
                stop_detected = 1
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)
                
            else:
                stop_detected = 0
            rospy.loginfo("Stop detection is: %f  \n",stop_detected)    
        if ((line_detection==1) & (stop_detected!=1)):
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            self.velocity_publisher.publish(vel_msg)
        elif stop_detected==1:                                            #Acts like a counter, slowly reduced speed to zero
            for i in range(10,0,-1):
                vel_msg.linear.x = i*0.01
                self.velocity_publisher.publish(vel_msg)

            #Stopping for 4 sec after zero speed
            now = time.time()
            diff=0
            while diff<3:
                current = time.time()
                diff = current - now
            stop_detected=0

    def clean_up(self):
        cv2.destroyAllWindows()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

def main():
    rospy.init_node('line_following_node', anonymous=True)
    stop_detection =0
    line_follower_object = LineFollower()
    rate = rospy.Rate(10)
    ctrl_c = False
    def shutdownhook():
        # Works better than rospy.is_shutdown()
        line_follower_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()

stop_detected =0
if __name__ == '__main__':
        main()