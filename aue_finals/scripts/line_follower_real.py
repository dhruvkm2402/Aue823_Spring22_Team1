#!/usr/bin/env python3
import rospy
import time
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Int16
from cmath import inf

frontcount = 0

class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.detect_line_publisher = rospy.Publisher('/detect_line', Int16, queue_size=10)
        self.image_sub = rospy.Subscriber("/camera/image",Image,self.camera_callback)
        #self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",Image,self.camera_callback)
        self.stop_sign_detect = rospy.Subscriber("/detect_stop",Int16,self.stop_detection)
        self.distance_detect = rospy.Subscriber('/scan', LaserScan, self.distance_detection)

    def stop_detection(self,msg):
        global stop_detected
        stop_detected = msg.data
    def distance_detection(self, dt):
        global frontcount
        #distance = np.mean(np.array(dt.ranges[20:90]))

        front = []
        front.extend(dt.ranges[0:20])
        front.extend(dt.ranges[350:359])

        front_avg = 0
        front_avg_counter = 0
        for i in range(len(front)):
            if(front[i]==0):
                front_avg+=3
                front_avg_counter+=1
            else:
                front_avg+=front[i]
                front_avg_counter+=1
        front_avg = front_avg/front_avg_counter
        #rospy.loginfo("\n front avg is: %f  \n",front_avg)
        if(front_avg>3):
            frontcount+=1   

    def camera_callback(self, data):
        global stop_detected
        global stopped_before
        global counter_line_detect
        global frontcount
        
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
    
        descenter = 160
        rows_to_watch = 400 

        # Cropping the image to get only desired amount of data
        height, width, channels = cv_image.shape
        crop_img = cv_image[int((height)/2)+descenter:int((height)/2)+descenter+rows_to_watch][1:width+200]
       

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Threshold range to detect yello colour
        if frontcount < 5: 
            lower_yellow = np.array([20,100,100])
            upper_yellow = np.array([50,255,255])
            #rospy.loginfo("\n the value of frontcount in IF is %d \n", frontcount)
        else:
            lower_yellow = np.array([10,50,50])
            upper_yellow = np.array([255,255,255])
            #rospy.loginfo("\n the value of frontcount in ELSE is %d \n", frontcount)    
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        global line_detection
        line_detection=0
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            line_detection=1
            self.detect_line_publisher.publish(line_detection)             # to let other nodes know that line is detected
        except ZeroDivisionError:
            cx, cy = height/2, width/2
            self.detect_line_publisher.publish(line_detection)             # to let other nodes know that line is not detected

        res = cv2.bitwise_and(crop_img,crop_img,mask=mask)

        # Draw the centroid on the image
        
        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

        vel_msg = Twist()

        err = cx - height/2

        vel_msg.angular.z = -float(err) /780
        vel_msg.linear.x = 0.1
        
        global stop_detected
        if ((line_detection==1) & (stop_detected!=1)):
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            self.velocity_publisher.publish(vel_msg)
        elif stop_detected==1:
            for i in range(5,0,-2):
                vel_msg.linear.x = i*0.01
                self.velocity_publisher.publish(vel_msg)

            #stop for 3 seconds at stop sign
            now = time.time()
            diff=0
            while diff<4:
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

counter_line_detect = 0
stop_detected =0
stopped_before =0
if __name__ == '__main__':
        main()