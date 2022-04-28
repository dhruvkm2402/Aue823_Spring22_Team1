#!/usr/bin/env python3
import rospy
import time
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int16


class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.detect_line_publisher = rospy.Publisher('/detect_line', Int16, queue_size=10)
        self.image_sub = rospy.Subscriber("/camera/image",Image,self.camera_callback)
        #self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",Image,self.camera_callback)
        self.stop_sign_detect = rospy.Subscriber("/detect_stop",Int16,self.stop_detection)

    def stop_detection(self,msg):
        global stop_detected
        stop_detected = msg.data

    def camera_callback(self, data):
        global stop_detected
        global stopped_before
        global counter_line_detect
        
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
    
        descenter = 200
        rows_to_watch = 200

        # Cropping the image to get only desired amount of data
        height, width, channels = cv_image.shape
        crop_img = cv_image[int((height)/2)+descenter:int((height)/2)+descenter+rows_to_watch][1:width+200]
       

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Threshold range to detect yello colour 
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])
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
        
        cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)
        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

        vel_msg = Twist()

        err = cx - height/2

        vel_msg.angular.z = -float(err) /740
        vel_msg.linear.x = 0.1
        
        if ((line_detection==1) & (stop_detected!=1)):
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            self.velocity_publisher.publish(vel_msg)
            rospy.loginfo('if state \n')
        elif stop_detected==1:
            vel_msg.linear.x = 0
            rospy.loginfo('Elif state \n')
            self.velocity_publisher.publish(vel_msg)
        else:
            pass
            rospy.loginfo('Else state \n')    

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