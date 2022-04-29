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
count = 0

class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.detect_line_publisher = rospy.Publisher('/detect_line', Int16, queue_size=10)
<<<<<<< HEAD
        self.image_sub = rospy.Subscriber("/camera/image",Image,self.camera_callback)
        #self.stop_sign_detect = rospy.Subscriber("/detect_stop",Int16,self.stop_detection)
=======
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.stop_sign_detect = rospy.Subscriber("/detect_stop",Int16,self.stop_detection)

    def stop_detection(self,msg):
        global stop_detected
        stop_detected = msg.data
>>>>>>> 0c32652b68fdfcde6100a6a8e45a8719b04a48fc
    
    def detect_shape(self, approx):
        x, y, w, h = cv2.boundingRect(approx)
        if len(approx) == 8:
            s = "Octagon"
        else:
            s = "None"
            
        return s, x, y, w, h

    def get_contours(self, img, img_contour):
        contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]
        shape = "None"

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 9000:
                cv2.drawContours(img_contour, cnt, -1, (255, 0, 255), 1)
                param = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.01 * param, True)
                shape, x, y, w, h = self.detect_shape(approx)
        return shape              

    def camera_callback(self, data):
        #global stop_detected
        global count

        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
<<<<<<< HEAD
        descenter = 200
        rows_to_watch = 200 

        height, width, channels = cv_image.shape
        crop_img = cv_image[int((height)/2)+descenter:int((height)/2)+descenter+rows_to_watch][1:width+200]
        vel_msg = Twist()
=======
        descenter = 160
        rows_to_watch = 200 

        height, width, channels = cv_image.shape
        crop_img = cv_image[int((height)/2)+descenter:int((height)/2)+descenter+rows_to_watch][1:width]
       
>>>>>>> 0c32652b68fdfcde6100a6a8e45a8719b04a48fc

       
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
            cy, cx = height/2, width/2
            #vel_msg.angular.z = 0.05
            #self.velocity_publisher.publish(vel_msg)
            self.detect_line_publisher.publish(line_detection)            

        res = cv2.bitwise_and(crop_img,crop_img,mask=mask)

        
        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

        

        err = cx - height/2

        vel_msg.angular.z = -float(err) /740
        vel_msg.linear.x = 0.08
        
        img_contour = cv_image
        
        img_blur = cv2.GaussianBlur(cv_image, (7,7),1)
        img_gray = cv2.cvtColor(img_blur, cv2.COLOR_BGR2GRAY)
        
        img_canny = cv2.Canny(img_gray, 100, 200)
        
#        cv2.imshow('thresholds',img_canny)
        
        kernel = np.ones((3))
        img_dilated = cv2.dilate(img_canny, kernel, iterations = 1)
        sha = self.get_contours(img_dilated, img_contour)
        if sha!= 'None':
            stop_detected = 1
            rospy.loginfo('Stop Sign Detected /n')
        else:
            stop_detected = 0    

        if ((line_detection==1) & (stop_detected!=1)):
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            self.velocity_publisher.publish(vel_msg)
            
        elif stop_detected==1 & line_detection==1:                                            #Acts like a counter, slowly reduced speed to zero
            #rospy.loginfo('Going into elif /n')
            count += 1
            rospy.loginfo('Count is %d /n' , count)
            if count >= 40:
                rospy.sleep(2)
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)
            #vel_msg.linear.x = 0
            #vel_msg.angular.z = 0
            #self.velocity_publisher.publish(vel_msg)

            #Stopping for 4 sec after zero speed
            #now = time.time()
            #diff=0
            #while diff<3:
             #   current = time.time()
              #  diff = current - now
            #stop_detected=0
        else:
            pass
    

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


if __name__ == '__main__':
        main()