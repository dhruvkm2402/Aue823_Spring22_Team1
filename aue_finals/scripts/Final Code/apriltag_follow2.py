#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveTurtlebot3
from apriltag import apriltag

class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image",Image,self.camera_callback)
        self.moveTurtlebot3_object = MoveTurtlebot3()

    def camera_callback(self, data):
        # We select bgr8 because its the OpenCV encoding by default
        image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        height, width, channels = image.shape
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Original", image)
        cv2.waitKey(1)
        detector = apriltag("tag36h11")
        detections = detector.detect(gray)
        detct = len(detections)
        
        pub_= rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        msg=Twist()
        if detct>0:
            cx,cy = detections[0]['center']
            
            k = 0.001/1.5
            angular_z = -k*(cx-width/2)
            linear_x = 0.1

            thresh = 1.5
            if angular_z > thresh:
                angular_z = thresh
            elif angular_z < -thresh:
                angular_z = -thresh
            print("Errors:")
            print([cx,cy,width/2,height/2])
            
        else:
            print("NO DETECTIONS")
            linear_x = 0
            angular_z = 0
            
        print("Controls")
        print([angular_z,linear_x])
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        pub_.publish(msg)

        #rospy.loginfo("ANGULAR VALUE SENT===>"+str(msg.angular.z))
        # Make it start turning
        #self.moveTurtlebot3_object.move_robot(msg)

    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()

def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()
    rate = rospy.Rate(100)
    
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        line_follower_object.clean_up()
        print("shutdown time!")
        ctrl_c = True
        pub_= rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        msg=Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        pub_.publish(msg)
        print("HERE!!!!!")
    
    rospy.on_shutdown(shutdownhook)
    
    rospy.spin()

if __name__ == '__main__':
        main()
