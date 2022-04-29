#!/usr/bin/env python3
import rospy
import time
import roslaunch
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes

class stop_detect():
    def __init__(self):
        self.rospy.init_node('stopsignnode', anonymous=True)
        self.yolo_sub = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.prediction)
        self.stop_pub = rospy.Publisher('/detect_stop', Int32, queue_size=10)
        self.rate = rospy.Rate(10)
    
    def prediction(self, data):
        global stop_sign_detect
        boxes = data.BoundingBoxes
        for box in boxes:
            identified_class=data.Class
            probability = float(box.probability)
            area = abs(data.xmax-data.xmin)*abs(data.ymax-data.ymin)
            if ((identified_class == 'stop sign')):
                stop_sign_detect = 1
                self.stop_pub.publish(stop_sign_detect)
            break
            rospy.sleep(0.1)
            
            
if __name__ == '__main__':
    stop_detect()
    rospy.spin()
   
        
        
        
          
        
