#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import struct
import numpy as np
from std_msgs.msg import Int16


class avoidance:

    def __init__(self):
        # unique node (using anonymous=True).
        rospy.init_node('avoidance', anonymous=True)

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.new_measurment)
        self.rate = rospy.Rate(10)
        self.max= 3.5

        self.detect_line_sub = rospy.Subscriber("/detect_line",Int16,self.line_detection)

    def new_measurment(self,lidar_readings):
        self.rate.sleep()

        lidar_processed_data = []
        for i in lidar_readings.ranges:
            if i>3.5:
                lidar_processed_data.append(3.5)
            else:
                lidar_processed_data.append(i)

        self.left_mean = np.mean(lidar_processed_data[20:80])
        self.right_mean = np.mean(lidar_processed_data[280:340])

        self.front = np.minimum((np.mean(lidar_processed_data[0:25])+np.mean(lidar_processed_data[335:360]))/2,self.max)

        self.front_l_cr = np.mean(lidar_processed_data[0:35])
        self.front_r_cr = np.mean(lidar_processed_data[325:360])

    def line_detection(self,msg):
            global line_detection
            line_detection = msg.data

    def move(self):
        rospy.sleep(1)              #Needed while running from launch file
        vel_msg = Twist()

        #Tuning Parameters
        # (1) threshhold, (2) critical, (3) speeds (linear and angular), (4) difference min and mean
        speed_lin = 0.15
        speed_ang = 0.3

        threshold = 1               #Need to test it in real world
        critical = 0.5              

        #let's move it
        vel_msg.linear.x = speed_lin
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()

        
        while line_detection==0:
            
            diff_ctitical = self.front_l_cr-self.front_r_cr     
            diff_mean_raw = (self.left_mean-self.right_mean)    

            if (diff_mean_raw<speed_ang) and (diff_mean_raw>(-1*speed_ang)):
                diff_mean = diff_mean_raw
            elif diff_mean_raw>speed_ang:
                diff_mean = speed_ang
            elif diff_mean_raw<(-1*speed_ang):
                diff_mean = -1*speed_ang

            
            #what if we will hit something ahead --> reduce the linear speed and rotate .. take it easy :)
            if (self.front_l_cr<critical) or (self.front_r_cr<critical):
                # Reduce linear speed and rotate
                vel_msg.linear.x = 0
                vel_msg.angular.z = diff_mean
                
            elif (self.front_l_cr<threshold and self.front_l_cr>critical) or (self.front_r_cr<threshold and self.front_r_cr>critical):
                vel_msg.linear.x = self.front*speed_lin/threshold
                vel_msg.angular.z = diff_mean
                
            else:
                vel_msg.linear.x = speed_lin
                vel_msg.angular.z = diff_mean
                
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        
        rospy.spin()

try:
    x = avoidance()
    x.move()

except rospy.ROSInterruptException:
    pass

