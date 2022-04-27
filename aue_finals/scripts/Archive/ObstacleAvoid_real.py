#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import struct
import numpy as np
from std_msgs.msg import Int16


class avoidance:

    def __init__(self):
        # Triggering its own node avoidance
        rospy.init_node('avoidance', anonymous=True)

        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.new_measurment)
        self.rate = rospy.Rate(10)
        self.max= 3.5

        self.detect_line_sub = rospy.Subscriber("/detect_line",Int16,self.line_detection)

    def new_measurment(self,lidar_readings):
        self.rate.sleep()
        lidar_data = []
        for i in lidar_readings.ranges:
            if i==0 or i>3.5:
                lidar_data.append(3.5)
            else:
                lidar_data.append(i)

        self.left_mean = np.minimum(np.mean(lidar_data[30:70]),self.max)                                          #Need to tune in real world based on track
        self.right_mean = np.minimum(np.mean(lidar_data[290:330]), self.max)

        self.front = np.minimum((np.mean(lidar_data[0:25])+np.mean(lidar_data[335:360]))/2,self.max)

        self.front_l_cr = np.minimum((np.min(lidar_data[0:35])),self.max)
        self.front_r_cr = np.minimum((np.min(lidar_data[325:360])),self.max)
        self.front_cr = np.minimum(((np.mean(lidar_data[355:360]))+(np.mean(lidar_data[0:5])))/2,self.max)

    def line_detection(self,msg):
         global line_detection
         line_detection = msg.data

    def move(self):
        rospy.sleep(3)              #Needed while running from launch file
        vel_msg = Twist()

        #Need to tune parameters below based on track
        
        speed_lin = 0.2
        speed_ang = 0.5

        threshold = 1.2         
        critical = 0.4            
        critical_front = 0.3

        vel_msg.linear.x = speed_lin
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()

        diff_mean=speed_ang
        while line_detection==0:
            diff_mean_raw = (self.left_mean-self.right_mean)   #used in regular conditions
            diff_mean_previous = diff_mean

            if (diff_mean_raw<speed_ang) and (diff_mean_raw>(-1*speed_ang)):
                diff_mean = diff_mean_raw
            elif diff_mean_raw>speed_ang:
                diff_mean = speed_ang
            elif diff_mean_raw<(-1*speed_ang):
                diff_mean = -1*speed_ang

            if diff_mean<0.2 and diff_mean>-0.2:
                diff_mean = diff_mean_previous

            if (self.front_l_cr<critical) or (self.front_r_cr<critical) or (self.front_cr<critical_front):
                vel_msg.linear.x = 0
                vel_msg.angular.z = diff_mean/(abs(diff_mean)+0.00000000000001)*speed_ang

            elif (self.front_l_cr<threshold and self.front_l_cr>critical) or (self.front_r_cr<threshold and self.front_r_cr>critical_front):
                # Reduce linear speed and rotate
                vel_msg.linear.x = np.minimum((self.front*speed_lin/threshold), speed_lin)
                if vel_msg.linear.x>0.17:
                    vel_msg.angular.z = 0
                else:
                    vel_msg.angular.z = 0
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