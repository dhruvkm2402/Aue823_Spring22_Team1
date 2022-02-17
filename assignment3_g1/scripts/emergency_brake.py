#!/usr/bin/env python3

from math import pi
from tkinter import TRUE
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#global variables
laser_scaninfo = None
fThreshhold = 5
bIsClear = True

def callback(data):
    global laser_scaninfo
    global bIsClear
    global fThreshhold
    laser_scaninfo = data.ranges[0]
    #rospy.loginfo("The min angle is %f \n", laser_scaninfo.angle_min)
    #rospy.loginfo("The max angle is %f \n", laser_scaninfo.angle_max)
    #rospy.loginfo("The type is %s \n", type(laser_scaninfo))
        #rospy.loginfo("The length is %d \n", len(laser_scaninfo.ranges))
        #rospy.loginfo("The forward point index is %d \n", round((len(laser_scaninfo.ranges)/4)))
        #nparr =np.array(laser_scaninfo.ranges)
        #fCenterVal = laser_scaninfo.ranges[0]
    #fCenterVal = laser_scaninfo.ranges[round((len(laser_scaninfo.ranges)/4))]
    rospy.loginfo("the forward distance val is %f \n", laser_scaninfo)

    if(laser_scaninfo < fThreshhold):
        bIsClear = False
        rospy.loginfo("the distance is below threshold")
    #rospy.loginfo("I heard %s \n",data.data)
    #rospy.loginfo("data.data type is %s \n", type(data.data))
    #return TRUE
     
def move():
    global bIsClear
    rospy.init_node('turtlebot3_gazebo', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    rospy.loginfo("Let's move your robot")
    speed = 0.2
    current_angle = 0
    angular_speed=0.1
    
    while not rospy.is_shutdown():

        #t0 = rospy.Time.now().to_sec()
        #while (current_angle < pi):
        #        vel_msg.angular.z=angular_speed
        #    #Publish the velocity
        #        velocity_publisher.publish(vel_msg)
        #    #Takes actual time to velocity calculus
         #       t1=rospy.Time.now().to_sec()
         #   #Calculates distancePoseStamped
         #       current_angle= angular_speed*(t1-t0)
         #       rospy.loginfo("Rotating")
        #After the loop, stops the robot
        #vel_msg.angular.z = 0
        #Force the robot to stop
        #velocity_publisher.publish(vel_msg)
        
        #Loop to move the turtle on a side of square
        while(bIsClear):

            rospy.Subscriber('/scan', LaserScan, callback)
            rospy.loginfo("boolean is %r \n",bIsClear)
            vel_msg.linear.x = abs(float(speed))
            #print("inside distance criterion:"+str(vel_msg.linear.x))
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
        if(bIsClear!=True):
            vel_msg.linear.x = 0
            velocity_publisher.publish(vel_msg)
            rospy.loginfo("Obstacle in Front. Emergency Brake applied! \n")
        rospy.spin()
if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass