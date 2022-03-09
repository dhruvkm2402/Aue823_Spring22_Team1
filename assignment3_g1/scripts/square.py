#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from math import pi
speed =0.1
angular_speed=0.1
# length of square in units
side_length=2
def move():
    # Starts a new node named sqaure_ol with unique id
    rospy.init_node('Square_ol', anonymous=True)
    # creating a handle for publishing a message to topic
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # creating a variable to store twist message
    vel_msg = Twist()
    while not rospy.is_shutdown():
        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0
        current_angle=0
        turn=0
        while turn<4:
            #Loop to move the turtle in an specified distance
            while (current_distance < side_length):
                vel_msg.linear.x=speed
            #Publish the velocity
                velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
                t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
                current_distance= speed*(t1-t0)
        #After the loop, stops the robot
            vel_msg.linear.x = 0
        #Force the robot to stop
            velocity_publisher.publish(vel_msg)
            # resetting time
            t0 = rospy.Time.now().to_sec()
        # to move in circular direction
            while (current_angle < pi/2):
                vel_msg.angular.z=angular_speed
            #Publish the velocity
                velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
                t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
                current_angle= angular_speed*(t1-t0)
        #After the loop, stops the robot
            vel_msg.angular.z = 0
        #Force the robot to stop
            velocity_publisher.publish(vel_msg)
        # resetting the current angle,distance and t0
            current_angle=0
            current_distance=0
            t0 = rospy.Time.now().to_sec()
            turn+=1
        rospy.spin()
if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass

