#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897

def rotate():
    #Starts a new node
    rospy.init_node('turtlebot3', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Receiveing the user's input
    # print("Let's rotate your robot")
    #speed = float(input("Input your speed (degrees/sec):"))
    speed = 0.1
    #angle = input("Type your distance (degrees):")
    angle = 600
    #clockwise = input("Clockwise?: ") #True or false
    clockwise = True

    #Converting from angles to radians
    relative_angle = float(angle*2*PI/360)

    #We wont use linear components except for x-direction v=r*w
    vel_msg.linear.x=speed
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(speed*3)
    else:
        vel_msg.angular.z = abs(speed*3)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = speed*(t1-t0)
    #Forcing our robot to stop
    vel_msg.angular.z = 0
    vel_msg.linear.x=0
    velocity_publisher.publish(vel_msg)
    rospy.spin()

if __name__ == '__main__':
    try:
        # Testing our function
        rotate()
    except rospy.ROSInterruptException:
        pass
