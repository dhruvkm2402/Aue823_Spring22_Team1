#!/usr/bin/env python3
from cmath import inf

from numpy import False_, average
import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist #
from statistics import mean

corr_ang_list = [0.0]
counter = 0
bElseloop = True

def callback(dt):
    global corr_ang_list, counter, bElseloop
    thr1 = 1 # Laser scan range threshold
    thrleft = 0.8
    Kp = 1.5
    margin = 0.1
    saturated_dist = 3
    left_avg = 0
    left_counter = 0
    right_avg = 0
    right_counter = 0

    for i in range(75,106):
        if(dt.ranges[i]==inf):
            left_avg+=saturated_dist
            left_counter+=1
        else:
            left_avg+=dt.ranges[i]
            left_counter+=1
    left_avg=left_avg/left_counter

    for i in range(255,286):
        if(dt.ranges[i]==inf):
            right_avg+=saturated_dist
            right_counter+=1
        else:
            right_avg+=dt.ranges[i]
            right_counter+=1
    right_avg=right_avg/right_counter

    #left_avg = mean(dt.ranges[75:105])
    #right_avg = mean(dt.ranges[255:290])

    if(left_avg==inf):
        left_avg==saturated_dist
    if(right_avg==inf):
        right_avg==saturated_dist
    
    rospy.loginfo("The right dist is %f and left dist is %f and difference is %f\n", right_avg, left_avg, abs(left_avg-right_avg))

    if(dt.ranges[75]>thrleft or dt.ranges[105]>thrleft):
    
        
        if(0.65 > right_avg - left_avg > margin):
            rospy.loginfo("Right greater. Inside if \n")
            correction = Kp*(right_avg - left_avg)
            correction_lin = min(correction/2, 0.45)#limit the maximum speed
            correction_ang = min(correction/3, 0.3)#limit the maximum speed
            move.linear.x = correction_lin # go forward (linear velocity)
            move.angular.z = -correction_ang   #turn clockwise
            corr_ang_list[0] =  move.angular.z
            bElseloop = True
        elif(0.65 > left_avg - right_avg > margin):
            rospy.loginfo("Left greater. Inside elif \n")
            correction = Kp*(left_avg - right_avg)
            correction_lin = min(correction/2, 0.45)#limit the maximum speed
            correction_ang = min(correction/3, 0.3)#limit the maximum speed
            move.linear.x = correction_lin # go forward (linear velocity)
            move.angular.z = correction_ang #turn anti-clockwise
            corr_ang_list[0] =  move.angular.z
            bElseloop = True
            #move.linear.x = 0.2 # go forward (linear velocity)
            #move.angular.z = 0.1
        elif(right_avg - left_avg > 0.65):
            rospy.loginfo("Right too great. Inside elif2 \n")
            correction = Kp*(right_avg - left_avg)
            correction_lin = min(correction/3, 0.3)#limit the maximum speed
            correction_ang = min(correction/2, 0.4)#limit the maximum speed
            move.linear.x = correction_lin # go forward (linear velocity)
            move.angular.z = -correction_ang #turn clockwise
            corr_ang_list[0] =  move.angular.z
            bElseloop = True
        elif(left_avg - right_avg > 0.65):
            rospy.loginfo("Left too great. Inside elif3 \n")
            correction = Kp*(left_avg - right_avg)
            correction_lin = min(correction/3, 0.3)#limit the maximum speed
            correction_ang = min(correction/2, 0.4)#limit the maximum speed
            move.linear.x = correction_lin # go forward (linear velocity)
            move.angular.z = correction_ang #turn anti-clockwise
            corr_ang_list[0] =  move.angular.z
            bElseloop = True
        else:
            move.linear.x = 0.2
            move.angular.z = -corr_ang_list[0]
            #if(counter<10 and bool(bElseloop/)):
                #move.angular.z = -corr_ang_list[0]
                #corr_ang_list[0]=-corr_ang_list[0]/2
                #counter+=1
            #else:
                #counter=0
                #move.angular.z = 0
                #bElseloop = False
            rospy.loginfo("Almost equal. Inside else \n")
    else:
        move.linear.x = 0.2
        rospy.loginfo("equal. Inside outer else \n")

    rospy.loginfo("The linear vel is %f & angular vel is %f\n", move.linear.x, move.angular.z)
    pub.publish(move) # publish the move object


if __name__ == '__main__':
    move = Twist() # Creates a Twist message type object
    rospy.init_node('wallfollowing') # Initializes a node
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Publisher object which will publish "Twist" type messages
                                                # on the "/cmd_vel" Topic, "queue_size" is the size of the
                                                            # outgoing message queue used for asynchronous publishing

    sub = rospy.Subscriber("/scan", LaserScan, callback,  queue_size=1)  # Subscriber object which will listen "LaserScan" type messages
                                                        # from the "/scan" Topic and call the "callback" function
                                # each time it reads something from the Topic

    rospy.spin() # Loops infinitely until someone stops the program execution

