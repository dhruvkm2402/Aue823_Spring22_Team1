#!/usr/bin/env python3
from cmath import inf
import numpy as np
from numpy import False_, average
import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist #
from statistics import mean

class Clbk_obj(object):

    def __init__(self, pub, move):
        self.move = move
        self.pub = pub


def callback_ObsAvd(dt, args):
    pub = args.pub
    move = args.move
    scans = list(dt.ranges)
    #scans = np.array(scans)
    #front = []
    #front.extend(scans[0:10])
    #front.extend(scans[350:359])
    frontleft = scans[0:10]
    frontright = scans[350:359]
    front = np.concatenate((frontleft, frontright))
    
    #left = dt.ranges[30:90]
    #right = dt.ranges[270:330]
    
    left = scans[30:90]    
    right = scans[270:330]
    
    threshold = 0.5               # Threshold distance
    max_vel = 0.3               # Maximum Linear Velocity
    min_dist = 0.3              # Minimum distance from obstacle where the robot must stop
    kw = 1.2                      # Proportional constant for angular velocity control
    
    front = np.clip(front, min_dist,10)
    left = np.clip(left, min_dist, 10)
    right = np.clip(right, min_dist, 10)
    
    nearest_front = np.mean(front)
    
    front_error = nearest_front - threshold
    
    left_gap = np.array(np.where(left > threshold))
    right_gap = np.array(np.where(right > threshold))
    
    farleft = np.argmax(left)
    farright = np.argmax(right)
    
    if front_error > 0:
        move.linear.x = 0.5
        move.angular.z = 0.0
    else:
        if left_gap.shape[0] > right_gap.shape[0]:
            move.angular.z = -kw*farleft
            move.linear.x = max((max_vel/(threshold - min_dist))*(nearest_front-min_dist),0)
        elif right_gap.shape[0] > left_gap.shape[0]:
            move.angular.z = kw*farright
            move.linear.x = max((max_vel/(threshold - min_dist))*(nearest_front-min_dist),0)
            
    pub.publish(move) # publish the move object

corr_ang_list = [0.0]
counter = 0
bElseloop = True
thrcounter = 0
thrleft = 0
thr = 0
e11,e21,e31 = 0,0,0
u = 0

def callback_WlFlw(dt, args):
    global corr_ang_list, counter, bElseloop, thrcounter, thrleft, e11, e21, e31, u, thr

    pub = args.pub
    move = args.move

    margin = 0.1
    saturated_dist = 1
    left_avg = 0
    left_counter = 0
    right_avg = 0
    right_counter = 0
    front_avg = 0
    front_min = 0
    front_counter = 0
    front_saturated_dist = 1.5

    #creating a discretized pid controller
    kp = 0.4
    ki = 0.0
    kd = 0.3
    k11 = kp + ki + kd
    k21 = -kp - 2*kd
    k31 = kd

    #left section
    for i in range(75,106):
        if(dt.ranges[i]==inf):
            left_avg+=saturated_dist
            left_counter+=1
        else:
            left_avg+=dt.ranges[i]
            left_counter+=1
    left_avg=left_avg/left_counter

    #if(thrcounter < 1):
    #    thrleft = left_avg
    #    thr = abs(left_avg-right_avg)
    #    rospy.loginfo("\n The left threshold is %f & overall threshold is %f\n", thrleft, thr)
    #    thrcounter+=1

    #right section
    for i in range(255,286):
        if(dt.ranges[i]==inf):
            right_avg+=saturated_dist
            right_counter+=1
        else:
            right_avg+=dt.ranges[i]
            right_counter+=1
    right_avg=right_avg/right_counter

    #front section
    front1 = list(dt.ranges[0:10])
    front2 = list(dt.ranges[-11:])
    front_dist = front1 + front2

    for i in range(len(front_dist)):
        if(front_dist[i]==inf):
            front_avg+=front_saturated_dist
            front_counter+=1
        else:
            front_avg+=front_dist[i]
            front_counter+=1
    front_avg=front_avg/front_counter

    #using the minimum front
    front_min = min(front_dist)

    #left_avg = mean(dt.ranges[75:105])
    #right_avg = mean(dt.ranges[255:290])

    if(left_avg==inf):
        left_avg==saturated_dist
    if(right_avg==inf):
        right_avg==saturated_dist

    rospy.loginfo("The front avg dist is %f and min dist is %f\n", front_avg, front_min)
    rospy.loginfo("The right dist is %f and left dist is %f and difference is %f\n", right_avg, left_avg, abs(left_avg-right_avg))

    if(front_min<=0.5):
        move.linear.x = 0.0
        move.angular.z = 0.2
        rospy.loginfo("turning. Inside outer if \n")

    elif(abs(left_avg-right_avg)<= 0.3 and 0.5<front_avg<=2):
        
        move.linear.x = 0.2
        move.angular.z = 0.0
        rospy.loginfo("going straight. Inside outer elif \n")
        
    else:

        if(0.3 > right_avg - left_avg > margin):
            rospy.loginfo("Right greater. Inside if \n")
            correction = right_avg - left_avg
            correction_lin = 0.3#limit the maximum speed

            e31 = e21
            e21 = e11
            e11 = correction
            u = u + k11*e11 + k21*e21 + k31*e31

            move.angular.z = -u
            move.linear.x = correction_lin

            corr_ang_list[0] =  move.angular.z
            #bElseloop = True
        elif(0.3 > left_avg - right_avg > margin):
            rospy.loginfo("Left greater. Inside elif \n")
            correction = (left_avg - right_avg)
            correction_lin = 0.3#limit the maximum speed

            e31 = e21
            e21 = e11
            e11 = correction
            u = u + k11*e11 + k21*e21 + k31*e31

            move.angular.z = u
            move.linear.x = correction_lin
            
            corr_ang_list[0] =  move.angular.z
            #bElseloop = True
            #move.linear.x = 0.2 # go forward (linear velocity)
            #move.angular.z = 0.1
        elif(0.75 > right_avg - left_avg >0.3):
            rospy.loginfo("Right too great. Inside elif2 \n")
            correction = right_avg - left_avg
            correction_lin = 0.3#limit the maximum speed

            e31 = e21
            e21 = e11
            e11 = correction
            u = u + k11*e11 + k21*e21 + k31*e31

            move.angular.z = -u
            move.linear.x = correction_lin

            corr_ang_list[0] =  move.angular.z
            #bElseloop = True
        elif(0.75 > left_avg - right_avg >0.3):
            rospy.loginfo("Left too great. Inside elif3 \n")
            correction = (left_avg - right_avg)
            correction_lin = 0.3#limit the maximum speed

            e31 = e21
            e21 = e11
            e11 = correction
            u = u + k11*e11 + k21*e21 + k31*e31

            move.angular.z = 0.75*u
            move.linear.x = 0.75*correction_lin

            corr_ang_list[0] =  move.angular.z
            #bElseloop = True
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

    rospy.loginfo("The linear vel is %f & angular vel is %f\n", move.linear.x, move.angular.z)
    pub.publish(move) # publish the move object

def wallfollower_fn():
    move = Twist() # Creates a Twist message type object
    rospy.init_node('wallfollowing') # Initializes a node
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Publisher object which will publish "Twist" type messages
                                                # on the "/cmd_vel" Topic, "queue_size" is the size of the
                                                            # outgoing message queue used for asynchronous publishing

    #callback arguments storing object
    wlflw_obj = Clbk_obj(pub,move)

    sub = rospy.Subscriber("/scan", LaserScan, callback_WlFlw,  wlflw_obj, queue_size=1)  # Subscriber object which will listen "LaserScan" type messages
                                                        # from the "/scan" Topic and call the "callback" function
                                                        # each time it reads something from the Topic


def obstacleAvoidance_fn():
    move = Twist() # Creates a Twist message type object
    rospy.init_node('obstacle_avoidance_node')# Initializes a node
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Publisher object which will publish "Twist" type messages
                                                # on the "/cmd_vel" Topic, "queue_size" is the size of the
                                                            # outgoing message queue used for asynchronous publishing

    #callback arguments storing object
    ObsAvd_obj = Clbk_obj(pub,move)

    sub = rospy.Subscriber("/scan", LaserScan, callback_ObsAvd,  ObsAvd_obj, queue_size=1)  # Subscriber object which will listen "LaserScan" type messages
                                                        # from the "/scan" Topic and call the "callback" function
                                                        # each time it reads something from the Topic

if __name__ == '__main__':
    value = input("Please enter a string:\n")

    if value == "1":
        wallfollower_fn()

    if value == "2":
        obstacleAvoidance_fn()

    rospy.spin() # Loops infinitely until someone stops the program execution

