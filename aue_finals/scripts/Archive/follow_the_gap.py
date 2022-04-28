#!/usr/bin/env python3
import rospy
from cmath import inf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from itertools import groupby
import math

def new_measurment(lidar_readings):
    distances = list(lidar_readings.ranges[300:359]) + list(lidar_readings.ranges[0:60])
    for i in range(len(distances)):
        if distances[i] == inf:
            distances[i] = 10
    #ray_angle = []
    #for i in range(len(distances)):
     #   ray_angle.append(lidar_readings.angle_min - math.pi/3  + (i*lidar_readings.angle_increment))
   
    ray_angle = np.linspace(-60*math.pi/180,60*math.pi/180, len(distances))
    min_distance = np.amin(distances)
    idx = list(np.where(distances == min_distance))
    min_angle = np.amin(ray_angle[idx])                             # Getting multiple indices, need on
    #min_angle = ray_angle[idx]                    # Need to get minimum angle right

    for i in range(len(distances)):
        len_arc = min_distance*(abs(ray_angle[i] - min_angle))
        print('len arc is:', len_arc)
        if len_arc <= 0.105:
            distances[i] = 0

    #print('Distances are:' , distances)
        
    g = groupby(distances, key=lambda x:x>0.5)
    max_gap = max([list(s) for v, s in g if v > 0.0], key=len)

    #print('Max gap is' , max_gap)
    #max_gapdist = max_gap[int(len(max_gap)/2)]
    idxm = int(len(max_gap)/2)
    max_gapdist = np.amax(max_gap)
    print('Max gap dist is:', max_gapdist)
    idx2 = list(np.where(distances == max_gapdist))
    idx2 = int(idx2[0])    
    target_angle =  ray_angle[idx2]
    error = target_angle
    
    print('Error angle is', error)

        
    kp = 0.1
    if error > 0.4 or error < -0.4:
        vel_msg.linear.x = 0
        vel_msg.angular.z = kp*error
    else:
        vel_msg.linear.x = 0.3
        vel_msg.angular.z = 0
    pub.publish(vel_msg)

if __name__ == '__main__':
    vel_msg = Twist() # Creates a Twist message type object
    rospy.init_node('Follow_Gap') # Initializes a node
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Publisher object which will publish "Twist" type messages
                                                # on the "/cmd_vel" Topic, "queue_size" is the size of the
                                                            # outgoing message queue used for asynchronous publishing

    sub = rospy.Subscriber("/scan", LaserScan, new_measurment,  queue_size=1)  # Subscriber object which will listen "LaserScan" type messages
                                                        # from the "/scan" Topic and call the "callback" function
                                # each time it reads something from the Topic

    rospy.spin() # Loops infinitely until someone stops the program execution        