#! /usr/bin/env python3

## importing ros utilities
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import math


# global variables

pub1=None # publish topic is set to be none initially
regions_laser={'front': 0, 'left':0, 'right':0} # initial values for obstacles detected in laser region 
e_current=0 # initial error
e_previous=0 # previous error for derivative part
e_total=0 # total error for integral part in case we use integral

# definig callback message for subscriber topic scan
def clbk_laser(msg):
    global regions_laser

    regions_laser= {
        'right':  min(min(msg.ranges[270:315]),10),
        'front' : min(min(msg.ranges[350:360] + msg.ranges[0:10]),10),
        'left':   min(min(msg.ranges[50:100]),10),
    }

def follow_wall():
	global regions_laser, e_current, e_previous, e_total
	
	reg_las=regions_laser # obtaining detected values for laser regions
	
	msg=Twist() # variable for storing twist velocity
	
	vx=0 # initial linear velocity
	wz=0 # initial angular velocity
	
	# gain constants
	kp=0.9 # proportional gain
	ki=0 # integral gain
	kd=5 # derivative gain
	
	# detection ranges
	
	d=0.9 # limit for detection of obstacle in the front
	
	e_current =reg_las['left']-reg_las['right']
	
	if reg_las['front'] > d: # no obstacle in specified range 
		wz=kp*(e_current)+ki*(e_total)+kd*(e_current-e_previous) # changing angular velocity based on error in bot's position w.r.t to two walls
		if reg_las['right']>2 or reg_las['left']>2: # specifying linear velocity
			vx=0.5
		else:
			vx=0.6
	else:
		wz=0.4
		vx=0
	thresh=1.4
	if wz > thresh:
        	wz = thresh
	elif wz < -thresh:
        	wz = -thresh
	
	print(e_current,e_total,e_current-e_previous)
	print(wz,vx)
	e_previous = e_current
	e_total+=e_current
	msg.linear.x = vx
	msg.angular.z = wz
	pub1.publish(msg)
	
def move():
    global pub1
    
    rospy.init_node('reading_laser_values')
    
    pub1 = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    rate = rospy.Rate(5)
    while(1):
        msg = Twist()
        follow_wall()
        rate.sleep()

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException: pass
	
	
	
	


