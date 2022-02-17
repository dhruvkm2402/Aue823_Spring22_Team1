#!/usr/bin/env python3
import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist #

def callback(dt):
    thr1 = 1 # Laser scan range threshold
    thr2 = 1
    if dt.ranges[0]>thr1 and dt.ranges[15]>thr2 and dt.ranges[345]>thr2: # Checks if there are obstacles in front and
                                                                         # 15 degrees left and right (Try changing the
									 # the angle values as well as the thresholds)
        move.linear.x = 0.1 # go forward (linear velocity)
        move.angular.z = 0.0 # do not rotate (angular velocity)
    else:
        move.linear.x = 0.0 # stop
        move.angular.z = 0.0 # rotate counter-clockwise
    pub.publish(move) # publish the move object

<<<<<<< HEAD
#global variables
laser_scaninfo = None
fThreshhold = 0.5
bIsClear = True
=======
>>>>>>> 866e0a45635c8013f7157348238f4bb9d968a391

move = Twist() # Creates a Twist message type object
rospy.init_node('obstacle_avoidance_node') # Initializes a node
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Publisher object which will publish "Twist" type messages
                            				 # on the "/cmd_vel" Topic, "queue_size" is the size of the
                                                         # outgoing message queue used for asynchronous publishing

sub = rospy.Subscriber("/scan", LaserScan, callback)  # Subscriber object which will listen "LaserScan" type messages
                                                      # from the "/scan" Topic and call the "callback" function
						      # each time it reads something from the Topic

rospy.spin() # Loops infinitely until someone stops the program execution

<<<<<<< HEAD
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
=======
>>>>>>> 866e0a45635c8013f7157348238f4bb9d968a391
