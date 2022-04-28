#!/usr/bin/env python3
from cmath import inf
from pickletools import int4

from numpy import False_, average
import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist #
from statistics import mean
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
#from move_robot import MoveTurtlebot3
from sensor_msgs.msg import Image

class Clbk_obj(object):

    def __init__(self):
        self.move = Twist() # Creates a Twist message type object
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Publisher object which will publish "Twist" type messages
        self.node = rospy.init_node('wallAndObstacle')
        self.isObsAvd = False
        self.isObsL = False
        self.isObsR = False
        self.isObsF = False
        self.frontcount = 0
        self.bridge_object = CvBridge()
        self.isLnFlw = False

    def clean_up(self):
        #self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()

class LineFollower(object):

    def __init__(self, pub):
        self.bridge_object = CvBridge()
        self.pub = pub
        self.isLnFlw = False
        #self.image_sub = rospy.Subscriber("/camera/image",Image,self.camera_callback, self.)
        #self.moveTurtlebot3_object = MoveTurtlebot3()
    
#--------------------------------------callbacks-------------------------------------------        

def LnFlw_callback(data, args):
        if(args.isLnFlw == True):
        #code
            rospy.loginfo('-------------------Inside Line Follower----------------------')
            global e11, e21, e31, u, k11, k21, k31
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = args.bridge_object.compressed_imgmsg_to_cv2(data)

            # We get image dimensions and crop the parts of the image we dont need
            height, width, pages = cv_image.shape
            print(f'The height width and depth of image are {height}, {width}, and {pages}')
            #crop_img = cv_image[int((height/2)+150):int((height/2)+170)][1:int(width)]
            crop_img = cv_image[255:][200:int(width-200)]
            #crop_img = cv_image[340:360][1:640]

            # Convert from RGB to HSV
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

            # Define the Yellow Colour in HSV

            """
            To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
            """

            # Threshold the HSV image to get only yellow colors
            
            #lower_yellow = np.array([20,100,100])
            #upper_yellow = np.array([50,255,255])
            
            lower_red = np.array([150,80,90])
            upper_red = np.array([179,200,200])
            
            mask = cv2.inRange(hsv, lower_red, upper_red)
            #mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            # Calculate centroid of the blob of binary image using ImageMoments
            m = cv2.moments(mask, False)

            try:
                cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            except ZeroDivisionError:
                cx, cy = width/2,height/2
            print(f'The coordinates of centroid of blob cx, cy are {cx}, {cy}')
            # Draw the centroid in the resultant image
            # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
            cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
            cv2.imshow("Original", cv_image)
            cv2.imshow("MASK", mask)
            cv2.waitKey(1)

            #################################
            ###   ENTER CONTROLLER HERE   ###
            #################################

            pub_= args.pub
            msg=args.move
            if cx != width/2:
                k = 0.001/1.5
                angular_z = -k*(cx-width/2)
                linear_x = 0.1
            else:
                angular_z=0.18
                linear_x = 0.0
                ###
            thresh = 1.5
            if angular_z > thresh:
                angular_z = thresh
            elif angular_z < -thresh:
                angular_z = -thresh
            print("Errors:")
            print([cx,cy,width/2,height/2])
            print("Controls")
            print([angular_z,linear_x])
            msg.linear.x = linear_x
            msg.angular.z = angular_z
            #rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))
            pub_.publish(msg)
        else:
            pass


def callback_ObsAvd(dt, args):

    if(args.isObsAvd==True and args.isLnFlw==False):
        rospy.loginfo('-------------------Inside obstacle avoidance----------------------')
        pub = args.pub
        move = args.move
        front = []
        front.extend(dt.ranges[0:10])
        front.extend(dt.ranges[350:359])
        
        left = dt.ranges[30:90]
        right = dt.ranges[270:330]
        
        threshold = 1.2               # Threshold distance
        max_vel = 0.3               # Maximum Linear Velocity
        min_dist = 0.3              # Minimum distance from obstacle where the robot must stop
        kw = 1.2                      # Proportional constant for angular velocity control
        
        nearest_front = min(min(front),10)
        
        front_error = min(front) - threshold

        #logic to switch to line follower
        front_avg = 0
        front_avg_counter = 0
        for i in range(len(front)):
            if(front[i]==inf):
                front_avg+=3
                front_avg_counter+=1
            else:
                front_avg+=front[i]
                front_avg_counter+=1
        front_avg = front_avg/front_avg_counter
        rospy.loginfo("\n front avg is: %f  \n",front_avg)
        if(front_avg==3):
            args.frontcount+=1

        #the line follower should get triggered only when the args.frontcount is greater than 10    
        if(args.frontcount>5):
            rospy.loginfo("\n ************ condition for line follower met ************ \n")
            args.isLnFlw = True
            move.linear.x = 0.1
            move.angular.z = 0.0
        #the else will keep on triggering the routine obstacle avoidance controller    
        else:
            if front_error > 0:
                move.linear.x = 0.5
                move.angular.z = 0.0
            else:
                if mean(left) > mean(right):
                    move.angular.z = -kw*front_error
                    move.linear.x = max((max_vel/(threshold - min_dist))*(nearest_front-min_dist),0)
                elif mean(right) > mean(left):
                    move.angular.z = kw*front_error
                    move.linear.x = max((max_vel/(threshold - min_dist))*(nearest_front-min_dist),0)
                    
        rospy.loginfo("The linear vel is %f & angular vel is %f\n", move.linear.x, move.angular.z)         
        pub.publish(move) # publish the move object

    else:
        pass

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

    if(args.isObsAvd==False):
        #tme = rospy.Time.now()
        #rospy.loginfo("\n wall follower callback called at \n" +str(tme.secs))
        thr_obs = 0.3 # Laser scan range threshold
        saturated_dist = 1
        left_obs = 0
        left_obs_counter = 0
        right_obs = 0
        right_obs_counter = 0
        front_obs = 0
        front_obs_counter = 0
        front_saturated_dist = 1.5

        #left section
        for i in range(30,60):
            if(dt.ranges[i]==inf):
                left_obs+=saturated_dist
                left_obs_counter+=1
            else:
                left_obs+=dt.ranges[i]
                left_obs_counter+=1
        left_obs=left_obs/left_obs_counter

        #right section
        for i in range(300,330):
            if(dt.ranges[i]==inf):
                right_obs+=saturated_dist
                right_obs_counter+=1
            else:
                right_obs+=dt.ranges[i]
                right_obs_counter+=1
        right_obs=right_obs/right_obs_counter

        #front section
        front1 = list(dt.ranges[0:5])
        front2 = list(dt.ranges[-5:])
        front_dist = front1 + front2

        for i in range(len(front_dist)):
            if(front_dist[i]==inf):
                front_obs+=front_saturated_dist
                front_obs_counter+=1
            else:
                front_obs+=front_dist[i]
                front_obs_counter+=1
        front_obs=front_obs/front_obs_counter

        if(left_obs < thr_obs):
                args.isObsL = True
                rospy.loginfo("\n left below threshold \n")
        if(right_obs < thr_obs):
                args.isObsR = True
                rospy.loginfo("\n right below threshold \n")
        if(front_obs < thr_obs):
                args.isObsF = True
                rospy.loginfo("\n front below threshold \n")

        if(args.isObsL==True and args.isObsR==True or args.isObsF==True):
                args.isObsAvd =True
                rospy.loginfo("\n ************ condition for obstacle avoidance met ************ \n")
        else:

            rospy.loginfo("\n Inside wallfollower callback \n" )
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

            #getting parameters from the object
            pub = args.pub
            move = args.move

            #left section
            for i in range(75,106):
                if(dt.ranges[i]==inf):
                    left_avg+=saturated_dist
                    left_counter+=1
                else:
                    left_avg+=dt.ranges[i]
                    left_counter+=1
            left_avg=left_avg/left_counter

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

            #rospy.loginfo("The front avg dist is %f and min dist is %f\n", front_avg, front_min)
            #rospy.loginfo("The right dist is %f and left dist is %f and difference is %f\n", right_avg, left_avg, abs(left_avg-right_avg))

            if(front_min<=0.5):
                move.linear.x = 0.0
                move.angular.z = 0.2
                #rospy.loginfo("turning. Inside outer if \n")

            elif(abs(left_avg-right_avg)<= 0.3 and 0.5<front_avg<=2):
                
                move.linear.x = 0.2
                move.angular.z = 0.0
                #rospy.loginfo("going straight. Inside outer elif \n")
                
            else:

                if(0.3 > right_avg - left_avg > margin):
                    #rospy.loginfo("Right greater. Inside if \n")
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
                    #rospy.loginfo("Left greater. Inside elif \n")
                    correction = (left_avg - right_avg)
                    correction_lin = 0.3#limit the maximum speed

                    e31 = e21
                    e21 = e11
                    e11 = correction
                    u = u + k11*e11 + k21*e21 + k31*e31

                    move.angular.z = u
                    move.linear.x = correction_lin
                    
                    corr_ang_list[0] =  move.angular.z
                    
                elif(0.75 > right_avg - left_avg >0.3):
                    #rospy.loginfo("Right too great. Inside elif2 \n")
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
                    #rospy.loginfo("Left too great. Inside elif3 \n")
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
                    #rospy.loginfo("Almost equal. Inside else \n")

            #rospy.loginfo("The linear vel is %f & angular vel is %f\n", move.linear.x, move.angular.z)
            pub.publish(move) # publish the move object
        #outer else ending
    else:
        pass

def main():
    wlflw_obj = Clbk_obj()
    
    #create two dummy callbacks and put their data in a global variable
    #create an evaluation condition for the variable's value at the start and print it with timestamps
    try:
        sub_wlflw = rospy.Subscriber("/scan", LaserScan, callback_WlFlw,  wlflw_obj, queue_size=1)  # Subscriber object which will listen "LaserScan" type messages
                                                        # from the "/scan" Topic and call the "callback" function
                                                        # each time it reads something from the Topic
        sub_ObsAvd = rospy.Subscriber("/scan", LaserScan, callback_ObsAvd,  wlflw_obj, queue_size=1)  # Subscriber object which will listen "LaserScan" type messages
                                                        # from the "/scan" Topic and call the "callback" function
                                                        # each time it reads something from the Topic
        sub_LnFlw = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, LnFlw_callback,  wlflw_obj, queue_size=1)  
                                                        # Subscriber object which will listen "CompressedImage" type messages
                                                        # from the "/camera/rgb/image_raw/compressed" Topic and call the "callback" function
                                                        # each time it reads something from the Topic
        rospy.spin() # Loops infinitely until someone stops the program execution
    except rospy.ROSInterruptException:
        pass
    
    rospy.loginfo("Coming out of first callback \n")

if __name__ == '__main__':
    main()
    #rospy.spin() # Loops infinitely until someone stops the program execution

