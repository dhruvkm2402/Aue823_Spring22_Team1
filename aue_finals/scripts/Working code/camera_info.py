#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import message_filters


def callback(image, camera_info):
    info_pub.publish(camera_info)

rospy.init_node('camerainfo', anonymous=True)
info_pub = rospy.Publisher('/camera/rgb/camera_info', CameraInfo, queue_size=10)
image_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
info_sub = message_filters.Subscriber('/raspicam_node/camera_info', CameraInfo)

sync = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
sync.registerCallback(callback)
rospy.spin()
