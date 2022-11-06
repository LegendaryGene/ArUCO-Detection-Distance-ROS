#!/usr/bin/env python3
# license removed for brevity
import rospy
import cv2 as cv
from cv2 import aruco
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


pub = rospy.Publisher('chatter', Image, queue_size=10)
def callback(data):

    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, desired_encoding='rgb8')
    marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    param_markers = aruco.DetectorParameters_create()
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(gray_frame, marker_dict, parameters=param_markers)
    if marker_corners:
        for ids, corners in zip(marker_IDs, marker_corners):
            cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()
            cv.putText(frame,f"id: {ids[0]}",top_left,cv.FONT_HERSHEY_PLAIN,1.3,(200, 100, 0),2,cv.LINE_AA,)
    img=bridge.cv2_to_imgmsg(frame,'rgb8')
    pub.publish(img)
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
