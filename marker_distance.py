#!/usr/bin/env python3
# license removed for brevity
import rospy
import cv2 as cv
from cv2 import aruco
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
import numpy as np
import message_filters

pub = rospy.Publisher('chatter', Image, queue_size=10)
def callback(cam,data):

    cam_mat = np.array(cam.K).reshape([3, 3])
    dist_coef = np.array(cam.D)
    MARKER_SIZE = 8
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, desired_encoding='rgb8')
    marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    param_markers = aruco.DetectorParameters_create()

    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(gray_frame, marker_dict, parameters=param_markers)
    if marker_corners:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, MARKER_SIZE, cam_mat, dist_coef)
        total_markers = range(0, marker_IDs.size)

        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()
            distance = np.sqrt(tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2)
            point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
            cv.putText(frame,f"id: {ids[0]}",top_left,cv.FONT_HERSHEY_PLAIN,1.3,(200, 100, 0),2,cv.LINE_AA,)
            cv.putText(frame,f"id: {ids[0]} Dist: {round(distance, 2)}",top_right,cv.FONT_HERSHEY_PLAIN,1.3,(0, 0, 255),2,cv.LINE_AA,)
            cv.putText(frame,f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",bottom_right,cv.FONT_HERSHEY_PLAIN,1.0,(0, 0, 255),2,cv.LINE_AA,)

    img=bridge.cv2_to_imgmsg(frame,'rgb8')
    pub.publish(img)

def listener():

    rospy.init_node('listener', anonymous=True)
    cam_info=message_filters.Subscriber('/camera/color/camera_info', CameraInfo)
    img=message_filters.Subscriber('/camera/color/image_raw', Image)
    ts=message_filters.TimeSynchronizer([cam_info,img],10,1)
    ts.registerCallback(callback)                                                                                                     
    rospy.spin()

if __name__ == '__main__':
    listener()