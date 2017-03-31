#!/usr/bin/env python
import rospy
import cv2
import numpy  as np
from cartographer_ros_msgs.srv import SubmapQuery
from cartographer_ros_msgs.msg import SubmapList
import gzip
import StringIO
from time import sleep


def submap_list_callback(submap_list):
    n_submaps = len(submap_list.trajectory[0].submap) - 1
    cv2.setTrackbarMax('submap_id', 'img', n_submaps)


def get_img(submap):
    fileobject = StringIO.StringIO(submap.cells)
    f = gzip.GzipFile(fileobj=fileobject)
    string = f.read()
    img = np.fromstring(string, np.uint8)
    img = img.reshape((submap.height, submap.width, -1))
    img = np.concatenate((img[:, :, 1:], img[:, :, 1:], img[:, :, :1]), axis=2)
    return img


def fetch_and_render(map_id):
    submap = submap_query(0, map_id)
    img = get_img(submap)
    cv2.imshow('img',img)


def callback(mapid):
    fetch_and_render(mapid)


if __name__ == "__main__":
    cv2.namedWindow('img', flags=cv2.WINDOW_NORMAL)
    trackbar = cv2.createTrackbar('submap_id', 'img', 0, 1, callback)

    rospy.init_node("submap_visualization")

    submap_query = rospy.ServiceProxy('submap_query', SubmapQuery)
    rospy.loginfo("Wait for Submap Query")
    rospy.wait_for_service('/submap_query')
    rospy.loginfo("Submap Query Ready")
    rospy.loginfo("Initialized")
    rospy.Subscriber("/submap_list",SubmapList, submap_list_callback)

    while not rospy.is_shutdown():
        k = cv2.waitKey(1)
        if k==27:    # Esc key to stop
            break
