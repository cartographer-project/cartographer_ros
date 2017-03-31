#!/usr/bin/env python
import rospy
import cv2
import numpy  as np
from cartographer_ros_msgs.srv import SubmapQuery
from cartographer_ros_msgs.msg import SubmapList
import gzip
import StringIO
from time import sleep

global last_map_id, n_submaps

def submap_list_callback(submap_list):
    n_submaps = submap_list.trajectory[0].submap.__len__()-1
    cv2.setTrackbarMax('submap_id', 'img', n_submaps)
    #cv2.updateWindow('img')


if __name__ == "__main__":
    rospy.init_node("submap_visualization")
    rospy.Subscriber("/submap_list",SubmapList, submap_list_callback)

    submap_query = rospy.ServiceProxy('submap_query', SubmapQuery)

    last_map_id = 0

    def callback(mapid):
        global last_map_id
        last_map_id = mapid


    def fetch_and_render():
        submap = submap_query(0, last_map_id)
        fileobject = StringIO.StringIO(submap.cells)
        f = gzip.GzipFile(fileobj=fileobject)
        string = f.read()
        img = np.fromstring(string, np.uint8)

        print img.shape, submap.width, submap.height, submap.resolution
        img = img.reshape((submap.height, submap.width, -1))

        img = np.concatenate((img[:, :, :1], img[:, :, 1:], img[:, :, 1:]), axis=2)
        cv2.setWindowProperty("img", cv2.WINDOW_AUTOSIZE, False)
        cv2.imshow("img", img)
        #cv2.resizeWindow("img", 500, 500)


    cv2.namedWindow('img')
    trackbar = cv2.createTrackbar('submap_id', 'img', 0, 50, callback)

    rospy.wait_for_service('/submap_query')

    print type(trackbar)
    while not cv2.waitKey(1) & 0xFF == 27:
        try:
            fetch_and_render()
        except Exception,e:
            print e
        sleep(0.05)
