#!/usr/bin/env python

import rospy
import visualization_msgs.msg as viz_msgs
import cartographer_ros_msgs.msg as carto_msgs


global pub


def trajectory_to_marker_msg(traj, header, ns, inc, r, g ,b):

    m = viz_msgs.Marker()
    m.header = header
    m.ns = ns
    m.id = inc
    m.type = viz_msgs.Marker.TEXT_VIEW_FACING
    m.action = 0
    m.pose = traj
    m.scale.x = 1.0
    m.scale.y = 1.0
    m.scale.z = 1.0
    m.color.r = r
    m.color.g = g
    m.color.b = b
    m.color.a = 1.0
    m.text = str(inc)
    m.lifetime = rospy.Duration(0.0)
    return m


def process_submaps(msg):

    global pub

    markers = viz_msgs.MarkerArray()
    markers.markers = []
    inc = 0
    for t in msg.trajectory[0].submap:
        m1 = trajectory_to_marker_msg(t.pose, msg.header, 'pose', inc, 1.0, 0.0, 0.0)
        m2 = trajectory_to_marker_msg(t.local_pose, msg.header, 'local_pose', inc, 0.0, 1.0, 0.0)
        inc = inc + 1
        markers.markers.append(m1)
        markers.markers.append(m2)
    pub.publish(markers)


if __name__ == '__main__':

    rospy.init_node('submap_pose_publisher')
    pub = rospy.Publisher('submap_pose', viz_msgs.MarkerArray, queue_size=4)
    sub = rospy.Subscriber('submap_list', carto_msgs.SubmapList, process_submaps, queue_size=1)

    rospy.loginfo('Initialized')
    rospy.spin()
    rospy.loginfo('Bye Bye')
