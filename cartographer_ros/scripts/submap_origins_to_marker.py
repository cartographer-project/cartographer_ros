#!/usr/bin/env python
# Copyright 2017 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Publish submaps' origins as visualization marker
"""

import numpy as np
import matplotlib.colors as colors
import matplotlib.cm as cm

import rospy
import visualization_msgs.msg as viz_msgs
import cartographer_ros_msgs.msg as carto_msgs


def create_colors():
    x = np.arange(10)
    ys = [i + x + (i * x)**2 for i in range(10)]
    r = np.linspace(0, 1, len(ys))
    rainbow = cm.rainbow(r)
    return rainbow


COLORS = create_colors()
def get_color(trajectory_id):
    return COLORS[trajectory_id % len(COLORS)]


def create_marker_msg(index, pose, header, trajectory_id):
    color = get_color(trajectory_id)

    m = viz_msgs.Marker()
    m.header = header
    m.ns = str(trajectory_id)
    m.id = index
    m.type = viz_msgs.Marker.TEXT_VIEW_FACING
    m.action = 0
    m.pose = pose
    m.scale.x = 1.0
    m.scale.y = 1.0
    m.scale.z = 1.0
    m.color.r = color[0]
    m.color.g = color[1]
    m.color.b = color[2]
    m.color.a = color[3]
    m.text = str(index)
    m.lifetime = rospy.Duration(0.0)
    return m


def sort_by_trajectory(msg):
    """
    Sort msg by trajectory_id
    """
    data = {}
    for submap in msg.submap:
        if submap.trajectory_id not in data:
            data[submap.trajectory_id] = []
        data[submap.trajectory_id].append((submap.submap_index, submap.pose))
    return data


def main():
    rospy.init_node('submap_pose_to_marker')
    pub = rospy.Publisher('submap_pose', viz_msgs.MarkerArray, queue_size=4)

    def process_submaps(msg):
        markers = viz_msgs.MarkerArray()
        markers.markers = []
        data = sort_by_trajectory(msg)
        for trajectory_id, submap_list in data.items():
            for (index, pose) in submap_list:
                m = create_marker_msg(index, pose, msg.header, trajectory_id)
                markers.markers.append(m)
        pub.publish(markers)

    sub = rospy.Subscriber('submap_list', carto_msgs.SubmapList, process_submaps, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
