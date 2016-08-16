#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu


class PublishImu(object):  # pylint: disable=too-few-public-methods
    """
    Class that transforms a message and republishes it with a new timestamp
    """
    def __init__(self, topic):
        self._pub = rospy.Publisher(topic,
                                    Imu,
                                    queue_size=1)
        self.ctrl_rate = rospy.Rate(10)

    def start(self):
        """
        publishing data
        """
        msg = Imu()
        msg.header.frame_id = "base_link"
        while not rospy.is_shutdown():
            msg.header.stamp = rospy.Time.now()
            self._pub.publish(msg)
            self.ctrl_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('imu_publisher')
    rospy.loginfo("Initialized")
    imupub = PublishImu("/imu")
    imupub.start()
    rospy.loginfo("Bye Bye")
