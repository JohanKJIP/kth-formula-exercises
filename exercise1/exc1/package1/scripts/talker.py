#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16


def talker():
    pub = rospy.Publisher("von_hacht", UInt16, queue_size=10)
    rospy.init_node("nodeA")
    rate = rospy.Rate(20)

    k = 1
    n = 4
    while not rospy.is_shutdown():
        k += n
        # UInt16 max value
        if k > 65535:
            k = 1
        rospy.loginfo("Publishing: {0}".format(k))
        pub.publish(k)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
