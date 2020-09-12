#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32


def talker():
    pub = rospy.Publisher("von_hacht", Int32, queue_size=10)
    rospy.init_node("nodeA")
    rate = rospy.Rate(20)

    k = 1
    n = 4
    while not rospy.is_shutdown():
        k += n
        rospy.loginfo("Publishing: {0}".format(k))
        pub.publish(k)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
