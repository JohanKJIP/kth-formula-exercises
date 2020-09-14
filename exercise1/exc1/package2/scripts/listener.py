#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16, Float32

last_data = None


def callback(data):
    global last_data
    last_data = data.data


def listen_and_publish(q=0.15):
    rospy.init_node("nodeB", anonymous=True)

    pub = rospy.Publisher("kthfs/result", Float32, queue_size=10)
    rospy.Subscriber("von_hacht", UInt16, callback)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        # Wait for data before pubishing.
        if last_data is not None:
            data = last_data / q
            pub.publish(data)
            rospy.loginfo("Publishing: {0}".format(data))
            rate.sleep()


if __name__ == "__main__":
    try:
        listen_and_publish()
    except rospy.ROSInterruptException:
        pass
