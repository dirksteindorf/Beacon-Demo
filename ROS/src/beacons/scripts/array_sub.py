#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard: %d, %d, %d", data.x, data.y, data.z)
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("PoseChannel", Point, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
