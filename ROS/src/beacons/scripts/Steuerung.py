#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
import numpy as np
import time


control = 1

# Datei einlesen
fobj = open("/home/andre/Uni Magdeburg/Masterarbeit/MIRA/Konfiguration.txt", "r")

temp = np.array(fobj.readline().split(","), dtype=float)
fingerprints = np.array([temp])

for line in fobj:
    temp = np.array([line.split(",")], dtype=float)
    fingerprints = np.append(fingerprints, temp, 0)

fobj.close()


def callback(data):
    rospy.loginfo("%s", data.data)

    global control

    # rospy.init_node('Steuerung')
    pub = rospy.Publisher('/PoseChannel', Point)

    time.sleep(10)

    info = "%f /t %f /t %f -> timestamp: %s" % (fingerprints[control, 0], fingerprints[control, 1], fingerprints[control, 2],
                                                rospy.get_time())
    rospy.loginfo(info)
    msg = Point(fingerprints[control, 0], fingerprints[control, 1], fingerprints[control, 2])
    pub.publish(msg)

    if control < fingerprints.shape[0]:
        control += 1


def talker():
    global control

    rospy.init_node('Steuerung')
    r = rospy.Rate(10)
    pub = rospy.Publisher('/PoseChannel', Point)

    while control < 10:
        info = "%f   %f   %f -> timestamp: %s" % (fingerprints[control, 0], fingerprints[control, 1],
                                                  fingerprints[control, 2], rospy.get_time())
        rospy.loginfo(info)
        msg = Point(fingerprints[control, 0], fingerprints[control, 1], fingerprints[control, 2])
        pub.publish(msg)
        control += 1
        r.sleep()


def listener():
    # rospy.init_node('Steuerung', anonymous=True)
    rospy.Subscriber("PilotEvent", String, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        talker()
        listener()
    except rospy.ROSInterruptException:
        pass 
