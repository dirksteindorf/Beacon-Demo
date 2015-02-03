#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

# dummy values
pose_x   = [1.0, 2.0,  3.0]
pose_y   = [1.0, 2.0,  3.0]
pose_phi = [0.0, 90.0, 270.0]

index = 0

def talker():
    global index
    global pose_x
    global pose_y
    global pose_phi
    
    pub = rospy.Publisher('PoseChannel', Point, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) 
    
    while not rospy.is_shutdown():
        # built message with dummy values
        msg = Point(pose_x[index], pose_y[index], pose_phi[index])
        
        # index for dummy values
        if index < 2:
            index += 1
        
        pub.publish(msg)        
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
