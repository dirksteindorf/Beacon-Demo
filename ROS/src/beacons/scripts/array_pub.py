#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg
from geometry_msgs.msg import Point

pose_x   = [1.0, 2.0, 3.0]
pose_y   = [1.0, 2.0, 3.0]
pose_phi = [0.0, 90.0, 270.0]

index = 0

def talker():
    global index
    global pose_x
    global pose_y
    global pose_phi
    
    pub = rospy.Publisher('PoseChannel', std_msgs.msg.Float64MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) 
    
    msg = std_msgs.msg.Float64MultiArray()
    #msg.layout.data_offset = 0
    matrix_dim = std_msgs.msg.MultiArrayDimension()
    matrix_dim.label = "Pose"
    matrix_dim.size = 1
    #matrix_dim.stride = 1
    
    while not rospy.is_shutdown():
        rospy.loginfo(pose_x[index])
        rospy.loginfo(pose_y[index])
        rospy.loginfo(pose_phi[index])
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        msg.data[0] = pose_x[index]
        msg.data[1] = pose_y[index]
        msg.data[2] = pose_phi[index]
        
        pub.publish(msg)
        
        if index < 2:
            index += 1
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
