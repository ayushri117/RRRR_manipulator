#!/usr/bin/env python3
import rospy
import numpy
from std_msgs.msg import Float64MultiArray as arr
from math import sin,cos



if __name__=="__main__":
    rospy.init_node("pub")
    rospy.loginfo("Igoal pub node has been started")
    pub=rospy.Publisher("/goal",arr,queue_size=10)
    while not rospy.is_shutdown():

        msg=arr()
        a=[0,345,620,0 ]
        msg.data=a
        pub.publish(msg)
    
    