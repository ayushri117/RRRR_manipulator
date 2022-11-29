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
        a=[303.462176637767 , 525.6119081121374 , 601.8433003754266 , 0.6981317008888885]
        msg.data=a
        pub.publish(msg)
    
