#!/usr/bin/env python3
import rospy
import numpy
from sensor_msgs.msg import JointState
from math import sin,cos

def tf_matrix(dh):
    alpha=dh[0,0]
    a=dh[0,1]
    theta=dh[0,2]
    d=dh[0,3]
    tf=numpy.matrix([[cos(theta),-sin(theta),0,a],
                     [sin(theta)*cos(alpha),cos(theta)*cos(alpha),-sin(alpha),-d*sin(alpha)],
                     [sin(theta)*sin(alpha),cos(theta)*sin(alpha),cos(alpha),d*cos(alpha)],
                     [0,0,0,1]])
    return tf
def callback(msg):
    joints=msg.position
    dh_table=numpy.matrix([[0,0,joints[0],120],
                          [1.57079633,0,joints[3],0],
                          [0,500,joints[4],0],
                          [0,300,joints[5],0],
                          [0,45,0,0]])
    transformation_matrix=numpy.identity(4)
    for val in dh_table:
        transformation_matrix=numpy.matmul(transformation_matrix,tf_matrix(val[0]))
    print(transformation_matrix)

    
if __name__=="__main__":
    rospy.init_node("forward_kinematics")
    rospy.loginfo("Forward KInematics node has been started")
    rospy.Subscriber("/joint_states",JointState,callback=callback)
    rospy.spin()