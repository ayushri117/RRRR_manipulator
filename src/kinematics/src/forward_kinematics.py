#!/usr/bin/env python3
import rospy
import numpy
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray as arr
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
    # dh_table=numpy.matrix([[0,0,joints[0],120],
    #                       [1.57079632679,0,joints[3],0],
    #                       [0,500,joints[4],0],
    #                       [0,300,joints[5],0],
    #                       [0,45,0,0]])
    i=joints[0]
    j=joints[3]
    k=joints[4]
    l=joints[5]
    # transformation_matrix=numpy.identity(4)
    # for val in dh_table:
    #     transformation_matrix=numpy.matmul(transformation_matrix,tf_matrix(val[0]))
    # # for a in range(0,3):
    # #     for i in range(0,3):
    # #         transformation_matrix[a,i]='{:.2f}'.format(transformation_matrix[a,i])
    # float_formatter = "{:.2f}".format
    # numpy.set_printoptions(formatter={'float_kind':float_formatter})
    # print(transformation_matrix)
    x=5*cos(i)*(60*cos(j+k)+100*cos(j)+9*cos(j+k+l))
    y=5*sin(i)*(60*cos(j+k)+100*cos(j)+9*cos(j+k+l))
    z=300*sin(j+k)+45*sin(j+k+l)+500*sin(j)+120
    phi=j+k+l
    x=round(x,2)
    y=round(y,2)
    z=round(z,2)
    pub=rospy.Publisher("current",arr,queue_size=10)
    ar=arr()
    a=[x,y,z,phi]
    ar.data=a
    pub.publish(ar)


    
if __name__=="__main__":
    start=time.time()
    rospy.init_node("forward_kinematics")
    rospy.loginfo("Forward KInematics node has been started")
    end=time.time()
    pu1=rospy.Publisher('current',arr,queue_size=10)
    ar1=arr()
    f=[845,0,120,0]
    while(True):
        if(end-start>5):
            break
        print(end-start)
        end=time.time()
        ar1.data=f
        pu1.publish(ar1)
    print("hmm")
    rospy.Subscriber("/joint_states",JointState,callback=callback)
    rospy.spin()
