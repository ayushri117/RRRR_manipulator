#!/usr/bin/env python3
import rospy
import numpy
from std_msgs.msg import Float64MultiArray as arr
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from math import sin,cos,atan,acos,sqrt


def callback(msg):
    [x,y,z,phi]=msg.data
    l1=200
    l2=500
    l3=300
    l4=45
    if (x !=0 and y!=0):
        theta1=atan(y/x)
    elif(x!=0 and y==0):
        if(x<0):
            theta1=3.14159265359
        else:
            theta1=0
    elif(x==0 and y!=0):
        if(y>0):
            theta1=1.57079632679
        elif(y<0):
            theta1=-1.57079632679
        elif(y==0):
            theta1=0
    if (theta1==1.57079632679 or theta1==-1.57079632679):
        so=abs((y/sin(theta1)))
        A=so-l4*cos(phi)
    else:
        sw=abs(x/cos(theta1))
        A=sw -l4*cos(phi)
    # A=x-l4*cos(theta1)*cos(phi)
    C=z-l1-l4*sin(phi)
    theta3=acos((A**2  +C**2 -l2**2-l3**2)/(2*l2*l3))
    b=l3*sin(theta3)
    a=l2+l3*cos(theta3)
    # if (theta1==1.57079632679 or theta1==-1.57079632679):
    #     c=(y/sin(theta1)) -l4*cos(phi)
    # else:
    #     c=(x/cos(theta1)) -l4*cos(phi)
    theta2=acos((a*A-b*sqrt(a**2 + b**2 -A**2))/(a**2 +b**2))
    theta4=phi-theta2-theta3
    if(theta3-1.570796<1e-3 and theta4<=0):
        theta3=-theta3
        theta4=0

    print(theta1,theta2,theta3,theta4,sep=" ")
    pub=rospy.Publisher("joint_states",JointState,queue_size=10)
    content=JointState()
    content.header=Header()
    content.header.stamp=rospy.Time.now()
    content.name=['Rev179','Rev192','Rev193','Rev206','Rev207','Rev208']
    content.position=[theta1,0,0,theta2,theta3,theta4]
    content.velocity=[]
    content.effort=[]
    pub.publish(content)

if __name__=="__main__":
    rospy.init_node("inverse_kinematics")
    rospy.loginfo("Inverse KInematics node has been started")
    rospy.Subscriber("/goal",arr,callback=callback)
    rospy.spin()
