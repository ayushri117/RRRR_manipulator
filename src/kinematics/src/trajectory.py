#!/usr/bin/env python3
import rospy
import numpy
import time
from std_msgs.msg import Float64MultiArray as arr
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from math import sin,cos,atan,acos,sqrt
from sympy import symbols

once=False
coord=[]

def callback(msg):
    global once
    global coord
    tf=5
    N=10
    [xf,yf,zf,phi_f]=msg.data
    if(not once):
        coord = rospy.wait_for_message('/current', arr, timeout=5)
        once=True
    [xi,yi,zi,phi_i]=coord.data
    A=numpy.matrix([[1,0,0,0,0,0],
                    [0,1,0,0,0,0],
                    [0,0,2,0,0,0],
                    [1,tf,tf**2,tf**3,tf**4,tf**5],
                    [0,1,2*tf,3*tf**2,4*tf**3,5*tf**4],
                    [0,0,2,6*tf,12*tf**2,20*tf**3]])
    bx=numpy.matrix([[xi],
                    [0],
                    [0],
                    [xf],
                    [0],
                    [0]])
    by=numpy.matrix([[yi],
                    [0],
                    [0],
                    [yf],
                    [0],
                    [0]])
    bz=numpy.matrix([[zi],
                    [0],
                    [0],
                    [zf],
                    [0],
                    [0]])
    bphi=numpy.matrix([[phi_i],
                    [0],
                    [0],
                    [phi_f],
                    [0],
                    [0]])
    A=numpy.linalg.inv(A)
    
    coeff_x=A*bx
    coeff_y=A*by
    coeff_z=A*bz
    coeff_phi=A*bphi
    theta1=[]
    theta2=[]
    theta3=[]
    theta4=[]
    t=numpy.linspace(0,tf,N)
    
    for i in t:
        poly=numpy.matrix([[1,i,i**2,i**3,i**4,i**5]])
        cx=poly*coeff_x
        cy=poly*coeff_y
        cz=poly*coeff_z
        cphi=poly*coeff_phi
        l1=200
        l2=500
        l3=300
        l4=45
        x=cx[0,0]
        y=cy[0,0]
        z=cz[0,0]
        phi=cphi[0,0]
        if (x !=0 and y!=0):
            calc_theta1=atan(y/x)
        elif(x!=0 and y==0):
            if(x<0):
                calc_theta1=3.14159265359
            else:
                calc_theta1=0
        elif(x==0):
            if(y>0):
                calc_theta1=1.57079632679
            elif(y<0):
                calc_theta1=-1.57079632679
            elif(y==0):
                calc_theta1=0
        if (calc_theta1==1.57079632679 or calc_theta1==-1.57079632679):
            so=abs((y/sin(calc_theta1)))
            A=so-l4*cos(phi)
        else:
            sw=abs(x/cos(calc_theta1))
            A=sw -l4*cos(phi)
        # A=x-l4*cos(theta1)*cos(phi)
        C=z-l1-l4*sin(phi)
        calc_theta3=acos((A**2  +C**2 -l2**2-l3**2)/(2*l2*l3))
        b=l3*sin(calc_theta3)
        a=l2+l3*cos(calc_theta3)
        # if (theta1==1.57079632679 or theta1==-1.57079632679):
        #     c=(y/sin(theta1)) -l4*cos(phi)
        # else:
        #     c=(x/cos(theta1)) -l4*cos(phi)
        calc_theta2=acos((a*A-b*sqrt(a**2 + b**2 -A**2))/(a**2 +b**2))
        calc_theta4=phi-calc_theta2-calc_theta3
        if(calc_theta3-1.570796<1e-3 and calc_theta4<=0):
            calc_theta3=-calc_theta3
            calc_theta4=0
        theta1.append(calc_theta1)
        theta2.append(calc_theta2)
        theta3.append(calc_theta3)
        theta4.append(calc_theta4)
    theta=[theta1,theta2,theta3,theta4]
    pub=rospy.Publisher("joint_states",JointState,queue_size=10)
    content=JointState()
    content.header=Header()
    content.header.stamp=rospy.Time.now()
    content.name=['Rev179','Rev192','Rev193','Rev206','Rev207','Rev208']
    
    content.velocity=[]
    content.effort=[]
    
    count=1
    lt1=0.0
    lt2=0.0
    lt3=0.0
    lt4=0.0
    rate=rospy.Rate(100)
    for i in theta:
        for j in i:
            if (count==1):
                content.position=[j,0,0,0,0,0]
                lt1=j
            elif(count==2):
                content.position=[lt1,0,0,j,0,0]
                lt2=j
            elif(count==3):
                content.position=[lt1,0,0,lt2,j,0]
                lt3=j
            elif(count==4):
                content.position=[lt1,0,0,lt2,lt3,j]
                lt4=j
            pub.publish(content)
            
        count+=1


    
    


if __name__=="__main__":
    rospy.init_node("trajectory_planner")
    rospy.loginfo("Trajectory planner node has been started")
    rospy.Subscriber("/goal",arr,callback=callback)
    rospy.spin()