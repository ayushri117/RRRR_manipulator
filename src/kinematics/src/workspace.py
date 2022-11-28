
import numpy

from math import sin,cos,atan,acos,sqrt
# import matplotlib.pyplot as plt
# from mpl_toolkits import mplot3d
# from scipy.interpolate import griddata
import plotly.graph_objects as go

if __name__=="__main__":
    # [x,y,z,phi]=msg.data
    N=10
    theta1_max=3.14159265359
    theta1_min=-3.14159265359
    theta2_min=0
    theta2_max=3.14159265359
    theta3_min=-3.14159265359
    theta3_max=3.14159265359
    theta4_min=-1.57079632679
    theta4_max=1.57079632679
    #workspace calculation
    x=[]
    y=[]
    z=[]
    si=[]
    theta1=numpy.linspace(theta1_min,theta1_max,N)
    theta2=numpy.linspace(theta2_min,theta2_max,N)
    theta3=numpy.linspace(theta3_min,theta3_max,N)
    theta4=numpy.linspace(theta4_min,theta4_max,N)
    f=open('workspace.txt','w')
    for i in theta1:
        for j in theta2:
            for k in theta3:
                for l in theta4:
                    # print(cos(i),j,k,l,sep=" ")
                    if ( 300*sin(j+k)+45*sin(j+k+l)+500*sin(j)+120>0):
                        x.append(5*cos(i)*(60*cos(j+k)+100*cos(j)+9*cos(j+k+l)))
                        y.append(5*sin(i)*(60*cos(j+k)+100*cos(j)+9*cos(j+k+l)))
                        z.append(300*sin(j+k)+45*sin(j+k+l)+500*sin(j)+120)
                        si.append(j+k+l)
                        f.write(f"{5*cos(i)*(60*cos(j+k)+100*cos(j)+9*cos(j+k+l))} , {5*sin(i)*(60*cos(j+k)+100*cos(j)+9*cos(j+k+l))} , {300*sin(j+k)+45*sin(j+k+l)+500*sin(j)+120} , {j+k+l} \n")
    f.close()
    x=numpy.array(x)
    y=numpy.array(y)
    z=numpy.array(z)
    si=numpy.array(si)
    fig = go.Figure(data=[go.Mesh3d(x=x, y=y, z=z, color='lightpink', opacity=0.90)])
    fig.show()
    

    # xlin = numpy.linspace(min(x), max(x), 100)
    # ylin = numpy.linspace(min(y), max(y), 100)
    # [X,Y] = numpy.meshgrid(xlin, ylin)
    # Z = griddata(x,y,z,X,Y,'v4')
    # fig = plt.figure()
    # ax = plt.axes(projection='3d')
    # ax.plot_surface(x, y, z)

    
# import plotly.graph_objects as go
# import numpy as np

# # Download data set from plotly repo
# pts = np.loadtxt(np.DataSource().open('https://raw.githubusercontent.com/plotly/datasets/master/mesh_dataset.txt'))
# x, y, z = pts.T
# print(x)
# fig = go.Figure(data=[go.Mesh3d(x=x, y=y, z=z, color='lightpink', opacity=0.50)])
# fig.show()