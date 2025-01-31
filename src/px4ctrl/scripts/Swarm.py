#!/usr/bin/env python
# coding=UTF-8
import numpy as np
import rospy
from geometry_msgs.msg import  PoseStamped
from scipy.spatial.distance import cdist
import rospy
from std_msgs.msg import String
from quadrotos_msgs.msg import sgn_stamp
from scipy import interpolate
from pylab import *
import matplotlib.pyplot as plt
import time

filename = '/home/uav/twf_ws/src/test_wifi/scripts/hangji/HJ.csv'      
M = np.loadtxt(filename,delimiter=',')
signal_st1= sgn_stamp()
signal_st3=sgn_stamp()
signal_st2=sgn_stamp()

pose1 = PoseStamped()
pose3 = PoseStamped()
pose2= PoseStamped()

def callback1(data):
   signal_st1.Mocap_x=data.Mocap_x
   signal_st1.Mocap_y=data.Mocap_y
   signal_st1.Mocap_z=data.Mocap_z
   signal_st1.rssi = data.rssi
def callback3(data):
   signal_st3.Mocap_x=data.Mocap_x
   signal_st3.Mocap_y=data.Mocap_y
   signal_st3.Mocap_z=data.Mocap_z
   signal_st3.rssi=data.rssi
def callback2(data):
   signal_st2.Mocap_x=data.Mocap_x
   signal_st2.Mocap_y=data.Mocap_y
   signal_st2.Mocap_z=data.Mocap_z
   signal_st2.rssi=data.rssi
def point():
    global Point,E,X,Y,signal_st1,signal_st3,signal_st2
    rospy.init_node('wolf_pos', anonymous=True)
    pub1 = rospy.Publisher('/uav1/target_point', PoseStamped, queue_size=10) 
    pub3 = rospy.Publisher('/uav3/target_point', PoseStamped, queue_size=10) 
    pub2 = rospy.Publisher('/uav2/target_point', PoseStamped, queue_size=10) 

    rospy.Subscriber("data_save1", uav_sgn_status, callback1)
    rospy.Subscriber("data_save3", uav_sgn_status, callback3)
    rospy.Subscriber("data_save2", uav_sgn_status, callback2)
    while (True):
      if(signal_st1.rssi!=0 and signal_st2.rssi!=0 and signal_st3.rssi!=0 ):
            break
    #   if(signal_st2.rssi!=0):
    #       break
    [pose1.pose.position.x,pose1.pose.position.y,pose1.pose.position.z] = [round(M[0][0],2),round(M[0][1],2),1]
    [pose3.pose.position.x,pose3.pose.position.y,pose3.pose.position.z] = [round(M[0][2],2),round(M[0][3],2),1]
    [pose2.pose.position.x,pose2.pose.position.y,pose2.pose.position.z] = [round(M[0][4],2),round(M[0][5],2),1]
    E = [signal_st1.rssi,signal_st3.rssi,signal_st2.rssi]
    X = [round(M[0][0],2),round(M[0][2],2),round(M[0][4],2)]
    Y = [round(M[0][1],2),round(M[0][3],2),round(M[0][5],2)]
    while not rospy.is_shutdown():
        pub1.publish(pose1)
        pub3.publish(pose3)
        pub2.publish(pose2)
        for i in range(1,M.shape[0]):
            [pose1.pose.position.x,pose1.pose.position.y,pose1.pose.position.z] = [round(M[i][0],2),round(M[i][1],2),1]
            [pose3.pose.position.x,pose3.pose.position.y,pose3.pose.position.z] = [round(M[i][2],2),round(M[i][3],2),1]
            [pose2.pose.position.x,pose2.pose.position.y,pose2.pose.position.z] = [round(M[i][4],2),round(M[i][5],2),1]
            print(i,(pose3.pose.position,signal_st3.rssi),(pose1.pose.position,signal_st1.rssi),\
                (pose2.pose.position,signal_st2.rssi))
            pub1.publish(pose1)
            pub3.publish(pose3)
            pub2.publish(pose2)
            rate.sleep()
            time.sleep(1.4)
            temp_E = [signal_st1.rssi,signal_st3.rssi,signal_st2.rssi]
            E = E+temp_E
            temp_X =  [round(M[i][0],2),round(M[i][2],2),round(M[i][4],2)]
            X = X +temp_X
            temp_Y = [round(M[i][1],2),round(M[i][3],2),round(M[i][5],2)]
            Y  =Y +temp_Y
        if(i==M.shape[0]-1):
            break
    # np.savetxt('/home/uav/twf_ws/src/test_wifi/scripts/Swarm_Pos/x.csv',X,delimiter=',')
    # np.savetxt('/home/uav/twf_ws/src/test_wifi/scripts/Swarm_Pos/y.csv',Y,delimiter=',')
    # np.savetxt('/home/uav/twf_ws/src/test_wifi/scripts/Swarm_Pos/E.csv',E,delimiter=',')
    print(len(X),len(Y),len(E))

if __name__ == "__main__":
   point()