#!/usr/bin/env python
# coding=UTF-8
import numpy as np
import rospy
from geometry_msgs.msg import  PoseStamped
from scipy.spatial.distance import cdist
import rospy
from std_msgs.msg import String
from  test_wifi.msg import  uav_sgn_status
from scipy import interpolate
from pylab import *
import matplotlib.pyplot as plt
import time



filename = '/home/uav/twf_ws/src/test_wifi/scripts/caiji1.csv'      
M = np.loadtxt(filename,delimiter=',')
signal_st= uav_sgn_status()
pose = PoseStamped()

def callback(data):
   signal_st.Mocap_x=data.Mocap_x
   signal_st.Mocap_y=data.Mocap_y
   signal_st.Mocap_z=data.Mocap_z
   signal_st.rssi = data.rssi

def point():
      global Point,E,X,Y,signal_st1,M
      rospy.init_node('wolf_pos', anonymous=True)
      pub = rospy.Publisher('/uav1/target_point', PoseStamped, queue_size=10) 
      rospy.Subscriber("data_save1", uav_sgn_status, callback)
      while (True):
         if(signal_st.rssi!=0 ):
            break
         #####
      [pose.pose.position.x,pose.pose.position.y,pose.pose.position.z] = [round(M[0][0],2),round(M[0][1],2),1]
      E = [signal_st.rssi]
      ######
      X = [round(M[0][0],2)]
      Y = [round(M[0][1],2)]
      rate = rospy.Rate(2) 
      while not rospy.is_shutdown():
         pub.publish(pose)
         for i in range(1,M.shape[0]):
            #####
            [pose.pose.position.x,pose.pose.position.y,pose.pose.position.z] = [round(M[i][0],2),round(M[i][1],2),1]
            print(i,(pose.pose.position,signal_st.rssi))
            pub.publish(pose)
            rate.sleep()
            time.sleep(1)
            temp_E = [signal_st.rssi]
            E = E+temp_E
            #####
            temp_X =  [round(M[i][0],2)]
            X = X +temp_X
            temp_Y = [round(M[i][1],2)]
            Y  =Y +temp_Y
         if(i==M.shape[0]-1):
            break
         #####
      # np.savetxt('/home/uav/twf_ws/src/test_wifi/scripts/Singal_Pos/x2.csv',X,delimiter=',')
      # np.savetxt('/home/uav/twf_ws/src/test_wifi/scripts/Singal_Pos/y2.csv',Y,delimiter=',')
      # np.savetxt('/home/uav/twf_ws/src/test_wifi/scripts/Singal_Pos/E2.csv',E,delimiter=',')
      print(len(X),len(Y),len(E))


if __name__ == "__main__":
   point()
