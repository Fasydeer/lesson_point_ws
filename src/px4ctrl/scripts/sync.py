#!/usr/bin/env python
# coding=UTF-8
import  message_filters
import rospy
from std_msgs.msg import Float64
from  test_wifi.msg import  uav_sgn_status
from geometry_msgs.msg import  PoseStamped
gl_mark = '0'
def multi_callback1(pose_data, usrp_data):
    msg_ = uav_sgn_status()
    msg_.Mocap_x = pose_data.pose.position.x
    msg_.Mocap_y = pose_data.pose.position.y
    msg_.Mocap_z = pose_data.pose.position.z
    msg_.rssi = usrp_data.data
    pub1=rospy.Publisher('data_save1', uav_sgn_status, queue_size=1)
    pub1.publish(msg_)
def multi_callback2(pose_data, usrp_data):
    msg_ = uav_sgn_status()
    msg_.Mocap_x = pose_data.pose.position.x
    msg_.Mocap_y = pose_data.pose.position.y
    msg_.Mocap_z = pose_data.pose.position.z
    msg_.rssi = usrp_data.data
    pub2=rospy.Publisher('data_save2', uav_sgn_status, queue_size=1)
    pub2.publish(msg_)
def multi_callback3(pose_data, usrp_data):
    msg_ = uav_sgn_status()
    msg_.Mocap_x = pose_data.pose.position.x
    msg_.Mocap_y = pose_data.pose.position.y
    msg_.Mocap_z = pose_data.pose.position.z
    msg_.rssi = usrp_data.data
    print(msg_)
    pub3=rospy.Publisher('data_save3', uav_sgn_status, queue_size=1)
    pub3.publish(msg_)
def multi_callback4(pose_data, usrp_data):
    msg_ = uav_sgn_status()
    msg_.Mocap_x = pose_data.pose.position.x
    msg_.Mocap_y = pose_data.pose.position.y
    msg_.Mocap_z = pose_data.pose.position.z
    msg_.rssi = usrp_data.data
    print(msg_)
    pub4=rospy.Publisher('data_save4', uav_sgn_status, queue_size=1)
    pub4.publish(msg_)
def multi_callback5(pose_data, usrp_data):
    msg_ = uav_sgn_status()
    msg_.Mocap_x = pose_data.pose.position.x
    msg_.Mocap_y = pose_data.pose.position.y
    msg_.Mocap_z = pose_data.pose.position.z
    msg_.rssi = usrp_data.data
    pub5=rospy.Publisher('data_save5', uav_sgn_status, queue_size=1)
    pub5.publish(msg_)    
def multi_callback_pose1(pose_data):
    msg_ = uav_sgn_status()
    msg_.Mocap_x = pose_data.pose.position.x
    msg_.Mocap_y = pose_data.pose.position.y
    msg_.Mocap_z = pose_data.pose.position.z
    pub1=rospy.Publisher('data_save1', uav_sgn_status, queue_size=1)
    pub1.publish(msg_)        
def multi_callback_pose2(pose_data):
    msg_ = uav_sgn_status()
    msg_.Mocap_x = pose_data.pose.position.x
    msg_.Mocap_y = pose_data.pose.position.y
    msg_.Mocap_z = pose_data.pose.position.z
    pub2=rospy.Publisher('data_save2', uav_sgn_status, queue_size=1)
    pub2.publish(msg_)     
def multi_callback_pose3(pose_data):
    msg_ = uav_sgn_status()
    msg_.Mocap_x = pose_data.pose.position.x
    msg_.Mocap_y = pose_data.pose.position.y
    msg_.Mocap_z = pose_data.pose.position.z
    pub3=rospy.Publisher('data_save3', uav_sgn_status, queue_size=1)
    pub3.publish(msg_)     
def multi_callback_pose4(pose_data):
    msg_ = uav_sgn_status()
    msg_.Mocap_x = pose_data.pose.position.x
    msg_.Mocap_y = pose_data.pose.position.y
    msg_.Mocap_z = pose_data.pose.position.z
    pub4=rospy.Publisher('data_save4', uav_sgn_status, queue_size=1)
    pub4.publish(msg_)     
def multi_callback_pose5(pose_data):
    msg_ = uav_sgn_status()
    msg_.Mocap_x = pose_data.pose.position.x
    msg_.Mocap_y = pose_data.pose.position.y
    msg_.Mocap_z = pose_data.pose.position.z
    pub5=rospy.Publisher('data_save5', uav_sgn_status, queue_size=1)
    pub5.publish(msg_)     

if __name__ == "__main__":
    rospy.init_node('multi_receive', anonymous=True )
    #uav 1
    # rospy.Subscriber('/uav1/mavros/local_position/pose', PoseStamped, multi_callback_pose1)

    subscriber_pose1 = message_filters.Subscriber('/uav1/mavros/local_position/pose', PoseStamped, queue_size=1)
    subscriber_usrp1 = message_filters.Subscriber('/usrp_power1', Float64, queue_size=1)
    sync1 = message_filters.ApproximateTimeSynchronizer([subscriber_pose1, subscriber_usrp1], 1, 0.1, allow_headerless=True)
    sync1.registerCallback(multi_callback1)
    #uav 2
    # rospy.Subscriber('/uav2/mavros/local_position/pose', PoseStamped, multi_callback_pose2)

    subscriber_pose2 = message_filters.Subscriber('/uav2/mavros/local_position/pose', PoseStamped, queue_size=1)
    subscriber_usrp2 = message_filters.Subscriber('/usrp_power2', Float64, queue_size=1)
    sync2 = message_filters.ApproximateTimeSynchronizer([subscriber_pose2, subscriber_usrp2], 1, 0.1, allow_headerless=True)
    sync2.registerCallback(multi_callback2)
    #uav 3 
    # rospy.Subscriber('/uav3/mavros/local_position/pose', PoseStamped, multi_callback_pose3)

    subscriber_pose3 = message_filters.Subscriber('/uav3/mavros/local_position/pose', PoseStamped, queue_size=1)
    subscriber_usrp3 = message_filters.Subscriber('/usrp_power3', Float64, queue_size=1)
    sync3 = message_filters.ApproximateTimeSynchronizer([subscriber_pose3, subscriber_usrp3], 1, 0.1, allow_headerless=True)
    sync3.registerCallback(multi_callback3)
    # uav 4
    # rospy.Subscriber('/uav4/mavros/local_position/pose', PoseStamped, multi_callback_pose4)

    subscriber_pose4 = message_filters.Subscriber('/uav4/mavros/local_position/pose', PoseStamped, queue_size=1)
    subscriber_usrp4 = message_filters.Subscriber('/usrp_power4', Float64, queue_size=1)
    sync4 = message_filters.ApproximateTimeSynchronizer([subscriber_pose4, subscriber_usrp4], 1, 0.1, allow_headerless=True)
    sync4.registerCallback(multi_callback4)
    # uav 5
    # rospy.Subscriber('/uav5/mavros/local_position/pose', PoseStamped, multi_callback_pose5)

    subscriber_pose5 = message_filters.Subscriber('/uav5/mavros/local_position/pose', PoseStamped, queue_size=1)
    subscriber_usrp5 = message_filters.Subscriber('/usrp_power5', Float64, queue_size=1)
    sync5 = message_filters.ApproximateTimeSynchronizer([subscriber_pose5, subscriber_usrp5], 1, 0.1, allow_headerless=True)
    sync5.registerCallback(multi_callback5)


    rospy.spin()
