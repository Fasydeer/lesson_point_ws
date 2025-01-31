from ast import main
from matplotlib.hatch import SouthEastHatch
import numpy as np
from Func import Function
import math
import random
import rospy
from geometry_msgs.msg import  PoseStamped
from scipy.spatial.distance import cdist
import copy
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import String
from  test_wifi.msg import  uav_sgn_status
import scipy

random.seed(24)
Func =Function(2)

filename = '/home/uav/twf_ws/src/test_wifi/scripts/data5.csv'          
wolfnum = 5           
maxgen = 100          
Tmax = 50             
E_deta = 4
ll_brave = 5
d_min = 10
signal_st = uav_sgn_status()

signal_st1= signal_st
signal_st2=signal_st
signal_st3=signal_st
signal_st4=signal_st
signal_st5=signal_st



poses1 = PoseStamped()
poses2= PoseStamped()
poses3 = PoseStamped()
poses4= PoseStamped()
poses5= PoseStamped()
pose1 = poses1
pose2 = poses2
pose3 = poses3
pose4 = poses4
pose5 = poses5


M = np.loadtxt(filename,delimiter=',')
M = M.T
M = M[21:81,21:81]

# plt.xlim([0,81])
# plt.ylim([0,81])
# plt.imshow(M,cmap='jet')
AP = 10*np.ones_like(M) 
K = np.ones_like(M)        
P = np.ones_like(M)        
Ea = 0.5                   
da = 10                    
Ga = 0.5
count = 0
wolf_position = []
''''''

x = np.arange(10, 47, 9)
y = np.arange(40, 57, 4)
# x = np.arange(5, 26, 5)
# y = np.arange(52, 65, 3)
#cha zhi

X = np.array([x,y])
Z = np.ones((1,wolfnum))




# leadhandel = plt.scatter(X[0][0], X[1][0], marker='D')
# explorehandel = plt.scatter(X[0,1:7], X[1,1:7], marker='H')
# bravehandel = plt.scatter(X[0,7:10], X[1,7:10], marker='o')
# plt.show()
#
# plt.ion()
X_t = copy.deepcopy(X)
ALL_POS = copy.deepcopy(X)
Last_pos = copy.deepcopy(X)
#X_t = X[:]
E = M[x,y]
maxE = Func.get_arrmax(E)
idx = np.argwhere(E == maxE)
lead_index = X[:,idx[0][0]]
lead_label= idx[0][0]
TrueE= np.zeros_like(M)
#brave_muil = cdist([[39,59]],[[31,51]])[0][0]
brave_muil =  60
brave_muil1 = 5

position = []
#lead_postion = lead_index
Get_M = []
tmp = 0
def callback1(data):
   signal_st1.Mocap_x=data.Mocap_x
   signal_st1.Mocap_y=data.Mocap_y
   signal_st1.Mocap_z=data.Mocap_z
   #print(1111111,data.rssi,"\n")
   signal_st1.rssi = data.rssi
#    print(data.Mocap_x)
#    print(data.Mocap_y)
#    print(data.Mocap_z)
#    print(signal_st1.rssi)
 # print(signal_st1)
#    signal_st1.rssi[0]=data.rssi[0]
#    signal_st1.rssi[1]=data.rssi[1]
#    signal_st1.rssi[2]=data.rssi[2]


def callback2(data):
   signal_st2.Mocap_x=data.Mocap_x
   signal_st2.Mocap_y=data.Mocap_y
   signal_st2.Mocap_z=data.Mocap_z
#    print(2222222,data.rssi,"\n")
   signal_st2.rssi = data.rssi

def callback3(data):
   signal_st3.Mocap_x=data.Mocap_x
   signal_st3.Mocap_y=data.Mocap_y
   signal_st3.Mocap_z=data.Mocap_z
   signal_st3.rssi=data.rssi


def callback4(data):
   signal_st4.Mocap_x=data.Mocap_x
   signal_st4.Mocap_y=data.Mocap_y
   signal_st4.Mocap_z=data.Mocap_z
#    print(4444444,data.rssi,"\n")
   signal_st4.rssi=data.rssi

def callback5(data):
   signal_st5.Mocap_x=data.Mocap_x
   signal_st5.Mocap_y=data.Mocap_y
   signal_st5.Mocap_z=data.Mocap_z
#    print(5555555,data.rssi,"\n")
   signal_st5.rssi=data.rssi
 
    # rospy.init_node('listener', anonymous=True)



def wolf():
    global lead_label ,lead_index,P,X_t,Last_pos,signal_st1,signal_st2,signal_st3,signal_st4,signal_st5,Tem,PPPPos
    global AP,M,Z,E_n,signal_st1,TrueE
    rospy.init_node('wolf_pos', anonymous=True)
    #FLAG
    # print(signal_st1)
    pub1 = rospy.Publisher('/uav1/target_point', PoseStamped, queue_size=10) 
    # pub2 = rospy.Publisher('/uav2/target_point', PoseStamped, queue_size=10) 
    # pub3 = rospy.Publisher('/uav3/target_point', PoseStamped, queue_size=10) 
    # pub4 = rospy.Publisher('/uav4/target_point', PoseStamped, queue_size=10) 
    # pub5 = rospy.Publisher('/uav5/target_point', PoseStamped, queue_size=10) 

    #FLAG
    rospy.Subscriber("data_save1", uav_sgn_status, callback1)
    # rospy.Subscriber("data_save2", uav_sgn_status, callback2)
    # rospy.Subscriber("data_save3", uav_sgn_status, callback3)
    # rospy.Subscriber("data_save4", uav_sgn_status, callback4)
    # rospy.Subscriber("data_save5", uav_sgn_status, callback5)
      
    while (True):
        if (signal_st1.rssi!=0):
            break
    # print(signal_st1)
    while (True):
        if (signal_st2.rssi!=0):
            break   
    # print(signal_st3)
    while (True):
        if (signal_st3.rssi!=0):
            break 
    # print(signal_st5)
    
    while (True):
        if (signal_st4.rssi!=0):
            break 
    # print(signal_st4)
    while (True):
        if (signal_st5.rssi!=0):
            break 
    # print(signal_st5)
   
   
    Fei  =copy.deepcopy(X)
    Fei[0,:] = (30-X[0,:])/10
    Fei[1,:] = (X[1,:]-30)/10
    # PPPPos = Fei
    Fei = np.row_stack((Fei,Z))
    # pose1.pose.position = copy.deepcopy(Fei[:,0])
    # pose5.pose.position = copy.deepcopy(Fei[:,4])
    #FLAG
    [pose1.pose.position.x,pose1.pose.position.y,pose1.pose.position.z] = [ Fei[:,0][0], Fei[:,0][1], Fei[:,0][2]]
    # print(111111,pose1.pose.position)
    # [pose2.pose.position.x,pose2.pose.position.y,pose2.pose.position.z] = [ Fei[:,1][0], Fei[:,1][1], Fei[:,1][2]]
    # [pose3.pose.position.x,pose3.pose.position.y,pose3.pose.position.z] = [ Fei[:,2][0], Fei[:,2][1], Fei[:,2][2]]
    # [pose4.pose.position.x,pose4.pose.position.y,pose4.pose.position.z] = [ Fei[:,3][0], Fei[:,3][1], Fei[:,3][2]]
    # [pose5.pose.position.x,pose5.pose.position.y,pose5.pose.position.z] = [ Fei[:,4][0], Fei[:,4][1], Fei[:,4][2]]
    # print(111111111,pose1.pose.position)
    rate = rospy.Rate(1) 
    
    while not rospy.is_shutdown():
        #FLAG
        pub1.publish(pose1)
        # print(pose1)
        # rospy.loginfo("Message_uav1: %s",pose1)
        # pub2.publish(pose2)
        # pub3.publish(pose3)
        # # print(pose3)
        # pub4.publish(pose4)
        # pub5.publish(pose5)
        # rospy.spin()
        # rate.sleep()
        
        for iter in range(maxgen):
            print(iter)
            if(iter==0):
                position = X
                #E_n = np.array([-(float(signal_st1.rssi[1])*10+float(signal_st1.rssi[2]))])
                Last_position =X
                # print(signal_st1)
                #E_n = np.array([signal_st1.rssi,signal_st2.rssi,signal_st3.rssi,signal_st4.rssi,signal_st5.rssi])
                E_n = [signal_st1.rssi,0,0,0,0]
                #E_n = signal_st1.rssi
                #print(E_n)

                # print(E_n)
            else:
                #Temp_E = np.array([signal_st1.rssi,signal_st2.rssi,signal_st3.rssi,signal_st4.rssi,signal_st5.rssi])
                Temp_E = [signal_st1.rssi,0,0,0,0]
                Last_position = np.column_stack((Last_position,X_t))
                print(type(E_n))
                E_n  = E_n+Temp_E
                #E_n = signal_st1.rssi
                print(E_n)
            
            #Last_Fei = [Last_Fei]
            Snum = 3         
            random_num = random.sample(range(0,wolfnum),Snum)
            # for i in range(wolfnum):
            #    X[0][i],X[1][i] = Func.judge(X[0][i],X[1][i])
            explore_position = X[:,random_num]
            for i in range(Tmax):
                while(lead_label in random_num):
                    random_num = random.sample(range(0, wolfnum), Snum)
                explore_position = X[:, random_num]
                for j in range(Snum):
                    explore_position[0,j],explore_position[1,j] = Func.judge(explore_position[0,j], explore_position[1,j])
                    explore_pos =np.around(Func.updatepos(explore_position[:,j],AP,count))
                    explore_pos = Func.judge(explore_pos[0],explore_pos[1])
                    X[:,random_num[j]] = explore_pos
                    try:
                        K[explore_pos[0]][explore_pos[1]] = 0
                    except:
                        print(explore_pos)
                    if(M[lead_index[0]][lead_index[1]] < M[explore_pos[0]][explore_pos[1]] ):
                        # lead_index = copy.deepcopy(explore_pos)
                        #np.c_[lead_postion,lead_index]
                        (random_num[j],lead_label) = (lead_label,random_num[j])
                        # tmp = lead_label
                        # lead_label = random_num[j]
                        # random_num[j] = tmp
                position = np.c_[position,X]
                AP = Func.updateSa(AP,K,Ea,Ga,da,P)
            lead_index = copy.deepcopy(X[:, lead_label])
            uniq_position = np.unique(position, axis=1)
            P = Func.updatePrem(position,M)
            brave_label = []
            for e in range(X.shape[1]):
                if(e!=lead_label and (e not in random_num)):
                    brave_label.append(e)
            if(iter>0):
                while(lead_label in random_num):
                    brave_label = []
                    for e in range(X.shape[1]):
                        if (e != lead_label and (e not in random_num)):
                            brave_label.append(e)
            for g in range(len(brave_label)):
                l = (M[lead_index[0]][lead_index[1]]-M[X[0][brave_label[g]]][X[1][brave_label[g]]])*\
                    4*brave_muil/(ll_brave*E_deta)
                l = np.around(l)

                brave_angel = Func.get_angel(lead_index,X[:,brave_label[g]])
                brave_pos =np.around(X[:,brave_label[g]] + l*brave_angel)
                brave_pos = brave_pos.astype(int)
                dis = cdist([lead_index], [brave_pos])[0][0]
                if (dis <= d_min and dis >= 1):
                    continue
                brave_pos = Func.judge(brave_pos[0], brave_pos[1])
                X[:,brave_label[g]] = brave_pos
                if(M[lead_index[0]][lead_index[1]] < M[brave_pos[0]][brave_pos[1]]):
                    # lead_index  = copy.deepcopy(brave_pos)
                    #np.c_[lead_postion, lead_index]
                    # tmp = lead_label
                    # lead_label = brave_label[g]
                    # brave_label[g] = tmp
                    (lead_label,brave_label[g]) = (brave_label[g],lead_label)
            lead_index = copy.deepcopy(X[:,lead_label])
            #(lead_index,X[:,tmp]) = (X[:,tmp],lead_index)
            position = np.c_[position, X]

            for h in range(X.shape[1]):
                if(h==lead_label):
                    continue
                l1 = (M[lead_index[0]][lead_index[1]] - M[X[0][h]][X[1][h]]) * \
                    2 * brave_muil1 / (ll_brave * E_deta)
                l1 = np.around(l1)

                all_angel = Func.get_angel(lead_index, X[:, brave_label[g]])
                all_pos = np.around(X[:, brave_label[g]] + l * brave_angel)
                all_pos = all_pos.astype(int)
                all_pos = Func.judge(all_pos[0], all_pos[1])
                all_dis = cdist([lead_index],[all_pos])[0][0]
                if(all_dis<=d_min and all_dis>=1):
                    continue
                X[:,h] = all_pos
            X1 = np.zeros((2,len(random_num)))
            X1_t = copy.deepcopy(X1)
            X2 = np.zeros((2, len(brave_label)))
            X2_t = copy.deepcopy(X2)
            #X2_t = X2[:]
            X3 = np.zeros((2, 1))
            #X3_t = X3[:]
            X3_t = copy.deepcopy(X3)
            count1=0
            count2=0
            for i in range(len(X[1])):
                if i==lead_label:
                    X3 = copy.deepcopy(X[:, i])
                    X3_t = copy.deepcopy(X_t[:, i])
                elif i in random_num:
                    X1[:,count1]= copy.deepcopy(X[:, i])
                    X1_t[:,count1]=copy.deepcopy(X_t[:,i])
                    count1+=1
                else:
                    X2[:, count2] = copy.deepcopy(X[:, i])
                    X2_t[:, count2] = copy.deepcopy(X_t[:, i])
                    count2 += 1
            slot_num=10
            lead_out,explore_out,M_out = Func.generate_pos(X3,X3_t,X1,X1_t,X2,X2_t,slot_num)
            for slot in range(slot_num):
                temp_X = np.zeros((2, wolfnum))
                temp_X[:,lead_label] = lead_out[:,slot]
                for k in range(len(random_num)):
                    temp_X[:,random_num[k]] = explore_out[:,slot*Snum+k]
                for g in range(wolfnum-Snum-1):
                    temp_X[:,brave_label[g]] = M_out[:,slot*(wolfnum-Snum-1)+g]
                Last_pos = np.c_[Last_pos,temp_X]
                temp_X[0,:] = (30-temp_X[0,:])/10
                temp_X[1,:] = (temp_X[1,:]-30)/10
                Last_position = np.column_stack((Last_position,temp_X))
                temp_X = np.row_stack((temp_X,Z))

                Temp_E =[signal_st1.rssi,0,0,0,0]
                E_n = E_n + Temp_E
                #Temp_E = np.array([signal_st1.rssi,signal_st2.rssi,signal_st3.rssi,signal_st4.rssi,signal_st5.rssi])
               # Last_position = np.column_stack((Last_position,temp_X))
                # print(Last_position.shape)
                # E_n = np.column_stack((E_n,Temp_E))
                # print(E_n.shape)
                # PPPPos = np.column_stack((PPPPos,temp_X))

                #Tem = temp_X
                #pose1.pose.position = [0,0,0]
                #pose5.pose.position = copy.deepcopy(Tem[:,4])
                #FLAG
                [pose1.pose.position.x,pose1.pose.position.y,pose1.pose.position.z] = [ temp_X[:,0][0], temp_X[:,0][1], temp_X[:,0][2]]
                #print(pose1.pose.position)
                # [pose2.pose.position.x,pose2.pose.position.y,pose2.pose.position.z] = [ temp_X[:,1][0], temp_X[:,1][1], temp_X[:,1][2]]
                # #print(pose2.pose.position)
                # [pose3.pose.position.x,pose3.pose.position.y,pose3.pose.position.z] = [ temp_X[:,2][0], temp_X[:,2][1], temp_X[:,2][2]]
                # [pose4.pose.position.x,pose4.pose.position.y,pose4.pose.position.z] = [ temp_X[:,3][0], temp_X[:,3][1], temp_X[:,3][2]]
                # [pose5.pose.position.x,pose5.pose.position.y,pose5.pose.position.z] = [ temp_X[:,4][0], temp_X[:,4][1], temp_X[:,4][2]]
                # #print(pose1.pose.position, temp_X)
                #FLAG
                pub1.publish(pose1)
                #rospy.loginfo("Message_uav1: %s",pose1)
                # pub2.publish(pose2)
                # pub3.publish(pose3)
                # pub4.publish(pose4)
                # pub5.publish(pose5)

                #rospy.loginfo("Message_uav5: %s",pose5)
                #rospy.spin()
                #rate.sleep()

            # rospy.spin()
                rate.sleep()
            X_t = copy.deepcopy(X)
            #print(Last_pos.shape)
            position = np.c_[position, X]
            position = np.unique(position, axis=1)  
            if(abs(M[lead_index[0]][lead_index[1]]-Func.get_arrmax(M))>=0 and \
                    abs(M[lead_index[0]][lead_index[1]]-Func.get_arrmax(M))<0.14):
                np.savetxt('/home/uav/twf_ws/src/test_wifi/scripts/Last_pos.csv',Last_pos.T,delimiter=',')
                break
        break
    print(Last_position.shape)
    print(E_n.shape)
    a = Last_position[0,:]
    b = Last_position[1,:]
    c = E_n
    xx = np.linspace(0, 59, 60)
    yy = np.linspace(0, 59, 60)
    xx, yy = np.meshgrid(yy, xx)
    M1 = scipy.interpolate.griddata((a, b), c, (xx, yy), method='cubic')
    M1 = np.nan_to_num(M1)
    print(M1)
    np.savetxt('/home/M1.csv',M1,delimiter=',')

        

if __name__ == "__main__":
    wolf()