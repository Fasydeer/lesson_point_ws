#-*- coding:utf-8 -*-
# author:贤宁
# datetime:2022/4/24 9:08
# software: PyCharm
from array import array
import numpy as np
import random
import scipy
from scipy import signal
import scipy.interpolate
from scipy.spatial.distance import cdist
import copy
import math
# import matlab.engine
# eng = matlab.engine.start_matlab()
class Function:
    def __init__(self,num):
        self.num = num
    #获取矩阵的最大值
    def get_arrmax(self, arr):
        index = np.unravel_index(np.argmax(arr, axis=None), arr.shape)
        result = arr[index]
        return result
    #以概率生成(1-8）随机数
    def random_prob(self,P):
        P = np.around(P, 3)
        P = 1000*P
        Adaptation = P.astype(int)
        sum = np.sum(P)
        sum = np.int(sum)
        rand = random.sample(range(0, sum), 1)
        num = 0
        for i in range(8):
            if(0+num<=rand[0]<Adaptation[i]+num):
                result = i
                break
            else:
                num = num +Adaptation[i]
                continue
        return result
    #基于信息素的探狼飞行决策
    def updatepos(self,pos,AP,count):
        Vector = np.array([[1,1],[-1,1],[1,-1],[-1,-1],[0,-1],[1,0],[-1,0],[0,1]])
        V = np.zeros(8)
        i = pos[0]
        j = pos[1]
        if(count==0):
            TT=5
        elif(count==1):
            TT=2
        '''
        NE:表示东北方向，以此类似
        '''
        #NE
        try:
            V[0] = AP[i+1][j+1]
        except ZeroDivisionError as e:
            V[0] = -999
        #NW
        try:
            V[1] = AP[i-1][j+1]
        except ZeroDivisionError as e:
            V[1] = -999
        #SE
        try:
            V[2] = AP[i+1][j-1]
        except ZeroDivisionError as e:
            V[2] = -999
        #SW
        try:
            V[3] = AP[i-1][j-1]
        except ZeroDivisionError as e:
            V[3] = -999
        #S
        try:
            V[4] = AP[i][j-1]
        except ZeroDivisionError as e:
            V[4] = -999
        #E
        try:
            V[5] = AP[i+1][j]
        except ZeroDivisionError as e:
            V[5] = -999
        #W
        try:
            V[6] = AP[i-1][j]
        except ZeroDivisionError as e:
            V[6] = -999
        #N
        try:
            V[7] = AP[i][j+1]
        except ZeroDivisionError as e:
            V[7] = -999

        Vch = V[V>=0]
        Vout = V
        Vout[V<0] = 0
        if(Vch.shape[0]!=0):
            P = Vout/np.sum(Vout)
        else:
            P =1/np.minimum(V,-V)/np.sum(1/np.minimum(V,-V))
        a = self.random_prob(P)
        pos = pos + TT*Vector[a,:].T
        return pos
    #判断其游走坐标是否超出环境边界，超出则拉回
    def judge(self,x,y):
        if(x<0):
            x=0
        if(y<0):
            y=0
        if(x>=59):
            x=58
        if(y>=59):
            y=58
        return np.array([x,y])
    #更新信息素矩阵
    def updateSa(self,AP,K,Ea,Ga,da,P):
        updateGa = np.zeros_like(AP)
        for i in range(1,AP.shape[0]-2):
            for j in range(1,AP.shape[1]-2):
                updateGa[i][j] = AP[i-1][j]+AP[i+1][j]+AP[i][j-1]+AP[i][j+1]+AP[i-1][j-1]+AP[i-1][j+1]+AP[i+1][j+1]+AP[i+1][j-1];
                updateGa[i][j] = (updateGa[i][j] + 8*da)/8

        for i in range(1, AP.shape[0]-2):
            updateGa[i][0] = AP[i-1][0]+AP[i+1][0]+AP[i-1][1]+AP[i][1]+AP[i+1][1]
            updateGa[i][0] = (updateGa[i][0] + 5 * da) / 5
            updateGa[i][AP.shape[1]-1] = AP[i-1][AP.shape[1]-1]+AP[i+1][AP.shape[1]-1]+\
                                         AP[i-1][AP.shape[1]-1-1]+AP[i][AP.shape[1]-1-1]+AP[AP.shape[1]-1][j]+\
                                         AP[i+1][AP.shape[1]-1-1]
            updateGa[i][AP.shape[1]-1] = (updateGa[i][AP.shape[1]-1]+5*da)/5

        for j in range(1,AP.shape[1]-2):
            updateGa[0][j] = AP[0][j - 1] + AP[0][j + 1] + AP[1][j-1]+AP[1][j] + AP[1][j + 1]
            updateGa[0][j] = (updateGa[0][j] + 5 * da) / 5
            updateGa[AP.shape[0] - 1][j] = AP[AP.shape[0] - 1][j - 1] + AP[AP.shape[0] - 1][j + 1] + \
                                           AP[AP.shape[0] - 1 - 1][j - 1] + AP[AP.shape[0] - 1 - 1][j] + \
                                           AP[AP.shape[0] - 1][j] + AP[AP.shape[0] - 1 - 1][j + 1]
            updateGa[AP.shape[0] - 1][j] = (updateGa[AP.shape[0] - 1][j] + 5 * da) / 5
        updateGa[0][0] = (AP[0][1]+AP[1][1]+AP[1][0]+3*da)/3
        updateGa[0][AP.shape[1]-1] = (AP[0][AP.shape[1]-1-1]+AP[1][AP.shape[1]-1-1]+AP[1][AP.shape[1]-1]+3*da)/3
        updateGa[AP.shape[0]-1][0] = (AP[AP.shape[0]-1][1]+AP[AP.shape[0]-1-1][1]+AP[AP.shape[0]-1-1][0]+3*da)/3
        updateGa[AP.shape[0]-1][AP.shape[1]-1] = (AP[AP.shape[0]-1-1][AP.shape[1]-1]+AP[AP.shape[0]-1-1][AP.shape[1]-1-1]+AP[AP.shape[0]-1][AP.shape[1]-1-1]+3*da)/3
        output = (1-Ea)*((1-Ga)*(AP+K*P*da)+updateGa)
        return output
    #自定义sigmoid
    def sigmoid(self,x):
        temp = 1 + np.exp(-x)
        return 1/temp
    #更新优先级矩阵
    def updatePrem(self,pos,M):
        a = pos[0,:]
        b = pos[1,:]
        c = M[a,b]
        xx = np.linspace(0, 59, 60)
        yy = np.linspace(0, 59, 60)
        xx, yy = np.meshgrid(yy, xx)
        E = scipy.interpolate.griddata((a, b), c, (xx, yy), method='cubic')
        E = np.nan_to_num(E)
        parm = (E-np.mean(E))/np.std(E)

        Prem = self.sigmoid(parm)
        return Prem
    #获取飞行角度
    def get_angel(self,lead_pos,brave_pos):
        dis = lead_pos - brave_pos
        range = np.round(dis/(np.linalg.norm(dis)+10e-6))
        return range
    def generate_pos3(self,X1,X1_t,X2,X2_t,X3,X3_t,slot_num):
        lead_out = np.array([])
        explore_out = np.array([])
        M_out = np.array([])
        X_pos = np.array([])
        lead_pos1 = np.array([])
        explore_pos1 = np.array([])
        M_pos1 = np.array([])
        p_data1 = (X1-X1_t)/slot_num
        p_data2 = (X2-X2_t)/slot_num
        p_data3 = (X3-X3_t)/slot_num
        for slot in range(1,slot_num+1):
            lead_pos1 = np.vstack((X1_t[0]+slot*p_data1[0],X1_t[1]+slot*p_data1[1]))
            explore_pos1 = np.vstack((X2_t[0]+slot*p_data2[0],X2_t[1]+slot*p_data2[1]))
            M_pos1 = np.vstack((X3_t[0]+slot*p_data3[0],X3_t[1]+slot*p_data3[1]))
            X_pos1 = np.hstack((lead_pos1,explore_pos1,M_pos1))
            if(slot==1):
                X_pos = copy.deepcopy(X_pos1)
            else:
                X_pos = np.hstack((X_pos,X_pos1))
        for i in range(X_pos.shape[1]):
                if(i==X_pos.shape[1]-1):
                    break
                for j in range(i+1,X_pos.shape[1]):
                    all_dis = cdist([X_pos[:,i]], [X_pos[:,j]])[0][0]
                    if(all_dis<10):
                        d = math.sqrt((100-all_dis*all_dis) / 2)
                        if X_pos[1,j]>=X_pos[1,i]:
                            if abs(X_pos[1,j]-X_pos[1,i]) >= abs(X_pos[0,j]-X_pos[0,i]):
                                X_pos[1,j] = X_pos[1,j] + d
                            elif abs(X_pos[1,j]-X_pos[1,i]) < abs(X_pos[0,j]-X_pos[0,i]):
                                if X_pos[0,j]-X_pos[0,i] >=0:
                                    X_pos[0,j] = X_pos[0,j] + d
                                elif X_pos[0,j]-X_pos[0,i] <0:
                                    X_pos[0,j] = X_pos[0,j] - d
                            X_pos[:,j] = self.judge(X_pos[0,j],X_pos[1,j])
                        elif X_pos[1,j]<X_pos[1,i]:
                            if abs(X_pos[1,j]-X_pos[1,i]) >= abs(X_pos[0,j]-X_pos[0,i]):
                                X_pos[1,j] = X_pos[1,j] - d
                            elif abs(X_pos[1,j]-X_pos[1,i]) < abs(X_pos[0,j]-X_pos[0,i]):
                                if X_pos[0,j]-X_pos[0,i] >= 0:
                                    X_pos[0,j] = X_pos[0,j] + d
                                elif X_pos[0,j]-X_pos[0,i] < 0:
                                    X_pos[0,j] = X_pos[0,j] - d
                            X_pos[:,j] = self.judge(X_pos[0,j],X_pos[1,j])
        L_label = [0,5,10,15,20,25,30,35,40,45]
        M_label = [4,9,14,19,24,29,34,39,44,49]
        lead_out = X_pos[:,L_label]
        M_out  = X_pos[:,M_label]
        K = np.arange(1, 47, 5)
        for k in range(len(K)):
            explore_out1 = X_pos[:,K[k]:K[k]+3]
            if(k==0):
                explore_out = copy.deepcopy(explore_out1)
            else:
                explore_out = np.hstack((explore_out,explore_out1))
        return lead_out,explore_out,M_out

    def generate_pos1(self,X1,X1_t,X2,X2_t,X3,X3_t,slot_num):
        lead_out = np.array([])
        explore_out = np.array([])
        M_out = np.array([])
        p_data1 = (X1-X1_t)/slot_num
        p_data2 = (X2-X2_t)/slot_num
        p_data3 = (X3-X3_t)/slot_num

        for slot in range(1,slot_num+1):
            lead_pos = np.vstack((X1_t[0]+slot*p_data1[0],X1_t[1]+slot*p_data1[1]))
            explore_pos = np.vstack((X2_t[0]+slot*p_data2[0],X2_t[1]+slot*p_data2[1]))
            M_pos = np.vstack((X3_t[0]+slot*p_data3[0],X3_t[1]+slot*p_data3[1]))
            X_pos = np.hstack((lead_pos,explore_pos,M_pos))
            for i in range(X_pos.shape[1]):
                if(i==X_pos.shape[1]-1):
                    break
                for j in range(i+1,X_pos.shape[1]):
                    all_dis = cdist([X_pos[:,i]], [X_pos[:,j]])[0][0]
                    if(all_dis<10):
                        d = (math.sqrt((100-all_dis*all_dis) / 2)) / 2
                        if(X_pos[0,j]>=X_pos[0,i] and X_pos[1,j]>=X_pos[1,i]):
                            X_pos[:,j] = X_pos[:,j] + d
                            X_pos[:,j] = self.judge(X_pos[0,j],X_pos[1,j])
                        elif(X_pos[0,j]<=X_pos[0,i] and X_pos[1,j]<X_pos[1,i]):
                            X_pos[:,j] = X_pos[:,j] - d
                            X_pos[:,j] = self.judge(X_pos[0,j],X_pos[1,j])
                        elif(X_pos[0,j]>=X_pos[0,i] and X_pos[1,j]<X_pos[1,i]):
                            X_pos[0,j] = X_pos[0,j] + d
                            X_pos[1,j] = X_pos[1,j] - d
                            X_pos[:,j] = self.judge(X_pos[0,j],X_pos[1,j])
                        elif(X_pos[0,j]<X_pos[0,i] and X_pos[1,j]>=X_pos[1,i]):
                            X_pos[0,j] = X_pos[0,j] - d
                            X_pos[1,j] = X_pos[1,j] + d
                            X_pos[:,j] = self.judge(X_pos[0,j],X_pos[1,j])

            if(slot==1):
                lead_out = X_pos[:,0]
                explore_out = X_pos[:,1:4]
                M_out = X_pos[:,4]
                continue
            lead_out = copy.deepcopy(np.c_[lead_out, X_pos[:, 0]])
            explore_out = np.hstack((explore_out, X_pos[:, 1:4]))
            M_out = copy.deepcopy(np.c_[M_out,X_pos[:,4]])
           # M_out = np.hstack((M_out, X_pos[:, 4]))
        return lead_out,explore_out,M_out
    #获取插值后以及避撞的坐标
    def generate_pos(self,X1,X1_t,X2,X2_t,X3,X3_t,slot_num):
        lead_out = np.array([])
        explore_out = np.array([])
        M_out = np.array([])
        p_data1 = (X1-X1_t)/slot_num
        p_data2 = (X2-X2_t)/slot_num
        p_data3 = (X3-X3_t)/slot_num

        for slot in range(1,slot_num+1):
            lead_pos = np.vstack((X1_t[0]+slot*p_data1[0],X1_t[1]+slot*p_data1[1]))
            explore_pos = np.vstack((X2_t[0]+slot*p_data2[0],X2_t[1]+slot*p_data2[1]))
            M_pos = np.vstack((X3_t[0]+slot*p_data3[0],X3_t[1]+slot*p_data3[1]))
            X_pos = np.hstack((lead_pos,explore_pos,M_pos))
            for i in range(X_pos.shape[1]):
                if(i==X_pos.shape[1]-1):
                    break
                for j in range(i+1,X_pos.shape[1]):
                    ##判别是否“相撞” 1)用距离判断；2)直接判断是否重合
                    all_dis = cdist([X_pos[:,i]], [X_pos[:,j]])[0][0]
                    if (all_dis <= 10 and all_dis >= 0):
                    #if all(X_pos[:,i]==X_pos[:,j]):
                        X_pos[:,j]=X_pos[:,j] - np.array([0,9])
                        if (X_pos[1,j]<=0):
                            X_pos[:,j]=X_pos[:,j] + np.array([0,9]) - np.array([9,0])
                            X_pos[:,j] = self.judge(X_pos[0,j],X_pos[1,j])
            if(slot==1):
                lead_out = X_pos[:,0]
                explore_out = X_pos[:,1:4]
                M_out = X_pos[:,4]
                continue
            lead_out = copy.deepcopy(np.c_[lead_out, X_pos[:, 0]])
            explore_out = np.hstack((explore_out, X_pos[:, 1:4]))
            M_out = copy.deepcopy(np.c_[M_out,X_pos[:,4]])
           # M_out = np.hstack((M_out, X_pos[:, 4]))
        return lead_out,explore_out,M_out
    def generate_pos2(self,X1,X1_t,X2,X2_t,X3,X3_t,slot_num):
        lead_out = np.array([])
        explore_out = np.array([])
        M_out = np.array([])
        X_pos = np.array([])
        lead_pos = np.array([])
        explore_pos = np.array([])
        M_pos = np.array([])
        p_data1 = (X1-X1_t)/slot_num
        p_data2 = (X2-X2_t)/slot_num
        p_data3 = (X3-X3_t)/slot_num
        for slot in range(1,slot_num+1):
            lead_pos1 = np.vstack((X1_t[0]+slot*p_data1[0],X1_t[1]+slot*p_data1[1]))
            explore_pos1 = np.vstack((X2_t[0]+slot*p_data2[0],X2_t[1]+slot*p_data2[1]))
            M_pos1 = np.vstack((X3_t[0]+slot*p_data3[0],X3_t[1]+slot*p_data3[1]))
            X_pos1 = np.hstack((lead_pos1,explore_pos1,M_pos1))
            if(slot==1):
                X_pos = copy.deepcopy(X_pos1)
            else:
                X_pos = np.hstack((X_pos,X_pos1))
        for i in range(X_pos.shape[1]):
                if(i==X_pos.shape[1]-1):
                    break
                for j in range(i+1,X_pos.shape[1]):
                    all_dis = cdist([X_pos[:,i]], [X_pos[:,j]])[0][0]
                    if(all_dis<10):
                        if(X_pos[0,j]>=X_pos[0,i] and X_pos[1,j]>=X_pos[1,i]):
                            X_pos[:,j] = X_pos[: , j]  + np.array([np.around(10-all_dis)/1.414,np.around(10-all_dis)/1.414])
                            X_pos[:,j] = self.judge(X_pos[0,j],X_pos[1,j])
                        elif(X_pos[0,j]<=X_pos[0,i] and X_pos[1,j]<X_pos[1,i]):
                            X_pos[:,j] = X_pos[: , j]  - np.array([np.around(10-all_dis)/1.414,np.around(10-all_dis)/1.414])
                            X_pos[:,j] = self.judge(X_pos[0,j],X_pos[1,j])
                        elif(X_pos[0,j]>=X_pos[0,i] and X_pos[1,j]<X_pos[1,i]):
                            X_pos[0,j] = X_pos[0,j] + np.around(10-all_dis)/1.414
                            X_pos[1,j] = X_pos[1,j] - np.around(10-all_dis)/1.414
                            X_pos[:,j] = self.judge(X_pos[0,j],X_pos[1,j])
                        elif(X_pos[0,j]<X_pos[0,i] and X_pos[1,j]>=X_pos[1,i]):
                            X_pos[0,j] = X_pos[0,j] - np.around(10-all_dis)/1.414
                            X_pos[1,j] = X_pos[1,j] + np.around(10-all_dis)/1.414
                            X_pos[:,j] = self.judge(X_pos[0,j],X_pos[1,j])
        L_label = [0,5,10,15,20,25,30,35,40,45]
        M_label = [4,9,14,19,24,29,34,39,44,49]
        lead_out = X_pos[:,L_label]
        M_out  = X_pos[:,M_label]
        K = np.arange(1, 47, 5)
        for k in range(len(K)):
            explore_out1 = X_pos[:,K[k]:K[k]+3]
            if(k==0):
                explore_out = copy.deepcopy(explore_out1)
            else:
                explore_out = np.hstack((explore_out,explore_out1))
        return lead_out,explore_out,M_out