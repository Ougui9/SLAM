'''
File: Helper functions for SLAM Proj
Author: Yilun Zhang
Date:

'''
from load_data import *
import numpy as np
import matplotlib.pyplot as plt
def findClosestJointTime(ts_joint_raw,ts_lidar_cur):
    '''
    :param ts_joint_raw: (n_joint, 1)
    :param ts_lidar_cur: number
    :return:
    '''
    n_joint,_=ts_joint_raw.shape
    # ind_joint_new=[]
    # for i in range(n_lidar):
    diff=abs(ts_joint_raw-ts_lidar_cur)
    ind_joint_new=np.argmin(diff)
    return ind_joint_new
    # ind_joint=[]
    # # ind_lidar=[]
    #
    # n_lidar,_=ts_lidar.shape
    # n_joint,_=ts_joint.shape
    # if ts_joint[0]<=ts_lidar[0]:
    #     for i in range(n_lidar):
    #         ind_curjoint=0
    #         while ts_joint[ind_curjoint]<ts_lidar[i]:
    #             ind_curjoint+=1
    #             if ind_curjoint>=n_joint:
    #                 ind_joint.append(ind_curjoint-1)
    #                 break
    #     ind_lidar=np.arange(n_lidar)
    # else:
    #     ind_startlidar=0
    #     while ts_joint[0]>ts_lidar[ind_startlidar]:
    #         ind_startlidar+=1
    #     for i in range(ind_startlidar,n_lidar):
    #         ind_curjoint=0
    #         while ts_joint[ind_curjoint]<ts_lidar[i]:
    #             ind_curjoint+=1
    #             if ind_curjoint>=n_joint:
    #                 ind_joint.append(ind_curjoint-1)
    #                 break
    #     ind_lidar = np.arange(ind_startlidar,n_lidar)
    #
    # return ind_joint,ind_lidar

def rpy2rot(r,p,y):
    rot=np.zeros([3,3])
    a = y
    b = p
    c = r
    rot[0,0]=np.multiply(np.cos(a),np.cos(b))
    rot[0, 1] =np.multiply(np.multiply(np.cos(a),np.sin(b)),np.sin(c))-np.multiply(np.sin(a),np.cos(c))
    rot[0, 2] =np.multiply(np.multiply(np.cos(a),np.sin(b)),np.cos(c))+np.multiply(np.sin(a),np.sin(c))
    rot[1, 0] =np.multiply(np.sin(a),np.cos(b))
    rot[1, 1] =np.multiply(np.multiply(np.sin(a),np.sin(b)),np.sin(c))+np.multiply(np.cos(a),np.cos(c))
    rot[1, 2] =np.multiply(np.multiply(np.sin(a),np.sin(b)),np.cos(c))-np.multiply(np.cos(a),np.sin(c))
    rot[2, 0] =-np.sin(b)
    rot[2, 1] =np.multiply(np.cos(b),np.sin(c))
    rot[2, 2] =np.multiply(np.cos(b),np.cos(c))
    return rot

def rot2rpy(rot):
    '''
    :param rot: (3, 3)
    :return: (3,)
    '''
    rpy=np.zeros(3)
    rpy[2]=np.arctan2(rot[1,0],rot[0,0])
    rpy[1] = np.arctan2(-rot[2, 0], np.sqrt(rot[2, 1]**2+rot[2,2]**2))
    rpy[0] = np.arctan2(rot[2, 1], rot[2, 2])
    return rpy

def visualize2D(pose):
    # ax=plt.figure(0)
    # plt.hold(True)
    plt.xticks(np.linspace(-0.1, 3, 100, endpoint=True))
    plt.yticks(np.linspace(-0.1, 2, 100, endpoint=True))
    for i in range(1000,len(pose)):
        plt.scatter(pose[i,0],pose[i,1])
        plt.draw()
        plt.pause(0.001)

def embedOdoNew(pose_new,lidarData):
    n_lidar = len(lidarData)
    for i in range(n_lidar):
        lidarData[i]['pose'][0,-1]=pose_new[i,-1]

    return lidarData

def getData(jointPath, lidarPath):

    #get joint data
    jointData = get_joint(jointPath)
    # head_angles=jointData['head_angles']
    # ts_joint=jointData['ts'].T

    #get lidar data
    lidarData=get_lidar(lidarPath)
    # n_lidar=len(lidarData)

    # ts_lidar=np.zeros([n_lidar,1])#(n,1)
    # rpy=np.zeros([n_lidar,3])#(n,3)
    # for i in range(n_lidar):
    #     ts_lidar[i]=lidarData[i]['t'][0,0]
    #     rpy[i]=lidarData[i]['rpy'][0,0]
    # ind_joint_eff, ind_lidar_eff=findClosestTime(ts_joint,ts_lidar)

    #extract info by useful timestamps
    return jointData,lidarData