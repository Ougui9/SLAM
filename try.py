import numpy as np
from load_data import *
from helper import *
# def read

folder='./data/'
lidar_file='train_lidar0'
joint_file='train_joint0'


# def matchTime(ts_lidar,ts_joint):
#     if ts_lidar[0]<=ts_joint[0]:
#
#     return

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





    # return

def getOdometryPara(lidarData): #head_angles_B: (n_joint,2)
    n_lidar=len(lidarData)
    ori_H=np.array([1, 0, 0]).reshape(3,1)
    # pos=np.array([0,0])
    pose_G=np.zeros([n_lidar,3]) #(n,3)
    rpy_B=np.zeros([n_lidar,3]) #(n,3)
    # head_H=np.zeros([n_lidar,2]) #(n,2)()
    for i in range(n_lidar):
        pose_G[i]=lidarData[i]['pose'][0,0]
        rpy_B[i] = lidarData[i]['rpy'][0, 0]

    rpy_B=rpy_B-rpy_B[0]
    pos_theta_G=pose_G#(n,3)
    # pos_theta_G=
    for i in range(1, n_lidar):

        pos_theta_G[i,:2]=pos_theta_G[i-1,:2]+pose_G[i-1,:2]


    return pose





if __name__ =='__main__':
    jointData,lidarData=getData(jointPath=folder+joint_file,lidarPath=folder+lidar_file)

    pose_odo=getOdometryPara(lidarData)
