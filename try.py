import numpy as np
from load_data import *
from helper import *
# def read

folder='./data/'
lidar_file='train_lidar0'
joint_file='train_joint0'

##paras
dis_H_mass=0.33
dis_imu_mass=-16/100


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

def getOdometryPara(lidarData,jointData): #head_angles_B: (n_joint,2)
    n_lidar=len(lidarData)
    rot_H_G =np.zeros([3,3,n_lidar])
    # d_liar_imu =np.array([0,0,dis_H_mass-dis_imu_mass,1])
    # H_lidar_imu=np.zeros([4,4])
    # H_imu_G=np.zeros([4,4])
    # d_imu_G=np.array()

    lidarPose_lidar_G=np.zeros([n_lidar,3]) #(n,3)
    rpy_imu_G=np.zeros([n_lidar,3]) #(n,3)
    ts_lidar=np.zeros([n_lidar,3])
    for i in range(n_lidar):
        lidarPose_lidar_G[i]=lidarData[i]['pose'][0,0]
        rpy_imu_G[i] = lidarData[i]['rpy'][0, 0]
        ts_lidar=lidarData[i]['t'][0, 0]
    rpy_imu_G = rpy_imu_G - rpy_imu_G[0]
    #match time
    ts_joint_raw=jointData['ts'].T
    ind_joint=findClosestJointTime(ts_joint_raw,ts_lidar)
    head_angles_H_B=jointData['head_angles'].T[ind_joint]#(n,2)yaw,pitch

    # contruct rot lider wrt imu
    rot_H_B=rpy2rot(np.zeros(n_lidar,1),head_angles_H_B[:,1],head_angles_H_B[:,0])#(3,3,n) same with rot_lidar_imu
    # H_lidar_imu[:3,:3]=rot_H_B
    # H_lidar_imu[:,-1]=d_liar_imu

    # contruct rot imu wrt G
    rot_B_G=rpy2rot(rpy_imu_G[:,0],rpy_imu_G[:,1],rpy_imu_G[:,2])

    #rot H wrt G
    for i in range(n_lidar):
        rot_H_G[:,:,i]=rot_B_G[:,:,i].dot(rot_H_B[:,:,i])

    yaw=rot2rpy(rot_H_G)[-1]
    lidarPose_lidar_G[:,-1]=yaw

    return lidarPose_lidar_G #(n,3)
    # pose_new_G=pose_G#(n,3)
    # # # pos_theta_G=
    # for i in range(1, n_lidar):
    #
    #     pos_theta_G[i,:2]=pos_theta_G[i-1,:2]+pose_G[i-1,:2]
    #
    #
    # return pose





if __name__ =='__main__':
    jointData,lidarData=getData(jointPath=folder+joint_file,lidarPath=folder+lidar_file)

    pose_odo=getOdometryPara(lidarData,jointData)
