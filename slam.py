import numpy as np
from load_data import *
from helper import *
import matplotlib.pyplot as plt
import p4_util as util
folder='./data/'
lidar_file='train_lidar0'
joint_file='train_joint0'

##paras
dis_lidar_H=0.15
dis_H_mass=0.33
dis_imu_mass=-16/100
dis_mass_G=0.93


def getRelOdometry(lidarData,jointData): #head_angles_B: (n_joint,2)
    n_lidar=len(lidarData)
    rot_H_G =np.zeros([3,3,n_lidar])
    # d_liar_imu =np.array([0,0,dis_H_mass-dis_imu_mass,1])
    # H_lidar_imu=np.zeros([4,4])
    # H_imu_G=np.zeros([4,4])
    # d_imu_G=np.array()

    lidarPose_lidar_G=np.zeros([n_lidar,3]) #(n,3)
    rpy_imu_G=np.zeros([n_lidar,3]) #(n,3)
    ts_lidar=np.zeros([n_lidar,1])
    for i in range(n_lidar):
        lidarPose_lidar_G[i]=lidarData[i]['pose'][0]
        rpy_imu_G[i] = lidarData[i]['rpy'][0, 0]
        ts_lidar[i]=lidarData[i]['t'][0, 0]
    rpy_imu_G = rpy_imu_G - rpy_imu_G[0]
    #match time
    ts_joint_raw=jointData['ts'].T
    ind_joint=findClosestJointTime(ts_joint_raw,ts_lidar)
    head_angles_H_B=jointData['head_angles'].T[ind_joint]#(n,2)yaw,pitch

    # contruct rot lider wrt imu
    rot_H_B=rpy2rot(np.zeros(n_lidar),head_angles_H_B[:,1],head_angles_H_B[:,0])#(3,3,n) same with rot_lidar_imu


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
def calT(lidarData,jointData):
    n_lidar = len(lidarData)
    T_H_G = np.zeros([4, 4, n_lidar])
    T_H_B = np.zeros([4, 4, n_lidar])
    T_B_G = np.zeros([4, 4, n_lidar])

    #set d
    d_H_B = np.array([0, 0, dis_H_mass, 1])
    d_B_G = np.array([0, 0, dis_mass_G, 1])


    lidarPose_lidar_G = np.zeros([n_lidar, 3])  # (n,3)
    rpy_imu_G = np.zeros([n_lidar, 3])  # (n,3)
    ts_lidar = np.zeros([n_lidar, 1])
    for i in range(n_lidar):
        lidarPose_lidar_G[i] = lidarData[i]['pose'][0]
        rpy_imu_G[i] = lidarData[i]['rpy'][0, 0]
        ts_lidar[i] = lidarData[i]['t'][0, 0]
    rpy_imu_G = rpy_imu_G - rpy_imu_G[0]
    # match time
    ts_joint_raw = jointData['ts'].T
    ind_joint = findClosestJointTime(ts_joint_raw, ts_lidar)
    head_angles_H_B = jointData['head_angles'].T[ind_joint]  # (n,2)yaw,pitch

    # contruct T H wrt B
    rot_H_B = rpy2rot(np.zeros(n_lidar), head_angles_H_B[:, 1],
                      head_angles_H_B[:, 0])  # (3,3,n) same with rot_lidar_imu

    # contruct T B wrt G
    rot_B_G = rpy2rot(rpy_imu_G[:, 0], rpy_imu_G[:, 1], rpy_imu_G[:, 2])
    T_B_G[:3,:3,:]=rot_B_G
    f


    return T_H_B,T_B_G,T_H_G
def mapping(scan):




if __name__ =='__main__':
    jointData,lidarData=getData(jointPath=folder+joint_file,lidarPath=folder+lidar_file)



    pose_odo_new=getOdometryPara(lidarData,jointData)#(n, 3)

    lidarData=embedOdoNew(pose_odo_new,lidarData)
    # util.replay_lidar(lidarData)
    # visualize2D(pose_odo_new)
    plt.scatter(pose_odo_new[:,0],pose_odo_new[:,1])
    plt.show()


