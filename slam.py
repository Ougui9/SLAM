import numpy as np

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


ground_thre=0.1

def getRelOdometry(lidarData_cur,lidarData_pre,T_H_B, T_B_G): #head_angles_B: (n_joint,2)
    lidarPose_lidar_G_cur, lidarPose_lidar_G_pre = lidarData_cur['pose'][0], lidarData_pre['pose'][0]
    # dlidarPose_lidar_G_cur=lidarPose_lidar_G_cur- lidarPose_lidar_G_pre

    rot_H_B=T_H_B[:3,:3]
    rot_B_G=T_B_G[:3,:3]

    rot_H_G=rot_B_G.dot(rot_H_B)

    yaw=rot2rpy(rot_H_G)[-1]
    lidarPose_lidar_G_cur[-1]=yaw

    return lidarPose_lidar_G_cur #(3,)

def calT(jointData,lidarData_cur,lidarData_pre,lidarData_0):
    # n_lidar = len(lidarData_cur)
    T_H_G = np.zeros([4, 4])
    T_H_B = np.zeros([4, 4])
    T_B_G = np.zeros([4, 4])


    lidarPose_lidar_G_cur, lidarPose_lidar_G_pre = lidarData_cur['pose'][0], lidarData_pre['pose'][0]
    rpy_imu_G_cur, rpy_imu_G_0 = lidarData_cur['rpy'][0], lidarData_0['rpy'][0]
    rpy_imu_G_cur_unbais = rpy_imu_G_cur - rpy_imu_G_0
    ts_lidar_cur = lidarData_cur['t'][0, 0]


    # match time
    ts_joint_raw = jointData['ts'].T
    ts_lidar_cur=lidarData_cur['t'][0,0]
    ind_joint = findClosestJointTime(ts_joint_raw, ts_lidar_cur)
    head_angles_H_B = jointData['head_angles'].T[ind_joint]  # (n,2)yaw,pitch


    # contruct T H wrt B
    rot_H_B = rpy2rot(0, head_angles_H_B[1],
                      head_angles_H_B[0])  # (3,3,n) same with rot_lidar_imu
    d_H_B = np.array([0, 0, dis_H_mass, 1])
    T_H_B[:3, :3] = rot_H_B
    T_H_B[:, -1] = d_H_B
    # contruct T B wrt G
    rot_B_G = rpy2rot(rpy_imu_G_cur_unbais[0], rpy_imu_G_cur_unbais[1], rpy_imu_G_cur_unbais[2])

    d_B_G = np.array([lidarPose_lidar_G_cur[0], lidarPose_lidar_G_cur[1], dis_mass_G, 1])#???

    T_B_G[:3,:3]=rot_B_G
    T_B_G[:,-1]=d_B_G

    T_H_G=T_B_G.dot(T_H_B)

    return T_H_B,T_B_G,T_H_G,ind_joint
# def mapping(scan):
#
def correctScan(lidarData_current,T_H_G):
    # head_angles_H_B = jointData['head_angles'].T[ind_joint]  # (n,2)yaw,pitch
    scan_raw=lidarData_current['scan'][0]
    n_scan=len(scan_raw)
    gamma=np.arange(-135,136,0.25)
    scan_pts_lidar=scan_raw.reshape(-1,1).dot(np.array([np.cos(gamma),np.sin(gamma)]).reshape(1,2))
    scan_pts_H=np.zeros([n_scan,3])
    scan_pts_H[:,:2] = scan_pts_lidar
    scan_pts_H[-1]=dis_lidar_H
    scan_G = T_H_G.dot(scan_pts_H)
    valid=scan_G[:,-1]>ground_thre

    return scan_G,valid #(3, )

def mapping(scan_H,T_H_G):
    scan_G=T_H_G.dot(scan_H)


def slam(jointData, lidarData_current, lidarData_previous,lidarData_0):

    if lidarData_previous==None:
        lidarData_previous=lidarData_current

    T_H_B, T_B_G, T_H_G,ind_joint=calT(jointData, lidarData_current,lidarData_previous,lidarData_0)
    pose_odo_new=getRelOdometry(lidarData_current,lidarData_previous,T_H_B,T_B_G)#(n, 2), list, list

    scan_G,valid=correctScan(lidarData_current,T_H_G)
    mapping(scan_pts_H,T_H_G)
    # util.replay_lidar(lidarData)
    # visualize2D(pose_odo_new)
    plt.scatter(pose_odo_new[:,0],pose_odo_new[:,1])
    plt.show()


