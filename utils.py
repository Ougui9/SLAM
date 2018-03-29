import numpy as np

from helper import findClosestJointTime,rpy2rot
import matplotlib.pyplot as plt


##paras
dis_lidar_H=0.15
dis_H_mass=0.33
dis_imu_mass=-16/100
dis_mass_G=0.93


ground_thre=0.1

# def getRelOdometry(lidarData_cur,lidarData_pre,T_H_B, T_B_G): #head_angles_B: (n_joint,2)
#     lidarPose_lidar_G_cur, lidarPose_lidar_G_pre = lidarData_cur['pose'][0], lidarData_pre['pose'][0]
#     # dlidarPose_lidar_G_cur=lidarPose_lidar_G_cur- lidarPose_lidar_G_pre
#
#     rot_H_B=T_H_B[:3,:3]
#     rot_B_G=T_B_G[:3,:3]
#
#     rot_H_G=rot_B_G.dot(rot_H_B)
#
#     yaw=rot2rpy(rot_H_G)[-1]
#     lidarPose_lidar_G_cur[-1]=yaw
#
#     return lidarPose_lidar_G_cur #(3,)

# def calT(jointData,lidarData_cur,lidarData_pre,lidarData_0):
#     # n_lidar = len(lidarData_cur)
#     T_H_G = np.zeros([4, 4])
#     T_H_B = np.zeros([4, 4])
#     T_B_G = np.zeros([4, 4])
#
#
#     lidarPose_lidar_G_cur, lidarPose_lidar_G_pre = lidarData_cur['pose'][0], lidarData_pre['pose'][0]
#     rpy_imu_G_cur, rpy_imu_G_0 = lidarData_cur['rpy'][0], lidarData_0['rpy'][0]
#     rpy_imu_G_cur_unbais = rpy_imu_G_cur - rpy_imu_G_0
#     ts_lidar_cur = lidarData_cur['t'][0, 0]
#
#
#     # match time
#     ts_joint_raw = jointData['ts'].T
#     ts_lidar_cur=lidarData_cur['t'][0,0]
#     ind_joint = findClosestJointTime(ts_joint_raw, ts_lidar_cur)
#     head_angles_H_B = jointData['head_angles'].T[ind_joint]  # (n,2)yaw,pitch
#
#
#     # contruct T H wrt B
#     rot_H_B = rpy2rot(0, head_angles_H_B[1],
#                       head_angles_H_B[0])  # (3,3,n) same with rot_lidar_imu
#     d_H_B = np.array([0, 0, dis_H_mass, 1])
#     T_H_B[:3, :3] = rot_H_B
#     T_H_B[:, -1] = d_H_B
#     # contruct T B wrt G
#     rot_B_G = rpy2rot(rpy_imu_G_cur_unbais[0], rpy_imu_G_cur_unbais[1], rpy_imu_G_cur_unbais[2])
#
#     d_B_G = np.array([lidarPose_lidar_G_cur[0], lidarPose_lidar_G_cur[1], dis_mass_G, 1])#???
#
#     T_B_G[:3,:3]=rot_B_G
#     T_B_G[:,-1]=d_B_G
#
#     T_H_G=T_B_G.dot(T_H_B)
#
#     return T_H_B,T_B_G,T_H_G,ind_joint
# def mapping(scan):
#
def correctRange(range_raw,T_H_G):
    # head_angles_H_B = jointData['head_angles'].T[ind_joint]  # (n,2)yaw,pitch
    # range_raw=lidarData_current['scan'][0]
    n_range=len(range_raw)
    valid_c = range_raw > 0.1
    valid_f = range_raw < 30
    angles=np.arange(-135,135.25,0.25)*np.pi/180.
    range_pts_lidar=range_raw.reshape(-1,1)*np.array([np.cos(angles),np.sin(angles)]).T
    range_pts_H=np.zeros([n_range,3])
    range_pts_H[:,:2] = range_pts_lidar
    range_pts_H[-1]=dis_lidar_H
    range_pts_G = T_H_G.dot(np.concatenate((range_pts_H.T,np.ones([1,n_range])),axis=0))[:3].T
    valid_g=range_pts_G[:,-1]>ground_thre
    valid_pro=np.logical_and(np.logical_and(valid_c,valid_g),valid_f)
    # scan_pts_G=range_pts_G[valid]
    # angles=angles[valid]
    valid_cor=np.logical_and(valid_pro,range_raw < 15)
    # array_unique(array_merge($array1,$array2), SORT_REGULAR)
    return range_pts_G,valid_pro, valid_cor #(n,3),(n,)


def calT_h_b(ts1,joint):
    T_H_B = np.zeros([4, 4])
    # match time
    ts_joint_raw = joint['ts'].T
    # ts_lidar0 = lidar1['t'][0, 0]
    ind_joint = findClosestJointTime(ts_joint_raw, ts1)
    head_angles_H_B = joint['head_angles'].T[ind_joint]  # (n,2)yaw,pitch

    rot_H_B = rpy2rot(0, head_angles_H_B[1], head_angles_H_B[0])  # (3,3,n) same with rot_lidar_imu
    d_H_B = np.array([0, 0, dis_H_mass, 1])
    T_H_B[:3, :3] = rot_H_B
    T_H_B[:, -1] = d_H_B

    # rot_lidar_B = rpy2rot(0, head_angles_H_B[1], head_angles_H_B[0])  # (3,3,n) same with rot_lidar_imu
    # d_lidar_B = np.array([0, 0, dis_H_mass, 1])
    # T_lidar_B[:3, :3] = rot_lidar_B
    # T_lidar_B[:, -1] = d_lidar_B

    return T_H_B


def cal_T_b_g(x_p_best,y_p_best,rpy_unbiased):
    T_B_G = np.zeros([4, 4])
    # rpy_imu_G_1, rpy_imu_G_0 = lidar1['rpy'][0], lidar0['rpy'][0]
    # rpy_imu_G_cur_unbais = rpy_imu_G_1 - rpy_imu_G_0

    rot_B_G = rpy2rot(rpy_unbiased[0], rpy_unbiased[1], rpy_unbiased[2])

    d_B_G = np.array([x_p_best, y_p_best, dis_mass_G, 1])  # ???

    T_B_G[:3, :3] = rot_B_G
    T_B_G[:, -1] = d_B_G

    return T_B_G

def rangeH2rangeG(range_H,T_H_G):
    n_range = len(range_H)
    range_G = T_H_G.dot(np.concatenate((range_H.T, np.ones([1, n_range])), axis=0))[:3].T

    return range_G

def rangeRaw2range_xyz_Lidar(range_raw,angles):
    n_range = len(range_raw)
    range_lidar = np.zeros([n_range, 3])
    range_lidar[:, :2] = range_raw.reshape(-1, 1) * np.array([np.cos(angles), np.sin(angles)]).T
    # range_lidar[:, :2] = range_lidar
    # range_H[-1] = dis_lidar_H
    return range_lidar


def extractHead(ts_lidar,joint):
    ts_joint_raw = joint['ts'].T
    # ts_lidar0 = lidar1['t'][0, 0]
    ind_joint = findClosestJointTime(ts_joint_raw, ts_lidar)
    head_angles = joint['head_angles'].T[ind_joint]
    return ind_joint,head_angles

def uvd2xyz(u,v,d,fc):
    y = d / 1000
    x = u / fc[1]* y
    z = -v / fc[2]* y

    return np.stack((x,y,z),axis=-1)


