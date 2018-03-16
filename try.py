from helper import *
from utils import *

folder='./data/'
lidar_file='train_lidar0'
joint_file='train_joint0'




if __name__=='__main__':
    jointData, lidarData = getData(jointPath=folder + joint_file, lidarPath=folder + lidar_file)
    n_lidar=len(lidarData)




    for i in range(n_lidar):
        lidarData_current, lidarData_0=lidarData[i],lidarData[0]
        lidarData_previous=lidarData[i-1]if i>0 else lidarData[i]

        # compute Tranformation
        T_H_B, T_B_G, T_H_G, ind_joint = calT(jointData, lidarData_current, lidarData_previous, lidarData_0)

        #correct lidar pose
        pose_odo_new = getRelOdometry(lidarData_current, lidarData_previous, T_H_B, T_B_G)  # (n, 2), list, list

        #correct range
        range_pts_G, angles, valid = correctRange(lidarData_current, T_H_G)

        #mapping
        mapping(range_pts_G, angles)

