from helper import *
from slam import slam

folder='./data/'
lidar_file='train_lidar0'
joint_file='train_joint0'




if __name__=='__main__':
    jointData, lidarData = getData(jointPath=folder + joint_file, lidarPath=folder + lidar_file)
    n_lidar=len(lidarData)
    for i in range(n_lidar):
        slam(jointData,lidarData[i],lidarData[i-1],lidarData[0]) if i>0 else slam(jointData,lidarData[i],lidarData[i],lidarData[0])

