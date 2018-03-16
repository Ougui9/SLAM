from helper import *
from utils import *
from mapping import *
from localizeRob import *

folder='./data/'
lidar_file='train_lidar0'
joint_file='train_joint0'

logodd_thre=500
p11 = 0.65# P(occupied|measured occupied);
p00 = 0.7# P(free|measured free);


if __name__=='__main__':
    jointData, lidarData = getData(jointPath=folder + joint_file, lidarPath=folder + lidar_file)
    n_lidar=len(lidarData)

    lidarData_0=lidarData[0]

    #x0, y0, yaw0 best particle
    x0 = lidarData_0['pose'][0,0]
    y0 = lidarData_0['pose'][0,1]
    yaw0 = lidarData_0['pose'][0,2]


    particles=ini_particles(x0,y0,yaw0,n_sample)
    MAP = iniMap(res, xmin, xmax, ymin, ymax)
    logodd=iniLogOdd(MAP['sizex'],MAP['sizey'],logodd_thre,p11,p00)


    # sx, sy, syaw=sxyyaw[:,0], sxyyaw[:,1], sxyyaw[:,2]
    # sweights = np.ones([n_sample, 1]) / n_sample

    for i in range(n_lidar):
        lidarData_current=lidarData[i]
        lidarData_previous=lidarData[i-1]if i>0 else lidarData[i]

        # compute Tranformation
        T_H_B, T_B_G, T_H_G, ind_joint = calT(jointData, lidarData_current, lidarData_previous, lidarData_0)

        #correct lidar pose
        pose_odo_new = getRelOdometry(lidarData_current, lidarData_previous, T_H_B, T_B_G)  # (n, 2), list, list

        #correct range
        range_G, angles, valid_pro,valid_cor= correctRange(lidarData_current, T_H_G)#unfiltered



        #mapping
        MAP=mapping(range_G, angles,valid_pro,MAP,logodd)

