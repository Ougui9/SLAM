from helper import *
from utils import *
from mapping import *
from localization import *
# from MapUtils.MapUtils import *

folder='./data/'
lidar_file='train_lidar0'
joint_file='train_joint0'

logodd_thre=500
p11 = 0.65# P(occupied|measured occupied);
p00 = 0.7# P(free|measured free);


# def dead_reckon(o0,o1):
#
#
#     return p_cur
def SLAM(particles,range_raw, MAP,logodd,T_h_b, pose0,pose1, rpy_unbiased):
    ind_bestparticles=chooseBestParticle(particles['sweight'])

    p_best=np.array([particles['sx'][ind_bestparticles],particles['sy'][ind_bestparticles],particles['syaw'][ind_bestparticles]])



    MAP,logodd=mapping(range_raw,p_best,T_h_b,MAP,logodd,rpy_unbiased)

    particles=localizationPrediction(particles,pose0,pose1)


    particles=localizationUpdate(particles,range_raw,MAP,T_h_b)

    return particles, MAP,logodd






if __name__=='__main__':
    jointData, lidarData = getData(jointPath=folder + joint_file, lidarPath=folder + lidar_file)
    n_lidar=len(lidarData)

    lidarData_0=lidarData[0]

    #x0, y0, yaw0 best particle
    pose_odo_pre =lidarData_0['pose'][0]
    x0 = pose_odo_pre[0]
    y0 = pose_odo_pre[1]
    yaw0 = pose_odo_pre[2]


    particles=ini_particles(x0,y0,yaw0,n_sample)
    MAP = iniMap(res, xmin, xmax, ymin, ymax)
    logodd=iniLogOdd(MAP['sizex'],MAP['sizey'],logodd_thre,p11,p00)


    # sx, sy, syaw=sxyyaw[:,0], sxyyaw[:,1], sxyyaw[:,2]
    # sweights = np.ones([n_sample, 1]) / n_sample
    for i in range(0,n_lidar):
        # lidarData_current=lidarData[i]
        # lidarData_previous=lidarData[i-1]if i>0 else lidarData[i]
        #
        # # compute Tranformation
        # T_H_B, T_B_G, T_H_G, ind_joint = calT(jointData, lidarData_current, lidarData_previous, lidarData_0)
        #
        # #correct lidar pose
        # pose_odo_new = getRelOdometry(lidarData_current, lidarData_previous, T_H_B, T_B_G)  # (n, 2), list, list
        #
        # #correct range
        # range_G, valid_pro,valid_cor= correctRange(lidarData_current['scan'][0], T_H_G)#unfiltered
        #
        #
        #
        # #mapping
        # MAP,logodd=mapping(range_G,valid_pro,pose_odo_new,MAP,logodd)
        #
        # #localization
        #
        # ##loca prediciton, update particles location
        # particles=locPrediction(particles,pose_odo_new.reshape(-1,1), pose_odo_pre.reshape(-1,1))
        #
        # ##get map correlation matrix
        # cs = mapCorrelation(MAP[',)
        #
        # ##update sample weights
        # particles=update_sweight(particles,cs)


        #target for this loop is 1
        lidar1 = lidarData[i]
        lidar0=lidarData[i-1]if i>0 else lidarData[i]
        #define pose0 pose1
        pose0=lidar0['pose'][0] #if i>0 else lidarData[i]['pose'][0]
        pose1 = lidar1['pose'][0]
        rpy_unbiased=lidarData[i]['rpy'][0]-lidarData_0['rpy'][0]


        #cal T_h_b
        T_h_b=calT_h_b(lidar1[i]['t'][0,0],jointData)


        particles, MAP, logodd=SLAM(particles,lidarData[i]['scan'][0], MAP, logodd,T_h_b, pose0,pose1,rpy_unbiased)




