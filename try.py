from helper import *
from utils import *
from mapping import *
from localization import *
import pickle
# from MapUtils.MapUtils import *

folder='./data/'
NoFile=2
lidar_file='train_lidar'+str(NoFile)
joint_file='train_joint'+str(NoFile)

startInd=2000
interval=1
mappingInterval=50

logodd_thre=500
p11 = 0.65# P(occupied|measured occupied);
p00 = 0.7# P(free|measured free);

# define noise covariance


def SLAM(particles,range_raw, MAP,logodd,T_h_b, pose0,pose1, rpy_unbiased,i):
    ind_bestparticles=chooseBestParticle(particles['sweight'])

    p_best=np.array([particles['sx'][ind_bestparticles],particles['sy'][ind_bestparticles],particles['syaw'][ind_bestparticles]])


    if np.mod(i,mappingInterval)==0:

        MAP,logodd=mapping(range_raw,p_best,T_h_b,MAP,logodd,rpy_unbiased)


    particles=localizationPrediction(particles,pose1.reshape(-1,1),pose0.reshape(-1,1))


    particles=localizationUpdate(particles,range_raw,MAP,T_h_b,rpy_unbiased)
    # particles = localizationPrediction(particles, pose1.reshape(-1, 1), pose0.reshape(-1, 1))


    return particles, MAP,logodd,p_best






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

    # plt.figure()
    plt.ion()
    fig, ax = plt.subplots()

    for i in range(startInd,n_lidar,interval):


        #target for this loop is 1
        lidar1 = lidarData[i]
        lidar0=lidarData[i-interval]if i>0 else lidarData[i]
        #define pose0 pose1
        pose0=lidar0['pose'][0] #if i>0 else lidarData[i]['pose'][0]
        pose1 = lidar1['pose'][0]
        rpy_unbiased=lidarData[i]['rpy'][0]-lidarData_0['rpy'][0]


        #cal T_h_b
        T_h_b=calT_h_b(lidar1['t'][0,0],jointData)


        particles, MAP, logodd,p_best=SLAM(particles,lidarData[i]['scan'][0], MAP, logodd,T_h_b, pose0,pose1,rpy_unbiased,i)




        #visualize

        if np.mod(i,mappingInterval)==0 or i>n_lidar-2:
            print(i)
            # ax.imshow(sigmoid(MAP['map'],1,0))
            ax.imshow(MAP['binary'])
            x_map = np.ceil((p_best[0] - MAP['xmin']) / MAP['res']).astype(np.int16) - 1
            y_map = np.ceil((p_best[1] - MAP['ymin']) / MAP['res']).astype(np.int16) - 1
            ax.arrow(x=y_map,y=x_map,dx=100*np.sin(p_best[-1]),dy=100*np.cos(p_best[-1]),head_width=30, head_length=20,color='red')
            x_p=np.ceil((particles['sx'] - MAP['xmin']) / MAP['res']).astype(np.int16) - 1
            y_p=np.ceil((particles['sy'] - MAP['xmin']) / MAP['res']).astype(np.int16) - 1
            ax.scatter(y_p,x_p)
            plt.title('Dataset No. %d\n%d/%d'%(NoFile,i+1,n_lidar))
            # ax.annotate("", xy=(0.5, 0.5), xytext=(0, 0),rrowprops = dict(arrowstyle="->"))

            plt.pause(1)
            plt.draw()
            if i>n_lidar-2:
                plt.savefig(lidar_file + '.jpg')
            ax.cla()
    with open(lidar_file+'MAP.pickle', 'wb') as handle:
        pickle.dump(MAP, handle, protocol=pickle.HIGHEST_PROTOCOL)
    with open(lidar_file+'logodd.pickle', 'wb') as handle:
        pickle.dump(logodd, handle, protocol=pickle.HIGHEST_PROTOCOL)
    # plt.savefig(lidar_file+'.jpg')



