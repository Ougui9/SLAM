import numpy as np
from helper import smartMinus,smartPlus,Rot,scan2World,mapCorrelation2
from MapUtils.MapUtils import *
from utils import cal_T_b_g,rangeH2rangeG
from mapping import angles

n_sample=150
# process_var = np.array([1,1,5])*1e-5

# w=np
W = np.array([[1E-5, 0, 0],
              [0, 1E-5, 0],
              [0, 0, 5E-5]])

# W = np.array([[4E-4, 0, 0],
#               [0, 4E-4, 0],
#               [0, 0, 6E-2]])



def ini_particles(x0,y0,yaw0,n_sample):
    particles = {}
    sxyyaw = np.random.multivariate_normal(mean=[x0,y0,yaw0],cov=np.zeros([3,3]),size=n_sample)
    particles['sx'] = sxyyaw[:, 0]
    particles['sy'] = sxyyaw[:, 1]
    particles['syaw'] = sxyyaw[:, 2]
    particles['sweight'] = np.ones([n_sample]) / n_sample
    return particles
def localizationPrediction(particles,pose_cur, pose_pre):
    '''
    :param particles:{}: sx(n,), sy(n,), syaw(n,), sweight(n,1)
    :param pose_cur:(3,1)
    :param pose_pre:(3,1)
    :return:
    '''
    ppose0=np.zeros([3,n_sample])
    ppose0[0]=particles['sx']
    ppose0[1] = particles['sy']
    ppose0[2] = particles['syaw']
    ppose1=np.zeros_like(ppose0)
    w_xy = np.random.multivariate_normal(mean=np.zeros(2), cov=W[:2,:2], size=n_sample)
    # w_x=np.random.normal(0,W[0,0],(n_sample,1))
    # w_y = np.random.normal(0, W[1, 1], (n_sample, 1))
    w_yaw=np.random.normal(0,W[-1,-1],(n_sample,1))
    # w=np.concatenate((np.concatenate((w_x,w_y),axis=1),w_yaw),axis=1)
    w = np.concatenate((w_xy, w_yaw), axis=1)

    dxdy_gobal_raw = pose_cur[:2] - pose_pre[:2]
    # dxdy_local = Rot(pose_pre[-1,0]).T.dot(dxdy_gobal_raw)
    # dyaw = pose_cur[-1] - pose_pre[-1]
    # ppose1[-1] =ppose0[-1]+dyaw+w[:,-1]

    for i in range(n_sample):

        ppose1[:,i] = smartPlus(smartPlus(ppose0[:,i].reshape(-1,1), smartMinus(pose_cur, pose_pre)),w[i].reshape(-1,1))[:,0]


        # ppose1[:2,i]=ppose0[:2,i]+Rot(ppose1[-1,i]).dot(dxdy_local)[:,0]

    particles['sx'] = ppose1[0]
    particles['sy'] = ppose1[1]
    # particles['sx'] = ppose1[0]+w[:,0]
    # particles['sy'] = ppose1[1]+w[:,1]
    particles['syaw'] = ppose1[2]
    # print(1)

    return particles

def localizationUpdate(particles,MAP,rpy,range_xyz_lidar,head_angles,inValid_c):
    #get T_b_g for each particles
    x=particles['sx']
    y = particles['sy']

    # n_range=len(range_raw)
    # range_H_4=np.ones([4,n_range])
    # range_H_4[:3] = rangeRaw2rangeH(range_raw, angles).T  # (3, n)
    # indValid_cf = np.logical_and((range_raw < 10), (range_raw > 0.1))
    # cs=np.zeros(n_sample)
    range_xyz_lidar=range_xyz_lidar.copy()[inValid_c]
    #(3,n_ranges,n_samples)
    range_xyz_G=scan2World(lidar_xyz=range_xyz_lidar.T,neck_angle=head_angles[0],head_angle=head_angles[1], Particles=particles,T_gen=np.tile(np.identity(4).reshape(4, 4, 1), reps=(1, 1, n_sample)))



    # for i in range(n_sample):
    # #     T_b_g=cal_T_b_g(x[i],y[i],rpy)
    # #     range_G=T_b_g.dot(T_h_b).dot(range_H_4)[:3] #range2raw (n, 3)
    # #     # filter out valid index
    # #     indValid_ground = range_G[-1] > 0.1
    # #     indValid = np.logical_and(indValid_cf, indValid_ground)
    # #     range_G = range_G[:,indValid]
    # #
    # #     # convert phy corrds to map corr
    # #     # xrange_map = np.ceil((range_G[:, 0] - MAP['xmin']) / MAP['res']).astype(np.int16) - 1
    # #     # yrange_map = np.ceil((range_G[:, 1] - MAP['ymin']) / MAP['res']).astype(np.int16) - 1
    #     xrange_map = np.arange(-0.2, 0.2 + MAP['res'], MAP['res'])
    #     yrange_map = np.arange(-0.2, 0.2 + MAP['res'], MAP['res'])
    #     Y = np.concatenate([np.concatenate([range_xyz_G[0,:,i].reshape(1,-1), range_xyz_G[1,:,i].reshape(1,-1)], axis=0), np.zeros([1,len(range_xyz_G[0,:,i])])], axis=0)
    #     c=mapCorrelation(MAP['map'],MAP['xcell_phy'],MAP['ycell_phy'],Y[:3],xrange_map,yrange_map)
    #     cs[i]=np.linalg.norm(c)
        # cs[i] = np.max(c)
    cs=mapCorrelation2(range_xyz_G,MAP)
    particles['sweight']*=cs
    sum_sw=np.sum(particles['sweight'])
    if sum_sw==0:
        particles['sweight']=np.ones([n_sample]) / n_sample
    else:
        particles['sweight']/=np.sum(particles['sweight'])
    # particles['sweight'][np.isnan(particles['sweight'])]=np.ones([n_sample]) / n_sample


    #check Neff
    Neff=np.sum(particles['sweight'])/(particles['sweight'].dot(particles['sweight']))

    if Neff<0.7*n_sample:
        print("rasampling")
        ind_resample=resample(particles['sweight'])
        particles['sx']=particles['sx'][ind_resample]
        particles['sy'] = particles['sy'][ind_resample]
        particles['syaw'] = particles['syaw'][ind_resample]
        particles['sweight'] = particles['sweight'][ind_resample]
        particles['sweight']/=np.sum(particles['sweight'])


    return particles

def resample(weights):
    n_sample=len(weights)
    wsum=np.cumsum(weights)
    wsum/=wsum[-1]

    r=np.random.rand(n_sample)
    indsort=np.argsort(np.concatenate([wsum,r]))

    s=np.concatenate([np.ones(n_sample),np.zeros(n_sample)])
    s=s[indsort]
    ssum=np.cumsum(s)
    index=ssum[s==0]
    return index.astype(int)


def chooseBestParticle(sweight):
    return np.argmax(sweight,axis=0)