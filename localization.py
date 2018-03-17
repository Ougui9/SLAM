import numpy as np
from helper import smartMinus,smartPlus
from MapUtils.MapUtils import *
from utils import cal_T_b_g,rangeRaw2rangeH,rangeH2rangeG
from mapping import angles

n_sample=100
process_var = np.array([1,1,5])*1e-5

def ini_particles(x0,y0,yaw0,n_sample):
    particles = {}
    sxyyaw = np.random.normal([x0, y0, yaw0], [0, 0, 0], (n_sample, 3))
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
    for i in range(n_sample):
        ppose1[:,i] = smartPlus(ppose0[:,i].reshape(-1,1), smartMinus(pose_cur, pose_pre))[:,0]
    


    particles['sx'] = ppose1[0]
    particles['sy'] = ppose1[1]
    particles['syaw'] = ppose1[2]
    # print(1)

    return particles

def localizationUpdate(particles,range_raw,MAP,T_h_b,rpy):
    #get T_b_g for each particles
    x=particles['sx']
    y = particles['sy']

    n_range=len(range_raw)
    range_H_4=np.ones([4,n_range])
    range_H_4[:3] = rangeRaw2rangeH(range_raw, angles).T  # (3, n)
    indValid_cf = np.logical_and((range_raw < 10), (range_raw > 0.1))
    cs=np.zeros(n_sample)
    for i in range(n_sample):
        T_b_g=cal_T_b_g(x[i],y[i],rpy)
        range_G=T_b_g.dot(T_h_b).dot(range_H_4)[:3] #range2raw (n, 3)
        # filter out valid index
        indValid_ground = range_G[-1] > 0.1
        indValid = np.logical_and(indValid_cf, indValid_ground)
        range_G = range_G[:,indValid]

        # convert phy corrds to map corr
        xrange_map = np.ceil((range_G[:, 0] - MAP['xmin']) / MAP['res']).astype(np.int16) - 1
        yrange_map = np.ceil((range_G[:, 1] - MAP['ymin']) / MAP['res']).astype(np.int16) - 1
        Y = np.concatenate([np.concatenate([range_G[0].reshape(1,-1), range_G[1].reshape(1,-1)], axis=0), np.zeros([1,len(range_G[0])])], axis=0)
        c=mapCorrelation(MAP['map'],MAP['xcell_phy'],MAP['ycell_phy'],Y[:3],xrange_map,yrange_map)
        cs[i]=np.linalg.norm(c)
    particles['sweight']*=cs
    particles['sweight']/=np.sum(particles['sweight'])
    particles['sweight'][np.isnan(particles['sweight'])]=np.ones([n_sample]) / n_sample


    #check Neff
    Neff=np.sum(particles['sweight'])/(particles['sweight'].dot(particles['sweight']))

    if Neff<0.7*n_sample:
        resample(particles['sweight'], n_sample)









    return

def resample(weights,n_sample):

    wsum=np.cumsum(weights)
    wsum/=wsum[-1]

    np.random.rand(1,n_sample)
    return index





# def getMapCorr(MAP,range_G,valid_c):
#     # x_im=MAP['xcell_phy']
#     # len(particles['sx']
#     cs = np.zeros([n_sample, 1])
#     for i in range(n_sample):
#         c=mapCorrelation(MAP['map'],x_im=MAP['xcell_phy'],y_im=MAP['ycell_phy'],vp=,xs=,ys=)
#         cs[i]=np.linalg.norm(c)
#     return cs

# def update_sweight(particles,cs):
#     particles['sweight']*=cs
#     particles['sweight']/=np.sum(particles['sweight'])
#     return particles

def chooseBestParticle(sweight):
    return np.argmax(sweight,axis=0)