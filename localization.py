import numpy as np
from helper import smartMinus,smartPlus
from MapUtils.MapUtils import *

n_sample=100
process_var = np.array([1,1,5])*1e-5

def ini_particles(x0,y0,yaw0,n_sample):
    particles = {}
    sxyyaw = np.random.normal([x0, y0, yaw0], [0, 0, 0], (n_sample, 3))
    particles['sx'] = sxyyaw[:, 0]
    particles['sy'] = sxyyaw[:, 1]
    particles['syaw'] = sxyyaw[:, 2]
    particles['sweight'] = np.ones([n_sample, 1]) / n_sample
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
        ppose1[:,i] = smartPlus(ppose0[:,i].reshape(-1,1), smartMinus(pose_cur, pose_pre))
    


    particles['sx'] = ppose1[0]
    particles['sy'] = ppose1[1]
    particles['syaw'] = ppose1[2]
    # print(1)

    return particles

def localizationUpdate(particles,range_raw,MAP,T_h_b):

    return







def getMapCorr(MAP,range_G,valid_c):
    # x_im=MAP['xcell_phy']
    # len(particles['sx']
    cs = np.zeros([n_sample, 1])
    for i in range(n_sample):
        c=mapCorrelation(MAP['map'],x_im=MAP['xcell_phy'],y_im=MAP['ycell_phy'],vp=,xs=,ys=)
        cs[i]=np.linalg.norm(c)
    return cs

def update_sweight(particles,cs):
    particles['sweight']*=cs
    particles['sweight']/=np.sum(particles['sweight'])
    return particles

def chooseBestParticle(sweight):
    return np.argmax(sweight,axis=0)