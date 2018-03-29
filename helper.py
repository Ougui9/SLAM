'''
File: Helper functions for SLAM Proj
Author: Yilun Zhang
Date:

'''
from load_data import *
import numpy as np
import matplotlib.pyplot as plt
# from utils import



def findClosestJointTime(ts_joint_raw,ts_lidar_cur):
    '''
    :param ts_joint_raw: (n_joint, 1)
    :param ts_lidar_cur: number
    :return:
    '''
    n_joint,_=ts_joint_raw.shape

    diff=abs(ts_joint_raw-ts_lidar_cur)
    ind_joint_new=np.argmin(diff)
    return ind_joint_new


def rpy2rot(r,p,y):
    rot=np.zeros([3,3])
    a = y
    b = p
    c = r
    rot[0,0]=np.multiply(np.cos(a),np.cos(b))
    rot[0, 1] =np.multiply(np.multiply(np.cos(a),np.sin(b)),np.sin(c))-np.multiply(np.sin(a),np.cos(c))
    rot[0, 2] =np.multiply(np.multiply(np.cos(a),np.sin(b)),np.cos(c))+np.multiply(np.sin(a),np.sin(c))
    rot[1, 0] =np.multiply(np.sin(a),np.cos(b))
    rot[1, 1] =np.multiply(np.multiply(np.sin(a),np.sin(b)),np.sin(c))+np.multiply(np.cos(a),np.cos(c))
    rot[1, 2] =np.multiply(np.multiply(np.sin(a),np.sin(b)),np.cos(c))-np.multiply(np.cos(a),np.sin(c))
    rot[2, 0] =-np.sin(b)
    rot[2, 1] =np.multiply(np.cos(b),np.sin(c))
    rot[2, 2] =np.multiply(np.cos(b),np.cos(c))
    return rot

def rot2rpy(rot):
    '''
    :param rot: (3, 3)
    :return: (3,)
    '''
    rpy=np.zeros(3)
    rpy[2]=np.arctan2(rot[1,0],rot[0,0])
    rpy[1] = np.arctan2(-rot[2, 0], np.sqrt(rot[2, 1]**2+rot[2,2]**2))
    rpy[0] = np.arctan2(rot[2, 1], rot[2, 2])
    return rpy

# def visualize2D(pose):
#     # ax=plt.figure(0)
#     # plt.hold(True)
#     plt.xticks(np.linspace(-0.1, 3, 100, endpoint=True))
#     plt.yticks(np.linspace(-0.1, 2, 100, endpoint=True))
#     for i in range(1000,len(pose)):
#         plt.scatter(pose[i,0],pose[i,1])
#         plt.draw()
#         plt.pause(0.001)

# def embedOdoNew(pose_new,lidarData):
#     n_lidar = len(lidarData)
#     for i in range(n_lidar):
#         lidarData[i]['pose'][0,-1]=pose_new[i,-1]
#
#     return lidarData

def getData(jointPath, lidarPath):

    #get joint data
    jointData = get_joint(jointPath)

    #get lidar data
    lidarData=get_lidar(lidarPath)

    return jointData,lidarData

def sigmoid(x,a,c):
  return 1 / (1 + np.exp(-a*(x-c)))

def smartPlus(x1,x0):
    '''
    :param x1: (3,1)
    :param x0: (3,1)
    :return: (3,1)
    '''

    res=np.zeros([3,1])
    R0=Rot(x0[-1,0])
    res[:2]=x0[:2]+R0.dot(x1[:2])
    res[-1]=x0[-1]+x1[-1]

    return res

def smartMinus(x1,x0):
    '''
        :param x1: (3,1)
        :param x0: (3,1)
        :return: (3,1)
        '''
    res = np.zeros([3, 1])
    R0 = Rot(x0[-1,0])
    res[:2] = R0.T.dot((x1[:2]-x0[:2]).reshape(-1,1))
    res[-1] = x1[-1] - x0[-1]

    return res


def Rot(theta):
    R=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    return R

def scan2World(lidar_xyz, neck_angle, head_angle, pose=None, Particles=None, T_gen=None):
#this transform is utilized from https://github.com/ryanoldja/ese650/blob/master/Humanoid_SLAM/p3_util.py
    if pose is not None:

        # use absolute (x, y) position in pose to translate from world frame to body frame,
        # translate 93cm along z-axis from ground plane to center of mass,
        # then translate 33cm along z-axis from center of mass to head
        x = pose[0]
        y = pose[1]
        z1 = 0.93 + 0.33  # [m]
        Txyz = np.array([[1, 0, 0, x],
                         [0, 1, 0, y],
                         [0, 0, 1, z1],
                         [0, 0, 0, 1]])

        # use absolute yaw angle in pose to rotate into alignment with the lidar frame along the z-axis
        body_yaw = pose[2]
        total_yaw = body_yaw + neck_angle
        Rz = np.array([[np.cos(total_yaw), -np.sin(total_yaw), 0, 0],
                       [np.sin(total_yaw), np.cos(total_yaw), 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])

        # rotate by head pitch angle relative to body along y-axis
        Ry = np.array([[np.cos(head_angle), 0, np.sin(head_angle), 0],
                       [0, 1, 0, 0],
                       [-np.sin(head_angle), 0, np.cos(head_angle), 0],
                       [0, 0, 0, 1]])

        # translate 15cm along z-axis from head to lidar frame
        z2 = 0.15  # [m]
        Tz2 = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, z2],
                        [0, 0, 0, 1]])

        # compute composite transform
        T = Txyz.dot(Rz).dot(Ry).dot(Tz2)

        # convert lidar_xyz to homogeneous representation
        _, n = lidar_xyz.shape
        lidar_xyz1 = np.vstack((lidar_xyz, np.ones((1, n))))

        # apply composite transform to cartesian coordinates in lidar frame
        world_xyz = T.dot(lidar_xyz1)

        return world_xyz[:3, :]

    else:

        _, n_beams = lidar_xyz.shape

        # create xyz-translation transform for each particle
        # making world frame origin coincident with head frame origin
        Txyz = T_gen.copy()
        Txyz[0, 3, :] = Particles['sx']
        Txyz[1, 3, :] = Particles['sy']
        Txyz[2, 3, :] = 0.93 + 0.33  # [m]

        # create z-rotation (yaw) transform for each particle
        # making world frame x- and y- axes co-linear with lidar frame
        body_yaw = Particles['syaw']
        total_yaw = body_yaw + neck_angle
        Rz = T_gen.copy()
        Rz[[0, 1], [0, 1], :] = np.cos(total_yaw)
        Rz[0, 1, :] = -np.sin(total_yaw)
        Rz[1, 0, :] = np.sin(total_yaw)

        # create y-rotation (pitch) transform for each particle
        # making world frame z-axis colinear with lidar frame
        Ry = T_gen.copy()
        Ry[[0, 2], [0, 2], :] = np.cos(head_angle)
        Ry[0, 2, :] = np.sin(head_angle)
        Ry[2, 0, :] = -np.sin(head_angle)

        # create z-translation transform for each particle
        # making world frame origin coincident with lidar frame origin
        Tz = T_gen.copy()
        Tz[2, 3, :] = 0.15  # [m]

        # compute composite transform
        T1 = np.einsum('ijk,jlk->ilk', Txyz, Rz)
        T2 = np.einsum('ijk,jlk->ilk', T1, Ry)
        T = np.einsum('ijk,jlk->ilk', T2, Tz)

        # convert lidar_xyz to homogenous representation
        _, n = lidar_xyz.shape
        lidar_xyz1 = np.vstack((lidar_xyz, np.ones((1, n))))

        # apply composite transform to cartesian coordinates in lidar frame
        world_xyz = np.einsum('ijk,jl->ilk', T, lidar_xyz1)
        return world_xyz[:3]

def mapCorrelation2(range_xyz_G, Map):
    xis = np.ceil((range_xyz_G[0] - Map['xmin']) / Map['res']).astype(np.int16) - 1
    yis = np.ceil((range_xyz_G[1] - Map['ymin']) / Map['res']).astype(np.int16) - 1
    # indGood = np.logical_and(np.logical_and(np.logical_and((xis > 1), (yis > 1)), (xis < Map['sizex'])),
    #                          (yis < Map['sizey']))
    ji=np.concatenate((xis[np.newaxis,:,:],yis[np.newaxis,:,:]),axis=0)
    # ji = xy2map(xy=range_xyz_G[:2], Map=Map)
    # ji=np.array([[],[]])

    # filter out hits outside the map array
    # ji[ji >= ((Map['xmax']-Map['xmin']) * Map['res'])] = ((Map['xmax']-Map['xmin']) * Map['res']) - 1

    # compute particle correlations
    c = np.sum(Map['map'][ji[0], ji[1]], axis=0)
    return c
# #
# def mapCorrelation2(range_xyz_G, Map):
#     '''
#     Compute particle lidar scan correlations to current binary map.
#
#     Input Particles: dict, particles container
#     Input Map: dict, map container
#     '''
#
#     # convert particle xy-hits to pixel space
#     ji = xy2map(xy=range_xyz_G[:2], Map=Map)
#
#     # filter out hits outside the map array
#     ji[ji >= ((Map['xmax'] - Map['xmin']) * 1/Map['res'])] = ((Map['xmax'] - Map['xmin']) * 1/Map['res']) - 1
#
#     # compute particle correlations
#     c = np.linalg.norm(Map['map'][ji[1], ji[0]], axis=0)
#     return c



def xy2map(xy, Map):
    xy_ = xy.copy()
    xy_[1] *= -1
    ji = (1/Map['res'] * (xy_ + (Map['xmax']-Map['xmin'])/ 2)).astype(np.uint64)  # (x=column, y=row)
    return ji
def uvd2xyz(u, v, d, fc):
    y = d / 1000
    x = u / fc[0]* y
    z = -v / fc[1]* y

    xyz = np.concatenate((x,y,z),axis=-1)
    return xyz

def xyz2uvd(x, y, z, fc):
    d = y * 1000
    u = x/ y * fc[0]
    v = -z/ y * fc[1]

    uvd = np.concatenate((u, v, d), axis=-1)
    return uvd