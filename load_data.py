"""""""""""""""""""""""""""""""""""""""""""""

Author: Heejin Chloe Jeong (heejinj@seas.upenn.edu)
Affiliation: University of Pennsylvania
Date: Feb 2017

DESCRIPTION
: In this file, you can load .mat file data in python dictionary format.
  The output of the "get_lidar" function is an array with dictionary elements. The length of the array is the length of data. 
  The output of the "get_joint" function is a dictionary with eight different data (read data description for details). Each dictionary is an array with the same length.
  The output of the "get_rgb" function is an array with dictionary elements. The length of the array is the length of data.
  The output of the "get_depth" function is an array with dictionary elements. The length of the array is the lenght of data.

"""""""""""""""""""""""""""""""""""""""""""""

import pickle
from scipy import io
import numpy as np

def get_lidar(file_name):
    data = io.loadmat(file_name+".mat")
    lidar = []
    for m in data['lidar'][0]:
        x = {}
        x['t']= m[0][0][0]
        n = len(m[0][0])
        if (n != 5) and (n != 6):
            raise ValueError("different length!")
        x['pose'] = m[0][0][n-4]
        x['res'] = m[0][0][n-3]
        x['rpy'] = m[0][0][n-2]
        x['scan'] = m[0][0][n-1]

        lidar.append(x)
    return lidar

def get_joint(file_name):
    key_names_joint = ['acc', 'ts', 'rpy', 'gyro', 'pos', 'ft_l', 'ft_r', 'head_angles']
    data = io.loadmat(file_name+".mat")
    joint = {kn: data[kn] for kn in key_names_joint}
    return joint


def get_rgb_dict(file_name):
    key_names_rgb = ['t', 'width', 'imu_rpy', 'id', 'odom', 'head_angles', 'c', 'sz', 'vel', 'rsz', 'body_height', 'tr',
                     'bpp', 'name', 'height', 'image']
    # image size: 1080x1920x3 uint8
    data = io.loadmat(file_name + ".mat")
    data = data['RGB'][0]
    rgb = []
    for m in data:
        tmp = {v: m[0][0][i] for (i, v) in enumerate(key_names_rgb)}
        rgb.append(tmp)

    nec_key=['t','image']
    rgb_dict={}
    for n in nec_key:
        rgb_dict[n]=[]
        for i in range(len(rgb)):
            rgb_dict[n].append(rgb[i][n])
    return rgb_dict


def get_rgb_target(file_name,target):
    key_names_rgb = ['t', 'width', 'imu_rpy', 'id', 'odom', 'head_angles', 'c', 'sz', 'vel', 'rsz', 'body_height', 'tr',
                     'bpp', 'name', 'height', 'image']
    # image size: 1080x1920x3 uint8
    data = io.loadmat(file_name + ".mat")
    data = data['RGB'][0]
    rgb = []
    for m in data:
        for (i, v) in enumerate(key_names_rgb):
            if v =='t':
                tmp = {v: m[0][0][i]}
                rgb.append(tmp)

    nec_key=[target]
    # rgb_ts={}
    for n in nec_key:
#         rgb_ts[n]=[]
        rgb_ts=np.empty((1,))
        for i in range(len(rgb)):
            rgb_ts=np.append(rgb_ts,rgb[i][n][0],axis=0)
#             rgb_ts[n].append(rgb[i][n][0])
    return rgb_ts[1:]

def get_depth_dict(file_name):
    key_names_depth = ['t','width','imu_rpy','id','odom','head_angles','c','sz','vel','rsz','body_height','tr','bpp','name','height','depth']
    data = io.loadmat(file_name+".mat")
    data = data['DEPTH'][0]
    depth = []
    for m in data:
        tmp = {v:m[0][0][i] for (i,v) in enumerate(key_names_depth)}
        depth.append(tmp)

    nec_key = ['t','depth']
    depth_dict={}
    for n in nec_key:
        depth_dict[n] = []
        # depth_ts = np.empty((1,))
        for i in range(len(depth)):
            depth_dict[n].append(depth[i][n])
    #             rgb_ts[n].append(rgb[i][n][0])

    return depth_dict
    # return depth


def get_depth_target(file_name,target):
    key_names_depth = ['t', 'width', 'imu_rpy', 'id', 'odom', 'head_angles', 'c', 'sz', 'vel', 'rsz', 'body_height',
                       'tr', 'bpp', 'name', 'height', 'depth']
    data = io.loadmat(file_name + ".mat")
    data = data['DEPTH'][0]
    depth = []
    for m in data:
        for (i, v) in enumerate(key_names_depth):
            if v == 't':
                tmp = {v: m[0][0][i]}
                depth.append(tmp)

    nec_key = [target]

    for n in nec_key:
        #         rgb_ts[n]=[]
        depth_ts = np.empty((1,))
        for i in range(len(depth)):
#             print('depth_ts:')
#             print(depth_ts)
#             print(depth[i][n][0])
            depth_ts = np.append(depth_ts, depth[i][n][:,0], axis=0)
#             rgb_ts[n].append(rgb[i][n][0])
    return depth_ts


