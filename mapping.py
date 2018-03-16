import numpy as np
from MapUtils.MapUtils import getMapCellsFromRay
import matplotlib.pyplot as plt
from helper import sigmoid


# init MAP

res=0.05
xmin=-20
xmax=20
ymin=-20
ymax=20
occFactor=1/9


def iniLogOdd(sizex,sizey,thres,p11,p00):
    logodd={}
    logodd['odd']=np.zeros([sizex, sizey])
    logodd['p11'] =p11# P(occupied|measured occupied);(z=1,m=1)
    logodd['p01'] = 1-p11   # P(free|measured occupied)
    logodd['p00']= p00 # P(free | measured free);
    logodd['p10'] = 1 - p00 # P(occupied | measured free);
    logodd['logodd_occ'] = np.log(logodd.p11 / logodd.p10); # > 0
    logodd['logodd_free'] = np.log(logodd.p01 / logodd.p00); # < 0
    logodd['logodd_max'] = np.log(thres)
    logodd['logodd_min'] = -np.log(thres)
    logodd[thres] = thres
    return logodd
# class logodd



def iniMap(res,xmin,xmax,ymin,ymax):
    MAP = {}
    MAP['res']   = 0.05 #meters
    MAP['xmin']  = -20  #meters
    MAP['ymin']  = -20
    MAP['xmax']  =  20
    MAP['ymax']  =  20
    MAP['sizex']  = int(np.ceil((MAP['xmax'] - MAP['xmin']) / MAP['res'] + 1)) #cells
    MAP['sizey']  = int(np.ceil((MAP['ymax'] - MAP['ymin']) / MAP['res'] + 1))

    MAP['map'] = np.zeros((MAP['sizex'],MAP['sizey']),dtype=np.int8) #DATA TYPE: char or
    # MAP['binary']=MAP['odd'].copy()
    # MAP['occFactor']=1/9
    return MAP


def updateMAP_logodd(MAP,logodd,xcell_free,ycell_free,xcell_hit,ycell_hit):
    logodd['odd'][xcell_free, ycell_free] += logodd['logodd_free']
    logodd['odd'][xcell_free, ycell_free] = np.maximum(logodd['odd'][xcell_free, ycell_free], logodd['logodd_min'])
    logodd['odd'][xcell_hit, ycell_hit] += logodd['logodd_occ']
    logodd['odd'][xcell_hit, ycell_hit] = np.minimum(logodd['odd'][xcell_free, ycell_free], logodd['logodd_max'])

    # MAP['map']=sigmoid(logodd['odd'],20,0)
    MAP['map'][logodd['odd']>0]=1#occ
    MAP['map'][logodd['odd'] == 0]=0#unsure/free
    MAP['map'][logodd['odd'] < 0]=-1#free
    return MAP,logodd


def mapping(ranges_pts_G,valid_pro,pose_cur,MAP,logodd):#ranges:(n, 3)




    # MAP=iniMap(res,xmin,xmax,ymin,ymax)
    # # extract x,y
    ranges_G_pro=ranges_pts_G[valid_pro]
    xs0 = ranges_G_pro[:,0].reshape(1,-1)
    ys0 = ranges_G_pro[:, 1].reshape(1, -1)
    yaw0 = ranges_G_pro[:, 2].reshape(1, -1)


    # xs0 = np.array([ranges_raw * np.cos(angles)]);
    # ys0 = np.array([ranges_raw * np.sin(angles)]);


    # convert position in the map frame here
    Y = np.concatenate([np.concatenate([xs0, ys0], axis=0), np.zeros(xs0.shape)], axis=0)
    ### HERE
    # convert from meters to cells, compute pro pts index
    xind_map_pro = np.ceil((xs0 - MAP['xmin']) / MAP['res']).astype(np.int16) - 1
    yind_map_pro = np.ceil((ys0 - MAP['ymin']) / MAP['res']).astype(np.int16) - 1

    # # build an arbitrary map
    # indGood = np.logical_and(np.logical_and(np.logical_and((xind_map_pro > 1), (yind_map_pro > 1)), (xind_map_pro < MAP['sizex'])),
    #                          (yind_map_pro < MAP['sizey']))
    # ##inds = sub2ind(size(MAP.map),xis(indGood),yis(indGood));
    # MAP['map'][map_x_pro[indGood][0], map_x_pro[indGood][0]] = 1  # Maybe this is a problem
    # MAP['map'][xind_map_pro[0], yind_map_pro[0]] =1
    
    # map_x_phy = np.arange(MAP['xmin'], MAP['xmax'] + MAP['res'], MAP['res'])  # x-positions of each pixel of the map
    # map_y_phy = np.arange(MAP['ymin'], MAP['ymax'] + MAP['res'], MAP['res'])  # y-positions of each pixel of the map

    # x_range = np.arange(-0.2, 0.2 + 0.05, 0.05)
    # y_range = np.arange(-0.2, 0.2 + 0.05, 0.05)

    #pts used to

    # fig1 = plt.figure(1)
    # plt.plot(xs0, ys0, '.k')
    # # plt.xticks()
    #convert current cell-location
    pose_xcell=np.ceil((pose_cur[0] - MAP['xmin']) / MAP['res']).astype(np.int16) - 1
    pose_ycell=np.ceil((pose_cur[1] - MAP['xmin']) / MAP['res']).astype(np.int16) - 1

    #get believed free cells
    xycell_free=getMapCellsFromRay(pose_xcell,pose_ycell,xind_map_pro,yind_map_pro)

    #update occu map and logodd
    # MAP['odd']=+
    MAP, logodd=updateMAP_logodd(MAP,logodd,xycell_free[:,0],xycell_free[:,1],xind_map_pro,yind_map_pro)

    return MAP,logodd
    # #Correlation
    # ranges_G_cor=ranges_pts_G[valid_cor]
    # x_cor = ranges_G_cor[:, 0]
    # y_cor = ranges_G_cor[:, 1]
    # c = MU.mapCorrelation(MAP['map'], xmap_ind, ymap_ind, Y[0:3, :], x_range, y_range)
    #
    # logodd
