import numpy as np
from MapUtils.MapUtils import getMapCellsFromRay
from helper import scan2World
from utils import cal_T_b_g,rangeH2rangeG,rangeRaw2range_xyz_Lidar
# from

# init MAP

res=0.05
xmin=-20
xmax=20
ymin=-20
ymax=20
occFactor=1/9
angles = np.arange(-135, 135.25, 0.25) * np.pi / 180.

def iniLogOdd(sizex,sizey,thres,p11):
    logodd={}
    logodd['odd']=np.zeros([sizex, sizey])
    # logodd['p11'] =p11# P(occupied|measured occupied);(z=1,m=1)
    # logodd['p01'] = 1-p11   # P(free|measured occupied)
    # logodd['p00']= p00 # P(free | measured free);
    # logodd['p10'] = 1 - p00 # P(occupied | measured free);
    logodd['logodd_occ'] = np.log(p11 / (1 - p11)) # > 0
    logodd['logodd_free'] = np.log((1 - p11) / p11) * 0.5 # < 0
    logodd['logodd_max'] = np.log(thres)
    logodd['logodd_min'] = -np.log(thres)
    logodd[thres] = thres
    return logodd

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
    MAP['binary']=np.zeros((MAP['sizex'],MAP['sizey']),dtype=np.int8)
    # MAP['binary']=MAP['odd'].copy()
    # MAP['occFactor']=1/9
    MAP['xcell_phy']= np.arange(MAP['xmin'], MAP['xmax'] + MAP['res'], MAP['res'])  # x-positions of each pixel of the map
    MAP['ycell_phy'] = np.arange(MAP['ymin'], MAP['ymax'] + MAP['res'], MAP['res'])
    return MAP


def updateMAP_logodd(MAP,logodd,xcell_free,ycell_free,xcell_hit,ycell_hit):
    logodd['odd'][ycell_free, xcell_free] += logodd['logodd_free']
    logodd['odd'][ycell_free, xcell_free] = np.maximum(logodd['odd'][ycell_free, xcell_free], logodd['logodd_min'])
    logodd['odd'][ycell_hit, xcell_hit] += logodd['logodd_occ']
    logodd['odd'][ycell_hit, xcell_hit] = np.minimum(logodd['odd'][ycell_hit, xcell_hit], logodd['logodd_max'])

    # MAP['map']=sigmoid(logodd['odd'],20,0)
    MAP['map'][logodd['odd']>0]=1#occ
    MAP['map'][logodd['odd'] <= 0]=0#unsure/free
    # MAP['map'][logodd['odd'] < 0]=-1#free

    MAP['binary'][logodd['odd'] > 0] = 1  # occ
    MAP['binary'][logodd['odd'] <= 0] = 0  # unsure/free
    MAP['binary'][logodd['odd'] < 0]=-1#free

    return MAP,logodd


# def cal_T_b_g():
#
#
#     return T_b_g



def mapping(range_raw,head_angles,p_best,MAP,logodd):#ranges:(n, 3)


    #Transform ranges form H frame to G frame
    # T_b_g=cal_T_b_g(p_best[0],p_best[1],rpy_unbiased)
    angles = np.arange(-135, 135.25, 0.25) * np.pi / 180.
    range_xyz_lidar=rangeRaw2range_xyz_Lidar(range_raw,angles)#(n,4)
    # range_G=rangeH2rangeG(range_H,T_b_g.dot(T_h_b))
    # range_G = rangeRaw2G(range_raw, angles, T_b_g.dot(T_h_b))
    range_xyz_world=scan2World(lidar_xyz=range_xyz_lidar.T, neck_angle=head_angles[0],head_angle=head_angles[1],pose=p_best).T

    #remove invalid ranges
    indValid_cf = np.logical_and((range_raw < 30), (range_raw > 0.1))
    indValid_c = np.logical_and((range_raw < 10), (range_raw > 0.1))
    indValid_ground=range_xyz_world[:,-1]>0.1
    indValid=np.logical_and(indValid_cf,indValid_ground)
    range_xyz_world=range_xyz_world[indValid]
    # angles=angles[indValid]

    #convert phy corrds to map corr
    xrange_map = np.ceil((range_xyz_world[:,0] - MAP['xmin']) / MAP['res']).astype(np.int16) - 1
    yrange_map = np.ceil((range_xyz_world[:,1] - MAP['ymin']) / MAP['res']).astype(np.int16) - 1

    xp_best_map=np.ceil((p_best[0] - MAP['xmin']) / MAP['res']).astype(np.int16) - 1
    yp_best_map=np.ceil((p_best[1] - MAP['ymin']) / MAP['res']).astype(np.int16) - 1




    #get free cells corrds
    xy_free=getMapCellsFromRay(MAP['map'],xp_best_map,yp_best_map,xrange_map,yrange_map)
    xmap_free = xy_free[1]
    ymap_free = xy_free[0]

    #update map&logodd
    MAP, logodd=updateMAP_logodd(MAP,logodd,xmap_free,ymap_free,xrange_map,yrange_map)

    return MAP, logodd,range_xyz_lidar,indValid_c
