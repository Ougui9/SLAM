import numpy as np
import MapUtils.MapUtils as MU
import matplotlib.pyplot as plt


# init MAP

def iniMap():
    MAP = {}
    MAP['res']   = 0.05 #meters
    MAP['xmin']  = -20  #meters
    MAP['ymin']  = -20
    MAP['xmax']  =  20
    MAP['ymax']  =  20
    MAP['sizex']  = int(np.ceil((MAP['xmax'] - MAP['xmin']) / MAP['res'] + 1)) #cells
    MAP['sizey']  = int(np.ceil((MAP['ymax'] - MAP['ymin']) / MAP['res'] + 1))

    MAP['map'] = np.zeros((MAP['sizex'],MAP['sizey']),dtype=np.int8) #DATA TYPE: char or int8

    return MAP


def mapping(ranges_pts_G, angles):#ranges:(n, 3)

    # # take valid indices
    # indValid = np.logical_and((ranges < 30), (ranges > 0.1))
    # ranges_raw = ranges_raw[indValid]
    # angles = angles[indValid]
    MAP=iniMap()
    # # extract x,y
    xs0 = ranges_pts_G[:,0].reshape(1,-1)
    ys0 = ranges_pts_G[:,1].reshape(1,-1)

    # xs0 = np.array([ranges_raw * np.cos(angles)]);
    # ys0 = np.array([ranges_raw * np.sin(angles)]);


    # convert position in the map frame here
    Y = np.concatenate([np.concatenate([xs0, ys0], axis=0), np.zeros(xs0.shape)], axis=0)
    ### HERE
    # convert from meters to cells
    xis = np.ceil((xs0 - MAP['xmin']) / MAP['res']).astype(np.int16) - 1
    yis = np.ceil((ys0 - MAP['ymin']) / MAP['res']).astype(np.int16) - 1

    # build an arbitrary map
    indGood = np.logical_and(np.logical_and(np.logical_and((xis > 1), (yis > 1)), (xis < MAP['sizex'])),
                             (yis < MAP['sizey']))
    # ##inds = sub2ind(size(MAP.map),xis(indGood),yis(indGood));
    MAP['map'][xis[0], yis[0]] = 1  # Maybe this is a problem

    x_im = np.arange(MAP['xmin'], MAP['xmax'] + MAP['res'], MAP['res'])  # x-positions of each pixel of the map
    y_im = np.arange(MAP['ymin'], MAP['ymax'] + MAP['res'], MAP['res'])  # y-positions of each pixel of the map

    x_range = np.arange(-0.2, 0.2 + 0.05, 0.05)
    y_range = np.arange(-0.2, 0.2 + 0.05, 0.05)

    fig1 = plt.figure(1)
    plt.plot(xs0, ys0, '.k')
    # plt.xticks()


    #Correlation

    c = MU.mapCorrelation(MAP['map'], x_im, y_im, Y[0:3, :], x_range, y_range)
