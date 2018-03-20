#Daniel D. Lee, Alex Kushleyev, Kelsey Saulnier, Nikolay Atanasov
import numpy as np
import MapUtils.bresenham2D as bresenham2D

# INPUT 
# im              the map 
# x_im,y_im       physical x,y positions of the grid map cells
# vp(0:2,:)       occupied x,y positions from range sensor (in physical unit)  
# xs,ys           physical x,y,positions you want to evaluate "correlation" 
#
# OUTPUT 
# c               sum of the cell values of all the positions hit by range sensor
def mapCorrelation(im, x_im, y_im, vp, xs, ys):
	nx = im.shape[0]
	ny = im.shape[1]
	xmin = x_im[0]
	xmax = x_im[-1]
	xresolution = (xmax-xmin)/(nx-1)
	ymin = y_im[0]
	ymax = y_im[-1]
	yresolution = (ymax-ymin)/(ny-1)
	nxs = xs.size
	nys = ys.size
	cpr = np.zeros((nxs, nys))
	for jy in range(0, nys):
		y1 = vp[1, :] + ys[jy]  # 1 x 1076
		iy = np.int16(np.round((ymax - y1) / yresolution))  # changed from (y1-ymin)
		for jx in range(0, nxs):
			x1 = vp[0, :] + xs[jx]  # 1 x 1076
			ix = np.int16(np.round((x1 - xmin) / xresolution))
			valid = np.logical_and(np.logical_and((iy >= 0), (iy < ny)),np.logical_and((ix >= 0), (ix < nx)))
			cpr[jx, jy] = np.sum(im[iy[valid], ix[valid]])  # SWITCHED ix AND iy
	return cpr


#Bresenham's line algorithm
def getMapCellsFromRay(x0t,y0t,xis,yis):
	nPoints = np.size(xis)
	xyio = np.array([[],[]])
	for x1, y1 in zip(xis,yis):
		ox,oy = bresenham2D.bresenham2D(x0t,y0t,x1,y1)
		xyio = np.concatenate((xyio,np.array([ox,oy])),axis=1)
	return xyio.astype(int)


