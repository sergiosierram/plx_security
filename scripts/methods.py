import numpy as np

class security(object):
	def polar_2_cartesian(self,angle_min,angle_max,angle_increment,ranges):
		angles = np.arange(angle_min,angle_max,angle_increment)
		ranges = np.array(ranges)
		
		y = ranges*np.sin(angles)
		x = ranges*np.cos(angles)
		
		return x,y
		
	def is_inside(self,x,y,p1,p2):
		is_inside_x = np.zeros(x.shape)
		is_inside_y = np.zeros(y.shape)
		
		is_inside_x[np.nonzero(x>p1[0] and x<p2[0])] = 1
		is_inside_y[np.nonzero(y>=p1[1] and y<=p2[1]] = 1

		is_inside_xy = np.zeros(x.shape)
		indices = is_inside_x==1 and is_inside_y==1
		is_inside_xy[indices] = 1
		
		if np.sum(is_inside_xy)>0:
			is_inside = True
			d = np.amin(x[indeces])
		else:
			is_inside = False
			d = []
			
		return is_inside,d
			
			
	
