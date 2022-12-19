import numpy as np
from pandas import *
import sys, traceback
from general_robotics_toolbox import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import UnivariateSpline

sys.path.append('../../toolbox')
from lambda_calc import *
from utils import *

def main():

	leading_edge = read_csv('leading_edge.csv').values
	leading_edge_shift = read_csv('leading_edge+0.02.csv').values

	#fit leading edge curve
	lam1=calc_lam_cs(leading_edge)

	polyfit=np.polyfit(lam1,leading_edge,deg=47)
	polyfit_x=np.poly1d(polyfit[:,0])(lam1)
	polyfit_y=np.poly1d(polyfit[:,1])(lam1)
	polyfit_z=np.poly1d(polyfit[:,2])(lam1)
	leading_edge_polyfit=np.vstack((polyfit_x, polyfit_y, polyfit_z)).T

	diff=np.linalg.norm(leading_edge-leading_edge_polyfit,axis=1)
	print('leading edge polyfit error: ',np.max(diff),np.average(diff))

	#fit shifted leading edge curve
	lam2=calc_lam_cs(leading_edge_shift)

	polyfit_shift=np.polyfit(lam2,leading_edge_shift,deg=47)
	polyfit_shift_x=np.poly1d(polyfit_shift[:,0])(lam2)
	polyfit_shift_y=np.poly1d(polyfit_shift[:,1])(lam2)
	polyfit_shift_z=np.poly1d(polyfit_shift[:,2])(lam2)
	leading_edge_shift_polyfit=np.vstack((polyfit_shift_x, polyfit_shift_y, polyfit_shift_z)).T

	diff=np.linalg.norm(leading_edge_shift-leading_edge_shift_polyfit,axis=1)
	print('leading edge shifted polyfit error: ',np.max(diff),np.average(diff))

	####generating dense curve
	lam1_dense=np.linspace(0,lam1[-1],50000)
	polyfit_x=np.poly1d(polyfit[:,0])(lam1_dense)
	polyfit_y=np.poly1d(polyfit[:,1])(lam1_dense)
	polyfit_z=np.poly1d(polyfit[:,2])(lam1_dense)
	curve_dense=np.vstack((polyfit_x, polyfit_y, polyfit_z)).T

	lam2_dense=np.linspace(0,lam1[-1],50000)
	polyfit_shift_x=np.poly1d(polyfit_shift[:,0])(lam2_dense)
	polyfit_shift_y=np.poly1d(polyfit_shift[:,1])(lam2_dense)
	polyfit_shift_z=np.poly1d(polyfit_shift[:,2])(lam2_dense)
	curve_dense_shift=np.vstack((polyfit_shift_x, polyfit_shift_y, polyfit_shift_z)).T

	###generating corresponding underlying surface normal
	curve_tangent=np.gradient(curve_dense,axis=0)
	curve_shift_diff=curve_dense_shift-curve_dense
	curve_normal=np.cross(curve_shift_diff,curve_tangent)
	curve_normal=np.divide(curve_normal,np.tile(np.linalg.norm(curve_normal,axis=1),(3,1)).T)

	###rotate curve normal around curve tangent
	rotation_angle=np.radians(45)
	# rotation_angle=0
	curve_normal_rotated=[]
	for i in range(len(curve_dense)):
		R=rot(curve_tangent[i]/np.linalg.norm(curve_tangent[i]),rotation_angle)
		curve_normal_rotated.append(np.dot(R,curve_normal[i]))

	curve_normal_rotated=np.array(curve_normal_rotated)
	###plot results
	visualize_curve(curve_dense,curve_normal_rotated)

	###save results
	DataFrame(np.hstack((curve_dense,curve_normal_rotated))).to_csv('Curve_dense.csv',header=False,index=False)

if __name__ == "__main__":
	main()