import numpy as np
from pandas import *
import sys, traceback
from general_robotics_toolbox import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import UnivariateSpline

sys.path.append('../toolbox')
from robots_def import *
from lambda_calc import *


def main():

	col_names=['X', 'Y', 'Z','direction_x','direction_y','direction_z'] 
	data = read_csv("from_ge/Curve_in_base_frame2.csv", names=col_names)
	curve_x=data['X'].tolist()
	curve_y=data['Y'].tolist()
	curve_z=data['Z'].tolist()
	curve_direction_x=data['direction_x'].tolist()
	curve_direction_y=data['direction_y'].tolist()
	curve_direction_z=data['direction_z'].tolist()

	curve=np.vstack((curve_x, curve_y, curve_z)).T
	curve_direction=np.vstack((curve_direction_x,curve_direction_y,curve_direction_z)).T

	lam=calc_lam_cs(curve)

	# spl_x = UnivariateSpline(lam,curve_x,k=5)
	# spl_y = UnivariateSpline(lam,curve_y,k=5)
	# spl_z = UnivariateSpline(lam,curve_z,k=5)

	# splined_x = spl_x(lam)
	# splined_y = spl_y(lam)
	# splined_z = spl_z(lam)

	# curve_splined=np.vstack((splined_x, splined_y, splined_z)).T

	polyfit=np.polyfit(lam,curve,deg=47)
	polyfit_x=np.poly1d(polyfit[:,0])(lam)
	polyfit_y=np.poly1d(polyfit[:,1])(lam)
	polyfit_z=np.poly1d(polyfit[:,2])(lam)
	curve_polyfit=np.vstack((polyfit_x, polyfit_y, polyfit_z)).T

	diff=np.linalg.norm(curve-curve_polyfit,axis=1)
	print('polyfit error: ',np.max(diff),np.average(diff))

	polyfit_dir=np.polyfit(lam,curve_direction,deg=47)
	polyfit_x_dir=np.poly1d(polyfit_dir[:,0])(lam)
	polyfit_y_dir=np.poly1d(polyfit_dir[:,1])(lam)
	polyfit_z_dir=np.poly1d(polyfit_dir[:,2])(lam)
	curve_polyfit_dir=np.vstack((polyfit_x_dir, polyfit_y_dir, polyfit_z_dir)).T

	diff_dir=np.linalg.norm(curve_direction-curve_polyfit_dir,axis=1)
	print('polyfit_dir error: ',np.max(diff_dir),np.average(diff_dir))


	df=DataFrame({'x':lam})
	df.to_csv("from_ge/lambda.csv",header=False,index=False)

	df=DataFrame({'polyfit_x':polyfit[:,0],'polyfit_y':polyfit[:,1],'polyfit_z':polyfit[:,2],'polyfit_x_dir':polyfit_dir[:,0],'polyfit_y_dir':polyfit_dir[:,1],'polyfit_z_dir':polyfit_dir[:,2]})
	df.to_csv("from_ge/analytical_expression.csv",header=False,index=False)

if __name__ == "__main__":
	main()