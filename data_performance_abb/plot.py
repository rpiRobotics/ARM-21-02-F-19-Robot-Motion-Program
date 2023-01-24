import numpy as np
from pandas import *
import sys, traceback
from general_robotics_toolbox import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
sys.path.append('../toolbox')
from robots_def import *
from utils import *
from lambda_calc import *


def main():
	robot1=abb6640(d=50)
	robot2=abb1200()

	dataset='from_NX/'
	solution_dir='dual_arm/diffevo_pose2_2/'
	data_dir=dataset+solution_dir

	curve_dense=np.loadtxt(dataset+'Curve_dense.csv', delimiter=',')
	lam=calc_lam_cs(curve_dense)

	curve_js1 = np.loadtxt(data_dir+'arm1.csv', delimiter=',')
	curve_js2 = np.loadtxt(data_dir+'arm2.csv', delimiter=',')

	curve1=robot1.fwd_all(curve_js1).p_all
	curve2=robot2.fwd_all(curve_js2).p_all

	print(curve1)


	curvature1=calc_curvature(curve1)
	curvature2=calc_curvature(curve2)

	plt.plot(lam,curvature1)
	plt.plot(lam,curvature2)
	plt.xlabel('Lambda (mm)')
	plt.ylabel('Curvature')
	plt.title('Curvature Plot')
	plt.show()

	# visualize_curve_w_normal(curve,curve_normal,stepsize=1000,equal_axis=True)

	# print(np.amax(curve[:,0])-np.amin(curve[:,0]),np.amax(curve[:,1])-np.amin(curve[:,1]),np.amax(curve[:,2])-np.amin(curve[:,2]))

if __name__ == "__main__":
	main()