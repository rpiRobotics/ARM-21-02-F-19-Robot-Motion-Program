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

	col_names=['q1', 'q2', 'q3','q4', 'q5', 'q6'] 
	data = read_csv("from_ge/Curve_js2.csv", names=col_names)
	curve_q1=data['q1'].tolist()
	curve_q2=data['q2'].tolist()
	curve_q3=data['q3'].tolist()
	curve_q4=data['q4'].tolist()
	curve_q5=data['q5'].tolist()
	curve_q6=data['q6'].tolist()
	curve_js=np.vstack((curve_q1, curve_q2, curve_q3,curve_q4,curve_q5,curve_q6)).T

	robot=abb6640(d=50)
	lam=calc_lam_js(curve_js,robot)

	polyfit=np.polyfit(lam,curve_js,deg=47)
	polyfit_q1=np.poly1d(polyfit[:,0])(lam)
	polyfit_q2=np.poly1d(polyfit[:,1])(lam)
	polyfit_q3=np.poly1d(polyfit[:,2])(lam)
	polyfit_q4=np.poly1d(polyfit[:,3])(lam)
	polyfit_q5=np.poly1d(polyfit[:,4])(lam)
	polyfit_q6=np.poly1d(polyfit[:,5])(lam)
	curve_js_polyfit=np.vstack((polyfit_q1, polyfit_q2, polyfit_q3,polyfit_q4,polyfit_q5,polyfit_q6)).T

	diff=np.linalg.norm(curve_js-curve_js_polyfit,axis=1)

	print('polyfit error: ',np.max(diff),np.average(diff))


	df=DataFrame({'polyfit_q1':polyfit[:,0],'polyfit_q2':polyfit[:,1],'polyfit_q3':polyfit[:,2],'polyfit_q4':polyfit[:,3],'polyfit_q5':polyfit[:,4],'polyfit_q6':polyfit[:,5]})
	df.to_csv("from_ge/Curve_js_poly.csv",header=False,index=False)

if __name__ == "__main__":
	main()