import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import sys
from scipy.optimize import fminbound
sys.path.append('../../toolbox')
from lambda_calc import *
from utils import *




def main():

    relative_path=read_csv("Curve_dense.csv",header=None).values
    curve=relative_path[:,:3]
    curve_normal=relative_path[:,3:]
    ##############3D plots####################
    # xsurf = np.linspace(0,1000,100)
    # ysurf = np.linspace(- R,R,100)
    # zsurf = np.zeros((len(xsurf),len(ysurf)))
    # for i in range(len(ysurf)):
    #     zsurf[:,i] = (R ** 2 - ysurf[i] ** 2) * H / R ** 2

    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    # ax.plot_surface(xsurf, ysurf, zsurf, cmap=cm.coolwarm,
    #                        linewidth=0, antialiased=False)
    ax.plot3D(curve[:,0],curve[:,1],curve[:,2],'r.-')
    ax.quiver(curve[::1000,0],curve[::1000,1],curve[::1000,2],curve_normal[::1000,0],curve_normal[::1000,1],curve_normal[::1000,2],length=50, normalize=True)
    plt.title('Curve 1')
    plt.show()

if __name__ == '__main__':
	main()