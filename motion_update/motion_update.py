
from .ilc_toolbox import *
from fanuc_utils import *
from .max_gradient_fanuc import *

def motion_program_update(filepath,robot,vel,desired_curve_filename,desired_curvejs_filename,\
    err_tol,angerr_tol,velstd_tol):
    
    curve = read_csv(desired_curve_filename,header=None).values
    curve=np.array(curve)
    curve_js = read_csv(desired_curvejs_filename,header=None).values
    curve_js=np.array(curve_js)

    return max_grad_descent(filepath,robot,vel,curve,curve_js,err_tol,angerr_tol,velstd_tol,save_all_file=True,save_ls=True,save_name='final_ls')
