from toolbox.robots_def import *
import yaml

robot1_choose='FANUC_m10ia'
robot1=robot_obj(robot1_choose,'config/'+robot1_choose+'_robot_default_config.yml',tool_file_path='config/laser_ge.csv',d=0,acc_dict_path='config/'+robot1_choose+'_acc.pickle')

print('erere')

for i in range(100):
    joint_angles=np.random.uniform(np.clip(robot1.lower_limit,-np.pi,np.pi),np.clip(robot1.upper_limit,-np.pi,np.pi))
    tcp=robot1.fwd(joint_angles)

    inv_sol=robot1.inv(tcp.p,tcp.R)
    check=False
    for sol in inv_sol:
        if np.all(np.abs(joint_angles-sol)<np.ones(6)*1e-8):
            check=True
            break
    if not check:
        print("Errrorrrrr!!!")
        print(joint_angles)
        print(inv_sol)
        print(tcp)
print(sol)
print(inv_sol)