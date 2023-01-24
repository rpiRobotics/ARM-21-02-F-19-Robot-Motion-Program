import yaml
from math import degrees,radians
import argparse
import sys

parser = argparse.ArgumentParser(description='Filename of robot info')
parser.add_argument('filename', metavar='filename', type=str, nargs='+',
                    help='filename of robot info')
if len(sys.argv)!=2:
    print("Please enter one and only one robot info file.")
args=parser.parse_args()

robot_info_filename=args.filename[0]

with open('example_config.yml','r') as f:
    robot_config=yaml.load(f)

with open(robot_info_filename,'r') as f:
    robot_info=yaml.load(f)

robot_config['device_info']['device']['name']=robot_info['device_name']
robot_config['device_info']['manufacturer']['name']=robot_info['manufacturer']
robot_config['device_info']['model']['name']=robot_info['model']
robot_config['device_info']['user_description']=robot_info['user_description']
robot_config['chains'][0]['H']=robot_info['axis']
robot_config['chains'][0]['P']=robot_info['chain']

for i in range(6):
    robot_config['joint_info'][i]['joint_limits']['lower']=radians(robot_info['joint_limits_lower'][i])
    robot_config['joint_info'][i]['joint_limits']['upper']=radians(robot_info['joint_limits_upper'][i])
    robot_config['joint_info'][i]['joint_limits']['velocity']=radians(robot_info['joint_vel_limits'][i])

output_filename=robot_info['manufacturer']+'_'+robot_info['model']+'_robot_default_config.yml'
with open(output_filename,'w') as f:
    yaml.safe_dump(robot_config,f)