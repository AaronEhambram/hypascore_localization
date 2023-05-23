#!/usr/bin/python
import numpy as np

def create_gt_traj_data_dict():
  data_dict = {
    "times" : [],
    "world_T_car" : [],
    "world_T_laser" : [],
    "world_T_lcam" : []
  }
  return data_dict

def fill_gt_traj_data_timestamp(data_dict,file):
  line = file.readline()

  # time
  data_dict["times"] = np.append(data_dict["times"],[float(line)])
  
  # world_T_car 
  line = file.readline()
  data = [float(i) for i in line.split()]
  world_T_car = np.empty([0,6])
  if len(data) == 6:
    coord = np.array([data[0],data[1],data[2],data[3],data[4],data[5]])
    world_T_car = coord
  data_dict["world_T_car"].append(world_T_car)

  # world_T_laser
  line = file.readline()
  data = [float(i) for i in line.split()]
  world_T_laser = np.empty([0,6])
  if len(data) == 6:
    coord = np.array([data[0],data[1],data[2],data[3],data[4],data[5]])
    world_T_laser = coord
  data_dict["world_T_laser"].append(world_T_laser)

  # world_T_lcam
  line = file.readline()
  data = [float(i) for i in line.split()]
  world_T_lcam = np.empty([0,6])
  if len(data) == 6:
    coord = np.array([data[0],data[1],data[2],data[3],data[4],data[5]])
    world_T_lcam = coord
  data_dict["world_T_lcam"].append(world_T_lcam)


def get_gt_traj_data_dict(file_path):
  data_dict = create_gt_traj_data_dict()
  file = open(file_path,"r")
  if file.mode == "r":
    line = file.readline()
    while line:
      if(line == "---\n"):
        fill_gt_traj_data_timestamp(data_dict,file)
        line = file.readline()
      else:
        print("ERROR: sequence of '---' inconsistent")
  return data_dict