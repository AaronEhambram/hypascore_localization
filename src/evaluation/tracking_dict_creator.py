#!/usr/bin/python
import numpy as np

def create_data_dict():
  data_dict = {
    "times" : [],
    "polygon_corners" : [],
    "rotrect_hull_corners" : [],
    "rotrect_size" : [],
    "angle_interval" : [],
    "bounded_opt_pose" : [],
    "unbounded_opt_pose" : [],
    "num_associated_facades" : [],
    "ratio_associated" : [],
    "op_time_full" : [],
    "op_time_iHT" : [],
    "op_time_polygon" : [],
    "op_time_assocation" : []
  }
  return data_dict

def fill_data_timestamp(data_dict,file):
  line = file.readline()

  # time
  data_dict["times"] = np.append(data_dict["times"],[float(line)])
  
  # corner points 
  line = file.readline()
  data = [float(i) for i in line.split()]
  corners = np.empty([0,2])
  for i in range(0,len(data)-1,2):
    p = np.array([data[i],data[i+1]])
    corners = np.append(corners,[p],axis=0)
  data_dict["polygon_corners"].append(corners)

  # rotated hull
  line = file.readline()
  data = [float(i) for i in line.split()]
  rotrect_corners = np.empty([0,2])
  rotrect_size = np.empty([0,2])
  if len(data) == 10:
    p1 = np.array([data[0],data[1]])
    p2 = np.array([data[2],data[3]])
    p3 = np.array([data[4],data[5]])
    p4 = np.array([data[6],data[7]])
    rotrect_corners = np.append(rotrect_corners,[p1],axis=0)
    rotrect_corners = np.append(rotrect_corners,[p2],axis=0)
    rotrect_corners = np.append(rotrect_corners,[p3],axis=0)
    rotrect_corners = np.append(rotrect_corners,[p4],axis=0)
    size = np.array([data[8],data[9]])
    rotrect_size = size#np.append(rotrect_size,[size],axis=0)
  data_dict["rotrect_hull_corners"].append(rotrect_corners)
  data_dict["rotrect_size"].append(rotrect_size)

  # angle interval
  line = file.readline()
  data = [float(i) for i in line.split()]
  angle_interval = np.empty([0,2])
  if len(data) == 2:
    interval = np.array([data[0],data[1]])
    angle_interval = interval#np.append(angle_interval,[interval],axis=0)
  data_dict["angle_interval"].append(angle_interval)

  # bounded optimal pose
  line = file.readline()
  data = [float(i) for i in line.split()]
  bounded_opt_pose = np.empty([0,3])
  if len(data) == 3:
    pose = np.array([data[0],data[1],data[2]])
    bounded_opt_pose = pose#np.append(bounded_opt_pose,[pose],axis=0)
  data_dict["bounded_opt_pose"].append(bounded_opt_pose)

  # unbounded optimal pose
  line = file.readline()
  data = [float(i) for i in line.split()]
  unbounded_opt_pose = np.empty([0,3])
  if len(data) == 3:
    pose = np.array([data[0],data[1],data[2]])
    unbounded_opt_pose = pose#np.append(unbounded_opt_pose,[pose],axis=0)
  data_dict["unbounded_opt_pose"].append(unbounded_opt_pose)

  # num associated points
  line = file.readline()
  data_dict["num_associated_facades"] = np.append(data_dict["num_associated_facades"],[float(line)])

  # associated ratio
  line = file.readline()
  data_dict["ratio_associated"] = np.append(data_dict["ratio_associated"],[float(line)])

  # operation times
  line = file.readline()
  data = [float(i) for i in line.split()]
  if len(data) == 4:
    data_dict["op_time_full"] = np.append(data_dict["op_time_full"],[float(data[0])])
    data_dict["op_time_iHT"] = np.append(data_dict["op_time_iHT"],[float(data[1])])
    data_dict["op_time_polygon"] = np.append(data_dict["op_time_polygon"],[float(data[2])])
    data_dict["op_time_assocation"] = np.append(data_dict["op_time_assocation"],[float(data[3])])

def clear_empty_entries(data_dict):
  # get the indexes that need to be deleted
  to_delete_indxs = []
  for i in range(len(data_dict["times"])):
    if data_dict["polygon_corners"][i].size == 0:
      to_delete_indxs.append(i)
  
  for idx in range(len(to_delete_indxs)-1,-1,-1):
    i = to_delete_indxs[idx]
    data_dict["times"] = np.delete(data_dict["times"],i)
    del data_dict["polygon_corners"][i]
    del data_dict["rotrect_hull_corners"][i]
    del data_dict["rotrect_size"][i]
    del data_dict["angle_interval"][i]
    del data_dict["bounded_opt_pose"][i]
    del data_dict["unbounded_opt_pose"][i]
    data_dict["num_associated_facades"] = np.delete(data_dict["num_associated_facades"],i)
    data_dict["ratio_associated"] = np.delete(data_dict["ratio_associated"],i)
    data_dict["op_time_full"] = np.delete(data_dict["op_time_full"],i)
    data_dict["op_time_iHT"] = np.delete(data_dict["op_time_iHT"],i)
    data_dict["op_time_polygon"] = np.delete(data_dict["op_time_polygon"],i)
    data_dict["op_time_assocation"] = np.delete(data_dict["op_time_assocation"],i)

def get_data_dict(file_path):
  data_dict = create_data_dict()
  file = open(file_path,"r")
  if file.mode == "r":
    line = file.readline()
    while line:
      if(line == "---\n"):
        fill_data_timestamp(data_dict,file)
        line = file.readline()
      else:
        print("ERROR: sequence of '---' inconsistent")
  clear_empty_entries(data_dict)
  return data_dict