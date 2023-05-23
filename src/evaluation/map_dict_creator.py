#!/usr/bin/python
import numpy as np

def create_map_dict():
  data_dict = {
    "grounds" : [],
    "roofs" : [],
    "walls" : []
  }
  return data_dict

def insert_ground(map_dict,x_offset,y_offset,file):
  line = file.readline()
  subline = line.split(":")
  if subline[0]=="surface":
    line = file.readline()
    # read the number until "roof:", "wall:", "building_id:" appears
    surface = np.empty([0,2])
    while not(line.split(":")[0]=="roof" or line.split(":")[0]=="wall" or line.split(":")[0]=="building_id"):
      data = [float(i) for i in line.split()]
      point = np.array([data[0]-x_offset,data[1]-y_offset])
      surface = np.append(surface,[point],axis=0)
      line = file.readline()
    map_dict["grounds"].append(surface)
  

def get_map_dict(file_path,x_offset,y_offset):
  map_dict = create_map_dict()
  file = open(file_path,"r")
  if file.mode == "r":
    line = file.readline()
    while line:
      if(line == "ground:\n"):
        insert_ground(map_dict,x_offset,y_offset,file)
        line = file.readline()
      else:
        line = file.readline()
  return map_dict