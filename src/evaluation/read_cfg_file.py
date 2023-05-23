#!/usr/bin/python
import numpy as np

def read_cfg_file(file_path):
  file = open(file_path,"r")
  cfg = {
    "results_file" : "",
    "rt_only_results_file" : "", 
    "gt_trajectory_file" : "", 
    "map_file" : "", 
    "x_utm_offset" : float, 
    "y_utm_offset" : float,
    "sample_driven_distances" : []}
  if file.mode == "r":
    line = file.readline()
    while line:
      substrs = line.split(":")
      print(substrs[0])
      if substrs[0] == "results_file":
        cfg["results_file"] = substrs[1][:-1]
      elif substrs[0] == "rt_only_results_file":
        cfg["rt_only_results_file"] = substrs[1][:-1]
      elif substrs[0] == "gt_trajectory_file":
        cfg["gt_trajectory_file"] = substrs[1][:-1]
      elif substrs[0] == "map_file":
        cfg["map_file"] = substrs[1][:-1]
      elif substrs[0] == "x_utm_offset":
        cfg["x_utm_offset"] = float(substrs[1][:-1])
      elif substrs[0] == "y_utm_offset":
        cfg["y_utm_offset"] = float(substrs[1][:-1])
      elif substrs[0] == "sample_driven_distances":
        # comma separated list of driven distances at which the polygon and the boxes should be visualized
        str_dists = substrs[1][:-1].split(",")
        for str in str_dists:
          if not(str==""):
            cfg["sample_driven_distances"].append(float(str)) 
      line = file.readline()
  return cfg