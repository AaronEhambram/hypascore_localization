#!/usr/bin/python
import numpy as np
import math

class PoseParticle:
  pose = np.empty([3,1])
  age = 0
  weight = 0

class PoseEstimate:
  time = 0
  pose = np.empty([3,1])
  # feasible set
  feasible_set_box_corners = []
  feasible_set_angle = np.empty([1,2])
  #consistent set
  consistent_set_angle = np.empty([1,2])
  consistent_set_rot_rect_corners = []
  consistent_set_poly_corners = []

def create_data_dict():
  data_dict = {
    "real_times" : [], # this is real-time procesing time when the frame at loc_time was processed!
    "rt_pose_estimates" : [], # this is the according pose estimate for the real-time pose
    "loc_times" : [],
    "loc_pose_estimates" : [],
    "track_initialized" : [],
    "switch_status" : [],
    "contracted_exploration_region" : [],
    "track_reliability" : [],
    "facade_sine_angle_hull" : [],
    "loc_tracked_particles" : [],
    "loc_highest_weight_particles" : [],
    "vo_op_times" : [],
    "full_loc_op_times" : [],
    "coarse_loc_op_times" : [],
    "refined_loc_op_times" : []
  }
  return data_dict

def create_corner_array_from_box(xlb,xub,ylb,yub):
  bl = np.empty([2,1])
  bl[0][0] = xlb
  bl[1][0] = ylb
  tl = np.empty([2,1])
  tl[0][0] = xlb
  tl[1][0] = yub
  tr = np.empty([2,1])
  tr[0][0] = xub
  tr[1][0] = yub
  br = np.empty([2,1])
  br[0][0] = xub
  br[1][0] = ylb
  corners = []
  corners.append(bl)
  corners.append(tl)
  corners.append(tr)
  corners.append(br)
  return corners

def extract_angle_rotrect_poly(lsplit):
  consistent_set_angle = np.empty([1,2])
  consistent_set_rot_rect_corners = []
  consistent_set_poly_corners = []
  if(len(lsplit) > 1):
    # rot
    consistent_set_angle[0][0] = float(lsplit[0])
    consistent_set_angle[0][1] = float(lsplit[1])
    # rotrect corners
    c1 = np.empty([2,1])
    c1[0][0] = float(lsplit[2])
    c1[1][0] = float(lsplit[3])
    consistent_set_rot_rect_corners.append(c1)
    c2 = np.empty([2,1])
    c2[0][0] = float(lsplit[4])
    c2[1][0] = float(lsplit[5])
    consistent_set_rot_rect_corners.append(c2)
    c3 = np.empty([2,1])
    c3[0][0] = float(lsplit[6])
    c3[1][0] = float(lsplit[7])
    consistent_set_rot_rect_corners.append(c3)
    c4 = np.empty([2,1])
    c4[0][0] = float(lsplit[8])
    c4[1][0] = float(lsplit[9])
    consistent_set_rot_rect_corners.append(c4)
    # polygon corners
    i = 10
    while i < len(lsplit)-1:
      c = np.empty([2,1])
      c[0][0] = float(lsplit[i])
      c[1][0] = float(lsplit[i+1])
      consistent_set_poly_corners.append(c)
      i = i+2
  return consistent_set_angle, consistent_set_rot_rect_corners, consistent_set_poly_corners

def extract_frame_data(data_dict,file):
  line = file.readline()
  lsplit = line.split()
  # real_time
  real_time = float(lsplit[0])
  data_dict["real_times"].append(real_time)
  # loc time
  loc_time = float(lsplit[1])
  data_dict["loc_times"].append(loc_time)
  # track_initialized
  track_initialized = False
  if(lsplit[2] == "False"):
    track_initialized = False
  elif(lsplit[2] == "True"):
    track_initialized = True
  else:
    print("Error with track_initialized")
  data_dict["track_initialized"].append(track_initialized)
  # switch_status
  switch_status = lsplit[3] # Either "none", "switch", "reset"
  data_dict["switch_status"].append(switch_status)
  # contracted_exploration_region
  contracted_exploration_region = False
  if(lsplit[4] == "contracted"):
    contracted_exploration_region = True
  elif(lsplit[4] == "no"):
    contracted_exploration_region = False
  else:
    print("Error with contracted_exploration_region")
  data_dict["contracted_exploration_region"].append(contracted_exploration_region)
  # track_reliability
  track_reliability = float(lsplit[5]) 
  data_dict["track_reliability"].append(track_reliability)
  # age of the loc_tracked_particle
  loc_tracked_particle = PoseParticle()
  loc_tracked_particle.age = float(lsplit[6])
  # weight of the loc_tracked_particle
  loc_tracked_particle.weight = float(lsplit[7])
  # facade_sine_angle_hull 
  facade_sine_angle_hull = np.empty([1,2])
  facade_sine_angle_hull[0][0] = float(lsplit[8])
  facade_sine_angle_hull[0][1] = float(lsplit[9])
  data_dict["facade_sine_angle_hull"].append(facade_sine_angle_hull)
  # rt_pose_estimate
  rt_pose_estimate_obj = PoseEstimate()
  rt_pose_estimate_obj.time = real_time
  rt_pose_estimate = np.empty([3,1])
  rt_pose_estimate[0][0] = float(lsplit[10])
  rt_pose_estimate[1][0] = float(lsplit[11])
  rt_pose_estimate[2][0] = float(lsplit[12])
  rt_pose_estimate_obj.pose = rt_pose_estimate
  # loc_pose_estimate
  loc_pose_estimate_obj = PoseEstimate()
  loc_pose_estimate_obj.time = loc_time
  loc_pose_estimate = np.empty([3,1])
  loc_pose_estimate[0][0] = float(lsplit[13])
  loc_pose_estimate[1][0] = float(lsplit[14])
  loc_pose_estimate[2][0] = float(lsplit[15])
  loc_pose_estimate_obj.pose = loc_pose_estimate
  loc_tracked_particle.pose = loc_pose_estimate
  # operation times
  vo_time = float(lsplit[16])
  data_dict["vo_op_times"].append(vo_time)
  full_loc_time = float(lsplit[17])
  data_dict["full_loc_op_times"].append(full_loc_time)
  coarse_loc_time = float(lsplit[18])
  data_dict["coarse_loc_op_times"].append(coarse_loc_time)
  refined_loc_time = float(lsplit[19])
  data_dict["refined_loc_op_times"].append(refined_loc_time)

  line = file.readline()
  lsplit = line.split()
  # feasible set -> rt
  rt_feasible_box_rot = np.empty([1,2])
  rt_feasible_box_corners = []
  if(len(lsplit) > 1):
    rt_feasible_box_xlb = float(lsplit[0])
    rt_feasible_box_xub = float(lsplit[1])
    rt_feasible_box_ylb = float(lsplit[2])
    rt_feasible_box_yub = float(lsplit[3])
    rt_feasible_box_rot[0][0] = float(lsplit[4])
    rt_feasible_box_rot[0][1] = float(lsplit[5])
    rt_feasible_box_corners = create_corner_array_from_box(rt_feasible_box_xlb,rt_feasible_box_xub,rt_feasible_box_ylb,rt_feasible_box_yub)
  rt_pose_estimate_obj.feasible_set_angle = rt_feasible_box_rot
  rt_pose_estimate_obj.feasible_set_box_corners = rt_feasible_box_corners

  # feasible set  -> loc
  loc_feasible_box_rot = np.empty([1,2])
  loc_feasible_box_corners = []
  if(len(lsplit) > 1):
    loc_feasible_box_xlb = float(lsplit[6])
    loc_feasible_box_xub = float(lsplit[7])
    loc_feasible_box_ylb = float(lsplit[8])
    loc_feasible_box_yub = float(lsplit[9])
    loc_feasible_box_rot[0][0] = float(lsplit[10])
    loc_feasible_box_rot[0][1] = float(lsplit[11])
    loc_feasible_box_corners = create_corner_array_from_box(loc_feasible_box_xlb,loc_feasible_box_xub,loc_feasible_box_ylb,loc_feasible_box_yub)
  loc_pose_estimate_obj.feasible_set_angle = loc_feasible_box_rot
  loc_pose_estimate_obj.feasible_set_box_corners = loc_feasible_box_corners

  line = file.readline()
  # consistent set -> rt
  lsplit = line.split()
  if(len(lsplit) > 1):
    rt_consistent_set_angle, rt_consistent_set_rot_rect_corners, rt_consistent_set_poly_corners = extract_angle_rotrect_poly(lsplit)
    rt_pose_estimate_obj.consistent_set_angle = rt_consistent_set_angle
    rt_pose_estimate_obj.consistent_set_rot_rect_corners = rt_consistent_set_rot_rect_corners
    rt_pose_estimate_obj.consistent_set_poly_corners = rt_consistent_set_poly_corners

  line = file.readline()
  # consistent set -> loc
  lsplit = line.split()
  if(len(lsplit) > 1):
    loc_consistent_set_angle, loc_consistent_set_rot_rect_corners, loc_consistent_set_poly_corners = extract_angle_rotrect_poly(lsplit)
    loc_pose_estimate_obj.consistent_set_angle = loc_consistent_set_angle
    loc_pose_estimate_obj.consistent_set_rot_rect_corners = loc_consistent_set_rot_rect_corners
    loc_pose_estimate_obj.consistent_set_poly_corners = loc_consistent_set_poly_corners

  line = file.readline()
  # highest weight particle
  lsplit = line.split()
  if(len(lsplit) > 1):
    loc_highest_weight_particle = PoseParticle()
    loc_highest_weight_particle.pose[0][0] = float(lsplit[0])
    loc_highest_weight_particle.pose[1][0] = float(lsplit[1])
    loc_highest_weight_particle.pose[2][0] = float(lsplit[2])
    loc_highest_weight_particle.age = float(lsplit[3])
    loc_highest_weight_particle.weight = float(lsplit[4])

  data_dict["loc_tracked_particles"].append(loc_tracked_particle)
  data_dict["loc_highest_weight_particles"].append(loc_highest_weight_particle)
  data_dict["rt_pose_estimates"].append(rt_pose_estimate_obj)
  data_dict["loc_pose_estimates"].append(loc_pose_estimate_obj)

def get_data_dict(file_path):
  data_dict = create_data_dict()
  file = open(file_path,"r")
  if file.mode == "r":
    line = file.readline()
    while line:
      if(line == "---\n"):
        extract_frame_data(data_dict,file)
        line = file.readline()
      else:
        print("ERROR: sequence of '---' inconsistent")
  return data_dict


# real-time only data-dict creation
def create_rt_only_data_dict():
  data_dict = {
    "times" : [],
    "pose_estimates" : []
  }
  return data_dict

def extract_only_rt_frame_data(data_dict,file):
  line = file.readline()
  lsplit = line.split()
  # time
  time = float(lsplit[0])
  data_dict["times"].append(time)
  # pose_estimate -> pose
  pose_estimate_obj = PoseEstimate()
  pose_estimate = np.empty([3,1])
  pose_estimate[0][0] = float(lsplit[1])
  pose_estimate[1][0] = float(lsplit[2])
  pose_estimate[2][0] = float(lsplit[3])
  pose_estimate_obj.pose = pose_estimate
  pose_estimate_obj.time = time

  line = file.readline()
  lsplit = line.split()
  # feasible set
  if(len(lsplit) > 1):
    feasible_box_xlb = float(lsplit[0])
    feasible_box_xub = float(lsplit[1])
    feasible_box_ylb = float(lsplit[2])
    feasible_box_yub = float(lsplit[3])
    feasible_box_rot = np.empty([1,2])
    feasible_box_rot[0][0] = float(lsplit[4])
    feasible_box_rot[0][1] = float(lsplit[5])
    feasible_box_corners = create_corner_array_from_box(feasible_box_xlb,feasible_box_xub,feasible_box_ylb,feasible_box_yub)
    pose_estimate_obj.feasible_set_angle = feasible_box_rot
    pose_estimate_obj.feasible_set_box_corners = feasible_box_corners

  line = file.readline()
  lsplit = line.split()
  # consistent set -> rt
  if(len(lsplit) > 1):
    consistent_set_angle, consistent_set_rot_rect_corners, consistent_set_poly_corners = extract_angle_rotrect_poly(lsplit)
    pose_estimate_obj.consistent_set_angle = consistent_set_angle
    pose_estimate_obj.consistent_set_rot_rect_corners = consistent_set_rot_rect_corners
    pose_estimate_obj.consistent_set_poly_corners = consistent_set_poly_corners

  data_dict["pose_estimates"].append(pose_estimate_obj)

def get_rt_only_data_dict(file_path):
  data_dict = create_rt_only_data_dict()
  file = open(file_path,"r")
  if file.mode == "r":
    line = file.readline()
    while line:
      if(line == "---\n"):
        extract_only_rt_frame_data(data_dict,file)
        line = file.readline()
      else:
        print("ERROR: sequence of '---' inconsistent")
  return data_dict


# simple helper functions for evaluation
def small_large_side_from_rect_corners(corners):
  small_side = 0 
  large_side = 0
  if(len(corners) > 2):
    side1 = np.linalg.norm(corners[0]-corners[1])
    side2 = np.linalg.norm(corners[1]-corners[2])
    small_side = side2
    large_side = side1
    if(side1 < side2):
      small_side = side1
      large_side = side2
  return small_side, large_side

def point_on_line_segment_test(s,e,p_test):
  se = e-s
  sp = p_test-s
  det = se[0][0]*sp[1][0]-sp[0][0]*se[1][0]
  if(det == 0):
    # collinear
    se_sp = se[0][0]*sp[0][0]+se[1][0]*sp[1][0]
    se_se = se[0][0]*se[0][0]+se[1][0]*se[1][0]
    if(0 <= se_sp) and (se_sp <= se_se):
      return True
    else:
      return False
  else:
    return False

def point_in_polygon_test(polygon_in,test_point):
  angle_sum = 0
  polygon = []
  if(np.linalg.norm(polygon_in[0]-polygon_in[-1]) <= 0.000001):
    polygon = polygon_in
  else:
    polygon = polygon_in
    polygon.append(polygon[0])
  for i in range(1,len(polygon)):
    p1 = polygon[i-1]
    p2 = polygon[i]
    on_line = point_on_line_segment_test(p1,p2,test_point)
    if(on_line):
      return True
    p1p = p1-test_point
    p2p = p2-test_point
    angle = math.atan2(p2p[1][0]*p1p[0][0]-p2p[0][0]*p1p[1][0],p2p[0][0]*p1p[0][0]+p2p[1][0]*p1p[1][0]) # directed angle
    angle_sum = angle_sum+angle
  if(abs(angle_sum) >= 2*math.pi-0.0000001):
    return True
  else:
    return False
  
# multiple function evaluation
def compute_time_driven_distance_map(gt_traj_data_dict):
  time_driven_distance_map = {}
  time_driven_distance_map[gt_traj_data_dict["times"][0]] = 0
  abs_driven_distance = 0
  for i in range(0,len(gt_traj_data_dict['times'])-1):
      x0 = gt_traj_data_dict['world_T_car'][i][0]
      y0 = gt_traj_data_dict['world_T_car'][i][1]
      x1 = gt_traj_data_dict['world_T_car'][i+1][0]
      y1 = gt_traj_data_dict['world_T_car'][i+1][1]
      abs_driven_distance = abs_driven_distance + ((x0-x1)**2+(y0-y1)**2)**0.5
      time_driven_distance_map[gt_traj_data_dict["times"][i+1]] = abs_driven_distance
  return time_driven_distance_map

def compute_time_gt_pose_map(gt_traj_data_dict):
  time_gt_pose_map = {}
  for i in range(len(gt_traj_data_dict['times'])):
    x_gt = gt_traj_data_dict['world_T_laser'][i][0]
    y_gt = gt_traj_data_dict['world_T_laser'][i][1]
    psi_gt = gt_traj_data_dict['world_T_laser'][i][5]
    time = gt_traj_data_dict['times'][i]
    gt_pose = np.empty([3,1])
    gt_pose[0] = x_gt
    gt_pose[1] = y_gt
    gt_pose[2] = psi_gt
    time_gt_pose_map[time] = gt_pose
  return time_gt_pose_map

def compute_initialization_driven_dist(data_dict_out,time_driven_distance_map):
  initialization_driven_distance = float(0)
  for i in range(len(data_dict_out["loc_times"])):
    if(data_dict_out["track_initialized"][i]):
      if(not(data_dict_out["track_initialized"][i-1])):
        initialization_driven_distance = time_driven_distance_map[data_dict_out["loc_times"][i]]
        break
  return initialization_driven_distance

def compute_feasible_set_reliability_ratio(data_dict,time_gt_pose_map):
  in_feasible_counter = 0
  outside_feasible_counter = 0
  for i in range(len(data_dict['loc_pose_estimates'])):
    time = data_dict['loc_times'][i]
    # feasible set 
    if(len(data_dict['loc_pose_estimates'][i].feasible_set_box_corners) > 2):
      gt_pose = time_gt_pose_map[time]
      gt_position = np.empty([2,1])
      gt_position[0][0] = gt_pose[0]
      gt_position[1][0] = gt_pose[1]
      in_feasible_set = point_in_polygon_test(data_dict['loc_pose_estimates'][i].feasible_set_box_corners,gt_position)
      if(not(in_feasible_set)):
        outside_feasible_counter = outside_feasible_counter+1
      else:
        in_feasible_counter = in_feasible_counter+1
  ratio = float(in_feasible_counter)/float(outside_feasible_counter+in_feasible_counter)
  return ratio

def compute_consistent_set_reliability_ratio(data_dict,time_gt_pose_map):
  in_cosistent_counter = 0
  outside_consistent_counter = 0
  for i in range(len(data_dict['loc_pose_estimates'])):
    time = data_dict['loc_times'][i]
    # consistent set
    if(data_dict['track_initialized'][i]):
      if(len(data_dict['loc_pose_estimates'][i].consistent_set_poly_corners) > 2):
        gt_pose = time_gt_pose_map[time]
        gt_position = np.empty([2,1])
        gt_position[0][0] = gt_pose[0]
        gt_position[1][0] = gt_pose[1]
        in_consistent_polygon = point_in_polygon_test(data_dict['loc_pose_estimates'][i].consistent_set_poly_corners,gt_position)
        if(in_consistent_polygon):
          in_cosistent_counter = in_cosistent_counter+1
        else:
          outside_consistent_counter = outside_consistent_counter+1
  ratio = 0
  if(outside_consistent_counter+in_cosistent_counter > 0):
    ratio = float(in_cosistent_counter)/float(outside_consistent_counter+in_cosistent_counter)
  return ratio

def num_contracts_reset_switch(data_dict):
  exploration_region_contractions = 0
  resets = 0
  switches = 0
  for i in range(len(data_dict["loc_times"])):
    if(data_dict["contracted_exploration_region"][i]):
      exploration_region_contractions = exploration_region_contractions+1
    if(data_dict["switch_status"][i] == "reset"):
      resets = resets + 1
    if(data_dict["switch_status"][i] == "switch"):
      switches = switches + 1
  return exploration_region_contractions, resets, switches

def median_rot_rectangle_length(data_dict,rt_only_data_dict):
  # polygon sizes at localization time and at real-time for same postion:
  times = []
  loc_pose_estimates = []
  rt_pose_estimates = []
  for i in range(0,len(data_dict["loc_times"]),20):
    time = data_dict["loc_times"][i] # this is the refernce time to which we want have the real-time predited estimate and the localization estimate
    loc_pose_estimate = data_dict["loc_pose_estimates"][i]
    loc_poly_exists = False
    if(len(loc_pose_estimate.consistent_set_poly_corners) > 2):
      loc_poly_exists = True

    # look for the corresponding real-time estaimte
    rt_pose_estimate = PoseEstimate()
    for j in range(0,len(rt_only_data_dict["times"])):
      if(rt_only_data_dict["times"][j] == time):
        rt_pose_estimate = rt_only_data_dict["pose_estimates"][j]
        break
    rt_poly_exists = False
    if(len(rt_pose_estimate.consistent_set_poly_corners) > 2):
      rt_poly_exists = True

    if(loc_poly_exists and rt_poly_exists):
      # loc_pose_estimate and rt_pose_estimate correspond to the same frame
      times.append(time)
      loc_pose_estimates.append(loc_pose_estimate)
      rt_pose_estimates.append(rt_pose_estimate)
  # to each entry in times we have an entry in loc_pose_estimates and rt_pose_estimates at the same index
  loc_small_sides = []
  loc_large_sides = []
  rt_small_sides = []
  rt_large_sides = []
  for i in range(len(times)):
    time = times[i]
    loc_pose_estimate = loc_pose_estimates[i]
    rt_pose_estimate = rt_pose_estimates[i]
    # loc_pose_estimate side lengths
    loc_side1 = np.linalg.norm(loc_pose_estimate.consistent_set_rot_rect_corners[0]-loc_pose_estimate.consistent_set_rot_rect_corners[1])
    loc_side2 = np.linalg.norm(loc_pose_estimate.consistent_set_rot_rect_corners[1]-loc_pose_estimate.consistent_set_rot_rect_corners[2])
    if(loc_side1 < loc_side2):
      loc_small_sides.append(loc_side1)
      loc_large_sides.append(loc_side2)
    else:
      loc_small_sides.append(loc_side2)
      loc_large_sides.append(loc_side1)
    # rt_pose_estimate side lengths
    rt_side1 = np.linalg.norm(rt_pose_estimate.consistent_set_rot_rect_corners[0]-rt_pose_estimate.consistent_set_rot_rect_corners[1])
    rt_side2 = np.linalg.norm(rt_pose_estimate.consistent_set_rot_rect_corners[1]-rt_pose_estimate.consistent_set_rot_rect_corners[2])
    if(rt_side1 < rt_side2):
      rt_small_sides.append(rt_side1)
      rt_large_sides.append(rt_side2)
    else:
      rt_small_sides.append(rt_side2)
      rt_large_sides.append(rt_side1)
  median_loc_small_sides = np.median(loc_small_sides)
  media_loc_large_sides = np.median(loc_large_sides) 
  median_rt_small_sides = np.median(rt_small_sides) 
  median_rt_large_sides = np.median(rt_large_sides)
  return median_loc_small_sides,media_loc_large_sides,median_rt_small_sides,median_rt_large_sides

def median_feasible_set_lengths(data_dict):
  # polygon sizes at localization time and at real-time for same postion:
  times = []
  loc_pose_estimates = []
  for i in range(0,len(data_dict["loc_times"]),20):
    time = data_dict["loc_times"][i] # this is the refernce time to which we want have the real-time predited estimate and the localization estimate
    loc_pose_estimate = data_dict["loc_pose_estimates"][i]
    loc_poly_exists = False
    if(len(loc_pose_estimate.feasible_set_box_corners) > 2):
      loc_poly_exists = True

    if(loc_poly_exists):
      # loc_pose_estimate and rt_pose_estimate correspond to the same frame
      times.append(time)
      loc_pose_estimates.append(loc_pose_estimate)
  # to each entry in times we have an entry in loc_pose_estimates and rt_pose_estimates at the same index
  loc_small_sides = []
  loc_large_sides = []
  for i in range(len(times)):
    time = times[i]
    loc_pose_estimate = loc_pose_estimates[i]
    # loc_pose_estimate side lengths
    loc_side1 = np.linalg.norm(loc_pose_estimate.feasible_set_box_corners[0]-loc_pose_estimate.feasible_set_box_corners[1])
    loc_side2 = np.linalg.norm(loc_pose_estimate.feasible_set_box_corners[1]-loc_pose_estimate.feasible_set_box_corners[2])
    if(loc_side1 < loc_side2):
      loc_small_sides.append(loc_side1)
      loc_large_sides.append(loc_side2)
    else:
      loc_small_sides.append(loc_side2)
      loc_large_sides.append(loc_side1)
  median_loc_small_sides = np.median(loc_small_sides)
  media_loc_large_sides = np.median(loc_large_sides) 
  return median_loc_small_sides,media_loc_large_sides

def loc_rt_time_delays(data_dict):
  time_delays = []
  for i in range(len(data_dict["loc_times"])):
    if data_dict["track_initialized"][i]:
      rt_time = data_dict["real_times"][i]
      loc_time = data_dict["loc_times"][i]
      time_delays.append(abs(rt_time-loc_time))
  return np.median(time_delays)

def localization_error(data_dict,rt_only_data_dict,time_gt_pose_map):
  loc_transl_error = []
  loc_rot_error = []
  for i in range(len(data_dict['loc_pose_estimates'])):
    time = data_dict['loc_times'][i]
    if(data_dict['track_initialized'][i]):
      pose = data_dict['loc_pose_estimates'][i].pose
      gt_pose = time_gt_pose_map[time]
      transl_err = ((gt_pose[0] - pose[0])**2 + (gt_pose[1] - pose[1])**2)**0.5
      rot_err = abs(gt_pose[2] - pose[2])*180/math.pi
      loc_transl_error.append(transl_err)
      loc_rot_error.append(rot_err)
  # smooth rot error
  for i in range(len(loc_rot_error)):
    if(loc_rot_error[i] > 180):
      loc_rot_error[i] = loc_rot_error[i-1]
  # compute the error for real-time
  rt_transl_error = []
  rt_rot_error = []
  for i in range(len(rt_only_data_dict["times"])):
    time = rt_only_data_dict["times"][i]
    pose_estimate = rt_only_data_dict["pose_estimates"][i]
    if(len(pose_estimate.consistent_set_poly_corners) > 2):
      pose = pose_estimate.pose
      gt_pose = time_gt_pose_map[time]
      transl_err = ((gt_pose[0] - pose[0])**2 + (gt_pose[1] - pose[1])**2)**0.5
      rot_err = abs(gt_pose[2] - pose[2])*180/math.pi
      rt_transl_error.append(transl_err)
      rt_rot_error.append(rot_err)
  # smooth rot error
  for i in range(len(rt_rot_error)):
    if(rt_rot_error[i] > 180):
      rt_rot_error[i] = rt_rot_error[i-1]
  median_loc_transl_err = np.median(loc_transl_error)
  median_rt_transl_err = np.median(rt_transl_error)
  median_loc_rot_err = np.median(loc_rot_error)
  median_rt_rot_err = np.median(rt_rot_error)
  return median_loc_transl_err,median_loc_rot_err,median_rt_transl_err,median_rt_rot_err

def runtimes(data_dict):
  # vo
  median_vo_time = np.median(data_dict["vo_op_times"])
  # localization total
  median_full_loc_time = np.median(data_dict["full_loc_op_times"])
  # refined loc time
  median_refined_loc_time = np.median(data_dict["refined_loc_op_times"]) 
  # coarse loc time
  median_coarse_loc_time = np.median(data_dict["coarse_loc_op_times"])
  return median_vo_time,median_coarse_loc_time,median_refined_loc_time,median_full_loc_time

# functions for the stanford-esa diagram
def compute_true_error_feasible_error_pair(data_dict,time_gt_pose_map):
  true_error = []
  estimated_error = []
  for i in range(len(data_dict['loc_pose_estimates'])):
    time = data_dict['loc_times'][i]
    if(data_dict['track_initialized'][i]):
      pose = data_dict['loc_pose_estimates'][i].pose
      gt_pose = time_gt_pose_map[time]
      transl_err = ((gt_pose[0] - pose[0])**2 + (gt_pose[1] - pose[1])**2)**0.5
      loc_pose_estimate = data_dict["loc_pose_estimates"][i]
      if(len(loc_pose_estimate.feasible_set_box_corners) > 2):
        dist1 = np.linalg.norm(loc_pose_estimate.feasible_set_box_corners[0]-pose[:2])
        dist2 = np.linalg.norm(loc_pose_estimate.feasible_set_box_corners[1]-pose[:2])
        dist3 = np.linalg.norm(loc_pose_estimate.feasible_set_box_corners[2]-pose[:2])
        dist4 = np.linalg.norm(loc_pose_estimate.feasible_set_box_corners[3]-pose[:2])
        err_estim = max(dist1,dist2,dist3,dist4)
        true_error.append(transl_err)
        estimated_error.append(err_estim)
  return true_error,estimated_error

def compute_true_error_consistent_error_pair(data_dict,time_gt_pose_map):
  true_error = []
  estimated_error = []
  for i in range(len(data_dict['loc_pose_estimates'])):
    time = data_dict['loc_times'][i]
    if(data_dict['track_initialized'][i]):
      pose = data_dict['loc_pose_estimates'][i].pose
      gt_pose = time_gt_pose_map[time]
      transl_err = ((gt_pose[0] - pose[0])**2 + (gt_pose[1] - pose[1])**2)**0.5
      loc_pose_estimate = data_dict["loc_pose_estimates"][i]
      if(len(loc_pose_estimate.consistent_set_rot_rect_corners) > 2):
        dist1 = np.linalg.norm(loc_pose_estimate.consistent_set_rot_rect_corners[0]-pose[:2])
        dist2 = np.linalg.norm(loc_pose_estimate.consistent_set_rot_rect_corners[1]-pose[:2])
        dist3 = np.linalg.norm(loc_pose_estimate.consistent_set_rot_rect_corners[2]-pose[:2])
        dist4 = np.linalg.norm(loc_pose_estimate.consistent_set_rot_rect_corners[3]-pose[:2])
        err_estim = max(dist1,dist2,dist3,dist4)
        true_error.append(transl_err)
        estimated_error.append(err_estim)
  return true_error,estimated_error