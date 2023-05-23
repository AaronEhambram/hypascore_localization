#!/usr/bin/python
import sys
import numpy as np
import matplotlib.pyplot as plt
import math
from gt_trajectory_dict_creator import*
from map_dict_creator import*
from read_cfg_file import*
from overall_dict_creator import*

file_names = (sys.argv)[1:]
cfg = read_cfg_file(file_names[0])
map_offset_strings = [0 for i in range(2)]
print("Evaluation file: ",cfg["results_file"])
print("Real-time only evaluation file: ",cfg["rt_only_results_file"])
print("GT trajectory file: ",cfg["gt_trajectory_file"])
print("Map file: ", cfg["map_file"])
print("x_offset for map: ", cfg["x_utm_offset"])
print("y_offset_for_map: ", cfg["y_utm_offset"])
print("Sample distances: ", cfg["sample_driven_distances"])

data_dict = get_data_dict(cfg["results_file"])
rt_only_data_dict = get_rt_only_data_dict(cfg["rt_only_results_file"])
gt_traj_data_dict = get_gt_traj_data_dict(cfg["gt_trajectory_file"])
map_dict = get_map_dict(cfg["map_file"],cfg["x_utm_offset"],cfg["y_utm_offset"])

# compute driven distance
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

# Setup plot font
fontsize = 15
font = {'family' : 'normal',
        #'weight' : 'bold',
        'size'   : fontsize}
plt.rc('font', **font)

###### localization time vs. real-time #######
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
driven_dists = []
for i in range(len(times)):
  time = times[i]
  loc_pose_estimate = loc_pose_estimates[i]
  rt_pose_estimate = rt_pose_estimates[i]
  driven_dist = time_driven_distance_map[time]
  driven_dists.append(driven_dist)
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
# compute the indexes that need to marked in the plot and the map-image
mark_idxs = []
for mark_dist in cfg["sample_driven_distances"]:
  smallest_dist = 10000000
  smallest_dist_idx = 0
  for i in range(len(driven_dists)):
    if smallest_dist > abs(driven_dists[i]-mark_dist):
      smallest_dist = abs(driven_dists[i]-mark_dist)
      smallest_dist_idx = i
  mark_idxs.append(smallest_dist_idx)
# compute the initialization point of the tracking 
initialization_driven_distance = float(0)
for i in range(len(data_dict["loc_times"])):
  if(data_dict["track_initialized"][i]):
    if(not(data_dict["track_initialized"][i-1])):
      initialization_driven_distance = time_driven_distance_map[data_dict["loc_times"][i]]
      break
# plot the rotrect sizes for loc and rt along driven distance
fig1, axs1 =plt.subplots(1,1,figsize=(12, 4))
axs1.set_xlabel('Driven distance in m')
axs1.plot(driven_dists,loc_large_sides, color='navy', label='Localization long side')
axs1.plot(driven_dists,rt_large_sides, color='cornflowerblue', label='Real-time long side')
axs1.plot(driven_dists,loc_small_sides, color='orange', label='Localization short side')
axs1.plot(driven_dists,rt_small_sides, color='orangered', label='Real-time short side')
axs1.set_ylabel('Length in m')
axs1.grid(color='darkgray', linestyle=':', linewidth=1)
axs1.set_xlim([0, driven_dists[-1]])
# mark the driven distances
for idx in mark_idxs:
   axs1.axvline(driven_dists[idx],linestyle=':',color='red')
   axs1.annotate(str(driven_dists[idx]).split('.')[0]+" m",(driven_dists[idx], 10), color='black')
axs1.axvline(initialization_driven_distance,linestyle=':',color='black')
axs1.annotate("initialization",(initialization_driven_distance, 10), color='black')
axs1.legend()
fig1.canvas.manager.set_window_title('Rotated rectangle hull side lengths')
# plot the polygons in map for mark_idxs
fig2, axs2 =plt.subplots(1,1,figsize=(10, 10))
for srf in map_dict["grounds"]:
  if srf.size > 0:
    srf_xs, srf_ys = zip(*srf)
    axs2.fill(srf_xs,srf_ys,color='black', label='Building footprints')
for idx in mark_idxs:
  rt_pose_estimate = rt_pose_estimates[idx]
  poly = rt_pose_estimate.consistent_set_poly_corners
  poly = np.append(poly,[poly[0]],axis=0)
  poly_xs, poly_ys = zip(*poly)
  axs2.plot(poly_xs,poly_ys,color='tab:orange',alpha=0.7,label='Rt polygon')
  loc_pose_estimate = loc_pose_estimates[idx]
  poly = loc_pose_estimate.consistent_set_poly_corners
  poly = np.append(poly,[poly[0]],axis=0)
  poly_xs, poly_ys = zip(*poly)
  axs2.plot(poly_xs,poly_ys,color='tab:blue',alpha=0.7,label='Loc polygon')
  axs2.annotate(str(driven_dists[idx]).split('.')[0]+" m",(poly_xs[0], poly_ys[0]), color='green')
axs2.set_xlabel('East in m')
axs2.set_ylabel('North in m')
axs2.axis('equal')

# time delay between loc-time and real-time
time_delays = []
driven_dist_diff = []
farthest_driven_distance = 0
farthest_driven_idx = 0
for i in range(len(data_dict["loc_times"])):
  if data_dict["track_initialized"][i]:
    rt_time = data_dict["real_times"][i]
    loc_time = data_dict["loc_times"][i]
    time_delays.append(rt_time-loc_time)
    dist = abs(time_driven_distance_map[rt_time]-time_driven_distance_map[loc_time])
    if(dist > farthest_driven_distance):
      farthest_driven_distance = dist
      farthest_driven_idx = i
# plot time differences
fig3, ax3 =plt.subplots(1,1,figsize=(12, 4))
ax3.set_xlabel('Time difference in s')
ax3.set_ylabel('Number of frames')
ax3.set_xlim([np.min(time_delays), np.max(time_delays)])
bins = 30
width = (np.max(time_delays)-np.min(time_delays))/bins*0.9
ax3.hist(time_delays, bins=bins,density=False,log=False,width=width)
ax3.grid(color='darkgray', linestyle=':', linewidth=1)
fig3.canvas.manager.set_window_title('loc and rt time differences')
# show example of loc polygon and rt polygon for the farthest driven distance difference
fig4, axs4 =plt.subplots(1,1,figsize=(10, 10))
for srf in map_dict["grounds"]:
  if srf.size > 0:
    srf_xs, srf_ys = zip(*srf)
    axs4.fill(srf_xs,srf_ys,color='black', label='Building footprints')
rt_pose_estimate = data_dict["rt_pose_estimates"][farthest_driven_idx]
poly = rt_pose_estimate.consistent_set_poly_corners
poly = np.append(poly,[poly[0]],axis=0)
poly_xs, poly_ys = zip(*poly)
axs4.plot(poly_xs,poly_ys,color='tab:orange',alpha=0.7,label='Rt polygon')
axs4.annotate(str(time_driven_distance_map[rt_pose_estimate.time]).split('.')[0]+" m",(poly_xs[0], poly_ys[0]), color='tab:orange')
loc_pose_estimate = data_dict["loc_pose_estimates"][farthest_driven_idx]
poly = loc_pose_estimate.consistent_set_poly_corners
poly = np.append(poly,[poly[0]],axis=0)
poly_xs, poly_ys = zip(*poly)
axs4.plot(poly_xs,poly_ys,color='tab:blue',alpha=0.7,label='Loc rotated box hull')
axs4.annotate(str(time_driven_distance_map[loc_pose_estimate.time]).split('.')[0]+" m",(poly_xs[0], poly_ys[0]), color='tab:blue')
axs4.set_xlabel('East in m')
axs4.set_ylabel('North in m')
axs4.axis('equal')


###### Particle stability Check #######
# get highest weight particle age and the tracking ages
times = []
driven_dists = []
driven_dists_tracking = []
highest_weight_particle_ages = []
highest_weight_particle_weights = []
tracking_ages = []
tracking_weights = []
reset_driven_distances = []
switches_driven_distances = []
initialization_driven_distance = float(0)
exploration_region_contract_dists = []
polygon_reliability = []
sine_angle_interval_width = []
for i in range(len(data_dict["loc_times"])):
  time = data_dict["loc_times"][i]
  driven_dist = time_driven_distance_map[time]
  high_weight_particle = data_dict["loc_highest_weight_particles"][i]
  tracked_particle = data_dict["loc_tracked_particles"][i]
  times.append(time)
  driven_dists.append(driven_dist)
  highest_weight_particle_ages.append(high_weight_particle.age)
  #highest_weight_particle_weights.append(high_weight_particle.weight)
  if(data_dict["track_initialized"][i]):
    driven_dists_tracking.append(driven_dist)
    tracking_ages.append(tracked_particle.age)
    tracking_weights.append(tracked_particle.weight)
    highest_weight_particle_weights.append(high_weight_particle.weight)
    polygon_reliability.append(data_dict["track_reliability"][i])
    sine_angle_width = data_dict["facade_sine_angle_hull"][i][0][1]-data_dict["facade_sine_angle_hull"][i][0][0]
    sine_angle_interval_width.append(sine_angle_width)
    if(not(data_dict["track_initialized"][i-1])):
      initialization_driven_distance = driven_dist
  #else:
    #tracking_ages.append(0)
    #tracking_weights.append(0)
    #highest_weight_particle_weights.append(0)
    #polygon_reliability.append(0)
    #sine_angle_interval_width.append(0)
  if(data_dict["switch_status"][i] == "reset"):
    reset_driven_distances.append(driven_dist)
  if(data_dict["switch_status"][i] == "switch"):
    switches_driven_distances.append(driven_dist)
  if(data_dict["contracted_exploration_region"][i]):
    exploration_region_contract_dists.append(driven_dist)
# plot the age 
fig5, axs5 =plt.subplots(1,1,figsize=(12, 4))
for contract_dist in exploration_region_contract_dists:
  axs5.axvline(contract_dist,linestyle='solid',color='silver',alpha=0.5)
  #axs5.annotate("contract",(contract_dist, 10), color='silver')
axs5.set_xlabel('Driven distance in m')
axs5.plot(driven_dists,highest_weight_particle_ages, color='orange', label='Highest weighted particle age')
axs5.plot(driven_dists_tracking,tracking_ages, color='tab:blue', label='Tracking age')
axs5.set_ylabel('Length in m')
axs5.grid(color='darkgray', linestyle=':', linewidth=1)
axs5.set_xlim([driven_dists[0], driven_dists[-1]])
for reset_dist in reset_driven_distances:
   axs5.axvline(reset_dist,linestyle=':',color='red')
   axs5.annotate("reset",(reset_dist, 10), color='red')
for switch_dist in switches_driven_distances:
   axs5.axvline(switch_dist,linestyle=':',color='orange')
   axs5.annotate("switch",(switch_dist, 10), color='orange')
axs5.axvline(initialization_driven_distance,linestyle=':',color='black')
axs5.annotate("initialization",(initialization_driven_distance, 10), color='black')
axs5.legend()
fig5.canvas.manager.set_window_title('particle ages')
# plot the weights 
fig6, axs6 =plt.subplots(1,1,figsize=(12, 4))
axs6.set_xlabel('Driven distance in m')
axs6.plot(driven_dists_tracking,highest_weight_particle_weights, color='orange', label='Highest weighted particle weight')
axs6.plot(driven_dists_tracking,tracking_weights, color='tab:blue', label='Tracking particle weight')
axs6.set_ylabel('Length in m')
axs6.grid(color='darkgray', linestyle=':', linewidth=1)
axs6.set_xlim([driven_dists[0], driven_dists[-1]])
for reset_dist in reset_driven_distances:
   axs6.axvline(reset_dist,linestyle=':',color='red')
   axs6.annotate("reset",(reset_dist, 10), color='red')
for switch_dist in switches_driven_distances:
   axs6.axvline(switch_dist,linestyle=':',color='orange')
   axs6.annotate("switch",(switch_dist, 10), color='orange')
axs6.axvline(initialization_driven_distance,linestyle=':',color='black')
axs6.annotate("initialization",(initialization_driven_distance, 10), color='black')
axs6.legend()
fig6.canvas.manager.set_window_title('particle weights')
# plot polygon reliability
fig7, axs7 =plt.subplots(1,1,figsize=(12, 4))
for contract_dist in exploration_region_contract_dists:
  axs7.axvline(contract_dist,linestyle='solid',color='silver',alpha=0.5)
axs7.set_xlabel('Driven distance in m')
axs7.plot(driven_dists_tracking,polygon_reliability, color='tab:blue', label='Track reliability')
axs7.set_ylabel('Reliability Probability')
axs7.grid(color='darkgray', linestyle=':', linewidth=1)
axs7.set_xlim([driven_dists[0], driven_dists[-1]])
for reset_dist in reset_driven_distances:
   axs7.axvline(reset_dist,linestyle=':',color='red')
   axs7.annotate("reset",(reset_dist, 0.5), color='red')
for switch_dist in switches_driven_distances:
   axs7.axvline(switch_dist,linestyle=':',color='orange')
   axs7.annotate("switch",(switch_dist, 0.5), color='orange')
axs7.axvline(initialization_driven_distance,linestyle=':',color='black')
axs7.annotate("initialization",(initialization_driven_distance, 0.5), color='black')
fig7.canvas.manager.set_window_title('Reliability')
# plot sine angle width of all observed facades
fig8, axs8 =plt.subplots(1,1,figsize=(12, 4))
for contract_dist in exploration_region_contract_dists:
  axs8.axvline(contract_dist,linestyle='solid',color='silver',alpha=0.5)
axs8.set_xlabel('Driven distance in m')
axs8.plot(driven_dists_tracking,sine_angle_interval_width, color='tab:blue', label='Sine angle width')
axs8.set_ylabel('Interval width')
axs8.grid(color='darkgray', linestyle=':', linewidth=1)
axs8.set_xlim([driven_dists[0], driven_dists[-1]])
for reset_dist in reset_driven_distances:
   axs8.axvline(reset_dist,linestyle=':',color='red')
   axs8.annotate("reset",(reset_dist, 0.5), color='red')
for switch_dist in switches_driven_distances:
   axs8.axvline(switch_dist,linestyle=':',color='orange')
   axs8.annotate("switch",(switch_dist, 0.5), color='orange')
axs8.axvline(initialization_driven_distance,linestyle=':',color='black')
axs8.annotate("initialization",(initialization_driven_distance, 0.5), color='black')
fig8.canvas.manager.set_window_title('Sine angle width')

#######Bound the exploration region########
# plot feasible set side lengths and rotated rect side lengths
feasible_small_sides = []
feasible_large_sides = []
feasible_rot = []
poly_small_sides = []
poly_large_sides = []
poly_rot = []
driven_dists_tracking = []
for i in range(len(data_dict["loc_times"])):
  time = data_dict["loc_times"][i]
  driven_dist = time_driven_distance_map[time]
  loc_pose_estimate = data_dict["loc_pose_estimates"][i]
  feasible_small_side, feasible_large_side = small_large_side_from_rect_corners(loc_pose_estimate.feasible_set_box_corners)
  feasible_small_sides.append(feasible_small_side)
  feasible_large_sides.append(feasible_large_side)
  feasible_rot_val = (loc_pose_estimate.feasible_set_angle[0][1]-loc_pose_estimate.feasible_set_angle[0][0])*180/3.14
  if(feasible_rot_val < 180):
    feasible_rot.append(feasible_rot_val)
  else:
    feasible_rot.append(feasible_rot_val)
  if(data_dict["track_initialized"][i]):
    driven_dists_tracking.append(driven_dist)
    poly_small_side, poly_large_side = small_large_side_from_rect_corners(loc_pose_estimate.consistent_set_rot_rect_corners)
    poly_small_sides.append(poly_small_side)
    poly_large_sides.append(poly_large_side)
    poly_rot_val = (loc_pose_estimate.consistent_set_angle[0][1]-loc_pose_estimate.consistent_set_angle[0][0])*180/3.14
    if(poly_rot_val < 180):
      poly_rot.append(poly_rot_val)
    else:
      poly_rot.append(poly_rot[-1])
    
fig9, axs9 =plt.subplots(1,1,figsize=(12, 4))
for contract_dist in exploration_region_contract_dists:
  axs9.axvline(contract_dist,linestyle='solid',color='silver',alpha=0.5)
axs9.set_xlabel('Driven distance in m')
axs9.plot(driven_dists,feasible_small_sides,color='orangered', label='Feasible set small side')
axs9.plot(driven_dists,feasible_large_sides,color='orange', label='Feasible set large side')
axs9.plot(driven_dists_tracking,poly_small_sides,color='blue', label='Consistent set small side')
axs9.plot(driven_dists_tracking,poly_large_sides,color='navy', label='Consistent set large side')
axs9.set_ylabel('Side width in m')
axs9.grid(color='darkgray', linestyle=':', linewidth=1)
axs9.set_xlim([driven_dists[0], driven_dists[-1]])
for reset_dist in reset_driven_distances:
   axs9.axvline(reset_dist,linestyle=':',color='red')
   axs9.annotate("reset",(reset_dist, 0.5), color='red')
for switch_dist in switches_driven_distances:
   axs9.axvline(switch_dist,linestyle=':',color='orange')
   axs9.annotate("switch",(switch_dist, 0.5), color='orange')
axs9.axvline(initialization_driven_distance,linestyle=':',color='black')
axs9.annotate("initialization",(initialization_driven_distance, 0.5), color='black')
axs9.legend()
fig9.canvas.manager.set_window_title('Feasible and Consistent set size')
# plot rotation
fig10, axs10 =plt.subplots(1,1,figsize=(12, 4))
for contract_dist in exploration_region_contract_dists:
  axs10.axvline(contract_dist,linestyle='solid',color='silver',alpha=0.5)
axs10.set_xlabel('Driven distance in m')
axs10.plot(driven_dists,feasible_rot,color='orangered', label='Feasible set')
axs10.plot(driven_dists_tracking,poly_rot,color='blue', label='Consistent set')
axs10.set_ylabel('Angle Interval width in ${()}^\circ$')
axs10.grid(color='darkgray', linestyle=':', linewidth=1)
axs10.set_xlim([driven_dists[0], driven_dists[-1]])
for reset_dist in reset_driven_distances:
   axs10.axvline(reset_dist,linestyle=':',color='red')
   axs10.annotate("reset",(reset_dist, 0.5), color='red')
for switch_dist in switches_driven_distances:
   axs10.axvline(switch_dist,linestyle=':',color='orange')
   axs10.annotate("switch",(switch_dist, 0.5), color='orange')
axs10.axvline(initialization_driven_distance,linestyle=':',color='black')
axs10.annotate("initialization",(initialization_driven_distance, 0.5), color='black')
axs10.legend()
fig10.canvas.manager.set_window_title('Feasible and Consistent set rotation')

######## localization error and reliability ############
# get gt Data 
time_gt_pose_map = {}
gt_x = []
gt_y = []
gt_driven_dists = []
gt_driven_dists.append(0)
dist = 0
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
  gt_x.append(x_gt)
  gt_y.append(y_gt)
  if(i > 0):
    x_gt_bef = gt_traj_data_dict['world_T_laser'][i-1][0]
    y_gt_bef = gt_traj_data_dict['world_T_laser'][i-1][1]
    dist = dist + ((x_gt-x_gt_bef)**2+(y_gt-y_gt_bef)**2)**0.5
    gt_driven_dists.append(dist)

# compute the error for localization time
loc_driven_dist = []
loc_transl_error = []
loc_rot_error = []
loc_x = []
loc_y = []
for i in range(len(data_dict['loc_pose_estimates'])):
  time = data_dict['loc_times'][i]
  if(data_dict['track_initialized'][i]):
    pose = data_dict['loc_pose_estimates'][i].pose
    gt_pose = time_gt_pose_map[time]
    loc_driven_dist.append(time_driven_distance_map[time])
    transl_err = ((gt_pose[0] - pose[0])**2 + (gt_pose[1] - pose[1])**2)**0.5
    rot_err = abs(gt_pose[2] - pose[2])*180/3.14
    loc_transl_error.append(transl_err)
    loc_rot_error.append(rot_err)
    loc_x.append(pose[0])
    loc_y.append(pose[1])
# smooth rot error
for i in range(len(loc_rot_error)):
  if(loc_rot_error[i] > 180):
    loc_rot_error[i] = loc_rot_error[i-1]
# compute the error for real-time
rt_driven_dist = []
rt_transl_error = []
rt_rot_error = []
rt_x = []
rt_y = []
for i in range(len(rt_only_data_dict["times"])):
  time = rt_only_data_dict["times"][i]
  pose_estimate = rt_only_data_dict["pose_estimates"][i]
  if(len(pose_estimate.consistent_set_poly_corners) > 2):
    pose = pose_estimate.pose
    gt_pose = time_gt_pose_map[time]
    rt_driven_dist.append(time_driven_distance_map[time])
    transl_err = ((gt_pose[0] - pose[0])**2 + (gt_pose[1] - pose[1])**2)**0.5
    rot_err = abs(gt_pose[2] - pose[2])*180/3.14
    rt_transl_error.append(transl_err)
    rt_rot_error.append(rot_err)
    rt_x.append(pose[0])
    rt_y.append(pose[1])
# smooth rot error
for i in range(len(rt_rot_error)):
  if(rt_rot_error[i] > 180):
    rt_rot_error[i] = rt_rot_error[i-1]
# plot error along drivem distance
fig11, axs11 =plt.subplots(1,1,figsize=(12, 4))
axs11.set_xlabel('Driven distance in m')
axs11.plot(loc_driven_dist,loc_transl_error,color='tab:blue', label='Localization time')
axs11.plot(rt_driven_dist,rt_transl_error,color='tab:orange', label='Real-time')
axs11.set_ylabel('Error in m')
axs11.grid(color='darkgray', linestyle=':', linewidth=1)
axs11.set_xlim([driven_dists[0], driven_dists[-1]])
axs11.axvline(initialization_driven_distance,linestyle=':',color='black')
axs11.annotate("initialization",(initialization_driven_distance, 0.5), color='black')
axs11.legend()
fig11.canvas.manager.set_window_title('Translation error')
# plot rotation error
fig12, axs12 =plt.subplots(1,1,figsize=(12, 4))
axs12.set_xlabel('Driven distance in m')
axs12.plot(loc_driven_dist,loc_rot_error,color='tab:blue', label='Localization time')
axs12.plot(rt_driven_dist,rt_rot_error,color='tab:orange', label='Real-time')
axs12.set_ylabel('Error in ${()}^\circ$')
axs12.grid(color='darkgray', linestyle=':', linewidth=1)
axs12.set_xlim([driven_dists[0], driven_dists[-1]])
axs12.axvline(initialization_driven_distance,linestyle=':',color='black')
axs12.annotate("initialization",(initialization_driven_distance, 0.5), color='black')
axs12.legend()
fig12.canvas.manager.set_window_title('Rotation error')
# plot the trajectories in the map
fig13, axs13 =plt.subplots(1,1,figsize=(10, 10))
for srf in map_dict["grounds"]:
  if srf.size > 0:
    srf_xs, srf_ys = zip(*srf)
    axs13.fill(srf_xs,srf_ys,color='black')
axs13.plot(gt_x,gt_y,color='blue',alpha=1.0,label='Ground truth',linewidth=5)
#axs13.plot(loc_x,loc_y,color='tab:blue',alpha=1.0,label='Localization time')
#axs13.plot(rt_x,rt_y,color='tab:orange',alpha=1.0,label='real-time')
# driven distance points:
#axs13.scatter(gt_x[0], gt_y[0], color='green')
#axs13.annotate("start",(gt_x[0], gt_y[0]), color='green')
#axs13.scatter(gt_x[-1], gt_y[-1], color='green')
#axs13.annotate("end",(gt_x[-1], gt_y[-1]), color='green')
#last_annotation_dist = gt_driven_dists[0]
#for i in range(0,len(gt_driven_dists)):
#   if gt_driven_dists[i]-last_annotation_dist >= 200: 
      #axs13.scatter(gt_x[i], gt_y[i], color='green')
      #axs13.annotate(str(gt_driven_dists[i]).split('.')[0],(gt_x[i], gt_y[i]), color='green')
      #last_annotation_dist = gt_driven_dists[i]
axs13.set_xlabel('East in m')
axs13.set_ylabel('North in m')
axs13.axis('equal')
#axs13.legend()
fig13.canvas.manager.set_window_title('Trajectories: loc, rt and gt')

# check if gt is inside consistent set
in_consistent_polygon_arr = []
in_feasible_set_arr = []
driven_dists_consistent_poly = []
driven_dists_feasible_set = []
problem_poses = []
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
      problem_poses.append(data_dict['loc_pose_estimates'][i])
    in_feasible_set_arr.append(in_feasible_set)
    driven_dists_feasible_set.append(time_driven_distance_map[time])
  # consistent set
  if(data_dict['track_initialized'][i]):
    if(len(data_dict['loc_pose_estimates'][i].consistent_set_poly_corners) > 2):
      gt_pose = time_gt_pose_map[time]
      gt_position = np.empty([2,1])
      gt_position[0][0] = gt_pose[0]
      gt_position[1][0] = gt_pose[1]
      in_consistent_polygon = point_in_polygon_test(data_dict['loc_pose_estimates'][i].consistent_set_poly_corners,gt_position)
      in_consistent_polygon_arr.append(in_consistent_polygon)
      driven_dists_consistent_poly.append(time_driven_distance_map[time])
fig14, axs14 =plt.subplots(1,1,figsize=(12, 4))
axs14.set_xlabel('Driven distance in m')
axs14.plot(driven_dists_consistent_poly,in_consistent_polygon_arr,color='tab:blue', label='Consistent set')
axs14.plot(driven_dists_feasible_set,in_feasible_set_arr,color='tab:orange', label='Feasible set')
axs14.set_ylabel('Inside set')
axs14.grid(color='darkgray', linestyle=':', linewidth=1)
axs14.set_xlim([driven_dists[0], driven_dists[-1]])
axs14.axvline(initialization_driven_distance,linestyle=':',color='black')
axs14.annotate("initialization",(initialization_driven_distance, 0.5), color='black')
axs14.legend()
fig14.canvas.manager.set_window_title('Inside feasible set Localization time')

fig15, axs15 =plt.subplots(1,1,figsize=(10, 10))
for srf in map_dict["grounds"]:
  if srf.size > 0:
    srf_xs, srf_ys = zip(*srf)
    axs15.fill(srf_xs,srf_ys,color='black')
for pose in problem_poses:
  poly = pose.feasible_set_box_corners
  poly_xs, poly_ys = zip(*poly)
  axs15.plot(poly_xs,poly_ys,color='tab:orange',alpha=0.7)
  poly = pose.consistent_set_poly_corners
  if(len(poly) > 2):
    poly_xs, poly_ys = zip(*poly)
    axs15.plot(poly_xs,poly_ys,color='tab:blue',alpha=0.7)
  gt_pose = time_gt_pose_map[pose.time]
  axs15.scatter(gt_pose[0],gt_pose[1],color='green',alpha=1.0)
axs15.plot(gt_x,gt_y,color='green',alpha=1.0,label='Ground truth')
axs15.set_xlabel('East in m')
axs15.set_ylabel('North in m')
axs15.axis('equal')
axs15.legend()
fig15.canvas.manager.set_window_title('Outside feasible set frames')

# get the runtimes for total localization 
runtimes = []
driven_dists = []
for i in range(len(data_dict["full_loc_op_times"])):
  time = data_dict["loc_times"][i]
  runtime = data_dict["full_loc_op_times"][i]
  driven_dist = time_driven_distance_map[time]
  runtimes.append(runtime)
  driven_dists.append(driven_dist)
# plot runtimes
fig16, axs16 =plt.subplots(1,1,figsize=(12, 4))
axs16.set_xlabel('Driven distance in m')
axs16.plot(driven_dists,runtimes,color='tab:blue', label='Localization runtime')
axs16.set_ylabel('Runtime in s')
axs16.grid(color='darkgray', linestyle=':', linewidth=1)
axs16.set_xlim([driven_dists[0], driven_dists[-1]])
fig16.canvas.manager.set_window_title('Full Loclization Runtime')

####### Average output tables ########
# short and large sides of the polygon
avrg_loc_small_sides = np.average(loc_small_sides)
avrg_loc_large_sides = np.average(loc_large_sides) 
avrg_rt_small_sides = np.average(rt_small_sides) 
avrg_rt_large_sides = np.average(rt_large_sides)
avrg_time_delays = np.average(time_delays)
# draw the table
fig17, tab17 =plt.subplots(1,1,figsize=(12, 2))
data=[
  [str(avrg_loc_small_sides)+" m",str(avrg_rt_small_sides)+" m",str(avrg_loc_large_sides)+" m",str(avrg_rt_large_sides)+" m",str(avrg_time_delays)+" s"]]
column_labels=["avrg loc small","avrg rt small","avrg loc large","avrg rt large","avrg loc rt delay"]
row_labels=["val"]
tab17.axis('off')
tab17.table(cellText=data,colLabels=column_labels,rowLabels=row_labels,loc='center')
fig17.canvas.manager.set_window_title('loc and rt polygon comparison')

# driven distance until intialization of tracking, number of exploration region contractions, number or resets, number of switches
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
# draw the table
fig18, tab18 =plt.subplots(1,1,figsize=(12, 2))
data=[
  [str(initialization_driven_distance)+" m",str(exploration_region_contractions),str(resets),str(switches)]]
column_labels=["track init","expl. reg. contracts","resets","switches"]
row_labels=["val"]
tab18.axis('off')
tab18.table(cellText=data,colLabels=column_labels,rowLabels=row_labels,loc='center')
fig18.canvas.manager.set_window_title('iniit_contracts_resets_switches')

# avrg localization error (transl, rot)
median_loc_transl_err = np.median(loc_transl_error)
median_rt_transl_err = np.median(rt_transl_error)
median_loc_rot_err = np.median(loc_rot_error)
median_rt_rot_err = np.median(rt_rot_error)
# draw table
fig19, tab19 =plt.subplots(1,1,figsize=(12, 2))
data=[
  [str(median_loc_transl_err)+" m",str(median_rt_transl_err)+" m",str(median_loc_rot_err)+" deg",str(median_rt_rot_err)+" deg"]]
column_labels=["median loc transl","median rt tranls","median loc rot","median rt rot"]
row_labels=["val"]
tab19.axis('off')
tab19.table(cellText=data,colLabels=column_labels,rowLabels=row_labels,loc='center')
fig19.canvas.manager.set_window_title('median_localization_error')

# ratio of in consistent-set and feasible set 
# consistent set
in_consistent_polygon_arr
in_consistent_poly_counter = float(0)
for inside in in_consistent_polygon_arr:
  if(inside):
    in_consistent_poly_counter = in_consistent_poly_counter + 1.0
in_consistent_polygon_ratio = in_consistent_poly_counter/float(len(in_consistent_polygon_arr))
# feasible set
in_feasible_set_arr
in_feasible_set_counter = float(0)
for inside in in_feasible_set_arr:
  if(inside):
    in_feasible_set_counter = in_feasible_set_counter + 1.0
in_feasible_set_ratio = in_feasible_set_counter/float(len(in_feasible_set_arr))
# draw table
fig20, tab20 =plt.subplots(1,1,figsize=(12, 2))
data=[
  [str(in_feasible_set_ratio*100.0)+" %",str(in_consistent_polygon_ratio*100.0)+" %"]]
column_labels=["in feasible set","in consistent set"]
row_labels=["val"]
tab20.axis('off')
tab20.table(cellText=data,colLabels=column_labels,rowLabels=row_labels,loc='center')
fig20.canvas.manager.set_window_title('gt_inside_feasible_consistent_sets')

# runtimes
# vo
median_vo_time = np.median(data_dict["vo_op_times"])
# localization total
median_full_loc_time = np.median(data_dict["full_loc_op_times"])
# refined loc time
median_refined_loc_time = np.median(data_dict["refined_loc_op_times"]) 
# coarse loc time
median_coarse_loc_time = np.median(data_dict["coarse_loc_op_times"])
# draw table
fig21, tab21 =plt.subplots(1,1,figsize=(12, 2))
data=[
  [str(median_vo_time)+" s",str(median_full_loc_time)+" s",str(median_coarse_loc_time)+" s",str(median_refined_loc_time)+" s"]]
column_labels=["vo","full loc","coarse","refined"]
row_labels=["val"]
tab21.axis('off')
tab21.table(cellText=data,colLabels=column_labels,rowLabels=row_labels,loc='center')
fig21.canvas.manager.set_window_title('all_runtimes')

plt.show()