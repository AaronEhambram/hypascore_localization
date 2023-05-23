#!/usr/bin/python
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math
from scipy.signal import savgol_filter
from tracking_dict_creator import*
from gt_trajectory_dict_creator import*
from map_dict_creator import*
from read_cfg_file import*

file_names = (sys.argv)[1:]
cfg = read_cfg_file(file_names[0])
map_offset_strings = [0 for i in range(2)]
print("Evaluation file: ",cfg["results_file"])
print("GT trajectory file: ",cfg["gt_trajectory_file"])
print("Map file: ", cfg["map_file"])
print("x_offset for map: ", cfg["x_utm_offset"])
print("y_offset_for_map: ", cfg["y_utm_offset"])
print("Sample distances: ", cfg["sample_driven_distances"])

data_dict = get_data_dict(cfg["results_file"])
gt_traj_data_dict = get_gt_traj_data_dict(cfg["gt_trajectory_file"])
map_dict = get_map_dict(cfg["map_file"],cfg["x_utm_offset"],cfg["y_utm_offset"])

# compute driven distance
driven_distance_dict={"times" : [], "driven_distance" : []}
driven_distance_dict["times"].append(gt_traj_data_dict["times"][0])
driven_distance_dict["driven_distance"].append(0)
abs_driven_distance = 0
for i in range(0,len(gt_traj_data_dict['times'])-1):
    x0 = gt_traj_data_dict['world_T_car'][i][0]
    y0 = gt_traj_data_dict['world_T_car'][i][1]
    x1 = gt_traj_data_dict['world_T_car'][i+1][0]
    y1 = gt_traj_data_dict['world_T_car'][i+1][1]
    driven_distance_dict["times"].append(gt_traj_data_dict["times"][i+1])
    abs_driven_distance = abs_driven_distance + ((x0-x1)**2+(y0-y1)**2)**0.5
    driven_distance_dict["driven_distance"].append(abs_driven_distance)

# filter the driven distances only for the times in in data_dict
driven_distances = []
gt_traj_x = []
gt_traj_y = []
gt_traj_rot = []
for time in data_dict["times"]:
    for i in range(0,len(gt_traj_data_dict['times'])):
        if(gt_traj_data_dict['times'][i] == time):
            driven_distances.append(driven_distance_dict["driven_distance"][i])
            gt_traj_x.append(gt_traj_data_dict['world_T_laser'][i][0])
            gt_traj_y.append(gt_traj_data_dict['world_T_laser'][i][1])
            gt_traj_rot.append(gt_traj_data_dict['world_T_laser'][i][5])


# rotated rectangle sizes -> filter large and small size
rotrect_small = []
rotrect_large = []
for rotect_size in data_dict["rotrect_size"]:
    if(rotect_size[0] < rotect_size[1]):
        rotrect_small.append(rotect_size[0])
        rotrect_large.append(rotect_size[1])
    else:
        rotrect_small.append(rotect_size[1])
        rotrect_large.append(rotect_size[0])

# polygons that should be plotted
polygons = []
rotrect_hulls = []
driven_distance_for_polygon = []
np_driven_dist = np.asarray(driven_distances)
for dist in cfg["sample_driven_distances"]:
   idx = (np.abs(np_driven_dist - dist)).argmin()
   polygons.append(data_dict["polygon_corners"][idx])
   rotrect_hulls.append(data_dict["rotrect_hull_corners"][idx])
   driven_distance_for_polygon.append(driven_distances[idx])

# RMSE and rotation error for bounded and unbounded
t_rmse_bounded = []
x_bounded_traj = []
y_bounded_traj = []
t_rmse_unbounded = []
x_unbounded_traj = []
y_unbounded_traj = []
rot_err_bounded = []
rot_err_unbounded = []
for i in range(len(driven_distances)):
   # gt data
   x_gt = gt_traj_x[i]
   y_gt = gt_traj_y[i]
   rot_gt = gt_traj_rot[i]
   # bounded 
   x_bounded = data_dict["bounded_opt_pose"][i][0]
   y_bounded = data_dict["bounded_opt_pose"][i][1]
   rot_bounded = data_dict["bounded_opt_pose"][i][2]
   x_bounded_traj.append(x_bounded)
   y_bounded_traj.append(y_bounded)
   t_rmse_bounded.append(((x_gt-x_bounded)**2+(y_gt-y_bounded)**2)**0.5)
   rot_err_bounded.append(abs(rot_gt-rot_bounded)*180/3.14)
   #unbounded
   x_unbounded = data_dict["unbounded_opt_pose"][i][0]
   y_unbounded = data_dict["unbounded_opt_pose"][i][1]
   rot_unbounded = data_dict["unbounded_opt_pose"][i][2]
   x_unbounded_traj.append(x_unbounded)
   y_unbounded_traj.append(y_unbounded)
   t_rmse_unbounded.append(((x_gt-x_unbounded)**2+(y_gt-y_unbounded)**2)**0.5)
   rot_err_unbounded.append(abs(rot_gt-rot_unbounded)*180/3.14)
# clean 360 peaks in the rotation errors (happends when the rotation close to 0)
for i in range(len(rot_err_bounded)):
  if rot_err_bounded[i] > 180:
    rot_err_bounded[i] = rot_err_bounded[i-1]
for i in range(len(rot_err_unbounded)):
  if rot_err_unbounded[i] > 180:
    rot_err_unbounded[i] = rot_err_unbounded[i-1]

############# generate the plots
fontsize = 15
font = {'family' : 'normal',
        #'weight' : 'bold',
        'size'   : fontsize}
plt.rc('font', **font)

# rotrect sizes
fig, axs =plt.subplots(1,1,figsize=(12, 4))
axs.set_xlabel('Driven distance in m')
axs.plot(driven_distances,rotrect_large, color='navy', label='Long side')
axs.plot(driven_distances,rotrect_small, color='orange', label='Short side')
# insert vertical lines for polygon plots
for dist in driven_distance_for_polygon:
   axs.axvline(dist,linestyle=':',color='red')
   axs.annotate(str(dist).split('.')[0]+" m",(dist, 10), color='black')
axs.set_ylabel('Length in m')
axs.grid(color='darkgray', linestyle=':', linewidth=1)
axs.set_ylim([0, 15])
axs.set_xlim([driven_distances[0], driven_distances[-1]])
axs.legend()
fig.canvas.manager.set_window_title('Rotated rectangle hull side lengths')

# orientation interval
# hull size of the rotation angle
ori_hulls = []
for hull in data_dict["angle_interval"]:
  size = hull[1]-hull[0]
  ori_hulls.append(180/3.14*size)

fig2, axs2 =plt.subplots(1,1,figsize=(12, 4))
axs2.set_xlabel('Driven distance in m')
axs2.plot(driven_distances, ori_hulls, color='darkred')
axs2.set_ylabel('Width in $^\circ$')
axs2.grid(color='darkgray', linestyle=':', linewidth=1)
axs2.set_ylim([0, 10])
axs2.set_xlim([driven_distances[0], driven_distances[-1]])
fig2.canvas.manager.set_window_title('Interval angle width')

# draw gt trajectory with exemplary polygons and rotrect-boxes
fig3, axs3 =plt.subplots(1,1,figsize=(10, 10))
axs3.plot(gt_traj_x, gt_traj_y, color='green', label='Ground truth')
# driven distance points:
axs3.scatter(gt_traj_x[0], gt_traj_y[0], color='green')
axs3.annotate("start",(gt_traj_x[0], gt_traj_y[0]), color='green')
#last_annotation_dist = driven_distances[0]
#for i in range(0,len(driven_distances)):
#   if driven_distances[i]-last_annotation_dist >= 100: 
#      axs3.scatter(gt_traj_x[i], gt_traj_y[i], color='green')
#      axs3.annotate(str(driven_distances[i]).split('.')[0],(gt_traj_x[i], gt_traj_y[i]), color='green')
#      last_annotation_dist = driven_distances[i]
axs3.scatter(gt_traj_x[-1], gt_traj_y[-1], color='green')
axs3.annotate("end",(gt_traj_x[-1], gt_traj_y[-1]), color='green')
# polygons:
i = 0
for poly in polygons:
  poly_xs, poly_ys = zip(*poly)
  axs3.plot(poly_xs,poly_ys,color='blue',label='Polygon')
  axs3.annotate(str(driven_distance_for_polygon[i]).split('.')[0]+" m",(poly_xs[0], poly_ys[0]), color='green')
  i=i+1
# rotated rectangle hull
for rotrect in rotrect_hulls:
  rotect_closed = rotrect
  rotect_closed = np.append(rotect_closed,[rotect_closed[0]],axis=0)
  rotrect_xs, rotrect_ys = zip(*rotect_closed)
  axs3.plot(rotrect_xs,rotrect_ys,color='orange',alpha=0.7,label='Rotated box hull')
# draw the map
for srf in map_dict["grounds"]:
  if srf.size > 0:
    srf_xs, srf_ys = zip(*srf)
    axs3.fill(srf_xs,srf_ys,color='black', label='Building footprints')
axs3.set_xlabel('East in m')
axs3.set_ylabel('North in m')
axs3.axis('equal')
fig3.canvas.manager.set_window_title('Trajectory with exemplary polygons and rotated rectangle hulls')

# plot the number of seen facades
fig4, axs4 =plt.subplots(1,1,figsize=(12, 4))
#axs4.plot(driven_distances,data_dict["num_associated_facades"])
bar_width = 25
distance_bin = []
start_idx = 0
end_idx = 0
bar_driven_distance = []
bar_seen_facades_num = []
for i in range(len(driven_distances)):
   dist = driven_distances[i]
   distance_bin.append(dist)
   if(distance_bin[-1] - distance_bin[0] >= bar_width):
      end_idx = i
      bar_driven_distance.append(distance_bin[0] + (distance_bin[-1] - distance_bin[0])/2.0)
      bar_seen_facades_num.append(np.average(data_dict["num_associated_facades"][start_idx:end_idx]))
      start_idx = end_idx+1
      distance_bin = []
   
#axs4.bar(driven_distances,data_dict["num_associated_facades"],width=0.5,edgecolor='blue')
axs4.bar(bar_driven_distance,bar_seen_facades_num,width=bar_width-2)
axs4.set_xlabel('Driven distance in m')
axs4.set_ylabel('Number of associated facades')
axs4.grid(color='darkgray', linestyle=':', linewidth=1)
axs4.set_xlim([driven_distances[0], driven_distances[-1]])
fig4.canvas.manager.set_window_title('Number of seen facades')

# plot ratio of associated points
fig5, axs5 =plt.subplots(1,1,figsize=(12, 4))
axs5.plot(driven_distances,data_dict["ratio_associated"])
axs5.set_xlabel('Driven distance in m')
axs5.set_ylabel('Ratio of associated points')
axs5.grid(color='darkgray', linestyle=':', linewidth=1)
axs5.set_xlim([driven_distances[0], driven_distances[-1]])
fig5.canvas.manager.set_window_title('Ratio of associated points')

# plot RMSE for bounded and unbounded optimization
fig6, axs6 =plt.subplots(1,1,figsize=(12, 4))
axs6.plot(driven_distances,t_rmse_unbounded,color='royalblue', label='Unbounded')
axs6.plot(driven_distances,t_rmse_bounded,color='orange', label='Bounded')
axs6.set_xlabel('Driven distance in m')
axs6.set_ylabel('Error in m')
axs6.grid(color='darkgray', linestyle=':', linewidth=1)
axs6.legend()
axs6.set_xlim([driven_distances[0], driven_distances[-1]])
fig6.canvas.manager.set_window_title('Translation RMSE after bounded and unbounded optimization')

# plot rotation error for bounded and unbounded case
fig7, axs7 =plt.subplots(1,1,figsize=(12, 4))
axs7.plot(driven_distances,rot_err_unbounded,color='royalblue', label='Unbounded')
axs7.plot(driven_distances,rot_err_bounded,color='orange', label='Bounded')
axs7.set_xlabel('Driven distance in m')
axs7.set_ylabel('Error in ${}^{\circ}$')
axs7.grid(color='darkgray', linestyle=':', linewidth=1)
axs7.legend()
axs7.set_xlim([driven_distances[0], driven_distances[-1]])
fig7.canvas.manager.set_window_title('Rotation error after bounded and unbounded optimization')

# plot trajectories in map 
fig8, axs8 =plt.subplots(1,1,figsize=(10, 10))
axs8.plot(gt_traj_x, gt_traj_y, color='green', label='Ground truth')
# draw the map
for srf in map_dict["grounds"]:
  if srf.size > 0:
    srf_xs, srf_ys = zip(*srf)
    axs8.fill(srf_xs,srf_ys,color='black')
# driven distance points:
axs8.scatter(gt_traj_x[0], gt_traj_y[0], color='green')
axs8.annotate("start",(gt_traj_x[0], gt_traj_y[0]), color='green')
last_annotation_dist = driven_distances[0]
for i in range(0,len(driven_distances)):
   if driven_distances[i]-last_annotation_dist >= 100: 
      axs8.scatter(gt_traj_x[i], gt_traj_y[i], color='green')
      axs8.annotate(str(driven_distances[i]).split('.')[0],(gt_traj_x[i], gt_traj_y[i]), color='green')
      last_annotation_dist = driven_distances[i]
axs8.scatter(gt_traj_x[-1], gt_traj_y[-1], color='green')
axs8.annotate("end",(gt_traj_x[-1], gt_traj_y[-1]), color='green')
# draw unbounded trajectory
axs8.plot(x_unbounded_traj, y_unbounded_traj, color='royalblue', label='Unbounded')
# draw bounded trajectory
axs8.plot(x_bounded_traj, y_bounded_traj, color='orange', label='Bounded')
axs8.legend()
axs8.set_xlabel('East in m')
axs8.set_ylabel('North in m')
axs8.axis('equal')
fig8.canvas.manager.set_window_title('Bounded and unbounded optimized trajectories in map')

# plot the full runtime
fig9, axs9 =plt.subplots(1,1,figsize=(12, 4))
axs9.plot(driven_distances,data_dict["op_time_full"])
axs9.set_xlim([driven_distances[0], driven_distances[-1]])
axs9.set_xlabel('Driven distance in m')
axs9.set_ylabel('Runtime in s')
fig9.canvas.manager.set_window_title('Full runtimes for refined localization')

###### generate overall evaluation tables with average results
# set membership related evaluation: arvg length long side, avrg length short side, avrg rotation width
long_side_rotrect_avrg = np.average(rotrect_large)
short_side_rotrect_avrg = np.average(rotrect_small)
rot_width_avrg = np.average(ori_hulls)
# draw the table
fig10, tab10 =plt.subplots(1,1,figsize=(12, 2))
data=[
  [str(long_side_rotrect_avrg)+" m",str(short_side_rotrect_avrg)+" m",str(rot_width_avrg)+"deg"]]
column_labels=["Avrg long side width"," Arvg small side width","rot width"]
row_labels=["values"]
tab10.axis('off')
tab10.table(cellText=data,colLabels=column_labels,rowLabels=row_labels,loc='center')
fig10.canvas.manager.set_window_title('Set-membership evaluation')

# facade association related: avrg seen facades, avrg ration of point associations
avrg_seen_facades = np.average(data_dict["num_associated_facades"])
avrg_association_ratio = np.average(data_dict["ratio_associated"])
# draw the table
fig11, tab11 =plt.subplots(1,1,figsize=(12, 2))
data=[
  [str(avrg_seen_facades),str(avrg_association_ratio*100.0)+" %"]]
column_labels=["Avrg num of seen facades"," Avrg point association ratio"]
row_labels=["values"]
tab11.axis('off')
tab11.table(cellText=data,colLabels=column_labels,rowLabels=row_labels,loc='center')
fig11.canvas.manager.set_window_title('Facade association evaluation')

# comparison bounded and unbounded: avrg RMSE, avrg rot error, largest error transl, largest err rot
# bounded 
avrg_rmse_bounded = np.average(t_rmse_bounded)
avrg_rot_err_bounded = np.average(rot_err_bounded)
largest_rmse_bounded = np.max(t_rmse_bounded)
largest_rot_err_bounded = np.max(rot_err_bounded)
# unbounded
avrg_rmse_unbounded = np.average(t_rmse_unbounded)
avrg_rot_err_unbounded = np.average(rot_err_unbounded)
largest_rmse_unbounded = np.max(t_rmse_unbounded)
largest_rot_err_unbounded = np.max(rot_err_unbounded)
# draw the table
fig12, tab12 =plt.subplots(1,1,figsize=(12, 2))
data=[
  [str(avrg_rmse_bounded)+" m",str(avrg_rot_err_bounded)+" m",str(largest_rmse_bounded)+" m",str(largest_rot_err_bounded)+" deg"],
  [str(avrg_rmse_unbounded)+" m",str(avrg_rot_err_unbounded)+" m",str(largest_rmse_unbounded)+" m",str(largest_rot_err_unbounded)+" deg"]]
column_labels=["avrg rmse","avrg rot err","largest rmse","largest rot err"]
row_labels=["bounded","unbounded"]
tab12.axis('off')
tab12.table(cellText=data,colLabels=column_labels,rowLabels=row_labels,loc='center')
fig12.canvas.manager.set_window_title('Optimization comparison: bounded and unbounded')

# computation time 
avrg_op_time_full = np.average(data_dict["op_time_full"])
avrg_op_time_iHT = np.average(data_dict["op_time_iHT"])
avrg_op_time_polygon = np.average(data_dict["op_time_polygon"])
avrg_op_time_association = np.average(data_dict["op_time_assocation"])
# draw the table
fig13, tab13 =plt.subplots(1,1,figsize=(12, 2))
data=[
  [str(avrg_op_time_full)+" s",str(avrg_op_time_iHT)+" s",str(avrg_op_time_polygon)+" s",str(avrg_op_time_association)+" s"]]
column_labels=["full","iHT","polygon","association"]
row_labels=["times"]
tab13.axis('off')
tab13.table(cellText=data,colLabels=column_labels,rowLabels=row_labels,loc='center')
fig13.canvas.manager.set_window_title('Compuation time')

plt.show()