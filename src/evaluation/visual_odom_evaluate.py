#!/usr/bin/python
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math
from scipy.signal import savgol_filter
from gt_trajectory_dict_creator import*
from visual_odom_dict_creator import*

file_names = (sys.argv)[1:]
print("gt traj-file: ", file_names[0])
print("visual odom results file: ", file_names[1])

data_dict = get_data_dict(file_names[1])
gt_traj_data_dict = get_gt_traj_data_dict(file_names[0])

# compute driven distance
driven_distance_dict={"times" : [], "driven_distance" : []}
driven_distance_dict["times"].append(gt_traj_data_dict["times"][0])
driven_distance_dict["driven_distance"].append(0)
time_driven_distance_map = {}
time_driven_distance_map[gt_traj_data_dict["times"][0]] = 0.0
abs_driven_distance = 0
for i in range(0,len(gt_traj_data_dict['times'])-1):
    x0 = gt_traj_data_dict['world_T_car'][i][0]
    y0 = gt_traj_data_dict['world_T_car'][i][1]
    x1 = gt_traj_data_dict['world_T_car'][i+1][0]
    y1 = gt_traj_data_dict['world_T_car'][i+1][1]
    driven_distance_dict["times"].append(gt_traj_data_dict["times"][i+1])
    abs_driven_distance = abs_driven_distance + ((x0-x1)**2+(y0-y1)**2)**0.5
    driven_distance_dict["driven_distance"].append(abs_driven_distance)
    time_driven_distance_map[gt_traj_data_dict["times"][i+1]] = abs_driven_distance

# filter the driven distances only for the times in in data_dict
driven_distances = []
gt_traj_x = []
gt_traj_y = []
gt_traj_rot = []
for frame in data_dict["Frames"]:
  time = frame.time
  for i in range(0,len(gt_traj_data_dict['times'])):
    if(gt_traj_data_dict['times'][i] == time):
      driven_distances.append(driven_distance_dict["driven_distance"][i])
      gt_traj_x.append(gt_traj_data_dict['world_T_lcam'][i][0])
      gt_traj_y.append(gt_traj_data_dict['world_T_lcam'][i][1])
      gt_traj_rot.append(gt_traj_data_dict['world_T_lcam'][i][5])

# divide frame into the windows
windows = []
w = []
for frame in data_dict["Frames"]:
  if(frame.is_keyframe):
    w.append(frame)
    windows.append(w)
    w = []
    w.append(frame)
  else:
    w.append(frame)
windows.pop(0)
# get the lengths of the windows 
window_lengths = []
for w in windows:
  window_lengths.append(time_driven_distance_map[w[-1].time] - time_driven_distance_map[w[0].time])

# tracking length of landmarks
landmark_tracking_lengths = []
for l in data_dict["Landmarks"]:
  seen_driven_distances = []
  for seen_time in l.seen_times:
    seen_driven_distances.append(time_driven_distance_map[seen_time])
  first_seen_dist = np.min(seen_driven_distances)
  last_seen_dist = np.max(seen_driven_distances)
  landmark_tracking_lengths.append(last_seen_dist-first_seen_dist)

# number of connected frame per landmark
landmark_seen_number = []
for l in data_dict["Landmarks"]:
  landmark_seen_number.append(len(l.seen_times))

# number of seen landmark in frame
seen_landmarks_in_frame = []
for frame in data_dict["Frames"]:
  seen_landmarks_in_frame.append(frame.num_seen_landmarks)

# Uncertainty of interval stereo reconstruction
x_obs_sizes = []
y_obs_sizes = []
z_obs_sizes = []
for frame in data_dict["Frames"]:
  for l in frame.seen_landmarks:
    obs = frame.observations[l]
    if(not(math.isinf(obs[0][1])) and not(math.isinf(obs[0][0])) and 
       not(math.isinf(obs[1][1])) and not(math.isinf(obs[1][0])) and 
       not(math.isinf(obs[2][1])) and not(math.isinf(obs[2][0])) and 
       not(math.isnan(obs[0][1])) and not(math.isnan(obs[0][0])) and 
       not(math.isnan(obs[1][1])) and not(math.isnan(obs[1][0])) and 
       not(math.isnan(obs[2][1])) and not(math.isnan(obs[2][0]))):
      x_obs_sizes.append(float(obs[0][1]-obs[0][0]))
      y_obs_sizes.append(float(obs[1][1]-obs[1][0]))
      z_obs_sizes.append(float(obs[2][1]-obs[2][0]))

# reprojection errors
lx_repr_err = []
ly_repr_err = []
rx_repr_err = []
for frame in data_dict["Frames"]:
  for l in frame.seen_landmarks:
    repr_err = frame.reprojection_errors[l]
    lx_repr_err.append(repr_err[0])
    ly_repr_err.append(repr_err[1])
    rx_repr_err.append(repr_err[2])

# compute full operation time for front-end
front_end_op_time = []
preprocessing_times = []
track_duration = []
track_detect_duration = []
stereo_odom_opt_times = []
for frame in data_dict["Frames"]:
  front_end_op_time.append(frame.preprocessing_duration+frame.detect_track_duration+frame.stereo_odom_opt_duration)
  preprocessing_times.append(frame.preprocessing_duration)
  if(frame.is_keyframe):
    track_detect_duration.append(frame.detect_track_duration)
  else:
    track_duration.append(frame.detect_track_duration)
  stereo_odom_opt_times.append(frame.stereo_odom_opt_duration)

# extract invalid landmarks
invalid_landmarks = []
for l in data_dict["Landmarks"]:
  if not(l.is_good):
    invalid_landmarks.append(l)

# extract keyframes
keyframes = []
for frame in data_dict["Frames"]:
  if(frame.is_keyframe):
    keyframes.append(frame)

# relative pose uncertainties
x_pose_uncertainty = []
y_pose_uncertainty = []
z_pose_uncertainty = []
phi_pose_uncertainty = []
teta_pose_uncertainty = []
psi_pose_uncertainty = []
x_pose = []
y_pose = []
z_pose = []
phi_pose = []
teta_pose = []
psi_pose = []
odom_dist = []
for frame in data_dict["Frames"]:
  if frame.odom_computed:
    p = frame.lcam_prev_p_lcam_cur
    x_pose_uncertainty.append(abs(p[0][1]-p[0][0]))
    x_pose.append((p[0][1]+p[0][0])/2.0)
    y_pose_uncertainty.append(abs(p[1][1]-p[1][0]))
    y_pose.append((p[1][1]+p[1][0])/2.0)
    z_pose_uncertainty.append(abs(p[2][1]-p[2][0]))
    z_pose.append((p[2][1]+p[2][0])/2.0)
    phi_pose_uncertainty.append(abs(p[3][1]-p[3][0])*180.0/3.14)
    phi_pose.append((p[3][1]+p[3][0])/2.0*180.0/3.14)
    teta_pose_uncertainty.append(abs(p[4][1]-p[4][0])*180.0/3.14)
    teta_pose.append((p[4][1]+p[4][0])/2.0*180.0/3.14)
    psi_pose_uncertainty.append(abs(p[5][1]-p[5][0])*180.0/3.14)
    psi_pose.append((p[5][1]+p[5][0])/2.0*180.0/3.14)
    odom_dist.append((((p[0][1]+p[0][0])/2.0)**2+((p[2][1]+p[2][0])/2.0)**2)**0.5)

############# generate the plots
fontsize = 15
font = {'family' : 'normal',
        #'weight' : 'bold',
        'size'   : fontsize}
plt.rc('font', **font)

# plot window length histogram
fig, axs =plt.subplots(1,1,figsize=(12, 4))
axs.set_xlabel('Window length in m')
axs.set_ylabel('Number of windows')
axs.set_xlim([np.min(window_lengths), np.max(window_lengths)])
bins = 30
width = (np.max(window_lengths)-np.min(window_lengths))/bins*0.9
axs.hist(window_lengths, bins=bins,density=False,log=True,width=width)
axs.grid(color='darkgray', linestyle=':', linewidth=1)
fig.canvas.manager.set_window_title('Window lengths')

# plot landmark tracking lengths as histigram
fig1, axs1 =plt.subplots(1,1,figsize=(12, 4))
axs1.set_xlabel('Tracking length in m')
axs1.set_ylabel('Number of landmarks')
axs1.set_xlim([np.min(landmark_tracking_lengths), np.max(landmark_tracking_lengths)])
bins1 = 30
width1 = (np.max(landmark_tracking_lengths)-np.min(landmark_tracking_lengths))/bins1*0.9
axs1.hist(landmark_tracking_lengths, bins=bins1,density=False,log=True,width=width1)
axs1.grid(color='darkgray', linestyle=':', linewidth=1)
fig1.canvas.manager.set_window_title('Landmark tracking lengths')

# plot number of seen times of a landmark
#fig2, axs2 =plt.subplots(1,1,figsize=(12, 4))
#axs2.set_xlabel('Number of seen times')
#axs2.set_ylabel('Number of landmarks')
#axs2.set_xlim([np.min(landmark_seen_number), np.max(landmark_seen_number)])
#bins2 = 30
#width2 = (np.max(landmark_seen_number)-np.min(landmark_seen_number))/bins2*0.9
#axs2.hist(landmark_seen_number, bins=bins2,density=False,log=True,width=width2)
#axs2.grid(color='darkgray', linestyle=':', linewidth=1)
#fig2.canvas.manager.set_window_title('Landmark number of seen times')

# plot number of seen landmarks in frame
#fig3, axs3 =plt.subplots(1,1,figsize=(12, 4))
#axs3.set_xlabel('Number of seen landmarks')
#axs3.set_ylabel('Number of frames')
#axs3.set_xlim([np.min(seen_landmarks_in_frame), np.max(seen_landmarks_in_frame)])
#bins3 = 50
#width3 = (np.max(seen_landmarks_in_frame)-np.min(seen_landmarks_in_frame))/bins3*0.9
#axs3.hist(seen_landmarks_in_frame, bins=bins3,density=False,log=False,width=width3)
#axs3.grid(color='darkgray', linestyle=':', linewidth=1)
#fig3.canvas.manager.set_window_title('Number of seen landmarks in frame')

# plot the observation uncertainties
fig4, axs4 =plt.subplots(1,1,figsize=(12, 4))
observation_uncertainties = [x_obs_sizes,y_obs_sizes,z_obs_sizes]
axs4.set_ylabel('Direction')
axs4.set_xlabel('Interval width in m')
axs4.boxplot(observation_uncertainties,showfliers=False,vert=False)
axs4.set_yticklabels(['x','y','z'])
axs4.grid(color='darkgray', linestyle=':', linewidth=1)
fig4.canvas.manager.set_window_title('Uncertainties of the feature observations (residuals)')

# plot the observation uncertainties
fig12, axs12 =plt.subplots(1,1,figsize=(12, 4))
reprojection_errors = [lx_repr_err,ly_repr_err,rx_repr_err]
axs12.set_ylabel('Pixel direction')
axs12.set_xlabel('Pixels')
axs12.boxplot(reprojection_errors,showfliers=False,vert=False)
axs12.set_yticklabels(['left x','left y','right x'])
axs12.grid(color='darkgray', linestyle=':', linewidth=1)
fig12.canvas.manager.set_window_title('Reprojection errors after optimization')

# plot the front-end runtime
fig5, axs5 =plt.subplots(1,1,figsize=(12, 4))
axs5.plot(driven_distances,front_end_op_time)
axs5.set_xlim([driven_distances[0], driven_distances[-1]])
axs5.grid(color='darkgray', linestyle=':', linewidth=1)
axs5.set_xlabel('Driven distance in m')
axs5.set_ylabel('Runtime in s')
fig5.canvas.manager.set_window_title('Full runtimes for Front-End')

# plot relative pose along driven trajectory
fig8, axs8 =plt.subplots(1,1,figsize=(12, 4))
axs8.plot(driven_distances[0:len(odom_dist)],odom_dist)
axs8.set_xlim([driven_distances[0], driven_distances[-1]])
axs8.set_xlabel('Driven distance in m')
axs8.set_ylabel('Distance in m')
axs8.grid(color='darkgray', linestyle=':', linewidth=1)
fig8.canvas.manager.set_window_title('Driven distance between frames based on odometry')

# plot translation uncertainty along trajectory
fig9, axs9 =plt.subplots(1,1,figsize=(12, 4))
axs9.plot(driven_distances[0:len(x_pose_uncertainty)],x_pose_uncertainty,label='x',color='royalblue')
axs9.plot(driven_distances[0:len(z_pose_uncertainty)],z_pose_uncertainty,label='z',color='orange')
axs9.grid(color='darkgray', linestyle=':', linewidth=1)
axs9.set_xlim([driven_distances[0], driven_distances[-1]])
axs9.set_xlabel('Driven distance in m')
axs9.set_ylabel('Uncertainty in m')
axs9.legend()
fig9.canvas.manager.set_window_title('Odometry translation uncertainty')

# plot rotation uncertainty along trajectory
fig10, axs10 =plt.subplots(1,1,figsize=(12, 4))
axs10.plot(driven_distances[0:len(teta_pose_uncertainty)],teta_pose_uncertainty,color='darkred')
axs10.grid(color='darkgray', linestyle=':', linewidth=1)
axs10.set_xlim([driven_distances[0], driven_distances[-1]])
axs10.set_xlabel('Driven distance in m')
axs10.set_ylabel('Uncertainty in $()^\circ$')
fig10.canvas.manager.set_window_title('Odometry rotation uncertainty')

########## Table for Front-End
kf_seen_landmarks_amount = []
for kf in keyframes:
  kf_seen_landmarks_amount = kf.num_seen_landmarks
avrg_detected_landmarks = np.average(kf_seen_landmarks_amount)
num_invalid_landmarks = len(invalid_landmarks)
avrg_driven_dist_window = np.average(window_lengths)
avrg_tracking_length = np.average(landmark_tracking_lengths)
# draw the table
fig6, tab6 =plt.subplots(1,1,figsize=(12, 2))
data=[
  [str(avrg_driven_dist_window)+" m",str(avrg_detected_landmarks),str(avrg_tracking_length)+" m"]]
column_labels=["avrg window length","Arvg detected landmarks","Avrg tracking lenth"]
row_labels=["values"]
tab6.axis('off')
tab6.table(cellText=data,colLabels=column_labels,rowLabels=row_labels,loc='center')
fig6.canvas.manager.set_window_title('Front-End evaluation')

# landmark 
invalid_landmarks_ratio = float(num_invalid_landmarks)/float(len(data_dict["Landmarks"]))
pixel_reproj_err = []
for i in range(len(lx_repr_err)):
  pixel_reproj_err.append((lx_repr_err[i]**2+ly_repr_err[i]**2)**0.5)
avrg_reproj_err = np.average(pixel_reproj_err)
avrg_x_reconstr_uncert = np.median(x_obs_sizes)
avrg_y_reconstr_uncert = np.median(y_obs_sizes)
avrg_z_reconstr_uncert = np.median(z_obs_sizes)
#draw table
fig13, tab13 =plt.subplots(1,1,figsize=(12, 2))
data=[
  [str(invalid_landmarks_ratio*100.0)+' %',str(avrg_reproj_err)+' px',str(avrg_x_reconstr_uncert)+' m',str(avrg_y_reconstr_uncert)+' m',str(avrg_z_reconstr_uncert)+' m']]
column_labels=["invalid landmarks ratio","avrg reproj err","x lanmdark uncert","y lanmdark uncert","z lanmdark uncert"]
row_labels=["values"]
tab13.axis('off')
tab13.table(cellText=data,colLabels=column_labels,rowLabels=row_labels,loc='center')
fig13.canvas.manager.set_window_title('back-end evaluation')

# odometry results -> avrg driven distance between frames, average odoemtry uncertainty
avrg_odom_dist = np.average(odom_dist)
avrg_x_odom_uncertainty = np.median(x_pose_uncertainty) 
avrg_z_odom_uncertainty = np.median(z_pose_uncertainty)
avrg_teta_odom_uncertainty = np.median(teta_pose_uncertainty)
#draw table
fig11, tab11 =plt.subplots(1,1,figsize=(12, 2))
data=[
  [str(avrg_odom_dist)+' m',str(avrg_x_odom_uncertainty)+' m',str(avrg_z_odom_uncertainty)+' m',str(avrg_teta_odom_uncertainty)+' deg']]
column_labels=["avrg odom f2f dist","x odom uncert","z odom uncert","rot odom uncert"]
row_labels=["values"]
tab11.axis('off')
tab11.table(cellText=data,colLabels=column_labels,rowLabels=row_labels,loc='center')
fig11.canvas.manager.set_window_title('back-end evaluation')

# operation times
avrg_full_op_time = np.average(front_end_op_time)
avrg_preprocessing_times = np.average(preprocessing_times)
avrg_track_duration = np.average(track_duration)
avrg_track_detect_duration = np.average(track_detect_duration)
avrg_stereo_odom_opt_times = np.average(stereo_odom_opt_times)
# draw the table
fig7, tab7 =plt.subplots(1,1,figsize=(12, 2))
data=[
  [str(avrg_full_op_time)+' s',str(avrg_preprocessing_times)+' s',str(avrg_track_duration)+' s',str(avrg_track_detect_duration)+' s',str(avrg_stereo_odom_opt_times)+' s']]
column_labels=["full front-end","preprocessing","track only","track+detect","stereo-odom opt"]
row_labels=["values"]
tab7.axis('off')
tab7.table(cellText=data,colLabels=column_labels,rowLabels=row_labels,loc='center')
fig7.canvas.manager.set_window_title('runtimes evaluation')


plt.show()