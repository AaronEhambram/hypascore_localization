#!/usr/bin/python
import sys
import numpy as np
import matplotlib.pyplot as plt
import math
from os import listdir
from os.path import isfile, join
from gt_trajectory_dict_creator import*
from map_dict_creator import*
from overall_dict_creator import*

############### Get the data #########################
file_names = (sys.argv)[1:]
# the user provides multiple folders where all the evaluation files are saved
# for the evaluation we need 3 files for each run
# the naming convention is: (run)_gt_traj.txt, (run)_out.txt, (run)_out_rt.txt
all_dicts = {}
for results_folder in file_names:
  print('Folder Name: ', results_folder)
  # check how many runs are inside the folder
  onlyfiles = [f for f in listdir(results_folder) if isfile(join(results_folder, f))]
  runs = {}
  for i in range(0,len(onlyfiles)):
    file_name = onlyfiles[i]
    fnsplit = file_name.split("_")
    if((fnsplit[0]).isdigit()):
      id = int(fnsplit[0])
      if not(id in runs):
        runs[id] = {"gt" : "", "rt" : "", "out" : ""}
      if(len(fnsplit) == 3):
        if(fnsplit[-1] == "traj.txt"):
          runs[id]["gt"] = join(results_folder, file_name)
        elif(fnsplit[-1] == "rt.txt"):
          runs[id]["rt"] = join(results_folder, file_name)
        else:
          print("ERROR")
      elif(len(fnsplit) == 2):
        if(fnsplit[-1] == "out.txt"):
          runs[id]["out"] = join(results_folder, file_name)
        else:
          print("ERROR")
      else:
        print("ERROR")
  # load the results dictionaries for each run
  all_runs_dicts = {}
  for id in runs.keys():
    gt_file = runs[id]["gt"]
    rt_file = runs[id]["rt"]
    out_file = runs[id]["out"]
    if(not(gt_file == "" or rt_file == "" or out_file == "")):
      all_runs_dicts[id] = {"gt" : {}, "rt" : {}, "out" : {}}
      all_runs_dicts[id]["out"] = get_data_dict(out_file)
      all_runs_dicts[id]["rt"] = get_rt_only_data_dict(rt_file)
      all_runs_dicts[id]["gt"] = get_gt_traj_data_dict(gt_file)
      print("load the run with id: ", id)
  print("Total number of runs: ", len(all_runs_dicts.keys()))
  all_dicts[results_folder] = all_runs_dicts
######################################

###########Organize the data for the Stanford Diagram############
# get the true error (error between most refined loclization estimate and ground truth)
# get estimated error as half averge rotated rectangle length
# feasible-set
true_err_feas_arr = []
estim_err_feas_arr = []
# consistent-set
true_err_cons_arr = [] 
estim_err_cons_arr = []
for folder_name in all_dicts.keys():
  all_runs_dicts = all_dicts[folder_name]
  for id in all_runs_dicts.keys():
    data_dicts = all_runs_dicts[id]
    print(id,"times: ",len(data_dicts["gt"]["times"]))
    # get gt data
    time_gt_pose_map = compute_time_gt_pose_map(data_dicts["gt"])
    # get feasible set pairs 
    true_err_feas, estim_err_feas = compute_true_error_feasible_error_pair(data_dicts["out"],time_gt_pose_map)
    true_err_feas_arr.extend(true_err_feas)
    estim_err_feas_arr.extend(estim_err_feas)
    # get consistent set pairs
    true_err_cons, estim_err_cons = compute_true_error_consistent_error_pair(data_dicts["out"],time_gt_pose_map)
    true_err_cons_arr.extend(true_err_cons)
    estim_err_cons_arr.extend(estim_err_cons)

# Setup plot font
fontsize = 15
font = {'family' : 'normal',
        #'weight' : 'bold',
        'size'   : fontsize}
plt.rc('font', **font)

# plot the rotrect sizes for loc and rt along driven distance
fig1, axs1 =plt.subplots(1,1,figsize=(10, 10))
axs1.set_xlabel('True error in m')
axs1.set_ylabel('Estimated error in m')
axs1.scatter(true_err_cons_arr,estim_err_cons_arr, color='tab:orange',marker="o" ,s=12, alpha=0.9, label='Consistent set')
axs1.scatter(true_err_feas_arr,estim_err_feas_arr, color='tab:blue',marker="o",s=12, alpha=0.9, label='Feasible set')
axs1.grid(color='darkgray', linestyle=':', linewidth=1)
axs1.legend(loc="lower right")
axs1.axis('equal')
axs1.set_xlim([0, np.max(estim_err_feas_arr)])
axs1.set_ylim([0, np.max(estim_err_feas_arr)])
#axs1.set_xlim([0, 20])
#axs1.set_ylim([0, 20])
axs1.plot([0, 1], [0, 1], transform=axs1.transAxes, ls='--', color='black', linewidth=1.5)
fig1.canvas.manager.set_window_title('Stanford Diagram')

# plot the rotrect sizes for loc and rt along driven distance
fig2, axs2 =plt.subplots(1,1,figsize=(10, 10))
axs2.set_xlabel('True error in m')
axs2.set_ylabel('Estimated error in m')
axs2.scatter(true_err_cons_arr,estim_err_cons_arr, color='tab:orange',marker="o" ,s=12, alpha=0.9, label='Consistent set')
axs2.scatter(true_err_feas_arr,estim_err_feas_arr, color='tab:blue',marker="o",s=12, alpha=0.9, label='Feasible set')
axs2.grid(color='darkgray', linestyle=':', linewidth=1)
axs2.legend(loc="lower right")
axs2.axis('equal')
axs2.set_xlim([0, 10])
axs2.set_ylim([0, 10])
axs2.plot([0, 1], [0, 1], transform=axs1.transAxes, ls='--', color='black', linewidth=1.5)
fig2.canvas.manager.set_window_title('Stanford Diagram zoom')

plt.show()