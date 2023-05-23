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
# the user provides the folder where all the evaluation files are saved
# for the evaluation we need 3 files for each run
# the naming convention is: (run)_gt_traj.txt, (run)_out.txt, (run)_out_rt.txt
results_folder = file_names[0]
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
######################################

##############Perform Evaluation: get the average values for each run##################
init_dists = []
in_feasible_ratios = []
in_consistent_ratios = []
contract_nums = []
reset_nums = []
switch_nums = []
loc_small_sides_lengths = []
loc_large_sides_lengths = []
rt_small_sides_lengths = []
rt_large_sides_lengths = []
feasible_small_sides = []
feasible_large_sides = []
time_delays = []
loc_transl_errs = []
loc_rot_errs = []
rt_transl_errs = []
rt_rot_errs = []
vo_times = []
coarse_loc_times = []
refined_loc_times = []
full_loc_times = []
for id in all_runs_dicts.keys():
  data_dicts = all_runs_dicts[id]
  print(id,"times: ",len(data_dicts["gt"]["times"]))
  # get gt data
  time_driven_distance_map = compute_time_driven_distance_map(data_dicts["gt"])
  time_gt_pose_map = compute_time_gt_pose_map(data_dicts["gt"])
  # compute overall results
  init_dist = compute_initialization_driven_dist(data_dicts["out"],time_driven_distance_map)
  init_dists.append(init_dist)
  print(id,"init_dist: ",init_dist)

  in_feasible_ratio = compute_feasible_set_reliability_ratio(data_dicts["out"],time_gt_pose_map)
  in_feasible_ratios.append(in_feasible_ratio)
  print(id,"in_feasible_ratio: ",in_feasible_ratio)

  in_consistent_ratio = compute_consistent_set_reliability_ratio(data_dicts["out"],time_gt_pose_map)
  in_consistent_ratios.append(in_consistent_ratio)

  contracts, resets, switches = num_contracts_reset_switch(data_dicts["out"])
  contract_nums.append(contracts)
  reset_nums.append(resets)
  switch_nums.append(switches)

  loc_small_sides,loc_large_sides,rt_small_sides,rt_large_sides = median_rot_rectangle_length(data_dicts["out"],data_dicts["rt"])
  loc_small_sides_lengths.append(loc_small_sides)
  loc_large_sides_lengths.append(loc_large_sides)
  rt_small_sides_lengths.append(rt_small_sides)
  rt_large_sides_lengths.append(rt_large_sides)

  feasible_small_side, feasible_large_side = median_feasible_set_lengths(data_dicts["out"])
  feasible_small_sides.append(feasible_small_side)
  feasible_large_sides.append(feasible_large_side)

  time_delay = loc_rt_time_delays(data_dicts["out"])
  time_delays.append(time_delay)

  loc_transl_err,loc_rot_err,rt_transl_err,rt_rot_err = localization_error(data_dicts["out"],data_dicts["rt"],time_gt_pose_map)
  loc_transl_errs.append(loc_transl_err)
  loc_rot_errs.append(loc_rot_err)
  rt_transl_errs.append(rt_transl_err)
  rt_rot_errs.append(rt_rot_err)
  print(id,loc_transl_err,rt_transl_err,loc_rot_err,rt_rot_err)

  vo_time,coarse_loc_time,refined_loc_time,full_loc_time = runtimes(data_dicts["out"])
  vo_times.append(vo_time)
  coarse_loc_times.append(coarse_loc_time)
  refined_loc_times.append(refined_loc_time)
  full_loc_times.append(full_loc_time)
  print("---")

# Setup plot font
fontsize = 15
font = {'family' : 'normal',
        #'weight' : 'bold',
        'size'   : fontsize}
plt.rc('font', **font)

# draw the tables
avrg_loc_small_sides = np.average(loc_small_sides_lengths)
avrg_rt_small_sides = np.average(rt_small_sides_lengths)
avrg_loc_large_sides = np.average(loc_large_sides_lengths)
avrg_rt_large_sides = np.average(rt_large_sides_lengths)
avrg_time_delays = np.average(time_delays)
fig17, tab17 =plt.subplots(1,1,figsize=(12, 2))
data=[
  [str(avrg_loc_small_sides)+" m",str(avrg_rt_small_sides)+" m",str(avrg_loc_large_sides)+" m",str(avrg_rt_large_sides)+" m",str(avrg_time_delays)+" s"]]
column_labels=["avrg loc small","avrg rt small","avrg loc large","avrg rt large","avrg loc rt delay"]
row_labels=["val"]
tab17.axis('off')
tab17.table(cellText=data,colLabels=column_labels,rowLabels=row_labels,loc='center')
fig17.canvas.manager.set_window_title('loc and rt polygon comparison')

avrg_feasible_short_side = np.average(feasible_small_sides)
avrg_feasible_large_side = np.average(feasible_large_sides)
fig22, tab22 =plt.subplots(1,1,figsize=(12, 2))
data=[
  [str(avrg_feasible_short_side)+" m",str(avrg_loc_small_sides)+" m",str(avrg_feasible_large_side)+" m",str(avrg_loc_large_sides)+" m"]]
column_labels=["avrg feasible small","avrg consistent small","avrg feasible large","avrg consistent large"]
row_labels=["val"]
tab22.axis('off')
tab22.table(cellText=data,colLabels=column_labels,rowLabels=row_labels,loc='center')
fig22.canvas.manager.set_window_title('feasible vs consistent set size')


avrg_init = np.average(init_dists)
med_constracts = np.median(contract_nums)
med_resets = np.median(reset_nums)
med_switches = np.median(switch_nums)
avrg_feasible_rely = np.average(in_feasible_ratios)*100.0
avrg_consistent_rely = np.average(in_consistent_ratios)*100.0
fig18, tab18 =plt.subplots(1,1,figsize=(12, 2))
data=[
  [str(avrg_init)+" m",str(med_constracts),str(med_switches),str(med_resets),str(avrg_feasible_rely)+" %",str(avrg_consistent_rely)+" %"]]
column_labels=["avrg track init","median expl. reg. contracts"," median switches"," median resets","average feasible reli."," average consistent reli."]
row_labels=["val"]
tab18.axis('off')
tab18.table(cellText=data,colLabels=column_labels,rowLabels=row_labels,loc='center')
fig18.canvas.manager.set_window_title('init_contracts_resets_switches')

avrg_loc_transl_err = np.average(loc_transl_errs)
avrg_rt_transl_err = np.average(rt_transl_errs)
avrg_loc_rot_err = np.average(loc_rot_errs)
avrg_rt_rot_err = np.average(rt_rot_errs)
fig19, tab19 =plt.subplots(1,1,figsize=(12, 2))
data=[
  [str(avrg_loc_transl_err)+" m",str(avrg_rt_transl_err)+" m",str(avrg_loc_rot_err)+" deg",str(avrg_rt_rot_err)+" deg"]]
column_labels=["avrg loc transl","avrg rt tranls","avrg loc rot","avrg rt rot"]
row_labels=["val"]
tab19.axis('off')
tab19.table(cellText=data,colLabels=column_labels,rowLabels=row_labels,loc='center')
fig19.canvas.manager.set_window_title('median_localization_error')

avrg_vo_time = np.average(vo_times)
avrg_full_loc_time = np.average(full_loc_times)
avrg_coarse_loc_time = np.average(coarse_loc_times)
avrg_refined_loc_time = np.average(refined_loc_times)
fig21, tab21 =plt.subplots(1,1,figsize=(12, 2))
data=[
  [str(avrg_vo_time)+" s",str(avrg_coarse_loc_time)+" s",str(avrg_refined_loc_time)+" s",str(avrg_full_loc_time)+" s"]]
column_labels=["vo","coarse","refined","full loc"]
row_labels=["val"]
tab21.axis('off')
tab21.table(cellText=data,colLabels=column_labels,rowLabels=row_labels,loc='center')
fig21.canvas.manager.set_window_title('all_runtimes')

plt.show()