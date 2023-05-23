#!/usr/bin/python
import numpy as np

class Frame:
  def __init__(self, time, is_keyframe, odom_computed, preprocessing_duration, detect_track_duration, stereo_odom_opt_duration,num_seen_landmarks):
    self.time = time
    self.is_keyframe = is_keyframe
    self.lcam_prev_p_lcam_cur = {}
    self.odom_computed = odom_computed
    self.preprocessing_duration = preprocessing_duration
    self.detect_track_duration = detect_track_duration
    self.stereo_odom_opt_duration = stereo_odom_opt_duration
    self.num_seen_landmarks = num_seen_landmarks
    self.seen_landmarks = []
    self.observations = {} # landmark id to 3x2 np array
    self.reprojection_errors = {} # 3x1 np array -> 1. left x, 2. left y, 3. right x

class Landmark:
  def __init__(self, id, is_good, seen_times):
    self.id = id
    self.is_good = is_good
    self.seen_times = seen_times

def observation(x_lb,x_ub,y_lb,y_ub,z_lb,z_ub):
  # create observation container 
  obs = np.empty([3,2])
  obs[0][0] = x_lb
  obs[0][1] = x_ub
  obs[1][0] = y_lb
  obs[1][1] = y_ub
  obs[2][0] = z_lb
  obs[2][1] = z_ub
  return obs

def reprojection_error(lx_err,ly_err,rx_err):
  err = np.empty([3])
  err[0] = lx_err
  err[1] = ly_err
  err[2] = rx_err
  return err

def relative_pose(x_lb,x_ub,y_lb,y_ub,z_lb,z_ub,phi_lb,phi_ub,teta_lb,teta_ub,psi_lb,psi_ub):
  # create pose container 
  pose = np.empty([6,2])
  pose[0][0] = x_lb
  pose[0][1] = x_ub
  pose[1][0] = y_lb
  pose[1][1] = y_ub
  pose[2][0] = z_lb
  pose[2][1] = z_ub
  pose[3][0] = phi_lb
  pose[3][1] = phi_ub
  pose[4][0] = teta_lb
  pose[4][1] = teta_ub
  pose[5][0] = psi_lb
  pose[5][1] = psi_ub
  return pose

def create_data_dict():
  data_dict = {
    "Frames" : [],
    "Landmarks": []
  }
  return data_dict

def fill_frame_data_timestamp(frames,file):
  line = file.readline()
  while line:
    # read frame lines until "---" is found
    if(line == "---\n"):
      break
    else:
      # all infos of the line refer to one frame
      line_splitted = line.split(" ")
      # get time
      f_time = float(line_splitted[0])
      # keyframe or not
      f_is_keyframe = False
      if(line_splitted[1] == "k"):
        # it is a keyframe
        f_is_keyframe = True
      elif(line_splitted[1] == "f"):
        # normal frame
        f_is_keyframe = False
      else:
        print("ERROR with is_keyframe!!")
      # 12 entries to laser_prev_p_laser_cur
      f_relative_pose = relative_pose(line_splitted[2],line_splitted[3],line_splitted[4],line_splitted[5],line_splitted[6],line_splitted[7],line_splitted[8],line_splitted[9],line_splitted[10],line_splitted[11],line_splitted[12],line_splitted[13])
      # odom computed
      f_odom_computed = False
      if(line_splitted[14] == "True"):
        # it is compted
        f_odom_computed = True
      elif(line_splitted[14] == "False"):
        # not computed
        f_odom_computed = False
      else:
        print("ERROR with odom_computed!!") 
      #preproscessing time
      f_preprocessing_time = float(line_splitted[15])
      # detect/extraction duration
      f_detect_extract_time = float(line_splitted[16])
      # frame-to-frame optimization
      f_f2fodom_time = float(line_splitted[17])
      # num of seen landmarks
      f_num_seen_landmarks = float(line_splitted[18])
      # landmark id and measurements
      f_seen_landmarks = []
      f_landmark_obs = {}
      f_reproj_errors = {}
      i = 19
      f = Frame(f_time, f_is_keyframe, f_odom_computed, f_preprocessing_time, f_detect_extract_time, f_f2fodom_time,f_num_seen_landmarks)
      while i < len(line_splitted)-10:
        l_id = float(line_splitted[i])
        l_x_lb = float(line_splitted[i+1])
        l_x_ub = float(line_splitted[i+2])
        l_y_lb = float(line_splitted[i+3])
        l_y_ub = float(line_splitted[i+4])
        l_z_lb = float(line_splitted[i+5])
        l_z_ub = float(line_splitted[i+6])
        lx_err = float(line_splitted[i+7])
        ly_err = float(line_splitted[i+8])
        rx_err = float(line_splitted[i+9])
        i = i+10
        f_seen_landmarks.append(l_id)
        obs = observation(l_x_lb,l_x_ub,l_y_lb,l_y_ub,l_z_lb,l_z_ub)
        reproj_err = reprojection_error(lx_err,ly_err,rx_err)
        f_landmark_obs[l_id] = obs
        f_reproj_errors[l_id] = reproj_err
      f.lcam_prev_p_lcam_cur = f_relative_pose
      f.seen_landmarks = f_seen_landmarks
      f.observations = f_landmark_obs
      f.reprojection_errors = f_reproj_errors
      frames.append(f)
      line = file.readline()

def fill_landmarks(landmarks,file):
  line = file.readline()
  while line:
    line_splitted = line.split(" ")
    # a full line is dedicted for one landmark
    l_id = line_splitted[0]
    l_is_good = True
    if(line_splitted[1] == "good"):
      l_is_good = True
    elif(line_splitted[1] == "bad"):
      l_is_good = False
    else:

      print("Error in landmark good/bad: ", line_splitted[1])
    i = 2
    l_seen_times = []
    while i<len(line_splitted)-1:
      time = float(line_splitted[i])
      l_seen_times.append(time)
      i = i+1
    # get new line
    line = file.readline()
    l = Landmark(l_id,l_is_good,l_seen_times)
    landmarks.append(l)

def get_data_dict(file_path):
  data_dict = create_data_dict()
  file = open(file_path,"r")
  if file.mode == "r":
    # get the frame infos -> until "---\n"
    fill_frame_data_timestamp(data_dict["Frames"],file)
    # file points to the line "---\n", now the landmarks follow
    fill_landmarks(data_dict["Landmarks"],file)
  return data_dict