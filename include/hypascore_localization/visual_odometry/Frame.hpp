#ifndef FRAME_H_
#define FRAME_H_
#include "hypascore_localization/visual_odometry/Observation.hpp"
#include "hypascore_localization/visual_odometry/Landmark.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include "Eigen/Dense"
#include <map>
#include "ibex.h"
#include <opencv2/imgproc/imgproc.hpp>

namespace Graph
{
class Landmark;
class Observation;
class Frame
{
  public:
  double timestamp_;
  std::vector<Landmark*> seen_landmarks_;
  bool is_keyframe_ = false;
  Eigen::Affine3d car_prev_T_car_cur_ = Eigen::Affine3d::Identity(); 
  ibex::IntervalVector laser_prev_p_laser_cur = ibex::IntervalVector(6,0);
  bool computed_interval_odometry_ = false;

  // localization results
  Eigen::Affine2d map_T_laser = Eigen::Affine2d::Identity(); 
  ibex::IntervalVector gl__map_p_laser = ibex::IntervalVector(3); 
  ibex::IntervalVector track__map_p_laser = ibex::IntervalVector(3); 

  // processing times for evaluation
  double preprocessing_duration_, detect_track_duration_, stereo_odom_opt_duration_;
  ibex::IntervalVector lcam_prev_p_lcam_cur = ibex::IntervalVector(6,0);
};
}

#endif