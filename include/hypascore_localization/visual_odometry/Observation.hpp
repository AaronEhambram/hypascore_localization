#ifndef OBSERVATION_H_
#define OBSERVATION_H_
#include "hypascore_localization/visual_odometry/Frame.hpp"
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
class Frame;
class Observation
{
  public:
  cv::Point2f lf_;
  cv::Point2f rf_;
  Eigen::Vector3d reprojection_error_;
  ibex::IntervalVector lcam_box_;
  Eigen::Vector3d lcam_point_; 
  Frame* frame_;
  Landmark* landmark_;
};
}

#endif