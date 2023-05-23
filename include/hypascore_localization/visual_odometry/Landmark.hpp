#ifndef LANDMARK_H_
#define LANDMARK_H_
#include "hypascore_localization/visual_odometry/Frame.hpp"
#include "hypascore_localization/visual_odometry/Observation.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include "Eigen/Dense"
#include <map>
#include "ibex.h"

namespace Graph
{
class Observation;
class Frame;
class Landmark
{
  public:
  std::map<Frame*,Observation*> obsv_map_;
  cv::Scalar color_;
  bool good_ = true; 
};
}

#endif