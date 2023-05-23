#ifndef GPSDATA_H_
#define GPSDATA_H_
#include "ros/ros.h"
#include <Eigen/Dense>

class GpsMeasure{

  public:
  double timestamp_;
  // the input to the class
  Eigen::Vector3d utm_t_car_; 
  // the position in world is determined with the transform in GpsData
  Eigen::Vector3d world_t_car_; 
  // the uncertainty radius of the GPS-measurement
  double gps_uncertainty_;
};

class GpsData{

  public: 
  GpsData(ros::NodeHandle& nh);
  std::map<double,std::unique_ptr<GpsMeasure>> time_gps_map_;
  void save_gps_data(const Eigen::Vector3d& utm_position, double timestamp);

  private:
  double gps_uncertainty_; 
  Eigen::Vector3d utm_world_offset_;
};

#endif