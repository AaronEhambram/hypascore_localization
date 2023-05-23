#include "hypascore_localization/localization/GpsData.hpp"

GpsData::GpsData(ros::NodeHandle& nh)
{
  double utm_world_offset_x, utm_world_offset_y, utm_world_offset_z;
  nh.getParam("/hypascore_localization_node/gps_uncertainty", gps_uncertainty_);
  nh.getParam("/hypascore_localization_node/utm_world_offset_x", utm_world_offset_x);
  nh.getParam("/hypascore_localization_node/utm_world_offset_y", utm_world_offset_y);
  nh.getParam("/hypascore_localization_node/utm_world_offset_z", utm_world_offset_z);
  utm_world_offset_ << utm_world_offset_x, utm_world_offset_y, utm_world_offset_z; 
}

void GpsData::save_gps_data(const Eigen::Vector3d& utm_position, double timestamp)
{
  // setup the new GPS mesurement entry
  time_gps_map_[timestamp].reset(new GpsMeasure);
  std::unique_ptr<GpsMeasure>& gps = time_gps_map_[timestamp];
  gps->timestamp_ = timestamp; 
  gps->gps_uncertainty_ = gps_uncertainty_; 
  gps->utm_t_car_ = utm_position; 
  
  // transform utm to world (simple offset)
  gps->world_t_car_ = gps->utm_t_car_ - utm_world_offset_; 
}