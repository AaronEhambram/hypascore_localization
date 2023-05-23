#ifndef LIDARDATA_H_
#define LIDARDATA_H_
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Sparse>
#include "ibex.h"

class LidarScan
{
  public:
  double timestamp_; 
  // contains whole LiDAR-Pointcloud besides ground
  std::unique_ptr<std::vector<ibex::IntervalVector>> boxcloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_;
  // contains the filtered LiDAR-Pointcloud with long facades lines
  std::unique_ptr<std::vector<ibex::IntervalVector>> boxcloud_filtered_;
  pcl::PointCloud<pcl::PointXY>::Ptr pointcloud_filtered_;
  // virtual 2D-Scan filtered points
  std::unique_ptr<std::vector<ibex::IntervalVector>> boxcloud_virtual_2d_;
  pcl::PointCloud<pcl::PointXY>::Ptr pointcloud_virtual_2d_;
};

class LidarData
{
  public:
  LidarData(ros::NodeHandle& nh);
  std::map<double,std::unique_ptr<LidarScan>> time_scan_map_;
  void save_lidar_data(const sensor_msgs::PointCloud2ConstPtr pc_ptr, double timestamp);

  private:
   // LiDAR
  double distance_delta_ = 0.06;
  std::string lidar_type_ = "cepton";
  double vertical_angle_delta_ = 0.00087266426;
  double horizontal_angle_delta_ = 0.002792526803;
  int skip_row_ = 0;
  int skip_col_ = 30;
  double min_z_clip_; 
  double max_point_line_association_distance_;
  // Line Extraction 
  double distance_bin_size_ = 0.2;
  int max_line_points_step_size_ = 5;
  int max_angle_bin_dist_ = 10; 
  double max_neighbour_point_distance_ = 4.0;
  int min_num_points_on_line_ = 6; 
  double min_line_length_ = 7.5;
  double selected_points_distance_ = 1.5;  
};

#endif