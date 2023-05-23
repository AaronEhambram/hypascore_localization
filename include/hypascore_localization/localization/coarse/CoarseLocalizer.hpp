#ifndef COARSELOCALIZER_H_
#define COARSELOCALIZER_H_
#include <iostream>
#include <fstream>
#include <sstream>
#include "Eigen/Dense"
#include <thread>
#include "hypascore_localization/localization/CMMap.hpp"
#include "hypascore_localization/localization/coarse/IntervalPose.hpp"
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/PoseStamped.h"
#include "ibex.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Sparse>
#include <mutex>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/calib3d.hpp>
#include "opencv2/imgproc.hpp"
#include "hypascore_localization/localization/LidarData.hpp"
#include "hypascore_localization/localization/GpsData.hpp"

class CMGlobalLocalizer
{
  public:
  CMGlobalLocalizer(ros::NodeHandle& nh, CMMap* map);
  CMMap* map_;
  Eigen::Affine3d car_T_left_cam_, car_T_right_cam_, car_T_laser_, car_T_imu_;
  void set_initial_region(ibex::IntervalVector& translation, ibex::Interval& rotation);
  std::vector<IntervalPose> feasible_poses_;
  void shift_localization_region(Eigen::Affine2d& laser_bef_T_laser_cur, std::unique_ptr<LidarScan>& lidar_scan, std::unique_ptr<GpsMeasure>& gps_data);
  std::vector<ibex::IntervalVector> bc_laser_;
  pcl::PointCloud<pcl::PointXY>::Ptr laser_pc_;
  std::vector<PoseParticle> best_particles_;
  std::vector<PoseParticle> all_particles_;
  // get result function -> provides the hull over the estimted pose
  void get_localization_result(ibex::IntervalVector& map_p_laser);
  // Performance evaluation functions
  void save_timestamp_area_to_file(Eigen::Affine2d& world_T_laser_cur, double& t_stamp);

  private:
  int max_n_threads_; 
  int rotation_bins_amount_;
  bool use_ipose_threads_ = true; 
  double min_diam_length_ = 0.5;
  double min_rotation_diam_length_; 
  double x_odom_translation_uncertainty_radius_;
  double y_odom_translation_uncertainty_radius_;
  double odom_rotation_uncertainty_radius_;
  std::string results_file_;
  std::ofstream results_ofstream_;
  
  // LiDAR
  double max_point_line_association_distance_;

  // Map 
  double wall_line_uncertainty_; 

  // hull of the localization
  ibex::IntervalVector world_p_laser_ = ibex::IntervalVector(3); 

  void carve_out_buildings();
  void carve_out_buildings_subpaving_thread(ibex::IntervalVector* translation, ibex::Interval* rotation, std::vector<ibex::IntervalVector>* result);
  void carve_out_buildings_subpaving_sep_thread(ibex::IntervalVector* translation, ibex::Interval* rotation, std::vector<ibex::IntervalVector>* result); 
  enum polygon_test_action { bisect, include, ignore };
  void operation_selector(ibex::IntervalVector& box, Surface& polygon, polygon_test_action& action, double& wall_uncertainty);
  void operation_selector_map_hull(ibex::IntervalVector& box, polygon_test_action& action);
  bool line_segments_intersect(Eigen::Vector2d& p1, Eigen::Vector2d& p2, Eigen::Vector2d& p3, Eigen::Vector2d& p4);
  bool line_segments_intersect(Eigen::Vector2d& p1, Eigen::Vector2d& p2, Eigen::Vector2d& p3, Eigen::Vector2d& p4, Eigen::Vector2d& intersection);
  void carve_out_buildings_ipose_thread(IntervalPose* ipose, std::vector<std::vector<ibex::IntervalVector>>* result);
  void save_filtered_pointcloud(sensor_msgs::PointCloud2ConstPtr& pc_ptr);
  void delete_observation_cross_regions();
  void carve_out_observation_cross_region_subpaving_thread(ibex::IntervalVector* t, ibex::IntervalMatrix* R, std::vector<ibex::IntervalVector>* result); 
  static bool compare_index_subsetnumber_pairs(std::pair<int,double> p1, std::pair<int,double> p2);
  static bool compare_line_measurements_pairs(std::pair<WallGroundLine*,std::vector<size_t>> p1, std::pair<WallGroundLine*,std::vector<size_t>> p2); 

  // visualization
  std::shared_ptr<ros::Publisher> global_position_hull_publisher_;
  void visualize_results();
};

#endif