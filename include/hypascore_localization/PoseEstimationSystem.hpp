#ifndef POSEESTIMATIONSYSTEM_H_
#define POSEESTIMATIONSYSTEM_H_
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include "Eigen/Dense"
#include "hypascore_localization/localization/CMMap.hpp"
#include "sensor_msgs/Imu.h"
#include <opencv2/core/cuda.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaoptflow.hpp>
#include <opencv2/cudastereo.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/xfeatures2d/cuda.hpp>
#include "hypascore_localization/visual_odometry/Frame.hpp"
#include "hypascore_localization/visual_odometry/Observation.hpp"
#include "hypascore_localization/visual_odometry/Landmark.hpp"
#include <thread>
#include <mutex>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include "hypascore_localization/contractors/Ctc3DStereoVO.hpp"
#include "hypascore_localization/visual_odometry/VisualOdometry.hpp"
#include "hypascore_localization/localization/LidarData.hpp"
#include "hypascore_localization/localization/GpsData.hpp"
#include "hypascore_localization/localization/coarse/CoarseLocalizer.hpp"
#include "hypascore_localization/localization/refined/RefinedLocalizer.hpp"

class PoseEstimationSystem
{
  public:
  PoseEstimationSystem(ros::NodeHandle& nh);
  ~PoseEstimationSystem();
  void localize_and_track(const sensor_msgs::ImageConstPtr& l_im_ptr, const sensor_msgs::ImageConstPtr& r_im_ptr, const sensor_msgs::PointCloud2ConstPtr& pc_ptr);
  void process_gps(const Eigen::Vector3d& utm_position, double timestamp); 

  // the building map object
  std::unique_ptr<CMMap> map_;

  // peform all localization approches in this thread
  std::thread localization_thread_; 
  void localize();
  void initialize_localization(ibex::IntervalVector& translation, ibex::Interval& rotation);
  std::unique_ptr<CMGlobalLocalizer> global_localizer_; 
  std::unique_ptr<CMTracker> tracker_ ; 
  std::mutex localize_mutex_; 
  double cur_time_;

  // visual odometry object -> gets data continuously
  std::unique_ptr<VisualOdometry> vo_;

  // Save the point clouds 
  std::unique_ptr<LidarData> lidar_data_; 
  std::mutex lidar_data_mutex_; 

  // Save the GPS readings
  std::unique_ptr<GpsData> gps_data_;
  std::mutex gps_data_mutex_;

  // extrinsic calibration parameters
  Eigen::Vector3d utm_world_offset_;


  /*Visualization*/
  // Visulization functions for OpenCV image
  void create_full_building_image(); 
  void show_full_map();
  void show_cur_pose_full_map();
  void multiply_interval_poses(ibex::IntervalVector& a_T_b, ibex::IntervalVector& b_T_c, ibex::IntervalVector& a_T_c);
  // Visualization parameters
  double x_size_full_, y_size_full_; 
  double pixel_size_full_; 
  Eigen::Vector2d im_origin_full_; 
  cv::Mat full_building_image_;
  std::vector<std::string> cv_windows;
  cv::Mat output_im_; 
  Eigen::Affine2d map_localize_T_laser_cur_;
  std::mutex output_im_mutex_;
  std::mutex window_output_im_mutex_;
  PoseParticle tracker_particle_copy_;
  bool tracker_initialized_ = false; 
  double tracker_reliability_ = 0.0; 
  double localize_time_stamp_;
  // Visualization for RVIZ
  std::shared_ptr<ros::Publisher> cur_pose_publisher_;
  std::shared_ptr<ros::Publisher> tracked_pose_publisher_;
  std::shared_ptr<ros::Publisher> cur_polygon_publisher_;
  std::shared_ptr<ros::Publisher> cur_global_polygon_publisher_;
  std::vector<Eigen::Vector2d> tracked_polygone_;
  ibex::Interval tracked_rotation_;
  ibex::IntervalVector gl_pose_hull_;
  double x_odom_uncertainty_predicted_; 
  double y_odom_uncertainty_predicted_;
  double rot_odom_uncertainty_predicted_;
  int min_particle_age_to_overwrite_gl_ = 300;

  // output file for results
  void save_to_file(Graph::Frame* f);
  void save_only_rt_to_file();
  bool dump_results_ = false; 
  std::ofstream results_ofstream_;
  bool dump_only_rt_results_ = false; 
  std::ofstream only_rt_results_ofstream_;
  std::mutex file_write_mutex_; 
  double vo_time_, refined_loc_time_, coarse_loc_time_, full_loc_time_;
  ibex::IntervalVector map_T_laser_cur_feasible_hull_; 
  ibex::IntervalVector map_T_laser_loc_feasible_hull_; 
  ibex::Interval consistent_rot_cur_;
  std::vector<cv::Point2f> consistent_polygon_cur_;
};

#endif