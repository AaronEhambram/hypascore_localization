#ifndef VISUALODOMETRY_H_
#define VISUALODOMETRY_H_
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

class VisualOdometry
{
  public:
  VisualOdometry(ros::NodeHandle& nh);
  ~VisualOdometry();
  void track(const sensor_msgs::ImageConstPtr& l_im_ptr, const sensor_msgs::ImageConstPtr& r_im_ptr);

  /**********Externally provided parameters**********/
  // intrinsic camera parameters
  cv::Mat l_cam_, l_cam_rect_;
  cv::Mat l_dist_coeff_;
  cv::Mat r_cam_, r_cam_rect_;
  cv::Mat r_dist_coeff_;
  cv::Size l_im_size_, r_im_size_, l_im_rect_size_, r_im_rect_size_;

  // extrinsic calibration parameters
  Eigen::Vector3d utm_world_offset_;
  Eigen::Affine3d car_T_left_cam_, car_T_right_cam_, car_T_laser_, car_T_imu_, left_cam_T_right_cam_rect_;

  /**********Image related**********/
  // https://docs.opencv.org/4.x/d1/d1e/group__cuda.html
  // stereo  rectification
  cv::Mat left_map1_,left_map2_,right_map1_,right_map2_;
  cv::cuda::GpuMat left_map1_gpu_,left_map2_gpu_,right_map1_gpu_,right_map2_gpu_,translation_mat_gpu_;
  int right_image_px_y_translate_;
  double im_resize_factor_;
  cv::cuda::GpuMat l_im_, r_im_;
  void preprocess_images(const sensor_msgs::ImageConstPtr& l_im_ptr, const sensor_msgs::ImageConstPtr& r_im_ptr);
  void rectify_remap_gpu(cv_bridge::CvImageConstPtr& cv_ptr, cv::cuda::GpuMat& im, cv::cuda::GpuMat& map1, cv::cuda::GpuMat& map2);
  // Feature detection
  cv::Ptr<cv::cuda::CornersDetector> fdetector_;
  cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> ftracker_; 
  cv::Ptr<cv::cuda::ORB> orb_detector_; 
  cv::cuda::GpuMat l_im_prev_, r_im_prev_;
  std::vector<cv::cuda::GpuMat> detection_masks_; 
  std::vector<cv::Mat> detection_masks_cpu_;
  void detect_track_features(); 
  void detect_features(cv::cuda::GpuMat& im, bool is_left_image, std::vector<cv::KeyPoint>& all_keypoints_cpu, cv::Mat& all_descriptors_cpu);
  void detect_further_features();
  void track_features();
  void match_stereo_features(std::vector<cv::KeyPoint>& l_kp, cv::Mat& l_desc, std::vector<cv::KeyPoint>& r_kp, cv::Mat& r_desc, std::vector<std::pair<size_t,size_t>>& left_right_kp_index_matches);
  int orb_descriptor_distance(const cv::Mat &a, const cv::Mat &b);
  void visualize_stereo_matches(cv::Mat& l_im, cv::Mat& r_im);
  void next_iteration_preparation();
  bool feature_tracking_initialized = false; 
  int horizontal_tiles_masks_ = 4; 
  int vertical_tiles_masks_ = 3; 
  int min_features_to_init_ = 100;
  int min_features_to_track_only_ = 80;
  int min_features_in_mask_to_detect_new_ = 20;
  int min_features_in_mask_distribution_ = 5;
  double min_feature_distribution_ = 0.4;
  double max_optical_flow_error_ = 5.0;
  double max_vertical_pixel_error_ = 2.0; 
  double max_residual_stereo_odom_  = 1.0; 
  double max_bad_measurements_portion_stereo_odom_ = 0.2;
  double max_residual_bundle_adjustment_  = 3.0;
  int min_connected_good_landmarks_ = 5; 
  double eps_phi_bisection_ = 0.005; 
  double eps_x_bisection_ = 0.1; 
  // stereo matching parameters
  const int orb_match_th_high_ = 100;
  const int orb_match_th_low_ = 50;

  /**********Graph related**********/
  std::vector<Graph::Frame*> graph_frames_;
  //std::vector<Graph::Landmark*> graph_landmarks_;
  //std::vector<Graph::Observation*> graph_observations_;
  double cur_time_; // is set in preprocess_images(...)
  Graph::Frame* cur_frame_;
  Graph::Frame* last_kf_; 
  void landmarks_to_mat(cv::Mat& lfs, cv::Mat& rfs, std::vector<Graph::Landmark*>& landmarks, Graph::Frame* frame);
  void delete_once_seen_landmarks();
  std::thread delete_once_seen_landmarks_thread_;
  std::vector<Graph::Frame*> frames_with_once_seen_landmarks_;
  std::mutex mutex_frames_with_once_seen_landmarks_;
  std::mutex mutex_delete_landmarks_;
  std::mutex mutex_graph_;
  void graph_optimization_stereo_odometry(); // compute odometry between current and last frame
  void graph_optimization_batch_bundle_adjustment(); // optimize the poses and the visual features for the whole tracking-batch
  std::thread graph_opt_ba_thread_;
  std::vector<std::pair<int,int>> batch_start_end_index_; // first index: keyframe, second: end of the batch
  std::mutex mutex_batch_start_end_index_;
  void triangulate_stereo(cv::Point2f& lp, cv::Point2f& rp, Eigen::Vector3d& lcam_p);
  void triangulate_stereo_box(cv::Point2f& lp, cv::Point2f& rp, Eigen::Vector3d& lcam_p, ibex::IntervalVector& lcam_b);
  void project_stereo(Eigen::Vector3d& lcam_p, cv::Point2f& lp, cv::Point2f& rp);
  double visual_feature_uncertainty_radius_left_im_;
  double visual_feature_uncertainty_radius_right_im_;
  void contract_consecutive_frames_odom(Graph::Frame* f1, Graph::Frame* f2, ibex::IntervalVector& f1_lcam_p_f2_lcam);
  double max_batch_length_; 

  /**********Visualization**********/
  bool show_stereo_matches_;
  bool show_left_features_;
  int cv_wait_key_time_ = 0; 

  /**********Output file**********/
  bool dump_results_ = false;
  std::ofstream results_ofstream_;
  double cur_preprocessing_duration_, cur_detect_track_duration_, cur_stereo_odom_opt_duration_;
  void save_to_file();
};

#endif