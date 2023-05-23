#include "hypascore_localization/visual_odometry/VisualOdometry.hpp"
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/cudawarping.hpp>
#include "opencv2/opencv.hpp"
#include <thread>
#include "g2o/core/robust_kernel_impl.h"
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include "g2o/core/sparse_optimizer_terminate_action.h"
#include <chrono>

VisualOdometry::VisualOdometry(ros::NodeHandle& nh)
{
  std::string calibration_folder;
  double utm_world_offset_x;
  double utm_world_offset_y;
  double utm_world_offset_z;
  std::string city_model_file, file_visual_odom_results;

  nh.getParam("/hypascore_localization_node/calibration_folder", calibration_folder);
  nh.getParam("/hypascore_localization_node/utm_world_offset_x", utm_world_offset_x);
  nh.getParam("/hypascore_localization_node/utm_world_offset_y", utm_world_offset_y);
  nh.getParam("/hypascore_localization_node/utm_world_offset_z", utm_world_offset_z);
  nh.getParam("/hypascore_localization_node/city_model_file", city_model_file);
  nh.getParam("/hypascore_localization_node/right_image_px_y_translate", right_image_px_y_translate_);
  nh.getParam("/hypascore_localization_node/im_resize_factor", im_resize_factor_);
  nh.getParam("/hypascore_localization_node/visual_feature_uncertainty_radius_left_im", visual_feature_uncertainty_radius_left_im_);
  nh.getParam("/hypascore_localization_node/visual_feature_uncertainty_radius_right_im", visual_feature_uncertainty_radius_right_im_);
  nh.getParam("/hypascore_localization_node/horizontal_tiles_masks", horizontal_tiles_masks_);
  nh.getParam("/hypascore_localization_node/vertical_tiles_masks", vertical_tiles_masks_);
  nh.getParam("/hypascore_localization_node/min_features_to_init", min_features_to_init_);
  nh.getParam("/hypascore_localization_node/min_features_to_track_only", min_features_to_track_only_);
  nh.getParam("/hypascore_localization_node/max_optical_flow_error", max_optical_flow_error_);
  nh.getParam("/hypascore_localization_node/max_vertical_pixel_error", max_vertical_pixel_error_);
  nh.getParam("/hypascore_localization_node/max_residual_stereo_odom", max_residual_stereo_odom_);
  nh.getParam("/hypascore_localization_node/max_bad_measurements_portion_stereo_odom", max_bad_measurements_portion_stereo_odom_);
  nh.getParam("/hypascore_localization_node/max_residual_bundle_adjustment", max_residual_bundle_adjustment_);
  nh.getParam("/hypascore_localization_node/min_connected_good_landmarks", min_connected_good_landmarks_);
  nh.getParam("/hypascore_localization_node/eps_phi_bisection", eps_phi_bisection_);
  nh.getParam("/hypascore_localization_node/eps_x_bisection", eps_x_bisection_);
  nh.getParam("/hypascore_localization_node/show_stereo_matches", show_stereo_matches_);
  nh.getParam("/hypascore_localization_node/show_left_features", show_left_features_);
  nh.getParam("/hypascore_localization_node/cv_wait_key_time", cv_wait_key_time_);
  nh.getParam("/hypascore_localization_node/min_features_in_mask_to_detect_new", min_features_in_mask_to_detect_new_);
  nh.getParam("/hypascore_localization_node/min_features_in_mask_distribution", min_features_in_mask_distribution_);
  nh.getParam("/hypascore_localization_node/min_feature_distribution", min_feature_distribution_);
  nh.getParam("/hypascore_localization_node/max_batch_length", max_batch_length_);
  nh.getParam("/hypascore_localization_node/file_visual_odom_results", file_visual_odom_results);

  if(file_visual_odom_results != "") // if file_visual_odom_results string is not empty, dump results to file_tracker_results_
  {
    dump_results_ = true;
    std::cout << "Start saving to: " << file_visual_odom_results << std::endl; 
    results_ofstream_.open(file_visual_odom_results.c_str());
  }

  std::string extrinsic_calib_file = calibration_folder+"extrinsics.xml";
  std::string intrinsic_calib_file = calibration_folder+"intrinsics.xml";

  // intrinsic camera calibrations 
  std::cout << "load: " << intrinsic_calib_file << std::endl;
  cv::FileStorage fs_intr(intrinsic_calib_file, cv::FileStorage::READ);
  fs_intr["left_cam"] >> l_cam_;
  fs_intr["left_dist"] >> l_dist_coeff_;
  fs_intr["right_cam"] >> r_cam_;
  fs_intr["right_dist"] >> r_dist_coeff_;
  fs_intr["left_im_size"] >> l_im_size_;
  fs_intr["right_im_size"] >> r_im_size_;

  std::cout << "l_cam_: " << std::endl << l_cam_ << std::endl;
  std::cout << "l_dist_coeff_: " << std::endl << l_dist_coeff_ << std::endl;
  std::cout << "r_cam_: " << std::endl << r_cam_ << std::endl;
  std::cout << "r_dist_coeff_: " << std::endl << r_dist_coeff_ << std::endl;

  // extrinsic calibration data
  std::cout << "load: " << extrinsic_calib_file << std::endl;
  cv::FileStorage fs_extr(extrinsic_calib_file, cv::FileStorage::READ);
  cv::Mat car_T_left_cam__mat, car_T_right_cam__mat, car_T_laser__mat, car_T_imu__mat;
  fs_extr["car_T_left_cam"] >> car_T_left_cam__mat;
  fs_extr["car_T_right_cam"] >> car_T_right_cam__mat;
  fs_extr["car_T_laser"] >> car_T_laser__mat;
  fs_extr["car_T_imu"] >> car_T_imu__mat;
  cv::cv2eigen(car_T_left_cam__mat, car_T_left_cam_.matrix());
  cv::cv2eigen(car_T_right_cam__mat, car_T_right_cam_.matrix());
  cv::cv2eigen(car_T_laser__mat, car_T_laser_.matrix());
  cv::cv2eigen(car_T_imu__mat, car_T_imu_.matrix());

  std::cout << "car_T_left_cam_: " << std::endl << car_T_left_cam_.matrix() << std::endl;
  std::cout << "car_T_right_cam_: " << std::endl << car_T_right_cam_.matrix() << std::endl;
  std::cout << "car_T_laser_: " << std::endl << car_T_laser_.matrix() << std::endl;
  std::cout << "car_T_imu_: " << std::endl << car_T_imu_.matrix() << std::endl;
  std::cout << "laser_T_left_cam_: " << std::endl << (car_T_left_cam_.inverse()*car_T_laser_).matrix() << std::endl;
  Eigen::Vector3d utm_world_offset(utm_world_offset_x,utm_world_offset_y,utm_world_offset_z);
  utm_world_offset_ = utm_world_offset;

  // compute stereo rectification maps
  cv::Mat right_cam_T_left_cam__mat = car_T_right_cam__mat.inv()*car_T_left_cam__mat;
  cv::Rect R_ROI(0, 0, 3, 3);
  cv::Mat right_cam_R_left_cam(right_cam_T_left_cam__mat, R_ROI);
  cv::Rect T_ROI(3, 0, 1, 3);
  cv::Mat right_cam_t_left_cam(right_cam_T_left_cam__mat, T_ROI);
  cv::Mat lR,rR,lP,rP,Q;
  cv::stereoRectify(l_cam_, l_dist_coeff_, r_cam_, r_dist_coeff_, l_im_size_, right_cam_R_left_cam,
                           right_cam_t_left_cam, lR, rR, lP, rP, Q, cv::CALIB_ZERO_DISPARITY, -1, l_im_size_);
  cv::initUndistortRectifyMap(l_cam_,l_dist_coeff_,lR,lP,l_im_size_,CV_32F,left_map1_,left_map2_);
  cv::initUndistortRectifyMap(r_cam_,r_dist_coeff_,rR,rP,r_im_size_,CV_32F,right_map1_,right_map2_);
  left_map1_gpu_.upload(left_map1_);
  left_map2_gpu_.upload(left_map2_);
  right_map1_gpu_.upload(right_map1_);
  right_map2_gpu_.upload(right_map2_);
  std::cout << "rP: " << std::endl << rP << std::endl;
  std::cout << "lP: " << std::endl << lP << std::endl;
  left_cam_T_right_cam_rect_.setIdentity();
  left_cam_T_right_cam_rect_.matrix()(0,3) = -rP.at<double>(0,3)/lP.at<double>(0,0);
  cv::Rect cam_ROI(0, 0, 3, 3);
  l_cam_rect_ = cv::Mat(lP,cam_ROI);
  l_cam_rect_ = l_cam_rect_*im_resize_factor_;
  l_cam_rect_.at<double>(2,2) = 1;
  l_im_rect_size_.width = l_im_size_.width*im_resize_factor_;
  l_im_rect_size_.height = l_im_size_.height*im_resize_factor_;
  r_cam_rect_ = cv::Mat(rP,cam_ROI);
  r_cam_rect_ = r_cam_rect_*im_resize_factor_;
  r_cam_rect_.at<double>(2,2) = 1;
  r_im_rect_size_.width = r_im_size_.width*im_resize_factor_;
  r_im_rect_size_.height = r_im_size_.height*im_resize_factor_;
  right_image_px_y_translate_ = (int)round((double)right_image_px_y_translate_*im_resize_factor_);
  std::cout << "l_cam_rect_: " << std::endl << l_cam_rect_ << std::endl;
  std::cout << "r_cam_rect_: " << std::endl << r_cam_rect_ << std::endl;
  std::cout << "left_cam_T_right_cam_rect_: " << std::endl << left_cam_T_right_cam_rect_.matrix() << std::endl;
  std::cout << "right_image_px_y_translate_: " << right_image_px_y_translate_ << std::endl;
  cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, 0, 0, 1, right_image_px_y_translate_);
  translation_mat_gpu_.upload(trans_mat);
  // Get GPU information
  int gpucount = cv::cuda::getCudaEnabledDeviceCount();
  if(gpucount != 0){
      std::cout << "no. of gpu = " << gpucount << std::endl;
  }
  else
  {
      std::cout << "There is no CUDA supported GPU" << std::endl;
      return;
  }
  cv::cuda::setDevice(0);
  //cuda::resetDevice();
  enum cv::cuda::FeatureSet arch_avail;
  if(cv::cuda::TargetArchs::builtWith(arch_avail))
      std::cout << "yes, this Gpu arch is supported" << std::endl;

  cv::cuda::DeviceInfo deviceinfo;
  std::cout << "GPU: "<< deviceinfo.cv::cuda::DeviceInfo::name() << std::endl;

  fdetector_ = cv::cuda::createGoodFeaturesToTrackDetector(CV_8UC1,100,0.01,20,3,false,0.04);
  ftracker_ = cv::cuda::SparsePyrLKOpticalFlow::create();
  orb_detector_ = cv::cuda::ORB::create(200,1.2f,1,31,0,2,0,31,20,false);

  // create detection masks
  int horizontal_tiles = horizontal_tiles_masks_;
  int vertical_tiles = vertical_tiles_masks_;
  int mask_width = std::floor(l_im_rect_size_.width/horizontal_tiles); 
  int mask_height = std::floor(l_im_rect_size_.height/vertical_tiles);
  detection_masks_.resize(horizontal_tiles*vertical_tiles);
  detection_masks_cpu_.resize(horizontal_tiles*vertical_tiles);
  int mask_counter = 0; 
  for(int h = 0; h < horizontal_tiles; ++h)
  {
    for(int v = 0; v < vertical_tiles; ++v)
    {
      cv::Mat& mask = detection_masks_cpu_[mask_counter];
      mask = cv::Mat::zeros(l_im_rect_size_, CV_8U); 
      cv::Mat roi(mask, cv::Rect(h*mask_width,v*mask_height,mask_width,mask_height));
      roi = cv::Scalar(255);
      detection_masks_[mask_counter].upload(mask);
      mask_counter++; 
    }
  }

  // start thread for deleting once seen landmarks for the frames in the vector frames_with_once_seen_landmarks
  delete_once_seen_landmarks_thread_ = std::thread(&VisualOdometry::delete_once_seen_landmarks, this);

  // start thread for performing bundle adjustment for the batch, in which the features were well tracked
  graph_opt_ba_thread_ = std::thread(&VisualOdometry::graph_optimization_batch_bundle_adjustment, this);
}

VisualOdometry::~VisualOdometry()
{
  delete_once_seen_landmarks_thread_.join();
  graph_opt_ba_thread_.join(); 
}

void VisualOdometry::track(const sensor_msgs::ImageConstPtr& l_im_ptr, const sensor_msgs::ImageConstPtr& r_im_ptr)
{
  // perform image preprocessing and stereo rectification
  auto start_preprocessing = std::chrono::high_resolution_clock::now();
  preprocess_images(l_im_ptr, r_im_ptr);
  auto end_preprocessing = std::chrono::high_resolution_clock::now();
  auto duration_preprocessing = std::chrono::duration_cast<std::chrono::microseconds>(end_preprocessing - start_preprocessing);
  cur_preprocessing_duration_ = duration_preprocessing.count()*std::pow(10,-6);

  // Perform detection and tracking of the features
  auto start_detect_track = std::chrono::high_resolution_clock::now();
  detect_track_features();
  auto end_detect_track = std::chrono::high_resolution_clock::now();
  auto duration_detect_track = std::chrono::duration_cast<std::chrono::microseconds>(end_detect_track - start_detect_track);
  cur_detect_track_duration_ = duration_detect_track.count()*std::pow(10,-6);

  // prepare next iteration
  next_iteration_preparation();

  // compute the relative transformation between the current frame and previous frame
  auto start_stereo_odom_opt = std::chrono::high_resolution_clock::now();
  if(graph_frames_.size() > 1)
  {
    graph_optimization_stereo_odometry();
  }
  auto end_stereo_odom_opt = std::chrono::high_resolution_clock::now();
  auto duration_stereo_odom_opt = std::chrono::duration_cast<std::chrono::microseconds>(end_stereo_odom_opt - start_stereo_odom_opt);
  cur_stereo_odom_opt_duration_ = duration_stereo_odom_opt.count()*std::pow(10,-6);

  // visulize the current frame with the features
  cv::Mat l_im_kp,r_im_kp;
  l_im_.download(l_im_kp);
  r_im_.download(r_im_kp);
  visualize_stereo_matches(l_im_kp,r_im_kp);

  // save the processing times to cur_frame_
  cur_frame_->preprocessing_duration_ = cur_preprocessing_duration_;
  cur_frame_->detect_track_duration_ = cur_detect_track_duration_;
  cur_frame_->stereo_odom_opt_duration_ = cur_stereo_odom_opt_duration_; 
}

/****Image related functions*****/
void VisualOdometry::preprocess_images(const sensor_msgs::ImageConstPtr& l_im_ptr, const sensor_msgs::ImageConstPtr& r_im_ptr)
{
  // set the time 
  cur_time_ = l_im_ptr->header.stamp.toSec();
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrLeft;
  try
  {
      cv_ptrLeft = cv_bridge::toCvShare(l_im_ptr);
  }
  catch (cv_bridge::Exception& e)
  {
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrRight;
  try
  {
      cv_ptrRight = cv_bridge::toCvShare(r_im_ptr);
  }
  catch (cv_bridge::Exception& e)
  {
    return;
  }
  cv::cuda::GpuMat r_im_remapped;
  rectify_remap_gpu(cv_ptrLeft,l_im_,left_map1_gpu_,left_map2_gpu_);
  rectify_remap_gpu(cv_ptrRight,r_im_remapped,right_map1_gpu_,right_map2_gpu_);
  // shift the right image to have corresponding epipolar lines
  cv::cuda::warpAffine(r_im_remapped,r_im_,translation_mat_gpu_,r_im_remapped.size());

  /*cv::Mat im;
  cv::Mat imLeft;
  cv::Mat imRight;
  l_im_.download(imLeft);
  r_im_.download(imRight);
  cv::hconcat(imLeft, imRight, im);
  cv::Scalar color(255,255, 0);
  cv::line(im, cv::Point(0,100), cv::Point(3838,100), color, 1);
  cv::line(im, cv::Point(0,200), cv::Point(3838,200), color, 1);
  cv::line(im, cv::Point(0,300), cv::Point(3838,300), color, 1);
  cv::line(im, cv::Point(0,400), cv::Point(3838,400), color, 1);
  cv::line(im, cv::Point(0,500), cv::Point(3838,500), color, 1);
  cv::line(im, cv::Point(0,600), cv::Point(3838,600), color, 1);
  cv::line(im, cv::Point(0,700), cv::Point(3838,700), color, 1);
  cv::line(im, cv::Point(0,800), cv::Point(3838,800), color, 1);
  cv::line(im, cv::Point(0,900), cv::Point(3838,900), color, 1);
  //cv::imwrite("/home/ehambram/Documents/Promotion/Dokumentation/Bilder/orb_slam/Rectification_problems/y_minus_gpu.jpg", im);
  cv::imshow("test",im);*/
}

void VisualOdometry::rectify_remap_gpu(cv_bridge::CvImageConstPtr& cv_ptr, cv::cuda::GpuMat& im, cv::cuda::GpuMat& map1, cv::cuda::GpuMat& map2)
{
  cv::cuda::GpuMat color_mat;
  color_mat.upload(cv_ptr->image);
  // convert images to gray-scale
  cv::cuda::GpuMat gray_im;
  cv::cuda::cvtColor(color_mat, gray_im, CV_BGR2GRAY);
  // perform stereo-rectification
  cv::cuda::GpuMat gray_im_remapped;
  cv::cuda::remap(gray_im,gray_im_remapped,map1,map2,cv::INTER_LINEAR);
  // resize the images
  cv::cuda::GpuMat im_resized;
  cv::cuda::resize(gray_im_remapped,im,cv::Size(),im_resize_factor_,im_resize_factor_);
}

void VisualOdometry::detect_features(cv::cuda::GpuMat& im, bool is_left_image, std::vector<cv::KeyPoint>& all_keypoints_cpu, cv::Mat& all_descriptors_cpu)
{
  // check which of the masks have how many features
  std::vector<int> features_in_mask(detection_masks_cpu_.size(),0);
  for(Graph::Landmark* l : cur_frame_->seen_landmarks_)
  {
    cv::Point2f f;
    if(is_left_image)
    {
      f = l->obsv_map_[cur_frame_]->lf_;
    }
    else
    {
      f = l->obsv_map_[cur_frame_]->rf_;
    }
    for(int i = 0; i < detection_masks_cpu_.size(); ++i)
    {
      cv::Mat& mask = detection_masks_cpu_[i];
      int v = (int) mask.at<uchar>(f.y,f.x);
      if(v > 100) // pixel value is 255
      {
        features_in_mask[i]++; 
      }
    }
  }

  std::vector<int> mask_indices_for_detection; 
  for(int i = 0; i < detection_masks_cpu_.size(); ++i)
  { 
    if(features_in_mask[i] < min_features_in_mask_to_detect_new_)
    {
      mask_indices_for_detection.push_back(i);
    }
  }
  // detect features
  for(int i = 0; i < mask_indices_for_detection.size(); ++i)
  {
    int mask_counter = mask_indices_for_detection[i]; // only indices less than min_features_in_mask_ for detection
    cv::cuda::GpuMat keypoints; // -> CV_32FC1 [#feature x 6]
    cv::cuda::GpuMat descriptors; // -> CV_8UC1 [32 x #feature]
    orb_detector_->detectAndComputeAsync(im,detection_masks_[mask_counter],keypoints,descriptors);
    std::vector<cv::KeyPoint> keypoints_cpu;
    orb_detector_->convert(keypoints,keypoints_cpu);
    all_keypoints_cpu.insert(all_keypoints_cpu.end(), keypoints_cpu.begin(), keypoints_cpu.end());
    cv::Mat descriptors_cpu;
    descriptors.download(descriptors_cpu);
    all_descriptors_cpu.push_back(descriptors_cpu);
  }
}

int VisualOdometry::orb_descriptor_distance(const cv::Mat &a, const cv::Mat &b)
{
  const int *pa = a.ptr<int32_t>();
  const int *pb = b.ptr<int32_t>();
  int dist=0;
  for(int i=0; i<8; i++, pa++, pb++)
  {
    unsigned  int v = *pa ^ *pb;
    v = v - ((v >> 1) & 0x55555555);
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }
  return dist;
}

void VisualOdometry::match_stereo_features(std::vector<cv::KeyPoint>& l_kp, cv::Mat& l_desc, std::vector<cv::KeyPoint>& r_kp, cv::Mat& r_desc, std::vector<std::pair<size_t,size_t>>& left_right_kp_index_matches)
{ 
  //row table -> Each row of the left image corresponds to the right row -> assign the right keypoints to the rows
  std::vector<std::vector<size_t> > row_indices(l_im_.rows,std::vector<size_t>());
  for(int i=0; i<l_im_.rows; i++) 
  {
    row_indices[i].reserve(200);
  }
  //#pragma omp parallel for
  for(int iR=0; iR<r_kp.size(); iR++)
  {
    cv::KeyPoint& kp = r_kp[iR];
    float& kpY = kp.pt.y; // get the row value
    const float r = max_vertical_pixel_error_*orb_detector_->getScaleFactor()*((double)r_kp[iR].octave+1);
    const int maxr = ceil(kpY+r);
    const int minr = floor(kpY-r);

    for(int yi=minr;yi<=maxr;yi++)
    {
      row_indices[yi].push_back(iR); // Save the indices to the features in the right image to the specific row
    } 
  } 
  const float minD = 0;
  const float maxD = l_cam_rect_.at<double>(0,0);
  // For each left keypoint search a match in the right image -> compute parallel!!!
  const int thOrbDist = (orb_match_th_high_+orb_match_th_low_)/2;
  std::vector<std::vector<std::pair<size_t,size_t>>> matches(l_kp.size());
  #pragma omp parallel for
  for(int iL=0; iL<l_kp.size(); iL++)
  { 
    // get left keypoint
    cv::KeyPoint& kpL = l_kp[iL];
    // get candidates in the right image -> use row table
    std::vector<size_t>& r_candidates = row_indices[kpL.pt.y];
    if(r_candidates.empty()) continue;
    const float minU = kpL.pt.x-maxD;
    const float maxU = kpL.pt.x-minD;
    
    // Compare descriptor to right keypoints
    int bestDist = orb_match_th_high_;
    size_t bestIdxR = 0;
    const cv::Mat &dL = l_desc.row(iL);
    for(size_t iC=0; iC<r_candidates.size(); iC++)
    {
      size_t iR = r_candidates[iC];
      cv::KeyPoint &kpR = r_kp[iR];
      if(kpR.octave<kpL.octave-1 || kpR.octave>kpL.octave+1) continue; // keypoints should be found in similar scales
      if(kpR.pt.x>=minU && kpR.pt.x<=maxU)
      {
        const cv::Mat &dR = r_desc.row(iR);
        const int dist = orb_descriptor_distance(dL,dR);
        if(dist<bestDist)
        {
          bestDist = dist;
          bestIdxR = iR;
        }
      }
    }
    if(bestDist<thOrbDist)
    {
      std::pair<size_t,size_t> match = std::make_pair((size_t)iL,(size_t)bestIdxR);
      matches[iL].push_back(match);
    }
  } 
  for(std::vector<std::pair<size_t,size_t>>& matches_per_thread : matches)
  {
    left_right_kp_index_matches.insert(left_right_kp_index_matches.end(), matches_per_thread.begin(), matches_per_thread.end());
  } 
}

void VisualOdometry::visualize_stereo_matches(cv::Mat& l_im, cv::Mat& r_im)
{
  bool show_stereo_matches = false;
  cv::Mat im;
  if(show_stereo_matches_)
  {
    cv::Mat imLeft;
    l_im.copyTo(imLeft);
    cv::cvtColor(imLeft, imLeft, CV_GRAY2BGR);
    cv::Mat imRight;
    r_im.copyTo(imRight);
    cv::cvtColor(imRight, imRight, CV_GRAY2BGR);
    cv::hconcat(imLeft, imRight, im);
    for(int i = 0; i < cur_frame_->seen_landmarks_.size(); i++)
    {
      Graph::Landmark* l = cur_frame_->seen_landmarks_[i]; 
      Graph::Observation* obs = l->obsv_map_[cur_frame_];

      // draw detected keypoint
      int radiusCircle = 4;
      int thickness = 1;
      cv::Point2f r_kp_shift,l_kp;
      r_kp_shift.y = obs->rf_.y;
      r_kp_shift.x = obs->rf_.x+imLeft.size().width;
      l_kp = obs->lf_;
      cv::circle(im, l_kp, radiusCircle, l->color_, thickness);
      cv::circle(im, r_kp_shift, radiusCircle, l->color_, thickness);
      cv::line(im, l_kp, r_kp_shift, l->color_, thickness);
    }
    cv::imshow("im", im);
  }
  if(show_left_features_)
  {
    cv::Mat imLeft;
    l_im.copyTo(imLeft);
    cv::cvtColor(imLeft, imLeft, CV_GRAY2BGR);
    for(int i = 0; i < cur_frame_->seen_landmarks_.size(); i++)
    {
      Graph::Landmark* l = cur_frame_->seen_landmarks_[i]; 
      Graph::Observation* obs = l->obsv_map_[cur_frame_];

      // draw detected keypoint
      int radiusCircle = 7;
      int thickness = 6;
      cv::Point2f l_kp;
      l_kp = obs->lf_;
      cv::circle(imLeft, l_kp, radiusCircle, l->color_, thickness);
    }
    cv::imshow("Left image with features", imLeft);
  }
  cv::waitKey(cv_wait_key_time_);
}

void VisualOdometry::landmarks_to_mat(cv::Mat& lfs, cv::Mat& rfs, std::vector<Graph::Landmark*>& landmarks, Graph::Frame* frame)
{
  lfs = cv::Mat(cv::Size(landmarks.size(),1), CV_32FC2);
  cv::Vec2f* l_features_cpu_col = lfs.ptr<cv::Vec2f>(0); 
  rfs = cv::Mat(cv::Size(landmarks.size(),1), CV_32FC2);
  cv::Vec2f* r_features_cpu_col = rfs.ptr<cv::Vec2f>(0);
  #pragma omp parallel for
  for(int i = 0; i<landmarks.size(); i++)
  {
    Graph::Landmark* l = landmarks[i];
    Graph::Observation* obs = l->obsv_map_[frame];
    cv::Point2f& lf = obs->lf_;
    cv::Point2f& rf = obs->rf_;
    cv::Vec2f lkp,rkp;
    lkp(0) = lf.x;
    lkp(1) = lf.y;
    rkp(0) = rf.x;
    rkp(1) = rf.y;
    l_features_cpu_col[i] = lkp;
    r_features_cpu_col[i] = rkp;
  }
}

void VisualOdometry::detect_further_features()
{
  // detect ORB-Features
  std::vector<cv::KeyPoint> l_keypoints_cpu, r_keypoints_cpu;
  cv::Mat l_descriptors_cpu, r_descriptors_cpu;
  detect_features(l_im_, true, l_keypoints_cpu, l_descriptors_cpu);
  detect_features(r_im_, false, r_keypoints_cpu, r_descriptors_cpu);

  // peform stereo matching 
  std::vector<std::pair<size_t,size_t>> left_right_kp_index_matches;
  match_stereo_features(l_keypoints_cpu, l_descriptors_cpu, r_keypoints_cpu, r_descriptors_cpu, left_right_kp_index_matches);

  // the new landmarks are added to the tracked_landmarks_!!!
  // Each stereo feature is a new landmark with an observation from the current frame
  std::vector<Graph::Landmark*> new_landmarks(left_right_kp_index_matches.size());
  std::vector<Graph::Observation*> new_observations(left_right_kp_index_matches.size());
  #pragma omp parallel for 
  for(int i = 0; i < left_right_kp_index_matches.size(); i++)
  {
    Graph::Landmark* new_landmark = new Graph::Landmark();
    Graph::Observation* new_observation = new Graph::Observation();
    new_observation->lf_ = l_keypoints_cpu[left_right_kp_index_matches[i].first].pt;
    new_observation->rf_ = r_keypoints_cpu[left_right_kp_index_matches[i].second].pt;
    triangulate_stereo(new_observation->lf_, new_observation->rf_, new_observation->lcam_point_);
    triangulate_stereo_box(new_observation->lf_, new_observation->rf_, new_observation->lcam_point_, new_observation->lcam_box_);
    new_observation->frame_ = cur_frame_;
    new_observation->landmark_ = new_landmark;
    new_landmark->obsv_map_[cur_frame_] = new_observation;
    new_landmark->color_ = cv::Scalar(rand() % 255 + 0, rand() % 255 + 0, rand() % 255 + 0);

    // save to intermediate vectors
    new_landmarks[i] = new_landmark;
    new_observations[i] = new_observation;
  }
  cur_frame_->is_keyframe_ = true;
  last_kf_ = cur_frame_;
  cur_frame_->seen_landmarks_.insert(cur_frame_->seen_landmarks_.end(), new_landmarks.begin(), new_landmarks.end());
  //mutex_graph_.lock();
  //graph_landmarks_.insert(graph_landmarks_.end(),new_landmarks.begin(),new_landmarks.end());
  //graph_observations_.insert(graph_observations_.end(), new_observations.begin(), new_observations.end()); 
  //mutex_graph_.unlock();
}

void VisualOdometry::track_features()
{
  // tracked landmarks are those landmarks that were seen in the last frame
  Graph::Frame* last_frame = graph_frames_[graph_frames_.size()-1]; // the last entry of graph_frames_ is the last frame (previous iteration)
  std::vector<Graph::Landmark*>& tracked_landmarks = last_frame->seen_landmarks_; // tracked landmarks are those that were seen in the last frame!
  // NO new landamarks!! only new observations!!!!!!!!!
  cv::Mat l_features_cpu, r_features_cpu;
  landmarks_to_mat(l_features_cpu,r_features_cpu,tracked_landmarks,last_frame); // -> use the gpuMat to replace l_features_ and r_features_!!

  // upload to gpumats
  cv::cuda::GpuMat l_features, r_features; 
  l_features.upload(l_features_cpu);
  r_features.upload(r_features_cpu);

  // first try to track as many features as possible
  cv::cuda::GpuMat l_features_new, l_out, l_err, r_features_new, r_out, r_err;
  ftracker_->calc(l_im_prev_,l_im_,l_features,l_features_new,l_out, l_err);
  ftracker_->calc(r_im_prev_,r_im_,r_features,r_features_new,r_out, r_err);

  // download left tracking results to CPU
  cv::Mat l_out_cpu(l_out.size(), l_out.type()), l_features_new_cpu(l_features_new.size(), l_features_new.type()), l_err_cpu;
  l_out.download(l_out_cpu); 
  l_features_new.download(l_features_new_cpu);
  l_err.download(l_err_cpu); 

  // download right tracking results to CPU
  cv::Mat r_out_cpu(r_out.size(), r_out.type()), r_features_new_cpu(r_features_new.size(), r_features_new.type()), r_err_cpu;
  r_out.download(r_out_cpu); 
  r_features_new.download(r_features_new_cpu);
  r_err.download(r_err_cpu);

  // save the well tracked features
  const uchar* l_out_cpu_col = l_out_cpu.ptr<uchar>(0);
  const cv::Vec2f* l_features_new_cpu_col = l_features_new_cpu.ptr<cv::Vec2f>(0); 
  const float* l_err_cpu_col = l_err_cpu.ptr<float>(0);
  const uchar* r_out_cpu_col = r_out_cpu.ptr<uchar>(0);
  const cv::Vec2f* r_features_new_cpu_col = r_features_new_cpu.ptr<cv::Vec2f>(0); 
  const float* r_err_cpu_col = r_err_cpu.ptr<float>(0);
  std::vector<Graph::Observation*> new_observations; // ONLY new observations possible!
  for(int i = 0; i < l_out_cpu.cols; ++i)
  {
    if((int)l_out_cpu_col[i] == 1 && l_err_cpu_col[i] <= max_optical_flow_error_ && (int)r_out_cpu_col[i] == 1 && r_err_cpu_col[i] <= max_optical_flow_error_) // -> feature well tracked in both images and z-component of the feature point is positive!
    {
      const cv::Vec2f& lf = l_features_new_cpu_col[i];
      const cv::Vec2f& rf = r_features_new_cpu_col[i];
      Eigen::Vector3d point_3d;
      cv::Point2f lf_p2f,rf_p2f;
      lf_p2f.x = lf(0);
      lf_p2f.y = lf(1);
      rf_p2f.x = rf(0);
      rf_p2f.y = rf(1);
      triangulate_stereo(lf_p2f, rf_p2f, point_3d);
      if(abs(lf(1)-rf(1)) <= max_vertical_pixel_error_ && point_3d(2) > 0) // features can deviate maximum 2 pixel
      {
        Graph::Landmark* correctly_tracked_landmark = tracked_landmarks[i];
        Graph::Observation* new_observation = new Graph::Observation();
        new_observation->lf_.x = lf(0);
        new_observation->lf_.y = lf(1);
        new_observation->rf_.x = rf(0);
        new_observation->rf_.y = rf(1);
        new_observation->frame_ = cur_frame_;
        new_observation->landmark_ = correctly_tracked_landmark;
        triangulate_stereo(new_observation->lf_, new_observation->rf_, new_observation->lcam_point_);
        triangulate_stereo_box(new_observation->lf_, new_observation->rf_, new_observation->lcam_point_, new_observation->lcam_box_);
        correctly_tracked_landmark->obsv_map_[cur_frame_] = new_observation;
        cur_frame_->seen_landmarks_.push_back(correctly_tracked_landmark);

        // save to intermediat vector
        //mutex_graph_.lock();
        //graph_observations_.push_back(new_observation);
        //mutex_graph_.unlock();
      }
    } 
  }
  cur_frame_->is_keyframe_ = false; 

  // check how how old the this frame is after last keyframe
  double time_diff = cur_frame_->timestamp_ - last_kf_->timestamp_; 
  if(time_diff > max_batch_length_) 
  { 
    cur_frame_->is_keyframe_ = true; 
    last_kf_ = cur_frame_;
  }
}

void VisualOdometry::detect_track_features()
{
  // prepare graph frame object!
  cur_frame_ = new Graph::Frame();
  cur_frame_->timestamp_ = cur_time_;

  if(!feature_tracking_initialized)
  {
    // Detect the features!
    detect_further_features();
    if(cur_frame_->seen_landmarks_.size() >= min_features_to_init_)
    {
      feature_tracking_initialized = true;
    } 
  }
  else
  {
    track_features();
    /*std::vector<double> stereo_box_sizes; 
    for(Graph::Landmark* l : cur_frame_->seen_landmarks_)
    {
      if(!(l->obsv_map_[cur_frame_]->lcam_box_.is_unbounded()))
      {
        stereo_box_sizes.emplace_back(ibex::norm(l->obsv_map_[cur_frame_]->lcam_box_.diam()));
      }
    }
    size_t n = stereo_box_sizes.size() / 2;
    std::nth_element(stereo_box_sizes.begin(), stereo_box_sizes.begin()+n, stereo_box_sizes.end());*/

    // check which of the masks have how many features
    std::vector<int> features_in_mask(detection_masks_cpu_.size(),0);
    for(Graph::Landmark* l : cur_frame_->seen_landmarks_)
    {
      cv::Point2f f;
      f = l->obsv_map_[cur_frame_]->lf_;
      for(int i = 0; i < detection_masks_cpu_.size(); ++i)
      {
        cv::Mat& mask = detection_masks_cpu_[i];
        int v = (int) mask.at<uchar>(f.y,f.x);
        if(v > 100) // pixel value is 255
        {
          features_in_mask[i]++; 
        }
      }
    }
    double distribution_factor = 0; ; 
    for(int i = 0; i < features_in_mask.size(); ++i)
    {
      if(features_in_mask[i] > min_features_in_mask_distribution_)
      {
        distribution_factor = distribution_factor+1; 
      }
    }
    distribution_factor = distribution_factor/((double) features_in_mask.size());

    if( distribution_factor <= min_feature_distribution_ || cur_frame_->seen_landmarks_.size() <= min_features_to_track_only_)
    { 
      detect_further_features();
    }
    else
    {
      // if the previous frame was a keyframe -> delete the seen landmarks that are only seen once by this keyframe!
      if(graph_frames_[graph_frames_.size()-1]->is_keyframe_)
      {
        mutex_frames_with_once_seen_landmarks_.lock();
        frames_with_once_seen_landmarks_.push_back(graph_frames_[graph_frames_.size()-1]);
        mutex_frames_with_once_seen_landmarks_.unlock();
      }
    }
  }  
}

void VisualOdometry::next_iteration_preparation()
{
  // save the old images for the next iteration
  l_im_.copyTo(l_im_prev_);
  r_im_.copyTo(r_im_prev_);

  // save current frame to the vector!
  mutex_graph_.lock();
  graph_frames_.push_back(cur_frame_);
  mutex_graph_.unlock();
}

void VisualOdometry::triangulate_stereo(cv::Point2f& lp, cv::Point2f& rp, Eigen::Vector3d& lcam_p)
{
  // get intrinsic camera parameters, The stereo camera images are rectified!
  double& fx = l_cam_rect_.at<double>(0,0);
  double& fy = l_cam_rect_.at<double>(1,1);
  double& cx = l_cam_rect_.at<double>(0,2);
  double& cy = l_cam_rect_.at<double>(1,2);
  double& baseline = left_cam_T_right_cam_rect_(0,3);
  // build the system-matrix
  Eigen::MatrixXd M(4,3);
  M.setZero();
  Eigen::Vector4d b; 
  b.setZero();
  M(0,0) = -1.0f; M(0,2) = ((double)lp.x - cx)/fx;
  M(1,1) = -1.0f; M(1,2) = ((double)lp.y - cy)/fy;
  M(2,0) = -1.0f; M(2,2) = ((double)rp.x - cx)/fx;
  M(3,1) = -1.0f; M(3,2) = ((double)rp.y - cy)/fy;
  b(2) = -baseline;
  // solve the overdetermined system
  lcam_p = M.colPivHouseholderQr().solve(b);
}

void VisualOdometry::triangulate_stereo_box(cv::Point2f& lp, cv::Point2f& rp, Eigen::Vector3d& lcam_p, ibex::IntervalVector& lcam_b)
{
  // initialize the 3D-box
  lcam_b = ibex::IntervalVector(3);
  ibex::Interval& X = lcam_b[0];
  ibex::Interval& Y = lcam_b[1];
  ibex::Interval& Z = lcam_b[2];

  // get intrinsic camera parameters, The stereo camera images are rectified!
  double& fx = l_cam_rect_.at<double>(0,0);
  double& fy = l_cam_rect_.at<double>(1,1);
  double& cx = l_cam_rect_.at<double>(0,2);
  double& cy = l_cam_rect_.at<double>(1,2);
  double& baseline = left_cam_T_right_cam_rect_(0,3);
  
  // compute interval pixels
  ibex::Interval pxl(lp.x); pxl.inflate(visual_feature_uncertainty_radius_left_im_);
  ibex::Interval pyl(lp.y); pyl.inflate(visual_feature_uncertainty_radius_left_im_);
  ibex::Interval pxr(rp.x); pxr.inflate(visual_feature_uncertainty_radius_right_im_);
  ibex::Interval pyr(rp.y); pyr.inflate(visual_feature_uncertainty_radius_right_im_+max_vertical_pixel_error_);

  // rectified stereo constraint
  pyl = pyl & pyr;
  pyr = pyl & pyr;

  // depth-scale constraint for rectified stereo
  Z = Z & (baseline*fx * 1/(pxl-pxr));

  // constraints on Y-coordinate
  Y = Y & (Z*(pyl-cy)/fy) & (Z*(pyr-cy)/fy); // forward
  Z = Z & (Y*fy/(pyl-cy)) & (Y*fy/(pyr-cy)); // backward

  // constraints on X-coordinate
  X = X & (Z*(pxl-cx)/fx) & ((Z*(pxr-cx)/fx+baseline)); // forward
  Z = Z & (X*fx/(pxl-cx)) & ((X-baseline)*fx/(pxr-cx)); // backward
}

void VisualOdometry::project_stereo(Eigen::Vector3d& lcam_p, cv::Point2f& lp, cv::Point2f& rp)
{
  Eigen::Matrix3d K;
  K.setZero();
  double& fx = l_cam_rect_.at<double>(0,0);
  double& fy = l_cam_rect_.at<double>(1,1);
  double& cx = l_cam_rect_.at<double>(0,2);
  double& cy = l_cam_rect_.at<double>(1,2);
  double& baseline = left_cam_T_right_cam_rect_(0,3);

  K(0,0) = fx; K(0,2) = cx;
  K(1,1) = fy; K(1,2) = cy;
  K(2,2) = 1.0f;

  Eigen::Vector3d lpix;
  lpix = K*lcam_p;
  lpix = lpix*1/lpix(2);
  Eigen::Vector3d rpix;
  rpix = K*(lcam_p+Eigen::Vector3d(-baseline,0,0));
  rpix = rpix*1/rpix(2);

  lp.x = (float)lpix(0); lp.y = (float)lpix(1);
  rp.x = (float)rpix(0); rp.y = (float)rpix(1);
}

void VisualOdometry::graph_optimization_stereo_odometry()
{
  // create optimizer, solver and graph
  g2o::SparseOptimizer optimizer;
  std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver; // BlockSolver_6_3
  linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>>();//LinearSolverCSparse//LinearSolverCholmod//LinearSolverDense//LinearSolverEigen
  g2o::OptimizationAlgorithmLevenberg* solver;
  solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));
  optimizer.setAlgorithm(solver);
  int vertex_id = 0; 

  mutex_graph_.lock();
  Graph::Frame* prev_f = graph_frames_[graph_frames_.size()-2];
  Graph::Frame* cur_f = graph_frames_[graph_frames_.size()-1];

  // insert prev frame vertex -> set fixed and to identity! Optimization is in lcam of the previous frame!
  g2o::VertexSE3Expmap* v_f_prev = new g2o::VertexSE3Expmap();
  g2o::Isometry3 prev_lcam_T_prev_lcam;
  prev_lcam_T_prev_lcam.setIdentity();
  g2o::SE3Quat prev_lcam_T_prev_lcam_quat(prev_lcam_T_prev_lcam.linear(),prev_lcam_T_prev_lcam.translation());
  v_f_prev->setEstimate(prev_lcam_T_prev_lcam_quat);
  v_f_prev->setId(vertex_id);
  vertex_id++;
  v_f_prev->setFixed(true);       
  optimizer.addVertex(v_f_prev);
  // insert current frame vertex
  // But first we assume constant movement model
  // compute the velocity between the previous and the pose before previous
  if(graph_frames_.size()>=3)
  {
    Graph::Frame* bef_prev_f = graph_frames_[graph_frames_.size()-3];
    Eigen::Vector3d t = prev_f->car_prev_T_car_cur_.translation();
    t = t/(prev_f->timestamp_ - bef_prev_f->timestamp_) * (cur_f->timestamp_-prev_f->timestamp_);
    cur_f->car_prev_T_car_cur_.translation() = t; 

    Eigen::Matrix3d R = prev_f->car_prev_T_car_cur_.linear();
    Eigen::Vector3d r; 
    r(0) = atan2(R(2,1),R(2,2));
    r(1) = -asin(R(2,0));
    r(2) = atan2(R(1,0),R(0,0));
    r = r/(prev_f->timestamp_ - bef_prev_f->timestamp_) * (cur_f->timestamp_-prev_f->timestamp_);
    Eigen::Quaternion<double> q = Eigen::AngleAxisd(r(2),Eigen::Vector3d::UnitZ())
                            *Eigen::AngleAxisd(r(1),Eigen::Vector3d::UnitY())
                            *Eigen::AngleAxisd(r(0),Eigen::Vector3d::UnitX());
    cur_f->car_prev_T_car_cur_.linear() = q.matrix();
  }
  else
  {
    cur_f->car_prev_T_car_cur_ = prev_f->car_prev_T_car_cur_;
  }
  
  g2o::VertexSE3Expmap* v_f_cur = new g2o::VertexSE3Expmap();
  Eigen::Affine3d lcam_prev_T_lcam_cur = car_T_left_cam_.inverse()*(cur_f->car_prev_T_car_cur_)*car_T_left_cam_;
  g2o::SE3Quat lcam_cur_T_lcam_prev_quat(lcam_prev_T_lcam_cur.inverse().linear(),lcam_prev_T_lcam_cur.inverse().translation());
  v_f_cur->setEstimate(lcam_cur_T_lcam_prev_quat);
  v_f_cur->setId(vertex_id);
  vertex_id++;
  v_f_cur->setFixed(false);       
  optimizer.addVertex(v_f_cur);
  // iterate through the landmarks of the current frame -> if landmarks is seen also from previous frame, insert as vertex
  std::map<Graph::Landmark*,std::vector<g2o::EdgeStereoSE3ProjectXYZ*>> landmark_edge_map; 
  for(Graph::Landmark* landmark : cur_f->seen_landmarks_)
  {
    std::vector<Graph::Landmark*>::iterator l_it = std::find(prev_f->seen_landmarks_.begin(), prev_f->seen_landmarks_.end(), landmark);
    if(l_it != prev_f->seen_landmarks_.end())
    {
      landmark_edge_map[landmark].resize(2);
      // the landmark was also seen from previous frame
      // get the observations
      Graph::Observation* prev_obs = landmark->obsv_map_[prev_f];
      Graph::Observation* cur_obs = landmark->obsv_map_[cur_f]; 

      // create a landmark vertex
      g2o::VertexSBAPointXYZ* v_l = new g2o::VertexSBAPointXYZ();
      v_l->setId(vertex_id);
      vertex_id++;
      g2o::Vector3 pos(prev_obs->lcam_point_);
      v_l->setEstimate(pos);
      v_l->setMarginalized(false);
      optimizer.addVertex(v_l);

      // insert edge between previous frame and landmark
      {
      g2o::EdgeStereoSE3ProjectXYZ* l_e_prev = new g2o::EdgeStereoSE3ProjectXYZ; 
      l_e_prev->vertices()[0] = v_l;
      l_e_prev->vertices()[1] = v_f_prev;
      Eigen::Matrix<double,3,1> prev_obs_mat;
      prev_obs_mat << prev_obs->lf_.x, prev_obs->lf_.y, prev_obs->rf_.x;
      l_e_prev->setMeasurement(prev_obs_mat);
      Eigen::Matrix3d info;
      info.setIdentity();
      l_e_prev->setInformation(info);
      g2o::RobustKernelGemanMcClure* rk = new g2o::RobustKernelGemanMcClure;
      l_e_prev->setRobustKernel(rk);
      rk->setDelta(sqrt(1.0)); 
      l_e_prev->fx = l_cam_rect_.at<double>(0,0);
      l_e_prev->fy = l_cam_rect_.at<double>(1,1);
      l_e_prev->cx = l_cam_rect_.at<double>(0,2);
      l_e_prev->cy = l_cam_rect_.at<double>(1,2);
      l_e_prev->bf = l_cam_rect_.at<double>(0,0)*left_cam_T_right_cam_rect_(0,3);
      optimizer.addEdge(l_e_prev);
      landmark_edge_map[landmark][0] = l_e_prev;
      }

      // insert edge between current frame and landmark
      {
      g2o::EdgeStereoSE3ProjectXYZ* l_e_cur = new g2o::EdgeStereoSE3ProjectXYZ; 
      l_e_cur->vertices()[0] = v_l;
      l_e_cur->vertices()[1] = v_f_cur;
      Eigen::Matrix<double,3,1> cur_obs_mat;
      cur_obs_mat << cur_obs->lf_.x, cur_obs->lf_.y, cur_obs->rf_.x;
      l_e_cur->setMeasurement(cur_obs_mat);
      Eigen::Matrix3d info;
      info.setIdentity();
      l_e_cur->setInformation(info);
      g2o::RobustKernelGemanMcClure* rk = new g2o::RobustKernelGemanMcClure;
      l_e_cur->setRobustKernel(rk);
      rk->setDelta(sqrt(1.0)); 
      l_e_cur->fx = l_cam_rect_.at<double>(0,0);
      l_e_cur->fy = l_cam_rect_.at<double>(1,1);
      l_e_cur->cx = l_cam_rect_.at<double>(0,2);
      l_e_cur->cy = l_cam_rect_.at<double>(1,2);
      l_e_cur->bf = l_cam_rect_.at<double>(0,0)*left_cam_T_right_cam_rect_(0,3);
      optimizer.addEdge(l_e_cur);
      landmark_edge_map[landmark][1] = l_e_cur;
      }
    }
  }
  mutex_graph_.unlock();
  
  // optimize with Huber Cost Function
  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.optimize(10);

  // include saturate cost function
  for(std::pair<Graph::Landmark*,std::vector<g2o::EdgeStereoSE3ProjectXYZ*>> landmark_edge_pair : landmark_edge_map)
  {
    g2o::EdgeStereoSE3ProjectXYZ* l_e_prev = landmark_edge_pair.second[0];
    g2o::EdgeStereoSE3ProjectXYZ* l_e_cur = landmark_edge_pair.second[1];

    g2o::RobustKernelSaturated* rk_cur = new g2o::RobustKernelSaturated;
    l_e_cur->setRobustKernel(rk_cur);
    rk_cur->setDelta(sqrt(max_residual_stereo_odom_));

    g2o::RobustKernelSaturated* rk_prev = new g2o::RobustKernelSaturated;
    l_e_prev->setRobustKernel(rk_prev);
    rk_prev->setDelta(sqrt(max_residual_stereo_odom_));
  }

  // optimize with Saturate Cost Function
  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.optimize(20);

  Eigen::Isometry3d lcam_prev_T_lcam_cur_quat = v_f_cur->estimate().inverse();
  Eigen::Affine3d car_prev_T_car_cur;
  car_prev_T_car_cur.matrix() = (car_T_left_cam_.matrix() * lcam_prev_T_lcam_cur_quat * car_T_left_cam_.inverse().matrix()).matrix();
  double bad_measurements_counter = 0.0; 
  for(std::pair<Graph::Landmark*,std::vector<g2o::EdgeStereoSE3ProjectXYZ*>> landmark_edge_pair : landmark_edge_map)
  {
    g2o::EdgeStereoSE3ProjectXYZ* l_e_prev = landmark_edge_pair.second[0];
    g2o::EdgeStereoSE3ProjectXYZ* l_e_cur = landmark_edge_pair.second[1];

    if(abs(l_e_cur->error()(0)) >= max_residual_stereo_odom_ || abs(l_e_cur->error()(1)) >= max_residual_stereo_odom_ 
        || abs(l_e_cur->error()(2)) >= max_residual_stereo_odom_)
    {
      bad_measurements_counter = bad_measurements_counter+1.0; 
    }
  }

  // Bad meaurements percentage
  double bad_measurements_portion = bad_measurements_counter/(double)landmark_edge_map.size();
  if(bad_measurements_portion < max_bad_measurements_portion_stereo_odom_) // only update the relative pose if the portion of bad measurements is less than 20%
  {
    mutex_graph_.lock();
    cur_f->car_prev_T_car_cur_ = car_prev_T_car_cur; 
    mutex_graph_.unlock(); 
  }
  //std::cout << "computed: " << std::endl << cur_f->car_prev_T_car_cur_.matrix() << std::endl << "---" << std::endl;

  // save the previous keyframe index to keyframe_batch_start_index_ to signalize the batch-wise bunde adjustment, that a new bundle is ready.  
  if(cur_f->is_keyframe_) // if the current frame is a keyframe, the last batch just terminated
  {
    for(int i = graph_frames_.size()-2; i>=0; i--) // iterate backwards through the frames to search for the last keyframe
    {
      if(graph_frames_[i]->is_keyframe_) // found the previous keyframe -> save that frame to vector
      {
        mutex_batch_start_end_index_.lock();
        batch_start_end_index_.push_back(std::make_pair(i,graph_frames_.size()-1)); // save the index of the prevous keyframe and the end of the batch
        mutex_batch_start_end_index_.unlock();
        break; 
      }
    }
  }
}

/****delete only once seen landmarks in a separate thread*****/
void VisualOdometry::delete_once_seen_landmarks()
{
  while(ros::ok())
  {
    mutex_frames_with_once_seen_landmarks_.lock();
    if(frames_with_once_seen_landmarks_.size() > 0)
    {
      mutex_delete_landmarks_.lock();
      // get the frame for which we need to delete the once seen landmarks
      Graph::Frame* frame = frames_with_once_seen_landmarks_[0];
      frames_with_once_seen_landmarks_.erase(frames_with_once_seen_landmarks_.begin());
      mutex_frames_with_once_seen_landmarks_.unlock(); 

      // iterate through the seen landmarks 
      for(int i = 0; i<frame->seen_landmarks_.size(); i++)
      {
        Graph::Landmark* l = frame->seen_landmarks_[i];
        if(l->obsv_map_.size() <= 1)
        {
          // delete the landmark and the corresponding observations!
          // delete in the vectors
          mutex_graph_.lock();
          Graph::Observation* obs = l->obsv_map_[frame];
          //std::vector<Graph::Observation*>::iterator obs_it = std::find(graph_observations_.begin(), graph_observations_.end(), obs);
          //std::vector<Graph::Landmark*>::iterator l_it = std::find(graph_landmarks_.begin(), graph_landmarks_.end(), l);

          //graph_observations_.erase(obs_it);
          //graph_landmarks_.erase(l_it);
          // erase the landmark from the frame
          frame->seen_landmarks_.erase(frame->seen_landmarks_.begin()+i);

          i--;

          // delete the objects
          delete obs;
          delete l;
          mutex_graph_.unlock();
        }
      }
      mutex_delete_landmarks_.unlock();
    }
    else
    {
      mutex_frames_with_once_seen_landmarks_.unlock();
    }
    ros::Duration(0.5).sleep();
  }
}

/****Bundle adjustment and contraction in a separate thread*****/
void VisualOdometry::graph_optimization_batch_bundle_adjustment()
{
  //cv::namedWindow("test");
  while(ros::ok())
  {
    mutex_batch_start_end_index_.lock();
    if(batch_start_end_index_.size() > 0)
    {
      // get the start and end index of the batch
      int batch_start = batch_start_end_index_[0].first;
      int batch_end = batch_start_end_index_[0].second;
      batch_start_end_index_.erase(batch_start_end_index_.begin());
      mutex_batch_start_end_index_.unlock();

      // create optimizer, solver and graph
      g2o::SparseOptimizer optimizer;
      std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver; // BlockSolver_6_3
      linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>>();//LinearSolverCSparse//LinearSolverCholmod//LinearSolverDense//LinearSolverEigen
      g2o::OptimizationAlgorithmLevenberg* solver;
      solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));
      optimizer.setAlgorithm(solver);
      // Set the default terminate action
      g2o::SparseOptimizerTerminateAction* terminateAction = new g2o::SparseOptimizerTerminateAction;
      terminateAction->setGainThreshold(0.00001);
      optimizer.addPostIterationAction(terminateAction);
      int vertex_id = 0;

      // gather the poses that need to be optimized into a vector
      std::vector<Graph::Frame*> frames_opt;
      frames_opt.reserve(batch_end-batch_start+1);
      for(int i = batch_start; i <= batch_end; i++)
      {
        frames_opt.emplace_back(graph_frames_[i]);
      }

      // The very first frame is the corresponding keyframe
      Graph::Frame* key_frame = frames_opt[0]; 
      // none of the other frames should see landmarks that are not seen by key_frame!
      // so:  1.) insert all frame nodes to the graph
      //      2.) insert all landmarks connected to key_frame to graph
      //      3.) insert all observation edges by iterating through each frame and insert observation to each seen landmark

      // need to save a map from Frame* to VertexSE3Expmap*
      std::map<Graph::Frame*,g2o::VertexSE3Expmap*> frame_vertex_map; 

      // 1.) insert all pose-nodes (frames) to the graph
      // first we insert the key-frame pose-node  -> set fixed and to identity! Optimization is in lcam of the previous frame!
      g2o::VertexSE3Expmap* v_kf = new g2o::VertexSE3Expmap();
      g2o::Isometry3 lcam_kf_T_lcam_kf;
      lcam_kf_T_lcam_kf.setIdentity(); // set to identity
      g2o::SE3Quat lcam_kf_T_lcam_kf_quat(lcam_kf_T_lcam_kf.linear(),lcam_kf_T_lcam_kf.translation());
      v_kf->setEstimate(lcam_kf_T_lcam_kf_quat);
      v_kf->setId(vertex_id);
      vertex_id++;
      v_kf->setFixed(true);       
      optimizer.addVertex(v_kf);
      frame_vertex_map[key_frame] = v_kf; // save the frame-ptr to vertex map
      // second: insert the other frame into the graph
      // as the frames only save the transfromation from the previous frame to the current frame, the transormations need to be concatenated
      Eigen::Affine3d lcam_kf_T_lcam_f;
      lcam_kf_T_lcam_f.setIdentity();
      for(int i = 1; i < frames_opt.size(); i++) // frames_opt has the identical temporal order as graph_frames_
      {
        Graph::Frame* frame = frames_opt[i];
        Eigen::Affine3d lcam_prev_T_lcam_cur = car_T_left_cam_.inverse()*frame->car_prev_T_car_cur_*car_T_left_cam_;
        lcam_kf_T_lcam_f = lcam_kf_T_lcam_f*lcam_prev_T_lcam_cur; // concatednation of the transformations along the batch -> pose describen in kf!

        // insert the frame vertex
        g2o::VertexSE3Expmap* v_f = new g2o::VertexSE3Expmap();
        g2o::SE3Quat lcam_f_T_lcam_kf_quat(lcam_kf_T_lcam_f.inverse().linear(),lcam_kf_T_lcam_f.inverse().translation());
        v_f->setEstimate(lcam_f_T_lcam_kf_quat);
        v_f->setId(vertex_id);
        vertex_id++;
        v_f->setFixed(false);       
        optimizer.addVertex(v_f);
        frame_vertex_map[frame] = v_f;
      }

      // 2.) insert all landmarks connected to key_frame to graph
      mutex_delete_landmarks_.lock();
      // need to save a map from Landmark* to VertexSBAPointXYZ*
      std::map<Graph::Landmark*,g2o::VertexSBAPointXYZ*> landmark_vertex_map;
      for(Graph::Landmark* l : key_frame->seen_landmarks_)
      {
        // create a landmark vertex
        g2o::VertexSBAPointXYZ* v_l = new g2o::VertexSBAPointXYZ();
        v_l->setId(vertex_id);
        vertex_id++;
        g2o::Vector3 pos(l->obsv_map_[key_frame]->lcam_point_);
        v_l->setEstimate(pos);
        v_l->setMarginalized(false);
        v_l->setFixed(true);
        optimizer.addVertex(v_l);
        landmark_vertex_map[l] = v_l; 
      }

      // 3.) insert all observation edges by iterating through each frame and insert observation to each seen landmark
      std::map<Graph::Landmark*,std::vector<g2o::EdgeStereoSE3ProjectXYZ*>> landmark_edge_map;
      std::map<Graph::Observation*,g2o::EdgeStereoSE3ProjectXYZ*> observation_edge_map;
      for(Graph::Frame* f : frames_opt) // iterate through each frame, also the keyframe!
      {
        for(Graph::Landmark* l : f->seen_landmarks_)
        {
          // also the next keyframe is inserted, so check here, if the seen landmark is in landmarl_vertex_map
          if(landmark_vertex_map.find(l) != landmark_vertex_map.end())
          {
            // insert edge between frame and landmark
            g2o::EdgeStereoSE3ProjectXYZ* l_e_f = new g2o::EdgeStereoSE3ProjectXYZ; 
            l_e_f->vertices()[0] = landmark_vertex_map[l];
            l_e_f->vertices()[1] = frame_vertex_map[f];
            Eigen::Matrix<double,3,1> obs_mat;
            obs_mat << l->obsv_map_[f]->lf_.x, l->obsv_map_[f]->lf_.y, l->obsv_map_[f]->rf_.x;
            l_e_f->setMeasurement(obs_mat);
            Eigen::Matrix3d info;
            info.setIdentity();
            l_e_f->setInformation(info);
            g2o::RobustKernelGemanMcClure* rk = new g2o::RobustKernelGemanMcClure;
            l_e_f->setRobustKernel(rk);
            rk->setDelta(sqrt(1.0)); 
            l_e_f->fx = l_cam_rect_.at<double>(0,0);
            l_e_f->fy = l_cam_rect_.at<double>(1,1);
            l_e_f->cx = l_cam_rect_.at<double>(0,2);
            l_e_f->cy = l_cam_rect_.at<double>(1,2);
            l_e_f->bf = l_cam_rect_.at<double>(0,0)*left_cam_T_right_cam_rect_(0,3);
            optimizer.addEdge(l_e_f);
            landmark_edge_map[l].push_back(l_e_f);
            observation_edge_map[l->obsv_map_[f]] = l_e_f;
          }
        } 
      }

      // optimize with Geman Mc Clure Cost Function and fixed landmarks
      optimizer.setVerbose(false);
      optimizer.initializeOptimization();
      //std::cout << "Geman Mc Clure Cost Function and fixed landmarks: " << std::endl;
      optimizer.optimize(20);

      // make landmarks flexible
      for(std::pair<Graph::Landmark*,g2o::VertexSBAPointXYZ*> landmark_vertex_pair : landmark_vertex_map)
      {
        landmark_vertex_pair.second->setFixed(false);
      }

      // include saturate cost function
      for(std::pair<Graph::Landmark*,std::vector<g2o::EdgeStereoSE3ProjectXYZ*>> landmark_edge_pair : landmark_edge_map)
      {
        for(g2o::EdgeStereoSE3ProjectXYZ* l_e_f : landmark_edge_pair.second)
        {
          g2o::RobustKernelSaturated* rk_cur = new g2o::RobustKernelSaturated;
          l_e_f->setRobustKernel(rk_cur);
          rk_cur->setDelta(max_residual_bundle_adjustment_);
        }
      }

      // optimize with Saturate Cost Function and flexible landmarks
      optimizer.setVerbose(false);
      optimizer.initializeOptimization();
      //std::cout << "Saturate Cost Function and flexible landmarks" << std::endl;
      optimizer.optimize(20);

      // Now the batch is optimized! Two things need to be done now:
      // 1.) iterate through each landmark and check if all observations are below a threshold, if that is the case the landmark is good, else do not use it!
      // 2.) compute the relative odometry of the optimized poses and correct the odometry data in the frames
      // 1.) iterate through all landmarks and mark problematic landmarks
      for(std::pair<Graph::Landmark*,std::vector<g2o::EdgeStereoSE3ProjectXYZ*>> landmark_edge_pair : landmark_edge_map)
      {
        for(g2o::EdgeStereoSE3ProjectXYZ* e : landmark_edge_pair.second)
        {
          if(abs(e->error()(0)) > max_residual_bundle_adjustment_ || abs(e->error()(1)) > max_residual_bundle_adjustment_ 
                || abs(e->error()(2)) > max_residual_bundle_adjustment_) // at least one edge has higher value, dismiss the landmark
          {
            landmark_edge_pair.first->good_ = false; 
            break; 
          }
        }
      } 
      // 2.) compute the relative odometry of the optimized poses and correct the odometry data in the frames
      for(int i = 1; i < frames_opt.size(); ++i) // 0 is the keyframe
      {
        // a minimal number of good landmarks need to be connected with the frames_opt[i]
        // if that is not the case, we  should not update frames_opt[i]->car_prev_T_car_cur_ !!
        int good_landmarks_counter = 0; 
        for(Graph::Landmark* l : frames_opt[i]->seen_landmarks_)
        {
          if(l->good_)
          {
            good_landmarks_counter++; 
            if(good_landmarks_counter > min_connected_good_landmarks_)
            {
              break; 
            }
          }
        }

        if(good_landmarks_counter > min_connected_good_landmarks_)
        {
          g2o::VertexSE3Expmap* prev_vertex = frame_vertex_map[frames_opt[i-1]];
          g2o::VertexSE3Expmap* cur_vertex = frame_vertex_map[frames_opt[i]];

          // transform of the previous frame in kf
          Eigen::Isometry3d lcam_kf_T_lcam_prev_f_quat = prev_vertex->estimate().inverse();
          Eigen::Affine3d car_kf_T_car_prev_f;
          car_kf_T_car_prev_f.matrix() = (car_T_left_cam_.matrix() * lcam_kf_T_lcam_prev_f_quat * car_T_left_cam_.inverse().matrix()).matrix();

          // transform of the current frame in kf
          Eigen::Isometry3d lcam_kf_T_lcam_cur_f_quat = cur_vertex->estimate().inverse();
          Eigen::Affine3d car_kf_T_car_cur_f;
          car_kf_T_car_cur_f.matrix() = (car_T_left_cam_.matrix() * lcam_kf_T_lcam_cur_f_quat * car_T_left_cam_.inverse().matrix()).matrix();

          // compute the relative tranform from prev and cur
          Eigen::Affine3d car_prev_T_car_cur = car_kf_T_car_prev_f.inverse()*car_kf_T_car_cur_f;

          // copy the corrected relative transformation to the frames
          frames_opt[i]->car_prev_T_car_cur_ = car_prev_T_car_cur; // WHAT TODO IF OPTIMIZATION DOES NOT CONVERGE?!
        }
      }

      // now the landmarks are qualified as good and not good
      // If at least one observation of a landmark has higher reprojection error, then the landmark is considered as not good
      // That means we can use the remaining good landmarks and observations for the interval-based odometry computation 
      #pragma omp parallel for
      for(int i = 1; i < frames_opt.size(); i++) // 0 is the keyframe
      {
        ibex::IntervalVector f1_lcam_p_f2_lcam(6);
        contract_consecutive_frames_odom(frames_opt[i-1], frames_opt[i], f1_lcam_p_f2_lcam);
        // transform to the laser-frame
        // compute first the matrix f1_lcam_T_f2_lcam
        ibex::IntervalMatrix f1_lcam_T_f2_lcam(4,4);
        codac::Ctc3DRotation ctc_rot; 
        ibex::IntervalMatrix f1_lcam_R_f2_lcam(3,3);
        ibex::IntervalVector rot_vec = f1_lcam_p_f2_lcam.subvector(3,5);
        ctc_rot.contract(rot_vec,f1_lcam_R_f2_lcam);
        f1_lcam_T_f2_lcam = ibex::Matrix::eye(4);
        f1_lcam_T_f2_lcam[0][0] = f1_lcam_R_f2_lcam[0][0]; f1_lcam_T_f2_lcam[0][1] = f1_lcam_R_f2_lcam[0][1]; f1_lcam_T_f2_lcam[0][2] = f1_lcam_R_f2_lcam[0][2];
        f1_lcam_T_f2_lcam[1][0] = f1_lcam_R_f2_lcam[1][0]; f1_lcam_T_f2_lcam[1][1] = f1_lcam_R_f2_lcam[1][1]; f1_lcam_T_f2_lcam[1][2] = f1_lcam_R_f2_lcam[1][2];
        f1_lcam_T_f2_lcam[2][0] = f1_lcam_R_f2_lcam[2][0]; f1_lcam_T_f2_lcam[2][1] = f1_lcam_R_f2_lcam[2][1]; f1_lcam_T_f2_lcam[2][2] = f1_lcam_R_f2_lcam[2][2];
        f1_lcam_T_f2_lcam[0][3] = f1_lcam_p_f2_lcam[0];
        f1_lcam_T_f2_lcam[1][3] = f1_lcam_p_f2_lcam[1];
        f1_lcam_T_f2_lcam[2][3] = f1_lcam_p_f2_lcam[2];

        // compute the lcam to laser transform
        Eigen::Affine3d lcam_T_laser_eig = (car_T_left_cam_.inverse()*car_T_laser_);
        Eigen::Affine3d laser_T_lcam_eig = lcam_T_laser_eig.inverse();
        ibex::Matrix lcam_T_laser(4,4), laser_T_lcam(4,4);
        lcam_T_laser = ibex::Matrix::eye(4);
        laser_T_lcam = ibex::Matrix::eye(4);
        lcam_T_laser[0][0] = lcam_T_laser_eig.matrix()(0,0); lcam_T_laser[0][1] = lcam_T_laser_eig.matrix()(0,1); lcam_T_laser[0][2] = lcam_T_laser_eig.matrix()(0,2);
        lcam_T_laser[1][0] = lcam_T_laser_eig.matrix()(1,0); lcam_T_laser[1][1] = lcam_T_laser_eig.matrix()(1,1); lcam_T_laser[1][2] = lcam_T_laser_eig.matrix()(1,2);
        lcam_T_laser[2][0] = lcam_T_laser_eig.matrix()(2,0); lcam_T_laser[2][1] = lcam_T_laser_eig.matrix()(2,1); lcam_T_laser[2][2] = lcam_T_laser_eig.matrix()(2,2);
        lcam_T_laser[0][3] = lcam_T_laser_eig.matrix()(0,3);
        lcam_T_laser[1][3] = lcam_T_laser_eig.matrix()(1,3);
        lcam_T_laser[2][3] = lcam_T_laser_eig.matrix()(2,3);

        laser_T_lcam[0][0] = laser_T_lcam_eig.matrix()(0,0); laser_T_lcam[0][1] = laser_T_lcam_eig.matrix()(0,1); laser_T_lcam[0][2] = laser_T_lcam_eig.matrix()(0,2);
        laser_T_lcam[1][0] = laser_T_lcam_eig.matrix()(1,0); laser_T_lcam[1][1] = laser_T_lcam_eig.matrix()(1,1); laser_T_lcam[1][2] = laser_T_lcam_eig.matrix()(1,2);
        laser_T_lcam[2][0] = laser_T_lcam_eig.matrix()(2,0); laser_T_lcam[2][1] = laser_T_lcam_eig.matrix()(2,1); laser_T_lcam[2][2] = laser_T_lcam_eig.matrix()(2,2);
        laser_T_lcam[0][3] = laser_T_lcam_eig.matrix()(0,3);
        laser_T_lcam[1][3] = laser_T_lcam_eig.matrix()(1,3);
        laser_T_lcam[2][3] = laser_T_lcam_eig.matrix()(2,3);

        ibex::IntervalMatrix f1_laser_T_f2_laser(4,4);
        f1_laser_T_f2_laser = laser_T_lcam * f1_lcam_T_f2_lcam * lcam_T_laser; 
        ibex::IntervalMatrix f1_laser_R_f2_laser(3,3);
        f1_laser_R_f2_laser[0][0] = f1_laser_T_f2_laser[0][0]; f1_laser_R_f2_laser[0][1] = f1_laser_T_f2_laser[0][1]; f1_laser_R_f2_laser[0][2] = f1_laser_T_f2_laser[0][2];
        f1_laser_R_f2_laser[1][0] = f1_laser_T_f2_laser[1][0]; f1_laser_R_f2_laser[1][1] = f1_laser_T_f2_laser[1][1]; f1_laser_R_f2_laser[1][2] = f1_laser_T_f2_laser[1][2];
        f1_laser_R_f2_laser[2][0] = f1_laser_T_f2_laser[2][0]; f1_laser_R_f2_laser[2][1] = f1_laser_T_f2_laser[2][1]; f1_laser_R_f2_laser[2][2] = f1_laser_T_f2_laser[2][2];
        ibex::IntervalVector rot_vec_laser(3);
        f1_laser_R_f2_laser.inflate(0.00001);
        ctc_rot.contract(rot_vec_laser,f1_laser_R_f2_laser);

        // Save interval-odometry to frame
        ibex::IntervalVector f1_laser_p_f2_laser(6);
        f1_laser_p_f2_laser[0] = f1_laser_T_f2_laser[0][3];
        f1_laser_p_f2_laser[1] = f1_laser_T_f2_laser[1][3];
        f1_laser_p_f2_laser[2] = f1_laser_T_f2_laser[2][3];
        f1_laser_p_f2_laser[3] = rot_vec_laser[0];
        f1_laser_p_f2_laser[4] = rot_vec_laser[1];
        f1_laser_p_f2_laser[5] = rot_vec_laser[2]; 
        frames_opt[i]->laser_prev_p_laser_cur = f1_laser_p_f2_laser; 
        frames_opt[i]->lcam_prev_p_lcam_cur = f1_lcam_p_f2_lcam;

        frames_opt[i]->computed_interval_odometry_ = true;
      }
      frames_opt[0]->computed_interval_odometry_ = true;
      mutex_delete_landmarks_.unlock();
    }
    else
    {
      mutex_batch_start_end_index_.unlock();
    }
    ros::Duration(0.1).sleep();
  }
}

void VisualOdometry::contract_consecutive_frames_odom(Graph::Frame* f1, Graph::Frame* f2, ibex::IntervalVector& f1_lcam_p_f2_lcam)
{

  // create the stereo vo contractor
  double f = l_cam_rect_.at<double>(0,0);
  double cx = l_cam_rect_.at<double>(0,2);
  double cy = l_cam_rect_.at<double>(1,2);
  double b = left_cam_T_right_cam_rect_.matrix()(0,3); 
  codac::Ctc3DStereoVO ctc_vo(b,f,cx,cy);

  // save the good observations in a vector of observations
  std::vector<std::pair<Graph::Observation*,Graph::Observation*>> obsv_pairs; // first: observation from f1; second: observation from f2
  obsv_pairs.reserve(f2->seen_landmarks_.size());
  // iterate through the seen landmarks of f2 
  for(Graph::Landmark* l : f2->seen_landmarks_)
  {
    if(l->good_) // only continue if the landmark was always observed well 
    {
      // check if l was seen by f1
      std::vector<Graph::Landmark*>::iterator l_it = std::find(f1->seen_landmarks_.begin(), f1->seen_landmarks_.end(), l);
      if(l_it != f1->seen_landmarks_.end()) // if this holds true, the landmark was seen by f1
      {
        obsv_pairs.emplace_back(std::make_pair(l->obsv_map_[f1],l->obsv_map_[f2])); // the observations are lcam!
        // CAUTION!!!! CAMERA FRAME IS DIFFERENTLY ORIENTED THAN LIDAR FRAME!!!!!!!!
      }
    }
  } 

  //-------------------------------------------------------------
  // compute stereo-odoemtry optimization here again
  // create optimizer, solver and graph
  g2o::SparseOptimizer optimizer;
  std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver; // BlockSolver_6_3
  linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>>();//LinearSolverCSparse//LinearSolverCholmod//LinearSolverDense//LinearSolverEigen
  g2o::OptimizationAlgorithmLevenberg* solver;
  solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));
  optimizer.setAlgorithm(solver);
  int vertex_id = 0; 

  // insert f1 frame vertex -> set fixed and to identity! Optimization is in lcam of the previous frame!
  g2o::VertexSE3Expmap* v_f1 = new g2o::VertexSE3Expmap();
  g2o::Isometry3 f1_lcam_T_f1_lcam;
  f1_lcam_T_f1_lcam.setIdentity();
  g2o::SE3Quat f1_lcam_T_f1_lcam_quat(f1_lcam_T_f1_lcam.linear(),f1_lcam_T_f1_lcam.translation());
  v_f1->setEstimate(f1_lcam_T_f1_lcam_quat);
  v_f1->setId(vertex_id);
  vertex_id++;
  v_f1->setFixed(true);       
  optimizer.addVertex(v_f1);

  // insert f2 frame vertex
  g2o::VertexSE3Expmap* v_f2 = new g2o::VertexSE3Expmap();
  Eigen::Affine3d lcam_f1_T_lcam_f2 = car_T_left_cam_.inverse()*(f2->car_prev_T_car_cur_)*car_T_left_cam_;
  g2o::SE3Quat lcam_f2_T_lcam_f1_quat(lcam_f1_T_lcam_f2.inverse().linear(),lcam_f1_T_lcam_f2.inverse().translation());
  v_f2->setEstimate(lcam_f2_T_lcam_f1_quat);
  v_f2->setId(vertex_id);
  vertex_id++;
  v_f2->setFixed(false);       
  optimizer.addVertex(v_f2);

  // insert landmark vertices and observation edges
  std::map<Graph::Observation*,g2o::EdgeStereoSE3ProjectXYZ*> obs_edge_map; 
  for(std::pair<Graph::Observation*,Graph::Observation*>& obsv_pair : obsv_pairs)
  {
    Graph::Observation* f1_obsv = obsv_pair.first;
    Graph::Observation* f2_obsv = obsv_pair.second; 
    // create a landmark vertex
    g2o::VertexSBAPointXYZ* v_l = new g2o::VertexSBAPointXYZ();
    v_l->setId(vertex_id);
    vertex_id++;
    g2o::Vector3 pos(f1_obsv->lcam_point_);
    v_l->setEstimate(pos);
    v_l->setMarginalized(false);
    optimizer.addVertex(v_l);

    // insert edge between previous frame and landmark
    {
    g2o::EdgeStereoSE3ProjectXYZ* l_e1 = new g2o::EdgeStereoSE3ProjectXYZ; 
    l_e1->vertices()[0] = v_l;
    l_e1->vertices()[1] = v_f1;
    Eigen::Matrix<double,3,1> f1_obs_mat;
    f1_obs_mat << f1_obsv->lf_.x, f1_obsv->lf_.y, f1_obsv->rf_.x;
    l_e1->setMeasurement(f1_obs_mat);
    Eigen::Matrix3d info;
    info.setIdentity();
    l_e1->setInformation(info);
    l_e1->fx = l_cam_rect_.at<double>(0,0);
    l_e1->fy = l_cam_rect_.at<double>(1,1);
    l_e1->cx = l_cam_rect_.at<double>(0,2);
    l_e1->cy = l_cam_rect_.at<double>(1,2);
    l_e1->bf = l_cam_rect_.at<double>(0,0)*left_cam_T_right_cam_rect_(0,3);
    optimizer.addEdge(l_e1);
    obs_edge_map[f1_obsv] = l_e1;
    }

    // insert edge between current frame and landmark
    {
    g2o::EdgeStereoSE3ProjectXYZ* l_e2 = new g2o::EdgeStereoSE3ProjectXYZ; 
    l_e2->vertices()[0] = v_l;
    l_e2->vertices()[1] = v_f2;
    Eigen::Matrix<double,3,1> f2_obs_mat;
    f2_obs_mat << f2_obsv->lf_.x, f2_obsv->lf_.y, f2_obsv->rf_.x;
    l_e2->setMeasurement(f2_obs_mat);
    Eigen::Matrix3d info;
    info.setIdentity();
    l_e2->setInformation(info);
    l_e2->fx = l_cam_rect_.at<double>(0,0);
    l_e2->fy = l_cam_rect_.at<double>(1,1);
    l_e2->cx = l_cam_rect_.at<double>(0,2);
    l_e2->cy = l_cam_rect_.at<double>(1,2);
    l_e2->bf = l_cam_rect_.at<double>(0,0)*left_cam_T_right_cam_rect_(0,3);
    optimizer.addEdge(l_e2);
    obs_edge_map[f2_obsv] = l_e2;
    }
  }
  // optimize
  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  //-------------------------------------------------------------


  // prepare the initial pose vector
  Eigen::Isometry3d lcam_f1_T_lcam_f2_quat = v_f2->estimate().inverse();
  Eigen::Affine3d f1_lcam_T_f2_lcam = lcam_f1_T_lcam_f2_quat; 
  // correct odometry data 
  f2->car_prev_T_car_cur_ = car_T_left_cam_ * f1_lcam_T_f2_lcam * car_T_left_cam_.inverse();

  f1_lcam_p_f2_lcam[0] = f1_lcam_T_f2_lcam.translation()(0); f1_lcam_p_f2_lcam[0].inflate(1.0);
  f1_lcam_p_f2_lcam[1] = f1_lcam_T_f2_lcam.translation()(1); 
  f1_lcam_p_f2_lcam[2] = f1_lcam_T_f2_lcam.translation()(2); f1_lcam_p_f2_lcam[2].inflate(2.0);
  f1_lcam_p_f2_lcam[3] = atan2(f1_lcam_T_f2_lcam.linear()(2,1),f1_lcam_T_f2_lcam.linear()(2,2)); 
  f1_lcam_p_f2_lcam[4] = ibex::Interval(-M_PI,M_PI);
  f1_lcam_p_f2_lcam[5] = atan2(f1_lcam_T_f2_lcam.linear()(1,0),f1_lcam_T_f2_lcam.linear()(0,0));  
  // Perform the contraction in a SIVIA fasion
  // First create a stack and push a largely uncertainpose vector to the stack
  std::stack<ibex::IntervalVector> s; 
  s.push(f1_lcam_p_f2_lcam);
  ibex::IntervalVector result(6); result.set_empty();
  while(!s.empty())
  {
    // take first element in the stack and perfom contraction for all measurements
    ibex::IntervalVector f1_lcam_p_f2_lcam_local = s.top();
    s.pop();
    // apply the fwd-bwd-contractor sequentially to the measurements
    for(std::pair<Graph::Observation*,Graph::Observation*>& obsv_pair : obsv_pairs)
    {
      ibex::IntervalVector p_l1(2), p_r1(2), p_l2(2), p_r2(2);
      // get the pixel-error of the landmark in f1
      Eigen::Vector3d& f1_pix_err = obs_edge_map[obsv_pair.first]->error(); // error = measurement - estimate
      Eigen::Vector2d f1_lcam_mid((double)obsv_pair.first->lf_.x - f1_pix_err(0)/2.0,(double)obsv_pair.first->lf_.y - f1_pix_err(1)/2.0);
      Eigen::Vector2d f1_rcam_mid((double)obsv_pair.first->rf_.x - f1_pix_err(2)/2.0,(double)obsv_pair.first->lf_.y - f1_pix_err(1)/2.0);
      p_l1[0] = f1_lcam_mid(0); p_l1[0].inflate(abs(f1_pix_err(0))/2.0+0.00001);
      p_l1[1] = f1_lcam_mid(1); p_l1[1].inflate(abs(f1_pix_err(1))/2.0+0.00001);
      p_r1[0] = f1_rcam_mid(0); p_r1[0].inflate(abs(f1_pix_err(2))/2.0+0.00001);
      p_r1[1] = f1_rcam_mid(1); p_r1[1].inflate(abs(f1_pix_err(1))/2.0+0.00001); 

      // get the pixel-error of the landmark in f2
      Eigen::Vector3d& f2_pix_err = obs_edge_map[obsv_pair.second]->error();
      Eigen::Vector2d f2_lcam_mid((double)obsv_pair.second->lf_.x - f2_pix_err(0)/2.0,(double)obsv_pair.second->lf_.y - f2_pix_err(1)/2.0);
      Eigen::Vector2d f2_rcam_mid((double)obsv_pair.second->rf_.x - f2_pix_err(2)/2.0,(double)obsv_pair.second->lf_.y - f2_pix_err(1)/2.0);
      p_l2[0] = f2_lcam_mid(0); p_l2[0].inflate(abs(f2_pix_err(0))/2.0+0.00001);
      p_l2[1] = f2_lcam_mid(1); p_l2[1].inflate(abs(f2_pix_err(1))/2.0+0.00001);
      p_r2[0] = f2_rcam_mid(0); p_r2[0].inflate(abs(f2_pix_err(2))/2.0+0.00001);
      p_r2[1] = f2_rcam_mid(1); p_r2[1].inflate(abs(f2_pix_err(1))/2.0+0.00001);

      ibex::IntervalVector P_l1(3);
      P_l1[2] = ibex::Interval::POS_REALS;
      ctc_vo.contract(p_l2,p_r2,p_l1,p_r1,P_l1,f1_lcam_p_f2_lcam_local);
      if(!P_l1.is_empty())
      {
        obsv_pair.first->lcam_box_ = P_l1;
        obsv_pair.first->reprojection_error_ = f1_pix_err;
        obsv_pair.second->reprojection_error_ = f2_pix_err;
      }
    }
    if(!f1_lcam_p_f2_lcam_local.is_empty())
    {
      if(f1_lcam_p_f2_lcam_local[4].diam() > eps_phi_bisection_)
      {
        std::pair<ibex::IntervalVector,ibex::IntervalVector> bisected_boxes = f1_lcam_p_f2_lcam_local.bisect(4);
        s.push(bisected_boxes.first);
        s.push(bisected_boxes.second); 
      }
      else if(f1_lcam_p_f2_lcam_local[0].diam() > eps_x_bisection_)
      {
        std::pair<ibex::IntervalVector,ibex::IntervalVector> bisected_boxes = f1_lcam_p_f2_lcam_local.bisect(0);
        s.push(bisected_boxes.first);
        s.push(bisected_boxes.second); 
      }
      else
      {
        result = result | f1_lcam_p_f2_lcam_local; 
      }
    }
  } 
  f1_lcam_p_f2_lcam = f1_lcam_p_f2_lcam & result; 
}

/********Output file for evaluation**********/
void VisualOdometry::save_to_file()
{
  std::cout << "start writing VO results" << std::endl; 
  // set the number precision -> for timestamp!
  results_ofstream_ << std::setprecision(16);

  // associate an id to each seen landmark
  std::map<Graph::Landmark*,int> landmark_id_map;
  int id_counter = 0;  
  for(Graph::Frame* f : graph_frames_)
  {
    for(Graph::Landmark* l : f->seen_landmarks_)
    {
      if(landmark_id_map.find(l) == landmark_id_map.end())
      {
        landmark_id_map[l] = id_counter; 
        id_counter++;
      }
    }
  }

  // iterate through each frame 
  for(Graph::Frame* f : graph_frames_)
  {
    results_ofstream_ << f->timestamp_ << " "; // timestamp
    std::string frame_type  = (f->is_keyframe_) ? "k" : "f";
    results_ofstream_ << frame_type << " "; // keyframe of frame
    results_ofstream_ << f->lcam_prev_p_lcam_cur[0].lb() << " "; // interval odom 
    results_ofstream_ << f->lcam_prev_p_lcam_cur[0].ub() << " ";
    results_ofstream_ << f->lcam_prev_p_lcam_cur[1].lb() << " ";
    results_ofstream_ << f->lcam_prev_p_lcam_cur[1].ub() << " ";
    results_ofstream_ << f->lcam_prev_p_lcam_cur[2].lb() << " ";
    results_ofstream_ << f->lcam_prev_p_lcam_cur[2].ub() << " ";
    results_ofstream_ << f->lcam_prev_p_lcam_cur[3].lb() << " ";
    results_ofstream_ << f->lcam_prev_p_lcam_cur[3].ub() << " ";
    results_ofstream_ << f->lcam_prev_p_lcam_cur[4].lb() << " ";
    results_ofstream_ << f->lcam_prev_p_lcam_cur[4].ub() << " ";
    results_ofstream_ << f->lcam_prev_p_lcam_cur[5].lb() << " ";
    results_ofstream_ << f->lcam_prev_p_lcam_cur[5].ub() << " ";
    std::string odom_computed  = (f->computed_interval_odometry_) ? "True" : "False";
    results_ofstream_ << odom_computed << " ";
    results_ofstream_ << f->preprocessing_duration_ << " "; // preprocessing times
    results_ofstream_ << f->detect_track_duration_ << " "; // detection extraction duration
    results_ofstream_ << f->stereo_odom_opt_duration_ << " "; // frame-to-frame optimization duration 
    results_ofstream_ << f->seen_landmarks_.size() << " "; // number of seen landmarks
    // save the landmark observations
    for(Graph::Landmark* l : f->seen_landmarks_)
    {
      results_ofstream_ << landmark_id_map[l] << " "; // id 
      results_ofstream_ << l->obsv_map_[f]->lcam_box_[0].lb() << " "; // lb of x
      results_ofstream_ << l->obsv_map_[f]->lcam_box_[0].ub() << " "; // ub of x
      results_ofstream_ << l->obsv_map_[f]->lcam_box_[1].lb() << " "; // lb of y
      results_ofstream_ << l->obsv_map_[f]->lcam_box_[1].ub() << " "; // ub of y
      results_ofstream_ << l->obsv_map_[f]->lcam_box_[2].lb() << " "; // lb of z
      results_ofstream_ << l->obsv_map_[f]->lcam_box_[2].ub() << " "; // ub of z
      results_ofstream_ << l->obsv_map_[f]->reprojection_error_(0) << " "; // reprojection error left x
      results_ofstream_ << l->obsv_map_[f]->reprojection_error_(1) << " "; // reprojection error left y
      results_ofstream_ << l->obsv_map_[f]->reprojection_error_(2) << " "; // reprojection error right x
    }
    results_ofstream_ << std::endl; 
  }
  results_ofstream_ << "---" << std::endl; // mark the end of frame infos, start landmark infos
  for(std::pair<Graph::Landmark*,int> landmark_id_pair : landmark_id_map)
  {
    std::string landmark_type = (landmark_id_pair.first->good_) ? "good" : "bad";
    results_ofstream_ << landmark_id_pair.second << " " <<  landmark_type << " ";
    for(std::pair<Graph::Frame*,Graph::Observation*> frame_obs_pair : landmark_id_pair.first->obsv_map_)   
    {
      results_ofstream_ << frame_obs_pair.first->timestamp_ << " "; 
    } 
    results_ofstream_ << std::endl; 
  }
  std::cout << "finished writing VO results" << std::endl;
}
