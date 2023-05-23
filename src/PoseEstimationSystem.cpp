#include "hypascore_localization/PoseEstimationSystem.hpp"
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
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/MarkerArray.h>

PoseEstimationSystem::PoseEstimationSystem(ros::NodeHandle& nh)
{
  double utm_world_offset_x;
  double utm_world_offset_y;
  double utm_world_offset_z;
  std::string city_model_file, overall_results_file, overall_only_rt_results_file;

  nh.getParam("/hypascore_localization_node/utm_world_offset_x", utm_world_offset_x);
  nh.getParam("/hypascore_localization_node/utm_world_offset_y", utm_world_offset_y);
  nh.getParam("/hypascore_localization_node/utm_world_offset_z", utm_world_offset_z);
  nh.getParam("/hypascore_localization_node/city_model_file", city_model_file);
  nh.getParam("/hypascore_localization_node/x_odom_translation_uncertainty_radius", x_odom_uncertainty_predicted_);
  nh.getParam("/hypascore_localization_node/y_odom_translation_uncertainty_radius", y_odom_uncertainty_predicted_);
  nh.getParam("/hypascore_localization_node/odom_rotation_uncertainty_radius", rot_odom_uncertainty_predicted_);
  Eigen::Vector3d utm_world_offset(utm_world_offset_x,utm_world_offset_y,utm_world_offset_z);
  utm_world_offset_ = utm_world_offset;
  nh.getParam("/hypascore_localization_node/min_particle_age_to_overwrite_gl", min_particle_age_to_overwrite_gl_);
  nh.getParam("/hypascore_localization_node/file_overall_results", overall_results_file);
  nh.getParam("/hypascore_localization_node/file_overall_only_rt_results", overall_only_rt_results_file);

  // load LOD2-City Model
  map_.reset(new CMMap(city_model_file, utm_world_offset_));

  // create the Visual Odometry object
  vo_.reset(new VisualOdometry(nh));

  // create Lidar-data object
  lidar_data_.reset(new LidarData(nh));

  // create GPS-data object
  gps_data_.reset(new GpsData(nh));

  // create the global localization object
  global_localizer_.reset(new CMGlobalLocalizer(nh,map_.get()));

  // prepare results file (if necessary)
  if(overall_results_file != "") // if overall_results_file string is not empty, dump results to file
  {
    dump_results_ = true;
    std::cout << "Start saving to: " << overall_results_file << std::endl; 
    results_ofstream_.open(overall_results_file.c_str());
    map_T_laser_cur_feasible_hull_ = ibex::IntervalVector(3);
    map_T_laser_loc_feasible_hull_ = ibex::IntervalVector(3);
  }
  // prpare only real-time results file
  if(overall_only_rt_results_file != "")
  {
    dump_only_rt_results_ = true;
    std::cout << "Start saving to: " << overall_only_rt_results_file << std::endl; 
    only_rt_results_ofstream_.open(overall_only_rt_results_file.c_str());
    map_T_laser_cur_feasible_hull_ = ibex::IntervalVector(3);
  }

  /*Visualization initialization*/
  // create the building map image
  create_full_building_image();
  full_building_image_.copyTo(output_im_);
  // create the windows in which the result should be shown
  std::string win_name1 = "Global Localization Result";
  cv_windows.push_back(win_name1);
  cv::namedWindow(cv_windows[0]);
  cv::moveWindow(cv_windows[0],3840,0);
  // create RVIZ publishers
  cur_pose_publisher_.reset(new ros::Publisher);
  *cur_pose_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("cur_pose", 1);
  tracked_pose_publisher_.reset(new ros::Publisher);
  *tracked_pose_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("tracked_pose", 1);
  gl_pose_hull_ = ibex::IntervalVector(3);
  cur_polygon_publisher_.reset(new ros::Publisher);
  *cur_polygon_publisher_ = nh.advertise<visualization_msgs::Marker>("cur_position_polygone", 1);
  cur_global_polygon_publisher_.reset(new ros::Publisher);
  *cur_global_polygon_publisher_ = nh.advertise<visualization_msgs::Marker>("cur_global_position_polygon", 1);

  // create the tracking object
  tracker_.reset(new CMTracker(nh,map_.get()));

  // start the localization thread
  localization_thread_ = std::thread(&PoseEstimationSystem::localize, this); 

}

PoseEstimationSystem::~PoseEstimationSystem()
{
  localization_thread_.join(); 
}

void PoseEstimationSystem::localize_and_track(const sensor_msgs::ImageConstPtr& l_im_ptr, const sensor_msgs::ImageConstPtr& r_im_ptr, const sensor_msgs::PointCloud2ConstPtr& pc_ptr)
{
  // only lock the mutex if output file is written 
  if(dump_results_) file_write_mutex_.lock();
  
  // get current time
  cur_time_ = l_im_ptr->header.stamp.toSec();

  // transfrom point cloud in another thread
  lidar_data_mutex_.lock(); 
  std::thread lidar_data_thread(&LidarData::save_lidar_data, lidar_data_.get(), pc_ptr, cur_time_);

  // call the vo_ to build the slam-graph -> provides the information on the movement of the vehicle 
  auto start_vo = std::chrono::high_resolution_clock::now();
  vo_->track(l_im_ptr,r_im_ptr);
  auto end_vo = std::chrono::high_resolution_clock::now();
  auto duration_vo = std::chrono::duration_cast<std::chrono::microseconds>(end_vo - start_vo);
  vo_time_ = duration_vo.count()*std::pow(10,-6);

  // wait until lidar_data_thread is finished
  lidar_data_thread.join(); 
  lidar_data_mutex_.unlock();

  // Visualize current pose
  show_cur_pose_full_map();

  // dump if necessary the real-time results
  if(dump_only_rt_results_) save_only_rt_to_file(); 

  if(dump_results_) file_write_mutex_.unlock(); // unlock the mutex for writing to file; 
}

void PoseEstimationSystem::process_gps(const Eigen::Vector3d& utm_position, double timestamp)
{
  gps_data_mutex_.lock(); 
  gps_data_->save_gps_data(utm_position, timestamp);
  gps_data_mutex_.unlock(); 
}

void PoseEstimationSystem::initialize_localization(ibex::IntervalVector& translation, ibex::Interval& rotation)
{
  global_localizer_->set_initial_region(translation,rotation);
}

void PoseEstimationSystem::localize()
{
  int frame_counter = 0;
  while(ros::ok())
  {
    vo_->mutex_graph_.lock();
    if(vo_->graph_frames_.size() > frame_counter && vo_->graph_frames_[frame_counter]->computed_interval_odometry_)
    {
      // get the frame
      Graph::Frame* f = vo_->graph_frames_[frame_counter]; 
      vo_->mutex_graph_.unlock(); 
      // get the LiDAR scan
      lidar_data_mutex_.lock();
      std::unique_ptr<LidarScan>& lidar_scan = lidar_data_->time_scan_map_[f->timestamp_];
      lidar_data_mutex_.unlock();
      // transform odometry data from car to lidar
      Eigen::Affine3d laser_prev_T_laser_cur = global_localizer_->car_T_laser_.inverse() * f->car_prev_T_car_cur_ * global_localizer_->car_T_laser_;
      // build the affine2d object from the 6dof
      double psi = atan2(laser_prev_T_laser_cur.linear()(1,0),laser_prev_T_laser_cur.linear()(0,0));
      Eigen::Affine2d laser_prev_T_laser_cur_2d; 
      laser_prev_T_laser_cur_2d.matrix()(0,0) = cos(psi); laser_prev_T_laser_cur_2d.matrix()(0,1) = -sin(psi); 
      laser_prev_T_laser_cur_2d.matrix()(1,0) = sin(psi); laser_prev_T_laser_cur_2d.matrix()(1,1) = cos(psi); 
      laser_prev_T_laser_cur_2d.matrix()(0,2) = laser_prev_T_laser_cur.translation()(0);
      laser_prev_T_laser_cur_2d.matrix()(1,2) = laser_prev_T_laser_cur.translation()(1);

      /**********************
       * Use Global Localizer
       **********************/
      auto start_coarse = std::chrono::high_resolution_clock::now();
      gps_data_mutex_.lock(); 
      if(gps_data_->time_gps_map_.find(f->timestamp_) == gps_data_->time_gps_map_.end())
      {
        gps_data_mutex_.unlock(); 
        // no gps-data for this timestamp
        std::unique_ptr<GpsMeasure> gps_measure; 
        gps_measure.reset(NULL); 
        global_localizer_->shift_localization_region(laser_prev_T_laser_cur_2d, lidar_scan, gps_measure);
      }
      else
      {
        // gps-data for thos timestamp exists
        std::unique_ptr<GpsMeasure>& gps_measure = gps_data_->time_gps_map_[f->timestamp_];
        gps_data_mutex_.unlock(); 
        global_localizer_->shift_localization_region(laser_prev_T_laser_cur_2d, lidar_scan, gps_measure);
      }
      // save the hull of the localization result to the frame object
      global_localizer_->get_localization_result(f->gl__map_p_laser);
      auto end_coarse = std::chrono::high_resolution_clock::now();
      auto duration_coarse = std::chrono::duration_cast<std::chrono::microseconds>(end_coarse - start_coarse);
      coarse_loc_time_ = duration_coarse.count()*std::pow(10,-6);

      /**********************
       * Use Tracker
       **********************/
      auto start_refined = std::chrono::high_resolution_clock::now();
      // Check if the global localization result is good enough for tracking -> if yes: track!
      tracker_->track(lidar_scan,global_localizer_->feasible_poses_, laser_prev_T_laser_cur_2d, f->laser_prev_p_laser_cur, f->timestamp_);
      // visualize result
      show_full_map();
      frame_counter++; 
      auto end_refined = std::chrono::high_resolution_clock::now();
      auto duration_refined = std::chrono::duration_cast<std::chrono::microseconds>(end_refined - start_refined);
      refined_loc_time_ = duration_refined.count()*std::pow(10,-6);
      auto duration_full_loc = std::chrono::duration_cast<std::chrono::microseconds>(end_refined - start_coarse);
      full_loc_time_ = duration_full_loc.count()*std::pow(10,-6);

      // Save the results (necessary)
      if(dump_results_)
      {
        save_to_file(f);
      }
    }
    else
    {
      vo_->mutex_graph_.unlock(); // unlock the graph-mutex
      // if the last batch was not computed, wait a 0.1s
      ros::Duration(0.1).sleep();
    }
  }
}

/********Visualization***********/
void PoseEstimationSystem::create_full_building_image()
{
  std::vector<cv::Point2f>& cv_hull = map_->cv_points_convex_hull_map_;
  cv::Rect2f rect = cv::boundingRect(cv_hull);
  int large_im_width = 1920;
  int large_im_height = 1000;
  double pixel_per_m = std::min((double)large_im_width/(double)rect.height,(double)large_im_height/(double)rect.width);
  x_size_full_ = rect.height;
  y_size_full_ = rect.width;
  std::cout << "Map Width: " << x_size_full_ << "m; " << "Map Height: " << y_size_full_ << "m" << std::endl; 
  pixel_size_full_ = 1/pixel_per_m; 
  cv::Scalar background_color(255,255,255);

  cv::Mat large_im(cv::Size(std::ceil(x_size_full_/pixel_size_full_),std::ceil(y_size_full_/pixel_size_full_)),CV_8UC3, background_color);

  std::vector<size_t> building_idxs;
  for(int i = 0; i<map_->buildings_.size(); i++)
  {
    building_idxs.push_back(i);
  }
 
  im_origin_full_(0) = rect.tl().y;
  im_origin_full_ (1) = rect.tl().x;

  // Draw Buildings
  for(size_t& i : building_idxs)
  {
    Building& b = map_->buildings_[i];
    std::vector<cv::Point2i> b_points; 
    for(Surface& s : b.grounds)
    {
      for(Eigen::Vector3d& p : s.points)
      {
        cv::Point2i poly_p; 
        poly_p.x = (int)std::round((p(1)-im_origin_full_(0))/pixel_size_full_);
        poly_p.y = (int)std::round((p(0)-im_origin_full_(1))/pixel_size_full_);
        b_points.push_back(poly_p);
      }
    }
    if(b_points.size() > 2)
    {
      cv::fillPoly( large_im, b_points, cv::Scalar(0,0,0));
    }
  }

  // Draw convex hull
  for(int i = 1; i < map_->cv_points_convex_hull_map_.size(); i++)
  {
    cv::Point2f& world_p1 = map_->cv_points_convex_hull_map_[i];
    cv::Point2f& world_p2 = map_->cv_points_convex_hull_map_[i-1];

    cv::Point2i line_p1; 
    line_p1.x = (int)std::round((world_p1.y-im_origin_full_(0))/pixel_size_full_);
    line_p1.y = (int)std::round((world_p1.x-im_origin_full_(1))/pixel_size_full_);

    cv::Point2i line_p2; 
    line_p2.x = (int)std::round((world_p2.y-im_origin_full_(0))/pixel_size_full_);
    line_p2.y = (int)std::round((world_p2.x-im_origin_full_(1))/pixel_size_full_);

    cv::line(large_im, line_p1, line_p2, cv::Scalar(0, 165, 255), 5);
  }
  cv::Point2f& world_p1 = map_->cv_points_convex_hull_map_[0];
  cv::Point2f& world_p2 = map_->cv_points_convex_hull_map_[map_->cv_points_convex_hull_map_.size()-1];
  cv::Point2i line_p1; 
  line_p1.x = (int)std::round((world_p1.y-im_origin_full_(0))/pixel_size_full_);
  line_p1.y = (int)std::round((world_p1.x-im_origin_full_(1))/pixel_size_full_);
  cv::Point2i line_p2; 
  line_p2.x = (int)std::round((world_p2.y-im_origin_full_(0))/pixel_size_full_);
  line_p2.y = (int)std::round((world_p2.x-im_origin_full_(1))/pixel_size_full_);
  cv::line(large_im, line_p1, line_p2, cv::Scalar(0, 165, 255), 5);

  large_im.copyTo(full_building_image_);
}

void PoseEstimationSystem::multiply_interval_poses(ibex::IntervalVector& a_T_b, ibex::IntervalVector& b_T_c, ibex::IntervalVector& a_T_c)
{
  // SE2 function: a_T_b*b_T_c = a_T_c; 
  // compute laser_prev_T_laser_cur_2d * laser_localize_T_laser_cur 
  ibex::Interval tx1 = a_T_b[0];
  ibex::Interval ty1 = a_T_b[1];
  ibex::Interval alpha = a_T_b[2];
  ibex::Interval tx2 = b_T_c[0];
  ibex::Interval ty2 = b_T_c[1];
  ibex::Interval beta = b_T_c[2];

  a_T_c[2] = alpha+beta;
  a_T_c[0] = cos(alpha)*tx2-sin(alpha)*ty2+tx1;
  a_T_c[1] = sin(alpha)*tx2+cos(alpha)*ty2+ty1;
}

void PoseEstimationSystem::show_cur_pose_full_map()
{
  output_im_mutex_.lock();
  PoseParticle particle = tracker_particle_copy_;
  double localize_time_stamp = localize_time_stamp_; 
  bool tracker_initialized = tracker_initialized_;
  double tracker_reliability = tracker_reliability_; 
  std::vector<Eigen::Vector2d> tracked_polygone = tracked_polygone_;
  ibex::Interval tracked_rotation = tracked_rotation_;
  ibex::IntervalVector laser_map_T_laser_localize_gl_ibx(3);  
  laser_map_T_laser_localize_gl_ibx = gl_pose_hull_;
  output_im_mutex_.unlock();
  consistent_rot_cur_.set_empty();
  consistent_polygon_cur_.clear();
  map_T_laser_cur_feasible_hull_.set_empty();

  // draw current pose based on the tracked particle
  Eigen::Affine2d laser_localize_T_laser_cur = Eigen::Affine2d::Identity();
  ibex::IntervalVector laser_localize_T_laser_cur_ibx(3);
  laser_localize_T_laser_cur_ibx = ibex::Vector::zeros(3);
  for(int i = vo_->graph_frames_.size()-1; i >= 0 && vo_->graph_frames_[i]->timestamp_>=localize_time_stamp; i--)
  {
    Graph::Frame* f = (vo_->graph_frames_[i]);
    // transform odometry data from car to lidar
    Eigen::Affine3d laser_prev_T_laser_cur = global_localizer_->car_T_laser_.inverse() * f->car_prev_T_car_cur_ * global_localizer_->car_T_laser_;
    // build the affine2d object from the 6dof
    double psi = atan2(laser_prev_T_laser_cur.linear()(1,0),laser_prev_T_laser_cur.linear()(0,0));
    Eigen::Affine2d laser_prev_T_laser_cur_2d; 
    laser_prev_T_laser_cur_2d.matrix()(0,0) = cos(psi); laser_prev_T_laser_cur_2d.matrix()(0,1) = -sin(psi); 
    laser_prev_T_laser_cur_2d.matrix()(1,0) = sin(psi); laser_prev_T_laser_cur_2d.matrix()(1,1) = cos(psi); 
    laser_prev_T_laser_cur_2d.matrix()(0,2) = laser_prev_T_laser_cur.translation()(0);
    laser_prev_T_laser_cur_2d.matrix()(1,2) = laser_prev_T_laser_cur.translation()(1);

    laser_localize_T_laser_cur = laser_prev_T_laser_cur_2d * laser_localize_T_laser_cur;

    // compute odometry with intervals
    ibex::IntervalVector laser_prev_T_laser_cur_ibx(3); 
    laser_prev_T_laser_cur_ibx[0] = laser_prev_T_laser_cur.translation()(0);
    laser_prev_T_laser_cur_ibx[1] = laser_prev_T_laser_cur.translation()(1);
    laser_prev_T_laser_cur_ibx[2] = psi;
    laser_prev_T_laser_cur_ibx[0].inflate(x_odom_uncertainty_predicted_);
    laser_prev_T_laser_cur_ibx[1].inflate(y_odom_uncertainty_predicted_);
    laser_prev_T_laser_cur_ibx[2].inflate(rot_odom_uncertainty_predicted_);
    multiply_interval_poses(laser_prev_T_laser_cur_ibx,laser_localize_T_laser_cur_ibx,laser_localize_T_laser_cur_ibx);
  }
  Eigen::Affine2d map_localize_T_laser_cur = particle.pose * laser_localize_T_laser_cur;


  // compute the current polygon from the previous applying laser_localize_T_laser_cur_ibx
  std::vector<cv::Point2f> corner_points; 
  for(Eigen::Vector2d& p_localize : tracked_polygone)
  {
    // 1. each corner is  an "edge position"
    ibex::IntervalVector laser_map_T_laser_localize(3);
    laser_map_T_laser_localize[0] = p_localize.x();
    laser_map_T_laser_localize[1] = p_localize.y();
    laser_map_T_laser_localize[2] = tracked_rotation; 

    ibex::IntervalVector laser_map_T_laser_cur_ibx(3);
    multiply_interval_poses(laser_map_T_laser_localize,laser_localize_T_laser_cur_ibx,laser_map_T_laser_cur_ibx);
    consistent_rot_cur_ = consistent_rot_cur_ | laser_map_T_laser_cur_ibx[2];

    cv::Point2f p1;
    p1.x = (float)laser_map_T_laser_cur_ibx[0].lb();
    p1.y = (float)laser_map_T_laser_cur_ibx[1].lb();
    cv::Point2f p2;
    p2.x = (float)laser_map_T_laser_cur_ibx[0].lb();
    p2.y = (float)laser_map_T_laser_cur_ibx[1].ub();
    cv::Point2f p3;
    p3.x = (float)laser_map_T_laser_cur_ibx[0].ub();
    p3.y = (float)laser_map_T_laser_cur_ibx[1].ub();
    cv::Point2f p4;
    p4.x = (float)laser_map_T_laser_cur_ibx[0].ub();
    p4.y = (float)laser_map_T_laser_cur_ibx[1].lb();
    corner_points.push_back(p1);
    corner_points.push_back(p2);
    corner_points.push_back(p3);
    corner_points.push_back(p4);
  }
  if(corner_points.size() > 0)
  {
    std::vector<cv::Point2f> hull_points;
    cv::convexHull(corner_points,hull_points);
    consistent_polygon_cur_ = hull_points;
    // draw position polygon
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time();
      marker.ns = "cur_position_polygone";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.scale.x = 0.23;
      marker.color.a = 1; // Don't forget to set the alpha!
      marker.color.r = 0.6*(1.0-tracker_reliability);
      marker.color.g = 0.6*tracker_reliability;
      marker.color.b = 0;
      for(cv::Point2f& point : hull_points)
      {
        geometry_msgs::Point p;
        p.x = (double)point.x;
        p.y = (double)point.y;
        p.z = 0;
        marker.points.push_back(p);
      }   
      geometry_msgs::Point p;
      p.x = (double)hull_points[0].x;
      p.y = (double)hull_points[0].y;
      p.z = 0;
      marker.points.push_back(p);
      cur_polygon_publisher_->publish(marker);
    }
  }
  else
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "cur_position_polygone";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::DELETE;
    cur_polygon_publisher_->publish(marker);
  }
  
  // compute global position hull
  ibex::IntervalVector laser_map_T_laser_cur_gl_ibx(3);  
  multiply_interval_poses(laser_map_T_laser_localize_gl_ibx,laser_localize_T_laser_cur_ibx,laser_map_T_laser_cur_gl_ibx);
  map_T_laser_cur_feasible_hull_ = laser_map_T_laser_cur_gl_ibx;
  // draw the marker
  if(!laser_map_T_laser_cur_gl_ibx.is_unbounded())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "cur_global_position_hull_polygon";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.23;
    marker.color.a = 0.5; // Don't forget to set the alpha!
    marker.color.r = 0.6;
    marker.color.g = 0.6;
    marker.color.b = 0.0;
    ibex::Interval& x = laser_map_T_laser_cur_gl_ibx[0];
    ibex::Interval& y = laser_map_T_laser_cur_gl_ibx[1];
    std::vector<Eigen::Vector2d> box_corners(5); 
    box_corners[0] << x.lb(), y.lb();
    box_corners[1] << x.lb(), y.ub();
    box_corners[2] << x.ub(), y.ub();
    box_corners[3] << x.ub(), y.lb();
    box_corners[4] = box_corners[0];
    for(Eigen::Vector2d& point : box_corners)
    {
      geometry_msgs::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = 0;
      marker.points.push_back(p);
    }    
    cur_global_polygon_publisher_->publish(marker);
  }

  if(tracker_initialized)
  {
    cv::Mat im;
    output_im_mutex_.lock();
    output_im_.copyTo(im);
    map_localize_T_laser_cur_ = map_localize_T_laser_cur; 
    output_im_mutex_.unlock();

    cv::Point2i pos_pixel; 
    pos_pixel.x = (int)std::round((map_localize_T_laser_cur.translation()(1)-im_origin_full_(0))/pixel_size_full_);
    pos_pixel.y = (int)std::round((map_localize_T_laser_cur.translation()(0)-im_origin_full_(1))/pixel_size_full_);
    cv::Point2i x_end_pos_pixel; 
    Eigen::Vector2d x_direction =  map_localize_T_laser_cur*Eigen::Vector2d(10,0);
    x_end_pos_pixel.x = (int)std::round((x_direction(1)-im_origin_full_(0))/pixel_size_full_);
    x_end_pos_pixel.y = (int)std::round((x_direction(0)-im_origin_full_(1))/pixel_size_full_);
    cv::Point2i y_end_pos_pixel; 
    Eigen::Vector2d y_direction =  map_localize_T_laser_cur*Eigen::Vector2d(0,10);
    y_end_pos_pixel.x = (int)std::round((y_direction(1)-im_origin_full_(0))/pixel_size_full_);
    y_end_pos_pixel.y = (int)std::round((y_direction(0)-im_origin_full_(1))/pixel_size_full_);
    cv::line(im,pos_pixel,x_end_pos_pixel,cv::Scalar(0,0,200),3);
    cv::line(im,pos_pixel,y_end_pos_pixel,cv::Scalar(0,0,200),3);

    // publish the current pose to RVIZ
    geometry_msgs::PoseStamped cur_pose; 
    cur_pose.header.stamp = ros::Time::now();
    cur_pose.header.frame_id = "world"; 
    cur_pose.pose.position.x = map_localize_T_laser_cur.translation().x();
    cur_pose.pose.position.y = map_localize_T_laser_cur.translation().y();
    cur_pose.pose.position.z = 0.0;
    Eigen::Matrix3d cur_rot;
    cur_rot <<  map_localize_T_laser_cur.matrix()(0,0), map_localize_T_laser_cur.matrix()(0,1), 0,
                map_localize_T_laser_cur.matrix()(1,0), map_localize_T_laser_cur.matrix()(1,1), 0,
                                                    0,                                      0,  1;
    Eigen::Quaterniond q(cur_rot);
    cur_pose.pose.orientation.x = q.x();
    cur_pose.pose.orientation.y = q.y();
    cur_pose.pose.orientation.z = q.z();
    cur_pose.pose.orientation.w = q.w();
    cur_pose_publisher_->publish(cur_pose);

    window_output_im_mutex_.lock();
    cv::imshow(cv_windows[0], im);
    cv::waitKey(1);
    window_output_im_mutex_.unlock();
  }
}

void PoseEstimationSystem::show_full_map()
{
  cv::Mat im; 
  full_building_image_.copyTo(im);
  // draw the subpaving
  for(int ipose_counter = 0; ipose_counter < global_localizer_->feasible_poses_.size(); ipose_counter++)
  {
    IntervalPose& ipose = global_localizer_->feasible_poses_[ipose_counter];
    for(int t_counter = 0; t_counter < ipose.translation.size(); t_counter++)
    {
      ibex::IntervalVector& t = ipose.translation[t_counter];
      Eigen::Vector2d p1(t[0].lb(),t[1].lb());
      Eigen::Vector2d p2(t[0].ub(),t[1].ub());

      cv::Point2i p1_pixel; 
      p1_pixel.x = (int)std::round((p1(1)-im_origin_full_(0))/pixel_size_full_);
      p1_pixel.y = (int)std::round((p1(0)-im_origin_full_(1))/pixel_size_full_);

      cv::Point2i p2_pixel; 
      p2_pixel.x = (int)std::round((p2(1)-im_origin_full_(0))/pixel_size_full_);
      p2_pixel.y = (int)std::round((p2(0)-im_origin_full_(1))/pixel_size_full_);

      cv::rectangle(im, p1_pixel, p2_pixel, cv::Scalar(0,170,0), -1);
      //cv::rectangle(im, p1_pixel, p2_pixel, cv::Scalar(0,170,200), 2);
    }
  }

  // draw the hull over all subpavings
  ibex::IntervalVector map_p_laser(3); 
  global_localizer_->get_localization_result(map_p_laser);
  {
    ibex::IntervalVector t = map_p_laser.subvector(0,1);
    Eigen::Vector2d p1(t[0].lb(),t[1].lb());
    Eigen::Vector2d p2(t[0].ub(),t[1].ub());

    cv::Point2i p1_pixel; 
    p1_pixel.x = (int)std::round((p1(1)-im_origin_full_(0))/pixel_size_full_);
    p1_pixel.y = (int)std::round((p1(0)-im_origin_full_(1))/pixel_size_full_);

    cv::Point2i p2_pixel; 
    p2_pixel.x = (int)std::round((p2(1)-im_origin_full_(0))/pixel_size_full_);
    p2_pixel.y = (int)std::round((p2(0)-im_origin_full_(1))/pixel_size_full_);

    cv::rectangle(im, p1_pixel, p2_pixel, cv::Scalar(0,170,200), 2);
  }

  // draw all particle
  for(int i = 0; i < global_localizer_->all_particles_.size(); i++)
  {
    PoseParticle& particle = global_localizer_->all_particles_[i];
    cv::Point2i pos_pixel; 
    pos_pixel.x = (int)std::round((particle.pose.translation()(1)-im_origin_full_(0))/pixel_size_full_);
    pos_pixel.y = (int)std::round((particle.pose.translation()(0)-im_origin_full_(1))/pixel_size_full_);
    cv::Point2i x_end_pos_pixel; 
    Eigen::Vector2d x_direction =  particle.pose*Eigen::Vector2d(10,0);
    x_end_pos_pixel.x = (int)std::round((x_direction(1)-im_origin_full_(0))/pixel_size_full_);
    x_end_pos_pixel.y = (int)std::round((x_direction(0)-im_origin_full_(1))/pixel_size_full_);
    cv::Point2i y_end_pos_pixel; 
    Eigen::Vector2d y_direction =  particle.pose*Eigen::Vector2d(0,10);
    y_end_pos_pixel.x = (int)std::round((y_direction(1)-im_origin_full_(0))/pixel_size_full_);
    y_end_pos_pixel.y = (int)std::round((y_direction(0)-im_origin_full_(1))/pixel_size_full_);
    cv::line(im,pos_pixel,x_end_pos_pixel,cv::Scalar(255,0,0),3);
    cv::line(im,pos_pixel,y_end_pos_pixel,cv::Scalar(255,0,0),3);
  }

  // draw the tracked particle
  if(tracker_->tracking_initialized_)
  {
    PoseParticle& particle = tracker_->particle_;
    cv::Point2i pos_pixel; 
    pos_pixel.x = (int)std::round((particle.pose.translation()(1)-im_origin_full_(0))/pixel_size_full_);
    pos_pixel.y = (int)std::round((particle.pose.translation()(0)-im_origin_full_(1))/pixel_size_full_);
    cv::Point2i x_end_pos_pixel; 
    Eigen::Vector2d x_direction =  particle.pose*Eigen::Vector2d(10,0);
    x_end_pos_pixel.x = (int)std::round((x_direction(1)-im_origin_full_(0))/pixel_size_full_);
    x_end_pos_pixel.y = (int)std::round((x_direction(0)-im_origin_full_(1))/pixel_size_full_);
    cv::Point2i y_end_pos_pixel; 
    Eigen::Vector2d y_direction =  particle.pose*Eigen::Vector2d(0,10);
    y_end_pos_pixel.x = (int)std::round((y_direction(1)-im_origin_full_(0))/pixel_size_full_);
    y_end_pos_pixel.y = (int)std::round((y_direction(0)-im_origin_full_(1))/pixel_size_full_);
    cv::line(im,pos_pixel,x_end_pos_pixel,cv::Scalar(0,180,180),3);
    cv::line(im,pos_pixel,y_end_pos_pixel,cv::Scalar(0,180,180),3);
  }

  output_im_mutex_.lock();
  im.copyTo(output_im_);
  tracker_particle_copy_ = tracker_->particle_;
  localize_time_stamp_ = tracker_->processed_timestamp_; 
  tracker_initialized_ = tracker_->tracking_initialized_;
  tracker_reliability_ = tracker_->tracking_reliablility_; 
  tracked_polygone_ = tracker_->polygone_;
  tracked_rotation_ = tracker_->rotation_;
  gl_pose_hull_ = ibex::IntervalVector(3);  
  global_localizer_->get_localization_result(gl_pose_hull_);
  output_im_mutex_.unlock();

  // draw current pose based on the tracked particle
  if(tracker_->tracking_initialized_)
  {
    output_im_mutex_.lock();
    Eigen::Affine2d map_localize_T_laser_cur = map_localize_T_laser_cur_;
    output_im_mutex_.unlock();
    cv::Point2i pos_pixel; 
    pos_pixel.x = (int)std::round((map_localize_T_laser_cur.translation()(1)-im_origin_full_(0))/pixel_size_full_);
    pos_pixel.y = (int)std::round((map_localize_T_laser_cur.translation()(0)-im_origin_full_(1))/pixel_size_full_);
    cv::Point2i x_end_pos_pixel; 
    Eigen::Vector2d x_direction =  map_localize_T_laser_cur*Eigen::Vector2d(10,0);
    x_end_pos_pixel.x = (int)std::round((x_direction(1)-im_origin_full_(0))/pixel_size_full_);
    x_end_pos_pixel.y = (int)std::round((x_direction(0)-im_origin_full_(1))/pixel_size_full_);
    cv::Point2i y_end_pos_pixel; 
    Eigen::Vector2d y_direction =  map_localize_T_laser_cur*Eigen::Vector2d(0,10);
    y_end_pos_pixel.x = (int)std::round((y_direction(1)-im_origin_full_(0))/pixel_size_full_);
    y_end_pos_pixel.y = (int)std::round((y_direction(0)-im_origin_full_(1))/pixel_size_full_);
    cv::line(im,pos_pixel,x_end_pos_pixel,cv::Scalar(0,0,200),3);
    cv::line(im,pos_pixel,y_end_pos_pixel,cv::Scalar(0,0,200),3);
  
    // publish the tracked pose to RVIZ
    geometry_msgs::PoseStamped tracked_pose; 
    tracked_pose.header.stamp = ros::Time::now();
    tracked_pose.header.frame_id = "world"; 
    tracked_pose.pose.position.x = tracker_->particle_.pose.translation().x();
    tracked_pose.pose.position.y = tracker_->particle_.pose.translation().y();
    tracked_pose.pose.position.z = 0.0;
    Eigen::Matrix3d cur_rot;
    cur_rot <<  tracker_->particle_.pose.matrix()(0,0), tracker_->particle_.pose.matrix()(0,1), 0,
                tracker_->particle_.pose.matrix()(1,0), tracker_->particle_.pose.matrix()(1,1), 0,
                                                    0,                                      0,  1;
    Eigen::Quaterniond q(cur_rot);
    tracked_pose.pose.orientation.x = q.x();
    tracked_pose.pose.orientation.y = q.y();
    tracked_pose.pose.orientation.z = q.z();
    tracked_pose.pose.orientation.w = q.w();
    tracked_pose_publisher_->publish(tracked_pose);
  }

  window_output_im_mutex_.lock(); 
  cv::imshow(cv_windows[0], im);
  cv::waitKey(1);
  window_output_im_mutex_.unlock(); 
}

void PoseEstimationSystem::save_to_file(Graph::Frame* f)
{
  file_write_mutex_.lock();

  // set the number precision -> for timestamp!
  results_ofstream_ << std::setprecision(16);
  results_ofstream_ << "---" << std::endl;  // mark that new data comes
  results_ofstream_ << cur_time_ << " ";
  results_ofstream_ << f->timestamp_ << " ";

  // check if track initialized
  std::string track_status = (tracker_->tracking_initialized_) ? "True":"False";
  results_ofstream_ << track_status << " ";

  // check if reset necessary
  std::string reset_status = "none";
  if(tracker_->internal_switch_) reset_status = "switch";
  if(tracker_->full_reset_) reset_status = "reset";
  results_ofstream_ << reset_status << " ";

  // feasible set contracted or not
  std::string feasible_set_contract = "no";
  if(tracker_->contracted_feasible_set_) feasible_set_contract = "contracted";
  results_ofstream_ << feasible_set_contract << " ";

  // track reliability
  results_ofstream_ << tracker_->tracking_reliablility_ << " ";

  // tracked particle age
  results_ofstream_ << tracker_->particle_.age << " ";

  // tracked particle weight
  results_ofstream_ << tracker_->particle_.weight_ << " "; 

  // sine angle hull of observed facade orientations
  if(tracker_->facade_observation_sin_angle_hull_.is_empty() || tracker_->facade_observation_sin_angle_hull_.is_unbounded())
  {
    results_ofstream_ << 0 << " ";
    results_ofstream_ << 0 << " ";
  }
  else
  {
    results_ofstream_ << tracker_->facade_observation_sin_angle_hull_.lb() << " ";
    results_ofstream_ << tracker_->facade_observation_sin_angle_hull_.ub() << " ";
  }

  // Get the poses 
  Eigen::Affine2d map_T_laser_rt = map_localize_T_laser_cur_; // pose at real-time
  results_ofstream_ << map_T_laser_rt.translation()(0) << " "; // x
  results_ofstream_ << map_T_laser_rt.translation()(1) << " "; // y
  results_ofstream_ << atan2(map_T_laser_rt.matrix()(1,0),map_T_laser_rt.matrix()(0,0)) << " "; // rot
  Eigen::Affine2d map_T_laser_loc = tracker_->particle_.pose; // pose at localization time
  results_ofstream_ << map_T_laser_loc.translation()(0) << " "; // x
  results_ofstream_ << map_T_laser_loc.translation()(1) << " "; // y
  results_ofstream_ << atan2(map_T_laser_loc.matrix()(1,0),map_T_laser_loc.matrix()(0,0)) << " "; // rot

  // runtimes
  results_ofstream_ << vo_time_ << " ";
  results_ofstream_ << full_loc_time_ << " ";
  results_ofstream_ << coarse_loc_time_ << " ";
  results_ofstream_ << refined_loc_time_ << " ";

  // feasible set 
  results_ofstream_ << std::endl; 
  global_localizer_->get_localization_result(map_T_laser_loc_feasible_hull_);
  if(!map_T_laser_cur_feasible_hull_.is_empty() && !map_T_laser_loc_feasible_hull_.is_empty() &&
     !map_T_laser_cur_feasible_hull_.is_unbounded() && !map_T_laser_loc_feasible_hull_.is_unbounded())
  {
    // real-time
    results_ofstream_ << map_T_laser_cur_feasible_hull_[0].lb() << " " << map_T_laser_cur_feasible_hull_[0].ub() << " ";
    results_ofstream_ << map_T_laser_cur_feasible_hull_[1].lb() << " " << map_T_laser_cur_feasible_hull_[1].ub() << " ";
    results_ofstream_ << map_T_laser_cur_feasible_hull_[2].lb() << " " << map_T_laser_cur_feasible_hull_[2].ub() << " ";
    // loc time  
    results_ofstream_ << map_T_laser_loc_feasible_hull_[0].lb() << " " << map_T_laser_loc_feasible_hull_[0].ub() << " ";
    results_ofstream_ << map_T_laser_loc_feasible_hull_[1].lb() << " " << map_T_laser_loc_feasible_hull_[1].ub() << " ";
    results_ofstream_ << map_T_laser_loc_feasible_hull_[2].lb() << " " << map_T_laser_loc_feasible_hull_[2].ub() << " ";
  }

  // consistent set 
  results_ofstream_ << std::endl; 
  if(consistent_polygon_cur_.size() > 0 && tracker_->polygone_.size() > 0 && !consistent_rot_cur_.is_empty() && !consistent_rot_cur_.is_unbounded() &&
    !tracker_->rotation_.is_empty() && !tracker_->rotation_.is_unbounded())
  {
    // real-time  
    results_ofstream_ << consistent_rot_cur_.lb() << " " << consistent_rot_cur_.ub() << " "; 
    // create rotated rectangle around consistent_polygon_cur_
    cv::RotatedRect rot_rect_consistent_polygon_cur_;
    cv::Point2f corners_rot_rect_consistent_polygon_cur[4];
    rot_rect_consistent_polygon_cur_ = cv::minAreaRect(consistent_polygon_cur_);
    rot_rect_consistent_polygon_cur_.points(corners_rot_rect_consistent_polygon_cur);
    for(int i = 0; i < 4; i++)
    {
      cv::Point2f& corner = corners_rot_rect_consistent_polygon_cur[i]; 
      results_ofstream_ << corner.x << " " << corner.y << " "; 
    }
    for(int i = 0; i < consistent_polygon_cur_.size(); i++)
    {
      cv::Point2f& corner = consistent_polygon_cur_[i];
      results_ofstream_ << corner.x << " " << corner.y << " ";
    }
    results_ofstream_ << std::endl;
    // loc time
    results_ofstream_ << tracker_->rotation_.lb() << " " << tracker_->rotation_.ub() << " "; 
    std::vector<cv::Point2f> cur_poly_corners(tracker_->polygone_.size()); 
    for(int i = 0; i < tracker_->polygone_.size(); i++)
    {
      const Eigen::Vector2d& p = tracker_->polygone_[i];
      cur_poly_corners[i].x = (float)p(0);
      cur_poly_corners[i].y = (float)p(1);
    }
    cv::RotatedRect rot_rect_consistent_polygon_loc_ = cv::minAreaRect(cur_poly_corners);
    cv::Point2f corners_rot_rect_consistent_polygon_loc[4];
    rot_rect_consistent_polygon_loc_.points(corners_rot_rect_consistent_polygon_loc);
    for(int i = 0; i < 4; i++)
    {
      cv::Point2f& corner = corners_rot_rect_consistent_polygon_loc[i]; 
      results_ofstream_ << corner.x << " " << corner.y << " "; 
    }
    for(int i = 0; i < tracker_->polygone_.size(); i++)
    {
      const Eigen::Vector2d& corner = tracker_->polygone_[i];
      results_ofstream_ << corner(0) << " " << corner(1) << " ";
    }
  }
  else
  {
    results_ofstream_ << std::endl;
  }
  results_ofstream_ << std::endl;

  
  // highest weighted particle in global_localizer_
  results_ofstream_ << tracker_->max_weight_gl_particle_.pose.translation()(0) << " "; // x
  results_ofstream_ << tracker_->max_weight_gl_particle_.pose.translation()(1) << " "; // y
  results_ofstream_ << atan2(tracker_->max_weight_gl_particle_.pose.matrix()(1,0),tracker_->max_weight_gl_particle_.pose.matrix()(0,0)) << " "; // rot
  results_ofstream_ << tracker_->max_weight_gl_particle_.age << " ";
  results_ofstream_ << tracker_->max_weight_gl_particle_.weight_ << " ";
  results_ofstream_ << std::endl;

  file_write_mutex_.unlock();
}

void PoseEstimationSystem::save_only_rt_to_file()
{ 
  only_rt_results_ofstream_ << std::setprecision(16);
  only_rt_results_ofstream_ << "---" << std::endl;  // mark that new data comes
  only_rt_results_ofstream_ << cur_time_ << " ";

  // Get the poses 
  Eigen::Affine2d map_T_laser_rt = map_localize_T_laser_cur_; // pose at real-time
  only_rt_results_ofstream_ << map_T_laser_rt.translation()(0) << " "; // x
  only_rt_results_ofstream_ << map_T_laser_rt.translation()(1) << " "; // y
  only_rt_results_ofstream_ << atan2(map_T_laser_rt.matrix()(1,0),map_T_laser_rt.matrix()(0,0)) << " "; // rot

  // feasible set 
  only_rt_results_ofstream_ << std::endl;
  if(!map_T_laser_cur_feasible_hull_.is_empty() && !map_T_laser_cur_feasible_hull_.is_unbounded())
  {
    // real-time
    only_rt_results_ofstream_ << map_T_laser_cur_feasible_hull_[0].lb() << " " << map_T_laser_cur_feasible_hull_[0].ub() << " ";
    only_rt_results_ofstream_ << map_T_laser_cur_feasible_hull_[1].lb() << " " << map_T_laser_cur_feasible_hull_[1].ub() << " ";
    only_rt_results_ofstream_ << map_T_laser_cur_feasible_hull_[2].lb() << " " << map_T_laser_cur_feasible_hull_[2].ub() << " ";
  }

  // consistent set 
  only_rt_results_ofstream_ << std::endl; 
  if(consistent_polygon_cur_.size() > 0 && !consistent_rot_cur_.is_empty() && !consistent_rot_cur_.is_unbounded())
  {
    // real-time  
    only_rt_results_ofstream_ << consistent_rot_cur_.lb() << " " << consistent_rot_cur_.ub() << " "; 
    // create rotated rectangle around consistent_polygon_cur_
    cv::RotatedRect rot_rect_consistent_polygon_cur_;
    cv::Point2f corners_rot_rect_consistent_polygon_cur[4];
    rot_rect_consistent_polygon_cur_ = cv::minAreaRect(consistent_polygon_cur_);
    rot_rect_consistent_polygon_cur_.points(corners_rot_rect_consistent_polygon_cur);
    for(int i = 0; i < 4; i++)
    {
      cv::Point2f& corner = corners_rot_rect_consistent_polygon_cur[i]; 
      only_rt_results_ofstream_ << corner.x << " " << corner.y << " "; 
    }
    for(int i = 0; i < consistent_polygon_cur_.size(); i++)
    {
      cv::Point2f& corner = consistent_polygon_cur_[i];
      only_rt_results_ofstream_ << corner.x << " " << corner.y << " ";
    }
  }
  only_rt_results_ofstream_ << std::endl;
}