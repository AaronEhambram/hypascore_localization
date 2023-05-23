#include "hypascore_localization/localization/refined/RefinedLocalizer.hpp"
#include "hypascore_localization/localization/refined/HoughAccumulator.hpp"
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/MarkerArray.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/tfMessage.h>
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include <chrono>

CMTracker::CMTracker(ros::NodeHandle& nh, CMMap* map)
{
  nh_ptr = &nh; 
  std::string calibration_folder;

  nh.getParam("/hypascore_localization_node/calibration_folder", calibration_folder);
  nh.getParam("/hypascore_localization_node/min_age_to_initiate_track", min_age_to_initiate_track_);
  nh.getParam("/hypascore_localization_node/min_dist_to_associate_point_to_facade", min_dist_to_associate_point_to_facade_);
  nh.getParam("/hypascore_localization_node/angle_quantization", angle_quantization_);
  nh.getParam("/hypascore_localization_node/d_quantization", d_quantization_);
  nh.getParam("/hypascore_localization_node/min_facade_length", min_facade_length_);
  nh.getParam("/hypascore_localization_node/wall_position_uncertainty", wall_position_uncertainty_);
  nh.getParam("/hypascore_localization_node/wall_orientation_uncertainty", wall_orientation_uncertainty_);
  nh.getParam("/hypascore_localization_node/min_number_points_on_facade", min_number_points_on_facade_);
  nh.getParam("/hypascore_localization_node/max_translation_difference", max_translation_difference_);
  nh.getParam("/hypascore_localization_node/max_rotation_difference", max_rotation_difference_);
  nh.getParam("/hypascore_localization_node/gl_particle_better_factor", gl_particle_better_factor_);
  nh.getParam("/hypascore_localization_node/min_age_to_switch_track", min_age_to_switch_track_);
  nh.getParam("/hypascore_localization_node/min_particle_age_to_overwrite_gl", min_particle_age_to_overwrite_gl_);
  nh.getParam("/hypascore_localization_node/min_sin_angle_hull_diam", min_sin_angle_hull_diam_);
  nh.getParam("/hypascore_localization_node/file_tracker_results", file_tracker_results_);
  nh.getParam("/hypascore_localization_node/pca_eigen_line_length", pca_eigen_line_length_);

  if(file_tracker_results_ != "") // if file_tracker_results_ string is not empty, dump results to file_tracker_results_
  {
    dump_tracker_results_ = true;
    std::cout << "Start saving to: " << file_tracker_results_ << std::endl; 
    results_ofstream_.open(file_tracker_results_.c_str());
  }

  // extrinsic calibration data
  std::string extrinsic_calib_file = calibration_folder+"extrinsics.xml";
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

  // save City Model
  map_ = map;

  // set initial parameters
  tracking_initialized_ = false;
  translation_hull_ = ibex::IntervalVector(2);   

  // Visualization
  associated_points_publisher_.reset(new ros::Publisher);
  *associated_points_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("associated_points_tracking", 1);
  evaluation_lidar_points_publisher_.reset(new ros::Publisher);
  *evaluation_lidar_points_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("evaluation_points_tracking", 1);
  matched_line_publisher_.reset(new ros::Publisher);
  *matched_line_publisher_ = nh.advertise<visualization_msgs::Marker>("matched_line_tracking", 1);
  extracted_line_publisher_.reset(new ros::Publisher);
  *extracted_line_publisher_ = nh.advertise<visualization_msgs::Marker>("extracted_line_tracking", 1);
  position_polygone_publisher_ptr.reset(new ros::Publisher);
  *position_polygone_publisher_ptr = nh.advertise<visualization_msgs::Marker>("position_polygone", 1);
}

PoseParticle& CMTracker::get_max_weight_particle(std::vector<IntervalPose>& gl_feasible_poses, bool& correct_result)
{
  double max_weight = -10000; 
  PoseParticle* gl_particle_max_weight = NULL; 
  IntervalPose* gl_ipose_max_weight = NULL;  
  for(IntervalPose& ipose : gl_feasible_poses)
  {
    if(ipose.max_weight_ > max_weight)
    { 
      max_weight = ipose.max_weight_;
      gl_ipose_max_weight = &ipose;
    }
  }
  for(PoseParticle& particle : gl_ipose_max_weight->particles_)
  {
    if(max_weight <= particle.weight_)
    {
      gl_particle_max_weight = &particle; 
      break; 
    }
  }
  if(gl_particle_max_weight == NULL)
  {
    correct_result = false;
  }
  else
  {
    correct_result = true; 
  }
  return *gl_particle_max_weight; 
}

void CMTracker::initialize_track(std::unique_ptr<LidarScan>& lidar_scan, std::vector<IntervalPose>& gl_feasible_poses, PoseParticle& gl_particle_max_weight)
{
  // initialize the particle 
  particle_ = PoseParticle(); 
  particle_.pose = gl_particle_max_weight.pose; 
  particle_.age = 0; 

  // compute initial region
  ibex::IntervalVector world_t_laser(2); world_t_laser.set_empty();
  ibex::Interval world_theta_laser; world_theta_laser.set_empty();
  for(IntervalPose& ipose : gl_feasible_poses)
  {
    world_theta_laser = world_theta_laser | ipose.rotation; 
    for(ibex::IntervalVector& transaltion_subset : ipose.translation)
    {
      world_t_laser = world_t_laser | transaltion_subset; 
    }
  }

  // Initialize the interval parts based on the associations!
  translation_hull_ = world_t_laser;
  rotation_ = world_theta_laser; 
  
  // Associate the lidar pointcloud to the map based on the current particle! 
  associate_lidar_to_map(lidar_scan);

  // Hough-Transform to each associated line -> provides interval-line-parameters for each line in local frame
  double angle_uncertainty = world_theta_laser.diam(); 
  extract_line_parameters(angle_uncertainty, lidar_scan); // computes local_line_matches_

  // 2.) compute the interval pose estimate 
  // 2.)a.) compute the position boundary from global localization result
  std::vector<PositionBoundary> gl_boundaries;
  PositionBoundary position_boundary_x_init;
  position_boundary_x_init.angle = 0;
  position_boundary_x_init.d = world_t_laser[0];
  position_boundary_x_init.r_color = 0;
  position_boundary_x_init.g_color = 0;
  position_boundary_x_init.b_color = 0;
  gl_boundaries.push_back(position_boundary_x_init);
  PositionBoundary position_boundary_y_init;
  position_boundary_y_init.angle = M_PI/2;
  position_boundary_y_init.d = world_t_laser[1];
  position_boundary_y_init.r_color = 0;
  position_boundary_y_init.g_color = 0;
  position_boundary_y_init.b_color = 0;
  gl_boundaries.push_back(position_boundary_y_init);
  std::vector<ibex::Interval> gl_angle; gl_angle.push_back(world_theta_laser);
  polygon_pose_estimation(gl_boundaries, gl_angle);
}

void CMTracker::update_track(std::unique_ptr<LidarScan>& lidar_scan, std::vector<IntervalPose>& gl_feasible_poses, Eigen::Affine2d& laser_bef_T_laser_cur, ibex::IntervalVector& laser_bef_p_laser_cur)
{
  // compute initial region
  ibex::IntervalVector world_t_laser(2); world_t_laser.set_empty();
  ibex::Interval world_theta_laser; world_theta_laser.set_empty();
  for(IntervalPose& ipose : gl_feasible_poses)
  {
    world_theta_laser = world_theta_laser | ipose.rotation; 
    for(ibex::IntervalVector& transaltion_subset : ipose.translation)
    {
      world_t_laser = world_t_laser | transaltion_subset; 
    }
  }

  // update the interval parts with the odometry
  // 1.) Update the previous polygon by taking the hull and update the hull
  // + 2.) Update the current particle
  update_estimates(laser_bef_T_laser_cur, laser_bef_p_laser_cur);

  // Associate the lidar pointcloud to the map! 
  auto start_association = std::chrono::high_resolution_clock::now();
  associate_lidar_to_map(lidar_scan);
  auto end_association = std::chrono::high_resolution_clock::now();
  auto duration_association = std::chrono::duration_cast<std::chrono::microseconds>(end_association - start_association);
  op_time_association = duration_association.count()*std::pow(10,-6);
  
  // Hough-Transform to each associated line -> provides interval-line-parameters for each line in local frame
  auto start_iHT = std::chrono::high_resolution_clock::now();
  double angle_uncertainty = world_theta_laser.diam();
  extract_line_parameters(angle_uncertainty, lidar_scan); // computes local_line_matches_
  auto end_iHT = std::chrono::high_resolution_clock::now();
  auto duration_iHT = std::chrono::duration_cast<std::chrono::microseconds>(end_iHT - start_iHT);
  op_time_iHT = duration_iHT.count()*std::pow(10,-6);

  // compute the interval localization results based on the associations
  // global localization results
  std::vector<PositionBoundary> gl_boundaries;
  PositionBoundary position_boundary_x_init;
  position_boundary_x_init.angle = 0;
  position_boundary_x_init.d = world_t_laser[0];
  position_boundary_x_init.r_color = 0;
  position_boundary_x_init.g_color = 0;
  position_boundary_x_init.b_color = 0;
  gl_boundaries.push_back(position_boundary_x_init);
  PositionBoundary position_boundary_y_init;
  position_boundary_y_init.angle = M_PI/2;
  position_boundary_y_init.d = world_t_laser[1];
  position_boundary_y_init.r_color = 0;
  position_boundary_y_init.g_color = 0;
  position_boundary_y_init.b_color = 0;
  gl_boundaries.push_back(position_boundary_y_init);
  std::vector<ibex::Interval> gl_angle; 
  gl_angle.push_back(world_theta_laser);
  // odometry updated previous results
  PositionBoundary position_boundary_x_odom;
  position_boundary_x_odom.angle = 0;
  position_boundary_x_odom.d = translation_hull_[0];
  position_boundary_x_odom.r_color = 0;
  position_boundary_x_odom.g_color = 0;
  position_boundary_x_odom.b_color = 0;
  gl_boundaries.push_back(position_boundary_x_odom);
  PositionBoundary position_boundary_y_odom;
  position_boundary_y_odom.angle = M_PI/2;
  position_boundary_y_odom.d = translation_hull_[1];
  position_boundary_y_odom.r_color = 0;
  position_boundary_y_odom.g_color = 0;
  position_boundary_y_odom.b_color = 0;
  gl_boundaries.push_back(position_boundary_y_odom);
  gl_angle.push_back(rotation_);
  auto start_poly = std::chrono::high_resolution_clock::now();
  polygon_pose_estimation(gl_boundaries, gl_angle);
  auto end_poly = std::chrono::high_resolution_clock::now();
  auto duration_poly = std::chrono::duration_cast<std::chrono::microseconds>(end_poly - start_poly);
  op_time_polygon = duration_poly.count()*std::pow(10,-6);
}

void CMTracker::update_estimates(Eigen::Affine2d& laser_bef_T_laser_cur, ibex::IntervalVector& laser_bef_p_laser_cur)
{ 
  // the polygon is not valid anymore -> delete
  polygone_.clear(); 
  local_line_matches_.clear();
  // translation part
  ibex::IntervalVector t_odom_rot(2);
  t_odom_rot[0] = cos(rotation_)*laser_bef_p_laser_cur[0]-sin(rotation_)*laser_bef_p_laser_cur[1];
  t_odom_rot[1] = sin(rotation_)*laser_bef_p_laser_cur[0]+cos(rotation_)*laser_bef_p_laser_cur[1];
  translation_hull_ = translation_hull_ + t_odom_rot;
  // update the orientation
  rotation_ = laser_bef_p_laser_cur[5]+rotation_; 
  if(rotation_.lb() > M_PI)
  {
    rotation_ = rotation_-2*M_PI;
  }
  else if(rotation_.ub() < -M_PI)
  {
    rotation_ = rotation_+2*M_PI;
  }

  // update the particle with the odometry data
  particle_.pose = particle_.pose*laser_bef_T_laser_cur;
}

void CMTracker::line_intersection(Eigen::Vector2d& n1, double c1, Eigen::Vector2d& n2, double c2, Eigen::Vector2d& x)
{
  double& a1 = n1(0);
  double& b1 = n1(1);
  double& a2 = n2(0);
  double& b2 = n2(1);
  // line 1: a1*x+b1*y = c1
  // line 2: a2*x+b2*y = c2
  if(abs(a1) > pow(10,-9))
  {
    x(1) = (c2-a2/a1*c1)/(b2-a2/a1*b1); 
    x(0) = (c1-b1*x(1))/a1;
  }
  else
  {
    x(1) = c1/b1;
    x(0) = (c2-b2*x(1))/a2;
  }
}

bool CMTracker::line_segments_intersect(Eigen::Vector2d& p1, Eigen::Vector2d& p2, Eigen::Vector2d& p3, Eigen::Vector2d& p4, Eigen::Vector2d& intersection)
{
  double& x1 = p1(0);
  double& y1 = p1(1);
  double& x2 = p2(0);
  double& y2 = p2(1);
  double& x3 = p3(0);
  double& y3 = p3(1);
  double& x4 = p4(0);
  double& y4 = p4(1);

  double t = ((x1-x3)*(y3-y4)-(y1-y3)*(x3-x4))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));
  double u = ((x1-x3)*(y1-y2)-(y1-y3)*(x1-x2))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));
  
  if(0.0<=t && t<=1.0 && 0.0<=u && u<=1.0)
  {
    intersection(0) = x1+t*(x2-x1);
    intersection(1) = y1+t*(y2-y1);  
    return true;
  }
  return false;
}

void CMTracker::polygon_pose_estimation(std::vector<PositionBoundary>& further_position_boundaries, std::vector<ibex::Interval>& further_orientations)
{ 
  ibex::Interval rotation_local = rotation_; 
  ibex::IntervalVector translation_hull_local = translation_hull_; 
  for(ibex::Interval& orientation : further_orientations)
  {
    rotation_local = rotation_local & orientation;
  }
  // gather all position boundaries together
  std::vector<PositionBoundary> position_boundaries;
  position_boundaries.insert(position_boundaries.end(), further_position_boundaries.begin(), further_position_boundaries.end());
  for(std::pair<WallGroundLine*,LocalLine>& pair : local_line_matches_)
  {
    WallGroundLine& wgl = *(pair.first);
    LocalLine& ll = pair.second; 
    ibex::Interval& angle_laser = ll.angle;
    ibex::Interval& d_laser = ll.d;
    ibex::Interval angle_world(atan2(wgl.line_params(1),wgl.line_params(0)));
    angle_world.inflate(wall_orientation_uncertainty_);
    ibex::Interval d_world(wgl.line_params(2));
    d_world.inflate(wall_position_uncertainty_);

    // contract the angle
    rotation_local = rotation_local & (angle_world-angle_laser);

    // save the polygone boundaries to contract the pose
    ibex::Interval d_diff = d_world-d_laser;
    PositionBoundary position_boundary;
    position_boundary.angle = angle_world;
    position_boundary.d = d_diff;
    position_boundary.r_color = wgl.r_color;
    position_boundary.g_color = wgl.g_color;
    position_boundary.b_color = wgl.b_color;
    position_boundaries.push_back(position_boundary);
  }
  // compute all intersections between the position-boundaries 
  int n_intersections = 2*(position_boundaries.size()-1)*position_boundaries.size(); // -> folgt aus der Gauss-Summenformel
  std::vector<Eigen::Vector2d> line_intersections;
  for(int idx1 = 0; idx1 < position_boundaries.size(); idx1++)
  {
    Eigen::Vector2d n1(cos(position_boundaries[idx1].angle.mid()),sin(position_boundaries[idx1].angle.mid()));
    ibex::Interval& d1 = position_boundaries[idx1].d;
    for(int idx2 = idx1+1; idx2 < position_boundaries.size(); idx2++)
    {
      Eigen::Vector2d n2(cos(position_boundaries[idx2].angle.mid()),sin(position_boundaries[idx2].angle.mid()));
      ibex::Interval& d2 = position_boundaries[idx2].d;

      Eigen::Vector2d p1;
      line_intersection(n1, d1.lb(), n2, d2.lb(), p1);
      Eigen::Vector2d p2;
      line_intersection(n1, d1.ub(), n2, d2.lb(), p2);
      Eigen::Vector2d p3;
      line_intersection(n1, d1.ub(), n2, d2.ub(), p3);
      Eigen::Vector2d p4;
      line_intersection(n1, d1.lb(), n2, d2.ub(), p4);
      line_intersections.push_back(p1);
      line_intersections.push_back(p2);
      line_intersections.push_back(p3);
      line_intersections.push_back(p4);
    }
  }
  for(int idx1 = 0; idx1 < position_boundaries.size(); idx1++)
  {
    double alpha = position_boundaries[idx1].angle.mid();
    Eigen::Matrix2d w_R_l;
    w_R_l(0,0) = cos(alpha); w_R_l(0,1) = -sin(alpha);
    w_R_l(1,0) = sin(alpha); w_R_l(1,1) = cos(alpha);
    for(int p_counter = 0; p_counter < line_intersections.size(); p_counter++)
    {
      Eigen::Vector2d p_l = w_R_l.inverse()*line_intersections[p_counter]; 
      ibex::Interval line_dist = position_boundaries[idx1].d;
      line_dist.inflate(0.001);
      if(!line_dist.contains(p_l(0)))
      {
        line_intersections.erase(line_intersections.begin()+p_counter);
        p_counter--;
      }
    }
  }
  // save the intersection points as polygone
  std::vector<Eigen::Vector2d> intersections;
  intersections = line_intersections;
  std::vector<cv::Point2f> cv_points;
  polygone_.clear();
  ibex::IntervalVector polygon_hull(2);
  polygon_hull.set_empty();
  if(intersections.size() > 2)
  {
    for(Eigen::Vector2d& p : intersections)
    {
      cv::Point2f cv_p;
      cv_p.x = (float)p.x();
      cv_p.y = (float)p.y();
      cv_points.push_back(cv_p);
      ibex::IntervalVector corner(2); 
      corner[0] = p.x(); 
      corner[1] = p.y();
      polygon_hull = polygon_hull | corner; 
    }
    std::vector<int> polygone_order;
    cv::convexHull(cv_points,polygone_order);
    for(int idx : polygone_order)
    {
      polygone_.push_back(intersections[idx]); 
    }
    polygone_.push_back(polygone_[0]);
  }
  translation_hull_local = translation_hull_local & polygon_hull; 
  if(!rotation_local.is_empty())
  {
    rotation_ = rotation_local; 
  }
  if(!translation_hull_local.is_empty())
  {
    translation_hull_ = translation_hull_local; 
  }
}

void CMTracker::associate_lidar_to_map(std::unique_ptr<LidarScan>& lidar_scan)
{
  std::vector<std::set<WallGroundLine*>> matches(lidar_scan->pointcloud_->size()); 
  // perform matching with multiple threads
  #pragma omp parallel for
  for(int i = 0; i < lidar_scan->pointcloud_->size(); i++)
  {
    pcl::PointXYZ& p = (*(lidar_scan->pointcloud_))[i];
    Eigen::Vector2d laser_p_eig(p.x,p.y);
    Eigen::Vector2d map_p_eig = particle_.pose*laser_p_eig;
    std::set<WallGroundLine*> close_wgl_ptrs;
    map_->wall_ground_lines_radius_search(map_p_eig, min_dist_to_associate_point_to_facade_, close_wgl_ptrs);
    if(close_wgl_ptrs.size() > 0)
    {
      matches[i] = close_wgl_ptrs;
    }
  }

  // reorder the matches into a more convenient form
  std::map<WallGroundLine*,std::vector<size_t>> matches_map;
  for(int i = 0; i < matches.size(); i++)
  {
    for(WallGroundLine* wgl : matches[i])
    {
      matches_map[wgl].push_back(i);
    }
  } 

  // filter line associations based on the number of associated points
  for(std::pair<WallGroundLine*,std::vector<size_t>> match : matches_map)
  {
    // only consider facades on which a minimal number of points is located
    if(match.second.size() > min_number_points_on_facade_)
    {
      particle_.line_measurement_matches[match.first] = match.second; 
    } 
  }
}

void CMTracker::extract_line_parameters(double& angle_uncertainty, std::unique_ptr<LidarScan>& lidar_scan)
{
  Eigen::Affine2d& map_T_laser = particle_.pose;
  double map_theta_laser = atan2(map_T_laser.linear()(1,0),map_T_laser.linear()(0,0));
  if((rotation_-map_theta_laser).contains(2*M_PI))
  {
    map_theta_laser += 2*M_PI;
  }
  else if((rotation_-map_theta_laser).contains(-2*M_PI))
  {
    map_theta_laser -= 2*M_PI;
  }
  // Create Hough-Accumulator object
  HoughAccumulator HA(*nh_ptr);
  // object to save the results
  std::vector<std::pair<WallGroundLine*,LocalLine>> wall_line_local_line_matches;
  local_line_matches_.clear(); 
  // iterate through each associated line and extract local lines
  for(std::pair<WallGroundLine*,std::vector<size_t>> wgl_points_match : particle_.line_measurement_matches)
  {
    WallGroundLine* wgl_ptr = wgl_points_match.first; 
    std::vector<size_t>& measure_idx = wgl_points_match.second; 
    // transform the associated WallGroundLines to the local frame using the particle pose
    double line_angle_world = atan2(wgl_ptr->line_params(1),wgl_ptr->line_params(0));
    double line_angle_laser = line_angle_world - map_theta_laser;
    double line_d_laser = wgl_ptr->line_params(2) - cos(line_angle_world)*map_T_laser.translation().x() - sin(line_angle_world)*map_T_laser.translation().y();
    // set the bounds for the accumulator space based on the locally transformes lines (where we should see them)
    double angle_min = line_angle_laser-angle_uncertainty;
    double angle_max = line_angle_laser+angle_uncertainty;
    double d_min = line_d_laser - min_dist_to_associate_point_to_facade_;
    double d_max = line_d_laser + min_dist_to_associate_point_to_facade_;
    HA.set_accumulator(angle_min, angle_max, angle_quantization_, d_min, d_max, d_quantization_);
    // Fill the data into the accumulator
    HA.fill_data(measure_idx, lidar_scan->boxcloud_); 
    // get the local line parameters
    ibex::Interval angle;
    ibex::Interval d;
    std::set<size_t> considered_idxs; 
    bool valid = HA.process_accumulator(angle,d,considered_idxs);
    if(valid && considered_idxs.size() > min_number_points_on_facade_)
    {
      // the locally extracted line is significant enough
      wall_line_local_line_matches.push_back(std::pair<WallGroundLine*,LocalLine>()); 
      std::pair<WallGroundLine*,LocalLine>& pair = wall_line_local_line_matches.back(); 
      pair.first = wgl_ptr; 
      LocalLine& ll = pair.second; 
      ll.angle = angle; 
      ll.considered_measures = considered_idxs;
      ll.d = d;
    } 
  }

  // Now the locally extracted lines are saved in wall_line_local_line_matches and we know to which facade they belong to
  // Now we need to extract those local lines that have a minimal length to avoid weird associations
  // Therefore we need to compute the length of the local line -> Therefore we need the start and end points
  #pragma omp parallel for 
  for(int i = 0; i < wall_line_local_line_matches.size(); i++)
  {
    std::pair<WallGroundLine*,LocalLine>& wall_line_pair = wall_line_local_line_matches[i]; 
    LocalLine& ll = wall_line_pair.second;
    // compute the orthogonal normal vector components to the local line
    double n_ortho_x = -sin(ll.angle.mid());
    double n_ortho_y = cos(ll.angle.mid());

    std::pair<double,size_t> lowest_angle;
    lowest_angle.first = 1000000;
    lowest_angle.second = 0;
    std::pair<double,size_t> largest_angle;
    largest_angle.first = -1000000;
    largest_angle.second = 0;
    // iterate through each considered point of the local line and compute the distance to the orthogonal line that goes though the laser-frame
    for(size_t idx : ll.considered_measures)
    {
      pcl::PointXYZ& pcl_point = (*(lidar_scan->pointcloud_))[idx];
      double dist = n_ortho_x*pcl_point.x + n_ortho_y*pcl_point.y;
      if(dist < lowest_angle.first)
      {
        lowest_angle.first = dist;
        lowest_angle.second = idx;
      }
      if(dist > largest_angle.first)
      {
        largest_angle.first = dist;
        largest_angle.second = idx;
      }
    }
    // the points that have the largest distances to the orthogonal line are the start and end points
    ll.start_measure_idx = lowest_angle.second;
    ll.end_measure_idx = largest_angle.second;  
  }

  // Check if the local lines are long enough, if yes, they are qualified to local_line_matches_
  pcl::PointCloud<pcl::PointXYZ>& pc = *(lidar_scan->pointcloud_);
  for(int i = 0; i < wall_line_local_line_matches.size(); i++)
  {
    std::pair<WallGroundLine*,LocalLine>& wall_line_pair = wall_line_local_line_matches[i]; 
    LocalLine& ll = wall_line_pair.second;
    // compute the length of the local line
    Eigen::Vector2d ll_start((double)pc[ll.start_measure_idx].x,(double)pc[ll.start_measure_idx].y); 
    Eigen::Vector2d ll_end((double)pc[ll.end_measure_idx].x,(double)pc[ll.end_measure_idx].y); 
    double ll_length = (ll_end - ll_start).norm(); 
    // compute the length of the facade
    WallGroundLine& wgl = *(wall_line_pair.first);
    double wgl_length = (wgl.end_point - wgl.start_point).norm();

    // TODO: Check if the facades overlap (taking the particle pose into account)!!! 

    if(wgl_length >= min_facade_length_ && ll_length >= min_facade_length_)
    {
      // check the point-distribution with PCA
      int num_of_points = ll.considered_measures.size();
      Eigen::MatrixXd A(3,num_of_points);
      // compute the mean point
      Eigen::Vector3d mean; 
      mean.setZero();
      for(const size_t& idx : ll.considered_measures) 
      {
        pcl::PointXYZ& p = pc[idx];
        mean(0) =  mean(0) + (double)p.x;
        mean(1) =  mean(1) + (double)p.y;
        mean(2) =  mean(2) + (double)p.z;
      }
      mean = mean/((double)num_of_points);
      // fill A
      int col_counter = 0; 
      for(const size_t& idx : ll.considered_measures) 
      {
        pcl::PointXYZ& p = pc[idx];
        A(0,col_counter) = (double)p.x - mean(0);
        A(1,col_counter) = (double)p.y - mean(1);
        A(2,col_counter) = (double)p.z - mean(2);
        col_counter++; 
      } 
      Eigen::Matrix3d AAT = A*A.transpose();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(AAT);
      double eigen_len = sqrt(abs(eigensolver.eigenvalues()(2)/((double)num_of_points)));
      if(abs(eigensolver.eigenvectors()(2,2)) < 0.5 && eigen_len >= pca_eigen_line_length_)
      {
        // the facade and the exracted lines are large enough
        local_line_matches_.push_back(wall_line_pair);
      }
    }
  }

}

void CMTracker::improve_particle()
{
  // compute a line for each match on which the vehicle has to be
  if(local_line_matches_.size() > 0)
  {
    Eigen::MatrixXd A(local_line_matches_.size(),2);
    Eigen::VectorXd b(local_line_matches_.size());
    double angle_sum = 0;
    for(int i = 0; i < local_line_matches_.size(); i++)
    {
      std::pair<WallGroundLine*,LocalLine>& match = local_line_matches_[i];
      WallGroundLine& wgl = *(match.first);
      LocalLine& ll = match.second; 
      double angle_world = atan2(wgl.line_params(1),wgl.line_params(0));
      double d_world = wgl.line_params(2);
      double angle_laser = ll.angle.mid();
      double d_laser = ll.d.mid(); 
      double d_diff = d_world-d_laser; 
      A(i,0) = cos(angle_world);
      A(i,1) = sin(angle_world);
      b(i) = d_diff; 
      angle_sum += angle_world-angle_laser;
    }
    Eigen::Vector2d world_t_laser_best = A.colPivHouseholderQr().solve(b);
    double world_angle_laser_best = angle_sum/((double)local_line_matches_.size());

    // get current particle parameters
    Eigen::Vector2d world_t_laser_cur = particle_.pose.translation();
    double world_angle_laser_cur = atan2(particle_.pose.matrix()(1,0),particle_.pose.matrix()(0,0)); 

    //std::cout << world_t_laser_best - world_t_laser_cur << std::endl << world_angle_laser_best - world_angle_laser_cur << std::endl << std::endl; 
    if((world_t_laser_best - world_t_laser_cur).norm() <= min_dist_to_associate_point_to_facade_)
    {
      particle_.pose.matrix()(0,0) = cos(world_angle_laser_best); 
      particle_.pose.matrix()(0,1) = -sin(world_angle_laser_best); 
      particle_.pose.matrix()(0,2) = world_t_laser_best(0); 
      particle_.pose.matrix()(1,0) = sin(world_angle_laser_best);
      particle_.pose.matrix()(1,1) = cos(world_angle_laser_best);
      particle_.pose.matrix()(1,2) = world_t_laser_best(1); 
    }
    else
    {
      if(rotation_.contains(world_angle_laser_best))
      {
        particle_.pose.matrix()(0,0) = cos(world_angle_laser_best); 
        particle_.pose.matrix()(0,1) = -sin(world_angle_laser_best);
        particle_.pose.matrix()(1,0) = sin(world_angle_laser_best);
        particle_.pose.matrix()(1,1) = cos(world_angle_laser_best);
      }
      for(int i = 0; i < local_line_matches_.size(); i++)
      {
        Eigen::Vector2d t = particle_.pose.translation(); 
        double dist = A(i,0)*t.x()+A(i,1)*t.y()-b(i);
        if(dist < min_dist_to_associate_point_to_facade_)
        {
          Eigen::Vector2d t_new = t-Eigen::Vector2d(A(i,0),A(i,1))*dist; 
          particle_.pose.matrix()(0,2) = t_new(0); 
          particle_.pose.matrix()(1,2) = t_new(1);
        }
      }
    }
  }
}

void CMTracker::compute_optimal_particle(Eigen::Vector2d& optimal_position, double& optimal_orientation)
{
  // compute a line for each match on which the vehicle has to be
  // compute a line for each match on which the vehicle has to be
  if(local_line_matches_.size() > 0)
  {
    Eigen::MatrixXd A(local_line_matches_.size(),2);
    Eigen::VectorXd b(local_line_matches_.size());
    double angle_sum = 0;
    for(int i = 0; i < local_line_matches_.size(); i++)
    {
      std::pair<WallGroundLine*,LocalLine>& match = local_line_matches_[i];
      WallGroundLine& wgl = *(match.first);
      LocalLine& ll = match.second; 
      double angle_world = atan2(wgl.line_params(1),wgl.line_params(0));
      double d_world = wgl.line_params(2);
      double angle_laser = ll.angle.mid();
      double d_laser = ll.d.mid(); 
      double d_diff = d_world-d_laser; 
      A(i,0) = cos(angle_world);
      A(i,1) = sin(angle_world);
      b(i) = d_diff; 
      angle_sum += angle_world-angle_laser;
    }
    Eigen::Vector2d world_t_laser_best = A.colPivHouseholderQr().solve(b);
    double world_angle_laser_best = angle_sum/((double)local_line_matches_.size());

    // get current particle parameters
    Eigen::Vector2d world_t_laser_cur = particle_.pose.translation();
    double world_angle_laser_cur = atan2(particle_.pose.matrix()(1,0),particle_.pose.matrix()(0,0)); 

    //std::cout << world_t_laser_best - world_t_laser_cur << std::endl << world_angle_laser_best - world_angle_laser_cur << std::endl << std::endl; 
    if((world_t_laser_best - world_t_laser_cur).norm() <= min_dist_to_associate_point_to_facade_)
    {
      optimal_position = world_t_laser_best;
      optimal_orientation = world_angle_laser_best; 
    }
    else
    {
      optimal_orientation = world_angle_laser_best;
      Eigen::Vector2d t = particle_.pose.translation();
      for(int i = 0; i < local_line_matches_.size(); i++)
      { 
        double dist = A(i,0)*t.x()+A(i,1)*t.y()-b(i);
        if(dist < min_dist_to_associate_point_to_facade_)
        {
          t = t-Eigen::Vector2d(A(i,0),A(i,1))*dist;
        }
      }
      optimal_position = t; 
    }
  }
  else
  {
    // no lines detected -> optimal_position and optimal_orientation are the current particle parameters
    // get current particle parameters
    optimal_position = particle_.pose.translation();
    optimal_orientation = atan2(particle_.pose.matrix()(1,0),particle_.pose.matrix()(0,0)); 
  }
}

void CMTracker::compute_particle_score(PoseParticle& particle, std::unique_ptr<LidarScan>& lidar_scan)
{
  Eigen::Affine2d& pose = particle.pose;

  // transform
  pcl::PointCloud<pcl::PointXY> map_pc;
  map_pc.resize(lidar_scan->pointcloud_filtered_->size());
  for(int i = 0; i < lidar_scan->pointcloud_filtered_->size(); i++)
  {
    pcl::PointXY& p_pcl = (*(lidar_scan->pointcloud_filtered_))[i];
    Eigen::Vector2d laser_p((double)p_pcl.x, (double)p_pcl.y); 
    Eigen::Vector2d map_p = pose*laser_p;
    map_pc[i].x = map_p[0];
    map_pc[i].y = map_p[1];
  }

  // get close facades to the point
  std::map<WallGroundLine*,std::vector<size_t>> line_matches;
  map_->connected_wall_ground_lines_radius_search(map_pc, min_dist_to_associate_point_to_facade_, line_matches);

  // if point close to facade, score up! 
  particle.weight_ = 0;  
  Eigen::Vector2d p_eig, n, p_world_eig;
  double d, abs_distance;
  for(std::pair<WallGroundLine*,std::vector<size_t>> matches : line_matches)
  {
    WallGroundLine& wgl = *(matches.first);
    for(size_t& p_idx : matches.second)
    {
      p_eig(0) = (double)map_pc[p_idx].x;
      p_eig(1) = (double)map_pc[p_idx].y;
      n(0) = wgl.line_params(0);
      n(1) = wgl.line_params(1);
      d = wgl.line_params(2);
      abs_distance = abs(p_eig.transpose()*n-d);
      particle.weight_ += 1.0-1.0/(1.0+exp(-7.0*1.5/min_dist_to_associate_point_to_facade_*(abs_distance-min_dist_to_associate_point_to_facade_/1.5)));
    }
  }
}

void CMTracker::particle_correction(std::unique_ptr<LidarScan>& lidar_scan, std::vector<IntervalPose>& gl_feasible_poses, PoseParticle& gl_particle_max_weight, Eigen::Affine2d& laser_bef_T_laser_cur)
{
  ibex::IntervalVector world_t_laser(2); world_t_laser.set_empty();
  ibex::Interval world_theta_laser; world_theta_laser.set_empty();
  for(IntervalPose& ipose : gl_feasible_poses)
  {
    world_theta_laser = world_theta_laser | ipose.rotation; 
    for(ibex::IntervalVector& transaltion_subset : ipose.translation)
    {
      world_t_laser = world_t_laser | transaltion_subset; 
    }
  }

  // if the particle is outside the feasible set after optimization -> take the best particles pose! an reset translation_hull and rotation_
  double psi_improved = atan2(particle_.pose.linear()(1,0),particle_.pose.linear()(0,0)); 
  ibex::Vector t_improved(2); 
  t_improved[0] = particle_.pose.translation()(0); 
  t_improved[1] = particle_.pose.translation()(1);

  // use the ipose images to check on the translation part!
  bool track_in_feasible_region_translation = true;
  bool track_in_feasible_region_rotation = true;
  
  // check which of the Interval Poses contains the rotation
  IntervalPose* ipose_ptr = NULL;
  for(IntervalPose& ipose : gl_feasible_poses)
  {
    if(ipose.rotation.contains(psi_improved))
    {
      ipose_ptr = &ipose;
      break;
    }
  }
  if(ipose_ptr == NULL)
  {
    std::cout << "rotation outside!" << std::endl; 
    track_in_feasible_region_rotation = false; 
    double max_weight_orientation = atan2(gl_particle_max_weight.pose.linear()(1,0),gl_particle_max_weight.pose.linear()(0,0));
    for(IntervalPose& ipose : gl_feasible_poses)
    {
      if(ipose.rotation.contains(max_weight_orientation))
      {
        ipose_ptr = &ipose;
        break;
      }
    }
  }
  // if there is an IntervalPose, then check for the translation part!
  if(ipose_ptr != NULL)
  {
    // rotation is indeed part of an IntervalPose of the global localization
    Eigen::Vector2d pos_im_origin_trans;
    pos_im_origin_trans(0) = ipose_ptr->translation_hull[0].lb();
    pos_im_origin_trans(1) = ipose_ptr->translation_hull[1].ub(); 
    Eigen::Vector2d local_im_origin_trans = particle_.pose.translation(); 
    Eigen::Vector2d diff = local_im_origin_trans-pos_im_origin_trans;
    int col = (int) std::round(1/ipose_ptr->min_diam_length_*diff(0));
    int row = (int) std::round(-1/ipose_ptr->min_diam_length_*diff(1));
    double cell_value = 0; 
    if(col >= 0 && col <= ipose_ptr->pos_im.size().width-1 && row >= 0 && row <= ipose_ptr->pos_im.size().height-1)
    {
      cell_value = ipose_ptr->pos_im.at<double>(row,col);
    }
    // if cell value 1 -> particle in feasible region
    if(cell_value < 0.5)
    {
      track_in_feasible_region_translation = false;
      std::cout << "translation outside!" << std::endl; 
    }
  }
  else
  {
    track_in_feasible_region_translation = false;
    track_in_feasible_region_rotation = false;
    std::cout << "no consistent feasible ipose found!" << std::endl;
  }
  

  if(!track_in_feasible_region_rotation /*&& gl_particle_max_weight.age > gl_particle_better_factor_*particle_.weight_*/)
  {
    // Have to switch to rotation of global particle due to drift
    particle_.pose.linear() = gl_particle_max_weight.pose.linear();
    rotation_ = world_theta_laser;
    particle_.age = 0;
    std::cout << "switch track rotation because outside fesible region!" << std::endl;
  }
  if(!track_in_feasible_region_translation /*&& gl_particle_max_weight.age > gl_particle_better_factor_*particle_.weight_*/)
  {
    // Have to switch to translation of global particle due to drift
    particle_.pose.translation() = gl_particle_max_weight.pose.translation();
    translation_hull_ = world_t_laser;
    particle_.age = 0;
    std::cout << "switch track translation because outside fesible region!" << std::endl;
  }


  if(track_in_feasible_region_translation || track_in_feasible_region_rotation)
  {
    PoseParticle gl_particle_max_weight_copy = gl_particle_max_weight;
    // compute pose difference
    Eigen::Affine2d diff = particle_.pose.inverse() * gl_particle_max_weight_copy.pose; 
    double diff_angle = abs(atan2(diff.linear()(1,0),diff.linear()(0,0)));
    double diff_trans = diff.translation().norm();
    if(diff_trans > max_translation_difference_ || diff_angle > max_rotation_difference_)
    {
      compute_particle_score(gl_particle_max_weight_copy, lidar_scan);
      compute_particle_score(particle_, lidar_scan);
      if(gl_particle_max_weight_copy.weight_ > gl_particle_better_factor_*particle_.weight_)
      {
        if(gl_particle_max_weight.age >= min_age_to_switch_track_)
        {
          // overwrite the particle pose, since the own particle pose is less good!
          particle_.pose = gl_particle_max_weight_copy.pose; 
          particle_.age = 0;
          // Have to switch to global particle anyway, since the own particle drifted away
          translation_hull_ = world_t_laser;
          rotation_ = world_theta_laser;
          std::cout << "switch track to better particle!" << std::endl; 
        }
      }
      else
      {
        // set the global localization best particle to the improved tracked particle pose!
        gl_particle_max_weight.pose = particle_.pose;
        particle_.age++;  
      }
    } 
    else
    {
      particle_.age++;
    }
  }
}

void CMTracker::particle_correction_V2(std::unique_ptr<LidarScan>& lidar_scan, std::vector<IntervalPose>& gl_feasible_poses, PoseParticle& gl_particle_max_weight, Eigen::Affine2d& laser_bef_T_laser_cur)
{
  // compute the optimal particle parameters, these are optimal with respect to the current association
  Eigen::Vector2d optimal_position; 
  double optimal_orientation;
  compute_optimal_particle(optimal_position, optimal_orientation);
  unbounded_opt_pose_.translation() = optimal_position;
  unbounded_opt_pose_.linear()(0,0) = cos(optimal_orientation); unbounded_opt_pose_.linear()(0,1) = -sin(optimal_orientation);
  unbounded_opt_pose_.linear()(1,0) = sin(optimal_orientation); unbounded_opt_pose_.linear()(1,1) = cos(optimal_orientation);

  // compute current particle parameters
  Eigen::Vector2d cur_position = particle_.pose.translation();
  double cur_orientation = atan2(particle_.pose.matrix()(1,0),particle_.pose.matrix()(0,0));

  // compute the OpenCV polygon for point test (we will need that in the coming lines)
  // REMINDER: polygone_ is ordered anti-clockwise AND has the very first point also at the end!!!
  std::vector<cv::Point2f> cv_polygon;
  for(int i = 0; i < (int)polygone_.size()-1; i++) // exclude the very last point, since it is identical to the first point
  {
    Eigen::Vector2d& p = polygone_[i]; 
    cv::Point2f cv_p;
    cv_p.x = (float)p.x();
    cv_p.y = (float)p.y();
    cv_polygon.push_back(cv_p);
  }
  // We can ONLY switch tracks, if the there is an old enough better particle than the current (improved) one, or one of the states are empty
  bool switch_mendatory = false; // the bool if the current particle should be overwitten with the best particle from global localization
  // 1.) improve the current particle parameters 
  if(cv_polygon.size() > 2 && !rotation_.is_empty())
  {
    // 1.1.) polygone_ and rotation_ are not empty
    cv::Point2f cv_opt_pos;
    cv_opt_pos.x = (float)optimal_position.x();
    cv_opt_pos.y = (float)optimal_position.y();
    double optimal_position_in_poly = cv::pointPolygonTest(cv_polygon,cv_opt_pos,false); // if 1 -> inside, -1 -> outside, 0 -> on polygon
    if(optimal_position_in_poly >= 1 && rotation_.contains(optimal_orientation))
    {
      // 1.1) a) the improved particle parameters are inside polygone_ and rotation_
      // We can update the parameters,since it is feasible!
      particle_.pose.matrix()(0,0) = cos(optimal_orientation); 
      particle_.pose.matrix()(0,1) = -sin(optimal_orientation); 
      particle_.pose.matrix()(0,2) = optimal_position(0); 
      particle_.pose.matrix()(1,0) = sin(optimal_orientation);
      particle_.pose.matrix()(1,1) = cos(optimal_orientation);
      particle_.pose.matrix()(1,2) = optimal_position(1);
    }
    else
    {
      // 1.1) b) if not inside, consider the parameters that are close to the optimal, but still inside polygone_ and rotation_
      // First we consider the rotation
      if(rotation_.contains(optimal_orientation))
      {
        // optimal rotation is inside rotation_, so just copy the optimal rotation
        particle_.pose.matrix()(0,0) = cos(optimal_orientation); 
        particle_.pose.matrix()(0,1) = -sin(optimal_orientation);
        particle_.pose.matrix()(1,0) = sin(optimal_orientation);
        particle_.pose.matrix()(1,1) = cos(optimal_orientation);
      }
      else
      {
        // the optimal rotation is outside rotation_! -> select the closest rotation inside the interval
        ibex::Interval rot_diff = rotation_ - optimal_orientation;
        if(abs(rotation_.lb()-optimal_orientation) < abs(rotation_.ub()-optimal_orientation))
        {
          optimal_orientation = rotation_.lb() + 0.0001;
        }
        else
        {
          optimal_orientation = rotation_.ub() - 0.0001;
        }
        particle_.pose.matrix()(0,0) = cos(optimal_orientation); 
        particle_.pose.matrix()(0,1) = -sin(optimal_orientation);
        particle_.pose.matrix()(1,0) = sin(optimal_orientation);
        particle_.pose.matrix()(1,1) = cos(optimal_orientation);
      }

      // Second we consider the translation
      if(optimal_position_in_poly >= 0)
      {
        // the optimal position is inside or on the border of the polygon -> so its fine
        particle_.pose.matrix()(0,2) = optimal_position(0);
        particle_.pose.matrix()(1,2) = optimal_position(1);
      }
      else
      {
        // the optimal position is not in the polygon!
        // choose the position that is inside the polygon and as close as possible to the optimal position 
        // iterate through the edges of the polygon
        bool intersection_is_there = false; 
        for(int poly_idx = 0; poly_idx < (int)polygone_.size()-1; poly_idx++)
        {
          // compute the intersection with each edge of the polygon
          Eigen::Vector2d& c1 = polygone_[poly_idx]; 
          Eigen::Vector2d& c2 = polygone_[poly_idx+1];
          Eigen::Vector2d intersection;
          intersection_is_there = line_segments_intersect(c1, c2, cur_position, optimal_position, intersection);
          if(intersection_is_there)
          {
            // found the intersecting edge
            // the intersection point is the best position
            Eigen::Vector2d cur_to_optimal = optimal_position - cur_position; 
            optimal_position = intersection + cur_to_optimal/cur_to_optimal.norm() * 0.001; // shift 1mm into the polygon, so that it not on the border
            particle_.pose.matrix()(0,2) = optimal_position(0);
            particle_.pose.matrix()(1,2) = optimal_position(1);
            break;
          }
        }
        if(!intersection_is_there)
        {
          // no intersection -> the current particle has to be outside the polygon (this should not happen!!!)
          // but in case that this happens -> the current track has problems! -> switch the track to better particle! (jump to 2)
          switch_mendatory = true; 
        }
      }
    }
  }
  else
  {
    // 1.2) if polygone_ or rotation_ are empty, the current track has problems! -> switch the track to better particle! (jump to 2)
    switch_mendatory = true;
  }

  // 1. a) if switch is not mendatory, check whether the particle is indside the feasible region (sometimes smaller than polygon)
  if(!switch_mendatory)
  {
    bool translation_in_feasible_region = false; 
    for(int i = 0; i < gl_feasible_poses.size(); i++)
    {
      IntervalPose& ipose = gl_feasible_poses[i];
      Eigen::Vector2d pos_im_origin_trans;
      pos_im_origin_trans(0) = ipose.translation_hull[0].lb();
      pos_im_origin_trans(1) = ipose.translation_hull[1].ub(); 
      Eigen::Vector2d local_im_origin_trans = particle_.pose.translation(); 
      Eigen::Vector2d diff = local_im_origin_trans-pos_im_origin_trans;
      int col = (int) std::round(1/ipose.min_diam_length_*diff(0));
      int row = (int) std::round(-1/ipose.min_diam_length_*diff(1));
      double cell_value = 0; 
      if(col >= 0 && col <= ipose.pos_im.size().width-1 && row >= 0 && row <= ipose.pos_im.size().height-1)
      {
        cell_value = ipose.pos_im.at<double>(row,col);
      }
      // if cell value 1 -> particle in feasible region
      if(cell_value > 0)
      {
        translation_in_feasible_region = true; 
        break;
      }
    }
    // if translation not inside feasible set, build KD-Tree for each i_pose
    if(!translation_in_feasible_region)
    {
      std::vector<pcl::KdTreeFLANN<pcl::PointXY>> ipose_kd_trees(gl_feasible_poses.size()); 
      std::vector<pcl::PointCloud<pcl::PointXY>::Ptr> corresponding_pointclouds(gl_feasible_poses.size());
      std::vector<std::pair<Eigen::Vector2d,float>> closest_point_sqr_dists(gl_feasible_poses.size()); 
      Eigen::Vector2d particle_pos = particle_.pose.translation();
      pcl::PointXY particle_pos_pcl; 
      particle_pos_pcl.x = (float)particle_pos.x();
      particle_pos_pcl.y = (float)particle_pos.y();
      #pragma omp parallel for
      for(int i = 0; i < gl_feasible_poses.size(); i++)
      {
        IntervalPose& ipose = gl_feasible_poses[i];
        // create locations
        std::vector<cv::Point2i> locations;   // output, locations of non-zero pixels
        cv::Mat pos_im_char(ipose.pos_im.size(),CV_8UC1);
        ipose.pos_im.convertTo(pos_im_char,CV_8UC1); 
        cv::findNonZero(pos_im_char, locations);
        // transform each location to world frame
        pcl::PointCloud<pcl::PointXY>::Ptr& pc = corresponding_pointclouds[i];
        pc.reset(new pcl::PointCloud<pcl::PointXY>);
        pcl::KdTreeFLANN<pcl::PointXY>& kd_tree = ipose_kd_trees[i]; 
        pc->resize(locations.size());
        for(int pix_counter = 0; pix_counter < locations.size(); pix_counter++)
        {
          Eigen::Vector2d pos_im_origin_trans;
          pos_im_origin_trans(0) = ipose.translation_hull[0].lb();
          pos_im_origin_trans(1) = ipose.translation_hull[1].ub();
          cv::Point2i& pix = locations[pix_counter];
          int& col = pix.x;
          int& row = pix.y;
          Eigen::Vector2d new_trans;
          new_trans(0) = (double)col*ipose.min_diam_length_+pos_im_origin_trans(0);
          new_trans(1) = -(double)row*ipose.min_diam_length_+pos_im_origin_trans(1);

          (*pc)[pix_counter].x = (float)new_trans.x();
          (*pc)[pix_counter].y = (float)new_trans.y();
        }
        if(locations.size() > 0)
        {
          kd_tree.setInputCloud(pc);
          std::vector<int> indices; //indices.resize(1);
          std::vector<float> sqr_dist; //sqr_dist.resize(1); 
          kd_tree.nearestKSearch(particle_pos_pcl,1,indices,sqr_dist);
          closest_point_sqr_dists[i].first = Eigen::Vector2d((double)(*pc)[indices[0]].x,(double)(*pc)[indices[0]].y);
          closest_point_sqr_dists[i].second = sqr_dist[0]; 
        }
        else
        {
          closest_point_sqr_dists[i].second = 10000000000.0f; 
        }
      }
      // get the closest position among all Interval Poses
      float smallest_dist = 100000; 
      Eigen::Vector2d smallest_dist_point; 
      for(std::pair<Eigen::Vector2d,float>& closest_positions : closest_point_sqr_dists)
      {
        if(closest_positions.second < smallest_dist)
        {
          smallest_dist = closest_positions.second;
          smallest_dist_point = closest_positions.first;
        }
      }
      // set the particle position to the closest point in the feasible set
      particle_.pose.matrix()(0,2) = smallest_dist_point(0);
      particle_.pose.matrix()(1,2) = smallest_dist_point(1);
    }
  }

  /*// 1. b) if switch is not mendatory, check whether partcile has associated point that are behind a wall
  if(!switch_mendatory)
  {
    for(std::pair<WallGroundLine*,std::vector<size_t>> match : particle_.line_measurement_matches)
    {
      WallGroundLine* matched_wgl = (match.first);
      std::vector<size_t>& point_idxs = match.second; 
      std::vector<bool> occluded_by_other_wall(point_idxs.size()); 
      int max_counter = (int)((double)point_idxs.size()*0.3);
      #pragma omp parallel for
      for(int i = 0; i < max_counter; i++)
      {
        pcl::PointXYZ& pcl_p = (*(lidar_scan->pointcloud_))[point_idxs[i/0.3]]; 
        Eigen::Vector2d p_laser(pcl_p.x,pcl_p.y);
        // only rotate the local vector so that we obtain the local vector in the world frame pointing from the particle to the meausured point
        Eigen::Vector2d map_p_particle_to_point = particle_.pose.rotation()*p_laser;
        Eigen::Vector2d search_point = particle_.pose.translation()+0.5*map_p_particle_to_point;
        Eigen::Vector2d particle_position = particle_.pose.translation();
        Eigen::Vector2d p_world = particle_.pose*p_laser;
        double radius = p_laser.norm(); 
        std::set<WallGroundLine*> close_wgl_ptrs;
        map_->wall_ground_lines_radius_search(search_point, radius, close_wgl_ptrs);
        for(WallGroundLine* wgl_ptr : close_wgl_ptrs)
        {
          if(wgl_ptr != matched_wgl)
          {
            Eigen::Vector2d intersection;
            // reduce the length of the wgls, since wall uncertainty can cause false alarms
            Eigen::Vector2d start_end_normed = wgl_ptr->end_point - wgl_ptr->start_point;
            start_end_normed.normalize();
            Eigen::Vector2d start_reduced = wgl_ptr->start_point + start_end_normed*wall_position_uncertainty_;
            Eigen::Vector2d end_reduced = wgl_ptr->end_point - start_end_normed*wall_position_uncertainty_;
            occluded_by_other_wall[i] = line_segments_intersect(particle_position, p_world, start_reduced, end_reduced, intersection);
          }
          else
          {
            occluded_by_other_wall[i] = false; // the intersection would only be with the associated wall, thats fine
          }
        }
      }
      // if more than 50% of the associated points are occluded -> need to switch!
      double num_occlusions = 0.0; 
      for(bool occluded : occluded_by_other_wall)
      {
        if(occluded)
        {
          num_occlusions++;
        }
      }
      double occlusion_ratio = num_occlusions/(double)max_counter; 
      if(occlusion_ratio > 0.5)
      {
        std::cout << occlusion_ratio << std::endl;
        switch_mendatory = true; // switch is mendatory due to occlusion!
        break; 
      }
    }
  }*/

  // 2.) check if there is better particle that is old enough -> if yes, switch is mendatory!
  compute_particle_score(gl_particle_max_weight, lidar_scan);
  compute_particle_score(particle_, lidar_scan);
  if(!switch_mendatory)
  {
    // only compute this if the switch is not necessary up to now!
    Eigen::Matrix3d diff = particle_.pose.inverse().matrix() * gl_particle_max_weight.pose.matrix(); 
    double diff_angle = abs(atan2(diff(1,0),diff(0,0)));
    Eigen::Vector2d diff_trans_vec(diff(0,2),diff(1,2)); 
    double diff_trans = diff_trans_vec.norm();
    if(diff_trans > max_translation_difference_ || diff_angle > max_rotation_difference_)
    {
      //compute_particle_score(gl_particle_max_weight, lidar_scan);
      //compute_particle_score(particle_, lidar_scan);
      if(gl_particle_max_weight.weight_ > gl_particle_better_factor_*particle_.weight_)
      {
        if(gl_particle_max_weight.age >= min_age_to_switch_track_)
        {
          switch_mendatory = true; 
        }
        else
        {
          particle_.age = 0; 
        }
      }
    }
  }

  // 3.) if switch mendatory then:
  if(switch_mendatory)
  {
    double optimal_position_in_poly = -1; 
    if(cv_polygon.size() > 2)
    {
      // check if optimal position in polygon
      cv::Point2f cv_opt_pos;
      cv_opt_pos.x = (float)optimal_position.x();
      cv_opt_pos.y = (float)optimal_position.y();
      optimal_position_in_poly = cv::pointPolygonTest(cv_polygon,cv_opt_pos,false);
    }
    if(optimal_position_in_poly >= 1 && rotation_.contains(optimal_orientation))
    {
      // 2.1.) if better particle is inside polygone_ and rotation_ take without consideration (no reset necessary)
      // We can update the parameters,since it is feasible!
      std::cout << "Switch: no reset" << std::endl;
      particle_.pose = gl_particle_max_weight.pose;
      particle_.weight_ = gl_particle_max_weight.weight_;
      particle_.age = 0; 
      full_reset_ = false;
      internal_switch_ = true; 
    }
    else
    {
      std::cout << "Switch: RESET" << std::endl;
      // 2.2.) if better particle is outside polygone_ and rotation_ take without consideration, reset translation_hull_, polygone_ and rotation_
      particle_.pose = gl_particle_max_weight.pose;
      particle_.weight_ = gl_particle_max_weight.weight_;

      ibex::IntervalVector gl_world_t_laser(2); gl_world_t_laser.set_empty();
      ibex::Interval gl_world_theta_laser; gl_world_theta_laser.set_empty();
      for(IntervalPose& ipose : gl_feasible_poses)
      {
        gl_world_theta_laser = gl_world_theta_laser | ipose.rotation; 
        for(ibex::IntervalVector& translation_subset : ipose.translation)
        {
          gl_world_t_laser = gl_world_t_laser | translation_subset; 
        }
      }
      translation_hull_ = gl_world_t_laser;
      rotation_ = gl_world_theta_laser;
      particle_.age = 0; 
      full_reset_ = true;
      internal_switch_ = false;
    }
  }
  else
  {
    // set the global localization best particle to the improved tracked particle pose!
    if(particle_.weight_ >= gl_particle_max_weight.weight_) // only overwrite the best particle 
    {
      gl_particle_max_weight.pose = particle_.pose;
      gl_particle_max_weight.weight_ = particle_.weight_;
    }
    particle_.age++;
    full_reset_ = false;
    internal_switch_ = false;  
  }
  if(particle_.age >= min_particle_age_to_overwrite_gl_)
  {
    tracking_reliablility_ = 1.0;
  }
  else
  {
    tracking_reliablility_ =  (double)particle_.age/(double)min_particle_age_to_overwrite_gl_;
  }
}

void CMTracker::track(std::unique_ptr<LidarScan>& lidar_scan, std::vector<IntervalPose>& gl_feasible_poses, Eigen::Affine2d& laser_bef_T_laser_cur, ibex::IntervalVector& laser_bef_p_laser_cur, double& time)
{ 
  // start timer 
  auto start_time_full = std::chrono::high_resolution_clock::now();

  // Delete the old line-measurement matches
  particle_.line_measurement_matches.clear();
  contracted_feasible_set_ = false; 

  // set the current timestamp
  processed_timestamp_ = time;

  int num_particles = 0; 
  for(IntervalPose& ipose : gl_feasible_poses)
  {
    num_particles += ipose.particles_.size();
  }
  if(num_particles > 0)
  {
    // Get the maximally weighted particle from the global localization result
    bool correct_max_weight_particle_result;
    PoseParticle& gl_particle_max_weight = get_max_weight_particle(gl_feasible_poses, correct_max_weight_particle_result);
    // IMPORTANT: gl_particle_max_weight is the already updated particle!

    if(correct_max_weight_particle_result)
    {
      // check if there is already a track 
      if(tracking_initialized_)
      {
        // a track is already initialized
        // 1. update the states
        update_track(lidar_scan, gl_feasible_poses, laser_bef_T_laser_cur, laser_bef_p_laser_cur); 

        // 2. compare if current track is still good enough or switch necessary
        //particle_correction(lidar_scan, gl_feasible_poses, gl_particle_max_weight, laser_bef_T_laser_cur);
        particle_correction_V2(lidar_scan, gl_feasible_poses, gl_particle_max_weight, laser_bef_T_laser_cur);

        // 3. Check if the global localization translations can be overwritten
        facade_observation_sin_angle_hull_.set_empty();
        // copute the sine-angle-hull
        for(std::pair<WallGroundLine*,LocalLine>& match : local_line_matches_)
        {
          LocalLine& ll = match.second;
          facade_observation_sin_angle_hull_ = facade_observation_sin_angle_hull_ | sin(ll.angle);
        }
        if(particle_.age >= min_particle_age_to_overwrite_gl_ && facade_observation_sin_angle_hull_.diam() >= min_sin_angle_hull_diam_)
        {
          // the particles are old enough and facade_observation_sin_angle_hull_ (good orentations of the walls)
          contracted_feasible_set_ = true;
          for(int i = 0; i < gl_feasible_poses.size(); i++)
          {
            // overwrite the interval pose translations parts
            IntervalPose& ipose = gl_feasible_poses[i]; 
            ipose.translation.clear();
            if(!((ipose.rotation & rotation_).is_empty()))
            {
              ipose.rotation = ipose.rotation & rotation_;
              ipose.translation.push_back(translation_hull_);
            }  
          }
        }
      }
      else
      {
        // No track was initialized -> check if the particle is good enough to initiate a track
        if(gl_particle_max_weight.age >= min_age_to_initiate_track_)
        {
          // The particle is good enough to start a track -> initialize new track
          initialize_track(lidar_scan, gl_feasible_poses, gl_particle_max_weight); 
          particle_correction_V2(lidar_scan, gl_feasible_poses, gl_particle_max_weight, laser_bef_T_laser_cur);
          tracking_initialized_ = true; 
        }
      }
      visualize_associated_points(lidar_scan);
      max_weight_gl_particle_ = gl_particle_max_weight;
    }
  }
  auto end_time_full = std::chrono::high_resolution_clock::now();
  auto duration_full = std::chrono::duration_cast<std::chrono::microseconds>(end_time_full - start_time_full);
  op_time_full = duration_full.count()*std::pow(10,-6);
  // if results should be save, do so:
  if(dump_tracker_results_) save_to_file(lidar_scan); 
}

/***Visualization***/
void CMTracker::visualize_associated_points(std::unique_ptr<LidarScan>& lidar_scan)
{
  visualization_msgs::Marker marker; 
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "associated line";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  tf::Quaternion q_tf;
  tf::Matrix3x3 obs_mat3d;
  obs_mat3d.setIdentity(); 
  obs_mat3d.getRotation(q_tf);
  marker.pose.orientation.x = q_tf.getX();
  marker.pose.orientation.y = q_tf.getY();
  marker.pose.orientation.z = q_tf.getZ();
  marker.pose.orientation.w = q_tf.getW();
  marker.scale.x = 0.2;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.color.a = 1; // Don't forget to set the alpha!
  marker.color.r = 1;
  marker.color.g = 1;
  marker.color.b = 1;
  std_msgs::ColorRGBA p_color; 
  p_color.a = 0.8; 
  p_color.r = 0;
  p_color.g = 1;
  p_color.b = 0;
  pcl::PointCloud<pcl::PointXYZRGB> pc_matched;
  pc_matched.header.frame_id = "world";
  pc_matched.header.stamp = processed_timestamp_*1000000; // pcl::PointCloud wants microseconds
  pc_matched.width = 1;
  int points_amount = 0;
  for(std::pair<WallGroundLine*,LocalLine> line_measurement_match : local_line_matches_)
  {
    points_amount += line_measurement_match.second.considered_measures.size();
  }
  pc_matched.height = points_amount; 
  for(std::pair<WallGroundLine*,LocalLine> line_measurement_match : local_line_matches_)
  {
    WallGroundLine* wgl_ptr = line_measurement_match.first;

    p_color.r = wgl_ptr->r_color;//static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    p_color.g = wgl_ptr->g_color;//static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    p_color.b = wgl_ptr->b_color;//static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

    geometry_msgs::Point p2;
    p2.x = wgl_ptr->start_point(0);
    p2.y = wgl_ptr->start_point(1);
    p2.z = -2;
    marker.points.push_back(p2);
    marker.colors.push_back(p_color);

    geometry_msgs::Point p1;
    p1.x = wgl_ptr->end_point(0);
    p1.y = wgl_ptr->end_point(1);
    p1.z = -2;
    marker.points.push_back(p1);
    marker.colors.push_back(p_color);

    Eigen::Vector2d mid_point = (wgl_ptr->start_point+wgl_ptr->end_point)/2;
    geometry_msgs::Point pmid;
    pmid.x = mid_point(0);
    pmid.y = mid_point(1);
    pmid.z = -2;

    for(size_t idx : line_measurement_match.second.considered_measures)
    {
      pcl::PointXYZ& point = (*(lidar_scan->pointcloud_))[idx];
      Eigen::Vector2d p_laser_eig, p_map_eig;
      p_laser_eig(0) = (double)point.x;
      p_laser_eig(1) = (double)point.y; 
      p_map_eig = particle_.pose*p_laser_eig; 
      pcl::PointXYZRGB color_point;
      color_point.x = (float)(p_map_eig(0));
      color_point.y = (float)(p_map_eig(1));
      color_point.z = point.z; 
      color_point.r = p_color.r*255;
      color_point.g = p_color.g*255;
      color_point.b = p_color.b*255;
      //pc_matched.push_back(point);
      pc_matched.push_back(color_point);
    }
  }
  associated_points_publisher_->publish(pc_matched);
  matched_line_publisher_->publish(marker);

  /*// publish the evaluation points
  {
    std_msgs::ColorRGBA p_color; 
    p_color.a = 0.8; 
    p_color.r = 1;
    p_color.g = 1;
    p_color.b = 1;
    pcl::PointCloud<pcl::PointXYZRGB> pc;
    pc.header.frame_id = "laser";
    pc.header.stamp = processed_timestamp_*1000000; // pcl::PointCloud wants microseconds
    pc.width = 1;
    int points_amount = (*(lidar_scan->pointcloud_)).size();
    pc.height = points_amount; 
    for(pcl::PointXYZ& point : (*(lidar_scan->pointcloud_)))
    {
      pcl::PointXYZRGB color_point;
      color_point.x = point.x;
      color_point.y = point.y;
      color_point.z = point.z; 
      color_point.r = p_color.r*255;
      color_point.g = p_color.g*255;
      color_point.b = p_color.b*255;
      pc.push_back(color_point);
    }
    evaluation_lidar_points_publisher_->publish(pc);
  }*/

  /*// publish ground_map
  visualization_msgs::Marker line_marker; 
  line_marker.header.frame_id = "laser";
  line_marker.header.stamp = ros::Time(processed_timestamp_);
  line_marker.ns = "hough_lines";
  line_marker.id = 0;
  line_marker.type = visualization_msgs::Marker::LINE_LIST;
  line_marker.pose.position.x = 0;
  line_marker.pose.position.y = 0;
  line_marker.pose.position.z = 0;
  //tf::Quaternion q_tf;
  //tf::Matrix3x3 obs_mat3d;
  obs_mat3d.setIdentity(); 
  obs_mat3d.getRotation(q_tf);
  line_marker.pose.orientation.x = q_tf.getX();
  line_marker.pose.orientation.y = q_tf.getY();
  line_marker.pose.orientation.z = q_tf.getZ();
  line_marker.pose.orientation.w = q_tf.getW();
  line_marker.scale.x = 0.2;
  line_marker.scale.y = 1;
  line_marker.scale.z = 1;
  line_marker.color.a = 1; // Don't forget to set the alpha!
  line_marker.color.r = 1;
  line_marker.color.g = 1;
  line_marker.color.b = 1;
  //std_msgs::ColorRGBA p_color; 
  p_color.a = 0.8; 
  p_color.r = 1;
  p_color.g = 1;
  p_color.b = 1;
  for(std::pair<WallGroundLine*,LocalLine> line_measurement_match : local_line_matches_)
  {
    
    p_color.r = line_measurement_match.first->r_color;
    p_color.g = line_measurement_match.first->g_color;
    p_color.b = line_measurement_match.first->b_color;

    ibex::Interval& angle = line_measurement_match.second.angle;
    ibex::Interval& d = line_measurement_match.second.d;

    geometry_msgs::Point p2;
    p2.x = -30;//localizer->line_measurement_results_[line_measurement_match.first].start_box[0].mid();
    p2.y = (d.mid()-(p2.x)*cos(angle.mid()))/sin(angle.mid());//localizer->line_measurement_results_[line_measurement_match.first].start_box[1].mid();//
    p2.z = -2;
    line_marker.points.push_back(p2);
    line_marker.colors.push_back(p_color);

    geometry_msgs::Point p1;
    p1.x = 30;//localizer->line_measurement_results_[line_measurement_match.first].end_box[0].mid();
    p1.y = (d.mid()-(p1.x)*cos(angle.mid()))/sin(angle.mid());//localizer->line_measurement_results_[line_measurement_match.first].end_box[1].mid();//
    p1.z = -2;
    line_marker.points.push_back(p1);
    line_marker.colors.push_back(p_color);
  }
  extracted_line_publisher_->publish(line_marker);*/

  // draw position polygon
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time(processed_timestamp_);
    marker.ns = "position_polygone";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.23;
    marker.color.a = 0.5; // Don't forget to set the alpha!
    
    marker.color.r = 0.6*(1.0-tracking_reliablility_);
    marker.color.g = 0.6*tracking_reliablility_;
    marker.color.b = 0;
    for(Eigen::Vector2d& point : polygone_)
    {
      geometry_msgs::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = 0;
      marker.points.push_back(p);
    }    
    position_polygone_publisher_ptr->publish(marker);
  }
}

/***Evaluation Output***/
void CMTracker::save_to_file(std::unique_ptr<LidarScan>& lidar_scan)
{
  /**
   * the following data will be dumped (to the file ... if ... is set to true) for evaluation purposes:
   * - polygon points
   * - rotated box hull of the polygon (4 corner points, width and length)
   * - angle inteerval lower bound upper bound
   * - bounded optimal pose x,y,theta
   * - unbounded optimal pose x,y,theta
   * - number of associated facades
   * - ratio of how many points are associated and points in the pointcloud
   * - operation times
   */
  /**Data preparation**/
  // rotated box hull
  cv::RotatedRect rot_rect;
  cv::Point2f rot_rect_corners[4];
  if(polygone_.size() > 2)
  {
    std::vector<cv::Point2f> cv_polygon_points(polygone_.size());
    for(int i = 0; i < polygone_.size(); i++) 
    {
      Eigen::Vector2d& p = polygone_[i];
      cv::Point2f& cv_p = cv_polygon_points[i]; 
      cv_p.x = p(0);
      cv_p.y = p(1);
    }
    rot_rect = cv::minAreaRect(cv_polygon_points);
    rot_rect.points(rot_rect_corners);
  }

  /**Save to to file**/
  // set the precision
  results_ofstream_ << std::setprecision(16);
  
  // mark the start of new time stamp
  results_ofstream_ << "---" << std::endl;

  // 1. time 
  results_ofstream_ << processed_timestamp_ << std::endl;

  // 2. polygon points 
  for(Eigen::Vector2d& p : polygone_) 
  {
    results_ofstream_ << p(0) << " " << p(1) << " "; 
  }
  results_ofstream_ << std::endl;

  // 3. rotated hull corner points and height and width
  if(polygone_.size() > 2)
  {
    for(int i = 0; i < 4; i++)
    {
      cv::Point2f& corner = rot_rect_corners[i]; 
      results_ofstream_ << corner.x << " " << corner.y << " "; 
    }
    results_ofstream_ << rot_rect.size.width << " " << rot_rect.size.height << std::endl;
  } 
  else
  {
    results_ofstream_ << std::endl;
  }

  // 4. angle interval lb and ub
  if(polygone_.size() > 2)
  {
    results_ofstream_ << rotation_.lb() << " " << rotation_.ub() << std::endl; 
  }
  else
  {
    results_ofstream_ << std::endl;
  }

  // 5. bounded optimal pose
  Eigen::Affine2d& bounded_opt_pose = particle_.pose; 
  double bounded_opt_psi = atan2(bounded_opt_pose.matrix()(1,0),bounded_opt_pose.matrix()(0,0)); 
  results_ofstream_ << bounded_opt_pose.translation()(0) << " " << bounded_opt_pose.translation()(1) << " " << bounded_opt_psi << std::endl;

  // 6. unbounded optimal pose
  double unbounded_opt_psi = atan2(unbounded_opt_pose_.matrix()(1,0),unbounded_opt_pose_.matrix()(0,0));
  results_ofstream_ << unbounded_opt_pose_.translation()(0) << " " << unbounded_opt_pose_.translation()(1) << " " << unbounded_opt_psi << std::endl;

  // 7. number of associated facades
  results_ofstream_ << local_line_matches_.size() << std::endl; 

  // 8. ratio of of matched and unmatched points
  int num_mached_points = 0;
  for(std::pair<WallGroundLine*,LocalLine>& match : local_line_matches_)
  {
    num_mached_points += (int)match.second.considered_measures.size();
  }
  double ratio = (double)num_mached_points/(double)lidar_scan->pointcloud_->size();
  results_ofstream_ << ratio << std::endl; 

  // 9. Operation times
  results_ofstream_ << op_time_full << " " << op_time_iHT << " " << op_time_polygon << " " << op_time_association << std::endl;  
}