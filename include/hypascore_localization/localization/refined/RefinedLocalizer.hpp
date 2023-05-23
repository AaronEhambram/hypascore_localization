#ifndef REFINEDLOCALIZER_H_
#define REFINEDLOCALIZER_H_
#include "hypascore_localization/localization/coarse/IntervalPose.hpp"
#include "hypascore_localization/localization/LidarData.hpp"

struct LocalLine
{
  ibex::Interval angle;
  ibex::Interval d;
  std::set<size_t> considered_measures; 
  size_t start_measure_idx;
  size_t end_measure_idx; 
};

struct PositionBoundary
{
  ibex::Interval angle;
  ibex::Interval d;
  double r_color;
  double g_color;
  double b_color; 
};

class CMTracker
{
  public:
  CMTracker(ros::NodeHandle& nh, CMMap* map);
  CMMap* map_; 
  Eigen::Affine3d car_T_left_cam_, car_T_right_cam_, car_T_laser_, car_T_imu_;
  void track(std::unique_ptr<LidarScan>& lidar_scan, std::vector<IntervalPose>& gl_feasible_poses, Eigen::Affine2d& laser_bef_T_laser_cur, ibex::IntervalVector& laser_bef_p_laser_cur, double& time); 
  bool tracking_initialized_ = false;
  double processed_timestamp_;
  // States that need to be estimated
  // best tracked particle:
  PoseParticle particle_; 
  std::vector<std::pair<WallGroundLine*,LocalLine>> local_line_matches_; 
  // the interval representation:
  ibex::IntervalVector translation_hull_;
  std::vector<Eigen::Vector2d> polygone_;
  ibex::Interval rotation_;
  // reliability based on the age of the particle
  double tracking_reliablility_ = 0.0; 
  bool full_reset_ = false;
  bool internal_switch_ = false; 
  ibex::Interval facade_observation_sin_angle_hull_;
  bool contracted_feasible_set_ = false;
  PoseParticle max_weight_gl_particle_;

  private:
  int min_age_to_initiate_track_ = 200;
  double min_dist_to_associate_point_to_facade_ = 0.5; 
  int min_number_points_on_facade_ = 100;
  double angle_quantization_ = M_PI/180.0*0.5;
  double d_quantization_ = 0.01; 
  double min_facade_length_ = 3.0; 
  double wall_position_uncertainty_ = 0.7;
  double wall_orientation_uncertainty_ = 3.0*M_PI/180.0;
  double max_translation_difference_ = 3.0; 
  double max_rotation_difference_ = 10*M_PI/180.0;
  double gl_particle_better_factor_ = 1.5; 
  int min_particle_age_to_overwrite_gl_ = 300;
  double min_sin_angle_hull_diam_ = 0.9; 
  int min_age_to_switch_track_ = 30; 
  double pca_eigen_line_length_ = 2.0; 
  PoseParticle& get_max_weight_particle(std::vector<IntervalPose>& gl_feasible_poses, bool& correct_result); 
  void initialize_track(std::unique_ptr<LidarScan>& lidar_scan, std::vector<IntervalPose>& gl_feasible_poses, PoseParticle& gl_particle_max_weight);
  void update_track(std::unique_ptr<LidarScan>& lidar_scan, std::vector<IntervalPose>& gl_feasible_poses, Eigen::Affine2d& laser_bef_T_laser_cur, ibex::IntervalVector& laser_bef_p_laser_cur);
  void associate_lidar_to_map(std::unique_ptr<LidarScan>& lidar_scan);  
  void extract_line_parameters(double& angle_uncertainty, std::unique_ptr<LidarScan>& lidar_scan);
  void polygon_pose_estimation(std::vector<PositionBoundary>& further_position_boundaries, std::vector<ibex::Interval>& further_orientations);
  void line_intersection(Eigen::Vector2d& n1, double c1, Eigen::Vector2d& n2, double c2, Eigen::Vector2d& x);
  bool line_segments_intersect(Eigen::Vector2d& p1, Eigen::Vector2d& p2, Eigen::Vector2d& p3, Eigen::Vector2d& p4, Eigen::Vector2d& intersection); 
  void update_estimates(Eigen::Affine2d& laser_bef_T_laser_cur, ibex::IntervalVector& laser_bef_p_laser_cur);
  void improve_particle();
  void compute_optimal_particle(Eigen::Vector2d& optimal_position, double& optimal_orientation);
  void compute_particle_score(PoseParticle& particle, std::unique_ptr<LidarScan>& lidar_scan); 
  void particle_correction(std::unique_ptr<LidarScan>& lidar_scan, std::vector<IntervalPose>& gl_feasible_poses, PoseParticle& gl_particle_max_weight, Eigen::Affine2d& laser_bef_T_laser_cur);
  void particle_correction_V2(std::unique_ptr<LidarScan>& lidar_scan, std::vector<IntervalPose>& gl_feasible_poses, PoseParticle& gl_particle_max_weight, Eigen::Affine2d& laser_bef_T_laser_cur); 
  ros::NodeHandle* nh_ptr; 
  std::shared_ptr<ros::Publisher> associated_points_publisher_;
  std::shared_ptr<ros::Publisher> evaluation_lidar_points_publisher_;
  std::shared_ptr<ros::Publisher> matched_line_publisher_;
  std::shared_ptr<ros::Publisher> extracted_line_publisher_;
  std::shared_ptr<ros::Publisher> position_polygone_publisher_ptr;
  void visualize_associated_points(std::unique_ptr<LidarScan>& lidar_scan); 
  // Evaluation
  void save_to_file(std::unique_ptr<LidarScan>& lidar_scan);
  bool dump_tracker_results_ = false; 
  std::string file_tracker_results_ = ""; 
  std::ofstream results_ofstream_;
  Eigen::Affine2d unbounded_opt_pose_;
  double op_time_full, op_time_iHT, op_time_polygon, op_time_association; 
};

#endif