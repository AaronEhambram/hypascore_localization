#ifndef INTERVALPOSE_H_
#define INTERVALPOSE_H_
#include <iostream>
#include <fstream>
#include <sstream>
#include "Eigen/Dense"
#include <thread>
#include "ibex.h"
#include <mutex>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/calib3d.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include "hypascore_localization/localization/CMMap.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <random>
#include "hypascore_localization/contractors/CtcRange.hpp"

class PoseParticle
{
  public:
  Eigen::Affine2d pose;
  double weight_ = 0;
  double weight_sum_ = 0;
  int age = 0; 
  std::map<WallGroundLine*,std::vector<size_t>> line_measurement_matches;
};

class IntervalPose
{
  public:
  IntervalPose(double min_diam_length, double& x_odom_translation_uncertainty_radius, double& y_odom_translation_uncertainty_radius, double& odom_rotation_uncertainty_radius, double& max_point_line_association_distance);
  std::vector<ibex::IntervalVector> translation;
  ibex::Interval rotation; 
  void add_odometry(Eigen::Affine2d* laser_bef_T_laser_cur);
  void delete_outside_circle(Eigen::Vector3d& center, double& radius);
  void particle_filter(CMMap* map, pcl::PointCloud<pcl::PointXY>::Ptr& laser_pc);
  void particles_normalize(double& norm_factor);
  void particles_low_weight_resampling(double& min_weight);
  void check_reordering();
  cv::Mat pos_im;
  cv::Mat integral_im_;
  ibex::Interval initial_rotation;
  ibex::IntervalVector translation_hull;
  double translation_area_;
  std::vector<PoseParticle> particles_;
  double sum_weight_; 
  double max_weight_;
  double min_diam_length_;

  private:
  double x_odom_translation_uncertainty_radius_;
  double y_odom_translation_uncertainty_radius_;
  double odom_rotation_uncertainty_radius_;
  int max_particles_ = 20; 
  double max_line_distance_ = 1.5;
  double max_weight_occurance_ = 5; 
  double max_translation_diff_resampling = 1.5; 
  cv::Mat integral_overlap_im_;
  cv::Mat overlap_pos_im; 
  void create_images(); 
  void reorder_subpavings(); 

  void particles_delete_outside_feasible_set();
};

#endif