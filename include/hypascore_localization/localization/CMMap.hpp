#ifndef CMMAP_H
#define CMMAP_H

#include <iostream>
#include <fstream>
#include <sstream>
#include "Eigen/Dense"
#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "ibex.h"
#include <opencv2/imgproc/imgproc.hpp>

enum class States {NEW_BUILDING, ROOF_SURFACES, GROUND_SURFACES, WALL_SURFACES};

struct Surface
{
 std::vector<Eigen::Vector3d> points; 
};

struct Building
{
  std::vector<Surface> roofs;
  std::vector<Surface> grounds;
  std::vector<Surface> walls;
  std::vector<int> outer_wgl_idxs; 
};

struct WallGroundLine
{
  Eigen::Vector3d line_params;
  Eigen::Vector2d start_point;
  Eigen::Vector2d end_point; 
  size_t building_idx;
  size_t wall_idx;
  float r_color = 0;
  float g_color = 0;
  float b_color = 0;
};

class CMMap
{
  public:
  CMMap(std::string& city_model_file, Eigen::Vector3d& utm_world_offset); 
  std::vector<Building> buildings_; 
  std::vector<WallGroundLine> connected_wall_ground_lines_;
  std::vector<WallGroundLine> wall_ground_lines_;
  std::vector<WallGroundLine> wall_ground_lines_all_;
  Eigen::Vector3d utm_world_offset_;
  std::vector<cv::Point2f> cv_points_convex_hull_map_;
  void wall_ground_lines_radius_search(pcl::PointCloud<pcl::PointXYZ>& pc, std::vector<ibex::IntervalVector>& bc_laser, double& sampled_point_radius, double& max_line_distance, std::set<WallGroundLine*>& close_wgl_ptrs, std::map<WallGroundLine*,std::vector<size_t>>& line_measurement_matches);
  void wall_ground_lines_radius_search(std::vector<ibex::IntervalVector>& bc_world, std::set<WallGroundLine*>& close_wgl_ptrs, std::map<WallGroundLine*,std::vector<size_t>>& line_measurement_matches, double search_radius_factor);
  void wall_ground_lines_radius_search(ibex::IntervalVector& b_world, int& matches);
  void wall_ground_lines_radius_search(ibex::IntervalVector& b_world, double& wall_line_uncertainty, int& matches);
  void wall_ground_lines_radius_search(pcl::PointCloud<pcl::PointXY>& pc, double& max_line_distance, std::map<WallGroundLine*,std::vector<size_t>>& line_measurement_matches);
  void wall_ground_lines_radius_search(Eigen::Vector2d& mid, double& radius, std::set<WallGroundLine*>& close_wgl_ptrs);
  void connected_wall_ground_lines_radius_search(pcl::PointCloud<pcl::PointXY>& pc, double& max_line_distance, std::map<WallGroundLine*,std::vector<size_t>>& line_measurement_matches);
  void buildings_radius_search(ibex::IntervalVector& translation_box, std::vector<size_t>& buildings_idxs);

  private:
  States cur_state_;
  pcl::PointCloud<pcl::PointXY>::Ptr wall_ground_pc_ptr_;
  pcl::KdTreeFLANN<pcl::PointXY> wall_ground_kdtree_;
  std::map<int,int> pc_to_wgl_map_; // first index is the index in wall_ground_pc, second index is the index in wall_ground_lines_
  pcl::PointCloud<pcl::PointXY>::Ptr connected_wall_ground_pc_ptr_;
  pcl::KdTreeFLANN<pcl::PointXY> connected_wall_ground_kdtree_;
  std::map<int,int> connected_pc_to_wgl_map_; // first index is the index in wall_ground_pc, second index is the index in wall_ground_lines_
  void parsing_state_machine(std::ifstream& map_file);
  void compute_wall_ground_lines();
  void build_wall_ground_kdtree();
  void build_connected_wall_ground_kdtree();
  void compute_convex_hull(); 
};

#endif