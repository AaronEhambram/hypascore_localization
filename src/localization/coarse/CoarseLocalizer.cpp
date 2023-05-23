#include "hypascore_localization/localization/coarse/CoarseLocalizer.hpp"
#include "dirent.h"
#include <tf/tfMessage.h>
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/conversions.h>
#include <queue>
#include <chrono>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/calib3d.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include "hypascore_localization/contractors/CtcRangeBearingNoCross.hpp"
#include "g2o/core/robust_kernel_impl.h"
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include "g2o/types/slam2d/edge_se2.h"
#include "hypascore_localization/contractors/SepPolygon.hpp"
#include <visualization_msgs/MarkerArray.h>

CMGlobalLocalizer::CMGlobalLocalizer(ros::NodeHandle& nh, CMMap* map)
{
  std::string calibration_folder;
  bool compute_results;

  nh.getParam("/hypascore_localization_node/calibration_folder", calibration_folder);
  nh.getParam("/hypascore_localization_node/rotation_bins_amount", rotation_bins_amount_);
  nh.getParam("/hypascore_localization_node/min_position_box_diam_length", min_diam_length_);
  nh.getParam("/hypascore_localization_node/x_odom_translation_uncertainty_radius", x_odom_translation_uncertainty_radius_);
  nh.getParam("/hypascore_localization_node/y_odom_translation_uncertainty_radius", y_odom_translation_uncertainty_radius_);
  nh.getParam("/hypascore_localization_node/odom_rotation_uncertainty_radius", odom_rotation_uncertainty_radius_);
  nh.getParam("/hypascore_localization_node/results_file", results_file_);
  nh.getParam("/hypascore_localization_node/compute_results", compute_results);
  nh.getParam("/hypascore_localization_node/min_rotation_diam_length", min_rotation_diam_length_);
  nh.getParam("/hypascore_localization_node/wall_line_uncertainty", wall_line_uncertainty_);
  nh.getParam("/hypascore_localization_node/max_point_line_association_distance", max_point_line_association_distance_);

  std::string extrinsic_calib_file = calibration_folder+"extrinsics.xml";

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

  max_n_threads_ = std::thread::hardware_concurrency();
  cv::setNumThreads(max_n_threads_);

  if(compute_results)
  {
    std::cout << "Start saving to: " << results_file_ << std::endl; 
    results_ofstream_.open(results_file_.c_str());
  }

  // save City Model
  map_ = map;

  // create the publisher to RVIZ
  global_position_hull_publisher_.reset(new ros::Publisher);
  *global_position_hull_publisher_ = nh.advertise<visualization_msgs::Marker>("global_position_polygon", 1);
}

void CMGlobalLocalizer::set_initial_region(ibex::IntervalVector& translation, ibex::Interval& rotation)
{
  double bin_size = rotation.diam()/(double)rotation_bins_amount_;
  for(int i = 1; i <= rotation_bins_amount_; i++)
  {
    IntervalPose ipose(min_diam_length_,x_odom_translation_uncertainty_radius_,y_odom_translation_uncertainty_radius_,odom_rotation_uncertainty_radius_,max_point_line_association_distance_); 
    ipose.translation.push_back(translation);
    ipose.translation_hull = translation; 
    ipose.rotation = ibex::Interval(rotation.lb()+(double)(i-1)*bin_size,rotation.lb()+(double)i*bin_size); 
    ipose.initial_rotation = ibex::Interval(rotation.lb()+(double)(i-1)*bin_size,rotation.lb()+(double)i*bin_size); 
    feasible_poses_.push_back(ipose);
  }
  carve_out_buildings();
}

void CMGlobalLocalizer::carve_out_buildings()
{
  // delete out the regions that intersect with buildings!
  std::vector<std::vector<std::vector<ibex::IntervalVector>>> results; 
  results.resize(feasible_poses_.size());
  
  for(int ipose_counter = 0; ipose_counter < feasible_poses_.size(); ipose_counter++)
  {
    IntervalPose& ipose = feasible_poses_[ipose_counter];
    results[ipose_counter].resize(ipose.translation.size());
    carve_out_buildings_ipose_thread(&ipose, &(results[ipose_counter]));
  }

  #pragma omp parallel for
  for(int feasible_poses_counter = 0; feasible_poses_counter < results.size(); feasible_poses_counter++)
  {
    std::vector<std::vector<ibex::IntervalVector>>& results_feasible_pose = results[feasible_poses_counter];
    IntervalPose& ipose = feasible_poses_[feasible_poses_counter];
    std::vector<ibex::IntervalVector> new_translation; 
    for(size_t t_counter = 0; t_counter < results_feasible_pose.size(); ++t_counter)
    {
      std::vector<ibex::IntervalVector>& results_subpavings = results_feasible_pose[t_counter]; 
      for(ibex::IntervalVector& subpaving : results_subpavings)
      {
        new_translation.push_back(subpaving); 
      }
    }
    ipose.translation.clear(); 
    ipose.translation = new_translation; 
  }
}

void CMGlobalLocalizer::carve_out_buildings_ipose_thread(IntervalPose* ipose, std::vector<std::vector<ibex::IntervalVector>>* result)
{ 
  #pragma omp parallel for
  for(int subpaving_counter = 0; subpaving_counter < ipose->translation.size(); subpaving_counter++) 
  {
    ibex::IntervalVector& translation_interval = ipose->translation[subpaving_counter];
    carve_out_buildings_subpaving_thread(&translation_interval, &(ipose->rotation), &((*result)[subpaving_counter]));
    //carve_out_buildings_subpaving_sep_thread(&translation_interval, &(ipose->rotation), &((*result)[subpaving_counter]));
  }
}

void CMGlobalLocalizer::carve_out_buildings_subpaving_thread(ibex::IntervalVector* translation, ibex::Interval* rotation, std::vector<ibex::IntervalVector>* result)
{
  std::vector<ibex::IntervalVector> intermediate_result;
  intermediate_result.push_back(*translation); 
  // 2.a) only consider parts that are inside the convex hull of the map!
  std::queue<ibex::IntervalVector> Queue_map_hull; 
  for(ibex::IntervalVector& b : intermediate_result)
  {
    Queue_map_hull.push(b);
  }
  intermediate_result.clear(); 
  ibex::IntervalVector box_map_hull(2);
  while(!Queue_map_hull.empty())
  {
    box_map_hull = Queue_map_hull.front();
    Queue_map_hull.pop();
    polygon_test_action action;
    operation_selector_map_hull(box_map_hull,action);
    if(action == bisect)
    {
      if(box_map_hull.min_diam() > min_diam_length_)
      {
        std::pair<ibex::IntervalVector,ibex::IntervalVector> bisected = box_map_hull.bisect(box_map_hull.extr_diam_index(false));
        Queue_map_hull.push(bisected.first);
        Queue_map_hull.push(bisected.second);
      }
    }
    else if(action == include)
    {
      intermediate_result.push_back(box_map_hull);
    }
  }

  if(intermediate_result.size() == 0)
  {
    return;
  }

  // 2. iterate through each building and check if building inside or outside or both. If both, bisect. if inside, delete. if outside, just keep
  std::vector<size_t> buildings_idxs;
  map_->buildings_radius_search(*translation, buildings_idxs);
  for(int i = 0; i < buildings_idxs.size(); i++) 
  { 
    size_t idx = buildings_idxs[i]; 
    Building& b = map_->buildings_[idx];
    Surface& ground = b.grounds[0];
    std::queue<ibex::IntervalVector> Queue; 
    for(ibex::IntervalVector& box : intermediate_result)
    {
      Queue.push(box);
    }
    intermediate_result.clear(); 
    ibex::IntervalVector box(2);
    while(!Queue.empty())
    {
      box = Queue.front();
      Queue.pop();
      polygon_test_action action;
      double wall_uncertainty = 0; // for all poses the vehicle is never inside the building, although the wall are uncertain!
      operation_selector(box, ground,action, wall_uncertainty); 
      if(action == bisect)
      {
        if(box.min_diam() > min_diam_length_)
        {
          std::pair<ibex::IntervalVector,ibex::IntervalVector> bisected = box.bisect(box.extr_diam_index(false));
          Queue.push(bisected.first);
          Queue.push(bisected.second);
        }
      }
      else if(action == include)
      {
        intermediate_result.push_back(box);
      }
    }
  }
  result->resize(intermediate_result.size());
  *result = intermediate_result; 
}

void CMGlobalLocalizer::operation_selector(ibex::IntervalVector& box, Surface& polygon, polygon_test_action& action, double& wall_uncertainty)
{
  // compute box-hull to polygon, check if there can exist an intersection at all
  ibex::IntervalVector poylgon_box_hull(2);
  poylgon_box_hull.set_empty();
  for(Eigen::Vector3d& p_eig : polygon.points)
  {
    ibex::Vector p(2);
    p[0] = p_eig(0);
    p[1] = p_eig(1);
    poylgon_box_hull = poylgon_box_hull|p; 
  }
  if((box & poylgon_box_hull).is_empty())
  {
    action = include;
    return; 
  }

  // extract corner-points from box
  cv::Point2f pos_dl;
  pos_dl.x = (float)box[0].lb();
  pos_dl.y = (float)box[1].lb();
  cv::Point2f pos_tl;
  pos_tl.x = (float)box[0].lb();
  pos_tl.y = (float)box[1].ub();
  cv::Point2f pos_tr;
  pos_tr.x = (float)box[0].ub();
  pos_tr.y = (float)box[1].ub();
  cv::Point2f pos_dr;
  pos_dr.x = (float)box[0].ub();
  pos_dr.y = (float)box[1].lb();

  // Case 1: box fully contains polygon (each corner in the box) -> bisect
  bool in_box = true;
  for(Eigen::Vector3d& p_eig : polygon.points)
  {
    ibex::Vector p(2);
    p[0] = p_eig(0);
    p[1] = p_eig(1);
    if(!box.contains(p))
    {
      in_box = false; 
      break;
    }
  }
  if(in_box)
  {
    action = bisect; 
    return; 
  }

  // Case 2: box is inside polygon (each corner in the polygon) -> ignore
  std::vector<cv::Point2f> cv_points;
  for(Eigen::Vector3d& p : polygon.points)
  {
    cv::Point2f cv_p;
    cv_p.x = (float)p.x();
    cv_p.y = (float)p.y();
    cv_points.push_back(cv_p);
  }
  double poly_test_dl = -1;
  double poly_test_tl = -1;
  double poly_test_tr = -1;
  double poly_test_dr = -1;
  if(cv_points.size() > 2)
  {
    poly_test_dl = cv::pointPolygonTest(cv_points,pos_dl,false);
    poly_test_tl = cv::pointPolygonTest(cv_points,pos_tl,false);
    poly_test_tr = cv::pointPolygonTest(cv_points,pos_tr,false);
    poly_test_dr = cv::pointPolygonTest(cv_points,pos_dr,false);
  }
  if(poly_test_dl>=0 && poly_test_tl>=0 && poly_test_tr>=0 && poly_test_dr>=0) // fully included! -> ignore
  {
    action = ignore; 
    return; 
  }
  if(poly_test_dl>=0 || poly_test_tl>=0 || poly_test_tr>=0 || poly_test_dr>=0) // at least partially included! -> bisect
  {
    action = bisect; 

    /****contract the box!****/
    Eigen::Vector2d dl(box[0].lb(),box[1].lb()); // dl 
    Eigen::Vector2d tl(box[0].lb(),box[1].ub()); // tl
    Eigen::Vector2d tr(box[0].ub(),box[1].ub()); // tr
    Eigen::Vector2d dr(box[0].ub(),box[1].lb()); // dr
    // 1.) check how many corner points are inside
    std::vector<Eigen::Vector2d*> corners(4); 
    corners[0] = &dl;
    corners[1] = &tl;
    corners[2] = &tr;
    corners[3] = &dr;
    std::vector<int> inside_polygon_idxs; 
    std::vector<Eigen::Vector2d> points_for_hull;
    if(poly_test_dl > 0)
    {
      inside_polygon_idxs.push_back(0);   
    }
    else
    {
      points_for_hull.push_back(dl); 
    }
    if(poly_test_tl > 0)
    {
      inside_polygon_idxs.push_back(1);
    }
    else
    {
      points_for_hull.push_back(tl); 
    }
    if(poly_test_tr > 0)
    {
      inside_polygon_idxs.push_back(2);
    }
    else
    {
      points_for_hull.push_back(tr);
    }
    if(poly_test_dr > 0)
    {
      inside_polygon_idxs.push_back(3);
    }
    else
    {
      points_for_hull.push_back(dr);
    }
    // 2.) only perform contraction if more than 2 corners are in the polygon 
    if(inside_polygon_idxs.size() >= 2)
    { 
      for(int i = 1; i < polygon.points.size(); i++)
      {
        Eigen::Vector2d poly0(polygon.points[i-1](0),polygon.points[i-1](1));
        Eigen::Vector2d poly1(polygon.points[i](0),polygon.points[i](1));

        Eigen::Vector2d i1,i2,i3,i4;
        if(line_segments_intersect(*corners[0], *corners[1], poly0, poly1, i1))
        {
          points_for_hull.push_back(i1);
        }
        if(line_segments_intersect(*corners[1], *corners[2], poly0, poly1, i2))
        {
          points_for_hull.push_back(i2);
        }
        if(line_segments_intersect(*corners[2], *corners[3], poly0, poly1, i3))
        {
          points_for_hull.push_back(i3);
        }
        if(line_segments_intersect(*corners[3], *corners[0], poly0, poly1, i4))
        {
          points_for_hull.push_back(i4);
        }
      }
      ibex::IntervalVector new_box(2); 
      new_box.set_empty(); 
      for(Eigen::Vector2d& p_eig : points_for_hull)
      {
        ibex::IntervalVector p(2);
        p[0] = p_eig(0);
        p[1] = p_eig(1);
        p.inflate(wall_uncertainty);
        new_box = new_box | p;  
      }
      box = box & new_box; 
    }

    return;  
  }

  // Case 3: line segments intersect (overlap exists) -> bisect
  Eigen::Vector2d box0(box[0].lb(),box[1].lb()); 
  Eigen::Vector2d box1(box[0].lb(),box[1].ub());
  Eigen::Vector2d box2(box[0].ub(),box[1].ub());
  Eigen::Vector2d box3(box[0].ub(),box[1].lb());
  bool valid_intersection = false; 
  for(int i = 1; i < polygon.points.size(); i++)
  {
    Eigen::Vector2d poly0(polygon.points[i-1](0),polygon.points[i-1](1));
    Eigen::Vector2d poly1(polygon.points[i](0),polygon.points[i](1));

    bool box0_box1_line_intersect = line_segments_intersect(box0, box1, poly0, poly1);
    bool box1_box2_line_intersect = line_segments_intersect(box1, box2, poly0, poly1);
    bool box2_box3_line_intersect = line_segments_intersect(box2, box3, poly0, poly1);
    bool box3_box0_line_intersect = line_segments_intersect(box3, box0, poly0, poly1);
    if(box0_box1_line_intersect || box1_box2_line_intersect || box2_box3_line_intersect || box3_box0_line_intersect)
    {
      valid_intersection = true; 
      break;
    }
  }
  if(valid_intersection)
  {
    action = bisect; 
    return; 
  }

  // Case 4: box and polygon do not intersect and box is not inside polygon and box does not fully contain polygon (box outside polygon) -> include
  action = include; 
}

void CMGlobalLocalizer::operation_selector_map_hull(ibex::IntervalVector& box, polygon_test_action& action)
{
  // compute box-hull to polygon, check if there can exist an intersection at all
  ibex::IntervalVector poylgon_box_hull(2);
  poylgon_box_hull.set_empty();
  for(cv::Point2f& p_eig :  map_->cv_points_convex_hull_map_)
  {
    ibex::Vector p(2);
    p[0] = (double)p_eig.x;
    p[1] = (double)p_eig.y;
    poylgon_box_hull = poylgon_box_hull|p; 
  }
  if((box & poylgon_box_hull).is_empty())
  {
    action = ignore;
    return; 
  }

  // extract corner-points from box
  cv::Point2f pos_dl;
  pos_dl.x = (float)box[0].lb();
  pos_dl.y = (float)box[1].lb();
  cv::Point2f pos_tl;
  pos_tl.x = (float)box[0].lb();
  pos_tl.y = (float)box[1].ub();
  cv::Point2f pos_tr;
  pos_tr.x = (float)box[0].ub();
  pos_tr.y = (float)box[1].ub();
  cv::Point2f pos_dr;
  pos_dr.x = (float)box[0].ub();
  pos_dr.y = (float)box[1].lb();

  // Case 1: box fully contains polygon (each corner in the box) -> bisect
  bool in_box = true;
  for(cv::Point2f& p_eig : map_->cv_points_convex_hull_map_)
  {
    ibex::Vector p(2);
    p[0] = (double)p_eig.x;
    p[1] = (double)p_eig.y;
    if(!box.contains(p))
    {
      in_box = false; 
      break;
    }
  }
  if(in_box)
  {
    action = bisect; 
    return; 
  }

  // Case 2: box is inside map hull (each corner in the polygon) -> include
  double poly_test_dl = -1;
  double poly_test_tl = -1;
  double poly_test_tr = -1;
  double poly_test_dr = -1;
  if(map_->cv_points_convex_hull_map_.size() > 2)
  {
    poly_test_dl = cv::pointPolygonTest(map_->cv_points_convex_hull_map_,pos_dl,false);
    poly_test_tl = cv::pointPolygonTest(map_->cv_points_convex_hull_map_,pos_tl,false);
    poly_test_tr = cv::pointPolygonTest(map_->cv_points_convex_hull_map_,pos_tr,false);
    poly_test_dr = cv::pointPolygonTest(map_->cv_points_convex_hull_map_,pos_dr,false);
  }
  if(poly_test_dl>=0 && poly_test_tl>=0 && poly_test_tr>=0 && poly_test_dr>=0) // fully included! -> include
  {
    action = include; 
    return;  
  }
  if(poly_test_dl>=0 || poly_test_tl>=0 || poly_test_tr>=0 || poly_test_dr>=0) // at least partially included! -> bisect
  {
    action = bisect; 
    return;  
  }

  // Case 3: line segments intersect (overlap exists) -> bisect
  bool valid_intersection = false; 
  Eigen::Vector2d box0(box[0].lb(),box[1].lb()); 
  Eigen::Vector2d box1(box[0].lb(),box[1].ub());
  Eigen::Vector2d box2(box[0].ub(),box[1].ub());
  Eigen::Vector2d box3(box[0].ub(),box[1].lb());

  for(int i = 1; i < map_->cv_points_convex_hull_map_.size(); i++)
  {
    Eigen::Vector2d poly0((double)map_->cv_points_convex_hull_map_[i-1].x,(double)map_->cv_points_convex_hull_map_[i-1].y);
    Eigen::Vector2d poly1((double)map_->cv_points_convex_hull_map_[i].x,(double)map_->cv_points_convex_hull_map_[i].y);

    bool box0_box1_line_intersect = line_segments_intersect(box0, box1, poly0, poly1);
    bool box1_box2_line_intersect = line_segments_intersect(box1, box2, poly0, poly1);
    bool box2_box3_line_intersect = line_segments_intersect(box2, box3, poly0, poly1);
    bool box3_box0_line_intersect = line_segments_intersect(box3, box0, poly0, poly1);
    if(box0_box1_line_intersect || box1_box2_line_intersect || box2_box3_line_intersect || box3_box0_line_intersect)
    {
      valid_intersection = true; 
      break;
    }
  }
  if(valid_intersection)
  {
    action = bisect; 
    return; 
  }

  // Case 2: box and hull do not intersect and box is not inside hull (box outside polygon) -> ignore
  action = ignore;
}

void CMGlobalLocalizer::carve_out_buildings_subpaving_sep_thread(ibex::IntervalVector* translation, ibex::Interval* rotation, std::vector<ibex::IntervalVector>* result)
{
  std::vector<ibex::IntervalVector> intermediate_result;
  intermediate_result.push_back(*translation); 
  // 2.a) only consider parts that are inside the convex hull of the map!
  std::queue<ibex::IntervalVector> Queue_map_hull; 
  for(ibex::IntervalVector& b : intermediate_result)
  {
    Queue_map_hull.push(b);
  }
  intermediate_result.clear(); 
  ibex::IntervalVector box_map_hull(2);
  while(!Queue_map_hull.empty())
  {
    box_map_hull = Queue_map_hull.front();
    Queue_map_hull.pop();
    polygon_test_action action;
    operation_selector_map_hull(box_map_hull,action);
    if(action == bisect)
    {
      if(box_map_hull.min_diam() > min_diam_length_)
      {
        std::pair<ibex::IntervalVector,ibex::IntervalVector> bisected = box_map_hull.bisect(box_map_hull.extr_diam_index(false));
        Queue_map_hull.push(bisected.first);
        Queue_map_hull.push(bisected.second);
      }
    }
    else if(action == include)
    {
      intermediate_result.push_back(box_map_hull);
    }
  }

  if(intermediate_result.size() == 0)
  {
    return;
  }

  // 2. iterate through each building and check if building inside or outside or both. If both, bisect. if inside, delete. if outside, just keep
  std::vector<size_t> buildings_idxs;
  map_->buildings_radius_search(*translation, buildings_idxs);
  SepPolygon sep_poly; 
  std::vector<ibex::IntervalVector> polygon_vertices;
  for(int i = 0; i < buildings_idxs.size(); i++) 
  { 
    size_t idx = buildings_idxs[i]; 
    Building& b = map_->buildings_[idx];
    Surface& ground = b.grounds[0];
    polygon_vertices.clear();
    for(Eigen::Vector3d& p : ground.points)
    {
      ibex::IntervalVector a(2);
      a[0] = p(0); 
      a[1] = p(1);
      polygon_vertices.push_back(a);
    }
    std::queue<ibex::IntervalVector> Queue; 
    for(ibex::IntervalVector& box : intermediate_result)
    {
      Queue.push(box);
    }
    intermediate_result.clear(); 
    ibex::IntervalVector box(2);
    while(!Queue.empty())
    {
      box = Queue.front();
      Queue.pop();
      polygon_test_action action;
      double wall_uncertainty = 0; // for all poses the vehicle is never inside the building, although the wall are uncertain!
      //operation_selector(box, ground,action, wall_uncertainty); 
      std::vector<ibex::IntervalVector> boxes_in;
      std::vector<ibex::IntervalVector> boxes_out; 
      ibex::IntervalVector boundary_hull(2);
      sep_poly.separate(box,polygon_vertices,boxes_in,boxes_out,boundary_hull);
      for(ibex::IntervalVector& box_out : boxes_out)
      {
        intermediate_result.push_back(box_out);
      }
      if(!boundary_hull.is_empty() && boundary_hull.min_diam() > min_diam_length_)
      {
        std::pair<ibex::IntervalVector,ibex::IntervalVector> bisected = boundary_hull.bisect(boundary_hull.extr_diam_index(false));
        Queue.push(bisected.first);
        Queue.push(bisected.second);
      }
    }
  }
  result->resize(intermediate_result.size());
  *result = intermediate_result; 
}

bool CMGlobalLocalizer::line_segments_intersect(Eigen::Vector2d& p1, Eigen::Vector2d& p2, Eigen::Vector2d& p3, Eigen::Vector2d& p4)
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
    return true;
  }
  return false;
}

bool CMGlobalLocalizer::line_segments_intersect(Eigen::Vector2d& p1, Eigen::Vector2d& p2, Eigen::Vector2d& p3, Eigen::Vector2d& p4, Eigen::Vector2d& intersection)
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

void CMGlobalLocalizer::delete_observation_cross_regions()
{
  /***************
   *  None of the local measurements in bc_laser_ should should cross a wall!
   * ***************/
  // sort the iposes based on the number of subsets in the subpaving
  auto start_time = std::chrono::high_resolution_clock::now();
  auto now = std::chrono::high_resolution_clock::now();
  std::vector<std::pair<int,double>> index_subsetnumber_pairs; 
  for(int ipose_counter = 0; ipose_counter < feasible_poses_.size(); ipose_counter++) 
  {
    IntervalPose& ipose = feasible_poses_[ipose_counter];
    //index_subsetnumber_pairs.push_back(std::make_pair(ipose_counter,(double)ipose.translation.size()));
    index_subsetnumber_pairs.push_back(std::make_pair(ipose_counter,ipose.translation_area_));
  }
  std::sort(index_subsetnumber_pairs.begin(), index_subsetnumber_pairs.end(), compare_index_subsetnumber_pairs);

  // delete the blind regions. iposes with more subsets are computed first. do this only for 0.03s!
  std::vector<IntervalPose*> iposes; 
  std::vector<std::vector<std::vector<ibex::IntervalVector>>> results;
  for(int i = 0; i < index_subsetnumber_pairs.size(); i++)
  {
    int ipose_idx = index_subsetnumber_pairs[i].first;
    IntervalPose& ipose = feasible_poses_[ipose_idx]; 
    iposes.push_back(&ipose); 
    results.push_back(std::vector<std::vector<ibex::IntervalVector>>(ipose.translation.size()));
    ibex::Interval& psi = ipose.rotation;
    ibex::IntervalMatrix R(2,2);
    R[0][0] = cos(psi); R[0][1] = -sin(psi);
    R[1][0] = sin(psi); R[1][1] = cos(psi);
    #pragma omp parallel for
    for(int subset_counter = 0; subset_counter < ipose.translation.size(); subset_counter++)
    {
      ibex::IntervalVector& t = ipose.translation[subset_counter];
      carve_out_observation_cross_region_subpaving_thread(&t, &R, &results[results.size()-1][subset_counter]);     
    }
    now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time);
    if(duration.count()*std::pow(10,-6) > 0.05)
    {
      break; 
    }
  }

  // copy the data to all those iposes that were taken into account
  #pragma omp parallel for
  for(int feasible_poses_counter = 0; feasible_poses_counter < iposes.size(); feasible_poses_counter++)
  {
    std::vector<std::vector<ibex::IntervalVector>>& results_feasible_pose = results[feasible_poses_counter];
    IntervalPose& ipose = *iposes[feasible_poses_counter];
    //ipose.translation.clear();
    std::vector<ibex::IntervalVector> new_translation;
    for(size_t t_counter = 0; t_counter < results_feasible_pose.size(); ++t_counter)
    {
      std::vector<ibex::IntervalVector>& results_subpavings = results_feasible_pose[t_counter]; 
      for(ibex::IntervalVector& subpaving : results_subpavings)
      {
        //ipose.translation.push_back(subpaving);
        new_translation.push_back(subpaving); 
      }
    }
    ipose.translation.clear(); 
    ipose.translation = new_translation;
  }
}

void CMGlobalLocalizer::carve_out_observation_cross_region_subpaving_thread(ibex::IntervalVector* t, ibex::IntervalMatrix* R, std::vector<ibex::IntervalVector>* result)
{
  // 1.) iterate though each bc_laser_ box
  // a.) rotate all local measurements to the map frame using R!
  // b.) get the measurement with the highest distance
  std::vector<ibex::IntervalVector> bc_laser_rotated;
  double largest_dist = 0;  
  for(int bc_counter = 0; bc_counter < bc_laser_.size(); bc_counter++)
  {
    // 1.) rotate local boxes frame
    ibex::IntervalVector& b_laser = bc_laser_[bc_counter];
    ibex::Interval dist = b_laser.subvector(0,1).norm2();
    if(largest_dist < dist.ub())
    {
      largest_dist = dist.ub(); 
    }
    ibex::IntervalVector b_laser_rot(2); 
    b_laser_rot[0] = (*R)[0][0]*b_laser[0]+(*R)[0][1]*b_laser[1];
    b_laser_rot[1] = (*R)[1][0]*b_laser[0]+(*R)[1][1]*b_laser[1];
    bc_laser_rotated.push_back(b_laser_rot); 
  }

  // 2.) get the wall ground lines around the vehicle based on the measurement with the highest distance
  double radius = largest_dist + ibex::norm(t->diam())/2;
  Eigen::Vector2d translation_mid((*t)[0].mid(),(*t)[1].mid()); 
  std::set<WallGroundLine*> close_wgl_ptrs;
  map_->wall_ground_lines_radius_search(translation_mid, radius, close_wgl_ptrs);

  // 3.) iterate through each lines
  codac::CtcNoCross ctc_nocross; 
  ibex::IntervalVector p(*t); 
  for(WallGroundLine* wgl_ptr : close_wgl_ptrs)
  {
    ibex::IntervalVector a(2),b(2); 
    a[0] = wgl_ptr->start_point(0);
    a[1] = wgl_ptr->start_point(1);
    b[0] = wgl_ptr->end_point(0);
    b[1] = wgl_ptr->end_point(1);
    a.inflate(wall_line_uncertainty_);
    b.inflate(wall_line_uncertainty_); 
    for(int bc_counter = 0; bc_counter < bc_laser_rotated.size() && !p.is_empty(); bc_counter++)
    {
      ibex::IntervalVector& b_laser_rot = bc_laser_rotated[bc_counter];
      ctc_nocross.contract(p,b_laser_rot,a,b); 
    }
    if(p.is_empty())
    {
      break;
    }
  }
  if(!p.is_empty())
  {
    result->push_back(p); 
  }
}

bool CMGlobalLocalizer::compare_index_subsetnumber_pairs(std::pair<int,double> p1, std::pair<int,double> p2)
{
  return (p1.second > p2.second); 
}

bool CMGlobalLocalizer::compare_line_measurements_pairs(std::pair<WallGroundLine*,std::vector<size_t>> p1, std::pair<WallGroundLine*,std::vector<size_t>> p2)
{
  return (p1.second.size() > p2.second.size());
}

void CMGlobalLocalizer::shift_localization_region(Eigen::Affine2d& laser_bef_T_laser_cur, std::unique_ptr<LidarScan>& lidar_scan, std::unique_ptr<GpsMeasure>& gps_data)
{
  // copy the LiDAR-data for the specific frame
  bc_laser_ = *(lidar_scan->boxcloud_filtered_);
  laser_pc_ = lidar_scan->pointcloud_filtered_;

  // apply odoemtry updates
  #pragma omp parallel for
  for(int ipose_counter = 0; ipose_counter < feasible_poses_.size(); ipose_counter++)
  {
    IntervalPose& ipose = feasible_poses_[ipose_counter];
    ipose.add_odometry(&laser_bef_T_laser_cur);
  }

  // delete subsets outside the gps-measure uncertainty range if the gps is valid
  if(gps_data.get() != NULL)
  {
    // gps-measure is valid!
    #pragma omp parallel for
    for(int ipose_counter = 0; ipose_counter < feasible_poses_.size(); ipose_counter++)
    {
      IntervalPose& ipose = feasible_poses_[ipose_counter];
      Eigen::Vector3d center = gps_data->world_t_car_;
      double radius = gps_data->gps_uncertainty_; 
      ipose.delete_outside_circle(center,radius);
    }
  } 
  
  // delete the subsets inside the buildings
  carve_out_buildings();

  // delete infeasible poses
  int deleted_feasible_poses = 0; 
  //std::vector<int> deleted_idx;  
  for(int ipose_counter = 0; ipose_counter < feasible_poses_.size(); ipose_counter++)
  {
    IntervalPose& ipose = feasible_poses_[ipose_counter];
    if(ipose.translation.size() == 0)
    {
      feasible_poses_.erase(feasible_poses_.begin()+ipose_counter);
      ipose_counter--; 
      deleted_feasible_poses++;
      //deleted_idx.push_back(ipose_counter);
    }
  }

  // contract poses based on the no-cross assumption
  if(bc_laser_.size() > 0)
  {
    delete_observation_cross_regions();
  }

  // build the images in the iposes and perform reordering of necessary
  #pragma omp parallel for
  for(int ipose_counter = 0; ipose_counter < feasible_poses_.size(); ipose_counter++)
  {
    IntervalPose& ipose = feasible_poses_[ipose_counter];
    ipose.check_reordering();
    // update the particles and weight (no normalization!)
    ipose.particle_filter(map_, laser_pc_);
  }

  // 1.) Normalization of the particles 
  double sum_weight = 0;
  double max_weight = 0; 
  for(IntervalPose& ipose : feasible_poses_)
  {
    sum_weight += ipose.sum_weight_;
    if(ipose.max_weight_ > max_weight)
    {
      max_weight = ipose.max_weight_; 
    }
  }
  /*// Normalize weights of the particles
  double norm_factor = 1.0/sum_weight;
  max_weight = max_weight*norm_factor;  
  #pragma omp parallel for
  for(int ipose_counter = 0; ipose_counter < feasible_poses_.size(); ipose_counter++)
  {
    feasible_poses_[ipose_counter].particles_normalize(norm_factor);
  }*/

  // optimize particles with high weight
  double min_weight_for_optimize = 0.98*max_weight;

  // 2.) perform resampling, generate new particles close to good ones
  double min_weight_to_keep = max_weight*0.9; 
  #pragma omp parallel for
  for(int ipose_counter = 0; ipose_counter < feasible_poses_.size(); ipose_counter++)
  {
    feasible_poses_[ipose_counter].particles_low_weight_resampling(min_weight_to_keep);
  } 

  best_particles_.clear();
  all_particles_.clear(); 
  for(IntervalPose& ipose : feasible_poses_)
  {
    for(PoseParticle& particle : ipose.particles_)
    {
      if(particle.weight_ >= min_weight_for_optimize)
      {
        best_particles_.push_back(particle); 
      }
      all_particles_.push_back(particle);
    }
  }
  
  // save the hull of the result
  ibex::IntervalVector world_t_laser(2); world_t_laser.set_empty();
  ibex::Interval world_theta_laser; world_theta_laser.set_empty();
  for(IntervalPose& ipose : feasible_poses_)
  {
    if(!world_theta_laser.is_empty())
    {
      ibex::Interval diff1 = abs(world_theta_laser - ipose.rotation);
      ibex::Interval diff2 = abs(world_theta_laser - (ipose.rotation + 2*M_PI));
      ibex::Interval diff3 = abs(world_theta_laser - (ipose.rotation - 2*M_PI));
      if(diff1.ub() <= diff2.ub() && diff1.ub() <= diff3.ub())
      {
        world_theta_laser = world_theta_laser | ipose.rotation;
      }
      else if(diff2.ub() <= diff1.ub() && diff2.ub() <= diff3.ub())
      {
        world_theta_laser = world_theta_laser | (ipose.rotation + 2*M_PI);
      }
      else
      {
        world_theta_laser = world_theta_laser | (ipose.rotation - 2*M_PI);
      }
    }
    else
    {
      world_theta_laser = world_theta_laser | ipose.rotation; 
    }
    for(ibex::IntervalVector& transaltion_subset : ipose.translation)
    {
      world_t_laser = world_t_laser | transaltion_subset; 
    }
  }
  world_p_laser_[0] = world_t_laser[0]; 
  world_p_laser_[1] = world_t_laser[1]; 
  world_p_laser_[2] = world_theta_laser;

  // Check if rotation-splitting of the feasible poses is possible 
  if(deleted_feasible_poses > 0)
  {
    std::vector<std::pair<int,int>> index_subsetnumber_pairs; 
    for(int ipose_counter = 0; ipose_counter < feasible_poses_.size(); ipose_counter++) 
    {
      IntervalPose& ipose = feasible_poses_[ipose_counter];
      index_subsetnumber_pairs.push_back(std::make_pair(ipose_counter,ipose.translation.size()));
    }
    std::sort(index_subsetnumber_pairs.begin(), index_subsetnumber_pairs.end(), compare_index_subsetnumber_pairs);
    //int deleted_idx_counter = 0;
    for(int index_subsetnumber_pairs_counter = 0; index_subsetnumber_pairs_counter < index_subsetnumber_pairs.size() && feasible_poses_.size() < rotation_bins_amount_; index_subsetnumber_pairs_counter++)
    {
      IntervalPose& ipose = feasible_poses_[index_subsetnumber_pairs[index_subsetnumber_pairs_counter].first];
      if(ipose.rotation.diam() > min_rotation_diam_length_)
      {
        IntervalPose ipose_new(min_diam_length_,x_odom_translation_uncertainty_radius_,y_odom_translation_uncertainty_radius_,odom_rotation_uncertainty_radius_,max_point_line_association_distance_); 
        ipose_new.translation = ipose.translation;
        std::pair<ibex::Interval,ibex::Interval> bisection_pair = ipose.rotation.bisect();
        ipose.rotation = bisection_pair.first;
        ipose_new.rotation = bisection_pair.second; 
        ipose_new.initial_rotation = ipose.initial_rotation;
        ipose_new.particles_ = ipose.particles_; // copy the particles
        feasible_poses_.push_back(ipose_new);
        //feasible_poses_[deleted_idx[deleted_idx_counter]] = ipose_new; 
        //deleted_idx_counter++; 
      }
    }
  }
  visualize_results(); 
}

void CMGlobalLocalizer::get_localization_result(ibex::IntervalVector& map_p_laser)
{ 
  map_p_laser = world_p_laser_; 
}

// Anlaysis of the method
void CMGlobalLocalizer::save_timestamp_area_to_file(Eigen::Affine2d& world_T_laser_cur, double& t_stamp)
{
  /*******compute the area of the intervals*******/
  // 1.) compute the hull over all possible positions (for all orientations)
  ibex::IntervalVector hull(2); 
  hull.set_empty(); 
  for(IntervalPose& ipose : feasible_poses_)
  {
    for(ibex::IntervalVector& t : ipose.translation)
    {
      hull = hull | t; 
    }
  }

  // 2.) build a common image from all feasible_poses_
  ibex::Vector diams = hull.diam();
  ibex::Vector im_size = 1/min_diam_length_*diams;
  cv::Mat pos_im = cv::Mat::zeros(cv::Size((int)(std::ceil(im_size[0])), (int)(std::ceil(im_size[1]))), CV_64F); // rounding error!
  ibex::Vector pos_im_origin_trans(2);
  pos_im_origin_trans[0] = hull[0].lb();
  pos_im_origin_trans[1] = hull[1].ub();
  std::vector<cv::Point2f> area_points;
  for(IntervalPose& ipose : feasible_poses_)
  {
    //#pragma omp parallel for
    for(int i = 0; i < ipose.translation.size(); i++)
    {
      ibex::IntervalVector& t = ipose.translation[i];
      ibex::Vector local_diams = t.diam();
      ibex::Vector local_im_size = 1/min_diam_length_*local_diams;
      cv::Mat local_im = cv::Mat::ones(cv::Size((int)(std::ceil(local_im_size[0])), (int)(std::ceil(local_im_size[1]))), CV_64F); // rounding error!
      ibex::Vector local_im_origin_trans(2);
      local_im_origin_trans[0] = t[0].lb();
      local_im_origin_trans[1] = t[1].ub();
      ibex::Vector diff = local_im_origin_trans-pos_im_origin_trans;
      int start_col = (int) std::floor(1/min_diam_length_*diff[0]);
      int start_row = (int) std::floor(-1/min_diam_length_*diff[1]);

      if(start_col+local_im.cols > pos_im.cols)
      {
        local_im = cv::Mat::ones(cv::Size((int)(pos_im.cols-start_col),local_im.rows), CV_64F);
      }
      if(start_row+local_im.rows > pos_im.rows)
      {
        local_im = cv::Mat::ones(cv::Size(local_im.cols, (int)(pos_im.rows-start_row)), CV_64F);
      }

      cv::Mat dst_roi = pos_im(cv::Rect(start_col, start_row, local_im.cols, local_im.rows));
      local_im.copyTo(dst_roi);

      cv::Point2f tl,tr,bl,br;
      tl.x = (float)t[0].lb(); tl.y = (float)t[1].lb();
      tr.x = (float)t[0].ub(); tr.y = (float)t[1].lb();
      bl.x = (float)t[0].lb(); bl.y = (float)t[1].ub();
      br.x = (float)t[0].ub(); br.y = (float)t[1].ub();
      area_points.push_back(tl); 
      area_points.push_back(tr);
      area_points.push_back(bl);
      area_points.push_back(br);
    }
  }
  //cv::imshow("local_im",pos_im);
  // save the contour
  cv::RotatedRect rot_rect = cv::minAreaRect(area_points);

  // 3.) compute the integral image
  cv::Mat integral_im = cv::Mat::zeros(pos_im.size(), CV_64F);
  cv::integral(pos_im,integral_im);

  // 4.) get the last value of the integral image
  double pos_im_sum = integral_im.at<double>(integral_im.size().height-1,integral_im.size().width-1); 

  // 5.) multiply that with the resolution of the image
  double area = min_diam_length_*min_diam_length_*(pos_im_sum); 

  /*******compute the size of the orientation intervals*******/
  // 1.) add up the size of the intervals for the orientations
  ibex::Interval orientation_uncertainty_size; 
  orientation_uncertainty_size.set_empty(); 
  for(IntervalPose& ipose : feasible_poses_)
  {
    if(ipose.translation.size() > 0)
    {
      orientation_uncertainty_size = orientation_uncertainty_size | ipose.rotation; 
    }
  }

  /*******check if ground truth is within intervals*******/
  // 1.) get ground truth position and orientation
  double x_gt = world_T_laser_cur.translation()[0];
  double y_gt = world_T_laser_cur.translation()[1];
  double psi_gt = atan2(world_T_laser_cur.matrix()(1,0),world_T_laser_cur.matrix()(0,0));

  // 2.) compute the image-cell, in which the ground truth has to lie
  ibex::Vector local_gt_trans(2);
  local_gt_trans[0] = x_gt;
  local_gt_trans[1] = y_gt;
  ibex::Vector diff = local_gt_trans-pos_im_origin_trans;
  int gt_col = (int) std::floor(1/min_diam_length_*diff[0]);
  int gt_row = (int) std::floor(-1/min_diam_length_*diff[1]);

  // 3.) check if this cell has value 1. If not, ground truth not in feasible set
  double cell_value = pos_im.at<double>(gt_row,gt_col);

  // 4.) check if orientation is in one of the rotation intervals
  bool contains_psi_gt = false;
  for(IntervalPose& ipose : feasible_poses_)
  {
    if(ipose.translation.size() > 0 && ipose.rotation.contains(psi_gt))
    {
      contains_psi_gt = true; 
      break; 
    }
  }
  if(!contains_psi_gt)
  {
    for(IntervalPose& ipose : feasible_poses_)
    {
      if(ipose.translation.size() > 0 && (ipose.rotation.contains(psi_gt+2*M_PI) || ipose.rotation.contains(psi_gt-2*M_PI)))
      {
        contains_psi_gt = true; 
        break; 
      }
    }
  }

  // get closest, farthest and average partcile error to Ground Truth
  double closest_error_value = 1000000000;
  PoseParticle closest_particle, farthest_particle; 
  double farthest_error_value = 0; 
  for(int i = 0; i < all_particles_.size(); i++)
  {
    PoseParticle& particle = all_particles_[i];
    Eigen::Affine2d diff_pose = world_T_laser_cur.inverse()*particle.pose;
    double x_diff = diff_pose.translation()[0];
    double y_diff = diff_pose.translation()[1];
    double psi_diff = atan2(diff_pose.matrix()(1,0),diff_pose.matrix()(0,0));
    Eigen::Vector3d diff_vector(x_diff,y_diff,psi_diff);

    if(diff_vector.norm() > farthest_error_value)
    {
      farthest_error_value = diff_vector.norm();
      farthest_particle = particle;
    }
    if(diff_vector.norm() < closest_error_value)
    {
      closest_error_value = diff_vector.norm();
      closest_particle = particle;
    }
  }

  // average particle distance
  double average_transl_dist = 0;
  double average_orient_dist = 0;
  for(int i = 0; i < all_particles_.size(); i++)
  {
    PoseParticle& particle = all_particles_[i];
    Eigen::Affine2d diff_pose = world_T_laser_cur.inverse()*particle.pose;
    double x_diff = diff_pose.translation()[0];
    double y_diff = diff_pose.translation()[1];
    double psi_diff = atan2(diff_pose.matrix()(1,0),diff_pose.matrix()(0,0));
    average_transl_dist += diff_pose.translation().norm();
    average_orient_dist += abs(psi_diff);
  }
  bool method_failed = false;
  if(all_particles_.size() > 0)
  {
    average_transl_dist = average_transl_dist/(double)all_particles_.size();
    average_orient_dist = average_orient_dist/(double)all_particles_.size();
  }
  else
  {
    method_failed = true;
  }
  

  results_ofstream_ << std::setprecision(16) 
  << t_stamp << " "
  << x_gt << " "
  << y_gt << " "
  << psi_gt << " "
  << area << " "
  << rot_rect.size.height << " "
  << rot_rect.size.width << " "
  << orientation_uncertainty_size.diam() << " "
  << cell_value << " "
  << contains_psi_gt << " "
  << farthest_particle.pose.translation().x() << " "
  << farthest_particle.pose.translation().y() << " "
  << atan2(farthest_particle.pose.matrix()(1,0),farthest_particle.pose.matrix()(0,0)) << " "
  << closest_particle.pose.translation().x() << " "
  << closest_particle.pose.translation().y() << " "
  << atan2(closest_particle.pose.matrix()(1,0),closest_particle.pose.matrix()(0,0)) << " "
  << average_transl_dist << " "
  << average_orient_dist << " "
  << method_failed
  << std::endl;
}

// Visuaization
void CMGlobalLocalizer::visualize_results()
{
  // draw position polygon
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "global_position_hull_polygon";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = 0.23;
  marker.color.a = 0.5; // Don't forget to set the alpha!
  marker.color.r = 0.6;
  marker.color.g = 0.6;
  marker.color.b = 0.0;
  ibex::Interval& x = world_p_laser_[0];
  ibex::Interval& y = world_p_laser_[1];
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
  global_position_hull_publisher_->publish(marker);
}