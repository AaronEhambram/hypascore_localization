#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <assert.h>
#include <algorithm>
#include <dirent.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tfMessage.h>
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <eigen3/Eigen/Eigenvalues>
#include "hypascore_localization/PoseEstimationSystem.hpp"
#include <sensor_msgs/point_cloud2_iterator.h>

// Topics in rosbag
std::vector<std::string> rosbags_list;
std::string left_cam_image_topic;
std::string right_cam_image_topic;
std::string lidar_topic;
std::string imu_topic;
std::string gt_pose_topic;
std::string tf_topic;
std::string file_gt_poses;

// Processing parameters
double start_stamp;
double stop_stamp;
double utm_world_offset_z, utm_world_offset_x, utm_world_offset_y;
Eigen::Vector3d utm_world_offset;
std::string calibration_folder;
std::string city_model_file;
bool use_real_time_speed; 
bool first_call = true; 
double gps_data_frequency;

// ROS publishers
std::shared_ptr<ros::Publisher> left_im_publisher_ptr;
std::shared_ptr<ros::Publisher> right_im_publisher_ptr;
std::shared_ptr<ros::Publisher> lidar_publisher_ptr;
std::shared_ptr<ros::Publisher> gt_pose_publisher_ptr;
std::shared_ptr<ros::Publisher> imu_publisher_ptr;
std::shared_ptr<ros::Publisher> tf_publisher_ptr;
std::shared_ptr<ros::Publisher> map_publisher_ptr;
std::shared_ptr<ros::Publisher> ground_map_publisher_ptr;
std::shared_ptr<ros::Publisher> feature_publisher_ptr;
std::shared_ptr<ros::Publisher> interval_odom_publisher_ptr;
std::shared_ptr<ros::Publisher> pointlcoud_z_clipped_publisher_ptr;
std::shared_ptr<ros::Publisher> pointlcoud_line_filtered_publisher_ptr;
std::shared_ptr<ros::Publisher> boxcloud_mid_line_filtered_publisher_ptr;

// Processing Objects
std::unique_ptr<PoseEstimationSystem> pes_;

// initializing parameters
double init_x_translation_uncertainty_radius;
double init_y_translation_uncertainty_radius;
double init_rotation_uncertainty_radius;

// output
std::ofstream gt_poses_ofstream;
bool dump_gt_poses = false; 

// publisher functions
void map_publisher()
{
  Eigen::Vector3d utm_to_world_offset(0,0,57-utm_world_offset_z);
  while(ros::ok() && pes_!=NULL && pes_->map_!=NULL)
  {
    visualization_msgs::MarkerArray marker_arr;
    int id = 0; 
    for(Building& b : pes_->map_->buildings_)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time();
      marker.ns = "building";
      marker.id = id;
      marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
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
      marker.scale.x = 1;
      marker.scale.y = 1;
      marker.scale.z = 1;
      marker.color.a = 1; // Don't forget to set the alpha!
      marker.color.r = 1;
      marker.color.g = 1;
      marker.color.b = 1;
      // Walls:
      for(Surface& s : b.walls)
      {
        Eigen::Vector3d average_point;
        average_point.setZero();
        double counter = 0; 
        for(Eigen::Vector3d& p : s.points)
        {
          average_point = average_point + p;
          counter += 1;  
        }
        average_point = average_point/counter; 
        average_point = average_point - utm_to_world_offset;
        geometry_msgs::Point pa;
        pa.x = average_point(0);
        pa.y = average_point(1);
        pa.z = average_point(2);
        std_msgs::ColorRGBA p_color; 
        p_color.a = 0.3; 
        p_color.r = 1;
        p_color.g = 1;
        p_color.b = 1;
        for(int p_idx = 1; p_idx < s.points.size(); p_idx++)
        {
          marker.points.push_back(pa);
          marker.colors.push_back(p_color);
          Eigen::Vector3d p2_eig = s.points[p_idx] - utm_to_world_offset;
          geometry_msgs::Point p2;
          p2.x = p2_eig(0);
          p2.y = p2_eig(1);
          p2.z = p2_eig(2);
          marker.points.push_back(p2);
          marker.colors.push_back(p_color);
          Eigen::Vector3d p1_eig = s.points[p_idx-1] - utm_to_world_offset;
          geometry_msgs::Point p1;
          p1.x = p1_eig(0);
          p1.y = p1_eig(1);
          p1.z = p1_eig(2);
          marker.points.push_back(p1);
          marker.colors.push_back(p_color);
        }
      }
      // roof:
      for(Surface& s : b.roofs)
      {
        Eigen::Vector3d average_point;
        average_point.setZero();
        double counter = 0; 
        for(Eigen::Vector3d& p : s.points)
        {
          average_point = average_point + p;
          counter += 1;  
        }
        average_point = average_point/counter; 
        average_point = average_point - utm_to_world_offset;
        geometry_msgs::Point pa;
        pa.x = average_point(0);
        pa.y = average_point(1);
        pa.z = average_point(2);
        std_msgs::ColorRGBA p_color; 
        p_color.a = 0.3; 
        p_color.r = 1;
        p_color.g = 0;
        p_color.b = 0;
        for(int p_idx = 1; p_idx < s.points.size(); p_idx++)
        {
          marker.points.push_back(pa);
          marker.colors.push_back(p_color);
          Eigen::Vector3d p2_eig = s.points[p_idx] - utm_to_world_offset;
          geometry_msgs::Point p2;
          p2.x = p2_eig(0);
          p2.y = p2_eig(1);
          p2.z = p2_eig(2);
          marker.points.push_back(p2);
          marker.colors.push_back(p_color);
          Eigen::Vector3d p1_eig = s.points[p_idx-1] - utm_to_world_offset;
          geometry_msgs::Point p1;
          p1.x = p1_eig(0);
          p1.y = p1_eig(1);
          p1.z = p1_eig(2);
          marker.points.push_back(p1);
          marker.colors.push_back(p_color);
        }
      }
      if(marker.points.size() > 0)
      {
        marker_arr.markers.push_back(marker);
      }
      id++; 
    }
    map_publisher_ptr->publish(marker_arr);

    // publish ground_map
    visualization_msgs::Marker marker; 
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "ground_map";
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
    marker.scale.x = 0.5;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1; // Don't forget to set the alpha!
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 0;
    std_msgs::ColorRGBA p_color; 
    p_color.a = 0.4; 
    p_color.r = 0;
    p_color.g = 0;
    p_color.b = 0;
    std_msgs::ColorRGBA n_color;
    n_color.a = 0.4; 
    n_color.r = 1;
    n_color.g = 0;
    n_color.b = 0;
    for(WallGroundLine wgl : pes_->map_->connected_wall_ground_lines_)
    {
      geometry_msgs::Point p2;
      p2.x = wgl.start_point(0)-utm_to_world_offset(0);
      p2.y = wgl.start_point(1)-utm_to_world_offset(1);
      p2.z = -2;
      marker.points.push_back(p2);
      marker.colors.push_back(p_color);

      geometry_msgs::Point p1;
      p1.x = wgl.end_point(0)-utm_to_world_offset(0);
      p1.y = wgl.end_point(1)-utm_to_world_offset(1);
      p1.z = -2;
      marker.points.push_back(p1);
      marker.colors.push_back(p_color);

      /*geometry_msgs::Point pn1;
      pn1.x = (wgl.end_point(0)+wgl.start_point(0))/2-utm_to_world_offset(0);
      pn1.y = (wgl.end_point(1)+wgl.start_point(1))/2-utm_to_world_offset(1);
      pn1.z = -2;
      marker.points.push_back(pn1);
      marker.colors.push_back(n_color);

      geometry_msgs::Point pn2;
      pn2.x = (wgl.end_point(0)+wgl.start_point(0))/2+wgl.line_params(0)-utm_to_world_offset(0);
      pn2.y = (wgl.end_point(1)+wgl.start_point(1))/2+wgl.line_params(1)-utm_to_world_offset(1);
      pn2.z = -2;
      marker.points.push_back(pn2);
      marker.colors.push_back(n_color);*/
    }
    ground_map_publisher_ptr->publish(marker);

    /*// publish position polygons
    if(position_polygons.markers.size() > 0)
    {
      polygon_publisher_ptr->publish(position_polygons);
    }*/

    ros::Duration(1).sleep(); 
  }
}

void visual_feature_publisher()
{
  while(ros::ok() && pes_!=NULL)
  {
    if(pes_->vo_->graph_frames_.size() >= 1)
    {
      visualization_msgs::MarkerArray marker_arr;
      int id = 0; 
      Graph::Frame* cur_frame = pes_->vo_->graph_frames_[pes_->vo_->graph_frames_.size()-1];
      for(Graph::Landmark* l : cur_frame->seen_landmarks_)
      {
        Graph::Observation* obsv = l->obsv_map_[cur_frame];
        visualization_msgs::Marker marker;
        marker.header.frame_id = "camera_left";
        marker.header.stamp = ros::Time();
        marker.lifetime = ros::Duration(0.11);
        marker.ns = "visual_features";
        marker.id = id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.pose.position.x = obsv->lcam_point_(0);
        marker.pose.position.y = obsv->lcam_point_(1);
        marker.pose.position.z = obsv->lcam_point_(2);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 0.85; // Don't forget to set the alpha!
        marker.color.r = l->color_(2)/255;
        marker.color.g = l->color_(1)/255;
        marker.color.b = l->color_(0)/255;
        id++;
        marker_arr.markers.push_back(marker);

        if(!obsv->lcam_box_.is_empty() && !obsv->lcam_box_.is_unbounded())
        {
          marker.ns = "visual_features_box";
          marker.type = visualization_msgs::Marker::CUBE;
          marker.pose.position.x = obsv->lcam_box_[0].mid();
          marker.pose.position.y = obsv->lcam_box_[1].mid();
          marker.pose.position.z = obsv->lcam_box_[2].mid();
          marker.scale.x = obsv->lcam_box_[0].diam();
          marker.scale.y = obsv->lcam_box_[1].diam();
          marker.scale.z = obsv->lcam_box_[2].diam();
          marker.color.a = 0.5;
          marker_arr.markers.push_back(marker); 
        }
      }
      feature_publisher_ptr->publish(marker_arr);
    }
    ros::Duration(0.1).sleep(); 
  }
}

void visual_odom()
{
  cv::Mat im;
  while(ros::ok() && pes_!=NULL)
  {
    visualization_msgs::MarkerArray arr; 
    if(pes_->vo_->graph_frames_.size() >= 1)
    {
      ibex::IntervalVector origin_p_cur(3);
      origin_p_cur[0] = 0; 
      origin_p_cur[1] = 0;
      origin_p_cur[2] = 0;
      ibex::IntervalVector origin_p_kf(3);
      origin_p_kf[0] = 0; 
      origin_p_kf[1] = 0;
      origin_p_kf[2] = 0;
      for(int i = 1; i < pes_->vo_->graph_frames_.size(); i++)
      {
        Graph::Frame* f = pes_->vo_->graph_frames_[i];
        if(f->computed_interval_odometry_)
        {
          origin_p_cur[0] = origin_p_cur[0] + cos(origin_p_cur[2])*f->laser_prev_p_laser_cur[0] - sin(origin_p_cur[2])*f->laser_prev_p_laser_cur[1];
          origin_p_cur[1] = origin_p_cur[1] + sin(origin_p_cur[2])*f->laser_prev_p_laser_cur[0] + cos(origin_p_cur[2])*f->laser_prev_p_laser_cur[1];
          origin_p_cur[2] = origin_p_cur[2] + f->laser_prev_p_laser_cur[5];

          // relative to previous frame
          Eigen::Affine3d laser_prev_T_laser_cur = pes_->vo_->car_T_laser_.inverse() * f->car_prev_T_car_cur_ * pes_->vo_->car_T_laser_;
          visualization_msgs::Marker marker_rel_prev_ls;
          marker_rel_prev_ls.header.frame_id = "laser";
          marker_rel_prev_ls.header.stamp = ros::Time(pes_->vo_->graph_frames_[i-1]->timestamp_);
          marker_rel_prev_ls.ns = "ls-rel to prev odom";
          marker_rel_prev_ls.id = i;
          marker_rel_prev_ls.type = visualization_msgs::Marker::CUBE;
          marker_rel_prev_ls.pose.position.x = laser_prev_T_laser_cur.translation()(0);
          marker_rel_prev_ls.pose.position.y = laser_prev_T_laser_cur.translation()(1);
          marker_rel_prev_ls.pose.position.z = 0;
          marker_rel_prev_ls.pose.orientation.x = 0.0;
          marker_rel_prev_ls.pose.orientation.y = 0.0;
          marker_rel_prev_ls.pose.orientation.z = 0.0;
          marker_rel_prev_ls.pose.orientation.w = 1.0;
          marker_rel_prev_ls.scale.x = 0.01;
          marker_rel_prev_ls.scale.y = 0.01;
          marker_rel_prev_ls.scale.z = 0.01;
          marker_rel_prev_ls.color.a = 0.5;
          marker_rel_prev_ls.color.r = 0.9;
          marker_rel_prev_ls.color.g = 0.9;
          marker_rel_prev_ls.color.b = 0.0;
          arr.markers.push_back(marker_rel_prev_ls);

          // relative to previous frame
          visualization_msgs::Marker marker_rel_prev;
          marker_rel_prev.header.frame_id = "laser";
          marker_rel_prev.header.stamp = ros::Time(pes_->vo_->graph_frames_[i-1]->timestamp_);
          marker_rel_prev.ns = "rel to prev odom";
          marker_rel_prev.id = i;
          marker_rel_prev.type = visualization_msgs::Marker::CUBE;
          marker_rel_prev.pose.position.x = f->laser_prev_p_laser_cur[0].mid();
          marker_rel_prev.pose.position.y = f->laser_prev_p_laser_cur[1].mid();
          marker_rel_prev.pose.position.z = 0;
          marker_rel_prev.pose.orientation.x = 0.0;
          marker_rel_prev.pose.orientation.y = 0.0;
          marker_rel_prev.pose.orientation.z = 0.0;
          marker_rel_prev.pose.orientation.w = 1.0;
          marker_rel_prev.scale.x = f->laser_prev_p_laser_cur[0].diam();
          marker_rel_prev.scale.y = f->laser_prev_p_laser_cur[1].diam();
          marker_rel_prev.scale.z = 0.01;
          marker_rel_prev.color.a = 0.5;
          marker_rel_prev.color.r = 0.9;
          marker_rel_prev.color.g = 0.0;
          marker_rel_prev.color.b = 0.6;
          arr.markers.push_back(marker_rel_prev);

          // accumulated pose
          visualization_msgs::Marker marker;
          marker.header.frame_id = "laser";
          marker.header.stamp = ros::Time(pes_->vo_->graph_frames_[0]->timestamp_);
          marker.ns = "accumulated odom";
          marker.id = i;
          marker.type = visualization_msgs::Marker::CUBE;
          marker.pose.position.x = origin_p_cur[0].mid();
          marker.pose.position.y = origin_p_cur[1].mid();
          marker.pose.position.z = 0;
          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;
          marker.scale.x = origin_p_cur[0].diam();
          marker.scale.y = origin_p_cur[1].diam();
          marker.scale.z = 0.01;
          marker.color.a = 0.5;
          marker.color.r = 0.0;
          marker.color.g = 0.3;
          marker.color.b = 0.7;
          arr.markers.push_back(marker);

          // ground truth
          visualization_msgs::Marker marker_gt;
          marker_gt.header.frame_id = "laser";
          marker_gt.header.stamp = ros::Time(f->timestamp_);
          marker_gt.ns = "Ground truth pose";
          marker_gt.id = i;
          marker_gt.type = visualization_msgs::Marker::CUBE;
          marker_gt.pose.position.x = 0;
          marker_gt.pose.position.y = 0;
          marker_gt.pose.position.z = 0;
          marker_gt.pose.orientation.x = 0.0;
          marker_gt.pose.orientation.y = 0.0;
          marker_gt.pose.orientation.z = 0.0;
          marker_gt.pose.orientation.w = 1.0;
          marker_gt.scale.x = 0.01;
          marker_gt.scale.y = 0.01;
          marker_gt.scale.z = 0.01;
          marker_gt.color.a = 0.5;
          marker_gt.color.r = 0.0;
          marker_gt.color.g = 1.0;
          marker_gt.color.b = 0.0;
          arr.markers.push_back(marker_gt);
        }
        else
        {
          break; 
        }
      }
      //std::cout << "---" << std::endl;
      // publish the poses in the laserframe of the very first frame
      interval_odom_publisher_ptr->publish(arr);

    }
    ros::Duration(0.1).sleep();
  }
}

void pc_publisher()
{
  // get the last entry of the frames
  Graph::Frame* f = pes_->vo_->graph_frames_[pes_->vo_->graph_frames_.size()-1];
  double time = f->timestamp_; 

  { // keep the namespace in case we need to copy this part for another cloud
  // get the pointcloud that we want to visulize 
  pcl::PointCloud<pcl::PointXYZ>::Ptr& pc = pes_->lidar_data_->time_scan_map_[time]->pointcloud_;
  //declare message and sizes
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "laser";
  cloud.header.stamp = ros::Time(time);
  cloud.width  = pc->size();
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false; // there may be invalid points

  //for fields setup
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1,"xyz");
  modifier.resize(cloud.width);

  //iterators
  sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
  
  for (int y=0; y<1; y++)
  {
    for (int x=0; x<pc->size(); x++)
    {
        //get the image coordinate for this point and convert to mm
        float X = (float)(*(pc))[x].x;
        float Y = (float)(*(pc))[x].y;
        float Z = (float)(*(pc))[x].z;

        //store xyz in point cloud, transforming from image coordinates, (Z Forward to X Forward)
        *out_x = X;
        *out_y = Y;
        *out_z = Z;

        //increment
        ++out_x;
        ++out_y;
        ++out_z;
    }
  }
  pointlcoud_z_clipped_publisher_ptr->publish(cloud);
  }

  { // keep the namespace in case we need to copy this part for another cloud
  // get the pointcloud that we want to visulize 
  pcl::PointCloud<pcl::PointXY>::Ptr& pc = pes_->lidar_data_->time_scan_map_[time]->pointcloud_filtered_;
  //declare message and sizes
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "laser";
  cloud.header.stamp = ros::Time(time);
  cloud.width  = pc->size();
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false; // there may be invalid points

  //for fields setup
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1,"xyz");
  modifier.resize(cloud.width);

  //iterators
  sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
  
  for (int y=0; y<1; y++)
  {
    for (int x=0; x<pc->size(); x++)
    {
        //get the image coordinate for this point and convert to mm
        float X = (float)(*(pc))[x].x;
        float Y = (float)(*(pc))[x].y;
        float Z = 0;

        //store xyz in point cloud, transforming from image coordinates, (Z Forward to X Forward)
        *out_x = X;
        *out_y = Y;
        *out_z = Z;

        //increment
        ++out_x;
        ++out_y;
        ++out_z;
    }
  }
  pointlcoud_line_filtered_publisher_ptr->publish(cloud);
  }

  { // keep the namespace in case we need to copy this part for another cloud
  // get the pointcloud that we want to visulize
  std::unique_ptr<std::vector<ibex::IntervalVector>>& bc = pes_->lidar_data_->time_scan_map_[time]->boxcloud_filtered_;
  //declare message and sizes
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "laser";
  cloud.header.stamp = ros::Time(time);
  cloud.width  = bc->size();
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false; // there may be invalid points

  //for fields setup
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1,"xyz");
  modifier.resize(cloud.width);

  //iterators
  sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
  
  for (int y=0; y<1; y++)
  {
    for (int x=0; x<bc->size(); x++)
    {
        //get the image coordinate for this point and convert to mm
        float X = (float)(*(bc))[x][0].mid();
        float Y = (float)(*(bc))[x][1].mid();
        float Z = 0;

        //store xyz in point cloud, transforming from image coordinates, (Z Forward to X Forward)
        *out_x = X;
        *out_y = Y;
        *out_z = Z;

        //increment
        ++out_x;
        ++out_y;
        ++out_z;
    }
  }
  boxcloud_mid_line_filtered_publisher_ptr->publish(cloud);
  }

}

Eigen::Vector3d mat_to_rpy(const Eigen::Matrix3d& R)
{
  return Eigen::Vector3d(atan2(R(2,1),R(2,2)),-asin(R(2,0)),atan2(R(1,0),R(0,0)));
}

void save_gt_pose(const Eigen::Affine3d& world_T_car, double time)
{
  Eigen::Affine3d world_T_laser = world_T_car*pes_->global_localizer_->car_T_laser_;
  Eigen::Affine3d world_T_lcam = world_T_car*pes_->global_localizer_->car_T_left_cam_;
  // start of time section
  gt_poses_ofstream << "---" << std::endl; 
  // 1. time
  gt_poses_ofstream << time << std::endl; 
  // 2. compute RPY euler angles
  Eigen::Vector3d rpy_car = mat_to_rpy(world_T_car.linear());
  const Eigen::Vector3d& t_car = world_T_car.translation();
  gt_poses_ofstream << t_car(0) << " " << t_car(1) << " " << t_car(2) << " "
                    << rpy_car(0) << " " << rpy_car(1) << " " << rpy_car(2)  << std::endl; 
  // 3. compute RPY euler angles in for laser
  Eigen::Vector3d rpy_laser = mat_to_rpy(world_T_laser.linear());
  const Eigen::Vector3d& t_laser = world_T_laser.translation();
  gt_poses_ofstream << t_laser(0) << " " << t_laser(1) << " " << t_laser(2) << " "
                    << rpy_laser(0) << " " << rpy_laser(1) << " " << rpy_laser(2)  << std::endl; 
  // 4. compute RPY euler angles in for lcam
  Eigen::Vector3d rpy_lcam = mat_to_rpy(world_T_lcam.linear());
  const Eigen::Vector3d& t_lcam = world_T_lcam.translation();
  gt_poses_ofstream << t_lcam(0) << " " << t_lcam(1) << " " << t_lcam(2) << " "
                    << rpy_lcam(0) << " " << rpy_lcam(1) << " " << rpy_lcam(2)  << std::endl;
}

void read_rosbags()
{
  for(std::string& rosbag_file : rosbags_list)
  {
    rosbag::Bag bag;
    std::cout << "read: " << rosbag_file << std::endl;
    bag.open(rosbag_file, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(left_cam_image_topic);
    topics.push_back(right_cam_image_topic);
    topics.push_back(lidar_topic);
    topics.push_back(imu_topic);
    topics.push_back(gt_pose_topic);
    topics.push_back(tf_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    sensor_msgs::Image left_im;
    sensor_msgs::Image right_im;
    sensor_msgs::PointCloud2 lidar_pc;
    geometry_msgs::PoseStamped gt_pose;
    std::vector<ros::Time> sync_times;
    auto last_call_time = std::chrono::system_clock::now();
    double last_msg_time = start_stamp;
    double last_gps_msg_time; 

    for(rosbag::MessageInstance m : view)
    {
      if(m.getTopic() == left_cam_image_topic && m.getTime().toSec()>=start_stamp && m.getTime().toSec()<=stop_stamp)
      {
        sensor_msgs::ImageConstPtr im_left_ptr = m.instantiate<sensor_msgs::Image>();
        //left_im_publisher_ptr->publish(*im_left_ptr);
        left_im = *im_left_ptr;
        if(sync_times.size())
        {
          if((left_im.header.stamp - sync_times[sync_times.size()-1]).toSec() > 0.01)
          {
            sync_times.clear();
          }
        }
        sync_times.push_back(left_im.header.stamp);
      }
      if(m.getTopic() == right_cam_image_topic && m.getTime().toSec()>=start_stamp && m.getTime().toSec()<=stop_stamp)
      {
        sensor_msgs::ImageConstPtr im_right_ptr = m.instantiate<sensor_msgs::Image>();
        //right_im_publisher_ptr->publish(*im_right_ptr);
        right_im = *im_right_ptr;
        if(sync_times.size())
        {
          if((right_im.header.stamp - sync_times[sync_times.size()-1]).toSec() > 0.01)
          {
            sync_times.clear();
          }
        }
        sync_times.push_back(right_im.header.stamp);
      }
      if(m.getTopic() == lidar_topic && m.getTime().toSec()>=start_stamp && m.getTime().toSec()<=stop_stamp)
      {
        sensor_msgs::PointCloud2ConstPtr pc_ptr = m.instantiate<sensor_msgs::PointCloud2>();
        //lidar_publisher_ptr->publish(*pc_ptr);
        lidar_pc = *pc_ptr;
        if(sync_times.size())
        {
          if((lidar_pc.header.stamp - sync_times[sync_times.size()-1]).toSec() > 0.01)
          {
            sync_times.clear();
          }
        }
        sync_times.push_back(lidar_pc.header.stamp);
      }
      if(m.getTopic() == gt_pose_topic)
      {
        geometry_msgs::PoseStampedConstPtr gt_pose_ptr = m.instantiate<geometry_msgs::PoseStamped>();
        gt_pose_publisher_ptr->publish(*gt_pose_ptr);
        gt_pose = *gt_pose_ptr;
        if(sync_times.size())
        {
          if((gt_pose.header.stamp - sync_times[sync_times.size()-1]).toSec() > 0.01)
          {
            sync_times.clear();
          }
        }
        sync_times.push_back(gt_pose.header.stamp);
      }
      if(m.getTopic() == tf_topic)
      {
        tf::tfMessageConstPtr tf_msg_ptr = m.instantiate<tf::tfMessage>();
        tf_publisher_ptr->publish(*tf_msg_ptr);

        if(sync_times.size() == 4)
        {
          ros::Duration d = sync_times[3] - sync_times[0];
          double diff_time = d.toSec();
          if(diff_time < 0.005)
          {
            if(sync_times[0].toSec()>=start_stamp && sync_times[0].toSec()<=stop_stamp)
            {
              left_im_publisher_ptr->publish(left_im);
              right_im_publisher_ptr->publish(right_im);
              lidar_publisher_ptr->publish(lidar_pc);

              // mechanism to run the bag on real-time speed
              std::chrono::duration<double> system_elpsed_time_chrono = std::chrono::system_clock::now() - last_call_time;
              double system_elpsed_time = system_elpsed_time_chrono.count(); 
              double bag_elapsed_time = (left_im.header.stamp.toSec() - last_msg_time);
              if(use_real_time_speed)
              {
                double wait_time = bag_elapsed_time - system_elpsed_time;
                if(wait_time > 0)
                {
                  ros::Duration(wait_time).sleep();
                }
              }
              last_call_time = std::chrono::system_clock::now();
              last_msg_time = left_im.header.stamp.toSec();

              // process the very first call 
              if(first_call)
              {
                first_call = false;
                tf::Stamped<tf::Pose> utm_T_car__tf; 
                tf::poseStampedMsgToTF(gt_pose, utm_T_car__tf); 
                Eigen::Affine3d utm_T_car; 
                tf::poseTFToEigen(utm_T_car__tf, utm_T_car);
                Eigen::Affine3d utm_T_laser = utm_T_car*pes_->global_localizer_->car_T_laser_;
                double x_gt = utm_T_laser.translation()[0] - utm_world_offset(0);
                double y_gt = utm_T_laser.translation()[1] - utm_world_offset(1);
                double psi = atan2(utm_T_laser.matrix()(1,0),utm_T_laser.matrix()(0,0));
                std::cout << "Approximate initial pose: " << std::endl << "x: " <<  x_gt << std::endl << "y: " << y_gt << std::endl << "psi: " << psi << std::endl; 
                ibex::IntervalVector t(2);
                t[0] = ibex::Interval(x_gt);
                t[0].inflate(init_x_translation_uncertainty_radius);
                t[1] = ibex::Interval(y_gt);
                t[1].inflate(init_y_translation_uncertainty_radius);
                ibex::Interval r(psi);
                r.inflate(init_rotation_uncertainty_radius);
                pes_->initialize_localization(t,r);
                last_gps_msg_time = left_im.header.stamp.toSec();
              }
              // Call this always
              // check if new gps measure can be processed
              if(left_im.header.stamp.toSec() - last_gps_msg_time >= 1/gps_data_frequency)
              {
                tf::Stamped<tf::Pose> utm_T_car__tf; 
                tf::poseStampedMsgToTF(gt_pose, utm_T_car__tf); 
                Eigen::Affine3d utm_T_car; 
                tf::poseTFToEigen(utm_T_car__tf, utm_T_car);
                Eigen::Affine3d utm_T_laser = utm_T_car*pes_->global_localizer_->car_T_laser_;
                Eigen::Vector3d utm_position = utm_T_laser.translation();
                pes_->process_gps(utm_position,left_im.header.stamp.toSec());
                last_gps_msg_time = left_im.header.stamp.toSec(); 
              }
              // perform localization and tracking
              sensor_msgs::ImageConstPtr left_im_ptr(new sensor_msgs::Image(left_im));
              sensor_msgs::ImageConstPtr right_im_ptr(new sensor_msgs::Image(right_im));
              sensor_msgs::PointCloud2ConstPtr pc_ptr(new sensor_msgs::PointCloud2(lidar_pc));
              pes_->localize_and_track(left_im_ptr,right_im_ptr,pc_ptr); 

              // Visualization
              pc_publisher();

              // save gt_pose to file
              if(dump_gt_poses)
              {
                tf::Stamped<tf::Pose> utm_T_car__tf; 
                tf::poseStampedMsgToTF(gt_pose, utm_T_car__tf); 
                Eigen::Affine3d utm_T_car; 
                tf::poseTFToEigen(utm_T_car__tf, utm_T_car);
                Eigen::Affine3d world_T_car = utm_T_car;
                world_T_car.translation()(0) = world_T_car.translation()(0) - utm_world_offset(0);
                world_T_car.translation()(1) = world_T_car.translation()(1) - utm_world_offset(1);
                save_gt_pose(world_T_car,left_im_ptr->header.stamp.toSec()); 
              }
            }

            sync_times.clear(); 
            
          }
        }

      }
      if(m.getTopic() == imu_topic)
      {
        if(m.getTime().toSec()>=start_stamp && m.getTime().toSec()<=stop_stamp)
        {
          sensor_msgs::ImuConstPtr imu_ptr = m.instantiate<sensor_msgs::Imu>();
          imu_publisher_ptr->publish(*imu_ptr);
        }
      }
      
      if(m.getTime().toSec()>=stop_stamp)
      {
        if(pes_->vo_->dump_results_)
        {
          pes_->vo_->save_to_file();
        }
        // loop in while until node is killed manually
        /*while(ros::ok())
        {
          ros::Duration(1).sleep(); 
        }*/
        bag.close();
        ros::shutdown();
        break;
      }
      
      if(!ros::ok())
      {
        break;
      }
      //ros::Duration(0.01).sleep();
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hypascore_localization_node");
  ros::NodeHandle nh;
  // get all parameter values from launch-file
  nh.getParam("/hypascore_localization_node/rosbags_list", rosbags_list);
  nh.getParam("/hypascore_localization_node/left_cam_image_topic", left_cam_image_topic);
  nh.getParam("/hypascore_localization_node/right_cam_image_topic", right_cam_image_topic);
  nh.getParam("/hypascore_localization_node/lidar_topic", lidar_topic);
  nh.getParam("/hypascore_localization_node/imu_topic", imu_topic);
  nh.getParam("/hypascore_localization_node/gt_pose_topic", gt_pose_topic);
  nh.getParam("/hypascore_localization_node/tf_topic", tf_topic);
  nh.getParam("/hypascore_localization_node/rosbag_read_start_stamp", start_stamp);
  nh.getParam("/hypascore_localization_node/rosbag_read_stop_stamp", stop_stamp);
  nh.getParam("/hypascore_localization_node/utm_world_offset_x", utm_world_offset_x);
  nh.getParam("/hypascore_localization_node/utm_world_offset_y", utm_world_offset_y);
  nh.getParam("/hypascore_localization_node/utm_world_offset_z", utm_world_offset_z);
  nh.getParam("/hypascore_localization_node/use_real_time_speed", use_real_time_speed);
  nh.getParam("/hypascore_localization_node/init_x_translation_uncertainty_radius", init_x_translation_uncertainty_radius);
  nh.getParam("/hypascore_localization_node/init_y_translation_uncertainty_radius", init_y_translation_uncertainty_radius);
  nh.getParam("/hypascore_localization_node/init_rotation_uncertainty_radius", init_rotation_uncertainty_radius);
  nh.getParam("/hypascore_localization_node/gps_data_frequency", gps_data_frequency);
  nh.getParam("/hypascore_localization_node/file_gt_poses", file_gt_poses);

  left_im_publisher_ptr.reset(new ros::Publisher);
  *left_im_publisher_ptr = nh.advertise<sensor_msgs::Image>(left_cam_image_topic, 1);
  right_im_publisher_ptr.reset(new ros::Publisher);
  *right_im_publisher_ptr = nh.advertise<sensor_msgs::Image>(right_cam_image_topic, 1);
  lidar_publisher_ptr.reset(new ros::Publisher);
  *lidar_publisher_ptr = nh.advertise<sensor_msgs::PointCloud2>(lidar_topic, 1);
  gt_pose_publisher_ptr.reset(new ros::Publisher);
  *gt_pose_publisher_ptr = nh.advertise<geometry_msgs::PoseStamped>(gt_pose_topic, 1);
  imu_publisher_ptr.reset(new ros::Publisher);
  *imu_publisher_ptr = nh.advertise<sensor_msgs::Imu>(imu_topic, 1);
  tf_publisher_ptr.reset(new ros::Publisher);
  *tf_publisher_ptr = nh.advertise<tf::tfMessage>(tf_topic, 1);
  map_publisher_ptr.reset(new ros::Publisher);
  *map_publisher_ptr = nh.advertise<visualization_msgs::MarkerArray>("building_map", 1);
  ground_map_publisher_ptr.reset(new ros::Publisher);
  *ground_map_publisher_ptr = nh.advertise<visualization_msgs::Marker>("building_footprints", 1);
  feature_publisher_ptr.reset(new ros::Publisher);
  *feature_publisher_ptr = nh.advertise<visualization_msgs::MarkerArray>("visual_features", 1);
  interval_odom_publisher_ptr.reset(new ros::Publisher);
  *interval_odom_publisher_ptr = nh.advertise<visualization_msgs::MarkerArray>("interval_odom", 1);
  pointlcoud_z_clipped_publisher_ptr.reset(new ros::Publisher);
  *pointlcoud_z_clipped_publisher_ptr = nh.advertise<sensor_msgs::PointCloud2>("clipped_point_cloud", 1);
  pointlcoud_line_filtered_publisher_ptr.reset(new ros::Publisher);
  *pointlcoud_line_filtered_publisher_ptr = nh.advertise<sensor_msgs::PointCloud2>("line_filtered_point_cloud", 1);
  boxcloud_mid_line_filtered_publisher_ptr.reset(new ros::Publisher);
  *boxcloud_mid_line_filtered_publisher_ptr = nh.advertise<sensor_msgs::PointCloud2>("line_filtered_box_cloud", 1);

  utm_world_offset = Eigen::Vector3d(utm_world_offset_x,utm_world_offset_y,utm_world_offset_z);
  
  // create objects
  pes_.reset(new PoseEstimationSystem(nh));

  // Visualization Threads
  std::thread map_publish_th(map_publisher);
  //std::thread visual_feature_publish_th(visual_feature_publisher);
  //std::thread visual_odom_th(visual_odom);

  // prepare gt-pose file
  if(file_gt_poses != "") // if file_gt_poses string is not empty, dump gt-poses
  {
    dump_gt_poses = true;
    std::cout << "Start saving to: " << file_gt_poses << std::endl; 
    gt_poses_ofstream.open(file_gt_poses.c_str());
    gt_poses_ofstream << std::setprecision(16);
  }

  // read out the bag
  read_rosbags();
  map_publish_th.join(); 
  ros::shutdown();

  return 0;
}