#include "hypascore_localization/localization/LidarData.hpp"

LidarData::LidarData(ros::NodeHandle& nh)
{
  nh.getParam("/hypascore_localization_node/distance_delta", distance_delta_);
  nh.getParam("/hypascore_localization_node/lidar_type", lidar_type_);
  nh.getParam("/hypascore_localization_node/vertical_angle_delta", vertical_angle_delta_);
  nh.getParam("/hypascore_localization_node/horizontal_angle_delta", horizontal_angle_delta_);
  nh.getParam("/hypascore_localization_node/skip_row", skip_row_);
  nh.getParam("/hypascore_localization_node/skip_col", skip_col_);
  nh.getParam("/hypascore_localization_node/min_z_clip", min_z_clip_);
  nh.getParam("/hypascore_localization_node/distance_bin_size", distance_bin_size_);
  nh.getParam("/hypascore_localization_node/max_line_points_step_size", max_line_points_step_size_);
  nh.getParam("/hypascore_localization_node/max_angle_bin_dist", max_angle_bin_dist_);
  nh.getParam("/hypascore_localization_node/max_neighbour_point_distance", max_neighbour_point_distance_);
  nh.getParam("/hypascore_localization_node/min_num_points_on_line", min_num_points_on_line_);
  nh.getParam("/hypascore_localization_node/min_line_length", min_line_length_);
  nh.getParam("/hypascore_localization_node/selected_points_distance", selected_points_distance_);
}

void LidarData::save_lidar_data(const sensor_msgs::PointCloud2ConstPtr pc_ptr, double timestamp)
{
  // setup the new LidarScan entry
  time_scan_map_[timestamp].reset(new LidarScan);
  std::unique_ptr<LidarScan>& scan = time_scan_map_[timestamp]; 
  scan->timestamp_ = timestamp;
  scan->pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  scan->boxcloud_.reset(new std::vector<ibex::IntervalVector>);
  scan->pointcloud_filtered_.reset(new pcl::PointCloud<pcl::PointXY>);
  scan->boxcloud_filtered_.reset(new std::vector<ibex::IntervalVector>);
  scan->pointcloud_virtual_2d_.reset(new pcl::PointCloud<pcl::PointXY>);
  scan->boxcloud_virtual_2d_.reset(new std::vector<ibex::IntervalVector>);

  /******************
   * Save the Pointcloud and order the indexes along the horizontal angle
   * ********************/
  std::map<int,std::vector<size_t>> angle_point_idx_map; 
  std::vector<Eigen::Vector3d> points;
  std::vector<ibex::IntervalVector> boxes;
  for(int row_counter = 0; row_counter < pc_ptr->height; row_counter+=(1+skip_row_))
  {
    for(int col_counter = 0; col_counter < pc_ptr->width; col_counter+=(1+skip_col_))
    {
      int arrayPosition = row_counter*(int)pc_ptr->row_step + col_counter*(int)pc_ptr->point_step;

      int arrayPos_time = arrayPosition + pc_ptr->fields[0].offset;
      int arrayPos_image_x = arrayPosition + pc_ptr->fields[1].offset;
      int arrayPos_distance = arrayPosition + pc_ptr->fields[2].offset;
      int arrayPos_image_z = arrayPosition + pc_ptr->fields[3].offset;
      int arrayPos_X = arrayPosition + pc_ptr->fields[7].offset;
      int arrayPos_Y = arrayPosition + pc_ptr->fields[8].offset;
      int arrayPos_Z = arrayPosition + pc_ptr->fields[9].offset;
      double time;
      float image_x, image_z, distance, X, Y, Z;

      memcpy(&time, &(pc_ptr->data[arrayPos_time]), sizeof(int64_t));
      memcpy(&image_x, &(pc_ptr->data[arrayPos_image_x]), sizeof(float));
      memcpy(&image_z, &(pc_ptr->data[arrayPos_image_z]), sizeof(float));
      memcpy(&distance, &(pc_ptr->data[arrayPos_distance]), sizeof(float));
      memcpy(&X, &(pc_ptr->data[arrayPos_X]), sizeof(float));
      memcpy(&Y, &(pc_ptr->data[arrayPos_Y]), sizeof(float));
      memcpy(&Z, &(pc_ptr->data[arrayPos_Z]), sizeof(float));

      Eigen::Vector3d p_laser((double)X,(double)Y,(double)Z);
      if(p_laser.norm()>1.0 && p_laser(2) > min_z_clip_ && !(isnan(X)||isnan(Y)||isnan(Z)) && !(isinf(X)||isinf(Y)||isinf(Z)))
      {
        double angle = atan2(p_laser(1),p_laser(0)); 
        int index = (int)std::round(1/(horizontal_angle_delta_*2.0f)*angle);
        angle_point_idx_map[index].push_back(points.size());
        points.push_back(p_laser);
        scan->pointcloud_->push_back(pcl::PointXYZ((float)X,(float)Y,(float)Z));

        ibex::Interval distance_ibx((double)distance);
        distance_ibx.inflate(distance_delta_);
        ibex::IntervalVector box_laser(3);
        if(lidar_type_ == "cepton")
        {
          double r = pow(image_x*image_x+1+image_z*image_z , 0.5);
          double horizontal_angle= atan2(1,image_x);
          double vertical_angle  = acos(image_z/r);
          ibex::Interval r_ibx(r);
          ibex::Interval vertical_angle_ibx(vertical_angle);
          vertical_angle_ibx.inflate(vertical_angle_delta_);
          ibex::Interval horizontal_angle_ibx(horizontal_angle);
          horizontal_angle_ibx.inflate(horizontal_angle_delta_);

          box_laser[0] = -sin(vertical_angle_ibx) * cos(horizontal_angle_ibx)*distance_ibx;
          box_laser[1] = distance_ibx * sin(vertical_angle_ibx) * sin(horizontal_angle_ibx);
          box_laser[2] = -cos(vertical_angle_ibx)*distance_ibx;
        }
        else if(lidar_type_ == "velodyne")
        {
          double r = pow(pow(X,2)+pow(Y,2)+pow(Z,2),0.5);
          double v_angle = acos(Z/r);
          double h_angle = atan2(Y,X);
          ibex::Interval r_ibx(r);
          ibex::Interval vertical_angle_ibx(v_angle);
          vertical_angle_ibx.inflate(vertical_angle_delta_);
          ibex::Interval horizontal_angle_ibx(h_angle);
          horizontal_angle_ibx.inflate(horizontal_angle_delta_);

          box_laser[0] = r_ibx*sin(vertical_angle_ibx)*cos(horizontal_angle_ibx);
          box_laser[1] = r_ibx*sin(vertical_angle_ibx)*sin(horizontal_angle_ibx);
          box_laser[2] = r_ibx*cos(vertical_angle_ibx);
        }
        else
        {
          std::cout << "LiDAR type '" << lidar_type_ << "' incorrect" << std::endl; 
          throw std::exception();
        }
        boxes.push_back(box_laser);
        scan->boxcloud_->push_back(box_laser);
      }
    }
  }

  /****************
   * for each angle bin, find the x-y-distance-bin with the highest support
   * ******************/
  std::vector<Eigen::Vector2d> virtual_2d_scan;
  std::vector<ibex::IntervalVector> virtual_2d_scan_boxes;
  virtual_2d_scan.reserve(angle_point_idx_map.size()); 
  std::vector<size_t> remaining_idxs;
  remaining_idxs.reserve(angle_point_idx_map.size());
  for(std::pair<int,std::vector<size_t>> pair : angle_point_idx_map)
  {
    std::vector<size_t>& points_idxs = pair.second; 
    std::map<int,std::vector<size_t>> distance_point_idx_map; 
    for(size_t& idx : points_idxs)
    {
      Eigen::Vector3d& point = points[idx]; 
      Eigen::Vector2d point_2d(point(0),point(1));
      double dist = point_2d.norm(); 
      int index = (int)std::round(1/(distance_bin_size_)*dist);
      distance_point_idx_map[index].push_back(idx); 
    }

    // get the distance with the largest vertical_angle_span
    std::vector<size_t> highest_support_points;
    double largest_vertical_angle_span = 0; 
    for(std::pair<int,std::vector<size_t>> distance_points_pair : distance_point_idx_map)
    {
      ibex::Interval vertical_angle_span; 
      vertical_angle_span.set_empty();
      for (size_t& idx: distance_points_pair.second)
      {
        Eigen::Vector2d point_xy(points[idx](0), points[idx](1));
        double vertical_angle = atan2(points[idx](2),point_xy.norm());
        vertical_angle_span = vertical_angle_span | vertical_angle;
      }
      if(vertical_angle_span.diam() > largest_vertical_angle_span)
      {
        largest_vertical_angle_span = vertical_angle_span.diam();
        highest_support_points = distance_points_pair.second; 
      }
    }
    // save one associated point to angle-bin and distance-bin
    if(highest_support_points.size() > 0)
    {
      remaining_idxs.emplace_back(virtual_2d_scan.size());
      Eigen::Vector2d point_2d(points[highest_support_points[0]](0),points[highest_support_points[0]](1));
      virtual_2d_scan.push_back(point_2d);
      virtual_2d_scan_boxes.push_back(boxes[highest_support_points[0]]);
      // push the points to the data constainers
      for(size_t idx : highest_support_points)
      {
        pcl::PointXY p_laser_pcl;
        p_laser_pcl.x = (float)points[idx].x();
        p_laser_pcl.y = (float)points[idx].y();
        scan->pointcloud_virtual_2d_->push_back(p_laser_pcl);
        scan->boxcloud_virtual_2d_->push_back(boxes[idx]);
      }
    }  
  }


  /******************
   * Extract points that lie on a line
   * *******************/
  std::vector<std::set<size_t>> lines;  
  while(remaining_idxs.size() > 2)
  {
    size_t idx1 = remaining_idxs[0]; 
    remaining_idxs.erase(remaining_idxs.begin()); 
    size_t idx2; 
    if(remaining_idxs.size() > max_line_points_step_size_)
    {
      idx2 = remaining_idxs[max_line_points_step_size_];
      remaining_idxs.erase(remaining_idxs.begin()+max_line_points_step_size_);
    }
    else
    {
      idx2 = remaining_idxs[remaining_idxs.size()-1]; 
      remaining_idxs.erase(remaining_idxs.begin()+remaining_idxs.size()-1);
    }
    Eigen::Vector2d& p1 = virtual_2d_scan[idx1]; 
    Eigen::Vector2d& p2 = virtual_2d_scan[idx2];

    // generate new points-index-list
    std::set<size_t> points_on_line; 
    points_on_line.insert(idx1);
    points_on_line.insert(idx2); 

    // compute line 
    Eigen::Vector2d diff = p1-p2;
    double a = atan2(-diff(0),diff(1));
    Eigen::Vector2d n(cos(a), sin(a)); 
    double d = n.transpose()*p1; 

    // iterate through all remaining points and and insert to line
    double max_line_dist = 0.3; 
    for(size_t i = 0; i < remaining_idxs.size(); ++i) 
    {
      size_t test_idx = remaining_idxs[i]; 
      Eigen::Vector2d& test_point = virtual_2d_scan[test_idx];
      double point_dist = abs(d - n.transpose()*test_point);
      if(point_dist < max_line_dist)
      { 
        points_on_line.insert(test_idx);
        remaining_idxs.erase(remaining_idxs.begin()+i);
        i--;
      }
    }

    // get largest list of connected points
    std::vector<std::vector<size_t>> splitted_lines(1);
    std::vector<size_t>* splitted_line = &splitted_lines[0];  
    for(size_t idx : points_on_line)
    {
      if(splitted_line->size() == 0)
      {
        splitted_line->push_back(idx);
      }
      else
      {
        Eigen::Vector2d& p1 = virtual_2d_scan[(*splitted_line)[splitted_line->size()-1]];
        Eigen::Vector2d& p2 = virtual_2d_scan[idx];
        if(abs((int)(*splitted_line)[splitted_line->size()-1]-(int)idx) <= max_angle_bin_dist_ && (p1-p2).norm() <= max_neighbour_point_distance_)
        {
          splitted_line->push_back(idx);
        }
        else
        {
          std::vector<size_t> new_splitted_line;
          splitted_lines.push_back(new_splitted_line);
          splitted_line = &splitted_lines[splitted_lines.size()-1];
        }
      }
    }
    int max_size = 0; 
    std::vector<size_t> largest_line;
    for(std::vector<size_t>& l : splitted_lines)
    {
      if(max_size < l.size())
      {
        largest_line = l;
        max_size = l.size(); 
      }
    }
    points_on_line.clear(); 
    for(size_t idx : largest_line)
    {
      points_on_line.insert(idx); 
    }
    lines.push_back(points_on_line); 
  } 

  /*****************
   * Iterate through the extracted lines, only copy those points that belong to lines with enough points and sufficient length
   * *********************/ 
  for(std::set<size_t>& line : lines)
  {
    size_t first_idx = *(line.begin());
    size_t last_idx = *(line.rbegin());  
    double length = (virtual_2d_scan[first_idx] - virtual_2d_scan[last_idx]).norm(); 

    // copy to laser_scan points
    if(length >= min_line_length_*0.4)
    {
      for(size_t idx : line)
      { 
        pcl::PointXY p_laser_pcl;
        p_laser_pcl.x = (float)virtual_2d_scan[idx](0);
        p_laser_pcl.y = (float)virtual_2d_scan[idx](1);
        scan->pointcloud_filtered_->push_back(p_laser_pcl);
      }
    }

    // copy to boxes only if enough points and sufficient length
    if(line.size() >= min_num_points_on_line_ && length >= min_line_length_)
    { 
      std::vector<size_t> inserted_idxs; 
      for(size_t idx : line)
      { 
        if(idx == last_idx || idx == first_idx)
        {
          inserted_idxs.push_back(idx);
          scan->boxcloud_filtered_->push_back(virtual_2d_scan_boxes[idx]);
        }
        else
        {
          double point_dist = (virtual_2d_scan[idx] - virtual_2d_scan[inserted_idxs[inserted_idxs.size()-1]]).norm();
          if(point_dist >= selected_points_distance_)
          {
            inserted_idxs.push_back(idx);
            scan->boxcloud_filtered_->push_back(virtual_2d_scan_boxes[idx]);
          }
        }        
      } 
    }
  } 
}