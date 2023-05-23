#include "hypascore_localization/localization/CMMap.hpp"
#include <iomanip>
#include <chrono>

CMMap::CMMap(std::string& city_model_file, Eigen::Vector3d& utm_world_offset)
{
  utm_world_offset_ = utm_world_offset;
  buildings_.clear();
  std::cout << "CMMap Input File: " << city_model_file << std::endl; 
  std::ifstream map_file(city_model_file);
  parsing_state_machine(map_file);
  std::cout << "Buildings: " << buildings_.size() << std::endl;
  compute_wall_ground_lines();
  std::cout << "Wall Ground Lines: " << wall_ground_lines_.size() << std::endl;
  build_wall_ground_kdtree();
  build_connected_wall_ground_kdtree();
  std::cout << "Wall Ground Point Cloud: " << wall_ground_pc_ptr_->size() << std::endl;
  compute_convex_hull();
  std::cout << "Computed convex hull to the map: " << cv_points_convex_hull_map_.size() << " corners" << std::endl; 
}

void CMMap::parsing_state_machine(std::ifstream& map_file)
{
  std::string line;
  std::getline(map_file, line);
  if(line.find("building_id:")==std::string::npos)
  {
    std::cout << "Parsing Error: building_id is not first token!" << std::endl;
    return; 
  } 
  cur_state_ = States::NEW_BUILDING;
  Building* cur_building_ptr = NULL; 
  while(std::getline(map_file, line))
  {
    /***************
    *NEW_BUILDING
    ****************/
    if(cur_state_ == States::NEW_BUILDING)
    {
      // create a new building object
      Building new_building;
      buildings_.push_back(new_building);
      cur_building_ptr = &(buildings_[buildings_.size()-1]);
      if(line.find("roof:")!=std::string::npos)
      {
        cur_state_ = States::ROOF_SURFACES;
      }
      else if(line.find("ground:")!=std::string::npos)
      {
        cur_state_ = States::GROUND_SURFACES;
      }
      else if(line.find("wall:")!=std::string::npos)
      {
        cur_state_ = States::WALL_SURFACES;
      }
      else
      {
        std::cout << "Parsing Error: no surfaces for the building" << std::endl;
        return;
      }
    }

    /***************
    *ROOF_SURFACES
    ****************/
    else if(cur_state_ == States::ROOF_SURFACES)
    {
      Surface* cur_surface_ptr = NULL;
      bool iterate = true; 
      if(line.find("surface:")!=std::string::npos)
      {
        Surface surface;
        cur_building_ptr->roofs.push_back(surface);
        cur_surface_ptr = &(cur_building_ptr->roofs[cur_building_ptr->roofs.size()-1]);
      }
      else
      {
        // no surfaces!
        if(line.find("ground:")!=std::string::npos)
        {
          cur_state_ = States::GROUND_SURFACES;
        }
        else if(line.find("wall:")!=std::string::npos)
        {
          cur_state_ = States::WALL_SURFACES;
        }
        else if(line.find("building_id:")!=std::string::npos)
        {
          cur_state_ = States::NEW_BUILDING;
        }
        iterate = false; 
      }
      while(iterate && std::getline(map_file, line))
      {
        if(line.find("surface:")!=std::string::npos)
        {
          // insert new surface
          Surface surface;
          cur_building_ptr->roofs.push_back(surface);
          cur_surface_ptr = &(cur_building_ptr->roofs[cur_building_ptr->roofs.size()-1]);
        }
        else if(line.find("ground:")!=std::string::npos)
        {
          cur_state_ = States::GROUND_SURFACES;
          break;
        }
        else if(line.find("wall:")!=std::string::npos)
        {
          cur_state_ = States::WALL_SURFACES;
          break;
        }
        else if(line.find("building_id:")!=std::string::npos)
        {
          cur_state_ = States::NEW_BUILDING;
          break;
        }
        else
        {
          // fill the current surface
          std::stringstream ss(line);
          Eigen::Vector3d point; 
          ss >> point(0) >> point(1) >> point(2);
          point = point - utm_world_offset_;
          cur_surface_ptr->points.push_back(point);
        }
      }
    }

    /***************
    *GROUND_SURFACES
    ****************/
    else if(cur_state_ == States::GROUND_SURFACES)
    {
      Surface* cur_surface_ptr = NULL;
      bool iterate = true; 
      if(line.find("surface:")!=std::string::npos)
      {
        Surface surface;
        cur_building_ptr->grounds.push_back(surface);
        cur_surface_ptr = &(cur_building_ptr->grounds[cur_building_ptr->grounds.size()-1]);
      }
      else
      {
        if(line.find("roof:")!=std::string::npos)
        {
          cur_state_ = States::ROOF_SURFACES;
        }
        else if(line.find("wall:")!=std::string::npos)
        {
          cur_state_ = States::WALL_SURFACES;
        }
        else if(line.find("building_id:")!=std::string::npos)
        {
          cur_state_ = States::NEW_BUILDING;
        }
        iterate = false; 
      }
      while(iterate && std::getline(map_file, line))
      {
        if(line.find("surface:")!=std::string::npos)
        {
          // insert new surface
          Surface surface;
          cur_building_ptr->grounds.push_back(surface);
          cur_surface_ptr = &(cur_building_ptr->grounds[cur_building_ptr->grounds.size()-1]);
        }
        else if(line.find("roof:")!=std::string::npos)
        {
          cur_state_ = States::ROOF_SURFACES;
          break;
        }
        else if(line.find("wall:")!=std::string::npos)
        {
          cur_state_ = States::WALL_SURFACES;
          break;
        }
        else if(line.find("building_id:")!=std::string::npos)
        {
          cur_state_ = States::NEW_BUILDING;
          break;
        }
        else
        {
          // fill the current surface
          std::stringstream ss(line);
          Eigen::Vector3d point; 
          ss >> point(0) >> point(1) >> point(2);
          point = point - utm_world_offset_;
          cur_surface_ptr->points.push_back(point);
        }
      }
    }

    /***************
    *WALL_SURFACES
    ****************/
    else if(cur_state_ == States::WALL_SURFACES)
    {
      Surface* cur_surface_ptr = NULL;
      bool iterate = true; 
      if(line.find("surface:")!=std::string::npos)
      {
        Surface surface;
        cur_building_ptr->walls.push_back(surface);
        cur_surface_ptr = &(cur_building_ptr->walls[cur_building_ptr->walls.size()-1]);
      }
      else
      {
        if(line.find("roof:")!=std::string::npos)
        {
          cur_state_ = States::ROOF_SURFACES;
        }
        else if(line.find("ground:")!=std::string::npos)
        {
          cur_state_ = States::GROUND_SURFACES;
        }
        else if(line.find("building_id:")!=std::string::npos)
        {
          cur_state_ = States::NEW_BUILDING;
        }
        iterate = false; 
      }
      while(iterate && std::getline(map_file, line))
      {
        if(line.find("surface:")!=std::string::npos)
        {
          // insert new surface
          Surface surface;
          cur_building_ptr->walls.push_back(surface);
          cur_surface_ptr = &(cur_building_ptr->walls[cur_building_ptr->walls.size()-1]);
        }
        else if(line.find("roof:")!=std::string::npos)
        {
          cur_state_ = States::ROOF_SURFACES;
          break;
        }
        else if(line.find("ground:")!=std::string::npos)
        {
          cur_state_ = States::GROUND_SURFACES;
          break;
        }
        else if(line.find("building_id:")!=std::string::npos)
        { 
          cur_state_ = States::NEW_BUILDING;
          break;
        }
        else
        {
          // fill the current surface
          std::stringstream ss(line);
          Eigen::Vector3d point; 
          ss >> point(0) >> point(1) >> point(2);
          point = point - utm_world_offset_;
          cur_surface_ptr->points.push_back(point);
        }
      }
    }
 
  }
}

void CMMap::compute_wall_ground_lines()
{
  for(size_t b_counter = 0; b_counter < buildings_.size(); b_counter++)
  {
    Building& b = buildings_[b_counter];
    for(size_t w_counter = 0; w_counter < b.walls.size(); w_counter++)
    {
      WallGroundLine wgl;
      Surface& s = b.walls[w_counter];

      // check if s also has points that are on the ground surface
      bool wall_ground_surface = false;
      for(Eigen::Vector3d& wall_point : s.points)
      {
        for(Surface& s_ground : b.grounds)
        {
          for(Eigen::Vector3d& ground_point : s_ground.points)
          {
            if(ground_point == wall_point)
            {
              wall_ground_surface = true;
              break;
            }
          }
          if(wall_ground_surface)
          {
            break;
          }
        }
        if(wall_ground_surface)
        {
          break;
        }
      }

      if(s.points.size() > 2 && wall_ground_surface)
      {
        Eigen::Vector3d average_point;
        average_point.setZero();
        for(Eigen::Vector3d& p : s.points)
        {
          average_point = average_point + p;
        }
        average_point = average_point/((double)s.points.size());
        Eigen::MatrixXd A(s.points.size(),2);
        int row_counter = 0; 
        for(Eigen::Vector3d& p : s.points)
        {
          Eigen::Vector3d cleaned_p = p - average_point;
          A(row_counter,0) = cleaned_p(0);
          A(row_counter,1) = cleaned_p(1);
          row_counter++;
        }
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(2);
        es.compute(A.transpose()*A);
        Eigen::Vector2d eigenvalues =  es.eigenvalues();
        Eigen::Matrix2d eigenvectors = es.eigenvectors();
        int idx_to_smallest_eigenvalue = 0;
        for(int idx_counter = 0; idx_counter < 2; idx_counter++)
        {
          if(abs(eigenvalues(idx_to_smallest_eigenvalue)) > abs(eigenvalues(idx_counter)))
          {
            idx_to_smallest_eigenvalue = idx_counter;
          }
        }
        Eigen::Vector2d ls_n(eigenvectors.col(idx_to_smallest_eigenvalue)(0),eigenvectors.col(idx_to_smallest_eigenvalue)(1));
        ls_n = ls_n/ls_n.norm();
        if(ls_n(1) < 0)
        {
          ls_n = -ls_n;
        }
        Eigen::Vector2d average_point_2d(average_point(0),average_point(1));
        double ls_d = ls_n.transpose()*average_point_2d;
        Eigen::Vector3d line_params(ls_n(0),ls_n(1),ls_d);
        wgl.line_params = line_params;
        wgl.building_idx = b_counter;
        wgl.wall_idx = w_counter;

        //wgl.start_point and wgl.end_point
        Eigen::Vector2d start(s.points[0](0),s.points[0](1));
        double largest_dist = 0; 
        for(Eigen::Vector3d& p : s.points)
        {
          Eigen::Vector2d p_2d(p(0),p(1));
          double dist = (average_point_2d-p_2d).norm(); 
          if(dist > largest_dist)
          {
            start = p_2d;
            largest_dist = dist;
          }
        }
        wgl.start_point = start;

        Eigen::Vector2d end(s.points[0](0),s.points[0](1));
        largest_dist = 0;
        for(Eigen::Vector3d& p : s.points)
        {
          Eigen::Vector2d p_2d(p(0),p(1));
          double dist = (start-p_2d).norm(); 
          if(dist > largest_dist)
          {
            end = p_2d;
            largest_dist = dist;
          }
        }
        wgl.end_point = end;
        wgl.r_color = 0.2 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(1.0-0.2)));
        wgl.g_color = 0.2 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(1.0-0.2)));
        wgl.b_color = 0.2 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(1.0-0.2)));

        wall_ground_lines_.push_back(wgl);
        wall_ground_lines_all_.push_back(wgl);
      }
    }
  }

  // walls that belong to 2 buildings cannot be seen -> delete those
  for(int wgl_counter = 0; wgl_counter < wall_ground_lines_.size(); wgl_counter++)
  {
    for(int wgl_counter_internal = wgl_counter+1; wgl_counter_internal < wall_ground_lines_.size(); wgl_counter_internal++)
    {
      if(((wall_ground_lines_[wgl_counter].start_point - wall_ground_lines_[wgl_counter_internal].start_point).norm() < 0.1 &&
        (wall_ground_lines_[wgl_counter].end_point - wall_ground_lines_[wgl_counter_internal].end_point).norm() < 0.1) ||
        ((wall_ground_lines_[wgl_counter].start_point - wall_ground_lines_[wgl_counter_internal].end_point).norm() < 0.1 &&
        (wall_ground_lines_[wgl_counter].end_point - wall_ground_lines_[wgl_counter_internal].start_point).norm() < 0.1)
        )
      {
        // wgl_counter_internal > wgl_counter holds always! 
        wall_ground_lines_.erase(wall_ground_lines_.begin()+wgl_counter_internal);
        wall_ground_lines_.erase(wall_ground_lines_.begin()+wgl_counter);
        wgl_counter--;
        wgl_counter_internal--;
        break;
      }
    }
  }

  // add wall_ground_lines_ indices to the buildings
  for(int wgl_counter = 0; wgl_counter < wall_ground_lines_.size(); wgl_counter++)
  {
    WallGroundLine& wgl = wall_ground_lines_[wgl_counter];
    buildings_[wgl.building_idx].outer_wgl_idxs.push_back(wgl_counter); 
  }

  // merge all wall_ground_lines_ into one 
  std::set<int> considered_indices; 
  for(int wgl1_counter = 0; wgl1_counter < wall_ground_lines_.size(); wgl1_counter++)
  {
    if(considered_indices.find(wgl1_counter) == considered_indices.end())
    {
      WallGroundLine& wgl1 = wall_ground_lines_[wgl1_counter]; 
      considered_indices.insert(wgl1_counter); 
      Eigen::Vector2d start = wgl1.start_point; 
      Eigen::Vector2d end = wgl1.end_point;
      for(int wgl2_counter = 0; wgl2_counter < wall_ground_lines_.size(); wgl2_counter++)
      {
        if(wgl1_counter!=wgl2_counter && considered_indices.find(wgl2_counter) == considered_indices.end())
        {
          WallGroundLine& wgl2 = wall_ground_lines_[wgl2_counter];
          // check if line parameters identical and start or end are connected
          double wgl1_angle = atan2(wgl1.line_params(1),wgl1.line_params(0));
          double wgl2_angle = atan2(wgl2.line_params(1),wgl2.line_params(0));
          if(abs(wgl1_angle-wgl2_angle) < 0.03) // on the same line
          { 
            if((wgl2.end_point-start).norm() <= 0.1)
            {
              start = wgl2.start_point;
              considered_indices.insert(wgl2_counter);
              wgl2_counter = -1; 
            }
            else if((wgl2.end_point - end).norm() <= 0.1)
            {
              end = wgl2.start_point; 
              considered_indices.insert(wgl2_counter);
              wgl2_counter = -1;
            }
            else if((wgl2.start_point - end).norm() <= 0.1)
            {
              end = wgl2.end_point; 
              considered_indices.insert(wgl2_counter);
              wgl2_counter = -1;
            }
            else if((wgl2.start_point - start).norm() <= 0.1)
            {
              start = wgl2.end_point; 
              considered_indices.insert(wgl2_counter);
              wgl2_counter = -1;
            }
          }
        }
      }
      WallGroundLine connected_wgl;
      connected_wgl = wgl1;
      connected_wgl.start_point = start; 
      connected_wgl.end_point = end; 
      connected_wall_ground_lines_.push_back(connected_wgl); 
    }
  }
}

void CMMap::build_wall_ground_kdtree()
{
  wall_ground_pc_ptr_.reset(new pcl::PointCloud<pcl::PointXY>);
  wall_ground_pc_ptr_->clear();
  pc_to_wgl_map_.clear();

  for(int wgl_counter = 0; wgl_counter<wall_ground_lines_.size(); wgl_counter++)
  {
    WallGroundLine& wgl = wall_ground_lines_[wgl_counter];
    Eigen::Vector2d& start = wgl.start_point;
    Eigen::Vector2d& end = wgl.end_point;
    Eigen::Vector2d direction = end - start;
    double max_length = direction.norm();
    direction = direction/direction.norm();
    double length = 0; 
    while(length < max_length)
    {
      Eigen::Vector2d line_point = start + length*direction;
      pcl::PointXY pcl_point;
      pcl_point.x = line_point(0);
      pcl_point.y = line_point(1);
      wall_ground_pc_ptr_->push_back(pcl_point);
      pc_to_wgl_map_[wall_ground_pc_ptr_->size()-1] = wgl_counter;
      length = length+1;
    }
    Eigen::Vector2d line_point = end;
    pcl::PointXY pcl_point;
    pcl_point.x = line_point(0);
    pcl_point.y = line_point(1);
    wall_ground_pc_ptr_->push_back(pcl_point);
    pc_to_wgl_map_[wall_ground_pc_ptr_->size()-1] = wgl_counter;
  }

  wall_ground_kdtree_.setInputCloud(wall_ground_pc_ptr_);
}

void CMMap::build_connected_wall_ground_kdtree()
{
  connected_wall_ground_pc_ptr_.reset(new pcl::PointCloud<pcl::PointXY>);
  connected_wall_ground_pc_ptr_->clear();
  connected_pc_to_wgl_map_.clear();

  for(int wgl_counter = 0; wgl_counter<connected_wall_ground_lines_.size(); wgl_counter++)
  {
    WallGroundLine& wgl = connected_wall_ground_lines_[wgl_counter];
    Eigen::Vector2d& start = wgl.start_point;
    Eigen::Vector2d& end = wgl.end_point;
    Eigen::Vector2d direction = end - start;
    double max_length = direction.norm();
    direction = direction/direction.norm();
    double length = 0; 
    while(length < max_length)
    {
      Eigen::Vector2d line_point = start + length*direction;
      pcl::PointXY pcl_point;
      pcl_point.x = line_point(0);
      pcl_point.y = line_point(1);
      connected_wall_ground_pc_ptr_->push_back(pcl_point);
      connected_pc_to_wgl_map_[connected_wall_ground_pc_ptr_->size()-1] = wgl_counter;
      length = length+1;
    }
    Eigen::Vector2d line_point = end;
    pcl::PointXY pcl_point;
    pcl_point.x = line_point(0);
    pcl_point.y = line_point(1);
    connected_wall_ground_pc_ptr_->push_back(pcl_point);
    connected_pc_to_wgl_map_[connected_wall_ground_pc_ptr_->size()-1] = wgl_counter;
  }

  connected_wall_ground_kdtree_.setInputCloud(connected_wall_ground_pc_ptr_);
}

void CMMap::wall_ground_lines_radius_search(pcl::PointCloud<pcl::PointXYZ>& pc, std::vector<ibex::IntervalVector>& bc_laser, double& sampled_point_radius, double& max_line_distance, std::set<WallGroundLine*>& close_wgl_ptrs, std::map<WallGroundLine*,std::vector<size_t>>& line_measurement_matches)
{
  //auto start = std::chrono::high_resolution_clock::now();
  std::map<WallGroundLine*,int> accumulator;
  std::vector<std::pair<size_t,WallGroundLine*>> point_line_matches;
  for(int point_counter = 0; point_counter < pc.size(); point_counter+=2) // take every second point!
  {
    if(pc[point_counter].z > -1.7f)
    {
      pcl::PointXY p;
      p.x = pc[point_counter].x;
      p.y = pc[point_counter].y;
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      wall_ground_kdtree_.radiusSearch (p, sampled_point_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      if(pointIdxRadiusSearch.size() > 0)
      {
        //close_wgl_ptrs.insert(&wall_ground_lines_[pc_to_wgl_map_[pointIdxRadiusSearch[0]]]);
        WallGroundLine& wgl = wall_ground_lines_[pc_to_wgl_map_[pointIdxRadiusSearch[0]]];
        Eigen::Vector2d p_eig((double)p.x, (double)p.y);
        Eigen::Vector2d n(wgl.line_params(0),wgl.line_params(1));
        double d = wgl.line_params(2);
        double distance = p_eig.transpose()*n-d;
        if(abs(distance) < max_line_distance)
        {
          std::pair<size_t,WallGroundLine*> match_pair;
          match_pair.first = point_counter;
          match_pair.second = &wgl;
          point_line_matches.push_back(match_pair);
          //close_wgl_ptrs.insert(&wall_ground_lines_[pc_to_wgl_map_[pointIdxRadiusSearch[0]]]);
          if(accumulator.find(&wall_ground_lines_[pc_to_wgl_map_[pointIdxRadiusSearch[0]]]) == accumulator.end())
          {
            accumulator[&wall_ground_lines_[pc_to_wgl_map_[pointIdxRadiusSearch[0]]]] = 1; 
          }
          else
          {
            accumulator[&wall_ground_lines_[pc_to_wgl_map_[pointIdxRadiusSearch[0]]]] += 1; 
          }        
        }
      }
    }
  }
  /*auto close_points_time = std::chrono::high_resolution_clock::now();
  auto close_points_duration = std::chrono::duration_cast<std::chrono::microseconds>(close_points_time - start);
  std::cout << "close_points_duration: " << close_points_duration.count()*std::pow(10,-6) << "s" << std::endl;*/

  float average = 0; 
  for(std::pair<WallGroundLine*,int> pair : accumulator)
  {
    average+=pair.second;
  } 
  average = average/accumulator.size();
  for(int point_line_pair_counter = 0; point_line_pair_counter < point_line_matches.size(); point_line_pair_counter++)
  {
    std::pair<size_t,WallGroundLine*> point_line_pair = point_line_matches[point_line_pair_counter];
    if(accumulator[point_line_pair.second] < average)
    {
      point_line_matches.erase(point_line_matches.begin()+point_line_pair_counter);
      point_line_pair_counter--; 
    }
  }
  /*auto average_filter_time = std::chrono::high_resolution_clock::now();
  auto average_filter_duration = std::chrono::duration_cast<std::chrono::microseconds>(average_filter_time - close_points_time);
  std::cout << "average_filter_duration: " << average_filter_duration.count()*std::pow(10,-6) << "s" << std::endl;*/

  // convert the matched data in a convenient form, where a wall line the corresponding points are assigned
  std::map<WallGroundLine*,std::vector<size_t>> line_measurement_matches_all;
  for(std::pair<size_t,WallGroundLine*>& match : point_line_matches)
  {
    line_measurement_matches_all[match.second].push_back(match.first);
  } 
  /*auto convert_time = std::chrono::high_resolution_clock::now();
  auto convert_duration = std::chrono::duration_cast<std::chrono::microseconds>(convert_time - average_filter_time);
  std::cout << "convert_duration: " << convert_duration.count()*std::pow(10,-6) << "s" << std::endl;*/

  // Fit an average plane through the points -> only use boxes that have intersection with that plane!
  std::map<WallGroundLine*,std::vector<size_t>> line_measurement_matches_plane_filtered;
  for(std::pair<WallGroundLine*,std::vector<size_t>> match_pair : line_measurement_matches_all)
  {
    // fit a plane to the associated boxes
    Eigen::MatrixXd A(match_pair.second.size(),3);
    Eigen::MatrixXd W(match_pair.second.size(),match_pair.second.size());
    W.setZero();
    int row_counter = 0;
    Eigen::Vector3d average_point; 
    average_point.setZero(); 
    double weight = 0; 
    for(size_t& idx : match_pair.second)
    {
      ibex::IntervalVector& plane_box = bc_laser[idx];
      weight += 1/(ibex::norm(plane_box.diam())); 
      average_point = average_point + 1/(ibex::norm(plane_box.diam()))*Eigen::Vector3d(plane_box[0].mid(),plane_box[1].mid(),plane_box[2].mid());
    }
    average_point = average_point/weight;
    for(size_t& idx : match_pair.second)
    { 
      ibex::IntervalVector& plane_box = bc_laser[idx];
      A(row_counter,0) = plane_box[0].mid()-average_point(0);
      A(row_counter,1) = plane_box[1].mid()-average_point(1);
      A(row_counter,2) = plane_box[2].mid()-average_point(2);
      W(row_counter,row_counter) = 1/(ibex::norm(plane_box.diam()));
      row_counter++; 
    }
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(3);
    es.compute(A.transpose()*W*A);
    Eigen::Vector3d eigenvalues =  es.eigenvalues();
    Eigen::Matrix3d eigenvectors = es.eigenvectors();
    int idx_to_smallest_eigenvalue = 0;
    for(int idx_counter = 0; idx_counter < 3; idx_counter++)
    {
      if(abs(eigenvalues(idx_to_smallest_eigenvalue)) > abs(eigenvalues(idx_counter)))
      {
        idx_to_smallest_eigenvalue = idx_counter;
      }
    }
    Eigen::Vector3d ls_n(eigenvectors.col(idx_to_smallest_eigenvalue)(0),eigenvectors.col(idx_to_smallest_eigenvalue)(1),eigenvectors.col(idx_to_smallest_eigenvalue)(2));
    ls_n = ls_n/ls_n.norm();
    if(ls_n(1) < 0)
    {
      ls_n = -ls_n;
    }
    double ls_d = ls_n.transpose()*average_point;

    for(size_t& idx : match_pair.second)
    { 
      ibex::IntervalVector& plane_box = bc_laser[idx];
      ibex::Interval d = (ls_n(0)*plane_box[0]+ls_n(1)*plane_box[1]+ls_n(2)*plane_box[2]);
      if(d.contains(ls_d))
      {
        line_measurement_matches_plane_filtered[match_pair.first].push_back(idx);
      }
    }
  }
  /*auto average_plane_time = std::chrono::high_resolution_clock::now();
  auto average_plane_duration = std::chrono::duration_cast<std::chrono::microseconds>(average_plane_time - convert_time);
  std::cout << "average_plane_duration: " << average_plane_duration.count()*std::pow(10,-6) << "s" << std::endl;*/

  // compute the average number of the associated points 
  std::map<WallGroundLine*,std::vector<size_t>> line_measurement_matches_average_filtered;
  int average_point_number_per_plane = 0;
  for(std::pair<WallGroundLine*,std::vector<size_t>> match_pair : line_measurement_matches_plane_filtered)
  {
    average_point_number_per_plane = average_point_number_per_plane+match_pair.second.size();
  }
  average_point_number_per_plane = average_point_number_per_plane/line_measurement_matches_plane_filtered.size();
  for(std::pair<WallGroundLine*,std::vector<size_t>> match_pair : line_measurement_matches_plane_filtered)
  {
    if(match_pair.second.size() >= average_point_number_per_plane*0.2f) // 0.7f
    {
      line_measurement_matches_average_filtered[match_pair.first] = match_pair.second;
    }
  }
  /*auto associated_points_time = std::chrono::high_resolution_clock::now();
  auto associated_points_duration = std::chrono::duration_cast<std::chrono::microseconds>(associated_points_time - average_plane_time);
  std::cout << "associated_points_duration: " << associated_points_duration.count()*std::pow(10,-6) << "s" << std::endl;*/

  // PCA-Filter, only planar structures are 
  std::map<WallGroundLine*,std::vector<size_t>> line_measurement_matches_pca_filtered;
  for(std::pair<WallGroundLine*,std::vector<size_t>> matched_pair : line_measurement_matches_average_filtered)
  {
    Eigen::Vector3d average;
    average.setZero();
    for(size_t& idx : matched_pair.second)
    {
      average(0) += bc_laser[idx].mid()[0];
      average(1) += bc_laser[idx].mid()[1];
      average(2) += bc_laser[idx].mid()[2];
    }
    average = average/matched_pair.second.size();

    Eigen::Matrix3d cov;
    cov.setZero();
    Eigen::Vector3d X;
    for(size_t& idx : matched_pair.second)
    {
      X(0) = bc_laser[idx].mid()[0];
      X(1) = bc_laser[idx].mid()[1];
      X(2) = bc_laser[idx].mid()[2];
      cov = cov + (X-average)*((X-average).transpose());
    }
    cov = cov/matched_pair.second.size();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(3);
    es.compute(cov);
    Eigen::Vector3d eigenvalues =  es.eigenvalues();
    Eigen::Matrix3d eigenvectors = es.eigenvectors();
    Eigen::Vector3d biggest_eigenvector = eigenvectors.col(2);
    Eigen::Vector2d xy_biggest_eigenvector(biggest_eigenvector(0),biggest_eigenvector(1));
    if(xy_biggest_eigenvector.norm()/biggest_eigenvector.norm() > 0.3)
    {
      line_measurement_matches_pca_filtered[matched_pair.first] = matched_pair.second; 
      close_wgl_ptrs.insert(matched_pair.first);
    }
  }
  line_measurement_matches = line_measurement_matches_pca_filtered;
  /*auto pca_time = std::chrono::high_resolution_clock::now();
  auto pca_duration = std::chrono::duration_cast<std::chrono::microseconds>(pca_time - associated_points_time);
  std::cout << "pca_duration: " << pca_duration.count()*std::pow(10,-6) << "s" << std::endl;*/
}

void CMMap::wall_ground_lines_radius_search(std::vector<ibex::IntervalVector>& bc_world, std::set<WallGroundLine*>& close_wgl_ptrs, std::map<WallGroundLine*,std::vector<size_t>>& line_measurement_matches, double search_radius_factor)
{
  std::map<WallGroundLine*,int> accumulator;
  std::vector<std::pair<size_t,WallGroundLine*>> box_line_matches;
  for(size_t box_counter = 0; box_counter < bc_world.size(); box_counter++)
  {
    //if(bc_world[box_counter][2].mid() > -1.7f)
    {
      pcl::PointXY p;
      p.x = bc_world[box_counter][0].mid();
      p.y = bc_world[box_counter][1].mid();
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      double radius = ibex::norm(bc_world[box_counter].diam().subvector(0,1))/2*search_radius_factor; // REDUCING THE SEARCH-RADIUS!!!!
      wall_ground_kdtree_.radiusSearch (p, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      if(pointIdxRadiusSearch.size() > 0)
      {
         int i = 0; // taking the closest match 
        // get the wall ground line
        WallGroundLine& wgl = wall_ground_lines_[pc_to_wgl_map_[pointIdxRadiusSearch[i]]];
        // extract the line parameters -> n and d
        Eigen::Vector2d n(wgl.line_params(0),wgl.line_params(1));
        double d = wgl.line_params(2);
        // check if bc_world[box_counter] intersects with the line
        ibex::Interval d_test = n(0)*bc_world[box_counter][0]+n(1)*bc_world[box_counter][1];
        if(d_test.lb() <= d && d_test.ub()>=d)
        {
          // non-empty intersection! potential association!
          std::pair<size_t,WallGroundLine*> match_pair;
          match_pair.first = box_counter;
          match_pair.second = &wgl;
          box_line_matches.push_back(match_pair);
          // The accumulator saves for each seen wall-ground-line how many boxes were observed
          if(accumulator.find(&wall_ground_lines_[pc_to_wgl_map_[pointIdxRadiusSearch[i]]]) == accumulator.end())
          {
            accumulator[&wall_ground_lines_[pc_to_wgl_map_[pointIdxRadiusSearch[0]]]] = 1; 
          }
          else
          {
            accumulator[&wall_ground_lines_[pc_to_wgl_map_[pointIdxRadiusSearch[0]]]] += 1; 
          } 
        }
      }
    }
  }

  // lines that were observed with less then the average oberservations are deleted
  float average = 0; 
  for(std::pair<WallGroundLine*,int> pair : accumulator)
  {
    average+=pair.second;
  } 
  average = average/accumulator.size();
  for(int box_line_pair_counter = 0; box_line_pair_counter < box_line_matches.size(); box_line_pair_counter++)
  {
    std::pair<size_t,WallGroundLine*> point_line_pair = box_line_matches[box_line_pair_counter];
    if(accumulator[point_line_pair.second] < average)
    {
      box_line_matches.erase(box_line_matches.begin()+box_line_pair_counter);
      box_line_pair_counter--; 
    }
  }

  // convert the matched data in a convenient form, where a wall line the corresponding points are assigned
  std::map<WallGroundLine*,std::vector<size_t>> line_measurement_matches_all;
  for(std::pair<size_t,WallGroundLine*>& match : box_line_matches)
  {
    line_measurement_matches_all[match.second].push_back(match.first);
  } 

  line_measurement_matches = line_measurement_matches_all;

}

void CMMap::wall_ground_lines_radius_search(ibex::IntervalVector& b_world, int& matches)
{
  pcl::PointXY p;
  p.x = b_world[0].mid();
  p.y = b_world[1].mid();
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  double radius = ibex::norm(b_world.diam())/2;
  wall_ground_kdtree_.radiusSearch (p, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
  matches = pointIdxRadiusSearch.size();
}

void CMMap::wall_ground_lines_radius_search(ibex::IntervalVector& b_world, double& wall_line_uncertainty, int& matches)
{
  pcl::PointXY p;
  p.x = b_world[0].mid();
  p.y = b_world[1].mid();
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  double radius = ibex::norm(b_world.diam())/2 + wall_line_uncertainty;
  wall_ground_kdtree_.radiusSearch (p, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
  matches = pointIdxRadiusSearch.size();
}

void CMMap::buildings_radius_search(ibex::IntervalVector& translation_box, std::vector<size_t>& buildings_idxs)
{
  ibex::Vector mid = translation_box.mid();
  double radius = ibex::norm(translation_box.diam())/2.0;

  pcl::PointXY p;
  p.x = mid[0];
  p.y = mid[1];
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  wall_ground_kdtree_.radiusSearch (p, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

  std::set<size_t> near_buildings_idx; 
  if(pointIdxRadiusSearch.size() > 0)
  {
    for(int i = 0; i < pointIdxRadiusSearch.size(); i++)
    {
      // get the wall ground line
      WallGroundLine& wgl = wall_ground_lines_[pc_to_wgl_map_[pointIdxRadiusSearch[i]]];
      near_buildings_idx.insert(wgl.building_idx);
    }
  }
  else
  {
    // the search-radius is too small, fin nearest building!
    std::vector<int> pointIdxKNSearch;
    std::vector<float> pointKNSquaredDistance;
    wall_ground_kdtree_.nearestKSearch(p, 1, pointIdxKNSearch, pointKNSquaredDistance); 
    if(pointIdxKNSearch.size() > 0)
    {
      for(int i = 0; i < pointIdxKNSearch.size(); i++)
      {
        // get the wall ground line
        WallGroundLine& wgl = wall_ground_lines_[pc_to_wgl_map_[pointIdxKNSearch[i]]];
        near_buildings_idx.insert(wgl.building_idx);
      }
    }
  }

  buildings_idxs.resize(near_buildings_idx.size());
  std::copy(near_buildings_idx.begin(), near_buildings_idx.end(), buildings_idxs.begin());
}

void CMMap::wall_ground_lines_radius_search(Eigen::Vector2d& mid, double& radius, std::set<WallGroundLine*>& close_wgl_ptrs)
{
  close_wgl_ptrs.clear(); 
  pcl::PointXY p;
  p.x = mid(0);
  p.y = mid(1);
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  connected_wall_ground_kdtree_.radiusSearch (p, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
  for(int i = 0; i < pointRadiusSquaredDistance.size(); ++i)
  {
    close_wgl_ptrs.insert(&connected_wall_ground_lines_[connected_pc_to_wgl_map_[pointIdxRadiusSearch[i]]]); 
  }
}

void CMMap::wall_ground_lines_radius_search(pcl::PointCloud<pcl::PointXY>& pc, double& max_line_distance, std::map<WallGroundLine*,std::vector<size_t>>& line_measurement_matches)
{
  line_measurement_matches.clear();
  for(int point_counter = 0; point_counter < pc.size(); point_counter++) // take every second point!
  {
    pcl::PointXY& p = pc[point_counter];
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    wall_ground_kdtree_.nearestKSearch (p,1, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    if(pointIdxRadiusSearch.size() > 0)
    {
      //close_wgl_ptrs.insert(&wall_ground_lines_[pc_to_wgl_map_[pointIdxRadiusSearch[0]]]);
      WallGroundLine& wgl = wall_ground_lines_[pc_to_wgl_map_[pointIdxRadiusSearch[0]]];
      Eigen::Vector2d p_eig((double)p.x, (double)p.y);
      Eigen::Vector2d n(wgl.line_params(0),wgl.line_params(1));
      double d = wgl.line_params(2);
      double distance = p_eig.transpose()*n-d;
      if(abs(distance) <= max_line_distance)
      {
        line_measurement_matches[&wgl].push_back(point_counter);    
      }
    }
  }
}

void CMMap::connected_wall_ground_lines_radius_search(pcl::PointCloud<pcl::PointXY>& pc, double& max_line_distance, std::map<WallGroundLine*,std::vector<size_t>>& line_measurement_matches)
{
  line_measurement_matches.clear();
  for(int point_counter = 0; point_counter < pc.size(); point_counter++) // take every second point!
  {
    pcl::PointXY& p = pc[point_counter];
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    connected_wall_ground_kdtree_.nearestKSearch (p,1, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    if(pointIdxRadiusSearch.size() > 0)
    {
      WallGroundLine& wgl = connected_wall_ground_lines_[connected_pc_to_wgl_map_[pointIdxRadiusSearch[0]]];
      Eigen::Vector2d p_eig((double)p.x, (double)p.y);
      Eigen::Vector2d n(wgl.line_params(0),wgl.line_params(1));
      double d = wgl.line_params(2);
      double distance = p_eig.transpose()*n-d;
      if(abs(distance) <= max_line_distance)
      {
        line_measurement_matches[&wgl].push_back(point_counter);    
      }
    }
  }
}

void CMMap::compute_convex_hull()
{
  std::vector<cv::Point2f> cv_points;
  for(WallGroundLine& wgl : wall_ground_lines_)
  {
    Eigen::Vector2d& s = wgl.start_point;
    cv::Point2f cv_s;
    cv_s.x = (float)s.x();
    cv_s.y = (float)s.y();
    cv_points.push_back(cv_s);
    Eigen::Vector2d& e = wgl.end_point;
    cv::Point2f cv_e;
    cv_e.x = (float)e.x();
    cv_e.y = (float)e.y();
    cv_points.push_back(cv_e);
  }
  cv_points_convex_hull_map_.clear();
  cv::convexHull(cv_points,cv_points_convex_hull_map_);
}