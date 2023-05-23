#include "hypascore_localization/localization/coarse/IntervalPose.hpp"
#include "hypascore_localization/contractors/CtcRaster.hpp"
#include <queue>

IntervalPose::IntervalPose(double min_diam_length, double& x_odom_translation_uncertainty_radius, double& y_odom_translation_uncertainty_radius, double& odom_rotation_uncertainty_radius, double& max_point_line_association_distance)
{
  min_diam_length_ = 0.3*min_diam_length; 
  x_odom_translation_uncertainty_radius_ = x_odom_translation_uncertainty_radius;
  y_odom_translation_uncertainty_radius_ = y_odom_translation_uncertainty_radius;
  odom_rotation_uncertainty_radius_ = odom_rotation_uncertainty_radius;
  translation_hull = ibex::IntervalVector(2);
  max_line_distance_ = max_point_line_association_distance;
}

void IntervalPose::add_odometry(Eigen::Affine2d* laser_bef_T_laser_cur)
{
  ibex::Interval psi_odom(atan2(laser_bef_T_laser_cur->matrix()(1,0),laser_bef_T_laser_cur->matrix()(0,0)));
  psi_odom.inflate(odom_rotation_uncertainty_radius_); // 0.0001° rotation uncertainty
  ibex::Interval psi = rotation; 
  ibex::IntervalVector t_odom(2);
  t_odom[0] = ibex::Interval(laser_bef_T_laser_cur->translation()(0));
  t_odom[1] = ibex::Interval(laser_bef_T_laser_cur->translation()(1));
  t_odom[0].inflate(x_odom_translation_uncertainty_radius_); // 0.005m translation uncertainty; 
  t_odom[1].inflate(y_odom_translation_uncertainty_radius_);
  ibex::IntervalVector t_odom_rot(2);
  t_odom_rot[0] = cos(psi)*t_odom[0]-sin(psi)*t_odom[1];
  t_odom_rot[1] = sin(psi)*t_odom[0]+cos(psi)*t_odom[1];

  double particle_odom_psi = atan2(laser_bef_T_laser_cur->rotation()(1,0),laser_bef_T_laser_cur->rotation()(0,0));
  double particle_odom_tx =  laser_bef_T_laser_cur->translation()(0);
  double particle_odom_ty =  laser_bef_T_laser_cur->translation()(1);
  //thread_local std::random_device rd;
  //std::mt19937 gen(rd());
  //std::default_random_engine gen(rd());
  thread_local std::default_random_engine gen;
  std::uniform_real_distribution<> dis_odom_psi(particle_odom_psi-odom_rotation_uncertainty_radius_, particle_odom_psi+odom_rotation_uncertainty_radius_);
  std::uniform_real_distribution<> dis_odom_tx(particle_odom_tx-x_odom_translation_uncertainty_radius_, particle_odom_tx+x_odom_translation_uncertainty_radius_);
  std::uniform_real_distribution<> dis_odom_ty(particle_odom_ty-y_odom_translation_uncertainty_radius_, particle_odom_ty+y_odom_translation_uncertainty_radius_);
  translation_area_ = 0; 
  if(translation.size() > 0)
  {
    translation_hull.set_empty();
    for(int box_counter = 0; box_counter < translation.size(); box_counter++)
    {
      ibex::IntervalVector& t = translation[box_counter];
      t = t_odom_rot + t;  
      translation_hull = translation_hull | t;
      translation_area_ += t.volume();
    }
    
    rotation = psi_odom+psi; 
    if(rotation.lb() > M_PI)
    {
      rotation = rotation-2*M_PI;
    }
    else if(rotation.ub() < -M_PI)
    {
      rotation = rotation+2*M_PI;
    }
    
    // update particles
    for(int p_counter = 0; p_counter < particles_.size(); ++p_counter)
    {
      double tx_sample = dis_odom_tx(gen);
      double ty_sample = dis_odom_ty(gen);
      double psi_sample = dis_odom_psi(gen);
      Eigen::Matrix2d R;
      R(0,0) = cos(psi_sample); R(0,1) = -sin(psi_sample);
      R(1,0) = sin(psi_sample); R(1,1) = cos(psi_sample);
      Eigen::Affine2d cur_T_next_sample; 
      cur_T_next_sample.linear() = R;  
      cur_T_next_sample.translation() = Eigen::Vector2d(tx_sample,ty_sample);

      particles_[p_counter].pose = particles_[p_counter].pose*cur_T_next_sample;
      particles_[p_counter].age++; 
    }
  }
}

void IntervalPose::delete_outside_circle(Eigen::Vector3d& center, double& radius)
{
  // use the CtcRange here for each translation component!
  codac::CtcRange ctc_range; 
  translation_area_ = 0;
  if(translation.size() > 0)
  {
    translation_hull.set_empty();
    for(int box_counter = 0; box_counter < translation.size(); box_counter++)
    {
      ibex::IntervalVector& t = translation[box_counter]; 
      ibex::IntervalVector c(2); 
      c[0] = center(0); c[1] = center(1); 
      ibex::Interval r(0,radius); 
      ctc_range.contract(t,c,r);
      if(t.is_empty())
      {
        // translation box NOT inside gps-circle
        translation.erase(translation.begin()+box_counter); // delete the subpaving 
        box_counter--; 
      }
      else
      {
        // translation box inside gps-circle
        translation_hull = translation_hull | t;
        translation_area_ += t.volume();
      }
    } 
  }
}

void IntervalPose::check_reordering()
{
  if(translation.size() > 0)
  { 
    create_images(); 
    integral_overlap_im_ = cv::Mat::zeros(pos_im.size(), CV_64F);
    integral_im_ = cv::Mat::zeros(pos_im.size(), CV_64F);
    cv::integral(pos_im,integral_im_);
    cv::integral(overlap_pos_im,integral_overlap_im_);
    double* pos_im_sum = integral_im_.ptr<double>(integral_im_.rows - 1) + integral_im_.cols - 1;
    double* overlap_pos_im_sum = integral_overlap_im_.ptr<double>(integral_overlap_im_.rows - 1) + integral_overlap_im_.cols - 1;
    if((*pos_im_sum)/(*overlap_pos_im_sum) <= 0.6)
    {
      reorder_subpavings();
    }
  }
}

void IntervalPose::create_images()
{
  ibex::Vector diams = translation_hull.diam();
  ibex::Vector im_size = 1/min_diam_length_*diams;
  pos_im = cv::Mat::zeros(cv::Size((int)(std::ceil(im_size[0])), (int)(std::ceil(im_size[1]))), CV_64F); // rounding error!
  pos_im.copyTo(overlap_pos_im); 
  ibex::Vector pos_im_origin_trans(2);
  pos_im_origin_trans[0] = translation_hull[0].lb();
  pos_im_origin_trans[1] = translation_hull[1].ub();
  for(int trans_counter = 0; trans_counter < translation.size(); trans_counter++)
  {
    ibex::IntervalVector& t = translation[trans_counter];
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
    cv::Mat overlap_dst_roi = overlap_pos_im(cv::Rect(start_col, start_row, local_im.cols, local_im.rows));
    overlap_dst_roi = overlap_dst_roi+local_im; 
  } 
}

void IntervalPose::reorder_subpavings()
{
  ibex::Vector diams = translation_hull.diam();
  ibex::Vector pos_im_origin_trans(2);
  pos_im_origin_trans[0] = translation_hull[0].lb();
  pos_im_origin_trans[1] = translation_hull[1].ub();
  /*cv::Mat normalized; 
  cv::normalize(pos_im,normalized,INT32_MAX,cv::NORM_MINMAX);
  cv::imshow("Display", normalized);
  cv::waitKey(0);*/
  //integral_im = cv::Mat(cv::Size((int)(im_size[0]+1), (int)(im_size[1]+1)), CV_64F);
  CtcRaster ctc_raster(&pos_im, &integral_im_);

  // convert translation_hull to pixel
  ibex::IntervalVector translation_hull_pixel(2);
  translation_hull_pixel[0] = ibex::Interval(0,pos_im.cols);
  translation_hull_pixel[1] = ibex::Interval(0,pos_im.rows);

  std::queue<ibex::IntervalVector> Queue;
  Queue.push(translation_hull_pixel);
  std::vector<ibex::IntervalVector> intermediate_result;  
  intermediate_result.clear(); 
  ibex::IntervalVector internal_box(2);
  while(!Queue.empty())
  {
    internal_box = Queue.front();
    Queue.pop();
    CtcRaster::sivia_action action;
    ctc_raster.contract(internal_box,action);
    if(action == CtcRaster::sivia_action::bisect)
    {
      if(internal_box.min_diam() >= min_diam_length_)
      {
        int idx = internal_box.extr_diam_index(false);
        std::pair<ibex::IntervalVector,ibex::IntervalVector> bisected = internal_box.bisect(idx);
        ibex::IntervalVector left = bisected.first;
        left[idx] = ibex::Interval(left[idx].lb(), std::floor(left[idx].ub())); 
        ibex::IntervalVector right = bisected.second;
        right[idx] = ibex::Interval(std::floor(right[idx].lb()), right[idx].ub());
        Queue.push(left);
        Queue.push(right);
      }
    }
    else if(action == CtcRaster::sivia_action::include)
    {
      intermediate_result.push_back(internal_box);
    }
  } 

  // intermediate_result overestimates translation
  translation.clear(); 
  translation.reserve(intermediate_result.size()); 
  for(int result_counter = 0; result_counter < intermediate_result.size(); result_counter++)
  {
    ibex::IntervalVector& part_result_pixel = intermediate_result[result_counter];
    ibex::IntervalVector new_translation(2); 
    new_translation[0] = part_result_pixel[0]*min_diam_length_+pos_im_origin_trans[0];
    new_translation[1] = -part_result_pixel[1]*min_diam_length_+pos_im_origin_trans[1];
    translation.emplace_back(new_translation); 
  }
}

void IntervalPose::particle_filter(CMMap* map, pcl::PointCloud<pcl::PointXY>::Ptr& laser_pc)
{
  // 1.) delete particles outside the feasible set
  particles_delete_outside_feasible_set();  
  
  // 2.) generate new particles randomly in the feasible set 
  Eigen::Vector2d pos_im_origin_trans;
  pos_im_origin_trans(0) = translation_hull[0].lb();
  pos_im_origin_trans(1) = translation_hull[1].ub();
  std::vector<cv::Point2i> locations;   // output, locations of non-zero pixels
  cv::Mat pos_im_char(pos_im.size(),CV_8UC1);
  pos_im.convertTo(pos_im_char,CV_8UC1); 
  cv::findNonZero(pos_im_char, locations);
  //thread_local std::random_device rd;
  //std::mt19937 gen(rd());
  //std::default_random_engine gen(rd());
  thread_local std::default_random_engine gen;
  std::uniform_int_distribution<> distrib(0, (int)locations.size()-1);
  std::uniform_real_distribution<> dis_odom_psi(rotation.lb(), rotation.ub());
  while(particles_.size() < max_particles_ && locations.size() > 0)
  {
    PoseParticle p; 
    int idx = distrib(gen);
    cv::Point2i& pixel = locations[idx];
    int& col = pixel.x;
    int& row = pixel.y;
    Eigen::Vector2d new_trans;
    new_trans(0) = (double)col*min_diam_length_+pos_im_origin_trans(0);
    new_trans(1) = -(double)row*min_diam_length_+pos_im_origin_trans(1);
    double psi_sample = dis_odom_psi(gen);
    Eigen::Matrix2d R;
    R(0,0) = cos(psi_sample); R(0,1) = -sin(psi_sample);
    R(1,0) = sin(psi_sample); R(1,1) = cos(psi_sample);
    Eigen::Affine2d cur_T_next_sample; 
    p.pose.linear() = R;  
    p.pose.translation() = new_trans;
    particles_.push_back(p); 
  }

  // 3.) Weight the particles! NO normalization!
  // transform the points 
  pcl::PointCloud<pcl::PointXY>::Ptr map_pc;
  max_weight_ = 0; 
  sum_weight_ = 0; 
  for(int p_counter = 0; p_counter < (int)particles_.size(); ++p_counter)
  {
    PoseParticle& particle = particles_[p_counter];
    Eigen::Affine2d& pose = particle.pose;
    map_pc.reset(new pcl::PointCloud<pcl::PointXY>);
    map_pc->resize(laser_pc->size());
    for(int i = 0; i < laser_pc->size(); i++)
    {
      pcl::PointXY& p_pcl = (*laser_pc)[i];
      Eigen::Vector2d laser_p((double)p_pcl.x, (double)p_pcl.y); 
      Eigen::Vector2d map_p = pose*laser_p;
      (*map_pc)[i].x = map_p[0];
      (*map_pc)[i].y = map_p[1];
    }

    // get close facades to the point
    map->connected_wall_ground_lines_radius_search(*map_pc, max_line_distance_, particle.line_measurement_matches);

    // if point close to facade, score up! 
    particle.weight_ = 0;  
    Eigen::Vector2d p_eig, n;
    double d, abs_distance;
    for(std::pair<WallGroundLine*,std::vector<size_t>> matches : particle.line_measurement_matches)
    {
      WallGroundLine& wgl = *(matches.first);
      for(size_t& p_idx : matches.second)
      {
        p_eig(0) = (double)(*map_pc)[p_idx].x;
        p_eig(1) = (double)(*map_pc)[p_idx].y;
        n(0) = wgl.line_params(0);
        n(1) = wgl.line_params(1);
        d = wgl.line_params(2);
        abs_distance = abs(p_eig.transpose()*n-d);
        particle.weight_ += 1.0-1.0/(1.0+exp(-7.0*1.5/max_line_distance_*(abs_distance-max_line_distance_/1.5)));
      }
    }
    particle.weight_sum_ += particle.weight_;
    if(particle.age > 0)
    {
      double average_weight = particle.weight_sum_/(double)particle.age;
      if(particle.weight_ < average_weight)
      {
        particle.weight_ = average_weight;
      }
    }
    if(particle.age > 20)
    {
      particle.weight_ = 2.0*particle.weight_;
    }
    
    sum_weight_ += particle.weight_; 
    if(max_weight_ < particle.weight_)
    {
      max_weight_ = particle.weight_;
    }
  } 
}

void IntervalPose::particles_normalize(double& norm_factor)
{
  sum_weight_ = sum_weight_*norm_factor; 
  max_weight_ = max_weight_*norm_factor; 
  for(PoseParticle& particle : particles_)
  {
    particle.weight_ = particle.weight_*norm_factor;
  }
}

void IntervalPose::particles_low_weight_resampling(double& min_weight)
{
  // build a vector with according number of samples for a particles 
  int number_of_particle_samples = 0;
  for(PoseParticle& p : particles_)
  {
    if(p.weight_ >= min_weight)
    {
      number_of_particle_samples += (int)std::round(max_weight_occurance_/max_weight_*p.weight_);
    }
  }
  std::vector<PoseParticle> sampling_vector;
  sampling_vector.resize(number_of_particle_samples); 
  int index_counter = 0; 
  for(int i = 0; i < particles_.size(); i++)
  {
    if(particles_[i].weight_ >= min_weight)
    {
      for(int place_counter = 0; place_counter < (int)std::round(max_weight_occurance_/max_weight_*particles_[i].weight_); place_counter++)
      {
        sampling_vector[index_counter] = particles_[i]; 
        index_counter++; 
      } 
    }
  }
  // sample from sampling_vector, but not exactly the same particle, but close to it!
  if(sampling_vector.size()>0)
  {
    //thread_local std::random_device rd;
    //std::mt19937 gen(rd());
    //std::default_random_engine gen(rd());
    thread_local std::default_random_engine gen;
    std::uniform_int_distribution<> distrib(0, sampling_vector.size()-1);
    std::uniform_real_distribution<> dis_psi(rotation.lb(), rotation.ub());
    std::uniform_real_distribution<> dis_trans(-max_translation_diff_resampling, max_translation_diff_resampling);
    for(int p_counter = 0; p_counter < particles_.size(); ++p_counter)
    {
      PoseParticle& particle = particles_[p_counter];
      if(particle.weight_ < min_weight)
      {
        // this particle has to be replaced by resampling
        int idx = distrib(gen);
        PoseParticle sampled_particle = sampling_vector[idx];
        Eigen::Vector2d new_trans;
        new_trans(0) = sampled_particle.pose.translation()(0)+dis_trans(gen);
        new_trans(1) = sampled_particle.pose.translation()(1)+dis_trans(gen);
        double psi_sample = dis_psi(gen);
        Eigen::Matrix2d R;
        R(0,0) = cos(psi_sample); R(0,1) = -sin(psi_sample);
        R(1,0) = sin(psi_sample); R(1,1) = cos(psi_sample);
        particle.pose.linear() = R;  
        particle.pose.translation() = new_trans;
        particle.weight_ = sampled_particle.weight_;
        particle.weight_sum_ = sampled_particle.weight_sum_;
        particle.age = sampled_particle.age;
      }
    }
  }
  else
  {
    particles_.clear();
  }
  particles_delete_outside_feasible_set();  
}

void IntervalPose::particles_delete_outside_feasible_set()
{
  // delete particles outside the feasible set
  Eigen::Vector2d pos_im_origin_trans;
  pos_im_origin_trans(0) = translation_hull[0].lb();
  pos_im_origin_trans(1) = translation_hull[1].ub();  
  for(int p_counter = 0; p_counter < particles_.size(); ++p_counter)
  {
    PoseParticle p = particles_[p_counter];
    Eigen::Vector2d local_im_origin_trans = p.pose.translation(); 
    Eigen::Vector2d diff = local_im_origin_trans-pos_im_origin_trans;
    int col = (int) std::round(1/min_diam_length_*diff(0));
    int row = (int) std::round(-1/min_diam_length_*diff(1));
    double cell_value = 0; 
    if(col >= 0 && col <= pos_im.size().width-1 && row >= 0 && row <= pos_im.size().height-1)
    {
      cell_value = pos_im.at<double>(row,col);
    }
    double psi(atan2(p.pose.matrix()(1,0),p.pose.matrix()(0,0)));
    if(cell_value < 0.5 || !rotation.contains(psi))
    {
      particles_.erase(particles_.begin()+p_counter);
      p_counter--;  
    }
  }
}