#include "hypascore_localization/localization/refined/HoughAccumulator.hpp"

HoughAccumulator::HoughAccumulator(ros::NodeHandle& nh)
{
  nh.getParam("/hypascore_localization_node/largest_accumulator_value_factor", largest_accumulator_value_factor_);
  nh.getParam("/hypascore_localization_node/significance_factor_min", significance_factor_min_);
}

void HoughAccumulator::set_accumulator(double angle_min, double angle_max, double angle_quantization, double d_min, double d_max, double d_quantization)
{
  angle_min_ = angle_min;
  angle_max_ = angle_max;
  angle_quantization_ = angle_quantization;
  d_min_ = d_min;
  d_max_ = d_max;
  d_init_ = ibex::Interval(d_min_,d_max_);
  d_quantization_ = d_quantization;
  angle_bins_ = std::ceil((angle_max-angle_min)/angle_quantization);
  d_bins_ = std::ceil((d_max-d_min)/d_quantization);
  acc.clear(); 
  acc.resize(angle_bins_);
  for(std::vector<std::vector<size_t>>& d_bins_data : acc)
  {
    d_bins_data.resize(d_bins_);
  }
}

void HoughAccumulator::fill_data(std::vector<size_t>& measurement_idxs, std::unique_ptr<std::vector<ibex::IntervalVector>>& measurements_ptr)
{
  largest_accumulator_value_ = 0;
  #pragma omp parallel for
  for(int angle_idx = 0; angle_idx < (int)angle_bins_; angle_idx++)
  {
    ibex::Interval angle(angle_min_+(double)angle_idx*angle_quantization_,angle_min_+(double)(angle_idx+1)*angle_quantization_); 
    //std::cout << 180/M_PI*angle << std::endl; 
    for(size_t idx : measurement_idxs)
    {
      ibex::IntervalVector& box = (*measurements_ptr)[idx];
      ibex::Interval d = box[0]*cos(angle)+box[1]*sin(angle);
      ibex::Interval d_intersect = d & d_init_;
      if(!d_intersect.is_empty())
      {
        int d_idx_lb = (int)std::floor((d_intersect.lb()-d_min_)/d_quantization_);
        int d_idx_ub = acc[angle_idx].size()-1 - (int)std::floor((d_max_- d_intersect.ub())/d_quantization_);
        for(int d_idx = d_idx_lb; d_idx <= d_idx_ub; d_idx++)
        {
          acc[angle_idx][d_idx].push_back(idx);
        }
      }
    }
  }

  for(int angle_idx = 0; angle_idx < angle_bins_; angle_idx++)
  {
    for(int d_idx = 0; d_idx < d_bins_; d_idx++)
    {
      if(acc[angle_idx][d_idx].size() > largest_accumulator_value_)
      {
        largest_accumulator_value_ = (double)acc[angle_idx][d_idx].size();
      }
    }
  }
}

bool HoughAccumulator::process_accumulator(ibex::Interval& angle_out, ibex::Interval& d_out, std::set<size_t>& considered_set)
{ 
  considered_set.clear();
  ibex::Interval d; d.set_empty();
  ibex::Interval angle; angle.set_empty();
  double non_zero_value_sum = 0; 
  double non_zero_value_positions = 0;
  for(int row = 0; row < angle_bins_; row++)
  {
    for(int col = 0; col < d_bins_; col++)
    {
      if(acc[row][col].size()>0)
      {
        non_zero_value_sum+=(acc[row][col].size());
        non_zero_value_positions++;
      }
      
      if(acc[row][col].size() >= largest_accumulator_value_*largest_accumulator_value_factor_)
      {
        double angle_lb = angle_min_+(double)row*angle_quantization_;
        double angle_ub = angle_min_+(double)(row+1)*angle_quantization_;
        double d_lb = d_min_+(double)col*d_quantization_;
        double d_ub = d_min_+(double)(col+1)*d_quantization_;
        ibex::Interval local_angle(angle_lb,angle_ub);
        ibex::Interval local_d(d_lb,d_ub);
        angle = angle|local_angle;
        d = d|local_d;
        for(size_t box_idx : acc[row][col])
        {
          considered_set.insert(box_idx);
        }
      }
    }
  }
  double significance_factor = largest_accumulator_value_*largest_accumulator_value_factor_*1/(non_zero_value_sum/non_zero_value_positions);

  angle_out = angle;
  d_out = d;  
  if(significance_factor >= significance_factor_min_) 
  {
    return true;
  }
  else
  {
    return false;
  }
}