#include "thread"
#include <random>
#include "opencv2/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include "ibex.h"
#include <chrono>
#include "ros/ros.h"

class HoughAccumulator
{
  public:
  HoughAccumulator(ros::NodeHandle& nh);
  void set_accumulator(double angle_min, double angle_max, double angle_quantization, double d_min, double d_max, double d_quantization);
  void fill_data(std::vector<size_t>& measurement_idxs, std::unique_ptr<std::vector<ibex::IntervalVector>>& measurements_ptr);
  bool process_accumulator(ibex::Interval& angle, ibex::Interval& d, std::set<size_t>& considered_set);

  private:
  std::vector<std::vector<std::vector<size_t>>> acc;
  double angle_min_;
  double angle_quantization_;
  double angle_max_;
  double d_min_;
  double d_quantization_;
  double d_max_;
  double angle_bins_, d_bins_; 
  ibex::Interval d_init_;
  double largest_accumulator_value_ = 0;
  double largest_accumulator_value_factor_ = 0.98;
  double significance_factor_min_ = 2; 
};