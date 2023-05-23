#ifndef CTCRASTER_H_
#define CTCRASTER_H_

#include "ibex.h"
#include <opencv2/imgproc/imgproc.hpp>

class CtcRaster
{
  public:
  CtcRaster(cv::Mat* im, cv::Mat* integral_im = NULL);
  enum sivia_action { bisect, include, ignore };
  void contract(ibex::IntervalVector& box, sivia_action& action); 

  private:
  cv::Mat im_;
  cv::Mat integral_im_; 
  int enclosing_white_pixels(ibex::IntervalVector& box); 
  void set_maximal_lower_border(ibex::IntervalVector& box, int idx);
  void set_minimal_upper_border(ibex::IntervalVector& box, int idx);
};

#endif