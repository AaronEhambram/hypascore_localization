#include "hypascore_localization/contractors/CtcRaster.hpp"
#include "cmath"

CtcRaster::CtcRaster(cv::Mat* im, cv::Mat* integral_im)
{
  im_ = *im;
  if(integral_im == NULL)
  {
    integral_im_ = cv::Mat::zeros(im_.size(), CV_64F);
    cv::integral(im_,integral_im_); 
  }
  else
  {
    integral_im_ = *integral_im; 
  }
}

void CtcRaster::contract(ibex::IntervalVector& box, sivia_action& action)
{
  int n_pix = enclosing_white_pixels(box); 
  
  if(box.is_empty())
  {
    action = ignore; 
    return;
  }
  if(n_pix == 0)
  {
    action = ignore; 
    box.set_empty(); 
    return;
  }
  else
  {
    if(n_pix != box.diam()[0]*box.diam()[1])
    {
      set_maximal_lower_border(box, 0);
      set_maximal_lower_border(box, 1);
      set_minimal_upper_border(box, 0);
      set_minimal_upper_border(box, 1);
    }
    if(n_pix == box.diam()[0]*box.diam()[1])
    {
      action = include;
    }
    else
    {
      action = bisect;
    }  
  }
}

int CtcRaster::enclosing_white_pixels(ibex::IntervalVector& box)
{
  return (int) (integral_im_.at<double>(box[1].ub(),box[0].ub())
  - integral_im_.at<double>(box[1].lb(),box[0].ub())
  - integral_im_.at<double>(box[1].ub(),box[0].lb())
  + integral_im_.at<double>(box[1].lb(),box[0].lb())); 
}

void CtcRaster::set_maximal_lower_border(ibex::IntervalVector& box, int idx)
{
  ibex::IntervalVector internal_box = box;
  while(internal_box[idx].ub() - internal_box[idx].lb() > 1)
  {
    std::pair<ibex::IntervalVector,ibex::IntervalVector> bisected = internal_box.bisect(idx);
    ibex::IntervalVector left = bisected.first;
    left[idx] = ibex::Interval(left[idx].lb(), std::floor(left[idx].ub())); 
    ibex::IntervalVector right = bisected.second;
    right[idx] = ibex::Interval(std::floor(right[idx].lb()), right[idx].ub()); 
    int n_pix_left = enclosing_white_pixels(left);
    int n_pix_right = enclosing_white_pixels(right);
    if(n_pix_left == 0)
    {
      internal_box = right;
    }
    else
    {
      internal_box = left;
    }
  }
  box[idx] = ibex::Interval(internal_box[idx].lb(),box[idx].ub());
}

void CtcRaster::set_minimal_upper_border(ibex::IntervalVector& box, int idx)
{
  ibex::IntervalVector internal_box = box;
  while(internal_box[idx].ub() - internal_box[idx].lb() > 1)
  {
    std::pair<ibex::IntervalVector,ibex::IntervalVector> bisected = internal_box.bisect(idx);
    ibex::IntervalVector left = bisected.first;
    left[idx] = ibex::Interval(left[idx].lb(), std::floor(left[idx].ub())); 
    ibex::IntervalVector right = bisected.second;
    right[idx] = ibex::Interval(std::floor(right[idx].lb()), right[idx].ub()); 
    int n_pix_left = enclosing_white_pixels(left);
    int n_pix_right = enclosing_white_pixels(right);
    if(n_pix_right == 0)
    {
      internal_box = left;
    }
    else
    {
      internal_box = right; 
    }
  }
  box[idx] = ibex::Interval(box[idx].lb(),internal_box[idx].ub());
}