#include "hypascore_localization/contractors/CtcRangeBearingNoCross.hpp"

namespace codac
{
  CtcRangeBearingNoCross::CtcRangeBearingNoCross() : Ctc(9)
  {}

  void CtcRangeBearingNoCross::contract(ibex::IntervalVector& x)
  {
    ibex::IntervalVector pose(3),a(2),b(2); 
    ibex::Interval range, bearing; 
    pose = x.subvector(0,2);
    range = x[3];
    bearing = x[4];
    a = x.subvector(5,6);
    b = x.subvector(7,8);
    contract(pose, range, bearing, a, b);
    x[0] = x[0] & pose[0];
    x[1] = x[1] & pose[1];
    x[2] = x[2] & pose[2];
    x[3] = x[3] & range;
    x[4] = x[4] & bearing;
    x[5] = x[5] & a[0];
    x[6] = x[6] & a[1];
    x[7] = x[7] & b[0];
    x[8] = x[8] & b[1];
  }

  void CtcRangeBearingNoCross::contract(ibex::IntervalVector& pose, ibex::Interval& range, ibex::Interval& bearing, ibex::IntervalVector& a, ibex::IntervalVector& b)
  {
    /**
    * pose = ([t_x],[t_y],[psi]) -> pose in the map
    * range = [d] -> Range of the measurement
    * bearing = [beta] -> bearing of the measurement
    * a = ([x],[y]) -> start of the line-segment in map frame
    * b = ([x],[y]) -> end of the line-segment in map frame 
    */

    // Forward
    ibex::Interval alpha = pose[2] + bearing; 
    ibex::IntervalVector pm(2), p_t(2); 
    ctc_polar.contract(pm[0],pm[1],range,alpha);
    p_t[0] = pose[0]; p_t[1] = pose[1];
    ctc_nocross.contract(p_t,pm,a,b);

    // Backward
    pose[0] = pose[0] & p_t[0]; pose[1] = pose[1] & p_t[1];
    ctc_polar.contract(pm[0],pm[1],range,alpha);
    ibex::bwd_add(alpha,pose[2],bearing);

    if(pose[0].is_empty()||pose[1].is_empty()||pose[2].is_empty()||range.is_empty()||bearing.is_empty()||a.is_empty()||b.is_empty())
    {
      pose.set_empty(); 
      range.set_empty();
      bearing.set_empty(); 
      a.set_empty(); 
      b.set_empty(); 
    }
  }

}