#include "hypascore_localization/contractors/CtcRange.hpp"

namespace codac
{
  CtcRange::CtcRange() : Ctc(6)
  {}

  void CtcRange::contract(ibex::IntervalVector& x)
  {
    ibex::IntervalVector b(2),c(2);
    ibex::Interval r; 
    b = x.subvector(0,1);
    c = x.subvector(2,3);
    r = x[4];
    contract(b, c, r);
    x[0] = x[0] & b[0];
    x[1] = x[1] & b[1];
    x[2] = x[2] & c[0];
    x[3] = x[3] & c[1];
    x[4] = x[4] & r;
  }

  void CtcRange::contract(ibex::IntervalVector& b, ibex::IntervalVector& c, ibex::Interval& r)
  {
    if(!b.is_empty())
    {
      // fwd
      ibex::Interval dx = b[0]-c[0];
      ibex::Interval dy = b[1]-c[1]; 
      ibex::Interval r2 = sqr(r);
      ibex::Interval dx2 = sqr(dx);
      ibex::Interval dy2 = sqr(dy);
      ibex::Interval d2 = dx2+dy2; 
      d2 = d2 & r2; 

      // bwd
      ibex::bwd_add(d2, dx2, dy2);
      ibex::bwd_sqr(dy2,dy);
      ibex::bwd_sqr(dx2,dx);
      ibex::bwd_sub(dy, b[1], c[1]);
      ibex::bwd_sub(dx, b[0], c[0]);
    }
  }
}