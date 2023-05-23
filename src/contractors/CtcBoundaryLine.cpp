#include "hypascore_localization/contractors/CtcBoundaryLine.hpp"
#include "cmath"

namespace codac
{
  CtcBoundaryLine::CtcBoundaryLine() : Ctc(6)
  {}

  void CtcBoundaryLine::contract(ibex::IntervalVector& x)
  {
    ibex::IntervalVector p(2),a(2),b(2); 
    p = x.subvector(0,1);
    a = x.subvector(2,3);
    b = x.subvector(4,5);
    contract(p, a, b);
    x[0] = x[0] & p[0];
    x[1] = x[1] & p[1];
    x[2] = x[2] & a[0];
    x[3] = x[3] & a[1];
    x[4] = x[4] & b[0];
    x[5] = x[5] & b[1];
  }

  void CtcBoundaryLine::contract(ibex::IntervalVector& box, ibex::IntervalVector& a, ibex::IntervalVector& b)
  {
    ibex::IntervalVector hull = a|b; 
    box = box & hull; 
    if(!box.is_empty())
    {
      ibex::IntervalVector ab = b-a;
      ibex::IntervalVector boxa = a-box;
      ibex::Interval det = 0;
      bwd_det(det, ab, boxa);
      ibex::bwd_sub(ab,b,a);
      ibex::bwd_sub(boxa,a,box);
    }
  }

  void CtcBoundaryLine::fwd_det(ibex::Interval& det, ibex::IntervalVector& u, ibex::IntervalVector& v)
  {
    // forward
    det = det & (u[0]*v[1] - v[0]*u[1]);
  }

  void CtcBoundaryLine::bwd_det(ibex::Interval& det, ibex::IntervalVector& u, ibex::IntervalVector& v)
  {
    ibex::Interval z1 = u[0]*v[1];
    ibex::Interval z2 = v[0]*u[1];
    // backward
    ibex::bwd_sub(det,z1,z2);
    ibex::bwd_mul(z2,v[0],u[1]);
    ibex::bwd_mul(z1,u[0],v[1]);
  }
}