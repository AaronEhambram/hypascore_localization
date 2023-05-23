#include "hypascore_localization/contractors/CtcNoCross.hpp"
#include "cmath"

namespace codac
{
  CtcNoCross::CtcNoCross() : Ctc(8)
  {}

  void CtcNoCross::contract(ibex::IntervalVector& x)
  {
    ibex::IntervalVector p(2),pm(2),a(2),b(2); 
    p = x.subvector(0,1);
    pm = x.subvector(2,3);
    a = x.subvector(4,5);
    b = x.subvector(6,7);
    contract(p, pm, a, b);
    x[0] = x[0] & p[0];
    x[1] = x[1] & p[1];
    x[2] = x[2] & pm[0];
    x[3] = x[3] & pm[1];
    x[4] = x[4] & a[0];
    x[5] = x[5] & a[1];
    x[6] = x[6] & b[0];
    x[7] = x[7] & b[1];
  }

  void CtcNoCross::contract(ibex::IntervalVector& p, ibex::IntervalVector& pm, ibex::IntervalVector& a, ibex::IntervalVector& b)
  {
    /**
    * p = ([t_x],[t_y]) -> position in the map frame
    * pm = ([x],[y]) -> position in the map frame
    * a = ([x],[y]) -> start of the line-segment in map frame
    * b = ([x],[y]) -> end of the line-segment in map frame 
    */

    // hull-test - simple and fast
    ibex::IntervalVector measurement = p | p+pm;
    ibex::IntervalVector line = a | b; 
    if((measurement & line).is_empty())
    {
      return; 
    }

    // only continue, if the hulls intersect

    /***FORWARD***/

    // 1.) compute m
    ibex::IntervalVector m = p+pm; 

    // 2.) compute orientation vectors
    ibex::IntervalVector ab = b-a;
    ibex::IntervalVector am = m-a;
    ibex::IntervalVector ap = p-a;
    ibex::IntervalVector pb = b-p;
    ibex::IntervalVector pa = a-p;

    // 3) compute the determinants for orientations
    ibex::Interval z1, z2, z3, z4, z5, z6, z7; 
    fwd_det(z1, ab, am);
    fwd_det(z2, ab, ap);
    z3 = z1*z2;
    fwd_det(z4, pm, pa);
    fwd_det(z5, pm, pb);
    z6 = z4*z5;
    z7 = ibex::Interval(0,POS_INFINITY);
    fwd_max(z7,z3,z6);

    /***BACKWARD***/ 

    ibex::bwd_max(z7,z3,z6);
    ibex::bwd_mul(z6,z4,z5);
    bwd_det(z5, pm, pb);
    bwd_det(z4, pm, pa);
    ibex::bwd_mul(z3,z1,z2);
    bwd_det(z2, ab, ap);
    bwd_det(z1, ab, am);
    ibex::bwd_sub(pa,a,p);
    ibex::bwd_sub(pb,b,p);
    ibex::bwd_sub(ap,p,a);
    ibex::bwd_sub(am,m,a);
    ibex::bwd_sub(ab,b,a);
    ibex::bwd_add(m,p,pm);
  }

  void CtcNoCross::fwd_det(ibex::Interval& det, ibex::IntervalVector& u, ibex::IntervalVector& v)
  {
    // forward
    det = det & (u[0]*v[1] - v[0]*u[1]);
  }

  void CtcNoCross::bwd_det(ibex::Interval& det, ibex::IntervalVector& u, ibex::IntervalVector& v)
  {
    ibex::Interval z1 = u[0]*v[1];
    ibex::Interval z2 = v[0]*u[1];
    // backward
    ibex::bwd_sub(det,z1,z2);
    ibex::bwd_mul(z2,v[0],u[1]);
    ibex::bwd_mul(z1,u[0],v[1]);
  }

  void CtcNoCross::fwd_max(ibex::Interval& a, ibex::Interval& b, ibex::Interval& c)
  {
    // a = max(b,c)
    // forward
    a = a & ibex::max(b,c);
  }
}