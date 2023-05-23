#ifndef CTCBOUNDARYLINE_H_
#define CTCBOUNDARYLINE_H_

#include "codac/codac_Ctc.h"

namespace codac
{
  class CtcBoundaryLine : public Ctc
  {
    public:
    CtcBoundaryLine();
    virtual void contract(ibex::IntervalVector& x);
    void contract(ibex::IntervalVector& box, ibex::IntervalVector& a, ibex::IntervalVector& b); 

    private:
    void fwd_det(ibex::Interval& det, ibex::IntervalVector& u, ibex::IntervalVector& v);
    void bwd_det(ibex::Interval& det, ibex::IntervalVector& u, ibex::IntervalVector& v);
  };
}

#endif