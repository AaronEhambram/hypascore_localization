#ifndef CTCNOCROSS_H_
#define CTCNOCROSS_H_

#include "codac/codac_Ctc.h"

namespace codac
{
  class CtcNoCross : public Ctc
  {
    public:
    CtcNoCross();
    virtual void contract(ibex::IntervalVector& x);
    void contract(ibex::IntervalVector& pose, ibex::IntervalVector& local_measurement, ibex::IntervalVector& a, ibex::IntervalVector& b); 

    private:
    void fwd_det(ibex::Interval& det, ibex::IntervalVector& u, ibex::IntervalVector& v);
    void bwd_det(ibex::Interval& det, ibex::IntervalVector& u, ibex::IntervalVector& v);
    void fwd_max(ibex::Interval& a, ibex::Interval& b, ibex::Interval& c);
  };
}

#endif