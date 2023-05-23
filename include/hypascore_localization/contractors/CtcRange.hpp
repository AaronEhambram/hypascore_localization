#ifndef CTCRANGE_H_
#define CTCRANGE_H_

#include "codac/codac_Ctc.h"

namespace codac
{
  class CtcRange : public Ctc
  {
    public:
    CtcRange();
    virtual void contract(ibex::IntervalVector& x);
    void contract(ibex::IntervalVector& b, ibex::IntervalVector& c, ibex::Interval& r); 
  };
}

#endif