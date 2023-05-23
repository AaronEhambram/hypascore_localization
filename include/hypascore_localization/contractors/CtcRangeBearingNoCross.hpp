#ifndef CTCRANGEBEARINGNOCROSS_H_
#define CTCRANGEBEARINGNOCROSS_H_

#include "codac/codac_Ctc.h"
#include "hypascore_localization/contractors/CtcNoCross.hpp"
#include "codac/codac_CtcPolar.h"

namespace codac
{
  class CtcRangeBearingNoCross : public Ctc
  {
    public:
    CtcRangeBearingNoCross();
    virtual void contract(ibex::IntervalVector& x);
    void contract(ibex::IntervalVector& pose, ibex::Interval& range, ibex::Interval& bearing, ibex::IntervalVector& a, ibex::IntervalVector& b); 

    private:
    codac::CtcNoCross ctc_nocross;
    codac::CtcPolar ctc_polar; 
  };
}

#endif