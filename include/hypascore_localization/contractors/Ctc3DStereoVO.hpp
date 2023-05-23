#ifndef CTC3DSTEREOVO_H_
#define CTC3DSTEREOVO_H_

#include "codac/codac_Ctc.h"
#include "hypascore_localization/contractors/Ctc3DRotation.hpp"

namespace codac
{
  class Ctc3DStereoVO : public Ctc
  {
    public:
    Ctc3DStereoVO(double b, double f, double cx, double cy);
    virtual void contract(ibex::IntervalVector& x);
    void contract(ibex::IntervalVector& p_l1, ibex::IntervalVector& p_r1, ibex::IntervalVector& p_l2, ibex::IntervalVector& p_r2, ibex::IntervalVector& P_l1, ibex::IntervalVector& f2_p_f1);

    private:
    Ctc3DRotation ctc_3d_rot_;
    ibex::Interval b_, b_copy; // stereo baseline
    ibex::Interval f_,f_copy; // focal length
    ibex::Interval cx_, cx_copy, cy_, cy_copy; // pixel offsets 
  };
}

#endif