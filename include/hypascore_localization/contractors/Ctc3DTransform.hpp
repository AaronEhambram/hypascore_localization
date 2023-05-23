#ifndef CTC3DTRANSFORM_H_
#define CTC3DTRANSFORM_H_

#include "codac/codac_Ctc.h"
#include "hypascore_localization/contractors/Ctc3DRotation.hpp"

namespace codac
{
  class Ctc3DTransform : public Ctc
  {
    public:
    Ctc3DTransform();
    virtual void contract(ibex::IntervalVector& x);
    void contract(ibex::IntervalVector& b1, ibex::IntervalVector& b2, ibex::IntervalVector& f1_p_f2);

    private:
    Ctc3DRotation ctc_3d_rot_; 
  };
}

#endif