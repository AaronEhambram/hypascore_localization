#ifndef CTC3DROTATION_H_
#define CTC3DROTATION_H_

#include "codac/codac_Ctc.h"

namespace codac
{
  class Ctc3DRotation : public Ctc
  {
    public:
    Ctc3DRotation();
    virtual void contract(ibex::IntervalVector& x);
    void contract(ibex::IntervalVector& phi_teta_psi_angles, ibex::IntervalMatrix& R); 
  };
}

#endif