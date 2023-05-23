#ifndef SEPPOLYGON_H_
#define SEPPOLYGON_H_

#include "codac/codac_Ctc.h"

class SepPolygon
{
  public:
  void separate(ibex::IntervalVector& box, std::vector<ibex::IntervalVector>& polygon_vertices, std::vector<ibex::IntervalVector>& boxes_in, std::vector<ibex::IntervalVector>& boxes_out, ibex::IntervalVector& boundary_hull); 
};

#endif