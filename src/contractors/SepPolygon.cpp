#include "hypascore_localization/contractors/SepPolygon.hpp"
#include "hypascore_localization/contractors/CtcBoundaryLine.hpp"
#include "cmath"
#include "opencv2/imgproc.hpp"

void SepPolygon::separate(ibex::IntervalVector& box, std::vector<ibex::IntervalVector>& polygon_vertices, std::vector<ibex::IntervalVector>& boxes_in, std::vector<ibex::IntervalVector>& boxes_out, ibex::IntervalVector& boundary_hull)
{ 
  boxes_in.clear();
  boxes_out.clear(); 
  // iterate through each segment and compute the boundary hull
  codac::CtcBoundaryLine ctc;
  ibex::IntervalVector in_box = box; 
  boundary_hull.set_empty(); 
  for(int i = 1; i < polygon_vertices.size(); ++i)
  {
    ibex::IntervalVector a_vert = polygon_vertices[i-1]; 
    ibex::IntervalVector b_vert = polygon_vertices[i];
    ctc.contract(in_box,a_vert,b_vert);
    boundary_hull = boundary_hull | in_box;
    in_box = box; 
  }
  ibex::IntervalVector a_vert = polygon_vertices[polygon_vertices.size()-1]; 
  ibex::IntervalVector b_vert = polygon_vertices[0];
  ctc.contract(in_box,a_vert,b_vert);
  boundary_hull = boundary_hull | in_box;

  std::vector<cv::Point2f> cv_points;
  for(ibex::IntervalVector& p : polygon_vertices)
  {
    cv::Point2f cv_p;
    cv_p.x = (float)p[0].mid();
    cv_p.y = (float)p[1].mid();
    cv_points.push_back(cv_p);
  }

  // No intersection with boundaries
  if(boundary_hull.is_empty())
  {
    // check if box inside or outside
    cv::Point2f box_test_point; 
    ibex::Vector box_mid = box.mid(); 
    box_test_point.x = (float)box_mid[0];
    box_test_point.y = (float)box_mid[1];
    int poly_test = cv::pointPolygonTest(cv_points,box_test_point,false);
    if(poly_test > 0)
    {
      boxes_in.push_back(box);
    }
    else
    {
      boxes_out.push_back(box);
    }
    return;
  }
  // if boundary_hull is not empty, we need to consider regions
  // divide the regions outside the boundary hull into 4 regions (l (left),tr (top-right),br (bottom-right),b (bottom))
  std::vector<ibex::IntervalVector> test_regions; 
  // get the left-most subbox
  ibex::IntervalVector l(2);
  if(box.lb()[0] == boundary_hull.lb()[0])
  {
    l.set_empty();
  }
  else
  {
    l[0] = ibex::Interval(box.lb()[0], boundary_hull.lb()[0]);
    l[1] = box[1]; 
    test_regions.push_back(l);
  }
  // get the top-right box
  ibex::IntervalVector tr(2);
  if(box.ub()[1] == boundary_hull.ub()[1] || box.ub()[0] == boundary_hull.lb()[0])
  {
    tr.set_empty();
  }
  else
  {
    tr[0] = ibex::Interval(boundary_hull.lb()[0],box.ub()[0]);
    tr[1] = ibex::Interval(boundary_hull.ub()[1],box.ub()[1]);
    test_regions.push_back(tr);
  }
  // get the bottom-right most box
  ibex::IntervalVector br(2);
  if(boundary_hull.ub()[0] == box.ub()[0] || boundary_hull.ub()[1] == box.lb()[1])
  {
    br.set_empty();
  }
  else
  {
    br[0] = ibex::Interval(boundary_hull.ub()[0],box.ub()[0]);
    br[1] = ibex::Interval(box.lb()[1],boundary_hull.ub()[1]);
    test_regions.push_back(br);
  }
  // get the bottom-most box
  ibex::IntervalVector b(2);
  if(box.lb()[1] == boundary_hull.lb()[1] || boundary_hull.ub()[0] == boundary_hull.lb()[0])
  {
    b.set_empty();
  }
  else
  {
    b[0] = boundary_hull[0];
    b[1] = ibex::Interval(box.lb()[1],boundary_hull.lb()[1]);
    test_regions.push_back(b);
  }

  // check for each of the regions if the mid-point is inside or outside the polygon
  cv::Point2f region_test_point;
  ibex::Vector region_mid(2);
  int poly_test = 0; 
  for(ibex::IntervalVector& region : test_regions)
  {
    region_mid = region.mid(); 
    region_test_point.x = (float)region_mid[0];
    region_test_point.y = (float)region_mid[1];
    poly_test = cv::pointPolygonTest(cv_points,region_test_point,false);
    if(poly_test > 0)
    {
      boxes_in.push_back(region);
    }
    else
    {
      boxes_out.push_back(region);
    }
  }
}