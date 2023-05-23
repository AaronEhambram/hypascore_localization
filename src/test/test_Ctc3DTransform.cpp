#include "hypascore_localization/contractors/Ctc3DTransform.hpp"
#include "Eigen/Dense"
#include <stack>

int main()
{
  codac::Ctc3DTransform ctc; 

  // set groudn truth pose
  double tx = 0.0;
  double ty = -0.0;
  double tz = 0.68;
  double phi = 0.00;
  double teta = M_PI/4.0;
  double psi = 0.00;

  // create fake points
  std::vector<Eigen::Vector3d> points_f1(5);
  points_f1[0] = Eigen::Vector3d(3,2,10);
  points_f1[1] = Eigen::Vector3d(-5,7,20);
  points_f1[2] = Eigen::Vector3d(-10,4,13);
  points_f1[3] = Eigen::Vector3d(20,3,30);
  points_f1[4] = Eigen::Vector3d(10,-7,15);

  Eigen::Affine3d f1_T_f2;
  f1_T_f2.translation() = Eigen::Vector3d(tx,ty,tz);
  Eigen::Quaternion<double> q = Eigen::AngleAxisd(psi,Eigen::Vector3d::UnitZ())
                            *Eigen::AngleAxisd(teta,Eigen::Vector3d::UnitY())
                            *Eigen::AngleAxisd(phi,Eigen::Vector3d::UnitX());
  f1_T_f2.linear() = q.matrix(); 
  std::cout << "Ground Truth: " << std::endl << f1_T_f2.matrix() << std::endl << "---" << std::endl << std::endl;

  // transform points to te second frame
  std::vector<Eigen::Vector3d> points_f2(5);
  for(int i = 0; i<5; i++)
  {
    points_f2[i] = f1_T_f2.inverse()*points_f1[i];
  }

  // inflate the local measurements
  std::vector<ibex::IntervalVector> boxes_f1;
  std::vector<ibex::IntervalVector> boxes_f2;
  for(int i = 0; i<5; i++)
  {
    Eigen::Vector3d& p_f1 = points_f1[i];
    Eigen::Vector3d& p_f2 = points_f2[i];

    ibex::IntervalVector b_f1(3), b_f2(3);
    b_f1[0] = ibex::Interval(p_f1(0)).inflate(0.05);
    b_f1[1] = ibex::Interval(p_f1(1)).inflate(0.05);
    b_f1[2] = ibex::Interval(p_f1(2)).inflate(0.2);
    b_f2[0] = ibex::Interval(p_f2(0)).inflate(0.05);
    b_f2[1] = ibex::Interval(p_f2(1)).inflate(0.05);
    b_f2[2] = ibex::Interval(p_f2(2)).inflate(0.2);
    boxes_f1.push_back(b_f1);
    boxes_f2.push_back(b_f2);
  }

  ibex::IntervalVector f1_p_f2(6);
  //f1_p_f2[0] = tx;
  f1_p_f2[1] = ty;
  f1_p_f2[3] = phi;
  //f1_p_f2[4] = teta;
  f1_p_f2[5] = psi;
  ibex::IntervalVector result(6); result.set_empty(); 
  std::cout << f1_p_f2 << std::endl;
  std::stack<ibex::IntervalVector> s;
  s.push(f1_p_f2);
  while(!s.empty())
  {
    ibex::IntervalVector box = s.top();
    s.pop();
    for(int i = 0; i<5; i++)
    {
      ibex::IntervalVector b1 = boxes_f1[i];
      ibex::IntervalVector b2 = boxes_f2[i];
      ctc.contract(b1,b2,box);
    }
    if(!box.is_empty())
    {
      if(box[4].diam() > 0.005)
      {
        std::pair<ibex::IntervalVector,ibex::IntervalVector> bisected_boxes = box.bisect(4);
        s.push(bisected_boxes.first);
        s.push(bisected_boxes.second); 
      }
      else
      {
        result = result | box; 
      }
    }
  }
  std::cout << result << std::endl; 
  
  /*for(int i = 0; i<5; i++)
  {
    ibex::IntervalVector b1 = boxes_f1[i];
    ibex::IntervalVector b2 = boxes_f2[i];
    ctc.contract(b1,b2,f1_p_f2);
    std::cout << f1_p_f2 << std::endl << "----------" << std::endl; 
  }*/
  
}