#include "hypascore_localization/contractors/Ctc3DStereoVO.hpp"
#include "Eigen/Dense"
#include <stack>

int main()
{
  double b = 0.85251;
  double f = 2189.843480347636;
  double cx = 970.9728546142579;
  double cy = 591.4599838256836;
  codac::Ctc3DStereoVO ctc(b,f,cx,cy); 

  Eigen::Matrix3d K; // camera matrix, identical for left and right camera
  K << f,0,cx,0,f,cy,0,0,1; 
  std::cout << "Camera Matrix: " << std::endl << K << std::endl; 

  // set ground truth pose
  double tx = 0.05;
  double ty = -0.15;
  double tz = 0.6;
  double phi = 0.01;
  double teta = 0.15;
  double psi = 0.02;
  Eigen::Affine3d f1_T_f2;
  f1_T_f2.translation() = Eigen::Vector3d(tx,ty,tz);
  Eigen::Quaternion<double> q = Eigen::AngleAxisd(psi,Eigen::Vector3d::UnitZ())
                            *Eigen::AngleAxisd(teta,Eigen::Vector3d::UnitY())
                            *Eigen::AngleAxisd(phi,Eigen::Vector3d::UnitX());
  f1_T_f2.linear() = q.matrix(); 
  std::cout << "Ground Truth: " << std::endl << f1_T_f2.matrix() << std::endl;
  Eigen::Affine3d f2_T_f1 = f1_T_f2.inverse();

  // set transfrom between streo rectified left and right cam
  Eigen::Affine3d lcam_T_rcam; lcam_T_rcam.setIdentity();
  lcam_T_rcam.matrix()(0,3) = b; 
  std::cout << "lcam_T_rcam: " << std::endl << lcam_T_rcam.matrix() << std::endl << "---" << std::endl << std::endl;

  // create fake points
  std::vector<Eigen::Vector3d> points_f1(5);
  points_f1[0] = Eigen::Vector3d(2,3,10);
  points_f1[1] = Eigen::Vector3d(-5,7,20);
  points_f1[2] = Eigen::Vector3d(-10,4,13);
  points_f1[3] = Eigen::Vector3d(20,3,30);
  points_f1[4] = Eigen::Vector3d(10,-7,15);

  std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> pixel_f1(points_f1.size()); // first is left, second is right
  std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> pixel_f2(points_f1.size());
  // project points to images
  for(int i = 0; i < points_f1.size(); i++)
  {
    Eigen::Vector3d& p_f1 = points_f1[i]; 

    // f1 
    // left camera:
    Eigen::Vector3d proj_l1 = K * p_f1;
    pixel_f1[i].first(0) =  proj_l1(0)/proj_l1(2); 
    pixel_f1[i].first(1) =  proj_l1(1)/proj_l1(2); 
    // eight camera:
    Eigen::Vector3d proj_r1 = K * lcam_T_rcam.inverse() * p_f1;
    pixel_f1[i].second(0) =  proj_r1(0)/proj_r1(2); 
    pixel_f1[i].second(1) =  proj_r1(1)/proj_r1(2);
    // f2
    // left camera:
    Eigen::Vector3d proj_l2 = K * f1_T_f2.inverse() * p_f1;
    pixel_f2[i].first(0) = proj_l2(0)/proj_l2(2);
    pixel_f2[i].first(1) = proj_l2(1)/proj_l2(2);
    // right camera:
    Eigen::Vector3d proj_r2 = K * lcam_T_rcam.inverse() * f1_T_f2.inverse() * p_f1;
    pixel_f2[i].second(0) = proj_r2(0)/proj_r2(2);
    pixel_f2[i].second(1) = proj_r2(1)/proj_r2(2);
  }

  ibex::IntervalVector f2_p_f1(6); 
  //f2_p_f1[0] = f2_T_f1.translation()(0);
  f2_p_f1[1] = f2_T_f1.translation()(1);
  //f2_p_f1[2] = f2_T_f1.translation()(2); f2_p_f1[2].inflate(2.0);
  f2_p_f1[3] = atan2(f2_T_f1.linear()(2,1),f2_T_f1.linear()(2,2));
  //f2_p_f1[4] = -asin(f2_T_f1.linear()(2,0)); f2_p_f1[4].inflate(0.2);
  f2_p_f1[5] = atan2(f2_T_f1.linear()(1,0),f2_T_f1.linear()(0,0));
  ibex::IntervalVector result(6); result.set_empty();

  ibex::IntervalVector f1_p_f2(6); 
  //f2_p_f1[0] = f2_T_f1.translation()(0);
  f1_p_f2[1] = f1_T_f2.translation()(1);
  //f2_p_f1[2] = f2_T_f1.translation()(2); f2_p_f1[2].inflate(2.0);
  f1_p_f2[3] = atan2(f1_T_f2.linear()(2,1),f1_T_f2.linear()(2,2));
  //f2_p_f1[4] = -asin(f2_T_f1.linear()(2,0)); f2_p_f1[4].inflate(0.2);
  f1_p_f2[5] = atan2(f1_T_f2.linear()(1,0),f1_T_f2.linear()(0,0));


  std::stack<ibex::IntervalVector> s;
  s.push(f1_p_f2);
  while(!s.empty())
  {
    ibex::IntervalVector box = s.top();
    s.pop();
    for(int i = 0; i < points_f1.size(); i++)
    {
      ibex::IntervalVector p_l1(2), p_r1(2), p_l2(2), p_r2(2); 
      p_l1[0] = pixel_f1[i].first(0);
      p_l1[1] = pixel_f1[i].first(1);
      p_l1.inflate(0.01);
      p_r1[0] = pixel_f1[i].second(0);
      p_r1[1] = pixel_f1[i].second(1);
      p_r1.inflate(0.01);
      p_l2[0] = pixel_f2[i].first(0);
      p_l2[1] = pixel_f2[i].first(1);
      p_l2.inflate(0.01);
      p_r2[0] = pixel_f2[i].second(0);
      p_r2[1] = pixel_f2[i].second(1);
      p_r2.inflate(0.01); 
      ibex::IntervalVector P_l1(3);
      P_l1[2] = ibex::Interval::POS_REALS;
      ctc.contract(p_l2,p_r2,p_l1,p_r1,P_l1,box); 
    }
    if(!box.is_empty())
    {
      if(box[4].diam() > 0.005)
      {
        std::pair<ibex::IntervalVector,ibex::IntervalVector> bisected_boxes = box.bisect(4);
        s.push(bisected_boxes.first);
        s.push(bisected_boxes.second); 
      }
      else if(box[0].diam() > 0.01)
      {
        std::pair<ibex::IntervalVector,ibex::IntervalVector> bisected_boxes = box.bisect(0);
        s.push(bisected_boxes.first);
        s.push(bisected_boxes.second); 
      }
      else
      {
        result = result | box; 
      }
    }
  }


  std::cout << result.diam() << std::endl;
}