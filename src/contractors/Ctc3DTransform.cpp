#include "hypascore_localization/contractors/Ctc3DTransform.hpp"
#include "cmath"

namespace codac
{

  Ctc3DTransform::Ctc3DTransform() : Ctc(12)
  {}

  void Ctc3DTransform::contract(ibex::IntervalVector& x)
  {
    ibex::IntervalVector b1 = x.subvector(0,2);
    ibex::IntervalVector b2 = x.subvector(3,5);
    ibex::IntervalVector f1_p_f2 = x.subvector(6,11);
    contract(b1,b2,f1_p_f2);
    x[0] = x[0] & b1[0];
    x[1] = x[1] & b1[1];
    x[2] = x[2] & b1[2];
    x[3] = x[3] & b2[0];
    x[4] = x[4] & b2[1];
    x[5] = x[5] & b2[2];
    x[6] = x[6] & f1_p_f2[0];
    x[7] = x[7] & f1_p_f2[1];
    x[8] = x[8] & f1_p_f2[2];
    x[9] = x[9] & f1_p_f2[3];
    x[10] = x[10] & f1_p_f2[4];
    x[11] = x[11] & f1_p_f2[5];
  }

  void Ctc3DTransform::contract(ibex::IntervalVector& b1, ibex::IntervalVector& b2, ibex::IntervalVector& f1_p_f2)
  {
    // get the references to the variables 
    ibex::Interval& tx = f1_p_f2[0];
    ibex::Interval& ty = f1_p_f2[1];
    ibex::Interval& tz = f1_p_f2[2];
    ibex::Interval& phi = f1_p_f2[3];
    ibex::Interval& teta = f1_p_f2[4];
    ibex::Interval& psi = f1_p_f2[5];
    ibex::Interval& x1 = b1[0];
    ibex::Interval& y1 = b1[1];
    ibex::Interval& z1 = b1[2];
    ibex::Interval& x2 = b2[0];
    ibex::Interval& y2 = b2[1];
    ibex::Interval& z2 = b2[2];
    ibex::IntervalMatrix f1_R_f2(3,3);
    ibex::Interval& r11 = f1_R_f2[0][0]; ibex::Interval& r12 = f1_R_f2[0][1]; ibex::Interval& r13 = f1_R_f2[0][2];
    ibex::Interval& r21 = f1_R_f2[1][0]; ibex::Interval& r22 = f1_R_f2[1][1]; ibex::Interval& r23 = f1_R_f2[1][2];
    ibex::Interval& r31 = f1_R_f2[2][0]; ibex::Interval& r32 = f1_R_f2[2][1]; ibex::Interval& r33 = f1_R_f2[2][2];

    // contract the rotation matrix parameters
    ibex::IntervalVector euler_angles(3);
    euler_angles[0] = phi;
    euler_angles[1] = teta;
    euler_angles[2] = psi; 
    ctc_3d_rot_.contract(euler_angles,f1_R_f2);
    phi = phi & euler_angles[0];
    teta = teta & euler_angles[1];
    psi = psi & euler_angles[2]; 

    // rigid body transformation equation system
    // forward
    // first equation forward
    ibex::Interval k1 = r11*x2;
    ibex::Interval k2 = r12*y2;
    ibex::Interval k3 = r13*z2;
    ibex::Interval k4 = k1+k2;
    ibex::Interval k5 = k4+k3;
    x1 = x1 & (k5+tx);
    // second equation forward
    ibex::Interval l1 = r21*x2;
    ibex::Interval l2 = r22*y2;
    ibex::Interval l3 = r23*z2;
    ibex::Interval l4 = l1+l2;
    ibex::Interval l5 = l4+l3;
    y1 = y1 & (l5+ty);
    // third equation forward
    ibex::Interval m1 = r31*x2;
    ibex::Interval m2 = r32*y2;
    ibex::Interval m3 = r33*z2;
    ibex::Interval m4 = m1+m2;
    ibex::Interval m5 = m4+m3;
    z1 = z1 & (m5+tz);

    // backward
    // first equation
    ibex::bwd_add(z1,m5,tz);
    ibex::bwd_add(m5,m4,m3);
    ibex::bwd_add(m4,m1,m2);
    ibex::bwd_mul(m3,r33,z2);
    ibex::bwd_mul(m2,r32,y2);
    ibex::bwd_mul(m1,r31,x2);
    // second equation
    ibex::bwd_add(y1,l5,ty);
    ibex::bwd_add(l5,l4,l3);
    ibex::bwd_add(l4,l1,l2);
    ibex::bwd_mul(l3,r23,z2);
    ibex::bwd_mul(l2,r22,y2);
    ibex::bwd_mul(l1,r21,x2);
    // third equation
    ibex::bwd_add(x1,k5,tx);
    ibex::bwd_add(k5,k4,k3);
    ibex::bwd_add(k4,k1,k2);
    ibex::bwd_mul(k3,r13,z2);
    ibex::bwd_mul(k2,r12,y2);
    ibex::bwd_mul(k1,r11,x2);

    // call the rotation matrix contractor 
    ctc_3d_rot_.contract(euler_angles,f1_R_f2);
    phi = phi & euler_angles[0];
    teta = teta & euler_angles[1];
    psi = psi & euler_angles[2]; 

  }
}