#include "hypascore_localization/contractors/Ctc3DStereoVO.hpp"
#include "cmath"

namespace codac
{

  Ctc3DStereoVO::Ctc3DStereoVO(double b, double f, double cx, double cy) : Ctc(17)
  {
    b_ = b; f_ = f; cx_ = cx; cy_ = cy;
    b_copy = b; f_copy = f; cx_copy = cx; cy_copy = cy; 
  }

  void Ctc3DStereoVO::contract(ibex::IntervalVector& x)
  {
    ibex::IntervalVector p_l1 = x.subvector(0,1);
    ibex::IntervalVector p_r1 = x.subvector(2,3);
    ibex::IntervalVector p_l2 = x.subvector(4,5);
    ibex::IntervalVector p_r2 = x.subvector(6,7);
    ibex::IntervalVector P_l1 = x.subvector(8,10);
    ibex::IntervalVector f2_p_f1 = x.subvector(11,16);
    contract(p_l1,p_r1,p_l2,p_r2,P_l1,f2_p_f1);
    x[0] = x[0] & p_l1[0];
    x[1] = x[1] & p_l1[1];
    x[2] = x[2] & p_r1[0];
    x[3] = x[3] & p_r1[1];
    x[4] = x[4] & p_l2[0];
    x[5] = x[5] & p_l2[1];
    x[6] = x[6] & p_r2[0];
    x[7] = x[7] & p_r2[1];
    x[8] = x[8] & P_l1[0];
    x[9] = x[9] & P_l1[1];
    x[10] = x[10] & P_l1[2]; 
    x[11] = x[11] & f2_p_f1[0];
    x[12] = x[12] & f2_p_f1[1];
    x[13] = x[13] & f2_p_f1[2];
    x[14] = x[14] & f2_p_f1[3];
    x[15] = x[15] & f2_p_f1[4];
    x[16] = x[16] & f2_p_f1[5];
  }

  void Ctc3DStereoVO::contract(ibex::IntervalVector& p_l1, ibex::IntervalVector& p_r1, ibex::IntervalVector& p_l2, ibex::IntervalVector& p_r2, ibex::IntervalVector& P_l1, ibex::IntervalVector& f2_p_f1)
  {
    b_ = b_copy; f_ = f_copy; cx_ = cx_copy; cy_ = cy_copy;
    // save the variables in refereces for convenience
    ibex::Interval& p_l1x = p_l1[0]; ibex::Interval& p_l1y = p_l1[1];
    ibex::Interval& p_r1x = p_r1[0]; ibex::Interval& p_r1y = p_r1[1];
    ibex::Interval& p_l2x = p_l2[0]; ibex::Interval& p_l2y = p_l2[1];
    ibex::Interval& p_r2x = p_r2[0]; ibex::Interval& p_r2y = p_r2[1];
    ibex::Interval& X = P_l1[0]; ibex::Interval& Y = P_l1[1]; ibex::Interval& Z = P_l1[2]; 
    ibex::Interval& tx = f2_p_f1[0];
    ibex::Interval& ty = f2_p_f1[1];
    ibex::Interval& tz = f2_p_f1[2];
    ibex::Interval& phi = f2_p_f1[3];
    ibex::Interval& teta = f2_p_f1[4];
    ibex::Interval& psi = f2_p_f1[5];

    // first pose, left camera -> l1
    // 1. equation 
    // forward
    ibex::Interval Xf = X*f_;
    ibex::Interval a_l11 = cx_-p_l1x;
    ibex::Interval a_l12 = Z*a_l11;
    // backward
    Xf = Xf & (-a_l12); a_l12 = a_l12 & (-Xf); 
    ibex::bwd_mul(a_l12,Z,a_l11);
    ibex::bwd_sub(a_l11,cx_,p_l1x);
    ibex::bwd_mul(Xf,X,f_);
    // 2. equation
    // forward
    ibex::Interval Yf = Y*f_;
    ibex::Interval a_l13 = cy_-p_l1y;
    ibex::Interval a_l14 = Z*a_l13;
    // backward
    Yf = Yf & (-a_l14); a_l14 = a_l14 & (-Yf); 
    ibex::bwd_mul(a_l14,Z,a_l13);
    ibex::bwd_sub(a_l13,cy_,p_l1y);
    ibex::bwd_mul(Yf,Y,f_); 

    // first pose, right camera -> r1
    // 1. equation
    // forward
    Xf = Xf & X*f_;
    ibex::Interval a_r11 = cx_-p_r1x;
    ibex::Interval a_r12 = Z*a_r11; 
    // backward
    Xf = Xf & (b_*f_-a_r12); a_r12 = a_r12 & (b_*f_-Xf);
    ibex::bwd_mul(a_r12,Z,a_r11);
    ibex::bwd_sub(a_r11,cx_,p_r1x);
    ibex::bwd_mul(Xf,X,f_);
    // 2. equation 
    Yf = Yf & Y*f_;
    ibex::Interval a_r13 = cy_-p_r1y;
    ibex::Interval a_r14 = Z*a_r13;
    // backward
    Yf = Yf & (-a_r14); a_r14 = a_r14 & (-Yf); 
    ibex::bwd_mul(a_r14,Z,a_r13);
    ibex::bwd_sub(a_r13,cy_,p_r1y);
    ibex::bwd_mul(Yf,Y,f_); 

    // rectified stereo constraint
    p_l1y = p_l1y & p_r1y;
    p_r1y = p_l1y & p_r1y;
    // depth-scale constraint for rectified stereo
    Z = Z & (b_*f_ * 1/(p_l1x-p_r1x));
    // constraints on Y-coordinate
    Y = Y & (Z*(p_l1y-cy_)/f_) & (Z*(p_r1y-cy_)/f_); // forward
    Z = Z & (Y*f_/(p_l1y-cy_)) & (Y*f_/(p_r1y-cy_)); // backward
    // constraints on X-coordinate
    X = X & (Z*(p_l1x-cx_)/f_) & ((Z*(p_r1x-cx_)/f_+b_)); // forward
    Z = Z & (X*f_/(p_l1x-cx_)) & ((X-b_)*f_/(p_r1x-cx_)); // backward

    // compute rotation matrix
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

    // second pose, left camera -> l2
    // 3. equation
    // forward
    ibex::Interval Xr31 = X*r31;
    ibex::Interval Yr32 = Y*r32;
    ibex::Interval Zr33 = Z*r33;
    ibex::Interval Xr31pYr32 = Xr31+Yr32;
    ibex::Interval Xr31pYr32pZr33 = Xr31pYr32+Zr33;
    ibex::Interval s2 = Xr31pYr32pZr33 + tz; 
    // backward
    ibex::bwd_add(s2,Xr31pYr32pZr33,tz);
    ibex::bwd_add(Xr31pYr32pZr33,Xr31pYr32,Zr33);
    ibex::bwd_add(Xr31pYr32,Xr31,Yr32);
    ibex::bwd_mul(Zr33,Z,r33);
    ibex::bwd_mul(Yr32,Y,r32);
    ibex::bwd_mul(Xr31,X,r31);
    // 1. equation
    // forward
    ibex::Interval fr11 = f_*r11;
    ibex::Interval cxr31 = cx_*r31;
    ibex::Interval fr11pcxr31 = fr11 + cxr31;
    ibex::Interval Xfr11pcxr31 = X*fr11pcxr31;
    ibex::Interval fr12 = f_*r12;
    ibex::Interval cxr32 = cx_*r32;
    ibex::Interval fr12pcxr32 = fr12 + cxr32;
    ibex::Interval Yfr12pcxr32 = Y * fr12pcxr32;
    ibex::Interval fr13 = f_*r13;
    ibex::Interval cxr33 = cx_*r33;
    ibex::Interval fr13pcxr33 = fr13 + cxr33;
    ibex::Interval Zfr13pcxr33 = Z * fr13pcxr33;
    ibex::Interval ftx = f_*tx;
    ibex::Interval cxtz = cx_*tz; 
    ibex::Interval ftxpcxtz = ftx+cxtz;
    ibex::Interval Xfr11pcxr31_p_Yfr12pcxr32 = Xfr11pcxr31 + Yfr12pcxr32;
    ibex::Interval Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33 = Xfr11pcxr31_p_Yfr12pcxr32 + Zfr13pcxr33;
    ibex::Interval Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33_p_ftxpcxtz = Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33 + ftxpcxtz;
    ibex::Interval s2p_l2x = s2 * p_l2x;

    // connection between left and right camera
    ibex::Interval fb = f_*b_;
    ibex::Interval const_bet = 1.0 - p_r2x/p_l2x;
    ibex::bwd_mul(fb,const_bet,Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33_p_ftxpcxtz);

    // backward
    s2p_l2x = s2p_l2x & Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33_p_ftxpcxtz; 
    Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33_p_ftxpcxtz = Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33_p_ftxpcxtz & s2p_l2x; 
    ibex::bwd_mul(s2p_l2x, s2, p_l2x); 
    ibex::bwd_add(Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33_p_ftxpcxtz, Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33, ftxpcxtz);
    ibex::bwd_add(Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33, Xfr11pcxr31_p_Yfr12pcxr32, Zfr13pcxr33);
    ibex::bwd_add(Xfr11pcxr31_p_Yfr12pcxr32,Xfr11pcxr31,Yfr12pcxr32);
    ibex::bwd_add(ftxpcxtz,ftx,cxtz);
    ibex::bwd_mul(cxtz,cx_,tz);
    ibex::bwd_mul(ftx, f_, tx);
    ibex::bwd_mul(Zfr13pcxr33,Z,fr13pcxr33);
    ibex::bwd_add(fr13pcxr33, fr13, cxr33);
    ibex::bwd_mul(cxr33, cx_, r33);
    ibex::bwd_mul(fr13, f_, r13); 
    ibex::bwd_mul(Yfr12pcxr32, Y, fr12pcxr32);
    ibex::bwd_add(fr12pcxr32, fr12, cxr32);
    ibex::bwd_mul(cxr32, cx_, r32);
    ibex::bwd_mul(fr12, f_, r12);
    ibex::bwd_mul(Xfr11pcxr31, X, fr11pcxr31);
    ibex::bwd_add(fr11pcxr31, fr11, cxr31);
    ibex::bwd_mul(cxr31, cx_, r31);
    ibex::bwd_mul(fr11, f_, r11); 
    // 2. Equation
    // forward
    ibex::Interval fr21 = f_*r21;
    ibex::Interval cyr31 = cy_*r31; 
    ibex::Interval fr21pcyr31 = fr21+cyr31;
    ibex::Interval Xfr21pcyr31 = X * fr21pcyr31;
    ibex::Interval fr22 = f_*r22;
    ibex::Interval cyr32 = cy_*r32;
    ibex::Interval fr22pcyr32 = fr22 + cyr32;
    ibex::Interval Yfr22pcyr32 = Y*fr22pcyr32; 
    ibex::Interval fr23 = f_*r23;
    ibex::Interval cyr33 = cy_*r33;
    ibex::Interval fr23pcyr33 = fr23 + cyr33;
    ibex::Interval Zfr23pcyr33 = Z*fr23pcyr33;
    ibex::Interval fty = f_*ty;
    ibex::Interval cytz = cy_*tz; 
    ibex::Interval ftypcytz = fty + cytz; 
    ibex::Interval Xfr21pcyr31_p_Yfr22pcyr32 = Xfr21pcyr31 + Yfr22pcyr32; 
    ibex::Interval Xfr21pcyr31_p_Yfr22pcyr32_p_Zfr23pcyr33 = Xfr21pcyr31_p_Yfr22pcyr32 + Zfr23pcyr33;
    ibex::Interval Xfr21pcyr31_p_Yfr22pcyr32_p_Zfr23pcyr33_p_ftypcytz = Xfr21pcyr31_p_Yfr22pcyr32_p_Zfr23pcyr33 + ftypcytz; 
    ibex::Interval s2p_l2y = s2*p_l2y;
    // backward
    s2p_l2y = s2p_l2y & Xfr21pcyr31_p_Yfr22pcyr32_p_Zfr23pcyr33_p_ftypcytz;
    Xfr21pcyr31_p_Yfr22pcyr32_p_Zfr23pcyr33_p_ftypcytz = Xfr21pcyr31_p_Yfr22pcyr32_p_Zfr23pcyr33_p_ftypcytz & s2p_l2y; 
    ibex::bwd_mul(s2p_l2y, s2, p_l2y);
    ibex::bwd_add(Xfr21pcyr31_p_Yfr22pcyr32_p_Zfr23pcyr33_p_ftypcytz, Xfr21pcyr31_p_Yfr22pcyr32_p_Zfr23pcyr33, ftypcytz);
    ibex::bwd_add(Xfr21pcyr31_p_Yfr22pcyr32_p_Zfr23pcyr33, Xfr21pcyr31_p_Yfr22pcyr32, Zfr23pcyr33);
    ibex::bwd_add(Xfr21pcyr31_p_Yfr22pcyr32, Xfr21pcyr31, Yfr22pcyr32);
    ibex::bwd_add(ftypcytz, fty, cytz);
    ibex::bwd_mul(cytz, cy_, tz);
    ibex::bwd_mul(fty, f_, ty);
    ibex::bwd_mul(Zfr23pcyr33, Z, fr23pcyr33); 
    ibex::bwd_add(fr23pcyr33, fr23, cyr33);
    ibex::bwd_mul(cyr33, cy_, r33);
    ibex::bwd_mul(fr23, f_, r23);
    ibex::bwd_mul(Yfr22pcyr32, Y, fr22pcyr32);
    ibex::bwd_add(fr22pcyr32, fr22, cyr32);
    ibex::bwd_mul(cyr32, cy_, r32);
    ibex::bwd_mul(fr22, f_, r22);
    ibex::bwd_mul(Xfr21pcyr31, X, fr21pcyr31);
    ibex::bwd_add(fr21pcyr31, fr21, cyr31);
    ibex::bwd_mul(cyr31, cy_, r31);
    ibex::bwd_mul(fr21, f_, r21);

    // second pose, right camera -> r2
    // 3. equation
    // forward
    Xr31 = Xr31 & X*r31;
    Yr32 = Yr32 & Y*r32;
    Zr33 = Zr33 & Z*r33;
    Xr31pYr32 = Xr31pYr32 & (Xr31+Yr32);
    Xr31pYr32pZr33 = Xr31pYr32pZr33 & (Xr31pYr32+Zr33);
    s2 = s2 & (Xr31pYr32pZr33 + tz); 
    // backward
    ibex::bwd_add(s2,Xr31pYr32pZr33,tz);
    ibex::bwd_add(Xr31pYr32pZr33,Xr31pYr32,Zr33);
    ibex::bwd_add(Xr31pYr32,Xr31,Yr32);
    ibex::bwd_mul(Zr33,Z,r33);
    ibex::bwd_mul(Yr32,Y,r32);
    ibex::bwd_mul(Xr31,X,r31);
    // 1. equation
    // forward
    fr11 = fr11 & f_*r11;
    cxr31 = cxr31 & cx_*r31;
    fr11pcxr31 = fr11pcxr31 & (fr11 + cxr31);
    Xfr11pcxr31 = Xfr11pcxr31 & (X*fr11pcxr31);
    fr12 = fr12 & (f_*r12);
    cxr32 = cxr32 & (cx_*r32);
    fr12pcxr32 = fr12pcxr32 & (fr12 + cxr32);
    Yfr12pcxr32 = Yfr12pcxr32 & (Y * fr12pcxr32);
    fr13 = fr13 & (f_*r13);
    cxr33 = cxr33 & (cx_*r33);
    fr13pcxr33 = fr13pcxr33 & (fr13 + cxr33);
    Zfr13pcxr33 = Zfr13pcxr33 & (Z * fr13pcxr33);
    ftx = ftx & (f_*tx);
    cxtz = cxtz & (cx_*tz); 
    ftxpcxtz = ftxpcxtz & (ftx+cxtz);
    Xfr11pcxr31_p_Yfr12pcxr32 = Xfr11pcxr31_p_Yfr12pcxr32 & (Xfr11pcxr31 + Yfr12pcxr32);
    Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33 = Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33 & (Xfr11pcxr31_p_Yfr12pcxr32 + Zfr13pcxr33);
    Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33_p_ftxpcxtz = Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33_p_ftxpcxtz & (Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33 + ftxpcxtz); 
    ibex::Interval Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33_p_ftxpcxtz_m_fb = Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33_p_ftxpcxtz - fb; 
    ibex::Interval s2p_r2x = s2 * p_r2x;
    // backward
    s2p_r2x = s2p_r2x & Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33_p_ftxpcxtz_m_fb; 
    Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33_p_ftxpcxtz_m_fb = Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33_p_ftxpcxtz_m_fb & s2p_r2x; 
    ibex::bwd_mul(s2p_r2x, s2, p_r2x);
    ibex::bwd_sub(Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33_p_ftxpcxtz_m_fb, Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33_p_ftxpcxtz, fb);
    ibex::bwd_mul(fb, f_, b_);
    ibex::bwd_add(Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33_p_ftxpcxtz, Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33, ftxpcxtz);
    ibex::bwd_add(Xfr11pcxr31_p_Yfr12pcxr32_p_Zfr13pcxr33, Xfr11pcxr31_p_Yfr12pcxr32, Zfr13pcxr33);
    ibex::bwd_add(Xfr11pcxr31_p_Yfr12pcxr32,Xfr11pcxr31,Yfr12pcxr32);
    ibex::bwd_add(ftxpcxtz,ftx,cxtz);
    ibex::bwd_mul(cxtz,cx_,tz);
    ibex::bwd_mul(ftx, f_, tx);
    ibex::bwd_mul(Zfr13pcxr33,Z,fr13pcxr33);
    ibex::bwd_add(fr13pcxr33, fr13, cxr33);
    ibex::bwd_mul(cxr33, cx_, r33);
    ibex::bwd_mul(fr13, f_, r13); 
    ibex::bwd_mul(Yfr12pcxr32, Y, fr12pcxr32);
    ibex::bwd_add(fr12pcxr32, fr12, cxr32);
    ibex::bwd_mul(cxr32, cx_, r32);
    ibex::bwd_mul(fr12, f_, r12);
    ibex::bwd_mul(Xfr11pcxr31, X, fr11pcxr31);
    ibex::bwd_add(fr11pcxr31, fr11, cxr31);
    ibex::bwd_mul(cxr31, cx_, r31);
    ibex::bwd_mul(fr11, f_, r11);
    // 2. Equation
    // forward
    fr21 = fr21 & (f_*r21);
    cyr31 = cyr31 & (cy_*r31); 
    fr21pcyr31 = fr21pcyr31 & (fr21+cyr31);
    Xfr21pcyr31 = Xfr21pcyr31 & (X * fr21pcyr31);
    fr22 = fr22 & (f_*r22);
    cyr32 = cyr32 & (cy_*r32);
    fr22pcyr32 = fr22pcyr32 & (fr22 + cyr32);
    Yfr22pcyr32 = Yfr22pcyr32 & (Y*fr22pcyr32); 
    fr23 = fr23 & (f_*r23);
    cyr33 = cyr33 & (cy_*r33);
    fr23pcyr33 = fr23pcyr33 & (fr23 + cyr33);
    Zfr23pcyr33 = Zfr23pcyr33 & (Z*fr23pcyr33);
    fty = fty & (f_*ty);
    cytz = cytz & (cy_*tz); 
    ftypcytz = ftypcytz & (fty + cytz); 
    Xfr21pcyr31_p_Yfr22pcyr32 = Xfr21pcyr31_p_Yfr22pcyr32 & (Xfr21pcyr31 + Yfr22pcyr32); 
    Xfr21pcyr31_p_Yfr22pcyr32_p_Zfr23pcyr33 = Xfr21pcyr31_p_Yfr22pcyr32_p_Zfr23pcyr33 & (Xfr21pcyr31_p_Yfr22pcyr32 + Zfr23pcyr33);
    Xfr21pcyr31_p_Yfr22pcyr32_p_Zfr23pcyr33_p_ftypcytz = Xfr21pcyr31_p_Yfr22pcyr32_p_Zfr23pcyr33_p_ftypcytz & (Xfr21pcyr31_p_Yfr22pcyr32_p_Zfr23pcyr33 + ftypcytz); 
    ibex::Interval s2p_r2y = s2*p_r2y;
    // backward
    s2p_r2y = s2p_r2y & Xfr21pcyr31_p_Yfr22pcyr32_p_Zfr23pcyr33_p_ftypcytz;
    Xfr21pcyr31_p_Yfr22pcyr32_p_Zfr23pcyr33_p_ftypcytz = Xfr21pcyr31_p_Yfr22pcyr32_p_Zfr23pcyr33_p_ftypcytz & s2p_r2y; 
    ibex::bwd_mul(s2p_r2y, s2, p_r2y);
    ibex::bwd_add(Xfr21pcyr31_p_Yfr22pcyr32_p_Zfr23pcyr33_p_ftypcytz, Xfr21pcyr31_p_Yfr22pcyr32_p_Zfr23pcyr33, ftypcytz);
    ibex::bwd_add(Xfr21pcyr31_p_Yfr22pcyr32_p_Zfr23pcyr33, Xfr21pcyr31_p_Yfr22pcyr32, Zfr23pcyr33);
    ibex::bwd_add(Xfr21pcyr31_p_Yfr22pcyr32, Xfr21pcyr31, Yfr22pcyr32);
    ibex::bwd_add(ftypcytz, fty, cytz);
    ibex::bwd_mul(cytz, cy_, tz);
    ibex::bwd_mul(fty, f_, ty);
    ibex::bwd_mul(Zfr23pcyr33, Z, fr23pcyr33); 
    ibex::bwd_add(fr23pcyr33, fr23, cyr33);
    ibex::bwd_mul(cyr33, cy_, r33);
    ibex::bwd_mul(fr23, f_, r23);
    ibex::bwd_mul(Yfr22pcyr32, Y, fr22pcyr32);
    ibex::bwd_add(fr22pcyr32, fr22, cyr32);
    ibex::bwd_mul(cyr32, cy_, r32);
    ibex::bwd_mul(fr22, f_, r22);
    ibex::bwd_mul(Xfr21pcyr31, X, fr21pcyr31);
    ibex::bwd_add(fr21pcyr31, fr21, cyr31);
    ibex::bwd_mul(cyr31, cy_, r31);
    ibex::bwd_mul(fr21, f_, r21);

    // call the rotation matrix contractor 
    ctc_3d_rot_.contract(euler_angles,f1_R_f2);
    phi = phi & euler_angles[0];
    teta = teta & euler_angles[1];
    psi = psi & euler_angles[2]; 

    
  }
}