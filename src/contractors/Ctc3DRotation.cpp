#include "hypascore_localization/contractors/Ctc3DRotation.hpp"
#include "cmath"

namespace codac
{

  Ctc3DRotation::Ctc3DRotation() : Ctc(12)
  {}

  void Ctc3DRotation::contract(ibex::IntervalVector& x)
  {
    ibex::IntervalVector phi_teta_psi_angles = x.subvector(0,2);
    ibex::IntervalMatrix R(3,3);
    R[0][0] = x[3]; R[0][1] = x[4]; R[0][2] = x[5]; 
    R[1][0] = x[6]; R[1][1] = x[7]; R[1][2] = x[8]; 
    R[2][0] = x[9]; R[2][1] = x[10]; R[2][2] = x[11];
    contract(phi_teta_psi_angles,R);
    x[0] = x[0] & phi_teta_psi_angles[0];
    x[1] = x[1] & phi_teta_psi_angles[1];
    x[2] = x[2] & phi_teta_psi_angles[2];
    x[3] = x[3] & R[0][0];
    x[4] = x[4] & R[0][1];
    x[5] = x[5] & R[0][2];
    x[6] = x[6] & R[1][0];
    x[7] = x[7] & R[1][1];
    x[8] = x[8] & R[1][2];
    x[9] = x[9] & R[2][0];
    x[10] = x[10] & R[2][1];
    x[11] = x[11] & R[2][2];
  }

  void Ctc3DRotation::contract(ibex::IntervalVector& phi_teta_psi_angles, ibex::IntervalMatrix& R)
  {
    /****Build up the rotation matrix****/
    ibex::Interval& r11 = R[0][0]; ibex::Interval& r12 = R[0][1]; ibex::Interval& r13 = R[0][2];
    ibex::Interval& r21 = R[1][0]; ibex::Interval& r22 = R[1][1]; ibex::Interval& r23 = R[1][2];
    ibex::Interval& r31 = R[2][0]; ibex::Interval& r32 = R[2][1]; ibex::Interval& r33 = R[2][2];
    ibex::Interval& phi = phi_teta_psi_angles[0];
    ibex::Interval& teta = phi_teta_psi_angles[1];
    ibex::Interval& psi = phi_teta_psi_angles[2];
    // rotation parameters
    ibex::Interval sphi = sin(phi);
    ibex::Interval cphi = cos(phi);
    ibex::Interval steta = sin(teta);
    ibex::Interval cteta = cos(teta);
    ibex::Interval spsi = sin(psi);
    ibex::Interval cpsi = cos(psi);
    // first row:
    r11 = r11 & (cteta*cpsi);
    ibex::Interval a12 = cphi*spsi;
    ibex::Interval b12 = steta*cpsi;
    ibex::Interval c12 = b12*sphi;
    r12 = r12 & (c12-a12);
    ibex::Interval a13 = spsi*sphi;
    ibex::Interval c13 = b12*cphi;
    r13 = r13 & (a13+c13);
    // second row:
    r21 = r21 & (cteta*spsi);
    ibex::Interval a22 = cpsi*cphi;
    ibex::Interval b22 = steta*spsi;
    ibex::Interval c22 = b22*sphi;
    r22 = r22 & (a22+c22);
    ibex::Interval a23 = cpsi*sphi;
    ibex::Interval c23 = b22*cphi;
    r23 = r23 & (c23-a23);
    // third row:
    r31 = r31 & (-steta);
    r32 = r32 & (cteta*sphi);
    r33 = r33 & (cteta*cphi);
    // Use the rotation symmetries
    ibex::Interval sqrr11 = sqr(r11); ibex::Interval sqrr12 = sqr(r12); ibex::Interval sqrr13 = sqr(r13);
    ibex::Interval sqrr21 = sqr(r21); ibex::Interval sqrr22 = sqr(r22); ibex::Interval sqrr23 = sqr(r23);
    ibex::Interval sqrr31 = sqr(r31); ibex::Interval sqrr32 = sqr(r32); ibex::Interval sqrr33 = sqr(r33);
    ibex::Interval Z1 = sqrr11+sqrr12;
    ibex::Interval Z2 = sqrr21+sqrr22;
    ibex::Interval Z3 = sqrr31+sqrr32;
    ibex::Interval s1 = sqrr11+sqrr21;
    ibex::Interval s2 = sqrr12+sqrr22;
    ibex::Interval s3 = sqrr13+sqrr23;
    ibex::Interval r11xr21 = r11*r21; ibex::Interval r12xr22 = r12*r22; ibex::Interval r13xr23 = r13*r23;
    ibex::Interval A1 = r11xr21+r12xr22;
    ibex::Interval r11xr31 = r11*r31; ibex::Interval r12xr32 = r12*r32; ibex::Interval r13xr33 = r13*r33;
    ibex::Interval A2 = r11xr31+r12xr32;
    ibex::Interval r21xr31 = r21*r31; ibex::Interval r22xr32 = r22*r32; ibex::Interval r23xr33 = r23*r33;
    ibex::Interval A3 = r21xr31+r22xr32;
    ibex::Interval r11xr12 = r11*r12; ibex::Interval r21xr22 = r21*r22; ibex::Interval r31xr32 = r31*r32;
    ibex::Interval A4 = r11xr12+r21xr22;
    ibex::Interval r11xr13 = r11*r13; ibex::Interval r21xr23 = r21*r23; ibex::Interval r31xr33 = r31*r33;
    ibex::Interval A5 = r11xr13+r21xr23;
    ibex::Interval r12xr13 = r12*r13; ibex::Interval r22xr23 = r22*r23; ibex::Interval r32xr33 = r32*r33;
    ibex::Interval A6 = r12xr13+r22xr23;

    // backward
    r32xr33 = r32xr33 & (-A6); A6 = A6 & (-r32xr33); // sum zero constraint
    r31xr33 = r31xr33 & (-A5); A5 = A5 & (-r31xr33);
    r31xr32 = r31xr32 & (-A4); A4 = A4 & (-r31xr32);
    r23xr33 = r23xr33 & (-A3); A3 = A3 & (-r23xr33);
    r13xr33 = r13xr33 & (-A2); A2 = A2 & (-r13xr33);
    r13xr23 = r13xr23 & (-A1); A1 = A1 & (-r13xr23);
    ibex::bwd_add(A6,r12xr13,r22xr23);
    ibex::bwd_mul(r12xr13,r12,r13);ibex::bwd_mul(r22xr23,r22,r23); ibex::bwd_mul(r32xr33,r32,r33);
    ibex::bwd_add(A5,r11xr13,r21xr23);
    ibex::bwd_mul(r11xr13,r11,r13);ibex::bwd_mul(r21xr23,r21,r23); ibex::bwd_mul(r31xr33,r31,r33);
    ibex::bwd_add(A4,r11xr12,r21xr22);
    ibex::bwd_mul(r11xr12,r11,r12);ibex::bwd_mul(r21xr22,r21,r22); ibex::bwd_mul(r31xr32,r31,r32);
    ibex::bwd_add(A3,r21xr31,r22xr32);
    ibex::bwd_mul(r21xr31,r21,r31);ibex::bwd_mul(r22xr32,r22,r32); ibex::bwd_mul(r23xr33,r23,r33);
    ibex::bwd_add(A2,r11xr31,r12xr32);
    ibex::bwd_mul(r11xr31,r11,r31);ibex::bwd_mul(r12xr32,r12,r32); ibex::bwd_mul(r13xr33,r13,r33);
    ibex::bwd_add(A1,r11xr21,r12xr22);
    ibex::bwd_mul(r11xr21,r11,r21);ibex::bwd_mul(r12xr22,r12,r22); ibex::bwd_mul(r13xr23,r13,r23);
    sqrr13 = sqrr13 & (1-Z1); Z1 = Z1 & (1-sqrr13); // sum one constraint
    sqrr23 = sqrr23 & (1-Z2); Z2 = Z2 & (1-sqrr23);
    sqrr33 = sqrr33 & (1-Z3); Z3 = Z3 & (1-sqrr33);
    sqrr31 = sqrr31 & (1-s1); s1 = s1 & (1-sqrr31);
    sqrr32 = sqrr32 & (1-s2); s2 = s2 & (1-sqrr32);
    sqrr33 = sqrr33 & (1-s3); s3 = s3 & (1-sqrr33);
    ibex::bwd_add(s3,sqrr13,sqrr23);
    ibex::bwd_add(s2,sqrr12,sqrr22);
    ibex::bwd_add(s1,sqrr11,sqrr21);
    ibex::bwd_add(Z3,sqrr31,sqrr32);
    ibex::bwd_add(Z2,sqrr21,sqrr22);
    ibex::bwd_add(Z1,sqrr11,sqrr12);
    ibex::bwd_sqr(sqrr11,r11); ibex::bwd_sqr(sqrr12,r12); ibex::bwd_sqr(sqrr13,r13);
    ibex::bwd_sqr(sqrr21,r21); ibex::bwd_sqr(sqrr22,r22); ibex::bwd_sqr(sqrr23,r23);
    ibex::bwd_sqr(sqrr31,r31); ibex::bwd_sqr(sqrr32,r32); ibex::bwd_sqr(sqrr33,r33);
    phi = phi & atan2(r32,r33);
    teta = teta & (-asin(r31));
    psi = psi & atan2(r21,r11);
  }
}