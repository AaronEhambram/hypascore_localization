#include "hypascore_localization/contractors/Ctc3DRotation.hpp"

int main()
{
  codac::Ctc3DRotation ctc;
  ibex::IntervalVector euler_angles(3);
  ibex::IntervalMatrix R(3,3);
  
  // 1. Test: 0 rotation
  euler_angles[0] = 0;
  euler_angles[1] = 0;
  euler_angles[2] = 0;
  ctc.contract(euler_angles,R);

  std::cout << "Test 1: no rotation" << std::endl; 
  std::cout << "phi: " << euler_angles[0] << "; teta: " << euler_angles[1] << "; psi: " << euler_angles[2] << std::endl
            << R << std::endl << std::endl; 

  // 2. Test: pi/2 for phi
  R = ibex::IntervalMatrix(3,3);
  euler_angles[0] = M_PI/2;
  euler_angles[1] = 0;
  euler_angles[2] = 0;
  ctc.contract(euler_angles,R);

  std::cout << "Test 1: phi pi/2" << std::endl; 
  std::cout << "phi: " << euler_angles[0] << "; teta: " << euler_angles[1] << "; psi: " << euler_angles[2] << std::endl
            << R << std::endl << std::endl;

  // 3. Test: [0,0.3] for teta
  R = ibex::IntervalMatrix(3,3);
  euler_angles[0] = 0;
  euler_angles[1] = ibex::Interval(0,0.3);
  euler_angles[2] = 0;
  ctc.contract(euler_angles,R);

  std::cout << "Test 1: phi pi/2" << std::endl; 
  std::cout << "phi: " << euler_angles[0] << "; teta: " << euler_angles[1] << "; psi: " << euler_angles[2] << std::endl
            << R << std::endl << std::endl;
}