#include "dualquat.h"
#include <iostream>
#include <cmath>

float epsilon = 1e-6;

int main(int argc, char* argv[]) {
  Quat<float> orig_a(1., 2., 3., 4.);
  Quat<float> a(1., 2., 3., 4.);
  Quat<float> orig_b(4., 3., 2., 1.);
  Quat<float> b(4., 3., 2., 1.);
  Quat<float> orig_c(7., 7., 7., 7.);
  Quat<float> c(7., 7., 7., 7.);
  Quat<float> r, e;
  std::cout << a << std::endl << b << std::endl << c << std::endl << std::endl;

  e.w = e.x = e.y = e.z = 5.;
  r = a+b;
  if (r == e) std::cout << ":-D addition test passed" << std::endl;
  if (r != e) std::cout << ":-( addition test failed" << std::endl;
  if (a != orig_a) std::cout << ":-( addition changed lhs" << std::endl;

  e.w = -4.; e.x = -3.; e.y = -2.; e.z = -1.;
  r = -b;
  if (r == e) std::cout << ":-D negation test passed" << std::endl;
  if (r != e) std::cout << ":-( negation test failed" << std::endl;
  if (b != orig_b) std::cout << ":-( negation changed rhs" << std::endl;

  e.w = 3.; e.x = 4.; e.y = 5.; e.z = 6.;
  r = c - b;
  if (r == e) std::cout << ":-D subtraction test passed" << std::endl;
  if (r != e) std::cout << ":-( subtraction test failed" << std::endl;
  if (c != orig_c) std::cout << ":-( subtraction changed lhs" << std::endl;

  e.w = -12.; e.x = 6.; e.y = 24.; e.z = 12.;
  r = a*b;
  if (r == e) std::cout << ":-D multiplication test passed" << std::endl;
  if (r != e) std::cout << ":-( multiplication test failed" << std::endl;
  if (a != orig_a) std::cout << ":-( multiplication changed lhs" << std::endl;

  e.w = 70./30.; e.x = 0.; e.y = 28./30.; e.z = 14./30.;
  r = c/b;
  if ((r - e).magnitude() < epsilon)
    std::cout << ":-D division test passed" << std::endl;
  if ((r - e).magnitude() > epsilon)
    std::cout << ":-( division test failed" << std::endl;
  if (c != orig_c) std::cout << ":-( division changed lhs" << std::endl;

  e.w = 4.; e.x = -3.; e.y = -2.; e.z = -1.;
  r = b.conjugate();
  if (r == e && b == e) std::cout << ":-D conjugate test passed" << std::endl;
  if (r != e || b != e) std::cout << ":-( conjugate test failed" << std::endl;

  if (std::abs(a.magnitude() - sqrt(30.)) < epsilon)
    std::cout << ":-D magnitude test passed" << std::endl;
  if (std::abs(a.magnitude() - sqrt(30.)) > epsilon)
    std::cout << ":-( magnitude test failed" << std::endl;

  e.w = 1./sqrt(30.); e.x = 2./sqrt(30.); e.y = 3./sqrt(30.); e.z = 4./sqrt(30.);
  a.normalize();
  if ((a - e).magnitude() < epsilon)
    std::cout << ":-D normalization test passed" << std::endl;
  if ((a - e).magnitude() > epsilon)
    std::cout << ":-( normalization test failed" << std::endl;

  DualQuat<float> orig_A(Quat<float>(1., 2., 3., 4.), Quat<float>(5., 6., 7., 8.));
  DualQuat<float> A(Quat<float>(1., 2., 3., 4.), Quat<float>(5., 6., 7., 8.));
  DualQuat<float> orig_B(Quat<float>(8., 7., 6., 5.), Quat<float>(4., 3., 2., 1.));
  DualQuat<float> B(Quat<float>(8., 7., 6., 5.), Quat<float>(4., 3., 2., 1.));
  DualQuat<float> orig_C(Quat<float>(9., 9., 9., 9.), Quat<float>(9., 9., 9., 9.));
  DualQuat<float> C(Quat<float>(9., 9., 9., 9.), Quat<float>(9., 9., 9., 9.));
  DualQuat<float> R, E;
  std::cout << std::endl << A << std::endl << B << std::endl << C << std::endl
    << std::endl;

  R = A+B;
  if (R==C) std::cout << ":-D addition test passed" << std::endl;
  if (R!=C) std::cout << ":-( addition test failed" << std::endl;
  if (A != orig_A) std::cout << ":-( addition changed lhs" << std::endl;

  E.r.w = -8.; E.r.x = -7.; E.r.y = -6.; E.r.z = -5.;
  E.d.w = -4.; E.d.x = -3.; E.d.y = -2.; E.d.z = -1.;
  R = -B;
  if (R == E) std::cout << ":-D negation test passed" << std::endl;
  if (R != E) std::cout << ":-( negation test failed" << std::endl;
  if (B != orig_B) std::cout << ":-( negation changed rhs" << std::endl;

  R = C - B;
  if (R == A) std::cout << ":-D subtraction test passed" << std::endl;
  if (R != A) std::cout << ":-( subtraction test failed" << std::endl;
  if (C != orig_C) std::cout << ":-( subtraction changed lhs" << std::endl;

  E.r.w =  -72.; E.r.x =  18.; E.r.y =  54.; E.r.z =  36.;
  E.d.w = -216.; E.d.x = 108.; E.d.y = 180.; E.d.z = 144.;
  R = A*C;
  if (R == E) std::cout << ":-D multiplication test passed" << std::endl;
  if (R != E) std::cout << ":-( multiplication test failed" << std::endl;
  if (A != orig_A) std::cout << ":-( multiplication changed lhs" << std::endl;

  if (std::abs(B.realMagnitude() - sqrt(174.)) < epsilon)
    std::cout << ":-D magnitude test passed" << std::endl;
  if (std::abs(B.realMagnitude() - sqrt(174.)) > epsilon)
    std::cout << ":-( magnitude test failed" << std::endl;

  E.r.w = 1.; E.r.x = -2.; E.r.y = -3.; E.r.z = -4.;
  E.d.w = 5.; E.d.x = -6.; E.d.y = -7.; E.d.z = -8.;
  R = A.conjugate();
  if (R == E && A == E) std::cout << ":-D conjugate test passed" << std::endl;
  if (R != E || A != E) std::cout << ":-( conjugate test failed" << std::endl;

  E.r.w = 8./sqrt(174.); E.r.x = 7./sqrt(174.); E.r.y = 6./sqrt(174.); E.r.z = 5./sqrt(174.);
  E.d.w = 4./sqrt(174.); E.d.x = 3./sqrt(174.); E.d.y = 2./sqrt(174.); E.d.z = 1./sqrt(174.);
  B.normalize();
  if ((B - E).realMagnitude() < epsilon)
    std::cout << ":-D normalization test passed" << std::endl;
  if ((B - E).realMagnitude() > epsilon)
    std::cout << ":-( normalization test failed" << std::endl;
  return 0;
}
