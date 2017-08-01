// Copyright 2017 John Stechschulte
#include "include/dualquat.h"
#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>

float epsilon = 1e-6;
float pi = M_PI;

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

  e.w = e.x = e.y = e.z = 21.;
  r = 3.f*c;
  if (r == e) std::cout << ":-D scalar multiplication test passed" << std::endl;
  if (r != e) std::cout << ":-( scalar multiplication test failed" << std::endl;
  if (c != orig_c) std::cout << ":-( scalar multiplication changed lhs" << std::endl;

  Eigen::Matrix<float, 4, 4> M;
  M <<  1., -4.,  3.,  2.,
        4.,  1., -2.,  3.,
       -3.,  2.,  1.,  4.,
       -2., -3., -4.,  1.;
  if (orig_a.Q() == M) std::cout << ":-D Q matrix test passed" << std::endl;
  if (orig_a.Q() != M) std::cout << ":-( Q matrix test failed" << std::endl;

  M <<  1.,  4., -3.,  2.,
       -4.,  1.,  2.,  3.,
        3., -2.,  1.,  4.,
       -2., -3., -4.,  1.;
  if (orig_a.W() == M) std::cout << ":-D W matrix test passed" << std::endl;
  if (orig_a.W() != M) std::cout << ":-( W matrix test failed" << std::endl;

  M << -20./30.,   4./30.,  22./30., 0.,
        20./30., -10./30.,  20./30., 0.,
        10./30.,  28./30.,   4./30., 0.,
             0.,       0.,       0., 1.;
  if ((a.R() - M).array().abs().sum() < epsilon)
    std::cout << ":-D R matrix test passed" << std::endl;
  if ((a.R() - M).array().abs().sum() > epsilon)
    std::cout << ":-( R matrix test failed" << std::endl;

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
  if (R == C) std::cout << ":-D addition test passed" << std::endl;
  if (R != C) std::cout << ":-( addition test failed" << std::endl;
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

  E.r.w = E.r.x = E.r.y = E.r.z = E.d.w = E.d.x = E.d.y = E.d.z = 27.f;
  R = 3.f*C;
  if (R == E) std::cout << ":-D scalar multiplication test passed" << std::endl;
  if (R != E) std::cout << ":-( scalar multiplication test failed" << std::endl;
  if (C != orig_C) std::cout << ":-( scalar multiplication changed lhs" << std::endl;

  /////////////////// putting it all together: transformations ///////////////////

  // based on: http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/other
                                       //  /dualQuaternion/example/index.htm
  typedef DualQuat<float> DQf;
  typedef Quat<float> Qf;

  DQf Q_pc(Qf().Identity(), Qf(pcl::PointXYZ(0., 0., 100.)));
  E.r.w = 1.0;
  E.r.x = E.r.y = E.r.z = E.d.w = E.d.x = E.d.y = 0.0;
  E.d.z = 50.0;
  if (Q_pc == E) std::cout << ":-D constructing Q_pc passed" << std::endl;
  if (Q_pc != E) std::cout << ":-( constructing Q_pc failed" << std::endl;

  DQf Q_ch(Qf(Eigen::Vector3d(0., 1., 0.), pi/6), Qf(0., 0., 0., 0.));
  DQf Q_ph = Q_pc * Q_ch;
  E.r.w = cos(pi/12); E.r.x = 0.; E.r.y = sin(pi/12); E.r.z = 0.;
  E.d.w = 0.; E.d.x = -12.940952255126037;
  E.d.y = 0.; E.d.z = 48.296291314453413;
  if ((Q_ph - E).sumSq() < epsilon) std::cout << ":-D calculating Q_ph passed" << std::endl;
  if ((Q_ph - E).sumSq() > epsilon) std::cout << ":-( calculating Q_ph failed" << std::endl;

  std::cout << std::endl << "Transformations" << std::endl << std::endl;

  // let's do something more complex
  DQf Q_p1(pcl::PointXYZ(1., 2., 3.));
  E.r.w = 1.;
  E.r.x = E.r.y = E.r.z = E.d.w = 0.;
  E.d.x = 0.5; E.d.y = 1.; E.d.z = 1.5;
  if (Q_p1 == E) std::cout << ":-D constructing Q_p1 passed" << std::endl;
  if (Q_p1 != E) std::cout << ":-( constructing Q_p1 failed" << std::endl;

  // translate by (1, -1, 1) to 2, 1, 4
  DQf Q_t1(Qf().Identity(), 0.5f*Qf(0., 1., -1., 1.));
  E.r.w = 1.;
  E.r.x = E.r.y = E.r.z = E.d.w = 0.;
  E.d.x = 0.5; E.d.y = -0.5; E.d.z = 0.5;
  if (Q_t1 == E) std::cout << ":-D constructing Q_t1 passed" << std::endl;
  if (Q_t1 != E) std::cout << ":-( constructing Q_t1 failed" << std::endl;

  // rotate by pi/5 around (-1., 0.5, 0.75) to
  //    2.005723993542627 3.289322031376978 2.481417303805519
  DQf Q_r1(Qf(Eigen::Vector3d(-1., 0.5, 0.75), pi/5), Qf());
  E.r.w = 0.951056516295154; E.r.x = -0.229532061091648;
  E.r.y = 0.114766030545824; E.r.z = 0.172149045818736;
  E.d.w = E.d.x = E.d.y = E.d.z = 0.;
  if ((Q_r1 - E).sumSq() < epsilon) std::cout << ":-D constructing Q_r1 passed" << std::endl;
  if ((Q_r1 - E).sumSq() > epsilon) std::cout << ":-( constructing Q_r1 failed" << std::endl;

  R = Q_r1 * Q_t1 * Q_p1;
  Eigen::Vector3d t(2.005723993542627, 3.289322031376978, 2.481417303805519);
  if ((t - R.getTranslation()).norm() < epsilon) {
    std::cout << ":-D entire translation passed" << std::endl;
  }
  if ((t - R.getTranslation()).norm() > epsilon) {
    std::cout << ":-( entire translation failed" << std::endl;
  }

  return 0;
}
