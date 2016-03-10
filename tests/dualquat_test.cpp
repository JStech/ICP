#include "dualquat.h"
#include <iostream>

int main(int argc, char* argv[]) {
  Quat<float> orig_a(1., 2., 3., 4.);
  Quat<float> a(1., 2., 3., 4.);
  Quat<float> orig_b(4., 3., 2., 1.);
  Quat<float> b(4., 3., 2., 1.);
  Quat<float> orig_c(7., 7., 7., 7.);
  Quat<float> c(7., 7., 7., 7.);

  Quat<float> r, e;
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
  if (r == e) std::cout << ":-D division test passed" << std::endl;
  if (r != e) std::cout << ":-( division test failed" << std::endl;
  if (c != orig_c) std::cout << ":-( division changed lhs" << std::endl;

  e.w = 4.; e.x = -3.; e.y = -2.; e.z = -1.;
  r = b.conjugate();
  if (r == e && b == e) std::cout << ":-D conjugate test passed" << std::endl;
  if (r != e || b != e) std::cout << ":-( conjugate test failed" << std::endl;

  return 0;
}
