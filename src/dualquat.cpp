#include "dualquat.h"

template <typename T>
Quat<T>& Quat<T>::conjugate() {
  this->x = -this->x; this->y = -this->y; this->z = -this->z;
  return *this;
}

template class Quat<float>;
template class DualQuat<float>;
