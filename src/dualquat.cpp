#include "dualquat.h"
#include <cmath>

template <typename T>
Quat<T>& Quat<T>::conjugate() {
  this->x = -this->x; this->y = -this->y; this->z = -this->z;
  return *this;
}

template <typename T>
T Quat<T>::magnitude() {
  return this->w*this->w + this->x*this->x + this->y*this->y + this->z*this->z;
}

template <typename T>
DualQuat<T>& DualQuat<T>::conjugate() {
  this->r = this->r.conjugate();
  this->d = this->d.conjugate();
  return *this;
}

template <typename T>
T DualQuat<T>::realMagnitude() {
  return this->r.magnitude();
}

template class Quat<float>;
template class DualQuat<float>;
