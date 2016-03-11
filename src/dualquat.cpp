#include "dualquat.h"
#include <cmath>
#include <Eigen/Dense>

template <typename T>
Quat<T>& Quat<T>::conjugate() {
  this->x = -this->x; this->y = -this->y; this->z = -this->z;
  return *this;
}

template <typename T>
T Quat<T>::magnitude() const {
  return sqrt(this->w*this->w + this->x*this->x + this->y*this->y + this->z*this->z);
}

template <typename T>
void Quat<T>::normalize() {
  T magnitude = this->magnitude();
  this->w /= magnitude;
  this->x /= magnitude;
  this->y /= magnitude;
  this->z /= magnitude;
}

template <typename T>
Eigen::Matrix<T, 3, 3> Quat<T>::K() const {
  Eigen::Matrix<T, 3, 3> k;
  k <<           (T) 0., -this->vector[2],  this->vector[1],
        this->vector[2],           (T) 0., -this->vector[0],
       -this->vector[1],  this->vector[0],           (T) 0.;
  return k;
}

template <typename T>
Eigen::Matrix<T, 4, 4> Quat<T>::Q() const {
  Eigen::Matrix<T, 4, 4> q;
  q.block(0, 0, 3, 3) = this->K();
  q(0, 0) = q(1, 1) = q(2, 2) = q(3, 3) = this->w;
  q(0, 3) = this->x; q(3, 0) = -this->x;
  q(1, 3) = this->y; q(3, 1) = -this->y;
  q(2, 3) = this->z; q(3, 2) = -this->z;
  return q;
}

template <typename T>
Eigen::Matrix<T, 4, 4> Quat<T>::W() const {
  Eigen::Matrix<T, 4, 4> w;
  w.block(0, 0, 3, 3) = -this->K();
  w(0, 0) = w(1, 1) = w(2, 2) = w(3, 3) = this->w;
  w(0, 3) = this->x; w(3, 0) = -this->x;
  w(1, 3) = this->y; w(3, 1) = -this->y;
  w(2, 3) = this->z; w(3, 2) = -this->z;
  return w;
}

template <typename T>
DualQuat<T>& DualQuat<T>::conjugate() {
  this->r = this->r.conjugate();
  this->d = this->d.conjugate();
  return *this;
}

template <typename T>
T DualQuat<T>::realMagnitude() const {
  return this->r.magnitude();
}

template <typename T>
void DualQuat<T>::normalize() {
  this->r.normalize();
}

template class Quat<float>;
template class DualQuat<float>;
