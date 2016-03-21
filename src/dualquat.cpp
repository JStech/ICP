#include "dualquat.h"
#include <cmath>
#include <Eigen/Dense>

template <typename T>
Quat<T>::Quat(pcl::PointXYZ p) {
  this->w = (T) 0.;
  this->x = (T) p.x;
  this->y = (T) p.y;
  this->z = (T) p.z;
  *this *= (T) 0.5;
}

template <typename T>
Quat<T>::Quat(Eigen::Vector3d vector, T rotation_radians) {
  vector = vector / vector.norm();
  T s = sin(rotation_radians/2);
  this->w = cos(rotation_radians/2);
  this->x = vector[0] * s;
  this->y = vector[1] * s;
  this->z = vector[2] * s;
  this->normalize();
}

template <typename T>
Quat<T>::Quat(Eigen::Matrix<T, 4, 1> vector) {
  this->w = vector[3];
  this->x = vector[0];
  this->y = vector[1];
  this->z = vector[2];
}

template <typename T>
Quat<T>& Quat<T>::Identity() {
  this->w = (T) 1.0;
  this->x = this->y = this->z = (T) 0.0;
  return *this;
}

template <typename T>
Quat<T> Quat<T>::conjugate() const {
  return Quat<T>(this->w, -this->x, -this->y, -this->z);
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
Eigen::Matrix<T, 4, 4> Quat<T>::R() const {
  return this->W().transpose() * this->Q();
}

template <typename T>
Eigen::Matrix<T, 3, 3> Quat<T>::Rot() const {
  T norm = this->magnitude();
  T w = this->w/norm;
  T x = this->x/norm;
  T y = this->y/norm;
  T z = this->z/norm;
  Eigen::Matrix<T, 3, 3> r;
  r << 1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w,
           2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w,
           2*x*z - 2*w*y,     2*w*x + 2*y*z, 1 - 2*x*x - 2*y*y;
  return r;
}

template <typename T>
DualQuat<T>::DualQuat(pcl::PointXYZ point) {
  this->d = Quat<T>(point);
  this->r = Quat<T>().Identity();
}

template <typename T>
DualQuat<T>& DualQuat<T>::Identity() {
  this->r.w = 1.;
  this->r.x = this->r.y = this->r.z = 0.;
  this->d.w = this->d.x = this->d.y = this->d.z = 0.;
  return *this;
}

template <typename T>
DualQuat<T> DualQuat<T>::conjugate() const {
  return DualQuat<T>(this->r.conjugate(), this->d.conjugate());
}

template <typename T>
T DualQuat<T>::realMagnitude() const {
  return this->r.magnitude();
}

template <typename T>
T DualQuat<T>::sumSq() const {
  return this->r.w*this->r.w + this->r.x*this->r.x +
    this->r.y*this->r.y + this->r.z*this->r.z +
    this->d.w*this->d.w + this->d.x*this->d.x +
    this->d.y*this->d.y + this->d.z*this->d.z;
}

template <typename T>
void DualQuat<T>::normalize() {
  this->r.normalize();
}

template <typename T>
Eigen::Matrix<T, 4, 4> DualQuat<T>::Matrix() const {
  Eigen::Matrix<T, 4, 4> out = this->r.W().transpose()*this->r.Q();
  out.block(0, 3, 3, 1) = (this->getTranslation()).template cast<T>();
  return out;
}

template <typename T>
Eigen::Vector3d DualQuat<T>::getTranslation() const {
  Quat<T> t = this->r.conjugate();
  t = (((T) 2.)*this->d)*t;
  Eigen::Vector3d ret(t.x, t.y, t.z);
  return ret;
}

template class Quat<float>;
template class DualQuat<float>;
