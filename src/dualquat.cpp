#include "dualquat.h"

template <typename T>
Quat<T>& Quat<T>::operator+=(const Quat<T>& rhs) {
  this->w += rhs.w; this->x += rhs.x; this->y += rhs.y; this->z += rhs.z;
  return *this;
}

template <typename T>
Quat<T> Quat<T>::operator+(const Quat<T>& rhs) {
  *this += rhs;
  return *this;
}

template <typename T>
Quat<T> Quat<T>::operator-() const {
  return Quat<T>(-this->w, -this->x, -this->y, -this->z);
}

template <typename T>
Quat<T>& Quat<T>::operator-=(const Quat<T>& rhs) {
  *this += -rhs;
  return *this;
}

template <typename T>
Quat<T> Quat<T>::operator-(const Quat<T>& rhs) {
  *this -= rhs;
  return *this;
}

template <typename T>
Quat<T>& Quat<T>::operator*=(const Quat<T>& rhs) {
  T w = this->w*rhs.w - this->x*rhs.x - this->y*rhs.y - this->z*rhs.z;
  T x = this->x*rhs.w + this->w*rhs.x - this->z*rhs.y + this->y*rhs.z;
  T y = this->y*rhs.w + this->z*rhs.x + this->w*rhs.y - this->x*rhs.z;
  T z = this->z*rhs.w - this->y*rhs.x + this->x*rhs.y + this->w*rhs.z;
  this->w = w; this->x = x; this->y = y; this->z = z;
  return *this;
}

template <typename T>
Quat<T> Quat<T>::operator*(const Quat<T>& rhs) {
  *this *= rhs;
  return *this;
}

template <typename T>
Quat<T>& Quat<T>::operator/=(const Quat<T>& rhs) {
  T denom = rhs.w*rhs.w + rhs.x*rhs.x + rhs.y*rhs.y + rhs.z*rhs.z;
  T w = (this->w*rhs.w + this->x*rhs.x + this->y*rhs.y + this->z*rhs.z)/denom;
  T x = (this->x*rhs.w - this->w*rhs.x - this->z*rhs.y + this->y*rhs.z)/denom;
  T y = (this->y*rhs.w + this->z*rhs.x - this->w*rhs.y - this->x*rhs.z)/denom;
  T z = (this->z*rhs.w - this->y*rhs.x + this->x*rhs.y - this->w*rhs.z)/denom;
  this->w = w; this->x = x; this->y = y; this->z = z;
  return *this;
}

template <typename T>
Quat<T> Quat<T>::operator/(const Quat<T>& rhs) {
  *this /= rhs;
  return *this;
}

template <typename T>
Quat<T>& Quat<T>::conjugate() {
  this->x = -this->x; this->y = -this->y; this->z = -this->z;
  return *this;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const Quat<T>& rhs) {
  return os << rhs.w << " + " << rhs.x << "i + " << rhs.y << "j + " <<
    rhs.z << "k";
}

template <typename T>
bool Quat<T>::operator==(const Quat<T>& rhs) const {
  return (this->w == rhs.w) && (this->x == rhs.x) &&
    (this->y == rhs.y) && (this->z == rhs.z);
}

template <typename T>
bool Quat<T>::operator!=(const Quat<T>& rhs) const {
  return !(this == rhs);
}
