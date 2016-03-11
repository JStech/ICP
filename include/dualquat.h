#ifndef DUALQUAT_H
#define DUALQUAT_H
#include <iostream>

template <typename T>
class Quat {
  public:
    union {
      T data[4];
      struct {
        T w; T x; T y; T z;
      };
    };

    Quat(){};
    Quat(T w, T x, T y, T z) :
      w(w), x(x), y(y), z(z) {}

    Quat& operator+=(const Quat& rhs) {
      this->w += rhs.w; this->x += rhs.x; this->y += rhs.y; this->z += rhs.z;
      return *this;
    }

    friend Quat operator+(Quat lhs, const Quat& rhs) {
      lhs += rhs;
      return lhs;
    }

    Quat operator-() const {
      return Quat(-this->w, -this->x, -this->y, -this->z);
    }

    Quat& operator-=(const Quat& rhs) {
      *this += -rhs;
      return *this;
    }

    friend Quat operator-(Quat lhs, const Quat& rhs) {
      lhs -= rhs;
      return lhs;
    }

    Quat& operator*=(const Quat& rhs) {
      T w = this->w*rhs.w - this->x*rhs.x - this->y*rhs.y - this->z*rhs.z;
      T x = this->x*rhs.w + this->w*rhs.x - this->z*rhs.y + this->y*rhs.z;
      T y = this->y*rhs.w + this->z*rhs.x + this->w*rhs.y - this->x*rhs.z;
      T z = this->z*rhs.w - this->y*rhs.x + this->x*rhs.y + this->w*rhs.z;
      this->w = w; this->x = x; this->y = y; this->z = z;
      return *this;
    }

    friend Quat operator*(Quat lhs, const Quat& rhs) {
      lhs *= rhs;
      return lhs;
    }

    Quat& operator/=(const Quat& rhs) {
      T denom = rhs.w*rhs.w + rhs.x*rhs.x + rhs.y*rhs.y + rhs.z*rhs.z;
      T w = (this->w*rhs.w + this->x*rhs.x + this->y*rhs.y + this->z*rhs.z)/denom;
      T x = (this->x*rhs.w - this->w*rhs.x - this->z*rhs.y + this->y*rhs.z)/denom;
      T y = (this->y*rhs.w + this->z*rhs.x - this->w*rhs.y - this->x*rhs.z)/denom;
      T z = (this->z*rhs.w - this->y*rhs.x + this->x*rhs.y - this->w*rhs.z)/denom;
      this->w = w; this->x = x; this->y = y; this->z = z;
      return *this;
    }

    friend Quat operator/(Quat lhs, const Quat& rhs) {
      lhs /= rhs;
      return lhs;
    }

    Quat& conjugate();

    friend std::ostream& operator<<(std::ostream& os, const Quat& rhs) {
      return os << rhs.w << " + " << rhs.x << "i + " << rhs.y << "j + " <<
        rhs.z << "k";
    }
};

template <typename T>
inline bool operator==(const Quat<T>& lhs, const Quat<T>& rhs) {
  return (lhs.w == rhs.w) && (lhs.x == rhs.x) &&
    (lhs.y == rhs.y) && (lhs.z == rhs.z);
}

template <typename T>
inline bool operator!=(const Quat<T>& lhs, const Quat<T>& rhs) {
  return !(lhs == rhs);
}

template <typename T>
class DualQuat {
  public:
    Quat<T> r;
    Quat<T> d;

    DualQuat();
    DualQuat(Quat<T> r, Quat<T> d) : r(r), d(d) {}

    DualQuat& operator+=(const DualQuat& rhs) {
      this->r += rhs.r;
      this->d += rhs.d;
      return *this;
    }

    friend DualQuat operator+(const DualQuat lhs, const DualQuat& rhs) {
      lhs += rhs;
      return lhs;
    }

    DualQuat& operator-() {
      this->r = -this->r;
      this->d = -this->d;
      return *this;
    }

    DualQuat& operator-=(DualQuat& rhs) {
      *this += -rhs;
      return *this;
    }

    friend DualQuat operator-(const DualQuat lhs, const DualQuat& rhs) {
      lhs -= rhs;
      return lhs;
    }

    DualQuat& operator*=(const DualQuat& rhs) {
      this->r = this->r * rhs.r;
      this->d = this->r * rhs.d + this->d * rhs.r;
      return *this;
    }

    friend DualQuat operator*(const DualQuat lhs, const DualQuat& rhs) {
      lhs *= rhs;
      return lhs;
    }

    DualQuat& conjugate() {
      this->r = this->r.conjugate();
      this->d = this->d.conjugate();
      return *this;
    }
};

#endif // DUALQUAT_H
