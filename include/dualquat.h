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

    Quat() {};
    Quat(T w, T x, T y, T z) : w(w), x(x), y(y), z(z) {};

    Quat& operator+=(const Quat& rhs);
    Quat& operator-=(const Quat& rhs);
    Quat& operator*=(const Quat& rhs);
    Quat& operator/=(const Quat& rhs);
    Quat operator-() const;
    Quat operator+(const Quat& rhs) const;
    Quat operator-(const Quat& rhs) const;
    Quat operator*(const Quat& rhs) const;
    Quat operator/(const Quat& rhs) const;
    Quat& conjugate();
    bool operator==(const Quat& rhs) const;
    bool operator!=(const Quat& rhs) const;
    friend std::ostream& operator<<(std::ostream& os, const Quat& rhs);
};

template <typename T>
inline bool operator==(const Quat<T>& lhs, const Quat<T>& rhs);

template <typename T>
inline bool operator!=(const Quat<T>& lhs, const Quat<T>& rhs);

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

template class Quat<float>;
template class DualQuat<float>;

#endif // DUALQUAT_H
