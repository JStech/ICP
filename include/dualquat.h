#ifndef DUALQUAT_H
#define DUALQUAT_H

template <typename T>
class Quat {
  public:
    union {
      T data[4];
      struct {
        T w; T x; T y; T z;
      }
    };

    Quat();
    Quat(T w, T x, T y, T z) :
      w(w), x(x), y(y), z(z) {}

    Quat& operator+=(const Quat& rhs) {
      this.w += rhs.w; this.x += rhs.x; this.y += rhs.y; this.z += rhs.z;
      return *this;
    }

    friend Quat operator+(const Quat lhs, const Quat& rhs) {
      lhs += rhs;
      return lhs;
    }

    Quat& operator-() {
      this.w = -this.w; this.x = -this.x; this.y = -this.y; this.z = -this.z;
      return *this;
    }

    Quat& operator-=(const Quat& rhs) {
      this += -rhs;
    }

    friend Quat operator-(const Quat lhs, const Quat& rhs) {
      lhs -= rhs;
      return lhs;
    }

    Quat& operator*=(const Quat& rhs) {
      this.w = this.w*rhs.w - this.x*rhs.x - this.y*rhs.y - this.z*rhs.z;
      this.x = this.x*rhs.w + this.w*rhs.x - this.z*rhs.y + this.y*rhs.z;
      this.y = this.y*rhs.w + this.z*rhs.x + this.w*rhs.y - this.x*rhs.z;
      this.y = this.z*rhs.w - this.y*rhs.x + this.x*rhs.y + this.w*rhs.z;
      return *this;
    }

    friend Quat operator*(const Quat lhs, const Quat& rhs) {
      lhs *= rhs;
      return lhs;
    }

    Quat& operator/=(const Quat& rhs) {
      T denom = rhs.w*rhs.w + rhs.x*rhs.x + rhs.y*rhs.y + rhs.z*rhs.z;
      this.w = (this.w*rhs.w + this.x*rhs.x + this.y*rhs.y + this.z*rhs.z)/denom;
      this.x = (this.x*rhs.w - this.w*rhs.x - this.z*rhs.y + this.y*rhs.z)/denom;
      this.y = (this.y*rhs.w + this.z*rhs.x - this.w*rhs.y - this.x*rhs.z)/denom;
      this.y = (this.z*rhs.w - this.y*rhs.x + this.x*rhs.y - this.w*rhs.z)/denom;
      return *this;
    }

    friend Quat operator/(const Quat lhs, const Quat& rhs) {
      lhs /= rhs;
      return lhs;
    }

    Quat& conjugate() {
      this.x = -this.x; this.y = -this.y; this.z = -this.z;
      return *this;
    }
}

template <typename T>
class DualQuat {
  public:
    Quat<T> r;
    Quat<T> d;

    DualQuat();
    DualQuat(Quat<T> r, Quat<T> d) : r(r), d(d) {}

    DualQuat& operator+=(const DualQuat& rhs) {
      this.r += rhs.r;
      this.d += rhs.d;
      return *this;
    }

    friend DualQuat operator+(const DualQuat lhs, const DualQuat& rhs) {
      lhs += rhs;
      return lhs;
    }

    DualQuat& operator-() {
      this.r -= rhs.r;
      this.d -= rhs.d;
      return *this;
    }

    DualQuat& operator-=(const DualQuat& rhs) {
      this += -rhs;
    }

    friend DualQuat operator-(const DualQuat lhs, const DualQuat& rhs) {
      lhs -= rhs;
      return lhs;
    }

    DualQuat& operator*=(const DualQuat& rhs) {
      this.r = this.r * rhs.r;
      this.d = this.r * rhs.d + this.d * rhs.r;
      return *this;
    }

    friend DualQuat operator*(const DualQuat lhs, const DualQuat& rhs) {
      lhs *= rhs;
      return lhs;
    }

    DualQuat& conjugate() {
      this.r = this.r.conjugate();
      this.d = this.d.conjugate();
      return *this;
    }
}

#endif // DUALQUAT_H
