#ifndef FVINS_TIME_DURATION_HPP_
#define FVINS_TIME_DURATION_HPP_

#ifdef _MSC_VER
#pragma warning(disable: 4244)
#pragma warning(disable: 4661)
#endif

#include <iostream>
#include <math.h>
#include <stdexcept>
#include <climits>
#include <stdint.h>


namespace Fvins{
void normalizeSecNSecSigned(int64_t& sec, int64_t& nsec);
void normalizeSecNSecSigned(int32_t& sec, int32_t& nsec);

template<class T>
class DurationBase {
 public:
  int32_t sec, nsec;
  DurationBase()
      : sec(0),
        nsec(0) {
  }
  DurationBase(int32_t _sec, int32_t _nsec);
  explicit DurationBase(double t) {
    fromSec(t);
  }
  ;
  ~DurationBase() {
  }
  T operator+(const T &rhs) const;
  T operator-(const T &rhs) const;
  T operator-() const;
  T operator*(double scale) const;
  T& operator+=(const T &rhs);
  T& operator-=(const T &rhs);
  T& operator*=(double scale);
  bool operator==(const T &rhs) const;
  inline bool operator!=(const T &rhs) const {
    return !(*static_cast<const T*>(this) == rhs);
  }
  bool operator>(const T &rhs) const;
  bool operator<(const T &rhs) const;
  bool operator>=(const T &rhs) const;
  bool operator<=(const T &rhs) const;
  double toSec() const {
    return (double) sec + 1e-9 * (double) nsec;
  }
  ;
	int64_t toNSec() const {
    return (int64_t) sec * 1000000000ll + (int64_t) nsec;
  }
  ;
  T& fromSec(double t);
  T& fromNSec(int64_t t);
  bool isZero();
};

class Rate;

class Duration : public DurationBase<Duration> {
 public:
  Duration()
      : DurationBase<Duration>() {
  }

  Duration(int32_t _sec, int32_t _nsec)
      : DurationBase<Duration>(_sec, _nsec) {
  }

  explicit Duration(double t) {
    fromSec(t);
  }
  explicit Duration(const Rate&);
  bool sleep() const;
};

extern const Duration DURATION_MAX;
extern const Duration DURATION_MIN;

class WallDuration : public DurationBase<WallDuration> {
 public:
  WallDuration()
      : DurationBase<WallDuration>() {
  }

  WallDuration(int32_t _sec, int32_t _nsec)
      : DurationBase<WallDuration>(_sec, _nsec) {
  }

  explicit WallDuration(double t) {
    fromSec(t);
  }
  explicit WallDuration(const Rate&);
  bool sleep() const;
};

std::ostream &operator <<(std::ostream &os, const Duration &rhs);
std::ostream &operator <<(std::ostream &os, const WallDuration &rhs);

// IMPLEMENTATION
template<class T>
DurationBase<T>::DurationBase(int32_t _sec, int32_t _nsec)
    : sec(_sec),
      nsec(_nsec) {
  normalizeSecNSecSigned(sec, nsec);
}

template<class T>
T& DurationBase<T>::fromSec(double d) {
#ifdef HAVE_TRUNC
  sec = (int32_t)trunc(d);
#else
  if (d >= 0.0)
    sec = (int32_t) floor(d);
  else
    sec = (int32_t) floor(d) + 1;
#endif
  nsec = (int32_t)((d - (double) sec) * 1000000000);
  return *static_cast<T*>(this);
}

template<class T>
T& DurationBase<T>::fromNSec(int64_t t) {
  sec = (int32_t)(t / 1000000000);
  nsec = (int32_t)(t % 1000000000);

  normalizeSecNSecSigned(sec, nsec);

  return *static_cast<T*>(this);
}

template<class T>
T DurationBase<T>::operator+(const T &rhs) const {
  return T(sec + rhs.sec, nsec + rhs.nsec);
}

template<class T>
T DurationBase<T>::operator*(double scale) const {
  return T(toSec() * scale);
}

template<class T>
T DurationBase<T>::operator-(const T &rhs) const {
  return T(sec - rhs.sec, nsec - rhs.nsec);
}

template<class T>
T DurationBase<T>::operator-() const {
  return T(-sec, -nsec);
}

template<class T>
T& DurationBase<T>::operator+=(const T &rhs) {
  *this = *this + rhs;
  return *static_cast<T*>(this);
}

template<class T>
T& DurationBase<T>::operator-=(const T &rhs) {
  *this += (-rhs);
  return *static_cast<T*>(this);
}

template<class T>
T& DurationBase<T>::operator*=(double scale) {
  fromSec(toSec() * scale);
  return *static_cast<T*>(this);
}

template<class T>
bool DurationBase<T>::operator<(const T &rhs) const {
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec < rhs.nsec)
    return true;
  return false;
}

template<class T>
bool DurationBase<T>::operator>(const T &rhs) const {
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec > rhs.nsec)
    return true;
  return false;
}

template<class T>
bool DurationBase<T>::operator<=(const T &rhs) const {
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec <= rhs.nsec)
    return true;
  return false;
}

template<class T>
bool DurationBase<T>::operator>=(const T &rhs) const {
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec >= rhs.nsec)
    return true;
  return false;
}

template<class T>
bool DurationBase<T>::operator==(const T &rhs) const {
  return sec == rhs.sec && nsec == rhs.nsec;
}

template<class T>
bool DurationBase<T>::isZero() {
  return sec == 0 && nsec == 0;
}

}

#endif //FVINS_TIME_DURATION_HPP_