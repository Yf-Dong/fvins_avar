#ifndef FVINS_TIME_TIME_HPP_
#define FVINS_TIME_TIME_HPP_

#ifdef _MSC_VER
#pragma warning(disable: 4244)
#pragma warning(disable: 4661)
#endif

#ifdef _WIN32
#include <windows.h>
#endif

/*********************************************************************
 ** Headers
 *********************************************************************/

#include <iostream>
#include <cmath>
#include <chrono>
#include "Duration.h"

/*********************************************************************
 ** Cross Platform Headers
 *********************************************************************/

#ifdef WIN32
#include <sys/timeb.h>
#else
#include <sys/time.h>
#endif

namespace Fvins {

class NoHighPerformanceTimersException : std::runtime_error {
 public:
  NoHighPerformanceTimersException()
      : std::runtime_error("This windows platform does not "
                           "support the high-performance timing api.") {
  }
};

/*********************************************************************
 ** Functions
 *********************************************************************/

void normalizeSecNSec(uint64_t& sec, uint64_t& nsec);
void normalizeSecNSec(uint32_t& sec, uint32_t& nsec);
void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec);

/*********************************************************************
 ** Time Classes
 *********************************************************************/
class TicToc {
 public:
  TicToc() {tic();};
 private:
  void tic() {start_ = std::chrono::system_clock::now();}
  
  double toc() {
    end_ = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_ - start_;
    return elapsed_seconds.count() * 1000;
  }
  
  std::chrono::time_point<std::chrono::system_clock> start_, end_;
};

template<class T, class D>
class TimeBase {
 public:
  uint32_t sec, nsec;

  TimeBase()
      : sec(0),
        nsec(0) {
  }
  TimeBase(uint32_t _sec, uint32_t _nsec)
      : sec(_sec),
        nsec(_nsec) {
    normalizeSecNSec(sec, nsec);
  }
  explicit TimeBase(double t) {
    fromSec(t);
  }
  ~TimeBase() {
  }
  D operator-(const T &rhs) const;
  T operator+(const D &rhs) const;
  T operator-(const D &rhs) const;
  T& operator+=(const D &rhs);
  T& operator-=(const D &rhs);
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
  T& fromSec(double t) {
    sec = (uint32_t) floor(t);
    nsec = (uint32_t) std::round((t - sec) * 1e9);
    return *static_cast<T*>(this);
  }

  uint64_t toNSec() const {
    return (uint64_t) sec * 1000000000ull + (uint64_t) nsec;
  }
  T& fromNSec(uint64_t t);

  inline bool isZero() const {
    return sec == 0 && nsec == 0;
  }
  inline bool is_zero() const {
    return isZero();
  }

};



class Time : public TimeBase<Time, Duration> {
 public:
  Time()
      : TimeBase<Time, Duration>() {
  }

  Time(uint32_t _sec, uint32_t _nsec)
      : TimeBase<Time, Duration>(_sec, _nsec) {
  }

  explicit Time(double t) {
    fromSec(t);
  }

  static Time now();
  static bool sleepUntil(const Time& end);
  static void init();
  static void shutdown();
  static void setNow(const Time& new_now);
  static bool useSystemTime();
  static bool isSimTime();
  static bool isSystemTime();
  static bool isValid();
  static bool waitForValid();
  static bool waitForValid(const Fvins::WallDuration& timeout);
};

class WallTime : public TimeBase<WallTime, WallDuration> {
 public:
  WallTime()
      : TimeBase<WallTime, WallDuration>() {
  }

  WallTime(uint32_t _sec, uint32_t _nsec)
      : TimeBase<WallTime, WallDuration>(_sec, _nsec) {
  }

  explicit WallTime(double t) {
    fromSec(t);
  }

  static WallTime now();

  static bool sleepUntil(const WallTime& end);
  
  static bool isSystemTime() {
    return true;
  }
};

// IMPLEMENTATION
template<class T, class D>
T& TimeBase<T, D>::fromNSec(uint64_t t) {
  sec = (int32_t)(t / 1000000000);
  nsec = (int32_t)(t % 1000000000);

  normalizeSecNSec(sec, nsec);

  return *static_cast<T*>(this);
}

template<class T, class D>
D TimeBase<T, D>::operator-(const T &rhs) const {
  return D((int32_t) sec - (int32_t) rhs.sec,
           (int32_t) nsec - (int32_t) rhs.nsec);  
}

template<class T, class D>
T TimeBase<T, D>::operator-(const D &rhs) const {
  return *static_cast<const T*>(this) + (-rhs);
}

template<class T, class D>
T TimeBase<T, D>::operator+(const D &rhs) const {
  int64_t sec_sum = (int64_t) sec + (int64_t) rhs.sec;
  int64_t nsec_sum = (int64_t) nsec + (int64_t) rhs.nsec;
  normalizeSecNSecUnsigned(sec_sum, nsec_sum);
  return T((uint32_t) sec_sum, (uint32_t) nsec_sum);
}

template<class T, class D>
T& TimeBase<T, D>::operator+=(const D &rhs) {
  *this = *this + rhs;
  return *static_cast<T*>(this);
}

template<class T, class D>
T& TimeBase<T, D>::operator-=(const D &rhs) {
  *this += (-rhs);
  return *static_cast<T*>(this);
}

template<class T, class D>
bool TimeBase<T, D>::operator==(const T &rhs) const {
  return sec == rhs.sec && nsec == rhs.nsec;
}

template<class T, class D>
bool TimeBase<T, D>::operator<(const T &rhs) const {
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec < rhs.nsec)
    return true;
  return false;
}

template<class T, class D>
bool TimeBase<T, D>::operator>(const T &rhs) const {
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec > rhs.nsec)
    return true;
  return false;
}

template<class T, class D>
bool TimeBase<T, D>::operator<=(const T &rhs) const {
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec <= rhs.nsec)
    return true;
  return false;
}

template<class T, class D>
bool TimeBase<T, D>::operator>=(const T &rhs) const {
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec >= rhs.nsec)
    return true;
  return false;
}

std::ostream &operator <<(std::ostream &os, const Time &rhs);
std::ostream &operator <<(std::ostream &os, const WallTime &rhs);

} //namespace Fvins
#endif // FVINS_TIME_TIME_BASE_HPP_