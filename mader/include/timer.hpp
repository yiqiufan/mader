#ifndef TIMER_HPP_
#define TIMER_HPP_

#include "elapsed_timer.hpp"
#include <ros/ros.h>

namespace MADER_timers
{
class ROSTimer
{
public:
  ROSTimer(bool run = false)
  {
    if (run)
      Reset();
  }
  void Reset()
  {
    _start = ros::Time::now().toSec();
  }
  double ElapsedMs() const
  {
    return 1000 * (ros::Time::now().toSec() - _start);
  }
  template <typename T, typename Traits>
  friend std::basic_ostream<T, Traits>& operator<<(std::basic_ostream<T, Traits>& out, const ROSTimer& timer)
  {
    return out << timer.ElapsedMs();
  }

private:
  double _start;
};

class ROSWallTimer
{
public:
  ROSWallTimer(bool run = false)
  {
    if (run)
      Reset();
  }
  void Reset()
  {
    _start = ros::WallTime::now().toSec();
  }
  double ElapsedMs() const
  {
    return 1000 * (ros::WallTime::now().toSec() - _start);
  }
  template <typename T, typename Traits>
  friend std::basic_ostream<T, Traits>& operator<<(std::basic_ostream<T, Traits>& out, const ROSWallTimer& timer)
  {
    return out << timer.ElapsedMs();
  }

private:
  double _start;
};
}  // namespace MADER_timers

#endif  // TIMER_HPP_