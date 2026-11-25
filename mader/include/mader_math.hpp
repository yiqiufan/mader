/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#ifndef MADER_MATH_HPP
#define MADER_MATH_HPP

#include <Eigen/Core>

namespace mu
{
// sign function
template <typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

inline void saturate(double& var, const double min, const double max);

inline void saturate(Eigen::Vector3d& tmp, const Eigen::Vector3d& min, const Eigen::Vector3d& max)
{
  mu::saturate(tmp(0), min(0), max(0));
  mu::saturate(tmp(1), min(1), max(1));
  mu::saturate(tmp(2), min(2), max(2));
}

inline void saturate(int& var, const int min, const int max)
{
  if (var < min)
  {
    var = min;
  }
  else if (var > max)
  {
    var = max;
  }
}

// TODO: Make a template here
inline void saturate(double& var, const double min, const double max)
{
  if (var < min)
  {
    var = min;
  }
  else if (var > max)
  {
    var = max;
  }
}

}  // namespace mu

#endif
