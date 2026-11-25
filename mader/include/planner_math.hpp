/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <algorithm>
#include <cmath>

#include "mader_math.hpp"
#include "mader_types.hpp"

namespace mu
{
inline double getMinTimeDoubleIntegrator1D(const double& p0, const double& v0, const double& pf, const double& vf,
                                    const double& v_max, const double& a_max);
inline double getMinTimeDoubleIntegrator3D(const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, const Eigen::Vector3d& pf,
                                    const Eigen::Vector3d& vf, const Eigen::Vector3d& v_max,
                                    const Eigen::Vector3d& a_max);
inline mt::PieceWisePol composePieceWisePol(const double t, const double dc, mt::PieceWisePol& p1, mt::PieceWisePol& p2);
inline bool boxIntersectsSphere(Eigen::Vector3d center, double r, Eigen::Vector3d c1, Eigen::Vector3d c2);
inline void angle_wrap(double& diff);

inline double getMinTimeDoubleIntegrator1D(const double& p0, const double& v0, const double& pf, const double& vf,
                                        const double& v_max, const double& a_max)
{
  // See also minimum_time.m
  // %The notation of this function is based on the paper "Constrained time-optimal
  // %control of double integrator system and its application in MPC"
  // % https://iopscience.iop.org/article/10.1088/1742-6596/783/1/012024

  double x1 = v0;
  double x2 = p0;
  double x1r = vf;
  double x2r = pf;

  double k1 = a_max;  // Note that the paper uses u\in[-1, 1].But setting k1 to a_max has the same effect k2 = 1.0;
  double k2 = 1.0;

  double x1_bar = v_max;

  double B = (k2 / (2 * k1)) * sgn(-x1 + x1r) * (pow(x1, 2) - pow(x1r, 2)) + x2r;
  double C = (k2 / (2 * k1)) * (pow(x1, 2) + pow(x1r, 2)) - (k2 / k1) * pow(x1_bar, 2) + x2r;
  double D = (-k2 / (2 * k1)) * (pow(x1, 2) + pow(x1r, 2)) + (k2 / k1) * pow(x1_bar, 2) + x2r;

  double time;

  if ((x2 <= B) && (x2 >= C))
  {
    time = (-k2 * (x1 + x1r) +
            2 * sqrt(pow(k2, 2) * pow(x1, 2) - k1 * k2 * ((k2 / (2 * k1)) * (pow(x1, 2) - pow(x1r, 2)) + x2 - x2r))) /
           (k1 * k2);
  }

  else if ((x2 <= B) && (x2 < C))
  {
    time = (x1_bar - x1 - x1r) / k1 + (pow(x1, 2) + pow(x1r, 2)) / (2 * k1 * x1_bar) + (x2r - x2) / (k2 * x1_bar);
  }

  else if ((x2 > B) && (x2 <= D))
  {
    time = (k2 * (x1 + x1r) +
            2 * sqrt(pow(k2, 2) * pow(x1, 2) + k1 * k2 * ((k2 / (2 * k1)) * (-pow(x1, 2) + pow(x1r, 2)) + x2 - x2r))) /
           (k1 * k2);
  }

  else
  {  // (x2 > B) && (x2 > D)

    time = (x1_bar + x1 + x1r) / k1 + (pow(x1, 2) + pow(x1r, 2)) / (2 * k1 * x1_bar) + (-x2r + x2) / (k2 * x1_bar);
  }

  return time;
}

inline double getMinTimeDoubleIntegrator3D(const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, const Eigen::Vector3d& pf,
                                        const Eigen::Vector3d& vf, const Eigen::Vector3d& v_max,
                                        const Eigen::Vector3d& a_max)
{
  double min_x = getMinTimeDoubleIntegrator1D(p0.x(), v0.x(), pf.x(), vf.x(), v_max.x(), a_max.x());
  double min_y = getMinTimeDoubleIntegrator1D(p0.y(), v0.y(), pf.y(), vf.y(), v_max.y(), a_max.y());
  double min_z = getMinTimeDoubleIntegrator1D(p0.z(), v0.z(), pf.z(), vf.z(), v_max.z(), a_max.z());

  double min_time = std::max({ min_x, min_y, min_z });  // Note that it's the maximum of all the axes

  return min_time;
}

// returns a mt::PieceWisePol, taking the polynomials of p1 and p2 that should satisfy p1>t, p2>t
inline mt::PieceWisePol composePieceWisePol(const double t, const double dc, mt::PieceWisePol& p1, mt::PieceWisePol& p2)
{
  // if t is in between p1 and p2, force p2[0] to be t
  if (t > p1.times.back() && t < p2.times.front())  // && fabs(t - p2.times.front()) <= dc) TODO Sometimes fabs(t -
                                                    // p2.times.front()) is 0.18>c
  {
    p2.times.front() = t;
  }

  if (p1.times.back() < p2.times.front())  // && fabs(p1.times.back() - p2.times.front()) <= dc) TODO
  {
    p2.times.front() = p1.times.back();
  }

  if (t < p1.times.front())  // TODO
  {
    p1.times.front() = t;
  }

  if (fabs(t - p2.times.front()) < 1e-5)
  {
    return p2;
  }

  if (p1.times.back() < p2.times.front() || t > p2.times.back() || t < p1.times.front())
  {
    // TODO: does this happen?

    // std::cout << "Error composing the piecewisePol" << std::endl;
    // std::cout << std::setprecision(30) << "t= " << t << std::endl;
    // std::cout << std::setprecision(30) << "p1.times.front()= " << p1.times.front() << std::endl;
    // std::cout << std::setprecision(30) << "p1.times.back()= " << p1.times.back() << std::endl;
    // std::cout << std::setprecision(30) << "p2.times.front() = " << p2.times.front() << std::endl;
    // std::cout << std::setprecision(30) << "p2.times.back() = " << p2.times.back() << std::endl;
    mt::PieceWisePol dummy;
    return dummy;
  }

  std::vector<int> indexes1, indexes2;

  for (int i = 0; i < p1.times.size(); i++)
  {
    if (p1.times[i] > t && p1.times[i] < p2.times[0])
    {
      indexes1.push_back(i);
    }
  }

  for (int i = 0; i < p2.times.size(); i++)
  {
    if (p2.times[i] > t)
    {
      indexes2.push_back(i);
    }
  }

  mt::PieceWisePol p;
  p.times.push_back(t);

  for (auto index_1_i : indexes1)
  {
    p.times.push_back(p1.times[index_1_i]);
    p.coeff_x.push_back(p1.coeff_x[index_1_i - 1]);
    p.coeff_y.push_back(p1.coeff_y[index_1_i - 1]);
    p.coeff_z.push_back(p1.coeff_z[index_1_i - 1]);
  }

  for (auto index_2_i : indexes2)
  {
    if (index_2_i == 0)
    {
      p.coeff_x.push_back(p1.coeff_x.back());
      p.coeff_y.push_back(p1.coeff_y.back());
      p.coeff_z.push_back(p1.coeff_z.back());
      p.times.push_back(p2.times[index_2_i]);
      continue;
    }
    p.times.push_back(p2.times[index_2_i]);
    p.coeff_x.push_back(p2.coeff_x[index_2_i - 1]);
    p.coeff_y.push_back(p2.coeff_y[index_2_i - 1]);
    p.coeff_z.push_back(p2.coeff_z[index_2_i - 1]);
  }

  return p;
}

inline bool boxIntersectsSphere(Eigen::Vector3d center, double r, Eigen::Vector3d c1, Eigen::Vector3d c2)
{
  // https://developer.mozilla.org/en-US/docs/Games/Techniques/3D_collision_detection
  //(Section Sphere vs AABB)

  Eigen::Vector3d closest_point;  // closest point from the center of the sphere to the box

  closest_point(0) = std::max(c1.x(), std::min(center.x(), c2.x()));
  closest_point(1) = std::max(c1.y(), std::min(center.y(), c2.y()));
  closest_point(2) = std::max(c1.z(), std::min(center.z(), c2.z()));

  // this is the same as isPointInsideSphere
  double distance_to_closest_point = (center - closest_point).norm();

  return (distance_to_closest_point < r);  // true if the box intersects the sphere
}

inline void angle_wrap(double& diff)
{
  diff = fmod(diff + M_PI, 2 * M_PI);
  if (diff < 0)
    diff += 2 * M_PI;
  diff -= M_PI;
}
}  // namespace mu
