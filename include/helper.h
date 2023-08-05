// Copyright (c) 2022 Yongkie Wiyogo
//
// Licensed under the MIT license: https://opensource.org/licenses/MIT
// Permission is granted to use, copy, modify, and redistribute the work.
// Full license information available in the project LICENSE file.
//
#ifndef __HELPER_H__
#define __HELPER_H__

#include <vector>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

template <typename T>
class Point2D
{
public:
  T x = 0;
  T y = 0;
  Point2D(){};
  Point2D(T _x, T _y)
  {
    x = _x;
    y = _y;
  }
  Point2D &operator=(const Point2D &other)
  {
    x = other.x;
    y = other.y;
    return *this;
  }
};

template <typename T>
class Pose
{
public:
  T x = 0;
  T y = 0;
  double orient = 0.;
  Pose(){};
  Pose(T x_in, T y_in, double orient_in_rad) : x(x_in), y(y_in), orient(orient_in_rad){};
  Pose &operator=(const Pose &other)
  {
    x = other.x;
    y = other.y;
    orient = other.orient;
    return *this;
  }
};

template <typename T>
static T max(const std::vector<T> &vec)
{
  // Identify the max element in an array
  T max = 0;
  for (int i = 0; i < vec.size(); i++)
  {
    if (vec[i] > max)
      max = vec[i];
  }
  return max;
}

/**
 * @brief Probability of x for 1D Gaussian with mu or mean and sigma or variance
 */
template <typename T>
T calc_gaussian(T mean, T variance, T x)
{
  return exp(-(pow((mean - x), 2)) / (pow(variance, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(variance, 2)));
}
#endif // __HELPER_H__
