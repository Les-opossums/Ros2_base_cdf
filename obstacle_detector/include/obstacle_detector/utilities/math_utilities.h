/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#pragma once

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "tf2/convert.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "obstacle_detector/utilities/point.h"

namespace obstacle_detector
{

inline double signum(double x) { return (x < 0.0) ? -1.0 : 1.0; }
inline double abs(double x) { return (x < 0.0) ? -x : x; }
inline double max(double x, double y) { return (x > y) ? x : y; }

inline double length(const geometry_msgs::msg::Point& point) {
  return sqrt(point.x * point.x + point.y * point.y);
}

inline double squaredLength(const geometry_msgs::msg::Point& point) {
  return point.x * point.x + point.y * point.y;
}

inline double length(const geometry_msgs::msg::Vector3& vec) {
  return sqrt(vec.x * vec.x + vec.y * vec.y);
}

inline double squaredLength(const geometry_msgs::msg::Vector3& vec) {
  return vec.x * vec.x + vec.y * vec.y;
}

inline geometry_msgs::msg::Point transformPoint(const geometry_msgs::msg::Point& point, double x, double y, double theta) {
  geometry_msgs::msg::Point p;

  p.x = point.x * cos(theta) - point.y * sin(theta) + x;
  p.y = point.x * sin(theta) + point.y * cos(theta) + y;
  p.z = point.z;

  return p;
}

inline geometry_msgs::msg::Point32 transformPoint(const geometry_msgs::msg::Point32& point, double x, double y, double theta) {
  geometry_msgs::msg::Point32 p;

  p.x = point.x * cos(theta) - point.y * sin(theta) + x;
  p.y = point.x * sin(theta) + point.y * cos(theta) + y;
  p.z = point.z;

  return p;
}

inline Point transformPoint(const Point point, double x, double y, double theta) {
  Point p;

  p.x = point.x * cos(theta) - point.y * sin(theta) + x;
  p.y = point.x * sin(theta) + point.y * cos(theta) + y;
  p.z = point.z;

  return p;
}

inline Point transformPoint(const Point& point, const geometry_msgs::msg::TransformStamped& transform) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = point.x;
  pose.position.y = point.y;
  pose.position.z = point.z;
  tf2::doTransform(pose, pose, transform);
  return {pose.position.x, pose.position.y, 0., pose.position.z};
}

inline bool checkPointInLimits(const geometry_msgs::msg::Point32& p, double x_min, double x_max, double y_min, double y_max) {
  if ((p.x > x_max) || (p.x < x_min) || (p.y > y_max) || (p.y < y_min))
    return false;
  else
    return true;
}

} // namespace obstacle_detector
