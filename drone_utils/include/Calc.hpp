#pragma once
#include <algorithm>
#include <cmath>

#include "models/Coord.hpp"

namespace Calc {

// how much to turn from current direction to dest
inline float calculate_turning_angle(const Coord& from, const Coord& to, float curr_direction)
{
  Coord diff = to - from;

  float dir_to_target = std::atan2(diff.y_, diff.x_);
  float delta = dir_to_target - curr_direction;

  return std::atan2(std::sin(delta), std::cos(delta));
}

inline float angle(const Coord& from, const Coord& to)
{
  return std::atan2(to.y_ - from.y_, to.x_ - from.x_);
}

inline float lenght(const Coord& from, const Coord& to)
{
  float xd = to.x_ - from.x_;
  float yd = to.y_ - from.y_;

  return std::hypot(xd, yd);
}

// given point {intermididate or fire} check does line from start to end crossed point
inline float point_to_segment_distance(const Coord& point, const Coord& start, const Coord& end)
{
  Coord ab = end - start;
  float len_sq = ab.x_ * ab.x_ + ab.y_ * ab.y_;

  float t = 0.0F;
  if (len_sq > 0.0F) {
    t = ((point.x_ - start.x_) * ab.x_ + (point.y_ - start.y_) * ab.y_) / len_sq;
    t = std::clamp(t, 0.0F, 1.0F);
  }

  Coord closest = start + ab * t;
  return lenght(point, closest);
}
}  // namespace Calc
