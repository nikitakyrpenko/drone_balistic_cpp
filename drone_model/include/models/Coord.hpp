#pragma once

struct Coord {
  float x_, y_;

  Coord operator+(const Coord& other) const { return {.x_ = x_ + other.x_, .y_ = y_ + other.y_}; }
  Coord operator-(const Coord& other) const { return {.x_ = x_ - other.x_, .y_ = y_ - other.y_}; }
  Coord operator*(float multi) const { return {.x_ = x_ * multi, .y_ = y_ * multi}; }
  Coord operator/(float div) const { return {.x_ = x_ / div, .y_ = y_ / div}; }
  bool operator==(const Coord& other) const { return (x_ == other.x_ && y_ == other.y_); }
};
