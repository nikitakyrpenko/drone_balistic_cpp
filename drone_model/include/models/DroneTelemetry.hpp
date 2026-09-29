#pragma once

#include "models/Coord.hpp"

class DroneTelemetry {
  Coord pos_;
  float current_speed_;
  float dir_;
  float elapsed_;
  float altitude_;

public:
  DroneTelemetry(const Coord& pos, float initial_dir, float altitude)
    : pos_(pos)
    , current_speed_(0.0F)
    , dir_(initial_dir)
    , elapsed_(0.0F)
    , altitude_(altitude)
  {
  }

  inline Coord get_position() const { return pos_; }
  inline float get_current_speed() const { return current_speed_; }
  inline float get_current_direction() const { return dir_; }
  inline float elapsed() const { return elapsed_; }
  inline float get_altitude() const { return altitude_; }

  inline void set_position(const Coord& position) { pos_ = position; }
  inline void set_current_speed(float speed) { current_speed_ = speed; }
  inline void set_current_direction(float direction) { dir_ = direction; }
  inline void set_elapsed(float elapsed) { elapsed_ = elapsed; }
  inline void set_altitude(float altitude) { altitude_ = altitude; }
};