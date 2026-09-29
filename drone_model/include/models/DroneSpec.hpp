#pragma once

class DroneSpec {
  float attack_speed_;
  float acceleration_path_;
  float angular_speed_;
  float turn_threshold_;
  float acceleration_;

public:
  DroneSpec(float attack_speed, float acceleration_path, float angular_speed, float turn_threshold)
    : attack_speed_(attack_speed)
    , acceleration_path_(acceleration_path)
    , angular_speed_(angular_speed)
    , turn_threshold_(turn_threshold)
    , acceleration_((attack_speed_ * attack_speed_) / (2.0F * acceleration_path_))
  {
  }

  inline float get_attack_speed() const { return attack_speed_; }
  inline float get_acceleration_path() const { return acceleration_path_; }
  inline float get_angular_speed() const { return angular_speed_; }
  inline float get_turn_threshold() const { return turn_threshold_; }
  inline float get_acceleration() const { return acceleration_; }
};