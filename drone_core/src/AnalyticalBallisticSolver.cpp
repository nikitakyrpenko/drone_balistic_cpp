#include "service/AnalyticalBallisticSolver.hpp"

#include <cmath>

namespace {

constexpr float Grav = 9.81F;

float time_to_fall(float mass, float drag, float lift, float altitude, float on_speed)
{
  float a = (drag * Grav * mass) - (2.0f * drag * drag * lift * on_speed);
  float b = (-3.0f * Grav * mass * mass) + (3.0f * drag * lift * mass * on_speed);
  float c = 6.0f * mass * mass * altitude;

  float p = (-b * b) / (3.0f * a * a);
  float q = (2.0f * b * b * b) / (27.0f * a * a * a) + (c / a);

  float acos_arg = (3.0f * q) / (2.0f * p) * std::sqrt(-3.0f / p);
  if (acos_arg < -1.0f || acos_arg > 1.0f) {
    return 0.0f;
  }

  return 2.0f * std::sqrt(-p / 3.0f) * std::cos((std::acos(acos_arg) + 4.0f * std::acos(-1.0f)) / 3.0f) - b / (3.0f * a);
}

float distance_to_fall(float mass, float drag, float lift, float ammo_time_to_fall, float on_speed)
{
  if (ammo_time_to_fall == 0.0f) {
    return 0.0f;
  }

  return (std::pow(ammo_time_to_fall, 3) *
          ((6.0f * drag * Grav * lift * mass) - (6.0f * std::pow(drag, 2) * (std::pow(lift, 2) - 1) * on_speed))) /
           (36.0f * std::pow(mass, 2)) +
         (std::pow(ammo_time_to_fall, 5) * ((3.0f * std::pow(drag, 3) * Grav * std::pow(lift, 3) * mass) -
                                            (3.0f * std::pow(drag, 4) * std::pow(lift, 2) * (std::pow(lift, 2) + 1) * on_speed))) /
           (36.0f * (std::pow(lift, 2) + 1) * std::pow(mass, 4)) +
         (std::pow(ammo_time_to_fall, 4) *
          ((3.0f * std::pow(drag, 3) * (std::pow(lift, 2) + 1) * std::pow(lift, 2) * on_speed) +
           (6.0f * std::pow(drag, 3) * (std::pow(lift, 2) + 1) * std::pow(lift, 4) * on_speed) -
           (6.0f * std::pow(drag, 2) * Grav * (std::pow(lift, 4) + std::pow(lift, 2) + 1) * lift * mass))) /
           (36.0f * std::pow(std::pow(lift, 2) + 1, 2) * std::pow(mass, 3)) -
         (drag * std::pow(ammo_time_to_fall, 2) * on_speed) / (2.0f * mass) + (ammo_time_to_fall * on_speed);
}

}  // namespace

FallResult AnalyticalBallisticSolver::fall(const Ammo& ammo, float altitude, float speed) const
{
  const float mass = ammo.get_mass();
  const float drag = ammo.get_drag();
  const float lift = ammo.get_lift();

  const float time = time_to_fall(mass, drag, lift, altitude, speed);
  const float distance = distance_to_fall(mass, drag, lift, time, speed);

  return {time, distance};
}
