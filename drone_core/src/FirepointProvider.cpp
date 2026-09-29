#include "service/FirepointProvider.hpp"
#include "models/Task.hpp"
#include "Calc.hpp"

#include <optional>

const BallisticSolution FirepointProvider::solve(const DroneTelemetry& tel,
                                                 const DroneSpec& spec,
                                                 const FallResult& fall,
                                                 const Coord& target) const
{
  const float ammo_distance_to_fall = fall.distance;
  const Coord pos = tel.get_position();

  const float distance = Calc::lenght(target, pos);

  const float remaining_acceleration_distance =
    (spec.get_attack_speed() * spec.get_attack_speed() - tel.get_current_speed() * tel.get_current_speed()) /
    (2.0F * spec.get_acceleration());

  if (ammo_distance_to_fall + remaining_acceleration_distance < distance) {
    const float ratio = (distance - ammo_distance_to_fall) / distance;

    return {(pos + ((target - pos) * ratio)), std::nullopt};
  }

  if (distance == 0.0F) {
    return {pos, std::nullopt};
  }

  const float intermediate_ratio = (ammo_distance_to_fall + spec.get_acceleration_path()) / distance;

  if (intermediate_ratio >= 1.0F) {
    const float direct_ratio = (distance - ammo_distance_to_fall) / distance;
    return {pos + (target - pos) * direct_ratio, std::nullopt};
  }

  const Coord intermidiate = target - (target - pos) * intermediate_ratio;

  float distance_from_intermidiate_to_target = Calc::lenght(intermidiate, target);
  float ratio = (distance_from_intermidiate_to_target - ammo_distance_to_fall) / distance_from_intermidiate_to_target;

  Coord fire{intermidiate + (target - intermidiate) * ratio};

  return {fire, intermidiate};
}

FirepointProvider::~FirepointProvider() {}