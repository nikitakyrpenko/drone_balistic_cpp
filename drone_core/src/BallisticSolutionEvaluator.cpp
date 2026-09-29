#include "service/BallisticSolutionEvaluator.hpp"

#include "Calc.hpp"
#include "models/Coord.hpp"
#include "models/Task.hpp"

#include <cmath>
#include <limits>

float BallisticSolutionEvaluator::calculate_time_to_reach(
  const Coord& from, const Coord& to, float current_speed, float attack_speed, float acceleration, bool decelerate_in_dest) const
{
  float distance = Calc::lenght(from, to);

  // how much distance needed to obtain attack_speed
  float remaining_acceleration_distance = (attack_speed * attack_speed - current_speed * current_speed) / (2.0F * acceleration);

  if (!decelerate_in_dest) {
    if (distance < remaining_acceleration_distance) {
      return std::numeric_limits<float>::max();
    }

    float time_to_accelerate = (attack_speed - current_speed) / acceleration;
    float time_to_cruise = (distance - remaining_acceleration_distance) / attack_speed;

    return time_to_accelerate + time_to_cruise;
  }
  float distance_to_accelerate = (attack_speed * attack_speed - current_speed * current_speed) / (2.0F * acceleration);
  float distance_to_decelerate = (attack_speed * attack_speed) / (2.0F * acceleration);

  // trapezoid case: accelerate -> move -> decelerate
  if (distance >= distance_to_accelerate + distance_to_decelerate) {
    float time_to_accelerate = (attack_speed - current_speed) / acceleration;
    float time_to_decelerate = attack_speed / acceleration;
    float time_to_cruise = (distance - distance_to_accelerate - distance_to_decelerate) / attack_speed;

    return time_to_accelerate + time_to_cruise + time_to_decelerate;
  }

  // triangle case: accelerate -> decelerate
  float velocity_peak = std::sqrt(acceleration * distance + (current_speed * current_speed) / 2.0F);
  float time_to_accelerate = (velocity_peak - current_speed) / acceleration;
  float time_to_decelerate = velocity_peak / acceleration;

  return time_to_decelerate + time_to_accelerate;
}

// will return Coordinates of zero velocity and time spent to turn from this coord to desired coord
BallisticSolutionEvaluator::Turn BallisticSolutionEvaluator::calculate_time_to_turn(const Coord& from,
                                                                                    const Coord& to,
                                                                                    float current_direction,
                                                                                    float current_speed,
                                                                                    float turn_threshold,
                                                                                    float angular_speed,
                                                                                    float acceleration) const
{
  float turning_angle = std::abs(Calc::calculate_turning_angle(from, to, current_direction));
  if (turning_angle <= turn_threshold) {
    return {from, 0.0F, Calc::angle(from, to)};
  }

  float decel_distance = (current_speed * current_speed) / (2.0F * acceleration);
  Coord zero_velocity_position = from + Coord{std::cos(current_direction), std::sin(current_direction)} * decel_distance;
  float time_to_decelerate = current_speed / acceleration;

  float turning_angle_at_stop = std::abs(Calc::calculate_turning_angle(zero_velocity_position, to, current_direction));
  float turning_time = turning_angle_at_stop / angular_speed;

  return {zero_velocity_position, time_to_decelerate + turning_time, Calc::angle(zero_velocity_position, to)};
}

BallisticSolutionEvaluator::CompResult BallisticSolutionEvaluator::calculate_leg(
  const DroneSpec& spec, const Coord& from, const Coord& to, float current_direction, float current_speed, bool decelerate_in_dest) const
{
  Turn data = calculate_time_to_turn(
    from, to, current_direction, current_speed, spec.get_turn_threshold(), spec.get_angular_speed(), spec.get_acceleration());

  // snap turn performed -> drone keeps current speed, otherwise it decelerated to a stop
  float starting_speed = (data.time == 0.0F) ? current_speed : 0.0F;
  float travel_time =
    calculate_time_to_reach(data.position, to, starting_speed, spec.get_attack_speed(), spec.get_acceleration(), decelerate_in_dest);

  return {data.time + travel_time, data.direction};
}

const Task BallisticSolutionEvaluator::calculate_time_taken(const DroneTelemetry& tel,
                                                            const DroneSpec& spec,
                                                            int target_id,
                                                            const BallisticSolution& solution) const
{
  if (!solution.intermididate_.has_value()) {
    CompResult leg =
      calculate_leg(spec, tel.get_position(), solution.fire_, tel.get_current_direction(), tel.get_current_speed(), false);

    return {target_id, solution, leg.time};
  }

  // leg 1: current position -> intermidiate, must arrive at zero velocity
  CompResult leg1 =
    calculate_leg(spec, tel.get_position(), *solution.intermididate_, tel.get_current_direction(), tel.get_current_speed(), true);

  // leg 2: intermidiate -> fire, accelerate from zero to attack speed
  CompResult leg2 = calculate_leg(spec, *solution.intermididate_, solution.fire_, leg1.direction, 0.0F, false);

  // cannot reach attack speed at fire, not valid task
  if (leg2.time == std::numeric_limits<float>::max()) {
    return {target_id, solution, std::numeric_limits<float>::max()};
  }

  return {target_id, solution, leg1.time + leg2.time};
}
