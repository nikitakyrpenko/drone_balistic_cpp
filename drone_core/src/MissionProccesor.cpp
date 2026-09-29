#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>
#include <ranges>
#include <utility>
#include <vector>

#include "Calc.hpp"
#include "models/Coord.hpp"
#include "models/DroneCommand.hpp"
#include "models/DroneTelemetry.hpp"
#include "models/SimulationStep.hpp"
#include "models/Target.hpp"
#include "models/Task.hpp"
#include "service/MissionProccesor.hpp"

auto MissionProccessor::score(int t_id,
                              const Coord& t_position,
                              const DroneTelemetry& telemetry,
                              const DroneSpec& spec,
                              const FallResult& ammo_parameters) const -> Task
{
  const BallisticSolution firepoint = firepoint_provider_->solve(telemetry, spec, ammo_parameters, t_position);
  return time_evaluator_->calculate_time_taken(telemetry, spec, t_id, firepoint);
}

auto MissionProccessor::is_task_possible(const Task& t) const -> bool
{
  return std::isfinite(t.time_taken) && t.time_taken != std::numeric_limits<float>::max();
}

const Task& MissionProccessor::select_task(const DroneTelemetry& telemetry, const std::vector<Task>& tasks) const
{
  const Task& optimal =
    *std::min_element(tasks.begin(), tasks.end(), [](const Task& a, const Task& b) { return a.time_taken < b.time_taken; });

  const auto current = std::find_if(tasks.begin(), tasks.end(), [this](const Task& t) { return t.id_ == current_task.id_; });

  if (current == tasks.end()) {
    return optimal;
  }

  // check is lock on target needed, if prerequisits are good, keep current
  const bool lock_in =
    Calc::lenght(telemetry.get_position(), current->solution_.fire_) <= LOCK_FACTOR * config_loader_->get_config().hit_radius_;

  // check is margin to switch is more then 50% to reduce states jittering
  const bool beat_margin = optimal.time_taken < current->time_taken * SWITCH_FACTOR;

  return (lock_in || !beat_margin) ? *current : optimal;
}

void MissionProccessor::log_simulation(const DroneTelemetry& tel,
                                       const Task& tsk,
                                       const IState* s,
                                       const FallResult& ap,
                                       const Target& tar,
                                       const Coord& active,
                                       const std::vector<Target>& all_targets)
{
  const Coord heading{std::cos(tel.get_current_direction()), std::sin(tel.get_current_direction())};

  const SimulationStep step{tsk.id_,
                            tel.get_current_direction(),
                            s->name(),
                            tel.get_position(),
                            active,
                            tel.get_position() + heading * ap.distance,
                            tar.approximate(current_task.time_taken + ap.time),
                            tar.pos_,
                            tel.elapsed(),
                            tel.get_current_speed(),
                            ammo_.get_mass(),
                            ammo_.get_drag(),
                            ammo_.get_lift(),
                            ap.time,
                            ap.distance,
                            all_targets};

  // std::cout << "[step " << steps_.size() << "] target=" << step.target_id_ << " state=" << step.state_ << " pos=(" << step.position_.x_
  //          << ", " << step.position_.y_ << ")" << " drop=(" << step.drop_point_.x_ << ", " << step.drop_point_.y_ << ")\n";

  steps_.push_back(step);
}

bool MissionProccessor::has_point_visited(const Coord& point, const Coord& previous, const Coord& current, float tolerance) const
{
  return Calc::point_to_segment_distance(point, previous, current) <= tolerance;
}

void MissionProccessor::step()
{
  const DroneTelemetry telemetry = drone_physics_->get_telemetry();
  const DroneSpec spec = drone_physics_->get_spec();

  const auto targets = target_provider_->get_targets();
  const auto ammo_parameters = ballistic_solver_->fall(ammo_, telemetry.get_altitude(), spec.get_attack_speed());

  // clang-format off

  auto view =
    targets |
    std::views::transform([&](const Target& t) { return std::pair{t, score(t.target_id_, t.pos_, telemetry, spec, ammo_parameters)}; }) |
    std::views::filter([&](const auto& p) { return is_task_possible(p.second); }) |
    std::views::transform([&](const auto& p) {
      const auto& [target, task] = p;
      return score(target.target_id_, target.approximate(task.time_taken + ammo_parameters.time), telemetry, spec, ammo_parameters);
    }) |
    std::views::filter([&](const Task& task) { return is_task_possible(task); });

  // clang-format on

  std::vector<Task> tasks;
  std::ranges::copy(view, std::back_inserter(tasks));

  // update current_task with optimal value
  if (!tasks.empty()) {
    const Task& optimal = select_task(telemetry, tasks);
    if (optimal.id_ == current_task.id_) {
      current_task.solution_ = optimal.solution_;
      current_task.time_taken = optimal.time_taken;
    }
    else {
      // if new target has intermididate flush the navigation flag
      optimal.solution_.intermididate_.has_value() ? has_visited_intermidiate = false : has_visited_intermidiate = true;
      current_task = optimal;
    }
  }

  // nothing feasible has ever been committed yet -> hold (stay stopped)
  if (current_task.id_ == -1) {
    return;
  }

  if (should_visit_intermididate()) {
    const Coord current = telemetry.get_position();
    const Coord previous = steps_.empty() ? current : steps_.back().position_;

    if (has_point_visited(*current_task.solution_.intermididate_, previous, current, config_loader_->get_config().hit_radius_)) {
      has_visited_intermidiate = true;
    }
  }

  const Coord& active = get_active_navigation();
  const StateDecision next = drone_physics_->get_state()->decide(spec, telemetry, active, should_visit_intermididate());

  drone_physics_->submit_command(DroneCommand(next.next_state_->mode(), next.dir));

  log_simulation(
    telemetry, current_task, next.next_state_, ammo_parameters, target_provider_->get_target(current_task.id_), active, targets);
}

bool MissionProccessor::has_finished()
{
  if (current_task.id_ == -1 || steps_.size() < 2) {
    return false;
  }

  const Coord& now = steps_.back().position_;
  const Coord& before = steps_.at(steps_.size() - 2).position_;

  return !should_visit_intermididate() &&
         has_point_visited(
           current_task.solution_.fire_, before, now, drone_physics_->get_spec().get_attack_speed() * (sim_timestep_ + COMMAND_LATENCY));
}

void MissionProccessor::start(std::latch& latch, int max_iterations)
{
  running_ = true;

  worker_ = std::thread([this, &latch, max_iterations]() {
    latch.arrive_and_wait();

    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<clock::duration>(std::chrono::duration<float>(sim_timestep_ / timescale_));

    auto next = clock::now();
    const auto deadline = next + MISSION_TIMEOUT;
    for (int iter = 0; running_ && iter < max_iterations && clock::now() < deadline && !has_finished(); ++iter) {
      step();
      next += period;
      std::this_thread::sleep_until(next);
    }
  });
}

void MissionProccessor::interrupt()
{
  running_ = false;
}

void MissionProccessor::join()
{
  if (worker_.joinable()) {
    worker_.join();
  }
}

MissionProccessor::~MissionProccessor()
{
  interrupt();

  if (worker_.joinable()) {
    worker_.join();
  }
}
