#pragma once

#include "dto/AmmoDTO.hpp"
#include "models/Ammo.hpp"
#include "models/Coord.hpp"
#include "models/DroneCommand.hpp"

#include "models/DroneTelemetry.hpp"
#include "models/SimulationStep.hpp"
#include "models/Target.hpp"
#include "models/Task.hpp"
#include "service/interface/state/IState.hpp"
#include "service/interfaces/IBallisticSolutionEvaluator.hpp"
#include "service/interfaces/IBallisticSolver.hpp"
#include "service/interfaces/IConfigLoader.hpp"
#include "service/interfaces/IFirepointProvider.hpp"
#include "service/interfaces/IMissionProcessor.hpp"
#include "service/interfaces/ITargetProvider.hpp"
#include "service/interfaces/IDronePhysics.hpp"
#include <atomic>
#include <chrono>
#include <latch>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <algorithm>
#include <vector>

constexpr float SWITCH_FACTOR = 0.5F;
constexpr float LOCK_FACTOR = 2.0F;
constexpr float COMMAND_LATENCY = 0.15F;  // measured GPIO/UART round-trip from fire decision to actual release
constexpr std::chrono::seconds MISSION_TIMEOUT{90};

class MissionProccessor : public IMissionProccessor {
  // internal dependencies
  std::unique_ptr<IFirepointProvider> firepoint_provider_;
  std::unique_ptr<IBallisticSolutionEvaluator> time_evaluator_;
  std::unique_ptr<IBallisticSolver> ballistic_solver_;

  // shared dependencies
  std::shared_ptr<const ITargetProvider> target_provider_;
  std::shared_ptr<const IDronePhysics> drone_physics_;
  std::shared_ptr<const IConfigLoader> config_loader_;

  const Ammo ammo_;

  Task current_task{-1, {}};
  bool has_visited_intermidiate{false};

  std::vector<SimulationStep> steps_;

  const float sim_timestep_{0.1F};
  const float timescale_{1.0F};

  std::thread worker_;
  mutable std::mutex mtx_;
  std::atomic<bool> running_{false};

  const Task& select_task(const DroneTelemetry& telemetry, const std::vector<Task>& tasks) const;
  void log_simulation(const DroneTelemetry& tel,
                      const Task& tsk,
                      const IState* s,
                      const FallResult& ap,
                      const Target& tar,
                      const Coord& active,
                      const std::vector<Target>& all_targets);

  bool has_point_visited(const Coord& point, const Coord& previous, const Coord& current, float tolerance) const;

  inline bool should_visit_intermididate() const { return current_task.solution_.intermididate_.has_value() && !has_visited_intermidiate; }

  inline const Coord& get_active_navigation() const
  {
    return should_visit_intermididate() ? *current_task.solution_.intermididate_ : current_task.solution_.fire_;
  }

  auto score(int id, const Coord& aim, const DroneTelemetry& tel, const DroneSpec& spec, const FallResult& fall) const -> Task;
  auto is_task_possible(const Task& t) const -> bool;

public:
  MissionProccessor(std::shared_ptr<const ITargetProvider> tp,
                    std::shared_ptr<const IDronePhysics> dp,
                    std::shared_ptr<const IConfigLoader> cl,
                    std::unique_ptr<IBallisticSolver> bs,
                    std::unique_ptr<IFirepointProvider> fp,
                    std::unique_ptr<IBallisticSolutionEvaluator> te)
    : firepoint_provider_(std::move(fp))
    , time_evaluator_(std::move(te))
    , ballistic_solver_(std::move(bs))
    , target_provider_(std::move(tp))
    , drone_physics_(std::move(dp))
    , config_loader_(std::move(cl))
    , ammo_([&]() {
      const auto& arsenal = this->config_loader_->get_arsenal();
      const auto& name = this->config_loader_->get_config().ammo_;
      auto it = std::find_if(arsenal.ammos_.begin(), arsenal.ammos_.end(), [&name](const AmmoDTO::Ammo& a) { return a.name == name; });

      if (it == arsenal.ammos_.end()) {
        throw std::runtime_error("Ammo not found");
      }
      return Ammo(it->name, it->mass, it->drag, it->lift);
    }())
    , sim_timestep_(config_loader_->get_config().time_step_)
    , timescale_(config_loader_->get_config().timescale)
  {
  }

  void step() override;
  bool has_finished() override;

  void start(std::latch& latch, int max_iterations);
  void interrupt();
  void join();

  inline const std::vector<SimulationStep>& get_steps() const { return steps_; }

  inline void set_ballistic_solver(std::unique_ptr<IBallisticSolver> ballistic_solver) override
  {
    ballistic_solver_ = std::move(ballistic_solver);
  };

  inline void set_firepoint_provider(std::unique_ptr<IFirepointProvider> firepoint_provider) override
  {
    firepoint_provider_ = std::move(firepoint_provider);
  }

  inline void set_time_evaluator(std::unique_ptr<IBallisticSolutionEvaluator> time_evaluator) override
  {
    time_evaluator_ = std::move(time_evaluator);
  }

  ~MissionProccessor() override;
};