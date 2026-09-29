#include "service/MissionFactory.hpp"

#include "UartLink.hpp"
#include "service/AnalyticalBallisticSolver.hpp"
#include "service/BallisticSolutionEvaluator.hpp"
#include "service/BallisticTableLoader.hpp"
#include "service/ConfigLoader.hpp"
#include "service/DronePhysics.hpp"
#include "service/FirepointProvider.hpp"
#include "service/MissionProccesor.hpp"
#include "service/TableBallisticSolver.hpp"
#include "service/TargetProvider.hpp"
#include "service/UartConfigLoader.hpp"
#include "service/UartDronePhysics.hpp"
#include "service/UartTargetProvider.hpp"
#include "service/interfaces/IMissionFactory.hpp"

#include <latch>
#include <memory>
#include <stdexcept>

namespace {

std::unique_ptr<IBallisticSolver> make_fall_solver(SolverType type, const char* table_source)
{
  switch (type) {
    case SolverType::ANALYTICAL:
      return std::make_unique<AnalyticalBallisticSolver>();
    case SolverType::TABLE:
      return std::make_unique<TableBallisticSolver>(load_ballistic_table(table_source));
  }
  throw std::runtime_error("MissionFactory: unsupported solver type");
}

SimulationBundle create_json(
  const char* config_source, const char* ammo_source, const char* target_source, SolverType solver_type, const char* table_source)
{
  auto loader = std::make_shared<ConfigLoader>(config_source, ammo_source, target_source);

  auto target = std::make_shared<TargetProvider>(loader);
  auto physics = std::make_shared<DronePhysics>(*loader);

  auto mission = std::make_unique<MissionProccessor>(target,
                                                     physics,
                                                     loader,
                                                     make_fall_solver(solver_type, table_source),
                                                     std::make_unique<FirepointProvider>(),
                                                     std::make_unique<BallisticSolutionEvaluator>());

  return SimulationBundle{std::move(target), std::move(physics), std::move(mission)};
}

SimulationBundle create_uart(const char* serial_device, SolverType solver_type, const char* table_source)
{
  // Every ThreadWorker::start(latch) spawns a thread that calls latch.arrive_and_wait()
  // whenever the OS gets around to scheduling it. Since start() itself doesn't block,
  // this function must also arrive_and_wait() on each latch before moving on, or the
  // latch (stack-local to this function) can be destroyed before the spawned thread
  // ever touches it — a dangling reference.
  auto link = std::make_shared<UartLink>(serial_device);
  std::latch link_latch{2};
  link->start(link_latch);
  link_latch.arrive_and_wait();

  auto loader = std::make_shared<UartConfigLoader>(link);
  std::latch config_latch{2};
  loader->start(config_latch);
  config_latch.arrive_and_wait();
  loader->wait_ready();

  auto target = std::make_shared<UartTargetProvider>(link, loader);
  auto physics = std::make_shared<UartDronePhysics>(link, loader);

  std::latch worker_latch{3};
  target->start(worker_latch);
  physics->start(worker_latch);
  worker_latch.arrive_and_wait();

  auto mission = std::make_unique<MissionProccessor>(target,
                                                     physics,
                                                     loader,
                                                     make_fall_solver(solver_type, table_source),
                                                     std::make_unique<FirepointProvider>(),
                                                     std::make_unique<BallisticSolutionEvaluator>());

  return SimulationBundle{std::move(target), std::move(physics), std::move(mission)};
}

}  // namespace

SimulationBundle MissionFactory::create(LoaderType loader_type,
                                        SolverType solver_type,
                                        const char* config_source,
                                        const char* ammo_source,
                                        const char* target_source,
                                        const char* table_source) const
{
  switch (loader_type) {
    case LoaderType::JSON:
      return create_json(config_source, ammo_source, target_source, solver_type, table_source);
    case LoaderType::UART:
      return create_uart(config_source, solver_type, table_source);
  }
  throw std::runtime_error("MissionFactory: unsupported loader type");
}
