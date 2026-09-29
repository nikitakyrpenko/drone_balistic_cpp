#pragma once

#include "service/MissionProccesor.hpp"
#include "service/interfaces/IDronePhysics.hpp"
#include "service/interfaces/ITargetProvider.hpp"

#include <memory>

struct SimulationBundle {
  std::shared_ptr<ITargetProvider> target;
  std::shared_ptr<IDronePhysics> physics;
  std::unique_ptr<MissionProccessor> mission;
};
