#pragma once

#include "service/interfaces/IMissionFactory.hpp"

class MissionFactory : public IMissionFactory {
public:
  SimulationBundle create(LoaderType loader_type,
                          SolverType solver_type,
                          const char* config_source,
                          const char* ammo_source,
                          const char* target_source,
                          const char* table_source = nullptr) const override;
};
