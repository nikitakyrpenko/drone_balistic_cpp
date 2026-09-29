#pragma once

#include "service/SimulationBundle.hpp"

enum class LoaderType { JSON, UART };
enum class SolverType { ANALYTICAL, TABLE };

class IMissionFactory {
public:
  virtual SimulationBundle create(LoaderType loader_type,
                                  SolverType solver_type,
                                  const char* config_source,
                                  const char* ammo_source,
                                  const char* target_source,
                                  const char* table_source = nullptr) const = 0;

  virtual ~IMissionFactory() = default;
};
