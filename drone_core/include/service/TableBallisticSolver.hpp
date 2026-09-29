#pragma once

#include "dto/BallisticTableDTO.hpp"
#include "service/interfaces/IBallisticSolver.hpp"

#include <utility>

class TableBallisticSolver : public IBallisticSolver {
  BallisticTableDTO table_;

public:
  explicit TableBallisticSolver(BallisticTableDTO table)
    : table_(std::move(table))
  {
  }

  FallResult fall(const Ammo& ammo, float altitude, float speed) const override;
};
