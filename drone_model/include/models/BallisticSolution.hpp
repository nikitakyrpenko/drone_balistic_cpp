#include <optional>
#include "models/Coord.hpp"

struct BallisticSolution {
  Coord fire_;
  std::optional<Coord> intermididate_;
};