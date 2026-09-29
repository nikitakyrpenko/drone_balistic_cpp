#pragma once

#include "service/interfaces/IBallisticSolutionEvaluator.hpp"
#include "service/interfaces/IBallisticSolver.hpp"
#include "service/interfaces/IFirepointProvider.hpp"

#include <memory>

class IMissionProccessor {
public:
  virtual void step() = 0;
  virtual bool has_finished() = 0;

  virtual void set_ballistic_solver(std::unique_ptr<IBallisticSolver> ballistic_solver) = 0;
  virtual void set_firepoint_provider(std::unique_ptr<IFirepointProvider> firepoint_provider) = 0;
  virtual void set_time_evaluator(std::unique_ptr<IBallisticSolutionEvaluator> time_evaluator) = 0;

  virtual ~IMissionProccessor() = default;
};