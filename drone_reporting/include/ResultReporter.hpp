#pragma once

#include "models/SimulationStep.hpp"

#include "json.hpp"

#include <string>
#include <vector>

class ResultReporter {
public:
  virtual ~ResultReporter() = default;

  virtual bool save(const std::vector<SimulationStep>& steps, const std::string& student_id, const std::string& test_id) = 0;
  virtual bool check(const std::string& student_id, const std::string& test_id) const = 0;

protected:
  nlohmann::ordered_json to_json(const std::vector<SimulationStep>& steps, const std::string& student_id, const std::string& test_id) const;
};
