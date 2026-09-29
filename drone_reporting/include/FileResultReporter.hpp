#pragma once

#include "ResultReporter.hpp"

#include <string>

class FileResultReporter : public ResultReporter {
public:
  explicit FileResultReporter(std::string path);

  bool save(const std::vector<SimulationStep>& steps, const std::string& student_id, const std::string& test_id) override;
  bool check(const std::string& student_id, const std::string& test_id) const override;

private:
  nlohmann::ordered_json to_json(const std::vector<SimulationStep>& steps, const std::string& student_id, const std::string& test_id) const;

  std::string path_;
};
