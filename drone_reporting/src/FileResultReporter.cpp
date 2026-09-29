#include "FileResultReporter.hpp"

#include <filesystem>
#include <fstream>

FileResultReporter::FileResultReporter(std::string path)
  : path_(std::move(path))
{
}

bool FileResultReporter::save(const std::vector<SimulationStep>& steps, const std::string& student_id, const std::string& test_id)
{
  const nlohmann::ordered_json report = to_json(steps, student_id, test_id);

  std::ofstream ofs(path_);
  ofs << report.dump(2);
  return static_cast<bool>(ofs);
}

bool FileResultReporter::check(const std::string& /*student_id*/, const std::string& /*test_id*/) const
{
  return std::filesystem::exists(path_);
}

nlohmann::ordered_json FileResultReporter::to_json(const std::vector<SimulationStep>& steps,
                                                    const std::string& student_id,
                                                    const std::string& test_id) const
{
  nlohmann::ordered_json j = ResultReporter::to_json(steps, student_id, test_id);
  nlohmann::ordered_json& json_steps = j["simulation"]["steps"];

  for (std::size_t i = 0; i < steps.size(); ++i) {
    const SimulationStep& s = steps[i];

    nlohmann::ordered_json targets = nlohmann::json::array();
    for (const Target& t : s.all_targets_) {
      targets.push_back({
        {"id", t.target_id_},
        {"x", t.pos_.x_},
        {"y", t.pos_.y_},
      });
    }

    nlohmann::ordered_json& step = json_steps[i];
    step["currentSpeed"] = s.current_speed_;
    step["ammo"] = {{"mass", s.ammo_mass_}, {"drag", s.ammo_drag_}, {"lift", s.ammo_lift_}};
    step["fallParameters"] = {{"time", s.fall_time_}, {"distance", s.fall_distance_}};
    step["targets"] = targets;
  }

  return j;
}
