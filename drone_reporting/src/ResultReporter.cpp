#include "ResultReporter.hpp"

nlohmann::ordered_json ResultReporter::to_json(const std::vector<SimulationStep>& steps,
                                               const std::string& student_id,
                                               const std::string& test_id) const
{
  nlohmann::ordered_json j;
  j["studentId"] = student_id;
  j["testId"] = test_id;

  nlohmann::ordered_json& simulation = j["simulation"];
  simulation["totalSteps"] = steps.size();
  simulation["steps"] = nlohmann::json::array();

  for (const SimulationStep& s : steps) {
    nlohmann::ordered_json targets = nlohmann::json::array();
    for (const Target& t : s.all_targets_) {
      targets.push_back({
        {"id", t.target_id_},
        {"x", t.pos_.x_},
        {"y", t.pos_.y_},
      });
    }

    simulation["steps"].push_back({{"targetIndex", s.target_id_},
                                   {"direction", s.direction_},
                                   {"state", s.state_},
                                   {"position", {{"x", s.position_.x_}, {"y", s.position_.y_}}},
                                   {"dropPoint", {{"x", s.drop_point_.x_}, {"y", s.drop_point_.y_}}},
                                   {"aimPoint", {{"x", s.aim_point_.x_}, {"y", s.aim_point_.y_}}},
                                   {"predictedTarget", {{"x", s.predicted_target_.x_}, {"y", s.predicted_target_.y_}}},
                                   {"targetPosition", {{"x", s.target_position_.x_}, {"y", s.target_position_.y_}}},
                                   {"timeSecSinceStart", s.elapsed_}});
  }

  return j;
}
