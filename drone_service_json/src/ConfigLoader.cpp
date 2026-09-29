#include "json.hpp"

#include "models/Coord.hpp"
#include "service/ConfigLoader.hpp"

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

std::unique_ptr<ConfigDTO> parse_config(const char* source)
{
  std::ifstream file(source);
  if (!file.is_open()) {
    return nullptr;
  }

  nlohmann::json j;
  file >> j;

  try {
    auto dto = std::make_unique<ConfigDTO>();

    dto->position_ = {j["drone"]["position"]["x"].get<float>(), j["drone"]["position"]["y"].get<float>()};
    dto->altitude_ = j["drone"]["altitude"].get<float>();
    dto->initial_direction_ = j["drone"]["initialDirection"].get<float>();
    dto->attack_speed_ = j["drone"]["attackSpeed"].get<float>();
    dto->acceleration_path_ = j["drone"]["accelerationPath"].get<float>();
    dto->angular_speed_ = j["drone"]["angularSpeed"].get<float>();
    dto->turn_threshold_ = j["drone"]["turnThreshold"].get<float>();
    dto->ammo_ = j["ammo"].get<std::string>();
    dto->target_array_timestep_ = j["targetArrayTimeStep"].get<float>();

    dto->time_step_ = j["simulation"]["timeStep"].get<float>();
    dto->physics_timestep = j["simulation"]["physicsTimeStep"].get<float>();
    dto->target_timestep = j["simulation"]["targetTimeStep"].get<float>();
    dto->timescale = j["simulation"]["timeScale"].get<float>();
    dto->hit_radius_ = j["simulation"]["hitRadius"].get<float>();

    return dto;
  }
  catch (const nlohmann::json::exception&) {
    return nullptr;
  }
}

std::unique_ptr<TargetDTO> parse_targets(const char* source)
{
  std::ifstream file(source);
  if (!file.is_open()) {
    return nullptr;
  }

  nlohmann::json j;
  file >> j;

  try {
    auto dto = std::make_unique<TargetDTO>();

    dto->target_count_ = j["targetCount"].get<int>();
    dto->time_steps_ = j["timeSteps"].get<int>();

    for (const nlohmann::json& target : j["targets"]) {
      std::vector<Coord> coords;
      coords.reserve(dto->time_steps_);

      for (const nlohmann::json& position : target["positions"]) {
        coords.push_back({position["x"].get<float>(), position["y"].get<float>()});
      }
      dto->positions_.push_back(std::move(coords));
    }

    return dto;
  }
  catch (const nlohmann::json::exception&) {
    return nullptr;
  }
}

std::unique_ptr<AmmoDTO> parse_arsenal(const char* source)
{
  std::ifstream file(source);
  if (!file.is_open()) {
    return nullptr;
  }

  nlohmann::json j;
  file >> j;

  try {
    auto dto = std::make_unique<AmmoDTO>();

    for (const auto& item : j) {
      dto->ammos_.push_back(
        {item["name"].get<std::string>(), item["mass"].get<float>(), item["drag"].get<float>(), item["lift"].get<float>()});
    }

    return dto;
  }
  catch (const nlohmann::json::exception&) {
    return nullptr;
  }
}
}  // namespace

ConfigLoader::ConfigLoader(const char* config_source, const char* ammo_source, const char* target_source)
  : config_(parse_config(config_source))
  , arsenal_(parse_arsenal(ammo_source))
  , targets_(parse_targets(target_source))
{
  if (!config_) {
    throw std::runtime_error("ConfigLoader: cannot parse config source");
  }

  if (!arsenal_) {
    throw std::runtime_error("ConfigLoader: cannot parse ammo source");
  }

  if (!targets_) {
    throw std::runtime_error("ConfigLoader: cannot parse targets source");
  }
}

ConfigLoader::~ConfigLoader(){};
