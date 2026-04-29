#define ENABLE_LOG 1
#define ENABLE_DEBUG 1

#if ENABLE_LOG
#define LOG(fn, msg)    \
  std::cout << "[LOG] " \
            << "[" << fn << "]" << msg << std::endl
#else
#define LOG(msg)
#endif

#if ENABLE_DEBUG
#define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
#define DEBUG(msg)
#endif

#include <cfloat>
#include <cmath>
#include <fstream>
#include <iostream>

#include "json.hpp"

using json = nlohmann::json;

static const float G = 9.81F;
static const int SIM_MAX_STEPS = 10000;

enum State { STOPPED, ACCELERATING, DECELERATING, TURNING, MOVING };

struct Coord {
  float x, y;

  Coord operator+(const Coord& other) const {
    return {.x = this->x + other.x, .y = this->y + other.y};
  }
  Coord operator-(const Coord& other) const {
    return {.x = this->x - other.x, .y = this->y - other.y};
  }
  Coord operator*(float m) const {
    return {.x = this->x * m, .y = this->y * m};
  }
  Coord operator/(float d) const {
    return {.x = this->x / d, .y = this->y / d};
  }
  bool operator==(const Coord& other) const {
    return (this->x == other.x && this->y == other.y);
  }
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Coord, x, y)

float length(const Coord& from, const Coord& to) {
  float xd = to.x - from.x;
  float yd = to.y - from.y;

  return std::hypot(xd, yd);
}

struct SimulationStep {
  int targetIndex;
  float direction;
  State state;
  Coord position;
  Coord dropPoint;
  Coord aimPoint;
  Coord predictedTarget;
};

struct Output {
  int steps;
  SimulationStep** result;
};

// 8b
struct Simulation {
  float timeStep;
  float hitRadius;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Simulation, timeStep, hitRadius)

float calcTurningAngle(const Coord& from, const Coord& to, float direction) {
  float dirToTarget = std::atan2(to.y - from.y, to.x - from.x);
  float delta = dirToTarget - direction;
  while (delta > M_PI) delta -= 2.0f * M_PI;
  while (delta < -M_PI) delta += 2.0f * M_PI;
  return delta;
}

struct Drone {
  Coord position;
  float altitude;
  float initialDirection;
  float attackSpeed;
  float accelerationPath;
  float angularSpeed;
  float turnThreshold;
  float currentSpeed{0.0f};
  float currentDirection{0.0f};
  State state{State::STOPPED};

  float calcDirection(const Coord& target) const {
    return std::abs(
        calcTurningAngle(this->position, target, this->currentDirection));
  }

  inline float acceleration() const {
    return (this->attackSpeed * this->attackSpeed) /
           (2.0f * this->accelerationPath);
  }

  void incrementPosition(float t) {
    this->position.x +=
        this->currentSpeed * std::cos(this->currentDirection) * t;
    this->position.y +=
        this->currentSpeed * std::sin(this->currentDirection) * t;
  }

  bool isPositionReached(const Coord& target, float threshold) {
    Coord c = target - this->position;
    return (c.x * c.x + c.y * c.y) <= (threshold * threshold);
  }

  void incrementSpeed(float t) {
    switch (this->state) {
      case STOPPED:
        this->state = TURNING;
        return;
      case TURNING:
        this->currentSpeed = 0.0f;
        return;
      case MOVING:
        this->currentSpeed = this->attackSpeed;
        return;
      case ACCELERATING:
        this->currentSpeed += t * acceleration();
        if (this->currentSpeed >= this->attackSpeed) {
          this->currentSpeed = this->attackSpeed;
          this->state = State::MOVING;
        }
        return;
      case DECELERATING:
        this->currentSpeed -= t * acceleration();
        if (this->currentSpeed <= 0.0f) {
          this->currentSpeed = 0.0f;
          this->state = State::TURNING;
        }
        return;
    }
  }

  void incrementDirection(const Coord& target, float t) {
    float delta =
        calcTurningAngle(this->position, target, this->currentDirection);

    if (this->state == MOVING) {
      if (std::abs(delta) > this->turnThreshold)
        this->state = DECELERATING;
      else
        this->currentDirection = std::atan2(target.y - this->position.y,
                                            target.x - this->position.x);
      return;
    }

    if (this->state != TURNING) return;
    float rotStep = this->angularSpeed * t;

    if (std::abs(delta) <= rotStep) {
      this->currentDirection += delta;
      this->state = ACCELERATING;
    } else {
      this->currentDirection += (delta > 0 ? 1.0f : -1.0f) * rotStep;
    }
  }

  float penalty(const Coord& target) const {
    switch (this->state) {
      case STOPPED:
      case TURNING: {
        float dir = calcDirection(target);
        if (dir <= this->turnThreshold) return 0.0f;
        return dir / this->angularSpeed;
      }
      case ACCELERATING:
      case DECELERATING: {
        float dir = calcDirection(target);
        float t = this->currentSpeed / acceleration();
        if (dir <= this->turnThreshold) return t;
        return t + (dir / this->angularSpeed);
      }
      case MOVING: {
        float dir = calcDirection(target);
        float t = this->attackSpeed / acceleration();
        if (dir <= this->turnThreshold) return t;
        return t + (dir / this->angularSpeed);
      }
    }
    return 0.0f;
  }
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Drone, position, altitude, initialDirection,
                                   attackSpeed, accelerationPath, angularSpeed,
                                   turnThreshold)

//
struct Config {
  Drone drone;
  Simulation simulation;
  std::string ammo;
  float targetArrayTimeStep;
};

struct Ammo {
  char name[32];
  float mass;
  float drag;
  float lift;
};

struct Arsenal {
  int size;
  Ammo* ammos;
};

struct Task {
  int targetId;
  Coord intermidiate;
  Coord fire;
  float ttr;
  bool hasIntermidiate;
};

//?
struct Targets {
  int targetCount;
  int timeSteps;
  Coord** coords;

  Coord* operator[](int i) const { return coords[i]; }

  Coord* targetsAtT(float t, float arrayTimeStep) const {
    int idx = (int)std::floor(t / arrayTimeStep) % timeSteps;
    int next = (idx + 1) % timeSteps;
    float frac = (t - idx * arrayTimeStep) / arrayTimeStep;

    Coord* slice = new Coord[targetCount];
    for (int i = 0; i < targetCount; i++)
      slice[i] = coords[i][idx] + (coords[i][next] - coords[i][idx]) * frac;
    return slice;
  }

  Coord* interpolateByTravelTime(int targetId, float t, float arrayTimeStep,
                                 float travelTime) const {
    int idx = (int)std::floor(t / arrayTimeStep) % timeSteps;
    int next = (idx + 1) % timeSteps;
    float frac = (t - idx * arrayTimeStep) / arrayTimeStep;

    Coord curr = coords[targetId][idx] +
                 (coords[targetId][next] - coords[targetId][idx]) * frac;
    Coord future = coords[targetId][next];

    float vx = (future.x - curr.x) / arrayTimeStep;
    float vy = (future.y - curr.y) / arrayTimeStep;

    return new Coord{curr.x + vx * travelTime, curr.y + vy * travelTime};
  }
};

Config* config(const char filename[]) {
  std::ifstream ifs(filename);

  if (!ifs.is_open()) {
    std::cerr << "File not found : " << filename << std::endl;
    return nullptr;
  }

  json j;
  ifs >> j;

  Drone d;
  Simulation s;
  std::string ammo;
  float targetArrayTimeStep;

  try {
    d = j.at("drone").get<Drone>();
    s = j.at("simulation").get<Simulation>();
    ammo = j.at("ammo").get<std::string>();
    targetArrayTimeStep = j.at("targetArrayTimeStep").get<float>();
  } catch (const json::exception& e) {
    std::cerr << e.what() << std::endl;
    return nullptr;
  };

  return new Config{.drone = std::move(d),
                    .simulation = std::move(s),
                    .ammo = std::move(ammo),
                    .targetArrayTimeStep = targetArrayTimeStep};
}

Targets* targets(const char filename[]) {
  std::ifstream ifs(filename);

  if (!ifs.is_open()) {
    std::cerr << "File not found : " << filename << std::endl;
    return nullptr;
  }

  json jt;
  ifs >> jt;

  int targetCount, timeSteps;
  Coord** targets;

  try {
    targetCount = jt.at("targetCount").get<int>();
    timeSteps = jt.at("timeSteps").get<int>();

    targets = new Coord*[targetCount];

    for (int i = 0; i < targetCount; i++) {
      targets[i] = new Coord[timeSteps];
      for (int j = 0; j < timeSteps; j++) {
        targets[i][j].x = jt["targets"][i]["positions"][j]["x"];
        targets[i][j].y = jt["targets"][i]["positions"][j]["y"];
      }
    }
  } catch (const json::exception& e) {
    std::cerr << e.what() << std::endl;
    return nullptr;
  };

  return new Targets{targetCount, timeSteps, targets};
}

Arsenal* arsenal(const char filename[]) {
  std::ifstream fa(filename);

  if (!fa.is_open()) {
    std::cerr << "File not found : " << filename << std::endl;
    return nullptr;
  }

  json ja;

  try {
    fa >> ja;
    int ammoCount = ja.size();
    Ammo* ammo = new Ammo[ammoCount];
    for (int i = 0; i < ammoCount; i++) {
      std::strncpy(ammo[i].name, ja[i]["name"].get<std::string>().c_str(), 31);
      ammo[i].mass = ja[i]["mass"];
      ammo[i].drag = ja[i]["drag"];
      ammo[i].lift = ja[i]["lift"];
    }
    return new Arsenal{.size = ammoCount, .ammos = ammo};
  } catch (const json::exception& e) {
    std::cerr << e.what() << std::endl;
    return nullptr;
  }
}

Ammo* fetchByName(const char ammo[], const Arsenal& arsenal) {
  for (size_t i = 0; i < arsenal.size; i++) {
    if (std::strcmp(arsenal.ammos[i].name, ammo) == 0) {
      return &arsenal.ammos[i];
    }
  }
  std::cerr << "No such ammo type : " << ammo << std::endl;
  return nullptr;
}

float calcTAmmo(const Drone& drone, const Ammo& ammo) {
  float a = (ammo.drag * G * ammo.mass) -
            (2.0f * ammo.drag * ammo.drag * ammo.lift * drone.attackSpeed);
  float b = (-3.0f * G * ammo.mass * ammo.mass) +
            (3.0f * ammo.drag * ammo.lift * ammo.mass * drone.attackSpeed);
  float c = 6.0f * ammo.mass * ammo.mass * drone.altitude;

  float p = (-1.0f * b * b) / (3.0f * a * a);
  float q = (2.0f * b * b * b) / (27.0f * a * a * a) + (c / a);

  float acos_arg = (3.0f * q) / (2.0f * p) * std::sqrt(-3.0f / p);
  if (acos_arg < -1.0f || acos_arg > 1.0f) {
    LOG("calcTAmmo", "Zerro return");
    return 0;
  };

  float ttf =
      2.0f * std::sqrt(-p / 3.0f) *
          std::cos((std::acos(acos_arg) + 4.0f * std::acos(-1.0f)) / 3.0f) -
      b / (3.0f * a);
  return ttf;
}

float calcHDistance(const Drone& drone, const Ammo& ammo, float ttf) {
  if (ttf == 0) {
    LOG("calcHDistance", "Zerro return");
    return 0;
  }

  float d = ammo.drag;
  float l = ammo.lift;
  float m = ammo.mass;
  float attackSpeed = drone.attackSpeed;

  float dtf =
      (std::pow(ttf, 3) *
       ((6.0f * d * G * l * m) -
        (6.0f * std::pow(d, 2) * (std::pow(l, 2) - 1) * attackSpeed))) /
          (36.0f * std::pow(m, 2)) +
      (std::pow(ttf, 5) * ((3.0f * std::pow(d, 3) * G * std::pow(l, 3) * m) -
                           (3.0f * std::pow(d, 4) * std::pow(l, 2) *
                            (std::pow(l, 2) + 1) * attackSpeed))) /
          (36.0f * (std::pow(l, 2) + 1) * std::pow(m, 4)) +
      (std::pow(ttf, 4) * ((3.0f * std::pow(d, 3) * (std::pow(l, 2) + 1) *
                            std::pow(l, 2) * attackSpeed) +
                           (6.0f * std::pow(d, 3) * (std::pow(l, 2) + 1) *
                            std::pow(l, 4) * attackSpeed) -
                           (6.0f * std::pow(d, 2) * G *
                            (std::pow(l, 4) + std::pow(l, 2) + 1) * l * m))) /
          (36.0f * std::pow(std::pow(l, 2) + 1, 2) * std::pow(m, 3)) -
      (d * std::pow(ttf, 2) * attackSpeed) / (2.0f * m) + (ttf * attackSpeed);
  return dtf;
}

// calculate time that will take to reach "to" from "from" given drone
// parameters
float calcTReach(const Coord& from, const Coord& to, float currentSpeed,
                 float attackSpeed, float accelerationPath, float a) {
  float d = length(from, to);

  float remAccelerationDistance =
      (attackSpeed * attackSpeed - currentSpeed * currentSpeed) / (2.0f * a);

  if (remAccelerationDistance > d) {
    return (-currentSpeed +
            std::sqrt(currentSpeed * currentSpeed + 2.0f * a * d)) /
           a;
  }
  return (d +
          accelerationPath * std::pow(1.0f - currentSpeed / attackSpeed, 2)) /
         attackSpeed;
}

Task* calcTask(const Drone& drone, const Coord& target, float dtf,
               int targetId) {
  float distance = length(target, drone.position);

  if (dtf + drone.accelerationPath < distance) {
    float ratio = (distance - dtf) / distance;

    Coord fire = Coord{(drone.position + ((target - drone.position) * ratio))};

    float ttt = std::abs(calcTurningAngle(drone.position, fire,
                                          drone.currentDirection)) /
                drone.angularSpeed;
    float ttr =
        calcTReach(drone.position, fire, drone.currentSpeed, drone.attackSpeed,
                   drone.accelerationPath, drone.acceleration());
    return new Task{.targetId = targetId,
                    .intermidiate = fire,
                    .fire = fire,
                    .ttr = ttr + ttt,
                    .hasIntermidiate = false};
  } else {
    Coord intermidiate = {Coord(target - (target - drone.position) *
                                             (dtf + drone.accelerationPath) /
                                             distance)};
    float d2 = length(target, intermidiate);
    float ratio = (d2 - dtf) / d2;
    Coord fire = Coord{(intermidiate + ((target - intermidiate) * ratio))};

    // calculate time to reach from curr position to intermidiate position
    float ttrIntermidiate = calcTReach(
        drone.position, intermidiate, drone.currentSpeed, drone.attackSpeed,
        drone.accelerationPath, drone.acceleration());

    // calculate time to reach from intermidiate position to fire position
    float ttrFire = calcTReach(intermidiate, fire, 0.0f, drone.attackSpeed,
                               drone.accelerationPath, drone.acceleration());

    // calculate time to turn from intermidiate to fire
    float tttFire = std::abs(calcTurningAngle(
                        intermidiate, fire,
                        std::atan2(intermidiate.y - drone.position.y,
                                   intermidiate.x - drone.position.x))) /
                    drone.angularSpeed;

    return new Task{.targetId = targetId,
                    .intermidiate = intermidiate,
                    .fire = fire,
                    .ttr = ttrIntermidiate + tttFire + ttrFire,
                    .hasIntermidiate = true};
  }
}

Task* getTaskByMinTTR(Task* tasks[], int size) {
  int idx = 0;

  for (int i = 0; i < size; i++)
    if (tasks[i]->ttr < tasks[idx]->ttr) idx = i;

  return tasks[idx];
}

Output loop(const Config& c, const Targets& targets, const Ammo& a) {
  Drone drone = c.drone;
  float quant{c.simulation.timeStep};
  int iter{0};

  float ttf = calcTAmmo(drone, a);
  float dtf = calcHDistance(drone, a, ttf);

  int currentTargetId = -1;
  bool visitedIntermidiate = false;

  SimulationStep* result[SIM_MAX_STEPS]{};

  while (iter < SIM_MAX_STEPS) {
    Coord* targetsAtT = targets.targetsAtT(quant, c.targetArrayTimeStep);

    Task* tasks[targets.targetCount];
    for (int i = 0; i < targets.targetCount; i++) {
      tasks[i] = calcTask(drone, targetsAtT[i], dtf, i);
    }

    Coord* interpolated[targets.targetCount];
    for (int i = 0; i < targets.targetCount; i++) {
      interpolated[i] = targets.interpolateByTravelTime(
          i, quant, c.targetArrayTimeStep, tasks[i]->ttr + ttf);
    }

    Task* predicted[targets.targetCount];
    for (int i = 0; i < targets.targetCount; i++) {
      predicted[i] = calcTask(drone, *interpolated[i], dtf, i);
    }

    Task* optimal = getTaskByMinTTR(predicted, targets.targetCount);

    if (currentTargetId == -1 || currentTargetId == optimal->targetId) {
      currentTargetId = optimal->targetId;
    } else {
      float penalty = drone.penalty(
          optimal->hasIntermidiate ? optimal->intermidiate : optimal->fire);
      if (optimal->ttr + penalty < predicted[currentTargetId]->ttr) {
        if (currentTargetId != optimal->targetId) {
          visitedIntermidiate = false;
        }
        DEBUG("Target switched ["
              << currentTargetId << " ttr: " << predicted[currentTargetId]->ttr
              << "] -> [" << optimal->targetId << " ttr: " << optimal->ttr
              << " pen: " << penalty << "]");
        currentTargetId = optimal->targetId;
      }
    }

    if (!visitedIntermidiate)
      visitedIntermidiate =
          !optimal->hasIntermidiate ||
          drone.isPositionReached(optimal->intermidiate, 0.5f);

    Coord& dropPoint =
        visitedIntermidiate ? optimal->fire : optimal->intermidiate;

    drone.incrementSpeed(c.simulation.timeStep);
    drone.incrementDirection(dropPoint, c.simulation.timeStep);
    drone.incrementPosition(c.simulation.timeStep);

    DEBUG("Step " << iter);
    DEBUG("Drone "
          << "[x: " << drone.position.x << "; y: " << drone.position.y << "]"
          << " State : " << drone.state);
    DEBUG("Drop "
          << " [x: " << dropPoint.x << "; y: " << dropPoint.y
          << "; intermidiate : " << visitedIntermidiate << "]");

    // will copy by value to simplify memory free
    result[iter] = new SimulationStep{
        .targetIndex = currentTargetId,
        .direction = drone.currentDirection,
        .state = drone.state,
        .position = drone.position,
        .dropPoint = dropPoint,
        .aimPoint = drone.position + Coord{std::cos(drone.currentDirection),
                                           std::sin(drone.currentDirection)} *
                                         dtf,
        .predictedTarget = *(interpolated[currentTargetId]),
    };

    for (int i = 0; i < targets.targetCount; i++) {
      delete tasks[i];
      delete interpolated[i];
      delete predicted[i];
    }

    if (visitedIntermidiate &&
        length(drone.position, targetsAtT[currentTargetId]) <= dtf) {
      delete[] targetsAtT;
      iter++;
      break;
    }

    delete[] targetsAtT;

    quant += c.simulation.timeStep;
    iter++;
  }

  SimulationStep** simSteps = new SimulationStep*[iter];
  for (int i = 0; i < iter; i++) {
    simSteps[i] = result[i];
  }

  return {.steps = iter, .result = simSteps};
}

#ifdef ENABLE_DEBUG
void dumpCsv(const Output& out) {
  std::ofstream ofs("simulation.txt");

  ofs << out.steps << "\n";

  for (int i = 0; i < out.steps; i++)
    ofs << out.result[i]->position.x << " " << out.result[i]->position.y << " ";
  ofs << "\n";

  for (int i = 0; i < out.steps; i++) ofs << out.result[i]->direction << " ";
  ofs << "\n";

  for (int i = 0; i < out.steps; i++) ofs << out.result[i]->state << " ";
  ofs << "\n";

  for (int i = 0; i < out.steps; i++) ofs << out.result[i]->targetIndex << " ";
  ofs << "\n";

  ofs.close();
}
#endif

void dumpJson(const Output& out) {
  nlohmann::ordered_json j;
  j["totalSteps"] = out.steps;
  j["steps"] = json::array();

  for (int i = 0; i < out.steps; i++) {
    const SimulationStep* s = out.result[i];
    j["steps"].push_back(
        {{"position", {{"x", s->position.x}, {"y", s->position.y}}},
         {"direction", s->direction},
         {"state", s->state},
         {"targetIndex", s->targetIndex},
         {"dropPoint", {{"x", s->dropPoint.x}, {"y", s->dropPoint.y}}},
         {"aimPoint", {{"x", s->aimPoint.x}, {"y", s->aimPoint.y}}},
         {"predictedTarget",
          {{"x", s->predictedTarget.x}, {"y", s->predictedTarget.y}}}});
  }

  std::ofstream ofs("simulation.json");
  ofs << j.dump(2);
  ofs.close();
}

int main() {
  Config* c = config("config.json");

  if (c == nullptr) return 1;

  const Targets* t = targets("targets.json");

  if (t == nullptr) return 1;

  const Arsenal* a = arsenal("ammo.json");

  if (a == nullptr) return 1;

  const Ammo* ammo = fetchByName(c->ammo.c_str(), *a);

  if (ammo == nullptr) return 1;

  Output out = loop(*c, *t, *ammo);

#ifdef ENABLE_DEBUG
  dumpCsv(out);
#endif

  dumpJson(out);

  for (int i = 0; i < out.steps; i++) delete out.result[i];
  delete[] out.result;

  delete c;

  delete[] a->ammos;
  delete a;

  for (int i = 0; i < t->targetCount; i++) delete[] t->coords[i];
  delete[] t->coords;
  delete t;

  return 0;
}