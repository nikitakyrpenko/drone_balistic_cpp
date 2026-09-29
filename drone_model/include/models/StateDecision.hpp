class IState;

struct StateDecision {
  const IState* next_state_;
  float dir{};
};
