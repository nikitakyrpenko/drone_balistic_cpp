#pragma once

#include "DroneLink.hpp"
#include "ScheduledWorker.hpp"
#include "ThreadSafeQueue.hpp"
#include "UartLink.hpp"
#include "service/interfaces/IConfigLoader.hpp"
#include "service/interfaces/IDronePhysics.hpp"

#include <memory>
#include <mutex>

class UartDronePhysics : public IDronePhysics, public ScheduledWorker {
private:
  std::shared_ptr<UartLink> uart_link_;
  std::shared_ptr<IConfigLoader> config_loader_;

  SynchronizedQueue<dlink::Telemetry>& telemetry_channel_;
  mutable SynchronizedQueue<DroneCommand> command_channel_;

  DroneTelemetry telemetry_;
  const DroneSpec spec_;
  DroneCommand command_{};

  mutable std::mutex tel_mtx_;
  mutable std::mutex command_mtx_;

  static const IState* state_from_mode(DroneMode mode);

  void apply(const dlink::Telemetry& t);
  void tick() override;

  dlink::Control to_control(const DroneCommand& cmd) const;

public:
  UartDronePhysics(std::shared_ptr<UartLink> uart_link, std::shared_ptr<IConfigLoader> config_loader);

  void submit_command(const DroneCommand& command) const override;
  const DroneTelemetry get_telemetry() const override;
  const DroneSpec get_spec() const override;
  const DroneCommand get_active_command() const override;
  const IState* get_state() const override;
};
