#include "FileResultReporter.hpp"
#include "GpioSignal.hpp"
#include "HttpResultReporter.hpp"
#include "MavlinkCommandService.hpp"
#include "MavlinkDispatcherService.hpp"
#include "MavlinkLivenessService.hpp"
#include "MavlinkTelemetryService.hpp"
#include "UdpLink.hpp"
#include "models/SimulationStep.hpp"
#include "service/MissionFactory.hpp"

#include <chrono>
#include <exception>
#include <iostream>
#include <latch>
#include <string>
#include <vector>

static constexpr int MAX_ITERATIONS = 10000;
static constexpr std::chrono::milliseconds MISSION_DONE_PULSE{100};
static constexpr const char* BALLISTIC_TABLE_PATH = "data/ballistic_table.txt";

static constexpr const char* REPORT_HOST = "cppmiltech.com.ua";
static constexpr const char* REPORT_PATH = "/api/dz12/results";
static constexpr std::string CURRENT_STUDENT_ID = "2063";

static constexpr const char* QGC_HOST = "127.0.0.1";
static constexpr uint16_t QGC_PORT = 14550;

int main(int argc, char* argv[])
{
  if (argc < 7) {
    std::cerr << "Usage: " << argv[0] << " <gpiochip> <serial_device> <start_line> <end_line> <api_key> <task_id>\n";
    return 1;
  }

  const char* gpiochip_name = argv[1];
  const char* serial = argv[2];
  const unsigned int start_line = static_cast<unsigned int>(std::stoul(argv[3]));
  const unsigned int end_line = static_cast<unsigned int>(std::stoul(argv[4]));
  const std::string API_KEY = argv[5];
  const std::string TASK_ID = argv[6];

  try {
    UdpLink qgc_link(QGC_HOST, QGC_PORT);

    // heartbeat service
    std::latch liveness_latch{1};
    MavlinkLivenessService liveness(qgc_link);
    liveness.start(liveness_latch);
    std::cout << "mavlink heartbeat started\n";

    // command ack service
    MavlinkCommandService commands(qgc_link);

    // command service to dispatch incoming udp packets
    std::latch dispatcher_latch{1};
    MavlinkDispatcherService dispatcher(qgc_link);

    dispatcher.register_handler(MAVLINK_MSG_ID_COMMAND_ACK, [&commands](const mavlink_message_t& msg) { commands.handle_ack(msg); });
    dispatcher.start(dispatcher_latch);
    std::cout << "mavlink dispatcher started\n";

    GpioSignal gpio(gpiochip_name);
    gpio.request_output(start_line, 0);
    gpio.request_output(end_line, 0);
    std::cout << "occupied " << gpiochip_name << " line " << start_line << ", set low\n";

    gpio.set_high(start_line);
    std::cout << "set line " << start_line << " high — checker should start sending\n";

    std::cout << "waiting for ammo, config and telemetry packets...\n";
    MissionFactory factory;
    SimulationBundle sim = factory.create(LoaderType::UART, SolverType::TABLE, serial, nullptr, nullptr, BALLISTIC_TABLE_PATH);
    std::cout << "listening on " << serial << "\n";

    // mavlink telemetry service
    std::latch telemetry_latch{1};
    MavlinkTelemetryService mavlink_telemetry(qgc_link, sim.physics);
    mavlink_telemetry.start(telemetry_latch);
    std::cout << "mavlink telemetry started\n";

    std::latch mission_latch{1};
    sim.mission->start(mission_latch, MAX_ITERATIONS);

    sim.mission->join();  // blocks until the drone reaches the fire point (or MAX_ITERATIONS)

    const DroneTelemetry fire_telemetry = sim.physics->get_telemetry();
    if (!commands.command_long(fire_telemetry.get_position(), fire_telemetry.get_altitude())) {
      std::cerr << "fire command was never acked\n";
    }

    gpio.pulse_high(end_line, MISSION_DONE_PULSE);

    const std::vector<SimulationStep>& steps = sim.mission->get_steps();
    std::cout << "mission complete with steps:" << steps.size() << "— pulsed line :" << end_line << "\n";

    // HttpResultReporter http_reporter(REPORT_HOST, REPORT_PATH, API_KEY);
    FileResultReporter http_reporter("simulation.json");
  }
  catch (const std::exception& e) {
    std::cerr << "test harness failed: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
