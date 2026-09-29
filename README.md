# drone_balistic_cpp

A C++ simulation of a drone autonomously navigating to and engaging moving ground targets using ballistic drop calculations and predictive targeting.

## Layout

| Module | Purpose |
|---|---|
| `drone_model` | Core data types (position, ammo, target, drone) |
| `drone_utils` | Shared helpers and the drone link protocol |
| `drone_state` | Drone state machine |
| `drone_core` | Ballistic solver, target prediction, mission processing |
| `drone_service_json` | JSON config / ammo / targets loading |
| `drone_service_uart` | UART / GPIO hardware-in-the-loop service |
| `drone_transport` | Serial and TCP transport |
| `drone_mavlink` | MAVLink integration |
| `drone_mission_factory` | Mission assembly |
| `drone_reporting` | Result reporting (file, HTTP) |
| `external` | Vendored `json.hpp` and MAVLink `c_library_v2` |
| `data` | Sample config, ammo, targets, ballistic table |
| `docs` | Design notes |

## Build

    cmake --preset debug
    cmake --build --preset debug

Presets: `debug`, `relwithdebinfo`, `release`, `aarch64-debug` (cross-compile). Unit tests are off by default; enable with `-DBUILD_TESTS=ON`.

## Run

Run from the repository root so the relative `data/` paths resolve:

    ./build/debug/drone_target_cli <gpiochip> <serial-device> <start_line> <end_line> <api_key> <task_id>

The mission log is written to `simulation.json`; animate it with `scripts/visualize_simulation.py`.

## Tooling

`scripts/run-formatter.sh`, `scripts/run-tidy.sh`, `scripts/run-tests.sh`.
