# Task 03 — Drone Strike Simulation

A C++ simulation of a drone autonomously navigating to and engaging moving ground targets using ballistic drop calculations and predictive targeting.

## Overview

The simulation models a drone that:
1. Selects the optimal target from a set of moving targets based on time-to-reach (TTR)
2. Predicts where each target will be by the time the munition arrives
3. Navigates to a computed fire point (accounting for munition horizontal drift during fall)
4. Switches targets only when switching is provably faster (with a penalty for re-orientation)

## Files

| File | Description |
|------|-------------|
| `main.cpp` | Full simulation source |
| `json.hpp` | Single-header JSON library ([nlohmann/json](https://github.com/nlohmann/json)) |
| `config.json` | Drone parameters and simulation settings |
| `ammo.json` | Munition definitions (mass, drag, lift) |
| `targets.json` | Pre-recorded target trajectories (5 targets × 120 time steps) |
| `simulation.json` | Output — step-by-step drone state (generated on run) |
| `simulation.txt` | Debug CSV output (generated when `ENABLE_DEBUG=1`) |

## Build

Requires g++ with C++17 support.

```bash
g++ -std=c++17 -O2 main.cpp -o main
```

Or use the VS Code build task (`Ctrl+Shift+B`).

## Run

```bash
./main
```

Reads `config.json`, `ammo.json`, and `targets.json` from the current directory.  
Writes results to `simulation.json` (and `simulation.txt` in debug builds).

## Configuration (`config.json`)

```json
{
  "drone": {
    "position": { "x": 250, "y": 250 },
    "altitude": 120,
    "attackSpeed": 15,
    "accelerationPath": 20,
    "angularSpeed": 0.8,
    "turnThreshold": 0.1
  },
  "ammo": "VOG-17",
  "simulation": { "timeStep": 0.1, "hitRadius": 3 },
  "targetArrayTimeStep": 10
}
```

| Parameter | Unit | Description |
|-----------|------|-------------|
| `altitude` | m | Drop height for ballistic calculation |
| `attackSpeed` | m/s | Max drone speed |
| `accelerationPath` | m | Distance to reach full speed from stop |
| `angularSpeed` | rad/s | Rotation rate |
| `turnThreshold` | rad | Angle within which the drone does not stop to turn |
| `timeStep` | s | Simulation tick size |
| `targetArrayTimeStep` | s | Time interval between target trajectory samples |

## Ammo Types (`ammo.json`)

| Name | Mass (kg) | Drag | Lift | Notes |
|------|-----------|------|------|-------|
| VOG-17 | 0.35 | 0.07 | 0 | Default grenade |
| M67 | 0.60 | 0.10 | 0 | |
| RKG-3 | 1.20 | 0.10 | 0 | |
| GLIDING-VOG | 0.45 | 0.10 | 1 | Gliding variant |
| GLIDING-RKG | 1.40 | 0.10 | 1 | Gliding variant |

`lift > 0` enables horizontal glide — the munition travels further forward during descent.

## Physics

**Time-to-fall (`calcTAmmo`)** — solves the vertical motion cubic equation accounting for drag and lift to find flight time from `altitude`.

**Horizontal drift (`calcHDistance`)** — computes forward distance travelled during fall at drone attack speed with drag, used to determine the fire point offset from target.

**Predictive targeting** — at each step, the drone estimates where each target will be at `t + ttr + ttf`, plans a task to that predicted position, then picks the task with minimum TTR.

**Target switching** — a switch only happens if `new_ttr + switching_penalty < current_ttr`, where the penalty models the time lost stopping, turning, and re-accelerating.

## Output (`simulation.json`)

Each step contains:

```json
{
  "position": { "x": ..., "y": ... },
  "direction": 1.23,
  "state": 4,
  "targetIndex": 2,
  "dropPoint": { "x": ..., "y": ... },
  "aimPoint":  { "x": ..., "y": ... },
  "predictedTarget": { "x": ..., "y": ... }
}
```

**State values:** `0` STOPPED · `1` ACCELERATING · `2` DECELERATING · `3` TURNING · `4` MOVING

## Debug Build

Enable verbose logging by setting the macros at the top of `main.cpp`:

```cpp
#define ENABLE_LOG 1
#define ENABLE_DEBUG 1
```

This prints per-step drone state and target-switch events to stdout, and writes `simulation.txt` with positions, directions, states, and target indices as space-separated rows.
