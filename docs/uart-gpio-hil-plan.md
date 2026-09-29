# Hardware-in-the-loop (HIL) UART + GPIO support

Branch: `feature/10-uart-control`

## Context

`checker_pi_arm64` is a grading/simulator binary the student app must talk to over UART
(`/dev/ttyHS2`) plus a GPIO handshake, instead of (today's) JSON-file-driven offline
simulation. Goal: add HIL support **alongside** the existing JSON path, without modifying it.

## Locked-in decisions

1. **Location**: new sibling module `drone_transport/`, depends on `drone_utils`
   + `drone_model`, feeds `drone_service` through its existing interfaces (`ITargetProvider`,
   `IDronePhysics`, `IConfigLoader`) rather than duplicating `MissionProccessor`'s logic.
2. **Wire protocol — already implemented**, `drone_utils/include/DroneLink.hpp`
   (header-only, untracked/unused today). Frame:
   `MAGIC0(0xA5) MAGIC1(0x5A) TYPE LEN payload[LEN] CRC16-LE`, CRC-16/CCITT-FALSE
   (poly 0x1021, init 0xFFFF) over `TYPE+LEN+payload`, little-endian fields.
   `PacketType`: `PKT_TELEMETRY=0x01`, `PKT_TARGET=0x02`, `PKT_AMMO=0x03`, `PKT_RESULT=0x04`,
   `PKT_CONTROL=0x05`. Packed structs: `Telemetry{t_ms,x,y,z,vx,vy,speed,dir,state}`,
   `TargetPos{id,x,y}`, `AmmoCfg{name[16],mass,drag,lift,hitRadius,nTargets}`,
   `Result{hit,targetId,miss_m,drop_t_ms}`, `Control{accel,turnRate}`. Ships `dlink::encode()`
   and incremental `dlink::Parser::feed(byte, outType, outPayload, outLen) -> bool`
   (auto-resyncs on garbage). **Reuse as-is.**
3. **Augment, not replace** — `Main.cpp`/`MissionFactory`/JSON `ConfigLoader`/`TargetProvider`
   stay untouched; HIL is a new additional entrypoint/build target.
4. **UART setup** (hardcoded, no config):
   ```c
   tcgetattr(fd, &tio);
   cfmakeraw(&tio);              // 8N1, no special char processing
   cfsetispeed(&tio, B115200);
   cfsetospeed(&tio, B115200);   // same speed both directions
   tcsetattr(fd, TCSANOW, &tio);
   ```
5. **GPIO**: libgpiod v1 C API. Line 24 on `gpiochip4` requested as **output, default LOW**,
   pulsed **HIGH** only to signal the checker. No `ACTIVE_LOW` flag — checker reads the raw
   level on line 25 (jumpered to 24) and triggers on raw==1.

## Reverse-engineering findings from `checker_pi_arm64`

- **Baud = 115200**, hardcoded in the checker too (no `--baud` flag exists) — matches decision #4.
- **`--drop-line 58` is a second checker-side GPIO *input***, distinct from `--start-line 25`.
  Log strings (`"hw GPIO: %s START=line%d DROP=line%d"`, MISS-on-timeout message) show the
  checker waits for a **second physical pulse** signaling "ammo released now," independent of
  any UART traffic, and auto-scores MISS if it times out. This means the student app needs a
  **second GPIO output line** (offset TBD) wired to the checker's drop-line input, pulsed at
  the moment the mission logic decides to fire — this materially extends the original
  single-GPIO-line plan to **two output lines** (START + DROP).
- Packet direction is inferred as **checker → student** for `PKT_AMMO`/`PKT_TELEMETRY`/
  `PKT_TARGET` (checker owns ground-truth physics/mission config in `--hw` mode), and
  **student → checker** for `PKT_CONTROL`. Not 100% certain — flagged below.

## New module: `drone_transport/`

```
drone_transport/
  CMakeLists.txt
  include/transport/
    UartPort.hpp        GpioSignal.hpp       FrameReader.hpp     FrameWriter.hpp
    UartTargetProvider.hpp   UartDronePhysics.hpp   UartConfigLoader.hpp
    ControlBridge.hpp   HilSession.hpp
  src/  (mirrors each header)
```

- **`UartPort`** — RAII wrapper around the termios setup above; `read_some()`/`write_all()`.
- **`GpioSignal`** — libgpiod v1 wrapper, one instance per line (`gpiod_line_request_output`
  with default value 0); `set_high()/set_low()/pulse_high(duration)`. One instance per line,
  touched from a single call site only (no internal locking).
- **`FrameReader`** — background thread feeding bytes into `dlink::Parser`, demuxing complete
  frames by type into sinks: `SynchronizedQueue<Telemetry>` (drain-to-last), a target sink
  feeding `UartTargetProvider`, a one-shot callback for `PKT_AMMO`, a queue for `PKT_RESULT`
  (logged only). Mirrors the existing `start()/interrupt()`/destructor-joins-thread pattern
  already used by `TargetProvider`/`DronePhysics`.
- **`FrameWriter`** — `send_control(Control)` via `dlink::encode(PKT_CONTROL, ...)`.
- **`UartTargetProvider : ITargetProvider`** — aggregates `PKT_TARGET` packets by id. The wire
  packet has no velocity field, so velocity is estimated by finite-difference between
  consecutive packets per id using wall-clock receipt time (no per-target timestamp exists on
  the wire) — flagged as a real modeling choice, not free reuse.
- **`UartDronePhysics : IDronePhysics`** — reflects the latest `Telemetry` packet;
  `step()` is a no-op (checker owns real physics), just records the last `DroneCommand` for
  `get_active_command()` parity. `DroneSpec` (altitude, attack_speed, acceleration_path,
  angular_speed, turn_threshold) **has no wire representation at all** — must come from a
  small local JSON file (same schema subset as today's `config.json`, ammo field unused/
  overwritten at runtime).
- **`HilConfigLoader : IConfigLoader`** (decorator, not full reimplementation) — wraps a
  normal `ConfigLoader` loaded from a local config+table file for drone-spec/timestep/
  ballistic-table fields, and **overlays** ammo name/mass/drag/lift/hitRadius once a
  `PKT_AMMO` frame arrives. Keeps `MissionProccessor` **100% unmodified**.
- **`ControlBridge`** — drains the *same* `SynchronizedQueue<DroneCommand>` channel
  `MissionProccessor` already writes to (zero changes there), translates to `Control`:

  | `DroneCommand.state` | `accel` | `turnRate` |
  |---|---|---|
  | ACCELERATING | +1.0 | from heading error |
  | DECELERATING | -1.0 | from heading error |
  | MOVING | 0.0 | from heading error |
  | TURNING | 0.0 | from heading error (likely near ±1.0) |
  | STOPPED | 0.0 | 0.0 |

  `turnRate = clamp(heading_error / max_expected_delta, -1, 1)` — `max_expected_delta`
  normalization basis is unconfirmed (checker has its own `maxW` we can't read); recommend
  exposing it as a tunable constant to calibrate empirically against the real checker.
- **`HilSession`** — orchestrator: requests both GPIO lines low at startup (before opening
  UART) → opens UART → starts `FrameReader` → waits for `PKT_AMMO` → pulses START line →
  constructs `MissionProccessor` (deferred until ammo known, since its ctor does an
  ammo-name lookup that throws if missing) → starts `MissionProccessor` + `ControlBridge` →
  on `has_finished()`, pulses DROP line → logs any `PKT_RESULT` → tears down via RAII.

## Build changes

- `drone_transport/CMakeLists.txt`: new pattern for this repo —
  `pkg_check_modules(GPIOD REQUIRED libgpiod)` (libgpiod-dev 1.6.3 confirmed installed),
  static lib linking `drone_model drone_utils drone_service ${GPIOD_LIBRARIES}`.
- `CMakeLists.txt`: add `add_subdirectory(drone_transport)` and a new
  `drone_target_hil` executable from a new `src/MainHil.cpp`. Existing `drone_target_cli`
  target and `Main.cpp` untouched.

## Reused unmodified

`DroneLink.hpp`, `ThreadSafeQueue.hpp`, `MissionProccessor`, the `IState`/state-machine
classes, `FirepointProvider`/ballistic solvers, JSON `ConfigLoader`/`TargetProvider`,
`MissionFactory`, `Main.cpp`.

## Open questions (must confirm before/while implementing)

1. **Second GPIO line (DROP)** — exact `gpiochip4` offset to drive for the checker's
   `--drop-line 58`, and its idle/pulse polarity scheme.
2. **GPIO pulse width and START timing relative to `PKT_AMMO`** — does the checker wait for
   START before sending anything over UART, or proactively send AMMO first? Needs testing
   against the real checker.
3. **Packet direction confirmation** for `PKT_AMMO`/`PKT_TELEMETRY`/`PKT_TARGET` — inferred,
   not certain.
4. **`AmmoCfg.nTargets` gating** — start the mission loop as soon as ammo arrives (targets
   populate asynchronously; `MissionProccessor::step()` already handles an empty target list
   safely), or block until exactly `nTargets` distinct `PKT_TARGET` ids have been seen?
5. **`PKT_RESULT` handling** — confirmed it's just a logged back-channel verdict; confirm
   nothing else needs to react to it.
6. **UART error/reconnect behavior** — no spec; current plan is to let `Parser` resync on
   garbage and log read errors without crashing, no auto-reconnect.
7. **`accel`/`turnRate` normalization basis** — the checker scales `[-1,1]` by its own
   `maxAccel`/`maxW`, values unknown to the student app; likely needs empirical calibration.
8. **HIL pacing** — JSON mode's `sim_timestep_`/`timescale_` drive an artificial sleep loop;
   in HIL mode real telemetry arrival is the true pacing signal. Recommend `timescale_ = 1.0`
   and a small `sim_timestep_`, relying on `drain_to_last()` semantics already in place.

## Critical files

- `drone_utils/include/DroneLink.hpp` (protocol, reuse as-is)
- `drone_service/include/service/MissionProccesor.hpp` / `src/MissionProccesor.cpp`
- `drone_service/include/service/interfaces/{IConfigLoader,ITargetProvider,IDronePhysics}.hpp`
- `drone_service/include/service/ConfigLoader.hpp` / `src/ConfigLoader.cpp` (pattern to mirror for `HilConfigLoader`)
- `drone_service/CMakeLists.txt` (pattern to mirror for `drone_transport/CMakeLists.txt`)
- `src/Main.cpp` (sequencing pattern to mirror for `MainHil.cpp`)
- `CMakeLists.txt`

## Verification

- `cmake --build` the new `drone_target_hil` target cross-compiled for the board; confirm
  `drone_target_cli`/existing tests still build unchanged.
- Bench test: run `checker_pi_arm64 --hw --uart /dev/ttyHS2 --gpiochip gpiochip4 --start-line 25
  --drop-line 58`, run `drone_target_hil` against it, confirm START pulse is observed
  (checker log transitions out of "waiting for START"), AMMO/TARGET/TELEMETRY packets are
  parsed (add temporary stderr logging in `FrameReader`), CONTROL packets are sent at a
  reasonable rate, and DROP pulses at mission end with a `PKT_RESULT` verdict logged.
