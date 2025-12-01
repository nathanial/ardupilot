# SITL (Software In The Loop) Architecture

This document explains how ArduPilot's SITL simulation works in detail, covering the architecture, data flow, and integration points for building custom debugging/visualization tools.

---

## Part 1: Conceptual Overview

### 1.1 What is SITL?

SITL (Software In The Loop) is ArduPilot's simulation environment that allows the **exact same flight code** that runs on real hardware to run on a desktop computer. This is achieved through a Hardware Abstraction Layer (HAL) that provides simulated versions of all hardware interfaces.

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│                    ArduPilot Flight Code                        │
│            (ArduCopter, ArduPlane, Rover, ArduSub)             │
│                                                                 │
│                  [ Identical code path ]                        │
│                                                                 │
├────────────────────────┬────────────────────────────────────────┤
│                        │                                        │
│    Real Hardware       │           SITL Simulation              │
│                        │                                        │
│   ┌──────────────┐     │     ┌──────────────────────────┐      │
│   │ AP_HAL_ChibiOS│     │     │     AP_HAL_SITL          │      │
│   │              │     │     │                          │      │
│   │ Real IMU     │     │     │  Simulated IMU           │      │
│   │ Real GPS     │     │     │  Simulated GPS           │      │
│   │ Real Baro    │     │     │  Simulated Baro          │      │
│   │ Real Motors  │     │     │  Simulated Physics       │      │
│   └──────────────┘     │     └──────────────────────────┘      │
│                        │                                        │
│         ↓              │                ↓                       │
│   Physical World       │        Physics Engine                  │
│                        │     (MultiCopter, Plane, etc.)         │
│                        │                                        │
└────────────────────────┴────────────────────────────────────────┘
```

**Key Benefits:**
- Test flight code without risk to hardware
- Debug with full access to internal state
- Automated testing (CI/CD pipelines)
- Develop without physical vehicle
- Simulate sensor failures and edge cases

### 1.2 High-Level Architecture

The SITL system consists of several layered components:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        ArduPilot Firmware                               │
│                (ArduCopter / ArduPlane / Rover / ArduSub)               │
├─────────────────────────────────────────────────────────────────────────┤
│                           AP_HAL Interface                              │
│    (Abstract interface: Scheduler, Serial, GPIO, Sensors, etc.)         │
├─────────────────────────────────────────────────────────────────────────┤
│                          AP_HAL_SITL                                    │
│                                                                         │
│  ┌─────────────┐ ┌─────────────┐ ┌────────────┐ ┌──────────────┐       │
│  │  Scheduler  │ │ UARTDriver  │ │   GPIO     │ │  AnalogIn    │       │
│  │ (time mgmt) │ │  (sockets)  │ │ (simulated)│ │  (voltages)  │       │
│  └─────────────┘ └─────────────┘ └────────────┘ └──────────────┘       │
│                                                                         │
├──────────────────────────┬──────────────────────────────────────────────┤
│       SITL_State         │              SITL::SIM                       │
│   (Coordination layer)   │        (Parameters & State)                  │
│                          │                                              │
│  - Manages simulation    │  - sitl_fdm struct (shared state)           │
│  - Creates aircraft      │  - Sensor parameters                        │
│  - Updates sensors       │  - Fault injection                          │
│  - Handles I/O           │  - Wind/environment simulation              │
├──────────────────────────┴──────────────────────────────────────────────┤
│                      Aircraft Physics Model                             │
│                                                                         │
│  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌────────────┐          │
│  │ MultiCopter│ │   Plane    │ │   Rover    │ │ Submarine  │   ...    │
│  └────────────┘ └────────────┘ └────────────┘ └────────────┘          │
│                                                                         │
│  - Aerodynamics / ground contact                                        │
│  - Motor/propeller models                                               │
│  - Position/velocity/attitude integration                               │
└─────────────────────────────────────────────────────────────────────────┘
```

### 1.3 The Simulation Loop

SITL runs a tight loop that:
1. Gets servo/motor outputs from the flight controller
2. Feeds them to the physics model
3. Updates the simulated vehicle state
4. Provides new sensor readings back to the flight controller

```
┌──────────────────────────────────────────────────────────────────┐
│                     SITL Main Loop (~1200 Hz)                    │
│                                                                  │
│   ┌─────────────────────────────────────────────────────────┐   │
│   │ 1. Read servo outputs from flight controller            │   │
│   │    (PWM values for motors/servos)                       │   │
│   └────────────────────────┬────────────────────────────────┘   │
│                            ↓                                     │
│   ┌─────────────────────────────────────────────────────────┐   │
│   │ 2. Pack into sitl_input struct                          │   │
│   │    { servos[16], wind_speed, wind_direction, ... }      │   │
│   └────────────────────────┬────────────────────────────────┘   │
│                            ↓                                     │
│   ┌─────────────────────────────────────────────────────────┐   │
│   │ 3. Aircraft::update(sitl_input)                         │   │
│   │    ├── update_wind()                                    │   │
│   │    ├── calculate_forces()                               │   │
│   │    ├── update_dynamics() (integrate acceleration)       │   │
│   │    ├── update_position() (lat/lon/alt from NED)         │   │
│   │    └── update_mag_field_bf()                            │   │
│   └────────────────────────┬────────────────────────────────┘   │
│                            ↓                                     │
│   ┌─────────────────────────────────────────────────────────┐   │
│   │ 4. Fill sitl_fdm from aircraft state                    │   │
│   │    (position, velocity, attitude, sensor data)          │   │
│   └────────────────────────┬────────────────────────────────┘   │
│                            ↓                                     │
│   ┌─────────────────────────────────────────────────────────┐   │
│   │ 5. Sensor drivers read from sitl_fdm                    │   │
│   │    (IMU, Barometer, GPS, Compass, Airspeed, etc.)       │   │
│   └────────────────────────┬────────────────────────────────┘   │
│                            ↓                                     │
│   ┌─────────────────────────────────────────────────────────┐   │
│   │ 6. Flight controller processes sensor data              │   │
│   │    (EKF, attitude control, navigation)                  │   │
│   └────────────────────────┬────────────────────────────────┘   │
│                            ↓                                     │
│   ┌─────────────────────────────────────────────────────────┐   │
│   │ 7. sync_frame_time() - match wall clock for speedup     │   │
│   └─────────────────────────────────────────────────────────┘   │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

**Timing and Speedup:**
- Default physics rate: **1200 Hz** (configurable via `SIM_LOOP_RATE_HZ`)
- Speedup factor (default 1x): Run faster than real-time with `--speedup N`
- The `sync_frame_time()` function manages wall-clock synchronization

### 1.4 How SITL Differs from Real Hardware

| Aspect | Real Hardware | SITL |
|--------|--------------|------|
| Sensors | Physical devices on I2C/SPI | Software models reading from `sitl_fdm` |
| Timing | Hardware timers/interrupts | Scheduler with time-stepping |
| Serial | Physical UART | TCP/UDP sockets |
| Storage | EEPROM/Flash | File on disk (`eeprom.bin`) |
| Motors | ESCs driving real motors | Values stored in array |
| Physics | Real world | Numerical integration |

---

## Part 2: Implementation Details

### 2.1 Starting SITL

**Entry Point: `Tools/autotest/sim_vehicle.py`**

This Python script handles:
1. Building the SITL binary (via WAF)
2. Launching the ArduPilot executable
3. Starting MAVProxy for control
4. Managing multi-vehicle instances

```bash
# Basic usage
cd ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --console --map

# With speedup
../Tools/autotest/sim_vehicle.py -v ArduCopter --speedup 10

# Multiple instances
../Tools/autotest/sim_vehicle.py -v ArduCopter -I 0 &
../Tools/autotest/sim_vehicle.py -v ArduCopter -I 1 &
```

**Build System:**
```bash
./waf configure --board sitl
./waf copter       # Build ArduCopter for SITL
```

**Command Line Arguments (parsed in `SITL_cmdline.cpp`):**

| Argument | Description |
|----------|-------------|
| `--model` | Vehicle model (quad, plane, rover, etc.) |
| `--speedup` | Simulation speedup factor |
| `--home` | Starting location (lat,lon,alt,yaw) |
| `--instance` or `-I` | Instance number (for multi-vehicle) |
| `--base-port` | Base port number (default 5670) |
| `--serial0` | Override SERIAL0 connection string |

### 2.2 The Main Loop Deep Dive

**File: `libraries/AP_HAL_SITL/HAL_SITL_Class.cpp`**

The main loop is in `HAL_SITL::run()` (line 223):

```cpp
void HAL_SITL::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    // Initialize HAL components
    _sitl_state->init(argc, argv);
    scheduler->init();
    serial(0)->begin(115200);
    rcin->init();
    rcout->init();
    analogin->init();

    // Setup complete - call vehicle's setup()
    callbacks->setup();
    scheduler->set_system_initialized();

    // Main simulation loop
    while (true) {
        if (HALSITL::Scheduler::_should_exit) {
            exit(0);
        }

        // Call vehicle's loop() - this runs the flight controller
        callbacks->loop();

        // Run I/O procedures (sensor updates, etc.)
        HALSITL::Scheduler::_run_io_procs();

        // Watchdog handling...
    }
}
```

### 2.3 Key Data Structures

#### The `sitl_fdm` Structure

**File: `libraries/SITL/SITL.h` (lines 59-108)**

This is the **central data structure** passed between the physics model and flight controller:

```cpp
struct sitl_fdm {
    // Timing
    uint64_t timestamp_us;

    // Position
    Location home;
    double latitude, longitude;     // degrees
    double altitude;                // MSL in meters

    // Velocity (NED frame, m/s)
    double speedN, speedE, speedD;

    // Attitude (degrees)
    double rollDeg, pitchDeg, yawDeg;
    double heading;
    Quaternion quaternion;

    // Angular rates (deg/s, body frame)
    double rollRate, pitchRate, yawRate;

    // Accelerations (m/s², body frame)
    double xAccel, yAccel, zAccel;
    Vector3f angAccel;              // Angular acceleration (deg/s²)

    // Airspeed
    double airspeed;                // m/s, EAS
    Vector3f velocity_air_bf;       // TAS, body frame

    // Power system
    double battery_voltage;         // Volts
    double battery_current;         // Amps
    double battery_remaining;       // Ah remaining

    // Motors
    uint8_t num_motors;
    uint32_t motor_mask;
    float rpm[32];                  // RPM of all motors

    // RC input simulation
    uint8_t rcin_chan_count;
    float rcin[12];                 // RC input 0..1

    // Sensors
    double range;                   // Rangefinder
    Vector3f bodyMagField;          // Magnetic field (milli-Gauss)
    float rangefinder_m[10];        // Multiple rangefinders
    float airspeed_raw_pressure[AIRSPEED_MAX_SENSORS];

    // Wind vane
    struct {
        float speed;
        float direction;
    } wind_vane_apparent;

    // Terrain
    float height_agl;               // Height above ground level

    // External physics coordination
    bool is_lock_step_scheduled;
    Vector3f wind_ef;               // Earth-frame wind
};
```

#### The `sitl_input` Structure

**File: `libraries/SITL/SITL_Input.h`**

Input from flight controller to physics model:

```cpp
struct sitl_input {
    uint16_t servos[SITL_NUM_CHANNELS];  // PWM values (typically 1000-2000)

    // Wind simulation
    struct {
        float speed;          // m/s
        float direction;      // degrees
        float turbulence;
        float dir_z;         // vertical component
    } wind;
};
```

#### The `SITL::SIM` Singleton

**File: `libraries/SITL/SITL.h` (line 113)**

Access the simulation state from anywhere:

```cpp
// Get the SITL singleton
SITL::SIM *sitl = AP::sitl();

// Or via the class method
SITL::SIM *sitl = SITL::SIM::get_singleton();

// Access the FDM state
struct sitl_fdm &state = sitl->state;

// Read current position
double lat = sitl->state.latitude;
double lon = sitl->state.longitude;
double alt = sitl->state.altitude;

// Read current attitude
double roll = sitl->state.rollDeg;
double pitch = sitl->state.pitchDeg;
double yaw = sitl->state.yawDeg;
```

### 2.4 The Scheduler System

**File: `libraries/AP_HAL_SITL/Scheduler.h`**

The SITL scheduler manages time progression and periodic callbacks:

```cpp
class Scheduler : public AP_HAL::Scheduler {
public:
    // Register callbacks (up to 8 each)
    void register_timer_process(AP_HAL::MemberProc);  // ~1000 Hz
    void register_io_process(AP_HAL::MemberProc);     // ~100 Hz

    // Time control
    void delay(uint16_t ms);
    void delay_microseconds(uint16_t us);

    // Atomic sections (prevent time stepping)
    void sitl_begin_atomic() { _nested_atomic_ctr++; }
    void sitl_end_atomic();

    // Check if interrupts are blocked
    bool interrupts_are_blocked() const { return _nested_atomic_ctr != 0; }

    // Run I/O procedures manually
    static void _run_io_procs();

private:
    static AP_HAL::MemberProc _timer_proc[8];  // High-frequency callbacks
    static AP_HAL::MemberProc _io_proc[8];     // Low-frequency callbacks
    static uint8_t _num_timer_procs;
    static uint8_t _num_io_procs;

    uint64_t _stopped_clock_usec;  // For deterministic replay
};
```

**Time Stepping:**

The scheduler coordinates with `SITL_State::wait_clock()` to advance simulation time:

```cpp
void Scheduler::delay_microseconds(uint16_t usec)
{
    uint64_t start = AP_HAL::micros64();
    do {
        uint64_t dtime = AP_HAL::micros64() - start;
        if (dtime >= usec) break;
        _sitlState->wait_clock(start + usec);
    } while (true);
}
```

### 2.5 Sensor Simulation Pipeline

Each sensor type has a SITL implementation that reads from `sitl_fdm`:

```
┌──────────────────────────────────────────────────────────────────┐
│                    Physics Engine (Aircraft)                     │
│                                                                  │
│  Calculates: position, velocity, attitude, angular rates         │
└────────────────────────────┬─────────────────────────────────────┘
                             │
                             ↓ fill_fdm()
┌──────────────────────────────────────────────────────────────────┐
│                      sitl_fdm struct                             │
│                                                                  │
│  Contains: lat/lon/alt, velocities, attitudes, accelerations     │
└──────┬───────────┬───────────┬───────────┬───────────┬──────────┘
       │           │           │           │           │
       ↓           ↓           ↓           ↓           ↓
┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐
│   IMU    │ │  Baro    │ │ Compass  │ │   GPS    │ │ Airspeed │
│  SITL    │ │  SITL    │ │  SITL    │ │  SITL    │ │  SITL    │
│          │ │          │ │          │ │          │ │          │
│ + noise  │ │ + noise  │ │ + noise  │ │ + delay  │ │ + noise  │
│ + bias   │ │ + drift  │ │ + motor  │ │ + glitch │ │ + offset │
│ + scale  │ │ + glitch │ │   interf.│ │          │ │          │
└────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘
     │            │            │            │            │
     ↓            ↓            ↓            ↓            ↓
┌──────────────────────────────────────────────────────────────────┐
│                    ArduPilot Sensor Backends                     │
│                                                                  │
│  AP_InertialSensor, AP_Baro, AP_Compass, AP_GPS, AP_Airspeed     │
└──────────────────────────────────────────────────────────────────┘
```

**Sensor Parameters (Examples):**

```cpp
// GPS parameters (per-instance)
AP_Float gps[i].noise;       // Position noise amplitude
AP_Int16 gps[i].lock_time;   // Seconds to acquire lock
AP_Int8  gps[i].numsats;     // Number of satellites
AP_Vector3f gps[i].glitch;   // Position glitch injection
AP_Int16 gps[i].delay_ms;    // Data delay

// Barometer parameters
AP_Float baro[i].noise;      // Pressure noise
AP_Float baro[i].drift;      // Drift rate
AP_Float baro[i].glitch;     // Glitch injection
AP_Int8  baro[i].freeze;     // Freeze at current altitude

// IMU parameters
AP_Float gyro_noise[i];      // Gyro noise (deg/s)
AP_Vector3f gyro_bias[i];    // Gyro bias (rad/s)
AP_Float accel_noise[i];     // Accel noise (m/s²)
AP_Vector3f accel_bias[i];   // Accel bias (m/s²)
```

### 2.6 Communication Architecture

**File: `libraries/AP_HAL_SITL/UARTDriver.cpp`**

SITL maps serial ports to network sockets:

```
┌────────────────────────────────────────────────────────────────┐
│                    ArduPilot Serial Ports                      │
├────────────────────────────────────────────────────────────────┤
│ SERIAL0 → tcp:5760:wait  (GCS connection - MAVProxy)           │
│ SERIAL1 → tcp:5762       (Secondary MAVLink)                   │
│ SERIAL2 → tcp:5763       (Telemetry)                           │
│ SERIAL3 → GPS1           (Simulated GPS)                       │
│ SERIAL4 → GPS2           (Second GPS)                          │
│ SERIAL5 → tcp:5765       (Custom)                              │
│ ...                                                            │
└────────────────────────────────────────────────────────────────┘

Port Numbering Formula:
  actual_port = base_port + offset + (instance * 10)

  base_port = 5760 (default)

  Instance 0: 5760, 5762, 5763, ...
  Instance 1: 5770, 5772, 5773, ...
  Instance 2: 5780, 5782, 5783, ...
```

**Connection String Formats:**

```
tcp:5760:wait          # TCP server, wait for connection
tcp:5760               # TCP server, don't wait
tcpclient:IP:port      # TCP client, connect to server
udpclient:IP:port      # UDP client
mcast:239.255.145.50:14550  # UDP multicast
uart:/dev/ttyUSB0:57600     # Real serial port (for hardware-in-loop)
```

---

## Part 3: Building Into SITL (UI Integration Guide)

### 3.1 Integration Points for a Debugger UI

There are several approaches for integrating a visualization/debugging UI:

#### Option A: In the Main Loop (Highest Access)

**File: `libraries/AP_HAL_SITL/HAL_SITL_Class.cpp` (line 287)**

```cpp
while (true) {
    callbacks->loop();
    HALSITL::Scheduler::_run_io_procs();

    // >>> INSERT YOUR VISUALIZATION UPDATE HERE <<<
    // You have access to everything at this point
}
```

**Pros:** Full access to all state every iteration
**Cons:** Blocks main loop, must be fast

#### Option B: Via Scheduler Callbacks (Recommended)

Register a periodic callback that receives control at defined intervals:

```cpp
// In your initialization code:
void MyDebugger::init() {
    hal.scheduler->register_io_process(
        FUNCTOR_BIND_MEMBER(&MyDebugger::update, void)
    );
}

// Called approximately every 10ms
void MyDebugger::update() {
    SITL::SIM *sitl = AP::sitl();

    // Read vehicle state
    double lat = sitl->state.latitude;
    double lon = sitl->state.longitude;
    double alt = sitl->state.altitude;

    double roll = sitl->state.rollDeg;
    double pitch = sitl->state.pitchDeg;
    double yaw = sitl->state.yawDeg;

    // Update your UI...
}
```

**Pros:** Non-blocking, predictable timing
**Cons:** Slightly delayed data (10ms)

#### Option C: In SITL_State (Physics Coordination)

**File: `libraries/AP_HAL_SITL/SITL_State.cpp`**

Modify `SITL_State::loop_hook()` which is called during simulation updates:

```cpp
void SITL_State::loop_hook(void) {
    // Called every physics iteration
    // Access to sitl_model (Aircraft*) and _sitl (SITL::SIM*)
}
```

### 3.2 Data You Can Access

From `SITL::SIM *sitl = AP::sitl()`:

```cpp
// === Position & Navigation ===
sitl->state.latitude         // degrees
sitl->state.longitude        // degrees
sitl->state.altitude         // meters MSL
sitl->state.height_agl       // meters above ground
sitl->state.home             // Location struct (home position)

// === Velocity ===
sitl->state.speedN           // m/s North
sitl->state.speedE           // m/s East
sitl->state.speedD           // m/s Down (positive = descending)
sitl->state.airspeed         // m/s equivalent airspeed

// === Attitude ===
sitl->state.rollDeg          // degrees
sitl->state.pitchDeg         // degrees
sitl->state.yawDeg           // degrees (heading)
sitl->state.quaternion       // Quaternion

// === Angular Rates ===
sitl->state.rollRate         // deg/s
sitl->state.pitchRate        // deg/s
sitl->state.yawRate          // deg/s

// === Accelerations ===
sitl->state.xAccel           // m/s² body X
sitl->state.yAccel           // m/s² body Y
sitl->state.zAccel           // m/s² body Z

// === Motors ===
sitl->state.num_motors       // Number of motors
sitl->state.motor_mask       // Bitmask of active motors
sitl->state.rpm[i]           // RPM of motor i

// === Power ===
sitl->state.battery_voltage  // Volts
sitl->state.battery_current  // Amps
sitl->state.battery_remaining // Ah

// === Environment ===
sitl->wind_speed_active      // Current wind speed m/s
sitl->wind_direction_active  // Current wind direction deg

// === Sensors ===
sitl->state.rangefinder_m[i] // Rangefinder readings
sitl->state.bodyMagField     // Magnetometer (milli-Gauss)
```

From `SITL_State_Common`:

```cpp
SITL_State_Common *state = ...;

// PWM outputs (what flight controller commands)
state->pwm_output[0..15]     // PWM values 1000-2000

// PWM inputs (RC receiver simulation)
state->pwm_input[0..15]      // PWM values 1000-2000

// Analog sensor voltages
state->sonar_pin_voltage     // Rangefinder analog
state->airspeed_pin_voltage[i]
state->voltage_pin_voltage   // Battery voltage sense
state->current_pin_voltage   // Battery current sense
```

### 3.3 Modifying SITL for Visualization

#### Adding a Visualization Module

Create a new visualization module:

```cpp
// libraries/SITL/SIM_Visualization.h
#pragma once

#include "SITL.h"

namespace SITL {

class Visualization {
public:
    void init();
    void update(const struct sitl_fdm &fdm, const uint16_t *servo_out);

private:
    // Your rendering state here
    uint64_t last_update_us;
    static constexpr uint32_t UPDATE_RATE_HZ = 30;
};

} // namespace SITL
```

#### Thread Safety Considerations

SITL is primarily single-threaded, but be careful with:

1. **Don't block the main loop** - Use non-blocking I/O
2. **If using separate render thread:**
   - Use mutexes for shared data
   - Copy data atomically in main thread, render in background
3. **Timing:**
   - Physics runs at 1200 Hz
   - Your visualization probably needs 30-60 Hz max

```cpp
// Example: Safe data copying for threaded rendering
class Visualizer {
    HAL_Semaphore data_sem;
    struct VisualizationFrame {
        double lat, lon, alt;
        double roll, pitch, yaw;
        uint64_t timestamp_us;
    } current_frame;

public:
    // Called from main SITL thread
    void update_from_fdm(const sitl_fdm &fdm) {
        WITH_SEMAPHORE(data_sem);
        current_frame.lat = fdm.latitude;
        current_frame.lon = fdm.longitude;
        current_frame.alt = fdm.altitude;
        current_frame.roll = fdm.rollDeg;
        current_frame.pitch = fdm.pitchDeg;
        current_frame.yaw = fdm.yawDeg;
        current_frame.timestamp_us = fdm.timestamp_us;
    }

    // Called from render thread
    VisualizationFrame get_frame() {
        WITH_SEMAPHORE(data_sem);
        return current_frame;
    }
};
```

### 3.4 Useful Patterns

#### Getting Access to Internal State

```cpp
#include <SITL/SITL.h>
#include <AP_HAL_SITL/SITL_State.h>

void my_debug_function() {
    // Get SITL singleton
    SITL::SIM *sitl = AP::sitl();
    if (sitl == nullptr) return;  // Not running in SITL

    // Access FDM state
    const struct sitl_fdm &fdm = sitl->state;

    // Current position
    printf("Position: %.6f, %.6f, %.1fm\n",
           fdm.latitude, fdm.longitude, fdm.altitude);

    // Current attitude
    printf("Attitude: R=%.1f P=%.1f Y=%.1f\n",
           fdm.rollDeg, fdm.pitchDeg, fdm.yawDeg);

    // Current velocities
    printf("Velocity: N=%.1f E=%.1f D=%.1f m/s\n",
           fdm.speedN, fdm.speedE, fdm.speedD);
}
```

#### Reading Parameters at Runtime

```cpp
// Get simulation speedup
float speedup = sitl->speedup.get();

// Get wind settings
float wind_speed = sitl->wind_speed.get();
float wind_dir = sitl->wind_direction.get();

// Get sensor parameters
float gps_noise = sitl->gps[0].noise.get();
int gps_numsats = sitl->gps[0].numsats.get();
```

#### Injecting Faults for Testing

```cpp
// Fail motor 0
sitl->engine_fail.set(1);  // Bitmask of failed motors

// Add GPS glitch
sitl->gps[0].glitch.set(Vector3f(10, 10, 5));  // lat, lon, alt offset

// Freeze barometer
sitl->baro[0].freeze.set(1);

// Fail RC input
sitl->rc_fail.set(1);

// Simulate compass failure
sitl->mag_fail[0].set(1);
```

#### Accessing the Aircraft Model

```cpp
// From SITL_State_Common (protected member)
SITL::Aircraft *aircraft = sitl_model;

// Get aircraft-specific data
Vector3f velocity = aircraft->get_velocity_ef();
Matrix3f dcm = aircraft->get_dcm();
Location loc = aircraft->get_location();
float hagl = aircraft->rangefinder_range();
```

---

## Part 4: Reference

### 4.1 Directory Structure

```
ardupilot/
├── libraries/
│   ├── SITL/                          # Physics models & simulation
│   │   ├── SITL.h                     # Core SIM class, sitl_fdm struct
│   │   ├── SITL.cpp                   # Parameter definitions
│   │   ├── SITL_Input.h               # sitl_input struct
│   │   ├── SIM_Aircraft.h             # Base aircraft class
│   │   ├── SIM_Aircraft.cpp           # Physics integration
│   │   ├── SIM_Multicopter.h/cpp      # Multicopter physics
│   │   ├── SIM_Plane.h/cpp            # Fixed-wing physics
│   │   ├── SIM_Rover.h/cpp            # Ground vehicle physics
│   │   ├── SIM_Submarine.h/cpp        # Underwater vehicle
│   │   ├── SIM_Helicopter.h/cpp       # Helicopter physics
│   │   ├── SIM_Frame.h/cpp            # Motor frame configuration
│   │   ├── SIM_Motor.h/cpp            # Motor model
│   │   ├── SIM_Battery.h/cpp          # Battery simulation
│   │   ├── SIM_GPS.h/cpp              # GPS simulation
│   │   ├── SIM_JSON.h/cpp             # External physics via JSON
│   │   ├── SIM_Gazebo.h/cpp           # Gazebo integration
│   │   └── ...                        # Many more sensor/device sims
│   │
│   └── AP_HAL_SITL/                   # HAL implementation for SITL
│       ├── HAL_SITL_Class.h/cpp       # Main HAL class, run loop
│       ├── Scheduler.h/cpp            # Time management
│       ├── SITL_State.h/cpp           # Main simulation state
│       ├── SITL_State_common.h/cpp    # Shared state & serial devices
│       ├── SITL_cmdline.cpp           # Command line parsing
│       ├── UARTDriver.h/cpp           # Serial over TCP/UDP
│       ├── AnalogIn.h/cpp             # Analog inputs (voltages)
│       ├── GPIO.h/cpp                 # Digital I/O
│       ├── RCInput.h/cpp              # RC receiver simulation
│       ├── RCOutput.h/cpp             # Servo/motor outputs
│       ├── Storage.h/cpp              # Parameter storage (file)
│       ├── I2CDevice.h/cpp            # I2C bus simulation
│       └── SPIDevice.h/cpp            # SPI bus simulation
│
└── Tools/
    └── autotest/
        ├── sim_vehicle.py             # Main launch script
        ├── pysim/                     # Python simulation helpers
        └── locations.txt              # Predefined locations
```

### 4.2 Key Files Quick Reference

| File | Purpose | Key Contents |
|------|---------|--------------|
| `SITL/SITL.h` | Core simulation | `sitl_fdm`, `SITL::SIM` class |
| `SITL/SIM_Aircraft.h` | Physics base | `Aircraft` class, `update()` |
| `AP_HAL_SITL/HAL_SITL_Class.cpp` | Main loop | `HAL_SITL::run()` |
| `AP_HAL_SITL/Scheduler.h` | Time control | Timer/IO callbacks |
| `AP_HAL_SITL/SITL_State.h` | Coordination | `SITL_State` class |
| `AP_HAL_SITL/SITL_State_common.h` | Shared state | PWM I/O, voltages |

### 4.3 Common Parameters

All parameters prefixed with `SIM_` control simulation behavior:

**General:**
- `SIM_SPEEDUP` - Simulation speedup factor
- `SIM_LOOP_RATE_HZ` - Physics update rate (default 1200)

**GPS:**
- `SIM_GPS_TYPE` - GPS type (0=UBLOX, 1=MTK, etc.)
- `SIM_GPS_NUMSATS` - Number of satellites
- `SIM_GPS_NOISE` - Position noise
- `SIM_GPS_GLITCH` - Position glitch vector
- `SIM_GPS_DELAY_MS` - Data latency

**Barometer:**
- `SIM_BARO_NOISE` - Pressure noise
- `SIM_BARO_DRIFT` - Altitude drift
- `SIM_BARO_FREEZE` - Freeze at current altitude

**Compass:**
- `SIM_MAG_NOISE` - Magnetic noise
- `SIM_MAG_MOT` - Motor interference
- `SIM_MAG_FAIL` - Compass failure mode

**IMU:**
- `SIM_GYRO_NOISE` - Gyro noise (deg/s)
- `SIM_ACCEL_NOISE` - Accel noise (m/s²)
- `SIM_GYR_BIAS_X/Y/Z` - Gyro bias
- `SIM_ACC_BIAS_X/Y/Z` - Accel bias

**Wind:**
- `SIM_WIND_SPD` - Wind speed (m/s)
- `SIM_WIND_DIR` - Wind direction (degrees)
- `SIM_WIND_TURB` - Turbulence intensity

**Failures:**
- `SIM_ENGINE_FAIL` - Motor failure mask
- `SIM_RC_FAIL` - RC failure mode
- `SIM_BATT_VOLTAGE` - Battery voltage

### 4.4 External Simulator Integration

SITL can connect to external physics engines via JSON protocol:

```bash
# Launch with external physics
sim_vehicle.py -v ArduCopter -f JSON --sim-address=127.0.0.1
```

**JSON Protocol (UDP port 9002):**

SITL sends servo commands:
```json
{
    "magic": 18458,
    "frame_rate": 1200,
    "frame_count": 12345,
    "pwm": [1500, 1500, 1000, 1500, ...]
}
```

External simulator responds with state:
```json
{
    "timestamp": 1.234,
    "imu": {
        "gyro": [0.0, 0.0, 0.0],
        "accel_body": [0.0, 0.0, -9.81]
    },
    "position": [0.0, 0.0, 0.0],
    "velocity": [0.0, 0.0, 0.0],
    "attitude": [0.0, 0.0, 0.0],
    "quaternion": [1.0, 0.0, 0.0, 0.0]
}
```

---

## Summary

SITL enables ArduPilot to run realistic simulations by:

1. **Using the same code** - The flight controller code is identical to what runs on hardware
2. **Abstracting hardware** - AP_HAL_SITL provides software implementations of all hardware interfaces
3. **Simulating physics** - Aircraft models calculate realistic dynamics
4. **Providing access** - The `sitl_fdm` structure exposes all internal state

For building a debugger/visualization UI:
- Access `SITL::SIM::get_singleton()->state` for complete vehicle state
- Register callbacks via `hal.scheduler->register_io_process()` for periodic updates
- Tap into the main loop in `HAL_SITL_Class.cpp` for maximum access
- Use thread-safe patterns if rendering in a separate thread

---

## Appendix A: Glossary of Acronyms and Terms

### Acronyms

| Acronym | Full Name | Description |
|---------|-----------|-------------|
| **ADSB** | Automatic Dependent Surveillance-Broadcast | Aircraft tracking system where aircraft broadcast their position |
| **AGL** | Above Ground Level | Altitude measured from the ground directly below |
| **AHRS** | Attitude and Heading Reference System | System that provides 3D orientation (roll, pitch, yaw) |
| **API** | Application Programming Interface | Set of functions/protocols for building software |
| **Baro** | Barometer | Pressure sensor used to estimate altitude |
| **CAN** | Controller Area Network | Serial bus standard for vehicle electronics |
| **CI/CD** | Continuous Integration/Continuous Deployment | Automated testing and deployment pipeline |
| **DCM** | Direction Cosine Matrix | 3x3 rotation matrix representing orientation |
| **DMA** | Direct Memory Access | Hardware feature for memory transfers without CPU |
| **EAS** | Equivalent Airspeed | Airspeed corrected for air density to sea level |
| **EEPROM** | Electrically Erasable Programmable Read-Only Memory | Non-volatile storage for parameters |
| **EFI** | Electronic Fuel Injection | Computer-controlled fuel delivery system |
| **EKF** | Extended Kalman Filter | Algorithm that fuses sensor data to estimate state |
| **ESC** | Electronic Speed Controller | Device that controls motor speed from PWM signal |
| **FDM** | Flight Dynamics Model | Mathematical model of aircraft physics |
| **FPU** | Floating Point Unit | Hardware for floating-point math |
| **GCS** | Ground Control Station | Software for monitoring/controlling the vehicle (e.g., Mission Planner, QGroundControl) |
| **GPIO** | General Purpose Input/Output | Digital pins that can be read or written |
| **GPS** | Global Positioning System | Satellite navigation system for position |
| **HAL** | Hardware Abstraction Layer | Software layer that provides uniform interface to hardware |
| **HAGL** | Height Above Ground Level | Same as AGL |
| **HIL** | Hardware In the Loop | Simulation where real autopilot connects to simulated sensors |
| **I2C** | Inter-Integrated Circuit | Two-wire serial bus for sensors/peripherals |
| **IMU** | Inertial Measurement Unit | Combined accelerometer + gyroscope sensor package |
| **JSON** | JavaScript Object Notation | Text-based data format |
| **MAVLink** | Micro Air Vehicle Link | Communication protocol for drones |
| **MSL** | Mean Sea Level | Altitude reference (0 = average sea level) |
| **NED** | North-East-Down | Coordinate frame where X=North, Y=East, Z=Down |
| **OSD** | On-Screen Display | Overlay of flight data on video feed |
| **PWM** | Pulse Width Modulation | Signal encoding (typically 1000-2000μs for servos) |
| **RC** | Radio Control | Remote control system (transmitter + receiver) |
| **RPM** | Revolutions Per Minute | Rotational speed of motors/propellers |
| **RTK** | Real-Time Kinematic | High-precision GPS using correction data |
| **SITL** | Software In The Loop | Simulation running on PC without hardware |
| **SOAP** | Simple Object Access Protocol | XML-based messaging protocol |
| **SPI** | Serial Peripheral Interface | High-speed serial bus for sensors |
| **TAS** | True Airspeed | Actual speed through the air (vs EAS) |
| **TCP** | Transmission Control Protocol | Reliable network protocol |
| **UART** | Universal Asynchronous Receiver-Transmitter | Serial communication hardware |
| **UDP** | User Datagram Protocol | Fast, connectionless network protocol |
| **WAF** | Build tool | ArduPilot's build system (Python-based) |

### Terms of Art

| Term | Description |
|------|-------------|
| **Accelerometer** | Sensor measuring linear acceleration (m/s²) in 3 axes |
| **Accel** | Short for accelerometer |
| **Airspeed** | Speed of aircraft relative to surrounding air |
| **Attitude** | Orientation of vehicle in 3D space (roll, pitch, yaw angles) |
| **Body Frame** | Coordinate system fixed to the vehicle (X=forward, Y=right, Z=down) |
| **Callback** | Function passed to be called later by the system |
| **Compass** | Magnetometer measuring Earth's magnetic field for heading |
| **Earth Frame** | Coordinate system fixed to Earth (typically NED) |
| **Euler Angles** | Roll, pitch, yaw representation of orientation |
| **Frame** | (1) Coordinate system, or (2) physical structure/motor layout of multicopter |
| **Glitch** | Sudden erroneous sensor reading (can be injected for testing) |
| **Ground Effect** | Increased lift when flying close to ground |
| **Gyro/Gyroscope** | Sensor measuring angular velocity (deg/s or rad/s) |
| **Heading** | Direction the vehicle nose points (degrees from North) |
| **Lock Step** | Simulation mode where physics waits for controller |
| **Magnetometer** | Sensor measuring magnetic field strength and direction |
| **Mutex** | Mutual exclusion lock for thread synchronization |
| **Noise** | Random variation in sensor readings |
| **Pitch** | Rotation about the Y (lateral) axis - nose up/down |
| **Quaternion** | 4-component representation of 3D rotation (avoids gimbal lock) |
| **Rangefinder** | Sensor measuring distance to ground/obstacles (sonar, lidar, etc.) |
| **Roll** | Rotation about the X (longitudinal) axis - banking left/right |
| **Semaphore** | Synchronization primitive for thread coordination |
| **Servo** | Actuator that moves to commanded position (for control surfaces) |
| **Singleton** | Design pattern ensuring only one instance of a class exists |
| **Speedup** | Running simulation faster than real-time (e.g., 10x) |
| **Throttle** | Power level commanded to motors (0-100% or PWM value) |
| **Thrust** | Force produced by motors/propellers |
| **Time Stepping** | Advancing simulation time in discrete increments |
| **Turbulence** | Random wind variations |
| **Yaw** | Rotation about the Z (vertical) axis - turning left/right |

### Coordinate Frames

**Body Frame (Vehicle-Fixed):**
```
       X (Forward/Roll axis)
       ↑
       │
       │
Y ←────┼──── (Right/Pitch axis)
       │
       │
       ↓
       Z (Down/Yaw axis)
```

**NED Frame (Earth-Fixed):**
```
       N (North)
       ↑
       │
       │
E ←────┼──── (East)
       │
       │
       ↓
       D (Down - into Earth)
```

### Attitude Angles

```
Roll (φ):   Rotation about X-axis (longitudinal)
            Positive = right wing down

Pitch (θ):  Rotation about Y-axis (lateral)
            Positive = nose up

Yaw (ψ):    Rotation about Z-axis (vertical)
            Positive = nose right (clockwise from above)
```

### PWM Signal Conventions

```
Standard RC PWM range: 1000μs - 2000μs

Throttle (typically):
  1000μs = 0% (idle/off)
  2000μs = 100% (full power)

Servo/Control (typically):
  1000μs = Full deflection one direction
  1500μs = Center/neutral
  2000μs = Full deflection other direction

ESC Calibration:
  Motors won't spin until armed and throttle > ~1100μs
```

### Sensor Typical Values

| Sensor | Typical Output | Units |
|--------|---------------|-------|
| Accelerometer | ±16g max, ~9.81 m/s² at rest (Z-axis) | m/s² |
| Gyroscope | ±2000 deg/s max, ~0 at rest | deg/s or rad/s |
| Barometer | ~101325 Pa at sea level | Pascals (Pa) |
| Magnetometer | ~200-600 mGauss (varies by location) | milli-Gauss |
| GPS | ±2-5m accuracy (standard), ±0.01m (RTK) | meters |
| Airspeed | 0-100+ m/s | m/s |
| Rangefinder | 0.1-40m typical | meters |
