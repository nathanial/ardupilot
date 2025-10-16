# ArduPlane Architecture Analysis

## Table of Contents
1. [Overview](#overview)
2. [Folder Structure](#folder-structure)
3. [Core Architecture](#core-architecture)
4. [Flight Modes](#flight-modes)
5. [Control Systems](#control-systems)
6. [Execution Loop](#execution-loop)
7. [Important Files](#important-files)
8. [Build System](#build-system)
9. [Debugging](#debugging)
10. [Interesting Properties](#interesting-properties)

---

## Overview

ArduPlane is the fixed-wing aircraft autopilot component of the ArduPilot project. It's a sophisticated, real-time flight control system written in C++ that provides autonomous flight capabilities for fixed-wing aircraft, including support for VTOL (Vertical Take-Off and Landing) quadplanes.

**Key Features:**
- 26+ flight modes (including fixed-wing and VTOL modes)
- Advanced PID-based attitude control
- Total Energy Control System (TECS) for altitude/speed management
- L1 navigation controller for waypoint following
- Extended Kalman Filter (EKF) for sensor fusion
- MAVLink telemetry and GCS integration
- Comprehensive logging and debugging capabilities
- SITL (Software In The Loop) simulation support

**License:** GNU GPL v3+

**Lead Developers:** Andrew Tridgell, Tom Pittenger

---

## Folder Structure

The ArduPlane directory contains approximately 90 files organized as follows:

### Core System Files
```
Plane.h              - Main class definition (1333 lines)
Plane.cpp            - Main execution loop and scheduler (1089 lines)
Parameters.h/.cpp    - Parameter definitions and storage (18KB/85KB)
defines.h            - Constants, enums, and macros
config.h             - Compile-time configuration
version.h            - Version information
```

### Flight Modes (mode_*.cpp/h)
```
mode.h               - Base Mode class and all mode definitions (1040 lines)
mode.cpp             - Common mode functionality
mode_manual.cpp      - Direct RC passthrough
mode_stabilize.cpp   - Attitude stabilization
mode_fbwa.cpp        - Fly-By-Wire A (altitude hold)
mode_fbwb.cpp        - Fly-By-Wire B (speed/altitude hold)
mode_cruise.cpp      - Cruise mode with heading lock
mode_training.cpp    - Training mode
mode_acro.cpp        - Aerobatic mode
mode_autotune.cpp    - Automatic PID tuning
mode_auto.cpp        - Autonomous mission following
mode_rtl.cpp         - Return to launch
mode_loiter.cpp      - Loiter at location
mode_circle.cpp      - Circle waypoint
mode_guided.cpp      - External control (GCS/companion)
mode_takeoff.cpp     - Autonomous takeoff
mode_thermal.cpp     - Thermal soaring
mode_autoland.cpp    - Autonomous landing
mode_avoidADSB.cpp   - ADSB collision avoidance
```

### VTOL/QuadPlane Support
```
quadplane.h/.cpp     - QuadPlane VTOL implementation (21KB/189KB)
mode_qstabilize.cpp  - VTOL stabilize mode
mode_qhover.cpp      - VTOL hover mode
mode_qloiter.cpp     - VTOL loiter mode
mode_qland.cpp       - VTOL land mode
mode_qrtl.cpp        - VTOL return to launch
mode_qacro.cpp       - VTOL acrobatic mode
mode_qautotune.cpp   - VTOL autotuning
mode_LoiterAltQLand.cpp - Loiter to altitude then VTOL land
tailsitter.h/.cpp    - Tailsitter VTOL configuration
tiltrotor.h/.cpp     - Tiltrotor VTOL configuration
transition.h         - VTOL transition state management
VTOL_Assist.h/.cpp   - VTOL-assisted fixed-wing flight
```

### Control Systems
```
Attitude.cpp         - Attitude control (roll/pitch/yaw) (27KB)
navigation.cpp       - Navigation algorithms (19KB)
altitude.cpp         - Altitude control and TECS (32KB)
servos.cpp           - Servo output and mixing (45KB)
radio.cpp            - RC input processing (13KB)
control_modes.cpp    - Mode switching logic
```

### Mission & Commands
```
commands.cpp         - Mission command handling
commands_logic.cpp   - Mission command execution logic (47KB)
takeoff.cpp          - Takeoff sequence management (18KB)
```

### Communication & Telemetry
```
GCS_Plane.h/.cpp           - GCS interface
GCS_MAVLink_Plane.h/.cpp   - MAVLink protocol implementation (47KB)
RC_Channel_Plane.h/.cpp    - RC channel mapping
```

### Safety & Monitoring
```
failsafe.cpp         - Failsafe logic
fence.cpp            - Geofence implementation
ekf_check.cpp        - EKF health monitoring
is_flying.cpp        - Flight detection (13KB)
parachute.cpp        - Parachute deployment
AP_Arming_Plane.h/.cpp - Arming checks
crash_detection.cpp  - (integrated in is_flying.cpp)
```

### Logging & Debugging
```
Log.cpp              - Data logging (22KB)
events.cpp           - Event logging (12KB)
```

### Sensors & State Estimation
```
sensors.cpp          - Sensor reading
system.cpp           - System initialization (14KB)
```

### Advanced Features
```
soaring.cpp          - Thermal soaring
avoidance_adsb.h/.cpp - ADSB avoidance
tuning.h/.cpp        - In-flight tuning
systemid.h/.cpp      - System identification
pullup.h/.cpp        - Glider pullup maneuver
reverse_thrust.cpp   - Reverse thrust control
motor_test.cpp       - Motor testing
afs_plane.h/.cpp     - Advanced Failsafe System (AFS)
```

### Build System
```
wscript              - WAF build configuration
Makefile.waf         - Make wrapper
createTags           - Tag generation script
```

### Documentation
```
ReleaseNotes.txt     - Version history and changes (277KB)
```

---

## Core Architecture

### Main Class: `Plane`

The `Plane` class (defined in `Plane.h:131-1332`) inherits from `AP_Vehicle` and serves as the central orchestrator for the entire system.

#### Key Components

**1. Controllers (Plane.h:245-252)**
```cpp
AP_TECS TECS_controller;              // Total Energy Control System
AP_L1_Control L1_controller;          // L1 navigation controller
AP_RollController rollController;     // Roll stabilization
AP_PitchController pitchController;   // Pitch stabilization
AP_YawController yawController;       // Yaw/rudder control
AP_SteerController steerController;   // Ground steering
```

**2. State Variables (Plane.h:302-346)**
```cpp
Mode *control_mode;                   // Current flight mode
Mode *previous_mode;                  // Previous flight mode
int32_t nav_roll_cd;                  // Desired roll (centidegrees)
int32_t nav_pitch_cd;                 // Desired pitch (centidegrees)
float aerodynamic_load_factor;        // Load factor (1/cos(roll))
float smoothed_airspeed;              // Filtered airspeed
int32_t target_airspeed_cm;           // Target airspeed (cm/s)
```

**3. Flight Mode Instances (Plane.h:302-337)**
All flight modes are instantiated as member variables:
```cpp
ModeManual mode_manual;
ModeStabilize mode_stabilize;
ModeAuto mode_auto;
ModeRTL mode_rtl;
// ... 20+ more modes
```

**4. Subsystems**
- `AP_AHRS ahrs` - Attitude and Heading Reference System
- `AP_GPS gps` - GPS interface
- `AP_InertialSensor ins` - IMU interface
- `AP_Compass compass` - Magnetometer interface
- `AP_Airspeed airspeed` - Airspeed sensor
- `AP_BattMonitor battery` - Battery monitoring
- `AP_Mission mission` - Mission management
- `AP_Logger logger` - Data logging
- `GCS_Plane _gcs` - Ground Control Station interface
- `QuadPlane quadplane` - VTOL support (if enabled)

### Design Patterns

**1. Mode Pattern**
Flight modes use inheritance with a common `Mode` base class:
```cpp
class Mode {
public:
    virtual void update() = 0;        // Update logic
    virtual void navigate() {}        // Navigation logic
    virtual void run();               // Control loop
    virtual bool _enter() { return true; }
    virtual void _exit() {}
};
```

**2. Scheduler Pattern**
Priority-based task scheduling with guaranteed execution times:
```cpp
FAST_TASK(ahrs_update)              // Runs every loop
SCHED_TASK(read_radio, 50, 100, 6)  // 50Hz, 100us budget, priority 6
```

**3. Parameter System**
Hierarchical parameter management with persistent storage:
```cpp
AP_Param::GroupInfo var_info[] = {
    AP_GROUPINFO("ROLL_LIMIT", 1, Parameters, roll_limit, 45),
    // ...
};
```

**4. Callback/Functor Pattern**
Used extensively for decoupling subsystems:
```cpp
AP_Mission mission{
    FUNCTOR_BIND_MEMBER(&Plane::start_command_callback, ...),
    FUNCTOR_BIND_MEMBER(&Plane::verify_command_callback, ...)
};
```

---

## Flight Modes

ArduPlane supports 26+ flight modes. This section explains each mode based on the actual source code implementation.

### Mode Hierarchy

The mode system is implemented using class inheritance:

```
Mode (base class, mode.h:29-206)
 ├── ModeManual
 ├── ModeStabilize
 ├── ModeTraining
 ├── ModeAcro
 ├── ModeFBWA
 ├── ModeFBWB
 ├── ModeCruise
 ├── ModeCircle
 ├── ModeLoiter
 │    └── ModeLoiterAltQLand
 ├── ModeRTL
 ├── ModeAuto
 ├── ModeAutoTune
 ├── ModeGuided
 ├── ModeTakeoff
 ├── ModeAutoLand
 ├── ModeThermal
 ├── ModeAvoidADSB
 ├── ModeInitializing
 └── QuadPlane Modes (VTOL)
      ├── ModeQStabilize
      ├── ModeQHover
      ├── ModeQLoiter
      ├── ModeQLand
      ├── ModeQRTL
      ├── ModeQAcro
      └── ModeQAutotune
```

Each mode must implement:
- `mode_number()` - Unique mode identifier
- `name()` / `name4()` - Human-readable names
- `update()` - Called every loop to update mode logic
- `_enter()` / `_exit()` - Mode transition handlers (optional)
- `navigate()` - Navigation calculations (optional)
- `run()` - Control loop execution (optional)

---

### Manual Control Modes

#### **MANUAL (mode 0)** - `mode_manual.cpp:4-15`
**Pure passthrough - no stabilization whatsoever.**

**How it works:**
```cpp
void ModeManual::update() {
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.roll_in_expo(false));
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitch_in_expo(false));
    output_rudder_and_steering(plane.rudder_in_expo(false));
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.get_throttle_input(true));
}
```

- **Aileron:** Direct RC input → servo with expo curve
- **Elevator:** Direct RC input → servo with expo curve
- **Rudder:** Direct RC input → servo with expo curve
- **Throttle:** Direct passthrough
- **Controllers:** Reset every loop - no stabilization active
- **Use case:** Experienced pilots, 3D aerobatics, testing control linkages

**Key point:** You are flying the servos directly. The aircraft will not level itself.

---

#### **STABILIZE (mode 2)** - `mode_stabilize.cpp:4-18`
**Attitude stabilization with manual throttle.**

**How it works:**
```cpp
void ModeStabilize::update() {
    plane.nav_roll_cd = 0;      // Target: wings level
    plane.nav_pitch_cd = 0;     // Target: level pitch
}

void ModeStabilize::run() {
    plane.stabilize_roll();             // PID to hold level roll
    plane.stabilize_pitch();            // PID to hold level pitch
    stabilize_stick_mixing_direct();    // Blend stick input
    plane.stabilize_yaw();              // Coordinated turns
    output_pilot_throttle();            // Manual throttle
}
```

- **Target attitude:** Wings level (0° roll, 0° pitch)
- **Stick mixing:** Blends RC input with stabilization
- **Behavior:** Release sticks = aircraft levels out automatically
- **Throttle:** Pilot controlled
- **Use case:** Most common beginner mode, safe recovery from upsets

---

#### **TRAINING (mode 3)** - `mode_training.cpp:4-71`
**Intelligent stabilization that only engages when approaching limits.**

**How it works:**
```cpp
void ModeTraining::update() {
    // Check roll limits
    if (ahrs.roll_sensor >= plane.roll_limit_cd) {
        plane.nav_roll_cd = plane.roll_limit_cd;  // Hit limit - stabilize
    } else if (ahrs.roll_sensor <= -plane.roll_limit_cd) {
        plane.nav_roll_cd = -plane.roll_limit_cd;
    } else {
        plane.training_manual_roll = true;  // Within limits - manual control
        plane.nav_roll_cd = 0;
    }

    // Same logic for pitch limits...
}
```

**Behavior:**
- **Within limits:** Direct stick passthrough (like MANUAL)
- **At limits:** Automatic stabilization prevents exceeding configured angles
- **Pilot override:** Can push back from limits
- **Rudder:** Always manual
- **Anti-stall:** Prevents dangerous attitudes while learning
- **Use case:** Teaching new pilots, prevents accidents

---

#### **ACRO (mode 4)** - `mode_acro.cpp:27-227`
**Rate-based aerobatic mode with optional locking.**

**Two implementations:**

**1. Standard Acro** (line 46-127):
```cpp
// Stick input commands angular rates (deg/s)
float roll_rate = (rexpo/SERVO_MAX) * plane.g.acro_roll_rate;
float pitch_rate = (pexpo/SERVO_MAX) * plane.g.acro_pitch_rate;

// Rate control - stick commands how fast to roll/pitch
SRV_Channels::set_output_scaled(SRV_Channel::k_aileron,
    plane.rollController.get_rate_out(roll_rate, speed_scaler));
```

**With ACRO_LOCKING=1:**
- **Zero stick input:** Locks current angle, tries to hold it
- **Roll locking:** Integrates gyro error to eliminate drift
- **Pitch locking:** Remembers pitch angle when stick released

**2. Quaternion Acro** (`ACRO_LOCKING=2`, line 132-227):
```cpp
// 3D continuous locking - no gimbal lock
Quaternion q;  // Target attitude
q.rotate_fast(r);  // Rotate by rate demand
// Computes attitude error as quaternion difference
```

**Advantages:**
- **Rate control:** Stick position = rotation speed (not angle)
- **Knife-edge capable:** Works in any orientation
- **Quaternion mode:** Prevents gimbal lock at 90° pitch
- **Use case:** Aerobatic pilots, 3D flying, inverted flight

---

### Fly-By-Wire Modes

#### **FLY_BY_WIRE_A (FBWA, mode 5)** - `mode_fbwa.cpp:4-45`
**Stabilized flight with pitch/roll stick control.**

**How it works:**
```cpp
void ModeFBWA::update() {
    // Stick input directly commands bank angle
    plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;

    // Stick input commands pitch angle
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max*100;
    } else {
        plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min*100);
    }
}
```

**Features:**
- **Roll:** Stick → desired bank angle (full stick = max bank)
- **Pitch:** Stick → desired pitch angle (with limits)
- **Throttle:** Manual pilot control
- **Load factor:** Prevents stall in turns
- **Failsafe:** Levels wings and cuts throttle if RC lost
- **Taildragger mode:** Special takeoff handling
- **Use case:** Sport flying, FPV racing, manual control with safety

---

#### **FLY_BY_WIRE_B (FBWB, mode 6)** - `mode_fbwb.cpp:4-23`
**Altitude and airspeed hold.**

**How it works:**
```cpp
bool ModeFBWB::_enter() {
    plane.set_target_altitude_current();  // Lock current altitude
    return true;
}

void ModeFBWB::update() {
    plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_fbwb_speed_height();  // TECS altitude/speed control
}
```

**Key behavior:**
- **Entry:** Captures current altitude as target
- **Elevator stick:** Changes altitude *target* (not pitch directly)
  - Stick forward = descend
  - Stick back = climb
  - Stick centered = hold altitude
- **Throttle stick:** Changes airspeed *target*
- **Roll:** Manual bank control
- **TECS:** Manages pitch/throttle to achieve altitude/speed targets
- **Use case:** Hands-off cruising, photography, relaxed flying

---

#### **CRUISE (mode 7)** - `mode_cruise.cpp:4-97`
**FBWB with GPS heading lock.**

**How it works:**
```cpp
void ModeCruise::navigate() {
    // Lock trigger conditions
    if (!locked_heading &&
        plane.channel_roll->get_control_in() == 0 &&  // Aileron centered
        plane.rudder_input() == 0 &&                   // Rudder centered
        plane.gps.ground_speed() >= GPS_GND_CRS_MIN_SPD &&  // Moving (5 m/s)
        lock_timer_ms != 0 &&
        (millis() - lock_timer_ms) > 500) {  // 0.5 second delay

        locked_heading = true;
        locked_heading_cd = ground_course_cd;  // Lock GPS course

        // Create virtual waypoint 1km ahead on locked heading
        plane.next_WP_loc.offset_bearing(locked_heading_cd*0.01f,
            plane.prev_WP_loc.get_distance(plane.current_loc) + 1000);
    }
}
```

**Behavior:**
- **Same as FBWB** for altitude/speed
- **Heading lock:** After 0.5s with sticks centered at >5 m/s
- **Locked:** L1 controller tracks GPS course as waypoint
- **Unlock:** Any aileron or rudder input
- **Use case:** Long straight legs, minimal pilot input, return trips

---

### Autonomous Modes

#### **AUTO (mode 10)** - `mode_auto.cpp:4-202`
**Executes waypoint missions.**

**How it works:**
```cpp
bool ModeAuto::_enter() {
    plane.mission.start_or_resume();  // Start/resume mission
    return true;
}

void ModeAuto::update() {
    uint16_t nav_cmd_id = plane.mission.get_current_nav_cmd().id;

    if (nav_cmd_id == MAV_CMD_NAV_TAKEOFF) {
        plane.takeoff_calc_roll();
        plane.takeoff_calc_pitch();
        plane.takeoff_calc_throttle();
    } else if (nav_cmd_id == MAV_CMD_NAV_LAND) {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        // Throttle suppression when landing complete
    } else {
        // Normal navigation
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
}

void ModeAuto::navigate() {
    plane.mission.update();  // Advances through waypoints
}
```

**Features:**
- **Mission execution:** Follows waypoint list
- **Special commands:** Takeoff, landing, loiter, altitude wait
- **VTOL support:** Seamlessly handles QHOVER/QLAND commands
- **Scripting:** Supports NAV_SCRIPT_TIME for custom maneuvers
- **Watchdog recovery:** Resumes from crash location if rebooted
- **Use case:** Autonomous missions, mapping, surveys, delivery

---

#### **RTL - Return To Launch (mode 11)** - `mode_rtl.cpp:4-169`
**Returns to home or rally point.**

**How it works:**
```cpp
bool ModeRTL::_enter() {
    plane.do_RTL(plane.get_RTL_altitude_cm());  // Set home as target
    plane.rtl.done_climb = false;

    // QuadPlane: may immediately switch to QRTL if already close
    if (plane.quadplane.rtl_mode == QuadPlane::RTL_MODE::QRTL_ALWAYS) {
        plane.set_mode(plane.mode_qrtl, ModeReason::QRTL_INSTEAD_OF_RTL);
    }
}

void ModeRTL::update() {
    // Climb-before-turn logic
    if (plane.g2.rtl_climb_min > 0) {
        if (!plane.rtl.done_climb) {
            // Limit bank angle during climb
            plane.roll_limit_cd = MIN(plane.roll_limit_cd,
                plane.g.level_roll_limit*100);
        }
    }
}

void ModeRTL::navigate() {
    // Loiter at home
    plane.update_loiter(radius);

    // Auto-land check
    if (plane.g.rtl_autoland && reached_target) {
        // Jump to landing sequence in mission
        plane.mission.jump_to_landing_sequence(plane.current_loc);
        plane.set_mode(plane.mode_auto, ModeReason::RTL_COMPLETE);
    }
}
```

**Features:**
- **Climb first:** `RTL_CLIMB_MIN` climbs before turning (anti-stall)
- **Rally points:** Can target nearest rally point instead of home
- **Auto-land:** Can automatically start landing sequence
- **QRTL switch:** Transitions to VTOL landing when within radius
- **Loiter:** Circles at home if no landing sequence
- **Use case:** Emergency return, lost link, low battery

---

#### **LOITER (mode 12)** - `mode_loiter.cpp:4-161`
**Circles at current location.**

**How it works:**
```cpp
bool ModeLoiter::_enter() {
    plane.do_loiter_at_location();  // Set waypoint to current position
    return true;
}

void ModeLoiter::update() {
    plane.calc_nav_roll();  // L1 calculates bank angle for circle

    if (plane.stick_mixing_enabled() && ENABLE_LOITER_ALT_CONTROL) {
        plane.update_fbwb_speed_height();  // FBWB-style altitude control
    } else {
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
}

void ModeLoiter::navigate() {
    plane.update_loiter(0);  // Use WP_LOITER_RAD parameter
}
```

**Advanced feature - Heading alignment** (line 40-135):
```cpp
bool ModeLoiter::isHeadingLinedUp(const Location loiterCenterLoc,
                                   const Location targetLoc) {
    // Complex math to exit loiter on proper heading to target
    // Tolerance: 10° + 10°/circle (grows to prevent infinite loops)
    // Handles destination inside vs outside loiter radius
}
```

**Features:**
- **Circular loiter:** Uses L1 for smooth circles
- **Radius:** Configurable via `WP_LOITER_RAD`
- **Direction:** Clockwise or counter-clockwise
- **Exit logic:** Aligns heading before proceeding to next waypoint
- **Altitude:** Optional FBWB-style stick control or fixed
- **Use case:** Hold position, wait for clearance, photo/video orbits

---

#### **CIRCLE (mode 1)** - `mode_circle.cpp:4-23`
**Emergency GPS-independent circle.**

**How it works:**
```cpp
void ModeCircle::update() {
    // Fixed gentle bank angle
    plane.nav_roll_cd = plane.roll_limit_cd / 3;
    plane.update_load_factor();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}
```

**Key points:**
- **Fixed bank:** 1/3 of roll limit (e.g., 15° if limit is 45°)
- **No GPS required:** Just banks and circles based on angle
- **Altitude hold:** TECS maintains altitude
- **Designed for:** GPS failure or lost RC link
- **Use case:** Failsafe mode, no-GPS situations

---

#### **GUIDED (mode 15)** - `mode_guided.cpp:4-202`
**External control from GCS or companion computer.**

**Multiple control modes:**

**1. Forced RPY** (line 40-42):
```cpp
// Direct roll/pitch/yaw commands from external controller
if (plane.guided_state.last_forced_rpy_ms.x > 0 &&
    millis() - plane.guided_state.last_forced_rpy_ms.x < timeout) {
    plane.nav_roll_cd = plane.guided_state.forced_rpy_cd.x;
}
```

**2. Heading control** (line 48-71):
```cpp
// PID-controlled heading tracking
float error = wrap_PI(plane.guided_state.target_heading - ahrs.get_yaw_rad());
float desired = plane.g2.guidedHeading.update_error(error, delta);
plane.nav_roll_cd = constrain(desired, -bank_limit, bank_limit);
```

**3. Forced throttle** (line 90-94):
```cpp
// Direct throttle command
SRV_Channels::set_output_scaled(SRV_Channel::k_throttle,
    plane.guided_state.forced_throttle);
```

**4. Airspeed slew** (line 123-151):
```cpp
// Gradual speed changes with acceleration limit
plane.guided_state.target_airspeed_cm = new_airspeed_cm;
plane.guided_state.target_airspeed_accel = acceleration;
```

**Features:**
- **Waypoint navigation:** Default - flies to GCS-commanded location
- **Direct attitude:** Companion can command roll/pitch/yaw
- **Throttle override:** Direct throttle control
- **Heading tracking:** PID-based heading hold
- **Speed control:** Gradual speed changes
- **Timeout:** Commands expire after `guided_timeout` seconds
- **Use case:** Companion computer, object tracking, precision maneuvers

---

#### **TAKEOFF (mode 13)** - `mode_takeoff.cpp:71-205`
**Autonomous takeoff to loiter.**

**Parameters:**
```
TKOFF_ALT:        Target altitude (default 50m)
TKOFF_LVL_ALT:    Wings-level altitude (default 10m)
TKOFF_LVL_PITCH:  Initial pitch angle (default 15°)
TKOFF_DIST:       Distance to loiter point (default 200m)
TKOFF_GND_PITCH:  Ground pitch angle (default 5°)
```

**Sequence:**

**1. Setup** (line 82-138):
```cpp
if (!takeoff_mode_setup) {
    if (plane.is_flying() && groundspeed > 3) {
        // Already flying - skip ground roll
        gcs().send_text(MAV_SEVERITY_INFO, "Climbing to TKOFF alt");
    } else {
        // Ground start - set target waypoint
        plane.next_WP_loc = plane.current_loc;
        plane.next_WP_loc.offset_up_m(alt);
        plane.next_WP_loc.offset_bearing(direction, dist);

        // Lock in when groundspeed > 5 m/s
        if (groundspeed > GPS_GND_CRS_MIN_SPD) {
            takeoff_mode_setup = true;
        }
    }
}
```

**2. Takeoff phase** (line 177-181):
```cpp
if (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF) {
    plane.takeoff_calc_roll();      // Wings level or heading hold
    plane.takeoff_calc_pitch();     // Initial pitch angle
    plane.takeoff_calc_throttle();  // Full throttle
}
```

**3. Completion** (line 165-168):
```cpp
// Complete when altitude reached OR distance traveled
if (altitude_cm >= (alt*100 - 200) ||
    start_loc.get_distance(plane.current_loc) >= dist) {
    plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
}
```

**Smart features:**
- **Heading correction:** Updates heading based on actual track (corrects compass errors)
- **Already flying:** Skips ground roll if already airborne
- **Timeout:** Gives up if takeoff takes too long
- **Level-off:** Gradually reduces pitch as target altitude approaches
- **Use case:** Autonomous missions, automatic takeoff

---

### Special Modes

#### **AUTOTUNE (mode 8)** - `mode_autotune.cpp`
**Automatically tunes PID gains.**
- Wrapper mode enabling autotune library
- Flies patterns (doublets, sweeps) to measure response
- Adjusts P, I, D, FF gains for optimal performance
- **Use case:** Initial setup, after airframe changes

#### **THERMAL (mode 24)** - `mode_thermal.cpp`
**Autonomous thermal soaring.**
- Detects thermals (rising air) automatically
- Centers on thermal using MacCready theory
- Climbs efficiently, exits when lift weakens
- **Use case:** Gliders, long-endurance missions

#### **AVOID_ADSB (mode 14)** - `mode_avoidADSB.cpp`
**Collision avoidance.**
- Similar to GUIDED
- Automatically maneuvers away from ADSB-equipped aircraft
- Returns to previous mode when clear
- **Use case:** Shared airspace safety

#### **AUTOLAND (mode 26)** - `mode_autoland.cpp`
**Autonomous landing pattern.**
- Three stages: CLIMB → LOITER → LANDING
- Captures heading on first armed takeoff
- Uses rangefinder for flare
- **Use case:** Return-to-base landing

---

### VTOL/QuadPlane Modes

#### **QSTABILIZE (mode 17)** - VTOL stabilize with manual throttle
#### **QHOVER (mode 18)** - VTOL hover with altitude hold
#### **QLOITER (mode 19)** - VTOL loiter with GPS position hold
#### **QLAND (mode 20)** - VTOL vertical landing
#### **QRTL (mode 21)** - VTOL return to launch and land
#### **QACRO (mode 23)** - VTOL rate control (acrobatic)

---

### Summary Table

| Mode | Stabilization | Auto Throttle | Auto Navigation | GPS Required |
|------|--------------|---------------|-----------------|--------------|
| MANUAL | ❌ None | ❌ | ❌ | ❌ |
| STABILIZE | ✅ Level | ❌ | ❌ | ❌ |
| TRAINING | ✅ At limits | ❌ | ❌ | ❌ |
| ACRO | ✅ Rate | ❌ | ❌ | ❌ |
| FBWA | ✅ Attitude | ❌ | ❌ | ❌ |
| FBWB | ✅ Alt/Speed | ✅ | ❌ | ❌ |
| CRUISE | ✅ Alt/Speed | ✅ | ✅ Heading | ✅ |
| AUTO | ✅ Full | ✅ | ✅ Mission | ✅ |
| RTL | ✅ Full | ✅ | ✅ Home | ✅ |
| LOITER | ✅ Full | ✅ | ✅ Circle | ✅ |
| CIRCLE | ✅ Basic | ✅ | ❌ | ❌ |
| GUIDED | ✅ Full | ✅ | ✅ External | ✅ |
| TAKEOFF | ✅ Full | ✅ | ✅ Sequence | ✅ |

---

## Control Systems

### Hierarchical Control Architecture

```
Mission/Navigation
        ↓
   Guidance Loop (10Hz)
   [L1 Controller, Mission Logic]
        ↓
   Attitude Loop (FAST ~400Hz)
   [Roll/Pitch/Yaw Controllers]
        ↓
   Rate Loop (implicit in PID)
   [PID Controllers]
        ↓
   Servo Output (FAST)
   [PWM Generation]
```

### 1. Navigation Layer (10Hz)

**L1 Navigation Controller** (Plane.h:246)
- Implements L1 guidance for waypoint tracking
- Calculates desired roll angle (`nav_roll_cd`) for path following
- Handles:
  - Waypoint navigation
  - Loiter circles
  - Cross-track error correction
  - Turn anticipation

**Key File:** `navigation.cpp:19235`

**Navigation Update Flow:**
```cpp
navigate() [10Hz]
  ├── Update waypoint distance/bearing
  ├── L1_controller.update_waypoint()
  ├── Calculate nav_roll_cd
  └── Update nav_pitch_cd (from TECS)
```

### 2. Altitude/Speed Control (50Hz + 10Hz)

**TECS (Total Energy Control System)** (Plane.h:245)

TECS manages the aircraft's total energy (kinetic + potential):

```
Total Energy = Kinetic Energy + Potential Energy
             = ½mv² + mgh
```

**Two-Loop Structure:**
- **50Hz Inner Loop:** `update_50hz()` - Fast throttle/pitch response
- **10Hz Outer Loop:** `update_pitch_throttle()` - Energy management

**Key File:** `altitude.cpp:32429`

**Outputs:**
- Demanded pitch angle → `nav_pitch_cd`
- Demanded throttle percentage → servo output

**TECS Advantages:**
- Decouples speed and altitude control
- Optimal energy distribution
- Handles wind disturbances
- Prevents phugoid oscillations

### 3. Attitude Control (FAST Loop ~400Hz)

**Roll Control** (`stabilize_roll()` in Attitude.cpp)

```cpp
Input:  nav_roll_cd (from navigation)
        ahrs.roll_sensor (current roll)

Process:
1. error = nav_roll_cd - ahrs.roll_sensor
2. PID calculation with speed scaling
3. Stick mixing (if enabled)
4. Constrain to roll limits

Output: aileron_demand → SRV_Channel::k_aileron
```

**Pitch Control** (`stabilize_pitch()` in Attitude.cpp)

```cpp
Input:  nav_pitch_cd (from TECS)
        ahrs.pitch_sensor (current pitch)
        pitch_trim (from calibration)

Process:
1. error = nav_pitch_cd - ahrs.pitch_sensor + trim
2. PID with kff_throttle_to_pitch feedforward
3. Speed scaling
4. Landing flare override (if landing)

Output: elevator_demand → SRV_Channel::k_elevator
```

**Yaw Control** (`calc_nav_yaw_coordinated()` in Attitude.cpp)

Three modes:
1. **Coordinated Turn:** Rudder = G × tan(roll) / airspeed
2. **Ground Steering:** Course hold using bearing error
3. **Rate Control:** Direct yaw rate demand

**Speed Scaling** (`calc_speed_scaler()`)

Critical for all control surfaces:
```cpp
speed_scaler = SCALING_SPEED / current_airspeed
// Constrained to [0.5, 2.0]

// At high speed: less surface deflection needed
// At low speed: more surface deflection needed
```

### 4. Control Surface Mixing (servos.cpp)

**Supported Configurations:**
- Standard (aileron, elevator, rudder, throttle)
- Elevon (flying wing)
- V-tail
- Flaperon
- Differential spoilers
- Twin engines with differential thrust

**Output Pipeline:**
```cpp
set_servos() [FAST]
  ├── Throttle processing
  │   ├── Voltage compensation
  │   ├── Slew rate limiting
  │   └── Min/max constraints
  ├── Control surface outputs
  │   ├── Speed scaling
  │   ├── Mixing (elevon/vtail/etc)
  │   ├── Flap/airbrake logic
  │   └── Trim adjustment
  └── servos_output()
      └── SRV_Channels::output_ch_all()
```

### 5. Sensor Fusion (AHRS + EKF)

**AHRS (Attitude and Heading Reference System)**

Provides:
- Attitude (roll, pitch, yaw)
- Position estimates
- Velocity (NED frame)
- Wind speed estimates

**EKF (Extended Kalman Filter)**

Two implementations available:
- EKF2 (legacy)
- **EKF3** (default, recommended)

**Sensor Inputs:**
- IMU (accel + gyro): 200+ Hz
- GPS position/velocity: 5-10 Hz
- Compass (magnetometer): 10 Hz
- Airspeed: 10-50 Hz (optional)
- Barometer: 50 Hz
- Rangefinder: 50 Hz (optional, for landing)

**Update Rate:** `ahrs_update()` runs on FAST loop (~400Hz)

### PID Controller Architecture

All attitude controllers use the same structure:

```cpp
class AP_XXXController {
public:
    int32_t get_servo_out(int32_t error,     // Angle error
                          float speed_scaler, // Airspeed scaling
                          bool disable_i,     // Inhibit integrator
                          bool ground_mode);  // Ground suppression
private:
    float kP;  // Proportional gain
    float kI;  // Integral gain
    float kD;  // Derivative gain
    float kFF; // Feedforward gain
};
```

**Key Features:**
- Speed-dependent gain scheduling
- Integrator wind-up protection
- Ground mode suppression
- Notch filtering for harmonics
- Rate limiting

**Parameter Groups:**
```
ROLL:  RLL2SRV_P, RLL2SRV_I, RLL2SRV_D, RLL2SRV_FF
PITCH: PTCH2SRV_P, PTCH2SRV_I, PTCH2SRV_D, PTCH2SRV_FF
YAW:   YW2SRV_P, YW2SRV_I, YW2SRV_D, YW2SRV_FF
```

---

## Execution Loop

### Main Loop Structure (Plane.cpp)

**Entry Point:**
```cpp
AP_HAL_MAIN_CALLBACKS(&plane);  // Line 1089
  ↓
Plane::Plane()                   // Constructor
  ↓
init_ardupilot()                 // Initialization
  ↓
AP_Scheduler::run()              // Main loop
```

### Scheduler Tasks (Plane.cpp:62-147)

The scheduler is priority-based with guaranteed execution times:

```cpp
const AP_Scheduler::Task scheduler_tasks[] = {
    // FAST tasks (run every loop, no time limit)
    FAST_TASK(ahrs_update),
    FAST_TASK(update_control_mode),
    FAST_TASK(stabilize),
    FAST_TASK(set_servos),

    // Scheduled tasks (Hz, time budget us, priority)
    SCHED_TASK(read_radio, 50, 100, 6),
    SCHED_TASK(update_speed_height, 50, 200, 12),
    SCHED_TASK(update_GPS_50Hz, 50, 300, 30),
    SCHED_TASK(navigate, 10, 150, 36),
    SCHED_TASK(update_compass, 10, 200, 39),
    SCHED_TASK(calc_airspeed_errors, 10, 100, 42),
    SCHED_TASK(update_alt, 10, 200, 45),
    SCHED_TASK(ekf_check, 10, 75, 54),
    SCHED_TASK(one_second_loop, 1, 400, 90),
    SCHED_TASK(three_hz_loop, 3, 75, 93),
    // ... logging, telemetry, etc.
};
```

### Loop Execution Sequence

**Per-Loop (FAST tasks, ~400Hz):**

```
Loop Start
    ↓
ahrs_update()                    [~100us]
  ├── Update EKF with IMU data
  ├── Calculate attitude (roll/pitch/yaw)
  ├── Update position estimates
  ├── Calculate roll/pitch limits
  └── Log IMU data
    ↓
update_control_mode()            [varies]
  ├── Check mode-specific conditions
  └── Call control_mode->update()
    ↓
stabilize()                      [~50us]
  ├── stabilize_roll() → aileron output
  ├── stabilize_pitch() → elevator output
  └── calc_nav_yaw_coordinated() → rudder output
    ↓
set_servos()                     [~200us]
  ├── Process throttle (slew, limits, mixing)
  ├── Apply control surface mixing
  ├── Flap/airbrake deployment
  └── servos_output() → PWM generation
    ↓
Loop End (check timing, run scheduled tasks)
```

**Scheduled Tasks:**

```
50 Hz Tasks (every 2.5 loops @ 100Hz):
  - read_radio()             : Read RC inputs
  - update_speed_height()    : TECS 50Hz update
  - update_GPS_50Hz()        : GPS position update

10 Hz Tasks (every 10 loops):
  - navigate()               : L1 navigation calculations
  - update_compass()         : Magnetometer update
  - calc_airspeed_errors()   : Airspeed error calculation
  - update_alt()             : TECS outer loop (10Hz)
  - ekf_check()              : EKF health monitoring

1-3 Hz Tasks:
  - one_second_loop()        : Home update, parameter updates
  - three_hz_loop()          : Fence checks
  - check_long_failsafe()    : Failsafe monitoring

Variable Rate:
  - GCS update_receive()     : 300Hz (MAVLink input)
  - GCS update_send()        : 300Hz (telemetry output)
  - Log_Write_FullRate()     : 400Hz (high-rate logging)
```

### Loop Timing

**Target Loop Rate:** 400 Hz (2.5ms per loop)
**Typical Performance:** 200-400 Hz depending on hardware

**Time Budget Allocation (microseconds):**
```
ahrs_update:           ~100 us (typical <50)
stabilize:             ~100 us (typical <50)
set_servos:            ~500 us (typical <200)
read_radio:            ~100 us
update_speed_height:   ~200 us
navigate:              ~150 us
update_alt:            ~200 us
GCS update_send:       ~750 us
```

**Critical Timing:** FAST tasks have no time budget - they always run

### Failsafe Handling

The execution loop includes multiple failsafe monitors:

```cpp
check_short_rc_failsafe()    // 50Hz - RC signal loss
check_long_failsafe()        // 3Hz - Extended failsafe
ekf_check()                  // 10Hz - Navigation failure
fence_check()                // 3Hz - Geofence breach
battery failsafe             // 10Hz - Low battery
```

---

## Important Files

### Must-Read Files (Core Understanding)

| File | Lines | Purpose | Priority |
|------|-------|---------|----------|
| **Plane.h** | 1333 | Main class definition, all member variables | ★★★★★ |
| **Plane.cpp** | 1089 | Scheduler, main loop, ahrs_update | ★★★★★ |
| **mode.h** | 1040 | Flight mode class hierarchy | ★★★★★ |
| **Attitude.cpp** | 676 | Roll/pitch/yaw control (stabilize loop) | ★★★★★ |
| **Parameters.h** | 561 | Parameter definitions | ★★★★ |

### Control & Navigation

| File | Lines | Purpose | Priority |
|------|-------|---------|----------|
| **navigation.cpp** | 574 | L1 controller, waypoint logic | ★★★★ |
| **altitude.cpp** | 970 | TECS implementation, altitude control | ★★★★ |
| **servos.cpp** | 1365 | Servo output, mixing, throttle control | ★★★★ |
| **radio.cpp** | 417 | RC input processing | ★★★ |

### Mission & Autonomous Flight

| File | Lines | Purpose | Priority |
|------|-------|---------|----------|
| **mode_auto.cpp** | 186 | AUTO mode implementation | ★★★★ |
| **mode_rtl.cpp** | 209 | Return to launch logic | ★★★ |
| **mode_guided.cpp** | 247 | External control mode | ★★★ |
| **commands_logic.cpp** | 1423 | Mission command execution (waypoint, loiter, land, etc.) | ★★★★ |
| **takeoff.cpp** | 541 | Takeoff sequence management | ★★★ |

### VTOL/QuadPlane (if applicable)

| File | Lines | Purpose | Priority |
|------|-------|---------|----------|
| **quadplane.cpp** | 5681 | VTOL flight control | ★★★★ |
| **quadplane.h** | 650 | QuadPlane class definition | ★★★ |
| **tailsitter.cpp** | 1392 | Tailsitter VTOL type | ★★★ |
| **tiltrotor.cpp** | 1005 | Tiltrotor VTOL type | ★★★ |

### Safety & Monitoring

| File | Lines | Purpose | Priority |
|------|-------|---------|----------|
| **failsafe.cpp** | 106 | Failsafe logic | ★★★★ |
| **fence.cpp** | 278 | Geofence implementation | ★★★ |
| **ekf_check.cpp** | 213 | EKF health monitoring | ★★★ |
| **is_flying.cpp** | 402 | Flight detection, crash detection | ★★★ |
| **AP_Arming_Plane.cpp** | 457 | Pre-arm safety checks | ★★★ |

### Communication

| File | Lines | Purpose | Priority |
|------|-------|---------|----------|
| **GCS_MAVLink_Plane.cpp** | 1429 | MAVLink protocol implementation | ★★★ |
| **GCS_Plane.cpp** | 132 | GCS interface | ★★ |

### Logging & Debugging

| File | Lines | Purpose | Priority |
|------|-------|---------|----------|
| **Log.cpp** | 680 | Data logging (CTUN, NTUN, etc.) | ★★★ |
| **events.cpp** | 380 | Event logging | ★★ |

### Configuration

| File | Lines | Purpose | Priority |
|------|-------|---------|----------|
| **Parameters.cpp** | 2546 | Parameter initialization, conversion | ★★★★ |
| **config.h** | 197 | Compile-time configuration | ★★★ |
| **defines.h** | 205 | Constants and enums | ★★★ |
| **system.cpp** | 432 | System initialization | ★★★ |

### Flight Mode Implementation Files

Each flight mode typically has:
- Simple modes: 100-500 lines
- Complex modes (AUTO, QRTL): 500-1500 lines

Example breakdown:
```
mode_manual.cpp:      27 lines   (passthrough only)
mode_stabilize.cpp:   10 lines   (calls common stabilize)
mode_fbwa.cpp:        52 lines   (simple attitude control)
mode_auto.cpp:        186 lines  (mission following)
mode_guided.cpp:      247 lines  (external control logic)
mode_qrtl.cpp:        290 lines  (complex VTOL RTL sequence)
```

### Key Code Locations Reference

**For specific functionality, look here:**

```
Attitude control:       Attitude.cpp:stabilize()
Roll output:            Attitude.cpp:stabilize_roll_get_roll_out()
Pitch output:           Attitude.cpp:stabilize_pitch_get_pitch_out()
Yaw coordination:       Attitude.cpp:calc_nav_yaw_coordinated()
Speed scaling:          Attitude.cpp:calc_speed_scaler()

Navigation:             navigation.cpp:navigate()
L1 guidance:            L1_controller.update_waypoint() (in library)
Waypoint following:     navigation.cpp:update_loiter()

Altitude control:       altitude.cpp:update_alt()
TECS 50Hz:              altitude.cpp:update_speed_height()
TECS 10Hz:              TECS_controller.update_pitch_throttle()

Servo output:           servos.cpp:set_servos()
Throttle:               servos.cpp:set_throttle()
Surface mixing:         servos.cpp:flaperon_update() / channel_function_mixer()

RC input:               radio.cpp:read_radio()
Failsafe check:         radio.cpp:control_failsafe()

Mode switching:         control_modes.cpp:set_mode()
Mode update:            Plane.cpp:update_control_mode()

Mission execution:      commands_logic.cpp:start_command() / verify_command()
Takeoff:                takeoff.cpp:auto_takeoff_check()
Landing:                landing library (AP_Landing)

QuadPlane control:      quadplane.cpp:update() / assisted_flight()
VTOL transitions:       quadplane.cpp:transition logic
```

---

## Build System

### WAF Build System (wscript)

ArduPlane uses the WAF build system (Python-based):

```python
def build(bld):
    vehicle = bld.path.name  # "ArduPlane"

    # Build static library with dependencies
    bld.ap_stlib(
        name=vehicle + '_libs',
        ap_vehicle=vehicle,
        ap_libraries=bld.ap_common_vehicle_libraries() + [
            'APM_Control',          # PID controllers
            'AP_AdvancedFailsafe',
            'AP_Avoidance',
            'AP_Camera',
            'AP_L1_Control',        # L1 navigation
            'AP_Navigation',
            'AP_TECS',              # TECS altitude control
            'AP_InertialNav',       # QuadPlane position control
            'AC_WPNav',             # QuadPlane waypoint navigation
            'AC_AttitudeControl',   # QuadPlane attitude control
            'AP_Motors',            # Motor control
            'AP_Landing',           # Landing library
            'PID',                  # PID controller base
            'AP_Soaring',           # Thermal soaring
            'AP_LTM_Telem',        # LTM telemetry
            'AP_Devo_Telem',       # Devo telemetry
            'AC_AutoTune',          # Autotuning
            'AP_Follow',            # Follow mode
            'AC_PrecLand',          # Precision landing
            'AP_IRLock',            # IR-LOCK sensor
            'AP_Quicktune',         # Quick tuning
        ],
    )

    # Build executable
    bld.ap_program(
        program_name='arduplane',
        program_groups=['bin', 'plane'],
        use=vehicle + '_libs',
    )
```

### Building ArduPlane

**1. Configure:**
```bash
./waf configure --board=<board_name>
```

Common boards:
- `CubeOrange` - Pixhawk Cube Orange
- `Pixhawk1` - Original Pixhawk
- `MatekH743` - Matek H743
- `linux` - Linux (SITL)
- `sitl` - Software In The Loop simulation

**2. Build:**
```bash
./waf plane
```

**3. Upload:**
```bash
./waf --upload plane
```

### Compile-Time Configuration

**Feature Flags (config.h):**
```cpp
#define HAL_QUADPLANE_ENABLED        // Enable VTOL support
#define MODE_AUTOLAND_ENABLED 1      // Enable AUTOLAND mode
#define HAL_SOARING_ENABLED          // Enable thermal soaring
#define AP_QUICKTUNE_ENABLED         // Enable quick tuning
#define HAL_LOGGING_ENABLED          // Enable logging
#define AP_ADSB_AVOIDANCE_ENABLED    // Enable ADSB avoidance
```

**Default Parameters (config.h):**
```cpp
#define ROLL_LIMIT_DEG 45            // Max roll angle
#define PITCH_MAX 20                 // Max pitch up
#define PITCH_MIN -25                // Max pitch down
#define AIRSPEED_CRUISE 12           // Default cruise (m/s)
#define RUDDER_MIX 0.5f              // Rudder mixing gain
```

### Build Variants

**SITL (Software In The Loop):**
```bash
sim_vehicle.py -v ArduPlane
```

**Hardware Upload:**
```bash
./waf configure --board=CubeOrange
./waf plane --upload
```

**Debugging Build:**
```bash
./waf configure --board=linux --debug
./waf plane
```

---

## Debugging

### 1. Logging System

**Log Message Types (Log.cpp):**

| Message | Frequency | Purpose |
|---------|-----------|---------|
| **CTUN** | 25Hz | Control tuning (roll/pitch/throttle desired vs actual, airspeed, climb) |
| **NTUN** | 25Hz | Navigation tuning (waypoint distance, crosstrack, altitude error, bearing) |
| **AETR** | 400Hz | Servo outputs (aileron, elevator, throttle, rudder, flap, steering) |
| **IMU** | 400Hz | IMU data (accel, gyro) |
| **GPS** | 5-10Hz | GPS position, velocity, accuracy |
| **ATTITUDE** | 10-400Hz | Roll, pitch, yaw angles |
| **STATUS** | Events | Flight state (armed, flying, crashed) |
| **TECS** | Variable | TECS state (energy, altitude, speed) |
| **RC** | 25Hz | RC input values |
| **COMPASS** | 10Hz | Magnetometer readings |

**Enable Logging:**
```
LOG_BITMASK = 0xFFFF  (enable all)
```

**Log Bitmask Values (defines.h:101-120):**
```cpp
MASK_LOG_ATTITUDE_FAST      (1<<0)   // 25Hz attitude
MASK_LOG_ATTITUDE_MED       (1<<1)   // 10Hz attitude
MASK_LOG_GPS                (1<<2)   // GPS data
MASK_LOG_PM                 (1<<3)   // Performance monitoring
MASK_LOG_CTUN               (1<<4)   // Control tuning
MASK_LOG_NTUN               (1<<5)   // Navigation tuning
MASK_LOG_IMU                (1<<7)   // IMU data
MASK_LOG_CMD                (1<<8)   // Mission commands
MASK_LOG_CURRENT            (1<<9)   // Battery current
MASK_LOG_COMPASS            (1<<10)  // Magnetometer
MASK_LOG_TECS               (1<<11)  // TECS data
MASK_LOG_RC                 (1<<13)  // RC input/output
MASK_LOG_IMU_RAW            (1<<19)  // Raw IMU
MASK_LOG_ATTITUDE_FULLRATE  (1<<20)  // 400Hz attitude
```

**Example: Enable CTUN + NTUN + IMU + RC:**
```
LOG_BITMASK = 16 + 32 + 128 + 8192 = 8368
```

### 2. Real-Time Debugging (GCS Text Messages)

**Severity Levels:**
```cpp
gcs().send_text(MAV_SEVERITY_INFO,     "Info message");
gcs().send_text(MAV_SEVERITY_NOTICE,   "Notice");
gcs().send_text(MAV_SEVERITY_WARNING,  "Warning!");
gcs().send_text(MAV_SEVERITY_CRITICAL, "Critical!");
gcs().send_text(MAV_SEVERITY_ALERT,    "Alert!");
```

**Example Messages in Code:**
```cpp
"Beginning INS calibration"
"EKF2 IMU0 is using GPS"
"Landing aborted, climbing to %dm"
"Auto disarmed"
"PreArm: Compass not calibrated"
```

### 3. PID Tuning Interface

**Enable PID Telemetry:**
```
GCS_PID_MASK = 63  (all PIDs)
```

**Bitmask (defines.h:72-79):**
```cpp
TUNING_BITS_ROLL  = (1<<0)   // Roll PID telemetry
TUNING_BITS_PITCH = (1<<1)   // Pitch PID telemetry
TUNING_BITS_YAW   = (1<<2)   // Yaw PID telemetry
TUNING_BITS_STEER = (1<<3)   // Steering PID
TUNING_BITS_LAND  = (1<<4)   // Landing PID
TUNING_BITS_ACCZ  = (1<<5)   // VTOL Z-accel PID
```

View in GCS "Tuning" window to see:
- Desired vs actual values
- P, I, D contributions
- Output values
- Integrator state

### 4. SITL (Software In The Loop)

**Basic SITL:**
```bash
sim_vehicle.py -v ArduPlane -L CMAC --console --map
```

**Parameters:**
- `-v ArduPlane` - Vehicle type
- `-L CMAC` - Location (or use `--location=lat,lon,alt,hdg`)
- `--console` - Open MAVProxy console
- `--map` - Open map display
- `--speedup=1` - Simulation speed multiplier

**Advanced SITL with Physics:**
```bash
sim_vehicle.py -v ArduPlane --console --map \
  --sim-param="SIM_WIND_SPD=5" \
  --sim-param="SIM_WIND_DIR=270" \
  --sim-param="SIM_WIND_TURB=0.5"
```

**Available SITL Parameters (SITL.h):**

```cpp
// Wind simulation
SIM_WIND_SPD       // Wind speed (m/s)
SIM_WIND_DIR       // Wind direction (degrees)
SIM_WIND_TURB      // Turbulence amplitude

// GPS simulation
SIM_GPS_DISABLE    // Disable GPS
SIM_GPS_DELAY      // GPS delay (ms)
SIM_GPS_GLITCH     // GPS glitch injection
SIM_GPS_ALT_OFS    // GPS altitude offset (m)

// Airspeed simulation
SIM_ARSPD_FAIL     // Airspeed sensor failure
SIM_ARSPD_OFS      // Airspeed offset (m/s)
SIM_ARSPD_NOISE    // Airspeed noise

// IMU simulation
SIM_GYRO_NOISE     // Gyro noise (deg/s)
SIM_ACCEL_NOISE    // Accel noise (m/s²)
SIM_GYRO_FAIL      // Gyro failure mask
SIM_ACCEL_FAIL     // Accel failure mask

// Barometer simulation
SIM_BARO_DRIFT     // Baro drift (m/s)
SIM_BARO_NOISE     // Baro noise (m)

// Compass simulation
SIM_MAG_FAIL       // Compass failure mask
SIM_MAG_NOISE      // Compass noise
```

**Inject Failures in SITL:**
```python
# In MAVProxy console:
param set SIM_GPS_DISABLE 1        # Lose GPS
param set SIM_RC_FAIL 1            # RC failsafe
param set SIM_BARO_NOISE 5         # 5m baro noise
param set SIM_MAG_FAIL 1           # Compass failure
```

### 5. Parameter Analysis

**Key Parameters for Tuning:**

**Roll Control:**
```
RLL2SRV_P          - Roll P gain (default 1.0)
RLL2SRV_I          - Roll I gain (default 0.3)
RLL2SRV_D          - Roll D gain (default 0.08)
RLL2SRV_FF         - Roll feedforward (default 0)
RLL_RATE_MAX       - Max roll rate (deg/s, default 0)
ROLL_LIMIT_DEG     - Max roll angle (default 45)
```

**Pitch Control:**
```
PTCH2SRV_P         - Pitch P gain (default 1.5)
PTCH2SRV_I         - Pitch I gain (default 0.3)
PTCH2SRV_D         - Pitch D gain (default 0.08)
PTCH2SRV_FF        - Pitch feedforward (default 0)
PTCH_RATE_MAX      - Max pitch rate (deg/s, default 0)
```

**Yaw Control:**
```
YW2SRV_P           - Yaw P gain (default 0)
YW2SRV_I           - Yaw I gain (default 0)
YW2SRV_D           - Yaw D gain (default 0)
YW2SRV_RLL         - Yaw from roll (default 1.0)
```

**TECS (Altitude/Speed):**
```
TECS_PITCH_MAX     - Max pitch for TECS (default 15)
TECS_PITCH_MIN     - Min pitch for TECS (default 0)
TECS_THR_DAMP      - Throttle damping (default 0.7)
TECS_SINK_MAX      - Max descent rate (m/s, default 5)
TECS_CLMB_MAX      - Max climb rate (m/s, default 5)
TECS_SPDWEIGHT     - Speed vs height priority (default 1.0)
```

**L1 Navigation:**
```
NAVL1_PERIOD       - L1 period (sec, default 20)
NAVL1_DAMPING      - L1 damping (default 0.75)
WP_RADIUS          - Waypoint radius (m, default 90)
WP_LOITER_RAD      - Loiter radius (m, default 60)
```

### 6. Common Debug Workflows

**A. Control Oscillation:**
1. Enable `LOG_BITMASK` with CTUN + ATTITUDE_FAST
2. Fly test mission
3. Download logs
4. Plot desired vs actual roll/pitch
5. Check for:
   - P too high: Fast oscillation
   - I too high: Slow oscillation + overshoot
   - D too high: High-frequency noise

**B. Navigation Issues:**
1. Enable `LOG_BITMASK` with NTUN + GPS
2. Enable `GCS_PID_MASK` = 1 (roll tuning)
3. Fly waypoint mission
4. Check logs for:
   - `crosstrack_error` - Should converge to near zero
   - `nav_roll` vs `roll` - Should track closely
   - GPS accuracy (`hdop` < 2.0 is good)

**C. Altitude Control:**
1. Enable `MASK_LOG_TECS`
2. Fly altitude changes
3. Analyze TECS log:
   - `h_dem` vs `h` - Demanded vs actual altitude
   - `spd_dem` vs `spd` - Demanded vs actual speed
   - `thr` - Throttle output
   - `ptch` - Pitch output

**D. EKF Issues:**
1. Check `EKF_CHECK_THRESH` parameter
2. Enable GPS + IMU + COMPASS logging
3. Watch for messages:
   - "EKF variance"
   - "GPS glitch"
   - "DCM bad heading"
4. Analyze:
   - EKF innovations
   - GPS accuracy metrics
   - Velocity consistency

### 7. In-Flight Debugging

**MAVLink Commands:**
```
MAV_CMD_PREFLIGHT_CALIBRATION  - Calibrate sensors
MAV_CMD_DO_SET_SERVO           - Test servo output
MAV_CMD_DO_SET_PARAMETER       - Change parameter
MAV_CMD_REQUEST_MESSAGE        - Request specific message
MAV_CMD_LOGGING_START/STOP     - Control logging
```

**RC Aux Functions:**
Available via RC switches:
- RC_OPTIONS: Various flight options
- AUTOTUNE: Enable autotuning
- TUNE_MIN/MAX: Adjust parameters in flight
- FENCE: Enable/disable geofence
- LOG: Start/stop logging

### 8. Performance Monitoring

**Scheduler Performance:**
```
LOG_BITMASK with MASK_LOG_PM
```

Logs:
- Loop rate (should be ~200-400Hz)
- Task overruns
- Maximum loop time
- Load percentage

**Memory Usage:**
Watch for:
- Stack overflows
- Heap exhaustion
- Memory fragmentation

**CPU Load:**
`SCHED_LOOP_RATE` parameter shows target loop rate
Actual rate logged in PM (Performance Monitoring) messages

---

## Interesting Properties

### 1. Sophisticated Energy Management (TECS)

**What makes it special:**

Traditional altitude controllers decouple altitude and speed, leading to:
- Pitch changes affect both speed and altitude
- Throttle changes affect both speed and altitude
- Pilots must coordinate pitch and throttle

TECS treats the aircraft as an energy system:
```
Total Energy = Kinetic + Potential
             = ½mv² + mgh

Energy Rate = Throttle - Drag
```

**Benefits:**
- Optimal pitch/throttle coordination
- Eliminates phugoid oscillations
- Handles wind shear gracefully
- Works well in turbulence

**Implementation:** `altitude.cpp:update_alt()` (Plane.cpp:587-661)

### 2. L1 Navigation Controller

**Why L1 is elegant:**

Classic navigation uses proportional navigation:
```
roll_cmd = K × cross_track_error
```
Problem: Oscillates on straight lines, poor performance in wind

L1 uses **non-linear guidance**:
```
L1 = (1/π) × V² × Period
steering_point = look-ahead point on path
roll_cmd = f(lateral_accel_to_steering_point)
```

**Advantages:**
- Smooth path tracking
- No overshoot
- Wind-independent performance
- Single tuning parameter (`NAVL1_PERIOD`)
- Mathematical proof of stability

**Reference:** Small unmanned aircraft - Beard & McLain

### 3. Unified VTOL Integration (QuadPlane)

**Remarkable aspects:**

ArduPlane seamlessly integrates:
- Fixed-wing flight dynamics
- Multicopter hovering
- Transition between modes
- Multiple VTOL configurations (tailsitter, tiltrotor, standard)

**Single codebase supports:**
- Pure fixed-wing
- Pure multicopter modes (QHOVER, QLOITER)
- Assisted flight (copter helping fixed-wing)
- Autonomous transitions
- VTOL takeoff → fixed-wing cruise → VTOL landing

**Implementation:** `quadplane.cpp` (189KB, 5681 lines)

**Transition strategies:**
- Airspeed-based
- Throttle-based
- Time-based
- Altitude-based

### 4. Speed Scaling Innovation

**Problem:** Control surfaces are speed-dependent
- High speed: small deflections needed
- Low speed: large deflections needed
- Stall speed: maximum deflections

**Solution:** Dynamic gain scheduling:
```cpp
speed_scaler = SCALING_SPEED / current_airspeed
constrained to [0.5, 2.0]

// All PID gains multiplied by speed_scaler
// Surface deflections automatically adjusted
```

**Result:**
- Single PID tune works across speed range
- No need for gain scheduling tables
- Handles gusts and turbulence
- Prevents stall in slow flight

**Implementation:** `Attitude.cpp:calc_speed_scaler()`

### 5. Scheduler Design Philosophy

**Key insight:** Not all tasks are equally critical

**Three-tier priority system:**
1. **FAST tasks:** Always run (attitude control, servo output)
2. **High-frequency tasks:** 50Hz (RC input, TECS inner loop)
3. **Medium/low frequency:** 10Hz, 1Hz (navigation, housekeeping)

**Guaranteed execution:**
- FAST tasks never skipped
- Other tasks have time budgets
- Scheduler monitors overruns
- System degrades gracefully under load

**Example:** If GCS telemetry falls behind, attitude control continues unaffected

**Implementation:** `Plane.cpp:scheduler_tasks[]` + `AP_Scheduler` library

### 6. Comprehensive Failsafe Architecture

**Multi-layer safety net:**

```
Layer 1: Input Failsafe
  - RC signal loss
  - GCS heartbeat loss
  - Throttle failsafe

Layer 2: State Failsafe
  - Battery low/critical
  - EKF failure (navigation loss)
  - Geofence breach

Layer 3: Physical Failsafe
  - Crash detection
  - Stall detection
  - Terrain collision

Layer 4: Emergency Actions
  - Parachute deployment
  - Engine cutoff
  - Forced landing
```

**Failsafe Actions** (defines.h:36-51):
```cpp
FS_ACTION_SHORT_CIRCLE    // Short failsafe: circle
FS_ACTION_SHORT_FBWA      // Short: FBWA
FS_ACTION_LONG_RTL        // Long failsafe: RTL
FS_ACTION_LONG_GLIDE      // Long: dead-stick glide
FS_ACTION_LONG_PARACHUTE  // Long: deploy parachute
FS_ACTION_LONG_AUTOLAND   // Long: autonomous land
```

**Priority-based failsafe** (Plane.h:1249-1261):
If multiple failsafes active simultaneously, highest priority wins:
```cpp
Priority order:
1. Terminate
2. Parachute
3. QLand (VTOL)
4. Land
5. RTL
6. None
```

### 7. Parameter System Sophistication

**Features:**
- 1000+ parameters
- Persistent storage (EEPROM/flash)
- Runtime modification
- Parameter versioning
- Automatic conversion between versions
- Hierarchical grouping
- Validation and constraints

**Example:** Updating from v3.9 to v4.0 automatically converts old parameter names to new ones

**Implementation:**
```cpp
AP_Param::convert_class(old_index, new_object, var_info, flags)
```

File: `Parameters.cpp` (2546 lines)

### 8. Thermal Soaring Intelligence

**Automatic thermal detection and exploitation:**

```
1. Detect thermal (vertical speed increase)
2. Switch to THERMAL mode
3. Center on thermal using MacCready theory
4. Climb in thermal
5. Exit when thermal weakens
6. Resume mission
```

**Uses:**
- Variometer (vertical speed sensor)
- Extended Kalman Filter for thermal estimation
- MacCready speed-to-fly theory
- Optimal circling algorithms

**Perfect for:**
- Gliders
- Motor gliders
- Long-endurance missions

**Implementation:** `soaring.cpp` + `mode_thermal.cpp` + `AP_Soaring` library

### 9. Aerobatic Capabilities

**ACRO mode features:**
- **Quaternion-based control** (prevents gimbal lock)
- Rate-based inputs (pilot commands angular rates)
- Knife-edge flight support
- Inverted flight
- Yaw damper (optional)

**Advanced maneuvers possible:**
- Rolls (aileron)
- Loops (elevator)
- Knife-edge (rudder)
- Snap rolls
- Hammerheads

**Implementation:** `mode_acro.cpp:stabilize_quaternion()` (line 225)

### 10. Dual-Use Navigation System

**Interesting property:** Same navigation code handles:

**Fixed-Wing:**
- L1 controller
- Coordinated turns
- Bank angle limits
- Airspeed-dependent turns

**VTOL (QuadPlane):**
- Position control
- Velocity control
- Loiter (multicopter style)
- Precision landing

**Seamless switching:**
```cpp
if (quadplane.in_vtol_mode()) {
    // Use multicopter navigation
    quadplane.wp_nav->update()
} else {
    // Use fixed-wing navigation
    L1_controller.update_waypoint()
}
```

### 11. Auto-Landing Sophistication

**Landing phase management:**
```
1. Approach (normal flight)
2. Flare initiation (rangefinder trigger)
3. Flare (pitch up, throttle cut)
4. Touchdown (disarm when stopped)
```

**Flare control:**
- Rangefinder-based (measures actual height AGL)
- Exponential flare path
- Airspeed-dependent pitch
- Throttle management
- Go-around capability

**Implementation:** `AP_Landing` library + `altitude.cpp:tecs_hgt_afe()`

### 12. Model Predictive Control (implicit)

**Interesting observation:**

While not explicitly MPC, the system exhibits predictive behavior:

**L1 Controller:**
- Looks ahead on path (steering point)
- Predicts future position
- Optimizes trajectory

**TECS:**
- Predicts energy state
- Anticipates energy needs
- Optimal control allocation

**Takeoff/Landing:**
- Predicts altitude at waypoint
- Pre-emptively adjusts pitch/throttle
- Anticipates turns (setup_turn_angle)

### 13. Dual EKF Support

**Redundancy:**
- Can run EKF2 and EKF3 simultaneously
- Automatic fallback if one fails
- Different sensor fusion strategies
- Voting algorithm for best estimate

**EKF3 improvements over EKF2:**
- Better handling of GPS glitches
- Improved wind estimation
- Faster convergence
- Better terrain following
- Support for more sensors

**Parameter:** `AHRS_EKF_TYPE` (2=EKF2, 3=EKF3)

### 14. Advanced Mission Capabilities

**Beyond simple waypoints:**

```
NAV_WAYPOINT           - Go to waypoint
NAV_LOITER_UNLIM       - Loiter forever
NAV_LOITER_TIME        - Loiter for N seconds
NAV_LOITER_TURNS       - Loiter for N circles
NAV_LOITER_TO_ALT      - Descend in loiter to altitude
NAV_RETURN_TO_LAUNCH   - RTL
NAV_LAND               - Auto-land
NAV_TAKEOFF            - Auto-takeoff
NAV_CONTINUE_AND_CHANGE_ALT - Abort landing, climb
NAV_LOITER_TO_ALT      - Loiter descending
NAV_ALTITUDE_WAIT      - Wait at altitude

CONDITION_DELAY        - Wait N seconds
CONDITION_DISTANCE     - Wait until N meters from WP
CONDITION_YAW          - Turn to heading

DO_SET_SERVO          - Servo output
DO_SET_RELAY          - Relay on/off
DO_CHANGE_SPEED       - Change airspeed
DO_SET_CAM_TRIGG_DIST - Camera trigger
DO_PARACHUTE          - Deploy parachute
DO_INVERTED_FLIGHT    - Enable/disable inverted
DO_VTOL_TRANSITION    - Force VTOL transition
DO_LAND_START         - Mark landing approach start
```

**Mission features:**
- Rally points (alternate landing sites)
- Geofencing with breach actions
- DO_JUMP (loops)
- Conditional commands
- Scripted missions (Lua)

### 15. External Control Integration

**Multiple control interfaces:**

1. **MAVLink GUIDED mode:**
   - External position commands
   - Velocity commands
   - Attitude commands

2. **AP_Scripting (Lua):**
   - Custom flight logic
   - New flight modes
   - Mission commands
   - Sensor data access

3. **AP_ExternalControl:**
   - Companion computer integration
   - High-rate control
   - Custom controllers

**Example use cases:**
- Vision-based landing
- Object tracking
- Formation flying
- Custom aerobatics
- Research experiments

**Implementation:**
- `mode_guided.cpp`
- `AP_ExternalControl_Plane.cpp`
- `AP_Scripting` library

---

## Summary

ArduPlane is a **production-grade, real-time autopilot** with:

**Strengths:**
- Sophisticated control algorithms (TECS, L1)
- Extensive flight mode support
- Robust failsafe architecture
- Comprehensive logging and debugging
- Active development and community
- Hardware abstraction (runs on many platforms)
- VTOL integration (QuadPlane)

**Code Quality:**
- Well-structured (mode pattern, scheduler pattern)
- Extensively documented (comments, release notes)
- Parameter-driven (highly configurable)
- Hardware abstraction layer (AP_HAL)
- Modular design

**Best For:**
- Fixed-wing autonomous flight
- VTOL aircraft (QuadPlane)
- Research and education
- Commercial applications
- Long-range missions
- Precision agriculture
- Mapping and surveying

**Learning Curve:**
- **Entry point:** Start with `Plane.h`, `Plane.cpp`, `mode.h`
- **Control systems:** Read `Attitude.cpp`, `altitude.cpp`
- **Navigation:** Study `navigation.cpp`, L1 controller
- **Debugging:** Use SITL + MAVProxy + log analysis

**Further Reading:**
- ArduPilot documentation: https://ardupilot.org/plane/
- Developer documentation: https://ardupilot.org/dev/
- MAVLink protocol: https://mavlink.io/
- Source code: https://github.com/ArduPilot/ardupilot

---

*Document generated: 2025-10-16*
*ArduPlane version: Master branch (based on file structure)*
*Analysis scope: /Users/Shared/Projects/ardupilot/ArduPlane/*
