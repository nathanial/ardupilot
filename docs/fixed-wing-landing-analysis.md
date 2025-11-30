# ArduPlane Fixed Wing Landing System - Complete Guide

This document provides a comprehensive analysis of how landing works for fixed wing aircraft in ArduPilot (ArduPlane). It serves both developers working on the codebase and operators tuning landing parameters.

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [System Architecture](#system-architecture)
3. [Landing Entry Points](#landing-entry-points)
4. [Landing Phases Deep Dive](#landing-phases-deep-dive)
5. [TECS During Landing](#tecs-during-landing)
6. [L1 Navigation During Landing](#l1-navigation-during-landing)
7. [Altitude Sources and Sensor Fusion](#altitude-sources-and-sensor-fusion)
8. [Safety Systems](#safety-systems)
9. [Wind Handling](#wind-handling)
10. [Parameters Reference](#parameters-reference)
11. [Tuning Guide](#tuning-guide)
12. [Troubleshooting](#troubleshooting)
13. [Source Files Reference](#source-files-reference)
14. [Appendix: Glossary of Terms](#appendix-glossary-of-terms)

---

## Executive Summary

### How Landing Works in 30 Seconds

ArduPlane landing uses a **glide slope approach** followed by a **flare** to touchdown. The aircraft:
1. Approaches on a calculated descent path (glide slope)
2. Reduces speed in an optional pre-flare phase
3. Flares (pitches up, cuts throttle) at a configurable altitude/time
4. Touches down and disarms

### Key Systems Involved

| System | Role |
|--------|------|
| **AP_Landing** | State machine controlling landing phases |
| **TECS** | Manages altitude vs. airspeed trade-off (pitch & throttle) |
| **L1 Controller** | Lateral navigation (steering to landing point) |
| **Rangefinder** | Optional ground-truth altitude for flare timing |

### How to Use This Document

- **Operators**: Start with [Tuning Guide](#tuning-guide) and [Parameters Reference](#parameters-reference)
- **Developers**: Read [System Architecture](#system-architecture) and [Landing Phases Deep Dive](#landing-phases-deep-dive)
- **Troubleshooting**: Jump to [Troubleshooting](#troubleshooting) for common issues

---

## System Architecture

### Component Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        FLIGHT MODE LAYER                                │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐                          │
│  │   AUTO   │    │ AUTOLAND │    │   RTL    │                          │
│  │  Mode    │    │   Mode   │    │   Mode   │                          │
│  └────┬─────┘    └────┬─────┘    └────┬─────┘                          │
│       │               │               │                                 │
│       └───────────────┴───────────────┘                                 │
│                       │                                                 │
│                       ▼                                                 │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                      AP_Landing                                  │   │
│  │  - Landing state machine (NORMAL → APPROACH → PREFLARE → FINAL) │   │
│  │  - Flare triggering logic                                        │   │
│  │  - Rangefinder integration                                       │   │
│  │  - Go-around management                                          │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                       │                                                 │
│       ┌───────────────┼───────────────┐                                 │
│       ▼               ▼               ▼                                 │
│  ┌─────────┐    ┌─────────┐    ┌─────────────┐                         │
│  │  TECS   │    │   L1    │    │ Rangefinder │                         │
│  │         │    │ Control │    │   State     │                         │
│  │ Pitch & │    │         │    │             │                         │
│  │Throttle │    │  Roll   │    │  Altitude   │                         │
│  └─────────┘    └─────────┘    └─────────────┘                         │
│       │               │               │                                 │
│       └───────────────┴───────────────┘                                 │
│                       │                                                 │
│                       ▼                                                 │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                    SERVO OUTPUTS                                 │   │
│  │            Elevator    Throttle    Ailerons                      │   │
│  └─────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
```

### Core Components

#### 1. AP_Landing Library

Location: `libraries/AP_Landing/`

The landing library is the central coordinator. It:
- Manages the landing state machine (stages)
- Determines when to flare
- Integrates rangefinder corrections
- Handles go-around requests

**Key Files:**
| File | Purpose |
|------|---------|
| `AP_Landing.h` | Interface definitions, stage enums |
| `AP_Landing.cpp` | Landing dispatcher, parameter definitions |
| `AP_Landing_Slope.cpp` | Standard glide slope implementation |
| `AP_Landing_Deepstall.cpp` | Deep stall landing (optional) |

**Landing Types:**
- `LAND_TYPE=0`: Standard glide slope (most common)
- `LAND_TYPE=1`: Deep stall (requires `HAL_LANDING_DEEPSTALL_ENABLED`)

#### 2. TECS (Total Energy Control System)

Location: `libraries/AP_TECS/`

TECS manages the fundamental trade-off between altitude and airspeed by coordinating pitch and throttle. During landing, it:
- Controls descent rate on the glide slope
- Manages the flare sink rate
- Transitions smoothly between flight phases

#### 3. L1 Navigation Controller

Location: `libraries/AP_L1_Control/`

L1 provides lateral guidance (roll commands) to:
- Track the approach path to the landing point
- Correct for crosswind
- Keep wings level during flare

#### 4. Flight Stage Management

Defined in `AP_FixedWing::FlightStage`:

| Stage | Value | Description |
|-------|-------|-------------|
| `NORMAL` | 3 | Standard flight |
| `LAND` | 4 | Landing in progress |
| `ABORT_LANDING` | 7 | Go-around sequence |

---

## Landing Entry Points

There are three ways to initiate a landing:

### 1. AUTO Mode (Mission-Based)

**When to use:** Planned missions with specific landing locations.

**How it works:**
1. Mission includes `MAV_CMD_NAV_LAND` waypoint
2. Previous waypoint defines approach direction
3. Landing point defines touchdown location

**Flow** (from `ArduPlane/commands_logic.cpp:405-433`):
```
MAV_CMD_NAV_LAND received
    │
    ├── Set landing waypoint as target
    ├── Configure abort altitude (cmd.p1, default 30m)
    ├── Reset rangefinder state
    └── Call landing.do_land()
         │
         └── Begin landing state machine
```

**Mission Example:**
```
WP1: Approach waypoint (100m altitude)
WP2: MAV_CMD_NAV_LAND (landing point, 0m altitude)
```

### 2. AUTOLAND Mode (Automatic Return and Land)

**When to use:** Emergency landing without a mission, or repeatable landings to home.

**How it works:**
The aircraft automatically:
1. Climbs to safe altitude (if configured)
2. Loiters to align with landing direction
3. Executes glide slope landing to home

**Three Stages** (from `ArduPlane/mode_autoland.cpp`):

```
┌─────────────────────────────────────────────────────────────────┐
│                      AUTOLAND STAGES                            │
└─────────────────────────────────────────────────────────────────┘

Stage 1: CLIMB (optional, if ALND_CLIMB > 0)
    │
    │   Aircraft climbs with limited roll
    │   Purpose: Clear terrain before turning
    │
    ▼
Stage 2: LOITER
    │
    │   ┌─────────────────────────────────────┐
    │   │  Loiter at base leg waypoint        │
    │   │  • ALND_WP_DIST from home (400m)    │
    │   │  • ALND_WP_ALT above home (55m)     │
    │   │  • Direction: takeoff + ALND_DIR_OFF│
    │   └─────────────────────────────────────┘
    │
    │   Wait until: altitude reached AND lined up
    │
    ▼
Stage 3: LANDING
    │
    │   Execute standard glide slope to home
    │
    ▼
  TOUCHDOWN
```

**Direction Capture:**
- Captured after ground speed > 5 m/s during initial flight
- Uses GPS ground course (or compass if `ALND_OPTIONS` bit 0 set)
- Can be offset by `ALND_DIR_OFF` parameter

### 3. RTL with AutoLand

**When to use:** Return-to-launch with automatic landing.

**RTL_AUTOLAND Options:**

| Value | Name | Behavior |
|-------|------|----------|
| 0 | Disabled | RTL only, no landing |
| 1 | Then Land | Loiter at RTL point, then execute DO_LAND_START |
| 2 | Immediate | Jump directly to landing sequence |
| 4 | Return Path | Jump to closest mission waypoint |

**Flow** (from `ArduPlane/mode_rtl.cpp:105-139`):
```
RTL Mode entered
    │
    ├── Fly to home/rally point
    │
    ├── Loiter at RTL altitude
    │
    └── If RTL_AUTOLAND enabled:
        │
        ├── Wait for alignment (within 1m of target altitude)
        │
        └── Jump to mission landing sequence (DO_LAND_START)
```

---

## Landing Phases Deep Dive

### Landing Stage State Machine

The landing uses four stages, defined in `AP_Landing::SlopeStage`:

```
  NORMAL ──────► APPROACH ──────► PREFLARE ──────► FINAL
    │               │                │               │
    │   Setup &     │   Glide        │   Speed       │   Flare &
    │   Alignment   │   Slope        │   Reduction   │   Touchdown
    │               │                │   (optional)  │
```

### Stage 1: NORMAL

**Purpose:** Verify aircraft is aligned and ready to begin approach.

**Entry:** Landing command received.

**Exit Conditions** (ANY of these, from `AP_Landing_Slope.cpp:64-73`):
```
Transition to APPROACH when:
  • Previous command was LOITER_TO_ALT, OR
  • (waypoint_proportion >= 0) AND (heading error < 10°) AND (crosstrack error < 5m), OR
  • (waypoint_proportion > 15%) AND (heading error < 10°) AND (below prev WP altitude), OR
  • (waypoint_proportion > 50%)
```

**What's Happening:**
- L1 controller steering toward landing waypoint
- TECS maintaining cruise altitude and speed
- System verifying heading and position alignment

### Stage 2: APPROACH

**Purpose:** Descend on calculated glide slope toward landing point.

**Entry:** Transition from NORMAL.

**Exit Conditions:**
- To PREFLARE: Below `LAND_PF_ALT` or `LAND_PF_SEC` AND `LAND_PF_ARSPD > 0`
- To FINAL: Below `LAND_FLARE_ALT` or `LAND_FLARE_SEC` (if no pre-flare configured)

**What's Happening:**
- Glide slope calculated and tracked
- TECS controlling descent rate via pitch/throttle
- L1 providing lateral guidance
- Rangefinder corrections applied (if available)

**Glide Slope Calculation** (simplified pseudocode):
```
sink_height = prev_WP_altitude - landing_altitude
ground_distance = distance(prev_WP, landing_WP)
sink_rate = sink_height / (ground_distance / groundspeed)

flare_distance = groundspeed * LAND_FLARE_SEC
aim_height = LAND_FLARE_SEC * sink_rate

slope = (sink_height - aim_height) / (ground_distance - flare_distance)

target_altitude = landing_altitude + slope * distance_to_landing
```

### Stage 3: PREFLARE (Optional)

**Purpose:** Reduce airspeed before flare for slower touchdown.

**Entry:** Below `LAND_PF_ALT` or within `LAND_PF_SEC` of ground, AND `LAND_PF_ARSPD > 0`.

**Exit:** Below `LAND_FLARE_ALT` or within `LAND_FLARE_SEC` of ground.

**What's Happening:**
- Target airspeed reduced to `LAND_PF_ARSPD`
- Still descending on glide slope
- Throttle may be reduced to slow down

**When to Use Pre-flare:**
- Large/heavy aircraft that need time to slow down
- Runways with obstacles requiring slower approach
- Aircraft with high stall speeds

### Stage 4: FINAL (Flare)

**Purpose:** Arrest descent rate and touch down gently.

**Entry:** Flare triggered (see conditions below).

**Flare Trigger Conditions** (ANY of these, from `AP_Landing_Slope.cpp:76-104`):
```
1. (in APPROACH or PREFLARE) AND (height <= LAND_FLARE_ALT)
2. (in APPROACH or PREFLARE) AND (height <= sink_rate * LAND_FLARE_SEC) AND (> 50% to landing)
3. (no rangefinder) AND (passed landing point)
4. (crash detection) AND (sink_rate < 0.2 m/s) AND (not flying)
```

**What's Happening:**

| Control | Behavior |
|---------|----------|
| **Throttle** | Suppressed to 0 (cut) |
| **Pitch** | Minimum = `LAND_PITCH_DEG`, TECS controls sink rate above this |
| **Roll** | Constrained to ±`LEVEL_ROLL_LIMIT` |
| **Sink Rate** | TECS targets `TECS_LAND_SINK` (default 0.25 m/s) |

**Flare Sink Rate Blending** (simplified pseudocode):
```
# At flare entry, record current state
flare_entry_height = current_height
flare_entry_sink_rate = current_sink_rate

# As height decreases, blend to target sink rate
progress = (flare_entry_height - current_height) / flare_entry_height
progress = clamp(progress, 0, 1)

target_sink_rate = flare_entry_sink_rate * (1 - progress) + TECS_LAND_SINK * progress
```

### Complete Phase Timeline

```
Time ──────────────────────────────────────────────────────────────────►

Altitude
  │
  │   ╔═══════════╗
  │   ║  NORMAL   ║   Verify alignment
  │   ╚═════╤═════╝
  │         │
  │         ▼
  │   ╔═══════════════════════════════════╗
  │   ║           APPROACH                 ║   Descend on glide slope
  │   ║                                    ║
  │   ║   slope = (sink_height - aim) /   ║
  │   ║           (distance - flare_dist) ║
  │   ╚═══════════════════════╤═══════════╝
  │                           │
  │                           │   ← LAND_PF_ALT (if configured)
  │                     ╔═════╧═════╗
  │                     ║ PREFLARE  ║   Slow to LAND_PF_ARSPD
  │                     ╚═════╤═════╝
  │                           │
  │                           │   ← LAND_FLARE_ALT (default 3m)
  │                     ╔═════╧═════╗
  │                     ║   FINAL   ║   Throttle 0, pitch up
  │                     ╚═════╤═════╝
  │                           │
  ├───────────────────────────┴─── Ground ────────────────────────────
  │
```

---

## TECS During Landing

TECS (Total Energy Control System) manages the fundamental trade-off between potential energy (altitude) and kinetic energy (airspeed) by coordinating pitch and throttle.

### Energy Control Concept

```
Total Energy = Potential Energy + Kinetic Energy
             = m*g*h + 0.5*m*v²

TECS controls:
  • Throttle → Changes total energy (climb/descend capability)
  • Pitch    → Trades between altitude and speed
```

### Landing-Specific TECS Behavior

#### Speed Weighting Transition

During landing, TECS gradually shifts priority from speed to altitude:

```
Normal Flight:  Speed Weight = TECS_SPDWEIGHT (default 1.0)
                Both altitude and speed controlled equally

Landing:        Speed Weight = TECS_SPDWEIGHT * (1 - path_proportion)
                As aircraft approaches landing, altitude becomes priority

Flare:          Speed Weight = TECS_LAND_SPDWGT (or continues transition)
                Altitude/sink rate is primary concern
```

**Why This Matters:**
- Early approach: Maintains target airspeed for consistent glide slope
- Late approach: Prioritizes hitting the target altitude
- Flare: Focuses on sink rate control for soft touchdown

#### Time Constant Change

TECS responds faster during landing:

| Phase | Time Constant | Response |
|-------|---------------|----------|
| Cruise | `TECS_TIME_CONST` (5s) | Slower, smoother |
| Landing | `TECS_LAND_TCONST` (2s) | Faster, more responsive |

#### Flare Sink Rate Control

During flare, TECS switches from altitude tracking to sink rate control:

```
Before Flare:
  • Target = calculated glide slope altitude
  • TECS adjusts pitch to maintain altitude

During Flare:
  • Target = sink rate (TECS_LAND_SINK, default 0.25 m/s)
  • TECS adjusts pitch to achieve smooth descent
  • If past landing point: sink rate += TECS_LAND_SRC * distance_past
```

**TECS_LAND_SRC Effect:**
- If positive (e.g., 0.2): Sink rate increases 0.2 m/s per meter past landing point
- Forces faster descent if overshooting
- Helps aircraft land even if carrying excess speed

### Landing TECS Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `TECS_LAND_ARSPD` | -1 | Landing approach airspeed (-1 = use calculated) |
| `TECS_LAND_SPDWGT` | -1 | Landing speed weight (-1 = dynamic transition) |
| `TECS_LAND_SINK` | 0.25 | Target sink rate in flare (m/s) |
| `TECS_LAND_TCONST` | 2.0 | Time constant during landing (s) |
| `TECS_LAND_DAMP` | 0.5 | Pitch damping during flare |
| `TECS_LAND_PMAX` | 10 | Max pitch during flare (deg) |
| `TECS_APPR_SMAX` | 0 | Max sink rate during approach (0 = use TECS_SINK_MAX) |
| `TECS_LAND_SRC` | 0 | Sink rate change past landing point (m/s per m) |
| `TECS_FLARE_HGT` | 1.0 | Height to complete sink rate transition (m) |

---

## L1 Navigation During Landing

The L1 controller provides lateral (roll) guidance to track the approach path.

### How L1 Works

L1 uses a "look-ahead" point on the desired path to generate steering commands:

```
                    L1 Distance
            ◄───────────────────►

Aircraft ●─────────────────────── ○ L1 Point
          \                      /
           \    Crosstrack      /
            \      Error       /
             \                /
              ───────────────── Desired Path ──────────────────►
                                                      Landing Point
```

**Key Calculation** (simplified):
```
crosstrack_error = perpendicular distance from path
Nu = angle to L1 point

lateral_acceleration = K_L1 * groundspeed² / L1_distance * sin(Nu)
roll_command = atan(lateral_acceleration / gravity)
```

### Landing-Specific L1 Behavior

#### Extended Waypoint Target

During landing, L1 doesn't aim at the landing point directly. Instead, it aims at a point **200m past** the landing point:

```
                Extended Target (L1 aims here)
                           ○
                          /
                         /
    Landing Point       /
          ○────────────/
         /
        /  Approach Path
       /
      /
Previous WP
     ○
```

**Why extend 200m?**
- Prevents sudden turns if aircraft reaches landing point
- Ensures smooth path all the way through touchdown
- L1 would otherwise try to orbit around the landing point

#### Roll Constraints During Flare

Once in FINAL stage, roll is constrained:

```
if (landing_stage == FINAL):
    roll_command = constrain(roll_command, -LEVEL_ROLL_LIMIT, +LEVEL_ROLL_LIMIT)
```

Default `LEVEL_ROLL_LIMIT` = 5°, keeping wings nearly level for touchdown.

### Crosswind Correction

L1 naturally compensates for crosswind through its path-tracking algorithm:

```
Wind ───────►

Aircraft ●───────► Ground Track
          \
           \  Crab Angle
            \
             ▼ Heading
```

The aircraft crabs into the wind to maintain the ground track to the landing point. No special crosswind logic is needed - it's inherent to the L1 algorithm.

---

## Altitude Sources and Sensor Fusion

### Altitude Source Priority

During landing, altitude is determined from multiple sources in priority order:

```
┌─────────────────────────────────────────────────────────────────┐
│                    ALTITUDE SOURCE SELECTION                    │
└─────────────────────────────────────────────────────────────────┘

    1. External HAGL (MAV_CMD_SET_HAGL)
           │
           │ Available?
           ├─── YES ──► Use external height above ground
           │
           NO
           │
    2. Rangefinder
           │
           │ In range AND validated?
           ├─── YES ──► Use rangefinder height
           │
           NO
           │
    3. Terrain Database
           │
           │ Terrain data available?
           ├─── YES ──► Use terrain-corrected altitude
           │
           NO
           │
    4. Barometric Altitude (fallback)
           │
           └──────────► Use baro relative to home
```

### Rangefinder Integration

#### Validation Process

Rangefinder data goes through validation before use (from `ArduPlane/altitude.cpp:682-835`):

```
1. Attitude Check
   - Verify aircraft isn't banked excessively
   - Rangefinder must be pointing approximately at ground

2. Sample Collection
   - Collect 10 valid samples (0.2 seconds at 50Hz)
   - Track correction delta between samples

3. Stability Check
   - Correction must not change by >5% between readings
   - Large changes indicate sensor noise or terrain features

4. Range Check
   - Must be within sensor min/max range
   - Out-of-range readings are rejected

5. Consistency Check
   - Correction vs initial must not exceed 30m
   - Excessive change indicates sensor failure
```

#### Baro Drift Correction

When rangefinder detects altitude different from barometric:

```
correction = baro_altitude - rangefinder_altitude

if correction is stable:
    Apply correction to glide slope calculations

    if (correction < 0) AND (new_slope - original_slope > LAND_ABORT_DEG):
        # Aircraft is too high, slope would be dangerously steep
        Request go-around
```

### Rangefinder State Variables

| Variable | Purpose |
|----------|---------|
| `in_range` | Rangefinder reading is valid |
| `in_use` | Actively using rangefinder for landing |
| `correction` | Current altitude correction (baro - rangefinder) |
| `last_stable_correction` | Previous stable correction value |
| `height_estimate` | Corrected height above ground |

---

## Safety Systems

### Go-Around (Abort) Triggers

The aircraft will automatically abort landing under these conditions:

```
┌─────────────────────────────────────────────────────────────────┐
│                     GO-AROUND TRIGGERS                          │
└─────────────────────────────────────────────────────────────────┘

1. SLOPE TOO STEEP (Rangefinder)
   │
   │   Condition: Rangefinder detects aircraft too high
   │              AND new slope exceeds original + LAND_ABORT_DEG
   │
   │   Parameter: LAND_ABORT_DEG (default 0 = disabled)
   │
   └── Automatic abort, logs altitude offset for next attempt

2. PILOT THROTTLE ABORT
   │
   │   Condition: Throttle input >= 90%
   │              AND LAND_ABORT_THR = 1
   │
   │   Parameter: LAND_ABORT_THR (default 0 = disabled)
   │
   └── Pilot can abort by pushing throttle

3. LANDING GEAR NOT DEPLOYED
   │
   │   Condition: Aircraft entering flare
   │              AND landing gear not in down position
   │
   │   Automatic abort with "Landing gear not deployed" message
   │
   └── Requires AP_LANDINGGEAR_ENABLED

4. GCS COMMAND
   │
   │   MAV_CMD_DO_GO_AROUND received
   │
   └── Ground station commanded abort

5. SCRIPT/LUA COMMAND
   │
   │   landing:request_go_around() called
   │
   └── Scripted abort
```

### Go-Around Sequence

When abort is triggered:

```
flight_stage = ABORT_LANDING
        │
        ▼
┌───────────────────────────────────────┐
│  1. Apply full climb throttle         │
│  2. Target TAKEOFF_ALT altitude       │
│  3. Maintain current heading          │
└───────────────────────────────────────┘
        │
        ▼
    Once above abort altitude
        │
        ├── If landing sequence available → Restart landing
        │
        └── Otherwise → RTL or continue mission
```

### Crash Detection

The system distinguishes between hard landings and crashes (from `ArduPlane/is_flying.cpp:211-328`):

**Hard Landing:** (near landing waypoint)
- Within 75m of landing point
- Attitude > 60° pitch or roll
- Logged as `MAV_SEVERITY_CRITICAL`

**Crash:** (away from landing waypoint)
- More than 75m from landing point
- Attitude > 60° pitch or roll
- Logged as `MAV_SEVERITY_EMERGENCY`
- May trigger auto-disarm

### Sensor Failure Handling

| Sensor | Failure Response |
|--------|------------------|
| **GPS Lost** | Use airspeed for flying detection; landing continues on last known position |
| **Airspeed Failed** | Crash detection disabled; landing continues with GPS groundspeed |
| **Rangefinder Failed** | Fall back to barometric altitude; flare at configured altitude |
| **Both GPS & Airspeed** | Crash detection disabled; may false-trigger landing complete |

### Touchdown Detection

The system doesn't directly detect ground contact. Instead, it infers landing complete via:

1. **Flying Probability** drops below threshold
   - Uses airspeed, groundspeed, acceleration, sink rate
   - Low-pass filtered with ~3 second time constant

2. **After `LAND_DISARMDELAY`** seconds in FINAL stage
   - Default 20 seconds
   - Auto-disarm if armed

---

## Wind Handling

### Wind Estimation

Wind is estimated by the EKF (Extended Kalman Filter) using:
- Difference between airspeed and GPS groundspeed
- Continuous refinement during flight

```
wind_vector = groundspeed_vector - airspeed_vector

head_wind = wind component in direction of flight (positive = headwind)
```

### Headwind Compensation

During landing approach, target airspeed is increased to compensate for headwind:

```
landing_airspeed = base_airspeed + (head_wind * LAND_WIND_COMP / 100)

Constraints:
  - Never less than base_airspeed
  - Never more than AIRSPEED_MAX (if LAND_OPTIONS bit 1 set)
  - Otherwise capped at AIRSPEED_CRUISE
```

**Example:**
```
Base landing airspeed: 15 m/s
Headwind: 10 m/s
LAND_WIND_COMP: 50%

Compensated airspeed = 15 + (10 * 0.5) = 20 m/s
```

**Why compensate?**
- Headwind reduces groundspeed
- Lower groundspeed = steeper glide slope angle
- Adding airspeed maintains consistent ground approach angle

### Crosswind Handling

Crosswind is handled automatically by L1:
- Aircraft crabs into wind to maintain ground track
- No explicit crosswind parameter needed
- Works throughout approach and flare

---

## Parameters Reference

### Approach Parameters

| Parameter | Default | Range | Description | Effect if Too High | Effect if Too Low |
|-----------|---------|-------|-------------|-------------------|-------------------|
| `LAND_FLARE_ALT` | 3 m | 0-30 | Altitude to start flare | Flares too early, floats | Flares too late, hard landing |
| `LAND_FLARE_SEC` | 2 s | 0-10 | Time-to-ground flare trigger | Flares too early | Flares too late |
| `LAND_PF_ALT` | 10 m | 0-30 | Pre-flare altitude | Slows too early | Slows too late |
| `LAND_PF_SEC` | 6 s | 0-10 | Pre-flare time trigger | Slows too early | Slows too late |
| `LAND_PF_ARSPD` | 0 | 0-30 | Pre-flare airspeed (0=disabled) | Approaches too slow | Approaches too fast |
| `LAND_WIND_COMP` | 50% | 0-100 | Headwind compensation | Over-speeds in headwind | Under-speeds in headwind |

### Flare Parameters

| Parameter | Default | Range | Description | Effect if Too High | Effect if Too Low |
|-----------|---------|-------|-------------|-------------------|-------------------|
| `LAND_PITCH_DEG` | 0° | -20 to 20 | Minimum pitch during flare | Stalls, drops hard | Doesn't arrest sink rate |
| `TECS_LAND_SINK` | 0.25 m/s | 0-2 | Target sink rate in flare | Floats, overshoots | Hard landing |
| `TECS_LAND_DAMP` | 0.5 | 0-1 | Pitch damping in flare | Sluggish pitch response | Pitch oscillations |
| `TECS_LAND_PMAX` | 10° | 0-30 | Max pitch during flare | May stall | Poor sink rate control |
| `TECS_LAND_SRC` | 0 | -5 to 5 | Sink rate change past landing | Dives if past WP | Floats if past WP |
| `LEVEL_ROLL_LIMIT` | 5° | 0-45 | Max roll during flare | Wing tip may strike | Poor tracking |

### Safety Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `LAND_ABORT_DEG` | 0° | 0-90 | Auto-abort slope threshold (0=disabled) |
| `LAND_ABORT_THR` | 0 | 0-1 | Allow throttle abort (0=disabled, 1=enabled) |
| `LAND_SLOPE_RCALC` | 2 m | 0-5 | Rangefinder slope recalc threshold |
| `LAND_DISARMDELAY` | 20 s | 0-127 | Auto-disarm delay after landing |

### AUTOLAND Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `ALND_WP_ALT` | 55 m | 0-200 | Final approach altitude above home |
| `ALND_WP_DIST` | 400 m | 0-700 | Final approach distance from home |
| `ALND_DIR_OFF` | 0° | -360 to 360 | Landing direction offset from takeoff |
| `ALND_OPTIONS` | 0 | Bitmask | Bit 0: capture heading at arm |
| `ALND_CLIMB` | 0 m | 0-100 | Min terrain climb before turn |

### TECS Landing Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `TECS_LAND_ARSPD` | -1 | -1 to 50 | Landing airspeed (-1=calculated) |
| `TECS_LAND_SPDWGT` | -1 | -1 to 2 | Speed weight (-1=dynamic) |
| `TECS_LAND_TCONST` | 2 s | 0.1-10 | Landing time constant |
| `TECS_APPR_SMAX` | 0 | 0-20 | Max approach sink rate (0=use SINK_MAX) |
| `TECS_FLARE_HGT` | 1 m | 0-20 | Flare transition completion height |

---

## Tuning Guide

### Step-by-Step Tuning Process

#### Step 1: Verify Basic Flight Tuning

Before tuning landing, ensure:
- [ ] TECS is tuned for level flight (altitude and speed hold work)
- [ ] L1 navigation tracks waypoints accurately
- [ ] Airspeed sensor is calibrated
- [ ] Stall speed (`ARSPD_FBW_MIN`) is correctly set

#### Step 2: Set Conservative Initial Values

Start with these safe defaults:
```
LAND_FLARE_ALT = 5        # Higher than default for margin
LAND_FLARE_SEC = 3        # More time to flare
LAND_PITCH_DEG = 2        # Slight nose-up
TECS_LAND_SINK = 0.5      # Gentle descent
LAND_PF_ARSPD = 0         # Disable pre-flare initially
```

#### Step 3: Test Approach (No Flare)

Set `LAND_FLARE_ALT = 0` temporarily and observe:
- Does aircraft track glide slope smoothly?
- Is airspeed stable during approach?
- Is descent rate reasonable (typically 1-3 m/s)?

**Fix issues before proceeding:**
- Oscillating altitude → Increase `TECS_LAND_TCONST`
- Too fast → Decrease approach airspeed or increase `LAND_WIND_COMP`
- Wandering laterally → Check L1 tuning

#### Step 4: Tune Flare Altitude

With manual takeover ready:
1. Set `LAND_FLARE_ALT = 5`
2. Observe flare behavior
3. Adjust based on results:

| Observation | Adjustment |
|-------------|------------|
| Flares too high, floats | Decrease `LAND_FLARE_ALT` |
| Flares too low, hard landing | Increase `LAND_FLARE_ALT` |
| Flare timing good but hard | Increase `LAND_PITCH_DEG` or decrease `TECS_LAND_SINK` |
| Flare timing good but floats | Decrease `LAND_PITCH_DEG` or increase `TECS_LAND_SINK` |

#### Step 5: Add Rangefinder (Optional)

If using a rangefinder:
1. Verify rangefinder works in flight (`RNGFND1_TYPE`, etc.)
2. Set `RNGFND_LANDING = 1`
3. Reduce `LAND_FLARE_ALT` since rangefinder is more accurate than baro

#### Step 6: Tune Pre-flare (Optional)

For aircraft that need to slow down before flare:
1. Set `LAND_PF_ARSPD` to desired approach speed (e.g., 1.2 × stall speed)
2. Set `LAND_PF_ALT` to altitude where slowdown should begin
3. Verify aircraft has time to slow down before flare

### Aircraft-Specific Recommendations

#### Small Foam Aircraft (< 2 kg)

```
LAND_FLARE_ALT = 2-3
LAND_FLARE_SEC = 1.5
LAND_PITCH_DEG = 0-3
TECS_LAND_SINK = 0.3
```

Light aircraft respond quickly; minimal flare needed.

#### Medium Aircraft (2-10 kg)

```
LAND_FLARE_ALT = 3-5
LAND_FLARE_SEC = 2
LAND_PITCH_DEG = 3-5
TECS_LAND_SINK = 0.25
LAND_PF_ARSPD = stall_speed * 1.3
```

Standard values work well; pre-flare helps heavier models.

#### Large/Heavy Aircraft (> 10 kg)

```
LAND_FLARE_ALT = 5-8
LAND_FLARE_SEC = 3
LAND_PITCH_DEG = 5-8
TECS_LAND_SINK = 0.2
LAND_PF_ARSPD = stall_speed * 1.3
LAND_PF_ALT = 15
```

Need more time and distance to slow down and flare.

---

## Troubleshooting

### Symptom → Solution Table

| Symptom | Likely Cause | Parameters to Adjust |
|---------|--------------|---------------------|
| **Landing too fast** | Approach airspeed too high | Decrease `TECS_LAND_ARSPD`, increase `LAND_WIND_COMP` |
| **Landing too slow / stalling** | Approach airspeed too low | Increase `TECS_LAND_ARSPD`, decrease `LAND_PF_ARSPD` |
| **Flaring too early** | Flare triggers too high | Decrease `LAND_FLARE_ALT`, `LAND_FLARE_SEC` |
| **Flaring too late / hard landing** | Flare triggers too low | Increase `LAND_FLARE_ALT`, `LAND_FLARE_SEC` |
| **Floating after flare** | Sink rate too low | Increase `TECS_LAND_SINK`, decrease `LAND_PITCH_DEG` |
| **Dropping after flare** | Sink rate too high or pitch too low | Decrease `TECS_LAND_SINK`, increase `LAND_PITCH_DEG` |
| **Oscillating on approach** | TECS too aggressive | Increase `TECS_LAND_TCONST`, decrease `TECS_LAND_DAMP` |
| **Not tracking runway** | L1 tuning or waypoint issue | Check `NAVL1_PERIOD`, verify waypoint positions |
| **Aborting unexpectedly** | Rangefinder slope abort | Decrease `LAND_ABORT_DEG` or set to 0 |
| **Not flaring at all** | Flare altitude below ground level | Check landing waypoint altitude, increase `LAND_FLARE_ALT` |
| **Overshooting runway** | Too fast, wind, or late flare | Increase `LAND_WIND_COMP`, use pre-flare, check airspeed |
| **Undershooting runway** | Tailwind or too slow | Decrease `LAND_WIND_COMP` (won't add speed in tailwind) |
| **Flaring at wrong altitude** | Landing waypoint altitude incorrect | Verify waypoint alt is 0 (relative) or correct AMSL; use rangefinder |
| **Stalling high above ground** | Landing waypoint set too high | Check waypoint altitude; see [Wrong Waypoint Altitude](#what-happens-if-landing-waypoint-altitude-is-wrong) |
| **Flying into ground (no flare)** | Landing waypoint set below ground | Check waypoint altitude; use rangefinder for safety |

### Log Analysis Tips

Key log messages to examine:

| Log Field | What to Look For |
|-----------|------------------|
| `ATT.Pitch` | Should increase during flare |
| `CTUN.ThO` | Should drop to 0 at flare |
| `CTUN.NavPitch` | TECS pitch demand |
| `TECS.hDem` | Height demand vs actual |
| `TECS.sDem` | Speed demand |
| `LAND.stage` | Stage transitions (0→1→2→3) |
| `LAND.flare` | 1 when flaring |
| `RFND.Dist` | Rangefinder distance |

**Good landing signature:**
1. Steady descent during APPROACH
2. Pitch increases at flare
3. Throttle drops to 0 at flare
4. Sink rate decreases during flare
5. Smooth touchdown

### Common Mistakes

1. **Setting `LAND_FLARE_ALT` too low**
   - Aircraft doesn't have time to arrest sink rate
   - Result: Hard landing

2. **Not accounting for baro drift**
   - Landing point altitude may be wrong
   - Solution: Use rangefinder or conservative `LAND_FLARE_ALT`

3. **Pre-flare airspeed below stall speed**
   - Aircraft stalls before flare
   - Solution: `LAND_PF_ARSPD` should be > 1.2 × stall speed

4. **Landing into tailwind**
   - Higher groundspeed, steeper approach
   - Solution: Plan missions with headwind landings

5. **Rangefinder mounted at angle**
   - Gives incorrect height readings when banked
   - Solution: Mount pointing straight down, verify attitude correction

### What Happens If Landing Waypoint Altitude Is Wrong?

This is a critical edge case that can cause crashes. The landing system calculates flare height **relative to the landing waypoint altitude, NOT actual ground level**.

#### How Height Is Calculated

From `ArduPlane/altitude.cpp:598-615` and `930-958`:

```
height = (current_altitude - home_altitude) - landing_waypoint_altitude

If rangefinder available:
    height -= rangefinder_correction   // Adjusts to actual AGL
```

**Key insight:** Without a rangefinder, the system has no way to know if the landing waypoint altitude is correct.

#### Scenario 1: Landing Waypoint Set Too HIGH (e.g., 50m above ground)

```
                        ┌─── Flare triggers here (LAND_FLARE_ALT above waypoint)
                        │    = 53m above actual ground!
                        ▼
        ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─   Landing waypoint (incorrectly at 50m AGL)
                        │
                        │   Aircraft in "flare" mode:
                        │   - Throttle CUT (0%)
                        │   - Pitching up
                        │   - Stalling/gliding down from 50m
                        │
        ═══════════════════════  Actual ground (0m)

        Result: Aircraft stalls at 50m and falls. Crash or very hard landing.
```

**What happens:**
1. Aircraft approaches the 50m-high waypoint normally
2. At 53m AGL (3m above waypoint), flare triggers
3. Throttle cuts to 0, aircraft pitches up
4. Aircraft is now 50m above ground with no power
5. Aircraft stalls and drops

#### Scenario 2: Landing Waypoint Set Too LOW (e.g., 10m below ground)

```
        ═══════════════════════  Actual ground (0m)
                        │
                        │   Aircraft tries to descend to reach waypoint
                        │   (impossible - waypoint is underground)
                        │
                        ▼
        ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─   Landing waypoint (incorrectly at -10m)
```

**What happens:**
1. Glide slope aims underground - aircraft descends aggressively
2. Aircraft reaches actual ground before flare altitude
3. **Safety backup:** When `wp_proportion >= 1` (passed waypoint), flare triggers regardless of altitude
4. But this is usually too late - aircraft has already hit the ground

**The safety check** (from `AP_Landing_Slope.cpp:103`):
```cpp
if (!rangefinder_state_in_range && wp_proportion >= 1) {
    // Trigger flare - we've passed the landing point
}
```

This prevents infinite descent but doesn't prevent ground impact.

#### How Rangefinder Helps

With a rangefinder, `get_landing_height()` applies a correction:

```
height -= rangefinder_correction()
// Where: rangefinder_correction = baro_altitude - rangefinder_altitude
```

This converts the height to **actual AGL** when the rangefinder is in range:

| Waypoint Error | Without Rangefinder | With Rangefinder |
|---------------|---------------------|------------------|
| **50m too high** | Flares at 53m AGL, stalls, crashes | Glide slope shallow, but flares at correct 3m AGL |
| **10m too low** | Flies into ground, late emergency flare | Steep approach (may auto-abort), flares at correct 3m AGL |
| **Correct (0m)** | Works correctly | Works correctly |

**Important:** Even with a rangefinder, the glide slope is still calculated from the waypoint altitudes. So:
- Waypoint too high → Shallow approach (may overshoot)
- Waypoint too low → Steep approach (may trigger `LAND_ABORT_DEG` go-around)

#### How To Avoid This Problem

1. **Always verify landing waypoint altitude**
   - Should typically be 0 for relative altitude (home-referenced)
   - Or actual ground elevation if using AMSL (absolute)

2. **Use a rangefinder for landing**
   - Set `RNGFND_LANDING = 1`
   - Provides ground-truth altitude for flare timing
   - Enables slope recalculation and auto-abort if needed

3. **Enable slope abort protection**
   - Set `LAND_ABORT_DEG` to a reasonable value (e.g., 15°)
   - If rangefinder detects the required slope is too steep, aircraft will go around

4. **Check mission before flight**
   - Verify landing waypoint altitude in ground station
   - Compare to actual terrain elevation at landing site

5. **Use terrain data if available**
   - Enable terrain following for approach
   - System can use terrain database to validate altitudes

#### Detecting This Problem

**In logs, look for:**
- `LAND.height` much higher than expected at flare
- Large `rangefinder_correction` values during approach
- Flare message showing unexpected altitude: `"Flare 53.0m sink=2.5 speed=15.0"`

**Warning signs during flight:**
- Aircraft flares much higher than expected
- Aircraft doesn't flare and flies into ground
- Unexpected go-around triggered by steep slope

---

## Source Files Reference

### Core Landing Logic

| File | Key Functions | Lines |
|------|---------------|-------|
| `libraries/AP_Landing/AP_Landing.h` | Class definition, enums | All |
| `libraries/AP_Landing/AP_Landing.cpp` | `do_land()`, `verify_land()`, parameters | 27-182 |
| `libraries/AP_Landing/AP_Landing_Slope.cpp` | `type_slope_verify_land()`, flare logic | 56-188 |

### TECS Integration

| File | Key Functions | Lines |
|------|---------------|-------|
| `libraries/AP_TECS/AP_TECS.cpp` | `_update_height_demand()` (flare) | 536-644 |
| `libraries/AP_TECS/AP_TECS.cpp` | Speed weighting transition | 986-1013 |
| `libraries/AP_TECS/AP_TECS.cpp` | Pitch limits during landing | 1496-1537 |

### Navigation

| File | Key Functions | Lines |
|------|---------------|-------|
| `libraries/AP_L1_Control/AP_L1_Control.cpp` | `update_waypoint()` | 206-347 |
| `libraries/AP_L1_Control/AP_L1_Control.cpp` | `nav_roll_cd()` | 79-95 |

### Altitude and Sensors

| File | Key Functions | Lines |
|------|---------------|-------|
| `ArduPlane/altitude.cpp` | `relative_ground_altitude()` | 111-157 |
| `ArduPlane/altitude.cpp` | `rangefinder_height_update()` | 682-835 |
| `ArduPlane/altitude.cpp` | `get_landing_height()` | 930-958 |

### Flight Modes

| File | Key Functions | Lines |
|------|---------------|-------|
| `ArduPlane/mode_auto.cpp` | Landing control integration | 93-105 |
| `ArduPlane/mode_autoland.cpp` | AUTOLAND stages | 219-254 |
| `ArduPlane/mode_rtl.cpp` | RTL autoland | 105-139 |

### Safety

| File | Key Functions | Lines |
|------|---------------|-------|
| `ArduPlane/is_flying.cpp` | `crash_detection_update()` | 211-328 |
| `ArduPlane/commands_logic.cpp` | `do_land()` | 405-433 |
| `ArduPlane/Plane.cpp` | Flight stage management | 680-691 |

---

## Design Summary

The ArduPlane landing system provides:

1. **Modular Architecture**: Separate concerns (AP_Landing, TECS, L1) for maintainability
2. **Multiple Entry Points**: AUTO mission, AUTOLAND mode, RTL integration
3. **Robust Flare Triggering**: Both altitude and time-based triggers
4. **Sensor Fusion**: Prioritized altitude sources with rangefinder correction
5. **Safety Features**: Auto-abort, crash detection, gear checks
6. **Wind Compensation**: Automatic headwind adjustment
7. **Tunable Behavior**: Extensive parameters for different aircraft types

The system is designed to handle real-world conditions including sensor drift, wind, and terrain variations while providing operators with the controls needed to achieve consistent, safe landings.

---

## Appendix: Glossary of Terms

This glossary defines technical terms, acronyms, and jargon used throughout this document.

### Flight Control Terms

| Term | Definition |
|------|------------|
| **Pitch** | Rotation around the lateral (side-to-side) axis. Nose up = positive pitch, nose down = negative pitch. Controls climb/descent. |
| **Roll** | Rotation around the longitudinal (front-to-back) axis. Right wing down = positive roll. Controls turning. |
| **Yaw** | Rotation around the vertical axis. Nose right = positive yaw. Usually controlled by rudder. |
| **Throttle** | Engine power setting, typically 0-100%. Controls total energy (speed + altitude capability). |
| **Elevator** | Control surface on the tail that controls pitch. Deflects up to pitch nose up. |
| **Aileron** | Control surfaces on the wings that control roll. Deflect differentially (one up, one down). |
| **Flaps** | Surfaces on the trailing edge of wings that increase lift and drag. Extended during landing for slower approach. |

### Navigation & Guidance Terms

| Term | Definition |
|------|------------|
| **L1 Controller** | A lateral (sideways) navigation algorithm that steers the aircraft toward a "look-ahead" point on the desired path. Named after its key parameter, the L1 distance. Generates roll commands to track waypoints and correct for wind. |
| **Crosstrack Error** | The perpendicular distance between the aircraft's current position and the desired flight path. L1 works to minimize this. |
| **Waypoint (WP)** | A geographic coordinate (latitude, longitude, altitude) that defines a point the aircraft should fly to or through. |
| **Ground Track** | The path the aircraft actually follows over the ground. May differ from heading due to wind. |
| **Heading** | The direction the aircraft's nose is pointing (compass direction). |
| **Bearing** | The compass direction from one point to another (e.g., bearing to waypoint). |
| **Crab Angle** | The difference between heading and ground track when flying in crosswind. Aircraft "crabs" sideways to maintain straight ground track. |

### Energy & Speed Terms

| Term | Definition |
|------|------------|
| **TECS** | Total Energy Control System. A flight controller that manages the trade-off between altitude (potential energy) and airspeed (kinetic energy) by coordinating pitch and throttle together rather than separately. |
| **Airspeed** | Speed of the aircraft relative to the air mass. What the wings "feel." Critical for lift and stall avoidance. |
| **Groundspeed** | Speed of the aircraft relative to the ground. Airspeed ± wind speed. |
| **Stall Speed** | Minimum airspeed at which the wings can generate enough lift. Below this, the aircraft loses lift and drops. |
| **Sink Rate** | Rate of descent, typically in meters per second. Positive = descending. |
| **Glide Slope** | The angle or path of descent from approach altitude to the landing point. Steeper slope = faster descent. |
| **Indicated Airspeed (IAS)** | Airspeed as shown on instruments, uncorrected for altitude/temperature. What pilots typically use. |
| **True Airspeed (TAS)** | Actual speed through the air, corrected for altitude and temperature. Higher than IAS at altitude. |

### Altitude Terms

| Term | Definition |
|------|------------|
| **AGL** | Above Ground Level. Height above the terrain directly below the aircraft. What matters for landing. |
| **AMSL** | Above Mean Sea Level. Absolute altitude reference. Used for airspace and terrain clearance. |
| **Barometric Altitude (Baro)** | Altitude calculated from air pressure. Drifts with weather changes. Primary altitude source but can be inaccurate near ground. |
| **HAGL** | Height Above Ground Level. Same as AGL but emphasizes it's a measured/calculated value. |
| **Terrain Altitude** | Ground elevation at a location, from terrain database. Used to calculate AGL from AMSL. |
| **Rangefinder** | A sensor (usually laser or ultrasonic) that directly measures distance to the ground. More accurate than baro for low-altitude operations. |

### Landing-Specific Terms

| Term | Definition |
|------|------------|
| **Flare** | The final phase of landing where the aircraft pitches up to reduce sink rate just before touchdown. Arrests descent for soft landing. |
| **Pre-flare** | Optional phase before flare where airspeed is reduced. Gives heavy aircraft time to slow down. |
| **Touchdown** | The moment the wheels contact the ground. |
| **Go-Around** | Aborting a landing attempt and climbing back to altitude to try again. Safety maneuver. |
| **Approach** | The descent phase from cruise altitude toward the landing point, following the glide slope. |
| **Final Approach** | The last straight segment before landing, aligned with the runway. |
| **Base Leg** | Flight segment perpendicular to the runway, before turning onto final approach. |
| **Abort Altitude** | The altitude the aircraft climbs to during a go-around before attempting another landing. |
| **Disarm** | Shutting down the motors and flight controller after landing. Safety state. |

### Sensor Terms

| Term | Definition |
|------|------------|
| **EKF** | Extended Kalman Filter. A mathematical algorithm that fuses data from multiple sensors (GPS, IMU, baro, compass) to estimate the aircraft's position, velocity, and attitude. The "brain" that figures out where the aircraft is. |
| **IMU** | Inertial Measurement Unit. Contains accelerometers and gyroscopes to measure acceleration and rotation rates. |
| **GPS** | Global Positioning System. Satellite-based navigation providing position and velocity. |
| **Compass / Magnetometer** | Sensor that measures Earth's magnetic field to determine heading. |
| **Pitot Tube** | A probe that measures dynamic air pressure to calculate airspeed. |
| **Lidar** | Light Detection and Ranging. A type of rangefinder using laser pulses to measure distance. |

### ArduPilot-Specific Terms

| Term | Definition |
|------|------------|
| **ArduPilot** | Open-source autopilot software that runs on flight controllers. Supports planes, copters, rovers, boats, and submarines. |
| **ArduPlane** | The fixed-wing aircraft variant of ArduPilot. What this document describes. |
| **Mission** | A sequence of commands (waypoints, actions) that the aircraft executes autonomously. |
| **GCS** | Ground Control Station. Software (like Mission Planner or QGroundControl) used to plan missions, monitor flights, and configure parameters. |
| **MAVLink** | Micro Air Vehicle Link. Communication protocol between the aircraft and GCS. |
| **Parameter** | A configurable setting stored in the flight controller. Changed via GCS or command line. |
| **Flight Mode** | The current control mode (AUTO, MANUAL, RTL, etc.) determining how the aircraft behaves. |
| **AUTO Mode** | Flight mode where the aircraft follows a pre-programmed mission. |
| **RTL** | Return To Launch. Flight mode where the aircraft automatically returns to its takeoff point. |
| **AUTOLAND** | A flight mode specifically for autonomous landing without a pre-programmed mission. |
| **Failsafe** | Automatic safety action taken when something goes wrong (e.g., lost radio link triggers RTL). |
| **Servo** | An actuator that moves control surfaces (elevator, ailerons, rudder) based on autopilot commands. |
| **PWM** | Pulse Width Modulation. The signal format used to control servos and ESCs. |
| **ESC** | Electronic Speed Controller. Controls motor speed for electric aircraft. |

### Command Terms

| Term | Definition |
|------|------------|
| **MAV_CMD_NAV_LAND** | Mission command that tells the aircraft to land at a specified location. |
| **MAV_CMD_DO_LAND_START** | Mission command marking the beginning of a landing sequence. Used with RTL_AUTOLAND. |
| **DO_GO_AROUND** | Command to abort landing and climb back to altitude. |
| **LOITER_TO_ALT** | Command to circle at current location until reaching a target altitude. Often used before landing approach. |

### Mathematical/Control Terms

| Term | Definition |
|------|------------|
| **PID Controller** | Proportional-Integral-Derivative controller. Common feedback control algorithm that adjusts output based on error, accumulated error, and rate of change of error. |
| **Time Constant** | How quickly a system responds to changes. Smaller = faster response. Measured in seconds. |
| **Damping** | Resistance to oscillation. Higher damping = less overshoot but slower response. |
| **Gain** | Multiplier that determines how strongly a controller responds to error. Higher gain = stronger response. |
| **Integrator** | Part of a controller that accumulates error over time. Eliminates steady-state errors but can cause overshoot. |
| **Low-pass Filter** | Smooths out rapid changes in a signal, keeping only slow variations. Reduces noise. |
| **Setpoint** | The target value a controller is trying to achieve (e.g., target altitude, target airspeed). |
| **Constraint** | A limit placed on a value (e.g., max roll angle, min airspeed). |

### Units

| Unit | Meaning |
|------|---------|
| **m** | Meters (distance/altitude) |
| **m/s** | Meters per second (speed, sink rate) |
| **cm** | Centimeters (often used internally for altitude) |
| **deg** or **°** | Degrees (angles) |
| **rad** | Radians (angles, used internally; 180° = π radians) |
| **%** | Percent (throttle, flap position) |
| **s** | Seconds (time) |
| **ms** | Milliseconds (1/1000 of a second) |
| **Hz** | Hertz (frequency, cycles per second) |
| **cd** | Centidegrees (1/100 of a degree, used internally) |

### Acronyms Quick Reference

| Acronym | Expansion |
|---------|-----------|
| AGL | Above Ground Level |
| AHRS | Attitude and Heading Reference System |
| AMSL | Above Mean Sea Level |
| EKF | Extended Kalman Filter |
| ESC | Electronic Speed Controller |
| GCS | Ground Control Station |
| GPS | Global Positioning System |
| HAGL | Height Above Ground Level |
| IAS | Indicated Airspeed |
| IMU | Inertial Measurement Unit |
| L1 | L1 Navigation Controller |
| MAV | Micro Air Vehicle |
| PWM | Pulse Width Modulation |
| RTL | Return To Launch |
| TAS | True Airspeed |
| TECS | Total Energy Control System |
| WP | Waypoint |
