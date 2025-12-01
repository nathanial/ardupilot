# MAVLink Missions in ArduPilot (ArduPlane Focus)

This document provides a comprehensive analysis of how MAVLink missions are handled in ArduPilot, with a focus on ArduPlane fixed-wing aircraft. It covers the architecture, protocols, storage, and execution of missions.

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [System Architecture](#system-architecture)
3. [Mission Upload Protocol](#mission-upload-protocol)
4. [Mission Storage](#mission-storage)
5. [Mission Command Types](#mission-command-types)
6. [Mission Execution (ArduPlane)](#mission-execution-arduplane)
7. [ArduPlane-Specific Commands](#arduplane-specific-commands)
8. [Mission State Management](#mission-state-management)
9. [Practical Guide for Operators](#practical-guide-for-operators)
10. [Troubleshooting](#troubleshooting)
11. [Source Files Reference](#source-files-reference)
12. [Appendix: Glossary of Terms](#appendix-glossary-of-terms)

---

## Executive Summary

### How Missions Work in 30 Seconds

A MAVLink mission is a sequence of commands stored on the vehicle that tell it where to go and what to do. The process works like this:

1. **Upload**: Ground Control Station (GCS) sends mission items via MAVLink protocol
2. **Storage**: Vehicle stores commands in EEPROM/Flash (15 bytes per command)
3. **Execution**: In AUTO mode, vehicle executes commands sequentially
4. **Completion**: Mission ends when all commands complete, or RTL if configured

### Key Systems Involved

| System | Role |
|--------|------|
| **AP_Mission** | Mission storage, state machine, command advancement |
| **MissionItemProtocol** | MAVLink protocol handling for upload/download |
| **commands_logic.cpp** | ArduPlane-specific command execution (`do_*`/`verify_*`) |
| **Mode_Auto** | Flight mode that runs the mission |

### How to Use This Document

- **Operators**: Start with [Practical Guide](#practical-guide-for-operators) and [Troubleshooting](#troubleshooting)
- **Developers**: Read [System Architecture](#system-architecture) and [Mission Execution](#mission-execution-arduplane)
- **Protocol Integration**: See [Mission Upload Protocol](#mission-upload-protocol)

---

## System Architecture

### Component Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                       GROUND CONTROL STATION (GCS)                          │
│              Mission Planner / QGroundControl / MAVProxy                    │
└────────────────────────────────┬────────────────────────────────────────────┘
                                 │
                                 │ MAVLink Protocol
                                 │ (MISSION_COUNT, MISSION_ITEM_INT, etc.)
                                 ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         GCS_MAVLink Library                                 │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    MissionItemProtocol                               │   │
│  │  - handle_mission_count()      - Receives upload count               │   │
│  │  - handle_mission_item()       - Receives individual items           │   │
│  │  - queued_request_send()       - Sends MISSION_REQUEST               │   │
│  │  - transfer_is_complete()      - Finalizes upload                    │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
└────────────────────────────────┬────────────────────────────────────────────┘
                                 │
                                 │ mavlink_int_to_mission_cmd()
                                 │ (Convert MAVLink → Internal Format)
                                 ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           AP_Mission Library                                │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    Mission Storage & Management                      │   │
│  │  - add_cmd() / replace_cmd()   - Store commands to EEPROM           │   │
│  │  - read_cmd_from_storage()     - Load commands from EEPROM          │   │
│  │  - start() / stop() / resume() - Mission control                    │   │
│  │  - update()                    - Main execution loop (10Hz+)        │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                 │                                           │
│                    Registered Callbacks                                     │
│                    (Vehicle-Specific)                                       │
└────────────────────────────────┬────────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                     ArduPlane (commands_logic.cpp)                          │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  start_command_callback()    verify_command_callback()               │   │
│  │         │                           │                                │   │
│  │         ▼                           ▼                                │   │
│  │  ┌─────────────┐             ┌─────────────┐                        │   │
│  │  │ do_takeoff  │             │verify_takeoff│  (Called repeatedly   │   │
│  │  │ do_nav_wp   │             │verify_nav_wp │   until true)         │   │
│  │  │ do_land     │             │verify_land   │                        │   │
│  │  │ do_loiter_* │             │verify_loiter*│                        │   │
│  │  │ ...         │             │...           │                        │   │
│  │  └─────────────┘             └─────────────┘                        │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Core Components

#### 1. AP_Mission Library

**Location**: `libraries/AP_Mission/`

The central mission management library that:
- Stores and retrieves mission commands from persistent storage
- Manages the mission state machine (STOPPED, RUNNING, COMPLETE)
- Advances through navigation and do commands
- Handles DO_JUMP loops and counters
- Calls vehicle-specific callbacks for command execution

**Key Files:**

| File | Purpose |
|------|---------|
| `AP_Mission.h` | Class definition, data structures, enums |
| `AP_Mission.cpp` | Core logic: `update()`, storage, advancement |
| `AP_Mission_Commands.cpp` | Common command handlers (camera, gripper, etc.) |

#### 2. MissionItemProtocol

**Location**: `libraries/GCS_MAVLink/`

Implements the MAVLink mission upload/download protocol:
- Polymorphic base class supporting multiple mission types
- Handles request-response state machine
- Manages timeouts and retries
- Validates incoming mission items

**Protocol Types:**
- `MAV_MISSION_TYPE_MISSION` (0) - Waypoint missions
- `MAV_MISSION_TYPE_FENCE` (1) - Geofence
- `MAV_MISSION_TYPE_RALLY` (2) - Rally points

#### 3. ArduPlane Integration

**Location**: `ArduPlane/`

ArduPlane registers three callbacks with AP_Mission:

```cpp
// From ArduPlane/Plane.h:680-683
AP_Mission mission{
    FUNCTOR_BIND_MEMBER(&Plane::start_command_callback, bool, const AP_Mission::Mission_Command &),
    FUNCTOR_BIND_MEMBER(&Plane::verify_command_callback, bool, const AP_Mission::Mission_Command &),
    FUNCTOR_BIND_MEMBER(&Plane::exit_mission_callback, void)};
```

| Callback | Purpose | Return |
|----------|---------|--------|
| `start_command_callback` | Initialize a new command | `true` = success |
| `verify_command_callback` | Check if command is complete | `true` = complete |
| `exit_mission_callback` | Called when mission ends | void |

### Data Flow Diagram

```
GCS uploads mission via MAVLink
            │
            ▼
┌───────────────────────────────────┐
│     MISSION_COUNT message         │
│     count=5, mission_type=0       │
└───────────────┬───────────────────┘
                │
                ▼
┌───────────────────────────────────┐
│   MissionItemProtocol receives    │
│   - Validates count <= max_items  │
│   - Allocates resources           │
│   - Sends MISSION_REQUEST seq=0   │
└───────────────┬───────────────────┘
                │
                ▼ (repeat for each item)
┌───────────────────────────────────┐
│   MISSION_ITEM_INT received       │
│   - Validates sequence number     │
│   - Converts via mavlink_int_to_  │
│     mission_cmd()                 │
│   - Stores via add_cmd()          │
└───────────────┬───────────────────┘
                │
                ▼ (after all items)
┌───────────────────────────────────┐
│   MISSION_ACK sent                │
│   result=MAV_MISSION_ACCEPTED     │
│   - Mission logged to dataflash   │
└───────────────────────────────────┘
```

---

## Mission Upload Protocol

### Protocol State Machine

The mission upload follows a strict request-response protocol:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     MISSION UPLOAD STATE MACHINE                            │
└─────────────────────────────────────────────────────────────────────────────┘

     GCS                                              Vehicle
      │                                                  │
      │───── MISSION_COUNT (count=N) ────────────────►  │
      │                                                  │
      │                              ┌───────────────────┤
      │                              │ • Validate count  │
      │                              │ • Allocate memory │
      │                              │ • Set receiving   │
      │                              │   = true          │
      │                              └───────────────────┤
      │                                                  │
      │  ◄──── MISSION_REQUEST (seq=0) ─────────────────│
      │                                                  │
      │───── MISSION_ITEM_INT (seq=0) ──────────────►   │
      │                                                  │
      │                              ┌───────────────────┤
      │                              │ • Validate seq    │
      │                              │ • Store item      │
      │                              │ • request_i++     │
      │                              └───────────────────┤
      │                                                  │
      │  ◄──── MISSION_REQUEST (seq=1) ─────────────────│
      │                                                  │
      │───── MISSION_ITEM_INT (seq=1) ──────────────►   │
      │                                                  │
      │           ... repeat for seq=2 to N-1 ...       │
      │                                                  │
      │                              ┌───────────────────┤
      │                              │ All items rcvd    │
      │                              │ • Log mission     │
      │                              │ • receiving=false │
      │                              └───────────────────┤
      │                                                  │
      │  ◄──── MISSION_ACK (result=ACCEPTED) ───────────│
      │                                                  │
```

### Message Handlers

**File**: `libraries/GCS_MAVLink/MissionItemProtocol.cpp`

| MAVLink Message | Handler | Purpose |
|-----------------|---------|---------|
| `MISSION_COUNT` (44) | `handle_mission_count()` | Start upload, receive item count |
| `MISSION_ITEM_INT` (73) | `handle_mission_item()` | Receive individual waypoint |
| `MISSION_ITEM` (39) | `handle_mission_item()` | Legacy format (converted) |
| `MISSION_REQUEST_LIST` (43) | `handle_mission_request_list()` | GCS requests download |
| `MISSION_REQUEST_INT` (51) | `handle_mission_request_int()` | GCS requests specific item |
| `MISSION_CLEAR_ALL` (45) | `handle_mission_clear_all()` | Delete all items |
| `MISSION_WRITE_PARTIAL_LIST` (38) | `handle_mission_write_partial_list()` | Update subset |

### Timeout and Recovery

**From `MissionItemProtocol.h:95`:**

```cpp
const uint16_t upload_timeout_ms = 8000;  // 8 second total timeout
```

**Recovery Behavior:**
1. If no item received within 8 seconds → abort upload
2. If item not received within 1 second → resend MISSION_REQUEST
3. If sequence number wrong → send `MAV_MISSION_INVALID_SEQUENCE`

### Security Features

1. **Sender Verification**: Only accepts items from the GCS that initiated upload
2. **Sequence Validation**: Items must arrive in order (seq must equal request_i)
3. **Gap Prevention**: Cannot skip sequence numbers
4. **Single Upload**: Only one upload active at a time per mission type

### Error Codes (MISSION_ACK)

| Code | Name | Meaning |
|------|------|---------|
| 0 | `MAV_MISSION_ACCEPTED` | Success |
| 1 | `MAV_MISSION_ERROR` | Generic error |
| 2 | `MAV_MISSION_UNSUPPORTED` | Mission type not supported |
| 3 | `MAV_MISSION_DENIED` | Another upload in progress |
| 4 | `MAV_MISSION_INVALID_SEQUENCE` | Wrong seq number |
| 5 | `MAV_MISSION_NO_SPACE` | Storage full |
| 6 | `MAV_MISSION_OPERATION_CANCELLED` | Timeout |

---

## Mission Storage

### Storage Architecture

Missions are stored in persistent storage (EEPROM, Flash, or SD card):

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        MISSION STORAGE LAYOUT                               │
└─────────────────────────────────────────────────────────────────────────────┘

Bytes 0-3:   Version Header (0x65AE)
Bytes 4+:    Mission Commands (15 bytes each)

┌──────────────────────────────────────────────────────────────────────────┐
│  Offset    │  Size  │  Content                                           │
├────────────┼────────┼────────────────────────────────────────────────────┤
│  0-3       │  4     │  Version: 0x65AE (AP_MISSION_EEPROM_VERSION)       │
│  4-18      │  15    │  Command 0 (Home position)                         │
│  19-33     │  15    │  Command 1 (First mission item)                    │
│  34-48     │  15    │  Command 2                                         │
│  ...       │  ...   │  ...                                               │
│  4+15*N    │  15    │  Command N                                         │
└──────────────────────────────────────────────────────────────────────────┘

Position formula: pos = 4 + (index * 15)
```

### Command Storage Format (15 bytes)

**From `AP_Mission.h:27`:**

```cpp
#define AP_MISSION_EEPROM_COMMAND_SIZE  15  // bytes per command
```

**Storage Layout:**

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    COMMAND STORAGE FORMAT (15 bytes)                        │
└─────────────────────────────────────────────────────────────────────────────┘

For commands with ID < 256 (original format):
┌───────┬───────┬─────────────────────────────────────────────────────────────┐
│ Byte  │ Size  │ Content                                                     │
├───────┼───────┼─────────────────────────────────────────────────────────────┤
│   0   │   1   │ Command ID (8-bit)                                          │
│  1-2  │   2   │ p1 (16-bit general parameter)                               │
│  3-14 │  12   │ Content (command-specific data)                             │
└───────┴───────┴─────────────────────────────────────────────────────────────┘

For commands with ID >= 256 (extended format):
┌───────┬───────┬─────────────────────────────────────────────────────────────┐
│ Byte  │ Size  │ Content                                                     │
├───────┼───────┼─────────────────────────────────────────────────────────────┤
│   0   │   1   │ Tag byte = 1 (indicates extended format)                    │
│  1-2  │   2   │ Command ID (16-bit)                                         │
│  3-4  │   2   │ p1 (16-bit general parameter)                               │
│  5-14 │  10   │ Content (command-specific data)                             │
└───────┴───────┴─────────────────────────────────────────────────────────────┘
```

### Mission_Command Structure

**From `AP_Mission.h:409-447`:**

```cpp
struct Mission_Command {
    uint16_t index;             // Position in command list
    uint16_t id;                // MAVLink command ID (MAV_CMD_*)
    uint16_t p1;                // General purpose parameter
    Content content;            // Union of command-specific data
    uint8_t type_specific_bits; // Extra storage bits for some commands
};
```

### Content Union

The `Content` union stores command-specific data. For location-based commands:

**From `AP_Mission.h:305-406`:**

```cpp
union Content {
    Jump_Command jump;              // DO_JUMP: target index, repeat count
    Conditional_Delay_Command delay; // CONDITION_DELAY: seconds
    Yaw_Command yaw;                // CONDITION_YAW: angle, rate, direction
    Change_Speed_Command speed;     // DO_CHANGE_SPEED: type, speed, throttle
    Set_Servo_Command servo;        // DO_SET_SERVO: channel, PWM
    Navigation_Delay_Command nav_delay; // NAV_DELAY: time, UTC components
    // ... many more command types ...
    Location location{};            // Waypoint location (lat/lon/alt)
};
```

### Location Storage (Packed Format)

For commands that include a location, the position is packed:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      PACKED LOCATION FORMAT (12 bytes)                      │
└─────────────────────────────────────────────────────────────────────────────┘

┌───────┬───────┬─────────────────────────────────────────────────────────────┐
│ Byte  │ Bits  │ Content                                                     │
├───────┼───────┼─────────────────────────────────────────────────────────────┤
│   0   │  8    │ Flags:                                                      │
│       │       │   Bit 0: relative_alt (1 = alt relative to home)            │
│       │       │   Bit 1: (unused)                                           │
│       │       │   Bit 2: loiter_ccw (1 = counter-clockwise)                 │
│       │       │   Bit 3: terrain_alt (1 = alt relative to terrain)          │
│       │       │   Bit 4: origin_alt (1 = alt relative to EKF origin)        │
│       │       │   Bit 5: loiter_xtrack (crosstrack mode)                    │
│       │       │   Bit 6-7: type_specific_bits                               │
├───────┼───────┼─────────────────────────────────────────────────────────────┤
│  1-3  │  24   │ Altitude (cm, signed, supports ±83 km)                      │
├───────┼───────┼─────────────────────────────────────────────────────────────┤
│  4-7  │  32   │ Latitude (degrees × 10^7)                                   │
├───────┼───────┼─────────────────────────────────────────────────────────────┤
│  8-11 │  32   │ Longitude (degrees × 10^7)                                  │
└───────┴───────┴─────────────────────────────────────────────────────────────┘
```

### Storage Limits

**From `AP_Mission.h:29-35`:**

```cpp
// Maximum DO_JUMP commands tracked
#if HAL_MEM_CLASS >= HAL_MEM_CLASS_500
#define AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS 100
#else
#define AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS 15
#endif
```

| Platform | Max Commands | Max DO_JUMPs |
|----------|--------------|--------------|
| Large boards (F7, H7) | ~700-1000+ | 100 |
| Small boards (F4) | ~500 | 15 |

### SD Card Storage (Optional)

On ChibiOS boards with SD card:
```cpp
#define AP_MISSION_SDCARD_FILENAME "APM/mission.stg"
```

---

## Mission Command Types

### Command Classification

Commands are classified into three categories:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        COMMAND CLASSIFICATION                               │
└─────────────────────────────────────────────────────────────────────────────┘

1. NAVIGATION (NAV) Commands
   - Control where the vehicle goes
   - Only ONE active at a time
   - Checked via verify_command() until complete
   - Examples: NAV_WAYPOINT, NAV_LAND, NAV_LOITER_*

2. DO Commands
   - Actions to perform immediately
   - Execute between NAV commands
   - Usually complete instantly (verify returns true)
   - Examples: DO_SET_SERVO, DO_CHANGE_SPEED, DO_JUMP

3. CONDITIONAL Commands
   - Wait for a condition before proceeding
   - May take multiple verify() calls to complete
   - Examples: CONDITION_DELAY, CONDITION_DISTANCE
```

### Navigation Commands

| Command ID | Name | Parameters | Description |
|------------|------|------------|-------------|
| 16 | `NAV_WAYPOINT` | p1=delay(s), loc=target | Fly to waypoint |
| 17 | `NAV_LOITER_UNLIM` | p1=radius, loc=center | Loiter indefinitely |
| 18 | `NAV_LOITER_TURNS` | p1=turns+radius, loc=center | Loiter for N turns |
| 19 | `NAV_LOITER_TIME` | p1=time(s), loc=center | Loiter for time |
| 20 | `NAV_RETURN_TO_LAUNCH` | - | Return to home |
| 21 | `NAV_LAND` | p1=abort_alt, loc=landing | Land at location |
| 22 | `NAV_TAKEOFF` | p1=pitch(deg), loc=target | Takeoff to altitude |
| 30 | `NAV_CONTINUE_AND_CHANGE_ALT` | p1=climb(1)/desc(2), loc=alt | Change altitude |
| 31 | `NAV_LOITER_TO_ALT` | p1=radius, loc=target | Loiter until altitude |
| 93 | `NAV_DELAY` | time/UTC | Delay before next command |

### DO Commands

| Command ID | Name | Parameters | Description |
|------------|------|------------|-------------|
| 177 | `DO_JUMP` | p1=target, p2=count | Jump to command index |
| 178 | `DO_CHANGE_SPEED` | type, speed, throttle | Change airspeed |
| 179 | `DO_SET_HOME` | p1=use_current | Set home position |
| 181 | `DO_SET_RELAY` | relay_num, state | Control relay |
| 183 | `DO_SET_SERVO` | channel, PWM | Set servo position |
| 189 | `DO_LAND_START` | loc=landing | Mark landing sequence start |
| 201 | `DO_SET_ROI` | loc=target | Point camera at location |
| 203 | `DO_DIGICAM_CONTROL` | session, zoom, etc. | Camera control |

### Conditional Commands

| Command ID | Name | Parameters | Description |
|------------|------|------------|-------------|
| 112 | `CONDITION_DELAY` | seconds | Wait for time |
| 114 | `CONDITION_DISTANCE` | meters | Wait until distance from next WP |
| 115 | `CONDITION_YAW` | angle, rate, dir | Wait until heading reached |

### DO_JUMP Handling

**From `AP_Mission.h:923-926`:**

```cpp
struct jump_tracking_struct {
    uint16_t index;          // Index of do-jump command
    int16_t num_times_run;   // Number of times executed
} _jump_tracking[AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS];
```

**Special Values:**
- `p2 = -1` → Repeat forever (`AP_MISSION_JUMP_REPEAT_FOREVER`)
- Maximum iterations: 32767 (`AP_MISSION_JUMP_TIMES_MAX`)

---

## Mission Execution (ArduPlane)

### Callback Registration

**File**: `ArduPlane/Plane.h:680-683`

```cpp
AP_Mission mission{
    FUNCTOR_BIND_MEMBER(&Plane::start_command_callback, bool, const AP_Mission::Mission_Command &),
    FUNCTOR_BIND_MEMBER(&Plane::verify_command_callback, bool, const AP_Mission::Mission_Command &),
    FUNCTOR_BIND_MEMBER(&Plane::exit_mission_callback, void)};
```

### The update() Loop

**File**: `libraries/AP_Mission/AP_Mission.cpp`

The `update()` function is called at 10Hz+ and manages command execution:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      AP_Mission::update() Flow                              │
└─────────────────────────────────────────────────────────────────────────────┘

                    update() called at 10Hz+
                              │
                              ▼
                    ┌─────────────────┐
                    │ Mission running? │──── NO ───► Return
                    └────────┬────────┘
                             │ YES
                             ▼
              ┌──────────────────────────────┐
              │   NAV COMMAND PROCESSING     │
              └──────────────────────────────┘
                             │
              ┌──────────────┴──────────────┐
              │                             │
              ▼                             ▼
     nav_cmd_loaded?                  nav_cmd_loaded?
          NO                               YES
              │                             │
              ▼                             ▼
     advance_current_nav_cmd()      verify_command(_nav_cmd)
              │                             │
              │                             │ returns true?
              │                             │
              │                    ┌────────┴────────┐
              │                    │                 │
              │                   NO                YES
              │                    │                 │
              │                    ▼                 ▼
              │              Continue          nav_cmd_loaded = false
              │                                advance_current_nav_cmd()
              │                                      │
              └──────────────┬───────────────────────┘
                             │
                             ▼
              ┌──────────────────────────────┐
              │    DO COMMAND PROCESSING     │
              └──────────────────────────────┘
                             │
              ┌──────────────┴──────────────┐
              │                             │
              ▼                             ▼
     do_cmd_loaded?                   do_cmd_loaded?
          NO                               YES
              │                             │
              ▼                             ▼
     advance_current_do_cmd()        verify_command(_do_cmd)
              │                             │
              │                        Usually returns
              │                        true immediately
              │                             │
              └──────────────┬──────────────┘
                             │
                             ▼
                         Return
```

### Command Advancement

**Navigation Commands** (`advance_current_nav_cmd`):
1. Search forward from current position for next NAV command
2. Handle DO_JUMP commands (increment counter, jump if needed)
3. Load the NAV command and call `start_command()`
4. Set `_flags.nav_cmd_loaded = true`

**Do Commands** (`advance_current_do_cmd`):
1. Search forward for next DO command (stop at next NAV)
2. Call `start_command()` for the DO command
3. Most DO commands return true immediately from `verify_command()`

### start_command() Dispatch (ArduPlane)

**File**: `ArduPlane/commands_logic.cpp:6-210`

```cpp
bool Plane::start_command(const AP_Mission::Mission_Command& cmd)
{
    // Reset state for nav commands
    if (AP_Mission::is_nav_cmd(cmd)) {
        auto_state.takeoff_complete = true;
        nav_controller->set_data_is_stale();
        loiter.start_time_ms = 0;
    }

    switch(cmd.id) {
    case MAV_CMD_NAV_TAKEOFF:
        do_takeoff(cmd);
        break;
    case MAV_CMD_NAV_WAYPOINT:
        do_nav_wp(cmd);
        break;
    case MAV_CMD_NAV_LAND:
        do_land(cmd);
        break;
    case MAV_CMD_NAV_LOITER_UNLIM:
        do_loiter_unlimited(cmd);
        break;
    // ... more cases ...
    }
    return true;
}
```

### verify_command() Dispatch (ArduPlane)

**File**: `ArduPlane/commands_logic.cpp:221-331`

```cpp
bool Plane::verify_command(const AP_Mission::Mission_Command& cmd)
{
    switch(cmd.id) {
    case MAV_CMD_NAV_TAKEOFF:
        return verify_takeoff();
    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp(cmd);
    case MAV_CMD_NAV_LAND:
        return verify_land();  // Complex, uses AP_Landing
    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlim(cmd);  // Returns false (infinite)
    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_loiter_turns(cmd);
    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time();
    // ... more cases ...

    // DO commands return true immediately
    case MAV_CMD_DO_CHANGE_SPEED:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_LAND_START:
        return true;
    }
}
```

### do_* Function Pattern

**Purpose**: Initialize command execution (called once)

**Example - do_nav_wp()** (`commands_logic.cpp:400-403`):

```cpp
void Plane::do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    set_next_WP(cmd.content.location);
}
```

**Example - do_takeoff()** (`commands_logic.cpp:376-398`):

```cpp
void Plane::do_takeoff(const AP_Mission::Mission_Command& cmd)
{
    prev_WP_loc = current_loc;
    set_next_WP(cmd.content.location);

    // Set takeoff pitch (p1 parameter)
    auto_state.takeoff_pitch_cd = (int16_t)cmd.p1 * 100;
    if (auto_state.takeoff_pitch_cd <= 0) {
        auto_state.takeoff_pitch_cd = 400;  // Default 4 degrees
    }

    auto_state.takeoff_altitude_rel_cm = next_WP_loc.alt - home.alt;
    auto_state.takeoff_complete = false;
    // ... more initialization ...
}
```

### verify_* Function Pattern

**Purpose**: Check if command is complete (called repeatedly)

**Returns**: `true` when complete, `false` to continue

**Example - verify_nav_wp()** (`commands_logic.cpp:634-697`):

```cpp
bool Plane::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // Update navigation
    if (auto_state.crosstrack) {
        nav_controller->update_waypoint(prev_WP_loc, flex_next_WP_loc);
    } else {
        nav_controller->update_waypoint(current_loc, flex_next_WP_loc);
    }

    // Check acceptance distance
    float acceptance_distance_m = nav_controller->turn_distance(...);
    if (wp_dist <= acceptance_distance_m) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached waypoint #%i", cmd.index);
        return true;  // Complete!
    }

    // Check if passed waypoint
    if (current_loc.past_interval_finish_line(prev_WP_loc, flex_next_WP_loc)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Passed waypoint #%i", cmd.index);
        return true;  // Complete!
    }

    return false;  // Still navigating
}
```

**Example - verify_takeoff()** (`commands_logic.cpp:567-628`):

```cpp
bool Plane::verify_takeoff()
{
    // Check if reached takeoff altitude
    int32_t relative_alt_cm = adjusted_relative_altitude_cm();
    if (relative_alt_cm > auto_state.takeoff_altitude_rel_cm) {
        gcs().send_text(MAV_SEVERITY_INFO, "Takeoff complete at %.2fm",
                        (double)(relative_alt_cm*0.01f));
        auto_state.takeoff_complete = true;
        return true;  // Complete!
    }
    return false;  // Still climbing
}
```

---

## ArduPlane-Specific Commands

### NAV_TAKEOFF

**Command ID**: 22

**Parameters**:
- `p1`: Pitch angle in degrees (default 4° if 0)
- `location.alt`: Target altitude

**Execution** (`commands_logic.cpp:376-398`):
1. Save current location as prev_WP
2. Set takeoff pitch angle
3. Calculate target altitude relative to home
4. Disable crosstrack (fly straight ahead)
5. Hold GPS ground course for heading

**Completion** (`commands_logic.cpp:567-628`):
- When altitude reached OR takeoff timeout

### NAV_LAND

**Command ID**: 21

**Parameters**:
- `p1`: Abort altitude (meters, default 30m if 0)
- `location`: Landing point

**Execution** (`commands_logic.cpp:405-433`):
1. Set landing waypoint
2. Configure abort altitude for go-around
3. Reset rangefinder state
4. Delegate to `AP_Landing::do_land()`

**Completion**:
- Handled by `AP_Landing::verify_land()` (see [Fixed Wing Landing Analysis](fixed-wing-landing-analysis.md))

### NAV_LOITER_TO_ALT

**Command ID**: 31

**Parameters**:
- `p1`: Loiter radius (meters)
- `location`: Loiter center and target altitude
- `location.loiter_ccw`: Counter-clockwise flag

**Execution** (`commands_logic.cpp:533-543`):
1. Set loiter waypoint
2. Set loiter direction from flags
3. Initialize condition_value = 0

**Completion** (`commands_logic.cpp:774-801`):
1. Update loiter (circle around point)
2. Check if target altitude reached
3. Once altitude reached, verify heading alignment with next waypoint
4. Complete when both altitude and heading satisfied

### DO_LAND_START

**Command ID**: 189

**Purpose**: Mark the beginning of a landing sequence

**Behavior**:
- Does nothing when executed directly
- Used by `jump_to_landing_sequence()` to find landing entry point
- RTL with `RTL_AUTOLAND` jumps to this command

### DO_CHANGE_SPEED

**Command ID**: 178

**Parameters**:
- `speed.speed_type`: 0=airspeed, 1=groundspeed
- `speed.target_ms`: Target speed in m/s (-2 = reset to default)
- `speed.throttle_pct`: Throttle percentage (if speed not used)

**Execution** (`commands_logic.cpp:962-1003`):
```cpp
switch (speedtype) {
case SPEED_TYPE_AIRSPEED:
    if (speed_target_ms >= aparm.airspeed_min &&
        speed_target_ms <= aparm.airspeed_max) {
        new_airspeed_cm = speed_target_ms * 100;
    }
    break;
case SPEED_TYPE_GROUNDSPEED:
    aparm.min_groundspeed.set(speed_target_ms);
    break;
}
```

### NAV_LOITER_TURNS and NAV_LOITER_TIME

**Loiter Turns** (`commands_logic.cpp:464-474`):
- `p1` low byte: Number of turns (fractional supported via type_specific_bits)
- `p1` high byte: Radius (×10 if bit set)
- Completes after specified turns AND heading aligned

**Loiter Time** (`commands_logic.cpp:476-486`):
- `p1`: Time in seconds
- Starts timer when loiter target reached
- Completes after time elapsed AND heading aligned

---

## Mission State Management

### Mission States

**From `AP_Mission.h:488-492`:**

```cpp
enum mission_state {
    MISSION_STOPPED = 0,   // Not running
    MISSION_RUNNING = 1,   // Currently executing
    MISSION_COMPLETE = 2   // Finished
};
```

### Mission Flags

**From `AP_Mission.h:805-813`:**

```cpp
struct Mission_Flags {
    mission_state state;           // Current mission state
    bool nav_cmd_loaded;           // Navigation command is loaded
    bool do_cmd_loaded;            // Do command is loaded
    bool do_cmd_all_done;          // No more do commands before next nav
    bool in_landing_sequence;      // Jumped to landing sequence
    bool resuming_mission;         // Resuming from interrupt
    bool in_return_path;           // In DO_RETURN_PATH_START sequence
};
```

### Waypoint History

**From `AP_Mission.h:816`:**

```cpp
uint16_t _wp_index_history[AP_MISSION_MAX_WP_HISTORY];  // Last 7 waypoint indices
```

Used for:
- Mission resume (MIS_RESTART parameter)
- `DO_SET_RESUME_REPEAT_DIST` command
- Rewinding to previous waypoint on resume

### Mission Control Methods

| Method | Purpose |
|--------|---------|
| `start()` | Begin mission from command 1 |
| `stop()` | Pause mission execution |
| `resume()` | Continue from last command |
| `start_or_resume()` | Uses MIS_RESTART parameter to decide |
| `reset()` | Reset to beginning without clearing |
| `clear()` | Delete all mission commands |
| `set_current_cmd(index)` | Jump to specific command |

### Landing Sequence Handling

```cpp
// Find and jump to landing sequence
bool jump_to_landing_sequence(const Location &current_loc);

// Find landing start nearest to current location
uint16_t get_landing_sequence_start(const Location &current_loc);

// Jump to abort landing sequence (go-around)
bool jump_to_abort_landing_sequence(const Location &current_loc);
```

**Landing Sequence Flow**:
1. `DO_LAND_START` marks the entry point
2. `jump_to_landing_sequence()` finds nearest `DO_LAND_START`
3. Sets `_flags.in_landing_sequence = true`
4. Mission executes landing commands

---

## Practical Guide for Operators

### Creating Missions in GCS

**Basic Mission Structure**:
```
Index 0:  HOME (automatically set)
Index 1:  NAV_TAKEOFF      alt=50m
Index 2:  NAV_WAYPOINT     lat, lon, alt=100m
Index 3:  NAV_WAYPOINT     lat, lon, alt=100m
Index 4:  NAV_LOITER_TO_ALT  alt=30m (approach altitude)
Index 5:  DO_LAND_START    (marks landing sequence)
Index 6:  NAV_LAND         landing point
```

### Common Mission Patterns

**Survey Pattern**:
```
TAKEOFF → WP1 → WP2 → WP3 → ... → WPn → RTL
```

**Landing Pattern**:
```
... → LOITER_TO_ALT (approach alt) → DO_LAND_START → NAV_LAND
```

**Looping Pattern**:
```
WP1 → WP2 → WP3 → DO_JUMP(target=1, count=5) → RTL
```

### Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MIS_RESTART` | 0 | 0=resume, 1=restart on mode change |
| `MIS_OPTIONS` | 0 | Bitmask: bit0=clear on boot |
| `WP_RADIUS` | 90m | Waypoint acceptance radius |
| `WP_MAX_RADIUS` | 0 | Max distance for WP completion (0=off) |
| `WP_LOITER_RAD` | 60m | Default loiter radius |
| `RTL_AUTOLAND` | 0 | 0=off, 1=after loiter, 2=immediate |

### Best Practices

1. **Always include a takeoff command** at the start
2. **Set appropriate altitudes** for terrain
3. **Use LOITER_TO_ALT before landing** for stable approach
4. **Include DO_LAND_START** before NAV_LAND for RTL landing
5. **Test missions in SITL** before flying
6. **Verify mission upload** - check item count after upload

---

## Troubleshooting

### Symptom → Solution Table

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| **Mission won't upload** | Storage full or comm error | Check `MIS_TOTAL`, verify connection |
| **Mission starts from wrong point** | MIS_RESTART=0, resuming | Set MIS_RESTART=1 or use `reset` |
| **Vehicle skips waypoints** | WP_RADIUS too large | Decrease WP_RADIUS |
| **Vehicle never reaches waypoint** | WP_RADIUS too small | Increase WP_RADIUS |
| **DO_JUMP not working** | Count exhausted or index wrong | Check jump count and target index |
| **Mission ends early** | Missing commands or RTL | Verify full mission uploaded |
| **Wrong altitude** | Frame mismatch (relative vs absolute) | Check altitude frame in GCS |
| **Landing not triggered** | Missing DO_LAND_START | Add DO_LAND_START before NAV_LAND |

### Log Analysis Tips

**Key Log Messages**:

| Log Type | Field | What to Look For |
|----------|-------|------------------|
| `CMD` | `CId` | Command ID being executed |
| `CMD` | `CNum` | Command index number |
| `CTUN` | `NavPitch` | Navigation pitch demand |
| `NTUN` | `WpDist` | Distance to waypoint |
| `NTUN` | `TargBrg` | Bearing to target |
| `MSG` | text | "Reached waypoint", "Passed waypoint" |

**Checking Mission Execution**:
1. Look for `CMD` messages showing command transitions
2. Check `NTUN.WpDist` approaching zero
3. Verify `MSG` log shows "Reached waypoint #X"

### Common Mistakes

1. **Setting landing waypoint altitude > 0**
   - Landing point should typically be altitude 0 (relative)
   - See [Fixed Wing Landing Analysis](fixed-wing-landing-analysis.md#what-happens-if-landing-waypoint-altitude-is-wrong)

2. **Forgetting to arm before AUTO**
   - Mission won't start if disarmed
   - Some commands require armed state

3. **DO_JUMP targeting wrong index**
   - Index 0 is HOME, mission starts at index 1
   - Jump target must exist in mission

4. **Loiter radius too small**
   - Fixed-wing needs minimum turn radius
   - Use at least 30-50m for most aircraft

5. **Missing takeoff in mission**
   - First NAV command should be NAV_TAKEOFF
   - Or use TAKEOFF flight mode first

---

## Source Files Reference

### Core Mission Logic

| File | Key Functions | Lines |
|------|---------------|-------|
| `libraries/AP_Mission/AP_Mission.h` | `Mission_Command`, `mission_state`, callbacks | 409-447, 488-492 |
| `libraries/AP_Mission/AP_Mission.cpp` | `update()`, `advance_current_nav_cmd()` | 310+, 833+ |
| `libraries/AP_Mission/AP_Mission.cpp` | `read_cmd_from_storage()`, `write_cmd_to_storage()` | 818+, 822+ |

### MAVLink Protocol

| File | Key Functions | Lines |
|------|---------------|-------|
| `libraries/GCS_MAVLink/MissionItemProtocol.h` | Protocol class definition | All |
| `libraries/GCS_MAVLink/MissionItemProtocol.cpp` | `handle_mission_count()`, `handle_mission_item()` | 78+, 264+ |
| `libraries/GCS_MAVLink/GCS_Common.cpp` | Message dispatch | Various |

### ArduPlane Integration

| File | Key Functions | Lines |
|------|---------------|-------|
| `ArduPlane/Plane.h` | Mission callback registration | 680-683 |
| `ArduPlane/commands_logic.cpp` | `start_command()`, `verify_command()` | 6-210, 221-331 |
| `ArduPlane/commands_logic.cpp` | `do_takeoff()`, `do_nav_wp()`, `do_land()` | 376-433 |
| `ArduPlane/commands_logic.cpp` | `verify_takeoff()`, `verify_nav_wp()` | 567-697 |

---

## Appendix: Glossary of Terms

### MAVLink Terms

| Term | Definition |
|------|------------|
| **MAVLink** | Micro Air Vehicle Link - communication protocol for drones |
| **MISSION_ITEM** | Legacy MAVLink message for mission waypoints (float lat/lon) |
| **MISSION_ITEM_INT** | Preferred MAVLink message for mission waypoints (int32 lat/lon) |
| **MISSION_COUNT** | Message indicating number of items to upload |
| **MISSION_REQUEST** | Vehicle requesting specific mission item from GCS |
| **MISSION_ACK** | Acknowledgment of mission transfer completion |
| **MAV_CMD** | Mission command identifier (e.g., MAV_CMD_NAV_WAYPOINT = 16) |

### Mission Terms

| Term | Definition |
|------|------------|
| **NAV Command** | Navigation command that controls vehicle movement |
| **DO Command** | Action command that executes immediately |
| **Conditional Command** | Command that waits for a condition |
| **Waypoint** | A location the vehicle should fly to |
| **Loiter** | Circle around a point |
| **DO_JUMP** | Command to repeat a sequence of mission items |
| **Landing Sequence** | Series of commands starting with DO_LAND_START |

### ArduPilot Terms

| Term | Definition |
|------|------------|
| **AP_Mission** | ArduPilot library managing mission storage and execution |
| **GCS** | Ground Control Station (Mission Planner, QGroundControl) |
| **AUTO Mode** | Flight mode that executes the uploaded mission |
| **RTL** | Return To Launch - fly back to home location |
| **FBWA/FBWB** | Fly By Wire modes - stabilized manual control |
| **TECS** | Total Energy Control System - manages altitude/speed |

### Coordinate Frames

| Frame | Description |
|-------|-------------|
| **Relative** | Altitude above home position |
| **Absolute (AMSL)** | Altitude above mean sea level |
| **Terrain** | Altitude above terrain (requires terrain data) |
| **Origin** | Altitude above EKF origin |

### Parameters

| Parameter | Description |
|-----------|-------------|
| **MIS_RESTART** | Mission restart behavior on mode change |
| **MIS_OPTIONS** | Mission option flags |
| **WP_RADIUS** | Waypoint acceptance radius |
| **WP_LOITER_RAD** | Default loiter radius |
| **RTL_AUTOLAND** | Automatic landing after RTL |

---

## Summary

ArduPilot's mission system provides:

1. **Robust Upload Protocol**: Request-response with timeout recovery
2. **Efficient Storage**: 15 bytes per command in persistent memory
3. **Flexible Command Set**: NAV, DO, and Conditional commands
4. **Vehicle Abstraction**: Callbacks allow vehicle-specific handling
5. **Safety Features**: Jump limits, validation, landing sequences

For ArduPlane specifically:
- Commands are handled in `commands_logic.cpp`
- Each command has `do_*()` for initialization and `verify_*()` for completion
- Landing is delegated to the `AP_Landing` library
- Special handling for VTOL/QuadPlane transitions

The system is designed for reliability with:
- Persistent storage surviving power loss
- Protocol recovery from communication errors
- Clear separation between generic logic and vehicle-specific behavior
