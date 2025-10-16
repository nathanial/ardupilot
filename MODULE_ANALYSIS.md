# ArduPilot Modules Analysis

This document provides a comprehensive analysis of all git submodules in the ArduPilot project, explaining their purpose, integration, and usage within the codebase.

---

## Table of Contents

1. [ChibiOS](#1-chibios)
2. [CrashDebug](#2-crashdebug)
3. [DroneCAN](#3-dronecan)
4. [Micro-CDR and Micro-XRCE-DDS-Client](#4-micro-cdr-and-micro-xrce-dds-client)
5. [Testing Frameworks (gtest & gbenchmark)](#5-testing-frameworks-gtest--gbenchmark)
6. [gSOAP](#6-gsoap)
7. [LittleFS](#7-littlefs)
8. [lwIP](#8-lwip)
9. [MAVLink](#9-mavlink)
10. [Waf](#10-waf)

---

## 1. ChibiOS

### Overview
ChibiOS is a professional-grade, open-source **Real-Time Operating System (RTOS)** kernel designed for embedded systems, particularly ARM-based microcontrollers.

**Location:** `/modules/ChibiOS`

### Purpose
- Provides RTOS kernel features: thread management, scheduling, mutexes, semaphores, events, message passing
- Hardware Abstraction Layer (HAL) with drivers for peripherals (SPI, I2C, UART, CAN, USB, ADC, GPIO, PWM)
- Support for multiple STM32 processor families (F0, F1, F3, F4, F7, H7, L0, L1, L4, G0, G4)
- Version: 21.11.x

### Integration in ArduPilot

**Location:** `/libraries/AP_HAL_ChibiOS/`

ChibiOS is integrated via the **AP_HAL_ChibiOS** abstraction layer, which wraps ChibiOS functionality to provide a standardized hardware interface.

**Key Components:**
- Scheduler (thread management with priority-based preemptive scheduling)
- GPIO (digital I/O control)
- RCInput/RCOutput (RC signal reading and PWM output)
- UART/Serial (up to 10 UART drivers)
- I2C/SPI device managers
- AnalogIn (ADC sampling)
- Storage (flash memory management)
- CANIface (CAN/CAN-FD bus interface)
- DSP (digital signal processing)

**Thread Architecture:**
```
Priority Hierarchy (lower number = lower priority):
- APM_MONITOR_PRIORITY       183 (System monitoring)
- APM_TIMER_PRIORITY         181 (Fast timer callbacks)
- APM_RCOUT_PRIORITY         181 (PWM output)
- APM_MAIN_PRIORITY          180 (Main autopilot loop)
- APM_CAN_PRIORITY           178 (CAN bus)
- APM_RCIN_PRIORITY          177 (RC input)
- APM_SPI_PRIORITY           181 (SPI sampling)
- APM_I2C_PRIORITY           176 (I2C communication)
- APM_UART_PRIORITY          60  (Serial I/O)
- APM_STORAGE_PRIORITY       59  (Flash operations)
```

### Supported Platforms
All STM32 microcontroller-based autopilots including:
- Pixhawk/Pixhawk 2/4 series
- CubeBlack, CubeOrange variants
- fmuv3, fmuv4, fmuv5
- Numerous third-party STM32-based autopilots

### Build Configuration
- Board class: `ChibiOS`
- Define: `CONFIG_HAL_BOARD = HAL_BOARD_CHIBIOS`
- Library: `AP_HAL_ChibiOS`
- Compiler: GCC 10.2.1 or later
- Hardware definitions in `hwdef/` subdirectories

---

## 2. CrashDebug

### Overview
CrashDebug is a post-mortem debugging tool for ARM Cortex-M microcontrollers that enables investigation of crashes without requiring direct access to the failing device.

**Location:** `/modules/CrashDebug`

### Purpose
- Captures processor state at crash time (registers r0-r12, sp, lr, pc, xpsr)
- Captures RAM contents to create crash dumps
- Works with GDB for interactive post-mortem debugging
- Enables offline crash investigation using only crash dump + .elf file

### Integration in ArduPilot

**Feature Flag:** `AP_CRASHDUMP_ENABLED`

**Build Configuration:**
- Automatically enabled for boards with flash >= 2MB
- Category: Developer features
- Label: `CRASHCATCHER`

**Main Implementation:**
- `/libraries/AP_HAL_ChibiOS/hwdef/common/crashdump.c` - Main crash dump handler
- `/libraries/AP_HAL_ChibiOS/system.cpp` - Fault handler integration

### Crash Dump Storage

**Two storage methods:**

1. **Flash Storage** (`HAL_CRASH_DUMP_FLASHPAGE`)
   - Stores crash dump in dedicated flash sector
   - Retrieved via MAVFTP as `@SYS/crash_dump.bin`
   - Persists across reboots

2. **Serial Port Output** (`HAL_CRASH_SERIAL_PORT`)
   - Outputs crash dump over UART at 921600 baud
   - Hex-encoded memory dump

### Debugging Workflow

**Via stored dump:**
```bash
Tools/debug/crash_debugger.py <elf> --dump-debug --dump-filein crash_dump.bin
```

**Via serial (real-time):**
```bash
Tools/debug/crash_debugger.py <elf> --ser-debug --ser-port /dev/ttyXXX
```

**Via SWD/debugger:**
```bash
Tools/debug/crash_debugger.py <elf> --swd-debug --gdb-port :3333
```

### Captured Information
- CPU registers (r0-r12, sp, lr, pc, xpsr)
- Entire RAM contents
- Thread stack data
- ChibiOS system structures
- Thread names and context

---

## 3. DroneCAN

### Overview
DroneCAN is a lightweight protocol stack for reliable communication in aerospace and robotic applications via CAN (Controller Area Network) bus. It's a fork of UAVCAN v0.9.

**Location:** `/modules/DroneCAN/`

### Submodules

#### A. DSDL (Data Structure Definition Language)
**Location:** `/modules/DroneCAN/DSDL/`

Defines standard message and service formats for DroneCAN communication.

**Structure:**
- `uavcan/` - Standard UAVCAN/DroneCAN protocol messages
- `uavcan/equipment/` - Equipment-specific (ESCs, actuators, sensors, GNSS)
- `uavcan/protocol/` - Protocol infrastructure
- `ardupilot/` - ArduPilot-specific extensions
- `com/` - Vendor-specific namespaces (Hobbywing, T-Motor, Xacti, etc.)

#### B. dronecan_dsdlc (DSDL Compiler)
**Location:** `/modules/DroneCAN/dronecan_dsdlc/`

Python-based code generator that converts DSDL definitions into C/C++ serialization/deserialization code.

**Output:**
- Header files with generated C structures
- Pack/unpack functions for each message type
- Automatic bit-level field packing

#### C. libcanard (CAN Library)
**Location:** `/modules/DroneCAN/libcanard/`

Minimal C implementation of DroneCAN protocol stack for resource-constrained embedded systems.

**Design Goals:**
- Small ROM footprint (4K minimum)
- Small RAM footprint (4K minimum)
- Deterministic, predictable execution
- Standard C99 portability

**Core Components:**
- `canard.c` / `canard.h` - Main library
- Memory pool allocation (32-byte blocks)
- RX transfer state management
- Prioritized TX queue

**Platform Drivers:**
- STM32 microcontroller CAN support
- NuttX RTOS support
- Linux SocketCAN interface
- AVR microcontroller support

#### D. pydronecan (Python Tools)
**Location:** `/modules/DroneCAN/pydronecan/`

Python implementation of DroneCAN protocol stack for testing, debugging, and PC-based tools.

**Tools:**
- `dronecan_bridge.py` - Bridge between CAN interfaces
- `dronecan_msgid_decode.py` - Decode CAN message IDs
- `dronecan_tcplink.py` - TCP/IP link for DroneCAN

### Integration in ArduPilot

**Location:** `/libraries/AP_DroneCAN/`

**Main Class:**
```cpp
class AP_DroneCAN : public AP_CANDriver, public AP_ESC_Telem_Backend {
    CanardInterface canard_iface;  // Libcanard interface wrapper
    AP_DroneCAN_DNA_Server _dna_server;  // Dynamic Node Allocation
    // Publishers and Subscribers for various messages
};
```

**Configuration Parameters:**
- `CAN_D*_NODE` - Own node ID
- `CAN_D*_SRV_BM` - Servo output bitmask
- `CAN_D*_ESC_BM` - ESC output bitmask
- `CAN_D*_SRV_RATE` - Servo update rate

### Supported Peripherals

**Sensors:**
- GNSS (Fix2, Auxiliary, Heading, RTK)
- Compass/Magnetometer
- Barometric altitude
- Airspeed sensors
- Range finders
- Optical flow
- Proximity sensors

**Actuators:**
- ESCs (Electronic Speed Controllers)
- Servos (standard and Himark/Volz)
- RGB LEDs
- Buzzers

**Power:**
- Battery monitors
- MPPT chargers

**Specialized:**
- Camera/Gimbal (Xacti)
- Engine monitoring (EFI)
- RPM sensors
- Temperature sensors

### Build System Integration
**Location:** `/Tools/ardupilotwaf/dronecangen.py`

WAF build system task generates C++ headers from DSDL definitions during build.

---

## 4. Micro-CDR and Micro-XRCE-DDS-Client

### Overview
These modules provide DDS (Data Distribution Service) communication capabilities for integration with ROS 2.

### Micro-CDR
**Location:** `/modules/Micro-CDR`

Lightweight C library implementing CDR (Common Data Representation) serialization standard from OMG.

**Features:**
- Static buffers for memory efficiency
- Big-endian and little-endian support
- Serialization of basic types, arrays, sequences
- ROS 2 Quality Level 1 certified

### Micro-XRCE-DDS-Client
**Location:** `/modules/Micro-XRCE-DDS-Client`

C library providing DDS-XRCE (DDS for eXtreme Resources Constrained Devices) implementation.

**Features:**
- Session management for DDS communications
- Multiple transport protocols (UDP, TCP, Serial, CAN)
- Stream framing protocol
- Publish-Subscribe messaging
- Request-Reply (service) messaging
- Quality of Service (QoS) management
- ROS 2 Quality Level 1 certified

### Integration in ArduPilot

**Location:** `/libraries/AP_DDS/`

**Architecture:**
```
ArduPilot Vehicle (XRCE Client)
    ↓
Micro-XRCE-DDS-Client (Session Management)
    ↓
Micro-CDR (Serialization)
    ↓
Transport Layer (Serial/UDP)
    ↓
Micro-ROS Agent (on external computer)
    ↓
ROS 2 DDS System
```

**Published Topics (from ArduPilot):**
- `/ap/time` - System time
- `/ap/navsat` - GPS data
- `/ap/imu/experimental/data` - IMU data
- `/ap/battery` - Battery state
- `/ap/pose/filtered` - Local pose
- `/ap/clock` - Clock information

**Subscribed Topics (received by ArduPilot):**
- `/ap/joy` - Joystick input
- `/ap/cmd_vel` - Velocity commands
- `/ap/cmd_gps_pose` - GPS position commands
- `/ap/tf` - Dynamic transforms

**Services:**
- `/ap/arm_motors` - Arm/disarm motors
- `/ap/mode_switch` - Switch flight modes
- `/ap/prearm_check` - Pre-arm checks

### Transport Mechanisms

**Serial Transport:**
- Custom UART transport via AP_HAL
- Default 115200 baud
- Serial protocol ID: 45 (`SerialProtocol_DDS_XRCE`)
- Implemented in `/libraries/AP_DDS/AP_DDS_Serial.cpp`

**UDP Transport:**
- Socket-based UDP communication
- Default port: 2019
- Implemented in `/libraries/AP_DDS/AP_DDS_UDP.cpp`

### Build System Integration

**Location:** `/libraries/AP_DDS/wscript`

- Uses `microxrceddsgen` tool to generate C code from IDL files
- Build flag: `--enable-DDS`
- IDL definitions in `/libraries/AP_DDS/Idl/`

### Usage Example

**Starting ArduPilot with DDS:**
```bash
sim_vehicle.py -v ArduPlane -DG --console --enable-DDS
```

**Running Micro-ROS Agent:**
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

**Accessing data:**
```bash
ros2 topic echo /ap/time
ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: True}"
```

---

## 5. Testing Frameworks (gtest & gbenchmark)

### Overview
ArduPilot uses Google's testing frameworks for unit testing and performance benchmarking.

### gtest (Google Test)
**Location:** `/modules/gtest/`

Unit testing framework for individual components and functions.

**Includes:**
- Main gtest library in `modules/gtest/googletest/`
- Google Mock library in `modules/gtest/googlemock/`

**Wrapper:** `/tests/AP_gtest.h`
- Provides `AP_GTEST_MAIN()` macro
- Provides `AP_GTEST_PANIC()` macro for HAL panic override

### gbenchmark (Google Benchmark)
**Location:** `/modules/gbenchmark/`

Performance testing framework to measure execution time of code.

**Wrapper:** `/benchmarks/AP_gbenchmark.h`
- Provides `BENCHMARK_MAIN()` macro
- Provides `gbenchmark_escape()` to prevent compiler optimization

### Build System Integration

**Test Configuration:**
- WAF module: `/Tools/ardupilotwaf/gtest.py`
- Tests disabled for ChibiOS/STM32 boards
- Tests incompatible with static linking

**Benchmark Configuration:**
- WAF module: `/Tools/ardupilotwaf/gbenchmark.py`
- Only supports native (non-cross-compiled) builds
- Uses CMake to build benchmark library

**Discovery:**
- `ap_find_tests()` function automatically discovers test files
- `ap_find_benchmarks()` function automatically discovers benchmark files
- Each test compiled as separate executable

### Test Coverage

**23 Libraries with Tests:**
- AP_Math (vectors, matrices, quaternions, control calculations)
- AP_HAL (prescaler, timeout, string formatting, ring buffers)
- AP_Baro (barometric calculations)
- AP_GPS (GPS functionality)
- AP_Param (parameter management)
- AP_Common (sorting, bitmasking, location data)
- Filter (audio filters)
- And many more...

**2 Libraries with Benchmarks:**
- AP_Math (matrix operations, vector operations)
- AP_HAL_Linux (performance testing)

### Running Tests

**All tests:**
```bash
./waf configure --board linux
./waf check
```

**Specific test:**
```bash
./waf check --target test_vector3
```

**Benchmarks:**
```bash
./waf --program-group benchmarks
./build/linux/benchmarks/benchmark_matrix
```

### Test Structure Example

```cpp
#include <AP_gtest.h>
#include <AP_Math/AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(Vector3Test, Operator)
{
    Vector3f v_float0{1.0f, 1.0f, 1.0f};
    EXPECT_FALSE(v_float0.is_zero());
    v_float0 = Vector3f();
    EXPECT_TRUE(v_float0.is_zero());
}

AP_GTEST_MAIN()
```

---

## 6. gSOAP

### Overview
gSOAP is a comprehensive SOAP (Simple Object Access Protocol) and XML web services toolkit for C and C++.

**Location:** `/modules/gsoap`

### Purpose
- SOAP 1.1/1.2 client and server implementations
- XML Web service support with automatic serialization
- WSDL 1.1/2.0 parsing and code generation
- XML schema binding and processing
- WS-Security, WS-Addressing support
- REST HTTP operations
- XML-RPC and JSON support

### Integration in ArduPilot

**Used exclusively for ONVIF camera support**

**Location:** `/libraries/AP_ONVIF/`

Provides client-side ONVIF (Open Network Video Interface Forum) support for IP cameras.

**ONVIF Protocol Services:**
- Device Service (`onvifDeviceBindingProxy`) - Device info, capabilities
- Media Service (`onvifMediaBindingProxy`) - Media profiles, streaming
- PTZ Service (`onvifPTZBindingProxy`) - Pan/Tilt/Zoom control

**WS-Security Integration:**
- Uses `wsseapi-lite.cpp` plugin
- Username Token authentication
- SHA-1 hashing with Base64 encoding
- SOAP security headers with nonce and timestamp

### Build System Integration

**Location:** `/libraries/AP_ONVIF/wscript`

**Code Generation:**
```bash
soapcpp2 -2 -Cp onvif -j -x <onvif.h>
```

Generates:
- `onvifC.cpp` - Parameter marshalling
- `onvifDeviceBindingProxy.cpp` - Device service proxy
- `onvifMediaBindingProxy.cpp` - Media service proxy
- `onvifPTZBindingProxy.cpp` - PTZ service proxy

**Linked Components:**
- `stdsoap2.cpp` - Core gSOAP SOAP/XML engine
- `dom.cpp` - DOM parser
- `wsseapi-lite.cpp` - WS-Security plugin

**Feature Flag:** `ENABLE_ONVIF`

### Functionality

**Camera Operations:**
- Device information retrieval
- Capability discovery
- Profile management
- Pan/Tilt/Zoom absolute positioning
- WS-Security authentication

---

## 7. LittleFS

### Overview
LittleFS (Little Filesystem) is an embedded filesystem designed for microcontroller flash memory.

**Location:** `/modules/littlefs/`

### Purpose
- Lightweight with minimal overhead
- Journaling with power-loss resilience
- Wear-leveling support
- Block-based abstraction for flash chips
- POSIX-like API (open, read, write, mkdir, etc.)

### Integration in ArduPilot

**Location:** `/libraries/AP_Filesystem/AP_Filesystem_FlashMemory_LittleFS.{h,cpp}`

Part of ArduPilot's pluggable filesystem abstraction layer.

**Filesystem Backends:**
```
AP_Filesystem (main interface)
  ├── AP_Filesystem_FlashMemory_LittleFS (littlefs)
  ├── AP_Filesystem_FATFS (SD cards)
  ├── AP_Filesystem_ESP32 (SPIFFS on ESP32)
  └── AP_Filesystem_Posix (Linux/SITL)
```

**Conditional Compilation:**
- Enabled via `HAL_OS_LITTLEFS_IO` flag
- Flag: `AP_FILESYSTEM_LITTLEFS_ENABLED`
- Default for ChibiOS boards with external flash

### Flash Hardware Support

**Flash Chip Types:**
- JEDEC NOR Flash (W25Q16/32/64/128/256, Winbond, Macronix, Micron)
- W25NXX NAND Flash (W25N01GV 128MB, W25N02KV 256MB)
- SITL Mode (file-based simulation)

**Flash Device Configuration:**
- NOR flash: 256-byte pages, 64KB erase blocks
- NAND flash: 2048-byte pages, 128KB erase blocks
- SPI-based communication

### Storage Uses

**1. Flight Logs (AP_Logger_File):**
- Log files: `logs/NNNNNNNN.BIN`
- Metadata: `LASTLOG.TXT`
- Directory: `/APM` (ChibiOS default)

**2. Lua Scripts (AP_Scripting):**
- User-uploaded scripts
- Directory: `scripts/`

**3. Parameter/Mission Files:**
- Mission waypoints
- Parameter files
- Via `@PARAM` and `@MISSION` prefixes

**4. Generic Storage:**
- Calibration data
- Configuration files

### Platforms Using littlefs

**Primary:**
- ChibiOS-based flight controllers (STM32F4, STM32H7)
- Pixhawk and variants with SPI external flash
- Custom flight controllers with `DATAFLASH` declarations

**Testing:**
- SITL with `sim_littlefs` option

### Technical Details

**File Management:**
- Maximum 16 concurrent open files
- Modification time attributes
- Extended attributes support

**Flash Operations:**
- Page-aligned JEDEC commands
- Write-enable handshake
- 64KB block erase
- Busy-wait polling

**Optimization:**
- Lookahead buffer for entire flash state
- Smart write batching
- Block boundary alignment
- Wear-leveling (75,000 cycle tracking)

---

## 8. lwIP

### Overview
lwIP is a lightweight TCP/IP stack for embedded systems.

**Location:** `/modules/lwip/`

### Purpose
- Core Protocols: IPv4, IPv6, ICMP, UDP, TCP, ARP, DHCP
- Advanced Features: IGMP (multicast), Berkeley socket API
- PPP Support: PPPoS and PPPoE
- Optional: DNS/mDNS, SNMP, HTTP server
- Minimal resources: tens of KB RAM, ~40KB ROM

### Integration in ArduPilot

**Location:** `/libraries/AP_Networking/`

Provides unified networking abstraction layer for ArduPilot.

**Configuration** (`lwipopts.h`):
- TCP window size: 12KB
- TCP max segment: 1024 bytes
- PBUF pool: 120 buffers × 256 bytes
- Heap memory: 10KB
- 5 simultaneous TCP connections
- DHCP enabled
- IP forwarding enabled

**Socket API** (`SocketAPM`):
- BSD-like socket interface
- UDP/TCP clients and servers
- Non-blocking I/O with polling
- Multicast UDP operations

### Platforms Using lwIP

**ChibiOS-based STM32 boards:**
- Two backend modes:
  - **Ethernet** (`HAL_USE_MAC=1`): Direct Ethernet MAC driver
  - **PPP** (`HAL_USE_MAC=0`): Point-to-Point over UART

**Example boards:**
- BotBloxDroneNet
- NarinFC-H7
- LuminousBee5
- ESP32-based platforms

**SITL (Simulation):**
- Mixing lwip and native OS sockets

**Linux:**
- Enabled but uses OS networking

### Network Protocols

**Transport:**
- UDP: Lightweight datagram for telemetry
- TCP: Reliable connection for file transfers
- Raw: Low-level packet handling

**Network:**
- IPv4 (primary)
- ICMP (ping/diagnostics)
- ARP (address resolution)
- DHCP (dynamic IP)
- IGMP (multicast)

**Application:**
- PPP: Serial-based networking
- Sockets API: UDP/TCP (client/server)
- CAN Multicast: CAN-to-Ethernet bridging

**Advanced:**
- IP Forwarding: Routes between Ethernet and PPP
- Proxy ARP: PPP-to-Ethernet gateway
- Network Capture: PCAP packet capture (SITL)

### Build System Integration

**Location:** `/libraries/AP_Networking/wscript`

**Source Components:**
- Core: `modules/lwip/src/core/*` (IP/UDP/TCP stacks)
- IPv4: `modules/lwip/src/core/ipv4/*`
- API: `modules/lwip/src/api/*` (socket/netconn API)
- Network Interfaces: `modules/lwip/src/netif/*`
- PPP: `modules/lwip/src/netif/ppp/*`

**Custom HAL Bindings:**
- `/libraries/AP_Networking/lwip_hal/arch/*`
- Custom `lwipopts.h` configuration

### Key Features for ArduPilot

- Multithreaded TCPIP processing (dedicated network thread)
- Ethernet support (direct MAC driver integration)
- PPP Gateway mode (Ethernet-to-serial)
- Network port registration with AP_SerialManager
- DHCP support (automatic IP assignment)
- Static IP configuration
- Efficient file transmission (sendfile operations)

---

## 9. MAVLink

### Overview
MAVLink (Micro Air Vehicle Link) is **the** primary communication protocol for ArduPilot and ground control stations.

**Location:** `/modules/mavlink/`

### Purpose
- Standardized binary communication protocol for UAVs
- Lightweight header-only C library
- Version 2.0 with backward compatibility to v1.0
- Compact binary serialization (~5-8 bytes overhead)
- Support for up to 16 concurrent channels
- CRC-based validation and sequence numbering

### Protocol Features

**Wire Format:**
- Binary serialization for efficiency
- Message signing support (MAVLink 2.0)
- Extension fields
- Message routing

**Message Types:**
- 150+ standard messages
- Vehicle-specific extensions
- Custom ArduPilot messages

### Integration in ArduPilot

**Location:** `/libraries/GCS_MAVLink/`

MAVLink is the core communication framework for all ArduPilot vehicles.

**Main Classes:**
```cpp
class GCS_MAVLINK {
    void update_receive(uint32_t max_time_us=1000);
    void update_send();
    void send_message(enum ap_message id);
    void packetReceived(const mavlink_status_t &status,
                       const mavlink_message_t &msg);
};
```

**Dual Role:**
- **Server**: Receives commands from GCS, sends telemetry
- **Router**: Forwards MAVLink packets between components

### Message Definitions and Generation

**XML Definitions:**
**Location:** `/modules/mavlink/message_definitions/v1.0/`

- `common.xml` - Standard messages (50+ messages)
- `ardupilotmega.xml` - ArduPilot-specific (100+ messages)
- Specialized: `ASLUAV.xml`, `cubepilot.xml`, `icarous.xml`

**Example Message Definition:**
```xml
<message id="0" name="HEARTBEAT">
  <field type="uint8_t" name="type">Type of the MAV</field>
  <field type="uint8_t" name="autopilot">Autopilot type</field>
</message>
```

**Code Generation Pipeline:**

**Tool:** `/Tools/ardupilotwaf/mavgen.py`
1. Parses XML definitions
2. Generates C header files (`mavlink_msg_*.h`)
3. Creates encode/decode functions
4. Runs during WAF build process

**Internal Message Mapping:**

**Location:** `/libraries/GCS_MAVLink/ap_message.h`

101 internal message IDs that map to MAVLink messages:
```cpp
enum ap_message : uint8_t {
    MSG_HEARTBEAT = 0,
    MSG_ATTITUDE = 3,
    MSG_LOCATION = 5,
    MSG_VFR_HUD = 6,
    MSG_SYS_STATUS = 7,
    // ... 100+ message types
};
```

### Message Streaming

**9 Stream Categories:**
```cpp
enum streams : uint8_t {
    STREAM_RAW_SENSORS,
    STREAM_EXTENDED_STATUS,
    STREAM_RC_CHANNELS,
    STREAM_RAW_CONTROLLER,
    STREAM_POSITION,
    STREAM_EXTRA1,
    STREAM_EXTRA2,
    STREAM_EXTRA3,
    STREAM_PARAMS,
};
```

Each stream:
- Configurable rate (`SRn_*` parameters)
- Associated message groups
- Bucketing system for deferred sending

### Ground Control Station Integration

**Multi-Vehicle Support:**
- GCS singleton manages all communication
- Multiple GCS_MAVLINK instances for parallel connections
- File: `/libraries/GCS_MAVLink/GCS.h`

**Command Handling:**
- `handle_command_int()` - Structured commands
- `handle_command_long()` - Legacy format
- Examples: arm/disarm, mode changes, mission uploads

**MAVLink Routing:**

**Location:** `/libraries/GCS_MAVLink/MAVLink_routing.h`

```cpp
class MAVLink_routing {
    bool check_and_forward(GCS_MAVLINK &link,
                          const mavlink_message_t &msg);
    void send_to_components(uint32_t msgid,
                           const char *pkt, uint8_t pkt_len);
    // Routing table tracks components by sysid/compid
};
```

### Communication Flow

**Incoming:**
```
UART Reception → MAVLink Parser → packetReceived()
  → Routing check
  → Message dispatch (switch on MAVLINK_MSG_ID)
  → Handler function
```

**Outgoing:**
```
Mission Loop → update_send() on each GCS_MAVLINK
  → Stream rate checking
  → try_send_message(ap_message id)
  → MAVLink encode
  → UART transmit
```

### Key Message Patterns

- **Heartbeat**: Sent at ~1 Hz to maintain connection
- **Parameters**: Chunk-based request/response
- **Missions**: Multi-message upload/download protocol
- **Telemetry**: Grouped streams based on GCS configuration

### Advanced Features

**Message Signing:**
- Cryptographic signing (MAVLink 2.0)
- Prevents command injection
- Timestamp-based replay protection

**Alternative Protocols:**
- Can install alternative handlers
- Coexistence with LTM, Devo, FrSky telemetry

**Scripting Support:**
- Lua scripts can send/receive MAVLink
- Location: `/libraries/AP_Scripting/modules/MAVLink/`

**Analysis Tool:**
- `/Tools/scripts/mavlink_parse.py`
- Documents supported messages/commands
- Exports CSV/Markdown

---

## 10. Waf

### Overview
Waf (Waf build system) is ArduPilot's primary build system - a Python-based, flexible, scalable build tool.

**Location:** `/modules/waf/`

### Purpose
- Flexible build automation
- Python-based configuration and scripting
- Alternative to Make/CMake
- Dependency tracking
- Incremental compilation
- Cross-platform support

### Integration in ArduPilot

**Entry Point:** `/waf` (Python wrapper)
**Real WAF:** `/modules/waf/waf-light` (submodule)
**Root Configuration:** `/wscript` (1025+ lines)

### Build Flow

**Phase 1: Configuration**
```bash
./waf configure --board <board_name> [options]
```

**Configuration steps:**
1. Sets environment variables and build options
2. Loads board-specific configuration
3. Configures toolchain (compilers, linkers)
4. Generates `ap_config.h`
5. Stores configuration in `.lock` files

**Phase 2: Building**
```bash
./waf copter          # Build copter vehicle
./waf plane           # Build plane vehicle
./waf --targets bin/arducopter  # Build specific target
./waf check           # Build and run tests
```

**Build process:**
1. Recursively traverses wscript files
2. Creates task generators
3. Manages dependency tracking
4. Runs post-build steps

### Board Classes

**Location:** `/Tools/ardupilotwaf/boards.py`

```
Board (abstract base)
├── sitl (Software-In-The-Loop)
├── chibios (ARM embedded - ChibiOS RTOS)
├── linux (Linux-based boards)
├── esp32 (Espressif ESP32)
└── QURT (Qualcomm QURT RTOS)
```

### Board Definition Files

Each board has `hwdef.dat` specifying:
- Microcontroller type and features
- GPIO/peripheral pin mappings
- Memory layout (flash, RAM)
- Sensor configurations
- Special features

**Locations:**
- ChibiOS: `/libraries/AP_HAL_ChibiOS/hwdef/<board>/hwdef.dat`
- Linux: `/libraries/AP_HAL_Linux/hwdef/<board>/hwdef.dat`
- ESP32: `/libraries/AP_HAL_ESP32/hwdef/<board>/hwdef.dat`

### Configuration Options

```bash
# Board selection
--board <board_name>              # Target board
--board sitl                      # Default
--board CubeBlack                 # ChibiOS example
--board linux                     # Linux example

# Build type
--debug                           # Debug build
--debug-symbols                   # Debug symbols

# Features
--enable-scripting                # Lua scripting
--disable-scripting
--enable-benchmarks               # Benchmark tests
--disable-tests                   # Skip tests
--enable-networking               # Networking API

# Advanced
--toolchain <toolchain>           # Override toolchain
--coverage                        # Coverage instrumentation
--Werror                          # Warnings as errors
--bootloader                      # Build bootloader only
```

### Vehicle Types

Each vehicle has its own wscript and source:

1. **ArduCopter** (`/ArduCopter/wscript`)
   - Programs: `arducopter`, `arducopter-heli`

2. **ArduPlane** (`/ArduPlane/wscript`)
   - Program: `arduplane`

3. **Rover** (`/Rover/wscript`)
   - Program: `ardurover`

4. **ArduSub** (`/ArduSub/wscript`)
   - Program: `ardusub`

5. **Blimp** (`/Blimp/wscript`)
   - Program: `ardublimp`

6. **AntennaTracker** (`/AntennaTracker/wscript`)
   - Program: `antennatracker`

7. **Tools**
   - AP_Periph: Peripheral device firmware
   - AP_Bootloader: Bootloader firmware
   - Replay: Log replay tool

### Build Groups

Programs organized into groups:

```bash
# Main groups
bin           # Vehicle binaries (default)
tools         # Build tools
examples      # Library examples
tests         # Unit tests
benchmarks    # Performance benchmarks

# Vehicle groups (shortcuts)
copter        # All copter variants
plane         # Fixed-wing
rover         # Ground rovers
sub           # Submarines/ROVs
antennatracker # Antenna tracker
AP_Periph     # Peripheral devices
bootloader    # Bootloader
heli          # Helicopter variants
replay        # Log replay

# Special
all           # All programs
```

### Build Example

**Building ArduCopter for Pixhawk1:**

```bash
# Step 1: Configure
./waf configure --board Pixhawk1

# Step 2: Build
./waf copter
```

**Process:**
1. Sets BOARD=Pixhawk1
2. Loads `/libraries/AP_HAL_ChibiOS/hwdef/Pixhawk1/hwdef.dat`
3. Configures ARM toolchain (arm-none-eabi-g++)
4. Recursively processes wscript files
5. Compiles libraries into `ap` static library
6. Links to create `bin/arducopter`
7. Output: `build/Pixhawk1/bin/arducopter`

### Cross-Platform Support

**ChibiOS boards (ARM):**
- ChibiOS RTOS
- Statically linked
- Examples: Pixhawk, CubeBlack

**Linux boards (SBC):**
- Dynamically linked
- ARM and x86_64
- Examples: BeagleBone, Raspberry Pi

**ESP32 boards:**
- ESP-IDF framework
- ESP32 and ESP32-S3
- Examples: SparkFun

**SITL simulator:**
- Native host (x86_64, macOS)
- Includes physics simulation

### Key WAF Modules

**Location:** `/Tools/ardupilotwaf/`

- `ardupilotwaf.py` - Core helpers
- `boards.py` - Board definitions
- `chibios.py` - ChibiOS configuration
- `linux.py` - Linux configuration
- `esp32.py` - ESP32 configuration
- `toolchain.py` - Compiler setup
- `mavgen.py` - MAVLink generation
- `dronecangen.py` - DroneCAN generation
- `embed.py` - ROMFS embedding
- `gtest.py` - Google Test integration
- `gbenchmark.py` - Google Benchmark integration

### Build Output Structure

```
build/
├── <board>/              # Per-board directory
│   ├── ap_config.h       # Generated config
│   ├── bin/              # Main binaries
│   │   ├── arducopter
│   │   ├── arduplane
│   │   └── ardurover
│   ├── lib/              # Static libraries
│   ├── examples/         # Example programs
│   ├── tests/            # Test programs
│   ├── benchmarks/       # Benchmark programs
│   └── .waf/             # Build cache
```

### Summary

Waf provides ArduPilot with:
- Hardware abstraction through board classes
- Support for multiple vehicle types
- Cross-platform builds (embedded, desktop, simulator)
- Incremental compilation with dependency tracking
- Build customization via extensive options
- Logical build organization via groups
- Integrated code generation (MAVLink, DroneCAN)
- Seamless testing and benchmarking workflows

---

## Conclusion

The ArduPilot module ecosystem provides a comprehensive foundation for autopilot development:

- **ChibiOS** - Real-time operating system for embedded platforms
- **CrashDebug** - Post-mortem debugging for fault analysis
- **DroneCAN** - Distributed CAN-based peripheral communication
- **Micro-XRCE-DDS** - ROS 2 integration for modern robotics
- **gtest/gbenchmark** - Testing and performance validation
- **gSOAP** - ONVIF camera integration
- **LittleFS** - Reliable flash filesystem
- **lwIP** - Networking stack for Ethernet/PPP
- **MAVLink** - Primary telemetry and command protocol
- **Waf** - Flexible build system

Each module is carefully integrated to support ArduPilot's mission of providing a robust, feature-rich autopilot platform for unmanned vehicles.
