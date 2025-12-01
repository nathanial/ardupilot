# SITL Cleanup Candidates

This document tracks potential files/directories that could be removed for a SITL-only build.

## Already Removed

### HAL Implementations
- `libraries/AP_HAL_ChibiOS/` - ChibiOS RTOS HAL
- `libraries/AP_HAL_ESP32/` - ESP32 HAL
- `libraries/AP_HAL_QURT/` - Qualcomm QURT HAL

### Firmware/Bootloader
- `Tools/AP_Bootloader/` - Bootloader utilities
- `Tools/IO_Firmware/` - IO board firmware
- `AP_Periph/` - Peripheral board firmware

### Build/CI
- `.github/` - GitHub workflows and CI/CD
- `Tools/ardupilotwaf/chibios.py` - ChibiOS build support
- `Tools/ardupilotwaf/esp32.py` - ESP32 build support
- `Tools/ardupilotwaf/qurt.py` - QURT build support

### Tools
- `Tools/simulink/` - Simulink integration
- `Tools/ros2/` - ROS2 integration
- `Tools/Vicon/` - Motion capture
- `Tools/Pozyx/` - UWB positioning
- `Tools/CHDK-Scripts/` - Canon camera control
- `Tools/geotag/` - Photo geotagging
- `Tools/Hello/` - Example tool
- `Tools/vagrant/` - Vagrant VM configs
- `Tools/environment_install/` - Legacy setup scripts
- `Tools/CPUInfo/` - CPU info tool
- `Tools/GIT_Test/` - Git testing
- `Tools/UDP_Proxy/` - UDP proxy

### Modules/Submodules
- `modules/ChibiOS/` - ChibiOS RTOS
- `modules/CrashDebug/` - Crash debugging
- `modules/Micro-CDR/` - DDS serialization
- `modules/Micro-XRCE-DDS-Client/` - DDS middleware
- `modules/gsoap/` - SOAP/WSDL tools
- `modules/gbenchmark/` - Google Benchmark (empty)
- `libraries/AP_GyroFFT/CMSIS_5/` - ARM math library

### Libraries (Medium Confidence)
- `libraries/AP_HAL_Linux/` - Linux embedded HAL (924K) - SITL uses AP_HAL_SITL, all includes guarded
- `libraries/AP_ONVIF/` - ONVIF camera protocol (2.8M) - opt-in only via --enable-onvif flag

### Libraries (Required Code Changes)
- `libraries/AP_PiccoloCAN/` - Piccolo CAN ESC (1.3M) - Added include guards, removed from waf
- `libraries/AP_KDECAN/` - KDE CAN ESC (50K) - Added include guards, removed from waf/wscripts
- `libraries/AP_IOMCU/` - IO coprocessor (100K) - Guarded in bindings.desc, removed from waf
- `libraries/AP_BLHeli/` - BLHeli ESC protocol (100K) - Added include guards, removed from waf
- `libraries/AP_FETtecOneWire/` - FETtec ESC (50K) - Added include guards, removed from waf
- `libraries/AP_Radio/` - Direct radio (280K) - AP_RADIO_ENABLED=0 by default, added `__has_include` guards
- `modules/lwip/` - Lightweight IP stack (8.8M) - Guarded with AP_NETWORKING_NEED_LWIP

### Navigation (EKF2 Removal)
- `libraries/AP_NavEKF2/` - Old EKF2 estimator (~4.5M) - NavEKF3 is current and sufficient

### Other
- `benchmarks/` - Benchmark stubs

### DroneCAN (Full Removal)
- `libraries/AP_DroneCAN/` - DroneCAN protocol library (~200KB)
- `modules/DroneCAN/` - DroneCAN submodule (~300KB)
- `libraries/SITL/SIM_DroneCANDevice.cpp/h` - SITL DroneCAN simulator
- `libraries/AP_Notify/DroneCAN_RGB_LED.cpp/h` - DroneCAN LED driver
- DroneCAN backend drivers in sensor libraries:
  - `libraries/AP_GPS/AP_GPS_DroneCAN.cpp/h`
  - `libraries/AP_Compass/AP_Compass_DroneCAN.cpp/h`
  - `libraries/AP_Baro/AP_Baro_DroneCAN.cpp/h`
  - `libraries/AP_BattMonitor/AP_BattMonitor_DroneCAN.cpp/h`
  - `libraries/AP_Airspeed/AP_Airspeed_DroneCAN.cpp/h`
  - `libraries/AP_RCProtocol/AP_RCProtocol_DroneCAN.cpp/h`
  - `libraries/AP_EFI/AP_EFI_DroneCAN.cpp/h`
  - `libraries/AP_RPM/RPM_DroneCAN.cpp/h`
  - `libraries/AP_TemperatureSensor/AP_TemperatureSensor_DroneCAN.cpp/h`
  - `libraries/AP_Mount/AP_Mount_Xacti.cpp/h`
  - `libraries/AP_OpenDroneID/AP_OpenDroneID_DroneCAN.cpp`

Code changes: Set `with_can = False` in `Tools/ardupilotwaf/boards.py` (SITL class), added `__has_include` guards in 9 files.

---

## High Confidence - Already Removed

These items from the original "High Confidence" list have been removed:

| Item | Size | Status |
|------|------|--------|
| `Tools/Linux_HAL_Essentials/` | 328K | ✅ Removed |
| `Tools/Frame_params/` | 708K | ✅ Removed |
| `Tools/cameras_gimbals/` | 32K | ✅ Removed |
| `Tools/FilterTestTool/` | 44K | ✅ Removed |
| `Tools/terrain-tools/` | 20K | ✅ Removed |
| `Tools/mavproxy_modules/` | 40K | ✅ Removed |

**Note:** `libraries/AP_HAL_Empty/` is referenced by AP_HAL_SITL and must be kept.

---

## Code Changes Made to Enable Removal

The following libraries required code changes before they could be removed. These changes have been applied:

### Files Modified (Previous Sessions)

1. **`libraries/AP_CANManager/AP_CANManager.cpp`** - Added `__has_include` guards for AP_KDECAN and AP_PiccoloCAN
2. **`libraries/AP_CANManager/AP_CANManager_CANDriver_Params.cpp`** - Added guards for AP_KDECAN
3. **`libraries/AP_Generator/AP_Generator_config.h`** - Guarded AP_PiccoloCAN_config.h include
4. **`libraries/SRV_Channel/SRV_Channel.h`** - Guarded AP_BLHeli and AP_FETtecOneWire includes
5. **`libraries/SRV_Channel/SRV_Channels.cpp`** - Guarded AP_BLHeli and AP_FETtecOneWire includes
6. **`libraries/AP_Arming/AP_Arming.cpp`** - Guarded AP_BLHeli include
7. **`libraries/AP_Notify/AP_Notify.h`** - Guarded AP_IOMCU include with HAL_WITH_IO_MCU
8. **`libraries/AP_Notify/ProfiLED_IOMCU.h`** - Guarded AP_IOMCU include
9. **`libraries/AP_BoardConfig/IMU_heater.cpp`** - Guarded AP_IOMCU include
10. **`libraries/AP_Logger/AP_Logger_Backend.cpp`** - Guarded AP_IOMCU include
11. **`libraries/AP_Vehicle/AP_Vehicle.h`** - Guarded KDECAN and BLHeli includes
12. **`libraries/AP_Vehicle/AP_Vehicle.cpp`** - Guarded KDECAN include
13. **`libraries/AP_EFI/AP_EFI_config.h`** - Made CURRAWONG_ECU depend on HAL_PICCOLO_CAN_ENABLE
14. **`libraries/GCS_MAVLink/GCS_Common.cpp`** - Guarded PiccoloCAN, KDECAN, BLHeli includes
15. **`libraries/AP_Networking/AP_Networking.cpp`** - Wrapped lwip includes with AP_NETWORKING_NEED_LWIP
16. **`libraries/AP_Scripting/generator/description/bindings.desc`** - Made AP_IOMCU include conditional
17. **`Tools/ardupilotwaf/ardupilotwaf.py`** - Removed libraries from dependency lists
18. **`ArduCopter/wscript`** - Removed AP_KDECAN from ap_libraries
19. **`Blimp/wscript`** - Removed AP_KDECAN from ap_libraries

### Files Modified (AP_NavEKF2 Removal)

1. **`Tools/ardupilotwaf/ardupilotwaf.py`** - Removed AP_NavEKF2 from COMMON_VEHICLE_DEPENDENT_LIBRARIES
2. **`Tools/ardupilotwaf/boards.py`** - Removed HAL_NAVEKF2_AVAILABLE=1 override for SITL board
3. **`Tools/scripts/build_options.py`** - Removed EKF2 feature option
4. **`Tools/scripts/extract_features.py`** - Removed EKF2 feature detection
5. **`libraries/AP_DAL/examples/AP_DAL_Standalone/wscript`** - Removed AP_NavEKF2 from library list
6. **`libraries/AP_AHRS/AP_AHRS_config.h`** - Set HAL_NAVEKF2_AVAILABLE=0 (compile-out guard)
7. **`libraries/AP_AHRS/AP_AHRS.h`** - Removed EKF2 include, NavEKF2 member, EKFType::TWO enum
8. **`libraries/AP_AHRS/AP_AHRS.cpp`** - Simplified EKF selection to auto-migrate type=2 to EKF3
9. **`ArduPlane/Plane.h`** - Removed AP_NavEKF2 include
10. **`libraries/AP_Logger/LogStructure.h`** - Removed EKF2 log structure include and LOG_IDS_FROM_NAVEKF2
11. **`libraries/AP_NavEKF/LogStructure.h`** - Removed NKY0/NKY1 (EKF2) log message entries
12. **`Tools/Replay/Replay.cpp`** - Emptied EKF2_log_structures array

### Files Modified (AP_RangeFinder Hardware Driver Removal)

Removed 85 hardware driver files from `libraries/AP_RangeFinder/`, keeping only:
- `AP_RangeFinder.cpp/h` - Core library
- `AP_RangeFinder_Backend.cpp/h` - Base backend class
- `AP_RangeFinder_config.h` - Configuration
- `AP_RangeFinder_Params.cpp/h` - Parameters
- `AP_RangeFinder_SITL.cpp/h` - SITL backend

Code changes:
1. **`libraries/AP_RangeFinder/AP_RangeFinder.cpp`** - Added `__has_include` guards with macro disabling for all hardware drivers, added `HAS_RANGEFINDER_SERIAL_BACKENDS` guard, added `default:` case to switch
2. **`libraries/AP_RangeFinder/AP_RangeFinder_config.h`** - Added `__has_include` check for MSP backend
3. **`libraries/AP_DroneCAN/AP_DroneCAN.cpp`** - Added `__has_include` guard for AP_RangeFinder_DroneCAN with macro disabling

### Files Modified (AP_OpticalFlow Hardware Driver Removal)

Removed 17 hardware driver files from `libraries/AP_OpticalFlow/`, keeping only:
- `AP_OpticalFlow.cpp/h` - Core library
- `AP_OpticalFlow_Backend.cpp/h` - Base backend class
- `AP_OpticalFlow_config.h` - Configuration
- `AP_OpticalFlow_Calibrator.cpp/h` - Calibration
- `AP_OpticalFlow_SITL.cpp/h` - SITL backend

Code changes:
1. **`libraries/AP_OpticalFlow/AP_OpticalFlow.cpp`** - Added `__has_include` guards with macro disabling for all hardware drivers, added `default:` case to switch
2. **`libraries/AP_OpticalFlow/AP_OpticalFlow_config.h`** - Added `__has_include` check for MSP backend
3. **`libraries/AP_DroneCAN/AP_DroneCAN.cpp`** - Added `__has_include` guard for AP_OpticalFlow_HereFlow

### Files Modified (AP_Proximity Hardware Driver Removal)

Removed 24 hardware driver files from `libraries/AP_Proximity/`, keeping only:
- `AP_Proximity.cpp/h` - Core library
- `AP_Proximity_Backend.cpp/h` - Base backend class
- `AP_Proximity_Backend_Serial.cpp/h` - Serial backend base
- `AP_Proximity_config.h` - Configuration
- `AP_Proximity_Params.cpp/h` - Parameters
- `AP_Proximity_Utils.cpp` - Utilities
- `AP_Proximity_Boundary_3D.cpp/h` - Boundary processing
- `LogStructure.h` - Logging
- `AP_Proximity_SITL.cpp/h` - SITL backend
- `AP_Proximity_AirSimSITL.cpp/h` - AirSim SITL backend

Code changes:
1. **`libraries/AP_Proximity/AP_Proximity.cpp`** - Added `__has_include` guards with macro disabling for all hardware drivers, added `default:` case to switch
2. **`libraries/AP_DroneCAN/AP_DroneCAN.cpp`** - Added `__has_include` guard for AP_Proximity_DroneCAN

### Files Modified (AP_Radio Removal)

Removed entire `libraries/AP_Radio/` directory (16 files):
- `AP_Radio.cpp/h` - Core library
- `AP_Radio_backend.cpp/h` - Backend base class
- `AP_Radio_config.h` - Configuration
- `AP_Radio_bk2425.cpp/h`, `AP_Radio_cc2500.cpp/h`, `AP_Radio_cypress.cpp/h` - Hardware drivers
- `driver_bk2425.cpp/h`, `driver_cc2500.cpp/h` - Low-level drivers
- `telem_structure.h` - Telemetry structures

Code changes:
1. **`libraries/AP_RCProtocol/AP_RCProtocol_config.h`** - Added `__has_include` guard for AP_Radio_config.h
2. **`libraries/AP_BoardConfig/AP_BoardConfig.h`** - Added `__has_include` guard for AP_Radio_config.h
3. **`Tools/ardupilotwaf/ardupilotwaf.py`** - Removed AP_Radio from COMMON_VEHICLE_DEPENDENT_LIBRARIES

Note: `AP_RADIO_ENABLED` already defaulted to `0`, all external references were properly guarded.

### Files Modified (AP_Beacon Hardware Driver Removal)

Removed 10 files from `libraries/AP_Beacon/` (3 hardware drivers + examples), keeping:
- `AP_Beacon.cpp/h` - Core library
- `AP_Beacon_Backend.cpp/h` - Base backend class
- `AP_Beacon_config.h` - Configuration
- `LogStructure.h` - Logging
- `AP_Beacon_SITL.cpp/h` - SITL backend
- `sitl/sitl_beacons.param` - SITL params

Code changes:
1. **`libraries/AP_Beacon/AP_Beacon_config.h`** - Set hardware backends (MARVELMIND, NOOPLOOP, POZYX) to 0
2. **`libraries/AP_Beacon/AP_Beacon.cpp`** - Added `#if` guards around hardware driver includes and switch cases

Files deleted:
- `AP_Beacon_Marvelmind.cpp/h` - Marvelmind driver
- `AP_Beacon_Nooploop.cpp/h` - Nooploop driver
- `AP_Beacon_Pozyx.cpp/h` - Pozyx driver
- `examples/AP_Marvelmind_test/` - Test example (4 files)

### Files Modified (AP_Airspeed Hardware Driver Removal)

Removed 18 files from `libraries/AP_Airspeed/` (7 I2C drivers + examples + models), keeping:
- `AP_Airspeed.cpp/h` - Core library
- `AP_Airspeed_Backend.cpp/h` - Base backend class
- `AP_Airspeed_config.h` - Configuration
- `AP_Airspeed_Params.cpp` - Parameters
- `AP_Airspeed_Health.cpp` - Health checking
- `Airspeed_Calibration.cpp` - Calibration
- `AP_Airspeed_SITL.cpp/h` - SITL simulation
- `AP_Airspeed_analog.cpp/h` - **Required for SITL** (TYPE_ANALOG default)
- `AP_Airspeed_MSP.cpp/h` - MSP protocol
- `AP_Airspeed_External.cpp/h` - External AHRS

Code changes:
1. **`libraries/AP_Airspeed/AP_Airspeed_config.h`** - Set hardware backends (MS4525, MS5525, SDP3X, DLVR, ASP5033, AUAV, NMEA) to 0
2. **`libraries/AP_Airspeed/AP_Airspeed.cpp`** - Added `#if` guards around hardware driver includes

Files deleted:
- `AP_Airspeed_MS4525.cpp/h` - MS4525 I2C sensor
- `AP_Airspeed_MS5525.cpp/h` - MS5525 I2C sensor
- `AP_Airspeed_SDP3X.cpp/h` - SDP3X I2C sensor
- `AP_Airspeed_DLVR.cpp/h` - DLVR I2C sensor
- `AP_Airspeed_ASP5033.cpp/h` - ASP5033 I2C sensor
- `AP_Airspeed_AUAV.cpp/h` - AUAV I2C sensor
- `AP_Airspeed_NMEA.cpp/h` - NMEA water speed
- `examples/Airspeed/` - Example (2 files)
- `models/` - MATLAB models (2 files)

### Files Modified (AP_RPM Hardware Driver Removal)

Removed 5 files from `libraries/AP_RPM/` (1 hardware driver + examples), keeping:
- `AP_RPM.cpp/h` - Core library
- `RPM_Backend.cpp/h` - Base backend class
- `AP_RPM_config.h` - Configuration
- `AP_RPM_Params.cpp/h` - Parameters
- `RPM_SITL.cpp/h` - SITL backend
- `RPM_EFI.cpp/h` - EFI integration (feature-dependent)
- `RPM_ESC_Telem.cpp/h` - ESC telemetry (feature-dependent)
- `RPM_Generator.cpp/h` - Generator (feature-dependent)
- `RPM_HarmonicNotch.cpp/h` - Harmonic notch (feature-dependent)

Code changes:
1. **`libraries/AP_RPM/AP_RPM_config.h`** - Set `AP_RPM_PIN_ENABLED` to 0
2. **`libraries/AP_RPM/AP_RPM.cpp`** - Added `#if` guard around `RPM_Pin.h` include

Files deleted:
- `RPM_Pin.cpp/h` - Pin/PWM hardware driver
- `examples/ArduinoHallEffectDebug.ino` - Arduino example
- `examples/RPM_generic/RPM_generic.cpp` - RPM example
- `examples/RPM_generic/wscript` - Example wscript

### Files Modified (AP_TemperatureSensor Hardware Driver Removal)

Removed 14 files from `libraries/AP_TemperatureSensor/` (7 hardware drivers), keeping:
- `AP_TemperatureSensor.cpp/h` - Core library
- `AP_TemperatureSensor_Backend.cpp/h` - Base backend class
- `AP_TemperatureSensor_config.h` - Configuration
- `AP_TemperatureSensor_Params.cpp/h` - Parameters

**Note:** No SITL backend exists - all hardware drivers removed.

Code changes:
1. **`libraries/AP_TemperatureSensor/AP_TemperatureSensor_config.h`** - Added missing macros (TSYS01, TSYS03, MCP9600), set all hardware backends to 0
2. **`libraries/AP_TemperatureSensor/AP_TemperatureSensor.cpp`** - Added unconditional `#include "AP_TemperatureSensor_Backend.h"`, added `#if` guards around all hardware driver includes

Files deleted:
- `AP_TemperatureSensor_TSYS01.cpp/h` - TSYS01 I2C sensor
- `AP_TemperatureSensor_TSYS03.cpp/h` - TSYS03 I2C sensor
- `AP_TemperatureSensor_MCP9600.cpp/h` - MCP9600 I2C thermocouple
- `AP_TemperatureSensor_MAX31865.cpp/h` - MAX31865 SPI RTD
- `AP_TemperatureSensor_Analog.cpp/h` - Analog thermistor
- `AP_TemperatureSensor_MLX90614.cpp/h` - MLX90614 IR sensor
- `AP_TemperatureSensor_SHT3x.cpp/h` - SHT3x humidity/temp sensor

### Pattern Used

For each removed library, includes were wrapped with `__has_include`:
```cpp
#if __has_include(<AP_SomeLib/AP_SomeLib.h>)
#include <AP_SomeLib/AP_SomeLib.h>
#else
#undef AP_SOMELIB_ENABLED
#define AP_SOMELIB_ENABLED 0
#endif
```

---

## Sensor Hardware Drivers - Completed

The following sensor libraries had hardware-specific drivers removed using `__has_include` guards:

| Library | Files Removed | Files Kept | Notes |
|---------|--------------|------------|-------|
| `libraries/AP_GPS/` | 16 files | SITL, MAV, MSP, ExternalAHRS, Blended, **UBLOX** | UBLOX required - SIM_GPS defaults to UBLOX protocol |
| `libraries/AP_Baro/` | 26 files | SITL, MSP, ExternalAHRS, Dummy, Wind | SimpleUnderWaterAtmosphere moved to AP_Baro.cpp |
| `libraries/AP_InertialSensor/` | 27 files | SITL, NONE, ExternalAHRS | |
| `libraries/AP_Compass/` | 36 files | SITL, MSP, ExternalAHRS, Calibration | |
| `libraries/AP_BattMonitor/` | 35+ files | Sum, Scripting, **Analog** | Analog required - SITL uses it for battery simulation |

**Total removed: ~140 hardware driver files**

### Critical SITL Dependencies Discovered

**These backends MUST be kept for SITL to function:**

1. **AP_GPS_UBLOX** - SIM_GPS module defaults to UBLOX protocol. Without it, GPS auto-detection fails.
2. **AP_BattMonitor_Analog** - SITL simulates battery via analog voltage/current readings.
3. **AP_Baro::SimpleUnderWaterAtmosphere()** - Was in AP_Baro_HIL.cpp (deleted), moved to AP_Baro.cpp for submarine SITL.
4. **AP_Airspeed_analog** - SITL uses TYPE_ANALOG as default airspeed type for ArduPlane.

### Files Modified

1. **`libraries/AP_GPS/AP_GPS.cpp`** - Added `__has_include` guards for 9 hardware drivers
2. **`libraries/AP_GPS/AP_GPS_config.h`** - Added `__has_include` guards for SBF and ERB (affects RTK macros)
3. **`libraries/AP_Baro/AP_Baro.cpp`** - Added `__has_include` guards for 13 hardware drivers, added SimpleUnderWaterAtmosphere function
4. **`libraries/AP_InertialSensor/AP_InertialSensor.cpp`** - Added `__has_include` guards for 12 hardware drivers
5. **`libraries/AP_Compass/AP_Compass.cpp`** - Added `__has_include` guards for 18 hardware drivers
6. **`libraries/AP_BattMonitor/AP_BattMonitor.cpp`** - Added `__has_include` guards for 21 hardware drivers

---

## Lower Confidence - Need Investigation

| Item | Size | Reason |
|------|------|--------|
| `modules/gtest/` | 4.4M | Google Test - only if you don't run unit tests |
| `Tools/Replay/` | 120K | Log replay tool - useful for debugging |

---

## Keep - Required for SITL

| Item | Reason |
|------|--------|
| `libraries/SITL/` (42M) | Core simulator physics/models |
| `libraries/AP_HAL_SITL/` | SITL HAL implementation |
| `libraries/AP_NavEKF/` | Shared EKF code (used by EKF3, includes EKFGSF_yaw) |
| `libraries/AP_NavEKF3/` | Current EKF implementation |
| `Tools/autotest/` | SITL test framework |
| `Tools/ardupilotwaf/` | Build system |
| `Tools/scripts/` | Build/utility scripts |
| `modules/mavlink/` | MAVLink protocol (essential) |
| `modules/waf/` | Build system |
| `modules/littlefs/` | Filesystem support |
| `docs/` | Documentation |

---

## Notes

- Building all vehicles in parallel (`./waf plane copter ...`) has a pre-existing linker issue with ArduPlane
- Use `./build_all.sh` to build vehicles sequentially instead
- A convenience script `run_sitl.sh` is available at the repo root to launch ArduPlane SITL

## Removal Summary

| Session | Items Removed | Approx Size |
|---------|---------------|-------------|
| Initial | HALs, Tools, Modules, etc. | ~50MB+ |
| High Confidence | 6 Tools directories | ~1.2MB |
| Medium Confidence | AP_HAL_Linux, AP_ONVIF | ~3.7MB |
| Code Changes Required | AP_PiccoloCAN, AP_KDECAN, AP_IOMCU, AP_BLHeli, AP_FETtecOneWire, modules/lwip | ~10.4MB |
| EKF2 Removal | libraries/AP_NavEKF2 | ~4.5MB |
| Sensor Drivers | AP_RangeFinder hardware drivers (85 files) | ~500KB |
| Sensor Drivers | AP_OpticalFlow hardware drivers (17 files) | ~150KB |
| Sensor Drivers | AP_Proximity hardware drivers (24 files) | ~200KB |
| DroneCAN | libraries/AP_DroneCAN, modules/DroneCAN, backend drivers | ~650KB |
| Sensor Drivers | AP_GPS, AP_Baro, AP_InertialSensor, AP_Compass, AP_BattMonitor (~140 files) | ~2MB |
| Standalone Library | AP_Radio (16 files) | ~280KB |
| Sensor Drivers | AP_Beacon hardware drivers (10 files) | ~100KB |
| Sensor Drivers | AP_Airspeed hardware drivers (18 files) | ~180KB |
| Sensor Drivers | AP_RPM hardware drivers (5 files) | ~50KB |
| Sensor Drivers | AP_TemperatureSensor hardware drivers (14 files) | ~100KB |

**Total saved: ~73.4MB+**
