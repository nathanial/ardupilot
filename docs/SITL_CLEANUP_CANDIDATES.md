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
- `modules/lwip/` - Lightweight IP stack (8.8M) - Guarded with AP_NETWORKING_NEED_LWIP

### Navigation (EKF2 Removal)
- `libraries/AP_NavEKF2/` - Old EKF2 estimator (~4.5M) - NavEKF3 is current and sufficient

### Other
- `benchmarks/` - Benchmark stubs

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

## Future Cleanup Candidates - Sensor Hardware Drivers

The following sensor libraries contain hardware-specific drivers that could be removed using the same pattern as AP_RangeFinder. Each library has a SITL backend that must be kept.

| Library | Est. Files to Remove | SITL Backend |
|---------|---------------------|--------------|
| `libraries/AP_Compass/` | ~40 files | AP_Compass_SITL.cpp/h |
| `libraries/AP_Baro/` | ~34 files | AP_Baro_SITL.cpp/h |
| `libraries/AP_GPS/` | ~19 files | AP_GPS_SITL.cpp/h |
| `libraries/AP_InertialSensor/` | ~21 files | AP_InertialSensor_SITL.cpp/h |
| `libraries/AP_OpticalFlow/` | ~11 files | AP_OpticalFlow_SITL.cpp/h |
| `libraries/AP_Proximity/` | ~12 files | AP_Proximity_SITL.cpp/h |
| `libraries/AP_BattMonitor/` | ~26 files | (uses Analog backend for SITL) |

**Estimated total: ~163 additional hardware driver files**

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
| `modules/DroneCAN/` | DroneCAN support |
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

**Total saved: ~70MB+**

**Potential additional savings:** Removing hardware drivers from remaining sensor libraries (AP_Compass, AP_Baro, AP_GPS, AP_InertialSensor, AP_OpticalFlow, AP_Proximity, AP_BattMonitor) could save an additional ~2-3MB and significantly simplify the codebase for understanding.
