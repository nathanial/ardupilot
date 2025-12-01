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

## Cannot Remove Without Code Changes

These libraries compile to nothing or are disabled for SITL, but have unguarded includes or build system dependencies that prevent simple removal:

| Item | Size | Blocker |
|------|------|---------|
| `libraries/AP_PiccoloCAN/` | 1.3M | Unguarded includes in AP_CANManager, AP_Generator, AP_EFI; listed in ardupilotwaf.py |
| `libraries/AP_IOMCU/` | ~100K | Listed in COMMON_VEHICLE_DEPENDENT_LIBRARIES in ardupilotwaf.py |
| `libraries/AP_BLHeli/` | ~100K | Listed in COMMON_VEHICLE_DEPENDENT_LIBRARIES in ardupilotwaf.py |
| `libraries/AP_FETtecOneWire/` | ~50K | Listed in COMMON_VEHICLE_DEPENDENT_LIBRARIES in ardupilotwaf.py |
| `libraries/AP_KDECAN/` | ~50K | Unguarded includes in AP_CANManager; listed in ardupilotwaf.py |
| `modules/lwip/` | 8.8M | AP_Networking.cpp includes lwipopts.h unconditionally when AP_NETWORKING_ENABLED (true for SITL) |

**To remove these**, you would need to:
1. Add compile guards around the includes
2. Remove entries from `Tools/ardupilotwaf/ardupilotwaf.py` library lists

---

## Lower Confidence - Need Investigation

| Item | Size | Reason |
|------|------|--------|
| `modules/gtest/` | 4.4M | Google Test - only if you don't run unit tests |
| `Tools/Replay/` | 120K | Log replay tool - useful for debugging |
| `libraries/AP_NavEKF/` + `AP_NavEKF2/` | 4.5M | Old EKF versions - NavEKF3 is current, but might be needed for compatibility |

---

## Keep - Required for SITL

| Item | Reason |
|------|--------|
| `libraries/SITL/` (42M) | Core simulator physics/models |
| `libraries/AP_HAL_SITL/` | SITL HAL implementation |
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

## Removal Summary

| Session | Items Removed | Approx Size |
|---------|---------------|-------------|
| Initial | HALs, Tools, Modules, etc. | ~50MB+ |
| High Confidence | 6 Tools directories | ~1.2MB |
| Medium Confidence | AP_HAL_Linux, AP_ONVIF | ~3.7MB |
