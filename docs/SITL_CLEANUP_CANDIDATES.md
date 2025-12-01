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

### Other
- `benchmarks/` - Benchmark stubs

---

## High Confidence - Safe to Remove

| Item | Size | Reason |
|------|------|--------|
| `Tools/Linux_HAL_Essentials/` | 328K | PRU/devicetree files for BeagleBone, not needed for SITL |
| `Tools/Frame_params/` | 708K | Hardware frame parameters for specific drones |
| `Tools/cameras_gimbals/` | 32K | Camera/gimbal hardware parameters |
| `Tools/FilterTestTool/` | 44K | Standalone filter testing tool |
| `Tools/terrain-tools/` | 20K | Terrain data generation tools |
| `Tools/mavproxy_modules/` | 40K | MAVProxy extensions (external tool) |

**Note:** `libraries/AP_HAL_Empty/` is referenced by AP_HAL_SITL and must be kept.

---

## Medium Confidence - Likely Safe

| Item | Size | Reason |
|------|------|--------|
| `libraries/AP_HAL_Linux/` | 924K | Linux embedded HAL - SITL uses AP_HAL_SITL instead. Need to verify no cross-dependencies |
| `libraries/AP_ONVIF/` | 2.8M | ONVIF camera protocol (IP cameras) - likely disabled for SITL |
| `libraries/AP_PiccoloCAN/` | 1.3M | Piccolo CAN protocol for specific ESCs |
| `libraries/AP_IOMCU/` | ~100K | IO co-processor communication (Pixhawk-specific) |
| `libraries/AP_BLHeli/` | ~100K | BLHeli ESC passthrough (hardware ESCs) |
| `libraries/AP_FETtecOneWire/` | ~50K | FETtec ESC protocol |
| `libraries/AP_KDECAN/` | ~50K | KDE CAN ESCs |
| `modules/lwip/` | 8.8M | Lightweight IP stack - check if SITL networking uses this |

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
