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

### Files Modified

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
| Code Changes Required | AP_PiccoloCAN, AP_KDECAN, AP_IOMCU, AP_BLHeli, AP_FETtecOneWire, modules/lwip | ~10.4MB |

**Total saved: ~65MB+**
