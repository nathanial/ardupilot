#!/bin/bash
# Build all SITL vehicle types sequentially
# This avoids linker issues that occur when building all vehicles in parallel

set -e  # Exit on first error

echo "=== Cleaning build directory ==="
./waf distclean

echo "=== Configuring for SITL ==="
./waf configure --board sitl

VEHICLES="copter plane heli rover sub blimp antennatracker"

for vehicle in $VEHICLES; do
    echo "=== Building $vehicle ==="
    ./waf $vehicle
done

echo "=== Build complete ==="
ls -la build/sitl/bin/
