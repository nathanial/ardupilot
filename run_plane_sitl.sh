#!/bin/bash
# Script to initialize submodules, build, and run ArduPlane SITL

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== Initializing git submodules ==="
git submodule update --init --recursive

echo "=== Configuring waf for SITL ==="
./waf configure --board sitl

echo "=== Building ArduPlane ==="
./waf plane

echo "=== Starting ArduPlane SITL ==="
python3 Tools/autotest/sim_vehicle.py -v ArduPlane --no-rebuild "$@"
