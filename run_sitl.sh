#!/bin/bash
# Run ArduPlane SITL
# Connect via TCP port 5760 (e.g., QGroundControl or MAVProxy)

cd "$(dirname "$0")"

# Kill any existing instances
pkill -f arduplane 2>/dev/null

# Run SITL
Tools/autotest/sim_vehicle.py -v ArduPlane -N --no-mavproxy "$@"
