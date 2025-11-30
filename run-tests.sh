#!/bin/bash
#
# ArduPilot Test Runner
# Runs various test suites with selectable options
#

set -o pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Track failures
FAILURES=0

# Test flags
RUN_UNIT=false
RUN_PYTHON=false
RUN_SITL=""
RUN_BUILD_OPTIONS=false

print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

print_pass() {
    echo -e "${GREEN}[PASS]${NC} $1"
}

print_fail() {
    echo -e "${RED}[FAIL]${NC} $1"
}

print_info() {
    echo -e "${YELLOW}[INFO]${NC} $1"
}

usage() {
    cat << EOF
ArduPilot Test Runner

Usage: $0 [OPTIONS]

Options:
  --unit            Run C++ unit tests (Google Test)
  --python          Run Python pytest suite
  --sitl <vehicle>  Run SITL tests for vehicle (copter, plane, rover, sub, tracker, all)
  --build-options   Run build options validation
  --all             Run all tests (unit, python, and all SITL)
  --help            Show this help message

Examples:
  $0 --unit                    # Run unit tests only
  $0 --unit --python           # Run unit and Python tests
  $0 --sitl copter             # Run SITL tests for Copter
  $0 --sitl all                # Run SITL tests for all vehicles
  $0 --all                     # Run everything

Note: SITL tests can take 30+ minutes per vehicle.
EOF
}

run_unit_tests() {
    print_header "Running C++ Unit Tests"

    cd "$SCRIPT_DIR"

    # Configure if needed
    if [ ! -f "build/sitl/waf_config.py" ]; then
        print_info "Configuring build for SITL..."
        ./waf configure --board sitl
    fi

    print_info "Building unit tests..."
    if ! ./waf tests; then
        print_fail "Unit tests (build failed)"
        return 1
    fi

    print_info "Running unit tests..."
    local test_failures=0
    local test_count=0

    for test_bin in ./build/sitl/tests/test_*; do
        if [ -x "$test_bin" ]; then
            test_name=$(basename "$test_bin")
            ((test_count++))
            if "$test_bin" > /dev/null 2>&1; then
                echo -e "  ${GREEN}✓${NC} $test_name"
            else
                echo -e "  ${RED}✗${NC} $test_name"
                ((test_failures++))
            fi
        fi
    done

    echo ""
    if [ $test_failures -eq 0 ]; then
        print_pass "Unit tests ($test_count tests passed)"
        return 0
    else
        print_fail "Unit tests ($test_failures/$test_count failed)"
        return 1
    fi
}

run_python_tests() {
    print_header "Running Python Tests"

    cd "$SCRIPT_DIR"

    if ! command -v pytest &> /dev/null; then
        print_fail "pytest not found. Install with: pip install pytest"
        return 1
    fi

    if pytest tests/ -v; then
        print_pass "Python tests"
        return 0
    else
        print_fail "Python tests"
        return 1
    fi
}

run_sitl_tests() {
    local vehicle="$1"

    print_header "Running SITL Tests for $vehicle"

    cd "$SCRIPT_DIR"

    # Map vehicle name to proper case
    case "$vehicle" in
        copter|Copter)
            vehicle="Copter"
            ;;
        plane|Plane)
            vehicle="Plane"
            ;;
        rover|Rover)
            vehicle="Rover"
            ;;
        sub|Sub)
            vehicle="Sub"
            ;;
        tracker|Tracker)
            vehicle="Tracker"
            ;;
        *)
            print_fail "Unknown vehicle: $vehicle"
            return 1
            ;;
    esac

    print_info "Building and testing $vehicle..."
    if Tools/autotest/autotest.py "build.$vehicle" "test.$vehicle"; then
        print_pass "SITL tests for $vehicle"
        return 0
    else
        print_fail "SITL tests for $vehicle"
        return 1
    fi
}

run_build_options_tests() {
    print_header "Running Build Options Tests"

    cd "$SCRIPT_DIR"

    print_info "Testing build options (this may take a while)..."
    if Tools/autotest/test_build_options.py --board=sitl; then
        print_pass "Build options tests"
        return 0
    else
        print_fail "Build options tests"
        return 1
    fi
}

# Parse arguments
if [ $# -eq 0 ]; then
    usage
    exit 0
fi

while [[ $# -gt 0 ]]; do
    case $1 in
        --unit)
            RUN_UNIT=true
            shift
            ;;
        --python)
            RUN_PYTHON=true
            shift
            ;;
        --sitl)
            if [ -z "$2" ] || [[ "$2" == --* ]]; then
                echo "Error: --sitl requires a vehicle argument"
                exit 1
            fi
            RUN_SITL="$2"
            shift 2
            ;;
        --build-options)
            RUN_BUILD_OPTIONS=true
            shift
            ;;
        --all)
            RUN_UNIT=true
            RUN_PYTHON=true
            RUN_SITL="all"
            RUN_BUILD_OPTIONS=true
            shift
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

# Run selected tests
print_header "ArduPilot Test Runner"
echo "Selected tests:"
$RUN_UNIT && echo "  - Unit tests"
$RUN_PYTHON && echo "  - Python tests"
[ -n "$RUN_SITL" ] && echo "  - SITL tests ($RUN_SITL)"
$RUN_BUILD_OPTIONS && echo "  - Build options tests"
echo ""

if $RUN_UNIT; then
    run_unit_tests || ((FAILURES++))
fi

if $RUN_PYTHON; then
    run_python_tests || ((FAILURES++))
fi

if [ -n "$RUN_SITL" ]; then
    if [ "$RUN_SITL" = "all" ]; then
        for v in Copter Plane Rover Sub; do
            run_sitl_tests "$v" || ((FAILURES++))
        done
    else
        run_sitl_tests "$RUN_SITL" || ((FAILURES++))
    fi
fi

if $RUN_BUILD_OPTIONS; then
    run_build_options_tests || ((FAILURES++))
fi

# Summary
print_header "Test Summary"
if [ $FAILURES -eq 0 ]; then
    print_pass "All tests passed!"
    exit 0
else
    print_fail "$FAILURES test suite(s) failed"
    exit 1
fi
