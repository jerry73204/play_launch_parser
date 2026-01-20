#!/usr/bin/env bash
#
# Run all comparison tests
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=============================================================================="
echo "RUST vs PYTHON COMPARISON TESTS"
echo "=============================================================================="
echo ""

# Check if Rust binary exists
RUST_BIN="../../src/play_launch_parser/target/release/play_launch_parser"
if [ ! -f "$RUST_BIN" ]; then
    echo "Error: Rust binary not found at $RUST_BIN"
    echo "Please run: cargo build --release --features python"
    exit 1
fi

# Test 1: Simple test
echo "Test 1: Simple namespace scoping test"
echo "--------------------------------------"
python3 compare_rust_python.py test_simple.launch.xml
test1_result=$?
echo ""

# Test 2: Nested namespace test
echo "Test 2: Nested namespace scoping test"
echo "--------------------------------------"
python3 compare_rust_python.py test_group_namespace.launch.xml
test2_result=$?
echo ""

# Summary
echo "=============================================================================="
echo "TEST SUMMARY"
echo "=============================================================================="
echo ""

if [ $test1_result -eq 0 ]; then
    echo "✓ Test 1: PASSED (simple namespace scoping)"
else
    echo "✗ Test 1: FAILED (simple namespace scoping)"
fi

if [ $test2_result -eq 0 ]; then
    echo "✓ Test 2: PASSED (nested namespace scoping)"
else
    echo "✗ Test 2: FAILED (nested namespace scoping)"
fi

echo ""

if [ $test1_result -eq 0 ] && [ $test2_result -eq 0 ]; then
    echo "✓ All tests passed!"
    exit 0
else
    echo "✗ Some tests failed"
    exit 1
fi
