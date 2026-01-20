# Autoware Planning Simulator Test

This directory contains real-world tests using Autoware's planning simulator launch file as a testbed for the Rust parser.

## Overview

Tests the Rust `play_launch_parser` against Autoware's production launch files to verify:
- Parsing accuracy with complex, real-world launch files
- Output compatibility with Python `dump_launch`
- Performance on production-scale launch files

## Setup

1. **Create the autoware symlink** pointing to your Autoware workspace:
   ```bash
   cd tests/autoware_test
   ln -s /path/to/your/autoware autoware
   ```

   The symlink should point to a built Autoware workspace (e.g., `/home/aeon/repos/autoware/2025.02-ws`).

2. **Ensure Python dump_launch is available** (for comparison):
   ```bash
   # Should be available if play_launch is installed
   which dump_launch
   ```

## Test Structure

```
tests/autoware_test/
├── README.md                # This file
├── autoware                 # Symlink to Autoware workspace
├── scripts/
│   ├── test_parse.sh       # Test Rust parser against Autoware launch file
│   ├── compare_outputs.py  # Compare Rust vs Python output
│   └── benchmark.sh        # Performance benchmarking
└── output/                 # Generated test outputs (gitignored)
    ├── rust_output.json    # Rust parser output
    ├── python_output.json  # Python dump_launch output
    └── comparison.txt      # Diff results
```

## Usage

### Quick Test

```bash
# Run comparison with default Autoware launch file
cd tests/autoware_test/scripts
./compare_rust_python.py
```

This will automatically:
1. Use `planning_simulator.launch.xml` as the default test file
2. Run both Rust and Python parsers
3. Compare outputs and report differences
4. Save results to `rust_output.json` and `python_output.json`

### Custom Launch File

```bash
# Test with a specific launch file
./compare_rust_python.py /path/to/test.launch.xml

# Test with a different Autoware launch file
./compare_rust_python.py autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml
```

## Test Files

The primary test file is:
- `autoware_launch/launch/planning_simulator.launch.xml` (111 lines)
  - Complex real-world launch file
  - Multiple arguments with defaults
  - Nested includes
  - Conditional logic
  - Group scoping
  - Let variables
  - find-pkg-share substitutions

## Success Criteria

- ✅ Rust parser successfully parses Autoware launch file
- ✅ Output structure matches Python dump_launch
- ✅ All nodes, arguments, and parameters correctly extracted
- ✅ Parse time < 10ms (significantly faster than Python)
- ✅ Memory usage minimal (< 50MB)

## Notes

- This test uses production Autoware launch files to ensure real-world compatibility
- The test is optional and requires Autoware to be installed
- Results help validate that the Rust parser is production-ready
