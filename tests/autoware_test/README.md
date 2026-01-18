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
# Run the basic parsing test
just test-autoware
```

This will:
1. Parse Autoware's `planning_simulator.launch.xml` with Rust parser
2. Parse the same file with Python dump_launch
3. Compare outputs and report differences

### Manual Testing

```bash
# Parse with Rust parser
./scripts/test_parse.sh

# Compare outputs
./scripts/compare_outputs.py

# Run performance benchmark
./scripts/benchmark.sh
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
