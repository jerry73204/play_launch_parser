# Test Directory

This directory contains test fixtures and testing utilities for the play_launch_parser project.

## Directory Structure

```
tests/
├── fixtures/
│   ├── launch/         # Main test launch files
│   └── includes/       # Launch files to be included by other launch files
├── autoware_test/      # Autoware-specific testing
│   ├── scripts/        # Test and comparison scripts
│   │   ├── activate_autoware.sh          # User-specific workspace activation (git-ignored)
│   │   ├── activate_autoware.sh.template # Template for activation script
│   │   └── compare_rust_python.py        # Autoware comparison script
│   └── output/         # Test output files
├── comparison_tests/   # Rust vs Python comparison tests
│   ├── compare_rust_python.py  # Main comparison script
│   ├── test_simple.launch.xml  # Simple test case
│   ├── test_group_namespace.launch.xml  # Namespace test
│   └── run_tests.sh    # Run all comparison tests
└── README.md
```

## Test Launch Files

### Main Test Files (fixtures/launch/)

- **test_all_features.launch.xml** - Comprehensive test covering multiple features:
  - CLI arguments with defaults
  - Let variables
  - Groups with namespaces
  - Nodes with if/unless conditions
  - Includes with conditions and arguments
  - Substitutions (find-pkg-share, var)

- **test_args.launch.xml** - Tests argument handling:
  - Multiple arguments with defaults
  - Argument usage in node configurations

- **test_conditions.launch.xml** - Tests conditional execution:
  - If conditions on nodes
  - Unless conditions on nodes
  - Condition evaluation with variables

- **test_find_pkg.launch.xml** - Tests find-pkg-share substitution:
  - Package directory resolution
  - Path construction with substitutions

- **test_include.launch.xml** - Tests include functionality:
  - Basic includes without arguments
  - Includes with argument overrides
  - Multiple includes of the same file

- **test_include_pkg.launch.xml** - Tests package-based includes:
  - Including launch files from ROS 2 packages
  - Cross-package dependencies

### Included Files (fixtures/includes/)

- **included.launch.xml** - Simple launch file used by include tests:
  - Accepts node_name argument
  - Launches a single node

## Usage

These fixtures are used by integration tests to verify the parser handles various ROS 2 launch file features correctly.

To use these fixtures in tests:
```rust
use std::path::PathBuf;

let fixture_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
    .join("../../tests/fixtures/launch/test_all_features.launch.xml");
```

## Adding New Fixtures

When adding new test fixtures:
1. Place main launch files in `fixtures/launch/`
2. Place included launch files in `fixtures/includes/`
3. Use descriptive names like `test_<feature>.launch.xml`
4. Document the fixture purpose in this README
5. Update any include paths to use relative paths (e.g., `../includes/included.launch.xml`)

## Comparison Tests

The `comparison_tests/` directory contains tools for comparing Rust and Python parser outputs.

### Quick Start

```bash
# Run all comparison tests
cd tests/comparison_tests
./run_tests.sh

# Run individual test
python3 compare_rust_python.py test_simple.launch.xml
```

### Test Files

- **test_simple.launch.xml** - Basic namespace scoping test
- **test_group_namespace.launch.xml** - Nested namespace inheritance test
- **compare_rust_python.py** - Comprehensive comparison script
- **run_tests.sh** - Run all comparison tests

See `comparison_tests/README.md` for detailed documentation.

## Autoware Tests

The `autoware_test/` directory contains Autoware-specific test scripts and output.

### Setup

Create an activation script to point to your Autoware workspace:

```bash
cd tests/autoware_test/scripts
cp activate_autoware.sh.template activate_autoware.sh
# Edit activate_autoware.sh to point to your Autoware workspace
```

Example `activate_autoware.sh` content:
```bash
#!/usr/bin/env bash
source /path/to/your/autoware/workspace/install/setup.bash
```

**Note:** The `activate_autoware.sh` file is git-ignored since it contains user-specific paths.

### Scripts

- **scripts/compare_rust_python.py** - Comparison tool that validates Rust parser against Python dump_launch
- **scripts/activate_autoware.sh.template** - Template for workspace activation script
- **scripts/benchmark.sh** - Performance benchmarking

### Usage

```bash
# Run default test (planning_simulator.launch.xml)
just test-autoware

# Or run comparison directly
cd tests/autoware_test/scripts
python3 compare_rust_python.py

# Test specific launch file
python3 compare_rust_python.py /path/to/launch/file.launch.xml
```

See `autoware_test/README.md` for more details.
