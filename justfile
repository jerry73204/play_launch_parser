# Detect ROS2 distro based on Ubuntu version
ros_distro := if `lsb_release -rs 2>/dev/null || echo "22.04"` == "24.04" { "jazzy" } else { "humble" }
colcon_flags := "--symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --cargo-args --release"

# Show available recipes
default:
    @just --list

# Install all dependencies
install-deps:
    #!/usr/bin/env bash
    set -e

    # Install colcon-cargo-ros2 for Rust support
    pip install colcon-cargo-ros2

    source /opt/ros/{{ros_distro}}/setup.bash
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y

# Build all packages with colcon
build:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash

    echo "Building packages with colcon..."
    colcon build {{colcon_flags}} --base-paths src

# Run tests
test:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash
    source install/setup.bash

    colcon test --base-paths src
    colcon test-result --all --verbose

# Run Rust unit tests
test-rust:
    #!/usr/bin/env bash
    set -e

    echo "Running Rust unit tests..."
    find src -name "Cargo.toml" -not -path "*/target/*" | while read cargo_toml; do
        dir=$(dirname "$cargo_toml")
        echo "Testing $dir..."
        (cd "$dir" && cargo test)
    done

    echo "All Rust tests passed!"

# Run comparison tests (Rust vs Python parser)
test-compare:
    #!/usr/bin/env bash
    set -e
    cd tests/comparison_tests
    ./run_tests.sh

# Run linters and formatters
check:
    #!/usr/bin/env bash
    set -e

    echo "Running Rust checks..."
    find src -name "Cargo.toml" -not -path "*/target/*" | while read cargo_toml; do
        dir=$(dirname "$cargo_toml")
        echo "Checking $dir..."
        (cd "$dir" && cargo clippy --all-targets --all-features -- -D warnings)
        (cd "$dir" && cargo fmt -- --check)
    done

    echo "All checks passed!"

# Run quality checks (linters + tests)
quality: check test-rust
    @echo "✓ All quality checks passed!"

# Format all code
format:
    #!/usr/bin/env bash
    set -e

    echo "Formatting Rust code..."
    find src -name "Cargo.toml" -not -path "*/target/*" | while read cargo_toml; do
        dir=$(dirname "$cargo_toml")
        (cd "$dir" && cargo fmt)
    done

    echo "Code formatted!"

# Clean all build artifacts
clean:
    rm -rf build install log

# Clean everything including Cargo artifacts
clean-all: clean
    find src -type d -name "target" -not -path "*/build/*" -exec rm -rf {} + 2>/dev/null || true

# List ROS 2 packages in workspace
list-packages:
    #!/usr/bin/env bash
    source /opt/ros/{{ros_distro}}/setup.bash
    colcon list --base-paths src

# Download demo packages for testing
download-demos:
    #!/usr/bin/env bash
    set -e
    cd src

    if [ ! -d "demos" ]; then
        echo "Downloading ROS 2 demos..."
        git clone https://github.com/ros2/demos.git -b {{ros_distro}}
    else
        echo "demos directory already exists, skipping..."
    fi

    cd ..
    echo "Run 'just build' to compile the demos"

# Compare parser output with dump_launch
compare-output PACKAGE LAUNCH_FILE:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash
    source install/setup.bash

    echo "Generating record with dump_launch (Python)..."
    dump_launch launch {{PACKAGE}} {{LAUNCH_FILE}}
    mv record.json record_python.json

    echo "Generating record with play_launch_parser (Rust)..."
    # TODO: Implement parser command
    # play_launch_parser launch {{PACKAGE}} {{LAUNCH_FILE}}
    # mv record.json record_rust.json

    echo "Comparing outputs..."
    # diff -u record_python.json record_rust.json
    echo "Parser not yet implemented"

# Test Rust parser with Autoware launch file (requires Autoware)
test-autoware:
    #!/usr/bin/env bash
    set -e
    cd tests/autoware_test/scripts

    if [ ! -L "../autoware" ]; then
        echo "ERROR: Autoware symlink not found"
        echo "Create symlink: cd tests/autoware_test && ln -s /path/to/autoware autoware"
        exit 1
    fi

    # Run comparison with default launch file (no args needed)
    ./compare_rust_python.py

# Compare Rust and Python parser outputs for Autoware (alias for test-autoware)
compare-autoware: test-autoware

# Benchmark Rust parser performance with Autoware
benchmark-autoware:
    #!/usr/bin/env bash
    cd tests/autoware_test
    ./scripts/benchmark.sh
