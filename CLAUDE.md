# Claude Development Guidelines

This document contains essential practices and guidelines for Claude (AI assistant) when working on this project.

## Quality Assurance

### Always Run Quality Checks Before Finishing

**CRITICAL PRACTICE**: Before marking any task as complete, ALWAYS run quality checks and fix errors.

**Quick Command**:
```bash
just quality
```

This runs:
1. `just check` - Linters and formatters (clippy, rustfmt)
2. `just test-rust` - All Rust unit tests

**Required Steps**:
1. Run clippy: `cargo clippy --all-targets --all-features -- -D warnings`
2. Fix ALL warnings and errors (no exceptions)
3. Check formatting: `cargo fmt -- --check`
4. Run all tests: `cargo test --all`
5. Verify all tests pass (260 tests expected)

**Never**:
- Mark a task complete with failing tests
- Leave clippy warnings unfixed
- Skip quality checks "for later"
- Commit code that doesn't compile

## Development Workflow

### When Making Changes

1. **Read files first**: Always use the Read tool before modifying
2. **Run tests after changes**: After modifications, run `cargo test` or `just test-rust`
3. **Fix all errors**: Never leave compilation errors or failing tests
4. **Update tests**: When changing functionality, update relevant tests
5. **Add tests for new features**: New functionality requires comprehensive test coverage

### Test-Driven Development

Write tests covering:
- Happy path (normal operation)
- Edge cases (boundary conditions)
- Error cases (invalid input, failures)
- Integration scenarios (features working together)

### Code Quality Standards

- **No warnings**: Fix all clippy warnings
- **Formatted code**: Always format with `cargo fmt`
- **No unused code**: Remove unused imports, variables, functions
- **Clear naming**: Use descriptive names
- **Documentation**: Add doc comments for public APIs

## Project-Specific Practices

### Temporary Files and Scripts

**IMPORTANT**: Always create temporary files and scripts in `$project/tmp/` directory instead of `/tmp/`

- **Location**: `$project/tmp/` (i.e., `/home/aeon/repos/play_launch_parser/tmp/`)
- **Create if needed**: `mkdir -p tmp/`
- **Clean up**: Remove when done if appropriate
- **Applies to**: Debug files, analysis scripts, test outputs, comparison tools, etc.

Examples:
- Temporary data: `tmp/debug_output.json`, `tmp/test_results.txt`
- Analysis scripts: `tmp/compare_outputs.py`, `tmp/analyze_nodes.sh`
- Test data: `tmp/sample_launch.xml`, `tmp/captured_data.md`

### File Creation

**CRITICAL RULE**: Always use the `Write` tool for creating files. Never use Bash commands.

Rules:
- ✅ **Correct**: Use `Write` tool for ALL file creation
- ❌ **Wrong**: `cat > file << 'EOF'`, `echo > file`, Bash redirection
- ✅ **Correct**: Use `Edit` tool for modifying existing files
- ❌ **Wrong**: `sed`, `awk`, or any Bash text processing

**Why this matters**:
- Write tool has better error handling
- Write tool requires reading file first (prevents accidental overwrites)
- Write tool is explicit and clear in the tool call log
- Bash heredoc can have quoting/escaping issues

**No exceptions**: Use Write even for:
- Long files (100+ lines)
- Files with complex content
- Multiple files in sequence
- Temporary test scripts

### Test Organization

**Test Structure** (Session 12 - Reorganized):

Test files are organized by category in `src/play_launch_parser/tests/`:

```
src/play_launch_parser/tests/
├── edge_cases.rs           # Edge case tests (18 tests)
├── xml_tests.rs            # XML parsing tests (20 tests)
├── python_tests.rs         # Python launch tests (15 tests)
├── integration_tests.rs    # Performance tests (3 tests)
└── fixtures/
    ├── launch/             # Main test launch files
    └── includes/           # Files to be included by tests
```

**Test Fixtures Location**: `src/play_launch_parser/tests/fixtures/`
- Helper functions use `env!("CARGO_MANIFEST_DIR")/tests/fixtures/launch`
- Fixtures are self-contained within the crate

**Total Test Count**: 260 tests (218 unit + 18 edge + 20 XML + 15 Python + 3 performance + 6 integration base)

### Python Test Requirements

**CRITICAL**: Python tests MUST use serialization to prevent race conditions.

**Problem**: Python's global interpreter state causes test contamination when tests run in parallel.

**Solution**: All Python tests must acquire `python_test_guard()` at the start:

```rust
#[test]
#[cfg(feature = "python")]
fn test_my_python_feature() {
    let _guard = python_test_guard();  // ← REQUIRED for all Python tests
    // ... test code ...
}
```

**Why This Matters**:
- Python's global capture storage (CAPTURED_NODES, etc.) is shared across all tests
- Parallel test execution causes data races and false failures
- The guard ensures Python tests run sequentially
- Without the guard, tests will fail intermittently

**When Adding New Python Tests**:
1. Add `#[cfg(feature = "python")]` attribute
2. Add `let _guard = python_test_guard();` as first line in test function
3. Place test in `python_tests.rs` file

See `src/play_launch_parser/tests/python_tests.rs` for examples.

## Common Commands

```bash
# Run ALL tests (Rust + comparison + Autoware if available)
just test

# Run only Rust unit tests (260 tests)
just test-rust

# Run comparison tests (Rust vs Python parser)
just test-compare

# Run Autoware validation tests
just test-autoware

# Run colcon tests (ROS 2 integration tests)
just test-colcon

# Run all quality checks (linters + Rust tests)
just quality

# Run linters and formatters
just check

# Format code
just format

# Build the project
just build

# Clean artifacts
just clean
```

**Test Organization**:
- `just test` - Runs ALL tests in the project (recommended for CI/validation)
- `just test-rust` - Fast Rust unit tests only (recommended during development)
- `just test-compare` - Comparison tests between Rust and Python parsers
- `just test-autoware` - Full Autoware validation (requires Autoware symlink)
- `just quality` - Linters + Rust tests (recommended before commits)

## Before Completing a Task

Checklist:
- [ ] All code compiles without errors
- [ ] All tests pass (`just test` or `just quality`)
- [ ] Code is formatted (`just format`)
- [ ] No clippy warnings (`just check`)
- [ ] Quality checks pass (`just quality`)
- [ ] New functionality has tests
- [ ] Documentation is updated if needed

## Environment Setup

### direnv Configuration

The project uses `.envrc` to automatically source ROS 2 environment:

```bash
# .envrc sources:
# - /opt/ros/humble/setup.bash (if exists)
# - install/setup.bash (if exists, watched for changes)
```

After creating/modifying `.envrc`, run `direnv allow` to enable it.

## Project Overview

**Goal**: Fast Rust implementation of ROS 2 launch file parser to replace slow Python `dump_launch`

**Current Status**: ✅ **PRODUCTION READY** - Autoware Compatibility Complete
- **Test Coverage**: 260 tests passing (~95% code coverage)
- **Feature Completion**: 95% (Phase 5 complete)
- **Autoware Compatibility**: ✅ **100%** - Full planning_simulator test passes (46 nodes, 15 containers, 54 composable nodes)
- **Performance**: <5s to parse full Autoware launch tree (vs ~10-15s with Python)

### Current Capabilities

- ✅ Complete XML launch file parsing
- ✅ All core substitutions (`$(var)`, `$(env)`, `$(find-pkg-share)`, etc.)
- ✅ Eval expressions (arithmetic & string comparisons)
- ✅ Python launch file support (complete core + extended API)
- ✅ Container and composable node support (XML + Python)
- ✅ YAML launch file support
- ✅ Boolean attribute substitutions (respawn, respawn_delay)
- ✅ Python API: launch, launch_ros, launch_xml, launch.frontend, launch.utilities
- ✅ Autoware production workload validated

### Detailed Documentation

For detailed information, see:
- **Feature tracking**: `docs/feature_list.md`
- **Implementation status**: `docs/roadmap/implementation_status.md`
- **Phase 5 roadmap**: `docs/roadmap/phase-5-python_support.md`
- **Test organization**: `tests/README.md`

### Quick Reference

**Test Count Baseline**: 260 tests (218 unit + 18 edge + 20 XML + 15 Python + 3 performance + 6 integration base)

**Key Architectural Notes**:
- Substitution system uses recursive `Vec<Substitution>` for nesting
- Parser uses character-by-character parsing with depth counting
- Include arguments use `Vec` (not `HashMap`) to preserve order
- Python API uses capture-on-construction pattern

See implementation docs for detailed architecture and design decisions.

## Recent Session Work (Latest)

### Session 12: Test Organization & Documentation ✅

**Work Completed**:
1. ✅ Cleaned up debug code - Converted all `eprintln!` to structured logging (`log::trace!`, `log::debug!`, `log::error!`)
2. ✅ Fixed test script bugs - `compare_rust_python.py` now correctly validates all metrics including node count adjustments for containers
3. ✅ Created comprehensive edge case tests - 4 new fixtures testing Autoware-derived patterns (OpaqueFunction conditional logic, list concatenation, ParameterFile, nested substitutions)
4. ✅ Reorganized test structure:
   - Moved fixtures from `/tests/fixtures` to `/src/play_launch_parser/tests/fixtures`
   - Split 2191-line `integration_tests.rs` into 3 focused files:
     - `xml_tests.rs` (20 tests, 36KB)
     - `python_tests.rs` (15 tests, 35KB)
     - `integration_tests.rs` (3 performance tests, 3.5KB)
5. ✅ Fixed Python test race conditions - Added `python_test_guard()` to serialize Python tests and prevent global state contamination
6. ✅ Updated documentation:
   - `docs/feature_list.md` - Added edge case testing section, test coverage matrix, and Autoware validation results
   - Test counts updated to 260 total tests

**Key Technical Fixes**:
- **Logging**: All debug output now uses proper log levels for RUST_LOG control
- **Test Serialization**: Python tests run sequentially to avoid race conditions in global capture storage
- **Test Organization**: Logical separation of XML, Python, and performance tests

**Documentation Created**:
- `/tmp/test_coverage_update.md` - Comprehensive test coverage documentation
- Edge case test fixtures validating Autoware patterns

**Validation**:
- ✅ 260 tests passing (218 unit + 18 edge + 20 XML + 15 Python + 3 performance + 6 base)
- ✅ 100% Autoware compatibility maintained (46 nodes, 15 containers, 54 composable nodes)
- ✅ Zero clippy warnings
- ✅ All quality checks passing
