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
5. Verify all tests pass (249 tests expected)

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

### Temporary Files

- Use `$project/tmp/` directory instead of `/tmp/`
- Create if needed: `mkdir -p tmp/`
- Clean up when done if appropriate
- Example: `tmp/debug_output.json`, `tmp/test_results.txt`

### File Creation

- **Always use**: `Write` tool for creating files
- **Never use**: `cat` with heredoc pattern (`cat > file << 'EOF'`)
- **No exceptions**: Even for long files or complex content

### Test Fixtures

Test launch files are organized in the `tests/` directory:

```
tests/
├── fixtures/
│   ├── launch/         # Main test launch files
│   └── includes/       # Launch files to be included
└── README.md
```

See `tests/README.md` for test organization details.

## Common Commands

```bash
# Format code
just format

# Run linters
just check

# Run Rust unit tests
just test-rust

# Run all quality checks
just quality

# Build the project
just build

# Clean artifacts
just clean
```

## Before Completing a Task

Checklist:
- [ ] All code compiles without errors
- [ ] All tests pass (`just test-rust`)
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

**Current Status**: Phase 5.4 - Autoware Compatibility Testing
- **Test Coverage**: 249 tests passing (~95% code coverage)
- **Feature Completion**: 93% (Phase 5 in progress)
- **Autoware Compatibility**: 95% XML files, 80-85% Python files

### Current Capabilities

- ✅ Complete XML launch file parsing
- ✅ All core substitutions (`$(var)`, `$(env)`, `$(find-pkg-share)`, etc.)
- ✅ Eval expressions (arithmetic & string comparisons)
- ✅ Python launch file support (core API)
- ✅ Container and composable node support
- ✅ YAML launch file support
- 🔄 Python API enhancements (in progress)

### Detailed Documentation

For detailed information, see:
- **Feature tracking**: `docs/feature_list.md`
- **Implementation status**: `docs/roadmap/implementation_status.md`
- **Phase 5 roadmap**: `docs/roadmap/phase-5-python_support.md`
- **Test organization**: `tests/README.md`

### Quick Reference

**Test Count Baseline**: 249 tests (208 unit + 18 edge + 23 integration)

**Key Architectural Notes**:
- Substitution system uses recursive `Vec<Substitution>` for nesting
- Parser uses character-by-character parsing with depth counting
- Include arguments use `Vec` (not `HashMap`) to preserve order
- Python API uses capture-on-construction pattern

See implementation docs for detailed architecture and design decisions.
