# Claude Development Guidelines

This document contains best practices and guidelines for Claude (AI assistant) when working on this project.

## Quality Assurance

### Always Run Quality Checks Before Finishing

**IMPORTANT**: After completing any task that modifies code, always run:

```bash
just quality
```

This command runs:
1. `just check` - Linters and formatters (clippy, rustfmt)
2. `just test-rust` - All Rust unit tests

Only mark a task as complete after all quality checks pass.

## Development Workflow

### When Making Changes

1. **Read files first**: Always use the Read tool to examine files before modifying them
2. **Run tests after changes**: After any code modification, run `cargo test` or `just test-rust`
3. **Fix all errors**: Never leave compilation errors or failing tests
4. **Update tests**: When changing functionality, update relevant tests
5. **Add tests for new features**: New functionality requires comprehensive test coverage

### Test-Driven Development

- Write tests for new features before or during implementation
- Ensure tests cover:
  - Happy path (normal operation)
  - Edge cases (boundary conditions)
  - Error cases (invalid input, failures)
  - Integration scenarios (features working together)

### Code Quality Standards

- **No warnings**: Fix all clippy warnings (`cargo clippy`)
- **Formatted code**: Always format with `cargo fmt`
- **No unused code**: Remove unused imports, variables, functions
- **Clear naming**: Use descriptive names for variables, functions, types
- **Documentation**: Add doc comments for public APIs

## Project-Specific Practices

### Temporary Files

When creating temporary files for debugging or testing:
- Use `$project/tmp/` directory instead of `/tmp/`
- Create the directory if it doesn't exist: `mkdir -p tmp/`
- Clean up temporary files when done if appropriate
- Example: `tmp/debug_output.json`, `tmp/test_results.txt`

### File Creation

When creating files:
- **Prefer**: Use `Write` tool for creating files
- **Avoid**: Using `cat` with heredoc pattern (`cat > file << 'EOF'`)
- **Reason**: Write tool is clearer and less error-prone
- **Exception**: Small one-liners or when appending to existing files

### Substitution System

When working on the substitution system:
- All substitution types support nested substitutions via `Vec<Substitution>`
- Parser uses character-by-character parsing with parenthesis depth counting
- Resolution happens recursively from inside-out
- Always test both parsing and resolution separately

### Testing Strategy

Current test count baseline: **194 tests**

When adding features:
- Add parser tests (verify AST structure)
- Add resolution tests (verify runtime behavior)
- Add integration tests (verify end-to-end functionality)
- Test nested/complex scenarios
- Test error cases

#### Test Fixtures Organization

Test launch files are organized in the `tests/` directory:

```
tests/
├── fixtures/
│   ├── launch/         # Main test launch files
│   └── includes/       # Launch files to be included by other files
└── README.md
```

When adding new test fixtures:
- Place main test files in `tests/fixtures/launch/`
- Place included files in `tests/fixtures/includes/`
- Use relative paths for includes: `../includes/file.launch.xml`
- Name test files descriptively: `test_<feature>.launch.xml`
- Document new fixtures in `tests/README.md`

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

## Notes

- This is a ROS 2 launch file parser written in Rust
- Goal: Replace slow Python `dump_launch` with fast Rust implementation
- Focus: Parser correctness, performance, and ROS 2 feature parity
