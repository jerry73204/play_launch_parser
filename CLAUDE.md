# CLAUDE.md

Guide for Claude Code when working with this repository.

## Project Overview

ROS 2 Launch Parser in Rust - A high-performance reimplementation of ROS 2 launch file parsing:
- **play_launch_parser** (Rust): Parse XML and YAML launch files, generate record.json
- **Goal**: Replace the Python-based dump_launch bottleneck (40s → target <5s for Autoware)
- **Strategy**: Replicate official ROS 2 launch semantics in Rust, starting with XML/YAML (Python launch files deferred)

## Motivation

The existing play_launch project uses dump_launch (Python) to scan ROS 2 launch trees. This is a performance bottleneck:
- **Current**: ~40s to walk through Autoware launch tree
- **Root Cause**: Python overhead, dynamic launch execution, complex tree traversal
- **Solution**: Native Rust parser with static analysis for XML/YAML formats

## Project Status

**Phase 1 - Project Setup** (In Progress)
- Setting up directory structure
- Creating initial documentation
- Planning architecture

## Installation & Usage

```sh
# Build from source
just build

# Run tests
just test

# Check code quality
just check
```

## Architecture

### Planned Components

1. **XML Parser** (`src/xml_parser/`)
   - Parse ROS 2 XML launch files
   - Handle substitutions, includes, groups
   - Generate intermediate representation

2. **YAML Parser** (`src/yaml_parser/`)
   - Parse ROS 2 YAML launch files
   - Support YAML-specific features
   - Generate intermediate representation

3. **Launch Tree Builder** (`src/tree_builder/`)
   - Build launch tree from parsed files
   - Resolve substitutions and includes
   - Handle parameter files and argument passing

4. **Record Generator** (`src/record_generator/`)
   - Generate record.json compatible with play_launch
   - Collect node, container, and composable node metadata
   - Preserve parameter types and values

### Design Principles

- **Fast**: Minimal overhead, parallel processing where possible
- **Compatible**: Generate record.json compatible with play_launch runtime
- **Correct**: Match official ROS 2 launch semantics for XML/YAML
- **Extensible**: Architecture ready for Python launch support (future)

## Development Practices

### Building
- **ALWAYS** use `just build` for consistent builds
- Run `just check` before committing to verify code quality
- Use `just test` to run all tests

### Testing Strategy
- Download test packages to `src/` directory
- Use `colcon build` to build test packages alongside parser
- Compare parser output with dump_launch output
- Side-by-side log comparison for verification

### Code Quality
- Run `just check` to run linters and formatters
- Keep code modular and well-documented
- Write unit tests for parsers and tree builders
- Integration tests with real ROS 2 launch files

## Directory Structure

```
play_launch_parser/
├── src/                    # Source packages (ROS 2 + parser code)
├── docs/                   # Documentation
│   └── roadmap/           # Phase-based roadmap files
├── build/                 # Build artifacts (gitignored)
├── install/               # Install artifacts (gitignored)
├── log/                   # Build logs (gitignored)
├── justfile               # Build/test/check commands
├── CLAUDE.md              # This file
└── README.md              # User-facing documentation
```

## Comparison with play_launch

| Aspect | play_launch | play_launch_parser |
|--------|-------------|-------------------|
| Language | Python (dump) + Rust (runtime) | Rust (parser) |
| Launch Formats | Python, XML, YAML | XML, YAML (Python future) |
| Parse Time | ~40s (Autoware) | Target: <5s |
| Output | record.json | record.json (compatible) |
| Use Case | Full dump + replay | Fast parsing only |

## Key Challenges

1. **ROS 2 Launch Semantics**: XML/YAML formats have complex substitution, parameter passing, and include mechanics
2. **Compatibility**: Must generate record.json compatible with play_launch runtime
3. **Performance**: Parser must be significantly faster than Python approach
4. **Testing**: Need comprehensive test suite with real ROS 2 packages

## Roadmap

See `docs/roadmap/` for detailed phase plans:
- Phase 1: Project Setup & Architecture Design
- Phase 2: XML Parser Implementation (TBD)
- Phase 3: YAML Parser Implementation (TBD)
- Phase 4: Launch Tree Builder (TBD)
- Phase 5: Record Generator & Integration (TBD)

## Development Workflow

1. **Exploration**: Download ROS 2 packages to `src/`, build with colcon
2. **Implementation**: Write parser code, iterate with tests
3. **Verification**: Compare parser output with dump_launch
4. **Optimization**: Profile and optimize hot paths

## Testing

Test packages should be placed in `src/` and built with colcon:
```sh
# Example workflow
cd src/
git clone https://github.com/ros2/demos.git
cd ..
colcon build --base-paths src/demos
```

Compare parser output:
```sh
# Generate with dump_launch (Python)
dump_launch launch demo_nodes_cpp talker_listener.launch.py
mv record.json record_python.json

# Generate with play_launch_parser (Rust)
play_launch_parser launch demo_nodes_cpp talker_listener.launch.xml
mv record.json record_rust.json

# Compare
diff -u record_python.json record_rust.json
```

## Notes

- Focus on XML/YAML first due to Python parsing complexity
- Target compatibility with play_launch v0.x.x runtime
- Document semantic differences between parser and dump_launch
- Keep performance metrics for each phase
