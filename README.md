# play_launch_parser

High-performance ROS 2 launch file parser written in Rust.

## Overview

`play_launch_parser` reimplements ROS 2 launch file parsing in Rust to address the performance bottleneck in the Python-based `dump_launch` tool. It generates `record.json` files compatible with the [play_launch](https://github.com/tier4/play_launch) runtime.

### Performance Goals

| Metric | dump_launch (Python) | play_launch_parser (Target) |
|--------|---------------------|---------------------------|
| Autoware Launch Tree | ~40 seconds | <5 seconds |
| Language | Python | Rust |
| Formats Supported | Python, XML, YAML | XML, YAML (Python future) |

### Key Features

- **Fast**: Native Rust performance with minimal overhead
- **Compatible**: Generates `record.json` compatible with play_launch runtime
- **Correct**: Matches ROS 2 launch semantics for XML and YAML formats
- **Extensible**: Architecture designed to support Python launch files in the future

## Project Status

🚧 **Phase 1 - Project Setup** (In Progress)

See [roadmap](docs/roadmap/) for detailed development phases.

## Installation

### Prerequisites

- ROS 2 Humble or Jazzy
- Rust toolchain (latest stable)
- colcon and colcon-cargo-ros2

### Building from Source

```bash
# Install dependencies
just install-deps

# Build the project
just build

# Run tests
just test
```

## Usage

### Parsing Launch Files

```bash
# Source the workspace
source install/setup.bash

# Parse an XML launch file
play_launch_parser launch <package_name> <launch_file.xml>

# Parse a YAML launch file
play_launch_parser launch <package_name> <launch_file.yaml>

# Output: record.json
```

### Integration with play_launch

```bash
# 1. Generate record.json with play_launch_parser
play_launch_parser launch autoware_launch planning_simulator.launch.xml

# 2. Replay with play_launch runtime
play_launch replay
```

## Architecture

The parser is organized into modular components:

```
┌─────────────────┐
│  Launch Files   │
│ (XML, YAML)     │
└────────┬────────┘
         │
         v
┌─────────────────┐
│  Format Parser  │
│  (XML/YAML)     │
└────────┬────────┘
         │
         v
┌─────────────────┐
│  Tree Builder   │
│ (Substitutions, │
│    Includes)    │
└────────┬────────┘
         │
         v
┌─────────────────┐
│ Record Generator│
│ (record.json)   │
└─────────────────┘
```

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for detailed design.

## Development

### Quick Start

```bash
# Format code
just format

# Run linters
just check

# Run all tests
just test

# Clean build artifacts
just clean
```

### Feature Tracking

See [docs/FEATURE_LIST.md](docs/FEATURE_LIST.md) for comprehensive feature tracking:
- **229 features** identified across 15 categories
- Current status: Phase 1 complete (project setup)
- Next milestone: MVP (16 critical features for basic XML parsing)

### Testing

```bash
# Download demo packages
just download-demos

# Build with demos
just build

# Compare with dump_launch output
just compare-output demo_nodes_cpp talker_listener.launch.xml
```

### Project Structure

```
play_launch_parser/
├── src/                      # Source packages
│   └── play_launch_parser/  # Main parser package
├── docs/                     # Documentation
│   ├── ARCHITECTURE.md      # Architecture design
│   └── roadmap/             # Development phases
├── build/                   # Build artifacts (gitignored)
├── install/                 # Install directory (gitignored)
├── log/                     # Build logs (gitignored)
├── justfile                 # Build recipes
├── CLAUDE.md                # Guide for Claude Code
└── README.md                # This file
```

## Comparison with dump_launch

| Feature | dump_launch | play_launch_parser |
|---------|-------------|-------------------|
| Language | Python | Rust |
| Launch Execution | Dynamic (Python interpreter) | Static analysis |
| Python Launch Files | ✅ Supported | ❌ Not yet (future) |
| XML Launch Files | ✅ Supported | 🚧 In development |
| YAML Launch Files | ✅ Supported | 🚧 In development |
| Parse Time (Autoware) | ~40s | Target: <5s |
| Output Format | record.json | record.json (compatible) |

## Roadmap

- [x] **Phase 1**: Project Setup & Architecture Design ([roadmap](docs/roadmap/phase-1-PROJECT_SETUP.md))
- [ ] **Phase 2**: MVP XML Parser ([roadmap](docs/roadmap/phase-2-MVP_XML_PARSER.md)) ← Next
- [ ] **Phase 3**: Include Resolution & Parameter Files
- [ ] **Phase 4**: YAML Parser Implementation
- [ ] **Phase 5**: Advanced Substitutions & Package Finding
- [ ] **Phase 6**: Composable Nodes Support
- [ ] **Phase 7**: Performance Optimization
- [ ] **Future**: Python Launch File Support

See [docs/roadmap/](docs/roadmap/) for detailed phase plans.

### Current Status: Phase 1 Complete ✅

**Phase 1 Deliverables:**
- ✅ Project structure and build system (justfile)
- ✅ Comprehensive documentation (3,575 lines across 5 docs)
- ✅ Research complete (ROS 2 launch, dump_launch, record.json)
- ✅ Feature tracking system (229 features identified)
- ✅ External repositories downloaded and analyzed

**Next Milestone: MVP (Phase 2)**
- Target: Parse `talker_listener.launch.xml` and generate valid `record.json`
- Features: 16 critical features for basic XML parsing
- Timeline: 2-3 weeks

## Contributing

This project follows the same conventions as [play_launch](https://github.com/tier4/play_launch):

- Use `just build` for building (not `colcon build` directly)
- Run `just check` before committing
- Keep code modular and well-documented
- Write tests for new features

## License

[License TBD - should match play_launch]

## Acknowledgments

- Inspired by [play_launch](https://github.com/tier4/play_launch)
- Built to complement the play_launch runtime
- Addresses the dump_launch performance bottleneck

## Support

For issues and questions:
- GitHub Issues: [Repository issues page]
- Related to play_launch: See [play_launch repository]

---

**Note**: This project is in early development (Phase 1). The parser is not yet functional. See the roadmap for current status and upcoming features.
