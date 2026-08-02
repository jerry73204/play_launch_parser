> # ⚠️ ARCHIVED — moved into `play_launch`
>
> This repository is **read-only** and no longer developed. Its full history
> and every file were folded into the `play_launch` repository on 2026-08-03
> (`git subtree`, so `git log` there covers this repo's commits too):
>
> **→ https://github.com/NEWSLabNTU/play_launch — directory `src/ros-launch-resolve/parser/`**
>
> The final commit here, `f1cad5b`, is the exact tree that was absorbed;
> nothing was left behind.
>
> It was a submodule of `ros-launch-resolve`, which was itself a submodule of
> `play_launch` — three levels deep. Both were folded in by play_launch
> phase-55; nano-ros RFC-0060 was amended to match (`9baebb2eb`). Reasoning:
> `docs/design/launch-toolchain-topology.md` in play_launch.
>
> **The text below is out of date and kept only as a historical record.** In
> particular: the parser no longer produces `record.json` — that artifact was
> retired in phase 47 in favour of the SystemModel (`system_model.yaml`) — and
> the `play_launch` link points at a fork that is not upstream. Current
> documentation lives in play_launch.

# play_launch_parser

Fast ROS 2 launch file parser in Rust. Generates `record.json` for [play_launch](https://github.com/tier4/play_launch).

## Features

- XML, Python, and YAML launch files
- 5-10x faster than dump_launch
- Compatible output format
- 260 tests, 100% Autoware compatible

## Install

```bash
git clone https://github.com/tier4/play_launch_parser.git
cd play_launch_parser
just install-deps
just build
source install/setup.bash
```

Requires ROS 2 (Humble/Jazzy), Rust 1.70+, and [just](https://github.com/casey/just).

## Usage

```bash
# Basic
play_launch_parser launch <package> <file.xml>

# With arguments
play_launch_parser launch <package> <file.xml> -- use_sim:=true

# Autoware example
play_launch_parser launch autoware_launch planning_simulator.launch.xml
```

Output: `record.json` in current directory.

## Supported Elements

**Actions**: `<node>`, `<include>`, `<group>`, `<let>`, `<arg>`, `<set_parameter>`
**Containers**: `<node_container>`, `<composable_node>`, `<load_composable_node>`
**Substitutions**: `$(var)`, `$(env)`, `$(find-pkg-share)`, `$(eval)`, nested lists
**Python**: OpaqueFunction, DeclareLaunchArgument, containers, all standard substitutions

## Performance

| Workload | dump_launch | play_launch_parser |
|----------|-------------|-------------------|
| Simple | ~100ms | <10ms |
| Autoware | ~40s | <5s |

## Development

```bash
just test          # All tests
just test-rust     # Rust tests only
just quality       # Linters + tests
just format        # Format code
```

## Documentation

- [Feature List](docs/feature_list.md) - Complete feature documentation
- [CLAUDE.md](CLAUDE.md) - Development guidelines

## License

[License TBD - should match play_launch]
