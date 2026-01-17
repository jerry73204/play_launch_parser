# Feature Implementation Tracking

Comprehensive feature list for the play_launch_parser project. Track implementation status, priorities, and test coverage.

**Last Updated:** 2026-01-18
**Current Phase:** Phase 1 (Project Setup) ✅ Complete
**Next Phase:** Phase 2 (XML Parser Foundation)

---

## Status Legend

- ❌ **Not Started** - Feature not yet implemented
- 🚧 **In Progress** - Currently being worked on
- ✅ **Complete** - Fully implemented and tested
- 🧪 **Needs Testing** - Implemented but needs test coverage
- 📝 **Needs Documentation** - Implemented but poorly documented

---

## 1. Core Infrastructure

### 1.1 Project Setup

| Feature                  | Status | Priority | Notes                      |
|--------------------------|--------|----------|----------------------------|
| Directory structure      | ✅     | Critical | src/, docs/, external/     |
| Build system (justfile)  | ✅     | Critical | build, test, check, format |
| Documentation (initial)  | ✅     | Critical | 2,387 lines complete       |
| .gitignore configuration | ✅     | Critical |                            |
| README.md                | ✅     | High     | User-facing documentation  |
| CLAUDE.md                | ✅     | High     | AI assistant guide         |

### 1.2 Rust Package Structure

| Feature                           | Status | Priority | Notes                         |
|-----------------------------------|--------|----------|-------------------------------|
| Create play_launch_parser package | ❌     | Critical | ROS 2 package in src/         |
| Cargo.toml configuration          | ❌     | Critical | Dependencies, metadata        |
| package.xml for ROS 2             | ❌     | Critical | ROS 2 package manifest        |
| Module structure                  | ❌     | Critical | xml_parser, yaml_parser, etc. |
| CLI entry point                   | ❌     | Critical | main.rs with clap             |
| Error types                       | ❌     | High     | Custom error enum             |
| Logging setup                     | ❌     | Medium   | env_logger or tracing         |

---

## 2. XML Launch Parser

### 2.1 XML Parsing Core

| Feature                | Status | Priority | Notes                      |
|------------------------|--------|----------|----------------------------|
| XML file loading       | ❌     | Critical | roxmltree or quick-xml     |
| Element tree traversal | ❌     | Critical | Recursive descent          |
| Attribute extraction   | ❌     | Critical | key-value pairs            |
| Child element access   | ❌     | Critical | Iteration                  |
| Type coercion          | ❌     | High     | String → bool/int/float    |
| `-sep` suffix lists    | ❌     | Medium   | args-sep=";", args="a;b;c" |
| Error reporting        | ❌     | High     | Line numbers, context      |

### 2.2 XML Entity Abstraction

| Feature                        | Status | Priority | Notes                               |
|--------------------------------|--------|----------|-------------------------------------|
| Entity trait                   | ❌     | Critical | type_name(), get_attr(), children() |
| XmlEntity implementation       | ❌     | Critical | Wraps roxmltree::Node               |
| Attribute validation           | ❌     | High     | Required vs optional                |
| Unexpected attribute detection | ❌     | Medium   | Error on unknown attrs              |
| Complete parsing assertion     | ❌     | Medium   | All attrs/children consumed         |

### 2.3 XML Actions

| Feature                            | Status | Priority | Notes                   |
|------------------------------------|--------|----------|-------------------------|
| `<node>` - Regular nodes           | ❌     | Critical | Most common action      |
| `<executable>` - Generic processes | ❌     | High     | For non-ROS executables |
| `<arg>` - Launch arguments         | ❌     | Critical | DeclareLaunchArgument   |
| `<include>` - File includes        | ❌     | Critical | Recursive launch files  |
| `<group>` - Action grouping        | ❌     | High     | Scoping and namespacing |
| `<let>` - Local variables          | ❌     | High     | Scoped substitutions    |
| `<set_launch_configuration>`       | ❌     | High     | Global configuration    |
| `<set_env>` - Environment vars     | ❌     | Medium   | Process environment     |
| `<composable_node_container>`      | ❌     | High     | Component containers    |
| `<composable_node>` (child)        | ❌     | High     | Composable node defs    |
| `<load_composable_nodes>`          | ❌     | High     | Dynamic loading         |
| `<lifecycle_node>`                 | ❌     | Medium   | Lifecycle-aware nodes   |
| `<push_ros_namespace>`             | ❌     | Medium   | Namespace stack         |

### 2.4 XML Node Sub-Elements

| Feature                            | Status | Priority | Notes                  |
|------------------------------------|--------|----------|------------------------|
| `<param>` - Inline parameters      | ❌     | Critical | name, value attributes |
| `<param from="...">` - Param files | ❌     | Critical | Load from YAML file    |
| `<remap>` - Topic remapping        | ❌     | Critical | from, to attributes    |
| `<env>` - Environment variables    | ❌     | High     | name, value attributes |

---

## 3. YAML Launch Parser

### 3.1 YAML Parsing Core

| Feature             | Status | Priority | Notes                       |
|---------------------|--------|----------|-----------------------------|
| YAML file loading   | ❌     | High     | serde_yaml                  |
| Root key extraction | ❌     | High     | Single key dict             |
| Dict/List handling  | ❌     | High     | Nested structures           |
| Type preservation   | ❌     | Critical | No coercion (type checking) |
| Error reporting     | ❌     | High     | YAML line numbers           |

### 3.2 YAML Entity Abstraction

| Feature                      | Status | Priority | Notes                   |
|------------------------------|--------|----------|-------------------------|
| YamlEntity implementation    | ❌     | High     | Wraps serde_json::Value |
| Attribute access from dict   | ❌     | High     | key → value             |
| Children from 'children' key | ❌     | High     | List of child dicts     |
| Type validation              | ❌     | High     | is_instance_of() checks |

### 3.3 YAML Actions

| Feature              | Status | Priority | Notes                       |
|----------------------|--------|----------|-----------------------------|
| All actions from XML | ❌     | High     | Same as XML but YAML syntax |
| YAML-specific syntax | ❌     | Medium   | Lists, dicts, multiline     |

---

## 4. Substitution Engine

### 4.1 Substitution Grammar Parser

| Feature                             | Status | Priority | Notes                      |
|-------------------------------------|--------|----------|----------------------------|
| Grammar parser (Lark port or regex) | ❌     | Critical | `$(name args...)`          |
| Quoted string handling              | ❌     | Critical | Single/double quotes       |
| Escape sequence support             | ❌     | High     | `\$`, `\"`, `\'`           |
| Recursive substitution              | ❌     | High     | `$(var $(var inner))`      |
| Mixed text and substitutions        | ❌     | Critical | `"prefix $(var x) suffix"` |

### 4.2 Core Substitutions

| Feature                                   | Status | Priority | Notes                 |
|-------------------------------------------|--------|----------|-----------------------|
| `$(var name)` - LaunchConfiguration       | ❌     | Critical | Most common           |
| `$(env VAR)` - Environment variable       | ❌     | Critical | With optional default |
| `$(env VAR default)` - Env with default   | ❌     | High     | Fallback value        |
| `$(find-pkg-share pkg)` - Package path    | ❌     | Critical | ROS 2 package finding |
| `$(find-pkg-prefix pkg)` - Install prefix | ❌     | Medium   | Package install dir   |
| Text substitution (literal)               | ❌     | Critical | Plain strings         |

### 4.3 Advanced Substitutions

| Feature                                   | Status | Priority | Notes               |
|-------------------------------------------|--------|----------|---------------------|
| `$(command cmd)` - Shell command          | ❌     | Medium   | Execute and capture |
| `$(file path)` - File content             | ❌     | Low      | Read file           |
| `$(find-exec name)` - Find executable     | ❌     | Low      | Search PATH         |
| `$(this-launch-file)` - Current file path | ❌     | Medium   | Self-reference      |
| `$(this-launch-file-dir)` - Current dir   | ❌     | Medium   | Directory           |
| `$(anon name)` - Anonymous name           | ❌     | Low      | name_12345          |
| `$(dirname path)` - Directory name        | ❌     | Low      | Path manipulation   |
| `$(filename path)` - File name            | ❌     | Low      | Path manipulation   |

### 4.4 Substitution Context

| Feature                     | Status | Priority | Notes                   |
|-----------------------------|--------|----------|-------------------------|
| LaunchContext structure     | ❌     | Critical | Holds configurations    |
| Configuration storage       | ❌     | Critical | Key-value map           |
| Environment variable access | ❌     | Critical | System env              |
| Scoped variables (locals)   | ❌     | Medium   | let-bound vars          |
| Package path resolution     | ❌     | Critical | ament_index integration |

---

## 5. Launch Tree Building

### 5.1 Tree Traversal

| Feature                     | Status | Priority | Notes                     |
|-----------------------------|--------|----------|---------------------------|
| Recursive entity visitation | ❌     | Critical | Depth-first traversal     |
| Action dispatching          | ❌     | Critical | Type-based routing        |
| Context propagation         | ❌     | Critical | Pass context through tree |
| Sub-entity collection       | ❌     | Critical | Gather children           |

### 5.2 Include Resolution

| Feature                       | Status | Priority | Notes                    |
|-------------------------------|--------|----------|--------------------------|
| Include file path resolution  | ❌     | Critical | Resolve substitutions    |
| Recursive include parsing     | ❌     | Critical | Nested launches          |
| Argument passing to includes  | ❌     | High     | `<arg>` in `<include>`   |
| Circular dependency detection | ❌     | Medium   | Prevent infinite loops   |
| Include path caching          | ❌     | Low      | Performance optimization |
| Parallel include parsing      | ❌     | Low      | Concurrency              |

### 5.3 Namespace Management

| Feature                 | Status | Priority | Notes                    |
|-------------------------|--------|----------|--------------------------|
| Namespace stack         | ❌     | High     | push_ros_namespace       |
| Namespace normalization | ❌     | High     | Leading/trailing slashes |
| Full name construction  | ❌     | Critical | namespace + node_name    |
| Scoped namespaces       | ❌     | Medium   | Group-based scoping      |

---

## 6. Node Metadata Extraction

### 6.1 Regular Nodes

| Feature                    | Status | Priority | Notes                       |
|----------------------------|--------|----------|-----------------------------|
| Package name extraction    | ❌     | Critical | `pkg` attribute             |
| Executable name extraction | ❌     | Critical | `exec` attribute            |
| Node name extraction       | ❌     | Critical | `name` attribute (optional) |
| Namespace extraction       | ❌     | Critical | `namespace` attribute       |
| Output mode                | ❌     | High     | screen/log/both             |
| Respawn configuration      | ❌     | High     | respawn, respawn_delay      |
| Launch prefix              | ❌     | Medium   | gdb, valgrind, etc.         |
| Working directory          | ❌     | Medium   | `cwd` attribute             |

### 6.2 Parameters

| Feature                          | Status | Priority | Notes                  |
|----------------------------------|--------|----------|------------------------|
| Inline parameters (`<param>`)    | ❌     | Critical | name, value pairs      |
| Parameter files (`<param from>`) | ❌     | Critical | Load YAML files        |
| Parameter file parsing           | ❌     | Critical | ROS 2 YAML format      |
| Parameter file content storage   | ❌     | Critical | Store in file_data map |
| Parameter type preservation      | ❌     | High     | bool/int/float/string  |
| Global parameters                | ❌     | High     | SetParameter actions   |
| Nested parameter namespaces      | ❌     | Medium   | YAML nested dicts      |

### 6.3 Remappings

| Feature                        | Status | Priority | Notes                    |
|--------------------------------|--------|----------|--------------------------|
| Topic remapping                | ❌     | Critical | `<remap>` elements       |
| Service remapping              | ❌     | High     | Same as topic            |
| Node name remapping (`__node`) | ❌     | Medium   | Special remapping        |
| Namespace remapping (`__ns`)   | ❌     | Medium   | Special remapping        |
| config_file=/home/aeon/repos/LCTK/install/lctk_launch/share/lctk_launch/config/examples/sample_data.yamlRemapping substitutions        | ❌     | High     | Resolve $(var) in remaps |

### 6.4 Command-Line Generation

| Feature                    | Status | Priority | Notes                       |
|----------------------------|--------|----------|-----------------------------|
| Executable path resolution | ❌     | Critical | Package + executable lookup |
| ROS args formatting        | ❌     | Critical | `--ros-args` delimiter      |
| Node name argument         | ❌     | Critical | `-r __node:=name`           |
| Namespace argument         | ❌     | Critical | `-r __ns:=/namespace`       |
| Parameter arguments        | ❌     | Critical | `-p name:=value`            |
| Remapping arguments        | ❌     | Critical | `-r from:=to`               |
| Parameter file arguments   | ❌     | Critical | `--params-file path`        |
| User arguments             | ❌     | High     | Custom args                 |
| Log level argument         | ❌     | Medium   | `--log-level`               |
| Complete cmd array         | ❌     | Critical | [exec, arg1, arg2, ...]     |

### 6.5 Environment Variables

| Feature                         | Status | Priority | Notes                |
|---------------------------------|--------|----------|----------------------|
| Environment variable extraction | ❌     | High     | `<env>` elements     |
| Environment inheritance         | ❌     | Medium   | Parent process env   |
| Environment substitutions       | ❌     | Medium   | $(env VAR) in values |

---

## 7. Composable Nodes

### 7.1 Containers

| Feature                       | Status | Priority | Notes                         |
|-------------------------------|--------|----------|-------------------------------|
| Container recognition         | ❌     | High     | ComposableNodeContainer       |
| Container metadata extraction | ❌     | High     | name, namespace               |
| Container as regular node     | ❌     | High     | Also in node[] array          |
| Container record generation   | ❌     | High     | ComposableNodeContainerRecord |

### 7.2 Composable Node Descriptions

| Feature                         | Status | Priority | Notes                   |
|---------------------------------|--------|----------|-------------------------|
| Plugin package extraction       | ❌     | High     | `pkg` attribute         |
| Plugin class name extraction    | ❌     | High     | `plugin` attribute      |
| Node name extraction            | ❌     | High     | `name` attribute        |
| Namespace extraction            | ❌     | High     | `namespace` attribute   |
| Parameters for composable nodes | ❌     | High     | `<param>` in composable |
| Remappings for composable nodes | ❌     | High     | `<remap>` in composable |
| Extra arguments                 | ❌     | Medium   | Plugin-specific args    |

### 7.3 LoadComposableNodes

| Feature                         | Status | Priority | Notes                       |
|---------------------------------|--------|----------|-----------------------------|
| Target container resolution     | ❌     | High     | target_container attribute  |
| Composable node list extraction | ❌     | High     | Child elements              |
| Log level mapping               | ❌     | Medium   | DEBUG/INFO/WARN/ERROR/FATAL |
| LoadNodeRecord generation       | ❌     | High     | Complete record structure   |

---

## 8. record.json Generation

### 8.1 Data Structures

| Feature                       | Status | Priority | Notes                  |
|-------------------------------|--------|----------|------------------------|
| NodeRecord struct             | ❌     | Critical | 15 fields              |
| ComposableNodeContainerRecord | ❌     | High     | 2 fields               |
| LoadNodeRecord struct         | ❌     | High     | 10 fields              |
| LifecycleNode list            | ❌     | Medium   | String array           |
| FileData map                  | ❌     | High     | path → content         |
| RecordJson root struct        | ❌     | Critical | All arrays + file_data |

### 8.2 Serialization

| Feature                    | Status | Priority | Notes               |
|----------------------------|--------|----------|---------------------|
| Serde JSON serialization   | ❌     | Critical | serde_json crate    |
| Field name mapping         | ❌     | Critical | snake_case matching |
| Null handling              | ❌     | Critical | Option<T> fields    |
| Tuple serialization        | ❌     | Critical | params, remaps, env |
| Pretty printing (optional) | ❌     | Low      | Readable JSON       |
| Compact output             | ❌     | Medium   | Default format      |

### 8.3 Validation

| Feature                    | Status | Priority | Notes                 |
|----------------------------|--------|----------|-----------------------|
| Required field validation  | ❌     | High     | executable, cmd, etc. |
| Type constraint validation | ❌     | Medium   | Correct types         |
| Semantic validation        | ❌     | Medium   | Container references  |
| Unique name checking       | ❌     | Low      | Warn on duplicates    |

---

## 9. Lifecycle Nodes

| Feature                        | Status | Priority | Notes                |
|--------------------------------|--------|----------|----------------------|
| Lifecycle node recognition     | ❌     | Medium   | LifecycleNode action |
| Lifecycle node list generation | ❌     | Medium   | String array         |
| Lifecycle node as regular node | ❌     | Medium   | Also in node[] array |

---

## 10. Conditions

| Feature                     | Status | Priority | Notes              |
|-----------------------------|--------|----------|--------------------|
| IfCondition support         | ❌     | Medium   | `if` attribute     |
| UnlessCondition support     | ❌     | Medium   | `unless` attribute |
| Condition evaluation        | ❌     | Medium   | Boolean expression |
| Conditional entity skipping | ❌     | Medium   | Skip if false      |

---

## 11. Error Handling

### 11.1 Parse Errors

| Feature                     | Status | Priority | Notes               |
|-----------------------------|--------|----------|---------------------|
| XML syntax errors           | ❌     | Critical | Line numbers        |
| YAML syntax errors          | ❌     | High     | Line numbers        |
| Unknown action types        | ❌     | High     | Clear error message |
| Missing required attributes | ❌     | Critical | Which attribute?    |
| Invalid attribute values    | ❌     | High     | Type mismatch       |
| Unexpected attributes       | ❌     | Medium   | Warn or error       |

### 11.2 Substitution Errors

| Feature                        | Status | Priority | Notes                        |
|--------------------------------|--------|----------|------------------------------|
| Invalid substitution syntax    | ❌     | High     | Grammar errors               |
| Undefined variable access      | ❌     | High     | $(var undefined)             |
| Undefined environment variable | ❌     | Medium   | $(env UNDEF) without default |
| Circular substitution          | ❌     | Medium   | $(var x) → $(var x)          |
| Substitution type errors       | ❌     | Medium   | Wrong argument types         |

### 11.3 Semantic Errors

| Feature                     | Status | Priority | Notes                            |
|-----------------------------|--------|----------|----------------------------------|
| Package not found           | ❌     | Critical | ROS 2 package missing            |
| Executable not found        | ❌     | High     | Binary missing                   |
| Include file not found      | ❌     | Critical | Launch file missing              |
| Parameter file not found    | ❌     | High     | YAML file missing                |
| Circular include detection  | ❌     | Medium   | A includes B includes A          |
| Invalid parameter YAML      | ❌     | High     | Malformed param file             |
| Container reference invalid | ❌     | Medium   | LoadNode → nonexistent container |

---

## 12. Testing

### 12.1 Unit Tests

| Feature                 | Status | Priority | Coverage Target |
|-------------------------|--------|----------|-----------------|
| Substitution parsing    | ❌     | Critical | 100%            |
| XML entity handling     | ❌     | Critical | 90%+            |
| YAML entity handling    | ❌     | High     | 90%+            |
| Type coercion           | ❌     | High     | 100%            |
| Namespace normalization | ❌     | High     | 100%            |
| Command-line generation | ❌     | Critical | 90%+            |
| Parameter file parsing  | ❌     | High     | 90%+            |

### 12.2 Integration Tests

| Feature               | Status | Priority | Notes                      |
|-----------------------|--------|----------|----------------------------|
| Simple node launch    | ❌     | Critical | talker_listener.launch.xml |
| Node with parameters  | ❌     | Critical | Inline + file params       |
| Node with remappings  | ❌     | High     | Topic remapping            |
| Nested includes       | ❌     | High     | Multi-level includes       |
| Composable nodes      | ❌     | High     | Container + components     |
| Complex substitutions | ❌     | Medium   | Nested, recursive          |
| Lifecycle nodes       | ❌     | Low      | Lifecycle-aware            |
| Conditional actions   | ❌     | Low      | if/unless conditions       |

### 12.3 Comparison Tests

| Feature                        | Status | Priority | Notes                 |
|--------------------------------|--------|----------|-----------------------|
| vs dump_launch (simple)        | ❌     | Critical | Basic node launch     |
| vs dump_launch (parameters)    | ❌     | Critical | Param handling        |
| vs dump_launch (composable)    | ❌     | High     | Component system      |
| vs dump_launch (Autoware)      | ❌     | High     | Large real-world test |
| JSON field-by-field comparison | ❌     | Critical | Exact matching        |

### 12.4 Performance Tests

| Feature                    | Status | Priority | Target       |
|----------------------------|--------|----------|--------------|
| Simple launch parse time   | ❌     | High     | <100ms       |
| Medium launch parse time   | ❌     | High     | <500ms       |
| Autoware launch parse time | ❌     | Critical | <5s (vs 40s) |
| Memory usage               | ❌     | Medium   | <100MB       |
| Parallel include scaling   | ❌     | Low      | Near-linear  |

---

## 13. Documentation

### 13.1 Code Documentation

| Feature               | Status | Priority | Notes                   |
|-----------------------|--------|----------|-------------------------|
| Module-level docs     | ❌     | High     | Each module doc comment |
| Public API docs       | ❌     | High     | All pub items           |
| Example code          | ❌     | Medium   | Doc examples            |
| Architecture diagrams | ❌     | Low      | Visual aids             |

### 13.2 User Documentation

| Feature                          | Status | Priority | Notes             |
|----------------------------------|--------|----------|-------------------|
| Usage guide                      | ❌     | High     | How to use parser |
| CLI help text                    | ❌     | High     | --help output     |
| Error message guide              | ❌     | Medium   | Common errors     |
| Migration guide from dump_launch | ❌     | Medium   | Switching guide   |

---

## 14. Performance Optimization

### 14.1 Parse Performance

| Feature                          | Status | Priority | Notes              |
|----------------------------------|--------|----------|--------------------|
| Lazy substitution resolution     | ❌     | Low      | Defer until needed |
| Parallel include parsing         | ❌     | Low      | Tokio/rayon        |
| Substitution caching             | ❌     | Low      | Memoization        |
| Zero-copy parsing where possible | ❌     | Low      | Cow<str>           |

### 14.2 Memory Optimization

| Feature               | Status | Priority | Notes                    |
|-----------------------|--------|----------|--------------------------|
| String interning      | ❌     | Low      | Deduplicate strings      |
| Arena allocation      | ❌     | Low      | Reduce allocations       |
| Streaming JSON output | ❌     | Low      | Don't buffer entire JSON |

---

## 15. CLI Interface

### 15.1 Commands

| Feature                     | Status | Priority | Notes                   |
|-----------------------------|--------|----------|-------------------------|
| `parse launch <pkg> <file>` | ❌     | Critical | Main command            |
| `parse file <path>`         | ❌     | High     | Direct file path        |
| `--output <path>`           | ❌     | High     | Custom output path      |
| `--format <json|yaml>`      | ❌     | Low      | Output format           |
| `--validate`                | ❌     | Medium   | Validate without output |
| `--verbose`                 | ❌     | Medium   | Verbose logging         |
| `--quiet`                   | ❌     | Medium   | Suppress output         |
| `--help`                    | ❌     | High     | Help text               |
| `--version`                 | ❌     | High     | Version info            |

### 15.2 Arguments

| Feature                         | Status | Priority | Notes                 |
|---------------------------------|--------|----------|-----------------------|
| Launch arguments (`key:=value`) | ❌     | Critical | Pass to launch file   |
| Package path override           | ❌     | Low      | Custom package paths  |
| ROS_PACKAGE_PATH env            | ❌     | High     | Standard ROS behavior |

---

## Summary Statistics

### Overall Progress

- **Total Features**: 229 features identified
- **Completed**: 6 (2.6%)
- **In Progress**: 0 (0%)
- **Not Started**: 223 (97.4%)

### By Category

| Category               | Total | Complete | In Progress | Not Started |
|------------------------|-------|----------|-------------|-------------|
| Core Infrastructure    | 13    | 6        | 0           | 7           |
| XML Parser             | 33    | 0        | 0           | 33          |
| YAML Parser            | 9     | 0        | 0           | 9           |
| Substitution Engine    | 20    | 0        | 0           | 20          |
| Launch Tree Building   | 11    | 0        | 0           | 11          |
| Node Metadata          | 34    | 0        | 0           | 34          |
| Composable Nodes       | 13    | 0        | 0           | 13          |
| record.json Generation | 11    | 0        | 0           | 11          |
| Lifecycle Nodes        | 3     | 0        | 0           | 3           |
| Conditions             | 4     | 0        | 0           | 4           |
| Error Handling         | 17    | 0        | 0           | 17          |
| Testing                | 20    | 0        | 0           | 20          |
| Documentation          | 8     | 0        | 0           | 8           |
| Performance            | 6     | 0        | 0           | 6           |
| CLI Interface          | 12    | 0        | 0           | 12          |

### By Priority

| Priority | Count | Percentage |
|----------|-------|------------|
| Critical | 71    | 31.0%      |
| High     | 85    | 37.1%      |
| Medium   | 55    | 24.0%      |
| Low      | 18    | 7.9%       |

---

## Next Milestone: MVP (Minimum Viable Parser)

**Target:** Basic XML parser with simple nodes and substitutions

**Required Features (16 critical features):**
1. ✅ Rust package structure
2. ❌ XML file loading
3. ❌ XML entity abstraction
4. ❌ `<node>` action parsing
5. ❌ `<arg>` action parsing
6. ❌ Basic substitution grammar (`var`, `env`)
7. ❌ LaunchContext structure
8. ❌ Package name extraction
9. ❌ Executable name extraction
10. ❌ Node name extraction
11. ❌ Namespace extraction
12. ❌ Inline parameters
13. ❌ Topic remappings
14. ❌ Command-line generation
15. ❌ NodeRecord generation
16. ❌ JSON serialization

**Success Criteria:**
- Parse `talker_listener.launch.xml` from demos
- Generate valid `record.json`
- Compare output with dump_launch (structural match)

---

## References

- **Architecture Analysis**: `docs/ROS2_LAUNCH_ARCHITECTURE.md`
- **Bottleneck Analysis**: `docs/DUMP_LAUNCH_ANALYSIS.md`
- **Format Specification**: `docs/RECORD_JSON_FORMAT.md`
- **Research Summary**: `docs/RESEARCH_SUMMARY.md`
- **Roadmap**: `docs/roadmap/phase-1-PROJECT_SETUP.md`
