# Phase 5: Python Launch File Support & Remaining Features

**Status**: 📋 **NOT STARTED**
**Priority**: HIGH (for full Autoware compatibility)
**Dependencies**: Phase 4 (Complete ✅)

---

## Overview

Phase 5 focuses on achieving **100% Autoware compatibility** by:
1. **Quick wins**: YAML file handling and missing action types
2. **Major feature**: Python launch file support via pyo3
3. **Polish**: Documentation and testing

**Current Coverage**: 85-90% (XML files only)
**Target Coverage**: 95-100% (XML + Python files)

---

## Phase 5.1: Quick Wins (Polish) ✨

**Priority**: HIGH
**Estimated Time**: 2-3 hours
**Status**: 📋 Not Started

### 5.1.1: YAML Configuration File Handling

**Priority**: HIGH (prevents error messages)
**Difficulty**: EASY
**Estimated Time**: 30 minutes

#### Problem
Parser tries to parse YAML config files as XML when they appear in `<include>` tags:
```
[INFO] Including: .../config/planning/preset/default_preset.yaml
Error: XML parsing error: unknown token at 1:1
```

#### Solution
Detect YAML file extensions and skip gracefully (like Python files):

```rust
// In src/lib.rs, process_include() function
fn process_include(&mut self, file_path: &Path) -> Result<()> {
    // Check file extension
    if let Some(ext) = file_path.extension() {
        match ext.to_str() {
            Some("py") => {
                log::warn!("Skipping Python launch file (not yet supported): {}",
                    file_path.display());
                return Ok(());
            }
            Some("yaml") | Some("yml") => {
                log::info!("Skipping YAML configuration file: {}",
                    file_path.display());
                return Ok(());
            }
            _ => {}
        }
    }

    // Continue with XML parsing
    // ...
}
```

#### Acceptance Criteria
- ✅ YAML files detected by extension
- ✅ Graceful skip with INFO log message
- ✅ No error thrown
- ✅ Test added for YAML include handling

#### Files to Modify
- `src/lib.rs` - Add extension check in include processing

---

### 5.1.2: Missing Action Types

**Priority**: MEDIUM
**Difficulty**: MEDIUM
**Estimated Time**: 2-4 hours (1-2 hours per action type)

#### Currently Missing

1. **`<node_container>`** - Composable node containers
2. **`<composable_node>`** - Nodes loaded into containers
3. **`<load_composable_node>`** - Dynamic loading
4. **`<set_env>` / `<unset_env>`** - Environment variable management (may be partial)

#### Implementation Priority

**High Priority**:
1. `<node_container>` - Commonly used in modern ROS 2 launch files

**Medium Priority**:
2. `<composable_node>` - Used with containers
3. `<set_env>` / `<unset_env>` - Environment setup

**Low Priority**:
4. `<load_composable_node>` - Dynamic scenarios (rare)

#### Implementation: `<node_container>`

```rust
// In src/lib.rs
"node_container" => {
    let name = entity.get_attr_str("name", true)?
        .ok_or_else(|| ParseError::MissingAttribute {
            element: "node_container".to_string(),
            attribute: "name".to_string(),
        })?;

    // Parse container-specific attributes
    let pkg = entity.get_attr_str("pkg", true)?
        .ok_or_else(|| /* error */)?;
    let exec = entity.get_attr_str("exec", true)?
        .ok_or_else(|| /* error */)?;
    let namespace = entity.get_attr_str("namespace", false)?;

    // Create a special NodeRecord for the container
    // Or store container metadata separately
    let container_node = NodeRecord {
        pkg,
        exec,
        name,
        namespace: namespace.unwrap_or_default(),
        params: vec![],
        remaps: vec![],
        env: vec![],
        args: vec![],
        ros_args: vec![],
        is_container: true, // Add this field to NodeRecord
    };

    self.nodes.push(container_node);

    // Process composable_node children
    for child in entity.children() {
        if child.tag_name() == "composable_node" {
            // Process composable node and associate with container
        }
    }
}
```

#### Implementation: `<composable_node>`

```rust
"composable_node" => {
    // Similar to <node> but with plugin/component info
    let plugin = entity.get_attr_str("plugin", true)?
        .ok_or_else(|| /* error */)?;
    let name = entity.get_attr_str("name", true)?
        .ok_or_else(|| /* error */)?;

    // Store as special node type or metadata
    let composable_node = ComposableNodeRecord {
        plugin,
        name,
        namespace: entity.get_attr_str("namespace", false)?,
        params: parse_params(entity)?,
        remaps: parse_remaps(entity)?,
        // Container info if nested
    };
}
```

#### Acceptance Criteria
- ✅ `<node_container>` parses without error
- ✅ `<composable_node>` parses and associates with container
- ✅ Container metadata captured in output
- ✅ Tests added for each action type
- ✅ Integration test with real Autoware container usage

#### Files to Modify
- `src/lib.rs` - Add action handlers
- `src/record.rs` - Add container fields to NodeRecord
- `tests/integration_tests.rs` - Add tests

---

## Phase 5.2: Python Launch File Support 🐍

**Priority**: HIGH (for 95-100% Autoware coverage)
**Difficulty**: HARD
**Estimated Time**: 1-2 weeks

### Strategy: Mock Python API with pyo3

Instead of parsing Python code or executing external tools, we'll:
1. Embed Python interpreter via pyo3
2. Provide a mock `launch` Python module that matches ROS 2's API
3. Execute Python launch files within our Rust process
4. Capture declared nodes/actions via our mock API

**Advantages**:
- ✅ Handles all Python features (loops, conditionals, imports)
- ✅ No external dependencies (embeds Python)
- ✅ Full control over execution
- ✅ Can capture exact node configurations
- ✅ Performance (cached interpreter)

**Disadvantages**:
- ⚠️ Requires Python interpreter embedded
- ⚠️ Complex implementation
- ⚠️ Need to maintain API compatibility

---

### 5.2.1: pyo3 Integration Setup

**Estimated Time**: 1 day

#### Add Dependencies

```toml
# Cargo.toml
[dependencies]
pyo3 = { version = "0.20", features = ["auto-initialize"] }

[features]
python = ["pyo3"]
```

#### Basic Python Module Structure

```rust
// src/python/mod.rs
use pyo3::prelude::*;

pub struct PythonLaunchExecutor {
    py: Python<'static>,
}

impl PythonLaunchExecutor {
    pub fn new() -> PyResult<Self> {
        pyo3::prepare_freethreaded_python();
        let py = Python::acquire_gil().python();

        // Register our mock launch module
        register_launch_module(py)?;

        Ok(Self { py })
    }

    pub fn execute_launch_file(&self, path: &Path, args: &HashMap<String, String>)
        -> PyResult<Vec<NodeRecord>> {
        // Execute the Python launch file
        // Capture nodes via our mock API
        todo!()
    }
}
```

---

### 5.2.2: Mock `launch` Python API

**Estimated Time**: 3-4 days

We need to mock the key ROS 2 launch Python API:

#### Core Classes to Mock

```python
# What Autoware Python launch files use:
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
```

#### Implementation Approach

```rust
// src/python/launch_api.rs

use pyo3::prelude::*;
use pyo3::types::PyModule;
use std::sync::{Arc, Mutex};

// Thread-safe storage for captured nodes
lazy_static! {
    static ref CAPTURED_NODES: Arc<Mutex<Vec<NodeRecord>>> =
        Arc::new(Mutex::new(Vec::new()));
}

/// Mock LaunchDescription class
#[pyclass]
struct LaunchDescription {
    actions: Vec<PyObject>,
}

#[pymethods]
impl LaunchDescription {
    #[new]
    fn new(actions: Vec<PyObject>) -> Self {
        Self { actions }
    }
}

/// Mock Node action
#[pyclass]
struct Node {
    package: String,
    executable: String,
    name: Option<String>,
    namespace: Option<String>,
    parameters: Vec<PyObject>,
    remappings: Vec<(String, String)>,
    arguments: Vec<String>,
}

#[pymethods]
impl Node {
    #[new]
    #[pyo3(signature = (package, executable, name=None, namespace=None, parameters=None, remappings=None, arguments=None))]
    fn new(
        package: String,
        executable: String,
        name: Option<String>,
        namespace: Option<String>,
        parameters: Option<Vec<PyObject>>,
        remappings: Option<Vec<(String, String)>>,
        arguments: Option<Vec<String>>,
    ) -> Self {
        let node = Self {
            package,
            executable,
            name,
            namespace,
            parameters: parameters.unwrap_or_default(),
            remappings: remappings.unwrap_or_default(),
            arguments: arguments.unwrap_or_default(),
        };

        // Capture this node
        Self::capture_node(&node);

        node
    }

    fn capture_node(node: &Node) {
        let record = NodeRecord {
            pkg: node.package.clone(),
            exec: node.executable.clone(),
            name: node.name.clone().unwrap_or_else(|| node.executable.clone()),
            namespace: node.namespace.clone().unwrap_or_default(),
            params: vec![], // TODO: parse parameters
            remaps: node.remappings.clone(),
            env: vec![],
            args: node.arguments.clone(),
            ros_args: vec![],
        };

        CAPTURED_NODES.lock().unwrap().push(record);
    }
}

/// Mock LaunchConfiguration substitution
#[pyclass]
struct LaunchConfiguration {
    variable_name: String,
}

#[pymethods]
impl LaunchConfiguration {
    #[new]
    fn new(variable_name: String) -> Self {
        Self { variable_name }
    }

    fn perform(&self, context: PyObject) -> String {
        // Resolve from our context
        // For now, return placeholder
        format!("$(var {})", self.variable_name)
    }
}

/// Register our mock modules
pub fn register_launch_module(py: Python) -> PyResult<()> {
    // Create launch module
    let launch_module = PyModule::new(py, "launch")?;
    launch_module.add_class::<LaunchDescription>()?;

    // Create launch.actions submodule
    let actions_module = PyModule::new(py, "actions")?;
    // Add action classes

    // Create launch.substitutions submodule
    let substitutions_module = PyModule::new(py, "substitutions")?;
    substitutions_module.add_class::<LaunchConfiguration>()?;

    // Create launch_ros module
    let launch_ros_module = PyModule::new(py, "launch_ros")?;

    // Create launch_ros.actions submodule
    let ros_actions_module = PyModule::new(py, "launch_ros.actions")?;
    ros_actions_module.add_class::<Node>()?;

    // Register modules in sys.modules
    py.import("sys")?
        .getattr("modules")?
        .set_item("launch", launch_module)?;
    py.import("sys")?
        .getattr("modules")?
        .set_item("launch.actions", actions_module)?;
    py.import("sys")?
        .getattr("modules")?
        .set_item("launch.substitutions", substitutions_module)?;
    py.import("sys")?
        .getattr("modules")?
        .set_item("launch_ros", launch_ros_module)?;
    py.import("sys")?
        .getattr("modules")?
        .set_item("launch_ros.actions", ros_actions_module)?;

    Ok(())
}
```

---

### 5.2.3: Python Launch File Execution

**Estimated Time**: 2-3 days

```rust
// src/python/executor.rs

impl PythonLaunchExecutor {
    pub fn execute_launch_file(
        &self,
        path: &Path,
        args: &HashMap<String, String>
    ) -> Result<Vec<NodeRecord>> {
        Python::with_gil(|py| {
            // Clear captured nodes
            CAPTURED_NODES.lock().unwrap().clear();

            // Read the Python file
            let code = std::fs::read_to_string(path)?;

            // Create a context for execution
            let locals = PyDict::new(py);

            // Set launch arguments in the context
            for (key, value) in args {
                locals.set_item(key, value)?;
            }

            // Execute the Python code
            py.run(&code, None, Some(locals))?;

            // Look for generate_launch_description function
            let generate_fn = locals.get_item("generate_launch_description")
                .ok_or_else(|| Error::PythonLaunchFileError(
                    "No generate_launch_description function found".to_string()
                ))?;

            // Call the function
            let launch_description = generate_fn.call0()?;

            // The Node constructors already captured nodes via CAPTURED_NODES
            // Return the captured nodes
            let nodes = CAPTURED_NODES.lock().unwrap().clone();

            Ok(nodes)
        })
    }
}
```

---

### 5.2.4: Integration with Main Parser

**Estimated Time**: 1 day

```rust
// src/lib.rs

use crate::python::PythonLaunchExecutor;

impl LaunchParser {
    fn process_include(&mut self, file_path: &Path) -> Result<()> {
        // Check file extension
        if let Some(ext) = file_path.extension() {
            match ext.to_str() {
                Some("py") => {
                    #[cfg(feature = "python")]
                    {
                        log::info!("Processing Python launch file: {}", file_path.display());
                        return self.process_python_launch_file(file_path);
                    }

                    #[cfg(not(feature = "python"))]
                    {
                        log::warn!("Skipping Python launch file (Python support not enabled): {}",
                            file_path.display());
                        return Ok(());
                    }
                }
                Some("yaml") | Some("yml") => {
                    log::info!("Skipping YAML configuration file: {}", file_path.display());
                    return Ok(());
                }
                _ => {}
            }
        }

        // Continue with XML parsing
        self.process_xml_launch_file(file_path)
    }

    #[cfg(feature = "python")]
    fn process_python_launch_file(&mut self, file_path: &Path) -> Result<()> {
        let executor = PythonLaunchExecutor::new()
            .map_err(|e| Error::PythonError(e.to_string()))?;

        // Get current launch arguments
        let args = self.context.configurations();

        // Execute and get nodes
        let nodes = executor.execute_launch_file(file_path, &args)
            .map_err(|e| Error::PythonLaunchFileError(e.to_string()))?;

        // Add nodes to our collection
        self.nodes.extend(nodes);

        Ok(())
    }
}
```

---

### 5.2.5: Testing & Validation

**Estimated Time**: 2 days

#### Unit Tests

```rust
#[cfg(test)]
mod python_tests {
    use super::*;

    #[test]
    fn test_simple_python_node() {
        let python_code = r#"
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker_node'
        )
    ])
"#;

        // Execute and validate
        let executor = PythonLaunchExecutor::new().unwrap();
        let nodes = executor.execute_code(python_code).unwrap();

        assert_eq!(nodes.len(), 1);
        assert_eq!(nodes[0].pkg, "demo_nodes_cpp");
        assert_eq!(nodes[0].exec, "talker");
        assert_eq!(nodes[0].name, "talker_node");
    }

    #[test]
    fn test_python_with_substitutions() {
        let python_code = r#"
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            namespace=LaunchConfiguration('robot_namespace')
        )
    ])
"#;

        let executor = PythonLaunchExecutor::new().unwrap();
        let mut args = HashMap::new();
        args.insert("robot_namespace".to_string(), "robot1".to_string());

        let nodes = executor.execute_code_with_args(python_code, &args).unwrap();

        assert_eq!(nodes[0].namespace, "robot1");
    }
}
```

#### Integration Tests with Autoware

```rust
#[test]
#[cfg(feature = "python")]
fn test_autoware_python_launch_file() {
    // Use actual Autoware Python launch file
    let path = Path::new("tests/autoware/global_params.launch.py");

    if !path.exists() {
        // Skip if Autoware not available
        return;
    }

    let executor = PythonLaunchExecutor::new().unwrap();
    let nodes = executor.execute_launch_file(path, &HashMap::new()).unwrap();

    // Validate we captured some nodes
    assert!(!nodes.is_empty());
}
```

---

### 5.2.6: Documentation

**Estimated Time**: 1 day

#### Update README

```markdown
## Python Launch File Support

The parser supports Python launch files via embedded Python interpreter (pyo3).

### Requirements
- Python 3.8+ installed
- Enable the `python` feature:
  ```bash
  cargo build --features python
  ```

### Supported Python API
- `launch.LaunchDescription`
- `launch_ros.actions.Node`
- `launch.actions.DeclareLaunchArgument`
- `launch.substitutions.LaunchConfiguration`
- More classes as needed...

### Limitations
- OpaqueFunction and custom callbacks have limited support
- Some advanced substitutions may not work identically to ROS 2
```

---

## Phase 5.3: Advanced Polish 💎

**Priority**: LOW
**Estimated Time**: 2-3 days

### 5.3.1: Comprehensive Documentation
- API documentation (rustdoc)
- Usage guide
- Migration guide from dump_launch
- Python API compatibility guide

### 5.3.2: Performance Optimization
- Benchmark Python execution overhead
- Cache Python interpreter between files
- Parallel processing of independent launch files

### 5.3.3: Error Handling
- Better error messages for Python execution failures
- Stack traces for Python errors
- Suggestions for common mistakes

---

## Implementation Timeline

### Week 1: Quick Wins
- **Day 1**: YAML file handling (0.5 day)
- **Day 1-2**: `<node_container>` support (1 day)
- **Day 2-3**: `<composable_node>` support (1 day)
- **Day 3-4**: `<set_env>` / `<unset_env>` support (1 day)
- **Day 4-5**: Testing and integration (1 day)

### Week 2: Python Support - Setup
- **Day 1**: pyo3 integration setup
- **Day 2-3**: Mock launch.LaunchDescription
- **Day 3-4**: Mock launch_ros.actions.Node
- **Day 4-5**: Mock substitutions and basic API

### Week 3: Python Support - Execution
- **Day 1-2**: Python file execution engine
- **Day 2-3**: Integration with main parser
- **Day 3-4**: Testing with simple Python files
- **Day 4-5**: Testing with Autoware Python files

### Week 4: Polish & Documentation
- **Day 1-2**: Advanced Python API classes
- **Day 2-3**: Error handling and edge cases
- **Day 3-4**: Documentation
- **Day 4-5**: Final testing and validation

**Total Estimated Time**: 3-4 weeks for full implementation

---

## Success Criteria

### Phase 5.1: Quick Wins
- ✅ YAML files skip gracefully
- ✅ `<node_container>` parses correctly
- ✅ `<composable_node>` parses correctly
- ✅ No errors on Autoware launch files

### Phase 5.2: Python Support
- ✅ Simple Python launch files execute
- ✅ Nodes captured from Python files
- ✅ Launch arguments passed correctly
- ✅ LaunchConfiguration substitutions work
- ✅ Autoware Python files parse successfully
- ✅ 95%+ Autoware node coverage

### Overall
- ✅ 100% of Autoware XML files supported
- ✅ 90%+ of Autoware Python files supported
- ✅ 95%+ overall Autoware node coverage
- ✅ No crashes on any Autoware launch files
- ✅ Clear documentation for Python support

---

## Risk Assessment

### Technical Risks

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| pyo3 integration issues | Medium | High | Start with simple examples, extensive testing |
| Python API compatibility | High | Medium | Implement incrementally, test with real files |
| Performance overhead | Low | Medium | Cache interpreter, profile execution |
| Python version compatibility | Medium | Low | Support Python 3.8+ (common baseline) |

### Implementation Risks

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| Scope creep (too many Python features) | High | Medium | Focus on Autoware use cases only |
| Maintenance burden | Medium | Medium | Good documentation, automated tests |
| Feature parity with ROS 2 | Medium | Low | Document limitations clearly |

---

## Future Enhancements (Post Phase 5)

### Phase 6: Advanced Python Features
- Event handlers (`OnProcessStart`, `OnProcessExit`)
- Conditional actions (`IfCondition`, `UnlessCondition`)
- Timers and delayed actions
- OpaqueFunction full support

### Phase 7: Performance & Scale
- Parallel launch file parsing
- Incremental parsing for large launch trees
- Memory optimization
- Caching and memoization

### Phase 8: IDE Integration
- Language server protocol (LSP) for launch files
- Syntax highlighting
- Auto-completion
- Error checking

---

## References

- [pyo3 Documentation](https://pyo3.rs/)
- [ROS 2 Launch Python API](https://github.com/ros2/launch)
- [Autoware Launch Files](https://github.com/autowarefoundation/autoware.universe)
- [Phase 4 Roadmap](./phase-2-MVP_XML_PARSER.md)

---

**Last Updated**: 2026-01-18
**Status**: 📋 Planning Phase
**Next Step**: Begin Phase 5.1 (Quick Wins)
