# Phase 5: Python Launch File Support

**Status**: ✅ Phase 5.1 Complete | 📋 Phase 5.2 Not Started
**Priority**: HIGH (for 95-100% Autoware coverage)
**Dependencies**: Phase 4 Complete ✅

---

## Overview

Achieve **full Autoware compatibility** through Python launch file support via pyo3 embedded interpreter with mock ROS 2 API.

**Current**: 90% coverage (XML only)
**Target**: 95-100% coverage (XML + Python)

---

## Phase 5.1: Quick Wins ✅ COMPLETE

**Time**: 1 day
**Status**: ✅ Complete (Session 8)

| Task | Status | Result |
|------|--------|--------|
| YAML file handling | ✅ | Skip .yaml/.yml with INFO log |
| `<set_env>` / `<unset_env>` | ✅ | Both hyphen/underscore variants |
| `<node_container>` | ✅ | Parse as regular node |
| `<composable_node>` | ✅ | Graceful handling in containers |

**Outcome**: 243 tests passing, 0 errors on Autoware XML files

---

## Phase 5.2: Python Support via pyo3

**Time**: 3-4 weeks
**Status**: 📋 Not Started

### Architecture

**See**: `docs/PYTHON_WORKSPACE_ARCHITECTURE.md`

**Strategy**: Embed Python interpreter, mock `launch` API, capture nodes on construction

```
XML file → Python include → Execute with mock API → Capture nodes → NodeRecord
```

---

### Work Items

#### 5.2.1: Workspace Setup (Day 1)

**Tasks**:
- [ ] Add pyo3 dependency with `python` feature flag
- [ ] Create `src/python/` module structure
- [ ] Set up conditional compilation
- [ ] Add build configuration

**Files**:
- `Cargo.toml`: Add pyo3, feature flags
- `src/python/mod.rs`: Module root
- `src/lib.rs`: Conditional Python integration

**Expected Result**: Compiles with/without `--features python`

**Guidance**:
```toml
[dependencies]
pyo3 = { version = "0.20", features = ["auto-initialize"], optional = true }

[features]
python = ["pyo3"]
```

---

#### 5.2.2: Python Executor (Days 1-2)

**Tasks**:
- [ ] `PythonLaunchExecutor` struct
- [ ] Initialize Python interpreter (cached)
- [ ] Execute Python file with `Python::with_gil`
- [ ] Call `generate_launch_description()`
- [ ] Global capture storage (`CAPTURED_NODES`)

**Files**:
- `src/python/executor.rs`
- `src/python/bridge.rs`

**Expected Result**: Can execute simple Python file

**Guidance**:
```rust
static CAPTURED_NODES: Lazy<Arc<Mutex<Vec<NodeCapture>>>> =
    Lazy::new(|| Arc::new(Mutex::new(Vec::new())));
```

---

#### 5.2.3: Mock Core API (Days 2-4)

**Priority Order**:
1. **Node** (most critical)
2. **LaunchDescription**
3. **LaunchConfiguration**
4. **DeclareLaunchArgument**

**Tasks**:
- [ ] `#[pyclass] Node` with capture on construction
- [ ] `#[pyclass] LaunchDescription`
- [ ] `#[pyclass] LaunchConfiguration`
- [ ] `#[pyclass] DeclareLaunchArgument`
- [ ] Module registration in `sys.modules`

**Files**:
- `src/python/api/launch_ros.rs`: Node
- `src/python/api/launch.rs`: LaunchDescription
- `src/python/api/substitutions.rs`: LaunchConfiguration
- `src/python/api/actions.rs`: DeclareLaunchArgument
- `src/python/api/mod.rs`: Module registration

**Expected Result**: Simple Python launch files work

**Guidance**:
```rust
#[pymethods]
impl Node {
    #[new]
    fn new(package: String, executable: String, /* ... */) -> Self {
        let node = Self { package, executable, /* ... */ };
        CAPTURED_NODES.lock().unwrap().push(NodeCapture::from_node(&node));
        node
    }
}
```

---

#### 5.2.4: Container Support (Days 5-6)

**Tasks**:
- [ ] `#[pyclass] ComposableNodeContainer`
- [ ] `#[pyclass] ComposableNode`
- [ ] Container-node association

**Files**:
- `src/python/api/launch_ros.rs`: Container classes

**Expected Result**: Composable nodes work in Python

---

#### 5.2.5: Substitution Support (Days 7-8)

**Tasks**:
- [ ] `PathJoinSubstitution`
- [ ] `FindPackageShare`
- [ ] `TextSubstitution`
- [ ] Resolve to Rust substitution format

**Files**:
- `src/python/api/substitutions.rs`

**Expected Result**: Substitutions in Python resolve correctly

**Guidance**:
```rust
impl LaunchConfiguration {
    fn __str__(&self) -> String {
        format!("$(var {})", self.variable_name)
    }
}
```

---

#### 5.2.6: Integration (Days 9-10)

**Tasks**:
- [ ] Integrate executor in `LaunchTraverser`
- [ ] Convert `NodeCapture` → `NodeRecord`
- [ ] Pass LaunchContext to Python
- [ ] Merge Python + XML nodes

**Files**:
- `src/lib.rs`: `process_python_file()`
- `src/python/bridge.rs`: Conversion logic

**Expected Result**: Mixed XML+Python launch files work

**Guidance**:
```rust
#[cfg(feature = "python")]
fn process_python_file(&mut self, path: &Path) -> Result<()> {
    let executor = python::PythonLaunchExecutor::new()?;
    let nodes = executor.execute_launch_file(path, &self.context.configurations())?;
    self.records.extend(nodes);
    Ok(())
}
```

---

#### 5.2.7: Testing (Days 11-14)

**Tasks**:
- [ ] Unit tests for each mock class
- [ ] Python fixture files (simple, params, containers)
- [ ] Integration tests with Autoware files
- [ ] Error handling tests

**Files**:
- `tests/python_tests.rs`
- `tests/fixtures/python/*.launch.py`

**Expected Result**: 95%+ Autoware coverage

**Test Fixtures**:
```python
# tests/fixtures/python/simple.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='demo', executable='talker', name='talker')
    ])
```

---

#### 5.2.8: Documentation (Days 15-16)

**Tasks**:
- [ ] Update README with Python support
- [ ] Document supported/unsupported features
- [ ] Python API compatibility guide
- [ ] Build instructions with `--features python`

**Files**:
- `README.md`
- `docs/PYTHON_API_SUPPORT.md`

---

### Implementation Timeline

| Week | Focus | Deliverable |
|------|-------|-------------|
| **1** | Setup + Core API | Node, LaunchDescription working |
| **2** | Containers + Substitutions | Composable nodes, path substitutions |
| **3** | Integration + Error Handling | Mixed XML+Python files |
| **4** | Testing + Documentation | Autoware validation, docs |

---

## Success Criteria

### Phase 5.1 ✅
- [x] YAML files skip gracefully
- [x] node_container parses
- [x] composable_node handled
- [x] 0 errors on Autoware XML

### Phase 5.2 📋
- [ ] Simple Python launch files execute
- [ ] Nodes captured from Python
- [ ] LaunchConfiguration resolves
- [ ] Composable nodes in containers
- [ ] 95%+ Autoware node coverage
- [ ] No crashes on Python files

---

## API Support Matrix

### Must Implement (Week 1-2)
| API | Priority | Notes |
|-----|----------|-------|
| `launch.LaunchDescription` | Critical | Container for actions |
| `launch_ros.actions.Node` | Critical | Primary node type |
| `launch.actions.DeclareLaunchArgument` | Critical | Arguments |
| `launch.substitutions.LaunchConfiguration` | Critical | Variable access |

### Should Implement (Week 2-3)
| API | Priority | Notes |
|-----|----------|-------|
| `launch_ros.actions.ComposableNodeContainer` | High | Autoware uses heavily |
| `launch_ros.descriptions.ComposableNode` | High | Component definitions |
| `launch.substitutions.PathJoinSubstitution` | High | Path manipulation |
| `launch.substitutions.FindPackageShare` | High | Package paths |

### Nice to Have (Week 3-4)
| API | Priority | Notes |
|-----|----------|-------|
| `launch.actions.OpaqueFunction` | Medium | Limited support |
| `launch.conditions.IfCondition` | Medium | Conditionals |
| `launch.conditions.UnlessCondition` | Medium | Conditionals |

---

## Risk Mitigation

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| pyo3 integration complexity | Medium | High | Start simple, incremental testing |
| Python API compatibility | High | Medium | Focus on Autoware patterns only |
| Performance overhead | Low | Medium | Cache interpreter, profile |
| Scope creep | High | Medium | Limit to Autoware use cases |

---

## Limitations (Initial Release)

### Supported ✅
- Node creation
- Launch arguments
- Basic substitutions
- Composable nodes
- Python-native conditionals/loops

### Not Supported ❌
- Full `OpaqueFunction` (complex callbacks)
- Event handlers (`OnProcessStart`, etc.)
- Custom Python dependencies
- Launch services

---

## Testing Strategy

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    #[test]
    fn test_node_capture() {
        // Test Node capture mechanism
    }
}
```

### Integration Tests
```bash
cargo test --features python --test python_tests
```

### Autoware Validation
- Parse all Autoware Python launch files
- Compare node counts with ROS 2 output
- Verify parameter passing

---

## Build Commands

```bash
# Standard build (no Python)
cargo build

# With Python support
cargo build --features python

# Test with Python
cargo test --features python

# Clippy
cargo clippy --features python
```

---

## Next Steps After Phase 5

### Phase 6: Advanced Python Features
- Full OpaqueFunction support
- Event system
- Launch services
- Timers

### Phase 7: Performance
- Parallel file parsing
- Lazy evaluation
- Memory optimization

---

## References

- **Architecture**: `docs/PYTHON_WORKSPACE_ARCHITECTURE.md`
- **pyo3 Docs**: https://pyo3.rs/
- **ROS 2 Launch**: https://github.com/ros2/launch
- **Autoware**: https://github.com/autowarefoundation/autoware.universe

---

**Last Updated**: 2026-01-18
**Status**: Phase 5.1 Complete ✅ | Phase 5.2 Design Phase
