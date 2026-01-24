# Phase 8: ROS API Completeness

**Status**: 📝 Planned (Session 14)
**Priority**: MEDIUM (for broader ROS 2 ecosystem compatibility)
**Dependencies**: Phase 7 Complete ✅

---

## Overview

Implement remaining features from official ROS 2 launch repositories to achieve broader ecosystem compatibility beyond Autoware.

**Current Coverage**: 24/56 official ROS features (43%)
**Target Coverage**: 40+/56 (70%+)

**Key Insight**: While we have 100% Autoware compatibility, many ROS 2 launch files in the broader ecosystem use additional features not found in Autoware. This phase targets commonly-used features to maximize real-world compatibility.

**Estimated Time**: 3-4 weeks (phased implementation)

---

## Gap Analysis

### Current Implementation Status

Based on official ROS 2 Humble launch repositories:

| Category                  | Implemented | Total | Coverage |
|---------------------------|-------------|-------|----------|
| `launch.actions`          | 10          | 24    | 42%      |
| `launch.substitutions`    | 8           | 17    | 47%      |
| `launch_ros.actions`      | 5           | 11    | 45%      |
| `launch_ros.substitutions`| 1           | 4     | 25%      |
| **Total**                 | **24**      | **56**| **43%**  |

### Implementation Priorities

Features prioritized by:
1. **Usage frequency** in ROS 2 ecosystem
2. **User requests** and common patterns
3. **Complexity vs. value** ratio
4. **Dependencies** on other features

---

## Work Items

### Phase 8.1: High-Priority Substitutions 🔴 HIGH

**Priority**: HIGH
**Estimated Time**: 3-4 days
**Expected Impact**: Better conditional logic and configuration management

#### 8.1.1: EqualsSubstitution / NotEqualsSubstitution

**Impact**: Common pattern for conditional configuration

**Tasks**:
- [ ] Implement `EqualsSubstitution` class
  - [ ] Parse two substitution arguments
  - [ ] Resolve both and compare
  - [ ] Return "true" or "false" string
- [ ] Implement `NotEqualsSubstitution` class
  - [ ] Same as Equals but inverted result
- [ ] Add tests
  - [ ] String equality/inequality
  - [ ] Numeric equality
  - [ ] With nested substitutions
  - [ ] In condition contexts

**Files**:
- `src/python/api/substitutions.rs` (new classes)
- `src/python/api/mod.rs` (export)

**Example Usage**:
```python
from launch.substitutions import EqualsSubstitution, LaunchConfiguration

EqualsSubstitution(LaunchConfiguration('use_sim_time'), 'true')
```

**Validation**:
- Unit tests for equality comparisons
- Integration test with conditional nodes

---

#### 8.1.2: IfElseSubstitution

**Impact**: Ternary conditional - very common pattern

**Tasks**:
- [ ] Implement `IfElseSubstitution` class
  - [ ] Parse condition, if_value, else_value
  - [ ] Evaluate condition substitution
  - [ ] Return appropriate branch
- [ ] Add tests
  - [ ] With boolean conditions
  - [ ] With EqualsSubstitution
  - [ ] Nested IfElse

**Files**:
- `src/python/api/substitutions.rs`

**Example Usage**:
```python
IfElseSubstitution(
    condition=EqualsSubstitution(LaunchConfiguration('mode'), 'debug'),
    if_value='debug.yaml',
    else_value='release.yaml'
)
```

---

#### 8.1.3: FileContent Substitution

**Impact**: Read configuration from files

**Tasks**:
- [ ] Implement `FileContent` substitution
  - [ ] Parse file path substitution
  - [ ] Read file contents
  - [ ] Return as string
  - [ ] Handle missing files gracefully
- [ ] Add tests
  - [ ] Read existing file
  - [ ] Missing file error handling
  - [ ] With path substitutions

**Files**:
- `src/python/api/substitutions.rs`

**Example Usage**:
```python
FileContent(PathJoinSubstitution([FindPackageShare('pkg'), 'config', 'value.txt']))
```

---

### Phase 8.2: Parameter Management Actions 🟡 HIGH

**Priority**: HIGH
**Estimated Time**: 2-3 days
**Expected Impact**: Common parameter loading pattern

#### 8.2.1: SetParametersFromFile

**Impact**: Very common pattern for loading parameter files

**Tasks**:
- [ ] Implement `SetParametersFromFile` action
  - [ ] Parse filename substitution
  - [ ] Load YAML file
  - [ ] Apply to target nodes (all or specific)
  - [ ] Support node_name filter
- [ ] Add tests
  - [ ] Load params for all nodes
  - [ ] Load params for specific node
  - [ ] With substitutions in filename
  - [ ] Missing file handling

**Files**:
- `src/python/api/actions.rs`

**Example Usage**:
```python
SetParametersFromFile(
    PathJoinSubstitution([FindPackageShare('pkg'), 'config', 'params.yaml']),
    node_name='my_node'  # Optional
)
```

---

### Phase 8.3: Lifecycle Node Support 🟡 MEDIUM

**Priority**: MEDIUM
**Estimated Time**: 3-4 days
**Expected Impact**: Managed node lifecycle support

#### 8.3.1: LifecycleNode Action

**Impact**: Common pattern for managed nodes

**Tasks**:
- [ ] Implement `LifecycleNode` class (extends Node)
  - [ ] All Node features
  - [ ] Additional lifecycle-specific metadata
  - [ ] State transition configuration
- [ ] Update record generation
  - [ ] Add lifecycle flag to NodeRecord
  - [ ] Preserve lifecycle metadata
- [ ] Add tests
  - [ ] Basic lifecycle node
  - [ ] With parameters and remaps
  - [ ] State configuration

**Files**:
- `src/python/api/launch_ros.rs`
- `src/record/types.rs` (add lifecycle field)

**Example Usage**:
```python
from launch_ros.actions import LifecycleNode

LifecycleNode(
    package='my_pkg',
    executable='my_node',
    name='lifecycle_node',
    namespace='/',
)
```

---

#### 8.3.2: LifecycleTransition Action

**Impact**: State transition control

**Tasks**:
- [ ] Implement `LifecycleTransition` action
  - [ ] Parse node reference
  - [ ] Parse transition id/label
  - [ ] Capture for record (informational)
- [ ] Add tests

**Files**:
- `src/python/api/actions.rs`

**Note**: This is primarily informational for static analysis. Runtime execution is not in scope.

---

### Phase 8.4: Environment Management 🟢 MEDIUM

**Priority**: MEDIUM
**Estimated Time**: 2 days
**Expected Impact**: Better environment control

#### 8.4.1: Stack Management Actions

**Tasks**:
- [ ] Implement `PushEnvironment` / `PopEnvironment`
  - [ ] Environment stack in LaunchContext
  - [ ] Push current env state
  - [ ] Pop and restore
- [ ] Implement `ResetEnvironment`
  - [ ] Reset to initial state
- [ ] Implement `AppendEnvironmentVariable`
  - [ ] Append to existing value
  - [ ] With separator
- [ ] Add tests

**Files**:
- `src/python/api/actions.rs`
- `src/substitution/context.rs` (environment stack)

**Example Usage**:
```python
PushEnvironment()
SetEnvironmentVariable('PATH', '/custom/path')
# ... nodes use custom PATH
PopEnvironment()
# PATH restored
```

---

### Phase 8.5: Launch Configuration Management 🟢 LOW

**Priority**: LOW
**Estimated Time**: 1-2 days
**Expected Impact**: Advanced configuration scoping

**Tasks**:
- [ ] Implement `PushLaunchConfigurations` / `PopLaunchConfigurations`
  - [ ] Configuration stack
  - [ ] Scoped configurations
- [ ] Implement `ResetLaunchConfigurations`
- [ ] Implement `UnsetLaunchConfiguration`
- [ ] Add tests

**Files**:
- `src/python/api/actions.rs`
- `src/python/bridge.rs` (global config stack)

---

### Phase 8.6: Additional ROS Substitutions 🔵 LOW

**Priority**: LOW
**Estimated Time**: 2 days
**Expected Impact**: Edge cases and convenience

#### 8.6.1: ExecutableInPackage

**Tasks**:
- [ ] Implement `ExecutableInPackage` substitution
  - [ ] Find package prefix
  - [ ] Locate executable in lib/<package>
  - [ ] Return full path
- [ ] Add tests

**Files**:
- `src/python/api/substitutions.rs`

**Example Usage**:
```python
ExecutableInPackage(package='my_pkg', executable='my_node')
```

---

#### 8.6.2: FindPackage (prefix path)

**Tasks**:
- [ ] Implement `FindPackage` substitution
  - [ ] Return package install prefix
  - [ ] Different from FindPackageShare (which returns share/)
- [ ] Add tests

**Files**:
- `src/python/api/substitutions.rs`

---

#### 8.6.3: Parameter Substitution

**Tasks**:
- [ ] Implement `Parameter` substitution
  - [ ] Read ROS parameter value
  - [ ] Return as string
- [ ] Add tests

**Files**:
- `src/python/api/substitutions.rs`

**Note**: Limited to static parameters set earlier in launch file.

---

#### 8.6.4: Utility Substitutions

**Tasks**:
- [ ] Implement `BooleanSubstitution`
  - [ ] Convert value to boolean string
- [ ] Implement `FindExecutable`
  - [ ] Search PATH for executable
- [ ] Implement `LaunchLogDir`
  - [ ] Return log directory path
- [ ] Implement `ThisLaunchFile`
  - [ ] Return full path to current launch file
- [ ] Add tests

**Files**:
- `src/python/api/substitutions.rs`

---

### Phase 8.7: Event System (Optional) 🔵 LOW

**Priority**: LOW
**Estimated Time**: 3-4 days
**Expected Impact**: Dynamic event handling (rarely used)

**Tasks**:
- [ ] Implement `EmitEvent` action
  - [ ] Create event object
  - [ ] Store in event list
- [ ] Implement `RegisterEventHandler` action
  - [ ] Register handler for event type
  - [ ] Capture actions to execute
- [ ] Implement `UnregisterEventHandler` action
- [ ] Add tests

**Files**:
- `src/python/api/actions.rs`
- `src/python/api/events.rs` (new)

**Note**: This is complex with limited benefit for static analysis. Consider deferring.

---

### Phase 8.8: Advanced Actions 🔵 LOW

**Priority**: LOW
**Estimated Time**: 2-3 days
**Expected Impact**: Edge cases

#### 8.8.1: ExecuteLocal

**Tasks**:
- [ ] Implement `ExecuteLocal` action
  - [ ] Similar to ExecuteProcess
  - [ ] Different execution context
- [ ] Add tests

**Files**:
- `src/python/api/actions.rs`

---

#### 8.8.2: OpaqueCoroutine

**Tasks**:
- [ ] Implement `OpaqueCoroutine` action
  - [ ] Basic async function support
  - [ ] Limited coroutine execution
- [ ] Add tests

**Files**:
- `src/python/api/actions.rs`

**Note**: Complex with limited usage. Consider deferring.

---

#### 8.8.3: Miscellaneous Actions

**Tasks**:
- [ ] Implement `ShutdownAction`
  - [ ] Informational capture
- [ ] Implement `RosTimer`
  - [ ] ROS time-based timer
- [ ] Implement `SetUseSimTime`
  - [ ] Global simulation time flag
- [ ] Implement `PushRosNamespace` (Python action)
  - [ ] Complement to XML push-ros-namespace
- [ ] Add tests

**Files**:
- `src/python/api/launch_ros.rs`

---

### Phase 8.9: Testing & Documentation ✅ CRITICAL

**Priority**: CRITICAL
**Estimated Time**: Ongoing
**Expected Impact**: Quality assurance

**Tasks**:
- [ ] Add comprehensive unit tests for each new feature
- [ ] Add integration tests
  - [ ] IfElseSubstitution with conditions
  - [ ] SetParametersFromFile workflows
  - [ ] Lifecycle node patterns
  - [ ] Environment stack management
- [ ] Update documentation
  - [ ] feature_list.md with new features
  - [ ] Python API compatibility notes
  - [ ] Usage examples
- [ ] Quality checks
  - [ ] All tests passing
  - [ ] 0 clippy warnings
  - [ ] Code formatted

**Success Criteria**:
- [ ] ROS API coverage ≥ 70% (40+/56 features)
- [ ] All 300+ tests passing
- [ ] 0 clippy warnings
- [ ] Documentation complete

---

## Implementation Timeline

### Phase 1: High-Priority Features (Week 1-2)

**Week 1**:
- Days 1-2: Phase 8.1 - Conditional substitutions (Equals, IfElse)
- Days 3-4: Phase 8.2 - SetParametersFromFile
- Day 5: Testing & documentation

**Week 2**:
- Days 1-3: Phase 8.3 - Lifecycle node support
- Days 4-5: Phase 8.4 - Environment management

### Phase 2: Medium-Priority Features (Week 3)

**Week 3**:
- Days 1-2: Phase 8.5 - Launch configuration management
- Days 3-4: Phase 8.6 - Additional substitutions
- Day 5: Testing & documentation

### Phase 3: Optional Features (Week 4, if needed)

**Week 4**:
- Days 1-2: Phase 8.7 (Optional) - Event system
- Days 3-4: Phase 8.8 (Optional) - Advanced actions
- Day 5: Final testing & documentation

---

## Success Metrics

### Coverage Targets

| Category                  | Current | Phase 1 | Phase 2 | Phase 3 | Target |
|---------------------------|---------|---------|---------|---------|--------|
| launch.actions            | 10/24   | 12/24   | 14/24   | 18/24   | 18/24  |
| launch.substitutions      | 8/17    | 11/17   | 14/17   | 15/17   | 14/17  |
| launch_ros.actions        | 5/11    | 7/11    | 8/11    | 10/11   | 8/11   |
| launch_ros.substitutions  | 1/4     | 1/4     | 4/4     | 4/4     | 4/4    |
| **Total**                 | **24/56** | **31/56** | **40/56** | **47/56** | **40/56** |
| **Percentage**            | **43%** | **55%** | **71%** | **84%** | **71%** |

### Milestone Checklist

- [ ] Phase 8.1: High-priority substitutions complete
- [ ] Phase 8.2: SetParametersFromFile complete
- [ ] Phase 8.3: Lifecycle node support complete
- [ ] ROS API coverage ≥ 55% (Phase 1 complete)
- [ ] ROS API coverage ≥ 70% (Phase 2 complete)
- [ ] All tests passing (300+ tests)
- [ ] Documentation updated
- [ ] User feedback incorporated

---

## Risk Assessment

### High Risk

**None** - All features are well-scoped and understood

### Medium Risk

- **Event system complexity** (Phase 8.7)
  - Mitigation: Mark as optional, defer if needed
  - Fallback: Skip this feature entirely

- **OpaqueCoroutine implementation** (Phase 8.8.2)
  - Mitigation: Limited async support, not full coroutines
  - Fallback: Defer to future phase

### Low Risk

- **Substitution implementations**
  - Well-defined scope and patterns
  - Similar to existing substitutions

- **Action implementations**
  - Most are simple capture-and-record patterns
  - Low complexity

---

## Phased Implementation Strategy

### Minimum Viable Phase 8 (Weeks 1-2)

Focus on **high-value, frequently-used features**:
- ✅ EqualsSubstitution / IfElseSubstitution
- ✅ SetParametersFromFile
- ✅ LifecycleNode

**Target**: 55% ROS API coverage (31/56 features)

### Extended Phase 8 (Weeks 1-3)

Add **commonly-used features**:
- ✅ All Phase 1 features
- ✅ Environment stack management
- ✅ Additional substitutions (ExecutableInPackage, FindPackage, Parameter)

**Target**: 71% ROS API coverage (40/56 features)

### Complete Phase 8 (Weeks 1-4, optional)

Add **edge case features**:
- ✅ All Phase 2 features
- ✅ Event system (if requested)
- ✅ Advanced actions

**Target**: 84% ROS API coverage (47/56 features)

---

## Dependencies and Prerequisites

### External Dependencies

- pyo3 0.20+ (already integrated)
- ROS 2 Humble packages (for testing)
- Python 3.10+ (already required)

### Internal Dependencies

- Phase 5 complete ✅ (Python support)
- Phase 7 complete ✅ (Performance optimization)
- All 279 tests passing ✅

---

## Post-Phase 8 Work

After completing Phase 8:

**Phase 9: Production Readiness**
- Comprehensive documentation
- Migration guide from dump_launch
- Performance benchmarks
- Production hardening

**Phase 10: Community & Ecosystem**
- ROS 2 package integration
- Community feedback
- Additional feature requests
- 1.0 release

---

## Notes

### Implementation Principles

1. **Incremental**: Each sub-phase adds value independently
2. **Tested**: Every feature has unit + integration tests
3. **Documented**: Update docs as features are added
4. **Quality**: Maintain 0 clippy warnings, 100% test pass rate

### Why 70% Target?

- Remaining 30% are edge cases rarely used in practice
- Autoware (100% compatible) is a good representative of real-world launch files
- Diminishing returns: 70% covers 95%+ of real-world usage
- Can add remaining features based on user requests

### User Feedback Loop

- Implement high-priority features first
- Gather feedback from real-world usage
- Prioritize remaining features based on actual need
- Avoid implementing rarely-used features

---

## Comparison: Before and After Phase 8

### Before Phase 8 (Current)

- ROS API Coverage: 43% (24/56)
- Autoware Coverage: 100% ✅
- Test Count: 279 ✅
- Known Limitations:
  - No conditional substitutions (IfElse, Equals)
  - No SetParametersFromFile
  - No lifecycle node support
  - Limited substitution types

### After Phase 8 (Target)

- ROS API Coverage: 71% (40/56) - **+28 percentage points**
- Autoware Coverage: 100% ✅ (maintained)
- Test Count: 325+ ✅ (+46 tests)
- Key Improvements:
  - ✅ Conditional logic (IfElse, Equals)
  - ✅ Parameter file loading
  - ✅ Lifecycle node support
  - ✅ Comprehensive substitution support
  - ✅ Environment stack management
  - ✅ Broader ROS 2 ecosystem compatibility

---

## Success Criteria

Phase 8 is considered complete when:

1. ✅ ROS API coverage ≥ 70% (40/56 features implemented)
2. ✅ All new features have ≥90% test coverage
3. ✅ Total test count ≥ 325
4. ✅ All tests passing (100%)
5. ✅ 0 clippy warnings
6. ✅ Documentation updated (feature_list.md, API docs)
7. ✅ Performance maintained (Autoware parse time <1s)
8. ✅ Autoware compatibility maintained (100%)

---

**Last Updated**: 2026-01-25 (Session 14)
**Next Review**: After Phase 8.1 completion
