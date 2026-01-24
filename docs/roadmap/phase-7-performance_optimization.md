# Phase 7: Performance Optimization

**Status**: 📋 Planned
**Priority**: HIGH (for production deployment)
**Dependencies**: Phase 6 Complete ✅

---

## Overview

Achieve **5-7x performance improvement** for complex launch files through systematic optimization.

**Current**: ~5s for Autoware planning_simulator.launch.xml
**Target**: <1s (5-7x improvement)

**Strategy**: Three-phase approach combining caching, architectural refactoring, and parallelization.

---

## Performance Goals

| Metric                  | Current | Phase 1 | Phase 2 | Phase 3 | Target |
|-------------------------|---------|---------|---------|---------|--------|
| **Autoware parse time** | ~5s     | ~3s     | ~2s     | ~1s     | <1s    |
| **Memory usage**        | ~50MB   | ~45MB   | ~35MB   | ~40MB   | <40MB  |
| **Package lookup**      | 10-20ms | <1ms    | <1ms    | <1ms    | <1ms   |
| **Simple launch**       | <10ms   | <5ms    | <5ms    | <3ms    | <5ms   |

---

## Phase 7.1: DashMap Caching (2-3 days) ⭐ HIGHEST IMPACT

**Time**: 2-3 days
**Priority**: P0 (quick win, high impact)
**Risk**: Low

### Overview

Eliminate I/O overhead through intelligent caching. Makes workload 80%+ CPU-bound.

**Expected Impact**: 60-80% improvement (Autoware: ~5s → ~3s)

### 7.1.1: Package Resolution Cache (2-3 hours)

**Problem**: `$(find-pkg-share pkg)` does 10+ filesystem checks per call, repeated for same packages.

**Solution**: Global DashMap cache keyed by package name.

**Tasks**:
- [ ] Add `dashmap = "5.5"` and `once_cell = "1.19"` dependencies
- [ ] Create global `PACKAGE_CACHE: Lazy<DashMap<String, String>>`
- [ ] Wrap `find_package_share()` with cache lookup
- [ ] Add cache statistics (hits/misses) with `log::trace!`
- [ ] Add `#[cfg(test)]` cache clearing function

**Files to modify**:
- `Cargo.toml`: Add dependencies
- `src/play_launch_parser/src/substitution/types.rs:138-166`: Wrap `find_package_share()`

**Implementation**:
```rust
use dashmap::DashMap;
use once_cell::sync::Lazy;

static PACKAGE_CACHE: Lazy<DashMap<String, String>> =
    Lazy::new(DashMap::new);

fn find_package_share(package_name: &str) -> Option<String> {
    // Fast path: Check cache (lock-free read)
    if let Some(entry) = PACKAGE_CACHE.get(package_name) {
        log::trace!("Package cache hit: {}", package_name);
        return Some(entry.value().clone());
    }

    log::debug!("Package cache miss: {}", package_name);

    // Slow path: Expensive filesystem lookup
    let result = find_package_share_uncached(package_name)?;

    // Cache result
    PACKAGE_CACHE.insert(package_name.to_string(), result.clone());
    Some(result)
}
```

**Testing**:
- [ ] Verify cache hits after first lookup
- [ ] Test concurrent access (multiple threads)
- [ ] Benchmark Autoware (expect 20-30% improvement alone)
- [ ] All 260 tests pass

**Success Criteria**:
- >95% cache hit rate for Autoware
- Package lookups <1ms (from 10-20ms)
- Memory increase <100KB

---

### 7.1.2: File Content Cache (2-3 hours)

**Problem**: Repeated includes read same files multiple times from disk.

**Solution**: DashMap cache with modification time validation.

**Tasks**:
- [ ] Create `CachedFile` struct with `content` and `modified` timestamp
- [ ] Create global `FILE_CACHE: Lazy<DashMap<PathBuf, CachedFile>>`
- [ ] Create `read_file_cached()` wrapper function
- [ ] Check modification time to invalidate stale cache
- [ ] Replace all `std::fs::read_to_string()` calls with cached version

**Files to modify**:
- `src/play_launch_parser/src/lib.rs:119, 348, 464, 693`: Replace file reads

**Implementation**:
```rust
use dashmap::DashMap;
use std::time::SystemTime;
use std::path::{Path, PathBuf};

struct CachedFile {
    content: String,
    modified: SystemTime,
}

static FILE_CACHE: Lazy<DashMap<PathBuf, CachedFile>> =
    Lazy::new(DashMap::new);

fn read_file_cached(path: &Path) -> Result<String> {
    let metadata = std::fs::metadata(path)?;
    let modified = metadata.modified()?;

    // Check cache with modification time
    if let Some(entry) = FILE_CACHE.get(path) {
        if entry.modified == modified {
            log::trace!("File cache hit: {}", path.display());
            return Ok(entry.content.clone());
        }
    }

    log::debug!("File cache miss: {}", path.display());

    // Read and cache
    let content = std::fs::read_to_string(path)?;
    FILE_CACHE.insert(
        path.to_path_buf(),
        CachedFile { content: content.clone(), modified }
    );

    Ok(content)
}
```

**Testing**:
- [ ] Verify cache hits for repeated includes
- [ ] Test cache invalidation on file modification
- [ ] Benchmark Autoware (expect 30-50% improvement)
- [ ] All 260 tests pass

**Success Criteria**:
- 30-50% reduction in file I/O time
- Memory increase <10MB (reasonable for Autoware)

---

### 7.1.3: Python Mutex Upgrade (1 hour)

**Problem**: `std::sync::Mutex` has high contention for Python global capture storage.

**Solution**: Replace with `parking_lot::Mutex` (faster, smaller).

**Tasks**:
- [ ] Add `parking_lot = "0.12"` dependency
- [ ] Replace `std::sync::Mutex` with `parking_lot::Mutex`
- [ ] Update all lock acquisitions (API is identical)

**Files to modify**:
- `Cargo.toml`: Add dependency
- `src/play_launch_parser/src/python/bridge.rs:11, 59, 80, 111`: Replace mutex type

**Implementation**:
```rust
// Before:
use std::sync::Mutex;
pub static CAPTURED_NODES: Lazy<Arc<Mutex<Vec<NodeCapture>>>> = ...;

// After:
use parking_lot::Mutex;
pub static CAPTURED_NODES: Lazy<Arc<Mutex<Vec<NodeCapture>>>> = ...;
// Lock API is identical: CAPTURED_NODES.lock().push(...)
```

**Testing**:
- [ ] All Python tests pass (15 tests)
- [ ] Benchmark Python-heavy workload (expect 15-25% improvement)

**Success Criteria**:
- 15-25% improvement for Python-heavy workloads
- No API changes needed (drop-in replacement)

---

### Phase 7.1 Deliverables

**Expected Results**:
- ✅ Autoware parse time: ~3s (from ~5s) = 60-80% improvement
- ✅ Package lookup cache hit rate: >95%
- ✅ File cache hit rate: >70%
- ✅ Memory increase: <10MB
- ✅ All 260 tests pass

**Dependencies Added**:
```toml
[dependencies]
dashmap = "5.5"           # Concurrent HashMap
parking_lot = "0.12"      # Faster mutexes
once_cell = "1.19"        # Already present, use more
```

---

## Phase 7.2: Hybrid Arc + Local Context (1 week) ⭐ ARCHITECTURE IMPROVEMENT

**Time**: 1 week
**Priority**: P0 (enables parallelization)
**Risk**: Medium (well-understood compiler pattern)

### Overview

Eliminate expensive context cloning through scope-chain pattern used by V8, Python, Rust compiler, LLVM.

**Current Problem**: Full HashMap clone on every `<include>` (50 includes × 20 vars = 1000 clones)

**Solution**: Parent scope frozen in `Arc<ParentScope>`, child has local HashMap for new variables only.

**Expected Impact**: 20-40% improvement + enables parallel processing (Autoware: ~3s → ~2s)

### 7.2.1: Refactor Context Structure (2-3 days)

**Tasks**:
- [ ] Create `ParentScope` struct with all current context fields
- [ ] Refactor `LaunchContext` to include `parent: Option<Arc<ParentScope>>`
- [ ] Move HashMaps to `local_*` versions (e.g., `local_configurations`)
- [ ] Implement `child()` method to create child contexts

**Files to modify**:
- `src/play_launch_parser/src/substitution/context.rs:26-286`: Major refactor

**Implementation**:
```rust
use std::sync::Arc;

// Frozen parent scope
struct ParentScope {
    configurations: HashMap<String, Vec<Substitution>>,
    environment: HashMap<String, String>,
    declared_arguments: HashMap<String, ArgumentMetadata>,
    global_parameters: HashMap<String, String>,
    remappings: Vec<(String, String)>,
    parent: Option<Arc<ParentScope>>,  // Chain to grandparent
}

// New context structure
pub struct LaunchContext {
    // Parent scope (shared, immutable via Arc)
    parent: Option<Arc<ParentScope>>,

    // Local scope (owned, mutable, initially empty for children)
    local_configurations: HashMap<String, Vec<Substitution>>,
    local_environment: HashMap<String, String>,
    local_declared_arguments: HashMap<String, ArgumentMetadata>,
    local_global_parameters: HashMap<String, String>,
    local_remappings: Vec<(String, String)>,

    // Always local
    current_file: Option<PathBuf>,
    namespace_stack: Vec<String>,
}

impl LaunchContext {
    /// Create child context - O(1) instead of O(n)!
    pub fn child(&self) -> Self {
        // Freeze current local scope and make it parent
        let parent = ParentScope {
            configurations: self.local_configurations.clone(),
            environment: self.local_environment.clone(),
            declared_arguments: self.local_declared_arguments.clone(),
            global_parameters: self.local_global_parameters.clone(),
            remappings: self.local_remappings.clone(),
            parent: self.parent.clone(),  // Arc clone - cheap!
        };

        Self {
            parent: Some(Arc::new(parent)),
            local_configurations: HashMap::new(),  // Empty local scope
            local_environment: HashMap::new(),
            local_declared_arguments: HashMap::new(),
            local_global_parameters: HashMap::new(),
            local_remappings: Vec::new(),
            current_file: None,
            namespace_stack: self.namespace_stack.clone(),  // Small vec
        }
    }
}
```

**Testing**:
- [ ] All context tests pass (18 tests in context.rs)
- [ ] Test scope shadowing (local overrides parent)
- [ ] Test parent visibility (child sees parent variables)
- [ ] Test grandparent chain (depth > 2)

---

### 7.2.2: Update Lookup Methods (1 day)

**Tasks**:
- [ ] Update `get_configuration()` to walk parent chain
- [ ] Update `get_environment_variable()` to walk parent chain
- [ ] Update `get_argument_metadata()` to walk parent chain
- [ ] Update `get_global_parameter()` to walk parent chain
- [ ] Add depth tracking for debugging (depth ~5-10 for Autoware)

**Files to modify**:
- `src/play_launch_parser/src/substitution/context.rs:94-215`: All `get_*` methods

**Implementation**:
```rust
impl LaunchContext {
    /// Lookup with local-first, then parent chain
    pub fn get_configuration(&self, name: &str) -> Option<String> {
        // 1. Check local scope (fast path - O(1))
        if let Some(subs) = self.local_configurations.get(name) {
            return resolve_substitutions(subs, self).ok();
        }

        // 2. Walk parent chain (depth ~5-10 for Autoware)
        let mut current = &self.parent;
        while let Some(parent) = current {
            if let Some(subs) = parent.configurations.get(name) {
                return resolve_substitutions(subs, self).ok();
            }
            current = &parent.parent;
        }

        None
    }

    // Similar for get_environment_variable, get_argument_metadata, etc.
}
```

**Testing**:
- [ ] Test local-first lookup (local shadows parent)
- [ ] Test parent fallback (not found locally)
- [ ] Test grandparent chain (depth > 2)
- [ ] Benchmark lookup performance (expect ~1.5x slower, negligible overall)

---

### 7.2.3: Update Mutation Methods and Include Logic (1.5 days)

**Tasks**:
- [ ] Update all `set_configuration()` to modify local scope only
- [ ] Update `set_environment_variable()` to modify local scope only
- [ ] Update `declare_argument()` to modify local scope only
- [ ] Update `set_global_parameter()` to modify local scope only
- [ ] Replace `context.clone()` with `context.child()` in lib.rs
- [ ] Update LaunchTraverser to use `Arc<LaunchContext>` (for future parallelization)

**Files to modify**:
- `src/play_launch_parser/src/substitution/context.rs:77-215`: All `set_*` methods
- `src/play_launch_parser/src/lib.rs:334, 444`: Replace `context.clone()` with `context.child()`

**Implementation**:
```rust
impl LaunchContext {
    /// Set always modifies local scope only
    pub fn set_configuration(&mut self, name: String, value: String) {
        match parse_substitutions(&value) {
            Ok(subs) => {
                self.local_configurations.insert(name, subs);  // Local only!
            }
            Err(_) => {
                self.local_configurations.insert(name, vec![Substitution::Text(value)]);
            }
        }
    }

    // Similar for other set_* methods
}

// In lib.rs
// Before (SLOW):
let mut include_context = self.context.clone();  // O(n) - expensive!

// After (FAST):
let mut include_context = self.context.child();  // O(1) - just Arc clone!
```

**Testing**:
- [ ] All integration tests pass (260 tests)
- [ ] Test include scoping (child args don't leak to parent)
- [ ] Benchmark Autoware (expect 20-40% improvement)
- [ ] Memory profiling (expect 60-80% reduction in context memory)

---

### Phase 7.2 Deliverables

**Expected Results**:
- ✅ Autoware parse time: ~2s (from ~3s) = 20-40% improvement
- ✅ Context memory: ~2-4MB (from ~10-20MB) = 60-80% reduction
- ✅ Child context creation: 500-1000x faster (O(1) vs O(n))
- ✅ All 260 tests pass
- ✅ **Enables**: Phase 7.3 parallel processing (Arc is thread-safe)

**Reference**: `tmp/context_cloning_best_practices.md`

---

## Phase 7.3: Parallel Processing with rayon (2-3 days) ⭐ FINAL MULTIPLIER

**Time**: 2-3 days
**Priority**: P1 (requires Phase 7.2)
**Risk**: Low (well-tested library)

### Overview

Process includes in parallel using rayon's work-stealing threadpool.

**Why rayon (NOT async, NOT manual task queue)**:
- ✅ Workload is 80%+ CPU-bound after Phase 7.1 caching
- ✅ rayon's work-stealing queue handles dynamic task spawning automatically
- ✅ No runtime overhead (unlike async/tokio)
- ✅ Simple: just change `.iter()` to `.par_iter()`

**Expected Impact**: 2.1x speedup on parallel portion (Autoware: ~2s → ~1s)

### 7.3.1: Enable rayon Parallelization (2-3 days)

**Tasks**:
- [ ] Add `rayon = "1.8"` dependency
- [ ] Refactor include processing to return `Vec<LaunchData>`
- [ ] Change `.iter()` to `.par_iter()` for include processing
- [ ] Update LaunchTraverser to use `Arc<LaunchContext>` (thread-safe)
- [ ] Implement result merging (preserve order if needed)
- [ ] Add circular include detection with `Arc<Mutex<HashSet<PathBuf>>>`

**Files to modify**:
- `Cargo.toml`: Add rayon dependency
- `src/play_launch_parser/src/lib.rs:369-483, 325-367`: Parallelize includes

**Implementation**:
```rust
use rayon::prelude::*;
use std::sync::Arc;

fn process_includes_parallel(
    includes: &[IncludeAction],
    context: Arc<LaunchContext>,
) -> Result<Vec<LaunchData>> {
    includes
        .par_iter()  // ← Parallel iterator (only change needed!)
        .map(|include_action| {
            // O(1) child creation from Phase 7.2
            let child_context = context.child();
            process_include(include_action, Arc::new(child_context))
        })
        .collect::<Result<Vec<_>>>()
}

// rayon's work-stealing queue handles:
// - Dynamic task spawning (includes spawn more includes)
// - Load balancing (idle threads steal from busy threads)
// - Automatic parallelism across all CPU cores
```

**Testing**:
- [ ] Verify deterministic output (same as sequential)
- [ ] Test circular include detection (thread-safe)
- [ ] Test with varying include depths (1-10 levels)
- [ ] Benchmark Autoware on 1, 2, 4, 8 cores
- [ ] All 260 tests pass in parallel mode

**Success Criteria**:
- Autoware parse time: <1s (from ~2s)
- Near-linear scaling up to 4-8 cores
- No race conditions or deadlocks

---

### Phase 7.3 Deliverables

**Expected Results**:
- ✅ Autoware parse time: ~1s (from ~2s) = 2.1x improvement
- ✅ Scales to available CPU cores (up to 8 cores tested)
- ✅ All 260 tests pass
- ✅ **Combined with Phase 7.1+7.2**: 5x total improvement (~5s → ~1s)

**Dependencies Added**:
```toml
[dependencies]
rayon = "1.8"             # Parallel processing
```

**Reference**: `tmp/parallelism_strategy_analysis.md`

---

## Phase 7.4: Additional Optimizations (1 week) - OPTIONAL

**Time**: 1 week
**Priority**: P2 (nice to have)
**Risk**: Low-Medium

### 7.4.1: Substitution Parsing Cache (1 day)

**Problem**: Repeated parsing of identical substitution strings.

**Solution**: thread_local LRU cache (bounded, eviction policy).

**Implementation**:
```rust
use lru::LruCache;
use std::cell::RefCell;

thread_local! {
    static PARSE_CACHE: RefCell<LruCache<String, Vec<Substitution>>> =
        RefCell::new(LruCache::new(1024));
}

pub fn parse_substitutions_cached(text: &str) -> Result<Vec<Substitution>> {
    PARSE_CACHE.with(|cache| {
        let mut cache = cache.borrow_mut();
        if let Some(result) = cache.get(text) {
            return Ok(result.clone());
        }
        let result = parse_substitutions(text)?;
        cache.put(text.to_string(), result.clone());
        Ok(result)
    })
}
```

**Expected Impact**: 40-60% for repeated substitution patterns

---

### 7.4.2: Command Execution Cache (0.5 days)

**Problem**: `$(command ...)` executes repeatedly for same commands.

**Solution**: DashMap cache with TTL (time-to-live).

**Implementation**:
```rust
use std::time::{Instant, Duration};

struct CachedCommand {
    output: String,
    timestamp: Instant,
}

static COMMAND_CACHE: Lazy<DashMap<String, CachedCommand>> =
    Lazy::new(DashMap::new);

const COMMAND_TTL: Duration = Duration::from_secs(5);

fn execute_command_cached(cmd: &str) -> Result<String> {
    if let Some(entry) = COMMAND_CACHE.get(cmd) {
        if entry.timestamp.elapsed() < COMMAND_TTL {
            return Ok(entry.output.clone());
        }
    }

    let output = execute_command_uncached(cmd)?;
    COMMAND_CACHE.insert(
        cmd.to_string(),
        CachedCommand { output: output.clone(), timestamp: Instant::now() }
    );

    Ok(output)
}
```

**Expected Impact**: 50-70% for command-heavy files

---

### 7.4.3: Record Generation Clone Elimination (1 day)

**Problem**: Unnecessary clones during record building.

**Solution**: Use `std::mem::take()` and entry API.

**Files to modify**:
- `src/play_launch_parser/src/record/generator.rs:42, 50, 80, 86, 95, 112`

**Expected Impact**: 15-25% reduction in string allocations

---

### 7.4.4: XML Iterator Returns (1 day)

**Problem**: `children()` allocates Vec every call.

**Solution**: Return iterator instead.

**Implementation**:
```rust
// Before:
pub fn children(&self) -> Vec<XmlEntity<'a, 'input>> {
    self.node.children()
        .filter(|n| n.is_element())
        .map(XmlEntity::new)
        .collect()  // Allocates Vec
}

// After:
pub fn children(&self) -> impl Iterator<Item = XmlEntity<'a, 'input>> {
    self.node.children()
        .filter(|n| n.is_element())
        .map(XmlEntity::new)
        // No collect - lazy evaluation
}
```

**Files to modify**:
- `src/play_launch_parser/src/xml/entity.rs:90-96`

**Expected Impact**: 20-30% reduction for large XML files

---

### Phase 7.4 Deliverables

**Expected Results**:
- ✅ Autoware parse time: ~0.7-0.8s (from ~1s) = additional 20-30%
- ✅ **Combined total**: 5-7x improvement (~5s → ~0.7-1s)
- ✅ All 260 tests pass

**Dependencies Added**:
```toml
[dependencies]
lru = "0.12"              # LRU cache
```

---

## Success Criteria

### Performance Targets

| Phase     | Autoware Time | Cache Hit Rate          | Memory | Tests    |
|-----------|---------------|-------------------------|--------|----------|
| Baseline  | ~5s           | N/A                     | ~50MB  | 260 pass |
| Phase 7.1 | <3s           | >95% package, >70% file | <60MB  | 260 pass |
| Phase 7.2 | <2s           | Same                    | <45MB  | 260 pass |
| Phase 7.3 | <1s           | Same                    | <50MB  | 260 pass |
| Phase 7.4 | <0.8s         | Same                    | <40MB  | 260 pass |

### Quality Gates

Each phase must meet:
- ✅ All 260 tests pass
- ✅ No clippy warnings (`cargo clippy --all-targets --all-features`)
- ✅ Code formatted (`cargo fmt`)
- ✅ Memory profiling shows no leaks
- ✅ Benchmark results documented

---

## Benchmarking Strategy

### Tools

```bash
# Performance benchmarking
hyperfine 'play_launch_parser launch autoware_launch planning_simulator.launch.xml'

# Memory profiling
heaptrack play_launch_parser launch autoware_launch planning_simulator.launch.xml

# CPU profiling
cargo flamegraph --bin play_launch_parser -- launch autoware_launch planning_simulator.launch.xml

# Cache statistics
RUST_LOG=trace play_launch_parser launch ... | grep "cache hit"
```

### Baseline Measurements

Before starting Phase 7.1, capture:
- [ ] Autoware parse time (average of 5 runs)
- [ ] Memory peak usage
- [ ] Flamegraph showing hotspots
- [ ] Disk I/O statistics

### Per-Phase Validation

After each phase:
- [ ] Run benchmarks (5 runs, report median)
- [ ] Compare against baseline
- [ ] Capture cache hit rates
- [ ] Update flamegraph
- [ ] Document improvement in this file

---

## Risk Mitigation

### Medium Risk Items

**Phase 7.2 (Hybrid Context)**:
- Risk: Incorrect scope chain implementation
- Mitigation: Excellent test coverage (260 tests), incremental implementation, well-understood pattern

### Rollback Plan

Each phase is independent:
- Phase 7.1: Can disable caching via feature flag
- Phase 7.2: Can revert context refactor (git branch)
- Phase 7.3: Can disable rayon with feature flag

---

## Dependencies Summary

```toml
[dependencies]
dashmap = "5.5"           # Phase 7.1: Concurrent HashMap
parking_lot = "0.12"      # Phase 7.1: Faster mutexes
once_cell = "1.19"        # Phase 7.1: Lazy static (already present)
rayon = "1.8"             # Phase 7.3: Parallel processing
lru = "0.12"              # Phase 7.4: LRU cache (optional)
```

---

## Reference Documents

Detailed analysis and implementation guides:

1. **`tmp/optimization_opportunities.md`** - Complete optimization analysis
2. **`tmp/cache_strategy_dashmap.md`** - DashMap vs LRU comparison
3. **`tmp/context_cloning_best_practices.md`** - Compiler pattern details
4. **`tmp/package_resolution_context_analysis.md`** - Context awareness analysis
5. **`tmp/parallelism_strategy_analysis.md`** - Async vs rayon vs task queue comparison

---

## Timeline Summary

| Phase                | Time          | Dependency      | Risk   | Impact   |
|----------------------|---------------|-----------------|--------|----------|
| 7.1: DashMap Caching | 2-3 days      | None            | Low    | 60-80%   |
| 7.2: Hybrid Context  | 1 week        | 7.1 recommended | Medium | 20-40%   |
| 7.3: rayon Parallel  | 2-3 days      | 7.2 required    | Low    | 2.1x     |
| 7.4: Additional Opts | 1 week        | 7.1-7.3         | Low    | +10-20%  |
| **Total**            | **3-4 weeks** | -               | -      | **5-7x** |

**Recommended approach**: Sequential implementation (7.1 → 7.2 → 7.3 → 7.4)

---

## Completion Checklist

### Phase 7.1 Complete When:
- [ ] Package cache implemented with >95% hit rate
- [ ] File cache implemented with >70% hit rate
- [ ] parking_lot::Mutex integrated
- [ ] Autoware parse time <3s
- [ ] All 260 tests pass
- [ ] Benchmarks documented

### Phase 7.2 Complete When:
- [ ] Hybrid Arc + Local context implemented
- [ ] All lookup methods walk parent chain
- [ ] All mutation methods modify local scope
- [ ] Context memory reduced by 60-80%
- [ ] Autoware parse time <2s
- [ ] All 260 tests pass
- [ ] Scope shadowing tests added

### Phase 7.3 Complete When:
- [ ] rayon parallelization implemented
- [ ] Deterministic output verified
- [ ] Circular include detection works
- [ ] Autoware parse time <1s
- [ ] All 260 tests pass
- [ ] Scaling measured (1, 2, 4, 8 cores)

### Phase 7.4 Complete When:
- [ ] Additional optimizations implemented
- [ ] Autoware parse time <0.8s
- [ ] All 260 tests pass
- [ ] Final benchmarks documented

---

**Status Updates**: Track progress in `docs/roadmap/implementation_status.md`
