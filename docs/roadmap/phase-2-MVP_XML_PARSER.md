# Phase 2: MVP XML Parser

**Status**: 📋 Not Started
**Priority**: Critical (Foundation)
**Estimated Effort**: 2-3 weeks
**Dependencies**: Phase 1 (Complete ✅)

---

## Overview

Implement a Minimum Viable Parser (MVP) capable of parsing simple XML launch files with basic nodes and substitutions. This phase establishes the core architecture and proves the concept with real ROS 2 launch files.

**Goal:** Parse `talker_listener.launch.xml` from ros2/demos and generate valid `record.json`

---

## Success Criteria

- [ ] Parse simple `<node>` elements from XML
- [ ] Handle `<arg>` declarations
- [ ] Support basic substitutions (`$(var ...)`, `$(env ...)`)
- [ ] Generate complete `NodeRecord` structures
- [ ] Output valid `record.json` compatible with play_launch
- [ ] Pass comparison test with dump_launch output
- [ ] Parse time: <100ms for simple launch files

---

## Work Items

### Task 2.1: Rust Package Setup ✅ Priority: Critical

**Objective**: Create the play_launch_parser ROS 2 package structure

**Subtasks**:
- [ ] 2.1.1: Create `src/play_launch_parser/` directory
- [ ] 2.1.2: Create `Cargo.toml` with dependencies
- [ ] 2.1.3: Create `package.xml` for ROS 2 integration
- [ ] 2.1.4: Set up module structure (lib.rs, main.rs)
- [ ] 2.1.5: Add initial dependencies (serde, serde_json, roxmltree)
- [ ] 2.1.6: Configure clap for CLI
- [ ] 2.1.7: Set up error types with thiserror
- [ ] 2.1.8: Configure env_logger for logging

**Dependencies**:
```toml
[dependencies]
serde = { version = "1.0", features = ["derive"] }
serde_json = "1.0"
roxmltree = "0.19"
thiserror = "1.0"
anyhow = "1.0"
clap = { version = "4.0", features = ["derive"] }
env_logger = "0.11"
log = "0.4"
```

**Module Structure**:
```
src/play_launch_parser/
├── Cargo.toml
├── package.xml
└── src/
    ├── main.rs              # CLI entry point
    ├── lib.rs               # Library exports
    ├── error.rs             # Error types
    ├── xml/
    │   ├── mod.rs           # XML parser module
    │   ├── parser.rs        # XML parsing logic
    │   └── entity.rs        # XML entity abstraction
    ├── substitution/
    │   ├── mod.rs           # Substitution module
    │   ├── parser.rs        # Substitution grammar parser
    │   ├── types.rs         # Substitution types
    │   └── context.rs       # LaunchContext
    ├── actions/
    │   ├── mod.rs           # Action types
    │   ├── node.rs          # Node action
    │   └── arg.rs           # Arg action
    └── record/
        ├── mod.rs           # Record generation
        ├── types.rs         # NodeRecord, etc.
        └── generator.rs     # record.json generator
```

**Files to Create**:
- `src/play_launch_parser/Cargo.toml`
- `src/play_launch_parser/package.xml`
- `src/play_launch_parser/src/main.rs`
- `src/play_launch_parser/src/lib.rs`
- `src/play_launch_parser/src/error.rs`

**Acceptance Criteria**:
- [ ] Package builds with `colcon build`
- [ ] CLI runs with `--help` flag
- [ ] All modules compile without errors
- [ ] Can run via `just build`

**Estimated Time**: 2 days

---

### Task 2.2: XML Parsing Foundation 🔧 Priority: Critical

**Objective**: Parse XML launch files into a tree structure

**Subtasks**:
- [ ] 2.2.1: Implement XML file loading with roxmltree
- [ ] 2.2.2: Create Entity trait abstraction
- [ ] 2.2.3: Implement XmlEntity wrapping roxmltree::Node
- [ ] 2.2.4: Implement attribute extraction with type support
- [ ] 2.2.5: Implement child element iteration
- [ ] 2.2.6: Add error reporting with line numbers
- [ ] 2.2.7: Write unit tests for entity abstraction

**Entity Trait Design**:
```rust
pub trait Entity {
    /// Get entity type name (e.g., "node", "arg")
    fn type_name(&self) -> &str;

    /// Get attribute value with optional type coercion
    fn get_attr<T: FromStr>(
        &self,
        name: &str,
        optional: bool,
    ) -> Result<Option<T>, ParseError>;

    /// Get all attributes as key-value pairs
    fn attributes(&self) -> Vec<(&str, &str)>;

    /// Get child entities
    fn children(&self) -> Vec<Box<dyn Entity>>;

    /// Get text content
    fn text(&self) -> Option<&str>;
}
```

**XmlEntity Implementation**:
```rust
pub struct XmlEntity<'a> {
    node: roxmltree::Node<'a, 'a>,
}

impl<'a> Entity for XmlEntity<'a> {
    fn type_name(&self) -> &str {
        self.node.tag_name().name()
    }

    fn get_attr<T: FromStr>(&self, name: &str, optional: bool) -> Result<Option<T>, ParseError> {
        match self.node.attribute(name) {
            Some(value) => {
                let parsed = value.parse::<T>()
                    .map_err(|_| ParseError::TypeCoercion {
                        attribute: name.to_string(),
                        value: value.to_string(),
                        expected_type: std::any::type_name::<T>(),
                    })?;
                Ok(Some(parsed))
            }
            None if optional => Ok(None),
            None => Err(ParseError::MissingAttribute {
                element: self.type_name().to_string(),
                attribute: name.to_string(),
            }),
        }
    }

    // ... other methods
}
```

**Files to Create**:
- `src/play_launch_parser/src/xml/mod.rs`
- `src/play_launch_parser/src/xml/parser.rs`
- `src/play_launch_parser/src/xml/entity.rs`

**Tests to Write**:
- Parse simple XML element
- Extract attributes (string, bool, int, float)
- Handle missing required attributes (error)
- Handle missing optional attributes (None)
- Iterate child elements
- Type coercion errors

**Acceptance Criteria**:
- [ ] Can load XML file with roxmltree
- [ ] Entity trait implemented
- [ ] XmlEntity wraps roxmltree::Node
- [ ] Attribute extraction with type coercion works
- [ ] Child iteration works
- [ ] Unit tests pass (90%+ coverage)

**Estimated Time**: 3 days

---

### Task 2.3: Substitution Engine (Core) 🔄 Priority: Critical

**Objective**: Parse and resolve basic substitutions

**Subtasks**:
- [ ] 2.3.1: Design Substitution enum
- [ ] 2.3.2: Implement substitution grammar parser (regex-based for MVP)
- [ ] 2.3.3: Implement LaunchContext structure
- [ ] 2.3.4: Implement `$(var name)` - LaunchConfiguration
- [ ] 2.3.5: Implement `$(env VAR)` - Environment variable
- [ ] 2.3.6: Implement `$(env VAR default)` - Env with default
- [ ] 2.3.7: Implement text substitution (literal strings)
- [ ] 2.3.8: Implement substitution resolution with context
- [ ] 2.3.9: Write unit tests for all substitution types

**Substitution Design**:
```rust
#[derive(Debug, Clone, PartialEq)]
pub enum Substitution {
    Text(String),
    LaunchConfiguration(String),  // $(var name)
    EnvironmentVariable { name: String, default: Option<String> },  // $(env VAR [default])
    FindPackageShare(String),  // $(find-pkg-share pkg) - Deferred to later
}

impl Substitution {
    /// Parse substitution string like "$(var x)" or "text $(env Y) more"
    pub fn parse(input: &str) -> Result<Vec<Substitution>, ParseError> {
        // Regex-based parser for MVP
        // Pattern: \$\(([a-z-]+)\s+([^)]+)\)
    }

    /// Resolve substitution to string value
    pub fn resolve(&self, context: &LaunchContext) -> Result<String, SubstitutionError> {
        match self {
            Substitution::Text(s) => Ok(s.clone()),
            Substitution::LaunchConfiguration(name) => {
                context.get_configuration(name)
                    .ok_or_else(|| SubstitutionError::UndefinedVariable(name.clone()))
            }
            Substitution::EnvironmentVariable { name, default } => {
                std::env::var(name).or_else(|_| {
                    default.clone()
                        .ok_or_else(|| SubstitutionError::UndefinedEnvVar(name.clone()))
                })
            }
            // ...
        }
    }
}

/// Resolve list of substitutions to single string
pub fn resolve_substitutions(
    subs: &[Substitution],
    context: &LaunchContext,
) -> Result<String, SubstitutionError> {
    let mut result = String::new();
    for sub in subs {
        result.push_str(&sub.resolve(context)?);
    }
    Ok(result)
}
```

**LaunchContext Design**:
```rust
pub struct LaunchContext {
    configurations: HashMap<String, String>,
    // More fields added in later phases
}

impl LaunchContext {
    pub fn new() -> Self {
        Self {
            configurations: HashMap::new(),
        }
    }

    pub fn set_configuration(&mut self, name: String, value: String) {
        self.configurations.insert(name, value);
    }

    pub fn get_configuration(&self, name: &str) -> Option<String> {
        self.configurations.get(name).cloned()
    }
}
```

**Files to Create**:
- `src/play_launch_parser/src/substitution/mod.rs`
- `src/play_launch_parser/src/substitution/parser.rs`
- `src/play_launch_parser/src/substitution/types.rs`
- `src/play_launch_parser/src/substitution/context.rs`

**Tests to Write**:
- Parse literal text (no substitutions)
- Parse `$(var name)` substitution
- Parse `$(env VAR)` substitution
- Parse `$(env VAR default)` with default
- Parse mixed text and substitutions
- Resolve substitutions with context
- Error on undefined variable
- Error on undefined env var (no default)

**Acceptance Criteria**:
- [ ] Substitution enum defined
- [ ] Parser handles $(var ...) and $(env ...)
- [ ] LaunchContext stores configurations
- [ ] Substitution resolution works
- [ ] Unit tests pass (100% coverage)

**Estimated Time**: 4 days

---

### Task 2.4: Node Action Parsing 🎯 Priority: Critical

**Objective**: Parse `<node>` elements and extract metadata

**Subtasks**:
- [ ] 2.4.1: Define NodeAction struct
- [ ] 2.4.2: Implement node attribute extraction (pkg, exec, name, namespace)
- [ ] 2.4.3: Handle optional attributes (name defaults to exec)
- [ ] 2.4.4: Parse inline `<param>` elements
- [ ] 2.4.5: Parse `<remap>` elements
- [ ] 2.4.6: Parse `<env>` elements
- [ ] 2.4.7: Resolve substitutions in attributes
- [ ] 2.4.8: Write unit tests for node parsing

**NodeAction Design**:
```rust
pub struct NodeAction {
    pub package: Vec<Substitution>,
    pub executable: Vec<Substitution>,
    pub name: Option<Vec<Substitution>>,
    pub namespace: Option<Vec<Substitution>>,
    pub parameters: Vec<Parameter>,
    pub remappings: Vec<Remapping>,
    pub environment: Vec<(String, String)>,
    pub output: Option<String>,  // "screen", "log", "both"
    pub respawn: bool,
    pub respawn_delay: Option<f64>,
}

impl NodeAction {
    pub fn from_entity(entity: &dyn Entity) -> Result<Self, ParseError> {
        let package = entity.get_attr::<String>("pkg", false)?
            .map(|s| Substitution::parse(&s))??;
        let executable = entity.get_attr::<String>("exec", false)?
            .map(|s| Substitution::parse(&s))??;
        let name = entity.get_attr::<String>("name", true)?
            .map(|s| Substitution::parse(&s)).transpose()?;
        let namespace = entity.get_attr::<String>("namespace", true)?
            .map(|s| Substitution::parse(&s)).transpose()?;

        // Parse children for params, remaps, env
        let mut parameters = Vec::new();
        let mut remappings = Vec::new();
        let mut environment = Vec::new();

        for child in entity.children() {
            match child.type_name() {
                "param" => parameters.push(Parameter::from_entity(&*child)?),
                "remap" => remappings.push(Remapping::from_entity(&*child)?),
                "env" => environment.push(parse_env(&*child)?),
                other => return Err(ParseError::UnexpectedElement {
                    parent: "node".to_string(),
                    child: other.to_string(),
                }),
            }
        }

        Ok(Self {
            package,
            executable,
            name,
            namespace,
            parameters,
            remappings,
            environment,
            output: entity.get_attr("output", true)?,
            respawn: entity.get_attr("respawn", true)?.unwrap_or(false),
            respawn_delay: entity.get_attr("respawn_delay", true)?,
        })
    }
}

#[derive(Debug, Clone)]
pub struct Parameter {
    pub name: String,
    pub value: String,
}

impl Parameter {
    pub fn from_entity(entity: &dyn Entity) -> Result<Self, ParseError> {
        Ok(Self {
            name: entity.get_attr("name", false)?.unwrap(),
            value: entity.get_attr("value", false)?.unwrap(),
        })
    }
}

#[derive(Debug, Clone)]
pub struct Remapping {
    pub from: String,
    pub to: String,
}

impl Remapping {
    pub fn from_entity(entity: &dyn Entity) -> Result<Self, ParseError> {
        Ok(Self {
            from: entity.get_attr("from", false)?.unwrap(),
            to: entity.get_attr("to", false)?.unwrap(),
        })
    }
}
```

**Files to Create**:
- `src/play_launch_parser/src/actions/mod.rs`
- `src/play_launch_parser/src/actions/node.rs`

**Tests to Write**:
- Parse node with required attributes
- Parse node with optional name (defaults to exec)
- Parse node with namespace
- Parse node with inline params
- Parse node with remappings
- Parse node with environment vars
- Parse node with substitutions
- Error on missing required attributes

**Acceptance Criteria**:
- [ ] NodeAction struct defined
- [ ] Can parse all node attributes
- [ ] Handles optional attributes correctly
- [ ] Parses child elements (param, remap, env)
- [ ] Substitutions in attributes work
- [ ] Unit tests pass (90%+ coverage)

**Estimated Time**: 3 days

---

### Task 2.5: Arg Action Parsing 📋 Priority: Critical

**Objective**: Parse `<arg>` declarations for launch arguments

**Subtasks**:
- [ ] 2.5.1: Define ArgAction struct
- [ ] 2.5.2: Parse arg attributes (name, default, description)
- [ ] 2.5.3: Update LaunchContext when arg is declared
- [ ] 2.5.4: Handle command-line argument overrides
- [ ] 2.5.5: Write unit tests for arg parsing

**ArgAction Design**:
```rust
pub struct ArgAction {
    pub name: String,
    pub default: Option<String>,
    pub description: Option<String>,
}

impl ArgAction {
    pub fn from_entity(entity: &dyn Entity) -> Result<Self, ParseError> {
        Ok(Self {
            name: entity.get_attr("name", false)?.unwrap(),
            default: entity.get_attr("default", true)?,
            description: entity.get_attr("description", true)?,
        })
    }

    pub fn apply(&self, context: &mut LaunchContext, cli_args: &HashMap<String, String>) {
        let value = cli_args.get(&self.name)
            .cloned()
            .or_else(|| self.default.clone());

        if let Some(v) = value {
            context.set_configuration(self.name.clone(), v);
        }
    }
}
```

**Files to Create**:
- `src/play_launch_parser/src/actions/arg.rs`

**Tests to Write**:
- Parse arg with name only
- Parse arg with default
- Parse arg with description
- Apply arg to context (default value)
- Apply arg to context (CLI override)

**Acceptance Criteria**:
- [ ] ArgAction struct defined
- [ ] Can parse arg attributes
- [ ] Updates LaunchContext correctly
- [ ] CLI overrides work
- [ ] Unit tests pass (100% coverage)

**Estimated Time**: 2 days

---

### Task 2.6: record.json Data Structures 📊 Priority: Critical

**Objective**: Define data structures for record.json output

**Subtasks**:
- [ ] 2.6.1: Define NodeRecord struct with serde
- [ ] 2.6.2: Define RecordJson root struct
- [ ] 2.6.3: Implement serialization with field name mapping
- [ ] 2.6.4: Handle Option fields (null in JSON)
- [ ] 2.6.5: Handle tuple serialization (params, remaps, env)
- [ ] 2.6.6: Write serialization tests

**NodeRecord Design**:
```rust
use serde::{Serialize, Deserialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeRecord {
    pub executable: String,
    pub package: Option<String>,
    pub name: Option<String>,
    pub namespace: Option<String>,
    pub exec_name: Option<String>,
    #[serde(serialize_with = "serialize_tuple_vec")]
    pub params: Vec<(String, String)>,
    pub params_files: Vec<String>,
    #[serde(serialize_with = "serialize_tuple_vec")]
    pub remaps: Vec<(String, String)>,
    pub ros_args: Option<Vec<String>>,
    pub args: Option<Vec<String>>,
    pub cmd: Vec<String>,
    #[serde(serialize_with = "serialize_option_tuple_vec")]
    pub env: Option<Vec<(String, String)>>,
    pub respawn: Option<bool>,
    pub respawn_delay: Option<f64>,
    #[serde(serialize_with = "serialize_option_tuple_vec")]
    pub global_params: Option<Vec<(String, String)>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecordJson {
    pub node: Vec<NodeRecord>,
    pub container: Vec<ComposableNodeContainerRecord>,
    pub load_node: Vec<LoadNodeRecord>,
    pub lifecycle_node: Vec<String>,
    pub file_data: HashMap<String, String>,
}

impl RecordJson {
    pub fn new() -> Self {
        Self {
            node: Vec::new(),
            container: Vec::new(),
            load_node: Vec::new(),
            lifecycle_node: Vec::new(),
            file_data: HashMap::new(),
        }
    }

    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }
}
```

**Files to Create**:
- `src/play_launch_parser/src/record/mod.rs`
- `src/play_launch_parser/src/record/types.rs`

**Tests to Write**:
- Serialize empty RecordJson
- Serialize NodeRecord with all fields
- Serialize NodeRecord with optional fields (null)
- Tuple serialization format
- Deserialize and re-serialize (round-trip)

**Acceptance Criteria**:
- [ ] NodeRecord struct defined with serde
- [ ] RecordJson root struct defined
- [ ] Serialization produces correct JSON format
- [ ] Null handling works
- [ ] Tuple serialization works
- [ ] Unit tests pass (100% coverage)

**Estimated Time**: 2 days

---

### Task 2.7: Command-Line Generation 🖥️ Priority: Critical

**Objective**: Generate ROS 2 command-line arrays from node metadata

**Subtasks**:
- [ ] 2.7.1: Implement package path resolution (ament_index)
- [ ] 2.7.2: Build executable path from package + executable
- [ ] 2.7.3: Format node name argument (-r __node:=name)
- [ ] 2.7.4: Format namespace argument (-r __ns:=/namespace)
- [ ] 2.7.5: Format parameter arguments (-p name:=value)
- [ ] 2.7.6: Format remapping arguments (-r from:=to)
- [ ] 2.7.7: Assemble complete cmd array
- [ ] 2.7.8: Write unit tests for command generation

**Command Generator Design**:
```rust
pub struct CommandGenerator;

impl CommandGenerator {
    pub fn generate_node_command(
        node: &NodeAction,
        context: &LaunchContext,
    ) -> Result<Vec<String>, GenerationError> {
        let mut cmd = Vec::new();

        // 1. Resolve executable path
        let package = resolve_substitutions(&node.package, context)?;
        let executable = resolve_substitutions(&node.executable, context)?;
        let exec_path = Self::resolve_executable_path(&package, &executable)?;
        cmd.push(exec_path);

        // 2. ROS args delimiter
        cmd.push("--ros-args".to_string());

        // 3. Node name
        let node_name = if let Some(name_subs) = &node.name {
            resolve_substitutions(name_subs, context)?
        } else {
            executable.clone()
        };
        cmd.push("-r".to_string());
        cmd.push(format!("__node:={}", node_name));

        // 4. Namespace
        let namespace = if let Some(ns_subs) = &node.namespace {
            resolve_substitutions(ns_subs, context)?
        } else {
            "/".to_string()
        };
        cmd.push("-r".to_string());
        cmd.push(format!("__ns:={}", namespace));

        // 5. Remappings
        for remap in &node.remappings {
            cmd.push("-r".to_string());
            cmd.push(format!("{}:={}", remap.from, remap.to));
        }

        // 6. Parameters
        for param in &node.parameters {
            cmd.push("-p".to_string());
            cmd.push(format!("{}:={}", param.name, param.value));
        }

        Ok(cmd)
    }

    fn resolve_executable_path(package: &str, executable: &str) -> Result<String, GenerationError> {
        // Use ament_index to find package path
        // For MVP: simplified implementation
        let package_path = format!("/opt/ros/humble/lib/{}", package);
        Ok(format!("{}/{}", package_path, executable))
    }
}
```

**Files to Create**:
- `src/play_launch_parser/src/record/generator.rs`

**Tests to Write**:
- Generate command for simple node
- Generate command with custom name
- Generate command with namespace
- Generate command with parameters
- Generate command with remappings
- Full command array format

**Acceptance Criteria**:
- [ ] Package path resolution works (simplified for MVP)
- [ ] Command array format matches ROS 2 spec
- [ ] All arguments included correctly
- [ ] Order matches dump_launch output
- [ ] Unit tests pass (90%+ coverage)

**Estimated Time**: 3 days

---

### Task 2.8: Launch Tree Traversal 🌳 Priority: Critical

**Objective**: Traverse launch XML and execute actions

**Subtasks**:
- [ ] 2.8.1: Implement recursive entity visitor
- [ ] 2.8.2: Dispatch entities to action parsers
- [ ] 2.8.3: Accumulate NodeRecords during traversal
- [ ] 2.8.4: Handle unknown action types (error or skip)
- [ ] 2.8.5: Write integration tests for traversal

**Traversal Design**:
```rust
pub struct LaunchTraverser {
    context: LaunchContext,
    records: Vec<NodeRecord>,
}

impl LaunchTraverser {
    pub fn new(cli_args: HashMap<String, String>) -> Self {
        let mut context = LaunchContext::new();
        // Apply CLI args as initial configurations
        for (k, v) in cli_args {
            context.set_configuration(k, v);
        }

        Self {
            context,
            records: Vec::new(),
        }
    }

    pub fn traverse(&mut self, entity: &dyn Entity) -> Result<(), TraversalError> {
        match entity.type_name() {
            "launch" => {
                // Root element, traverse children
                for child in entity.children() {
                    self.traverse(&*child)?;
                }
            }
            "arg" => {
                let arg = ArgAction::from_entity(entity)?;
                arg.apply(&mut self.context, &HashMap::new());
            }
            "node" => {
                let node = NodeAction::from_entity(entity)?;
                let record = self.generate_node_record(&node)?;
                self.records.push(record);
            }
            other => {
                log::warn!("Unsupported action type: {}", other);
                // For MVP: skip unknown actions
            }
        }
        Ok(())
    }

    fn generate_node_record(&self, node: &NodeAction) -> Result<NodeRecord, GenerationError> {
        let cmd = CommandGenerator::generate_node_command(node, &self.context)?;

        let package = resolve_substitutions(&node.package, &self.context)?;
        let executable = resolve_substitutions(&node.executable, &self.context)?;

        let name = if let Some(name_subs) = &node.name {
            Some(resolve_substitutions(name_subs, &self.context)?)
        } else {
            Some(executable.clone())
        };

        let namespace = if let Some(ns_subs) = &node.namespace {
            Some(resolve_substitutions(ns_subs, &self.context)?)
        } else {
            Some("/".to_string())
        };

        let params = node.parameters.iter()
            .map(|p| (p.name.clone(), p.value.clone()))
            .collect();

        let remaps = node.remappings.iter()
            .map(|r| (r.from.clone(), r.to.clone()))
            .collect();

        Ok(NodeRecord {
            executable,
            package: Some(package),
            name,
            namespace,
            exec_name: None,  // TODO: Generate exec_name-1, exec_name-2, etc.
            params,
            params_files: Vec::new(),  // TODO: Phase 3
            remaps,
            ros_args: None,
            args: None,
            cmd,
            env: if node.environment.is_empty() { None } else { Some(node.environment.clone()) },
            respawn: Some(node.respawn),
            respawn_delay: node.respawn_delay,
            global_params: None,  // TODO: Phase 3
        })
    }

    pub fn into_record_json(self) -> RecordJson {
        RecordJson {
            node: self.records,
            container: Vec::new(),
            load_node: Vec::new(),
            lifecycle_node: Vec::new(),
            file_data: HashMap::new(),
        }
    }
}
```

**Files to Create**:
- `src/play_launch_parser/src/lib.rs` (public API)

**Tests to Write**:
- Traverse simple launch with one node
- Traverse launch with multiple nodes
- Traverse launch with args
- Traverse with substitutions
- Error on invalid XML

**Acceptance Criteria**:
- [ ] Can traverse launch root
- [ ] Processes args correctly
- [ ] Processes nodes correctly
- [ ] Generates NodeRecords
- [ ] Integration tests pass

**Estimated Time**: 3 days

---

### Task 2.9: CLI Implementation 💻 Priority: High

**Objective**: Implement command-line interface for parser

**Subtasks**:
- [ ] 2.9.1: Define CLI structure with clap
- [ ] 2.9.2: Implement `parse launch <pkg> <file>` command
- [ ] 2.9.3: Implement `parse file <path>` command
- [ ] 2.9.4: Add `--output <path>` option
- [ ] 2.9.5: Add launch arguments (`key:=value`)
- [ ] 2.9.6: Add `--verbose` and `--quiet` flags
- [ ] 2.9.7: Integrate with logging
- [ ] 2.9.8: Write CLI tests

**CLI Design**:
```rust
use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(name = "play_launch_parser")]
#[command(about = "High-performance ROS 2 launch file parser", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,

    #[arg(short, long)]
    verbose: bool,

    #[arg(short, long)]
    quiet: bool,
}

#[derive(Subcommand)]
enum Commands {
    /// Parse a launch file from a ROS 2 package
    Launch {
        /// Package name
        package: String,

        /// Launch file name
        file: String,

        /// Launch arguments (key:=value)
        #[arg(value_parser = parse_launch_arg)]
        args: Vec<(String, String)>,

        /// Output file path (default: record.json)
        #[arg(short, long, default_value = "record.json")]
        output: String,
    },

    /// Parse a launch file from a direct file path
    File {
        /// Launch file path
        path: String,

        /// Launch arguments (key:=value)
        #[arg(value_parser = parse_launch_arg)]
        args: Vec<(String, String)>,

        /// Output file path (default: record.json)
        #[arg(short, long, default_value = "record.json")]
        output: String,
    },
}

fn parse_launch_arg(s: &str) -> Result<(String, String), String> {
    let parts: Vec<&str> = s.split(":=").collect();
    if parts.len() != 2 {
        return Err(format!("Invalid launch argument format: {}", s));
    }
    Ok((parts[0].to_string(), parts[1].to_string()))
}
```

**Files to Modify**:
- `src/play_launch_parser/src/main.rs`

**Tests to Write**:
- Parse launch command
- Parse file command
- Launch arguments parsing
- Output path option
- Verbose/quiet flags

**Acceptance Criteria**:
- [ ] CLI parses all commands
- [ ] Help text displays correctly
- [ ] Launch arguments work
- [ ] Output to custom path works
- [ ] Logging levels work

**Estimated Time**: 2 days

---

### Task 2.10: Integration & Testing ✅ Priority: Critical

**Objective**: End-to-end testing with real launch files

**Subtasks**:
- [ ] 2.10.1: Download ros2/demos to external/
- [ ] 2.10.2: Test with talker_listener.launch.xml
- [ ] 2.10.3: Generate record.json with parser
- [ ] 2.10.4: Generate record.json with dump_launch
- [ ] 2.10.5: Compare outputs (field-by-field)
- [ ] 2.10.6: Fix any discrepancies
- [ ] 2.10.7: Add integration test to CI
- [ ] 2.10.8: Benchmark parse time

**Test Launch File**:
```xml
<!-- external/demos/demo_nodes_cpp/launch/topics/talker_listener.launch.xml -->
<launch>
  <node pkg="demo_nodes_cpp" exec="talker" output="screen" />
  <node pkg="demo_nodes_cpp" exec="listener" output="screen" />
</launch>
```

**Comparison Script**:
```bash
#!/bin/bash
set -e

# Generate with dump_launch
source /opt/ros/humble/setup.bash
dump_launch launch demo_nodes_cpp talker_listener.launch.xml
mv record.json record_python.json

# Generate with Rust parser
source install/setup.bash
play_launch_parser launch demo_nodes_cpp talker_listener.launch.xml
mv record.json record_rust.json

# Compare
echo "Comparing outputs..."
diff -u <(jq --sort-keys . record_python.json) <(jq --sort-keys . record_rust.json)

if [ $? -eq 0 ]; then
    echo "✅ Outputs match!"
else
    echo "❌ Outputs differ!"
    exit 1
fi
```

**Files to Create**:
- `test/compare_outputs.sh`
- `test/integration_test.rs`

**Acceptance Criteria**:
- [ ] Parser handles talker_listener.launch.xml
- [ ] Generated record.json is valid
- [ ] Outputs match dump_launch (semantic equivalence)
- [ ] Parse time <100ms
- [ ] Integration test passes in CI

**Estimated Time**: 3 days

---

## Testing Strategy

### Unit Tests

**Coverage Target**: 90%+

**Test Files**:
- `src/xml/entity_test.rs` - Entity abstraction
- `src/substitution/parser_test.rs` - Substitution parsing
- `src/substitution/context_test.rs` - Context management
- `src/actions/node_test.rs` - Node parsing
- `src/actions/arg_test.rs` - Arg parsing
- `src/record/types_test.rs` - Serialization

### Integration Tests

**Test Cases**:
1. Simple node launch (talker_listener.launch.xml)
2. Node with substitutions
3. Node with parameters
4. Node with remappings
5. Launch with args

### Comparison Tests

**Comparison Points**:
- Node count matches
- Package names match
- Executable names match
- Command arrays match (order-independent for some args)
- Parameters match
- Remappings match

### Performance Tests

**Benchmarks**:
- Parse talker_listener.launch.xml: <100ms
- Memory usage: <50MB for simple launches

---

## Risk Assessment

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| roxmltree API complexity | Medium | Medium | Read docs thoroughly, write wrapper layer |
| Substitution grammar too complex | Medium | High | Start with regex, upgrade to pest if needed |
| record.json format mismatch | Medium | High | Test extensively with dump_launch comparison |
| Package path resolution | Medium | Medium | Use simplified approach for MVP, improve later |
| Type coercion edge cases | Low | Medium | Comprehensive unit tests |

---

## Dependencies

**Rust Crates**:
- `roxmltree` (0.19) - XML parsing
- `serde` (1.0) - Serialization
- `serde_json` (1.0) - JSON output
- `thiserror` (1.0) - Error types
- `anyhow` (1.0) - Error handling
- `clap` (4.0) - CLI parsing
- `env_logger` (0.11) - Logging
- `log` (0.4) - Logging facade

**ROS 2 Dependencies** (for package resolution):
- `ament_index` - Package finding (via Python or FFI)
- Alternative: Use environment variables and paths for MVP

---

## Deliverables

**Code**:
- [ ] Rust package in `src/play_launch_parser/`
- [ ] XML parser with entity abstraction
- [ ] Substitution engine (var, env)
- [ ] Node and arg action parsers
- [ ] record.json data structures
- [ ] Command-line generator
- [ ] CLI interface
- [ ] Unit tests (90%+ coverage)
- [ ] Integration tests

**Documentation**:
- [ ] README for package
- [ ] API documentation (rustdoc)
- [ ] Usage examples
- [ ] Updated FEATURE_LIST.md

**Tests**:
- [ ] All unit tests passing
- [ ] Integration test with talker_listener.launch.xml
- [ ] Comparison test vs dump_launch (passing)

---

## Success Metrics

- ✅ Parses talker_listener.launch.xml successfully
- ✅ Generates valid record.json
- ✅ Output matches dump_launch (semantic equivalence)
- ✅ Parse time <100ms for simple launches
- ✅ 90%+ unit test coverage
- ✅ All tests passing
- ✅ CLI works end-to-end

---

## Next Steps After MVP

**Phase 3: Include Resolution & Parameter Files**
- Recursive include parsing
- Parameter file loading and parsing
- Argument passing to includes

**Phase 4: YAML Parser**
- YAML file parsing with serde_yaml
- YamlEntity implementation
- YAML-specific features

**Phase 5: Advanced Substitutions**
- `$(find-pkg-share pkg)` with ament_index
- `$(command cmd)` execution
- Other advanced substitutions

**Phase 6: Composable Nodes**
- ComposableNodeContainer parsing
- LoadComposableNodes parsing
- Container metadata extraction

---

## Timeline Estimate

**Total Estimated Time**: 2-3 weeks (25-30 working days)

**Breakdown**:
- Task 2.1: Package Setup (2 days)
- Task 2.2: XML Foundation (3 days)
- Task 2.3: Substitution Engine (4 days)
- Task 2.4: Node Parsing (3 days)
- Task 2.5: Arg Parsing (2 days)
- Task 2.6: record.json Structures (2 days)
- Task 2.7: Command Generation (3 days)
- Task 2.8: Tree Traversal (3 days)
- Task 2.9: CLI Implementation (2 days)
- Task 2.10: Integration & Testing (3 days)

**Buffer**: +5 days for unexpected issues

---

## References

- **Feature List**: `docs/FEATURE_LIST.md`
- **ROS 2 Architecture**: `docs/ROS2_LAUNCH_ARCHITECTURE.md`
- **dump_launch Analysis**: `docs/DUMP_LAUNCH_ANALYSIS.md`
- **record.json Spec**: `docs/RECORD_JSON_FORMAT.md`
- **Research Summary**: `docs/RESEARCH_SUMMARY.md`
