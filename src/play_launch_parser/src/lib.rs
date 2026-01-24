//! play_launch_parser library

pub mod actions;
pub mod condition;
pub mod error;
pub mod params;

#[cfg(feature = "python")]
pub mod python;

pub mod record;
pub mod substitution;
pub mod xml;

use actions::{
    ArgAction, ContainerAction, DeclareArgumentAction, ExecutableAction, GroupAction,
    IncludeAction, LetAction, LoadComposableNodeAction, NodeAction, SetEnvAction,
    SetParameterAction, SetRemapAction, UnsetEnvAction,
};
use condition::should_process_entity;
use dashmap::DashMap;
use error::{ParseError, Result};
use once_cell::sync::Lazy;
use record::{CommandGenerator, RecordJson};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::time::SystemTime;
use substitution::{parse_substitutions, resolve_substitutions, ArgumentMetadata, LaunchContext};
use xml::{Entity, XmlEntity};

/// Cached file content with modification time
struct CachedFile {
    content: String,
    modified: SystemTime,
}

/// Global file content cache
///
/// Thread-safe, lock-free reads. Bounded by actual files in workspace.
/// Expected size for Autoware: ~50-100 files × ~50KB/file = ~5-10MB total.
static FILE_CACHE: Lazy<DashMap<PathBuf, CachedFile>> = Lazy::new(DashMap::new);

/// Read file with caching and modification time validation
fn read_file_cached(path: &Path) -> Result<String> {
    let metadata = std::fs::metadata(path)?;
    let modified = metadata.modified()?;

    // Check cache with modification time validation
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
        CachedFile {
            content: content.clone(),
            modified,
        },
    );

    Ok(content)
}

/// Launch tree traverser for parsing launch files
pub struct LaunchTraverser {
    context: LaunchContext,
    records: Vec<record::NodeRecord>,
    containers: Vec<record::ComposableNodeContainerRecord>,
    load_nodes: Vec<record::LoadNodeRecord>,
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
            containers: Vec::new(),
            load_nodes: Vec::new(),
        }
    }

    /// Apply namespace prefix to a path (handles both absolute and relative namespaces)
    fn apply_namespace_prefix(prefix: &str, path: &str) -> String {
        // If path is empty, return the prefix
        if path.is_empty() {
            return prefix.to_string();
        }

        // If path is already absolute and starts with prefix, don't duplicate
        if path.starts_with(prefix) {
            return path.to_string();
        }

        // If path is absolute (starts with /), join properly
        if path.starts_with('/') {
            // If prefix ends with / or path is just /, handle carefully
            if prefix == "/" {
                return path.to_string();
            }
            // Remove leading / from path and join with prefix
            let path_without_slash = path.strip_prefix('/').unwrap_or(path);
            if path_without_slash.is_empty() {
                return prefix.to_string();
            }
            return format!("{}/{}", prefix, path_without_slash);
        }

        // Path is relative - join with /
        if prefix.ends_with('/') {
            format!("{}{}", prefix, path)
        } else {
            format!("{}/{}", prefix, path)
        }
    }

    pub fn traverse_file(&mut self, path: &Path) -> Result<()> {
        // Check file extension and handle non-XML files
        if let Some(ext) = path.extension().and_then(|s| s.to_str()) {
            match ext {
                "py" => {
                    #[cfg(feature = "python")]
                    {
                        log::info!("Executing Python launch file: {}", path.display());
                        return self.execute_python_file(path, &self.context.configurations());
                    }
                    #[cfg(not(feature = "python"))]
                    {
                        log::warn!(
                            "Skipping Python launch file (Python support not enabled): {}",
                            path.display()
                        );
                        return Ok(());
                    }
                }
                "yaml" | "yml" => {
                    // YAML files in traverse_file are always launch files
                    // (parameter files are handled in <param from="..."> context)
                    log::info!("Processing YAML launch file: {}", path.display());
                    self.process_yaml_launch_file(path)?;
                    return Ok(());
                }
                _ => {}
            }
        }

        // Set current file in context
        self.context.set_current_file(path.to_path_buf());

        let content = read_file_cached(path)?;
        let doc = roxmltree::Document::parse(&content)?;
        let root = xml::XmlEntity::new(doc.root_element());
        self.traverse_entity(&root)?;
        Ok(())
    }

    #[cfg(feature = "python")]
    fn execute_python_file(&mut self, path: &Path, args: &HashMap<String, String>) -> Result<()> {
        use crate::python::PythonLaunchExecutor;

        log::debug!("Executing Python file: {}", path.display());
        log::trace!("Python file arguments: {} args", args.len());
        for (k, v) in args {
            log::trace!("  {} = {}", k, v);
        }

        let executor =
            PythonLaunchExecutor::new().map_err(|e| ParseError::PythonError(e.to_string()))?;
        let (nodes, containers, load_nodes, includes) = executor.execute_launch_file(path, args)?;

        log::debug!(
            "Python file '{}' generated {} nodes, {} containers, {} load_nodes",
            path.display(),
            nodes.len(),
            containers.len(),
            load_nodes.len()
        );

        // Apply the current namespace context to all entities from Python
        // (Python launch files don't know about the XML namespace context they're included in)
        let current_ns = self.context.current_namespace();
        if current_ns != "/" {
            log::debug!(
                "Applying namespace '{}' to Python-generated entities",
                current_ns
            );

            // Apply namespace to nodes
            let namespaced_nodes: Vec<_> = nodes
                .into_iter()
                .map(|mut node| {
                    if let Some(ref ns) = node.namespace {
                        node.namespace = Some(Self::apply_namespace_prefix(&current_ns, ns));
                    } else {
                        node.namespace = Some(current_ns.clone());
                    }
                    node
                })
                .collect();

            // Apply namespace to containers and build a mapping of old names to new names
            let mut container_name_map = std::collections::HashMap::new();
            let namespaced_containers: Vec<_> = containers
                .into_iter()
                .map(|mut container| {
                    // Build old full name (namespace + name)
                    let old_full_name = if container.namespace.starts_with('/') {
                        if container.namespace == "/" {
                            format!("/{}", container.name)
                        } else {
                            format!("{}/{}", container.namespace, container.name)
                        }
                    } else {
                        format!("/{}/{}", container.namespace, container.name)
                    };

                    // Apply namespace prefix
                    container.namespace =
                        Self::apply_namespace_prefix(&current_ns, &container.namespace);

                    // Build new full name (namespace + name)
                    let new_full_name = if container.namespace.starts_with('/') {
                        if container.namespace == "/" {
                            format!("/{}", container.name)
                        } else {
                            format!("{}/{}", container.namespace, container.name)
                        }
                    } else {
                        format!("/{}/{}", container.namespace, container.name)
                    };

                    // Store mapping for updating load_nodes
                    container_name_map.insert(old_full_name, new_full_name);

                    container
                })
                .collect();

            // Apply namespace to load_nodes
            let namespaced_load_nodes: Vec<_> = load_nodes
                .into_iter()
                .map(|mut load_node| {
                    load_node.namespace =
                        Self::apply_namespace_prefix(&current_ns, &load_node.namespace);

                    // Update target_container_name if it matches a container that was renamed
                    if let Some(new_name) = container_name_map.get(&load_node.target_container_name) {
                        log::debug!(
                            "Updating target_container_name from '{}' to '{}'",
                            load_node.target_container_name,
                            new_name
                        );
                        load_node.target_container_name = new_name.clone();
                    } else {
                        // If not in the map, still apply namespace prefix for containers defined elsewhere
                        // This handles the case where target_container is a string reference to an external container
                        if load_node.target_container_name.starts_with('/') {
                            // Already absolute path - check if it needs namespace prefix
                            let prefixed = Self::apply_namespace_prefix(&current_ns, &load_node.target_container_name);
                            if prefixed != load_node.target_container_name {
                                log::debug!(
                                    "Applying namespace prefix to target_container_name: '{}' -> '{}'",
                                    load_node.target_container_name,
                                    prefixed
                                );
                                load_node.target_container_name = prefixed;
                            }
                        }
                    }

                    load_node
                })
                .collect();

            self.records.extend(namespaced_nodes);
            self.containers.extend(namespaced_containers);
            self.load_nodes.extend(namespaced_load_nodes);
        } else {
            self.records.extend(nodes);
            self.containers.extend(containers);
            self.load_nodes.extend(load_nodes);
        }

        // Process includes recursively
        for include in includes {
            log::debug!("Processing Python include: {}", include.file_path);

            // Convert args to HashMap
            let mut include_args = args.clone();
            for (key, value) in include.args {
                include_args.insert(key, value);
            }

            // Parse and resolve substitutions in the file path
            // (Python includes may contain substitutions like $(find-pkg-share pkg)/launch/file.xml)
            let file_path_subs = parse_substitutions(&include.file_path)?;
            let file_path_str =
                resolve_substitutions(&file_path_subs, &self.context).map_err(|e| {
                    ParseError::InvalidSubstitution(format!(
                        "Failed to resolve Python include path '{}': {}",
                        include.file_path, e
                    ))
                })?;

            log::debug!(
                "Resolved Python include path: {} -> {}",
                include.file_path,
                file_path_str
            );

            // Resolve relative paths relative to the current Python file
            let include_path = Path::new(&file_path_str);
            let resolved_include_path = if include_path.is_relative() {
                // Resolve relative to the current Python file
                if let Some(parent_dir) = path.parent() {
                    parent_dir.join(include_path)
                } else {
                    include_path.to_path_buf()
                }
            } else {
                include_path.to_path_buf()
            };

            // Recursively process the included file (determine type by extension)
            if let Some(ext) = resolved_include_path.extension().and_then(|s| s.to_str()) {
                match ext {
                    "py" => {
                        self.execute_python_file(&resolved_include_path, &include_args)?;
                    }
                    "xml" => {
                        // Process as XML include
                        self.process_xml_include(&resolved_include_path, &include_args)?;
                    }
                    "yaml" | "yml" => {
                        // Process as YAML launch file
                        self.process_yaml_launch_file(&resolved_include_path)?;
                    }
                    _ => {
                        log::warn!(
                            "Unknown file type for Python include: {}",
                            resolved_include_path.display()
                        );
                    }
                }
            } else {
                log::warn!(
                    "No file extension for Python include: {}",
                    resolved_include_path.display()
                );
            }
        }

        Ok(())
    }

    /// Helper to process an XML include with given arguments
    fn process_xml_include(
        &mut self,
        resolved_path: &Path,
        include_args: &HashMap<String, String>,
    ) -> Result<()> {
        log::info!("Including XML launch file: {}", resolved_path.display());

        // Create a new context for the included file (O(1) with Arc, not O(n) clone!)
        let mut include_context = self.context.child();
        include_context.set_current_file(resolved_path.to_path_buf());

        // Apply include arguments
        for (key, value) in include_args {
            log::debug!(
                "[RUST] Setting include arg from Python: {} = {}",
                key,
                value
            );
            include_context.set_configuration(key.clone(), value.clone());
        }

        // Parse and traverse the included file
        let content = read_file_cached(resolved_path)?;
        let doc = roxmltree::Document::parse(&content)?;
        let root = xml::XmlEntity::new(doc.root_element());

        // Create temporary traverser for included file
        let mut included_traverser = LaunchTraverser {
            context: include_context,
            records: Vec::new(),
            containers: Vec::new(),
            load_nodes: Vec::new(),
        };
        included_traverser.traverse_entity(&root)?;

        // Merge records from included file into current records
        self.records.extend(included_traverser.records);
        self.containers.extend(included_traverser.containers);
        self.load_nodes.extend(included_traverser.load_nodes);

        Ok(())
    }

    fn process_include(&mut self, include: &IncludeAction) -> Result<()> {
        // Resolve the file path
        let file_path_str = resolve_substitutions(&include.file, &self.context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
        let file_path = Path::new(&file_path_str);

        log::trace!("Processing include: {}", file_path_str);

        // Resolve relative paths relative to the current launch file
        let resolved_path = if file_path.is_relative() {
            if let Some(current_file) = self.context.current_file() {
                if let Some(parent_dir) = current_file.parent() {
                    parent_dir.join(file_path)
                } else {
                    file_path.to_path_buf()
                }
            } else {
                file_path.to_path_buf()
            }
        } else {
            file_path.to_path_buf()
        };

        log::info!("Including launch file: {}", resolved_path.display());

        // Log include arguments being passed
        if !include.args.is_empty() {
            let arg_names: Vec<&str> = include.args.iter().map(|(k, _)| k.as_str()).collect();
            log::trace!("Include args: {:?}", arg_names);
        }

        // Check if this is a Python launch file
        // Check file extension and handle non-XML files
        if let Some(ext) = resolved_path.extension().and_then(|s| s.to_str()) {
            match ext {
                "py" => {
                    #[cfg(feature = "python")]
                    {
                        log::info!("Including Python launch file: {}", resolved_path.display());

                        // Create args for the Python file (include args override current context)
                        let mut python_args = self.context.configurations();
                        for (key, value) in &include.args {
                            let resolved_value_subs = parse_substitutions(value)?;
                            let resolved_value =
                                resolve_substitutions(&resolved_value_subs, &self.context)
                                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                            log::trace!("  Include arg: {} = {}", key, resolved_value);
                            python_args.insert(key.clone(), resolved_value);
                        }

                        return self.execute_python_file(&resolved_path, &python_args);
                    }
                    #[cfg(not(feature = "python"))]
                    {
                        log::warn!(
                            "Skipping Python launch file (Python support not enabled): {}",
                            resolved_path.display()
                        );
                        return Ok(());
                    }
                }
                "yaml" | "yml" => {
                    // YAML files in <include> are always launch files
                    // (parameter files are handled in <param from="..."> context)
                    log::info!("Including YAML launch file: {}", resolved_path.display());
                    self.process_yaml_launch_file(&resolved_path)?;
                    return Ok(());
                }
                _ => {}
            }
        }

        // Create a new context for the included file (O(1) with Arc, not O(n) clone!)
        // Start with current context and apply include args
        let mut include_context = self.context.child();
        include_context.set_current_file(resolved_path.clone());
        for (key, value) in &include.args {
            // IMPORTANT: Resolve substitutions in the argument value using the include_context
            // (not the parent context) so that later args can reference earlier args
            // Example: <arg name="A" value="x"/>
            //          <arg name="B" value="$(var A)/y"/>  <-- B can reference A
            let resolved_value_subs = parse_substitutions(value)?;
            let resolved_value = resolve_substitutions(&resolved_value_subs, &include_context)
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
            log::debug!("[RUST] Setting include arg: {} = {}", key, resolved_value);
            include_context.set_configuration(key.clone(), resolved_value);
        }

        log::debug!(
            "[RUST] Include context has {} configs after setting include args",
            include_context.configurations().len()
        );

        // Parse and traverse the included file
        let content = read_file_cached(&resolved_path)?;
        let doc = roxmltree::Document::parse(&content)?;
        let root = xml::XmlEntity::new(doc.root_element());

        // Create temporary traverser for included file
        let mut included_traverser = LaunchTraverser {
            context: include_context,
            records: Vec::new(),
            containers: Vec::new(),
            load_nodes: Vec::new(),
        };
        included_traverser.traverse_entity(&root)?;

        // Merge records from included file into current records
        self.records.extend(included_traverser.records);
        self.containers.extend(included_traverser.containers);
        self.load_nodes.extend(included_traverser.load_nodes);

        Ok(())
    }

    fn traverse_entity(&mut self, entity: &XmlEntity) -> Result<()> {
        // Check if entity should be processed based on if/unless conditions
        if !should_process_entity(entity, &self.context)? {
            log::debug!("Skipping {} due to condition", entity.type_name());
            return Ok(());
        }

        match entity.type_name() {
            "launch" => {
                // Root element, traverse children
                for child in entity.children() {
                    self.traverse_entity(&child)?;
                }
            }
            "arg" => {
                let arg = ArgAction::from_entity(entity)?;
                arg.apply(&mut self.context, &HashMap::new());
            }
            "declare_argument" => {
                let declare_arg = DeclareArgumentAction::from_entity(entity)?;

                // Resolve default value if present
                let default = if let Some(default_subs) = &declare_arg.default {
                    Some(
                        resolve_substitutions(default_subs, &self.context)
                            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?,
                    )
                } else {
                    None
                };

                log::debug!(
                    "[RUST] Declaring argument: {} = {:?}",
                    declare_arg.name,
                    default
                );

                // Store metadata
                let metadata = ArgumentMetadata {
                    name: declare_arg.name.clone(),
                    default,
                    description: declare_arg.description.clone(),
                    choices: declare_arg.choices.clone(),
                };
                self.context.declare_argument(metadata);

                // If a default value is provided and the argument is not yet set, apply it
                if let Some(default_val) = &declare_arg.default {
                    if self.context.get_configuration(&declare_arg.name).is_none() {
                        let resolved_default = resolve_substitutions(default_val, &self.context)
                            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                        log::debug!(
                            "[RUST] Setting default value for {}: {}",
                            declare_arg.name,
                            resolved_default
                        );
                        self.context
                            .set_configuration(declare_arg.name, resolved_default);
                    }
                }
            }
            "node" => {
                let node = NodeAction::from_entity(entity)?;
                let record = CommandGenerator::generate_node_record(&node, &self.context)
                    .map_err(|e| ParseError::IoError(std::io::Error::other(e.to_string())))?;
                self.records.push(record);
            }
            "executable" => {
                let exec = ExecutableAction::from_entity(entity)?;
                let record = CommandGenerator::generate_executable_record(&exec, &self.context)
                    .map_err(|e| ParseError::IoError(std::io::Error::other(e.to_string())))?;
                self.records.push(record);
            }
            "include" => {
                let include = IncludeAction::from_entity(entity)?;
                self.process_include(&include)?;
            }
            "group" => {
                let group = GroupAction::from_entity(entity)?;

                // Save namespace depth and remapping count to restore after group
                let initial_depth = self.context.namespace_depth();
                let initial_remap_count = self.context.remapping_count();

                // Push namespace if specified
                if let Some(ns_subs) = &group.namespace {
                    let namespace = resolve_substitutions(ns_subs, &self.context)
                        .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                    self.context.push_namespace(namespace);
                }

                // Traverse children with scoped namespace and remappings
                for child in entity.children() {
                    self.traverse_entity(&child)?;
                }

                // Restore namespace depth and remappings (scoped to this group)
                self.context.restore_namespace_depth(initial_depth);
                self.context.restore_remapping_count(initial_remap_count);
            }
            "let" => {
                let let_action = LetAction::from_entity(entity)?;
                // Set variable in context (acts like arg)
                self.context
                    .set_configuration(let_action.name, let_action.value);
            }
            "set_env" | "set-env" => {
                let set_env = SetEnvAction::from_entity(entity)?;
                let value = resolve_substitutions(&set_env.value, &self.context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                self.context.set_environment_variable(set_env.name, value);
            }
            "unset_env" | "unset-env" => {
                let unset_env = UnsetEnvAction::from_entity(entity)?;
                self.context.unset_environment_variable(&unset_env.name);
            }
            "set_parameter" => {
                let set_param = SetParameterAction::from_entity(entity)?;
                let value = resolve_substitutions(&set_param.value, &self.context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                self.context.set_global_parameter(set_param.name, value);
            }
            "set_remap" | "set-remap" => {
                let set_remap = SetRemapAction::from_entity(entity)?;
                let from = resolve_substitutions(&set_remap.from, &self.context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                let to = resolve_substitutions(&set_remap.to, &self.context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                self.context.add_remapping(from, to);
            }
            "push-ros-namespace" => {
                // Try "namespace" attribute first (Autoware/ROS 2 standard),
                // then fall back to "ns" for backwards compatibility
                let ns_str = entity
                    .get_attr_str("namespace", false)
                    .ok()
                    .flatten()
                    .or_else(|| entity.get_attr_str("ns", false).ok().flatten())
                    .ok_or_else(|| ParseError::MissingAttribute {
                        element: "push-ros-namespace".to_string(),
                        attribute: "namespace or ns".to_string(),
                    })?;

                let ns_subs = parse_substitutions(&ns_str)?;
                let namespace = resolve_substitutions(&ns_subs, &self.context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                self.context.push_namespace(namespace);
            }
            "pop-ros-namespace" => {
                self.context.pop_namespace();
            }
            "node_container" | "node-container" => {
                // Parse container and its composable nodes
                let container_action = ContainerAction::from_entity(entity, &self.context)?;

                // Add container record
                self.containers.push(container_action.to_container_record());

                // Add load_node records for each composable node
                self.load_nodes
                    .extend(container_action.to_load_node_records());

                log::info!(
                    "Parsed container '{}' with {} composable nodes",
                    container_action.name,
                    container_action.composable_nodes.len()
                );
            }
            "composable_node" | "composable-node" => {
                // Composable nodes are loaded into containers
                // Standalone composable nodes should be inside a node_container
                log::info!("Skipping standalone composable_node (should be in node_container)");
            }
            "load_composable_node" | "load-composable-node" => {
                // Parse load_composable_node action and convert to LoadNodeRecords
                let action = LoadComposableNodeAction::from_entity(entity, &self.context)?;
                let load_records = action.to_load_node_records(&self.context)?;

                log::info!(
                    "Loaded {} composable node(s) into container via load_composable_node",
                    load_records.len()
                );

                // Add to load_nodes
                self.load_nodes.extend(load_records);
            }
            other => {
                log::warn!("Unsupported action type: {}", other);
                // For MVP: skip unknown actions
            }
        }
        Ok(())
    }

    pub fn into_record_json(self) -> RecordJson {
        RecordJson {
            node: self.records,
            container: self.containers,
            load_node: self.load_nodes,
            lifecycle_node: Vec::new(),
            file_data: HashMap::new(),
        }
    }

    /// Process a YAML launch file to extract argument declarations
    fn process_yaml_launch_file(&mut self, path: &Path) -> Result<()> {
        use serde_yaml::Value;

        let content = read_file_cached(path)?;
        let yaml: Value = serde_yaml::from_str(&content)
            .map_err(|e| ParseError::InvalidSubstitution(format!("Invalid YAML: {}", e)))?;

        // Extract 'launch' key
        if let Some(launch_list) = yaml.get("launch").and_then(|v| v.as_sequence()) {
            for item in launch_list {
                // Check if this is an arg declaration
                if let Some(arg_map) = item.get("arg").and_then(|v| v.as_mapping()) {
                    if let (Some(name), default_value) = (
                        arg_map
                            .get(Value::String("name".to_string()))
                            .and_then(|v| v.as_str()),
                        arg_map
                            .get(Value::String("default".to_string()))
                            .and_then(|v| v.as_str()),
                    ) {
                        // Declare the argument in the context
                        log::debug!("[RUST] YAML declares arg: {} = {:?}", name, default_value);
                        self.context.declare_argument(ArgumentMetadata {
                            name: name.to_string(),
                            default: default_value.map(|s| s.to_string()),
                            description: None,
                            choices: None,
                        });

                        // Set the configuration value if a default is provided and not already set
                        if let Some(default) = default_value {
                            if self.context.get_configuration(name).is_none() {
                                log::debug!(
                                    "[RUST] YAML setting default value for {}: {}",
                                    name,
                                    default
                                );
                                self.context
                                    .set_configuration(name.to_string(), default.to_string());
                            }
                        }
                    }
                }
            }
        }

        Ok(())
    }
}

/// Parse launch file and generate record.json
pub fn parse_launch_file(path: &Path, cli_args: HashMap<String, String>) -> Result<RecordJson> {
    let mut traverser = LaunchTraverser::new(cli_args);
    traverser.traverse_file(path)?;
    Ok(traverser.into_record_json())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use tempfile::NamedTempFile;

    #[test]
    fn test_parse_simple_launch() {
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(record.node[0].executable, "talker");
        assert_eq!(record.node[0].package, Some("demo_nodes_cpp".to_string()));
    }

    #[test]
    fn test_parse_with_args() {
        let xml = r#"<launch>
            <arg name="node_name" default="my_talker" />
            <node pkg="demo_nodes_cpp" exec="talker" name="$(var node_name)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(record.node[0].name, Some("my_talker".to_string()));
    }

    #[test]
    fn test_parse_with_cli_override() {
        let xml = r#"<launch>
            <arg name="node_name" default="default_name" />
            <node pkg="demo_nodes_cpp" exec="talker" name="$(var node_name)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let mut cli_args = HashMap::new();
        cli_args.insert("node_name".to_string(), "cli_name".to_string());

        let record = parse_launch_file(file.path(), cli_args).unwrap();
        assert_eq!(record.node[0].name, Some("cli_name".to_string()));
    }

    #[test]
    fn test_parse_multiple_nodes() {
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" />
            <node pkg="demo_nodes_cpp" exec="listener" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 2);
        assert_eq!(record.node[0].executable, "talker");
        assert_eq!(record.node[1].executable, "listener");
    }

    #[test]
    fn test_parse_executable() {
        let xml = r#"<launch>
            <executable cmd="rosbag" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(record.node[0].executable, "rosbag");
        assert_eq!(record.node[0].package, None);
    }

    #[test]
    fn test_parse_executable_with_args() {
        let xml = r#"<launch>
            <executable cmd="rosbag">
                <arg value="record" />
                <arg value="-a" />
            </executable>
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(record.node[0].executable, "rosbag");
        assert_eq!(record.node[0].cmd, vec!["rosbag", "record", "-a"]);
        assert_eq!(
            record.node[0].args,
            Some(vec!["record".to_string(), "-a".to_string()])
        );
    }

    #[test]
    fn test_parse_mixed_nodes_and_executables() {
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" />
            <executable cmd="rviz2" />
            <node pkg="demo_nodes_cpp" exec="listener" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 3);
        assert_eq!(record.node[0].executable, "talker");
        assert_eq!(record.node[0].package, Some("demo_nodes_cpp".to_string()));
        assert_eq!(record.node[1].executable, "rviz2");
        assert_eq!(record.node[1].package, None);
        assert_eq!(record.node[2].executable, "listener");
        assert_eq!(record.node[2].package, Some("demo_nodes_cpp".to_string()));
    }

    #[test]
    fn test_parse_node_with_param_file() {
        use std::io::Write;

        // Create a parameter YAML file
        let yaml_content = r#"
test_node:
  ros__parameters:
    param1: "value1"
    param2: 42
    nested:
      param3: "nested_value"
"#;
        let mut param_file = NamedTempFile::new().unwrap();
        param_file.write_all(yaml_content.as_bytes()).unwrap();
        param_file.flush().unwrap();

        let param_file_path = param_file.path().to_str().unwrap();

        // Create a launch file that references the parameter file
        let xml = format!(
            r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker">
                <param name="inline_param" value="inline_value" />
                <param from="{}" />
            </node>
        </launch>"#,
            param_file_path
        );

        let mut launch_file = NamedTempFile::new().unwrap();
        launch_file.write_all(xml.as_bytes()).unwrap();
        launch_file.flush().unwrap();

        let record = parse_launch_file(launch_file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);

        // Check that both inline and file parameters are present
        let node = &record.node[0];
        assert!(node.params.len() >= 4); // inline_param + 3 from YAML

        // Check inline parameter
        let inline_param = node
            .params
            .iter()
            .find(|(k, _)| k == "inline_param")
            .expect("inline_param not found");
        assert_eq!(inline_param.1, "inline_value");

        // Check parameters from file
        let param1 = node
            .params
            .iter()
            .find(|(k, _)| k == "param1")
            .expect("param1 not found");
        assert_eq!(param1.1, "value1");

        let param2 = node
            .params
            .iter()
            .find(|(k, _)| k == "param2")
            .expect("param2 not found");
        assert_eq!(param2.1, "42");

        let nested_param = node
            .params
            .iter()
            .find(|(k, _)| k == "nested.param3")
            .expect("nested.param3 not found");
        assert_eq!(nested_param.1, "nested_value");

        // Check that the param file path is recorded
        assert_eq!(node.params_files.len(), 1);
        assert_eq!(node.params_files[0], param_file_path);
    }

    #[test]
    fn test_namespace_scoping_simple() {
        let xml = r#"<launch>
            <group>
                <push-ros-namespace ns="robot1" />
                <node pkg="demo_nodes_cpp" exec="talker" />
            </group>
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(record.node[0].namespace, Some("/robot1".to_string()));
    }

    #[test]
    fn test_namespace_scoping_nested() {
        let xml = r#"<launch>
            <group>
                <push-ros-namespace ns="robot1" />
                <group>
                    <push-ros-namespace ns="sensors" />
                    <node pkg="demo_nodes_cpp" exec="talker" />
                </group>
            </group>
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(
            record.node[0].namespace,
            Some("/robot1/sensors".to_string())
        );
    }

    #[test]
    fn test_namespace_scoping_with_group_attribute() {
        let xml = r#"<launch>
            <group ns="robot1">
                <node pkg="demo_nodes_cpp" exec="talker" />
                <group ns="sensors">
                    <node pkg="demo_nodes_cpp" exec="listener" />
                </group>
            </group>
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 2);
        assert_eq!(record.node[0].namespace, Some("/robot1".to_string()));
        assert_eq!(
            record.node[1].namespace,
            Some("/robot1/sensors".to_string())
        );
    }

    #[test]
    fn test_namespace_explicit_overrides_group() {
        let xml = r#"<launch>
            <group ns="robot1">
                <node pkg="demo_nodes_cpp" exec="talker" namespace="/override" />
            </group>
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        // Explicit namespace should override group namespace
        assert_eq!(record.node[0].namespace, Some("/override".to_string()));
    }

    #[test]
    fn test_namespace_absolute() {
        let xml = r#"<launch>
            <group ns="robot1">
                <group ns="/absolute">
                    <node pkg="demo_nodes_cpp" exec="talker" />
                </group>
            </group>
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        // Absolute namespace should override parent
        assert_eq!(record.node[0].namespace, Some("/absolute".to_string()));
    }

    #[test]
    fn test_set_env_literal() {
        let xml = r#"<launch>
            <set_env name="ROS_DOMAIN_ID" value="42" />
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);

        // Check that environment was set
        let env = record.node[0].env.as_ref().unwrap();
        assert!(env.iter().any(|(k, v)| k == "ROS_DOMAIN_ID" && v == "42"));
    }

    #[test]
    fn test_set_env_with_substitution() {
        let xml = r#"<launch>
            <arg name="domain_id" default="99" />
            <set_env name="ROS_DOMAIN_ID" value="$(var domain_id)" />
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);

        let env = record.node[0].env.as_ref().unwrap();
        assert!(env.iter().any(|(k, v)| k == "ROS_DOMAIN_ID" && v == "99"));
    }

    #[test]
    fn test_unset_env() {
        let xml = r#"<launch>
            <set_env name="VAR1" value="value1" />
            <set_env name="VAR2" value="value2" />
            <unset_env name="VAR1" />
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);

        let env = record.node[0].env.as_ref().unwrap();
        // VAR1 should not be present
        assert!(!env.iter().any(|(k, _)| k == "VAR1"));
        // VAR2 should be present
        assert!(env.iter().any(|(k, v)| k == "VAR2" && v == "value2"));
    }

    #[test]
    fn test_env_inheritance_in_nodes() {
        let xml = r#"<launch>
            <set_env name="GLOBAL_VAR" value="global" />
            <node pkg="demo_nodes_cpp" exec="talker">
                <env name="NODE_VAR" value="node_specific" />
            </node>
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);

        let env = record.node[0].env.as_ref().unwrap();
        // Both global and node-specific environment should be present
        assert!(env.iter().any(|(k, v)| k == "GLOBAL_VAR" && v == "global"));
        assert!(env
            .iter()
            .any(|(k, v)| k == "NODE_VAR" && v == "node_specific"));
    }

    #[test]
    fn test_node_env_overrides_context() {
        let xml = r#"<launch>
            <set_env name="MY_VAR" value="context_value" />
            <node pkg="demo_nodes_cpp" exec="talker">
                <env name="MY_VAR" value="node_value" />
            </node>
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);

        let env = record.node[0].env.as_ref().unwrap();
        // Node-specific value should override context value
        let my_var = env.iter().find(|(k, _)| k == "MY_VAR").unwrap();
        assert_eq!(my_var.1, "node_value");
    }

    #[test]
    fn test_env_with_executable() {
        let xml = r#"<launch>
            <set_env name="EXEC_VAR" value="exec_value" />
            <executable cmd="echo" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);

        let env = record.node[0].env.as_ref().unwrap();
        assert!(env
            .iter()
            .any(|(k, v)| k == "EXEC_VAR" && v == "exec_value"));
    }

    #[test]
    fn test_declare_argument_with_default() {
        let xml = r#"<launch>
            <declare_argument name="my_arg" default="default_value" />
            <node pkg="demo_nodes_cpp" exec="talker" name="$(var my_arg)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        // The default value should be used
        assert_eq!(record.node[0].name, Some("default_value".to_string()));
    }

    #[test]
    fn test_declare_argument_cli_override() {
        let xml = r#"<launch>
            <declare_argument name="my_arg" default="default_value" />
            <node pkg="demo_nodes_cpp" exec="talker" name="$(var my_arg)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let mut cli_args = HashMap::new();
        cli_args.insert("my_arg".to_string(), "cli_value".to_string());

        let record = parse_launch_file(file.path(), cli_args).unwrap();
        assert_eq!(record.node.len(), 1);
        // CLI value should override default
        assert_eq!(record.node[0].name, Some("cli_value".to_string()));
    }

    #[test]
    fn test_declare_argument_without_default() {
        let xml = r#"<launch>
            <declare_argument name="required_arg" description="A required argument" />
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        // This should not crash even though required_arg is declared without a default
        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
    }

    #[test]
    fn test_declare_argument_then_arg() {
        let xml = r#"<launch>
            <declare_argument name="my_param" description="Test parameter" />
            <arg name="my_param" default="set_value" />
            <node pkg="demo_nodes_cpp" exec="talker" name="$(var my_param)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        // Value set by <arg> should be used
        assert_eq!(record.node[0].name, Some("set_value".to_string()));
    }

    #[test]
    fn test_declare_argument_with_description() {
        let xml = r#"<launch>
            <declare_argument
                name="robot_name"
                default="robot1"
                description="Name of the robot"
            />
            <node pkg="demo_nodes_cpp" exec="talker" name="$(var robot_name)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(record.node[0].name, Some("robot1".to_string()));
    }

    #[test]
    fn test_set_parameter_simple() {
        let xml = r#"<launch>
            <set_parameter name="use_sim_time" value="true" />
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);

        // Check that global parameter was set
        let global_params = record.node[0].global_params.as_ref().unwrap();
        assert!(global_params
            .iter()
            .any(|(k, v)| k == "use_sim_time" && v == "true"));
    }

    #[test]
    fn test_set_parameter_with_substitution() {
        let xml = r#"<launch>
            <arg name="sim_mode" default="true" />
            <set_parameter name="use_sim_time" value="$(var sim_mode)" />
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);

        let global_params = record.node[0].global_params.as_ref().unwrap();
        assert!(global_params
            .iter()
            .any(|(k, v)| k == "use_sim_time" && v == "true"));
    }

    #[test]
    fn test_multiple_set_parameters() {
        let xml = r#"<launch>
            <set_parameter name="use_sim_time" value="true" />
            <set_parameter name="log_level" value="debug" />
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);

        let global_params = record.node[0].global_params.as_ref().unwrap();
        assert_eq!(global_params.len(), 2);
        assert!(global_params
            .iter()
            .any(|(k, v)| k == "use_sim_time" && v == "true"));
        assert!(global_params
            .iter()
            .any(|(k, v)| k == "log_level" && v == "debug"));
    }

    #[test]
    fn test_set_parameter_applies_to_all_nodes() {
        let xml = r#"<launch>
            <set_parameter name="use_sim_time" value="true" />
            <node pkg="demo_nodes_cpp" exec="talker" />
            <node pkg="demo_nodes_cpp" exec="listener" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 2);

        // Both nodes should have the global parameter
        for node_record in &record.node {
            let global_params = node_record.global_params.as_ref().unwrap();
            assert!(global_params
                .iter()
                .any(|(k, v)| k == "use_sim_time" && v == "true"));
        }
    }

    #[test]
    fn test_optenv_with_existing_var() {
        std::env::set_var("TEST_OPTENV_LAUNCH", "from_env");
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="$(optenv TEST_OPTENV_LAUNCH)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(record.node[0].name, Some("from_env".to_string()));
    }

    #[test]
    fn test_optenv_with_missing_var_no_default() {
        std::env::remove_var("NONEXISTENT_OPTENV_LAUNCH");
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="node_$(optenv NONEXISTENT_OPTENV_LAUNCH)_end" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        // Should use empty string for missing var
        assert_eq!(record.node[0].name, Some("node__end".to_string()));
    }

    #[test]
    fn test_optenv_with_missing_var_with_default() {
        std::env::remove_var("NONEXISTENT_OPTENV_LAUNCH2");
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="$(optenv NONEXISTENT_OPTENV_LAUNCH2 default_name)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        // Should use default value
        assert_eq!(record.node[0].name, Some("default_name".to_string()));
    }

    #[test]
    fn test_optenv_vs_env_missing_var() {
        std::env::remove_var("MISSING_TEST_VAR");

        // optenv should work with missing variable
        let xml_optenv = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="$(optenv MISSING_TEST_VAR fallback)" />
        </launch>"#;
        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml_optenv.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node[0].name, Some("fallback".to_string()));

        // env without default should error
        let xml_env = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="$(env MISSING_TEST_VAR)" />
        </launch>"#;
        let mut file2 = NamedTempFile::new().unwrap();
        file2.write_all(xml_env.as_bytes()).unwrap();

        let result = parse_launch_file(file2.path(), HashMap::new());
        assert!(result.is_err());
    }

    #[test]
    fn test_command_simple() {
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="$(command echo test_node)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(record.node[0].name, Some("test_node".to_string()));
    }

    #[test]
    fn test_command_with_args() {
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="$(command echo foo bar)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(record.node[0].name, Some("foo bar".to_string()));
    }

    #[test]
    fn test_command_output_trimming() {
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="$(command printf '  test  \n')" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        // Output should be trimmed
        assert_eq!(record.node[0].name, Some("test".to_string()));
    }

    #[test]
    fn test_command_in_parameter() {
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker">
                <param name="hostname" value="$(command hostname)" />
            </node>
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        // Should have the hostname parameter with some value
        let hostname_param = record.node[0]
            .params
            .iter()
            .find(|(k, _)| k == "hostname")
            .expect("hostname param not found");
        // Just verify it's not empty
        assert!(!hostname_param.1.is_empty());
    }

    #[test]
    fn test_command_with_pipe() {
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="$(command echo test | tr a-z A-Z)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(record.node[0].name, Some("TEST".to_string()));
    }

    #[test]
    fn test_command_failed() {
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="$(command exit 1)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        // Should fail because the command exits with non-zero status
        let result = parse_launch_file(file.path(), HashMap::new());
        assert!(result.is_err());
    }
}
