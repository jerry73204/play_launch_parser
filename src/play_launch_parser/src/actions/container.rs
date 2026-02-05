//! Container action for composable nodes

use crate::{
    error::{ParseError, Result},
    params::extract_params_from_yaml,
    record::{ComposableNodeContainerRecord, LoadNodeRecord},
    substitution::{parse_substitutions, resolve_substitutions, LaunchContext},
    xml::{Entity, XmlEntity},
};
use std::{collections::HashMap, path::Path};

/// Container action representing a composable node container
#[derive(Debug, Clone)]
pub struct ContainerAction {
    pub name: String,
    pub namespace: String,
    pub package: String,
    pub executable: String,
    pub composable_nodes: Vec<ComposableNodeAction>,
}

/// Composable node action
#[derive(Debug, Clone)]
pub struct ComposableNodeAction {
    pub package: String,
    pub plugin: String,
    pub name: String,
    pub namespace: Option<String>,
    pub parameters: Vec<(String, String)>,
    pub remappings: Vec<(String, String)>,
    pub extra_args: HashMap<String, String>,
}

impl ContainerAction {
    pub fn from_entity(entity: &XmlEntity, context: &LaunchContext) -> Result<Self> {
        // Get container name
        let name_subs =
            entity
                .get_attr_str("name", false)?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "node_container".to_string(),
                    attribute: "name".to_string(),
                })?;
        let name_parsed = parse_substitutions(&name_subs)?;
        let name = resolve_substitutions(&name_parsed, context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;

        // Get package (defaults to "rclcpp_components" if not specified)
        let package = if let Some(pkg_str) = entity.get_attr_str("pkg", true)? {
            let pkg_parsed = parse_substitutions(&pkg_str)?;
            resolve_substitutions(&pkg_parsed, context)
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?
        } else {
            "rclcpp_components".to_string()
        };

        // Get executable (defaults to "component_container" if not specified)
        let executable = if let Some(exec_str) = entity.get_attr_str("exec", true)? {
            let exec_parsed = parse_substitutions(&exec_str)?;
            resolve_substitutions(&exec_parsed, context)
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?
        } else {
            "component_container".to_string()
        };

        // Get namespace, combining with accumulated namespace from context
        let namespace = if let Some(ns_str) = entity.get_attr_str("namespace", true)? {
            let ns_parsed = parse_substitutions(&ns_str)?;
            let ns_resolved = resolve_substitutions(&ns_parsed, context)
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;

            // If namespace is absolute (starts with '/'), use it as-is
            if ns_resolved.starts_with('/') {
                ns_resolved
            } else if ns_resolved.is_empty() {
                // Empty namespace means use current namespace from context
                context.current_namespace()
            } else {
                // Relative namespace: combine with current namespace from context
                let current_ns = context.current_namespace();
                if current_ns == "/" {
                    format!("/{}", ns_resolved)
                } else {
                    format!("{}/{}", current_ns, ns_resolved)
                }
            }
        } else {
            // No namespace attribute: use current namespace from context
            context.current_namespace()
        };

        // Parse composable_node children
        let mut composable_nodes = Vec::new();
        for child in entity.children() {
            match child.type_name() {
                "composable_node" | "composable-node" => {
                    composable_nodes.push(ComposableNodeAction::from_entity(&child, context)?);
                }
                "param" | "remap" | "env" => {
                    // Skip these for now - containers don't typically have their own params
                    log::debug!("Skipping {} in node_container", child.type_name());
                }
                other => {
                    log::warn!("Unexpected element '{}' in node_container", other);
                }
            }
        }

        Ok(Self {
            name,
            namespace,
            package,
            executable,
            composable_nodes,
        })
    }

    pub fn to_container_record(
        &self,
        context: &LaunchContext,
    ) -> Result<ComposableNodeContainerRecord> {
        // Generate the command line for the container executable
        let mut cmd = vec![
            format!("/opt/ros/humble/lib/{}/{}", self.package, self.executable),
            "--ros-args".to_string(),
            "-r".to_string(),
            format!("__node:={}", self.name),
            "-r".to_string(),
            format!("__ns:={}", self.namespace),
        ];

        // Add global parameters to the command (already filtered to SetParameter values)
        for (key, value) in context.global_parameters() {
            cmd.push("-p".to_string());
            cmd.push(format!("{}:={}", key, value));
        }

        // Generate exec_name using container name (not executable)
        // Python parser uses node name and strips counter suffixes
        let exec_name = Some(self.name.clone());

        // Collect global parameters (already filtered to SetParameter values)
        let global_params_vec: Vec<(String, String)> = context
            .global_parameters()
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect();
        let global_params = if global_params_vec.is_empty() {
            None
        } else {
            Some(global_params_vec)
        };

        Ok(ComposableNodeContainerRecord {
            args: None,
            cmd,
            env: None,
            exec_name,
            executable: self.executable.clone(),
            global_params,
            name: self.name.clone(),
            namespace: self.namespace.clone(),
            package: self.package.clone(),
            params: Vec::new(),
            params_files: Vec::new(),
            remaps: Vec::new(),
            respawn: None,
            respawn_delay: None,
            ros_args: None,
        })
    }

    pub fn to_load_node_records(&self) -> Vec<LoadNodeRecord> {
        self.composable_nodes
            .iter()
            .map(|node| node.to_load_node_record(&self.name, &self.namespace))
            .collect()
    }

    pub fn to_node_record(&self, context: &LaunchContext) -> Result<crate::record::NodeRecord> {
        use crate::record::NodeRecord;

        // Generate the command line for the container executable
        let mut cmd = vec![
            format!("/opt/ros/humble/lib/{}/{}", self.package, self.executable),
            "--ros-args".to_string(),
            "-r".to_string(),
            format!("__node:={}", self.name),
            "-r".to_string(),
            format!("__ns:={}", self.namespace),
        ];

        // Add global parameters to the command (already filtered to SetParameter values)
        for (key, value) in context.global_parameters() {
            cmd.push("-p".to_string());
            cmd.push(format!("{}:={}", key, value));
        }

        Ok(NodeRecord {
            args: None,
            cmd,
            env: None,
            exec_name: None,
            executable: self.executable.clone(),
            global_params: None,
            name: Some(self.name.clone()),
            namespace: Some(self.namespace.clone()),
            package: Some(self.package.clone()),
            params: Vec::new(),
            params_files: Vec::new(),
            remaps: Vec::new(),
            respawn: None,
            respawn_delay: None,
            ros_args: None,
        })
    }
}

impl ComposableNodeAction {
    pub fn from_entity(entity: &XmlEntity, context: &LaunchContext) -> Result<Self> {
        // Get required attributes
        let package_str =
            entity
                .get_attr_str("pkg", false)?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "composable_node".to_string(),
                    attribute: "pkg".to_string(),
                })?;
        let package_parsed = parse_substitutions(&package_str)?;
        let package = resolve_substitutions(&package_parsed, context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;

        let plugin_str =
            entity
                .get_attr_str("plugin", false)?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "composable_node".to_string(),
                    attribute: "plugin".to_string(),
                })?;
        let plugin_parsed = parse_substitutions(&plugin_str)?;
        let plugin = resolve_substitutions(&plugin_parsed, context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;

        // Get name (required for composable nodes)
        let name_str =
            entity
                .get_attr_str("name", false)?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "composable_node".to_string(),
                    attribute: "name".to_string(),
                })?;
        let name_parsed = parse_substitutions(&name_str)?;
        let name = resolve_substitutions(&name_parsed, context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;

        // Get optional namespace
        let namespace = if let Some(ns_str) = entity.get_attr_str("namespace", true)? {
            let ns_parsed = parse_substitutions(&ns_str)?;
            let ns_resolved = resolve_substitutions(&ns_parsed, context)
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;

            // Ensure namespace is absolute (starts with '/') or combine with current namespace
            let normalized_ns = if ns_resolved.starts_with('/') {
                // Already absolute
                ns_resolved
            } else if ns_resolved.is_empty() {
                // Empty namespace means use current namespace from context
                context.current_namespace()
            } else {
                // Relative namespace: combine with current namespace from context
                let current_ns = context.current_namespace();
                if current_ns == "/" {
                    format!("/{}", ns_resolved)
                } else {
                    format!("{}/{}", current_ns, ns_resolved)
                }
            };
            Some(normalized_ns)
        } else {
            None
        };

        // Parse children for params and remaps
        let mut parameters = Vec::new();
        let mut remappings = Vec::new();
        let extra_args = HashMap::new();

        for child in entity.children() {
            match child.type_name() {
                "param" => {
                    // Check if this is a parameter file reference
                    if let Some(from_attr) = child.get_attr_str("from", true)? {
                        // This is a parameter file - resolve path and load YAML
                        let from_parsed = parse_substitutions(&from_attr)?;
                        let from_resolved = resolve_substitutions(&from_parsed, context)
                            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;

                        // Load YAML parameter file and extract parameters
                        log::warn!(
                            "DEBUG container.rs: Found param file='{}', checking if YAML",
                            from_resolved
                        );
                        if from_resolved.ends_with(".yaml") || from_resolved.ends_with(".yml") {
                            log::warn!("DEBUG container.rs: Loading YAML file");
                            match extract_params_from_yaml(Path::new(&from_resolved), context) {
                                Ok(yaml_params) => {
                                    log::warn!(
                                        "DEBUG container.rs: Loaded {} parameters from {}",
                                        yaml_params.len(),
                                        from_resolved
                                    );
                                    parameters.extend(yaml_params);
                                }
                                Err(e) => {
                                    log::warn!(
                                        "DEBUG container.rs: Failed to load YAML parameter file {}: {}",
                                        from_resolved,
                                        e
                                    );
                                    // Fallback: store as __param_file
                                    parameters.push(("__param_file".to_string(), from_resolved));
                                }
                            }
                        } else {
                            // Non-YAML file - store as reference
                            parameters.push(("__param_file".to_string(), from_resolved));
                        }
                    } else {
                        // This is an inline parameter
                        let name = child.get_attr_str("name", false)?.ok_or_else(|| {
                            ParseError::MissingAttribute {
                                element: "param".to_string(),
                                attribute: "name".to_string(),
                            }
                        })?;
                        let value = child.get_attr_str("value", false)?.ok_or_else(|| {
                            ParseError::MissingAttribute {
                                element: "param".to_string(),
                                attribute: "value".to_string(),
                            }
                        })?;
                        let name_parsed = parse_substitutions(&name)?;
                        let value_parsed = parse_substitutions(&value)?;
                        let name_resolved = resolve_substitutions(&name_parsed, context)
                            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                        let value_resolved = resolve_substitutions(&value_parsed, context)
                            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                        parameters.push((name_resolved, value_resolved));
                    }
                }
                "remap" => {
                    let from = child.get_attr_str("from", false)?.ok_or_else(|| {
                        ParseError::MissingAttribute {
                            element: "remap".to_string(),
                            attribute: "from".to_string(),
                        }
                    })?;
                    let to = child.get_attr_str("to", false)?.ok_or_else(|| {
                        ParseError::MissingAttribute {
                            element: "remap".to_string(),
                            attribute: "to".to_string(),
                        }
                    })?;
                    let from_parsed = parse_substitutions(&from)?;
                    let to_parsed = parse_substitutions(&to)?;
                    let from_resolved = resolve_substitutions(&from_parsed, context)
                        .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                    let to_resolved = resolve_substitutions(&to_parsed, context)
                        .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                    remappings.push((from_resolved, to_resolved));
                }
                other => {
                    log::debug!("Skipping '{}' in composable_node", other);
                }
            }
        }

        Ok(Self {
            package,
            plugin,
            name,
            namespace,
            parameters,
            remappings,
            extra_args,
        })
    }

    pub fn to_load_node_record(
        &self,
        container_name: &str,
        container_namespace: &str,
    ) -> LoadNodeRecord {
        log::warn!(
            "DEBUG to_load_node_record: node='{}', self.parameters.len()={}",
            self.name,
            self.parameters.len()
        );

        // Build full target container name: namespace + name
        // This matches the Python implementation's behavior
        let target_container_name = if container_namespace == "/" {
            format!("/{}", container_name)
        } else if container_namespace.is_empty() {
            container_name.to_string()
        } else {
            format!("{}/{}", container_namespace, container_name)
        };

        // Merge global parameters with node-specific parameters
        let mut merged_params = Vec::new();
        {
            use crate::python::bridge::GLOBAL_PARAMETERS;
            let global_params = GLOBAL_PARAMETERS.lock();
            for (key, value) in global_params.iter() {
                merged_params.push((key.clone(), value.clone()));
            }
        }

        // Add node-specific parameters (may override global params)
        for (key, value) in &self.parameters {
            if let Some(existing) = merged_params.iter_mut().find(|(k, _)| k == key) {
                existing.1 = value.clone();
            } else {
                merged_params.push((key.clone(), value.clone()));
            }
        }

        log::warn!(
            "DEBUG to_load_node_record: node='{}', final merged_params.len()={}",
            self.name,
            merged_params.len()
        );

        LoadNodeRecord {
            package: self.package.clone(),
            plugin: self.plugin.clone(),
            target_container_name,
            node_name: self.name.clone(),
            namespace: self
                .namespace
                .clone()
                .unwrap_or_else(|| container_namespace.to_string()),
            log_level: None,
            remaps: self.remappings.clone(),
            params: merged_params,
            extra_args: self.extra_args.clone(),
            env: None,
        }
    }
}
