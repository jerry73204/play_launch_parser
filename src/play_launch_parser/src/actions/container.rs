//! Container action for composable nodes

use crate::error::{ParseError, Result};
use crate::record::{ComposableNodeContainerRecord, LoadNodeRecord};
use crate::substitution::{parse_substitutions, resolve_substitutions, LaunchContext};
use crate::xml::{Entity, XmlEntity};
use std::collections::HashMap;

/// Container action representing a composable node container
#[derive(Debug, Clone)]
pub struct ContainerAction {
    pub name: String,
    pub namespace: String,
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

        // Get namespace (default to "/")
        let namespace = if let Some(ns_str) = entity.get_attr_str("namespace", true)? {
            let ns_parsed = parse_substitutions(&ns_str)?;
            resolve_substitutions(&ns_parsed, context)
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?
        } else {
            "/".to_string()
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
            composable_nodes,
        })
    }

    pub fn to_container_record(&self) -> ComposableNodeContainerRecord {
        ComposableNodeContainerRecord {
            name: self.name.clone(),
            namespace: self.namespace.clone(),
        }
    }

    pub fn to_load_node_records(&self) -> Vec<LoadNodeRecord> {
        self.composable_nodes
            .iter()
            .map(|node| node.to_load_node_record(&self.name, &self.namespace))
            .collect()
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
            Some(
                resolve_substitutions(&ns_parsed, context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?,
            )
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
        LoadNodeRecord {
            package: self.package.clone(),
            plugin: self.plugin.clone(),
            target_container_name: container_name.to_string(),
            node_name: self.name.clone(),
            namespace: self
                .namespace
                .clone()
                .unwrap_or_else(|| container_namespace.to_string()),
            log_level: None,
            remaps: self.remappings.clone(),
            params: self.parameters.clone(),
            extra_args: self.extra_args.clone(),
            env: None,
        }
    }
}
