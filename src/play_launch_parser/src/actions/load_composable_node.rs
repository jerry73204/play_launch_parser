//! Load composable node action

use crate::{
    actions::ComposableNodeAction,
    error::{ParseError, Result},
    record::LoadNodeRecord,
    substitution::{parse_substitutions, resolve_substitutions, LaunchContext, Substitution},
    xml::{Entity, XmlEntity},
};

/// Load composable node action - loads composable nodes into an existing container
#[derive(Debug, Clone)]
pub struct LoadComposableNodeAction {
    /// Target container name (can contain substitutions)
    pub target: Vec<Substitution>,
    /// Composable nodes to load
    pub composable_nodes: Vec<ComposableNodeAction>,
}

impl LoadComposableNodeAction {
    pub fn from_entity(entity: &XmlEntity, context: &LaunchContext) -> Result<Self> {
        // Get target attribute (container name/reference)
        let target_str =
            entity
                .get_attr_str("target", false)?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "load_composable_node".to_string(),
                    attribute: "target".to_string(),
                })?;
        let target = parse_substitutions(&target_str)?;

        // Parse composable_node children
        let mut composable_nodes = Vec::new();
        for child in entity.children() {
            match child.type_name() {
                "composable_node" | "composable-node" => {
                    composable_nodes.push(ComposableNodeAction::from_entity(&child, context)?);
                }
                other => {
                    log::debug!("Skipping '{}' in load_composable_node", other);
                }
            }
        }

        if composable_nodes.is_empty() {
            log::warn!("load_composable_node has no composable_node children");
        }

        Ok(Self {
            target,
            composable_nodes,
        })
    }

    /// Convert to LoadNodeRecords by resolving the target container
    pub fn to_load_node_records(&self, context: &LaunchContext) -> Result<Vec<LoadNodeRecord>> {
        // Resolve target container name
        let target_container_name = resolve_substitutions(&self.target, context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;

        // Normalize target container name
        // If it starts with '/', keep it as is
        // Otherwise, prepend current namespace
        let normalized_target = if target_container_name.starts_with('/') {
            target_container_name
        } else {
            let current_ns = context.current_namespace();
            if current_ns == "/" {
                format!("/{}", target_container_name)
            } else {
                format!("{}/{}", current_ns, target_container_name)
            }
        };

        // Convert composable nodes to LoadNodeRecords
        let records: Vec<LoadNodeRecord> = self
            .composable_nodes
            .iter()
            .map(|node| {
                // Extract the container namespace from the target (everything except the last segment)
                let container_namespace = if let Some(last_slash_idx) = normalized_target.rfind('/')
                {
                    if last_slash_idx == 0 {
                        "/"
                    } else {
                        &normalized_target[..last_slash_idx]
                    }
                } else {
                    "/"
                };

                // Build LoadNodeRecord
                LoadNodeRecord {
                    package: node.package.clone(),
                    plugin: node.plugin.clone(),
                    target_container_name: normalized_target.clone(),
                    node_name: node.name.clone(),
                    namespace: node
                        .namespace
                        .clone()
                        .unwrap_or_else(|| container_namespace.to_string()),
                    log_level: None,
                    remaps: node.remappings.clone(),
                    params: node.parameters.clone(),
                    extra_args: node.extra_args.clone(),
                    env: None,
                }
            })
            .collect();

        Ok(records)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xml::XmlEntity;
    use roxmltree::Document;

    #[test]
    fn test_parse_load_composable_node_simple() {
        let xml = r#"
            <load_composable_node target="/control/control_container">
                <composable_node pkg="my_pkg" plugin="my_plugin" name="my_node"/>
            </load_composable_node>
        "#;

        let doc = Document::parse(xml).unwrap();
        let entity = XmlEntity::new(doc.root_element());
        let context = LaunchContext::new();

        let action = LoadComposableNodeAction::from_entity(&entity, &context).unwrap();
        assert_eq!(action.composable_nodes.len(), 1);
        assert_eq!(action.composable_nodes[0].name, "my_node");
    }

    #[test]
    fn test_parse_load_composable_node_missing_target() {
        let xml = r#"
            <load_composable_node>
                <composable_node pkg="my_pkg" plugin="my_plugin" name="my_node"/>
            </load_composable_node>
        "#;

        let doc = Document::parse(xml).unwrap();
        let entity = XmlEntity::new(doc.root_element());
        let context = LaunchContext::new();

        let result = LoadComposableNodeAction::from_entity(&entity, &context);
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_load_composable_node_with_substitution() {
        let xml = r#"
            <load_composable_node target="$(var container_name)">
                <composable_node pkg="my_pkg" plugin="my_plugin" name="my_node"/>
            </load_composable_node>
        "#;

        let doc = Document::parse(xml).unwrap();
        let entity = XmlEntity::new(doc.root_element());
        let mut context = LaunchContext::new();
        context.set_configuration("container_name".to_string(), "/test/container".to_string());

        let action = LoadComposableNodeAction::from_entity(&entity, &context).unwrap();
        let records = action.to_load_node_records(&context).unwrap();

        assert_eq!(records.len(), 1);
        assert_eq!(records[0].target_container_name, "/test/container");
    }

    #[test]
    fn test_to_load_node_records_absolute_target() {
        let xml = r#"
            <load_composable_node target="/control/control_container">
                <composable_node pkg="pkg1" plugin="plugin1" name="node1"/>
                <composable_node pkg="pkg2" plugin="plugin2" name="node2"/>
            </load_composable_node>
        "#;

        let doc = Document::parse(xml).unwrap();
        let entity = XmlEntity::new(doc.root_element());
        let context = LaunchContext::new();

        let action = LoadComposableNodeAction::from_entity(&entity, &context).unwrap();
        let records = action.to_load_node_records(&context).unwrap();

        assert_eq!(records.len(), 2);
        assert_eq!(
            records[0].target_container_name,
            "/control/control_container"
        );
        assert_eq!(records[0].node_name, "node1");
        assert_eq!(records[0].namespace, "/control");
        assert_eq!(
            records[1].target_container_name,
            "/control/control_container"
        );
        assert_eq!(records[1].node_name, "node2");
        assert_eq!(records[1].namespace, "/control");
    }

    #[test]
    fn test_to_load_node_records_relative_target() {
        let xml = r#"
            <load_composable_node target="my_container">
                <composable_node pkg="pkg1" plugin="plugin1" name="node1"/>
            </load_composable_node>
        "#;

        let doc = Document::parse(xml).unwrap();
        let entity = XmlEntity::new(doc.root_element());
        let mut context = LaunchContext::new();
        context.push_namespace("test_ns".to_string());

        let action = LoadComposableNodeAction::from_entity(&entity, &context).unwrap();
        let records = action.to_load_node_records(&context).unwrap();

        assert_eq!(records.len(), 1);
        assert_eq!(records[0].target_container_name, "/test_ns/my_container");
        assert_eq!(records[0].namespace, "/test_ns");
    }

    #[test]
    fn test_to_load_node_records_with_node_namespace() {
        let xml = r#"
            <load_composable_node target="/control/control_container">
                <composable_node pkg="pkg1" plugin="plugin1" name="node1" namespace="/custom_ns"/>
            </load_composable_node>
        "#;

        let doc = Document::parse(xml).unwrap();
        let entity = XmlEntity::new(doc.root_element());
        let context = LaunchContext::new();

        let action = LoadComposableNodeAction::from_entity(&entity, &context).unwrap();
        let records = action.to_load_node_records(&context).unwrap();

        assert_eq!(records.len(), 1);
        assert_eq!(records[0].namespace, "/custom_ns");
    }
}
