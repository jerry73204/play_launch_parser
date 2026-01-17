//! play_launch_parser library

pub mod actions;
pub mod error;
pub mod record;
pub mod substitution;
pub mod xml;

use actions::{ArgAction, NodeAction};
use error::{ParseError, Result};
use record::{CommandGenerator, RecordJson};
use std::collections::HashMap;
use std::path::Path;
use substitution::LaunchContext;
use xml::{Entity, XmlEntity};

/// Launch tree traverser for parsing launch files
pub struct LaunchTraverser {
    context: LaunchContext,
    records: Vec<record::NodeRecord>,
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

    pub fn traverse_file(&mut self, path: &Path) -> Result<()> {
        let content = std::fs::read_to_string(path)?;
        let doc = roxmltree::Document::parse(&content)?;
        let root = xml::XmlEntity::new(doc.root_element());
        self.traverse_entity(&root)?;
        Ok(())
    }

    fn traverse_entity(&mut self, entity: &XmlEntity) -> Result<()> {
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
            "node" => {
                let node = NodeAction::from_entity(entity)?;
                let record = CommandGenerator::generate_node_record(&node, &self.context)
                    .map_err(|e| ParseError::IoError(std::io::Error::other(e.to_string())))?;
                self.records.push(record);
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
            container: Vec::new(),
            load_node: Vec::new(),
            lifecycle_node: Vec::new(),
            file_data: HashMap::new(),
        }
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
}
