//! play_launch_parser library

pub mod actions;
pub mod condition;
pub mod error;
pub mod record;
pub mod substitution;
pub mod xml;

use actions::{ArgAction, ExecutableAction, GroupAction, IncludeAction, LetAction, NodeAction};
use condition::should_process_entity;
use error::{ParseError, Result};
use record::{CommandGenerator, RecordJson};
use std::collections::HashMap;
use std::path::Path;
use substitution::{resolve_substitutions, LaunchContext};
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
        // Set current file in context
        self.context.set_current_file(path.to_path_buf());

        let content = std::fs::read_to_string(path)?;
        let doc = roxmltree::Document::parse(&content)?;
        let root = xml::XmlEntity::new(doc.root_element());
        self.traverse_entity(&root)?;
        Ok(())
    }

    fn process_include(&mut self, include: &IncludeAction) -> Result<()> {
        // Resolve the file path
        let file_path_str = resolve_substitutions(&include.file, &self.context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
        let file_path = Path::new(&file_path_str);

        log::info!("Including launch file: {}", file_path.display());

        // Create a new context for the included file
        // Start with current context and apply include args
        let mut include_context = self.context.clone();
        include_context.set_current_file(file_path.to_path_buf());
        for (key, value) in &include.args {
            include_context.set_configuration(key.clone(), value.clone());
        }

        // Parse and traverse the included file
        let content = std::fs::read_to_string(file_path)?;
        let doc = roxmltree::Document::parse(&content)?;
        let root = xml::XmlEntity::new(doc.root_element());

        // Create temporary traverser for included file
        let mut included_traverser = LaunchTraverser {
            context: include_context,
            records: Vec::new(),
        };
        included_traverser.traverse_entity(&root)?;

        // Merge records from included file into current records
        self.records.extend(included_traverser.records);

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
                let _group = GroupAction::from_entity(entity)?;
                // For now, simply traverse children
                // TODO: Implement namespace scoping
                for child in entity.children() {
                    self.traverse_entity(&child)?;
                }
            }
            "let" => {
                let let_action = LetAction::from_entity(entity)?;
                // Set variable in context (acts like arg)
                self.context
                    .set_configuration(let_action.name, let_action.value);
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
}
