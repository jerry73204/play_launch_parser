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
    ArgAction, DeclareArgumentAction, ExecutableAction, GroupAction, IncludeAction, LetAction,
    NodeAction, SetEnvAction, SetParameterAction, UnsetEnvAction,
};
use condition::should_process_entity;
use error::{ParseError, Result};
use record::{CommandGenerator, RecordJson};
use std::collections::HashMap;
use std::path::Path;
use substitution::{parse_substitutions, resolve_substitutions, ArgumentMetadata, LaunchContext};
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
                    log::info!("Skipping YAML configuration file: {}", path.display());
                    // YAML files are configuration files, not launch files
                    return Ok(());
                }
                _ => {}
            }
        }

        // Set current file in context
        self.context.set_current_file(path.to_path_buf());

        let content = std::fs::read_to_string(path)?;
        let doc = roxmltree::Document::parse(&content)?;
        let root = xml::XmlEntity::new(doc.root_element());
        self.traverse_entity(&root)?;
        Ok(())
    }

    #[cfg(feature = "python")]
    fn execute_python_file(&mut self, path: &Path, args: &HashMap<String, String>) -> Result<()> {
        use crate::python::PythonLaunchExecutor;

        let executor =
            PythonLaunchExecutor::new().map_err(|e| ParseError::PythonError(e.to_string()))?;
        let nodes = executor.execute_launch_file(path, args)?;

        log::info!("Captured {} nodes from Python launch file", nodes.len());
        self.records.extend(nodes);

        Ok(())
    }

    fn process_include(&mut self, include: &IncludeAction) -> Result<()> {
        // Resolve the file path
        let file_path_str = resolve_substitutions(&include.file, &self.context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
        let file_path = Path::new(&file_path_str);

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
                    log::info!(
                        "Skipping YAML configuration file: {}",
                        resolved_path.display()
                    );
                    // YAML files are configuration files, not launch files
                    // They are typically loaded as parameter files, not executed
                    return Ok(());
                }
                _ => {}
            }
        }

        // Create a new context for the included file
        // Start with current context and apply include args
        let mut include_context = self.context.clone();
        include_context.set_current_file(resolved_path.clone());
        for (key, value) in &include.args {
            // Resolve substitutions in the argument value using the parent context
            let resolved_value_subs = parse_substitutions(value)?;
            let resolved_value = resolve_substitutions(&resolved_value_subs, &self.context)
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
            include_context.set_configuration(key.clone(), resolved_value);
        }

        // Parse and traverse the included file
        let content = std::fs::read_to_string(&resolved_path)?;
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

                // Push namespace if specified
                if let Some(ns_subs) = &group.namespace {
                    let namespace = resolve_substitutions(ns_subs, &self.context)
                        .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                    self.context.push_namespace(namespace);
                }

                // Traverse children with scoped namespace
                for child in entity.children() {
                    self.traverse_entity(&child)?;
                }

                // Pop namespace if we pushed one
                if group.namespace.is_some() {
                    self.context.pop_namespace();
                }
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
                // Parse node container similar to regular node
                // Containers are special nodes that load composable node plugins
                // NodeAction will handle composable_node children gracefully
                let node_action = NodeAction::from_entity(entity)?;
                let record = CommandGenerator::generate_node_record(&node_action, &self.context)
                    .map_err(|e| ParseError::IoError(std::io::Error::other(e.to_string())))?;
                self.records.push(record);
            }
            "composable_node" | "composable-node" => {
                // Composable nodes are loaded into containers
                // For now, we log and skip them when they appear standalone
                log::info!("Skipping standalone composable_node (should be in node_container)");
            }
            "load_composable_node" | "load-composable-node" => {
                // Action to dynamically load composable nodes
                // For now, we log and skip
                log::info!("Skipping load_composable_node action (not yet supported)");
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
