//! Command-line and record generation

use crate::actions::NodeAction;
use crate::error::GenerationError;
use crate::record::types::NodeRecord;
use crate::substitution::{resolve_substitutions, LaunchContext};

pub struct CommandGenerator;

impl CommandGenerator {
    pub fn generate_node_record(
        node: &NodeAction,
        context: &LaunchContext,
    ) -> Result<NodeRecord, GenerationError> {
        let cmd = Self::generate_node_command(node, context)?;

        let package = resolve_substitutions(&node.package, context)?;
        let executable = resolve_substitutions(&node.executable, context)?;

        let name = if let Some(name_subs) = &node.name {
            Some(resolve_substitutions(name_subs, context)?)
        } else {
            Some(executable.clone())
        };

        let namespace = if let Some(ns_subs) = &node.namespace {
            Some(resolve_substitutions(ns_subs, context)?)
        } else {
            Some("/".to_string())
        };

        let params = node
            .parameters
            .iter()
            .map(|p| (p.name.clone(), p.value.clone()))
            .collect();

        let remaps = node
            .remappings
            .iter()
            .map(|r| (r.from.clone(), r.to.clone()))
            .collect();

        let env = if node.environment.is_empty() {
            None
        } else {
            Some(node.environment.clone())
        };

        Ok(NodeRecord {
            executable,
            package: Some(package),
            name,
            namespace,
            exec_name: None, // TODO: Generate exec_name-1, exec_name-2, etc.
            params,
            params_files: Vec::new(), // TODO: Phase 3
            remaps,
            ros_args: None,
            args: None,
            cmd,
            env,
            respawn: Some(node.respawn),
            respawn_delay: node.respawn_delay,
            global_params: None, // TODO: Phase 3
        })
    }

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
        // Simplified for MVP: just construct path
        // TODO: Use ament_index for proper resolution
        Ok(format!("/opt/ros/humble/lib/{}/{}", package, executable))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::actions::{Parameter, Remapping};
    use crate::substitution::{LaunchContext, Substitution};

    #[test]
    fn test_generate_simple_command() {
        let node = NodeAction {
            package: vec![Substitution::Text("demo_nodes_cpp".to_string())],
            executable: vec![Substitution::Text("talker".to_string())],
            name: None,
            namespace: None,
            parameters: vec![],
            remappings: vec![],
            environment: vec![],
            output: None,
            respawn: false,
            respawn_delay: None,
        };

        let context = LaunchContext::new();
        let cmd = CommandGenerator::generate_node_command(&node, &context).unwrap();

        assert_eq!(cmd[0], "/opt/ros/humble/lib/demo_nodes_cpp/talker");
        assert_eq!(cmd[1], "--ros-args");
        assert!(cmd.contains(&"-r".to_string()));
        assert!(cmd.contains(&"__node:=talker".to_string()));
        assert!(cmd.contains(&"__ns:=/".to_string()));
    }

    #[test]
    fn test_generate_command_with_params() {
        let node = NodeAction {
            package: vec![Substitution::Text("demo".to_string())],
            executable: vec![Substitution::Text("node".to_string())],
            name: None,
            namespace: None,
            parameters: vec![Parameter {
                name: "rate".to_string(),
                value: "10.0".to_string(),
            }],
            remappings: vec![],
            environment: vec![],
            output: None,
            respawn: false,
            respawn_delay: None,
        };

        let context = LaunchContext::new();
        let cmd = CommandGenerator::generate_node_command(&node, &context).unwrap();

        assert!(cmd.contains(&"-p".to_string()));
        assert!(cmd.contains(&"rate:=10.0".to_string()));
    }

    #[test]
    fn test_generate_command_with_remaps() {
        let node = NodeAction {
            package: vec![Substitution::Text("demo".to_string())],
            executable: vec![Substitution::Text("node".to_string())],
            name: None,
            namespace: None,
            parameters: vec![],
            remappings: vec![Remapping {
                from: "chatter".to_string(),
                to: "/chat".to_string(),
            }],
            environment: vec![],
            output: None,
            respawn: false,
            respawn_delay: None,
        };

        let context = LaunchContext::new();
        let cmd = CommandGenerator::generate_node_command(&node, &context).unwrap();

        assert!(cmd.contains(&"-r".to_string()));
        assert!(cmd.contains(&"chatter:=/chat".to_string()));
    }
}
