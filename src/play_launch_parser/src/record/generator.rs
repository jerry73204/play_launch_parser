//! Command-line and record generation

use crate::actions::{ExecutableAction, NodeAction};
use crate::error::GenerationError;
use crate::params::load_param_file;
use crate::record::types::NodeRecord;
use crate::substitution::{resolve_substitutions, LaunchContext};
use std::path::Path;

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
            // Node has explicit namespace
            Some(resolve_substitutions(ns_subs, context)?)
        } else {
            // Use namespace from context (group scoping)
            Some(context.current_namespace())
        };

        // Process inline parameters
        let mut params: Vec<(String, String)> = node
            .parameters
            .iter()
            .map(|p| {
                let resolved_value = resolve_substitutions(&p.value, context)?;
                Ok((p.name.clone(), resolved_value))
            })
            .collect::<Result<Vec<_>, GenerationError>>()?;

        // Process parameter files
        let mut params_files = Vec::new();
        for param_file_subs in &node.param_files {
            let param_file_path = resolve_substitutions(param_file_subs, context)?;
            params_files.push(param_file_path.clone());

            // Load parameters from file and merge with inline params
            match load_param_file(Path::new(&param_file_path)) {
                Ok(file_params) => {
                    params.extend(file_params);
                }
                Err(e) => {
                    log::warn!("Failed to load parameter file {}: {}", param_file_path, e);
                }
            }
        }

        let remaps: Result<Vec<_>, GenerationError> = node
            .remappings
            .iter()
            .map(|r| {
                let from = resolve_substitutions(&r.from, context)?;
                let to = resolve_substitutions(&r.to, context)?;
                Ok((from, to))
            })
            .collect();
        let remaps = remaps?;

        // Merge context environment with node-specific environment
        // Node-specific environment takes precedence
        let mut merged_env = context.environment().clone();
        for (key, value) in &node.environment {
            merged_env.insert(key.clone(), value.clone());
        }

        let env = if merged_env.is_empty() {
            None
        } else {
            Some(
                merged_env
                    .into_iter()
                    .map(|(k, v)| (k, v))
                    .collect::<Vec<_>>(),
            )
        };

        // Get global parameters from context
        let global_params = if context.global_parameters().is_empty() {
            None
        } else {
            Some(
                context
                    .global_parameters()
                    .iter()
                    .map(|(k, v)| (k.clone(), v.clone()))
                    .collect::<Vec<_>>(),
            )
        };

        Ok(NodeRecord {
            executable,
            package: Some(package),
            name,
            namespace,
            exec_name: None, // TODO: Generate exec_name-1, exec_name-2, etc.
            params,
            params_files,
            remaps,
            ros_args: None,
            args: None,
            cmd,
            env,
            respawn: Some(node.respawn),
            respawn_delay: node.respawn_delay,
            global_params,
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
            let from = resolve_substitutions(&remap.from, context)?;
            let to = resolve_substitutions(&remap.to, context)?;
            cmd.push("-r".to_string());
            cmd.push(format!("{}:={}", from, to));
        }

        // 6. Parameters
        for param in &node.parameters {
            let value = resolve_substitutions(&param.value, context)?;
            cmd.push("-p".to_string());
            cmd.push(format!("{}:={}", param.name, value));
        }

        Ok(cmd)
    }

    fn resolve_executable_path(package: &str, executable: &str) -> Result<String, GenerationError> {
        // Simplified for MVP: just construct path
        // TODO: Use ament_index for proper resolution
        Ok(format!("/opt/ros/humble/lib/{}/{}", package, executable))
    }

    pub fn generate_executable_record(
        exec: &ExecutableAction,
        context: &LaunchContext,
    ) -> Result<NodeRecord, GenerationError> {
        let cmd_str = resolve_substitutions(&exec.cmd, context)?;
        let mut cmd = vec![cmd_str.clone()];

        // Add arguments
        for arg in &exec.arguments {
            let arg_str = resolve_substitutions(arg, context)?;
            cmd.push(arg_str);
        }

        let name = if let Some(name_subs) = &exec.name {
            Some(resolve_substitutions(name_subs, context)?)
        } else {
            Some(cmd_str.clone())
        };

        // Merge context environment with executable-specific environment
        // Executable-specific environment takes precedence
        let mut merged_env = context.environment().clone();
        for (key, value) in &exec.environment {
            merged_env.insert(key.clone(), value.clone());
        }

        let env = if merged_env.is_empty() {
            None
        } else {
            Some(
                merged_env
                    .into_iter()
                    .map(|(k, v)| (k, v))
                    .collect::<Vec<_>>(),
            )
        };

        Ok(NodeRecord {
            executable: cmd_str,
            package: None, // Executables don't have packages
            name,
            namespace: Some("/".to_string()),
            exec_name: None,
            params: Vec::new(),
            params_files: Vec::new(),
            remaps: Vec::new(),
            ros_args: None,
            args: if exec.arguments.is_empty() {
                None
            } else {
                Some(
                    exec.arguments
                        .iter()
                        .map(|a| resolve_substitutions(a, context))
                        .collect::<Result<Vec<_>, _>>()?,
                )
            },
            cmd,
            env,
            respawn: None,
            respawn_delay: None,
            global_params: None,
        })
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
            param_files: vec![],
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
                value: vec![Substitution::Text("10.0".to_string())],
            }],
            param_files: vec![],
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
            param_files: vec![],
            remappings: vec![Remapping {
                from: vec![Substitution::Text("chatter".to_string())],
                to: vec![Substitution::Text("/chat".to_string())],
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
