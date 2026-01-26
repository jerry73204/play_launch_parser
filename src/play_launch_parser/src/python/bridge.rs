//! Bridge between Python and Rust types

use crate::error::Result;
use crate::record::{ComposableNodeContainerRecord, LoadNodeRecord, NodeRecord};
use once_cell::sync::Lazy;
use parking_lot::Mutex;
use std::collections::HashMap;
use std::sync::Arc;

/// Global storage for launch configurations (arguments passed to the launch file)
/// This allows conditions to access and resolve LaunchConfiguration substitutions
pub static LAUNCH_CONFIGURATIONS: Lazy<Arc<Mutex<HashMap<String, String>>>> =
    Lazy::new(|| Arc::new(Mutex::new(HashMap::new())));

/// Global ROS namespace stack
/// Tracks the current ROS namespace hierarchy from PushRosNamespace/PopRosNamespace actions
pub static ROS_NAMESPACE_STACK: Lazy<Arc<Mutex<Vec<String>>>> =
    Lazy::new(|| Arc::new(Mutex::new(vec!["".to_string()])));

/// Captured node data from Python
#[derive(Debug, Clone)]
pub struct NodeCapture {
    pub package: String,
    pub executable: String,
    pub name: Option<String>,
    pub namespace: Option<String>,
    pub parameters: Vec<(String, String)>,
    pub remappings: Vec<(String, String)>,
    pub arguments: Vec<String>,
    pub env_vars: Vec<(String, String)>,
}

impl NodeCapture {
    /// Convert to NodeRecord and generate command line
    pub fn to_record(&self) -> Result<NodeRecord> {
        // Generate ROS command line
        let cmd = self.generate_command();

        // Extract ROS args (everything after --ros-args)
        let ros_args = cmd
            .iter()
            .position(|s| s == "--ros-args")
            .map(|pos| cmd[pos + 1..].to_vec());

        Ok(NodeRecord {
            args: if self.arguments.is_empty() {
                None
            } else {
                Some(self.arguments.clone())
            },
            cmd,
            env: if self.env_vars.is_empty() {
                None
            } else {
                Some(self.env_vars.clone())
            },
            exec_name: self.name.clone(), // Match Python: exec_name = node name
            executable: self.executable.clone(),
            global_params: None,
            name: self.name.clone(),
            namespace: self.namespace.clone(),
            package: Some(self.package.clone()),
            params: self.parameters.clone(),
            params_files: Vec::new(),
            remaps: self.remappings.clone(),
            respawn: None,
            respawn_delay: None,
            ros_args,
        })
    }

    /// Generate ROS 2 command line
    fn generate_command(&self) -> Vec<String> {
        // 1. Base command: ros2 run <package> <executable>
        let mut cmd = vec![
            "ros2".to_string(),
            "run".to_string(),
            self.package.clone(),
            self.executable.clone(),
        ];

        // 2. Add custom arguments if any
        if !self.arguments.is_empty() {
            cmd.extend(self.arguments.clone());
        }

        // 3. ROS args delimiter
        cmd.push("--ros-args".to_string());

        // 4. Node name
        if let Some(ref name) = self.name {
            cmd.push("-r".to_string());
            cmd.push(format!("__node:={}", name));
        }

        // 5. Namespace
        if let Some(ref ns) = self.namespace {
            if !ns.is_empty() && ns != "/" {
                cmd.push("-r".to_string());
                cmd.push(format!("__ns:={}", ns));
            }
        }

        // 6. Remappings
        for (from, to) in &self.remappings {
            cmd.push("-r".to_string());
            cmd.push(format!("{}:={}", from, to));
        }

        // 7. Parameters
        for (name, value) in &self.parameters {
            cmd.push("-p".to_string());
            cmd.push(format!("{}:={}", name, value));
        }

        cmd
    }
}

/// Global storage for captured nodes (thread-safe)
pub static CAPTURED_NODES: Lazy<Arc<Mutex<Vec<NodeCapture>>>> =
    Lazy::new(|| Arc::new(Mutex::new(Vec::new())));

/// Captured container data from Python
#[derive(Debug, Clone)]
pub struct ContainerCapture {
    pub name: String,
    pub namespace: String,
}

impl ContainerCapture {
    /// Convert to ComposableNodeContainerRecord
    pub fn to_record(&self) -> Result<ComposableNodeContainerRecord> {
        Ok(ComposableNodeContainerRecord {
            name: self.name.clone(),
            namespace: self.namespace.clone(),
        })
    }
}

/// Global storage for captured containers (thread-safe)
pub static CAPTURED_CONTAINERS: Lazy<Arc<Mutex<Vec<ContainerCapture>>>> =
    Lazy::new(|| Arc::new(Mutex::new(Vec::new())));

/// Captured composable node data from Python
#[derive(Debug, Clone)]
pub struct LoadNodeCapture {
    pub package: String,
    pub plugin: String,
    pub target_container_name: String,
    pub node_name: String,
    pub namespace: String,
    pub parameters: Vec<(String, String)>,
    pub remappings: Vec<(String, String)>,
}

impl LoadNodeCapture {
    /// Convert to LoadNodeRecord
    pub fn to_record(&self) -> Result<LoadNodeRecord> {
        Ok(LoadNodeRecord {
            package: self.package.clone(),
            plugin: self.plugin.clone(),
            target_container_name: self.target_container_name.clone(),
            node_name: self.node_name.clone(),
            namespace: self.namespace.clone(),
            log_level: None,
            remaps: self.remappings.clone(),
            params: self.parameters.clone(),
            extra_args: HashMap::new(),
            env: None,
        })
    }
}

/// Global storage for captured load_nodes (thread-safe)
pub static CAPTURED_LOAD_NODES: Lazy<Arc<Mutex<Vec<LoadNodeCapture>>>> =
    Lazy::new(|| Arc::new(Mutex::new(Vec::new())));

/// Captured include data from Python
#[derive(Debug, Clone)]
pub struct IncludeCapture {
    pub file_path: String,
    pub args: Vec<(String, String)>,
    pub ros_namespace: String,
}

/// Global storage for captured includes (thread-safe)
pub static CAPTURED_INCLUDES: Lazy<Arc<Mutex<Vec<IncludeCapture>>>> =
    Lazy::new(|| Arc::new(Mutex::new(Vec::new())));

/// Push a namespace onto the ROS namespace stack
pub fn push_ros_namespace(namespace: String) {
    let normalized = if namespace.is_empty() {
        String::new()
    } else if namespace.starts_with('/') {
        namespace
    } else {
        format!("/{}", namespace)
    };

    log::debug!("Pushing ROS namespace: '{}'", normalized);
    ROS_NAMESPACE_STACK.lock().push(normalized);
}

/// Pop a namespace from the ROS namespace stack
pub fn pop_ros_namespace() {
    let mut stack = ROS_NAMESPACE_STACK.lock();
    if stack.len() > 1 {
        let popped = stack.pop();
        log::debug!("Popped ROS namespace: {:?}", popped);
    } else {
        log::warn!("Attempted to pop from empty namespace stack");
    }
}

/// Get the current ROS namespace (concatenation of all namespaces in the stack)
pub fn get_current_ros_namespace() -> String {
    let stack = ROS_NAMESPACE_STACK.lock();
    let result = stack.join("");
    if result.is_empty() {
        "/".to_string()
    } else {
        result
    }
}

/// Clear all captured global state
///
/// This should be called at the start of each parse_launch_file call to prevent
/// test contamination when running multiple tests in sequence.
/// Note: LAUNCH_CONFIGURATIONS is managed separately by the executor and should
/// not be cleared here.
pub fn clear_all_captured() {
    CAPTURED_NODES.lock().clear();
    CAPTURED_CONTAINERS.lock().clear();
    CAPTURED_LOAD_NODES.lock().clear();
    CAPTURED_INCLUDES.lock().clear();

    // Reset namespace stack to root
    let mut stack = ROS_NAMESPACE_STACK.lock();
    stack.clear();
    stack.push("".to_string());
}
