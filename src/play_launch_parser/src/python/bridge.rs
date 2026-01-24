//! Bridge between Python and Rust types

use crate::error::Result;
use crate::record::{ComposableNodeContainerRecord, LoadNodeRecord, NodeRecord};
use once_cell::sync::Lazy;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

/// Global storage for launch configurations (arguments passed to the launch file)
/// This allows conditions to access and resolve LaunchConfiguration substitutions
pub static LAUNCH_CONFIGURATIONS: Lazy<Arc<Mutex<HashMap<String, String>>>> =
    Lazy::new(|| Arc::new(Mutex::new(HashMap::new())));

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
    /// Convert to NodeRecord
    pub fn to_record(&self) -> Result<NodeRecord> {
        Ok(NodeRecord {
            args: if self.arguments.is_empty() {
                None
            } else {
                Some(self.arguments.clone())
            },
            cmd: Vec::new(), // Will be generated later
            env: if self.env_vars.is_empty() {
                None
            } else {
                Some(self.env_vars.clone())
            },
            exec_name: None,
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
            ros_args: None,
        })
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
}

/// Global storage for captured includes (thread-safe)
pub static CAPTURED_INCLUDES: Lazy<Arc<Mutex<Vec<IncludeCapture>>>> =
    Lazy::new(|| Arc::new(Mutex::new(Vec::new())));

/// Clear all captured global state
///
/// This should be called at the start of each parse_launch_file call to prevent
/// test contamination when running multiple tests in sequence.
/// Note: LAUNCH_CONFIGURATIONS is managed separately by the executor and should
/// not be cleared here.
#[cfg(feature = "python")]
pub fn clear_all_captured() {
    CAPTURED_NODES.lock().unwrap().clear();
    CAPTURED_CONTAINERS.lock().unwrap().clear();
    CAPTURED_LOAD_NODES.lock().unwrap().clear();
    CAPTURED_INCLUDES.lock().unwrap().clear();
}
