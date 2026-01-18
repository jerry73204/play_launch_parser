//! Bridge between Python and Rust types

use crate::error::Result;
use crate::record::NodeRecord;
use once_cell::sync::Lazy;
use std::sync::{Arc, Mutex};

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
            executable: self.executable.clone(),
            package: Some(self.package.clone()),
            name: self.name.clone(),
            namespace: self.namespace.clone(),
            exec_name: None,
            params: self.parameters.clone(),
            params_files: Vec::new(),
            remaps: self.remappings.clone(),
            ros_args: None,
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
            respawn: None,
            respawn_delay: None,
            global_params: None,
        })
    }
}

/// Global storage for captured nodes (thread-safe)
pub static CAPTURED_NODES: Lazy<Arc<Mutex<Vec<NodeCapture>>>> =
    Lazy::new(|| Arc::new(Mutex::new(Vec::new())));
