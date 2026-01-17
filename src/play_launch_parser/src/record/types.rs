//! record.json data structures

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Root structure for record.json
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecordJson {
    pub node: Vec<NodeRecord>,
    pub container: Vec<ComposableNodeContainerRecord>,
    pub load_node: Vec<LoadNodeRecord>,
    pub lifecycle_node: Vec<String>,
    pub file_data: HashMap<String, String>,
}

impl RecordJson {
    pub fn new() -> Self {
        Self {
            node: Vec::new(),
            container: Vec::new(),
            load_node: Vec::new(),
            lifecycle_node: Vec::new(),
            file_data: HashMap::new(),
        }
    }

    pub fn to_json(&self) -> serde_json::Result<String> {
        serde_json::to_string_pretty(self)
    }
}

impl Default for RecordJson {
    fn default() -> Self {
        Self::new()
    }
}

/// Node record structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeRecord {
    pub executable: String,
    pub package: Option<String>,
    pub name: Option<String>,
    pub namespace: Option<String>,
    pub exec_name: Option<String>,
    pub params: Vec<(String, String)>,
    pub params_files: Vec<String>,
    pub remaps: Vec<(String, String)>,
    pub ros_args: Option<Vec<String>>,
    pub args: Option<Vec<String>>,
    pub cmd: Vec<String>,
    pub env: Option<Vec<(String, String)>>,
    pub respawn: Option<bool>,
    pub respawn_delay: Option<f64>,
    pub global_params: Option<Vec<(String, String)>>,
}

/// Composable node container record
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComposableNodeContainerRecord {
    pub name: String,
    pub namespace: String,
}

/// Load node record (for composable nodes)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoadNodeRecord {
    pub package: String,
    pub plugin: String,
    pub target_container_name: String,
    pub node_name: String,
    pub namespace: String,
    pub log_level: Option<String>,
    pub remaps: Vec<(String, String)>,
    pub params: Vec<(String, String)>,
    pub extra_args: HashMap<String, String>,
    pub env: Option<Vec<(String, String)>>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_record() {
        let record = RecordJson::new();
        assert_eq!(record.node.len(), 0);
        assert_eq!(record.container.len(), 0);
        assert_eq!(record.load_node.len(), 0);
        assert_eq!(record.lifecycle_node.len(), 0);
        assert_eq!(record.file_data.len(), 0);
    }

    #[test]
    fn test_serialize_empty() {
        let record = RecordJson::new();
        let json = record.to_json().unwrap();
        assert!(json.contains("\"node\""));
        assert!(json.contains("\"container\""));
        assert!(json.contains("\"load_node\""));
        assert!(json.contains("\"lifecycle_node\""));
        assert!(json.contains("\"file_data\""));
    }

    #[test]
    fn test_serialize_node_record() {
        let node = NodeRecord {
            executable: "talker".to_string(),
            package: Some("demo_nodes_cpp".to_string()),
            name: Some("/talker".to_string()),
            namespace: Some("/".to_string()),
            exec_name: Some("talker-1".to_string()),
            params: vec![("rate".to_string(), "10.0".to_string())],
            params_files: vec![],
            remaps: vec![("chatter".to_string(), "/chat".to_string())],
            ros_args: None,
            args: None,
            cmd: vec![
                "/path/to/talker".to_string(),
                "--ros-args".to_string(),
                "-r".to_string(),
                "__node:=talker".to_string(),
            ],
            env: None,
            respawn: Some(false),
            respawn_delay: None,
            global_params: None,
        };

        let json = serde_json::to_string(&node).unwrap();
        assert!(json.contains("\"executable\":\"talker\""));
        assert!(json.contains("\"package\":\"demo_nodes_cpp\""));
    }

    #[test]
    fn test_tuple_serialization() {
        let node = NodeRecord {
            executable: "node".to_string(),
            package: None,
            name: None,
            namespace: None,
            exec_name: None,
            params: vec![
                ("param1".to_string(), "value1".to_string()),
                ("param2".to_string(), "value2".to_string()),
            ],
            params_files: vec![],
            remaps: vec![],
            ros_args: None,
            args: None,
            cmd: vec![],
            env: None,
            respawn: None,
            respawn_delay: None,
            global_params: None,
        };

        let json = serde_json::to_string(&node).unwrap();
        // Tuples should serialize as arrays
        assert!(json.contains("[\"param1\",\"value1\"]"));
        assert!(json.contains("[\"param2\",\"value2\"]"));
    }
}
