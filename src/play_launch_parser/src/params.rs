//! Parameter file loading

use crate::error::ParseError;
use serde_yaml::Value;
use std::fs;
use std::path::Path;

/// Load parameters from a YAML file
pub fn load_param_file(path: &Path) -> Result<Vec<(String, String)>, ParseError> {
    let content = fs::read_to_string(path)?;
    let yaml: Value = serde_yaml::from_str(&content)
        .map_err(|e| ParseError::InvalidSubstitution(format!("YAML parse error: {}", e)))?;

    let mut params = Vec::new();

    // ROS 2 parameter files have the structure:
    // node_name:
    //   ros__parameters:
    //     param1: value1
    //     param2: value2

    if let Value::Mapping(root_map) = yaml {
        // Iterate through each node in the YAML
        for (_node_name, node_value) in root_map.iter() {
            if let Value::Mapping(node_map) = node_value {
                // Look for ros__parameters key
                if let Some(Value::Mapping(params_map)) = node_map.get("ros__parameters") {
                    flatten_params("", params_map, &mut params);
                }
            }
        }
    }

    Ok(params)
}

/// Recursively flatten nested parameter maps
fn flatten_params(prefix: &str, map: &serde_yaml::Mapping, output: &mut Vec<(String, String)>) {
    for (key, value) in map.iter() {
        if let Value::String(key_str) = key {
            let full_key = if prefix.is_empty() {
                key_str.clone()
            } else {
                format!("{}.{}", prefix, key_str)
            };

            match value {
                Value::Mapping(nested_map) => {
                    // Nested parameters
                    flatten_params(&full_key, nested_map, output);
                }
                Value::String(s) => {
                    output.push((full_key, s.clone()));
                }
                Value::Number(n) => {
                    output.push((full_key, n.to_string()));
                }
                Value::Bool(b) => {
                    output.push((full_key, b.to_string()));
                }
                Value::Sequence(seq) => {
                    // Arrays get serialized as JSON
                    let json_str =
                        serde_json::to_string(seq).unwrap_or_else(|_| format!("{:?}", seq));
                    output.push((full_key, json_str));
                }
                Value::Null => {
                    output.push((full_key, "null".to_string()));
                }
                _ => {
                    // For other types, convert to string
                    output.push((full_key, format!("{:?}", value)));
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use tempfile::NamedTempFile;

    #[test]
    fn test_load_simple_params() {
        let yaml = r#"
my_node:
  ros__parameters:
    param1: "value1"
    param2: 42
    param3: true
"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(yaml.as_bytes()).unwrap();
        file.flush().unwrap();

        let params = load_param_file(file.path()).unwrap();
        assert_eq!(params.len(), 3);

        // Find specific parameters
        let param1 = params.iter().find(|(k, _)| k == "param1");
        assert_eq!(param1, Some(&("param1".to_string(), "value1".to_string())));

        let param2 = params.iter().find(|(k, _)| k == "param2");
        assert_eq!(param2, Some(&("param2".to_string(), "42".to_string())));

        let param3 = params.iter().find(|(k, _)| k == "param3");
        assert_eq!(param3, Some(&("param3".to_string(), "true".to_string())));
    }

    #[test]
    fn test_load_nested_params() {
        let yaml = r#"
my_node:
  ros__parameters:
    simple: "value"
    nested:
      param1: "nested_value"
      param2: 100
"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(yaml.as_bytes()).unwrap();
        file.flush().unwrap();

        let params = load_param_file(file.path()).unwrap();
        assert_eq!(params.len(), 3);

        let nested_param1 = params.iter().find(|(k, _)| k == "nested.param1");
        assert_eq!(
            nested_param1,
            Some(&("nested.param1".to_string(), "nested_value".to_string()))
        );

        let nested_param2 = params.iter().find(|(k, _)| k == "nested.param2");
        assert_eq!(
            nested_param2,
            Some(&("nested.param2".to_string(), "100".to_string()))
        );
    }

    #[test]
    fn test_load_array_params() {
        let yaml = r#"
my_node:
  ros__parameters:
    my_array: [1, 2, 3, 4]
"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(yaml.as_bytes()).unwrap();
        file.flush().unwrap();

        let params = load_param_file(file.path()).unwrap();
        assert_eq!(params.len(), 1);

        let array_param = params.iter().find(|(k, _)| k == "my_array");
        assert!(array_param.is_some());
        let (_, value) = array_param.unwrap();
        // Should be JSON array
        assert!(value.contains("[1,2,3,4]") || value.contains("[1, 2, 3, 4]"));
    }

    #[test]
    fn test_multiple_nodes() {
        let yaml = r#"
node1:
  ros__parameters:
    param1: "value1"

node2:
  ros__parameters:
    param2: "value2"
"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(yaml.as_bytes()).unwrap();
        file.flush().unwrap();

        let params = load_param_file(file.path()).unwrap();
        // Should load parameters from all nodes
        assert_eq!(params.len(), 2);
    }
}
