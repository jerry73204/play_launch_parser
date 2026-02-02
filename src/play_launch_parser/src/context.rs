//! Parse context for launch file parsing
//!
//! Replaces global state with a local context object that's threaded through
//! the parsing pipeline. Each parse operation gets its own isolated context,
//! preventing state leakage and accumulation bugs.

use pyo3::{prelude::*, types::PyDict};
use std::collections::HashMap;

use crate::{
    error::Result,
    python::bridge::{ContainerCapture, IncludeCapture, LoadNodeCapture, NodeCapture},
    record::RecordJson,
};

/// Context for parsing a single launch file tree
///
/// Contains all state needed during parsing. Each parse operation
/// gets its own context, eliminating global state and enabling
/// parallel parsing.
///
/// # Example
///
/// ```no_run
/// use play_launch_parser::ParseContext;
/// use std::collections::HashMap;
///
/// let args = HashMap::new();
/// let mut context = ParseContext::new(args);
///
/// // Use context during parsing
/// context.set_configuration("arg1".to_string(), "value1".to_string());
/// context.push_namespace("/ns1".to_string());
/// // ... parsing operations ...
///
/// // Convert to final result
/// let record = context.to_record_json()?;
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
#[derive(Debug)]
pub struct ParseContext {
    /// Launch configuration variables (arguments)
    launch_configurations: HashMap<String, String>,

    /// ROS namespace stack for push/pop operations
    ros_namespace_stack: Vec<String>,

    /// Captured node data from Python/XML
    captured_nodes: Vec<NodeCapture>,

    /// Captured container data
    captured_containers: Vec<ContainerCapture>,

    /// Captured composable node load operations
    captured_load_nodes: Vec<LoadNodeCapture>,

    /// Captured include operations
    captured_includes: Vec<IncludeCapture>,

    /// Environment variables (optional, defaults to std::env)
    environment: Option<HashMap<String, String>>,
}

impl ParseContext {
    /// Create new context with launch arguments
    pub fn new(args: HashMap<String, String>) -> Self {
        Self {
            launch_configurations: args,
            ros_namespace_stack: vec!["".to_string()], // Start with empty root namespace
            captured_nodes: Vec::new(),
            captured_containers: Vec::new(),
            captured_load_nodes: Vec::new(),
            captured_includes: Vec::new(),
            environment: None, // Use std::env by default
        }
    }

    // ========== Launch Configuration Methods ==========

    /// Get a launch configuration value by name
    pub fn get_configuration(&self, name: &str) -> Option<&String> {
        self.launch_configurations.get(name)
    }

    /// Set a launch configuration value
    pub fn set_configuration(&mut self, name: String, value: String) {
        self.launch_configurations.insert(name, value);
    }

    /// Get all launch configurations
    pub fn configurations(&self) -> &HashMap<String, String> {
        &self.launch_configurations
    }

    /// Get mutable reference to all launch configurations
    pub fn configurations_mut(&mut self) -> &mut HashMap<String, String> {
        &mut self.launch_configurations
    }

    // ========== Namespace Methods ==========

    /// Push a namespace onto the stack
    ///
    /// If the namespace is absolute (starts with '/'), it's used as-is.
    /// If relative, it's appended to the current namespace.
    pub fn push_namespace(&mut self, namespace: String) {
        let trimmed = namespace.trim();

        if trimmed.is_empty() || trimmed == "/" {
            // Empty or root namespace - don't change the stack
            return;
        }

        // Normalize the namespace (ensure leading /, no trailing /)
        let normalized = if trimmed.starts_with('/') {
            trimmed.trim_end_matches('/').to_string()
        } else {
            format!("/{}", trimmed.trim_end_matches('/'))
        };

        if normalized.is_empty() {
            return;
        }

        self.ros_namespace_stack.push(normalized);
    }

    /// Pop a namespace from the stack
    ///
    /// Returns the popped namespace, or None if we're at the root
    pub fn pop_namespace(&mut self) -> Option<String> {
        // Never pop the root namespace (first element)
        if self.ros_namespace_stack.len() > 1 {
            self.ros_namespace_stack.pop()
        } else {
            None
        }
    }

    /// Get the current namespace (concatenation of stack)
    pub fn current_namespace(&self) -> String {
        let result = self.ros_namespace_stack.join("");
        if result.is_empty() {
            "/".to_string()
        } else {
            result
        }
    }

    /// Get the current namespace depth
    pub fn namespace_depth(&self) -> usize {
        self.ros_namespace_stack.len()
    }

    /// Set the namespace stack directly (for save/restore operations)
    pub fn set_namespace_stack(&mut self, stack: Vec<String>) {
        self.ros_namespace_stack = stack;
    }

    /// Get a clone of the namespace stack (for save/restore operations)
    pub fn namespace_stack(&self) -> Vec<String> {
        self.ros_namespace_stack.clone()
    }

    // ========== Capture Methods ==========

    /// Capture a node definition
    pub fn capture_node(&mut self, node: NodeCapture) {
        self.captured_nodes.push(node);
    }

    /// Capture a container definition
    pub fn capture_container(&mut self, container: ContainerCapture) {
        self.captured_containers.push(container);
    }

    /// Capture a composable node load operation
    pub fn capture_load_node(&mut self, load_node: LoadNodeCapture) {
        self.captured_load_nodes.push(load_node);
    }

    /// Capture an include operation
    pub fn capture_include(&mut self, include: IncludeCapture) {
        self.captured_includes.push(include);
    }

    /// Get captured nodes
    pub fn captured_nodes(&self) -> &[NodeCapture] {
        &self.captured_nodes
    }

    /// Get captured containers
    pub fn captured_containers(&self) -> &[ContainerCapture] {
        &self.captured_containers
    }

    /// Get captured load nodes
    pub fn captured_load_nodes(&self) -> &[LoadNodeCapture] {
        &self.captured_load_nodes
    }

    /// Get captured includes
    pub fn captured_includes(&self) -> &[IncludeCapture] {
        &self.captured_includes
    }

    /// Get mutable reference to captured nodes
    pub fn captured_nodes_mut(&mut self) -> &mut Vec<NodeCapture> {
        &mut self.captured_nodes
    }

    /// Get mutable reference to captured containers
    pub fn captured_containers_mut(&mut self) -> &mut Vec<ContainerCapture> {
        &mut self.captured_containers
    }

    /// Get mutable reference to captured load nodes
    pub fn captured_load_nodes_mut(&mut self) -> &mut Vec<LoadNodeCapture> {
        &mut self.captured_load_nodes
    }

    /// Get mutable reference to captured includes
    pub fn captured_includes_mut(&mut self) -> &mut Vec<IncludeCapture> {
        &mut self.captured_includes
    }

    // ========== Environment Methods ==========

    /// Get an environment variable
    ///
    /// First checks the optional environment map, then falls back to std::env
    pub fn get_env(&self, name: &str) -> Option<String> {
        if let Some(ref env_map) = self.environment {
            env_map.get(name).cloned()
        } else {
            std::env::var(name).ok()
        }
    }

    /// Set an environment variable (in the local context only)
    pub fn set_env(&mut self, name: String, value: String) {
        self.environment
            .get_or_insert_with(HashMap::new)
            .insert(name, value);
    }

    // ========== PyO3 Conversion Methods ==========

    /// Convert launch configurations to a Python dictionary
    ///
    /// Used when passing context to Python code
    pub fn launch_configurations_as_pydict<'py>(&self, py: Python<'py>) -> PyResult<&'py PyDict> {
        let dict = PyDict::new(py);
        for (k, v) in &self.launch_configurations {
            dict.set_item(k, v)?;
        }
        Ok(dict)
    }

    // ========== Conversion to RecordJson ==========

    /// Convert captured data to RecordJson
    ///
    /// This consumes the context and converts all captured entities
    /// to their final record format.
    pub fn to_record_json(self) -> Result<RecordJson> {
        // Convert captures to records
        let nodes: Result<Vec<_>> = self
            .captured_nodes
            .into_iter()
            .map(|n| n.to_record())
            .collect();

        let containers: Result<Vec<_>> = self
            .captured_containers
            .into_iter()
            .map(|c| c.to_record())
            .collect();

        let load_nodes: Result<Vec<_>> = self
            .captured_load_nodes
            .into_iter()
            .map(|ln| ln.to_record())
            .collect();

        // Build the final record
        Ok(RecordJson {
            node: nodes?,
            container: containers?,
            load_node: load_nodes?,
            lifecycle_node: Vec::new(),
            variables: self.launch_configurations,
            file_data: HashMap::new(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_context() {
        let args: HashMap<String, String> = [("key".to_string(), "value".to_string())]
            .into_iter()
            .collect();
        let ctx = ParseContext::new(args);

        assert_eq!(ctx.get_configuration("key"), Some(&"value".to_string()));
        assert_eq!(ctx.current_namespace(), "/");
    }

    #[test]
    fn test_set_configuration() {
        let mut ctx = ParseContext::new(HashMap::new());

        ctx.set_configuration("arg1".to_string(), "val1".to_string());
        assert_eq!(ctx.get_configuration("arg1"), Some(&"val1".to_string()));

        ctx.set_configuration("arg2".to_string(), "val2".to_string());
        assert_eq!(ctx.get_configuration("arg2"), Some(&"val2".to_string()));
    }

    #[test]
    fn test_namespace_stack_simple() {
        let mut ctx = ParseContext::new(HashMap::new());

        ctx.push_namespace("/ns1".to_string());
        assert_eq!(ctx.current_namespace(), "/ns1");

        ctx.push_namespace("/ns2".to_string());
        assert_eq!(ctx.current_namespace(), "/ns1/ns2");

        ctx.pop_namespace();
        assert_eq!(ctx.current_namespace(), "/ns1");

        ctx.pop_namespace();
        assert_eq!(ctx.current_namespace(), "/");
    }

    #[test]
    fn test_namespace_stack_cant_pop_root() {
        let mut ctx = ParseContext::new(HashMap::new());

        assert_eq!(ctx.current_namespace(), "/");
        assert_eq!(ctx.pop_namespace(), None); // Can't pop root
        assert_eq!(ctx.current_namespace(), "/");
    }

    #[test]
    fn test_namespace_normalization() {
        let mut ctx = ParseContext::new(HashMap::new());

        // Leading slash is preserved
        ctx.push_namespace("/ns1".to_string());
        assert_eq!(ctx.current_namespace(), "/ns1");

        // Trailing slashes are removed
        ctx.push_namespace("/ns2/".to_string());
        assert_eq!(ctx.current_namespace(), "/ns1/ns2");

        // Empty and "/" are ignored
        ctx.push_namespace("".to_string());
        assert_eq!(ctx.current_namespace(), "/ns1/ns2");

        ctx.push_namespace("/".to_string());
        assert_eq!(ctx.current_namespace(), "/ns1/ns2");
    }

    #[test]
    fn test_namespace_depth() {
        let mut ctx = ParseContext::new(HashMap::new());

        assert_eq!(ctx.namespace_depth(), 1); // Root

        ctx.push_namespace("/ns1".to_string());
        assert_eq!(ctx.namespace_depth(), 2);

        ctx.push_namespace("/ns2".to_string());
        assert_eq!(ctx.namespace_depth(), 3);

        ctx.pop_namespace();
        assert_eq!(ctx.namespace_depth(), 2);
    }

    #[test]
    fn test_capture_node() {
        let mut ctx = ParseContext::new(HashMap::new());

        let node = NodeCapture {
            package: "pkg".to_string(),
            executable: "exec".to_string(),
            name: Some("node1".to_string()),
            namespace: Some("/ns".to_string()),
            parameters: Vec::new(),
            params_files: Vec::new(),
            remappings: Vec::new(),
            arguments: Vec::new(),
            env_vars: Vec::new(),
        };

        ctx.capture_node(node);
        assert_eq!(ctx.captured_nodes().len(), 1);
        assert_eq!(ctx.captured_nodes()[0].package, "pkg");
    }

    #[test]
    fn test_capture_multiple_entities() {
        let mut ctx = ParseContext::new(HashMap::new());

        // Capture 2 nodes
        ctx.capture_node(NodeCapture {
            package: "pkg1".to_string(),
            executable: "exec1".to_string(),
            name: None,
            namespace: None,
            parameters: Vec::new(),
            params_files: Vec::new(),
            remappings: Vec::new(),
            arguments: Vec::new(),
            env_vars: Vec::new(),
        });

        ctx.capture_node(NodeCapture {
            package: "pkg2".to_string(),
            executable: "exec2".to_string(),
            name: None,
            namespace: None,
            parameters: Vec::new(),
            params_files: Vec::new(),
            remappings: Vec::new(),
            arguments: Vec::new(),
            env_vars: Vec::new(),
        });

        assert_eq!(ctx.captured_nodes().len(), 2);
    }

    #[test]
    fn test_environment_variables() {
        let mut ctx = ParseContext::new(HashMap::new());

        // Set local environment variable
        ctx.set_env("MY_VAR".to_string(), "my_value".to_string());
        assert_eq!(ctx.get_env("MY_VAR"), Some("my_value".to_string()));

        // Non-existent variable
        assert_eq!(ctx.get_env("NONEXISTENT"), None);
    }

    #[test]
    fn test_namespace_save_restore() {
        let mut ctx = ParseContext::new(HashMap::new());

        ctx.push_namespace("/ns1".to_string());
        ctx.push_namespace("/ns2".to_string());

        // Save the stack
        let saved = ctx.namespace_stack();
        assert_eq!(saved.len(), 3); // ["", "/ns1", "/ns2"]

        // Modify the stack
        ctx.push_namespace("/ns3".to_string());
        assert_eq!(ctx.current_namespace(), "/ns1/ns2/ns3");

        // Restore
        ctx.set_namespace_stack(saved);
        assert_eq!(ctx.current_namespace(), "/ns1/ns2");
    }
}
