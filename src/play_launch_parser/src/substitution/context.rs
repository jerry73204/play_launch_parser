//! Launch context for managing configurations

use std::collections::HashMap;
use std::path::PathBuf;

/// Launch context holding configurations and state
#[derive(Debug, Clone)]
pub struct LaunchContext {
    configurations: HashMap<String, String>,
    current_file: Option<PathBuf>,
    namespace_stack: Vec<String>,
}

impl LaunchContext {
    pub fn new() -> Self {
        Self {
            configurations: HashMap::new(),
            current_file: None,
            namespace_stack: vec!["/".to_string()], // Start with root namespace
        }
    }

    pub fn set_current_file(&mut self, path: PathBuf) {
        self.current_file = Some(path);
    }

    pub fn current_file(&self) -> Option<&PathBuf> {
        self.current_file.as_ref()
    }

    pub fn current_dir(&self) -> Option<PathBuf> {
        self.current_file
            .as_ref()
            .and_then(|p| p.parent().map(|p| p.to_path_buf()))
    }

    pub fn current_filename(&self) -> Option<String> {
        self.current_file
            .as_ref()
            .and_then(|p| p.file_name())
            .and_then(|n| n.to_str())
            .map(|s| s.to_string())
    }

    pub fn set_configuration(&mut self, name: String, value: String) {
        self.configurations.insert(name, value);
    }

    pub fn get_configuration(&self, name: &str) -> Option<String> {
        self.configurations.get(name).cloned()
    }

    pub fn configurations(&self) -> &HashMap<String, String> {
        &self.configurations
    }

    /// Push a namespace onto the stack
    pub fn push_namespace(&mut self, namespace: String) {
        let trimmed = namespace.trim();

        if trimmed.is_empty() || trimmed == "/" {
            // Empty or root namespace - don't change the stack
            return;
        }

        // Check if absolute (starts with /) BEFORE normalization
        let is_absolute = trimmed.starts_with('/');

        // Normalize the namespace
        let normalized = normalize_namespace(trimmed);

        if normalized.is_empty() || normalized == "/" {
            return;
        }

        // Get current namespace
        let current = self.current_namespace();

        // Combine namespaces
        let new_ns = if is_absolute {
            // Absolute namespace - use as-is
            normalized
        } else {
            // Relative namespace - append to current
            if current == "/" {
                format!("/{}", normalized)
            } else {
                format!("{}/{}", current, normalized)
            }
        };

        self.namespace_stack.push(new_ns);
    }

    /// Pop a namespace from the stack
    pub fn pop_namespace(&mut self) {
        // Never pop the root namespace
        if self.namespace_stack.len() > 1 {
            self.namespace_stack.pop();
        }
    }

    /// Get the current namespace
    pub fn current_namespace(&self) -> String {
        self.namespace_stack
            .last()
            .cloned()
            .unwrap_or_else(|| "/".to_string())
    }
}

/// Normalize a namespace string
fn normalize_namespace(ns: &str) -> String {
    let trimmed = ns.trim();

    if trimmed.is_empty() {
        return String::new();
    }

    // Remove trailing slashes
    let normalized = trimmed.trim_end_matches('/').to_string();

    normalized
}

impl Default for LaunchContext {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_context() {
        let context = LaunchContext::new();
        assert!(context.get_configuration("any").is_none());
    }

    #[test]
    fn test_set_and_get() {
        let mut context = LaunchContext::new();
        context.set_configuration("key".to_string(), "value".to_string());
        assert_eq!(context.get_configuration("key"), Some("value".to_string()));
    }

    #[test]
    fn test_override_configuration() {
        let mut context = LaunchContext::new();
        context.set_configuration("key".to_string(), "value1".to_string());
        context.set_configuration("key".to_string(), "value2".to_string());
        assert_eq!(context.get_configuration("key"), Some("value2".to_string()));
    }

    #[test]
    fn test_namespace_default() {
        let context = LaunchContext::new();
        assert_eq!(context.current_namespace(), "/");
    }

    #[test]
    fn test_push_namespace_relative() {
        let mut context = LaunchContext::new();
        context.push_namespace("ns1".to_string());
        assert_eq!(context.current_namespace(), "/ns1");

        context.push_namespace("ns2".to_string());
        assert_eq!(context.current_namespace(), "/ns1/ns2");
    }

    #[test]
    fn test_push_namespace_absolute() {
        let mut context = LaunchContext::new();
        context.push_namespace("ns1".to_string());
        assert_eq!(context.current_namespace(), "/ns1");

        // Absolute namespace overrides current
        context.push_namespace("/other".to_string());
        assert_eq!(context.current_namespace(), "/other");
    }

    #[test]
    fn test_pop_namespace() {
        let mut context = LaunchContext::new();
        context.push_namespace("ns1".to_string());
        context.push_namespace("ns2".to_string());
        assert_eq!(context.current_namespace(), "/ns1/ns2");

        context.pop_namespace();
        assert_eq!(context.current_namespace(), "/ns1");

        context.pop_namespace();
        assert_eq!(context.current_namespace(), "/");

        // Can't pop root
        context.pop_namespace();
        assert_eq!(context.current_namespace(), "/");
    }

    #[test]
    fn test_namespace_normalization() {
        let mut context = LaunchContext::new();

        // Trailing slashes should be removed
        context.push_namespace("ns1/".to_string());
        assert_eq!(context.current_namespace(), "/ns1");
        context.pop_namespace();

        // Leading slash makes it absolute
        context.push_namespace("/absolute".to_string());
        assert_eq!(context.current_namespace(), "/absolute");
        context.pop_namespace();

        // Empty namespace doesn't change stack
        context.push_namespace("".to_string());
        assert_eq!(context.current_namespace(), "/");

        // Root namespace doesn't change stack
        context.push_namespace("/".to_string());
        assert_eq!(context.current_namespace(), "/");
    }

    #[test]
    fn test_nested_namespaces() {
        let mut context = LaunchContext::new();

        context.push_namespace("robot1".to_string());
        assert_eq!(context.current_namespace(), "/robot1");

        context.push_namespace("sensors".to_string());
        assert_eq!(context.current_namespace(), "/robot1/sensors");

        context.push_namespace("camera".to_string());
        assert_eq!(context.current_namespace(), "/robot1/sensors/camera");

        context.pop_namespace();
        assert_eq!(context.current_namespace(), "/robot1/sensors");

        context.pop_namespace();
        assert_eq!(context.current_namespace(), "/robot1");

        context.pop_namespace();
        assert_eq!(context.current_namespace(), "/");
    }
}
