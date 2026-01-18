//! Launch context for managing configurations

use std::collections::HashMap;
use std::path::PathBuf;

/// Metadata for a declared argument
#[derive(Debug, Clone)]
pub struct ArgumentMetadata {
    pub name: String,
    pub default: Option<String>,
    pub description: Option<String>,
    pub choices: Option<Vec<String>>,
}

/// Launch context holding configurations and state
#[derive(Debug, Clone)]
pub struct LaunchContext {
    configurations: HashMap<String, String>,
    current_file: Option<PathBuf>,
    namespace_stack: Vec<String>,
    environment: HashMap<String, String>,
    declared_arguments: HashMap<String, ArgumentMetadata>,
    global_parameters: HashMap<String, String>,
}

impl LaunchContext {
    pub fn new() -> Self {
        Self {
            configurations: HashMap::new(),
            current_file: None,
            namespace_stack: vec!["/".to_string()], // Start with root namespace
            environment: HashMap::new(),
            declared_arguments: HashMap::new(),
            global_parameters: HashMap::new(),
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

    pub fn set_environment_variable(&mut self, name: String, value: String) {
        self.environment.insert(name, value);
    }

    pub fn unset_environment_variable(&mut self, name: &str) {
        self.environment.remove(name);
    }

    pub fn get_environment_variable(&self, name: &str) -> Option<String> {
        self.environment.get(name).cloned()
    }

    pub fn environment(&self) -> &HashMap<String, String> {
        &self.environment
    }

    pub fn declare_argument(&mut self, metadata: ArgumentMetadata) {
        self.declared_arguments
            .insert(metadata.name.clone(), metadata);
    }

    pub fn get_argument_metadata(&self, name: &str) -> Option<&ArgumentMetadata> {
        self.declared_arguments.get(name)
    }

    pub fn declared_arguments(&self) -> &HashMap<String, ArgumentMetadata> {
        &self.declared_arguments
    }

    pub fn set_global_parameter(&mut self, name: String, value: String) {
        self.global_parameters.insert(name, value);
    }

    pub fn get_global_parameter(&self, name: &str) -> Option<String> {
        self.global_parameters.get(name).cloned()
    }

    pub fn global_parameters(&self) -> &HashMap<String, String> {
        &self.global_parameters
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

    #[test]
    fn test_set_environment_variable() {
        let mut context = LaunchContext::new();
        context.set_environment_variable("MY_VAR".to_string(), "my_value".to_string());
        assert_eq!(
            context.get_environment_variable("MY_VAR"),
            Some("my_value".to_string())
        );
    }

    #[test]
    fn test_unset_environment_variable() {
        let mut context = LaunchContext::new();
        context.set_environment_variable("MY_VAR".to_string(), "my_value".to_string());
        assert_eq!(
            context.get_environment_variable("MY_VAR"),
            Some("my_value".to_string())
        );

        context.unset_environment_variable("MY_VAR");
        assert_eq!(context.get_environment_variable("MY_VAR"), None);
    }

    #[test]
    fn test_get_nonexistent_environment_variable() {
        let context = LaunchContext::new();
        assert_eq!(context.get_environment_variable("NONEXISTENT"), None);
    }

    #[test]
    fn test_override_environment_variable() {
        let mut context = LaunchContext::new();
        context.set_environment_variable("MY_VAR".to_string(), "value1".to_string());
        context.set_environment_variable("MY_VAR".to_string(), "value2".to_string());
        assert_eq!(
            context.get_environment_variable("MY_VAR"),
            Some("value2".to_string())
        );
    }

    #[test]
    fn test_declare_argument() {
        let mut context = LaunchContext::new();
        let metadata = ArgumentMetadata {
            name: "my_arg".to_string(),
            default: Some("default_value".to_string()),
            description: Some("Test argument".to_string()),
            choices: None,
        };

        context.declare_argument(metadata);

        let retrieved = context.get_argument_metadata("my_arg").unwrap();
        assert_eq!(retrieved.name, "my_arg");
        assert_eq!(retrieved.default, Some("default_value".to_string()));
        assert_eq!(retrieved.description, Some("Test argument".to_string()));
    }

    #[test]
    fn test_get_nonexistent_argument() {
        let context = LaunchContext::new();
        assert!(context.get_argument_metadata("nonexistent").is_none());
    }

    #[test]
    fn test_declare_argument_with_choices() {
        let mut context = LaunchContext::new();
        let metadata = ArgumentMetadata {
            name: "mode".to_string(),
            default: Some("fast".to_string()),
            description: None,
            choices: Some(vec!["fast".to_string(), "slow".to_string()]),
        };

        context.declare_argument(metadata);

        let retrieved = context.get_argument_metadata("mode").unwrap();
        assert_eq!(
            retrieved.choices,
            Some(vec!["fast".to_string(), "slow".to_string()])
        );
    }

    #[test]
    fn test_set_global_parameter() {
        let mut context = LaunchContext::new();
        context.set_global_parameter("use_sim_time".to_string(), "true".to_string());
        assert_eq!(
            context.get_global_parameter("use_sim_time"),
            Some("true".to_string())
        );
    }

    #[test]
    fn test_get_nonexistent_global_parameter() {
        let context = LaunchContext::new();
        assert_eq!(context.get_global_parameter("nonexistent"), None);
    }

    #[test]
    fn test_override_global_parameter() {
        let mut context = LaunchContext::new();
        context.set_global_parameter("param".to_string(), "value1".to_string());
        context.set_global_parameter("param".to_string(), "value2".to_string());
        assert_eq!(
            context.get_global_parameter("param"),
            Some("value2".to_string())
        );
    }
}
