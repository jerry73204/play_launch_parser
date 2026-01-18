//! Launch context for managing configurations

use crate::substitution::parser::parse_substitutions;
use crate::substitution::types::{resolve_substitutions, Substitution};
use std::cell::Cell;
use std::collections::HashMap;
use std::path::PathBuf;

/// Maximum recursion depth for variable resolution to prevent stack overflow
const MAX_RESOLUTION_DEPTH: usize = 20;

// Thread-local storage for tracking resolution depth
thread_local! {
    static RESOLUTION_DEPTH: Cell<usize> = const { Cell::new(0) };
}

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
    /// Store configurations as parsed substitutions for lazy evaluation
    configurations: HashMap<String, Vec<Substitution>>,
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

    /// Set a configuration value by parsing it as substitutions
    /// This allows variables to contain nested substitutions that are resolved lazily
    pub fn set_configuration(&mut self, name: String, value: String) {
        // Parse the value as substitutions
        match parse_substitutions(&value) {
            Ok(subs) => {
                self.configurations.insert(name, subs);
            }
            Err(_) => {
                // If parsing fails, store as literal text
                self.configurations
                    .insert(name, vec![Substitution::Text(value)]);
            }
        }
    }

    /// Get a configuration value by resolving its stored substitutions
    /// This resolves any nested substitutions at reference time
    /// If resolution fails, returns None (variable exists but can't be resolved)
    pub fn get_configuration(&self, name: &str) -> Option<String> {
        self.configurations.get(name).and_then(|subs| {
            // Try to resolve the substitutions in the context of the current state
            // If resolution fails (e.g., package not found), return None
            // This allows the caller to handle the error appropriately
            resolve_substitutions(subs, self).ok()
        })
    }

    /// Get a configuration value by resolving its substitutions, with fallback
    /// If resolution fails, constructs an unresolved string representation
    /// This is used for lenient resolution where we want a value even if some
    /// substitutions can't be resolved (e.g., for static analysis)
    ///
    /// To prevent infinite recursion from circular references, this uses a
    /// resolution depth tracker stored in thread-local storage
    pub fn get_configuration_lenient(&self, name: &str) -> Option<String> {
        self.configurations.get(name).map(|subs| {
            // Check recursion depth to prevent stack overflow from circular references
            RESOLUTION_DEPTH.with(|depth| {
                let current = depth.get();
                if current >= MAX_RESOLUTION_DEPTH {
                    // Circular reference detected or too deep nesting
                    // Return the unresolved string
                    return reconstruct_substitution_string(subs);
                }

                // Increment depth
                depth.set(current + 1);

                // Try to resolve substitutions
                let result = resolve_substitutions(subs, self).unwrap_or_else(|_| {
                    // If resolution fails, reconstruct the original string
                    // This preserves unresolved substitutions like $(find-pkg-share pkg)
                    reconstruct_substitution_string(subs)
                });

                // Decrement depth
                depth.set(current);

                result
            })
        })
    }

    /// Get the raw substitutions for a configuration (for debugging/testing)
    pub fn get_configuration_raw(&self, name: &str) -> Option<&Vec<Substitution>> {
        self.configurations.get(name)
    }

    pub fn configurations(&self) -> HashMap<String, String> {
        // Return resolved configurations
        let mut resolved = HashMap::new();
        for (key, subs) in &self.configurations {
            if let Ok(value) = resolve_substitutions(subs, self) {
                resolved.insert(key.clone(), value);
            }
        }
        resolved
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

/// Reconstruct the original string representation of substitutions
/// Used when resolution fails but we still want a string value
fn reconstruct_substitution_string(subs: &[Substitution]) -> String {
    let mut result = String::new();
    for sub in subs {
        match sub {
            Substitution::Text(s) => result.push_str(s),
            Substitution::LaunchConfiguration(name_subs) => {
                result.push_str("$(var ");
                result.push_str(&reconstruct_substitution_string(name_subs));
                result.push(')');
            }
            Substitution::EnvironmentVariable { name, default } => {
                result.push_str("$(env ");
                result.push_str(&reconstruct_substitution_string(name));
                if let Some(def) = default {
                    result.push(' ');
                    result.push_str(&reconstruct_substitution_string(def));
                }
                result.push(')');
            }
            Substitution::OptionalEnvironmentVariable { name, default } => {
                result.push_str("$(optenv ");
                result.push_str(&reconstruct_substitution_string(name));
                if let Some(def) = default {
                    result.push(' ');
                    result.push_str(&reconstruct_substitution_string(def));
                }
                result.push(')');
            }
            Substitution::Command(cmd) => {
                result.push_str("$(command ");
                result.push_str(&reconstruct_substitution_string(cmd));
                result.push(')');
            }
            Substitution::FindPackageShare(pkg) => {
                result.push_str("$(find-pkg-share ");
                result.push_str(&reconstruct_substitution_string(pkg));
                result.push(')');
            }
            Substitution::Dirname => result.push_str("$(dirname)"),
            Substitution::Filename => result.push_str("$(filename)"),
            Substitution::Anon(name) => {
                result.push_str("$(anon ");
                result.push_str(&reconstruct_substitution_string(name));
                result.push(')');
            }
            Substitution::Eval(expr) => {
                result.push_str("$(eval ");
                result.push_str(&reconstruct_substitution_string(expr));
                result.push(')');
            }
        }
    }
    result
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
