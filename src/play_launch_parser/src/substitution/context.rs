//! Launch context for managing configurations

use crate::substitution::{
    parser::parse_substitutions,
    types::{resolve_substitutions, Substitution},
};
use std::{cell::Cell, collections::HashMap, path::PathBuf, sync::Arc};

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

/// Frozen parent scope - immutable, shared via Arc
/// Used to create a scope chain without expensive cloning
#[derive(Debug, Clone)]
struct ParentScope {
    configurations: HashMap<String, Vec<Substitution>>,
    environment: HashMap<String, String>,
    declared_arguments: HashMap<String, ArgumentMetadata>,
    global_parameters: HashMap<String, String>,
    remappings: Vec<(String, String)>,
    /// Chain to grandparent scope
    parent: Option<Arc<ParentScope>>,
}

/// Launch context holding configurations and state
/// Uses hybrid Arc + Local pattern: parent scope is shared (Arc), local scope is owned
/// This makes child context creation O(1) instead of O(n)
#[derive(Debug, Clone)]
pub struct LaunchContext {
    /// Parent scope (shared, immutable via Arc)
    parent: Option<Arc<ParentScope>>,

    /// Local scope (owned, mutable, initially empty for children)
    /// Store configurations as parsed substitutions for lazy evaluation
    local_configurations: HashMap<String, Vec<Substitution>>,
    local_environment: HashMap<String, String>,
    local_declared_arguments: HashMap<String, ArgumentMetadata>,
    local_global_parameters: HashMap<String, String>,
    /// Local topic remappings (from -> to)
    local_remappings: Vec<(String, String)>,

    /// Always local (not inherited)
    current_file: Option<PathBuf>,
    namespace_stack: Vec<String>,
}

impl LaunchContext {
    pub fn new() -> Self {
        Self {
            parent: None,
            local_configurations: HashMap::new(),
            local_environment: HashMap::new(),
            local_declared_arguments: HashMap::new(),
            local_global_parameters: HashMap::new(),
            local_remappings: Vec::new(),
            current_file: None,
            namespace_stack: vec!["/".to_string()], // Start with root namespace
        }
    }

    /// Create a child context with current scope frozen as parent
    /// This is O(1) for Arc clone vs O(n) for full HashMap clone
    /// Enables efficient scope chaining for includes
    pub fn child(&self) -> Self {
        // Freeze current local scope and make it the parent
        let parent = ParentScope {
            configurations: self.local_configurations.clone(),
            environment: self.local_environment.clone(),
            declared_arguments: self.local_declared_arguments.clone(),
            global_parameters: self.local_global_parameters.clone(),
            remappings: self.local_remappings.clone(),
            parent: self.parent.clone(), // Arc clone - cheap!
        };

        Self {
            parent: Some(Arc::new(parent)),
            local_configurations: HashMap::new(), // Empty local scope
            local_environment: HashMap::new(),
            local_declared_arguments: HashMap::new(),
            local_global_parameters: HashMap::new(),
            local_remappings: Vec::new(),
            current_file: None,
            namespace_stack: self.namespace_stack.clone(), // Small vec, acceptable to clone
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
    /// Always modifies local scope only (doesn't affect parent)
    pub fn set_configuration(&mut self, name: String, value: String) {
        // Parse the value as substitutions
        match parse_substitutions(&value) {
            Ok(subs) => {
                self.local_configurations.insert(name, subs);
            }
            Err(_) => {
                // If parsing fails, store as literal text
                self.local_configurations
                    .insert(name, vec![Substitution::Text(value)]);
            }
        }
    }

    /// Get a configuration value by resolving its stored substitutions
    /// This resolves any nested substitutions at reference time
    /// If resolution fails, returns None (variable exists but can't be resolved)
    /// Walks parent chain: local first, then parent, then grandparent, etc.
    pub fn get_configuration(&self, name: &str) -> Option<String> {
        // 1. Check local scope first (fast path - O(1))
        if let Some(subs) = self.local_configurations.get(name) {
            return resolve_substitutions(subs, self).ok();
        }

        // 2. Walk parent chain (depth ~5-10 for Autoware)
        let mut current = &self.parent;
        while let Some(parent) = current {
            if let Some(subs) = parent.configurations.get(name) {
                return resolve_substitutions(subs, self).ok();
            }
            current = &parent.parent;
        }

        None
    }

    /// Get a configuration value by resolving its substitutions, with fallback
    /// If resolution fails, constructs an unresolved string representation
    /// This is used for lenient resolution where we want a value even if some
    /// substitutions can't be resolved (e.g., for static analysis)
    ///
    /// To prevent infinite recursion from circular references, this uses a
    /// resolution depth tracker stored in thread-local storage
    /// Walks parent chain: local first, then parent, then grandparent, etc.
    pub fn get_configuration_lenient(&self, name: &str) -> Option<String> {
        // 1. Check local scope first
        if let Some(subs) = self.local_configurations.get(name) {
            return Some(RESOLUTION_DEPTH.with(|depth| {
                let current = depth.get();
                if current >= MAX_RESOLUTION_DEPTH {
                    return reconstruct_substitution_string(subs);
                }
                depth.set(current + 1);
                let result = resolve_substitutions(subs, self)
                    .unwrap_or_else(|_| reconstruct_substitution_string(subs));
                depth.set(current);
                result
            }));
        }

        // 2. Walk parent chain
        let mut current = &self.parent;
        while let Some(parent) = current {
            if let Some(subs) = parent.configurations.get(name) {
                return Some(RESOLUTION_DEPTH.with(|depth| {
                    let current_depth = depth.get();
                    if current_depth >= MAX_RESOLUTION_DEPTH {
                        return reconstruct_substitution_string(subs);
                    }
                    depth.set(current_depth + 1);
                    let result = resolve_substitutions(subs, self)
                        .unwrap_or_else(|_| reconstruct_substitution_string(subs));
                    depth.set(current_depth);
                    result
                }));
            }
            current = &parent.parent;
        }

        None
    }

    /// Get the raw substitutions for a configuration (for debugging/testing)
    /// Walks parent chain: local first, then parent, then grandparent, etc.
    pub fn get_configuration_raw(&self, name: &str) -> Option<&Vec<Substitution>> {
        // 1. Check local scope first
        if let Some(subs) = self.local_configurations.get(name) {
            return Some(subs);
        }

        // 2. Walk parent chain
        let mut current = &self.parent;
        while let Some(parent) = current {
            if let Some(subs) = parent.configurations.get(name) {
                return Some(subs);
            }
            current = &parent.parent;
        }

        None
    }

    pub fn configurations(&self) -> HashMap<String, String> {
        // Return resolved configurations from entire scope chain
        // Walk from root to local, so local values override parent values
        let mut resolved = HashMap::new();

        // 1. Collect all parent scopes into a vec (to iterate from root to child)
        let mut scopes = Vec::new();
        let mut current = &self.parent;
        while let Some(parent) = current {
            scopes.push(parent);
            current = &parent.parent;
        }

        // 2. Apply parent scopes from root to immediate parent
        for parent in scopes.iter().rev() {
            for (key, subs) in &parent.configurations {
                if let Ok(value) = resolve_substitutions(subs, self) {
                    resolved.insert(key.clone(), value);
                }
            }
        }

        // 3. Apply local scope (overrides parent)
        for (key, subs) in &self.local_configurations {
            if let Ok(value) = resolve_substitutions(subs, self) {
                resolved.insert(key.clone(), value);
            }
        }

        resolved
    }

    /// Set environment variable in local scope only
    pub fn set_environment_variable(&mut self, name: String, value: String) {
        self.local_environment.insert(name, value);
    }

    /// Unset environment variable in local scope only
    pub fn unset_environment_variable(&mut self, name: &str) {
        self.local_environment.remove(name);
    }

    /// Get environment variable, walking parent chain
    pub fn get_environment_variable(&self, name: &str) -> Option<String> {
        // 1. Check local scope first
        if let Some(value) = self.local_environment.get(name) {
            return Some(value.clone());
        }

        // 2. Walk parent chain
        let mut current = &self.parent;
        while let Some(parent) = current {
            if let Some(value) = parent.environment.get(name) {
                return Some(value.clone());
            }
            current = &parent.parent;
        }

        None
    }

    /// Get all environment variables from entire scope chain
    pub fn environment(&self) -> HashMap<String, String> {
        // Walk from root to local, so local values override parent values
        let mut result = HashMap::new();

        // 1. Collect all parent scopes
        let mut scopes = Vec::new();
        let mut current = &self.parent;
        while let Some(parent) = current {
            scopes.push(parent);
            current = &parent.parent;
        }

        // 2. Apply parent scopes from root to immediate parent
        for parent in scopes.iter().rev() {
            result.extend(
                parent
                    .environment
                    .iter()
                    .map(|(k, v)| (k.clone(), v.clone())),
            );
        }

        // 3. Apply local scope (overrides parent)
        result.extend(
            self.local_environment
                .iter()
                .map(|(k, v)| (k.clone(), v.clone())),
        );

        result
    }

    /// Add a global topic remapping to local scope
    pub fn add_remapping(&mut self, from: String, to: String) {
        self.local_remappings.push((from, to));
    }

    /// Get all global remappings from entire scope chain
    pub fn remappings(&self) -> Vec<(String, String)> {
        // Collect remappings from parent chain, then local
        let mut result = Vec::new();

        // 1. Collect all parent scopes
        let mut scopes = Vec::new();
        let mut current = &self.parent;
        while let Some(parent) = current {
            scopes.push(parent);
            current = &parent.parent;
        }

        // 2. Apply parent remappings from root to immediate parent
        for parent in scopes.iter().rev() {
            result.extend_from_slice(&parent.remappings);
        }

        // 3. Apply local remappings
        result.extend_from_slice(&self.local_remappings);

        result
    }

    /// Get current count of local remappings (for scope restoration)
    pub fn remapping_count(&self) -> usize {
        self.local_remappings.len()
    }

    /// Restore local remappings to a specific count
    /// Used to clean up all remappings added within a scope (e.g., group)
    pub fn restore_remapping_count(&mut self, count: usize) {
        self.local_remappings.truncate(count);
    }

    /// Declare argument in local scope only
    pub fn declare_argument(&mut self, metadata: ArgumentMetadata) {
        self.local_declared_arguments
            .insert(metadata.name.clone(), metadata);
    }

    /// Get argument metadata, walking parent chain
    pub fn get_argument_metadata(&self, name: &str) -> Option<&ArgumentMetadata> {
        // 1. Check local scope first
        if let Some(metadata) = self.local_declared_arguments.get(name) {
            return Some(metadata);
        }

        // 2. Walk parent chain
        let mut current = &self.parent;
        while let Some(parent) = current {
            if let Some(metadata) = parent.declared_arguments.get(name) {
                return Some(metadata);
            }
            current = &parent.parent;
        }

        None
    }

    /// Get all declared arguments from entire scope chain
    pub fn declared_arguments(&self) -> HashMap<String, ArgumentMetadata> {
        // Walk from root to local, so local values override parent values
        let mut result = HashMap::new();

        // 1. Collect all parent scopes
        let mut scopes = Vec::new();
        let mut current = &self.parent;
        while let Some(parent) = current {
            scopes.push(parent);
            current = &parent.parent;
        }

        // 2. Apply parent scopes from root to immediate parent
        for parent in scopes.iter().rev() {
            result.extend(
                parent
                    .declared_arguments
                    .iter()
                    .map(|(k, v)| (k.clone(), v.clone())),
            );
        }

        // 3. Apply local scope (overrides parent)
        result.extend(
            self.local_declared_arguments
                .iter()
                .map(|(k, v)| (k.clone(), v.clone())),
        );

        result
    }

    /// Set global parameter in local scope only
    pub fn set_global_parameter(&mut self, name: String, value: String) {
        self.local_global_parameters.insert(name, value);
    }

    /// Get global parameter, walking parent chain
    pub fn get_global_parameter(&self, name: &str) -> Option<String> {
        // 1. Check local scope first
        if let Some(value) = self.local_global_parameters.get(name) {
            return Some(value.clone());
        }

        // 2. Walk parent chain
        let mut current = &self.parent;
        while let Some(parent) = current {
            if let Some(value) = parent.global_parameters.get(name) {
                return Some(value.clone());
            }
            current = &parent.parent;
        }

        None
    }

    /// Get all global parameters from entire scope chain
    pub fn global_parameters(&self) -> HashMap<String, String> {
        // Walk from root to local, so local values override parent values
        let mut result = HashMap::new();

        // 1. Collect all parent scopes
        let mut scopes = Vec::new();
        let mut current = &self.parent;
        while let Some(parent) = current {
            scopes.push(parent);
            current = &parent.parent;
        }

        // 2. Apply parent scopes from root to immediate parent
        for parent in scopes.iter().rev() {
            result.extend(
                parent
                    .global_parameters
                    .iter()
                    .map(|(k, v)| (k.clone(), v.clone())),
            );
        }

        // 3. Apply local scope (overrides parent)
        result.extend(
            self.local_global_parameters
                .iter()
                .map(|(k, v)| (k.clone(), v.clone())),
        );

        result
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

    /// Get the current namespace stack depth
    /// Used to restore namespace state when exiting scopes
    pub fn namespace_depth(&self) -> usize {
        self.namespace_stack.len()
    }

    /// Restore namespace stack to a specific depth
    /// Used to clean up all namespace pushes within a scope
    pub fn restore_namespace_depth(&mut self, depth: usize) {
        // Never shrink below 1 (root namespace must always exist)
        let target_depth = depth.max(1);
        while self.namespace_stack.len() > target_depth {
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
            Substitution::Command { cmd, error_mode } => {
                result.push_str("$(command ");
                result.push_str(&reconstruct_substitution_string(cmd));
                // Include error mode if not default (Strict)
                if *error_mode != crate::substitution::types::CommandErrorMode::Strict {
                    result.push_str(" '");
                    result.push_str(match error_mode {
                        crate::substitution::types::CommandErrorMode::Warn => "warn",
                        crate::substitution::types::CommandErrorMode::Ignore => "ignore",
                        crate::substitution::types::CommandErrorMode::Strict => "strict",
                    });
                    result.push('\'');
                }
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
