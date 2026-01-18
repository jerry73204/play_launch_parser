//! Launch context for managing configurations

use std::collections::HashMap;
use std::path::PathBuf;

/// Launch context holding configurations and state
#[derive(Debug, Clone)]
pub struct LaunchContext {
    configurations: HashMap<String, String>,
    current_file: Option<PathBuf>,
}

impl LaunchContext {
    pub fn new() -> Self {
        Self {
            configurations: HashMap::new(),
            current_file: None,
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
}
