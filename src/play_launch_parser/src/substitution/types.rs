//! Substitution types

use crate::error::SubstitutionError;
use crate::substitution::context::LaunchContext;

/// Substitution enum representing different types of substitutions
#[derive(Debug, Clone, PartialEq)]
pub enum Substitution {
    /// Plain text (no substitution)
    Text(String),
    /// $(var name) - Launch configuration variable
    LaunchConfiguration(String),
    /// $(env VAR [default]) - Environment variable with optional default
    EnvironmentVariable {
        name: String,
        default: Option<String>,
    },
    /// $(optenv VAR [default]) - Optional environment variable (returns empty string if not set)
    OptionalEnvironmentVariable {
        name: String,
        default: Option<String>,
    },
    /// $(command cmd) - Execute shell command and capture output
    Command(String),
    /// $(find-pkg-share package_name) - Find ROS 2 package share directory
    FindPackageShare(String),
    /// $(dirname) - Directory of the current launch file
    Dirname,
    /// $(filename) - Filename of the current launch file
    Filename,
    /// $(anon name) - Generate anonymous unique name
    Anon(String),
}

impl Substitution {
    /// Resolve substitution to string value
    pub fn resolve(&self, context: &LaunchContext) -> Result<String, SubstitutionError> {
        match self {
            Substitution::Text(s) => Ok(s.clone()),
            Substitution::LaunchConfiguration(name) => context
                .get_configuration(name)
                .ok_or_else(|| SubstitutionError::UndefinedVariable(name.clone())),
            Substitution::EnvironmentVariable { name, default } => {
                std::env::var(name).or_else(|_| {
                    default
                        .clone()
                        .ok_or_else(|| SubstitutionError::UndefinedEnvVar(name.clone()))
                })
            }
            Substitution::OptionalEnvironmentVariable { name, default } => {
                // Never errors - returns default or empty string if not set
                Ok(std::env::var(name)
                    .unwrap_or_else(|_| default.clone().unwrap_or_else(|| String::new())))
            }
            Substitution::Command(cmd) => execute_command(cmd),
            Substitution::FindPackageShare(package_name) => find_package_share(package_name)
                .ok_or_else(|| SubstitutionError::PackageNotFound(package_name.clone())),
            Substitution::Dirname => context
                .current_dir()
                .and_then(|p| p.to_str().map(String::from))
                .ok_or_else(|| {
                    SubstitutionError::InvalidSubstitution(
                        "dirname: no current file set".to_string(),
                    )
                }),
            Substitution::Filename => context.current_filename().ok_or_else(|| {
                SubstitutionError::InvalidSubstitution("filename: no current file set".to_string())
            }),
            Substitution::Anon(name) => {
                // Generate a unique anonymous name
                // Format: name_<timestamp>_<random>
                use std::time::{SystemTime, UNIX_EPOCH};
                let timestamp = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_micros();
                let random: u32 = rand::random();
                Ok(format!("{}_{:x}_{:x}", name, timestamp, random))
            }
        }
    }
}

/// Find ROS 2 package share directory
fn find_package_share(package_name: &str) -> Option<String> {
    // Try ROS_DISTRO environment variable first
    if let Ok(distro) = std::env::var("ROS_DISTRO") {
        let share_path = format!("/opt/ros/{}/share/{}", distro, package_name);
        if std::path::Path::new(&share_path).exists() {
            return Some(share_path);
        }
    }

    // Fallback: Try common ROS 2 distributions
    for distro in &["jazzy", "iron", "humble", "galactic", "foxy"] {
        let share_path = format!("/opt/ros/{}/share/{}", distro, package_name);
        if std::path::Path::new(&share_path).exists() {
            return Some(share_path);
        }
    }

    // Try AMENT_PREFIX_PATH
    if let Ok(prefix_path) = std::env::var("AMENT_PREFIX_PATH") {
        for prefix in prefix_path.split(':') {
            let share_path = format!("{}/share/{}", prefix, package_name);
            if std::path::Path::new(&share_path).exists() {
                return Some(share_path);
            }
        }
    }

    None
}

/// Execute shell command and capture output
///
/// # Security Note
/// This function executes arbitrary shell commands. Only use with trusted input.
/// Commands are executed in a shell context and can access the full system.
fn execute_command(cmd: &str) -> Result<String, SubstitutionError> {
    use std::process::Command;

    let output = Command::new("sh")
        .arg("-c")
        .arg(cmd)
        .output()
        .map_err(|e| {
            SubstitutionError::InvalidSubstitution(format!("Failed to execute command: {}", e))
        })?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(SubstitutionError::InvalidSubstitution(format!(
            "Command failed with exit code {:?}: {}",
            output.status.code(),
            stderr.trim()
        )));
    }

    let stdout = String::from_utf8_lossy(&output.stdout);
    // Trim whitespace from output as per ROS 2 behavior
    Ok(stdout.trim().to_string())
}

/// Resolve list of substitutions to single string
pub fn resolve_substitutions(
    subs: &[Substitution],
    context: &LaunchContext,
) -> Result<String, SubstitutionError> {
    let mut result = String::new();
    for sub in subs {
        result.push_str(&sub.resolve(context)?);
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_text_substitution() {
        let sub = Substitution::Text("hello".to_string());
        let context = LaunchContext::new();
        assert_eq!(sub.resolve(&context).unwrap(), "hello");
    }

    #[test]
    fn test_launch_configuration() {
        let sub = Substitution::LaunchConfiguration("my_var".to_string());
        let mut context = LaunchContext::new();
        context.set_configuration("my_var".to_string(), "value123".to_string());
        assert_eq!(sub.resolve(&context).unwrap(), "value123");
    }

    #[test]
    fn test_undefined_variable() {
        let sub = Substitution::LaunchConfiguration("undefined".to_string());
        let context = LaunchContext::new();
        assert!(sub.resolve(&context).is_err());
    }

    #[test]
    fn test_env_var() {
        std::env::set_var("TEST_VAR", "test_value");
        let sub = Substitution::EnvironmentVariable {
            name: "TEST_VAR".to_string(),
            default: None,
        };
        let context = LaunchContext::new();
        assert_eq!(sub.resolve(&context).unwrap(), "test_value");
    }

    #[test]
    fn test_env_var_with_default() {
        let sub = Substitution::EnvironmentVariable {
            name: "NONEXISTENT_VAR".to_string(),
            default: Some("default_value".to_string()),
        };
        let context = LaunchContext::new();
        assert_eq!(sub.resolve(&context).unwrap(), "default_value");
    }

    #[test]
    fn test_resolve_multiple() {
        let subs = vec![
            Substitution::Text("Hello ".to_string()),
            Substitution::LaunchConfiguration("name".to_string()),
            Substitution::Text("!".to_string()),
        ];
        let mut context = LaunchContext::new();
        context.set_configuration("name".to_string(), "World".to_string());
        assert_eq!(
            resolve_substitutions(&subs, &context).unwrap(),
            "Hello World!"
        );
    }

    #[test]
    fn test_dirname_substitution() {
        use std::path::PathBuf;

        let sub = Substitution::Dirname;
        let mut context = LaunchContext::new();
        context.set_current_file(PathBuf::from("/home/user/launch/test.launch.xml"));

        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "/home/user/launch");
    }

    #[test]
    fn test_filename_substitution() {
        use std::path::PathBuf;

        let sub = Substitution::Filename;
        let mut context = LaunchContext::new();
        context.set_current_file(PathBuf::from("/home/user/launch/test.launch.xml"));

        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "test.launch.xml");
    }

    #[test]
    fn test_dirname_no_file_set() {
        let sub = Substitution::Dirname;
        let context = LaunchContext::new();
        assert!(sub.resolve(&context).is_err());
    }

    #[test]
    fn test_anon_substitution() {
        let sub = Substitution::Anon("my_node".to_string());
        let context = LaunchContext::new();

        let result = sub.resolve(&context).unwrap();
        // Should start with the name and have a timestamp and random suffix
        assert!(result.starts_with("my_node_"));
        // Should have the format name_timestamp_random
        let parts: Vec<&str> = result.split('_').collect();
        assert!(parts.len() >= 3);
    }

    #[test]
    fn test_anon_uniqueness() {
        let sub1 = Substitution::Anon("node".to_string());
        let sub2 = Substitution::Anon("node".to_string());
        let context = LaunchContext::new();

        let result1 = sub1.resolve(&context).unwrap();
        let result2 = sub2.resolve(&context).unwrap();

        // Should generate different names
        assert_ne!(result1, result2);
    }

    #[test]
    fn test_optenv_with_existing_var() {
        std::env::set_var("TEST_OPTENV_VAR", "test_value");
        let sub = Substitution::OptionalEnvironmentVariable {
            name: "TEST_OPTENV_VAR".to_string(),
            default: None,
        };
        let context = LaunchContext::new();
        assert_eq!(sub.resolve(&context).unwrap(), "test_value");
    }

    #[test]
    fn test_optenv_with_missing_var_no_default() {
        // Make sure the variable doesn't exist
        std::env::remove_var("NONEXISTENT_OPTENV_VAR");
        let sub = Substitution::OptionalEnvironmentVariable {
            name: "NONEXISTENT_OPTENV_VAR".to_string(),
            default: None,
        };
        let context = LaunchContext::new();
        // Should return empty string, not error
        assert_eq!(sub.resolve(&context).unwrap(), "");
    }

    #[test]
    fn test_optenv_with_missing_var_with_default() {
        std::env::remove_var("NONEXISTENT_OPTENV_VAR2");
        let sub = Substitution::OptionalEnvironmentVariable {
            name: "NONEXISTENT_OPTENV_VAR2".to_string(),
            default: Some("default_value".to_string()),
        };
        let context = LaunchContext::new();
        assert_eq!(sub.resolve(&context).unwrap(), "default_value");
    }

    #[test]
    fn test_optenv_vs_env_behavior() {
        std::env::remove_var("MISSING_VAR_TEST");

        // optenv should not error
        let optenv = Substitution::OptionalEnvironmentVariable {
            name: "MISSING_VAR_TEST".to_string(),
            default: None,
        };
        let context = LaunchContext::new();
        assert!(optenv.resolve(&context).is_ok());

        // env should error without default
        let env = Substitution::EnvironmentVariable {
            name: "MISSING_VAR_TEST".to_string(),
            default: None,
        };
        assert!(env.resolve(&context).is_err());
    }

    #[test]
    fn test_command_simple() {
        let sub = Substitution::Command("echo hello".to_string());
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "hello");
    }

    #[test]
    fn test_command_with_output_trimming() {
        let sub = Substitution::Command("echo '  spaces  '".to_string());
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        // Output should be trimmed
        assert_eq!(result, "spaces");
    }

    #[test]
    fn test_command_with_newlines() {
        let sub = Substitution::Command("printf 'line1\\nline2\\n'".to_string());
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        // Trailing newline should be trimmed
        assert_eq!(result, "line1\nline2");
    }

    #[test]
    fn test_command_failed() {
        let sub = Substitution::Command("exit 1".to_string());
        let context = LaunchContext::new();
        let result = sub.resolve(&context);
        assert!(result.is_err());
    }

    #[test]
    fn test_command_with_args() {
        let sub = Substitution::Command("echo foo bar".to_string());
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "foo bar");
    }

    #[test]
    fn test_command_pwd() {
        let sub = Substitution::Command("pwd".to_string());
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        // Should return some directory path (non-empty)
        assert!(!result.is_empty());
        assert!(result.starts_with('/'));
    }

    #[test]
    fn test_command_env_access() {
        std::env::set_var("TEST_CMD_VAR", "test_value");
        let sub = Substitution::Command("echo $TEST_CMD_VAR".to_string());
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "test_value");
    }
}
