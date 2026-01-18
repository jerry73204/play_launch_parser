//! Substitution parser

use crate::error::{ParseError, Result};
use crate::substitution::types::Substitution;
use regex::Regex;

/// Parse substitution string like "$(var x)" or "text $(env Y) more"
pub fn parse_substitutions(input: &str) -> Result<Vec<Substitution>> {
    let mut result = Vec::new();
    // Match both $(type args) and $(type) patterns
    let re = Regex::new(r"\$\(([a-z-]+)(?:\s+([^)]+))?\)").unwrap();

    let mut last_end = 0;

    for cap in re.captures_iter(input) {
        let match_obj = cap.get(0).unwrap();
        let start = match_obj.start();
        let end = match_obj.end();

        // Add text before the substitution
        if start > last_end {
            let text = &input[last_end..start];
            if !text.is_empty() {
                result.push(Substitution::Text(text.to_string()));
            }
        }

        // Parse the substitution
        let sub_type = cap.get(1).unwrap().as_str();
        let args = cap.get(2).map(|m| m.as_str());

        let substitution = parse_single_substitution(sub_type, args)?;
        result.push(substitution);

        last_end = end;
    }

    // Add remaining text
    if last_end < input.len() {
        let text = &input[last_end..];
        if !text.is_empty() {
            result.push(Substitution::Text(text.to_string()));
        }
    }

    // If no substitutions found, treat entire input as text
    if result.is_empty() {
        result.push(Substitution::Text(input.to_string()));
    }

    Ok(result)
}

fn parse_single_substitution(sub_type: &str, args: Option<&str>) -> Result<Substitution> {
    match sub_type {
        "var" => {
            let name = args
                .ok_or_else(|| {
                    ParseError::InvalidSubstitution("var requires an argument".to_string())
                })?
                .trim()
                .to_string();
            Ok(Substitution::LaunchConfiguration(name))
        }
        "env" => {
            let args_str = args.ok_or_else(|| {
                ParseError::InvalidSubstitution("env requires an argument".to_string())
            })?;
            let parts: Vec<&str> = args_str.splitn(2, ' ').map(|s| s.trim()).collect();
            let name = parts[0].to_string();
            let default = parts.get(1).map(|s| s.to_string());
            Ok(Substitution::EnvironmentVariable { name, default })
        }
        "find-pkg-share" => {
            let package = args
                .ok_or_else(|| {
                    ParseError::InvalidSubstitution(
                        "find-pkg-share requires an argument".to_string(),
                    )
                })?
                .trim()
                .to_string();
            Ok(Substitution::FindPackageShare(package))
        }
        "dirname" => Ok(Substitution::Dirname),
        "filename" => Ok(Substitution::Filename),
        "anon" => {
            let name = args
                .ok_or_else(|| {
                    ParseError::InvalidSubstitution("anon requires a name argument".to_string())
                })?
                .trim()
                .to_string();
            Ok(Substitution::Anon(name))
        }
        _ => Err(ParseError::InvalidSubstitution(format!(
            "Unknown substitution type: {}",
            sub_type
        ))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_plain_text() {
        let subs = parse_substitutions("hello world").unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(subs[0], Substitution::Text("hello world".to_string()));
    }

    #[test]
    fn test_parse_var_substitution() {
        let subs = parse_substitutions("$(var my_var)").unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(
            subs[0],
            Substitution::LaunchConfiguration("my_var".to_string())
        );
    }

    #[test]
    fn test_parse_env_substitution() {
        let subs = parse_substitutions("$(env HOME)").unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(
            subs[0],
            Substitution::EnvironmentVariable {
                name: "HOME".to_string(),
                default: None
            }
        );
    }

    #[test]
    fn test_parse_env_with_default() {
        let subs = parse_substitutions("$(env MY_VAR default_value)").unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(
            subs[0],
            Substitution::EnvironmentVariable {
                name: "MY_VAR".to_string(),
                default: Some("default_value".to_string())
            }
        );
    }

    #[test]
    fn test_parse_mixed() {
        let subs = parse_substitutions("prefix $(var x) middle $(env Y) suffix").unwrap();
        assert_eq!(subs.len(), 5);
        assert_eq!(subs[0], Substitution::Text("prefix ".to_string()));
        assert_eq!(subs[1], Substitution::LaunchConfiguration("x".to_string()));
        assert_eq!(subs[2], Substitution::Text(" middle ".to_string()));
        assert_eq!(
            subs[3],
            Substitution::EnvironmentVariable {
                name: "Y".to_string(),
                default: None
            }
        );
        assert_eq!(subs[4], Substitution::Text(" suffix".to_string()));
    }

    #[test]
    fn test_parse_consecutive_substitutions() {
        let subs = parse_substitutions("$(var a)$(var b)").unwrap();
        assert_eq!(subs.len(), 2);
        assert_eq!(subs[0], Substitution::LaunchConfiguration("a".to_string()));
        assert_eq!(subs[1], Substitution::LaunchConfiguration("b".to_string()));
    }

    #[test]
    fn test_parse_dirname() {
        let subs = parse_substitutions("$(dirname)").unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(subs[0], Substitution::Dirname);
    }

    #[test]
    fn test_parse_filename() {
        let subs = parse_substitutions("$(filename)").unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(subs[0], Substitution::Filename);
    }

    #[test]
    fn test_parse_dirname_in_path() {
        let subs = parse_substitutions("$(dirname)/config.yaml").unwrap();
        assert_eq!(subs.len(), 2);
        assert_eq!(subs[0], Substitution::Dirname);
        assert_eq!(subs[1], Substitution::Text("/config.yaml".to_string()));
    }

    #[test]
    fn test_parse_anon() {
        let subs = parse_substitutions("$(anon my_node)").unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(subs[0], Substitution::Anon("my_node".to_string()));
    }

    #[test]
    fn test_parse_anon_in_name() {
        let subs = parse_substitutions("node_$(anon suffix)").unwrap();
        assert_eq!(subs.len(), 2);
        assert_eq!(subs[0], Substitution::Text("node_".to_string()));
        assert_eq!(subs[1], Substitution::Anon("suffix".to_string()));
    }
}
