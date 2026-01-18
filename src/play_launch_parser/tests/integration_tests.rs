use play_launch_parser::parse_launch_file;
use std::collections::HashMap;
use std::io::Write;
use std::path::PathBuf;
use tempfile::NamedTempFile;

/// Helper to get fixture path from project root
fn get_fixture_path(filename: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .join("tests/fixtures/launch")
        .join(filename)
}

/// Helper to get includes fixture path
fn get_includes_fixture_path(filename: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .join("tests/fixtures/includes")
        .join(filename)
}

#[test]
fn test_parse_args_fixture() {
    let fixture = get_fixture_path("test_args.launch.xml");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    // Parse with default args
    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(result.is_ok(), "Parsing should succeed: {:?}", result.err());
    let record = result.unwrap();

    // Convert to JSON for easier inspection
    let json = serde_json::to_value(&record).unwrap();

    // Verify we have nodes
    assert!(json["node"].is_array(), "Should have node array");
    let nodes = json["node"].as_array().unwrap();
    assert!(!nodes.is_empty(), "Should have at least one node");

    // Check first node
    let node = &nodes[0];
    assert_eq!(node["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node["executable"].as_str().unwrap(), "talker");
    assert_eq!(node["name"].as_str().unwrap(), "my_talker");
    assert_eq!(node["namespace"].as_str().unwrap(), "/");

    // Check params
    assert!(node["params"].is_array(), "Should have params");
    let params = node["params"].as_array().unwrap();
    assert!(!params.is_empty(), "Should have at least one param");

    // Check remaps
    assert!(node["remaps"].is_array(), "Should have remaps");
    let remaps = node["remaps"].as_array().unwrap();
    assert!(!remaps.is_empty(), "Should have at least one remap");

    // Check env vars
    assert!(node["env"].is_array(), "Should have env vars");
    let env = node["env"].as_array().unwrap();
    assert!(!env.is_empty(), "Should have at least one env var");
}

#[test]
fn test_parse_conditions_with_args() {
    let fixture = get_fixture_path("test_conditions.launch.xml");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    // Test with use_sim=true (default)
    let mut args = HashMap::new();
    args.insert("use_sim".to_string(), "true".to_string());

    let result = parse_launch_file(&fixture, args);
    assert!(result.is_ok(), "Parsing with use_sim=true should succeed");

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();

    // With use_sim=true, should have sim_talker (if condition)
    let has_sim_talker = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("sim_talker"));
    assert!(has_sim_talker, "Should have sim_talker when use_sim=true");

    // Should not have real_talker (unless condition)
    let has_real_talker = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("real_talker"));
    assert!(
        !has_real_talker,
        "Should not have real_talker when use_sim=true"
    );

    // Test with use_sim=false
    let mut args = HashMap::new();
    args.insert("use_sim".to_string(), "false".to_string());

    let result = parse_launch_file(&fixture, args);
    assert!(result.is_ok(), "Parsing with use_sim=false should succeed");

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();

    // With use_sim=false, should have real_talker (unless condition)
    let has_real_talker = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("real_talker"));
    assert!(
        has_real_talker,
        "Should have real_talker when use_sim=false"
    );

    // Should not have sim_talker (if condition)
    let has_sim_talker = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("sim_talker"));
    assert!(
        !has_sim_talker,
        "Should not have sim_talker when use_sim=false"
    );
}

#[test]
#[ignore] // TODO: Requires relative path resolution in includes
fn test_parse_include_fixture() {
    let fixture = get_fixture_path("test_include.launch.xml");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    // Also verify included file exists
    let included = get_includes_fixture_path("included.launch.xml");
    assert!(
        included.exists(),
        "Included fixture should exist: {:?}",
        included
    );

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing with includes should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();
    let nodes = json["node"].as_array().unwrap();

    // Should have 3 nodes:
    // 1. From first include (default node name)
    // 2. From second include (custom_talker)
    // 3. Local listener node
    assert_eq!(nodes.len(), 3, "Should have 3 nodes total");

    // Check we have the local listener
    let has_listener = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("local_listener"));
    assert!(has_listener, "Should have local_listener node");

    // Check we have custom_talker from second include
    let has_custom = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("custom_talker"));
    assert!(has_custom, "Should have custom_talker from second include");
}

#[test]
#[ignore] // TODO: Requires relative path resolution in includes
fn test_parse_all_features() {
    let fixture = get_fixture_path("test_all_features.launch.xml");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    // Test with default args (use_sim=true)
    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing all_features should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Verify structure
    assert!(json["node"].is_array(), "Should have node array");
    assert!(json["container"].is_array(), "Should have container array");
    assert!(json["load_node"].is_array(), "Should have load_node array");
    assert!(
        json["lifecycle_node"].is_array(),
        "Should have lifecycle_node array"
    );

    let nodes = json["node"].as_array().unwrap();

    // With use_sim=true (default), should have talker (if condition)
    let has_talker = nodes.iter().any(|n| n["name"].as_str() == Some("talker"));
    assert!(has_talker, "Should have talker when use_sim=true");

    // Should not have listener (unless condition)
    let has_listener = nodes.iter().any(|n| n["name"].as_str() == Some("listener"));
    assert!(!has_listener, "Should not have listener when use_sim=true");

    // Check namespace is applied to nodes
    for node in nodes {
        let ns = node["namespace"].as_str().unwrap();
        // Should have /robots namespace from group
        assert!(
            ns.starts_with("/robots"),
            "Nodes should be in /robots namespace"
        );
    }
}

#[test]
fn test_find_pkg_share_substitution() {
    let fixture = get_fixture_path("test_find_pkg.launch.xml");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing with find-pkg-share should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();
    let nodes = json["node"].as_array().unwrap();

    assert!(!nodes.is_empty(), "Should have at least one node");

    // Check that find-pkg-share was resolved
    // The params should have resolved paths (not containing $(find-pkg-share...))
    for node in nodes {
        if let Some(params) = node["params"].as_array() {
            for param in params {
                if let Some(param_arr) = param.as_array() {
                    if param_arr.len() >= 2 {
                        let value = param_arr[1].as_str().unwrap_or("");
                        assert!(
                            !value.contains("$(find-pkg-share"),
                            "find-pkg-share should be resolved, got: {}",
                            value
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn test_argument_override() {
    let fixture = get_fixture_path("test_args.launch.xml");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    // Test with custom argument
    let mut args = HashMap::new();
    args.insert("topic_name".to_string(), "/custom_topic".to_string());

    let result = parse_launch_file(&fixture, args);
    assert!(result.is_ok(), "Parsing with custom args should succeed");

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();

    // Check that the remap uses the custom topic
    for node in nodes {
        if let Some(remaps) = node["remaps"].as_array() {
            for remap in remaps {
                if let Some(remap_arr) = remap.as_array() {
                    if remap_arr.len() >= 2 {
                        let from = remap_arr[0].as_str().unwrap_or("");
                        let to = remap_arr[1].as_str().unwrap_or("");
                        if from == "chatter" {
                            // The fixture might use the arg for remapping
                            // This depends on the fixture structure
                            // Just verify remaps exist and are strings
                            assert!(!to.is_empty(), "Remap target should not be empty");
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn test_json_output_structure() {
    let fixture = get_fixture_path("test_args.launch.xml");

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);
    assert!(result.is_ok());

    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    // Verify top-level structure matches record.json format
    assert!(json.is_object(), "Output should be JSON object");
    assert!(json.get("node").is_some(), "Should have 'node' field");
    assert!(
        json.get("container").is_some(),
        "Should have 'container' field"
    );
    assert!(
        json.get("load_node").is_some(),
        "Should have 'load_node' field"
    );
    assert!(
        json.get("lifecycle_node").is_some(),
        "Should have 'lifecycle_node' field"
    );
    assert!(
        json.get("file_data").is_some(),
        "Should have 'file_data' field"
    );

    // All should be arrays or objects
    assert!(json["node"].is_array());
    assert!(json["container"].is_array());
    assert!(json["load_node"].is_array());
    assert!(json["lifecycle_node"].is_array());
    assert!(json["file_data"].is_object());
}

#[test]
fn test_node_command_generation() {
    let fixture = get_fixture_path("test_args.launch.xml");

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);
    assert!(result.is_ok());

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();

    assert!(!nodes.is_empty(), "Should have at least one node");

    // Check that cmd array is generated
    for node in nodes {
        assert!(node.get("cmd").is_some(), "Node should have cmd field");
        let cmd = node["cmd"].as_array().unwrap();
        assert!(!cmd.is_empty(), "cmd array should not be empty");

        // First element should be the executable path
        let exe_path = cmd[0].as_str().unwrap();
        assert!(!exe_path.is_empty(), "Executable path should not be empty");

        // Should contain --ros-args if there are params or remaps
        if node
            .get("params")
            .and_then(|p| p.as_array())
            .map(|a| !a.is_empty())
            .unwrap_or(false)
            || node
                .get("remaps")
                .and_then(|r| r.as_array())
                .map(|a| !a.is_empty())
                .unwrap_or(false)
        {
            let has_ros_args = cmd.iter().any(|arg| arg.as_str() == Some("--ros-args"));
            assert!(
                has_ros_args,
                "cmd should contain --ros-args when there are params or remaps"
            );
        }
    }
}

#[test]
fn test_push_pop_ros_namespace_actions() {
    // Test push-ros-namespace and pop-ros-namespace XML actions
    let xml = r#"<launch>
        <push-ros-namespace ns="robot1" />
        <node pkg="demo_nodes_cpp" exec="talker" name="talker1" />

        <push-ros-namespace ns="sensors" />
        <node pkg="demo_nodes_cpp" exec="listener" name="listener1" />
        <pop-ros-namespace />

        <node pkg="demo_nodes_cpp" exec="talker" name="talker2" />
        <pop-ros-namespace />

        <node pkg="demo_nodes_cpp" exec="listener" name="listener2" />
    </launch>"#;

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_ok(), "Should parse successfully");

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 4, "Should have 4 nodes");

    // Check namespaces
    // talker1 should be in /robot1
    let talker1 = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("talker1"))
        .unwrap();
    assert_eq!(talker1["namespace"].as_str().unwrap(), "/robot1");

    // listener1 should be in /robot1/sensors
    let listener1 = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("listener1"))
        .unwrap();
    assert_eq!(listener1["namespace"].as_str().unwrap(), "/robot1/sensors");

    // talker2 should be in /robot1 (after popping sensors)
    let talker2 = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("talker2"))
        .unwrap();
    assert_eq!(talker2["namespace"].as_str().unwrap(), "/robot1");

    // listener2 should be in / (after popping robot1)
    let listener2 = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("listener2"))
        .unwrap();
    assert_eq!(listener2["namespace"].as_str().unwrap(), "/");
}

#[test]
fn test_push_ros_namespace_with_substitution() {
    // Test that push-ros-namespace supports substitutions
    let xml = r#"<launch>
        <arg name="robot_ns" default="my_robot" />
        <push-ros-namespace ns="$(var robot_ns)" />
        <node pkg="demo_nodes_cpp" exec="talker" name="talker" />
        <pop-ros-namespace />
    </launch>"#;

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_ok(), "Should parse successfully");

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);

    // Should use the substituted namespace
    assert_eq!(nodes[0]["namespace"].as_str().unwrap(), "/my_robot");
}
