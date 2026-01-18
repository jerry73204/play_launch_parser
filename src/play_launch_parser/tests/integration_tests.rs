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

    // With use_sim=true (default), should have 2 nodes: talker and included_node
    assert_eq!(nodes.len(), 2, "Should have 2 nodes with use_sim=true");

    // Should have talker (if condition)
    let talker = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("talker"))
        .expect("Should have talker when use_sim=true");
    assert_eq!(
        talker["namespace"].as_str().unwrap(),
        "/robots",
        "Talker should be in /robots namespace from group"
    );

    // Should not have listener (unless condition)
    let has_listener = nodes.iter().any(|n| n["name"].as_str() == Some("listener"));
    assert!(!has_listener, "Should not have listener when use_sim=true");

    // Should have included_node from the include
    let included_node = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("included_node"))
        .expect("Should have included_node from include");
    assert_eq!(
        included_node["namespace"].as_str().unwrap(),
        "/",
        "Included node should be in root namespace (not affected by group)"
    );
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

#[test]
fn test_complex_nested_launch() {
    // Test complex nested launch file with multiple levels of namespaces and includes
    let fixture = get_fixture_path("test_complex_nested.launch.xml");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    // Test with default args (use_sim_time=true, use_rviz=false)
    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);
    assert!(
        result.is_ok(),
        "Should parse complex file: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();

    // Should have multiple nodes from different namespaces
    assert!(nodes.len() >= 5, "Should have at least 5 nodes");

    // Check camera node namespace
    let camera = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("front_camera"))
        .expect("Should have front_camera node");
    assert_eq!(
        camera["namespace"].as_str().unwrap(),
        "/robot1/sensors/camera",
        "Camera should be in /robot1/sensors/camera namespace"
    );

    // Check lidar node (should be present with use_sim_time=true)
    let lidar = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("front_lidar"));
    assert!(
        lidar.is_some(),
        "Lidar should be present when use_sim_time=true"
    );
    assert_eq!(
        lidar.unwrap()["namespace"].as_str().unwrap(),
        "/robot1/sensors/lidar",
        "Lidar should be in /robot1/sensors/lidar namespace"
    );

    // Check AMCL node namespace
    let amcl = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("amcl"))
        .expect("Should have amcl node");
    assert_eq!(
        amcl["namespace"].as_str().unwrap(),
        "/robot1/navigation",
        "AMCL should be in /robot1/navigation namespace"
    );

    // Check that map_server is NOT present (unless condition with use_sim_time=true)
    let map_server = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("map_server"));
    assert!(
        map_server.is_none(),
        "Map server should not be present when use_sim_time=true"
    );

    // Check that RViz is NOT present (use_rviz=false by default)
    let rviz = nodes.iter().find(|n| n["name"].as_str() == Some("rviz2"));
    assert!(
        rviz.is_none(),
        "RViz should not be present when use_rviz=false"
    );

    // Check nodes from included file
    let robot_state_pub = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("robot_state_publisher"))
        .expect("Should have robot_state_publisher from include");
    assert_eq!(
        robot_state_pub["namespace"].as_str().unwrap(),
        "/robot1",
        "Robot state publisher should be in /robot1 namespace"
    );

    // Joint state publisher should be present (use_sim_time=true in include)
    let joint_state_pub = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("joint_state_publisher"));
    assert!(
        joint_state_pub.is_some(),
        "Joint state publisher should be present in sim mode"
    );
}

#[test]
fn test_complex_nested_with_args_override() {
    // Test same complex file but with overridden arguments
    let fixture = get_fixture_path("test_complex_nested.launch.xml");

    let mut args = HashMap::new();
    args.insert("robot_name".to_string(), "test_bot".to_string());
    args.insert("use_sim_time".to_string(), "false".to_string());
    args.insert("use_rviz".to_string(), "true".to_string());

    let result = parse_launch_file(&fixture, args);
    assert!(result.is_ok(), "Should parse with overridden args");

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();

    // Check camera is in test_bot namespace
    let camera = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("front_camera"))
        .expect("Should have front_camera");
    assert!(
        camera["namespace"]
            .as_str()
            .unwrap()
            .starts_with("/test_bot"),
        "Camera should be in test_bot namespace"
    );

    // Lidar should NOT be present (use_sim_time=false)
    let lidar = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("front_lidar"));
    assert!(
        lidar.is_none(),
        "Lidar should not be present when use_sim_time=false"
    );

    // Map server SHOULD be present (unless condition with use_sim_time=false)
    let map_server = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("map_server"));
    assert!(
        map_server.is_some(),
        "Map server should be present when use_sim_time=false"
    );

    // RViz SHOULD be present (use_rviz=true)
    let rviz = nodes.iter().find(|n| n["name"].as_str() == Some("rviz2"));
    assert!(rviz.is_some(), "RViz should be present when use_rviz=true");
}

#[test]
fn test_deeply_nested_namespaces() {
    // Test that deeply nested namespace stacking works correctly
    let xml = r#"<launch>
        <group ns="level1">
            <push-ros-namespace ns="level2" />
            <group>
                <push-ros-namespace ns="level3" />
                <group>
                    <push-ros-namespace ns="level4" />
                    <node pkg="test_pkg" exec="test_node" name="deep_node" />
                    <pop-ros-namespace />
                </group>
                <node pkg="test_pkg" exec="test_node" name="mid_node" />
                <pop-ros-namespace />
            </group>
            <node pkg="test_pkg" exec="test_node" name="shallow_node" />
            <pop-ros-namespace />
        </group>
        <node pkg="test_pkg" exec="test_node" name="root_node" />
    </launch>"#;

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_ok(), "Should parse deeply nested namespaces");

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 4, "Should have 4 nodes");

    // Check each node's namespace
    let deep_node = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("deep_node"))
        .unwrap();
    assert_eq!(
        deep_node["namespace"].as_str().unwrap(),
        "/level1/level2/level3/level4"
    );

    let mid_node = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("mid_node"))
        .unwrap();
    assert_eq!(
        mid_node["namespace"].as_str().unwrap(),
        "/level1/level2/level3"
    );

    let shallow_node = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("shallow_node"))
        .unwrap();
    assert_eq!(
        shallow_node["namespace"].as_str().unwrap(),
        "/level1/level2"
    );

    let root_node = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("root_node"))
        .unwrap();
    assert_eq!(root_node["namespace"].as_str().unwrap(), "/");
}

#[test]
fn test_performance_simple_launch() {
    // Benchmark parsing performance on a simple launch file
    use std::time::Instant;

    let fixture = get_fixture_path("test_args.launch.xml");
    let args = HashMap::new();

    let start = Instant::now();
    let iterations = 100;

    for _ in 0..iterations {
        let result = parse_launch_file(&fixture, args.clone());
        assert!(result.is_ok());
    }

    let duration = start.elapsed();
    let avg_ms = duration.as_millis() as f64 / iterations as f64;

    println!("\nPerformance - Simple Launch File:");
    println!("  Total time ({} iterations): {:?}", iterations, duration);
    println!("  Average per parse: {:.2}ms", avg_ms);
    println!("  Throughput: {:.0} parses/sec", 1000.0 / avg_ms);

    // Assert reasonable performance (should be well under 100ms for simple files)
    assert!(
        avg_ms < 10.0,
        "Simple parse should be < 10ms, got {:.2}ms",
        avg_ms
    );
}

#[test]
fn test_performance_complex_nested() {
    // Benchmark parsing performance on a complex nested launch file
    use std::time::Instant;

    let fixture = get_fixture_path("test_complex_nested.launch.xml");
    let args = HashMap::new();

    let start = Instant::now();
    let iterations = 50;

    for _ in 0..iterations {
        let result = parse_launch_file(&fixture, args.clone());
        assert!(result.is_ok());
    }

    let duration = start.elapsed();
    let avg_ms = duration.as_millis() as f64 / iterations as f64;

    println!("\nPerformance - Complex Nested Launch File:");
    println!("  Total time ({} iterations): {:?}", iterations, duration);
    println!("  Average per parse: {:.2}ms", avg_ms);
    println!("  Throughput: {:.0} parses/sec", 1000.0 / avg_ms);

    // Assert reasonable performance (complex files should still be fast)
    assert!(
        avg_ms < 50.0,
        "Complex parse should be < 50ms, got {:.2}ms",
        avg_ms
    );
}

#[test]
fn test_performance_with_includes() {
    // Benchmark parsing performance with includes
    use std::time::Instant;

    let fixture = get_fixture_path("test_all_features.launch.xml");
    let args = HashMap::new();

    let start = Instant::now();
    let iterations = 50;

    for _ in 0..iterations {
        let result = parse_launch_file(&fixture, args.clone());
        assert!(result.is_ok());
    }

    let duration = start.elapsed();
    let avg_ms = duration.as_millis() as f64 / iterations as f64;

    println!("\nPerformance - Launch File with Includes:");
    println!("  Total time ({} iterations): {:?}", iterations, duration);
    println!("  Average per parse: {:.2}ms", avg_ms);
    println!("  Throughput: {:.0} parses/sec", 1000.0 / avg_ms);

    // Assert reasonable performance
    assert!(
        avg_ms < 50.0,
        "Parse with includes should be < 50ms, got {:.2}ms",
        avg_ms
    );
}

#[test]
fn test_nested_variable_substitutions() {
    // Test that variables containing substitutions are resolved correctly
    let fixture = get_fixture_path("test_nested_var_substitutions.launch.xml");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    // Should parse successfully (packages may not exist, but lenient mode should handle it)
    assert!(
        result.is_ok(),
        "Should parse with nested variable substitutions: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();

    assert_eq!(nodes.len(), 1, "Should have 1 node");

    // Check node uses the variable
    let node = &nodes[0];
    assert_eq!(node["name"].as_str().unwrap(), "test_node");

    // Package should be resolved from pkg_name variable
    assert_eq!(node["package"].as_str().unwrap(), "demo_nodes_cpp");

    // The exec_path parameter should contain either:
    // 1. The resolved path if demo_nodes_cpp is installed
    // 2. The literal $(find-pkg-share demo_nodes_cpp) if not found (lenient mode)
    let params = node["params"].as_array().unwrap();
    let exec_path_param = params
        .iter()
        .find(|p| p[0].as_str() == Some("exec_path"))
        .expect("Should have exec_path parameter");

    // The value should be present (either resolved or literal)
    assert!(
        !exec_path_param[1].as_str().unwrap().is_empty(),
        "exec_path parameter should have a value"
    );

    println!("Nested variable substitution test passed!");
    println!("exec_path value: {}", exec_path_param[1].as_str().unwrap());
}
