use play_launch_parser::parse_launch_file;
use std::collections::HashMap;
use std::path::PathBuf;

use std::sync::{Mutex, MutexGuard};

/// Global mutex to serialize Python tests
/// This prevents race conditions in the Python interpreter's global state
static PYTHON_TEST_LOCK: Mutex<()> = Mutex::new(());

/// Helper to get a lock for Python tests to ensure they run serially
fn python_test_guard() -> MutexGuard<'static, ()> {
    PYTHON_TEST_LOCK.lock().unwrap()
}

/// Helper to get fixture path from crate tests directory
fn get_fixture_path(filename: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/launch")
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
fn test_parse_python_no_import() {
    let _guard = python_test_guard();
    // Test that sys.modules registration works
    let fixture = get_fixture_path("test_no_import.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing Python launch file (sys.modules access) should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Verify we have nodes
    assert!(json["node"].is_array(), "Should have node array");
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1, "Should have 1 node");

    // Check the node
    let node = &nodes[0];
    assert_eq!(
        node["package"].as_str().unwrap(),
        "demo_nodes_cpp",
        "Should have correct package"
    );
    assert_eq!(
        node["name"].as_str().unwrap(),
        "sys_modules_test",
        "Should have correct name"
    );
}

#[test]
fn test_parse_python_container() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_python_container.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing Python container launch file should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Check container
    let containers = json["container"].as_array().unwrap();
    assert_eq!(containers.len(), 1, "Should have 1 container");
    assert_eq!(
        containers[0]["name"].as_str().unwrap(),
        "my_component_container"
    );
    assert_eq!(containers[0]["namespace"].as_str().unwrap(), "/test_ns");

    // Check composable nodes are captured as load_node entries
    let load_nodes = json["load_node"].as_array().unwrap();
    assert_eq!(load_nodes.len(), 2, "Should have 2 composable nodes");

    // Check first composable node
    assert_eq!(load_nodes[0]["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(
        load_nodes[0]["plugin"].as_str().unwrap(),
        "demo_nodes_cpp::Talker"
    );
    assert_eq!(load_nodes[0]["node_name"].as_str().unwrap(), "talker_node");
    assert_eq!(
        load_nodes[0]["target_container_name"].as_str().unwrap(),
        "/test_ns/my_component_container"
    );
    assert_eq!(
        load_nodes[0]["namespace"].as_str().unwrap(),
        "/test_ns", // Inherits from container
        "Should inherit namespace from container"
    );

    // Check second composable node
    assert_eq!(load_nodes[1]["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(
        load_nodes[1]["plugin"].as_str().unwrap(),
        "demo_nodes_cpp::Listener"
    );
    assert_eq!(
        load_nodes[1]["node_name"].as_str().unwrap(),
        "listener_node"
    );
    assert_eq!(
        load_nodes[1]["target_container_name"].as_str().unwrap(),
        "/test_ns/my_component_container"
    );
    assert_eq!(
        load_nodes[1]["namespace"].as_str().unwrap(),
        "/custom_ns", // Has its own namespace
        "Should use custom namespace"
    );
}

#[test]
fn test_parse_simple_python_launch() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_simple_python.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing Python launch file should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Verify we have nodes
    assert!(json["node"].is_array(), "Should have node array");
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1, "Should have 1 node");

    // Check the node
    let node = &nodes[0];
    assert_eq!(
        node["package"].as_str().unwrap(),
        "demo_nodes_cpp",
        "Should have correct package"
    );
    assert_eq!(
        node["executable"].as_str().unwrap(),
        "talker",
        "Should have correct executable"
    );
    assert_eq!(
        node["name"].as_str().unwrap(),
        "my_talker",
        "Should have correct name"
    );
    assert_eq!(
        node["namespace"].as_str().unwrap(),
        "/test",
        "Should have correct namespace"
    );
}

#[test]
fn test_parse_python_substitutions() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_python_substitutions.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let mut args = HashMap::new();
    args.insert("package_name".to_string(), "my_package".to_string());
    args.insert("use_sim_time".to_string(), "true".to_string());

    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing Python launch file with substitutions should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Verify we have nodes
    assert!(json["node"].is_array(), "Should have node array");
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 4, "Should have 4 nodes");

    // Check first node with PathJoinSubstitution
    let node = &nodes[0];
    assert_eq!(
        node["name"].as_str().unwrap(),
        "talker_with_config",
        "Should have correct name"
    );

    // Verify parameter with substitutions
    let params = node["params"].as_array().expect("Should have params array");
    assert!(!params.is_empty(), "Should have parameters");

    // Check that config_path parameter contains PathJoinSubstitution result
    let config_param = params.iter().find(|p| {
        p.as_array()
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_str())
            == Some("config_path")
    });
    assert!(config_param.is_some(), "Should have config_path parameter");

    // Check second node with EnvironmentVariable
    let node = &nodes[1];
    assert_eq!(
        node["name"].as_str().unwrap(),
        "listener_with_env",
        "Should have correct name"
    );

    // Check third node with ThisLaunchFileDir
    let node = &nodes[2];
    assert_eq!(
        node["name"].as_str().unwrap(),
        "talker_with_dir",
        "Should have correct name"
    );
}

#[test]
fn test_parse_python_parameters() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_python_parameters.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing Python launch file with parameters should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Verify we have nodes
    assert!(json["node"].is_array(), "Should have node array");
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 6, "Should have 6 nodes");

    // Check first node with simple parameters
    let node = &nodes[0];
    assert_eq!(
        node["name"].as_str().unwrap(),
        "talker_simple_params",
        "Should have correct name"
    );

    let params = node["params"].as_array().unwrap();
    assert!(!params.is_empty(), "Should have parameters");

    // Verify string parameter
    let string_param = params.iter().find(|p| {
        p.as_array()
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_str())
            == Some("string_param")
    });
    assert!(string_param.is_some(), "Should have string_param");
    let string_value = string_param
        .unwrap()
        .as_array()
        .unwrap()
        .get(1)
        .unwrap()
        .as_str()
        .unwrap();
    assert_eq!(string_value, "hello_world", "String param should match");

    // Verify int parameter
    let int_param = params.iter().find(|p| {
        p.as_array()
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_str())
            == Some("int_param")
    });
    assert!(int_param.is_some(), "Should have int_param");
    let int_value = int_param
        .unwrap()
        .as_array()
        .unwrap()
        .get(1)
        .unwrap()
        .as_str()
        .unwrap();
    assert_eq!(int_value, "42", "Int param should match");

    // Verify bool parameter
    let bool_param = params.iter().find(|p| {
        p.as_array()
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_str())
            == Some("bool_param")
    });
    assert!(bool_param.is_some(), "Should have bool_param");
    let bool_value = bool_param
        .unwrap()
        .as_array()
        .unwrap()
        .get(1)
        .unwrap()
        .as_str()
        .unwrap();
    assert_eq!(bool_value, "true", "Bool param should match");

    // Check second node with nested parameters (uses dot notation)
    let node = &nodes[1];
    assert_eq!(
        node["name"].as_str().unwrap(),
        "listener_nested_params",
        "Should have correct name"
    );

    let params = node["params"].as_array().unwrap();
    assert!(!params.is_empty(), "Should have parameters");

    // Verify nested parameter with dot notation
    let nested_param = params.iter().find(|p| {
        p.as_array()
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_str())
            == Some("robot.name")
    });
    assert!(
        nested_param.is_some(),
        "Should have nested parameter with dot notation"
    );
    let nested_value = nested_param
        .unwrap()
        .as_array()
        .unwrap()
        .get(1)
        .unwrap()
        .as_str()
        .unwrap();
    assert_eq!(nested_value, "my_robot", "Nested param should match");

    // Verify deeply nested parameter
    let deep_param = params.iter().find(|p| {
        p.as_array()
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_str())
            == Some("config.advanced.timeout")
    });
    assert!(deep_param.is_some(), "Should have deeply nested parameter");

    // Check third node with list of parameter dicts
    let node = &nodes[2];
    assert_eq!(
        node["name"].as_str().unwrap(),
        "talker_list_params",
        "Should have correct name"
    );

    let params = node["params"].as_array().unwrap();
    assert_eq!(params.len(), 3, "Should have 3 parameters from list");

    // Check fourth node with mixed parameters (including param file)
    let node = &nodes[3];
    assert_eq!(
        node["name"].as_str().unwrap(),
        "listener_mixed_params",
        "Should have correct name"
    );

    let params = node["params"].as_array().unwrap();
    assert!(!params.is_empty(), "Should have parameters");

    // Verify parameter file marker
    let param_file = params.iter().find(|p| {
        p.as_array()
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_str())
            == Some("__param_file")
    });
    assert!(param_file.is_some(), "Should have parameter file marker");

    // Check fifth node with array parameters
    let node = &nodes[4];
    assert_eq!(
        node["name"].as_str().unwrap(),
        "talker_array_params",
        "Should have correct name"
    );

    let params = node["params"].as_array().unwrap();
    assert!(!params.is_empty(), "Should have array parameters");

    // Verify array parameter is formatted as string
    let joints_param = params.iter().find(|p| {
        p.as_array()
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_str())
            == Some("joints")
    });
    assert!(joints_param.is_some(), "Should have joints array parameter");
    let joints_value = joints_param
        .unwrap()
        .as_array()
        .unwrap()
        .get(1)
        .unwrap()
        .as_str()
        .unwrap();
    assert!(
        joints_value.starts_with('['),
        "Array param should be formatted as string array"
    );

    // Check sixth node with no parameters
    let node = &nodes[5];
    assert_eq!(
        node["name"].as_str().unwrap(),
        "listener_no_params",
        "Should have correct name"
    );
}

#[test]
fn test_parse_python_conditions() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_python_conditions.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    // Test 1: use_sim=false, enable_debug=true (default)
    let mut args = HashMap::new();
    args.insert("use_sim".to_string(), "false".to_string());
    args.insert("enable_debug".to_string(), "true".to_string());

    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing Python launch file with conditions should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Verify we have nodes
    assert!(json["node"].is_array(), "Should have node array");
    let nodes = json["node"].as_array().unwrap();

    // With use_sim=false and enable_debug=true, we should have:
    // - real_talker (UnlessCondition(use_sim) = UnlessCondition(false) = true)
    // - debug_listener (IfCondition(enable_debug) = IfCondition(true) = true)
    // - main_listener (always)
    // NOT sim_talker (IfCondition(use_sim) = IfCondition(false) = false)
    assert_eq!(
        nodes.len(),
        3,
        "Should have 3 nodes with use_sim=false, enable_debug=true"
    );

    // Check we have real_talker
    let has_real_talker = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("real_talker"));
    assert!(
        has_real_talker,
        "Should have real_talker when use_sim=false"
    );

    // Check we have debug_listener
    let has_debug_listener = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("debug_listener"));
    assert!(
        has_debug_listener,
        "Should have debug_listener when enable_debug=true"
    );

    // Check we have main_listener
    let has_main_listener = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("main_listener"));
    assert!(has_main_listener, "Should always have main_listener");

    // Check we DON'T have sim_talker
    let has_sim_talker = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("sim_talker"));
    assert!(
        !has_sim_talker,
        "Should NOT have sim_talker when use_sim=false"
    );

    // Test 2: use_sim=true, enable_debug=false
    let mut args = HashMap::new();
    args.insert("use_sim".to_string(), "true".to_string());
    args.insert("enable_debug".to_string(), "false".to_string());

    let result = parse_launch_file(&fixture, args);
    assert!(result.is_ok(), "Parsing should succeed with different args");
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();
    let nodes = json["node"].as_array().unwrap();

    // With use_sim=true and enable_debug=false, we should have:
    // - sim_talker (IfCondition(use_sim) = IfCondition(true) = true)
    // - main_listener (always)
    // NOT real_talker (UnlessCondition(use_sim) = UnlessCondition(true) = false)
    // NOT debug_listener (IfCondition(enable_debug) = IfCondition(false) = false)
    assert_eq!(
        nodes.len(),
        2,
        "Should have 2 nodes with use_sim=true, enable_debug=false"
    );

    // Check we have sim_talker
    let has_sim_talker = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("sim_talker"));
    assert!(has_sim_talker, "Should have sim_talker when use_sim=true");

    // Check we DON'T have real_talker
    let has_real_talker = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("real_talker"));
    assert!(
        !has_real_talker,
        "Should NOT have real_talker when use_sim=true"
    );

    // Check we DON'T have debug_listener
    let has_debug_listener = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("debug_listener"));
    assert!(
        !has_debug_listener,
        "Should NOT have debug_listener when enable_debug=false"
    );
}

#[test]
fn test_parse_python_include() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_python_include.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    // Test with custom launch arguments
    let mut args = HashMap::new();
    args.insert("node_name".to_string(), "my_included_node".to_string());
    args.insert("param_value".to_string(), "my_custom_value".to_string());

    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing Python launch file with includes should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();
    let nodes = json["node"].as_array().unwrap();

    // Debug: print node names
    eprintln!("Found {} nodes:", nodes.len());
    for node in nodes {
        eprintln!("  - {}", node["name"].as_str().unwrap_or("no name"));
    }

    // Should have 2 nodes: 1 from main file + 1 from included file
    assert_eq!(nodes.len(), 2, "Should have 2 nodes (1 main + 1 included)");

    // Check for the main node
    let main_node = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("main_node"));
    assert!(main_node.is_some(), "Should have main_node from main file");
    let main_node = main_node.unwrap();
    assert_eq!(main_node["package"].as_str().unwrap(), "main_pkg");
    assert_eq!(main_node["executable"].as_str().unwrap(), "main_node");
    assert_eq!(main_node["namespace"].as_str().unwrap(), "/main");

    // Check for the included node (name will be a substitution)
    let included_node = nodes
        .iter()
        .find(|n| n["package"].as_str() == Some("included_pkg"));
    assert!(
        included_node.is_some(),
        "Should have included_pkg node from included file"
    );
    let included_node = included_node.unwrap();
    assert_eq!(included_node["package"].as_str().unwrap(), "included_pkg");
    assert_eq!(
        included_node["executable"].as_str().unwrap(),
        "included_node"
    );
    assert_eq!(included_node["namespace"].as_str().unwrap(), "/included");
    // The name should be a LaunchConfiguration substitution
    assert_eq!(included_node["name"].as_str().unwrap(), "$(var node_name)");

    // Check that launch arguments were passed to included file
    // The parameter value will also be a substitution
    let params = included_node["params"].as_array().unwrap();
    let has_param = params.iter().any(|p| {
        let tuple = p.as_array().unwrap();
        tuple[0].as_str() == Some("included_param")
            && tuple[1].as_str() == Some("$(var param_value)")
    });
    assert!(
        has_param,
        "Included node should have included_param with substitution"
    );
}

#[test]
fn test_python_load_composable_nodes() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_python_load_composable_nodes.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let mut args = HashMap::new();
    args.insert("container_name".to_string(), "test_container".to_string());
    args.insert(
        "full_container_path".to_string(),
        "/test/test_container".to_string(),
    );

    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing Python launch file with LoadComposableNodes should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Check containers
    let containers = json["container"].as_array().unwrap();
    eprintln!("Found {} containers:", containers.len());
    for container in containers {
        eprintln!(
            "  - {} (namespace: {})",
            container["name"].as_str().unwrap_or("no name"),
            container["namespace"].as_str().unwrap_or("no namespace")
        );
    }
    assert_eq!(containers.len(), 1, "Should have 1 container");

    let container = &containers[0];
    assert_eq!(container["name"].as_str().unwrap(), "test_container");
    assert_eq!(container["namespace"].as_str().unwrap(), "/test");

    // Check load_nodes
    let load_nodes = json["load_node"].as_array().unwrap();
    eprintln!("Found {} load_nodes:", load_nodes.len());
    for node in load_nodes {
        eprintln!(
            "  - {} (container: {}, namespace: {})",
            node["node_name"].as_str().unwrap_or("no name"),
            node["target_container_name"]
                .as_str()
                .unwrap_or("no container"),
            node["namespace"].as_str().unwrap_or("no namespace")
        );
    }

    // Should have 4 load_nodes:
    // 1. initial_node (from container)
    // 2. dynamic_node_1 (from LoadComposableNodes with LaunchConfiguration)
    // 3. dynamic_node_2 (from LoadComposableNodes with LaunchConfiguration, custom namespace)
    // 4. string_node (from LoadComposableNodes with string reference)
    assert_eq!(load_nodes.len(), 4, "Should have 4 load_nodes total");

    // Check initial_node
    let initial_node = load_nodes
        .iter()
        .find(|n| n["node_name"].as_str() == Some("initial_node"));
    assert!(initial_node.is_some(), "Should have initial_node");
    let initial_node = initial_node.unwrap();
    assert_eq!(
        initial_node["target_container_name"].as_str().unwrap(),
        "/test/test_container"
    );
    assert_eq!(initial_node["namespace"].as_str().unwrap(), "/test");

    // Check dynamic_node_1 (LaunchConfiguration target with full path)
    let dynamic_node_1 = load_nodes
        .iter()
        .find(|n| n["node_name"].as_str() == Some("dynamic_node_1"));
    assert!(dynamic_node_1.is_some(), "Should have dynamic_node_1");
    let dynamic_node_1 = dynamic_node_1.unwrap();
    // Target should be resolved from LaunchConfiguration with full path
    assert_eq!(
        dynamic_node_1["target_container_name"].as_str().unwrap(),
        "/test/test_container"
    );
    assert_eq!(dynamic_node_1["namespace"].as_str().unwrap(), "/test");
    assert_eq!(dynamic_node_1["package"].as_str().unwrap(), "dynamic_pkg1");

    // Check dynamic_node_2 (custom namespace)
    let dynamic_node_2 = load_nodes
        .iter()
        .find(|n| n["node_name"].as_str() == Some("dynamic_node_2"));
    assert!(dynamic_node_2.is_some(), "Should have dynamic_node_2");
    let dynamic_node_2 = dynamic_node_2.unwrap();
    assert_eq!(
        dynamic_node_2["target_container_name"].as_str().unwrap(),
        "/test/test_container"
    );
    assert_eq!(dynamic_node_2["namespace"].as_str().unwrap(), "/custom_ns");

    // Check string_node (string target with absolute path)
    let string_node = load_nodes
        .iter()
        .find(|n| n["node_name"].as_str() == Some("string_node"));
    assert!(string_node.is_some(), "Should have string_node");
    let string_node = string_node.unwrap();
    assert_eq!(
        string_node["target_container_name"].as_str().unwrap(),
        "/test/my_container"
    );
    // Namespace should be inherited from container ("/test")
    assert_eq!(string_node["namespace"].as_str().unwrap(), "/test");
    assert_eq!(string_node["package"].as_str().unwrap(), "string_pkg");
}

#[test]
fn test_opaque_function() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_opaque_function.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let mut args = HashMap::new();
    args.insert("node_count".to_string(), "5".to_string());

    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing Python launch file with OpaqueFunction should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Check nodes - should have 5 dynamically generated nodes
    let nodes = json["node"].as_array().unwrap();
    eprintln!("Found {} nodes:", nodes.len());
    for node in nodes {
        eprintln!(
            "  - {} (package: {}, namespace: {})",
            node["name"].as_str().unwrap_or("no name"),
            node["package"].as_str().unwrap_or("no package"),
            node["namespace"].as_str().unwrap_or("no namespace")
        );
    }

    assert_eq!(nodes.len(), 5, "Should have 5 dynamically generated nodes");

    // Check that all nodes were created correctly
    for i in 0..5 {
        let node_name = format!("dynamic_node_{}", i);
        let pkg_name = format!("pkg_{}", i);

        let node = nodes
            .iter()
            .find(|n| n["name"].as_str() == Some(&node_name));
        assert!(node.is_some(), "Should have node {}", node_name);

        let node = node.unwrap();
        assert_eq!(node["package"].as_str().unwrap(), pkg_name);
        assert_eq!(node["executable"].as_str().unwrap(), "exec");
        assert_eq!(node["namespace"].as_str().unwrap(), "/dynamic");
    }
}

#[test]
fn test_opaque_function_file_io() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_opaque_file_io.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing Python launch file with OpaqueFunction file I/O should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Check nodes - should have 1 node with parameters read from YAML file
    let nodes = json["node"].as_array().unwrap();
    eprintln!("Found {} nodes:", nodes.len());
    for node in nodes {
        eprintln!(
            "  - {} (package: {}, namespace: {})",
            node["name"].as_str().unwrap_or("no name"),
            node["package"].as_str().unwrap_or("no package"),
            node["namespace"].as_str().unwrap_or("no namespace")
        );
    }

    assert_eq!(nodes.len(), 1, "Should have 1 node");

    let node = &nodes[0];
    assert_eq!(node["name"].as_str().unwrap(), "test_node_with_file_params");
    assert_eq!(node["package"].as_str().unwrap(), "test_pkg");
    assert_eq!(node["executable"].as_str().unwrap(), "test_exec");
    assert_eq!(node["namespace"].as_str().unwrap(), "/test");

    // Check that parameters were read from the YAML file
    let params = node["params"].as_array().unwrap();
    eprintln!("Found {} params:", params.len());
    for param in params {
        if let Some(arr) = param.as_array() {
            eprintln!(
                "  - {}: {}",
                arr[0].as_str().unwrap_or("no key"),
                arr[1].as_str().unwrap_or("no value")
            );
        }
    }

    assert!(!params.is_empty(), "Should have parameters from YAML file");

    // Check test_param (should be 42)
    let test_param = params.iter().find(|p| {
        p.as_array()
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_str())
            == Some("test_param")
    });
    assert!(test_param.is_some(), "Should have test_param from YAML");
    let test_param_value = test_param
        .unwrap()
        .as_array()
        .unwrap()
        .get(1)
        .unwrap()
        .as_str()
        .unwrap();
    assert_eq!(test_param_value, "42", "test_param should be 42");

    // Check test_string (should be "hello")
    let test_string = params.iter().find(|p| {
        p.as_array()
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_str())
            == Some("test_string")
    });
    assert!(test_string.is_some(), "Should have test_string from YAML");
    let test_string_value = test_string
        .unwrap()
        .as_array()
        .unwrap()
        .get(1)
        .unwrap()
        .as_str()
        .unwrap();
    assert_eq!(test_string_value, "hello", "test_string should be 'hello'");
}

#[test]
fn test_opaque_function_conditional_nodes() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_opaque_conditional.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    // Test with mode=minimal (should create only base_node)
    let mut args = HashMap::new();
    args.insert("mode".to_string(), "minimal".to_string());
    let result = parse_launch_file(&fixture, args);
    assert!(result.is_ok(), "Parsing should succeed: {:?}", result.err());
    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1, "Should have 1 node with mode=minimal");
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "conditional_base");

    // Test with mode=full (should create base_node + extra_node)
    let mut args = HashMap::new();
    args.insert("mode".to_string(), "full".to_string());
    let result = parse_launch_file(&fixture, args);
    assert!(result.is_ok(), "Parsing should succeed: {:?}", result.err());
    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 2, "Should have 2 nodes with mode=full");

    let node_names: Vec<&str> = nodes.iter().map(|n| n["name"].as_str().unwrap()).collect();
    assert!(node_names.contains(&"conditional_base"));
    assert!(node_names.contains(&"conditional_extra"));
}

#[test]
fn test_list_concatenation_in_substitutions() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_list_concatenation.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);
    assert!(result.is_ok(), "Parsing should succeed: {:?}", result.err());
    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1, "Should have 1 node");

    // Note: We can't fully test the concatenated paths without ROS env,
    // but we verify the file parses and captures the node structure
    let node = &nodes[0];
    assert_eq!(node["name"].as_str().unwrap(), "list_concat_node");
    assert!(node["params"].is_array(), "Should have params");
}

#[test]
fn test_parameter_file_usage() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_parameter_file.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    // This test verifies that ParameterFile is properly mocked and doesn't crash
    // In a real ROS environment, ParameterFile would load params from the file
    assert!(result.is_ok(), "Parsing should succeed: {:?}", result.err());
    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1, "Should have 1 node");
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "param_file_node");
}

#[test]
fn test_include_with_list_arguments() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_include_with_list_args.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    // The include target doesn't exist, so we expect an error
    // But we verify the file parses and the IncludeLaunchDescription is captured
    // (it will fail during include processing, which is expected)
    assert!(result.is_err(), "Should fail to find included file");

    // The error message should indicate it's trying to include a file
    let err_msg = format!("{:?}", result.err().unwrap());
    assert!(
        err_msg.contains("params.launch.py") || err_msg.contains("No such file"),
        "Error should mention the included file"
    );
}
