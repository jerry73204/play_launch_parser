//! Bridge between Python and Rust types

use crate::{
    context::ParseContext,
    error::Result,
    record::{ComposableNodeContainerRecord, LoadNodeRecord, NodeRecord},
};
use indexmap::IndexMap;
use once_cell::sync::Lazy;
use parking_lot::Mutex;
use std::{cell::RefCell, collections::HashMap, sync::Arc};

/// Global storage for launch configurations (arguments passed to the launch file)
/// This allows conditions to access and resolve LaunchConfiguration substitutions
pub static LAUNCH_CONFIGURATIONS: Lazy<Arc<Mutex<HashMap<String, String>>>> =
    Lazy::new(|| Arc::new(Mutex::new(HashMap::new())));

/// Global storage for ROS global parameters set via SetParameter actions
/// Separate from LAUNCH_CONFIGURATIONS to distinguish launch args from global params
/// Uses IndexMap to preserve insertion order (matches Python dict behavior)
pub static GLOBAL_PARAMETERS: Lazy<Arc<Mutex<IndexMap<String, String>>>> =
    Lazy::new(|| Arc::new(Mutex::new(IndexMap::new())));

/// Captured node data from Python
#[derive(Debug, Clone)]
pub struct NodeCapture {
    pub package: String,
    pub executable: String,
    pub name: Option<String>,
    pub namespace: Option<String>,
    pub parameters: Vec<(String, String)>,
    pub params_files: Vec<String>,
    pub remappings: Vec<(String, String)>,
    pub arguments: Vec<String>,
    pub env_vars: Vec<(String, String)>,
}

impl NodeCapture {
    /// Convert to NodeRecord and generate command line
    pub fn to_record(&self) -> Result<NodeRecord> {
        // Extract global parameters from GLOBAL_PARAMETERS (set via SetParameter)
        let global_params = {
            let params = GLOBAL_PARAMETERS.lock();

            if params.is_empty() {
                None
            } else {
                Some(params.iter().map(|(k, v)| (k.clone(), v.clone())).collect())
            }
        };

        // Generate ROS command line (now includes global params)
        let cmd = self.generate_command(&global_params);

        // Extract ROS args (everything after --ros-args)
        let ros_args = cmd
            .iter()
            .position(|s| s == "--ros-args")
            .map(|pos| cmd[pos + 1..].to_vec());

        // Extract parameter files from command (--params-file arguments)
        let params_files = self.extract_params_files_from_cmd(&cmd);

        Ok(NodeRecord {
            args: if self.arguments.is_empty() {
                None
            } else {
                Some(self.arguments.clone())
            },
            cmd,
            env: if self.env_vars.is_empty() {
                None
            } else {
                Some(self.env_vars.clone())
            },
            exec_name: self.name.clone(), // Match Python: exec_name = node name
            executable: self.executable.clone(),
            global_params,
            name: self.name.clone(),
            namespace: self.namespace.clone(),
            package: Some(self.package.clone()),
            params: self.parameters.clone(),
            params_files,
            remaps: self.remappings.clone(),
            respawn: None,
            respawn_delay: None,
            ros_args,
        })
    }

    /// Extract parameter file paths from command line
    fn extract_params_files_from_cmd(&self, cmd: &[String]) -> Vec<String> {
        let mut params_files = Vec::new();
        let mut i = 0;
        while i < cmd.len() {
            if cmd[i] == "--params-file" && i + 1 < cmd.len() {
                params_files.push(cmd[i + 1].clone());
                i += 2;
            } else {
                i += 1;
            }
        }
        params_files
    }

    /// Generate ROS 2 command line
    fn generate_command(&self, global_params: &Option<Vec<(String, String)>>) -> Vec<String> {
        // 1. Base command: ros2 run <package> <executable>
        let mut cmd = vec![
            "ros2".to_string(),
            "run".to_string(),
            self.package.clone(),
            self.executable.clone(),
        ];

        // 2. Add custom arguments if any
        if !self.arguments.is_empty() {
            cmd.extend(self.arguments.clone());
        }

        // 3. ROS args delimiter
        cmd.push("--ros-args".to_string());

        // 4. Node name
        if let Some(ref name) = self.name {
            cmd.push("-r".to_string());
            cmd.push(format!("__node:={}", name));
        }

        // 5. Namespace
        if let Some(ref ns) = self.namespace {
            if !ns.is_empty() && ns != "/" {
                cmd.push("-r".to_string());
                cmd.push(format!("__ns:={}", ns));
            }
        }

        // 6. Remappings
        for (from, to) in &self.remappings {
            cmd.push("-r".to_string());
            cmd.push(format!("{}:={}", from, to));
        }

        // 7. Parameters
        for (name, value) in &self.parameters {
            cmd.push("-p".to_string());
            cmd.push(format!("{}:={}", name, value));
        }

        // 8. Global parameters
        if let Some(ref params) = global_params {
            for (key, value) in params {
                cmd.push("-p".to_string());
                cmd.push(format!("{}:={}", key, value));
            }
        }

        // 9. Parameter files
        for params_file in &self.params_files {
            cmd.push("--params-file".to_string());
            cmd.push(params_file.clone());
        }

        cmd
    }
}

/// Captured container data from Python
#[derive(Debug, Clone)]
pub struct ContainerCapture {
    pub name: String,
    pub namespace: String,
    pub package: Option<String>,
    pub executable: Option<String>,
    pub cmd: Vec<String>,
}

impl ContainerCapture {
    /// Convert to ComposableNodeContainerRecord
    pub fn to_record(&self) -> Result<ComposableNodeContainerRecord> {
        let package = self
            .package
            .clone()
            .unwrap_or_else(|| "rclcpp_components".to_string());
        let executable = self
            .executable
            .clone()
            .unwrap_or_else(|| "component_container".to_string());

        // Extract global parameters from LAUNCH_CONFIGURATIONS
        let global_params = {
            let configs = LAUNCH_CONFIGURATIONS.lock();
            let mut params = Vec::new();

            // Common global parameters to check
            for key in [
                "use_sim_time",
                "wheel_radius",
                "wheel_width",
                "wheel_base",
                "wheel_tread",
                "front_overhang",
                "rear_overhang",
                "left_overhang",
                "right_overhang",
                "vehicle_height",
                "max_steer_angle",
            ] {
                if let Some(value) = configs.get(key) {
                    params.push((key.to_string(), value.clone()));
                }
            }

            if params.is_empty() {
                None
            } else {
                Some(params)
            }
        };

        // Generate command if not provided
        let cmd = if self.cmd.is_empty() {
            let mut cmd = vec![
                format!("/opt/ros/humble/lib/{}/{}", package, executable),
                "--ros-args".to_string(),
                "-r".to_string(),
                format!("__node:={}", self.name),
                "-r".to_string(),
                format!("__ns:={}", self.namespace),
            ];

            // Add global parameters to command
            if let Some(ref params) = global_params {
                for (key, value) in params {
                    cmd.push("-p".to_string());
                    cmd.push(format!("{}:={}", key, value));
                }
            }

            cmd
        } else {
            self.cmd.clone()
        };

        Ok(ComposableNodeContainerRecord {
            args: None,
            cmd,
            env: None,
            exec_name: Some(format!("{}-1", executable)),
            executable,
            global_params,
            name: self.name.clone(),
            namespace: self.namespace.clone(),
            package,
            params: Vec::new(),
            params_files: Vec::new(),
            remaps: Vec::new(),
            respawn: Some(false),
            respawn_delay: None,
            ros_args: None,
        })
    }
}

/// Captured composable node data from Python
#[derive(Debug, Clone)]
pub struct LoadNodeCapture {
    pub package: String,
    pub plugin: String,
    pub target_container_name: String,
    pub node_name: String,
    pub namespace: String,
    pub parameters: Vec<(String, String)>,
    pub remappings: Vec<(String, String)>,
}

impl LoadNodeCapture {
    /// Convert to LoadNodeRecord
    pub fn to_record(&self) -> Result<LoadNodeRecord> {
        log::warn!(
            "DEBUG LoadNodeCapture::to_record: node='{}', self.parameters.len()={}",
            self.node_name,
            self.parameters.len()
        );

        // Merge global parameters with node-specific parameters
        // Global parameters are added first, then node-specific parameters
        // (node-specific parameters can override global ones)
        let mut merged_params = Vec::new();

        // Add global parameters
        {
            let global_params = GLOBAL_PARAMETERS.lock();
            log::warn!(
                "DEBUG LoadNodeCapture::to_record: GLOBAL_PARAMETERS has {} entries for node '{}'",
                global_params.len(),
                self.node_name
            );
            for (key, value) in global_params.iter() {
                merged_params.push((key.clone(), value.clone()));
            }
        }

        // Add node-specific parameters (may override global params)
        for (key, value) in &self.parameters {
            // Check if this key already exists in merged_params
            if let Some(existing) = merged_params.iter_mut().find(|(k, _)| k == key) {
                // Override global parameter with node-specific value
                existing.1 = value.clone();
            } else {
                // Add new parameter
                merged_params.push((key.clone(), value.clone()));
            }
        }

        log::warn!(
            "DEBUG LoadNodeCapture::to_record: Final merged_params count={} for node '{}'",
            merged_params.len(),
            self.node_name
        );

        Ok(LoadNodeRecord {
            package: self.package.clone(),
            plugin: self.plugin.clone(),
            target_container_name: self.target_container_name.clone(),
            node_name: self.node_name.clone(),
            namespace: self.namespace.clone(),
            log_level: None,
            remaps: self.remappings.clone(),
            params: merged_params,
            extra_args: HashMap::new(),
            env: None,
        })
    }
}

/// Captured include data from Python
#[derive(Debug, Clone)]
pub struct IncludeCapture {
    pub file_path: String,
    pub args: Vec<(String, String)>,
    pub ros_namespace: String,
}

/// Push a namespace onto the ROS namespace stack
/// Uses ParseContext via thread-local storage
pub fn push_ros_namespace(namespace: String) {
    with_parse_context(|ctx| {
        let normalized = if namespace.is_empty() {
            String::new()
        } else if namespace.starts_with('/') {
            namespace
        } else {
            format!("/{}", namespace)
        };

        log::debug!("Pushing ROS namespace to ParseContext: '{}'", normalized);
        ctx.push_namespace(normalized);
    });
}

/// Pop a namespace from the ROS namespace stack
/// Uses ParseContext via thread-local storage
pub fn pop_ros_namespace() {
    with_parse_context(|ctx| {
        if let Some(popped) = ctx.pop_namespace() {
            log::debug!("Popped ROS namespace from ParseContext: {:?}", popped);
        } else {
            log::warn!("Attempted to pop from root namespace in ParseContext");
        }
    });
}

/// Get the current ROS namespace (concatenation of all namespaces in the stack)
/// Uses ParseContext via thread-local storage
pub fn get_current_ros_namespace() -> String {
    with_parse_context(|ctx| {
        let result = ctx.current_namespace();
        log::debug!("get_current_ros_namespace from ParseContext: '{}'", result);
        result
    })
}

// ========== Thread-Local ParseContext (replaces global state) ==========

thread_local! {
    /// Thread-local storage for the current ParseContext
    /// This allows Python API classes to access the context without global state
    static CURRENT_PARSE_CONTEXT: RefCell<Option<*mut ParseContext>> = const { RefCell::new(None) };
}

/// Set the current ParseContext for this thread
/// SAFETY: The caller must ensure the ParseContext lives for the duration of Python execution
pub fn set_current_parse_context(ctx: &mut ParseContext) {
    CURRENT_PARSE_CONTEXT.with(|cell| {
        *cell.borrow_mut() = Some(ctx as *mut ParseContext);
    });
}

/// Get the current ParseContext for this thread
/// Returns None if no context is set
pub fn get_current_parse_context() -> Option<*mut ParseContext> {
    CURRENT_PARSE_CONTEXT.with(|cell| *cell.borrow())
}

/// Clear the current ParseContext for this thread
pub fn clear_current_parse_context() {
    CURRENT_PARSE_CONTEXT.with(|cell| {
        *cell.borrow_mut() = None;
    });
}

/// Execute a closure with access to the current ParseContext
/// Panics if no context is set
pub fn with_parse_context<F, R>(f: F) -> R
where
    F: FnOnce(&mut ParseContext) -> R,
{
    let ctx_ptr = get_current_parse_context()
        .expect("No ParseContext set - Python execution must be called through LaunchTraverser");

    // SAFETY: The pointer is valid for the duration of Python execution
    // as guaranteed by set_current_parse_context
    unsafe { f(&mut *ctx_ptr) }
}

/// Get a clone of all captured nodes from ParseContext
/// Panics if no context is set
pub fn get_captured_nodes() -> Vec<NodeCapture> {
    with_parse_context(|ctx| ctx.captured_nodes().to_vec())
}

/// Get a clone of all captured containers from ParseContext
/// Panics if no context is set
pub fn get_captured_containers() -> Vec<ContainerCapture> {
    with_parse_context(|ctx| ctx.captured_containers().to_vec())
}

/// Get a clone of all captured load_nodes from ParseContext
/// Panics if no context is set
pub fn get_captured_load_nodes() -> Vec<LoadNodeCapture> {
    with_parse_context(|ctx| ctx.captured_load_nodes().to_vec())
}

/// Get a clone of all captured includes from ParseContext
/// Panics if no context is set
pub fn get_captured_includes() -> Vec<IncludeCapture> {
    with_parse_context(|ctx| ctx.captured_includes().to_vec())
}

/// Update captured nodes in ParseContext by applying a mutation function
/// Panics if no context is set
pub fn update_captured_nodes<F>(f: F)
where
    F: FnOnce(&mut Vec<NodeCapture>),
{
    with_parse_context(|ctx| {
        let nodes = ctx.captured_nodes_mut();
        f(nodes);
    })
}

/// Update captured containers in ParseContext by applying a mutation function
/// Panics if no context is set
pub fn update_captured_containers<F>(f: F)
where
    F: FnOnce(&mut Vec<ContainerCapture>),
{
    with_parse_context(|ctx| {
        let containers = ctx.captured_containers_mut();
        f(containers);
    })
}

/// Update captured load_nodes in ParseContext by applying a mutation function
/// Panics if no context is set
pub fn update_captured_load_nodes<F>(f: F)
where
    F: FnOnce(&mut Vec<LoadNodeCapture>),
{
    with_parse_context(|ctx| {
        let load_nodes = ctx.captured_load_nodes_mut();
        f(load_nodes);
    })
}

// ========== Capture Functions (use ParseContext via thread-local) ==========

/// Capture a node to ParseContext
pub fn capture_node(node: NodeCapture) {
    with_parse_context(|ctx| ctx.capture_node(node));
}

/// Capture a container to ParseContext
pub fn capture_container(container: ContainerCapture) {
    with_parse_context(|ctx| ctx.capture_container(container));
}

/// Capture a load_node to ParseContext
pub fn capture_load_node(load_node: LoadNodeCapture) {
    with_parse_context(|ctx| ctx.capture_load_node(load_node));
}

/// Capture an include to ParseContext
pub fn capture_include(include: IncludeCapture) {
    with_parse_context(|ctx| ctx.capture_include(include));
}
