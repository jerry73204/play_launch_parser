//! Bridge between Python and Rust types

use crate::{
    captures::{ContainerCapture, IncludeCapture, LoadNodeCapture, NodeCapture},
    error::Result,
    record::{ComposableNodeContainerRecord, LoadNodeRecord, NodeRecord},
    substitution::LaunchContext,
};
use std::{cell::RefCell, collections::HashMap};

impl NodeCapture {
    /// Convert to NodeRecord and generate command line
    ///
    /// # Arguments
    /// * `global_params` - Global ROS parameters from SetParameter actions (passed from context)
    pub fn to_record(&self, global_params: &Option<Vec<(String, String)>>) -> Result<NodeRecord> {
        // Generate ROS command line (now includes global params)
        let cmd = self.generate_command(global_params);

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
            exec_name: Some(self.executable.clone()), // exec_name = executable name
            executable: self.executable.clone(),
            global_params: global_params.clone(),
            name: self.name.clone(),
            namespace: self.namespace.clone(),
            package: Some(self.package.clone()),
            params: self
                .parameters
                .iter()
                .map(|(k, v)| {
                    (
                        k.clone(),
                        crate::record::generator::normalize_param_value(v),
                    )
                })
                .collect(),
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
        // 1. Base command: Try to resolve full executable path, fallback to ros2 run
        let mut cmd =
            if let Some(exec_path) = find_package_executable(&self.package, &self.executable) {
                vec![exec_path]
            } else {
                // Fallback to ros2 run if path resolution fails
                log::warn!(
                    "Could not resolve executable path for {}/{}, using 'ros2 run'",
                    self.package,
                    self.executable
                );
                vec![
                    "ros2".to_string(),
                    "run".to_string(),
                    self.package.clone(),
                    self.executable.clone(),
                ]
            };

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

        // 7. Parameters (normalize booleans to Python convention: True/False)
        for (name, value) in &self.parameters {
            cmd.push("-p".to_string());
            cmd.push(format!(
                "{}:={}",
                name,
                crate::record::generator::normalize_param_value(value)
            ));
        }

        // 8. Global parameters
        if let Some(ref params) = global_params {
            for (key, value) in params {
                cmd.push("-p".to_string());
                cmd.push(format!(
                    "{}:={}",
                    key,
                    crate::record::generator::normalize_param_value(value)
                ));
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

impl ContainerCapture {
    /// Convert to ComposableNodeContainerRecord
    ///
    /// # Arguments
    /// * `global_params` - Global ROS parameters from SetParameter actions (passed from context)
    pub fn to_record(
        &self,
        global_params: &Option<Vec<(String, String)>>,
    ) -> Result<ComposableNodeContainerRecord> {
        let package = self
            .package
            .clone()
            .unwrap_or_else(|| "rclcpp_components".to_string());
        let executable = self
            .executable
            .clone()
            .unwrap_or_else(|| "component_container".to_string());

        // Generate command if not provided
        let cmd = if self.cmd.is_empty() {
            let exec_path = find_package_executable(&package, &executable)
                .unwrap_or_else(|| format!("/opt/ros/humble/lib/{}/{}", package, executable));
            let mut cmd = vec![
                exec_path,
                "--ros-args".to_string(),
                "-r".to_string(),
                format!("__node:={}", self.name),
            ];

            // Only add namespace if non-root (matches Python parser behavior)
            if !self.namespace.is_empty() && self.namespace != "/" {
                cmd.push("-r".to_string());
                cmd.push(format!("__ns:={}", self.namespace));
            }

            // Add global parameters to command
            if let Some(ref params) = global_params {
                for (key, value) in params {
                    cmd.push("-p".to_string());
                    cmd.push(format!(
                        "{}:={}",
                        key,
                        crate::record::generator::normalize_param_value(value)
                    ));
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
            exec_name: Some(self.name.clone()),
            executable,
            global_params: global_params.clone(),
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

impl LoadNodeCapture {
    /// Convert to LoadNodeRecord
    ///
    /// # Arguments
    /// * `global_params` - Global ROS parameters from SetParameter actions (passed from context)
    pub fn to_record(
        &self,
        global_params: &Option<Vec<(String, String)>>,
    ) -> Result<LoadNodeRecord> {
        log::debug!(
            "LoadNodeCapture::to_record: node='{}', self.parameters.len()={}",
            self.node_name,
            self.parameters.len()
        );

        // Merge global parameters with node-specific parameters
        // Global parameters are added first, then node-specific parameters
        // (node-specific parameters can override global ones)
        let mut merged_params = Vec::new();

        // Add global parameters
        if let Some(ref gp) = global_params {
            log::debug!(
                "LoadNodeCapture::to_record: {} global params for node '{}'",
                gp.len(),
                self.node_name
            );
            for (key, value) in gp {
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

        log::debug!(
            "LoadNodeCapture::to_record: Final merged_params count={} for node '{}'",
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

/// Push a namespace onto the ROS namespace stack
/// Uses LaunchContext via thread-local storage
///
/// IMPORTANT: Do NOT add "/" prefix here. LaunchContext.push_namespace() handles
/// relative vs absolute namespace semantics correctly:
/// - Relative "ns1" → appended to current (e.g., "/" → "/ns1")
/// - Absolute "/ns1" → used as-is
pub fn push_ros_namespace(namespace: String) -> bool {
    with_launch_context(|ctx| {
        let depth_before = ctx.namespace_depth();
        log::debug!("Pushing ROS namespace to LaunchContext: '{}'", namespace);
        ctx.push_namespace(namespace);
        let did_push = ctx.namespace_depth() > depth_before;
        if !did_push {
            log::debug!("Namespace push was a no-op (empty or root namespace)");
        }
        did_push
    })
}

/// Pop a namespace from the ROS namespace stack
/// Uses LaunchContext via thread-local storage
pub fn pop_ros_namespace() {
    with_launch_context(|ctx| {
        if ctx.namespace_depth() > 1 {
            ctx.pop_namespace();
            log::debug!("Popped ROS namespace from LaunchContext");
        } else {
            log::warn!("Attempted to pop from root namespace in LaunchContext");
        }
    });
}

/// Get the current ROS namespace
/// Uses LaunchContext via thread-local storage
pub fn get_current_ros_namespace() -> String {
    with_launch_context(|ctx| {
        let result = ctx.current_namespace();
        log::debug!("get_current_ros_namespace from LaunchContext: '{}'", result);
        result
    })
}

// ========== Thread-Local LaunchContext (unified context) ==========

thread_local! {
    /// Thread-local storage for the current LaunchContext
    /// This allows Python API classes to access the context without global state
    static CURRENT_LAUNCH_CONTEXT: RefCell<Option<*mut LaunchContext>> = const { RefCell::new(None) };
}

/// Set the current LaunchContext for this thread
/// SAFETY: The caller must ensure the LaunchContext lives for the duration of Python execution
pub fn set_current_launch_context(ctx: &mut LaunchContext) {
    CURRENT_LAUNCH_CONTEXT.with(|cell| {
        *cell.borrow_mut() = Some(ctx as *mut LaunchContext);
    });
}

/// Get the current LaunchContext for this thread
/// Returns None if no context is set
pub fn get_current_launch_context() -> Option<*mut LaunchContext> {
    CURRENT_LAUNCH_CONTEXT.with(|cell| *cell.borrow())
}

/// Clear the current LaunchContext for this thread
pub fn clear_current_launch_context() {
    CURRENT_LAUNCH_CONTEXT.with(|cell| {
        *cell.borrow_mut() = None;
    });
}

/// Execute a closure with access to the current LaunchContext
/// Panics if no context is set
pub fn with_launch_context<F, R>(f: F) -> R
where
    F: FnOnce(&mut LaunchContext) -> R,
{
    let ctx_ptr = get_current_launch_context()
        .expect("No LaunchContext set - Python execution must be called through LaunchTraverser");

    // SAFETY: The pointer is valid for the duration of Python execution
    // as guaranteed by set_current_launch_context
    unsafe { f(&mut *ctx_ptr) }
}

/// Get a clone of all captured nodes from LaunchContext
/// Panics if no context is set
pub fn get_captured_nodes() -> Vec<NodeCapture> {
    with_launch_context(|ctx| ctx.captured_nodes().to_vec())
}

/// Get a clone of all captured containers from LaunchContext
/// Panics if no context is set
pub fn get_captured_containers() -> Vec<ContainerCapture> {
    with_launch_context(|ctx| ctx.captured_containers().to_vec())
}

/// Get a clone of all captured load_nodes from LaunchContext
/// Panics if no context is set
pub fn get_captured_load_nodes() -> Vec<LoadNodeCapture> {
    with_launch_context(|ctx| ctx.captured_load_nodes().to_vec())
}

/// Get a clone of all captured includes from LaunchContext
/// Panics if no context is set
pub fn get_captured_includes() -> Vec<IncludeCapture> {
    with_launch_context(|ctx| ctx.captured_includes().to_vec())
}

/// Update captured nodes in LaunchContext by applying a mutation function
/// Panics if no context is set
pub fn update_captured_nodes<F>(f: F)
where
    F: FnOnce(&mut Vec<NodeCapture>),
{
    with_launch_context(|ctx| {
        let nodes = ctx.captured_nodes_mut();
        f(nodes);
    })
}

/// Update captured containers in LaunchContext by applying a mutation function
/// Panics if no context is set
pub fn update_captured_containers<F>(f: F)
where
    F: FnOnce(&mut Vec<ContainerCapture>),
{
    with_launch_context(|ctx| {
        let containers = ctx.captured_containers_mut();
        f(containers);
    })
}

/// Update captured load_nodes in LaunchContext by applying a mutation function
/// Panics if no context is set
pub fn update_captured_load_nodes<F>(f: F)
where
    F: FnOnce(&mut Vec<LoadNodeCapture>),
{
    with_launch_context(|ctx| {
        let load_nodes = ctx.captured_load_nodes_mut();
        f(load_nodes);
    })
}

// ========== Helper Functions ==========

/// Find ROS 2 package executable path
///
/// Searches for the executable in the ROS 2 package lib directory.
/// Returns the full path to the executable if found.
///
/// Search order:
/// 1. `{AMENT_PREFIX_PATH}/{package}/lib/{package}/{executable}`
/// 2. `/opt/ros/{ROS_DISTRO}/lib/{package}/{executable}`
/// 3. Common ROS 2 distributions (jazzy, iron, humble, galactic, foxy)
pub(crate) fn find_package_executable(package_name: &str, executable: &str) -> Option<String> {
    // Try AMENT_PREFIX_PATH first (for local installs)
    if let Ok(prefix_path) = std::env::var("AMENT_PREFIX_PATH") {
        for prefix in prefix_path.split(':') {
            let exec_path = format!("{}/lib/{}/{}", prefix, package_name, executable);
            if std::path::Path::new(&exec_path).exists() {
                log::debug!("Resolved executable: {} -> {}", executable, exec_path);
                return Some(exec_path);
            }
        }
    }

    // Try ROS_DISTRO environment variable
    if let Ok(distro) = std::env::var("ROS_DISTRO") {
        let exec_path = format!("/opt/ros/{}/lib/{}/{}", distro, package_name, executable);
        if std::path::Path::new(&exec_path).exists() {
            log::debug!("Resolved executable: {} -> {}", executable, exec_path);
            return Some(exec_path);
        }
    }

    // Fallback: Try common ROS 2 distributions
    for distro in &["jazzy", "iron", "humble", "galactic", "foxy"] {
        let exec_path = format!("/opt/ros/{}/lib/{}/{}", distro, package_name, executable);
        if std::path::Path::new(&exec_path).exists() {
            log::debug!("Resolved executable: {} -> {}", executable, exec_path);
            return Some(exec_path);
        }
    }

    log::debug!(
        "Could not resolve executable path for {}/{}",
        package_name,
        executable
    );
    None
}

// ========== Capture Functions (use LaunchContext via thread-local) ==========

/// Capture a node to LaunchContext
pub fn capture_node(node: NodeCapture) {
    with_launch_context(|ctx| ctx.capture_node(node));
}

/// Capture a container to LaunchContext
pub fn capture_container(container: ContainerCapture) {
    with_launch_context(|ctx| ctx.capture_container(container));
}

/// Capture a load_node to LaunchContext
pub fn capture_load_node(load_node: LoadNodeCapture) {
    with_launch_context(|ctx| ctx.capture_load_node(load_node));
}

/// Capture an include to LaunchContext
pub fn capture_include(include: IncludeCapture) {
    with_launch_context(|ctx| ctx.capture_include(include));
}
