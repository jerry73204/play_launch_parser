//! Mock `launch_ros` module classes

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

use crate::python::bridge::{
    ContainerCapture, LoadNodeCapture, NodeCapture, CAPTURED_CONTAINERS, CAPTURED_LOAD_NODES,
    CAPTURED_NODES,
};
use pyo3::{
    prelude::*,
    types::{PyAny, PyDict, PyList},
};

/// Mock Node class
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import Node
/// node = Node(
///     package='my_package',
///     executable='my_executable',
///     name='my_node',
///     namespace='/my_namespace',
///     parameters=[...],
///     remappings=[...],
///     arguments=[...],
/// )
/// ```
///
/// When constructed, automatically captures the node definition.
#[pyclass(module = "launch_ros.actions")]
#[derive(Clone)]
pub struct Node {
    package: String,
    executable: String,
    name: Option<String>,
    namespace: Option<String>,
    parameters: Vec<PyObject>,
    remappings: Vec<PyObject>,
    arguments: Vec<String>,
    env_vars: Vec<(String, String)>,
    #[allow(dead_code)] // Used for condition evaluation, not stored in captures
    condition: Option<PyObject>,
}

#[pymethods]
impl Node {
    #[new]
    #[pyo3(signature = (
        *,
        package,
        executable,
        name=None,
        namespace=None,
        parameters=None,
        remappings=None,
        arguments=None,
        env=None,
        condition=None,
        **_kwargs
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        py: Python,
        package: PyObject,
        executable: PyObject,
        name: Option<PyObject>,
        namespace: Option<PyObject>,
        parameters: Option<Vec<PyObject>>,
        remappings: Option<Vec<PyObject>>,
        arguments: Option<Vec<PyObject>>,
        env: Option<Vec<(String, String)>>,
        condition: Option<PyObject>,
        _kwargs: Option<&PyDict>,
    ) -> PyResult<Self> {
        // Convert PyObjects to strings (handles both strings and substitutions)
        log::debug!("Node::new: creating node with package and executable");
        let package = Self::pyobject_to_string(py, &package)?;
        let executable = Self::pyobject_to_string(py, &executable)?;

        log::debug!(
            "Node::new: package='{}', executable='{}'",
            package,
            executable
        );

        let name = name
            .map(|obj| {
                log::debug!("Node::new: processing name parameter");
                Self::pyobject_to_string(py, &obj)
            })
            .transpose()?;

        let namespace = namespace
            .map(|obj| {
                log::debug!("Node::new: processing namespace parameter");
                Self::pyobject_to_string(py, &obj)
            })
            .transpose()?;

        // Convert arguments from Vec<PyObject> to Vec<String>
        let arguments_vec = if let Some(args) = arguments {
            args.iter()
                .map(|obj| Self::pyobject_to_string(py, obj))
                .collect::<Result<Vec<String>, _>>()?
        } else {
            Vec::new()
        };

        log::debug!("Node::new: name={:?}, namespace={:?}", name, namespace);

        let node = Self {
            package: package.clone(),
            executable: executable.clone(),
            name: name.clone(),
            namespace: namespace.clone(),
            parameters: parameters.unwrap_or_default(),
            remappings: remappings.unwrap_or_default(),
            arguments: arguments_vec,
            env_vars: env.unwrap_or_default(),
            condition: condition.clone(),
        };

        // Evaluate condition (if present) and only capture if true
        let should_capture = if let Some(cond_obj) = &condition {
            Self::evaluate_condition(py, cond_obj).unwrap_or(true)
        } else {
            true // No condition means always capture
        };

        if should_capture {
            // Capture this node immediately
            Self::capture_node(&node);
        } else {
            log::debug!(
                "Skipping node capture due to condition: {} / {}",
                package,
                executable
            );
        }

        Ok(node)
    }

    fn __repr__(&self) -> String {
        format!(
            "Node(package='{}', executable='{}', name='{}')",
            self.package,
            self.executable,
            self.name.as_deref().unwrap_or(&self.executable)
        )
    }
}

impl Node {
    /// Evaluate a condition object
    ///
    /// Calls the evaluate() method on the condition if it exists
    fn evaluate_condition(py: Python, condition: &PyObject) -> PyResult<bool> {
        let cond_ref = condition.as_ref(py);

        // Try calling evaluate() method on the condition object
        // Note: The py: Python parameter is automatically injected by pyo3,
        // so we call with no arguments from Python's perspective
        if let Ok(result) = cond_ref.call_method0("evaluate") {
            if let Ok(bool_val) = result.extract::<bool>() {
                log::debug!("Condition evaluated to: {}", bool_val);
                return Ok(bool_val);
            }
        }

        // Fallback: treat as truthy if we can't evaluate
        log::warn!("Failed to evaluate condition, defaulting to true");
        Ok(true)
    }

    /// Convert a PyObject to a string (handles both strings and substitutions)
    fn pyobject_to_string(py: Python, obj: &PyObject) -> PyResult<String> {
        crate::python::api::utils::pyobject_to_string(py, obj)
    }

    /// Capture node to global storage
    fn capture_node(node: &Node) {
        use crate::python::bridge::get_current_ros_namespace;

        // Parse parameters and remappings from Python objects
        let all_params = Python::with_gil(|py| node.parse_parameters(py).unwrap_or_default());
        let remappings = Python::with_gil(|py| node.parse_remappings(py).unwrap_or_default());

        // Separate regular parameters from parameter files
        let mut parameters = Vec::new();
        let mut params_files = Vec::new();
        for (key, value) in all_params {
            if key == "__param_file" {
                params_files.push(value);
            } else {
                parameters.push((key, value));
            }
        }

        // Get current ROS namespace and combine with node's namespace
        let ros_namespace = get_current_ros_namespace();
        let node_ns = node.namespace.as_ref().map(|ns| {
            if ns.is_empty() {
                String::new()
            } else if ns.starts_with('/') {
                ns.clone()
            } else {
                format!("/{}", ns)
            }
        });

        let full_namespace = match node_ns {
            Some(ns) => {
                if ros_namespace == "/" {
                    if ns.is_empty() {
                        None
                    } else {
                        Some(ns)
                    }
                } else if ns.is_empty() {
                    Some(ros_namespace.clone())
                } else {
                    Some(format!("{}{}", ros_namespace, ns))
                }
            }
            None => {
                if ros_namespace == "/" {
                    None
                } else {
                    Some(ros_namespace.clone())
                }
            }
        };

        let params_files_count = params_files.len();
        let capture = NodeCapture {
            package: node.package.clone(),
            executable: node.executable.clone(),
            name: node.name.clone(),
            namespace: full_namespace.clone(),
            parameters,
            params_files,
            remappings,
            arguments: node.arguments.clone(),
            env_vars: node.env_vars.clone(),
        };

        log::debug!(
            "Captured Python node: {} / {} (ros_namespace: {}, full_namespace: {:?}, params_files: {})",
            capture.package,
            capture.executable,
            ros_namespace,
            full_namespace,
            params_files_count
        );

        CAPTURED_NODES.lock().push(capture);
    }

    /// Parse Python parameters to string tuples
    ///
    /// Handles:
    /// - Dict parameters: `{'param': 'value'}` -> `[("param", "value")]`
    /// - Nested dicts: `{'ns': {'param': 'value'}}` -> `[("ns.param", "value")]`
    /// - List of dicts: `[{'p1': 'v1'}, {'p2': 'v2'}]` -> `[("p1", "v1"), ("p2", "v2")]`
    /// - Parameter files: `/path/to/params.yaml` -> special marker
    fn parse_parameters(&self, py: Python) -> PyResult<Vec<(String, String)>> {
        let mut parsed_params = Vec::new();

        for param_obj in &self.parameters {
            // Try to extract as different types
            let param_any = param_obj.as_ref(py);

            // Case 1: String (parameter file path)
            if let Ok(path) = param_any.extract::<String>() {
                // Check if it looks like a file path
                if path.contains('/') || path.ends_with(".yaml") || path.ends_with(".yml") {
                    // Mark as parameter file with special prefix
                    parsed_params.push(("__param_file".to_string(), path));
                } else {
                    // Treat as a literal parameter value
                    parsed_params.push(("value".to_string(), path));
                }
                continue;
            }

            // Case 2: Dict (single parameter dict or nested dict)
            if let Ok(dict) = param_any.downcast::<PyDict>() {
                Self::parse_dict_params(dict, "", &mut parsed_params)?;
                continue;
            }

            // Case 3: List (list of parameter dicts)
            if let Ok(list) = param_any.downcast::<PyList>() {
                for item in list.iter() {
                    if let Ok(dict) = item.downcast::<PyDict>() {
                        Self::parse_dict_params(dict, "", &mut parsed_params)?;
                    }
                }
                continue;
            }

            // Case 4: Try calling __str__ on the object (for substitutions)
            if let Ok(str_val) = param_any.call_method0("__str__") {
                if let Ok(s) = str_val.extract::<String>() {
                    parsed_params.push(("substitution".to_string(), s));
                }
            }
        }

        Ok(parsed_params)
    }

    /// Parse a Python dict into parameter tuples
    ///
    /// Handles nested dicts by using dot notation: `{"ns": {"param": "value"}}` -> `("ns.param", "value")`
    fn parse_dict_params(
        dict: &PyDict,
        prefix: &str,
        params: &mut Vec<(String, String)>,
    ) -> PyResult<()> {
        for (key, value) in dict.iter() {
            let key_str = key.extract::<String>()?;
            let full_key = if prefix.is_empty() {
                key_str.clone()
            } else {
                format!("{}.{}", prefix, key_str)
            };

            // Check if value is a nested dict
            if let Ok(nested_dict) = value.downcast::<PyDict>() {
                // Recursively parse nested dict
                Self::parse_dict_params(nested_dict, &full_key, params)?;
            } else {
                // Extract value as string (handles various Python types)
                let value_str = Self::extract_param_value(value)?;
                params.push((full_key, value_str));
            }
        }

        Ok(())
    }

    /// Extract a parameter value from a Python object
    ///
    /// Handles: str, int, float, bool, substitutions (including PythonExpression), and objects with __str__
    fn extract_param_value(value: &PyAny) -> PyResult<String> {
        use pyo3::types::{PyBool, PyList};

        // Try direct string extraction
        if let Ok(s) = value.extract::<String>() {
            return Ok(s);
        }

        // Try boolean BEFORE integer (Python bools are subclass of int)
        if value.is_instance_of::<PyBool>() {
            if let Ok(b) = value.extract::<bool>() {
                return Ok(if b { "true" } else { "false" }.to_string());
            }
        }

        // Try integer
        if let Ok(i) = value.extract::<i64>() {
            return Ok(i.to_string());
        }

        // Try float
        if let Ok(f) = value.extract::<f64>() {
            // Always format floats with decimal point to preserve type information
            if f.fract() == 0.0 && f.is_finite() {
                return Ok(format!("{:.1}", f));
            } else {
                return Ok(f.to_string());
            }
        }

        // Try list (convert to YAML-like string)
        if let Ok(list) = value.downcast::<PyList>() {
            let items: Result<Vec<String>, _> =
                list.iter().map(Self::extract_param_value).collect();
            if let Ok(items) = items {
                return Ok(format!("[{}]", items.join(", ")));
            }
        }

        // For substitutions (including PythonExpression), use the centralized utility
        // This handles evaluation of PythonExpression and other evaluating substitutions
        let py = value.py();
        let obj_py = value.to_object(py);
        crate::python::api::utils::pyobject_to_string(py, &obj_py)
    }

    /// Parse Python remappings to string tuples
    ///
    /// Handles:
    /// - Tuple pairs: `[('old_topic', 'new_topic')]` -> `[("old_topic", "new_topic")]`
    /// - LaunchConfiguration objects in tuples: `[('topic', LaunchConfiguration('name'))]`
    /// - Substitutions that need string conversion
    fn parse_remappings(&self, py: Python) -> PyResult<Vec<(String, String)>> {
        let mut parsed_remaps = Vec::new();

        for remap_obj in &self.remappings {
            let remap_any = remap_obj.as_ref(py);

            // Remappings should be tuples of (from, to)
            if let Ok(remap_tuple) = remap_any.downcast::<pyo3::types::PyTuple>() {
                if remap_tuple.len() == 2 {
                    // Extract both elements and convert to strings
                    // This handles both plain strings and LaunchConfiguration objects
                    let from_obj = remap_tuple.get_item(0)?;
                    let to_obj = remap_tuple.get_item(1)?;

                    // Convert to strings (handles LaunchConfiguration via __str__)
                    let from = if let Ok(s) = from_obj.extract::<String>() {
                        s
                    } else if let Ok(str_result) = from_obj.call_method0("__str__") {
                        str_result.extract::<String>()?
                    } else {
                        from_obj.to_string()
                    };

                    let to = if let Ok(s) = to_obj.extract::<String>() {
                        s
                    } else if let Ok(str_result) = to_obj.call_method0("__str__") {
                        str_result.extract::<String>()?
                    } else {
                        to_obj.to_string()
                    };

                    parsed_remaps.push((from, to));
                }
            }
        }

        Ok(parsed_remaps)
    }
}

/// Mock ComposableNodeContainer class
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import ComposableNodeContainer
/// from launch_ros.descriptions import ComposableNode
///
/// container = ComposableNodeContainer(
///     name='my_container',
///     namespace='/my_namespace',
///     package='rclcpp_components',
///     executable='component_container',
///     composable_node_descriptions=[
///         ComposableNode(package='pkg', plugin='Plugin', name='node'),
///     ]
/// )
/// ```
#[pyclass(module = "launch_ros.actions")]
#[derive(Clone)]
pub struct ComposableNodeContainer {
    name: String,
    namespace: Option<String>,
    #[allow(dead_code)] // Keep for API compatibility but not used in container record
    package: String,
    #[allow(dead_code)] // Keep for API compatibility but not used in container record
    executable: String,
    composable_nodes: Vec<Py<ComposableNode>>,
}

#[pymethods]
impl ComposableNodeContainer {
    #[new]
    #[pyo3(signature = (
        *,
        name,
        namespace=None,
        package,
        executable,
        composable_node_descriptions=None,
        condition=None,
        **_kwargs
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        py: Python,
        name: PyObject,
        namespace: Option<PyObject>,
        package: PyObject,
        executable: PyObject,
        composable_node_descriptions: Option<Vec<Py<ComposableNode>>>,
        condition: Option<PyObject>,
        _kwargs: Option<&PyDict>,
    ) -> PyResult<Self> {
        let composable_nodes = composable_node_descriptions.unwrap_or_default();

        // Convert PyObject parameters to strings (may be substitutions)
        log::debug!("ComposableNodeContainer::new: creating container");
        let name_str = Self::pyobject_to_string(py, &name)?;

        // Debug: Log if name is empty
        if name_str.is_empty() {
            use crate::python::bridge::LAUNCH_CONFIGURATIONS;
            let configs = LAUNCH_CONFIGURATIONS.lock();
            log::warn!(
                "ComposableNodeContainer has empty name! Available configs: {:?}",
                configs.keys().collect::<Vec<_>>()
            );
        }

        let namespace_str = namespace
            .map(|ns| {
                log::debug!("ComposableNodeContainer::new: processing namespace parameter");
                Self::pyobject_to_string(py, &ns)
            })
            .transpose()?;
        let package_str = Self::pyobject_to_string(py, &package)?;
        let executable_str = Self::pyobject_to_string(py, &executable)?;

        log::debug!(
            "ComposableNodeContainer::new: name='{}', namespace={:?}, package='{}', executable='{}'",
            name_str, namespace_str, package_str, executable_str
        );

        let container = Self {
            name: name_str.clone(),
            namespace: namespace_str.clone(),
            package: package_str,
            executable: executable_str,
            composable_nodes: composable_nodes.clone(),
        };

        // Evaluate condition (if present) and only capture if true
        let should_capture = if let Some(cond_obj) = &condition {
            let result = Self::evaluate_condition(py, cond_obj).unwrap_or(true);
            log::debug!(
                "ComposableNodeContainer name='{}' namespace={:?}: condition evaluated to {}",
                name_str,
                namespace_str,
                result
            );
            result
        } else {
            log::debug!(
                "ComposableNodeContainer name='{}' namespace={:?}: no condition, capturing",
                name_str,
                namespace_str
            );
            true // No condition means always capture
        };

        if should_capture {
            // Capture the container
            Self::capture_container(&container);
            log::debug!("Captured ComposableNodeContainer '{}'", name_str);
        } else {
            log::debug!(
                "Skipping container capture due to condition: {} (namespace: {:?})",
                name_str,
                namespace_str
            );
        }

        Ok(container)
    }

    fn __repr__(&self) -> String {
        format!(
            "ComposableNodeContainer(name='{}', namespace='{}')",
            self.name,
            self.namespace.as_deref().unwrap_or("/")
        )
    }
}

impl ComposableNodeContainer {
    fn capture_container(container: &ComposableNodeContainer) {
        use crate::python::bridge::get_current_ros_namespace;

        // Get current ROS namespace from the stack
        let ros_namespace = get_current_ros_namespace();
        log::trace!(
            "Container capture: ros_namespace from stack: '{}'",
            ros_namespace
        );
        log::trace!(
            "Container capture: container.namespace: {:?}",
            container.namespace
        );

        // Normalize container's namespace
        let container_ns = container
            .namespace
            .as_ref()
            .map(|ns| {
                if ns.is_empty() {
                    String::new()
                } else if ns.starts_with('/') {
                    ns.clone()
                } else {
                    format!("/{}", ns)
                }
            })
            .unwrap_or_default();

        log::trace!(
            "Container capture: normalized container_ns: '{}'",
            container_ns
        );

        // Combine ROS namespace with container namespace
        let full_namespace = if ros_namespace == "/" {
            if container_ns.is_empty() {
                "/".to_string()
            } else {
                container_ns
            }
        } else if container_ns.is_empty() {
            ros_namespace.clone()
        } else {
            format!("{}{}", ros_namespace, container_ns)
        };

        log::trace!("Container capture: full_namespace: '{}'", full_namespace);

        let capture = ContainerCapture {
            name: container.name.clone(),
            namespace: full_namespace.clone(),
            package: Some(container.package.clone()),
            executable: Some(container.executable.clone()),
            cmd: Vec::new(), // Will be generated in to_record()
        };

        log::debug!(
            "Captured Python container: {} (namespace: {}, pkg: {}, exec: {}, ros_namespace: {})",
            capture.name,
            capture.namespace,
            container.package,
            container.executable,
            ros_namespace
        );

        CAPTURED_CONTAINERS.lock().push(capture);

        // Capture each composable node as a load_node entry
        Python::with_gil(|py| {
            for node_obj in &container.composable_nodes {
                let node = node_obj.borrow(py);
                node.capture_as_load_node(&container.name, &container.namespace);
            }
        });
    }

    /// Evaluate a condition object (same logic as Node)
    fn evaluate_condition(py: Python, condition: &PyObject) -> PyResult<bool> {
        let cond_ref = condition.as_ref(py);

        log::debug!(
            "Evaluating container condition, type: {:?}",
            cond_ref.get_type().name()
        );

        // Try calling evaluate() method on the condition object
        if let Ok(result) = cond_ref.call_method0("evaluate") {
            if let Ok(bool_val) = result.extract::<bool>() {
                log::debug!("Container condition evaluated to: {}", bool_val);
                return Ok(bool_val);
            }
        }

        // Fallback: treat as truthy if we can't evaluate
        log::warn!("Failed to evaluate container condition, defaulting to true");
        Ok(true)
    }

    /// Convert PyObject to string (handles both strings and substitutions)
    fn pyobject_to_string(py: Python, obj: &PyObject) -> PyResult<String> {
        crate::python::api::utils::pyobject_to_string(py, obj)
    }
}

/// Mock ComposableNode class (for descriptions module)
///
/// Python equivalent:
/// ```python
/// from launch_ros.descriptions import ComposableNode
///
/// node = ComposableNode(
///     package='my_package',
///     plugin='my_package::MyPlugin',
///     name='my_node',
///     namespace='/my_namespace',
///     parameters=[...],
///     remappings=[...],
/// )
/// ```
#[pyclass(module = "launch_ros.descriptions")]
#[derive(Clone)]
pub struct ComposableNode {
    package: String,
    plugin: String,
    name: String,
    namespace: Option<String>,
    parameters: Vec<PyObject>,
    remappings: Vec<PyObject>,
}

#[pymethods]
impl ComposableNode {
    #[new]
    #[pyo3(signature = (
        *,
        package,
        plugin,
        name,
        namespace=None,
        parameters=None,
        remappings=None,
        **_kwargs
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        py: Python,
        package: PyObject,
        plugin: PyObject,
        name: PyObject,
        namespace: Option<PyObject>,
        parameters: Option<Vec<PyObject>>,
        remappings: Option<Vec<PyObject>>,
        _kwargs: Option<&PyDict>,
    ) -> PyResult<Self> {
        // Convert PyObjects to strings (handles both strings and substitutions/lists)
        log::debug!("ComposableNode::new: creating composable node");
        let package_str = Self::pyobject_to_string(py, &package)?;
        let plugin_str = Self::pyobject_to_string(py, &plugin)?;
        let name_str = Self::pyobject_to_string(py, &name)?;

        let namespace_str = namespace
            .map(|ns| {
                log::debug!("ComposableNode::new: processing namespace parameter");
                Self::pyobject_to_string(py, &ns)
            })
            .transpose()?;

        log::debug!(
            "ComposableNode::new: package='{}', plugin='{}', name='{}', namespace={:?}",
            package_str,
            plugin_str,
            name_str,
            namespace_str
        );
        Ok(Self {
            package: package_str,
            plugin: plugin_str,
            name: name_str,
            namespace: namespace_str,
            parameters: parameters.unwrap_or_default(),
            remappings: remappings.unwrap_or_default(),
        })
    }

    fn __repr__(&self) -> String {
        format!(
            "ComposableNode(package='{}', plugin='{}', name='{}')",
            self.package, self.plugin, self.name
        )
    }

    // Getter methods for LoadComposableNodes to access attributes
    #[getter]
    fn package(&self) -> &str {
        &self.package
    }

    #[getter]
    fn plugin(&self) -> &str {
        &self.plugin
    }

    #[getter]
    fn name(&self) -> &str {
        &self.name
    }

    #[getter]
    fn namespace(&self) -> Option<&str> {
        self.namespace.as_deref()
    }

    #[getter]
    fn parameters(&self) -> Vec<PyObject> {
        self.parameters.clone()
    }

    #[getter]
    fn remappings(&self) -> Vec<PyObject> {
        self.remappings.clone()
    }
}

impl ComposableNode {
    /// Convert a PyObject to a string (handles both strings and substitutions)
    fn pyobject_to_string(py: Python, obj: &PyObject) -> PyResult<String> {
        crate::python::api::utils::pyobject_to_string(py, obj)
    }

    fn capture_as_load_node(&self, container_name: &str, container_namespace: &Option<String>) {
        // Parse parameters and remappings from Python objects
        let parameters = Python::with_gil(|py| self.parse_parameters(py).unwrap_or_default());
        let remappings = Python::with_gil(|py| self.parse_remappings(py).unwrap_or_default());

        // Build full target container name: namespace + name
        // Normalize to ensure consistent path format with leading slash
        let target_container_name = if let Some(ns) = container_namespace {
            Self::normalize_namespace_path(ns, container_name)
        } else {
            // If no namespace, assume root and add leading slash
            if container_name.starts_with('/') {
                container_name.to_string()
            } else {
                format!("/{}", container_name)
            }
        };

        // Use namespace from composable node if provided, otherwise inherit from container
        // Don't normalize - preserve namespace as-is from list concatenation
        let node_namespace = self
            .namespace
            .clone()
            .or_else(|| container_namespace.clone())
            .unwrap_or_else(|| "/".to_string());

        // Only add leading slash if namespace is empty (not if it just doesn't start with '/')
        // This preserves the namespace format from list concatenation
        let normalized_namespace = if node_namespace.is_empty() {
            "/".to_string()
        } else {
            node_namespace
        };

        let capture = LoadNodeCapture {
            package: self.package.clone(),
            plugin: self.plugin.clone(),
            target_container_name,
            node_name: self.name.clone(),
            namespace: normalized_namespace,
            parameters,
            remappings,
        };

        log::debug!(
            "Captured Python composable node: {} / {} (container: {})",
            capture.package,
            capture.plugin,
            capture.target_container_name
        );

        CAPTURED_LOAD_NODES.lock().push(capture);
    }

    /// Normalize namespace + name into a proper path
    fn normalize_namespace_path(namespace: &str, name: &str) -> String {
        // Handle empty name - just return namespace
        if name.is_empty() {
            return if namespace.is_empty() || namespace == "/" {
                "/".to_string()
            } else if namespace.starts_with('/') {
                namespace.to_string()
            } else {
                format!("/{}", namespace)
            };
        }

        // Handle empty namespace
        if namespace.is_empty() {
            return if name.starts_with('/') {
                name.to_string()
            } else {
                format!("/{}", name)
            };
        }

        let ns = if namespace == "/" {
            ""
        } else if namespace.starts_with('/') {
            namespace
        } else {
            return format!("/{}/{}", namespace, name);
        };

        format!("{}/{}", ns, name)
    }

    /// Parse Python parameters to string tuples (same logic as Node)
    fn parse_parameters(&self, py: Python) -> PyResult<Vec<(String, String)>> {
        let mut parsed_params = Vec::new();

        for param_obj in &self.parameters {
            let param_any = param_obj.as_ref(py);

            // String (parameter file path)
            if let Ok(path) = param_any.extract::<String>() {
                if path.contains('/') || path.ends_with(".yaml") || path.ends_with(".yml") {
                    parsed_params.push(("__param_file".to_string(), path));
                } else {
                    parsed_params.push(("value".to_string(), path));
                }
                continue;
            }

            // Dict (single parameter dict or nested dict)
            if let Ok(dict) = param_any.downcast::<PyDict>() {
                Node::parse_dict_params(dict, "", &mut parsed_params)?;
                continue;
            }

            // List (list of parameter dicts)
            if let Ok(list) = param_any.downcast::<PyList>() {
                for item in list.iter() {
                    if let Ok(dict) = item.downcast::<PyDict>() {
                        Node::parse_dict_params(dict, "", &mut parsed_params)?;
                    }
                }
                continue;
            }

            // Try calling __str__ on the object (for substitutions)
            if let Ok(str_val) = param_any.call_method0("__str__") {
                if let Ok(s) = str_val.extract::<String>() {
                    parsed_params.push(("substitution".to_string(), s));
                }
            }
        }

        Ok(parsed_params)
    }

    /// Parse Python remappings to string tuples (same logic as Node)
    ///
    /// Handles:
    /// - Tuple pairs: `[('old_topic', 'new_topic')]` -> `[("old_topic", "new_topic")]`
    /// - LaunchConfiguration objects in tuples: `[('topic', LaunchConfiguration('name'))]`
    /// - Substitutions that need string conversion
    fn parse_remappings(&self, py: Python) -> PyResult<Vec<(String, String)>> {
        let mut parsed_remaps = Vec::new();

        for remap_obj in &self.remappings {
            let remap_any = remap_obj.as_ref(py);

            // Remappings should be tuples of (from, to)
            if let Ok(remap_tuple) = remap_any.downcast::<pyo3::types::PyTuple>() {
                if remap_tuple.len() == 2 {
                    // Extract both elements and convert to strings
                    // This handles both plain strings and LaunchConfiguration objects
                    let from_obj = remap_tuple.get_item(0)?;
                    let to_obj = remap_tuple.get_item(1)?;

                    // Convert to strings (handles LaunchConfiguration via __str__)
                    let from = if let Ok(s) = from_obj.extract::<String>() {
                        s
                    } else if let Ok(str_result) = from_obj.call_method0("__str__") {
                        str_result.extract::<String>()?
                    } else {
                        from_obj.to_string()
                    };

                    let to = if let Ok(s) = to_obj.extract::<String>() {
                        s
                    } else if let Ok(str_result) = to_obj.call_method0("__str__") {
                        str_result.extract::<String>()?
                    } else {
                        to_obj.to_string()
                    };

                    parsed_remaps.push((from, to));
                }
            }
        }

        Ok(parsed_remaps)
    }
}

/// Mock SetParameter action
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import SetParameter
/// set_param = SetParameter(name='parameter_name', value='value')
/// ```
///
/// Sets a global ROS parameter that applies to all nodes
#[pyclass(module = "launch_ros.actions")]
#[derive(Clone)]
pub struct SetParameter {
    #[allow(dead_code)] // Stored for API compatibility
    name: String,
    #[allow(dead_code)] // Stored for API compatibility
    value: PyObject,
}

#[pymethods]
impl SetParameter {
    #[new]
    #[pyo3(signature = (*, name, value, **_kwargs))]
    fn new(name: String, value: PyObject, _kwargs: Option<&PyDict>) -> Self {
        log::debug!("Python Launch SetParameter: {}=<value>", name);
        // TODO: Capture this as a global parameter
        // For now, just store it but don't use it
        Self { name, value }
    }

    fn __repr__(&self) -> String {
        format!("SetParameter(name='{}')", self.name)
    }
}

/// Mock SetParametersFromFile action
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import SetParametersFromFile
///
/// SetParametersFromFile(
///     PathJoinSubstitution([FindPackageShare('my_pkg'), 'config', 'params.yaml']),
///     node_name='my_node'  # Optional - if not specified, applies to all nodes
/// )
/// ```
///
/// Loads parameters from a YAML file and applies them to nodes.
/// For static analysis, we just capture the intent without loading the file.
#[pyclass(module = "launch_ros.actions")]
#[derive(Clone)]
pub struct SetParametersFromFile {
    #[allow(dead_code)] // Stored for API compatibility
    filename: PyObject,
    #[allow(dead_code)] // Stored for API compatibility
    node_name: Option<String>,
}

#[pymethods]
impl SetParametersFromFile {
    #[new]
    #[pyo3(signature = (filename, *, node_name=None, **_kwargs))]
    fn new(filename: PyObject, node_name: Option<String>, _kwargs: Option<&PyDict>) -> Self {
        log::debug!(
            "Python Launch SetParametersFromFile: node_name={:?}",
            node_name
        );
        // For static analysis, we just capture the action
        // We don't actually load the YAML file since we're parsing structure, not executing
        Self {
            filename,
            node_name,
        }
    }

    fn __repr__(&self) -> String {
        if let Some(ref name) = self.node_name {
            format!("SetParametersFromFile(node_name='{}')", name)
        } else {
            "SetParametersFromFile(for all nodes)".to_string()
        }
    }
}

/// Mock LifecycleNode class
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import LifecycleNode
///
/// lifecycle_node = LifecycleNode(
///     package='my_package',
///     executable='my_lifecycle_node',
///     name='my_node',
///     namespace='/my_namespace',
///     output='screen'
/// )
/// ```
///
/// LifecycleNode is similar to Node but adds lifecycle management support
/// For now, we treat it the same as a regular Node
#[pyclass(module = "launch_ros.actions")]
#[derive(Clone)]
pub struct LifecycleNode {
    package: String,
    executable: String,
    name: Option<String>,
    namespace: Option<String>,
    parameters: Vec<PyObject>,
    remappings: Vec<PyObject>,
    arguments: Vec<PyObject>,
    #[allow(dead_code)] // Keep for API compatibility
    output: String,
}

#[pymethods]
impl LifecycleNode {
    #[new]
    #[pyo3(signature = (
        *,
        package,
        executable,
        name=None,
        namespace=None,
        parameters=None,
        remappings=None,
        arguments=None,
        output=None,
        **_kwargs
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        py: Python,
        package: PyObject,
        executable: PyObject,
        name: Option<PyObject>,
        namespace: Option<PyObject>,
        parameters: Option<Vec<PyObject>>,
        remappings: Option<Vec<PyObject>>,
        arguments: Option<Vec<PyObject>>,
        output: Option<String>,
        _kwargs: Option<&PyDict>,
    ) -> PyResult<Self> {
        // Convert PyObjects to strings (handles both strings and substitutions)
        let package_str = Self::pyobject_to_string_static(py, &package)?;
        let executable_str = Self::pyobject_to_string_static(py, &executable)?;

        let name_str = name
            .map(|obj| Self::pyobject_to_string_static(py, &obj))
            .transpose()?;

        let namespace_str = namespace
            .map(|obj| Self::pyobject_to_string_static(py, &obj))
            .transpose()?;

        let node = Self {
            package: package_str,
            executable: executable_str,
            name: name_str,
            namespace: namespace_str,
            parameters: parameters.unwrap_or_default(),
            remappings: remappings.unwrap_or_default(),
            arguments: arguments.unwrap_or_default(),
            output: output.unwrap_or_else(|| "screen".to_string()),
        };

        // Capture as a regular node (lifecycle management not supported in static parsing)
        Self::capture_node(&node, py)?;

        Ok(node)
    }

    fn __repr__(&self) -> String {
        format!(
            "LifecycleNode(package='{}', executable='{}', name='{}')",
            self.package,
            self.executable,
            self.name.as_deref().unwrap_or(&self.executable)
        )
    }
}

impl LifecycleNode {
    /// Convert PyObject to string (static version for use in constructor)
    fn pyobject_to_string_static(py: Python, obj: &PyObject) -> PyResult<String> {
        // Try direct string extraction
        if let Ok(s) = obj.extract::<String>(py) {
            return Ok(s);
        }

        // Try calling __str__ method (for substitutions)
        if let Ok(str_result) = obj.call_method0(py, "__str__") {
            if let Ok(s) = str_result.extract::<String>(py) {
                return Ok(s);
            }
        }

        // Fallback to repr
        Ok(obj.to_string())
    }

    /// Capture the lifecycle node as a regular NodeCapture
    fn capture_node(&self, py: Python) -> PyResult<()> {
        use crate::python::bridge::{get_current_ros_namespace, NodeCapture, CAPTURED_NODES};

        // Parse parameters (same logic as regular Node)
        let all_params = self.parse_parameters(py)?;

        // Separate regular parameters from parameter files
        let mut parsed_params = Vec::new();
        let mut params_files = Vec::new();
        for (key, value) in all_params {
            if key == "__param_file" {
                params_files.push(value);
            } else {
                parsed_params.push((key, value));
            }
        }

        // Parse remappings
        let parsed_remaps = self.parse_remappings(py)?;

        // Parse arguments
        let parsed_args = self.parse_arguments(py)?;

        // Get current ROS namespace and combine with node's namespace
        let ros_namespace = get_current_ros_namespace();
        let node_ns = self.namespace.as_ref().map(|ns| {
            if ns.is_empty() {
                String::new()
            } else if ns.starts_with('/') {
                ns.clone()
            } else {
                format!("/{}", ns)
            }
        });

        let full_namespace = match node_ns {
            Some(ns) => {
                if ros_namespace == "/" {
                    if ns.is_empty() {
                        None
                    } else {
                        Some(ns)
                    }
                } else if ns.is_empty() {
                    Some(ros_namespace.clone())
                } else {
                    Some(format!("{}{}", ros_namespace, ns))
                }
            }
            None => {
                if ros_namespace == "/" {
                    None
                } else {
                    Some(ros_namespace.clone())
                }
            }
        };

        let params_files_count = params_files.len();
        let capture = NodeCapture {
            package: self.package.clone(),
            executable: self.executable.clone(),
            name: self.name.clone(),
            namespace: full_namespace,
            parameters: parsed_params,
            params_files,
            remappings: parsed_remaps,
            arguments: parsed_args,
            env_vars: Vec::new(),
        };

        log::debug!(
            "Captured Python lifecycle node: {} / {} (ros_namespace: {}, params_files: {})",
            capture.package,
            capture.executable,
            ros_namespace,
            params_files_count
        );

        CAPTURED_NODES.lock().push(capture);
        Ok(())
    }

    /// Parse Python parameters (similar to Node)
    fn parse_parameters(&self, py: Python) -> PyResult<Vec<(String, String)>> {
        let mut parsed_params = Vec::new();

        for param_obj in &self.parameters {
            let param_any = param_obj.as_ref(py);

            // Try to extract as dict (most common case for parameters)
            if let Ok(param_dict) = param_any.downcast::<pyo3::types::PyDict>() {
                for (key, value) in param_dict.iter() {
                    let key_str = key.extract::<String>()?;
                    // Convert value to string
                    let value_str = Self::pyobject_to_string(value)?;
                    parsed_params.push((key_str, value_str));
                }
            }
            // Try to extract as string (path to YAML parameter file)
            else if let Ok(path_str) = param_any.extract::<String>() {
                // This is a parameter file path
                parsed_params.push(("__param_file".to_string(), path_str));
            }
        }

        Ok(parsed_params)
    }

    /// Parse Python remappings (similar to Node)
    fn parse_remappings(&self, py: Python) -> PyResult<Vec<(String, String)>> {
        let mut parsed_remaps = Vec::new();

        for remap_obj in &self.remappings {
            if let Ok(remap_tuple) = remap_obj.downcast::<pyo3::types::PyTuple>(py) {
                if remap_tuple.len() == 2 {
                    // Extract both elements and convert to strings
                    // This handles both plain strings and LaunchConfiguration objects
                    let from_obj = remap_tuple.get_item(0)?;
                    let to_obj = remap_tuple.get_item(1)?;

                    // Convert to strings (handles LaunchConfiguration via __str__)
                    let from = if let Ok(s) = from_obj.extract::<String>() {
                        s
                    } else if let Ok(str_result) = from_obj.call_method0("__str__") {
                        str_result.extract::<String>()?
                    } else {
                        from_obj.to_string()
                    };

                    let to = if let Ok(s) = to_obj.extract::<String>() {
                        s
                    } else if let Ok(str_result) = to_obj.call_method0("__str__") {
                        str_result.extract::<String>()?
                    } else {
                        to_obj.to_string()
                    };

                    parsed_remaps.push((from, to));
                }
            }
        }

        Ok(parsed_remaps)
    }

    /// Parse Python arguments (similar to Node)
    fn parse_arguments(&self, py: Python) -> PyResult<Vec<String>> {
        let mut parsed_args = Vec::new();

        for arg_obj in &self.arguments {
            if let Ok(arg_str) = arg_obj.extract::<String>(py) {
                parsed_args.push(arg_str);
            }
        }

        Ok(parsed_args)
    }

    /// Convert PyObject to string (handles substitutions)
    fn pyobject_to_string(obj: &PyAny) -> PyResult<String> {
        let py = obj.py();
        let py_obj = obj.into();
        crate::python::api::utils::pyobject_to_string(py, &py_obj)
    }
}

/// Mock LifecycleTransition action
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import LifecycleTransition
///
/// LifecycleTransition(
///     lifecycle_node_names=['my_lifecycle_node'],
///     transition_id=3,  # e.g., configure, activate, etc.
///     transition_label='activate'
/// )
/// ```
///
/// Triggers a lifecycle state transition for managed nodes.
/// For static analysis, we just capture the intent without executing transitions.
#[pyclass(module = "launch_ros.actions")]
#[derive(Clone)]
pub struct LifecycleTransition {
    #[allow(dead_code)] // Stored for API compatibility
    lifecycle_node_names: Vec<String>,
    #[allow(dead_code)] // Stored for API compatibility
    transition_id: Option<i32>,
    #[allow(dead_code)] // Stored for API compatibility
    transition_label: Option<String>,
}

#[pymethods]
impl LifecycleTransition {
    #[new]
    #[pyo3(signature = (*, lifecycle_node_names, transition_id=None, transition_label=None, **_kwargs))]
    fn new(
        lifecycle_node_names: Vec<String>,
        transition_id: Option<i32>,
        transition_label: Option<String>,
        _kwargs: Option<&PyDict>,
    ) -> Self {
        log::debug!(
            "Python Launch LifecycleTransition: nodes={:?}, transition={:?}",
            lifecycle_node_names,
            transition_label
        );
        // For static analysis, we just capture the action
        // We don't actually trigger lifecycle transitions
        Self {
            lifecycle_node_names,
            transition_id,
            transition_label,
        }
    }

    fn __repr__(&self) -> String {
        if let Some(ref label) = self.transition_label {
            format!(
                "LifecycleTransition(nodes={:?}, transition='{}')",
                self.lifecycle_node_names, label
            )
        } else if let Some(id) = self.transition_id {
            format!(
                "LifecycleTransition(nodes={:?}, transition_id={})",
                self.lifecycle_node_names, id
            )
        } else {
            format!("LifecycleTransition(nodes={:?})", self.lifecycle_node_names)
        }
    }
}

/// Mock PushRosNamespace action
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import PushRosNamespace
/// PushRosNamespace('my_namespace')
/// ```
///
/// Pushes a namespace onto the namespace stack
#[pyclass(module = "launch_ros.actions")]
#[derive(Clone)]
pub struct PushRosNamespace {
    #[allow(dead_code)] // Stored for API compatibility
    namespace: PyObject,
}

#[pymethods]
impl PushRosNamespace {
    #[new]
    fn new(py: Python, namespace: PyObject) -> PyResult<Self> {
        // Convert namespace to string
        let namespace_str = if let Ok(s) = namespace.extract::<String>(py) {
            s
        } else if let Ok(str_result) = namespace.call_method0(py, "__str__") {
            str_result.extract::<String>(py)?
        } else {
            namespace.to_string()
        };

        log::debug!("Python Launch PushRosNamespace: '{}'", namespace_str);

        // Push onto the namespace stack
        use crate::python::bridge::push_ros_namespace;
        push_ros_namespace(namespace_str);

        Ok(Self { namespace })
    }

    fn __repr__(&self) -> String {
        "PushRosNamespace(...)".to_string()
    }
}

/// Mock PopRosNamespace action
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import PopRosNamespace
/// PopRosNamespace()
/// ```
///
/// Pops a namespace from the namespace stack
#[pyclass(module = "launch_ros.actions")]
#[derive(Clone)]
pub struct PopRosNamespace {}

#[pymethods]
impl PopRosNamespace {
    #[new]
    fn new() -> Self {
        log::debug!("Python Launch PopRosNamespace");

        // Pop from the namespace stack
        use crate::python::bridge::pop_ros_namespace;
        pop_ros_namespace();

        Self {}
    }

    fn __repr__(&self) -> String {
        "PopRosNamespace()".to_string()
    }
}

/// Mock LoadComposableNodes action
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import LoadComposableNodes
/// from launch_ros.descriptions import ComposableNode
///
/// LoadComposableNodes(
///     target_container='container_name',
///     composable_node_descriptions=[
///         ComposableNode(
///             package='my_package',
///             plugin='my_package::MyPlugin',
///             name='my_node',
///         ),
///     ],
/// )
/// ```
///
/// Loads composable nodes into an existing container
#[pyclass(module = "launch_ros.actions")]
#[derive(Clone)]
pub struct LoadComposableNodes {
    #[allow(dead_code)] // Keep for API compatibility
    target_container: PyObject,
    composable_node_descriptions: Vec<PyObject>,
}

#[pymethods]
impl LoadComposableNodes {
    #[new]
    #[pyo3(signature = (*, target_container, composable_node_descriptions, condition=None, **_kwargs))]
    fn new(
        py: Python,
        target_container: PyObject,
        composable_node_descriptions: Vec<PyObject>,
        condition: Option<PyObject>,
        _kwargs: Option<&pyo3::types::PyDict>,
    ) -> PyResult<Self> {
        // Extract target container name for logging
        let target_str = Self::pyobject_to_string(target_container.as_ref(py))
            .unwrap_or_else(|_| "<unknown>".to_string());

        // Evaluate condition (if present) and only capture if true
        let should_capture = if let Some(cond_obj) = &condition {
            let result = Self::evaluate_condition(py, cond_obj).unwrap_or(true);
            log::debug!(
                "LoadComposableNodes target='{}': condition evaluated to {}",
                target_str,
                result
            );
            result
        } else {
            log::debug!(
                "LoadComposableNodes target='{}': no condition, capturing",
                target_str
            );
            true // No condition means always capture
        };

        if should_capture {
            // Capture the composable nodes
            Self::capture_composable_nodes(py, &target_container, &composable_node_descriptions)?;

            log::debug!(
                "Python Launch LoadComposableNodes created with {} nodes",
                composable_node_descriptions.len()
            );
        } else {
            log::debug!(
                "Skipping LoadComposableNodes capture due to condition ({} nodes)",
                composable_node_descriptions.len()
            );
        }

        Ok(Self {
            target_container,
            composable_node_descriptions,
        })
    }

    fn __repr__(&self) -> String {
        format!(
            "LoadComposableNodes({} nodes)",
            self.composable_node_descriptions.len()
        )
    }
}

impl LoadComposableNodes {
    /// Capture composable nodes from the descriptions list
    fn capture_composable_nodes(
        py: Python,
        target_container: &PyObject,
        descriptions: &[PyObject],
    ) -> PyResult<()> {
        use crate::python::bridge::{LoadNodeCapture, CAPTURED_LOAD_NODES};

        // Extract target container name and namespace
        let (container_name, container_namespace) =
            Self::extract_target_container(py, target_container)?;

        for desc_obj in descriptions {
            // Try to get the ComposableNode attributes
            let desc = desc_obj.as_ref(py);

            // Extract package (required)
            let package = if let Ok(pkg) = desc.getattr("package") {
                Self::pyobject_to_string(pkg)?
            } else {
                continue; // Skip if no package
            };

            // Extract plugin (required)
            let plugin = if let Ok(plg) = desc.getattr("plugin") {
                Self::pyobject_to_string(plg)?
            } else {
                continue; // Skip if no plugin
            };

            // Extract name (required)
            let node_name = if let Ok(n) = desc.getattr("name") {
                Self::pyobject_to_string(n)?
            } else {
                continue; // Skip if no name
            };

            // Extract namespace (optional, defaults to container namespace or "/")
            let namespace = desc
                .getattr("namespace")
                .ok()
                .and_then(|ns| {
                    // Check if it's None
                    if ns.is_none() {
                        None
                    } else {
                        Self::pyobject_to_string(ns).ok()
                    }
                })
                .or_else(|| container_namespace.clone())
                .unwrap_or_else(|| "/".to_string());

            // Extract parameters (optional)
            let parameters = if let Ok(params_obj) = desc.getattr("parameters") {
                Self::extract_parameters(params_obj)?
            } else {
                Vec::new()
            };

            // Extract remappings (optional)
            let remappings = if let Ok(remaps_obj) = desc.getattr("remappings") {
                Self::extract_remappings(remaps_obj)?
            } else {
                Vec::new()
            };

            let capture = LoadNodeCapture {
                package,
                plugin,
                target_container_name: container_name.clone(),
                node_name,
                namespace,
                parameters,
                remappings,
            };

            log::debug!(
                "Captured Python composable node from LoadComposableNodes: {} / {} (container: {})",
                capture.package,
                capture.plugin,
                capture.target_container_name
            );

            CAPTURED_LOAD_NODES.lock().push(capture);
        }

        Ok(())
    }

    /// Evaluate a condition object (same logic as Node)
    fn evaluate_condition(py: Python, condition: &PyObject) -> PyResult<bool> {
        let cond_ref = condition.as_ref(py);

        // Try calling evaluate() method on the condition object
        if let Ok(result) = cond_ref.call_method0("evaluate") {
            if let Ok(bool_val) = result.extract::<bool>() {
                log::debug!("LoadComposableNodes condition evaluated to: {}", bool_val);
                return Ok(bool_val);
            }
        }

        // Fallback: treat as truthy if we can't evaluate
        log::warn!("Failed to evaluate LoadComposableNodes condition, defaulting to true");
        Ok(true)
    }

    /// Extract target container name and namespace from a PyObject
    ///
    /// The target_container can be:
    /// - A ComposableNodeContainer instance
    /// - A string with the container name
    /// - A LaunchConfiguration or other substitution
    fn extract_target_container(
        py: Python,
        target: &PyObject,
    ) -> PyResult<(String, Option<String>)> {
        let target_ref = target.as_ref(py);

        // Try to extract as ComposableNodeContainer instance
        if let Ok(name_attr) = target_ref.getattr("name") {
            let name = Self::pyobject_to_string(name_attr)?;
            let namespace = target_ref
                .getattr("namespace")
                .ok()
                .and_then(|ns| Self::pyobject_to_string(ns).ok());

            // Build full target container name: namespace + name
            let full_name = if let Some(ref ns) = namespace {
                Self::normalize_namespace_path(ns, &name)
            } else {
                format!("/{}", name)
            };

            log::debug!(
                "Extracted target container from ComposableNodeContainer instance: {}",
                full_name
            );
            return Ok((full_name, namespace));
        }

        // Try as string, LaunchConfiguration, or other substitution
        // Use pyobject_to_string to get the substitution string for output
        let name = Self::pyobject_to_string(target_ref)?;

        log::debug!(
            "Extracted target container from string/substitution: '{}'",
            name
        );

        // For namespace extraction, we need the resolved value (not the substitution string)
        // If this is a substitution, try to resolve it using perform() with real context
        let resolved_value = if target_ref.hasattr("perform")? {
            // Import create_launch_context helper
            use crate::python::api::utils::create_launch_context;

            // Try to resolve using real context
            if let Ok(context) = create_launch_context(py) {
                if let Ok(result) = target_ref.call_method1("perform", (context,)) {
                    result.extract::<String>().ok()
                } else {
                    None
                }
            } else {
                None
            }
        } else {
            None
        };

        // Use resolved value for namespace extraction if available, otherwise use name as-is
        let value_for_namespace = resolved_value.as_ref().unwrap_or(&name);

        // Extract the namespace from the path for composable nodes to inherit
        let namespace = if value_for_namespace.contains('/') {
            // Extract namespace from the path (everything before the last /)
            if let Some(last_slash_idx) = value_for_namespace.rfind('/') {
                if last_slash_idx == 0 {
                    // Path like "/container_name" -> namespace is "/"
                    Some("/".to_string())
                } else {
                    // Path like "/ns/container_name" -> namespace is "/ns"
                    Some(value_for_namespace[..last_slash_idx].to_string())
                }
            } else {
                None
            }
        } else {
            // Simple name without slashes -> no namespace to extract
            None
        };

        log::debug!(
            "Target container: '{}' (resolved: {:?}), extracted namespace: {:?}",
            name,
            resolved_value,
            namespace
        );

        Ok((name, namespace))
    }

    /// Normalize namespace + name into a proper path
    fn normalize_namespace_path(namespace: &str, name: &str) -> String {
        // Handle empty name - just return namespace
        if name.is_empty() {
            return if namespace.is_empty() || namespace == "/" {
                "/".to_string()
            } else if namespace.starts_with('/') {
                namespace.to_string()
            } else {
                format!("/{}", namespace)
            };
        }

        // Handle empty namespace
        if namespace.is_empty() {
            return if name.starts_with('/') {
                name.to_string()
            } else {
                format!("/{}", name)
            };
        }

        let ns = if namespace == "/" {
            ""
        } else if namespace.starts_with('/') {
            namespace
        } else {
            return format!("/{}/{}", namespace, name);
        };

        format!("{}/{}", ns, name)
    }

    /// Extract parameters from Python object
    fn extract_parameters(params_obj: &PyAny) -> PyResult<Vec<(String, String)>> {
        let mut parsed_params = Vec::new();

        if let Ok(params_list) = params_obj.downcast::<pyo3::types::PyList>() {
            for param_item in params_list.iter() {
                if let Ok(param_dict) = param_item.downcast::<pyo3::types::PyDict>() {
                    for (key, value) in param_dict.iter() {
                        let key_str = key.extract::<String>()?;
                        let value_str = Self::pyobject_to_string(value)?;
                        parsed_params.push((key_str, value_str));
                    }
                } else if let Ok(path_str) = param_item.extract::<String>() {
                    parsed_params.push(("__param_file".to_string(), path_str));
                }
            }
        }

        Ok(parsed_params)
    }

    /// Extract remappings from Python object
    fn extract_remappings(remaps_obj: &PyAny) -> PyResult<Vec<(String, String)>> {
        let mut parsed_remaps = Vec::new();

        if let Ok(remaps_list) = remaps_obj.downcast::<pyo3::types::PyList>() {
            for remap_item in remaps_list.iter() {
                if let Ok(remap_tuple) = remap_item.downcast::<pyo3::types::PyTuple>() {
                    if remap_tuple.len() == 2 {
                        // Use pyobject_to_string to handle LaunchConfiguration objects
                        let from = Self::pyobject_to_string(remap_tuple.get_item(0)?)?;
                        let to = Self::pyobject_to_string(remap_tuple.get_item(1)?)?;
                        parsed_remaps.push((from, to));
                    }
                }
            }
        }

        Ok(parsed_remaps)
    }

    /// Convert PyObject to string (handles substitutions)
    fn pyobject_to_string(obj: &PyAny) -> PyResult<String> {
        let py = obj.py();
        let py_obj = obj.into();
        crate::python::api::utils::pyobject_to_string(py, &py_obj)
    }
}

/// Mock RosTimer action
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import RosTimer
/// RosTimer(period=10.0, actions=[action1, action2])
/// ```
///
/// Timer that uses ROS time instead of wall clock time.
/// Requires use_sim_time to be set for simulation time support.
#[pyclass(module = "launch_ros.actions")]
#[derive(Clone)]
pub struct RosTimer {
    #[allow(dead_code)] // Keep for future use
    period: PyObject,
    #[allow(dead_code)] // Keep for future use
    actions: Option<PyObject>,
}

#[pymethods]
impl RosTimer {
    #[new]
    #[pyo3(signature = (*, period, actions=None, **_kwargs))]
    fn new(
        py: Python,
        period: PyObject,
        actions: Option<PyObject>,
        _kwargs: Option<&pyo3::types::PyDict>,
    ) -> PyResult<Self> {
        // Convert period to string for logging
        let period_str = if let Ok(f) = period.extract::<f64>(py) {
            f.to_string()
        } else if let Ok(i) = period.extract::<i64>(py) {
            i.to_string()
        } else {
            period.to_string()
        };

        log::debug!("Python Launch RosTimer: period={}", period_str);

        Ok(Self { period, actions })
    }

    fn __repr__(&self) -> String {
        "RosTimer(...)".to_string()
    }
}

/// Mock SetUseSimTime action
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import SetUseSimTime
/// SetUseSimTime(True)
/// ```
///
/// Sets the 'use_sim_time' parameter in the current context.
/// This enables simulation time for ROS nodes.
#[pyclass(module = "launch_ros.actions")]
#[derive(Clone)]
pub struct SetUseSimTime {
    value: bool,
}

#[pymethods]
impl SetUseSimTime {
    #[new]
    fn new(value: bool) -> Self {
        log::debug!("Python Launch SetUseSimTime: {}", value);
        Self { value }
    }

    fn __repr__(&self) -> String {
        format!("SetUseSimTime({})", self.value)
    }
}

/// Mock SetRemap action
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import SetRemap
/// SetRemap(src='old_topic', dst='new_topic')
/// ```
///
/// Sets a remapping rule in the current context for nodes in the same scope.
#[pyclass(module = "launch_ros.actions")]
#[derive(Clone)]
pub struct SetRemap {
    src: String,
    dst: String,
}

#[pymethods]
impl SetRemap {
    #[new]
    #[pyo3(signature = (src, dst, **_kwargs))]
    fn new(src: String, dst: String, _kwargs: Option<&pyo3::types::PyDict>) -> Self {
        log::debug!("Python Launch SetRemap: {} -> {}", src, dst);
        Self { src, dst }
    }

    fn __repr__(&self) -> String {
        format!("SetRemap(src='{}', dst='{}')", self.src, self.dst)
    }
}

/// Mock SetROSLogDir action
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import SetROSLogDir
/// SetROSLogDir('/path/to/log/dir')
/// ```
///
/// Sets the ROS log directory for nodes launched in the same scope.
#[pyclass(module = "launch_ros.actions")]
#[derive(Clone)]
pub struct SetROSLogDir {
    #[allow(dead_code)] // Keep for future use
    log_dir: PyObject,
}

#[pymethods]
impl SetROSLogDir {
    #[new]
    fn new(log_dir: PyObject) -> Self {
        log::debug!("Python Launch SetROSLogDir: log directory provided");
        Self { log_dir }
    }

    fn __repr__(&self) -> String {
        "SetROSLogDir(...)".to_string()
    }
}
