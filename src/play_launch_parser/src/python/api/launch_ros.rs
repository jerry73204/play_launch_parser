//! Mock `launch_ros` module classes

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

use crate::python::bridge::{
    ContainerCapture, LoadNodeCapture, NodeCapture, CAPTURED_CONTAINERS, CAPTURED_LOAD_NODES,
    CAPTURED_NODES,
};
use pyo3::prelude::*;
use pyo3::types::{PyAny, PyDict, PyList};

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
#[pyclass]
#[derive(Clone)]
pub struct Node {
    package: String,
    executable: String,
    name: Option<String>,
    namespace: Option<String>,
    parameters: Vec<PyObject>,
    remappings: Vec<(String, String)>,
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
        remappings: Option<Vec<(String, String)>>,
        arguments: Option<Vec<String>>,
        env: Option<Vec<(String, String)>>,
        condition: Option<PyObject>,
        _kwargs: Option<&PyDict>,
    ) -> PyResult<Self> {
        // Convert PyObjects to strings (handles both strings and substitutions)
        let package = Self::pyobject_to_string(py, &package)?;
        let executable = Self::pyobject_to_string(py, &executable)?;
        let name = name
            .map(|obj| Self::pyobject_to_string(py, &obj))
            .transpose()?;
        let namespace = namespace
            .map(|obj| Self::pyobject_to_string(py, &obj))
            .transpose()?;

        let node = Self {
            package: package.clone(),
            executable: executable.clone(),
            name: name.clone(),
            namespace: namespace.clone(),
            parameters: parameters.unwrap_or_default(),
            remappings: remappings.unwrap_or_default(),
            arguments: arguments.unwrap_or_default(),
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
        let obj_ref = obj.as_ref(py);

        // Try direct string extraction first
        if let Ok(s) = obj_ref.extract::<String>() {
            return Ok(s);
        }

        // Try calling __str__ method (for substitutions like LaunchConfiguration)
        if let Ok(str_result) = obj_ref.call_method0("__str__") {
            if let Ok(s) = str_result.extract::<String>() {
                return Ok(s);
            }
        }

        // Fallback to repr
        Ok(obj_ref.to_string())
    }

    /// Capture node to global storage
    fn capture_node(node: &Node) {
        // Parse parameters from Python objects
        let parameters = Python::with_gil(|py| node.parse_parameters(py).unwrap_or_default());

        let capture = NodeCapture {
            package: node.package.clone(),
            executable: node.executable.clone(),
            name: node.name.clone(),
            namespace: node.namespace.clone(),
            parameters,
            remappings: node.remappings.clone(),
            arguments: node.arguments.clone(),
            env_vars: node.env_vars.clone(),
        };

        log::debug!(
            "Captured Python node: {} / {}",
            capture.package,
            capture.executable
        );

        CAPTURED_NODES.lock().unwrap().push(capture);
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
    /// Handles: str, int, float, bool, and objects with __str__
    fn extract_param_value(value: &PyAny) -> PyResult<String> {
        // Try direct string extraction
        if let Ok(s) = value.extract::<String>() {
            return Ok(s);
        }

        // Try boolean BEFORE integer (Python bools are subclass of int)
        // Use is_instance to check specifically for bool type
        if value.is_instance_of::<pyo3::types::PyBool>() {
            if let Ok(b) = value.extract::<bool>() {
                return Ok(if b {
                    "true".to_string()
                } else {
                    "false".to_string()
                });
            }
        }

        // Try integer
        if let Ok(i) = value.extract::<i64>() {
            return Ok(i.to_string());
        }

        // Try float
        if let Ok(f) = value.extract::<f64>() {
            return Ok(f.to_string());
        }

        // Try list (convert to YAML-like string)
        if let Ok(list) = value.downcast::<PyList>() {
            let items: Result<Vec<String>, _> =
                list.iter().map(Self::extract_param_value).collect();
            if let Ok(items) = items {
                return Ok(format!("[{}]", items.join(", ")));
            }
        }

        // Fallback: call __str__
        if let Ok(str_method) = value.call_method0("__str__") {
            if let Ok(s) = str_method.extract::<String>() {
                return Ok(s);
            }
        }

        // Final fallback: use repr
        Ok(value.to_string())
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
#[pyclass]
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
        **_kwargs
    ))]
    fn new(
        name: String,
        namespace: Option<String>,
        package: String,
        executable: String,
        composable_node_descriptions: Option<Vec<Py<ComposableNode>>>,
        _kwargs: Option<&PyDict>,
    ) -> PyResult<Self> {
        let composable_nodes = composable_node_descriptions.unwrap_or_default();

        let container = Self {
            name: name.clone(),
            namespace: namespace.clone(),
            package,
            executable,
            composable_nodes: composable_nodes.clone(),
        };

        // Capture the container
        Self::capture_container(&container);

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
        let capture = ContainerCapture {
            name: container.name.clone(),
            namespace: container
                .namespace
                .clone()
                .unwrap_or_else(|| "/".to_string()),
        };

        log::debug!(
            "Captured Python container: {} (namespace: {})",
            capture.name,
            capture.namespace
        );

        CAPTURED_CONTAINERS.lock().unwrap().push(capture);

        // Capture each composable node as a load_node entry
        Python::with_gil(|py| {
            for node_obj in &container.composable_nodes {
                let node = node_obj.borrow(py);
                node.capture_as_load_node(&container.name, &container.namespace);
            }
        });
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
#[pyclass]
#[derive(Clone)]
pub struct ComposableNode {
    package: String,
    plugin: String,
    name: String,
    namespace: Option<String>,
    parameters: Vec<PyObject>,
    remappings: Vec<(String, String)>,
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
    fn new(
        package: String,
        plugin: String,
        name: String,
        namespace: Option<String>,
        parameters: Option<Vec<PyObject>>,
        remappings: Option<Vec<(String, String)>>,
        _kwargs: Option<&PyDict>,
    ) -> PyResult<Self> {
        Ok(Self {
            package,
            plugin,
            name,
            namespace,
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
}

impl ComposableNode {
    fn capture_as_load_node(&self, container_name: &str, container_namespace: &Option<String>) {
        // Parse parameters from Python objects (reuse Node's logic)
        let parameters = Python::with_gil(|py| self.parse_parameters(py).unwrap_or_default());

        let capture = LoadNodeCapture {
            package: self.package.clone(),
            plugin: self.plugin.clone(),
            target_container_name: container_name.to_string(),
            node_name: self.name.clone(),
            namespace: self
                .namespace
                .clone()
                .or_else(|| container_namespace.clone())
                .unwrap_or_else(|| "/".to_string()),
            parameters,
            remappings: self.remappings.clone(),
        };

        log::debug!(
            "Captured Python composable node: {} / {} (container: {})",
            capture.package,
            capture.plugin,
            capture.target_container_name
        );

        CAPTURED_LOAD_NODES.lock().unwrap().push(capture);
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
#[pyclass]
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
