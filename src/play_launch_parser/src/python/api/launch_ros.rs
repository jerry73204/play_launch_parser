//! Mock `launch_ros` module classes

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

use crate::python::bridge::{
    ContainerCapture, LoadNodeCapture, NodeCapture, CAPTURED_CONTAINERS, CAPTURED_LOAD_NODES,
    CAPTURED_NODES,
};
use pyo3::prelude::*;
use pyo3::types::PyDict;

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
    #[allow(dead_code)] // TODO: Parse Python parameters (dict/list to string tuples)
    parameters: Vec<PyObject>,
    remappings: Vec<(String, String)>,
    arguments: Vec<String>,
    env_vars: Vec<(String, String)>,
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
        **_kwargs
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        package: String,
        executable: String,
        name: Option<String>,
        namespace: Option<String>,
        parameters: Option<Vec<PyObject>>,
        remappings: Option<Vec<(String, String)>>,
        arguments: Option<Vec<String>>,
        env: Option<Vec<(String, String)>>,
        _kwargs: Option<&PyDict>,
    ) -> PyResult<Self> {
        let node = Self {
            package: package.clone(),
            executable: executable.clone(),
            name: name.clone(),
            namespace: namespace.clone(),
            parameters: parameters.unwrap_or_default(),
            remappings: remappings.unwrap_or_default(),
            arguments: arguments.unwrap_or_default(),
            env_vars: env.unwrap_or_default(),
        };

        // Capture this node immediately
        Self::capture_node(&node);

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
    /// Capture node to global storage
    fn capture_node(node: &Node) {
        let capture = NodeCapture {
            package: node.package.clone(),
            executable: node.executable.clone(),
            name: node.name.clone(),
            namespace: node.namespace.clone(),
            parameters: Vec::new(), // TODO: Parse Python parameters
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
    #[allow(dead_code)] // TODO: Parse Python parameters (dict/list to string tuples)
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
            parameters: Vec::new(), // TODO: Parse Python parameters
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
}
