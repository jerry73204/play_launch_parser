//! Mock `launch_ros` module classes

use crate::python::bridge::{NodeCapture, CAPTURED_NODES};
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
