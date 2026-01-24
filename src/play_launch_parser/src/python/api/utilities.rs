//! Mock `launch_ros.utilities` module functions

use pyo3::prelude::*;

/// Mock make_namespace_absolute function
///
/// Python equivalent:
/// ```python
/// from launch_ros.utilities import make_namespace_absolute
/// abs_ns = make_namespace_absolute(namespace)
/// ```
///
/// Ensures namespace has a leading slash
#[pyfunction]
pub fn make_namespace_absolute(namespace: String) -> String {
    if namespace.is_empty() {
        "/".to_string()
    } else if namespace.starts_with('/') {
        namespace
    } else {
        format!("/{}", namespace)
    }
}

/// Mock prefix_namespace function
///
/// Python equivalent:
/// ```python
/// from launch_ros.utilities import prefix_namespace
/// prefixed = prefix_namespace(prefix, name)
/// ```
///
/// Joins a namespace prefix with a name
#[pyfunction]
pub fn prefix_namespace(prefix: String, name: String) -> String {
    let prefix = make_namespace_absolute(prefix);

    if name.is_empty() {
        prefix
    } else if name.starts_with('/') {
        name
    } else if prefix == "/" {
        format!("/{}", name)
    } else {
        format!("{}/{}", prefix, name)
    }
}

/// Register utilities module in Python
pub fn register_utilities_module(py: Python) -> PyResult<()> {
    let utilities_module = PyModule::new(py, "utilities")?;
    utilities_module.add_function(wrap_pyfunction!(make_namespace_absolute, utilities_module)?)?;
    utilities_module.add_function(wrap_pyfunction!(prefix_namespace, utilities_module)?)?;

    // Add utilities as a submodule of launch_ros
    let launch_ros = py.import("launch_ros")?;
    launch_ros.add_submodule(utilities_module)?;

    Ok(())
}
