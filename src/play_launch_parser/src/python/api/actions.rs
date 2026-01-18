//! Mock `launch.actions` module classes

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

use pyo3::prelude::*;

/// Mock DeclareLaunchArgument action
///
/// Python equivalent:
/// ```python
/// from launch.actions import DeclareLaunchArgument
/// arg = DeclareLaunchArgument(
///     'variable_name',
///     default_value='default',
///     description='Description of argument'
/// )
/// ```
///
/// For now, this is a placeholder. Launch arguments are typically
/// passed via command line, not captured from Python files.
#[pyclass]
#[derive(Clone)]
pub struct DeclareLaunchArgument {
    name: String,
    #[allow(dead_code)] // Keep for future use when we fully support launch arguments
    default_value: Option<String>,
    #[allow(dead_code)] // Keep for future use when we fully support launch arguments
    description: Option<String>,
}

#[pymethods]
impl DeclareLaunchArgument {
    #[new]
    #[pyo3(signature = (name, *, default_value=None, description=None, **_kwargs))]
    fn new(
        name: String,
        default_value: Option<String>,
        description: Option<String>,
        _kwargs: Option<&pyo3::types::PyDict>,
    ) -> Self {
        Self {
            name,
            default_value,
            description,
        }
    }

    fn __repr__(&self) -> String {
        format!("DeclareLaunchArgument('{}')", self.name)
    }
}
