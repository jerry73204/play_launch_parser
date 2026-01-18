//! Mock `launch` module classes

use pyo3::prelude::*;

/// Mock LaunchDescription class
///
/// Python equivalent:
/// ```python
/// from launch import LaunchDescription
/// launch_desc = LaunchDescription([action1, action2, ...])
/// ```
#[pyclass]
#[derive(Clone)]
pub struct LaunchDescription {
    actions: Vec<PyObject>,
}

#[pymethods]
impl LaunchDescription {
    #[new]
    fn new(actions: Vec<PyObject>) -> Self {
        Self { actions }
    }

    fn __repr__(&self) -> String {
        format!("LaunchDescription({} actions)", self.actions.len())
    }
}
