//! Mock `launch.launch_description_sources` module classes

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

use pyo3::prelude::*;

/// Mock PythonLaunchDescriptionSource class
///
/// Python equivalent:
/// ```python
/// from launch.launch_description_sources import PythonLaunchDescriptionSource
/// from launch.substitutions import PathJoinSubstitution
///
/// source = PythonLaunchDescriptionSource([
///     PathJoinSubstitution([
///         FindPackageShare('my_package'),
///         'launch',
///         'my_launch.launch.py'
///     ])
/// ])
/// ```
///
/// Represents a Python launch file source for IncludeLaunchDescription
#[pyclass]
#[derive(Clone)]
pub struct PythonLaunchDescriptionSource {
    launch_file_path: PyObject,
}

#[pymethods]
impl PythonLaunchDescriptionSource {
    #[new]
    fn new(launch_file_path: PyObject) -> Self {
        Self { launch_file_path }
    }

    /// Get the launch file path
    ///
    /// In real ROS 2, this resolves substitutions and returns the path
    pub fn get_launch_file_path(&self, py: Python) -> PyResult<String> {
        // Try to extract as string
        if let Ok(s) = self.launch_file_path.extract::<String>(py) {
            return Ok(s);
        }

        // Try to extract as list of substitutions
        if let Ok(list) = self.launch_file_path.downcast::<pyo3::types::PyList>(py) {
            let mut parts = Vec::new();
            for item in list.iter() {
                if let Ok(s) = item.extract::<String>() {
                    parts.push(s);
                } else if let Ok(str_result) = item.call_method0("__str__") {
                    parts.push(str_result.extract::<String>()?);
                }
            }
            return Ok(parts.join(""));
        }

        // Try calling __str__ method (for single substitution)
        if let Ok(str_result) = self.launch_file_path.call_method0(py, "__str__") {
            if let Ok(s) = str_result.extract::<String>(py) {
                return Ok(s);
            }
        }

        // Fallback
        Ok(self.launch_file_path.to_string())
    }

    fn __repr__(&self) -> String {
        "PythonLaunchDescriptionSource(...)".to_string()
    }
}

/// Mock XMLLaunchDescriptionSource class
///
/// Python equivalent:
/// ```python
/// from launch.launch_description_sources import XMLLaunchDescriptionSource
///
/// source = XMLLaunchDescriptionSource('/path/to/file.launch.xml')
/// ```
///
/// Represents an XML launch file source for IncludeLaunchDescription
#[pyclass]
#[derive(Clone)]
pub struct XMLLaunchDescriptionSource {
    launch_file_path: PyObject,
}

#[pymethods]
impl XMLLaunchDescriptionSource {
    #[new]
    fn new(launch_file_path: PyObject) -> Self {
        Self { launch_file_path }
    }

    /// Get the launch file path
    pub fn get_launch_file_path(&self, py: Python) -> PyResult<String> {
        // Try to extract as string
        if let Ok(s) = self.launch_file_path.extract::<String>(py) {
            return Ok(s);
        }

        // Try to extract as list of substitutions
        if let Ok(list) = self.launch_file_path.downcast::<pyo3::types::PyList>(py) {
            let mut parts = Vec::new();
            for item in list.iter() {
                if let Ok(s) = item.extract::<String>() {
                    parts.push(s);
                } else if let Ok(str_result) = item.call_method0("__str__") {
                    parts.push(str_result.extract::<String>()?);
                }
            }
            return Ok(parts.join(""));
        }

        // Try calling __str__ method
        if let Ok(str_result) = self.launch_file_path.call_method0(py, "__str__") {
            if let Ok(s) = str_result.extract::<String>(py) {
                return Ok(s);
            }
        }

        Ok(self.launch_file_path.to_string())
    }

    fn __repr__(&self) -> String {
        "XMLLaunchDescriptionSource(...)".to_string()
    }
}

/// Mock YAMLLaunchDescriptionSource class
///
/// Python equivalent:
/// ```python
/// from launch.launch_description_sources import YAMLLaunchDescriptionSource
///
/// source = YAMLLaunchDescriptionSource('/path/to/file.launch.yaml')
/// ```
///
/// Represents a YAML launch file source for IncludeLaunchDescription
#[pyclass]
#[derive(Clone)]
pub struct YAMLLaunchDescriptionSource {
    launch_file_path: PyObject,
}

#[pymethods]
impl YAMLLaunchDescriptionSource {
    #[new]
    fn new(launch_file_path: PyObject) -> Self {
        Self { launch_file_path }
    }

    /// Get the launch file path
    pub fn get_launch_file_path(&self, py: Python) -> PyResult<String> {
        // Try to extract as string
        if let Ok(s) = self.launch_file_path.extract::<String>(py) {
            return Ok(s);
        }

        // Try to extract as list of substitutions
        if let Ok(list) = self.launch_file_path.downcast::<pyo3::types::PyList>(py) {
            let mut parts = Vec::new();
            for item in list.iter() {
                if let Ok(s) = item.extract::<String>() {
                    parts.push(s);
                } else if let Ok(str_result) = item.call_method0("__str__") {
                    parts.push(str_result.extract::<String>()?);
                }
            }
            return Ok(parts.join(""));
        }

        // Try calling __str__ method
        if let Ok(str_result) = self.launch_file_path.call_method0(py, "__str__") {
            if let Ok(s) = str_result.extract::<String>(py) {
                return Ok(s);
            }
        }

        Ok(self.launch_file_path.to_string())
    }

    fn __repr__(&self) -> String {
        "YAMLLaunchDescriptionSource(...)".to_string()
    }
}
