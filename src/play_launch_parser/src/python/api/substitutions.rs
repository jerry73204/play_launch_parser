//! Mock `launch.substitutions` module classes

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

use pyo3::prelude::*;

/// Mock LaunchConfiguration substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import LaunchConfiguration
/// config = LaunchConfiguration('variable_name')
/// ```
///
/// When converted to string, returns substitution format: `$(var variable_name)`
#[pyclass]
#[derive(Clone)]
pub struct LaunchConfiguration {
    variable_name: String,
}

#[pymethods]
impl LaunchConfiguration {
    #[new]
    fn new(variable_name: String) -> Self {
        Self { variable_name }
    }

    fn __str__(&self) -> String {
        format!("$(var {})", self.variable_name)
    }

    fn __repr__(&self) -> String {
        format!("LaunchConfiguration('{}')", self.variable_name)
    }
}

/// Mock TextSubstitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import TextSubstitution
/// text = TextSubstitution(text='literal text')
/// ```
#[pyclass]
#[derive(Clone)]
pub struct TextSubstitution {
    text: String,
}

#[pymethods]
impl TextSubstitution {
    #[new]
    #[pyo3(signature = (*, text))]
    fn new(text: String) -> Self {
        Self { text }
    }

    fn __str__(&self) -> String {
        self.text.clone()
    }

    fn __repr__(&self) -> String {
        format!("TextSubstitution(text='{}')", self.text)
    }
}

/// Mock PathJoinSubstitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import PathJoinSubstitution
/// path = PathJoinSubstitution([part1, part2, part3])
/// ```
///
/// Joins path components using '/' separator
#[pyclass]
#[derive(Clone)]
pub struct PathJoinSubstitution {
    substitutions: Vec<PyObject>,
}

#[pymethods]
impl PathJoinSubstitution {
    #[new]
    fn new(substitutions: Vec<PyObject>) -> Self {
        Self { substitutions }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        // Convert each substitution to string and join with '/'
        let mut parts = Vec::new();
        for obj in &self.substitutions {
            let part = if let Ok(s) = obj.extract::<String>(py) {
                s
            } else {
                // Try calling __str__ on the object
                obj.call_method0(py, "__str__")?.extract::<String>(py)?
            };
            parts.push(part);
        }
        Ok(parts.join("/"))
    }

    fn __repr__(&self) -> String {
        format!("PathJoinSubstitution({} parts)", self.substitutions.len())
    }
}

/// Mock FindPackageShare substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import FindPackageShare
/// pkg_path = FindPackageShare('package_name')
/// # or
/// pkg_path = FindPackageShare(LaunchConfiguration('pkg_name'))
/// ```
///
/// Returns substitution format: `$(find-pkg-share package_name)`
#[pyclass]
#[derive(Clone)]
pub struct FindPackageShare {
    package_name: PyObject,
}

#[pymethods]
impl FindPackageShare {
    #[new]
    fn new(package_name: PyObject) -> Self {
        Self { package_name }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        // Extract package name (could be string or substitution)
        let pkg = if let Ok(s) = self.package_name.extract::<String>(py) {
            s
        } else {
            // Try calling __str__ on the object (for LaunchConfiguration, etc.)
            self.package_name
                .call_method0(py, "__str__")?
                .extract::<String>(py)?
        };
        Ok(format!("$(find-pkg-share {})", pkg))
    }

    fn __repr__(&self) -> String {
        "FindPackageShare(...)".to_string()
    }
}

/// Mock EnvironmentVariable substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import EnvironmentVariable
/// env_var = EnvironmentVariable('VAR_NAME')
/// # or with default
/// env_var = EnvironmentVariable('VAR_NAME', default_value='default')
/// ```
///
/// Returns substitution format: `$(env VAR_NAME)` or `$(optenv VAR_NAME default)`
#[pyclass]
#[derive(Clone)]
pub struct EnvironmentVariable {
    name: String,
    default_value: Option<String>,
}

#[pymethods]
impl EnvironmentVariable {
    #[new]
    #[pyo3(signature = (name, *, default_value=None))]
    fn new(name: String, default_value: Option<String>) -> Self {
        Self {
            name,
            default_value,
        }
    }

    fn __str__(&self) -> String {
        if let Some(default) = &self.default_value {
            format!("$(optenv {} {})", self.name, default)
        } else {
            format!("$(env {})", self.name)
        }
    }

    fn __repr__(&self) -> String {
        format!("EnvironmentVariable('{}')", self.name)
    }
}

/// Mock ThisLaunchFileDir substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import ThisLaunchFileDir
/// launch_dir = ThisLaunchFileDir()
/// ```
///
/// Returns substitution format: `$(dirname)`
#[pyclass]
#[derive(Clone)]
pub struct ThisLaunchFileDir {}

#[pymethods]
impl ThisLaunchFileDir {
    #[new]
    fn new() -> Self {
        Self {}
    }

    fn __str__(&self) -> String {
        "$(dirname)".to_string()
    }

    fn __repr__(&self) -> String {
        "ThisLaunchFileDir()".to_string()
    }
}

/// Mock PythonExpression substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import PythonExpression
/// expr = PythonExpression(["'value1' if condition else 'value2'"])
/// ```
///
/// Note: Limited support - we just return the expression as-is for now
#[pyclass]
#[derive(Clone)]
pub struct PythonExpression {
    expression: Vec<String>,
}

#[pymethods]
impl PythonExpression {
    #[new]
    fn new(expression: Vec<String>) -> Self {
        Self { expression }
    }

    fn __str__(&self) -> String {
        // For now, just return the expression joined
        // In a full implementation, this would evaluate the Python expression
        self.expression.join("")
    }

    fn __repr__(&self) -> String {
        "PythonExpression(...)".to_string()
    }
}
