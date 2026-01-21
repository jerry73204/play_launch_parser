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
    #[allow(dead_code)] // Store for API compatibility
    default: Option<String>,
}

#[pymethods]
impl LaunchConfiguration {
    #[new]
    #[pyo3(signature = (variable_name, *, default=None, **_kwargs))]
    fn new(
        variable_name: String,
        default: Option<String>,
        _kwargs: Option<&pyo3::types::PyDict>,
    ) -> Self {
        Self {
            variable_name,
            default,
        }
    }

    fn __str__(&self) -> String {
        format!("$(var {})", self.variable_name)
    }

    fn __repr__(&self) -> String {
        format!("LaunchConfiguration('{}')", self.variable_name)
    }

    /// Perform the substitution - extract the actual value from launch configurations
    ///
    /// In ROS 2, this method is called with a launch context to resolve the value.
    /// We use our global LAUNCH_CONFIGURATIONS storage instead.
    fn perform(&self, _context: &PyAny) -> PyResult<String> {
        use crate::python::bridge::LAUNCH_CONFIGURATIONS;

        let configs = LAUNCH_CONFIGURATIONS.lock().unwrap();
        let result = if let Some(value) = configs.get(&self.variable_name) {
            value.clone()
        } else if let Some(ref default) = self.default {
            default.clone()
        } else {
            // Return empty string if not found (ROS 2 behavior)
            String::new()
        };

        log::debug!(
            "LaunchConfiguration('{}').perform() -> '{}'",
            self.variable_name,
            result
        );

        Ok(result)
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

    /// Perform the substitution - resolve all parts and join with '/'
    ///
    /// This method calls perform() on each substitution if available, or falls back to __str__
    fn perform(&self, py: Python, context: &PyAny) -> PyResult<String> {
        let mut parts = Vec::new();
        for obj in &self.substitutions {
            // Try to call perform() if available (for FindPackageShare, LaunchConfiguration, etc.)
            let part = if let Ok(result) = obj.call_method1(py, "perform", (context,)) {
                result.extract::<String>(py)?
            } else if let Ok(s) = obj.extract::<String>(py) {
                s
            } else {
                // Fallback to __str__
                obj.call_method0(py, "__str__")?.extract::<String>(py)?
            };
            parts.push(part);
        }
        Ok(parts.join("/"))
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

    /// Perform the substitution - resolve to actual package path
    ///
    /// In ROS 2, this method is called with a launch context to resolve the package path.
    /// We implement the same package finding logic as the Rust substitution system.
    fn perform(&self, py: Python, _context: &PyAny) -> PyResult<String> {
        // Extract package name (could be string or LaunchConfiguration)
        let pkg_name = if let Ok(s) = self.package_name.extract::<String>(py) {
            s
        } else {
            // Try calling perform() on the object if it has it (for LaunchConfiguration)
            if let Ok(result) = self.package_name.call_method1(py, "perform", (_context,)) {
                result.extract::<String>(py)?
            } else {
                // Fallback to __str__
                self.package_name
                    .call_method0(py, "__str__")?
                    .extract::<String>(py)?
            }
        };

        // Resolve package path using ROS package finding logic
        Self::find_package_share(&pkg_name).ok_or_else(|| {
            pyo3::exceptions::PyFileNotFoundError::new_err(format!(
                "Package '{}' not found. Ensure the package is installed and sourced.",
                pkg_name
            ))
        })
    }
}

impl FindPackageShare {
    /// Find ROS 2 package share directory
    /// Same logic as the Rust substitution system
    fn find_package_share(package_name: &str) -> Option<String> {
        // Try ROS_DISTRO environment variable first
        if let Ok(distro) = std::env::var("ROS_DISTRO") {
            let share_path = format!("/opt/ros/{}/share/{}", distro, package_name);
            if std::path::Path::new(&share_path).exists() {
                return Some(share_path);
            }
        }

        // Fallback: Try common ROS 2 distributions
        for distro in &["jazzy", "iron", "humble", "galactic", "foxy"] {
            let share_path = format!("/opt/ros/{}/share/{}", distro, package_name);
            if std::path::Path::new(&share_path).exists() {
                return Some(share_path);
            }
        }

        // Try AMENT_PREFIX_PATH
        if let Ok(prefix_path) = std::env::var("AMENT_PREFIX_PATH") {
            for prefix in prefix_path.split(':') {
                let share_path = format!("{}/share/{}", prefix, package_name);
                if std::path::Path::new(&share_path).exists() {
                    return Some(share_path);
                }
            }
        }

        None
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

/// Mock Command substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import Command
/// cmd = Command(['echo', 'hello'])
/// ```
///
/// Executes a shell command and returns its output
#[pyclass]
#[derive(Clone)]
pub struct Command {
    command: Vec<PyObject>,
}

#[pymethods]
impl Command {
    #[new]
    fn new(command: Vec<PyObject>) -> Self {
        Self { command }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        // Convert command parts to strings
        let cmd_parts: Result<Vec<String>, _> = self
            .command
            .iter()
            .map(|obj| {
                if let Ok(s) = obj.extract::<String>(py) {
                    Ok(s)
                } else if let Ok(str_result) = obj.call_method0(py, "__str__") {
                    str_result.extract::<String>(py)
                } else {
                    Ok(obj.to_string())
                }
            })
            .collect();

        let parts = cmd_parts?;
        // Return as substitution format
        Ok(format!("$(command {})", parts.join(" ")))
    }

    fn __repr__(&self) -> String {
        format!("Command({} parts)", self.command.len())
    }
}

/// Mock NotSubstitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import NotSubstitution
/// not_sub = NotSubstitution(some_condition)
/// ```
///
/// Returns the boolean NOT of the input
#[pyclass]
#[derive(Clone)]
pub struct NotSubstitution {
    condition: PyObject,
}

#[pymethods]
impl NotSubstitution {
    #[new]
    fn new(condition: PyObject) -> Self {
        Self { condition }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        // Try to evaluate the condition
        if let Ok(s) = self.condition.extract::<String>(py) {
            let is_true = matches!(s.to_lowercase().as_str(), "true" | "1" | "yes");
            return Ok(if is_true { "false" } else { "true" }.to_string());
        }

        if let Ok(b) = self.condition.extract::<bool>(py) {
            return Ok(if b { "false" } else { "true" }.to_string());
        }

        // Fallback: call __str__ and negate
        if let Ok(str_result) = self.condition.call_method0(py, "__str__") {
            if let Ok(s) = str_result.extract::<String>(py) {
                let is_true = matches!(s.to_lowercase().as_str(), "true" | "1" | "yes");
                return Ok(if is_true { "false" } else { "true" }.to_string());
            }
        }

        Ok("true".to_string())
    }

    fn __repr__(&self) -> String {
        "NotSubstitution(...)".to_string()
    }
}

/// Mock AndSubstitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import AndSubstitution
/// and_sub = AndSubstitution(left, right)
/// ```
///
/// Returns the boolean AND of two inputs
#[pyclass]
#[derive(Clone)]
pub struct AndSubstitution {
    left: PyObject,
    right: PyObject,
}

#[pymethods]
impl AndSubstitution {
    #[new]
    fn new(left: PyObject, right: PyObject) -> Self {
        Self { left, right }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        let left_val = Self::to_bool(&self.left, py)?;
        let right_val = Self::to_bool(&self.right, py)?;
        Ok(if left_val && right_val {
            "true"
        } else {
            "false"
        }
        .to_string())
    }

    fn __repr__(&self) -> String {
        "AndSubstitution(...)".to_string()
    }
}

impl AndSubstitution {
    fn to_bool(obj: &PyObject, py: Python) -> PyResult<bool> {
        if let Ok(b) = obj.extract::<bool>(py) {
            return Ok(b);
        }

        if let Ok(s) = obj.extract::<String>(py) {
            return Ok(matches!(s.to_lowercase().as_str(), "true" | "1" | "yes"));
        }

        if let Ok(str_result) = obj.call_method0(py, "__str__") {
            if let Ok(s) = str_result.extract::<String>(py) {
                return Ok(matches!(s.to_lowercase().as_str(), "true" | "1" | "yes"));
            }
        }

        Ok(false)
    }
}

/// Mock OrSubstitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import OrSubstitution
/// or_sub = OrSubstitution(left, right)
/// ```
///
/// Returns the boolean OR of two inputs
#[pyclass]
#[derive(Clone)]
pub struct OrSubstitution {
    left: PyObject,
    right: PyObject,
}

#[pymethods]
impl OrSubstitution {
    #[new]
    fn new(left: PyObject, right: PyObject) -> Self {
        Self { left, right }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        let left_val = Self::to_bool(&self.left, py)?;
        let right_val = Self::to_bool(&self.right, py)?;
        Ok(if left_val || right_val {
            "true"
        } else {
            "false"
        }
        .to_string())
    }

    fn __repr__(&self) -> String {
        "OrSubstitution(...)".to_string()
    }
}

impl OrSubstitution {
    fn to_bool(obj: &PyObject, py: Python) -> PyResult<bool> {
        if let Ok(b) = obj.extract::<bool>(py) {
            return Ok(b);
        }

        if let Ok(s) = obj.extract::<String>(py) {
            return Ok(matches!(s.to_lowercase().as_str(), "true" | "1" | "yes"));
        }

        if let Ok(str_result) = obj.call_method0(py, "__str__") {
            if let Ok(s) = str_result.extract::<String>(py) {
                return Ok(matches!(s.to_lowercase().as_str(), "true" | "1" | "yes"));
            }
        }

        Ok(false)
    }
}

/// Mock EqualsSubstitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import EqualsSubstitution
/// equals_sub = EqualsSubstitution(left, right)
/// ```
///
/// Returns true if two values are equal
#[pyclass]
#[derive(Clone)]
pub struct EqualsSubstitution {
    left: PyObject,
    right: PyObject,
}

#[pymethods]
impl EqualsSubstitution {
    #[new]
    fn new(left: PyObject, right: PyObject) -> Self {
        Self { left, right }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        let left_str = Self::to_string(&self.left, py)?;
        let right_str = Self::to_string(&self.right, py)?;
        Ok(if left_str == right_str {
            "true"
        } else {
            "false"
        }
        .to_string())
    }

    fn __repr__(&self) -> String {
        "EqualsSubstitution(...)".to_string()
    }
}

impl EqualsSubstitution {
    fn to_string(obj: &PyObject, py: Python) -> PyResult<String> {
        if let Ok(s) = obj.extract::<String>(py) {
            return Ok(s);
        }

        if let Ok(str_result) = obj.call_method0(py, "__str__") {
            if let Ok(s) = str_result.extract::<String>(py) {
                return Ok(s);
            }
        }

        Ok(obj.to_string())
    }
}

/// Mock IfElseSubstitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import IfElseSubstitution
/// ifelse_sub = IfElseSubstitution(condition, if_value, else_value)
/// ```
///
/// Returns if_value if condition is true, else returns else_value
#[pyclass]
#[derive(Clone)]
pub struct IfElseSubstitution {
    condition: PyObject,
    if_value: PyObject,
    else_value: PyObject,
}

#[pymethods]
impl IfElseSubstitution {
    #[new]
    fn new(condition: PyObject, if_value: PyObject, else_value: PyObject) -> Self {
        Self {
            condition,
            if_value,
            else_value,
        }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        let cond_val = Self::to_bool(&self.condition, py)?;
        let obj = if cond_val {
            &self.if_value
        } else {
            &self.else_value
        };

        if let Ok(s) = obj.extract::<String>(py) {
            return Ok(s);
        }

        if let Ok(str_result) = obj.call_method0(py, "__str__") {
            if let Ok(s) = str_result.extract::<String>(py) {
                return Ok(s);
            }
        }

        Ok(obj.to_string())
    }

    fn __repr__(&self) -> String {
        "IfElseSubstitution(...)".to_string()
    }
}

impl IfElseSubstitution {
    fn to_bool(obj: &PyObject, py: Python) -> PyResult<bool> {
        if let Ok(b) = obj.extract::<bool>(py) {
            return Ok(b);
        }

        if let Ok(s) = obj.extract::<String>(py) {
            return Ok(matches!(s.to_lowercase().as_str(), "true" | "1" | "yes"));
        }

        if let Ok(str_result) = obj.call_method0(py, "__str__") {
            if let Ok(s) = str_result.extract::<String>(py) {
                return Ok(matches!(s.to_lowercase().as_str(), "true" | "1" | "yes"));
            }
        }

        Ok(false)
    }
}

/// Mock AnonName substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import AnonName
/// anon = AnonName('my_node')
/// ```
///
/// Generates an anonymous name with a random suffix
#[pyclass]
#[derive(Clone)]
pub struct AnonName {
    name: String,
}

#[pymethods]
impl AnonName {
    #[new]
    fn new(name: String) -> Self {
        Self { name }
    }

    fn __str__(&self) -> String {
        format!("$(anon {})", self.name)
    }

    fn __repr__(&self) -> String {
        format!("AnonName('{}')", self.name)
    }
}
