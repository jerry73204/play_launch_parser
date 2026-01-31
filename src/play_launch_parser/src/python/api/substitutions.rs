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
#[pyclass(module = "launch.substitutions")]
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

        let configs = LAUNCH_CONFIGURATIONS.lock();
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
#[pyclass(module = "launch.substitutions")]
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
#[pyclass(module = "launch.substitutions")]
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
#[pyclass(module = "launch.substitutions")]
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
        use pyo3::types::PyList;

        // Extract package name (could be string, substitution, or list)
        let pkg = if let Ok(s) = self.package_name.extract::<String>(py) {
            s
        } else if let Ok(list) = self.package_name.downcast::<PyList>(py) {
            // Handle list case: concatenate all elements using __str__()
            let mut result = String::new();
            for item in list.iter() {
                // Try to extract as string
                if let Ok(s) = item.extract::<String>() {
                    result.push_str(&s);
                }
                // Try calling __str__
                else if let Ok(str_result) = item.call_method0("__str__") {
                    if let Ok(s) = str_result.extract::<String>() {
                        result.push_str(&s);
                    }
                }
            }
            result
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
        use pyo3::types::PyList;

        // Extract package name (could be string, LaunchConfiguration, or list)
        let pkg_name = if let Ok(s) = self.package_name.extract::<String>(py) {
            s
        } else if let Ok(list) = self.package_name.downcast::<PyList>(py) {
            // Handle list case: concatenate all elements after performing them
            let mut result = String::new();
            for item in list.iter() {
                // Try to extract as string
                if let Ok(s) = item.extract::<String>() {
                    result.push_str(&s);
                }
                // Try calling perform() if it exists (for LaunchConfiguration, etc.)
                else if item.hasattr("perform")? {
                    if let Ok(performed) = item.call_method1("perform", (_context,)) {
                        if let Ok(s) = performed.extract::<String>() {
                            result.push_str(&s);
                            continue;
                        }
                    }
                    // Fallback to __str__ if perform fails
                    if let Ok(str_result) = item.call_method0("__str__") {
                        if let Ok(s) = str_result.extract::<String>() {
                            result.push_str(&s);
                        }
                    }
                }
                // Try calling __str__
                else if let Ok(str_result) = item.call_method0("__str__") {
                    if let Ok(s) = str_result.extract::<String>() {
                        result.push_str(&s);
                    }
                }
            }
            result
        } else {
            // Try calling perform() on the object if it has it (for LaunchConfiguration)
            let obj_ref = self.package_name.as_ref(py);
            if obj_ref.hasattr("perform")? {
                if let Ok(result) = obj_ref.call_method1("perform", (_context,)) {
                    result.extract::<String>()?
                } else {
                    // Fallback to __str__
                    obj_ref.call_method0("__str__")?.extract::<String>()?
                }
            } else {
                // Fallback to __str__
                obj_ref.call_method0("__str__")?.extract::<String>()?
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
#[pyclass(module = "launch.substitutions")]
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
#[pyclass(module = "launch.substitutions")]
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
/// expr = PythonExpression(["'true' if '", LaunchConfiguration('mode'), "' == 'realtime' else 'false'"])
/// ```
///
/// Note: Limited support - we concatenate and return the expression as-is
#[pyclass(module = "launch.substitutions")]
#[derive(Clone)]
pub struct PythonExpression {
    expression: String,
}

#[pymethods]
impl PythonExpression {
    #[new]
    fn new(py: Python, expression: Vec<PyObject>) -> PyResult<Self> {
        // Convert each element to string and concatenate
        let mut result = String::new();
        for obj in expression {
            let s = Self::pyobject_to_string(py, &obj)?;
            result.push_str(&s);
        }
        Ok(Self { expression: result })
    }

    fn __str__(&self) -> String {
        // Return the concatenated expression
        self.expression.clone()
    }

    fn __repr__(&self) -> String {
        "PythonExpression(...)".to_string()
    }
}

impl PythonExpression {
    /// Convert a PyObject to a string (handles strings and substitutions)
    fn pyobject_to_string(py: Python, obj: &PyObject) -> PyResult<String> {
        use pyo3::types::PyList;
        let obj_ref = obj.as_ref(py);

        // Try direct string extraction
        if let Ok(s) = obj_ref.extract::<String>() {
            return Ok(s);
        }

        // Handle lists (concatenate)
        if let Ok(list) = obj_ref.downcast::<PyList>() {
            let mut result = String::new();
            for item in list.iter() {
                let item_str = Self::pyobject_to_string(py, &item.into())?;
                result.push_str(&item_str);
            }
            return Ok(result);
        }

        // Try perform() for LaunchConfiguration
        if obj_ref.hasattr("perform")? {
            if let Ok(context) = py.eval("type('Context', (), {})()", None, None) {
                if let Ok(result) = obj_ref.call_method1("perform", (context,)) {
                    if let Ok(s) = result.extract::<String>() {
                        return Ok(s);
                    }
                }
            }
        }

        // Try __str__
        if let Ok(str_result) = obj_ref.call_method0("__str__") {
            if let Ok(s) = str_result.extract::<String>() {
                return Ok(s);
            }
        }

        // Fallback to repr
        Ok(obj_ref.to_string())
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
#[pyclass(module = "launch.substitutions")]
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
#[pyclass(module = "launch.substitutions")]
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
#[pyclass(module = "launch.substitutions")]
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
#[pyclass(module = "launch.substitutions")]
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
#[pyclass(module = "launch.substitutions")]
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

    /// Perform the substitution - evaluate both sides and compare
    fn perform(&self, py: Python, context: &PyAny) -> PyResult<String> {
        let left_str = Self::perform_obj(&self.left, py, context)?;
        let right_str = Self::perform_obj(&self.right, py, context)?;
        Ok(if left_str == right_str {
            "true"
        } else {
            "false"
        }
        .to_string())
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

    fn perform_obj(obj: &PyObject, py: Python, context: &PyAny) -> PyResult<String> {
        let obj_ref = obj.as_ref(py);

        // Try to call perform() if available
        if obj_ref.hasattr("perform")? {
            if let Ok(result) = obj_ref.call_method1("perform", (context,)) {
                return result.extract::<String>();
            }
        }

        // Fallback to __str__
        Self::to_string(obj, py)
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
#[pyclass(module = "launch.substitutions")]
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

    /// Perform the substitution - evaluate condition and return appropriate value
    fn perform(&self, py: Python, context: &PyAny) -> PyResult<String> {
        let cond_val = Self::perform_bool(&self.condition, py, context)?;
        let obj = if cond_val {
            &self.if_value
        } else {
            &self.else_value
        };

        Self::perform_str(obj, py, context)
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

    fn perform_bool(obj: &PyObject, py: Python, context: &PyAny) -> PyResult<bool> {
        let obj_ref = obj.as_ref(py);

        // Try to call perform() if available
        let val_str = if obj_ref.hasattr("perform")? {
            if let Ok(result) = obj_ref.call_method1("perform", (context,)) {
                result.extract::<String>()?
            } else {
                Self::to_string_fallback(obj, py)?
            }
        } else {
            Self::to_string_fallback(obj, py)?
        };

        // Convert string to boolean
        Ok(matches!(
            val_str.to_lowercase().as_str(),
            "true" | "1" | "yes"
        ))
    }

    fn perform_str(obj: &PyObject, py: Python, context: &PyAny) -> PyResult<String> {
        let obj_ref = obj.as_ref(py);

        // Try to call perform() if available
        if obj_ref.hasattr("perform")? {
            if let Ok(result) = obj_ref.call_method1("perform", (context,)) {
                return result.extract::<String>();
            }
        }

        // Fallback to extract or __str__
        Self::to_string_fallback(obj, py)
    }

    fn to_string_fallback(obj: &PyObject, py: Python) -> PyResult<String> {
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

/// Mock NotEqualsSubstitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import NotEqualsSubstitution
/// not_equals_sub = NotEqualsSubstitution(left, right)
/// ```
///
/// Returns true if two values are NOT equal
#[pyclass(module = "launch.substitutions")]
#[derive(Clone)]
pub struct NotEqualsSubstitution {
    left: PyObject,
    right: PyObject,
}

#[pymethods]
impl NotEqualsSubstitution {
    #[new]
    fn new(left: PyObject, right: PyObject) -> Self {
        Self { left, right }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        let left_str = Self::to_string(&self.left, py)?;
        let right_str = Self::to_string(&self.right, py)?;
        Ok(if left_str != right_str {
            "true"
        } else {
            "false"
        }
        .to_string())
    }

    fn __repr__(&self) -> String {
        "NotEqualsSubstitution(...)".to_string()
    }

    /// Perform the substitution - evaluate both sides and compare
    fn perform(&self, py: Python, context: &PyAny) -> PyResult<String> {
        let left_str = Self::perform_obj(&self.left, py, context)?;
        let right_str = Self::perform_obj(&self.right, py, context)?;
        Ok(if left_str != right_str {
            "true"
        } else {
            "false"
        }
        .to_string())
    }
}

impl NotEqualsSubstitution {
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

    fn perform_obj(obj: &PyObject, py: Python, context: &PyAny) -> PyResult<String> {
        let obj_ref = obj.as_ref(py);

        // Try to call perform() if available
        if obj_ref.hasattr("perform")? {
            if let Ok(result) = obj_ref.call_method1("perform", (context,)) {
                return result.extract::<String>();
            }
        }

        // Fallback to __str__
        Self::to_string(obj, py)
    }
}

/// Mock FileContent substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import FileContent
/// content = FileContent(PathJoinSubstitution([...]))
/// ```
///
/// Reads file contents and returns as string
#[pyclass(module = "launch.substitutions")]
#[derive(Clone)]
pub struct FileContent {
    path: PyObject,
}

#[pymethods]
impl FileContent {
    #[new]
    fn new(path: PyObject) -> Self {
        Self { path }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        // Try to get the path as string
        let path_str = if let Ok(s) = self.path.extract::<String>(py) {
            s
        } else if let Ok(str_result) = self.path.call_method0(py, "__str__") {
            str_result.extract::<String>(py)?
        } else {
            return Ok(String::new());
        };

        // Read file contents
        match std::fs::read_to_string(&path_str) {
            Ok(content) => Ok(content),
            Err(e) => {
                log::warn!("FileContent: Failed to read file '{}': {}", path_str, e);
                // Return empty string on error (graceful degradation)
                Ok(String::new())
            }
        }
    }

    fn __repr__(&self) -> String {
        "FileContent(...)".to_string()
    }

    /// Perform the substitution - resolve path and read file contents
    fn perform(&self, py: Python, context: &PyAny) -> PyResult<String> {
        let path_obj_ref = self.path.as_ref(py);

        // Try to call perform() on the path if it has it (for substitutions)
        let path_str = if path_obj_ref.hasattr("perform")? {
            if let Ok(result) = path_obj_ref.call_method1("perform", (context,)) {
                result.extract::<String>()?
            } else {
                // Fallback to __str__
                path_obj_ref.call_method0("__str__")?.extract::<String>()?
            }
        } else if let Ok(s) = self.path.extract::<String>(py) {
            s
        } else {
            // Fallback to __str__
            path_obj_ref.call_method0("__str__")?.extract::<String>()?
        };

        log::debug!("FileContent: Reading file '{}'", path_str);

        // Read file contents
        match std::fs::read_to_string(&path_str) {
            Ok(content) => {
                log::debug!(
                    "FileContent: Successfully read {} bytes from '{}'",
                    content.len(),
                    path_str
                );
                Ok(content)
            }
            Err(e) => {
                log::warn!("FileContent: Failed to read file '{}': {}", path_str, e);
                // Return empty string on error for graceful degradation
                // This matches ROS 2 behavior where missing files don't crash the parser
                Ok(String::new())
            }
        }
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
#[pyclass(module = "launch.substitutions")]
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

/// Mock ExecutableInPackage substitution
///
/// Python equivalent:
/// ```python
/// from launch_ros.substitutions import ExecutableInPackage
/// exec_path = ExecutableInPackage(package='my_pkg', executable='my_node')
/// ```
///
/// Finds the full path to an executable within a ROS package
#[pyclass(module = "launch_ros.substitutions")]
#[derive(Clone)]
pub struct ExecutableInPackage {
    package: PyObject,
    executable: PyObject,
}

#[pymethods]
impl ExecutableInPackage {
    #[new]
    #[pyo3(signature = (package, executable, **_kwargs))]
    fn new(package: PyObject, executable: PyObject, _kwargs: Option<&pyo3::types::PyDict>) -> Self {
        Self {
            package,
            executable,
        }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        let pkg_str = Self::pyobject_to_string(&self.package, py)?;
        let exec_str = Self::pyobject_to_string(&self.executable, py)?;

        // Return a placeholder path that represents the executable location
        // In static analysis, we can't actually find the executable
        Ok(format!("$(find-exec {} {})", pkg_str, exec_str))
    }

    fn __repr__(&self, py: Python) -> String {
        let pkg_str =
            Self::pyobject_to_string(&self.package, py).unwrap_or_else(|_| "<package>".to_string());
        let exec_str = Self::pyobject_to_string(&self.executable, py)
            .unwrap_or_else(|_| "<executable>".to_string());
        format!("ExecutableInPackage('{}', '{}')", pkg_str, exec_str)
    }

    fn perform(&self, py: Python, context: &PyAny) -> PyResult<String> {
        let pkg_str = Self::perform_obj(&self.package, py, context)?;
        let exec_str = Self::perform_obj(&self.executable, py, context)?;

        // Return a placeholder path
        Ok(format!("$(find-exec {} {})", pkg_str, exec_str))
    }
}

impl ExecutableInPackage {
    fn pyobject_to_string(obj: &PyObject, py: Python) -> PyResult<String> {
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

    fn perform_obj(obj: &PyObject, py: Python, context: &PyAny) -> PyResult<String> {
        let obj_ref = obj.as_ref(py);
        if obj_ref.hasattr("perform")? {
            if let Ok(result) = obj_ref.call_method1("perform", (context,)) {
                return result.extract::<String>();
            }
        }
        Self::pyobject_to_string(obj, py)
    }
}

/// Mock FindPackage substitution
///
/// Python equivalent:
/// ```python
/// from launch_ros.substitutions import FindPackage
/// pkg_prefix = FindPackage('my_pkg')
/// ```
///
/// Finds the install prefix path of a ROS package (different from FindPackageShare)
#[pyclass(module = "launch_ros.substitutions")]
#[derive(Clone)]
pub struct FindPackage {
    package: PyObject,
}

#[pymethods]
impl FindPackage {
    #[new]
    fn new(package: PyObject) -> Self {
        Self { package }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        let pkg_str = Self::pyobject_to_string(&self.package, py)?;
        Ok(format!("$(find-pkg-prefix {})", pkg_str))
    }

    fn __repr__(&self, py: Python) -> String {
        let pkg_str =
            Self::pyobject_to_string(&self.package, py).unwrap_or_else(|_| "<package>".to_string());
        format!("FindPackage('{}')", pkg_str)
    }

    fn perform(&self, py: Python, context: &PyAny) -> PyResult<String> {
        let pkg_str = Self::perform_obj(&self.package, py, context)?;
        Ok(format!("$(find-pkg-prefix {})", pkg_str))
    }
}

impl FindPackage {
    fn pyobject_to_string(obj: &PyObject, py: Python) -> PyResult<String> {
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

    fn perform_obj(obj: &PyObject, py: Python, context: &PyAny) -> PyResult<String> {
        let obj_ref = obj.as_ref(py);
        if obj_ref.hasattr("perform")? {
            if let Ok(result) = obj_ref.call_method1("perform", (context,)) {
                return result.extract::<String>();
            }
        }
        Self::pyobject_to_string(obj, py)
    }
}

/// Mock Parameter substitution
///
/// Python equivalent:
/// ```python
/// from launch_ros.substitutions import Parameter
/// param_value = Parameter('my_parameter')
/// ```
///
/// Reads a ROS parameter value and returns it as a string
#[pyclass(module = "launch_ros.substitutions")]
#[derive(Clone)]
pub struct Parameter {
    name: PyObject,
}

#[pymethods]
impl Parameter {
    #[new]
    fn new(name: PyObject) -> Self {
        Self { name }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        let name_str = Self::pyobject_to_string(&self.name, py)?;
        Ok(format!("$(param {})", name_str))
    }

    fn __repr__(&self, py: Python) -> String {
        let name_str =
            Self::pyobject_to_string(&self.name, py).unwrap_or_else(|_| "<name>".to_string());
        format!("Parameter('{}')", name_str)
    }

    fn perform(&self, py: Python, context: &PyAny) -> PyResult<String> {
        let name_str = Self::perform_obj(&self.name, py, context)?;
        Ok(format!("$(param {})", name_str))
    }
}

impl Parameter {
    fn pyobject_to_string(obj: &PyObject, py: Python) -> PyResult<String> {
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

    fn perform_obj(obj: &PyObject, py: Python, context: &PyAny) -> PyResult<String> {
        let obj_ref = obj.as_ref(py);
        if obj_ref.hasattr("perform")? {
            if let Ok(result) = obj_ref.call_method1("perform", (context,)) {
                return result.extract::<String>();
            }
        }
        Self::pyobject_to_string(obj, py)
    }
}

/// Mock BooleanSubstitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import BooleanSubstitution
/// bool_val = BooleanSubstitution('true')
/// ```
///
/// Converts a value to a boolean string representation ("true" or "false")
#[pyclass(module = "launch.substitutions")]
#[derive(Clone)]
pub struct BooleanSubstitution {
    value: PyObject,
}

#[pymethods]
impl BooleanSubstitution {
    #[new]
    fn new(value: PyObject) -> Self {
        Self { value }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        let val_str = Self::pyobject_to_string(&self.value, py)?;
        Ok(Self::to_boolean_string(&val_str))
    }

    fn __repr__(&self, py: Python) -> String {
        let val_str =
            Self::pyobject_to_string(&self.value, py).unwrap_or_else(|_| "<value>".to_string());
        format!("BooleanSubstitution('{}')", val_str)
    }

    fn perform(&self, py: Python, context: &PyAny) -> PyResult<String> {
        let val_str = Self::perform_obj(&self.value, py, context)?;
        Ok(Self::to_boolean_string(&val_str))
    }
}

impl BooleanSubstitution {
    fn to_boolean_string(s: &str) -> String {
        let lower = s.to_lowercase();
        match lower.as_str() {
            "true" | "1" | "yes" | "on" => "true".to_string(),
            "false" | "0" | "no" | "off" | "" => "false".to_string(),
            _ => "true".to_string(), // Non-empty strings are truthy
        }
    }

    fn pyobject_to_string(obj: &PyObject, py: Python) -> PyResult<String> {
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

    fn perform_obj(obj: &PyObject, py: Python, context: &PyAny) -> PyResult<String> {
        let obj_ref = obj.as_ref(py);
        if obj_ref.hasattr("perform")? {
            if let Ok(result) = obj_ref.call_method1("perform", (context,)) {
                return result.extract::<String>();
            }
        }
        Self::pyobject_to_string(obj, py)
    }
}

/// Mock FindExecutable substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import FindExecutable
/// exec_path = FindExecutable('python3')
/// ```
///
/// Searches PATH for an executable
#[pyclass(module = "launch.substitutions")]
#[derive(Clone)]
pub struct FindExecutable {
    name: PyObject,
}

#[pymethods]
impl FindExecutable {
    #[new]
    fn new(name: PyObject) -> Self {
        Self { name }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        let name_str = Self::pyobject_to_string(&self.name, py)?;
        // Return placeholder - can't actually search PATH in static analysis
        Ok(format!("$(find-executable {})", name_str))
    }

    fn __repr__(&self, py: Python) -> String {
        let name_str =
            Self::pyobject_to_string(&self.name, py).unwrap_or_else(|_| "<name>".to_string());
        format!("FindExecutable('{}')", name_str)
    }

    fn perform(&self, py: Python, context: &PyAny) -> PyResult<String> {
        let name_str = Self::perform_obj(&self.name, py, context)?;
        Ok(format!("$(find-executable {})", name_str))
    }
}

impl FindExecutable {
    fn pyobject_to_string(obj: &PyObject, py: Python) -> PyResult<String> {
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

    fn perform_obj(obj: &PyObject, py: Python, context: &PyAny) -> PyResult<String> {
        let obj_ref = obj.as_ref(py);
        if obj_ref.hasattr("perform")? {
            if let Ok(result) = obj_ref.call_method1("perform", (context,)) {
                return result.extract::<String>();
            }
        }
        Self::pyobject_to_string(obj, py)
    }
}

/// Mock LaunchLogDir substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import LaunchLogDir
/// log_dir = LaunchLogDir()
/// ```
///
/// Returns the launch log directory path
#[pyclass(module = "launch.substitutions")]
#[derive(Clone)]
pub struct LaunchLogDir {}

#[pymethods]
impl LaunchLogDir {
    #[new]
    fn new() -> Self {
        Self {}
    }

    fn __str__(&self) -> String {
        // Return placeholder - can't determine actual log dir in static analysis
        "$(launch-log-dir)".to_string()
    }

    fn __repr__(&self) -> String {
        "LaunchLogDir()".to_string()
    }

    fn perform(&self, _py: Python, _context: &PyAny) -> PyResult<String> {
        Ok("$(launch-log-dir)".to_string())
    }
}

/// Mock ThisLaunchFile substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import ThisLaunchFile
/// this_file = ThisLaunchFile()
/// ```
///
/// Returns the full path to the current launch file
#[pyclass(module = "launch.substitutions")]
#[derive(Clone)]
pub struct ThisLaunchFile {}

#[pymethods]
impl ThisLaunchFile {
    #[new]
    fn new() -> Self {
        Self {}
    }

    fn __str__(&self) -> String {
        // Return placeholder - actual path would be set during parsing
        "$(this-launch-file)".to_string()
    }

    fn __repr__(&self) -> String {
        "ThisLaunchFile()".to_string()
    }

    fn perform(&self, _py: Python, _context: &PyAny) -> PyResult<String> {
        Ok("$(this-launch-file)".to_string())
    }
}
