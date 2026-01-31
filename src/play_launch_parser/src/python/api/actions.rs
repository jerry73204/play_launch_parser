//! Mock `launch.actions` module classes

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

use crate::python::bridge::CAPTURED_INCLUDES;
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
#[pyclass(module = "launch.actions")]
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
        py: Python,
        name: String,
        default_value: Option<PyObject>,
        description: Option<String>,
        _kwargs: Option<&pyo3::types::PyDict>,
    ) -> PyResult<Self> {
        use crate::python::bridge::LAUNCH_CONFIGURATIONS;

        // Convert default_value PyObject to string (may be string, substitution, or list)
        let default_str = default_value
            .map(|dv| Self::pyobject_to_string(py, &dv))
            .transpose()?;

        // Register the default value in LAUNCH_CONFIGURATIONS if not already set
        if let Some(ref default_val) = default_str {
            let mut configs = LAUNCH_CONFIGURATIONS.lock();
            // Only set if not already present (CLI args and include args take precedence)
            if !configs.contains_key(&name) {
                configs.insert(name.clone(), default_val.clone());
                log::debug!(
                    "Registered launch configuration '{}' with default value '{}'",
                    name,
                    default_val
                );
            } else {
                log::debug!(
                    "Launch configuration '{}' already set to '{}', not overriding with default '{}'",
                    name,
                    configs.get(&name).unwrap(),
                    default_val
                );
            }
        }

        Ok(Self {
            name,
            default_value: default_str,
            description,
        })
    }

    fn __repr__(&self) -> String {
        format!("DeclareLaunchArgument('{}')", self.name)
    }
}

impl DeclareLaunchArgument {
    /// Convert PyObject to string (handles strings, substitutions, and lists)
    /// For substitutions, this will call perform() to resolve them
    fn pyobject_to_string(py: Python, obj: &PyObject) -> PyResult<String> {
        use pyo3::types::PyList;

        // Try direct string extraction
        if let Ok(s) = obj.extract::<String>(py) {
            return Ok(s);
        }

        // Try list (concatenate all elements after performing them)
        if let Ok(list) = obj.downcast::<PyList>(py) {
            log::trace!(
                "pyobject_to_string: processing list with {} items",
                list.len()
            );
            let mut result = String::new();
            for item in list.iter() {
                // Try to extract as string
                if let Ok(s) = item.extract::<String>() {
                    result.push_str(&s);
                }
                // Try calling perform() if it exists (for substitutions)
                else if item.hasattr("perform")? {
                    // Create a mock context for perform()
                    if let Ok(context) = py.eval("type('Context', (), {})()", None, None) {
                        match item.call_method1("perform", (context,)) {
                            Ok(performed) => {
                                if let Ok(s) = performed.extract::<String>() {
                                    result.push_str(&s);
                                    continue;
                                }
                            }
                            Err(e) => {
                                log::trace!("perform() failed: {}", e);
                            }
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
            log::trace!("pyobject_to_string: concatenated to '{}'", result);
            return Ok(result);
        }

        // For single substitution objects, try perform() first
        let obj_ref = obj.as_ref(py);
        if obj_ref.hasattr("perform")? {
            // Create a mock context for perform()
            if let Ok(context) = py.eval("type('Context', (), {})()", None, None) {
                if let Ok(performed) = obj_ref.call_method1("perform", (context,)) {
                    if let Ok(s) = performed.extract::<String>() {
                        return Ok(s);
                    }
                }
            }
        }

        // Try calling __str__ method (for substitutions)
        if let Ok(str_result) = obj.call_method0(py, "__str__") {
            if let Ok(s) = str_result.extract::<String>(py) {
                return Ok(s);
            }
        }

        // Fallback to repr
        Ok(obj.to_string())
    }
}

/// Mock OpaqueFunction action
///
/// Python equivalent:
/// ```python
/// from launch.actions import OpaqueFunction
/// def my_function(context):
///     return [Node(...), ...]
/// opaque = OpaqueFunction(function=my_function)
/// ```
///
/// This action executes a Python function and captures the returned actions.
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct OpaqueFunction {
    function: Option<PyObject>,
}

#[pymethods]
impl OpaqueFunction {
    #[new]
    #[pyo3(signature = (*, function=None, **_kwargs))]
    fn new(function: Option<PyObject>, _kwargs: Option<&pyo3::types::PyDict>) -> PyResult<Self> {
        Ok(Self { function })
    }

    fn __repr__(&self) -> String {
        "OpaqueFunction(...)".to_string()
    }

    /// Execute the function and return the result
    /// This is called by our executor
    pub fn execute(&self, py: Python) -> PyResult<PyObject> {
        log::debug!("OpaqueFunction::execute called");
        if let Some(ref func) = self.function {
            log::debug!("OpaqueFunction has function, executing it");
            // Create a mock LaunchContext that provides access to launch configurations
            // This allows OpaqueFunction code to call LaunchConfiguration().perform(context)
            use crate::python::bridge::{get_current_ros_namespace, LAUNCH_CONFIGURATIONS};
            let configs = LAUNCH_CONFIGURATIONS.lock().clone();
            let ros_namespace = get_current_ros_namespace();

            log::debug!("OpaqueFunction context: ros_namespace='{}'", ros_namespace);

            // Create a context dict with launch_configurations and ros_namespace
            let context_code = r#"
class MockLaunchContext:
    def __init__(self, launch_configurations, ros_namespace):
        self.launch_configurations = launch_configurations
        # Store ros_namespace for OpaqueFunction access (e.g. context.launch_configurations.get('ros_namespace'))
        self.launch_configurations['ros_namespace'] = ros_namespace

    def perform_substitution(self, sub):
        # Support LaunchConfiguration.perform(context)
        if hasattr(sub, 'variable_name'):
            return self.launch_configurations.get(sub.variable_name, '')
        return str(sub)

context = MockLaunchContext(launch_configurations, ros_namespace)
"#;

            // Use __main__.dict() as namespace - this is the same namespace used by exec()
            // and has access to the isolated sys.modules with our mocks
            let main_module = py.import("__main__")?;
            let namespace = main_module.dict();

            // Add launch configurations to namespace
            let configs_dict = pyo3::types::PyDict::new(py);
            for (key, value) in configs {
                configs_dict.set_item(key, value)?;
            }
            namespace.set_item("launch_configurations", configs_dict)?;

            // Add ros_namespace to namespace
            namespace.set_item("ros_namespace", ros_namespace)?;

            // Add Python builtins that OpaqueFunction might need
            // Import Path for file operations
            py.run("from pathlib import Path", Some(namespace), Some(namespace))?;
            // Import yaml for YAML loading
            py.run("import yaml", Some(namespace), Some(namespace)).ok(); // Ignore if yaml not available

            // Create the context using the code above
            py.run(context_code, Some(namespace), Some(namespace))?;
            let context = namespace.get_item("context")?.unwrap();

            // Call the function with the context
            log::debug!("Calling OpaqueFunction with context");
            let result = func.call1(py, (context,))?;

            // Log the result type
            let result_type = result.as_ref(py).get_type().name().unwrap_or("unknown");
            log::debug!("OpaqueFunction returned type: {}", result_type);

            // If it's a list, log the count
            if let Ok(list) = result.as_ref(py).extract::<Vec<PyObject>>() {
                log::debug!("OpaqueFunction returned list with {} items", list.len());
            }

            Ok(result)
        } else {
            Ok(py.None())
        }
    }
}

/// Mock LogInfo action
///
/// Python equivalent:
/// ```python
/// from launch.actions import LogInfo
/// log_action = LogInfo(msg='Information message')
/// log_action = LogInfo(msg=['Prefix: ', LaunchConfiguration('var'), ' suffix'])
/// ```
///
/// Logs an information message when the action is executed
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct LogInfo {
    msg: String,
}

#[pymethods]
impl LogInfo {
    #[new]
    #[pyo3(signature = (*, msg, **_kwargs))]
    fn new(py: Python, msg: PyObject, _kwargs: Option<&pyo3::types::PyDict>) -> PyResult<Self> {
        // Convert msg to string (handles strings, lists, LaunchConfiguration, etc.)
        let msg_str = Self::pyobject_to_string(py, &msg)?;

        // Log the message immediately
        log::info!("Python Launch LogInfo: {}", msg_str);
        Ok(Self { msg: msg_str })
    }

    fn __repr__(&self) -> String {
        format!("LogInfo(msg='{}')", self.msg)
    }
}

impl LogInfo {
    /// Convert a PyObject to a string (handles both strings and substitutions)
    /// Same logic as Node::pyobject_to_string
    fn pyobject_to_string(py: Python, obj: &PyObject) -> PyResult<String> {
        use pyo3::types::PyList;
        let obj_ref = obj.as_ref(py);

        // Try direct string extraction first
        if let Ok(s) = obj_ref.extract::<String>() {
            return Ok(s);
        }

        // Handle lists (concatenate elements)
        if let Ok(list) = obj_ref.downcast::<PyList>() {
            let mut result = String::new();
            for item in list.iter() {
                let item_str = Self::pyobject_to_string(py, &item.into())?;
                result.push_str(&item_str);
            }
            return Ok(result);
        }

        // Try calling perform() method (for LaunchConfiguration)
        if obj_ref.hasattr("perform")? {
            if let Ok(context) = py.eval("type('Context', (), {})()", None, None) {
                if let Ok(result) = obj_ref.call_method1("perform", (context,)) {
                    if let Ok(s) = result.extract::<String>() {
                        return Ok(s);
                    }
                }
            }
        }

        // Try calling __str__ method
        if let Ok(str_result) = obj_ref.call_method0("__str__") {
            if let Ok(s) = str_result.extract::<String>() {
                return Ok(s);
            }
        }

        // Fallback to repr
        Ok(obj_ref.to_string())
    }
}

/// Mock SetEnvironmentVariable action
///
/// Python equivalent:
/// ```python
/// from launch.actions import SetEnvironmentVariable
/// set_env = SetEnvironmentVariable('VAR_NAME', 'value')
/// set_env = SetEnvironmentVariable('VAR_NAME', LaunchConfiguration('var'))
/// set_env = SetEnvironmentVariable('VAR_NAME', [LaunchConfiguration('prefix'), '/suffix'])
/// ```
///
/// Sets an environment variable
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct SetEnvironmentVariable {
    name: String,
    value: String,
}

#[pymethods]
impl SetEnvironmentVariable {
    #[new]
    fn new(py: Python, name: PyObject, value: PyObject) -> PyResult<Self> {
        // Convert PyObjects to strings (handles strings, substitutions, and lists)
        let name_str = Self::pyobject_to_string(py, &name)?;
        let value_str = Self::pyobject_to_string(py, &value)?;

        log::debug!(
            "Python Launch SetEnvironmentVariable: {}={}",
            name_str,
            value_str
        );
        Ok(Self {
            name: name_str,
            value: value_str,
        })
    }

    fn __repr__(&self) -> String {
        format!("SetEnvironmentVariable('{}', '{}')", self.name, self.value)
    }
}

impl SetEnvironmentVariable {
    /// Convert a PyObject to a string (handles strings, substitutions, and lists)
    /// Reuses the same pattern as LogInfo
    fn pyobject_to_string(py: Python, obj: &PyObject) -> PyResult<String> {
        use pyo3::types::PyList;
        let obj_ref = obj.as_ref(py);

        // Try direct string extraction first
        if let Ok(s) = obj_ref.extract::<String>() {
            return Ok(s);
        }

        // Handle lists (concatenate elements)
        if let Ok(list) = obj_ref.downcast::<PyList>() {
            let mut result = String::new();
            for item in list.iter() {
                let item_str = Self::pyobject_to_string(py, &item.into())?;
                result.push_str(&item_str);
            }
            return Ok(result);
        }

        // Try calling perform() method (for LaunchConfiguration)
        if obj_ref.hasattr("perform")? {
            if let Ok(context) = py.eval("type('Context', (), {})()", None, None) {
                if let Ok(result) = obj_ref.call_method1("perform", (context,)) {
                    if let Ok(s) = result.extract::<String>() {
                        return Ok(s);
                    }
                }
            }
        }

        // Try calling __str__ method
        if let Ok(str_result) = obj_ref.call_method0("__str__") {
            if let Ok(s) = str_result.extract::<String>() {
                return Ok(s);
            }
        }

        // Fallback to repr
        Ok(obj_ref.to_string())
    }
}

/// Mock UnsetEnvironmentVariable action
///
/// Python equivalent:
/// ```python
/// from launch.actions import UnsetEnvironmentVariable
/// unset_env = UnsetEnvironmentVariable('VAR_NAME')
/// ```
///
/// Unsets an environment variable
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct UnsetEnvironmentVariable {
    name: String,
}

#[pymethods]
impl UnsetEnvironmentVariable {
    #[new]
    fn new(name: String) -> Self {
        log::debug!("Python Launch UnsetEnvironmentVariable: {}", name);
        Self { name }
    }

    fn __repr__(&self) -> String {
        format!("UnsetEnvironmentVariable('{}')", self.name)
    }
}

/// Mock GroupAction
///
/// Python equivalent:
/// ```python
/// from launch.actions import GroupAction
/// group = GroupAction(
///     actions=[action1, action2],
///     scoped=True,
///     forwarding=True
/// )
/// ```
///
/// Groups actions together with optional scoping
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct GroupAction {
    #[pyo3(get)] // Make actions directly accessible as an attribute (like LaunchDescription)
    pub actions: Vec<PyObject>,
    #[allow(dead_code)] // Keep for API compatibility
    scoped: bool,
    #[allow(dead_code)] // Keep for API compatibility
    forwarding: bool,
}

#[pymethods]
impl GroupAction {
    #[new]
    #[pyo3(signature = (actions, *, scoped=true, forwarding=true, **_kwargs))]
    fn new(
        actions: Vec<PyObject>,
        scoped: Option<bool>,
        forwarding: Option<bool>,
        _kwargs: Option<&pyo3::types::PyDict>,
    ) -> Self {
        Self {
            actions,
            scoped: scoped.unwrap_or(true),
            forwarding: forwarding.unwrap_or(true),
        }
    }

    fn __repr__(&self) -> String {
        format!("GroupAction({} actions)", self.actions.len())
    }
}

/// Mock ExecuteProcess action
///
/// Python equivalent:
/// ```python
/// from launch.actions import ExecuteProcess
/// proc = ExecuteProcess(
///     cmd=['command', 'arg1', 'arg2'],
///     cwd='/path/to/dir',
///     name='process_name',
///     output='screen'
/// )
/// # With substitutions:
/// proc = ExecuteProcess(
///     cmd=['ros2', 'run', LaunchConfiguration('package'), 'node'],
///     cwd=[FindPackageShare('my_pkg'), '/dir']
/// )
/// ```
///
/// Executes a non-ROS process
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct ExecuteProcess {
    cmd: Vec<String>,
    #[allow(dead_code)] // Keep for future use
    cwd: Option<String>,
    #[allow(dead_code)] // Keep for future use
    name: Option<String>,
    #[allow(dead_code)] // Keep for future use
    output: String,
}

#[pymethods]
impl ExecuteProcess {
    #[new]
    #[pyo3(signature = (*, cmd, cwd=None, name=None, output=None, **_kwargs))]
    fn new(
        py: Python,
        cmd: Vec<PyObject>,
        cwd: Option<PyObject>,
        name: Option<PyObject>,
        output: Option<String>,
        _kwargs: Option<&pyo3::types::PyDict>,
    ) -> PyResult<Self> {
        // Convert cmd elements to strings
        let cmd_strs: Result<Vec<String>, _> = cmd
            .iter()
            .map(|obj| Self::pyobject_to_string(py, obj))
            .collect();
        let cmd_vec = cmd_strs?;

        let cwd_str = cwd
            .map(|obj| Self::pyobject_to_string(py, &obj))
            .transpose()?;

        let name_str = name
            .map(|obj| Self::pyobject_to_string(py, &obj))
            .transpose()?;

        log::debug!("Python Launch ExecuteProcess: {:?}", cmd_vec);

        Ok(Self {
            cmd: cmd_vec,
            cwd: cwd_str,
            name: name_str,
            output: output.unwrap_or_else(|| "screen".to_string()),
        })
    }

    fn __repr__(&self) -> String {
        format!("ExecuteProcess(cmd={:?})", self.cmd)
    }
}

impl ExecuteProcess {
    /// Convert a PyObject to a string (handles strings, substitutions, and lists)
    /// Reuses the same pattern as SetEnvironmentVariable
    fn pyobject_to_string(py: Python, obj: &PyObject) -> PyResult<String> {
        use pyo3::types::PyList;
        let obj_ref = obj.as_ref(py);

        // Try direct string extraction first
        if let Ok(s) = obj_ref.extract::<String>() {
            return Ok(s);
        }

        // Handle lists (concatenate elements)
        if let Ok(list) = obj_ref.downcast::<PyList>() {
            let mut result = String::new();
            for item in list.iter() {
                let item_str = Self::pyobject_to_string(py, &item.into())?;
                result.push_str(&item_str);
            }
            return Ok(result);
        }

        // Try calling perform() method (for LaunchConfiguration)
        if obj_ref.hasattr("perform")? {
            if let Ok(context) = py.eval("type('Context', (), {})()", None, None) {
                if let Ok(result) = obj_ref.call_method1("perform", (context,)) {
                    if let Ok(s) = result.extract::<String>() {
                        return Ok(s);
                    }
                }
            }
        }

        // Try calling __str__ method
        if let Ok(str_result) = obj_ref.call_method0("__str__") {
            if let Ok(s) = str_result.extract::<String>() {
                return Ok(s);
            }
        }

        // Fallback to repr
        Ok(obj_ref.to_string())
    }
}

/// Mock ExecuteLocal action
///
/// Python equivalent:
/// ```python
/// from launch.actions import ExecuteLocal
/// ExecuteLocal(
///     process_description=Executable(cmd=['ls', '-la']),
///     output='screen'
/// )
/// ```
///
/// Executes a process on the local system with more control than ExecuteProcess
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct ExecuteLocal {
    #[allow(dead_code)] // Keep for future use
    process_description: Option<PyObject>,
    #[allow(dead_code)] // Keep for future use
    cmd: Option<Vec<String>>,
    #[allow(dead_code)] // Keep for future use
    cwd: Option<String>,
    #[allow(dead_code)] // Keep for future use
    output: String,
}

#[pymethods]
impl ExecuteLocal {
    #[new]
    #[pyo3(signature = (*, process_description=None, cmd=None, cwd=None, output=None, shell=None, **_kwargs))]
    fn new(
        process_description: Option<PyObject>,
        cmd: Option<Vec<String>>,
        cwd: Option<String>,
        output: Option<String>,
        shell: Option<bool>,
        _kwargs: Option<&pyo3::types::PyDict>,
    ) -> Self {
        // Log what we're executing
        if let Some(ref c) = cmd {
            log::debug!("Python Launch ExecuteLocal: cmd={:?}", c);
        } else {
            log::debug!("Python Launch ExecuteLocal: process_description provided");
        }

        if let Some(s) = shell {
            log::debug!("  shell={}", s);
        }

        Self {
            process_description,
            cmd,
            cwd,
            output: output.unwrap_or_else(|| "log".to_string()),
        }
    }

    fn __repr__(&self) -> String {
        if let Some(ref cmd) = self.cmd {
            format!("ExecuteLocal(cmd={:?})", cmd)
        } else {
            "ExecuteLocal(process_description=...)".to_string()
        }
    }
}

/// Mock TimerAction
///
/// Python equivalent:
/// ```python
/// from launch.actions import TimerAction
/// timer = TimerAction(period=10.0, actions=[action1, action2])
/// ```
///
/// Executes actions after a delay
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct TimerAction {
    #[allow(dead_code)] // Keep for future use
    period: f64,
    #[allow(dead_code)] // Keep for future use
    actions: Vec<PyObject>,
}

#[pymethods]
impl TimerAction {
    #[new]
    #[pyo3(signature = (*, period, actions, **_kwargs))]
    fn new(period: f64, actions: Vec<PyObject>, _kwargs: Option<&pyo3::types::PyDict>) -> Self {
        log::debug!("Python Launch TimerAction: period={}s", period);
        Self { period, actions }
    }

    fn __repr__(&self) -> String {
        format!(
            "TimerAction(period={}, {} actions)",
            self.period,
            self.actions.len()
        )
    }
}

/// Mock OpaqueCoroutine action
///
/// Python equivalent:
/// ```python
/// from launch.actions import OpaqueCoroutine
/// async def my_coroutine(context):
///     # async operations
///     pass
/// OpaqueCoroutine(coroutine=my_coroutine)
/// ```
///
/// Adds a Python coroutine function to the launch run loop.
/// For static analysis, we just capture that it was called.
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct OpaqueCoroutine {
    #[allow(dead_code)] // Keep for future use
    coroutine: PyObject,
    #[allow(dead_code)] // Keep for future use
    args: Vec<PyObject>,
    #[allow(dead_code)] // Keep for future use
    func_kwargs: Option<PyObject>,
}

#[pymethods]
impl OpaqueCoroutine {
    #[new]
    #[pyo3(signature = (*, coroutine, args=None, kwargs=None, **_extra_kwargs))]
    fn new(
        coroutine: PyObject,
        args: Option<Vec<PyObject>>,
        kwargs: Option<PyObject>,
        _extra_kwargs: Option<&pyo3::types::PyDict>,
    ) -> Self {
        log::debug!("Python Launch OpaqueCoroutine: coroutine provided");
        Self {
            coroutine,
            args: args.unwrap_or_default(),
            func_kwargs: kwargs,
        }
    }

    fn __repr__(&self) -> String {
        "OpaqueCoroutine(...)".to_string()
    }
}

/// Mock IncludeLaunchDescription action
///
/// Python equivalent:
/// ```python
/// from launch.actions import IncludeLaunchDescription
/// from launch.launch_description_sources import PythonLaunchDescriptionSource
///
/// include = IncludeLaunchDescription(
///     PythonLaunchDescriptionSource([path]),
///     launch_arguments={'arg': 'value'}.items()
/// )
/// ```
///
/// Includes another launch file (Python, XML, or YAML)
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct IncludeLaunchDescription {
    #[allow(dead_code)] // Stored for API compatibility, used during construction
    launch_description_source: PyObject,
    #[allow(dead_code)] // Stored for API compatibility, used during construction
    launch_arguments: Vec<(String, String)>,
}

/// Convert PyObject to string for launch arguments (handles strings, lists, substitutions)
fn pyobject_to_string_for_include_args(_py: Python, obj: &PyAny) -> PyResult<String> {
    use pyo3::types::PyList;

    // Try direct string extraction
    if let Ok(s) = obj.extract::<String>() {
        return Ok(s);
    }

    // Try list (concatenate all elements)
    if let Ok(list) = obj.downcast::<PyList>() {
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
        return Ok(result);
    }

    // Try calling __str__ method (for substitutions)
    if let Ok(str_result) = obj.call_method0("__str__") {
        if let Ok(s) = str_result.extract::<String>() {
            return Ok(s);
        }
    }

    // Fallback to to_string
    Ok(obj.to_string())
}

#[pymethods]
impl IncludeLaunchDescription {
    #[new]
    #[pyo3(signature = (launch_description_source, *, launch_arguments=None, **_kwargs))]
    fn new(
        py: Python,
        launch_description_source: PyObject,
        launch_arguments: Option<PyObject>,
        _kwargs: Option<&pyo3::types::PyDict>,
    ) -> PyResult<Self> {
        // Parse launch_arguments
        let mut args = Vec::new();
        if let Some(launch_args_obj) = launch_arguments {
            // Try to extract as dict
            if let Ok(dict) = launch_args_obj.downcast::<pyo3::types::PyDict>(py) {
                for (key, value) in dict.iter() {
                    let key_str = key.extract::<String>()?;
                    // Try to extract value as string, list, or substitution
                    let value_str = pyobject_to_string_for_include_args(py, value)?;
                    args.push((key_str, value_str));
                }
            }
            // Try to extract as list/iterator of tuples
            else if let Ok(iter) = launch_args_obj.as_ref(py).iter() {
                for item in iter {
                    let item = item?;
                    if let Ok(tuple) = item.downcast::<pyo3::types::PyTuple>() {
                        if tuple.len() == 2 {
                            let key = tuple.get_item(0)?.extract::<String>()?;
                            let value_obj = tuple.get_item(1)?;
                            // Try to extract value as string, list, or substitution
                            let value = pyobject_to_string_for_include_args(py, value_obj)?;
                            args.push((key, value));
                        }
                    }
                }
            }
        }

        // Extract file path from launch_description_source
        // Get the context from __main__ namespace if available (created by OpaqueFunction)
        let context = py
            .import("__main__")
            .ok()
            .and_then(|m| m.dict().get_item("context").ok().flatten());

        let file_path = if let Ok(path_str) = launch_description_source.extract::<String>(py) {
            // Direct string path
            path_str
        } else {
            // Try get_launch_file_path which will now use the context from __main__
            if let Ok(path_str) = launch_description_source.call_method0(py, "get_launch_file_path")
            {
                path_str
                    .extract::<String>(py)
                    .unwrap_or_else(|_| "unknown".to_string())
            } else if let Some(ctx) = context {
                // Fallback: try calling perform with context
                if let Ok(performed) = launch_description_source.call_method1(py, "perform", (ctx,))
                {
                    performed
                        .extract::<String>(py)
                        .unwrap_or_else(|_| "unknown".to_string())
                } else {
                    "unknown".to_string()
                }
            } else {
                "unknown".to_string()
            }
        };

        // Capture the include request with current ROS namespace
        {
            use crate::python::bridge::get_current_ros_namespace;
            let ros_namespace = get_current_ros_namespace();

            log::debug!("Capturing include with ROS namespace: '{}'", ros_namespace);

            let mut includes = CAPTURED_INCLUDES.lock();
            includes.push(crate::python::bridge::IncludeCapture {
                file_path: file_path.clone(),
                args: args.clone(),
                ros_namespace,
            });
        }

        log::info!(
            "Python Launch IncludeLaunchDescription: file_path='{}' with {} args",
            file_path,
            args.len()
        );

        Ok(Self {
            launch_description_source,
            launch_arguments: args,
        })
    }

    fn __repr__(&self) -> String {
        format!(
            "IncludeLaunchDescription({} args)",
            self.launch_arguments.len()
        )
    }
}

/// Mock SetLaunchConfiguration action
///
/// Python equivalent:
/// ```python
/// from launch.actions import SetLaunchConfiguration
/// set_config = SetLaunchConfiguration(name='config_name', value='value')
/// ```
///
/// Sets a launch configuration value
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct SetLaunchConfiguration {
    #[allow(dead_code)] // Stored for API compatibility
    name: PyObject,
    #[allow(dead_code)] // Stored for API compatibility
    value: PyObject,
}

#[pymethods]
impl SetLaunchConfiguration {
    #[new]
    #[pyo3(signature = (name, value, **_kwargs))]
    fn new(
        py: Python,
        name: PyObject,
        value: PyObject,
        _kwargs: Option<&pyo3::types::PyDict>,
    ) -> PyResult<Self> {
        // Convert name to string (may be a substitution)
        let name_str = if let Ok(s) = name.extract::<String>(py) {
            s
        } else if let Ok(str_result) = name.call_method0(py, "__str__") {
            str_result.extract::<String>(py)?
        } else {
            name.to_string()
        };

        log::debug!("Python Launch SetLaunchConfiguration: {}=<value>", name_str);
        Ok(Self { name, value })
    }

    fn __repr__(&self) -> String {
        "SetLaunchConfiguration(...)".to_string()
    }
}

/// Mock RegisterEventHandler action
///
/// Python equivalent:
/// ```python
/// from launch.actions import RegisterEventHandler
/// from launch.event_handlers import OnProcessStart
///
/// RegisterEventHandler(
///     OnProcessStart(
///         target_action=some_node,
///         on_start=[LogInfo(msg='Started!')]
///     )
/// )
/// ```
///
/// Event handlers allow actions to be triggered in response to events
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct RegisterEventHandler {
    #[allow(dead_code)] // Keep for API compatibility
    event_handler: PyObject,
}

#[pymethods]
impl RegisterEventHandler {
    #[new]
    #[pyo3(signature = (event_handler, **_kwargs))]
    fn new(event_handler: PyObject, _kwargs: Option<&pyo3::types::PyDict>) -> Self {
        log::debug!("Python Launch RegisterEventHandler created (limited support)");
        Self { event_handler }
    }

    fn __repr__(&self) -> String {
        "RegisterEventHandler(...)".to_string()
    }
}

/// Mock PushEnvironment action
///
/// Python equivalent:
/// ```python
/// from launch.actions import PushEnvironment
/// PushEnvironment()
/// ```
///
/// Pushes the current environment state onto a stack.
/// This allows temporary environment modifications that can be reverted with PopEnvironment.
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct PushEnvironment {}

#[pymethods]
impl PushEnvironment {
    #[new]
    #[pyo3(signature = (**_kwargs))]
    fn new(_kwargs: Option<&pyo3::types::PyDict>) -> Self {
        log::debug!("Python Launch PushEnvironment: pushing environment state");
        Self {}
    }

    fn __repr__(&self) -> String {
        "PushEnvironment()".to_string()
    }
}

/// Mock PopEnvironment action
///
/// Python equivalent:
/// ```python
/// from launch.actions import PopEnvironment
/// PopEnvironment()
/// ```
///
/// Pops the most recent environment state from the stack, restoring it.
/// Must be paired with a previous PushEnvironment.
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct PopEnvironment {}

#[pymethods]
impl PopEnvironment {
    #[new]
    #[pyo3(signature = (**_kwargs))]
    fn new(_kwargs: Option<&pyo3::types::PyDict>) -> Self {
        log::debug!("Python Launch PopEnvironment: popping environment state");
        Self {}
    }

    fn __repr__(&self) -> String {
        "PopEnvironment()".to_string()
    }
}

/// Mock ResetEnvironment action
///
/// Python equivalent:
/// ```python
/// from launch.actions import ResetEnvironment
/// ResetEnvironment()
/// ```
///
/// Resets the environment to its initial state (before any modifications).
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct ResetEnvironment {}

#[pymethods]
impl ResetEnvironment {
    #[new]
    #[pyo3(signature = (**_kwargs))]
    fn new(_kwargs: Option<&pyo3::types::PyDict>) -> Self {
        log::debug!("Python Launch ResetEnvironment: resetting environment to initial state");
        Self {}
    }

    fn __repr__(&self) -> String {
        "ResetEnvironment()".to_string()
    }
}

/// Mock AppendEnvironmentVariable action
///
/// Python equivalent:
/// ```python
/// from launch.actions import AppendEnvironmentVariable
/// AppendEnvironmentVariable('PATH', '/custom/path')
/// AppendEnvironmentVariable('LD_LIBRARY_PATH', '/custom/lib', prepend=True, separator=':')
/// AppendEnvironmentVariable(LaunchConfiguration('var_name'), '/path', separator=LaunchConfiguration('sep'))
/// ```
///
/// Appends (or prepends) a value to an existing environment variable.
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct AppendEnvironmentVariable {
    name: String,
    value: PyObject,
    #[allow(dead_code)]
    prepend: bool,
    #[allow(dead_code)]
    separator: String,
}

#[pymethods]
impl AppendEnvironmentVariable {
    #[new]
    #[pyo3(signature = (name, value, *, prepend=None, separator=None, **_kwargs))]
    fn new(
        py: Python,
        name: PyObject,
        value: PyObject,
        prepend: Option<PyObject>,
        separator: Option<PyObject>,
        _kwargs: Option<&pyo3::types::PyDict>,
    ) -> PyResult<Self> {
        // Convert name to string (handles strings, substitutions, and lists)
        let name_str = Self::pyobject_to_string(py, &name)?;

        // Handle prepend (bool or substitution resolving to bool)
        let prepend_val = if let Some(p) = prepend {
            if let Ok(b) = p.extract::<bool>(py) {
                b
            } else if let Ok(s) = Self::pyobject_to_string(py, &p) {
                // Parse string as bool (YAML rules: true, True, yes, 1, etc.)
                matches!(s.to_lowercase().as_str(), "true" | "yes" | "1")
            } else {
                false
            }
        } else {
            false
        };

        // Handle separator (default to ":")
        let sep_str = if let Some(s) = separator {
            Self::pyobject_to_string(py, &s)?
        } else {
            ":".to_string()
        };

        // Convert PyObject to string for logging
        let value_str = if let Ok(s) = value.extract::<String>(py) {
            s.clone()
        } else if let Ok(str_result) = value.call_method0(py, "__str__") {
            str_result
                .extract::<String>(py)
                .unwrap_or_else(|_| "<value>".to_string())
        } else {
            "<value>".to_string()
        };

        log::debug!(
            "Python Launch AppendEnvironmentVariable: {}{}{}{}",
            if prepend_val {
                "prepending "
            } else {
                "appending "
            },
            value_str,
            sep_str,
            name_str
        );

        Ok(Self {
            name: name_str,
            value,
            prepend: prepend_val,
            separator: sep_str,
        })
    }

    fn __repr__(&self, py: Python) -> String {
        let value_str = if let Ok(s) = self.value.extract::<String>(py) {
            s
        } else if let Ok(str_result) = self.value.call_method0(py, "__str__") {
            str_result
                .extract::<String>(py)
                .unwrap_or_else(|_| "<value>".to_string())
        } else {
            "<value>".to_string()
        };

        format!(
            "AppendEnvironmentVariable('{}', '{}', prepend={}, separator='{}')",
            self.name, value_str, self.prepend, self.separator
        )
    }
}

impl AppendEnvironmentVariable {
    /// Convert a PyObject to a string (handles strings, substitutions, and lists)
    /// Reuses the same pattern as SetEnvironmentVariable
    fn pyobject_to_string(py: Python, obj: &PyObject) -> PyResult<String> {
        use pyo3::types::PyList;
        let obj_ref = obj.as_ref(py);

        // Try direct string extraction first
        if let Ok(s) = obj_ref.extract::<String>() {
            return Ok(s);
        }

        // Handle lists (concatenate elements)
        if let Ok(list) = obj_ref.downcast::<PyList>() {
            let mut result = String::new();
            for item in list.iter() {
                let item_str = Self::pyobject_to_string(py, &item.into())?;
                result.push_str(&item_str);
            }
            return Ok(result);
        }

        // Try calling perform() method (for LaunchConfiguration)
        if obj_ref.hasattr("perform")? {
            if let Ok(context) = py.eval("type('Context', (), {})()", None, None) {
                if let Ok(result) = obj_ref.call_method1("perform", (context,)) {
                    if let Ok(s) = result.extract::<String>() {
                        return Ok(s);
                    }
                }
            }
        }

        // Try calling __str__ method
        if let Ok(str_result) = obj_ref.call_method0("__str__") {
            if let Ok(s) = str_result.extract::<String>() {
                return Ok(s);
            }
        }

        // Fallback to repr
        Ok(obj_ref.to_string())
    }
}

/// Mock PushLaunchConfigurations action
///
/// Python equivalent:
/// ```python
/// from launch.actions import PushLaunchConfigurations
/// PushLaunchConfigurations()
/// ```
///
/// Pushes the current launch configurations onto a stack.
/// This allows temporary configuration modifications that can be reverted with PopLaunchConfigurations.
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct PushLaunchConfigurations {}

#[pymethods]
impl PushLaunchConfigurations {
    #[new]
    #[pyo3(signature = (**_kwargs))]
    fn new(_kwargs: Option<&pyo3::types::PyDict>) -> Self {
        log::debug!("Python Launch PushLaunchConfigurations: pushing configuration state");
        Self {}
    }

    fn __repr__(&self) -> String {
        "PushLaunchConfigurations()".to_string()
    }
}

/// Mock PopLaunchConfigurations action
///
/// Python equivalent:
/// ```python
/// from launch.actions import PopLaunchConfigurations
/// PopLaunchConfigurations()
/// ```
///
/// Pops the most recent launch configurations from the stack, restoring them.
/// Must be paired with a previous PushLaunchConfigurations.
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct PopLaunchConfigurations {}

#[pymethods]
impl PopLaunchConfigurations {
    #[new]
    #[pyo3(signature = (**_kwargs))]
    fn new(_kwargs: Option<&pyo3::types::PyDict>) -> Self {
        log::debug!("Python Launch PopLaunchConfigurations: popping configuration state");
        Self {}
    }

    fn __repr__(&self) -> String {
        "PopLaunchConfigurations()".to_string()
    }
}

/// Mock ResetLaunchConfigurations action
///
/// Python equivalent:
/// ```python
/// from launch.actions import ResetLaunchConfigurations
/// ResetLaunchConfigurations()
/// ```
///
/// Resets all launch configurations to their initial state (clearing any modifications).
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct ResetLaunchConfigurations {}

#[pymethods]
impl ResetLaunchConfigurations {
    #[new]
    #[pyo3(signature = (**_kwargs))]
    fn new(_kwargs: Option<&pyo3::types::PyDict>) -> Self {
        log::debug!(
            "Python Launch ResetLaunchConfigurations: resetting configurations to initial state"
        );
        Self {}
    }

    fn __repr__(&self) -> String {
        "ResetLaunchConfigurations()".to_string()
    }
}

/// Mock UnsetLaunchConfiguration action
///
/// Python equivalent:
/// ```python
/// from launch.actions import UnsetLaunchConfiguration
/// UnsetLaunchConfiguration('variable_name')
/// ```
///
/// Removes a specific launch configuration variable.
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct UnsetLaunchConfiguration {
    name: String,
}

#[pymethods]
impl UnsetLaunchConfiguration {
    #[new]
    fn new(name: String) -> Self {
        log::debug!("Python Launch UnsetLaunchConfiguration: {}", name);
        Self { name }
    }

    fn __repr__(&self) -> String {
        format!("UnsetLaunchConfiguration('{}')", self.name)
    }
}

/// Mock Shutdown action
///
/// Python equivalent:
/// ```python
/// from launch.actions import Shutdown
/// Shutdown()
/// ```
///
/// Triggers a shutdown of the launch system when executed.
/// For static analysis, this is purely informational.
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct Shutdown {
    #[allow(dead_code)] // Keep for future use
    reason: Option<String>,
}

#[pymethods]
impl Shutdown {
    #[new]
    #[pyo3(signature = (*, reason=None, **_kwargs))]
    fn new(reason: Option<String>, _kwargs: Option<&pyo3::types::PyDict>) -> Self {
        if let Some(ref r) = reason {
            log::debug!("Python Launch Shutdown: reason={}", r);
        } else {
            log::debug!("Python Launch Shutdown");
        }
        Self { reason }
    }

    fn __repr__(&self) -> String {
        if let Some(ref r) = self.reason {
            format!("Shutdown(reason='{}')", r)
        } else {
            "Shutdown()".to_string()
        }
    }
}
