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
            let mut configs = LAUNCH_CONFIGURATIONS.lock().unwrap();
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
                        if let Ok(performed) = item.call_method1("perform", (context,)) {
                            if let Ok(s) = performed.extract::<String>() {
                                result.push_str(&s);
                                continue;
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
#[pyclass]
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
        if let Some(ref func) = self.function {
            // Create a mock LaunchContext that provides access to launch configurations
            // This allows OpaqueFunction code to call LaunchConfiguration().perform(context)
            use crate::python::bridge::LAUNCH_CONFIGURATIONS;
            let configs = LAUNCH_CONFIGURATIONS.lock().unwrap().clone();

            // Create a context dict with launch_configurations
            let context_code = r#"
class MockLaunchContext:
    def __init__(self, launch_configurations):
        self.launch_configurations = launch_configurations

    def perform_substitution(self, sub):
        # Support LaunchConfiguration.perform(context)
        if hasattr(sub, 'variable_name'):
            return self.launch_configurations.get(sub.variable_name, '')
        return str(sub)

context = MockLaunchContext(launch_configurations)
"#;

            let main_module = py.import("__main__")?;
            let namespace = main_module.dict();

            // Add launch configurations to namespace
            let configs_dict = pyo3::types::PyDict::new(py);
            for (key, value) in configs {
                configs_dict.set_item(key, value)?;
            }
            namespace.set_item("launch_configurations", configs_dict)?;

            // Add Python builtins that OpaqueFunction might need
            // Import Path for file operations
            py.run("from pathlib import Path", Some(namespace), Some(namespace))?;
            // Import yaml for YAML loading
            py.run("import yaml", Some(namespace), Some(namespace)).ok(); // Ignore if yaml not available

            // Create the context using the code above
            py.run(context_code, Some(namespace), Some(namespace))?;
            let context = namespace.get_item("context")?.unwrap();

            // Call the function with the context
            func.call1(py, (context,))
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
/// ```
///
/// Logs an information message when the action is executed
#[pyclass]
#[derive(Clone)]
pub struct LogInfo {
    msg: String,
}

#[pymethods]
impl LogInfo {
    #[new]
    #[pyo3(signature = (*, msg, **_kwargs))]
    fn new(msg: String, _kwargs: Option<&pyo3::types::PyDict>) -> Self {
        // Log the message immediately
        log::info!("Python Launch LogInfo: {}", msg);
        Self { msg }
    }

    fn __repr__(&self) -> String {
        format!("LogInfo(msg='{}')", self.msg)
    }
}

/// Mock SetEnvironmentVariable action
///
/// Python equivalent:
/// ```python
/// from launch.actions import SetEnvironmentVariable
/// set_env = SetEnvironmentVariable('VAR_NAME', 'value')
/// ```
///
/// Sets an environment variable
#[pyclass]
#[derive(Clone)]
pub struct SetEnvironmentVariable {
    name: String,
    value: String,
}

#[pymethods]
impl SetEnvironmentVariable {
    #[new]
    fn new(name: String, value: String) -> Self {
        // TODO: Actually set the environment variable in the launch context
        log::debug!("Python Launch SetEnvironmentVariable: {}={}", name, value);
        Self { name, value }
    }

    fn __repr__(&self) -> String {
        format!("SetEnvironmentVariable('{}', '{}')", self.name, self.value)
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
#[pyclass]
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
#[pyclass]
#[derive(Clone)]
pub struct GroupAction {
    actions: Vec<PyObject>,
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
/// ```
///
/// Executes a non-ROS process
#[pyclass]
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
        cmd: Vec<String>,
        cwd: Option<String>,
        name: Option<String>,
        output: Option<String>,
        _kwargs: Option<&pyo3::types::PyDict>,
    ) -> Self {
        // TODO: Capture this as an ExecutableRecord
        log::debug!("Python Launch ExecuteProcess: {:?}", cmd);
        Self {
            cmd,
            cwd,
            name,
            output: output.unwrap_or_else(|| "screen".to_string()),
        }
    }

    fn __repr__(&self) -> String {
        format!("ExecuteProcess(cmd={:?})", self.cmd)
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
#[pyclass]
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
#[pyclass]
#[derive(Clone)]
pub struct IncludeLaunchDescription {
    #[allow(dead_code)] // Stored for API compatibility, used during construction
    launch_description_source: PyObject,
    #[allow(dead_code)] // Stored for API compatibility, used during construction
    launch_arguments: Vec<(String, String)>,
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
                    // Try to extract value as string, or call __str__ for substitutions
                    let value_str = if let Ok(s) = value.extract::<String>() {
                        s
                    } else if let Ok(str_result) = value.call_method0("__str__") {
                        str_result.extract::<String>()?
                    } else {
                        value.to_string()
                    };
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
                            // Try to extract value as string, or call __str__ for substitutions
                            let value = if let Ok(s) = value_obj.extract::<String>() {
                                s
                            } else if let Ok(str_result) = value_obj.call_method0("__str__") {
                                str_result.extract::<String>()?
                            } else {
                                value_obj.to_string()
                            };
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

        // Capture the include request
        {
            let mut includes = CAPTURED_INCLUDES.lock().unwrap();
            includes.push(crate::python::bridge::IncludeCapture {
                file_path: file_path.clone(),
                args: args.clone(),
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
#[pyclass]
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
#[pyclass]
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
