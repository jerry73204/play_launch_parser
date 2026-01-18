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

/// Mock OpaqueFunction action
///
/// Python equivalent:
/// ```python
/// from launch.actions import OpaqueFunction
/// opaque = OpaqueFunction(function=my_function)
/// ```
///
/// Executes a Python function (limited support)
#[pyclass]
#[derive(Clone)]
pub struct OpaqueFunction {
    #[allow(dead_code)] // Keep for future use
    function: PyObject,
}

#[pymethods]
impl OpaqueFunction {
    #[new]
    #[pyo3(signature = (*, function, **_kwargs))]
    fn new(function: PyObject, _kwargs: Option<&pyo3::types::PyDict>) -> Self {
        log::debug!("Python Launch OpaqueFunction created (limited support)");
        Self { function }
    }

    fn __repr__(&self) -> String {
        "OpaqueFunction(...)".to_string()
    }
}
