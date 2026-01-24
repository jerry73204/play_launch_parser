//! Python launch file execution engine

use crate::error::{ParseError, Result};
use crate::record::{ComposableNodeContainerRecord, LoadNodeRecord, NodeRecord};
use pyo3::prelude::*;
use pyo3::types::PyAny;
use std::collections::HashMap;
use std::path::Path;

use super::api;
use super::bridge::{
    IncludeCapture, CAPTURED_CONTAINERS, CAPTURED_INCLUDES, CAPTURED_LOAD_NODES, CAPTURED_NODES,
    LAUNCH_CONFIGURATIONS,
};

/// Python launch file executor
///
/// Manages the Python interpreter lifecycle and executes Python launch files,
/// capturing node definitions via the mock API.
pub struct PythonLaunchExecutor;

impl PythonLaunchExecutor {
    /// Create a new Python executor
    pub fn new() -> PyResult<Self> {
        Ok(Self)
    }

    /// Process a LaunchDescription object, recursively handling actions like OpaqueFunction
    fn process_launch_description(py: Python, launch_desc: &PyAny) -> PyResult<()> {
        // Get the actions list from LaunchDescription
        let actions = launch_desc.getattr("actions")?;

        // Iterate through actions
        if let Ok(actions_list) = actions.extract::<Vec<PyObject>>() {
            for action in actions_list {
                Self::process_action(py, action.as_ref(py))?;
            }
        }

        Ok(())
    }

    /// Process a single action, handling OpaqueFunction, GroupAction, etc.
    fn process_action(py: Python, action: &PyAny) -> PyResult<()> {
        use crate::python::api::actions::OpaqueFunction;

        // Check if this is an OpaqueFunction - extract and execute it
        if let Ok(opaque_func) = action.extract::<OpaqueFunction>() {
            log::trace!("Executing OpaqueFunction");
            match opaque_func.execute(py) {
                Ok(result) => {
                    log::trace!("OpaqueFunction succeeded");
                    // Process the result (list of actions returned from function)
                    Self::process_action_result(py, result.as_ref(py))?;
                }
                Err(e) => {
                    log::error!("OpaqueFunction execution failed: {}", e);
                    // Don't fail the entire parse, but log the error
                    // This matches Python's behavior where OpaqueFunction errors are often caught
                }
            }
            return Ok(());
        }

        // Check if this is a GroupAction (has nested actions)
        if let Ok(nested_actions) = action.getattr("actions") {
            if let Ok(actions_list) = nested_actions.extract::<Vec<PyObject>>() {
                for nested_action in actions_list {
                    Self::process_action(py, nested_action.as_ref(py))?;
                }
            }
        }

        // Note: Node and ComposableNodeContainer are captured on construction
        // So we don't need to explicitly process them here
        Ok(())
    }

    /// Process the result of an action (handles lists of actions or single actions)
    fn process_action_result(py: Python, result: &PyAny) -> PyResult<()> {
        // Check if result is None
        if result.is_none() {
            return Ok(());
        }

        // Try to extract as a list of actions
        if let Ok(actions) = result.extract::<Vec<PyObject>>() {
            for action in actions {
                Self::process_action(py, action.as_ref(py))?;
            }
        }
        // If not a list, it might be a single action - process it
        else {
            Self::process_action(py, result)?;
        }

        Ok(())
    }

    /// Execute a Python launch file and return captured entities
    ///
    /// # Arguments
    ///
    /// * `path` - Path to the .launch.py file
    /// * `args` - Launch arguments (key-value pairs)
    ///
    /// # Returns
    ///
    /// Tuple of (nodes, containers, load_nodes, includes) captured during execution
    #[allow(clippy::type_complexity)]
    pub fn execute_launch_file(
        &self,
        path: &Path,
        args: &HashMap<String, String>,
    ) -> Result<(
        Vec<NodeRecord>,
        Vec<ComposableNodeContainerRecord>,
        Vec<LoadNodeRecord>,
        Vec<IncludeCapture>,
    )> {
        let result = Python::with_gil(|py| {
            // Clear previous captures
            CAPTURED_NODES.lock().clear();
            CAPTURED_CONTAINERS.lock().clear();
            CAPTURED_LOAD_NODES.lock().clear();
            CAPTURED_INCLUDES.lock().clear();

            // Store launch configurations for condition evaluation
            {
                let mut configs = LAUNCH_CONFIGURATIONS.lock();
                configs.clear();
                for (key, value) in args {
                    log::trace!(
                        "Setting launch configuration from args: '{}' = '{}'",
                        key,
                        value
                    );
                    configs.insert(key.clone(), value.clone());
                }
            }

            // Register our mock modules
            api::register_modules(py)?;

            // Read the Python file
            let code = std::fs::read_to_string(path)?;

            // Use __main__ module's dict as both globals and locals for proper Python environment
            // This ensures imports work correctly and functions are defined in the right scope
            let main_module = py.import("__main__")?;
            // Clear the main module's namespace except for builtins to avoid test contamination
            let main_dict = main_module.dict();
            let builtins = main_dict.get_item("__builtins__")?.unwrap();
            let name = main_dict.get_item("__name__")?.unwrap();
            let doc = main_dict.get_item("__doc__")?;
            main_dict.clear();
            main_dict.set_item("__builtins__", builtins)?;
            main_dict.set_item("__name__", name)?;
            if let Some(doc_val) = doc {
                main_dict.set_item("__doc__", doc_val)?;
            }
            let namespace = main_dict.copy()?;

            // Set launch arguments in namespace
            for (key, value) in args {
                namespace.set_item(key, value)?;
            }

            // Execute the Python code with same dict for globals and locals
            py.run(&code, Some(namespace), Some(namespace))?;

            // Look for generate_launch_description function
            let generate_fn = match namespace.get_item("generate_launch_description")? {
                Some(func) => func,
                None => {
                    return Err(PyErr::new::<pyo3::exceptions::PyAttributeError, _>(
                        "No generate_launch_description function found",
                    ))
                }
            };

            // Call the function
            let launch_description = generate_fn.call0()?;

            // Process the LaunchDescription to handle OpaqueFunction actions
            Self::process_launch_description(py, launch_description)?;

            // Convert captured entities to records
            let node_captures = CAPTURED_NODES.lock().clone();
            let nodes = node_captures
                .into_iter()
                .map(|capture| capture.to_record())
                .collect::<Result<Vec<_>>>()
                .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!("{}", e)))?;

            let container_captures = CAPTURED_CONTAINERS.lock().clone();
            let containers = container_captures
                .into_iter()
                .map(|capture| capture.to_record())
                .collect::<Result<Vec<_>>>()
                .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!("{}", e)))?;

            let load_node_captures = CAPTURED_LOAD_NODES.lock().clone();
            let load_nodes = load_node_captures
                .into_iter()
                .map(|capture| capture.to_record())
                .collect::<Result<Vec<_>>>()
                .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!("{}", e)))?;

            let includes = CAPTURED_INCLUDES.lock().clone();

            Ok((nodes, containers, load_nodes, includes))
        });

        result.map_err(|e: PyErr| ParseError::PythonError(e.to_string()))
    }
}

impl Default for PythonLaunchExecutor {
    fn default() -> Self {
        Self::new().expect("Failed to initialize Python executor")
    }
}
