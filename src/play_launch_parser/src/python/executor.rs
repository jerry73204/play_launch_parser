//! Python launch file execution engine

use crate::error::{ParseError, Result};
use crate::record::NodeRecord;
use pyo3::prelude::*;
use std::collections::HashMap;
use std::path::Path;

use super::api;
use super::bridge::CAPTURED_NODES;

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

    /// Execute a Python launch file and return captured nodes
    ///
    /// # Arguments
    ///
    /// * `path` - Path to the .launch.py file
    /// * `args` - Launch arguments (key-value pairs)
    ///
    /// # Returns
    ///
    /// Vector of NodeRecord structs captured during execution
    pub fn execute_launch_file(
        &self,
        path: &Path,
        args: &HashMap<String, String>,
    ) -> Result<Vec<NodeRecord>> {
        let result = Python::with_gil(|py| {
            // Clear previous captures
            CAPTURED_NODES.lock().unwrap().clear();

            // Register our mock modules
            api::register_modules(py)?;

            // Read the Python file
            let code = std::fs::read_to_string(path)?;

            // Use __main__ module's dict as both globals and locals for proper Python environment
            // This ensures imports work correctly and functions are defined in the right scope
            let main_module = py.import("__main__")?;
            let namespace = main_module.dict().copy()?;

            // Set launch arguments in namespace
            for (key, value) in args {
                namespace.set_item(key, value)?;
            }

            // Execute the Python code with same dict for globals and locals
            py.run(&code, Some(&namespace), Some(&namespace))?;

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
            let _launch_description = generate_fn.call0()?;

            // Nodes are already captured via our mock API
            // Convert to NodeRecord and return
            let captures = CAPTURED_NODES.lock().unwrap().clone();
            let nodes = captures
                .into_iter()
                .map(|capture| capture.to_record())
                .collect::<Result<Vec<_>>>()
                .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!("{}", e)))?;

            Ok(nodes)
        });

        result.map_err(|e: PyErr| ParseError::PythonError(e.to_string()))
    }
}

impl Default for PythonLaunchExecutor {
    fn default() -> Self {
        Self::new().expect("Failed to initialize Python executor")
    }
}
