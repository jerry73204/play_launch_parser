//! Python launch file executor
//!
//! Executes Python launch files with PyO3 mocks to capture node definitions.

use crate::{
    error::Result,
    python::bridge::{
        CAPTURED_CONTAINERS, CAPTURED_INCLUDES, CAPTURED_LOAD_NODES, CAPTURED_NODES,
        LAUNCH_CONFIGURATIONS, ROS_NAMESPACE_STACK,
    },
};
use pyo3::prelude::*;
use std::collections::HashMap;

/// Executes Python launch files with mock API
pub struct PythonLaunchExecutor {
    global_params: HashMap<String, String>,
}

impl PythonLaunchExecutor {
    /// Create new executor with global parameters
    pub fn new(global_params: HashMap<String, String>) -> Self {
        Self { global_params }
    }

    /// Execute a Python launch file and capture entities
    pub fn execute(&self, launch_file_path: &str) -> Result<()> {
        Python::with_gil(|py| {
            log::info!("Executing Python launch file: {}", launch_file_path);

            // Register PyO3 mock modules in sys.modules
            crate::python::api::register_modules(py)?;

            // Clear entity capture storage before execution
            // Note: Don't clear ROS_NAMESPACE_STACK as it may have been set by XML context
            CAPTURED_NODES.lock().clear();
            CAPTURED_CONTAINERS.lock().clear();
            CAPTURED_LOAD_NODES.lock().clear();
            CAPTURED_INCLUDES.lock().clear();

            // Initialize ROS_NAMESPACE_STACK if it's empty
            if ROS_NAMESPACE_STACK.lock().is_empty() {
                ROS_NAMESPACE_STACK.lock().push("".to_string());
            }

            // Populate LAUNCH_CONFIGURATIONS with global parameters
            {
                let mut configs = LAUNCH_CONFIGURATIONS.lock();
                configs.clear();
                for (k, v) in &self.global_params {
                    configs.insert(k.clone(), v.clone());
                }
            }

            // Execute the Python file to load its globals
            let runpy = py.import("runpy")?;
            let globals = runpy.call_method1("run_path", (launch_file_path,))?;

            // Get and call generate_launch_description()
            let gen_fn = globals.get_item("generate_launch_description")?;
            let launch_desc: PyObject = gen_fn.call0()?.into();

            // Visit all entities in the launch description
            visit_launch_description(py, &launch_desc)?;

            log::debug!("Python launch file execution complete");
            Ok(())
        })
    }
}

/// Visit all entities in a launch description
fn visit_launch_description(py: Python, launch_desc: &PyObject) -> PyResult<()> {
    // Get entities from launch description (it's a property, not a method)
    let entities = launch_desc.getattr(py, "actions")?;
    let entities_list: Vec<PyObject> = entities.extract(py)?;

    log::debug!("Visiting {} entities", entities_list.len());

    // Visit each entity
    for entity in entities_list {
        visit_entity(py, &entity)?;
    }

    Ok(())
}

/// Visit a single entity
fn visit_entity(py: Python, entity: &PyObject) -> PyResult<()> {
    // Get entity type
    let entity_class = entity.getattr(py, "__class__")?;
    let entity_type = entity_class
        .getattr(py, "__name__")?
        .extract::<String>(py)?;

    log::trace!("Visiting entity type: {}", entity_type);

    match entity_type.as_str() {
        "Node" | "LifecycleNode" | "ComposableNodeContainer" | "LoadComposableNodes" => {
            // Already captured during construction
            log::debug!("Entity {} already captured", entity_type);
        }

        "OpaqueFunction" => {
            log::debug!("Executing OpaqueFunction");
            // Call execute() to run the function and get returned entities
            let result = entity.call_method0(py, "execute")?;

            // Result should be a list of entities or None
            if !result.is_none(py) {
                if let Ok(entities) = result.extract::<Vec<PyObject>>(py) {
                    for entity in entities {
                        visit_entity(py, &entity)?;
                    }
                }
            }
        }

        "IncludeLaunchDescription" => {
            log::debug!("Processing IncludeLaunchDescription");
            // Includes are captured during construction
        }

        "GroupAction" => {
            log::debug!("Processing GroupAction");
            // Get sub-entities
            let sub_entities = entity.getattr(py, "actions")?;
            let sub_list: Vec<PyObject> = sub_entities.extract(py)?;
            for sub in sub_list {
                visit_entity(py, &sub)?;
            }
        }

        "DeclareLaunchArgument"
        | "SetLaunchConfiguration"
        | "SetEnvironmentVariable"
        | "PushRosNamespace"
        | "PopRosNamespace" => {
            log::debug!("Skipping action: {}", entity_type);
            // These are handled during static parsing
        }

        _ => {
            log::trace!("Skipping unknown entity type: {}", entity_type);
        }
    }

    Ok(())
}
