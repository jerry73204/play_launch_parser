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

            // CRITICAL: Aggressively isolate Python environment to prevent real ROS packages from loading
            let isolation_code = r#"
import sys
import types

# STEP 0: Disable bytecode caching to prevent stale imports
sys.dont_write_bytecode = True

# STEP 1: Install import hook to block only launch.* filesystem searches
# This allows other ROS packages (ament_index_python, etc.) to load normally
import sys
import importlib.util
import importlib.machinery

class LaunchModuleBlocker:
    """Blocks filesystem imports of launch.* modules while allowing sys.modules"""
    def find_spec(self, fullname, path, target=None):
        # Only intercept launch.* module imports
        if fullname.startswith(('launch', 'launch_ros', 'launch_xml')):
            # If it's already in sys.modules (our mock), let default machinery handle it
            if fullname in sys.modules:
                return None  # Continue with default import machinery
            # Block filesystem search by returning a failing spec
            # This prevents real ROS launch modules from being found on sys.path
            raise ImportError(f"Blocked filesystem import of {fullname} (use mocks only)")
        # Allow all other imports to proceed normally
        return None

# Install at beginning of sys.meta_path to intercept before filesystem finders
if not any(isinstance(f, LaunchModuleBlocker) for f in sys.meta_path):
    sys.meta_path.insert(0, LaunchModuleBlocker())

# STEP 2: Remove existing launch* submodules from sys.modules that aren't our mocks
# This clears any cached imports from real ROS packages
# Keep our registered top-level and submodule mocks
our_mocks = {
    'launch', 'launch.actions', 'launch.substitutions', 'launch.conditions',
    'launch.event_handlers', 'launch.launch_description_sources', 'launch.frontend',
    'launch.frontend.type_utils', 'launch.utilities', 'launch.utilities.type_utils',
    'launch.some_substitutions_type',
    'launch_ros', 'launch_ros.actions', 'launch_ros.descriptions',
    'launch_ros.substitutions', 'launch_ros.parameter_descriptions', 'launch_ros.utilities',
    'launch_xml', 'launch_xml.launch_description_sources',
}

to_delete = [k for k in list(sys.modules.keys())
             if k.startswith(('launch.', 'launch_ros.', 'launch_xml.'))
             and k not in our_mocks]

for key in to_delete:
    del sys.modules[key]

# STEP 4: Clear import caches to force fresh lookups
import importlib
importlib.invalidate_caches()

# STEP 5: Verify our mocks are properly registered and accessible
def _verify_mocks():
    """Verify that our mocks are accessible and have correct attributes"""
    try:
        from launch.actions import GroupAction
        # Test that actions attribute exists
        ga = GroupAction([])
        if hasattr(ga, 'actions'):
            return True, f"OK: GroupAction.actions exists, module={GroupAction.__module__}"
        else:
            return False, f"ERROR: GroupAction missing .actions, module={GroupAction.__module__}"
    except Exception as e:
        return False, f"ERROR: {e}"

_ok, _msg = _verify_mocks()
if not _ok:
    raise RuntimeError(f"Mock verification failed: {_msg}")
"#;

            // IMPORTANT: Run isolation in the GLOBAL context first so sys.modules is clean
            py.run(isolation_code, None, None)?;
            log::debug!("Installed aggressive Python environment isolation for launch* mocks");

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

            // Execute the Python file directly with exec() instead of runpy
            // This gives us complete control over the execution environment
            use std::fs;
            let code = fs::read_to_string(launch_file_path).map_err(|e| {
                crate::error::ParseError::PythonError(format!("Failed to read Python file: {}", e))
            })?;

            // CRITICAL: Create a FRESH namespace for each Python file execution
            // This prevents imports from one file polluting another file's namespace
            // We use a new empty dict but set __builtins__ to maintain access to built-in functions
            let globals = pyo3::types::PyDict::new(py);
            let builtins = py.import("builtins")?;
            globals.set_item("__builtins__", builtins)?;

            // Set __file__ and __name__
            globals.set_item("__file__", launch_file_path)?;
            globals.set_item("__name__", "__main__")?;

            // CRITICAL: Run isolation AGAIN right before executing the file
            // This ensures any modifications to sys.modules/sys.path by previous files are reset
            py.run(isolation_code, None, None)?;
            log::debug!("Re-applied isolation right before file execution");

            // Add debug logging to catch Python execution errors
            log::debug!("Executing Python code from: {}", launch_file_path);
            log::trace!("Python code length: {} bytes", code.len());

            // Execute the file
            if let Err(e) = py.run(&code, Some(globals), None) {
                log::error!("Python execution failed: {}", e);
                // Log the Python traceback if available
                if let Some(traceback) = e.traceback(py) {
                    if let Ok(tb_str) = traceback.format() {
                        log::error!("Python traceback:\n{}", tb_str);
                    }
                }
                return Err(e.into());
            }

            // Get and call generate_launch_description()
            let gen_fn = globals
                .get_item("generate_launch_description")?
                .ok_or_else(|| {
                    crate::error::ParseError::PythonError(
                        "No generate_launch_description() function found".to_string(),
                    )
                })?;

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

    log::debug!(
        "Visiting {} entities in launch description",
        entities_list.len()
    );

    // Visit each entity
    for entity in entities_list.iter() {
        visit_entity(py, entity)?;
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

    log::debug!("Visiting entity: {}", entity_type);

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
            // Get nested actions and visit them
            let sub_entities = entity.getattr(py, "actions")?;
            let sub_list: Vec<PyObject> = sub_entities.extract(py)?;
            log::debug!("GroupAction contains {} sub-actions", sub_list.len());
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
