//! Mock Python API for ROS 2 launch system

pub mod actions;
pub mod launch;
pub mod launch_ros;
pub mod substitutions;

use pyo3::prelude::*;

/// Register all mock Python modules in sys.modules
///
/// This creates mock versions of:
/// - `launch`
/// - `launch.actions`
/// - `launch.substitutions`
/// - `launch_ros`
/// - `launch_ros.actions`
/// - `launch_ros.descriptions`
pub fn register_modules(py: Python) -> PyResult<()> {
    // Create launch module
    let launch_mod = PyModule::new(py, "launch")?;
    launch_mod.add_class::<launch::LaunchDescription>()?;

    // Create launch.actions submodule
    let launch_actions = PyModule::new(py, "launch.actions")?;
    launch_actions.add_class::<actions::DeclareLaunchArgument>()?;

    // Create launch.substitutions submodule
    let launch_subs = PyModule::new(py, "launch.substitutions")?;
    launch_subs.add_class::<substitutions::LaunchConfiguration>()?;
    launch_subs.add_class::<substitutions::TextSubstitution>()?;

    // Add submodules to parent module as attributes
    launch_mod.add_submodule(launch_actions)?;
    launch_mod.add_submodule(launch_subs)?;

    // Create launch_ros module
    let launch_ros_mod = PyModule::new(py, "launch_ros")?;

    // Create launch_ros.actions submodule
    let launch_ros_actions = PyModule::new(py, "launch_ros.actions")?;
    launch_ros_actions.add_class::<launch_ros::Node>()?;

    // Create launch_ros.descriptions submodule
    let launch_ros_desc = PyModule::new(py, "launch_ros.descriptions")?;

    // Add submodules to parent module as attributes
    launch_ros_mod.add_submodule(launch_ros_actions)?;
    launch_ros_mod.add_submodule(launch_ros_desc)?;

    // Register in sys.modules
    let sys = py.import("sys")?;
    let modules = sys.getattr("modules")?;

    modules.set_item("launch", launch_mod)?;
    modules.set_item("launch.actions", launch_actions)?;
    modules.set_item("launch.substitutions", launch_subs)?;
    modules.set_item("launch_ros", launch_ros_mod)?;
    modules.set_item("launch_ros.actions", launch_ros_actions)?;
    modules.set_item("launch_ros.descriptions", launch_ros_desc)?;

    Ok(())
}
