//! Mock Python API for ROS 2 launch system

pub mod actions;
pub mod conditions;
pub mod launch;
pub mod launch_description_sources;
pub mod launch_ros;
pub mod substitutions;

use pyo3::prelude::*;

/// Register all mock Python modules in sys.modules
///
/// This creates mock versions of:
/// - `launch`
/// - `launch.actions`
/// - `launch.substitutions`
/// - `launch.conditions`
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
    launch_actions.add_class::<actions::LogInfo>()?;
    launch_actions.add_class::<actions::SetEnvironmentVariable>()?;
    launch_actions.add_class::<actions::UnsetEnvironmentVariable>()?;
    launch_actions.add_class::<actions::GroupAction>()?;
    launch_actions.add_class::<actions::ExecuteProcess>()?;
    launch_actions.add_class::<actions::TimerAction>()?;
    launch_actions.add_class::<actions::OpaqueFunction>()?;
    launch_actions.add_class::<actions::IncludeLaunchDescription>()?;
    launch_actions.add_class::<actions::SetLaunchConfiguration>()?;

    // Create launch.substitutions submodule
    let launch_subs = PyModule::new(py, "launch.substitutions")?;
    launch_subs.add_class::<substitutions::LaunchConfiguration>()?;
    launch_subs.add_class::<substitutions::TextSubstitution>()?;
    launch_subs.add_class::<substitutions::PathJoinSubstitution>()?;
    launch_subs.add_class::<substitutions::FindPackageShare>()?;
    launch_subs.add_class::<substitutions::EnvironmentVariable>()?;
    launch_subs.add_class::<substitutions::ThisLaunchFileDir>()?;
    launch_subs.add_class::<substitutions::PythonExpression>()?;

    // Create launch.conditions submodule
    let launch_conditions = PyModule::new(py, "launch.conditions")?;
    launch_conditions.add_class::<conditions::IfCondition>()?;
    launch_conditions.add_class::<conditions::UnlessCondition>()?;
    launch_conditions.add_class::<conditions::LaunchConfigurationEquals>()?;
    launch_conditions.add_class::<conditions::LaunchConfigurationNotEquals>()?;

    // Create launch.launch_description_sources submodule
    let launch_sources = PyModule::new(py, "launch.launch_description_sources")?;
    launch_sources.add_class::<launch_description_sources::PythonLaunchDescriptionSource>()?;
    launch_sources.add_class::<launch_description_sources::XMLLaunchDescriptionSource>()?;
    launch_sources.add_class::<launch_description_sources::YAMLLaunchDescriptionSource>()?;

    // Add submodules to parent module as attributes
    launch_mod.add_submodule(launch_actions)?;
    launch_mod.add_submodule(launch_subs)?;
    launch_mod.add_submodule(launch_conditions)?;
    launch_mod.add_submodule(launch_sources)?;

    // Create launch_ros module
    let launch_ros_mod = PyModule::new(py, "launch_ros")?;

    // Create launch_ros.actions submodule
    let launch_ros_actions = PyModule::new(py, "launch_ros.actions")?;
    launch_ros_actions.add_class::<launch_ros::Node>()?;
    launch_ros_actions.add_class::<launch_ros::ComposableNodeContainer>()?;
    launch_ros_actions.add_class::<launch_ros::SetParameter>()?;

    // Create launch_ros.descriptions submodule
    let launch_ros_desc = PyModule::new(py, "launch_ros.descriptions")?;
    launch_ros_desc.add_class::<launch_ros::ComposableNode>()?;

    // Create launch_ros.substitutions submodule
    let launch_ros_subs = PyModule::new(py, "launch_ros.substitutions")?;
    launch_ros_subs.add_class::<substitutions::FindPackageShare>()?;

    // Add submodules to parent module as attributes
    launch_ros_mod.add_submodule(launch_ros_actions)?;
    launch_ros_mod.add_submodule(launch_ros_desc)?;
    launch_ros_mod.add_submodule(launch_ros_subs)?;

    // Register in sys.modules
    let sys = py.import("sys")?;
    let modules = sys.getattr("modules")?;

    modules.set_item("launch", launch_mod)?;
    modules.set_item("launch.actions", launch_actions)?;
    modules.set_item("launch.substitutions", launch_subs)?;
    modules.set_item("launch.conditions", launch_conditions)?;
    modules.set_item("launch.launch_description_sources", launch_sources)?;
    modules.set_item("launch_ros", launch_ros_mod)?;
    modules.set_item("launch_ros.actions", launch_ros_actions)?;
    modules.set_item("launch_ros.descriptions", launch_ros_desc)?;
    modules.set_item("launch_ros.substitutions", launch_ros_subs)?;

    Ok(())
}
