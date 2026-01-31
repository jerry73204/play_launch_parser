//! Shared utilities for Python API type conversions
//!
//! This module provides common helper functions for converting between
//! Python types and Rust types, particularly for handling ROS 2's
//! SomeSubstitutionsType pattern.

use pyo3::{prelude::*, types::PyDict};

/// Create a LaunchContext-like object with access to LAUNCH_CONFIGURATIONS
/// This allows substitutions to resolve LaunchConfiguration values during perform()
pub fn create_launch_context(py: Python) -> PyResult<PyObject> {
    // Import the launch configuration storage
    use crate::python::bridge::LAUNCH_CONFIGURATIONS;

    // Get the launch configurations
    let configs = LAUNCH_CONFIGURATIONS.lock();

    // Create a simple context object that has a launch_configurations dict
    let context_class = py.eval(
        "type('Context', (), {
            'launch_configurations': None,
            '__init__': lambda self, configs: setattr(self, 'launch_configurations', configs)
        })",
        None,
        None,
    )?;

    // Create a Python dict from our LAUNCH_CONFIGURATIONS
    let py_configs = PyDict::new(py);
    for (key, value) in configs.iter() {
        py_configs.set_item(key, value)?;
    }

    // Instantiate the context with our configs
    let context = context_class.call1((py_configs,))?;

    Ok(context.into())
}

/// Convert a PyObject to String, handling ROS 2's SomeSubstitutionsType pattern.
///
/// This function accepts three forms that match ROS 2's `SomeSubstitutionsType`:
/// 1. **Plain string**: `"literal_value"`
/// 2. **Substitution**: `LaunchConfiguration('var')` -> calls `__str__()` or `perform()`
/// 3. **List**: `[LaunchConfiguration('prefix'), '/suffix']` -> concatenates elements
///
/// This mirrors ROS 2's type definition:
/// ```python
/// SomeSubstitutionsType = Union[
///     Text,                              # Plain string
///     Substitution,                      # LaunchConfiguration, FindPackageShare, etc.
///     Iterable[Union[Text, Substitution]], # List of strings and/or substitutions
/// ]
/// ```
///
/// # Conditional Substitutions
///
/// Conditional substitutions (EqualsSubstitution, IfElseSubstitution, NotEqualsSubstitution, etc.)
/// need to evaluate during parsing to determine which path to take. These are handled specially:
/// - They call `perform()` with a real LaunchContext containing LAUNCH_CONFIGURATIONS values
/// - This allows them to resolve LaunchConfiguration values and evaluate to "true"/"false"
///
/// # LaunchConfiguration Preservation
///
/// LaunchConfiguration substitutions are preserved as strings like "$(var node_name)" in the output.
/// This allows play_launch to re-resolve them at replay time with different values if needed.
///
/// # Examples
///
/// ```ignore
/// // Plain string
/// let result = pyobject_to_string(py, &py_string)?;
///
/// // LaunchConfiguration substitution
/// let result = pyobject_to_string(py, &launch_config)?;
/// // Result: "$(var variable_name)"
///
/// // Conditional substitution
/// let result = pyobject_to_string(py, &equals_sub)?;
/// // Result: "true" or "false" (evaluated with real context)
///
/// // List with mixed types
/// let result = pyobject_to_string(py, &py_list)?;
/// // Result: concatenated string from all elements
/// ```
///
/// # Conversion Strategy
///
/// The function tries multiple approaches in order:
/// 1. Direct string extraction (`extract::<String>()`)
/// 2. List handling (recursively convert and concatenate elements)
/// 3. Conditional substitutions: Call `perform()` with real context
/// 4. Other substitutions: Call `__str__()` to preserve substitution format
/// 5. Fallback to `to_string()` (Python repr)
///
/// # Errors
///
/// Returns `PyResult<String>` which will contain a PyErr if:
/// - The object cannot be converted to a string through any method
/// - Recursive conversion of list elements fails
///
/// # Performance
///
/// This function is called during launch file parsing (one-time cost),
/// not during node runtime, so performance impact is negligible.
pub fn pyobject_to_string(py: Python, obj: &PyObject) -> PyResult<String> {
    use pyo3::types::PyList;
    let obj_ref = obj.as_ref(py);

    // Try direct string extraction first (most common case)
    if let Ok(s) = obj_ref.extract::<String>() {
        return Ok(s);
    }

    // Handle lists (concatenate elements recursively)
    if let Ok(list) = obj_ref.downcast::<PyList>() {
        let mut result = String::new();
        for item in list.iter() {
            let item_str = pyobject_to_string(py, &item.into())?;
            result.push_str(&item_str);
        }
        return Ok(result);
    }

    // Check if this is a substitution that needs to be evaluated during parsing
    // These substitutions need to call perform() to resolve their content/logic
    if is_evaluating_substitution(obj_ref)? {
        // Create a real launch context with access to LAUNCH_CONFIGURATIONS
        let context = create_launch_context(py)?;

        // Call perform() to evaluate the condition
        if let Ok(result) = obj_ref.call_method1("perform", (context,)) {
            if let Ok(s) = result.extract::<String>() {
                return Ok(s);
            }
        }
    }

    // For other substitution objects (LaunchConfiguration, etc.), use __str__()
    // This preserves substitutions like "$(var node_name)" in the record.json output
    // The substitutions will be resolved later during actual launch execution
    if let Ok(str_result) = obj_ref.call_method0("__str__") {
        if let Ok(s) = str_result.extract::<String>() {
            return Ok(s);
        }
    }

    // Fallback to Python repr
    Ok(obj_ref.to_string())
}

/// Check if a PyObject is a substitution that needs evaluation during parsing.
///
/// Substitutions that need evaluation include:
/// - **Conditional substitutions**: EqualsSubstitution, NotEqualsSubstitution, IfElseSubstitution,
///   AndSubstitution, OrSubstitution, NotSubstitution (evaluate to "true"/"false")
/// - **Content substitutions**: FileContent (reads file and resolves nested substitutions),
///   PathJoinSubstitution (joins paths and resolves nested substitutions)
/// - **Expression substitutions**: PythonExpression (evaluates Python code)
///
/// These need to call perform() with real context during parsing to:
/// - Evaluate conditional logic (for conditionals)
/// - Resolve nested substitutions (for FileContent, PathJoinSubstitution)
/// - Read external content (for FileContent)
/// - Execute Python expressions (for PythonExpression)
///
/// LaunchConfiguration is NOT in this list - it should be preserved as "$(var name)"
/// for replay-time resolution.
fn is_evaluating_substitution(obj: &PyAny) -> PyResult<bool> {
    let type_name = obj.get_type().name()?;
    Ok(matches!(
        type_name,
        // Conditional substitutions
        "EqualsSubstitution"
            | "NotEqualsSubstitution"
            | "IfElseSubstitution"
            | "AndSubstitution"
            | "OrSubstitution"
            | "NotSubstitution"
            // Content substitutions that need evaluation
            | "FileContent"
            | "PathJoinSubstitution"
            // Expression substitutions
            | "PythonExpression"
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pyobject_to_string_plain_string() {
        pyo3::prepare_freethreaded_python();
        Python::with_gil(|py| {
            let s = "hello world";
            let py_str = s.to_object(py);
            let result = pyobject_to_string(py, &py_str).unwrap();
            assert_eq!(result, "hello world");
        });
    }

    #[test]
    fn test_pyobject_to_string_list() {
        pyo3::prepare_freethreaded_python();
        Python::with_gil(|py| {
            // Create a list: ["hello", " ", "world"]
            let list = pyo3::types::PyList::new(py, ["hello", " ", "world"]);
            let py_list = list.to_object(py);
            let result = pyobject_to_string(py, &py_list).unwrap();
            assert_eq!(result, "hello world");
        });
    }

    #[test]
    fn test_pyobject_to_string_number() {
        pyo3::prepare_freethreaded_python();
        Python::with_gil(|py| {
            let num = 42;
            let py_num = num.to_object(py);
            let result = pyobject_to_string(py, &py_num).unwrap();
            assert_eq!(result, "42");
        });
    }
}
