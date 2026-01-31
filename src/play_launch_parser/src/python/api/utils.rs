//! Shared utilities for Python API type conversions
//!
//! This module provides common helper functions for converting between
//! Python types and Rust types, particularly for handling ROS 2's
//! SomeSubstitutionsType pattern.

use pyo3::prelude::*;

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
/// # Examples
///
/// ```ignore
/// // Plain string
/// let result = pyobject_to_string(py, &py_string)?;
///
/// // LaunchConfiguration substitution
/// let result = pyobject_to_string(py, &launch_config)?;
/// // Result: "$(var variable_name)" or resolved value
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
/// 3. Call `perform()` method (for LaunchConfiguration with context)
/// 4. Call `__str__()` method (for substitution objects)
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

    // Try calling __str__ method first (for substitutions)
    // This preserves substitution format like "$(var node_name)" instead of resolving it
    if let Ok(str_result) = obj_ref.call_method0("__str__") {
        if let Ok(s) = str_result.extract::<String>() {
            return Ok(s);
        }
    }

    // Try calling perform() method as fallback (for LaunchConfiguration)
    // This resolves the substitution to its actual value
    if obj_ref.hasattr("perform")? {
        if let Ok(context) = py.eval("type('Context', (), {})()", None, None) {
            if let Ok(result) = obj_ref.call_method1("perform", (context,)) {
                if let Ok(s) = result.extract::<String>() {
                    return Ok(s);
                }
            }
        }
    }

    // Fallback to Python repr
    Ok(obj_ref.to_string())
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
