//! XML entity abstraction

use crate::error::{ParseError, Result};
use std::str::FromStr;

/// Entity trait for abstracting XML/YAML differences
pub trait Entity {
    /// Get entity type name (e.g., "node", "arg")
    fn type_name(&self) -> &str;

    /// Get attribute as string
    fn get_attr_str(&self, name: &str, optional: bool) -> Result<Option<String>>;

    /// Get all attributes as key-value pairs
    fn attributes(&self) -> Vec<(&str, &str)>;

    /// Get text content
    fn text(&self) -> Option<&str>;
}

/// Extension trait for type-aware attribute access
pub trait EntityExt: Entity {
    /// Get attribute value with type coercion
    fn get_attr<T: FromStr>(&self, name: &str, optional: bool) -> Result<Option<T>> {
        match self.get_attr_str(name, optional)? {
            Some(value) => {
                let parsed = value.parse::<T>().map_err(|_| ParseError::TypeCoercion {
                    attribute: name.to_string(),
                    value: value.to_string(),
                    expected_type: std::any::type_name::<T>(),
                })?;
                Ok(Some(parsed))
            }
            None => Ok(None),
        }
    }

    /// Get child entities (must be implemented per entity type)
    fn children(&self) -> Vec<&dyn Entity>;
}

// Blanket implementation for all Entity types
impl<T: Entity + ?Sized> EntityExt for T {
    fn children(&self) -> Vec<&dyn Entity> {
        // Default: no children
        Vec::new()
    }
}

/// XML entity implementation wrapping roxmltree::Node
pub struct XmlEntity<'a, 'input> {
    node: roxmltree::Node<'a, 'input>,
}

impl<'a, 'input> XmlEntity<'a, 'input> {
    pub fn new(node: roxmltree::Node<'a, 'input>) -> Self {
        Self { node }
    }
}

impl<'a, 'input> Entity for XmlEntity<'a, 'input> {
    fn type_name(&self) -> &str {
        self.node.tag_name().name()
    }

    fn get_attr_str(&self, name: &str, optional: bool) -> Result<Option<String>> {
        match self.node.attribute(name) {
            Some(value) => Ok(Some(value.to_string())),
            None if optional => Ok(None),
            None => Err(ParseError::MissingAttribute {
                element: self.type_name().to_string(),
                attribute: name.to_string(),
            }),
        }
    }

    fn attributes(&self) -> Vec<(&str, &str)> {
        self.node
            .attributes()
            .map(|attr| (attr.name(), attr.value()))
            .collect()
    }

    fn text(&self) -> Option<&str> {
        self.node.text()
    }
}

impl<'a, 'input> XmlEntity<'a, 'input> {
    /// Return an iterator over child elements
    /// This avoids allocating a Vec on every call, enabling lazy evaluation
    pub fn children(&self) -> impl Iterator<Item = XmlEntity<'a, 'input>> {
        self.node
            .children()
            .filter(|n| n.is_element())
            .map(XmlEntity::new)
        // No .collect() - returns iterator directly
    }
}
