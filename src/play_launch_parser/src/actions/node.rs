//! Node action implementation

use crate::error::{ParseError, Result};
use crate::substitution::{parse_substitutions, Substitution};
use crate::xml::{Entity, EntityExt, XmlEntity};

/// Node action representing a ROS 2 node
#[derive(Debug, Clone)]
pub struct NodeAction {
    pub package: Vec<Substitution>,
    pub executable: Vec<Substitution>,
    pub name: Option<Vec<Substitution>>,
    pub namespace: Option<Vec<Substitution>>,
    pub parameters: Vec<Parameter>,
    pub remappings: Vec<Remapping>,
    pub environment: Vec<(String, String)>,
    pub output: Option<String>,
    pub respawn: bool,
    pub respawn_delay: Option<f64>,
}

impl NodeAction {
    pub fn from_entity(entity: &XmlEntity) -> Result<Self> {
        let package =
            entity
                .get_attr_str("pkg", false)?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "node".to_string(),
                    attribute: "pkg".to_string(),
                })?;
        let package = parse_substitutions(&package)?;

        let executable =
            entity
                .get_attr_str("exec", false)?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "node".to_string(),
                    attribute: "exec".to_string(),
                })?;
        let executable = parse_substitutions(&executable)?;

        let name = entity
            .get_attr_str("name", true)?
            .map(|s| parse_substitutions(&s))
            .transpose()?;

        let namespace = entity
            .get_attr_str("namespace", true)?
            .map(|s| parse_substitutions(&s))
            .transpose()?;

        // Parse children for params, remaps, env
        let mut parameters = Vec::new();
        let mut remappings = Vec::new();
        let mut environment = Vec::new();

        for child in entity.children() {
            match child.type_name() {
                "param" => parameters.push(Parameter::from_entity(&child)?),
                "remap" => remappings.push(Remapping::from_entity(&child)?),
                "env" => environment.push(parse_env(&child)?),
                other => {
                    return Err(ParseError::UnexpectedElement {
                        parent: "node".to_string(),
                        child: other.to_string(),
                    })
                }
            }
        }

        Ok(Self {
            package,
            executable,
            name,
            namespace,
            parameters,
            remappings,
            environment,
            output: entity.get_attr("output", true)?,
            respawn: entity.get_attr("respawn", true)?.unwrap_or(false),
            respawn_delay: entity.get_attr("respawn_delay", true)?,
        })
    }
}

#[derive(Debug, Clone)]
pub struct Parameter {
    pub name: String,
    pub value: String,
}

impl Parameter {
    pub fn from_entity(entity: &XmlEntity) -> Result<Self> {
        Ok(Self {
            name: entity
                .get_attr("name", false)?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "param".to_string(),
                    attribute: "name".to_string(),
                })?,
            value: entity.get_attr("value", false)?.ok_or_else(|| {
                ParseError::MissingAttribute {
                    element: "param".to_string(),
                    attribute: "value".to_string(),
                }
            })?,
        })
    }
}

#[derive(Debug, Clone)]
pub struct Remapping {
    pub from: String,
    pub to: String,
}

impl Remapping {
    pub fn from_entity(entity: &XmlEntity) -> Result<Self> {
        Ok(Self {
            from: entity
                .get_attr("from", false)?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "remap".to_string(),
                    attribute: "from".to_string(),
                })?,
            to: entity
                .get_attr("to", false)?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "remap".to_string(),
                    attribute: "to".to_string(),
                })?,
        })
    }
}

fn parse_env(entity: &XmlEntity) -> Result<(String, String)> {
    let name: String =
        entity
            .get_attr("name", false)?
            .ok_or_else(|| ParseError::MissingAttribute {
                element: "env".to_string(),
                attribute: "name".to_string(),
            })?;
    let value: String =
        entity
            .get_attr("value", false)?
            .ok_or_else(|| ParseError::MissingAttribute {
                element: "env".to_string(),
                attribute: "value".to_string(),
            })?;
    Ok((name, value))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xml::parse_xml_string;

    #[test]
    fn test_parse_simple_node() {
        let xml = r#"<node pkg="demo_nodes_cpp" exec="talker" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let node = NodeAction::from_entity(&entity).unwrap();

        assert_eq!(node.package.len(), 1);
        assert_eq!(node.executable.len(), 1);
        assert!(node.name.is_none());
        assert!(node.namespace.is_none());
    }

    #[test]
    fn test_parse_node_with_name() {
        let xml = r#"<node pkg="demo_nodes_cpp" exec="talker" name="my_talker" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let node = NodeAction::from_entity(&entity).unwrap();

        assert!(node.name.is_some());
    }

    #[test]
    fn test_parse_node_with_param() {
        let xml = r#"<node pkg="demo" exec="node">
            <param name="rate" value="10.0" />
        </node>"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let node = NodeAction::from_entity(&entity).unwrap();

        assert_eq!(node.parameters.len(), 1);
        assert_eq!(node.parameters[0].name, "rate");
        assert_eq!(node.parameters[0].value, "10.0");
    }

    #[test]
    fn test_parse_node_with_remap() {
        let xml = r#"<node pkg="demo" exec="node">
            <remap from="chatter" to="/chat" />
        </node>"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let node = NodeAction::from_entity(&entity).unwrap();

        assert_eq!(node.remappings.len(), 1);
        assert_eq!(node.remappings[0].from, "chatter");
        assert_eq!(node.remappings[0].to, "/chat");
    }

    #[test]
    fn test_parse_node_with_env() {
        let xml = r#"<node pkg="demo" exec="node">
            <env name="MY_VAR" value="my_value" />
        </node>"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let node = NodeAction::from_entity(&entity).unwrap();

        assert_eq!(node.environment.len(), 1);
        assert_eq!(node.environment[0].0, "MY_VAR");
        assert_eq!(node.environment[0].1, "my_value");
    }
}
