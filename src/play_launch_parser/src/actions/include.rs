//! Include action implementation

use crate::{
    error::{ParseError, Result},
    substitution::{parse_substitutions, Substitution},
    xml::{Entity, EntityExt, XmlEntity},
};

/// Include action representing a nested launch file
#[derive(Debug, Clone)]
pub struct IncludeAction {
    pub file: Vec<Substitution>,
    /// Args as Vec to preserve order (later args can reference earlier ones)
    pub args: Vec<(String, String)>,
}

impl IncludeAction {
    pub fn from_entity(entity: &XmlEntity) -> Result<Self> {
        let file_str =
            entity
                .get_attr_str("file", false)?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "include".to_string(),
                    attribute: "file".to_string(),
                })?;
        let file = parse_substitutions(&file_str)?;

        // Parse child <arg> elements for passing arguments to the included file
        // Use Vec to preserve order (later args can reference earlier ones)
        let mut args = Vec::new();
        for child in entity.children() {
            if child.type_name() == "arg" {
                let name: String =
                    child
                        .get_attr("name", false)?
                        .ok_or_else(|| ParseError::MissingAttribute {
                            element: "arg".to_string(),
                            attribute: "name".to_string(),
                        })?;

                let value: String = child.get_attr("value", false)?.ok_or_else(|| {
                    ParseError::MissingAttribute {
                        element: "arg".to_string(),
                        attribute: "value".to_string(),
                    }
                })?;

                args.push((name, value));
            }
        }

        Ok(Self { file, args })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xml::parse_xml_string;

    #[test]
    fn test_parse_simple_include() {
        let xml = r#"<include file="/path/to/file.launch.xml" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let include = IncludeAction::from_entity(&entity).unwrap();

        assert_eq!(include.file.len(), 1);
        assert!(include.args.is_empty());
    }

    #[test]
    fn test_parse_include_with_args() {
        let xml = r#"<include file="/path/to/file.launch.xml">
            <arg name="param1" value="value1" />
            <arg name="param2" value="value2" />
        </include>"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let include = IncludeAction::from_entity(&entity).unwrap();

        assert_eq!(include.args.len(), 2);
        assert_eq!(
            include.args[0],
            ("param1".to_string(), "value1".to_string())
        );
        assert_eq!(
            include.args[1],
            ("param2".to_string(), "value2".to_string())
        );
    }

    #[test]
    fn test_parse_include_with_substitution() {
        let xml = r#"<include file="$(find-pkg-share demo_nodes_cpp)/launch/talker_listener.launch.xml" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let include = IncludeAction::from_entity(&entity).unwrap();

        // Should parse substitution in file path
        assert!(
            include.file.len() > 1 || matches!(include.file[0], Substitution::FindPackageShare(_))
        );
    }
}
