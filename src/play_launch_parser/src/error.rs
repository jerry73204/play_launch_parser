//! Error types for the play_launch_parser

use thiserror::Error;

#[derive(Error, Debug)]
pub enum ParseError {
    #[error("XML parsing error: {0}")]
    XmlError(#[from] roxmltree::Error),

    #[error("Missing required attribute '{attribute}' on element '{element}'")]
    MissingAttribute { element: String, attribute: String },

    #[error("Type coercion failed for attribute '{attribute}' with value '{value}' (expected {expected_type})")]
    TypeCoercion {
        attribute: String,
        value: String,
        expected_type: &'static str,
    },

    #[error("Unexpected element '{child}' in '{parent}'")]
    UnexpectedElement { parent: String, child: String },

    #[error("Invalid substitution syntax: {0}")]
    InvalidSubstitution(String),

    #[error("File not found: {0}")]
    FileNotFound(String),

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),
}

#[derive(Error, Debug)]
pub enum SubstitutionError {
    #[error("Undefined variable: {0}")]
    UndefinedVariable(String),

    #[error("Undefined environment variable: {0}")]
    UndefinedEnvVar(String),

    #[error("Package not found: {0}")]
    PackageNotFound(String),

    #[error("Invalid substitution: {0}")]
    InvalidSubstitution(String),
}

#[derive(Error, Debug)]
pub enum GenerationError {
    #[error("Substitution error: {0}")]
    Substitution(#[from] SubstitutionError),

    #[error("Package not found: {0}")]
    PackageNotFound(String),

    #[error("Executable not found: {0}")]
    ExecutableNotFound(String),
}

pub type Result<T> = std::result::Result<T, ParseError>;
