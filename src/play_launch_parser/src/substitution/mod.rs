//! Substitution module

pub mod context;
pub mod parser;
pub mod types;

pub use context::LaunchContext;
pub use parser::parse_substitutions;
pub use types::{resolve_substitutions, Substitution};
