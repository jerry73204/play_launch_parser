//! Record module for generating record.json

pub mod generator;
pub mod types;

pub use generator::CommandGenerator;
pub use types::{ComposableNodeContainerRecord, LoadNodeRecord, NodeRecord, RecordJson};
