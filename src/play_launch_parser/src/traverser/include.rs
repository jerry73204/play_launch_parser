use super::super::LaunchTraverser;
use crate::{
    actions::IncludeAction,
    error::{ParseError, Result},
    file_cache::read_file_cached,
    record,
    substitution::{parse_substitutions, resolve_substitutions},
    xml,
};
use rayon::prelude::*;
use std::path::Path;

impl LaunchTraverser {
    pub(crate) fn process_include(&mut self, include: &IncludeAction) -> Result<()> {
        // Resolve the file path
        let file_path_str = resolve_substitutions(&include.file, &self.context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
        let file_path = Path::new(&file_path_str);

        log::trace!("Processing include: {}", file_path_str);

        // Resolve relative paths relative to the current launch file
        let resolved_path = if file_path.is_relative() {
            if let Some(current_file) = self.context.current_file() {
                if let Some(parent_dir) = current_file.parent() {
                    parent_dir.join(file_path)
                } else {
                    file_path.to_path_buf()
                }
            } else {
                file_path.to_path_buf()
            }
        } else {
            file_path.to_path_buf()
        };

        // Canonicalize path for circular include detection
        let canonical_path = resolved_path
            .canonicalize()
            .unwrap_or_else(|_| resolved_path.clone());

        // Check for circular includes in the current include chain
        if self.include_chain.contains(&canonical_path) {
            log::warn!("Circular include detected: {}", canonical_path.display());
            return Ok(()); // Skip circular includes
        }

        log::info!("Including launch file: {}", resolved_path.display());

        // Log include arguments being passed
        if !include.args.is_empty() {
            let arg_names: Vec<&str> = include.args.iter().map(|(k, _)| k.as_str()).collect();
            log::trace!("Include args: {:?}", arg_names);
        }

        // Check if this is a Python launch file
        // Check file extension and handle non-XML files
        if let Some(ext) = resolved_path.extension().and_then(|s| s.to_str()) {
            match ext {
                "py" => {
                    log::info!("Including Python launch file: {}", resolved_path.display());

                    // Create args for the Python file (include args override current context)
                    let mut python_args = self.context.configurations();
                    for (key, value) in &include.args {
                        let resolved_value_subs = parse_substitutions(value)?;
                        let resolved_value =
                            resolve_substitutions(&resolved_value_subs, &self.context)
                                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                        log::trace!("  Include arg: {} = {}", key, resolved_value);
                        python_args.insert(key.clone(), resolved_value);
                    }

                    return self.execute_python_file(&resolved_path, &python_args);
                }
                "yaml" | "yml" => {
                    // YAML files in <include> are always launch files
                    // (parameter files are handled in <param from="..."> context)
                    log::info!("Including YAML launch file: {}", resolved_path.display());
                    self.process_yaml_launch_file(&resolved_path)?;
                    return Ok(());
                }
                _ => {}
            }
        }

        // Create a new context for the included file (O(1) with Arc, not O(n) clone!)
        // Start with current context and apply include args
        let mut include_context = self.context.child();
        include_context.set_current_file(resolved_path.clone());
        for (key, value) in &include.args {
            // IMPORTANT: Resolve substitutions in the argument value using the include_context
            // (not the parent context) so that later args can reference earlier args
            // Example: <arg name="A" value="x"/>
            //          <arg name="B" value="$(var A)/y"/>  <-- B can reference A
            let resolved_value_subs = parse_substitutions(value)?;
            let resolved_value = resolve_substitutions(&resolved_value_subs, &include_context)
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
            log::debug!("[RUST] Setting include arg: {} = {}", key, resolved_value);
            include_context.set_configuration(key.clone(), resolved_value);
        }

        log::debug!(
            "[RUST] Include context has {} configs after setting include args",
            include_context.configurations().len()
        );

        // Parse and traverse the included file
        let content = read_file_cached(&resolved_path)?;
        let doc = roxmltree::Document::parse(&content)?;
        let root = xml::XmlEntity::new(doc.root_element());

        // Create temporary traverser for included file with extended include chain
        let mut child_chain = self.include_chain.clone();
        child_chain.push(canonical_path);

        let mut included_traverser = LaunchTraverser {
            context: include_context,
            include_chain: child_chain,
            records: Vec::new(),
            containers: Vec::new(),
            load_nodes: Vec::new(),
        };
        included_traverser.traverse_entity(&root)?;

        // Merge records from included file into current records
        self.records.extend(included_traverser.records);
        self.containers.extend(included_traverser.containers);
        self.load_nodes.extend(included_traverser.load_nodes);

        // Merge captures from included file's context
        for node in included_traverser.context.captured_nodes() {
            self.context.capture_node(node.clone());
        }

        for container in included_traverser.context.captured_containers() {
            self.context.capture_container(container.clone());
        }

        for load_node in included_traverser.context.captured_load_nodes() {
            self.context.capture_load_node(load_node.clone());
        }

        // CRITICAL: Merge configurations from included file back to parent context
        // This ensures that <arg> defaults defined in included files are visible to parent
        // Example: included file has <arg name="use_multithread" default="true"/>
        //          Parent file needs to see use_multithread=true for conditionals
        let included_configs = included_traverser.context.configurations();
        log::debug!(
            "Included file has {} configurations to merge",
            included_configs.len()
        );
        for (key, value) in included_configs {
            // Only set if not already defined in parent (parent takes precedence)
            if self.context.get_configuration(&key).is_none() {
                log::debug!("Merging config from included file: {} = {}", key, value);
                self.context.set_configuration(key, value);
            } else {
                log::debug!("Skipping {} (already set in parent)", key);
            }
        }

        Ok(())
    }

    /// Process multiple includes in parallel using rayon
    /// Returns aggregated (nodes, containers, load_nodes)
    pub(crate) fn process_includes_parallel(
        &self,
        includes: Vec<IncludeAction>,
    ) -> Result<(
        Vec<record::NodeRecord>,
        Vec<record::ComposableNodeContainerRecord>,
        Vec<record::LoadNodeRecord>,
    )> {
        // Capture current file and include chain for parallel tasks
        let current_file = self.context.current_file().cloned();
        let include_chain = self.include_chain.clone();

        // Process includes in parallel using rayon
        let results: Vec<Result<_>> = includes
            .par_iter()
            .map(|include| {
                // Each thread gets its own traverser with preserved context
                // We clone the context for each thread (context is cheap to clone after Phase 7.2)
                let mut thread_context = self.context.clone();

                // Preserve current file for relative path resolution
                if let Some(ref file) = current_file {
                    thread_context.set_current_file(file.clone());
                }

                let mut traverser = LaunchTraverser {
                    context: thread_context,
                    include_chain: include_chain.clone(), // Each thread gets its own chain
                    records: Vec::new(),
                    containers: Vec::new(),
                    load_nodes: Vec::new(),
                };

                // Process the include
                traverser.process_include(include)?;

                Ok((
                    traverser.records,
                    traverser.containers,
                    traverser.load_nodes,
                ))
            })
            .collect();

        // Merge results from all parallel tasks
        let mut all_records = Vec::new();
        let mut all_containers = Vec::new();
        let mut all_load_nodes = Vec::new();

        for result in results {
            let (records, containers, load_nodes) = result?;
            all_records.extend(records);
            all_containers.extend(containers);
            all_load_nodes.extend(load_nodes);
        }

        Ok((all_records, all_containers, all_load_nodes))
    }
}
