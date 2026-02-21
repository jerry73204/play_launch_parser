//! Action module

pub mod arg;
pub mod container;
pub mod declare_argument;
pub mod executable;
pub mod group;
pub mod include;
pub mod let_action;
pub mod load_composable_node;
pub mod node;
pub mod set_env;
pub mod set_parameter;
pub mod set_remap;

pub use arg::ArgAction;
pub use container::{ComposableNodeAction, ContainerAction};
pub use declare_argument::DeclareArgumentAction;
pub use executable::ExecutableAction;
pub use group::GroupAction;
pub use include::IncludeAction;
pub use let_action::LetAction;
pub use load_composable_node::LoadComposableNodeAction;
pub use node::{NodeAction, Parameter, Remapping};
pub use set_env::{SetEnvAction, UnsetEnvAction};
pub use set_parameter::SetParameterAction;
pub use set_remap::SetRemapAction;

// --- From impls for converting action types to IR ActionKind ---

use crate::ir::{ActionKind, ComposableNodeDecl, EnvDecl, Expr, IncludeArg, ParamDecl, RemapDecl};

impl From<NodeAction> for ActionKind {
    fn from(node: NodeAction) -> Self {
        ActionKind::SpawnNode {
            package: Expr(node.package),
            executable: Expr(node.executable),
            name: node.name.map(Expr),
            namespace: node.namespace.map(Expr),
            params: node
                .parameters
                .into_iter()
                .map(|p| ParamDecl {
                    name: p.name,
                    value: Expr(p.value),
                })
                .collect(),
            param_files: node.param_files.into_iter().map(Expr).collect(),
            remaps: node
                .remappings
                .into_iter()
                .map(|r| RemapDecl {
                    from: Expr(r.from),
                    to: Expr(r.to),
                })
                .collect(),
            env: node
                .environment
                .into_iter()
                .map(|(k, v)| EnvDecl {
                    name: Expr::literal(k),
                    value: Expr::literal(v),
                })
                .collect(),
            args: node.args.map(Expr),
            respawn: node.respawn.map(Expr),
            respawn_delay: node.respawn_delay.map(Expr),
        }
    }
}

impl From<ExecutableAction> for ActionKind {
    fn from(exec: ExecutableAction) -> Self {
        ActionKind::SpawnExecutable {
            cmd: Expr(exec.cmd),
            name: exec.name.map(Expr),
            args: exec.arguments.into_iter().map(Expr).collect(),
            env: exec
                .environment
                .into_iter()
                .map(|(k, v)| EnvDecl {
                    name: Expr::literal(k),
                    value: Expr::literal(v),
                })
                .collect(),
        }
    }
}

impl From<ContainerAction> for ActionKind {
    fn from(c: ContainerAction) -> Self {
        ActionKind::SpawnContainer {
            package: Expr(c.package),
            executable: Expr(c.executable),
            name: Expr(c.name),
            namespace: c.namespace.map(Expr),
            args: c.args.map(Expr),
            nodes: c.composable_nodes.into_iter().map(Into::into).collect(),
        }
    }
}

impl From<ComposableNodeAction> for ComposableNodeDecl {
    fn from(n: ComposableNodeAction) -> Self {
        ComposableNodeDecl {
            package: Expr(n.package),
            plugin: Expr(n.plugin),
            name: Expr(n.name),
            namespace: n.namespace.map(Expr),
            params: n
                .parameters
                .into_iter()
                .map(|(k, v)| ParamDecl {
                    name: k,
                    value: Expr::literal(v),
                })
                .collect(),
            remaps: n
                .remappings
                .into_iter()
                .map(|(f, t)| RemapDecl {
                    from: Expr::literal(f),
                    to: Expr::literal(t),
                })
                .collect(),
            extra_args: n.extra_args.into_iter().collect(),
            condition: None,
            span: None,
        }
    }
}

impl From<LoadComposableNodeAction> for ActionKind {
    fn from(l: LoadComposableNodeAction) -> Self {
        ActionKind::LoadComposableNode {
            target: Expr(l.target),
            nodes: l.composable_nodes.into_iter().map(Into::into).collect(),
        }
    }
}

impl From<IncludeAction> for ActionKind {
    fn from(inc: IncludeAction) -> Self {
        ActionKind::Include {
            file: Expr(inc.file),
            args: inc
                .args
                .into_iter()
                .map(|(name, value)| IncludeArg {
                    name,
                    value: Expr(value),
                })
                .collect(),
            body: None,
        }
    }
}

impl From<SetEnvAction> for ActionKind {
    fn from(a: SetEnvAction) -> Self {
        ActionKind::SetEnv {
            name: a.name,
            value: Expr(a.value),
        }
    }
}

impl From<UnsetEnvAction> for ActionKind {
    fn from(a: UnsetEnvAction) -> Self {
        ActionKind::UnsetEnv { name: a.name }
    }
}

impl From<SetParameterAction> for ActionKind {
    fn from(a: SetParameterAction) -> Self {
        ActionKind::SetParameter {
            name: a.name,
            value: Expr(a.value),
        }
    }
}

impl From<SetRemapAction> for ActionKind {
    fn from(a: SetRemapAction) -> Self {
        ActionKind::SetRemap {
            from: Expr(a.from),
            to: Expr(a.to),
        }
    }
}

impl From<GroupAction> for ActionKind {
    fn from(g: GroupAction) -> Self {
        ActionKind::Group {
            namespace: g.namespace.map(Expr),
            body: Vec::new(),
        }
    }
}
