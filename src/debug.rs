//! Debug groups and stage timers (the suite's debug-flag standard).
//!
//! - `VYGES_DEBUG=drt:<group>=<level>[,...]` turns a group on (`drt:*=1` turns on all of them).
//!   The reference's drt has no debug groups for these traces, so their names start with `vy_`.
//!   `vyges-drt --describe` lists them.
//! - The old variables still work. Each one turns its group on, with a one-time deprecation note.
//!   Where an old variable carries a value (a box, a net, an output path), the trace still reads
//!   that value.
//! - A box-filtered trace (`vy_worker_detail`, `vy_queue`, `vy_route_cost`, `vy_stage_cost`) runs
//!   only in the worker whose route box starts at `<xl>,<yl>`. The box comes from the group's old
//!   variable when that holds one, otherwise from `VYGES_DRT_DEBUG_BOX=<xl>,<yl>`. With neither
//!   set, the trace runs in every worker.
//! - A group only changes what is printed, never a result. The traces write to stderr, and the
//!   worker record goes to its own file.
//! - `VYGES_STAGE_TIMES=1` prints one line per stage: wall time, CPU time, RSS and peak RSS. The
//!   stages are named after the reference functions they reproduce. This needs the `odb` feature
//!   (vyges-events); without it [`stage`] does nothing.
//!
//! Every answer is read once per process and then cached, so a check costs one atomic load.

use crate::polygon90::Rect;
use std::sync::OnceLock;

/// The tool name in `VYGES_DEBUG`.
pub const TOOL: &str = "drt";

/// A drt debug group.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Group {
    /// One record per worker at its batch's write-back, and the design's marker count after each
    /// batch (`VYGW|w|...`, `VYGW|b|...`).
    WorkerRecord,
    /// After the boxed worker's record: its routed nets' shapes and its best markers.
    WorkerDetail,
    /// The boxed worker's route queue: each route, check, marker (rule, side boxes) and push.
    Queue,
    /// Grid costs before a net's route, and its wavefront pops.
    RouteCost,
    /// Grid costs after each initMazeCost stage, markerCost and initQueue.
    StageCost,
    /// The check's view of an owner (`gcinit` / `gcupd` and its maximal rectangles).
    Gc,
    /// One grid edge's costs at each step (`VYGD_EDGEWATCH=z,x,y,d`).
    EdgeWatch,
    /// The block-pin corner cost, each call (`c2t` / `c2m` / `c2c`).
    C2c,
}

/// One row of the group table: group, old variable, whether it is box-filtered, what it prints.
pub struct Row {
    pub group: Group,
    pub name: &'static str,
    pub old_var: &'static str,
    pub boxed: bool,
    pub what: &'static str,
}

/// The group table, listed by `--describe`.
pub const GROUPS: [Row; 8] = [
    Row { group: Group::WorkerRecord, name: "vy_worker_record", old_var: "VYGW_OUT", boxed: false, what: "per-worker record at write-back + design markers per batch (file: VYGW_OUT, else VYGES_DEBUG_DIR/drt/worker-record.vygw, else stderr)" },
    Row { group: Group::WorkerDetail, name: "vy_worker_detail", old_var: "VYGW_BOX", boxed: true, what: "the boxed worker's best route shapes per net and best markers, after its record" },
    Row { group: Group::Queue, name: "vy_queue", old_var: "VYGD_QUEUE_BOX", boxed: true, what: "the boxed worker's route queue: q|R route, q|C check, q|M/q|X marker and sides, q|P push" },
    Row { group: Group::RouteCost, name: "vy_route_cost", old_var: "VYGD_ROUTECL", boxed: true, what: "grid costs before a net's route (VYGD_ROUTECL=<net id>,<reroutes>[,<nth>] picks one; nth adds its wavefront pops)" },
    Row { group: Group::StageCost, name: "vy_stage_cost", old_var: "VYGD_STAGECL_BOX", boxed: true, what: "the boxed worker's grid costs after each initMazeCost stage, markerCost and initQueue" },
    Row { group: Group::Gc, name: "vy_gc", old_var: "VYGD_GC", boxed: false, what: "the check's view of an owner: gcinit / gcupd and its maximal rectangles" },
    Row { group: Group::EdgeWatch, name: "vy_edgewatch", old_var: "VYGD_EDGEWATCH", boxed: false, what: "one grid edge's costs at each step (VYGD_EDGEWATCH=z,x,y,d)" },
    Row { group: Group::C2c, name: "vy_c2c", old_var: "VYGC_C2C", boxed: false, what: "the block-pin corner cost, each call (c2t / c2m / c2c lines)" },
];

fn row(g: Group) -> &'static Row {
    &GROUPS[g as usize]
}

/// The value of an environment variable, or `None` if it is unset, empty or `0`.
fn set_value(var: &str) -> Option<String> {
    std::env::var(var).ok().filter(|v| !v.trim().is_empty() && v.trim() != "0")
}

/// The old variables, mapped onto their groups (each with a one-time deprecation note when set).
/// Idempotent; the binary calls it at the start of `main`, and the first group check calls it
/// for a library caller that did not.
#[cfg(feature = "odb")]
pub fn register_aliases() {
    static ALIASES: std::sync::Once = std::sync::Once::new();
    ALIASES.call_once(|| {
        for r in &GROUPS {
            vyges_events::debug::register_alias(r.old_var, TOOL, r.name, 1);
        }
    });
}

/// The group table as the shared descriptor type (`--describe`, unknown-group warnings).
#[cfg(feature = "odb")]
pub fn debug_groups() -> Vec<vyges_events::debug::DebugGroup> {
    GROUPS.iter().map(|r| vyges_events::debug::DebugGroup { group: r.name, max_level: 1, what: r.what, reference: None }).collect()
}

/// Start of `main`: the aliases, then a warning for each unknown `drt:` group in `VYGES_DEBUG`.
#[cfg(feature = "odb")]
pub fn init() {
    register_aliases();
    vyges_events::debug::warn_unknown_debug_groups(TOOL, &debug_groups());
}

#[cfg(feature = "odb")]
fn compute(g: Group) -> bool {
    register_aliases();
    vyges_events::debug::debug_enabled(TOOL, row(g).name, 1)
}

#[cfg(not(feature = "odb"))]
fn compute(g: Group) -> bool {
    set_value(row(g).old_var).is_some()
}

/// Whether a group is on (`VYGES_DEBUG` or its old variable). Read once, then cached.
pub fn on(g: Group) -> bool {
    static C: [OnceLock<bool>; GROUPS.len()] = [const { OnceLock::new() }; GROUPS.len()];
    *C[g as usize].get_or_init(|| compute(g))
}

/// The value the group's old variable carries (a box, a net, a path), if it is set.
pub fn value(g: Group) -> Option<String> {
    set_value(row(g).old_var)
}

/// The box a boxed group is restricted to (`<xl>,<yl>`), or `None` for every worker.
fn box_of(g: Group) -> Option<String> {
    static C: [OnceLock<Option<String>>; GROUPS.len()] = [const { OnceLock::new() }; GROUPS.len()];
    C[g as usize]
        .get_or_init(|| {
            let own = match g {
                Group::RouteCost => set_value("VYGD_ROUTECL_BOX"),
                _ if row(g).boxed => value(g),
                _ => None,
            };
            own.or_else(|| set_value("VYGES_DRT_DEBUG_BOX"))
        })
        .clone()
}

/// Whether a box-filtered group traces the worker whose route box is `route_box`.
pub fn box_wanted(g: Group, route_box: &Rect) -> bool {
    on(g) && box_matches(box_of(g).as_deref(), route_box)
}

/// A box filter (`<xl>,<yl>`; none = every box) against a route box.
pub fn box_matches(filter: Option<&str>, route_box: &Rect) -> bool {
    filter.is_none_or(|b| b == format!("{},{}", route_box.xl, route_box.yl))
}

/// Where the worker record goes: its old variable's path, else `VYGES_DEBUG_DIR/drt/`, else
/// stderr (`None`).
pub fn worker_record_path() -> Option<std::path::PathBuf> {
    if let Some(p) = value(Group::WorkerRecord) {
        return Some(p.into());
    }
    #[cfg(feature = "odb")]
    if let Some(d) = vyges_events::debug::debug_dir(TOOL) {
        return Some(d.join("worker-record.vygw"));
    }
    None
}

/// A stage timer, named after the reference function it reproduces. It reports when dropped, and
/// only when `VYGES_STAGE_TIMES` is set.
#[cfg(feature = "odb")]
pub fn stage(name: &str) -> vyges_events::stage::Stage {
    vyges_events::stage::stage_in(TOOL, name)
}

/// Without the `odb` feature there is no logger and no stage line.
#[cfg(not(feature = "odb"))]
pub fn stage(_name: &str) -> NoStage {
    NoStage
}

/// The stage guard without the `odb` feature: reports nothing.
#[cfg(not(feature = "odb"))]
pub struct NoStage;

#[cfg(test)]
mod tests {
    use super::*;

    /// The table and the enum agree, so `on` and `--describe` name the same groups.
    #[test]
    fn every_group_has_its_row_in_order() {
        for (k, r) in GROUPS.iter().enumerate() {
            assert_eq!(r.group as usize, k, "{}", r.name);
            assert!(r.name.starts_with("vy_"), "{}: no reference group, so vy_", r.name);
        }
    }

    /// A box filter names a route box by its lower-left corner; no filter = every worker.
    #[test]
    fn a_box_filter_names_the_lower_left_corner() {
        let rb = Rect { xl: 47880, yl: 323190, xh: 51870, yh: 327180 };
        assert!(box_matches(None, &rb));
        assert!(box_matches(Some("47880,323190"), &rb));
        assert!(!box_matches(Some("47880,323191"), &rb) && !box_matches(Some("51870,327180"), &rb));
    }
}
