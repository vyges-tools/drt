// SPDX-License-Identifier: Apache-2.0
//! The search-and-repair driver's decisions, iteration by iteration: the strategy rows, the flow
//! an iteration takes, a worker's starting markers and whether it runs, and whether its result
//! is written back.
//!
//! Rules:
//! - the iterations walk the strategy rows in order (the shape cost 8, the marker cost 32), a
//!   row whose rip-up is not ALL widening the clip by the rounded clip increase (up to 18): +2
//!   after an iteration with a congested worker, else −0.2 (never below 0); the run stops once no
//!   marker stands;
//! - an iteration's flow: OPTIMIZATION (the checkerboard) above 100 markers or when everything is
//!   ripped up; else GUIDE tiles when the last iteration changed something or the row differs
//!   from the last one beyond size and offset; else STUBBORN tiles right after guide tiles; else
//!   the iteration is skipped. A markers-driven row with no marker standing does not run at all;
//! - a worker's markers are the design's in its check box, a re-check marker only asking for a
//!   check over everything first; its starting count is theirs, never 0 in the first two
//!   iterations; after the first it is skipped with a count of 0 and nothing to re-check;
//! - a worker's result is the markers touching its check box; it is written back unless it
//!   started from 0 with nothing to re-check (after the first iteration), or, markers driving the
//!   queue, it ended with more than it started, or, everything ripped up after the first
//!   iteration, with more than five times as many.

use crate::gc::{Marker, Rule};
use crate::polygon90::Rect;

/// How much a worker rips up at its start.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RipUp {
    /// Nothing: the worker's markers drive the queue.
    Drc,
    /// Every net.
    All,
    /// The nets near the worker's markers.
    NearDrc,
    /// The nets without routing of their own (an incremental run).
    Incr,
}

/// One strategy row.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IterArgs {
    pub size: i32,
    pub offset: i32,
    pub maze_end: u32,
    pub drc_cost: u32,
    pub marker_cost: u32,
    pub fixed_cost: u32,
    pub decay: f32,
    pub ripup: RipUp,
    pub follow_guide: bool,
}

pub const ROUTE_SHAPE_COST: u32 = 8;
pub const MARKER_COST: u32 = 32;
pub const MAX_CLIPSIZE_INCREASE: i32 = 18;
pub const END_ITERATION: usize = 80;
pub const STUBBORN_FLOW_VIOLATION_THRESHOLD: usize = 100;

impl IterArgs {
    /// Alike but for size and offset (the decay within 1e-6).
    pub fn equal_ignoring_size_and_offset(&self, o: &IterArgs) -> bool {
        self.maze_end == o.maze_end && self.drc_cost == o.drc_cost && self.marker_cost == o.marker_cost && self.fixed_cost == o.fixed_cost && (self.decay - o.decay).abs() < 1e-6 && self.ripup == o.ripup && self.follow_guide == o.follow_guide
    }
}

/// The strategy rows.
pub fn strategy(s: u32, m: u32) -> Vec<IterArgs> {
    use RipUp::*;
    let r = |size, offset, maze_end, drc_cost, marker_cost, fixed_cost, decay, ripup, follow_guide| IterArgs { size, offset, maze_end, drc_cost, marker_cost, fixed_cost, decay, ripup, follow_guide };
    vec![
        r(7, 0, 3, s, 0, s, 0.950, All, true),
        r(7, -2, 3, s, s, s, 0.950, All, true),
        r(7, -5, 3, s, s, s, 0.950, All, true),
        r(7, 0, 8, s, m, 2 * s, 0.950, Drc, false),
        r(7, -1, 8, s, m, 2 * s, 0.950, Drc, false),
        r(7, -2, 8, s, m, 2 * s, 0.950, Drc, false),
        r(7, -3, 8, s, m, 2 * s, 0.950, Drc, false),
        r(7, -4, 8, s, m, 2 * s, 0.950, Drc, false),
        r(7, -5, 8, s, m, 2 * s, 0.950, Drc, false),
        r(7, -6, 8, s, m, 2 * s, 0.950, Drc, false),
        r(7, 0, 8, 2 * s, m, 3 * s, 0.950, Drc, false),
        r(7, -1, 8, 2 * s, m, 3 * s, 0.950, Drc, false),
        r(7, -2, 8, 2 * s, m, 3 * s, 0.950, Drc, false),
        r(7, -3, 8, 2 * s, m, 3 * s, 0.950, Drc, false),
        r(7, -4, 8, 2 * s, m, 3 * s, 0.950, Drc, false),
        r(7, -5, 8, 2 * s, m, 4 * s, 0.950, Drc, false),
        r(7, -6, 8, 2 * s, m, 4 * s, 0.950, Drc, false),
        r(7, -3, 8, s, m, 4 * s, 0.950, All, false),
        r(7, 0, 8, 4 * s, m, 4 * s, 0.950, Drc, false),
        r(7, -1, 8, 4 * s, m, 4 * s, 0.950, Drc, false),
        r(7, -2, 8, 4 * s, m, 10 * s, 0.950, Drc, false),
        r(7, -3, 8, 4 * s, m, 10 * s, 0.950, Drc, false),
        r(7, -4, 8, 4 * s, m, 10 * s, 0.950, Drc, false),
        r(7, -5, 8, s, m, 10 * s, 0.950, NearDrc, false),
        r(7, -6, 8, 4 * s, m, 10 * s, 0.950, Drc, false),
        r(5, -2, 8, s, m, 10 * s, 0.950, All, false),
        r(7, 0, 8, 8 * s, 2 * m, 10 * s, 0.950, Drc, false),
        r(7, -1, 8, 8 * s, 2 * m, 10 * s, 0.950, Drc, false),
        r(7, -2, 8, 8 * s, 2 * m, 10 * s, 0.950, Drc, false),
        r(7, -3, 8, 8 * s, 2 * m, 10 * s, 0.950, Drc, false),
        r(7, -4, 8, s, m, 50 * s, 0.950, NearDrc, false),
        r(7, -5, 8, 8 * s, 2 * m, 50 * s, 0.950, Drc, false),
        r(7, -6, 8, 8 * s, 2 * m, 50 * s, 0.950, Drc, false),
        r(3, -1, 8, s, m, 50 * s, 0.950, All, false),
        r(7, 0, 8, 16 * s, 4 * m, 50 * s, 0.950, Drc, false),
        r(7, -1, 8, 16 * s, 4 * m, 50 * s, 0.950, Drc, false),
        r(7, -2, 8, 16 * s, 4 * m, 50 * s, 0.950, Drc, false),
        r(7, -3, 8, s, m, 50 * s, 0.950, NearDrc, false),
        r(7, -4, 8, 16 * s, 4 * m, 50 * s, 0.950, Drc, false),
        r(7, -5, 8, 16 * s, 4 * m, 50 * s, 0.950, Drc, false),
        r(7, -6, 8, 16 * s, 4 * m, 100 * s, 0.990, Drc, false),
        r(3, -2, 8, s, m, 100 * s, 0.990, All, false),
        r(7, 0, 16, 16 * s, 4 * m, 100 * s, 0.990, Drc, false),
        r(7, -1, 16, 16 * s, 4 * m, 100 * s, 0.990, Drc, false),
        r(7, -2, 16, s, m, 100 * s, 0.990, NearDrc, false),
        r(7, -3, 16, 16 * s, 4 * m, 100 * s, 0.990, Drc, false),
        r(7, -4, 16, 16 * s, 4 * m, 100 * s, 0.990, Drc, false),
        r(7, -5, 16, 16 * s, 4 * m, 100 * s, 0.990, Drc, false),
        r(7, -6, 16, 16 * s, 4 * m, 100 * s, 0.990, Drc, false),
        r(3, 0, 8, s, m, 100 * s, 0.990, All, false),
        r(7, 0, 32, 32 * s, 8 * m, 100 * s, 0.999, Drc, false),
        r(7, -1, 32, s, m, 100 * s, 0.999, NearDrc, false),
        r(7, -2, 32, 32 * s, 8 * m, 100 * s, 0.999, Drc, false),
        r(7, -3, 32, 32 * s, 8 * m, 100 * s, 0.999, Drc, false),
        r(7, -4, 32, 32 * s, 8 * m, 100 * s, 0.999, Drc, false),
        r(7, -5, 32, 32 * s, 8 * m, 100 * s, 0.999, Drc, false),
        r(7, -6, 32, 32 * s, 8 * m, 100 * s, 0.999, Drc, false),
        r(3, -1, 8, s, m, 100 * s, 0.999, All, false),
        r(7, 0, 64, s, m, 100 * s, 0.999, NearDrc, false),
        r(7, -1, 64, 64 * s, 16 * m, 100 * s, 0.999, Drc, false),
        r(7, -2, 64, 64 * s, 16 * m, 100 * s, 0.999, Drc, false),
        r(7, -3, 64, 64 * s, 16 * m, 100 * s, 0.999, Drc, false),
        r(7, -4, 64, 64 * s, 16 * m, 100 * s, 0.999, Drc, false),
        r(7, -5, 64, 64 * s, 16 * m, 100 * s, 0.999, Drc, false),
        r(7, -6, 64, 64 * s, 16 * m, 100 * s, 0.999, Drc, false),
    ]
}

/// The clip widening between iterations.
#[derive(Debug, Clone, Copy, Default)]
pub struct ClipSize {
    inc: f32,
}

impl ClipSize {
    /// The row's size for this iteration; `congested`: a worker of the last iteration was.
    pub fn size(&mut self, row: &IterArgs, congested: bool) -> i32 {
        if row.ripup == RipUp::All {
            return row.size;
        }
        if congested {
            self.inc += 2.0;
        } else {
            self.inc = (self.inc - 0.2).max(0.0);
        }
        row.size + MAX_CLIPSIZE_INCREASE.min(self.inc.round() as i32)
    }
}

/// An iteration's flow.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Flow {
    Optimization,
    Guides,
    Stubborn,
    Skip,
}

/// The flow chooser's memory.
#[derive(Debug, Clone)]
pub struct FlowState {
    current: Flow,
    pub last_effective: bool,
    last_args: Option<IterArgs>,
}

impl Default for FlowState {
    fn default() -> Self {
        FlowState { current: Flow::Optimization, last_effective: true, last_args: None }
    }
}

impl FlowState {
    /// The next iteration's flow, with `violations` markers standing.
    pub fn next(&mut self, violations: usize, args: &IterArgs, fixing_max_spacing: bool) -> Flow {
        self.current = if violations > STUBBORN_FLOW_VIOLATION_THRESHOLD || matches!(args.ripup, RipUp::All | RipUp::Incr) || fixing_max_spacing {
            Flow::Optimization
        } else if self.last_effective || self.last_args.is_none_or(|l| !args.equal_ignoring_size_and_offset(&l)) {
            Flow::Guides
        } else if self.current == Flow::Guides {
            Flow::Stubborn
        } else {
            Flow::Skip
        };
        self.last_args = Some(*args);
        self.current
    }
}

/// A worker's markers at its start.
#[derive(Debug, Clone)]
pub struct WorkerMarkers {
    /// The design's markers in its check box, re-check markers left out (query order).
    pub markers: Vec<Marker>,
    /// A re-check marker stood among them.
    pub need_recheck: bool,
    /// The count it starts from.
    pub init_num: usize,
}

/// The worker's markers from the design's in its check box (`iter`: 0-based).
pub fn worker_markers(in_drc: Vec<Marker>, iter: usize) -> WorkerMarkers {
    let need_recheck = in_drc.iter().any(|m| m.rule == Rule::Recheck);
    let markers: Vec<Marker> = in_drc.into_iter().filter(|m| m.rule != Rule::Recheck).collect();
    let init_num = if iter <= 1 && markers.is_empty() { 1 } else { markers.len() };
    WorkerMarkers { markers, need_recheck, init_num }
}

impl WorkerMarkers {
    /// Whether the worker is skipped (nothing to fix, nothing to re-check).
    pub fn skipped(&self, iter: usize) -> bool {
        iter > 0 && self.init_num == 0 && !self.need_recheck
    }
}

/// The markers touching the check box (a worker's markers, as it keeps them).
pub fn in_check_box(markers: &[Marker], drc: &Rect) -> Vec<Marker> {
    markers.iter().filter(|m| m.bbox.xh >= drc.xl && m.bbox.xl <= drc.xh && m.bbox.yh >= drc.yl && m.bbox.yl <= drc.yh).cloned().collect()
}

/// Whether a worker's result is written back (`best`: its markers in the check box at its end).
pub fn written_back(iter: usize, ripup: RipUp, wm: &WorkerMarkers, best: usize) -> bool {
    if iter > 0 && wm.init_num == 0 && !wm.need_recheck {
        return false;
    }
    if matches!(ripup, RipUp::Drc | RipUp::NearDrc) && best > wm.init_num {
        return false;
    }
    !(iter > 0 && ripup == RipUp::All && best > 5 * wm.init_num)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gc::Owner;

    fn marker(rule: Rule, x: i32) -> Marker {
        let o = Owner::Net("a".into());
        let r = Rect { xl: x, yl: 0, xh: x + 10, yh: 10 };
        Marker { rule, layer: 4, bbox: r, owners: vec![o.clone()], victim: (o.clone(), 4, r, false), aggressor: (o, 4, r, false) }
    }

    /// Rule: 65 rows; the first three rip everything up; row 3 is the first markers-driven one
    /// (marker cost 32, fixed-shape cost 16, maze end 8).
    #[test]
    fn the_strategy_rows() {
        let s = strategy(ROUTE_SHAPE_COST, MARKER_COST);
        assert_eq!(s.len(), 65);
        assert!(s[..3].iter().all(|r| r.ripup == RipUp::All));
        assert_eq!((s[3].ripup, s[3].marker_cost, s[3].fixed_cost, s[3].maze_end), (RipUp::Drc, 32, 16, 8));
        assert_eq!((s[0].marker_cost, s[1].marker_cost), (0, 8));
    }

    /// Rule: the starting count is never 0 in the first two iterations; after the first a worker
    /// with none and nothing to re-check is skipped; a re-check marker only asks for a re-check.
    #[test]
    fn worker_starting_markers() {
        assert_eq!(worker_markers(Vec::new(), 1).init_num, 1);
        assert!(!worker_markers(Vec::new(), 1).skipped(1));
        assert!(worker_markers(Vec::new(), 2).skipped(2));
        let wm = worker_markers(vec![marker(Rule::Recheck, 0)], 2);
        assert!(wm.need_recheck && wm.markers.is_empty() && !wm.skipped(2));
    }

    /// Rule: written back unless, markers driving the queue, the end count exceeds the start, or,
    /// everything ripped up after the first iteration, five times the start.
    #[test]
    fn written_back_when_not_worse() {
        let wm = worker_markers(vec![marker(Rule::Short, 0)], 2);
        assert!(written_back(2, RipUp::Drc, &wm, 1) && !written_back(2, RipUp::Drc, &wm, 2));
        assert!(written_back(2, RipUp::All, &wm, 5) && !written_back(2, RipUp::All, &wm, 6));
        assert!(written_back(0, RipUp::All, &worker_markers(Vec::new(), 0), 99));
    }

    /// Rule: optimization above 100 markers or when all is ripped up; else guides after an
    /// effective iteration, stubborn right after guides, else skip.
    #[test]
    fn the_flow_chooser() {
        let s = strategy(ROUTE_SHAPE_COST, MARKER_COST);
        let mut f = FlowState::default();
        assert_eq!(f.next(3596, &s[0], false), Flow::Optimization);
        assert_eq!(f.next(298, &s[3], false), Flow::Optimization);
        assert_eq!(f.next(18, &s[4], false), Flow::Guides);
        f.last_effective = false;
        assert_eq!(f.next(18, &s[5], false), Flow::Stubborn);
        assert_eq!(f.next(18, &s[6], false), Flow::Skip);
    }

    /// Rule: a markers-driven row widens by the rounded increase, +2 after congestion, −0.2 else.
    #[test]
    fn the_clip_widening() {
        let s = strategy(ROUTE_SHAPE_COST, MARKER_COST);
        let mut c = ClipSize::default();
        assert_eq!(c.size(&s[0], true), 7);
        assert_eq!(c.size(&s[3], true), 9);
        assert_eq!(c.size(&s[4], false), 9);
        assert_eq!(c.size(&s[5], false), 9);
    }
}
