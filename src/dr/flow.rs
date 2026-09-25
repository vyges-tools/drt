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

use std::collections::BTreeSet;

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

/// The rip-up mode an iteration runs: in an incremental run (some nets routed before routing
/// began) the first three iterations (0–2) rip up only the other nets where the strategy rips up
/// everything.
pub fn effective_ripup(row: RipUp, incremental: bool, iter: usize) -> RipUp {
    if row == RipUp::All && incremental && iter <= 2 {
        RipUp::Incr
    } else {
        row
    }
}

/// Whether a worker net starts in the first queue: ripping up everything, a net with more than
/// one pin; incremental, a net not routed before (whatever its pins).
pub fn first_ripped(ripup: RipUp, pins: usize, routed_before: bool) -> bool {
    if ripup == RipUp::Incr {
        !routed_before
    } else {
        pins > 1
    }
}

/// Whether a marker may NOT rip up a worker net: in an incremental iteration, a net routed before.
pub fn ripup_pinned(ripup: RipUp, routed_before: bool) -> bool {
    ripup == RipUp::Incr && routed_before
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
    let markers: Vec<Marker> = in_drc.into_iter().filter(|m| m.rule != Rule::Recheck).map(|m| m.copied()).collect();
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

/// Whether a box runs vertically (narrower than tall; a square counts horizontal).
fn is_vertical(r: &Rect) -> bool {
    r.xh - r.xl < r.yh - r.yl
}

fn touch(a: &Rect, b: &Rect) -> bool {
    a.xh >= b.xl && a.xl <= b.xh && a.yh >= b.yl && a.yl <= b.yh
}

/// The guide-tiles flow's worker boxes: per marker (design order), per source net with original
/// guides (`guides`), each of its guides touching the marker; a marker with a guided net that no
/// guide covers takes `off_guide`'s boxes instead. Then boxes alike in direction that touch are
/// merged, the scan starting over after each merge.
pub fn guide_tile_boxes(markers: &[Marker], guides: &dyn Fn(&crate::gc::Owner) -> Option<Vec<Rect>>, off_guide: &dyn Fn(&Marker) -> Vec<Rect>) -> Vec<Rect> {
    let mut boxes: Vec<Rect> = Vec::new();
    for m in markers {
        let (mut covered, mut guided) = (false, false);
        for o in &m.owners {
            let Some(g) = guides(o) else { continue };
            if g.is_empty() {
                continue;
            }
            guided = true;
            for r in g {
                if touch(&r, &m.bbox) {
                    boxes.push(r);
                    covered = true;
                }
            }
        }
        if guided && !covered {
            boxes.extend(off_guide(m));
        }
    }
    let mut i = 0;
    while i < boxes.len() {
        let mut j = i + 1;
        while j < boxes.len() {
            if is_vertical(&boxes[i]) == is_vertical(&boxes[j]) && touch(&boxes[i], &boxes[j]) {
                let b = boxes.remove(j);
                let a = &mut boxes[i];
                (a.xl, a.yl, a.xh, a.yh) = (a.xl.min(b.xl), a.yl.min(b.yl), a.xh.max(b.xh), a.yh.max(b.yh));
                j = i + 1;
            } else {
                j += 1;
            }
        }
        i += 1;
    }
    boxes
}

/// The tiles' batches: each box bloated by `bloat`, placed in the first batch none of whose boxes
/// it touches, else a new batch.
pub fn tile_batches(boxes: &[Rect], bloat: i32) -> Vec<Vec<usize>> {
    let big: Vec<Rect> = boxes.iter().map(|b| Rect { xl: b.xl - bloat, yl: b.yl - bloat, xh: b.xh + bloat, yh: b.yh + bloat }).collect();
    let mut batches: Vec<Vec<usize>> = Vec::new();
    for (i, b) in big.iter().enumerate() {
        match batches.iter_mut().find(|batch| batch.iter().all(|&k| !touch(b, &big[k]))) {
            Some(batch) => batch.push(i),
            None => batches.push(vec![i]),
        }
    }
    batches
}

/// Stubborn tiles: the markers' gcell boxes merged, each grown to 7×7-gcell route boxes (a
/// centred one first, then four off-centre variants, the boxes with the fewest variants grown
/// first), in design coordinates; and the worker ids in batches whose route boxes, widened by
/// `bloat`, do not touch. Per worker id its distinct route boxes in coordinate order.
pub fn stubborn_boxes(markers: &[Marker], grid: &crate::dr::guides::GCellGrid, bloat: i32) -> (Vec<Vec<Rect>>, Vec<Vec<usize>>) {
    let mut drv: Vec<Rect> = Vec::new();
    for m in markers {
        let (a, b) = (grid.idx((m.bbox.xl, m.bbox.yl)), grid.idx((m.bbox.xh, m.bbox.yh)));
        drv.push(Rect { xl: a.0, yl: a.1, xh: b.0, yh: b.1 });
    }
    let merged = merge_drv_boxes(&drv);
    let expanded = expand_drv_boxes(&merged);
    let coords: Vec<Vec<Rect>> = expanded
        .iter()
        .map(|set| {
            let mut v: Vec<Rect> = set
                .iter()
                .map(|&(xl, yl, xh, yh)| {
                    let (lo, hi) = (grid.gcell_box((xl, yl)), grid.gcell_box((xh, yh)));
                    Rect { xl: lo.xl, yl: lo.yl, xh: hi.xh, yh: hi.yh }
                })
                .collect();
            v.sort_by_key(|r| (r.xl, r.yl, r.xh, r.yh));
            v.dedup();
            v
        })
        .collect();
    let batches = stubborn_batches(&coords, bloat);
    (coords, batches)
}

/// Greedy: the first pair (in order) whose merged box spans at most 4 gcells each way is merged
/// into the first and the second dropped, from the start again, until no pair merges.
pub fn merge_drv_boxes(drv: &[Rect]) -> Vec<Rect> {
    let mut boxes = drv.to_vec();
    'again: loop {
        for i in 0..boxes.len() {
            for j in i + 1..boxes.len() {
                let (a, b) = (boxes[i], boxes[j]);
                let m = Rect { xl: a.xl.min(b.xl), yl: a.yl.min(b.yl), xh: a.xh.max(b.xh), yh: a.yh.max(b.yh) };
                if m.xh - m.xl > 4 || m.yh - m.yl > 4 {
                    continue;
                }
                boxes[i] = m;
                boxes.remove(j);
                continue 'again;
            }
        }
        return boxes;
    }
}

/// The occupancy grid of the growth: per gcell column and row, the box id holding it (-1 none).
struct DrvGrid(Vec<Vec<i32>>);

impl DrvGrid {
    fn has_other(&self, r: &Rect, id: i32) -> bool {
        (r.xl..=r.xh).any(|x| (r.yl..=r.yh).any(|y| {
            let v = self.0[x as usize][y as usize];
            v != -1 && v != id
        }))
    }

    fn fill(&mut self, r: &Rect, id: i32) {
        for x in r.xl..=r.xh {
            for y in r.yl..=r.yh {
                self.0[x as usize][y as usize] = id;
            }
        }
    }

    /// Grow `b` one gcell at a time toward `dir` (0 E, 1 W, 2 N, 3 S), at most `max` times,
    /// stopping at the grid's low edge or where it would take another box's gcell; how far.
    fn expand(&self, b: &mut Rect, id: i32, dir: u8, max: i32) -> i32 {
        let mut r = *b;
        for i in 1..=max {
            match dir {
                0 => r.xh += 1,
                1 => {
                    if r.xl == 0 {
                        return i - 1;
                    }
                    r.xl -= 1;
                }
                2 => r.yh += 1,
                _ => {
                    if r.yl == 0 {
                        return i - 1;
                    }
                    r.yl -= 1;
                }
            }
            if self.has_other(&r, id) {
                return i - 1;
            }
            *b = r;
        }
        max
    }
}

/// Each merged box grown to its route boxes (gcell indices): a centred 7×7 first, then variants
/// with the box at the west, east, south and north (each wave taken fewest-variants first, then
/// by box id, then by its expansions), each variant grown from the merged box. Distinct boxes
/// per merged box, in `(xl, yl, xh, yh)` order.
pub fn expand_drv_boxes(merged: &[Rect]) -> Vec<BTreeSet<(i32, i32, i32, i32)>> {
    use std::cmp::Reverse;
    use std::collections::BinaryHeap;
    let max_x = merged.iter().map(|b| b.xh).max().unwrap_or(0) + 7;
    let max_y = merged.iter().map(|b| b.yh).max().unwrap_or(0) + 7;
    let mut grid = DrvGrid(vec![vec![-1; max_y.max(0) as usize]; max_x.max(0) as usize]);
    for (id, b) in merged.iter().enumerate() {
        grid.fill(b, id as i32);
    }
    let key = |r: &Rect| (r.xl, r.yl, r.xh, r.yh);
    let mut expanded: Vec<BTreeSet<(i32, i32, i32, i32)>> = Vec::new();
    // (expansions done, id, east, west, north, south), smallest first.
    type Wave = (usize, usize, i32, i32, i32, i32);
    let mut waves: BinaryHeap<Reverse<Wave>> = BinaryHeap::new();
    for (id, m) in merged.iter().enumerate() {
        let mut b = *m;
        let h = 6 - (b.xh - b.xl);
        let v = 6 - (b.yh - b.yl);
        let e = grid.expand(&mut b, id as i32, 0, h / 2);
        let w = grid.expand(&mut b, id as i32, 1, h - e);
        let n = grid.expand(&mut b, id as i32, 2, v / 2);
        let s = grid.expand(&mut b, id as i32, 3, v - n);
        waves.push(Reverse((1, id, 0, h, n, s)));
        waves.push(Reverse((1, id, h, 0, n, s)));
        waves.push(Reverse((1, id, e, w, 0, v)));
        waves.push(Reverse((1, id, e, w, v, 0)));
        expanded.push(BTreeSet::from([key(&b)]));
        grid.fill(&b, id as i32);
    }
    while let Some(Reverse(mut wf)) = waves.pop() {
        let id = wf.1;
        if expanded[id].len() != wf.0 {
            wf.0 = expanded[id].len();
            waves.push(Reverse(wf));
            continue;
        }
        let mut b = merged[id];
        grid.expand(&mut b, id as i32, 0, wf.2);
        grid.expand(&mut b, id as i32, 1, wf.3);
        grid.expand(&mut b, id as i32, 2, wf.4);
        grid.expand(&mut b, id as i32, 3, wf.5);
        expanded[id].insert(key(&b));
        grid.fill(&b, id as i32);
    }
    expanded
}

/// Worker ids in batches: each id's boxes widened by `bloat` and merged; first fit into the
/// first batch none of whose members' boxes it touches.
pub fn stubborn_batches(boxes: &[Vec<Rect>], bloat: i32) -> Vec<Vec<usize>> {
    if boxes.is_empty() {
        return Vec::new();
    }
    let max: Vec<Rect> = boxes
        .iter()
        .map(|set| set.iter().map(|r| Rect { xl: r.xl - bloat, yl: r.yl - bloat, xh: r.xh + bloat, yh: r.yh + bloat }).reduce(|a, r| Rect { xl: a.xl.min(r.xl), yl: a.yl.min(r.yl), xh: a.xh.max(r.xh), yh: a.yh.max(r.yh) }).unwrap_or(Rect { xl: i32::MAX, yl: i32::MAX, xh: i32::MIN, yh: i32::MIN }))
        .collect();
    let mut batches: Vec<Vec<usize>> = vec![vec![0]];
    for i in 1..max.len() {
        match batches.iter_mut().find(|b| b.iter().all(|&k| !touch(&max[i], &max[k]))) {
            Some(b) => b.push(i),
            None => batches.push(vec![i]),
        }
    }
    batches
}

#[cfg(test)]
mod tests {
    use super::*;

    // Rule: an incremental run rips up only the nets not routed before in iterations 0–2 where
    // the strategy rips up everything; later rows, and other modes, as the strategy says.
    #[test]
    fn an_incremental_run_rips_up_incrementally_in_its_first_three_iterations() {
        assert_eq!((0..4).map(|i| effective_ripup(RipUp::All, true, i)).collect::<Vec<_>>(), vec![RipUp::Incr, RipUp::Incr, RipUp::Incr, RipUp::All]);
        assert_eq!(effective_ripup(RipUp::All, false, 0), RipUp::All);
        assert_eq!(effective_ripup(RipUp::Drc, true, 1), RipUp::Drc);
    }

    // Rule: the first queue takes, ripping up everything, nets with more than one pin;
    // incrementally, the nets not routed before — a one-pin one too.
    #[test]
    fn the_first_queue_takes_the_nets_the_mode_rips_up() {
        assert!(!first_ripped(RipUp::All, 1, false) && first_ripped(RipUp::All, 2, true));
        assert!(first_ripped(RipUp::Incr, 1, false) && !first_ripped(RipUp::Incr, 5, true));
    }

    // Rule: in an incremental iteration a marker may not rip up a net routed before.
    #[test]
    fn a_net_routed_before_is_pinned_only_incrementally() {
        assert!(ripup_pinned(RipUp::Incr, true));
        assert!(!ripup_pinned(RipUp::Incr, false) && !ripup_pinned(RipUp::Drc, true) && !ripup_pinned(RipUp::All, true));
    }
    use crate::gc::Owner;

    fn r(xl: i32, yl: i32, xh: i32, yh: i32) -> Rect {
        Rect { xl, yl, xh, yh }
    }

    /// Rule: stubborn tiles merge marker boxes greedily — the first pair (in order) whose union
    /// spans at most 4 gcells each way, into the first — until no pair merges.
    #[test]
    fn stubborn_marker_boxes_merge_up_to_four_gcells() {
        let m = merge_drv_boxes(&[r(0, 0, 0, 0), r(10, 10, 10, 10), r(4, 4, 4, 4), r(5, 0, 5, 0)]);
        assert_eq!(m, vec![r(0, 0, 4, 4), r(10, 10, 10, 10), r(5, 0, 5, 0)]);
        // 0..=5 spans 5: never merged.
        assert_eq!(merge_drv_boxes(&[r(0, 0, 0, 0), r(5, 0, 5, 0)]).len(), 2);
    }

    /// Rule: a lone box grows to a centred 7×7 (east first, half the growth; west the rest; then
    /// north half, south the rest), then four variants — the box at the west, east, south and north
    /// edge; at the grid's low edge the growth stops, so fewer distinct boxes remain.
    #[test]
    fn stubborn_boxes_grow_to_seven_gcells_centred_then_off_centre() {
        let e = expand_drv_boxes(&[r(10, 10, 10, 10)]);
        let got: Vec<(i32, i32, i32, i32)> = e[0].iter().copied().collect();
        assert_eq!(got, vec![(4, 7, 10, 13), (7, 4, 13, 10), (7, 7, 13, 13), (7, 10, 13, 16), (10, 7, 16, 13)]);
        let low = expand_drv_boxes(&[r(0, 0, 0, 0)]);
        let got: Vec<(i32, i32, i32, i32)> = low[0].iter().copied().collect();
        // Centred: east 3, west stops at 0, north 3, south stops at 0. Variants from the merged
        // box: west 6 and south 6 stop at once, leaving a box only 3 wide (or high).
        assert_eq!(got, vec![(0, 0, 0, 3), (0, 0, 3, 0), (0, 0, 3, 3), (0, 0, 3, 6), (0, 0, 6, 3)]);
    }

    /// Rule: a box does not grow into another box's gcells.
    #[test]
    fn stubborn_boxes_do_not_grow_into_each_other() {
        let e = expand_drv_boxes(&[r(10, 10, 10, 10), r(12, 10, 12, 10)]);
        for (xl, _, xh, _) in &e[0] {
            assert!(*xh < 12 || *xl > 12, "box 0 took box 1's gcell: {xl}..{xh}");
        }
    }

    /// Rule: worker ids go first-fit into the first batch none of whose members' widened boxes
    /// they touch.
    #[test]
    fn stubborn_batches_first_fit() {
        let b = stubborn_batches(&[vec![r(0, 0, 10, 10)], vec![r(15, 0, 20, 10)], vec![r(100, 0, 110, 10)]], 3);
        assert_eq!(b, vec![vec![0, 2], vec![1]]);
    }

    fn marker(rule: Rule, x: i32) -> Marker {
        let o = Owner::Net("a".into());
        let r = Rect { xl: x, yl: 0, xh: x + 10, yh: 10 };
        Marker { rule, layer: 4, bbox: r, owners: vec![o.clone()], victim: None, aggressor: None }
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

    /// Rule: a worker's starting markers are copies — their sources kept, their sides (victim,
    /// aggressor) not.
    #[test]
    fn starting_markers_are_copies_without_sides() {
        let o = Owner::Net("a".into());
        let r = Rect { xl: 0, yl: 0, xh: 10, yh: 10 };
        let sided = Marker { rule: Rule::Short, layer: 4, bbox: r, owners: vec![o.clone()], victim: Some((o.clone(), 4, r, false)), aggressor: Some((o.clone(), 4, r, false)) };
        let wm = worker_markers(vec![sided], 2);
        assert_eq!(wm.markers[0].owners, vec![o]);
        assert!(wm.markers[0].victim.is_none() && wm.markers[0].aggressor.is_none());
    }

    /// Rule: written back unless, markers driving the queue, the end count exceeds the start, or,
    /// everything ripped up after the first iteration, five times the start.
    #[test]
    fn written_back_when_not_worse() {
        let wm = worker_markers(vec![marker(Rule::Short, 0)], 2);
        assert!(written_back(2, RipUp::Drc, &wm, 1) && !written_back(2, RipUp::Drc, &wm, 2));
        assert!(written_back(2, RipUp::All, &wm, 5) && !written_back(2, RipUp::All, &wm, 6));
        assert!(written_back(0, RipUp::All, &worker_markers(Vec::new(), 0), 99));
        // Incremental: neither cap — written back however many markers it ends with.
        assert!(written_back(2, RipUp::Incr, &wm, 99));
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

    /// Rule: boxes alike in direction that touch merge (the scan restarting); across directions
    /// they stay apart; batches keep bloated boxes apart.
    #[test]
    fn guide_tiles_merge_and_batch() {
        let o = Owner::Net("a".into());
        let g = vec![Rect { xl: 0, yl: 0, xh: 100, yh: 10 }, Rect { xl: 90, yl: 0, xh: 200, yh: 10 }, Rect { xl: 0, yl: 0, xh: 10, yh: 300 }];
        let m = Marker { rule: Rule::Short, layer: 4, bbox: Rect { xl: 5, yl: 5, xh: 95, yh: 6 }, owners: vec![o], victim: None, aggressor: None };
        let boxes = guide_tile_boxes(&[m], &|_| Some(g.clone()), &|_| Vec::new());
        assert_eq!(boxes, vec![Rect { xl: 0, yl: 0, xh: 200, yh: 10 }, Rect { xl: 0, yl: 0, xh: 10, yh: 300 }]);
        assert_eq!(tile_batches(&[Rect { xl: 0, yl: 0, xh: 10, yh: 10 }, Rect { xl: 30, yl: 0, xh: 40, yh: 10 }, Rect { xl: 100, yl: 0, xh: 110, yh: 10 }], 10), vec![vec![0, 2], vec![1]]);
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
