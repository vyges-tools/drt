// SPDX-License-Identifier: Apache-2.0
//! The maze search: A* over a worker's grid from a net's connected points to its next pin.
//!
//! Rules:
//! - the wavefront pops by a total order — lowest cost (path + estimate), then closest to the
//!   pins' centre, upper layer, LARGER path cost (depth first), smaller x, smaller y, then the
//!   remaining state — so its heap's tie order never shows;
//! - a step's cost is its length, plus a bend 1, plus per edge flag its weight times the length
//!   (grid or access-point cost ×2, route shape × the worker's route cost, marker × its marker
//!   cost, fixed shape × its fixed cost, off-guide ×1), a blocked edge 32 × min width × 20, and
//!   the via-to-via and via-to-turn spacing penalties from the rule tables; 32-bit unsigned,
//!   wrapping;
//! - the estimate is the distance to the destination pins' box (layers by their accumulated
//!   height: pitch × 4 per layer) plus one per axis still to turn onto;
//! - each grid remembers its last two moves; the move dropped off that buffer is written as the
//!   arrival direction of the node two back, unless it already has a different one (then the
//!   grid is not pushed). A node with an arrival direction is never expanded again, nor entered;
//! - a found path is traced back from the destination to a source (turning points only, the
//!   source last); every node it passes is added to the connected set.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::dr::cost::Dir6;
use crate::dr::drw::GridGraph;
use crate::dr::rules::{NdrTables, RuleTables};
use crate::tech::Tech;

type P = (i32, i32);
pub type Idx = (i32, i32, i32);

/// A direction's code: unknown 0, D 1, S 2, W 3, E 4, N 5, U 6.
pub fn code(d: Option<Dir6>) -> u8 {
    match d {
        None => 0,
        Some(Dir6::D) => 1,
        Some(Dir6::S) => 2,
        Some(Dir6::W) => 3,
        Some(Dir6::E) => 4,
        Some(Dir6::N) => 5,
        Some(Dir6::U) => 6,
    }
}

pub fn dir_of(c: u8) -> Option<Dir6> {
    match c {
        1 => Some(Dir6::D),
        2 => Some(Dir6::S),
        3 => Some(Dir6::W),
        4 => Some(Dir6::E),
        5 => Some(Dir6::N),
        6 => Some(Dir6::U),
        _ => None,
    }
}

/// The expansion order.
const ALL: [Dir6; 6] = [Dir6::D, Dir6::S, Dir6::W, Dir6::E, Dir6::N, Dir6::U];

fn opposite(d: Dir6) -> Dir6 {
    match d {
        Dir6::D => Dir6::U,
        Dir6::U => Dir6::D,
        Dir6::S => Dir6::N,
        Dir6::N => Dir6::S,
        Dir6::W => Dir6::E,
        Dir6::E => Dir6::W,
    }
}

fn next(i: Idx, d: Option<Dir6>) -> Idx {
    let (x, y, z) = i;
    match d {
        Some(Dir6::E) => (x + 1, y, z),
        Some(Dir6::S) => (x, y - 1, z),
        Some(Dir6::W) => (x - 1, y, z),
        Some(Dir6::N) => (x, y + 1, z),
        Some(Dir6::U) => (x, y, z + 1),
        Some(Dir6::D) => (x, y, z - 1),
        None => i,
    }
}

fn prev(i: Idx, d: Option<Dir6>) -> Idx {
    next(i, d.map(opposite))
}

const MAX: i32 = i32::MAX;
const BUF_BITS: u8 = 0b11_1111;

/// A wavefront entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct WGrid {
    x: i32,
    y: i32,
    z: i32,
    path_cost: u32,
    cost: u32,
    vlen_x: i32,
    vlen_y: i32,
    dist: i32,
    prev_via_up: bool,
    tlen: i32,
    /// The last two moves, three bits each, the latest lowest.
    buf: u8,
    /// The taper box of the source it came from, while inside it (not ordered on).
    src_taper: Option<usize>,
}

impl WGrid {
    fn last_dir(&self) -> Option<Dir6> {
        dir_of(self.buf & 0b111)
    }
    /// Push a move; the one it drops.
    fn shift_add(&mut self, d: Dir6) -> Option<Dir6> {
        let tail = dir_of(self.buf >> 3);
        self.buf = ((self.buf << 3) | code(Some(d))) & BUF_BITS;
        tail
    }
}

impl Ord for WGrid {
    /// Greater pops first.
    fn cmp(&self, b: &Self) -> Ordering {
        b.cost
            .cmp(&self.cost)
            .then(b.dist.cmp(&self.dist))
            .then(self.z.cmp(&b.z))
            .then(self.path_cost.cmp(&b.path_cost))
            .then(b.x.cmp(&self.x))
            .then(b.y.cmp(&self.y))
            .then(code(b.last_dir()).cmp(&code(self.last_dir())))
            .then(b.prev_via_up.cmp(&self.prev_via_up))
            .then(b.tlen.cmp(&self.tlen))
            .then(b.vlen_x.cmp(&self.vlen_x))
            .then(b.vlen_y.cmp(&self.vlen_y))
            .then(b.buf.cmp(&self.buf))
    }
}

impl PartialOrd for WGrid {
    fn partial_cmp(&self, b: &Self) -> Option<Ordering> {
        Some(self.cmp(b))
    }
}

/// What the search reads beyond the grid.
pub struct MazeCfg<'a> {
    pub tech: &'a Tech,
    pub rules: &'a RuleTables,
    /// The worker's costs: route shape, marker, fixed shape.
    pub drc_cost: u32,
    pub marker_cost: u32,
    pub fixed_cost: u32,
    pub iter: i32,
}

const GRID_COST: u32 = 2;
const BLOCK_COST: u32 = 32;
const GUIDE_COST: u32 = 1;
const VIA_COST: i32 = 4;

/// The search's own per-node state, beside the grid's costs: it lives across a net's searches
/// (and its guides across the net), while the grid's costs change between them.
pub struct MazeState {
    pub src: Vec<bool>,
    pub dst: Vec<bool>,
    prev_dir: Vec<u8>,
    pub guide: Vec<bool>,
    z_heights: Vec<i32>,
    pub die: crate::polygon90::Rect,
}

impl MazeState {
    pub fn new(tech: &Tech, g: &GridGraph, die: crate::polygon90::Rect) -> MazeState {
        let n = g.nodes.len();
        let mut h = 0;
        let z_heights = g
            .zs
            .iter()
            .map(|&l| {
                h += tech.layers[l].pitch * VIA_COST;
                h
            })
            .collect();
        MazeState { src: vec![false; n], dst: vec![false; n], prev_dir: vec![0; n], guide: vec![false; n], z_heights, die }
    }
    pub fn reset_status(&mut self) {
        self.src.iter_mut().for_each(|v| *v = false);
        self.dst.iter_mut().for_each(|v| *v = false);
        self.reset_prev_dirs();
    }
    pub fn reset_prev_dirs(&mut self) {
        self.prev_dir.iter_mut().for_each(|v| *v = 0);
    }
    pub fn z_height(&self, z: i32) -> i32 {
        self.z_heights[z as usize]
    }
    pub fn set_src_i(&mut self, g: &GridGraph, i: Idx, on: bool) {
        self.src[g.idx(i.0 as usize, i.1 as usize, i.2 as usize)] = on;
    }
    pub fn set_dst_i(&mut self, g: &GridGraph, i: Idx, on: bool) {
        self.dst[g.idx(i.0 as usize, i.1 as usize, i.2 as usize)] = on;
    }
    /// Whether the edge leaving `i` in `d` exists.
    pub fn has_edge(&self, g: &GridGraph, i: Idx, d: Dir6) -> bool {
        let (c, d) = corrected(i, d);
        if !valid(g, c) {
            return false;
        }
        let n = &g.nodes[g.idx(c.0 as usize, c.1 as usize, c.2 as usize)];
        match d {
            Dir6::E => n.east,
            Dir6::N => n.north,
            _ => n.up,
        }
    }
    /// The length of the edge leaving `i` in `d` (a via: the layers' height difference).
    pub fn edge_len(&self, g: &GridGraph, i: Idx, d: Dir6) -> i32 {
        let (c, d) = corrected(i, d);
        let o = next(c, Some(d));
        // Off the grid (a walk that stepped to the edge measures before it tests the edge).
        if !valid(g, c) || !valid(g, o) {
            return 0;
        }
        let (x, y, z) = (c.0 as usize, c.1 as usize, c.2 as usize);
        match d {
            Dir6::E => g.xs[x + 1] - g.xs[x],
            Dir6::N => g.ys[y + 1] - g.ys[y],
            _ => self.z_heights[z + 1] - self.z_heights[z],
        }
    }
}

/// The edge leaving `i` in `d`, as stored: west, south and down are the neighbour's.
fn corrected(i: Idx, d: Dir6) -> (Idx, Dir6) {
    match d {
        Dir6::W | Dir6::S | Dir6::D => (next(i, Some(d)), opposite(d)),
        _ => (i, d),
    }
}

fn valid(g: &GridGraph, i: Idx) -> bool {
    i.0 >= 0 && i.1 >= 0 && i.2 >= 0 && (i.0 as usize) < g.xs.len() && (i.1 as usize) < g.ys.len() && (i.2 as usize) < g.zs.len()
}

/// A search's view: the grid as it is now, the state, and the net's non-default rule.
pub struct Maze<'a, 'g, 's> {
    pub cfg: &'a MazeCfg<'a>,
    pub g: &'g GridGraph,
    pub st: &'s mut MazeState,
    /// The net's non-default rule tables, while it routes.
    pub ndr: Option<&'a NdrTables>,
    /// The net's non-default rule widths per z, while it routes.
    pub ndr_widths: Option<&'a [i32]>,
    /// The net's taper boxes (a non-default-rule net's instance pins), and which point is in
    /// which (the last pin's box where they overlap).
    pub tapers: &'a [TaperBox],
    pub taper_at: &'a std::collections::HashMap<Idx, usize>,
    /// The destination pin's taper box.
    pub dst_taper: Option<usize>,
}

/// A box of maze indices, z inclusive.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TaperBox {
    pub lo: Idx,
    pub hi: Idx,
}

impl TaperBox {
    pub fn contains(&self, i: Idx) -> bool {
        self.lo.2 <= i.2 && self.hi.2 >= i.2 && self.lo.0 <= i.0 && self.hi.0 >= i.0 && self.lo.1 <= i.1 && self.hi.1 >= i.1
    }
}

impl Maze<'_, '_, '_> {
    fn k(&self, i: Idx) -> usize {
        self.g.idx(i.0 as usize, i.1 as usize, i.2 as usize)
    }
    pub fn point(&self, i: Idx) -> P {
        (self.g.xs[i.0 as usize], self.g.ys[i.1 as usize])
    }
    pub fn is_src(&self, i: Idx) -> bool {
        self.st.src[self.k(i)]
    }
    pub fn is_dst(&self, i: Idx) -> bool {
        self.st.dst[self.k(i)]
    }
    pub fn set_src(&mut self, i: Idx, on: bool) {
        let k = self.k(i);
        self.st.src[k] = on;
    }
    pub fn set_dst(&mut self, i: Idx, on: bool) {
        let k = self.k(i);
        self.st.dst[k] = on;
    }
    fn prev_dir_at(&self, i: Idx) -> Option<Dir6> {
        dir_of(self.st.prev_dir[self.k(i)])
    }
    fn set_prev_dir(&mut self, i: Idx, d: Dir6) {
        let k = self.k(i);
        self.st.prev_dir[k] = code(Some(d));
    }

    /// The edge leaving `i` in `d`, as stored: west, south and down are the neighbour's.
    fn has_grid_cost(&self, i: Idx, d: Dir6) -> bool {
        let (c, d) = corrected(i, d);
        let n = &self.g.nodes[self.k(c)];
        match d {
            Dir6::E => n.grid_cost_e,
            Dir6::N => n.grid_cost_n,
            _ => n.grid_cost_u,
        }
    }
    fn has_ap_cost(&self, i: Idx, d: Dir6) -> bool {
        let (c, d) = corrected(i, d);
        let n = &self.g.nodes[self.k(c)];
        match d {
            Dir6::E => n.ap_cost_e,
            Dir6::N => n.ap_cost_n,
            _ => n.ap_cost_u,
        }
    }
    /// The guide flag of the node the edge leads to (for down, the node below).
    fn has_guide(&self, i: Idx, d: Dir6) -> bool {
        self.st.guide[self.k(next(i, Some(d)))]
    }
    fn route_cost(&self, i: Idx, d: Dir6, ndr: bool) -> bool {
        let n = match d {
            Dir6::U | Dir6::D => {
                let (c, _) = corrected(i, d);
                let n = &self.g.nodes[self.k(c)];
                if ndr {
                    n.route_via.max(n.route_via_ndr)
                } else {
                    n.route_via
                }
            }
            _ => {
                let n = &self.g.nodes[self.k(next(i, Some(d)))];
                if ndr {
                    n.route_planar.max(n.route_planar_ndr)
                } else {
                    n.route_planar
                }
            }
        };
        n != 0
    }
    fn marker_cost(&self, i: Idx, d: Dir6) -> bool {
        let n = match d {
            Dir6::U | Dir6::D => self.g.nodes[self.k(corrected(i, d).0)].marker_via,
            _ => self.g.nodes[self.k(next(i, Some(d)))].marker_planar,
        };
        n != 0
    }
    fn fixed_cost(&self, i: Idx, d: Dir6, ndr: bool) -> bool {
        let v = match d {
            Dir6::U | Dir6::D => {
                let n = &self.g.nodes[self.k(corrected(i, d).0)];
                if n.override_via {
                    0
                } else if ndr {
                    n.fixed_via.max(n.fixed_via_ndr)
                } else {
                    n.fixed_via
                }
            }
            Dir6::E | Dir6::W => {
                let n = &self.g.nodes[self.k(next(i, Some(d)))];
                if ndr {
                    n.fixed_h.max(n.fixed_h_ndr)
                } else {
                    n.fixed_h
                }
            }
            _ => {
                let n = &self.g.nodes[self.k(next(i, Some(d)))];
                if ndr {
                    n.fixed_v.max(n.fixed_v_ndr)
                } else {
                    n.fixed_v
                }
            }
        };
        v != 0
    }
    fn is_blocked(&self, i: Idx, d: Dir6) -> bool {
        let (c, d) = corrected(i, d);
        if !valid(self.g, c) {
            return false;
        }
        let n = &self.g.nodes[self.k(c)];
        match d {
            Dir6::E => n.blocked_e,
            Dir6::N => n.blocked_n,
            _ => n.blocked_u,
        }
    }

    fn via2via_forbidden(&self, z: i32, prev_down: bool, curr_down: bool, dir_x: bool, len: i32) -> bool {
        let k = usize::from(!prev_down) * 4 + usize::from(!curr_down) * 2 + usize::from(!dir_x);
        let r = match self.ndr {
            Some(n) => n.via2via.get(z as usize).map(|t| &t[k]),
            None => self.cfg.rules.layers.get(z as usize).map(|t| &t.via2via[k]),
        };
        r.is_some_and(|r| r.iter().any(|&(lo, hi)| lo <= len && hi >= len))
    }
    fn via2via_prl(&self, z: i32, prev_down: bool, curr_down: bool, dir_x: bool, len: i32) -> bool {
        let k = usize::from(!prev_down) * 4 + usize::from(!curr_down) * 2 + usize::from(!dir_x);
        self.cfg.rules.layers.get(z as usize).is_some_and(|t| len <= t.via2via_prl[k])
    }
    fn turn_forbidden(&self, z: i32, down: bool, dir_x: bool, len: i32) -> bool {
        let k = usize::from(!down) * 2 + usize::from(!dir_x);
        let r = match self.ndr {
            Some(n) => n.turn.get(z as usize).map(|t| &t[k]),
            None => self.cfg.rules.layers.get(z as usize).map(|t| &t.turn[k]),
        };
        r.is_some_and(|r| r.iter().any(|&(lo, hi)| lo <= len && hi >= len))
    }

    /// The estimate from `src` moved one step in `d`: distance to the destination box plus a
    /// bend per axis still to cover that `d` does not run along. (The forbidden-via penalty
    /// applies only without non-preferred tracks or on a one-way layer — neither is modelled.)
    fn est_cost(&self, src: Idx, d1: Idx, d2: Idx, d: Option<Dir6>) -> u32 {
        let n = next(src, d);
        let (sp, p1, p2) = (self.point(n), self.point(d1), self.point(d2));
        let mx = (p1.0 - sp.0).max(sp.0 - p2.0).max(0);
        let my = (p1.1 - sp.1).max(sp.1 - p2.1).max(0);
        let mz = (self.st.z_height(d1.2) - self.st.z_height(n.2)).max(self.st.z_height(n.2) - self.st.z_height(d2.2)).max(0);
        let mut bend = 0;
        let not = |a: Dir6, b: Dir6| d.is_some() && d != Some(a) && d != Some(b);
        if mx != 0 && not(Dir6::E, Dir6::W) {
            bend += 1;
        }
        if my != 0 && not(Dir6::S, Dir6::N) {
            bend += 1;
        }
        if mz != 0 && not(Dir6::U, Dir6::D) {
            bend += 1;
        }
        (mx as u32).wrapping_add(my as u32).wrapping_add(mz as u32).wrapping_add(bend)
    }

    /// A non-default rule's costs apply outside the taper boxes of the source it came from and
    /// of the destination pin.
    fn use_ndr_costs(&self, g: &WGrid) -> bool {
        if self.ndr.is_none() {
            return false;
        }
        let i = (g.x, g.y, g.z);
        if g.src_taper.is_some_and(|t| self.tapers[t].contains(i)) {
            return false;
        }
        !self.dst_taper.is_some_and(|t| self.tapers[t].contains(i))
    }

    fn step_cost(&self, i: Idx, d: Dir6, ndr: bool) -> u32 {
        let len = self.st.edge_len(self.g, i, d) as u32;
        let layer = &self.cfg.tech.layers[self.g.zs[i.2 as usize]];
        let mut c = len;
        if self.has_grid_cost(i, d) || self.has_ap_cost(i, d) {
            c = c.wrapping_add(GRID_COST.wrapping_mul(len));
        }
        if self.route_cost(i, d, ndr) {
            c = c.wrapping_add(self.cfg.drc_cost.wrapping_mul(len));
        }
        if self.marker_cost(i, d) {
            c = c.wrapping_add(self.cfg.marker_cost.wrapping_mul(len));
        }
        if self.fixed_cost(i, d, ndr) {
            c = c.wrapping_add(self.cfg.fixed_cost.wrapping_mul(len));
        }
        if self.is_blocked(i, d) {
            c = c.wrapping_add(BLOCK_COST.wrapping_mul(layer.min_width as u32).wrapping_mul(20));
        }
        if !self.has_guide(i, d) {
            c = c.wrapping_add(GUIDE_COST.wrapping_mul(len));
        }
        c
    }

    fn next_path_cost(&self, g: &WGrid, d: Dir6) -> u32 {
        let i = (g.x, g.y, g.z);
        let mut pc = g.path_cost;
        let len = self.st.edge_len(self.g, i, d) as u32;
        let curr = g.last_dir();
        if curr != Some(d) && curr.is_some() {
            pc = pc.wrapping_add(1);
        }
        let late = self.cfg.iter >= 3;
        if matches!(d, Dir6::U | Dir6::D) {
            let up = d == Dir6::U;
            let (vx, vy) = (g.vlen_x, g.vlen_y);
            let pd = !g.prev_via_up;
            let forbidden = if vx == 0 && vy > 0 {
                self.via2via_forbidden(g.z, pd, !up, false, vy)
            } else if vx > 0 && vy == 0 {
                self.via2via_forbidden(g.z, pd, !up, true, vx)
            } else if self.via2via_prl(g.z, pd, !up, false, vy) || self.via2via_prl(g.z, pd, !up, true, vx) {
                self.via2via_forbidden(g.z, pd, !up, false, vy) || self.via2via_forbidden(g.z, pd, !up, true, vx)
            } else {
                self.via2via_forbidden(g.z, pd, !up, false, vy) && self.via2via_forbidden(g.z, pd, !up, true, vx)
            };
            if forbidden {
                let w = if late { self.cfg.marker_cost } else { self.cfg.drc_cost };
                pc = pc.wrapping_add(2u32.wrapping_mul(w).wrapping_mul(len));
            }
        }
        if let Some(cd) = curr {
            if cd != d {
                let along_x = matches!(cd, Dir6::W | Dir6::E);
                let along_y = matches!(cd, Dir6::S | Dir6::N);
                let mut forbidden = false;
                if matches!(d, Dir6::U | Dir6::D) {
                    let up = d == Dir6::U;
                    if along_x || along_y {
                        forbidden = self.turn_forbidden(g.z, !up, along_x, g.tlen);
                    }
                } else {
                    let up = g.prev_via_up;
                    if along_x {
                        forbidden = self.turn_forbidden(g.z, !up, true, g.vlen_x);
                    } else if along_y {
                        forbidden = self.turn_forbidden(g.z, !up, false, g.vlen_y);
                    }
                }
                if forbidden {
                    // (The weights are the other way round from the via-to-via penalty.)
                    let w = if late { self.cfg.drc_cost } else { self.cfg.marker_cost };
                    pc = pc.wrapping_add(2u32.wrapping_mul(w).wrapping_mul(len));
                }
            }
        }
        pc.wrapping_add(self.step_cost(i, d, self.use_ndr_costs(g)))
    }

    fn is_expandable(&self, g: &WGrid, d: Dir6) -> bool {
        let i = (g.x, g.y, g.z);
        if !self.st.has_edge(self.g, i, d) {
            return false;
        }
        let n = next(i, Some(d));
        if self.is_src(n) || self.prev_dir_at(n).is_some() || g.last_dir() == Some(opposite(d)) {
            return false;
        }
        if let (Some(w), true) = (self.ndr_widths, self.ndr.is_some()) {
            let lw = self.cfg.tech.layers[self.g.zs[g.z as usize]].width;
            let mut half = lw / 2;
            let nw = w.get(g.z as usize).copied().unwrap_or(0);
            if nw > 2 * half && !self.is_src(i) {
                half = nw / 2;
                let (x, y) = self.point(i);
                match d {
                    Dir6::N | Dir6::S if x - half < self.st.die.xl || x + half > self.st.die.xh => return false,
                    Dir6::E | Dir6::W if y - half < self.st.die.yl || y + half > self.st.die.yh => return false,
                    _ => {}
                }
            }
        }
        true
    }

    fn tail_idx(n: Idx, buf: u8) -> Idx {
        let mut i = n;
        let mut b = buf;
        for _ in 0..2 {
            i = prev(i, dir_of(b & 0b111));
            b >>= 3;
        }
        i
    }

    fn expand(&mut self, heap: &mut BinaryHeap<WGrid>, g: &WGrid, d: Dir6, d1: Idx, d2: Idx, center: P) {
        let i = (g.x, g.y, g.z);
        let n = next(i, Some(d));
        let est = self.est_cost(i, d1, d2, Some(d));
        let pc = self.next_path_cost(g, d);
        let np = self.point(n);
        let dist = (np.0 - center.0).abs() + (np.1 - center.1).abs();
        let len = self.st.edge_len(self.g, i, d);
        let via = matches!(d, Dir6::U | Dir6::D);
        let (mut vx, mut vy) = (g.vlen_x, g.vlen_y);
        let mut via_up = g.prev_via_up;
        if via {
            vx = 0;
            vy = 0;
            via_up = d == Dir6::D;
        } else if vx != MAX && vy != MAX {
            if matches!(d, Dir6::W | Dir6::E) {
                vx += len;
            } else {
                vy += len;
            }
        }
        let mut tl = g.tlen;
        if tl != MAX {
            tl += len;
        }
        if g.last_dir().is_some() && g.last_dir() != Some(d) {
            tl = len;
        }
        if via {
            tl = MAX;
        }
        let mut ng = WGrid { x: n.0, y: n.1, z: n.2, path_cost: pc, cost: pc.wrapping_add(est), vlen_x: vx, vlen_y: vy, dist, prev_via_up: via_up, tlen: tl, buf: g.buf, src_taper: None };
        if g.src_taper.is_some_and(|t| self.tapers[t].contains(n)) {
            ng.src_taper = g.src_taper;
        }
        if via {
            ng.vlen_x = 0;
            ng.vlen_y = 0;
            ng.prev_via_up = d != Dir6::U;
        }
        let tail_dir = ng.shift_add(d);
        let tail = Self::tail_idx(n, ng.buf);
        if let Some(td) = tail_dir {
            let pd = self.prev_dir_at(tail);
            if pd.is_none() || pd == Some(td) {
                self.set_prev_dir(tail, td);
                heap.push(ng);
            }
        } else {
            heap.push(ng);
        }
    }

    fn trace_back(&self, g: &WGrid, path: &mut Vec<Idx>, root: &mut Vec<Idx>, cc1: &mut Idx, cc2: &mut Idx) {
        let mut prev_d: Option<Dir6> = None;
        let mut c = (g.x, g.y, g.z);
        let mut b = g.buf;
        for _ in 0..2 {
            if self.is_src(c) {
                break;
            }
            let d = dir_of(b & 0b111);
            b >>= 3;
            if d.is_none() {
                break;
            }
            root.push(c);
            if d != prev_d {
                path.push(c);
            }
            c = prev(c, d);
            prev_d = d;
        }
        while !self.is_src(c) {
            let d = self.prev_dir_at(c);
            root.push(c);
            if d.is_none() {
                break;
            }
            if d != prev_d {
                path.push(c);
            }
            c = prev(c, d);
            prev_d = d;
        }
        if !path.is_empty() {
            path.push(c);
        }
        for m in path.iter() {
            *cc1 = (cc1.0.min(m.0), cc1.1.min(m.1), cc1.2.min(m.2));
            *cc2 = (cc2.0.max(m.0), cc2.1.max(m.1), cc2.2.max(m.2));
        }
    }

    /// Search from the connected points to the destination pin's access points: the path, dst
    /// first (a single point when a connected point is already a destination).
    pub fn search(&mut self, conn: &mut Vec<Idx>, dst_aps: &[Idx], path: &mut Vec<Idx>, cc1: &mut Idx, cc2: &mut Idx, center: P) -> bool {
        let (xd, yd, zd) = (self.g.xs.len() as i32, self.g.ys.len() as i32, self.g.zs.len() as i32);
        let mut d1 = (xd - 1, yd - 1, zd - 1);
        let mut d2 = (0, 0, 0);
        for m in dst_aps {
            d1 = (d1.0.min(m.0), d1.1.min(m.1), d1.2.min(m.2));
            d2 = (d2.0.max(m.0), d2.1.max(m.1), d2.2.max(m.2));
        }
        let mut heap = BinaryHeap::new();
        for &i in conn.iter() {
            if self.is_dst(i) {
                path.push(i);
                return true;
            }
            let p = self.point(i);
            let dist = (p.0 - center.0).abs() + (p.1 - center.1).abs();
            let src_taper = if self.ndr.is_some() { self.taper_at.get(&i).copied() } else { None };
            heap.push(WGrid { x: i.0, y: i.1, z: i.2, path_cost: 0, cost: self.est_cost(i, d1, d2, None), vlen_x: MAX, vlen_y: MAX, dist, prev_via_up: true, tlen: MAX, buf: 0, src_taper });
        }
        while let Some(g) = heap.pop() {
            let i = (g.x, g.y, g.z);
            if self.prev_dir_at(i).is_some() {
                continue;
            }
            if self.is_dst(i) {
                self.trace_back(&g, path, conn, cc1, cc2);
                return true;
            }
            for d in ALL {
                if self.is_expandable(&g, d) {
                    self.expand(&mut heap, &g, d, d1, d2, center);
                }
            }
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wg(cost: u32, dist: i32, z: i32, path_cost: u32, x: i32) -> WGrid {
        WGrid { x, y: 0, z, path_cost, cost, vlen_x: 0, vlen_y: 0, dist, prev_via_up: false, tlen: 0, buf: 0, src_taper: None }
    }

    // Rule: the wavefront pops lowest cost, then nearest the centre, then the UPPER layer, then
    // the LARGER path cost, then the smaller x.
    #[test]
    fn the_wavefront_pops_by_the_total_order() {
        let mut h = BinaryHeap::new();
        for g in [wg(10, 5, 0, 3, 0), wg(9, 9, 0, 0, 0), wg(10, 4, 0, 3, 0), wg(10, 4, 1, 3, 0), wg(10, 4, 1, 4, 0), wg(10, 4, 1, 4, 2), wg(10, 4, 1, 4, 1)] {
            h.push(g);
        }
        let order: Vec<(u32, i32, i32, u32, i32)> = std::iter::from_fn(|| h.pop()).map(|g| (g.cost, g.dist, g.z, g.path_cost, g.x)).collect();
        assert_eq!(order, vec![(9, 9, 0, 0, 0), (10, 4, 1, 4, 0), (10, 4, 1, 4, 1), (10, 4, 1, 4, 2), (10, 4, 1, 3, 0), (10, 4, 0, 3, 0), (10, 5, 0, 3, 0)]);
    }

    // Rule: the move buffer holds two moves; pushing a third returns the first.
    #[test]
    fn the_move_buffer_drops_the_third_move_back() {
        let mut g = wg(0, 0, 0, 0, 0);
        assert_eq!(g.shift_add(Dir6::E), None);
        assert_eq!(g.shift_add(Dir6::N), None);
        assert_eq!(g.shift_add(Dir6::U), Some(Dir6::E));
        assert_eq!(g.last_dir(), Some(Dir6::U));
    }
}
