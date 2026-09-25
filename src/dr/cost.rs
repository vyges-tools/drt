// SPDX-License-Identifier: Apache-2.0
//! A worker's grid costs: which edges are blocked and how much each node costs to route through
//! because of fixed shapes (pins, special-net wiring, blockages) — before any net is routed.
//!
//! Stages, in order ([`init_maze_cost`]): the fixed shapes, layer by layer (blockages first,
//! then pins, special-net wires and vias; then each net's pin shapes as a group); each access
//! point's own edges (blocked unless it may leave that way; a special via where it has a via);
//! each net's end-of-line route costs from its merged pin shapes; block pins' planar access (blocking the directions it may not use).
//!
//! Rules:
//! - a cost is an 8-bit count: adding saturates at 255, subtracting floors at 0;
//! - a node is costed for a shape when the wire (or default via) centred on it would be closer to
//!   the shape than the spacing the layer requires (for that width and parallel run; for a
//!   blockage, the table's smallest), squared-distance tested from the wire's corners;
//! - west, south and down are the neighbour's east, north and up.

use std::collections::{BTreeMap, BTreeSet};

use crate::dr::drw::{DrAp, DrNet, GridGraph};
use crate::dr::rules::{EolTable, NdrRule};
use crate::dr::ta::Fixed;
use crate::polygon90::{Edge, Polygon90Set, Rect};
use crate::rtree::PackedRTree;
use crate::tech::{LayerKind, Tech};

type P = (i32, i32);
/// A net's non-default rule, with its end-of-line rule per z.
pub type Ndr<'a> = (&'a NdrRule, &'a [Option<EolTable>]);
/// A non-default rule as a via's spacing reads it: width, spacing, preferred via up and down, its
/// end-of-line rule.
type NdrVia = (i32, i32, Option<usize>, Option<usize>, EolTable);
/// A net's terminal, ordered block pins first: kind, owner, terminal, record.
type TermKey = (u8, usize, usize, Fixed);

/// The six directions, in the router's order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dir6 {
    D,
    S,
    W,
    E,
    N,
    U,
}

pub const ALL: [Dir6; 6] = [Dir6::D, Dir6::S, Dir6::W, Dir6::E, Dir6::N, Dir6::U];

impl DrAp {
    /// Whether the access point may leave in `d`.
    pub fn has(&self, d: Dir6) -> bool {
        // Bits: north, south, east, west, up, down.
        let bit = match d {
            Dir6::N => 0,
            Dir6::S => 1,
            Dir6::E => 2,
            Dir6::W => 3,
            Dir6::U => 4,
            Dir6::D => 5,
        };
        self.access & (1 << bit) != 0
    }
}

/// How a cost is changed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModCost {
    AddFixed,
    SubFixed,
    AddRoute,
    SubRoute,
    SetFixed,
    ResetFixed,
    SetBlocked,
    ResetBlocked,
}

fn add8(v: &mut u8) {
    *v = v.saturating_add(1);
}
fn sub8(v: &mut u8) {
    *v = v.saturating_sub(1);
}

fn pt_box_d2(p: P, b: &Rect) -> i64 {
    let dx = i64::from((b.xl - p.0).max(p.0 - b.xh).max(0));
    let dy = i64::from((b.yl - p.1).max(p.1 - b.yh).max(0));
    dx * dx + dy * dy
}

/// Squared distance, and the gaps along x and y (negative: overlap).
fn box_box_d2(a: &Rect, b: &Rect) -> (i64, i32, i32) {
    let dx = a.xl.max(b.xl) - a.xh.min(b.xh);
    let dy = a.yl.max(b.yl) - a.yh.min(b.yh);
    let (cx, cy) = (i64::from(dx.max(0)), i64::from(dy.max(0)));
    (cx * cx + cy * cy, dx, dy)
}

fn overlaps(a: &Rect, b: &Rect) -> bool {
    a.xl < b.xh && b.xl < a.xh && a.yl < b.yh && b.yl < a.yh
}

fn shift(r: &Rect, p: P) -> Rect {
    Rect { xl: r.xl + p.0, yl: r.yl + p.1, xh: r.xh + p.0, yh: r.yh + p.1 }
}

fn min_side(r: &Rect) -> i32 {
    r.dx().min(r.dy())
}
fn max_side(r: &Rect) -> i32 {
    r.dx().max(r.dy())
}

impl GridGraph {
    /// The z of a routing layer.
    pub fn z_of(&self, layer: usize) -> Option<usize> {
        self.zs.iter().position(|&l| l == layer)
    }

    /// The index range covering a box: low from the first coordinate at or above the box's low
    /// side, high from the last at or below its high side (the high clamped at 0).
    pub fn idx_box(&self, b: &Rect) -> (usize, usize, usize, usize) {
        let x1 = self.xs.partition_point(|&c| c < b.xl);
        let y1 = self.ys.partition_point(|&c| c < b.yl);
        let x2 = self.xs.partition_point(|&c| c <= b.xh).saturating_sub(1);
        let y2 = self.ys.partition_point(|&c| c <= b.yh).saturating_sub(1);
        (x1, y1, x2, y2)
    }

    /// The index range ENCLOSING a box: as [`GridGraph::idx_box`], the low end stepped back one
    /// when its coordinate lies past the box's low side.
    pub fn idx_box_enclose(&self, b: &Rect) -> (usize, usize, usize, usize) {
        let (mut x1, mut y1, x2, y2) = self.idx_box(b);
        if self.xs.get(x1).is_some_and(|&c| c > b.xl) {
            x1 = x1.saturating_sub(1);
        }
        if self.ys.get(y1).is_some_and(|&c| c > b.yl) {
            y1 = y1.saturating_sub(1);
        }
        (x1, y1, x2, y2)
    }

    fn corrected(&self, x: i64, y: i64, z: i64, d: Dir6) -> (i64, i64, i64, Dir6) {
        match d {
            Dir6::W => (x - 1, y, z, Dir6::E),
            Dir6::S => (x, y - 1, z, Dir6::N),
            Dir6::D => (x, y, z - 1, Dir6::U),
            _ => (x, y, z, d),
        }
    }

    fn valid(&self, x: i64, y: i64, z: i64) -> bool {
        x >= 0 && y >= 0 && z >= 0 && (x as usize) < self.xs.len() && (y as usize) < self.ys.len() && (z as usize) < self.zs.len()
    }

    pub fn set_blocked(&mut self, x: i64, y: i64, z: i64, d: Dir6, on: bool) {
        let (x, y, z, d) = self.corrected(x, y, z, d);
        if !self.valid(x, y, z) {
            return;
        }
        let k = self.idx(x as usize, y as usize, z as usize);
        match d {
            Dir6::E => self.nodes[k].blocked_e = on,
            Dir6::N => self.nodes[k].blocked_n = on,
            Dir6::U => self.nodes[k].blocked_u = on,
            _ => {}
        }
    }

    pub fn is_blocked(&self, x: usize, y: usize, z: usize, d: Dir6) -> bool {
        let (x, y, z, d) = self.corrected(x as i64, y as i64, z as i64, d);
        if !self.valid(x, y, z) {
            return false;
        }
        let n = &self.nodes[self.idx(x as usize, y as usize, z as usize)];
        match d {
            Dir6::E => n.blocked_e,
            Dir6::N => n.blocked_n,
            Dir6::U => n.blocked_u,
            _ => false,
        }
    }

    /// The fixed-shape cost of the edge leaving (x, y, z) in `d` (the adjacent node's planar cost;
    /// the node's via cost unless overridden). Not across a non-default rule.
    pub fn fixed_cost_adj(&self, x: usize, y: usize, z: usize, d: Dir6) -> u32 {
        match d {
            Dir6::E => u32::from(self.nodes[self.idx(x + 1, y, z)].fixed_h),
            Dir6::W => u32::from(self.nodes[self.idx(x - 1, y, z)].fixed_h),
            Dir6::N => u32::from(self.nodes[self.idx(x, y + 1, z)].fixed_v),
            Dir6::S => u32::from(self.nodes[self.idx(x, y - 1, z)].fixed_v),
            Dir6::U | Dir6::D => {
                let z = if d == Dir6::D { z - 1 } else { z };
                let n = &self.nodes[self.idx(x, y, z)];
                if n.override_via {
                    0
                } else {
                    u32::from(n.fixed_via)
                }
            }
        }
    }

    pub fn route_cost_adj(&self, x: usize, y: usize, z: usize, d: Dir6) -> u32 {
        match d {
            Dir6::E => u32::from(self.nodes[self.idx(x + 1, y, z)].route_planar),
            Dir6::W => u32::from(self.nodes[self.idx(x - 1, y, z)].route_planar),
            Dir6::N => u32::from(self.nodes[self.idx(x, y + 1, z)].route_planar),
            Dir6::S => u32::from(self.nodes[self.idx(x, y - 1, z)].route_planar),
            Dir6::U => u32::from(self.nodes[self.idx(x, y, z)].route_via),
            Dir6::D => u32::from(self.nodes[self.idx(x, y, z - 1)].route_via),
        }
    }

    pub fn marker_cost_adj(&self, x: usize, y: usize, z: usize, d: Dir6) -> u32 {
        match d {
            Dir6::E => u32::from(self.nodes[self.idx(x + 1, y, z)].marker_planar),
            Dir6::W => u32::from(self.nodes[self.idx(x - 1, y, z)].marker_planar),
            Dir6::N => u32::from(self.nodes[self.idx(x, y + 1, z)].marker_planar),
            Dir6::S => u32::from(self.nodes[self.idx(x, y - 1, z)].marker_planar),
            Dir6::U => u32::from(self.nodes[self.idx(x, y, z)].marker_via),
            Dir6::D => u32::from(self.nodes[self.idx(x, y, z - 1)].marker_via),
        }
    }

    fn mod_planar(&mut self, k: usize, t: ModCost, ndr: bool, reset_h: bool, reset_v: bool) {
        let n = &mut self.nodes[k];
        match (t, ndr) {
            (ModCost::AddRoute, false) => add8(&mut n.route_planar),
            (ModCost::AddRoute, true) => add8(&mut n.route_planar_ndr),
            (ModCost::SubRoute, false) => sub8(&mut n.route_planar),
            (ModCost::SubRoute, true) => sub8(&mut n.route_planar_ndr),
            (ModCost::AddFixed, false) => {
                add8(&mut n.fixed_h);
                add8(&mut n.fixed_v);
            }
            (ModCost::AddFixed, true) => {
                add8(&mut n.fixed_h_ndr);
                add8(&mut n.fixed_v_ndr);
            }
            (ModCost::SubFixed, false) => {
                sub8(&mut n.fixed_h);
                sub8(&mut n.fixed_v);
            }
            (ModCost::SubFixed, true) => {
                sub8(&mut n.fixed_h_ndr);
                sub8(&mut n.fixed_v_ndr);
            }
            (ModCost::ResetFixed, false) => {
                if reset_h {
                    n.fixed_h = 0;
                }
                if reset_v {
                    n.fixed_v = 0;
                }
            }
            (ModCost::ResetFixed, true) => {
                if reset_h {
                    n.fixed_h_ndr = 0;
                }
                if reset_v {
                    n.fixed_v_ndr = 0;
                }
            }
            (ModCost::SetFixed, false) => {
                n.fixed_h = 1;
                n.fixed_v = 1;
            }
            (ModCost::SetFixed, true) => {
                n.fixed_h_ndr = 1;
                n.fixed_v_ndr = 1;
            }
            _ => {}
        }
    }

    fn mod_via(&mut self, k: usize, t: ModCost, ndr: bool) {
        let n = &mut self.nodes[k];
        match (t, ndr) {
            (ModCost::AddRoute, false) => add8(&mut n.route_via),
            (ModCost::AddRoute, true) => add8(&mut n.route_via_ndr),
            (ModCost::SubRoute, false) => sub8(&mut n.route_via),
            (ModCost::SubRoute, true) => sub8(&mut n.route_via_ndr),
            (ModCost::AddFixed, false) => add8(&mut n.fixed_via),
            (ModCost::AddFixed, true) => add8(&mut n.fixed_via_ndr),
            (ModCost::SubFixed, false) => sub8(&mut n.fixed_via),
            (ModCost::SubFixed, true) => sub8(&mut n.fixed_via_ndr),
            _ => {}
        }
    }
}

/// What a worker's costs read.
pub struct CostCtx<'a> {
    pub tech: &'a Tech,
    pub defaults: &'a [Option<usize>],
    /// The router's end-of-line rule, per routing-layer index.
    pub eol: &'a [EolTable],
    /// The non-default rules of the worker's nets — one entry PER NET (so a rule shared by two
    /// nets counts twice), with their end-of-line rules per routing-layer index.
    pub ndrs: Vec<(&'a NdrRule, Vec<Option<EolTable>>)>,
    pub use_min_spacing_obs: bool,
    /// Per routing-layer index, the via-through table: below/above (`k / 2`), along x/y (`k % 2`).
    pub through: &'a [[bool; 4]],
    /// The via-access layer (a layer number; 2 unless set): instance pins at or below half of it,
    /// less one, as a grid z, also cost planar spacing as a blockage does.
    pub via_access_layer: usize,
    /// Per layer, the fixed-shape tree.
    pub fixed: &'a [PackedRTree<Fixed>],
    /// A terminal's pin shapes (design coordinates), by its fixed-shape record.
    pub term_shapes: &'a dyn Fn(&Fixed) -> Vec<(usize, Rect)>,
    /// A port's access points (all its pins).
    pub port_aps: &'a dyn Fn(usize) -> Vec<DrAp>,
    /// Whether an instance's master is a block.
    pub inst_is_block: &'a dyn Fn(usize) -> bool,
}

impl CostCtx<'_> {
    fn width(&self, l: usize) -> i32 {
        self.tech.layers[l].width
    }
    fn default_via(&self, cut: i64) -> Option<usize> {
        usize::try_from(cut).ok().and_then(|c| self.defaults.get(c)).copied().flatten()
    }
    fn eol_of(&self, l: usize) -> EolTable {
        let idx = (0..l).filter(|&k| self.tech.layers[k].kind == LayerKind::Routing).count();
        self.eol.get(idx).copied().unwrap_or_default()
    }
    /// The minimum spacing for two widths and a run (the table's smallest when asked; 0 without
    /// a rule).
    fn min_spacing_value(&self, l: usize, w1: i32, w2: i32, prl: i32, use_min: bool) -> i32 {
        match &self.tech.layers[l].spacing {
            None => 0,
            Some(t) if use_min => t.find_min(),
            Some(t) => t.find(w1.max(w2), prl),
        }
    }
}

/// A worker's grid, costed.
pub struct CostWorker<'a, 'b> {
    pub cx: &'b CostCtx<'a>,
    pub g: &'b mut GridGraph,
    /// The special via at each node that has one: the access point's via choices, best first.
    pub ap_svia: BTreeMap<(usize, usize, usize), usize>,
}

impl CostWorker<'_, '_> {
    fn range(&self, b: &Rect) -> Option<(usize, usize, usize, usize)> {
        let (x1, y1, x2, y2) = self.g.idx_box(b);
        (x1 <= x2 && y1 <= y2).then_some((x1, y1, x2, y2))
    }

    pub fn mod_blocked_planar(&mut self, b: &Rect, z: usize, set: bool) {
        let Some((x1, y1, x2, y2)) = self.range(b) else { return };
        for i in x1..=x2 {
            for j in y1..=y2 {
                for d in [Dir6::E, Dir6::N, Dir6::W, Dir6::S] {
                    self.g.set_blocked(i as i64, j as i64, z as i64, d, set);
                }
            }
        }
    }

    pub fn mod_blocked_via(&mut self, b: &Rect, z: usize, set: bool) {
        let Some((x1, y1, x2, y2)) = self.range(b) else { return };
        for i in x1..=x2 {
            for j in y1..=y2 {
                self.g.set_blocked(i as i64, j as i64, z as i64, Dir6::U, set);
                self.g.set_blocked(i as i64, j as i64, z as i64, Dir6::D, set);
            }
        }
    }

    /// Planar spacing costs around a shape: once for the default wire, then once per
    /// non-default rule of the worker's nets.
    #[allow(clippy::too_many_arguments)]
    pub fn mod_min_spacing_cost_planar(&mut self, b: &Rect, z: usize, t: ModCost, is_blockage: bool, ndr: Option<Ndr<'_>>, is_macro_pin: bool, reset_h: bool, reset_v: bool) {
        let l = self.g.zs[z];
        let w = self.cx.width(l);
        let dsp = ndr.map_or(0, |(n, _)| n.spacings.get(z).copied().unwrap_or(0));
        self.mod_min_spacing_cost_planar_helper(b, z, t, w, dsp, is_blockage, is_macro_pin, reset_h, reset_v, false);
        let ndrs: Vec<(i32, i32)> = self.cx.ndrs.iter().map(|(n, _)| (n.widths.get(z).copied().unwrap_or(0), n.spacings.get(z).copied().unwrap_or(0))).collect();
        for (nw, ns) in ndrs {
            let width = if nw != 0 { nw } else { w };
            self.mod_min_spacing_cost_planar_helper(b, z, t, width, dsp.max(ns), is_blockage, is_macro_pin, reset_h, reset_v, true);
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn mod_min_spacing_cost_planar_helper(&mut self, b: &Rect, z: usize, t: ModCost, width2: i32, min_spacing: i32, is_blockage: bool, is_macro_pin: bool, reset_h: bool, reset_v: bool, ndr: bool) {
        let l = self.g.zs[z];
        let w1 = min_side(b);
        let hw2 = width2 / 2;
        let use_min = is_blockage && self.cx.use_min_spacing_obs;
        let d = self.cx.min_spacing_value(l, w1, width2, max_side(b), use_min).max(min_spacing);
        let d2 = i64::from(d) * i64::from(d);
        let bx = Rect { xl: b.xl - d - hw2 + 1, yl: b.yl - d - hw2 + 1, xh: b.xh + d + hw2 - 1, yh: b.yh + d + hw2 - 1 };
        let Some((x1, y1, x2, y2)) = self.range(&bx) else { return };
        let pin = if is_macro_pin && t == ModCost::ResetBlocked {
            let s = Rect { xl: b.xl + width2 / 2, yl: b.yl + width2 / 2, xh: b.xh - width2 / 2, yh: b.yh - width2 / 2 };
            let (a, c, e, f) = self.g.idx_box(&s);
            Some((a, c, e, f))
        } else {
            None
        };
        for i in x1..=x2 {
            for j in y1..=y2 {
                let p = (self.g.xs[i], self.g.ys[j]);
                let corners = [(p.0 + hw2, p.1 - hw2), (p.0 + hw2, p.1 + hw2), (p.0 - hw2, p.1 - hw2), (p.0 - hw2, p.1 + hw2)];
                let dist = corners.iter().map(|&c| pt_box_d2(c, b)).min().expect("four corners");
                if dist >= d2 {
                    continue;
                }
                let k = self.g.idx(i, j, z);
                match t {
                    ModCost::ResetBlocked => {
                        if ndr {
                            return;
                        }
                        let (ii, jj, zz) = (i as i64, j as i64, z as i64);
                        if let Some((px1, py1, px2, py2)) = pin {
                            if j >= py1 && j <= py2 {
                                self.g.set_blocked(ii, jj, zz, Dir6::E, false);
                                if i == 0 {
                                    self.g.set_blocked(ii, jj, zz, Dir6::W, false);
                                }
                            }
                            if i >= px1 && i <= px2 {
                                self.g.set_blocked(ii, jj, zz, Dir6::N, false);
                                if j == 0 {
                                    self.g.set_blocked(ii, jj, zz, Dir6::S, false);
                                }
                            }
                        } else {
                            self.g.set_blocked(ii, jj, zz, Dir6::E, false);
                            self.g.set_blocked(ii, jj, zz, Dir6::N, false);
                            if i == 0 {
                                self.g.set_blocked(ii, jj, zz, Dir6::W, false);
                            }
                            if j == 0 {
                                self.g.set_blocked(ii, jj, zz, Dir6::S, false);
                            }
                        }
                    }
                    ModCost::SetBlocked => {
                        if ndr {
                            return;
                        }
                        let (ii, jj, zz) = (i as i64, j as i64, z as i64);
                        self.g.set_blocked(ii, jj, zz, Dir6::E, true);
                        self.g.set_blocked(ii, jj, zz, Dir6::N, true);
                        if i == 0 {
                            self.g.set_blocked(ii, jj, zz, Dir6::W, true);
                        }
                        if j == 0 {
                            self.g.set_blocked(ii, jj, zz, Dir6::S, true);
                        }
                    }
                    _ => self.g.mod_planar(k, t, ndr, reset_h, reset_v),
                }
            }
        }
    }

    /// Via spacing costs around a shape: for the via below or above (the layer's default, or a
    /// non-default rule's preferred), at each node where it would sit too close.
    #[allow(clippy::too_many_arguments)]
    pub fn mod_min_spacing_cost_via(&mut self, b: &Rect, z: usize, t: ModCost, upper: bool, curr_ps: bool, is_blockage: bool, ndr: Option<Ndr<'_>>) {
        let l = self.g.zs[z];
        let (min_l, max_l) = (self.g.zs[0], *self.g.zs.last().expect("a layer"));
        let default_via = if upper {
            if l < max_l { self.cx.default_via(l as i64 + 1) } else { None }
        } else if l > min_l {
            self.cx.default_via(l as i64 - 1)
        } else {
            None
        };
        let dsp = ndr.map_or(0, |(n, _)| n.spacings.get(z).copied().unwrap_or(0));
        // The net's own rule's end of line (none: the layer's, chosen in the helper).
        let dcon = ndr.and_then(|(_, e)| e.get(z).copied().flatten()).unwrap_or_default();
        let w = self.cx.width(l);
        self.mod_min_spacing_cost_via_helper(b, z, t, w, dsp, default_via, dcon, upper, curr_ps, is_blockage, false);
        let ndrs: Vec<NdrVia> = self
            .cx
            .ndrs
            .iter()
            .map(|(n, e)| {
                let pref = |zz: i64| usize::try_from(zz).ok().and_then(|zz| n.vias.get(zz)).and_then(|v| v.first()).copied();
                (n.widths.get(z).copied().unwrap_or(0), n.spacings.get(z).copied().unwrap_or(0), pref(z as i64), pref(z as i64 - 1), e.get(z).copied().flatten().unwrap_or_default())
            })
            .collect();
        for (nw, ns, pref_up, pref_down, econ) in ndrs {
            let mut via = default_via;
            if upper && l < max_l && pref_up.is_some() {
                via = pref_up;
            } else if !upper && l > min_l && pref_down.is_some() {
                via = pref_down;
            }
            let width = if nw != 0 { nw } else { w };
            self.mod_min_spacing_cost_via_helper(b, z, t, width, dsp.max(ns), via, econ, upper, curr_ps, is_blockage, true);
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn mod_min_spacing_cost_via_helper(&mut self, b: &Rect, z: usize, t: ModCost, width: i32, min_spacing: i32, via: Option<usize>, dcon: EolTable, upper: bool, curr_ps: bool, is_blockage: bool, ndr: bool) {
        let Some(v) = via else { return };
        let tech = self.cx.tech;
        let vd = &tech.via_defs[v];
        let vb = if upper { vd.layer1_bbox() } else { vd.layer2_bbox() };
        let (w2, l2) = (min_side(&vb), max_side(&vb));
        let l = self.g.zs[z];
        let is_h = tech.layers[l].is_horizontal();
        let fat = if is_h { vb.dy() > width } else { vb.dx() > width };
        let mut l2_mar = l2;
        let mut patch = 0;
        if !fat {
            let min_area = tech.layers[l].min_area;
            let mg = i64::from(tech.manufacturing_grid.max(1));
            patch = ((min_area as f64 / f64::from(width) / mg as f64).ceil() as i64 * mg) as i32;
            l2_mar = l2_mar.max(patch);
        }
        let (w1, l1) = (min_side(b), max_side(b));
        let prl0 = if curr_ps { l2_mar } else { l1.min(l2_mar) };
        let use_min = is_blockage && self.cx.use_min_spacing_obs && !fat;
        let d = self.cx.min_spacing_value(l, w1, w2, prl0, use_min).max(min_spacing);
        let con = if dcon.width == 0 { self.cx.eol_of(l) } else { dcon };
        let mut eol_x = 0;
        let mut eol_y = 0;
        if vb.dx() <= con.width {
            eol_y = eol_y.max(con.space);
        }
        if vb.dy() <= con.width {
            eol_x = eol_x.max(con.space);
        }
        let bx = Rect { xl: b.xl - d.max(eol_x) - vb.xh + 1, yl: b.yl - d.max(eol_y) - vb.yh + 1, xh: b.xh + d.max(eol_x) - vb.xl - 1, yh: b.yh + d.max(eol_y) - vb.yl - 1 };
        let Some((x1, y1, x2, y2)) = self.range(&bx) else { return };
        let zi = if upper { z as i64 } else { z as i64 - 1 };
        if zi < 0 {
            return;
        }
        let zi = zi as usize;
        for i in x1..=x2 {
            for j in y1..=y2 {
                let p = (self.g.xs[i], self.g.ys[j]);
                let k = self.g.idx(i, j, zi);
                let mut tb = vb;
                if self.g.nodes[k].svia {
                    if let Some(&sv) = self.ap_svia.get(&(i, j, zi)) {
                        let svd = &tech.via_defs[sv];
                        tb = if upper { svd.layer1_bbox() } else { svd.layer2_bbox() };
                    }
                }
                let tb = shift(&tb, p);
                let (d2, dx, dy) = box_box_d2(b, &tb);
                let mut prl = (-dx).max(-dy);
                if curr_ps {
                    if -dy >= 0 && prl == -dy {
                        prl = vb.dy();
                        if !is_h && !fat {
                            prl = prl.max(patch);
                        }
                    } else if -dx >= 0 && prl == -dx {
                        prl = vb.dx();
                        if is_h && !fat {
                            prl = prl.max(patch);
                        }
                    }
                }
                let req = self.cx.min_spacing_value(l, w1, w2, prl, use_min).max(min_spacing);
                if d2 < i64::from(req) * i64::from(req) {
                    self.g.mod_via(k, t, ndr);
                }
                // End of line: the other shape against this via.
                if con.space != 0 {
                    let mut tests = Vec::new();
                    if tb.dx() <= con.width {
                        tests.push(Rect { xl: tb.xl - con.within, yl: tb.yh, xh: tb.xh + con.within, yh: tb.yh + con.space });
                        tests.push(Rect { xl: tb.xl - con.within, yl: tb.yl - con.space, xh: tb.xh + con.within, yh: tb.yl });
                    }
                    if tb.dy() <= con.width {
                        tests.push(Rect { xl: tb.xh, yl: tb.yl - con.within, xh: tb.xh + con.space, yh: tb.yh + con.within });
                        tests.push(Rect { xl: tb.xl - con.space, yl: tb.yl - con.within, xh: tb.xl, yh: tb.yh + con.within });
                    }
                    for tst in tests {
                        if overlaps(&tst, b) {
                            self.g.mod_via(k, t, ndr);
                        }
                    }
                }
            }
        }
    }

    /// End-of-line costs around a shape's ends narrower than the rule: planar, and (unless
    /// skipped) the vias below and above.
    #[allow(clippy::too_many_arguments)]
    pub fn mod_eol_spacing_rules_cost(&mut self, b: &Rect, z: usize, t: ModCost, skip_via: bool, ndr_eol: Option<EolTable>, reset_h: bool, reset_v: bool) {
        let l = self.g.zs[z];
        let mut con = ndr_eol.unwrap_or_default();
        if con.width == 0 {
            con = self.cx.eol_of(l);
        }
        if con.space == 0 {
            return;
        }
        let mut tests = Vec::new();
        if b.dx() <= con.width {
            tests.push(Rect { xl: b.xl - con.within, yl: b.yh, xh: b.xh + con.within, yh: b.yh + con.space });
            tests.push(Rect { xl: b.xl - con.within, yl: b.yl - con.space, xh: b.xh + con.within, yh: b.yl });
        }
        if b.dy() <= con.width {
            tests.push(Rect { xl: b.xh, yl: b.yl - con.within, xh: b.xh + con.space, yh: b.yh + con.within });
            tests.push(Rect { xl: b.xl - con.space, yl: b.yl - con.within, xh: b.xl, yh: b.yh + con.within });
        }
        for tst in tests {
            self.mod_eol_spacing_cost_helper(&tst, z, t, 0, reset_h, reset_v);
            if !skip_via {
                self.mod_eol_spacing_cost_helper(&tst, z, t, 1, reset_h, reset_v);
                self.mod_eol_spacing_cost_helper(&tst, z, t, 2, reset_h, reset_v);
            }
        }
    }

    /// The space beyond one end-of-line edge (away from the shape): route cost for a wire there,
    /// and for the vias below and above.
    fn mod_eol_cost(&mut self, e: &Edge, z: usize, eol: EolTable, t: ModCost) {
        let (lo, hi, line) = (e.low, e.high, e.line);
        let tst = match (e.vertical, e.inner_increasing) {
            (true, true) => Rect { xl: line - eol.space, yl: lo - eol.within, xh: line, yh: hi + eol.within },
            (true, false) => Rect { xl: line, yl: lo - eol.within, xh: line + eol.space, yh: hi + eol.within },
            (false, true) => Rect { xl: lo - eol.within, yl: line - eol.space, xh: hi + eol.within, yh: line },
            (false, false) => Rect { xl: lo - eol.within, yl: line, xh: hi + eol.within, yh: line + eol.space },
        };
        self.mod_eol_spacing_cost_helper(&tst, z, t, 0, true, true);
        self.mod_eol_spacing_cost_helper(&tst, z, t, 1, true, true);
        self.mod_eol_spacing_cost_helper(&tst, z, t, 2, true, true);
    }

    fn mod_eol_spacing_cost_helper(&mut self, tst: &Rect, z: usize, t: ModCost, eol_type: u8, reset_h: bool, reset_v: bool) {
        let tech = self.cx.tech;
        let l = self.g.zs[z];
        let bx = if eol_type == 0 {
            let hw2 = self.cx.width(l) / 2;
            Rect { xl: tst.xl - hw2 + 1, yl: tst.yl - hw2 + 1, xh: tst.xh + hw2 - 1, yh: tst.yh + hw2 - 1 }
        } else {
            let via = if eol_type == 1 {
                if l > 0 { self.cx.default_via(l as i64 - 1) } else { None }
            } else if l + 1 < tech.layers.len() {
                self.cx.default_via(l as i64 + 1)
            } else {
                None
            };
            let Some(v) = via else { return };
            let vd = &tech.via_defs[v];
            let vb = if eol_type == 2 { vd.layer1_bbox() } else { vd.layer2_bbox() };
            Rect { xl: tst.xl - vb.xh + 1, yl: tst.yl - vb.yh + 1, xh: tst.xh - vb.xl - 1, yh: tst.yh - vb.yl - 1 }
        };
        let Some((x1, y1, x2, y2)) = self.range(&bx) else { return };
        for i in x1..=x2 {
            for j in y1..=y2 {
                match eol_type {
                    0 => {
                        let k = self.g.idx(i, j, z);
                        self.g.mod_planar(k, t, false, reset_h, reset_v);
                    }
                    _ => {
                        let zi = if eol_type == 1 { z as i64 - 1 } else { z as i64 };
                        if zi < 0 {
                            continue;
                        }
                        let zi = zi as usize;
                        let k = self.g.idx(i, j, zi);
                        if self.g.nodes[k].svia {
                            if let Some(&sv) = self.ap_svia.get(&(i, j, zi)) {
                                let svd = &tech.via_defs[sv];
                                let sb = shift(&if eol_type == 1 { svd.layer2_bbox() } else { svd.layer1_bbox() }, (self.g.xs[i], self.g.ys[j]));
                                if !overlaps(&sb, tst) {
                                    continue;
                                }
                            }
                        }
                        self.g.mod_via(k, t, false);
                    }
                }
            }
        }
    }

    /// Cut spacing costs around a cut shape: the default via's cut at each node too close to it
    /// (one plain, edge-to-edge, different-net rule).
    /// `avoid`: a node left alone (the via's own, when a routed via's cut is the shape).
    pub fn mod_cut_spacing_cost(&mut self, b: &Rect, z: usize, t: ModCost, avoid: Option<(usize, usize)>) {
        let tech = self.cx.tech;
        let cut = self.g.zs[z] + 1;
        let Some(spacing) = tech.layers.get(cut).and_then(|c| c.cut_spacing) else { return };
        let Some(v) = self.cx.default_via(cut as i64) else { return };
        let vd = &tech.via_defs[v];
        let cf = &vd.cut_figs;
        let vb = cf.iter().skip(1).fold(cf[0], |a, f| Rect { xl: a.xl.min(f.xl), yl: a.yl.min(f.yl), xh: a.xh.max(f.xh), yh: a.yh.max(f.yh) });
        let d = spacing;
        let bx = Rect { xl: b.xl - d - vb.xh + 1, yl: b.yl - d - vb.yh + 1, xh: b.xh + d - vb.xl - 1, yh: b.yh + d - vb.yl - 1 };
        let Some((x1, y1, x2, y2)) = self.range(&bx) else { return };
        let req = i64::from(spacing) * i64::from(spacing);
        for i in x1..=x2 {
            for j in y1..=y2 {
                if avoid == Some((i, j)) {
                    continue;
                }
                let p = (self.g.xs[i], self.g.ys[j]);
                for f in cf.clone() {
                    let (d2, _, _) = box_box_d2(b, &shift(&f, p));
                    if d2 < req {
                        let k = self.g.idx(i, j, z);
                        self.g.mod_via(k, t, false);
                        break;
                    }
                }
            }
        }
    }
}

/// A shape a route writes: a wire (its ends as maze indices too), a via (bottom and top maze
/// indices), or a patch (its layer's metal around an origin).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DrFig {
    Seg { layer: usize, begin: P, end: P, width: i32, begin_ext: i32, end_ext: i32, bi: (usize, usize, usize), ei: (usize, usize, usize), tapered: bool },
    Via { via: usize, origin: P, bi: (usize, usize, usize), ei: (usize, usize, usize) },
    Patch { layer: usize, origin: P, offset: Rect },
}

impl DrFig {
    /// The metal a route shape puts on routing layers, as the design-rule check merges it: a
    /// wire's box, each of a via's enclosure rectangles, a patch's box.
    pub fn metal(&self, tech: &Tech) -> Vec<(usize, Rect)> {
        match *self {
            DrFig::Seg { layer, begin, end, width, begin_ext, end_ext, .. } => vec![(layer, DrFig::seg_box(begin, end, width, begin_ext, end_ext))],
            DrFig::Via { via, origin, .. } => {
                let vd = &tech.via_defs[via];
                vd.layer1_figs.iter().map(|f| (vd.layer1, shift(f, origin))).chain(vd.layer2_figs.iter().map(|f| (vd.layer2, shift(f, origin)))).collect()
            }
            DrFig::Patch { layer, origin, offset } => vec![(layer, shift(&offset, origin))],
        }
    }

    /// A wire's box: extended past its ends by their extensions, half its width to each side.
    pub fn seg_box(begin: P, end: P, width: i32, begin_ext: i32, end_ext: i32) -> Rect {
        let hw = width / 2;
        if begin.1 == end.1 {
            Rect { xl: begin.0 - begin_ext, yl: begin.1 - hw, xh: end.0 + end_ext, yh: end.1 + hw }
        } else {
            Rect { xl: begin.0 - hw, yl: begin.1 - begin_ext, xh: end.0 + hw, yh: end.1 + end_ext }
        }
    }
}

fn bbox(figs: &[Rect]) -> Rect {
    figs.iter().skip(1).fold(figs[0], |a, f| Rect { xl: a.xl.min(f.xl), yl: a.yl.min(f.yl), xh: a.xh.max(f.xh), yh: a.yh.max(f.yh) })
}

impl CostWorker<'_, '_> {
    /// A route shape's costs on the grid: its spacing to wires and vias, the vias it forbids
    /// passing through a wire, its end of line (a wire only along its layer's direction — the
    /// other way ends at a via or a wire, never at an end), a via's cut spacing (not at its own
    /// node).
    pub fn mod_path_cost(&mut self, fig: &DrFig, t: ModCost, mod_eol: bool, mod_cut: bool, ndr: Option<Ndr<'_>>) {
        let tech = self.cx.tech;
        let eol_of = |z: usize| ndr.and_then(|(_, e)| e.get(z).copied().flatten());
        match *fig {
            DrFig::Seg { layer, begin, end, width, begin_ext, end_ext, bi, ei, tapered } => {
                let b = DrFig::seg_box(begin, end, width, begin_ext, end_ext);
                let ndr = if tapered { None } else { ndr };
                let z = bi.2;
                self.mod_min_spacing_cost_planar(&b, z, t, false, ndr, false, true, true);
                self.mod_min_spacing_cost_via(&b, z, t, true, true, false, ndr);
                self.mod_min_spacing_cost_via(&b, z, t, false, true, false, ndr);
                self.mod_via_forbidden_through(bi, ei, t);
                if mod_eol && tech.layers[layer].is_horizontal() == (bi.1 == ei.1) {
                    self.mod_eol_spacing_rules_cost(&b, z, t, false, ndr.and(eol_of(z)), true, true);
                }
            }
            DrFig::Patch { layer, origin, offset } => {
                let Some(z) = self.g.z_of(layer) else { return };
                let b = shift(&offset, origin);
                self.mod_min_spacing_cost_planar(&b, z, t, false, ndr, false, true, true);
                self.mod_min_spacing_cost_via(&b, z, t, true, true, false, ndr);
                self.mod_min_spacing_cost_via(&b, z, t, false, true, false, ndr);
                if mod_eol {
                    self.mod_eol_spacing_rules_cost(&b, z, t, false, None, true, true);
                }
            }
            DrFig::Via { via, origin, bi, ei } => {
                let vd = &tech.via_defs[via];
                for (b, z) in [(shift(&bbox(&vd.layer1_figs), origin), bi.2), (shift(&bbox(&vd.layer2_figs), origin), ei.2)] {
                    self.mod_min_spacing_cost_planar(&b, z, t, false, ndr, false, true, true);
                    self.mod_min_spacing_cost_via(&b, z, t, true, false, false, ndr);
                    self.mod_min_spacing_cost_via(&b, z, t, false, false, false, ndr);
                    if mod_eol {
                        self.mod_eol_spacing_rules_cost(&b, z, t, false, eol_of(z), true, true);
                    }
                }
                if mod_cut {
                    for f in vd.cut_figs.clone() {
                        self.mod_cut_spacing_cost(&shift(&f, origin), bi.2, t, Some((bi.0, bi.1)));
                    }
                }
            }
        }
    }

    /// Via cost at every node a wire runs over (not its last) where the layer forbids a via
    /// through a wire in that direction.
    fn mod_via_forbidden_through(&mut self, bi: (usize, usize, usize), ei: (usize, usize, usize), t: ModCost) {
        let horz = bi.1 == ei.1;
        let row = self.cx.through.get(bi.2).copied().unwrap_or_default();
        let (lower, upper) = (row[usize::from(!horz)], row[2 + usize::from(!horz)]);
        let steps: Vec<(usize, usize)> = if horz { (bi.0..ei.0).map(|x| (x, bi.1)).collect() } else { (bi.1..ei.1).map(|y| (bi.0, y)).collect() };
        for (x, y) in steps {
            if lower && bi.2 > 0 {
                let k = self.g.idx(x, y, bi.2 - 1);
                self.g.mod_via(k, t, false);
            }
            if upper {
                let k = self.g.idx(x, y, bi.2);
                self.g.mod_via(k, t, false);
            }
        }
    }
}

/// The unmodelled term shape kinds (a block master's pins).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Unmodelled(pub String);

/// The costs before routing: fixed shapes, access points, block pins' planar access.
pub fn init_maze_cost(w: &mut CostWorker<'_, '_>, nets: &[DrNet], ext_box: &Rect) -> Result<(), Unmodelled> {
    init_maze_cost_fixed_obj(w, ext_box)?;
    init_maze_cost_ap(w, nets);
    init_maze_cost_conn_fig(w, nets, ext_box);
    init_maze_cost_planar_term(w, ext_box);
    Ok(())
}

/// Each worker net's routes (none before routing) and then its end-of-line costs: the net's
/// shapes in the worker (its pin shapes the fixed-shape query returns, merged per layer), and
/// for each boundary edge shorter than the layer's end-of-line width, route cost where a wire
/// or via would sit in the space beyond it.
fn init_maze_cost_conn_fig(w: &mut CostWorker<'_, '_>, nets: &[DrNet], ext_box: &Rect) {
    let owners: BTreeSet<usize> = nets.iter().map(|n| n.net).collect();
    for &owner in &owners {
        mod_eol_costs_poly(w, owner, ext_box, ModCost::AddRoute);
    }
}

/// A net's end-of-line route costs from its shapes in the worker (before routing, its pin shapes
/// the fixed-shape query returns), merged per layer: every boundary edge shorter than the
/// layer's end-of-line width.
pub fn mod_eol_costs_poly(w: &mut CostWorker<'_, '_>, owner: usize, ext_box: &Rect, t: ModCost) {
    mod_eol_costs_poly_with(w, owner, ext_box, &[], t);
}

/// As [`mod_eol_costs_poly`], the net's route shapes (layer, box) merged in too.
pub fn mod_eol_costs_poly_with(w: &mut CostWorker<'_, '_>, owner: usize, ext_box: &Rect, routes: &[(usize, Rect)], t: ModCost) {
    let tech = w.cx.tech;
    for l in 0..tech.layers.len() {
        if tech.layers[l].kind != LayerKind::Routing {
            continue;
        }
        let eol = w.cx.eol_of(l);
        if eol.space == 0 {
            continue;
        }
        let Some(z) = w.g.z_of(l) else { continue };
        let mut set = Polygon90Set::new();
        for (b, obj) in w.cx.fixed.get(l).map_or(Vec::new(), |t| t.query(ext_box)) {
            if obj.term_net() == Some(Some(owner)) {
                set.insert_rect(*b);
            }
        }
        for &(rl, b) in routes {
            if rl == l {
                set.insert_rect(b);
            }
        }
        if set.is_empty() {
            continue;
        }
        for e in set.boundary_edges() {
            if e.high - e.low >= eol.width {
                continue;
            }
            w.mod_eol_cost(&e, z, eol, t);
        }
    }
}

fn init_maze_cost_fixed_obj(w: &mut CostWorker<'_, '_>, ext_box: &Rect) -> Result<(), Unmodelled> {
    let tech = w.cx.tech;
    let (min_l, max_l) = (w.g.zs[0], *w.g.zs.last().expect("a layer"));
    // Per net (unconnected first, then by net), its terminals (block pins first, then database
    // order).
    let mut net_terms: BTreeMap<Option<usize>, BTreeSet<TermKey>> = BTreeMap::new();
    for l in 0..tech.layers.len() {
        let (routing, z) = match tech.layers[l].kind {
            LayerKind::Routing => {
                if l < min_l || l > max_l {
                    continue;
                }
                (true, w.g.z_of(l).expect("a grid layer"))
            }
            LayerKind::Cut => {
                if l == 0 || tech.layers[l - 1].kind != LayerKind::Routing || l - 1 < min_l || l - 1 > max_l {
                    continue;
                }
                (false, w.g.z_of(l - 1).expect("a grid layer"))
            }
            LayerKind::Placeholder => continue,
        };
        let objs: Vec<(Rect, Fixed)> = w.cx.fixed.get(l).map_or(Vec::new(), |t| t.query(ext_box).into_iter().copied().collect());
        for (b, obj) in &objs {
            if matches!(obj, Fixed::Blockage | Fixed::InstBlockage { .. }) {
                if routing {
                    w.mod_min_spacing_cost_planar(b, z, ModCost::AddFixed, true, None, false, true, true);
                    w.mod_min_spacing_cost_via(b, z, ModCost::AddFixed, true, false, true, None);
                    w.mod_min_spacing_cost_via(b, z, ModCost::AddFixed, false, false, true, None);
                    w.mod_eol_spacing_rules_cost(b, z, ModCost::AddFixed, false, None, true, true);
                    w.mod_blocked_planar(b, z, true);
                    w.mod_blocked_via(b, z, true);
                } else {
                    w.mod_cut_spacing_cost(b, z, ModCost::AddFixed, None);
                }
            }
        }
        for (b, obj) in &objs {
            match *obj {
                Fixed::BTerm { net, port } => {
                    net_terms.entry(net).or_default().insert((0, port, 0, *obj));
                }
                Fixed::InstTerm { net, inst, term } => {
                    net_terms.entry(net).or_default().insert((1, inst, term, *obj));
                    if routing {
                        // Unblocked for the pin; its access points unblock the via edges.
                        w.mod_blocked_planar(b, z, false);
                        // At or below the via-access layer's z (half its number, less one).
                        if (z as i64) < w.cx.via_access_layer as i64 / 2 {
                            w.mod_min_spacing_cost_planar(b, z, ModCost::AddFixed, true, None, false, true, true);
                            w.mod_eol_spacing_rules_cost(b, z, ModCost::AddFixed, false, None, true, true);
                        }
                    } else {
                        w.mod_cut_spacing_cost(b, z, ModCost::AddFixed, None);
                    }
                }
                Fixed::Seg { supply } => {
                    w.mod_min_spacing_cost_planar(b, z, ModCost::AddFixed, false, None, false, true, true);
                    w.mod_min_spacing_cost_via(b, z, ModCost::AddFixed, true, true, false, None);
                    w.mod_min_spacing_cost_via(b, z, ModCost::AddFixed, false, true, false, None);
                    w.mod_eol_spacing_rules_cost(b, z, ModCost::AddFixed, false, None, true, true);
                    if supply {
                        w.mod_blocked_planar(b, z, true);
                        w.mod_blocked_via(b, z, true);
                    }
                }
                Fixed::Via { .. } => {
                    if routing {
                        w.mod_min_spacing_cost_planar(b, z, ModCost::AddFixed, false, None, false, true, true);
                        w.mod_min_spacing_cost_via(b, z, ModCost::AddFixed, true, false, false, None);
                        w.mod_min_spacing_cost_via(b, z, ModCost::AddFixed, false, false, false, None);
                        w.mod_eol_spacing_rules_cost(b, z, ModCost::AddFixed, false, None, true, true);
                    } else {
                        // (Adjacent-cut spacing: refused, not modelled.)
                        w.mod_cut_spacing_cost(b, z, ModCost::AddFixed, None);
                    }
                }
                _ => {}
            }
        }
    }
    for terms in net_terms.values() {
        init_maze_cost_terms(w, terms)?;
    }
    Ok(())
}

fn init_maze_cost_terms(w: &mut CostWorker<'_, '_>, terms: &BTreeSet<TermKey>) -> Result<(), Unmodelled> {
    for (_, _, _, obj) in terms {
        if let Fixed::InstTerm { inst, .. } = *obj {
            if (w.cx.inst_is_block)(inst) {
                return Err(Unmodelled("a block master's pins".into()));
            }
        }
        mod_term_cost(w, obj, true, false);
    }
    Ok(())
}

/// One terminal's fixed-shape costs, every pin shape on a grid layer (cut shapes are not
/// costed): a block pin's planar, via and end-of-line spacing; an instance pin's via (unless
/// skipped — a net lifting its own pins keeps their via costs), end-of-line and planar spacing.
pub fn mod_term_cost(w: &mut CostWorker<'_, '_>, obj: &Fixed, add: bool, skip_via: bool) {
    let tech = w.cx.tech;
    let (min_l, max_l) = (w.g.zs[0], *w.g.zs.last().expect("a layer"));
    let is_inst = matches!(obj, Fixed::InstTerm { .. });
    let t = if add { ModCost::AddFixed } else { ModCost::SubFixed };
    for (l, b) in (w.cx.term_shapes)(obj) {
        if tech.layers[l].kind != LayerKind::Routing || l < min_l || l > max_l {
            continue;
        }
        let z = w.g.z_of(l).expect("a grid layer");
        if is_inst {
            if !skip_via {
                w.mod_min_spacing_cost_via(&b, z, t, true, false, false, None);
                w.mod_min_spacing_cost_via(&b, z, t, false, false, false, None);
            }
            w.mod_eol_spacing_rules_cost(&b, z, t, false, None, true, true);
            w.mod_min_spacing_cost_planar(&b, z, t, false, None, false, true, true);
        } else {
            w.mod_min_spacing_cost_planar(&b, z, t, false, None, false, true, true);
            w.mod_min_spacing_cost_via(&b, z, t, true, false, false, None);
            w.mod_min_spacing_cost_via(&b, z, t, false, false, false, None);
            w.mod_eol_spacing_rules_cost(&b, z, t, false, None, true, true);
        }
    }
}

fn init_maze_cost_ap(w: &mut CostWorker<'_, '_>, nets: &[DrNet]) {
    for net in nets {
        for pin in &net.pins {
            for ap in &pin.patterns {
                let (Some(x), Some(y), Some(z)) = (w.g.xs.binary_search(&ap.point.0).ok(), w.g.ys.binary_search(&ap.point.1).ok(), w.g.z_of(ap.layer)) else { continue };
                for d in ALL {
                    let valid = ap.ap.as_ref().is_none_or(|a| a.has(d));
                    w.g.set_blocked(x as i64, y as i64, z as i64, d, !valid);
                }
                if let Some(a) = &ap.ap {
                    if (a.has(Dir6::U) || a.has(Dir6::D)) && !a.vias.is_empty() {
                        let k = w.g.idx(x, y, z);
                        w.g.nodes[k].svia = true;
                        w.ap_svia.insert((x, y, z), a.vias[0]);
                    }
                }
            }
        }
    }
}

fn init_maze_cost_planar_term(w: &mut CostWorker<'_, '_>, ext_box: &Rect) {
    let tech = w.cx.tech;
    for l in 0..tech.layers.len() {
        if tech.layers[l].kind != LayerKind::Routing {
            continue;
        }
        let Some(z) = w.g.z_of(l) else { continue };
        let objs: Vec<(Rect, Fixed)> = w.cx.fixed.get(l).map_or(Vec::new(), |t| t.query(ext_box).into_iter().copied().collect());
        for (b, obj) in objs {
            let Fixed::BTerm { port, .. } = obj else { continue };
            let aps = (w.cx.port_aps)(port);
            let (mut h, mut v, mut up, mut down) = (false, false, false, false);
            for a in aps.iter().filter(|a| a.layer == l) {
                v |= a.has(Dir6::N) || a.has(Dir6::S);
                h |= a.has(Dir6::E) || a.has(Dir6::W);
                up |= a.has(Dir6::U);
                down |= a.has(Dir6::D);
            }
            let Some((x1, y1, x2, y2)) = w.range(&b) else { continue };
            let horz = tech.layers[l].is_horizontal();
            for i in x1..=x2 {
                for j in y1..=y2 {
                    let (ii, jj, zz) = (i as i64, j as i64, z as i64);
                    if !up {
                        w.g.set_blocked(ii, jj, zz, Dir6::U, true);
                    }
                    if !down {
                        w.g.set_blocked(ii, jj, zz, Dir6::D, true);
                    }
                    if horz && h {
                        w.g.set_blocked(ii, jj, zz, Dir6::N, true);
                        w.g.set_blocked(ii, jj, zz, Dir6::S, true);
                    } else if !horz && v {
                        w.g.set_blocked(ii, jj, zz, Dir6::W, true);
                        w.g.set_blocked(ii, jj, zz, Dir6::E, true);
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dr::drw::Node;
    use crate::tech::{Dir, Layer, SpacingTable};

    fn tech(spacing: i32) -> Tech {
        let routing = |dir| Layer { kind: LayerKind::Routing, dir, width: 100, spacing: Some(SpacingTable { widths: vec![0], prls: vec![0], values: vec![vec![spacing]] }), ..Default::default() };
        Tech { layers: vec![Layer::default(), Layer::default(), routing(Dir::Horizontal), Layer { kind: LayerKind::Cut, ..Default::default() }, routing(Dir::Vertical)], manufacturing_grid: 5, via_defs: vec![] }
    }

    /// A 10 × 10 grid at a 100 pitch from 0, on layers 2 and 4, every edge present.
    fn grid() -> GridGraph {
        let node = Node { east: true, north: true, up: true, ..Default::default() };
        GridGraph { xs: (0..10).map(|i| i * 100).collect(), ys: (0..10).map(|i| i * 100).collect(), zs: vec![2, 4], nodes: vec![node; 200] }
    }

    fn with<R>(t: &Tech, g: &mut GridGraph, f: impl FnOnce(&mut CostWorker<'_, '_>) -> R) -> R {
        let none = |_: &Fixed| Vec::new();
        let aps = |_: usize| Vec::new();
        let block = |_: usize| false;
        let fixed: Vec<PackedRTree<Fixed>> = Vec::new();
        let cx = CostCtx { tech: t, defaults: &[], eol: &[], ndrs: Vec::new(), use_min_spacing_obs: true, through: &[], via_access_layer: 2, fixed: &fixed, term_shapes: &none, port_aps: &aps, inst_is_block: &block };
        let mut w = CostWorker { cx: &cx, g, ap_svia: BTreeMap::new() };
        f(&mut w)
    }

    /// A two-layer technology with a cut between (layers 2 H, 3 cut, 4 V), a default via whose
    /// metal is `via` (both layers) and cut 20 × 20, the routing layers `width` wide with a
    /// two-column run-length spacing table: `near` under a 500 run, `far` over it.
    fn tech_via(width: i32, via: Rect, min_area: i64, near: i32, far: i32, cut_spacing: Option<i32>) -> Tech {
        let routing = |dir| Layer { kind: LayerKind::Routing, dir, width, min_width: width, min_area, spacing: Some(SpacingTable { widths: vec![0], prls: vec![0, 500], values: vec![vec![near, far]] }), ..Default::default() };
        let vd = crate::tech::ViaDef { name: "V".into(), is_default: true, layer1: 2, cut: 3, layer2: 4, layer1_figs: vec![via], cut_figs: vec![Rect { xl: -10, yl: -10, xh: 10, yh: 10 }], layer2_figs: vec![via] };
        Tech { layers: vec![Layer::default(), Layer::default(), routing(Dir::Horizontal), Layer { kind: LayerKind::Cut, cut_spacing, ..Default::default() }, routing(Dir::Vertical)], manufacturing_grid: 5, via_defs: vec![vd] }
    }

    fn with_defaults<R>(t: &Tech, g: &mut GridGraph, eol: &[EolTable], fixed: &[PackedRTree<Fixed>], f: impl FnOnce(&mut CostWorker<'_, '_>) -> R) -> R {
        let shapes = |f: &Fixed| match f {
            Fixed::InstTerm { .. } => vec![(2, Rect { xl: 400, yl: 400, xh: 500, yh: 1400 })],
            _ => Vec::new(),
        };
        let aps = |_: usize| Vec::new();
        let block = |_: usize| false;
        let defaults = [None, None, None, Some(0), None];
        let cx = CostCtx { tech: t, defaults: &defaults, eol, ndrs: Vec::new(), use_min_spacing_obs: true, through: &[], via_access_layer: 2, fixed, term_shapes: &shapes, port_aps: &aps, inst_is_block: &block };
        let mut w = CostWorker { cx: &cx, g, ap_svia: BTreeMap::new() };
        f(&mut w)
    }

    // Rule: a via's metal that is not wider than the wire (height ≤ width on a horizontal layer
    // — "fat" is STRICTLY wider) counts the minimum-area patch in its run length; against a
    // special-net wire (curr_ps) the run is the via's side stretched to the patch, which here
    // crosses the spacing table's 500 column.
    #[test]
    fn a_non_fat_via_counts_the_min_area_patch_in_its_run() {
        // Via metal 100 × 100 on a 100-wide layer: not fat. Patch: 100,000 / 100 → 1,000.
        let t = tech_via(100, Rect { xl: -50, yl: -50, xh: 50, yh: 50 }, 100_000, 100, 300, None);
        let mut g = grid();
        g.xs = (0..10).map(|i| i * 250).collect();
        g.ys = (0..10).map(|i| i * 50).collect();
        // A wire 0..2000 × 0..100; the via at (1000, 350): 200 from it — within 300, not 100.
        with_defaults(&t, &mut g, &[], &[], |w| w.mod_min_spacing_cost_via(&Rect { xl: 0, yl: 0, xh: 2000, yh: 100 }, 0, ModCost::AddFixed, true, true, false, None));
        assert_eq!(g.nodes[g.idx(4, 7, 0)].fixed_via, 1);
    }

    // Rule: at a special-via node the via's metal is the access point's own via, not the default.
    #[test]
    fn a_special_via_node_uses_the_access_points_via() {
        let mut t = tech_via(100, Rect { xl: -50, yl: -50, xh: 50, yh: 50 }, 0, 100, 100, None);
        // The access via: metal 130 each way.
        t.via_defs.push(crate::tech::ViaDef { name: "AP".into(), is_default: false, layer1: 2, cut: 3, layer2: 4, layer1_figs: vec![Rect { xl: -130, yl: -130, xh: 130, yh: 130 }], cut_figs: vec![Rect { xl: -10, yl: -10, xh: 10, yh: 10 }], layer2_figs: vec![Rect { xl: -50, yl: -50, xh: 50, yh: 50 }] });
        let mut g = grid();
        g.xs = vec![0, 100, 200, 300, 430, 500, 600, 700, 800, 900];
        g.ys = vec![0, 100, 200, 230, 300, 400, 500, 600, 700, 800];
        // A wire 0..300 × 0..100; the node at (430, 230), diagonal: the default via's metal is 80
        // and 80 away (113 > 100, clean), the access via's touches the wire.
        let k = g.idx(4, 3, 0);
        g.nodes[k].svia = true;
        let mut g2 = g.clone();
        with_defaults(&t, &mut g, &[], &[], |w| {
            w.ap_svia.insert((4, 3, 0), 1);
            w.mod_min_spacing_cost_via(&Rect { xl: 0, yl: 0, xh: 300, yh: 100 }, 0, ModCost::AddFixed, true, false, false, None)
        });
        assert_eq!(g.nodes[k].fixed_via, 1);
        // Without the special via, the default metal is clean there.
        g2.nodes[k].svia = false;
        with_defaults(&t, &mut g2, &[], &[], |w| w.mod_min_spacing_cost_via(&Rect { xl: 0, yl: 0, xh: 300, yh: 100 }, 0, ModCost::AddFixed, true, false, false, None));
        assert_eq!(g2.nodes[k].fixed_via, 0);
    }

    // Rule: an end is an end of line when its width is AT MOST the rule's width.
    #[test]
    fn an_end_exactly_the_eol_width_is_an_end_of_line() {
        let t = tech(100);
        let mut g = grid();
        let eol = [EolTable { width: 100, space: 150, within: 0 }, EolTable::default()];
        // A vertical wire 100 wide ending at y 300: the space above it (300..450) is costed.
        with_defaults(&t, &mut g, &eol, &[], |w| w.mod_eol_spacing_rules_cost(&Rect { xl: 350, yl: 0, xh: 450, yh: 300 }, 0, ModCost::AddFixed, true, None, true, true));
        assert_eq!(g.nodes[g.idx(4, 4, 0)].fixed_h, 1);
    }

    // Rule: cut spacing is violated only when STRICTLY closer than the rule. (Along an axis the
    // candidate box already stops one short of the rule; exactly-at-the-rule happens diagonally.)
    #[test]
    fn a_cut_exactly_at_the_spacing_is_clean() {
        let t = tech_via(100, Rect { xl: -50, yl: -50, xh: 50, yh: 50 }, 0, 100, 100, Some(50));
        let mut g = grid();
        g.xs = vec![0, 100, 200, 300, 349, 350, 500, 600, 700, 800];
        g.ys = vec![0, 100, 200, 300, 359, 360, 500, 600, 700, 800];
        // A cut 290..310 square; the via cut at (350, 360) spans 340..360 × 350..370: 30 and 40
        // away — 50 exactly. At (349, 359) it is closer.
        with_defaults(&t, &mut g, &[], &[], |w| w.mod_cut_spacing_cost(&Rect { xl: 290, yl: 290, xh: 310, yh: 310 }, 0, ModCost::AddFixed, None));
        assert_eq!(g.nodes[g.idx(5, 5, 0)].fixed_via, 0);
        assert_eq!(g.nodes[g.idx(4, 4, 0)].fixed_via, 1);
    }

    // Rule: an instance pin UNBLOCKS the planar edges a blockage blocked under it (its access
    // points then unblock the vias).
    #[test]
    fn a_pin_unblocks_the_planar_edges_under_it() {
        let t = tech_via(100, Rect { xl: -50, yl: -50, xh: 50, yh: 50 }, 0, 100, 100, None);
        let mut g = grid();
        let blk = Rect { xl: 0, yl: 0, xh: 900, yh: 900 };
        let pin = Rect { xl: 400, yl: 400, xh: 500, yh: 1400 };
        let fixed: Vec<PackedRTree<Fixed>> = (0..5).map(|l| PackedRTree::new(if l == 2 { vec![(blk, Fixed::Blockage), (pin, Fixed::InstTerm { net: Some(0), inst: 0, term: 0 })] } else { Vec::new() })).collect();
        with_defaults(&t, &mut g, &[], &fixed, |w| init_maze_cost(w, &[], &Rect { xl: 0, yl: 0, xh: 900, yh: 900 }).expect("modelled"));
        assert!(!g.is_blocked(4, 5, 0, Dir6::E));
        assert!(g.is_blocked(2, 2, 0, Dir6::E));
    }

    // Rule: a pin edge is an end of line only when SHORTER than the rule's width.
    #[test]
    fn a_pin_edge_exactly_the_eol_width_costs_no_route() {
        let t = tech_via(100, Rect { xl: -50, yl: -50, xh: 50, yh: 50 }, 0, 100, 100, None);
        let mut g = grid();
        let pin = Rect { xl: 400, yl: 400, xh: 500, yh: 1400 };
        let fixed: Vec<PackedRTree<Fixed>> = (0..5).map(|l| PackedRTree::new(if l == 2 { vec![(pin, Fixed::InstTerm { net: Some(0), inst: 0, term: 0 })] } else { Vec::new() })).collect();
        let eol = [EolTable { width: 100, space: 150, within: 0 }, EolTable::default()];
        with_defaults(&t, &mut g, &eol, &fixed, |w| mod_eol_costs_poly(w, 0, &Rect { xl: 0, yl: 0, xh: 900, yh: 900 }, ModCost::AddRoute));
        assert!(g.nodes.iter().all(|n| n.route_planar == 0 && n.route_via == 0));
    }

    // Rule: a cost is an 8-bit count — adding saturates at 255, subtracting floors at 0.
    #[test]
    fn costs_saturate_and_floor() {
        let mut g = grid();
        for _ in 0..300 {
            g.mod_planar(0, ModCost::AddFixed, false, true, true);
        }
        assert_eq!((g.nodes[0].fixed_h, g.nodes[0].fixed_v), (255, 255));
        for _ in 0..300 {
            g.mod_planar(0, ModCost::SubFixed, false, true, true);
        }
        assert_eq!(g.nodes[0].fixed_h, 0);
    }

    // Rule: the index range's high end is the last coordinate at or below the box's high side,
    // clamped at 0 — so a box wholly left of the grid still spans index 0 (and blocks it).
    #[test]
    fn a_box_left_of_the_grid_still_spans_index_zero() {
        let t = tech(100);
        let mut g = grid();
        with(&t, &mut g, |w| w.mod_blocked_planar(&Rect { xl: -500, yl: 300, xh: -400, yh: 300 }, 0, true));
        assert!(g.is_blocked(0, 3, 0, Dir6::E));
        assert!(!g.is_blocked(1, 3, 0, Dir6::E));
    }

    // Rule: a node is costed when the default wire's nearest CORNER (centre ± half width) is
    // closer than the spacing — a node inside the bloated box but diagonal to the shape is not.
    #[test]
    fn planar_spacing_is_tested_from_the_wire_corners() {
        let t = tech(200);
        let mut g = grid();
        with(&t, &mut g, |w| w.mod_min_spacing_cost_planar(&Rect { xl: 400, yl: 400, xh: 500, yh: 500 }, 0, ModCost::AddFixed, false, None, false, true, true));
        let at = |x, y| g.nodes[g.idx(x, y, 0)].fixed_h;
        // (200, 400): corner 250 is 150 from 400, inside 200.
        assert_eq!(at(2, 4), 1);
        // (200, 200): corner (250, 250) is 150 away in x and y — 212 diagonally, not closer.
        assert_eq!(at(2, 2), 0);
        // (100, 400): corner 150 is 250 away.
        assert_eq!(at(1, 4), 0);
        assert_eq!(at(7, 7), 0);
    }

    // Rule: an access point blocks each direction it may not leave in and unblocks the rest; one
    // that may leave by a via, and has a via, marks a special via there (its best via).
    #[test]
    fn access_points_block_and_mark_special_vias() {
        use crate::dr::drw::{DrAccessPattern, DrNet, DrPin};
        let t = tech(100);
        let mut g = grid();
        // Up and north only (bits N 1, U 16).
        let ap = DrAp { point: (300, 300), layer: 2, access: 1 | 16, vias: vec![7] };
        let pat = DrAccessPattern { point: (300, 300), layer: 2, begin_area: 0, pin_cost: 0, ap: Some(ap) };
        // A boundary point (no access record): every direction open, even one blocked before.
        let edge = DrAccessPattern { point: (500, 500), layer: 2, begin_area: 0, pin_cost: 0, ap: None };
        g.set_blocked(5, 5, 0, Dir6::E, true);
        let net = DrNet { id: 0, net: 0, pins: vec![DrPin { term: None, id: 0, patterns: vec![pat] }, DrPin { term: None, id: 1, patterns: vec![edge] }], num_pins_in: 2, pin_box: Rect { xl: 0, yl: 0, xh: 0, yh: 0 } };
        let svia = with(&t, &mut g, |w| {
            init_maze_cost_ap(w, std::slice::from_ref(&net));
            w.ap_svia.clone()
        });
        assert!(g.is_blocked(3, 3, 0, Dir6::E) && g.is_blocked(3, 3, 0, Dir6::W) && g.is_blocked(3, 3, 0, Dir6::S));
        assert!(!g.is_blocked(3, 3, 0, Dir6::N) && !g.is_blocked(3, 3, 0, Dir6::U));
        assert!(g.nodes[g.idx(3, 3, 0)].svia);
        assert_eq!(svia.get(&(3, 3, 0)), Some(&7));
        assert!(ALL.iter().all(|&d| !g.is_blocked(5, 5, 0, d)));
        assert!(!g.nodes[g.idx(5, 5, 0)].svia);
    }
}
