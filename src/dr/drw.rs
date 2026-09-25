// SPDX-License-Identifier: Apache-2.0
//! Detailed routing's search and repair: workers over clipped boxes of the die, each routing the
//! nets that reach its box. This module: the workers' boxes and order, and each worker's nets.
//!
//! Stages, in order:
//! - [`gcell_boundary_pins`]: where each net's assigned wires (from track assignment) cross a
//!   gcell's side — the points a worker's box must connect its part of the net to;
//! - [`worker_boxes`]: one worker per `size × size` gcells from `offset`, its route box, its
//!   extended box (the route box plus a margin) and its check box; grouped into a 2 × 2
//!   checkerboard — workers of a group never touch — in groups (0,0), (0,1), (1,0), (1,1);
//! - [`init_nets_init_dr`] (the first iteration): the nets whose gr pins or guides touch the route
//!   box, ordered by net; each split into connected parts (its guides' gcells and its terminals,
//!   touching), each part a routed net with its terminals' access points inside the box and its
//!   boundary points.
//!
//! Rules:
//! - a terminal set is ordered block pins first, then instance terminals, each by database order;
//! - a boundary point set is ordered by point (x, then y), then layer;
//! - connected parts are found depth first with an explicit stack (the last neighbour first).

use std::collections::{BTreeMap, BTreeSet};

use crate::dr::guides::GCellGrid;
use crate::polygon90::Rect;
use crate::rtree::PackedRTree;

type P = (i32, i32);

/// A terminal set's key: block pins first, then database order; then the terminal.
type TermKey = ((bool, usize), usize);

/// One part of a net: its terminals and its boundary points.
type NetPart = (Vec<usize>, Vec<(P, usize)>);

/// Per net, the boundary points `(point, layer)` of one gcell (ordered).
pub type BoundaryPins = BTreeMap<usize, BTreeSet<(P, usize)>>;

/// Where each net's assigned wires cross a gcell side: per gcell `[x][y]`, per net, the points on
/// the gcell's low side (the wire starts before it) and high side (it ends at or beyond it). A
/// wire of length 0 or 1 is skipped. `wires`: `(net, layer, begin, end)` with `begin <= end`.
pub fn gcell_boundary_pins(grid: &GCellGrid, wires: &[(usize, usize, P, P)]) -> Vec<Vec<BoundaryPins>> {
    let (nx, ny) = (grid.x.1 as usize, grid.y.1 as usize);
    let mut out = vec![vec![BoundaryPins::new(); ny]; nx];
    for &(net, layer, bp, ep) in wires {
        let d = (ep.0 - bp.0).abs() + (ep.1 - bp.1).abs();
        if d <= 1 {
            continue;
        }
        let (i1, i2) = (grid.idx(bp), grid.idx(ep));
        if bp.1 == ep.1 {
            let y = i1.1;
            for x in i1.0..=i2.0 {
                let b = grid.gcell_box((x, y));
                let cell = out[x as usize][y as usize].entry(net).or_default();
                if bp.0 < b.xl {
                    cell.insert(((b.xl, bp.1), layer));
                }
                if ep.0 >= b.xh {
                    cell.insert(((b.xh, ep.1), layer));
                }
            }
        } else if bp.0 == ep.0 {
            let x = i1.0;
            for y in i1.1..=i2.1 {
                let b = grid.gcell_box((x, y));
                let cell = out[x as usize][y as usize].entry(net).or_default();
                if bp.1 < b.yl {
                    cell.insert(((bp.0, b.yl), layer));
                }
                if ep.1 >= b.yh {
                    cell.insert(((ep.0, b.yh), layer));
                }
            }
        }
    }
    out
}

/// A worker's boundary points: those of its gcells lying on its route box's border.
pub fn merge_boundary_pins(pins: &[Vec<BoundaryPins>], start: P, size: i32, route_box: &Rect) -> BoundaryPins {
    let mut out = BoundaryPins::new();
    let nx = pins.len() as i32;
    let ny = pins.first().map_or(0, |c| c.len()) as i32;
    let mut i = start.0;
    while i < nx && i < start.0 + size {
        let mut j = start.1;
        while j < ny && j < start.1 + size {
            for (&net, s) in &pins[i as usize][j as usize] {
                for &(pt, l) in s {
                    if pt.0 == route_box.xl || pt.0 == route_box.xh || pt.1 == route_box.yl || pt.1 == route_box.yh {
                        out.entry(net).or_default().insert((pt, l));
                    }
                }
            }
            j += 1;
        }
        i += 1;
    }
    out
}

/// A worker's boxes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WorkerBoxes {
    /// The gcell index of its low corner.
    pub start: P,
    pub route: Rect,
    pub ext: Rect,
    pub drc: Rect,
}

/// The workers of one iteration, in the order they run: the four checkerboard groups, each in
/// creation order (x outer, y inner).
pub fn worker_boxes(grid: &GCellGrid, size: i32, offset: i32, mt_safe: i32, drc_safe: i32) -> Vec<WorkerBoxes> {
    let (nx, ny) = (grid.x.1, grid.y.1);
    let mut groups: Vec<Vec<WorkerBoxes>> = vec![Vec::new(); 4];
    let bloat = |r: &Rect, d: i32| Rect { xl: r.xl - d, yl: r.yl - d, xh: r.xh + d, yh: r.yh + d };
    let mut xi = 0;
    let mut i = offset;
    while i < nx {
        let mut yi = 0;
        let mut j = offset;
        while j < ny {
            let b1 = grid.gcell_box((i, j));
            // ⚠️ The y bound is the gcell COUNT (not count − 1); the box lookup clamps it.
            let b2 = grid.gcell_box(((nx - 1).min(i + size - 1), ny.min(j + size - 1)));
            let route = Rect { xl: b1.xl, yl: b1.yl, xh: b2.xh, yh: b2.yh };
            groups[((xi % 2) * 2 + yi % 2) as usize].push(WorkerBoxes { start: (i, j), route, ext: bloat(&route, mt_safe), drc: bloat(&route, drc_safe) });
            yi += 1;
            j += size;
        }
        xi += 1;
        i += size;
    }
    groups.into_iter().flatten().collect()
}

/// An access point as a worker reads it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DrAp {
    pub point: P,
    pub layer: usize,
    /// The access bits (east, south, west, north, up, down → 1, 2, 4, 8, 16, 32).
    pub access: u8,
    /// Via choices, best first.
    pub vias: Vec<usize>,
}

/// A terminal as a worker reads it.
#[derive(Debug, Clone)]
pub struct DrTerm {
    /// `inst/term` or `PIN/name`.
    pub name: String,
    pub is_port: bool,
    /// Database order among terminals of its kind.
    pub order: usize,
    pub net: Option<usize>,
    pub bbox: Rect,
    /// Per pin: whether it has access at all, its points (the instance's class), and the index of
    /// the instance's chosen point among them.
    pub pins: Vec<(bool, Vec<DrAp>, Option<usize>)>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DrAccessPattern {
    pub point: P,
    pub layer: usize,
    pub begin_area: i64,
    pub pin_cost: u32,
    pub ap: Option<DrAp>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DrPin {
    /// The terminal (index into the terminal list); none for a boundary point.
    pub term: Option<usize>,
    pub id: usize,
    pub patterns: Vec<DrAccessPattern>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DrNet {
    pub id: usize,
    pub net: usize,
    pub pins: Vec<DrPin>,
    pub num_pins_in: usize,
    pub pin_box: Rect,
}

/// What a worker reads of the design to build its nets.
pub struct DrNetInput<'a> {
    pub grid: &'a GCellGrid,
    /// Per layer, the guide tree: `(net, begin, end)` per guide.
    pub guides: &'a [PackedRTree<(usize, P, P)>],
    /// The gr pin tree: the terminal of each.
    pub gr_pins: &'a PackedRTree<usize>,
    pub terms: &'a [DrTerm],
    /// Per layer, its minimum area (0 without one).
    pub min_area: &'a [i64],
}

fn sq_dist(a: &Rect, b: &Rect) -> i64 {
    let dx = i64::from((a.xl.max(b.xl) - a.xh.min(b.xh)).max(0));
    let dy = i64::from((a.yl.max(b.yl) - a.yh.min(b.yh)).max(0));
    dx * dx + dy * dy
}

fn touches(a: &Rect, b: &Rect) -> bool {
    a.xl <= b.xh && b.xl <= a.xh && a.yl <= b.yh && b.yl <= a.yh
}

/// The first iteration's nets of a worker (ripping everything up: no existing routes).
pub fn init_nets_init_dr(inp: &DrNetInput<'_>, route_box: &Rect, ext_box: &Rect, boundary: &BoundaryPins) -> Vec<DrNet> {
    // Terminal order: block pins first, then instance terminals, each by database order.
    let term_key = |t: usize| (!inp.terms[t].is_port, inp.terms[t].order);
    let mut nets: BTreeSet<usize> = BTreeSet::new();
    let mut net_terms: BTreeMap<usize, BTreeSet<TermKey>> = BTreeMap::new();
    for v in inp.gr_pins.query(route_box) {
        let t = v.1;
        let Some(net) = inp.terms[t].net else { continue };
        nets.insert(net);
        net_terms.entry(net).or_default().insert((term_key(t), t));
    }
    let mut guides: Vec<(usize, usize, P, P)> = Vec::new();
    for (l, tree) in inp.guides.iter().enumerate() {
        for v in tree.query(route_box) {
            let (net, b, e) = v.1;
            nets.insert(net);
            guides.push((net, l, b, e));
        }
    }
    let mut net_guides: BTreeMap<usize, Vec<(usize, Rect)>> = BTreeMap::new();
    for &(net, l, b, e) in &guides {
        let (bb, eb) = (inp.grid.gcell_box(inp.grid.idx(b)), inp.grid.gcell_box(inp.grid.idx(e)));
        net_guides.entry(net).or_default().push((l, Rect { xl: bb.xl, yl: bb.yl, xh: eb.xh, yh: eb.yh }));
    }
    let mut out: Vec<DrNet> = Vec::new();
    let mut pin_cnt = 0usize;
    for &net in &nets {
        let terms: Vec<usize> = net_terms.get(&net).map_or(Vec::new(), |s| s.iter().map(|&(_, t)| t).collect());
        let g = net_guides.get(&net).cloned().unwrap_or_default();
        let bounds: Vec<(P, usize)> = boundary.get(&net).map_or(Vec::new(), |s| s.iter().copied().collect());
        for (part_terms, part_bounds) in init_nets_init_dr_helper(inp, &terms, &g, &bounds) {
            let id = out.len();
            let mut dnet = DrNet { id, net, pins: Vec::new(), num_pins_in: 0, pin_box: *ext_box };
            init_net_term(inp, route_box, &mut dnet, &part_terms, &mut pin_cnt);
            // Boundary points, ordered (a map by point then layer), area 0 in the first iteration.
            let set: BTreeSet<(P, usize)> = part_bounds.into_iter().collect();
            for (pt, l) in set {
                dnet.pins.push(DrPin { term: None, id: pin_cnt, patterns: vec![DrAccessPattern { point: pt, layer: l, begin_area: 0, pin_cost: 0, ap: None }] });
                pin_cnt += 1;
            }
            out.push(dnet);
        }
    }
    init_nets_num_pins_in(&mut out, ext_box);
    out
}

/// A net's parts: its guides and terminals (terminals' boxes) that touch, depth first; parts
/// without a guide dropped. One part (or none): everything. Several: each terminal to the part
/// nearest it (its own part first), each boundary point to the part whose guide is nearest (plus
/// the layer difference).
fn init_nets_init_dr_helper(inp: &DrNetInput<'_>, terms: &[usize], guides: &[(usize, Rect)], bounds: &[(P, usize)]) -> Vec<NetPart> {
    // Nodes: guides first, then terminals.
    let mut rects: Vec<Rect> = guides.iter().map(|g| g.1).collect();
    let n_guides = rects.len();
    rects.extend(terms.iter().map(|&t| inp.terms[t].bbox));
    let n = rects.len();
    let mut adj: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for i in 0..n {
        for j in i + 1..n {
            if touches(&rects[i], &rects[j]) {
                adj.entry(i).or_default().push(j);
                adj.entry(j).or_default().push(i);
            }
        }
    }
    let mut visited = vec![false; n];
    let mut comps: Vec<Vec<usize>> = Vec::new();
    for i in 0..n {
        if visited[i] {
            continue;
        }
        let mut comp = Vec::new();
        let mut stack = vec![i];
        while let Some(node) = stack.pop() {
            if visited[node] {
                continue;
            }
            visited[node] = true;
            comp.push(node);
            if let Some(nb) = adj.get(&node) {
                for &x in nb {
                    if !visited[x] {
                        stack.push(x);
                    }
                }
            }
        }
        comps.push(comp);
    }
    comps.retain(|c| c.iter().any(|&k| k < n_guides));
    if comps.len() <= 1 {
        return vec![(terms.to_vec(), bounds.to_vec())];
    }
    let mut out: Vec<NetPart> = vec![(Vec::new(), Vec::new()); comps.len()];
    for (i, &t) in terms.iter().enumerate() {
        let r = inp.terms[t].bbox;
        let (mut best_d, mut best) = (i64::MAX, None);
        'parts: for (j, comp) in comps.iter().enumerate() {
            for &k in comp {
                let d = sq_dist(&rects[k], &r);
                if d < best_d {
                    best_d = d;
                    best = Some(j);
                }
                if k == n_guides + i {
                    best = Some(j);
                    break 'parts;
                }
            }
        }
        if let Some(j) = best {
            out[j].0.push(t);
        }
    }
    for &(pt, l) in bounds {
        let q = Rect { xl: pt.0, yl: pt.1, xh: pt.0, yh: pt.1 };
        let (mut best_d, mut best) = (i64::MAX, None);
        for (j, comp) in comps.iter().enumerate() {
            for &k in comp {
                if k >= n_guides {
                    continue;
                }
                let d = sq_dist(&rects[k], &q) + (guides[k].0 as i64 - l as i64).abs();
                if d < best_d {
                    best_d = d;
                    best = Some(j);
                }
            }
        }
        if let Some(j) = best {
            out[j].1.push((pt, l));
        }
    }
    out
}

/// Each terminal a pin: its pins' points (the instance's class) inside the route box, the chosen
/// one costing 0 and the rest 1, each with its layer's minimum area.
fn init_net_term(inp: &DrNetInput<'_>, route_box: &Rect, net: &mut DrNet, terms: &[usize], pin_cnt: &mut usize) {
    for &t in terms {
        let term = &inp.terms[t];
        let mut patterns = Vec::new();
        // ⚠️ The chosen point is looked up by a counter that advances only past pins WITH
        // access, and compared by identity with this pin's points: once the counter falls behind
        // (a pin without access came first), nothing matches and every point costs 1.
        let mut pin_idx = 0usize;
        for (cur, (has, aps, _)) in term.pins.iter().enumerate() {
            let pref = if pin_idx == cur { term.pins[cur].2 } else { None };
            if !has {
                continue;
            }
            for (k, ap) in aps.iter().enumerate() {
                let dap = DrAccessPattern { point: ap.point, layer: ap.layer, begin_area: inp.min_area.get(ap.layer).copied().unwrap_or(0), pin_cost: u32::from(Some(k) != pref), ap: Some(ap.clone()) };
                if touches(route_box, &Rect { xl: ap.point.0, yl: ap.point.1, xh: ap.point.0, yh: ap.point.1 }) {
                    patterns.push(dap);
                }
            }
            pin_idx += 1;
        }
        net.pins.push(DrPin { term: Some(t), id: *pin_cnt, patterns });
        *pin_cnt += 1;
    }
}

/// Per net, how many of the worker's pins lie in the box of its pins' (chosen, else first)
/// points; a net without a point counts 99999 over the extended box.
fn init_nets_num_pins_in(nets: &mut [DrNet], ext_box: &Rect) {
    let point_of = |p: &DrPin| -> Option<P> { p.patterns.iter().find(|a| a.pin_cost == 0).or(p.patterns.first()).map(|a| a.point) };
    let all: Vec<P> = nets.iter().flat_map(|n| n.pins.iter().filter_map(point_of)).collect();
    for net in nets.iter_mut() {
        let (mut x1, mut x2, mut y1, mut y2) = (ext_box.xh, ext_box.xl, ext_box.yh, ext_box.yl);
        for pin in &net.pins {
            // A pin with no point leaves the point at the origin.
            let pt = point_of(pin).unwrap_or((0, 0));
            x1 = x1.min(pt.0);
            x2 = x2.max(pt.0);
            y1 = y1.min(pt.1);
            y2 = y2.max(pt.1);
        }
        if x1 <= x2 && y1 <= y2 {
            let b = Rect { xl: x1, yl: y1, xh: x2, yh: y2 };
            net.num_pins_in = all.iter().filter(|p| touches(&b, &Rect { xl: p.0, yl: p.1, xh: p.0, yh: p.1 })).count();
            net.pin_box = b;
        } else {
            net.num_pins_in = 99999;
            net.pin_box = *ext_box;
        }
    }
}

/// The routing settings a worker's grid reads.
#[derive(Debug, Clone, Copy)]
pub struct GridConfig {
    pub bottom_routing_layer: usize,
    pub top_routing_layer: usize,
}

/// The layer next to `l` routed across it (two up if within the top routing layer, else two
/// down if within the bottom one).
pub fn non_pref_layer(tech: &crate::tech::Tech, cfg: &GridConfig, l: usize) -> Option<usize> {
    let h = tech.layers[l].is_horizontal();
    if l + 2 <= cfg.top_routing_layer && l + 2 < tech.layers.len() && tech.layers[l + 2].is_horizontal() != h && tech.layers[l + 2].dir != crate::tech::Dir::None {
        return Some(l + 2);
    }
    if l >= 2 && l - 2 >= cfg.bottom_routing_layer && tech.layers[l - 2].is_horizontal() != h && tech.layers[l - 2].dir != crate::tech::Dir::None {
        return Some(l - 2);
    }
    None
}

/// A worker's grid: the x and y coordinates (sorted, distinct) and the routing layers (up to the
/// top routing layer). Coordinates: every routing layer's PREFERRED-direction tracks inside the
/// extended box (low side included, high side not), the route and extended boxes' sides, and each
/// access point's coordinate across its layer on every layer from it to the next one routed
/// across (clamped into the routing layers).
pub fn grid_coords(tech: &crate::tech::Tech, tracks: &[crate::tech::TrackPattern], cfg: &GridConfig, route_box: &Rect, ext_box: &Rect, nets: &[DrNet]) -> (Vec<i32>, Vec<i32>, Vec<usize>) {
    let mut xs: BTreeSet<i32> = BTreeSet::new();
    let mut ys: BTreeSet<i32> = BTreeSet::new();
    let mut zs: Vec<usize> = Vec::new();
    for (l, layer) in tech.layers.iter().enumerate() {
        if layer.kind != crate::tech::LayerKind::Routing || l > cfg.top_routing_layer {
            continue;
        }
        for tp in tracks.iter().filter(|t| t.layer == l) {
            // ⚠️ With non-preferred tracks allowed, only the preferred-direction patterns.
            let pref = (tp.vertical_tracks && layer.dir == crate::tech::Dir::Vertical) || (!tp.vertical_tracks && layer.dir == crate::tech::Dir::Horizontal);
            if !pref {
                continue;
            }
            let (lo, hi) = if tp.vertical_tracks { (ext_box.xl, ext_box.xh) } else { (ext_box.yl, ext_box.yh) };
            let mut k = ((lo - tp.start) / tp.spacing).max(0);
            if k * tp.spacing + tp.start < lo {
                k += 1;
            }
            while k < tp.num && k * tp.spacing + tp.start < hi {
                let c = k * tp.spacing + tp.start;
                if tp.vertical_tracks {
                    xs.insert(c);
                } else {
                    ys.insert(c);
                }
                k += 1;
            }
        }
        zs.push(l);
    }
    for b in [route_box, ext_box] {
        xs.insert(b.xl);
        xs.insert(b.xh);
        ys.insert(b.yl);
        ys.insert(b.yh);
    }
    for net in nets {
        for pin in &net.pins {
            for ap in &pin.patterns {
                let mut l = ap.layer;
                let end = if l < cfg.bottom_routing_layer {
                    cfg.bottom_routing_layer
                } else if l > cfg.top_routing_layer {
                    cfg.top_routing_layer
                } else {
                    non_pref_layer(tech, cfg, l).unwrap_or(l)
                };
                loop {
                    if tech.layers[l].is_horizontal() {
                        ys.insert(ap.point.1);
                    } else {
                        xs.insert(ap.point.0);
                    }
                    if end > l {
                        l += 2;
                    } else if end < l {
                        l -= 2;
                    } else {
                        break;
                    }
                }
            }
        }
    }
    (xs.into_iter().collect(), ys.into_iter().collect(), zs)
}

/// The margin around a worker's route box: 2000, or the largest spacing any non-default rule
/// sets on any layer when larger.
pub fn mt_safe_dist(ndrs: &[&crate::dr::rules::NdrRule]) -> i32 {
    ndrs.iter().flat_map(|n| n.spacings.iter().copied()).fold(2000, i32::max)
}
