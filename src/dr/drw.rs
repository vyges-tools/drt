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
type NetPart = (Vec<usize>, Vec<(P, usize)>, Vec<crate::dr::cost::DrFig>);

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
    worker_groups(grid, size, offset, mt_safe, drc_safe).into_iter().flatten().collect()
}

/// The workers of one iteration as its four checkerboard groups (each written back before the
/// next group starts).
pub fn worker_groups(grid: &GCellGrid, size: i32, offset: i32, mt_safe: i32, drc_safe: i32) -> Vec<Vec<WorkerBoxes>> {
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
    groups
}

/// The workers a batch holds at most.
pub const BATCH_SIZE: usize = 1024;

/// The workers of one iteration as the batches they run in: each checkerboard group cut, in
/// creation order, into runs of at most `batch_size`. A batch initialises from the design as the
/// batches before it (in its group too) left it — all its workers route, then all write back.
pub fn worker_batches(groups: Vec<Vec<WorkerBoxes>>, batch_size: usize) -> Vec<Vec<WorkerBoxes>> {
    groups.into_iter().flat_map(|g| g.chunks(batch_size).map(<[WorkerBoxes]>::to_vec).collect::<Vec<_>>()).collect()
}

/// An access point as a worker reads it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DrAp {
    pub point: P,
    pub layer: usize,
    /// The access bits as the database stores them (north, south, east, west, up, down → 1, 2,
    /// 4, 8, 16, 32).
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
    /// Committed shapes of the net around the route box (kept as they are while it reroutes).
    pub ext: Vec<crate::dr::cost::DrFig>,
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
    /// The committed routes (none before the first worker writes back).
    pub routes: Option<(&'a crate::tech::Tech, &'a crate::dr::design::DesignRoutes)>,
    /// A net's terminals with a pin shape at the point on the layer.
    pub term_at: Option<&'a dyn Fn(P, usize, usize) -> Vec<usize>>,
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
    // Committed shapes in the extended box: each net's parts inside the route box (dropped — the
    // first iteration rips everything up) and outside it (kept, as the net's ext shapes).
    let mut net_route: BTreeMap<usize, Vec<crate::dr::cost::DrFig>> = BTreeMap::new();
    let mut net_ext: BTreeMap<usize, Vec<crate::dr::cost::DrFig>> = BTreeMap::new();
    if let Some((tech, d)) = inp.routes {
        for k in d.query(tech, ext_box) {
            let sh = d.shapes[k].as_ref().expect("a shape");
            nets.insert(sh.net);
            let (r, e) = split_obj(route_box, &sh.fig);
            net_route.entry(sh.net).or_default().extend(r);
            net_ext.entry(sh.net).or_default().extend(e);
        }
    }
    for (&net, objs) in &mut net_route {
        let ext = net_ext.entry(net).or_default();
        for f in std::mem::take(objs) {
            if let crate::dr::cost::DrFig::Seg { layer, begin, end, begin_trunc, end_trunc, .. } = f {
                let on_border = seg_on_border(route_box, begin, end);
                if in_box(route_box, begin) && in_box(route_box, end) && (!on_border || (begin_trunc && end_trunc)) {
                    if on_border {
                        if let Some(term_at) = inp.term_at {
                            for p in [begin, end] {
                                for t in term_at(p, layer, net) {
                                    net_terms.entry(net).or_default().insert((term_key(t), t));
                                }
                            }
                        }
                    }
                } else {
                    ext.push(f);
                }
            }
        }
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
        let ext = net_ext.remove(&net).unwrap_or_default();
        for (part_terms, part_bounds, part_ext) in init_nets_init_dr_helper(inp, &terms, &g, &bounds, ext) {
            let id = out.len();
            let mut dnet = DrNet { id, net, pins: Vec::new(), num_pins_in: 0, pin_box: *ext_box, ext: part_ext };
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
    if let Some((tech, _)) = inp.routes {
        init_nets_boundary_area(tech, route_box, &mut out);
    }
    out
}

/// Each boundary point's access area: the net's committed wires that start (or end) there and
/// leave the route box, length × width; plus half the box of a via of the net (a patch's whole
/// box) found AT THE POINT whose origin is the wire's far end (⛔ the reference searches the
/// shapes at the point, not at the far end — so this adds only when the far end is the point).
fn init_nets_boundary_area(tech: &crate::tech::Tech, rb: &Rect, nets: &mut [DrNet]) {
    use crate::dr::cost::DrFig;
    for net in nets.iter_mut() {
        let ext = net.ext.clone();
        for pin in net.pins.iter_mut() {
            if pin.term.is_some() {
                continue;
            }
            for ap in pin.patterns.iter_mut() {
                let (bp, l) = (ap.point, ap.layer);
                let q = Rect { xl: bp.0, yl: bp.1, xh: bp.0, yh: bp.1 };
                // The net's shapes at the point on the layer, with their boxes there.
                let mut here: Vec<(&DrFig, Rect)> = Vec::new();
                for f in &ext {
                    match f {
                        DrFig::Via { via, origin, .. } => {
                            let vd = &tech.via_defs[*via];
                            let figs = if vd.layer1 == l { &vd.layer1_figs } else if vd.layer2 == l { &vd.layer2_figs } else if vd.cut == l { &vd.cut_figs } else { continue };
                            for r in figs {
                                let b = Rect { xl: r.xl + origin.0, yl: r.yl + origin.1, xh: r.xh + origin.0, yh: r.yh + origin.1 };
                                if touches(&b, &q) {
                                    here.push((f, b));
                                }
                            }
                        }
                        _ => {
                            let (fl, b) = crate::dr::design::stored_box(tech, f);
                            if fl == l && touches(&b, &q) {
                                here.push((f, b));
                            }
                        }
                    }
                }
                let mut area: i64 = 0;
                for &(f, _) in &here {
                    let DrFig::Seg { begin: psb, end: pse, width, .. } = *f else { continue };
                    let len = i64::from((pse.0 - psb.0).abs() + (pse.1 - psb.1).abs());
                    let far = if bp == psb && !in_box(rb, pse) {
                        Some(pse)
                    } else if !in_box(rb, psb) && bp == pse {
                        Some(psb)
                    } else {
                        None
                    };
                    let Some(far) = far else { continue };
                    area += len * i64::from(width);
                    for &(g, b) in &here {
                        match g {
                            DrFig::Via { origin, .. } if *origin == far => {
                                area += i64::from(b.dx()) * i64::from(b.dy()) / 2;
                                break;
                            }
                            DrFig::Patch { origin, .. } if *origin == far => {
                                area += i64::from(b.dx()) * i64::from(b.dy());
                                break;
                            }
                            _ => {}
                        }
                    }
                }
                ap.begin_area = area;
            }
        }
    }
}

/// A net's parts: its guides and terminals (terminals' boxes) that touch, depth first; parts
/// without a guide dropped. One part (or none): everything. Several: each terminal to the part
/// nearest it (its own part first), each boundary point to the part whose guide is nearest (plus
/// the layer difference).
fn init_nets_init_dr_helper(inp: &DrNetInput<'_>, terms: &[usize], guides: &[(usize, Rect)], bounds: &[(P, usize)], ext: Vec<crate::dr::cost::DrFig>) -> Vec<NetPart> {
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
        return vec![(terms.to_vec(), bounds.to_vec(), ext)];
    }
    let mut out: Vec<NetPart> = vec![(Vec::new(), Vec::new(), Vec::new()); comps.len()];
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
    // Each ext shape to the part whose guide it lies on (same layer, touching), else the nearest.
    for f in ext {
        if let Some(j) = obj_component(inp, &f, &comps, guides, n_guides) {
            out[j].2.push(f);
        }
    }
    out
}

fn obj_component(inp: &DrNetInput<'_>, f: &crate::dr::cost::DrFig, comps: &[Vec<usize>], guides: &[(usize, Rect)], n_guides: usize) -> Option<usize> {
    let (tech, _) = inp.routes?;
    let (layer, rect, same_layer_hit) = match f {
        crate::dr::cost::DrFig::Seg { layer, .. } | crate::dr::cost::DrFig::Patch { layer, .. } => (*layer, crate::dr::design::stored_box(tech, f).1, true),
        crate::dr::cost::DrFig::Via { .. } => (usize::MAX, crate::dr::design::stored_box(tech, f).1, false),
    };
    let (mut best_d, mut best) = (i64::MAX, None);
    for (j, comp) in comps.iter().enumerate() {
        for &k in comp {
            if k >= n_guides {
                continue;
            }
            if same_layer_hit && guides[k].0 == layer && touches(&guides[k].1, &rect) {
                return Some(j);
            }
            let d = sq_dist(&guides[k].1, &rect);
            if d < best_d {
                if !same_layer_hit && d == 0 {
                    return Some(j);
                }
                best_d = d;
                best = Some(j);
            }
        }
    }
    best
}

fn in_box(r: &Rect, p: P) -> bool {
    p.0 >= r.xl && p.0 <= r.xh && p.1 >= r.yl && p.1 <= r.yh
}

fn seg_on_border(r: &Rect, b: P, e: P) -> bool {
    if b.0 == e.0 {
        b.0 == r.xl || b.0 == r.xh
    } else {
        b.1 == r.yl || b.1 == r.yh
    }
}

/// A committed shape against the route box (the first iteration): a wire across it or on its
/// low sides split into the parts before, inside and after it (a part that reaches into the box
/// is a route part, its cut ends extending), one along the far side of it or wholly across its
/// line outside it ext; a via or patch a route part when its origin is strictly inside.
fn split_obj(rb: &Rect, f: &crate::dr::cost::DrFig) -> (Vec<crate::dr::cost::DrFig>, Vec<crate::dr::cost::DrFig>) {
    use crate::dr::cost::DrFig;
    let (mut route, mut ext) = (Vec::new(), Vec::new());
    match *f {
        DrFig::Seg { begin, end, .. } => {
            let vertical = begin.0 == end.0;
            let (c, clo, chi) = if vertical { (begin.0, rb.xl, rb.xh) } else { (begin.1, rb.yl, rb.yh) };
            if c <= clo || chi <= c {
                ext.push(f.clone());
                return (route, ext);
            }
            let (bc, ec, bmin, bmax) = if vertical { (begin.1, end.1, rb.yl, rb.yh) } else { (begin.0, end.0, rb.xl, rb.xh) };
            let part = |lo: i32, hi: i32, ext_begin: bool, ext_end: bool| -> DrFig {
                let mut g = f.clone();
                if let DrFig::Seg { begin: b, end: e, begin_trunc, end_trunc, .. } = &mut g {
                    *b = if vertical { (c, lo) } else { (lo, c) };
                    *e = if vertical { (c, hi) } else { (hi, c) };
                    if ext_begin {
                        *begin_trunc = false;
                    }
                    if ext_end {
                        *end_trunc = false;
                    }
                }
                g
            };
            if bc < bmin {
                let ne = ec.min(bmin);
                if ec < bmin {
                    ext.push(part(bc, ne, false, false));
                } else {
                    route.push(part(bc, ne, false, ec != bmin));
                }
            }
            if bc < bmax && ec > bmin {
                route.push(part(bc.max(bmin), ec.min(bmax), bc < bmin, ec > bmax));
            }
            if ec > bmax {
                let nb = bc.max(bmax);
                if bc > bmax {
                    ext.push(part(nb, ec, false, false));
                } else {
                    route.push(part(nb, ec, bc != bmax, false));
                }
            }
        }
        DrFig::Via { origin, .. } | DrFig::Patch { origin, .. } => {
            if origin.0 > rb.xl && origin.0 < rb.xh && origin.1 > rb.yl && origin.1 < rb.yh {
                route.push(f.clone());
            } else {
                ext.push(f.clone());
            }
        }
    }
    (route, ext)
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

/// Per layer, per coordinate: whether it is a real track (`true`) or added for an access point, a
/// route or a box side (`false`). Layer `None` holds the route and extended boxes' sides.
pub type CoordMaps = BTreeMap<Option<usize>, BTreeMap<i32, bool>>;

/// A worker's grid coordinates: the x and y maps (see [`CoordMaps`]), and the routing layers (up
/// to the top routing layer). Added in order — a later add of a real track overrides: each
/// access point's coordinate across its layer on every layer from it to the next one routed
/// across (clamped into the routing layers), the route and extended boxes' sides, then every
/// routing layer's PREFERRED-direction tracks inside the extended box (low side included, high
/// side not).
pub fn grid_maps(tech: &crate::tech::Tech, tracks: &[crate::tech::TrackPattern], cfg: &GridConfig, route_box: &Rect, ext_box: &Rect, nets: &[DrNet]) -> (CoordMaps, CoordMaps, Vec<usize>) {
    let mut xm: CoordMaps = BTreeMap::new();
    let mut ym: CoordMaps = BTreeMap::new();
    for b in [route_box, ext_box] {
        for c in [b.xl, b.xh] {
            xm.entry(None).or_default().insert(c, false);
        }
        for c in [b.yl, b.yh] {
            ym.entry(None).or_default().insert(c, false);
        }
    }
    for net in nets {
        // The net's committed shapes: a wire's ends and line, each via's point on its layers.
        for f in &net.ext {
            match *f {
                crate::dr::cost::DrFig::Seg { layer: l, begin, end, .. } => {
                    let l2 = non_pref_layer(tech, cfg, l).unwrap_or(l);
                    if begin.0 == end.0 {
                        let (lx, ly) = if tech.layers[l].is_horizontal() { (l2, l) } else { (l, l2) };
                        xm.entry(Some(lx)).or_default().insert(begin.0, false);
                        ym.entry(Some(ly)).or_default().insert(begin.1, false);
                        ym.entry(Some(ly)).or_default().insert(end.1, false);
                    } else {
                        let (lx, ly) = if tech.layers[l].is_vertical() { (l, l2) } else { (l2, l) };
                        xm.entry(Some(lx)).or_default().insert(begin.0, false);
                        xm.entry(Some(lx)).or_default().insert(end.0, false);
                        ym.entry(Some(ly)).or_default().insert(begin.1, false);
                    }
                }
                crate::dr::cost::DrFig::Via { via, origin, .. } => {
                    let vd = &tech.via_defs[via];
                    for l in [vd.layer1, vd.layer2] {
                        if tech.layers[l].is_horizontal() {
                            ym.entry(Some(l)).or_default().insert(origin.1, false);
                        } else {
                            xm.entry(Some(l)).or_default().insert(origin.0, false);
                        }
                    }
                }
                crate::dr::cost::DrFig::Patch { .. } => {}
            }
        }
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
                        ym.entry(Some(l)).or_default().insert(ap.point.1, false);
                    } else {
                        xm.entry(Some(l)).or_default().insert(ap.point.0, false);
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
                    xm.entry(Some(l)).or_default().insert(c, true);
                } else {
                    ym.entry(Some(l)).or_default().insert(c, true);
                }
                k += 1;
            }
        }
        zs.push(l);
    }
    (xm, ym, zs)
}

/// The sorted, distinct coordinates of all layers' maps.
pub fn coords(m: &CoordMaps) -> Vec<i32> {
    let s: BTreeSet<i32> = m.values().flat_map(|v| v.keys().copied()).collect();
    s.into_iter().collect()
}

/// [`grid_maps`], flattened: the x and y coordinates and the layers.
pub fn grid_coords(tech: &crate::tech::Tech, tracks: &[crate::tech::TrackPattern], cfg: &GridConfig, route_box: &Rect, ext_box: &Rect, nets: &[DrNet]) -> (Vec<i32>, Vec<i32>, Vec<usize>) {
    let (xm, ym, zs) = grid_maps(tech, tracks, cfg, route_box, ext_box, nets);
    (coords(&xm), coords(&ym), zs)
}

/// A grid node: its edges east, north and up; whether each is blocked, off-track (a "grid
/// cost") or an access point's (an "ap cost"); a special via at it; and its costs — each an
/// 8-bit count that saturates at 255 and floors at 0 (the adjacent-node getters read them).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Node {
    pub east: bool,
    pub north: bool,
    pub up: bool,
    pub blocked_e: bool,
    pub blocked_n: bool,
    pub blocked_u: bool,
    pub svia: bool,
    pub override_via: bool,
    pub grid_cost_e: bool,
    pub grid_cost_n: bool,
    pub grid_cost_u: bool,
    pub ap_cost_e: bool,
    pub ap_cost_n: bool,
    pub ap_cost_u: bool,
    pub route_planar: u8,
    pub route_via: u8,
    pub marker_planar: u8,
    pub marker_via: u8,
    pub fixed_via: u8,
    pub fixed_h: u8,
    pub fixed_v: u8,
    pub route_planar_ndr: u8,
    pub route_via_ndr: u8,
    pub fixed_via_ndr: u8,
    pub fixed_h_ndr: u8,
    pub fixed_v_ndr: u8,
}

/// A worker's grid graph.
#[derive(Debug, Clone)]
pub struct GridGraph {
    pub xs: Vec<i32>,
    pub ys: Vec<i32>,
    /// The routing layer of each z.
    pub zs: Vec<usize>,
    pub nodes: Vec<Node>,
}

impl GridGraph {
    pub fn idx(&self, x: usize, y: usize, z: usize) -> usize {
        (z * self.ys.len() + y) * self.xs.len() + x
    }
}

/// The edges: per layer (z), its preferred-direction edges along each of its own tracks (off-track
/// coordinates and the route box's sides costed), up-vias where the layer two above has a
/// coordinate (costed when either is off-track) unless the default via there would leave the
/// die, and non-preferred edges along the coordinates of the layer routed across it (all
/// costed). An edge exists only with both ends inside the ROUTE box.
/// ⚠️ An off-track cost bit is set whether or not its edge was added (outside the route box it
/// was not).
#[allow(clippy::too_many_arguments)]
pub fn init_edges(tech: &crate::tech::Tech, defaults: &[Option<usize>], cfg: &GridConfig, xm: &CoordMaps, ym: &CoordMaps, zs: &[usize], route_box: &Rect, die: &Rect) -> GridGraph {
    let (xs, ys) = (coords(xm), coords(ym));
    let mut g = GridGraph { nodes: vec![Node::default(); xs.len() * ys.len() * zs.len()], xs, ys, zs: zs.to_vec() };
    let empty: BTreeMap<i32, bool> = BTreeMap::new();
    let map = |m: &'_ CoordMaps, l: usize| -> BTreeMap<i32, bool> { m.get(&Some(l)).cloned().unwrap_or_else(|| empty.clone()) };
    let in_box = |p: P| route_box.xl <= p.0 && p.0 <= route_box.xh && route_box.yl <= p.1 && p.1 <= route_box.yh;
    let out_of_die_via = |g: &GridGraph, x: usize, y: usize, l: usize| -> bool {
        if l + 1 >= tech.layers.len() {
            return false;
        }
        let Some(v) = defaults.get(l + 1).copied().flatten() else { return true };
        let vd = &tech.via_defs[v];
        let (b1, b2) = (vd.layer1_bbox(), vd.layer2_bbox());
        let (px, py) = (g.xs[x], g.ys[y]);
        let b = Rect { xl: b1.xl.min(b2.xl) + px, yl: b1.yl.min(b2.yl) + py, xh: b1.xh.max(b2.xh) + px, yh: b1.yh.max(b2.yh) + py };
        !(die.xl <= b.xl && die.yl <= b.yl && b.xh <= die.xh && b.yh <= die.yh)
    };
    let (nx, ny) = (g.xs.len(), g.ys.len());
    for (z, &l) in zs.iter().enumerate() {
        let non_pref = if l + 2 <= cfg.top_routing_layer { l + 2 } else if l >= 2 { l - 2 } else { l };
        let in_range = l >= cfg.bottom_routing_layer && l <= cfg.top_routing_layer;
        let horizontal = tech.layers[l].is_horizontal();
        let (own, up2, np) = if horizontal { (map(ym, l), map(xm, l + 2), map(xm, non_pref)) } else { (map(xm, l), map(ym, l + 2), map(ym, non_pref)) };
        // Preferred edges and up-vias.
        let (outer, inner) = if horizontal { (ny, nx) } else { (nx, ny) };
        for o in 0..outer {
            let oc = if horizontal { g.ys[o] } else { g.xs[o] };
            let Some(&track) = own.get(&oc) else { continue };
            for i in 0..inner {
                let (x, y) = if horizontal { (i, o) } else { (o, i) };
                let ood = out_of_die_via(&g, x, y, l);
                if in_range {
                    // (Leaving the die only matters on a unidirectional layer, which is refused.)
                    let (x2, y2) = if horizontal { (x + 1, y) } else { (x, y + 1) };
                    let added = x2 < nx && y2 < ny && in_box((g.xs[x], g.ys[y])) && in_box((g.xs[x2], g.ys[y2]));
                    let border = if horizontal { oc == route_box.yl || oc == route_box.yh } else { oc == route_box.xl || oc == route_box.xh };
                    let k = g.idx(x, y, z);
                    if horizontal {
                        g.nodes[k].east |= added;
                        g.nodes[k].grid_cost_e |= !track || border;
                    } else {
                        g.nodes[k].north |= added;
                        g.nodes[k].grid_cost_n |= !track || border;
                    }
                }
                if ood {
                    continue;
                }
                let ic = if horizontal { g.xs[x] } else { g.ys[y] };
                let Some(&track2) = up2.get(&ic) else { continue };
                let k = g.idx(x, y, z);
                g.nodes[k].up |= z + 1 < zs.len() && in_box((g.xs[x], g.ys[y]));
                g.nodes[k].grid_cost_u |= !(track && track2);
            }
        }
        // Non-preferred edges along the coordinates of the layer routed across.
        if in_range {
            for i in 0..inner {
                let ic = if horizontal { g.xs[i] } else { g.ys[i] };
                if !np.contains_key(&ic) {
                    continue;
                }
                for o in 0..outer {
                    let (x, y) = if horizontal { (i, o) } else { (o, i) };
                    let (x2, y2) = if horizontal { (x, y + 1) } else { (x + 1, y) };
                    let added = x2 < nx && y2 < ny && in_box((g.xs[x], g.ys[y])) && in_box((g.xs[x2], g.ys[y2]));
                    let k = g.idx(x, y, z);
                    if horizontal {
                        g.nodes[k].north |= added;
                        g.nodes[k].grid_cost_n = true;
                    } else {
                        g.nodes[k].east |= added;
                        g.nodes[k].grid_cost_e = true;
                    }
                }
            }
        }
    }
    g
}

/// The margin around a worker's route box: 2000, or the largest spacing any non-default rule
/// sets on any layer when larger.
pub fn mt_safe_dist(ndrs: &[&crate::dr::rules::NdrRule]) -> i32 {
    ndrs.iter().flat_map(|n| n.spacings.iter().copied()).fold(2000, i32::max)
}

/// Each net's committed shapes placed on the worker's grid: a wire's ends (clamped into the
/// extended box), a via's point on its two layers. A shape whose point the grid lacks keeps
/// index 0 there.
pub fn localize_ext(tech: &crate::tech::Tech, g: &GridGraph, ext_box: &Rect, nets: &mut [DrNet]) {
    let ix = |v: &[i32], c: i32| v.binary_search(&c).unwrap_or(0);
    for n in nets.iter_mut() {
        for f in n.ext.iter_mut() {
            match f {
                crate::dr::cost::DrFig::Seg { layer, begin, end, bi, ei, .. } => {
                    let b = (begin.0.max(ext_box.xl), begin.1.max(ext_box.yl));
                    let e = (end.0.min(ext_box.xh), end.1.min(ext_box.yh));
                    let z = g.zs.iter().position(|&l| l == *layer).unwrap_or(0);
                    *bi = (ix(&g.xs, b.0), ix(&g.ys, b.1), z);
                    *ei = (ix(&g.xs, e.0), ix(&g.ys, e.1), z);
                }
                crate::dr::cost::DrFig::Via { via, origin, bi, ei, .. } => {
                    let vd = &tech.via_defs[*via];
                    let z1 = g.zs.iter().position(|&l| l == vd.layer1).unwrap_or(0);
                    let z2 = g.zs.iter().position(|&l| l == vd.layer2).unwrap_or(0);
                    *bi = (ix(&g.xs, origin.0), ix(&g.ys, origin.1), z1);
                    *ei = (ix(&g.xs, origin.0), ix(&g.ys, origin.1), z2);
                }
                crate::dr::cost::DrFig::Patch { .. } => {}
            }
        }
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    fn wb(i: i32) -> WorkerBoxes {
        let r = Rect { xl: i, yl: 0, xh: i + 1, yh: 1 };
        WorkerBoxes { start: (i, 0), route: r, ext: r, drc: r }
    }

    /// Rule: a checkerboard group larger than the batch size runs as several batches, cut in
    /// creation order; groups never share a batch.
    #[test]
    fn a_group_runs_in_batches_of_at_most_the_batch_size() {
        let groups = vec![(0..5).map(wb).collect::<Vec<_>>(), vec![wb(9)]];
        let starts: Vec<Vec<i32>> = worker_batches(groups, 2).iter().map(|b| b.iter().map(|w| w.start.0).collect()).collect();
        assert_eq!(starts, vec![vec![0, 1], vec![2, 3], vec![4], vec![9]]);
    }
}
