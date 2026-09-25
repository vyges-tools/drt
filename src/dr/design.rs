// SPDX-License-Identifier: Apache-2.0
//! The routes committed so far and the markers standing, as later workers read them; and a
//! worker's write-back.
//!
//! Rules (the first iteration's write-back):
//! - every net the worker routed has its shapes in the worker's extended box removed and its
//!   best shapes added; a wire crossing the route box keeps its parts outside (the crossing end
//!   now extends), and the points where it crossed are BOUNDARY POINTS; a wire along the route
//!   box's own side (x on its left or right for a vertical wire, y on its bottom or top for a
//!   horizontal one) is kept whole; a via or patch is removed when its origin lies STRICTLY inside
//!   the route box in the first iteration (on its edge it stays, as the worker counted it ext),
//!   inside or on it afterwards;
//! - at each boundary point not on a pin of the net (or off the manufacturing grid), exactly two
//!   of the net's wires of one direction through it (none of the other, no patch there, both
//!   tapered or neither) merge into one, each outer end keeping its style;
//! - the markers in the worker's check box are replaced by its best markers that touch it.
//!
//! The design keeps its committed shapes and its markers each in a region query (per layer; a
//! via on its cut layer), updated insert by insert and removal by removal in the order the
//! write-back makes them: a query's ORDER is what later readers see (a worker's objects and
//! markers, which of two wires a merge keeps).
//! - removal: the shapes of the modified nets in the extended box, in query order (a crossing
//!   wire's outside parts added before the wire is removed);
//! - addition: the routed nets' best shapes, in worker net order;
//! - merges: per net (by index), per boundary point (by point, then layer): the two wires in
//!   query order — the merged wire is a copy of the FIRST — removed, the merged one added;
//! - markers: those in the check box removed in query order, the new ones added in order.

use std::collections::{BTreeMap, BTreeSet};

use crate::dr::conn::{self, CheckLog, NetShapes, Op, Ref};
use crate::dr::cost::DrFig;
use crate::gc::{Marker, Owner, Rule};
use crate::polygon90::Rect;
use crate::rtree::DynRTree;
use crate::tech::Tech;

type P = (i32, i32);

/// A committed shape: its net (routing index), the shape.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Shape {
    pub net: usize,
    pub fig: DrFig,
}

/// The committed routes (slots in the order they were written, removed ones empty) and markers,
/// each with its region query.
#[derive(Default)]
pub struct DesignRoutes {
    pub shapes: Vec<Option<Shape>>,
    /// Each live shape's (layer, id) in its layer's query.
    rq: Vec<Option<(usize, usize)>>,
    trees: Vec<DynRTree<usize>>,
    /// The markers in the order they were added (removed ones empty).
    marker_slots: Vec<Option<Marker>>,
    marker_rq: Vec<Option<(usize, usize)>>,
    marker_trees: Vec<DynRTree<usize>>,
    /// The nets a write-back changed since the last connectivity check.
    pub modified: BTreeSet<usize>,
}

/// A shape's box as the region query stores it: a wire's (ends extended), a via's over its three
/// layers, a patch's; and the layer it is stored on (a via: its cut layer).
pub fn stored_box(tech: &Tech, fig: &DrFig) -> (usize, Rect) {
    match *fig {
        DrFig::Seg { layer, begin, end, width, begin_ext, end_ext, .. } => (layer, DrFig::seg_box(begin, end, width, begin_ext, end_ext)),
        DrFig::Via { via, origin, .. } => {
            let vd = &tech.via_defs[via];
            let all: Vec<Rect> = vd.layer1_figs.iter().chain(&vd.cut_figs).chain(&vd.layer2_figs).copied().collect();
            let b = all.iter().skip(1).fold(all[0], |a, f| Rect { xl: a.xl.min(f.xl), yl: a.yl.min(f.yl), xh: a.xh.max(f.xh), yh: a.yh.max(f.yh) });
            (vd.cut, Rect { xl: b.xl + origin.0, yl: b.yl + origin.1, xh: b.xh + origin.0, yh: b.yh + origin.1 })
        }
        DrFig::Patch { layer, origin, offset } => (layer, Rect { xl: offset.xl + origin.0, yl: offset.yl + origin.1, xh: offset.xh + origin.0, yh: offset.yh + origin.1 }),
    }
}

fn touches(a: &Rect, b: &Rect) -> bool {
    a.xh >= b.xl && a.xl <= b.xh && a.yh >= b.yl && a.yl <= b.yh
}

fn in_box(r: &Rect, p: P) -> bool {
    p.0 >= r.xl && p.0 <= r.xh && p.1 >= r.yl && p.1 <= r.yh
}

fn tree_at<T>(trees: &mut Vec<DynRTree<T>>, l: usize) -> &mut DynRTree<T> {
    while trees.len() <= l {
        trees.push(DynRTree::new(Vec::new()));
    }
    &mut trees[l]
}

impl DesignRoutes {
    /// The shapes whose stored box touches `b`: layer by layer (ascending), each in its query's
    /// order.
    pub fn query(&self, _tech: &Tech, b: &Rect) -> Vec<usize> {
        self.trees.iter().flat_map(|t| t.query(b).into_iter().map(|(_, v)| v.1)).collect()
    }

    /// The shapes stored on layer `l` whose box touches `b`, in query order.
    pub fn query_layer(&self, b: &Rect, l: usize) -> Vec<usize> {
        self.trees.get(l).map_or(Vec::new(), |t| t.query(b).into_iter().map(|(_, v)| v.1).collect())
    }

    /// Commit a shape (the end of the net's list and of the writing order); its slot.
    pub fn add(&mut self, tech: &Tech, net: usize, fig: DrFig) -> usize {
        let k = self.shapes.len();
        let (l, b) = stored_box(tech, &fig);
        let id = tree_at(&mut self.trees, l).insert(b, k);
        self.shapes.push(Some(Shape { net, fig }));
        self.rq.push(Some((l, id)));
        k
    }

    /// Remove a committed shape.
    pub fn remove(&mut self, k: usize) {
        if let Some((l, id)) = self.rq[k].take() {
            self.trees[l].remove(id);
        }
        self.shapes[k] = None;
    }

    /// The markers whose box touches `b`: layer by layer, each in its query's order.
    pub fn markers_in(&self, b: &Rect) -> Vec<Marker> {
        self.marker_ids_in(b).into_iter().map(|k| self.marker_slots[k].clone().expect("a live marker")).collect()
    }

    fn marker_ids_in(&self, b: &Rect) -> Vec<usize> {
        self.marker_trees.iter().flat_map(|t| t.query(b).into_iter().map(|(_, v)| v.1)).collect()
    }

    /// Every standing marker, in the order added.
    pub fn markers(&self) -> impl Iterator<Item = &Marker> {
        self.marker_slots.iter().flatten()
    }

    /// Add a marker (the end of the design's list).
    pub fn add_marker(&mut self, m: Marker) {
        let (k, l) = (self.marker_slots.len(), m.layer);
        let id = tree_at(&mut self.marker_trees, l).insert(m.bbox, k);
        self.marker_slots.push(Some(m));
        self.marker_rq.push(Some((l, id)));
    }

    /// Take a shape out of the region query (it stays in its net's list).
    fn unindex(&mut self, k: usize) {
        if let Some((l, id)) = self.rq[k].take() {
            self.trees[l].remove(id);
        }
    }

    /// Put a shape back in the region query with new geometry.
    fn reindex(&mut self, tech: &Tech, k: usize, fig: DrFig) {
        let (l, b) = stored_box(tech, &fig);
        let id = tree_at(&mut self.trees, l).insert(b, k);
        self.rq[k] = Some((l, id));
        if let Some(sh) = self.shapes[k].as_mut() {
            sh.fig = fig;
        }
    }

    /// Append a shape to its net's list without indexing it yet; its slot.
    fn append_unindexed(&mut self, net: usize, fig: DrFig) -> usize {
        self.shapes.push(Some(Shape { net, fig }));
        self.rq.push(None);
        self.shapes.len() - 1
    }

    fn remove_marker(&mut self, k: usize) {
        if let Some((l, id)) = self.marker_rq[k].take() {
            self.marker_trees[l].remove(id);
        }
        self.marker_slots[k] = None;
    }
}

/// What a worker writes back (the first iteration).
pub struct WriteBack<'a> {
    pub route_box: Rect,
    pub ext_box: Rect,
    pub drc_box: Rect,
    /// The nets it routed, with their best shapes (worker net order).
    pub routed: &'a [(usize, Vec<DrFig>)],
    pub markers: &'a [Marker],
    /// Whether a pin shape of `net` lies at the point on the layer.
    pub on_pin: &'a dyn Fn(P, usize, usize) -> bool,
    pub manufacturing_grid: i32,
    /// The first iteration (a via or patch is a route part only strictly inside the box).
    pub init_dr: bool,
}

/// Whether a via or patch at `origin` is the worker's to remove: strictly inside the route box in
/// the first iteration, inside or on it afterwards.
fn is_route_origin(rb: &Rect, origin: P, init_dr: bool) -> bool {
    if init_dr {
        origin.0 > rb.xl && origin.0 < rb.xh && origin.1 > rb.yl && origin.1 < rb.yh
    } else {
        in_box(rb, origin)
    }
}

/// A worker's write-back into the design.
pub fn end(d: &mut DesignRoutes, tech: &Tech, wb: &WriteBack<'_>) {
    let mod_nets: BTreeSet<usize> = wb.routed.iter().map(|(n, _)| *n).collect();
    d.modified.extend(mod_nets.iter().copied());
    let mut bound: std::collections::BTreeMap<usize, BTreeSet<(P, usize)>> = Default::default();
    // Remove the modified nets' shapes in the extended box.
    for k in d.query(tech, &wb.ext_box) {
        let Some(s) = d.shapes[k].clone() else { continue };
        if !mod_nets.contains(&s.net) {
            continue;
        }
        match s.fig {
            DrFig::Seg { .. } => remove_seg(d, tech, k, &wb.route_box, bound.entry(s.net).or_default()),
            DrFig::Via { origin, .. } | DrFig::Patch { origin, .. } => {
                if is_route_origin(&wb.route_box, origin, wb.init_dr) {
                    d.remove(k);
                }
            }
        }
    }
    // Add the best shapes.
    for (net, figs) in wb.routed {
        for f in figs {
            d.add(tech, *net, f.clone());
        }
    }
    for (net, pts) in &bound {
        for &(pt, l) in pts {
            merge_at(d, tech, *net, pt, l, wb);
        }
    }
    // Markers.
    for k in d.marker_ids_in(&wb.drc_box) {
        d.remove_marker(k);
    }
    for m in wb.markers {
        if touches(&m.bbox, &wb.drc_box) {
            d.add_marker(m.clone());
        }
    }
}

/// A committed wire of a net the worker rerouted: kept whole along the route box's sides; else
/// the parts outside the route box kept (the crossing end now extending), the crossing points
/// recorded.
fn remove_seg(d: &mut DesignRoutes, tech: &Tech, k: usize, rb: &Rect, bound: &mut BTreeSet<(P, usize)>) {
    let Some(Shape { net, fig }) = d.shapes[k].clone() else { return };
    let DrFig::Seg { layer, begin, end, .. } = fig else { return };
    let vertical = begin.0 == end.0;
    // Along the box's own sides: kept (the first iteration merges on those boundaries).
    if vertical && (begin.0 == rb.xl || begin.0 == rb.xh) || !vertical && (begin.1 == rb.yl || begin.1 == rb.yh) {
        return;
    }
    let (along, cross, lo, hi, blo, bhi) = if vertical { (begin.0, 0, begin.1, end.1, rb.yl, rb.yh) } else { (begin.1, 1, begin.0, end.0, rb.xl, rb.xh) };
    let (clo, chi) = if vertical { (rb.xl, rb.xh) } else { (rb.yl, rb.yh) };
    let _ = cross;
    if !(clo <= along && along <= chi && lo <= bhi && hi >= blo) {
        return;
    }
    let mk = |b: i32, e: i32| if vertical { ((along, b), (along, e)) } else { ((b, along), (e, along)) };
    // The part below the box.
    if lo < blo {
        let bp = hi.min(blo);
        let (nb, ne) = mk(lo, bp);
        let mut f = fig.clone();
        if let DrFig::Seg { begin, end, end_trunc, .. } = &mut f {
            *begin = nb;
            *end = ne;
            if hi > blo {
                *end_trunc = false;
            }
        }
        d.add(tech, net, f);
        if hi >= blo {
            bound.insert((ne, layer));
        }
    }
    // The part above the box.
    if hi > bhi {
        let bp = lo.max(bhi);
        let (nb, ne) = mk(bp, hi);
        let mut f = fig.clone();
        if let DrFig::Seg { begin, end, begin_trunc, .. } = &mut f {
            *begin = nb;
            *end = ne;
            if lo < bhi {
                *begin_trunc = false;
            }
        }
        d.add(tech, net, f);
        if lo <= bhi {
            bound.insert((nb, layer));
        }
    }
    d.remove(k);
}

/// At a boundary point: two of the net's wires of one direction through it (and none of the
/// other, no patch there) merge.
fn merge_at(d: &mut DesignRoutes, tech: &Tech, net: usize, pt: P, layer: usize, wb: &WriteBack<'_>) {
    let mg = wb.manufacturing_grid.max(1);
    let off_grid = pt.0 % mg != 0 || pt.1 % mg != 0;
    if (wb.on_pin)(pt, layer, net) && !off_grid {
        return;
    }
    let q = Rect { xl: pt.0, yl: pt.1, xh: pt.0, yh: pt.1 };
    let (mut horz, mut vert): (Vec<usize>, Vec<usize>) = (Vec::new(), Vec::new());
    for k in d.query_layer(&q, layer) {
        let Some(s) = &d.shapes[k] else { continue };
        if s.net != net {
            continue;
        }
        match s.fig {
            DrFig::Seg { begin, end, .. } => {
                let on_line = if begin.0 == end.0 { pt.0 == begin.0 && pt.1 >= begin.1 && pt.1 <= end.1 } else { pt.1 == begin.1 && pt.0 >= begin.0 && pt.0 <= end.0 };
                if on_line {
                    if begin.0 == end.0 {
                        vert.push(k);
                    } else {
                        horz.push(k);
                    }
                }
            }
            DrFig::Patch { origin, .. } => {
                if origin == pt {
                    return;
                }
            }
            DrFig::Via { .. } => {}
        }
    }
    for (group, other, is_horz) in [(&horz, &vert, true), (&vert, &horz, false)] {
        if group.len() != 2 || !other.is_empty() {
            continue;
        }
        let (a, b) = (d.shapes[group[0]].clone().expect("a shape"), d.shapes[group[1]].clone().expect("a shape"));
        let (DrFig::Seg { begin: b1, end: e1, tapered: t1, .. }, DrFig::Seg { begin: b2, end: e2, tapered: t2, end_ext: ee2, end_trunc: et2, begin_ext: be2, begin_trunc: bt2, .. }) = (&a.fig, &b.fig) else { continue };
        if t1 != t2 {
            continue;
        }
        let mut f = a.fig.clone();
        if let DrFig::Seg { begin, end, end_ext, end_trunc, begin_ext, begin_trunc, .. } = &mut f {
            if is_horz {
                *begin = (b1.0.min(b2.0), b1.1);
                *end = (e1.0.max(e2.0), e1.1);
                if b1.0 < b2.0 {
                    *end_ext = *ee2;
                    *end_trunc = *et2;
                } else {
                    *begin_ext = *be2;
                    *begin_trunc = *bt2;
                }
            } else {
                *begin = (b1.0, b1.1.min(b2.1));
                *end = (e1.0, e1.1.max(e2.1));
                if b1.1 < b2.1 {
                    *end_ext = *ee2;
                    *end_trunc = *et2;
                } else {
                    *begin_ext = *be2;
                    *begin_trunc = *bt2;
                }
            }
        }
        d.remove(group[0]);
        d.remove(group[1]);
        d.add(tech, net, f);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Rule: in the first iteration a via whose origin lies ON the route box's edge is not the
    /// worker's (it read it as ext), so its write-back keeps it; from the second iteration on the
    /// edge counts as inside.
    #[test]
    fn a_via_on_the_route_box_edge_is_removed_only_after_the_first_iteration() {
        let rb = Rect { xl: 0, yl: 0, xh: 100, yh: 100 };
        for (p, first, later) in [((50, 100), false, true), ((0, 50), false, true), ((50, 50), true, true), ((50, 101), false, false)] {
            assert_eq!(is_route_origin(&rb, p, true), first, "{p:?} first iteration");
            assert_eq!(is_route_origin(&rb, p, false), later, "{p:?} later");
        }
    }
}

/// Where each of a net's shapes (as the connectivity check indexes them) sits in the design.
#[derive(Debug, Clone, Default)]
pub struct NetSlots {
    pub segs: Vec<usize>,
    pub vias: Vec<usize>,
    pub patches: Vec<usize>,
}

/// Every net's shapes in list order (the writing order, removed slots skipped), with their slots.
pub fn net_shapes(d: &DesignRoutes) -> BTreeMap<usize, (NetShapes, NetSlots)> {
    let mut out: BTreeMap<usize, (NetShapes, NetSlots)> = BTreeMap::new();
    for (k, sh) in d.shapes.iter().enumerate() {
        let Some(sh) = sh else { continue };
        let (n, slots) = out.entry(sh.net).or_default();
        match sh.fig {
            DrFig::Seg { layer, begin, end, width, begin_ext, end_ext, tapered, begin_trunc, end_trunc, .. } => {
                n.segs.push(Some(conn::Seg { layer, begin, end, width, begin_trunc, begin_ext, end_trunc, end_ext, tapered }));
                slots.segs.push(k);
            }
            DrFig::Via { via, origin, tapered, bottom_connected, top_connected, .. } => {
                n.vias.push(Some(conn::Via { via, origin, tapered, bottom_connected, top_connected }));
                slots.vias.push(k);
            }
            DrFig::Patch { layer, origin, offset } => {
                n.patches.push(Some(conn::Patch { layer, origin, offset }));
                slots.patches.push(k);
            }
        }
    }
    out
}

fn seg_fig(s: &conn::Seg) -> DrFig {
    DrFig::Seg { layer: s.layer, begin: s.begin, end: s.end, width: s.width, begin_ext: s.begin_ext, end_ext: s.end_ext, bi: (0, 0, 0), ei: (0, 0, 0), tapered: s.tapered, begin_trunc: s.begin_trunc, end_trunc: s.end_trunc }
}

/// One phase of the connectivity check's changes to one net, replayed on the design in order
/// (region-query removals and insertions, list drops and appends, markers — each marker the
/// net's own victim and aggressor, of the re-check kind).
pub fn apply_check_ops(d: &mut DesignRoutes, tech: &Tech, net: usize, name: &str, slots: &mut NetSlots, ops: &[Op]) {
    let slot = |slots: &NetSlots, r: Ref| match r {
        Ref::Seg(k) => slots.segs[k],
        Ref::Via(k) => slots.vias[k],
        Ref::Patch(k) => slots.patches[k],
    };
    for op in ops {
        match op {
            Op::Unindex(r) => d.unindex(slot(slots, *r)),
            Op::IndexSeg(k, s) => d.reindex(tech, slots.segs[*k], seg_fig(s)),
            Op::Drop(r) => {
                let k = slot(slots, *r);
                debug_assert!(d.rq[k].is_none(), "dropped while indexed");
                d.shapes[k] = None;
            }
            Op::AppendSeg(k) => {
                debug_assert_eq!(*k, slots.segs.len());
                let placeholder = DrFig::Patch { layer: 0, origin: (0, 0), offset: Rect { xl: 0, yl: 0, xh: 0, yh: 0 } };
                slots.segs.push(d.append_unindexed(net, placeholder));
            }
            Op::Marker(l, r) => {
                let o = Owner::Net(name.to_string());
                d.add_marker(Marker { rule: Rule::Recheck, layer: *l, bbox: *r, owners: vec![o.clone()], victim: (o.clone(), *l, *r, false), aggressor: (o, *l, *r, false) });
            }
        }
    }
}

/// The connectivity check over every modified net (in net order), each phase applied to the
/// design over all of them before the next; the modified set cleared. `check(net, shapes)` runs
/// the check on one net's shapes; an error stops the run.
pub fn connectivity_check(d: &mut DesignRoutes, tech: &Tech, name: &dyn Fn(usize) -> String, check: &mut dyn FnMut(usize, &mut NetShapes) -> Result<CheckLog, String>) -> Result<(), String> {
    let mut per = net_shapes(d);
    let nets: Vec<usize> = std::mem::take(&mut d.modified).into_iter().collect();
    let mut logs: Vec<(usize, CheckLog)> = Vec::new();
    for &n in &nets {
        let Some((shapes, _)) = per.get_mut(&n) else { continue };
        logs.push((n, check(n, shapes)?));
    }
    for phase in 0..3 {
        for (n, log) in &logs {
            let ops = match phase {
                0 => &log.split,
                1 => &log.merge,
                _ => &log.finish,
            };
            let slots = &mut per.get_mut(n).expect("a checked net").1;
            apply_check_ops(d, tech, *n, &name(*n), slots, ops);
        }
    }
    Ok(())
}
