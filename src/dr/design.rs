// SPDX-License-Identifier: Apache-2.0
//! The routes committed so far and the markers standing, as later workers read them; and a
//! worker's write-back.
//!
//! Rules (the first iteration's write-back):
//! - every net the worker routed has its shapes in the worker's extended box removed and its
//!   best shapes added; a wire crossing the route box keeps its parts outside (the crossing end
//!   now extends), and the points where it crossed are BOUNDARY POINTS; a wire along the route
//!   box's own side (x on its left or right for a vertical wire, y on its bottom or top for a
//!   horizontal one) is kept whole; a via or patch is removed when its origin lies in the route
//!   box;
//! - at each boundary point not on a pin of the net (or off the manufacturing grid), exactly two
//!   of the net's wires of one direction through it (none of the other, no patch there, both
//!   tapered or neither) merge into one, each outer end keeping its style;
//! - the markers in the worker's check box are replaced by its best markers that touch it.

use std::collections::BTreeSet;

use crate::dr::cost::DrFig;
use crate::gc::Marker;
use crate::polygon90::Rect;
use crate::tech::Tech;

type P = (i32, i32);

/// A committed shape: its net (routing index), the shape.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Shape {
    pub net: usize,
    pub fig: DrFig,
}

/// The committed routes (in the order they were written; removed slots empty) and markers.
#[derive(Debug, Clone, Default)]
pub struct DesignRoutes {
    pub shapes: Vec<Option<Shape>>,
    pub markers: Vec<Marker>,
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

impl DesignRoutes {
    /// The shapes whose stored box touches `b` (any layer), in writing order.
    pub fn query(&self, tech: &Tech, b: &Rect) -> Vec<usize> {
        (0..self.shapes.len()).filter(|&k| self.shapes[k].as_ref().is_some_and(|s| touches(&stored_box(tech, &s.fig).1, b))).collect()
    }

    fn add(&mut self, net: usize, fig: DrFig) {
        self.shapes.push(Some(Shape { net, fig }));
    }

    /// The markers whose box touches `b`.
    pub fn markers_in(&self, b: &Rect) -> Vec<Marker> {
        self.markers.iter().filter(|m| touches(&m.bbox, b)).cloned().collect()
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
}

/// A worker's write-back into the design.
pub fn end(d: &mut DesignRoutes, tech: &Tech, wb: &WriteBack<'_>) {
    let mod_nets: BTreeSet<usize> = wb.routed.iter().map(|(n, _)| *n).collect();
    let mut bound: std::collections::BTreeMap<usize, BTreeSet<(P, usize)>> = Default::default();
    // Remove the modified nets' shapes in the extended box.
    for k in d.query(tech, &wb.ext_box) {
        let Some(s) = d.shapes[k].clone() else { continue };
        if !mod_nets.contains(&s.net) {
            continue;
        }
        match s.fig {
            DrFig::Seg { .. } => remove_seg(d, k, &wb.route_box, bound.entry(s.net).or_default()),
            DrFig::Via { origin, .. } | DrFig::Patch { origin, .. } => {
                if in_box(&wb.route_box, origin) {
                    d.shapes[k] = None;
                }
            }
        }
    }
    // Add the best shapes.
    for (net, figs) in wb.routed {
        for f in figs {
            d.add(*net, f.clone());
        }
    }
    for (net, pts) in &bound {
        for &(pt, l) in pts {
            merge_at(d, tech, *net, pt, l, wb);
        }
    }
    // Markers.
    d.markers.retain(|m| !touches(&m.bbox, &wb.drc_box));
    for m in wb.markers {
        if touches(&m.bbox, &wb.drc_box) {
            d.markers.push(m.clone());
        }
    }
}

/// A committed wire of a net the worker rerouted: kept whole along the route box's sides; else
/// the parts outside the route box kept (the crossing end now extending), the crossing points
/// recorded.
fn remove_seg(d: &mut DesignRoutes, k: usize, rb: &Rect, bound: &mut BTreeSet<(P, usize)>) {
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
        d.add(net, f);
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
        d.add(net, f);
        if lo <= bhi {
            bound.insert((nb, layer));
        }
    }
    d.shapes[k] = None;
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
    for k in 0..d.shapes.len() {
        let Some(s) = &d.shapes[k] else { continue };
        if s.net != net {
            continue;
        }
        let (sl, b) = stored_box(tech, &s.fig);
        if sl != layer || !touches(&b, &q) {
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
        d.shapes[group[0]] = None;
        d.shapes[group[1]] = None;
        d.add(net, f);
    }
}
