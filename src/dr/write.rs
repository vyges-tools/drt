// SPDX-License-Identifier: Apache-2.0
//! What a found path becomes: wires and vias, then patches where a layer's metal falls short of
//! its minimum area.
//!
//! Rules:
//! - each straight run is a wire on the grid points, its ends extended by half the layer width
//!   (the default style) unless the end is at an access point (a real pin's always; another's or
//!   one on the worker border when a same-net terminal has an access point there): truncated;
//! - a wire across its layer's direction takes the layer's wrong-way width; a preferred-direction
//!   wire meeting a wrong-way one at an end extends only half the wrong-way width there (when that
//!   is narrower);
//! - a non-default-rule net's wire takes the rule's width (and its extensions half of it), except
//!   inside a pin's taper box, where the wire is split at the box edge and the inner piece keeps
//!   the default style (tapered);
//! - each layer change is one via per cut layer: the cut's default via, an access point's own via
//!   at a special-via node, a non-default rule's preferred via;
//! - minimum area: walking the path, a layer's metal (the start pin's access area, wire lengths ×
//!   width, half each via's enclosure) short of the rule gets a patch along the layer's direction
//!   at whichever end of that stretch costs less to cover — 64-bit unsigned area arithmetic,
//!   wrapping, as the gap narrows to a 32-bit length.

use std::collections::{BTreeSet, HashMap};

use crate::dr::cost::{CostWorker, DrFig};
use crate::dr::maze::{Idx, MazeState, TaperBox};
use crate::dr::rules::NdrRule;
use crate::polygon90::Rect;

type P = (i32, i32);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum End {
    Extend,
    Truncate,
}

#[derive(Debug, Clone, Copy)]
struct Style {
    width: i32,
    begin_ext: i32,
    end_ext: i32,
    begin: End,
    end: End,
}

/// What writing a path reads.
pub struct WriteCtx<'a> {
    pub route_box: Rect,
    /// Real pins' access points, and every access point of the net.
    pub real_ap: &'a BTreeSet<Idx>,
    pub ap: &'a BTreeSet<Idx>,
    /// Whether a terminal of this net has an access point at the point on the layer.
    pub has_access_point: &'a dyn Fn(P, usize) -> bool,
    pub ndr: Option<&'a NdrRule>,
    pub auto_taper: bool,
    pub tapers: &'a [TaperBox],
    pub taper_at: &'a HashMap<Idx, usize>,
    /// The access area at each access point (the largest of the pins there), when kept.
    pub area_at: &'a HashMap<Idx, i64>,
}

fn in_border(r: &Rect, x: i32, y: i32) -> bool {
    ((x == r.xl || x == r.xh) && y <= r.yh && y >= r.yl) || ((y == r.yl || y == r.yh) && x <= r.xh && x >= r.xl)
}

fn idx_lt(a: Idx, b: Idx) -> bool {
    (a.0, a.1, a.2) < (b.0, b.1, b.2)
}

fn contains_bloat(b: &TaperBox, x: i32, y: i32, z: i32, bx: i32, by: i32) -> bool {
    b.lo.2 <= z && b.hi.2 >= z && b.lo.0 - bx <= x && b.hi.0 + bx >= x && b.lo.1 - by <= y && b.hi.1 + by >= y
}

/// A found path's wires and vias, in the order they are written.
pub fn write_path(w: &CostWorker<'_, '_>, cx: &WriteCtx<'_>, points: &[Idx]) -> Vec<DrFig> {
    let mut out = Vec::new();
    if points.len() <= 1 {
        // (An access point's own path segments would go here; the modelled pins have none.)
        return out;
    }
    let dst_box = cx.taper_at.get(&points[0]).copied();
    let src_box = cx.taper_at.get(points.last().expect("a point")).copied();
    for i in 0..points.len() - 1 {
        let (start, end) = if idx_lt(points[i + 1], points[i]) { (points[i + 1], points[i]) } else { (points[i], points[i + 1]) };
        if start.2 == end.2 && ((start.0 != end.0 && start.1 == end.1) || (start.0 == end.0 && start.1 != end.1)) {
            let (mut sx, mut sy) = (start.0, start.1);
            let (ex, ey, z) = (end.0, end.1, start.2);
            let vertical = sx == ex;
            let (split, mut taper) = split_path_seg(cx, sx, sy, ex, ey, z, src_box, dst_box);
            if let Some((mx, my)) = split {
                out.push(process_path_seg(w, cx, sx, sy, mx, my, z, vertical, taper, i, points));
                sx = mx;
                sy = my;
                let (split, t) = split_path_seg(cx, sx, sy, ex, ey, z, src_box, dst_box);
                taper = t;
                if let Some((mx, my)) = split {
                    out.push(process_path_seg(w, cx, sx, sy, mx, my, z, vertical, taper, i, points));
                    sx = mx;
                    sy = my;
                    taper = true;
                }
            }
            out.push(process_path_seg(w, cx, sx, sy, ex, ey, z, vertical, taper, i, points));
        } else if start.0 == end.0 && start.1 == end.1 && start.2 != end.2 {
            for z in start.2..end.2 {
                let l = w.g.zs[z as usize];
                let origin = (w.g.xs[start.0 as usize], w.g.ys[start.1 as usize]);
                let mut via = w.cx.defaults.get(l + 1).copied().flatten().expect("a default via");
                let k = w.g.idx(start.0 as usize, start.1 as usize, z as usize);
                if w.g.nodes[k].svia {
                    if let Some(&v) = w.ap_svia.get(&(start.0 as usize, start.1 as usize, z as usize)) {
                        via = v;
                    }
                }
                if let Some(n) = cx.ndr {
                    if let Some(&v) = n.vias.get(z as usize).and_then(|v| v.first()) {
                        via = v;
                    }
                }
                // Tapered: a rule net's via at a point of a taper box on either of the path's layers.
                let tapered = cx.ndr.is_some() && cx.auto_taper && (cx.taper_at.contains_key(&(end.0, end.1, start.2)) || cx.taper_at.contains_key(&(end.0, end.1, end.2)));
                let (bot, top) = ((start.0, start.1, z), (start.0, start.1, z + 1));
                let bottom_connected = pin_connected(cx, bot, origin, l);
                let top_connected = pin_connected(cx, top, origin, w.g.zs[z as usize + 1]);
                out.push(DrFig::Via { via, origin, bi: (start.0 as usize, start.1 as usize, z as usize), ei: (start.0 as usize, start.1 as usize, z as usize + 1), tapered, bottom_connected, top_connected });
            }
        }
    }
    out
}

/// Where a non-default-rule wire leaves (or enters) a taper box: the split point (none: no
/// split), and whether the first piece tapers.
#[allow(clippy::too_many_arguments)]
fn split_path_seg(cx: &WriteCtx<'_>, sx: i32, sy: i32, ex: i32, ey: i32, z: i32, src: Option<usize>, dst: Option<usize>) -> (Option<(i32, i32)>, bool) {
    if cx.ndr.is_none() || !cx.auto_taper {
        return (None, false);
    }
    let bx = |x: i32, y: i32| -> Option<usize> {
        if let Some(s) = src.filter(|&s| cx.tapers[s].contains((x, y, z))) {
            return Some(s);
        }
        dst.filter(|&d| cx.tapers[d].contains((x, y, z)))
    };
    if let Some(b) = bx(sx, sy) {
        let b = &cx.tapers[b];
        if contains_bloat(b, ex, ey, z, 1, 1) {
            return (None, true);
        }
        return (Some(if sx == ex { (sx, b.hi.1 + 1) } else { (b.hi.0 + 1, sy) }), true);
    }
    if let Some(b) = bx(ex, ey) {
        let b = &cx.tapers[b];
        if contains_bloat(b, sx, sy, z, 1, 1) {
            return (None, true);
        }
        return (Some(if sx == ex { (sx, b.lo.1 - 1) } else { (b.lo.0 - 1, sy) }), false);
    }
    (None, false)
}

/// Whether a wire end or via end at the node lands on the net's pin: a real access point of it,
/// or (at any access point, or on the worker's border) a point where the pin has an access point
/// on the layer.
fn pin_connected(cx: &WriteCtx<'_>, idx: Idx, pt: P, l: usize) -> bool {
    cx.real_ap.contains(&idx) || ((cx.ap.contains(&idx) || in_border(&cx.route_box, pt.0, pt.1)) && (cx.has_access_point)(pt, l))
}

#[allow(clippy::too_many_arguments)]
fn process_path_seg(w: &CostWorker<'_, '_>, cx: &WriteCtx<'_>, sx: i32, sy: i32, ex: i32, ey: i32, z: i32, vertical: bool, taper: bool, i: usize, points: &[Idx]) -> DrFig {
    let tech = w.cx.tech;
    let l = w.g.zs[z as usize];
    let layer = &tech.layers[l];
    let begin = (w.g.xs[sx as usize], w.g.ys[sy as usize]);
    let end = (w.g.xs[ex as usize], w.g.ys[ey as usize]);
    let mut st = Style { width: layer.width, begin_ext: layer.width / 2, end_ext: layer.width / 2, begin: End::Extend, end: End::Extend };
    for (is_begin, idx, pt) in [(true, (sx, sy, z), begin), (false, (ex, ey, z), end)] {
        if pin_connected(cx, idx, pt, l) {
            if is_begin {
                st.begin = End::Truncate;
                st.begin_ext = 0;
            } else {
                st.end = End::Truncate;
                st.end_ext = 0;
            }
        }
    }
    let prev = if i >= 1 { Some(points[i - 1]) } else { None };
    let next = points.get(i + 2).copied();
    let mut tapered = false;
    if let Some(n) = cx.ndr {
        if taper {
            tapered = true;
        } else {
            set_ndr_style(n, &mut st, z);
        }
    } else if layer.is_vertical() != vertical {
        st.width = layer.wrong_way_width;
    } else if layer.wrong_way_width < layer.width {
        let orth = |m: Option<Idx>, x: i32| m.is_some_and(|m| m.2 == z && (vertical != (m.0 == x)));
        if orth(next, ex) && st.end == End::Extend {
            st.end_ext = layer.wrong_way_width / 2;
        }
        if orth(prev, sx) && st.begin == End::Extend {
            st.begin_ext = layer.wrong_way_width / 2;
        }
    }
    DrFig::Seg { layer: l, begin, end, width: st.width, begin_ext: st.begin_ext, end_ext: st.end_ext, bi: (sx as usize, sy as usize, z as usize), ei: (ex as usize, ey as usize, z as usize), tapered, begin_trunc: st.begin == End::Truncate, end_trunc: st.end == End::Truncate }
}

/// A non-default rule's width (extensions half of it) when wider. (Wire extensions are not
/// modelled: the modelled rules set none.)
fn set_ndr_style(n: &NdrRule, st: &mut Style, z: i32) {
    let w = n.widths.get(z as usize).copied().unwrap_or(0);
    if w > st.width {
        st.width = w;
        st.begin_ext = w / 2;
        st.end_ext = w / 2;
    }
}

/// The path with each stacked via's layers spelled out.
fn separated(path: &[Idx]) -> Vec<Idx> {
    let mut pts = Vec::new();
    for k in 0..path.len() - 1 {
        let (c, n) = (path[k], path[k + 1]);
        if c.2 == n.2 {
            pts.push(c);
        } else if c.2 < n.2 {
            for z in c.2..n.2 {
                pts.push((c.0, c.1, z));
            }
        } else {
            let mut z = c.2;
            while z > n.2 {
                pts.push((c.0, c.1, z));
                z -= 1;
            }
        }
    }
    pts.push(*path.last().expect("a point"));
    pts
}

/// Half a via's enclosure area on its bottom (layer 1) or top layer: the cut's default via (a
/// non-default rule's preferred one when it has it).
fn half_via_enc_area(w: &CostWorker<'_, '_>, z: i32, layer1: bool, ndr: Option<&NdrRule>) -> i32 {
    let tech = w.cx.tech;
    let area = |v: usize| {
        let vd = &tech.via_defs[v];
        let b = if layer1 { vd.layer1_bbox() } else { vd.layer2_bbox() };
        (i64::from(b.dx()) * i64::from(b.dy()) / 2) as i32
    };
    if let Some(v) = ndr.and_then(|n| n.vias.get(z as usize)).and_then(|v| v.first()) {
        return area(*v);
    }
    if z < 0 || z as usize >= w.g.zs.len() {
        return 0;
    }
    let l = w.g.zs[z as usize];
    match w.cx.defaults.get(l + 1).copied().flatten() {
        Some(v) if tech.layers.get(l + 1).is_some_and(|c| c.kind == crate::tech::LayerKind::Cut) => area(v),
        _ => 0,
    }
}

/// The patches a path needs for minimum area, in the order they are written.
pub fn patch_min_area(w: &CostWorker<'_, '_>, st: &MazeState, cx: &WriteCtx<'_>, path: &[Idx], drc: u32, fixed: u32, marker: u32) -> Vec<DrFig> {
    let mut out = Vec::new();
    if path.is_empty() {
        return out;
    }
    let tech = w.cx.tech;
    let pts = separated(path);
    let min_area = |z: i32| tech.layers[w.g.zs[z as usize]].min_area as u64;
    let mut curr: u64 = match cx.area_at.get(&pts[0]) {
        Some(&a) => a as u64,
        None => min_area(pts[0].2),
    };
    let mut start_half: i32 = 0;
    let mut end_half: i32;
    let mut ci = pts[0];
    let mut prev_i = 0usize;
    let mut prev_wire = true;
    let mut i = 1;
    while i < pts.len() {
        let ni = pts[i];
        if ni.2 != ci.2 {
            let req = min_area(ci.2);
            let z = if ni.2 < ci.2 { ci.2 - 1 } else { ci.2 };
            let l1 = ni.2 >= ci.2;
            let h = half_via_enc_area(w, z, l1, cx.ndr);
            if prev_wire {
                curr = curr.wrapping_add(h as i64 as u64);
            } else {
                curr = ((h as i64 * 2) as u64).max(curr);
            }
            end_half = h;
            if curr < req {
                out.extend(patch_helper(w, st, cx, ci.2, req, curr, start_half, end_half, &pts, i, prev_i, drc, fixed, marker));
            }
            if ni.2 < ci.2 {
                let h = half_via_enc_area(w, ci.2 - 1, true, cx.ndr);
                curr = (h as i64 * 2) as u64;
                start_half = h;
            } else {
                curr = (half_via_enc_area(w, ci.2, false, cx.ndr) as i64 * 2) as u64;
                // (The default via's, not the rule's: the grid's own table.)
                start_half = half_via_enc_area(w, ci.2, false, None);
            }
            prev_i = i;
            prev_wire = false;
        } else {
            let req = min_area(ci.2);
            let width = tech.layers[w.g.zs[ci.2 as usize]].width;
            let (b, e) = ((w.g.xs[ci.0 as usize], w.g.ys[ci.1 as usize]), (w.g.xs[ni.0 as usize], w.g.ys[ni.1 as usize]));
            let len = (b.0 - e.0).abs() + (b.1 - e.1).abs();
            if curr < req {
                if !prev_wire {
                    curr /= 2;
                }
                curr = curr.wrapping_add((len as u64).wrapping_mul(width as i64 as u64));
            }
            prev_wire = true;
        }
        ci = ni;
        i += 1;
    }
    let req = min_area(ci.2);
    if curr < req {
        if let Some(&a) = cx.area_at.get(&ci) {
            if !prev_wire {
                curr /= 2;
            }
            curr = curr.wrapping_add(a as u64);
        }
    }
    end_half = 0;
    if curr < req {
        out.extend(patch_helper(w, st, cx, ci.2, req, curr, start_half, end_half, &pts, i, prev_i, drc, fixed, marker));
    }
    out
}

#[allow(clippy::too_many_arguments)]
fn patch_helper(w: &CostWorker<'_, '_>, st: &MazeState, cx: &WriteCtx<'_>, z: i32, req: u64, curr: u64, start_half: i32, end_half: i32, pts: &[Idx], pi: usize, prev_i: usize, drc: u32, fixed: u32, marker: u32) -> Vec<DrFig> {
    let tech = w.cx.tech;
    let layer = &tech.layers[w.g.zs[z as usize]];
    let (sh, eh) = (start_half as i64 as u64, end_half as i64 as u64);
    let gap = req.wrapping_sub(curr.wrapping_sub(sh).wrapping_sub(eh)).wrapping_sub(sh.min(eh));
    let (bp, ep, bp_left, ep_right);
    if pi - 1 == prev_i {
        bp = pts[pi - 1];
        ep = pts[pi - 1];
        bp_left = true;
        ep_right = false;
    } else {
        bp = pts[prev_i];
        ep = pts[pi - 1];
        let (bs, epp) = (pts[prev_i + 1], pts[pi - 2]);
        if layer.is_horizontal() {
            bp_left = if bp.0 == bs.0 { bp.0 < ep.0 } else { bp.0 < bs.0 };
            ep_right = if ep.0 == epp.0 { ep.0 <= bp.0 } else { ep.0 < epp.0 };
        } else {
            bp_left = if bp.1 == bs.1 { bp.1 < ep.1 } else { bp.1 < bs.1 };
            ep_right = if ep.1 == epp.1 { ep.1 <= bp.1 } else { ep.1 < epp.1 };
        }
    }
    let width = layer.width;
    let mg = tech.manufacturing_grid.max(1);
    let gap32 = gap as i32;
    let len = ((f64::from(gap32) / f64::from(width) / f64::from(mg)).ceil() as i32) * mg;
    let horz = layer.is_horizontal();
    let cl = patch_cost(w, st, &cx.route_box, bp, horz, bp_left, len, drc, fixed, marker);
    let cr = patch_cost(w, st, &cx.route_box, ep, horz, ep_right, len, drc, fixed, marker);
    let (at, left) = if cl <= cr { (bp, bp_left) } else { (ep, ep_right) };
    let origin = (w.g.xs[at.0 as usize], w.g.ys[at.1 as usize]);
    let offset = match (horz, left) {
        (true, true) => Rect { xl: -len, yl: -width / 2, xh: 0, yh: width / 2 },
        (true, false) => Rect { xl: 0, yl: -width / 2, xh: len, yh: width / 2 },
        (false, true) => Rect { xl: -width / 2, yl: -len, xh: width / 2, yh: 0 },
        (false, false) => Rect { xl: -width / 2, yl: 0, xh: width / 2, yh: len },
    };
    vec![DrFig::Patch { layer: w.g.zs[z as usize], origin, offset }]
}

/// What a patch from `at` would sit on: route, fixed and marker cost × length along it (the
/// planar cost is read at the edge's far node); unbounded when it would leave the route box.
#[allow(clippy::too_many_arguments)]
fn patch_cost(w: &CostWorker<'_, '_>, st: &MazeState, rb: &Rect, at: Idx, horz: bool, left: bool, len: i32, drc: u32, fixed: u32, marker: u32) -> i32 {
    let o = (w.g.xs[at.0 as usize], w.g.ys[at.1 as usize]);
    let e = match (horz, left) {
        (true, true) => (o.0 - len, o.1),
        (true, false) => (o.0 + len, o.1),
        (false, true) => (o.0, o.1 - len),
        (false, false) => (o.0, o.1 + len),
    };
    let _ = st;
    if !(e.0 >= rb.xl && e.0 <= rb.xh && e.1 >= rb.yl && e.1 <= rb.yh) {
        return i32::MAX;
    }
    let (lo, hi) = (o.min(e), o.max(e));
    let b = Rect { xl: lo.0, yl: lo.1, xh: hi.0, yh: hi.1 };
    let (x1, y1, x2, y2) = w.g.idx_box_enclose(&b);
    let mut cost: i32 = 0;
    let z = at.2 as usize;
    let dir = if horz { crate::dr::cost::Dir6::E } else { crate::dr::cost::Dir6::N };
    let (from, to) = if horz { (x1.saturating_sub(1), x2) } else { (y1.saturating_sub(1), y2) };
    for k in from..to {
        let (x, y) = if horz { (k, at.1 as usize) } else { (at.0 as usize, k) };
        let len_e = if horz { w.g.xs[x + 1] - w.g.xs[x] } else { w.g.ys[y + 1] - w.g.ys[y] };
        if w.g.route_cost_adj(x, y, z, dir) != 0 {
            cost = cost.wrapping_add(len_e.wrapping_mul(drc as i32));
        }
        if w.g.fixed_cost_adj(x, y, z, dir) != 0 {
            cost = cost.wrapping_add(len_e.wrapping_mul(fixed as i32));
        }
        if w.g.marker_cost_adj(x, y, z, dir) != 0 {
            cost = cost.wrapping_add(len_e.wrapping_mul(marker as i32));
        }
    }
    cost
}
