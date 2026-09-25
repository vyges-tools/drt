// SPDX-License-Identifier: Apache-2.0
//! Access points kept: per pin, cost rounds of candidates, each filtered by design-rule trials into
//! the directions (planar) and vias it may be reached by; the rounds stop once enough points are
//! kept.
//!
//! Stages, in order ([`gen_pin_access`]): the pin's merged shapes; per round, upper class outer and
//! lower class inner ([`gen_pin_access_cost_bounded`]): the round's candidates and their allowed
//! accesses ([`create_access_point`]), each planar direction tried ([`validate_ap_for_planar_access`],
//! [`filter_planar_access`]), the vias tried ([`filter_multiple_ap_accesses`],
//! [`filter_via_access`], [`check_via_planar_access`]), the points kept, the stopping test
//! ([`enough_access_points`]); finally each point's vias ordered by how far they stick out of the
//! pin ([`via_max_ext`]).
//!
//! Rules:
//! - a planar direction is tried only if allowed; its segment ends three layer-widths out (for a
//!   block, a pitch past the pin's extent); an end still INSIDE the pin (boundary included) drops
//!   the direction untried; otherwise the design-rule verdict decides;
//! - a standard cell's pin at or below the via-access layer (or in the via-in-pin range) gets no
//!   planar access; a layer without non-preferred tracks allows no wrong-way planar access;
//! - vias are the cut layer's single-cut vias in priority order, at most two (every one when
//!   retrying), plus — for a top-level port — those of the cut layer below; for a standard cell's
//!   or block's pin, a via whose shapes leave the cell is skipped (not when retrying); a via that
//!   sticks out of the pin is skipped when the via must stay in the pin (the via-in-pin range, or
//!   an enclosed-boundary point); a via is kept when a segment leaving it on its other layer passes
//!   in some direction (south, west, east, north, first to pass) — at most two per point;
//! - if no point got an access (for a standard cell: a via at or below the via-access layer, any
//!   access above it), every point's vias are tried again with the retry rules; a top-level port
//!   with a planar access skips the vias entirely;
//! - a point is kept when it has an access — for a standard cell at or below the via-access layer,
//!   an up via; the rounds stop when a top-level port has a point, or when the kept points,
//!   counting one per cluster of same-layer points within half a width, reach the minimum;
//! - a point's vias are ordered by the most they stick out of the pin, least first (stable).
//!
//! Not transcribed (a caller gets [`Unsupported`]): a nearby-track round (its points carry path
//! segments). Taken as absent because the technologies here do not have them: the metal-width via
//! map, unidirectional (multi-mask or rect-only) layers, right-way-on-grid-only layers, a net's
//! non-default rule without auto-taper, stubborn terminals.

use std::collections::BTreeSet;

use crate::gc::{Marker, Owner};
use crate::pa::candidates::{merge_pin_shapes, round_candidates, ApType, Context, TermKind};
use crate::pa::verdict::{planar_markers, via_markers, TargetShape};
use crate::polygon90::{Polygon90Set, Rect};
use crate::tech::{Dir, ViaDef};

/// An access direction. `index` is the position in [`AccessPoint::access`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Access {
    E = 0,
    W = 1,
    S = 2,
    N = 3,
    U = 4,
    D = 5,
}

/// The planar directions, in the order they are tried.
pub const PLANAR: [Access; 4] = [Access::S, Access::W, Access::E, Access::N];

/// The router settings pin access reads.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Config {
    pub via_access_layer: usize,
    /// The via-in-pin layer range, if set.
    pub via_in_pin: Option<(usize, usize)>,
    pub top_routing_layer: usize,
    pub min_std_cell_points: usize,
    pub min_macro_points: usize,
    pub use_nonpref_tracks: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AccessPoint {
    pub point: (i32, i32),
    pub layer: usize,
    pub lower: ApType,
    pub upper: ApType,
    /// E, W, S, N, U, D.
    pub access: [bool; 6],
    pub allow_via: bool,
    /// Single-cut vias (indices into `Tech::via_defs`), in order.
    pub vias: Vec<usize>,
}

impl AccessPoint {
    pub fn has(&self, d: Access) -> bool {
        self.access[d as usize]
    }
    fn set(&mut self, d: Access, v: bool) {
        self.access[d as usize] = v;
    }
    pub fn has_any(&self) -> bool {
        self.access.iter().any(|&a| a)
    }
    pub fn has_planar(&self) -> bool {
        PLANAR.iter().any(|&d| self.has(d))
    }
    /// The accesses as the design database stores them: bits N 1, S 2, E 4, W 8, U 16, D 32.
    pub fn db_access_bits(&self) -> u8 {
        [Access::N, Access::S, Access::E, Access::W, Access::U, Access::D].iter().enumerate().filter(|(_, &d)| self.has(d)).map(|(k, _)| 1u8 << k).sum()
    }
    /// The cost the pattern search reads: lower class plus four times the upper.
    pub fn cost(&self) -> i32 {
        self.lower as i32 + 4 * self.upper as i32
    }
}

/// A pin as its access points are searched.
pub struct Pin<'a> {
    pub cx: &'a Context<'a>,
    pub cfg: &'a Config,
    pub kind: TermKind,
    /// A block's pin (its planar segments end a pitch past the pin).
    pub is_block: bool,
    /// Its instance's placed box (none for a top-level port).
    pub boundary: Option<Rect>,
    /// The shapes the checks see: the instance's, or the port's.
    pub target: &'a [TargetShape],
    /// The owner of the trial shapes.
    pub owner: &'a Owner,
}

/// What the search did, in order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Event {
    Round { lower: ApType, upper: ApType },
    Cand { point: (i32, i32), layer: usize, lower: ApType, upper: ApType },
    /// A planar trial: the segment's two stored points and the markers.
    Planar { point: (i32, i32), layer: usize, seg: ((i32, i32), (i32, i32)), markers: Vec<Marker> },
    VList { point: (i32, i32), layer: usize, try_all: bool, must_in_pin: bool, vias: Vec<usize> },
    /// A via considered: `None` checked, else the reason it was skipped (`B1`, `B2`, `P`); its
    /// extension when measured.
    VTry { via: usize, skip: Option<&'static str>, ext: Option<i32> },
    Via { point: (i32, i32), layer: usize, via: usize, seg: ((i32, i32), (i32, i32)), markers: Vec<Marker> },
    TryAll,
    State(AccessPoint),
    Enough { enough: bool, kept: usize },
    /// A kept point after its vias are ordered, with each via's extension.
    Apx(AccessPoint, Vec<i32>),
}

pub type Trace = Option<Vec<Event>>;

fn push(t: &mut Trace, e: impl FnOnce() -> Event) {
    if let Some(v) = t {
        v.push(e());
    }
}

/// A search step this transcription does not model.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Unsupported(pub &'static str);

/// The pin's shapes per routing layer: the merged set and its slices.
struct Shapes {
    sets: Vec<Polygon90Set>,
    rects: Vec<Vec<Rect>>,
}

/// Every kept access point of a pin whose shapes (design coordinates) are `shapes`.
pub fn gen_pin_access(pin: &Pin<'_>, shapes: &[(usize, Rect)], trace: &mut Trace) -> Result<Vec<AccessPoint>, Unsupported> {
    let mut sets = merge_pin_shapes(pin.cx.tech, shapes);
    let rects = sets.iter_mut().map(|s| s.rectangles()).collect();
    let mut sh = Shapes { sets, rects };
    let mut aps: Vec<AccessPoint> = Vec::new();
    let mut apset: BTreeSet<((i32, i32), usize)> = BTreeSet::new();
    let mut enough = false;
    for upper in [ApType::OnGrid, ApType::HalfGrid, ApType::Center, ApType::EncOpt, ApType::NearbyGrid] {
        for lower in [ApType::OnGrid, ApType::HalfGrid, ApType::Center, ApType::EncOpt] {
            if upper == ApType::NearbyGrid && !aps.is_empty() {
                continue;
            }
            if enough {
                break;
            }
            if upper == ApType::NearbyGrid {
                return Err(Unsupported("a nearby-track round"));
            }
            enough = gen_pin_access_cost_bounded(pin, &mut sh, &mut aps, &mut apset, lower, upper, trace);
        }
    }
    for ap in &mut aps {
        let exts = sort_via_defs(pin, ap, &sh.rects[ap.layer]);
        push(trace, || Event::Apx(ap.clone(), exts));
    }
    Ok(aps)
}

/// A point's vias ordered by how far they stick out of the pin, least first (stable); returns the
/// extensions in the new order.
pub fn sort_via_defs(pin: &Pin<'_>, ap: &mut AccessPoint, pin_rects: &[Rect]) -> Vec<i32> {
    let mut sorted: Vec<(usize, i32)> = ap.vias.iter().map(|&v| (v, via_max_ext(pin, ap, pin_rects, &pin.cx.tech.via_defs[v]))).collect();
    sorted.sort_by_key(|&(_, e)| e);
    ap.vias = sorted.iter().map(|&(v, _)| v).collect();
    sorted.iter().map(|&(_, e)| e).collect()
}

/// One round: candidates, their filters, the points kept; whether enough are kept now.
#[allow(clippy::too_many_arguments)]
fn gen_pin_access_cost_bounded(pin: &Pin<'_>, sh: &mut Shapes, aps: &mut Vec<AccessPoint>, apset: &mut BTreeSet<((i32, i32), usize)>, lower: ApType, upper: ApType, trace: &mut Trace) -> bool {
    let cands = round_candidates(pin.cx, &mut sh.sets, pin.kind, lower, upper, apset);
    push(trace, || Event::Round { lower, upper });
    for c in &cands {
        push(trace, || Event::Cand { point: (c.x, c.y), layer: c.layer, lower: c.lower, upper: c.upper });
    }
    let mut new_aps: Vec<AccessPoint> = cands.iter().map(|c| create_access_point(pin, (c.x, c.y), c.layer, c.lower, c.upper)).collect();
    for ap in &mut new_aps {
        validate_ap_for_planar_access(pin, sh, ap, trace);
    }
    filter_multiple_ap_accesses(pin, sh, &mut new_aps, trace);
    for ap in &new_aps {
        push(trace, || Event::State(ap.clone()));
    }
    let std_cell = pin.kind == TermKind::StdCell;
    for ap in new_aps {
        if !ap.has_any() {
            continue;
        }
        if std_cell && ap.layer <= pin.cfg.via_access_layer && !ap.has(Access::U) {
            continue;
        }
        aps.push(ap);
    }
    let enough = enough_access_points(pin, aps);
    push(trace, || Event::Enough { enough, kept: aps.len() });
    enough
}

fn in_via_in_pin(cfg: &Config, layer: usize) -> bool {
    cfg.via_in_pin.is_some_and(|(lo, hi)| layer >= lo && layer <= hi)
}

/// A candidate's allowed accesses: every planar direction unless planar access is barred on the
/// layer, wrong-way ones only with non-preferred tracks; vias always tried.
pub fn create_access_point(pin: &Pin<'_>, point: (i32, i32), layer: usize, lower: ApType, upper: ApType) -> AccessPoint {
    let cfg = pin.cfg;
    let allow_planar = !(pin.kind == TermKind::StdCell && (in_via_in_pin(cfg, layer) || layer <= cfg.via_access_layer));
    let mut ap = AccessPoint { point, layer, lower, upper, access: [false; 6], allow_via: true, vias: Vec::new() };
    for d in PLANAR {
        ap.set(d, allow_planar);
    }
    if allow_planar && !cfg.use_nonpref_tracks {
        match pin.cx.tech.layers[layer].dir {
            Dir::Horizontal => {
                ap.set(Access::N, false);
                ap.set(Access::S, false);
            }
            Dir::Vertical => {
                ap.set(Access::W, false);
                ap.set(Access::E, false);
            }
            _ => {}
        }
    }
    ap
}

fn validate_ap_for_planar_access(pin: &Pin<'_>, sh: &Shapes, ap: &mut AccessPoint, trace: &mut Trace) -> bool {
    let mut allow = false;
    for d in PLANAR {
        allow |= filter_planar_access(pin, sh, ap, d, trace);
    }
    allow
}

/// The segment's two points as stored: toward W or S the far end first.
fn stored(d: Access, begin: (i32, i32), end: (i32, i32)) -> ((i32, i32), (i32, i32)) {
    if matches!(d, Access::W | Access::S) {
        (end, begin)
    } else {
        (begin, end)
    }
}

fn filter_planar_access(pin: &Pin<'_>, sh: &Shapes, ap: &mut AccessPoint, d: Access, trace: &mut Trace) -> bool {
    if !ap.has(d) {
        return false;
    }
    let end = gen_end_point(pin, &sh.rects[ap.layer], ap.point, ap.layer, d);
    if !is_point_outside_shapes(end, &sh.rects[ap.layer]) {
        ap.set(d, false);
        return false;
    }
    let markers = planar_markers(pin.cx.tech, pin.target, pin.owner, ap.point, ap.layer, end);
    let no_drv = markers.is_empty();
    push(trace, || Event::Planar { point: ap.point, layer: ap.layer, seg: stored(d, ap.point, end), markers });
    ap.set(d, no_drv);
    no_drv
}

/// Three widths of `layer` out from `begin` in `d`; for a block, a pitch past the pin's extent.
fn gen_end_point(pin: &Pin<'_>, rects: &[Rect], begin: (i32, i32), layer: usize, d: Access) -> (i32, i32) {
    let l = &pin.cx.tech.layers[layer];
    let step = 3 * l.width;
    let (mut x, mut y) = begin;
    let ext = pin.is_block.then(|| extents(rects));
    match (d, ext) {
        (Access::W, Some(e)) => x = e.xl - l.pitch,
        (Access::W, None) => x -= step,
        (Access::E, Some(e)) => x = e.xh + l.pitch,
        (Access::E, None) => x += step,
        (Access::S, Some(e)) => y = e.yl - l.pitch,
        (Access::S, None) => y -= step,
        (Access::N, Some(e)) => y = e.yh + l.pitch,
        (Access::N, None) => y += step,
        _ => unreachable!("a planar direction"),
    }
    (x, y)
}

fn extents(rects: &[Rect]) -> Rect {
    let mut e = rects[0];
    for r in &rects[1..] {
        e = Rect::new(e.xl.min(r.xl), e.yl.min(r.yl), e.xh.max(r.xh), e.yh.max(r.yh));
    }
    e
}

/// Outside every shape of the layer; a point on a shape's boundary is inside.
fn is_point_outside_shapes(p: (i32, i32), rects: &[Rect]) -> bool {
    !rects.iter().any(|r| r.contains(p.0, p.1))
}

fn filter_multiple_ap_accesses(pin: &Pin<'_>, sh: &Shapes, aps: &mut [AccessPoint], trace: &mut Trace) {
    if pin.kind == TermKind::Io && aps.iter().any(|ap| ap.has_planar()) {
        return;
    }
    let va = pin.cfg.via_access_layer;
    let mut has_access = false;
    for ap in aps.iter_mut() {
        filter_via_access(pin, sh, ap, false, trace);
        has_access |= if pin.kind == TermKind::StdCell { (ap.layer <= va && ap.has(Access::U)) || (ap.layer > va && ap.has_any()) } else { ap.has_any() };
    }
    if !has_access {
        push(trace, || Event::TryAll);
        for ap in aps.iter_mut() {
            filter_via_access(pin, sh, ap, true, trace);
        }
    }
}

/// The first `max` vias of the cut layer in priority order — at least one, even when `max` is not
/// positive (the reference checks the count only after adding).
fn get_priority_via_defs(pin: &Pin<'_>, cut: usize, max: i64) -> Vec<usize> {
    let mut out = Vec::new();
    if let Some(vias) = pin.cx.via_priority.get(&cut) {
        for &v in vias {
            out.push(v);
            if out.len() as i64 >= max {
                break;
            }
        }
    }
    out
}

fn filter_via_access(pin: &Pin<'_>, sh: &Shapes, ap: &mut AccessPoint, try_all: bool, trace: &mut Trace) {
    if !ap.allow_via {
        return;
    }
    let cfg = pin.cfg;
    let layer = ap.layer;
    let must_in_pin = in_via_in_pin(cfg, layer)
        || (ap.lower == ApType::EncOpt && ap.upper != ApType::NearbyGrid)
        || (ap.upper == ApType::EncOpt && ap.lower != ApType::NearbyGrid);
    let max_vias: i64 = if try_all { i64::from(i32::MAX) } else { 2 };
    let mut via_defs: Vec<usize> = Vec::new();
    if layer < cfg.top_routing_layer {
        via_defs.extend(get_priority_via_defs(pin, layer + 1, max_vias));
    }
    if pin.kind == TermKind::Io && layer > pin.cx.tech.bottom_layer_num() && layer - 1 <= cfg.top_routing_layer {
        let remaining = max_vias - via_defs.len() as i64;
        via_defs.extend(get_priority_via_defs(pin, layer - 1, remaining));
    }
    push(trace, || Event::VList { point: ap.point, layer, try_all, must_in_pin, vias: via_defs.clone() });
    let mut valid = 0;
    for v in via_defs {
        let vd = &pin.cx.tech.via_defs[v];
        if let (Some(b), false) = (pin.boundary, try_all) {
            if !contains_rect(b, shift(vd.layer1_bbox(), ap.point)) {
                push(trace, || Event::VTry { via: v, skip: Some("B1"), ext: None });
                continue;
            }
            if !contains_rect(b, shift(vd.layer2_bbox(), ap.point)) {
                push(trace, || Event::VTry { via: v, skip: Some("B2"), ext: None });
                continue;
            }
        }
        let ext = via_max_ext(pin, ap, &sh.rects[layer], vd);
        if must_in_pin && ext > 0 {
            push(trace, || Event::VTry { via: v, skip: Some("P"), ext: Some(ext) });
            continue;
        }
        push(trace, || Event::VTry { via: v, skip: None, ext: Some(ext) });
        if check_via_planar_access(pin, sh, ap, v, trace) {
            ap.vias.push(v);
            if vd.layer1 == layer {
                ap.set(Access::U, true);
            } else {
                ap.set(Access::D, true);
            }
            valid += 1;
            if valid >= 2 {
                break;
            }
        }
    }
}

fn shift(r: Rect, p: (i32, i32)) -> Rect {
    Rect::new(r.xl + p.0, r.yl + p.1, r.xh + p.0, r.yh + p.1)
}

fn contains_rect(outer: Rect, inner: Rect) -> bool {
    outer.xl <= inner.xl && outer.yl <= inner.yl && inner.xh <= outer.xh && inner.yh <= outer.yh
}

fn check_via_planar_access(pin: &Pin<'_>, sh: &Shapes, ap: &AccessPoint, v: usize, trace: &mut Trace) -> bool {
    PLANAR.iter().any(|&d| check_directional_via_access(pin, sh, ap, v, d, trace))
}

/// The via, and a segment leaving it on its other layer in `d`: wrong-way only with non-preferred
/// tracks; the end three of that layer's widths out.
fn check_directional_via_access(pin: &Pin<'_>, sh: &Shapes, ap: &AccessPoint, v: usize, d: Access, trace: &mut Trace) -> bool {
    let tech = pin.cx.tech;
    let vd = &tech.via_defs[v];
    let target_layer = if vd.layer1 == ap.layer { vd.layer2 } else { vd.layer1 };
    let tl = &tech.layers[target_layer];
    let vert = matches!(d, Access::S | Access::N);
    let wrong = (tl.is_horizontal() && vert) || (tl.is_vertical() && !vert);
    if wrong && !pin.cfg.use_nonpref_tracks {
        return false;
    }
    let end = gen_end_point(pin, &sh.rects[ap.layer], ap.point, target_layer, d);
    let markers = via_markers(tech, pin.target, pin.owner, ap.point, ap.layer, vd, end);
    let ok = markers.is_empty();
    push(trace, || Event::Via { point: ap.point, layer: ap.layer, via: v, seg: stored(d, ap.point, end), markers });
    ok
}

/// How far the via's lower-layer shape sticks out of the pin: over the overlap's horizontal slices
/// the x overhang; unless the point is within three widths of the cell's left or right side, also
/// the y overhang (over vertical slices when there is more than one). No overlap, no overhang.
pub fn via_max_ext(pin: &Pin<'_>, ap: &AccessPoint, pin_rects: &[Rect], vd: &ViaDef) -> i32 {
    let bx = shift(vd.layer1_bbox(), ap.point);
    let w = pin.cx.tech.layers[ap.layer].width;
    let side_bound = pin.boundary.is_some_and(|b| ap.point.0 <= b.xl + 3 * w || ap.point.0 >= b.xh - 3 * w);
    let pieces: Vec<Rect> = pin_rects
        .iter()
        .filter_map(|r| {
            // Not `Rect::new`: it orders the corners, and would turn an empty overlap into a box.
            let c = Rect { xl: r.xl.max(bx.xl), yl: r.yl.max(bx.yl), xh: r.xh.min(bx.xh), yh: r.yh.min(bx.yh) };
            (c.xl < c.xh && c.yl < c.yh).then_some(c)
        })
        .collect();
    let horizontal = slices(&pieces, false);
    let mut max_ext = 0;
    for r in &horizontal {
        max_ext = max_ext.max(bx.xh - r.xh).max(r.xl - bx.xl);
    }
    if !side_bound {
        let rs = if horizontal.len() > 1 { slices(&pieces, true) } else { horizontal };
        for r in &rs {
            max_ext = max_ext.max(bx.yh - r.yh).max(r.yl - bx.yl);
        }
    }
    max_ext
}

/// The union of `pieces` sliced into rectangles: horizontally (the set's own slicing), or
/// vertically (transposed).
fn slices(pieces: &[Rect], vertical: bool) -> Vec<Rect> {
    let t = |r: &Rect| if vertical { Rect::new(r.yl, r.xl, r.yh, r.xh) } else { *r };
    let mut set = Polygon90Set::new();
    for r in pieces {
        set.insert_rect(t(r));
    }
    set.rectangles().iter().map(t).collect()
}

/// A top-level port needs one point; any other pin enough SPARSE points: one per point that has no
/// later same-layer point within half the layer's width of it.
fn enough_access_points(pin: &Pin<'_>, aps: &[AccessPoint]) -> bool {
    if pin.kind == TermKind::Io {
        return !aps.is_empty();
    }
    enough_sparse_points(pin, aps)
}

fn enough_sparse_points(pin: &Pin<'_>, aps: &[AccessPoint]) -> bool {
    let mut n = aps.len();
    for i in 0..aps.len() {
        let h = pin.cx.tech.layers[aps[i].layer].width / 2;
        let b = Rect::new(aps[i].point.0 - h, aps[i].point.1 - h, aps[i].point.0 + h, aps[i].point.1 + h);
        if aps[i + 1..].iter().any(|a| a.layer == aps[i].layer && b.contains(a.point.0, a.point.1)) {
            n -= 1;
        }
    }
    match pin.kind {
        TermKind::StdCell => n >= pin.cfg.min_std_cell_points,
        TermKind::Macro => n >= pin.cfg.min_macro_points,
        TermKind::Io => false,
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    /// The gc test technology with one via: l2 → c3 → l4 (170 square below and in the cut, 140
    /// square above).
    fn tech() -> crate::tech::Tech {
        let mut t = crate::gc::tests::tech();
        let sq = |h: i32| Rect::new(-h, -h, h, h);
        t.via_defs.push(ViaDef { name: "v".into(), is_default: true, layer1: 2, cut: 3, layer2: 4, layer1_figs: vec![sq(85)], cut_figs: vec![sq(85)], layer2_figs: vec![sq(70)] });
        t
    }

    fn cfg(use_nonpref_tracks: bool) -> Config {
        Config { via_access_layer: 2, via_in_pin: None, top_routing_layer: 4, min_std_cell_points: 3, min_macro_points: 3, use_nonpref_tracks }
    }

    fn shapes(t: &crate::tech::Tech, rects: &[(usize, Rect)]) -> Shapes {
        let mut sets = merge_pin_shapes(t, rects);
        let rects = sets.iter_mut().map(|s| s.rectangles()).collect();
        Shapes { sets, rects }
    }

    fn ap(point: (i32, i32), layer: usize) -> AccessPoint {
        AccessPoint { point, layer, lower: ApType::OnGrid, upper: ApType::OnGrid, access: [false; 6], allow_via: true, vias: vec![] }
    }

    /// Two more vias on the same cut: `w` (wide below: 400 × 170) and `x` (like `v`, but not a
    /// default via — an equal priority would REPLACE `v`, see the next test).
    fn tech3() -> crate::tech::Tech {
        let mut t = tech();
        let sq = |h: i32| Rect::new(-h, -h, h, h);
        t.via_defs.push(ViaDef { name: "w".into(), is_default: true, layer1: 2, cut: 3, layer2: 4, layer1_figs: vec![Rect::new(-200, -85, 200, 85)], cut_figs: vec![sq(85)], layer2_figs: vec![sq(70)] });
        t.via_defs.push(ViaDef { name: "x".into(), is_default: false, layer1: 2, cut: 3, layer2: 4, layer1_figs: vec![sq(85)], cut_figs: vec![sq(85)], layer2_figs: vec![sq(70)] });
        t
    }

    /// Vias of EQUAL priority on a cut layer: the later one replaces the earlier (the reference
    /// assigns into a map keyed by priority).
    #[test]
    fn an_equal_priority_via_replaces_the_earlier() {
        let mut t = tech();
        let v2 = ViaDef { name: "v2".into(), ..t.via_defs[0].clone() };
        t.via_defs.push(v2);
        assert_eq!(crate::pa::candidates::via_priority(&t)[&3], vec![1]);
    }

    /// Retrying tries EVERY via, but still keeps at most two.
    #[test]
    fn a_retry_keeps_at_most_two_vias() {
        let t = tech3();
        let cx = Context::new(&t, &[]);
        let (c, owner) = (cfg(true), Owner::Net("n".into()));
        let pin = Pin { cx: &cx, cfg: &c, kind: TermKind::StdCell, is_block: false, boundary: None, target: &[], owner: &owner };
        let sh = shapes(&t, &[(2, Rect::new(-2000, -2000, 2000, 2000))]);
        let mut a = ap((0, 0), 2);
        let mut trace = Some(Vec::new());
        filter_via_access(&pin, &sh, &mut a, true, &mut trace);
        let listed = trace.unwrap().iter().find_map(|e| if let Event::VList { vias, .. } = e { Some(vias.len()) } else { None });
        assert_eq!((listed, a.vias.len()), (Some(3), 2));
    }

    /// Vias are ordered by how far their lower shape sticks out of the pin: on a 300-wide pin the
    /// 400-wide `w` sticks out 50 (x overhang), `v` not at all — so `v` comes first.
    #[test]
    fn vias_order_by_how_far_they_stick_out() {
        let t = tech3();
        let cx = Context::new(&t, &[]);
        let (c, owner) = (cfg(true), Owner::Net("n".into()));
        let pin = Pin { cx: &cx, cfg: &c, kind: TermKind::StdCell, is_block: false, boundary: None, target: &[], owner: &owner };
        let w = t.via_defs.iter().position(|v| v.name == "w").unwrap();
        let mut a = AccessPoint { vias: vec![w, 0], ..ap((0, 0), 2) };
        let exts = sort_via_defs(&pin, &mut a, &[Rect::new(-150, -1000, 150, 1000)]);
        assert_eq!((a.vias.clone(), exts), (vec![0, w], vec![0, 50]));
    }

    /// The database's access bits run N, S, E, W, U, D — not the point's own E, W, S, N order.
    #[test]
    fn database_access_bits() {
        let mut a = ap((0, 0), 2);
        a.set(Access::N, true);
        a.set(Access::W, true);
        a.set(Access::U, true);
        assert_eq!(a.db_access_bits(), 1 | 8 | 16);
        let mut b = ap((0, 0), 2);
        b.set(Access::E, true);
        b.set(Access::D, true);
        assert_eq!(b.db_access_bits(), 4 | 32);
    }

    /// A top-level port also tries the vias of the cut layer BELOW; a via whose lower layer is not
    /// the point's gives a DOWN access. (Here the port is on the top routing layer: no via up.)
    #[test]
    fn a_port_also_tries_the_vias_below() {
        let t = tech();
        let cx = Context::new(&t, &[]);
        let (c, owner) = (cfg(true), Owner::Net("n".into()));
        let pin = Pin { cx: &cx, cfg: &c, kind: TermKind::Io, is_block: false, boundary: None, target: &[], owner: &owner };
        let sh = shapes(&t, &[(4, Rect::new(-2000, -2000, 2000, 2000))]);
        let mut a = ap((0, 0), 4);
        filter_via_access(&pin, &sh, &mut a, false, &mut None);
        assert_eq!((a.vias.clone(), a.has(Access::D), a.has(Access::U)), (vec![0], true, false));
    }

    /// The priority list stops only after adding: asked for none, it still gives one.
    #[test]
    fn priority_vias_take_at_least_one() {
        let t = tech();
        let cx = Context::new(&t, &[]);
        let (c, owner) = (cfg(true), Owner::Net("n".into()));
        let pin = Pin { cx: &cx, cfg: &c, kind: TermKind::Io, is_block: false, boundary: None, target: &[], owner: &owner };
        assert_eq!(get_priority_via_defs(&pin, 3, 0), vec![0]);
        assert_eq!(get_priority_via_defs(&pin, 3, 2), vec![0]);
    }

    /// A cell pin's via whose lower shape leaves the cell's box is skipped untried (not when
    /// retrying).
    #[test]
    fn a_via_leaving_the_cell_is_skipped() {
        let t = tech();
        let cx = Context::new(&t, &[]);
        let (c, owner) = (cfg(true), Owner::Net("n".into()));
        let pin = Pin { cx: &cx, cfg: &c, kind: TermKind::StdCell, is_block: false, boundary: Some(Rect::new(-80, -500, 500, 500)), target: &[], owner: &owner };
        let sh = shapes(&t, &[(2, Rect::new(-2000, -2000, 2000, 2000))]);
        let mut a = ap((0, 0), 2);
        let mut trace = Some(Vec::new());
        filter_via_access(&pin, &sh, &mut a, false, &mut trace);
        assert!(a.vias.is_empty());
        assert!(trace.unwrap().contains(&Event::VTry { via: 0, skip: Some("B1"), ext: None }));
        let mut b = ap((0, 0), 2);
        filter_via_access(&pin, &sh, &mut b, true, &mut None);
        assert_eq!(b.vias, vec![0]);
    }

    /// Without non-preferred tracks a planar access runs only along the layer (l2 is vertical).
    #[test]
    fn without_nonpreferred_tracks_no_wrong_way_access() {
        let t = tech();
        let cx = Context::new(&t, &[]);
        let owner = Owner::Net("n".into());
        let c = cfg(false);
        let pin = Pin { cx: &cx, cfg: &c, kind: TermKind::Macro, is_block: false, boundary: None, target: &[], owner: &owner };
        let a = create_access_point(&pin, (0, 0), 2, ApType::OnGrid, ApType::OnGrid);
        assert_eq!([a.has(Access::N), a.has(Access::S), a.has(Access::E), a.has(Access::W)], [true, true, false, false]);
        let c2 = cfg(true);
        let pin2 = Pin { cfg: &c2, ..pin };
        assert!(create_access_point(&pin2, (0, 0), 2, ApType::OnGrid, ApType::OnGrid).has(Access::E));
    }

    /// A block's planar segment ends a pitch past the pin's extent, not three widths out.
    #[test]
    fn a_block_pin_segment_ends_a_pitch_past_the_pin() {
        let t = tech();
        let cx = Context::new(&t, &[]);
        let (c, owner) = (cfg(true), Owner::Net("n".into()));
        let pin = Pin { cx: &cx, cfg: &c, kind: TermKind::Macro, is_block: true, boundary: None, target: &[], owner: &owner };
        let rects = [Rect::new(0, 0, 1000, 200)];
        assert_eq!(gen_end_point(&pin, &rects, (500, 100), 2, Access::E), (1000 + 480, 100));
        assert_eq!(gen_end_point(&Pin { is_block: false, ..pin }, &rects, (500, 100), 2, Access::E), (500 + 3 * 170, 100));
    }
}
