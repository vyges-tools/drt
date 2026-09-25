// SPDX-License-Identifier: Apache-2.0
//! Route guides for detailed routing: a net's global-route guides turned into a connected set of
//! one-track guides, each running along its layer between gcell centres, and the gcell where each
//! pin is reached (its "gr pin").
//!
//! Stages, in order ([`gen_guides`]): guides patched so every pin is covered ([`cover_pins`]); the
//! guides as intervals per layer and track, with bridges between touching ones ([`gen_guides_prep`]);
//! each pin's gcells ([`init_gcell_pin_map`]); up to four rounds of splitting the intervals at
//! crossings and pins ([`gen_guides_split`]) and searching a tree through the pieces from the first
//! pin ([`PathFinder`]), a bridge added between disconnected parts on the third; the tree's pieces
//! clipped, merged and committed as guides.
//!
//! Rules:
//! - intervals on a track JOIN when they overlap or touch (closed integer intervals);
//! - the search is a shortest path over pieces, cost one per hop plus a piece's gcell span, with a
//!   TOTAL tie order (cost, then node, then predecessor); pins are passed through, except that a
//!   pin already on the tree re-enters at cost 2 (10 when forced, in the last rounds);
//! - pieces cross between layers only at shared gcells two layers apart, and never along the layer
//!   at or below the via-access layer;
//! - committed guides are sorted by layer, then box (x low, y low, x high, y high), duplicates
//!   dropped.

use std::cmp::Reverse;
use std::collections::{BTreeMap, BTreeSet, BinaryHeap, VecDeque};

use crate::polygon90::Rect;
use crate::tech::Tech;

/// A point with a layer, ordered x, then y, then layer.
pub type P3 = (i32, i32, usize);

/// The gcell grid: per axis, the first line, the number of gcells and their size; and the die.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GCellGrid {
    pub x: (i32, i32, i32),
    pub y: (i32, i32, i32),
    pub die: Rect,
}

impl GCellGrid {
    /// The gcell holding a point, clamped into the grid.
    pub fn idx(&self, p: (i32, i32)) -> (i32, i32) {
        let ax = |v: i32, (start, count, step): (i32, i32, i32)| ((v - start) / step).max(0).min(count - 1);
        (ax(p.0, self.x), ax(p.1, self.y))
    }

    /// The gcells a point touches: on a gcell line, both sides (not below the first gcell).
    pub fn indices(&self, p: (i32, i32)) -> Vec<(i32, i32)> {
        let ax = |v: i32, (start, count, step): (i32, i32, i32)| {
            let c = (v - start).clamp(0, step * count - 1);
            let base = c / step;
            let mut s = BTreeSet::from([base]);
            if c % step == 0 && base != 0 {
                s.insert(base - 1);
            }
            s
        };
        let (xs, ys) = (ax(p.0, self.x), ax(p.1, self.y));
        xs.iter().flat_map(|&x| ys.iter().map(move |&y| (x, y))).collect()
    }

    /// A gcell's box; the edge gcells reach the die.
    pub fn gcell_box(&self, idx: (i32, i32)) -> Rect {
        let i = (idx.0.clamp(0, self.x.1 - 1), idx.1.clamp(0, self.y.1 - 1));
        let (mut xl, mut yl) = (self.x.2 * i.0 + self.x.0, self.y.2 * i.1 + self.y.0);
        let (mut xh, mut yh) = (self.x.2 * (i.0 + 1) + self.x.0, self.y.2 * (i.1 + 1) + self.y.0);
        if i.0 <= 0 {
            xl = self.die.xl;
        }
        if i.1 <= 0 {
            yl = self.die.yl;
        }
        if i.0 >= self.x.1 - 1 {
            xh = self.die.xh;
        }
        if i.1 >= self.y.1 - 1 {
            yh = self.die.yh;
        }
        Rect { xl, yl, xh, yh }
    }

    /// A gcell's centre (the edge gcells' boxes reach the die).
    pub fn center(&self, idx: (i32, i32)) -> (i32, i32) {
        let (mut xl, mut yl) = (self.x.2 * idx.0 + self.x.0, self.y.2 * idx.1 + self.y.0);
        let (mut xh, mut yh) = (self.x.2 * (idx.0 + 1) + self.x.0, self.y.2 * (idx.1 + 1) + self.y.0);
        if idx.0 == 0 {
            xl = self.die.xl;
        }
        if idx.1 == 0 {
            yl = self.die.yl;
        }
        if idx.0 == self.x.1 - 1 {
            xh = self.die.xh;
        }
        if idx.1 == self.y.1 - 1 {
            yh = self.die.yh;
        }
        ((xl + xh) / 2, (yl + yh) / 2)
    }
}

/// The grid from the guides themselves, when the design has none: the most common guide width
/// across each direction (the smallest on a tie) and the most common offset, the first line at
/// or below the die's edge.
pub fn build_gcell_patterns(tech: &Tech, die: Rect, guides: &[(usize, Rect)]) -> Option<GCellGrid> {
    let most = |m: &BTreeMap<i32, i32>| -> Option<i32> {
        let mut best: Option<(i32, i32)> = None;
        for (&v, &c) in m {
            if best.is_none_or(|(_, bc)| c > bc) {
                best = Some((v, c));
            }
        }
        best.map(|b| b.0)
    };
    let (mut wx, mut wy) = (BTreeMap::new(), BTreeMap::new());
    for &(l, r) in guides {
        if tech.layers[l].is_horizontal() {
            *wy.entry(r.dy()).or_insert(0) += 1;
        } else if tech.layers[l].is_vertical() {
            *wx.entry(r.dx()).or_insert(0) += 1;
        }
    }
    let (gx, gy) = (most(&wx)?, most(&wy)?);
    let (mut ox, mut oy) = (BTreeMap::new(), BTreeMap::new());
    for &(_, r) in guides {
        let (mut x, mut y) = (r.xl % gx, r.yl % gy);
        if x < 0 {
            x = gx - x;
        }
        if y < 0 {
            y = gy - y;
        }
        *ox.entry(x).or_insert(0) += 1;
        *oy.entry(y).or_insert(0) += 1;
    }
    let (offx, offy) = (most(&ox)?, most(&oy)?);
    let mut sx = die.xl / gx * gx + offx;
    if sx > die.xl {
        sx -= gx;
    }
    let mut sy = die.yl / gy * gy + offy;
    if sy > die.yl {
        sy -= gy;
    }
    Some(GCellGrid { x: (sx, (die.xh - sx) / gx, gx), y: (sy, (die.yh - sy) / gy, gy), die })
}

/// Closed integer intervals that join when they overlap or touch.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct IntervalSet(BTreeMap<i32, i32>);

impl IntervalSet {
    pub fn insert(&mut self, lo: i32, hi: i32) {
        let (mut lo, mut hi) = (lo, hi);
        let touching: Vec<i32> = self.0.iter().filter(|(&a, &b)| a <= hi.saturating_add(1) && b.saturating_add(1) >= lo).map(|(&a, _)| a).collect();
        for a in touching {
            let b = self.0.remove(&a).expect("present");
            lo = lo.min(a);
            hi = hi.max(b);
        }
        self.0.insert(lo, hi);
    }
    pub fn contains(&self, v: i32) -> bool {
        self.0.range(..=v).next_back().is_some_and(|(_, &b)| v <= b)
    }
    pub fn iter(&self) -> impl Iterator<Item = (i32, i32)> + '_ {
        self.0.iter().map(|(&a, &b)| (a, b))
    }
    pub fn intersect(&self, other: &IntervalSet) -> IntervalSet {
        let mut out = IntervalSet::default();
        for (a, b) in self.iter() {
            for (c, d) in other.iter() {
                let (lo, hi) = (a.max(c), b.min(d));
                if lo <= hi {
                    out.insert(lo, hi);
                }
            }
        }
        out
    }
}

/// Per layer, per track index, the guide intervals along it (gcell indices).
pub type TrackIntervals = Vec<BTreeMap<i32, IntervalSet>>;

/// The gr pins: (pin, gcell centre).
pub type GrPins = Vec<(usize, (i32, i32))>;

/// A pin as guide processing reads it.
#[derive(Debug, Clone)]
pub struct GuidePin {
    pub name: String,
    /// A port (ordered before instance terminals in the gcell maps).
    pub is_port: bool,
    /// Its order among pins of its kind (the database id order).
    pub id: usize,
    /// Its shapes, design coordinates.
    pub shapes: Vec<(usize, Rect)>,
    /// Every access point of every pin (design coordinates, layer).
    pub aps: Vec<((i32, i32), usize)>,
}

/// The router settings guide processing reads.
#[derive(Debug, Clone, Copy)]
pub struct GuideConfig {
    pub bottom_routing_layer: usize,
    pub top_routing_layer: usize,
    pub via_access_layer: usize,
    pub allow_pin_feedthrough: bool,
}

/// A committed guide: from one gcell centre to another, on a layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Guide {
    pub begin: (i32, i32),
    pub end: (i32, i32),
    pub layer: usize,
}

/// What guide processing did for a net, in order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Event {
    Covered(Vec<(usize, Rect)>),
    Round { i: usize, rects: Vec<(usize, Rect)> },
    Traverse(bool),
    Bridge,
    Guides(Vec<Guide>),
    /// Each gr pin: the pin's index in the net's pin order, and the gcell centre.
    GrPins(Vec<(usize, (i32, i32))>),
}

pub struct GuideNet<'a> {
    pub tech: &'a Tech,
    pub grid: &'a GCellGrid,
    pub cfg: &'a GuideConfig,
    /// The net's instance terminals, then its ports — the search's pin order.
    pub pins: Vec<GuidePin>,
}


impl GuideNet<'_> {
    fn horizontal(&self, l: usize) -> bool {
        self.tech.layers[l].is_horizontal()
    }
    fn vertical(&self, l: usize) -> bool {
        self.tech.layers[l].is_vertical()
    }
}

// ---- patching guides to cover pins ----

fn manhattan(r: &Rect, p: (i32, i32)) -> i32 {
    (p.0 - p.0.clamp(r.xl, r.xh)).abs() + (p.1 - p.1.clamp(r.yl, r.yh)).abs()
}

fn is_pin_covered_by_guides(pin: &GuidePin, guides: &[(usize, Rect)]) -> bool {
    pin.aps.iter().any(|&(p, l)| guides.iter().any(|&(gl, r)| gl == l && r.contains(p.0, p.1)))
}

/// The gcell (and layer) nearest the guides with the most access points; the first in (x, y,
/// layer) order on a tie.
fn find_best_pin_location(net: &GuideNet<'_>, pin: &GuidePin, guides: &[(usize, Rect)]) -> P3 {
    let mut count: BTreeMap<P3, i32> = BTreeMap::new();
    let mut min_dist = i32::MAX;
    for &(p, l) in &pin.aps {
        let idx = net.grid.idx(p);
        let c = net.grid.center(idx);
        for (_, r) in guides {
            let d = manhattan(r, c);
            if d < min_dist {
                count.clear();
                min_dist = d;
                *count.entry((idx.0, idx.1, l)).or_insert(0) += 1;
            } else if d == min_dist {
                *count.entry((idx.0, idx.1, l)).or_insert(0) += 1;
            }
        }
    }
    let (mut best, mut high) = ((0, 0, 0), 0);
    for (&k, &c) in &count {
        if c > high {
            best = k;
            high = c;
        }
    }
    best
}

fn find_closest_guide(net: &GuideNet<'_>, best: P3, guides: &[(usize, Rect)], penalty: i32) -> usize {
    let (mut closest, mut min) = (0, i32::MAX);
    for (k, &(l, r)) in guides.iter().enumerate() {
        let mut d = manhattan(&r, (best.0, best.1)) + (l as i32 - best.2 as i32).abs() * penalty;
        if l < net.cfg.bottom_routing_layer {
            d += 1_000_000_000;
        }
        if d < min {
            min = d;
            closest = k;
        }
    }
    closest
}

fn adjust_guide_point(pt: &mut P3, b: &Rect, hh: i32, hv: i32) {
    pt.0 = if (b.xl - pt.0).abs() <= (b.xh - pt.0).abs() { b.xl + hh } else { b.xh - hh };
    pt.1 = if (b.yl - pt.1).abs() <= (b.yh - pt.1).abs() { b.yl + hv } else { b.yh - hv };
}

fn extend_guide(net: &GuideNet<'_>, best: (i32, i32), hh: i32, hv: i32, guide: &mut Rect, pt: &mut P3) {
    let b = *guide;
    if net.horizontal(pt.2) {
        if pt.0 != best.0 {
            if best.0 < b.xl {
                guide.xl = best.0 - hh;
            } else if best.0 > b.xh {
                guide.xh = best.0 + hh;
            }
            pt.0 = best.0;
        }
    } else if net.vertical(pt.2) && pt.1 != best.1 {
        if best.1 < b.yl {
            guide.yl = best.1 - hv;
        } else if best.1 > b.yh {
            guide.yh = best.1 + hv;
        }
        pt.1 = best.1;
    }
}

fn connect_guides_with_best_pin_loc(net: &GuideNet<'_>, pt: &mut P3, best: (i32, i32), hh: i32, hv: i32, guides: &mut Vec<(usize, Rect)>) {
    if pt.0 == best.0 && pt.1 == best.1 {
        return;
    }
    let pl = (best.0.min(pt.0), best.1.min(pt.1));
    let ph = (best.0.max(pt.0), best.1.max(pt.1));
    let (horz, vert) = (pl.0 != ph.0, pl.1 != ph.1);
    let mut layer = pt.2;
    if horz ^ vert && ((vert && net.horizontal(layer)) || (horz && net.vertical(layer))) {
        if layer + 2 <= net.cfg.top_routing_layer {
            layer += 2;
        } else {
            layer -= 2;
        }
    }
    pt.2 = layer;
    guides.push((layer, Rect { xl: pl.0 - hh, yl: pl.1 - hv, xh: ph.0 + hh, yh: ph.1 + hv }));
}

fn fill_guides_up_to_z(best: P3, start_z: usize, hh: i32, hv: i32, guides: &mut Vec<(usize, Rect)>) {
    let inc: i64 = if start_z < best.2 { 2 } else { -2 };
    let mut z = start_z as i64 + inc;
    while z != best.2 as i64 + inc {
        guides.push((z as usize, Rect { xl: best.0 - hh, yl: best.1 - hv, xh: best.0 + hh, yh: best.1 + hv }));
        z += inc;
    }
}

fn patch_guides(net: &GuideNet<'_>, pin: &GuidePin, guides: &mut Vec<(usize, Rect)>) {
    if is_pin_covered_by_guides(pin, guides) {
        return;
    }
    let idx = find_best_pin_location(net, pin, guides);
    let c = net.grid.center((idx.0, idx.1));
    let best = (c.0, c.1, idx.2);
    let closest = find_closest_guide(net, best, guides, 1);
    // patchGuides_helper
    let (gl, g) = guides[closest];
    let mut pt: P3 = (best.0.clamp(g.xl, g.xh), best.1.clamp(g.yl, g.yh), gl);
    let (hh, hv) = (net.grid.x.2 / 2, net.grid.y.2 / 2);
    adjust_guide_point(&mut pt, &g, hh, hv);
    let mut guide = guides[closest].1;
    extend_guide(net, (best.0, best.1), hh, hv, &mut guide, &mut pt);
    guides[closest].1 = guide;
    if pt == best {
        return;
    }
    connect_guides_with_best_pin_loc(net, &mut pt, (best.0, best.1), hh, hv, guides);
    fill_guides_up_to_z(best, pt.2, hh, hv, guides);
}

/// Every pin covered: instance terminals, then ports.
pub fn cover_pins(net: &GuideNet<'_>, guides: &mut Vec<(usize, Rect)>) {
    let order: Vec<usize> = (0..net.pins.len()).filter(|&i| !net.pins[i].is_port).chain((0..net.pins.len()).filter(|&i| net.pins[i].is_port)).collect();
    for i in order {
        patch_guides(net, &net.pins[i], guides);
    }
}

// ---- intervals ----

fn init_guide_intervals(net: &GuideNet<'_>, rects: &[(usize, Rect)], intvs: &mut TrackIntervals) {
    for &(l, r) in rects {
        let (x1, y1) = net.grid.idx((r.xl, r.yl));
        let (x2, y2) = net.grid.idx((r.xh - 1, r.yh - 1));
        if net.horizontal(l) {
            for t in y1..=y2 {
                intvs[l].entry(t).or_default().insert(x1, x2);
            }
        } else {
            for t in x1..=x2 {
                intvs[l].entry(t).or_default().insert(y1, y2);
            }
        }
    }
}

fn gcell_has_guide(pivot: i32, other: i32, curr_layer: usize, intvs: &TrackIntervals) -> bool {
    let (mut pivot, mut other) = (pivot, other);
    if curr_layer % 4 == 2 {
        std::mem::swap(&mut pivot, &mut other);
    }
    let mut l = 0;
    while l < intvs.len() {
        if intvs[l].get(&pivot).is_some_and(|s| s.contains(other)) {
            return true;
        }
        if curr_layer.is_multiple_of(2) {
            std::mem::swap(&mut pivot, &mut other);
        }
        l += 2;
    }
    false
}

fn has_guide_interval(begin: i32, end: i32, t1: i32, t2: i32, layer: i64, intvs: &TrackIntervals) -> bool {
    if layer < 0 || layer as usize >= intvs.len() {
        return false;
    }
    intvs[layer as usize].range(begin..).take_while(|(&k, _)| k <= end).any(|(_, s)| s.contains(t1) && s.contains(t2))
}

/// A bridge on a neighbouring layer: along `track`, from `begin` to `end`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct BridgeGuide {
    track: i32,
    begin: i32,
    end: i32,
    layer: i64,
}

impl BridgeGuide {
    fn new(track: i32, a: i32, b: i32, layer: i64) -> BridgeGuide {
        BridgeGuide { track, begin: a.min(b), end: a.max(b), layer }
    }
    fn none() -> BridgeGuide {
        BridgeGuide { track: -1, begin: -1, end: -1, layer: -1 }
    }
    fn dist(&self) -> i32 {
        if self.layer == -1 {
            i32::MAX
        } else {
            self.end - self.begin
        }
    }
    /// Shorter first; on equal length the HIGHER layer.
    fn less(&self, o: &BridgeGuide) -> bool {
        if self.dist() == o.dist() {
            self.layer > o.layer
        } else {
            self.dist() < o.dist()
        }
    }
}

/// Corner-touching intervals on consecutive tracks get a one-gcell overlap; tracks that run side
/// by side get a bridge on the layer below or above when neither has one.
fn add_touching_guides_bridges(intvs: &mut TrackIntervals) {
    for l in (0..intvs.len()).rev() {
        let tracks: Vec<i32> = intvs[l].keys().copied().collect();
        let mut prev: i32 = -2;
        for &t in &tracks {
            if t == prev + 1 {
                // ⚠️ The reference inserts into these sets while iterating them; a snapshot is the
                // reading that does not depend on the container's invalidation behaviour.
                let cur: Vec<(i32, i32)> = intvs[l][&t].iter().collect();
                let prv: Vec<(i32, i32)> = intvs[l][&prev].iter().collect();
                for &(a1, b1) in &cur {
                    for &(a2, b2) in &prv {
                        if a1.max(a2) <= b1.min(b2) {
                            continue;
                        }
                        if b1 + 1 == a2 && !gcell_has_guide(t, b1 + 1, l, intvs) && !gcell_has_guide(prev, b1, l, intvs) {
                            intvs[l].get_mut(&t).expect("track").insert(b1, b1 + 1);
                        } else if b2 + 1 == a1 && !gcell_has_guide(prev, b2 + 1, l, intvs) && !gcell_has_guide(t, b2, l, intvs) {
                            intvs[l].get_mut(&prev).expect("track").insert(b2, b2 + 1);
                        }
                    }
                }
            }
            prev = t;
        }
    }
    let mut bridges = Vec::new();
    for l in 0..intvs.len() {
        let mut prev: i32 = -2;
        for (&t, set) in &intvs[l] {
            if t == prev + 1 {
                let inter = set.intersect(&intvs[l][&prev]);
                for (begin, end) in inter.iter() {
                    let mut has = false;
                    let mut bridge_layer: Option<i64> = None;
                    if l >= 2 {
                        bridge_layer = Some(l as i64 - 2);
                        has = has_guide_interval(begin, end, t, prev, l as i64 - 2, intvs);
                    }
                    if l + 2 < intvs.len() && !has {
                        bridge_layer = Some(l as i64 + 2);
                        has = has_guide_interval(begin, end, t, prev, l as i64 + 2, intvs);
                    }
                    if !has {
                        let bl = bridge_layer.expect("a bridge layer");
                        bridges.push(BridgeGuide::new(begin, prev, t, bl));
                    }
                }
            }
            prev = t;
        }
    }
    for b in bridges {
        intvs[b.layer as usize].entry(b.track).or_default().insert(b.begin, b.end);
    }
}

pub fn gen_guides_prep(net: &GuideNet<'_>, rects: &[(usize, Rect)], intvs: &mut TrackIntervals) {
    init_guide_intervals(net, rects, intvs);
    add_touching_guides_bridges(intvs);
}

// ---- pins' gcells ----

/// Pins by gcell (layer): each access point's gcells.
pub fn init_gcell_pin_map(net: &GuideNet<'_>) -> BTreeMap<P3, BTreeSet<usize>> {
    let mut m: BTreeMap<P3, BTreeSet<usize>> = BTreeMap::new();
    let order: Vec<usize> = (0..net.pins.len()).filter(|&i| !net.pins[i].is_port).chain((0..net.pins.len()).filter(|&i| net.pins[i].is_port)).collect();
    for i in order {
        for &(p, l) in &net.pins[i].aps {
            for (x, y) in net.grid.indices(p) {
                m.entry((x, y, l)).or_default().insert(i);
            }
        }
    }
    m
}

// ---- splitting ----

fn split_rect(track: i32, begin: i32, end: i32, layer: usize, horizontal: bool, rects: &mut Vec<(usize, Rect)>) {
    let r = if horizontal { Rect { xl: begin, yl: track, xh: end, yh: track } } else { Rect { xl: track, yl: begin, xh: track, yh: end } };
    rects.push((layer, r));
}

/// The intervals cut into pieces (gcell indices): at crossings with the layers below and above and
/// at pins; below the bottom routing layer (and, in the first round, at or below the via-access
/// layer) every gcell its own piece.
pub fn gen_guides_split(net: &GuideNet<'_>, intvs: &TrackIntervals, gcell_pin_map: &BTreeMap<P3, BTreeSet<usize>>, pin_gcell_map: &mut BTreeMap<usize, BTreeSet<P3>>, via_access_only: bool) -> Vec<(usize, Rect)> {
    let mut rects = Vec::new();
    // layer -> track (across) -> index along -> pins
    let mut pin_helper: Vec<BTreeMap<i32, BTreeMap<i32, BTreeSet<usize>>>> = vec![BTreeMap::new(); net.tech.layers.len()];
    for (&(x, y, l), pins) in gcell_pin_map {
        if net.horizontal(l) {
            pin_helper[l].entry(y).or_default().insert(x, pins.clone());
        } else {
            pin_helper[l].entry(x).or_default().insert(y, pins.clone());
        }
    }
    let split_by_pins = |layer: usize, horizontal: bool, track: i32, begin: i32, end: i32, pin_gcell_map: &mut BTreeMap<usize, BTreeSet<P3>>, split: &mut BTreeSet<i32>| {
        if let Some(along) = pin_helper[layer].get(&track) {
            for (&idx, pins) in along.range(begin..) {
                if idx > end {
                    break;
                }
                split.insert(idx);
                for &p in pins {
                    let pt = if horizontal { (idx, track, layer) } else { (track, idx, layer) };
                    pin_gcell_map.entry(p).or_default().insert(pt);
                }
            }
        }
    };
    let split_by_layer = |layer: i64, track: i32, begin: i32, end: i32, split: &mut BTreeSet<i32>| {
        if layer < 0 || layer as usize >= intvs.len() {
            return;
        }
        for (&t, set) in intvs[layer as usize].range(begin..) {
            if t > end {
                break;
            }
            if set.contains(track) {
                split.insert(t);
            }
        }
    };
    for (layer, tracks) in intvs.iter().enumerate() {
        let horizontal = net.horizontal(layer);
        for (&track, set) in tracks {
            for (begin, end) in set.iter() {
                let mut split = BTreeSet::new();
                let via_only = layer < net.cfg.bottom_routing_layer || (via_access_only && layer <= net.cfg.via_access_layer);
                if via_only {
                    split_by_pins(layer, horizontal, track, begin, end, pin_gcell_map, &mut split);
                    for i in begin..=end {
                        split_rect(track, i, i, layer, horizontal, &mut rects);
                    }
                } else {
                    split_by_layer(layer as i64 - 2, track, begin, end, &mut split);
                    split_by_layer(layer as i64 + 2, track, begin, end, &mut split);
                    split_by_pins(layer, horizontal, track, begin, end, pin_gcell_map, &mut split);
                    let v: Vec<i32> = split.iter().copied().collect();
                    split_rect(track, v[0], v[0], layer, horizontal, &mut rects);
                    for w in v.windows(2) {
                        split_rect(track, w[1], w[1], layer, horizontal, &mut rects);
                        split_rect(track, w[0], w[1], layer, horizontal, &mut rects);
                    }
                }
            }
        }
    }
    rects
}

// ---- the search ----

/// A search state: its cost, node and predecessor — popped cheapest first, then lowest node, then
/// lowest predecessor (a total order).
type Wave = Reverse<(i32, i64, i64)>;

pub struct PathFinder<'a, 'n> {
    net: &'a GuideNet<'n>,
    force_feed_through: bool,
    /// The search's pins (the net's pin order) that have gcells.
    pins: Vec<usize>,
    rects: Vec<(usize, Rect)>,
    guide_count: usize,
    node_count: usize,
    node_map: BTreeMap<P3, BTreeSet<i64>>,
    adj: Vec<Vec<i64>>,
    is_on_path: Vec<bool>,
    visited: Vec<bool>,
    prev: Vec<i64>,
}

impl<'a, 'n> PathFinder<'a, 'n> {
    pub fn new(net: &'a GuideNet<'n>, force_feed_through: bool, rects: &[(usize, Rect)], pin_gcell_map: &BTreeMap<usize, BTreeSet<P3>>) -> PathFinder<'a, 'n> {
        let search_pins: Vec<usize> = (0..net.pins.len()).filter(|p| pin_gcell_map.contains_key(p)).collect();
        let mut f = PathFinder {
            net,
            force_feed_through,
            pins: search_pins,
            rects: rects.to_vec(),
            guide_count: rects.len(),
            node_count: 0,
            node_map: BTreeMap::new(),
            adj: Vec::new(),
            is_on_path: Vec::new(),
            visited: Vec::new(),
            prev: Vec::new(),
        };
        f.build_node_map(pin_gcell_map);
        f.construct_adj_list();
        f.is_on_path = vec![false; f.node_count];
        f.visited = vec![false; f.node_count];
        f.prev = vec![-1; f.node_count];
        f
    }

    fn is_pin(&self, i: i64) -> bool {
        i >= self.guide_count as i64
    }
    fn pin_count(&self) -> usize {
        self.node_count - self.guide_count
    }

    fn build_node_map(&mut self, pin_gcell_map: &BTreeMap<usize, BTreeSet<P3>>) {
        self.node_map.clear();
        for (i, &(l, r)) in self.rects.iter().enumerate() {
            self.node_map.entry((r.xl, r.yl, l)).or_default().insert(i as i64);
            self.node_map.entry((r.xh, r.yh, l)).or_default().insert(i as i64);
        }
        let mut node = self.rects.len() as i64;
        for &p in &self.pins {
            for &g in &pin_gcell_map[&p] {
                self.node_map.entry(g).or_default().insert(node);
            }
            node += 1;
        }
        self.node_count = node as usize;
    }

    fn construct_adj_list(&mut self) {
        self.adj = vec![Vec::new(); self.node_count];
        let va = self.net.cfg.via_access_layer;
        let entries: Vec<(P3, Vec<i64>)> = self.node_map.iter().map(|(&k, v)| (k, v.iter().copied().collect())).collect();
        for (pt, idx) in &entries {
            let layer = pt.2;
            for (k, &i1) in idx.iter().enumerate() {
                for &i2 in &idx[k + 1..] {
                    if self.is_pin(i1) && self.is_pin(i2) {
                        continue;
                    }
                    if !self.is_pin(i1) && !self.is_pin(i2) {
                        if layer > va {
                            self.adj[i1 as usize].push(i2);
                            self.adj[i2 as usize].push(i1);
                        }
                    } else {
                        let (g, p) = (i1.min(i2), i1.max(i2));
                        if self.net.cfg.allow_pin_feedthrough || self.force_feed_through {
                            self.adj[p as usize].push(g);
                            self.adj[g as usize].push(p);
                        } else if p == self.guide_count as i64 {
                            self.adj[p as usize].push(g);
                        } else {
                            self.adj[g as usize].push(p);
                        }
                    }
                }
                if !self.is_pin(i1) {
                    if let Some(up) = self.node_map.get(&(pt.0, pt.1, layer + 2)) {
                        for &n in up {
                            if !self.is_pin(n) {
                                self.adj[i1 as usize].push(n);
                                self.adj[n as usize].push(i1);
                            }
                        }
                    }
                }
            }
        }
    }

    fn init_search_queue(&self) -> BinaryHeap<Wave> {
        let mut q = BinaryHeap::new();
        let first = self.guide_count;
        if !self.visited[first] {
            q.push(Reverse((0, first as i64, -1)));
        } else {
            for i in 0..self.node_count {
                if self.is_on_path[i] {
                    let cost = if self.net.cfg.allow_pin_feedthrough && self.is_pin(i as i64) {
                        2
                    } else if self.force_feed_through && self.is_pin(i as i64) {
                        10
                    } else {
                        0
                    };
                    q.push(Reverse((cost, i as i64, self.prev[i])));
                }
            }
        }
        q
    }

    /// A tree from the first pin to every other: one shortest path at a time from the tree so far.
    pub fn traverse_graph(&mut self) -> bool {
        let pins = self.pin_count();
        for _ in 0..pins.saturating_sub(1) {
            let mut q = self.init_search_queue();
            let mut visited_pin: i64 = -1;
            while let Some(Reverse((cost, node, prev))) = q.pop() {
                let n = node as usize;
                if !self.is_on_path[n] && self.visited[n] {
                    continue;
                }
                if n > self.guide_count && !self.visited[n] {
                    self.visited[n] = true;
                    self.prev[n] = prev;
                    visited_pin = node;
                    break;
                }
                self.visited[n] = true;
                self.prev[n] = prev;
                for k in 0..self.adj[n].len() {
                    let nb = self.adj[n][k];
                    if !self.visited[nb as usize] {
                        let mut c = 1;
                        if !self.is_pin(nb) {
                            let r = self.rects[nb as usize].1;
                            c += r.dx() + r.dy();
                        }
                        q.push(Reverse((cost + c, nb, node)));
                    }
                }
            }
            let mut last = visited_pin;
            while last != -1 && !self.is_on_path[last as usize] {
                self.is_on_path[last as usize] = true;
                last = self.prev[last as usize];
            }
            self.visited = self.is_on_path.clone();
        }
        if pins == 1 {
            return true;
        }
        self.visited[self.guide_count..].iter().filter(|&&v| v).count() == pins
    }

    fn pin_to_gcell_list(&self, pin_gcell_map: &BTreeMap<usize, BTreeSet<P3>>) -> Vec<Vec<P3>> {
        let mut out = vec![Vec::new(); self.pin_count()];
        for i in 0..self.node_count {
            if !self.visited[i] {
                continue;
            }
            let (ii, pv) = (i as i64, self.prev[i]);
            let (pin, guide) = if self.is_pin(ii) && pv >= 0 && !self.is_pin(pv) {
                (ii, pv)
            } else if !self.is_pin(ii) && pv >= 0 && self.is_pin(pv) {
                (pv, ii)
            } else {
                continue;
            };
            let tp = pin as usize - self.guide_count;
            let (l, r) = self.rects[guide as usize];
            let g = &pin_gcell_map[&self.pins[tp]];
            if g.contains(&(r.xl, r.yl, l)) {
                out[tp].push((r.xl, r.yl, l));
            } else if g.contains(&(r.xh, r.yh, l)) {
                out[tp].push((r.xh, r.yh, l));
            }
        }
        out
    }

    fn update_node_map(&mut self, pin_to_gcell: &[Vec<P3>]) {
        self.node_map.clear();
        for (i, pts) in pin_to_gcell.iter().enumerate() {
            for &pt in pts {
                self.node_map.entry(pt).or_default().insert((i + self.guide_count) as i64);
            }
        }
        for i in 0..self.guide_count {
            if !self.visited[i] {
                continue;
            }
            let (l, r) = self.rects[i];
            self.node_map.entry((r.xl, r.yl, l)).or_default().insert(i as i64);
            self.node_map.entry((r.xh, r.yh, l)).or_default().insert(i as i64);
        }
    }

    /// The keys of the node map in order — including keys added while walking it.
    fn next_key(&self, after: Option<P3>) -> Option<P3> {
        match after {
            None => self.node_map.keys().next().copied(),
            Some(k) => self.node_map.range((std::ops::Bound::Excluded(k), std::ops::Bound::Unbounded)).next().map(|(&k, _)| k),
        }
    }

    fn clip_guides(&mut self) {
        let mut cur = self.next_key(None);
        while let Some(pt) = cur {
            let idx: Vec<i64> = self.node_map[&pt].iter().copied().collect();
            if idx.len() == 1 && !self.is_pin(idx[0]) {
                let i = idx[0];
                let up = self.node_map.contains_key(&(pt.0, pt.1, pt.2 + 2));
                let down = pt.2 >= 2 && self.node_map.contains_key(&(pt.0, pt.1, pt.2 - 2));
                if !up && !down {
                    let r = self.rects[i as usize].1;
                    if (r.xl, r.yl) != (r.xh, r.yh) {
                        self.rects[i as usize].1 = if (r.xl, r.yl) == (pt.0, pt.1) { Rect { xl: r.xh, yl: r.yh, xh: r.xh, yh: r.yh } } else { Rect { xl: r.xl, yl: r.yl, xh: r.xl, yh: r.yl } };
                        self.node_map.get_mut(&pt).expect("key").remove(&i);
                    }
                }
            }
            cur = self.next_key(Some(pt));
        }
    }

    fn visited_indices(&self, pt: &P3) -> Vec<i64> {
        self.node_map.get(pt).map_or(Vec::new(), |s| s.iter().copied().filter(|&i| self.visited[i as usize]).collect())
    }

    fn merge_guides(&mut self) {
        let mut cur = self.next_key(None);
        while let Some(pt) = cur {
            let v = self.visited_indices(&pt);
            if v.len() == 2 {
                let (a, b) = (v[0], v[1]);
                let up = !self.visited_indices(&(pt.0, pt.1, pt.2 + 2)).is_empty();
                let down = pt.2 >= 2 && !self.visited_indices(&(pt.0, pt.1, pt.2 - 2)).is_empty();
                if !self.is_pin(a) && !self.is_pin(b) && !up && !down {
                    let (r1, r2) = (self.rects[a as usize].1, self.rects[b as usize].1);
                    let dir = |r: &Rect| r.dx() >= r.dy();
                    if dir(&r1) == dir(&r2) {
                        self.rects[b as usize].1 = Rect { xl: r1.xl.min(r2.xl), yl: r1.yl.min(r2.yl), xh: r1.xh.max(r2.xh), yh: r1.yh.max(r2.yh) };
                        self.node_map.get_mut(&pt).expect("key").clear();
                        let other = if (r1.xl, r1.yl) == (pt.0, pt.1) { (r1.xh, r1.yh, pt.2) } else { (r1.xl, r1.yl, pt.2) };
                        let set = self.node_map.entry(other).or_default();
                        set.remove(&a);
                        set.insert(b);
                        self.visited[a as usize] = false;
                    }
                }
            }
            cur = self.next_key(Some(pt));
        }
    }

    /// The tree's pieces as guides (gcell centres) and the gr pins.
    pub fn commit_path_to_guides(&mut self, pin_gcell_map: &BTreeMap<usize, BTreeSet<P3>>) -> (Vec<Guide>, GrPins) {
        let p2g = self.pin_to_gcell_list(pin_gcell_map);
        self.update_node_map(&p2g);
        let mut gr = Vec::new();
        for (i, pts) in p2g.iter().enumerate() {
            for &pt in pts {
                gr.push((self.pins[i], self.net.grid.center((pt.0, pt.1))));
            }
        }
        self.clip_guides();
        self.merge_guides();
        let mut fin: Vec<(usize, (i32, i32, i32, i32))> = (0..self.guide_count).filter(|&i| self.visited[i]).map(|i| (self.rects[i].0, { let r = self.rects[i].1; (r.xl, r.yl, r.xh, r.yh) })).collect();
        fin.sort();
        fin.dedup();
        let guides = fin.iter().map(|&(l, (xl, yl, xh, yh))| Guide { begin: self.net.grid.center((xl, yl)), end: self.net.grid.center((xh, yh)), layer: l }).collect();
        (guides, gr)
    }

    fn bfs(&mut self, start: usize) {
        self.visited = vec![false; self.node_count];
        let mut q = VecDeque::from([start as i64]);
        while let Some(n) = q.pop_front() {
            self.visited[n as usize] = true;
            for &nb in &self.adj[n as usize] {
                if !self.visited[nb as usize] {
                    q.push_back(nb);
                }
            }
        }
    }

    /// The shortest bridge (on a track next to one) between the first pin's part and the first
    /// unreached pin's part, added to the intervals.
    pub fn connect_disconnected_components(&mut self, intvs: &mut TrackIntervals) {
        let unvisited = (self.guide_count + 1..self.node_count).find(|&i| !self.visited[i]).unwrap_or(self.node_count);
        let visited_guides = |f: &PathFinder<'_, '_>| (0..f.guide_count).filter(|&i| f.visited[i]).collect::<Vec<_>>();
        self.bfs(self.guide_count);
        let c1 = visited_guides(self);
        if unvisited < self.node_count {
            self.bfs(unvisited);
        } else {
            self.visited = vec![false; self.node_count];
        }
        let c2 = visited_guides(self);
        let mut best = BridgeGuide::none();
        for &i in &c1 {
            for &j in &c2 {
                let ((l1, r1), (l2, r2)) = (self.rects[i], self.rects[j]);
                if l1 != l2 {
                    continue;
                }
                let h = self.net.horizontal(l1);
                let (t1, t2) = if h { (r1.yl, r2.yl) } else { (r1.xl, r2.xl) };
                if (t1 - t2).abs() > 1 {
                    continue;
                }
                let (b1, e1) = if h { (r1.xl, r1.xh) } else { (r1.yl, r1.yh) };
                let (b2, e2) = if h { (r2.xl, r2.xh) } else { (r2.yl, r2.yh) };
                for cand in [BridgeGuide::new(t1, e1, b2, l1 as i64), BridgeGuide::new(t2, e2, b1, l1 as i64)] {
                    if cand.less(&best) {
                        best = cand;
                    }
                }
            }
        }
        if best.layer == -1 {
            return;
        }
        intvs[best.layer as usize].entry(best.track).or_default().insert(best.begin, best.end);
        add_touching_guides_bridges(intvs);
    }
}

/// A net's guides and gr pins from its global-route guides (design units); `None` when no round
/// connects its pins.
pub fn gen_guides(net: &GuideNet<'_>, input: &[(usize, Rect)], trace: &mut Option<Vec<Event>>) -> Option<(Vec<Guide>, GrPins)> {
    let mut rects = input.to_vec();
    cover_pins(net, &mut rects);
    if let Some(t) = trace {
        t.push(Event::Covered(rects.clone()));
    }
    let size = net.tech.layers.len().min(net.cfg.top_routing_layer + 1);
    let mut intvs: TrackIntervals = vec![BTreeMap::new(); size];
    gen_guides_prep(net, &rects, &mut intvs);
    let gcell_pin_map = init_gcell_pin_map(net);
    let mut pin_gcell_map: BTreeMap<usize, BTreeSet<P3>> = (0..net.pins.len()).map(|p| (p, BTreeSet::new())).collect();
    for i in 0..4 {
        let force = i >= 2;
        let via_access_only = i == 0;
        let patch_on_failure = i == 2;
        if i != 2 {
            rects = gen_guides_split(net, &intvs, &gcell_pin_map, &mut pin_gcell_map, via_access_only);
            if let Some(t) = trace {
                t.push(Event::Round { i, rects: rects.clone() });
            }
        }
        let mut f = PathFinder::new(net, force, &rects, &pin_gcell_map);
        let ok = f.traverse_graph();
        if let Some(t) = trace {
            t.push(Event::Traverse(ok));
        }
        if ok {
            let (guides, gr) = f.commit_path_to_guides(&pin_gcell_map);
            if let Some(t) = trace {
                t.push(Event::Guides(guides.clone()));
                t.push(Event::GrPins(gr.clone()));
            }
            return Some((guides, gr));
        }
        if patch_on_failure {
            if let Some(t) = trace {
                t.push(Event::Bridge);
            }
            f.connect_disconnected_components(&mut intvs);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid() -> GCellGrid {
        GCellGrid { x: (0, 4, 100), y: (0, 3, 100), die: Rect { xl: -10, yl: -10, xh: 410, yh: 305 } }
    }

    // Closed integer intervals: [1,3] and [4,6] TOUCH and join; [1,3] and [5,6] do not.
    #[test]
    fn touching_intervals_join() {
        let mut s = IntervalSet::default();
        s.insert(1, 3);
        s.insert(5, 6);
        assert_eq!(s.iter().collect::<Vec<_>>(), vec![(1, 3), (5, 6)]);
        s.insert(4, 4);
        assert_eq!(s.iter().collect::<Vec<_>>(), vec![(1, 6)]);
        assert!(s.contains(6) && !s.contains(7) && !s.contains(0));
    }

    #[test]
    fn intersection_keeps_the_overlap() {
        let (mut a, mut b) = (IntervalSet::default(), IntervalSet::default());
        a.insert(0, 5);
        a.insert(8, 9);
        b.insert(3, 8);
        assert_eq!(a.intersect(&b).iter().collect::<Vec<_>>(), vec![(3, 5), (8, 8)]);
    }

    // A point clamps into the grid; on a gcell line it touches both sides, but not below gcell 0.
    #[test]
    fn a_point_on_a_gcell_line_touches_both_sides() {
        let g = grid();
        assert_eq!(g.idx((-50, 999)), (0, 2));
        assert_eq!(g.indices((100, 150)), vec![(0, 1), (1, 1)]);
        assert_eq!(g.indices((0, 0)), vec![(0, 0)]);
        assert_eq!(g.indices((999, 50)), vec![(3, 0)]);
    }

    // The edge gcells reach the die, so their centres move.
    #[test]
    fn edge_gcells_reach_the_die() {
        let g = grid();
        assert_eq!(g.center((0, 0)), (45, 45));
        assert_eq!(g.center((1, 1)), (150, 150));
        assert_eq!(g.center((3, 2)), (355, 252));
        assert_eq!(g.gcell_box((9, 9)), Rect { xl: 300, yl: 200, xh: 410, yh: 305 });
    }

    // Shorter bridges first; on equal length the HIGHER layer; no bridge is longest of all.
    #[test]
    fn bridges_order_by_length_then_higher_layer() {
        let (a, b, c) = (BridgeGuide::new(0, 5, 3, 2), BridgeGuide::new(0, 1, 3, 4), BridgeGuide::new(0, 1, 4, 2));
        assert!(b.less(&a) && !a.less(&b));
        assert!(a.less(&c));
        assert!(c.less(&BridgeGuide::none()));
    }

    // Corner-touching intervals on consecutive tracks get a one-gcell overlap: here the previous
    // track's interval ends where the current one's begins minus one, so the PREVIOUS grows.
    #[test]
    fn corner_touching_intervals_overlap_by_one_gcell() {
        let mut intvs: TrackIntervals = vec![BTreeMap::new(); 3];
        intvs[2].entry(5).or_default().insert(0, 3);
        intvs[2].entry(6).or_default().insert(4, 8);
        add_touching_guides_bridges(&mut intvs);
        assert_eq!(intvs[2][&5].iter().collect::<Vec<_>>(), vec![(0, 4)]);
        assert_eq!(intvs[2][&6].iter().collect::<Vec<_>>(), vec![(4, 8)]);
        // Now side by side at gcell 4 with nothing below or above: a bridge on the layer below,
        // along that gcell, across both tracks.
        assert_eq!(intvs[0][&4].iter().collect::<Vec<_>>(), vec![(5, 6)]);
    }

    /// Layers 0–1 placeholders, 2 horizontal routing, 3 cut, 4 vertical routing.
    fn tech2() -> Tech {
        use crate::tech::{Dir, Layer, LayerKind};
        let mut t = Tech::default();
        for (kind, dir) in [(LayerKind::Placeholder, Dir::None), (LayerKind::Placeholder, Dir::None), (LayerKind::Routing, Dir::Horizontal), (LayerKind::Cut, Dir::None), (LayerKind::Routing, Dir::Vertical)] {
            t.layers.push(Layer { kind, dir, ..Default::default() });
        }
        t
    }

    fn pin(name: &str, id: usize, ap: (i32, i32), layer: usize) -> GuidePin {
        GuidePin { name: name.into(), is_port: false, id, shapes: Vec::new(), aps: vec![(ap, layer)] }
    }

    fn r(xl: i32, yl: i32, xh: i32, yh: i32) -> Rect {
        Rect { xl, yl, xh, yh }
    }

    // Two pins on the via-access layer (2), joined along it by one guide three gcells long and
    // crossed at the middle gcell by a guide on layer 4. Round 0 cuts the via-access layer into
    // single gcells, and pieces there never link along the layer — so it FAILS; round 1 splits at
    // the crossing and the pins, and the path still may not run piece-to-piece along layer 2: it
    // climbs to the crossing on layer 4 and back.
    #[test]
    fn the_via_access_layer_is_single_gcells_first_and_never_linked_along() {
        let tech = tech2();
        let grid = GCellGrid { x: (0, 10, 100), y: (0, 10, 100), die: r(0, 0, 1000, 1000) };
        let cfg = GuideConfig { bottom_routing_layer: 2, top_routing_layer: 4, via_access_layer: 2, allow_pin_feedthrough: true };
        let net = GuideNet { tech: &tech, grid: &grid, cfg: &cfg, pins: vec![pin("a", 0, (50, 50), 2), pin("b", 1, (250, 50), 2)] };
        let mut trace = Some(Vec::new());
        let (guides, _) = gen_guides(&net, &[(2, r(0, 0, 300, 100)), (4, r(100, 0, 200, 100))], &mut trace).expect("connected");
        let verdicts: Vec<(usize, bool)> = {
            let mut round = 0;
            trace.unwrap().iter().filter_map(|e| match e {
                Event::Round { i, .. } => {
                    round = *i;
                    None
                }
                Event::Traverse(ok) => Some((round, *ok)),
                _ => None,
            }).collect()
        };
        assert_eq!(verdicts, vec![(0, false), (1, true)]);
        assert!(guides.iter().any(|g| g.layer == 4 && g.begin == (150, 50) && g.end == (150, 50)), "{guides:?}");
    }

    // A pin beyond a guide three gcells wide on a vertical layer, level with its middle: the
    // point nearest the pin is equally far from both sides, and the tie goes to the LOW side.
    #[test]
    fn a_patch_ties_to_the_low_side() {
        let tech = tech2();
        let grid = GCellGrid { x: (0, 10, 100), y: (0, 10, 100), die: r(0, 0, 1000, 1000) };
        let cfg = GuideConfig { bottom_routing_layer: 2, top_routing_layer: 4, via_access_layer: 2, allow_pin_feedthrough: true };
        let net = GuideNet { tech: &tech, grid: &grid, cfg: &cfg, pins: vec![pin("a", 0, (150, 850), 4)] };
        let mut guides = vec![(4, r(0, 0, 300, 300))];
        cover_pins(&net, &mut guides);
        assert_eq!(guides, vec![(4, r(0, 0, 300, 900)), (2, r(0, 800, 200, 900)), (4, r(100, 800, 200, 900))]);
    }

}
