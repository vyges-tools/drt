// SPDX-License-Identifier: Apache-2.0
//! Design-rule checks over one small window: whether trial route shapes may stand next to the
//! shapes already there.
//!
//! Stages, in order: shapes are added per OWNER (a net, or the terminal or instance that stands in
//! for one), each FIXED (already in the design) or not (the trial); [`Worker::init`] merges each
//! owner's shapes per layer and derives its maximal rectangles and boundary edges; [`Worker::run`]
//! checks metal spacing (shorts, non-sufficient metal, the parallel-run spacing table), then cut
//! spacing (cut shorts, cut spacing). The verdict is whether any marker was made.
//!
//! Rules:
//! - two fixed shapes are never checked against each other: a violation needs a trial shape;
//! - routing-layer shapes merge per owner; a maximal rectangle of the merge is FIXED when it is
//!   also a maximal rectangle of the owner's fixed shapes alone;
//! - cut shapes do not merge: each rectangle stands alone, fixed when it equals a fixed rectangle;
//! - boundary edges run with the shape's inside on their LEFT (outer boundaries counter-clockwise);
//! - an owner that is an instance (its obstructions) is a BLOCKAGE: its width in a spacing lookup
//!   is the layer's width, and a short with it is judged only where the other owner's fixed
//!   shapes do not already cover the overlap;
//! - a marker is kept once per (box, layer, rule, the owners involved).
//!
//! Not modelled, because the technologies checked here do not have them (a caller must not rely on
//! these being checked): minimum width, minimum step, minimum enclosed area, end-of-line spacing,
//! corner spacing, spacing ranges and same-net spacing, adjacent-cut and parallel-overlap cut
//! spacing, non-default rules.

use std::collections::{BTreeSet, HashMap};

use crate::polygon90::{Polygon90Set, Rect};
use crate::tech::{LayerKind, Tech};

/// Who a shape belongs to. Two shapes are the same net exactly when their owners are equal.
#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Owner {
    Net(String),
    /// An unconnected instance terminal: `(instance, terminal)`.
    InstTerm(String, String),
    /// An unconnected block terminal.
    BlockTerm(String),
    /// An instance: its obstructions.
    Inst(String),
    /// Every unconnected ground terminal's design shapes.
    FloatingGround,
    /// Every unconnected power terminal's design shapes.
    FloatingPower,
}

impl Owner {
    pub fn is_blockage(&self) -> bool {
        matches!(self, Owner::Inst(_))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Rule {
    /// Two owners' shapes overlap or touch (routing or cut layer).
    Short,
    /// One owner's shapes meet through less than the minimum width.
    NonSufficientMetal,
    /// Closer than the routing layer's spacing table allows.
    MetalSpacing,
    /// Closer than the cut layer's spacing.
    CutSpacing,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct Marker {
    pub rule: Rule,
    pub layer: usize,
    pub bbox: Rect,
    /// The owners involved, sorted, without repeats.
    pub owners: Vec<Owner>,
}

/// A maximal rectangle of one owner on one layer.
#[derive(Debug, Clone, Copy)]
struct Shape {
    rect: Rect,
    net: usize,
    fixed: bool,
}

/// A boundary edge, from `from` to `to`.
#[derive(Debug, Clone, Copy)]
struct Edge {
    from: (i32, i32),
    to: (i32, i32),
    net: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EdgeDir {
    N,
    S,
    E,
    W,
}

impl Edge {
    fn dir(&self) -> EdgeDir {
        if self.from.0 == self.to.0 {
            if self.from.1 < self.to.1 {
                EdgeDir::N
            } else {
                EdgeDir::S
            }
        } else if self.from.0 < self.to.0 {
            EdgeDir::E
        } else {
            EdgeDir::W
        }
    }
}

#[derive(Debug, Clone, Default)]
struct Net {
    owner: Option<Owner>,
    fixed: Vec<Polygon90Set>,
    route: Vec<Polygon90Set>,
    fixed_cuts: Vec<Vec<Rect>>,
    route_cuts: Vec<Vec<Rect>>,
    /// After `init`: the fixed and the route shapes as disjoint slices, and the fixed shapes'
    /// maximal rectangles.
    fixed_slices: Vec<Vec<Rect>>,
    route_slices: Vec<Vec<Rect>>,
    fixed_max: Vec<Vec<Rect>>,
}

pub struct Worker<'a> {
    tech: &'a Tech,
    nets: Vec<Net>,
    index: HashMap<Owner, usize>,
    shapes: Vec<Vec<Shape>>,
    edges: Vec<Vec<Edge>>,
    markers: Vec<Marker>,
    seen: BTreeSet<(Rect, usize, Rule, Vec<Owner>)>,
}

fn area(r: &Rect) -> i64 {
    i64::from(r.dx()) * i64::from(r.dy())
}

/// The overlap of two rectangles when it has an area.
fn overlap(a: &Rect, b: &Rect) -> Option<Rect> {
    let r = Rect { xl: a.xl.max(b.xl), yl: a.yl.max(b.yl), xh: a.xh.min(b.xh), yh: a.yh.min(b.yh) };
    (r.xl < r.xh && r.yl < r.yh).then_some(r)
}

/// The closed intersection of two rectangles, touching included.
fn meet(a: &Rect, b: &Rect) -> Option<Rect> {
    let r = Rect { xl: a.xl.max(b.xl), yl: a.yl.max(b.yl), xh: a.xh.min(b.xh), yh: a.yh.min(b.yh) };
    (r.xl <= r.xh && r.yl <= r.yh).then_some(r)
}

fn touches(a: &Rect, b: &Rect) -> bool {
    meet(a, b).is_some()
}

fn contains(outer: &Rect, inner: &Rect) -> bool {
    outer.xl <= inner.xl && inner.xh <= outer.xh && outer.yl <= inner.yl && inner.yh <= outer.yh
}

fn bloat(r: &Rect, v: i32) -> Rect {
    Rect { xl: r.xl - v, yl: r.yl - v, xh: r.xh + v, yh: r.yh + v }
}

/// A rectangle's width: its smaller side.
fn width(r: &Rect) -> i32 {
    r.dx().min(r.dy())
}

/// The gap between two intervals, 0 when they overlap or touch.
fn gap(a: (i32, i32), b: (i32, i32)) -> i32 {
    (b.0 - a.1).max(a.0 - b.1).max(0)
}

/// Per axis the overlap of the two rectangles, or, where they are apart, the gap between them.
fn generalized_intersect(a: &Rect, b: &Rect) -> Rect {
    let axis = |al: i32, ah: i32, bl: i32, bh: i32| {
        let (lo, hi) = (al.max(bl), ah.min(bh));
        (lo.min(hi), lo.max(hi))
    };
    let (xl, xh) = axis(a.xl, a.xh, b.xl, b.xh);
    let (yl, yh) = axis(a.yl, a.yh, b.yl, b.yh);
    Rect { xl, yl, xh, yh }
}

/// Area of a region (disjoint slices) inside `r`.
fn area_in(slices: &[Rect], r: &Rect) -> i64 {
    slices.iter().filter_map(|s| overlap(s, r)).map(|o| area(&o)).sum()
}

/// Closed intervals merged where they overlap or touch.
fn union(mut v: Vec<(i32, i32)>) -> Vec<(i32, i32)> {
    v.sort();
    let mut out: Vec<(i32, i32)> = Vec::new();
    for (a, b) in v {
        match out.last_mut() {
            Some(l) if a <= l.1 => l.1 = l.1.max(b),
            _ => out.push((a, b)),
        }
    }
    out
}

/// The parts of `a` not in `b`, with a length.
fn minus(a: &[(i32, i32)], b: &[(i32, i32)]) -> Vec<(i32, i32)> {
    let mut out = Vec::new();
    for &(lo, hi) in a {
        let mut cur = lo;
        for &(bl, bh) in b {
            if bh <= cur || bl >= hi {
                continue;
            }
            if bl > cur {
                out.push((cur, bl));
            }
            cur = cur.max(bh);
        }
        if cur < hi {
            out.push((cur, hi));
        }
    }
    out
}

/// A region's boundary edges (maximal), inside on the left: bottom edges run east, top edges
/// west, left edges south, right edges north.
fn boundary(slices: &[Rect]) -> Vec<((i32, i32), (i32, i32))> {
    let mut out = Vec::new();
    let ys: BTreeSet<i32> = slices.iter().flat_map(|s| [s.yl, s.yh]).collect();
    for &y in &ys {
        let above = union(slices.iter().filter(|s| s.yl <= y && y < s.yh).map(|s| (s.xl, s.xh)).collect());
        let below = union(slices.iter().filter(|s| s.yl < y && y <= s.yh).map(|s| (s.xl, s.xh)).collect());
        for (a, b) in minus(&above, &below) {
            out.push(((a, y), (b, y)));
        }
        for (a, b) in minus(&below, &above) {
            out.push(((b, y), (a, y)));
        }
    }
    let xs: BTreeSet<i32> = slices.iter().flat_map(|s| [s.xl, s.xh]).collect();
    for &x in &xs {
        let right = union(slices.iter().filter(|s| s.xl <= x && x < s.xh).map(|s| (s.yl, s.yh)).collect());
        let left = union(slices.iter().filter(|s| s.xl < x && x <= s.xh).map(|s| (s.yl, s.yh)).collect());
        for (a, b) in minus(&right, &left) {
            out.push(((x, b), (x, a)));
        }
        for (a, b) in minus(&left, &right) {
            out.push(((x, a), (x, b)));
        }
    }
    out
}

/// The maximal rectangles of `r` minus the union of `holes`.
fn max_rects_of_difference(r: &Rect, holes: &[Rect]) -> Vec<Rect> {
    let mut xs: Vec<i32> = vec![r.xl, r.xh];
    let mut ys: Vec<i32> = vec![r.yl, r.yh];
    for h in holes {
        xs.extend([h.xl, h.xh].into_iter().filter(|&v| v > r.xl && v < r.xh));
        ys.extend([h.yl, h.yh].into_iter().filter(|&v| v > r.yl && v < r.yh));
    }
    xs.sort();
    xs.dedup();
    ys.sort();
    ys.dedup();
    let mut set = Polygon90Set::new();
    for i in 0..xs.len() - 1 {
        for j in 0..ys.len() - 1 {
            let cell = Rect { xl: xs[i], xh: xs[i + 1], yl: ys[j], yh: ys[j + 1] };
            if !holes.iter().any(|h| contains(h, &cell)) {
                set.insert_rect(cell);
            }
        }
    }
    set.max_rectangles()
}

impl<'a> Worker<'a> {
    /// A worker with the floating ground and power owners in place.
    pub fn new(tech: &'a Tech) -> Worker<'a> {
        let mut w = Worker { tech, nets: Vec::new(), index: HashMap::new(), shapes: Vec::new(), edges: Vec::new(), markers: Vec::new(), seen: BTreeSet::new() };
        w.net(&Owner::FloatingGround);
        w.net(&Owner::FloatingPower);
        w
    }

    fn net(&mut self, owner: &Owner) -> usize {
        if let Some(&i) = self.index.get(owner) {
            return i;
        }
        let n = self.tech.layers.len();
        self.nets.push(Net {
            owner: Some(owner.clone()),
            fixed: vec![Polygon90Set::new(); n],
            route: vec![Polygon90Set::new(); n],
            fixed_cuts: vec![Vec::new(); n],
            route_cuts: vec![Vec::new(); n],
            ..Net::default()
        });
        self.index.insert(owner.clone(), self.nets.len() - 1);
        self.nets.len() - 1
    }

    /// A shape of `owner`: a cut rectangle on a cut layer, else part of the owner's merged shapes.
    pub fn add(&mut self, owner: &Owner, layer: usize, r: Rect, fixed: bool) {
        let i = self.net(owner);
        let net = &mut self.nets[i];
        match (self.tech.layers[layer].kind == LayerKind::Cut, fixed) {
            (true, true) => net.fixed_cuts[layer].push(r),
            (true, false) => net.route_cuts[layer].push(r),
            (false, true) => net.fixed[layer].insert_rect(r),
            (false, false) => net.route[layer].insert_rect(r),
        }
    }

    /// Per owner and layer: the merged shapes' maximal rectangles (fixed or not) and boundary
    /// edges; cut rectangles as they are.
    pub fn init(&mut self) {
        let n = self.tech.layers.len();
        self.shapes = vec![Vec::new(); n];
        self.edges = vec![Vec::new(); n];
        for (i, net) in self.nets.iter_mut().enumerate() {
            net.fixed_slices = net.fixed.iter_mut().map(|s| s.rectangles()).collect();
            net.route_slices = net.route.iter_mut().map(|s| s.rectangles()).collect();
            net.fixed_max = net.fixed.iter_mut().map(|s| s.max_rectangles()).collect();
            for layer in 0..n {
                let mut all = Polygon90Set::new();
                for s in net.fixed_slices[layer].iter().chain(&net.route_slices[layer]) {
                    all.insert_rect(*s);
                }
                let slices = all.rectangles();
                for (from, to) in boundary(&slices) {
                    self.edges[layer].push(Edge { from, to, net: i });
                }
                for r in all.max_rectangles() {
                    let fixed = net.fixed_max[layer].contains(&r);
                    self.shapes[layer].push(Shape { rect: r, net: i, fixed });
                }
                for &r in net.route_cuts[layer].iter().chain(&net.fixed_cuts[layer]) {
                    let fixed = net.fixed_cuts[layer].contains(&r);
                    self.shapes[layer].push(Shape { rect: r, net: i, fixed });
                }
            }
        }
    }

    /// Metal spacing, then cut spacing, over every owner's shapes; the markers made.
    pub fn run(&mut self) -> &[Marker] {
        self.check_metal_spacing();
        self.check_cut_spacing();
        &self.markers
    }

    fn owner(&self, net: usize) -> &Owner {
        self.nets[net].owner.as_ref().expect("an owner")
    }

    fn add_marker(&mut self, rule: Rule, layer: usize, bbox: Rect, a: usize, b: usize) {
        let mut owners = vec![self.owner(a).clone(), self.owner(b).clone()];
        owners.sort();
        owners.dedup();
        if self.seen.insert((bbox, layer, rule, owners.clone())) {
            self.markers.push(Marker { rule, layer, bbox, owners });
        }
    }

    /// Every shape on `layer` touching `r`.
    fn query(&self, layer: usize, r: &Rect) -> Vec<usize> {
        (0..self.shapes[layer].len()).filter(|&k| touches(&self.shapes[layer][k].rect, r)).collect()
    }

    // ---- metal spacing ----

    /// Per routing layer, per owner, per maximal rectangle: every shape within the layer's largest
    /// spacing.
    fn check_metal_spacing(&mut self) {
        for layer in 0..self.tech.layers.len() {
            if self.tech.layers[layer].kind != LayerKind::Routing {
                continue;
            }
            for net in 0..self.nets.len() {
                let mine: Vec<usize> = (0..self.shapes[layer].len()).filter(|&k| self.shapes[layer][k].net == net).collect();
                for k in mine {
                    self.metal_spacing_of(layer, k);
                }
            }
        }
    }

    fn metal_spacing_of(&mut self, layer: usize, k: usize) {
        let max_spc = self.tech.layers[layer].spacing.as_ref().map_or(0, |t| t.find_max());
        let q = bloat(&self.shapes[layer][k].rect, max_spc);
        for o in self.query(layer, &q) {
            self.metal_spacing_pair(layer, k, o);
        }
    }

    /// Two shapes: overlapping or touching is a short (or non-sufficient metal within one owner);
    /// apart, the spacing table.
    fn metal_spacing_pair(&mut self, layer: usize, k1: usize, k2: usize) {
        if k1 == k2 {
            return;
        }
        let (r1, r2) = (self.shapes[layer][k1], self.shapes[layer][k2]);
        let dist_x = gap((r1.rect.xl, r1.rect.xh), (r2.rect.xl, r2.rect.xh));
        let dist_y = gap((r1.rect.yl, r1.rect.yh), (r2.rect.yl, r2.rect.yh));
        let mut marker = generalized_intersect(&r1.rect, &r2.rect);
        let mut prl_x = marker.dx();
        let mut prl_y = marker.dy();
        if dist_x != 0 {
            prl_x = -prl_x;
        }
        if dist_y != 0 {
            prl_y = -prl_y;
        }
        if dist_x == 0 && dist_y == 0 {
            // A marker with no extent in a direction gets 1 each side there.
            let abut = prl_x == 0 || prl_y == 0;
            if prl_x == 0 {
                marker.xl -= 1;
                marker.xh += 1;
            }
            if prl_y == 0 {
                marker.yl -= 1;
                marker.yh += 1;
            }
            if self.owner(r1.net).is_blockage() || self.owner(r2.net).is_blockage() {
                self.short_with_blockage(layer, r1, r2, marker, abut);
            } else {
                self.short(layer, r1, r2, marker);
            }
        } else {
            self.spacing_table(layer, r1, r2, marker, prl_x.max(prl_y), dist_x, dist_y);
        }
    }

    /// The spacing two shapes need: the table at the wider of the two widths (a blockage counts
    /// as the layer's width) and the run length.
    fn required_spacing(&self, layer: usize, r1: &Shape, r2: &Shape, prl: i32) -> i32 {
        let l = &self.tech.layers[layer];
        let w = |s: &Shape| if self.owner(s.net).is_blockage() { l.width } else { width(&s.rect) };
        l.spacing.as_ref().map_or(0, |t| t.find(w(r1).max(w(r2)), prl))
    }

    /// Apart but closer than the table allows is a violation only when the gap lies between TRUE
    /// boundary edges of the two owners (both sides of it), and some trial shape reaches it.
    #[allow(clippy::too_many_arguments)]
    fn spacing_table(&mut self, layer: usize, r1: Shape, r2: Shape, marker: Rect, prl: i32, dist_x: i32, dist_y: i32) {
        if r1.fixed && r2.fixed {
            return;
        }
        let req = i64::from(self.required_spacing(layer, &r1, &r2, prl));
        if i64::from(dist_x).pow(2) + i64::from(dist_y).pow(2) >= req * req {
            return;
        }
        // Which sides need an edge: both horizontal sides (0), both vertical (1), either (2).
        let kind = if prl <= 0 {
            2
        } else if dist_x == 0 {
            0
        } else {
            1
        };
        if !self.has_poly_edges(layer, &r1, &r2, &marker, kind, prl) {
            return;
        }
        if !self.has_route(layer, &r1, &marker) && !self.has_route(layer, &r2, &marker) {
            return;
        }
        self.add_marker(Rule::MetalSpacing, layer, marker, r1.net, r2.net);
    }

    /// Whether the marker's sides lie on boundary edges of either owner.
    ///
    /// With no run length (`prl <= 0`) an edge counts when it lies ON the side. With a run length
    /// an edge counts when it runs the right way at the side or inside the marker, strictly within
    /// the marker's span the other way: a bottom side needs an edge running WEST (a top boundary
    /// below the gap), a top side one running EAST, a left side one running NORTH, a right side
    /// one running SOUTH.
    fn has_poly_edges(&self, layer: usize, r1: &Shape, r2: &Shape, m: &Rect, kind: u8, prl: i32) -> bool {
        let (mut b, mut t, mut l, mut r) = (false, false, false, false);
        for e in &self.edges[layer] {
            if e.net != r1.net && e.net != r2.net {
                continue;
            }
            let seg = Rect::new(e.from.0, e.from.1, e.to.0, e.to.1);
            if !touches(&seg, m) {
                continue;
            }
            let (lo, hi) = (e.from, e.to);
            let d = e.dir();
            if prl <= 0 {
                if d == EdgeDir::W && lo.1 == m.yl {
                    b = true;
                } else if d == EdgeDir::E && lo.1 == m.yh {
                    t = true;
                } else if d == EdgeDir::N && lo.0 == m.xl {
                    l = true;
                } else if d == EdgeDir::S && lo.0 == m.xh {
                    r = true;
                }
            } else if d == EdgeDir::W && lo.1 >= m.yl && lo.1 < m.yh && lo.0 > m.xl && hi.0 < m.xh {
                b = true;
            } else if d == EdgeDir::E && lo.1 > m.yl && lo.1 <= m.yh && hi.0 > m.xl && lo.0 < m.xh {
                t = true;
            } else if d == EdgeDir::N && lo.0 >= m.xl && lo.0 < m.xh && hi.1 > m.yl && lo.1 < m.yh {
                l = true;
            } else if d == EdgeDir::S && lo.0 > m.xl && lo.0 <= m.xh && lo.1 > m.yl && hi.1 < m.yh {
                r = true;
            }
        }
        ((kind == 0 || kind == 2) && b && t) || ((kind == 1 || kind == 2) && l && r)
    }

    /// Whether a trial shape of the rectangle's owner lies near the marker: the rectangle's part
    /// within one width of the marker is not wholly covered by the owner's fixed shapes.
    fn has_route(&self, layer: usize, s: &Shape, marker: &Rect) -> bool {
        let near = bloat(marker, width(&s.rect));
        let Some(part) = overlap(&near, &s.rect) else { return false };
        area_in(&self.nets[s.net].fixed_slices[layer], &part) < area(&part)
    }

    /// Overlapping shapes: nothing when neither owner has a trial shape at the overlap, or (one
    /// owner) when the metal there is sufficient; else a short, or non-sufficient metal within one
    /// owner.
    fn short(&mut self, layer: usize, r1: Shape, r2: Shape, marker: Rect) {
        if r1.fixed && r2.fixed {
            return;
        }
        if self.short_all_fixed(layer, &r1, &r2, &marker) {
            return;
        }
        if self.short_same_net_sufficient(layer, &r1, &r2, &marker) {
            return;
        }
        let rule = if r1.net == r2.net { Rule::NonSufficientMetal } else { Rule::Short };
        self.add_marker(rule, layer, marker, r1.net, r2.net);
    }

    /// Neither owner's trial shapes cover any area of the marker (widened by 1 where it has no
    /// extent).
    fn short_all_fixed(&self, layer: usize, r1: &Shape, r2: &Shape, marker: &Rect) -> bool {
        let mut m = *marker;
        if m.dx() == 0 {
            m.xl -= 1;
            m.xh += 1;
        }
        if m.dy() == 0 {
            m.yl -= 1;
            m.yh += 1;
        }
        let none = |net: usize| self.nets[net].route_slices[layer].iter().all(|s| overlap(s, &m).is_none());
        none(r1.net) && none(r2.net)
    }

    /// Within one owner the overlap is sufficient when: its diagonal is at least the minimum
    /// width; or either rectangle is itself narrower than the minimum width; or a third rectangle
    /// of the owner, at least minimum width both ways, contains the overlap and meets each of the
    /// two with a diagonal of at least the minimum width.
    fn short_same_net_sufficient(&self, layer: usize, r1: &Shape, r2: &Shape, m: &Rect) -> bool {
        if r1.net != r2.net {
            return false;
        }
        let mw = i64::from(self.tech.layers[layer].min_width);
        let diag = |r: &Rect| i64::from(r.dx()).pow(2) + i64::from(r.dy()).pow(2);
        if diag(m) >= mw * mw {
            return true;
        }
        let narrow = |r: &Rect| i64::from(r.dx()) < mw || i64::from(r.dy()) < mw;
        if narrow(&r1.rect) || narrow(&r2.rect) {
            return true;
        }
        let (cx, cy) = ((m.xl + m.xh) / 2, (m.yl + m.yh) / 2);
        let q = bloat(&Rect { xl: cx, yl: cy, xh: cx, yh: cy }, mw as i32);
        for k in self.query(layer, &q) {
            let o = &self.shapes[layer][k];
            if (o.rect == r1.rect && o.net == r1.net && o.fixed == r1.fixed) || (o.rect == r2.rect && o.net == r2.net && o.fixed == r2.fixed) {
                continue;
            }
            if o.net != r1.net || !contains(&o.rect, m) || narrow(&o.rect) {
                continue;
            }
            if let (Some(a), Some(b)) = (meet(&r1.rect, &o.rect), meet(&r2.rect, &o.rect)) {
                if diag(&a) >= mw * mw && diag(&b) >= mw * mw {
                    return true;
                }
            }
        }
        false
    }

    /// A short with a blockage: none between two blockages; none when the shapes only abut and
    /// need no spacing; none where the other owner's fixed shapes contain the marker; else a short
    /// check on each maximal rectangle of the marker that those fixed shapes do not cover.
    fn short_with_blockage(&mut self, layer: usize, r1: Shape, r2: Shape, marker: Rect, abut: bool) {
        if r1.fixed && r2.fixed {
            return;
        }
        let (b1, b2) = (self.owner(r1.net).is_blockage(), self.owner(r2.net).is_blockage());
        if b1 && b2 {
            return;
        }
        if abut && self.required_spacing(layer, &r1, &r2, 0) == 0 {
            return;
        }
        let (r1, r2) = if b1 { (r2, r1) } else { (r1, r2) };
        let mut pins = Vec::new();
        for f in &self.nets[r1.net].fixed_max[layer] {
            if contains(f, &marker) {
                return;
            }
            if touches(f, &marker) {
                pins.push(*f);
            }
        }
        for r in max_rects_of_difference(&marker, &pins) {
            let r3 = Shape { rect: r, ..r2 };
            let m = meet(&marker, &r).unwrap_or(marker);
            self.short(layer, r1, r3, m);
        }
    }

    // ---- cut spacing ----

    /// Per cut layer with a spacing, per owner, per cut: every cut within the spacing.
    fn check_cut_spacing(&mut self) {
        for layer in 0..self.tech.layers.len() {
            let l = &self.tech.layers[layer];
            let Some(spc) = l.cut_spacing.filter(|_| l.kind == LayerKind::Cut) else { continue };
            for net in 0..self.nets.len() {
                let mine: Vec<usize> = (0..self.shapes[layer].len()).filter(|&k| self.shapes[layer][k].net == net).collect();
                for k in mine {
                    let q = bloat(&self.shapes[layer][k].rect, spc);
                    for o in self.query(layer, &q) {
                        self.cut_pair(layer, k, o, spc);
                    }
                }
            }
        }
    }

    /// Two cuts: overlapping or touching is a short; apart, closer than the spacing (edge to edge)
    /// is a cut-spacing violation. Same-owner cuts are checked too.
    fn cut_pair(&mut self, layer: usize, k1: usize, k2: usize, spc: i32) {
        if k1 == k2 {
            return;
        }
        let (r1, r2) = (self.shapes[layer][k1], self.shapes[layer][k2]);
        let dist_x = gap((r1.rect.xl, r1.rect.xh), (r2.rect.xl, r2.rect.xh));
        let dist_y = gap((r1.rect.yl, r1.rect.yh), (r2.rect.yl, r2.rect.yh));
        let marker = generalized_intersect(&r1.rect, &r2.rect);
        if dist_x == 0 && dist_y == 0 {
            if r1.fixed && r2.fixed {
                return;
            }
            self.add_marker(Rule::Short, layer, marker, r1.net, r2.net);
            return;
        }
        let d2 = i64::from(dist_x).pow(2) + i64::from(dist_y).pow(2);
        if d2 >= i64::from(spc).pow(2) || (r1.fixed && r2.fixed) {
            return;
        }
        self.add_marker(Rule::CutSpacing, layer, marker, r1.net, r2.net);
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::tech::{Dir, Layer, SpacingTable};

    /// li1-like routing layer 2 (width 170, spacing 170 at any width), a cut layer 3 (spacing
    /// 190), met1-like routing layer 4 (width 140, pitch 370; 140, or 280 above width 3000).
    pub(crate) fn tech() -> Tech {
        let table = |rows: Vec<(i32, i32)>| SpacingTable { widths: rows.iter().map(|r| r.0).collect(), prls: vec![0], values: rows.iter().map(|r| vec![r.1]).collect() };
        Tech {
            layers: vec![
                Layer::default(),
                Layer::default(),
                Layer { name: "l2".into(), kind: LayerKind::Routing, dir: Dir::Vertical, width: 170, min_width: 170, pitch: 480, wrong_way_width: 170, spacing: Some(table(vec![(0, 170)])), cut_spacing: None },
                Layer { name: "c3".into(), kind: LayerKind::Cut, width: 170, cut_spacing: Some(190), ..Layer::default() },
                Layer { name: "l4".into(), kind: LayerKind::Routing, dir: Dir::Horizontal, width: 140, min_width: 140, pitch: 370, wrong_way_width: 140, spacing: Some(table(vec![(0, 140), (3000, 280)])), cut_spacing: None },
            ],
            manufacturing_grid: 5,
            via_defs: Vec::new(),
        }
    }

    fn markers(shapes: &[(Owner, usize, Rect, bool)]) -> Vec<Marker> {
        let t = tech();
        let mut w = Worker::new(&t);
        for (o, l, r, f) in shapes {
            w.add(o, *l, *r, *f);
        }
        w.init();
        w.run().to_vec()
    }

    fn net(n: &str) -> Owner {
        Owner::Net(n.into())
    }

    /// The table's row is the last width STRICTLY below: a width equal to a row's is the row
    /// before, one above it the row itself.
    #[test]
    fn spacing_table_row_is_last_strictly_below() {
        let t = SpacingTable { widths: vec![0, 3000], prls: vec![0], values: vec![vec![140], vec![280]] };
        assert_eq!(t.find(3000, 0), 140);
        assert_eq!(t.find(3001, 0), 280);
        assert_eq!(t.find(0, 0), 140);
        assert_eq!((t.find_min(), t.find_max()), (140, 280));
    }

    /// Two fixed shapes of different owners that overlap are never a violation.
    #[test]
    fn fixed_against_fixed_is_silent() {
        let m = markers(&[(net("a"), 4, Rect::new(0, 0, 1000, 140), true), (net("b"), 4, Rect::new(500, 0, 1500, 140), true)]);
        assert!(m.is_empty());
    }

    /// A trial shape over another owner's fixed shape is a short, once for the pair.
    #[test]
    fn trial_over_other_owner_is_one_short() {
        let m = markers(&[(net("a"), 4, Rect::new(0, 0, 1000, 140), true), (net("b"), 4, Rect::new(500, 0, 1500, 140), false)]);
        assert_eq!(m.len(), 1);
        assert_eq!((m[0].rule, m[0].bbox), (Rule::Short, Rect::new(500, 0, 1000, 140)));
    }

    /// Side by side 100 apart (the table asks 140) with a run length: a spacing violation; at 140
    /// apart none.
    #[test]
    fn parallel_run_closer_than_table() {
        let near = markers(&[(net("a"), 4, Rect::new(0, 0, 1000, 140), true), (net("b"), 4, Rect::new(0, 240, 1000, 380), false)]);
        assert_eq!(near.iter().map(|m| m.rule).collect::<Vec<_>>(), vec![Rule::MetalSpacing]);
        let far = markers(&[(net("a"), 4, Rect::new(0, 0, 1000, 140), true), (net("b"), 4, Rect::new(0, 280, 1000, 420), false)]);
        assert!(far.is_empty());
    }

    /// The width that picks the row is the WIDER shape's, and a width of exactly 3000 is still the
    /// first row: 200 apart passes at width 3000, fails at 3010.
    #[test]
    fn wide_shape_picks_the_wide_row() {
        let at = |w: i32| markers(&[(net("a"), 4, Rect::new(0, 0, 10000, w), true), (net("b"), 4, Rect::new(0, w + 200, 10000, w + 340), false)]);
        assert!(at(3000).is_empty());
        assert_eq!(at(3010).len(), 1);
    }

    /// Diagonal neighbours (no run length) are measured corner to corner: 100 x 100 apart is 141 —
    /// passes 140; 90 x 90 (127) fails.
    #[test]
    fn diagonal_is_euclidean() {
        let at = |d: i32| markers(&[(net("a"), 4, Rect::new(0, 0, 1000, 140), true), (net("b"), 4, Rect::new(1000 + d, 140 + d, 2000, 280 + d), false)]);
        assert!(at(100).is_empty());
        assert_eq!(at(90).len(), 1);
    }

    /// Same owner, overlapping through less than the minimum width: non-sufficient metal; through
    /// the full width: nothing.
    #[test]
    fn same_owner_thin_overlap_is_non_sufficient() {
        // Two 140-wide bars meeting only corner to corner over 50 x 50.
        let m = markers(&[(net("a"), 4, Rect::new(0, 0, 1000, 140), true), (net("a"), 4, Rect::new(950, 90, 1950, 230), false)]);
        assert!(m.iter().any(|m| m.rule == Rule::NonSufficientMetal), "{m:?}");
        let full = markers(&[(net("a"), 4, Rect::new(0, 0, 1000, 140), true), (net("a"), 4, Rect::new(500, 0, 1500, 140), false)]);
        assert!(full.is_empty());
    }

    /// A trial shape over an instance blockage is a short, except where the trial owner's own
    /// fixed shapes already cover the overlap.
    #[test]
    fn blockage_short_skips_own_pin_cover() {
        let inst = Owner::Inst("u1".into());
        let bare = markers(&[(inst.clone(), 4, Rect::new(0, 0, 1000, 140), true), (net("a"), 4, Rect::new(500, 0, 1500, 140), false)]);
        assert_eq!(bare.iter().map(|m| m.rule).collect::<Vec<_>>(), vec![Rule::Short]);
        let covered = markers(&[
            (inst, 4, Rect::new(0, 0, 1000, 140), true),
            (net("a"), 4, Rect::new(400, 0, 1100, 140), true),
            (net("a"), 4, Rect::new(500, 0, 1500, 140), false),
        ]);
        assert!(covered.is_empty(), "{covered:?}");
    }

    /// Cuts 150 apart (spacing 190): cut spacing; overlapping: a short; two fixed: nothing.
    #[test]
    fn cut_spacing_and_cut_short() {
        let c = |x: i32| Rect::new(x, 0, x + 170, 170);
        let m = markers(&[(net("a"), 3, c(0), true), (net("b"), 3, c(320), false)]);
        assert_eq!(m.iter().map(|m| m.rule).collect::<Vec<_>>(), vec![Rule::CutSpacing]);
        let s = markers(&[(net("a"), 3, c(0), true), (net("b"), 3, c(100), false)]);
        assert_eq!(s.iter().map(|m| m.rule).collect::<Vec<_>>(), vec![Rule::Short]);
        let f = markers(&[(net("a"), 3, c(0), true), (net("b"), 3, c(100), true)]);
        assert!(f.is_empty());
        // Exactly the spacing apart passes.
        assert!(markers(&[(net("a"), 3, c(0), true), (net("b"), 3, c(360), false)]).is_empty());
    }

    fn rules(m: &[Marker]) -> Vec<Rule> {
        m.iter().map(|m| m.rule).collect()
    }

    /// A blockage's width in the table is the LAYER's width, not its own: a 4000-wide obstruction
    /// 200 from a trial needs 140 (the first row), not 280.
    #[test]
    fn blockage_counts_as_layer_width() {
        let m = markers(&[(Owner::Inst("u".into()), 4, Rect::new(0, 0, 4000, 4000), true), (net("a"), 4, Rect::new(0, 4200, 4000, 4340), false)]);
        assert!(m.is_empty(), "{m:?}");
    }

    /// Too close is not enough: some trial shape of either owner must lie within one width of the
    /// gap. Here the other owner's trial extends its shape far away (the maximal rectangle is not
    /// fixed), but next to the gap it is all fixed metal — nothing, and an EQUAL area is "all
    /// fixed".
    #[test]
    fn spacing_needs_a_trial_shape_near_the_gap() {
        let m = markers(&[
            (net("a"), 4, Rect::new(0, 0, 500, 140), true),
            (net("b"), 4, Rect::new(0, 240, 1000, 380), true),
            (net("b"), 4, Rect::new(900, 240, 5000, 380), false),
        ]);
        assert!(m.is_empty(), "{m:?}");
    }

    /// With a run length, a bottom side needs a westward edge at or ABOVE the marker's bottom and
    /// strictly BELOW its top. The lower shape's own top edge is outside the marker's span (a bump
    /// covers it there), and the bump's top lies exactly at the marker's top — which does not
    /// count: no spacing marker for that pair (only the bump's short).
    #[test]
    fn a_bottom_edge_at_the_markers_top_does_not_count() {
        let m = markers(&[
            (net("a"), 4, Rect::new(0, 0, 1000, 140), true),
            (net("a"), 4, Rect::new(0, 140, 500, 240), true),
            (net("b"), 4, Rect::new(0, 240, 400, 380), false),
        ]);
        assert!(!rules(&m).contains(&Rule::MetalSpacing), "{m:?}");
        assert!(rules(&m).contains(&Rule::Short), "{m:?}");
    }

    /// No run length at all (touching corner-wise in y, prl exactly 0) accepts EITHER pair of
    /// sides: here the left side has no edge (the lower owner's boundary there is interior), yet
    /// the bottom and top edges make it a violation.
    #[test]
    fn zero_run_length_accepts_either_pair_of_sides() {
        let m = markers(&[
            (net("a"), 4, Rect::new(0, 0, 1000, 1000), true),
            (net("a"), 4, Rect::new(1000, 600, 1050, 1000), true),
            (net("a"), 4, Rect::new(1000, 0, 3000, 500), true),
            (net("b"), 4, Rect::new(1100, 1000, 2100, 2000), false),
        ]);
        assert!(m.iter().any(|m| m.rule == Rule::MetalSpacing && m.bbox == Rect::new(1000, 1000, 1100, 1000)), "{m:?}");
    }

    /// A trial drawn INSIDE its own pin leaves the pin's maximal rectangle fixed, so an overlap
    /// with another owner's fixed shape there is fixed against fixed: nothing.
    #[test]
    fn a_trial_inside_its_pin_keeps_the_pin_fixed() {
        let m = markers(&[
            (net("a"), 4, Rect::new(0, 0, 1000, 140), true),
            (net("a"), 4, Rect::new(850, 0, 950, 140), false),
            (net("b"), 4, Rect::new(800, 0, 1800, 140), true),
        ]);
        assert!(m.is_empty(), "{m:?}");
    }

    /// Shapes that only abut make a short whose marker is widened by 1 on each side of the zero
    /// extent.
    #[test]
    fn an_abutting_short_marker_is_widened() {
        let m = markers(&[(net("a"), 4, Rect::new(0, 0, 1000, 140), true), (net("b"), 4, Rect::new(1000, 0, 2000, 140), false)]);
        assert_eq!(m.iter().map(|m| (m.rule, m.bbox)).collect::<Vec<_>>(), vec![(Rule::Short, Rect::new(999, 0, 1001, 140))]);
    }

    /// One owner's overlap whose diagonal is EXACTLY the minimum width (84 x 112 → 140) is
    /// sufficient.
    #[test]
    fn a_diagonal_of_exactly_min_width_is_sufficient() {
        let m = markers(&[(net("a"), 4, Rect::new(0, 0, 500, 500), true), (net("a"), 4, Rect::new(416, 388, 1000, 1000), false)]);
        assert!(m.is_empty(), "{m:?}");
    }

    /// A thin overlap is not judged when either rectangle is itself narrower than the minimum
    /// width.
    #[test]
    fn a_narrow_rectangle_is_not_judged_for_sufficient_metal() {
        let m = markers(&[(net("a"), 4, Rect::new(0, 0, 1000, 100), true), (net("a"), 4, Rect::new(950, 50, 1950, 190), false)]);
        assert!(m.is_empty(), "{m:?}");
    }

    /// A square's boundary runs counter-clockwise: bottom east, right north, top west, left south.
    #[test]
    fn boundary_is_counter_clockwise() {
        let mut e = boundary(&[Rect::new(0, 0, 10, 10)]);
        e.sort();
        assert_eq!(e, vec![((0, 0), (10, 0)), ((0, 10), (0, 0)), ((10, 0), (10, 10)), ((10, 10), (0, 10))]);
    }
}
