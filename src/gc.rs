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
use crate::rtree::DynRTree;
use crate::tech::{EolRule, LayerKind, ParallelEdge, Tech};

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
    /// A routing blockage (each its own owner, by its index in the design).
    Blockage(usize),
    /// Every unconnected ground terminal's design shapes.
    FloatingGround,
    /// Every unconnected power terminal's design shapes.
    FloatingPower,
}

impl Owner {
    pub fn is_blockage(&self) -> bool {
        matches!(self, Owner::Inst(_) | Owner::Blockage(_))
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
    /// A line end closer than its end-of-line spacing to a facing edge.
    EolSpacing,
    /// Left by the connectivity check between iterations where it removed or changed a net's
    /// shape: the next iteration re-checks the worker it falls in (it is not itself a violation).
    Recheck,
    /// A slice of one owner's polygon narrower than the layer's minimum width.
    MinWidth,
    /// A polygon on a rect-only layer that is not one rectangle.
    RectOnly,
    /// One owner's polygon smaller than the layer's minimum area (`checkMetalShape_minArea`, the
    /// marker pass): what the patch pass could not fix, or found where no net is the target.
    MinArea,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct Marker {
    pub rule: Rule,
    pub layer: usize,
    pub bbox: Rect,
    /// The owners involved, sorted, without repeats.
    pub owners: Vec<Owner>,
    /// The checked shape's owner (layer, rectangle, fixed) and the other's — the check's own
    /// markers only: a COPY keeps its sources but not its sides (the design's markers and a
    /// worker's starting markers are copies).
    pub victim: Option<Side>,
    pub aggressor: Option<Side>,
}

/// The markers' final order: each run of consecutive markers alike in layer, rule, x extent and
/// owners is stably sorted by bottom (ascending), then top (descending), then area (descending).
/// (⚠️ A rule is its kind here: two end-of-line rules on one layer would share a run.)
pub fn normalize_marker_order(markers: &mut [Marker]) {
    let same_run = |a: &Marker, b: &Marker| a.layer == b.layer && a.rule == b.rule && a.bbox.xl == b.bbox.xl && a.bbox.xh == b.bbox.xh && a.owners == b.owners;
    let area = |r: &Rect| i64::from(r.xh - r.xl) * i64::from(r.yh - r.yl);
    let mut begin = 0;
    while begin < markers.len() {
        let mut end = begin + 1;
        while end < markers.len() && same_run(&markers[begin], &markers[end]) {
            end += 1;
        }
        markers[begin..end].sort_by(|a, b| a.bbox.yl.cmp(&b.bbox.yl).then(b.bbox.yh.cmp(&a.bbox.yh)).then(area(&b.bbox).cmp(&area(&a.bbox))));
        begin = end;
    }
}

/// A marker side: its owner, layer, rectangle, and whether the shape is fixed.
pub type Side = (Owner, usize, Rect, bool);

impl Marker {
    /// A copy as the design and a worker's starting list hold it: its sources, not its sides.
    pub fn copied(&self) -> Marker {
        Marker { victim: None, aggressor: None, ..self.clone() }
    }
}

/// A maximal rectangle of one owner on one layer.
#[derive(Debug, Clone, Copy)]
struct Shape {
    rect: Rect,
    net: usize,
    fixed: bool,
    /// A route rectangle touching one of its net's tapered shapes (a non-default-rule net inside
    /// a pin's taper box): the rule's spacing does not apply to it.
    tapered: bool,
    /// Its pin (piece) on the layer, in the reference's order.
    pin: u32,
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

/// A polygon edge of one owner's merged shapes on a layer, vertex to vertex, inside on its left.
#[derive(Debug, Clone, Copy)]
struct Seg {
    from: (i32, i32),
    to: (i32, i32),
    net: usize,
    /// Which polygon of the owner it bounds (a connected piece of the owner's shapes, holes
    /// included).
    pin: usize,
    /// On the owner's FIXED shapes' boundary too (or a fixed rectangle's).
    fixed: bool,
    prev: usize,
    next: usize,
}

impl Seg {
    fn dir(&self) -> EdgeDir {
        Edge { from: self.from, to: self.to, net: self.net }.dir()
    }
    fn len(&self) -> i32 {
        (self.to.0 - self.from.0).abs() + (self.to.1 - self.from.1).abs()
    }
    fn vec(&self) -> (i64, i64) {
        (i64::from(self.to.0 - self.from.0), i64::from(self.to.1 - self.from.1))
    }
}

/// The turn from `a` to `b`: 1 left, -1 right, 0 parallel (the sign of the cross product).
fn orientation(a: &Seg, b: &Seg) -> i32 {
    let ((a1, b1), (a2, b2)) = (a.vec(), b.vec());
    (a1 * b2 - b1 * a2).signum() as i32
}

/// A region's polygon edges, with their polygon, whether fixed, and the ring order. At a point
/// where two polygons touch only at a corner, an edge continues with the LEFT turn — each polygon
/// stays its own.
/// An edge as its two points.
type EdgePoints = ((i32, i32), (i32, i32));

fn polygon_segs(out: &mut Vec<Seg>, slices: &[Rect], fixed_edges: &BTreeSet<EdgePoints>, net: usize) {
    // The polygons: slices joined where they share a boundary of some length.
    let mut parent: Vec<usize> = (0..slices.len()).collect();
    fn find(p: &mut [usize], i: usize) -> usize {
        let mut r = i;
        while p[r] != r {
            r = p[r];
        }
        p[i] = r;
        r
    }
    for i in 0..slices.len() {
        for j in i + 1..slices.len() {
            let (a, b) = (&slices[i], &slices[j]);
            let x_run = a.xh.min(b.xh) - a.xl.max(b.xl);
            let y_run = a.yh.min(b.yh) - a.yl.max(b.yl);
            let joined = ((a.xh == b.xl || b.xh == a.xl) && y_run > 0) || ((a.yh == b.yl || b.yh == a.yl) && x_run > 0);
            if joined {
                let (ri, rj) = (find(&mut parent, i), find(&mut parent, j));
                parent[ri] = rj;
            }
        }
    }
    let base = out.len();
    let raw = boundary(slices);
    for &(from, to) in &raw {
        let e = Edge { from, to, net };
        // The slice on the edge's inside.
        let inside = slices.iter().position(|s| match e.dir() {
            EdgeDir::E => s.yl == from.1 && s.xl.max(from.0) < s.xh.min(to.0),
            EdgeDir::W => s.yh == from.1 && s.xl.max(to.0) < s.xh.min(from.0),
            EdgeDir::N => s.xh == from.0 && s.yl.max(from.1) < s.yh.min(to.1),
            EdgeDir::S => s.xl == from.0 && s.yl.max(to.1) < s.yh.min(from.1),
        });
        let pin = inside.map_or(usize::MAX, |k| find(&mut parent, k));
        out.push(Seg { from, to, net, pin, fixed: fixed_edges.contains(&(from, to)), prev: usize::MAX, next: usize::MAX });
    }
    let mut starts: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
    for (k, seg) in out.iter().enumerate().skip(base) {
        starts.entry(seg.from).or_default().push(k);
    }
    for k in base..out.len() {
        let cands = &starts[&out[k].to];
        let next = if cands.len() == 1 { cands[0] } else { *cands.iter().find(|&&c| orientation(&out[k], &out[c]) == 1).unwrap_or(&cands[0]) };
        out[k].next = next;
        out[next].prev = k;
    }
}

#[derive(Debug, Clone, Default)]
struct Net {
    owner: Option<Owner>,
    fixed: Vec<Polygon90Set>,
    route: Vec<Polygon90Set>,
    fixed_cuts: Vec<Vec<Rect>>,
    route_cuts: Vec<Vec<Rect>>,
    /// The fixed routing-layer rectangles as added (their edges count as fixed edges too).
    fixed_rects: Vec<Vec<Rect>>,
    /// After `init`: the fixed and the route shapes as disjoint slices, and the fixed shapes'
    /// maximal rectangles.
    fixed_slices: Vec<Vec<Rect>>,
    route_slices: Vec<Vec<Rect>>,
    fixed_max: Vec<Vec<Rect>>,
    /// A non-default-rule net: its spacing per z (layer / 2 − 1).
    ndr_spacing: Option<Vec<i32>>,
    /// A non-default-rule net's route shapes, tapered or not, per layer.
    tapered: Vec<Vec<Rect>>,
    non_tapered: Vec<Vec<Rect>>,
    /// After `init`: per layer, the owner's pins (connected pieces of its fixed and route shapes
    /// merged), in the reference's order.
    pins: Vec<Vec<Polygon90Set>>,
}

pub struct Worker<'a> {
    tech: &'a Tech,
    nets: Vec<Net>,
    index: HashMap<Owner, usize>,
    shapes: Vec<Vec<Shape>>,
    edges: Vec<Vec<Edge>>,
    /// Per layer, every owner's polygon edges (for end-of-line checks).
    segs: Vec<Vec<Seg>>,
    /// Skip end-of-line checks on edges running along the layer's direction above the first metal
    /// layer (the long sides of a wire): set for via and pattern trials, not planar ones.
    pub ignore_long_side_eol: bool,
    /// Check only from this owner's shapes (the detailed router checks the net it just routed
    /// against everything around it); every owner when unset.
    pub target: Option<Owner>,
    /// Apply non-default rules (the detailed router's checks): the largest rule spacing widens
    /// the search, a rule net's untapered route rectangles need its spacing, and each tapered
    /// rectangle's untapered neighbours are checked as special spacing rectangles.
    pub check_ndrs: bool,
    /// Skip the minimum-area check (`setIgnoreMinArea`): pin access sets it for its trials.
    pub ignore_min_area: bool,
    /// The worker's check box (`drcBox_`): a minimum-area marker needs its polygon WHOLLY inside.
    /// Unset: the whole design (the check between iterations).
    pub drc_box: Option<Rect>,
    /// Per z, the largest spacing of any non-default rule in the technology.
    pub max_ndr_spacing: Vec<i32>,
    /// Per layer, the special spacing rectangles (after `init`; by index, never reused), whether
    /// each is still in its owner's list, and the layer's tree of them — which the reference
    /// keeps by value: packed at `init`, a removal takes the first equal rectangle in tree order.
    spc: Vec<Vec<Shape>>,
    spc_listed: Vec<Vec<bool>>,
    spc_rq: Vec<DynRTree<usize>>,
    /// Per layer: whether each shape (by its index, never reused) still stands, its id in the
    /// layer's tree, and the tree — packed at `init`, then updated as the reference's is.
    alive: Vec<Vec<bool>>,
    rq_id: Vec<Vec<usize>>,
    rq: Vec<DynRTree<usize>>,
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

/// A one-unit strip just inside an edge, along its length.
fn parallel_edge_rect(e: &Seg) -> Rect {
    let (lo, hi) = (e.from, e.to);
    match e.dir() {
        EdgeDir::E => Rect { xl: lo.0, yl: lo.1, xh: hi.0, yh: hi.1 + 1 },
        EdgeDir::W => Rect { xl: hi.0, yl: hi.1 - 1, xh: lo.0, yh: lo.1 },
        EdgeDir::N => Rect { xl: lo.0 - 1, yl: lo.1, xh: hi.0, yh: hi.1 },
        EdgeDir::S => Rect { xl: hi.0, yl: hi.1, xh: lo.0 + 1, yh: lo.1 },
    }
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
        let mut w = Worker { tech, nets: Vec::new(), index: HashMap::new(), shapes: Vec::new(), edges: Vec::new(), segs: Vec::new(), ignore_long_side_eol: false, target: None, check_ndrs: false, ignore_min_area: false, drc_box: None, max_ndr_spacing: Vec::new(), spc: Vec::new(), spc_listed: Vec::new(), spc_rq: Vec::new(), alive: Vec::new(), rq_id: Vec::new(), rq: Vec::new(), markers: Vec::new(), seen: BTreeSet::new() };
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
            fixed_rects: vec![Vec::new(); n],
            tapered: vec![Vec::new(); n],
            non_tapered: vec![Vec::new(); n],
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
            (false, true) => {
                net.fixed[layer].insert_rect(r);
                net.fixed_rects[layer].push(r);
            }
            (false, false) => net.route[layer].insert_rect(r),
        }
    }

    /// Every owner, in creation order.
    pub fn owners(&self) -> Vec<Owner> {
        self.nets.iter().filter_map(|n| n.owner.clone()).collect()
    }

    /// An owner's maximal rectangles per layer and pin: `|<layer>:<pin>:<xl,yl,xh,yh;>...`.
    pub fn dump(&self, owner: &Owner) -> String {
        let Some(&i) = self.index.get(owner) else { return String::new() };
        let mut out = String::new();
        for layer in 0..self.shapes.len() {
            let mut cur: Option<u32> = None;
            for (k, sh) in self.shapes[layer].iter().enumerate() {
                if !self.alive[layer][k] || sh.net != i {
                    continue;
                }
                if cur != Some(sh.pin) {
                    out += &format!("|{layer}:{}:", sh.pin);
                    cur = Some(sh.pin);
                }
                out += &format!("{},{},{},{};", sh.rect.xl, sh.rect.yl, sh.rect.xh, sh.rect.yh);
            }
        }
        out
    }

    /// Create an owner (in order) if it is not there yet.
    pub fn ensure_owner(&mut self, owner: &Owner) {
        self.net(owner);
    }

    /// A non-default-rule owner: its spacing per z.
    pub fn set_ndr_spacing(&mut self, owner: &Owner, spacing: Vec<i32>) {
        let i = self.net(owner);
        self.nets[i].ndr_spacing = Some(spacing);
    }

    /// A non-default-rule owner's route shape, tapered or not (besides [`Worker::add`]).
    pub fn add_taper(&mut self, owner: &Owner, layer: usize, r: Rect, tapered: bool) {
        let i = self.net(owner);
        if tapered {
            self.nets[i].tapered[layer].push(r);
        } else {
            self.nets[i].non_tapered[layer].push(r);
        }
    }

    /// Per owner and layer: the merged shapes' pieces (pins) and each pin's maximal rectangles
    /// (fixed or not), boundary edges; cut rectangles as they are (route ones first). The shapes
    /// of every layer packed into its tree in owner, pin, rectangle order.
    pub fn init(&mut self) {
        let n = self.tech.layers.len();
        self.shapes = vec![Vec::new(); n];
        self.alive = vec![Vec::new(); n];
        self.rq_id = vec![Vec::new(); n];
        self.edges = vec![Vec::new(); n];
        self.segs = vec![Vec::new(); n];
        self.spc = vec![Vec::new(); n];
        self.spc_listed = vec![Vec::new(); n];
        for i in 0..self.nets.len() {
            self.build_net(i);
        }
        self.rq = (0..n).map(|l| DynRTree::new(self.shapes[l].iter().enumerate().map(|(k, s)| (s.rect, k)).collect())).collect();
        self.rq_id = (0..n).map(|l| (0..self.shapes[l].len()).collect()).collect();
        self.spc_rq = (0..n).map(|l| DynRTree::new(self.spc[l].iter().enumerate().map(|(k, s)| (s.rect, k)).collect())).collect();
    }

    /// One owner's pins, maximal rectangles, edges and special spacing rectangles, appended.
    fn build_net(&mut self, i: usize) {
        let n = self.tech.layers.len();
        let net = &mut self.nets[i];
        net.fixed_slices = net.fixed.iter_mut().map(|s| s.rectangles()).collect();
        net.route_slices = net.route.iter_mut().map(|s| s.rectangles()).collect();
        net.fixed_max = net.fixed.iter_mut().map(|s| s.max_rectangles()).collect();
        net.pins = vec![Vec::new(); n];
        for layer in 0..n {
            let net = &self.nets[i];
            let mut all = Polygon90Set::new();
            for s in net.fixed_slices[layer].iter().chain(&net.route_slices[layer]) {
                all.insert_rect(*s);
            }
            let slices = all.rectangles();
            for (from, to) in boundary(&slices) {
                self.edges[layer].push(Edge { from, to, net: i });
            }
            if self.tech.layers[layer].kind == LayerKind::Routing && !slices.is_empty() {
                let mut fixed_edges: BTreeSet<EdgePoints> = boundary(&net.fixed_slices[layer]).into_iter().collect();
                for r in &net.fixed_rects[layer] {
                    fixed_edges.extend([((r.xl, r.yl), (r.xh, r.yl)), ((r.xh, r.yl), (r.xh, r.yh)), ((r.xh, r.yh), (r.xl, r.yh)), ((r.xl, r.yh), (r.xl, r.yl))]);
                }
                polygon_segs(&mut self.segs[layer], &slices, &fixed_edges, i);
            }
            let mut new_shapes: Vec<Shape> = Vec::new();
            let mut pin_k: u32 = 0;
            let pins = all.polygons();
            for mut pin in pins.iter().cloned() {
                let k = pin_k;
                pin_k += 1;
                for r in pin.max_rectangles() {
                    let fixed = net.fixed_max[layer].contains(&r);
                    let mut tapered = false;
                    if !fixed && net.tapered[layer].iter().any(|t| touches(&r, t)) {
                        tapered = true;
                        for nt in &net.non_tapered[layer] {
                            if touches(&r, nt) {
                                self.spc[layer].push(Shape { rect: *nt, net: i, fixed: false, tapered: false, pin: 0 });
                                self.spc_listed[layer].push(true);
                            }
                        }
                    }
                    new_shapes.push(Shape { rect: r, net: i, fixed, tapered, pin: k });
                }
            }
            for &r in net.route_cuts[layer].iter().chain(&net.fixed_cuts[layer]) {
                let fixed = net.fixed_cuts[layer].contains(&r);
                new_shapes.push(Shape { rect: r, net: i, fixed, tapered: false, pin: pin_k });
                pin_k += 1;
            }
            if self.tech.layers[layer].kind == LayerKind::Routing {
                self.nets[i].pins[layer] = pins;
            }
            for sh in new_shapes {
                self.shapes[layer].push(sh);
                self.alive[layer].push(true);
                self.rq_id[layer].push(usize::MAX);
            }
        }
    }

    /// Replace an owner's route shapes (after `init`): its rectangles out of the trees (layer,
    /// pin, rectangle order), its pins rebuilt from its fixed shapes and these, the new
    /// rectangles in — as the reference's check updates a net it rerouted.
    pub fn replace_route(&mut self, owner: &Owner, route: &[(usize, Rect)], taper: &[(usize, Rect, bool)]) {
        let i = self.net(owner);
        let n = self.tech.layers.len();
        let old: Vec<usize> = self.shapes.iter().map(|v| v.len()).collect();
        let old_spc: Vec<usize> = self.spc.iter().map(|v| v.len()).collect();
        for layer in 0..n {
            for k in 0..self.shapes[layer].len() {
                if self.alive[layer][k] && self.shapes[layer][k].net == i {
                    self.rq[layer].remove(self.rq_id[layer][k]);
                    self.alive[layer][k] = false;
                }
            }
            self.edges[layer].retain(|e| e.net != i);
            self.segs[layer].retain(|e| e.net != i);
            for k in 0..self.spc[layer].len() {
                if self.spc_listed[layer][k] && self.spc[layer][k].net == i {
                    self.spc_rq[layer].remove_eq(&self.spc[layer][k].rect);
                    self.spc_listed[layer][k] = false;
                }
            }
        }
        {
            let net = &mut self.nets[i];
            net.route = vec![Polygon90Set::new(); n];
            net.route_cuts = vec![Vec::new(); n];
            net.tapered = vec![Vec::new(); n];
            net.non_tapered = vec![Vec::new(); n];
        }
        for &(l, r) in route {
            self.add(owner, l, r, false);
        }
        for &(l, r, t) in taper {
            self.add_taper(owner, l, r, t);
        }
        self.build_net(i);
        for (layer, &from) in old.iter().enumerate() {
            for k in from..self.shapes[layer].len() {
                let r = self.shapes[layer][k].rect;
                self.rq_id[layer][k] = self.rq[layer].insert(r, k);
            }
        }
        for (layer, &from) in old_spc.iter().enumerate() {
            for k in from..self.spc[layer].len() {
                let r = self.spc[layer][k].rect;
                self.spc_rq[layer].insert(r, k);
            }
        }
    }

    /// Metal spacing, then metal shapes (minimum width, rect-only), then end-of-line spacing, then
    /// cut spacing, over every owner's shapes; the markers made.
    pub fn run(&mut self) -> &[Marker] {
        self.markers.clear();
        self.seen.clear();
        self.check_metal_spacing();
        self.check_metal_shape();
        self.check_metal_end_of_line();
        self.check_cut_spacing();
        normalize_marker_order(&mut self.markers);
        &self.markers
    }

    /// Whether checks start from this owner's shapes.
    fn checks_from(&self, net: usize) -> bool {
        self.target.as_ref().is_none_or(|t| self.nets[net].owner.as_ref() == Some(t))
    }

    fn owner(&self, net: usize) -> &Owner {
        self.nets[net].owner.as_ref().expect("an owner")
    }

    fn add_marker(&mut self, rule: Rule, layer: usize, bbox: Rect, a: usize, b: usize) {
        self.add_marker_of(rule, layer, bbox, (a, bbox, false), (b, bbox, false));
    }

    /// A marker between the checked shape `v` and the other `g` (owner index, rectangle, fixed);
    /// kept once per (box, layer, rule, owners) — the first found keeps its victim and aggressor.
    fn add_marker_of(&mut self, rule: Rule, layer: usize, bbox: Rect, v: (usize, Rect, bool), g: (usize, Rect, bool)) {
        let mut owners = vec![self.owner(v.0).clone(), self.owner(g.0).clone()];
        owners.sort();
        owners.dedup();
        if self.seen.insert((bbox, layer, rule, owners.clone())) {
            let victim = (self.owner(v.0).clone(), layer, v.1, v.2);
            let aggressor = (self.owner(g.0).clone(), layer, g.1, g.2);
            self.markers.push(Marker { rule, layer, bbox, owners, victim: Some(victim), aggressor: Some(aggressor) });
        }
    }

    /// Every standing shape on `layer` touching `r`, in the tree's order.
    fn query(&self, layer: usize, r: &Rect) -> Vec<usize> {
        self.rq[layer].query(r).into_iter().map(|(_, v)| v.1).collect()
    }

    // ---- metal shapes ----

    /// Per routing layer (bottom up), per owner checked from, per pin: minimum width, minimum area,
    /// then rect-only — `checkMetalShape_main`'s order, less the checks not modelled.
    fn check_metal_shape(&mut self) {
        for layer in 0..self.tech.layers.len() {
            if self.tech.layers[layer].kind != LayerKind::Routing {
                continue;
            }
            for net in 0..self.nets.len() {
                if !self.checks_from(net) || self.nets[net].pins.len() <= layer {
                    continue;
                }
                for k in 0..self.nets[net].pins[layer].len() {
                    let mut pin = self.nets[net].pins[layer][k].clone();
                    self.metal_shape_of(layer, net, &mut pin);
                }
            }
        }
    }

    fn metal_shape_of(&mut self, layer: usize, net: usize, pin: &mut Polygon90Set) {
        // Minimum width: the pin sliced horizontally, each slice's x length; then sliced
        // vertically, each slice's y length.
        for r in pin.rectangles() {
            self.min_width(layer, net, r, r.xh - r.xl);
        }
        for r in vertical_slices(pin) {
            self.min_width(layer, net, r, r.yh - r.yl);
        }
        self.min_area(layer, net, pin);
        self.rect_only(layer, net, pin);
    }

    /// `checkMetalShape_minArea`, the marker pass: a polygon below the layer's minimum area gives a
    /// marker on its bounding box — unless the layer has no area rule, the check is ignored (pin
    /// access), the box is not WHOLLY inside the worker's check box, or any of its edges is fixed
    /// (on the owner's fixed shapes). The patch pass (`allow_patching`) is the router's write-out.
    fn min_area(&mut self, layer: usize, net: usize, pin: &mut Polygon90Set) {
        let required = self.tech.layers[layer].min_area;
        if self.ignore_min_area || required == 0 {
            return;
        }
        let slices = pin.rectangles();
        if slices.iter().map(area).sum::<i64>() >= required {
            return;
        }
        let bbox = slices.iter().skip(1).fold(slices[0], |b, r| Rect { xl: b.xl.min(r.xl), yl: b.yl.min(r.yl), xh: b.xh.max(r.xh), yh: b.yh.max(r.yh) });
        if let Some(d) = self.drc_box {
            if !(d.xl <= bbox.xl && d.yl <= bbox.yl && bbox.xh <= d.xh && bbox.yh <= d.yh) {
                return;
            }
        }
        let net_ref = &self.nets[net];
        let mut fixed_edges: BTreeSet<EdgePoints> = boundary(&net_ref.fixed_slices[layer]).into_iter().collect();
        for r in &net_ref.fixed_rects[layer] {
            fixed_edges.extend([((r.xl, r.yl), (r.xh, r.yl)), ((r.xh, r.yl), (r.xh, r.yh)), ((r.xh, r.yh), (r.xl, r.yh)), ((r.xl, r.yh), (r.xl, r.yl))]);
        }
        let mut segs = Vec::new();
        polygon_segs(&mut segs, &slices, &fixed_edges, net);
        if segs.iter().any(|e| e.fixed) {
            return;
        }
        self.add_marker(Rule::MinArea, layer, bbox, net, net);
    }

    /// A slice narrower than the minimum width, unless the owner's fixed shapes cover it whole.
    fn min_width(&mut self, layer: usize, net: usize, r: Rect, len: i32) {
        if len >= self.tech.layers[layer].min_width {
            return;
        }
        if area_in(&self.nets[net].fixed_slices[layer], &r) == area(&r) {
            return;
        }
        self.add_marker(Rule::MinWidth, layer, r, net, net);
    }

    /// On a rect-only layer, a pin that is not one rectangle: around each concave corner (a right
    /// turn from an edge into the next; the corner is the edge's end), the pin within the layer's
    /// minimum width each way — unless the owner's fixed shapes cover all of that — gives a marker
    /// per maximal rectangle.
    fn rect_only(&mut self, layer: usize, net: usize, pin: &mut Polygon90Set) {
        if !self.tech.layers[layer].rect_only {
            return;
        }
        if pin.max_rectangles().len() == 1 {
            return;
        }
        let w = self.tech.layers[layer].min_width;
        let slices = pin.rectangles();
        let mut segs = Vec::new();
        polygon_segs(&mut segs, &slices, &BTreeSet::new(), net);
        let corners: Vec<(i32, i32)> = segs.iter().filter(|e| orientation(e, &segs[e.next]) == -1).map(|e| e.to).collect();
        for c in corners {
            let window = Rect { xl: c.0 - w, yl: c.1 - w, xh: c.0 + w, yh: c.1 + w };
            let inside: Vec<Rect> = slices.iter().filter_map(|s| overlap(s, &window)).collect();
            let total: i64 = inside.iter().map(area).sum();
            let fixed: i64 = inside.iter().map(|r| area_in(&self.nets[net].fixed_slices[layer], r)).sum();
            if fixed == total {
                continue;
            }
            let mut set = Polygon90Set::new();
            for r in inside {
                set.insert_rect(r);
            }
            for m in set.max_rectangles() {
                self.add_marker(Rule::RectOnly, layer, m, net, net);
            }
        }
    }

    // ---- end-of-line spacing ----

    /// Per routing layer with end-of-line rules, per owner, per polygon edge: each rule —
    /// skipping, with `ignore_long_side_eol`, edges along the layer above the first metal layer.
    fn check_metal_end_of_line(&mut self) {
        for layer in 0..self.tech.layers.len() {
            let l = &self.tech.layers[layer];
            if l.kind != LayerKind::Routing || l.eol.is_empty() {
                continue;
            }
            let vertical = l.is_vertical();
            let rules = l.eol.clone();
            for net in 0..self.nets.len() {
                if !self.checks_from(net) {
                    continue;
                }
                for k in 0..self.segs[layer].len() {
                    let e = self.segs[layer][k];
                    if e.net != net {
                        continue;
                    }
                    if self.ignore_long_side_eol && layer > 2 {
                        let along = match e.dir() {
                            EdgeDir::N | EdgeDir::S => vertical,
                            EdgeDir::E | EdgeDir::W => !vertical,
                        };
                        if along {
                            continue;
                        }
                    }
                    for r in &rules {
                        self.check_eol(layer, k, r);
                    }
                }
            }
        }
    }

    fn check_eol(&mut self, layer: usize, k: usize, r: &EolRule) {
        if !self.is_eol_edge(layer, k, r) {
            return;
        }
        if let Some(has_route) = self.qualifies_as_eol(layer, k, r) {
            self.eol_has_eol(layer, k, r, has_route);
        }
    }

    /// Shorter than the rule's width, with a convex corner at each end.
    fn is_eol_edge(&self, layer: usize, k: usize, r: &EolRule) -> bool {
        let segs = &self.segs[layer];
        let e = &segs[k];
        if e.len() >= r.width {
            return false;
        }
        orientation(&segs[e.prev], e) == 1 && orientation(e, &segs[e.next]) == 1
    }

    /// Whether the owner's ROUTE shapes overlap `r` (with an area).
    fn route_overlaps(&self, net: usize, layer: usize, r: &Rect) -> bool {
        self.nets[net].route_slices[layer].iter().any(|s| overlap(s, r).is_some())
    }

    /// `None` unless the edge is a qualifying line end; else whether it carries route shapes.
    fn qualifies_as_eol(&self, layer: usize, k: usize, r: &EolRule) -> Option<bool> {
        if !self.is_eol_edge(layer, k, r) {
            return None;
        }
        let e = self.segs[layer][k];
        let mut has_route = !e.fixed && self.route_overlaps(e.net, layer, &parallel_edge_rect(&e));
        let triggered = match r.parallel {
            None => true,
            Some(p) => {
                let left = self.eol_parallel_edge_one_dir(layer, k, r, &p, true, &mut has_route);
                let right = self.eol_parallel_edge_one_dir(layer, k, r, &p, false, &mut has_route);
                if p.two_edges {
                    left && right
                } else {
                    left || right
                }
            }
        };
        triggered.then_some(has_route)
    }

    /// Every polygon edge on `layer` touching `q`.
    fn query_segs(&self, layer: usize, q: &Rect) -> Vec<usize> {
        (0..self.segs[layer].len())
            .filter(|&i| {
                let s = &self.segs[layer][i];
                touches(&Rect::new(s.from.0, s.from.1, s.to.0, s.to.1), q)
            })
            .collect()
    }

    fn eol_parallel_edge_one_dir(&self, layer: usize, k: usize, r: &EolRule, p: &ParallelEdge, is_low: bool, has_route: &mut bool) -> bool {
        let e = self.segs[layer][k];
        let (pt, par_within, par_space, w) = (if is_low { e.from } else { e.to }, p.within, p.space, r.within);
        let (x, y) = pt;
        let q = match (is_low, e.dir()) {
            (true, EdgeDir::E) => Rect { xl: x - par_space, yl: y - w, xh: x, yh: y + par_within },
            (true, EdgeDir::W) => Rect { xl: x, yl: y - par_within, xh: x + par_space, yh: y + w },
            (true, EdgeDir::N) => Rect { xl: x - par_within, yl: y - par_space, xh: x + w, yh: y },
            (true, EdgeDir::S) => Rect { xl: x - w, yl: y, xh: x + par_within, yh: y + par_space },
            (false, EdgeDir::E) => Rect { xl: x, yl: y - w, xh: x + par_space, yh: y + par_within },
            (false, EdgeDir::W) => Rect { xl: x - par_space, yl: y - par_within, xh: x, yh: y + w },
            (false, EdgeDir::N) => Rect { xl: x - par_within, yl: y, xh: x + w, yh: y + par_space },
            (false, EdgeDir::S) => Rect { xl: x - w, yl: y - par_space, xh: x + par_within, yh: y },
        };
        let mut sol = false;
        for i in self.query_segs(layer, &q) {
            let ptr = self.segs[layer][i];
            let turn = if is_low { orientation(&ptr, &e) } else { orientation(&e, &ptr) };
            if turn != -1 {
                continue;
            }
            let trig = parallel_edge_rect(&ptr);
            if overlap(&q, &trig).is_none() {
                continue;
            }
            sol = true;
            if !*has_route && !ptr.fixed {
                if let Some(t) = overlap(&q, &trig) {
                    if self.route_overlaps(ptr.net, layer, &t) {
                        *has_route = true;
                        break;
                    }
                }
            }
        }
        sol
    }

    /// The window beyond a line end: `space` out, `within` past each side.
    fn eol_query_rect(e: &Seg, r: &EolRule) -> Rect {
        let (w, sp) = (r.within, r.space);
        let (lo, hi) = (e.from, e.to);
        match e.dir() {
            EdgeDir::E => Rect { xl: lo.0 - w, yl: lo.1 - sp, xh: hi.0 + w, yh: hi.1 },
            EdgeDir::W => Rect { xl: hi.0 - w, yl: hi.1, xh: lo.0 + w, yh: lo.1 + sp },
            EdgeDir::N => Rect { xl: lo.0, yl: lo.1 - w, xh: hi.0 + sp, yh: hi.1 + w },
            EdgeDir::S => Rect { xl: hi.0 - sp, yl: hi.1 - w, xh: lo.0, yh: lo.1 + w },
        }
    }

    fn eol_has_eol(&mut self, layer: usize, k: usize, r: &EolRule, has_route: bool) {
        let e = self.segs[layer][k];
        let q = Self::eol_query_rect(&e, r);
        for i in self.query_segs(layer, &q) {
            self.eol_has_eol_check(layer, k, i, &q, has_route);
        }
    }

    fn eol_has_eol_check(&mut self, layer: usize, k: usize, i: usize, q: &Rect, mut has_route: bool) {
        let (e, ptr) = (self.segs[layer][k], self.segs[layer][i]);
        if (ptr.net, ptr.pin) == (e.net, e.pin) {
            return;
        }
        if e.fixed && ptr.fixed {
            return;
        }
        let opposite = matches!((e.dir(), ptr.dir()), (EdgeDir::E, EdgeDir::W) | (EdgeDir::W, EdgeDir::E) | (EdgeDir::N, EdgeDir::S) | (EdgeDir::S, EdgeDir::N));
        if !opposite {
            return;
        }
        let trig = parallel_edge_rect(&ptr);
        if overlap(q, &trig).is_none() {
            return;
        }
        if !has_route && !ptr.fixed {
            if let Some(t) = overlap(q, &trig) {
                has_route = self.route_overlaps(ptr.net, layer, &t);
            }
        }
        if !has_route {
            return;
        }
        self.eol_has_eol_helper(layer, &e, &ptr);
    }

    /// The marker between the two edges — unless a shape already fills it.
    fn eol_has_eol_helper(&mut self, layer: usize, e1: &Seg, e2: &Seg) {
        let marker = generalized_intersect(&parallel_edge_rect(e1), &parallel_edge_rect(e2));
        let mut probe = marker;
        if area(&marker) == 0 {
            match e1.dir() {
                EdgeDir::W | EdgeDir::E => {
                    probe.xl -= 1;
                    probe.xh += 1;
                }
                EdgeDir::S | EdgeDir::N => {
                    probe.yl -= 1;
                    probe.yh += 1;
                }
            }
        }
        if self.query(layer, &probe).iter().any(|&s| overlap(&probe, &self.shapes[layer][s].rect).is_some()) {
            return;
        }
        self.add_marker(Rule::EolSpacing, layer, marker, e1.net, e2.net);
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
                if !self.checks_from(net) {
                    continue;
                }
                let mine: Vec<usize> = (0..self.shapes[layer].len()).filter(|&k| self.alive[layer][k] && self.shapes[layer][k].net == net).collect();
                for k in mine {
                    let s = self.shapes[layer][k];
                    self.metal_spacing_of(layer, s, false);
                }
                // The owner's special spacing rectangles (on any layer; markers are kept once).
                if self.check_ndrs {
                    for sl in 0..self.spc.len() {
                        let mine: Vec<Shape> = (0..self.spc[sl].len()).filter(|&k| self.spc_listed[sl][k] && self.spc[sl][k].net == net).map(|k| self.spc[sl][k]).collect();
                        for s in mine {
                            self.metal_spacing_of(sl, s, true);
                        }
                    }
                }
            }
        }
    }

    fn metal_spacing_of(&mut self, layer: usize, s: Shape, is_spc: bool) {
        let z = (layer / 2).saturating_sub(1);
        let max_spc = self.tech.layers[layer].spacing.as_ref().map_or(0, |t| {
            let m = t.find_max();
            if self.check_ndrs {
                m.max(self.max_ndr_spacing.get(z).copied().unwrap_or(0))
            } else {
                m
            }
        });
        let q = bloat(&s.rect, max_spc);
        if self.check_ndrs {
            let others: Vec<Shape> = self.spc_rq[layer].query(&q).into_iter().map(|(_, v)| self.spc[layer][v.1]).collect();
            for o in others {
                self.metal_spacing_pair(layer, s, o, is_spc, true);
            }
        }
        for o in self.query(layer, &q) {
            let o = self.shapes[layer][o];
            self.metal_spacing_pair(layer, s, o, is_spc, false);
        }
    }

    /// Two shapes: overlapping or touching is a short (or non-sufficient metal within one owner);
    /// apart, the spacing table. (`is_spc`: the first is a special spacing rectangle; `other_spc`:
    /// the second is.)
    fn metal_spacing_pair(&mut self, layer: usize, r1: Shape, r2: Shape, is_spc: bool, other_spc: bool) {
        // The same object: a shape against itself.
        if is_spc == other_spc && r1.rect == r2.rect && r1.net == r2.net && r1.fixed == r2.fixed {
            return;
        }
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
            self.spacing_table(layer, r1, r2, marker, prl_x.max(prl_y), dist_x, dist_y, !is_spc);
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
    fn spacing_table(&mut self, layer: usize, r1: Shape, r2: Shape, marker: Rect, prl: i32, dist_x: i32, dist_y: i32, check_poly_edge: bool) {
        if r1.fixed && r2.fixed {
            return;
        }
        let mut req = i64::from(self.required_spacing(layer, &r1, &r2, prl));
        if self.check_ndrs {
            let z = (layer / 2).saturating_sub(1);
            let ndr = |s: &Shape| -> i64 {
                if s.fixed || s.tapered {
                    return 0;
                }
                self.nets[s.net].ndr_spacing.as_ref().and_then(|v| v.get(z)).map_or(0, |&v| i64::from(v))
            };
            req = req.max(ndr(&r1)).max(ndr(&r2));
        }
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
        if check_poly_edge {
            if !self.has_poly_edges(layer, &r1, &r2, &marker, kind, prl) {
                return;
            }
            if !self.has_route(layer, &r1, &marker) && !self.has_route(layer, &r2, &marker) {
                return;
            }
        } else if !self.spc_marker_outside_net(layer, r1.net, &marker) {
            return;
        }
        self.add_marker_of(Rule::MetalSpacing, layer, marker, (r1.net, r1.rect, r1.fixed), (r2.net, r2.rect, r2.fixed));
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

    /// A special spacing rectangle's marker stands when some of it lies outside its owner's route
    /// shapes; a marker with no extent one way is widened by 1 each side there, and stands unless
    /// what remains is one rectangle with a side on the marker's own line.
    fn spc_marker_outside_net(&self, layer: usize, net: usize, m: &Rect) -> bool {
        let route = &self.nets[net].route_slices[layer];
        let zero_x = m.dx() == 0;
        let zero = zero_x || m.dy() == 0;
        let mut r = *m;
        if zero {
            if zero_x {
                r.xl -= 1;
                r.xh += 1;
            } else {
                r.yl -= 1;
                r.yh += 1;
            }
        }
        let rest = subtract(&r, route);
        if rest.is_empty() {
            return false;
        }
        if zero && rest.len() == 1 {
            let q = rest[0];
            if zero_x {
                if q.xl == m.xl || q.xh == m.xl {
                    return false;
                }
            } else if q.yl == m.yl || q.yh == m.yl {
                return false;
            }
        }
        true
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
        self.add_marker_of(rule, layer, marker, (r1.net, r1.rect, r1.fixed), (r2.net, r2.rect, r2.fixed));
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
                if !self.checks_from(net) {
                    continue;
                }
                let mine: Vec<usize> = (0..self.shapes[layer].len()).filter(|&k| self.alive[layer][k] && self.shapes[layer][k].net == net).collect();
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
            self.add_marker_of(Rule::Short, layer, marker, (r1.net, r1.rect, r1.fixed), (r2.net, r2.rect, r2.fixed));
            return;
        }
        let d2 = i64::from(dist_x).pow(2) + i64::from(dist_y).pow(2);
        if d2 >= i64::from(spc).pow(2) || (r1.fixed && r2.fixed) {
            return;
        }
        self.add_marker_of(Rule::CutSpacing, layer, marker, (r1.net, r1.rect, r1.fixed), (r2.net, r2.rect, r2.fixed));
    }
}

/// A rectangle minus a set of rectangles, as the set's slices (the remainder merged, then cut
/// into rectangles along the scan).
/// The set sliced VERTICALLY (`get_rectangles(…, VERTICAL)`): sliced with x and y swapped, each
/// slice swapped back.
fn vertical_slices(set: &mut Polygon90Set) -> Vec<Rect> {
    let mut t = Polygon90Set::new();
    for r in set.rectangles() {
        t.insert_rect(Rect { xl: r.yl, yl: r.xl, xh: r.yh, yh: r.xh });
    }
    t.rectangles().into_iter().map(|r| Rect { xl: r.yl, yl: r.xl, xh: r.yh, yh: r.xh }).collect()
}

fn subtract(r: &Rect, minus: &[Rect]) -> Vec<Rect> {
    let mut xs: Vec<i32> = vec![r.xl, r.xh];
    let mut ys: Vec<i32> = vec![r.yl, r.yh];
    for m in minus {
        for x in [m.xl, m.xh] {
            if x > r.xl && x < r.xh {
                xs.push(x);
            }
        }
        for y in [m.yl, m.yh] {
            if y > r.yl && y < r.yh {
                ys.push(y);
            }
        }
    }
    xs.sort_unstable();
    xs.dedup();
    ys.sort_unstable();
    ys.dedup();
    let mut set = Polygon90Set::new();
    for wx in xs.windows(2) {
        for wy in ys.windows(2) {
            let cell = Rect::new(wx[0], wy[0], wx[1], wy[1]);
            let covered = minus.iter().any(|m| m.xl <= cell.xl && m.xh >= cell.xh && m.yl <= cell.yl && m.yh >= cell.yh);
            if !covered {
                set.insert_rect(cell);
            }
        }
    }
    set.rectangles()
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    fn marker_at(yl: i32, yh: i32, xh: i32) -> Marker {
        let o = Owner::Net("a".into());
        let r = Rect { xl: 0, yl, xh, yh };
        Marker { rule: Rule::Short, layer: 4, bbox: r, owners: vec![o.clone()], victim: Some((o.clone(), 4, r, false)), aggressor: Some((o, 4, r, false)) }
    }

    /// Rule: a run of consecutive markers alike in layer, rule, x extent and owners is re-sorted by
    /// bottom ascending, then top descending; a marker breaking the run starts a new one (the run
    /// is consecutive, never merged across it).
    #[test]
    fn marker_runs_sort_by_bottom_then_top() {
        let mut m = vec![marker_at(50, 60, 10), marker_at(20, 30, 10), marker_at(20, 40, 10), marker_at(0, 5, 99), marker_at(10, 15, 10)];
        normalize_marker_order(&mut m);
        let got: Vec<(i32, i32, i32)> = m.iter().map(|x| (x.bbox.yl, x.bbox.yh, x.bbox.xh)).collect();
        assert_eq!(got, vec![(20, 40, 10), (20, 30, 10), (50, 60, 10), (0, 5, 99), (10, 15, 10)]);
    }
    use crate::tech::{Dir, Layer, SpacingTable};

    /// li1-like routing layer 2 (width 170, spacing 170 at any width), a cut layer 3 (spacing
    /// 190), met1-like routing layer 4 (width 140, pitch 370; 140, or 280 above width 3000).
    pub(crate) fn tech() -> Tech {
        let table = |rows: Vec<(i32, i32)>| SpacingTable { widths: rows.iter().map(|r| r.0).collect(), prls: vec![0], values: rows.iter().map(|r| vec![r.1]).collect() };
        Tech {
            layers: vec![
                Layer::default(),
                Layer::default(),
                Layer { name: "l2".into(), kind: LayerKind::Routing, dir: Dir::Vertical, width: 170, min_width: 170, pitch: 480, wrong_way_width: 170, spacing: Some(table(vec![(0, 170)])), cut_spacing: None, eol: vec![], min_area: 0, rect_only: false },
                Layer { name: "c3".into(), kind: LayerKind::Cut, width: 170, cut_spacing: Some(190), ..Layer::default() },
                Layer { name: "l4".into(), kind: LayerKind::Routing, dir: Dir::Horizontal, width: 140, min_width: 140, pitch: 370, wrong_way_width: 140, spacing: Some(table(vec![(0, 140), (3000, 280)])), cut_spacing: None, eol: vec![], min_area: 0, rect_only: false },
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

    fn markers_rect_only(shapes: &[(Owner, usize, Rect, bool)]) -> Vec<Marker> {
        let mut t = tech();
        t.layers[4].rect_only = true;
        let mut w = Worker::new(&t);
        for (o, l, r, f) in shapes {
            w.add(o, *l, *r, *f);
        }
        w.init();
        w.run().to_vec()
    }

    fn boxes(ms: &[Marker], rule: Rule) -> Vec<Rect> {
        let mut v: Vec<Rect> = ms.iter().filter(|m| m.rule == rule).map(|m| m.bbox).collect();
        v.sort();
        v
    }

    /// Rule: on a rect-only layer a pin (one owner's merged polygon) that is not one rectangle is
    /// marked at each CONCAVE corner: the polygon within the layer's minimum width (140) of the
    /// corner each way, one marker per maximal rectangle of that piece. A fixed pin with a trial
    /// arm sticking out east has concave corners (1000, 100) and (1000, 300).
    #[test]
    fn rect_only_marks_the_polygon_around_each_concave_corner() {
        let m = markers_rect_only(&[(net("a"), 4, Rect::new(0, 0, 1000, 400), true), (net("a"), 4, Rect::new(900, 100, 1500, 300), false)]);
        let want = vec![Rect::new(860, 0, 1000, 240), Rect::new(860, 100, 1140, 240), Rect::new(860, 160, 1000, 400), Rect::new(860, 160, 1140, 300)];
        let mut w = want.clone();
        w.sort();
        assert_eq!(boxes(&m, Rule::RectOnly), w);
        assert_eq!(m.len(), 4);
        // Not rect-only: nothing.
        assert!(markers(&[(net("a"), 4, Rect::new(0, 0, 1000, 400), true), (net("a"), 4, Rect::new(900, 100, 1500, 300), false)]).is_empty());
    }

    /// Rule: a concave corner whose surroundings are ALL the owner's fixed shapes is not marked —
    /// a fixed L with a trial inside it stands.
    #[test]
    fn rect_only_skips_corners_the_fixed_shapes_cover() {
        let m = markers_rect_only(&[(net("a"), 4, Rect::new(0, 0, 1000, 400), true), (net("a"), 4, Rect::new(1000, 100, 1500, 300), true), (net("a"), 4, Rect::new(100, 100, 300, 300), false)]);
        assert!(m.is_empty(), "{m:?}");
    }

    /// `markers`, on `tech()` with layer 4's minimum area set, a check box, and the ignore flag.
    fn markers_min_area(shapes: &[(Owner, usize, Rect, bool)], min_area: i64, drc: Option<Rect>, ignore: bool) -> Vec<Marker> {
        let mut t = tech();
        t.layers[4].min_area = min_area;
        let mut w = Worker::new(&t);
        w.drc_box = drc;
        w.ignore_min_area = ignore;
        for (o, l, r, f) in shapes {
            w.add(o, *l, *r, *f);
        }
        w.init();
        w.run().to_vec()
    }

    /// Rule (`checkMetalShape_minArea`, the marker pass): a polygon below the layer's minimum area
    /// is a marker on its bounding box — here a 300 × 200 trial stub (60,000 < 100,000). None when
    /// its box is not WHOLLY inside the check box (`drcBox_.contains`, a box it merely overlaps is
    /// not enough), when one of its edges is on the owner's fixed shapes, when the layer has no
    /// area rule, or when the check is ignored (pin access: `setIgnoreMinArea`).
    #[test]
    fn min_area_marks_a_small_polygon_wholly_inside_the_check_box() {
        let stub = [(net("a"), 4, Rect::new(0, 0, 300, 200), false)];
        let m = markers_min_area(&stub, 100_000, Some(Rect::new(-1000, -1000, 1000, 1000)), false);
        assert_eq!(boxes(&m, Rule::MinArea), vec![Rect::new(0, 0, 300, 200)]);
        // The check box ends inside the stub: no marker (overlap is not containment).
        assert!(boxes(&markers_min_area(&stub, 100_000, Some(Rect::new(-1000, -1000, 150, 1000)), false), Rule::MinArea).is_empty());
        // No check box: the whole design.
        assert_eq!(boxes(&markers_min_area(&stub, 100_000, None, false), Rule::MinArea).len(), 1);
        // At the minimum: none.
        assert!(boxes(&markers_min_area(&stub, 60_000, None, false), Rule::MinArea).is_empty());
        // No area rule, or ignored: none.
        assert!(boxes(&markers_min_area(&stub, 0, None, false), Rule::MinArea).is_empty());
        assert!(boxes(&markers_min_area(&stub, 100_000, None, true), Rule::MinArea).is_empty());
        // Part of the polygon is the owner's fixed shape (a fixed edge): none.
        let fixed = [(net("a"), 4, Rect::new(0, 0, 150, 200), true), (net("a"), 4, Rect::new(150, 0, 300, 200), false)];
        assert!(boxes(&markers_min_area(&fixed, 100_000, None, false), Rule::MinArea).is_empty());
    }

    /// Rule: minimum width is judged on the polygon's slices — sliced horizontally, each slice's
    /// x length; sliced vertically, each slice's y length. A trial wire 100 tall (min 140) is one
    /// marker, from the vertical slicing; a staircase whose horizontal slices include a 50-tall
    /// sliver (0..1200 × 350..400, its neighbours of other x extents so it stays a slice of its
    /// own) is none.
    #[test]
    fn min_width_is_judged_along_each_slicing() {
        let m = markers(&[(net("a"), 4, Rect::new(0, 0, 1000, 100), false)]);
        assert_eq!(boxes(&m, Rule::MinWidth), vec![Rect::new(0, 0, 1000, 100)]);
        assert_eq!(m.len(), 1);
        let m = markers(&[(net("a"), 4, Rect::new(0, 0, 1000, 400), false), (net("a"), 4, Rect::new(200, 350, 1200, 700), false)]);
        assert!(m.is_empty(), "{m:?}");
    }

    /// Rule: a narrow slice the owner's fixed shapes cover whole is not marked (a pin narrower
    /// than the minimum width stands), though its owner has a trial elsewhere.
    #[test]
    fn min_width_skips_slices_the_fixed_shapes_cover() {
        let m = markers(&[(net("a"), 4, Rect::new(0, 0, 1000, 100), true), (net("a"), 4, Rect::new(5000, 0, 6000, 400), false)]);
        assert!(m.is_empty(), "{m:?}");
    }

    /// Rule: special spacing rectangles live in a per-layer tree the reference keeps BY VALUE —
    /// bulk-loaded at init, an owner's taken out (the first equal rectangle in tree order, the
    /// leaf's last entry moved into its place) and put back when its route is replaced. So the
    /// order a check meets them in is the tree's, not the owners' — and with equal spacing the
    /// first met is the marker's aggressor.
    #[test]
    fn special_spacing_rects_are_met_in_tree_order() {
        let t = tech();
        let mut w = Worker::new(&t);
        let rect = |k: i32| Rect::new(k * 1000, 0, k * 1000 + 500, 140);
        for k in 0..4 {
            let o = net(&format!("n{k}"));
            w.add(&o, 4, rect(k), false);
            w.add_taper(&o, 4, rect(k), true);
            w.add_taper(&o, 4, rect(k), false);
        }
        w.init();
        w.replace_route(&net("n1"), &[(4, rect(1))], &[(4, rect(1), true), (4, rect(1), false)]);
        let met: Vec<i32> = w.spc_rq[4].query(&Rect::new(-10000, -10000, 10000, 10000)).into_iter().map(|(_, v)| w.spc[4][v.1].rect.xl / 1000).collect();
        assert_eq!(met, vec![0, 3, 2, 1]);
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

#[cfg(test)]
mod eol_tests {
    //! End-of-line spacing on constructed geometry: layer 4 (horizontal, width 140, spacing 140)
    //! with an end-of-line rule — space 200, width 150, within 30 — unless a test says otherwise.
    //! The base case: net `a`'s wire [0, 0]–[1000, 140] ends facing net `b`'s block 150 away.
    use super::*;
    use crate::tech::{EolRule, ParallelEdge};

    fn tech_with(rule: EolRule) -> Tech {
        let mut t = tests::tech();
        t.layers[4].eol = vec![rule];
        t
    }

    fn rule() -> EolRule {
        EolRule { space: 200, width: 150, within: 30, parallel: None }
    }

    fn net(n: &str) -> Owner {
        Owner::Net(n.into())
    }

    /// The end-of-line markers' boxes.
    fn eol(t: &Tech, shapes: &[(&str, Rect, bool)], ignore_long_side: bool) -> Vec<Rect> {
        let mut w = Worker::new(t);
        w.ignore_long_side_eol = ignore_long_side;
        for (o, r, f) in shapes {
            w.add(&net(o), 4, *r, *f);
        }
        w.init();
        w.run().iter().filter(|m| m.rule == Rule::EolSpacing).map(|m| m.bbox).collect()
    }

    const WIRE: Rect = Rect { xl: 0, yl: 0, xh: 1000, yh: 140 };
    const BLOCK: Rect = Rect { xl: 1150, yl: -500, xh: 1500, yh: 500 };
    const GAP: Rect = Rect { xl: 1000, yl: 0, xh: 1150, yh: 140 };

    /// The base case: one marker, spanning the gap between the line end and the facing edge.
    #[test]
    fn a_line_end_facing_an_edge_within_space() {
        assert_eq!(eol(&tech_with(rule()), &[("a", WIRE, false), ("b", BLOCK, true)], false), vec![GAP]);
    }

    /// A line end exactly as wide as the rule's width is not a line end.
    #[test]
    fn an_end_as_wide_as_the_rule_is_not_an_end() {
        let t = tech_with(EolRule { width: 140, ..rule() });
        assert!(eol(&t, &[("a", WIRE, false), ("b", BLOCK, true)], false).is_empty());
    }

    /// An end with a concave corner at either end is not a line end: here the short edge at x 1000
    /// (y 0–40) turns right into a tab; only the tab's own end (x 1050) is one.
    #[test]
    fn a_concave_corner_disqualifies_an_end() {
        let t = tech_with(rule());
        let tab = Rect::new(1000, 40, 1050, 140);
        let got = eol(&t, &[("a", WIRE, false), ("a", tab, false), ("b", Rect::new(1190, -500, 1500, 500), true)], false);
        assert!(!got.is_empty() && got.iter().all(|r| r.xl == 1050), "{got:?}");
    }

    /// An end facing an edge of its OWN polygon is not checked.
    #[test]
    fn an_end_facing_its_own_polygon_is_not_checked() {
        let t = tech_with(rule());
        let own = [Rect::new(0, -800, 140, 140), Rect::new(0, -800, 1300, -500), Rect::new(1150, -800, 1300, 500), WIRE];
        let shapes: Vec<(&str, Rect, bool)> = own.iter().map(|&r| ("a", r, false)).collect();
        assert!(eol(&t, &shapes, false).is_empty());
    }

    /// Two fixed edges are never checked against each other.
    #[test]
    fn fixed_against_fixed_is_not_checked() {
        assert!(eol(&tech_with(rule()), &[("a", WIRE, true), ("b", BLOCK, true)], false).is_empty());
    }

    /// A gap already (partly) filled by a shape is not marked.
    #[test]
    fn a_filled_gap_is_not_marked() {
        let got = eol(&tech_with(rule()), &[("a", WIRE, false), ("b", BLOCK, true), ("c", Rect::new(1050, -50, 1100, 190), true)], false);
        assert!(!got.contains(&GAP), "{got:?}");
    }

    /// Where the two edges only meet at a corner line the marker has no area; the probe for a
    /// filling shape is widened by one across it — so a shape straddling that line fills it.
    #[test]
    fn a_zero_area_gap_is_probed_one_unit_wide() {
        let t = tech_with(rule());
        let block = Rect::new(1150, 140, 1500, 600);
        let open = eol(&t, &[("a", WIRE, false), ("b", block, true)], false);
        let line = Rect { xl: 1000, yl: 140, xh: 1150, yh: 140 };
        assert!(open.contains(&line), "{open:?}");
        let filled = eol(&t, &[("a", WIRE, false), ("b", block, true), ("c", Rect::new(1050, 130, 1100, 170), true)], false);
        assert!(!filled.contains(&line), "{filled:?}");
    }

    /// With `ignore_long_side_eol`, above the first metal layer, edges running ALONG the layer
    /// (here the top of a vertical stub on a horizontal layer) are not checked; without it they are.
    #[test]
    fn long_sides_are_skipped_only_when_asked() {
        let t = tech_with(rule());
        let shapes = [("a", Rect::new(0, 0, 140, 1000), false), ("b", Rect::new(-500, 1150, 500, 1500), true)];
        assert!(!eol(&t, &shapes, false).is_empty());
        assert!(eol(&t, &shapes, true).is_empty());
    }

    /// A planar trial checks long sides: a wrong-way segment's end on a horizontal layer is one.
    #[test]
    fn a_planar_trial_checks_long_sides() {
        let t = tech_with(rule());
        let target = vec![(net("b"), 4, Rect::new(-500, 640, 500, 900))];
        let m = crate::pa::verdict::planar_markers(&t, &target, &net("a"), (0, 0), 4, (0, 420));
        assert!(m.iter().any(|m| m.rule == Rule::EolSpacing), "{m:?}");
    }

    fn parallel(two_edges: bool) -> EolRule {
        EolRule { parallel: Some(ParallelEdge { space: 120, within: 100, two_edges }), ..rule() }
    }

    const BELOW: Rect = Rect { xl: 800, yl: -400, xh: 1100, yh: -100 };

    /// With a parallel edge, an end counts only when a parallel edge lies beside it.
    #[test]
    fn a_parallel_edge_rule_needs_a_parallel_edge() {
        assert!(eol(&tech_with(parallel(false)), &[("a", WIRE, false), ("b", BLOCK, true)], false).is_empty());
        assert!(eol(&tech_with(parallel(false)), &[("a", WIRE, false), ("b", BLOCK, true), ("d", BELOW, true)], false).contains(&GAP));
    }

    /// With two edges, a parallel edge is needed on BOTH sides.
    #[test]
    fn two_edges_needs_both_sides() {
        let t = tech_with(parallel(true));
        assert!(!eol(&t, &[("a", WIRE, false), ("b", BLOCK, true), ("d", BELOW, true)], false).contains(&GAP));
        let above = Rect::new(800, 200, 1100, 500);
        assert!(eol(&t, &[("a", WIRE, false), ("b", BLOCK, true), ("d", BELOW, true), ("u", above, true)], false).contains(&GAP));
    }

    /// Where two polygons of one net touch only at a corner, each ring turns LEFT there: the wire's
    /// end keeps its convex corner and is checked.
    #[test]
    fn a_corner_pinch_keeps_each_polygon_convex() {
        let t = tech_with(rule());
        let got = eol(&t, &[("a", WIRE, false), ("a", Rect::new(1000, 140, 1100, 300), false), ("b", BLOCK, true)], false);
        assert!(got.contains(&GAP), "{got:?}");
    }

    /// A fixed edge carries no route of its own even where a route shape covers it: facing an edge
    /// whose inside is fixed, it makes no marker.
    #[test]
    fn a_fixed_end_does_not_carry_route() {
        let t = tech_with(rule());
        let b = [("b", BLOCK, true), ("b", Rect::new(1150, 500, 1500, 600), false)];
        let got = eol(&t, &[("a", WIRE, true), ("a", WIRE, false), b[0], b[1]], false);
        assert!(!got.contains(&GAP), "{got:?}");
    }

    /// Without route shapes at either end the pair is not checked.
    #[test]
    fn no_route_no_marker() {
        let t = tech_with(rule());
        let got = eol(&t, &[("a", WIRE, true), ("b", BLOCK, true), ("b", Rect::new(1150, 500, 1500, 600), false)], false);
        assert!(!got.contains(&GAP), "{got:?}");
    }

    /// A fixed RECTANGLE's edges count as fixed even when the owner's merged fixed shapes do not
    /// have that edge: fixed against fixed, no marker, though a route shape reaches the edge.
    #[test]
    fn a_fixed_rectangle_edge_is_fixed() {
        let t = tech_with(rule());
        let shapes = [
            ("a", Rect::new(0, 0, 1000, 140), true),
            ("a", Rect::new(500, 0, 1000, 300), true),
            ("a", Rect::new(1000, 140, 1300, 300), false),
            ("a", Rect::new(900, 0, 1000, 60), false),
            ("b", Rect::new(1150, 40, 2000, 100), true),
        ];
        let got = eol(&t, &shapes, false);
        assert!(!got.iter().any(|r| r.xl == 1000 && r.xh == 1150), "{got:?}");
    }
}
