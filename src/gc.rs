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
use crate::tech::{CornerSpacing, CutSpacingTable, EolKeepOut, EolRule, LayerKind, ParallelEdge, Tech};

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
    /// A hole in one owner's polygon smaller than a MINENCLOSEDAREA rule
    /// (`checkMetalShape_minEnclosedArea`).
    MinEnclosedArea,
    /// Other metal inside a LEF58 end-of-line keep-out box (`checkMetalEOLkeepout_main`).
    Lef58EolKeepOut,
    /// A line end closer than a LEF58 end-of-line spacing rule allows (`checkMetalEndOfLine_eol`
    /// with an `frLef58SpacingEndOfLineConstraint`).
    Lef58SpacingEndOfLine,
    /// A convex corner closer to another owner's corner than a LEF58 corner spacing rule allows
    /// (`checkMetalCornerSpacing`).
    CornerSpacing,
    /// Two cuts closer than a LEF58 cut spacing table allows (`checkLef58CutSpacingTbl`).
    Lef58CutSpacingTable,
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

/// A patch the check's surgical fix makes (`drPatchWire`): its layer, origin, box relative to the
/// origin, and owner.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PatchWire {
    pub layer: usize,
    pub origin: (i32, i32),
    pub offset: Rect,
    pub owner: Owner,
}

impl PatchWire {
    pub fn bbox(&self) -> Rect {
        Rect::new(self.offset.xl + self.origin.0, self.offset.yl + self.origin.1, self.offset.xh + self.origin.0, self.offset.yh + self.origin.1)
    }
}

/// `modifyMarkers`: per patch, per marker on its layer whose box touches the patch's, sourced by
/// the patch's owner and not holding its origin: the box grows to take the origin in.
pub fn modify_markers(markers: &mut [Marker], patches: &[PatchWire]) {
    for p in patches {
        let (b, o) = (p.bbox(), p.origin);
        for m in markers.iter_mut() {
            if m.layer != p.layer || !(m.bbox.xl <= b.xh && b.xl <= m.bbox.xh && m.bbox.yl <= b.yh && b.yl <= m.bbox.yh) || !m.owners.contains(&p.owner) {
                continue;
            }
            if m.bbox.xl <= o.0 && o.0 <= m.bbox.xh && m.bbox.yl <= o.1 && o.1 <= m.bbox.yh {
                continue;
            }
            m.bbox = Rect::new(m.bbox.xl.min(o.0), m.bbox.yl.min(o.1), m.bbox.xh.max(o.0), m.bbox.yh.max(o.1));
        }
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

/// Drop net `net`'s edges and renumber the survivors' `prev` / `next`. They are absolute indices
/// into the list, so a plain `retain` left every edge after a removed one pointing at the wrong
/// neighbours — and the end-of-line checks, which read them, judged the wrong corners on every
/// net updated after an earlier one. A survivor's links stay within its own polygon, so they
/// always survive too.
fn retain_segs(segs: &mut Vec<Seg>, net: usize) {
    let mut new_idx = vec![usize::MAX; segs.len()];
    let mut n = 0;
    for (k, e) in segs.iter().enumerate() {
        if e.net != net {
            new_idx[k] = n;
            n += 1;
        }
    }
    segs.retain(|e| e.net != net);
    for e in segs.iter_mut() {
        e.prev = new_idx[e.prev];
        e.next = new_idx[e.next];
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
    /// Skip the corner spacing check (`setIgnoreCornerSpacing`): pin access sets it too.
    pub ignore_corner_spacing: bool,
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
/// `gtl::intersects(a, b, false)`: both extents overlap with a length (touching does not count).
fn strictly_intersects(a: &Rect, b: &Rect) -> bool {
    a.xl < b.xh && b.xl < a.xh && a.yl < b.yh && b.yl < a.yh
}

/// `getEolKeepOutQueryBox`, by the line end's direction (polygon edges run counter-clockwise).
fn eol_keepout_box(e: &Seg, ko: &EolKeepOut) -> Rect {
    let (lo, hi) = (e.from, e.to);
    let (f, b, s) = (ko.forward, ko.backward, ko.side);
    match e.dir() {
        EdgeDir::S => Rect { xl: hi.0 - f, yl: hi.1 - s, xh: lo.0 + b, yh: lo.1 + s },
        EdgeDir::N => Rect { xl: lo.0 - b, yl: lo.1 - s, xh: hi.0 + f, yh: hi.1 + s },
        EdgeDir::E => Rect { xl: lo.0 - s, yl: lo.1 - f, xh: hi.0 + s, yh: hi.1 + b },
        EdgeDir::W => Rect { xl: hi.0 - s, yl: hi.1 - b, xh: lo.0 + s, yh: lo.1 + f },
    }
}

/// `getEolKeepOutExceptWithinRects`: the two side windows `within_low..within_high` off the line
/// end's two corners.
fn eol_keepout_except_rects(e: &Seg, ko: &EolKeepOut) -> (Rect, Rect) {
    let (lo, hi) = (e.from, e.to);
    let (f, b, wl, wh) = (ko.forward, ko.backward, ko.within_low, ko.within_high);
    let r = |xl: i32, yl: i32, xh: i32, yh: i32| Rect::new(xl.min(xh), yl.min(yh), xl.max(xh), yl.max(yh));
    match e.dir() {
        EdgeDir::S => (r(lo.0 - f, lo.1 + wl, lo.0 + b, lo.1 + wh), r(hi.0 - f, hi.1 - wh, hi.0 + b, hi.1 - wl)),
        EdgeDir::N => (r(lo.0 - b, lo.1 - wh, lo.0 + f, lo.1 - wl), r(hi.0 - b, hi.1 + wl, hi.0 + f, hi.1 + wh)),
        EdgeDir::E => (r(lo.0 - wh, lo.1 - f, lo.0 - wl, lo.1 + b), r(hi.0 + wl, hi.1 - f, hi.0 + wh, hi.1 + b)),
        EdgeDir::W => (r(lo.0 + wl, lo.1 - b, lo.0 + wh, lo.1 + f), r(hi.0 - wh, hi.1 - b, hi.0 - wl, hi.1 + f)),
    }
}

fn generalized_intersect(a: &Rect, b: &Rect) -> Rect {
    let axis = |al: i32, ah: i32, bl: i32, bh: i32| {
        let (lo, hi) = (al.max(bl), ah.min(bh));
        (lo.min(hi), lo.max(hi))
    };
    let (xl, xh) = axis(a.xl, a.xh, b.xl, b.xh);
    let (yl, yh) = axis(a.yl, a.yh, b.yl, b.yh);
    Rect { xl, yl, xh, yh }
}

/// A polygon corner (`gcCorner`): the vertex before an edge.
#[derive(Debug, Clone, Copy)]
struct Corner {
    x: i32,
    y: i32,
    convex: bool,
    dir: CornerDir,
    net: usize,
    fixed: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CornerDir {
    NE,
    SE,
    SW,
    NW,
}

/// `isCornerOverlap`: the corner is the rectangle's own corner on the corner's side.
fn corner_overlaps(c: &Corner, r: &Rect) -> bool {
    match c.dir {
        CornerDir::NE => (c.x, c.y) == (r.xh, r.yh),
        CornerDir::SE => (c.x, c.y) == (r.xh, r.yl),
        CornerDir::SW => (c.x, c.y) == (r.xl, r.yl),
        CornerDir::NW => (c.x, c.y) == (r.xl, r.yh),
    }
}

/// `isPolygonCorner`: `p` is a vertex of the region the disjoint `slices` cover — its four unit
/// quadrants neither all alike nor split into two halves side by side.
fn is_polygon_vertex(slices: &[Rect], (x, y): (i32, i32)) -> bool {
    let has = |xl: bool, yl: bool| {
        slices.iter().any(|s| {
            let in_x = if xl { s.xl < x && x <= s.xh } else { s.xl <= x && x < s.xh };
            let in_y = if yl { s.yl < y && y <= s.yh } else { s.yl <= y && y < s.yh };
            in_x && in_y
        })
    };
    let (sw, se, nw, ne) = (has(true, true), has(false, true), has(true, false), has(false, false));
    let n = [sw, se, nw, ne].iter().filter(|&&b| b).count();
    match n {
        1 | 3 => true,
        2 => sw == ne,
        _ => false,
    }
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
        let mut w = Worker { tech, nets: Vec::new(), index: HashMap::new(), shapes: Vec::new(), edges: Vec::new(), segs: Vec::new(), ignore_long_side_eol: false, target: None, check_ndrs: false, ignore_min_area: false, ignore_corner_spacing: false, drc_box: None, max_ndr_spacing: Vec::new(), spc: Vec::new(), spc_listed: Vec::new(), spc_rq: Vec::new(), alive: Vec::new(), rq_id: Vec::new(), rq: Vec::new(), markers: Vec::new(), seen: BTreeSet::new() };
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
        self.replace_routes(&[(owner.clone(), route.to_vec(), taper.to_vec())]);
    }

    /// Several owners' route shapes replaced at once (`updateGCWorker`): every one's rectangles
    /// out of the trees first, in the order given; then each rebuilt and re-inserted in turn.
    #[allow(clippy::type_complexity)]
    pub fn replace_routes(&mut self, batch: &[(Owner, Vec<(usize, Rect)>, Vec<(usize, Rect, bool)>)]) {
        let ids: Vec<usize> = batch.iter().map(|(o, _, _)| self.net(o)).collect();
        for &i in &ids {
            self.remove_route(i);
        }
        for (&i, (owner, route, taper)) in ids.iter().zip(batch) {
            self.insert_route(i, owner, route, taper);
        }
    }

    /// An owner's route rectangles out of the trees (layer, pin, rectangle order).
    fn remove_route(&mut self, i: usize) {
        let n = self.tech.layers.len();
        for layer in 0..n {
            for k in 0..self.shapes[layer].len() {
                if self.alive[layer][k] && self.shapes[layer][k].net == i {
                    self.rq[layer].remove(self.rq_id[layer][k]);
                    self.alive[layer][k] = false;
                }
            }
            self.edges[layer].retain(|e| e.net != i);
            retain_segs(&mut self.segs[layer], i);
            for k in 0..self.spc[layer].len() {
                if self.spc_listed[layer][k] && self.spc[layer][k].net == i {
                    self.spc_rq[layer].remove_eq(&self.spc[layer][k].rect);
                    self.spc_listed[layer][k] = false;
                }
            }
        }
    }

    /// An owner's pins rebuilt from its fixed shapes and these route shapes, the new rectangles
    /// into the trees.
    fn insert_route(&mut self, i: usize, owner: &Owner, route: &[(usize, Rect)], taper: &[(usize, Rect, bool)]) {
        let n = self.tech.layers.len();
        let old: Vec<usize> = self.shapes.iter().map(|v| v.len()).collect();
        let old_spc: Vec<usize> = self.spc.iter().map(|v| v.len()).collect();
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
        self.run_patched(&[])
    }

    /// `run`, with `modifyMarkers` for the surgical fix's patches before the markers' final order:
    /// a marker on a patch's layer, touching its box, sourced by its owner and not holding its
    /// origin grows to take the origin in.
    pub fn run_patched(&mut self, patches: &[PatchWire]) -> &[Marker] {
        self.markers.clear();
        self.seen.clear();
        self.check_metal_corner_spacing();
        self.check_metal_spacing();
        self.check_metal_shape();
        self.check_metal_end_of_line();
        self.check_cut_spacing();
        modify_markers(&mut self.markers, patches);
        normalize_marker_order(&mut self.markers);
        &self.markers
    }

    /// `checkMetalCornerSpacing` alone, as the surgical fix's patch pass runs it: its markers in
    /// the order made (the pass then clears them).
    pub fn corner_markers(&mut self) -> Vec<Marker> {
        self.markers.clear();
        self.seen.clear();
        self.check_metal_corner_spacing();
        let m = std::mem::take(&mut self.markers);
        self.seen.clear();
        m
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
        self.min_enclosed_area(layer, net, pin);
    }

    /// `checkMetalShape_minEnclosedArea`: per hole of the polygon, per MINENCLOSEDAREA rule, a hole
    /// smaller than the rule is a marker on the hole's extents — when the polygon holds any of the
    /// owner's ROUTE shapes (`net->getPolygons(layer, false) & poly` not empty: a hole wholly among
    /// fixed shapes is not the router's to fix). No ignore flag: pin access checks it too.
    fn min_enclosed_area(&mut self, layer: usize, net: usize, pin: &mut Polygon90Set) {
        if self.tech.layers[layer].min_enclosed_areas.is_empty() {
            return;
        }
        let holes = pin.holes();
        if holes.is_empty() {
            return;
        }
        let slices = pin.rectangles();
        let routed = self.nets[net].route_slices[layer].iter().any(|r| slices.iter().any(|p| overlap(r, p).is_some()));
        if !routed {
            return;
        }
        let rules = self.tech.layers[layer].min_enclosed_areas.clone();
        for (area, bbox) in holes {
            for &req in &rules {
                if area < i64::from(req) {
                    self.add_marker(Rule::MinEnclosedArea, layer, bbox, net, net);
                }
            }
        }
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
            if l.kind != LayerKind::Routing || (l.eol.is_empty() && l.lef58_eol.is_empty() && l.eol_keepout.is_empty()) {
                continue;
            }
            let vertical = l.is_vertical();
            let rules = l.eol.clone();
            let lef58 = l.lef58_eol.clone();
            let keepouts = l.eol_keepout.clone();
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
                    // checkMetalEndOfLine_main: the EOL rules, the LEF58 EOL spacing rules, then the
                    // keep-out rules, per edge.
                    for r in &rules {
                        self.check_eol(layer, k, r, Rule::EolSpacing);
                    }
                    for r in &lef58 {
                        self.check_eol(layer, k, r, Rule::Lef58SpacingEndOfLine);
                    }
                    for ko in &keepouts {
                        self.check_eol_keepout(layer, k, ko);
                    }
                }
            }
        }
    }

    /// `checkMetalEndOfLine_eol`; `rule` is the marker's (`EolSpacing` or `Lef58SpacingEndOfLine`,
    /// which also gates each pair on `endToEndHelper`).
    fn check_eol(&mut self, layer: usize, k: usize, r: &EolRule, rule: Rule) {
        if self.target.is_some() {
            self.eol_tn(layer, k, r, rule);
        }
        if !self.is_eol_edge(layer, k, r) {
            return;
        }
        if let Some(has_route) = self.qualifies_as_eol(layer, k, r) {
            self.eol_has_eol(layer, k, r, has_route, rule);
        }
    }

    /// `checkMetalEndOfLine_eol_TN` (a target's check only, before the edge's own): every other
    /// edge in the target edge's own query window that qualifies as a line end is checked AGAINST
    /// the target edge — so a marker whose line end is another owner's is made first, with that
    /// owner the victim (the target edge's own check then finds it made).
    fn eol_tn(&mut self, layer: usize, k: usize, r: &EolRule, rule: Rule) {
        let q = Self::eol_query_rect(&self.segs[layer][k], r);
        for i in self.query_segs(layer, &q) {
            if let Some(has_route) = self.qualifies_as_eol(layer, i, r) {
                let q2 = Self::eol_query_rect(&self.segs[layer][i], r);
                self.eol_has_eol_check(layer, (i, k), &q2, has_route, (r, rule));
            }
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

    /// The window beyond a line end: `space` out (at least the END-TO-END space), `within` past
    /// each side.
    fn eol_query_rect(e: &Seg, r: &EolRule) -> Rect {
        let (w, sp) = (r.within, r.space.max(r.end_to_end.unwrap_or(0)));
        let (lo, hi) = (e.from, e.to);
        match e.dir() {
            EdgeDir::E => Rect { xl: lo.0 - w, yl: lo.1 - sp, xh: hi.0 + w, yh: hi.1 },
            EdgeDir::W => Rect { xl: hi.0 - w, yl: hi.1, xh: lo.0 + w, yh: lo.1 + sp },
            EdgeDir::N => Rect { xl: lo.0, yl: lo.1 - w, xh: hi.0 + sp, yh: hi.1 + w },
            EdgeDir::S => Rect { xl: hi.0 - sp, yl: hi.1 - w, xh: lo.0, yh: lo.1 + w },
        }
    }

    fn eol_has_eol(&mut self, layer: usize, k: usize, r: &EolRule, has_route: bool, rule: Rule) {
        let e = self.segs[layer][k];
        let q = Self::eol_query_rect(&e, r);
        for i in self.query_segs(layer, &q) {
            self.eol_has_eol_check(layer, (k, i), &q, has_route, (r, rule));
        }
    }

    fn eol_has_eol_check(&mut self, layer: usize, (k, i): (usize, usize), q: &Rect, mut has_route: bool, (r, rule): (&EolRule, Rule)) {
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
        if rule == Rule::Lef58SpacingEndOfLine && !self.eol_end_to_end(layer, (k, i), r) {
            return;
        }
        self.eol_has_eol_helper(layer, &e, &ptr, rule);
    }

    /// `checkMetalEndOfLine_eol_hasEol_endToEndHelper` (LEF58 rules only): the two edges as
    /// rectangles; a facing line end takes the END-TO-END space, anything else the rule's space with
    /// the line end widened by `within` along itself. A violation needs a run (positive projection
    /// on the axis where they overlap) and the larger axis gap below that space; touching (no gap
    /// on either axis) is a short, handled elsewhere.
    fn eol_end_to_end(&self, layer: usize, (k, i): (usize, usize), r: &EolRule) -> bool {
        let (e1, e2) = (self.segs[layer][k], self.segs[layer][i]);
        let rect = |s: &Seg| Rect::new(s.from.0.min(s.to.0), s.from.1.min(s.to.1), s.from.0.max(s.to.0), s.from.1.max(s.to.1));
        let (mut r1, r2) = (rect(&e1), rect(&e2));
        let space = match r.end_to_end {
            Some(ete) if self.is_eol_edge(layer, i, r) => ete,
            _ => {
                if e1.from.1 == e1.to.1 {
                    r1.xl -= r.within;
                    r1.xh += r.within;
                } else {
                    r1.yl -= r.within;
                    r1.yh += r.within;
                }
                r.space
            }
        };
        let m = generalized_intersect(&r1, &r2);
        let (dist_x, dist_y) = ((r2.xl - r1.xh).max(r1.xl - r2.xh).max(0), (r2.yl - r1.yh).max(r1.yl - r2.yh).max(0));
        if dist_x == 0 && dist_y == 0 {
            return false;
        }
        let prl_x = if dist_x != 0 { -(m.xh - m.xl) } else { m.xh - m.xl };
        let prl_y = if dist_y != 0 { -(m.yh - m.yl) } else { m.yh - m.yl };
        prl_x.max(prl_y) > 0 && dist_x.max(dist_y) < space
    }

    // ---- LEF58 end-of-line keep-out ----

    /// `checkMetalEOLkeepout_main`: a line end (shorter than the rule's width, convex at both
    /// ends) keeps other pins' metal out of its keep-out box — `forward` beyond the end, `backward`
    /// behind it, `side` past each side. CORNER ONLY asks the box for polygon EDGES (so concave
    /// corners are caught), each tried as a rectangle; otherwise the maximal rectangles.
    fn check_eol_keepout(&mut self, layer: usize, k: usize, ko: &EolKeepOut) {
        let e = self.segs[layer][k];
        if e.len() >= ko.width || !(orientation(&self.segs[layer][e.prev], &e) == 1 && orientation(&e, &self.segs[layer][e.next]) == 1) {
            return;
        }
        let q = eol_keepout_box(&e, ko);
        if ko.corner_only {
            for i in self.query_segs(layer, &q) {
                let s = self.segs[layer][i];
                let r = Rect::new(s.from.0.min(s.to.0), s.from.1.min(s.to.1), s.from.0.max(s.to.0), s.from.1.max(s.to.1));
                self.eol_keepout_helper(layer, &e, (r, s.net, s.pin, s.fixed), &q, ko);
            }
        } else {
            // `queryMaxRectangle`: every pin's maximal rectangles on the layer — `shapes`, not the
            // tapered-spacing list `spc` (which is empty without a non-default rule). ⚠️ asap7's
            // keep-out rules are all CORNER ONLY, so the corpus never reaches this branch; the unit
            // test does.
            let found: Vec<Shape> = self.query(layer, &q).into_iter().map(|k| self.shapes[layer][k]).collect();
            for sh in found {
                self.eol_keepout_helper(layer, &e, (sh.rect, sh.net, sh.pin as usize, sh.fixed), &q, ko);
            }
        }
    }

    /// `checkMetalEOLkeepout_helper`: `(rect, net, pin, fixed)` against the edge's keep-out box `q`.
    /// Not its own pin, not both fixed, overlapping the box with an area (not merely touching). A
    /// corner-only rule reduces the rectangle to its corner(s) strictly inside the box (none: no
    /// violation); except-within excuses a rectangle meeting either side window. The marker is the
    /// line end generalized-intersected with the box's part of the rectangle.
    fn eol_keepout_helper(&mut self, layer: usize, e: &Seg, (mut r, net, pin, fixed): (Rect, usize, usize, bool), q: &Rect, ko: &EolKeepOut) {
        if (net, pin) == (e.net, e.pin) || (e.fixed && fixed) || !strictly_intersects(q, &r) {
            return;
        }
        if ko.corner_only {
            let inside = |x: i32, y: i32| q.xl < x && x < q.xh && q.yl < y && y < q.yh;
            if inside(r.xl, r.yl) {
                if !inside(r.xh, r.yh) {
                    r = Rect::new(r.xl, r.yl, r.xl, r.yl);
                }
            } else if inside(r.xh, r.yh) {
                r = Rect::new(r.xh, r.yh, r.xh, r.yh);
            } else {
                return;
            }
        }
        if ko.except_within {
            let (w1, w2) = eol_keepout_except_rects(e, ko);
            if strictly_intersects(&w1, &r) || strictly_intersects(&w2, &r) {
                return;
            }
        }
        let end = Rect::new(e.from.0.min(e.to.0), e.from.1.min(e.to.1), e.from.0.max(e.to.0), e.from.1.max(e.to.1));
        let part = Rect::new(q.xl.max(r.xl), q.yl.max(r.yl), q.xh.min(r.xh), q.yh.min(r.yh));
        let marker = generalized_intersect(&end, &part);
        self.add_marker_of(Rule::Lef58EolKeepOut, layer, marker, (e.net, end, e.fixed), (net, part, fixed));
    }

    /// The marker between the two edges — unless a shape already fills it.
    fn eol_has_eol_helper(&mut self, layer: usize, e1: &Seg, e2: &Seg, rule: Rule) {
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
        // The sides are the two EDGES (as rectangles), the line end's the victim.
        let edge = |s: &Seg| Rect::new(s.from.0.min(s.to.0), s.from.1.min(s.to.1), s.from.0.max(s.to.0), s.from.1.max(s.to.1));
        self.add_marker_of(rule, layer, marker, (e1.net, edge(e1), e1.fixed), (e2.net, edge(e2), e2.fixed));
    }

    // ---- LEF58 corner spacing ----

    /// `checkMetalCornerSpacing`: per routing layer with corner spacing rules, per owner, per
    /// polygon corner (the corner BEFORE each edge, in ring order): the maximal rectangles in the
    /// box reaching the rules' largest (last-row) spacing out from the corner, each against each
    /// rule.
    fn check_metal_corner_spacing(&mut self) {
        if self.ignore_corner_spacing {
            return;
        }
        for layer in 0..self.tech.layers.len() {
            let l = &self.tech.layers[layer];
            if l.kind != LayerKind::Routing || l.corner_spacing.is_empty() {
                continue;
            }
            let rules = l.corner_spacing.clone();
            let (mx, my) = rules.iter().fold((0, 0), |(x, y), r| (x.max(r.find_max().0), y.max(r.find_max().1)));
            for net in 0..self.nets.len() {
                if !self.checks_from(net) {
                    continue;
                }
                for k in 0..self.segs[layer].len() {
                    if self.segs[layer][k].net != net {
                        continue;
                    }
                    let Some(c) = self.corner(layer, k) else { continue };
                    let (x, y) = (c.x, c.y);
                    let q = match c.dir {
                        CornerDir::NE => Rect::new(x, y, x + mx, y + my),
                        CornerDir::SE => Rect::new(x, y - my, x + mx, y),
                        CornerDir::SW => Rect::new(x - mx, y - my, x, y),
                        CornerDir::NW => Rect::new(x - mx, y, x, y + my),
                    };
                    for s in self.query(layer, &q) {
                        for r in &rules {
                            self.corner_spacing_helper(layer, &c, s, r);
                        }
                    }
                }
            }
        }
    }

    /// The corner before edge `k` (`initNet_pins_polygonCorners_helper`): its type from the turn,
    /// its direction from the two edges, fixed when it is a vertex of the owner's FIXED shapes.
    /// `None` for a straight joint (no type or direction).
    fn corner(&self, layer: usize, k: usize) -> Option<Corner> {
        let next = self.segs[layer][k];
        let prev = self.segs[layer][next.prev];
        let convex = match orientation(&prev, &next) {
            1 => true,
            -1 => false,
            _ => return None,
        };
        use EdgeDir::*;
        let dir = match (prev.dir(), next.dir()) {
            (N, W) | (W, N) => CornerDir::NE,
            (W, S) | (S, W) => CornerDir::NW,
            (S, E) | (E, S) => CornerDir::SW,
            (E, N) | (N, E) => CornerDir::SE,
            _ => return None,
        };
        let (x, y) = next.from;
        Some(Corner { x, y, convex, dir, net: next.net, fixed: is_polygon_vertex(&self.nets[next.net].fixed_slices[layer], (x, y)) })
    }

    /// `checkMetalCornerSpacing_main(corner, rect, con)` for the asap7 subset (convex rules, no
    /// EXCEPTEOL): only a real corner-to-corner case — the rectangle wholly off one diagonal of the
    /// corner, facing it, with a polygon corner of ITS owner at the facing corner. Then the corner's
    /// own maximal rectangles (those with it as their corner) set the required spacing by their
    /// width; closer than that (larger axis gap, or Euclidean with CORNERTOCORNER), not both
    /// rectangles fixed, and one side's width box not all fixed metal: one marker, and done.
    fn corner_spacing_helper(&mut self, layer: usize, c: &Corner, s: usize, r: &CornerSpacing) {
        if !c.convex {
            return;
        }
        let rect = self.shapes[layer][s];
        let rr = rect.rect;
        let (x, y) = (c.x, c.y);
        if rr.contains(x, y) {
            return;
        }
        let (cand_x, cand_y);
        if x >= rr.xh {
            cand_x = rr.xh;
            if y >= rr.yh {
                cand_y = rr.yh;
                if c.dir != CornerDir::SW {
                    return;
                }
            } else if y <= rr.yl {
                cand_y = rr.yl;
                if c.dir != CornerDir::NW {
                    return;
                }
            } else {
                return;
            }
        } else if x <= rr.xl {
            cand_x = rr.xl;
            if y >= rr.yh {
                cand_y = rr.yh;
                if c.dir != CornerDir::SE {
                    return;
                }
            } else if y <= rr.yl {
                cand_y = rr.yl;
                if c.dir != CornerDir::NE {
                    return;
                }
            } else {
                return;
            }
        } else {
            return;
        }
        // `hasPolyCornerAt`: any corner of the rectangle owner's polygons on the layer.
        if !self.segs[layer].iter().any(|e| e.net == rect.net && e.from == (cand_x, cand_y)) {
            return;
        }
        let pt = Rect::new(x, y, x, y);
        for o in self.query(layer, &pt) {
            let obj = self.shapes[layer][o];
            if obj.net != c.net || !corner_overlaps(c, &obj.rect) {
                continue;
            }
            let marker = generalized_intersect(&pt, &rr);
            let max_xy = marker.dx().max(marker.dy());
            if !r.same_xy {
                continue;
            }
            let req = r.find(obj.rect.dx().min(obj.rect.dy())).0;
            if r.corner_to_corner {
                let (dx, dy) = (i64::from((rr.xl - x).max(x - rr.xh).max(0)), i64::from((rr.yl - y).max(y - rr.yh).max(0)));
                if dx * dx + dy * dy >= i64::from(req) * i64::from(req) {
                    continue;
                }
            } else if max_xy >= req {
                continue;
            }
            if rect.fixed && obj.fixed {
                continue;
            }
            // "No violation if width is not contributed by route obj": the marker bloated by a
            // rectangle's width, cut to that rectangle, must not be all its owner's fixed metal —
            // the corner's rectangle first, then the other.
            let routed = |w: &Worker, sh: &Shape| {
                let wd = sh.rect.dx().min(sh.rect.dy());
                let big = Rect::new(marker.xl - wd, marker.yl - wd, marker.xh + wd, marker.yh + wd);
                match overlap(&big, &sh.rect) {
                    Some(t) => area_in(&w.nets[sh.net].fixed_slices[layer], &t) < area(&t),
                    None => false,
                }
            };
            if !routed(self, &obj) && !routed(self, &rect) {
                continue;
            }
            self.add_marker_of(Rule::CornerSpacing, layer, marker, (c.net, pt, c.fixed), (rect.net, rr, rect.fixed));
            return;
        }
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
            if l.kind != LayerKind::Cut || (l.cut_spacing.is_none() && l.cut_table.is_none()) {
                continue;
            }
            let table = l.cut_table.clone();
            for net in 0..self.nets.len() {
                if !self.checks_from(net) {
                    continue;
                }
                let mine: Vec<usize> = (0..self.shapes[layer].len()).filter(|&k| self.alive[layer][k] && self.shapes[layer][k].net == net).collect();
                // `checkCutSpacing_main(rect)`: per cut, the plain rule, then the LEF58 table.
                for k in mine {
                    if let Some(spc) = self.tech.layers[layer].cut_spacing {
                        let q = bloat(&self.shapes[layer][k].rect, spc);
                        for o in self.query(layer, &q) {
                            self.cut_pair(layer, k, o, spc);
                        }
                    }
                    if let Some(t) = &table {
                        self.cut_spacing_table(layer, k, t);
                    }
                }
            }
        }
    }

    /// `checkLef58CutSpacingTbl` (a different-net table on the cut's own layer): every other pin's
    /// cut within the largest spacing the cut's class can need — its END spacing for a square cut,
    /// the larger of END and SIDE otherwise — not both fixed.
    fn cut_spacing_table(&mut self, layer: usize, k: usize, t: &CutSpacingTable) {
        let v = self.shapes[layer][k];
        let (w, l) = (v.rect.dx().min(v.rect.dy()), v.rect.dx().max(v.rect.dy()));
        let c = self.tech.layers[layer].cut_class_of(w, l);
        let max_spc = if w == l { t.max_spacing[c][0] } else { t.max_spacing[c][0].max(t.max_spacing[c][1]) };
        for o in self.query(layer, &bloat(&v.rect, max_spc)) {
            let p = self.shapes[layer][o];
            if (p.fixed && v.fixed) || (p.net, p.pin) == (v.net, v.pin) {
                continue;
            }
            self.cut_spacing_table_pair(layer, k, o, t);
        }
    }

    /// `checkLef58CutSpacingTbl_main` for a different-net table on one layer. Same-owner cuts are
    /// checked too (the layer has no same-net or same-metal table to hand them to). Overlapping
    /// cuts are a cut SHORT (unless the classes' largest spacing is 0); apart, each direction the
    /// second cut lies in is judged by `helper`.
    fn cut_spacing_table_pair(&mut self, layer: usize, k1: usize, k2: usize, t: &CutSpacingTable) {
        let (v1, v2) = (self.shapes[layer][k1], self.shapes[layer][k2]);
        let (r1, r2) = (v1.rect, v2.rect);
        let class = |r: &Rect| self.tech.layers[layer].cut_class_of(r.dx().min(r.dy()), r.dx().max(r.dy()));
        let (c1, c2) = (class(&r1), class(&r2));
        let marker = generalized_intersect(&r1, &r2);
        let (dx, dy) = (gap((r1.xl, r1.xh), (r2.xl, r2.xh)), gap((r1.yl, r1.yh), (r2.yl, r2.yh)));
        let dist2 = i64::from(dx).pow(2) + i64::from(dy).pow(2);
        if dist2 == 0 {
            if t.max_pair_spacing(c1, c2) == 0 {
                return;
            }
            // `checkCutSpacing_short`.
            if !(v1.fixed && v2.fixed) {
                self.add_marker_of(Rule::Short, layer, marker, (v1.net, r1, v1.fixed), (v2.net, r2, v2.fixed));
            }
        }
        let center = |r: &Rect| (i64::from((r.xl + r.xh) / 2), i64::from((r.yl + r.yh) / 2));
        let ((x1, y1), (x2, y2)) = (center(&r1), center(&r2));
        let c2c2 = (x1 - x2).pow(2) + (y1 - y2).pow(2);
        let (right, left, up, down) = (r2.xl > r1.xh, r2.xh < r1.xl, r2.yl > r1.yh, r2.yh < r1.yl);
        // `checkLef58CutSpacingTbl_prlValid`: a run past the classes' PRL on either axis.
        let req_prl = t.prl_entry[t.pair(c1, c2)];
        let prl_x = if dx != 0 { -(marker.xh - marker.xl) } else { marker.xh - marker.xl };
        let prl_y = if dy != 0 { -(marker.yh - marker.yl) } else { marker.yh - marker.yl };
        let prl_valid = prl_x > req_prl || prl_y > req_prl;
        let prl = if prl_valid { prl_x.max(prl_y) } else { -1 };
        let mut viol = false;
        if up || down {
            viol = self.cut_table_helper(layer, (&r1, &r2), (c1, c2), if up { EdgeDir::N } else { EdgeDir::S }, (dist2, c2c2), (prl_valid, prl), t);
        }
        if !viol && (right || left) {
            viol = self.cut_table_helper(layer, (&r1, &r2), (c1, c2), if right { EdgeDir::E } else { EdgeDir::W }, (dist2, c2c2), (prl_valid, prl), t);
        }
        if viol {
            // ⚠️ The reference records each side's rectangle with its x-high as its y-high too.
            let side = |r: &Rect| Rect::new(r.xl, r.yl, r.xh, r.xh);
            self.add_marker_of(Rule::Lef58CutSpacingTable, layer, marker, (v1.net, side(&r1), v1.fixed), (v2.net, side(&r2), v2.fixed));
        }
    }

    /// `checkLef58CutSpacingTbl_helper`: whether the pair is too close with the second cut in
    /// direction `dir`. Each cut's facing edge is a SIDE when it is the cut's long edge. NOPRL with
    /// CENTERANDEDGE: centre to centre below the larger spacing, or edge to edge below the smaller;
    /// the same class, exactly aligned (run equal to the cut's own width across `dir`, unless the
    /// table is limited to that direction), with an EXACTALIGNED spacing: edge to edge below it;
    /// otherwise the table's first value, or its second where the run is valid (back to the first
    /// under PRLFORALIGNEDCUT when no metal edge above lies on the second cut's facing edge), centre
    /// to centre for CENTERTOCENTER (or CENTERANDEDGE where it is the larger value), else edge to
    /// edge.
    #[allow(clippy::too_many_arguments)]
    fn cut_table_helper(&self, layer: usize, (r1, r2): (&Rect, &Rect), (c1, c2): (usize, usize), dir: EdgeDir, (dist2, c2c2): (i64, i64), (prl_valid, prl): (bool, i32), t: &CutSpacingTable) -> bool {
        let (h1, v1, h2, v2) = (r1.dx(), r1.dy(), r2.dx(), r2.dy());
        let ns = matches!(dir, EdgeDir::N | EdgeDir::S);
        let (side1, side2) = if ns { (h1 > v1, h2 > v2) } else { (v1 > h1, v2 > h2) };
        let pair = t.pair(c1, c2);
        let sq = |v: i32| i64::from(v) * i64::from(v);
        if t.no_prl && t.center_and_edge[pair] {
            let (f, s) = t.get(c1, side1, c2, side2);
            if c2c2 < sq(f.max(s)) {
                return true;
            }
            return dist2 < sq(f.min(s));
        }
        if c1 == c2 {
            let aligned = if ns { prl == h1 && !t.horizontal } else { prl == v1 && !t.vertical };
            let ex = t.exact_aligned[c1];
            if aligned && ex != -1 {
                return dist2 < sq(ex);
            }
        }
        let mut second = prl_valid;
        if prl_valid && t.prl_aligned[pair] {
            let e = match dir {
                EdgeDir::S => Rect::new(r2.xl, r2.yh, r2.xh, r2.yh),
                EdgeDir::N => Rect::new(r2.xl, r2.yl, r2.xh, r2.yl),
                EdgeDir::E => Rect::new(r2.xl, r2.yl, r2.xl, r2.yh),
                EdgeDir::W => Rect::new(r2.xh, r2.yl, r2.xh, r2.yh),
            };
            if layer + 1 >= self.segs.len() || self.query_segs(layer + 1, &e).is_empty() {
                second = false;
            }
        }
        let (f, s) = t.get(c1, side1, c2, side2);
        let req = if second { s } else { f };
        let center = t.center_to_center[pair] || (t.center_and_edge[pair] && req == f.max(s));
        if center {
            c2c2 < sq(req)
        } else {
            dist2 < sq(req)
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

    // Rule: removing a net's polygon edges must renumber the survivors' prev / next — they are
    // indices into the same list, and the end-of-line checks walk them to judge a corner.
    #[test]
    fn removing_a_nets_edges_keeps_the_others_linked() {
        let mut segs = Vec::new();
        polygon_segs(&mut segs, &[Rect::new(0, 0, 100, 100)], &BTreeSet::new(), 0);
        polygon_segs(&mut segs, &[Rect::new(200, 0, 300, 100), Rect::new(200, 100, 250, 200)], &BTreeSet::new(), 1);
        let before: Vec<_> = segs.iter().filter(|e| e.net == 1).map(|e| (e.from, segs[e.prev].from, segs[e.next].from)).collect();
        retain_segs(&mut segs, 0);
        assert!(segs.iter().all(|e| e.net == 1 && e.prev < segs.len() && e.next < segs.len()));
        for (k, e) in segs.iter().enumerate() {
            assert_eq!((segs[e.next].prev, segs[e.prev].next), (k, k), "links stay mutual");
        }
        let after: Vec<_> = segs.iter().map(|e| (e.from, segs[e.prev].from, segs[e.next].from)).collect();
        assert_eq!(before, after, "each survivor keeps the SAME neighbours");
    }
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
                Layer { name: "l2".into(), kind: LayerKind::Routing, dir: Dir::Vertical, width: 170, min_width: 170, pitch: 480, wrong_way_width: 170, spacing: Some(table(vec![(0, 170)])), cut_spacing: None, cut_classes: vec![], cut_table: None, eol: vec![], lef58_eol: vec![], eol_keepout: vec![], corner_spacing: vec![], min_area: 0, min_enclosed_areas: vec![], rect_only: false, right_way_on_grid_only: false },
                Layer { name: "c3".into(), kind: LayerKind::Cut, width: 170, cut_spacing: Some(190), ..Layer::default() },
                Layer { name: "l4".into(), kind: LayerKind::Routing, dir: Dir::Horizontal, width: 140, min_width: 140, pitch: 370, wrong_way_width: 140, spacing: Some(table(vec![(0, 140), (3000, 280)])), cut_spacing: None, cut_classes: vec![], cut_table: None, eol: vec![], lef58_eol: vec![], eol_keepout: vec![], corner_spacing: vec![], min_area: 0, min_enclosed_areas: vec![], rect_only: false, right_way_on_grid_only: false },
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

    /// Rule (`checkMetalShape_minEnclosedArea`): a hole smaller than a MINENCLOSEDAREA rule is a
    /// marker on the hole's extents — here a ring of 200-wide trial wires round a 200 × 200 hole
    /// (40,000 < 50,000). None when the ring is all fixed shapes (no route shape in the polygon),
    /// or when the hole meets the rule.
    #[test]
    fn min_enclosed_area_marks_a_small_hole_of_a_routed_polygon() {
        let ring = |fixed: bool| vec![
            (net("a"), 4, Rect::new(0, 0, 600, 200), fixed),
            (net("a"), 4, Rect::new(0, 400, 600, 600), fixed),
            (net("a"), 4, Rect::new(0, 200, 200, 400), fixed),
            (net("a"), 4, Rect::new(400, 200, 600, 400), fixed),
        ];
        let run = |shapes: &[(Owner, usize, Rect, bool)], req: i32| {
            let mut t = tech();
            t.layers[4].min_enclosed_areas = vec![req];
            let mut w = Worker::new(&t);
            for (o, l, r, f) in shapes {
                w.add(o, *l, *r, *f);
            }
            w.init();
            w.run().to_vec()
        };
        assert_eq!(boxes(&run(&ring(false), 50_000), Rule::MinEnclosedArea), vec![Rect::new(200, 200, 400, 400)]);
        assert!(boxes(&run(&ring(true), 50_000), Rule::MinEnclosedArea).is_empty());
        assert!(boxes(&run(&ring(false), 40_000), Rule::MinEnclosedArea).is_empty());
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
mod cut_table_tests {
    //! A LEF58 different-net cut spacing table on constructed geometry: cut layer 3 of
    //! `tests::tech()` without its plain rule, no cut classes, every lookup 34 (the table's
    //! DEFAULT, as asap7's all-"-" tables give).
    use super::*;

    fn table(prl: i32) -> CutSpacingTable {
        CutSpacingTable { n: 1, spacing: vec![(34, 34); 4], max_spacing: vec![[34, 34]], prl_entry: vec![prl], center_to_center: vec![false], center_and_edge: vec![false], prl_aligned: vec![false], exact_aligned: vec![-1], ..Default::default() }
    }

    fn check(cuts: &[(&str, Rect, bool)]) -> Vec<(Rule, Rect)> {
        let mut t = tests::tech();
        t.layers[3].cut_spacing = None;
        t.layers[3].cut_table = Some(table(0));
        let mut w = Worker::new(&t);
        for (o, r, f) in cuts {
            w.add(&Owner::Net((*o).into()), 3, *r, *f);
        }
        w.init();
        let mut v: Vec<(Rule, Rect)> = w.run().iter().filter(|m| m.layer == 3).map(|m| (m.rule, m.bbox)).collect();
        v.dedup();
        v
    }

    // Rule (`checkLef58CutSpacingTbl_main` / `_helper`): edge to edge below the table's spacing is
    // a violation — for cuts of ONE net too, where the layer has no same-net table; overlapping
    // cuts are a cut short; both fixed, nothing.
    #[test]
    fn two_cuts_closer_than_the_table_spacing() {
        let a = Rect::new(0, 0, 18, 24);
        let b = Rect::new(38, 0, 56, 24);
        assert_eq!(check(&[("a", a, false), ("b", b, false)]), vec![(Rule::Lef58CutSpacingTable, Rect::new(18, 0, 38, 24))]);
        assert!(check(&[("a", a, false), ("b", Rect::new(52, 0, 70, 24), false)]).is_empty(), "34 apart is not below 34");
        assert_eq!(check(&[("a", a, false), ("a", b, false)]).len(), 1, "one net, two pins");
        assert!(check(&[("a", a, true), ("b", b, true)]).is_empty(), "both fixed");
        assert_eq!(check(&[("a", a, false), ("b", Rect::new(9, 0, 27, 24), false)]), vec![(Rule::Short, Rect::new(9, 0, 18, 24))]);
    }

    // Rule (`getCutClassIdx`): the LAST class of exactly the cut's size — width the shorter side.
    #[test]
    fn a_cut_takes_the_last_class_of_its_size() {
        let class = |name: &str| crate::tech::CutClass { name: name.into(), width: 18, length: 24 };
        let l = crate::tech::Layer { cut_classes: vec![class("A"), class("B")], ..Default::default() };
        assert_eq!(l.cut_class_of(18, 24), 2);
        assert_eq!(l.cut_class_of(24, 24), 0);
    }
}

#[cfg(test)]
mod corner_tests {
    //! LEF58 corner spacing on constructed geometry: layer 4 of `tests::tech()`, net `a`'s square
    //! (0, 0)–(100, 100) and a square of net `b` off its north-east corner.
    use super::*;

    fn rule(widths: Vec<i32>, spacings: Vec<i32>) -> CornerSpacing {
        CornerSpacing { widths, spacings: spacings.iter().map(|&v| (v, v)).collect(), same_xy: true, corner_to_corner: false }
    }

    fn corners(r: CornerSpacing, b: Rect, pa: bool) -> Vec<Rect> {
        let mut t = tests::tech();
        t.layers[4].corner_spacing = vec![r];
        let mut w = Worker::new(&t);
        w.ignore_corner_spacing = pa;
        w.add(&Owner::Net("a".into()), 4, Rect::new(0, 0, 100, 100), false);
        w.add(&Owner::Net("b".into()), 4, b, false);
        w.init();
        let mut v: Vec<Rect> = w.run().iter().filter(|m| m.rule == Rule::CornerSpacing).map(|m| m.bbox).collect();
        v.dedup();
        v
    }

    // Rule (`checkMetalCornerSpacing_main`): a convex corner facing another owner's corner
    // diagonally, closer (larger axis gap) than the spacing its own rectangle's width selects; a
    // rectangle overlapping the corner's x or y range is not a corner-to-corner case at all.
    #[test]
    fn a_convex_corner_keeps_a_diagonal_corner_at_its_spacing() {
        let r = rule(vec![0], vec![200]);
        assert_eq!(corners(r.clone(), Rect::new(150, 160, 250, 260), false), vec![Rect::new(100, 100, 150, 160)]);
        assert!(corners(r.clone(), Rect::new(300, 160, 400, 260), false).is_empty(), "the larger gap (200) is not below 200");
        assert!(corners(r.clone(), Rect::new(50, 160, 250, 260), false).is_empty(), "overlaps the corner's x range");
        assert!(corners(r.clone(), Rect::new(150, 160, 250, 260), true).is_empty(), "pin access ignores it");
        // CORNERTOCORNER measures Euclidean: 150 x 160 apart is 219, not below 200 — though the
        // larger axis gap (160) is.
        let far = Rect::new(250, 260, 350, 360);
        assert_eq!(corners(r.clone(), far, false).len(), 1);
        let c2c = CornerSpacing { corner_to_corner: true, ..r };
        assert_eq!(corners(c2c.clone(), Rect::new(150, 160, 250, 260), false).len(), 1);
        assert!(corners(c2c, far, false).is_empty());
    }

    // Rule (`fr1DLookupTbl::find`, lower-bound mode): the width 100 of `a`'s square EQUALS the
    // second row, and takes the row before it (50); past the last row, the last (300).
    #[test]
    fn a_width_equal_to_a_row_takes_the_row_before() {
        let r = rule(vec![0, 100], vec![50, 300]);
        assert_eq!(r.find(100), (50, 50));
        assert_eq!(r.find(150), (300, 300));
        assert_eq!(r.find(-5), (50, 50));
        assert_eq!(r.find_max(), (300, 300));
        assert!(corners(r, Rect::new(160, 160, 260, 260), false).is_empty(), "60 is not below 50");
    }
}

#[cfg(test)]
mod patch_tests {
    use super::*;

    // Rule (`modifyMarkers`): a marker on the patch's layer that touches its box, is sourced by its
    // owner and does not hold its origin grows to take the origin in; any other is left alone.
    #[test]
    fn a_patch_stretches_the_markers_it_touches_to_its_origin() {
        let p = PatchWire { layer: 4, origin: (100, 50), offset: Rect::new(0, -10, 40, 10), owner: Owner::Net("a".into()) };
        let m = |owner: &str, layer: usize, b: Rect| Marker { rule: Rule::CornerSpacing, layer, bbox: b, owners: vec![Owner::Net(owner.into())], victim: None, aggressor: None };
        let mut ms = vec![m("a", 4, Rect::new(130, 40, 160, 60)), m("b", 4, Rect::new(130, 40, 160, 60)), m("a", 6, Rect::new(130, 40, 160, 60)), m("a", 4, Rect::new(90, 40, 160, 60)), m("a", 4, Rect::new(141, 40, 160, 60))];
        modify_markers(&mut ms, &[p]);
        let boxes: Vec<Rect> = ms.iter().map(|m| m.bbox).collect();
        assert_eq!(boxes, vec![Rect::new(100, 40, 160, 60), Rect::new(130, 40, 160, 60), Rect::new(130, 40, 160, 60), Rect::new(90, 40, 160, 60), Rect::new(141, 40, 160, 60)]);
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
        EolRule { space: 200, width: 150, within: 30, parallel: None, end_to_end: None }
    }

    fn net(n: &str) -> Owner {
        Owner::Net(n.into())
    }

    // Rule (`checkMetalEndOfLine_eol_hasEol_helper`): an end-of-line marker's sides are the two
    // EDGES — the line end (victim) and the edge it faces (aggressor) — not the marker box.
    // Rule (`checkMetalEndOfLine_eol_TN`): checking only `b` (the target), `a`'s line end is still
    // checked against `b`'s facing edge first, so the marker's victim is `a`, not the target.
    #[test]
    fn an_eol_marker_names_the_two_edges_and_a_target_check_keeps_the_other_line_end() {
        let t = tech_with(rule());
        let run = |target: Option<&str>| {
            let mut w = Worker::new(&t);
            w.add(&net("a"), 4, Rect::new(0, 0, 1000, 140), false);
            w.add(&net("b"), 4, Rect::new(1150, -500, 1500, 640), false);
            w.init();
            w.target = target.map(net);
            w.run().iter().filter(|m| m.rule == Rule::EolSpacing).map(|m| (m.victim.clone(), m.aggressor.clone())).collect::<Vec<_>>()
        };
        let want = vec![(Some((net("a"), 4, Rect::new(1000, 0, 1000, 140), false)), Some((net("b"), 4, Rect::new(1150, -500, 1150, 640), false)))];
        assert_eq!(run(None), want);
        assert_eq!(run(Some("b")), want);
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

    /// A layer-4 technology with one LEF58 end-of-line keep-out rule.
    fn keepout(ko: EolKeepOut, shapes: &[(&str, Rect, bool)]) -> Vec<Rect> {
        let mut t = tests::tech();
        t.layers[4].eol_keepout = vec![ko];
        let mut w = Worker::new(&t);
        for (o, r, f) in shapes {
            w.add(&net(o), 4, *r, *f);
        }
        w.init();
        w.run().iter().filter(|m| m.rule == Rule::Lef58EolKeepOut).map(|m| m.bbox).collect()
    }

    // Rule (`checkMetalEOLkeepout_main` / `_helper`): WIRE's east end (length 140 < width 200) keeps
    // other metal out of 950..1100 x -20..160 (backward 50, forward 100, side 20). The marker is the
    // line end generalized-intersected with the box's part of the metal; except-within excuses metal
    // meeting a side window; corner-only reduces the metal to its corner strictly inside the box.
    #[test]
    fn a_line_end_keeps_other_metal_out_of_its_keep_out_box() {
        let ko = EolKeepOut { width: 200, backward: 50, forward: 100, side: 20, ..Default::default() };
        let inside = Rect::new(1050, 50, 1200, 100);
        assert_eq!(keepout(ko, &[("a", WIRE, false), ("b", inside, false)]), vec![Rect::new(1000, 50, 1050, 100)]);
        assert!(keepout(ko, &[("a", WIRE, false), ("b", Rect::new(1150, 50, 1200, 100), false)]).is_empty(), "outside the box");
        assert!(keepout(ko, &[("a", WIRE, true), ("b", inside, true)]).is_empty(), "both fixed");
        // A rule width of 140 makes WIRE's end (140 long) no end; the other metal is made with sides
        // of 200 and more, so it has no line end of its own either.
        let tall = Rect::new(1050, 50, 1250, 300);
        assert!(!keepout(ko, &[("a", WIRE, false), ("b", tall, false)]).is_empty());
        assert!(keepout(EolKeepOut { width: 140, ..ko }, &[("a", WIRE, false), ("b", tall, false)]).is_empty(), "an end as wide as the rule is no end");
        // Except-within 0..60: the window above the end is 950..1100 x 140..200; metal meeting it is
        // excused, metal not meeting it is not.
        let near_top = Rect::new(1050, 120, 1080, 150);
        let ew = EolKeepOut { except_within: true, within_low: 0, within_high: 60, ..ko };
        assert!(keepout(ew, &[("a", WIRE, false), ("b", near_top, false)]).is_empty());
        assert!(!keepout(ko, &[("a", WIRE, false), ("b", near_top, false)]).is_empty());
        // Corner-only: only the metal's lower-left corner (1080, 100) is strictly inside the box.
        let co = EolKeepOut { corner_only: true, ..ko };
        assert_eq!(keepout(co, &[("a", WIRE, false), ("b", Rect::new(1080, 100, 1300, 300), false)]), vec![Rect::new(1000, 100, 1080, 100)]);
    }

    /// A layer-4 technology with one LEF58 end-of-line spacing rule: its markers' boxes, deduplicated.
    fn lef58(r: EolRule, shapes: &[(&str, Rect, bool)]) -> Vec<Rect> {
        let mut t = tests::tech();
        t.layers[4].lef58_eol = vec![r];
        let mut w = Worker::new(&t);
        for (o, r, f) in shapes {
            w.add(&net(o), 4, *r, *f);
        }
        w.init();
        let mut v: Vec<Rect> = w.run().iter().filter(|m| m.rule == Rule::Lef58SpacingEndOfLine).map(|m| m.bbox).collect();
        v.dedup();
        v
    }

    // Rule (`checkMetalEndOfLine_eol_hasEol_getQueryBox` / `_endToEndHelper`, asap7's
    // SPACING ENDOFLINE WITHIN ENDTOEND): the window reaches the END-TO-END space (260 here), but
    // only a facing LINE END is held to it; any other edge is held to the rule's space (200).
    #[test]
    fn a_lef58_line_end_holds_another_line_end_to_the_end_to_end_space() {
        let r = EolRule { end_to_end: Some(260), ..rule() };
        assert_eq!(lef58(r, &[("a", WIRE, false), ("b", BLOCK, true)]), vec![GAP], "a long edge within space");
        let block_220 = Rect::new(1220, -500, 1500, 500);
        assert!(lef58(r, &[("a", WIRE, false), ("b", block_220, true)]).is_empty(), "a long edge past space, inside end-to-end");
        let end_220 = Rect::new(1220, 0, 2200, 140);
        assert_eq!(lef58(r, &[("a", WIRE, false), ("b", end_220, false)]), vec![Rect::new(1000, 0, 1220, 140)], "a line end inside end-to-end");
        assert!(lef58(rule(), &[("a", WIRE, false), ("b", end_220, false)]).is_empty(), "no end-to-end clause: the space alone");
        assert!(lef58(r, &[("a", WIRE, false), ("b", Rect::new(1270, 0, 2200, 140), false)]).is_empty(), "a line end past end-to-end");
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
