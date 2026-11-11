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
use crate::tech::{AdjacentCuts, CornerSpacing, CutCorner, CutSpacingTable, EolKeepOut, EolRule, LayerKind, Lef58CutSpacing, MinSpacing, ParallelEdge, Tech};

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
    /// One obstruction of an instance carrying its own DESIGNRULEWIDTH / SPACING (the check makes
    /// it an owner of its own, `frInstBlockage`): instance, its place in the master's blockages,
    /// the design rule width and the spacing (`-1` for none).
    InstBlockage(String, usize, i32, i32),
    /// A routing blockage (each its own owner, by its index in the design).
    Blockage(usize),
    /// Every unconnected ground terminal's design shapes.
    FloatingGround,
    /// Every unconnected power terminal's design shapes.
    FloatingPower,
}

impl Owner {
    pub fn is_blockage(&self) -> bool {
        matches!(self, Owner::Inst(_) | Owner::InstBlockage(..) | Owner::Blockage(_))
    }
    /// `gcNet::getDesignRuleWidth`: an obstruction's own DESIGNRULEWIDTH, else -1.
    pub fn design_rule_width(&self) -> i32 {
        if let Owner::InstBlockage(_, _, w, _) = self { *w } else { -1 }
    }
    /// `gcNet::getMinSpacing`: an obstruction's own SPACING, else -1.
    pub fn min_spacing(&self) -> i32 {
        if let Owner::InstBlockage(_, _, _, s) = self { *s } else { -1 }
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
    /// Closer than the routing layer's TWOWIDTHS spacing table allows: the same check as
    /// [`Rule::MetalSpacing`], named after the layer's rule as the reference names the marker.
    SpacingTableTw,
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
    /// One owner's polygon smaller than the LEF58_AREA rule that judges it
    /// (`checkMetalShape_lef58Area`, the marker pass); the rule's place among the layer's
    /// LEF58_AREA rules (the reference keeps a marker once per constraint).
    Lef58Area(u16),
    /// A third cut in the region a SPACINGTABLE ORTHOGONAL row keeps clear beside a pair of cuts
    /// (`checkCutSpacingTableOrthogonal`; one constraint per layer).
    SpacingTableOrth,
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
    /// Two cuts on two layers closer than a cut layer's LEF58 inter-layer cut spacing table allows
    /// (`checkLef58CutSpacingTbl` with a LAYER table): named as [`Rule::Lef58CutSpacingTable`],
    /// kept apart from it because the reference keeps a marker once per CONSTRAINT (a pair of
    /// stacked cuts beside a third can give both tables the same box and owners).
    Lef58CutSpacingTableInter,
    /// A cut closer to a concave corner, or to a non-end-of-line convex corner, of the metal holding
    /// it than a LEF58 cut spacing LAYER … CONCAVECORNER / NONEOLCONVEXCORNER rule allows
    /// (`checkLef58CutSpacing_spc_layer`); the rule's place in its layer's list (the reference
    /// keeps a marker once per constraint).
    Lef58CutSpacing(u16),
    /// A run of edges shorter than the layer's MINSTEP between two longer ones
    /// (`checkMetalShape_minStep`).
    MinStep,
    /// A short edge breaking a LEF58_MINSTEP rule (`checkMetalShape_lef58MinStep`), named as
    /// [`Rule::MinStep`]; the rule's place among the layer's kept rules (the reference keeps a
    /// marker once per constraint).
    Lef58MinStep(u16),
    /// A wide metal rectangle reached by a single cut where MINIMUMCUT needs more
    /// (`checkMinimumCut`).
    MinimumCut,
    /// Closer than a SPACING … RANGE rule allows, where that rule decided the spacing
    /// (`frSpacingRangeConstraint`, the layer's first).
    SpacingRange,
    /// A cut whose metal above or below overhangs it less than any LEF58 enclosure rule for the
    /// metal's width allows (`checkLef58Enclosure_main`).
    Lef58Enclosure,
    /// A maximal rectangle of one owner's polygon with an edge off the manufacturing grid, where
    /// the polygon holds the owner's route shapes (`checkMetalShape_offGrid`; every routing layer
    /// carries the rule, `frOffGridConstraint`).
    OffGrid,
    /// A via with no other cut within its class's LEF58_MAXSPACING (`FlexDR::fixMaxSpacing`, not
    /// the check: the pass marks each lonely via over its three-layer box, on the cut layer, the
    /// via's net as source, victim and aggressor).
    Lef58MaxSpacing,
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
    /// A second victim, for a check that names two (`checkCutSpacingTableOrthogonal_helper`:
    /// the pair; the third cut is the aggressor).
    pub second_victim: Option<Side>,
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

/// A patch the patch pass makes for a pin below its minimum area or a LEF58_AREA rule, before the
/// router picks its worker net (`checkMetalShape_addPatch`): its layer, origin, box relative to
/// the origin, and the owner (the target) whose pin it grows.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AreaPatch {
    pub layer: usize,
    pub origin: (i32, i32),
    pub offset: Rect,
    pub owner: Owner,
}

/// `checkMetalShape_addPatch`, where the patch goes on a pin's polygon edges (`segs`, ring order).
/// Upstream rules: the edge is chosen among those running ACROSS the layer's preferred direction
/// (`e->isVertical() != prefDirIsVert`), in edge order, by `bestSuitable`: an edge with both
/// corners convex beats one without; otherwise only a strictly longer edge replaces the one
/// held, so the FIRST of equals stays. The patch is as wide as that edge (half its length each
/// way from the edge's midpoint, both halves truncated) and reaches out of the polygon, on the
/// edge's outer side (inside on an edge's left), by `ceil((float) gap / length / grid) * grid`
/// — computed in single precision. The origin is the edge's midpoint across the layer and its
/// own coordinate along it. `None` when no edge runs across (the reference would crash).
fn area_patch_geometry(segs: &[Seg], vertical_layer: bool, gap: i32, grid: i32) -> Option<((i32, i32), Rect)> {
    let convex = |e: &Seg| orientation(&segs[e.prev], e) == 1 && orientation(e, &segs[e.next]) == 1;
    let mut chosen: Option<usize> = None;
    for (k, e) in segs.iter().enumerate() {
        if (e.from.0 == e.to.0) == vertical_layer {
            continue;
        }
        chosen = match chosen {
            None => Some(k),
            Some(c) => {
                let (ce, cc) = (convex(e), convex(&segs[c]));
                let wins = if ce != cc { ce } else { e.len() > segs[c].len() };
                Some(if wins { k } else { c })
            }
        };
    }
    let e = &segs[chosen?];
    let len = e.len();
    let length = ((gap as f32 / len as f32 / grid as f32).ceil() as f64 * f64::from(grid)) as i32;
    let half = len / 2;
    if vertical_layer {
        let origin = ((e.from.0 + e.to.0) / 2, e.from.1);
        let (yl, yh) = match e.dir() {
            // outer N (the edge runs W)
            EdgeDir::W => (0, length),
            // outer S (the edge runs E)
            _ => (-length, 0),
        };
        Some((origin, Rect::new(-half, yl, half, yh)))
    } else {
        let origin = (e.from.0, (e.from.1 + e.to.1) / 2);
        let (xl, xh) = match e.dir() {
            // outer E (the edge runs N)
            EdgeDir::N => (0, length),
            // outer W (the edge runs S)
            _ => (-length, 0),
        };
        Some((origin, Rect::new(xl, -half, xh, half)))
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
        Marker { victim: None, aggressor: None, second_victim: None, ..self.clone() }
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
    /// A stable identity (the list is renumbered as nets are replaced): the edge tree's value.
    uid: usize,
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

/// A point.
type P = (i32, i32);

/// A ring edge for the MINSTEP walk: its two points, and whether it is on the fixed shapes.
pub(crate) type RingEdge = (P, P, bool);

/// `checkMetalShape_minStep`'s walk over one pin's rings, each edge (from, to, fixed) in ring
/// order: from the first edge at least MINSTEP long, each run of shorter edges up to the next long
/// one is measured (its edges' end points boxed, its length summed, whether any edge — the closing
/// long one included — is off the fixed shapes) and judged (`checkMetalShape_minStep_helper`).
/// ⚠️ The corner flags are never set in the source, so an INSIDECORNER or OUTSIDECORNER rule
/// never marks. The violations' boxes.
pub(crate) fn min_step_boxes(rings: &[Vec<RingEdge>], con: &crate::tech::MinStep) -> Vec<Rect> {
    let len = |e: &RingEdge| (e.1 .0 - e.0 .0).abs() + (e.1 .1 - e.0 .1).abs();
    let mut out = Vec::new();
    for ring in rings {
        let n = ring.len();
        let Some(first) = (0..n).find(|&i| len(&ring[i]) >= con.min_step_length) else { continue };
        let mut e = first;
        let mut be = e;
        let (mut edges, mut length, mut has_route) = (0, 0, !ring[e].2);
        let mut b = Rect::new(ring[e].1 .0, ring[e].1 .1, ring[e].1 .0, ring[e].1 .1);
        loop {
            e = (e + 1) % n;
            let (to, fixed) = (ring[e].1, ring[e].2);
            if len(&ring[e]) < con.min_step_length {
                edges += 1;
                length += len(&ring[e]);
                has_route |= !fixed;
                b = Rect::new(b.xl.min(to.0), b.yl.min(to.1), b.xh.max(to.0), b.yh.max(to.1));
            } else {
                if e == be {
                    break;
                }
                has_route |= !fixed;
                if min_step_marks(con, edges, length, has_route) {
                    out.push(b);
                }
                be = e;
                if be == first {
                    break;
                }
                edges = 0;
                length = 0;
                has_route = !fixed;
                b = Rect::new(to.0, to.1, to.0, to.1);
            }
        }
    }
    out
}

/// `checkMetalShape_minStep_helper`'s verdict on a run (the corner flags always false).
fn min_step_marks(con: &crate::tech::MinStep, edges: i32, length: i32, has_route: bool) -> bool {
    if edges == 0 || !has_route {
        return false;
    }
    match con.kind {
        Some(crate::tech::MinStepKind::InsideCorner) | Some(crate::tech::MinStepKind::OutsideCorner) => false,
        Some(crate::tech::MinStepKind::Step) => length > con.max_length,
        None => con.max_edges == -1 || edges > con.max_edges,
    }
}

/// A ring edge's length.
fn ring_len(e: &RingEdge) -> i32 {
    (e.1 .0 - e.0 .0).abs() + (e.1 .1 - e.0 .1).abs()
}

/// Whether the corner where ring edge `a` ends and `b` starts is CONVEX (a left turn: the
/// shape's inside is on each edge's left).
fn ring_convex(a: &RingEdge, b: &RingEdge) -> bool {
    let (ax, ay) = (i64::from(a.1 .0 - a.0 .0), i64::from(a.1 .1 - a.0 .1));
    let (bx, by) = (i64::from(b.1 .0 - b.0 .0), i64::from(b.1 .1 - b.0 .1));
    ax * by - ay * bx > 0
}

/// The box joining ring edges' end points.
fn ring_box(edges: &[&RingEdge]) -> Rect {
    let pts = edges.iter().flat_map(|e| [e.0, e.1]);
    let (mut xl, mut yl, mut xh, mut yh) = (i32::MAX, i32::MAX, i32::MIN, i32::MIN);
    for (x, y) in pts {
        (xl, yl, xh, yh) = (xl.min(x), yl.min(y), xh.max(x), yh.max(y));
    }
    Rect::new(xl, yl, xh, yh)
}

/// The start edges of a LEF58_MINSTEP check: every edge of every ring (outer, then holes) shorter
/// than MINSTEP, in ring order, as (ring, index).
fn lef58_min_step_starts(rings: &[Vec<RingEdge>], min_step_length: i32) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    for (r, ring) in rings.iter().enumerate() {
        out.extend((0..ring.len()).filter(|&i| ring_len(&ring[i]) < min_step_length).map(|i| (r, i)));
    }
    out
}

/// `checkMetalShape_lef58MinStep_noBetweenEol` on one pin's rings. Upstream rule: per start edge
/// ([`lef58_min_step_starts`]), skip unless the edge after next is shorter than MINSTEP, the next
/// edge has both corners CONVEX and is shorter than the NOBETWEENEOL width, and not all three
/// edges are fixed; the box joins the start edge's two ends and the next two edges' far ends.
pub(crate) fn lef58_min_step_no_between_eol_boxes(
    rings: &[Vec<RingEdge>],
    con: &crate::tech::Lef58MinStep,
) -> Vec<Rect> {
    let mut out = Vec::new();
    for (r, i) in lef58_min_step_starts(rings, con.min_step_length) {
        let ring = &rings[r];
        let n = ring.len();
        let (start, next, nnext) = (&ring[i], &ring[(i + 1) % n], &ring[(i + 2) % n]);
        if ring_len(nnext) >= con.min_step_length {
            continue;
        }
        if !ring_convex(start, next) || !ring_convex(next, nnext) {
            continue;
        }
        if ring_len(next) >= con.eol_width {
            continue;
        }
        if start.2 && next.2 && nnext.2 {
            continue;
        }
        out.push(ring_box(&[start, next, nnext]));
    }
    out
}

/// `checkMetalShape_lef58MinStep_minAdjLength` on one pin's rings. Upstream rule: nothing for a
/// pin whose outer ring has four edges under EXCEPTRECTANGLE (`poly->size() == 4`, holes or
/// not); per start edge ([`lef58_min_step_starts`]), a violation when the next or the previous
/// edge is shorter than MINADJACENTLENGTH or, with NOADJACENTEOL, has both corners CONVEX and is
/// shorter than its width — unless all three edges are fixed; the box joins the three edges.
pub(crate) fn lef58_min_step_min_adj_length_boxes(
    rings: &[Vec<RingEdge>],
    con: &crate::tech::Lef58MinStep,
) -> Vec<Rect> {
    if con.except_rectangle && rings.first().is_some_and(|outer| outer.len() == 4) {
        return Vec::new();
    }
    let mut out = Vec::new();
    for (r, i) in lef58_min_step_starts(rings, con.min_step_length) {
        let ring = &rings[r];
        let n = ring.len();
        let (pprev, prev, start) = (&ring[(i + n - 2) % n], &ring[(i + n - 1) % n], &ring[i]);
        let (next, nnext) = (&ring[(i + 1) % n], &ring[(i + 2) % n]);
        let mut violating = ring_len(next) < con.min_adjacent_length || ring_len(prev) < con.min_adjacent_length;
        if con.no_adj_eol > -1 {
            violating |= ring_convex(start, next) && ring_convex(next, nnext) && ring_len(next) < con.no_adj_eol;
            violating |= ring_convex(pprev, prev) && ring_convex(prev, start) && ring_len(prev) < con.no_adj_eol;
        }
        if start.2 && next.2 && prev.2 {
            continue;
        }
        if !violating {
            continue;
        }
        out.push(ring_box(&[start, next, prev]));
    }
    out
}

/// An edge's box (its two points, low to high).
fn seg_box(e: &Seg) -> Rect {
    Rect::new(e.from.0.min(e.to.0), e.from.1.min(e.to.1), e.from.0.max(e.to.0), e.from.1.max(e.to.1))
}

/// An edge as its two points.
type EdgePoints = ((i32, i32), (i32, i32));

/// A region's polygon edges, with their polygon, whether fixed, and the ring order.
fn polygon_segs(out: &mut Vec<Seg>, slices: &[Rect], fixed_edges: &BTreeSet<EdgePoints>, net: usize) {
    // `initNet_pins_polygonEdges`: per polygon (the scan's order — `polygons()`'s, so a pin's
    // number here is its shapes'), the outer ring then each hole, each from its first vertex as
    // the polygon formation left it; linked around the ring.
    for (pin, poly) in crate::polygon_formation::polygons_with_holes(slices).into_iter().enumerate() {
        for ring in std::iter::once(&poly.outer).chain(&poly.holes) {
            let base = out.len();
            let n = ring.len();
            for k in 0..n {
                let (from, to) = (ring[k], ring[(k + 1) % n]);
                out.push(Seg { from, to, net, pin, fixed: fixed_edges.contains(&(from, to)), prev: base + (k + n - 1) % n, next: base + (k + 1) % n, uid: 0 });
            }
        }
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
    /// Per layer, the polygon edges' query tree (`queryPolygonEdge`): each edge's box, by uid;
    /// bulk-loaded at `init` in the list's order (owner, pin, ring), updated per owner after.
    seg_rq: Vec<DynRTree<usize>>,
    /// Per layer: uid → the edge's tree id and its place in the list.
    seg_tree_id: Vec<HashMap<usize, usize>>,
    seg_at: Vec<HashMap<usize, usize>>,
    next_uid: usize,
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
    /// The router's top routing layer (`TOP_ROUTING_LAYER`): the default inter-layer cut spacing
    /// table of the layer two above a cut is read only below it. The technology's topmost routing
    /// layer unless set (io's default, without a maximum routing layer on the block).
    pub top_routing_layer: usize,
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
/// `checkLef58CutSpacingTbl_main`'s net filter for a table on the cut's own layer: a SAMENET
/// table skips cuts of two nets; the different-net table skips cuts of one net when the layer has
/// a SAMENET table (it then judges them).
/// The LEF58 cut spacing table a check reads: one on the cut's layer (SAMENET or different-net),
/// or a default inter-layer table (`table_layer` its own layer, `second` its second layer).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CutTableKind {
    SameNet,
    DiffNet,
    Inter { table_layer: usize, second: usize, no_stack: bool },
}

fn same_net_table_skips(same_net_table: bool, one_net: bool, layer_has_same_net: bool) -> bool {
    if same_net_table {
        !one_net
    } else {
        one_net && layer_has_same_net
    }
}

/// Two cuts' squared distance as a cut spacing rule measures it: centre to centre (each centre
/// the integer midpoint, `gtl::center`) or edge to edge (`square_euclidean_distance`, 0 when
/// they touch or overlap).
fn cut_d2(a: &Rect, b: &Rect, center_to_center: bool) -> i64 {
    if center_to_center {
        let c = |r: &Rect| (i64::from((r.xl + r.xh) / 2), i64::from((r.yl + r.yh) / 2));
        let ((x1, y1), (x2, y2)) = (c(a), c(b));
        (x1 - x2).pow(2) + (y1 - y2).pow(2)
    } else {
        i64::from(gap((a.xl, a.xh), (b.xl, b.xh))).pow(2) + i64::from(gap((a.yl, a.yh), (b.yl, b.yh))).pow(2)
    }
}

fn gap(a: (i32, i32), b: (i32, i32)) -> i32 {
    (b.0 - a.1).max(a.0 - b.1).max(0)
}

/// Per axis the overlap of the two rectangles, or, where they are apart, the gap between them.
/// `gtl::intersects(a, b, false)`: both extents overlap with a length (touching does not count).
fn strictly_intersects(a: &Rect, b: &Rect) -> bool {
    a.xl < b.xh && b.xl < a.xh && a.yl < b.yh && b.yl < a.yh
}

/// The half-width of the window beyond a line end past each of its sides
/// (`checkMetalEndOfLine_eol_hasEol_getQueryBox`'s `eolWithin`): the rule's WITHIN, replaced by
/// twice ENDTOEND's EXTENSION when the rule has one, even where that is the smaller.
fn eol_query_within(r: &EolRule) -> i32 {
    match (r.end_to_end, r.extension) {
        (Some(_), Some(x)) => x * 2,
        _ => r.within,
    }
}

/// `gtl::bloat(rect, edge->getOrientation(), d)`: widen the edge's rectangle by `d` at both ends
/// along the edge (x for a horizontal edge, y for a vertical one).
fn bloat_along(r: &mut Rect, e: &Seg, d: i32) {
    if e.from.1 == e.to.1 {
        r.xl -= d;
        r.xh += d;
    } else {
        r.yl -= d;
        r.yh += d;
    }
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

/// `getRectsPrl`: per axis the extent of `generalized_intersect` — the overlap where the two
/// rectangles overlap on that axis, negated where they are apart on it (a touch is 0).
fn rects_prl(a: &Rect, b: &Rect) -> (i32, i32) {
    let m = generalized_intersect(a, b);
    let (mut x, mut y) = (m.xh - m.xl, m.yh - m.yl);
    if gap((a.xl, a.xh), (b.xl, b.xh)) != 0 {
        x = -x;
    }
    if gap((a.yl, a.yh), (b.yl, b.yh)) != 0 {
        y = -y;
    }
    (x, y)
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
        let mut w = Worker { tech, nets: Vec::new(), index: HashMap::new(), shapes: Vec::new(), edges: Vec::new(), segs: Vec::new(), seg_rq: Vec::new(), seg_tree_id: Vec::new(), seg_at: Vec::new(), next_uid: 0, ignore_long_side_eol: false, target: None, check_ndrs: false, ignore_min_area: false, ignore_corner_spacing: false, drc_box: None, max_ndr_spacing: Vec::new(), top_routing_layer: tech.layers.iter().rposition(|l| l.kind == LayerKind::Routing).unwrap_or(0), spc: Vec::new(), spc_listed: Vec::new(), spc_rq: Vec::new(), alive: Vec::new(), rq_id: Vec::new(), rq: Vec::new(), markers: Vec::new(), seen: BTreeSet::new() };
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
        self.seg_rq = (0..n).map(|l| DynRTree::new(self.segs[l].iter().map(|e| (seg_box(e), e.uid)).collect())).collect();
        self.seg_tree_id = (0..n).map(|l| self.segs[l].iter().enumerate().map(|(k, e)| (e.uid, k)).collect()).collect();
        self.seg_at = vec![HashMap::new(); n];
        for l in 0..n {
            self.index_segs(l);
        }
    }

    /// Where each edge of `layer` sits in the list now.
    fn index_segs(&mut self, layer: usize) {
        self.seg_at[layer] = self.segs[layer].iter().enumerate().map(|(k, e)| (e.uid, k)).collect();
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
                let from = self.segs[layer].len();
                polygon_segs(&mut self.segs[layer], &slices, &fixed_edges, i);
                for e in &mut self.segs[layer][from..] {
                    e.uid = self.next_uid;
                    self.next_uid += 1;
                }
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
            for e in self.segs[layer].iter().filter(|e| e.net == i) {
                if let Some(id) = self.seg_tree_id[layer].remove(&e.uid) {
                    self.seg_rq[layer].remove(id);
                }
            }
            retain_segs(&mut self.segs[layer], i);
            self.index_segs(layer);
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
        let old_segs: Vec<usize> = self.segs.iter().map(|v| v.len()).collect();
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
        for (layer, &from) in old_segs.iter().enumerate() {
            for k in from..self.segs[layer].len() {
                let e = self.segs[layer][k];
                let id = self.seg_rq[layer].insert(seg_box(&e), e.uid);
                self.seg_tree_id[layer].insert(e.uid, id);
            }
            self.index_segs(layer);
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
        self.check_minimum_cut();
        modify_markers(&mut self.markers, patches);
        normalize_marker_order(&mut self.markers);
        &self.markers
    }

    /// `checkMinimumCut`: per routing layer with MINIMUMCUT rules, per owner checked from, per pin,
    /// per maximal rectangle at least WIDTH wide (and LENGTH long, if given): each same-owner cut
    /// below and/or above it (the rule's connection side; within its box, grown by the LENGTH
    /// distance) is a violation — outright without LENGTH; with it, only a cut not inside the
    /// rectangle whose own metal rectangle (same owner) also touches the wide one. Not both fixed.
    fn check_minimum_cut(&mut self) {
        let top = self.tech.layers.len() - 1;
        for layer in 0..self.tech.layers.len() {
            if self.tech.layers[layer].kind != LayerKind::Routing || self.tech.layers[layer].min_cuts.is_empty() {
                continue;
            }
            let rules = self.tech.layers[layer].min_cuts.clone();
            for net in 0..self.nets.len() {
                if !self.checks_from(net) {
                    continue;
                }
                let mine: Vec<Shape> = (0..self.shapes[layer].len()).filter(|&k| self.alive[layer][k] && self.shapes[layer][k].net == net).map(|k| self.shapes[layer][k]).collect();
                for r in mine {
                    for m in &rules {
                        self.minimum_cut_main(layer, top, &r, m);
                    }
                }
            }
        }
    }

    fn minimum_cut_main(&mut self, layer: usize, top: usize, r: &Shape, m: &crate::tech::MinCut) {
        let (w, len) = (r.rect.dx().min(r.rect.dy()), r.rect.dx().max(r.rect.dy()));
        if w < m.width || m.length.is_some_and(|(ml, _)| len < ml) {
            return;
        }
        let d = m.length.map_or(0, |(_, dist)| dist);
        let q = Rect::new(r.rect.xl - d, r.rect.yl - d, r.rect.xh + d, r.rect.yh + d);
        let mut found: Vec<(usize, usize)> = Vec::new();
        if m.from_above != Some(true) && layer > 0 {
            found.extend(self.query(layer - 1, &q).into_iter().map(|k| (layer - 1, k)));
        }
        if m.from_above != Some(false) && layer < top {
            found.extend(self.query(layer + 1, &q).into_iter().map(|k| (layer + 1, k)));
        }
        let touch = |a: &Rect, b: &Rect| a.xl <= b.xh && b.xl <= a.xh && a.yl <= b.yh && b.yl <= a.yh;
        for (vl, k) in found {
            let via = self.shapes[vl][k];
            if via.net != r.net || (via.fixed && r.fixed) {
                continue;
            }
            let inside = r.rect.xl <= via.rect.xl && r.rect.yl <= via.rect.yl && via.rect.xh <= r.rect.xh && via.rect.yh <= r.rect.yh;
            if m.length.is_some() && inside {
                continue;
            }
            if m.length.is_some() {
                let enclosed = self.query(layer, &via.rect).into_iter().any(|e| {
                    let s = &self.shapes[layer][e];
                    s.net == via.net && touch(&s.rect, &via.rect) && touch(&s.rect, &r.rect)
                });
                if !enclosed {
                    continue;
                }
            }
            let b = generalized_intersect(&r.rect, &via.rect);
            let mut owners = vec![self.owner(r.net).clone()];
            owners.dedup();
            if self.seen.insert((b, layer, Rule::MinimumCut, owners.clone())) {
                let victim = (self.owner(r.net).clone(), layer, r.rect, r.fixed);
                let aggressor = (self.owner(via.net).clone(), vl, via.rect, via.fixed);
                self.markers.push(Marker { rule: Rule::MinimumCut, layer, bbox: b, owners, victim: Some(victim), aggressor: Some(aggressor), second_victim: None });
            }
        }
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

    /// The minimum-step markers of `checkMetalShape(true)`, as the surgical fix's patch pass reads
    /// them (`patchMetalShape_minStep` walks `markers_` after that check): per routing layer
    /// (bottom up), per owner checked from, per pin, in the order made — the MINSTEP markers, then
    /// the LEF58_MINSTEP ones (`checkMetalShape_main` checks them in that order).
    pub fn min_step_markers(&mut self) -> Vec<Marker> {
        self.markers.clear();
        self.seen.clear();
        for layer in 0..self.tech.layers.len() {
            let l = &self.tech.layers[layer];
            if l.kind != LayerKind::Routing || (l.min_step.is_none() && l.lef58_min_step.is_empty()) {
                continue;
            }
            for net in 0..self.nets.len() {
                if !self.checks_from(net) || self.nets[net].pins.len() <= layer {
                    continue;
                }
                for k in 0..self.nets[net].pins[layer].len() {
                    self.min_step(layer, net, k);
                    self.lef58_min_step(layer, net, k);
                }
            }
        }
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
            self.markers.push(Marker { rule, layer, bbox, owners, victim: Some(victim), aggressor: Some(aggressor), second_victim: None });
        }
    }

    /// A marker whose two sides are on two layers (the box on the first's), once per rule, box and
    /// owners.
    fn add_marker_two_layers(&mut self, rule: Rule, bbox: Rect, v: (usize, usize, Rect, bool), g: (usize, usize, Rect, bool)) {
        let mut owners = vec![self.owner(v.0).clone(), self.owner(g.0).clone()];
        owners.sort();
        owners.dedup();
        if self.seen.insert((bbox, v.1, rule, owners.clone())) {
            let victim = (self.owner(v.0).clone(), v.1, v.2, v.3);
            let aggressor = (self.owner(g.0).clone(), g.1, g.2, g.3);
            self.markers.push(Marker { rule, layer: v.1, bbox, owners, victim: Some(victim), aggressor: Some(aggressor), second_victim: None });
        }
    }

    /// Every standing shape on `layer` touching `r`, in the tree's order.
    fn query(&self, layer: usize, r: &Rect) -> Vec<usize> {
        self.rq[layer].query(r).into_iter().map(|(_, v)| v.1).collect()
    }

    // ---- metal shapes ----

    /// Per routing layer (bottom up), per owner checked from, per pin: minimum width, minimum area,
    /// minimum step, rect-only, off-grid, minimum enclosed area — `checkMetalShape_main`'s order,
    /// less the checks not modelled.
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
                    self.metal_shape_of(layer, net, k, &mut pin);
                }
            }
        }
    }

    fn metal_shape_of(&mut self, layer: usize, net: usize, k: usize, pin: &mut Polygon90Set) {
        // Minimum width: the pin sliced horizontally, each slice's x length; then sliced
        // vertically, each slice's y length.
        for r in pin.rectangles() {
            self.min_width(layer, net, r, r.xh - r.xl);
        }
        for r in vertical_slices(pin) {
            self.min_width(layer, net, r, r.yh - r.yl);
        }
        self.min_area(layer, net, pin);
        self.min_step(layer, net, k);
        self.lef58_min_step(layer, net, k);
        self.rect_only(layer, net, pin);
        self.off_grid(layer, net, pin);
        self.min_enclosed_area(layer, net, pin);
        self.lef58_area(layer, net, pin);
    }

    /// `checkMetalShape_offGrid`: per maximal rectangle of the pin, in its order, one with any edge
    /// off the manufacturing grid (signed remainder, as `%` in the reference) is a marker on the
    /// rectangle itself — unless it holds none of the owner's ROUTE shapes (`net->getPolygons(layer,
    /// false) & maxRect` empty: fixed metal off the grid is not the router's to fix). Victim and
    /// aggressor are the owner on that rectangle, not fixed.
    fn off_grid(&mut self, layer: usize, net: usize, pin: &mut Polygon90Set) {
        let grid = self.tech.manufacturing_grid;
        if grid <= 0 {
            // No grid to be off (the reference would divide by zero).
            return;
        }
        for r in pin.max_rectangles() {
            if r.xl % grid == 0 && r.xh % grid == 0 && r.yl % grid == 0 && r.yh % grid == 0 {
                continue;
            }
            if !self.nets[net].route_slices[layer].iter().any(|s| overlap(s, &r).is_some()) {
                continue;
            }
            self.add_marker(Rule::OffGrid, layer, r, net, net);
        }
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
    /// marker on its bounding box (see [`Worker::min_area_breach`]).
    fn min_area(&mut self, layer: usize, net: usize, pin: &mut Polygon90Set) {
        if let Some((bbox, _)) = self.min_area_breach(layer, net, pin) {
            self.add_marker(Rule::MinArea, layer, bbox, net, net);
        }
    }

    /// `checkMetalShape_minArea`'s test, shared by the marker pass and the patch pass: the
    /// polygon's bounding box and the layer's minimum area, when the polygon is below it — unless
    /// the layer has no area rule, the check is ignored (pin access), the box is not WHOLLY
    /// inside the worker's check box, or any of its edges is fixed (on the owner's fixed shapes).
    fn min_area_breach(&self, layer: usize, net: usize, pin: &mut Polygon90Set) -> Option<(Rect, i64)> {
        let required = self.tech.layers[layer].min_area;
        if self.ignore_min_area || required == 0 {
            return None;
        }
        let slices = pin.rectangles();
        if slices.iter().map(area).sum::<i64>() >= required {
            return None;
        }
        let bbox = self.pin_bbox_in_drc_box(&slices)?;
        if self.pin_segs(layer, net, &slices).iter().any(|e| e.fixed) {
            return None;
        }
        Some((bbox, required))
    }

    /// A pin's bounding box, when the worker's check box holds it wholly (`drcBox_.contains`).
    fn pin_bbox_in_drc_box(&self, slices: &[Rect]) -> Option<Rect> {
        let bbox = slices.iter().skip(1).fold(slices[0], |b, r| Rect { xl: b.xl.min(r.xl), yl: b.yl.min(r.yl), xh: b.xh.max(r.xh), yh: b.yh.max(r.yh) });
        if let Some(d) = self.drc_box {
            if !(d.xl <= bbox.xl && d.yl <= bbox.yl && bbox.xh <= d.xh && bbox.yh <= d.yh) {
                return None;
            }
        }
        Some(bbox)
    }

    /// A pin's polygon edges (`pin->getPolygonEdges()`, outer ring then holes, ring order), each
    /// marked fixed when it lies on the owner's fixed shapes' boundary or a fixed rectangle's.
    fn pin_segs(&self, layer: usize, net: usize, slices: &[Rect]) -> Vec<Seg> {
        let net_ref = &self.nets[net];
        let mut fixed_edges: BTreeSet<EdgePoints> = boundary(&net_ref.fixed_slices[layer]).into_iter().collect();
        for r in &net_ref.fixed_rects[layer] {
            fixed_edges.extend([((r.xl, r.yl), (r.xh, r.yl)), ((r.xh, r.yl), (r.xh, r.yh)), ((r.xh, r.yh), (r.xl, r.yh)), ((r.xl, r.yh), (r.xl, r.yl))]);
        }
        let mut segs = Vec::new();
        polygon_segs(&mut segs, slices, &fixed_edges, net);
        segs
    }

    /// `checkMetalShape_lef58Area`, the marker pass: a polygon breaking a LEF58_AREA rule gives a
    /// marker on its bounding box, with that rule (see [`Worker::lef58_area_breach`]).
    fn lef58_area(&mut self, layer: usize, net: usize, pin: &mut Polygon90Set) {
        if let Some((bbox, rule)) = self.lef58_area_breach(layer, net, pin) {
            self.add_marker(Rule::Lef58Area(rule.index), layer, bbox, net, net);
        }
    }

    /// `checkMetalShape_lef58Area`'s test, shared by the marker pass and the patch pass: the
    /// polygon's bounding box and the rule it breaks ([`crate::tech::Lef58Area::violated`]) —
    /// unless the check is ignored (pin access: the reference reads `ignoreMinArea_` here too),
    /// the layer has no LEF58_AREA rule, the box is not WHOLLY inside the worker's check box, or
    /// any of the polygon's edges is fixed. A rectangle is a polygon whose outer boundary has
    /// four vertices (`poly->size() == 4`; with a hole it is one too, and no RECTWIDTH rule then
    /// applies, as it is not one maximal rectangle).
    fn lef58_area_breach(&self, layer: usize, net: usize, pin: &mut Polygon90Set) -> Option<(Rect, crate::tech::AreaRule)> {
        let rules = &self.tech.layers[layer].lef58_area;
        if self.ignore_min_area || rules.is_empty() {
            return None;
        }
        let slices = pin.rectangles();
        let bbox = self.pin_bbox_in_drc_box(&slices)?;
        if self.pin_segs(layer, net, &slices).iter().any(|e| e.fixed) {
            return None;
        }
        let is_rect = crate::polygon_formation::polygons_with_holes(&slices).first().is_some_and(|p| p.outer.len() == 4);
        let curr: i64 = slices.iter().map(area).sum();
        let max_rects = pin.max_rectangles();
        rules.violated(is_rect, &max_rects, curr).map(|r| (bbox, r))
    }

    /// The patch pass's minimum-area half (`checkMetalShape(true)`, its `checkMetalShape_minArea`
    /// and `checkMetalShape_lef58Area` with patching): per routing layer (bottom up), per pin of
    /// the TARGET owner, a patch for the layer's minimum area when the pin is below it, then one
    /// for the LEF58_AREA rule it breaks — both from the pin as it stands, so a pin below both
    /// gets two patches. None without a target (`allow_patching && !targetNet_`). Each patch is
    /// placed by [`area_patch_geometry`]; the router then picks its worker net and keeps it when
    /// valid (`checkMetalShape_addPatch`'s second half).
    pub fn area_patches(&mut self) -> Vec<AreaPatch> {
        let mut out = Vec::new();
        let Some(target) = self.target.clone() else { return out };
        for layer in 0..self.tech.layers.len() {
            if self.tech.layers[layer].kind != LayerKind::Routing {
                continue;
            }
            let vertical = self.tech.layers[layer].dir == crate::tech::Dir::Vertical;
            for net in 0..self.nets.len() {
                if !self.checks_from(net) || self.nets[net].pins.len() <= layer {
                    continue;
                }
                for k in 0..self.nets[net].pins[layer].len() {
                    let mut pin = self.nets[net].pins[layer][k].clone();
                    let mut required = Vec::new();
                    if let Some((_, req)) = self.min_area_breach(layer, net, &mut pin) {
                        required.push(req);
                    }
                    if let Some((_, rule)) = self.lef58_area_breach(layer, net, &mut pin) {
                        required.push(rule.area);
                    }
                    if required.is_empty() {
                        continue;
                    }
                    let slices = pin.rectangles();
                    let curr: i64 = slices.iter().map(area).sum();
                    let segs = self.pin_segs(layer, net, &slices);
                    for req in required {
                        // `checkMetalShape_addPatch(gcPin*, int min_area)`: the area narrowed to
                        // int, the gap to `frCoord`.
                        let gap = (i64::from(req as i32) - curr) as i32;
                        if let Some((origin, offset)) = area_patch_geometry(&segs, vertical, gap, self.tech.manufacturing_grid) {
                            out.push(AreaPatch { layer, origin, offset, owner: target.clone() });
                        }
                    }
                }
            }
        }
        out
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

    /// The owner's pin `k`'s rings (`gcPin::getPolygonEdges`) as the list holds them: the outer
    /// ring, then the holes, each edge (from, to, fixed) in ring order.
    fn pin_rings(&self, layer: usize, net: usize, k: usize) -> Vec<Vec<RingEdge>> {
        let segs = &self.segs[layer];
        let mut rings: Vec<Vec<RingEdge>> = Vec::new();
        let mut seen = vec![false; segs.len()];
        for s in 0..segs.len() {
            if seen[s] || segs[s].net != net || segs[s].pin != k {
                continue;
            }
            let mut ring = Vec::new();
            let mut i = s;
            while !seen[i] {
                seen[i] = true;
                ring.push((segs[i].from, segs[i].to, segs[i].fixed));
                i = segs[i].next;
            }
            rings.push(ring);
        }
        rings
    }

    /// `checkMetalShape_minStep` on the owner's pin `k`: its rings as the list holds them.
    fn min_step(&mut self, layer: usize, net: usize, k: usize) {
        let Some(con) = self.tech.layers[layer].min_step else { return };
        let rings = self.pin_rings(layer, net, k);
        for b in min_step_boxes(&rings, &con) {
            self.add_marker_of(Rule::MinStep, layer, b, (net, b, false), (net, b, false));
        }
    }

    /// `checkMetalShape_lef58MinStep` on the owner's pin `k`: per kept LEF58_MINSTEP rule, in
    /// order, NOBETWEENEOL ([`Self::lef58_min_step_no_between_eol`]) then MINADJACENTLENGTH
    /// ([`Self::lef58_min_step_min_adj_length`]), each when the rule has it.
    fn lef58_min_step(&mut self, layer: usize, net: usize, k: usize) {
        let tech = self.tech;
        if tech.layers[layer].lef58_min_step.is_empty() {
            return;
        }
        let rings = self.pin_rings(layer, net, k);
        for (i, con) in tech.layers[layer].lef58_min_step.iter().enumerate() {
            let rule = Rule::Lef58MinStep(u16::try_from(i).unwrap_or(u16::MAX));
            if con.eol_width != -1 {
                self.lef58_min_step_no_between_eol(layer, net, rule, &rings, con);
            }
            if con.min_adjacent_length != -1 {
                self.lef58_min_step_min_adj_length(layer, net, rule, &rings, con);
            }
        }
    }

    /// `checkMetalShape_lef58MinStep_noBetweenEol`: a marker per box of
    /// [`lef58_min_step_no_between_eol_boxes`], the owner as victim and aggressor, not fixed.
    fn lef58_min_step_no_between_eol(
        &mut self,
        layer: usize,
        net: usize,
        rule: Rule,
        rings: &[Vec<RingEdge>],
        con: &crate::tech::Lef58MinStep,
    ) {
        for b in lef58_min_step_no_between_eol_boxes(rings, con) {
            self.add_marker_of(rule, layer, b, (net, b, false), (net, b, false));
        }
    }

    /// `checkMetalShape_lef58MinStep_minAdjLength`: a marker per box of
    /// [`lef58_min_step_min_adj_length_boxes`], the owner as victim and aggressor, not fixed.
    fn lef58_min_step_min_adj_length(
        &mut self,
        layer: usize,
        net: usize,
        rule: Rule,
        rings: &[Vec<RingEdge>],
        con: &crate::tech::Lef58MinStep,
    ) {
        for b in lef58_min_step_min_adj_length_boxes(rings, con) {
            self.add_marker_of(rule, layer, b, (net, b, false), (net, b, false));
        }
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
        (triggered && self.eol_has_min_max_length(layer, k, r)).then_some(has_route)
    }

    /// `checkMetalEndOfLine_eol_hasMinMaxLength`, last in `qualifiesAsEol`: with MINLENGTH (or
    /// MAXLENGTH) the line end counts only when the edges before and after it are at least (at
    /// most) that long — both with TWOSIDES, either without. ⛔ TWOSIDES is the rule's own flag
    /// here, a divergence by decision (drt divergence register D8): the reference's io hands it
    /// PARALLELEDGE's TWOEDGES instead (upstream OpenROAD issue #12181); we follow the patched
    /// reference (vyges-openroad-patches upstream-fix 0001).
    fn eol_has_min_max_length(&self, layer: usize, k: usize, r: &EolRule) -> bool {
        let Some(m) = r.length else { return true };
        let segs = &self.segs[layer];
        let e = &segs[k];
        let ok = |len: i32| if m.max { len <= m.length } else { len >= m.length };
        let (prev, next) = (ok(segs[e.prev].len()), ok(segs[e.next].len()));
        if m.two_sides {
            prev && next
        } else {
            prev || next
        }
    }

    /// Every polygon edge on `layer` touching `q`.
    fn query_segs(&self, layer: usize, q: &Rect) -> Vec<usize> {
        self.seg_rq.get(layer).map_or(Vec::new(), |t| t.query(q).into_iter().map(|(_, v)| self.seg_at[layer][&v.1]).collect())
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

    /// `checkMetalEndOfLine_eol_hasEol_getQueryBox`: the window beyond a line end, `space` out
    /// (at least the END-TO-END space) and `within` past each side, or with ENDTOEND's EXTENSION
    /// twice the extension past each side INSTEAD of `within` (`eolWithin = getExtension() * 2`,
    /// even where that is the smaller: [`eol_query_within`]).
    fn eol_query_rect(e: &Seg, r: &EolRule) -> Rect {
        let (w, sp) = (eol_query_within(r), r.space.max(r.end_to_end.unwrap_or(0)));
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
        // An obstruction with its own SPACING on either side: no end-of-line check.
        if self.owner(e.net).min_spacing() != -1 || self.owner(ptr.net).min_spacing() != -1 {
            return;
        }
        let opposite = matches!((e.dir(), ptr.dir()), (EdgeDir::E, EdgeDir::W) | (EdgeDir::W, EdgeDir::E) | (EdgeDir::N, EdgeDir::S) | (EdgeDir::S, EdgeDir::N));
        if !opposite {
            return;
        }
        if r.opposite_width.is_some_and(|w| self.eol_opposite_wire_span(layer, &ptr) >= w) {
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

    /// OPPOSITEWIDTH's span of the wire beyond a line end, seen from its facing edge `ptr`: the
    /// largest extent, perpendicular to `ptr`, of a maximal rectangle of `ptr`'s polygon that has a
    /// side on `ptr`'s line overlapping `ptr` with positive length. The rule applies only when this
    /// is below OPPOSITEWIDTH (LEF definition). ⛔ DIVERGENCE by decision (drt divergence register
    /// D13, upstream OpenROAD issue #12226): the reference stores OPPOSITEWIDTH and never reads it;
    /// vyges-openroad-patches upstream-fix 0015 adds the same test (`getOppositeWireSpan`).
    fn eol_opposite_wire_span(&self, layer: usize, ptr: &Seg) -> i32 {
        let b = seg_box(ptr);
        let horizontal = ptr.from.1 == ptr.to.1;
        let mut span = 0;
        for s in self.query(layer, &b) {
            let sh = &self.shapes[layer][s];
            if sh.net != ptr.net || sh.pin as usize != ptr.pin {
                continue;
            }
            let r = sh.rect;
            let (at, (lo, hi), (alo, ahi), (elo, ehi)) = if horizontal {
                (b.yl, (r.yl, r.yh), (r.xl, r.xh), (b.xl, b.xh))
            } else {
                (b.xl, (r.xl, r.xh), (r.yl, r.yh), (b.yl, b.yh))
            };
            if lo != at && hi != at {
                continue;
            }
            if ahi.min(ehi) - alo.max(elo) <= 0 {
                continue;
            }
            span = span.max(hi - lo);
        }
        span
    }

    /// `checkMetalEndOfLine_eol_hasEol_endToEndHelper` (LEF58 rules only): the two edges as
    /// rectangles; a facing line end takes the END-TO-END space, BOTH edges widened along
    /// themselves by ENDTOEND's EXTENSION when the rule has one (else neither); anything else the
    /// rule's space with the line end widened by `within` along itself. A violation needs a run
    /// (positive projection on the axis where they overlap) and the larger axis gap below that
    /// space; touching (no gap on either axis) is a short, handled elsewhere.
    fn eol_end_to_end(&self, layer: usize, (k, i): (usize, usize), r: &EolRule) -> bool {
        let (e1, e2) = (self.segs[layer][k], self.segs[layer][i]);
        let rect = |s: &Seg| Rect::new(s.from.0.min(s.to.0), s.from.1.min(s.to.1), s.from.0.max(s.to.0), s.from.1.max(s.to.1));
        let (mut r1, mut r2) = (rect(&e1), rect(&e2));
        let space = match r.end_to_end {
            Some(ete) if self.is_eol_edge(layer, i, r) => {
                if let Some(x) = r.extension {
                    bloat_along(&mut r1, &e1, x);
                    bloat_along(&mut r2, &e2, x);
                }
                ete
            }
            _ => {
                bloat_along(&mut r1, &e1, r.within);
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
        // A SPACING … RANGE rule holding the rectangle's own width widens the window.
        let max_spc = self.tech.layers[layer].spacing_ranges.iter().filter(|c| c.in_range(width(&s.rect))).fold(max_spc, |m, c| m.max(c.min_spacing));
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
        self.required_spacing_range(layer, r1, r2, prl).0
    }

    /// `checkMetalSpacing_prl_getReqSpcVal`: the table's spacing, then — two different owners —
    /// a SPACING … RANGE rule holding either width that asks more (and so names the marker).
    fn required_spacing_range(&self, layer: usize, r1: &Shape, r2: &Shape, prl: i32) -> (i32, bool) {
        let l = &self.tech.layers[layer];
        // A blockage: the layer's width (the obstruction's own DESIGNRULEWIDTH over that); its own
        // SPACING is the answer outright (the first shape's, then the second's).
        let mut ws = [0; 2];
        for (k, s) in [r1, r2].into_iter().enumerate() {
            let o = self.owner(s.net);
            ws[k] = width(&s.rect);
            if o.is_blockage() {
                if !self.tech.obs_own_width {
                    ws[k] = l.width;
                }
                if o.design_rule_width() != -1 {
                    ws[k] = o.design_rule_width();
                }
                if o.min_spacing() != -1 {
                    return (o.min_spacing(), false);
                }
            }
        }
        let (w1, w2) = (ws[0], ws[1]);
        let mut req = l.spacing.as_ref().map_or(0, |t| t.find(w1, w2, prl));
        let mut is_range = false;
        if r1.net != r2.net {
            for c in &l.spacing_ranges {
                if (c.in_range(w1) || c.in_range(w2)) && c.min_spacing > req {
                    is_range = true;
                    req = c.min_spacing;
                }
            }
        }
        (req, is_range)
    }

    /// Apart but closer than the table allows is a violation only when the gap lies between TRUE
    /// boundary edges of the two owners (both sides of it), and some trial shape reaches it.
    #[allow(clippy::too_many_arguments)]
    fn spacing_table(&mut self, layer: usize, r1: Shape, r2: Shape, marker: Rect, prl: i32, dist_x: i32, dist_y: i32, check_poly_edge: bool) {
        if r1.fixed && r2.fixed {
            return;
        }
        let (req0, is_range) = self.required_spacing_range(layer, &r1, &r2, prl);
        let mut req = i64::from(req0);
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
        // The marker's constraint (`checkMetalSpacing_prl`): the range rule that raised the
        // requirement, else the layer's minimum spacing rule, whichever kind it is.
        let rule = match &self.tech.layers[layer].spacing {
            _ if is_range => Rule::SpacingRange,
            Some(MinSpacing::TwoWidths(_)) => Rule::SpacingTableTw,
            _ => Rule::MetalSpacing,
        };
        self.add_marker_of(rule, layer, marker, (r1.net, r1.rect, r1.fixed), (r2.net, r2.rect, r2.fixed));
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
            // `checkCutSpacing_main` reads the default inter-layer table of the layer two above too,
            // while that layer is below the top routing layer.
            let above = (layer + 2 < self.top_routing_layer && layer + 2 < self.tech.layers.len()).then(|| self.tech.layers[layer + 2].cut_table_inter.clone()).flatten();
            if l.kind != LayerKind::Cut || (l.cut_spacing.is_none() && l.cut_table.is_none() && l.cut_table_inter.is_none() && above.is_none() && l.cut_enclosures.is_empty() && l.lef58_cut_spacing.is_empty() && l.orth_spacing.is_empty()) {
                continue;
            }
            let (table, same_net, inter) = (l.cut_table.clone(), l.cut_table_same_net.clone(), l.cut_table_inter.clone());
            for net in 0..self.nets.len() {
                if !self.checks_from(net) {
                    continue;
                }
                let mine: Vec<usize> = (0..self.shapes[layer].len()).filter(|&k| self.alive[layer][k] && self.shapes[layer][k].net == net).collect();
                // `checkCutSpacing_main(rect)`: per cut, the plain rule, then the LEF58 rules
                // (the different-net list; no same-net rule is modelled, so none skips a
                // different net), then the LEF58 tables: SAMENET, then different-net, then the
                // default inter-layer table of this layer, then that of the layer two above,
                // then SPACINGTABLE ORTHOGONAL; then (`checkCutSpacing`) the LEF58 enclosure.
                for k in mine {
                    if let Some(spc) = self.tech.layers[layer].cut_spacing {
                        let q = bloat(&self.shapes[layer][k].rect, spc);
                        for o in self.query(layer, &q) {
                            self.cut_pair(layer, k, o, spc);
                        }
                    }
                    for i in 0..self.tech.layers[layer].lef58_cut_spacing.len() {
                        self.lef58_cut_spacing_main(layer, k, i);
                    }
                    if let Some(t) = &same_net {
                        self.cut_spacing_table(layer, k, t, CutTableKind::SameNet);
                    }
                    if let Some(t) = &table {
                        self.cut_spacing_table(layer, k, t, CutTableKind::DiffNet);
                    }
                    if let Some(t) = &inter {
                        self.cut_spacing_table(layer, k, &t.table, CutTableKind::Inter { table_layer: layer, second: t.second_layer, no_stack: t.no_stack });
                    }
                    if let Some(t) = &above {
                        self.cut_spacing_table(layer, k, &t.table, CutTableKind::Inter { table_layer: layer + 2, second: t.second_layer, no_stack: t.no_stack });
                    }
                    self.cut_spacing_table_orthogonal(layer, k);
                    self.lef58_enclosure(layer, k);
                }
            }
        }
    }

    /// `checkCutSpacingTableOrthogonal(rect)`: per (WITHIN, SPACING) row in order, every maximal
    /// rectangle (any owner, the cut itself aside) touching the cut grown by WITHIN, in the
    /// tree's order, is paired with it by [`Self::cut_spacing_table_orthogonal_helper`].
    fn cut_spacing_table_orthogonal(&mut self, layer: usize, k: usize) {
        let rows = self.tech.layers[layer].orth_spacing.clone();
        for (within, spacing) in rows {
            let q = bloat(&self.shapes[layer][k].rect, within);
            for o in self.query(layer, &q) {
                if o != k {
                    self.cut_spacing_table_orthogonal_helper(layer, k, o, spacing);
                }
            }
        }
    }

    /// `checkCutSpacingTableOrthogonal_helper(rect1, rect2, spacing)`: a pair that runs parallel
    /// on exactly ONE axis (`getRectsPrl`: no run on either, or a run on both, exits). The gap
    /// between them (`generalized_intersect`) and both cuts, each grown ACROSS the run by the
    /// spacing (along x when they overlap in x), make a region; its maximal rectangles are queried
    /// in order, every result in the tree's order (one cut may come back from several). A third
    /// cut, not all three fixed, whose interior meets a query rectangle marks: the box between
    /// the gap and the third cut, aggressor the third cut, victims the pair, sources all three.
    /// Upstream rule: each victim's box takes its RIGHT edge for its top (`ury = gtl::xh(...)`),
    /// an odb box then normalised; copied as written.
    fn cut_spacing_table_orthogonal_helper(&mut self, layer: usize, k1: usize, k2: usize, spacing: i32) {
        let (s1, s2) = (self.shapes[layer][k1], self.shapes[layer][k2]);
        let (r1, r2) = (s1.rect, s2.rect);
        let (prl_x, prl_y) = rects_prl(&r1, &r2);
        if prl_x.max(prl_y) <= 0 || prl_x.min(prl_y) > 0 {
            return;
        }
        let prl_rect = generalized_intersect(&r1, &r2);
        let across_x = prl_x > 0;
        let grow = |r: &Rect| if across_x { Rect { xl: r.xl - spacing, xh: r.xh + spacing, ..*r } } else { Rect { yl: r.yl - spacing, yh: r.yh + spacing, ..*r } };
        let mut set = Polygon90Set::new();
        for r in [grow(&prl_rect), grow(&r1), grow(&r2)] {
            set.insert_rect(r);
        }
        let query_rects = set.max_rectangles();
        let results: Vec<usize> = query_rects.iter().flat_map(|q| self.query(layer, q)).collect();
        for o in results {
            if o == k1 || o == k2 {
                continue;
            }
            let s3 = self.shapes[layer][o];
            if s1.fixed && s2.fixed && s3.fixed {
                continue;
            }
            if !query_rects.iter().any(|q| strictly_intersects(q, &s3.rect)) {
                continue;
            }
            let bbox = generalized_intersect(&prl_rect, &s3.rect);
            let mut owners = vec![self.owner(s3.net).clone(), self.owner(s1.net).clone(), self.owner(s2.net).clone()];
            owners.sort();
            owners.dedup();
            if self.seen.insert((bbox, layer, Rule::SpacingTableOrth, owners.clone())) {
                // ⛔ Divergence D5 (register; OpenROAD issue #12198): each victim box is its cut,
                // as the reference patched with vyges-openroad-patches upstream-fix 0011 records
                // it; the unpatched reference takes the top from the cut's RIGHT edge
                // (`ury = gtl::xh`, a typo).
                let side = |s: &Shape| (self.owner(s.net).clone(), layer, s.rect, s.fixed);
                let (victim, second_victim) = (side(&s1), side(&s2));
                let aggressor = (self.owner(s3.net).clone(), layer, s3.rect, s3.fixed);
                self.markers.push(Marker { rule: Rule::SpacingTableOrth, layer, bbox, owners, victim: Some(victim), aggressor: Some(aggressor), second_victim: Some(second_victim) });
            }
        }
    }

    /// `checkLef58CutSpacing_main(rect, con, skipDiffNet)` for rule `i`: every maximal rectangle
    /// (any owner) touching the cut grown by the rule's `getMaxSpcVal` (its spacing, or with
    /// ADJACENTCUTS the larger of spacing and WITHIN), on the second layer when the rule has one,
    /// else on the cut's own layer, in the tree's order, is paired with the cut.
    fn lef58_cut_spacing_main(&mut self, layer: usize, k: usize, i: usize) {
        let (ql, max_spc) = match self.tech.layers[layer].lef58_cut_spacing[i] {
            Lef58CutSpacing::Layer(con) => (con.second_layer, con.spacing),
            Lef58CutSpacing::AdjacentCuts(con) => (layer, con.spacing.max(con.within)),
        };
        let q = bloat(&self.shapes[layer][k].rect, max_spc);
        for o in self.query(ql, &q) {
            self.lef58_cut_spacing_pair(layer, k, o, i);
        }
    }

    /// `checkLef58CutSpacing_main(rect1, rect2, con)`: the cut itself is skipped (the same-net
    /// exit cannot fire: no SAMENET rule is modelled); a rule with a second layer goes to the
    /// LAYER branch, else one with adjacent cuts to the ADJACENTCUTS branch.
    fn lef58_cut_spacing_pair(&mut self, layer: usize, k: usize, o: usize, i: usize) {
        match self.tech.layers[layer].lef58_cut_spacing[i] {
            Lef58CutSpacing::Layer(_) => self.lef58_cut_spacing_spc_layer(layer, k, o, i),
            Lef58CutSpacing::AdjacentCuts(con) => {
                if o != k {
                    self.lef58_cut_spacing_spc_adj_cut(layer, k, o, i, con);
                }
            }
        }
    }

    /// `checkLef58CutSpacing_spc_adjCut` (rect1 the cut `k`, rect2 the cut `o`): skipped unless
    /// the first cut has the rule's adjacent cuts, or (TWOCUTS) both cuts have the two-cuts count;
    /// then cuts closer than the spacing (squared, centre to centre with CENTERTOCENTER, else
    /// edge to edge), not both fixed, give a marker on the cut layer, its box between the two
    /// (`generalized_intersect`), victim the first, aggressor the second. No net test: cuts of one
    /// net are judged too.
    fn lef58_cut_spacing_spc_adj_cut(&mut self, layer: usize, k: usize, o: usize, i: usize, con: AdjacentCuts) {
        let mut skip = true;
        if self.lef58_cut_spacing_has_adj_cuts(layer, k, &con, con.adjacent_cuts, con.no_prl) {
            skip = false;
        }
        if let Some(two) = con.two_cuts {
            if self.lef58_cut_spacing_has_adj_cuts(layer, k, &con, two, false) && self.lef58_cut_spacing_has_adj_cuts(layer, o, &con, two, false) {
                skip = false;
            }
        }
        if skip {
            return;
        }
        let (v1, v2) = (self.shapes[layer][k], self.shapes[layer][o]);
        if cut_d2(&v1.rect, &v2.rect, con.center_to_center) >= i64::from(con.spacing).pow(2) {
            return;
        }
        if v1.fixed && v2.fixed {
            return;
        }
        let marker = generalized_intersect(&v1.rect, &v2.rect);
        self.add_marker_of(Rule::Lef58CutSpacing(i as u16), layer, marker, (v1.net, v1.rect, v1.fixed), (v2.net, v2.rect, v2.fixed));
    }

    /// `checkLef58CutSpacing_spc_hasAdjCuts` (`no_prl` the rule's NOPRL) and, with `no_prl` false
    /// and `need` the TWOCUTS value, `checkLef58CutSpacing_spc_hasTwoCuts_helper`: over the
    /// maximal rectangles on the cut's layer (any owner, the cut itself among them) touching the
    /// cut grown by WITHIN, count those closer than WITHIN (squared, as the spacing is measured)
    /// whose class (by width and length) is the rule's — with NOPRL only those with no parallel
    /// run on either axis (abutting counts as none), or at distance 0; the count starts at -1, so
    /// the cut counts itself when it is of the class; at least `need` passes.
    fn lef58_cut_spacing_has_adj_cuts(&self, layer: usize, k: usize, con: &AdjacentCuts, need: i32, no_prl: bool) -> bool {
        let r = self.shapes[layer][k].rect;
        let within2 = i64::from(con.within).pow(2);
        let mut cnt = -1;
        for p in self.query(layer, &bloat(&r, con.within)) {
            let pr = self.shapes[layer][p].rect;
            let d2 = cut_d2(&r, &pr, con.center_to_center);
            if d2 >= within2 {
                continue;
            }
            if self.tech.layers[layer].cut_class_of(pr.dx().min(pr.dy()), pr.dx().max(pr.dy())) != con.class {
                continue;
            }
            if no_prl {
                let none = (pr.xl >= r.xh || pr.xh <= r.xl) && (pr.yl >= r.yh || pr.yh <= r.yl);
                if none || d2 == 0 {
                    cnt += 1;
                }
            } else {
                cnt += 1;
            }
        }
        cnt >= need
    }

    /// `checkLef58CutSpacing_spc_layer`: a cut of the rule's class goes to the rule's corner branch,
    /// CONCAVECORNER first, as the source tests them.
    fn lef58_cut_spacing_spc_layer(&mut self, layer: usize, k: usize, o: usize, i: usize) {
        let Lef58CutSpacing::Layer(con) = self.tech.layers[layer].lef58_cut_spacing[i] else { return };
        let r1 = self.shapes[layer][k].rect;
        if self.tech.layers[layer].cut_class_of(r1.dx().min(r1.dy()), r1.dx().max(r1.dy())) != con.class {
            return;
        }
        match con.corner {
            CutCorner::Concave => self.lef58_cut_spacing_spc_layer_concave(layer, k, o, i),
            CutCorner::NonEolConvex { eol_width, min_length } => self.lef58_cut_spacing_spc_layer_convex(layer, k, o, i, eol_width, min_length),
        }
    }

    /// `checkLef58CutSpacing_spc_layer`, its CONCAVECORNER branch: a cut wholly inside (edges may
    /// meet) the metal rectangle `o` is checked against every polygon edge of the second layer
    /// (any owner) touching that rectangle, in the tree's order, by the edge's starting corner: a
    /// CONCAVE corner with its x on the rectangle's left or right line, or its y on the bottom or
    /// top line, closer to the cut (squared Euclidean, rectangle to point; 0 inside) than the
    /// spacing, not with both cut and corner fixed, gives a marker
    /// (`lef58_cut_spacing_marker`).
    fn lef58_cut_spacing_spc_layer_concave(&mut self, layer: usize, k: usize, o: usize, i: usize) {
        let Lef58CutSpacing::Layer(con) = self.tech.layers[layer].lef58_cut_spacing[i] else { return };
        let (sl, cut, metal) = (con.second_layer, self.shapes[layer][k], self.shapes[con.second_layer][o]);
        let (r1, r2) = (cut.rect, metal.rect);
        if !contains(&r2, &r1) {
            return;
        }
        let req = i64::from(con.spacing) * i64::from(con.spacing);
        for e in self.query_segs(sl, &r2) {
            let Some(c) = self.corner(sl, e) else { continue };
            if c.convex {
                continue;
            }
            if (c.x != r2.xl && c.x != r2.xh) && (c.y != r2.yl && c.y != r2.yh) {
                continue;
            }
            let p = Rect::new(c.x, c.y, c.x, c.y);
            let bbox = generalized_intersect(&p, &r1);
            let (dx, dy) = (i64::from(gap((r1.xl, r1.xh), (c.x, c.x))), i64::from(gap((r1.yl, r1.yh), (c.y, c.y))));
            if dx * dx + dy * dy >= req {
                continue;
            }
            if cut.fixed && c.fixed {
                continue;
            }
            self.lef58_cut_spacing_marker(layer, k, o, i, &c, bbox);
        }
    }

    /// `checkLef58CutSpacing_spc_layer`, its NONEOLCONVEXCORNER branch: a cut wholly inside the
    /// metal rectangle `o` is checked against every polygon edge of the second layer (any owner)
    /// touching that rectangle, in the tree's order, by the edge's starting corner: a CONVEX
    /// corner, unless one of its two edges is a line end (`is_convex_line_end`) shorter than the rule's
    /// end-of-line width; then the corner-to-cut box (`generalized_intersect`) of width dx and
    /// height dy must lie inside the triangle `edge_x * dy + edge_y * dx < edge_x * edge_y`, each
    /// leg the spacing clipped to the corner's edge along that axis; not with both cut and corner
    /// fixed. No boundary-line test here (the concave branch's): a corner off the rectangle whose
    /// edge touches it is checked too.
    fn lef58_cut_spacing_spc_layer_convex(&mut self, layer: usize, k: usize, o: usize, i: usize, eol_width: i32, min_length: Option<i32>) {
        let Lef58CutSpacing::Layer(con) = self.tech.layers[layer].lef58_cut_spacing[i] else { return };
        let (sl, cut, metal) = (con.second_layer, self.shapes[layer][k], self.shapes[con.second_layer][o]);
        let (r1, r2) = (cut.rect, metal.rect);
        if !contains(&r2, &r1) {
            return;
        }
        for e in self.query_segs(sl, &r2) {
            let Some(c) = self.corner(sl, e) else { continue };
            if !c.convex {
                continue;
            }
            let next = self.segs[sl][e];
            let prev = self.segs[sl][next.prev];
            // The previous edge ends a line when the corner before it is convex too; the next edge
            // when the corner after it is.
            if self.is_convex_line_end(sl, next.prev, min_length) && prev.len() < eol_width {
                continue;
            }
            if self.is_convex_line_end(sl, e, min_length) && next.len() < eol_width {
                continue;
            }
            let p = Rect::new(c.x, c.y, c.x, c.y);
            let bbox = generalized_intersect(&p, &r1);
            let (dx, dy) = (i64::from(bbox.dx()), i64::from(bbox.dy()));
            let (edge_x, edge_y) = if matches!(next.dir(), EdgeDir::N | EdgeDir::S) {
                (con.spacing.min(prev.len()), con.spacing.min(next.len()))
            } else {
                (con.spacing.min(next.len()), con.spacing.min(prev.len()))
            };
            let (ex, ey) = (i64::from(edge_x), i64::from(edge_y));
            if ex * dy + ey * dx >= ex * ey {
                continue;
            }
            if cut.fixed && c.fixed {
                continue;
            }
            self.lef58_cut_spacing_marker(layer, k, o, i, &c, bbox);
        }
    }

    /// The NONEOLCONVEXCORNER branch's line-end test for polygon edge `k` of `layer`: both of its
    /// corners convex, and with MINLENGTH both edges beside it at least that long.
    fn is_convex_line_end(&self, layer: usize, k: usize, min_length: Option<i32>) -> bool {
        let e = self.segs[layer][k];
        let (before, after) = (self.segs[layer][e.prev], self.segs[layer][e.next]);
        let convex_ends = orientation(&before, &e) == 1 && orientation(&e, &after) == 1;
        convex_ends && min_length.is_none_or(|m| before.len() >= m && after.len() >= m)
    }

    /// The marker of a LEF58 cut spacing LAYER rule (both branches): on the cut layer, the box
    /// spanning the cut and the corner; victim the cut, aggressor the RECTANGLE's owner with the
    /// corner as a point on the second layer; once per rule, box and owners.
    fn lef58_cut_spacing_marker(&mut self, layer: usize, k: usize, o: usize, i: usize, c: &Corner, bbox: Rect) {
        let Lef58CutSpacing::Layer(con) = self.tech.layers[layer].lef58_cut_spacing[i] else { return };
        let sl = con.second_layer;
        let (cut, metal) = (self.shapes[layer][k], self.shapes[sl][o]);
        let rule = Rule::Lef58CutSpacing(i as u16);
        let mut owners = vec![self.owner(cut.net).clone(), self.owner(metal.net).clone()];
        owners.sort();
        owners.dedup();
        if self.seen.insert((bbox, layer, rule, owners.clone())) {
            let victim = (self.owner(cut.net).clone(), layer, cut.rect, cut.fixed);
            let aggressor = (self.owner(metal.net).clone(), sl, Rect::new(c.x, c.y, c.x, c.y), c.fixed);
            self.markers.push(Marker { rule, layer, bbox, owners, victim: Some(victim), aggressor: Some(aggressor), second_victim: None });
        }
    }

    /// `checkLef58Enclosure_main(rect)`: a routed cut of a class with LEF58 enclosure rules below
    /// and/or above: the same-owner metal rectangle there holding the cut (the widest, the first of
    /// equals) must overhang it as one of the rules for its width allows.
    fn lef58_enclosure(&mut self, layer: usize, k: usize) {
        let v = self.shapes[layer][k];
        if v.fixed || self.tech.layers[layer].cut_enclosures.is_empty() {
            return;
        }
        let c = self.tech.layers[layer].cut_class_of(v.rect.dx().min(v.rect.dy()), v.rect.dx().max(v.rect.dy()));
        let rules = |above: bool| -> Vec<crate::tech::CutEnclosure> {
            self.tech.layers[layer].cut_enclosures.iter().copied().filter(|e| !e.eol && e.class == c && if above { !e.below_only } else { !e.above_only }).collect()
        };
        let (below, above) = (rules(false), rules(true));
        let enclosing = |l: usize| -> Option<Shape> {
            let mut best: Option<Shape> = None;
            for o in self.query(l, &v.rect) {
                let s = self.shapes[l][o];
                if s.net != v.net || !(s.rect.xl <= v.rect.xl && s.rect.yl <= v.rect.yl && v.rect.xh <= s.rect.xh && v.rect.yh <= s.rect.yh) {
                    continue;
                }
                if best.is_none_or(|b| width(&s.rect) > width(&b.rect)) {
                    best = Some(s);
                }
            }
            best
        };
        // (Both looked up first: a check only adds markers, the queries read shapes.)
        let below_enc = if below.is_empty() { None } else { enclosing(layer - 1) };
        let above_enc = if above.is_empty() { None } else { enclosing(layer + 1) };
        if let Some(e) = below_enc {
            self.lef58_enclosure_main(layer, &v, layer - 1, &e, &below);
        }
        if let Some(e) = above_enc {
            self.lef58_enclosure_main(layer, &v, layer + 1, &e, &above);
        }
    }

    /// `checkLef58Enclosure_main(via, enc)`: the cut's overhangs inside the metal (end along the
    /// cut's longer side), against the rules of the widest WIDTH not above the metal's width, in
    /// order; none satisfied gives a marker on the metal's layer, the cut's box. ⚠️ The source's
    /// sides record their boxes with the top at the RIGHT edge (`ury = xh`), kept. A cut with no
    /// enclosing metal, or no rule for the width, is skipped (the source reads a null there).
    fn lef58_enclosure_main(&mut self, layer: usize, v: &Shape, el: usize, e: &Shape, rules: &[crate::tech::CutEnclosure]) {
        let (vr, er) = (v.rect, e.rect);
        let mut side = (er.xh - vr.xh).min(vr.xl - er.xl);
        let mut end = (er.yh - vr.yh).min(vr.yl - er.yl);
        if vr.dx() > vr.dy() {
            std::mem::swap(&mut side, &mut end);
        }
        let w = width(&er);
        let Some(key) = rules.iter().map(|r| r.min_width).filter(|&mw| mw <= w).max() else { return };
        if rules.iter().filter(|r| r.min_width == key).any(|r| r.valid(end, side)) {
            return;
        }
        let owners = vec![self.owner(v.net).clone()];
        if self.seen.insert((vr, el, Rule::Lef58Enclosure, owners.clone())) {
            let victim = (self.owner(v.net).clone(), layer, Rect::new(vr.xl, vr.yl, vr.xh, vr.xh), v.fixed);
            let aggressor = (self.owner(e.net).clone(), el, Rect::new(er.xl, er.yl, er.xh, er.xh), e.fixed);
            self.markers.push(Marker { rule: Rule::Lef58Enclosure, layer: el, bbox: vr, owners, victim: Some(victim), aggressor: Some(aggressor), second_victim: None });
        }
    }

    /// `checkLef58CutSpacingTbl`: every cut within the largest spacing the cut's class can need —
    /// its END spacing for a square cut, the larger of END and SIDE otherwise — on the query layer,
    /// not both fixed, not of the cut's own pin. A table on the cut's layer (SAMENET or
    /// different-net) queries that layer. An inter-layer table queries its second layer when the
    /// cut is not on that layer (the cut is then the pair's FIRST, `isUpperVia`), else the
    /// table's own layer (the cut SECOND): the first of a pair is always on the table's side.
    fn cut_spacing_table(&mut self, layer: usize, k: usize, t: &CutSpacingTable, kind: CutTableKind) {
        let v = self.shapes[layer][k];
        let (w, l) = (v.rect.dx().min(v.rect.dy()), v.rect.dx().max(v.rect.dy()));
        let c = t.class_index(&self.tech.layers[layer], self.tech.layers[layer].cut_class_of(w, l));
        let max_spc = if w == l { t.max_spacing[c][0] } else { t.max_spacing[c][0].max(t.max_spacing[c][1]) };
        let (upper, ql) = match kind {
            CutTableKind::Inter { table_layer, second, .. } => {
                let upper = second != layer;
                (upper, if upper { second } else { table_layer })
            }
            _ => (true, layer),
        };
        for o in self.query(ql, &bloat(&v.rect, max_spc)) {
            let p = self.shapes[ql][o];
            // (A pin is one layer's: cuts on two layers are never one pin's.)
            if (p.fixed && v.fixed) || (ql == layer && (p.net, p.pin) == (v.net, v.pin)) {
                continue;
            }
            if upper {
                self.cut_spacing_table_pair((layer, k), (ql, o), t, kind);
            } else {
                self.cut_spacing_table_pair((ql, o), (layer, k), t, kind);
            }
        }
    }

    /// `checkLef58CutSpacingTbl_main`. On one layer: a SAMENET table judges only cuts of one net;
    /// the different-net table judges cuts of two nets, and cuts of one net too unless the layer
    /// has a SAMENET table to hand them to. A default inter-layer table judges cuts of two nets
    /// only. Classes are each cut's on its own layer. Overlapping cuts: on one layer a cut SHORT
    /// (unless the classes' largest spacing is 0); on two layers (that spacing not 0) a violation
    /// under NOSTACK, else unless one cut contains the other. Apart, each direction the second
    /// cut lies in is judged by `helper` until one violates.
    fn cut_spacing_table_pair(&mut self, (l1, k1): (usize, usize), (l2, k2): (usize, usize), t: &CutSpacingTable, kind: CutTableKind) {
        let (v1, v2) = (self.shapes[l1][k1], self.shapes[l2][k2]);
        match kind {
            CutTableKind::Inter { .. } => {
                if v1.net == v2.net {
                    return;
                }
            }
            _ => {
                if same_net_table_skips(kind == CutTableKind::SameNet, v1.net == v2.net, self.tech.layers[l1].cut_table_same_net.is_some()) {
                    return;
                }
            }
        }
        let (r1, r2) = (v1.rect, v2.rect);
        let class = |l: usize, r: &Rect| t.class_index(&self.tech.layers[l], self.tech.layers[l].cut_class_of(r.dx().min(r.dy()), r.dx().max(r.dy())));
        let (c1, c2) = (class(l1, &r1), class(l2, &r2));
        let marker = generalized_intersect(&r1, &r2);
        let (dx, dy) = (gap((r1.xl, r1.xh), (r2.xl, r2.xh)), gap((r1.yl, r1.yh), (r2.yl, r2.yh)));
        let dist2 = i64::from(dx).pow(2) + i64::from(dy).pow(2);
        let mut viol = false;
        if dist2 == 0 {
            if t.max_pair_spacing(c1, c2) == 0 {
                return;
            }
            match kind {
                // `checkCutSpacing_short`.
                CutTableKind::SameNet | CutTableKind::DiffNet => {
                    if !(v1.fixed && v2.fixed) {
                        self.add_marker_of(Rule::Short, l1, marker, (v1.net, r1, v1.fixed), (v2.net, r2, v2.fixed));
                    }
                }
                CutTableKind::Inter { no_stack, .. } => viol = no_stack || !(r1 == r2 || contains(&r1, &r2) || contains(&r2, &r1)),
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
        let inter = matches!(kind, CutTableKind::Inter { .. });
        if !viol && (up || down) {
            viol = self.cut_table_helper(l2, (&r1, &r2), (c1, c2), if up { EdgeDir::N } else { EdgeDir::S }, (dist2, c2c2), (prl_valid, prl), t, inter);
        }
        if !viol && (right || left) {
            viol = self.cut_table_helper(l2, (&r1, &r2), (c1, c2), if right { EdgeDir::E } else { EdgeDir::W }, (dist2, c2c2), (prl_valid, prl), t, inter);
        }
        if viol {
            // ⚠️ The reference records each side's rectangle with its x-high as its y-high too.
            let side = |r: &Rect| Rect::new(r.xl, r.yl, r.xh, r.xh);
            if inter {
                self.add_marker_two_layers(Rule::Lef58CutSpacingTableInter, marker, (v1.net, l1, side(&r1), v1.fixed), (v2.net, l2, side(&r2), v2.fixed));
            } else {
                self.add_marker_of(Rule::Lef58CutSpacingTable, l1, marker, (v1.net, side(&r1), v1.fixed), (v2.net, side(&r2), v2.fixed));
            }
        }
    }

    /// `checkLef58CutSpacingTbl_helper`: whether the pair is too close with the second cut in
    /// direction `dir`. Each cut's facing edge is a SIDE when it is the cut's long edge. NOPRL with
    /// CENTERANDEDGE: centre to centre below the larger spacing, or edge to edge below the smaller;
    /// the same class (a table on one layer only, `inter` false), exactly aligned (run equal to the
    /// cut's own width across `dir`, unless the table is limited to that direction), with an
    /// EXACTALIGNED spacing: edge to edge below it;
    /// otherwise the table's first value, or its second where the run is valid (back to the first
    /// under PRLFORALIGNEDCUT when no metal edge on the layer above the SECOND cut's (`l2`) lies on
    /// its facing edge), centre
    /// to centre for CENTERTOCENTER (or CENTERANDEDGE where it is the larger value), else edge to
    /// edge.
    #[allow(clippy::too_many_arguments)]
    fn cut_table_helper(&self, l2: usize, (r1, r2): (&Rect, &Rect), (c1, c2): (usize, usize), dir: EdgeDir, (dist2, c2c2): (i64, i64), (prl_valid, prl): (bool, i32), t: &CutSpacingTable, inter: bool) -> bool {
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
        if c1 == c2 && !inter {
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
            if l2 + 1 >= self.segs.len() || self.query_segs(l2 + 1, &e).is_empty() {
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
        Marker { rule: Rule::Short, layer: 4, bbox: r, owners: vec![o.clone()], victim: Some((o.clone(), 4, r, false)), aggressor: Some((o, 4, r, false)), second_victim: None }
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
                Layer { name: "l2".into(), kind: LayerKind::Routing, dir: Dir::Vertical, width: 170, min_width: 170, pitch: 480, wrong_way_width: 170, spacing: Some(table(vec![(0, 170)]).into()), cut_spacing: None, cut_classes: vec![], cut_table: None, cut_table_same_net: None, cut_table_inter: None, cut_enclosures: vec![], lef58_cut_spacing: vec![], eol: vec![], lef58_eol: vec![], eol_keepout: vec![], corner_spacing: vec![], min_area: 0, min_enclosed_areas: vec![], rect_only: false, right_way_on_grid_only: false, num_masks: 1, min_cuts: vec![], mt_safe_dist: 0, min_step: None, lef58_min_step: vec![], spacing_ranges: vec![], lef58_area: Default::default(), orth_spacing: vec![], lef58_max_spacing: vec![] },
                Layer { name: "c3".into(), kind: LayerKind::Cut, width: 170, cut_spacing: Some(190), ..Layer::default() },
                Layer { name: "l4".into(), kind: LayerKind::Routing, dir: Dir::Horizontal, width: 140, min_width: 140, pitch: 370, wrong_way_width: 140, spacing: Some(table(vec![(0, 140), (3000, 280)]).into()), cut_spacing: None, cut_classes: vec![], cut_table: None, cut_table_same_net: None, cut_table_inter: None, cut_enclosures: vec![], lef58_cut_spacing: vec![], eol: vec![], lef58_eol: vec![], eol_keepout: vec![], corner_spacing: vec![], min_area: 0, min_enclosed_areas: vec![], rect_only: false, right_way_on_grid_only: false, num_masks: 1, min_cuts: vec![], mt_safe_dist: 0, min_step: None, lef58_min_step: vec![], spacing_ranges: vec![], lef58_area: Default::default(), orth_spacing: vec![], lef58_max_spacing: vec![] },
            ],
            obs_own_width: false,
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

    /// `markers`, on `tech()` with layer 4's LEF58_AREA rules (in io's lists), a check box, the
    /// ignore flag and layer 4's plain minimum area.
    fn markers_lef58_area(shapes: &[(Owner, usize, Rect, bool)], rules: &[crate::tech::AreaRule], drc: Option<Rect>, ignore: bool, min_area: i64) -> Vec<Marker> {
        let mut t = tech();
        for &r in rules {
            t.layers[4].lef58_area.add(r);
        }
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

    /// Rule (`checkMetalShape_main`, after `checkMetalShape_minStep`; `addMarker` keeps a marker
    /// once per box, layer, CONSTRAINT and sources): every kept LEF58_MINSTEP rule checks the pin,
    /// so two MINADJACENTLENGTH rules mark the bump's three boxes twice, each named after its rule;
    /// the minimum-step markers the router's patch pass reads (`checkMetalShape(true)`) hold them
    /// too (witness `aa`).
    #[test]
    fn each_lef58_min_step_rule_marks_its_own_boxes() {
        let mut t = tech();
        let rule = |min_adjacent_length| crate::tech::Lef58MinStep { min_step_length: 50, max_edges: 1, min_adjacent_length, no_adj_eol: -1, eol_width: -1, except_rectangle: false };
        t.layers[4].lef58_min_step = vec![rule(60), rule(70)];
        let mut w = Worker::new(&t);
        w.add(&net("a"), 4, Rect::new(980, 1000, 2020, 1040), false);
        w.add(&net("a"), 4, Rect::new(1395, 1000, 1435, 1070), false);
        w.init();
        let steps = |m: &[Marker]| m.iter().filter(|m| matches!(m.rule, Rule::Lef58MinStep(_))).map(|m| m.rule).collect::<Vec<_>>();
        let want = [Rule::Lef58MinStep(0), Rule::Lef58MinStep(0), Rule::Lef58MinStep(0), Rule::Lef58MinStep(1), Rule::Lef58MinStep(1), Rule::Lef58MinStep(1)];
        assert_eq!(steps(w.run()), want);
        assert_eq!(steps(&w.min_step_markers()), want);
    }

    /// Rule (`checkMetalShape_lef58Area`, the marker pass, after `checkMetalShape_minEnclosedArea`):
    /// a polygon breaking a LEF58_AREA rule is a marker on its bounding box, named after the rule
    /// broken (a marker per constraint). None when the box is not WHOLLY inside the check box, when
    /// one of its edges is on the owner's fixed shapes, or when the check is ignored (pin access:
    /// the reference reads `ignoreMinArea_` here too). Independent of the plain minimum area: a
    /// polygon below both gets both markers (witness: `a`, `t`, `fx`).
    #[test]
    fn lef58_area_marks_a_polygon_below_the_rule_that_judges_it() {
        use crate::tech::AreaRule;
        let a = [AreaRule { index: 0, area: 100_000, rect_width: 0, except_rectangle: false }];
        let stub = [(net("a"), 4, Rect::new(0, 0, 300, 200), false)];
        let m = markers_lef58_area(&stub, &a, Some(Rect::new(-1000, -1000, 1000, 1000)), false, 0);
        assert_eq!(m.iter().map(|m| (m.rule, m.bbox)).collect::<Vec<_>>(), [(Rule::Lef58Area(0), Rect::new(0, 0, 300, 200))]);
        assert!(markers_lef58_area(&stub, &a, Some(Rect::new(-1000, -1000, 150, 1000)), false, 0).is_empty(), "the check box must hold it");
        assert!(markers_lef58_area(&stub, &a, None, true, 0).is_empty(), "ignored");
        let fixed = [(net("a"), 4, Rect::new(0, 0, 150, 200), true), (net("a"), 4, Rect::new(150, 0, 300, 200), false)];
        assert!(markers_lef58_area(&fixed, &a, None, false, 0).is_empty(), "a fixed edge");
        let both = markers_lef58_area(&stub, &a, None, false, 80_000);
        assert_eq!(both.iter().map(|m| m.rule).collect::<Vec<_>>(), [Rule::MinArea, Rule::Lef58Area(0)]);
        // A tie: the later rule (EXCEPTRECTANGLE) is tried first; a rectangle passes to rule 0, an
        // L is marked by rule 1 — two constraints, two marker identities.
        let t = [AreaRule { index: 0, area: 100_000, rect_width: 0, except_rectangle: false }, AreaRule { index: 1, area: 100_000, rect_width: 0, except_rectangle: true }];
        assert_eq!(markers_lef58_area(&stub, &t, None, false, 0)[0].rule, Rule::Lef58Area(0));
        let ell = [(net("a"), 4, Rect::new(0, 0, 300, 100), false), (net("a"), 4, Rect::new(200, 0, 300, 300), false)];
        assert_eq!(area_rules(&markers_lef58_area(&ell, &t, None, false, 0)), [Rule::Lef58Area(1)]);
        // A RECTWIDTH rule judges a narrow rectangle alone; a ring (four outer vertices, a hole)
        // is not one maximal rectangle, so it falls to the other rules.
        let w = [AreaRule { index: 0, area: 100_000, rect_width: 200, except_rectangle: false }, AreaRule { index: 1, area: 300_000, rect_width: 0, except_rectangle: false }];
        assert_eq!(markers_lef58_area(&stub, &w, None, false, 0)[0].rule, Rule::Lef58Area(0));
        let ring = [(net("a"), 4, Rect::new(0, 0, 500, 100), false), (net("a"), 4, Rect::new(0, 400, 500, 500), false), (net("a"), 4, Rect::new(0, 0, 100, 500), false), (net("a"), 4, Rect::new(400, 0, 500, 500), false)];
        assert_eq!(area_rules(&markers_lef58_area(&ring, &w, None, false, 0)), [Rule::Lef58Area(1)]);
    }

    /// The LEF58_AREA rules among `m` (the thin test shapes also break the minimum width).
    fn area_rules(m: &[Marker]) -> Vec<Rule> {
        m.iter().map(|m| m.rule).filter(|r| matches!(r, Rule::Lef58Area(_))).collect()
    }

    /// Rule (`checkMetalShape(true)`: `checkMetalShape_minArea` then `checkMetalShape_lef58Area`
    /// with patching): only the TARGET's pins are patched (none without a target); both tests
    /// read the pin as it stands, so a pin below the plain minimum area AND a LEF58_AREA rule
    /// gets two patches, the minimum area's first; a pin with a fixed edge gets none.
    #[test]
    fn the_patch_pass_patches_the_target_for_each_area_rule_it_breaks() {
        use crate::tech::AreaRule;
        let mut t = tech();
        t.layers[4].lef58_area.add(AreaRule { index: 0, area: 100_000, rect_width: 0, except_rectangle: false });
        t.layers[4].min_area = 80_000;
        let run = |shapes: &[(Owner, usize, Rect, bool)], target: Option<Owner>| {
            let mut w = Worker::new(&t);
            for (o, l, r, f) in shapes {
                w.add(o, *l, *r, *f);
            }
            w.init();
            w.target = target;
            w.area_patches()
        };
        let stub = [(net("a"), 4, Rect::new(0, 0, 300, 200), false), (net("b"), 4, Rect::new(0, 1000, 300, 1200), false)];
        let p = run(&stub, Some(net("a")));
        // Layer 4 is horizontal: the first vertical edge, the right one, grows east by
        // ceil((80000 - 60000) / 200 / 5) * 5 = 100, then by (100000 - 60000) / 200 = 200.
        assert_eq!(p.iter().map(|p| (p.layer, p.origin, p.offset)).collect::<Vec<_>>(), [(4, (300, 100), Rect::new(0, -100, 100, 100)), (4, (300, 100), Rect::new(0, -100, 200, 100))]);
        assert!(p.iter().all(|p| p.owner == net("a")), "net b is not the target");
        assert!(run(&stub, None).is_empty(), "no target: no patching");
        let fixed = [(net("a"), 4, Rect::new(0, 0, 150, 200), true), (net("a"), 4, Rect::new(150, 0, 300, 200), false)];
        assert!(run(&fixed, Some(net("a"))).is_empty());
    }

    /// The polygon edges of `slices` (one polygon, nothing fixed), ring order.
    fn ring(slices: &[Rect]) -> Vec<Seg> {
        let mut segs = Vec::new();
        polygon_segs(&mut segs, slices, &BTreeSet::new(), 0);
        segs
    }

    /// Rule (`checkMetalShape_addPatch`; the reference's trace on gcd_nangate45 with a LEF58_AREA
    /// rule): the patch grows from an edge running ACROSS the layer's direction, the first of the
    /// best in ring order (a rectangle's ring starts at its lower-left corner, running east), as
    /// wide as the edge (half each way from its midpoint) and out of the polygon by
    /// `ceil(gap / edge / grid) * grid`. On a vertical layer a rectangle grows down from its
    /// bottom edge — (27480, 28560)-(27620, 28840), gap 40800: origin (27550, 28560), box
    /// (-70, -300, 70, 0); on a horizontal layer east from its right edge — (27410, 134750)-
    /// (27770, 134890), gap 29600: origin (27770, 134820), box (0, -70, 220, 70).
    #[test]
    fn an_area_patch_grows_from_the_first_best_edge_across_the_layer() {
        let v = ring(&[Rect::new(27480, 28560, 27620, 28840)]);
        assert_eq!(area_patch_geometry(&v, true, 40_800, 10), Some(((27550, 28560), Rect::new(-70, -300, 70, 0))));
        let h = ring(&[Rect::new(27410, 134750, 27770, 134890)]);
        assert_eq!(area_patch_geometry(&h, false, 29_600, 10), Some(((27770, 134820), Rect::new(0, -70, 220, 70))));
        // 18400 / 140 / 10 = 13.14…: 14 grids.
        let h2 = ring(&[Rect::new(87830, 138670, 88270, 138810)]);
        assert_eq!(area_patch_geometry(&h2, false, 18_400, 10), Some(((88270, 138740), Rect::new(0, -70, 140, 70))));
        // An exact multiple stays: 42000 / 140 / 10 = 30.
        assert_eq!(area_patch_geometry(&v, true, 42_000, 10).map(|g| g.1), Some(Rect::new(-70, -300, 70, 0)));
        // An odd edge: each half truncated (141 / 2 = 70), the origin at the truncated midpoint.
        let odd = ring(&[Rect::new(0, 0, 141, 280)]);
        assert_eq!(area_patch_geometry(&odd, true, 1_000, 10), Some(((70, 0), Rect::new(-70, -10, 70, 0))));
    }

    /// Rule (`bestSuitable`): an edge with BOTH corners convex beats any edge without, however
    /// long; among the same kind only a strictly longer edge replaces the one held. A "+" (bars
    /// 1000 x 100 and 100 x 300) on a vertical layer: the long horizontal edges (450) each end at
    /// a concave corner, so the vertical bar's bottom (100, convex) is chosen, and its top (100,
    /// convex, later in the ring) does not replace it.
    #[test]
    fn an_area_patch_prefers_a_convex_edge_to_a_longer_one() {
        let plus = ring(&[Rect::new(0, 100, 1000, 200), Rect::new(450, 0, 550, 100), Rect::new(450, 200, 550, 300)]);
        assert_eq!(area_patch_geometry(&plus, true, 1_000, 10), Some(((500, 0), Rect::new(-50, -10, 50, 0))));
        // On a horizontal layer the vertical edges: the bar's two ends (100, convex) tie, and the
        // one earlier in the ring wins. Where a ring starts is the polygon formation's accident
        // (polygon_formation.rs); this one starts on the bar's left end, so the left end grows west.
        let left = plus.iter().position(|e| e.from == (0, 200) && e.to == (0, 100)).expect("left end");
        let right = plus.iter().position(|e| e.from == (1000, 100) && e.to == (1000, 200)).expect("right end");
        assert!(left < right, "ring order: {plus:?}");
        assert_eq!(area_patch_geometry(&plus, false, 1_000, 10), Some(((0, 150), Rect::new(-10, -50, 0, 50))));
    }

    /// Rule (`checkMetalShape_offGrid`, every routing layer): a maximal rectangle of one owner's
    /// polygon with any edge off the manufacturing grid (5 here) is a marker on that rectangle,
    /// victim and aggressor the owner, not fixed — unless the rectangle holds none of the owner's
    /// ROUTE shapes by area (`net->getPolygons(layer, false) & maxRect` empty): fixed metal off the
    /// grid, or a route that only touches it, is not the router's to fix.
    #[test]
    fn off_grid_marks_a_maximal_rectangle_off_the_grid_that_holds_a_route() {
        let run = |shapes: &[(Owner, usize, Rect, bool)]| {
            let t = tech();
            let mut w = Worker::new(&t);
            for (o, l, r, f) in shapes {
                w.add(o, *l, *r, *f);
            }
            w.init();
            w.run().to_vec()
        };
        let pin = Rect::new(0, 0, 1003, 140);
        // A route off the grid; a fixed pin off the grid with a route on the grid inside it.
        assert_eq!(boxes(&run(&[(net("a"), 4, pin, false)]), Rule::OffGrid), vec![pin]);
        let m = run(&[(net("a"), 4, pin, true), (net("a"), 4, Rect::new(0, 0, 500, 140), false)]);
        assert_eq!(boxes(&m, Rule::OffGrid), vec![pin]);
        let v = m.iter().find(|m| m.rule == Rule::OffGrid).and_then(|m| m.victim.clone()).expect("the marker has its victim");
        assert_eq!((v.2, v.3), (pin, false));
        // Fixed metal alone, and a route that only touches the off-grid rectangle: no marker.
        assert!(boxes(&run(&[(net("a"), 4, pin, true)]), Rule::OffGrid).is_empty());
        let touching = run(&[(net("a"), 4, Rect::new(0, 0, 1003, 400), true), (net("a"), 4, Rect::new(1003, 0, 1500, 140), false)]);
        assert!(boxes(&touching, Rule::OffGrid).is_empty());
        // On the grid: none.
        assert!(boxes(&run(&[(net("a"), 4, Rect::new(0, 0, 1000, 140), false)]), Rule::OffGrid).is_empty());
    }

    /// Rule (`checkMetalShape_main`'s order per pin): rect-only, THEN off-grid, THEN minimum
    /// enclosed area — the marker list keeps that order. A routed ring off the grid (a 3-unit
    /// step on a 5-unit grid) on a rect-only layer with a hole rule gives all three.
    #[test]
    fn off_grid_markers_come_after_rect_only_and_before_min_hole() {
        let mut t = tech();
        t.layers[4].rect_only = true;
        t.layers[4].min_enclosed_areas = vec![50_000];
        let mut w = Worker::new(&t);
        for r in [Rect::new(0, 0, 603, 200), Rect::new(0, 400, 603, 600), Rect::new(0, 200, 200, 400), Rect::new(403, 200, 603, 400)] {
            w.add(&net("a"), 4, r, false);
        }
        w.init();
        let rules: Vec<Rule> = w.run().iter().map(|m| m.rule).filter(|r| matches!(r, Rule::RectOnly | Rule::OffGrid | Rule::MinEnclosedArea)).collect();
        let first = |x: Rule| rules.iter().position(|&r| r == x).expect("each of the three rules fires");
        let last = |x: Rule| rules.iter().rposition(|&r| r == x).expect("each of the three rules fires");
        assert!(last(Rule::RectOnly) < first(Rule::OffGrid) && last(Rule::OffGrid) < first(Rule::MinEnclosedArea), "{rules:?}");
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

    /// Rule (`checkMetalSpacing_prl_getReqSpcVal`, `checkMetalSpacing_prl`): on a layer whose
    /// minimum spacing is a TWOWIDTHS table the pair is looked up on BOTH widths (a 300-wide and a
    /// 140-wide shape need 200 here, the wider width alone would ask 300) and the marker is named
    /// after that rule (`SpacingTableTw`), not `Metal Spacing`.
    #[test]
    fn a_two_widths_layer_needs_both_widths_and_names_its_marker() {
        let mut t = tech();
        t.layers[4].spacing = Some(MinSpacing::TwoWidths(crate::tech::TwoWidthsTable { widths: vec![0, 200], prls: vec![-1, -1], values: vec![vec![140, 200], vec![200, 300]] }));
        let run = |gap: i32| {
            let mut w = Worker::new(&t);
            w.add(&net("a"), 4, Rect::new(0, 0, 10000, 300), true);
            w.add(&net("b"), 4, Rect::new(0, 300 + gap, 10000, 440 + gap), false);
            w.init();
            w.run().iter().map(|m| m.rule).filter(|r| matches!(r, Rule::SpacingTableTw | Rule::MetalSpacing)).collect::<Vec<_>>()
        };
        assert_eq!(run(199), vec![Rule::SpacingTableTw]);
        assert!(run(200).is_empty(), "the wide-wide 300 does not apply to a wide-narrow pair");
        assert_eq!(crate::dr::log::viol_name(Rule::SpacingTableTw), "SpacingTableTw");
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

    /// A blockage's width in the table is the LAYER's width, not its own (`USEMINSPACING OBS ON`,
    /// the default): a 4000-wide obstruction 200 from a trial needs 140 (the first row), not 280.
    /// Rule (`checkMetalSpacing_prl_getReqSpcVal` under `USEMINSPACING_OBS` false — sky130's tech
    /// LEF says OFF): the obstruction keeps its own width, needs 280, and the trial is marked
    /// (sram_fifo: a macro's met4 OBS corner against a pin wire, and the routes at its edge pins).
    #[test]
    fn blockage_counts_as_layer_width() {
        let shapes = [(Owner::Inst("u".into()), 4, Rect::new(0, 0, 4000, 4000), true), (net("a"), 4, Rect::new(0, 4200, 4000, 4340), false)];
        let m = markers(&shapes);
        assert!(m.is_empty(), "{m:?}");
        let t = Tech { obs_own_width: true, ..tech() };
        let mut w = Worker::new(&t);
        for (o, l, r, f) in &shapes {
            w.add(o, *l, *r, *f);
        }
        w.init();
        assert_eq!(rules(w.run()), vec![Rule::MetalSpacing], "USEMINSPACING OBS OFF: its own width");
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
    /// sufficient. (The route's corner is off this tech's 5-unit grid, so only its off-grid markers
    /// stand: no other rule.)
    #[test]
    fn a_diagonal_of_exactly_min_width_is_sufficient() {
        let m = markers(&[(net("a"), 4, Rect::new(0, 0, 500, 500), true), (net("a"), 4, Rect::new(416, 388, 1000, 1000), false)]);
        assert!(m.iter().all(|m| m.rule == Rule::OffGrid), "{m:?}");
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

    fn table_of(spc: i32) -> CutSpacingTable {
        CutSpacingTable { spacing: vec![(spc, spc); 4], max_spacing: vec![[spc, spc]], ..table(0) }
    }

    // Rule (`checkCutSpacing_main` then `checkLef58CutSpacingTbl_main`): a SAMENET table on the
    // layer judges cuts of ONE net (and only those), and the different-net table then leaves
    // them to it; cuts of two nets stay the different-net table's. Different-net 34, SAMENET 50.
    #[test]
    fn a_same_net_table_takes_the_cuts_of_one_net() {
        let run = |same: Option<i32>, cuts: &[(&str, Rect)]| {
            let mut t = tests::tech();
            t.layers[3].cut_spacing = None;
            t.layers[3].cut_table = Some(table(0));
            t.layers[3].cut_table_same_net = same.map(table_of);
            let mut w = Worker::new(&t);
            for (o, r) in cuts {
                w.add(&Owner::Net((*o).into()), 3, *r, false);
            }
            w.init();
            w.run().iter().filter(|m| m.layer == 3 && m.rule == Rule::Lef58CutSpacingTable).count()
        };
        let a = Rect::new(0, 0, 18, 24);
        let at = |x: i32| Rect::new(18 + x, 0, 36 + x, 24);
        assert_eq!(run(Some(50), &[("a", a), ("a", at(40))]), 1, "one net, 40 apart, below SAMENET 50");
        assert_eq!(run(None, &[("a", a), ("a", at(40))]), 0, "without it, the different-net 34 judges them");
        assert_eq!(run(Some(50), &[("a", a), ("a", at(50))]), 0, "50 apart is not below 50");
        assert_eq!(run(Some(50), &[("a", a), ("b", at(40))]), 0, "two nets: SAMENET skips them, 40 clears 34");
        assert_eq!(run(Some(50), &[("a", a), ("b", at(30))]), 1, "two nets below the different-net 34");
        assert_eq!(run(Some(20), &[("a", a), ("a", at(30))]), 0, "one net: SAMENET 20 decides, not the different-net 34");
    }

    // Call order (`checkCutSpacing_main`): per cut, the SAMENET table is checked BEFORE the
    // different-net one, so its markers come first. Cut a (net a) with a cut of its own net 40 to
    // the right (below SAMENET 50) and one of net b 30 to the left (below the different-net 34).
    #[test]
    fn the_same_net_table_is_checked_before_the_different_net_one() {
        let mut t = tests::tech();
        t.layers[3].cut_spacing = None;
        t.layers[3].cut_table = Some(table(0));
        t.layers[3].cut_table_same_net = Some(table_of(50));
        let mut w = Worker::new(&t);
        w.add(&Owner::Net("a".into()), 3, Rect::new(100, 0, 118, 24), false);
        w.add(&Owner::Net("a".into()), 3, Rect::new(158, 0, 176, 24), false);
        w.add(&Owner::Net("b".into()), 3, Rect::new(52, 0, 70, 24), false);
        w.init();
        let got: Vec<Rect> = w.run().iter().filter(|m| m.rule == Rule::Lef58CutSpacingTable).map(|m| m.bbox).collect();
        assert_eq!(got.first(), Some(&Rect::new(118, 0, 158, 24)), "{got:?}");
        assert!(got.contains(&Rect::new(70, 0, 100, 24)), "{got:?}");
    }

    // The net filter alone (`checkLef58CutSpacingTbl_main`, the non-LAYER branch).
    #[test]
    fn the_cut_table_net_filter() {
        assert!(!same_net_table_skips(true, true, true));
        assert!(same_net_table_skips(true, false, true));
        assert!(same_net_table_skips(false, true, true));
        assert!(!same_net_table_skips(false, true, false));
        assert!(!same_net_table_skips(false, false, true));
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
mod inter_cut_table_tests {
    //! A LEF58 default inter-layer cut spacing table (`SPACINGTABLE … LAYER c3`) on cut layer 5,
    //! on constructed geometry: `tests::tech()` with a cut layer c5 and a routing layer l6 above
    //! it; no classes, FIRST 50, SECOND 80 (a run past the PRL 20), the largest 80.
    use super::*;
    use crate::tech::{InterCutTable, Layer};

    fn table() -> CutSpacingTable {
        CutSpacingTable { n: 1, spacing: vec![(50, 80); 4], max_spacing: vec![[80, 80]], prl_entry: vec![20], center_to_center: vec![false], center_and_edge: vec![false], prl_aligned: vec![false], exact_aligned: vec![-1], names: vec![String::new()], ..Default::default() }
    }

    fn tech(no_stack: bool) -> Tech {
        let mut t = tests::tech();
        t.layers[3].cut_spacing = None;
        t.layers.push(Layer { name: "c5".into(), kind: LayerKind::Cut, width: 40, ..Layer::default() });
        let mut l6 = t.layers[4].clone();
        l6.name = "l6".into();
        t.layers.push(l6);
        t.layers[5].cut_table_inter = Some(InterCutTable { second_layer: 3, no_stack, table: table() });
        t
    }

    /// Cut `a` (net a) on c5 at the origin, the other cut on c3.
    fn markers(t: &Tech, other: (&str, Rect), target: Option<&str>, top: Option<usize>) -> Vec<Marker> {
        let mut w = Worker::new(t);
        if let Some(top) = top {
            w.top_routing_layer = top;
        }
        w.target = target.map(|n| Owner::Net(n.into()));
        w.add(&Owner::Net("a".into()), 5, Rect::new(0, 0, 40, 40), false);
        w.add(&Owner::Net(other.0.into()), 3, other.1, false);
        w.init();
        w.run().iter().filter(|m| matches!(m.rule, Rule::Lef58CutSpacingTable | Rule::Lef58CutSpacingTableInter)).cloned().collect()
    }

    // Rule (`checkLef58CutSpacingTbl_main`, LAYER branch, default table): cuts of two nets on the
    // two layers, edge to edge below the table's value (SECOND where they run past the PRL); one
    // net is never judged. The marker is on the table's layer, its box between the cuts, the
    // victim the cut on the table's layer and the aggressor the second layer's.
    #[test]
    fn cuts_of_two_nets_on_the_two_layers() {
        let t = tech(false);
        let m = markers(&t, ("b", Rect::new(85, 0, 125, 40)), None, None);
        assert_eq!(m.len(), 1, "{m:?}");
        assert_eq!((m[0].rule, m[0].layer, m[0].bbox), (Rule::Lef58CutSpacingTableInter, 5, Rect::new(40, 0, 85, 40)));
        assert_eq!((m[0].victim.as_ref().map(|v| v.1), m[0].aggressor.as_ref().map(|v| v.1)), (Some(5), Some(3)));
        assert_eq!(markers(&t, ("b", Rect::new(110, 0, 150, 40)), None, None).len(), 1, "70 apart, aligned: below SECOND 80");
        assert!(markers(&t, ("b", Rect::new(125, 0, 165, 40)), None, None).is_empty(), "85 apart");
        assert_eq!(markers(&t, ("b", Rect::new(70, 70, 110, 110)), None, None).len(), 1, "diagonal 30/30: no run, FIRST 50, 42.4 apart");
        assert!(markers(&t, ("b", Rect::new(80, 80, 120, 120)), None, None).is_empty(), "diagonal 40/40: 56.6 apart");
    }

    // Rule (same): one net is never judged by the default inter-layer table.
    #[test]
    fn one_net_is_not_judged() {
        assert!(markers(&tech(false), ("a", Rect::new(85, 0, 125, 40)), None, None).is_empty());
    }

    // Rule (`checkLef58CutSpacingTbl_main`, overlapping cuts of the two layers): without NOSTACK a
    // cut containing the other (or equal to it) is no violation, a partial overlap is; with
    // NOSTACK any overlap is.
    #[test]
    fn stacked_cuts_and_nostack() {
        let (stacked, partial) = (Rect::new(0, 0, 40, 40), Rect::new(20, 0, 60, 40));
        assert!(markers(&tech(false), ("b", stacked), None, None).is_empty());
        assert_eq!(markers(&tech(false), ("b", partial), None, None).len(), 1);
        assert_eq!(markers(&tech(true), ("b", stacked), None, None).len(), 1);
        let inside = Rect::new(10, 10, 30, 30);
        assert!(markers(&tech(false), ("b", inside), None, None).is_empty(), "contained, not equal: no violation");
        assert_eq!(markers(&tech(true), ("b", inside), None, None).len(), 1);
    }

    // Rule (`checkLef58CutSpacingTbl_main`): the exact-aligned spacing of a class is read only
    // for a table on one layer; an inter-layer table skips it, so two aligned cuts of one class
    // 70 apart still break SECOND 80 with an exact-aligned value of 60 in the table.
    #[test]
    fn an_inter_layer_table_skips_exact_aligned() {
        let mut t = tech(false);
        t.layers[5].cut_table_inter.as_mut().expect("table").table.exact_aligned = vec![60];
        assert_eq!(markers(&t, ("b", Rect::new(110, 0, 150, 40)), None, None).len(), 1);
    }

    // Call sequence (`checkCutSpacing_main`): a cut on c3 reads the table of the layer two above
    // (c5) too, while c5 is below the top routing layer — so the pair is judged when only the
    // LOWER cut's net is checked from, and not once the top routing layer is c5 itself.
    #[test]
    fn the_lower_cut_reads_the_table_two_above_below_the_top_routing_layer() {
        let t = tech(false);
        let b = ("b", Rect::new(85, 0, 125, 40));
        assert!(markers(&t, b, Some("x"), None).is_empty(), "no net checked from");
        let m = markers(&t, b, Some("b"), None);
        assert_eq!(m.iter().map(|m| (m.rule, m.layer)).collect::<Vec<_>>(), vec![(Rule::Lef58CutSpacingTableInter, 5)]);
        assert!(markers(&t, b, Some("b"), Some(5)).is_empty(), "5 + 0 is not below a top routing layer of 5");
        assert_eq!(markers(&t, b, Some("b"), Some(6)).len(), 1);
    }

    // Rule (`addMarker`: once per box, layer, CONSTRAINT and sources): a cut of net b on c5
    // stacked on one of b on c3, both 45 from cut a, gives the different-net table and the
    // inter-layer table the same box and owners — both markers stand.
    #[test]
    fn the_two_tables_keep_one_marker_each() {
        let mut t = tech(false);
        t.layers[5].cut_table = Some(CutSpacingTable { names: vec![], ..table() });
        let mut w = Worker::new(&t);
        let b = Rect::new(85, 0, 125, 40);
        w.add(&Owner::Net("a".into()), 5, Rect::new(0, 0, 40, 40), false);
        w.add(&Owner::Net("b".into()), 5, b, false);
        w.add(&Owner::Net("b".into()), 3, b, false);
        w.init();
        let mut got: Vec<(Rule, Rect)> = w.run().iter().filter(|m| m.layer == 5).map(|m| (m.rule, m.bbox)).collect();
        got.sort();
        assert_eq!(got, vec![(Rule::Lef58CutSpacingTable, Rect::new(40, 0, 85, 40)), (Rule::Lef58CutSpacingTableInter, Rect::new(40, 0, 85, 40))]);
    }

    // Rule (`checkLef58CutSpacingTbl_helper`, PRLFORALIGNEDCUT): the metal edge looked for is on
    // the layer above the SECOND cut (c3 + 1 = l4), on its facing edge; none there: FIRST.
    #[test]
    fn prl_for_aligned_cut_looks_above_the_second_cut() {
        let mut t = tech(false);
        if let Some(i) = t.layers[5].cut_table_inter.as_mut() {
            i.table.prl_aligned = vec![true];
        }
        let b = Rect::new(110, 0, 150, 40);
        assert!(markers(&t, ("b", b), None, None).is_empty(), "no l4 edge on b's left edge: FIRST 50, 70 clears it");
        let mut w = Worker::new(&t);
        w.add(&Owner::Net("a".into()), 5, Rect::new(0, 0, 40, 40), false);
        w.add(&Owner::Net("b".into()), 3, b, false);
        w.add(&Owner::Net("b".into()), 4, Rect::new(110, -100, 300, 140), false);
        w.init();
        assert_eq!(w.run().iter().filter(|m| m.rule == Rule::Lef58CutSpacingTableInter).count(), 1, "l4 metal edge on b's left edge: SECOND 80");
    }

    // Rule (`checkLef58CutSpacingTbl_main`): classes are each cut's on its OWN layer, looked up in
    // the table by NAME; a name the table does not hold is no class.
    #[test]
    fn classes_are_looked_up_by_name() {
        let class = |name: &str| crate::tech::CutClass { name: name.into(), width: 40, length: 40 };
        let t = CutSpacingTable { names: vec![String::new(), "CB".into(), "CA".into()], ..Default::default() };
        let lower = Layer { cut_classes: vec![class("CA")], ..Default::default() };
        let upper = Layer { cut_classes: vec![class("CB")], ..Default::default() };
        let other = Layer { cut_classes: vec![class("CZ")], ..Default::default() };
        assert_eq!((t.class_index(&lower, 1), t.class_index(&upper, 1), t.class_index(&other, 1), t.class_index(&lower, 0)), (2, 1, 0, 0));
        let same_layer = CutSpacingTable::default();
        assert_eq!(same_layer.class_index(&lower, 1), 1, "a table on one layer is indexed by its layer's classes");
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
mod min_step_tests {
    use super::*;
    use crate::tech::{MinStep, MinStepKind};

    fn ring(pts: &[P], fixed: bool) -> Vec<RingEdge> {
        (0..pts.len()).map(|k| (pts[k], pts[(k + 1) % pts.len()], fixed)).collect()
    }

    // Rule (`checkMetalShape_minStep`): from the first edge at least MINSTEP long, each run of
    // shorter edges up to the next long one is boxed by its edges' end points; a STEP rule marks it
    // past MAXLENGTH (none: any run), unless every edge of it and the closing one is fixed. A 100
    // square with a 10 x 10 notch at its top right corner: one run (the two notch edges).
    #[test]
    fn a_run_of_short_edges_between_long_ones_breaks_min_step() {
        let notch = [(0, 0), (100, 0), (100, 90), (90, 90), (90, 100), (0, 100)];
        let step = |max_length| MinStep { min_step_length: 20, kind: Some(MinStepKind::Step), max_length, max_edges: -1 };
        // The box starts at the long edge's end before the run, (100, 90).
        assert_eq!(min_step_boxes(&[ring(&notch, false)], &step(-1)), vec![Rect::new(90, 90, 100, 100)]);
        // Within MAXLENGTH (the run is 20 long): no marker; fixed edges only: none either.
        assert!(min_step_boxes(&[ring(&notch, false)], &step(20)).is_empty());
        assert!(min_step_boxes(&[ring(&notch, true)], &step(-1)).is_empty());
        // The corner types need a corner the walk never records.
        let inside = MinStep { kind: Some(MinStepKind::InsideCorner), ..step(-1) };
        assert!(min_step_boxes(&[ring(&notch, false)], &inside).is_empty());
        // MAXEDGES (type unknown): marks only past it.
        let edges = |n| MinStep { kind: None, max_edges: n, ..step(-1) };
        assert_eq!(min_step_boxes(&[ring(&notch, false)], &edges(1)).len(), 1);
        assert!(min_step_boxes(&[ring(&notch, false)], &edges(2)).is_empty());
        // An edge exactly MINSTEP long is not short: a 10 notch under MINSTEP 10 is clean.
        assert!(min_step_boxes(&[ring(&notch, false)], &MinStep { min_step_length: 10, ..step(-1) }).is_empty());
        // A plain rectangle has no short edge.
        assert!(min_step_boxes(&[ring(&[(0, 0), (100, 0), (100, 100), (0, 100)], false)], &step(-1)).is_empty());
    }

    /// A 1040 x 40 wire with a 40 wide, 30 tall bump on its top edge (counter-clockwise from the
    /// bottom left: bottom, right end, top right, bump right, bump top, bump left, top left, left
    /// end).
    const BUMP: [P; 8] = [(980, 1000), (2020, 1000), (2020, 1040), (1435, 1040), (1435, 1070), (1395, 1070), (1395, 1040), (980, 1040)];
    const SQUARE: [P; 4] = [(2480, 2480), (2520, 2480), (2520, 2520), (2480, 2520)];

    fn lef58(min_adjacent_length: i32, no_adj_eol: i32, eol_width: i32, except_rectangle: bool) -> crate::tech::Lef58MinStep {
        crate::tech::Lef58MinStep { min_step_length: 50, max_edges: 1, min_adjacent_length, no_adj_eol, eol_width, except_rectangle }
    }

    // Rule (`checkMetalShape_lef58MinStep_noBetweenEol`): from each edge shorter than MINSTEP, the
    // edge after next must be short too and the next edge an end of line (both corners convex)
    // narrower than the NOBETWEENEOL width; the box joins the start edge and the next two edges'
    // far ends. Only the bump's top qualifies; the wire's ends face a long edge.
    #[test]
    fn a_short_end_of_line_between_two_short_edges_breaks_lef58_min_step_no_between_eol() {
        let n = lef58(-1, -1, 50, false);
        assert_eq!(lef58_min_step_no_between_eol_boxes(&[ring(&BUMP, false)], &n), vec![Rect::new(1395, 1040, 1435, 1070)]);
        // The bump top exactly the width wide is not narrower: clean.
        assert!(lef58_min_step_no_between_eol_boxes(&[ring(&BUMP, false)], &lef58(-1, -1, 40, false)).is_empty());
        // The bump sides exactly MINSTEP long are not short: clean.
        let long_sides = crate::tech::Lef58MinStep { min_step_length: 30, ..n };
        assert!(lef58_min_step_no_between_eol_boxes(&[ring(&BUMP, false)], &long_sides).is_empty());
        // All fixed: skipped; one route edge among the three: marked.
        assert!(lef58_min_step_no_between_eol_boxes(&[ring(&BUMP, true)], &n).is_empty());
        let mut one = ring(&BUMP, true);
        one[4].2 = false;
        assert_eq!(lef58_min_step_no_between_eol_boxes(&[one], &n).len(), 1);
        // A square: each edge starts a qualifying run, the same box four times (the marker is kept
        // once per box).
        assert_eq!(lef58_min_step_no_between_eol_boxes(&[ring(&SQUARE, false)], &n), vec![Rect::new(2480, 2480, 2520, 2520); 4]);
        // A plain wire: its ends are short but the edge between them is long.
        assert!(lef58_min_step_no_between_eol_boxes(&[ring(&[(980, 1000), (2020, 1000), (2020, 1040), (980, 1040)], false)], &n).is_empty());
        // The bump's left side 40 long (the wire's top steps down to 1030 on the left): the box
        // reaches the far end of the edge after next.
        let tall = [(980, 1000), (2020, 1000), (2020, 1040), (1435, 1040), (1435, 1070), (1395, 1070), (1395, 1030), (980, 1030)];
        assert_eq!(lef58_min_step_no_between_eol_boxes(&[ring(&tall, false)], &n), vec![Rect::new(1395, 1030, 1435, 1070)]);
        // A narrow end of line after a short jog, the edge after it long: clean.
        let jog = [(0, -20), (60, -20), (60, 0), (100, 0), (100, 40), (0, 40)];
        assert!(lef58_min_step_no_between_eol_boxes(&[ring(&jog, false)], &n).is_empty());
        // A notch: the short edge between its two short sides has concave corners: clean.
        let notch = [(0, 0), (200, 0), (200, 100), (120, 100), (120, 70), (80, 70), (80, 100), (0, 100)];
        assert!(lef58_min_step_no_between_eol_boxes(&[ring(&notch, false)], &n).is_empty());
    }

    // Rule (`checkMetalShape_lef58MinStep_minAdjLength`): from each edge shorter than MINSTEP, a
    // violation when the next or the previous edge is shorter than MINADJACENTLENGTH; with
    // NOADJACENTEOL also when one of them is an end of line (both corners convex) narrower than its
    // width; the box joins the three edges. EXCEPTRECTANGLE exempts a pin whose outer ring has
    // four edges.
    #[test]
    fn a_short_edge_beside_a_short_neighbour_breaks_lef58_min_step_min_adj_length() {
        let a = lef58(60, -1, -1, false);
        // The bump's right side (next: its top), its top (next: its left side) and its left side
        // (previous: its top), each boxed with both neighbours.
        let want = vec![Rect::new(1395, 1040, 2020, 1070), Rect::new(1395, 1040, 1435, 1070), Rect::new(980, 1040, 1435, 1070)];
        assert_eq!(lef58_min_step_min_adj_length_boxes(&[ring(&BUMP, false)], &a), want);
        assert_eq!(lef58_min_step_min_adj_length_boxes(&[ring(&BUMP, false)], &lef58(60, -1, -1, true)), want, "not a rectangle");
        assert!(lef58_min_step_min_adj_length_boxes(&[ring(&BUMP, true)], &a).is_empty(), "all fixed");
        // An edge exactly MINSTEP long starts nothing: under MINSTEP 40 only the bump's sides do.
        let a40 = crate::tech::Lef58MinStep { min_step_length: 40, ..a };
        assert_eq!(lef58_min_step_min_adj_length_boxes(&[ring(&BUMP, false)], &a40), vec![want[0], want[2]]);
        // The square: every edge's neighbours are short; EXCEPTRECTANGLE exempts it, holes or not.
        assert_eq!(lef58_min_step_min_adj_length_boxes(&[ring(&SQUARE, false)], &a).len(), 4);
        assert!(lef58_min_step_min_adj_length_boxes(&[ring(&SQUARE, false)], &lef58(60, -1, -1, true)).is_empty());
        let hole = [(2490, 2490), (2490, 2510), (2510, 2510), (2510, 2490)];
        assert!(lef58_min_step_min_adj_length_boxes(&[ring(&SQUARE, false), ring(&hole, false)], &lef58(60, -1, -1, true)).is_empty());
        // NOADJACENTEOL 50 with MINADJACENTLENGTH 30: the bump's sides are not shorter than 30, but
        // each faces the bump top, an end of line narrower than 50: the right and the left side
        // mark; the top's neighbours have a concave corner, so it does not.
        let j = lef58(30, 50, -1, false);
        assert_eq!(lef58_min_step_min_adj_length_boxes(&[ring(&BUMP, false)], &j), vec![want[0], want[2]]);
        assert!(lef58_min_step_min_adj_length_boxes(&[ring(&BUMP, false)], &lef58(30, 40, -1, false)).is_empty(), "end of line not narrower");
    }
}

#[cfg(test)]
mod min_cut_tests {
    use super::*;
    use crate::tech::MinCut;

    // Rule (`checkMinimumCut`): a same-owner cut below a rectangle at least WIDTH wide is a
    // violation (no LENGTH); FROMABOVE looks only above; a narrower rectangle, or both fixed, none.
    // Layers 2 (routing), 3 (cut), 4 (routing).
    #[test]
    fn a_single_cut_under_a_wide_rectangle_breaks_minimum_cut() {
        let run = |m: MinCut, wide: Rect, fixed: bool| {
            let mut t = tests::tech();
            t.layers[4].min_cuts = vec![m];
            let mut w = Worker::new(&t);
            w.add(&Owner::Net("a".into()), 4, wide, fixed);
            w.add(&Owner::Net("a".into()), 3, Rect::new(100, 100, 150, 150), fixed);
            w.add(&Owner::Net("a".into()), 2, Rect::new(0, 0, 300, 300), fixed);
            w.init();
            w.run().iter().filter(|m| m.rule == Rule::MinimumCut).map(|m| (m.bbox, m.aggressor.as_ref().map(|a| a.1))).collect::<Vec<_>>()
        };
        let rule = MinCut { num_cuts: 2, width: 300, within: None, from_above: None, length: None };
        assert_eq!(run(rule, Rect::new(0, 0, 1000, 400), false), vec![(Rect::new(100, 100, 150, 150), Some(3))]);
        assert!(run(rule, Rect::new(0, 0, 1000, 290), false).is_empty());
        // Exactly WIDTH wide triggers (the check skips only narrower).
        assert_eq!(run(MinCut { width: 400, ..rule }, Rect::new(0, 0, 1000, 400), false).len(), 1);
        assert!(run(MinCut { from_above: Some(true), ..rule }, Rect::new(0, 0, 1000, 400), false).is_empty());
        assert!(run(rule, Rect::new(0, 0, 1000, 400), true).is_empty());
    }
}

#[cfg(test)]
mod enclosure_range_tests {
    use super::*;
    use crate::tech::{CutEnclosure, SpacingRange};

    // Rule (`checkLef58Enclosure_main`): a cut on layer 3 inside metal on layer 4 wider than the
    // rule's WIDTH must overhang it by (first, second) either way round (ENDSIDE: ends then sides);
    // the rule of the widest WIDTH not above the metal's width decides; the marker sits on the
    // metal's layer at the cut, its sides with the top at the right edge. Cut 100..150 square.
    #[test]
    fn a_cut_too_close_to_its_metal_edge_breaks_lef58_enclosure() {
        let run = |rules: Vec<CutEnclosure>, metal: Rect| {
            let mut t = tests::tech();
            t.layers[3].cut_enclosures = rules;
            let mut w = Worker::new(&t);
            w.add(&Owner::Net("a".into()), 3, Rect::new(100, 100, 150, 150), false);
            w.add(&Owner::Net("a".into()), 4, metal, false);
            w.init();
            w.run().iter().filter(|m| m.rule == Rule::Lef58Enclosure).map(|m| (m.layer, m.bbox, m.victim.as_ref().map(|v| v.2))).collect::<Vec<_>>()
        };
        let rule = |first, second, min_width, endside| CutEnclosure { class: 0, above_only: true, below_only: false, eol: false, endside, first, second, min_width };
        // Overhangs: sides (x) 10, ends (y) 30.
        let metal = Rect::new(90, 70, 160, 180);
        assert!(run(vec![rule(30, 10, 0, false)], metal).is_empty());
        assert!(run(vec![rule(10, 30, 0, false)], metal).is_empty(), "either way round");
        assert_eq!(run(vec![rule(10, 30, 0, true)], metal), vec![(4, Rect::new(100, 100, 150, 150), Some(Rect::new(100, 100, 150, 150)))]);
        // The WIDTH 100 rule does not apply to a 70-wide metal: the WIDTH 0 one (satisfied) does.
        assert!(run(vec![rule(30, 10, 0, false), rule(50, 50, 100, false)], metal).is_empty());
        // A metal exactly the WIDTH wide takes that rule (70 here: 50 overhangs not met).
        assert_eq!(run(vec![rule(30, 10, 0, false), rule(50, 50, 70, false)], metal).len(), 1);
        // BELOW-only rules are not checked above.
        assert!(run(vec![CutEnclosure { above_only: false, below_only: true, ..rule(50, 50, 0, false) }], metal).is_empty());
    }

    // Rule (`checkMetalSpacing_prl_getReqSpcVal`): between two owners, a SPACING … RANGE rule
    // holding either width that asks more than the table decides the spacing — and names the
    // marker. Layer 4 table: 140 (widths below 3000). Wires 140 wide, 200 apart.
    #[test]
    fn a_range_rule_asking_more_than_the_table_names_the_marker() {
        let run = |ranges: Vec<SpacingRange>| {
            let mut t = tests::tech();
            t.layers[4].spacing_ranges = ranges;
            let mut w = Worker::new(&t);
            w.add(&Owner::Net("a".into()), 4, Rect::new(0, 0, 1000, 140), false);
            w.add(&Owner::Net("b".into()), 4, Rect::new(0, 340, 1000, 480), false);
            w.init();
            w.run().iter().map(|m| m.rule).filter(|r| *r == Rule::SpacingRange || *r == Rule::MetalSpacing).collect::<Vec<_>>()
        };
        assert!(run(vec![]).is_empty());
        assert_eq!(run(vec![SpacingRange { min_spacing: 250, min_width: 100, max_width: 200 }]), vec![Rule::SpacingRange]);
        assert!(run(vec![SpacingRange { min_spacing: 250, min_width: 150, max_width: 200 }]).is_empty(), "out of range");
        assert_eq!(run(vec![SpacingRange { min_spacing: 250, min_width: 140, max_width: 140 }]), vec![Rule::SpacingRange], "both bounds inclusive");
    }
}

#[cfg(test)]
mod obstruction_rule_tests {
    use super::*;

    // Rule (`checkMetalSpacing_prl_getReqSpcVal`): an instance obstruction with its own SPACING
    // needs exactly that from other shapes (not the table's); with its own DESIGNRULEWIDTH it is
    // looked up at that width. Layer 4 table: 140 below width 3000, 280 from it.
    #[test]
    fn an_obstruction_with_its_own_rule_sets_the_spacing_it_needs() {
        let run = |obs: Owner, gap: i32| {
            let t = tests::tech();
            let mut w = Worker::new(&t);
            w.add(&obs, 4, Rect::new(0, 0, 1000, 140), true);
            w.add(&Owner::Net("a".into()), 4, Rect::new(0, 140 + gap, 1000, 280 + gap), false);
            w.init();
            w.run().iter().filter(|m| m.rule == Rule::MetalSpacing).count()
        };
        // 200 apart: a plain obstruction (layer width → 140) is clean; its own SPACING 250 is not.
        assert_eq!(run(Owner::Inst("u".into()), 200), 0);
        assert_eq!(run(Owner::InstBlockage("u".into(), 0, -1, 250), 200), 1);
        // Its own DESIGNRULEWIDTH 4000 takes the table's 280 row.
        assert_eq!(run(Owner::InstBlockage("u".into(), 0, 4000, -1), 200), 1);
        assert_eq!(run(Owner::InstBlockage("u".into(), 0, 4000, -1), 290), 0);
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
        let m = |owner: &str, layer: usize, b: Rect| Marker { rule: Rule::CornerSpacing, layer, bbox: b, owners: vec![Owner::Net(owner.into())], victim: None, aggressor: None, second_victim: None };
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
        EolRule { space: 200, width: 150, within: 30, parallel: None, end_to_end: None, length: None, extension: None, opposite_width: None }
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

    // Rule (`checkMetalEndOfLine_eol_hasEol_check`): no end-of-line check against — or from — an
    // obstruction with its own SPACING (its spacing check covers it).
    #[test]
    fn an_obstruction_with_its_own_spacing_takes_no_end_of_line_check() {
        let t = tech_with(rule());
        let count = |b: Owner| {
            let mut w = Worker::new(&t);
            w.add(&net("a"), 4, Rect::new(0, 0, 1000, 140), false);
            w.add(&b, 4, Rect::new(1150, -500, 1500, 640), true);
            w.init();
            w.run().iter().filter(|m| m.rule == Rule::EolSpacing).count()
        };
        assert_eq!(count(Owner::Inst("u".into())), 1);
        assert_eq!(count(Owner::InstBlockage("u".into(), 0, -1, 100)), 0);
        assert_eq!(count(Owner::InstBlockage("u".into(), 0, 300, -1)), 1, "a width alone does not skip it");
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

    // Rule (LEF/DEF reference, LEF58_SPACING ENDOFLINE ... OPPOSITEWIDTH ow): the rule applies only
    // if the wire beyond the line end spans LESS than ow perpendicular to the line end. Span: the
    // largest extent, perpendicular to the facing edge, of a maximal rectangle of the facing
    // polygon with a side on the facing edge's line overlapping it. ⛔ DIVERGENCE (register D13):
    // the reference (pin 4155ac31) never reads OPPOSITEWIDTH; upstream-fix 0015 applies this test.
    #[test]
    fn an_opposite_width_exempts_a_line_end_facing_a_wire_at_least_that_wide() {
        let ow = |w| EolRule { opposite_width: Some(w), ..rule() };
        // BLOCK faces a's line end 150 away (inside 200) and spans 350.
        assert_eq!(lef58(rule(), &[("a", WIRE, false), ("b", BLOCK, true)]), vec![GAP], "no clause");
        assert_eq!(lef58(ow(400), &[("a", WIRE, false), ("b", BLOCK, true)]), vec![GAP], "350 < 400");
        assert!(lef58(ow(350), &[("a", WIRE, false), ("b", BLOCK, true)]).is_empty(), "350 = 350: exempt");
        assert!(lef58(ow(300), &[("a", WIRE, false), ("b", BLOCK, true)]).is_empty(), "350 > 300: exempt");
        // A bar 100 wide with an arm along its top: the arm's maximal rectangle reaches the facing
        // edge's line, so the wire spans 1850.
        let bar = Rect::new(1150, -500, 1250, 500);
        let arm = Rect::new(1250, 400, 3000, 500);
        let ell = [("a", WIRE, false), ("b", bar, true), ("b", arm, true)];
        assert!(lef58(ow(400), &ell).is_empty(), "the arm's rectangle spans 1850");
        assert_eq!(lef58(ow(2000), &ell), vec![GAP], "1850 < 2000");
        // Another polygon behind the bar does not widen it: the bar alone spans 100.
        let behind = [("a", WIRE, false), ("b", bar, true), ("b", Rect::new(1300, -500, 3000, 500), true)];
        assert_eq!(lef58(ow(400), &behind), vec![GAP], "100 < 400, the other polygon not counted");
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

    // Rule (`checkMetalEndOfLine_eol_hasEol_getQueryBox` / `_endToEndHelper`, pin 4155ac31, the
    // kit's ENDTOEND e EXTENSION x): with an EXTENSION the window past each side of the line end is
    // 2x INSTEAD of WITHIN (even where 2x is the smaller), and a facing LINE END is judged with
    // BOTH edges widened along themselves by x before the run and the END-TO-END space are
    // measured. A facing edge that is not a line end keeps WITHIN on the first edge alone.
    #[test]
    fn an_end_to_end_extension_widens_both_line_ends_and_sets_the_window() {
        let e2e = EolRule { end_to_end: Some(260), ..rule() };
        let ext = EolRule { extension: Some(40), ..e2e };
        // b's line end (140 long) 220 past a's, 20 above it: no run without the extension; widened
        // by 40 each, the ends overlap 60 in y.
        let offset_end = Rect::new(1220, 160, 2200, 300);
        assert!(lef58(e2e, &[("a", WIRE, false), ("b", offset_end, false)]).is_empty(), "no extension: the ends do not run");
        assert_eq!(lef58(ext, &[("a", WIRE, false), ("b", offset_end, false)]).len(), 1, "extension 40: both ends widened run 60");
        // 80 above: b's end only touches the 2x window and, widened, a's widened end: no run.
        assert!(lef58(ext, &[("a", WIRE, false), ("b", Rect::new(1220, 220, 2200, 360), false)]).is_empty(), "widened ends touching do not run");
        assert_eq!(lef58(ext, &[("a", WIRE, false), ("b", Rect::new(1220, 210, 2200, 350), false)]).len(), 1, "widened ends run 10");
        // Past END-TO-END (270 > 260): none, extension or not.
        assert!(lef58(ext, &[("a", WIRE, false), ("b", Rect::new(1270, 160, 2200, 300), false)]).is_empty(), "past end-to-end");
        // A block (no line end) 20 above a's end, 150 away: the first edge widened by WITHIN 100
        // runs with it; the window, 2x = 20 with an extension of 10, never reaches it.
        let wide = EolRule { within: 100, ..e2e };
        let block = Rect::new(1150, 160, 1500, 600);
        assert_eq!(lef58(wide, &[("a", WIRE, false), ("b", block, true)]).len(), 1, "WITHIN 100 window: the block is held to the space");
        assert!(lef58(EolRule { extension: Some(10), ..wide }, &[("a", WIRE, false), ("b", block, true)]).is_empty(), "the 2x window replaces WITHIN even when smaller");
    }

    // Rule (`checkMetalEndOfLine_eol_hasMinMaxLength`, last in `qualifiesAsEol`): with MINLENGTH
    // (MAXLENGTH) a line end counts only when the edges before and after it are at least (at most)
    // the length: both with TWOSIDES, either without. ⛔ DIVERGENCE by decision (drt divergence
    // register D8, upstream OpenROAD issue #12181, vyges-openroad-patches upstream-fix 0001): the
    // upstream rule takes TWOSIDES from PARALLELEDGE's TWOEDGES; ours (the patched reference's)
    // takes the rule's own TWOSIDES, which this check then reads unchanged.
    // Here `a`'s end at x 1000 has a 400-long edge below it (a step at x 600) and 1000 above.
    #[test]
    fn min_and_max_length_test_the_edges_beside_the_end_on_one_or_both_sides() {
        use crate::tech::EolLength;
        let step = [("a", WIRE, false), ("a", Rect::new(0, -200, 600, 0), false), ("b", BLOCK, true)];
        let len = |max, length, two_sides| EolRule { length: Some(EolLength { max, length, two_sides }), ..rule() };
        assert_eq!(lef58(rule(), &step), vec![GAP], "no length clause");
        assert_eq!(lef58(len(false, 500, false), &step), vec![GAP], "MINLENGTH 500: the 1000 side is enough");
        assert!(lef58(len(false, 500, true), &step).is_empty(), "MINLENGTH 500 TWOSIDES: the 400 side fails");
        assert_eq!(lef58(len(false, 400, true), &step), vec![GAP], "MINLENGTH 400 TWOSIDES: 400 is at least 400");
        assert_eq!(lef58(len(true, 500, false), &step), vec![GAP], "MAXLENGTH 500: the 400 side is enough");
        assert!(lef58(len(true, 500, true), &step).is_empty(), "MAXLENGTH 500 TWOSIDES: the 1000 side fails");
        assert!(lef58(len(true, 300, false), &step).is_empty(), "MAXLENGTH 300: neither side");
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

#[cfg(test)]
mod cut_concave_tests {
    //! LEF58 cut spacing to a concave corner (`SPACING 70 LAYER l4 CUTCLASS CA CONCAVECORNER` on
    //! cut layer 3 of `tests::tech()`), on net `a`'s L on l4: the bar (0, 0)–(1000, 140) and the
    //! arm (860, 0)–(1000, 1000), concave corner (860, 140); class CA is a 100 × 100 cut.
    use super::*;

    fn run(shapes: &[(&str, usize, Rect, bool)]) -> Vec<Marker> {
        run_rules(shapes, 1)
    }

    fn run_rules(shapes: &[(&str, usize, Rect, bool)], rules: usize) -> Vec<Marker> {
        let mut t = tests::tech();
        t.layers[3].cut_spacing = None;
        t.layers[3].cut_classes = vec![crate::tech::CutClass { name: "CA".into(), width: 100, length: 100 }];
        t.layers[3].lef58_cut_spacing = vec![Lef58CutSpacing::Layer(crate::tech::CutLayerSpacing { spacing: 70, second_layer: 4, class: 1, corner: crate::tech::CutCorner::Concave }); rules];
        let mut w = Worker::new(&t);
        for (o, l, r, f) in shapes {
            w.add(&Owner::Net((*o).into()), *l, *r, *f);
        }
        w.init();
        w.run().iter().filter(|m| matches!(m.rule, Rule::Lef58CutSpacing(_))).cloned().collect()
    }

    fn l(fixed: bool) -> Vec<(&'static str, usize, Rect, bool)> {
        vec![("a", 4, Rect::new(0, 0, 1000, 140), fixed), ("a", 4, Rect::new(860, 0, 1000, 1000), fixed)]
    }

    fn with(mut s: Vec<(&'static str, usize, Rect, bool)>, cut: Rect, fixed: bool) -> Vec<(&'static str, usize, Rect, bool)> {
        s.push(("a", 3, cut, fixed));
        s
    }

    // Rule (`checkLef58CutSpacing_main` → `_spc_layer`, CONCAVECORNER): a cut of the rule's class
    // inside a maximal rectangle of the second layer's metal, closer (rectangle to point, squared)
    // than the spacing to a CONCAVE corner of that metal lying on one of the rectangle's boundary
    // lines, is one marker on the CUT layer whose box spans the cut and the corner
    // (`generalized_intersect`); victim the cut, aggressor the rectangle's owner with the corner as
    // a point on the second layer.
    #[test]
    fn a_cut_near_a_concave_corner_of_its_metal_is_a_marker() {
        let cut = Rect::new(740, 20, 840, 120);
        let m = run(&with(l(false), cut, false));
        assert_eq!(m.len(), 1, "{m:?}");
        assert_eq!((m[0].rule, m[0].layer, m[0].bbox), (Rule::Lef58CutSpacing(0), 3, Rect::new(840, 120, 860, 140)));
        assert_eq!(m[0].victim, Some((Owner::Net("a".into()), 3, cut, false)));
        assert_eq!(m[0].aggressor, Some((Owner::Net("a".into()), 4, Rect::new(860, 140, 860, 140), false)));
        assert_eq!(crate::dr::log::viol_name(m[0].rule), "Cut Spacing");
    }

    // Rule: the distance is Euclidean and strict — (50, 0) away marks (2500 < 4900), (70, 0) does
    // not (4900 is not below 4900), nor (70, 20) (5300) though each axis gap alone is within;
    // only a cut of the rule's class is checked.
    #[test]
    fn the_spacing_is_euclidean_and_strict() {
        assert_eq!(run(&with(l(false), Rect::new(710, 40, 810, 140), false)).len(), 1, "(50, 0): 2500");
        assert!(run(&with(l(false), Rect::new(690, 40, 790, 140), false)).is_empty(), "(70, 0): 4900 is not below 4900");
        assert!(run(&with(l(false), Rect::new(690, 20, 790, 120), false)).is_empty(), "(70, 20): 5300");
        assert!(run(&with(l(false), Rect::new(750, 30, 850, 100), false)).is_empty(), "not of class CA (100 x 70)");
        assert!(run(&with(l(false), Rect::new(200, 20, 300, 120), false)).is_empty(), "far");
    }

    // Rule (`addMarker`): a marker is kept once per (box, layer, CONSTRAINT, sources). On a T —
    // bar (0, 0)–(1000, 140), stem (400, 140)–(600, 1000) — a cut under the stem lies in the bar
    // AND in the stem's column through it, and both find the two concave corners: two markers
    // for one rule, four for two rules alike.
    #[test]
    fn a_marker_is_kept_once_per_rule() {
        let t = vec![("a", 4, Rect::new(0, 0, 1000, 140), false), ("a", 4, Rect::new(400, 140, 600, 1000), false)];
        let cut = Rect::new(450, 20, 550, 120);
        let one = run_rules(&with(t.clone(), cut, false), 1);
        let mut boxes: Vec<Rect> = one.iter().map(|m| m.bbox).collect();
        boxes.sort();
        assert_eq!(boxes, [Rect::new(400, 120, 450, 140), Rect::new(550, 120, 600, 140)]);
        let two = run_rules(&with(t, cut, false), 2);
        let mut rules: Vec<Rule> = two.iter().map(|m| m.rule).collect();
        rules.sort();
        assert_eq!(rules, [Rule::Lef58CutSpacing(0), Rule::Lef58CutSpacing(0), Rule::Lef58CutSpacing(1), Rule::Lef58CutSpacing(1)]);
    }

    // Rule: a cut and a corner both fixed are not checked; either one routed is.
    #[test]
    fn a_fixed_cut_at_a_fixed_corner_is_not_checked() {
        let cut = Rect::new(740, 20, 840, 120);
        assert!(run(&with(l(true), cut, true)).is_empty());
        assert_eq!(run(&with(l(true), cut, false)).len(), 1, "routed cut");
        assert_eq!(run(&with(l(false), cut, true)).len(), 1, "routed metal");
    }

    // Rule: the cut must lie wholly inside the maximal rectangle (edges may meet); a cut across the
    // L's inner corner sits in neither rectangle and is not checked.
    #[test]
    fn a_cut_not_inside_one_maximal_rectangle_is_not_checked() {
        assert_eq!(run(&with(l(false), Rect::new(760, 40, 860, 140), false)).len(), 1, "edges on the rectangle's");
        assert!(run(&with(l(false), Rect::new(800, 100, 900, 200), false)).is_empty());
    }
}

#[cfg(test)]
mod cut_convex_tests {
    //! LEF58 cut spacing to a non-end-of-line convex corner (`SPACING 70 LAYER l4 CUTCLASS CA
    //! NONEOLCONVEXCORNER w [MINLENGTH m]` on cut layer 3 of `tests::tech()`); class CA is a
    //! 100 × 100 cut.
    use super::*;
    use crate::tech::{CutClass, CutCorner, CutLayerSpacing};

    fn convex(eol_width: i32, min_length: Option<i32>) -> CutLayerSpacing {
        CutLayerSpacing { spacing: 70, second_layer: 4, class: 1, corner: CutCorner::NonEolConvex { eol_width, min_length } }
    }

    fn run_with(shapes: &[(&str, usize, Rect, bool)], rules: Vec<CutLayerSpacing>) -> Vec<Marker> {
        let mut t = tests::tech();
        t.layers[3].cut_spacing = None;
        t.layers[3].cut_classes = vec![CutClass { name: "CA".into(), width: 100, length: 100 }];
        t.layers[3].lef58_cut_spacing = rules.into_iter().map(Lef58CutSpacing::Layer).collect();
        let mut w = Worker::new(&t);
        for (o, l, r, f) in shapes {
            w.add(&Owner::Net((*o).into()), *l, *r, *f);
        }
        w.init();
        w.run().iter().filter(|m| matches!(m.rule, Rule::Lef58CutSpacing(_))).cloned().collect()
    }

    fn run(shapes: &[(&str, usize, Rect, bool)], eol_width: i32, min_length: Option<i32>) -> Vec<Marker> {
        run_with(shapes, vec![convex(eol_width, min_length)])
    }

    /// Net `a`'s 1000 × 1000 block on l4 (every edge a line end of 1000), and a cut on it.
    fn block(cut: Rect) -> Vec<(&'static str, usize, Rect, bool)> {
        vec![("a", 4, Rect::new(0, 0, 1000, 1000), false), ("a", 3, cut, false)]
    }

    fn boxes(m: &[Marker]) -> Vec<Rect> {
        let mut b: Vec<Rect> = m.iter().map(|m| m.bbox).collect();
        b.sort();
        b
    }

    // Rule (`checkLef58CutSpacing_spc_layer`, NONEOLCONVEXCORNER): a cut of the rule's class inside
    // a maximal rectangle of the second layer's metal, near a CONVEX corner of that metal whose
    // line ends are not narrower than the end-of-line width, is one marker on the CUT layer; its
    // box spans the corner and the cut (`generalized_intersect`); victim the cut, aggressor the
    // rectangle's owner with the corner as a point on the second layer.
    #[test]
    fn a_cut_near_a_wide_convex_corner_is_a_marker() {
        let cut = Rect::new(880, 880, 980, 980);
        let m = run(&block(cut), 150, None);
        assert_eq!(m.len(), 1, "{m:?}");
        assert_eq!((m[0].rule, m[0].layer, m[0].bbox), (Rule::Lef58CutSpacing(0), 3, Rect::new(980, 980, 1000, 1000)));
        assert_eq!(m[0].victim, Some((Owner::Net("a".into()), 3, cut, false)));
        assert_eq!(m[0].aggressor, Some((Owner::Net("a".into()), 4, Rect::new(1000, 1000, 1000, 1000), false)));
        assert_eq!(crate::dr::log::viol_name(m[0].rule), "Cut Spacing");
    }

    // Rule: the keep-out is the TRIANGLE `edgeX * dy + edgeY * dx < edgeX * edgeY` on the box's
    // width dx and height dy — not the circle or the square of radius s: (34, 34) marks
    // (4760 < 4900), (40, 40) does not (5600) though it is 56.6 away and 40 on each axis; strict:
    // (0, 70) does not mark, (0, 69) does.
    #[test]
    fn the_keep_out_is_a_triangle_and_strict() {
        assert_eq!(run(&block(Rect::new(866, 866, 966, 966)), 150, None).len(), 1, "(34, 34): 4760");
        assert!(run(&block(Rect::new(860, 860, 960, 960)), 150, None).is_empty(), "(40, 40): 5600");
        assert!(run(&block(Rect::new(900, 830, 1000, 930)), 150, None).is_empty(), "(0, 70): 4900 is not below 4900");
        assert_eq!(run(&block(Rect::new(900, 831, 1000, 931)), 150, None).len(), 1, "(0, 69)");
        assert!(run(&block(Rect::new(400, 400, 500, 500)), 150, None).is_empty(), "far");
        assert!(run(&block(Rect::new(880, 900, 980, 970)), 150, None).is_empty(), "not of class CA (100 x 70)");
    }

    // Rule: a corner on a line end (an edge between two convex corners) SHORTER than the
    // end-of-line width is not checked; one exactly as wide is. A 140-wide wire end: skipped at
    // width 150, both corners marked at 140.
    #[test]
    fn a_corner_of_a_line_end_narrower_than_the_width_is_not_checked() {
        let wire = |cut| vec![("a", 4, Rect::new(0, 0, 1000, 140), false), ("a", 3, cut, false)];
        let cut = Rect::new(880, 20, 980, 120);
        assert!(run(&wire(cut), 150, None).is_empty());
        assert_eq!(boxes(&run(&wire(cut), 140, None)), [Rect::new(980, 0, 1000, 20), Rect::new(980, 120, 1000, 140)]);
    }

    // Rule: with MINLENGTH an edge is a line end only when BOTH edges beside it are at least that
    // long: a 140 × 140 pad's edges are line ends without it (every corner skipped at width 150)
    // and none is with MINLENGTH 200, so all four corners mark.
    #[test]
    fn min_length_makes_a_short_pad_no_line_end() {
        let pad = vec![("a", 4, Rect::new(0, 0, 140, 140), false), ("a", 3, Rect::new(20, 20, 120, 120), false)];
        assert!(run(&pad, 150, None).is_empty());
        assert_eq!(run(&pad, 150, Some(200)).len(), 4);
        assert!(run(&pad, 150, Some(140)).is_empty(), "sides of 140 reach MINLENGTH 140");
    }

    // Rule: MINLENGTH tests the edges on BOTH sides of the line end, each on its own. The 140-long
    // end of a 1000-long arm whose other side steps in by 100 (a 100 edge to a concave corner):
    // MINLENGTH 100 keeps it a line end (both corners skipped), MINLENGTH 200 does not (the short
    // side alone fails it), so both corners mark — with the short side above, and mirrored below.
    #[test]
    fn min_length_tests_both_sides_of_the_line_end() {
        let above = vec![("a", 4, Rect::new(0, 0, 1000, 140), false), ("a", 4, Rect::new(0, 140, 900, 1000), false), ("a", 3, Rect::new(880, 20, 980, 120), false)];
        let below = vec![("a", 4, Rect::new(0, 860, 1000, 1000), false), ("a", 4, Rect::new(0, 0, 900, 860), false), ("a", 3, Rect::new(880, 880, 980, 980), false)];
        assert!(run(&above, 150, Some(100)).is_empty());
        assert_eq!(boxes(&run(&above, 150, Some(200))), [Rect::new(980, 0, 1000, 20), Rect::new(980, 120, 1000, 140)]);
        assert!(run(&below, 150, Some(100)).is_empty());
        assert_eq!(boxes(&run(&below, 150, Some(200))), [Rect::new(980, 860, 1000, 880), Rect::new(980, 980, 1000, 1000)]);
    }

    // Rule: each leg of the triangle is the spacing CLIPPED to the corner's edge on that axis. At
    // the corner (1000, 0) of the block with a step 40 above it (the edge up to the concave corner
    // (1000, 40) is not a line end), the vertical leg is 40: (0, 45) does not mark (70 * 45 =
    // 3150 >= 70 * 40), (0, 35) does.
    #[test]
    fn each_leg_is_clipped_to_the_corner_edge() {
        let step = |cut| vec![("a", 4, Rect::new(0, 0, 1000, 1000), false), ("a", 4, Rect::new(1000, 40, 2000, 1000), false), ("a", 3, cut, false)];
        assert!(run(&step(Rect::new(900, 45, 1000, 145)), 150, None).is_empty(), "(0, 45)");
        assert_eq!(boxes(&run(&step(Rect::new(900, 35, 1000, 135)), 150, None)), [Rect::new(1000, 0, 1000, 35)], "(0, 35)");
    }

    // Rule: unlike the concave branch there is NO boundary-line test: a corner of ANY owner whose
    // edge touches the rectangle is checked wherever it lies — here net b's corner (180, 100)
    // inside a's pad, off its boundary lines, 30 from the cut.
    #[test]
    fn a_corner_off_the_rectangle_boundary_is_checked() {
        let s = vec![("a", 4, Rect::new(0, 0, 200, 200), false), ("b", 4, Rect::new(180, 100, 400, 400), false), ("a", 3, Rect::new(50, 50, 150, 150), false)];
        let m = run(&s, 50, None);
        assert!(m.iter().any(|m| m.bbox == Rect::new(150, 100, 180, 100) && m.aggressor.as_ref().is_some_and(|a| a.0 == Owner::Net("a".into()))), "{m:?}");
    }

    // Rule: a cut and a corner both fixed are not checked; either one routed is.
    #[test]
    fn a_fixed_cut_at_a_fixed_corner_is_not_checked() {
        let cut = Rect::new(880, 880, 980, 980);
        let s = |metal_fixed, cut_fixed| vec![("a", 4, Rect::new(0, 0, 1000, 1000), metal_fixed), ("a", 3, cut, cut_fixed)];
        assert!(run(&s(true, true), 150, None).is_empty());
        assert_eq!(run(&s(true, false), 150, None).len(), 1, "routed cut");
        assert_eq!(run(&s(false, true), 150, None).len(), 1, "routed metal");
    }

    // Rule: the cut must lie wholly inside the maximal rectangle.
    #[test]
    fn a_cut_not_inside_the_rectangle_is_not_checked() {
        assert!(run(&[("a", 4, Rect::new(0, 0, 1000, 1000), false), ("a", 3, Rect::new(950, 900, 1050, 1000), false)], 150, None).is_empty());
    }

    // Rule: the two branches are one list in the technology's order, the marker's rule its place:
    // with a CONCAVECORNER rule first, the block's convex corner marks as rule 1 only.
    #[test]
    fn the_marker_names_the_rule_by_its_place() {
        let concave = CutLayerSpacing { corner: CutCorner::Concave, ..convex(0, None) };
        let m = run_with(&block(Rect::new(880, 880, 980, 980)), vec![concave, convex(150, None)]);
        assert_eq!(m.iter().map(|m| m.rule).collect::<Vec<_>>(), [Rule::Lef58CutSpacing(1)]);
    }
}

#[cfg(test)]
mod adjacent_cuts_tests {
    //! LEF58 cut spacing ADJACENTCUTS on cut layer 3 of `tests::tech()` (no plain cut spacing):
    //! class CA a 40 × 40 cut, class CW 50 × 50; the rule SPACING 100 CENTERTOCENTER ADJACENTCUTS
    //! 2 WITHIN 120 CUTCLASS CA unless a test says otherwise. Cuts are 40 squares centred at the
    //! points given.
    use super::*;
    use crate::tech::CutClass;

    fn rule() -> AdjacentCuts {
        AdjacentCuts { spacing: 100, center_to_center: true, adjacent_cuts: 2, two_cuts: None, within: 120, class: 1, no_prl: false }
    }

    fn cut(x: i32, y: i32) -> Rect {
        Rect::new(x - 20, y - 20, x + 20, y + 20)
    }

    fn run(rules: Vec<AdjacentCuts>, cuts: &[(&str, Rect, bool)]) -> Vec<Marker> {
        let mut t = tests::tech();
        t.layers[3].cut_spacing = None;
        t.layers[3].cut_classes = vec![CutClass { name: "CA".into(), width: 40, length: 40 }, CutClass { name: "CW".into(), width: 50, length: 50 }];
        t.layers[3].lef58_cut_spacing = rules.into_iter().map(Lef58CutSpacing::AdjacentCuts).collect();
        let mut w = Worker::new(&t);
        for (o, r, f) in cuts {
            w.add(&Owner::Net((*o).into()), 3, *r, *f);
        }
        w.init();
        w.run().iter().filter(|m| matches!(m.rule, Rule::Lef58CutSpacing(_))).cloned().collect()
    }

    fn tri(c: Rect) -> Vec<(&'static str, Rect, bool)> {
        vec![("a", cut(1000, 1000), false), ("b", cut(1090, 1000), false), ("c", c, false)]
    }

    // Rule (`checkLef58CutSpacing_spc_hasAdjCuts`, `_adjCut`): a cut with at least ADJACENTCUTS
    // cuts of the class closer than WITHIN (the count starts at -1 and the cut counts itself)
    // needs the spacing to every other cut; the pair is judged from the first cut alone, and a
    // marker repeated from the other side is the same marker. Victim the first cut.
    #[test]
    fn a_cut_with_two_adjacent_cuts_needs_the_spacing() {
        let m = run(vec![rule()], &tri(cut(1000, 1090)));
        assert_eq!(m.len(), 2, "{m:?}");
        let ab = m.iter().find(|x| x.bbox == Rect::new(1020, 980, 1070, 1020)).expect("the A-B marker");
        assert_eq!((ab.layer, ab.victim.as_ref().map(|v| v.2), ab.aggressor.as_ref().map(|v| v.2)), (3, Some(cut(1000, 1000)), Some(cut(1090, 1000))));
        assert!(run(vec![rule()], &tri(cut(1000, 1130))[..2]).is_empty(), "one neighbour");
        assert!(run(vec![rule()], &tri(cut(1000, 1120))).is_empty(), "120 is not within 120");
        let edge = [("a", cut(1000, 1000), false), ("b", cut(1100, 1000), false), ("c", cut(1000, 1090), false)];
        assert_eq!(run(vec![rule()], &edge).len(), 1, "100 apart is not below 100");
    }

    // Rule (`_adjCut`): no net test — the cuts of one net are judged too; two fixed cuts are not.
    #[test]
    fn one_net_is_judged_and_two_fixed_cuts_are_not() {
        let one: Vec<_> = tri(cut(1000, 1090)).into_iter().map(|(_, r, f)| ("a", r, f)).collect();
        assert_eq!(run(vec![rule()], &one).len(), 2);
        let fixed: Vec<_> = tri(cut(1000, 1090)).into_iter().map(|(o, r, _)| (o, r, true)).collect();
        assert!(run(vec![rule()], &fixed).is_empty());
        let mut one_fixed = tri(cut(1000, 1090));
        one_fixed[0].2 = true;
        assert_eq!(run(vec![rule()], &one_fixed).len(), 2, "one fixed cut is judged");
    }

    // Rule (marker merge, keyed by constraint): two rules on the layer mark the same pair once
    // EACH.
    #[test]
    fn each_rule_keeps_its_own_marker() {
        assert_eq!(run(vec![rule(), rule()], &tri(cut(1000, 1090))).len(), 4);
    }

    // Rule (`_hasAdjCuts`): only cuts of the rule's class count (by width and length); a rule
    // with no class counts the cuts with none.
    #[test]
    fn only_cuts_of_the_class_count() {
        let wide = [("a", cut(1000, 1000), false), ("b", Rect::new(1065, 975, 1115, 1025), false), ("c", cut(1000, 1090), false)];
        assert!(run(vec![rule()], &wide).is_empty());
        let none = AdjacentCuts { adjacent_cuts: 1, class: 0, ..rule() };
        let big = |x: i32| Rect::new(x - 30, 970, x + 30, 1030);
        assert_eq!(run(vec![none], &[("a", big(1000), false), ("b", big(1090), false)]).len(), 1);
        assert!(run(vec![none], &tri(cut(1000, 1090))[..2]).is_empty());
    }

    // Rule (`_hasAdjCuts`, NOPRL): only cuts with no parallel run on either axis count (and the
    // cut itself, at distance 0).
    #[test]
    fn noprl_counts_only_cuts_with_no_parallel_run() {
        let np = AdjacentCuts { spacing: 120, no_prl: true, ..rule() };
        assert!(run(vec![np], &tri(cut(1000, 1090))).is_empty());
        let diag = [("a", cut(1000, 1000), false), ("b", cut(1080, 1080), false), ("c", cut(920, 1080), false)];
        assert_eq!(run(vec![np], &diag).len(), 2);
        // Cuts meeting A at a corner abut on both axes: no parallel run, so they count.
        let corners = [("a", cut(1000, 1000), false), ("b", cut(1040, 1040), false), ("c", cut(960, 1040), false)];
        assert_eq!(run(vec![np], &corners).len(), 2);
    }

    // Rule (`_adjCut`, TWOCUTS): when the count fails, a pair each of whose cuts has TWOCUTS cuts
    // of the class within (itself counted) is judged.
    #[test]
    fn twocuts_judges_a_pair_with_its_own_count() {
        let t = AdjacentCuts { adjacent_cuts: 3, two_cuts: Some(1), ..rule() };
        assert_eq!(run(vec![t], &tri(cut(0, 0))[..2]).len(), 1);
        assert!(run(vec![AdjacentCuts { two_cuts: None, ..t }], &tri(cut(0, 0))[..2]).is_empty());
        // BOTH cuts need the count: B (class CW) does not count itself, so A alone leaves it at
        // 0; A has C too (110 away: within, not below the spacing), so A alone would pass.
        let one_side = [("a", cut(1000, 1000), false), ("b", Rect::new(1065, 975, 1115, 1025), false), ("c", cut(1000, 1110), false)];
        assert!(run(vec![t], &one_side).is_empty());
        // The TWOCUTS count ignores NOPRL: B runs parallel to A and still counts.
        assert_eq!(run(vec![AdjacentCuts { no_prl: true, ..t }], &tri(cut(0, 0))[..2]).len(), 1);
    }

    // Rule (`_adjCut`): edge to edge without CENTERTOCENTER, for the spacing and for WITHIN.
    #[test]
    fn edge_to_edge_without_centertocenter() {
        let e = AdjacentCuts { spacing: 70, center_to_center: false, within: 100, ..rule() };
        // B and C 70.7 apart edge to edge: not below 70; A-B and A-C 50.
        assert_eq!(run(vec![e], &tri(cut(1000, 1090))).len(), 2);
    }
}

#[cfg(test)]
mod orth_spacing_tests {
    //! SPACINGTABLE ORTHOGONAL on cut layer 3 of `tests::tech()` (no plain cut spacing). Cuts are
    //! 40 squares centred at the points given; the table WITHIN 60 SPACING 60 unless a test says
    //! otherwise (the witness `o1`, `o2` cases in the same coordinates).
    use super::*;

    fn cut(x: i32, y: i32) -> Rect {
        Rect::new(x - 20, y - 20, x + 20, y + 20)
    }

    fn run(rows: Vec<(i32, i32)>, cuts: &[(&str, Rect, bool)]) -> Vec<Marker> {
        let mut t = tests::tech();
        t.layers[3].cut_spacing = None;
        t.layers[3].orth_spacing = rows;
        let mut w = Worker::new(&t);
        for (o, r, f) in cuts {
            w.add(&Owner::Net((*o).into()), 3, *r, *f);
        }
        w.init();
        w.run().iter().filter(|m| m.rule == Rule::SpacingTableOrth).cloned().collect()
    }

    fn col(c: Rect) -> Vec<(&'static str, Rect, bool)> {
        vec![("a", cut(1000, 1000), false), ("b", cut(1000, 1090), false), ("c", c, false)]
    }

    /// Rule (`checkCutSpacingTableOrthogonal_helper`): a pair of cuts within WITHIN that runs
    /// parallel on one axis only; the gap between them and both cuts grown ACROSS the run by
    /// SPACING; a third cut whose INTERIOR meets that region marks the box between the gap and it
    /// (witness o1 col: (1020,1025)-(1070,1065)), sources all three, the pair from either cut one
    /// marker. Touching the region is not meeting it; beyond WITHIN no pair; no net test.
    #[test]
    fn a_third_cut_beside_a_pair_marks_the_gap_between() {
        let m = run(vec![(60, 60)], &col(cut(1090, 1045)));
        assert_eq!(m.iter().map(|m| (m.layer, m.bbox)).collect::<Vec<_>>(), [(3, Rect::new(1020, 1025, 1070, 1065))]);
        assert_eq!(m[0].owners.len(), 3);
        assert_eq!(m[0].aggressor.as_ref().map(|a| a.2), Some(cut(1090, 1045)));
        assert!(run(vec![(60, 60)], &col(cut(1110, 1045))).is_empty(), "beyond the region");
        assert!(run(vec![(60, 60)], &col(cut(1100, 1045))).is_empty(), "only touching the region");
        let gap70 = [("a", cut(1000, 1000), false), ("b", cut(1000, 1110), false), ("c", cut(1090, 1055), false)];
        assert!(run(vec![(60, 60)], &gap70).is_empty(), "the pair 70 apart is beyond WITHIN 60");
        let row = [("a", cut(1000, 1000), false), ("b", cut(1090, 1000), false), ("c", cut(1045, 1090), false)];
        assert_eq!(run(vec![(60, 60)], &row).iter().map(|m| m.bbox).collect::<Vec<_>>(), [Rect::new(1025, 1020, 1065, 1070)], "the same across");
        let one_net: Vec<_> = col(cut(1090, 1045)).into_iter().map(|(_, r, f)| ("a", r, f)).collect();
        assert_eq!(run(vec![(60, 60)], &one_net).len(), 1, "no net test");
    }

    /// Rule (`checkCutSpacingTableOrthogonal`): every row in order, each with its own WITHIN and
    /// SPACING; a box found by two rows is one marker (witness o2: row 2 alone reaches the third
    /// cut, and finds the 70-apart pair row 1 does not). A pair running parallel on both axes
    /// (overlapping) or on neither (diagonal) is no pair; three fixed cuts are not judged.
    #[test]
    fn each_row_judges_with_its_own_within_and_spacing() {
        let o2 = vec![(60, 40), (100, 70)];
        assert_eq!(run(o2.clone(), &col(cut(1100, 1045))).len(), 1);
        assert!(run(vec![(60, 40)], &col(cut(1100, 1045))).is_empty());
        let gap70 = [("a", cut(1000, 1000), false), ("b", cut(1000, 1110), false), ("c", cut(1090, 1055), false)];
        assert_eq!(run(o2.clone(), &gap70).len(), 1);
        assert_eq!(run(vec![(60, 60), (60, 60)], &col(cut(1090, 1045))).len(), 1, "one box, one marker");
        // Each two of these are apart on both axes: no pair, so no region to judge.
        let diag = [("a", cut(1000, 1000), false), ("b", cut(1050, 1050), false), ("c", cut(1100, 1100), false)];
        assert!(run(vec![(100, 100)], &diag).is_empty(), "no run on either axis");
        let fixed: Vec<_> = col(cut(1090, 1045)).into_iter().map(|(o, r, _)| (o, r, true)).collect();
        assert!(run(vec![(60, 60)], &fixed).is_empty(), "all three fixed");
        let mut two_fixed = fixed.clone();
        two_fixed[2].2 = false;
        assert_eq!(run(vec![(60, 60)], &two_fixed).len(), 1, "one of the three not fixed");
    }

    /// ⛔ Divergence D5 (register; OpenROAD issue #12198). Upstream rule
    /// (`checkCutSpacingTableOrthogonal_helper`, the marker's victims): the pair, each box taking
    /// its RIGHT edge for its top — `ury = gtl::xh(...)`, a typo — then normalised (pin probe: B
    /// (980,1070)-(1020,1110) recorded as (980,1020)-(1020,1070)). Our rule, the reference patched
    /// with vyges-openroad-patches upstream-fix 0011: each victim box is its cut (fixed probe,
    /// orth-spacing-witness o1 col and row: B recorded as (980,1070)-(1020,1110), and (1070,980)-
    /// (1110,1020) across). Marker boxes and counts are unchanged; the queue reads victim boxes
    /// when a marker lies off the route box.
    #[test]
    fn the_victim_boxes_are_the_cuts() {
        let m = run(vec![(60, 60)], &col(cut(1090, 1045)));
        assert_eq!(m[0].victim.as_ref().map(|v| v.2), Some(Rect::new(980, 980, 1020, 1020)));
        assert_eq!(m[0].second_victim.as_ref().map(|v| v.2), Some(Rect::new(980, 1070, 1020, 1110)));
        let row = [("a", cut(1000, 1000), false), ("b", cut(1090, 1000), false), ("c", cut(1045, 1090), false)];
        assert_eq!(run(vec![(60, 60)], &row)[0].second_victim.as_ref().map(|v| v.2), Some(Rect::new(1070, 980, 1110, 1020)));
    }

    /// Rule (`getRectsPrl`, the helper's first exit): a pair whose longest run is ZERO — two
    /// cuts meeting at one x edge only, here a and b, b up and to the right — is no pair; the
    /// exit is `max(prl_x, prl_y) <= 0`, so a zero run exits as a negative one does. Third cut c
    /// above a would meet a's grown box were the pair judged; the only pair c could be the third
    /// of is (a, b), so no marker has c for its aggressor.
    #[test]
    fn a_pair_meeting_at_an_edge_only_is_no_pair() {
        let (a, b, c) = (cut(1000, 1000), cut(1040, 1090), cut(1000, 1090));
        let m = run(vec![(60, 60)], &[("a", a, false), ("b", b, false), ("c", c, false)]);
        assert!(!m.iter().any(|m| m.aggressor.as_ref().map(|g| g.2) == Some(c)), "{m:?}");
    }

    /// Rule (`checkCutSpacingTableOrthogonal_helper`): the grown gap and both grown cuts are ONE
    /// polygon set, queried by EVERY one of its maximal rectangles and the results joined. With
    /// the pair offset (b 10 to the right of a) the set has three maximal rectangles; c reaches
    /// only the one past b's grown right side, d only the one past a's grown left side, and both
    /// are marked as the pair's third cut.
    #[test]
    fn every_maximal_rectangle_of_the_grown_pair_is_queried() {
        let (a, b) = (cut(1000, 1000), cut(1010, 1090));
        let (c, d) = (Rect::new(1085, 1080, 1125, 1120), Rect::new(885, 985, 925, 1025));
        let m = run(vec![(60, 60)], &[("a", a, false), ("b", b, false), ("c", c, false), ("d", d, false)]);
        for third in [c, d] {
            let of_pair = |m: &&Marker| m.aggressor.as_ref().map(|g| g.2) == Some(third) && m.victim.as_ref().is_some_and(|v| v.2 == a || v.2 == b);
            assert!(m.iter().any(|m| of_pair(&m)), "third cut {third:?} not marked from the pair: {m:?}");
        }
    }

    /// Rule (`getRectsPrl`, the helper's second exit, "short"): a pair running parallel on BOTH
    /// axes — overlapping cuts a and b — is no pair. Third cut c to their right would meet the
    /// pair's grown gap were it judged; the only pair c could be the third of is (a, b), so no
    /// marker has c for its aggressor.
    #[test]
    fn an_overlapping_pair_is_no_pair() {
        let (a, b, c) = (cut(1000, 1000), cut(1010, 1010), cut(1070, 1005));
        let m = run(vec![(60, 60)], &[("a", a, false), ("b", b, false), ("c", c, false)]);
        assert!(!m.iter().any(|m| m.aggressor.as_ref().map(|g| g.2) == Some(c)), "{m:?}");
    }
}
