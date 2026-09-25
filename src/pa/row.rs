// SPDX-License-Identifier: Apache-2.0
//! Row patterns: per run of abutting instances, one access pattern per instance, chosen so that
//! neighbours' facing boundary vias stand clear of each other; the chosen patterns' access points
//! become the instances' terminals' access points.
//!
//! Stages, in order: the routed standard cells in placement order ([`inst_set_order`]) split into
//! rows ([`compute_inst_rows`]); per row ([`gen_inst_row_pattern`]) a node per pattern of each
//! instance's class ([`gen_inst_row_pattern_init`]), a shortest path from the first instance to
//! the last ([`gen_inst_row_pattern_perform`], edge costs from [`get_edge_cost`]), and the path read
//! back ([`gen_inst_row_pattern_commit`]).
//!
//! Rules:
//! - instances are ordered by their placed box's lower-left, y then x; two at the SAME lower-left
//!   have no defined order ([`inst_set_order`] refuses them);
//! - a row continues while the next instance has the same lower y and does not start right of the
//!   previous one's right edge (abutting and overlapping instances continue it);
//! - an edge between neighbours checks only two vias: the previous pattern's RIGHT boundary point's
//!   and the current pattern's LEFT boundary point's (each only if it has an up access), placed at
//!   their own instances, owned by their own instances' nets, against both instances' shapes; a
//!   violation costs [`VIOLATION_COST`], else the two patterns' costs. Edges from the source and
//!   into the sink cost nothing, so a lone instance takes its FIRST pattern;
//! - a node's best predecessor is the first (lowest index) of the cheapest.

use crate::gc::Owner;
use crate::pa::pattern::{vias_markers, Instance, Pattern, VIOLATION_COST};
use crate::pa::verdict::TargetShape;
use crate::polygon90::Rect;
use crate::tech::{Tech, ViaDef};

/// An instance of a row.
pub struct RowInst<'a> {
    /// Its unique class's representative: the pins and access points its patterns index.
    pub class: &'a Instance<'a>,
    /// The class's patterns.
    pub patterns: &'a [Pattern],
    /// Its own placement location.
    pub location: (i32, i32),
    /// Per class pin: the owner of this instance's vias there (its net, or its unconnected
    /// terminal).
    pub owners: Vec<Owner>,
    /// Its own shapes.
    pub target: Vec<TargetShape>,
}

/// What a row did, in order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RowEvent {
    /// An instance edge: `(instance, pattern)` both ends, the vias checked, the verdict and cost.
    Edge { prev: (usize, usize), curr: (usize, usize), vias: Vec<((i32, i32), String, Owner)>, vio: bool, cost: i32 },
    /// The chosen pattern of an instance, read back from the last instance to the first.
    Pick { inst: usize, pattern: usize },
    /// The path's cost at the sink.
    End(i32),
}

/// Placement order: by the placed box's lower-left, y then x. `None` when two instances share a
/// lower-left (their order is undefined).
pub fn inst_set_order(boxes: &[Rect]) -> Option<Vec<usize>> {
    let mut order: Vec<usize> = (0..boxes.len()).collect();
    order.sort_by_key(|&i| (boxes[i].yl, boxes[i].xl));
    let tied = order.windows(2).any(|w| (boxes[w[0]].yl, boxes[w[0]].xl) == (boxes[w[1]].yl, boxes[w[1]].xl));
    (!tied).then_some(order)
}

/// Rows over instances in placement order (`boxes[order[k]]`): a new row when the lower y changes
/// or the instance starts right of the previous one's right edge.
pub fn compute_inst_rows(boxes: &[Rect], order: &[usize]) -> Vec<Vec<usize>> {
    let mut rows: Vec<Vec<usize>> = Vec::new();
    let mut row: Vec<usize> = Vec::new();
    let (mut prev_y, mut prev_x_end) = (i32::MIN, i32::MIN);
    for &i in order {
        let b = boxes[i];
        if (b.yl != prev_y || b.xl > prev_x_end) && !row.is_empty() {
            rows.push(std::mem::take(&mut row));
        }
        row.push(i);
        prev_y = b.yl;
        prev_x_end = b.xh;
    }
    if !row.is_empty() {
        rows.push(row);
    }
    rows
}

/// A row's chosen pattern per instance; `None` when no path reaches the sink (an instance with no
/// pattern).
pub fn gen_inst_row_pattern(tech: &Tech, insts: &[RowInst<'_>], trace: &mut Option<Vec<RowEvent>>) -> Option<Vec<usize>> {
    if insts.is_empty() {
        return Some(Vec::new());
    }
    let mut nodes = gen_inst_row_pattern_init(insts);
    gen_inst_row_pattern_perform(tech, &mut nodes, insts, trace);
    let picked = gen_inst_row_pattern_commit(&nodes, insts, trace);
    if let Some(t) = trace {
        t.push(RowEvent::End(nodes[insts.len()][0].path_cost));
    }
    picked
}

/// A DP node; the sink is at position `n`, the source at `n + 1`.
#[derive(Debug, Clone)]
struct Node {
    node_cost: i32,
    path_cost: i32,
    prev: Option<(usize, usize)>,
    source: bool,
    sink: bool,
}

fn gen_inst_row_pattern_init(insts: &[RowInst<'_>]) -> Vec<Vec<Node>> {
    let node = |node_cost, path_cost, source, sink| Node { node_cost, path_cost, prev: None, source, sink };
    let mut nodes: Vec<Vec<Node>> = insts.iter().map(|i| i.patterns.iter().map(|p| node(p.cost, i32::MAX, false, false)).collect()).collect();
    nodes.push(vec![node(0, i32::MAX, false, true)]);
    nodes.push(vec![node(0, 0, true, false)]);
    nodes
}

fn gen_inst_row_pattern_perform(tech: &Tech, nodes: &mut [Vec<Node>], insts: &[RowInst<'_>], trace: &mut Option<Vec<RowEvent>>) {
    let source = insts.len() + 1;
    for curr_inst in 0..=insts.len() {
        for curr_pat in 0..nodes[curr_inst].len() {
            if nodes[curr_inst][curr_pat].node_cost == i32::MAX {
                continue;
            }
            let prev_inst = if curr_inst > 0 { curr_inst - 1 } else { source };
            for prev_pat in 0..nodes[prev_inst].len() {
                let prev_cost = nodes[prev_inst][prev_pat].path_cost;
                if prev_cost == i32::MAX {
                    continue;
                }
                let edge = get_edge_cost(tech, nodes, insts, (prev_inst, prev_pat), (curr_inst, curr_pat), trace);
                let curr = &mut nodes[curr_inst][curr_pat];
                if curr.path_cost == i32::MAX || curr.path_cost > prev_cost + edge {
                    curr.path_cost = prev_cost + edge;
                    curr.prev = Some((prev_inst, prev_pat));
                }
            }
        }
    }
}

/// The path from the sink back to the source: the chosen pattern per instance.
fn gen_inst_row_pattern_commit(nodes: &[Vec<Node>], insts: &[RowInst<'_>], trace: &mut Option<Vec<RowEvent>>) -> Option<Vec<usize>> {
    let mut picked = vec![usize::MAX; insts.len()];
    let mut at = nodes[insts.len()][0].prev;
    loop {
        let (i, p) = at?;
        if nodes[i][p].source {
            break;
        }
        picked[i] = p;
        if let Some(t) = trace {
            t.push(RowEvent::Pick { inst: i, pattern: p });
        }
        at = nodes[i][p].prev;
    }
    Some(picked)
}

fn get_edge_cost(tech: &Tech, nodes: &[Vec<Node>], insts: &[RowInst<'_>], prev: (usize, usize), curr: (usize, usize), trace: &mut Option<Vec<RowEvent>>) -> i32 {
    if nodes[prev.0][prev.1].source || nodes[curr.0][curr.1].sink {
        return 0;
    }
    let (a, b) = (&insts[prev.0], &insts[curr.0]);
    let mut vias = add_access_pattern_obj(a, &a.patterns[prev.1], true);
    vias.extend(add_access_pattern_obj(b, &b.patterns[curr.1], false));
    let target: Vec<TargetShape> = a.target.iter().chain(&b.target).cloned().collect();
    let vio = !vias_markers(tech, &target, &vias).is_empty();
    let cost = if vio { VIOLATION_COST } else { nodes[prev.0][prev.1].node_cost + nodes[curr.0][curr.1].node_cost };
    if let Some(t) = trace {
        t.push(RowEvent::Edge { prev, curr, vias: vias.iter().map(|&(p, v, o)| (p, v.name.clone(), o.clone())).collect(), vio, cost });
    }
    cost
}

/// The facing boundary via of a pattern placed at `inst`: its right boundary point's when it is
/// the previous instance, else its left's — if that point has an up access.
fn add_access_pattern_obj<'b>(inst: &'b RowInst<'_>, pattern: &Pattern, is_prev: bool) -> Vec<((i32, i32), &'b ViaDef, &'b Owner)> {
    let boundary = if is_prev { pattern.right } else { pattern.left };
    let mut out = Vec::new();
    for (k, entry) in pattern.aps.iter().enumerate() {
        let Some(ap_ref) = *entry else { continue };
        if Some(ap_ref) != boundary {
            continue;
        }
        let ap = &inst.class.pins[ap_ref.0].aps[ap_ref.1];
        if let Some(via) = ap.via {
            let at = (ap.point.0 - inst.class.location.0 + inst.location.0, ap.point.1 - inst.class.location.1 + inst.location.1);
            out.push((at, via, &inst.owners[k]));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pa::pattern::{Ap, Pin};

    /// A via of the gc test technology: l2 and c3 170 square, l4 140 square (cut spacing 190).
    fn via() -> ViaDef {
        let sq = |h: i32| Rect::new(-h, -h, h, h);
        ViaDef { name: "v".into(), is_default: true, layer1: 2, cut: 3, layer2: 4, layer1_figs: vec![sq(85)], cut_figs: vec![sq(85)], layer2_figs: vec![sq(70)] }
    }

    /// A class at the origin: one pin per entry of `pins`, each with its access points' x (y 0).
    fn class<'a>(tech: &'a Tech, v: &'a ViaDef, pins: &[&[i32]]) -> Instance<'a> {
        let pins = pins
            .iter()
            .enumerate()
            .map(|(t, xs)| Pin { term: t, pin: 0, owner: Owner::Net(format!("c{t}")), aps: xs.iter().map(|&x| Ap { point: (x, 0), layer: 2, cost: 0, via: Some(v) }).collect() })
            .collect();
        Instance { tech, target: &[], location: (0, 0), pins }
    }

    fn pattern(aps: &[(usize, usize)], left: (usize, usize), right: (usize, usize), cost: i32) -> Pattern {
        Pattern { aps: aps.iter().map(|&a| Some(a)).collect(), left: Some(left), right: Some(right), cost }
    }

    fn row_inst<'a>(class: &'a Instance<'a>, patterns: &'a [Pattern], x: i32, name: &str, target: Vec<TargetShape>) -> RowInst<'a> {
        let owners = (0..class.pins.len()).map(|t| Owner::Net(format!("{name}{t}"))).collect();
        RowInst { class, patterns, location: (x, 0), owners, target }
    }

    /// Instance a (at 0): pin 0 at x 0, pin 1 at x 300 (pattern 0, cost 0) or x 200 (pattern 1,
    /// cost 1). Instance b (at 600): pins at x 50 and 350 → 650 and 950. The edge checks a's RIGHT
    /// boundary via against b's LEFT one, each at its own instance: 300 vs 650 leaves 180 between
    /// cuts (190 needed) — a violation; 200 vs 650 is clean, costing 1 + 0. So a takes pattern 1.
    /// Checking a's left via (x 0) or b's right one (950) would clear pattern 0 and pick it.
    #[test]
    fn neighbours_check_facing_boundary_vias() {
        let t = crate::gc::tests::tech();
        let v = via();
        let ca = class(&t, &v, &[&[0], &[300, 200]]);
        let cb = class(&t, &v, &[&[50], &[350]]);
        let pa = [pattern(&[(0, 0), (1, 0)], (0, 0), (1, 0), 0), pattern(&[(0, 0), (1, 1)], (0, 0), (1, 1), 1)];
        let pb = [pattern(&[(0, 0), (1, 0)], (0, 0), (1, 0), 0)];
        let row = [row_inst(&ca, &pa, 0, "a", vec![]), row_inst(&cb, &pb, 600, "b", vec![])];
        let mut trace = Some(Vec::new());
        assert_eq!(gen_inst_row_pattern(&t, &row, &mut trace), Some(vec![1, 0]));
        let edges: Vec<(bool, i32)> = trace.unwrap().iter().filter_map(|e| if let RowEvent::Edge { vio, cost, .. } = e { Some((*vio, *cost)) } else { None }).collect();
        assert_eq!(edges, vec![(true, VIOLATION_COST), (false, 1)]);
    }

    /// Both instances' shapes are in the check: an obstruction of b at x 400 stands 130 from a's
    /// pattern-1 via (l4 needs 140), so BOTH of a's patterns violate, and the tie keeps the first.
    /// Without b's shapes, pattern 1 would be clean and chosen.
    #[test]
    fn both_instances_shapes_are_checked() {
        let t = crate::gc::tests::tech();
        let v = via();
        let ca = class(&t, &v, &[&[0], &[300, 200]]);
        let cb = class(&t, &v, &[&[50], &[350]]);
        let pa = [pattern(&[(0, 0), (1, 0)], (0, 0), (1, 0), 0), pattern(&[(0, 0), (1, 1)], (0, 0), (1, 1), 1)];
        let pb = [pattern(&[(0, 0), (1, 0)], (0, 0), (1, 0), 0)];
        let obs = vec![(Owner::Inst("b".into()), 4, Rect::new(400, -200, 420, 200))];
        let row = [row_inst(&ca, &pa, 0, "a", vec![]), row_inst(&cb, &pb, 600, "b", obs)];
        assert_eq!(gen_inst_row_pattern(&t, &row, &mut None), Some(vec![0, 0]));
    }

    /// An instance with no pattern leaves the sink unreachable: no choice for the row.
    #[test]
    fn a_row_without_a_path_has_no_choice() {
        let t = crate::gc::tests::tech();
        let v = via();
        let ca = class(&t, &v, &[&[0]]);
        let pa = [pattern(&[(0, 0)], (0, 0), (0, 0), 0)];
        let row = [row_inst(&ca, &pa, 0, "a", vec![]), row_inst(&ca, &[], 400, "b", vec![])];
        assert_eq!(gen_inst_row_pattern(&t, &row, &mut None), None);
    }

    /// Placement order is lower y, then lower x; a shared lower-left has no order and is refused.
    #[test]
    fn placement_order_refuses_a_shared_lower_left() {
        let b = [Rect::new(500, 0, 900, 100), Rect::new(0, 100, 400, 200), Rect::new(0, 0, 500, 100)];
        assert_eq!(inst_set_order(&b), Some(vec![2, 0, 1]));
        assert_eq!(inst_set_order(&[Rect::new(0, 0, 10, 10), Rect::new(0, 0, 20, 10)]), None);
    }

    /// Abutting and overlapping instances continue a row; a gap or a new y starts one.
    #[test]
    fn rows_continue_through_abutment_and_overlap() {
        let b = [Rect::new(0, 0, 100, 10), Rect::new(100, 0, 200, 10), Rect::new(150, 0, 250, 10), Rect::new(251, 0, 300, 10), Rect::new(300, 10, 400, 20)];
        let order = inst_set_order(&b).unwrap();
        assert_eq!(compute_inst_rows(&b, &order), vec![vec![0, 1, 2], vec![3], vec![4]]);
    }
}
