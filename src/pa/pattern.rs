// SPDX-License-Identifier: Apache-2.0
//! Access patterns: per unique instance, one access point per pin, chosen so that the vias of
//! neighbouring pins stand clear of each other.
//!
//! Stages, in order ([`prep_pattern_inst`]): the pins sorted by the mean x of their access points
//! (by y when no pattern is valid by x) ([`prep_pattern_inst_helper`]); a pass over them, again
//! over the pins reversed when the first pass finds nothing valid ([`gen_patterns`]); per pass up
//! to [`END_ITERATIONS`] rounds of a shortest path through one access point per pin
//! ([`gen_patterns_perform`], edge costs from [`get_edge_cost`]) and a commit of the path found
//! ([`gen_patterns_commit`]). Every committed pattern whose vias pass the design-rule checks
//! together is kept.
//!
//! Rules:
//! - pins are sorted by the ROUNDED mean coordinate (half away from zero) of their access points
//!   RELATIVE TO THE INSTANCE'S PLACEMENT LOCATION (its placed box's lower-left — not the origin
//!   its orientation turns about; rounding half away from zero is not shift-invariant), then by pin index within its terminal; pins still tied keep
//!   their terminal order (the sort is stable at these sizes);
//! - an edge's cost is, in order: [`VIOLATION_COST`] if either end is a known violating access
//!   point; 0 from the source or into the sink; [`VIOLATION_COST`] if the two vias (each end's,
//!   when it has an up access) violate — a verdict cached per edge — or, the first time the edge is
//!   evaluated only, if they violate together with the via of the previous end's current
//!   predecessor; [`REPEATED_AP_COST`] if the first pin's (or the last pin's) end was used by an
//!   earlier round; else the two access points' costs;
//! - a node's best predecessor is the first (lowest index) of the cheapest;
//! - a round whose path repeats an earlier round's ends the pass; a path that violates as a whole
//!   marks each of its access points whose owner a marker names;
//! - a pattern lists one entry per pin of every routed terminal in terminal order (none for a pin
//!   without access points); its boundary points are the first with the least and the first with
//!   the greatest x; its cost is the sum of its points' costs.

use std::collections::BTreeSet;

use crate::gc::{Marker, Owner};
use crate::pa::verdict::{check_in, via_shapes, TargetShape};
use crate::polygon90::Rect;
use crate::tech::{Tech, ViaDef};

/// An edge touching a violating access point, or whose vias violate.
pub const VIOLATION_COST: i32 = 1_000_000;
/// An edge reusing the first or last pin's access point of an earlier round.
pub const REPEATED_AP_COST: i32 = 1_000;
/// Rounds per pass.
pub const END_ITERATIONS: usize = 10;
/// The check window around the vias.
const WINDOW_EXT: i32 = 3000;

/// An access point as the pattern search reads it.
#[derive(Debug, Clone)]
pub struct Ap<'a> {
    /// Design coordinates.
    pub point: (i32, i32),
    pub layer: usize,
    pub cost: i32,
    /// The via of its up access, if it has one.
    pub via: Option<&'a ViaDef>,
}

/// A pin of a routed terminal, in terminal order.
#[derive(Debug, Clone)]
pub struct Pin<'a> {
    /// The terminal's index in the master.
    pub term: usize,
    /// The pin's index within its terminal.
    pub pin: usize,
    /// The owner of its trial vias: its net, or the unconnected terminal.
    pub owner: Owner,
    pub aps: Vec<Ap<'a>>,
}

/// The instance the patterns are searched on.
pub struct Instance<'a> {
    pub tech: &'a Tech,
    /// Its shapes, as the checks see them.
    pub target: &'a [TargetShape],
    /// Its placement location (the lower-left of its placed box, not its orientation origin):
    /// access points are sorted, and boundary points compared, relative to it.
    pub location: (i32, i32),
    /// Every pin of every routed terminal, in terminal order.
    pub pins: Vec<Pin<'a>>,
}

/// An access point: `(index into Instance::pins, index into its aps)`.
pub type ApRef = (usize, usize);

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Pattern {
    /// One entry per `Instance::pins` entry.
    pub aps: Vec<Option<ApRef>>,
    pub left: Option<ApRef>,
    pub right: Option<ApRef>,
    pub cost: i32,
}

/// What the search did, in order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Event {
    /// The sorted pins (`Instance::pins` indices) and their sort coordinates.
    Sorted { use_x: bool, pins: Vec<(usize, i32)> },
    Pass { reversed: bool, max_aps: usize },
    /// An edge evaluated for the first time: `(pin, ap)` positions in the pass's order.
    Pair { prev: (usize, usize), curr: (usize, usize) },
    /// Its look-back to the previous end's predecessor.
    Third { at: (usize, usize) },
    /// A check: the vias (point, via, owner) and the markers.
    Check { commit: bool, vias: Vec<((i32, i32), String, Owner)>, markers: Vec<Marker> },
    Round { i: usize, cost: i32, pattern: Vec<i32> },
    Commit { dup: bool },
    Valid(Option<Pattern>),
    Viol(Vec<(usize, usize)>),
    End(i32),
}

/// Records events when asked to.
pub struct Trace(pub Option<Vec<Event>>);

impl Trace {
    fn push(&mut self, e: impl FnOnce() -> Event) {
        if let Some(v) = &mut self.0 {
            v.push(e());
        }
    }
}

/// The patterns of one unique instance.
pub fn prep_pattern_inst(inst: &Instance<'_>, trace: &mut Trace) -> Vec<Pattern> {
    let mut patterns = Vec::new();
    let n = prep_pattern_inst_helper(inst, true, &mut patterns, trace);
    if n > 0 {
        return patterns;
    }
    prep_pattern_inst_helper(inst, false, &mut patterns, trace);
    patterns
}

/// The pins with access points, sorted by the mean of one coordinate; then the passes.
pub fn prep_pattern_inst_helper(inst: &Instance<'_>, use_x: bool, patterns: &mut Vec<Pattern>, trace: &mut Trace) -> i32 {
    let mut pins: Vec<(i32, usize)> = Vec::new();
    for (i, pin) in inst.pins.iter().enumerate() {
        if pin.aps.is_empty() {
            continue;
        }
        let (mut sx, mut sy) = (0i32, 0i32);
        for ap in &pin.aps {
            sx += ap.point.0 - inst.location.0;
            sy += ap.point.1 - inst.location.1;
        }
        let coord = f64::from(if use_x { sx } else { sy }) / pin.aps.len() as f64;
        pins.push((coord.round() as i32, i));
    }
    pins.sort_by_key(|&(c, i)| (c, inst.pins[i].pin));
    trace.push(|| Event::Sorted { use_x, pins: pins.iter().map(|&(c, i)| (i, c)).collect() });
    let order: Vec<usize> = pins.iter().map(|&(_, i)| i).collect();
    let n = gen_patterns(inst, &order, patterns, trace);
    trace.push(|| Event::End(n));
    n
}

/// A pass over the pins, and over them reversed when it finds nothing valid.
pub fn gen_patterns(inst: &Instance<'_>, pins: &[usize], patterns: &mut Vec<Pattern>, trace: &mut Trace) -> i32 {
    if pins.is_empty() {
        return -1;
    }
    let max_aps = pins.iter().map(|&p| inst.pins[p].aps.len()).max().unwrap_or(0);
    if max_aps == 0 {
        return 0;
    }
    let mut n = gen_patterns_helper(inst, pins, false, max_aps, patterns, trace);
    if n == 0 {
        let reversed: Vec<usize> = pins.iter().rev().copied().collect();
        n += gen_patterns_helper(inst, &reversed, true, max_aps, patterns, trace);
    }
    n
}

/// A DP node: `(pin position, ap index)`; the sink is at position `n`, the source at `n + 1`.
#[derive(Debug, Clone)]
struct Node {
    node_cost: i32,
    path_cost: i32,
    prev: Option<(usize, usize)>,
    source: bool,
    sink: bool,
}

/// The state one pass carries across its rounds.
struct Pass {
    nodes: Vec<Vec<Node>>,
    vio_edges: Vec<i8>,
    access_patterns: BTreeSet<Vec<i32>>,
    used: BTreeSet<(usize, usize)>,
    viol: BTreeSet<(usize, usize)>,
    max_aps: usize,
}

/// One pass: rounds of reset, shortest path and commit until a round repeats.
pub fn gen_patterns_helper(inst: &Instance<'_>, pins: &[usize], reversed: bool, max_aps: usize, patterns: &mut Vec<Pattern>, trace: &mut Trace) -> i32 {
    trace.push(|| Event::Pass { reversed, max_aps });
    let num_edge = (pins.len() + 2) * max_aps * max_aps;
    let mut pass = gen_patterns_init(inst, pins, max_aps, num_edge);
    let mut n = 0;
    for i in 0..END_ITERATIONS {
        gen_patterns_reset(&mut pass, pins);
        gen_patterns_perform(inst, &mut pass, pins, trace);
        trace.push(|| round_event(&pass, pins, i));
        match gen_patterns_commit(inst, &mut pass, pins, patterns, trace) {
            Some(true) => n += 1,
            Some(false) => {}
            None => break,
        }
    }
    n
}

fn round_event(pass: &Pass, pins: &[usize], i: usize) -> Event {
    let sink = &pass.nodes[pins.len()][0];
    let mut pattern = vec![-1; pins.len()];
    let mut at = sink.prev;
    while let Some((p, a)) = at {
        let node = &pass.nodes[p][a];
        if node.source {
            break;
        }
        pattern[p] = a as i32;
        at = node.prev;
    }
    Event::Round { i, cost: sink.path_cost, pattern }
}

/// Nodes for every access point of every pin, plus the source and sink.
fn gen_patterns_init(inst: &Instance<'_>, pins: &[usize], max_aps: usize, num_edge: usize) -> Pass {
    let virt = |source, sink| Node { node_cost: 0, path_cost: if source { 0 } else { i32::MAX }, prev: None, source, sink };
    let mut nodes: Vec<Vec<Node>> = pins
        .iter()
        .map(|&pin| {
            inst.pins[pin].aps.iter().map(|ap| Node { node_cost: ap.cost, path_cost: i32::MAX, prev: None, source: false, sink: false }).collect()
        })
        .collect();
    nodes.push(vec![virt(false, true)]);
    nodes.push(vec![virt(true, false)]);
    Pass { nodes, vio_edges: vec![-1; num_edge], access_patterns: BTreeSet::new(), used: BTreeSet::new(), viol: BTreeSet::new(), max_aps }
}

fn gen_patterns_reset(pass: &mut Pass, pins: &[usize]) {
    for node in pass.nodes.iter_mut().flatten() {
        node.path_cost = i32::MAX;
        node.prev = None;
    }
    let source = &mut pass.nodes[pins.len() + 1][0];
    source.node_cost = 0;
    source.path_cost = 0;
    pass.nodes[pins.len()][0].node_cost = 0;
}

/// The shortest path, pin by pin, then into the sink.
fn gen_patterns_perform(inst: &Instance<'_>, pass: &mut Pass, pins: &[usize], trace: &mut Trace) {
    let source = pins.len() + 1;
    for curr_pin in 0..=pins.len() {
        for curr_ap in 0..pass.nodes[curr_pin].len() {
            if pass.nodes[curr_pin][curr_ap].node_cost == i32::MAX {
                continue;
            }
            let prev_pin = if curr_pin > 0 { curr_pin - 1 } else { source };
            for prev_ap in 0..pass.nodes[prev_pin].len() {
                let prev_cost = pass.nodes[prev_pin][prev_ap].path_cost;
                if prev_cost == i32::MAX {
                    continue;
                }
                let edge = get_edge_cost(inst, pass, pins, (prev_pin, prev_ap), (curr_pin, curr_ap), trace);
                let curr = &mut pass.nodes[curr_pin][curr_ap];
                if curr.path_cost == i32::MAX || curr.path_cost > prev_cost + edge {
                    curr.path_cost = prev_cost + edge;
                    curr.prev = Some((prev_pin, prev_ap));
                }
            }
        }
    }
}

/// The trial via of an access point (in the pass's order), if it has an up access.
fn trial_via<'b>(inst: &'b Instance<'_>, pins: &[usize], (p, a): (usize, usize)) -> Option<((i32, i32), &'b ViaDef, &'b Owner)> {
    let pin = &inst.pins[pins[p]];
    let ap = &pin.aps[a];
    ap.via.map(|v| (ap.point, v, &pin.owner))
}

fn get_edge_cost(inst: &Instance<'_>, pass: &mut Pass, pins: &[usize], prev: (usize, usize), curr: (usize, usize), trace: &mut Trace) -> i32 {
    if pass.viol.contains(&prev) || pass.viol.contains(&curr) {
        return VIOLATION_COST;
    }
    let (prev_node, curr_node) = (&pass.nodes[prev.0][prev.1], &pass.nodes[curr.0][curr.1]);
    if prev_node.source || curr_node.sink {
        return 0;
    }
    let edge_idx = get_flat_edge_idx(prev.0, prev.1, curr.1, pass.max_aps);
    if pass.vio_edges[edge_idx] == 1 {
        return VIOLATION_COST;
    }
    if pass.vio_edges[edge_idx] == -1 {
        trace.push(|| Event::Pair { prev, curr });
        let mut vias: Vec<_> = [trial_via(inst, pins, prev), trial_via(inst, pins, curr)].into_iter().flatten().collect();
        let has_vio = !gen_patterns_gc(inst, &vias, false, trace).0;
        pass.vio_edges[edge_idx] = i8::from(has_vio);
        if has_vio {
            return VIOLATION_COST;
        }
        if let Some(pp) = pass.nodes[prev.0][prev.1].prev {
            if !pass.nodes[pp.0][pp.1].source {
                trace.push(|| Event::Third { at: pp });
                vias.extend(trial_via(inst, pins, pp));
                if !gen_patterns_gc(inst, &vias, false, trace).0 {
                    return VIOLATION_COST;
                }
            }
        }
    }
    let last = pins.len() - 1;
    if (prev.0 == 0 && pass.used.contains(&prev)) || (curr.0 == last && pass.used.contains(&curr)) {
        return REPEATED_AP_COST;
    }
    pass.nodes[prev.0][prev.1].node_cost + pass.nodes[curr.0][curr.1].node_cost
}

/// The checks on a set of trial vias, over their bounding box widened by [`WINDOW_EXT`]: whether
/// they are clean, and the owners the markers name.
fn gen_patterns_gc(inst: &Instance<'_>, vias: &[((i32, i32), &ViaDef, &Owner)], commit: bool, trace: &mut Trace) -> (bool, BTreeSet<Owner>) {
    let markers = vias_markers(inst.tech, inst.target, vias);
    trace.push(|| Event::Check { commit, vias: vias.iter().map(|&(p, v, o)| (p, v.name.clone(), o.clone())).collect(), markers: markers.clone() });
    let owners = markers.iter().flat_map(|m| m.owners.iter().cloned()).collect();
    (markers.is_empty(), owners)
}

/// The markers of trial vias `(point, via, owner)` against `target`, over the vias' bounding box
/// widened by [`WINDOW_EXT`]; no vias, no markers.
pub fn vias_markers(tech: &Tech, target: &[TargetShape], vias: &[((i32, i32), &ViaDef, &Owner)]) -> Vec<Marker> {
    if vias.is_empty() {
        return Vec::new();
    }
    let mut shapes: Vec<(&Owner, usize, Rect)> = Vec::new();
    let mut bbox: Option<Rect> = None;
    for &(at, via, owner) in vias {
        for (layer, r) in via_shapes(via, at) {
            bbox = Some(bbox.map_or(r, |b| Rect::new(b.xl.min(r.xl), b.yl.min(r.yl), b.xh.max(r.xh), b.yh.max(r.yh))));
            shapes.push((owner, layer, r));
        }
    }
    let b = bbox.expect("a via has shapes");
    let win = Rect::new(b.xl - WINDOW_EXT, b.yl - WINDOW_EXT, b.xh + WINDOW_EXT, b.yh + WINDOW_EXT);
    check_in(tech, target, win, &shapes)
}

/// The path found, as an access point per pin; every point on it is marked used.
fn extract_access_pattern_from_nodes(pass: &mut Pass, pins: &[usize]) -> Vec<i32> {
    let mut pattern = vec![-1; pins.len()];
    let mut at = pass.nodes[pins.len()][0].prev;
    loop {
        let (p, a) = at.expect("a path from the source to the sink");
        if pass.nodes[p][a].source {
            break;
        }
        pattern[p] = a as i32;
        pass.used.insert((p, a));
        at = pass.nodes[p][a].prev;
    }
    pattern
}

/// `None` when the path repeats an earlier round's; else whether it is valid (and kept).
fn gen_patterns_commit(inst: &Instance<'_>, pass: &mut Pass, pins: &[usize], patterns: &mut Vec<Pattern>, trace: &mut Trace) -> Option<bool> {
    let access_pattern = extract_access_pattern_from_nodes(pass, pins);
    let dup = pass.access_patterns.contains(&access_pattern);
    trace.push(|| Event::Commit { dup });
    if dup {
        return None;
    }
    pass.access_patterns.insert(access_pattern.clone());
    let chosen: Vec<(usize, usize)> = access_pattern.iter().enumerate().map(|(p, &a)| (p, a as usize)).collect();
    let vias: Vec<_> = chosen.iter().filter_map(|&pa| trial_via(inst, pins, pa)).collect();

    let mut of_pin: Vec<Option<ApRef>> = vec![None; inst.pins.len()];
    for &(p, a) in &chosen {
        of_pin[pins[p]] = Some((pins[p], a));
    }
    let (mut left, mut right) = (None, None);
    let (mut left_x, mut right_x) = (i32::MAX, i32::MIN);
    for &ap in of_pin.iter().flatten() {
        let x = inst.pins[ap.0].aps[ap.1].point.0 - inst.location.0;
        if x < left_x {
            left = Some(ap);
            left_x = x;
        }
        if x > right_x {
            right = Some(ap);
            right_x = x;
        }
    }

    let (clean, owners) = gen_patterns_gc(inst, &vias, true, trace);
    let valid = if clean {
        let cost = of_pin.iter().flatten().map(|&(p, a)| inst.pins[p].aps[a].cost).sum();
        let pattern = Pattern { aps: of_pin, left, right, cost };
        trace.push(|| Event::Valid(Some(pattern.clone())));
        patterns.push(pattern);
        true
    } else {
        trace.push(|| Event::Valid(None));
        for &(p, a) in &chosen {
            if owners.contains(&inst.pins[pins[p]].owner) {
                pass.viol.insert((p, a));
            }
        }
        false
    };
    trace.push(|| Event::Viol(pass.viol.iter().copied().collect()));
    Some(valid)
}

fn get_flat_edge_idx(prev_pin: usize, prev_ap: usize, curr_ap: usize, dim: usize) -> usize {
    ((prev_pin + 1) * dim + prev_ap) * dim + curr_ap
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A via of the gc test technology: l2 and c3 170 square, l4 140 square.
    fn via() -> ViaDef {
        let sq = |h: i32| Rect::new(-h, -h, h, h);
        ViaDef { name: "v".into(), is_default: true, layer1: 2, cut: 3, layer2: 4, layer1_figs: vec![sq(85)], cut_figs: vec![sq(85)], layer2_figs: vec![sq(70)] }
    }

    fn pin<'a>(term: usize, pin: usize, aps: &[((i32, i32), i32)], via: Option<&'a ViaDef>) -> Pin<'a> {
        Pin { term, pin, owner: Owner::Net(format!("n{term}")), aps: aps.iter().map(|&(point, cost)| Ap { point, layer: 2, cost, via }).collect() }
    }

    fn sorted(inst: &Instance<'_>) -> Vec<usize> {
        let mut tr = Trace(Some(Vec::new()));
        prep_pattern_inst(inst, &mut tr);
        match &tr.0.unwrap()[0] {
            Event::Sorted { pins, .. } => pins.iter().map(|p| p.0).collect(),
            e => panic!("first event {e:?}"),
        }
    }

    /// The sort coordinate is the mean RELATIVE TO THE PLACEMENT LOCATION, rounded half away from
    /// zero: at location x 10, pin 1's x 7 and 8 average -2.5 → -3, before pin 0's 8 → -2. In
    /// design coordinates (7.5 → 8) they would tie and keep terminal order; truncated (-2) too.
    #[test]
    fn pins_sort_by_rounded_mean_relative_to_the_location() {
        let t = crate::gc::tests::tech();
        let pins = vec![pin(0, 0, &[((8, 0), 0)], None), pin(1, 0, &[((7, 0), 0), ((8, 0), 0)], None)];
        let inst = Instance { tech: &t, target: &[], location: (10, 0), pins };
        assert_eq!(sorted(&inst), vec![1, 0]);
    }

    /// Tied coordinates sort by the pin's index within its terminal.
    #[test]
    fn tied_pins_sort_by_pin_index() {
        let t = crate::gc::tests::tech();
        let pins = vec![pin(0, 1, &[((500, 0), 0)], None), pin(0, 0, &[((500, 0), 0)], None)];
        let inst = Instance { tech: &t, target: &[], location: (0, 0), pins };
        assert_eq!(sorted(&inst), vec![1, 0]);
    }

    /// Pins still tied after the pin index keep their terminal order.
    #[test]
    fn tied_pins_keep_terminal_order() {
        let t = crate::gc::tests::tech();
        let pins = vec![pin(0, 0, &[((500, 0), 0)], None), pin(1, 0, &[((500, 0), 0)], None), pin(2, 0, &[((500, 0), 0)], None)];
        let inst = Instance { tech: &t, target: &[], location: (0, 0), pins };
        assert_eq!(sorted(&inst), vec![0, 1, 2]);
    }

    /// The check window is the vias' box widened by 3000, and a target shape TOUCHING it is in:
    /// here the far block (y 2500 up, 2415 past the via's box) merges with the near one into a
    /// 3200-wide shape, whose spacing (280) the via 140 below violates. Widened by 2000 the far
    /// block is out, the near one is 2290 wide, and 140 suffices.
    #[test]
    fn a_shape_touching_the_window_is_checked() {
        let t = crate::gc::tests::tech();
        let v = via();
        let b = Owner::Net("b".into());
        let target = vec![(b.clone(), 4, Rect::new(-1600, 210, 1600, 2500)), (b, 4, Rect::new(-1600, 2500, 1600, 4800))];
        let inst = Instance { tech: &t, target: &target, location: (0, 0), pins: vec![pin(0, 0, &[((0, 0), 0)], Some(&v))] };
        let a = Owner::Net("a".into());
        let (clean, owners) = gen_patterns_gc(&inst, &[((0, 0), &v, &a)], false, &mut Trace(None));
        assert!(!clean);
        assert!(owners.contains(&a));
    }

    /// A pass that keeps nothing is retried over the pins REVERSED. Here the forward pass cannot
    /// find n0's point at x 50: n1 has one point, whose best predecessor is n0's x 1050, so the
    /// look-back from n1 to n2's x 950 is checked with x 1050 only (and it violates) — the
    /// look-back runs the first time an edge is costed, never again. Its commits then mark every
    /// point violating and a round repeats. Reversed, the pattern (x 50, 550, 950) is found and
    /// kept, still by x (no fall-back to y).
    #[test]
    fn a_pass_that_keeps_nothing_is_retried_over_the_pins_reversed() {
        let t = crate::gc::tests::tech();
        let v = via();
        let pins = vec![
            pin(0, 0, &[((1050, 400), 0), ((50, 400), 1)], Some(&v)),
            pin(1, 0, &[((550, 400), 0)], Some(&v)),
            pin(2, 0, &[((950, 200), 1), ((400, 400), 1)], Some(&v)),
        ];
        let inst = Instance { tech: &t, target: &[], location: (0, 0), pins };
        let mut tr = Trace(Some(Vec::new()));
        let got = prep_pattern_inst(&inst, &mut tr);
        let ev = tr.0.unwrap();
        assert_eq!(got, vec![Pattern { aps: vec![Some((0, 1)), Some((1, 0)), Some((2, 0))], left: Some((0, 1)), right: Some((2, 0)), cost: 2 }]);
        assert!(ev.iter().any(|e| *e == Event::Pass { reversed: true, max_aps: 2 }));
        // The forward pass's first path runs through the violating look-back: its cost says so.
        let first_round = ev.iter().find_map(|e| if let Event::Round { cost, .. } = e { Some(*cost) } else { None });
        assert_eq!(first_round, Some(VIOLATION_COST));
        assert!(!ev.iter().any(|e| matches!(e, Event::Sorted { use_x: false, .. })));
    }
}
