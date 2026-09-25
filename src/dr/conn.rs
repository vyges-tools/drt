// SPDX-License-Identifier: Apache-2.0
//! The connectivity check between iterations: every net the iteration modified, on its committed
//! shapes in list order.
//!
//! Stages (in order, per net):
//! 1. overlaps — the wires grouped by layer and track (horizontal tracks of every layer first,
//!    then vertical), each track's spans sorted; wires overlapping on a track are SPLIT at the
//!    truncated (pin-facing) ends strictly inside their overlap, then MERGED into the lowest wire
//!    of each overlapping run, which takes the end style of the highest-reaching one;
//! 2. the node map — every wire end and via layer is a node; a wire or via end on another wire's
//!    interior (the first wire on that track ending at or after the point) joins it; a truncated
//!    wire end or a pin-connected via layer on a term of the net joins that term (terms ordered by
//!    kind, then id);
//! 3. a best-first search from the first term: each pass reaches the nearest unreached term (one
//!    per hop, a term already on the tree costing 5 to pass through) and adds its path to the tree;
//!    a term reached by a wire only through a truncated end at the node, or by a via connected to a
//!    pin at all; a term left unreached is an error;
//! 4. finish — every object off the tree is removed (a marker on it; a via's on its bottom layer);
//!    a wire running THROUGH a term's node (no wire end or via there) is split there, splits taken
//!    from the right; every wire shrinks to its outermost node shared with another object (a marker
//!    before it shrinks); a patch not on a remaining wire end or via layer is removed (a marker).
//!
//! A new piece a split makes is a bare wire: no width, both extensions zero; a wire end a split
//! or merge touches loses its extension too. New pieces go to the end of the net's list.

use std::cmp::Reverse;
use std::collections::{BTreeMap, BTreeSet, BinaryHeap};

use crate::polygon90::Rect;
use crate::tech::Tech;

type P = (i32, i32);
type Node = (P, usize);

/// A committed wire as the check reads and writes it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Seg {
    pub layer: usize,
    pub begin: P,
    pub end: P,
    pub width: i32,
    pub begin_trunc: bool,
    pub begin_ext: i32,
    pub end_trunc: bool,
    pub end_ext: i32,
    pub tapered: bool,
}

impl Seg {
    fn is_vertical(&self) -> bool {
        self.begin.0 == self.end.0
    }
    fn low(&self) -> i32 {
        if self.is_vertical() {
            self.begin.1
        } else {
            self.begin.0
        }
    }
    fn high(&self) -> i32 {
        if self.is_vertical() {
            self.end.1
        } else {
            self.end.0
        }
    }
    fn set_low(&mut self, v: i32) {
        if self.is_vertical() {
            self.begin.1 = v;
        } else {
            self.begin.0 = v;
        }
    }
    fn set_high(&mut self, v: i32) {
        if self.is_vertical() {
            self.end.1 = v;
        } else {
            self.end.0 = v;
        }
    }
    /// The wire's box: a zero-length wire counts as vertical.
    pub fn bbox(&self) -> Rect {
        let hw = self.width / 2;
        if self.begin.0 != self.end.0 {
            Rect { xl: self.begin.0 - self.begin_ext, yl: self.begin.1 - hw, xh: self.end.0 + self.end_ext, yh: self.end.1 + hw }
        } else {
            Rect { xl: self.begin.0 - hw, yl: self.begin.1 - self.begin_ext, xh: self.end.0 + hw, yh: self.end.1 + self.end_ext }
        }
    }
}

/// A committed via: its definition, origin, and whether each end connects to a pin.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Via {
    pub via: usize,
    pub origin: P,
    pub tapered: bool,
    pub bottom_connected: bool,
    pub top_connected: bool,
}

/// A committed patch.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Patch {
    pub layer: usize,
    pub origin: P,
    pub offset: Rect,
}

/// A net's shapes in list order (removed slots `None`; new wires appended).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct NetShapes {
    pub segs: Vec<Option<Seg>>,
    pub vias: Vec<Option<Via>>,
    pub patches: Vec<Option<Patch>>,
}

/// A route object: a wire or via of the net, by list index.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Obj {
    Seg(usize),
    Via(usize),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct Span {
    lo: i32,
    hi: i32,
}

/// Layer → track coordinate → the objects on it.
type ByTrack = Vec<BTreeMap<i32, Vec<usize>>>;

/// The check of one net. `pins_at(point, layer)` gives the net's terms with a shape at the point
/// on the layer, each by its order key (kind, then id). Returns the markers it adds, in order;
/// an error when a term is left unreached.
pub fn check_net<K: Ord + Clone>(tech: &Tech, net: &mut NetShapes, pins_at: &dyn Fn(P, usize) -> Vec<K>) -> Result<Vec<(usize, Rect)>, String> {
    let nl = tech.layers.len();
    let mut objs = init_route_objs(net);
    let (horz, vert) = organize_path_segs_by_layer_and_track(net, &objs, nl);
    let found = find_segment_overlaps(net, &mut objs, &horz, &vert)?;
    handle_segment_overlaps(net, &objs, &horz, &vert, &found);
    let objs = init_route_objs(net);
    let pin2ep = build_pin2ep_map(tech, net, &objs, pins_at);
    let (npins, mut node_map) = build_node_map(tech, net, &objs, &pin2ep, nl);
    let (nr, n) = (objs.len(), objs.len() + npins);
    let visited = astar(net, &node_map, &objs, nr, n);
    let reached = visited[nr..].iter().filter(|&&v| v).count();
    if reached != n - nr {
        return Err(format!("{} term(s) not reached", n - nr - reached));
    }
    let mut objs: Vec<Option<Obj>> = objs.into_iter().map(Some).collect();
    let mut markers = Vec::new();
    finish(tech, net, &mut objs, &visited, nr, n, &mut node_map, &mut markers)?;
    Ok(markers)
}

fn seg(net: &NetShapes, o: Obj) -> &Seg {
    match o {
        Obj::Seg(k) => net.segs[k].as_ref().expect("a live wire"),
        Obj::Via(_) => unreachable!("a via where a wire is expected"),
    }
}

fn seg_mut(net: &mut NetShapes, o: Obj) -> &mut Seg {
    match o {
        Obj::Seg(k) => net.segs[k].as_mut().expect("a live wire"),
        Obj::Via(_) => unreachable!("a via where a wire is expected"),
    }
}

/// The net's wires in list order, then its vias.
fn init_route_objs(net: &NetShapes) -> Vec<Obj> {
    let segs = (0..net.segs.len()).filter(|&k| net.segs[k].is_some()).map(Obj::Seg);
    let vias = (0..net.vias.len()).filter(|&k| net.vias[k].is_some()).map(Obj::Via);
    segs.chain(vias).collect()
}

fn organize_path_segs_by_layer_and_track(net: &NetShapes, objs: &[Obj], nl: usize) -> (ByTrack, ByTrack) {
    let (mut horz, mut vert): (ByTrack, ByTrack) = (vec![BTreeMap::new(); nl], vec![BTreeMap::new(); nl]);
    for (i, &o) in objs.iter().enumerate() {
        if let Obj::Seg(_) = o {
            let s = seg(net, o);
            if s.begin.0 == s.end.0 {
                vert[s.layer].entry(s.begin.0).or_default().push(i);
            } else if s.begin.1 == s.end.1 {
                horz[s.layer].entry(s.begin.1).or_default().push(i);
            }
        }
    }
    (horz, vert)
}

/// Per track (horizontal tracks of every layer, then vertical): the victims and merged spans.
struct Overlaps {
    horz: Vec<Vec<(Vec<usize>, Vec<Span>)>>,
    vert: Vec<Vec<(Vec<usize>, Vec<Span>)>>,
}

fn find_segment_overlaps(net: &mut NetShapes, objs: &mut Vec<Obj>, horz: &ByTrack, vert: &ByTrack) -> Result<Overlaps, String> {
    let mut out = Overlaps { horz: Vec::new(), vert: Vec::new() };
    for (by, dst) in [(horz, &mut out.horz), (vert, &mut out.vert)] {
        for tracks in by {
            let mut per = Vec::new();
            for indices in tracks.values() {
                per.push(handle_overlaps_perform(net, objs, indices)?);
            }
            dst.push(per);
        }
    }
    Ok(out)
}

fn handle_segment_overlaps(net: &mut NetShapes, objs: &[Obj], horz: &ByTrack, vert: &ByTrack, found: &Overlaps) {
    for (by, per, is_horz) in [(horz, &found.horz, true), (vert, &found.vert, false)] {
        for (l, tracks) in by.iter().enumerate() {
            for (t, &track) in tracks.keys().enumerate() {
                let (victims, spans) = &per[l][t];
                merge_commit(net, objs, victims, track, spans, is_horz);
            }
        }
    }
}

fn handle_overlaps_perform(net: &mut NetShapes, objs: &mut Vec<Obj>, indices: &[usize]) -> Result<(Vec<usize>, Vec<Span>), String> {
    let mut spans: Vec<(Span, usize)> = indices.iter().map(|&i| (Span { lo: seg(net, objs[i]).low(), hi: seg(net, objs[i]).high() }, i)).collect();
    spans.sort();
    split_path_segs(net, objs, &mut spans)?;
    Ok(merge_perform_helper(&spans))
}

/// Overlapping runs of a track's sorted spans: split points are the truncated ends strictly
/// inside a run (not its first low, not its highest end), each run committed when the next span
/// starts at or past the run's highest end.
fn split_path_segs(net: &mut NetShapes, objs: &mut Vec<Obj>, spans: &mut Vec<(Span, usize)>) -> Result<(), String> {
    let mut highest: Option<Obj> = None;
    let mut first = 0usize;
    let mut split_points: Vec<i32> = Vec::new();
    if spans.is_empty() {
        return Ok(());
    }
    let mut i = 0usize;
    while i < spans.len() {
        let curr = spans[i];
        let curr_ps = objs[curr.1];
        if highest.is_none_or(|h| curr.0.lo >= seg(net, h).high()) {
            if !split_points.is_empty() {
                if let Some(h) = highest {
                    split_path_segs_commit(net, objs, &mut split_points, h, first, &mut i, spans)?;
                }
            }
            first = i;
            highest = Some(curr_ps);
        } else {
            let prev_ps = objs[spans[i - 1].1];
            let c = seg(net, curr_ps).clone();
            if curr.0.lo != spans[first].0.lo && c.begin_trunc && !split_points.contains(&curr.0.lo) {
                split_points.push(curr.0.lo);
            }
            if c.end_trunc && !split_points.contains(&curr.0.hi) {
                split_points.push(curr.0.hi);
            }
            if i - 1 == first {
                let p = seg(net, prev_ps);
                if p.end_trunc && !split_points.contains(&p.high()) {
                    split_points.push(p.high());
                }
            }
            if let Some(h) = highest {
                if seg(net, h).high() < curr.0.hi {
                    highest = Some(curr_ps);
                }
            }
        }
        i += 1;
    }
    if !split_points.is_empty() {
        if let Some(h) = highest {
            let mut end = spans.len();
            split_path_segs_commit(net, objs, &mut split_points, h, first, &mut end, spans)?;
        }
    }
    Ok(())
}

/// One run's splits: the run's wires with a split point strictly inside are re-cut to the
/// consecutive pieces between split points (the last reaching the run's highest end with its end
/// style); pieces left over become new bare wires appended to the net; the run re-sorted.
fn split_path_segs_commit(net: &mut NetShapes, objs: &mut Vec<Obj>, split_points: &mut Vec<i32>, highest: Obj, first: usize, i: &mut usize, spans: &mut Vec<(Span, usize)>) -> Result<(), String> {
    split_points.sort();
    let h = seg(net, highest).clone();
    if split_points.last() == Some(&h.high()) {
        split_points.pop();
    }
    if !split_points.is_empty() {
        let (highest_end_trunc, highest_hi) = (h.end_trunc, h.high());
        let split_span_idxs: Vec<usize> = (first..*i).filter(|&k| split_points.iter().any(|&p| spans[k].0.lo < p && spans[k].0.hi > p)).collect();
        let mut cur = 0usize;
        let mut s = 0usize;
        while s <= split_points.len() && cur < split_span_idxs.len() {
            let k = split_span_idxs[cur];
            let ps = seg_mut(net, objs[spans[k].1]);
            if s != 0 {
                spans[k].0.lo = split_points[s - 1];
                ps.set_low(split_points[s - 1]);
                (ps.begin_trunc, ps.begin_ext) = (true, 0);
            }
            if s == split_points.len() {
                ps.set_high(highest_hi);
                (ps.end_trunc, ps.end_ext) = (highest_end_trunc, 0);
                (ps.begin_trunc, ps.begin_ext) = (true, 0);
                spans[k].0.hi = highest_hi;
                // More split wires than pieces: the extra ones repeat the last piece (merged later).
                if cur < split_span_idxs.len() - 1 {
                    s -= 1;
                }
            } else {
                spans[k].0.hi = split_points[s];
                ps.set_high(split_points[s]);
                (ps.end_trunc, ps.end_ext) = (true, 0);
            }
            s += 1;
            cur += 1;
        }
        if s == 0 {
            // The reference reads split_points[-1] here.
            return Err("a split run with no wire across a split point".into());
        }
        while s <= split_points.len() {
            let lo = split_points[s - 1];
            let (hi, hi_trunc) = if s == split_points.len() { (highest_hi, highest_end_trunc) } else { (split_points[s], true) };
            spans.insert(*i, (Span { lo, hi }, objs.len()));
            *i += 1;
            let (begin, end) = if h.is_vertical() { ((h.begin.0, lo), (h.begin.0, hi)) } else { ((lo, h.begin.1), (hi, h.begin.1)) };
            objs.push(Obj::Seg(net.segs.len()));
            net.segs.push(Some(Seg { layer: h.layer, begin, end, width: 0, begin_trunc: true, begin_ext: 0, end_trunc: hi_trunc, end_ext: 0, tapered: false }));
            s += 1;
        }
        spans[first..*i].sort();
    }
    split_points.clear();
    Ok(())
}

/// The track's overlapping runs: each run's wires (the victims, in span order) and its merged
/// span.
fn merge_perform_helper(spans: &[(Span, usize)]) -> (Vec<usize>, Vec<Span>) {
    let (mut victims, mut new_spans) = (Vec::new(), Vec::new());
    let mut has_overlap = false;
    let (mut start, mut end) = (i32::MAX, i32::MIN);
    let mut local: Vec<usize> = Vec::new();
    for &(sp, idx) in spans {
        if sp.lo >= end {
            if has_overlap {
                new_spans.push(Span { lo: start, hi: end });
                victims.extend(local.iter().copied());
            }
            local.clear();
            has_overlap = false;
            (start, end) = (sp.lo, sp.hi);
            local.push(idx);
        } else {
            has_overlap = true;
            local.push(idx);
            end = end.max(sp.hi);
        }
    }
    if has_overlap {
        new_spans.push(Span { lo: start, hi: end });
        victims.extend(local);
    }
    (victims, new_spans)
}

/// Each run merges into its first victim (stretched to the run's span); the victims after it up
/// to the span's end are removed, the highest-reaching one's end style kept.
fn merge_commit(net: &mut NetShapes, objs: &[Obj], victims: &[usize], track: i32, new_spans: &[Span], is_horz: bool) {
    if victims.is_empty() {
        return;
    }
    let mut cnt = 0usize;
    for ns in new_spans {
        let vo = objs[victims[cnt]];
        let mut high = seg(net, vo).high();
        let (b, e) = if is_horz { ((ns.lo, track), (ns.hi, track)) } else { ((track, ns.lo), (track, ns.hi)) };
        {
            let v = seg_mut(net, vo);
            (v.begin, v.end) = (b, e);
        }
        cnt += 1;
        let (mut end_trunc, mut end_ext) = { (seg(net, vo).end_trunc, seg(net, vo).end_ext) };
        while cnt < victims.len() {
            let co = objs[victims[cnt]];
            let c = seg(net, co);
            if c.high() > ns.hi {
                break;
            }
            if c.high() >= high {
                (end_trunc, end_ext, high) = (c.end_trunc, c.end_ext, c.high());
            }
            if let Obj::Seg(k) = co {
                net.segs[k] = None;
            }
            cnt += 1;
        }
        let v = seg_mut(net, vo);
        (v.end_trunc, v.end_ext) = (end_trunc, end_ext);
    }
}

/// The net's terms at its truncated wire ends and pin-connected via layers.
fn build_pin2ep_map<K: Ord + Clone>(tech: &Tech, net: &NetShapes, objs: &[Obj], pins_at: &dyn Fn(P, usize) -> Vec<K>) -> BTreeMap<K, BTreeSet<Node>> {
    let mut map: BTreeMap<K, BTreeSet<Node>> = BTreeMap::new();
    let mut helper = |pt: P, l: usize| {
        for k in pins_at(pt, l) {
            map.entry(k).or_default().insert((pt, l));
        }
    };
    for &o in objs {
        if let Obj::Seg(_) = o {
            let s = seg(net, o);
            if s.begin_trunc {
                helper(s.begin, s.layer);
            }
            if s.end_trunc {
                helper(s.end, s.layer);
            }
        }
    }
    for &o in objs {
        if let Obj::Via(k) = o {
            let v = net.vias[k].as_ref().expect("a live via");
            let vd = &tech.via_defs[v.via];
            if v.bottom_connected {
                helper(v.origin, vd.layer1);
            }
            if v.top_connected {
                helper(v.origin, vd.layer2);
            }
        }
    }
    map
}

fn build_node_map<K: Ord + Clone>(tech: &Tech, net: &NetShapes, objs: &[Obj], pin2ep: &BTreeMap<K, BTreeSet<Node>>, nl: usize) -> (usize, BTreeMap<Node, BTreeSet<usize>>) {
    let mut node_map: BTreeMap<Node, BTreeSet<usize>> = BTreeMap::new();
    node_map_route_obj_end(tech, net, objs, &mut node_map);
    node_map_route_obj_split(tech, net, objs, &mut node_map, nl);
    let npins = node_map_pin(objs, pin2ep, &mut node_map);
    (npins, node_map)
}

fn via_layers(tech: &Tech, net: &NetShapes, k: usize) -> (P, usize, usize) {
    let v = net.vias[k].as_ref().expect("a live via");
    let vd = &tech.via_defs[v.via];
    (v.origin, vd.layer1, vd.layer2)
}

fn node_map_route_obj_end(tech: &Tech, net: &NetShapes, objs: &[Obj], node_map: &mut BTreeMap<Node, BTreeSet<usize>>) {
    for (i, &o) in objs.iter().enumerate() {
        match o {
            Obj::Seg(_) => {
                let s = seg(net, o);
                node_map.entry((s.begin, s.layer)).or_default().insert(i);
                node_map.entry((s.end, s.layer)).or_default().insert(i);
            }
            Obj::Via(k) => {
                let (origin, l1, l2) = via_layers(tech, net, k);
                node_map.entry((origin, l1)).or_default().insert(i);
                node_map.entry((origin, l2)).or_default().insert(i);
            }
        }
    }
}

/// Layer → track → end coordinate → (begin coordinate, object); a later wire with the same end
/// replaces an earlier one.
type MergeHelper = Vec<BTreeMap<i32, BTreeMap<i32, (i32, usize)>>>;

/// A point on a track joins the first wire of the track ending at or after it, when strictly
/// inside that wire.
fn node_map_route_obj_split_helper(cross: P, track: i32, split: i32, l: usize, helper: &MergeHelper, node_map: &mut BTreeMap<Node, BTreeSet<usize>>) {
    let Some(mp) = helper[l].get(&track) else { return };
    let Some((&end, &(begin, idx))) = mp.range(split..).next() else { return };
    if begin < split && split < end {
        node_map.entry((cross, l)).or_default().insert(idx);
    }
}

fn node_map_route_obj_split(tech: &Tech, net: &NetShapes, objs: &[Obj], node_map: &mut BTreeMap<Node, BTreeSet<usize>>, nl: usize) {
    let (mut horz, mut vert): (MergeHelper, MergeHelper) = (vec![BTreeMap::new(); nl], vec![BTreeMap::new(); nl]);
    for (i, &o) in objs.iter().enumerate() {
        if let Obj::Seg(_) = o {
            let s = seg(net, o);
            if s.begin.0 == s.end.0 {
                vert[s.layer].entry(s.begin.0).or_default().insert(s.end.1, (s.begin.1, i));
            } else {
                horz[s.layer].entry(s.begin.1).or_default().insert(s.end.0, (s.begin.0, i));
            }
        }
    }
    for &o in objs {
        match o {
            Obj::Seg(_) => {
                let s = seg(net, o);
                if s.begin.0 == s.end.0 {
                    node_map_route_obj_split_helper(s.begin, s.begin.1, s.begin.0, s.layer, &horz, node_map);
                    node_map_route_obj_split_helper(s.end, s.end.1, s.end.0, s.layer, &horz, node_map);
                } else {
                    node_map_route_obj_split_helper(s.begin, s.begin.0, s.begin.1, s.layer, &vert, node_map);
                    node_map_route_obj_split_helper(s.end, s.end.0, s.end.1, s.layer, &vert, node_map);
                }
            }
            Obj::Via(k) => {
                let (origin, l1, l2) = via_layers(tech, net, k);
                for l in [l1, l2] {
                    node_map_route_obj_split_helper(origin, origin.1, origin.0, l, &horz, node_map);
                    node_map_route_obj_split_helper(origin, origin.0, origin.1, l, &vert, node_map);
                }
            }
        }
    }
}

/// The terms, in key order, numbered after the route objects.
fn node_map_pin<K: Ord + Clone>(objs: &[Obj], pin2ep: &BTreeMap<K, BTreeSet<Node>>, node_map: &mut BTreeMap<Node, BTreeSet<usize>>) -> usize {
    for (cnt, locs) in (objs.len()..).zip(pin2ep.values()) {
        for &pr in locs {
            node_map.entry(pr).or_default().insert(cnt);
        }
    }
    pin2ep.len()
}

/// The search: which objects and terms the tree reaches.
fn astar(net: &NetShapes, node_map: &BTreeMap<Node, BTreeSet<usize>>, objs: &[Obj], nr: usize, n: usize) -> Vec<bool> {
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    for (pr, idxs) in node_map {
        let v: Vec<usize> = idxs.iter().copied().collect();
        for a in 0..v.len() {
            for &idx2 in &v[a + 1..] {
                let idx1 = v[a];
                if (idx1 >= nr) ^ (idx2 >= nr) {
                    let ro = if idx1 >= nr { idx2 } else { idx1 };
                    match objs[ro] {
                        Obj::Seg(_) => {
                            let s = seg(net, objs[ro]);
                            let valid = (s.begin == pr.0 && s.begin_trunc) || (s.end == pr.0 && s.end_trunc);
                            if !valid {
                                continue;
                            }
                        }
                        Obj::Via(k) => {
                            let via = net.vias[k].as_ref().expect("a live via");
                            if !via.bottom_connected && !via.top_connected {
                                continue;
                            }
                        }
                    }
                }
                adj[idx1].push(idx2);
                adj[idx2].push(idx1);
            }
        }
    }
    let mut on_path = vec![false; n];
    let mut visited = vec![false; n];
    let mut prev: Vec<i64> = vec![-1; n];
    let mut find_node = nr;
    while (find_node as i64) < n as i64 - 1 {
        // Smallest (cost, node, previous) first: a total order.
        let mut pq: BinaryHeap<Reverse<(i32, usize, i64)>> = BinaryHeap::new();
        if find_node == nr {
            pq.push(Reverse((0, nr, -1)));
        } else {
            for i in 0..n {
                if on_path[i] {
                    pq.push(Reverse((if i >= nr { 5 } else { 0 }, i, prev[i])));
                }
            }
        }
        let mut last: i64 = -1;
        while let Some(Reverse((cost, node, from))) = pq.pop() {
            if !on_path[node] && visited[node] {
                continue;
            }
            if node > nr && node < n && !visited[node] {
                visited[node] = true;
                prev[node] = from;
                last = node as i64;
                break;
            }
            visited[node] = true;
            prev[node] = from;
            for &nb in &adj[node] {
                if !visited[nb] {
                    pq.push(Reverse((cost + 1, nb, node as i64)));
                }
            }
        }
        while last != -1 && !on_path[last as usize] {
            on_path[last as usize] = true;
            last = prev[last as usize];
        }
        visited = on_path.clone();
        find_node += 1;
    }
    visited
}

#[allow(clippy::too_many_arguments)]
fn finish(tech: &Tech, net: &mut NetShapes, objs: &mut [Option<Obj>], visited: &[bool], g: usize, n: usize, node_map: &mut BTreeMap<Node, BTreeSet<usize>>, markers: &mut Vec<(usize, Rect)>) -> Result<(), String> {
    let mut reverse: BTreeMap<usize, BTreeSet<Node>> = BTreeMap::new();
    for (pr, idxs) in node_map.iter() {
        for &i in idxs {
            reverse.entry(i).or_default().insert(*pr);
        }
    }
    // Objects off the tree.
    for i in 0..visited.len() {
        if visited[i] {
            continue;
        }
        for pr in reverse.get(&i).into_iter().flatten() {
            if let Some(s) = node_map.get_mut(pr) {
                s.remove(&i);
            }
        }
        if i >= g {
            return Err("a term off the tree".into());
        }
        match objs[i] {
            Some(Obj::Seg(k)) => {
                let s = net.segs[k].take().expect("a live wire");
                markers.push((s.layer, s.bbox()));
            }
            Some(Obj::Via(k)) => {
                let v = net.vias[k].take().expect("a live via");
                let vd = &tech.via_defs[v.via];
                let b = vd.layer1_bbox();
                markers.push((vd.layer1, Rect { xl: b.xl + v.origin.0, yl: b.yl + v.origin.1, xh: b.xh + v.origin.0, yh: b.yh + v.origin.1 }));
            }
            None => {}
        }
        objs[i] = None;
    }
    // Nodes shared by two or more objects.
    reverse.clear();
    for (pr, idxs) in node_map.iter() {
        if idxs.len() == 1 {
            continue;
        }
        for &i in idxs {
            reverse.entry(i).or_default().insert(*pr);
        }
    }
    // A wire through a term's node, with no wire end or via there, splits (from the right).
    let mut ps_splits: BTreeMap<Node, usize> = BTreeMap::new();
    for (pr, idxs) in node_map.iter() {
        if !idxs.iter().any(|&i| i >= g) {
            continue;
        }
        let mut has_pin_ep = false;
        let mut ps_idx: Option<usize> = None;
        for &i in idxs {
            if i >= g {
                continue;
            }
            match objs[i] {
                Some(Obj::Seg(k)) => {
                    let s = net.segs[k].as_ref().expect("a live wire");
                    if s.begin == pr.0 || s.end == pr.0 {
                        has_pin_ep = true;
                        break;
                    }
                    ps_idx = Some(i);
                }
                Some(Obj::Via(_)) => {
                    has_pin_ep = true;
                    break;
                }
                None => {}
            }
        }
        if let (false, Some(p)) = (has_pin_ep, ps_idx) {
            ps_splits.insert(*pr, p);
        }
    }
    let mut added: Vec<usize> = Vec::new();
    for (&(split, _l), &idx1) in ps_splits.iter().rev() {
        let idx2 = n + added.len();
        let Some(Obj::Seg(k1)) = objs[idx1] else { continue };
        let ps1 = net.segs[k1].clone().expect("a live wire");
        let is_horz = ps1.begin.1 == ps1.end.1;
        let (mut pr1, mut pr2) = (BTreeSet::new(), BTreeSet::new());
        for &(pt, l) in reverse.get(&idx1).into_iter().flatten() {
            let (c, sc) = if is_horz { (pt.0, split.0) } else { (pt.1, split.1) };
            if c <= sc {
                pr1.insert((pt, l));
            } else if let Some(s) = node_map.get_mut(&(pt, l)) {
                s.remove(&idx1);
            }
            if c >= sc {
                pr2.insert((pt, l));
                node_map.entry((pt, l)).or_default().insert(idx2);
            }
        }
        reverse.insert(idx1, pr1);
        reverse.insert(idx2, pr2);
        let mut ps2 = ps1.clone();
        let k2 = net.segs.len();
        added.push(k2);
        let s1 = net.segs[k1].as_mut().expect("a live wire");
        (s1.end_trunc, s1.end_ext) = (true, 0);
        s1.end = split;
        (ps2.begin_trunc, ps2.begin_ext) = (true, 0);
        ps2.begin = split;
        net.segs.push(Some(ps2));
    }
    // Every wire shrinks to its outermost shared node.
    for (&idx, pts) in &reverse {
        let k = if idx < g {
            match objs[idx] {
                Some(Obj::Seg(k)) => k,
                _ => continue,
            }
        } else if idx >= n {
            added[idx - n]
        } else {
            continue;
        };
        let (Some(min), Some(max)) = (pts.iter().next(), pts.iter().next_back()) else { continue };
        let s = net.segs[k].as_mut().expect("a live wire");
        if s.begin < min.0 || max.0 < s.end {
            markers.push((s.layer, s.bbox()));
            (s.begin, s.end) = (min.0, max.0);
        }
    }
    // Patches off every remaining wire end and via layer.
    let mut valid: BTreeSet<Node> = BTreeSet::new();
    for s in net.segs.iter().flatten() {
        valid.insert((s.begin, s.layer));
        valid.insert((s.end, s.layer));
    }
    for v in net.vias.iter().flatten() {
        let vd = &tech.via_defs[v.via];
        valid.insert((v.origin, vd.layer1));
        valid.insert((v.origin, vd.layer2));
    }
    for p in net.patches.iter_mut() {
        let Some(pw) = p else { continue };
        if !valid.contains(&(pw.origin, pw.layer)) {
            let o = pw.offset;
            markers.push((pw.layer, Rect { xl: o.xl + pw.origin.0, yl: o.yl + pw.origin.1, xh: o.xh + pw.origin.0, yh: o.yh + pw.origin.1 }));
            *p = None;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn h(a: i32, b: i32, bt: bool, et: bool) -> Option<Seg> {
        Some(Seg { layer: 4, begin: (a, 0), end: (b, 0), width: 10, begin_trunc: bt, begin_ext: if bt { 0 } else { 5 }, end_trunc: et, end_ext: if et { 0 } else { 5 }, tapered: false })
    }

    fn run(segs: Vec<Option<Seg>>, pins: &[((i32, i32), usize)]) -> (NetShapes, Vec<(usize, Rect)>) {
        let t = crate::gc::tests::tech();
        let mut n = NetShapes { segs, vias: Vec::new(), patches: Vec::new() };
        let pins_at = |p: P, l: usize| -> Vec<usize> { pins.iter().filter(|(q, _)| l == 4 && *q == p).map(|(_, k)| *k).collect() };
        let m = check_net(&t, &mut n, &pins_at).expect("connected");
        (n, m)
    }

    /// Rule: overlapping wires on a track merge into the lowest, which takes the end style of the
    /// one reaching highest; the others leave the net's list.
    #[test]
    fn overlapping_wires_merge_into_the_lowest() {
        let (n, m) = run(vec![h(0, 100, true, false), h(50, 200, false, true)], &[((0, 0), 1), ((200, 0), 0)]);
        assert_eq!(n.segs, vec![Some(Seg { end: (200, 0), end_trunc: true, end_ext: 0, ..h(0, 100, true, false).unwrap() }), None]);
        assert!(m.is_empty());
    }

    /// Rule: a truncated end strictly inside an overlap splits the run there; the piece left over
    /// is a BARE new wire (no width, no extensions) appended to the net.
    #[test]
    fn a_split_leaves_a_bare_new_wire() {
        let (n, _) = run(vec![h(0, 200, true, true), h(50, 120, false, true)], &[((0, 0), 0), ((120, 0), 2), ((200, 0), 1)]);
        assert_eq!(n.segs[1], None);
        assert_eq!(n.segs[0].as_ref().map(|s| (s.begin, s.end, s.end_trunc, s.end_ext)), Some(((0, 0), (120, 0), true, 0)));
        assert_eq!(n.segs[2], Some(Seg { layer: 4, begin: (120, 0), end: (200, 0), width: 0, begin_trunc: true, begin_ext: 0, end_trunc: true, end_ext: 0, tapered: false }));
    }

    /// Rule: an object the tree does not reach is removed, a marker on its box.
    #[test]
    fn a_dangling_wire_is_removed_with_a_marker() {
        let (n, m) = run(vec![h(0, 100, true, true), h(300, 400, false, false)], &[((0, 0), 0), ((100, 0), 1)]);
        assert_eq!(n.segs[1], None);
        assert_eq!(m, vec![(4, Rect { xl: 295, yl: -5, xh: 405, yh: 5 })]);
    }

    /// Rule: a wire shrinks to its outermost node shared with another object (the marker is on
    /// its box BEFORE it shrinks); a wire end touching nothing does not count.
    #[test]
    fn a_wire_shrinks_to_its_outermost_shared_node() {
        let v = Some(Seg { layer: 4, begin: (100, 0), end: (100, 80), width: 10, begin_trunc: false, begin_ext: 5, end_trunc: true, end_ext: 0, tapered: false });
        let (n, m) = run(vec![h(0, 150, true, false), v], &[((0, 0), 0), ((100, 80), 1)]);
        assert_eq!(n.segs[0].as_ref().map(|s| (s.begin, s.end)), Some(((0, 0), (100, 0))));
        assert_eq!(m, vec![(4, Rect { xl: 0, yl: -5, xh: 155, yh: 5 })]);
    }

    /// Rule: a truncated begin at the run's OWN low is not a split point — two wires leaving the
    /// same pin point along a track just merge.
    #[test]
    fn a_shared_low_is_not_a_split_point() {
        let (n, _) = run(vec![h(0, 100, true, false), h(0, 200, true, true)], &[((0, 0), 0), ((200, 0), 1)]);
        assert_eq!(n.segs, vec![Some(Seg { end: (200, 0), end_trunc: true, end_ext: 0, ..h(0, 100, true, false).unwrap() }), None]);
    }

    fn v(x: i32, y0: i32, y1: i32, bt: bool, et: bool) -> Option<Seg> {
        Some(Seg { layer: 4, begin: (x, y0), end: (x, y1), width: 10, begin_trunc: bt, begin_ext: if bt { 0 } else { 5 }, end_trunc: et, end_ext: if et { 0 } else { 5 }, tapered: false })
    }

    /// Rule: in a merge, a victim reaching exactly as high as the merged wire hands over its end
    /// style (`>=`, not `>`).
    #[test]
    fn a_tie_at_the_top_hands_over_the_end_style() {
        let (n, _) = run(vec![h(0, 100, true, false), h(50, 100, false, true)], &[((0, 0), 0), ((100, 0), 1)]);
        assert_eq!(n.segs, vec![Some(Seg { end_trunc: true, end_ext: 0, ..h(0, 100, true, false).unwrap() }), None]);
    }

    /// Rule: a wire cut at a split point loses that end's extension (truncated, zero).
    #[test]
    fn a_split_end_loses_its_extension() {
        let (n, m) = run(vec![h(0, 200, true, false), h(120, 200, true, false)], &[((0, 0), 0), ((120, 0), 1)]);
        assert_eq!(n.segs[0], Some(Seg { end: (120, 0), end_trunc: true, end_ext: 0, ..h(0, 200, true, false).unwrap() }));
        // The piece past the split, merged with the far wire, is off the tree.
        assert_eq!((n.segs[1].clone(), m), (None, vec![(4, Rect { xl: 120, yl: -5, xh: 200, yh: 5 })]));
    }

    /// Rule: the first wire of a run ending truncated inside the run is a split point too.
    #[test]
    fn the_first_wires_truncated_end_splits_the_run() {
        let (n, m) = run(vec![h(0, 100, true, true), h(50, 200, false, false)], &[((0, 0), 0), ((100, 0), 1)]);
        assert_eq!(n.segs[0], Some(Seg { end: (100, 0), end_trunc: true, end_ext: 0, ..h(0, 100, true, true).unwrap() }));
        assert_eq!(n.segs[1], None);
        assert_eq!(m, vec![(4, Rect { xl: 100, yl: 0, xh: 200, yh: 0 })]);
    }

    /// Rule: a term reaches a wire only through a TRUNCATED end at the node — an extended end
    /// there does not connect, so the path runs through the stub (which stays, shrunk to the
    /// term's node: its far end touches nothing).
    #[test]
    fn an_extended_end_does_not_reach_a_term() {
        let (n, m) = run(vec![h(0, 100, true, false), v(100, 0, 50, true, false)], &[((0, 0), 0), ((100, 0), 1)]);
        assert_eq!(n.segs[1].as_ref().map(|s| (s.begin, s.end)), Some(((100, 0), (100, 0))));
        assert_eq!(m, vec![(4, Rect { xl: 95, yl: 0, xh: 105, yh: 55 })]);
    }

    /// Rule: after a redundant stub leaves, a term whose node now holds only a wire's interior
    /// splits that wire there (both halves truncated at the split).
    #[test]
    fn a_feedthrough_term_splits_the_wire() {
        // Term 2 at (0, 0) beside term 0, and at (100, 0) under the stub.
        let pins = [((0, 0), 0), ((0, 0), 2), ((200, 0), 1), ((100, 0), 2)];
        let (n, m) = run(vec![h(0, 200, true, true), v(100, 0, 40, true, false)], &pins);
        assert_eq!(m, vec![(4, Rect { xl: 95, yl: 0, xh: 105, yh: 45 })]);
        assert_eq!(n.segs[0].as_ref().map(|s| (s.begin, s.end, s.end_trunc)), Some(((0, 0), (100, 0), true)));
        assert_eq!(n.segs[1], None);
        assert_eq!(n.segs[2].as_ref().map(|s| (s.begin, s.end, s.begin_trunc, s.width)), Some(((100, 0), (200, 0), true, 10)));
    }

    /// Rule: a patch not on a remaining wire end or via layer is removed, a marker on its box.
    #[test]
    fn a_patch_off_the_wires_is_removed() {
        let t = crate::gc::tests::tech();
        let on = Patch { layer: 4, origin: (100, 0), offset: Rect { xl: -10, yl: -10, xh: 10, yh: 10 } };
        let off = Patch { layer: 4, origin: (50, 0), offset: Rect { xl: -10, yl: -10, xh: 10, yh: 10 } };
        let mut n = NetShapes { segs: vec![h(0, 100, true, true)], vias: Vec::new(), patches: vec![Some(on.clone()), Some(off)] };
        let pins = [((0, 0), 0), ((100, 0), 1)];
        let pins_at = |p: P, _l: usize| -> Vec<usize> { pins.iter().filter(|(q, _)| *q == p).map(|(_, k)| *k).collect() };
        let m = check_net(&t, &mut n, &pins_at).expect("connected");
        assert_eq!(n.patches, vec![Some(on), None]);
        assert_eq!(m, vec![(4, Rect { xl: 40, yl: -10, xh: 60, yh: 10 })]);
    }

    /// Rule: a zero-length wire's box is a vertical one (extensions along y, half width along x).
    #[test]
    fn a_zero_length_wire_is_vertical() {
        let s = Seg { layer: 4, begin: (0, 0), end: (0, 0), width: 10, begin_trunc: false, begin_ext: 7, end_trunc: false, end_ext: 7, tapered: false };
        assert_eq!(s.bbox(), Rect { xl: -5, yl: -7, xh: 5, yh: 7 });
    }

    /// Rule: a term left unreached is an error (the reference stops the run).
    #[test]
    fn an_unreached_term_is_an_error() {
        let t = crate::gc::tests::tech();
        let mut n = NetShapes { segs: vec![h(0, 100, true, true)], vias: Vec::new(), patches: Vec::new() };
        let pins: [((i32, i32), usize); 2] = [((0, 0), 0), ((100, 0), 0)];
        // Both ends on the SAME term: one term, nothing to search from it to — the reference's
        // pass count is terms − 1, so a single term is never marked reached.
        let pins_at = |p: P, _l: usize| -> Vec<usize> { pins.iter().filter(|(q, _)| *q == p).map(|(_, k)| *k).collect() };
        assert!(check_net(&t, &mut n, &pins_at).is_err());
    }
}
