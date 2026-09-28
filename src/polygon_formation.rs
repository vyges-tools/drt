// SPDX-License-Identifier: Apache-2.0
//! The polygons (with holes) of a set of rectangles, each ring in the vertex order the design-rule
//! check reads it: Boost.Polygon's scanline polygon formation (`polygon_formation.hpp`, Boost
//! Software License 1.0) transcribed — partial polygons as chains of polylines hanging off
//! "active tails" in a map keyed by coordinate, joined as the scan meets them; a polygon comes
//! out when its two tails meet, and its ring is walked from the tail it closed on.
//!
//! Why the transcription and not any correct boundary: the check gathers each pin's edges ring by
//! ring from these rings, and that list is the bulk-load input of its edge query tree and the order
//! corners are visited in — both decide marker order. The start vertex of a ring is an accident of
//! the scan (an L starts at its lower left, a T at the left end of its bar, a U in its notch).
//!
//! Frame: the set is HORIZONTAL (the check's default), so the scan position is y and the edges at
//! a position are x intervals; below, "x" is the scan position and "y" the interval coordinate, as
//! the source names them.

use std::collections::BTreeMap;

use crate::polygon90::Rect;

/// A point.
pub type P = (i32, i32);

/// One polygon: its outer ring and its holes, each as the ring's vertices in order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PolygonWithHoles {
    pub outer: Vec<P>,
    pub holes: Vec<Vec<P>>,
}

const H: bool = false;
const V: bool = true;
const HEAD: bool = false;
const TAIL: bool = true;
const VERTICAL_HEAD: i32 = 1;
const HEAD_TO_TAIL: i32 = 2;
const TAIL_TO_TAIL: i32 = 4;

/// `PolyLine`: alternating coordinates, the chain links at each end and the state bits.
#[derive(Debug, Clone)]
struct PolyLine {
    pt: Vec<i32>,
    head: Option<usize>,
    tail: Option<usize>,
    state: i32,
}

/// `ActiveTail`: the polyline it is the tail of, the other tail of its partial polygon, its holes.
#[derive(Debug, Clone, Default)]
struct ActiveTail {
    tail: usize,
    other: usize,
    holes: Vec<usize>,
}

#[derive(Default)]
struct Formation {
    lines: Vec<PolyLine>,
    tails: Vec<ActiveTail>,
    map: BTreeMap<i32, usize>,
    out: Vec<usize>,
}

impl Formation {
    // ---- PolyLine ----

    fn new_line(&mut self, orient: bool, coord: i32, solid_to_right: bool) -> usize {
        self.lines.push(PolyLine { pt: vec![coord], head: None, tail: None, state: i32::from(orient) + (i32::from(solid_to_right) << 3) });
        self.lines.len() - 1
    }
    fn vertical_head(&self, l: usize) -> bool {
        self.lines[l].state & VERTICAL_HEAD != 0
    }
    fn odd_length(&self, l: usize) -> bool {
        (self.lines[l].pt.len() - 1) % 2 != 0
    }
    fn tail_orient(&self, l: usize) -> bool {
        self.vertical_head(l) ^ self.odd_length(l)
    }
    fn end_connectivity(&self, l: usize, end: bool) -> bool {
        if end {
            self.lines[l].state & TAIL_TO_TAIL != 0
        } else {
            self.lines[l].state & HEAD_TO_TAIL != 0
        }
    }
    fn num(&self, l: usize) -> usize {
        self.lines[l].pt.len()
    }
    fn next(&self, l: usize, end: bool) -> usize {
        (if end { self.lines[l].tail } else { self.lines[l].head }).expect("a linked polyline")
    }
    fn end_coord(&self, l: usize, end: bool) -> i32 {
        let pt = &self.lines[l].pt;
        if end {
            *pt.last().expect("a coordinate")
        } else {
            pt[0]
        }
    }
    fn segment_orient(&self, l: usize, index: usize) -> bool {
        self.vertical_head(l) ^ (index % 2 == 1)
    }
    /// `getPoint`: (horizontal, vertical) coordinates.
    fn point(&self, l: usize, index: usize) -> P {
        let c = self.lines[l].pt[index];
        let mut p = (c, c);
        let prev = if index == 0 {
            let h = self.lines[l].head.expect("a head");
            self.end_coord(h, self.end_connectivity(l, HEAD))
        } else {
            self.lines[l].pt[index - 1]
        };
        if self.segment_orient(l, index) == V {
            p.1 = prev;
        } else {
            p.0 = prev;
        }
        p
    }
    fn push_point(&mut self, l: usize, p: P) {
        let n = self.num(l);
        let vert = self.tail_orient(l) == V;
        if n > 0 {
            let e = self.point(l, n - 1);
            if if vert { p.1 == e.1 } else { p.0 == e.0 } {
                self.lines[l].pt.pop();
                return;
            }
        }
        self.lines[l].pt.push(if vert { p.1 } else { p.0 });
    }
    fn join_to_(&mut self, l: usize, this_end: bool, that: usize, end: bool) {
        let s = &mut self.lines[l];
        if this_end {
            s.tail = Some(that);
            s.state &= !TAIL_TO_TAIL;
            s.state |= i32::from(end) << 2;
        } else {
            s.head = Some(that);
            s.state &= !HEAD_TO_TAIL;
            s.state |= i32::from(end) << 1;
        }
    }
    fn join_to(&mut self, l: usize, this_end: bool, that: usize, end: bool) {
        self.join_to_(l, this_end, that, end);
        self.join_to_(that, end, l, this_end);
    }

    // ---- ActiveTail ----

    fn new_tail(&mut self) -> usize {
        self.tails.push(ActiveTail::default());
        self.tails.len() - 1
    }
    fn orient(&self, at: usize) -> bool {
        self.tail_orient(self.tails[at].tail)
    }
    fn coordinate(&self, at: usize) -> i32 {
        self.end_coord(self.tails[at].tail, TAIL)
    }
    /// `pushCoordinate`: the point at the tail's end extended by `coord` (a colinear step undone).
    fn push_coordinate(&mut self, at: usize, coord: i32) {
        let mut p = (coord, coord);
        // Set the perpendicular of the tail's orientation to the tail's coordinate.
        if self.orient(at) == H {
            p.1 = self.coordinate(at);
        } else {
            p.0 = self.coordinate(at);
        }
        let l = self.tails[at].tail;
        self.push_point(l, p);
    }
    fn copy_holes(&mut self, to: usize, from: usize) {
        let moved = std::mem::take(&mut self.tails[from].holes);
        self.tails[to].holes.extend(moved);
    }
    /// `addHole` without fracturing: the hole (and the holes it and its other tail carry) join this
    /// tail's list.
    fn add_hole(&mut self, at: usize, hole: usize) -> usize {
        self.tails[at].holes.push(hole);
        self.copy_holes(at, hole);
        let o = self.tails[hole].other;
        self.copy_holes(at, o);
        at
    }
    /// `joinChains` (the output-list form): two tails of one partial polygon close it — a hole when
    /// joined across solid (returned), else a finished polygon (output); otherwise the two partial
    /// polygons become one.
    fn join_chains(&mut self, at1: usize, at2: usize, solid: bool) -> Option<usize> {
        if self.tails[at1].other == at2 {
            if solid {
                return Some(at1);
            }
            self.out.push(at1);
            self.copy_holes(at1, at2);
            return None;
        }
        let (l1, l2) = (self.tails[at1].tail, self.tails[at2].tail);
        self.join_to(l1, TAIL, l2, TAIL);
        let (o1, o2) = (self.tails[at1].other, self.tails[at2].other);
        self.tails[o1].other = o2;
        self.tails[o2].other = o1;
        self.copy_holes(o1, at1);
        self.copy_holes(o1, at2);
        None
    }
    /// `createActiveTailsAsPair` (no fracturing): a vertical tail at `x` and a horizontal one at `y`,
    /// joined head to head; a hole passed goes to the vertical one.
    fn create_pair(&mut self, x: i32, y: i32, solid: bool, hole: Option<usize>) -> (usize, usize) {
        let (at1, at2) = (self.new_tail(), self.new_tail());
        let l1 = self.new_line(V, x, solid);
        let l2 = self.new_line(H, y, !solid);
        self.tails[at1].tail = l1;
        self.tails[at1].other = at2;
        self.tails[at2].tail = l2;
        self.tails[at2].other = at1;
        self.join_to(l1, HEAD, l2, HEAD);
        if let Some(h) = hole {
            self.add_hole(at1, h);
        }
        (at1, at2)
    }

    // ---- ScanLineToPolygonItrs ----

    /// `findAtNext`: the map entry at `key` looked up from a cursor (a key; `None` is the end).
    fn find_at_next(&self, pos: Option<i32>, key: i32) -> Option<i32> {
        match pos {
            None => self.map.contains_key(&key).then_some(key),
            Some(p) if p < key => self.map.contains_key(&key).then_some(key),
            Some(p) if p > key => None,
            Some(p) => Some(p),
        }
    }
    fn succ(&self, k: i32) -> Option<i32> {
        self.map.range(k + 1..).next().map(|(&k, _)| k)
    }
    /// `std::map::insert`: an existing key keeps its value.
    fn insert(&mut self, k: i32, v: usize) {
        self.map.entry(k).or_insert(v);
    }

    /// `processEdges`: the left and right edges at scan position `cx`, each sorted and maximal.
    fn process_edges(&mut self, cx: i32, left: &[(i32, i32)], right: &[(i32, i32)]) {
        let mut next_itr = self.map.keys().next().copied();
        let (mut li, mut ri) = (0usize, 0usize);
        let mut bottom_done = false;
        let mut current: Option<usize> = None;
        const MAX: i32 = i32::MAX;
        while li < left.len() || ri < right.len() {
            let mut edges = [(MAX, MAX), (MAX, MAX)];
            let mut have_next = true;
            if li < left.len() {
                edges[0] = left[li];
            } else {
                have_next = false;
            }
            if ri < right.len() {
                edges[1] = right[ri];
            } else {
                have_next = false;
            }
            let trailing = edges[1].0 < edges[0].0;
            let edge = edges[usize::from(trailing)];
            let next_edge = edges[usize::from(!trailing)];
            if !bottom_done {
                if let Some(k) = self.find_at_next(next_itr, edge.0) {
                    // An edge in the map at this edge's low end turns upward: the current tail.
                    let tail = self.map[&k];
                    let c = match current {
                        Some(c) => self.add_hole(tail, c),
                        None => tail,
                    };
                    self.push_coordinate(c, cx);
                    current = Some(c);
                    next_itr = self.succ(k);
                    self.map.remove(&k);
                } else {
                    let (a1, a2) = self.create_pair(cx, edge.0, !trailing, current);
                    current = Some(a1);
                    self.insert(edge.0, a2);
                }
            }
            if have_next && edge.1 == next_edge.0 {
                bottom_done = true;
                let Some(k) = self.find_at_next(next_itr, edge.1) else { return };
                let cur = current.expect("a current tail");
                if trailing {
                    let tail = self.map[&k];
                    self.join_chains(cur, tail, false);
                    let (a1, a2) = self.create_pair(cx, edge.1, true, None);
                    current = Some(a1);
                    self.map.insert(k, a2);
                } else {
                    self.push_coordinate(cur, edge.1);
                    let t = self.map[&k];
                    self.push_coordinate(t, cx);
                    self.map.insert(k, cur);
                    current = Some(t);
                }
                next_itr = self.succ(k);
            } else {
                bottom_done = false;
                if let Some(k) = self.find_at_next(next_itr, edge.1) {
                    let tail = self.map[&k];
                    current = self.join_chains(current.expect("a current tail"), tail, !trailing);
                    next_itr = self.succ(k);
                    if let Some(c) = current {
                        let next_y = next_itr.unwrap_or(MAX);
                        let left_y = if li + 1 < left.len() { left[li + 1].0 } else { MAX };
                        let right_y = next_edge.0;
                        if !have_next || (next_y < left_y && next_y < right_y) {
                            let n = self.map[&next_itr.expect("an edge above the hole")];
                            self.add_hole(n, c);
                            current = None;
                        }
                    }
                    self.map.remove(&k);
                } else {
                    let c = current.expect("a current tail");
                    self.push_coordinate(c, edge.1);
                    self.insert(edge.1, c);
                    current = None;
                }
            }
            li += usize::from(!trailing);
            ri += usize::from(trailing);
        }
    }

    /// `ActiveTail::iterator` from `begin(isHole, HORIZONTAL)` to its end: the ring's compact
    /// coordinates (it joins the chain's two tails, as the source's constructor does).
    fn compact(&mut self, at: usize, is_hole: bool) -> Vec<i32> {
        // `!isHole ^ (orient == HORIZONTAL)`, the orientation HORIZONTAL: a hole switches tails.
        let at = if is_hole { self.tails[at].other } else { at };
        let mut start_end = TAIL;
        let t = self.tails[at].tail;
        let mut pline = t;
        let mut index = self.num(t).saturating_sub(1);
        let (pend, iend);
        if self.orient(at) == V {
            pend = t;
            iend = self.num(pend) - 1;
            if index == 0 {
                pline = self.next(t, HEAD);
                if self.end_connectivity(t, HEAD) == TAIL {
                    index = self.num(pline) - 1;
                } else {
                    start_end = HEAD;
                    index = 0;
                }
            } else {
                index -= 1;
            }
        } else {
            pend = self.tails[self.tails[at].other].tail;
            iend = self.num(pend).saturating_sub(1);
        }
        let o = self.tails[self.tails[at].other].tail;
        self.join_to(t, TAIL, o, TAIL);
        let mut out = Vec::new();
        loop {
            out.push(self.lines[pline].pt[index]);
            if pline == pend && index == iend {
                break;
            }
            if start_end == HEAD {
                index += 1;
                if index == self.num(pline) {
                    let e = self.end_connectivity(pline, TAIL);
                    pline = self.next(pline, TAIL);
                    if e == TAIL {
                        start_end = TAIL;
                        index = self.num(pline) - 1;
                    } else {
                        index = 0;
                    }
                }
            } else if index == 0 {
                let e = self.end_connectivity(pline, HEAD);
                pline = self.next(pline, HEAD);
                if e == TAIL {
                    index = self.num(pline) - 1;
                } else {
                    start_end = HEAD;
                    index = 0;
                }
            } else {
                index -= 1;
            }
        }
        out
    }
}

/// `iterator_compact_to_points` over a ring's compact coordinates (x first): its points, the
/// closing point added when the last x is not the first.
fn compact_to_points(c: &[i32]) -> Vec<P> {
    if c.len() < 2 {
        return Vec::new();
    }
    let n = c.len();
    let first_x = c[0];
    let mut pt = (c[0], c[1]);
    let mut out = vec![pt];
    let mut i = 1;
    let mut horizontal = true;
    loop {
        let prev = i;
        i += 1;
        if i == n {
            if pt.0 != first_x {
                i = prev;
                pt.0 = first_x;
                out.push(pt);
                continue;
            }
            break;
        }
        if horizontal {
            pt.0 = c[i];
        } else {
            pt.1 = c[i];
        }
        horizontal = !horizontal;
        out.push(pt);
    }
    out
}

/// Maximal intervals of a union of intervals, ascending.
fn union(mut v: Vec<(i32, i32)>) -> Vec<(i32, i32)> {
    v.sort_unstable();
    let mut out: Vec<(i32, i32)> = Vec::new();
    for (a, b) in v {
        match out.last_mut() {
            Some(l) if a <= l.1 => l.1 = l.1.max(b),
            _ => out.push((a, b)),
        }
    }
    out
}

/// `a` minus `b` (both maximal, ascending): maximal, ascending.
fn minus(a: &[(i32, i32)], b: &[(i32, i32)]) -> Vec<(i32, i32)> {
    let mut out = Vec::new();
    for &(lo, hi) in a {
        let mut x = lo;
        for &(c, d) in b {
            if d <= x || c >= hi {
                continue;
            }
            if c > x {
                out.push((x, c));
            }
            x = x.max(d);
        }
        if x < hi {
            out.push((x, hi));
        }
    }
    out
}

/// `polygon_90_set_data::get` into polygons with holes, for the union of `rects`: the polygons in
/// the order the scan finishes them, each ring's vertices in its order.
pub fn polygons_with_holes(rects: &[Rect]) -> Vec<PolygonWithHoles> {
    let mut ys: Vec<i32> = rects.iter().filter(|r| r.xl < r.xh && r.yl < r.yh).flat_map(|r| [r.yl, r.yh]).collect();
    ys.sort_unstable();
    ys.dedup();
    let mut f = Formation::default();
    let mut result = Vec::new();
    for &y in &ys {
        let above = union(rects.iter().filter(|r| r.xl < r.xh && r.yl <= y && y < r.yh).map(|r| (r.xl, r.xh)).collect());
        let below = union(rects.iter().filter(|r| r.xl < r.xh && r.yl < y && y <= r.yh).map(|r| (r.xl, r.xh)).collect());
        let left = minus(&above, &below);
        let right = minus(&below, &above);
        f.out.clear();
        f.process_edges(y, &left, &right);
        for at in std::mem::take(&mut f.out) {
            let outer = compact_to_points(&f.compact(at, false));
            let hs = f.tails[at].holes.clone();
            let holes = hs.into_iter().map(|h| compact_to_points(&f.compact(h, true))).collect();
            result.push(PolygonWithHoles { outer, holes });
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Rings as the reference's own Boost.Polygon 1.89 `get` prints them (probe on the box).
    #[test]
    fn rings_start_where_the_scan_closed_them() {
        let r = |xl, yl, xh, yh| Rect::new(xl, yl, xh, yh);
        let one = |rs: &[Rect]| polygons_with_holes(rs);
        assert_eq!(one(&[r(0, 0, 10, 20)]), vec![PolygonWithHoles { outer: vec![(0, 0), (10, 0), (10, 20), (0, 20)], holes: vec![] }]);
        assert_eq!(one(&[r(0, 0, 30, 10), r(0, 0, 10, 30)])[0].outer, vec![(0, 0), (30, 0), (30, 10), (10, 10), (10, 30), (0, 30)]);
        assert_eq!(one(&[r(0, 0, 30, 10), r(0, 0, 10, 30), r(20, 0, 30, 30)])[0].outer, vec![(20, 10), (10, 10), (10, 30), (0, 30), (0, 0), (30, 0), (30, 30), (20, 30)]);
        assert_eq!(one(&[r(0, 20, 30, 30), r(10, 0, 20, 30)])[0].outer, vec![(0, 20), (10, 20), (10, 0), (20, 0), (20, 20), (30, 20), (30, 30), (0, 30)]);
        let hole = one(&[r(0, 0, 30, 10), r(0, 20, 30, 30), r(0, 0, 10, 30), r(20, 0, 30, 30)]);
        assert_eq!(hole, vec![PolygonWithHoles { outer: vec![(0, 0), (30, 0), (30, 30), (0, 30)], holes: vec![vec![(20, 10), (10, 10), (10, 20), (20, 20)]] }]);
        let two = one(&[r(0, 0, 10, 10), r(20, 5, 30, 15)]);
        assert_eq!(two.iter().map(|p| p.outer[0]).collect::<Vec<_>>(), vec![(0, 0), (20, 5)]);
    }
}
