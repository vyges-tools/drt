// SPDX-License-Identifier: Apache-2.0 AND BSL-1.0
//! Manhattan polygon sets: union, slicing into rectangles, and the MAXIMAL rectangles of a set.
//!
//! Derived from Boost.Polygon (Copyright 2008 Intel Corporation; Boost Software License 1.0,
//! http://www.boost.org/LICENSE_1_0.txt): `polygon_90_set_data` with a HORIZONTAL scan, its
//! `BooleanOp` OR, `form_rectangles` and `MaxCover`. The algorithms are kept step for step,
//! because their OUTPUT ORDER is observable: a caller that walks the maximal rectangles in order
//! and keeps the first of equal points depends on it.
//!
//! The set's data is a list of vertices `(y, (x, count))` — the scan's major coordinate is y, each
//! vertex opens (+1) or closes (−1) an x interval at that y.

use std::collections::{BTreeMap, BTreeSet};

/// A rectangle, inclusive of its boundary: `x` is `[xl, xh]`, `y` is `[yl, yh]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Rect {
    pub xl: i32,
    pub yl: i32,
    pub xh: i32,
    pub yh: i32,
}

impl Rect {
    pub fn new(xl: i32, yl: i32, xh: i32, yh: i32) -> Rect {
        Rect { xl: xl.min(xh), yl: yl.min(yh), xh: xl.max(xh), yh: yl.max(yh) }
    }
    pub fn dx(&self) -> i32 {
        self.xh - self.xl
    }
    pub fn dy(&self) -> i32 {
        self.yh - self.yl
    }
    /// `contains(rect, point)` with the boundary included.
    pub fn contains(&self, x: i32, y: i32) -> bool {
        x >= self.xl && x <= self.xh && y >= self.yl && y <= self.yh
    }
}

/// An interval `[lo, hi]` (an `interval_data`: constructed low-first).
type Ivl = (i32, i32);

fn ivl(a: i32, b: i32) -> Ivl {
    (a.min(b), a.max(b))
}

/// `contains(interval, interval, consider_touch)`.
fn contains(outer: Ivl, inner: Ivl, touch: bool) -> bool {
    let has = |v: i32| if touch { v >= outer.0 && v <= outer.1 } else { v > outer.0 && v < outer.1 };
    has(inner.0) && has(inner.1)
}

/// `intersect(lvalue, rvalue, consider_touch)`: ⛔ narrows `lvalue` to the intersection when there
/// is one — callers read the narrowed value.
fn intersect(l: &mut Ivl, r: Ivl, touch: bool) -> bool {
    let (lo, hi) = (l.0.max(r.0), l.1.min(r.1));
    let valid = if touch { lo <= hi } else { lo < hi };
    if valid {
        *l = (lo, hi);
    }
    valid
}

/// A set of Manhattan shapes (`polygon_90_set_data`, HORIZONTAL).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Polygon90Set {
    data: Vec<(i32, (i32, i32))>,
    dirty: bool,
    unsorted: bool,
}

impl Polygon90Set {
    pub fn new() -> Polygon90Set {
        Polygon90Set::default()
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// `+= rect`: its four corners as scan vertices.
    pub fn insert_rect(&mut self, r: Rect) {
        self.data.push((r.yl, (r.xl, 1)));
        self.data.push((r.yl, (r.xh, -1)));
        self.data.push((r.yh, (r.xl, -1)));
        self.data.push((r.yh, (r.xh, 1)));
        self.dirty = true;
        self.unsorted = true;
    }

    /// `clean`: sorted, then the OR of everything inserted.
    pub fn clean(&mut self) {
        if self.unsorted {
            self.data.sort();
            self.unsorted = false;
        }
        if self.dirty {
            apply_boolean_or(&mut self.data);
            self.dirty = false;
        }
    }

    /// `get_rectangles`: the set sliced along the scan into rectangles, in the order they close.
    pub fn rectangles(&mut self) -> Vec<Rect> {
        self.clean();
        form_rectangles(&self.data)
    }

    /// `get_max_rectangles`: every maximal rectangle inside the set, in `MaxCover`'s order.
    pub fn max_rectangles(&mut self) -> Vec<Rect> {
        let rects = self.rectangles();
        max_cover(&rects)
    }
}

/// `BooleanOp`: the scan data, and the iterator hint the reference keeps (as the key it points
/// at; `None` is `end`).
struct BooleanOp {
    scan: BTreeMap<i32, i32>,
    next: Option<i32>,
}

impl BooleanOp {
    fn advance_scan(&mut self) {
        self.next = self.scan.keys().next().copied();
    }
    fn lower_bound(&self, pos: i32) -> Option<i32> {
        self.scan.range(pos..).next().map(|(&k, _)| k)
    }
    fn prev(&self, key: Option<i32>) -> Option<i32> {
        match key {
            Some(k) => self.scan.range(..k).next_back().map(|(&k, _)| k),
            None => self.scan.keys().next_back().copied(),
        }
    }
    fn succ(&self, key: i32) -> Option<i32> {
        self.scan.range(key + 1..).next().map(|(&k, _)| k)
    }
    fn is_begin(&self, key: Option<i32>) -> bool {
        key == self.scan.keys().next().copied()
    }
    /// `lookup_`: the hint when it is at or past `pos`, else a fresh `lower_bound`.
    fn lookup(&mut self, pos: i32) -> Option<i32> {
        if let Some(k) = self.next {
            if k >= pos {
                return Some(k);
            }
        }
        self.next = self.lower_bound(pos);
        self.next
    }
    /// `insert_`: `std::map::insert` — an existing key keeps its value.
    fn insert(&mut self, pos: i32, count: i32) -> Option<i32> {
        self.scan.entry(pos).or_insert(count);
        self.next = Some(pos);
        self.next
    }
    fn evaluate(out: &mut Vec<(Ivl, i32)>, iv: Ivl, before: i32, after: i32) {
        let (b, a) = (before > 0, after > 0);
        let value = i32::from(!b & a) - i32::from(b & !a);
        if value != 0 {
            out.push((iv, value));
        }
    }
    /// `processInterval`.
    fn process_interval(&mut self, out: &mut Vec<(Ivl, i32)>, iv: Ivl, delta: i32) {
        let mut low_itr = self.lookup(iv.0);
        let mut high_itr = self.lookup(iv.1);
        if low_itr.is_none() {
            self.insert(iv.0, delta);
            self.insert(iv.1, 0);
            Self::evaluate(out, iv, 0, delta);
            return;
        }
        // Ensure the high end is a key.
        if high_itr.is_none_or(|k| k > iv.1) {
            let mut value = 0;
            if !self.is_begin(high_itr) {
                high_itr = self.prev(high_itr);
                value = self.scan[&high_itr.expect("not begin")];
            }
            self.next = high_itr;
            high_itr = self.insert(iv.1, value);
        }
        // Split the low interval if needed.
        let lk = low_itr.expect("checked");
        if lk > iv.0 {
            if !self.is_begin(low_itr) {
                let p = self.prev(low_itr).expect("not begin");
                self.next = Some(p);
                let v = self.scan[&p];
                low_itr = self.insert(iv.0, v);
            } else {
                self.next = low_itr;
                low_itr = self.insert(iv.0, 0);
            }
        }
        // The scan data inside the interval.
        let hk = high_itr.expect("a key");
        let mut k = low_itr.expect("a key");
        while k != hk {
            let before = self.scan[&k];
            let after = before + delta;
            self.scan.insert(k, after);
            let next = self.succ(k).expect("the high key follows");
            Self::evaluate(out, (k, next), before, after);
            k = next;
        }
        // Merge the bottom interval with the one below if they have the same count.
        let lk = low_itr.expect("a key");
        if !self.is_begin(low_itr) {
            let below = self.prev(low_itr).expect("not begin");
            if self.scan[&below] == self.scan[&lk] {
                self.scan.remove(&lk);
            }
        }
        // Merge the top interval with the one above if they have the same count.
        if !self.is_begin(high_itr) {
            let before_high = self.prev(high_itr).expect("not begin");
            if self.scan[&before_high] == self.scan[&hk] {
                self.scan.remove(&hk);
                high_itr = self.succ(before_high);
            }
        }
        self.next = high_itr;
    }
}

/// `applyBooleanOr`: the OR of vertex data sorted by `(y, x, count)`, as vertex data again.
fn apply_boolean_or(input: &mut Vec<(i32, (i32, i32))>) {
    let mut op = BooleanOp { scan: BTreeMap::new(), next: None };
    let mut output: Vec<(i32, (i32, i32))> = Vec::with_capacity(input.len());
    let (mut prev_pos, mut prev_y, mut count) = (i32::MAX, i32::MAX, 0i32);
    let mut container: Vec<(Ivl, i32)> = Vec::new();
    for &(pos, (y, c)) in input.iter() {
        if pos != prev_pos {
            op.advance_scan();
            prev_pos = pos;
            prev_y = y;
            count = c;
            continue;
        }
        if y != prev_y && count != 0 {
            container.clear();
            op.process_interval(&mut container, (prev_y, y), count);
            for &(iv, v) in &container {
                if output.last().is_some_and(|l| l.0 == prev_pos && l.1 .0 == iv.0 && l.1 .1 == -v) {
                    output.pop();
                } else {
                    output.push((prev_pos, (iv.0, v)));
                }
                output.push((prev_pos, (iv.1, -v)));
            }
        }
        prev_y = y;
        count += c;
    }
    *input = output;
}

/// `ScanLineToRects` state.
struct ScanLineToRects {
    /// `std::set` under `less_rectangle_concept(HORIZONTAL)`: `(xl, yl, xh, yh)` lexicographic.
    scan: BTreeSet<(i32, i32, i32, i32)>,
    have_current: bool,
    current: Rect,
    coordinate: i32,
}

fn key(r: &Rect) -> (i32, i32, i32, i32) {
    (r.xl, r.yl, r.xh, r.yh)
}
fn unkey(k: (i32, i32, i32, i32)) -> Rect {
    Rect { xl: k.0, yl: k.1, xh: k.2, yh: k.3 }
}

impl ScanLineToRects {
    fn next_major(&mut self, coordinate: i32) {
        if self.have_current {
            self.scan.insert(key(&self.current));
            self.have_current = false;
        }
        self.coordinate = coordinate;
    }

    /// `processEdge_`: the x interval `edge` at the current y.
    fn process_edge(&mut self, out: &mut Vec<Rect>, edge: Ivl) {
        let cc = self.coordinate;
        let mut edge_processed = false;
        if !self.scan.is_empty() {
            // lower_bound(rectangle(edge, edge)), then back up while the low end is past the edge.
            let probe = (edge.0, edge.0, edge.1, edge.1);
            let mut it: Option<(i32, i32, i32, i32)> = self.scan.range(probe..).next().copied();
            loop {
                let past = it.is_none_or(|k| k.0 > edge.0);
                let at_begin = it == self.scan.iter().next().copied();
                if !(past && !at_begin) {
                    break;
                }
                it = match it {
                    Some(k) => self.scan.range(..k).next_back().copied(),
                    None => self.scan.iter().next_back().copied(),
                };
            }
            while let Some(k) = it {
                let rect = unkey(k);
                if rect.xl > edge.1 {
                    break;
                }
                if rect.xh >= edge.0 {
                    if contains((rect.xl, rect.xh), edge, true) {
                        // A closing edge: write out, and put back up to two overhangs.
                        let mut tmp = rect;
                        if rect.yl < cc {
                            tmp.yh = cc;
                            out.push(tmp);
                        }
                        self.scan.remove(&k);
                        if tmp.xl < edge.0 {
                            let low = Rect { xl: tmp.xl, xh: edge.0, yl: cc, yh: cc };
                            self.scan.insert(key(&low));
                        }
                        if tmp.xh > edge.1 {
                            let high = Rect { xl: edge.1, xh: tmp.xh, yl: cc, yh: cc };
                            self.scan.insert(key(&high));
                        }
                        edge_processed = true;
                        break;
                    }
                    // An opening edge that touches the rectangle.
                    let mut tmp = rect;
                    if tmp.yl < cc {
                        tmp.yh = cc;
                        out.push(tmp);
                    }
                    let next = self.scan.range((std::ops::Bound::Excluded(k), std::ops::Bound::Unbounded)).next().copied();
                    self.scan.remove(&k);
                    it = next;
                    if self.have_current {
                        if self.current.xh >= edge.0 {
                            if !edge_processed && self.current.xh > edge.0 {
                                let mut tmp2 = self.current;
                                tmp2.xh = edge.0;
                                self.scan.insert(key(&tmp2));
                                if self.current.xh > edge.1 {
                                    self.current.xl = edge.1;
                                } else {
                                    self.have_current = false;
                                }
                            } else {
                                self.current.xh = edge.1.max(tmp.xh);
                            }
                        } else {
                            self.scan.insert(key(&self.current));
                            self.current = Rect { xl: tmp.xl.min(edge.0), xh: tmp.xh.max(edge.1), yl: cc, yh: cc };
                        }
                    } else {
                        self.have_current = true;
                        self.current = Rect { xl: tmp.xl.min(edge.0), xh: tmp.xh.max(edge.1), yl: cc, yh: cc };
                    }
                    edge_processed = true;
                    continue;
                }
                it = self.scan.range((std::ops::Bound::Excluded(k), std::ops::Bound::Unbounded)).next().copied();
            }
        }
        if !edge_processed {
            if self.have_current {
                if self.current.yh == cc && self.current.xh >= edge.0 {
                    if self.current.xh > edge.0 {
                        let mut tmp = self.current;
                        tmp.xh = edge.0;
                        self.scan.insert(key(&tmp));
                        if self.current.xh > edge.1 {
                            self.current.xl = edge.1;
                        } else {
                            self.have_current = false;
                        }
                        return;
                    }
                    self.current.xh = edge.1;
                    return;
                }
                self.scan.insert(key(&self.current));
                self.have_current = false;
            }
            let tmp = Rect { xl: edge.0, xh: edge.1, yl: cc, yh: cc };
            self.scan.insert(key(&tmp));
        }
    }
}

/// `form_rectangles` over clean vertex data.
fn form_rectangles(data: &[(i32, (i32, i32))]) -> Vec<Rect> {
    let mut out = Vec::new();
    let mut s = ScanLineToRects { scan: BTreeSet::new(), have_current: false, current: Rect { xl: 0, yl: 0, xh: 0, yh: 0 }, coordinate: i32::MAX };
    let mut prev_pos = i32::MAX;
    let mut i = 0;
    while i < data.len() {
        let pos = data[i].0;
        if pos != prev_pos {
            s.next_major(pos);
            prev_pos = pos;
        }
        let low = data[i].1 .0;
        let tmp = i;
        i += 1;
        let high = data[i].1 .0;
        s.process_edge(&mut out, (low, high));
        if data[i].1 .1.abs() > 1 {
            i = tmp; // the next edge begins from this vertex
        }
        i += 1;
    }
    out
}

/// A `MaxCover` node: its rectangle, its children in the DAG, and the paths traced through it.
struct Node {
    rect: Rect,
    children: Vec<usize>,
    traced: BTreeSet<Ivl>,
}

/// `MaxCover::getMaxCover(rects)`: the DAG over the slices (a slice's children are those it
/// touches above), then every node's maximal rectangles in node order.
fn max_cover(rects: &[Rect]) -> Vec<Rect> {
    let mut out = Vec::new();
    if rects.is_empty() {
        return out;
    }
    if rects.len() == 1 {
        out.push(rects[0]);
        return out;
    }
    let mut nodes: Vec<Node> = rects.iter().map(|&r| Node { rect: r, children: Vec::new(), traced: BTreeSet::new() }).collect();
    compute_dag(&mut nodes);
    for i in 0..nodes.len() {
        max_cover_node(&mut out, &mut nodes, i);
    }
    out
}

/// `computeDag`: leading (bottom) edges sorted, walked against trailing (top) edges in node order.
fn compute_dag(nodes: &mut [Node]) {
    let mut leading: Vec<((i32, Ivl), usize)> = nodes.iter().enumerate().map(|(i, n)| ((n.rect.yl, (n.rect.xl, n.rect.xh)), i)).collect();
    leading.sort_by(|a, b| a.0.cmp(&b.0));
    let (mut lb, mut tb) = (0usize, 0usize);
    while lb < leading.len() {
        let ((lead_y, lead_ivl), lead_node) = leading[lb];
        let trailing = nodes[tb].rect.yh;
        let tivl = (nodes[tb].rect.xl, nodes[tb].rect.xh);
        if lead_y < trailing {
            lb += 1;
            continue;
        }
        if lead_y > trailing {
            tb += 1;
            continue;
        }
        if lead_ivl.1 <= tivl.0 {
            lb += 1;
            continue;
        }
        if tivl.1 <= lead_ivl.0 {
            tb += 1;
            continue;
        }
        nodes[tb].children.push(lead_node);
        if lead_ivl.1 > tivl.1 {
            tb += 1;
            continue;
        }
        if tivl.1 > lead_ivl.1 {
            lb += 1;
            continue;
        }
        lb += 1;
        tb += 1;
    }
}

/// `getMaxCover(output, node, orient)`: a root's own maximal rectangles.
fn max_cover_node(out: &mut Vec<Rect>, nodes: &mut [Node], n: usize) {
    let rect_ivl = (nodes[n].rect.xl, nodes[n].rect.xh);
    if nodes[n].traced.contains(&rect_ivl) {
        return;
    }
    nodes[n].traced.insert(rect_ivl);
    if nodes[n].children.is_empty() {
        out.push(nodes[n].rect);
        return;
    }
    let mut write_out = true;
    let children = nodes[n].children.clone();
    let rect = nodes[n].rect;
    for c in children {
        max_cover_path(out, nodes, c, rect);
        let node_ivl = (nodes[c].rect.xl, nodes[c].rect.xh);
        if contains(node_ivl, rect_ivl, true) {
            write_out = false;
        }
    }
    if write_out {
        out.push(nodes[n].rect);
    }
}

/// `getMaxCover(output, node, orient, rect)`: the rectangles down every path from `node`, with
/// `rect` the rectangle carried in from above (the reference's explicit-stack walk).
fn max_cover_path(out: &mut Vec<Rect>, nodes: &mut [Node], start: usize, rect_in: Rect) {
    let mut stack: Vec<(usize, Rect, usize)> = Vec::new();
    let (mut node, mut rect, mut itr) = (start, rect_in, 0usize);
    loop {
        let mut rect_ivl = (rect.xl, rect.xh);
        let node_ivl = (nodes[node].rect.xl, nodes[node].rect.xh);
        let iresult = intersect(&mut rect_ivl, node_ivl, false);
        let tresult = !nodes[node].traced.contains(&rect_ivl);
        let y = ivl(rect.yl, nodes[node].rect.yh);
        let next_rect1 = Rect { xl: rect_ivl.0, xh: rect_ivl.1, yl: y.0, yh: y.1 };
        if iresult && tresult {
            nodes[node].traced.insert(rect_ivl);
            let mut write_out = true;
            for &c in &nodes[node].children {
                if contains((nodes[c].rect.xl, nodes[c].rect.xh), rect_ivl, true) {
                    write_out = false;
                }
            }
            if write_out {
                out.push(next_rect1);
            }
        }
        let n_children = nodes[node].children.len();
        if itr != n_children && iresult && tresult {
            stack.push((node, rect, itr));
            rect = next_rect1;
            node = nodes[node].children[itr];
            itr = 0;
        } else {
            if let Some((n, r, i)) = stack.pop() {
                node = n;
                rect = r;
                itr = i;
            }
            if itr != nodes[node].children.len() {
                itr += 1;
                if itr != nodes[node].children.len() {
                    stack.push((node, rect, itr));
                    let mut rect_ivl2 = (rect.xl, rect.xh);
                    intersect(&mut rect_ivl2, (nodes[node].rect.xl, nodes[node].rect.xh), false);
                    let y2 = ivl(rect.yl, nodes[node].rect.yh);
                    rect = Rect { xl: rect_ivl2.0, xh: rect_ivl2.1, yl: y2.0, yh: y2.1 };
                    node = nodes[node].children[itr];
                    itr = 0;
                }
            }
        }
        if stack.is_empty() && itr == nodes[node].children.len() {
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn set(rects: &[Rect]) -> Polygon90Set {
        let mut s = Polygon90Set::new();
        for &r in rects {
            s.insert_rect(r);
        }
        s
    }

    /// One rectangle is its own maximal rectangle.
    #[test]
    fn a_rectangle_is_its_own_cover() {
        let r = Rect::new(0, 0, 10, 5);
        assert_eq!(set(&[r]).max_rectangles(), vec![r]);
    }

    /// Overlapping rectangles merge; a union that is a rectangle has one maximal rectangle.
    #[test]
    fn overlapping_rectangles_merge() {
        let mut s = set(&[Rect::new(0, 0, 10, 5), Rect::new(5, 0, 20, 5)]);
        assert_eq!(s.max_rectangles(), vec![Rect::new(0, 0, 20, 5)]);
    }

    /// An L of two arms has two maximal rectangles, each arm extended through the corner.
    #[test]
    fn an_l_has_two_maximal_rectangles() {
        let mut s = set(&[Rect::new(0, 0, 30, 10), Rect::new(0, 10, 10, 40)]);
        let mut m = s.max_rectangles();
        m.sort();
        assert_eq!(m, vec![Rect::new(0, 0, 10, 40), Rect::new(0, 0, 30, 10)]);
    }

    /// A cross (plus sign) has two maximal rectangles, the bars.
    #[test]
    fn a_cross_has_its_two_bars() {
        let mut s = set(&[Rect::new(0, 10, 30, 20), Rect::new(10, 0, 20, 30)]);
        let mut m = s.max_rectangles();
        m.sort();
        assert_eq!(m, vec![Rect::new(0, 10, 30, 20), Rect::new(10, 0, 20, 30)]);
    }
}
