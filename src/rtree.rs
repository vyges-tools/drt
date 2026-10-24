// SPDX-License-Identifier: Apache-2.0
//! A bulk-loaded R-tree whose QUERY ORDER is part of its contract.
//!
//! Derived from Boost.Geometry's R-tree packing (`index/detail/rtree/pack_create.hpp`; Boost
//! Software License 1.0, http://www.boost.org/LICENSE_1_0.txt) and the LLVM C++ library's
//! `nth_element` (Apache-2.0 WITH LLVM-exception), whose exact permutation it reproduces.
//!
//! The router takes objects from spatial queries in the order the tree returns them, and that
//! order decides results (it numbers the objects, and numbers break ties). So the tree is built
//! exactly one way and queried exactly one way:
//!
//! - nodes hold at most 16 entries (at least 4) — other limits through [`Limits`];
//! - building: every value's box centre (integer halves, truncated) is an entry; the entries are
//!   split recursively — at a median count of whole subtrees, along the LONGER side of the
//!   current bounding box (x on a tie), by a selection that leaves the entries in one particular
//!   permutation ([`nth_element`]), the box halved at its centre for each side — until a slice
//!   fits one leaf, whose values keep the slice's order;
//! - querying: depth first, children and values in stored order, boxes intersecting when they
//!   touch.

use crate::polygon90::Rect;

/// A node's entry limits: at most `max`, at least `min` (below it a node underflows on removal).
/// The router's trees are 16/4 ([`Limits::DEFAULT`]); an R*-tree of `n` keeps `max = n`,
/// `min = (n * 3) / 10` (at least 1) — `rstar<4>` is 4/1.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Limits {
    pub max: usize,
    pub min: usize,
}

impl Limits {
    pub const DEFAULT: Limits = Limits { max: 16, min: 4 };

    /// The limits of an R*-tree of at most `max` entries.
    pub fn rstar(max: usize) -> Limits {
        Limits { max, min: ((max * 3) / 10).max(1) }
    }
}

enum Node {
    Internal(Vec<(Rect, usize)>),
    Leaf(Vec<usize>),
}

pub struct PackedRTree<T> {
    values: Vec<(Rect, T)>,
    nodes: Vec<Node>,
    root: Option<usize>,
    limits: Limits,
}

fn touches(a: &Rect, b: &Rect) -> bool {
    a.xl <= b.xh && b.xl <= a.xh && a.yl <= b.yh && b.yl <= a.yh
}

fn expand(b: Option<Rect>, r: &Rect) -> Rect {
    match b {
        None => *r,
        Some(b) => Rect { xl: b.xl.min(r.xl), yl: b.yl.min(r.yl), xh: b.xh.max(r.xh), yh: b.yh.max(r.yh) },
    }
}

#[derive(Clone, Copy)]
struct Counts {
    maxc: usize,
    minc: usize,
}

impl<T> PackedRTree<T> {
    /// Built from `values` in the order given.
    pub fn new(values: Vec<(Rect, T)>) -> PackedRTree<T> {
        PackedRTree::with_limits(values, Limits::DEFAULT)
    }

    /// Built from `values` in the order given, nodes held to `limits` (the packing is the same
    /// for every split algorithm).
    pub fn with_limits(values: Vec<(Rect, T)>, limits: Limits) -> PackedRTree<T> {
        let mut t = PackedRTree { values, nodes: Vec::new(), root: None, limits };
        let n = t.values.len();
        if n == 0 {
            return t;
        }
        let Limits { max, min } = limits;
        // Each centre: half the SUM of the sides, truncated.
        let mut entries: Vec<((i32, i32), usize)> = t.values.iter().enumerate().map(|(i, (r, _))| (((r.xl + r.xh) / 2, (r.yl + r.yh) / 2), i)).collect();
        let hint = t.values.iter().fold(None, |b, (r, _)| Some(expand(b, r))).expect("a value");
        let mut counts = Counts { maxc: 1, minc: 1 };
        let mut smax = max;
        while smax < n {
            counts.maxc = smax;
            smax *= max;
        }
        counts.minc = min * (counts.maxc / max);
        let (_, root) = t.per_level(&mut entries, hint, counts);
        t.root = Some(root);
        t
    }

    fn per_level(&mut self, entries: &mut [((i32, i32), usize)], hint: Rect, counts: Counts) -> (Rect, usize) {
        if counts.maxc <= 1 {
            let ids: Vec<usize> = entries.iter().map(|e| e.1).collect();
            let b = ids.iter().fold(None, |b, &i| Some(expand(b, &self.values[i].0))).expect("a value");
            self.nodes.push(Node::Leaf(ids));
            return (b, self.nodes.len() - 1);
        }
        let max = self.limits.max;
        let next = Counts { maxc: counts.maxc / max, minc: counts.minc / max };
        let mut children = Vec::new();
        let mut b = None;
        self.per_level_packets(entries, hint, counts, next, &mut children, &mut b);
        self.nodes.push(Node::Internal(children));
        (b.expect("a child"), self.nodes.len() - 1)
    }

    fn per_level_packets(&mut self, entries: &mut [((i32, i32), usize)], hint: Rect, counts: Counts, next: Counts, children: &mut Vec<(Rect, usize)>, b: &mut Option<Rect>) {
        let count = entries.len();
        if count <= counts.maxc {
            let (cb, node) = self.per_level(entries, hint, next);
            children.push((cb, node));
            *b = Some(expand(*b, &cb));
            return;
        }
        let median = median_count(count, counts);
        let along_y = hint.yh - hint.yl > hint.xh - hint.xl;
        let (mut left, mut right) = (hint, hint);
        if along_y {
            nth_element(entries, median, |a, b| a.0 .1 < b.0 .1);
            let c = hint.yl + (hint.yh - hint.yl) / 2;
            left.yh = c;
            right.yl = c;
        } else {
            nth_element(entries, median, |a, b| a.0 .0 < b.0 .0);
            let c = hint.xl + (hint.xh - hint.xl) / 2;
            left.xh = c;
            right.xl = c;
        }
        let (lo, hi) = entries.split_at_mut(median);
        self.per_level_packets(lo, left, counts, next, children, b);
        self.per_level_packets(hi, right, counts, next, children, b);
    }

    /// Every value whose box touches `q`, in the tree's order.
    pub fn query(&self, q: &Rect) -> Vec<&(Rect, T)> {
        let mut out = Vec::new();
        if let Some(r) = self.root {
            self.walk(r, q, &mut out);
        }
        out
    }

    /// The ONE value nearest the point `p` among those `keep` accepts — Boost.Geometry 1.89's
    /// `nearest(p, 1) && satisfies(keep)` query (`visitors/distance_query.hpp`), in its order:
    ///
    /// - distances are COMPARABLE ones (squared, as `f64`): point to a value's box, point to a
    ///   child's box (0 inside);
    /// - best first: the root's children, then repeatedly the closest pending branch — a
    ///   `std::priority_queue` (libc++ 22's heap moves, [`BranchHeap`]) ranked by distance, then
    ///   the LOWER level first — skipping a branch once the kept value is no farther than it;
    /// - `keep` is asked of values only, never of branches;
    /// - a later value replaces the kept one only when STRICTLY nearer, so on a tie the first
    ///   value reached wins.
    pub fn nearest_one(&self, p: (i32, i32), keep: impl Fn(&T) -> bool) -> Option<&(Rect, T)> {
        let root = self.root?;
        let comparable = |r: &Rect| comparable(p, r);
        let mut best: Option<(f64, usize)> = None;
        let ignore = |best: &Option<(f64, usize)>, d: f64| matches!(best, Some((b, _)) if *b <= d);
        let mut branches = BranchHeap::default();
        let (mut node, mut level) = (root, self.leaf_level());
        loop {
            match &self.nodes[node] {
                Node::Internal(children) => {
                    for (b, c) in children {
                        let d = comparable(b);
                        if !ignore(&best, d) {
                            branches.push((d, level - 1, *c));
                        }
                    }
                }
                Node::Leaf(ids) => {
                    for &i in ids {
                        if !keep(&self.values[i].1) {
                            continue;
                        }
                        let d = comparable(&self.values[i].0);
                        if best.is_none_or(|(b, _)| d < b) {
                            best = Some((d, i));
                        }
                    }
                }
            }
            match branches.top() {
                Some(&(d, _, _)) if !ignore(&best, d) => {}
                _ => break,
            }
            let (_, l, n) = branches.pop();
            node = n;
            level = l;
        }
        best.map(|(_, i)| &self.values[i])
    }

    /// The root's level counted up from the leaves (a root that is a leaf is level 0).
    fn leaf_level(&self) -> usize {
        let (mut node, mut level) = (self.root.expect("a root"), 0);
        while let Node::Internal(children) = &self.nodes[node] {
            node = children[0].1;
            level += 1;
        }
        level
    }

    fn walk<'a>(&'a self, node: usize, q: &Rect, out: &mut Vec<&'a (Rect, T)>) {
        match &self.nodes[node] {
            Node::Internal(children) => {
                for (b, c) in children {
                    if touches(b, q) {
                        self.walk(*c, q, out);
                    }
                }
            }
            Node::Leaf(ids) => {
                for &i in ids {
                    if touches(&self.values[i].0, q) {
                        out.push(&self.values[i]);
                    }
                }
            }
        }
    }
}

/// The comparable (squared, as `f64`) distance from the point `p` to the box `r`, 0 inside.
fn comparable(p: (i32, i32), r: &Rect) -> f64 {
    let axis = |v: i32, lo: i32, hi: i32| -> f64 {
        if v < lo {
            f64::from(lo) - f64::from(v)
        } else if v > hi {
            f64::from(v) - f64::from(hi)
        } else {
            0.0
        }
    };
    let (dx, dy) = (axis(p.0, r.xl, r.xh), axis(p.1, r.yl, r.yh));
    dx * dx + dy * dy
}

/// The pending branches of a nearest query:`std::priority_queue<branch_data, vector,
/// branch_data_comp>` as libc++ 22 runs it. `comp(a, b)` — `a` ranks BELOW `b` — is "farther, or
/// as far and higher up", so the top is the nearest, lowest branch; branches equal on both are
/// ordered by the heap's moves alone (`push_heap` sifts up; `pop_heap` drives the hole to a leaf
/// through the larger child, Floyd's way, fills it with the last element and sifts that up).
#[derive(Default)]
struct BranchHeap {
    v: Vec<(f64, usize, usize)>,
}

impl BranchHeap {
    fn below(a: &(f64, usize, usize), b: &(f64, usize, usize)) -> bool {
        a.0 > b.0 || (a.0 == b.0 && a.1 > b.1)
    }
    fn sift_up(&mut self, len: usize) {
        if len <= 1 {
            return;
        }
        let mut last = len - 1;
        let mut parent = (len - 2) / 2;
        if Self::below(&self.v[parent], &self.v[last]) {
            let t = self.v[last];
            loop {
                self.v[last] = self.v[parent];
                last = parent;
                if parent == 0 {
                    break;
                }
                parent = (parent - 1) / 2;
                if !Self::below(&self.v[parent], &t) {
                    break;
                }
            }
            self.v[last] = t;
        }
    }
    fn push(&mut self, x: (f64, usize, usize)) {
        self.v.push(x);
        let n = self.v.len();
        self.sift_up(n);
    }
    fn top(&self) -> Option<&(f64, usize, usize)> {
        self.v.first()
    }
    fn clear(&mut self) {
        self.v.clear();
    }
    fn pop(&mut self) -> (f64, usize, usize) {
        let len = self.v.len();
        let top = self.v[0];
        if len > 1 {
            let mut hole = 0usize;
            loop {
                let mut child = 2 * hole + 1;
                if child + 1 < len && Self::below(&self.v[child], &self.v[child + 1]) {
                    child += 1;
                }
                self.v[hole] = self.v[child];
                hole = child;
                if child > (len - 2) / 2 {
                    break;
                }
            }
            let last = len - 1;
            if hole == last {
                self.v[hole] = top;
            } else {
                self.v[hole] = self.v[last];
                self.v[last] = top;
                self.sift_up(hole + 1);
            }
        }
        self.v.pop();
        top
    }
}

/// How many entries go to the first half of a split: whole subtrees, never leaving a remainder
/// below the minimum.
fn median_count(count: usize, c: Counts) -> usize {
    let n = count / c.maxc;
    let r = count % c.maxc;
    let mut median = (n / 2) * c.maxc;
    if r != 0 {
        if c.minc <= r {
            median = n.div_ceil(2) * c.maxc;
        } else {
            let cm = count - c.minc;
            let n = cm / c.maxc;
            let r = cm % c.maxc;
            if r == 0 {
                median = n.div_ceil(2) * c.maxc;
            } else if n == 0 {
                median = r;
            } else {
                median = (n + 2) / 2 * c.maxc;
            }
        }
    }
    median
}

/// Sorts three positions; whether anything moved.
fn sort3<E>(v: &mut [E], x: usize, y: usize, z: usize, less: &impl Fn(&E, &E) -> bool) -> bool {
    if !less(&v[y], &v[x]) {
        if !less(&v[z], &v[y]) {
            return false;
        }
        v.swap(y, z);
        if less(&v[y], &v[x]) {
            v.swap(x, y);
        }
        return true;
    }
    if less(&v[z], &v[y]) {
        v.swap(x, z);
        return true;
    }
    v.swap(x, y);
    if less(&v[z], &v[y]) {
        v.swap(y, z);
    }
    true
}

fn selection_sort<E>(v: &mut [E], first: usize, last: usize, less: &impl Fn(&E, &E) -> bool) {
    let mut f = first;
    while f != last - 1 {
        let mut m = f;
        for i in f + 1..last {
            if less(&v[i], &v[m]) {
                m = i;
            }
        }
        if m != f {
            v.swap(f, m);
        }
        f += 1;
    }
}

/// Puts the element that belongs at `nth` there, smaller ones before it and the rest after —
/// in one exact permutation: median of three, a guarded partition that keeps equal elements
/// right of the pivot, an early return when a side is already sorted, selection sort for 7 or
/// fewer. (The permutation is the one the LLVM C++ library's `nth_element` produces.)
pub fn nth_element<E>(v: &mut [E], nth: usize, less: impl Fn(&E, &E) -> bool) {
    let (mut first, mut last) = (0usize, v.len());
    loop {
        if nth == last {
            return;
        }
        let len = last - first;
        match len {
            0 | 1 => return,
            2 => {
                if less(&v[last - 1], &v[first]) {
                    v.swap(first, last - 1);
                }
                return;
            }
            3 => {
                sort3(v, first, first + 1, last - 1, &less);
                return;
            }
            _ => {}
        }
        if len <= 7 {
            selection_sort(v, first, last, &less);
            return;
        }
        let mut m = first + len / 2;
        let lm1 = last - 1;
        let mut swaps = u32::from(sort3(v, first, m, lm1, &less));
        let mut i = first;
        let mut j = lm1;
        if !less(&v[i], &v[m]) {
            // The first equals the pivot: look for a guard for the downward scan.
            let found = loop {
                j -= 1;
                if i == j {
                    break false;
                }
                if less(&v[j], &v[m]) {
                    break true;
                }
            };
            if found {
                v.swap(i, j);
                swaps += 1;
            } else {
                // Everything is at least the first: split off the elements equal to it.
                i += 1;
                j = last - 1;
                if !less(&v[first], &v[j]) {
                    loop {
                        if i == j {
                            return;
                        } else if less(&v[first], &v[i]) {
                            v.swap(i, j);
                            swaps += 1;
                            i += 1;
                            break;
                        }
                        i += 1;
                    }
                }
                if i == j {
                    return;
                }
                loop {
                    while !less(&v[first], &v[i]) {
                        i += 1;
                    }
                    loop {
                        j -= 1;
                        if !less(&v[first], &v[j]) {
                            break;
                        }
                    }
                    if i >= j {
                        break;
                    }
                    v.swap(i, j);
                    swaps += 1;
                    i += 1;
                }
                if nth < i {
                    return;
                }
                first = i;
                continue;
            }
        }
        i += 1;
        if i < j {
            loop {
                while less(&v[i], &v[m]) {
                    i += 1;
                }
                loop {
                    j -= 1;
                    if less(&v[j], &v[m]) {
                        break;
                    }
                }
                if i >= j {
                    break;
                }
                v.swap(i, j);
                swaps += 1;
                if m == i {
                    m = j;
                }
                i += 1;
            }
        }
        if i != m && less(&v[m], &v[i]) {
            v.swap(i, m);
            swaps += 1;
        }
        if nth == i {
            return;
        }
        if swaps == 0 {
            // Perfectly partitioned: the side holding `nth` may already be sorted.
            let (from, to) = if nth < i { (first, i) } else { (i, last) };
            let (mut j2, mut m2) = (from, from);
            let sorted = loop {
                j2 += 1;
                if j2 == to {
                    break true;
                }
                if less(&v[j2], &v[m2]) {
                    break false;
                }
                m2 = j2;
            };
            if sorted {
                return;
            }
        }
        if nth < i {
            last = i;
        } else {
            first = i + 1;
        }
    }
}


/// A packed R-tree that is then UPDATED: values inserted and removed one at a time, exactly as
/// the reference's tree does it, so its query order stays the reference's.
///
/// Derived from Boost.Geometry's R-tree visitors (`index/detail/rtree/visitors/insert.hpp`,
/// `remove.hpp`) and its quadratic split (`quadratic/redistribute_elements.hpp`; Boost Software
/// License 1.0):
///
/// - insert: from the root, into the child whose box grows least in area (then the smaller
///   area; the first on a tie), that child's box grown on the way down; a node over the maximum (16)
///   splits — the two seeds are the pair wasting the most area together (the first such pair),
///   then entries are taken from the BACK, each the one whose area increase differs most
///   between the groups (the last scanned on a tie, scanning back to front), to the group it
///   grows less (then the smaller group box, then the group with fewer entries), a group that
///   needs every remaining entry to reach the minimum (4) taking them; the node keeps the first group, a new
///   node takes the second — appended to the parent (a new root above a split root);
/// - remove: down every child whose box covers the value's box, the first match removed by
///   moving the node's LAST entry into its place; a node left under the minimum is taken out of
///   its parent the same way and its entries reinserted after the removal, highest level first,
///   at their own level; every box on the path shrinks to its entries; a root left with one
///   child is replaced by it.
pub struct DynRTree<T> {
    values: Vec<(Rect, T)>,
    alive: Vec<bool>,
    nodes: Vec<Node>,
    root: Option<usize>,
    /// The level of the leaves (the root is level 0).
    leafs_level: usize,
    limits: Limits,
}

/// An entry being inserted: a value, or a subtree (reinserted after a removal).
#[derive(Clone, Copy)]
enum Elem {
    Value(usize),
    Child(Rect, usize),
}

fn area(r: &Rect) -> i64 {
    i64::from(r.xh - r.xl) * i64::from(r.yh - r.yl)
}

fn covered(a: &Rect, b: &Rect) -> bool {
    b.xl <= a.xl && a.xh <= b.xh && b.yl <= a.yl && a.yh <= b.yh
}

impl<T> DynRTree<T> {
    /// Bulk-loaded as [`PackedRTree::new`].
    pub fn new(values: Vec<(Rect, T)>) -> DynRTree<T> {
        DynRTree::with_limits(values, Limits::DEFAULT)
    }

    /// Bulk-loaded as [`PackedRTree::with_limits`]. ⚠️ Insertion is the QUADRATIC tree's at any
    /// limits; an R*-tree's removals and queries are reproduced, its insertions are not.
    pub fn with_limits(values: Vec<(Rect, T)>, limits: Limits) -> DynRTree<T> {
        let p = PackedRTree::with_limits(values, limits);
        let mut depth = 0;
        if let Some(mut n) = p.root {
            while let Node::Internal(ch) = &p.nodes[n] {
                n = ch[0].1;
                depth += 1;
            }
        }
        let alive = vec![true; p.values.len()];
        DynRTree { values: p.values, alive, nodes: p.nodes, root: p.root, leafs_level: depth, limits }
    }

    pub fn value(&self, id: usize) -> &(Rect, T) {
        &self.values[id]
    }

    /// Insert a value; its id.
    pub fn insert(&mut self, r: Rect, v: T) -> usize {
        self.values.push((r, v));
        self.alive.push(true);
        let id = self.values.len() - 1;
        if self.root.is_none() {
            self.nodes.push(Node::Leaf(Vec::new()));
            self.root = Some(self.nodes.len() - 1);
            self.leafs_level = 0;
        }
        self.insert_elem(Elem::Value(id), 0);
        id
    }

    fn elem_box(&self, e: &Elem) -> Rect {
        match *e {
            Elem::Value(i) => self.values[i].0,
            Elem::Child(b, _) => b,
        }
    }

    fn insert_elem(&mut self, e: Elem, relative_level: usize) {
        let level = self.leafs_level - relative_level;
        let bounds = self.elem_box(&e);
        let root = self.root.expect("a root");
        let mut path: Vec<(usize, usize)> = Vec::new();
        self.insert_visit(root, 0, level, e, &bounds, &mut path);
    }

    fn insert_visit(&mut self, node: usize, current_level: usize, level: usize, e: Elem, bounds: &Rect, path: &mut Vec<(usize, usize)>) {
        let internal = matches!(self.nodes[node], Node::Internal(_));
        if internal {
            let descend = match e {
                Elem::Value(_) => true,
                Elem::Child(..) => current_level < level,
            };
            if descend {
                let idx = {
                    let Node::Internal(ch) = &self.nodes[node] else { unreachable!() };
                    let (mut best, mut best_diff, mut best_area) = (0usize, i64::MAX, i64::MAX);
                    for (i, (b, _)) in ch.iter().enumerate() {
                        let exp = expand(Some(*b), bounds);
                        let (a, d) = (area(&exp), area(&exp) - area(b));
                        if d < best_diff || (d == best_diff && a < best_area) {
                            best_diff = d;
                            best_area = a;
                            best = i;
                        }
                    }
                    best
                };
                let child = {
                    let Node::Internal(ch) = &mut self.nodes[node] else { unreachable!() };
                    ch[idx].0 = expand(Some(ch[idx].0), bounds);
                    ch[idx].1
                };
                path.push((node, idx));
                self.insert_visit(child, current_level + 1, level, e, bounds, path);
                path.pop();
            } else {
                let Elem::Child(b, c) = e else { unreachable!() };
                let Node::Internal(ch) = &mut self.nodes[node] else { unreachable!() };
                ch.push((b, c));
            }
        } else {
            let Elem::Value(i) = e else { unreachable!("a subtree reaches a leaf") };
            let Node::Leaf(ids) = &mut self.nodes[node] else { unreachable!() };
            ids.push(i);
        }
        self.post_traverse(node, path);
    }

    fn node_len(&self, node: usize) -> usize {
        match &self.nodes[node] {
            Node::Internal(c) => c.len(),
            Node::Leaf(v) => v.len(),
        }
    }

    fn post_traverse(&mut self, node: usize, path: &[(usize, usize)]) {
        if self.node_len(node) <= self.limits.max {
            return;
        }
        let (b1, b2, n2) = self.split(node);
        if let Some(&(parent, idx)) = path.last() {
            let Node::Internal(ch) = &mut self.nodes[parent] else { unreachable!() };
            ch[idx].0 = b1;
            ch.push((b2, n2));
        } else {
            let root = self.root.expect("a root");
            self.nodes.push(Node::Internal(vec![(b1, root), (b2, n2)]));
            self.root = Some(self.nodes.len() - 1);
            self.leafs_level += 1;
        }
    }

    /// The quadratic split of an overfull node: its boxes after, and the new node.
    fn split(&mut self, node: usize) -> (Rect, Rect, usize) {
        let leaf = matches!(self.nodes[node], Node::Leaf(_));
        let elems: Vec<(Rect, usize)> = match &self.nodes[node] {
            Node::Internal(c) => c.clone(),
            Node::Leaf(v) => v.iter().map(|&i| (self.values[i].0, i)).collect(),
        };
        let (g1, g2, b1, b2) = quadratic_split(elems, self.limits.min);
        let n2 = if leaf { Node::Leaf(g2.iter().map(|e| e.1).collect()) } else { Node::Internal(g2) };
        self.nodes[node] = if leaf { Node::Leaf(g1.iter().map(|e| e.1).collect()) } else { Node::Internal(g1) };
        self.nodes.push(n2);
        (b1, b2, self.nodes.len() - 1)
    }

    /// Remove the value `id`; whether it was found.
    pub fn remove(&mut self, id: usize) -> bool {
        if !self.alive[id] {
            return false;
        }
        let vbox = self.values[id].0;
        self.remove_matching(&vbox, Some(id)).is_some()
    }

    /// Remove the first value (in the tree's order) whose box is `r` — a removal by value where
    /// values compare by their box; the id removed.
    pub fn remove_eq(&mut self, r: &Rect) -> Option<usize> {
        self.remove_matching(r, None)
    }

    fn remove_matching(&mut self, vbox: &Rect, id: Option<usize>) -> Option<usize> {
        let root = self.root?;
        let mut st = RemoveState { removed: false, underflow: false, underflowed: Vec::new(), id: None };
        self.remove_visit(root, 0, None, id, vbox, &mut st);
        let gone = st.id.filter(|_| st.removed)?;
        self.alive[gone] = false;
        Some(gone)
    }

    fn remove_visit(&mut self, node: usize, current_level: usize, parent: Option<(usize, usize)>, id: Option<usize>, vbox: &Rect, st: &mut RemoveState) {
        match &self.nodes[node] {
            Node::Internal(_) => {
                let mut idx = 0;
                loop {
                    let n = self.node_len(node);
                    if idx >= n {
                        break;
                    }
                    let (b, c) = {
                        let Node::Internal(ch) = &self.nodes[node] else { unreachable!() };
                        ch[idx]
                    };
                    if covered(vbox, &b) {
                        self.remove_visit(c, current_level + 1, Some((node, idx)), id, vbox, st);
                        if st.removed {
                            break;
                        }
                    }
                    idx += 1;
                }
                if !st.removed {
                    return;
                }
                if st.underflow {
                    let relative = self.leafs_level - current_level;
                    let Node::Internal(ch) = &mut self.nodes[node] else { unreachable!() };
                    st.underflowed.push((relative, ch[idx].1));
                    let last = ch.len() - 1;
                    ch.swap(idx, last);
                    ch.pop();
                    st.underflow = ch.len() < self.limits.min;
                }
                if let Some((p, pi)) = parent {
                    let b = self.node_box(node);
                    let Node::Internal(pc) = &mut self.nodes[p] else { unreachable!() };
                    pc[pi].0 = b;
                } else {
                    // The root: reinsert what underflowed, highest level first; then shorten.
                    let under = std::mem::take(&mut st.underflowed);
                    for &(relative, n) in under.iter().rev() {
                        let elems: Vec<Elem> = match &self.nodes[n] {
                            Node::Leaf(v) => v.iter().map(|&i| Elem::Value(i)).collect(),
                            Node::Internal(c) => c.iter().map(|&(b, c)| Elem::Child(b, c)).collect(),
                        };
                        for e in elems {
                            self.insert_elem(e, relative - 1);
                        }
                    }
                    let root = self.root.expect("a root");
                    if let Node::Internal(ch) = &self.nodes[root] {
                        if ch.len() <= 1 {
                            self.root = ch.first().map(|c| c.1);
                            self.leafs_level = self.leafs_level.saturating_sub(1);
                        }
                    }
                }
            }
            Node::Leaf(v) => {
                let Some(pos) = v.iter().position(|&i| id.map_or(self.values[i].0 == *vbox, |id| i == id)) else { return };
                st.id = Some(v[pos]);
                let min = self.limits.min;
                let Node::Leaf(v) = &mut self.nodes[node] else { unreachable!() };
                let last = v.len() - 1;
                v.swap(pos, last);
                v.pop();
                st.removed = true;
                st.underflow = v.len() < min;
                if let Some((p, pi)) = parent {
                    let b = self.node_box(node);
                    let Node::Internal(pc) = &mut self.nodes[p] else { unreachable!() };
                    pc[pi].0 = b;
                }
            }
        }
    }

    /// A node's box over its entries (an empty node keeps an inverted box).
    fn node_box(&self, node: usize) -> Rect {
        let b = match &self.nodes[node] {
            Node::Internal(c) => c.iter().fold(None, |b, (r, _)| Some(expand(b, r))),
            Node::Leaf(v) => v.iter().fold(None, |b, &i| Some(expand(b, &self.values[i].0))),
        };
        b.unwrap_or(Rect { xl: i32::MAX, yl: i32::MAX, xh: i32::MIN, yh: i32::MIN })
    }

    /// The `k` values nearest the point `p`, nearest first, by id — Boost.Geometry 1.89's
    /// incremental `qbegin(nearest(p, k))` (`visitors/distance_query.hpp`,
    /// `distance_query_incremental`), in its order:
    ///
    /// - distances are comparable ones (squared, as `f64`), as in [`PackedRTree::nearest_one`];
    /// - pending branches in [`BranchHeap`]; candidate values in a min-max heap ([`MinMaxHeap`]),
    ///   at most `k` held or already returned — once that many, the FARTHEST held is dropped for
    ///   a nearer one, and a branch or value no nearer than it is skipped;
    /// - each step returns the nearest held value as soon as it is no farther than the closest
    ///   pending branch (`<=`), else opens that branch; so on a tie the heaps' moves decide.
    pub fn nearest(&self, p: (i32, i32), k: usize) -> Nearest<'_, T> {
        let mut q = Nearest { tree: self, p, k, branches: BranchHeap::default(), neighbors: MinMaxHeap::default(), count: 0, done: false };
        match self.root {
            Some(root) if k > 0 => q.apply(root, self.leafs_level),
            _ => q.done = true,
        }
        q
    }

    /// Every live value whose box touches `q`, in the tree's order, with its id.
    pub fn query(&self, q: &Rect) -> Vec<(usize, &(Rect, T))> {
        let mut out = Vec::new();
        if let Some(r) = self.root {
            self.walk(r, q, &mut out);
        }
        out
    }

    fn walk<'a>(&'a self, node: usize, q: &Rect, out: &mut Vec<(usize, &'a (Rect, T))>) {
        match &self.nodes[node] {
            Node::Internal(children) => {
                for (b, c) in children {
                    if touches(b, q) {
                        self.walk(*c, q, out);
                    }
                }
            }
            Node::Leaf(ids) => {
                for &i in ids {
                    if touches(&self.values[i].0, q) {
                        out.push((i, &self.values[i]));
                    }
                }
            }
        }
    }
}

/// An incremental nearest query over a [`DynRTree`] ([`DynRTree::nearest`]): value ids.
pub struct Nearest<'a, T> {
    tree: &'a DynRTree<T>,
    p: (i32, i32),
    k: usize,
    branches: BranchHeap,
    neighbors: MinMaxHeap,
    /// Values returned so far.
    count: usize,
    done: bool,
}

impl<T> Nearest<'_, T> {
    /// Open a node: its children become pending branches, its values candidates.
    fn apply(&mut self, node: usize, reverse_level: usize) {
        match &self.tree.nodes[node] {
            Node::Internal(children) => {
                for (b, c) in children {
                    let d = comparable(self.p, b);
                    if !self.ignore(d) {
                        self.branches.push((d, reverse_level - 1, *c));
                    }
                }
            }
            Node::Leaf(ids) => {
                for &i in ids {
                    let d = comparable(self.p, &self.tree.values[i].0);
                    if !self.ignore(d) {
                        self.neighbors.push((d, i));
                        if self.count + self.neighbors.len() > self.k {
                            self.neighbors.pop_bottom();
                        }
                    }
                }
            }
        }
    }

    /// `k` values are held or returned and the farthest held is no farther than `d`.
    fn ignore(&self, d: f64) -> bool {
        self.count + self.neighbors.len() == self.k && self.neighbors.bottom().is_none_or(|b| b.0 <= d)
    }
}

impl<T> Iterator for Nearest<'_, T> {
    type Item = usize;

    fn next(&mut self) -> Option<usize> {
        if self.done {
            return None;
        }
        loop {
            let Some(&(bd, _, _)) = self.branches.top() else {
                // No branch left: the nearest held value, or the end.
                if let Some(&(_, id)) = self.neighbors.top() {
                    self.count += 1;
                    self.neighbors.pop_top();
                    return Some(id);
                }
                self.count = self.k;
                self.done = true;
                return None;
            };
            if let Some(&(nd, id)) = self.neighbors.top() {
                if nd <= bd {
                    self.count += 1;
                    self.neighbors.pop_top();
                    return Some(id);
                }
            }
            if self.ignore(bd) {
                self.branches.clear();
                continue;
            }
            let (_, level, node) = self.branches.pop();
            self.apply(node, level);
        }
    }
}

/// The candidate values of an incremental nearest query: Boost.Geometry's `priority_dequeue`
/// (`index/detail/priority_dequeue.hpp`, `minmax_heap.hpp` — Boost Software License 1.0) of
/// `(distance, id)` ordered by distance alone: a min-max heap, the nearest at the root, the
/// farthest at one of its two children. Values as far apart are ordered by the heap's moves,
/// transcribed: a push climbs by parent then by grandparent; a pop moves the back element into
/// the hole and drives it down by grandchildren (the best of four, else of what exists), swapping
/// with a parent it passes the wrong way.
#[derive(Default)]
struct MinMaxHeap {
    v: Vec<(f64, usize)>,
}

impl MinMaxHeap {
    /// Position `i` is on a min level: an even one, the root's level being 0.
    fn min_level(i: usize) -> bool {
        (i + 1).ilog2().is_multiple_of(2)
    }

    /// `a` ranks before `b` on a min level (`min`) or a max level (`!min`).
    fn call(min: bool, a: &(f64, usize), b: &(f64, usize)) -> bool {
        if min {
            a.0 < b.0
        } else {
            b.0 < a.0
        }
    }

    fn len(&self) -> usize {
        self.v.len()
    }

    fn top(&self) -> Option<&(f64, usize)> {
        self.v.first()
    }

    fn bottom_index(&self) -> usize {
        match self.v.len() {
            0 | 1 => 0,
            2 => 1,
            _ if Self::call(true, &self.v[1], &self.v[2]) => 2,
            _ => 1,
        }
    }

    fn bottom(&self) -> Option<&(f64, usize)> {
        self.v.get(self.bottom_index())
    }

    fn push(&mut self, x: (f64, usize)) {
        self.v.push(x);
        let size = self.v.len();
        if size < 2 {
            return;
        }
        let c = size - 1;
        let min = Self::min_level(c);
        let val = self.v[c];
        let p = (c - 1) >> 1;
        if Self::call(min, &self.v[p], &val) {
            self.v[c] = self.v[p];
            self.push_up(!min, p, val);
        } else {
            self.push_up(min, c, val);
        }
    }

    /// Climb by grandparents.
    fn push_up(&mut self, min: bool, mut c: usize, val: (f64, usize)) {
        while c > 2 {
            let g = (c - 3) >> 2;
            if !Self::call(min, &val, &self.v[g]) {
                break;
            }
            self.v[c] = self.v[g];
            c = g;
        }
        self.v[c] = val;
    }

    fn pop_top(&mut self) {
        self.pop_at(0);
    }

    fn pop_bottom(&mut self) {
        let b = self.bottom_index();
        self.pop_at(b);
    }

    /// Remove position `el`: the back element moved into its hole and driven down.
    fn pop_at(&mut self, el: usize) {
        let size = self.v.len();
        if size >= 2 {
            let last = size - 1;
            let val = self.v[last];
            self.v[last] = self.v[el];
            let min = Self::min_level(el);
            self.pop_down(min, el, size - 1, val);
        }
        self.v.pop();
    }

    fn pick_grandchild4(&self, min: bool, f: usize) -> usize {
        let v = &self.v;
        let m1 = if Self::call(min, &v[f], &v[f + 1]) { f } else { f + 1 };
        let m2 = if Self::call(min, &v[f + 2], &v[f + 3]) { f + 2 } else { f + 3 };
        if Self::call(min, &v[m1], &v[m2]) {
            m1
        } else {
            m2
        }
    }

    /// Drive `val` down from the hole `p` over the first `size` positions.
    fn pop_down(&mut self, min: bool, mut p: usize, size: usize, mut val: (f64, usize)) {
        if size >= 7 {
            // While p has four grandchildren.
            let last_g = (size - 3) >> 2;
            while p < last_g {
                let ll = 4 * p + 3;
                let m = self.pick_grandchild4(min, ll);
                if !Self::call(min, &self.v[m], &val) {
                    break;
                }
                self.v[p] = self.v[m];
                let par = (m - 1) >> 1;
                if Self::call(min, &self.v[par], &val) {
                    std::mem::swap(&mut self.v[par], &mut val);
                }
                p = m;
            }
        }
        if size >= 2 && p <= ((size - 2) >> 1) {
            // At least one child: the best of the children and grandchildren that exist.
            let l = 2 * p + 1;
            let mut m = l;
            if size >= 3 && p <= ((size - 3) >> 1) {
                let mut m2 = l + 1;
                if size >= 4 && p <= ((size - 4) >> 2) {
                    let ll = 2 * l + 1;
                    m = ll;
                    if size >= 5 && p <= ((size - 5) >> 2) {
                        m = if Self::call(min, &self.v[ll], &self.v[ll + 1]) { ll } else { ll + 1 };
                        if size >= 6 && p <= ((size - 6) >> 2) {
                            m2 = ll + 2;
                        }
                    }
                }
                m = if Self::call(min, &self.v[m], &self.v[m2]) { m } else { m2 };
            }
            if Self::call(min, &self.v[m], &val) {
                self.v[p] = self.v[m];
                if m >= 3 && p <= ((m - 3) >> 2) {
                    // m is a grandchild.
                    let par = (m - 1) >> 1;
                    if Self::call(min, &self.v[par], &val) {
                        std::mem::swap(&mut self.v[par], &mut val);
                    }
                }
                p = m;
            }
        }
        self.v[p] = val;
    }
}

struct RemoveState {
    removed: bool,
    underflow: bool,
    underflowed: Vec<(usize, usize)>,
    /// The value removed.
    id: Option<usize>,
}

type Groups = (Vec<(Rect, usize)>, Vec<(Rect, usize)>, Rect, Rect);

/// The quadratic redistribution of an overfull node's entries into two groups of at least `min`
/// (boxes after).
fn quadratic_split(elems: Vec<(Rect, usize)>, min: usize) -> Groups {
    let n = elems.len();
    let (mut seed1, mut seed2) = (0usize, 1usize);
    let mut greatest: i64 = 0;
    for i in 0..n - 1 {
        for j in i + 1..n {
            let e = expand(Some(elems[i].0), &elems[j].0);
            let free = area(&e) - area(&elems[i].0) - area(&elems[j].0);
            if greatest < free {
                greatest = free;
                seed1 = i;
                seed2 = j;
            }
        }
    }
    let mut copy = elems.clone();
    let mut g1 = vec![copy[seed1]];
    let mut g2 = vec![copy[seed2]];
    let mut b1 = copy[seed1].0;
    let mut b2 = copy[seed2].0;
    let take = |v: &mut Vec<(Rect, usize)>, k: usize| {
        let last = v.len() - 1;
        if k != last {
            v[k] = v[last];
        }
        v.pop();
    };
    if seed1 < seed2 {
        take(&mut copy, seed2);
        take(&mut copy, seed1);
    } else {
        take(&mut copy, seed1);
        take(&mut copy, seed2);
    }
    let (mut c1, mut c2) = (area(&b1), area(&b2));
    let mut remaining = copy.len();
    while !copy.is_empty() {
        let mut pick = copy.len() - 1;
        let group1 = if g1.len() + remaining <= min {
            true
        } else if g2.len() + remaining <= min {
            false
        } else {
            // Scanning from the back: the entry whose two increases differ most.
            let (mut greatest, mut inc1, mut inc2) = (0i64, 0i64, 0i64);
            pick = copy.len() - 1;
            for k in (0..copy.len()).rev() {
                let i1 = area(&expand(Some(b1), &copy[k].0)) - c1;
                let i2 = area(&expand(Some(b2), &copy[k].0)) - c2;
                let d = (i1 - i2).abs();
                if greatest < d {
                    greatest = d;
                    pick = k;
                    inc1 = i1;
                    inc2 = i2;
                }
            }
            inc1 < inc2 || (inc1 == inc2 && (c1 < c2 || (c1 == c2 && g1.len() <= g2.len())))
        };
        let e = copy[pick];
        if group1 {
            g1.push(e);
            b1 = expand(Some(b1), &e.0);
            c1 = area(&b1);
        } else {
            g2.push(e);
            b2 = expand(Some(b2), &e.0);
            c2 = area(&b2);
        }
        take(&mut copy, pick);
        remaining -= 1;
    }
    (g1, g2, b1, b2)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn r(xl: i32, yl: i32, xh: i32, yh: i32) -> Rect {
        Rect { xl, yl, xh, yh }
    }

    // The partition property, on every position of shuffled, duplicated inputs.
    #[test]
    fn nth_element_partitions() {
        let base: Vec<i32> = (0..40).map(|i| (i * 7919) % 23).collect();
        for nth in 0..base.len() {
            let mut v = base.clone();
            nth_element(&mut v, nth, |a, b| a < b);
            let mut s = base.clone();
            s.sort();
            assert_eq!(v[nth], s[nth]);
            assert!(v[..nth].iter().all(|&x| x <= v[nth]) && v[nth + 1..].iter().all(|&x| x >= v[nth]));
        }
    }

    // Small ranges are selection-sorted (7 or fewer), so equal keys keep a particular order: the
    // FIRST minimum is taken each time.
    #[test]
    fn small_ranges_take_the_first_minimum() {
        let mut v = vec![(2, 'a'), (1, 'b'), (2, 'c'), (1, 'd'), (0, 'e')];
        nth_element(&mut v, 2, |a, b| a.0 < b.0);
        assert_eq!(v, vec![(0, 'e'), (1, 'b'), (1, 'd'), (2, 'c'), (2, 'a')]);
    }

    // Up to 16 values: one leaf, the input order.
    #[test]
    fn one_leaf_keeps_the_input_order() {
        let vals: Vec<(Rect, usize)> = (0..10).map(|i| (r(100 - i * 10, 0, 100 - i * 10, 0), i as usize)).collect();
        let t = PackedRTree::new(vals);
        let got: Vec<usize> = t.query(&r(-1000, -1000, 1000, 1000)).iter().map(|v| v.1).collect();
        assert_eq!(got, (0..10).collect::<Vec<_>>());
    }

    // More than one leaf: split along the longer side, the lower half first.
    #[test]
    fn more_values_split_along_the_longer_side() {
        // 20 points on a line along x, given in reverse: two leaves, the low x first.
        let vals: Vec<(Rect, usize)> = (0..20).map(|i| (r((19 - i) * 10, 0, (19 - i) * 10, 5), i as usize)).collect();
        let t = PackedRTree::new(vals);
        let got: Vec<usize> = t.query(&r(-1000, -1000, 1000, 1000)).iter().map(|v| v.1).collect();
        let first_leaf: Vec<usize> = got[..16].to_vec();
        assert!(first_leaf.iter().all(|&i| i >= 4), "{got:?}");
        assert_eq!(got.len(), 20);
        // Touching counts as intersecting.
        assert_eq!(t.query(&r(190, 5, 200, 9)).len(), 1);
    }

    // A removal by value (the reference's special spacing rectangles are kept by value, and
    // equal by their rectangle) takes the FIRST equal value in tree order, and the leaf's last
    // entry moves into its place — so which of two equal copies goes decides the order after.
    #[test]
    fn remove_by_value_takes_the_first_equal_and_moves_the_last_in() {
        let a = r(0, 0, 10, 10);
        let mut t = DynRTree::new(vec![(a, 'x'), (r(20, 0, 30, 10), 'y'), (a, 'z'), (r(40, 0, 50, 10), 'w')]);
        assert_eq!(t.remove_eq(&a), Some(0));
        let got: Vec<char> = t.query(&r(-100, -100, 100, 100)).iter().map(|(_, v)| v.1).collect();
        assert_eq!(got, vec!['w', 'y', 'z']);
        assert_eq!(t.remove_eq(&a), Some(2));
        assert_eq!(t.remove_eq(&a), None);
        // By id, the named copy goes even when an equal one comes first.
        let mut t = DynRTree::new(vec![(a, 'x'), (r(20, 0, 30, 10), 'y'), (a, 'z')]);
        assert!(t.remove(2));
        let got: Vec<char> = t.query(&r(-100, -100, 100, 100)).iter().map(|(_, v)| v.1).collect();
        assert_eq!(got, vec!['x', 'y']);
    }

    // The nearest kept value is the nearest one: against a brute-force scan, on scattered boxes
    // over several tree heights, with a filter that drops every third value.
    #[test]
    fn nearest_one_finds_the_nearest_kept_value() {
        for n in [1usize, 5, 16, 17, 40, 300] {
            let vals: Vec<(Rect, usize)> = (0..n)
                .map(|i| {
                    let (x, y) = (((i * 7919) % 997) as i32, ((i * 104_729) % 991) as i32);
                    (r(x, y, x + (i % 5) as i32, y + (i % 3) as i32), i)
                })
                .collect();
            let t = PackedRTree::new(vals.clone());
            let keep = |v: &usize| v % 3 != 1;
            let d = |p: (i32, i32), b: &Rect| -> i64 {
                let ax = |v: i32, lo: i32, hi: i32| i64::from((lo - v).max(v - hi).max(0));
                ax(p.0, b.xl, b.xh).pow(2) + ax(p.1, b.yl, b.yh).pow(2)
            };
            for p in [(0, 0), (500, 500), (996, 3), (-50, 1200), (311, 77)] {
                let want = vals.iter().filter(|v| keep(&v.1)).map(|v| d(p, &v.0)).min();
                let got = t.nearest_one(p, keep);
                assert_eq!(got.map(|v| d(p, &v.0)), want, "n={n} p={p:?}");
                assert!(got.is_none_or(|v| keep(&v.1)));
            }
        }
    }

    // Equally near values: the FIRST one reached wins (a later one replaces only when strictly
    // nearer). In one leaf that is the stored order; across leaves, the nearer branch is opened
    // first, and of two equally near branches the one pushed first sits on top of the heap.
    #[test]
    fn nearest_one_ties_go_to_the_first_reached() {
        let t = PackedRTree::new(vec![(r(10, 0, 10, 0), 'a'), (r(-10, 0, -10, 0), 'b'), (r(0, 10, 0, 10), 'c')]);
        assert_eq!(t.nearest_one((0, 0), |_| true).map(|v| v.1), Some('a'));
        assert_eq!(t.nearest_one((0, 0), |c| *c != 'a').map(|v| v.1), Some('b'));
        // Two leaves along x: low x [0, 150] and high x [160, 190]; the point at 155 is 5 from
        // both leaf boxes and from the values at 150 and 160.
        let vals: Vec<(Rect, usize)> = (0..20).map(|i| (r(i as i32 * 10, 0, i as i32 * 10, 0), i)).collect();
        let t = PackedRTree::new(vals);
        assert_eq!(t.nearest_one((155, 0), |_| true).map(|v| v.1), Some(15));
        // Nearer to the high leaf's box: it is opened first, and its value at the same distance
        // as one in the other leaf wins.
        assert_eq!(t.nearest_one((156, 0), |_| true).map(|v| v.1), Some(16));
    }

    // Equally near branches on DIFFERENT levels: the lower one is opened first, so a tie is won
    // inside the subtree already entered. 257 points on a 3 x 3 grid make a three-level tree with
    // every distance tied many ways; the answers are Boost.Geometry 1.89's own, run on the same
    // points in the same loop (each point in turn, nearest of the points not yet visited).
    #[test]
    fn nearest_one_ties_prefer_the_lower_branch() {
        let mut s = 1u32;
        let pts: Vec<(i32, i32)> = (0..257)
            .map(|_| {
                s = s.wrapping_mul(1_103_515_245).wrapping_add(12_345);
                (((s >> 16) % 3) as i32 * 10, ((s >> 20) % 3) as i32 * 10)
            })
            .collect();
        let t = PackedRTree::new(pts.iter().enumerate().map(|(i, &(x, y))| (r(x, y, x, y), i)).collect());
        let mut picks = vec![];
        for (i, &p) in pts.iter().enumerate() {
            picks.push(t.nearest_one(p, |v| *v > i).map(|v| v.1));
        }
        for (i, want) in [(0, 182), (13, 182), (104, 182), (182, 190), (188, 244), (228, 244)] {
            assert_eq!(picks[i], Some(want), "query {i}");
        }
        assert_eq!(picks[256], None);
    }

    // libc++'s heap moves: equal keys come off in the order these moves leave them, which is
    // NOT insertion order. Both sequences are what `std::priority_queue` with this comparator
    // printed under libc++ (Apple clang 17), the second with a pop after every seventh push.
    #[test]
    fn branch_heap_pops_in_libcxx_order() {
        let mut h = BranchHeap::default();
        for (i, d) in [3.0, 1.0, 1.0, 2.0, 1.0, 0.5].into_iter().enumerate() {
            h.push((d, 0, i));
        }
        let mut got = vec![];
        while h.top().is_some() {
            got.push(h.pop().2);
        }
        assert_eq!(got, [5, 4, 2, 1, 3, 0]);
        let (mut h, mut got, mut s) = (BranchHeap::default(), vec![], 1u32);
        for i in 0..40usize {
            s = s.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            h.push((f64::from((s >> 16) % 4), ((s >> 8) % 3) as usize, i));
            if i % 7 == 6 {
                got.push(h.pop().2);
            }
        }
        while h.top().is_some() {
            got.push(h.pop().2);
        }
        let want = [
            2, 8, 12, 13, 29, 39, 30, 18, 38, 10, 27, 31, 37, 16, 14, 19, 21, 9, 6, 36, 25, 28, 20, 32, 33, 1, 0, 5,
            11, 26, 17, 23, 4, 34, 3, 24, 22, 7, 35, 15,
        ];
        assert_eq!(got, want);
    }

    #[test]
    fn median_counts_are_whole_subtrees() {
        let c = Counts { maxc: 16, minc: 4 };
        assert_eq!(median_count(20, c), 16);
        assert_eq!(median_count(34, c), 16);
        assert_eq!(median_count(33, c), 16);
        assert_eq!(median_count(64, c), 32);
        assert_eq!(median_count(18, c), 14);
    }

    // An R*-tree's minimum is 30% of its maximum, truncated, never below 1: 4 → 1, 16 → 4.
    #[test]
    fn rstar_limits_keep_thirty_percent() {
        assert_eq!(Limits::rstar(4), Limits { max: 4, min: 1 });
        assert_eq!(Limits::rstar(2), Limits { max: 2, min: 1 });
        assert_eq!(Limits::rstar(16), Limits::DEFAULT);
    }

    /// The candidate heap driven as the reference's `priority_dequeue<…, pair_first_greater>` was
    /// for the goldens: `pushes` pushes of pseudo-random distances `0..keys`, the bottom dropped
    /// past `cap`, then drained top, bottom, top, … — the ids, `t` or `b` for the end taken.
    fn heap_trace(pushes: usize, keys: u32, cap: usize) -> String {
        let mut h = MinMaxHeap::default();
        let mut s: u32 = 12345;
        let mut got = Vec::new();
        for i in 0..pushes {
            s = s.wrapping_mul(1_103_515_245).wrapping_add(12345);
            h.push((f64::from((s >> 16) % keys), i));
            if h.len() > cap {
                got.push(format!("b{}", h.bottom().unwrap().1));
                h.pop_bottom();
            }
        }
        let mut top = true;
        while let Some(&(_, t)) = h.top() {
            if top {
                got.push(format!("t{t}"));
                h.pop_top();
            } else {
                got.push(format!("b{}", h.bottom().unwrap().1));
                h.pop_bottom();
            }
            top = !top;
        }
        got.join(" ")
    }

    // The candidate heap is a min-max heap on distance alone; among equal distances its moves
    // decide which value is the top and which the bottom. Heavy ties (distances 0..3), at most 9.
    #[test]
    fn min_max_heap_ties_in_the_reference_order() {
        let want = "b4 b3 b11 b8 b13 b14 b12 b7 b16 b9 b18 b20 b21 b22 b2 b24 b6 b5 b19 b28 b29 b30 b31 b27 \
                    b33 b34 b35 b36 b37 b38 b39 t0 b32 t17 b15 t23 b25 t10 b1 t26";
        assert_eq!(heap_trace(40, 4, 9), want);
    }

    // The same past 7 entries, where a pop moves down by the best of four grandchildren and
    // swaps with a parent it passes the wrong way (distances 0..5, at most 60).
    #[test]
    fn min_max_heap_deep_ties_in_the_reference_order() {
        let want = "b9 b22 b44 b41 b42 b36 b33 b28 b30 b49 b10 b71 b45 b43 b7 b38 b8 b17 b78 b15 b69 b67 b29 b27 \
                    b84 b83 b56 b55 b50 b47 b61 b46 b14 b62 b94 b95 b20 b18 b98 b99 b100 b37 b16 b4 b104 b59 \
                    b106 b92 b6 b2 b82 b111 b112 b113 b13 b114 b51 b53 b52 b48 t1 b11 t57 b79 t80 b90 t60 b0 \
                    t108 b70 t25 b93 t26 b23 t117 b73 t91 b89 t118 b40 t66 b19 t72 b97 t103 b102 t3 b77 t65 \
                    b86 t74 b107 t32 b109 t96 b87 t35 b58 t34 b110 t31 b12 t39 b119 t116 b24 t88 b105 t21 b63 \
                    t75 b64 t85 b76 t101 b115 t54 b81 t68 b5";
        assert_eq!(heap_trace(120, 6, 60), want);
    }

    // Spread distances (0..36): a pop's last step to a GRANDCHILD also swaps with the parent
    // between when the moved value ranks past it — without that swap the heap order breaks.
    #[test]
    fn min_max_heap_spread_distances_in_the_reference_order() {
        let want = "b41 b26 b13 b53 b60 b65 b35 b29 b44 b46 b70 b0 b15 b73 b34 b66 \
                    b45 b17 b78 b12 b32 b79 b58 b54 b52 b42 b86 b87 b76 b89 b80 b91 \
                    b92 b93 b88 b55 b5 b97 b98 b71 b16 b84 b28 b103 b104 b63 b106 b36 \
                    b33 b109 b19 b111 b39 b113 b114 b115 b116 b100 b11 b49 b120 b1 b122 b68 \
                    b3 b61 b82 b67 b31 b124 b102 b131 b132 b123 b134 b135 b136 b57 b138 b94 \
                    b121 b141 b142 b143 b23 b145 b146 b147 b148 b149 b77 b48 b62 b152 b154 b150 \
                    b156 b157 b158 b59 b160 b161 b162 b163 b164 b21 b166 b167 b2 b169 b127 b81 \
                    b172 b22 b64 b175 b130 b75 b178 b18 b180 b37 b14 b183 b99 b168 b186 b159 \
                    b8 b189 b190 b191 b185 b139 b194 b96 b196 b197 b198 b199 t9 b170 t112 b101 \
                    t173 b140 t24 b47 t20 b118 t179 b85 t187 b7 t155 b90 t192 b72 t30 b27 \
                    t40 b153 t38 b195 t174 b133 t151 b184 t50 b74 t144 b177 t128 b182 t43 b110 \
                    t165 b69 t181 b125 t4 b105 t176 b108 t117 b193 t83 b56 t119 b171 t25 b6 \
                    t95 b129 t51 b188 t137 b107 t126 b10";
        assert_eq!(heap_trace(200, 37, 60), want);
    }

    // Every size and both ends: 20000 random steps — push twice as often as pop the top or the
    // bottom — of distances 0..keys; FNV-1a 64 over the ids popped (a bottom's offset by 10^6).
    // The digests are the reference's on the same steps, tied (5 values) and spread (1000).
    #[test]
    fn min_max_heap_random_steps_in_the_reference_order() {
        let digest = |keys: u32| {
            let mut q = MinMaxHeap::default();
            let mut s: u32 = 777;
            let mut h: u64 = 1_469_598_103_934_665_603;
            let mut mix = |v: u64| h = (h ^ v).wrapping_mul(1_099_511_628_211);
            for i in 0..20000usize {
                s = s.wrapping_mul(1_103_515_245).wrapping_add(12345);
                let op = (s >> 16) % 4;
                if op < 2 || q.len() == 0 {
                    s = s.wrapping_mul(1_103_515_245).wrapping_add(12345);
                    q.push((f64::from((s >> 16) % keys), i));
                } else if op == 2 {
                    mix(q.top().unwrap().1 as u64);
                    q.pop_top();
                } else {
                    mix(q.bottom().unwrap().1 as u64 + 1_000_000);
                    q.pop_bottom();
                }
            }
            h
        };
        assert_eq!(digest(5), 7_424_331_541_090_690_868);
        assert_eq!(digest(1000), 6_467_840_096_304_922_571);
    }

    // The walk on an 8 x 6 grid of pitch 10 at limits 4/1 (three levels): emptied leaves and
    // then emptied internal nodes leave the tree as the walk goes — the reference's order.
    #[test]
    fn the_walk_over_a_deeper_tree_in_the_reference_order() {
        let pts: Vec<(i32, i32)> = (0..48).map(|i| (i % 8 * 10, i / 8 * 10)).collect();
        let values: Vec<(Rect, usize)> = pts.iter().enumerate().map(|(i, &(x, y))| (r(x, y, x, y), i)).collect();
        let mut t = DynRTree::with_limits(values, Limits::rstar(4));
        let mut order = vec![0usize];
        while order.len() < pts.len() {
            let cursor = *order.last().unwrap();
            let next = t.nearest(pts[cursor], 10).find(|&i| i != cursor).expect("another cell");
            assert!(t.remove(cursor));
            order.push(next);
        }
        let want = [
            0, 8, 16, 17, 9, 1, 2, 10, 18, 19, 20, 12, 4, 3, 11, 13, 21, 22, 14, 6, 7, 15, 23, 31, 30, 38, 39, 47,
            46, 45, 44, 43, 35, 36, 28, 27, 26, 25, 24, 32, 40, 41, 42, 34, 33, 37, 29, 5,
        ];
        assert_eq!(order, want);
    }

    // The placed-cell walk at R*-tree limits 4/1 on a 4 x 3 grid of pitch 10 with two extra
    // cells on (10, 10): from cell 0, each step lists the 4 nearest, takes the first that is not
    // the current cell, then removes the current one. Every list is full of distance ties; the
    // reference's lists, in order — packing, removal (a node underflows only when EMPTY at
    // minimum 1) and the incremental query's tie order all show in them.
    #[test]
    fn nearest_lists_at_rstar_limits_in_the_reference_order() {
        let pts = [(0, 0), (10, 0), (20, 0), (30, 0), (0, 10), (10, 10), (20, 10), (30, 10), (0, 20), (10, 20), (20, 20), (30, 20), (10, 10), (10, 10)];
        let values: Vec<(Rect, usize)> = pts.iter().enumerate().map(|(i, &(x, y))| (r(x, y, x, y), i)).collect();
        let mut t = DynRTree::with_limits(values, Limits::rstar(4));
        let (mut cursor, mut out) = (0usize, String::new());
        for _ in 1..pts.len() {
            let ids: Vec<usize> = t.nearest(pts[cursor], 4).collect();
            out += &ids.iter().map(|i| format!("{i} ")).collect::<String>();
            out += "; ";
            let next = *ids.iter().find(|&&i| i != cursor).expect("another cell");
            assert!(t.remove(cursor));
            cursor = next;
        }
        let want = "0 1 4 13 ; 1 13 5 12 ; 13 5 12 4 ; 5 12 9 4 ; 12 9 4 6 ; 9 8 10 4 ; 8 4 10 6 ; \
                    4 6 2 10 ; 6 7 2 10 ; 7 3 11 2 ; 3 2 11 10 ; 2 10 11 ; 10 11 ; ";
        assert_eq!(out, want);
    }
}
