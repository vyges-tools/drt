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
//! - nodes hold at most 16 entries (at least 4);
//! - building: every value's box centre (integer halves, truncated) is an entry; the entries are
//!   split recursively — at a median count of whole subtrees, along the LONGER side of the
//!   current bounding box (x on a tie), by a selection that leaves the entries in one particular
//!   permutation ([`nth_element`]), the box halved at its centre for each side — until a slice
//!   fits one leaf, whose values keep the slice's order;
//! - querying: depth first, children and values in stored order, boxes intersecting when they
//!   touch.

use crate::polygon90::Rect;

const MAX: usize = 16;
const MIN: usize = 4;

enum Node {
    Internal(Vec<(Rect, usize)>),
    Leaf(Vec<usize>),
}

pub struct PackedRTree<T> {
    values: Vec<(Rect, T)>,
    nodes: Vec<Node>,
    root: Option<usize>,
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
        let mut t = PackedRTree { values, nodes: Vec::new(), root: None };
        let n = t.values.len();
        if n == 0 {
            return t;
        }
        // Each centre: half the SUM of the sides, truncated.
        let mut entries: Vec<((i32, i32), usize)> = t.values.iter().enumerate().map(|(i, (r, _))| (((r.xl + r.xh) / 2, (r.yl + r.yh) / 2), i)).collect();
        let hint = t.values.iter().fold(None, |b, (r, _)| Some(expand(b, r))).expect("a value");
        let mut counts = Counts { maxc: 1, minc: 1 };
        let mut smax = MAX;
        while smax < n {
            counts.maxc = smax;
            smax *= MAX;
        }
        counts.minc = MIN * (counts.maxc / MAX);
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
        let next = Counts { maxc: counts.maxc / MAX, minc: counts.minc / MAX };
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
///   area; the first on a tie), that child's box grown on the way down; a node over 16 entries
///   splits — the two seeds are the pair wasting the most area together (the first such pair),
///   then entries are taken from the BACK, each the one whose area increase differs most
///   between the groups (the last scanned on a tie, scanning back to front), to the group it
///   grows less (then the smaller group box, then the group with fewer entries), a group that
///   needs every remaining entry to reach 4 taking them; the node keeps the first group, a new
///   node takes the second — appended to the parent (a new root above a split root);
/// - remove: down every child whose box covers the value's box, the first match removed by
///   moving the node's LAST entry into its place; a node left under 4 entries is taken out of
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
        let p = PackedRTree::new(values);
        let mut depth = 0;
        if let Some(mut n) = p.root {
            while let Node::Internal(ch) = &p.nodes[n] {
                n = ch[0].1;
                depth += 1;
            }
        }
        let alive = vec![true; p.values.len()];
        DynRTree { values: p.values, alive, nodes: p.nodes, root: p.root, leafs_level: depth }
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
        if self.node_len(node) <= MAX {
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
        let (g1, g2, b1, b2) = quadratic_split(elems);
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
        let Some(root) = self.root else { return false };
        let vbox = self.values[id].0;
        let mut st = RemoveState { removed: false, underflow: false, underflowed: Vec::new() };
        self.remove_visit(root, 0, None, id, &vbox, &mut st);
        if st.removed {
            self.alive[id] = false;
        }
        st.removed
    }

    fn remove_visit(&mut self, node: usize, current_level: usize, parent: Option<(usize, usize)>, id: usize, vbox: &Rect, st: &mut RemoveState) {
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
                    st.underflow = ch.len() < MIN;
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
                let Some(pos) = v.iter().position(|&i| i == id) else { return };
                let Node::Leaf(v) = &mut self.nodes[node] else { unreachable!() };
                let last = v.len() - 1;
                v.swap(pos, last);
                v.pop();
                st.removed = true;
                st.underflow = v.len() < MIN;
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

struct RemoveState {
    removed: bool,
    underflow: bool,
    underflowed: Vec<(usize, usize)>,
}

type Groups = (Vec<(Rect, usize)>, Vec<(Rect, usize)>, Rect, Rect);

/// The quadratic redistribution of 17 entries into two groups (boxes after).
fn quadratic_split(elems: Vec<(Rect, usize)>) -> Groups {
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
        let group1 = if g1.len() + remaining <= MIN {
            true
        } else if g2.len() + remaining <= MIN {
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

    #[test]
    fn median_counts_are_whole_subtrees() {
        let c = Counts { maxc: 16, minc: 4 };
        assert_eq!(median_count(20, c), 16);
        assert_eq!(median_count(34, c), 16);
        assert_eq!(median_count(33, c), 16);
        assert_eq!(median_count(64, c), 32);
        assert_eq!(median_count(18, c), 14);
    }
}
