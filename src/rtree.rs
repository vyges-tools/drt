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
