// SPDX-License-Identifier: Apache-2.0
//! Ports wholly ABOVE the top routing layer. Before the ports are read, a via stack is added from
//! the top routing layer up to each such port (one via per cut layer, that layer's
//! [`default_via`], at [`best_via_position`]); the port is then read as the box where that stack
//! meets the top routing layer ([`via_box_above_max_layer`]). The database side is
//! `pa::db::stacked_vias`.

use crate::pa::candidates::priority_tuple;
use crate::polygon90::Rect;
use crate::tech::{Dir, Tech, TrackPattern};

/// Where a port above the top routing layer meets its net's wire: walking the wire's via boxes on
/// the top routing layer in order, each one that touches the box so far BECOMES the box (so the
/// last of a chain wins); none, the port's own box (and layer 0, as the reference leaves it).
pub fn via_box_above_max_layer(via_boxes: &[Rect], bbox: Rect, top: usize) -> (usize, Rect) {
    let (mut b, mut layer) = (bbox, 0usize);
    for v in via_boxes {
        if b.xl <= v.xh && v.xl <= b.xh && b.yl <= v.yh && v.yl <= b.yh {
            b = *v;
            layer = top;
        }
    }
    (layer, b)
}

/// The via position in a pin above the top routing layer: of the top routing layer's PREFERRED
/// tracks crossing the pin, the one nearest its centre (the first on a tie); none, the centre.
pub fn best_via_position(tech: &Tech, tracks: &[TrackPattern], top: usize, pin: Rect) -> (i32, i32) {
    let c = ((pin.xl + pin.xh) / 2, (pin.yl + pin.yh) / 2);
    let horizontal = tech.layers[top].dir == Dir::Horizontal;
    let (lo, hi, centre) = if horizontal { (pin.yl, pin.yh, c.1) } else { (pin.xl, pin.xh, c.0) };
    let mut best: Option<(i32, i32)> = None;
    for tp in tracks.iter().filter(|t| t.layer == top && t.vertical_tracks == !horizontal) {
        let first = ((lo - tp.start + tp.spacing - 1) / tp.spacing).max(0);
        let last = ((hi - tp.start) / tp.spacing).min(tp.num - 1);
        for i in first..=last {
            let t = tp.start + i * tp.spacing;
            let d = (t - centre).abs();
            if best.is_none_or(|(bd, _)| d < bd) {
                best = Some((d, t));
            }
        }
    }
    match best {
        None => c,
        Some((_, t)) if horizontal => (c.0, t),
        Some((_, t)) => (t, c.1),
    }
}

/// A cut layer's default via: of its single-cut vias, the least by (not default; lower width;
/// upper width; upper not aligned; cut area; upper area; lower area; lower not aligned; name);
/// with none, above the top routing layer, the least of the fewest-cut vias.
pub fn default_via(tech: &Tech, cut: usize, top: usize) -> Option<usize> {
    let key = |k: usize| {
        let v = &tech.via_defs[k];
        let (nd, w1, w2, na2, a2, a1, na1) = priority_tuple(tech, v);
        let cut_area: i64 = v.cut_figs.iter().map(|f| i64::from(f.dx()) * i64::from(f.dy())).sum();
        (nd, w1, w2, na2, cut_area, a2, a1, na1, v.name.clone())
    };
    let vias: Vec<usize> = (0..tech.via_defs.len()).filter(|&k| tech.via_defs[k].cut == cut).collect();
    let fewest = vias.iter().map(|&k| tech.via_defs[k].cut_figs.len()).min()?;
    if fewest != 1 && cut <= top {
        return None;
    }
    vias.into_iter().filter(|&k| tech.via_defs[k].cut_figs.len() == fewest).min_by_key(|&k| key(k))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tech::{Layer, LayerKind, ViaDef};

    fn r(xl: i32, yl: i32, xh: i32, yh: i32) -> Rect {
        Rect { xl, yl, xh, yh }
    }

    /// Layers: 0 routing (horizontal), 1 cut, 2 routing (vertical).
    fn tech(vias: Vec<ViaDef>) -> Tech {
        let mut t = Tech::default();
        for (kind, dir) in [(LayerKind::Routing, Dir::Horizontal), (LayerKind::Cut, Dir::None), (LayerKind::Routing, Dir::Vertical)] {
            t.layers.push(Layer { kind, dir, ..Default::default() });
        }
        t.via_defs = vias;
        t
    }

    fn via(name: &str, is_default: bool, cuts: usize, lower: Rect) -> ViaDef {
        ViaDef { name: name.into(), is_default, layer1: 0, cut: 1, layer2: 2, layer1_figs: vec![lower], cut_figs: vec![r(-5, -5, 5, 5); cuts], layer2_figs: vec![r(-5, -10, 5, 10)] }
    }

    fn tp(vertical_tracks: bool, start: i32, num: i32, spacing: i32) -> TrackPattern {
        TrackPattern { layer: 0, vertical_tracks, start, num, spacing }
    }

    // A horizontal top layer: its preferred tracks are the horizontal ones (y); the nearest to the
    // pin's centre wins, x stays the centre.
    #[test]
    fn the_via_sits_on_the_preferred_track_nearest_the_centre() {
        let t = tech(vec![]);
        let tracks = [tp(true, 0, 100, 7), tp(false, 0, 100, 10)];
        assert_eq!(best_via_position(&t, &tracks, 0, r(0, 0, 40, 36)), (20, 20));
    }

    // Equally near: the FIRST track found wins (strictly less replaces).
    #[test]
    fn a_track_tie_keeps_the_first() {
        let t = tech(vec![]);
        assert_eq!(best_via_position(&t, &[tp(false, 0, 100, 10)], 0, r(0, 0, 40, 30)), (20, 10));
    }

    // Only tracks inside the pin count — the first index rounds UP, the last is clamped to the
    // pattern; none inside, the centre.
    #[test]
    fn no_track_in_the_pin_gives_the_centre() {
        let t = tech(vec![]);
        assert_eq!(best_via_position(&t, &[tp(false, 0, 100, 10)], 0, r(0, 11, 40, 19)), (20, 15));
        assert_eq!(best_via_position(&t, &[tp(false, 0, 2, 10)], 0, r(0, 20, 40, 40)), (20, 30));
        assert_eq!(best_via_position(&t, &[tp(false, 0, 3, 10)], 0, r(0, 20, 40, 40)), (20, 20));
    }

    // Single-cut vias only; not-default loses first, then the narrower lower shape.
    #[test]
    fn the_default_via_is_the_least_single_cut() {
        let t = tech(vec![via("a", false, 1, r(-5, -5, 5, 5)), via("b", true, 1, r(-8, -8, 8, 8)), via("c", true, 1, r(-6, -6, 6, 6)), via("d", true, 2, r(-5, -5, 5, 5))]);
        assert_eq!(default_via(&t, 1, 0), Some(2));
    }

    // Everything else equal, the NAME decides (not the order read).
    #[test]
    fn a_default_via_tie_goes_to_the_name() {
        let t = tech(vec![via("z", true, 1, r(-5, -5, 5, 5)), via("m", true, 1, r(-5, -5, 5, 5))]);
        assert_eq!(default_via(&t, 1, 0), Some(1));
    }

    // The cut's area decides before the upper shape's area: "a" has the smaller cut, "b" the
    // smaller upper shape.
    #[test]
    fn cut_area_decides_before_upper_area() {
        let mut a = via("a", true, 1, r(-5, -5, 5, 5));
        a.cut_figs = vec![r(-4, -4, 4, 4)];
        a.layer2_figs = vec![r(-5, -12, 5, 12)];
        let b = via("b", true, 1, r(-5, -5, 5, 5));
        let t = tech(vec![b, a]);
        assert_eq!(default_via(&t, 1, 0), Some(1));
    }

    // No single-cut via: above the top routing layer the least of the fewest-cut vias; at or
    // below it, none.
    #[test]
    fn without_a_single_cut_via_only_above_the_top_layer() {
        let t = tech(vec![via("a", true, 3, r(-5, -5, 5, 5)), via("b", true, 2, r(-9, -9, 9, 9)), via("c", true, 2, r(-7, -7, 7, 7))]);
        assert_eq!(default_via(&t, 1, 0), Some(2));
        assert_eq!(default_via(&t, 1, 1), None);
    }

    // Each touching via box becomes the box, so the next must touch THAT box; none touching, the
    // port's own box on layer 0.
    #[test]
    fn the_last_touching_via_box_wins() {
        let port = r(0, 0, 100, 100);
        assert_eq!(via_box_above_max_layer(&[], port, 6), (0, port));
        assert_eq!(via_box_above_max_layer(&[r(200, 0, 210, 10)], port, 6), (0, port));
        assert_eq!(via_box_above_max_layer(&[r(10, 10, 20, 20), r(20, 20, 30, 30)], port, 6), (6, r(20, 20, 30, 30)));
        assert_eq!(via_box_above_max_layer(&[r(10, 10, 20, 20), r(50, 50, 60, 60)], port, 6), (6, r(10, 10, 20, 20)));
    }
}
