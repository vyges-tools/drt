// SPDX-License-Identifier: Apache-2.0
//! Access trials: the route shapes an access point would add, checked for design-rule violations
//! against the pin's own instance (or port).
//!
//! Stages, in order: the target's shapes ([`instance_shapes`] or a port's pins, each with its
//! owner); per trial the window around the point ([`window`]); the trial's shapes — a planar
//! segment ([`planar_markers`]) or a via and the segment leaving it ([`via_markers`]) — added as
//! NOT fixed, owned by the pin's net; then the checks.
//!
//! Rules:
//! - only the target's own shapes are in the window: its instance's terminals and obstructions (or
//!   the port's pins) — never a neighbour's;
//! - the window is the point widened by five pitches of the access point's layer, and a target
//!   shape is in it when it touches it;
//! - a design shape of an unconnected POWER or GROUND terminal belongs to the floating power or
//!   ground owner; any other unconnected terminal owns its shapes itself — and so do the trial's
//!   shapes of an unconnected pin, even a power or ground one;
//! - a segment runs from the access point three layer-widths along its direction (see the caller);
//!   its width is the layer's, or the layer's wrong-way width when it runs across the layer's
//!   direction; it extends half the layer's width past its far end and not at all at the point.

use crate::gc::{Marker, Owner, Worker};
use crate::polygon90::Rect;
use crate::tech::{Layer, Master, Tech, Transform, ViaDef};

/// A target's shape: its owner, layer and rectangle (design coordinates).
pub type TargetShape = (Owner, usize, Rect);

/// Who owns a terminal's DESIGN shapes: its net; unconnected, the floating power or ground owner
/// for a supply terminal, else the terminal itself.
pub fn design_owner(net: Option<&str>, sig: &str, unconnected: Owner) -> Owner {
    match (net, sig) {
        (Some(n), _) => Owner::Net(n.into()),
        (None, "POWER") => Owner::FloatingPower,
        (None, "GROUND") => Owner::FloatingGround,
        (None, _) => unconnected,
    }
}

/// An instance's shapes: every pin shape of every terminal (owner from `owner_of`, by terminal
/// index), and every blockage (owned by the instance), after the transform.
pub fn instance_shapes(master: &Master, inst: &str, xf: &Transform, owner_of: impl Fn(usize) -> Owner) -> Vec<TargetShape> {
    let mut out = Vec::new();
    for (t, term) in master.terms.iter().enumerate() {
        let owner = owner_of(t);
        for pin in &term.pins {
            for &(layer, r) in &pin.shapes {
                out.push((owner.clone(), layer, xf.apply(r)));
            }
        }
    }
    for &(layer, r) in &master.blockages {
        out.push((Owner::Inst(inst.into()), layer, xf.apply(r)));
    }
    out
}

/// The window around an access point.
pub fn window(tech: &Tech, point: (i32, i32), ap_layer: usize) -> Rect {
    let ext = 5 * tech.layers[ap_layer].pitch;
    Rect::new(point.0 - ext, point.1 - ext, point.0 + ext, point.1 + ext)
}

/// The segment from `begin` (the access point) to `end` on `layer`.
pub fn access_segment(layer: &Layer, begin: (i32, i32), end: (i32, i32)) -> Rect {
    let along_y = begin.0 == end.0;
    let wrong_way = (layer.is_horizontal() && along_y) || (layer.is_vertical() && !along_y);
    let w = if wrong_way { layer.wrong_way_width } else { layer.width };
    let ext = layer.width / 2;
    // Toward lower coordinates the segment is stored end → point, else point → end; the point's
    // end is not extended.
    let (p, q, pe, qe) = if end < begin { (end, begin, ext, 0) } else { (begin, end, 0, ext) };
    if along_y {
        Rect::new(p.0 - w / 2, p.1 - pe, q.0 + w / 2, q.1 + qe)
    } else {
        Rect::new(p.0 - pe, p.1 - w / 2, q.0 + qe, q.1 + w / 2)
    }
}

fn check(tech: &Tech, target: &[TargetShape], point: (i32, i32), ap_layer: usize, trial: &[(usize, Rect)], owner: &Owner, ignore_long_side_eol: bool) -> Vec<Marker> {
    let trial: Vec<(&Owner, usize, Rect)> = trial.iter().map(|&(l, r)| (owner, l, r)).collect();
    check_in(tech, target, window(tech, point, ap_layer), &trial, ignore_long_side_eol)
}

/// The checks over `win`: the target's shapes that touch it, fixed; the trial's shapes, each with
/// its own owner, not fixed. `ignore_long_side_eol`: see [`Worker::ignore_long_side_eol`] — via
/// and pattern trials set it, planar trials do not.
pub(crate) fn check_in(tech: &Tech, target: &[TargetShape], win: Rect, trial: &[(&Owner, usize, Rect)], ignore_long_side_eol: bool) -> Vec<Marker> {
    let mut w = Worker::new(tech);
    w.ignore_long_side_eol = ignore_long_side_eol;
    // Pin access checks without the minimum-area rule (`setIgnoreMinArea`, in all three of its
    // checkers): an access trial's stub is short by construction.
    w.ignore_min_area = true;
    for (o, layer, r) in target {
        if r.xl <= win.xh && win.xl <= r.xh && r.yl <= win.yh && win.yl <= r.yh {
            w.add(o, *layer, *r, true);
        }
    }
    for &(owner, layer, r) in trial {
        w.add(owner, layer, r, false);
    }
    w.init();
    w.run().to_vec()
}

/// A via's shapes on its three layers, at `at`.
pub fn via_shapes(via: &ViaDef, at: (i32, i32)) -> Vec<(usize, Rect)> {
    let sh = |r: &Rect| Rect::new(r.xl + at.0, r.yl + at.1, r.xh + at.0, r.yh + at.1);
    let mut out: Vec<(usize, Rect)> = Vec::new();
    out.extend(via.layer1_figs.iter().map(|r| (via.layer1, sh(r))));
    out.extend(via.cut_figs.iter().map(|r| (via.cut, sh(r))));
    out.extend(via.layer2_figs.iter().map(|r| (via.layer2, sh(r))));
    out
}

/// A planar trial: the segment from the point to `end` on the access point's layer.
pub fn planar_markers(tech: &Tech, target: &[TargetShape], owner: &Owner, point: (i32, i32), layer: usize, end: (i32, i32)) -> Vec<Marker> {
    let seg = access_segment(&tech.layers[layer], point, end);
    check(tech, target, point, layer, &[(layer, seg)], owner, false)
}

/// A via trial: the via at the point (its three layers' shapes), and the segment from the point
/// to `end` on the via's OTHER metal layer.
pub fn via_markers(tech: &Tech, target: &[TargetShape], owner: &Owner, point: (i32, i32), layer: usize, via: &ViaDef, end: (i32, i32)) -> Vec<Marker> {
    let mut trial = via_shapes(via, point);
    let other = if via.layer1 == layer { via.layer2 } else { via.layer1 };
    trial.push((other, access_segment(&tech.layers[other], point, end)));
    check(tech, target, point, layer, &trial, owner, true)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tech::{Dir, LayerKind};

    fn layer(dir: Dir) -> Layer {
        Layer { kind: LayerKind::Routing, dir, width: 170, wrong_way_width: 200, ..Layer::default() }
    }

    /// Toward higher coordinates the far end extends half a width and the point's end not at all;
    /// toward lower coordinates the same, mirrored.
    #[test]
    fn segment_extends_only_its_far_end() {
        let l = layer(Dir::Horizontal);
        assert_eq!(access_segment(&l, (1000, 500), (1510, 500)), Rect::new(1000, 415, 1595, 585));
        assert_eq!(access_segment(&l, (1000, 500), (490, 500)), Rect::new(405, 415, 1000, 585));
    }

    /// Across the layer's direction the segment takes the wrong-way width; the extension stays
    /// half the layer's width.
    #[test]
    fn segment_across_the_layer_is_wrong_way_wide() {
        let l = layer(Dir::Horizontal);
        assert_eq!(access_segment(&l, (1000, 500), (1000, 1010)), Rect::new(900, 500, 1100, 1095));
        let v = layer(Dir::Vertical);
        assert_eq!(access_segment(&v, (1000, 500), (1000, 1010)), Rect::new(915, 500, 1085, 1095));
    }

    /// A target shape that only TOUCHES the window (five pitches) is in it, and merges with the
    /// shapes it touches: here it makes its neighbour's maximal rectangle 3200 wide, so the trial
    /// 200 away needs 280 — a violation that neither a four-pitch window nor a strict one sees.
    #[test]
    fn a_shape_touching_the_window_is_in_it() {
        let t = crate::gc::tests::tech();
        let b = Owner::Net("b".into());
        let target = vec![(b.clone(), 4, Rect::new(-1600, 10, 1600, 1590)), (b, 4, Rect::new(-1600, 1590, 1600, 4800))];
        let m = planar_markers(&t, &target, &Owner::Net("a".into()), (0, -260), 4, (420, -260));
        assert!(!m.is_empty());
    }

    /// An unconnected supply terminal's design shapes are the floating owner's; a signal's its own.
    #[test]
    fn unconnected_supply_is_floating() {
        let t = Owner::InstTerm("u".into(), "A".into());
        assert_eq!(design_owner(None, "POWER", t.clone()), Owner::FloatingPower);
        assert_eq!(design_owner(None, "GROUND", t.clone()), Owner::FloatingGround);
        assert_eq!(design_owner(None, "SIGNAL", t.clone()), t);
        assert_eq!(design_owner(Some("n1"), "POWER", t), Owner::Net("n1".into()));
    }
}
