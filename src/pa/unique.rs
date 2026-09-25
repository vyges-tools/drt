// SPDX-License-Identifier: Apache-2.0
//! Unique classes: instances that must have the same pin access, grouped so it is computed once,
//! on a representative.
//!
//! Stages, in order ([`compute_unique`]): the preferred-direction track patterns
//! ([`compute_pref_track_patterns`]); per master, the layer range its signal pins reach
//! ([`master_pin_layer_range`]); per instance, in database order, its class key
//! ([`compute_unique_class_key`]) — a new key opens a class; per class, the terminals it routes
//! ([`init_skip_inst_term`]).
//!
//! Rules:
//! - a track pattern is preferred when its tracks run in its layer's direction;
//! - a master's range runs from its lowest signal-pin layer to two above its highest, capped at the
//!   layer COUNT (one past the top layer); a master without signal pins has an empty range;
//! - the key is the master, the orientation, one offset per preferred pattern — the instance's
//!   placement location modulo the track spacing (x for vertical tracks, y for horizontal) when
//!   the pattern's layer is in the master's range and its tracks span the instance's box, else the
//!   spacing itself — and, apart, an instance on a non-default-rule net without auto-taper;
//! - classes are in order of their first instance; a class's instances are in database order and
//!   the FIRST is its representative;
//! - a class routes a terminal when ANY of its instances does; an instance routes a terminal whose
//!   net is a regular (not special) net, unless the terminal is a supply or the net is connected by
//!   abutment.

use std::collections::HashMap;

use crate::polygon90::Rect;
use crate::tech::{Dir, Master, Tech, TrackPattern};

/// An instance as the grouping reads it.
#[derive(Debug, Clone)]
pub struct UniqueInst {
    pub master: String,
    pub orient: String,
    /// Placement location (the placed box's lower-left).
    pub location: (i32, i32),
    /// The placed box.
    pub bbox: Rect,
    /// Per master terminal: whether THIS instance routes it.
    pub routes: Vec<bool>,
    /// On a non-default-rule net without auto-taper: a class of its own.
    pub ndr_no_taper: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct ClassKey {
    pub master: String,
    pub orient: String,
    pub offsets: Vec<i32>,
    /// The instance, when it is in a class of its own.
    pub ndr_inst: Option<usize>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UniqueClass {
    pub key: ClassKey,
    /// Instance indices, in database order; the first is the representative.
    pub insts: Vec<usize>,
    /// Per master terminal: whether the class routes it.
    pub routes: Vec<bool>,
}

/// Whether an instance routes a terminal.
pub fn routes_term(sig: &str, net: Option<&str>, net_is_special: bool, connected_by_abutment: bool) -> bool {
    if sig == "POWER" || sig == "GROUND" {
        return false;
    }
    if net.is_some() && connected_by_abutment {
        return false;
    }
    net.is_some() && !net_is_special
}

/// The patterns whose tracks run in their layer's direction, in order.
pub fn compute_pref_track_patterns(tech: &Tech, tracks: &[TrackPattern]) -> Vec<TrackPattern> {
    tracks
        .iter()
        .filter(|tp| match tech.layers[tp.layer].dir {
            Dir::Horizontal => !tp.vertical_tracks,
            _ => tp.vertical_tracks,
        })
        .copied()
        .collect()
}

/// The layers a master's signal pins reach: `(lowest, highest + 2)`, the top capped at the layer
/// count. Without signal pins the range is empty (`i32::MAX` up to `i32::MIN + 2`).
pub fn master_pin_layer_range(tech: &Tech, master: &Master) -> (i32, i32) {
    let bottom = tech.bottom_layer_num() as i32;
    let (mut lo, mut hi) = (i32::MAX, i32::MIN);
    for term in &master.terms {
        if term.sig == "POWER" || term.sig == "GROUND" {
            continue;
        }
        for pin in &term.pins {
            for &(layer, _) in &pin.shapes {
                lo = lo.min(bottom.max(layer as i32));
                hi = hi.max(layer as i32);
            }
        }
    }
    (lo, hi.saturating_add(2).min(tech.layers.len() as i32))
}

/// Whether a pattern's tracks span a box (inclusive) in the direction they step.
pub fn has_track_pattern(tp: &TrackPattern, b: Rect) -> bool {
    let low = tp.start;
    let high = low + tp.spacing * (tp.num - 1);
    if tp.vertical_tracks {
        low <= b.xh && high >= b.xl
    } else {
        low <= b.yh && high >= b.yl
    }
}

pub fn compute_unique_class_key(pref: &[TrackPattern], range: (i32, i32), inst: &UniqueInst, idx: usize) -> ClassKey {
    let offsets = pref
        .iter()
        .map(|tp| {
            let layer = tp.layer as i32;
            if layer >= range.0 && layer <= range.1 && has_track_pattern(tp, inst.bbox) {
                if tp.vertical_tracks {
                    inst.location.0 % tp.spacing
                } else {
                    inst.location.1 % tp.spacing
                }
            } else {
                tp.spacing
            }
        })
        .collect();
    ClassKey { master: inst.master.clone(), orient: inst.orient.clone(), offsets, ndr_inst: inst.ndr_no_taper.then_some(idx) }
}

/// The classes of `insts` (database order).
pub fn compute_unique(tech: &Tech, tracks: &[TrackPattern], masters: &HashMap<String, Master>, insts: &[UniqueInst]) -> Vec<UniqueClass> {
    let pref = compute_pref_track_patterns(tech, tracks);
    let ranges: HashMap<&str, (i32, i32)> = masters.iter().map(|(n, m)| (n.as_str(), master_pin_layer_range(tech, m))).collect();
    let mut classes: Vec<UniqueClass> = Vec::new();
    let mut by_key: HashMap<ClassKey, usize> = HashMap::new();
    for (i, inst) in insts.iter().enumerate() {
        let key = compute_unique_class_key(&pref, ranges[inst.master.as_str()], inst, i);
        let c = *by_key.entry(key.clone()).or_insert_with(|| {
            classes.push(UniqueClass { key, insts: Vec::new(), routes: Vec::new() });
            classes.len() - 1
        });
        classes[c].insts.push(i);
    }
    for class in &mut classes {
        init_skip_inst_term(class, insts);
    }
    classes
}

/// A class routes a terminal when any of its instances does.
pub fn init_skip_inst_term(class: &mut UniqueClass, insts: &[UniqueInst]) {
    let n = insts[class.insts[0]].routes.len();
    class.routes = (0..n).map(|t| class.insts.iter().any(|&i| insts[i].routes[t])).collect();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tech::{Layer, LayerKind, MasterPin, MasterTerm};

    fn tech() -> Tech {
        let l = |dir| Layer { kind: LayerKind::Routing, dir, ..Layer::default() };
        Tech { layers: vec![Layer::default(), Layer::default(), l(Dir::Vertical), Layer { kind: LayerKind::Cut, ..Layer::default() }, l(Dir::Horizontal)], manufacturing_grid: 5, via_defs: vec![] }
    }

    fn master(pins: &[(&str, usize)]) -> Master {
        let terms = pins.iter().map(|&(sig, layer)| MasterTerm { name: "t".into(), sig: sig.into(), pins: vec![MasterPin { shapes: vec![(layer, Rect::new(0, 0, 10, 10))] }] }).collect();
        Master { terms, blockages: vec![] }
    }

    /// The range ignores supply pins and ends two layers above the highest signal pin, capped at
    /// the layer COUNT (5 here, one past the top layer 4); no signal pins, an empty range.
    #[test]
    fn pin_layer_range_caps_at_the_layer_count() {
        let t = tech();
        assert_eq!(master_pin_layer_range(&t, &master(&[("SIGNAL", 2), ("POWER", 4)])), (2, 4));
        assert_eq!(master_pin_layer_range(&t, &master(&[("SIGNAL", 4)])), (4, 5));
        let (lo, hi) = master_pin_layer_range(&t, &master(&[("GROUND", 2)]));
        assert!(lo > hi);
    }

    /// Only tracks along their layer's direction are preferred.
    #[test]
    fn preferred_patterns_run_along_their_layer() {
        let tp = |layer, vertical_tracks| TrackPattern { layer, vertical_tracks, start: 0, num: 10, spacing: 100 };
        let got = compute_pref_track_patterns(&tech(), &[tp(2, true), tp(2, false), tp(4, true), tp(4, false)]);
        assert_eq!(got, vec![tp(2, true), tp(4, false)]);
    }

    /// Outside the master's range, or where the tracks do not reach the box, the offset is the
    /// spacing itself; inside, the location modulo the spacing.
    #[test]
    fn offsets_only_where_tracks_reach() {
        let tp = |layer, vertical_tracks, start| TrackPattern { layer, vertical_tracks, start, num: 3, spacing: 100 };
        let inst = UniqueInst { master: "m".into(), orient: "R0".into(), location: (1030, 470), bbox: Rect::new(1030, 470, 1200, 800), routes: vec![], ndr_no_taper: false };
        let pref = [tp(2, true, 1000), tp(4, false, 0), tp(2, true, 0)];
        assert_eq!(compute_unique_class_key(&pref, (2, 4), &inst, 0).offsets, vec![30, 100, 100]);
        assert_eq!(compute_unique_class_key(&pref, (4, 5), &inst, 0).offsets, vec![100, 100, 100]);
    }

    /// Tracks reaching the box's edge exactly still reach it (inclusive): the last track at the
    /// box's left x, the first at its right.
    #[test]
    fn tracks_touching_the_box_edge_reach_it() {
        let b = Rect::new(1000, 0, 1200, 500);
        let tp = |start| TrackPattern { layer: 2, vertical_tracks: true, start, num: 3, spacing: 100 };
        assert!(has_track_pattern(&tp(800), b));
        assert!(has_track_pattern(&tp(1200), b));
        assert!(!has_track_pattern(&tp(1201), b));
    }

    /// A class routes a terminal when ANY member does — here only the second instance connects it.
    #[test]
    fn a_class_routes_what_any_member_routes() {
        let t = tech();
        let mut masters = HashMap::new();
        masters.insert("m".to_string(), master(&[("SIGNAL", 2)]));
        let inst = |routes: bool, x| UniqueInst { master: "m".into(), orient: "R0".into(), location: (x, 0), bbox: Rect::new(x, 0, x + 100, 100), routes: vec![routes], ndr_no_taper: false };
        let classes = compute_unique(&t, &[], &masters, &[inst(false, 0), inst(true, 500)]);
        assert_eq!(classes.len(), 1);
        assert_eq!((classes[0].insts.clone(), classes[0].routes.clone()), (vec![0, 1], vec![true]));
    }

    /// A terminal is routed on a regular net only: not a supply, not special, not by abutment.
    #[test]
    fn routed_terminals() {
        assert!(routes_term("SIGNAL", Some("n"), false, false));
        assert!(!routes_term("SIGNAL", None, false, false));
        assert!(!routes_term("SIGNAL", Some("n"), true, false));
        assert!(!routes_term("SIGNAL", Some("n"), false, true));
        assert!(!routes_term("POWER", Some("n"), false, false));
    }
}
