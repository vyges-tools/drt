// SPDX-License-Identifier: Apache-2.0
//! Access-point candidates: for one pin and one cost round, every point a route could reach it
//! at, before any design rule is checked.
//!
//! Stages, in order: [`merge_pin_shapes`] (the pin's shapes per layer, after the instance
//! transform), then per round [`round_candidates`]: per layer (ascending) its MAXIMAL rectangles,
//! per rectangle the x and y coordinates of each cost class ([`coords_from_rect`]), then every
//! (x, y) whose two costs are exactly the round's ([`create_multiple`]).
//!
//! Rules:
//! - cost classes, cheapest first: on-track, half-track, centre, enclosed-boundary, nearby-track;
//!   a round asks for one class on the pin's layer (`lower`) and one on the layer two above
//!   (`upper`), and a class is generated only when the round's class is at least it;
//! - a coordinate keeps the FIRST class that produced it, except that centre and enclosed-boundary
//!   lower an existing class to themselves (the smaller wins);
//! - a candidate outside its rectangle is dropped (unless a nearby-track round), as is a point
//!   and layer already made by an earlier rectangle or round — the first one made stays.

use std::collections::{BTreeMap, BTreeSet};

use crate::polygon90::{Polygon90Set, Rect};
use crate::tech::{Dir, LayerKind, Tech, TrackPattern, ViaDef};

/// A cost class. The order is the cost: cheaper first.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ApType {
    OnGrid = 0,
    HalfGrid = 1,
    Center = 2,
    EncOpt = 3,
    NearbyGrid = 4,
}

impl ApType {
    pub fn from_int(v: i32) -> Option<ApType> {
        Some(match v {
            0 => ApType::OnGrid,
            1 => ApType::HalfGrid,
            2 => ApType::Center,
            3 => ApType::EncOpt,
            4 => ApType::NearbyGrid,
            _ => return None,
        })
    }
}

/// What kind of terminal the pin belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TermKind {
    /// A standard cell's terminal.
    StdCell,
    /// A block's terminal.
    Macro,
    /// A top-level port — treated as a macro terminal.
    Io,
}

/// One candidate: its point, layer and the round's two cost classes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Candidate {
    pub x: i32,
    pub y: i32,
    pub layer: usize,
    pub lower: ApType,
    pub upper: ApType,
}

/// What the rounds read that does not depend on the pin.
pub struct Context<'a> {
    pub tech: &'a Tech,
    /// Per layer, every track coordinate (the tracks along the layer's direction) and the
    /// half-track between each two, by coordinate.
    pub track_coords: Vec<BTreeMap<i32, ApType>>,
    /// Per cut layer, its single-cut via definitions in priority order.
    pub via_priority: BTreeMap<usize, Vec<usize>>,
}

impl<'a> Context<'a> {
    pub fn new(tech: &'a Tech, tracks: &[TrackPattern]) -> Context<'a> {
        Context { tech, track_coords: track_coords(tech, tracks), via_priority: via_priority(tech) }
    }
}

/// The track coordinates: a pattern counts on a layer when its tracks run along the layer's
/// direction; then between each two consecutive tracks the half-track, rounded DOWN to the
/// manufacturing grid, unless it lands on either track.
pub fn track_coords(tech: &Tech, tracks: &[TrackPattern]) -> Vec<BTreeMap<i32, ApType>> {
    let mg = tech.manufacturing_grid;
    let mut coords: Vec<BTreeMap<i32, ApType>> = vec![BTreeMap::new(); tech.layers.len()];
    for tp in tracks {
        let is_vert_layer = tech.layers[tp.layer].dir == Dir::Vertical;
        if is_vert_layer == tp.vertical_tracks {
            let mut c = tp.start;
            for _ in 0..tp.num {
                coords[tp.layer].insert(c, ApType::OnGrid);
                c += tp.spacing;
            }
        }
    }
    for layer in coords.iter_mut() {
        let mut halves = Vec::new();
        let mut prev = i32::MAX;
        for &cur in layer.keys() {
            if cur > prev {
                let half = (i64::from(cur) + i64::from(prev)) / 2 / i64::from(mg) * i64::from(mg);
                let half = half as i32;
                if half != cur && half != prev {
                    halves.push(half);
                }
            }
            prev = cur;
        }
        for h in halves {
            layer.insert(h, ApType::HalfGrid);
        }
    }
    coords
}

/// The priority tuple, compared ascending.
type ViaPriority = (bool, i32, i32, bool, i64, i64, bool);

/// Per cut layer, the single-cut vias ordered by the priority tuple (not default; narrower
/// shapes; aligned; smaller areas). ⛔ Two vias with the same tuple keep the LATER one.
pub fn via_priority(tech: &Tech) -> BTreeMap<usize, Vec<usize>> {
    let mut by_layer: BTreeMap<usize, BTreeMap<ViaPriority, usize>> = BTreeMap::new();
    for (k, v) in tech.via_defs.iter().enumerate() {
        if v.cut_figs.len() != 1 {
            continue;
        }
        by_layer.entry(v.cut).or_default().insert(priority_tuple(tech, v), k);
    }
    by_layer.into_iter().map(|(l, m)| (l, m.into_values().collect())).collect()
}

fn priority_tuple(tech: &Tech, v: &ViaDef) -> (bool, i32, i32, bool, i64, i64, bool) {
    let shape = |figs: &[Rect], layer: usize| -> (i32, bool, i64) {
        let mut ps = Polygon90Set::new();
        for &f in figs {
            ps.insert_rect(f);
        }
        let area: i64 = ps.rectangles().iter().map(|r| i64::from(r.dx()) * i64::from(r.dy())).sum();
        let e = figs.iter().skip(1).fold(figs[0], |b, f| Rect { xl: b.xl.min(f.xl), yl: b.yl.min(f.yl), xh: b.xh.max(f.xh), yh: b.yh.max(f.yh) });
        let horz = e.dx() > e.dy();
        let width = e.dx().min(e.dy());
        let dir = tech.layers[layer].dir;
        let not_align = (horz && dir == Dir::Vertical) || (!horz && dir == Dir::Horizontal);
        (width, not_align, area)
    };
    let (w1, na1, a1) = shape(&v.layer1_figs, v.layer1);
    let (w2, na2, a2) = shape(&v.layer2_figs, v.layer2);
    (!v.is_default, w1, w2, na2, a2, a1, na1)
}

/// The pin's shapes per layer (routing layers only), after the transform.
pub fn merge_pin_shapes(tech: &Tech, shapes: &[(usize, Rect)]) -> Vec<Polygon90Set> {
    let mut sets = vec![Polygon90Set::new(); tech.layers.len()];
    for &(layer, r) in shapes {
        if tech.layers[layer].kind == LayerKind::Routing {
            sets[layer].insert_rect(r);
        }
    }
    sets
}

/// One cost round over every layer the pin has shapes on; `apset` carries the points already made
/// (across rounds).
pub fn round_candidates(cx: &Context<'_>, pin: &mut [Polygon90Set], kind: TermKind, lower: ApType, upper: ApType, apset: &mut BTreeSet<((i32, i32), usize)>) -> Vec<Candidate> {
    let mut out = Vec::new();
    for (layer, set) in pin.iter_mut().enumerate() {
        if set.is_empty() || cx.tech.layers[layer].kind != LayerKind::Routing {
            continue;
        }
        let is_macro = kind != TermKind::StdCell;
        for rect in set.max_rectangles() {
            let (xs, ys) = coords_from_rect(cx, rect, layer, lower, upper, is_macro);
            create_multiple(cx, &mut out, apset, rect, layer, &xs, &ys, lower, upper);
        }
    }
    out
}

/// Every (x, y) whose classes are exactly the round's, x then y
/// ascending.
#[allow(clippy::too_many_arguments)]
fn create_multiple(cx: &Context<'_>, out: &mut Vec<Candidate>, apset: &mut BTreeSet<((i32, i32), usize)>, rect: Rect, layer: usize, xs: &BTreeMap<i32, ApType>, ys: &BTreeMap<i32, ApType>, lower: ApType, upper: ApType) {
    let l = &cx.tech.layers[layer];
    for (&x, &cost_x) in xs {
        for (&y, &cost_y) in ys {
            let low_type = if l.is_horizontal() { cost_y } else { cost_x };
            let up_type = if l.is_vertical() { cost_y } else { cost_x };
            if low_type == lower && up_type == upper {
                let nearby = lower == ApType::NearbyGrid || upper == ApType::NearbyGrid;
                if !rect.contains(x, y) && !nearby {
                    continue;
                }
                if apset.contains(&((x, y), layer)) {
                    continue;
                }
                out.push(Candidate { x, y, layer, lower, upper });
                // ⛔ A nearby-track point is joined to the rectangle by a path; the point RECORDED
                // as made is that path's first bend when it has two legs, not the point itself.
                let mut key = (x, y);
                if nearby {
                    let hw = l.min_width / 2;
                    let end = (x.clamp(rect.xl + hw, rect.xh - hw), y.clamp(rect.yl + hw, rect.yh - hw));
                    let mut e = (x, y);
                    if x != end.0 {
                        e.0 = end.0;
                    } else if y != end.1 {
                        e.1 = end.1;
                    }
                    if e != (x, y) && e != end {
                        key = e;
                    }
                }
                apset.insert((key, layer));
            }
        }
    }
}

/// The rectangle's x and y coordinates of each class the round allows.
fn coords_from_rect(cx: &Context<'_>, rect: Rect, layer: usize, lower: ApType, upper: ApType, is_macro: bool) -> (BTreeMap<i32, ApType>, BTreeMap<i32, ApType>) {
    let tech = cx.tech;
    let (mut xs, mut ys) = (BTreeMap::new(), BTreeMap::new());
    let l = &tech.layers[layer];
    if rect.dx().min(rect.dy()) < l.min_width {
        return (xs, ys);
    }
    let second = if layer + 2 <= tech.top_layer_num() { layer + 2 } else { layer - 2 };
    let horz = l.is_horizontal();
    let hwidth = l.width / 2;
    let mut use_center_line = false;
    if is_macro {
        let rect_horz = rect.dx() >= rect.dy();
        if (rect_horz && horz) || (!rect_horz && !horz) {
            let w = l.width;
            if (rect_horz && rect.dy() < 2 * w) || (!rect_horz && rect.dx() < 2 * w) {
                use_center_line = true;
            }
        }
    }
    let offset = if is_macro && !use_center_line { hwidth } else { 0 };
    let (layer1_min, layer1_max) = if horz { (rect.yl, rect.yh) } else { (rect.xl, rect.xh) };
    let (layer1, layer2) = if horz { (&mut ys, &mut xs) } else { (&mut xs, &mut ys) };
    const CLASSES: [ApType; 4] = [ApType::OnGrid, ApType::Center, ApType::EncOpt, ApType::NearbyGrid];
    for cost in CLASSES {
        if upper >= cost {
            gen_costed(cx, cost, layer2, second, layer, second, rect, offset);
        }
    }
    if !use_center_line {
        for cost in CLASSES {
            if lower >= cost {
                gen_costed(cx, cost, layer1, layer, layer, layer, rect, 0);
            }
        }
    } else {
        gen_centered(tech, layer1, layer1_min, layer1_max);
        for v in layer1.values_mut() {
            *v = ApType::OnGrid;
        }
    }
    (xs, ys)
}

/// One class's coordinates across `layer_num`'s direction.
#[allow(clippy::too_many_arguments)]
fn gen_costed(cx: &Context<'_>, cost: ApType, coords: &mut BTreeMap<i32, ApType>, track_layer: usize, base_layer: usize, layer_num: usize, rect: Rect, offset: i32) {
    let l = &cx.tech.layers[layer_num];
    let horz = l.is_horizontal();
    let (rmin, rmax) = if horz { (rect.yl, rect.yh) } else { (rect.xl, rect.xh) };
    let track = &cx.track_coords[track_layer];
    match cost {
        ApType::OnGrid => gen_on_track(coords, track, rmin + offset, rmax - offset, false),
        ApType::Center => gen_centered(cx.tech, coords, rmin + offset, rmax - offset),
        ApType::EncOpt => gen_enclosed_boundary(cx, coords, rect, base_layer, horz),
        ApType::NearbyGrid => {
            gen_on_track(coords, track, rmin - l.min_width, rmin, true);
            gen_on_track(coords, track, rmax, rmax + l.min_width, true);
        }
        ApType::HalfGrid => {}
    }
}

/// The track coordinates in `[low, high]`, each with its own class (or
/// nearby-track); an existing coordinate is kept.
fn gen_on_track(coords: &mut BTreeMap<i32, ApType>, track: &BTreeMap<i32, ApType>, low: i32, high: i32, nearby: bool) {
    for (&c, &cost) in track.range(low..) {
        if c > high {
            break;
        }
        coords.entry(c).or_insert(if nearby { ApType::NearbyGrid } else { cost });
    }
}

/// Unless three on-track coordinates already lie in `[low, high]`, the middle,
/// rounded down to the manufacturing grid.
fn gen_centered(tech: &Tech, coords: &mut BTreeMap<i32, ApType>, low: i32, high: i32) {
    let on_grid = coords.range(low..).take_while(|(&c, _)| c <= high).filter(|(_, &v)| v == ApType::OnGrid).count();
    if on_grid >= 3 {
        return;
    }
    let mg = tech.manufacturing_grid;
    let c = (low + high) / 2 / mg * mg;
    let e = coords.entry(c).or_insert(ApType::Center);
    *e = (*e).min(ApType::Center);
}

/// For every via of the cut layer above `layer`, in priority order, the
/// two coordinates that put its bottom shape flush with the rectangle's edges — when it fits.
fn gen_enclosed_boundary(cx: &Context<'_>, coords: &mut BTreeMap<i32, ApType>, rect: Rect, layer: usize, is_curr_horz: bool) {
    if layer + 1 > cx.tech.top_layer_num() {
        return;
    }
    let Some(vias) = cx.via_priority.get(&(layer + 1)) else { return };
    for &v in vias {
        let b = cx.tech.via_defs[v].layer1_bbox();
        if b.dx() > rect.dx() || b.dy() > rect.dy() {
            continue;
        }
        let top = if is_curr_horz { rect.yh - b.yh } else { rect.xh - b.xh };
        let low = if is_curr_horz { rect.yl - b.yl } else { rect.xl - b.xl };
        for c in [top, low] {
            let e = coords.entry(c).or_insert(ApType::EncOpt);
            *e = (*e).min(ApType::EncOpt);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tech::{Layer, ViaDef};

    /// A small stack: li1 (vertical, width 170), mcon, met1 (horizontal, width 140), via, met2.
    fn tech(vias: Vec<ViaDef>) -> Tech {
        let l = |name: &str, kind, dir, width| Layer { name: name.into(), kind, dir, width, min_width: width, ..Layer::default() };
        Tech {
            layers: vec![
                l("FR_MASTERSLICE", LayerKind::Placeholder, Dir::None, 0),
                l("Fr_VIA", LayerKind::Placeholder, Dir::None, 0),
                l("li1", LayerKind::Routing, Dir::Vertical, 170),
                l("mcon", LayerKind::Cut, Dir::None, 170),
                l("met1", LayerKind::Routing, Dir::Horizontal, 140),
                l("via", LayerKind::Cut, Dir::None, 150),
                l("met2", LayerKind::Routing, Dir::Vertical, 140),
            ],
            manufacturing_grid: 5,
            via_defs: vias,
        }
    }

    fn via(name: &str, l1: Rect) -> ViaDef {
        ViaDef { name: name.into(), is_default: true, layer1: 2, cut: 3, layer2: 4, layer1_figs: vec![l1], cut_figs: vec![Rect::new(-85, -85, 85, 85)], layer2_figs: vec![Rect::new(-145, -85, 145, 85)] }
    }

    /// Rule: two vias with the same priority tuple — the LATER one in technology order stays.
    #[test]
    fn a_via_priority_tie_keeps_the_later_via() {
        let t = tech(vec![via("A", Rect::new(-85, -85, 85, 85)), via("B", Rect::new(-85, -85, 85, 85))]);
        assert_eq!(via_priority(&t)[&3], vec![1]);
    }

    /// Rule: an on-track coordinate already present keeps its class (the first class wins).
    #[test]
    fn an_on_track_coordinate_keeps_its_first_class() {
        let mut coords = BTreeMap::from([(100, ApType::Center)]);
        let track = BTreeMap::from([(100, ApType::OnGrid), (200, ApType::OnGrid)]);
        gen_on_track(&mut coords, &track, 0, 300, false);
        assert_eq!(coords, BTreeMap::from([(100, ApType::Center), (200, ApType::OnGrid)]));
    }

    /// Rule: an enclosed-boundary coordinate that is already on-track stays on-track (the smaller
    /// class); a new one is enclosed-boundary.
    #[test]
    fn an_enclosed_coordinate_lowers_to_the_smaller_class() {
        let t = tech(vec![via("A", Rect::new(-85, -85, 85, 85))]);
        let cx = Context::new(&t, &[]);
        // A vertical li1 rect 0..400 wide: the via (170 wide) fits flush at x = 85 and x = 315.
        let mut coords = BTreeMap::from([(85, ApType::OnGrid)]);
        gen_enclosed_boundary(&cx, &mut coords, Rect::new(0, 0, 400, 1000), 2, false);
        assert_eq!(coords, BTreeMap::from([(85, ApType::OnGrid), (315, ApType::EncOpt)]));
    }

    /// Rule: a rectangle narrower than the layer's min width yields no coordinates at all.
    #[test]
    fn a_rect_narrower_than_min_width_gives_nothing() {
        let t = tech(vec![]);
        let cx = Context::new(&t, &[]);
        let (xs, ys) = coords_from_rect(&cx, Rect::new(0, 0, 160, 1000), 2, ApType::Center, ApType::Center, false);
        assert!(xs.is_empty() && ys.is_empty());
    }

    /// Rule (nearby-track rounds): the point recorded as made is the path's first bend when the
    /// path to the rectangle has two legs — so a later candidate AT the point itself is not a
    /// duplicate, while one at the bend is.
    #[test]
    fn a_nearby_point_records_its_first_bend() {
        let t = tech(vec![]);
        let cx = Context::new(&t, &[]);
        let rect = Rect::new(0, 0, 400, 400);
        let xs = BTreeMap::from([(-50, ApType::NearbyGrid)]);
        let ys = BTreeMap::from([(-50, ApType::NearbyGrid)]);
        let mut apset = BTreeSet::new();
        let mut out = Vec::new();
        create_multiple(&cx, &mut out, &mut apset, rect, 2, &xs, &ys, ApType::NearbyGrid, ApType::NearbyGrid);
        assert_eq!(out.len(), 1);
        // The end is clamped to (85, 85); the first leg moves x only, so the bend is (85, -50).
        assert!(apset.contains(&((85, -50), 2)));
        assert!(!apset.contains(&((-50, -50), 2)));
    }
}
