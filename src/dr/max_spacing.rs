// SPDX-License-Identifier: Apache-2.0
//! LEF58_MAXSPACING: the router's repair pass (`FlexDR::fixMaxSpacing`), run after an iteration
//! ends with no marker standing, stage by stage in the reference's order and named after its
//! functions.
//!
//! A rule `MAXSPACING max CUTCLASS c` asks that every via of class `c` have another cut within
//! `max` of it. The reference finds the vias that have none (`getLonelyVias`), marks them, and
//! re-routes a region around each with a via-swap worker that tries the layer's SECONDARY via
//! defs (`initSecondaryVias`: the vias of another class). Ported so far: the sequencer, the
//! secondary via defs, the lonely-via search, and the pin-access avoid list. ⛔ Not ported: the
//! stages after the lonely-via search (markers, stubborn terminals and the pin-access re-run, the
//! region workers' via swap, minimum-step patching). A run that reaches them — a lonely via found
//! — is REFUSED there, before anything is written; a run that finds no lonely via routes as the
//! reference does (the pass changes nothing).

use std::collections::{BTreeMap, BTreeSet};

use crate::dr::cost::DrFig;
use crate::dr::design::{stored_box, DesignRoutes};
use crate::dr::log::{Line, Sink};
use crate::dr::ta::Fixed;
use crate::polygon90::Rect;
use crate::tech::{Dir, LayerKind, MaxSpacingRule, Tech, ViaDef};

/// A via the design defines (DEF VIAS): io adds it to its cut layer's via defs as it does a
/// technology via. Only its cut class is read here.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BlockVia {
    pub name: String,
    pub cut: usize,
    /// Its cut class (from 0) when the via is written with parameters (`dbViaParams`: one cut size
    /// for every cut); `None` inside when no class has that size. `Err` when the via is written
    /// as rectangles, whose shapes are not read.
    pub cut_class: Result<Option<usize>, ()>,
}

/// What the pass keeps from call to call, as the reference keeps it on its technology and router.
#[derive(Debug, Default)]
pub struct MaxSpacingState {
    /// Per layer, its secondary via defs in order (`frLayer::secondaryViaDefs_`). ⚠️ The reference
    /// APPENDS to it on every call, so a second call lists each one twice (the via swap indexes it
    /// by its re-route count).
    pub secondary: Vec<Vec<ViaDef>>,
    /// The via defs `initSecondaryVias` generated (`<name>_FR`), as `frTechObject::addVia` keeps
    /// them by name.
    pub generated: Vec<ViaDef>,
    /// The via defs pin access avoids for a stubborn terminal (`addAvoidViaDefPA`), by name.
    pub avoid_via_defs_pa: BTreeSet<String>,
}

/// `FlexDR::fixMaxSpacing`, as far as it is ported: the sequence only. `defaults` is each cut
/// layer's default via (`frLayer::getDefaultViaDef`). Err: a stage that is not ported was reached.
#[allow(clippy::too_many_arguments)]
pub fn fix_max_spacing(tech: &Tech, defaults: &[Option<usize>], block_vias: &[BlockVia], fixed: &[Vec<(Rect, Fixed)>], routes: &DesignRoutes, state: &mut MaxSpacingState, log: Sink) -> Result<(), String> {
    log(Line::Info(227, "Checking For LEF58_MAXSPACING violations".into()));
    init_secondary_vias(tech, defaults, block_vias, state)?;
    let mut lonely_vias = Vec::new();
    for (l, layer) in tech.layers.iter().enumerate() {
        if layer.kind != LayerKind::Cut || layer.lef58_max_spacing.is_empty() {
            continue;
        }
        for rule in &layer.lef58_max_spacing {
            lonely_vias.extend(get_lonely_vias(tech, l, rule, state, fixed, routes));
            add_avoid_via_defs_pa(tech, l, rule, block_vias, state);
        }
    }
    if !lonely_vias.is_empty() {
        return Err(lonely_vias_refusal(tech, routes, &lonely_vias));
    }
    Ok(())
}

/// The refusal for a run whose pass found lonely vias: the repair stages are not ported.
fn lonely_vias_refusal(tech: &Tech, routes: &DesignRoutes, lonely: &[usize]) -> String {
    let mut per_layer: BTreeMap<usize, usize> = BTreeMap::new();
    for &k in lonely {
        if let Some(s) = &routes.shapes[k] {
            *per_layer.entry(stored_box(tech, &s.fig).0).or_default() += 1;
        }
    }
    let layers: Vec<String> = per_layer.iter().map(|(&l, n)| format!("{} {n}", tech.layers[l].name)).collect();
    format!(
        "lef58_max_spacing: {} via(s) have no other cut within the rule's MAXSPACING ({}); repairing them (the via swap after the lonely-via search) is not modelled",
        lonely.len(),
        layers.join(", ")
    )
}

/// `io::Parser::initSecondaryVias`: on every cut layer with a default via and a LEF58_MAXSPACING
/// rule, its via defs grouped by cut count (ascending), each group in `getViaRawPriority` order;
/// one of the default via's class is skipped; the rest are APPENDED to the layer's secondary list
/// — as they are, or, when an end-of-line enclosure would bloat it or its cut is off centre, as a
/// generated via `<name>_FR` (its shapes recentred on the cut). Err: a design via that would be a
/// secondary (its shapes are not read), or an end-of-line LEF58 enclosure on a secondary's class
/// (`getBloatingDist` is not ported).
pub fn init_secondary_vias(tech: &Tech, defaults: &[Option<usize>], block_vias: &[BlockVia], state: &mut MaxSpacingState) -> Result<(), String> {
    if state.secondary.len() < tech.layers.len() {
        state.secondary.resize(tech.layers.len(), Vec::new());
    }
    for (l, layer) in tech.layers.iter().enumerate() {
        if layer.kind != LayerKind::Cut {
            continue;
        }
        let Some(default) = defaults.get(l).copied().flatten() else { continue };
        if layer.lef58_max_spacing.is_empty() {
            continue;
        }
        let default_class = tech.via_defs[default].cut_class;
        // std::map<cuts, std::map<viaRawPriorityTuple, viaDef>> over the technology's vias (an
        // index into `tech.via_defs`); the tuple ends with the name. A design via is settled
        // first: of the default's class it is skipped wherever it sorts, else refused.
        let mut cuts_to_viadefs: BTreeMap<usize, BTreeMap<ViaRawPriority, usize>> = BTreeMap::new();
        for (v, vd) in tech.via_defs.iter().enumerate().filter(|(_, vd)| vd.cut == l) {
            cuts_to_viadefs.entry(vd.cut_figs.len()).or_default().insert(via_raw_priority(tech, vd), v);
        }
        for b in block_vias.iter().filter(|b| b.cut == l) {
            match b.cut_class {
                Ok(c) if c == default_class => continue,
                _ => return Err(format!("lef58_max_spacing: design via {} on {} would be a secondary via def of the max-spacing pass; its shapes are not read", b.name, layer.name)),
            }
        }
        for viadefs in cuts_to_viadefs.values() {
            for &v in viadefs.values() {
                let viadef = &tech.via_defs[v];
                if viadef.cut_class == default_class {
                    continue;
                }
                if let Some(c) = viadef.cut_class {
                    if layer.cut_enclosures.iter().any(|e| e.eol && e.class == c + 1) {
                        return Err(format!("lef58_max_spacing: via {} on {}: an end-of-line LEF58_ENCLOSURE of its cut class would bloat it as a secondary via def (getBloatingDist), which is not modelled", viadef.name, layer.name));
                    }
                }
                let cut_box = bbox(&viadef.cut_figs);
                let (dx, dy) = (cut_box.xl.midpoint_toward_zero(cut_box.xh), cut_box.yl.midpoint_toward_zero(cut_box.yh));
                if dx != 0 || dy != 0 {
                    if let Some(vd) = add_via(tech, state, recentred_via(viadef, cut_box, dx, dy)) {
                        state.secondary[l].push(vd);
                    }
                } else {
                    state.secondary[l].push(viadef.clone());
                }
            }
        }
    }
    Ok(())
}

/// io's generated secondary via: one rectangle per layer — the bounding boxes of the via's
/// shapes on each layer — moved by `(-dx, -dy)`, named `<name>_FR`, not a default, with no cut
/// class (io gives a router-added via none).
fn recentred_via(viadef: &ViaDef, cut_box: Rect, dx: i32, dy: i32) -> ViaDef {
    let mv = |r: Rect| Rect { xl: r.xl - dx, yl: r.yl - dy, xh: r.xh - dx, yh: r.yh - dy };
    ViaDef {
        name: format!("{}_FR", viadef.name),
        is_default: false,
        layer1: viadef.layer1,
        cut: viadef.cut,
        layer2: viadef.layer2,
        layer1_figs: vec![mv(bbox(&viadef.layer1_figs))],
        cut_figs: vec![mv(cut_box)],
        layer2_figs: vec![mv(bbox(&viadef.layer2_figs))],
        cut_class: None,
    }
}

/// `frTechObject::addVia`: a new name is added; a name already held returns the held via when it
/// equals the new one (`frViaDef::operator==`: name, cut class, default flag and the shapes as a
/// multiset), else nothing.
fn add_via(tech: &Tech, state: &mut MaxSpacingState, vd: ViaDef) -> Option<ViaDef> {
    let held = tech.via_defs.iter().chain(&state.generated).find(|h| h.name == vd.name);
    match held {
        Some(h) => via_def_eq(h, &vd).then(|| h.clone()),
        None => {
            state.generated.push(vd.clone());
            Some(vd)
        }
    }
}

fn via_def_eq(a: &ViaDef, b: &ViaDef) -> bool {
    let figs = |v: &ViaDef| {
        let mut f: Vec<(usize, Rect)> = v.layer1_figs.iter().map(|r| (v.layer1, *r)).chain(v.cut_figs.iter().map(|r| (v.cut, *r))).chain(v.layer2_figs.iter().map(|r| (v.layer2, *r))).collect();
        f.sort_by_key(|(l, r)| (*l, r.xl, r.yl, r.xh, r.yh));
        f
    };
    a.name == b.name && a.cut_class == b.cut_class && a.is_default == b.is_default && figs(a) == figs(b)
}

/// `viaRawPriorityTuple`: (not a default via, lower-layer width, upper-layer width, upper shape
/// not along its layer, total cut area, upper area, lower area, lower shape not along its layer,
/// name), compared in that order.
type ViaRawPriority = (bool, i32, i32, bool, i64, i64, i64, bool, String);

/// `io::Parser::getViaRawPriority`: a layer's width is the shorter side of the bounding box of
/// its shapes; a shape is "along" a horizontal layer when that box is wider than tall, along a
/// vertical one when not (a layer with neither direction is never misaligned); a layer's area is
/// the area of the union of its shapes.
pub fn via_raw_priority(tech: &Tech, vd: &ViaDef) -> ViaRawPriority {
    let side = |figs: &[Rect], layer: usize| {
        let b = bbox(figs);
        let horz = b.xh - b.xl > b.yh - b.yl;
        let width = (b.xh - b.xl).min(b.yh - b.yl);
        let not_align = (horz && tech.layers[layer].dir == Dir::Vertical) || (!horz && tech.layers[layer].dir == Dir::Horizontal);
        (width, not_align, union_area(figs))
    };
    let (l1_width, not_lower_align, l1_area) = side(&vd.layer1_figs, vd.layer1);
    let (l2_width, not_upper_align, l2_area) = side(&vd.layer2_figs, vd.layer2);
    let cut_area: i64 = vd.cut_figs.iter().map(|r| i64::from(r.xh - r.xl) * i64::from(r.yh - r.yl)).sum();
    (!vd.is_default, l1_width, l2_width, not_upper_align, cut_area, l2_area, l1_area, not_lower_align, vd.name.clone())
}

/// The area of a union of rectangles (`boost::polygon::area` of the polygon set they make).
fn union_area(figs: &[Rect]) -> i64 {
    let mut xs: Vec<i32> = figs.iter().flat_map(|r| [r.xl, r.xh]).collect();
    let mut ys: Vec<i32> = figs.iter().flat_map(|r| [r.yl, r.yh]).collect();
    xs.sort_unstable();
    xs.dedup();
    ys.sort_unstable();
    ys.dedup();
    let mut area = 0i64;
    for x in xs.windows(2) {
        for y in ys.windows(2) {
            if figs.iter().any(|r| r.xl <= x[0] && x[1] <= r.xh && r.yl <= y[0] && y[1] <= r.yh) {
                area += i64::from(x[1] - x[0]) * i64::from(y[1] - y[0]);
            }
        }
    }
    area
}

/// The bounding box of a via's shapes on one layer (empty figures give an empty box at 0).
fn bbox(figs: &[Rect]) -> Rect {
    let Some(first) = figs.first() else { return Rect { xl: 0, yl: 0, xh: 0, yh: 0 } };
    figs[1..].iter().fold(*first, |a, f| Rect { xl: a.xl.min(f.xl), yl: a.yl.min(f.yl), xh: a.xh.max(f.xh), yh: a.yh.max(f.yh) })
}

trait Midpoint {
    fn midpoint_toward_zero(self, other: Self) -> Self;
}

impl Midpoint for i32 {
    /// `odb::Rect::xCenter`: `(lo + hi) / 2`, C++ integer division (toward zero).
    fn midpoint_toward_zero(self, other: i32) -> i32 {
        ((i64::from(self) + i64::from(other)) / 2) as i32
    }
}

/// `FlexDR::getLonelyVias` for one rule on cut layer `l`: nothing when the layer has no secondary
/// via def; else every object the region query holds on the layer (`getVias`: the fixed shapes,
/// then the routed vias in their query's order) as the centre of its box, and the routed vias
/// among them whose centre has NO other centre within `max` (inclusive, Euclidean: the KD-tree's
/// `radiusSearch` keeps `d² <= max²`), of the rule's class, on a net that is neither fake nor
/// special — as committed-shape slots, in that order. The reference's `visited` skip changes
/// nothing: being within `max` is symmetric, so a point marked as another's neighbour is not
/// isolated either.
pub fn get_lonely_vias(tech: &Tech, l: usize, rule: &MaxSpacingRule, state: &MaxSpacingState, fixed: &[Vec<(Rect, Fixed)>], routes: &DesignRoutes) -> Vec<usize> {
    if state.secondary.get(l).is_none_or(Vec::is_empty) {
        return Vec::new();
    }
    let all = Rect { xl: i32::MIN, yl: i32::MIN, xh: i32::MAX, yh: i32::MAX };
    let routed = routes.query_layer(&all, l);
    let centre = |r: &Rect| (r.xl.midpoint_toward_zero(r.xh), r.yl.midpoint_toward_zero(r.yh));
    let mut points: Vec<(i32, i32)> = fixed.get(l).map_or(Vec::new(), |v| v.iter().map(|(r, _)| centre(r)).collect());
    let first_routed = points.len();
    for &k in &routed {
        let s = routes.shapes[k].as_ref().expect("a queried shape is live");
        points.push(centre(&stored_box(tech, &s.fig).1));
    }
    let isolated = isolated_points(&points, rule.max);
    isolated
        .into_iter()
        .filter(|&i| i >= first_routed)
        .map(|i| routed[i - first_routed])
        .filter(|&k| {
            let s = routes.shapes[k].as_ref().expect("a queried shape is live");
            matches!(s.fig, DrFig::Via { via, .. } if tech.via_defs[via].cut_class == Some(rule.cut_class))
        })
        .collect()
}

/// The indices (ascending, as `std::set<int>` iterates) of the points with no OTHER point within
/// `radius` (`dx² + dy² <= radius²`; a point at the same place counts).
pub fn isolated_points(points: &[(i32, i32)], radius: i32) -> Vec<usize> {
    let r = i64::from(radius);
    let mut by_x: Vec<usize> = (0..points.len()).collect();
    by_x.sort_by_key(|&i| points[i].0);
    let mut out = Vec::new();
    for i in 0..points.len() {
        let (x, y) = (i64::from(points[i].0), i64::from(points[i].1));
        let start = by_x.partition_point(|&j| i64::from(points[j].0) < x - r);
        let near = by_x[start..].iter().take_while(|&&j| i64::from(points[j].0) <= x + r).any(|&j| {
            let (dx, dy) = (i64::from(points[j].0) - x, i64::from(points[j].1) - y);
            j != i && dx * dx + dy * dy <= r * r
        });
        if !near {
            out.push(i);
        }
    }
    out
}

/// `FlexDR::fixMaxSpacing`'s second step per rule: every via def of the layer of the rule's class
/// goes on pin access's avoid list (`TritonRoute::addAvoidViaDefPA`) — technology vias and design
/// vias. It is read only for a stubborn terminal, which the later stages mark.
fn add_avoid_via_defs_pa(tech: &Tech, l: usize, rule: &MaxSpacingRule, block_vias: &[BlockVia], state: &mut MaxSpacingState) {
    for vd in tech.via_defs.iter().filter(|vd| vd.cut == l && vd.cut_class == Some(rule.cut_class)) {
        state.avoid_via_defs_pa.insert(vd.name.clone());
    }
    for b in block_vias.iter().filter(|b| b.cut == l && b.cut_class == Ok(Some(rule.cut_class))) {
        state.avoid_via_defs_pa.insert(b.name.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Rule (`FlexDR::getLonelyVias`, KDTree::radiusSearch): a point is isolated when no OTHER
    /// point lies within the radius, the boundary included (`d² <= r²`); two points at the same
    /// place are each other's neighbour.
    #[test]
    fn a_point_exactly_at_the_radius_is_a_neighbour() {
        assert_eq!(isolated_points(&[(0, 0), (30, 40)], 50), Vec::<usize>::new());
        assert_eq!(isolated_points(&[(0, 0), (30, 41)], 50), [0, 1]);
        assert_eq!(isolated_points(&[(7, 7), (7, 7), (100, 0)], 10), [2]);
        // Euclidean, not a box: (36, 36) is 50.9 away.
        assert_eq!(isolated_points(&[(0, 0), (36, 36)], 50), [0, 1]);
    }

    /// Rule (`getViaRawPriority`): the tuple orders by not-default first, then the lower and upper
    /// widths, and ties fall to the name; the area of a layer is its shapes' UNION.
    #[test]
    fn the_raw_priority_orders_default_first_then_width_then_name() {
        let mut t = Tech::default();
        for (name, kind, dir) in [("m1", LayerKind::Routing, Dir::Horizontal), ("v1", LayerKind::Cut, Dir::None), ("m2", LayerKind::Routing, Dir::Vertical)] {
            t.layers.push(crate::tech::Layer { name: name.into(), kind, dir, ..Default::default() });
        }
        let via = |name: &str, d: bool, w: i32| ViaDef { name: name.into(), is_default: d, layer1: 0, cut: 1, layer2: 2, layer1_figs: vec![Rect { xl: -w, yl: -5, xh: w, yh: 5 }], cut_figs: vec![Rect { xl: -5, yl: -5, xh: 5, yh: 5 }], layer2_figs: vec![Rect { xl: -5, yl: -w, xh: 5, yh: w }], cut_class: None };
        let mut v = [via("b", false, 20), via("a", false, 20), via("z", true, 30), via("c", false, 4)];
        v.sort_by_key(|vd| via_raw_priority(&t, vd));
        let names: Vec<&str> = v.iter().map(|vd| vd.name.as_str()).collect();
        // "c": lower width 8 (its box is 8 x 10) beats 10; "z" is the default.
        assert_eq!(names, ["z", "c", "a", "b"]);
        assert_eq!(union_area(&[Rect { xl: 0, yl: 0, xh: 10, yh: 10 }, Rect { xl: 5, yl: 0, xh: 15, yh: 10 }]), 150);
    }

    fn tech_with_rule() -> (Tech, Vec<Option<usize>>) {
        let mut t = Tech::default();
        for (name, kind, dir) in [("m1", LayerKind::Routing, Dir::Horizontal), ("v1", LayerKind::Cut, Dir::None), ("m2", LayerKind::Routing, Dir::Vertical)] {
            t.layers.push(crate::tech::Layer { name: name.into(), kind, dir, ..Default::default() });
        }
        t.layers[1].lef58_max_spacing = vec![MaxSpacingRule { max: 100, cut_class: 0 }];
        let sq = |h: i32| Rect { xl: -h, yl: -h, xh: h, yh: h };
        let via = |name: &str, class: Option<usize>, cut: Rect| ViaDef { name: name.into(), is_default: false, layer1: 0, cut: 1, layer2: 2, layer1_figs: vec![sq(20)], cut_figs: vec![cut], layer2_figs: vec![sq(20)], cut_class: class };
        t.via_defs = vec![via("A", Some(0), sq(10)), via("B", Some(1), sq(8)), via("C", Some(1), Rect { xl: -8, yl: -4, xh: 8, yh: 12 }), via("D", Some(0), sq(10))];
        (t, vec![None, Some(0), None])
    }

    /// Rule (`io::Parser::initSecondaryVias`): a via of the default via's class is never a
    /// secondary; one with its cut centred is kept as it is; one with its cut off centre becomes
    /// `<name>_FR`, its shapes moved so the cut is centred; and a second call APPENDS the same list
    /// again (the generated via found by name and equal, so it is reused).
    #[test]
    fn secondaries_skip_the_default_class_recentre_off_centre_cuts_and_append_per_call() {
        let (t, defaults) = tech_with_rule();
        let mut st = MaxSpacingState::default();
        init_secondary_vias(&t, &defaults, &[], &mut st).expect("ported");
        let names: Vec<&str> = st.secondary[1].iter().map(|v| v.name.as_str()).collect();
        // By priority: B's cut area 256 < C's 256? equal; lower width equal; so the name decides.
        assert_eq!(names, ["B", "C_FR"]);
        assert_eq!(st.secondary[1][1].cut_figs, [Rect { xl: -8, yl: -8, xh: 8, yh: 8 }]);
        assert_eq!(st.secondary[1][1].layer1_figs, [Rect { xl: -20, yl: -24, xh: 20, yh: 16 }]);
        assert_eq!(st.secondary[1][1].cut_class, None);
        init_secondary_vias(&t, &defaults, &[], &mut st).expect("ported");
        let names: Vec<&str> = st.secondary[1].iter().map(|v| v.name.as_str()).collect();
        assert_eq!(names, ["B", "C_FR", "B", "C_FR"]);
        assert_eq!(st.generated.len(), 1);
    }

    /// Rule (`initSecondaryVias`): a layer with no default via, or no max-spacing rule, gets no
    /// secondary; a design via of another class than the default's is refused (its shapes are not
    /// read), one of the default's class is skipped as io skips it.
    #[test]
    fn a_layer_without_default_or_rule_has_no_secondary_and_a_foreign_design_via_is_refused() {
        let (mut t, defaults) = tech_with_rule();
        let mut st = MaxSpacingState::default();
        init_secondary_vias(&t, &[None, None, None], &[], &mut st).expect("ported");
        assert!(st.secondary[1].is_empty());
        let same = BlockVia { name: "dv".into(), cut: 1, cut_class: Ok(Some(0)) };
        init_secondary_vias(&t, &defaults, std::slice::from_ref(&same), &mut st).expect("skipped as the default's class");
        let other = BlockVia { cut_class: Ok(Some(1)), ..same.clone() };
        assert!(init_secondary_vias(&t, &defaults, &[other], &mut MaxSpacingState::default()).is_err());
        let unread = BlockVia { cut_class: Err(()), ..same };
        assert!(init_secondary_vias(&t, &defaults, &[unread], &mut MaxSpacingState::default()).is_err());
        t.layers[1].lef58_max_spacing.clear();
        let mut st = MaxSpacingState::default();
        init_secondary_vias(&t, &defaults, &[], &mut st).expect("ported");
        assert!(st.secondary[1].is_empty());
    }

    /// Rule (`frRegionQuery::getVias` + `getLonelyVias`): a routed via stands at the CENTRE OF ITS
    /// BOX over all three layers (`frVia::getBBox`, as the region query stores it), not at its
    /// origin: a via with its enclosure off to one side sits nearer its neighbour.
    #[test]
    fn a_via_stands_at_its_box_centre_not_its_origin() {
        let (mut t, defaults) = tech_with_rule();
        let mut e = t.via_defs[0].clone();
        e.name = "E".into();
        e.layer1_figs = vec![Rect { xl: -20, yl: -20, xh: 200, yh: 20 }];
        t.via_defs.push(e);
        let via = |v: usize, x: i32| DrFig::Via { via: v, origin: (x, 0), bi: (0, 0, 0), ei: (0, 0, 0), tapered: false, bottom_connected: false, top_connected: false };
        // E at 0: box (-20..200), centre 90; A at 180: 90 apart (within 100), origins 180 apart.
        let routes = DesignRoutes::with_initial(&t, vec![(0, via(4, 0)), (0, via(0, 180))]);
        let mut st = MaxSpacingState::default();
        init_secondary_vias(&t, &defaults, &[], &mut st).expect("ported");
        let rule = t.layers[1].lef58_max_spacing[0];
        assert!(get_lonely_vias(&t, 1, &rule, &st, &[Vec::new(), Vec::new(), Vec::new()], &routes).is_empty());
    }

    /// Rule (`FlexDR::getLonelyVias`): only routed vias of the rule's class are lonely, a fixed
    /// shape's centre is a neighbour like any other, and with no secondary via def on the layer
    /// nothing is lonely.
    #[test]
    fn lonely_vias_are_routed_vias_of_the_class_with_no_cut_nearby() {
        let (t, defaults) = tech_with_rule();
        let via = |v: usize, x: i32| DrFig::Via { via: v, origin: (x, 0), bi: (0, 0, 0), ei: (0, 0, 0), tapered: false, bottom_connected: false, top_connected: false };
        // 0: alone (class 0) — lonely; 1000/1080: a pair; 3000: class 1, alone — not of the class;
        // 5000: next to a fixed shape at 5090.
        let routes = DesignRoutes::with_initial(&t, vec![(0, via(0, 0)), (0, via(3, 1000)), (1, via(0, 1080)), (2, via(1, 3000)), (3, via(0, 5000))]);
        let fixed = vec![Vec::new(), vec![(Rect { xl: 5080, yl: -10, xh: 5100, yh: 10 }, Fixed::Blockage)], Vec::new()];
        let rule = t.layers[1].lef58_max_spacing[0];
        assert!(get_lonely_vias(&t, 1, &rule, &MaxSpacingState::default(), &fixed, &routes).is_empty());
        let mut st = MaxSpacingState::default();
        init_secondary_vias(&t, &defaults, &[], &mut st).expect("ported");
        assert_eq!(get_lonely_vias(&t, 1, &rule, &st, &fixed, &routes), [0]);
        let no_fixed = vec![Vec::new(); 3];
        let mut got = get_lonely_vias(&t, 1, &rule, &st, &no_fixed, &routes);
        got.sort_unstable();
        assert_eq!(got, [0, 4]);
    }
}
