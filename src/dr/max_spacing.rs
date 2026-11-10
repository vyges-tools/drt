// SPDX-License-Identifier: Apache-2.0
//! LEF58_MAXSPACING: the router's repair pass (`FlexDR::fixMaxSpacing`), run after an iteration
//! ends with no marker standing, stage by stage in the reference's order and named after its
//! functions.
//!
//! A rule `MAXSPACING max CUTCLASS c` asks that every via of class `c` have another cut within
//! `max` of it. The reference finds the vias that have none (`getLonelyVias`), marks them, and
//! re-routes a region around each with a via-swap worker that tries the layer's SECONDARY via
//! defs (`initSecondaryVias`: the vias of another class). Ported: the sequencer, the secondary
//! via defs, the lonely-via search, the pin-access avoid list, and per lonely via its marker, its
//! region and the pin terminals it marks stubborn; the pin-access re-run for the stubborn
//! terminals (`updateDirtyPAData`, [`crate::pa::dirty`]); the regions' merge; the via-swap
//! workers (the router's, [`ViaSwapWorkers`]; the swap itself is `crate::dr::queue`'s, with
//! upstream fix 0013 for OpenROAD issue #12203, register D11). ⛔ Not ported: minimum-step patching of a swapped via.
//!
//! Where the REFERENCE itself aborts on the input (an upstream defect, filed), this router
//! refuses at the same point with a reason made by [`upstream_abort_reason`], which names the
//! reference's error and the issue; [`upstream_abort`] reads it back (the CLI's own exit code).

use std::collections::{BTreeMap, BTreeSet};

use std::collections::HashMap;

use crate::dr::cost::DrFig;
use crate::dr::design::{stored_box, DesignRoutes};
use crate::dr::log::{Line, Sink};
use crate::dr::guides::GCellGrid;
use crate::dr::ta::Fixed;
use crate::gc::{Marker, Owner, Rule};
use crate::pa::access::Config;
use crate::pa::dirty::{update_dirty_insts, DirtyError, DirtyIn};
use crate::pa::flow::{DesignInst, PinAccess};
use crate::order::libcxx_sort_by;
use crate::polygon90::Rect;
use crate::rtree::PackedRTree;
use crate::tech::{Dir, LayerKind, Master, MaxSpacingRule, Tech, TrackPattern, ViaDef};

/// A point where the reference aborts on its input: its error and the upstream issue filed for it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UpstreamAbort {
    pub code: &'static str,
    pub issue: u32,
}

/// The reference's aborts this pass refuses in its place: DRT-0073, a stubborn pin left with no
/// access point by the pin-access re-run (OpenROAD issue #12199); DRT-1231, a worker that overlaps
/// a pin whose access points the re-run moved outside it (OpenROAD issue #12200).
pub const UPSTREAM_ABORTS: [UpstreamAbort; 2] = [UpstreamAbort { code: "DRT-0073", issue: 12199 }, UpstreamAbort { code: "DRT-1231", issue: 12200 }];

/// The refusal's reason where the reference aborts: `lef58_max_spacing: the reference aborts here
/// with CODE (OpenROAD issue #N): DETAIL`.
pub fn upstream_abort_reason(abort: UpstreamAbort, detail: &str) -> String {
    format!("lef58_max_spacing: the reference aborts here with {} (OpenROAD issue #{}): {detail}", abort.code, abort.issue)
}

/// The abort a refusal's reason names, when [`upstream_abort_reason`] made it.
pub fn upstream_abort(reason: &str) -> Option<UpstreamAbort> {
    UPSTREAM_ABORTS.into_iter().find(|a| reason.starts_with(&format!("lef58_max_spacing: the reference aborts here with {} (OpenROAD issue #{}): ", a.code, a.issue)))
}

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
    /// The instance terminals marked stubborn (`frInstTerm::setStubborn`), as (instance, terminal).
    pub stubborn: BTreeSet<(usize, usize)>,
    /// The instances whose pin access is to be re-run (`FlexPA::dirty_insts_`, ordered by id;
    /// cleared by the re-run).
    pub dirty_insts: BTreeSet<usize>,
    /// Pin access as the re-runs left it (`FlexPA`'s state, kept from call to call); `None` until
    /// the first re-run with a dirty instance (until then it is the first run's).
    pub pa: Option<PinAccess>,
    /// How many re-runs changed pin access (the router rebuilds what its workers read of it).
    pub pa_reruns: usize,
    /// ⛔ MEASUREMENT ONLY: log the pass's steps as the correlation instrument prints the
    /// reference's (`VYC MAXSPC MARK/STUB/MERGED`), as lines of this engine's own.
    pub trace: bool,
}

/// What the pass reads of the design besides its routes.
pub struct PassIn<'a> {
    pub tech: &'a Tech,
    /// Each cut layer's default via (`frLayer::getDefaultViaDef`).
    pub defaults: &'a [Option<usize>],
    pub block_vias: &'a [BlockVia],
    /// The design's shapes per layer, in the region query's order, and their packed trees.
    pub fixed: &'a [Vec<(Rect, Fixed)>],
    pub fixed_trees: &'a [PackedRTree<Fixed>],
    pub grid: &'a GCellGrid,
    /// A net's name; an instance terminal's instance and terminal names.
    pub net_name: &'a dyn Fn(usize) -> String,
    pub inst_term_name: &'a dyn Fn(usize, usize) -> (String, String),
    /// What the pin-access re-run reads; `None`: not available (the re-run is refused).
    pub pa: Option<PaIn<'a>>,
}

/// Pin access's inputs as its first run read them, and its result.
#[derive(Clone, Copy)]
pub struct PaIn<'a> {
    pub tech: &'a Tech,
    pub tracks: &'a [TrackPattern],
    pub cfg: &'a Config,
    pub masters: &'a HashMap<String, Master>,
    pub insts: &'a [DesignInst],
    pub first: &'a PinAccess,
}

/// `FlexDR::fixMaxSpacing`, as far as it is ported: the sequence only, each stage named after the
/// reference's step. Err: a stage that is not ported was reached (nothing is written).
pub fn fix_max_spacing(p: &PassIn<'_>, routes: &mut DesignRoutes, state: &mut MaxSpacingState, log: Sink, swap: ViaSwapWorkers<'_>) -> Result<(), String> {
    log(Line::Info(227, "Checking For LEF58_MAXSPACING violations".into()));
    init_secondary_vias(p.tech, p.defaults, p.block_vias, state)?;
    let mut lonely_vias = Vec::new();
    for (l, layer) in p.tech.layers.iter().enumerate() {
        if layer.kind != LayerKind::Cut || layer.lef58_max_spacing.is_empty() {
            continue;
        }
        for rule in &layer.lef58_max_spacing {
            lonely_vias.extend(get_lonely_vias(p.tech, l, rule, state, p.fixed, routes));
            add_avoid_via_defs_pa(p.tech, l, rule, p.block_vias, state);
        }
    }
    let mut regions = Vec::with_capacity(lonely_vias.len());
    for &k in &lonely_vias {
        mark_lonely(routes, k);
        let via = lonely_via(p, routes, k);
        add_lonely_via_marker(p, routes, &via);
        regions.push(lonely_via_region(p.grid, via.origin));
        let stubborn = if via.bottom_connected || via.top_connected { connected_inst_terms(p, &via) } else { Vec::new() };
        if state.trace {
            trace_mark(p, &via, regions[regions.len() - 1], log);
        }
        for &(inst, term) in &stubborn {
            if state.trace {
                let (i, t) = (p.inst_term_name)(inst, term);
                log(Line::OwnInfo(format!("VYC MAXSPC STUB inst={i} term={t}")));
            }
            state.stubborn.insert((inst, term));
            state.dirty_insts.insert(inst);
        }
    }
    update_dirty_pa_data(p, state, log)?;
    let merged = merge_regions(regions)?;
    if state.trace {
        for r in &merged {
            log(Line::OwnInfo(format!("VYC MAXSPC MERGED ({},{},{},{})", r.xl, r.yl, r.xh, r.yh)));
        }
    }
    swap(routes, &merged, state, log)
}

/// The via-swap workers, one per merged region (`RipUpMode::VIASWAP`), run by the router (they are
/// its workers): the routes, the merged regions, the pass's state (its secondary via defs, pin
/// access as the re-run left it), the log.
pub type ViaSwapWorkers<'a> = &'a mut dyn FnMut(&mut DesignRoutes, &[Rect], &MaxSpacingState, Sink) -> Result<(), String>;

/// A lonely via as the pass reads it: its net, def, origin, the box over its three layers, and
/// which of its ends lands on a pin's access point.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LonelyVia {
    pub net: usize,
    pub via: usize,
    pub origin: (i32, i32),
    pub bbox: Rect,
    pub bottom_connected: bool,
    pub top_connected: bool,
}

/// `via->setIsLonely(true)` (in `getLonelyVias`): the committed via at slot `k` is flagged lonely,
/// for good (see [`DrFig::Via`]'s `lonely`).
fn mark_lonely(routes: &mut DesignRoutes, k: usize) {
    if let Some(DrFig::Via { lonely, .. }) = routes.shapes[k].as_mut().map(|s| &mut s.fig) {
        *lonely = true;
    }
}

fn lonely_via(p: &PassIn<'_>, routes: &DesignRoutes, k: usize) -> LonelyVia {
    let s = routes.shapes[k].as_ref().expect("a lonely via is a live shape");
    let DrFig::Via { via, origin, bottom_connected, top_connected, .. } = s.fig else { unreachable!("a lonely via is a via") };
    LonelyVia { net: s.net, via, origin, bbox: stored_box(p.tech, &s.fig).1, bottom_connected, top_connected }
}

/// The marker for a lonely via (`frMarker`): over the via's box (`frVia::getBBox`, its three
/// layers), on its cut layer, under the layer's FIRST max-spacing rule (`.at(0)`, whatever rule
/// found it), the via's net as source, victim and aggressor (layer, box, not fixed); added to the
/// region query and the design's list.
pub fn add_lonely_via_marker(p: &PassIn<'_>, routes: &mut DesignRoutes, via: &LonelyVia) {
    let layer = p.tech.via_defs[via.via].cut;
    let owner = Owner::Net((p.net_name)(via.net));
    let side = Some((owner.clone(), layer, via.bbox, false));
    routes.add_marker(Marker { rule: Rule::Lef58MaxSpacing, layer, bbox: via.bbox, owners: vec![owner], victim: side.clone(), aggressor: side, second_victim: None });
}

/// The region a worker re-routes around a lonely via: three by three gcells centred on the gcell
/// of the via's ORIGIN (`getGCellIdx`, not its box's centre), from the low corner of the gcell
/// below-left to the high corner of the one above-right, each clamped into the grid as
/// `getGCellBox` clamps it (an edge gcell reaches the die).
pub fn lonely_via_region(grid: &GCellGrid, origin: (i32, i32)) -> Rect {
    let (x, y) = grid.idx(origin);
    let (lo, hi) = (grid.gcell_box((x - 1, y - 1)), grid.gcell_box((x + 1, y + 1)));
    Rect { xl: lo.xl, yl: lo.yl, xh: hi.xh, yh: hi.yh }
}

/// The instance terminals a connected lonely via lands on: the design's shapes on the layer of
/// the connected end (the TOP one when both are, as the reference's `isTopConnected() ? … : …`
/// picks), touching that end's box (`getLayer2BBox` / `getLayer1BBox`, inclusive), that are an
/// instance terminal of the via's net — once PER PIN SHAPE, in the query's order (a terminal
/// with two shapes under the via is marked twice, as the reference marks it).
pub fn connected_inst_terms(p: &PassIn<'_>, via: &LonelyVia) -> Vec<(usize, usize)> {
    let vd = &p.tech.via_defs[via.via];
    let (layer, figs) = if via.top_connected { (vd.layer2, &vd.layer2_figs) } else { (vd.layer1, &vd.layer1_figs) };
    let b = bbox(figs);
    let q = Rect { xl: b.xl + via.origin.0, yl: b.yl + via.origin.1, xh: b.xh + via.origin.0, yh: b.yh + via.origin.1 };
    p.fixed_trees.get(layer).map_or(Vec::new(), |tr| {
        tr.query(&q)
            .into_iter()
            .filter_map(|(_, f)| match *f {
                Fixed::InstTerm { net: Some(n), inst, term } if n == via.net => Some((inst, term)),
                _ => None,
            })
            .collect()
    })
}

fn trace_mark(p: &PassIn<'_>, via: &LonelyVia, region: Rect, log: Sink) {
    let (b, o) = (via.bbox, via.origin);
    let g = p.grid.idx(o);
    log(Line::OwnInfo(format!(
        "VYC MAXSPC MARK net={} def={} box=({},{},{},{}) origin=({},{}) gcell=({},{}) region=({},{},{},{}) top={} bot={}",
        (p.net_name)(via.net),
        p.tech.via_defs[via.via].name,
        b.xl,
        b.yl,
        b.xh,
        b.yh,
        o.0,
        o.1,
        g.0,
        g.1,
        region.xl,
        region.yl,
        region.xh,
        region.yh,
        u8::from(via.top_connected),
        u8::from(via.bottom_connected)
    )));
}

/// `TritonRoute::updateDirtyPAData`: pin access re-run for the instances whose terminals the pass
/// marked stubborn (`FlexPA::updateDirtyInsts`, [`update_dirty_insts`]), on pin access as the
/// last re-run left it; with no dirty instance it changes nothing. The re-run's write-back into
/// the database (`io::Writer::updateDb`, pin access only) is not made: this router writes no
/// access point into its output at all (the routed DEF has none). Err: the reference aborts
/// (DRT-0073: a routed terminal of a re-run class with no access point, OpenROAD issue #12199 —
/// refused with [`upstream_abort_reason`]), or a case not modelled.
fn update_dirty_pa_data(p: &PassIn<'_>, state: &mut MaxSpacingState, log: Sink) -> Result<(), String> {
    if state.dirty_insts.is_empty() {
        return Ok(());
    }
    let Some(pin) = p.pa else {
        return Err(format!(
            "lef58_max_spacing: lonely vias land on {} pin(s) of {} instance(s); re-running pin access for them (TritonRoute::updateDirtyPAData, after the lonely-via markers) is not modelled",
            state.stubborn.len(),
            state.dirty_insts.len()
        ));
    };
    // `addAvoidViaDefPA` keeps via defs; pin access's own list is the technology's, by name.
    let avoid: BTreeSet<usize> = pin.tech.via_defs.iter().enumerate().filter(|(_, v)| state.avoid_via_defs_pa.contains(&v.name)).map(|(k, _)| k).collect();
    let inp = DirtyIn { tech: pin.tech, tracks: pin.tracks, cfg: pin.cfg, masters: pin.masters, insts: pin.insts, avoid: &avoid, stubborn: &state.stubborn };
    let pa = state.pa.get_or_insert_with(|| pin.first.clone());
    let mut sink = |l: String| log(Line::OwnInfo(l));
    let mut trace: crate::pa::dirty::TraceSink<'_> = if state.trace { Some(&mut sink) } else { None };
    let done = update_dirty_insts(&inp, pa, &state.dirty_insts, &mut trace);
    state.dirty_insts.clear();
    state.pa_reruns += 1;
    done.map_err(|e| match e {
        // ⚠️ INTERIM stage boundary. The clean reference aborts here (DRT-0073, OpenROAD issue
        // #12199). Once the patched reference build exists (vyges-openroad-patches, the fix for
        // #12199), this arm is the slot where its behaviour is transcribed and correlated against
        // that build instead of refused.
        DirtyError::NoAccessPoint { inst, term, master } => upstream_abort_reason(UPSTREAM_ABORTS[0], &format!("the pin-access re-run for the stubborn terminals finds no access point for {inst}/{term} ({master}) once the avoided via defs are dropped")),
        DirtyError::NoRowPath(row) => format!("lef58_max_spacing: the pin-access re-run finds no access pattern combination for the row {row} (the reference aborts with DRT-0085)"),
        DirtyError::Unsupported(u) => u,
    })
}

/// The regions merged: sorted by left edge with libc++'s `std::sort` (not stable: past 24 regions
/// two with the same left edge can swap), then each in turn absorbed by the FIRST merged region it
/// meets (`odb::Rect::intersects`: touching counts) or kept as a new one. A single pass: a region
/// grown by a merge is not re-merged with the ones before it. Err: the sort reached its heap
/// fallback (not transcribed).
pub fn merge_regions(mut regions: Vec<Rect>) -> Result<Vec<Rect>, String> {
    libcxx_sort_by(&mut regions, |a, b| a.xl < b.xl).map_err(|_| "lef58_max_spacing: the region sort reached libc++'s heap-sort fallback, which is not transcribed".to_string())?;
    let mut merged: Vec<Rect> = Vec::new();
    for r in regions {
        match merged.iter_mut().find(|m| r.xl <= m.xh && m.xl <= r.xh && r.yl <= m.yh && m.yl <= r.yh) {
            Some(m) => *m = Rect { xl: m.xl.min(r.xl), yl: m.yl.min(r.yl), xh: m.xh.max(r.xh), yh: m.yh.max(r.yh) },
            None => merged.push(r),
        }
    }
    Ok(merged)
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

/// The routing layers where the via-swap workers' check would patch a lonely via for minimum step,
/// which this router does not model: a layer with a MINSTEP rule next to a cut layer that keeps a
/// LEF58_MAXSPACING rule. Reference: the GC runs `patchMetalShape` on every surgical check when the
/// technology has any minimum-step rule (`addUConstraint` sets `hasCornerSpacingConstraint_` for
/// MINSTEP and LEF58 MINSTEP); `patchMetalShape_minStep` then acts on a minimum-step marker of a
/// layer with a via-to-via minimum-step violation (refused before routing on its own) or next to a
/// cut layer with a LEF58_MAXSPACING constraint, and on the second only for a lonely via in a
/// VIASWAP worker, bloating the marker to the layer's MINSTEP length. Whether a swapped secondary
/// leaves such a marker depends on the routed geometry, so the layer is refused as a whole.
pub fn min_step_swap_layers(tech: &Tech) -> Vec<usize> {
    let max_spacing_at = |l: Option<usize>| l.and_then(|l| tech.layers.get(l)).is_some_and(|c| c.kind == LayerKind::Cut && !c.lef58_max_spacing.is_empty());
    (0..tech.layers.len())
        .filter(|&l| tech.layers[l].kind == LayerKind::Routing && tech.layers[l].min_step.is_some())
        .filter(|&l| max_spacing_at(l.checked_sub(1)) || max_spacing_at(Some(l + 1)))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The refusal where the reference aborts names its error and the upstream issue, and is read
    /// back as that abort (the CLI's exit 4); any other refusal is not one.
    #[test]
    fn an_upstream_abort_reason_reads_back_as_its_abort() {
        for a in UPSTREAM_ABORTS {
            let r = upstream_abort_reason(a, "detail");
            assert!(r.contains(a.code) && r.contains(&format!("#{}", a.issue)), "{r}");
            assert_eq!(upstream_abort(&r), Some(a));
        }
        assert_eq!(upstream_abort("lef58_max_spacing: 4 via(s) have no other cut"), None);
        assert_eq!(UPSTREAM_ABORTS[0], UpstreamAbort { code: "DRT-0073", issue: 12199 });
    }

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

    /// Rule (`FlexGCWorker::Impl::patchMetalShape_minStep`, its VIASWAP branch): a lonely via is
    /// patched for minimum step only on a layer with a MINSTEP rule beside a cut layer that keeps a
    /// LEF58_MAXSPACING rule. Not modelled, so such a layer is named for refusal; a MINSTEP layer
    /// with no max-spacing cut beside it, or a max-spacing cut with no MINSTEP beside it, is not.
    #[test]
    fn a_min_step_layer_beside_a_max_spacing_cut_is_named_for_refusal() {
        let (mut t, _) = tech_with_rule();
        assert!(min_step_swap_layers(&t).is_empty(), "no MINSTEP anywhere");
        let ms = crate::tech::MinStep { min_step_length: 50, kind: None, max_length: -1, max_edges: -1 };
        t.layers[2].min_step = Some(ms);
        assert_eq!(min_step_swap_layers(&t), [2], "m2 has MINSTEP and v1 below it keeps the rule");
        t.layers[0].min_step = Some(ms);
        assert_eq!(min_step_swap_layers(&t), [0, 2], "m1, below v1, too");
        t.layers[1].lef58_max_spacing.clear();
        assert!(min_step_swap_layers(&t).is_empty(), "MINSTEP but no max-spacing cut beside it");
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
        let via = |v: usize, x: i32| DrFig::Via { via: v, origin: (x, 0), bi: (0, 0, 0), ei: (0, 0, 0), tapered: false, bottom_connected: false, top_connected: false, lonely: false };
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
        let via = |v: usize, x: i32| DrFig::Via { via: v, origin: (x, 0), bi: (0, 0, 0), ei: (0, 0, 0), tapered: false, bottom_connected: false, top_connected: false, lonely: false };
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

    fn grid() -> GCellGrid {
        // 10 x 10 gcells of 100 from 0; the die a little wider than the grid.
        GCellGrid { x: (0, 10, 100), y: (0, 10, 100), die: Rect { xl: -5, yl: -5, xh: 1010, yh: 1010 } }
    }

    /// Rule (`FlexDR::fixMaxSpacing`, the region): three by three gcells around the gcell of the
    /// via's ORIGIN (`getGCellIdx`), corners from `getGCellBox` of the gcells below-left and
    /// above-right, each clamped into the grid — an edge gcell reaches the die.
    #[test]
    fn a_region_is_three_by_three_gcells_around_the_origin_clamped_to_the_die() {
        let g = grid();
        assert_eq!(lonely_via_region(&g, (450, 450)), Rect { xl: 300, yl: 300, xh: 600, yh: 600 });
        // On a gcell line the origin belongs to the gcell above it.
        assert_eq!(lonely_via_region(&g, (500, 500)), Rect { xl: 400, yl: 400, xh: 700, yh: 700 });
        // At the edges: gcell (0, 9); the one below-left clamps to (0, 8), whose low x is the die's.
        assert_eq!(lonely_via_region(&g, (10, 990)), Rect { xl: -5, yl: 800, xh: 200, yh: 1010 });
    }

    /// Rule (`FlexDR::fixMaxSpacing`, the merge): sorted by left edge, each region joins the FIRST
    /// merged region it meets (touching counts, `odb::Rect::intersects`), else starts one; one pass
    /// only, so a region grown by a later merge is not re-merged with an earlier one.
    #[test]
    fn regions_merge_into_the_first_they_touch_in_one_pass() {
        let r = |xl, yl, xh, yh| Rect { xl, yl, xh, yh };
        // Sorted by left edge: (0..200), (100..150), (300..500), (500..600). The second touches
        // the first along y 100; the fourth touches the third along x 500 and y 50.
        let got = merge_regions(vec![r(300, 0, 500, 50), r(0, 0, 200, 100), r(500, 50, 600, 60), r(100, 100, 150, 200)]).expect("small");
        assert_eq!(got, [r(0, 0, 200, 200), r(300, 0, 600, 60)]);
        // First match, one pass: a and b (same left edge, input order) stay apart; c joins a; d
        // meets both a and b and joins a, the first; a grown over b is not merged with it.
        let got = merge_regions(vec![r(0, 0, 10, 10), r(0, 100, 10, 110), r(5, 5, 20, 8), r(8, 9, 30, 105)]).expect("small");
        assert_eq!(got, [r(0, 0, 30, 105), r(0, 100, 10, 110)]);
    }

    /// Rule (libc++ `std::sort` in `fixMaxSpacing`): past 24 regions, regions with the same left
    /// edge come out in libc++'s order, not their input order; it is the order the merged list
    /// keeps. The permutation is the reference's libc++'s (see `order`).
    #[test]
    fn the_region_sort_is_libcxx_not_stable() {
        let regions: Vec<Rect> = (0..30).map(|i: i32| Rect { xl: (i * 7 % 4) * 1000, yl: i * 100, xh: (i * 7 % 4) * 1000 + 10, yh: i * 100 + 10 }).collect();
        let got = merge_regions(regions).expect("no heap fallback");
        let order: Vec<i32> = got.iter().map(|r| r.yl / 100).collect();
        assert_eq!(order, [8, 28, 24, 20, 4, 16, 0, 12, 15, 11, 7, 19, 3, 23, 27, 10, 14, 6, 18, 22, 2, 26, 9, 13, 5, 17, 21, 25, 1, 29]);
    }

    fn pass_in<'a>(t: &'a Tech, defaults: &'a [Option<usize>], fixed: &'a [Vec<(Rect, Fixed)>], trees: &'a [PackedRTree<Fixed>], g: &'a GCellGrid) -> PassIn<'a> {
        PassIn { tech: t, defaults, block_vias: &[], fixed, fixed_trees: trees, grid: g, net_name: &|n| format!("n{n}"), inst_term_name: &|i, t| (format!("i{i}"), format!("t{t}")), pa: None }
    }

    /// Rule (`FlexDR::fixMaxSpacing`, stubborn terminals): the design's shapes on the layer of the
    /// connected end — the TOP when the via says top-connected — touching that end's box, that are
    /// an instance terminal of the via's own net, once per pin shape in the query's order.
    #[test]
    fn a_connected_via_marks_its_own_nets_terminals_once_per_pin_shape() {
        let (t, defaults) = tech_with_rule();
        let g = grid();
        let pin = |xl, xh| Rect { xl, yl: -10, xh, yh: 10 };
        let fixed = vec![
            vec![(pin(-30, -15), Fixed::InstTerm { net: Some(0), inst: 1, term: 2 }), (pin(15, 30), Fixed::InstTerm { net: Some(0), inst: 1, term: 2 }), (pin(0, 5), Fixed::InstTerm { net: Some(9), inst: 3, term: 0 }), (pin(0, 5), Fixed::BTerm { net: Some(0), port: 0 })],
            Vec::new(),
            vec![(pin(0, 5), Fixed::InstTerm { net: Some(0), inst: 7, term: 1 })],
        ];
        let trees: Vec<PackedRTree<Fixed>> = fixed.iter().map(|v| PackedRTree::new(v.clone())).collect();
        let p = pass_in(&t, &defaults, &fixed, &trees, &g);
        let v = LonelyVia { net: 0, via: 0, origin: (0, 0), bbox: Rect { xl: -20, yl: -20, xh: 20, yh: 20 }, bottom_connected: true, top_connected: false };
        let mut got = connected_inst_terms(&p, &v);
        got.sort_unstable();
        // Both shapes of (1, 2) touch the enclosure (-20..20); the other net's and the block pin's do not count.
        assert_eq!(got, [(1, 2), (1, 2)]);
        let top = LonelyVia { top_connected: true, ..v };
        assert_eq!(connected_inst_terms(&p, &top), [(7, 1)]);
        let both = LonelyVia { top_connected: true, bottom_connected: true, ..v };
        assert_eq!(connected_inst_terms(&p, &both), [(7, 1)]);
    }

    /// Rule (`FlexDR::fixMaxSpacing`): the sequence marks every lonely via (box over its three
    /// layers, on the cut layer, the net as owner, victim and aggressor) and flags it lonely
    /// (`setIsLonely(true)`), then hands the merged regions to the via-swap workers — or stops at
    /// the pin-access re-run when a terminal was marked stubborn and pin access is not available;
    /// a pass with no lonely via changes nothing and runs no worker.
    #[test]
    fn the_pass_marks_lonely_vias_then_runs_the_workers_on_the_merged_regions() {
        let (t, defaults) = tech_with_rule();
        let g = grid();
        let via = |x: i32, bottom: bool| DrFig::Via { via: 0, origin: (x, 450), bi: (0, 0, 0), ei: (0, 0, 0), tapered: false, bottom_connected: bottom, top_connected: false, lonely: false };
        let pin = (Rect { xl: 440, yl: 440, xh: 460, yh: 460 }, Fixed::InstTerm { net: Some(0), inst: 4, term: 0 });
        let fixed = vec![vec![pin], Vec::new(), Vec::new()];
        let trees: Vec<PackedRTree<Fixed>> = fixed.iter().map(|v| PackedRTree::new(v.clone())).collect();
        let p = pass_in(&t, &defaults, &fixed, &trees, &g);
        let mut lines = Vec::new();
        let mut routes = DesignRoutes::with_initial(&t, vec![(0, via(450, false))]);
        let mut st = MaxSpacingState::default();
        let mut seen: Vec<Vec<Rect>> = Vec::new();
        fix_max_spacing(&p, &mut routes, &mut st, &mut |l| lines.push(l), &mut |_, merged, _, _| {
            seen.push(merged.to_vec());
            Ok(())
        })
        .expect("the workers run");
        assert_eq!(seen, [vec![lonely_via_region(&g, (450, 450))]]);
        assert!(matches!(routes.shapes[0].as_ref().map(|s| &s.fig), Some(DrFig::Via { lonely: true, .. })), "the lonely via is flagged");
        let m: Vec<&Marker> = routes.markers().collect();
        assert_eq!(m.len(), 1);
        assert_eq!((m[0].rule, m[0].layer, m[0].bbox), (Rule::Lef58MaxSpacing, 1, Rect { xl: 430, yl: 430, xh: 470, yh: 470 }));
        assert_eq!(m[0].owners, [Owner::Net("n0".into())]);
        assert_eq!(m[0].victim, Some((Owner::Net("n0".into()), 1, m[0].bbox, false)));
        assert_eq!(m[0].victim, m[0].aggressor);
        assert!(st.stubborn.is_empty());
        let mut routes = DesignRoutes::with_initial(&t, vec![(0, via(450, true))]);
        let mut st = MaxSpacingState::default();
        let e = fix_max_spacing(&p, &mut routes, &mut st, &mut |_| {}, &mut |_, _, _, _| panic!("no worker before the re-run")).expect_err("pin access re-run not available");
        assert!(e.contains("updateDirtyPAData"), "{e}");
        assert_eq!(st.stubborn, BTreeSet::from([(4, 0)]));
        assert_eq!(st.dirty_insts, BTreeSet::from([4]));
        // A neighbour within 100: nothing lonely, nothing marked.
        let mut routes = DesignRoutes::with_initial(&t, vec![(0, via(450, true)), (1, via(540, false))]);
        let mut regions = 0;
        fix_max_spacing(&p, &mut routes, &mut MaxSpacingState::default(), &mut |_| {}, &mut |_, merged, _, _| {
            regions += merged.len();
            Ok(())
        })
        .expect("no lonely via");
        assert_eq!(routes.markers().count(), 0);
        assert_eq!(regions, 0);
        assert!(routes.shapes.iter().flatten().all(|s| !matches!(s.fig, DrFig::Via { lonely: true, .. })), "nothing flagged");
    }
}
