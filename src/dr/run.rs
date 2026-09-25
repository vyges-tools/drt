// SPDX-License-Identifier: Apache-2.0
//! Detailed routing end to end on a design database, in the router's call sequence: the design
//! and its access points ([`init_design`]), the route guides ([`init_guide`]), the rule tables
//! ([`prep`]), track assignment ([`ta`]), the search-and-repair iterations ([`dr`]) and the
//! write-out ([`end_fr`]). Each stage is its own function; [`detailed_route`] only calls them.
//!
//! What a stage does not model is refused (`Err`), never approximated.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

use vyges_opendb::Db;

use crate::dr::cost::{init_maze_cost, CostCtx, CostWorker, DrFig};
use crate::dr::db::TaDesign;
use crate::dr::design::{connectivity_check, DesignRoutes, WriteBack};
use crate::dr::drw::{gcell_boundary_pins, grid_maps, init_edges, init_nets_init_dr, init_nets_search_repair, localize_ext, merge_boundary_pins, mt_safe_dist, worker_batches, worker_groups, DrAp, DrNet, DrNetInput, DrPin, DrTerm, GridConfig, WorkerBoxes, BATCH_SIZE};
use crate::dr::flow::{guide_tile_boxes, in_check_box, strategy, tile_batches, worker_markers, written_back, ClipSize, Flow, FlowState, RipUp, MARKER_COST, ROUTE_SHAPE_COST};
use crate::dr::guides::{build_gcell_patterns, gen_guides, GCellGrid, GuideConfig, GuideNet, GuidePin};
use crate::dr::maze::{MazeCfg, MazeState};
use crate::dr::queue::{initial_marker_cost, recheck_markers, route_queue, Event, QueueCtx, Start};
use crate::dr::route::{abs_priority, init_queue, NetCtx};
use crate::dr::rules::{default_vias, rule_tables, EolTable, NdrRule, NdrTables, RuleConfig, RuleTables};
use crate::dr::ta::{track_assignment, Fixed, TaConfig, TaGuide, TaInput, TaState, TaTerm};
use crate::gc::{Marker, Owner};
use crate::pa::access::{AccessPoint, Config};
use crate::pa::flow::{pin_access_with, DesignInst, DesignPort, MasterClass, PinAccess};
use crate::polygon90::Rect;
use crate::rtree::PackedRTree;
use crate::tech::{read, LayerKind, Master, Tech, TrackPattern};

type Res<T> = Result<T, String>;
type P = (i32, i32);
/// A committed guide: net, begin, end.
type GuideSpan = (usize, P, P);
/// A net's committed guides (layer, begin, end) and gr pins (pin name, point).
type NetGuides = (Vec<(usize, P, P)>, Vec<(String, P)>);

/// The router's settings a caller may give; the rest are its defaults.
#[derive(Debug, Clone, Default)]
pub struct Options {
    /// The via-access layer (default: the second routing layer).
    pub via_access_layer: Option<usize>,
}

/// What routing did.
#[derive(Debug, Clone)]
pub struct Summary {
    /// Iterations run (the one that left no marker included).
    pub iterations: usize,
    /// Markers standing at the end (0 when routing is clean).
    pub markers: usize,
    /// Nets whose routing was written.
    pub nets_written: usize,
}

/// Route the design and write the routes into the database.
pub fn detailed_route(db: &mut Db, tech: &Tech, opts: &Options) -> Res<Summary> {
    let d = init_design(db, tech, opts)?;
    let g = init_guide(db, &d)?;
    let p = prep(&d);
    let t = ta(&d, &g, &p);
    let (routes, iterations) = dr(&d, &g, &p, &t)?;
    let markers = routes.markers().count();
    let nets_written = end_fr(db, &d, &g, &p, &routes)?;
    Ok(Summary { iterations, markers, nets_written })
}

/// The design as the router reads it: technology, tracks, instances, ports, nets and their
/// non-default rules, the fixed shapes, and the access points.
pub struct DesignIn {
    pub tech: Tech,
    pub tracks: Vec<TrackPattern>,
    pub cfg: Config,
    pub masters: HashMap<String, Master>,
    pub insts: Vec<DesignInst>,
    pub ports: Vec<DesignPort>,
    pub pa: PinAccess,
    pub design: TaDesign,
    pub ndrs: Vec<NdrRule>,
    /// Nets on a non-default rule with auto-taper turned off.
    pub no_taper: HashSet<usize>,
    pub bottom_layer: usize,
    pub die: Rect,
}

/// The reader, then pin access.
pub fn init_design(db: &Db, tech: &Tech, opts: &Options) -> Res<DesignIn> {
    let unmodelled = crate::dr::db::unmodelled_rules(db, tech);
    if !unmodelled.is_empty() {
        return Err(format!("rule families not modelled: {}", unmodelled.join("; ")));
    }
    let tracks = read::tracks(db, tech).map_err(|e| e.to_string())?;
    let mut cfg = crate::pa::db::config(db, tech);
    let (masters, insts, ports) = crate::pa::db::read_design(db, tech, cfg.top_routing_layer)?;
    if let Some(v) = opts.via_access_layer {
        cfg.via_access_layer = v;
    }
    let mut design = crate::dr::db::ta_design(db, tech, &masters, &insts, &ports)?;
    // Every design shape with its check owner: what a via trial of a pin on a non-default-rule
    // net without auto-taper is checked against.
    let shapes: Vec<crate::pa::verdict::TargetShape> = design.fixed.iter().zip(&design.owners).enumerate().flat_map(|(l, (v, o))| v.iter().zip(o).map(move |((r, _), o)| (o.clone(), l, *r))).collect();
    let pa = pin_access_with(tech, &tracks, &cfg, &masters, &insts, &ports, Some(&shapes)).map_err(|e| format!("pin access: {e:?}"))?;
    let min_level = db.block_get_min_routing_layer();
    let bottom_layer = (0..tech.layers.len()).find(|&l| min_level > 0 && tech.layers[l].kind == LayerKind::Routing && db.layer_get_routing_level(&tech.layers[l].name) == min_level).unwrap_or(2);
    // A net routed already makes the first iterations incremental (a rip-up mode not modelled).
    if let Some(n) = design.nets.iter().find(|n| db.net_has_wire(&n.name)) {
        return Err(format!("net {} is already routed: incremental routing not modelled", n.name));
    }
    let ndrs = read_ndrs(db, tech)?;
    for n in &mut design.nets {
        let r = db.net_get_non_default_rule(&n.name);
        if !r.is_empty() {
            n.ndr = Some(ndrs.iter().find(|x| x.name == r).cloned().ok_or_else(|| format!("net {}: rule {r} not read", n.name))?);
        }
    }
    // Non-default-rule nets without auto-taper: their routes are never tapered at the pins.
    let no_taper: HashSet<usize> = design.nets.iter().enumerate().filter(|(_, n)| n.ndr.is_some() && !db.net_is_auto_taper_enabled(&n.name)).map(|(i, _)| i).collect();
    let die = Rect { xl: db.block_get_die_area_x_min(), yl: db.block_get_die_area_y_min(), xh: db.block_get_die_area_x_max(), yh: db.block_get_die_area_y_max() };
    Ok(DesignIn { tech: tech.clone(), tracks, cfg, masters, insts, ports, pa, design, ndrs, no_taper, bottom_layer, die })
}

/// The non-default rules, the technology's then the block's (a name already read is skipped):
/// per routing layer its width and spacing (0 without a layer rule), its use-vias at their
/// bottom layer. Refused: a hard-spacing rule, a wire extension, use-via generate rules.
fn read_ndrs(db: &Db, tech: &Tech) -> Res<Vec<NdrRule>> {
    let n_routing = tech.layers.iter().filter(|l| l.kind == LayerKind::Routing).count();
    let z_of = |l: usize| (l / 2).saturating_sub(1);
    let mut out: Vec<NdrRule> = Vec::new();
    for name in db.tech_get_non_default_rules().into_iter().chain(db.block_get_non_default_rules()) {
        if out.iter().any(|r| r.name == name) {
            continue;
        }
        let e = |e: vyges_opendb::Error| e.to_string();
        if db.ndr_get_hard_spacing(&name) {
            return Err(format!("non-default rule {name}: hard spacing not modelled"));
        }
        if !db.ndr_use_via_rules(&name).map_err(e)?.is_empty() {
            return Err(format!("non-default rule {name}: via generate rules not modelled"));
        }
        if db.ndr_layer_rule_wire_exts(&name).map_err(e)?.iter().any(|&w| w != 0) {
            return Err(format!("non-default rule {name}: wire extension not modelled"));
        }
        let mut rule = NdrRule { name: name.clone(), widths: vec![0; n_routing], spacings: vec![0; n_routing], vias: vec![Vec::new(); n_routing] };
        for (layer, w, s) in db.ndr_layer_rules(&name).map_err(e)? {
            let l = tech.layer_num(&layer).ok_or_else(|| format!("non-default rule {name}: layer {layer}"))?;
            let z = z_of(l);
            if z < n_routing {
                rule.widths[z] = w;
                rule.spacings[z] = s;
            }
        }
        for v in db.ndr_use_vias(&name).map_err(e)? {
            let k = tech.via_defs.iter().position(|d| d.name == v).ok_or_else(|| format!("non-default rule {name}: via {v}"))?;
            let z = z_of(tech.via_defs[k].layer1);
            if z < n_routing {
                rule.vias[z].push(k);
            }
        }
        out.push(rule);
    }
    Ok(out)
}

/// The route guides processed: per net its guides and gr pins, the gcell grid.
pub struct GuidesIn {
    pub grid: GCellGrid,
    /// Per net name: its guides as read (layer, rectangle), in the database's order.
    pub raw: HashMap<String, Vec<(usize, Rect)>>,
    /// Per net name: its committed guides (layer, begin, end) and gr pins (pin name, point).
    pub by_net: HashMap<String, NetGuides>,
}

/// Guides read from the database (the layer check), the gcell grid (the database's single
/// pattern per axis, else built from the guides), each net's guides generated.
pub fn init_guide(db: &Db, d: &DesignIn) -> Res<GuidesIn> {
    let tech = &d.tech;
    let top = d.cfg.top_routing_layer;
    let grid_db = db.gcell_grid_pattern_x(0).is_ok() && db.gcell_grid_pattern_x(1).is_err() && db.gcell_grid_pattern_y(0).is_ok() && db.gcell_grid_pattern_y(1).is_err();
    let level = |l: i64| db.layer_get_routing_level(&db.layer_name_by_number(l));
    let top_level = db.layer_get_routing_level(&tech.layers[top].name);
    let above_top = |net: &str| -> Res<bool> {
        for t in db.net_bterms(net) {
            let mut bottom = i32::MAX;
            for p in 0..db.num_bterm_get_b_pins(&t) {
                for (l, ..) in db.bpin_layer_boxes(&t, p).map_err(|e| e.to_string())? {
                    bottom = bottom.min(level(l));
                }
            }
            if bottom != i32::MAX && bottom > top_level {
                return Ok(true);
            }
        }
        Ok(false)
    };
    // readGuides: nets in database order, each guide checked against the routing range.
    let mut raw: HashMap<String, Vec<(usize, Rect)>> = HashMap::new();
    let mut order: Vec<String> = Vec::new();
    for net in db.net_names() {
        let guides = db.net_guides(&net).map_err(|e| e.to_string())?;
        if guides.is_empty() {
            continue;
        }
        if db.net_is_special(&net) {
            return Err(format!("special net {net} has guides"));
        }
        for (layer, _, b, congested) in guides {
            if congested {
                return Err("input route guides are congested".into());
            }
            let l = tech.layer_num(&layer).ok_or_else(|| format!("guide layer {layer}"))?;
            if l > top {
                if above_top(&net)? {
                    continue;
                }
                return Err(format!("guide in net {net} on {layer}, above the top routing layer"));
            }
            let r = Rect { xl: b[0], yl: b[1], xh: b[2], yh: b[3] };
            if l < d.bottom_layer && grid_db {
                let (a, z) = ((r.xl + 1, r.yl + 1), (r.xh - 1, r.yh - 1));
                let one = db.gcell_x_idx(a.0).ok() == db.gcell_x_idx(z.0).ok() && db.gcell_y_idx(a.1).ok() == db.gcell_y_idx(z.1).ok();
                if !one {
                    return Err(format!("guide in net {net} on {layer}, below the bottom routing layer"));
                }
            }
            if !raw.contains_key(&net) {
                order.push(net.clone());
            }
            raw.entry(net.clone()).or_default().push((l, r));
        }
    }
    let grid = if grid_db {
        GCellGrid { x: db.gcell_grid_pattern_x(0).map_err(|e| e.to_string())?, y: db.gcell_grid_pattern_y(0).map_err(|e| e.to_string())?, die: d.die }
    } else {
        let all: Vec<(usize, Rect)> = order.iter().flat_map(|n| raw[n].iter().copied()).collect();
        build_gcell_patterns(tech, d.die, &all).ok_or("no gcell grid from the guides")?
    };
    let gcfg = GuideConfig { bottom_routing_layer: d.bottom_layer, top_routing_layer: top, via_access_layer: d.cfg.via_access_layer, allow_pin_feedthrough: true };
    let inst_index: HashMap<&str, usize> = d.insts.iter().enumerate().map(|(i, x)| (x.name.as_str(), i)).collect();
    let port_index: HashMap<&str, usize> = d.ports.iter().enumerate().map(|(k, p)| (p.name.as_str(), k)).collect();
    let class_of = classes_of(d);
    let mut by_net = HashMap::new();
    for name in &order {
        let mut pins: Vec<GuidePin> = Vec::new();
        for it in db.net_iterms(name) {
            let (iname, tname) = it.rsplit_once('/').ok_or("inst/term")?;
            let i = *inst_index.get(iname).ok_or_else(|| format!("instance {iname}"))?;
            let c = class_of[i];
            let m = &d.masters[&d.insts[i].unique.master];
            let t = m.terms.iter().position(|x| x.name == tname).ok_or_else(|| format!("terminal {it}"))?;
            let rep = &d.insts[d.pa.classes[c].insts[0]];
            let loc = d.insts[i].unique.location;
            let aps = d.pa.class_aps[c][t].iter().flatten().map(|a| ((a.point.0 - rep.unique.location.0 + loc.0, a.point.1 - rep.unique.location.1 + loc.1), a.layer)).collect();
            pins.push(GuidePin { name: it.clone(), is_port: false, id: i, shapes: Vec::new(), aps });
        }
        for bt in db.net_bterms(name) {
            let k = port_index[bt.as_str()];
            let aps = d.pa.port_aps[k].iter().flatten().map(|a| (a.point, a.layer)).collect();
            pins.push(GuidePin { name: format!("PIN/{bt}"), is_port: true, id: k, shapes: Vec::new(), aps });
        }
        let net = GuideNet { tech, grid: &grid, cfg: &gcfg, pins };
        let (guides, gr) = gen_guides(&net, &raw[name], &mut None).ok_or_else(|| format!("net {name}: guides do not connect its pins"))?;
        let gs = guides.iter().map(|g| (g.layer, g.begin, g.end)).collect();
        let gps = gr.iter().map(|&(p, pt)| (net.pins[p].name.clone(), pt)).collect();
        by_net.insert(name.clone(), (gs, gps));
    }
    Ok(GuidesIn { grid, raw, by_net })
}

/// Per instance, its unique class (`usize::MAX` for none).
fn classes_of(d: &DesignIn) -> Vec<usize> {
    let mut class_of = vec![usize::MAX; d.insts.len()];
    for (c, cl) in d.pa.classes.iter().enumerate() {
        for &i in &cl.insts {
            class_of[i] = c;
        }
    }
    class_of
}

/// The rule tables, on the technology with its default and generated vias.
pub struct Prep {
    pub tech: Tech,
    pub defaults: Vec<Option<usize>>,
    pub rules: RuleTables,
}

pub fn prep(d: &DesignIn) -> Prep {
    let rcfg = RuleConfig { bottom_routing_layer: d.bottom_layer, top_routing_layer: d.cfg.top_routing_layer, enable_via_gen: true };
    let mut tech = d.tech.clone();
    let defaults = default_vias(&mut tech, &rcfg);
    let rules = rule_tables(&tech, &defaults, &d.ndrs, &rcfg);
    Prep { tech, defaults, rules }
}

/// Track assignment's result and the terminal tables detailed routing reads.
pub struct TaOut {
    pub guides: Vec<TaGuide>,
    pub gr_pins: Vec<(usize, (i32, i32))>,
    pub dr_terms: Vec<DrTerm>,
    pub macro_term: Vec<bool>,
    pub term_fixed: Vec<Fixed>,
}

pub fn ta(d: &DesignIn, g: &GuidesIn, p: &Prep) -> TaOut {
    let (masters, insts, ports, pa, design) = (&d.masters, &d.insts, &d.ports, &d.pa, &d.design);
    let inst_index: HashMap<&str, usize> = insts.iter().enumerate().map(|(i, x)| (x.name.as_str(), i)).collect();
    let port_index: HashMap<&str, usize> = ports.iter().enumerate().map(|(k, p)| (p.name.as_str(), k)).collect();
    let mut term_base = vec![0usize; insts.len() + 1];
    for (i, x) in insts.iter().enumerate() {
        term_base[i + 1] = term_base[i] + masters[&x.unique.master].terms.len();
    }
    let mut ta_terms: Vec<TaTerm> = Vec::new();
    let mut dr_terms: Vec<DrTerm> = Vec::new();
    let mut macro_term: Vec<bool> = Vec::new();
    let mut term_fixed: Vec<Fixed> = Vec::new();
    let mut term_of: HashMap<String, usize> = HashMap::new();
    let mut guides = Vec::new();
    let mut gr_pins = Vec::new();
    let ap_of = |a: &AccessPoint, shift: (i32, i32)| DrAp { point: (a.point.0 + shift.0, a.point.1 + shift.1), layer: a.layer, access: a.db_access_bits(), vias: a.vias.clone() };
    for (ni, net) in design.nets.iter().enumerate() {
        let Some((gs, gps)) = g.by_net.get(&net.name) else { continue };
        for &(layer, b, e) in gs {
            guides.push(TaGuide { net: ni, layer, begin: b, end: e, route: None });
        }
        for (name, pt) in gps {
            let t = *term_of.entry(name.clone()).or_insert_with(|| {
                if let Some(pn) = name.strip_prefix("PIN/") {
                    let k = port_index[pn];
                    let net = match &ports[k].owner {
                        Owner::Net(n) => design.net_index.get(n).copied(),
                        _ => None,
                    };
                    ta_terms.push(TaTerm::Port { net, pins: pa.port_aps[k].iter().map(|aps| (!aps.is_empty(), aps.iter().map(|a| (a.point, a.layer)).collect())).collect() });
                    let bbox = ports[k].pins.iter().flatten().map(|&(_, r)| r).reduce(|a, r| Rect::new(a.xl.min(r.xl), a.yl.min(r.yl), a.xh.max(r.xh), a.yh.max(r.yh))).expect("a pin box");
                    macro_term.push(false);
                    term_fixed.push(Fixed::BTerm { net, port: k });
                    dr_terms.push(DrTerm { name: name.clone(), is_port: true, order: k, net, bbox, pins: pa.port_aps[k].iter().map(|aps| (!aps.is_empty(), aps.iter().map(|a| ap_of(a, (0, 0))).collect(), None)).collect() });
                } else {
                    let (iname, tname) = name.rsplit_once('/').expect("inst/term");
                    let i = inst_index[iname];
                    let m = &masters[&insts[i].unique.master];
                    let t = m.terms.iter().position(|x| x.name == tname).expect("term");
                    let prefs = pa.pref_access_points(insts, masters, i, t);
                    let same_master: Vec<usize> = (0..pa.classes.len()).filter(|&c| insts[pa.classes[c].insts[0]].unique.master == insts[i].unique.master).collect();
                    let c0 = same_master[0];
                    let rep0 = &insts[pa.classes[c0].insts[0]];
                    let loc = insts[i].unique.location;
                    let c = (0..pa.classes.len()).find(|&c| pa.classes[c].insts.contains(&i)).expect("a class");
                    let rep = &insts[pa.classes[c].insts[0]];
                    let shift = (loc.0 - rep.unique.location.0, loc.1 - rep.unique.location.1);
                    let mut ta_pins = Vec::new();
                    let mut dr_pins = Vec::new();
                    for pi in 0..m.terms[t].pins.len() {
                        let has = same_master.iter().any(|&c| pa.class_aps[c].get(t).and_then(|x| x.get(pi)).is_some_and(|a| !a.is_empty()));
                        let first = pa.class_aps[c0].get(t).and_then(|x| x.get(pi)).and_then(|a| a.first()).map(|a| ((a.point.0 - rep0.unique.location.0 + loc.0, a.point.1 - rep0.unique.location.1 + loc.1), a.layer));
                        let pref = prefs.get(pi).copied().flatten().map(|(q, l)| ((q.0 + loc.0, q.1 + loc.1), l));
                        ta_pins.push((has, pref, first));
                        let aps: Vec<DrAp> = pa.class_aps[c].get(t).and_then(|x| x.get(pi)).map_or(Vec::new(), |v| v.iter().map(|a| ap_of(a, shift)).collect());
                        let pref_idx = pref.and_then(|(q, l)| aps.iter().position(|a| a.point == q && a.layer == l));
                        dr_pins.push((has, aps, pref_idx));
                    }
                    let net = insts[i].nets[t].as_ref().and_then(|n| design.net_index.get(n)).copied();
                    ta_terms.push(TaTerm::Inst { net, pins: ta_pins });
                    let xf = &insts[i].transform;
                    let bbox = m.terms[t].pins.iter().flat_map(|p| p.shapes.iter()).map(|&(_, r)| xf.apply(r)).reduce(|a, r| Rect::new(a.xl.min(r.xl), a.yl.min(r.yl), a.xh.max(r.xh), a.yh.max(r.yh))).expect("a pin box");
                    dr_terms.push(DrTerm { name: name.clone(), is_port: false, order: term_base[i] + t, net, bbox, pins: dr_pins });
                    macro_term.push(matches!(insts[i].class, MasterClass::Macro));
                    term_fixed.push(Fixed::InstTerm { net, inst: i, term: t });
                }
                ta_terms.len() - 1
            });
            gr_pins.push((t, *pt));
        }
    }
    let input = TaInput {
        tech: &p.tech,
        defaults: &p.defaults,
        eol: &p.rules.eol,
        tables: &p.rules.layers,
        grid: &g.grid,
        die: d.die,
        tracks: &d.tracks,
        nets: &design.nets,
        fixed: &design.fixed,
        terms: &ta_terms,
        gr_pins: &gr_pins,
        cfg: TaConfig { bottom_routing_layer: d.bottom_layer, top_routing_layer: d.cfg.top_routing_layer, ..TaConfig::default() },
    };
    let mut st = TaState::new(input, guides);
    track_assignment(&mut st, &mut None);
    TaOut { guides: st.guides, gr_pins, dr_terms, macro_term, term_fixed }
}

/// What every worker reads of the design (built once).
struct DrCtx<'a> {
    d: &'a DesignIn,
    g: &'a GuidesIn,
    p: &'a Prep,
    t: &'a TaOut,
    guide_trees: Vec<PackedRTree<GuideSpan>>,
    gr_tree: PackedRTree<usize>,
    bpins: Vec<Vec<crate::dr::drw::BoundaryPins>>,
    min_area: Vec<i64>,
    through: Vec<[bool; 4]>,
    term_aps: HashSet<((i32, i32), usize, usize)>,
    fixed_trees: Vec<PackedRTree<Fixed>>,
    index_trees: Vec<PackedRTree<usize>>,
    inst_index: HashMap<&'a str, usize>,
    term_base: Vec<usize>,
}

/// The main loop: the strategy's rows in order until no marker stands, each row one
/// search-and-repair iteration and the connectivity check after it. The routes and the
/// iterations run.
pub fn dr(d: &DesignIn, g: &GuidesIn, p: &Prep, t: &TaOut) -> Res<(DesignRoutes, usize)> {
    let cx = dr_init(d, g, p, t);
    let mut routes = DesignRoutes::default();
    let mut flow = FlowState::default();
    let mut clip = ClipSize::default();
    let mut iterations = 0;
    for (iter, row) in strategy(ROUTE_SHAPE_COST, MARKER_COST).into_iter().enumerate() {
        if iter > crate::dr::flow::END_ITERATION {
            break;
        }
        // From iteration 7 a congested worker widens the clip of later rows: not modelled.
        if iter >= 7 {
            return Err(format!("iteration {iter} reached with markers standing: congestion-driven clip growth not modelled"));
        }
        let mut args = row;
        args.size = clip.size(&row, false);
        search_repair(&cx, &mut routes, &mut flow, iter, &args)?;
        iterations = iter + 1;
        if routes.markers().next().is_none() {
            break;
        }
    }
    Ok((routes, iterations))
}

/// Routing's set-up: what the workers share.
fn dr_init<'a>(d: &'a DesignIn, g: &'a GuidesIn, p: &'a Prep, t: &'a TaOut) -> DrCtx<'a> {
    let wires: Vec<(usize, usize, P, P)> = t.guides.iter().filter_map(|x| x.route.map(|(b, e)| (x.net, x.layer, b, e))).collect();
    let bpins = gcell_boundary_pins(&g.grid, &wires);
    let mut per_layer: Vec<Vec<(Rect, GuideSpan)>> = vec![Vec::new(); p.tech.layers.len()];
    for x in &t.guides {
        let b = Rect { xl: x.begin.0.min(x.end.0), yl: x.begin.1.min(x.end.1), xh: x.begin.0.max(x.end.0), yh: x.begin.1.max(x.end.1) };
        per_layer[x.layer].push((b, (x.net, x.begin, x.end)));
    }
    let guide_trees = per_layer.into_iter().map(PackedRTree::new).collect();
    let gr_tree = PackedRTree::new(t.gr_pins.iter().map(|&(k, pt)| (Rect { xl: pt.0, yl: pt.1, xh: pt.0, yh: pt.1 }, k)).collect());
    let min_area = p.tech.layers.iter().map(|l| l.min_area).collect();
    let through = p.rules.layers.iter().map(|l| l.through).collect();
    // Every connected terminal's access points: (point, layer, net).
    let class_of = classes_of(d);
    let mut term_aps = HashSet::new();
    for (i, inst) in d.insts.iter().enumerate() {
        let c = class_of[i];
        if c == usize::MAX {
            continue;
        }
        let rep = &d.insts[d.pa.classes[c].insts[0]];
        let shift = (inst.unique.location.0 - rep.unique.location.0, inst.unique.location.1 - rep.unique.location.1);
        for (k, net) in inst.nets.iter().enumerate() {
            let Some(n) = net.as_ref().and_then(|n| d.design.net_index.get(n)).copied() else { continue };
            for aps in d.pa.class_aps[c].get(k).into_iter().flatten() {
                for a in aps {
                    term_aps.insert(((a.point.0 + shift.0, a.point.1 + shift.1), a.layer, n));
                }
            }
        }
    }
    for (k, port) in d.ports.iter().enumerate() {
        let Owner::Net(name) = &port.owner else { continue };
        let Some(&n) = d.design.net_index.get(name) else { continue };
        for a in d.pa.port_aps[k].iter().flatten() {
            term_aps.insert((a.point, a.layer, n));
        }
    }
    let fixed_trees = d.design.fixed.iter().map(|v| PackedRTree::new(v.clone())).collect();
    let index_trees = d.design.fixed.iter().map(|v| PackedRTree::new(v.iter().enumerate().map(|(k, (b, _))| (*b, k)).collect())).collect();
    let inst_index = d.insts.iter().enumerate().map(|(i, x)| (x.name.as_str(), i)).collect();
    let mut term_base = vec![0usize; d.insts.len() + 1];
    for (i, x) in d.insts.iter().enumerate() {
        term_base[i + 1] = term_base[i] + d.masters[&x.unique.master].terms.len();
    }
    DrCtx { d, g, p, t, guide_trees, gr_tree, bpins, min_area, through, term_aps, fixed_trees, index_trees, inst_index, term_base }
}

impl DrCtx<'_> {
    /// A terminal's shapes (layer, rectangle), design coordinates.
    fn term_shapes(&self, f: &Fixed) -> Vec<(usize, Rect)> {
        match *f {
            Fixed::InstTerm { inst, term, .. } => {
                let m = &self.d.masters[&self.d.insts[inst].unique.master];
                m.terms[term].pins.iter().flat_map(|p| p.shapes.iter()).map(|&(l, r)| (l, self.d.insts[inst].transform.apply(r))).collect()
            }
            Fixed::BTerm { port, .. } => self.d.ports[port].pins.iter().flatten().copied().collect(),
            _ => Vec::new(),
        }
    }
}

/// One search-and-repair iteration: the flow for this iteration, its workers in batches, each batch
/// routed then written back worker by worker; the connectivity check after.
fn search_repair(cx: &DrCtx<'_>, routes: &mut DesignRoutes, flow_state: &mut FlowState, iter: usize, args: &crate::dr::flow::IterArgs) -> Res<()> {
    let flow = flow_state.next(routes.markers().count(), args, false);
    if matches!(args.ripup, RipUp::Drc | RipUp::NearDrc) && routes.markers().next().is_none() {
        return Ok(());
    }
    let ripup_all = match args.ripup {
        RipUp::All => true,
        RipUp::Drc => false,
        other => return Err(format!("rip-up mode {other:?} not modelled")),
    };
    let tech = &cx.p.tech;
    let mt = mt_safe_dist(&cx.d.ndrs.iter().collect::<Vec<_>>());
    let batches: Vec<Vec<WorkerBoxes>> = match flow {
        Flow::Optimization => worker_batches(worker_groups(&cx.g.grid, args.size, args.offset, mt, 500), BATCH_SIZE),
        Flow::Guides => {
            let markers: Vec<Marker> = routes.markers().cloned().collect();
            let net_of: HashMap<&str, usize> = cx.d.design.nets.iter().enumerate().map(|(i, n)| (n.name.as_str(), i)).collect();
            let guides = |o: &Owner| -> Option<Vec<Rect>> {
                match o {
                    Owner::Net(n) => Some(cx.g.raw.get(n).map_or(Vec::new(), |v| v.iter().map(|&(_, r)| r).collect())),
                    _ => None,
                }
            };
            let guided = |o: &Owner| -> Option<usize> {
                match o {
                    Owner::Net(n) if cx.g.raw.get(n).is_some_and(|v| !v.is_empty()) => net_of.get(n.as_str()).copied(),
                    _ => None,
                }
            };
            let boxes = guide_tile_boxes(&markers, &guides, &|m| crate::dr::design::off_guide_boxes(routes, tech, &cx.g.grid, m, &guided));
            let bloat = |r: &Rect, k: i32| Rect { xl: r.xl - k, yl: r.yl - k, xh: r.xh + k, yh: r.yh + k };
            tile_batches(&boxes, mt).into_iter().map(|b| b.into_iter().map(|i| WorkerBoxes { start: (0, 0), route: boxes[i], ext: bloat(&boxes[i], mt), drc: bloat(&boxes[i], 500) }).collect()).collect()
        }
        Flow::Stubborn => {
            stubborn_tiles_flow(cx, routes, flow_state, iter, args, ripup_all, mt)?;
            return check(cx, routes);
        }
        Flow::Skip => Vec::new(),
    };
    let mut changed = false;
    for group in batches {
        let mut results = Vec::new();
        for w in group {
            results.push(route_worker(cx, routes, iter, args, ripup_all, &w)?);
        }
        // The batch written back, worker by worker.
        for (w, routed, markers, wm) in &results {
            let best = in_check_box(markers, &w.drc).len();
            if !written_back(iter, args.ripup, wm, best) {
                continue;
            }
            changed |= best != wm.init_num;
            end_worker(cx, routes, w, routed, markers, iter);
        }
    }
    if flow == Flow::Guides {
        flow_state.last_effective = changed;
    }
    check(cx, routes)
}

/// The stubborn-tiles flow: route boxes grown around the markers, and per box nine workers (the
/// DRC and marker costs each as given, halved and doubled), all of a batch on the same design; per
/// worker id the first with the fewest markers is written back, in id order.
fn stubborn_tiles_flow(cx: &DrCtx<'_>, routes: &mut DesignRoutes, flow_state: &mut FlowState, iter: usize, args: &crate::dr::flow::IterArgs, ripup_all: bool, mt: i32) -> Res<()> {
    let markers: Vec<Marker> = routes.markers().cloned().collect();
    let (boxes, batches) = crate::dr::flow::stubborn_boxes(&markers, &cx.g.grid, mt);
    let bloat = |r: &Rect, k: i32| Rect { xl: r.xl - k, yl: r.yl - k, xh: r.xh + k, yh: r.yh + k };
    let drc_costs = [args.drc_cost, args.drc_cost / 2, args.drc_cost * 2];
    let marker_costs = [args.marker_cost, args.marker_cost / 2, args.marker_cost * 2];
    let mut changed = false;
    for batch in batches {
        let mut results: Vec<(usize, WorkerResult)> = Vec::new();
        for &id in &batch {
            for route in &boxes[id] {
                for &drc_cost in &drc_costs {
                    for &marker_cost in &marker_costs {
                        let wargs = crate::dr::flow::IterArgs { drc_cost, marker_cost, ..*args };
                        let w = WorkerBoxes { start: (0, 0), route: *route, ext: bloat(route, mt), drc: bloat(route, 500) };
                        results.push((id, route_worker(cx, routes, iter, &wargs, ripup_all, &w)?));
                    }
                }
            }
        }
        // Per worker id, the first with the fewest markers in its check box.
        let mut best: BTreeMap<usize, (usize, usize)> = BTreeMap::new();
        for (k, (id, (w, _, markers, _))) in results.iter().enumerate() {
            let n = in_check_box(markers, &w.drc).len();
            if best.get(id).is_none_or(|&(_, b)| n < b) {
                best.insert(*id, (k, n));
            }
        }
        for (_, (k, n)) in best {
            let (_, (w, routed, markers, wm)) = &results[k];
            if !written_back(iter, args.ripup, wm, n) {
                continue;
            }
            changed |= n != wm.init_num;
            end_worker(cx, routes, w, routed, markers, iter);
        }
    }
    flow_state.last_effective = changed;
    Ok(())
}

type WorkerResult = (WorkerBoxes, Vec<(usize, Vec<DrFig>)>, Vec<Marker>, crate::dr::flow::WorkerMarkers);

/// One worker: its nets, grid, costs and queue. Its routed nets' shapes, its final markers.
fn route_worker(cx: &DrCtx<'_>, routes: &DesignRoutes, iter: usize, args: &crate::dr::flow::IterArgs, ripup_all: bool, w: &WorkerBoxes) -> Res<WorkerResult> {
    let (d, p, t) = (cx.d, cx.p, cx.t);
    let tech = &p.tech;
    let term_shape_at = |k: usize, pt: (i32, i32), l: usize| cx.term_shapes(&t.term_fixed[k]).iter().any(|&(tl, r)| tl == l && r.xl <= pt.0 && pt.0 <= r.xh && r.yl <= pt.1 && pt.1 <= r.yh);
    let term_at = |pt: (i32, i32), l: usize, net: usize| -> Vec<usize> { (0..t.dr_terms.len()).filter(|&k| t.dr_terms[k].net == Some(net) && term_shape_at(k, pt, l)).collect() };
    let dinp = DrNetInput { grid: &cx.g.grid, guides: &cx.guide_trees, gr_pins: &cx.gr_tree, terms: &t.dr_terms, min_area: &cx.min_area, routes: Some((tech, routes)), term_at: Some(&term_at) };
    let wm = worker_markers(routes.markers_in(&w.drc), iter);
    let built = if wm.skipped(iter) {
        Vec::new()
    } else if iter == 0 {
        let bp = merge_boundary_pins(&cx.bpins, w.start, args.size, &w.route);
        init_nets_init_dr(&dinp, &w.route, &w.ext, &bp)?
    } else {
        init_nets_search_repair(&dinp, &w.route, &w.ext, !ripup_all)?
    };
    let mut nets = built;
    if nets.is_empty() {
        return Ok((*w, Vec::new(), Vec::new(), wm));
    }
    let gcfg = GridConfig { bottom_routing_layer: d.bottom_layer, top_routing_layer: d.cfg.top_routing_layer };
    let (xm, ym, zs) = grid_maps(tech, &d.tracks, &gcfg, &w.route, &w.ext, &nets);
    let mut g = init_edges(tech, &p.defaults, &gcfg, &xm, &ym, &zs, &w.route, &d.die);
    localize_ext(tech, &g, &w.ext, &mut nets);
    let rule_of = |n: &DrNet| -> Option<(&NdrRule, &[Option<EolTable>])> {
        let r = d.design.nets[n.net].ndr.as_ref()?;
        let rule = d.ndrs.iter().find(|x| x.name == r.name)?;
        let eol = p.rules.ndrs.iter().find(|x| x.name == r.name).map(|x| x.eol.as_slice()).unwrap_or(&[]);
        Some((rule, eol))
    };
    let ndr_of: Vec<(&NdrRule, Vec<Option<EolTable>>)> = nets
        .iter()
        .filter_map(|n| d.design.nets[n.net].ndr.as_ref())
        .map(|r| (d.ndrs.iter().find(|x| x.name == r.name).expect("a rule"), p.rules.ndrs.iter().find(|x| x.name == r.name).map(|x| x.eol.clone()).unwrap_or_default()))
        .collect();
    let term_shapes = |f: &Fixed| cx.term_shapes(f);
    let port_aps = |k: usize| -> Vec<DrAp> { d.pa.port_aps[k].iter().flatten().map(|a| DrAp { point: a.point, layer: a.layer, access: a.db_access_bits(), vias: a.vias.clone() }).collect() };
    let inst_is_block = |i: usize| d.insts[i].is_block;
    let ccx = CostCtx {
        tech,
        defaults: &p.defaults,
        eol: &p.rules.eol,
        ndrs: ndr_of,
        use_min_spacing_obs: true,
        through: &cx.through,
        via_access_layer: d.cfg.via_access_layer,
        fixed: &cx.fixed_trees,
        term_shapes: &term_shapes,
        port_aps: &port_aps,
        inst_is_block: &inst_is_block,
    };
    let mut cw = CostWorker { cx: &ccx, g: &mut g, ap_svia: Default::default() };
    init_maze_cost(&mut cw, &nets, &w.ext, &rule_of).map_err(|e| e.0)?;
    let (routed, markers) = run_queue(cx, &mut cw, &nets, &wm, iter, args, ripup_all, w);
    Ok((*w, routed, markers, wm))
}

/// The worker's search-and-repair queue: the nets it rerouted (every worker net of a net any of
/// whose worker nets rerouted, in worker order: a rerouted one's last shapes, another's the
/// routes it started from) and its final markers.
#[allow(clippy::too_many_arguments)]
fn run_queue(cx: &DrCtx<'_>, cw: &mut CostWorker<'_, '_>, nets: &[DrNet], wm: &crate::dr::flow::WorkerMarkers, iter: usize, args: &crate::dr::flow::IterArgs, ripup_all: bool, w: &WorkerBoxes) -> (Vec<(usize, Vec<DrFig>)>, Vec<Marker>) {
    let (d, p, t) = (cx.d, cx.p, cx.t);
    let tech = &p.tech;
    let (r, e) = (w.route, w.ext);
    let st_nets = &d.design.nets;
    let name_of = |i: usize| st_nets[nets[i].net].name.clone();
    // Per net: its guides in the route box (none when not following guides), its rule.
    let guides: Vec<Vec<(usize, Rect)>> = nets.iter().map(|n| if !args.follow_guide { Vec::new() } else { cx.g.raw.get(&st_nets[n.net].name).map_or(Vec::new(), |v| v.iter().filter(|(_, b)| b.xh >= r.xl && b.xl <= r.xh && b.yh >= r.yl && b.yl <= r.yh).copied().collect()) }).collect();
    let ndr_rule: Vec<Option<&NdrRule>> = nets.iter().map(|n| st_nets[n.net].ndr.as_ref().and_then(|rr| d.ndrs.iter().find(|x| x.name == rr.name))).collect();
    let ndr_t: Vec<Option<(&NdrTables, &[i32])>> = nets
        .iter()
        .zip(&ndr_rule)
        .map(|(n, rule)| {
            let rr = st_nets[n.net].ndr.as_ref()?;
            let tables = p.rules.ndrs.iter().find(|x| x.name == rr.name)?;
            Some((tables, (*rule)?.widths.as_slice()))
        })
        .collect();
    let ndr_eol: Vec<Option<crate::dr::cost::Ndr<'_>>> = nets
        .iter()
        .zip(&ndr_rule)
        .map(|(n, rule)| {
            let rr = st_nets[n.net].ndr.as_ref()?;
            let eol = p.rules.ndrs.iter().find(|x| x.name == rr.name).map(|x| x.eol.as_slice()).unwrap_or(&[]);
            Some((rule.as_ref().copied()?, eol))
        })
        .collect();
    type ApTest<'x> = Box<dyn Fn(P, usize) -> bool + 'x>;
    let haps: Vec<ApTest<'_>> = nets
        .iter()
        .map(|n| {
            let id = n.net;
            Box::new(move |pt: (i32, i32), l: usize| cx.term_aps.contains(&(pt, l, id))) as Box<dyn Fn((i32, i32), usize) -> bool>
        })
        .collect();
    let pin_name = |pn: &DrPin| pn.term.map_or(String::new(), |k| t.dr_terms[k].name.strip_prefix("PIN/").unwrap_or(&t.dr_terms[k].name).to_string());
    let tf = |k: usize| t.term_fixed[k];
    let mt = |k: usize| t.macro_term[k];
    let ipt = |k: usize| t.dr_terms[k].is_port;
    let net_ctx = |i: usize| NetCtx { ext_box: &e, guides: &guides[i], term_fixed: &tf, is_macro_term: &mt, pin_name: &pin_name, ndr: ndr_t[i], auto_taper: !d.no_taper.contains(&nets[i].net), is_port_term: &ipt, has_access_point: haps[i].as_ref(), ndr_rule: ndr_rule[i], route_box: r, ndr_cost: ndr_eol[i] };
    let ndr = |i: usize| ndr_eol[i];
    let max_avoids = |i: usize| if st_nets[nets[i].net].is_clock { 100 } else if st_nets[nets[i].net].ndr.is_some() { 3 } else { 0 };
    let is_supply = |n: &str| !d.design.net_index.contains_key(n);
    let inst_idx = |n: &str| cx.inst_index.get(n).copied().unwrap_or(0);
    // The design's shapes in the extended box, layer by layer in the region query's order.
    let mut fixed: Vec<(Owner, usize, Rect)> = Vec::new();
    for (l, tree) in cx.index_trees.iter().enumerate() {
        for v in tree.query(&e) {
            fixed.push((d.design.owners[l][v.1].clone(), l, v.0));
        }
    }
    let nz = tech.layers.iter().filter(|l| l.kind == LayerKind::Routing).count();
    let max_ndr: Vec<i32> = (0..nz).map(|z| d.ndrs.iter().map(|n| n.spacings.get(z).copied().unwrap_or(0)).max().unwrap_or(0)).collect();
    let ndr_spacing = |i: usize| ndr_rule[i].map(|rr| rr.spacings.clone());
    let mcfg = MazeCfg { tech, rules: &p.rules, drc_cost: args.drc_cost, marker_cost: args.marker_cost, fixed_cost: args.fixed_cost, iter: iter as i32, bottom_routing_layer: d.bottom_layer, ripup_all };
    let q = QueueCtx {
        mcfg: &mcfg,
        route_box: r,
        name: &name_of,
        net_ctx: &net_ctx,
        ndr: &ndr,
        max_ripup_avoids: &max_avoids,
        is_supply: &is_supply,
        inst_index: &inst_idx,
        fixed: &fixed,
        ndr_spacing: &ndr_spacing,
        max_ndr_spacing: &max_ndr,
        marker_decay: args.decay,
        maze_end_iter: args.maze_end,
        markers_drive: !ripup_all,
    };
    // The design's markers in the check box (or, with a re-check marker there, a check's; copies
    // too) cost first, then the via reservations.
    let touches_drc = |m: &Marker| m.bbox.xh >= w.drc.xl && m.bbox.xl <= w.drc.xh && m.bbox.yh >= w.drc.yl && m.bbox.yl <= w.drc.yh;
    let worker_markers: Vec<Marker> = if wm.need_recheck { recheck_markers(&q, nets).into_iter().filter(touches_drc).map(|m| m.copied()).collect() } else { wm.markers.clone() };
    let hist = initial_marker_cost(cw, &q, nets, &worker_markers);
    let abs = |n: &DrNet| abs_priority(st_nets[n.net].is_clock, st_nets[n.net].ndr.is_some());
    let net_name = |n: &DrNet| st_nets[n.net].name.as_str();
    let ndr_of = |n: &DrNet| -> Option<crate::dr::cost::Ndr<'_>> { nets.iter().position(|x| std::ptr::eq(x, n)).and_then(|i| ndr_eol[i]) };
    let order = if ripup_all { init_queue(cw, nets, &abs, &net_name, &ndr_of, &|k| t.macro_term[k]) } else { Vec::new() };
    let mut mst = MazeState::new(tech, cw.g, d.die);
    if !args.follow_guide {
        mst.all_guided();
    }
    let mut last: BTreeMap<usize, Vec<DrFig>> = BTreeMap::new();
    let mut final_markers = Vec::new();
    let start = if ripup_all { Start::Nets(&order) } else { Start::Markers(&worker_markers) };
    for ev in route_queue(cw, &mut mst, &q, nets, start, hist) {
        match ev {
            Event::Route { net, figs, .. } => {
                last.insert(net, figs);
            }
            Event::Final { markers } => final_markers = markers,
            _ => {}
        }
    }
    let modified: BTreeSet<usize> = last.keys().map(|&i| nets[i].net).collect();
    let mut routed = Vec::new();
    for (i, n) in nets.iter().enumerate() {
        if modified.contains(&n.net) {
            routed.push((n.net, last.get(&i).cloned().unwrap_or_else(|| n.route.clone())));
        }
    }
    (routed, final_markers)
}

/// A worker's end: its routes and markers into the design.
fn end_worker(cx: &DrCtx<'_>, routes: &mut DesignRoutes, w: &WorkerBoxes, routed: &[(usize, Vec<DrFig>)], markers: &[Marker], iter: usize) {
    let tech = &cx.p.tech;
    let on_pin = |pt: (i32, i32), l: usize, net: usize| cx.fixed_trees.get(l).is_some_and(|tr| tr.query(&Rect { xl: pt.0, yl: pt.1, xh: pt.0, yh: pt.1 }).iter().any(|v| v.1.term_net() == Some(Some(net))));
    crate::dr::design::end(routes, tech, &WriteBack { route_box: w.route, ext_box: w.ext, drc_box: w.drc, routed, markers, on_pin: &on_pin, manufacturing_grid: tech.manufacturing_grid, init_dr: iter == 0 });
}

/// The connectivity check between iterations: over the modified nets, applied to the design.
fn check(cx: &DrCtx<'_>, routes: &mut DesignRoutes) -> Res<()> {
    let tech = &cx.p.tech;
    let term_key = |net: usize, pt: (i32, i32), l: usize| -> Vec<(u8, usize)> {
        let q = Rect { xl: pt.0, yl: pt.1, xh: pt.0, yh: pt.1 };
        cx.fixed_trees.get(l).map_or(Vec::new(), |tr| {
            tr.query(&q)
                .iter()
                .filter_map(|v| match v.1 {
                    Fixed::BTerm { net: Some(n), port } if n == net => Some((0, port)),
                    Fixed::InstTerm { net: Some(n), inst, term } if n == net => Some((1, cx.term_base[inst] + term)),
                    _ => None,
                })
                .collect()
        })
    };
    let name = |n: usize| cx.d.design.nets[n].name.clone();
    let mut check = |n: usize, shapes: &mut crate::dr::conn::NetShapes| crate::dr::conn::check_net(tech, shapes, &|pt, l| term_key(n, pt, l));
    connectivity_check(routes, tech, &name, &mut check).map_err(|e| format!("connectivity check: {e}"))
}

/// The write-out: the routes and the gcell grid into the database. The nets written.
pub fn end_fr(db: &mut Db, d: &DesignIn, g: &GuidesIn, p: &Prep, routes: &DesignRoutes) -> Res<usize> {
    let names: Vec<String> = d.design.nets.iter().map(|n| n.name.clone()).collect();
    let has_ndr = |i: usize| d.design.nets[i].ndr.is_some();
    crate::dr::db::write_gcell_grid(db, &g.grid)?;
    crate::dr::db::write_routes(db, &p.tech, d.tech.via_defs.len(), routes, &names, &has_ndr)?;
    Ok(crate::dr::design::net_shapes(routes).len())
}
