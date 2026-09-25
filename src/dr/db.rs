// SPDX-License-Identifier: Apache-2.0
//! Routing against the design database: what the rule tables do not model, refused up front.

use std::collections::HashMap;

use vyges_opendb::Db;

use crate::dr::ta::{Fixed, TaNet};
use crate::gc::Owner;
use crate::pa::flow::{DesignInst, DesignPort, MasterClass};
use crate::polygon90::Rect;
use crate::tech::{LayerKind, Master, Tech};

/// The rule families and layer properties routing does not model, as `layer: family` for each
/// layer that carries one — a design with any must be refused, not routed with the rule ignored.
/// A cut layer may carry one plain spacing rule (more than one is refused too). Also refused: a
/// routing layer that is rect-only or multi-patterned (both make it unidirectional, which changes
/// access points and track assignment) and LEF 5.4 spacing limited to a width RANGE.
pub fn unmodelled_rules(db: &Db, tech: &Tech) -> Vec<String> {
    let mut out = Vec::new();
    for l in &tech.layers {
        if l.kind == LayerKind::Placeholder {
            continue;
        }
        if l.kind == LayerKind::Routing {
            if db.layer_is_rect_only(&l.name) || db.layer_is_rect_only_except_non_core_pins(&l.name) {
                out.push(format!("{}: rect-only", l.name));
            }
            if db.layer_get_num_masks(&l.name) > 1 {
                out.push(format!("{}: multi-patterned", l.name));
            }
            if db.layer_v54_spacing_rules(&l.name).unwrap_or_default().iter().any(|r| r.1.is_some()) {
                out.push(format!("{}: spacing with a width range", l.name));
            }
        }
        for (family, n) in db.layer_rule_census(&l.name) {
            let modelled = match family.as_str() {
                "cut_spacing" => n <= 1,
                "v55_influence" => true,
                _ => false,
            };
            if !modelled {
                out.push(format!("{}: {family} ({n})", l.name));
            }
        }
    }
    out
}

/// Track assignment's view of the design: the routed nets (database order, special nets left
/// out) and every fixed shape by layer — each instance terminal's pin shapes (connected or not),
/// instance obstructions, block pins, special-net wiring (wires and via metal, as stored), and
/// routing blockages.
pub struct TaDesign {
    pub nets: Vec<TaNet>,
    pub net_index: HashMap<String, usize>,
    pub fixed: Vec<Vec<(Rect, Fixed)>>,
    /// Beside each fixed shape, who owns it as the design-rule check sees it: a net (special nets
    /// too), an unconnected terminal (or the floating power / ground owner), an instance's
    /// obstructions, a routing blockage.
    pub owners: Vec<Vec<Owner>>,
}

/// Refuses (`Err`) a special wire with no shape type: its ends would extend by half its width,
/// which the stored box does not show.
pub fn ta_design(db: &Db, tech: &Tech, masters: &HashMap<String, Master>, insts: &[DesignInst], ports: &[DesignPort]) -> Result<TaDesign, String> {
    let mut nets = Vec::new();
    let mut net_index = HashMap::new();
    let mut special = Vec::new();
    for n in db.net_names() {
        if db.net_is_special(&n) {
            special.push(n);
            continue;
        }
        net_index.insert(n.clone(), nets.len());
        nets.push(TaNet { is_clock: db.net_get_sig_type(&n) == "CLOCK", name: n, ndr: None });
    }
    let mut fixed: Vec<Vec<(Rect, Fixed)>> = vec![Vec::new(); tech.layers.len()];
    let mut owners: Vec<Vec<Owner>> = vec![Vec::new(); tech.layers.len()];
    let layer_of = |l: i64| tech.layer_num(&db.layer_name_by_number(l));
    for (i, inst) in insts.iter().enumerate() {
        let m = &masters[&inst.unique.master];
        for (t, term) in m.terms.iter().enumerate() {
            let net = inst.nets[t].as_ref().and_then(|n| net_index.get(n)).copied();
            let owner = crate::pa::verdict::design_owner(inst.nets[t].as_deref(), &term.sig, Owner::InstTerm(inst.name.clone(), term.name.clone()));
            for pin in &term.pins {
                for &(l, r) in &pin.shapes {
                    fixed[l].push((inst.transform.apply(r), Fixed::InstTerm { net, inst: i, term: t }));
                    owners[l].push(owner.clone());
                }
            }
        }
        for &(l, r) in &m.blockages {
            fixed[l].push((inst.transform.apply(r), Fixed::InstBlockage { big: inst.class == MasterClass::Macro }));
            owners[l].push(Owner::Inst(inst.name.clone()));
        }
    }
    for (k, port) in ports.iter().enumerate() {
        let net = match &port.owner {
            Owner::Net(n) => net_index.get(n).copied(),
            _ => None,
        };
        for pin in &port.pins {
            for &(l, r) in pin {
                fixed[l].push((r, Fixed::BTerm { net, port: k }));
                owners[l].push(port.owner.clone());
            }
        }
    }
    for n in &special {
        let supply = matches!(db.net_get_sig_type(n).as_str(), "POWER" | "GROUND");
        let boxes = db.net_swire_expanded_boxes(n).map_err(|e| e.to_string())?;
        let vias: std::collections::HashSet<(i64, i32, i32, i32, i32)> = boxes.iter().filter(|b| b.1).map(|b| (b.0, b.2, b.3, b.4, b.5)).collect();
        for (l, x0, y0, x1, y1, shape, _) in db.net_swire_shapes(n).map_err(|e| e.to_string())? {
            if shape == 0 && !vias.contains(&(l, x0.min(x1), y0.min(y1), x0.max(x1), y0.max(y1))) && !vias.contains(&(l, x0, y0, x1, y1)) {
                return Err(format!("special net {n}: a wire with no shape type"));
            }
        }
        // Wires first, then vias (each net's shapes, then its vias).
        for from_via in [false, true] {
            for &(l, v, x0, y0, x1, y1) in &boxes {
                if v != from_via {
                    continue;
                }
                if let Some(l) = layer_of(l) {
                    let f = if from_via { Fixed::Via { supply } } else { Fixed::Seg { supply } };
                    fixed[l].push((Rect::new(x0, y0, x1, y1), f));
                    owners[l].push(Owner::Net(n.clone()));
                }
            }
        }
    }
    for (b, (l, x0, y0, x1, y1)) in db.obstruction_boxes().map_err(|e| e.to_string())?.into_iter().enumerate() {
        if let Some(l) = layer_of(l) {
            fixed[l].push((Rect::new(x0, y0, x1, y1), Fixed::Blockage));
            owners[l].push(Owner::Blockage(b));
        }
    }
    Ok(TaDesign { nets, net_index, fixed, owners })
}

/// The routed design written into the database, net by net (routed nets, database order): each
/// net's routing replaced by its committed shapes — its wires in list order, then its vias, then
/// its patches — as paths: a wire on its layer (with the net's rule unless tapered), each end a
/// point carrying its extension when it is truncated (0) or differs from half the layer's width;
/// a via at its origin (a technology via by name, else the router's via written as a default
/// block via the first time it is used: its above, cut, then below rectangles); a patch as a
/// rectangle about its origin. Then [`port_stacks`], appended to their nets' wires. `tech_vias`:
/// how many of the technology's vias come from the database (the rest the router made).
///
/// ⚠️ Divergence, refused nowhere yet: the reference first removes every non-FIXED wire of EVERY
/// net and keeps FIXED ones; here only the nets written are replaced (a FIXED wire included).
pub fn write_routes(db: &mut Db, tech: &Tech, tech_vias: usize, d: &crate::dr::design::DesignRoutes, names: &[String], has_ndr: &dyn Fn(usize) -> bool) -> Result<(), String> {
    let per = crate::dr::design::net_shapes(d);
    let mut wires: std::collections::BTreeMap<usize, NetWire> = std::collections::BTreeMap::new();
    for (&net, (shapes, _)) in &per {
        let ndr = has_ndr(net);
        let w = wires.entry(net).or_default();
        let point = |ops: &mut Vec<i32>, p: (i32, i32), trunc: bool, ext: i32, half: i32| ops.extend(crate::dr::write::end_point_op(p, trunc, ext, half));
        for s in shapes.segs.iter().flatten() {
            let l = &tech.layers[s.layer];
            let li = w.name(&l.name);
            w.ops.extend([0, li, i32::from(ndr && !s.tapered)]);
            point(&mut w.ops, s.begin, s.begin_trunc, s.begin_ext, l.width / 2);
            point(&mut w.ops, s.end, s.end_trunc, s.end_ext, l.width / 2);
            w.wires += 1;
        }
        for v in shapes.vias.iter().flatten() {
            let vd = &tech.via_defs[v.via];
            let li = w.name(&tech.layers[vd.layer1].name);
            w.ops.extend([0, li, i32::from(ndr && !v.tapered)]);
            w.ops.extend([1, v.origin.0, v.origin.1]);
            w.via_points.insert(v.origin);
            let vi = w.name(&vd.name);
            if v.via < tech_vias {
                w.ops.extend([3, vi]);
            } else {
                let mut boxes: Vec<i32> = Vec::new();
                for (tag, figs) in [(2, &vd.layer2_figs), (1, &vd.cut_figs), (0, &vd.layer1_figs)] {
                    for r in figs {
                        boxes.extend([tag, r.xl, r.yl, r.xh, r.yh]);
                    }
                }
                db.block_create_via(&vd.name, (&tech.layers[vd.layer1].name, &tech.layers[vd.cut].name, &tech.layers[vd.layer2].name), &boxes).map_err(|e| e.to_string())?;
                w.ops.extend([4, vi]);
            }
            w.vias += 1;
        }
        for p in shapes.patches.iter().flatten() {
            let li = w.name(&tech.layers[p.layer].name);
            w.ops.extend([0, li, 0]);
            w.ops.extend([1, p.origin.0, p.origin.1]);
            w.ops.extend([5, p.offset.xl, p.offset.yl, p.offset.xh, p.offset.yh]);
            w.wires += 1;
        }
    }
    port_stacks(db, tech, tech_vias, names, &mut wires)?;
    for (net, w) in &wires {
        db.net_write_wire(&names[*net], &w.ops, &w.names).map_err(|e| e.to_string())?;
    }
    Ok(())
}

/// One net's wire as written: its encoding ops, the names they index, and what the reference
/// counts of it — wire shapes (segments and rectangles), vias, and the distinct points its via
/// paths start at.
#[derive(Default)]
struct NetWire {
    ops: Vec<i32>,
    names: Vec<String>,
    wires: usize,
    vias: usize,
    via_points: std::collections::BTreeSet<(i32, i32)>,
}

impl NetWire {
    fn name(&mut self, n: &str) -> i32 {
        if let Some(k) = self.names.iter().position(|x| x == n) {
            return k as i32;
        }
        self.names.push(n.to_string());
        (self.names.len() - 1) as i32
    }
}

/// After the routes, a via stack from the top routing layer up to each port wholly above it
/// (only when the block's maximum routing layer is set; ports in database order, not on special
/// nets), appended to its net's wire: a path on the top routing layer at [`best_via_position`] in
/// the port's first pin, then each cut layer's default via up to the port's lowest layer. A net
/// whose wire is vias alone, at as many distinct points as it has ports above, is already stacked
/// and left alone.
fn port_stacks(db: &Db, tech: &Tech, tech_vias: usize, names: &[String], wires: &mut std::collections::BTreeMap<usize, NetWire>) -> Result<(), String> {
    use crate::pa::stack::{best_via_position, default_via};
    if db.block_get_max_routing_layer() < 0 {
        return Ok(());
    }
    let cfg = crate::pa::db::config(db, tech);
    let top = cfg.top_routing_layer;
    let tracks = crate::tech::read::tracks(db, tech).map_err(|e| e.to_string())?;
    let level = |l: i64| db.layer_get_routing_level(&db.layer_name_by_number(l));
    let top_level = db.layer_get_routing_level(&tech.layers[top].name);
    // A port's lowest routing level, and its first pin's box.
    let port = |term: &str| -> Result<(i32, Option<Rect>), String> {
        let mut bottom = i32::MAX;
        let mut first: Option<Rect> = None;
        for p in 0..db.num_bterm_get_b_pins(term) {
            let boxes = db.bpin_layer_boxes(term, p).map_err(|e| e.to_string())?;
            for &(l, ..) in &boxes {
                bottom = bottom.min(level(l));
            }
            if p == 0 {
                first = boxes.iter().map(|&(_, x0, y0, x1, y1)| Rect::new(x0, y0, x1, y1)).reduce(|a, r| Rect::new(a.xl.min(r.xl), a.yl.min(r.yl), a.xh.max(r.xh), a.yh.max(r.yh)));
            }
        }
        Ok((bottom, first))
    };
    let index: HashMap<&str, usize> = names.iter().enumerate().map(|(i, n)| (n.as_str(), i)).collect();
    for term in db.bterm_names() {
        let net = db.bterm_get_net(&term);
        if net.is_empty() {
            return Err(format!("port {term} has no net (the reference dereferences it)"));
        }
        if db.net_is_special(&net) {
            continue;
        }
        let (bottom, first) = port(&term)?;
        if bottom == i32::MAX || bottom <= top_level {
            continue;
        }
        let n_above = db.net_bterms(&net).iter().map(|t| port(t).map(|(b, _)| b != i32::MAX && b > top_level)).collect::<Result<Vec<bool>, String>>()?.into_iter().filter(|&a| a).count();
        let Some(&ni) = index.get(net.as_str()) else { return Err(format!("net {net} not in the design")) };
        let w = wires.entry(ni).or_default();
        if w.wires == 0 && w.vias > 0 && w.via_points.len() == n_above {
            continue;
        }
        let pin_rect = first.unwrap_or(Rect { xl: 0, yl: 0, xh: 0, yh: 0 });
        let at = best_via_position(tech, &tracks, top, pin_rect);
        let li = w.name(&tech.layers[top].name);
        w.ops.extend([0, li, 0, 1, at.0, at.1]);
        w.via_points.insert(at);
        for lvl in top_level..bottom {
            // The cut above routing level `lvl`: the technology's layers alternate routing, cut.
            let cut = top + 1 + 2 * (lvl - top_level) as usize;
            let via = default_via(tech, cut, top).filter(|&v| v < tech_vias).ok_or_else(|| format!("port {term}: no technology default via on {}", tech.layers.get(cut).map_or("?", |l| l.name.as_str())))?;
            let vi = w.name(&tech.via_defs[via].name);
            w.ops.extend([3, vi]);
            w.vias += 1;
        }
    }
    Ok(())
}

/// The gcell grid the routing used, written to the database (one uniform pattern per axis).
pub fn write_gcell_grid(db: &mut Db, grid: &crate::dr::guides::GCellGrid) -> Result<(), String> {
    db.block_set_gcell_grid(grid.x, grid.y).map_err(|e| e.to_string())
}
