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
    let layer_of = |l: i64| tech.layer_num(&db.layer_name_by_number(l));
    for (i, inst) in insts.iter().enumerate() {
        let m = &masters[&inst.unique.master];
        for (t, term) in m.terms.iter().enumerate() {
            let net = inst.nets[t].as_ref().and_then(|n| net_index.get(n)).copied();
            for pin in &term.pins {
                for &(l, r) in &pin.shapes {
                    fixed[l].push((inst.transform.apply(r), Fixed::InstTerm { net, inst: i, term: t }));
                }
            }
        }
        for &(l, r) in &m.blockages {
            fixed[l].push((inst.transform.apply(r), Fixed::InstBlockage { big: inst.class == MasterClass::Macro }));
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
                }
            }
        }
    }
    for (l, x0, y0, x1, y1) in db.obstruction_boxes().map_err(|e| e.to_string())? {
        if let Some(l) = layer_of(l) {
            fixed[l].push((Rect::new(x0, y0, x1, y1), Fixed::Blockage));
        }
    }
    Ok(TaDesign { nets, net_index, fixed })
}
