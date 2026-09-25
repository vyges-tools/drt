// SPDX-License-Identifier: Apache-2.0
//! Pin access against the design database: the design as the chain reads it ([`read_design`]),
//! the settings ([`config`]), and the results written back ([`update_db`]).
//!
//! What is written, and in which order, is [`write_plan`]'s (pure, unit-tested); this module
//! executes it.

use std::collections::HashMap;

use vyges_opendb::Db;

use crate::gc::Owner;
use crate::pa::access::Config;
use crate::pa::flow::{write_plan, ClassApKey, DesignInst, DesignPort, MasterClass, PinAccess, WriteOp};
use crate::pa::unique::{routes_term, UniqueInst};
use crate::pa::verdict::{design_owner, instance_shapes};
use crate::polygon90::Rect;
use crate::tech::{read, LayerKind, Master, Tech};

type Res<T> = Result<T, String>;

/// The masters in use, every instance (database order), every port.
pub type Design = (HashMap<String, Master>, Vec<DesignInst>, Vec<DesignPort>);

/// Every instance (database order), every port, and the masters they use.
pub fn read_design(db: &Db, tech: &Tech) -> Res<Design> {
    let mut masters: HashMap<String, Master> = HashMap::new();
    let mut insts = Vec::new();
    for name in db.inst_names() {
        let master = db.inst_get_master(&name);
        if !masters.contains_key(&master) {
            let terms = read::master_terms(db, tech, &master).map_err(|e| e.to_string())?;
            let obs = read::master_obstructions(db, tech, &master).map_err(|e| e.to_string())?;
            masters.insert(master.clone(), Master::import(tech, terms, &obs));
        }
        let m = &masters[&master];
        let nets: Vec<Option<String>> = m
            .terms
            .iter()
            .map(|t| {
                let n = db.iterm_get_net(&name, &t.name);
                (!n.is_empty()).then_some(n)
            })
            .collect();
        let routes = m.terms.iter().zip(&nets).map(|(t, n)| routes_term(&t.sig, n.as_deref(), n.as_deref().is_some_and(|n| db.net_is_special(n)), false)).collect();
        let owners: Vec<Owner> = m.terms.iter().zip(&nets).map(|(t, n)| design_owner(n.as_deref(), &t.sig, Owner::InstTerm(name.clone(), t.name.clone()))).collect();
        let transform = read::transform(db, &name);
        let target = instance_shapes(m, &name, &transform, |t| owners[t].clone());
        let b = db.inst_bbox(&name).map_err(|e| e.to_string())?;
        let mtype = db.master_get_type(&master).unwrap_or_default();
        let class = if mtype.starts_with("CORE") {
            MasterClass::StdCell
        } else if mtype.starts_with("BLOCK") || mtype.starts_with("PAD") || mtype == "RING" {
            MasterClass::Macro
        } else {
            MasterClass::Other
        };
        insts.push(DesignInst {
            unique: UniqueInst { master: master.clone(), orient: db.inst_get_orient(&name), location: read::location(db, &name), bbox: Rect::new(b[0], b[1], b[2], b[3]), routes, ndr_no_taper: false },
            name,
            class,
            is_block: mtype.starts_with("BLOCK"),
            transform,
            nets,
            target,
        });
    }
    let mut ports = Vec::new();
    for term in db.bterm_names() {
        let net = db.bterm_get_net(&term);
        let net = (!net.is_empty()).then_some(net);
        let sig = db.bterm_get_sig_type(&term);
        let routed = sig != "POWER" && sig != "GROUND" && net.as_deref().is_some_and(|n| !db.net_is_special(n));
        let dso = design_owner(net.as_deref(), &sig, Owner::BlockTerm(term.clone()));
        let mut pins = Vec::new();
        for p in 0..db.num_bterm_get_b_pins(&term) {
            let boxes = db.bpin_layer_boxes(&term, p).map_err(|e| e.to_string())?;
            pins.push(boxes.into_iter().filter_map(|(l, x0, y0, x1, y1)| tech.layer_num(&db.layer_name_by_number(l)).map(|l| (l, Rect::new(x0, y0, x1, y1)))).collect::<Vec<_>>());
        }
        let target = pins.iter().flatten().map(|&(l, r)| (dso.clone(), l, r)).collect();
        ports.push(DesignPort { owner: net.clone().map_or(Owner::BlockTerm(term.clone()), Owner::Net), name: term, routed, pins, target });
    }
    Ok((masters, insts, ports))
}

/// The router's defaults; the top routing layer is the block's maximum routing layer when set,
/// else the topmost routing layer.
pub fn config(db: &Db, tech: &Tech) -> Config {
    let max_level = db.block_get_max_routing_layer();
    let top = (0..tech.layers.len())
        .find(|&l| max_level > 0 && tech.layers[l].kind == LayerKind::Routing && db.layer_get_routing_level(&tech.layers[l].name) == max_level)
        .unwrap_or_else(|| tech.top_routing_layer());
    Config { via_access_layer: 2, via_in_pin: None, top_routing_layer: top, min_std_cell_points: 3, min_macro_points: 3, use_nonpref_tracks: true }
}

/// Write the results into the database: [`write_plan`], executed.
pub fn update_db(db: &mut Db, tech: &Tech, masters: &HashMap<String, Master>, insts: &[DesignInst], ports: &[DesignPort], pa: &PinAccess) -> Res<()> {
    let e = |r: vyges_opendb::Result<()>| r.map_err(|e| e.to_string());
    let vias = |ap: &crate::pa::access::AccessPoint| -> Vec<String> { ap.vias.iter().map(|&v| tech.via_defs[v].name.clone()).collect() };
    let mut db_ap: HashMap<ClassApKey, usize> = HashMap::new();
    for op in write_plan(masters, insts, ports, pa) {
        match op {
            WriteOp::ClearMaster { master, idx } => e(db.master_clear_pin_access(&master, idx as i32))?,
            WriteOp::AddMasterPoint { master, term, pin, idx, key, point, ap } => {
                let k = db
                    .mpin_add_access_point(&master, &term, pin, idx, point, &tech.layers[ap.layer].name, ap.db_access_bits(), (ap.lower as i32, ap.upper as i32), &vias(&ap), &[])
                    .map_err(|e| e.to_string())?;
                db_ap.insert(key, k as usize);
            }
            WriteOp::SetInstIdx { inst, idx } => e(db.inst_set_pin_access_idx(&inst, idx))?,
            WriteOp::ClearPref { inst, term } => e(db.iterm_clear_pref_access_points(&inst, &term))?,
            WriteOp::SetPref { inst, term, pin, idx, key } => e(db.iterm_set_access_point(&inst, &term, pin, idx, key.map(|k| db_ap[&k])))?,
            WriteOp::AddPortPoint { port, pin, ap } => e(db.bpin_add_access_point(&port, pin, ap.point, &tech.layers[ap.layer].name, ap.db_access_bits(), (ap.lower as i32, ap.upper as i32), &vias(&ap), &[]))?,
        }
    }
    Ok(())
}
