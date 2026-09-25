// SPDX-License-Identifier: Apache-2.0
//! Pin access end to end: unique classes, each class representative's access points, each port's,
//! the representatives' access patterns, each row's choice — and what becomes of it: every access
//! point of a class pin, each routed instance terminal's preferred points, each port's points.
//!
//! Stages, in order ([`pin_access`]): [`compute_unique`]; per standard-cell or block class, per
//! routed terminal and pin, [`gen_pin_access`] on the representative; per routed port, per pin;
//! per standard-cell class, [`prep_pattern_inst`]; the routed standard cells in placement order
//! split into rows, per row [`gen_inst_row_pattern`]; the chosen patterns' points per terminal.
//!
//! Rules:
//! - a port is routed when it is not a supply and its net is a regular (not special) net;
//! - a class takes part in the rows when its representative is a standard cell and it routes a
//!   terminal; each of its instances does;
//! - an instance terminal gets preferred points only when it is on a net and its class routes the
//!   terminal: per pin, the chosen pattern's point (none where the pattern has none);
//! - a class pin's points are stored relative to the representative's placement location (they
//!   hold for every member); a port's are absolute.

use std::collections::HashMap;

use crate::gc::Owner;
use crate::pa::access::{gen_pin_access, AccessPoint, Config, Pin as ApPin, Unsupported};
use crate::pa::candidates::{Context, TermKind};
use crate::pa::pattern::{prep_pattern_inst, Ap, Instance, Pattern, Pin as PatPin, Trace};
use crate::pa::row::{compute_inst_rows, gen_inst_row_pattern, inst_set_order, RowInst};
use crate::pa::unique::{compute_unique, UniqueClass, UniqueInst};
use crate::pa::verdict::TargetShape;
use crate::polygon90::Rect;
use crate::tech::{Master, Tech, Transform, TrackPattern};

/// What kind of master an instance is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MasterClass {
    /// `CORE` and its subclasses.
    StdCell,
    /// `BLOCK`, `PAD`, `RING`.
    Macro,
    Other,
}

/// An instance, with what the checks need.
pub struct DesignInst {
    pub name: String,
    pub unique: UniqueInst,
    pub class: MasterClass,
    pub is_block: bool,
    pub transform: Transform,
    /// Per master terminal: its net, if any.
    pub nets: Vec<Option<String>>,
    /// Its shapes as the checks see them.
    pub target: Vec<TargetShape>,
    /// Per master terminal: on a non-default-rule net without auto-taper.
    pub no_taper: Vec<bool>,
}

impl DesignInst {
    /// The owner of this instance's trial shapes on terminal `t`.
    pub fn pin_owner(&self, master: &Master, t: usize) -> Owner {
        match &self.nets[t] {
            Some(n) => Owner::Net(n.clone()),
            None => Owner::InstTerm(self.name.clone(), master.terms[t].name.clone()),
        }
    }
}

/// A top-level port.
pub struct DesignPort {
    pub name: String,
    pub routed: bool,
    pub owner: Owner,
    /// Per pin, its shapes.
    pub pins: Vec<Vec<(usize, Rect)>>,
    pub target: Vec<TargetShape>,
    /// On a non-default-rule net without auto-taper.
    pub no_taper: bool,
}

pub struct PinAccess {
    pub classes: Vec<UniqueClass>,
    /// Per class, per master terminal, per pin: its access points (design coordinates of the
    /// representative); empty for a terminal the class does not route.
    pub class_aps: Vec<Vec<Vec<Vec<AccessPoint>>>>,
    /// Per class: its patterns (standard-cell classes).
    pub patterns: Vec<Vec<Pattern>>,
    /// Per instance: the chosen pattern, when it takes part in a row.
    pub picks: Vec<Option<usize>>,
    /// Per port, per pin: its access points (routed ports).
    pub port_aps: Vec<Vec<Vec<AccessPoint>>>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FlowError {
    Unsupported(Unsupported),
    /// A routed terminal with no access point.
    NoAccessPoint(String),
    /// A row without a pattern combination.
    NoRowPath(String),
}

/// The pattern search's view of a class: its routed terminals' pins, in terminal order.
fn pattern_instance<'a>(tech: &'a Tech, rep: &'a DesignInst, master: &Master, class: &UniqueClass, aps: &[Vec<Vec<AccessPoint>>]) -> Instance<'a> {
    let mut pins = Vec::new();
    for (t, term) in master.terms.iter().enumerate() {
        if !class.routes[t] {
            continue;
        }
        let owner = rep.pin_owner(master, t);
        for (p, term_aps) in aps[t].iter().enumerate().take(term.pins.len()) {
            let pin_aps = term_aps
                .iter()
                .map(|a| Ap { point: a.point, layer: a.layer, cost: a.cost(), via: a.has(crate::pa::access::Access::U).then(|| &tech.via_defs[a.vias[0]]) })
                .collect();
            pins.push(PatPin { term: t, pin: p, owner: owner.clone(), aps: pin_aps });
        }
    }
    Instance { tech, target: &rep.target, location: rep.unique.location, pins }
}

pub fn pin_access(tech: &Tech, tracks: &[TrackPattern], cfg: &Config, masters: &HashMap<String, Master>, insts: &[DesignInst], ports: &[DesignPort]) -> Result<PinAccess, FlowError> {
    pin_access_with(tech, tracks, cfg, masters, insts, ports, None)
}

/// [`pin_access`], with every design shape (owner, layer, rectangle) for the via trials of pins
/// on non-default-rule nets without auto-taper; refused without them when there is such a pin.
pub fn pin_access_with(tech: &Tech, tracks: &[TrackPattern], cfg: &Config, masters: &HashMap<String, Master>, insts: &[DesignInst], ports: &[DesignPort], design: Option<&[TargetShape]>) -> Result<PinAccess, FlowError> {
    if design.is_none() && (insts.iter().any(|i| i.no_taper.iter().any(|&b| b)) || ports.iter().any(|p| p.no_taper)) {
        return Err(FlowError::Unsupported(Unsupported("a pin on a non-default-rule net without auto-taper needs the design's shapes")));
    }
    let cx = Context::new(tech, tracks);
    let uinsts: Vec<UniqueInst> = insts.iter().map(|i| i.unique.clone()).collect();
    let classes = compute_unique(tech, tracks, masters, &uinsts);

    // Each class representative's access points.
    let mut class_aps: Vec<Vec<Vec<Vec<AccessPoint>>>> = Vec::new();
    for class in &classes {
        let rep = &insts[class.insts[0]];
        let master = &masters[&class.key.master];
        let mut per_term: Vec<Vec<Vec<AccessPoint>>> = master.terms.iter().map(|t| vec![Vec::new(); t.pins.len()]).collect();
        if rep.class != MasterClass::Other {
            let kind = if rep.class == MasterClass::StdCell { TermKind::StdCell } else { TermKind::Macro };
            for (t, term) in master.terms.iter().enumerate() {
                if !class.routes[t] {
                    continue;
                }
                let owner = rep.pin_owner(master, t);
                let via_target = if rep.no_taper[t] { design } else { None };
                let pin = ApPin { cx: &cx, cfg, kind, is_block: rep.is_block, boundary: Some(rep.unique.bbox), target: &rep.target, owner: &owner, via_target };
                for (p, mp) in term.pins.iter().enumerate() {
                    let shapes: Vec<(usize, Rect)> = mp.shapes.iter().map(|&(l, r)| (l, rep.transform.apply(r))).collect();
                    per_term[t][p] = gen_pin_access(&pin, &shapes, &mut None).map_err(FlowError::Unsupported)?;
                }
                if per_term[t].iter().all(|a| a.is_empty()) {
                    return Err(FlowError::NoAccessPoint(format!("{}/{}", rep.name, term.name)));
                }
            }
        }
        class_aps.push(per_term);
    }

    // Each routed port's.
    let mut port_aps = Vec::new();
    for port in ports {
        let mut per_pin = Vec::new();
        if port.routed {
            let via_target = if port.no_taper { design } else { None };
            let pin = ApPin { cx: &cx, cfg, kind: TermKind::Io, is_block: false, boundary: None, target: &port.target, owner: &port.owner, via_target };
            for shapes in &port.pins {
                per_pin.push(gen_pin_access(&pin, shapes, &mut None).map_err(FlowError::Unsupported)?);
            }
            if per_pin.iter().all(|a: &Vec<AccessPoint>| a.is_empty()) {
                return Err(FlowError::NoAccessPoint(format!("PIN/{}", port.name)));
            }
        }
        port_aps.push(per_pin);
    }

    // Patterns per standard-cell class.
    let mut patterns: Vec<Vec<Pattern>> = Vec::new();
    for (c, class) in classes.iter().enumerate() {
        let rep = &insts[class.insts[0]];
        if rep.class != MasterClass::StdCell {
            patterns.push(Vec::new());
            continue;
        }
        let master = &masters[&class.key.master];
        let inst = pattern_instance(tech, rep, master, class, &class_aps[c]);
        patterns.push(prep_pattern_inst(&inst, &mut Trace(None)));
    }

    // Rows.
    let mut class_of = vec![0usize; insts.len()];
    for (c, class) in classes.iter().enumerate() {
        for &i in &class.insts {
            class_of[i] = c;
        }
    }
    let members: Vec<usize> = (0..insts.len()).filter(|&i| insts[i].class == MasterClass::StdCell && classes[class_of[i]].routes.iter().any(|&r| r)).collect();
    let boxes: Vec<Rect> = members.iter().map(|&i| insts[i].unique.bbox).collect();
    let order = inst_set_order(&boxes).ok_or_else(|| FlowError::NoRowPath("two instances share a lower-left".into()))?;
    let class_insts: Vec<Instance<'_>> = classes
        .iter()
        .enumerate()
        .map(|(c, class)| {
            let rep = &insts[class.insts[0]];
            pattern_instance(tech, rep, &masters[&class.key.master], class, &class_aps[c])
        })
        .collect();
    let mut picks = vec![None; insts.len()];
    for row in compute_inst_rows(&boxes, &order) {
        let row_insts: Vec<RowInst<'_>> = row
            .iter()
            .map(|&k| {
                let i = members[k];
                let c = class_of[i];
                let master = &masters[&classes[c].key.master];
                let owners = class_insts[c].pins.iter().map(|p| insts[i].pin_owner(master, p.term)).collect();
                RowInst { class: &class_insts[c], patterns: &patterns[c], location: insts[i].unique.location, owners, target: insts[i].target.clone() }
            })
            .collect();
        let chosen = gen_inst_row_pattern(tech, &row_insts, &mut None).ok_or_else(|| FlowError::NoRowPath(row.iter().map(|&k| insts[members[k]].name.clone()).collect::<Vec<_>>().join(" ")))?;
        for (&k, p) in row.iter().zip(chosen) {
            picks[members[k]] = Some(p);
        }
    }
    Ok(PinAccess { classes, class_aps, patterns, picks, port_aps })
}

/// A class point, as `(class, terminal, pin, index among the pin's points)`.
pub type ClassApKey = (usize, usize, usize, usize);

/// One write into the design database, in the order the reference's writer makes them.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WriteOp {
    /// Clear a master's points at a class index.
    ClearMaster { master: String, idx: u32 },
    /// A class point on its master pin: relative to the representative.
    AddMasterPoint { master: String, term: String, pin: usize, idx: u32, key: ClassApKey, point: (i32, i32), ap: AccessPoint },
    SetInstIdx { inst: String, idx: u32 },
    ClearPref { inst: String, term: String },
    /// A terminal pin's preferred point (a class point), or none.
    SetPref { inst: String, term: String, pin: usize, idx: u32, key: Option<ClassApKey> },
    /// A point on a block pin (absolute).
    AddPortPoint { port: String, pin: usize, ap: AccessPoint },
}

/// What pin access writes, in order:
/// - per master and class index (the number of EARLIER classes of the same master): the index
///   cleared (all classes first), then every point of every pin of every terminal, relative to
///   the class representative's placement location;
/// - per instance — EVERY one (the update flag starts set; only the row stage sets it again): its
///   class index, and per terminal the preferred points cleared, then — when the terminal is on a
///   net — per pin the chosen pattern's point, or none (always none outside the rows);
/// - per routed port with exactly ONE pin: every point of that pin.
pub fn write_plan(masters: &HashMap<String, Master>, insts: &[DesignInst], ports: &[DesignPort], pa: &PinAccess) -> Vec<WriteOp> {
    let mut ops = Vec::new();
    let mut per_master: HashMap<&str, u32> = HashMap::new();
    let idx: Vec<u32> = pa
        .classes
        .iter()
        .map(|c| {
            let n = per_master.entry(c.key.master.as_str()).or_insert(0);
            *n += 1;
            *n - 1
        })
        .collect();
    for (c, class) in pa.classes.iter().enumerate() {
        ops.push(WriteOp::ClearMaster { master: class.key.master.clone(), idx: idx[c] });
    }
    for (c, class) in pa.classes.iter().enumerate() {
        let rep = &insts[class.insts[0]];
        let master = &masters[&class.key.master];
        for (t, term) in master.terms.iter().enumerate() {
            for p in 0..term.pins.len() {
                for (a, ap) in pa.class_aps[c][t][p].iter().enumerate() {
                    let point = (ap.point.0 - rep.unique.location.0, ap.point.1 - rep.unique.location.1);
                    ops.push(WriteOp::AddMasterPoint { master: class.key.master.clone(), term: term.name.clone(), pin: p, idx: idx[c], key: (c, t, p, a), point, ap: ap.clone() });
                }
            }
        }
    }
    let mut class_of = vec![0usize; insts.len()];
    for (c, class) in pa.classes.iter().enumerate() {
        for &i in &class.insts {
            class_of[i] = c;
        }
    }
    for (i, inst) in insts.iter().enumerate() {
        let pick = pa.picks[i];
        let c = class_of[i];
        let class = &pa.classes[c];
        let master = &masters[&class.key.master];
        ops.push(WriteOp::SetInstIdx { inst: inst.name.clone(), idx: idx[c] });
        let mut k = 0; // the pattern's entry: routed terminals' pins in order
        for (t, term) in master.terms.iter().enumerate() {
            ops.push(WriteOp::ClearPref { inst: inst.name.clone(), term: term.name.clone() });
            let first = k;
            if class.routes[t] {
                k += term.pins.len();
            }
            if inst.nets[t].is_none() {
                continue;
            }
            for p in 0..term.pins.len() {
                let key = match (pick, class.routes[t]) {
                    (Some(pick), true) => pa.patterns[c][pick].aps[first + p].map(|(_, a)| (c, t, p, a)),
                    _ => None,
                };
                ops.push(WriteOp::SetPref { inst: inst.name.clone(), term: term.name.clone(), pin: p, idx: idx[c], key });
            }
        }
    }
    for (k, port) in ports.iter().enumerate() {
        if !port.routed || port.pins.len() != 1 {
            continue;
        }
        for ap in &pa.port_aps[k][0] {
            ops.push(WriteOp::AddPortPoint { port: port.name.clone(), pin: 0, ap: ap.clone() });
        }
    }
    ops
}

impl PinAccess {
    /// An instance terminal's preferred points, per pin, relative to the instance's location, as
    /// `(point, layer)` — none unless it is on a net and its class routes the terminal.
    pub fn pref_access_points(&self, insts: &[DesignInst], masters: &HashMap<String, Master>, inst: usize, t: usize) -> Vec<Option<((i32, i32), usize)>> {
        let Some(pick) = self.picks[inst] else { return Vec::new() };
        if insts[inst].nets[t].is_none() {
            return Vec::new();
        }
        let c = self.classes.iter().position(|c| c.insts.contains(&inst)).expect("a class");
        let class = &self.classes[c];
        if !class.routes[t] {
            return Vec::new();
        }
        let rep = &insts[class.insts[0]];
        let master = &masters[&class.key.master];
        // The pattern's entries run over the routed terminals' pins in terminal order.
        let start: usize = (0..t).filter(|&u| class.routes[u]).map(|u| master.terms[u].pins.len()).sum();
        let pattern = &self.patterns[c][pick];
        (0..master.terms[t].pins.len())
            .map(|p| {
                pattern.aps[start + p].map(|(pp, a)| {
                    let (term, pin) = self.pattern_pin(master, class, pp);
                    let ap = &self.class_aps[c][term][pin][a];
                    ((ap.point.0 - rep.unique.location.0, ap.point.1 - rep.unique.location.1), ap.layer)
                })
            })
            .collect()
    }

    /// The (terminal, pin) of a class's `k`-th pattern pin.
    fn pattern_pin(&self, master: &Master, class: &UniqueClass, k: usize) -> (usize, usize) {
        let mut n = 0;
        for (t, term) in master.terms.iter().enumerate() {
            if !class.routes[t] {
                continue;
            }
            if k < n + term.pins.len() {
                return (t, k - n);
            }
            n += term.pins.len();
        }
        unreachable!("a pattern pin of the class")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pa::candidates::ApType;
    use crate::pa::unique::ClassKey;
    use crate::tech::{MasterPin, MasterTerm};

    /// A master whose first terminal is a supply (not routed) and second a signal; two instances,
    /// the second with its signal unconnected; one pattern choosing the signal's only point.
    fn fixture() -> (HashMap<String, Master>, Vec<DesignInst>, PinAccess) {
        let pin = || MasterPin { shapes: vec![(2, Rect::new(0, 0, 100, 100))] };
        let master = Master { terms: vec![MasterTerm { name: "VPWR".into(), sig: "POWER".into(), pins: vec![pin()] }, MasterTerm { name: "A".into(), sig: "SIGNAL".into(), pins: vec![pin()] }], blockages: vec![] };
        let masters: HashMap<String, Master> = [("m".to_string(), master)].into();
        let inst = |name: &str, x: i32, net: Option<&str>| DesignInst {
            name: name.into(),
            unique: UniqueInst { master: "m".into(), orient: "R0".into(), location: (x, 0), bbox: Rect::new(x, 0, x + 1000, 1000), routes: vec![false, net.is_some()], ndr_no_taper: false },
            class: MasterClass::StdCell,
            is_block: false,
            transform: Transform { orient: "R0".into(), origin: (x, 0) },
            nets: vec![None, net.map(String::from)],
            target: vec![],
            no_taper: vec![false, false],
        };
        let insts = vec![inst("u0", 5000, Some("n")), inst("u1", 9000, None)];
        let ap = AccessPoint { point: (5050, 40), layer: 2, lower: ApType::OnGrid, upper: ApType::OnGrid, access: [false, false, false, false, true, false], allow_via: true, vias: vec![0] };
        let class = UniqueClass { key: ClassKey { master: "m".into(), orient: "R0".into(), offsets: vec![], ndr_inst: None }, insts: vec![0, 1], routes: vec![false, true] };
        let pa = PinAccess {
            classes: vec![class],
            class_aps: vec![vec![vec![vec![]], vec![vec![ap]]]],
            patterns: vec![vec![Pattern { aps: vec![Some((0, 0))], left: Some((0, 0)), right: Some((0, 0)), cost: 0 }]],
            picks: vec![Some(0), Some(0)],
            port_aps: vec![],
        };
        (masters, insts, pa)
    }

    /// A pattern's entries run over the ROUTED terminals' pins only: the signal after an unrouted
    /// supply is the pattern's FIRST entry. Its point is stored relative to the representative.
    #[test]
    fn pattern_entries_skip_unrouted_terminals() {
        let (masters, insts, pa) = fixture();
        assert_eq!(pa.pref_access_points(&insts, &masters, 0, 1), vec![Some(((50, 40), 2))]);
        assert!(pa.pref_access_points(&insts, &masters, 0, 0).is_empty());
    }

    /// The plan: a terminal pin on a net gets its pattern's point — keyed past the unrouted supply
    /// before it — an unconnected one nothing; EVERY instance gets its class index, even outside
    /// the rows (none of its pins then get a point).
    #[test]
    fn the_plan_writes_every_instance_and_only_connected_pins() {
        let (masters, insts, mut pa) = fixture();
        pa.picks[1] = None;
        let ops = write_plan(&masters, &insts, &[], &pa);
        let prefs: Vec<(&str, &str, Option<ClassApKey>)> = ops.iter().filter_map(|o| if let WriteOp::SetPref { inst, term, key, .. } = o { Some((inst.as_str(), term.as_str(), *key)) } else { None }).collect();
        assert_eq!(prefs, vec![("u0", "A", Some((0, 1, 0, 0)))]);
        let idx: Vec<&str> = ops.iter().filter_map(|o| if let WriteOp::SetInstIdx { inst, .. } = o { Some(inst.as_str()) } else { None }).collect();
        assert_eq!(idx, vec!["u0", "u1"]);
    }

    /// Only a routed port with exactly ONE pin gets its points written.
    #[test]
    fn the_plan_writes_single_pin_ports_only() {
        let (masters, insts, mut pa) = fixture();
        let ap = pa.class_aps[0][1][0][0].clone();
        let port = |name: &str, pins: usize| DesignPort { name: name.into(), routed: true, owner: Owner::Net(name.into()), pins: vec![vec![]; pins], target: vec![], no_taper: false };
        let ports = vec![port("one", 1), port("two", 2)];
        pa.port_aps = vec![vec![vec![ap.clone()]], vec![vec![ap.clone()], vec![ap]]];
        let ops = write_plan(&masters, &insts, &ports, &pa);
        let written: Vec<&str> = ops.iter().filter_map(|o| if let WriteOp::AddPortPoint { port, .. } = o { Some(port.as_str()) } else { None }).collect();
        assert_eq!(written, vec!["one"]);
    }

    /// A terminal without a net gets no preferred point, though its class routes it.
    #[test]
    fn an_unconnected_terminal_gets_no_preferred_point() {
        let (masters, insts, pa) = fixture();
        assert!(pa.pref_access_points(&insts, &masters, 1, 1).is_empty());
    }
}
