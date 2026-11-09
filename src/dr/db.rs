// SPDX-License-Identifier: Apache-2.0
//! Routing against the design database: what the rule tables do not model, refused up front.

use std::collections::HashMap;

use vyges_opendb::Db;

use crate::dr::ta::{Fixed, TaNet};
use crate::gc::{Marker, Owner};
use crate::pa::flow::{DesignInst, DesignPort, MasterClass};
use crate::polygon90::Rect;
use crate::tech::{LayerKind, Master, Tech};
pub use crate::dr::wire::InitialRouting;

/// The rule families and layer properties routing does not model, as `layer: family` for each
/// layer that carries one — a design with any must be refused, not routed with the rule ignored.
/// A cut layer may carry one plain spacing rule (more than one is refused too). A multi-patterned
/// routing layer is modelled as the reference's io takes it: unidirectional and
/// right-way-on-grid-only, with no colour assigned (colours are a later step's).
/// Spacing limited to a width RANGE and master obstructions carrying their own DESIGNRULEWIDTH or
/// SPACING are modelled. A rect-only layer is modelled (unidirectional, and the
/// check's rect-only rule); its "except non-core pins" flag is never read.
/// Right-way-on-grid-only is modelled in every consumer (pin access, the maze grid, guides and
/// the maze's pin feedthrough), and so is a multi-patterned layer, which io also gives it.
pub fn unmodelled_rules(db: &Db, tech: &Tech) -> Vec<String> {
    let mut out = Vec::new();
    let has_block_master = db.inst_names().iter().any(|i| db.master_get_type(&db.inst_get_master(i)).unwrap_or_default().starts_with("BLOCK"));
    for l in &tech.layers {
        if l.kind == LayerKind::Placeholder {
            continue;
        }
        // Families the reference's io translates that the database census does not list, counted
        // here from the generated accessors — only the rules io KEEPS (it drops the rest with a
        // warning, and sky130's cut layers carry two LEF58 enclosure rules without a CUTCLASS that
        // it drops that way) — and refused (none is modelled).
        let name = l.name.as_str();
        let kept = |n: usize, keeps: &dyn Fn(usize) -> bool| (0..n).filter(|&k| keeps(k)).count();
        let untracked = [
            ("lef58_max_spacing", kept(db.num_layer_get_tech_layer_max_spacing_rules(name), &|k| db.maxspacingrule_has_cut_class(name, k))),
            ("lef58_two_wires_forbidden_spacing", db.num_layer_get_tech_layer_two_wires_forbidden_spc_rules(name)),
            ("lef58_width_table_orthogonal", kept(db.num_layer_get_tech_layer_width_table_rules(name), &|k| db.widthtablerule_is_orthogonal(name, k))),
            ("lef58_wrong_dir_spacing", kept(db.num_layer_get_tech_layer_wrong_dir_spacing_rules(name), &|k| !db.wrongdirspacingrule_is_length_valid(name, k) && db.wrongdirspacingrule_get_prl_length(name, k) >= 0)),
            ("orthogonal_spacing_table", usize::from(db.layer_has_orth_spacing_table(name))),
        ];
        for (family, n) in untracked {
            if n > 0 {
                out.push(format!("{name}: {family} ({n})"));
            }
        }
        for (family, n) in db.layer_rule_census(&l.name) {
            if family == "lef58_spacing_eol" {
                // Modelled rule by rule: the first clause outside the subset names the refusal.
                if let Some(clause) = (0..n).find_map(|k| crate::tech::read::lef58_eol_rule(db, &l.name, k).err()) {
                    out.push(format!("{}: {family} ({n}, {clause})", l.name));
                }
                continue;
            }
            if family == "lef58_corner_spacing" {
                // Modelled rule by rule (the first clause outside the subset names the refusal),
                // with the router's patch pass (`patchMetalShape_cornerSpacing`). ⛔ Not modelled: a
                // BLOCK master pin's corner-to-corner cost (`modCornerToCornerSpacing`, block masters
                // only) — refused while the design instantiates a block master (conservatively,
                // whether or not its pins reach this layer).
                if let Some(clause) = (0..n).find_map(|k| crate::tech::read::corner_spacing_rule(db, &l.name, k).err()) {
                    out.push(format!("{}: {family} ({n}, {clause})", l.name));
                } else if has_block_master {
                    out.push(format!("{}: {family} ({n}, a block master's pin corner cost)", l.name));
                }
                continue;
            }
            if family == "lef58_cut_spacing" {
                // Modelled for the cut-to-corner forms alone (`SPACING s LAYER l CUTCLASS c
                // CONCAVECORNER`: the check, the only consumer reached — the via-to-via prep that
                // also reads it runs only for one named process node, an option this router does
                // not take; `… NONEOLCONVEXCORNER w [MINLENGTH m]`: the check, its only consumer);
                // the first rule outside them names the refusal.
                let classes = &l.cut_classes;
                if let Some(clause) = (0..n).find_map(|k| crate::tech::read::cut_layer_rule(db, &l.name, k, classes).err()) {
                    out.push(format!("{}: {family} ({n}, {clause})", l.name));
                }
                continue;
            }
            if family == "lef58_cut_spacing_table" {
                // Modelled for the different-net table on the layer itself (check, maze cost,
                // via-to-via and the default `prep_cutSpcTbl` sets); the reader names any other kind.
                if let Err(clause) = crate::tech::read::cut_spacing_table(db, &l.name, &l.cut_classes) {
                    out.push(format!("{}: {family} ({n}, {clause})", l.name));
                }
                continue;
            }
            let modelled = match family.as_str() {
                "cut_spacing" => n <= 1,
                // Cut classes: read into the layer, looked up by a cut's width and length.
                "lef58_cut_class" => true,
                // MINSTEP: the check (`checkMetalShape_minStep`) and the via-to-via prep; a layer
                // whose via pads break it is refused when the tables are built.
                "min_step" => true,
                // MINIMUMCUT: the check (`checkMinimumCut`), the maze cost around committed vias and
                // macro pins (`modMinimumcutCostVia`) and the via-to-via prep.
                "min_cut" => true,
                // ⛔ SPACINGTABLE INFLUENCE: the check (`checkMetalSpacingTableInfluence`) is not
                // modelled — refused (it was counted modelled before; no corpus tech carries it).
                "v55_influence" => false,
                "lef58_eol_keepout" => true,
                _ => false,
            };
            if !modelled {
                out.push(format!("{}: {family} ({n})", l.name));
            }
        }
    }
    out
}


/// The upstream issue for the end-of-line forms [`skipped_rules`] lists.
pub const SKIPPED_EOL_UPSTREAM_ISSUE: &str = "OpenROAD issue #12174";

/// A rule the technology carries that routing neither checks nor refuses: it is SKIPPED, with a
/// warning at tech load and an entry in the run's report.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SkippedRule {
    pub layer: String,
    /// The census family (`lef58_spacing_eol`).
    pub family: &'static str,
    /// The rule's index among the layer's rules of that family (from 0).
    pub index: usize,
    /// The clause that makes it skipped.
    pub clause: &'static str,
}

impl SkippedRule {
    /// The warning logged at tech load.
    pub fn warning(&self) -> String {
        format!(
            "layer {}: LEF58_SPACING end-of-line rule {} uses {}, which this router does not model; the rule is SKIPPED and NOT checked during routing, so sign-off DRC must cover it (the reference reads this form as a plain end-of-line rule with WITHIN 0, which is not followed here; see {}).",
            self.layer, self.index, self.clause, SKIPPED_EOL_UPSTREAM_ISSUE
        )
    }
}

/// The rules routing skips with a warning instead of refusing, layer by layer in the
/// technology's order: each LEF58 end-of-line rule with TOCONCAVECORNER or TONOTCHLENGTH
/// ([`crate::tech::read::lef58_eol_skipped_clause`]). ⛔ Not enforced and not emulated: the
/// reference checks these as a plain end-of-line rule with WITHIN 0 (upstream OpenROAD issue
/// #12174), a reading we do not copy. Every other refusal of [`unmodelled_rules`] stands.
pub fn skipped_rules(db: &Db, tech: &Tech) -> Vec<SkippedRule> {
    let mut out = Vec::new();
    for l in tech.layers.iter().filter(|l| l.kind == LayerKind::Routing) {
        for k in 0..db.num_layer_get_tech_layer_spacing_eol_rules(&l.name) {
            if let Some(clause) = crate::tech::read::lef58_eol_skipped_clause(db, &l.name, k) {
                out.push(SkippedRule { layer: l.name.clone(), family: "lef58_spacing_eol", index: k, clause });
            }
        }
    }
    out
}

/// A layer PROPERTY the database's LEF reader leaves unparsed, by kind: odb keeps it only as raw
/// text on the layer (its ODB-0388 "unsupported property" INFO), so no rule reaches routing and
/// it is NOT checked. Informational: never a refusal, and the reference is silent about it too.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnparsedProperty {
    pub layer: String,
    /// The property name as the LEF writes it (`LEF58_ENCLOSUREEDGE`, …).
    pub kind: String,
    /// The `;`-terminated statements in the raw text odb kept. odb keeps ONE string property per
    /// name and layer, and a later PROPERTY of the same name is APPENDED to it (with a space,
    /// `dbStringProperty::create`), so every statement of the kind is counted.
    pub statements: usize,
}

impl UnparsedProperty {
    /// The line logged at tech load.
    pub fn notice(&self) -> String {
        format!(
            "layer {}: {} ({} statement(s)) is not parsed by the database reader and is NOT checked during routing; sign-off DRC must cover it (informational: the reference does not check it either).",
            self.layer, self.kind, self.statements
        )
    }
}

/// Whether odb's LEF reader parses a layer PROPERTY of this name on a layer of this type into a
/// rule (lefin `layer()` at the pin, branch by branch: routing, cut, masterslice and implant
/// layers each have their own list; no other layer type parses any). Every other property gets
/// ODB-0388 and survives only as raw text.
fn odb_parses_layer_property(layer_type: &str, name: &str) -> bool {
    match layer_type {
        "ROUTING" => matches!(
            name,
            "LEF58_SPACING" | "LEF58_MINSTEP" | "LEF57_MINSTEP" | "LEF58_CORNERSPACING" | "LEF58_SPACINGTABLE"
                | "LEF58_RIGHTWAYONGRIDONLY" | "LEF58_RECTONLY" | "LEF58_BACKSIDE" | "LEF58_TYPE"
                | "LEF58_EOLEXTENSIONSPACING" | "LEF58_EOLKEEPOUT" | "LEF58_WIDTHTABLE" | "LEF58_MINIMUMCUT"
                | "LEF58_PITCH" | "LEF58_AREA" | "LEF58_FORBIDDENSPACING" | "LEF58_TWOWIRESFORBIDDENSPACING"
                | "LEF58_MINWIDTH" | "LEF57_ANTENNAGATEPLUSDIFF" | "LEF58_ANTENNAGATEPLUSDIFF" | "LEF58_VOLTAGESPACING"
        ),
        "CUT" => matches!(
            name,
            "LEF58_SPACING" | "LEF57_SPACING" | "LEF58_CUTCLASS" | "LEF58_ENCLOSURE" | "LEF58_ENCLOSURETABLE"
                | "LEF58_SPACINGTABLE" | "LEF58_ARRAYSPACING" | "LEF58_TYPE" | "LEF58_BACKSIDE" | "LEF58_KEEPOUTZONE"
                | "LEF58_MAXSPACING" | "LEF57_ANTENNAGATEPLUSDIFF" | "LEF58_ANTENNAGATEPLUSDIFF"
        ),
        "MASTERSLICE" => name == "LEF58_TYPE",
        "IMPLANT" => name == "LEF58_AREA",
        _ => false,
    }
}

/// The LEF58 layer properties the database's reader left unparsed (ODB-0388), per layer in the
/// technology's order and per kind in odb's property order, read-only from the raw text odb keeps
/// on each layer (one string property per name, repeats appended). Informational ([`UnparsedProperty`]): not
/// checked, never refused.
pub fn lef58_not_checked(db: &Db) -> Vec<UnparsedProperty> {
    let mut out: Vec<UnparsedProperty> = Vec::new();
    let layers = db.layers_with_direction().unwrap_or_default();
    for (layer, _) in layers {
        let kind = db.layer_get_type(&layer).unwrap_or_default();
        for (name, text) in db.layer_string_properties(&layer) {
            if !name.starts_with("LEF58_") || odb_parses_layer_property(&kind, &name) {
                continue;
            }
            let statements = text.split(';').filter(|t| !t.trim().is_empty()).count();
            out.push(UnparsedProperty { layer: layer.clone(), kind: name, statements });
        }
    }
    out
}


/// A regular net's routing read from the database (`None` without a wire). Refused: a FIXED wire
/// (the router then rips up incrementally in every iteration), a wire type other than ROUTED, a
/// block via (the reader would add the design's vias to the technology's), a layer or via the
/// technology does not have.
pub fn read_net_routing(db: &Db, tech: &Tech, net: &str) -> Result<Option<crate::dr::wire::InitialRouting>, String> {
    let recs = db.net_wire_decode(net).map_err(|e| e.to_string())?;
    let Some(first) = recs.first() else { return Ok(None) };
    match first.strip_prefix("T|") {
        Some("ROUTED") => {}
        Some(t) => return Err(format!("net {net}: {t} wiring not modelled")),
        None => return Err(format!("net {net}: no wire type")),
    }
    let ops = crate::dr::wire::wire_ops(&recs[1..])?;
    crate::dr::wire::parse_wire(tech, &ops).map(Some).map_err(|e| format!("net {net}: {e}"))
}

/// Each net's routing read from the database (`None` without a wire): what makes a run
/// incremental. Refused: vias or patches with no wire (the net would not count as routed), a
/// routed net on a non-default rule (its wires are read at the layer's width).
pub fn initial_routing(db: &Db, tech: &Tech, nets: &[TaNet]) -> Result<Vec<Option<InitialRouting>>, String> {
    let mut out = vec![None; nets.len()];
    for (i, n) in nets.iter().enumerate() {
        let Some(r) = read_net_routing(db, tech, &n.name)? else { continue };
        if !r.has_wire() {
            return Err(format!("net {}: vias or patches with no wire not modelled", n.name));
        }
        if n.ndr.is_some() {
            return Err(format!("net {}: a routed non-default-rule net not modelled (its wires read at the layer width)", n.name));
        }
        out[i] = Some(r);
    }
    Ok(out)
}

/// The nets routed before routing began, as the design's starting shapes: per net (design
/// order) its wires, vias and patches. A via landing on one of the net's terminals is marked
/// connected there — on the layer below when a terminal shape on it touches the via's shape
/// there; else on the layer above likewise. (Terminals are tried in the net's order and a
/// terminal after one that connected the bottom is not tried for the top: refused where that
/// order could matter — a via touching terminals both below and above.)
pub fn initial_shapes(tech: &Tech, nets: &[TaNet], insts: &[DesignInst], masters: &HashMap<String, Master>, ports: &[DesignPort], initial: &[Option<InitialRouting>]) -> Result<Vec<(usize, crate::dr::cost::DrFig)>, String> {
    use crate::dr::cost::DrFig;
    let mut out = Vec::new();
    let meets = |a: &Rect, b: &Rect| a.xl <= b.xh && b.xl <= a.xh && a.yl <= b.yh && b.yl <= a.yh;
    for (n, r) in initial.iter().enumerate() {
        let Some(r) = r else { continue };
        let name = &nets[n].name;
        let mut terms: Vec<Vec<(usize, Rect)>> = Vec::new();
        for inst in insts {
            for (k, net) in inst.nets.iter().enumerate() {
                if net.as_deref() == Some(name.as_str()) {
                    let m = &masters[&inst.unique.master];
                    terms.push(m.terms[k].pins.iter().flat_map(|p| p.shapes.iter()).map(|&(l, b)| (l, inst.transform.apply(b))).collect());
                }
            }
        }
        for port in ports {
            if port.owner == Owner::Net(name.clone()) {
                terms.push(port.pins.iter().flatten().copied().collect());
            }
        }
        out.extend(r.segs.iter().map(|f| (n, f.clone())));
        for f in &r.vias {
            let mut f = f.clone();
            if let DrFig::Via { via, origin, bottom_connected, top_connected, .. } = &mut f {
                let vd = &tech.via_defs[*via];
                let at = |b: Rect| Rect { xl: b.xl + origin.0, yl: b.yl + origin.1, xh: b.xh + origin.0, yh: b.yh + origin.1 };
                let (bot, top) = (at(vd.layer1_bbox()), at(vd.layer2_bbox()));
                let hits = |sh: &[(usize, Rect)], l: usize, b: &Rect| sh.iter().any(|(sl, r)| *sl == l && meets(r, b));
                let below: Vec<bool> = terms.iter().map(|sh| hits(sh, vd.layer1, &bot)).collect();
                let above: Vec<bool> = terms.iter().map(|sh| hits(sh, vd.layer2, &top)).collect();
                if below.iter().any(|&b| b) && above.iter().any(|&b| b) {
                    return Err(format!("net {name}: a via on terminals below and above not modelled"));
                }
                *bottom_connected = below.iter().any(|&b| b);
                *top_connected = !*bottom_connected && above.iter().any(|&b| b);
            }
            out.push((n, f));
        }
        out.extend(r.patches.iter().map(|f| (n, f.clone())));
    }
    Ok(out)
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

/// `io::Parser::setMarkers`: the markers of the database's `DRC` category and everything under it,
/// in the database's order, each on its layer with the layer's rule ([`db_marker_rule`]), its box
/// and its NET sources as owners; one on no layer, on a layer the router does not hold, or on a
/// layer with no such rule is dropped. They stand as the design's markers before the first
/// iteration (and make the run resume, [`start_iteration`]).
///
/// [`db_marker_rule`]: crate::dr::flow::db_marker_rule
/// [`start_iteration`]: crate::dr::flow::start_iteration
pub fn set_markers(db: &Db, tech: &Tech) -> Result<Vec<Marker>, String> {
    let mut out = Vec::new();
    for m in db.category_markers(crate::dr::flow::DRC_MARKER_CATEGORY).map_err(|e| e.to_string())? {
        let Some(name) = m.layer else { continue };
        let Some(layer) = tech.layers.iter().position(|l| l.name == name) else { continue };
        let Some(rule) = crate::dr::flow::db_marker_rule(tech.layers[layer].kind) else { continue };
        let mut owners: Vec<Owner> = m.source_nets.into_iter().map(Owner::Net).collect();
        owners.sort();
        owners.dedup();
        let [xl, yl, xh, yh] = m.rect;
        out.push(Marker { rule, layer, bbox: Rect { xl, yl, xh, yh }, owners, victim: None, aggressor: None });
    }
    Ok(out)
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
        for (k, &(l, r)) in m.blockages.iter().enumerate() {
            fixed[l].push((inst.transform.apply(r), Fixed::InstBlockage { big: inst.class == MasterClass::Macro }));
            owners[l].push(crate::pa::verdict::blockage_owner(m, &inst.name, k));
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

#[cfg(test)]
mod skipped_rule_tests {
    //! Rule (ours, not the reference's): a LEF58 end-of-line rule with TOCONCAVECORNER or
    //! TONOTCHLENGTH is SKIPPED with a warning — not enforced, not emulated, not refused. The
    //! reference's io never tests either branch and checks the rule as a plain end-of-line rule
    //! with WITHIN 0 (MINADJACENTLENGTH dropped), flagging line ends that face straight edges
    //! (upstream OpenROAD issue #12174). Copying that would transcribe a misreading, and the rule
    //! as written has no oracle, so it is left to sign-off DRC. Every other refusal stands.
    use super::{skipped_rules, unmodelled_rules};
    use vyges_opendb::Db;

    /// A synthetic two-metal technology; `m1` and `v1` are extra lines inside M1 and V1.
    fn db(name: &str, m1: &str, v1: &str) -> Db {
        let lef = format!(
            "VERSION 5.8 ;\nUNITS\n  DATABASE MICRONS 1000 ;\nEND UNITS\nMANUFACTURINGGRID 0.005 ;\n\
             LAYER M1\n  TYPE ROUTING ;\n  DIRECTION HORIZONTAL ;\n  PITCH 0.12 ;\n  WIDTH 0.04 ;\n  SPACING 0.04 ;\n{m1}END M1\n\
             LAYER V1\n  TYPE CUT ;\n  SPACING 0.05 ;\n  WIDTH 0.04 ;\n  PROPERTY LEF58_CUTCLASS \"CUTCLASS CA WIDTH 0.04 ;\" ;\n{v1}END V1\n\
             LAYER M2\n  TYPE ROUTING ;\n  DIRECTION VERTICAL ;\n  PITCH 0.12 ;\n  WIDTH 0.04 ;\n  SPACING 0.04 ;\nEND M2\nEND LIBRARY\n"
        );
        let path = std::env::temp_dir().join(format!("vyges-drt-skipped-{}-{name}.lef", std::process::id()));
        std::fs::write(&path, lef).expect("write lef");
        let mut db = Db::new();
        db.read_lef(path.to_str().expect("utf-8 path")).expect("read lef");
        let _ = std::fs::remove_file(&path);
        db
    }

    fn eol(rule: &str) -> String {
        format!("  PROPERTY LEF58_SPACING \"{rule}\" ;\n")
    }

    /// (skipped, refused, M1's enforced LEF58 end-of-line rules) for a technology.
    fn census(db: &Db) -> (Vec<super::SkippedRule>, Vec<String>, usize) {
        let tech = crate::tech::read::tech(db).expect("tech");
        let m1 = tech.layers.iter().find(|l| l.name == "M1").expect("M1").lef58_eol.len();
        (skipped_rules(db, &tech), unmodelled_rules(db, &tech), m1)
    }

    #[test]
    fn a_to_concave_corner_rule_is_skipped_with_a_warning_not_refused_and_not_checked() {
        let d = db("tcc", &eol("SPACING 0.09 ENDOFLINE 0.07 TOCONCAVECORNER MINADJACENTLENGTH 0.1 0.12 ;"), "");
        let (skipped, refused, enforced) = census(&d);
        assert!(refused.is_empty(), "{refused:?}");
        assert_eq!(enforced, 0, "not enforced: the reference's WITHIN-0 reading is not copied");
        assert_eq!(skipped.len(), 1);
        let s = &skipped[0];
        assert_eq!((s.layer.as_str(), s.family, s.index, s.clause), ("M1", "lef58_spacing_eol", 0, "TOCONCAVECORNER"));
        let w = s.warning();
        for part in ["layer M1", "TOCONCAVECORNER", "NOT checked during routing", "sign-off DRC must cover it", "OpenROAD issue #12174"] {
            assert!(w.contains(part), "{part}: {w}");
        }
    }

    #[test]
    fn a_to_notch_length_rule_is_skipped_too() {
        let d = db("tnl", &eol("SPACING 0.09 ENDOFLINE 0.07 TONOTCHLENGTH 0.1 ;"), "");
        let (skipped, refused, enforced) = census(&d);
        assert!(refused.is_empty(), "{refused:?}");
        assert_eq!(enforced, 0);
        assert_eq!(skipped.iter().map(|s| s.clause).collect::<Vec<_>>(), ["TONOTCHLENGTH"]);
    }

    /// The control: the same rule written as a plain end-of-line rule is checked, not skipped.
    #[test]
    fn a_plain_end_of_line_rule_is_checked_not_skipped() {
        let d = db("w0", &eol("SPACING 0.09 ENDOFLINE 0.07 WITHIN 0 ;"), "");
        let (skipped, refused, enforced) = census(&d);
        assert!(skipped.is_empty() && refused.is_empty(), "{skipped:?} {refused:?}");
        assert_eq!(enforced, 1);
    }

    /// The skip covers only its own rule: another rule on the layer keeps its own verdict (here a
    /// SAMEMASK end-of-line rule, refused), and only the skipped rule's index is listed.
    #[test]
    fn another_refused_rule_on_the_layer_is_still_refused() {
        let rules = format!("{}{}", eol("SPACING 0.09 ENDOFLINE 0.07 WITHIN 0.025 SAMEMASK ;"), eol("SPACING 0.09 ENDOFLINE 0.07 TOCONCAVECORNER ;"));
        let d = db("mixed", &rules, "");
        let (skipped, refused, _) = census(&d);
        assert_eq!(refused, ["M1: lef58_spacing_eol (2, SAMEMASK)"]);
        assert_eq!(skipped.iter().map(|s| (s.index, s.clause)).collect::<Vec<_>>(), [(1, "TOCONCAVECORNER")]);
    }

    /// Rule (reference io `setCutLayerProperties`, LAYER branch, + `io_parser_helper`): `SPACING s
    /// LAYER m CUTCLASS c CONCAVECORNER` is kept as a different-net LEF58 cut spacing rule with its
    /// second layer resolved by name after the technology (the metal above is defined later), so it
    /// is MODELLED — checked, neither refused nor skipped.
    #[test]
    fn the_cut_spacing_layer_concave_corner_form_is_modelled() {
        let d = db("cc", "", &eol("SPACING 0.07 LAYER M2 CUTCLASS CA CONCAVECORNER ;"));
        let (skipped, refused, _) = census(&d);
        assert!(skipped.is_empty() && refused.is_empty(), "{skipped:?} {refused:?}");
        let tech = crate::tech::read::tech(&d).expect("tech");
        let v1 = tech.layer_num("V1").expect("V1");
        let m2 = tech.layer_num("M2").expect("M2");
        assert_eq!(tech.layers[v1].cut_layer_spacing, [crate::tech::CutLayerSpacing { spacing: 70, second_layer: m2, class: 1, corner: crate::tech::CutCorner::Concave }]);
        // The second layer is the one the rule names, below the cut as well as above it.
        let d = db("cc-below", "", &eol("SPACING 0.07 LAYER M1 CUTCLASS CA CONCAVECORNER ;"));
        let tech = crate::tech::read::tech(&d).expect("tech");
        let m1 = tech.layer_num("M1").expect("M1");
        assert_eq!(tech.layers[v1].cut_layer_spacing, [crate::tech::CutLayerSpacing { spacing: 70, second_layer: m1, class: 1, corner: crate::tech::CutCorner::Concave }]);
    }

    /// Rule (reference io `setCutLayerProperties`, LAYER branch): `SPACING s LAYER m CUTCLASS c
    /// NONEOLCONVEXCORNER w [MINLENGTH l]` is kept as a different-net LEF58 cut spacing rule with
    /// the end-of-line width, and the minimum length only when MINLENGTH is given (it is read
    /// under NONEOLCONVEXCORNER alone), so it is MODELLED, in the technology's order beside a
    /// CONCAVECORNER rule.
    #[test]
    fn the_cut_spacing_layer_non_eol_convex_corner_form_is_modelled() {
        use crate::tech::{CutCorner, CutLayerSpacing};
        let rules = format!("{}{}{}", eol("SPACING 0.07 LAYER M2 CUTCLASS CA CONCAVECORNER ;"), eol("SPACING 0.07 LAYER M2 CUTCLASS CA NONEOLCONVEXCORNER 0.05 ;"), eol("SPACING 0.08 LAYER M1 CUTCLASS CA NONEOLCONVEXCORNER 0.06 MINLENGTH 0.03 ;"));
        let d = db("cv", "", &rules);
        let (skipped, refused, _) = census(&d);
        assert!(skipped.is_empty() && refused.is_empty(), "{skipped:?} {refused:?}");
        let tech = crate::tech::read::tech(&d).expect("tech");
        let (v1, m1, m2) = (tech.layer_num("V1").expect("V1"), tech.layer_num("M1").expect("M1"), tech.layer_num("M2").expect("M2"));
        assert_eq!(
            tech.layers[v1].cut_layer_spacing,
            [
                CutLayerSpacing { spacing: 70, second_layer: m2, class: 1, corner: CutCorner::Concave },
                CutLayerSpacing { spacing: 70, second_layer: m2, class: 1, corner: CutCorner::NonEolConvex { eol_width: 50, min_length: None } },
                CutLayerSpacing { spacing: 80, second_layer: m1, class: 1, corner: CutCorner::NonEolConvex { eol_width: 60, min_length: Some(30) } },
            ]
        );
    }

    /// Every other LEF58 cut spacing form stays refused, naming its first clause outside the
    /// modelled ones; a modelled rule beside it does not lift the refusal.
    #[test]
    fn other_lef58_cut_spacing_forms_stay_refused_naming_the_clause() {
        for (rule, clause) in [
            ("SPACING 0.07 SAMENET LAYER M2 CUTCLASS CA NONEOLCONVEXCORNER 0.05 ;", "SAMENET"),
            ("SPACING 0.07 LAYER M2 CUTCLASS CA CONCAVECORNER WIDTH 0.05 ENCLOSURE 0.01 EDGELENGTH 0.1 ;", "CONCAVECORNER WIDTH"),
            ("SPACING 0.07 LAYER M2 ;", "LAYER without CUTCLASS"),
            ("SPACING 0.07 SAMENET LAYER M2 ;", "LAYER without CUTCLASS"),
            ("SPACING 0.07 CENTERTOCENTER LAYER M2 CUTCLASS CA CONCAVECORNER ;", "CENTERTOCENTER"),
            ("SPACING 0.07 LAYER M2 CUTCLASS CA ;", "LAYER CUTCLASS without CONCAVECORNER or NONEOLCONVEXCORNER"),
            ("SPACING 0.07 ADJACENTCUTS 2 WITHIN 0.1 CUTCLASS CA ;", "ADJACENTCUTS"),
        ] {
            let rules = format!("{}{}", eol("SPACING 0.07 LAYER M2 CUTCLASS CA CONCAVECORNER ;"), eol(rule));
            let d = db("cc-other", "", &rules);
            let (_, refused, _) = census(&d);
            assert_eq!(refused, [format!("V1: lef58_cut_spacing (2, {clause})")], "{rule}");
        }
    }

    /// A rule the reference drops itself (FILLCONCAVECORNER, with its own warning) is the
    /// reference's skip, not one of ours.
    #[test]
    fn a_rule_the_reference_drops_is_not_listed_as_ours() {
        let d = db("fcc", &eol("SPACING 0.09 ENDOFLINE 0.07 WITHIN 0.025 FILLCONCAVECORNER 0.05 ;"), "");
        let (skipped, refused, enforced) = census(&d);
        assert!(skipped.is_empty() && refused.is_empty(), "{skipped:?} {refused:?}");
        assert_eq!(enforced, 0);
    }
}

#[cfg(test)]
mod lef58_not_checked_tests {
    //! Rule (odb lefin `layer()`): every layer PROPERTY is stored as raw text, and one the reader
    //! has no parser for (by layer type) gets only ODB-0388 INFO — no rule object, so routing
    //! never checks it, and the reference is silent. We LIST these (informational), never refuse.
    use super::{lef58_not_checked, unmodelled_rules, UnparsedProperty};
    use vyges_opendb::Db;

    fn db(lef: &str) -> Db {
        let path = std::env::temp_dir().join(format!("vyges-drt-unparsed-{}-{}.lef", std::process::id(), lef.len()));
        std::fs::write(&path, lef).expect("write lef");
        let mut db = Db::new();
        db.read_lef(path.to_str().expect("utf-8 path")).expect("read lef");
        let _ = std::fs::remove_file(&path);
        db
    }

    const LEF: &str = "VERSION 5.8 ;\nUNITS\n  DATABASE MICRONS 1000 ;\nEND UNITS\nMANUFACTURINGGRID 0.005 ;\n\
LAYER M1\n  TYPE ROUTING ;\n  DIRECTION HORIZONTAL ;\n  PITCH 0.12 ;\n  WIDTH 0.04 ;\n  SPACING 0.04 ;\n\
  PROPERTY LEF58_SPACING \"SPACING 0.09 ENDOFLINE 0.07 WITHIN 0.025 ;\" ;\n\
  PROPERTY LEF58_EOLENCLOSURE \"EOLENCLOSURE 0.05 ABOVE 0.01 ;\" ;\n\
  PROPERTY LEF58_PRLTWOSIDESSPACING \"PRLTWOSIDESSPACING 0.05 ;\" ;\nEND M1\n\
LAYER V1\n  TYPE CUT ;\n  SPACING 0.05 ;\n  WIDTH 0.04 ;\n\
  PROPERTY LEF58_CUTCLASS \"CUTCLASS CA WIDTH 0.04 ;\" ;\n\
  PROPERTY LEF58_ENCLOSUREEDGE \"ENCLOSUREEDGE CUTCLASS CA ABOVE 0.01 OPPOSITE NOCONCAVECORNER 0.05 ;\" ;\n\
  PROPERTY LEF58_ENCLOSUREEDGE \"ENCLOSUREEDGE CUTCLASS CA BELOW 0.02 ;\" ;\n\
  PROPERTY LEF58_VIACLUSTER \"VIACLUSTER 3 WITHIN 0.1 ; VIACLUSTER 4 WITHIN 0.2 ;\" ;\nEND V1\n\
LAYER M2\n  TYPE ROUTING ;\n  DIRECTION VERTICAL ;\n  PITCH 0.12 ;\n  WIDTH 0.04 ;\n  SPACING 0.04 ;\nEND M2\nEND LIBRARY\n";

    /// Rule (odb `dbStringProperty::create`): ONE string property per name and layer, a repeated
    /// PROPERTY appended to it with a space — so V1's two ENCLOSUREEDGE properties read as one
    /// property of two statements, as do two statements inside one PROPERTY string.
    #[test]
    fn unparsed_lef58_properties_are_listed_per_layer_and_kind_and_never_refused() {
        let d = db(LEF);
        let u = |layer: &str, kind: &str, statements| UnparsedProperty { layer: layer.into(), kind: kind.into(), statements };
        assert_eq!(
            lef58_not_checked(&d),
            [u("M1", "LEF58_EOLENCLOSURE", 1), u("M1", "LEF58_PRLTWOSIDESSPACING", 1), u("V1", "LEF58_ENCLOSUREEDGE", 2), u("V1", "LEF58_VIACLUSTER", 2)],
            "the parsed LEF58_SPACING and LEF58_CUTCLASS are not listed"
        );
        let kept: Vec<String> = d.layer_string_properties("V1").into_iter().filter(|(k, _)| k == "LEF58_ENCLOSUREEDGE").map(|(_, v)| v).collect();
        assert_eq!(kept.len(), 1, "one string property per name: {kept:?}");
        assert!(kept[0].contains("ABOVE 0.01") && kept[0].contains("BELOW 0.02"), "the repeat is appended: {kept:?}");
        let tech = crate::tech::read::tech(&d).expect("tech");
        assert!(unmodelled_rules(&d, &tech).is_empty(), "informational only: nothing refused");
        let n = u("V1", "LEF58_VIACLUSTER", 2).notice();
        assert!(n.contains("layer V1: LEF58_VIACLUSTER (2 statement(s))") && n.contains("NOT checked during routing"), "{n}");
    }

    /// A technology without unparsed properties lists none.
    #[test]
    fn a_technology_without_unparsed_properties_lists_none() {
        let plain: String = LEF.lines().filter(|l| !l.contains("EOLENCLOSURE") && !l.contains("PRLTWOSIDES") && !l.contains("ENCLOSUREEDGE") && !l.contains("VIACLUSTER")).map(|l| format!("{l}\n")).collect();
        assert!(lef58_not_checked(&db(&plain)).is_empty());
    }
}
