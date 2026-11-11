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

/// The design's vias (DEF VIAS) that io adds to the technology (`setVias`: one on a layer the
/// router does not hold, or on the masterslice, is skipped — `held`), in the database's order,
/// with the cut class the max-spacing pass reads: from the parameters' cut size when the via has
/// parameters (every cut is that size), else unknown.
pub fn block_vias(db: &Db, tech: &Tech, held: &dyn Fn(&str) -> bool) -> Vec<crate::dr::max_spacing::BlockVia> {
    let mut out = Vec::new();
    for v in db.block_get_vias() {
        let (top, bottom) = (db.via_get_top_layer(&v), db.via_get_bottom_layer(&v));
        if !held(&top) || !held(&bottom) {
            continue;
        }
        let Some(cut) = tech.layer_num(&bottom).map(|b| b + 1).filter(|&c| c < tech.layers.len() && tech.layers[c].kind == LayerKind::Cut) else {
            out.push(crate::dr::max_spacing::BlockVia { name: v.clone(), cut: usize::MAX, cut_class: Err(()) });
            continue;
        };
        let cut_class = if db.via_has_params(&v) {
            let (x, y) = (db.via_params_get_x_cut_size(&v), db.via_params_get_y_cut_size(&v));
            Ok(tech.layers[cut].cut_class_of(x.min(y), x.max(y)).checked_sub(1))
        } else {
            Err(())
        };
        out.push(crate::dr::max_spacing::BlockVia { name: v, cut, cut_class });
    }
    out
}

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
    for (num, l) in tech.layers.iter().enumerate() {
        if l.kind == LayerKind::Placeholder {
            continue;
        }
        // Families the reference's io translates that the database census does not list, counted
        // here from the generated accessors — only the rules io KEEPS (it drops the rest with a
        // warning, and sky130's cut layers carry two LEF58 enclosure rules without a CUTCLASS that
        // it drops that way) — and refused (none is modelled). SPACINGTABLE ORTHOGONAL is
        // modelled (the check, its only consumer) and no longer listed here; neither is
        // LEF58_MAXSPACING, whose pass refuses at run time when it reaches a stage not ported
        // (`crate::dr::max_spacing`).
        let name = l.name.as_str();
        let kept = |n: usize, keeps: &dyn Fn(usize) -> bool| (0..n).filter(|&k| keeps(k)).count();
        let untracked = [
            ("lef58_two_wires_forbidden_spacing", db.num_layer_get_tech_layer_two_wires_forbidden_spc_rules(name)),
            ("lef58_width_table_orthogonal", kept(db.num_layer_get_tech_layer_width_table_rules(name), &|k| db.widthtablerule_is_orthogonal(name, k))),
            ("lef58_wrong_dir_spacing", kept(db.num_layer_get_tech_layer_wrong_dir_spacing_rules(name), &|k| !db.wrongdirspacingrule_is_length_valid(name, k) && db.wrongdirspacingrule_get_prl_length(name, k) >= 0)),
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
                // Modelled for the cut-to-corner forms (`SPACING s LAYER l CUTCLASS c
                // CONCAVECORNER`: the check, the only consumer reached — the via-to-via prep that
                // also reads it runs only for one named process node, an option this router does
                // not take; `… NONEOLCONVEXCORNER w [MINLENGTH m]`: the check, its only consumer)
                // and for ADJACENTCUTS as io keeps it (the check, its only consumer; the forms io
                // skips are skipped, see `skipped_rules`); the first rule outside them names the
                // refusal.
                // A LAYER rule with no CUTCLASS, or a class but neither corner, is skipped with a
                // warning, SAMENET or not (divergence D6: the reference fixed for OpenROAD issue
                // #12187 drops it, so it never reaches the same-net list).
                let classes = &l.cut_classes;
                if let Some(clause) = (0..n).find_map(|k| crate::tech::read::cut_layer_rule(db, &l.name, k, classes).err()) {
                    out.push(format!("{}: {family} ({n}, {clause})", l.name));
                }
                continue;
            }
            if family == "lef58_min_step" {
                // LEF58_MINSTEP: the check (`checkMetalShape_lef58MinStep`, its NOBETWEENEOL and
                // MINADJACENTLENGTH forms, with NOADJACENTEOL and EXCEPTRECTANGLE) and the router's
                // minimum-step patch pass, its only consumers; a rule with MAXEDGES above 1 is
                // skipped with a warning (`skipped_rules`); the first rule the check does not read
                // as written names the refusal.
                if let Some(clause) = (0..n).find_map(|k| crate::tech::read::lef58_min_step_unmodelled(db, &l.name, k)) {
                    out.push(format!("{}: {family} ({n}, {clause})", l.name));
                }
                continue;
            }
            if family == "lef58_min_cut" {
                // Modelled through io's conversion into minimum-cut rules (the check, the maze cost
                // around vias and macro pins, the via-to-via prep, MTSAFEDIST): AREA,
                // SAMEMETALOVERLAP and FULLYENCLOSED rules are skipped and a rule asking for one
                // cut or none is dropped, as the reference skips and drops them; a rule written
                // per cut class takes its count from the default vias (divergence D9), refused at
                // run time only where its default via has no cut class.
                continue;
            }
            if family == "lef58_cut_spacing_table" {
                // Modelled for the tables on the layer itself: different-net (check, maze cost,
                // via-to-via and the default `prep_cutSpcTbl` sets) and SAMENET (check, via-to-via);
                // and for the default inter-layer table (check, from either layer; maze cost
                // between the two layers' vias, with the default `prep_cutSpcTbl` sets); the reader
                // names any other kind.
                match crate::tech::read::cut_spacing_tables(db, &l.name, num, &l.cut_classes) {
                    Err(clause) => out.push(format!("{}: {family} ({n}, {clause})", l.name)),
                    Ok(t) if t.inter.is_some() && l.cut_table_inter.is_none() => out.push(format!("{}: {family} ({n}, a LAYER (inter-layer) cut spacing table to a layer not read as a cut layer)", l.name)),
                    Ok(_) => {}
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
                // SPACINGTABLE TWOWIDTHS: the layer's minimum spacing rule, read by every consumer
                // the parallel-run table has (the check, the maze and track costs, the via-to-via
                // and line prep), each on both widths.
                "two_widths_spacing" => true,
                // LEF58_AREA: the check (`checkMetalShape_lef58Area`, marking) and the router's patch
                // pass (`checkMetalShape_addPatch`), its only consumers; the forms io skips are
                // skipped with a warning (`skipped_rules`), and io keeps nothing else.
                "lef58_area" => true,
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

/// The upstream issue for the LEF58 cut spacing LAYER rules [`skipped_rules`] lists: io keeps a
/// rule with no CUTCLASS, or a CUTCLASS but no corner, and the check never acts on it.
pub const SKIPPED_LAYER_UPSTREAM_ISSUE: &str = "OpenROAD issue #12187";

/// The upstream issue for the LEF58 cut spacing ADJACENTCUTS rules with two WITHIN values
/// [`skipped_rules`] lists: io reads only the first value, as the upper bound.
pub const SKIPPED_ADJ_TWO_WITHIN_UPSTREAM_ISSUE: &str = "OpenROAD issue #12202";

/// A rule the technology carries that routing neither checks nor refuses: it is SKIPPED, with a
/// warning at tech load and an entry in the run's report.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SkippedRule {
    pub layer: String,
    /// The census family (`lef58_spacing_eol`, `lef58_min_cut`).
    pub family: &'static str,
    /// The rule's index among the layer's rules of that family (from 0).
    pub index: usize,
    /// The clause that makes it skipped.
    pub clause: &'static str,
}

impl SkippedRule {
    /// The warning logged at tech load.
    pub fn warning(&self) -> String {
        match (self.family, self.clause) {
            ("lef58_cut_spacing", clause) if crate::tech::read::LAYER_UNCHECKED.contains(&clause) => format!(
                "layer {}: LEF58_SPACING cut spacing rule {} is a {clause} rule. The reference checks a LAYER rule only for a cut class with CONCAVECORNER or NONEOLCONVEXCORNER, so it never checks this one; the rule is SKIPPED and NOT checked during routing, as the reference fixed for {} skips it, so sign-off DRC must cover it.",
                self.layer, self.index, SKIPPED_LAYER_UPSTREAM_ISSUE
            ),
            ("lef58_cut_spacing", crate::tech::read::ADJ_TWO_WITHIN) => format!(
                "layer {}: LEF58_SPACING cut spacing rule {} is an ADJACENTCUTS rule with two WITHIN values, which this router does not model; the rule is SKIPPED and NOT checked during routing, so sign-off DRC must cover it (the unpatched reference checks it with the first value only; this router skips it as the reference fixed for {} does).",
                self.layer, self.index, SKIPPED_ADJ_TWO_WITHIN_UPSTREAM_ISSUE
            ),
            ("lef58_cut_spacing", clause) => format!(
                "layer {}: LEF58_SPACING cut spacing rule {} uses {clause}, which neither the reference (it warns and skips the rule) nor this router supports; the rule is SKIPPED and NOT checked during routing, so sign-off DRC must cover it.",
                self.layer, self.index
            ),
            ("lef58_area", clause) => format!(
                "layer {}: LEF58_AREA rule {} uses {clause}, which neither the reference (it warns and skips the rule) nor this router supports; the rule is SKIPPED and NOT checked during routing, so sign-off DRC must cover it.",
                self.layer, self.index
            ),
            ("lef58_min_step", clause) => format!(
                "layer {}: LEF58_MINSTEP rule {} uses {clause}, which neither the reference (it warns and skips the rule) nor this router supports; the rule is SKIPPED and NOT checked during routing, so sign-off DRC must cover it.",
                self.layer, self.index
            ),
            ("lef58_min_cut", clause) => format!(
                "layer {}: LEF58_MINIMUMCUT rule {} uses {clause}, which neither the reference (it warns and skips the rule) nor this router supports; the rule is SKIPPED and NOT checked during routing, so sign-off DRC must cover it.",
                self.layer, self.index
            ),
            _ => format!(
                "layer {}: LEF58_SPACING end-of-line rule {} uses {}, which this router does not model; the rule is SKIPPED and NOT checked during routing, so sign-off DRC must cover it (the unpatched reference reads this form as a plain end-of-line rule with WITHIN 0; this router skips it as the reference fixed for {} does).",
                self.layer, self.index, self.clause, SKIPPED_EOL_UPSTREAM_ISSUE
            ),
        }
    }

    /// The upstream issue that explains the skip, when one is filed.
    pub fn upstream(&self) -> Option<&'static str> {
        match (self.family, self.clause) {
            ("lef58_spacing_eol", _) => Some(SKIPPED_EOL_UPSTREAM_ISSUE),
            ("lef58_cut_spacing", clause) if crate::tech::read::LAYER_UNCHECKED.contains(&clause) => Some(SKIPPED_LAYER_UPSTREAM_ISSUE),
            ("lef58_cut_spacing", crate::tech::read::ADJ_TWO_WITHIN) => Some(SKIPPED_ADJ_TWO_WITHIN_UPSTREAM_ISSUE),
            _ => None,
        }
    }
}

/// The rules routing skips with a warning instead of refusing, layer by layer in the
/// technology's order: each LEF58 end-of-line rule with TOCONCAVECORNER or TONOTCHLENGTH
/// ([`crate::tech::read::lef58_eol_skipped_clause`]), then each LEF58 minimum-cut rule the
/// reference skips ([`crate::tech::read::lef58_min_cut_skipped_clause`]: AREA,
/// SAMEMETALOVERLAP, FULLYENCLOSED), then each LEF58_AREA rule io
/// skips ([`crate::tech::read::lef58_area_skipped_clause`]: EXCEPTMINWIDTH, EXCEPTEDGELENGTH,
/// EXCEPTMINSIZE, EXCEPTSTEP, MASK, LAYER), then each LEF58_MINSTEP rule io skips
/// ([`crate::tech::read::lef58_min_step_skipped`]: MAXEDGES above 1), then on each cut layer each LEF58
/// cut spacing ADJACENTCUTS rule io skips ([`crate::tech::read::adjacent_cuts_rule`]:
/// EXACTALIGNED, EXCEPTSAMEPGNET, TO ALL, SIDEPARALLELOVERLAP, SAMEMASK) and each LAYER rule with
/// no CUTCLASS (kept by io, never checked by the reference's GC) — routed as the
/// reference routes it, without the rule, but warned about. ⛔ Not enforced and not emulated: the
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
        for k in 0..db.num_layer_get_tech_layer_min_cut_rules(&l.name) {
            if let Some(clause) = crate::tech::read::lef58_min_cut_skipped_clause(db, &l.name, k) {
                out.push(SkippedRule { layer: l.name.clone(), family: "lef58_min_cut", index: k, clause });
            }
        }
        for k in 0..db.num_layer_get_tech_layer_area_rules(&l.name) {
            if let Some(clause) = crate::tech::read::lef58_area_skipped_clause(db, &l.name, k) {
                out.push(SkippedRule { layer: l.name.clone(), family: "lef58_area", index: k, clause });
            }
        }
        for k in 0..db.num_layer_get_tech_layer_min_step_rules(&l.name) {
            if crate::tech::read::lef58_min_step_skipped(db, &l.name, k) {
                let clause = "MAXEDGES above 1";
                out.push(SkippedRule { layer: l.name.clone(), family: "lef58_min_step", index: k, clause });
            }
        }
    }
    for l in tech.layers.iter().filter(|l| l.kind == LayerKind::Cut) {
        for k in 0..db.num_layer_get_tech_layer_cut_spacing_rules(&l.name) {
            if let Ok(crate::tech::read::Lef58CutRule::Skipped(clause)) = crate::tech::read::cut_layer_rule(db, &l.name, k, &l.cut_classes) {
                out.push(SkippedRule { layer: l.name.clone(), family: "lef58_cut_spacing", index: k, clause });
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
        out.push(Marker { rule, layer, bbox: Rect { xl, yl, xh, yh }, owners, victim: None, aggressor: None, second_victim: None });
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
    //! ⛔ A DIVERGENCE from the reference by decision (drt divergence register D10). Upstream rule
    //! (io `setRoutingLayerProperties`): a LEF58 end-of-line rule with TOCONCAVECORNER or
    //! TONOTCHLENGTH is never tested for either branch and is checked as a plain end-of-line rule
    //! with WITHIN 0 (MINADJACENTLENGTH dropped), flagging line ends that face straight edges
    //! (upstream OpenROAD issue #12174). Our rule, the reference patched with
    //! vyges-openroad-patches upstream-fix 0010: the rule is SKIPPED with a warning (DRT-357 /
    //! DRT-358 in the log, an entry in skipped_rules) — not enforced, not emulated, not refused;
    //! the rule as written has no oracle, so it is left to sign-off DRC. Every other refusal stands.
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

    /// Rule (reference io `setCutLayerProperties`, its LEF58_MAXSPACING loop): a rule with a
    /// CUTCLASS of the layer is kept (max, class index), one with no CUTCLASS is skipped with
    /// DRT-350 (io would also drop one naming a class the layer lacks, but the database's parser
    /// already rejects such a property whole). Nothing is refused up front: the pass refuses at
    /// run time only if it reaches a stage that is not ported.
    #[test]
    fn a_max_spacing_rule_is_kept_by_class_skipped_without_one_and_never_refused_up_front() {
        let d = db("maxspc", "", "  PROPERTY LEF58_MAXSPACING \"MAXSPACING 0.4 CUTCLASS CA ;\" ;\n  PROPERTY LEF58_MAXSPACING \"MAXSPACING 0.5 ;\" ;\n");
        let (skipped, refused, _) = census(&d);
        assert!(refused.is_empty() && skipped.is_empty(), "{refused:?} {skipped:?}");
        let tech = crate::tech::read::tech(&d).expect("tech");
        let v1 = &tech.layers[tech.layer_num("V1").expect("V1")];
        assert_eq!(v1.lef58_max_spacing, [crate::tech::MaxSpacingRule { max: 400, cut_class: 0 }]);
        assert_eq!(crate::tech::read::max_spacing_skips(&d, "V1"), [(350, "LEF58_MAXSPACING with no CUTCLASS is not supported. Skipping for layer V1".to_string())]);
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

    /// Rule (the reference patched with upstream-fix 0010, io `setRoutingLayerProperties`, after
    /// the DRT-400/401/403 skips): one warning per skipped rule, in the layer's rule order —
    /// DRT-357 for TOCONCAVECORNER, DRT-358 for TONOTCHLENGTH; a plain rule warns nothing. The
    /// unpatched reference warns nothing and checks both as WITHIN 0 (upstream OpenROAD #12174).
    #[test]
    fn the_skipped_end_of_line_forms_are_warned_as_the_patched_reference_warns() {
        let rules = format!(
            "{}{}{}",
            eol("SPACING 0.09 ENDOFLINE 0.07 TONOTCHLENGTH 0.1 ;"),
            eol("SPACING 0.09 ENDOFLINE 0.07 WITHIN 0 ;"),
            eol("SPACING 0.09 ENDOFLINE 0.07 TOCONCAVECORNER MINADJACENTLENGTH 0.1 0.12 ;")
        );
        let d = db("w357", &rules, "");
        assert_eq!(
            crate::tech::read::lef58_eol_skip_warnings(&d, "M1"),
            [
                (358, "Unsupported LEF58_SPACING rule with option TONOTCHLENGTH for layer M1.".to_string()),
                (357, "Unsupported LEF58_SPACING rule with option TOCONCAVECORNER for layer M1.".to_string()),
            ]
        );
        assert!(crate::tech::read::lef58_eol_skip_warnings(&d, "M2").is_empty());
    }

    #[test]
    fn a_to_notch_length_rule_is_skipped_too() {
        let d = db("tnl", &eol("SPACING 0.09 ENDOFLINE 0.07 TONOTCHLENGTH 0.1 ;"), "");
        let (skipped, refused, enforced) = census(&d);
        assert!(refused.is_empty(), "{refused:?}");
        assert_eq!(enforced, 0);
        assert_eq!(skipped.iter().map(|s| s.clause).collect::<Vec<_>>(), ["TONOTCHLENGTH"]);
    }

    /// ⛔ DIVERGENCE by decision (drt divergence register D8, upstream OpenROAD issue #12181,
    /// vyges-openroad-patches upstream-fix 0001). Upstream rule (io `setRoutingLayerProperties`):
    /// MINLENGTH / MAXLENGTH is kept with `isTwoEdgesValid()` (PARALLELEDGE's TWOEDGES) as its
    /// TWOSIDES. Our rule (the patched reference's): TWOSIDES as written (`isTwoSidesValid()`).
    /// The rule is checked, not refused; MINLENGTH wins when the database holds both.
    #[test]
    fn min_and_max_length_are_checked_with_twosides_as_written() {
        use crate::tech::EolLength;
        let read = |name: &str, rule: &str| {
            let d = db(name, &eol(rule), "");
            let (skipped, refused, enforced) = census(&d);
            assert!(skipped.is_empty() && refused.is_empty() && enforced == 1, "{rule}: {skipped:?} {refused:?} {enforced}");
            crate::tech::read::tech(&d).expect("tech").layers.iter().find(|l| l.name == "M1").expect("M1").lef58_eol[0].length
        };
        assert_eq!(read("min2", "SPACING 0.09 ENDOFLINE 0.07 WITHIN 0.025 MINLENGTH 0.1 TWOSIDES ;"), Some(EolLength { max: false, length: 100, two_sides: true }));
        assert_eq!(read("min1", "SPACING 0.09 ENDOFLINE 0.07 WITHIN 0.025 MINLENGTH 0.1 ;"), Some(EolLength { max: false, length: 100, two_sides: false }));
        assert_eq!(read("max1", "SPACING 0.09 ENDOFLINE 0.07 WITHIN 0.025 MAXLENGTH 0.2 ;"), Some(EolLength { max: true, length: 200, two_sides: false }));
        // TWOEDGES alone (the reference's TWOSIDES source) does not make MINLENGTH two-sided.
        assert_eq!(read("par2", "SPACING 0.09 ENDOFLINE 0.07 WITHIN 0.025 MINLENGTH 0.1 PARALLELEDGE 0.1 WITHIN 0.1 TWOEDGES ;"), Some(EolLength { max: false, length: 100, two_sides: false }));
        assert_eq!(read("none", "SPACING 0.09 ENDOFLINE 0.07 WITHIN 0.025 ;"), None);
    }

    /// ⛔ DIVERGENCE by decision (drt divergence register D8, upstream OpenROAD issue #12182,
    /// vyges-openroad-patches upstream-fix 0002). Upstream rule (`getEolWithin`): a SAMEMASK
    /// end-of-line rule's WITHIN reads 0. Our rule (the patched reference's): WITHIN as written.
    /// The rule is checked (mask-blind, as the reference checks it), not refused.
    #[test]
    fn a_same_mask_end_of_line_rule_is_checked_with_its_within_as_written() {
        let d = db("samemask", &eol("SPACING 0.09 ENDOFLINE 0.07 WITHIN 0.025 SAMEMASK ;"), "");
        let (skipped, refused, enforced) = census(&d);
        assert!(skipped.is_empty() && refused.is_empty() && enforced == 1, "{skipped:?} {refused:?}");
        let tech = crate::tech::read::tech(&d).expect("tech");
        assert_eq!(tech.layers.iter().find(|l| l.name == "M1").expect("M1").lef58_eol[0].within, 25);
    }

    /// Rule (reference io `setRoutingLayerProperties`, pin 4155ac31): ENDTOEND's EXTENSION is
    /// kept on the end-to-end constraint (`setExtension`, with or without its wrong-direction
    /// value, which drt never reads), so the rule is checked, neither refused nor skipped.
    #[test]
    fn an_end_to_end_extension_is_modelled() {
        let d = db("ext", &eol("SPACING 0.09 ENDOFLINE 0.07 WITHIN 0.025 ENDTOEND 0.08 EXTENSION 0.01 ;"), "");
        let (skipped, refused, enforced) = census(&d);
        assert!(skipped.is_empty() && refused.is_empty(), "{skipped:?} {refused:?}");
        assert_eq!(enforced, 1);
        let r = crate::tech::read::lef58_eol_rule(&d, "M1", 0).expect("modelled").expect("kept");
        assert_eq!((r.end_to_end, r.extension), (Some(80), Some(10)));
    }

    /// Rule (LEF/DEF reference, LEF58_SPACING ENDOFLINE ... OPPOSITEWIDTH ow WITHIN ...): the rule
    /// applies only when the wire beyond the line end spans less than ow. ⛔ DIVERGENCE (register
    /// D13): the reference's io stores ow (`setOppositeWidth`) and nothing reads it; we keep it and
    /// check it (upstream-fix 0015), so the rule is checked, neither refused nor skipped.
    #[test]
    fn an_opposite_width_is_modelled() {
        let d = db("ow", &eol("SPACING 0.09 ENDOFLINE 0.07 OPPOSITEWIDTH 0.03 WITHIN 0.025 ;"), "");
        let (skipped, refused, enforced) = census(&d);
        assert!(skipped.is_empty() && refused.is_empty(), "{skipped:?} {refused:?}");
        assert_eq!(enforced, 1);
        let r = crate::tech::read::lef58_eol_rule(&d, "M1", 0).expect("modelled").expect("kept");
        assert_eq!((r.opposite_width, r.within), (Some(30), 25));
    }

    /// The control: the same rule written as a plain end-of-line rule is checked, not skipped.
    #[test]
    fn a_plain_end_of_line_rule_is_checked_not_skipped() {
        let d = db("w0", &eol("SPACING 0.09 ENDOFLINE 0.07 WITHIN 0 ;"), "");
        let (skipped, refused, enforced) = census(&d);
        assert!(skipped.is_empty() && refused.is_empty(), "{skipped:?} {refused:?}");
        assert_eq!(enforced, 1);
    }

    /// The skip covers only its own rule: another rule on the layer keeps its own verdict (here an
    /// end-of-line rule with ENDTOEND OTHERENDWIDTH, refused), and only the skipped rule's index is listed.
    #[test]
    fn another_refused_rule_on_the_layer_is_still_refused() {
        let rules = format!("{}{}", eol("SPACING 0.09 ENDOFLINE 0.07 WITHIN 0.025 ENDTOEND 0.08 OTHERENDWIDTH 0.05 ;"), eol("SPACING 0.09 ENDOFLINE 0.07 TOCONCAVECORNER ;"));
        let d = db("mixed", &rules, "");
        let (skipped, refused, _) = census(&d);
        assert_eq!(refused, ["M1: lef58_spacing_eol (2, ENDTOEND OTHERENDWIDTH)"]);
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
        assert_eq!(tech.layers[v1].lef58_cut_spacing, [crate::tech::Lef58CutSpacing::Layer(crate::tech::CutLayerSpacing { spacing: 70, second_layer: m2, class: 1, corner: crate::tech::CutCorner::Concave })]);
        // The second layer is the one the rule names, below the cut as well as above it.
        let d = db("cc-below", "", &eol("SPACING 0.07 LAYER M1 CUTCLASS CA CONCAVECORNER ;"));
        let tech = crate::tech::read::tech(&d).expect("tech");
        let m1 = tech.layer_num("M1").expect("M1");
        assert_eq!(tech.layers[v1].lef58_cut_spacing, [crate::tech::Lef58CutSpacing::Layer(crate::tech::CutLayerSpacing { spacing: 70, second_layer: m1, class: 1, corner: crate::tech::CutCorner::Concave })]);
    }

    /// Rule (reference io `setRoutingLayerProperties`): a SPACINGTABLE TWOWIDTHS is the layer's
    /// minimum spacing rule (set after the parallel-run table, so it is the one kept on a layer
    /// with both), each row's PRL -1 when the row gives none; MODELLED, neither refused nor skipped.
    #[test]
    fn the_two_widths_spacing_table_is_modelled() {
        let tw = "  SPACINGTABLE\n    TWOWIDTHS\n      WIDTH 0.00 0.04 0.06\n      WIDTH 0.10 PRL 0.20 0.06 0.08 ;\n";
        let d = db("tw", tw, "");
        let (skipped, refused, _) = census(&d);
        assert!(skipped.is_empty() && refused.is_empty(), "{skipped:?} {refused:?}");
        let tech = crate::tech::read::tech(&d).expect("tech");
        let m1 = tech.layer_num("M1").expect("M1");
        let want = crate::tech::TwoWidthsTable { widths: vec![0, 100], prls: vec![-1, 200], values: vec![vec![40, 60], vec![60, 80]] };
        assert_eq!(tech.layers[m1].spacing, Some(crate::tech::MinSpacing::TwoWidths(want)));
        let m2 = tech.layer_num("M2").expect("M2");
        assert!(matches!(tech.layers[m2].spacing, Some(crate::tech::MinSpacing::Prl(_))));
    }

    /// Rule (reference io `setCutLayerProperties`, the LEF58_SPACINGTABLE loop): a SAMENET table
    /// on the cut layer itself is kept beside the different-net one (each kind's last rule kept)
    /// and MODELLED; a SAMEMETAL table (tested first, as io tests it) and an inter-layer table stay
    /// refused, naming the kind.
    #[test]
    fn a_same_net_cut_spacing_table_is_modelled_beside_the_different_net_one() {
        let tbl = |opt: &str, spc: &str| format!("  PROPERTY LEF58_SPACINGTABLE \"SPACINGTABLE DEFAULT {spc} {opt}CUTCLASS CA CA - - ;\" ;\n");
        let d = db("sn", "", &format!("{}{}", tbl("", "0.06"), tbl("SAMENET ", "0.08")));
        let (skipped, refused, _) = census(&d);
        assert!(skipped.is_empty() && refused.is_empty(), "{skipped:?} {refused:?}");
        let tech = crate::tech::read::tech(&d).expect("tech");
        let v1 = &tech.layers[tech.layer_num("V1").expect("V1")];
        assert_eq!(v1.cut_table.as_ref().map(|t| t.default_spacing), Some((60, 60)));
        assert_eq!(v1.cut_table_same_net.as_ref().map(|t| t.default_spacing), Some((80, 80)));
        let d = db("sm", "", &format!("{}{}", tbl("", "0.06"), tbl("SAMEMETAL ", "0.08")));
        let (_, refused, _) = census(&d);
        assert_eq!(refused, ["V1: lef58_cut_spacing_table (2, a SAMEMETAL cut spacing table)"]);
    }

    /// Two cut layers, V1 (class CA) and V2 (class CB, and CA too), and `v2` extra lines in V2.
    fn db2(name: &str, v2: &str) -> Db {
        let lef = format!(
            "VERSION 5.8 ;\nUNITS\n  DATABASE MICRONS 1000 ;\nEND UNITS\nMANUFACTURINGGRID 0.005 ;\n\
             LAYER M1\n  TYPE ROUTING ;\n  DIRECTION HORIZONTAL ;\n  PITCH 0.12 ;\n  WIDTH 0.04 ;\n  SPACING 0.04 ;\nEND M1\n\
             LAYER V1\n  TYPE CUT ;\n  SPACING 0.05 ;\n  WIDTH 0.04 ;\n  PROPERTY LEF58_CUTCLASS \"CUTCLASS CA WIDTH 0.04 ;\" ;\nEND V1\n\
             LAYER M2\n  TYPE ROUTING ;\n  DIRECTION VERTICAL ;\n  PITCH 0.12 ;\n  WIDTH 0.04 ;\n  SPACING 0.04 ;\nEND M2\n\
             LAYER V2\n  TYPE CUT ;\n  SPACING 0.05 ;\n  WIDTH 0.04 ;\n  PROPERTY LEF58_CUTCLASS \"CUTCLASS CB WIDTH 0.04 ; CUTCLASS CA WIDTH 0.04 LENGTH 0.08 ;\" ;\n{v2}END V2\n\
             LAYER M3\n  TYPE ROUTING ;\n  DIRECTION HORIZONTAL ;\n  PITCH 0.12 ;\n  WIDTH 0.04 ;\n  SPACING 0.04 ;\nEND M3\nEND LIBRARY\n"
        );
        let path = std::env::temp_dir().join(format!("vyges-drt-inter-{}-{name}.lef", std::process::id()));
        std::fs::write(&path, lef).expect("write lef");
        let mut db = Db::new();
        db.read_lef(path.to_str().expect("utf-8 path")).expect("read lef");
        let _ = std::fs::remove_file(&path);
        db
    }

    /// Rule (reference io `setCutLayerProperties`, the LEF58_SPACINGTABLE loop): a LAYER table that
    /// is neither SAMENET nor SAMEMETAL is the layer's DEFAULT inter-layer table (the last such
    /// kept), its second layer resolved by name, its lookups by class NAME over both layers'
    /// classes; NOSTACK kept; MODELLED. An inter-layer SAMENET or SAMEMETAL table stays refused.
    #[test]
    fn a_default_inter_layer_cut_spacing_table_is_modelled() {
        let tbl = |opt: &str, body: &str| format!("  PROPERTY LEF58_SPACINGTABLE \"SPACINGTABLE {opt}{body}\" ;\n");
        let d = db2("it", &format!("{}{}", tbl("DEFAULT 0.03 LAYER V1 ", "CUTCLASS CB CA 0.05 0.06 ;"), tbl("DEFAULT 0.04 LAYER V1 NOSTACK PRL 0.02 ", "CUTCLASS CB CA 0.05 0.08 ;")));
        let (skipped, refused, _) = census(&d);
        assert!(skipped.is_empty() && refused.is_empty(), "{skipped:?} {refused:?}");
        let tech = crate::tech::read::tech(&d).expect("tech");
        let (v1, v2) = (tech.layer_num("V1").expect("V1"), tech.layer_num("V2").expect("V2"));
        let it = tech.layers[v2].cut_table_inter.as_ref().expect("the inter-layer table");
        assert_eq!((it.second_layer, it.no_stack), (v1, true), "the LAST table is kept");
        assert_eq!(it.table.names, ["", "CB", "CA"], "both layers' class names, V2's first, CA once");
        let (cb, ca_v1) = (it.table.class_index(&tech.layers[v2], 1), it.table.class_index(&tech.layers[v1], 1));
        assert_eq!((cb, ca_v1), (1, 2));
        assert_eq!(it.table.get(cb, false, ca_v1, false), (50, 80), "V2's CB against V1's CA, by name");
        assert_eq!(it.table.get(0, false, 0, false), (40, 40), "no class: the DEFAULT");
        assert_eq!(it.table.prl_entry[it.table.pair(cb, ca_v1)], 20);
        assert!(tech.layers[v2].cut_table.is_none() && tech.layers[v1].cut_table_inter.is_none());
        for (opt, kind) in [("SAMENET LAYER V1 ", "SAMENET"), ("SAMEMETAL LAYER V1 ", "SAMEMETAL")] {
            let d = db2(kind, &tbl(opt, "CUTCLASS CB CA 0.05 0.06 ;"));
            let (_, refused, _) = census(&d);
            assert_eq!(refused, [format!("V2: lef58_cut_spacing_table (1, a {kind} LAYER (inter-layer) cut spacing table)")]);
        }
    }

    /// Rule (reference io, the same loop): a LAYER table on layer NUMBER 1 is skipped before
    /// anything else is read of it (the first cut layer is layer 1 when the technology starts
    /// with it); on any other layer it is kept.
    #[test]
    fn a_layer_table_on_layer_one_is_skipped() {
        let d = db2("l1", "");
        let tables = crate::tech::read::cut_spacing_tables(&d, "V2", 1, &[]).expect("tables");
        assert_eq!(tables, crate::tech::CutSpacingTables::default());
        let d = db2("l1b", "  PROPERTY LEF58_SPACINGTABLE \"SPACINGTABLE LAYER V1 CUTCLASS CB CA 0.05 0.06 ;\" ;\n");
        assert_eq!(crate::tech::read::cut_spacing_tables(&d, "V2", 1, &[]).expect("tables").inter, None, "layer 1: skipped");
        assert_eq!(crate::tech::read::cut_spacing_tables(&d, "V2", 5, &[]).expect("tables").inter, Some(0), "layer 5: kept");
    }

    /// A two-metal technology with one default via VA on V1 (cut 0.04 square, class CA); `m2dir`
    /// sets M2's direction (a VERTICAL M2 makes VA's horizontal M2 pad misaligned, so the default
    /// vias GENERATE `V1_FR`, which has no cut class).
    fn db_via(name: &str, m1: &str, m2: &str, m2dir: &str) -> Db {
        let lef = format!(
            "VERSION 5.8 ;\nUNITS\n  DATABASE MICRONS 1000 ;\nEND UNITS\nMANUFACTURINGGRID 0.005 ;\n\
             LAYER M1\n  TYPE ROUTING ;\n  DIRECTION HORIZONTAL ;\n  PITCH 0.12 ;\n  WIDTH 0.04 ;\n  SPACING 0.04 ;\n{m1}END M1\n\
             LAYER V1\n  TYPE CUT ;\n  SPACING 0.05 ;\n  WIDTH 0.04 ;\n  PROPERTY LEF58_CUTCLASS \"CUTCLASS CA WIDTH 0.04 ; CUTCLASS CB WIDTH 0.06 ;\" ;\nEND V1\n\
             LAYER M2\n  TYPE ROUTING ;\n  DIRECTION {m2dir} ;\n  PITCH 0.12 ;\n  WIDTH 0.04 ;\n  SPACING 0.04 ;\n{m2}END M2\n\
             VIA VA DEFAULT\n  LAYER M1 ;\n    RECT -0.05 -0.02 0.05 0.02 ;\n  LAYER V1 ;\n    RECT -0.02 -0.02 0.02 0.02 ;\n  LAYER M2 ;\n    RECT -0.05 -0.02 0.05 0.02 ;\nEND VA\n\
             END LIBRARY\n"
        );
        let path = std::env::temp_dir().join(format!("vyges-drt-mincut-{}-{name}.lef", std::process::id()));
        std::fs::write(&path, lef).expect("write lef");
        let mut db = Db::new();
        db.read_lef(path.to_str().expect("utf-8 path")).expect("read lef");
        let _ = std::fs::remove_file(&path);
        db
    }

    /// ⛔ Divergence D9 (register; upstream OpenROAD issue #12176). Upstream rule: the database's
    /// LEF reader fills a per-cut-class LEF58 MINIMUMCUT rule's class map but never sets its
    /// per-cut-class flag, so io `convertLef58MinCutConstraints` reads the count 0 and drops the
    /// rule. Our rule, the reference patched with vyges-openroad-patches upstream-fix 0003 (the
    /// flag set): after `initDefaultVias`, per (class, count) in map order (by class name), the
    /// default via of the cut layer below unless FROMABOVE, then the one above unless FROMBELOW;
    /// the first whose cut class matches gives the count; none matching drops the rule; one or
    /// no cut drops it. Where the via looked at has no cut class (here the generated `V1_FR`) the
    /// patched reference dereferences a null class: refused.
    #[test]
    fn a_per_cut_class_minimum_cut_rule_takes_its_count_from_the_default_vias() {
        use crate::dr::rules::{default_vias, RuleConfig};
        use crate::tech::MinCut;
        let p = |r: &str| format!("  PROPERTY LEF58_MINIMUMCUT \"{r}\" ;\n");
        let m1 = format!("{}{}{}", p("MINIMUMCUT CUTCLASS CB 3 CUTCLASS CA 2 WIDTH 0.2 ;"), p("MINIMUMCUT CUTCLASS CA 4 WIDTH 0.3 FROMBELOW ;"), p("MINIMUMCUT 2 WIDTH 0.5 ;"));
        let m2 = format!("{}{}{}", p("MINIMUMCUT CUTCLASS CA 3 WIDTH 0.2 FROMABOVE ;"), p("MINIMUMCUT CUTCLASS CA 3 WIDTH 0.25 ;"), p("MINIMUMCUT CUTCLASS CB 5 WIDTH 0.25 ;"));
        let resolved = |d: &Db| {
            let mut t = crate::tech::read::tech(d).expect("tech");
            let (lm1, lm2) = (t.layer_num("M1").expect("M1"), t.layer_num("M2").expect("M2"));
            let read: Vec<usize> = [lm1, lm2].iter().map(|&l| t.layers[l].min_cuts.len()).collect();
            let defaults = default_vias(&mut t, &RuleConfig { bottom_routing_layer: lm1, top_routing_layer: lm2, enable_via_gen: true });
            crate::tech::read::resolve_lef58_min_cuts(d, &mut t, &defaults).map(|()| (read, t.layers[lm1].min_cuts.clone(), t.layers[lm2].min_cuts.clone()))
        };
        let d = db_via("pcc", &m1, &m2, "HORIZONTAL");
        let (skipped, refused, _) = census(&d);
        assert!(skipped.is_empty() && refused.is_empty(), "{skipped:?} {refused:?}");
        let rule = |num_cuts, width| MinCut { num_cuts, width, within: None, from_above: None, length: None };
        let (read, m1_rules, m2_rules) = resolved(&d).expect("every default via has a class");
        assert_eq!(read, [1, 0], "as read, only the plain rule: the per-class ones wait for the default vias");
        assert_eq!(m1_rules, [rule(2, 200), rule(2, 500)], "M1: CA (map order first) above gives 2; FROMBELOW sees no via below");
        assert_eq!(m2_rules, [rule(3, 250)], "M2: FROMABOVE has no via above; CB matches no default via");
        let d = db_via("pcc-gen", &m1, &m2, "VERTICAL");
        let err = resolved(&d).expect_err("the generated default via has no cut class");
        assert!(err.contains("V1_FR") && err.contains("#12176"), "{err}");
    }

    /// Rule (reference io `setRoutingLayerProperties` then postProcess
    /// `convertLef58MinCutConstraints`): LEF58 MINIMUMCUT rules with AREA, SAMEMETALOVERLAP or
    /// FULLYENCLOSED are skipped; the rest become minimum-cut rules after the v5.4 ones, in order,
    /// unless they ask for one cut or none. LENGTH … WITHIN raises MTSAFEDIST (a v5.4 LENGTH does
    /// not). MODELLED, nothing refused. A per-cut-class rule (index 1) waits for the default vias
    /// (divergence D9): absent from the technology as read, NOT skipped.
    #[test]
    fn lef58_minimum_cut_rules_convert_after_the_v54_ones() {
        use crate::tech::MinCut;
        let rules = [
            "MINIMUMCUT 2 WIDTH 0.2 ;",
            "MINIMUMCUT CUTCLASS CA 2 WIDTH 0.2 ;",
            "MINIMUMCUT 3 WIDTH 0.3 WITHIN 0.1 FROMABOVE ;",
            "MINIMUMCUT 1 WIDTH 0.2 ;",
            "MINIMUMCUT 2 WIDTH 0.2 AREA 0.5 ;",
            "MINIMUMCUT 2 WIDTH 0.2 SAMEMETALOVERLAP ;",
            "MINIMUMCUT 2 WIDTH 0.2 FULLYENCLOSED ;",
            "MINIMUMCUT 2 WIDTH 0.4 FROMBELOW LENGTH 0.5 WITHIN 2.5 ;",
        ];
        let m1 = format!("  MINIMUMCUT 4 WIDTH 0.6 LENGTH 0.7 WITHIN 3.5 ;\n  PROPERTY LEF58_MINIMUMCUT \"{}\" ;\n", rules.join(" "));
        let d = db("mc", &m1, "");
        assert_eq!(d.num_layer_get_tech_layer_min_cut_rules("M1"), rules.len());
        // The database marks a per-cut-class rule as one (opendb-lib's LEF reader sets
        // isPerCutClass, as the reference patched with upstream-fix 0003 does; the unpatched
        // odb parser never calls setPerCutClass, OpenROAD issue #12176), and keeps its single
        // cut count at 0: the count is per class, read at conversion (D9).
        assert!(d.mincutrule_is_per_cut_class("M1", 1) && d.mincutrule_get_num_cuts("M1", 1) == 0);
        let (skipped, refused, _) = census(&d);
        assert!(refused.is_empty(), "{refused:?}");
        // Skipped with a warning: the rules the reference skips or drops unread (a one-cut rule
        // is dropped as one that can never be broken, not skipped).
        let sk = |index, clause| super::SkippedRule { layer: "M1".into(), family: "lef58_min_cut", index, clause };
        assert_eq!(skipped, [sk(4, "AREA"), sk(5, "SAMEMETALOVERLAP"), sk(6, "FULLYENCLOSED")]);
        assert!(skipped[0].warning().contains("uses AREA"));
        assert_eq!(skipped.iter().map(|s| s.upstream()).collect::<Vec<_>>(), [None, None, None]);
        let tech = crate::tech::read::tech(&d).expect("tech");
        let m1 = tech.layer_num("M1").expect("M1");
        let rule = |num_cuts, width, within, from_above, length| MinCut { num_cuts, width, within, from_above, length };
        assert_eq!(
            tech.layers[m1].min_cuts,
            [rule(4, 600, None, None, Some((700, 3500))), rule(2, 200, None, None, None), rule(3, 300, Some(100), Some(true), None), rule(2, 400, None, Some(false), Some((500, 2500)))]
        );
        assert_eq!(tech.layers[m1].mt_safe_dist, 2500);
        assert_eq!(crate::dr::drw::mt_safe_dist(&tech, &[]), 2500);
        assert_eq!(tech.layers[tech.layer_num("M2").expect("M2")].min_cuts, []);
    }

    /// Rule (reference io `setCutLayerProperties`' caller, `hasOrthSpacingTable`): a cut layer's
    /// SPACINGTABLE ORTHOGONAL is kept whole, its (WITHIN, SPACING) rows in the technology's order
    /// (`frOrthSpacingTableConstraint`); MODELLED (the check is its only consumer), nothing
    /// refused or skipped.
    #[test]
    fn an_orthogonal_spacing_table_is_kept_row_by_row() {
        let d = db("orth", "", "  SPACINGTABLE ORTHOGONAL WITHIN 0.06 SPACING 0.04 WITHIN 0.10 SPACING 0.07 ;\n");
        let (skipped, refused, _) = census(&d);
        assert!(refused.is_empty() && skipped.is_empty(), "{refused:?} {skipped:?}");
        let tech = crate::tech::read::tech(&d).expect("tech");
        assert_eq!(tech.layers[tech.layer_num("V1").expect("V1")].orth_spacing, [(60, 40), (100, 70)]);
        assert!(tech.layers[tech.layer_num("M1").expect("M1")].orth_spacing.is_empty());
    }

    /// Rule (reference io `setRoutingLayerProperties`, its LEF58_AREA loop): a rule with
    /// EXCEPTMINWIDTH, EXCEPTEDGELENGTH (one value or two), EXCEPTMINSIZE, EXCEPTSTEP, MASK or
    /// LAYER (a trim layer) is skipped with a warning (DRT-311..316), tested in that order; every
    /// other rule is kept, a RECTWIDTH one in the first list (ascending by RECTWIDTH, AREA), any
    /// other in the second (descending by AREA), a tie going before the rules it ties with.
    /// MODELLED (the check and the router's patch pass), nothing refused.
    #[test]
    fn lef58_area_rules_are_kept_in_io_lists_and_the_skipped_forms_warned() {
        let rules = [
            "AREA 0.02 ;",
            "AREA 0.03 EXCEPTMINWIDTH 0.05 ;",
            "AREA 0.01 RECTWIDTH 0.05 ;",
            "AREA 0.02 EXCEPTEDGELENGTH 0.1 ;",
            "AREA 0.02 EXCEPTEDGELENGTH 0.1 0.2 ;",
            "AREA 0.02 EXCEPTMINSIZE 0.05 0.1 ;",
            "AREA 0.02 EXCEPTSTEP 0.05 0.1 ;",
            "AREA 0.02 MASK 1 ;",
            "AREA 0.02 LAYER M1 OVERLAP 1 ;",
            "AREA 0.02 EXCEPTRECTANGLE ;",
            "AREA 0.005 RECTWIDTH 0.05 ;",
            "AREA 0.04 ;",
            "AREA 0.02 MASK 1 EXCEPTMINWIDTH 0.05 ;",
            "AREA 0.02 RECTWIDTH 0.04 EXCEPTRECTANGLE ;",
        ];
        let m1 = format!("  PROPERTY LEF58_AREA \"{}\" ;\n", rules.join(" "));
        let d = db("area", &m1, "");
        assert_eq!(d.num_layer_get_tech_layer_area_rules("M1"), rules.len(), "every rule parsed");
        let (skipped, refused, _) = census(&d);
        assert!(refused.is_empty(), "{refused:?}");
        let sk = |index, clause| super::SkippedRule { layer: "M1".into(), family: "lef58_area", index, clause };
        assert_eq!(skipped, [sk(1, "EXCEPTMINWIDTH"), sk(3, "EXCEPTEDGELENGTH"), sk(4, "EXCEPTEDGELENGTH"), sk(5, "EXCEPTMINSIZE"), sk(6, "EXCEPTSTEP"), sk(7, "MASK"), sk(8, "LAYER"), sk(12, "EXCEPTMINWIDTH")]);
        assert!(skipped[0].warning().contains("LEF58_AREA rule 1 uses EXCEPTMINWIDTH") && skipped[0].warning().contains("SKIPPED"));
        let tech = crate::tech::read::tech(&d).expect("tech");
        let l = &tech.layers[tech.layer_num("M1").expect("M1")].lef58_area;
        use crate::tech::AreaRule;
        let r = |index, area, rect_width, except_rectangle| AreaRule { index, area, rect_width, except_rectangle };
        assert_eq!(l.rect_width, [r(13, 20_000, 40, true), r(10, 5_000, 50, false), r(2, 10_000, 50, false)]);
        assert_eq!(l.area, [r(11, 40_000, 0, false), r(9, 20_000, 0, true), r(0, 20_000, 0, false)]);
        assert!(tech.layers[tech.layer_num("M2").expect("M2")].lef58_area.is_empty());
    }

    /// Rule (reference io `setRoutingLayerProperties`, its LEF58_MINSTEP loop): a rule whose
    /// MAXEDGES is above 1 is skipped with DRT-335 (MAXEDGES read whether given or not); every
    /// other rule is kept with MINSTEP, MAXEDGES (-1 when not given), the first MINADJACENTLENGTH
    /// (-1 when not given), the end-of-line width as NOADJACENTEOL's or NOBETWEENEOL's (-1 for
    /// the other) and EXCEPTRECTANGLE. MODELLED (the check and the router's minimum-step patch
    /// pass), nothing refused.
    #[test]
    fn lef58_min_step_rules_are_kept_but_maxedges_above_one_is_skipped_with_a_warning() {
        let rules = [
            "MINSTEP 0.05 MAXEDGES 1 MINADJACENTLENGTH 0.06 ;",
            "MINSTEP 0.05 MAXEDGES 1 NOBETWEENEOL 0.04 ;",
            "MINSTEP 0.05 MAXEDGES 2 MINADJACENTLENGTH 0.06 ;",
            "MINSTEP 0.07 EXCEPTRECTANGLE MINADJACENTLENGTH 0.08 ;",
        ];
        let m1 = format!("  PROPERTY LEF58_MINSTEP \"{}\" ;\n", rules.join(" "));
        let d = db("minstep", &m1, "");
        assert_eq!(d.num_layer_get_tech_layer_min_step_rules("M1"), rules.len(), "every rule parsed");
        let (skipped, refused, _) = census(&d);
        assert!(refused.is_empty(), "{refused:?}");
        assert_eq!(skipped, [super::SkippedRule { layer: "M1".into(), family: "lef58_min_step", index: 2, clause: "MAXEDGES above 1" }]);
        assert!(skipped[0].warning().contains("LEF58_MINSTEP rule 2 uses MAXEDGES above 1") && skipped[0].warning().contains("SKIPPED"));
        assert_eq!(crate::tech::read::lef58_min_step_skip_warnings(&d, "M1"), [(335, "LEF58_MINSTEP MAXEDGES 2  is not supported".to_string())]);
        let tech = crate::tech::read::tech(&d).expect("tech");
        let r = |min_step_length, max_edges, min_adjacent_length, no_adj_eol, eol_width, except_rectangle| crate::tech::Lef58MinStep { min_step_length, max_edges, min_adjacent_length, no_adj_eol, eol_width, except_rectangle };
        assert_eq!(
            tech.layers[tech.layer_num("M1").expect("M1")].lef58_min_step,
            [r(50, 1, 60, -1, -1, false), r(50, 1, -1, -1, 40, false), r(70, -1, 80, -1, -1, true)]
        );
        assert!(tech.layers[tech.layer_num("M2").expect("M2")].lef58_min_step.is_empty());
    }

    /// Upstream OpenROAD issue #12229 (register D14) asks about NOADJACENTEOL and EXCEPTRECTANGLE.
    /// A LEF58_MINSTEP rule the reference's check does not read as written is refused, naming the
    /// clause: MINBETWEENLENGTH, a second MINADJACENTLENGTH value or a corner form, or a rule with
    /// neither checked form (io keeps it and the check never reads it). A rule io skips is
    /// skipped, not refused.
    #[test]
    fn a_lef58_min_step_rule_the_check_does_not_read_as_written_is_refused() {
        for (rule, clause) in [
            ("MINSTEP 0.05 MINBETWEENLENGTH 0.06 ;", "MINBETWEENLENGTH"),
            ("MINSTEP 0.05 MINADJACENTLENGTH 0.06 0.07 ;", "MINADJACENTLENGTH with a second length"),
            ("MINSTEP 0.05 MINADJACENTLENGTH 0.06 CONVEXCORNER ;", "MINADJACENTLENGTH CONVEXCORNER"),
            ("MINSTEP 0.05 MINADJACENTLENGTH 0.06 CONCAVECORNER ;", "MINADJACENTLENGTH CONCAVECORNER"),
            ("MINSTEP 0.05 MAXEDGES 1 ;", "neither NOBETWEENEOL nor MINADJACENTLENGTH (never checked)"),
            ("MINSTEP 0.05 NOADJACENTEOL 0.06 ;", "NOADJACENTEOL"),
            ("MINSTEP 0.05 MAXEDGES 1 NOADJACENTEOL 0.06 MINADJACENTLENGTH 0.03 ;", "NOADJACENTEOL"),
            ("MINSTEP 0.05 MAXEDGES 1 EXCEPTRECTANGLE NOBETWEENEOL 0.06 ;", "EXCEPTRECTANGLE with NOBETWEENEOL"),
        ] {
            let d = db("msref", &format!("  PROPERTY LEF58_MINSTEP \"{rule}\" ;\n"), "");
            assert_eq!(d.num_layer_get_tech_layer_min_step_rules("M1"), 1, "{rule} parsed");
            let (skipped, refused, _) = census(&d);
            assert!(skipped.is_empty(), "{skipped:?}");
            assert_eq!(refused, [format!("M1: lef58_min_step (1, {clause})")], "{rule}");
        }
        // A refused rule is still read as io keeps it (the check reads it where refusals are
        // waived): NOADJACENTEOL's width, the first MINADJACENTLENGTH.
        let d = db("msadj", "  PROPERTY LEF58_MINSTEP \"MINSTEP 0.05 MAXEDGES 1 NOADJACENTEOL 0.06 MINADJACENTLENGTH 0.03 ;\" ;\n", "");
        let tech = crate::tech::read::tech(&d).expect("tech");
        let want = crate::tech::Lef58MinStep { min_step_length: 50, max_edges: 1, min_adjacent_length: 30, no_adj_eol: 60, eol_width: -1, except_rectangle: false };
        assert_eq!(tech.layers[tech.layer_num("M1").expect("M1")].lef58_min_step, [want]);
        let d = db("msskip", "  PROPERTY LEF58_MINSTEP \"MINSTEP 0.05 MAXEDGES 2 MINBETWEENLENGTH 0.06 ;\" ;\n", "");
        let (skipped, refused, _) = census(&d);
        assert!(refused.is_empty() && skipped.len() == 1, "{refused:?} {skipped:?}");
    }

    /// Rule (reference io `setCutLayerProperties`, LAYER branch): `SPACING s LAYER m CUTCLASS c
    /// NONEOLCONVEXCORNER w [MINLENGTH l]` is kept as a different-net LEF58 cut spacing rule with
    /// the end-of-line width, and the minimum length only when MINLENGTH is given (it is read
    /// under NONEOLCONVEXCORNER alone), so it is MODELLED, in the technology's order beside a
    /// CONCAVECORNER rule.
    #[test]
    fn the_cut_spacing_layer_non_eol_convex_corner_form_is_modelled() {
        use crate::tech::{CutCorner, CutLayerSpacing, Lef58CutSpacing::Layer};
        let rules = format!("{}{}{}", eol("SPACING 0.07 LAYER M2 CUTCLASS CA CONCAVECORNER ;"), eol("SPACING 0.07 LAYER M2 CUTCLASS CA NONEOLCONVEXCORNER 0.05 ;"), eol("SPACING 0.08 LAYER M1 CUTCLASS CA NONEOLCONVEXCORNER 0.06 MINLENGTH 0.03 ;"));
        let d = db("cv", "", &rules);
        let (skipped, refused, _) = census(&d);
        assert!(skipped.is_empty() && refused.is_empty(), "{skipped:?} {refused:?}");
        let tech = crate::tech::read::tech(&d).expect("tech");
        let (v1, m1, m2) = (tech.layer_num("V1").expect("V1"), tech.layer_num("M1").expect("M1"), tech.layer_num("M2").expect("M2"));
        assert_eq!(
            tech.layers[v1].lef58_cut_spacing,
            [
                Layer(CutLayerSpacing { spacing: 70, second_layer: m2, class: 1, corner: CutCorner::Concave }),
                Layer(CutLayerSpacing { spacing: 70, second_layer: m2, class: 1, corner: CutCorner::NonEolConvex { eol_width: 50, min_length: None } }),
                Layer(CutLayerSpacing { spacing: 80, second_layer: m1, class: 1, corner: CutCorner::NonEolConvex { eol_width: 60, min_length: Some(30) } }),
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
            ("SPACING 0.07 LAYER M2 STACK ;", "STACK"),
            ("SPACING 0.07 CENTERTOCENTER LAYER M2 CUTCLASS CA CONCAVECORNER ;", "CENTERTOCENTER"),
            ("SPACING 0.07 LAYER M2 CUTCLASS CA SHORTEDGEONLY ;", "SHORTEDGEONLY"),
            ("SPACING 0.07 SAMENET ADJACENTCUTS 2 WITHIN 0.1 CUTCLASS CA ;", "ADJACENTCUTS with SAMENET"),
            ("SPACING 0.07 SAMEMETAL ADJACENTCUTS 2 WITHIN 0.1 CUTCLASS CA ;", "ADJACENTCUTS with SAMEMETAL or SAMEVIA"),
            ("SPACING 0.07 ADJACENTCUTS 3 TWOCUTS 1 SAMECUT WITHIN 0.1 CUTCLASS CA ;", "ADJACENTCUTS TWOCUTS SAMECUT"),
            ("SPACING 0.07 PARALLELOVERLAP ;", "PARALLELOVERLAP"),
        ] {
            let rules = format!("{}{}", eol("SPACING 0.07 LAYER M2 CUTCLASS CA CONCAVECORNER ;"), eol(rule));
            let d = db("cc-other", "", &rules);
            let (_, refused, _) = census(&d);
            assert_eq!(refused, [format!("V1: lef58_cut_spacing (2, {clause})")], "{rule}");
        }
    }

    /// ⛔ Divergence D7 (register; upstream OpenROAD issue #12202). Upstream rule (io
    /// `setCutLayerProperties`, ADJACENTCUTS branch): only the first WITHIN is read, and GC uses
    /// it as the upper bound, so `WITHIN w1 w2` is checked as `WITHIN w1`. Our rule, the reference
    /// patched with vyges-openroad-patches upstream-fix 0012: after the reference's own skips the
    /// rule is SKIPPED with DRT-359 — not in the layer's list, not refused, not in the same-net
    /// list even with SAMENET — and a checked rule beside it is unaffected.
    #[test]
    fn an_adjacent_cuts_rule_with_two_within_values_is_skipped_with_drt_359() {
        use crate::tech::read::ADJ_TWO_WITHIN;
        let rules = format!(
            "{}{}{}",
            eol("SPACING 0.07 ADJACENTCUTS 2 WITHIN 0.1 0.12 CUTCLASS CA ;"),
            eol("SPACING 0.07 LAYER M2 CUTCLASS CA CONCAVECORNER ;"),
            eol("SPACING 0.07 SAMENET ADJACENTCUTS 2 WITHIN 0.05 0.12 CUTCLASS CA ;")
        );
        let d = db("adj2w", "", &rules);
        let (skipped, refused, _) = census(&d);
        assert!(refused.is_empty(), "{refused:?}");
        assert_eq!(skipped.iter().map(|s| (s.family, s.index, s.clause)).collect::<Vec<_>>(), [("lef58_cut_spacing", 0, ADJ_TWO_WITHIN), ("lef58_cut_spacing", 2, ADJ_TWO_WITHIN)]);
        for s in &skipped {
            let w = s.warning();
            assert!(w.contains("two WITHIN values") && w.contains("SKIPPED") && w.contains("#12202"), "{w}");
            assert_eq!(s.upstream(), Some(super::SKIPPED_ADJ_TWO_WITHIN_UPSTREAM_ISSUE));
        }
        let tech = crate::tech::read::tech(&d).expect("tech");
        let v1 = &tech.layers[tech.layer_num("V1").expect("V1")];
        assert_eq!(v1.lef58_cut_spacing.len(), 1, "only the checked LAYER rule stays");
        let msg = "Unsupported branch WITHIN with two values in LEF58_SPACING ADJACENTCUTS for layer V1, skipping rule.".to_string();
        assert_eq!(crate::tech::read::lef58_cut_spacing_skip_warnings(&d, "V1", &v1.cut_classes), [(359, msg.clone()), (359, msg)]);
        // One WITHIN value: kept and checked, no warning.
        let d = db("adj1w", "", &eol("SPACING 0.07 ADJACENTCUTS 2 WITHIN 0.1 CUTCLASS CA ;"));
        let (skipped, refused, _) = census(&d);
        assert!(skipped.is_empty() && refused.is_empty(), "{skipped:?} {refused:?}");
        let tech = crate::tech::read::tech(&d).expect("tech");
        let v1 = &tech.layers[tech.layer_num("V1").expect("V1")];
        assert_eq!(v1.lef58_cut_spacing.len(), 1);
        assert!(crate::tech::read::lef58_cut_spacing_skip_warnings(&d, "V1", &v1.cut_classes).is_empty());
    }

    /// ⛔ Divergence D6 (register; OpenROAD issue #12187). Upstream rule (io
    /// `setCutLayerProperties`, LAYER branch, and GC `checkLef58CutSpacing_spc_layer`): after STACK
    /// and ORTHOGONALSPACING, io KEEPS a LAYER rule with no CUTCLASS (spacing, CENTERTOCENTER,
    /// SAMENET, the second layer), a SAMENET one in the same-net list; but the check acts only
    /// under `hasCutClass()`, so the rule never marks and nothing warns. Our rule, the reference
    /// patched with vyges-openroad-patches upstream-fix 0005: SKIPPED with DRT-353, not in the
    /// layer's list, and a checked rule beside a plain one is unaffected.
    #[test]
    fn a_layer_rule_without_a_cut_class_is_skipped() {
        use crate::tech::read::{LAYER_NO_CLASS, LAYER_NO_CLASS_SAME_NET};
        let rules = format!("{}{}{}", eol("SPACING 0.07 LAYER M2 ;"), eol("SPACING 0.07 CENTERTOCENTER LAYER M1 ;"), eol("SPACING 0.07 SAMENET LAYER M1 ;"));
        let d = db("nc", "", &rules);
        let (skipped, refused, _) = census(&d);
        assert!(refused.is_empty(), "{refused:?}");
        assert_eq!(skipped.iter().map(|s| (s.family, s.index, s.clause)).collect::<Vec<_>>(), [("lef58_cut_spacing", 0, LAYER_NO_CLASS), ("lef58_cut_spacing", 1, LAYER_NO_CLASS), ("lef58_cut_spacing", 2, LAYER_NO_CLASS_SAME_NET)]);
        assert!(skipped.iter().all(|s| s.warning().contains("never checks this one") && s.warning().contains("SKIPPED") && s.warning().contains("#12187") && s.upstream() == Some(super::SKIPPED_LAYER_UPSTREAM_ISSUE)), "{skipped:?}");
        let tech = crate::tech::read::tech(&d).expect("tech");
        let v1 = &tech.layers[tech.layer_num("V1").expect("V1")];
        assert!(v1.lef58_cut_spacing.is_empty());
        let w = (353, "Unsupported LEF58_SPACING LAYER rule without CUTCLASS for layer V1, skipping rule.".to_string());
        assert_eq!(crate::tech::read::lef58_cut_spacing_skip_warnings(&d, "V1", &v1.cut_classes), [w.clone(), w.clone(), w]);
        // Beside a checked rule: the checked rule alone is in the list; the plain one is skipped.
        let d = db("nc-cc", "", &format!("{}{}", eol("SPACING 0.07 LAYER M2 ;"), eol("SPACING 0.07 LAYER M2 CUTCLASS CA CONCAVECORNER ;")));
        let (skipped, refused, _) = census(&d);
        assert!(refused.is_empty(), "{refused:?}");
        assert_eq!(skipped.iter().map(|s| (s.index, s.clause)).collect::<Vec<_>>(), [(0, LAYER_NO_CLASS)]);
        let tech = crate::tech::read::tech(&d).expect("tech");
        assert_eq!(tech.layers[tech.layer_num("V1").expect("V1")].lef58_cut_spacing.len(), 1);
    }

    /// ⛔ Divergence D6 (#12187). Upstream rule (io `setCutLayerProperties`, LAYER branch, and GC
    /// `checkLef58CutSpacing_spc_layer`): past the branches io drops, io KEEPS a LAYER rule with a
    /// defined CUTCLASS but neither CONCAVECORNER nor NONEOLCONVEXCORNER (CENTERTOCENTER and
    /// SAMENET kept too, a SAMENET one in the same-net list); past the class test the check has
    /// only those two branches, so the rule never marks and nothing warns. Our rule (upstream-fix
    /// 0005): SKIPPED with DRT-354, not in the layer's list.
    #[test]
    fn a_layer_rule_with_a_class_but_neither_corner_is_skipped() {
        use crate::tech::read::{LAYER_NO_CORNER, LAYER_NO_CORNER_SAME_NET};
        let rules = format!("{}{}{}", eol("SPACING 0.07 LAYER M2 CUTCLASS CA ;"), eol("SPACING 0.07 CENTERTOCENTER LAYER M1 CUTCLASS CA ;"), eol("SPACING 0.07 SAMENET LAYER M1 CUTCLASS CA ;"));
        let d = db("ncc", "", &rules);
        let (skipped, refused, _) = census(&d);
        assert!(refused.is_empty(), "{refused:?}");
        assert_eq!(skipped.iter().map(|s| (s.index, s.clause)).collect::<Vec<_>>(), [(0, LAYER_NO_CORNER), (1, LAYER_NO_CORNER), (2, LAYER_NO_CORNER_SAME_NET)]);
        assert!(skipped.iter().all(|s| s.warning().contains("never checks this one") && s.upstream() == Some(super::SKIPPED_LAYER_UPSTREAM_ISSUE)), "{skipped:?}");
        let tech = crate::tech::read::tech(&d).expect("tech");
        let v1 = &tech.layers[tech.layer_num("V1").expect("V1")];
        assert!(v1.lef58_cut_spacing.is_empty());
        let w = (354, "Unsupported LEF58_SPACING LAYER CUTCLASS rule without CONCAVECORNER or NONEOLCONVEXCORNER for layer V1, skipping rule.".to_string());
        assert_eq!(crate::tech::read::lef58_cut_spacing_skip_warnings(&d, "V1", &v1.cut_classes), [w.clone(), w.clone(), w]);
        // Its SAMENET form beside a checked rule is skipped too, and the checked rule stays.
        let d = db("ncc-sn", "", &format!("{}{}", eol("SPACING 0.07 LAYER M2 CUTCLASS CA CONCAVECORNER ;"), eol("SPACING 0.07 SAMENET LAYER M1 CUTCLASS CA ;")));
        let (skipped, refused, _) = census(&d);
        assert!(refused.is_empty(), "{refused:?}");
        assert_eq!(skipped.iter().map(|s| (s.index, s.clause)).collect::<Vec<_>>(), [(1, LAYER_NO_CORNER_SAME_NET)]);
        let tech = crate::tech::read::tech(&d).expect("tech");
        assert_eq!(tech.layers[tech.layer_num("V1").expect("V1")].lef58_cut_spacing.len(), 1);
    }

    /// ⛔ Divergence D6 (#12187, upstream-fix 0005). Upstream rule (GC `checkCutSpacing_main(rect)`):
    /// any rule in the layer's LEF58 SAME-NET list sets skipDiffNet, and every different-net LEF58
    /// rule then pairs a cut only with shapes of its own net (OpenROAD issue #12188), so the
    /// unpatched reference's SAMENET LAYER rule with no class silences a checked rule beside it on
    /// every other net. Our rule, as the patched reference: the SAMENET rule is dropped at read
    /// (DRT-353), the same-net list stays empty, and the checked rule beside it is KEPT and judges
    /// every net — nothing is refused.
    #[test]
    fn a_same_net_layer_rule_beside_a_checked_rule_is_skipped_and_the_checked_rule_kept() {
        use crate::tech::read::LAYER_NO_CLASS_SAME_NET;
        for (checked, kept) in [
            ("SPACING 0.07 LAYER M2 CUTCLASS CA CONCAVECORNER ;", 1),
            ("SPACING 0.1 CENTERTOCENTER ADJACENTCUTS 2 WITHIN 0.12 CUTCLASS CA ;", 1),
            ("SPACING 0.07 LAYER M2 ;", 0),
            ("SPACING 0.1 CENTERTOCENTER ADJACENTCUTS 2 WITHIN 0.12 EXCEPTSAMEPGNET CUTCLASS CA ;", 0),
        ] {
            let d = db("sn-mix", "", &format!("{}{}", eol(checked), eol("SPACING 0.07 SAMENET LAYER M1 ;")));
            let (skipped, refused, _) = census(&d);
            assert!(refused.is_empty(), "{checked}: {refused:?}");
            assert!(skipped.iter().any(|s| s.index == 1 && s.clause == LAYER_NO_CLASS_SAME_NET), "{checked}: {skipped:?}");
            let tech = crate::tech::read::tech(&d).expect("tech");
            assert_eq!(tech.layers[tech.layer_num("V1").expect("V1")].lef58_cut_spacing.len(), kept, "{checked}");
        }
    }

    /// Rule (reference io `setCutLayerProperties`, ADJACENTCUTS): io keeps the spacing,
    /// CENTERTOCENTER, the count, TWOCUTS, the first WITHIN value, the class and NOPRL — a rule
    /// naming no class has none (the reference's -1, ours 0) — in the technology's order beside a
    /// cut-to-corner rule; MODELLED.
    #[test]
    fn the_adjacent_cuts_form_is_modelled() {
        use crate::tech::{AdjacentCuts, Lef58CutSpacing};
        let rules = format!(
            "{}{}{}",
            eol("SPACING 0.1 CENTERTOCENTER ADJACENTCUTS 2 WITHIN 0.12 CUTCLASS CA ;"),
            eol("SPACING 0.07 LAYER M2 CUTCLASS CA CONCAVECORNER ;"),
            eol("SPACING 0.08 ADJACENTCUTS 3 TWOCUTS 1 WITHIN 0.11 NOPRL ;")
        );
        let d = db("adj", "", &rules);
        let (skipped, refused, _) = census(&d);
        assert!(skipped.is_empty() && refused.is_empty(), "{skipped:?} {refused:?}");
        let tech = crate::tech::read::tech(&d).expect("tech");
        let v1 = tech.layer_num("V1").expect("V1");
        let l = &tech.layers[v1].lef58_cut_spacing;
        assert_eq!(l.len(), 3);
        assert_eq!(l[0], Lef58CutSpacing::AdjacentCuts(AdjacentCuts { spacing: 100, center_to_center: true, adjacent_cuts: 2, two_cuts: None, within: 120, class: 1, no_prl: false }));
        assert!(matches!(l[1], Lef58CutSpacing::Layer(_)));
        assert_eq!(l[2], Lef58CutSpacing::AdjacentCuts(AdjacentCuts { spacing: 80, center_to_center: false, adjacent_cuts: 3, two_cuts: Some(1), within: 110, class: 0, no_prl: true }));
    }

    /// Rule (reference io `setCutLayerProperties`, ADJACENTCUTS): EXACTALIGNED, EXCEPTSAMEPGNET,
    /// TO ALL, SIDEPARALLELOVERLAP and SAMEMASK each make io warn and skip the rule, tested in
    /// that order; SKIPPED here too (warned, not refused, not in the layer's list).
    #[test]
    fn the_adjacent_cuts_forms_io_skips_are_skipped() {
        for (rule, clause) in [
            ("SPACING 0.1 ADJACENTCUTS 2 EXACTALIGNED 1 WITHIN 0.12 EXCEPTSAMEPGNET CUTCLASS CA ;", "ADJACENTCUTS EXACTALIGNED"),
            ("SPACING 0.1 CENTERTOCENTER ADJACENTCUTS 2 WITHIN 0.12 EXCEPTSAMEPGNET CUTCLASS CA ;", "ADJACENTCUTS EXCEPTSAMEPGNET"),
            ("SPACING 0.1 ADJACENTCUTS 2 WITHIN 0.12 CUTCLASS CA TO ALL ;", "ADJACENTCUTS CUTCLASS TO ALL"),
            ("SPACING 0.1 ADJACENTCUTS 2 WITHIN 0.12 CUTCLASS CA SIDEPARALLELOVERLAP ;", "ADJACENTCUTS SIDEPARALLELOVERLAP"),
            ("SPACING 0.1 ADJACENTCUTS 2 WITHIN 0.12 CUTCLASS CA SAMEMASK ;", "ADJACENTCUTS SAMEMASK"),
            ("SPACING 0.1 SAMENET ADJACENTCUTS 2 WITHIN 0.12 EXCEPTSAMEPGNET CUTCLASS CA ;", "ADJACENTCUTS EXCEPTSAMEPGNET"),
        ] {
            let d = db("adj-skip", "", &eol(rule));
            let (skipped, refused, _) = census(&d);
            assert!(refused.is_empty(), "{rule}: {refused:?}");
            assert_eq!(skipped.iter().map(|s| (s.family, s.clause)).collect::<Vec<_>>(), [("lef58_cut_spacing", clause)], "{rule}");
            let tech = crate::tech::read::tech(&d).expect("tech");
            assert!(tech.layers[tech.layer_num("V1").expect("V1")].lef58_cut_spacing.is_empty(), "{rule}");
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
