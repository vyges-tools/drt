// SPDX-License-Identifier: Apache-2.0
//! The rule tables detailed routing reads (computed once per run, before any routing): per
//! routing layer, the via-to-via, via-turn, via-planar and line-to-line forbidden lengths, the
//! via-to-via parallel run, forbidden via-through; the router's end-of-line rule; and per
//! non-default rule its own via-to-via and via-turn tables and end-of-line rule. And, first, each
//! cut layer's default via — which may be a via GENERATED here ([`default_vias`]).
//!
//! Stages, in order ([`rule_tables`]): via-to-via → via-turn → via-planar → line-to-line →
//! end-of-line → (cut-spacing-table defaults: not modelled) → via-through → per non-default rule:
//! via-to-via, via-turn → (via-to-via min step: not modelled).
//!
//! Rules:
//! - layer numbers are the router's (placeholder masterslice 0, placeholder cut 1, then routing
//!   and cut layers alternating); a routing layer's table index is its ordinal among routing
//!   layers, and a non-default rule's per-layer values are indexed the same way (`z = layer/2-1`);
//! - forbidden ranges are merged as closed integer intervals that JOIN when they overlap or touch
//!   (an empty one, low above high, is dropped); the via-to-via tables keep the merged intervals,
//!   the via-turn and line-to-line tables shrink each by one at both ends (which can leave it
//!   empty — kept, low above high);
//! - a via's shapes are its definition's, at the origin.
//!
//! Not modelled (the caller must refuse a technology that has them): min step, minimum cut,
//! width-via maps, cut spacing tables and LEF58 cut spacing, cut spacing between layers, adjacent
//! cuts, same-net and centre-to-centre cut spacing, two-width spacing tables, LEF58 end-of-line
//! families.

use crate::pa::stack::default_via;
use crate::polygon90::Rect;
use crate::tech::{MinCut, Dir, LayerKind, SpacingTable, Tech, ViaDef};

/// Forbidden length ranges, `(low, high)`.
pub type Ranges = Vec<(i32, i32)>;

/// A non-default rule as the tables read it: per routing-layer index, its width, spacing and
/// preferred via (the first via it names whose bottom layer is that layer).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct NdrRule {
    pub name: String,
    pub widths: Vec<i32>,
    pub spacings: Vec<i32>,
    pub vias: Vec<Vec<usize>>,
}

impl NdrRule {
    fn width(&self, z: i64) -> i32 {
        usize::try_from(z).ok().and_then(|z| self.widths.get(z)).copied().unwrap_or(0)
    }
    fn spacing(&self, z: i64) -> i32 {
        usize::try_from(z).ok().and_then(|z| self.spacings.get(z)).copied().unwrap_or(0)
    }
    fn pref_via(&self, z: i64) -> Option<usize> {
        usize::try_from(z).ok().and_then(|z| self.vias.get(z)).and_then(|v| v.first()).copied()
    }
}

/// The router's end-of-line rule: an end narrower than `width` needs `space`, `within` beyond
/// its sides.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct EolTable {
    pub width: i32,
    pub space: i32,
    pub within: i32,
}

/// One routing layer's tables. Via-to-via entry `k`: previous via down/up (`k / 4`), current via
/// down/up (`k / 2 % 2`), along x/y (`k % 2`); via-turn and via-planar `k`: last via down/up
/// (`k / 2`), along x/y; line-to-line `k`: z-shape/u-shape (`k / 2`), along x/y; via-through `k`:
/// down/up via (`k / 2`), along x/y.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct LayerTables {
    pub via2via: [Ranges; 8],
    pub via2via_prl: [i32; 8],
    pub turn: [Ranges; 4],
    pub planar: [Ranges; 4],
    pub line: [Ranges; 4],
    pub through: [bool; 4],
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct NdrTables {
    pub name: String,
    pub via2via: Vec<[Ranges; 8]>,
    pub turn: Vec<[Ranges; 4]>,
    /// Per routing-layer index where the rule has a width.
    pub eol: Vec<Option<EolTable>>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct RuleTables {
    /// Per routing-layer index.
    pub layers: Vec<LayerTables>,
    /// Per routing-layer index.
    pub eol: Vec<EolTable>,
    pub ndrs: Vec<NdrTables>,
    /// Per technology layer: `hasVia2ViaMinStepViol` (`prep_minStepViasCheck`).
    pub min_step_via: Vec<bool>,
}

/// The routing settings the tables read.
#[derive(Debug, Clone, Copy)]
pub struct RuleConfig {
    pub bottom_routing_layer: usize,
    pub top_routing_layer: usize,
    pub enable_via_gen: bool,
}

/// Each layer's default via (cut layers only), in layer order; generated vias are APPENDED to
/// `tech.via_defs`. A cut layer's default is its least single-cut via ([`default_via`]; above the
/// top routing layer, the least of the fewest-cut vias when none has one cut). Between the
/// bottom and top routing layers (by layer number), a default whose shape on a routing layer is
/// not square and runs across that layer's direction is replaced by a GENERATED via named after
/// the cut layer + `_FR`: the same shapes, each misaligned one turned a quarter.
pub fn default_vias(tech: &mut Tech, cfg: &RuleConfig) -> Vec<Option<usize>> {
    let mut out = Vec::with_capacity(tech.layers.len());
    for l in 0..tech.layers.len() {
        out.push(None);
        if tech.layers[l].kind != LayerKind::Cut {
            continue;
        }
        let Some(v) = default_via(tech, l, cfg.top_routing_layer) else { continue };
        out[l] = Some(v);
        if !(cfg.enable_via_gen && l >= cfg.bottom_routing_layer && l <= cfg.top_routing_layer) {
            continue;
        }
        let vd = &tech.via_defs[v];
        let (b1, b2) = (vd.layer1_bbox(), vd.layer2_bbox());
        let misaligned = |b: &Rect, layer: usize| b.dx() != b.dy() && (b.dx() > b.dy()) != (tech.layers[layer].dir == Dir::Horizontal);
        let (m1, m2) = (misaligned(&b1, vd.layer1), misaligned(&b2, vd.layer2));
        if !m1 && !m2 {
            continue;
        }
        // Turned a quarter: x and y swapped (the box is normalised).
        let turn = |b: Rect, horz_layer: bool| if (b.dx() > b.dy()) != horz_layer { Rect { xl: b.yl, yl: b.xl, xh: b.yh, yh: b.xh } } else { b };
        let generated = ViaDef {
            name: format!("{}_FR", tech.layers[l].name),
            is_default: false,
            layer1: vd.layer1,
            cut: vd.cut,
            layer2: vd.layer2,
            layer1_figs: vec![turn(b1, tech.layers[vd.layer1].dir == Dir::Horizontal)],
            cut_figs: vec![bbox(&vd.cut_figs)],
            layer2_figs: vec![turn(b2, tech.layers[vd.layer2].dir == Dir::Horizontal)],
        };
        tech.via_defs.push(generated);
        out[l] = Some(tech.via_defs.len() - 1);
    }
    out
}

fn bbox(figs: &[Rect]) -> Rect {
    figs.iter().skip(1).fold(figs[0], |b, f| Rect { xl: b.xl.min(f.xl), yl: b.yl.min(f.yl), xh: b.xh.max(f.xh), yh: b.yh.max(f.yh) })
}

/// Merged as closed integer intervals (touching ones join, empty ones dropped).
fn merge(ranges: &[(i32, i32)]) -> Ranges {
    let mut v: Vec<(i32, i32)> = ranges.iter().copied().filter(|&(lo, hi)| lo <= hi).collect();
    v.sort_unstable();
    let mut out: Ranges = Vec::new();
    for (lo, hi) in v {
        match out.last_mut() {
            Some(last) if lo <= last.1.saturating_add(1) => last.1 = last.1.max(hi),
            _ => out.push((lo, hi)),
        }
    }
    out
}

/// Merged, then each shrunk by one at both ends.
fn merge_shrunk(ranges: &[(i32, i32)]) -> Ranges {
    merge(ranges).into_iter().map(|(lo, hi)| (lo + 1, hi - 1)).collect()
}

/// A layer's minimum spacing for a width and a parallel run (`None`: the layer has none).
fn min_spacing(tech: &Tech, layer: usize, width: i32, prl: i32) -> Option<i32> {
    tech.layers[layer].spacing.as_ref().map(|t: &SpacingTable| t.find(width, prl))
}

/// The via's shape box on `layer` (its lower shapes when that is its lower layer, else upper).
fn enclosure(vd: &ViaDef, layer: usize) -> Rect {
    if vd.layer1 == layer {
        vd.layer1_bbox()
    } else {
        vd.layer2_bbox()
    }
}

/// `hasMinStepViol`: the two rectangles joined — one polygon, else no — its outer ring through the
/// check's MINSTEP walk (every edge off the fixed shapes).
fn has_min_step_viol(a: &Rect, b: &Rect, con: &crate::tech::MinStep) -> bool {
    let polys = crate::polygon_formation::polygons_with_holes(&[*a, *b]);
    if polys.len() != 1 {
        return false;
    }
    let o = &polys[0].outer;
    let ring: Vec<crate::gc::RingEdge> = (0..o.len()).map(|k| (o[k], o[(k + 1) % o.len()], false)).collect();
    !crate::gc::min_step_boxes(&[ring], con).is_empty()
}

fn z_of(layer: usize) -> i64 {
    layer as i64 / 2 - 1
}

struct Ctx<'a> {
    tech: &'a Tech,
    defaults: &'a [Option<usize>],
    cfg: &'a RuleConfig,
}

impl Ctx<'_> {
    fn routing_layers(&self) -> Vec<usize> {
        (0..self.tech.layers.len()).filter(|&l| self.tech.layers[l].kind == LayerKind::Routing).collect()
    }
    fn default_at(&self, layer: i64) -> Option<usize> {
        usize::try_from(layer).ok().and_then(|l| self.defaults.get(l)).copied().flatten()
    }
    /// The vias below and above a routing layer: a non-default rule's preferred ones when it has
    /// them (below only above the bottom routing layer), else the cut layers' defaults.
    fn down_up(&self, l: usize, ndr: Option<&NdrRule>) -> (Option<usize>, Option<usize>) {
        let down = match ndr.and_then(|n| (self.cfg.bottom_routing_layer < l).then(|| n.pref_via((l as i64 - 2) / 2 - 1)).flatten()) {
            Some(v) => Some(v),
            None => self.default_at(l as i64 - 1),
        };
        let up = if l + 1 < self.tech.layers.len() {
            match ndr.and_then(|n| n.pref_via(z_of(l))) {
                Some(v) => Some(v),
                None => self.default_at(l as i64 + 1),
            }
        } else {
            None
        };
        (down, up)
    }

    // ---- via to via ----

    fn via2via_min_spc(&self, l: usize, v1: usize, v2: usize, along_x: bool, out: &mut Ranges, ndr: Option<&NdrRule>) {
        let tech = self.tech;
        let default_width = tech.layers[l].width;
        let (vd1, vd2) = (&tech.via_defs[v1], &tech.via_defs[v2]);
        let b1 = enclosure(vd1, l);
        let (w1, fat1, prl1) = (b1.dx().min(b1.dy()), if along_x { b1.dy() > default_width } else { b1.dx() > default_width }, if along_x { b1.dy() } else { b1.dx() });
        let b2 = enclosure(vd2, l);
        let (w2, fat2, prl2) = (b2.dx().min(b2.dy()), if along_x { b2.dy() > default_width } else { b2.dx() > default_width }, if along_x { b2.dy() } else { b2.dx() });
        let non_overlap = if along_x { (b1.dx() + b2.dx()) / 2 } else { (b1.dy() + b2.dy()) / 2 };
        let mut req: Option<i32> = None;
        if fat1 && fat2 {
            req = min_spacing(tech, l, w1.max(w2), prl1.min(prl2));
            if let Some(n) = ndr {
                req = Some(req.map_or(n.spacing(z_of(l)), |r| r.max(n.spacing(z_of(l)))));
            }
            req = req.map(|r| r + non_overlap);
        }
        if let Some(r) = req {
            out.push((non_overlap, r));
        }
        // The same via twice: also on its other metal layer.
        if v1 == v2 {
            let (b, other) = if vd1.layer1 == l { (vd1.layer2_bbox(), l + 2) } else { (vd1.layer1_bbox(), l - 2) };
            let non_overlap = if along_x { b.dx() } else { b.dy() };
            let (w, prl) = (b.dx().min(b.dy()), if along_x { b.dy() } else { b.dx() });
            let mut req: Option<i32> = None;
            if tech.layers[other].spacing.is_some() {
                req = min_spacing(tech, other, w, prl);
                if let Some(n) = ndr {
                    req = Some(req.map_or(n.spacing(z_of(other)), |r| r.max(n.spacing(z_of(other)))));
                }
                req = req.map(|r| r + non_overlap);
            }
            if let Some(r) = req {
                out.push((non_overlap, r));
            }
        }
    }

    /// `prep_via2viaForbiddenLen_minimumCut`: a via whose pad on layer `l` triggers a MINIMUMCUT
    /// rule (wider than WIDTH, longer than LENGTH if given) keeps the other via's cut off that pad —
    /// by LENGTH's distance when given — along the LAYER's direction (not the entry's), when the
    /// rule's connection side (FROMABOVE / FROMBELOW) is the other via's. ⚠️ A rule whose first
    /// half fails its connection skips its second half too (the source's `continue`).
    fn via2via_min_cut(&self, l: usize, v1: usize, v2: usize, out: &mut Ranges) {
        let tech = self.tech;
        let horizontal = tech.layers[l].is_horizontal();
        let (vd1, vd2) = (&tech.via_defs[v1], &tech.via_defs[v2]);
        let (b1, b2) = (enclosure(vd1, l), enclosure(vd2, l));
        let (c1, c2) = (bbox(&vd1.cut_figs), bbox(&vd2.cut_figs));
        let (above1, above2) = (vd1.layer1 == l, vd2.layer1 == l);
        let wl = |b: &Rect| (b.dx().min(b.dy()), b.dx().max(b.dy()));
        let ((w1, len1), (w2, len2)) = (wl(&b1), wl(&b2));
        let triggers = |m: &MinCut, w: i32, len: i32| m.length.is_none_or(|(ml, _)| len > ml) && w > m.width;
        let side_ok = |m: &MinCut, other_above: bool| m.from_above.is_none_or(|a| a == other_above);
        let req = |m: &MinCut, cut: &Rect, pad: &Rect| {
            let d = m.length.map_or(0, |(_, dist)| dist);
            d + if horizontal { (cut.xh - pad.xl).max(pad.xh - cut.xl) } else { (cut.yh - pad.yl).max(pad.yh - cut.yl) }
        };
        for m in &tech.layers[l].min_cuts {
            if triggers(m, w1, len1) {
                if !side_ok(m, above2) {
                    continue;
                }
                out.push((0, req(m, &c2, &b1)));
            }
            if triggers(m, w2, len2) {
                if !side_ok(m, above1) {
                    continue;
                }
                out.push((0, req(m, &c1, &b2)));
            }
        }
    }

    fn via2via_cut_spc(&self, v1: usize, v2: usize, along_x: bool, out: &mut Ranges) {
        let tech = self.tech;
        let (vd1, vd2) = (&tech.via_defs[v1], &tech.via_defs[v2]);
        let cut1 = bbox(&vd1.cut_figs);
        // Same cut layer: its (different-net, edge-to-edge) spacing plus the cut's length along.
        if vd1.cut == vd2.cut {
            if let Some(s) = tech.layers[vd1.cut].cut_spacing {
                out.push((0, s + if along_x { cut1.dx() } else { cut1.dy() }));
            }
        }
        // Different cut layers: spacing between cut layers is not modelled.
    }

    /// `prep_via2viaForbiddenLen_lef58CutSpcTbl`: two cuts on the same layer under its LEF58
    /// different-net cut spacing table — the spacing between their classes (`getSpacing`'s MAX of
    /// first and second, END/SIDE by each cut's orientation across the direction) plus the cut's
    /// length along it (edge to edge); centre-and-edge: the larger of MAX and MIN plus the two half
    /// lengths; centre to centre: the spacing alone. Forbidden from 0. (Cuts on different layers
    /// read only the SAMEMETAL / SAMENET inter-layer tables, refused.)
    fn via2via_lef58_cut_spc_tbl(&self, v1: usize, v2: usize, along_x: bool, out: &mut Ranges) {
        let tech = self.tech;
        let (vd1, vd2) = (&tech.via_defs[v1], &tech.via_defs[v2]);
        // The reference swaps the two when the second's cut layer is higher; on one layer never.
        if vd1.cut != vd2.cut {
            return;
        }
        let Some(tb) = tech.layers[vd1.cut].cut_table.as_ref() else { return };
        let (cb1, cb2) = (bbox(&vd1.cut_figs), bbox(&vd2.cut_figs));
        let layer = &tech.layers[vd1.cut];
        let class = |r: &Rect| layer.cut_class_of(r.dx().min(r.dy()), r.dx().max(r.dy()));
        let (c1, c2) = (class(&cb1), class(&cb2));
        let side = |r: &Rect| if along_x { r.dx() < r.dy() } else { r.dx() > r.dy() };
        let (first, second) = tb.get(c1, side(&cb1), c2, side(&cb2));
        let pair = tb.pair(c1, c2);
        let along = |r: &Rect| if along_x { r.dx() } else { r.dy() };
        let mut req = first.max(second);
        if !tb.center_to_center[pair] && !tb.center_and_edge[pair] {
            req += along(&cb1);
        } else if tb.center_and_edge[pair] {
            req = first.max(second).max(first.min(second) + (along(&cb1) + along(&cb2)) / 2);
        }
        if req != 0 {
            out.push((0, req));
        }
    }

    /// `prep_via2viaForbiddenLen_minStep`: one via below and one above layer `l` (their pads on it):
    /// the narrower pad across `vertical` shifts along it until the joined shape no longer breaks
    /// the layer's MINSTEP (`hasMinStepViol`, the check's own walk) — the distances where it does
    /// are forbidden, `[minRange − 1, minRange + shift + 1]`.
    fn via2via_min_step(&self, l: usize, v1: usize, v2: usize, vertical: bool, out: &mut Ranges) {
        let tech = self.tech;
        let Some(con) = tech.layers[l].min_step else { return };
        let (vd1, vd2) = (&tech.via_defs[v1], &tech.via_defs[v2]);
        if vd1.layer1 == vd2.layer1 {
            return;
        }
        let (e1, e2) = if vd1.layer1 == l { (vd1.layer1_bbox(), vd2.layer2_bbox()) } else { (vd1.layer2_bbox(), vd2.layer1_bbox()) };
        let across = |r: &Rect| if vertical { r.dx() } else { r.dy() };
        let (mut sh, other) = if across(&e1) < across(&e2) {
            (e1, e2)
        } else if across(&e2) < across(&e1) {
            (e2, e1)
        } else {
            return;
        };
        let mv = |r: &mut Rect, d: i32| {
            if vertical {
                r.yl += d;
                r.yh += d;
            } else {
                r.xl += d;
                r.xh += d;
            }
        };
        let viol = |a: &Rect| has_min_step_viol(a, &other, &con);
        let mut min_range = 0;
        if other.xl <= sh.xl && other.yl <= sh.yl && sh.xh <= other.xh && sh.yh <= other.yh {
            min_range = if vertical { other.yh - sh.yh + 1 } else { other.xh - sh.xh + 1 };
            mv(&mut sh, min_range);
        }
        let (sh_edge, other_edge, sh_low, other_low, other_high) = if vertical {
            (sh.yh - other.yh, other.xh - sh.xh, sh.yl, other.yl, other.yh)
        } else {
            (sh.xh - other.xh, other.yh - sh.yh, sh.xl, other.xl, other.xh)
        };
        let msl = con.min_step_length;
        let shift;
        if viol(&sh) {
            if sh_edge < msl {
                let mut s = msl - sh_edge - 1;
                if sh_low < other_low {
                    s = s.max(other_low - sh_low - 1);
                }
                mv(&mut sh, s + 1);
                if viol(&sh) {
                    s = other_high - sh_low;
                }
                shift = s;
            } else {
                shift = other_high - sh_low;
            }
        } else if sh_edge < msl {
            if con.max_length <= 0 {
                return;
            }
            let mut div = 2;
            let mut length = sh_edge;
            let top_sh = if vertical { sh.dx() } else { sh.dy() };
            let top_other = if vertical { other.dy() } else { other.dx() };
            if top_sh < msl {
                length += top_sh + sh_edge;
                if other_edge < msl {
                    length += 2 * other_edge;
                    if top_other < msl {
                        return;
                    }
                }
            } else if other_edge < msl {
                length += other_edge;
                div = 1;
                if top_other < msl {
                    length += top_other + other_edge + sh_edge;
                }
            }
            let s = (con.max_length - length) / div + 1;
            if s < 0 {
                return;
            }
            mv(&mut sh, s);
            if !viol(&sh) {
                return;
            }
            min_range = s;
            shift = other_high - sh_low;
        } else {
            min_range = other_low - sh_low - msl + 1;
            mv(&mut sh, min_range);
            if !viol(&sh) {
                return;
            }
            let mut s = msl - 2;
            mv(&mut sh, s + 1);
            if viol(&sh) {
                s = other_high - sh_low;
            }
            shift = s;
        }
        out.push((min_range - 1, min_range + shift + 1));
    }

    /// `prep_minStepViasCheck`: per routing layer with a layer below and above it in the technology
    /// and a MINSTEP rule, whether the default vias' pads below and above it, joined, break the rule
    /// (the reference then patches such shapes in its check — `patchMetalShape_minStep`).
    pub fn min_step_via_flags(&self) -> Vec<bool> {
        let tech = self.tech;
        let top = tech.top_layer_num();
        (0..tech.layers.len())
            .map(|l| {
                if tech.layers[l].kind != LayerKind::Routing || l < 2 || l + 2 > top {
                    return false;
                }
                let (Some(down), Some(up)) = (self.default_at(l as i64 - 1), self.default_at(l as i64 + 1)) else { return false };
                let Some(con) = tech.layers[l].min_step else { return false };
                has_min_step_viol(&tech.via_defs[up].layer1_bbox(), &tech.via_defs[down].layer2_bbox(), &con)
            })
            .collect()
    }

    fn via2via_prl(&self, l: usize, v1: usize, v2: usize, along_x: bool) -> i32 {
        let (b1, b2) = (enclosure(&self.tech.via_defs[v1], l), enclosure(&self.tech.via_defs[v2], l));
        if along_x {
            (b1.dx() + b2.dx()) / 2
        } else {
            (b1.dy() + b2.dy()) / 2
        }
    }

    fn via2via_entry(&self, l: usize, v1: Option<usize>, v2: Option<usize>, along_x: bool, ndr: Option<&NdrRule>) -> (Ranges, i32) {
        let mut r = Ranges::new();
        if let (Some(a), Some(b)) = (v1, v2) {
            self.via2via_min_spc(l, a, b, along_x, &mut r, ndr);
            self.via2via_min_cut(l, a, b, &mut r);
            self.via2via_cut_spc(a, b, along_x, &mut r);
            self.via2via_lef58_cut_spc_tbl(a, b, along_x, &mut r);
            self.via2via_min_step(l, a, b, !along_x, &mut r);
        }
        let prl = match (v1, v2) {
            (Some(a), Some(b)) => self.via2via_prl(l, a, b, along_x),
            _ => 0,
        };
        (merge(&r), prl)
    }

    fn via2via_forbidden_len(&self, ndr: Option<&NdrRule>) -> Vec<([Ranges; 8], [i32; 8])> {
        self.routing_layers()
            .into_iter()
            .map(|l| {
                let (down, up) = self.down_up(l, ndr);
                let pairs = [(down, down, true), (down, down, false), (down, up, true), (down, up, false), (up, down, true), (up, down, false), (up, up, true), (up, up, false)];
                let mut t: [Ranges; 8] = Default::default();
                let mut p = [0i32; 8];
                for (k, &(a, b, x)) in pairs.iter().enumerate() {
                    let (r, prl) = self.via2via_entry(l, a, b, x, ndr);
                    t[k] = r;
                    p[k] = prl;
                }
                (t, p)
            })
            .collect()
    }

    // ---- via turn ----

    fn via_turn_min_spc(&self, l: usize, v: usize, along_x: bool, ndr: Option<&NdrRule>) -> Option<(i32, i32)> {
        let tech = self.tech;
        let default_width = tech.layers[l].width;
        let width = ndr.map_or(default_width, |n| default_width.max(n.width(z_of(l))));
        let b = enclosure(&tech.via_defs[v], l);
        let w1 = b.dx().min(b.dy());
        let fat = if along_x { b.dy() > default_width } else { b.dx() > default_width };
        let prl = if along_x { b.dy() } else { b.dx() };
        let non_overlap = if along_x { (b.dx() + width) / 2 } else { (b.dy() + width) / 2 };
        if !(fat || ndr.is_some()) {
            return None;
        }
        let mut req = min_spacing(tech, l, w1.max(width), prl);
        if let Some(n) = ndr {
            req = Some(req.map_or(n.spacing(z_of(l)), |r| r.max(n.spacing(z_of(l)))));
        }
        req.map(|r| (non_overlap, r + non_overlap))
    }

    fn via_forbidden_turn_len(&self, ndr: Option<&NdrRule>) -> Vec<[Ranges; 4]> {
        self.routing_layers()
            .into_iter()
            .map(|l| {
                let (down, up) = self.down_up(l, ndr);
                let mut t: [Ranges; 4] = Default::default();
                for (k, &(v, x)) in [(down, true), (down, false), (up, true), (up, false)].iter().enumerate() {
                    if let Some(v) = v {
                        let r: Ranges = self.via_turn_min_spc(l, v, x, ndr).into_iter().collect();
                        t[k] = merge_shrunk(&r);
                    }
                }
                t
            })
            .collect()
    }

    // ---- line to line ----

    fn line_min_spc(&self, l: usize, z_shape: bool) -> Option<(i32, i32)> {
        let layer = &self.tech.layers[l];
        let w = layer.width;
        let prl = if z_shape { w } else { layer.pitch };
        min_spacing(self.tech, l, w, prl).map(|s| (w, s + w))
    }

    fn line_forbidden_len(&self) -> Vec<[Ranges; 4]> {
        self.routing_layers()
            .into_iter()
            .map(|l| {
                let mut t: [Ranges; 4] = Default::default();
                for (k, z) in [true, true, false, false].into_iter().enumerate() {
                    let r: Ranges = self.line_min_spc(l, z).into_iter().collect();
                    t[k] = merge_shrunk(&r);
                }
                t
            })
            .collect()
    }

    // ---- end of line ----

    /// `getMinEol`: the narrowest end-of-line width — over the EOL, LEF58 EOL spacing and LEF58
    /// keep-out rules — less one, not below `min_width`; none, `min_width`.
    fn min_eol(&self, l: usize, min_width: i32) -> i32 {
        let t = &self.tech.layers[l];
        match t.eol.iter().chain(&t.lef58_eol).map(|e| e.width).chain(t.eol_keepout.iter().map(|k| k.width)).min() {
            None => min_width,
            Some(e) => (e - 1).max(min_width),
        }
    }

    /// `prep_eolForbiddenLen_helper`, over every rule wider than the table's width: EOL (space,
    /// within), LEF58 EOL spacing (space, within, and its END-TO-END space into the space), then
    /// LEF58 keep-out (FORWARD extension into the space, SIDE extension into the within).
    fn eol_table(&self, l: usize, min_width: i32) -> EolTable {
        let width = self.min_eol(l, min_width);
        let t = &self.tech.layers[l];
        let (mut space, mut within) = (0, 0);
        for e in t.eol.iter().chain(&t.lef58_eol) {
            if width < e.width {
                space = space.max(e.space).max(e.end_to_end.unwrap_or(0));
                within = within.max(e.within);
            }
        }
        for k in &t.eol_keepout {
            if width < k.width {
                space = space.max(k.forward);
                within = within.max(k.side);
            }
        }
        EolTable { width, space, within }
    }

    // ---- via through ----

    fn via_forbidden_through(&self) -> Vec<[bool; 4]> {
        self.routing_layers()
            .into_iter()
            .map(|l| {
                let (down, up) = self.down_up(l, None);
                // One named via on one layer (a single technology's rule); otherwise never.
                let f = |v: Option<usize>| v.is_some_and(|v| l == 10 && self.tech.via_defs[v].name == "CK_23_28_0_26_VH_CK");
                [f(down), f(down), f(up), f(up)]
            })
            .collect()
    }
}

/// Every table, in the order they are computed.
/// `prep_cutSpcTbl`: on each cut layer with a default via, the LEF58 different-net cut spacing
/// table's DEFAULT (what the maze's cut cost reads) is REPLACED by the table's maximum for the
/// default via's cut class against itself (`getMaxSpacing`, the most of END/SIDE either side),
/// first and second, and that pair's centre flags — io's raw first entry stays only where the
/// layer has no default via. (The inter-layer table's branch is not modelled: those are refused.)
pub fn prep_cut_spc_tbl(tech: &mut Tech, defaults: &[Option<usize>]) {
    for l in 0..tech.layers.len() {
        if tech.layers[l].kind != LayerKind::Cut {
            continue;
        }
        let Some(v) = defaults.get(l).copied().flatten() else { continue };
        let cb = bbox(&tech.via_defs[v].cut_figs);
        let c = tech.layers[l].cut_class_of(cb.dx().min(cb.dy()), cb.dx().max(cb.dy()));
        let Some(tb) = tech.layers[l].cut_table.as_mut() else { continue };
        let sides = [(true, true), (true, false), (false, true), (false, false)];
        let first = sides.iter().map(|&(a, b)| tb.get(c, a, c, b).0).max().unwrap_or(0);
        let second = sides.iter().map(|&(a, b)| tb.get(c, a, c, b).1).max().unwrap_or(0);
        let pair = tb.pair(c, c);
        tb.default_spacing = (first, second);
        tb.default_center_to_center = tb.center_to_center[pair];
        tb.default_center_and_edge = tb.center_and_edge[pair];
    }
}

pub fn rule_tables(tech: &Tech, defaults: &[Option<usize>], ndrs: &[NdrRule], cfg: &RuleConfig) -> RuleTables {
    let c = Ctx { tech, defaults, cfg };
    let v2v = c.via2via_forbidden_len(None);
    let turn = c.via_forbidden_turn_len(None);
    let line = c.line_forbidden_len();
    let through = c.via_forbidden_through();
    let layers: Vec<LayerTables> = (0..v2v.len())
        .map(|i| LayerTables { via2via: v2v[i].0.clone(), via2via_prl: v2v[i].1, turn: turn[i].clone(), planar: Default::default(), line: line[i].clone(), through: through[i] })
        .collect();
    let routing = c.routing_layers();
    let eol = routing.iter().map(|&l| c.eol_table(l, tech.layers[l].width)).collect();
    let ndr_eol = |n: &NdrRule| -> Vec<Option<EolTable>> {
        routing
            .iter()
            .map(|&l| {
                let w = n.width(z_of(l));
                (w != 0).then(|| c.eol_table(l, w))
            })
            .collect()
    };
    let ndrs = ndrs
        .iter()
        .map(|n| NdrTables { name: n.name.clone(), via2via: c.via2via_forbidden_len(Some(n)).into_iter().map(|(t, _)| t).collect(), turn: c.via_forbidden_turn_len(Some(n)), eol: ndr_eol(n) })
        .collect();
    RuleTables { layers, eol, ndrs, min_step_via: c.min_step_via_flags() }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tech::{EolKeepOut, EolRule, Layer};

    fn r(xl: i32, yl: i32, xh: i32, yh: i32) -> Rect {
        Rect { xl, yl, xh, yh }
    }

    /// 0–1 placeholders, 2 horizontal routing, 3 cut, 4 vertical routing: width 100, pitch 200,
    /// spacing 50 (70 for widths above 100 over runs above 150); cut spacing 80.
    fn tech(vias: Vec<ViaDef>) -> Tech {
        let mut t = Tech::default();
        let table = SpacingTable { widths: vec![0, 100], prls: vec![0, 150], values: vec![vec![50, 50], vec![50, 70]] };
        let routing = |dir| Layer { kind: LayerKind::Routing, dir, width: 100, min_width: 100, pitch: 200, spacing: Some(table.clone()), ..Default::default() };
        t.layers = vec![Layer::default(), Layer::default(), routing(Dir::Horizontal), Layer { kind: LayerKind::Cut, cut_spacing: Some(80), ..Default::default() }, routing(Dir::Vertical)];
        t.via_defs = vias;
        t
    }

    fn via(name: &str, lower: Rect, upper: Rect) -> ViaDef {
        ViaDef { name: name.into(), is_default: true, layer1: 2, cut: 3, layer2: 4, layer1_figs: vec![lower], cut_figs: vec![r(-40, -40, 40, 40)], layer2_figs: vec![upper] }
    }

    fn cfg(bottom: usize, top: usize) -> RuleConfig {
        RuleConfig { bottom_routing_layer: bottom, top_routing_layer: top, enable_via_gen: true }
    }

    // Touching closed intervals join; an empty one (low above high) is dropped.
    #[test]
    fn ranges_join_when_touching_and_drop_empties() {
        assert_eq!(merge(&[(5, 3), (4, 6), (1, 3), (9, 9), (20, 15)]), vec![(1, 6), (9, 9)]);
    }

    // Shrinking can leave a range empty — it is kept.
    #[test]
    fn a_shrunk_range_may_be_empty_and_stays() {
        assert_eq!(merge_shrunk(&[(3, 4), (10, 20)]), vec![(4, 3), (11, 19)]);
    }

    // A default via whose lower shape runs ACROSS its horizontal layer is replaced, within the
    // routing layers, by a generated via turned a quarter; outside them it is kept.
    #[test]
    fn a_misaligned_default_via_is_generated_turned() {
        let v = via("v", r(-50, -100, 50, 100), r(-50, -100, 50, 100));
        let mut t = tech(vec![v.clone()]);
        let d = default_vias(&mut t, &cfg(2, 4));
        assert_eq!(d[3], Some(1));
        assert_eq!(t.via_defs[1].name, "_FR");
        assert_eq!(t.via_defs[1].layer1_figs, vec![r(-100, -50, 100, 50)]);
        assert_eq!(t.via_defs[1].layer2_figs, vec![r(-50, -100, 50, 100)]);
        let mut t = tech(vec![v.clone()]);
        assert_eq!(default_vias(&mut t, &cfg(4, 4))[3], Some(0));
        assert_eq!(t.via_defs.len(), 1);
        // Above the top routing layer too.
        let mut t = tech(vec![v]);
        assert_eq!(default_vias(&mut t, &cfg(2, 2))[3], Some(0));
        assert_eq!(t.via_defs.len(), 1);
    }

    // Aligned or SQUARE shapes keep the technology's via (a square on a horizontal layer is not
    // "across" it).
    #[test]
    fn an_aligned_default_via_is_kept() {
        let mut t = tech(vec![via("v", r(-50, -50, 50, 50), r(-50, -100, 50, 100))]);
        assert_eq!(default_vias(&mut t, &cfg(2, 4))[3], Some(0));
        assert_eq!(t.via_defs.len(), 1);
    }

    // The end-of-line width is the narrowest rule's less one (not below the minimum); only rules
    // wider than that count, each taking the largest space and within.
    #[test]
    fn the_eol_rule_is_the_narrowest_less_one() {
        let mut t = tech(vec![]);
        t.layers[2].eol = vec![EolRule { space: 60, width: 140, within: 20, parallel: None, end_to_end: None }, EolRule { space: 90, width: 180, within: 10, parallel: None, end_to_end: None }];
        let c = Ctx { tech: &t, defaults: &[], cfg: &cfg(2, 4) };
        assert_eq!(c.eol_table(2, 100), EolTable { width: 139, space: 90, within: 20 });
        assert_eq!(c.eol_table(2, 150), EolTable { width: 150, space: 90, within: 10 });
        // A rule exactly as wide as the end does not count (strictly narrower ends only).
        assert_eq!(c.eol_table(2, 180), EolTable { width: 180, space: 0, within: 0 });
        t.layers[2].eol.clear();
        let c = Ctx { tech: &t, defaults: &[], cfg: &cfg(2, 4) };
        assert_eq!(c.eol_table(2, 100), EolTable { width: 100, space: 0, within: 0 });
    }

    // `getMinEol` / `prep_eolForbiddenLen_helper` read the LEF58 rules too: a LEF58 EOL rule's
    // end-to-end space counts as its space, and a keep-out's forward and side extensions as space
    // and within (asap7 M2: SPACING 18 ENDOFLINE 25 WITHIN 20 ENDTOEND 31, keep-out 25 / 0 12.5 31).
    #[test]
    fn the_eol_table_reads_the_lef58_eol_and_keep_out_rules() {
        let mut t = tech(vec![]);
        t.layers[2].lef58_eol = vec![EolRule { space: 72, width: 100, within: 80, parallel: None, end_to_end: Some(124) }];
        t.layers[2].eol_keepout = vec![EolKeepOut { width: 90, backward: 0, forward: 50, side: 124, corner_only: true, ..Default::default() }];
        let c = Ctx { tech: &t, defaults: &[], cfg: &cfg(2, 4) };
        // Narrowest 90 less one; both rules wider: space max(72, 124, 50), within max(80, 124).
        assert_eq!(c.eol_table(2, 40), EolTable { width: 89, space: 124, within: 124 });
        // 95 is not below the keep-out's 90: the LEF58 EOL rule alone.
        assert_eq!(c.eol_table(2, 95), EolTable { width: 95, space: 124, within: 80 });
    }

    // A via-turn range only for a via wider than the wire across the turn (or under a
    // non-default rule).
    #[test]
    fn a_turn_range_needs_a_fat_via() {
        let t = tech(vec![via("thin", r(-50, -50, 50, 50), r(-50, -50, 50, 50)), via("fat", r(-100, -100, 100, 100), r(-50, -50, 50, 50))]);
        let c = Ctx { tech: &t, defaults: &[], cfg: &cfg(2, 4) };
        assert_eq!(c.via_turn_min_spc(2, 0, true, None), None);
        // Fat: across 200 > 100, run 200 → the wide row, the long column: 70; half of 200 + 100.
        assert_eq!(c.via_turn_min_spc(2, 1, true, None), Some((150, 220)));
        let ndr = NdrRule { name: "n".into(), widths: vec![100, 100], spacings: vec![90, 0], vias: vec![] };
        assert_eq!(c.via_turn_min_spc(2, 0, true, Some(&ndr)), Some((100, 190)));
    }

    // The same via twice also needs its spacing on its OTHER metal layer; and the cut layer's
    // spacing plus the cut's length.
    #[test]
    fn the_same_via_twice_counts_both_metals_and_the_cut() {
        let t = tech(vec![via("v", r(-100, -100, 100, 100), r(-50, -50, 50, 50))]);
        let c = Ctx { tech: &t, defaults: &[None, None, None, Some(0), None], cfg: &cfg(2, 4) };
        let mut out = Ranges::new();
        c.via2via_min_spc(2, 0, 0, true, &mut out, None);
        // Layer 2: both fat, width 200, run 200 → 70 + 200; layer 4: width 100 → 50 + 100.
        assert_eq!(out, vec![(200, 270), (100, 150)]);
        let mut out = Ranges::new();
        c.via2via_cut_spc(0, 0, true, &mut out);
        assert_eq!(out, vec![(0, 160)]);
    }

    // Rule (`prep_cutSpcTbl`): a cut layer with a default via takes, as its table default, the
    // most of END/SIDE either side for the via's class against itself, first and second; a layer
    // without one keeps io's value.
    #[test]
    fn the_cut_table_default_is_the_default_via_class_maximum() {
        let mut t = tech(vec![via("v", r(-100, -100, 100, 100), r(-50, -50, 50, 50))]);
        // One class (no class): entries (c1, s1, c2, s2) = END/SIDE pairs.
        let spacing = vec![(10, 20), (30, 5), (7, 40), (1, 2)];
        t.layers[3].cut_table = Some(crate::tech::CutSpacingTable { n: 1, spacing, center_to_center: vec![true], center_and_edge: vec![false], default_spacing: (99, 99), ..Default::default() });
        let mut without = t.clone();
        prep_cut_spc_tbl(&mut t, &[None, None, None, Some(0), None]);
        let tb = t.layers[3].cut_table.as_ref().unwrap();
        assert_eq!((tb.default_spacing, tb.default_center_to_center), ((30, 40), true));
        prep_cut_spc_tbl(&mut without, &[None, None, None, None, None]);
        assert_eq!(without.layers[3].cut_table.as_ref().unwrap().default_spacing, (99, 99));
    }

    // Rule (`prep_via2viaForbiddenLen_lef58CutSpcTbl`): two cuts on one layer under its LEF58
    // different-net cut spacing table are forbidden from 0 to the classes' spacing (MAX of first
    // and second) plus the cut's length along (edge to edge); centre-and-edge: the larger of MAX
    // and MIN plus the two half lengths; centre to centre: the spacing alone. The cut is 80 long.
    #[test]
    fn a_cut_spacing_table_forbids_via_to_via_from_zero() {
        let range = |c2c: bool, cae: bool| {
            let mut t = tech(vec![via("v", r(-100, -100, 100, 100), r(-50, -50, 50, 50))]);
            t.layers[3].cut_spacing = None;
            t.layers[3].cut_table = Some(crate::tech::CutSpacingTable { n: 1, spacing: vec![(30, 50); 4], center_to_center: vec![c2c], center_and_edge: vec![cae], ..Default::default() });
            let c = Ctx { tech: &t, defaults: &[None, None, None, Some(0), None], cfg: &cfg(2, 4) };
            let mut out = Ranges::new();
            c.via2via_lef58_cut_spc_tbl(0, 0, true, &mut out);
            out
        };
        assert_eq!(range(false, false), vec![(0, 130)]);
        assert_eq!(range(false, true), vec![(0, 110)]);
        assert_eq!(range(true, false), vec![(0, 50)]);
    }

    // Via to via on one layer: only when BOTH vias are wider than the wire across the run, at
    // the wider via's width and the SHORTER run (here 120, below the table's 150 column).
    #[test]
    fn via_to_via_needs_both_fat_and_takes_the_shorter_run() {
        let t = tech(vec![via("big", r(-100, -100, 100, 100), r(-50, -50, 50, 50)), via("short", r(-100, -60, 100, 60), r(-50, -50, 50, 50)), via("thin", r(-100, -50, 100, 50), r(-50, -50, 50, 50))]);
        let c = Ctx { tech: &t, defaults: &[], cfg: &cfg(2, 4) };
        let mut out = Ranges::new();
        c.via2via_min_spc(2, 0, 1, true, &mut out, None);
        assert_eq!(out, vec![(200, 250)]);
        let mut out = Ranges::new();
        c.via2via_min_spc(2, 0, 2, true, &mut out, None);
        assert_eq!(out, vec![]);
    }

    // Line to line: a z-shape runs alongside for the wire's width, a u-shape for the pitch.
    #[test]
    fn line_ranges_use_width_then_pitch() {
        let mut t = tech(vec![]);
        t.layers[2].spacing = Some(SpacingTable { widths: vec![0], prls: vec![0, 150], values: vec![vec![50, 70]] });
        let c = Ctx { tech: &t, defaults: &[], cfg: &cfg(2, 4) };
        assert_eq!(c.line_min_spc(2, true), Some((100, 150)));
        assert_eq!(c.line_min_spc(2, false), Some((100, 170)));
    }
}
