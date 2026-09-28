// SPDX-License-Identifier: Apache-2.0
//! The technology and design as the router reads them.
//!
//! Rules:
//! - layers are numbered from the first routing layer, with a placeholder masterslice (0) and a
//!   placeholder cut layer (1) below it; routing and cut layers then follow in technology order;
//!   the masterslice placeholder takes the name of the last masterslice layer before the first
//!   routing layer that is not a well or diffusion layer (LEF58 type), else `FR_MASTERSLICE`;
//! - a routing layer's min width is `min(min width, width)`;
//! - a via definition is a technology via: its shapes on the layer below, the cut and the layer
//!   above, and whether it is a default via;
//! - a routing layer's minimum spacing is its parallel-run spacing table; a plain SPACING value is
//!   a table of one entry (width 0, run length 0);
//! - a cut layer's spacing is its plain SPACING value, edge to edge.

use crate::polygon90::{Polygon90Set, Rect};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LayerKind {
    Routing,
    Cut,
    /// The placeholders below the first routing layer.
    #[default]
    Placeholder,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Dir {
    Horizontal,
    Vertical,
    #[default]
    None,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Layer {
    pub name: String,
    pub kind: LayerKind,
    pub dir: Dir,
    pub width: i32,
    pub min_width: i32,
    pub pitch: i32,
    /// The width of a wire run across the layer's direction.
    pub wrong_way_width: i32,
    /// A routing layer's minimum spacing.
    pub spacing: Option<SpacingTable>,
    /// A cut layer's minimum spacing, edge to edge.
    pub cut_spacing: Option<i32>,
    /// A cut layer's LEF58 cut classes, in the technology's order.
    pub cut_classes: Vec<CutClass>,
    /// A cut layer's LEF58 different-net cut spacing table (the only kind modelled).
    pub cut_table: Option<CutSpacingTable>,
    /// A routing layer's end-of-line spacing rules, in the technology's order.
    pub eol: Vec<EolRule>,
    /// A routing layer's LEF58 end-of-line SPACING rules, in the technology's order (the
    /// modelled subset — `lef58_eol_rule`; the census refuses the rest).
    pub lef58_eol: Vec<EolRule>,
    /// A routing layer's LEF58 end-of-line KEEP-OUT rules, in the technology's order.
    pub eol_keepout: Vec<EolKeepOut>,
    /// A routing layer's LEF58 CONVEX corner spacing rules, in the technology's order.
    pub corner_spacing: Vec<CornerSpacing>,
    /// A routing layer's own minimum AREA (square database units; 0 without one).
    pub min_area: i64,
    /// MINENCLOSEDAREA rules without a width (`frMinEnclosedAreaConstraint`): each rule's area,
    /// narrowed to `frCoord` (a 32-bit int) as `io::Parser` stores it.
    pub min_enclosed_areas: Vec<i32>,
    /// A rect-only routing layer: every polygon of a net on it must be one rectangle (where its
    /// own fixed shapes do not already break that), and it is UNIDIRECTIONAL.
    pub rect_only: bool,
    /// The layer carries the right-way-on-grid-only constraint: io makes it for RIGHTWAYONGRIDONLY
    /// OR a multi-patterned layer (`numMasks > 1`, refused before routing). Its consumers: pin
    /// access (on-grid points only), the maze grid (right-way edges on tracks only; an off-grid via
    /// only at an access point) and, through `Tech::allow_pin_feedthrough`, guides and the maze.
    pub right_way_on_grid_only: bool,
}

/// An end-of-line spacing rule: a line end narrower than `width` needs `space` to a facing edge
/// within `within` beyond its sides; with a parallel edge, only when a parallel edge lies within
/// `par_space` of a side (on both sides with `two_edges`), up to `par_within` behind the end.
/// `end_to_end` (LEF58 only): the spacing when the facing edge is itself a line end.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EolRule {
    pub space: i32,
    pub width: i32,
    pub within: i32,
    pub parallel: Option<ParallelEdge>,
    pub end_to_end: Option<i32>,
}


/// A LEF58 end-of-line keep-out rule (`frLef58EolKeepOutConstraint`, as `io::Parser` reads it —
/// the class name is not read): a line end narrower than `width` keeps other metal out of a box
/// `forward` beyond it, `backward` behind it and `side` past each side; `corner_only` looks only
/// for other polygons' corners there; `except_within` excuses metal in the side windows
/// `within_low..within_high` from the line end.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct EolKeepOut {
    pub width: i32,
    pub backward: i32,
    pub forward: i32,
    pub side: i32,
    pub corner_only: bool,
    pub except_within: bool,
    pub within_low: i32,
    pub within_high: i32,
}

/// A LEF58 convex corner spacing rule (`frLef58CornerSpacingConstraint`, as `io::Parser` reads
/// it): per WIDTH row a spacing pair; `same_xy` when every pair's two values agree (the check only
/// runs then); `corner_to_corner` measures the corner's Euclidean distance instead of its larger
/// axis gap. CORNERONLY's within, EXCEPTSAMENET and EXCEPTSAMEMETAL are stored by the reference and
/// never read, so they are not kept.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct CornerSpacing {
    pub widths: Vec<i32>,
    pub spacings: Vec<(i32, i32)>,
    pub same_xy: bool,
    pub corner_to_corner: bool,
}

impl CornerSpacing {
    /// `fr1DLookupTbl::find` (lower-bound mode): within the rows, the row BEFORE the first row not
    /// below `width` (the first row itself when that is it) — so a width equal to a row takes the
    /// previous row; below the rows the first, above them the last.
    pub fn find(&self, width: i32) -> (i32, i32) {
        let (first, last) = (self.widths[0], self.widths[self.widths.len() - 1]);
        let idx = if width >= first && width <= last {
            self.widths.partition_point(|&w| w < width).saturating_sub(1)
        } else if width < first {
            0
        } else {
            self.widths.len() - 1
        };
        self.spacings[idx]
    }

    /// `fr1DLookupTbl::findMax`: the LAST row's pair (not the largest).
    pub fn find_max(&self) -> (i32, i32) {
        self.spacings[self.spacings.len() - 1]
    }
}

/// A LEF58 cut class (`frLef58CutClass`): a cut `width` by `length` (the width when the class
/// gives no length).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct CutClass {
    pub name: String,
    pub width: i32,
    pub length: i32,
}

impl Layer {
    /// `frLayer::getCutClassIdx(width, length)`: the LAST class of exactly that size (no early
    /// break), as a class index of the layer's cut spacing table (0: no class).
    pub fn cut_class_of(&self, width: i32, length: i32) -> usize {
        self.cut_classes.iter().rposition(|c| c.width == width && c.length == length).map_or(0, |i| i + 1)
    }
}

/// A LEF58 different-net cut spacing table (`frLef58CutSpacingTableConstraint`), its lookups —
/// the database rule's own `getSpacing`, `getMaxSpacing`, `getPrlEntry`, … — evaluated when the
/// technology is read for every pair of the layer's classes, class 0 being "no class" (the
/// empty name, which falls to the table's default). Indices: class `c`, side `s` (0 END, 1 SIDE).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct CutSpacingTable {
    /// The number of classes, "no class" included.
    pub n: usize,
    /// `getSpacing(c1, s1, c2, s2, FIRST / SECOND)`.
    pub spacing: Vec<(i32, i32)>,
    /// `getMaxSpacing(c, SIDE)` per class: [END, SIDE].
    pub max_spacing: Vec<[i32; 2]>,
    /// Per class pair: `getPrlEntry`, `isCenterToCenter`, `isCenterAndEdge`,
    /// `isPrlForAlignedCutClasses`.
    pub prl_entry: Vec<i32>,
    pub center_to_center: Vec<bool>,
    pub center_and_edge: Vec<bool>,
    pub prl_aligned: Vec<bool>,
    /// `getExactAlignedSpacing(c)` per class (-1: none).
    pub exact_aligned: Vec<i32>,
    pub no_prl: bool,
    pub horizontal: bool,
    pub vertical: bool,
    /// What the maze's cut cost reads (`getDefaultSpacing`, `getDefaultCenterToCenter`,
    /// `getDefaultCenterAndEdge`), as io sets them: the raw table's `[0][0]` pair (first, second),
    /// and the flags of the alphabetically FIRST column and row class names (`std::map::begin`,
    /// a `/SIDE` suffix dropped) — not necessarily the classes at `[0][0]`.
    pub default_spacing: (i32, i32),
    pub default_center_to_center: bool,
    pub default_center_and_edge: bool,
}

impl CutSpacingTable {
    pub fn get(&self, c1: usize, side1: bool, c2: usize, side2: bool) -> (i32, i32) {
        self.spacing[((c1 * 2 + usize::from(side1)) * self.n + c2) * 2 + usize::from(side2)]
    }
    pub fn pair(&self, c1: usize, c2: usize) -> usize {
        c1 * self.n + c2
    }
    /// `getMaxSpacing(c1, c2, MAX)`: the largest of the four side combinations' larger value.
    pub fn max_pair_spacing(&self, c1: usize, c2: usize) -> i32 {
        [(true, true), (true, false), (false, true), (false, false)].iter().map(|&(a, b)| { let (f, s) = self.get(c1, a, c2, b); f.max(s) }).max().unwrap_or(0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParallelEdge {
    pub space: i32,
    pub within: i32,
    pub two_edges: bool,
}

/// A parallel-run spacing table: rows by width, columns by parallel run length.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SpacingTable {
    pub widths: Vec<i32>,
    pub prls: Vec<i32>,
    pub values: Vec<Vec<i32>>,
}

impl SpacingTable {
    /// The spacing for a width and a run length: the row of the LAST width strictly below
    /// `width` and the column of the last run length strictly below `prl` — the first row or
    /// column when there is none. ⛔ Strictly below: a width equal to a row's is the row before.
    pub fn find(&self, width: i32, prl: i32) -> i32 {
        let idx = |axis: &[i32], v: i32| axis.iter().filter(|&&a| a < v).count().saturating_sub(1);
        self.values[idx(&self.widths, width)][idx(&self.prls, prl)]
    }
    /// The first row's first value.
    pub fn find_min(&self) -> i32 {
        self.values[0][0]
    }
    /// The last row's last value.
    pub fn find_max(&self) -> i32 {
        *self.values.last().and_then(|r| r.last()).expect("a value")
    }
}

impl Layer {
    pub fn is_horizontal(&self) -> bool {
        self.dir == Dir::Horizontal
    }
    pub fn is_vertical(&self) -> bool {
        self.dir == Dir::Vertical
    }
    pub fn is_routable(&self) -> bool {
        self.kind == LayerKind::Routing
    }
    /// No wire runs across the layer's direction: a rect-only layer is taken as unidirectional
    /// (a wrong-way rectangle would be legal on one, but the rare case is ignored). Multi-patterned
    /// layers are too, and are refused before this is read.
    pub fn is_unidirectional(&self) -> bool {
        self.rect_only
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ViaDef {
    pub name: String,
    pub is_default: bool,
    /// Layer numbers: below, cut, above.
    pub layer1: usize,
    pub cut: usize,
    pub layer2: usize,
    pub layer1_figs: Vec<Rect>,
    pub cut_figs: Vec<Rect>,
    pub layer2_figs: Vec<Rect>,
}

impl ViaDef {
    /// The bounding box of the shapes on the layer below, at the origin.
    pub fn layer1_bbox(&self) -> Rect {
        bbox(&self.layer1_figs)
    }
    pub fn layer2_bbox(&self) -> Rect {
        bbox(&self.layer2_figs)
    }
}

fn bbox(figs: &[Rect]) -> Rect {
    let mut b = figs[0];
    for f in &figs[1..] {
        b = Rect { xl: b.xl.min(f.xl), yl: b.yl.min(f.yl), xh: b.xh.max(f.xh), yh: b.yh.max(f.yh) };
    }
    b
}

/// A track pattern: `num` tracks from `start` every `spacing`, on `layer`; `vertical_tracks` when
/// the tracks are vertical lines (the pattern steps in x).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TrackPattern {
    pub layer: usize,
    pub vertical_tracks: bool,
    pub start: i32,
    pub num: i32,
    pub spacing: i32,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Tech {
    pub layers: Vec<Layer>,
    pub manufacturing_grid: i32,
    /// Technology vias, in technology order.
    pub via_defs: Vec<ViaDef>,
}

impl Tech {
    /// `ALLOW_PIN_AS_FEEDTHROUGH`: io clears it once any routing layer carries the
    /// right-way-on-grid-only constraint. Read by guide processing and the maze.
    pub fn allow_pin_feedthrough(&self) -> bool {
        !self.layers.iter().any(|l| l.right_way_on_grid_only)
    }
    pub fn top_layer_num(&self) -> usize {
        self.layers.len() - 1
    }
    pub fn bottom_layer_num(&self) -> usize {
        0
    }
    /// The topmost routing layer (the default top routing layer).
    pub fn top_routing_layer(&self) -> usize {
        (0..self.layers.len()).rev().find(|&l| self.layers[l].kind == LayerKind::Routing).expect("a routing layer")
    }
    pub fn layer_num(&self, name: &str) -> Option<usize> {
        self.layers.iter().position(|l| l.name == name)
    }
}

/// A master's pin: its shapes by layer number (master coordinates).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MasterPin {
    pub shapes: Vec<(usize, Rect)>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MasterTerm {
    pub name: String,
    /// The signal type (`SIGNAL`, `POWER`, `GROUND`, …).
    pub sig: String,
    pub pins: Vec<MasterPin>,
}

/// A master as the router imports it: its terminals, and its obstructions as blockages.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Master {
    pub terms: Vec<MasterTerm>,
    /// Blockage rectangles by layer number (master coordinates).
    pub blockages: Vec<(usize, Rect)>,
}

impl Master {
    /// The import rules for obstructions:
    /// - an obstruction on a CUT layer that touches the shapes of exactly ONE pin on the layer
    ///   above becomes a shape of that pin (a contact drawn as an obstruction is the pin's own);
    /// - every other obstruction merges with the rest on its layer, and the merged shapes'
    ///   MAXIMAL rectangles are the blockages.
    pub fn import(tech: &Tech, mut terms: Vec<MasterTerm>, obstructions: &[(usize, Rect)]) -> Master {
        let touches = |a: &Rect, b: &Rect| a.xl <= b.xh && b.xl <= a.xh && a.yl <= b.yh && b.yl <= a.yh;
        let mut merged: Vec<Polygon90Set> = vec![Polygon90Set::new(); tech.layers.len()];
        for &(layer, r) in obstructions {
            if tech.layers[layer].kind == LayerKind::Cut {
                let mut owner: Option<(usize, usize)> = None;
                let mut many = false;
                for (t, term) in terms.iter().enumerate() {
                    for (p, pin) in term.pins.iter().enumerate() {
                        if pin.shapes.iter().any(|(l, s)| *l == layer + 1 && touches(s, &r)) {
                            match owner {
                                None => owner = Some((t, p)),
                                Some(o) if o != (t, p) => many = true,
                                _ => {}
                            }
                        }
                    }
                }
                if let (Some((t, p)), false) = (owner, many) {
                    terms[t].pins[p].shapes.push((layer, r));
                    continue;
                }
            }
            merged[layer].insert_rect(r);
        }
        let mut blockages = Vec::new();
        // Per layer, per connected polygon (the reference's `get` order), that polygon's maximal
        // rectangles — NOT the layer's as one set: the order differs when a layer has two pieces.
        for (layer, set) in merged.iter_mut().enumerate() {
            for mut poly in set.polygons() {
                for r in poly.max_rectangles() {
                    blockages.push((layer, r));
                }
            }
        }
        Master { terms, blockages }
    }
}

/// A placed instance's transform: an orientation, then the origin.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Transform {
    pub orient: String,
    pub origin: (i32, i32),
}

impl Transform {
    /// The orientations as the database applies them to a point: `R90` → `(-y, x)`, `R180` →
    /// `(-x, -y)`, `R270` → `(y, -x)`, `MY` → `(-x, y)`, `MX` → `(x, -y)`, `MYR90` → `(-y, -x)`,
    /// `MXR90` → `(y, x)`; then the origin is added.
    pub fn apply(&self, r: Rect) -> Rect {
        let f = |(x, y): (i32, i32)| -> (i32, i32) {
            let (x, y) = match self.orient.as_str() {
                "R90" => (-y, x),
                "R180" => (-x, -y),
                "R270" => (y, -x),
                "MY" => (-x, y),
                "MYR90" => (-y, -x),
                "MX" => (x, -y),
                "MXR90" => (y, x),
                _ => (x, y),
            };
            (x + self.origin.0, y + self.origin.1)
        };
        let (a, b) = (f((r.xl, r.yl)), f((r.xh, r.yh)));
        Rect::new(a.0, a.1, b.0, b.1)
    }
}

#[cfg(feature = "odb")]
pub mod read {
    //! The technology, masters and tracks from the design database.
    use super::*;
    use vyges_opendb::Db;

    type Res<T> = Result<T, Box<dyn std::error::Error>>;

    /// A cut layer's LEF58 cut classes, as `io::Parser` reads them.
    pub fn cut_classes(db: &Db, layer: &str) -> Vec<CutClass> {
        db.layer_get_tech_layer_cut_class_rules(layer)
            .into_iter()
            .enumerate()
            .map(|(k, name)| {
                let width = db.cutclassrule_get_width(layer, k);
                let length = if db.cutclassrule_is_length_valid(layer, k) { db.cutclassrule_get_length(layer, k) } else { width };
                CutClass { name, width, length }
            })
            .collect()
    }

    /// A cut layer's LEF58 cut spacing table rules as `io::Parser` keeps them (it skips a SAMEMASK
    /// rule, and a LAYER rule on the first layer), or the first kind outside the modelled one — a
    /// different-net table on the layer itself (no LAYER, SAMENET or SAMEMETAL). Several such
    /// rules: the LAST is the one set, as io sets it.
    pub fn cut_spacing_table(db: &Db, layer: &str, classes: &[CutClass]) -> Result<Option<CutSpacingTable>, &'static str> {
        let mut out = None;
        for k in 0..db.num_layer_get_tech_layer_cut_spacing_table_def_rules(layer) {
            if db.cutspacingtablerule_is_same_mask(layer, k) {
                continue;
            }
            if db.cutspacingtablerule_is_layer_valid(layer, k) {
                return Err("a LAYER (inter-layer) cut spacing table");
            }
            if db.cutspacingtablerule_is_same_net(layer, k) || db.cutspacingtablerule_is_same_metal(layer, k) {
                return Err("a SAMENET or SAMEMETAL cut spacing table");
            }
            let names: Vec<&str> = std::iter::once("").chain(classes.iter().map(|c| c.name.as_str())).collect();
            let n = names.len();
            let mut t = CutSpacingTable { n, no_prl: db.cutspacingtablerule_is_no_prl(layer, k), horizontal: db.cutspacingtablerule_is_horizontal(layer, k), vertical: db.cutspacingtablerule_is_vertical(layer, k), ..Default::default() };
            for &c1 in &names {
                for s1 in [false, true] {
                    for &c2 in &names {
                        for s2 in [false, true] {
                            t.spacing.push((db.cutspacingtablerule_get_spacing(layer, k, c1, s1, c2, s2, "FIRST"), db.cutspacingtablerule_get_spacing(layer, k, c1, s1, c2, s2, "SECOND")));
                        }
                    }
                }
                t.max_spacing.push([db.cutspacingtablerule_get_max_spacing_cut_class_side(layer, k, c1, false), db.cutspacingtablerule_get_max_spacing_cut_class_side(layer, k, c1, true)]);
                t.exact_aligned.push(db.cutspacingtablerule_get_exact_aligned_spacing(layer, k, c1));
                for &c2 in &names {
                    t.prl_entry.push(db.cutspacingtablerule_get_prl_entry(layer, k, c1, c2));
                    t.center_to_center.push(db.cutspacingtablerule_is_center_to_center(layer, k, c1, c2));
                    t.center_and_edge.push(db.cutspacingtablerule_is_center_and_edge(layer, k, c1, c2));
                    t.prl_aligned.push(db.cutspacingtablerule_is_prl_for_aligned_cut_classes(layer, k, c1, c2));
                }
            }
            // The raw table: row count, then per row its length and its (first, second) pairs.
            let raw = db.cutspacingtablerule_get_spacing_table_table(layer, k);
            if raw.len() < 4 || raw[0] < 1 || raw[1] < 1 {
                return Err("an empty cut spacing table");
            }
            t.default_spacing = (raw[2], raw[3]);
            let first = |names: Vec<String>| names.into_iter().next().map(|n| n.split('/').next().unwrap_or("").to_string()).unwrap_or_default();
            let c1 = first(db.cutspacingtablerule_get_spacing_table_col_map_names(layer, k));
            let c2 = first(db.cutspacingtablerule_get_spacing_table_row_map_names(layer, k));
            t.default_center_to_center = db.cutspacingtablerule_is_center_to_center(layer, k, &c1, &c2);
            t.default_center_and_edge = db.cutspacingtablerule_is_center_and_edge(layer, k, &c1, &c2);
            out = Some(t);
        }
        Ok(out)
    }

    /// LEF58 corner spacing rule `k` of `layer`, as `io::Parser` translates it, or the first clause
    /// outside the modelled subset (CONVEXCORNER with its width table, CORNERONLY, CORNERTOCORNER;
    /// not CONCAVECORNER, SAMEMASK or EXCEPTEOL) — or an empty table, which the reference would
    /// index out of bounds.
    pub fn corner_spacing_rule(db: &Db, layer: &str, k: usize) -> Result<CornerSpacing, &'static str> {
        if db.cornerspacingrule_get_type(layer, k) != "CONVEXCORNER" {
            return Err("CONCAVECORNER");
        }
        if db.cornerspacingrule_is_same_mask(layer, k) {
            return Err("SAMEMASK");
        }
        if db.cornerspacingrule_is_except_eol(layer, k) {
            return Err("EXCEPTEOL");
        }
        let widths = db.cornerspacingrule_get_width_table(layer, k);
        let flat = db.cornerspacingrule_get_spacing_table(layer, k);
        let spacings: Vec<(i32, i32)> = flat.chunks(2).filter(|c| c.len() == 2).map(|c| (c[0], c[1])).collect();
        if widths.is_empty() || widths.len() != spacings.len() {
            return Err("an empty or ragged width table");
        }
        // `isCornerToCorner` is read only when CORNERONLY is not set (`else if`).
        let corner_to_corner = !db.cornerspacingrule_is_corner_only(layer, k) && db.cornerspacingrule_is_corner_to_corner(layer, k);
        Ok(CornerSpacing { same_xy: spacings.iter().all(|&(a, b)| a == b), widths, spacings, corner_to_corner })
    }

    /// LEF58 end-of-line spacing rule `k` of `layer`, as `io::Parser` translates it
    /// (`frLef58SpacingEndOfLineConstraint`): `Ok(None)` for a rule the reference DROPS with a warning
    /// (EXCEPTEXACTWIDTH, FILLCONCAVECORNER, EQUALRECTWIDTH — tested first, as it does), `Err(clause)`
    /// for a clause outside the modelled subset (SPACING / ENDOFLINE / WITHIN, ENDTOEND without its
    /// extension or cut spaces, PARALLELEDGE with only its space, within and TWOEDGES).
    pub fn lef58_eol_rule(db: &Db, layer: &str, k: usize) -> Result<Option<EolRule>, &'static str> {
        if db.spacingeolrule_is_except_exact_width_valid(layer, k) || db.spacingeolrule_is_fill_concave_corner_valid(layer, k) || db.spacingeolrule_is_equal_rect_width_valid(layer, k) {
            return Ok(None);
        }
        let unmodelled: [(bool, &'static str); 22] = [
            (db.spacingeolrule_is_exact_width_valid(layer, k), "EXACTWIDTH"),
            (db.spacingeolrule_is_wrong_dir_spacing_valid(layer, k), "WRONGDIRSPACING"),
            (db.spacingeolrule_is_opposite_width_valid(layer, k), "OPPOSITEWIDTH"),
            (db.spacingeolrule_is_end_prl_spacing_valid(layer, k), "ENDPRLSPACING"),
            (db.spacingeolrule_is_wrong_dir_within_valid(layer, k), "WRONGDIRWITHIN"),
            (db.spacingeolrule_is_same_mask_valid(layer, k), "SAMEMASK"),
            (db.spacingeolrule_is_extension_valid(layer, k), "ENDTOEND EXTENSION"),
            (db.spacingeolrule_is_other_end_width_valid(layer, k), "ENDTOEND OTHERENDWIDTH"),
            (db.spacingeolrule_is_cut_spaces_valid(layer, k), "ENDTOEND cut spaces"),
            (db.spacingeolrule_is_subtract_eol_width_valid(layer, k), "PARALLELEDGE SUBTRACTEOLWIDTH"),
            (db.spacingeolrule_is_par_prl_valid(layer, k), "PARALLELEDGE PRL"),
            (db.spacingeolrule_is_par_min_length_valid(layer, k), "PARALLELEDGE MINLENGTH"),
            (db.spacingeolrule_is_same_metal_valid(layer, k), "PARALLELEDGE SAMEMETAL"),
            (db.spacingeolrule_is_non_eol_corner_only_valid(layer, k), "PARALLELEDGE NONEOLCORNERONLY"),
            (db.spacingeolrule_is_parallel_same_mask_valid(layer, k), "PARALLELEDGE PARALLELSAMEMASK"),
            (db.spacingeolrule_is_min_length_valid(layer, k) || db.spacingeolrule_is_max_length_valid(layer, k), "MINLENGTH/MAXLENGTH"),
            (db.spacingeolrule_is_enclose_cut_valid(layer, k), "ENCLOSECUT"),
            (db.spacingeolrule_is_to_concave_corner_valid(layer, k), "TOCONCAVECORNER"),
            (db.spacingeolrule_is_to_notch_length_valid(layer, k), "TONOTCHLENGTH"),
            (db.spacingeolrule_is_min_adjacent_length_valid(layer, k), "MINADJACENTLENGTH"),
            (db.spacingeolrule_is_cut_class_valid(layer, k) || db.spacingeolrule_is_withcut_valid(layer, k), "CUTCLASS/WITHCUT"),
            (db.spacingeolrule_is_enclosure_end_valid(layer, k), "ENCLOSUREEND"),
        ];
        if let Some((_, clause)) = unmodelled.iter().find(|(set, _)| *set) {
            return Err(clause);
        }
        let parallel = db.spacingeolrule_is_parallel_edge_valid(layer, k).then(|| ParallelEdge {
            space: db.spacingeolrule_get_par_space(layer, k),
            within: db.spacingeolrule_get_par_within(layer, k),
            two_edges: db.spacingeolrule_is_two_edges_valid(layer, k),
        });
        Ok(Some(EolRule {
            space: db.spacingeolrule_get_eol_space(layer, k),
            width: db.spacingeolrule_get_eol_width(layer, k),
            within: db.spacingeolrule_get_eol_within(layer, k),
            parallel,
            end_to_end: db.spacingeolrule_is_end_to_end_valid(layer, k).then(|| db.spacingeolrule_get_end_to_end_space(layer, k)),
        }))
    }

    pub fn tech(db: &Db) -> Res<Tech> {
        let mut layers: Vec<Layer> = Vec::new();
        let placeholder = |name: &str| Layer { name: name.into(), ..Layer::default() };
        // The last masterslice layer read so far that is not a well or diffusion layer.
        let mut masterslice: Option<String> = None;
        for (name, dir) in db.layers_with_direction()? {
            let kind = db.layer_get_type(&name)?;
            match kind.as_str() {
                "MASTERSLICE" => {
                    if !matches!(db.layer_lef58_type(&name).as_str(), "NWELL" | "PWELL" | "DIFFUSION") {
                        masterslice = Some(name);
                    }
                }
                "ROUTING" if db.layer_get_routing_level(&name) > 0 => {
                    if layers.is_empty() {
                        layers.push(placeholder(masterslice.as_deref().unwrap_or("FR_MASTERSLICE")));
                        layers.push(placeholder("Fr_VIA"));
                    }
                    let width = db.layer_get_width(&name) as i32;
                    let min_width = (db.layer_get_min_width(&name) as i32).min(width);
                    let dir = match dir.as_str() {
                        "HORIZONTAL" => Dir::Horizontal,
                        "VERTICAL" => Dir::Vertical,
                        _ => Dir::None,
                    };
                    let pitch = db.layer_get_pitch(&name);
                    let wrong_way_width = db.layer_get_wrong_way_width(&name) as i32;
                    let v55 = db.layer_v55_spacing_table(&name)?;
                    let spacing = match (v55.widths_and_lengths, v55.table) {
                        (Some((w, l)), Some(t)) => Some(SpacingTable {
                            widths: w.iter().map(|&v| v as i32).collect(),
                            prls: l.iter().map(|&v| v as i32).collect(),
                            values: t.iter().map(|r| r.iter().map(|&v| v as i32).collect()).collect(),
                        }),
                        _ => match db.layer_get_spacing(&name) {
                            s if s > 0 => Some(SpacingTable { widths: vec![0], prls: vec![0], values: vec![vec![s]] }),
                            _ => None,
                        },
                    };
                    let eol = db
                        .layer_v54_eol_rules(&name)?
                        .into_iter()
                        .map(|(space, width, within, par)| EolRule { space: space as i32, width, within, parallel: par.map(|(space, within, two_edges)| ParallelEdge { space, within, two_edges }), end_to_end: None })
                        .collect();
                    let corner_spacing: Vec<CornerSpacing> = (0..db.num_layer_get_tech_layer_corner_spacing_rules(&name)).filter_map(|k| corner_spacing_rule(db, &name, k).ok()).collect();
                    let lef58_eol: Vec<EolRule> = (0..db.num_layer_get_tech_layer_spacing_eol_rules(&name)).filter_map(|k| lef58_eol_rule(db, &name, k).ok().flatten()).collect();
                    let eol_keepout: Vec<EolKeepOut> = (0..db.num_layer_get_tech_layer_eol_keep_out_rules(&name))
                        .map(|k| EolKeepOut {
                            width: db.eolkeepoutrule_get_eol_width(&name, k),
                            backward: db.eolkeepoutrule_get_backward_ext(&name, k),
                            forward: db.eolkeepoutrule_get_forward_ext(&name, k),
                            side: db.eolkeepoutrule_get_side_ext(&name, k),
                            corner_only: db.eolkeepoutrule_is_corner_only(&name, k),
                            except_within: db.eolkeepoutrule_is_except_within(&name, k),
                            within_low: db.eolkeepoutrule_get_within_low(&name, k),
                            within_high: db.eolkeepoutrule_get_within_high(&name, k),
                        })
                        .collect();
                    let min_area = db.layer_get_area(&name).unwrap_or(0);
                    // ⚠️ `frCoord minEnclosedArea = _minEnclosedArea`: an int64 narrowed to int.
                    let min_enclosed_areas: Vec<i32> = db.layer_min_enclosed_areas(&name).into_iter().map(|a| a as i32).collect();
                    // Only the plain flag makes the constraint; "except non-core pins" alone is
                    // stored and never read.
                    let rect_only = db.layer_is_rect_only(&name);
                    let right_way_on_grid_only = db.layer_is_right_way_on_grid_only(&name) || db.layer_get_num_masks(&name) > 1;
                    layers.push(Layer { name, kind: LayerKind::Routing, dir, width, min_width, pitch, wrong_way_width, spacing, cut_spacing: None, cut_classes: vec![], cut_table: None, eol, lef58_eol, eol_keepout, corner_spacing, min_area, min_enclosed_areas, rect_only, right_way_on_grid_only });
                }
                "CUT" if !layers.is_empty() => {
                    let width = db.layer_get_width(&name) as i32;
                    let cut_spacing = Some(db.layer_get_spacing(&name)).filter(|&s| s > 0);
                    let cut_classes = cut_classes(db, &name);
                    let cut_table = cut_spacing_table(db, &name, &cut_classes).ok().flatten();
                    layers.push(Layer { name, kind: LayerKind::Cut, width, cut_spacing, cut_classes, cut_table, ..Layer::default() });
                }
                _ => {}
            }
        }
        let num_of = |n: i64| -> Option<usize> {
            let name = db.layer_name_by_number(n);
            layers.iter().position(|l| l.name == name)
        };
        let mut via_defs = Vec::new();
        for via in db.tech_get_vias() {
            let boxes = db.tech_via_boxes(&via)?;
            let mut nums: Vec<usize> = boxes.iter().filter_map(|b| num_of(b.0)).collect();
            nums.sort();
            nums.dedup();
            let [l1, cut, l2] = nums[..] else { continue };
            let figs = |layer: usize| -> Vec<Rect> { boxes.iter().filter(|b| num_of(b.0) == Some(layer)).map(|b| Rect::new(b.1, b.2, b.3, b.4)).collect() };
            via_defs.push(ViaDef { name: via.clone(), is_default: db.techvia_is_default(&via), layer1: l1, cut, layer2: l2, layer1_figs: figs(l1), cut_figs: figs(cut), layer2_figs: figs(l2) });
        }
        Ok(Tech { layers, manufacturing_grid: db.tech_get_manufacturing_grid(), via_defs })
    }

    /// Every track pattern of the block, per layer: x patterns (vertical tracks), then y.
    pub fn tracks(db: &Db, tech: &Tech) -> Res<Vec<TrackPattern>> {
        let mut out = Vec::new();
        for (layer, l) in tech.layers.iter().enumerate() {
            if l.kind != LayerKind::Routing {
                continue;
            }
            let Ok((x, y)) = db.track_patterns(&l.name) else { continue };
            for (start, num, spacing) in x {
                out.push(TrackPattern { layer, vertical_tracks: true, start, num, spacing });
            }
            for (start, num, spacing) in y {
                out.push(TrackPattern { layer, vertical_tracks: false, start, num, spacing });
            }
        }
        Ok(out)
    }

    /// A master's terminals and their pins, by layer number (routing and cut shapes only).
    pub fn master_terms(db: &Db, tech: &Tech, master: &str) -> Res<Vec<MasterTerm>> {
        let mut out = Vec::new();
        for (term, sig) in db.master_mterms(master)? {
            let mut pins = Vec::new();
            for p in 0..db.num_mpins(master, &term) {
                let shapes = db
                    .mpin_boxes(master, &term, p)?
                    .into_iter()
                    .filter_map(|(n, x0, y0, x1, y1)| tech.layer_num(&db.layer_name_by_number(n)).map(|l| (l, Rect::new(x0, y0, x1, y1))))
                    .collect();
                pins.push(MasterPin { shapes });
            }
            out.push(MasterTerm { name: term, sig, pins });
        }
        Ok(out)
    }

    /// A master's obstructions, by layer number (the technology's layers only).
    pub fn master_obstructions(db: &Db, tech: &Tech, master: &str) -> Res<Vec<(usize, Rect)>> {
        Ok(db
            .master_obstruction_boxes(master)?
            .into_iter()
            .filter_map(|(n, x0, y0, x1, y1)| tech.layer_num(&db.layer_name_by_number(n)).map(|l| (l, Rect::new(x0, y0, x1, y1))))
            .collect())
    }

    pub fn transform(db: &Db, inst: &str) -> Transform {
        Transform { orient: db.inst_get_orient(inst), origin: (db.inst_get_origin_x(inst), db.inst_get_origin_y(inst)) }
    }

    /// The instance's placement location: the lower-left of its placed box — NOT the origin its
    /// orientation is applied about (they differ for every flipped or rotated instance).
    pub fn location(db: &Db, inst: &str) -> (i32, i32) {
        (db.inst_get_location_x(inst), db.inst_get_location_y(inst))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn term(name: &str, pins: Vec<Vec<(usize, Rect)>>) -> MasterTerm {
        MasterTerm { name: name.into(), sig: "SIGNAL".into(), pins: pins.into_iter().map(|shapes| MasterPin { shapes }).collect() }
    }

    /// A cut obstruction touching ONE pin's shapes on the layer above becomes that pin's shape.
    #[test]
    fn a_cut_obstruction_on_one_pin_joins_it() {
        let t = crate::gc::tests::tech();
        let cut = Rect::new(0, 0, 170, 170);
        let m = Master::import(&t, vec![term("A", vec![vec![(4, Rect::new(-50, -50, 400, 220))]])], &[(3, cut)]);
        assert!(m.blockages.is_empty());
        assert!(m.terms[0].pins[0].shapes.contains(&(3, cut)));
    }

    /// Touching two pins it stays an obstruction.
    #[test]
    fn a_cut_obstruction_on_two_pins_stays_a_blockage() {
        let t = crate::gc::tests::tech();
        let cut = Rect::new(0, 0, 170, 170);
        let terms = vec![term("A", vec![vec![(4, Rect::new(-50, -50, 100, 220))]]), term("B", vec![vec![(4, Rect::new(100, -50, 400, 220))]])];
        let m = Master::import(&t, terms, &[(3, cut)]);
        assert_eq!(m.blockages, vec![(3, cut)]);
        assert!(m.terms.iter().all(|t| t.pins[0].shapes.len() == 1));
    }

    /// Obstructions on a layer merge; the blockages are the merge's MAXIMAL rectangles (an L gives
    /// both arms through the corner).
    #[test]
    fn obstructions_merge_into_maximal_rectangles() {
        let t = crate::gc::tests::tech();
        let m = Master::import(&t, Vec::new(), &[(4, Rect::new(0, 0, 300, 100)), (4, Rect::new(0, 0, 100, 300))]);
        let mut b = m.blockages;
        b.sort();
        assert_eq!(b, vec![(4, Rect::new(0, 0, 100, 300)), (4, Rect::new(0, 0, 300, 100))]);
    }

    /// Rule: the merged obstructions are taken apart into connected polygons first (by top y,
    /// then the leftmost x on that top edge) and each polygon's maximal rectangles follow in
    /// turn — so the lower-topped piece on the right comes BEFORE the taller one on the left (the whole set's
    /// maximal rectangles would put it last). The
    /// blockage order is the region query's input order, which decides owner order in the checks.
    #[test]
    fn obstruction_blockages_follow_polygon_order() {
        let t = crate::gc::tests::tech();
        // Left: an L-shaped piece topped at 900; right: a bar topped at 600, which comes first.
        let (a, b, bar) = (Rect::new(0, 300, 200, 600), Rect::new(200, 400, 500, 900), Rect::new(600, 400, 900, 600));
        let m = Master::import(&t, Vec::new(), &[(4, a), (4, b), (4, bar)]);
        let want = vec![(4, bar), (4, a), (4, b), (4, Rect::new(0, 400, 500, 600))];
        assert_eq!(m.blockages, want);
    }
}
