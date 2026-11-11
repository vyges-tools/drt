// SPDX-License-Identifier: Apache-2.0
//! The technology and design as the router reads them.
//!
//! Rules:
//! - layers are numbered from a placeholder masterslice (0), then routing and cut layers in
//!   technology order — a placeholder cut layer (1) below the first routing layer when no cut layer
//!   came first (a cut layer read first, like asap7's V0, is layer 1 itself);
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
    /// A routing layer's minimum spacing (`frLayer::getMinSpacing`).
    pub spacing: Option<MinSpacing>,
    /// A cut layer's minimum spacing, edge to edge.
    pub cut_spacing: Option<i32>,
    /// A cut layer's LEF58 cut classes, in the technology's order.
    pub cut_classes: Vec<CutClass>,
    /// A cut layer's LEF58 different-net cut spacing table (`getLef58DiffNetCutSpcTblConstraint`).
    pub cut_table: Option<CutSpacingTable>,
    /// A cut layer's LEF58 SAMENET cut spacing table on the layer itself
    /// (`getLef58SameNetCutSpcTblConstraint`): checked between cuts of one net, which the
    /// different-net table then leaves to it; the via-to-via prep reads it before the
    /// different-net one.
    pub cut_table_same_net: Option<CutSpacingTable>,
    /// A cut layer's LEF58 inter-layer cut spacing table, neither SAMENET nor SAMEMETAL
    /// (`getLef58DefaultInterCutSpcTblConstraint`): checked between a cut of this layer and a cut
    /// of the table's second layer, of two nets; the maze costs vias by its default.
    pub cut_table_inter: Option<InterCutTable>,
    /// A cut layer's LEF58 enclosure rules as io keeps them (`frLef58EnclosureConstraint`), in the
    /// technology's order.
    pub cut_enclosures: Vec<CutEnclosure>,
    /// A cut layer's LEF58 cut spacing rules (`frLef58CutSpacingConstraint`), the different-net
    /// list io builds, in the technology's order: the rules to a corner of the metal on another
    /// layer (a second layer, a cut class and CONCAVECORNER or NONEOLCONVEXCORNER) and the
    /// ADJACENTCUTS rules io keeps (the only forms modelled; the census refuses every other LEF58
    /// cut spacing rule, and the ADJACENTCUTS forms io skips are skipped here too).
    pub lef58_cut_spacing: Vec<Lef58CutSpacing>,
    /// A routing layer's end-of-line spacing rules, in the technology's order.
    pub eol: Vec<EolRule>,
    /// A routing layer's LEF58 end-of-line SPACING rules, in the technology's order (the
    /// modelled subset — `lef58_eol_rule`; the census refuses the rest, and the TOCONCAVECORNER /
    /// TONOTCHLENGTH rules are skipped with a warning, never held here).
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
    /// OR a multi-patterned layer (`numMasks > 1`). Its consumers: pin
    /// access (on-grid points only), the maze grid (right-way edges on tracks only; an off-grid via
    /// only at an access point) and, through `Tech::allow_pin_feedthrough`, guides and the maze.
    pub right_way_on_grid_only: bool,
    /// A routing layer's mask count (`frLayer::getNumMasks`, the LEF `MASK`; 1 without one, 0 on
    /// a layer that is not read as routing). More than one makes the layer unidirectional and
    /// right-way-on-grid-only; the router assigns no colour.
    pub num_masks: u32,
    /// A routing layer's minimum-cut rules as io keeps them (`frMinimumcutConstraint`): the
    /// v5.4 MINIMUMCUT rules, then the LEF58 ones io converts
    /// ([`read::convert_lef58_min_cut_constraints`]).
    pub min_cuts: Vec<MinCut>,
    /// The largest LENGTH … WITHIN distance of the layer's converted LEF58 minimum-cut rules (0
    /// without one): io raises the router's multi-thread safe distance (MTSAFEDIST) to it. The
    /// v5.4 rules' LENGTH does not raise it.
    pub mt_safe_dist: i32,
    /// A routing layer's v5.8 MINSTEP rule as io keeps it (`frMinStepConstraint`).
    pub min_step: Option<MinStep>,
    /// A routing layer's LEF58_MINSTEP rules io keeps (`frLef58MinStepConstraint`), in the
    /// technology's order: every rule but one with MAXEDGES above 1 (skipped, DRT-335).
    pub lef58_min_step: Vec<Lef58MinStep>,
    /// A routing layer's v5.4 SPACING … RANGE rules, in the technology's order
    /// (`frSpacingRangeConstraint`; io keeps them out of the spacing table).
    pub spacing_ranges: Vec<SpacingRange>,
    /// A routing layer's LEF58_AREA rules as io keeps them, in io's two lists.
    pub lef58_area: Lef58Area,
    /// A cut layer's SPACINGTABLE ORTHOGONAL rows, (WITHIN, SPACING) in the technology's order
    /// (`frOrthSpacingTableConstraint`, io keeps the database's table as it is).
    pub orth_spacing: Vec<(i32, i32)>,
    /// A cut layer's LEF58_MAXSPACING rules io keeps (`frLef58MaxSpacingConstraint`), in the
    /// technology's order: the ones with a CUTCLASS naming a class of this layer.
    pub lef58_max_spacing: Vec<MaxSpacingRule>,
    /// A routing layer's LEF58_WIDTHTABLE … ORTHOGONAL rules as io folds them into one constraint
    /// (`frLef58WidthTableOrthConstraint`), see [`read::width_tbl_orth`].
    pub width_tbl_orth: Option<WidthTblOrth>,
}

/// The one width-table-orthogonal constraint io makes per routing layer: two concave corners of
/// one polygon closer than `horz_spc` in x AND `vert_spc` in y break it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WidthTblOrth {
    pub horz_spc: i32,
    pub vert_spc: i32,
}

/// A LEF58_MAXSPACING rule as io keeps it: `MAXSPACING max CUTCLASS c` — every via of class `c`
/// on the layer needs another cut within `max` of it (the router's repair pass, not the check).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MaxSpacingRule {
    pub max: i32,
    /// The class's place among the layer's cut classes (`getCutClassIdx`, from 0).
    pub cut_class: usize,
}

/// A LEF58_AREA rule io keeps on a routing layer (`frLef58AreaConstraint`): the forms it skips
/// with a warning (EXCEPTMINWIDTH, EXCEPTEDGELENGTH, EXCEPTMINSIZE, EXCEPTSTEP, MASK, LAYER) are
/// never held, see [`read::lef58_area_skipped_clause`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AreaRule {
    /// The rule's place among the layer's LEF58_AREA rules in the technology's order: the
    /// marker's constraint (the reference keeps a marker once per constraint).
    pub index: u16,
    /// AREA, in square database units.
    pub area: i64,
    /// RECTWIDTH (0 without one).
    pub rect_width: i32,
    /// EXCEPTRECTANGLE: a polygon that is one rectangle is judged by the next rule instead.
    pub except_rectangle: bool,
}

/// io's two LEF58_AREA lists for a layer (`frLayer`): the rules with a RECTWIDTH, and the rest.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Lef58Area {
    /// Ascending by (RECTWIDTH, AREA) (`addLef58AreaConstraintRectWidth`).
    pub rect_width: Vec<AreaRule>,
    /// Descending by AREA (`addLef58AreaConstraint`).
    pub area: Vec<AreaRule>,
}

impl Lef58Area {
    pub fn is_empty(&self) -> bool {
        self.rect_width.is_empty() && self.area.is_empty()
    }

    /// io's insertion, a rule with a RECTWIDTH into the first list, any other into the second.
    /// Upstream rule (`frLayer::addLef58AreaConstraintRectWidth` / `addLef58AreaConstraint`):
    /// each inserts at `std::ranges::lower_bound` of its key, so a rule whose key equals earlier
    /// ones goes BEFORE them (ties end up in reverse technology order), not after.
    pub fn add(&mut self, r: AreaRule) {
        if r.rect_width > 0 {
            let key = (r.rect_width, r.area);
            let at = self.rect_width.partition_point(|a| (a.rect_width, a.area) < key);
            self.rect_width.insert(at, r);
        } else {
            let at = self.area.partition_point(|a| a.area > r.area);
            self.area.insert(at, r);
        }
    }

    /// `checkMetalShape_lef58Area`, which rule a polygon breaks, if any: `is_rect` (its outer
    /// boundary has four vertices), its maximal rectangles, its area. Upstream rule: a
    /// rectangle first tries the RECTWIDTH rules in order; the first whose RECTWIDTH is at least
    /// the polygon's narrower side (the polygon being ONE maximal rectangle) decides it alone,
    /// broken below that rule's AREA, and no other rule is tried. Otherwise the other rules, by
    /// descending AREA: none is broken once the area reaches a rule's; the first rule not
    /// EXCEPTRECTANGLE, or EXCEPTRECTANGLE with the polygon NOT one maximal rectangle, is broken.
    pub fn violated(&self, is_rect: bool, max_rects: &[crate::polygon90::Rect], area: i64) -> Option<AreaRule> {
        if is_rect {
            for r in &self.rect_width {
                // `checkMetalShape_lef58Area_rectWidth`
                let applies = max_rects.len() == 1 && (max_rects[0].xh - max_rects[0].xl).min(max_rects[0].yh - max_rects[0].yl) <= r.rect_width;
                if applies {
                    return (area < r.area).then_some(*r);
                }
            }
        }
        for r in &self.area {
            if area >= r.area {
                break;
            }
            // `checkMetalShape_lef58Area_exceptRectangle`
            if !r.except_rectangle || max_rects.len() != 1 {
                return Some(*r);
            }
        }
        None
    }
}

/// `frSpacingRangeConstraint`: SPACING for a rectangle whose width is within [min, max].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpacingRange {
    pub min_spacing: i32,
    pub min_width: i32,
    pub max_width: i32,
}

impl SpacingRange {
    pub fn in_range(&self, w: i32) -> bool {
        w >= self.min_width && w <= self.max_width
    }
}

/// `frMinStepConstraint`: a run of edges each shorter than MINSTEP between two longer ones is a
/// violation — unless its type needs a corner the check never records (INSIDECORNER,
/// OUTSIDECORNER), the run is within MAXLENGTH, or (MAXEDGES, type then unknown) within MAXEDGES.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MinStep {
    pub min_step_length: i32,
    /// The rule's type; `None` (the source's UNKNOWN) with MAXEDGES or none given.
    pub kind: Option<MinStepKind>,
    /// MAXLENGTH, -1 without one.
    pub max_length: i32,
    /// MAXEDGES, -1 without one.
    pub max_edges: i32,
}

/// `frLef58MinStepConstraint` as io keeps a LEF58_MINSTEP rule. Only two forms are checked
/// (`checkMetalShape_lef58MinStep`): NOBETWEENEOL (`eol_width`) and MINADJACENTLENGTH
/// (`min_adjacent_length`, with NOADJACENTEOL's width and EXCEPTRECTANGLE); io keeps the rest of
/// the rule (MAXEDGES, the second length, the corner forms, MINBETWEENLENGTH) unread.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Lef58MinStep {
    pub min_step_length: i32,
    /// MAXEDGES, -1 without one (kept; the check never reads it).
    pub max_edges: i32,
    /// MINADJACENTLENGTH's first value, -1 without one.
    pub min_adjacent_length: i32,
    /// NOADJACENTEOL's width, -1 without one.
    pub no_adj_eol: i32,
    /// NOBETWEENEOL's width, -1 without one.
    pub eol_width: i32,
    pub except_rectangle: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MinStepKind {
    InsideCorner,
    OutsideCorner,
    Step,
}

/// `frMinimumcutConstraint`: NUMCUTS cuts needed where the metal is wider than WIDTH; WITHIN,
/// the connection side and LENGTH … WITHIN are optional (`None` for the source's -1 / UNKNOWN).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MinCut {
    pub num_cuts: i32,
    pub width: i32,
    pub within: Option<i32>,
    /// `Some(true)` FROMABOVE, `Some(false)` FROMBELOW.
    pub from_above: Option<bool>,
    /// LENGTH and its WITHIN distance.
    pub length: Option<(i32, i32)>,
}

impl MinCut {
    /// Whether a rule asking for `num_cuts` cuts is imported: only one asking for two or more.
    /// Every via has at least one cut, so a one-cut rule can only add markers (upstream
    /// 263dba46d0).
    pub fn applies(num_cuts: i32) -> bool {
        num_cuts > 1
    }
}

/// An end-of-line spacing rule: a line end narrower than `width` needs `space` to a facing edge
/// within `within` beyond its sides; with a parallel edge, only when a parallel edge lies within
/// `par_space` of a side (on both sides with `two_edges`), up to `par_within` behind the end.
/// `end_to_end` (LEF58 only): the spacing when the facing edge is itself a line end.
/// `length` (LEF58 only): MINLENGTH / MAXLENGTH on the edges beside the line end.
/// `extension` (LEF58 only, under ENDTOEND): ENDTOEND's EXTENSION, read by the check alone
/// (`frLef58SpacingEndOfLineWithinEndToEndConstraint::getExtension`, see the check's
/// `eol_query_rect` and `eol_end_to_end`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EolRule {
    pub space: i32,
    pub width: i32,
    pub within: i32,
    pub parallel: Option<ParallelEdge>,
    pub end_to_end: Option<i32>,
    pub length: Option<EolLength>,
    pub extension: Option<i32>,
    /// OPPOSITEWIDTH: the rule applies only when the wire beyond the line end spans less than
    /// this perpendicular to the line end (`Worker::eol_opposite_wire_span`). ⛔ A DIVERGENCE by
    /// decision (drt divergence register D13, upstream OpenROAD issue #12226): the reference keeps
    /// the value and never reads it.
    pub opposite_width: Option<i32>,
}

/// A LEF58 end-of-line rule's `MINLENGTH l [TWOSIDES]` or `MAXLENGTH l [TWOSIDES]`
/// (`frLef58SpacingEndOfLineWithinMaxMinLengthConstraint`): the line end counts only when the
/// edges beside it are at least (`max` false) or at most (`max` true) `length` long, on BOTH sides
/// with `two_sides`, else on either.
///
/// ⛔ **A DIVERGENCE from the reference, by decision (drt divergence register D8).** The reference's
/// io passes PARALLELEDGE's TWOEDGES flag as TWOSIDES (`isTwoEdgesValid()`), so `MINLENGTH l
/// TWOSIDES` is judged one-sided unless the rule also says TWOEDGES (upstream OpenROAD issue
/// #12181). This router reads TWOSIDES as written, as the patched reference does
/// (vyges-openroad-patches upstream-fix 0001).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EolLength {
    pub max: bool,
    pub length: i32,
    pub two_sides: bool,
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
/// `frLef58EnclosureConstraint`: for a cut of `class` (this crate's index: 0 no class, else the
/// class's place + 1), a metal of at least `min_width` above and/or below (ABOVE / BELOW: only
/// there) must overhang it by `first` and `second` — at the ends and sides for ENDSIDE, either way
/// round otherwise. EOL-type rules are kept but never checked (the check reads the others only).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CutEnclosure {
    pub class: usize,
    pub above_only: bool,
    pub below_only: bool,
    pub eol: bool,
    pub endside: bool,
    pub first: i32,
    pub second: i32,
    pub min_width: i32,
}

impl CutEnclosure {
    /// `isValidOverhang`.
    pub fn valid(&self, end: i32, side: i32) -> bool {
        if self.endside {
            return end >= self.first && side >= self.second;
        }
        (end >= self.first && side >= self.second) || (end >= self.second && side >= self.first)
    }
}

/// A LEF58 cut spacing rule `SPACING s LAYER l CUTCLASS c CONCAVECORNER` or `… NONEOLCONVEXCORNER
/// w [MINLENGTH m]` as `io::Parser` keeps it (`frLef58CutSpacingConstraint`: cut spacing, second
/// layer, cut class, and the concave corner flag or the end-of-line width and minimum length): a cut
/// of `class` (this crate's index: the class's place + 1) inside a maximal rectangle of the metal on
/// `second_layer` needs `spacing` to each corner of that metal the `corner` form selects.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CutLayerSpacing {
    pub spacing: i32,
    pub second_layer: usize,
    pub class: usize,
    pub corner: CutCorner,
}

/// A LEF58 cut spacing rule `SPACING s [CENTERTOCENTER] ADJACENTCUTS n [TWOCUTS t] WITHIN w
/// [CUTCLASS c] [NOPRL]` as `io::Parser` keeps it (`frLef58CutSpacingConstraint` with adjacent
/// cuts): a cut with at least `adjacent_cuts` cuts of `class` (this crate's index, 0 for no class:
/// the reference's -1) closer than `within` — itself among them when it is of that class; with
/// NOPRL only those with no parallel run, itself always — or, with TWOCUTS, a pair of cuts each with
/// at least `two_cuts` such cuts, needs `spacing` (centre to centre with CENTERTOCENTER, else edge
/// to edge) to every other cut on its layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdjacentCuts {
    pub spacing: i32,
    pub center_to_center: bool,
    pub adjacent_cuts: i32,
    pub two_cuts: Option<i32>,
    pub within: i32,
    pub class: usize,
    pub no_prl: bool,
}

/// One rule of a cut layer's LEF58 cut spacing list, as `checkLef58CutSpacing_main` dispatches
/// it: a second layer first, then adjacent cuts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Lef58CutSpacing {
    Layer(CutLayerSpacing),
    AdjacentCuts(AdjacentCuts),
}

/// The corner a LEF58 cut spacing LAYER rule is measured to (`checkLef58CutSpacing_spc_layer`'s
/// two branches, CONCAVECORNER first).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CutCorner {
    /// A concave corner on the rectangle's boundary lines, Euclidean edge-to-point distance.
    Concave,
    /// A convex corner that does not end a line end narrower than `eol_width` (a line end is an
    /// edge between two convex corners; with `min_length`, only when both edges beside it are at
    /// least that long), inside the triangular keep-out whose legs are the spacing, each clipped to
    /// the corner's edge on its axis.
    NonEolConvex { eol_width: i32, min_length: Option<i32> },
}

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
    /// An inter-layer table's class names ("" first, no class), in the order its lookups are
    /// indexed; empty for a table on one layer (indexed by that layer's classes).
    pub names: Vec<String>,
}

/// A cut layer's LEF58 cut spacing tables as io keeps them ([`read::cut_spacing_tables`]).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct CutSpacingTables {
    pub diff: Option<CutSpacingTable>,
    pub same_net: Option<CutSpacingTable>,
    /// The default inter-layer table's rule index.
    pub inter: Option<usize>,
}

/// A LEF58 inter-layer cut spacing table (`SPACINGTABLE … LAYER l`, the default kind: neither
/// SAMENET nor SAMEMETAL), with its second layer.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct InterCutTable {
    /// The table's second layer (a cut layer; the kit's is the one below).
    pub second_layer: usize,
    /// NOSTACK: overlapping cuts of the two layers violate even when one contains the other.
    pub no_stack: bool,
    /// Its lookups, by class NAME (`names`): the two layers' classes in one list.
    pub table: CutSpacingTable,
}

impl CutSpacingTable {
    /// The table's index for class `c` (0: no class) of `layer`. A same-layer table is indexed by
    /// the layer's own classes; an inter-layer table holds both layers' class names (`names`) and
    /// is looked up by the class's NAME, as the reference passes names to the database rule —
    /// a name the table does not hold is "no class" (the database rule then falls to its default
    /// alike).
    pub fn class_index(&self, layer: &Layer, c: usize) -> usize {
        if self.names.is_empty() || c == 0 {
            return c;
        }
        let name = &layer.cut_classes[c - 1].name;
        self.names.iter().position(|n| n == name).unwrap_or(0)
    }
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

/// A classic TWOWIDTHS spacing table (`frSpacingTableTwConstraint`): rows and columns both by
/// width, each row with an optional parallel run length (-1 when the row gives none).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TwoWidthsTable {
    pub widths: Vec<i32>,
    pub prls: Vec<i32>,
    pub values: Vec<Vec<i32>>,
}

impl TwoWidthsTable {
    /// odb `_dbTechLayer::getTwIdx`: the row BEFORE the first row whose width is not below
    /// `width` (row 0 when that is the first, the last row when there is none), then back while
    /// the row's run length exceeds `prl`; row 0 when none takes it. A row without one (-1) takes
    /// any run.
    fn tw_idx(&self, width: i32, prl: i32) -> usize {
        let pos = self.widths.partition_point(|&w| w < width);
        let mut idx = pos.saturating_sub(1);
        loop {
            if prl >= self.prls[idx] {
                return idx;
            }
            if idx == 0 {
                return 0;
            }
            idx -= 1;
        }
    }
    /// `frSpacingTableTwConstraint::find` = odb `dbTechLayer::findTwSpacing`: the row by the
    /// first width, the column by the second, both at the run clamped at 0.
    pub fn find(&self, width1: i32, width2: i32, prl: i32) -> i32 {
        let prl = prl.max(0);
        self.values[self.tw_idx(width1, prl)][self.tw_idx(width2, prl)]
    }
}

/// A routing layer's minimum spacing rule (`frLayer::getMinSpacing`): the parallel-run table (a
/// plain SPACING is its one-cell form) or the TWOWIDTHS table, whichever the tech load set last.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MinSpacing {
    /// `frSpacingTablePrlConstraint`.
    Prl(SpacingTable),
    /// `frSpacingTableTwConstraint`.
    TwoWidths(TwoWidthsTable),
}

impl From<SpacingTable> for MinSpacing {
    fn from(t: SpacingTable) -> Self {
        MinSpacing::Prl(t)
    }
}

impl MinSpacing {
    /// The spacing two shapes of these widths need over a run: the parallel-run table on the
    /// WIDER width alone, the TWOWIDTHS table on both (every reference consumer passes them so).
    pub fn find(&self, width1: i32, width2: i32, prl: i32) -> i32 {
        match self {
            MinSpacing::Prl(t) => t.find(width1.max(width2), prl),
            MinSpacing::TwoWidths(t) => t.find(width1, width2, prl),
        }
    }
    /// `findMin`: the first row's first value.
    pub fn find_min(&self) -> i32 {
        match self {
            MinSpacing::Prl(t) => t.find_min(),
            MinSpacing::TwoWidths(t) => t.values[0][0],
        }
    }
    /// `findMax`: the last row's last value.
    pub fn find_max(&self) -> i32 {
        match self {
            MinSpacing::Prl(t) => t.find_max(),
            MinSpacing::TwoWidths(t) => *t.values.last().and_then(|r| r.last()).expect("a TWOWIDTHS table has a value"),
        }
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
    /// No wire runs across the layer's direction (`frLayer::isUnidirectional`): a multi-patterned
    /// layer (`numMasks > 1`: the router does not colour, so it routes one way) or a rect-only one
    /// (a wrong-way rectangle would be legal on one, but the rare case is ignored).
    pub fn is_unidirectional(&self) -> bool {
        self.num_masks > 1 || self.rect_only
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
    /// The via's cut class (`frViaDef::getCutClassIdx`, a class index of its cut layer from 0),
    /// set as io sets it for a technology or design via: the class of its FIRST cut figure that
    /// has one ([`via_cut_class`]). `None` (-1) for a via the router generates (io's `_FR` vias,
    /// [`crate::dr::rules::default_vias`]), which io never gives a class.
    pub cut_class: Option<usize>,
}

/// io's cut class of a via read from the technology or the design: over its cut figures in
/// order, the first whose (width, length) — the shorter and longer sides — is a class of the cut
/// layer ([`Layer::cut_class_of`]).
pub fn via_cut_class(cut_layer: &Layer, cut_figs: &[Rect]) -> Option<usize> {
    cut_figs.iter().find_map(|f| cut_layer.cut_class_of(f.dx().min(f.dy()), f.dx().max(f.dy())).checked_sub(1))
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
    /// `USEMINSPACING OBS OFF`: an obstruction is spaced by its OWN width (the reference's
    /// `USEMINSPACING_OBS`, read from `dbTech::getUseMinSpacingObs`, false); ON — the default —
    /// by the layer's minimum width. Read by the check, the maze costs and track assignment.
    pub obs_own_width: bool,
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

/// A master obstruction as read: its layer, box, and its own (DESIGNRULEWIDTH, SPACING) if either
/// is set.
pub type Obstruction = (usize, Rect, Option<(i32, i32)>);

/// A master as the router imports it: its terminals, and its obstructions as blockages.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Master {
    pub terms: Vec<MasterTerm>,
    /// Blockage rectangles by layer number (master coordinates).
    pub blockages: Vec<(usize, Rect)>,
    /// Per blockage: an obstruction's own DESIGNRULEWIDTH and SPACING (`-1` for none), when it
    /// has either — such an obstruction is its own blockage, never merged.
    pub blockage_rules: Vec<Option<(i32, i32)>>,
}

impl Master {
    /// The import rules for obstructions:
    /// - an obstruction on a CUT layer that touches the shapes of exactly ONE pin on the layer
    ///   above becomes a shape of that pin (a contact drawn as an obstruction is the pin's own);
    /// - every other obstruction merges with the rest on its layer, and the merged shapes'
    ///   MAXIMAL rectangles are the blockages.
    pub fn import(tech: &Tech, terms: Vec<MasterTerm>, obstructions: &[(usize, Rect)]) -> Master {
        let with: Vec<Obstruction> = obstructions.iter().map(|&(l, r)| (l, r, None)).collect();
        Master::import_with_rules(tech, terms, &with)
    }

    /// [`Master::import`], each obstruction with its own DESIGNRULEWIDTH / SPACING (io's master
    /// import): after the cut-to-pin rule, an obstruction carrying either is a blockage of its own,
    /// in obstruction order, BEFORE the merged ones.
    pub fn import_with_rules(tech: &Tech, mut terms: Vec<MasterTerm>, obstructions: &[Obstruction]) -> Master {
        let mut own: Vec<Obstruction> = Vec::new();
        let touches = |a: &Rect, b: &Rect| a.xl <= b.xh && b.xl <= a.xh && a.yl <= b.yh && b.yl <= a.yh;
        let mut merged: Vec<Polygon90Set> = vec![Polygon90Set::new(); tech.layers.len()];
        for &(layer, r, rule) in obstructions {
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
            if rule.is_some() {
                own.push((layer, r, rule));
                continue;
            }
            merged[layer].insert_rect(r);
        }
        let mut blockages: Vec<(usize, Rect)> = own.iter().map(|&(l, r, _)| (l, r)).collect();
        let mut blockage_rules: Vec<Option<(i32, i32)>> = own.iter().map(|&(_, _, rule)| rule).collect();
        // Per layer, per connected polygon (the reference's `get` order), that polygon's maximal
        // rectangles — NOT the layer's as one set: the order differs when a layer has two pieces.
        for (layer, set) in merged.iter_mut().enumerate() {
            for mut poly in set.polygons() {
                for r in poly.max_rectangles() {
                    blockages.push((layer, r));
                    blockage_rules.push(None);
                }
            }
        }
        Master { terms, blockages, blockage_rules }
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

    /// A cut-to-corner rule as read: spacing, second layer NAME, class index, corner form.
    type LayerRuleByName = (i32, String, usize, CutCorner);

    /// A LEF58 cut spacing rule as io reads it: kept as a cut-to-corner rule (its second layer
    /// still a name) or an adjacent-cuts rule, or SKIPPED with the clause io warns about.
    #[derive(Debug, Clone, PartialEq, Eq)]
    pub enum Lef58CutRule {
        Layer(LayerRuleByName),
        AdjacentCuts(AdjacentCuts),
        Skipped(&'static str),
    }

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
    /// A LEF58 enclosure rule io keeps: not EOL with SIDESPACING / ENDSPACING, not HORIZONTAL /
    /// VERTICAL, INCLUDEABUTTED, OFFCENTERLINE, LENGTH, EXTRACUT, REDUNDANTCUT, PARALLEL or
    /// CONCAVECORNERS, and with a CUTCLASS.
    pub fn cut_enclosure_kept(db: &Db, layer: &str, k: usize) -> bool {
        cut_enclosure_skip(db, layer, k).is_none()
    }

    /// Why the reader skips a LEF58_ENCLOSURE rule: its warning code and the unsupported form,
    /// tested in the reader's order (the first that holds is the one warned).
    pub fn cut_enclosure_skip(db: &Db, layer: &str, k: usize) -> Option<(u16, &'static str)> {
        let ty = db.cutenclosurerule_get_type(layer, k);
        if ty == "EOL" && (db.cutenclosurerule_is_side_spacing_valid(layer, k) || db.cutenclosurerule_is_end_spacing_valid(layer, k)) {
            Some((340, "EOL SIDESPACING/ENDSPACING is not supported"))
        } else if ty == "HORZ_AND_VERT" {
            Some((341, "HORIZONTAL/VERTICAL is not supported"))
        } else if db.cutenclosurerule_is_include_abutted(layer, k) {
            Some((342, "INCLUDEABUTTED is not supported"))
        } else if db.cutenclosurerule_is_off_center_line(layer, k) {
            Some((343, "OFFCENTERLINE is not supported"))
        } else if db.cutenclosurerule_is_length_valid(layer, k) {
            Some((344, "LENGTH is not supported"))
        } else if db.cutenclosurerule_is_extra_cut_valid(layer, k) {
            Some((345, "EXTRACUT is not supported"))
        } else if db.cutenclosurerule_is_redundant_cut_valid(layer, k) {
            Some((346, "REDUNDANTCUT is not supported"))
        } else if db.cutenclosurerule_is_parallel_valid(layer, k) {
            Some((347, "PARALLEL is not supported"))
        } else if db.cutenclosurerule_is_concave_corners_valid(layer, k) {
            Some((348, "CONCAVECORNERS is not supported"))
        } else if !db.cutenclosurerule_is_cut_class_valid(layer, k) {
            Some((349, "with no CUTCLASS is not supported"))
        } else {
            None
        }
    }

    /// The cut layer's LEF58 cut spacing tables as io `setCutLayerProperties` keeps them, in the
    /// technology's order (a later table of a kind replaces an earlier one): a LAYER table on layer
    /// number 1 (`num`) is skipped, then a SAMEMASK one; a LAYER table goes to the inter-layer
    /// kinds, SAMEMETAL tested first, then SAMENET, else the default one; a table on the layer
    /// itself to SAMEMETAL, then SAMENET, else different-net. Returned: (different-net, SAMENET,
    /// the default inter-layer table's rule index — its lookups need the second layer's classes,
    /// read after all layers). Refused: a SAMEMETAL table, and an inter-layer SAMENET or SAMEMETAL
    /// one.
    pub fn cut_spacing_tables(db: &Db, layer: &str, num: usize, classes: &[CutClass]) -> Result<CutSpacingTables, &'static str> {
        let mut out = CutSpacingTables::default();
        for k in 0..db.num_layer_get_tech_layer_cut_spacing_table_def_rules(layer) {
            if db.cutspacingtablerule_is_layer_valid(layer, k) && num == 1 {
                continue;
            }
            if db.cutspacingtablerule_is_same_mask(layer, k) {
                continue;
            }
            if db.cutspacingtablerule_is_layer_valid(layer, k) {
                if db.cutspacingtablerule_is_same_metal(layer, k) {
                    return Err("a SAMEMETAL LAYER (inter-layer) cut spacing table");
                }
                if db.cutspacingtablerule_is_same_net(layer, k) {
                    return Err("a SAMENET LAYER (inter-layer) cut spacing table");
                }
                if db.cutspacingtablerule_get_second_layer(layer, k).is_empty() {
                    return Err("a LAYER (inter-layer) cut spacing table without its second layer");
                }
                out.inter = Some(k);
                continue;
            }
            if db.cutspacingtablerule_is_same_metal(layer, k) {
                return Err("a SAMEMETAL cut spacing table");
            }
            let t = cut_spacing_table_rule(db, layer, k, &own_names(classes))?;
            if db.cutspacingtablerule_is_same_net(layer, k) {
                out.same_net = Some(t);
            } else {
                out.diff = Some(t);
            }
        }
        Ok(out)
    }

    /// "" (no class), then the layer's class names, in its order.
    fn own_names(classes: &[CutClass]) -> Vec<&str> {
        std::iter::once("").chain(classes.iter().map(|c| c.name.as_str())).collect()
    }

    /// The default inter-layer table (rule `k` of cut layer `layer`) over every class NAME of the
    /// layer and of its second layer `second` (`second_classes`), with its second layer index.
    pub fn inter_cut_table(db: &Db, layer: &str, k: usize, classes: &[CutClass], second: usize, second_classes: &[CutClass]) -> Result<InterCutTable, &'static str> {
        let mut names = own_names(classes);
        for c in second_classes {
            if !names.contains(&c.name.as_str()) {
                names.push(c.name.as_str());
            }
        }
        let mut table = cut_spacing_table_rule(db, layer, k, &names)?;
        table.names = names.iter().map(|n| n.to_string()).collect();
        Ok(InterCutTable { second_layer: second, no_stack: db.cutspacingtablerule_is_no_stack(layer, k), table })
    }

    /// One LEF58 cut spacing table rule's lookups, evaluated for every class pair.
    fn cut_spacing_table_rule(db: &Db, layer: &str, k: usize, names: &[&str]) -> Result<CutSpacingTable, &'static str> {
        {
            let n = names.len();
            let mut t = CutSpacingTable { n, no_prl: db.cutspacingtablerule_is_no_prl(layer, k), horizontal: db.cutspacingtablerule_is_horizontal(layer, k), vertical: db.cutspacingtablerule_is_vertical(layer, k), ..Default::default() };
            for &c1 in names {
                for s1 in [false, true] {
                    for &c2 in names {
                        for s2 in [false, true] {
                            t.spacing.push((db.cutspacingtablerule_get_spacing(layer, k, c1, s1, c2, s2, "FIRST"), db.cutspacingtablerule_get_spacing(layer, k, c1, s1, c2, s2, "SECOND")));
                        }
                    }
                }
                t.max_spacing.push([db.cutspacingtablerule_get_max_spacing_cut_class_side(layer, k, c1, false), db.cutspacingtablerule_get_max_spacing_cut_class_side(layer, k, c1, true)]);
                t.exact_aligned.push(db.cutspacingtablerule_get_exact_aligned_spacing(layer, k, c1));
                for &c2 in names {
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
            Ok(t)
        }
    }

    /// LEF58 corner spacing rule `k` of `layer`, as `io::Parser` translates it, or the first clause
    /// outside the modelled subset (CONVEXCORNER with its width table, CORNERONLY, CORNERTOCORNER;
    /// not CONCAVECORNER, SAMEMASK or EXCEPTEOL) — or an empty table, which the reference would
    /// index out of bounds.
    /// LEF58 cut spacing rule `k` of cut layer `layer`, if it is a modelled form — `SPACING s
    /// LAYER l CUTCLASS c CONCAVECORNER` or `SPACING s LAYER l CUTCLASS c NONEOLCONVEXCORNER w
    /// [MINLENGTH m]` to a routing layer, nothing else — as its spacing, second layer name, class
    /// index (the class's LAST place + 1: io looks the name up in a map written in class order) and
    /// corner form (io keeps the end-of-line width, and MINLENGTH, only under NONEOLCONVEXCORNER).
    /// `Err` names the first clause outside the forms: every other type (ADJACENTCUTS,
    /// PARALLELOVERLAP, …), and on a LAYER rule each option io drops with a warning (STACK,
    /// ORTHOGONALSPACING, SHORTEDGEONLY, the CONCAVECORNER WIDTH / PARALLEL / EDGELENGTH branches,
    /// EXTENSION, ABOVEWIDTH, MASKOVERLAP, WRONGDIRECTION) or keeps but this engine does not model
    /// (on a corner form: CENTERTOCENTER, SAMENET, SAMEMETAL, SAMEVIA, PRL). `Skipped`, as io
    /// keeps them and their check never marks: a rule with NO CUTCLASS (after STACK and
    /// ORTHOGONALSPACING, which io tests first; `spc_layer` acts only under `hasCutClass()`), and
    /// a rule with a defined class but neither corner (after the branches io drops; `spc_layer`
    /// has no third branch).
    pub fn cut_layer_rule(db: &Db, layer: &str, k: usize, classes: &[CutClass]) -> Result<Lef58CutRule, &'static str> {
        let ty = db.cutspacingrule_get_type(layer, k);
        if ty == "ADJACENTCUTS" {
            return adjacent_cuts_rule(db, layer, k, classes);
        }
        if ty != "LAYER" {
            return Err(match ty.as_str() {
                "PARALLELOVERLAP" => "PARALLELOVERLAP",
                "PARALLELWITHIN" => "PARALLELWITHIN",
                "SAMEMETALSHAREDEDGE" => "SAMEMETALSHAREDEDGE",
                "AREA" => "AREA",
                "MAXXY" => "MAXXY",
                "SAMEMASK" => "SAMEMASK",
                _ => "a cut spacing rule of no type",
            });
        }
        let second = db.cutspacingrule_get_second_layer(layer, k);
        if second.is_empty() || db.layer_get_type(&second).map_or(true, |t| t != "ROUTING") {
            return Err("LAYER to a layer that is not a routing layer");
        }
        let class = db.cutspacingrule_get_cut_class(layer, k);
        let concave = db.cutspacingrule_is_concave_corner(layer, k);
        let convex = db.cutspacingrule_is_non_eol_convex_corner(layer, k);
        let unmodelled: [(bool, &'static str); 2] = [
            (db.cutspacingrule_is_stack(layer, k), "STACK"),
            (db.cutspacingrule_is_orthogonal_spacing_valid(layer, k), "ORTHOGONALSPACING"),
        ];
        if let Some((_, clause)) = unmodelled.iter().find(|(set, _)| *set) {
            return Err(clause);
        }
        // ⛔ Divergence D6 (register; upstream OpenROAD issue #12187). Upstream rule: io keeps a
        // rule with no CUTCLASS (spacing, CENTERTOCENTER, SAMENET / SAMEMETAL / SAMEVIA and the
        // second layer only; a SAMENET one in the same-net list, which then makes every
        // different-net rule skip other nets), but `checkLef58CutSpacing_spc_layer` acts only
        // under `hasCutClass()`, so the rule never marks, and nothing warns. Our rule, the
        // reference patched with vyges-openroad-patches upstream-fix 0005: warn (DRT-353) and
        // drop the rule, from both lists.
        if class.is_empty() {
            return Ok(Lef58CutRule::Skipped(if db.cutspacingrule_is_same_net(layer, k) { LAYER_NO_CLASS_SAME_NET } else { LAYER_NO_CLASS }));
        }
        let unmodelled: [(bool, &'static str); 8] = [
            (db.cutspacingrule_is_short_edge_only(layer, k), "SHORTEDGEONLY"),
            (db.cutspacingrule_is_concave_corner_width(layer, k), "CONCAVECORNER WIDTH"),
            (db.cutspacingrule_is_concave_corner_parallel(layer, k), "CONCAVECORNER PARALLEL"),
            (db.cutspacingrule_is_concave_corner_edge_length(layer, k), "CONCAVECORNER EDGELENGTH"),
            (db.cutspacingrule_is_extension_valid(layer, k), "EXTENSION"),
            (db.cutspacingrule_is_above_width_valid(layer, k), "ABOVEWIDTH"),
            (db.cutspacingrule_is_mask_overlap(layer, k), "MASKOVERLAP"),
            (db.cutspacingrule_is_wrong_direction(layer, k), "WRONGDIRECTION"),
        ];
        if let Some((_, clause)) = unmodelled.iter().find(|(set, _)| *set) {
            return Err(clause);
        }
        let Some(c) = classes.iter().rposition(|c| c.name == class) else {
            return Err("a CUTCLASS the layer does not define");
        };
        // ⛔ Divergence D6 (#12187). Upstream rule: io keeps a rule with a class but neither corner
        // (with CENTERTOCENTER, SAMENET / SAMEMETAL / SAMEVIA, PRL; a SAMENET one in the same-net
        // list), but `checkLef58CutSpacing_spc_layer`, past the class test, has only the
        // CONCAVECORNER and NONEOLCONVEXCORNER branches, so the rule never marks, and nothing
        // warns. Our rule, upstream-fix 0005: warn (DRT-354, after io's other CUTCLASS-branch
        // skips) and drop the rule.
        if !concave && !convex {
            return Ok(Lef58CutRule::Skipped(if db.cutspacingrule_is_same_net(layer, k) { LAYER_NO_CORNER_SAME_NET } else { LAYER_NO_CORNER }));
        }
        let unmodelled: [(bool, &'static str); 4] = [
            (db.cutspacingrule_is_center_to_center(layer, k), "CENTERTOCENTER"),
            (db.cutspacingrule_is_same_net(layer, k), "SAMENET"),
            (db.cutspacingrule_is_same_metal(layer, k) || db.cutspacingrule_is_same_via(layer, k), "SAMEMETAL/SAMEVIA"),
            (db.cutspacingrule_is_prl_valid(layer, k), "PRL"),
        ];
        if let Some((_, clause)) = unmodelled.iter().find(|(set, _)| *set) {
            return Err(clause);
        }
        // `getType` / the grammar: CONCAVECORNER and NONEOLCONVEXCORNER are alternatives, and the
        // check's concave branch comes first.
        let corner = if concave {
            CutCorner::Concave
        } else {
            let min_length = db.cutspacingrule_is_min_length_valid(layer, k).then(|| db.cutspacingrule_get_min_length(layer, k));
            CutCorner::NonEolConvex { eol_width: db.cutspacingrule_get_eol_width(layer, k), min_length }
        };
        Ok(Lef58CutRule::Layer((db.cutspacingrule_get_cut_spacing(layer, k), second, c + 1, corner)))
    }

    /// The skipped clause of a LEF58 cut spacing LAYER rule with no CUTCLASS.
    pub const LAYER_NO_CLASS: &str = "LAYER without CUTCLASS";
    /// The same, with SAMENET (the unpatched reference's same-net list).
    pub const LAYER_NO_CLASS_SAME_NET: &str = "LAYER without CUTCLASS, SAMENET";
    /// The skipped clause of a LEF58 cut spacing LAYER rule with a CUTCLASS but neither corner.
    pub const LAYER_NO_CORNER: &str = "LAYER CUTCLASS without CONCAVECORNER or NONEOLCONVEXCORNER";
    /// The same, with SAMENET (the unpatched reference's same-net list).
    pub const LAYER_NO_CORNER_SAME_NET: &str = "LAYER CUTCLASS without CONCAVECORNER or NONEOLCONVEXCORNER, SAMENET";

    /// Every clause of a LAYER rule io keeps but the reference's check never marks.
    pub const LAYER_UNCHECKED: [&str; 4] = [LAYER_NO_CLASS, LAYER_NO_CLASS_SAME_NET, LAYER_NO_CORNER, LAYER_NO_CORNER_SAME_NET];

    // ⛔ No same-net list (divergence D6, upstream-fix 0005): the unpatched reference keeps a
    // SAMENET LAYER rule with no CUTCLASS, or a class but neither corner, in the layer's same-net
    // list, and `checkCutSpacing_main(rect)` then runs every different-net LEF58 rule with
    // skipDiffNet, pairing a cut only with shapes of its own net (OpenROAD issue #12188). The
    // patched reference drops those rules, so the list holds no rule this engine reads, and a
    // checked rule beside one judges every net. Every other SAMENET form is refused.

    /// The clause of an ADJACENTCUTS rule with two WITHIN values, which this engine skips.
    ///
    /// ⛔ A DIVERGENCE from the reference by decision (drt divergence register D7). Upstream rule
    /// (io `setCutLayerProperties`): only the first WITHIN is read, and GC uses it as the upper
    /// bound, so `WITHIN w1 w2` is checked as `WITHIN w1` (upstream OpenROAD issue #12202). Our
    /// rule, the reference patched with vyges-openroad-patches upstream-fix 0012: warn (DRT-359,
    /// [`lef58_cut_spacing_skip_warnings`]) and skip the rule, after the reference's own skips.
    pub const ADJ_TWO_WITHIN: &str = "ADJACENTCUTS WITHIN with two values";

    /// The patched reference's io warnings for the LEF58 cut spacing rules of cut layer `layer`
    /// that only its fixes skip, in rule order (io's one loop over the rules): DRT-359 for an
    /// ADJACENTCUTS rule with a second WITHIN value ([`ADJ_TWO_WITHIN`], upstream-fix 0012,
    /// divergence D7, OpenROAD issue #12202), DRT-353 for a LAYER rule with no CUTCLASS and DRT-354
    /// for one with a CUTCLASS but neither corner ([`LAYER_UNCHECKED`], upstream-fix 0005,
    /// divergence D6, #12187). The unpatched reference says nothing for any of them.
    pub fn lef58_cut_spacing_skip_warnings(db: &Db, layer: &str, classes: &[CutClass]) -> Vec<(u16, String)> {
        (0..db.num_layer_get_tech_layer_cut_spacing_rules(layer))
            .filter_map(|k| match cut_layer_rule(db, layer, k, classes) {
                Ok(Lef58CutRule::Skipped(c)) if c == ADJ_TWO_WITHIN => Some((359, format!("Unsupported branch WITHIN with two values in LEF58_SPACING ADJACENTCUTS for layer {layer}, skipping rule."))),
                Ok(Lef58CutRule::Skipped(c)) if c == LAYER_NO_CLASS || c == LAYER_NO_CLASS_SAME_NET => Some((353, format!("Unsupported LEF58_SPACING LAYER rule without CUTCLASS for layer {layer}, skipping rule."))),
                Ok(Lef58CutRule::Skipped(c)) if c == LAYER_NO_CORNER || c == LAYER_NO_CORNER_SAME_NET => Some((354, format!("Unsupported LEF58_SPACING LAYER CUTCLASS rule without CONCAVECORNER or NONEOLCONVEXCORNER for layer {layer}, skipping rule."))),
                _ => None,
            })
            .collect()
    }

    /// io's ADJACENTCUTS branch for LEF58 cut spacing rule `k` of cut layer `layer`: EXACTALIGNED,
    /// EXCEPTSAMEPGNET, CUTCLASS … TO ALL, SIDEPARALLELOVERLAP and SAMEMASK, tested in that order,
    /// skip the rule (io warns and drops it), then a second WITHIN value ([`ADJ_TWO_WITHIN`]:
    /// skipped as the reference patched with upstream-fix 0012 skips it, divergence D7);
    /// otherwise io keeps the spacing, CENTERTOCENTER, the adjacent-cut count, TWOCUTS, the WITHIN
    /// value, the class (none when the rule names none) and NOPRL. `Err` names a clause io keeps
    /// but this engine does not model: SAMENET (the rule would join the same-net list, which
    /// changes how the different-net rules judge one net), SAMEMETAL or SAMEVIA and TWOCUTS
    /// SAMECUT (kept by io, read by no check).
    pub fn adjacent_cuts_rule(db: &Db, layer: &str, k: usize, classes: &[CutClass]) -> Result<Lef58CutRule, &'static str> {
        let skipped: [(bool, &'static str); 6] = [
            (db.cutspacingrule_is_exact_aligned(layer, k), "ADJACENTCUTS EXACTALIGNED"),
            (db.cutspacingrule_is_except_same_pgnet(layer, k), "ADJACENTCUTS EXCEPTSAMEPGNET"),
            (db.cutspacingrule_is_cut_class_to_all(layer, k), "ADJACENTCUTS CUTCLASS TO ALL"),
            (db.cutspacingrule_is_side_parallel_overlap(layer, k), "ADJACENTCUTS SIDEPARALLELOVERLAP"),
            (db.cutspacingrule_is_same_mask(layer, k), "ADJACENTCUTS SAMEMASK"),
            (db.cutspacingrule_get_second_within(layer, k) > 0, ADJ_TWO_WITHIN),
        ];
        if let Some((_, clause)) = skipped.iter().find(|(set, _)| *set) {
            return Ok(Lef58CutRule::Skipped(clause));
        }
        let unmodelled: [(bool, &'static str); 3] = [
            (db.cutspacingrule_is_same_net(layer, k), "ADJACENTCUTS with SAMENET"),
            (db.cutspacingrule_is_same_metal(layer, k) || db.cutspacingrule_is_same_via(layer, k), "ADJACENTCUTS with SAMEMETAL or SAMEVIA"),
            (db.cutspacingrule_is_same_cut(layer, k), "ADJACENTCUTS TWOCUTS SAMECUT"),
        ];
        if let Some((_, clause)) = unmodelled.iter().find(|(set, _)| *set) {
            return Err(clause);
        }
        let name = db.cutspacingrule_get_cut_class(layer, k);
        let class = if name.is_empty() {
            0
        } else {
            match classes.iter().rposition(|c| c.name == name) {
                Some(c) => c + 1,
                None => return Err("a CUTCLASS the layer does not define"),
            }
        };
        Ok(Lef58CutRule::AdjacentCuts(AdjacentCuts {
            spacing: db.cutspacingrule_get_cut_spacing(layer, k),
            center_to_center: db.cutspacingrule_is_center_to_center(layer, k),
            adjacent_cuts: db.cutspacingrule_get_adjacent_cuts(layer, k) as i32,
            two_cuts: db.cutspacingrule_is_two_cuts_valid(layer, k).then(|| db.cutspacingrule_get_two_cuts(layer, k) as i32),
            within: db.cutspacingrule_get_within(layer, k),
            class,
            no_prl: db.cutspacingrule_is_no_prl(layer, k),
        }))
    }

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

    /// Whether the reference's io DROPS LEF58 end-of-line rule `k` of `layer` with a warning
    /// (EXCEPTEXACTWIDTH, FILLCONCAVECORNER, EQUALRECTWIDTH). It tests these before any other clause.
    pub fn lef58_eol_dropped_by_reference(db: &Db, layer: &str, k: usize) -> bool {
        db.spacingeolrule_is_except_exact_width_valid(layer, k) || db.spacingeolrule_is_fill_concave_corner_valid(layer, k) || db.spacingeolrule_is_equal_rect_width_valid(layer, k)
    }

    /// The clause that makes this engine skip LEF58 end-of-line rule `k` of `layer` with a
    /// warning: `TOCONCAVECORNER` or `TONOTCHLENGTH`. Skipped, not enforced and not emulated.
    ///
    /// ⛔ A DIVERGENCE from the reference by decision (drt divergence register D10). Upstream rule
    /// (io `setRoutingLayerProperties`): neither branch is tested, so the rule is kept as a plain
    /// end-of-line rule whose WITHIN is the database default 0, MINADJACENTLENGTH dropped, and line
    /// ends that face straight edges are flagged (upstream OpenROAD issue #12174). Our rule (the
    /// reference patched with vyges-openroad-patches upstream-fix 0010): warn (DRT-357 / DRT-358,
    /// [`lef58_eol_skip_warnings`]) and skip the rule; it is not checked during routing and
    /// sign-off DRC must cover it. A rule the reference drops itself
    /// ([`lef58_eol_dropped_by_reference`]) is not one of these.
    pub fn lef58_eol_skipped_clause(db: &Db, layer: &str, k: usize) -> Option<&'static str> {
        // ⚠️ Unreachable through odb's LEF grammar (the dropped clauses belong to the WITHIN
        // branch, exclusive with TOCONCAVECORNER and TONOTCHLENGTH), so no test can fail on it
        // (the mutant removing it survives as EQUIVALENT); kept so the reference's own skip always
        // wins over ours should a database ever carry both.
        if lef58_eol_dropped_by_reference(db, layer, k) {
            None
        } else if db.spacingeolrule_is_to_concave_corner_valid(layer, k) {
            Some("TOCONCAVECORNER")
        } else if db.spacingeolrule_is_to_notch_length_valid(layer, k) {
            Some("TONOTCHLENGTH")
        } else {
            None
        }
    }

    /// The patched reference's io warnings (upstream-fix 0010) for the LEF58 end-of-line rules of
    /// routing layer `layer` that it skips ([`lef58_eol_skipped_clause`]), in the rules' order:
    /// DRT-357 for TOCONCAVECORNER, DRT-358 for TONOTCHLENGTH. ⛔ Divergence D10 (upstream OpenROAD
    /// issue #12174): the unpatched reference says nothing and checks the rule as WITHIN 0.
    pub fn lef58_eol_skip_warnings(db: &Db, layer: &str) -> Vec<(u16, String)> {
        (0..db.num_layer_get_tech_layer_spacing_eol_rules(layer))
            .filter_map(|k| match lef58_eol_skipped_clause(db, layer, k)? {
                "TOCONCAVECORNER" => Some((357, format!("Unsupported LEF58_SPACING rule with option TOCONCAVECORNER for layer {layer}."))),
                _ => Some((358, format!("Unsupported LEF58_SPACING rule with option TONOTCHLENGTH for layer {layer}."))),
            })
            .collect()
    }

    /// LEF58 end-of-line spacing rule `k` of `layer`, as `io::Parser` translates it
    /// (`frLef58SpacingEndOfLineConstraint`): `Ok(None)` for a rule the reference DROPS with a warning
    /// (EXCEPTEXACTWIDTH, FILLCONCAVECORNER, EQUALRECTWIDTH — tested first, as it does) and for a rule
    /// this engine SKIPS with a warning of its own ([`lef58_eol_skipped_clause`]: TOCONCAVECORNER,
    /// TONOTCHLENGTH), `Err(clause)` for a clause outside the modelled subset (SPACING / ENDOFLINE /
    /// OPPOSITEWIDTH (register D13), WITHIN, ENDTOEND with its EXTENSION but without OTHERENDWIDTH
    /// or cut spaces, PARALLELEDGE with only its space, within and TWOEDGES, MINLENGTH / MAXLENGTH
    /// with TWOSIDES read as written: [`lef58_eol_length`]; SAMEMASK with its WITHIN as written).
    ///
    /// ⛔ SAMEMASK is a DIVERGENCE by decision (drt divergence register D8, upstream OpenROAD issue
    /// #12182, vyges-openroad-patches upstream-fix 0002). Upstream rule:
    /// `frLef58SpacingEndOfLineWithinConstraint::getEolWithin` returns 0 for a SAMEMASK rule,
    /// whatever WITHIN says, so the check and the via-to-via prep lose the lateral window. Our rule
    /// (the patched reference's): WITHIN as written (the foundry kit's SAMEMASK end-of-line rules
    /// state it on purpose). As in the reference, the check and the router read the rule
    /// MASK-BLIND (drt never reads `hasSameMask()`): every pair is held to it, whatever its masks;
    /// the colour step (`vyges-opendb color-masks`) checks it per mask. That is not part of the
    /// divergence.
    pub fn lef58_eol_rule(db: &Db, layer: &str, k: usize) -> Result<Option<EolRule>, &'static str> {
        if lef58_eol_dropped_by_reference(db, layer, k) || lef58_eol_skipped_clause(db, layer, k).is_some() {
            return Ok(None);
        }
        let unmodelled: [(bool, &'static str); 16] = [
            (db.spacingeolrule_is_exact_width_valid(layer, k), "EXACTWIDTH"),
            (db.spacingeolrule_is_wrong_dir_spacing_valid(layer, k), "WRONGDIRSPACING"),
            (db.spacingeolrule_is_end_prl_spacing_valid(layer, k), "ENDPRLSPACING"),
            (db.spacingeolrule_is_wrong_dir_within_valid(layer, k), "WRONGDIRWITHIN"),
            (db.spacingeolrule_is_other_end_width_valid(layer, k), "ENDTOEND OTHERENDWIDTH"),
            (db.spacingeolrule_is_cut_spaces_valid(layer, k), "ENDTOEND cut spaces"),
            (db.spacingeolrule_is_subtract_eol_width_valid(layer, k), "PARALLELEDGE SUBTRACTEOLWIDTH"),
            (db.spacingeolrule_is_par_prl_valid(layer, k), "PARALLELEDGE PRL"),
            (db.spacingeolrule_is_par_min_length_valid(layer, k), "PARALLELEDGE MINLENGTH"),
            (db.spacingeolrule_is_same_metal_valid(layer, k), "PARALLELEDGE SAMEMETAL"),
            (db.spacingeolrule_is_non_eol_corner_only_valid(layer, k), "PARALLELEDGE NONEOLCORNERONLY"),
            (db.spacingeolrule_is_parallel_same_mask_valid(layer, k), "PARALLELEDGE PARALLELSAMEMASK"),
            (db.spacingeolrule_is_enclose_cut_valid(layer, k), "ENCLOSECUT"),
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
            length: lef58_eol_length(db, layer, k),
            // io keeps EXTENSION only under ENDTOEND (`setExtension` on the end-to-end
            // constraint). Its wrong-direction value (`isWrongDirExtensionValid`) is set there
            // too but nothing in drt reads it, so it is not kept.
            extension: (db.spacingeolrule_is_end_to_end_valid(layer, k)
                && db.spacingeolrule_is_extension_valid(layer, k))
            .then(|| db.spacingeolrule_get_extension(layer, k)),
            // Read as the LEF defines it, where the reference reads nothing (register D13).
            opposite_width: db
                .spacingeolrule_is_opposite_width_valid(layer, k)
                .then(|| db.spacingeolrule_get_opposite_width(layer, k)),
        }))
    }

    /// io's MINLENGTH / MAXLENGTH of LEF58 end-of-line rule `k` (MINLENGTH when the database
    /// holds both), with TWOSIDES AS WRITTEN (`isTwoSidesValid()`). ⛔ Diverges from the
    /// reference by decision (register D8): its io passes PARALLELEDGE's TWOEDGES flag
    /// (`isTwoEdgesValid()`) here, upstream OpenROAD issue #12181; we follow the patched reference
    /// (vyges-openroad-patches upstream-fix 0001).
    pub fn lef58_eol_length(db: &Db, layer: &str, k: usize) -> Option<EolLength> {
        let two_sides = db.spacingeolrule_is_two_sides_valid(layer, k);
        if db.spacingeolrule_is_min_length_valid(layer, k) {
            Some(EolLength { max: false, length: db.spacingeolrule_get_min_length(layer, k), two_sides })
        } else if db.spacingeolrule_is_max_length_valid(layer, k) {
            Some(EolLength { max: true, length: db.spacingeolrule_get_max_length(layer, k), two_sides })
        } else {
            None
        }
    }

    pub fn tech(db: &Db) -> Res<Tech> {
        let mut layers: Vec<Layer> = Vec::new();
        let placeholder = |name: &str| Layer { name: name.into(), ..Layer::default() };
        // The last masterslice layer read so far that is not a well or diffusion layer.
        let mut masterslice: Option<String> = None;
        // Cut-to-corner rules by cut layer, their second layer by name until every layer is read
        // (`io_parser_helper` resolves it after the technology, the metal above being defined
        // after the cut).
        let mut layer_rules_by_name: Vec<(usize, Vec<Lef58CutRule>)> = Vec::new();
        // Default inter-layer cut spacing tables: (cut layer, rule index).
        let mut inter_tables: Vec<(usize, usize)> = Vec::new();
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
                    let prl_spacing = match (v55.widths_and_lengths, v55.table) {
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
                    // io `setRoutingLayerProperties` sets the TWOWIDTHS table after the parallel-run
                    // one, so on a layer carrying both it is the one kept.
                    let spacing = match db.layer_two_widths_spacing_table(&name)? {
                        Some(t) => Some(MinSpacing::TwoWidths(TwoWidthsTable { widths: t.widths, prls: t.prls, values: t.spacing })),
                        None => prl_spacing.map(MinSpacing::Prl),
                    };
                    let eol = db
                        .layer_v54_eol_rules(&name)?
                        .into_iter()
                        .map(|(space, width, within, par)| EolRule { space: space as i32, width, within, parallel: par.map(|(space, within, two_edges)| ParallelEdge { space, within, two_edges }), end_to_end: None, length: None, extension: None, opposite_width: None })
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
                    let num_masks = db.layer_get_num_masks(&name);
                    // io: "We don't handle coloring so tracks on any multiple patterned layer are
                    // forced to be on grid" — the constraint for either flag.
                    let right_way_on_grid_only = db.layer_is_right_way_on_grid_only(&name) || num_masks > 1;
                    let spacing_ranges: Vec<SpacingRange> = db
                        .layer_v54_spacing_rules(&name)
                        .unwrap_or_default()
                        .into_iter()
                        .filter_map(|(sp, range)| range.map(|(lo, hi)| SpacingRange { min_spacing: sp as i32, min_width: lo as i32, max_width: hi as i32 }))
                        .collect();
                    // io: MAXEDGES makes the type unknown.
                    let min_step = db.layer_has_min_step(&name).then(|| {
                        let mut kind = match db.layer_get_min_step_type(&name).as_str() {
                            "INSIDECORNER" => Some(MinStepKind::InsideCorner),
                            "OUTSIDECORNER" => Some(MinStepKind::OutsideCorner),
                            "STEP" => Some(MinStepKind::Step),
                            _ => None,
                        };
                        let max_length = if db.layer_has_min_step_max_length(&name) { db.layer_get_min_step_max_length(&name) as i32 } else { -1 };
                        let mut max_edges = -1;
                        if db.layer_has_min_step_max_edges(&name) {
                            max_edges = db.layer_get_min_step_max_edges(&name) as i32;
                            kind = None;
                        }
                        MinStep { min_step_length: db.layer_get_min_step(&name) as i32, kind, max_length, max_edges }
                    });
                    let min_cuts = v54_min_cuts(db, &name);
                    let lef58_area = lef58_area_rules(db, &name);
                    let lef58_min_step = lef58_min_step_rules(db, &name);
                    let width_tbl_orth = width_tbl_orth(db, &name, dir == Dir::Horizontal);
                    layers.push(Layer { name, kind: LayerKind::Routing, dir, width, min_width, pitch, wrong_way_width, spacing, cut_spacing: None, cut_classes: vec![], cut_table: None, cut_table_same_net: None, cut_table_inter: None, cut_enclosures: vec![], lef58_cut_spacing: vec![], eol, lef58_eol, eol_keepout, corner_spacing, min_area, min_enclosed_areas, rect_only, right_way_on_grid_only, num_masks, min_cuts, mt_safe_dist: 0, min_step, lef58_min_step, spacing_ranges, lef58_area, orth_spacing: vec![], lef58_max_spacing: vec![], width_tbl_orth });
                }
                // `addCutLayer`: a MIM capacitor cut is not read; the first layer read being a cut
                // layer, the masterslice placeholder goes below it and the cut layer itself is
                // layer 1 (no fake cut then); a cut layer right above another is not read.
                "CUT" if db.layer_lef58_type(&name) != "MIMCAP" && {
                    if layers.is_empty() {
                        layers.push(placeholder(masterslice.as_deref().unwrap_or("FR_MASTERSLICE")));
                    }
                    let lower = db.layer_get_lower_layer(&name);
                    lower.is_empty() || db.layer_get_type(&lower).map_or(true, |t| t != "CUT")
                } => {
                    let width = db.layer_get_width(&name) as i32;
                    let cut_spacing = Some(db.layer_get_spacing(&name)).filter(|&s| s > 0);
                    let cut_classes = cut_classes(db, &name);
                    let tables = cut_spacing_tables(db, &name, layers.len(), &cut_classes).unwrap_or_default();
                    if let Some(k) = tables.inter {
                        inter_tables.push((layers.len(), k));
                    }
                    let (cut_table, cut_table_same_net) = (tables.diff, tables.same_net);
                    let cut_enclosures = (0..db.num_layer_get_tech_layer_cut_enclosure_rules(&name))
                        .filter(|&k| cut_enclosure_kept(db, &name, k))
                        .filter_map(|k| {
                            let class = cut_classes.iter().position(|c| c.name == db.cutenclosurerule_get_cut_class(&name, k))? + 1;
                            let ty = db.cutenclosurerule_get_type(&name, k);
                            Some(CutEnclosure {
                                class,
                                above_only: db.cutenclosurerule_is_above(&name, k),
                                below_only: db.cutenclosurerule_is_below(&name, k),
                                eol: ty == "EOL",
                                endside: ty == "ENDSIDE",
                                first: db.cutenclosurerule_get_first_overhang(&name, k),
                                second: db.cutenclosurerule_get_second_overhang(&name, k),
                                min_width: db.cutenclosurerule_get_min_width(&name, k),
                            })
                        })
                        .collect();
                    // io's list: the rules it keeps, in order (a skipped rule is dropped when the
                    // second layers are resolved, below).
                    let rules: Vec<Lef58CutRule> = (0..db.num_layer_get_tech_layer_cut_spacing_rules(&name)).filter_map(|k| cut_layer_rule(db, &name, k, &cut_classes).ok()).collect();
                    if !rules.is_empty() {
                        layer_rules_by_name.push((layers.len(), rules));
                    }
                    let orth_spacing = read::orth_spacing_table(db, &name);
                    let lef58_max_spacing = read::max_spacing_rules(db, &name, &cut_classes);
                    layers.push(Layer { name, kind: LayerKind::Cut, width, cut_spacing, cut_classes, cut_table, cut_table_same_net, cut_enclosures, orth_spacing, lef58_max_spacing, ..Layer::default() });
                }
                _ => {}
            }
        }
        // io's tables are built as the layer is read; the inter-layer table's lookups need its
        // second layer's classes (a LAYER the table names that was not read as a cut layer is
        // left out, and refused by the census).
        for (cut, k) in inter_tables {
            let second = db.cutspacingtablerule_get_second_layer(&layers[cut].name, k);
            let Some(sl) = layers.iter().position(|l| l.name == second && l.kind == LayerKind::Cut) else { continue };
            if let Ok(t) = inter_cut_table(db, &layers[cut].name, k, &layers[cut].cut_classes, sl, &layers[sl].cut_classes) {
                layers[cut].cut_table_inter = Some(t);
            }
        }
        for (cut, rules) in layer_rules_by_name {
            layers[cut].lef58_cut_spacing = rules
                .into_iter()
                .filter_map(|r| match r {
                    Lef58CutRule::Layer((spacing, second, class, corner)) => Some(Lef58CutSpacing::Layer(CutLayerSpacing { spacing, second_layer: layers.iter().position(|l| l.name == second && l.kind == LayerKind::Routing)?, class, corner })),
                    Lef58CutRule::AdjacentCuts(a) => Some(Lef58CutSpacing::AdjacentCuts(a)),
                    Lef58CutRule::Skipped(_) => None,
                })
                .collect();
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
            let cut_figs = figs(cut);
            let cut_class = via_cut_class(&layers[cut], &cut_figs);
            via_defs.push(ViaDef { name: via.clone(), is_default: db.techvia_is_default(&via), layer1: l1, cut, layer2: l2, layer1_figs: figs(l1), cut_figs, layer2_figs: figs(l2), cut_class });
        }
        // io postProcess: `convertLef58MinCutConstraints` runs after the default vias are chosen.
        convert_lef58_min_cut_constraints(db, &mut layers, None).map_err(|e| e.to_string())?;
        Ok(Tech { layers, manufacturing_grid: db.tech_get_manufacturing_grid(), via_defs, obs_own_width: !db.tech_use_min_spacing_obs() })
    }

    /// io (v5.4 MINIMUMCUT): a rule without a minimum-cuts value is skipped, and so is one asking
    /// for a single cut ([`MinCut::applies`]); BELOW ONLY overrides ABOVE.
    pub fn v54_min_cuts(db: &Db, name: &str) -> Vec<MinCut> {
        (0..db.num_layer_get_min_cut_rules(name))
            .filter(|&k| db.v54mincutrule_get_minimum_cuts_valid(name, k))
            .filter(|&k| MinCut::applies(db.v54mincutrule_get_minimum_cuts_numcuts(name, k) as i32))
            .map(|k| {
                let mut from_above = None;
                if db.v54mincutrule_is_above_only(name, k) {
                    from_above = Some(true);
                }
                if db.v54mincutrule_is_below_only(name, k) {
                    from_above = Some(false);
                }
                MinCut {
                    num_cuts: db.v54mincutrule_get_minimum_cuts_numcuts(name, k) as i32,
                    width: db.v54mincutrule_get_minimum_cuts_width(name, k) as i32,
                    within: db.v54mincutrule_get_cut_distance_valid(name, k).then(|| db.v54mincutrule_get_cut_distance_cut_distance(name, k) as i32),
                    from_above,
                    length: db.v54mincutrule_get_length_for_cuts_valid(name, k).then(|| (db.v54mincutrule_get_length_for_cuts_length(name, k) as i32, db.v54mincutrule_get_length_for_cuts_distance(name, k) as i32)),
                }
            })
            .collect()
    }

    /// io `setRoutingLayerProperties`, its LEF58 MINIMUMCUT loop: a rule with AREA,
    /// SAMEMETALOVERLAP or FULLYENCLOSED is skipped (the reference warns and never checks it);
    /// every other rule is kept for the conversion, in the layer's order.
    pub fn lef58_min_cut_kept(db: &Db, layer: &str, k: usize) -> bool {
        !(db.mincutrule_is_area_valid(layer, k) || db.mincutrule_is_same_metal_overlap(layer, k) || db.mincutrule_is_fully_enclosed(layer, k))
    }

    /// The clause that makes a LEF58 MINIMUMCUT rule SKIPPED with a warning: AREA,
    /// SAMEMETALOVERLAP or FULLYENCLOSED (the reference's read loop, in its order, warns and skips
    /// them). The reference routes without the rule; so does this router, but it says so. A rule
    /// written per cut class is NOT skipped (divergence D9: [`convert_lef58_min_cut_constraints`]).
    pub fn lef58_min_cut_skipped_clause(db: &Db, layer: &str, k: usize) -> Option<&'static str> {
        if db.mincutrule_is_area_valid(layer, k) {
            Some("AREA")
        } else if db.mincutrule_is_same_metal_overlap(layer, k) {
            Some("SAMEMETALOVERLAP")
        } else if db.mincutrule_is_fully_enclosed(layer, k) {
            Some("FULLYENCLOSED")
        } else {
            None
        }
    }

    /// A cut layer's SPACINGTABLE ORTHOGONAL as io reads it (`getOrthSpacingTable`): its
    /// (WITHIN, SPACING) rows in the database's order, none without a table.
    pub fn orth_spacing_table(db: &Db, layer: &str) -> Vec<(i32, i32)> {
        if !db.layer_has_orth_spacing_table(layer) {
            return Vec::new();
        }
        db.layer_get_orth_spacing_table(layer).as_chunks::<2>().0.iter().map(|p| (p[0], p[1])).collect()
    }

    /// A cut layer's LEF58_MAXSPACING rules as io `setCutLayerProperties` keeps them, in the
    /// database's order: one with no CUTCLASS is skipped (io warns, DRT-350: [`max_spacing_skips`]),
    /// one whose class the layer does not define is dropped silently (`getCutClassIdx(name)` is
    /// -1; the name map keeps a class's LAST index).
    pub fn max_spacing_rules(db: &Db, layer: &str, classes: &[CutClass]) -> Vec<MaxSpacingRule> {
        (0..db.num_layer_get_tech_layer_max_spacing_rules(layer))
            .filter(|&k| db.maxspacingrule_has_cut_class(layer, k))
            .filter_map(|k| {
                let name = db.maxspacingrule_get_cut_class(layer, k);
                let cut_class = classes.iter().rposition(|c| c.name == name)?;
                Some(MaxSpacingRule { max: db.maxspacingrule_get_max_spacing(layer, k), cut_class })
            })
            .collect()
    }

    /// io's DRT-350 warnings for a cut layer: one per LEF58_MAXSPACING rule with no CUTCLASS,
    /// which it skips.
    pub fn max_spacing_skips(db: &Db, layer: &str) -> Vec<(u16, String)> {
        (0..db.num_layer_get_tech_layer_max_spacing_rules(layer))
            .filter(|&k| !db.maxspacingrule_has_cut_class(layer, k))
            .map(|_| (350, format!("LEF58_MAXSPACING with no CUTCLASS is not supported. Skipping for layer {layer}")))
            .collect()
    }

    /// The LEF58_MINSTEP rules of routing layer `layer` io keeps, in the technology's order
    /// (`setRoutingLayerProperties`, after the end-of-line rules). Upstream rule: a rule whose
    /// MAXEDGES is above 1 is skipped with a warning ([`lef58_min_step_skip_warnings`]); the rest
    /// keep MINSTEP, MAXEDGES (-1 when not given), MINADJACENTLENGTH's first value (-1 when not
    /// given), the end-of-line width as NOADJACENTEOL's or as NOBETWEENEOL's (-1 for the other),
    /// and EXCEPTRECTANGLE.
    /// io's LEF58_WIDTHTABLE reading: only the ORTHOGONAL tables count; the first width of the last
    /// one without WRONGDIRECTION is the width, of the last one with it the wrong-way width (the
    /// width when there is none). Both 0: no constraint. A horizontal layer's horizontal spacing is
    /// the wrong-way width and its vertical spacing the width; a vertical layer's the other way
    /// round. (An empty table, which io would throw on, reads as 0.)
    pub fn width_tbl_orth(db: &Db, layer: &str, is_horz: bool) -> Option<WidthTblOrth> {
        let (mut width, mut wrong_way) = (0u32, 0u32);
        for k in 0..db.num_layer_get_tech_layer_width_table_rules(layer) {
            if !db.widthtablerule_is_orthogonal(layer, k) {
                continue;
            }
            // frUInt4: the database's int as io stores it.
            let first = db.widthtablerule_get_width_table(layer, k).first().copied().unwrap_or(0) as u32;
            if db.widthtablerule_is_wrong_direction(layer, k) {
                wrong_way = first;
            } else {
                width = first;
            }
        }
        width_tbl_orth_of(width, wrong_way, is_horz)
    }

    /// [`width_tbl_orth`]'s fold from the two widths (frCoord of frUInt4).
    pub fn width_tbl_orth_of(width: u32, wrong_way: u32, is_horz: bool) -> Option<WidthTblOrth> {
        let wrong_way = if wrong_way == 0 { width } else { wrong_way };
        if width == 0 && wrong_way == 0 {
            return None;
        }
        let (width, wrong_way) = (width as i32, wrong_way as i32);
        Some(if is_horz {
            WidthTblOrth { horz_spc: wrong_way, vert_spc: width }
        } else {
            WidthTblOrth { horz_spc: width, vert_spc: wrong_way }
        })
    }

    pub fn lef58_min_step_rules(db: &Db, layer: &str) -> Vec<Lef58MinStep> {
        (0..db.num_layer_get_tech_layer_min_step_rules(layer))
            .filter(|&k| !lef58_min_step_skipped(db, layer, k))
            .map(|k| {
                let eol = db.minsteprule_get_eol_width(layer, k);
                let max_edges =
                    i32::try_from(db.minsteprule_get_max_edges(layer, k)).unwrap_or(i32::MAX);
                Lef58MinStep {
                    min_step_length: db.minsteprule_get_min_step_length(layer, k),
                    max_edges: if db.minsteprule_is_max_edges_valid(layer, k) {
                        max_edges
                    } else {
                        -1
                    },
                    min_adjacent_length: if db.minsteprule_is_min_adj_length1_valid(layer, k) {
                        db.minsteprule_get_min_adj_length1(layer, k)
                    } else {
                        -1
                    },
                    no_adj_eol: if db.minsteprule_is_no_adjacent_eol(layer, k) { eol } else { -1 },
                    eol_width: if db.minsteprule_is_no_between_eol(layer, k) { eol } else { -1 },
                    except_rectangle: db.minsteprule_is_except_rectangle(layer, k),
                }
            })
            .collect()
    }

    /// Whether io skips LEF58_MINSTEP rule `k` of `layer`. Upstream rule: MAXEDGES above 1, read
    /// whether or not the rule gives MAXEDGES (the database's 0 when it does not).
    pub fn lef58_min_step_skipped(db: &Db, layer: &str, k: usize) -> bool {
        db.minsteprule_get_max_edges(layer, k) > 1
    }

    /// The first clause of LEF58_MINSTEP rule `k` of `layer` the reference's check does not read
    /// (NOADJACENTEOL and EXCEPTRECTANGLE: upstream OpenROAD issue #12229, register D14)
    /// as the LEF defines it, so this engine REFUSES it (a rule io skips is not one of these):
    /// NOADJACENTEOL (alone it is never checked; with MINADJACENTLENGTH the length is read as one
    /// more violation, where the LEF makes it a condition on the other neighbour edge), EXCEPTRECTANGLE
    /// with NOBETWEENEOL (that check never reads it), MINBETWEENLENGTH, a second MINADJACENTLENGTH
    /// value or its CONVEXCORNER / CONCAVECORNER (the check reads only the first length and ignores
    /// the corner), or a rule with neither NOBETWEENEOL nor MINADJACENTLENGTH (kept by io and never
    /// checked). The forms the foundry kits use (MAXEDGES 1 with MINADJACENTLENGTH or NOBETWEENEOL)
    /// are none of these.
    pub fn lef58_min_step_unmodelled(db: &Db, layer: &str, k: usize) -> Option<&'static str> {
        if lef58_min_step_skipped(db, layer, k) {
            None
        } else if db.minsteprule_is_no_adjacent_eol(layer, k) {
            Some("NOADJACENTEOL")
        } else if db.minsteprule_is_except_rectangle(layer, k) && db.minsteprule_is_no_between_eol(layer, k) {
            Some("EXCEPTRECTANGLE with NOBETWEENEOL")
        } else if db.minsteprule_is_min_between_length_valid(layer, k) {
            Some("MINBETWEENLENGTH")
        } else if db.minsteprule_is_min_adj_length2_valid(layer, k) {
            Some("MINADJACENTLENGTH with a second length")
        } else if db.minsteprule_is_convex_corner(layer, k) {
            Some("MINADJACENTLENGTH CONVEXCORNER")
        } else if db.minsteprule_is_concave_corner(layer, k) {
            Some("MINADJACENTLENGTH CONCAVECORNER")
        } else if !db.minsteprule_is_no_between_eol(layer, k)
            && !db.minsteprule_is_min_adj_length1_valid(layer, k)
        {
            Some("neither NOBETWEENEOL nor MINADJACENTLENGTH (never checked)")
        } else {
            None
        }
    }

    /// io's warnings for the LEF58_MINSTEP rules of routing layer `layer` it skips, in the
    /// rules' order: DRT-335 with the MAXEDGES value (the text as the reference words it).
    pub fn lef58_min_step_skip_warnings(db: &Db, layer: &str) -> Vec<(u16, String)> {
        (0..db.num_layer_get_tech_layer_min_step_rules(layer))
            .filter(|&k| lef58_min_step_skipped(db, layer, k))
            .map(|k| {
                let n = db.minsteprule_get_max_edges(layer, k);
                (335, format!("LEF58_MINSTEP MAXEDGES {n}  is not supported"))
            })
            .collect()
    }

    /// The clause that makes a LEF58_AREA rule SKIPPED with a warning: io
    /// `setRoutingLayerProperties`, its LEF58_AREA loop, warns and skips (DRT-311..316, in this
    /// order) a rule with EXCEPTMINWIDTH, EXCEPTEDGELENGTH (one value or two), EXCEPTMINSIZE,
    /// EXCEPTSTEP, MASK or LAYER (a trim layer; OVERLAP comes only with it). The reference routes
    /// without the rule; so does this router, but it says so.
    pub fn lef58_area_skipped_clause(db: &Db, layer: &str, k: usize) -> Option<&'static str> {
        if db.arearule_get_except_min_width(layer, k) != 0 {
            Some("EXCEPTMINWIDTH")
        } else if db.arearule_get_except_edge_length(layer, k) != 0 || (db.arearule_get_except_edge_lengths_first(layer, k), db.arearule_get_except_edge_lengths_second(layer, k)) != (0, 0) {
            Some("EXCEPTEDGELENGTH")
        } else if (db.arearule_get_except_min_size_first(layer, k), db.arearule_get_except_min_size_second(layer, k)) != (0, 0) {
            Some("EXCEPTMINSIZE")
        } else if (db.arearule_get_except_step_first(layer, k), db.arearule_get_except_step_second(layer, k)) != (0, 0) {
            Some("EXCEPTSTEP")
        } else if db.arearule_get_mask(layer, k) != 0 {
            Some("MASK")
        } else if !db.arearule_get_trim_layer(layer, k).is_empty() {
            Some("LAYER")
        } else {
            None
        }
    }

    /// io's LEF58_AREA loop for a routing layer: each rule not skipped
    /// ([`lef58_area_skipped_clause`]), in the technology's order, into its list
    /// ([`Lef58Area::add`]).
    pub fn lef58_area_rules(db: &Db, layer: &str) -> Lef58Area {
        let mut out = Lef58Area::default();
        for k in 0..db.num_layer_get_tech_layer_area_rules(layer) {
            if lef58_area_skipped_clause(db, layer, k).is_some() {
                continue;
            }
            out.add(AreaRule { index: k as u16, area: db.arearule_get_area(layer, k), rect_width: db.arearule_get_rect_width(layer, k), except_rectangle: db.arearule_is_except_rectangle(layer, k) });
        }
        out
    }

    /// io `convertLef58MinCutConstraints` (postProcess, after `initDefaultVias`): per routing
    /// layer, bottom to top, each kept LEF58 rule ([`lef58_min_cut_kept`]) asking for two or more
    /// cuts becomes a minimum-cut rule appended after the v5.4 ones: NUMCUTS, WIDTH, WITHIN when
    /// given, FROMABOVE or FROMBELOW, LENGTH … WITHIN (which also raises the layer's MTSAFEDIST
    /// contribution).
    ///
    /// A rule written per cut class (`MINIMUMCUT CUTCLASS c n …`) takes its count from the default
    /// vias (`vias`: the via defs and each layer's default, [`crate::dr::rules::default_vias`]):
    /// per (class, count) in the database's map order (by class name), the default via of the cut
    /// layer below when the rule is not FROMABOVE, then the one above when it is not FROMBELOW,
    /// the first whose cut class is that class giving the count; none matching drops the rule.
    /// Without `vias` (the technology as read, before the default vias exist) such a rule is left
    /// out until [`resolve_lef58_min_cuts`] runs.
    ///
    /// ⛔ A DIVERGENCE from the reference by decision (drt divergence register D9; upstream
    /// OpenROAD issue #12176). Upstream rule: the database's LEF reader fills the class map but
    /// never sets the rule's per-cut-class flag, so io reads the count 0 and drops the rule. Our
    /// rule, the reference patched with vyges-openroad-patches upstream-fix 0003, which sets the
    /// flag (read here from the class map itself, so a database written by either reader gives
    /// the same rules). ⛔ Refused (`Err`) where the patched reference would dereference a missing
    /// cut class: the default via looked at has none (a via generated by `initDefaultVias`, or one
    /// whose cut matches no class of its layer).
    pub fn convert_lef58_min_cut_constraints(db: &Db, layers: &mut [Layer], vias: Option<(&[ViaDef], &[Option<usize>])>) -> Result<(), String> {
        // `frViaDef::getCutClass()->getName()` of each layer's default via, read before the
        // layers are borrowed mutably; `None` for a via with no class.
        let class_of: Vec<Option<(String, Option<String>)>> = match vias {
            None => Vec::new(),
            Some((via_defs, defaults)) => (0..layers.len())
                .map(|l| {
                    let v = defaults.get(l).copied().flatten()?;
                    let vd = &via_defs[v];
                    Some((vd.name.clone(), vd.cut_class.map(|c| layers[vd.cut].cut_classes[c].name.clone())))
                })
                .collect(),
        };
        let num_layers = layers.len();
        for (li, l) in layers.iter_mut().enumerate().filter(|(_, l)| l.kind == LayerKind::Routing) {
            for k in (0..db.num_layer_get_tech_layer_min_cut_rules(&l.name)).filter(|&k| lef58_min_cut_kept(db, &l.name, k)) {
                let classes = db.mincutrule_cut_class_cuts(&l.name, k);
                let num_cuts = if classes.is_empty() {
                    db.mincutrule_get_num_cuts(&l.name, k)
                } else {
                    if vias.is_none() {
                        continue;
                    }
                    let below = if li > 0 { class_of.get(li - 1).cloned().flatten() } else { None };
                    let above = if li + 1 < num_layers { class_of.get(li + 1).cloned().flatten() } else { None };
                    let from_above = db.mincutrule_is_from_above(&l.name, k);
                    let from_below = db.mincutrule_is_from_below(&l.name, k);
                    let named = |via: &(String, Option<String>)| {
                        via.1.clone().ok_or_else(|| format!("{}: LEF58_MINIMUMCUT rule {k} is written per cut class and the default via {} has no cut class (the reference fixed for OpenROAD issue #12176 dereferences it; not modelled)", l.name, via.0))
                    };
                    let mut found = None;
                    for (class, n) in &classes {
                        if let Some(v) = below.as_ref().filter(|_| !from_above) {
                            if named(v)? == *class {
                                found = Some(*n);
                                break;
                            }
                        }
                        if let Some(v) = above.as_ref().filter(|_| !from_below) {
                            if named(v)? == *class {
                                found = Some(*n);
                                break;
                            }
                        }
                    }
                    let Some(n) = found else { continue };
                    n
                };
                if let Some(m) = lef58_min_cut(db, &l.name, k, num_cuts) {
                    if let Some((_, within)) = m.length {
                        l.mt_safe_dist = l.mt_safe_dist.max(within);
                    }
                    l.min_cuts.push(m);
                }
            }
        }
        Ok(())
    }

    /// The minimum-cut rules once the default vias exist (`defaults`, per layer, into
    /// `tech.via_defs`): each routing layer's v5.4 rules, then every LEF58 rule, per-cut-class ones
    /// included ([`convert_lef58_min_cut_constraints`]), with MTSAFEDIST recomputed. `Err` names a
    /// per-cut-class rule whose default via has no cut class.
    pub fn resolve_lef58_min_cuts(db: &Db, tech: &mut Tech, defaults: &[Option<usize>]) -> Result<(), String> {
        for l in tech.layers.iter_mut().filter(|l| l.kind == LayerKind::Routing) {
            l.min_cuts = v54_min_cuts(db, &l.name);
            l.mt_safe_dist = 0;
        }
        let vias = tech.via_defs.clone();
        convert_lef58_min_cut_constraints(db, &mut tech.layers, Some((&vias, defaults)))
    }

    /// One LEF58 rule as io converts it, asking for `num_cuts` (`None` for a single cut or none).
    fn lef58_min_cut(db: &Db, layer: &str, k: usize, num_cuts: i32) -> Option<MinCut> {
        if !MinCut::applies(num_cuts) {
            return None;
        }
        let mut from_above = None;
        if db.mincutrule_is_from_above(layer, k) {
            from_above = Some(true);
        }
        if db.mincutrule_is_from_below(layer, k) {
            from_above = Some(false);
        }
        Some(MinCut {
            num_cuts,
            width: db.mincutrule_get_width(layer, k),
            within: db.mincutrule_is_within_cut_dist_valid(layer, k).then(|| db.mincutrule_get_within_cut_dist(layer, k)),
            from_above,
            length: db.mincutrule_is_length_valid(layer, k).then(|| (db.mincutrule_get_length(layer, k), db.mincutrule_get_length_within_dist(layer, k))),
        })
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

    /// A master's obstructions with their own DESIGNRULEWIDTH / SPACING (`Some` when either is set),
    /// by layer number (the technology's layers only; the rules paired in box order first).
    pub fn master_obstructions_with_rules(db: &Db, tech: &Tech, master: &str) -> Res<Vec<Obstruction>> {
        let rules = db.master_obstruction_rules(master)?;
        Ok(db
            .master_obstruction_boxes(master)?
            .into_iter()
            .enumerate()
            .filter_map(|(k, (n, x0, y0, x1, y1))| {
                let rule = rules.get(k).copied().filter(|&(w, sp)| w != -1 || sp != -1);
                tech.layer_num(&db.layer_name_by_number(n)).map(|l| (l, Rect::new(x0, y0, x1, y1), rule))
            })
            .collect())
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

    /// Rule (upstream 263dba46d0, `io::Parser::addRoutingLayer`): a MINIMUMCUT rule asking for a
    /// single cut is not imported; two or more is.
    #[test]
    fn a_single_cut_minimum_cut_rule_is_not_imported() {
        assert!(!MinCut::applies(0));
        assert!(!MinCut::applies(1));
        assert!(MinCut::applies(2));
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

    /// Rule (`io::Parser` master import): an obstruction with its own DESIGNRULEWIDTH or SPACING is
    /// a blockage of its own — unmerged, in obstruction order, BEFORE the merged ones, with its rule;
    /// the others merge (two overlapping here: one maximal rectangle each way).
    #[test]
    fn an_obstruction_with_its_own_rule_stays_its_own_blockage_first() {
        let t = crate::gc::tests::tech();
        let own = Rect::new(500, 0, 600, 100);
        let m = Master::import_with_rules(&t, Vec::new(), &[(4, Rect::new(0, 0, 300, 100), None), (4, own, Some((200, -1))), (4, Rect::new(0, 0, 100, 300), None)]);
        assert_eq!(m.blockages[0], (4, own));
        assert_eq!(m.blockage_rules, vec![Some((200, -1)), None, None]);
        assert_eq!(m.blockages.len(), 3);
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

#[cfg(test)]
mod lef58_area_tests {
    //! LEF58_AREA on a routing layer: io's two lists (`frLayer`) and the rule a polygon breaks
    //! (`checkMetalShape_lef58Area`).
    use super::*;

    fn rule(index: u16, area: i64, rect_width: i32, except_rectangle: bool) -> AreaRule {
        AreaRule { index, area, rect_width, except_rectangle }
    }

    fn lists(rules: &[AreaRule]) -> Lef58Area {
        let mut l = Lef58Area::default();
        for &r in rules {
            l.add(r);
        }
        l
    }

    /// Rule (reference `frLayer::addLef58AreaConstraintRectWidth` / `addLef58AreaConstraint`): a
    /// RECTWIDTH rule goes to the first list, ascending by (RECTWIDTH, AREA); any other to the
    /// second, descending by AREA; each inserted at `lower_bound`, so a rule tying earlier ones
    /// goes BEFORE them (witness: two AREA 0.02 rules, the later one is tried first).
    #[test]
    fn io_inserts_each_rule_at_the_lower_bound_of_its_list() {
        let l = lists(&[rule(0, 20_000, 0, false), rule(1, 10_000, 50, false), rule(2, 20_000, 0, true), rule(3, 5_000, 50, false), rule(4, 40_000, 0, false), rule(5, 8_000, 40, false), rule(6, 10_000, 50, true), rule(7, 10_000, 0, false)]);
        let idx = |v: &[AreaRule]| v.iter().map(|r| r.index).collect::<Vec<_>>();
        assert_eq!(idx(&l.rect_width), [5, 3, 6, 1], "(40, 8000) (50, 5000) then the tie (50, 10000): the later first");
        assert_eq!(idx(&l.area), [4, 2, 0, 7], "40000, then the tie at 20000: the later first, then 10000");
    }

    const R300: Rect = Rect { xl: 980, yl: 980, xh: 1280, yh: 1020 };

    /// Rule (reference `checkMetalShape_lef58Area`): a rectangle (four vertices) first tries the
    /// RECTWIDTH rules in order; the first whose RECTWIDTH reaches the rectangle's narrower side
    /// (the polygon ONE maximal rectangle, `checkMetalShape_lef58Area_rectWidth`) decides alone —
    /// broken below its AREA, and no other rule is tried even when it would break. A RECTWIDTH
    /// rule never judges a polygon that is not a rectangle (witness `w`, `wa`).
    #[test]
    fn a_rect_width_rule_decides_a_narrow_rectangle_alone() {
        let w = lists(&[rule(0, 20_000, 50, false)]);
        assert_eq!(w.violated(true, &[R300], 12_000), Some(rule(0, 20_000, 50, false)));
        assert_eq!(w.violated(true, &[R300], 24_000), None);
        assert_eq!(w.violated(true, &[Rect::new(980, 980, 1180, 1060)], 16_000), None, "80 wide: the rule does not apply, no other rule");
        assert_eq!(w.violated(false, &[R300, Rect::new(1240, 980, 1280, 1180)], 18_400), None, "not a rectangle");
        assert_eq!(w.violated(true, &[R300, R300], 12_000), None, "four vertices but two maximal rectangles (a ring): not one");
        let at = lists(&[rule(0, 20_000, 40, false)]);
        assert!(at.violated(true, &[R300], 12_000).is_some(), "a side EQUAL to RECTWIDTH applies");
        let wa = lists(&[rule(0, 10_000, 50, false), rule(1, 30_000, 0, false)]);
        assert_eq!(wa.violated(true, &[R300], 12_000), None, "decided by the RECTWIDTH rule: the 0.03 rule is not tried");
        assert_eq!(wa.violated(true, &[Rect::new(980, 980, 1180, 1060)], 16_000).map(|r| r.index), Some(1));
        assert_eq!(wa.violated(false, &[R300, Rect::new(1240, 980, 1280, 1180)], 18_400).map(|r| r.index), Some(1));
        let first = lists(&[rule(0, 20_000, 50, false), rule(1, 10_000, 60, false)]);
        assert_eq!(first.violated(true, &[R300], 15_000).map(|r| r.index), Some(0), "the first RECTWIDTH rule that applies, in list order");
        let narrowest = lists(&[rule(0, 30_000, 60, false), rule(1, 20_000, 50, false)]);
        assert_eq!(narrowest.violated(true, &[R300], 25_000), None, "the 50 rule sorts first and passes: the 60 rule, which 25000 breaks, is not tried");
    }

    /// Rule (reference `checkMetalShape_lef58Area`): the other rules by descending AREA — none
    /// breaks once the polygon reaches a rule's AREA (strictly below breaks); EXCEPTRECTANGLE
    /// passes a polygon that is ONE maximal rectangle on to the next rule
    /// (`checkMetalShape_lef58Area_exceptRectangle`); the first rule not passing it on is broken
    /// (witness `a`, `x`, `xa`, `t`).
    #[test]
    fn the_area_rules_go_by_descending_area_and_except_rectangle_passes_a_rectangle_on() {
        let ell = [R300, Rect::new(1240, 980, 1280, 1180)];
        let a = lists(&[rule(0, 20_000, 0, false)]);
        assert!(a.violated(true, &[R300], 19_999).is_some() && a.violated(true, &[R300], 20_000).is_none());
        let x = lists(&[rule(0, 20_000, 0, true)]);
        assert_eq!(x.violated(true, &[R300], 12_000), None);
        assert_eq!(x.violated(false, &ell, 18_400).map(|r| r.index), Some(0));
        let xa = lists(&[rule(0, 30_000, 0, true), rule(1, 15_000, 0, false)]);
        assert_eq!(xa.violated(true, &[R300], 12_000).map(|r| r.index), Some(1));
        assert_eq!(xa.violated(true, &[R300], 24_000), None, "at or above the next rule: stop");
        assert_eq!(xa.violated(false, &ell, 18_400).map(|r| r.index), Some(0));
        let t = lists(&[rule(0, 20_000, 0, false), rule(1, 20_000, 0, true)]);
        assert_eq!(t.violated(true, &[R300], 12_000).map(|r| r.index), Some(0), "the later (EXCEPTRECTANGLE) rule is tried first and passes it on");
        assert_eq!(t.violated(false, &ell, 18_400).map(|r| r.index), Some(1));
        let stop = lists(&[rule(0, 30_000, 0, true), rule(1, 20_000, 0, false)]);
        assert_eq!(stop.violated(true, &[R300], 25_000), None, "a passed-on rectangle is still judged by the next AREA");
        assert_eq!(Lef58Area::default().violated(true, &[R300], 0), None);
    }
}

#[cfg(test)]
mod two_widths_tests {
    //! The TWOWIDTHS spacing table (`frSpacingTableTwConstraint`, its lookup odb's
    //! `findTwSpacing`).
    use super::*;

    fn t() -> TwoWidthsTable {
        TwoWidthsTable {
            widths: vec![0, 100, 300],
            prls: vec![-1, 200, 400],
            values: vec![vec![40, 60, 100], vec![60, 80, 120], vec![100, 125, 160]],
        }
    }

    // Rule (odb `getTwIdx`): a width picks the row BEFORE the first row width not below it — a
    // width equal to a row's is the row before, one past the last is the last; then rows are
    // walked back while their PRL exceeds the run, row 0 when none takes it; a row without PRL
    // (-1) takes any run.
    #[test]
    fn a_width_picks_the_row_before_then_walks_back_by_prl() {
        let t = t();
        assert_eq!(t.tw_idx(0, 0), 0);
        assert_eq!(t.tw_idx(100, 1000), 0, "equal to row 1's width: row 0");
        assert_eq!(t.tw_idx(101, 1000), 1);
        assert_eq!(t.tw_idx(301, 1000), 2);
        assert_eq!(t.tw_idx(301, 399), 1, "row 2 needs a run of 400");
        assert_eq!(t.tw_idx(301, 400), 2, "a run equal to the row's PRL takes it");
        assert_eq!(t.tw_idx(301, 199), 0, "row 1 needs 200; row 0 has none");
        let no_prl_row0 = TwoWidthsTable { prls: vec![50, 200, 400], ..t.clone() };
        assert_eq!(no_prl_row0.tw_idx(101, 10), 0, "no row takes the run: row 0");
    }

    // Rule (`findTwSpacing`): the ROW by the first width and the COLUMN by the second, each at the
    // run clamped at 0 — the table need not be symmetric and the order of the widths is kept.
    #[test]
    fn the_row_is_the_first_width_and_the_column_the_second() {
        let t = t();
        assert_eq!(t.find(500, 150, 1000), 125);
        assert_eq!(t.find(150, 500, 1000), 120);
        assert_eq!(t.find(500, 500, -7), 40, "a negative run is 0: rows 1 and 2 are out");
        // The clamp shows where a row after the first has no PRL: a run of -7 is 0, which a
        // PRL of -1 takes (row 1); unclamped it would not (row 0).
        let t = TwoWidthsTable { prls: vec![-1, -1, 400], ..t };
        assert_eq!(t.find(150, 150, -7), 80);
    }

    // Rule (every consumer: GC `checkMetalSpacing_prl_getReqSpcVal`, `frLayer::getMinSpacingValue`,
    // the RP prep): the parallel-run table is looked up on the WIDER width, the TWOWIDTHS table
    // on both; `findMin` is the first row's first value, `findMax` the last row's last.
    #[test]
    fn min_spacing_dispatches_on_the_rule_kind() {
        let tw = MinSpacing::TwoWidths(t());
        assert_eq!((tw.find(500, 150, 1000), tw.find(150, 500, 1000)), (125, 120));
        assert_eq!((tw.find_min(), tw.find_max()), (40, 160));
        let prl: MinSpacing = SpacingTable { widths: vec![0, 100], prls: vec![0], values: vec![vec![40], vec![90]] }.into();
        assert_eq!((prl.find(150, 50, 0), prl.find(50, 150, 0)), (90, 90));
        assert_eq!((prl.find_min(), prl.find_max()), (40, 90));
    }
}
