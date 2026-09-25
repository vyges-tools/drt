// SPDX-License-Identifier: Apache-2.0
//! Track assignment: each route guide's wire placed on one track of its gcell, cost-driven.
//!
//! Stages, in order ([`track_assignment`]): an initial pass, then one repair pass. Each pass runs
//! PANELS of 50 gcells across the die — vertical panels first (horizontal first only when layer
//! 0 is itself a horizontal routing layer) — in batches of 8; a panel ([`Worker`]):
//! - [`Worker::init_tracks`]: the preferred tracks of its direction's layers inside it (skipping
//!   ones whose via would leave the die);
//! - [`Worker::init_fixed_objs`]: costs from the fixed shapes (pins, special-net wiring,
//!   blockages), per track, bloated by spacing;
//! - [`Worker::init_iroutes`]: one "iroute" per guide of its layers touching the panel, IN THE
//!   GUIDE TREE'S QUERY ORDER (which numbers them — and numbers break ties): its span along the
//!   track, its vias to neighbouring guides, its pin coordinate; guides reaching outside the panel
//!   are fixed context;
//! - [`Worker::init_costs`], [`Worker::sort_iroutes`], [`Worker::assign`]: iroutes taken highest
//!   cost first (lowest number on a tie), each put on its best track, its neighbours' costs
//!   updated and the ones now in conflict queued again (once);
//! - after the batch, [`Worker::save_to_guides`]: each iroute's wire becomes its guide's route.
//!
//! Rules (the costs, per candidate track):
//! - DRC: the overlap of the iroute's wire and vias with other nets' cost boxes (at least two
//!   pitches; two pitches for a via); scaled by 0.05 in the initial pass, 32 in the repair pass
//!   (where it alone decides);
//! - next-iroute direction: distance to the gcell side the neighbouring guides lie on;
//! - pin: distance to the pin's track coordinate (plus 4 pitches when nonzero);
//! - alignment: minus (4 pitches + 1) when the track already carries the same net.

use std::collections::BTreeSet;

use crate::dr::guides::GCellGrid;
use crate::dr::rules::{EolTable, LayerTables, NdrRule};
use crate::polygon90::Rect;
use crate::rtree::PackedRTree;
use crate::tech::{LayerKind, Tech, TrackPattern};

type P = (i32, i32);

/// The router's track-assignment settings.
#[derive(Debug, Clone, Copy)]
pub struct TaConfig {
    pub bottom_routing_layer: usize,
    pub top_routing_layer: usize,
    pub shape_bloat_width: f32,
    pub drc_cost: u32,
    pub pin_cost: u32,
    pub align_cost: u32,
    pub batch_size: usize,
    pub use_min_spacing_obs: bool,
    pub use_nonpref_tracks: bool,
}

impl Default for TaConfig {
    fn default() -> TaConfig {
        TaConfig { bottom_routing_layer: 2, top_routing_layer: usize::MAX, shape_bloat_width: 1.5, drc_cost: 32, pin_cost: 4, align_cost: 4, batch_size: 8, use_min_spacing_obs: true, use_nonpref_tracks: true }
    }
}

/// A fixed shape the costs see.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Fixed {
    /// An instance terminal's pin shape: its net, the instance and the master terminal.
    InstTerm { net: Option<usize>, inst: usize, term: usize },
    /// A block pin's shape: its net and the port.
    BTerm { net: Option<usize>, port: usize },
    /// A special net's wire; `supply`: a power or ground net.
    Seg { supply: bool },
    /// A special net's via shape (on any of its layers).
    Via { supply: bool },
    /// A design blockage.
    Blockage,
    /// An instance obstruction; `big`: the master is a block, pad or ring.
    InstBlockage { big: bool },
}

impl Fixed {
    /// The net of a terminal's shape.
    pub fn term_net(&self) -> Option<Option<usize>> {
        match *self {
            Fixed::InstTerm { net, .. } | Fixed::BTerm { net, .. } => Some(net),
            _ => None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct TaNet {
    pub name: String,
    pub is_clock: bool,
    pub ndr: Option<NdrRule>,
}

/// A guide: gcell-centre points on a layer, and (after assignment) its wire.
#[derive(Debug, Clone)]
pub struct TaGuide {
    pub net: usize,
    pub layer: usize,
    pub begin: P,
    pub end: P,
    pub route: Option<(P, P)>,
}

impl TaGuide {
    fn bbox(&self) -> Rect {
        Rect { xl: self.begin.0.min(self.end.0), yl: self.begin.1.min(self.end.1), xh: self.begin.0.max(self.end.0), yh: self.begin.1.max(self.end.1) }
    }
}

/// One instance pin: whether it has access at all, the instance's chosen point, and the first
/// point of the master's FIRST class (design coordinates of THIS instance).
pub type InstPin = (bool, Option<(P, usize)>, Option<(P, usize)>);

/// A terminal a gr pin names: its net and, per pin, its access points.
#[derive(Debug, Clone)]
pub enum TaTerm {
    Inst { net: Option<usize>, pins: Vec<InstPin> },
    /// Per pin: whether it has access, and its points.
    Port { net: Option<usize>, pins: Vec<(bool, Vec<(P, usize)>)> },
}

pub struct TaInput<'a> {
    pub tech: &'a Tech,
    /// Default via per layer (cut layers).
    pub defaults: &'a [Option<usize>],
    /// The router's end-of-line rule, per routing-layer index.
    pub eol: &'a [EolTable],
    /// The rule tables per routing-layer index (the via-to-via forbidden lengths).
    pub tables: &'a [LayerTables],
    pub grid: &'a GCellGrid,
    pub die: Rect,
    pub tracks: &'a [TrackPattern],
    pub nets: &'a [TaNet],
    /// Per layer: fixed shapes, in the order the region is built (instances — each terminal's
    /// pin shapes, then its obstructions —, block pins, special nets — wires, then vias —,
    /// blockages); queried in the tree's order.
    pub fixed: &'a [Vec<(Rect, Fixed)>],
    pub terms: &'a [TaTerm],
    /// Gr pins, in order: (terminal, point).
    pub gr_pins: &'a [(usize, P)],
    pub cfg: TaConfig,
}

/// What track assignment did, in order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TaEvent {
    Worker { iter: usize, horizontal: bool, batch: usize, index: usize, route_box: Rect },
    Tracks { layer: usize, count: usize, first: i32, last: i32 },
    Fixed { layer: usize, bx: Rect, bloat: i32, net: Option<usize>, via: bool },
    Iroute { id: usize, ext: bool, guide: usize, begin: P, end: P, next_dir: i32, pin: Option<i32>, cost: u32, vias: Vec<(usize, P)> },
    Assign { id: usize, hard: bool, idx1: i32, idx2: i32, track: i32, drc: u32 },
    Try { track: i32, cost: u32, drc: u32 },
    Requeue { id: usize, drc: u32, queued: bool },
    Save { guide: usize, begin: P, end: P },
    EndWorker,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Fig {
    Seg { begin: P, end: P, layer: usize, width: i32, ext: i32 },
    Via { via: usize, origin: P },
}

#[derive(Debug, Clone)]
struct Iroute {
    guide: usize,
    figs: Vec<Fig>,
    next_dir: i32,
    pin: Option<i32>,
    cost: u32,
    assigned: u32,
}

/// Whose cost a box is: none (a blockage), a net's (fixed), or an iroute figure's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Owner {
    None,
    Net(usize),
    Fig(usize, usize),
}

/// Which rule made a cost box (its identity matters for removal).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Con {
    Short(usize),
    MinSpacing(usize),
    Cut(usize),
}

type CostBox = (Rect, Owner, Con);

/// Entries that each lie on one track (a line, or a point), kept per track coordinate across the
/// panel's direction; a query visits only the tracks it spans. Order within is irrelevant here
/// (costs are summed, pins collected into ordered sets).
#[derive(Debug, Clone, Default)]
struct TrackIndex<T> {
    horizontal: bool,
    by: std::collections::BTreeMap<i32, Vec<(Rect, T)>>,
}

impl<T: Copy + PartialEq> TrackIndex<T> {
    fn new(horizontal: bool) -> TrackIndex<T> {
        TrackIndex { horizontal, by: std::collections::BTreeMap::new() }
    }
    fn key(&self, r: &Rect) -> i32 {
        debug_assert!(if self.horizontal { r.yl == r.yh } else { r.xl == r.xh }, "an entry off a track");
        if self.horizontal {
            r.yl
        } else {
            r.xl
        }
    }
    fn push(&mut self, r: Rect, v: T) {
        let k = self.key(&r);
        self.by.entry(k).or_default().push((r, v));
    }
    fn remove_one(&mut self, r: Rect, v: T) {
        let k = self.key(&r);
        if let Some(b) = self.by.get_mut(&k) {
            if let Some(i) = b.iter().position(|e| e.0 == r && e.1 == v) {
                b.swap_remove(i);
            }
        }
    }
    fn query<'a>(&'a self, q: &'a Rect) -> impl Iterator<Item = &'a (Rect, T)> + 'a {
        let (lo, hi) = if self.horizontal { (q.yl, q.yh) } else { (q.xl, q.xh) };
        self.by.range(lo..=hi).flat_map(|(_, b)| b.iter()).filter(move |e| touches(&e.0, q))
    }
}

fn touches(a: &Rect, b: &Rect) -> bool {
    a.xl <= b.xh && b.xl <= a.xh && a.yl <= b.yh && b.yl <= a.yh
}

fn bloat(r: &Rect, d: i32) -> Rect {
    Rect { xl: r.xl - d, yl: r.yl - d, xh: r.xh + d, yh: r.yh + d }
}

fn norm(a: P, b: P) -> Rect {
    Rect { xl: a.0.min(b.0), yl: a.1.min(b.1), xh: a.0.max(b.0), yh: a.1.max(b.1) }
}

fn min_side(r: &Rect) -> i32 {
    r.dx().min(r.dy())
}

fn max_side(r: &Rect) -> i32 {
    r.dx().max(r.dy())
}

fn shift(r: &Rect, p: P) -> Rect {
    Rect { xl: r.xl + p.0, yl: r.yl + p.1, xh: r.xh + p.0, yh: r.yh + p.1 }
}

/// The tracks within `lo..=hi` (both ends included), as a first and last index; the last below
/// the first when none.
pub fn track_range(tracks: &[i32], lo: i32, hi: i32) -> (i32, i32) {
    let i1 = tracks.partition_point(|&x| x < lo) as i32;
    let i2 = tracks.partition_point(|&x| x <= hi) as i32 - 1;
    (i1, i2)
}

/// The shared state of a run: guides (with their tree) and the gr pin tree.
pub struct TaState<'a> {
    pub input: TaInput<'a>,
    pub guides: Vec<TaGuide>,
    guide_trees: Vec<PackedRTree<usize>>,
    gr_tree: PackedRTree<usize>,
    fixed_trees: Vec<PackedRTree<Fixed>>,
}

impl<'a> TaState<'a> {
    /// Guides in net order (each net's in its committed order).
    pub fn new(input: TaInput<'a>, guides: Vec<TaGuide>) -> TaState<'a> {
        let n = input.tech.layers.len();
        let mut per_layer: Vec<Vec<(Rect, usize)>> = vec![Vec::new(); n];
        for (i, g) in guides.iter().enumerate() {
            per_layer[g.layer].push((g.bbox(), i));
        }
        let guide_trees = per_layer.into_iter().map(PackedRTree::new).collect();
        let gr_tree = PackedRTree::new(input.gr_pins.iter().enumerate().map(|(i, &(_, p))| (Rect { xl: p.0, yl: p.1, xh: p.0, yh: p.1 }, i)).collect());
        let fixed_trees = input.fixed.iter().map(|v| PackedRTree::new(v.clone())).collect();
        TaState { input, guides, guide_trees, gr_tree, fixed_trees }
    }

    fn query_guides(&self, q: &Rect, layer: usize) -> Vec<usize> {
        self.guide_trees.get(layer).map_or(Vec::new(), |t| t.query(q).into_iter().map(|v| v.1).collect())
    }

    fn query_fixed(&self, q: &Rect, layer: usize) -> Vec<(Rect, Fixed)> {
        self.fixed_trees.get(layer).map_or(Vec::new(), |t| t.query(q).into_iter().copied().collect())
    }

    fn query_gr_pins(&self, q: &Rect) -> Vec<usize> {
        self.gr_tree.query(q).into_iter().map(|v| self.input.gr_pins[v.1].0).collect()
    }
}

/// One panel's worker.
pub struct Worker<'s, 'a> {
    st: &'s TaState<'a>,
    route_box: Rect,
    ext_box: Rect,
    horizontal: bool,
    iter: usize,
    hard: bool,
    track_locs: Vec<Vec<i32>>,
    iroutes: Vec<Iroute>,
    /// Ids of the iroutes the panel assigns (the rest are fixed context).
    own: Vec<usize>,
    shapes: Vec<TrackIndex<(usize, usize)>>,
    route_costs: Vec<TrackIndex<(Owner, Con)>>,
    via_costs: Vec<TrackIndex<(Owner, Con)>>,
    reassign: BTreeSet<(std::cmp::Reverse<u32>, usize)>,
    pub trace: Option<Vec<TaEvent>>,
}

impl<'s, 'a> Worker<'s, 'a> {
    fn tech(&self) -> &'a Tech {
        self.st.input.tech
    }
    fn cfg(&self) -> &TaConfig {
        &self.st.input.cfg
    }
    fn is_init(&self) -> bool {
        self.iter == 0
    }
    fn emit(&mut self, e: TaEvent) {
        if let Some(t) = &mut self.trace {
            t.push(e);
        }
    }
    fn default_via(&self, cut: i64) -> Option<usize> {
        usize::try_from(cut).ok().and_then(|c| self.st.input.defaults.get(c)).copied().flatten()
    }
    fn is_routing(&self, l: i64) -> bool {
        usize::try_from(l).ok().is_some_and(|l| l < self.tech().layers.len() && self.tech().layers[l].kind == LayerKind::Routing)
    }
    fn layer_dir_matches(&self, l: usize) -> bool {
        self.tech().layers[l].is_horizontal() == self.horizontal
    }
    fn top_layer(&self) -> usize {
        self.tech().layers.len() - 1
    }
    fn net_of(&self, iroute: usize) -> usize {
        self.st.guides[self.iroutes[iroute].guide].net
    }
    fn ndr_of(&self, net: usize) -> Option<&'a NdrRule> {
        self.st.input.nets[net].ndr.as_ref()
    }

    /// The minimum spacing for two widths and a run (0 without a rule).
    fn min_spacing_value(&self, layer: usize, w1: i32, w2: i32, prl: i32) -> i32 {
        self.tech().layers[layer].spacing.as_ref().map_or(0, |t| t.find(w1.max(w2), prl))
    }

    fn get_track_idx(&self, lo: i32, hi: i32, layer: usize) -> (i32, i32) {
        track_range(&self.track_locs[layer], lo, hi)
    }

    // ---- init ----

    /// Whether a track at `pt` is unusable: only on a unidirectional layer (or without
    /// non-preferred tracks) — when neither default via at `pt`, the one above nor the one below,
    /// fits inside the die. No via above the top layer counts as fitting (an empty box at the
    /// origin); a missing default via as not fitting.
    fn out_of_die_via(&self, layer: usize, pt: P) -> bool {
        let tech = self.tech();
        if self.cfg().use_nonpref_tracks && !tech.layers[layer].is_unidirectional() {
            return false;
        }
        let die = self.st.input.die;
        let via_box = |cut: usize| -> Rect {
            match self.default_via(cut as i64) {
                Some(v) => {
                    let vd = &tech.via_defs[v];
                    let b = vd.layer1_bbox();
                    let b2 = vd.layer2_bbox();
                    shift(&Rect { xl: b.xl.min(b2.xl), yl: b.yl.min(b2.yl), xh: b.xh.max(b2.xh), yh: b.yh.max(b2.yh) }, pt)
                }
                None => bloat(&die, 1),
            }
        };
        let contains = |r: &Rect| die.xl <= r.xl && die.yl <= r.yl && r.xh <= die.xh && r.yh <= die.yh;
        let up = if layer < self.top_layer() { via_box(layer + 1) } else { Rect { xl: 0, yl: 0, xh: 0, yh: 0 } };
        let down = if layer >= 1 { via_box(layer - 1) } else { Rect { xl: 0, yl: 0, xh: 0, yh: 0 } };
        !contains(&up) && !contains(&down)
    }

    pub fn init_tracks(&mut self) {
        let n = self.tech().layers.len();
        let mut sets: Vec<BTreeSet<i32>> = vec![BTreeSet::new(); n];
        let die = self.st.input.die;
        let center = ((die.xl + die.xh) / 2, (die.yl + die.yh) / 2);
        for (l, set) in sets.iter_mut().enumerate() {
            if self.tech().layers[l].kind != LayerKind::Routing || !self.layer_dir_matches(l) {
                continue;
            }
            for tp in self.st.input.tracks.iter().filter(|t| t.layer == l) {
                // A horizontal panel takes the horizontal tracks (y values).
                if tp.vertical_tracks == self.horizontal {
                    continue;
                }
                let (lo, hi) = if self.horizontal { (self.route_box.yl, self.route_box.yh) } else { (self.route_box.xl, self.route_box.xh) };
                let mut k = ((lo - tp.start) / tp.spacing).max(0);
                if k * tp.spacing + tp.start < lo {
                    k += 1;
                }
                while k < tp.num && k * tp.spacing + tp.start < hi {
                    let c = k * tp.spacing + tp.start;
                    let pt = if self.horizontal { (center.0, c) } else { (c, center.1) };
                    if !self.out_of_die_via(l, pt) {
                        set.insert(c);
                    }
                    k += 1;
                }
            }
        }
        self.track_locs = sets.into_iter().map(|s| s.into_iter().collect()).collect();
    }

    fn add_route_cost(&mut self, r: Rect, layer: usize, owner: Owner, con: Con) {
        self.route_costs[layer].push(r, (owner, con));
    }
    fn add_via_cost(&mut self, r: Rect, layer: usize, owner: Owner, con: Con) {
        self.via_costs[layer].push(r, (owner, con));
    }

    fn init_fixed_objs_helper(&mut self, bx: &Rect, bloat_dist: i32, layer: usize, net: Option<usize>, via: bool) {
        let b = bloat(bx, bloat_dist);
        self.emit(TaEvent::Fixed { layer, bx: *bx, bloat: bloat_dist, net, via });
        let (i1, i2) = if self.horizontal { self.get_track_idx(b.yl, b.yh, layer) } else { self.get_track_idx(b.xl, b.xh, layer) };
        let owner = net.map_or(Owner::None, Owner::Net);
        for i in i1..=i2 {
            let t = self.track_locs[layer][i as usize];
            let r = if self.horizontal { Rect { xl: b.xl, yl: t, xh: b.xh, yh: t } } else { Rect { xl: t, yl: b.yl, xh: t, yh: b.yh } };
            if via {
                self.add_via_cost(r, layer, owner, Con::Short(layer));
            } else {
                self.add_route_cost(r, layer, owner, Con::Short(layer));
            }
        }
    }

    fn calc_obs_bloat_dist_via(&self, via: usize, layer: usize, bx: &Rect, is_obs: bool) -> i32 {
        let tech = self.tech();
        let vd = &tech.via_defs[via];
        let vb = if vd.layer1 == layer { vd.layer1_bbox() } else { vd.layer2_bbox() };
        let (vw, vl) = (min_side(&vb), max_side(&vb));
        let mut obs_w = min_side(bx);
        if self.cfg().use_min_spacing_obs && is_obs {
            obs_w = tech.layers[layer].width;
        }
        let mut d = self.min_spacing_value(layer, obs_w, vw, vw);
        let eol = self.eol_of(layer);
        if min_side(&vb) < eol.width {
            d = d.max(eol.space);
        }
        d + if is_obs { vl / 2 } else { vw / 2 }
    }

    fn eol_of(&self, layer: usize) -> EolTable {
        let idx = (0..layer).filter(|&l| self.tech().layers[l].kind == LayerKind::Routing).count();
        self.st.input.eol.get(idx).copied().unwrap_or_default()
    }

    fn calc_bloat_dist(&self, is_blockage: bool, layer: usize, bx: &Rect) -> i32 {
        let l = &self.tech().layers[layer];
        let width = l.width;
        let mut obj_w = min_side(bx);
        let prl = if l.is_horizontal() { bx.dx() } else { bx.dy() };
        if is_blockage && self.cfg().use_min_spacing_obs {
            obj_w = width;
        }
        let mut d = width;
        if l.spacing.is_some() {
            d = self.min_spacing_value(layer, obj_w, width, prl);
        }
        d + width / 2
    }

    pub fn init_fixed_objs(&mut self) {
        let n = self.tech().layers.len();
        for layer in 0..n {
            if self.tech().layers[layer].kind != LayerKind::Routing || !self.layer_dir_matches(layer) {
                continue;
            }
            let width = self.tech().layers[layer].width;
            let objs = self.st.query_fixed(&self.ext_box, layer);
            for (bounds, obj) in &objs {
                let bx = bloat(bounds, -1);
                match *obj {
                    Fixed::InstTerm { net, .. } | Fixed::BTerm { net, .. } => {
                        let d = (self.cfg().shape_bloat_width * width as f32) as i32;
                        self.init_fixed_objs_helper(&bx, d, layer, net, false);
                    }
                    Fixed::Seg { .. } | Fixed::Via { .. } => {
                        // A special net's shape: never one of the routed nets.
                        let net = None;
                        let d = self.calc_bloat_dist(false, layer, bounds);
                        self.init_fixed_objs_helper(&bx, d, layer, net, false);
                        // Fat default vias below and above.
                        if layer >= 2 && self.is_routing(layer as i64 - 2) {
                            if let Some(v) = self.default_via(layer as i64 - 1) {
                                if min_side(&self.tech().via_defs[v].layer2_bbox()) > width {
                                    let d = self.calc_obs_bloat_dist_via(v, layer, bounds, false);
                                    self.init_fixed_objs_helper(&bx, d, layer, net, true);
                                }
                            }
                        }
                        if layer + 2 < n && self.is_routing(layer as i64 + 2) {
                            if let Some(v) = self.default_via(layer as i64 + 1) {
                                if min_side(&self.tech().via_defs[v].layer1_bbox()) > width {
                                    let d = self.calc_obs_bloat_dist_via(v, layer, bounds, false);
                                    self.init_fixed_objs_helper(&bx, d, layer, net, true);
                                }
                            }
                        }
                    }
                    Fixed::Blockage | Fixed::InstBlockage { .. } => {
                        let d = self.calc_bloat_dist(true, layer, bounds);
                        self.init_fixed_objs_helper(&bx, d, layer, None, false);
                    }
                }
            }
            // Big obstructions on the layers below and above block vias around them.
            for upper in [false, true] {
                let other = if upper { layer as i64 + 2 } else { layer as i64 - 2 };
                if !self.is_routing(other) {
                    continue;
                }
                let objs = self.st.query_fixed(&self.ext_box, other as usize);
                for (bounds, obj) in objs {
                    let Fixed::InstBlockage { big: true } = obj else { continue };
                    if min_side(&bounds) <= 2 * width {
                        continue;
                    }
                    let bx = bloat(&bounds, -1);
                    let cut = if upper { layer + 1 } else { layer - 1 };
                    let Some(v) = self.default_via(cut as i64) else { continue };
                    let d = self.calc_obs_bloat_dist_via(v, layer, &bounds, true);
                    let b = bloat(&bx, d);
                    for border in [Rect { xl: b.xl, yl: b.yl, xh: bx.xl, yh: b.yh }, Rect { xl: b.xl, yl: bx.yh, xh: b.xh, yh: b.yh }, Rect { xl: bx.xh, yl: b.yl, xh: b.xh, yh: b.yh }, Rect { xl: b.xl, yl: b.yl, xh: b.xh, yh: bx.yl }] {
                        self.init_fixed_objs_helper(&border, 0, layer, None, true);
                    }
                }
            }
        }
    }

    /// The chosen point of a gr pin's terminal on `layer` inside the panel, else none.
    fn pin_point(&self, term: usize, net: usize, layer: Option<usize>, fallback: bool) -> Option<P> {
        match &self.st.input.terms[term] {
            TaTerm::Inst { net: n, pins } => {
                if *n != Some(net) {
                    return None;
                }
                for (has, pref, first) in pins {
                    if !has {
                        continue;
                    }
                    let ap = match pref {
                        Some(a) => Some(*a),
                        None if fallback => *first,
                        None => None,
                    };
                    let Some((p, l)) = ap else { continue };
                    if layer.is_none_or(|x| x == l) && touches(&self.route_box, &Rect { xl: p.0, yl: p.1, xh: p.0, yh: p.1 }) {
                        return Some(p);
                    }
                }
                None
            }
            TaTerm::Port { net: n, pins } => {
                if *n != Some(net) {
                    return None;
                }
                for (has, aps) in pins {
                    if !has {
                        continue;
                    }
                    for &(p, l) in aps {
                        if layer.is_none_or(|x| x == l) && touches(&self.route_box, &Rect { xl: p.0, yl: p.1, xh: p.0, yh: p.1 }) {
                            return Some(p);
                        }
                    }
                }
                None
            }
        }
    }

    /// A pin guide (a single point): the pin's chosen point on this layer. `(begin, end, down,
    /// up, next dir, pin)`.
    #[allow(clippy::type_complexity)]
    fn init_iroute_helper_pin(&self, g: usize) -> Option<(i32, i32, BTreeSet<i32>, BTreeSet<i32>, i32, i32)> {
        let guide = &self.st.guides[g];
        if guide.begin != guide.end {
            return None;
        }
        let (net, layer, bp) = (guide.net, guide.layer, guide.begin);
        let pt = Rect { xl: bp.0, yl: bp.1, xh: bp.0, yh: bp.1 };
        let has = |l: i64| -> bool { usize::try_from(l).is_ok_and(|l| self.st.query_guides(&pt, l).iter().any(|&o| self.st.guides[o].net == net)) };
        let down = layer as i64 - 2 >= self.cfg().bottom_routing_layer as i64 && has(layer as i64 - 2);
        let up = layer + 2 < self.tech().layers.len() && has(layer as i64 + 2);
        for term in self.st.query_gr_pins(&pt) {
            if let Some(p) = self.pin_point(term, net, Some(layer), false) {
                let (along, across) = if self.horizontal { (p.0, p.1) } else { (p.1, p.0) };
                let (mut d, mut u) = (BTreeSet::new(), BTreeSet::new());
                if down {
                    d.insert(along);
                }
                if up {
                    u.insert(along);
                }
                return Some((along, along, d, u, 0, across));
            }
        }
        None
    }

    fn init_iroute_helper_generic_helper(&self, g: usize) -> Option<i32> {
        let guide = &self.st.guides[g];
        let mut terms = self.st.query_gr_pins(&Rect { xl: guide.begin.0, yl: guide.begin.1, xh: guide.begin.0, yh: guide.begin.1 });
        if guide.end != guide.begin {
            terms.extend(self.st.query_gr_pins(&Rect { xl: guide.end.0, yl: guide.end.1, xh: guide.end.0, yh: guide.end.1 }));
        }
        for term in terms {
            if let Some(p) = self.pin_point(term, guide.net, None, true) {
                return Some(if self.horizontal { p.1 } else { p.0 });
            }
        }
        None
    }

    #[allow(clippy::type_complexity)]
    fn init_iroute_helper_generic(&self, g: usize) -> (i32, i32, BTreeSet<i32>, BTreeSet<i32>, i32, Option<i32>) {
        let guide = &self.st.guides[g];
        let (net, layer) = (guide.net, guide.layer);
        let (mut min_begin, mut max_end) = (i32::MAX, i32::MIN);
        let (mut has_min, mut has_max) = (false, false);
        let mut next_dir = 0;
        let (mut down, mut up) = (BTreeSet::new(), BTreeSet::new());
        let along = |p: P| if self.horizontal { p.0 } else { p.1 };
        for i in 0..2 {
            let cp = if i == 0 { guide.begin } else { guide.end };
            let q = Rect { xl: cp.0, yl: cp.1, xh: cp.0, yh: cp.1 };
            let mut nbrs = Vec::new();
            if layer as i64 - 2 >= self.cfg().bottom_routing_layer as i64 {
                nbrs.extend(self.st.query_guides(&q, layer - 2));
            }
            if layer + 2 < self.tech().layers.len() {
                nbrs.extend(self.st.query_guides(&q, layer + 2));
            }
            for o in nbrs {
                let nb = &self.st.guides[o];
                if nb.net != net {
                    continue;
                }
                match nb.route {
                    None => {
                        if nb.layer + 2 == layer {
                            down.insert(along(nb.begin));
                        } else {
                            up.insert(along(nb.begin));
                        }
                    }
                    Some((sb, _)) => {
                        if i == 0 {
                            min_begin = min_begin.min(along(sb));
                            has_min = true;
                        } else {
                            max_end = max_end.max(along(sb));
                            has_max = true;
                        }
                        if nb.layer + 2 == layer {
                            down.insert(along(sb));
                        } else {
                            up.insert(along(sb));
                        }
                    }
                }
                if cp == nb.end {
                    next_dir -= 1;
                }
                if cp == nb.begin {
                    next_dir += 1;
                }
            }
        }
        if !has_min {
            min_begin = along(guide.begin);
        }
        if !has_max {
            max_end = along(guide.end);
        }
        if min_begin > max_end {
            std::mem::swap(&mut min_begin, &mut max_end);
        }
        if min_begin == max_end {
            max_end += 1;
        }
        (min_begin, max_end, down, up, next_dir, self.init_iroute_helper_generic_helper(g))
    }

    fn seg_style(&self, net: usize, layer: usize) -> (i32, i32) {
        let w = self.tech().layers[layer].width;
        let width = self.ndr_of(net).map_or(w, |n| w.max(n.widths.get(layer / 2 - 1).copied().unwrap_or(0)));
        (width, w / 2)
    }

    fn init_iroute(&mut self, g: usize) {
        let guide = self.st.guides[g].clone();
        let gb = guide.bbox();
        let ext = !(self.route_box.xl <= gb.xl && self.route_box.yl <= gb.yl && gb.xh <= self.route_box.xh && gb.yh <= self.route_box.yh);
        if ext && guide.route.is_none() {
            return;
        }
        let (begin, end, down, up, next_dir, pin) = match self.init_iroute_helper_pin(g) {
            Some((b, e, d, u, n, p)) => (b, e, d, u, n, Some(p)),
            None => self.init_iroute_helper_generic(g),
        };
        let track = if !self.is_init() { guide.route.map_or(0, |(sb, _)| if self.horizontal { sb.1 } else { sb.0 }) } else { 0 };
        let at = |c: i32| if self.horizontal { (c, track) } else { (track, c) };
        let (width, ext_len) = self.seg_style(guide.net, guide.layer);
        let mut figs = vec![Fig::Seg { begin: at(begin), end: at(end), layer: guide.layer, width, ext: ext_len }];
        let ndr = self.ndr_of(guide.net);
        let pref = |z: i64| -> Option<usize> { ndr.and_then(|n| usize::try_from(z).ok().and_then(|z| n.vias.get(z)).and_then(|v| v.first()).copied()) };
        for &c in &up {
            let via = pref(guide.layer as i64 / 2 - 1).or_else(|| self.default_via(guide.layer as i64 + 1)).expect("an up via");
            figs.push(Fig::Via { via, origin: at(c) });
        }
        for &c in &down {
            let via = pref((guide.layer as i64 - 2) / 2 - 1).or_else(|| self.default_via(guide.layer as i64 - 1)).expect("a down via");
            figs.push(Fig::Via { via, origin: at(c) });
        }
        let id = self.iroutes.len();
        self.iroutes.push(Iroute { guide: g, figs, next_dir, pin, cost: 0, assigned: 0 });
        if !ext {
            self.own.push(id);
        }
    }

    pub fn init_iroutes(&mut self) {
        for l in 0..self.tech().layers.len() {
            if self.tech().layers[l].kind != LayerKind::Routing || !self.layer_dir_matches(l) {
                continue;
            }
            for g in self.st.query_guides(&self.ext_box, l) {
                self.init_iroute(g);
            }
        }
    }

    pub fn init_costs(&mut self) {
        if self.is_init() {
            for &id in &self.own.clone() {
                let pitch = self.tech().layers[self.st.guides[self.iroutes[id].guide].layer].pitch;
                for f in self.iroutes[id].figs.clone() {
                    if let Fig::Seg { begin, end, .. } = f {
                        let (bc, ec) = if self.horizontal { (begin.0, end.0) } else { (begin.1, end.1) };
                        self.iroutes[id].cost = (ec - bc + i32::from(self.iroutes[id].pin.is_some()) * pitch * 1000) as u32;
                    }
                }
            }
        } else {
            for id in 0..self.iroutes.len() {
                for k in 0..self.iroutes[id].figs.len() {
                    self.rq_add(id, k);
                    self.add_cost(id, k, None);
                }
            }
            for &id in &self.own.clone() {
                let track = self.track_of(id);
                let (_, drc) = self.get_cost(id, track);
                self.iroutes[id].cost = drc;
            }
        }
    }

    fn track_of(&self, id: usize) -> i32 {
        for f in &self.iroutes[id].figs {
            if let Fig::Seg { begin, .. } = f {
                return if self.horizontal { begin.1 } else { begin.0 };
            }
        }
        panic!("an iroute without a wire");
    }

    pub fn sort_iroutes(&mut self) {
        for &id in &self.own {
            let clock = self.st.input.nets[self.net_of(id)].is_clock;
            if (self.is_init() || self.iroutes[id].cost != 0) && self.hard == clock {
                self.reassign.insert((std::cmp::Reverse(self.iroutes[id].cost), id));
            }
        }
    }

    // ---- the worker's shape and cost queries ----

    fn fig_box(&self, f: &Fig) -> (usize, Rect) {
        match *f {
            Fig::Seg { begin, end, layer, .. } => (layer, norm(begin, end)),
            Fig::Via { via, origin } => (self.tech().via_defs[via].cut, Rect { xl: origin.0, yl: origin.1, xh: origin.0, yh: origin.1 }),
        }
    }
    fn rq_add(&mut self, id: usize, k: usize) {
        let (l, b) = self.fig_box(&self.iroutes[id].figs[k]);
        self.shapes[l].push(b, (id, k));
    }
    fn rq_remove(&mut self, id: usize, k: usize) {
        let (l, b) = self.fig_box(&self.iroutes[id].figs[k]);
        self.shapes[l].remove_one(b, (id, k));
    }
    fn rq_query(&self, q: &Rect, layer: usize, out: &mut BTreeSet<usize>) {
        for (_, (id, _)) in self.shapes[layer].query(q) {
            out.insert(*id);
        }
    }

    // ---- cost modifiers ----

    fn seg_bbox(f: &Fig) -> Rect {
        let Fig::Seg { begin, end, width, ext, .. } = *f else { unreachable!() };
        if begin.0 != end.0 {
            Rect { xl: begin.0 - ext, yl: begin.1 - width / 2, xh: end.0 + ext, yh: end.1 + width / 2 }
        } else {
            Rect { xl: begin.0 - width / 2, yl: begin.1 - ext, xh: end.0 + width / 2, yh: end.1 + ext }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn put_cost(&mut self, via: bool, r: Rect, layer: usize, owner: Owner, con: Con, add: bool, pins: &mut Option<&mut BTreeSet<usize>>) {
        let v = if via { &mut self.via_costs[layer] } else { &mut self.route_costs[layer] };
        if add {
            v.push(r, (owner, con));
        } else {
            v.remove_one(r, (owner, con));
        }
        if let Some(p) = pins {
            self.rq_query(&r, layer, p);
        }
    }

    fn mod_min_spacing_cost_planar(&mut self, bx: &Rect, layer: usize, owner: Owner, net: usize, add: bool, pins: &mut Option<&mut BTreeSet<usize>>) {
        let (w1, l1) = (min_side(bx), max_side(bx));
        let w2 = self.tech().layers[layer].width;
        let hw2 = w2 / 2;
        let mut d = self.min_spacing_value(layer, w1, w2, l1);
        if let Some(n) = self.ndr_of(net) {
            d = d.max(n.spacings.get(layer / 2 - 1).copied().unwrap_or(0));
        }
        let d2 = i64::from(d) * i64::from(d);
        let (low, high, left, right) = if self.horizontal { (bx.yl, bx.yh, bx.xl, bx.xh) } else { (bx.xl, bx.xh, bx.yl, bx.yh) };
        let b1 = Rect { xl: left, yl: low, xh: right, yh: high };
        let (i1, i2) = self.get_track_idx(low - d - hw2 + 1, high + d + hw2 - 1, layer);
        for i in i1..=i2 {
            let t = self.track_locs[layer][i as usize];
            let b2 = Rect { xl: left - hw2, yl: t - hw2, xh: left + hw2, yh: t + hw2 };
            let dy = (b1.yl.max(b2.yl) - b1.yh.min(b2.yh)).max(0);
            if dy >= d {
                continue;
            }
            let mut max_x = (((d2 - i64::from(dy) * i64::from(dy)) as f64).sqrt()) as i32;
            if i64::from(max_x) * i64::from(max_x) + i64::from(dy) * i64::from(dy) == d2 {
                max_x = (max_x - 1).max(0);
            }
            let (bl, br) = (left - max_x - hw2, right + max_x + hw2);
            let r = if self.horizontal { Rect { xl: bl, yl: t, xh: br, yh: t } } else { Rect { xl: t, yl: bl, xh: t, yh: br } };
            self.put_cost(false, r, layer, owner, Con::MinSpacing(layer), add, pins);
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn mod_min_spacing_cost_via(&mut self, bx: &Rect, layer: usize, owner: Owner, net: usize, add: bool, upper: bool, curr_ps: bool, pins: &mut Option<&mut BTreeSet<usize>>) {
        let tech = self.tech();
        let (w1, l1) = (min_side(bx), max_side(bx));
        let (via, cut) = if upper {
            (if layer < self.top_layer() { self.default_via(layer as i64 + 1) } else { None }, layer as i64 + 1)
        } else {
            (if layer > 0 { self.default_via(layer as i64 - 1) } else { None }, layer as i64 - 1)
        };
        let Some(via) = via else { return };
        let vd = &tech.via_defs[via];
        let vb = if upper { vd.layer1_bbox() } else { vd.layer2_bbox() };
        let (w2, l2) = (min_side(&vb), max_side(&vb));
        let follow = if self.is_routing(cut - 1) && self.layer_dir_matches((cut - 1) as usize) {
            (cut - 1) as usize
        } else if self.is_routing(cut + 1) && self.layer_dir_matches((cut + 1) as usize) {
            (cut + 1) as usize
        } else {
            return;
        };
        let cut = cut as usize;
        let ndr_sp = self.ndr_of(net).map(|n| n.spacings.get(layer / 2 - 1).copied().unwrap_or(0));
        let mut d = self.min_spacing_value(layer, w1, w2, if curr_ps { l2 } else { l1.min(l2) });
        if let Some(s) = ndr_sp {
            d = d.max(s);
        }
        let (i1, i2) = if self.horizontal { self.get_track_idx(bx.yl - d - vb.yh + 1, bx.yh + d - vb.yl - 1, follow) } else { self.get_track_idx(bx.xl - d - vb.xh + 1, bx.xh + d - vb.xl - 1, follow) };
        for i in i1..=i2 {
            let t = self.track_locs[follow][i as usize];
            let off = if self.horizontal { (bx.xl, t) } else { (t, bx.yl) };
            let tb = shift(&vb, off);
            let dx = (bx.xl.max(tb.xl) - bx.xh.min(tb.xh)).max(0);
            let dy = (bx.yl.max(tb.yl) - bx.yh.min(tb.yh)).max(0);
            let prl = if self.horizontal {
                if dy > 0 {
                    if curr_ps { vb.dx() } else { bx.dx().min(vb.dx()) }
                } else if curr_ps {
                    vb.dy()
                } else {
                    bx.dy().min(vb.dy())
                }
            } else if dx > 0 {
                if curr_ps { vb.dy() } else { bx.dy().min(vb.dy()) }
            } else if curr_ps {
                vb.dx()
            } else {
                bx.dx().min(vb.dx())
            };
            let mut req = self.min_spacing_value(layer, w1, w2, prl);
            if let Some(s) = ndr_sp {
                req = req.max(s);
            }
            let gap = if self.horizontal { dy } else { dx };
            if gap >= req {
                continue;
            }
            let mut max_x = ((f64::from(req) * f64::from(req) - f64::from(gap) * f64::from(gap)).sqrt()) as i32;
            if max_x * max_x + gap * gap == req * req {
                max_x = (max_x - 1).max(0);
            }
            let r = if self.horizontal { Rect { xl: bx.xl - max_x - vb.xh, yl: t, xh: bx.xh + max_x - vb.xl, yh: t } } else { Rect { xl: t, yl: bx.yl - max_x - vb.yh, xh: t, yh: bx.yh + max_x - vb.yl } };
            self.put_cost(true, r, cut, owner, Con::MinSpacing(layer), add, pins);
        }
    }

    fn mod_cut_spacing_cost(&mut self, bx: &Rect, layer: usize, owner: Owner, add: bool, pins: &mut Option<&mut BTreeSet<usize>>) {
        let tech = self.tech();
        let Some(spacing) = tech.layers[layer].cut_spacing else { return };
        let Some(via) = self.default_via(layer as i64) else { return };
        let cf = &tech.via_defs[via].cut_figs;
        let vb = cf.iter().skip(1).fold(cf[0], |b, f| Rect { xl: b.xl.min(f.xl), yl: b.yl.min(f.yl), xh: b.xh.max(f.xh), yh: b.yh.max(f.yh) });
        let follow = if self.is_routing(layer as i64 - 1) && self.layer_dir_matches(layer - 1) {
            layer - 1
        } else if self.is_routing(layer as i64 + 1) && self.layer_dir_matches(layer + 1) {
            layer + 1
        } else {
            return;
        };
        let d = spacing;
        let (i1, i2) = if self.horizontal { self.get_track_idx(bx.yl - d - vb.yh + 1, bx.yh + d - vb.yl - 1, follow) } else { self.get_track_idx(bx.xl - d - vb.xh + 1, bx.xh + d - vb.xl - 1, follow) };
        for i in i1..=i2 {
            let t = self.track_locs[follow][i as usize];
            let off = if self.horizontal { (bx.xl, t) } else { (t, bx.yl) };
            let tb = shift(&vb, off);
            let dx = (bx.xl.max(tb.xl) - bx.xh.min(tb.xh)).max(0);
            let dy = (bx.yl.max(tb.yl) - bx.yh.min(tb.yh)).max(0);
            // One plain spacing rule: edge to edge, different net, always a violation.
            let req = spacing;
            let gap = if self.horizontal { dy } else { dx };
            if gap >= req {
                continue;
            }
            let mut max_x = ((f64::from(req) * f64::from(req) - f64::from(gap) * f64::from(gap)).sqrt()) as i32;
            if max_x * max_x + gap * gap == req * req {
                max_x = (max_x - 1).max(0);
            }
            let r = if self.horizontal { Rect { xl: bx.xl - max_x - vb.xh, yl: t, xh: bx.xh + max_x - vb.xl, yh: t } } else { Rect { xl: t, yl: bx.yl - max_x - vb.yh, xh: t, yh: bx.yh + max_x - vb.yl } };
            self.put_cost(true, r, layer, owner, Con::Cut(layer), add, pins);
        }
    }

    fn mod_cost(&mut self, id: usize, k: usize, add: bool, mut pins: Option<&mut BTreeSet<usize>>) {
        let f = self.iroutes[id].figs[k];
        let net = self.net_of(id);
        let owner = Owner::Fig(id, k);
        match f {
            Fig::Seg { layer, .. } => {
                let bx = Self::seg_bbox(&f);
                self.mod_min_spacing_cost_planar(&bx, layer, owner, net, add, &mut pins);
                self.mod_min_spacing_cost_via(&bx, layer, owner, net, add, true, true, &mut pins);
                self.mod_min_spacing_cost_via(&bx, layer, owner, net, add, false, true, &mut pins);
            }
            Fig::Via { via, origin } => {
                let vd = &self.tech().via_defs[via];
                for (bx, layer) in [(shift(&vd.layer1_bbox(), origin), vd.layer1), (shift(&vd.layer2_bbox(), origin), vd.layer2)] {
                    if self.layer_dir_matches(layer) {
                        self.mod_min_spacing_cost_planar(&bx, layer, owner, net, add, &mut pins);
                    }
                    self.mod_min_spacing_cost_via(&bx, layer, owner, net, add, true, false, &mut pins);
                    self.mod_min_spacing_cost_via(&bx, layer, owner, net, add, false, false, &mut pins);
                }
                for c in vd.cut_figs.clone() {
                    self.mod_cut_spacing_cost(&shift(&c, origin), vd.cut, owner, add, &mut pins);
                }
            }
        }
    }

    fn add_cost(&mut self, id: usize, k: usize, pins: Option<&mut BTreeSet<usize>>) {
        self.mod_cost(id, k, true, pins);
    }
    fn sub_cost(&mut self, id: usize, k: usize, pins: Option<&mut BTreeSet<usize>>) {
        self.mod_cost(id, k, false, pins);
    }

    // ---- assignment ----

    /// The tracks across the guide's first gcell (its top/right edge left out). On a
    /// unidirectional layer the range shrinks so that a default via centred on the first or last
    /// track stays inside the die: the via above the layer, or below it on the top layer; half its
    /// box's extent across the tracks.
    fn assign_iroute_avail_tracks(&self, id: usize) -> (usize, i32, i32) {
        let g = &self.st.guides[self.iroutes[id].guide];
        let layer = g.layer;
        let gb = self.st.input.grid.gcell_box(self.st.input.grid.idx(g.begin));
        let (mut lo, mut hi) = if self.horizontal { (gb.yl, gb.yh - 1) } else { (gb.xl, gb.xh - 1) };
        if self.tech().layers[layer].is_unidirectional() {
            let die = self.st.input.die;
            let cut = if layer < self.top_layer() { layer + 1 } else { layer - 1 };
            let vd = &self.tech().via_defs[self.default_via(cut as i64).expect("a default via beside a unidirectional layer")];
            let (b1, b2) = (vd.layer1_bbox(), vd.layer2_bbox());
            let test = Rect { xl: b1.xl.min(b2.xl), yl: b1.yl.min(b2.yl), xh: b1.xh.max(b2.xh), yh: b1.yh.max(b2.yh) };
            let (diff_lo, diff_hi) = if self.horizontal { (die.yl - (lo - test.dy() / 2), hi + test.dy() / 2 - die.yh) } else { (die.xl - (lo - test.dx() / 2), hi + test.dx() / 2 - die.xh) };
            if diff_lo > 0 {
                lo += diff_lo;
            }
            if diff_hi > 0 {
                hi -= diff_hi;
            }
        }
        let (i1, i2) = self.get_track_idx(lo, hi, layer);
        assert!(i2 >= i1, "no tracks in a gcell");
        (layer, i1, i2)
    }

    fn get_next_iroute_dir_cost(&self, id: usize, track: i32) -> u32 {
        let g = &self.st.guides[self.iroutes[id].guide];
        let eb = self.st.input.grid.gcell_box(self.st.input.grid.idx(g.end));
        let nd = self.iroutes[id].next_dir;
        let c = if nd <= 0 {
            nd.abs() * (track - if self.horizontal { eb.yl } else { eb.xl })
        } else {
            nd.abs() * (if self.horizontal { eb.yh } else { eb.xh } - track)
        };
        if c < 0 {
            0
        } else {
            c as u32
        }
    }

    /// The distance to the pin's coordinate; on a unidirectional layer, off the pin, plus the DRC
    /// cost when neither the layer below nor the one above can bridge that offset: each is ruled
    /// out by the via-to-via forbidden length (two vias down / two up, across a horizontal panel's
    /// tracks as along y, a vertical one's as along x) or by lying outside the routing layers.
    fn get_pin_cost(&self, id: usize, track: i32) -> u32 {
        let Some(p) = self.iroutes[id].pin else {
            return 0;
        };
        let mut sol = (track - p).unsigned_abs();
        let layer = self.st.guides[self.iroutes[id].guide].layer;
        if sol != 0 && self.tech().layers[layer].is_unidirectional() {
            let z = layer / 2 - 1;
            let dir_x = !self.horizontal;
            let forbidden = |prev_down: bool, curr_down: bool| -> bool {
                let k = usize::from(!prev_down) * 4 + usize::from(!curr_down) * 2 + usize::from(!dir_x);
                self.st.input.tables.get(z).is_some_and(|t| t.via2via[k].iter().any(|&(lo, hi)| lo <= sol as i32 && sol as i32 <= hi))
            };
            if (forbidden(false, false) || (layer as i64) - 2 < self.cfg().bottom_routing_layer as i64) && (forbidden(true, true) || layer + 2 > self.top_layer()) {
                sol += self.cfg().drc_cost;
            }
        }
        sol
    }

    fn get_drc_cost_helper(&self, id: usize, bx: &Rect, layer: usize) -> u32 {
        let net = self.net_of(id);
        let mut bx = *bx;
        if let Some(n) = self.ndr_of(net) {
            let r = n.widths.get(layer / 2 - 1).copied().unwrap_or(0) / 2 + n.spacings.get(layer / 2 - 1).copied().unwrap_or(0);
            bx = bloat(&bx, r);
        }
        let flat = |e: &(Rect, (Owner, Con))| (e.0, e.1 .0, e.1 .1);
        let mut result: Vec<CostBox> = self.route_costs[layer].query(&bx).map(flat).collect();
        let is_cut = self.tech().layers[layer].kind == LayerKind::Cut;
        if is_cut {
            result.extend(self.via_costs[layer].query(&bx).map(flat));
        } else {
            let (add_h, add_v) = if self.tech().layers[layer].is_horizontal() { (self.st.input.grid.x.2 / 2, 0) } else { (0, self.st.input.grid.y.2 / 2) };
            let b1 = Rect { xl: bx.xl, yl: bx.yl, xh: bx.xh.min(bx.xl + add_h), yh: bx.yh.min(bx.yl + add_v) };
            let b2 = Rect { xl: bx.xl.max(bx.xh - add_h), yl: bx.yl.max(bx.yh - add_v), xh: bx.xh, yh: bx.yh };
            result.extend(self.via_costs[layer].query(&b1).map(flat));
            result.extend(self.via_costs[layer].query(&b2).map(flat));
        }
        let same: Vec<Rect> = result.iter().filter(|e| e.1 == Owner::Net(net)).map(|e| e.0).collect();
        let mut overlap: i64 = 0;
        for (b, owner, _) in &result {
            if same.iter().any(|s| touches(s, b)) {
                continue;
            }
            let ov = -i64::from(bx.xl.max(b.xl)) + i64::from(bx.xh.min(b.xh)) - i64::from(bx.yl.max(b.yl)) + i64::from(bx.yh.min(b.yh)) + 1;
            match *owner {
                Owner::None => overlap += ov,
                Owner::Net(n) => {
                    if n != net {
                        overlap += ov;
                    }
                }
                Owner::Fig(o, _) => {
                    if o == id {
                        continue;
                    }
                    if self.net_of(o) != net {
                        overlap += ov;
                    }
                }
            }
        }
        let tech = self.tech();
        let pitch = if !is_cut {
            tech.layers[layer].pitch
        } else if layer < self.top_layer() && tech.layers[layer + 1].kind == LayerKind::Routing {
            tech.layers[layer + 1].pitch
        } else {
            tech.layers[layer - 1].pitch
        };
        if overlap == 0 {
            return 0;
        }
        if is_cut {
            (pitch * 2) as u32
        } else {
            (i64::from(pitch) * 2).max(overlap) as u32
        }
    }

    fn get_drc_cost(&self, id: usize, track: i32) -> u32 {
        let mut cost: u32 = 0;
        for f in &self.iroutes[id].figs {
            match *f {
                Fig::Seg { begin, end, layer, .. } => {
                    let (b, e) = if self.horizontal { ((begin.0, track), (end.0, track)) } else { ((track, begin.1), (track, end.1)) };
                    cost = cost.wrapping_add(self.get_drc_cost_helper(id, &norm(b, e), layer));
                }
                Fig::Via { via, origin } => {
                    let p = if self.horizontal { (origin.0, track) } else { (track, origin.1) };
                    cost = cost.wrapping_add(self.get_drc_cost_helper(id, &Rect { xl: p.0, yl: p.1, xh: p.0, yh: p.1 }, self.tech().via_defs[via].cut));
                }
            }
        }
        cost
    }

    fn get_align_cost(&self, id: usize, track: i32) -> u32 {
        let net = self.net_of(id);
        for f in &self.iroutes[id].figs {
            if let Fig::Seg { begin, end, layer, .. } = *f {
                let pitch = self.tech().layers[layer].pitch;
                let q = if self.horizontal { Rect { xl: begin.0, yl: track, xh: end.0, yh: track } } else { Rect { xl: track, yl: begin.1, xh: track, yh: end.1 } };
                let mut s = BTreeSet::new();
                self.rq_query(&q, layer, &mut s);
                if s.iter().any(|&o| self.net_of(o) == net) {
                    return pitch as u32;
                }
            }
        }
        0
    }

    /// The total cost of `track`, and its DRC part.
    fn get_cost(&self, id: usize, track: i32) -> (u32, u32) {
        let pitch = self.tech().layers[self.st.guides[self.iroutes[id].guide].layer].pitch;
        let drc_out = self.get_drc_cost(id, track);
        let drc: i32 = if self.is_init() { (0.05 * f64::from(drc_out)) as i32 } else { self.cfg().drc_cost.wrapping_mul(drc_out) as i32 };
        let next = self.get_next_iroute_dir_cost(id, track) as i32;
        let tmp_pin = self.get_pin_cost(id, track);
        let pin = if tmp_pin == 0 { 0 } else { (self.cfg().pin_cost as i32 * pitch) + tmp_pin as i32 };
        let tmp_align = self.get_align_cost(id, track);
        let align = if tmp_align == 0 { 0 } else { (self.cfg().align_cost as i32 * pitch) + tmp_align as i32 };
        ((drc + next + pin - align).max(0) as u32, drc_out)
    }

    fn best_track_helper(&mut self, id: usize, layer: usize, idx: i32, best: &mut (u32, i32, i32), drc: &mut u32) {
        let t = self.track_locs[layer][idx as usize];
        let (cost, d) = self.get_cost(id, t);
        *drc = d;
        self.emit(TaEvent::Try { track: t, cost, drc: d });
        let key = if self.is_init() { cost } else { d };
        if key < best.0 {
            *best = (key, t, idx);
        }
    }

    fn assign_iroute_best_track(&mut self, id: usize, layer: usize, i1: i32, i2: i32) -> i32 {
        let mut best: (u32, i32, i32) = (u32::MAX, 0, -1);
        let mut drc: u32 = 0;
        let nd = self.iroutes[id].next_dir;
        let tl = self.track_locs[layer].clone();
        if let Some(pin) = self.iroutes[id].pin {
            let start = (tl.partition_point(|&x| x < pin) as i32).min(i2).max(i1);
            if nd > 0 {
                let mut i = start;
                while i <= i2 {
                    self.best_track_helper(id, layer, i, &mut best, &mut drc);
                    if drc == 0 {
                        break;
                    }
                    i += 1;
                }
                if drc != 0 {
                    let mut i = start - 1;
                    while i >= i1 {
                        self.best_track_helper(id, layer, i, &mut best, &mut drc);
                        if drc == 0 {
                            break;
                        }
                        i -= 1;
                    }
                }
            } else if nd == 0 {
                for k in 0..=(i2 - i1) {
                    let c = start + k;
                    if c >= i1 && c <= i2 {
                        self.best_track_helper(id, layer, c, &mut best, &mut drc);
                    }
                    if drc == 0 {
                        break;
                    }
                    let c = start - k - 1;
                    if c >= i1 && c <= i2 {
                        self.best_track_helper(id, layer, c, &mut best, &mut drc);
                    }
                    if drc == 0 {
                        break;
                    }
                }
            } else {
                let mut i = start;
                while i >= i1 {
                    self.best_track_helper(id, layer, i, &mut best, &mut drc);
                    if drc == 0 {
                        break;
                    }
                    i -= 1;
                }
                if drc != 0 {
                    let mut i = start + 1;
                    while i <= i2 {
                        self.best_track_helper(id, layer, i, &mut best, &mut drc);
                        if drc == 0 {
                            break;
                        }
                        i += 1;
                    }
                }
            }
        } else if nd > 0 {
            let mut i = i2;
            while i >= i1 {
                self.best_track_helper(id, layer, i, &mut best, &mut drc);
                if drc == 0 {
                    break;
                }
                i -= 1;
            }
        } else if nd == 0 {
            let mid = (i1 + i2) / 2;
            let mut i = mid;
            while i <= i2 {
                self.best_track_helper(id, layer, i, &mut best, &mut drc);
                if drc == 0 {
                    break;
                }
                i += 1;
            }
            if drc != 0 {
                let mut i = mid - 1;
                while i >= i1 {
                    self.best_track_helper(id, layer, i, &mut best, &mut drc);
                    if drc == 0 {
                        break;
                    }
                    i -= 1;
                }
            }
        } else {
            let mut i = i1;
            while i <= i2 {
                self.best_track_helper(id, layer, i, &mut best, &mut drc);
                if drc == 0 {
                    break;
                }
                i += 1;
            }
        }
        assert!(best.2 != -1, "no track selected");
        self.iroutes[id].cost = drc;
        best.1
    }

    fn assign_iroute_update_iroute(&mut self, id: usize, track: i32, pins: &mut BTreeSet<usize>) {
        let h = self.horizontal;
        for f in &mut self.iroutes[id].figs {
            match f {
                Fig::Seg { begin, end, .. } => {
                    if h {
                        begin.1 = track;
                        end.1 = track;
                    } else {
                        begin.0 = track;
                        end.0 = track;
                    }
                }
                Fig::Via { origin, .. } => {
                    if h {
                        origin.1 = track;
                    } else {
                        origin.0 = track;
                    }
                }
            }
        }
        for k in 0..self.iroutes[id].figs.len() {
            if self.is_init() {
                self.add_cost(id, k, None);
            } else {
                self.add_cost(id, k, Some(pins));
            }
            self.rq_add(id, k);
        }
        self.iroutes[id].assigned += 1;
    }

    fn assign_iroute_init(&mut self, id: usize, pins: &mut BTreeSet<usize>) {
        if !self.is_init() {
            for k in 0..self.iroutes[id].figs.len() {
                self.rq_remove(id, k);
                self.sub_cost(id, k, Some(pins));
            }
        }
    }

    fn assign_iroute_update_others(&mut self, pins: &BTreeSet<usize>) {
        if self.is_init() {
            return;
        }
        for &o in pins {
            if self.st.input.nets[self.net_of(o)].is_clock && !self.hard {
                continue;
            }
            // ⚠️ Context iroutes (reaching outside the panel) are requeued and reassigned too —
            // only the panel's own are saved.
            self.reassign.remove(&(std::cmp::Reverse(self.iroutes[o].cost), o));
            let track = self.track_of(o);
            let (_, drc) = self.get_cost(o, track);
            self.iroutes[o].cost = drc;
            let queued = drc != 0 && self.iroutes[o].assigned < 1;
            self.emit(TaEvent::Requeue { id: o, drc, queued });
            if queued {
                self.reassign.insert((std::cmp::Reverse(drc), o));
            }
        }
    }

    fn assign_iroute(&mut self, id: usize) {
        let mut pins = BTreeSet::new();
        self.assign_iroute_init(id, &mut pins);
        let (layer, i1, i2) = self.assign_iroute_avail_tracks(id);
        let track = self.assign_iroute_best_track(id, layer, i1, i2);
        let (hard, drc) = (self.hard, self.iroutes[id].cost);
        self.emit(TaEvent::Assign { id, hard, idx1: i1, idx2: i2, track, drc });
        self.assign_iroute_update_iroute(id, track, &mut pins);
        self.assign_iroute_update_others(&pins);
    }

    pub fn assign(&mut self) {
        let mut buffers: Vec<Option<usize>> = vec![None; 20];
        let mut cur = 0;
        while let Some(&(k, id)) = self.reassign.iter().next() {
            self.reassign.remove(&(k, id));
            if buffers.contains(&Some(id)) || self.iroutes[id].assigned >= 1 {
                continue;
            }
            self.assign_iroute(id);
            buffers[cur] = Some(id);
            cur = (cur + 1) % 20;
        }
    }

    /// One panel: init, then (initial pass) the clock iroutes first, then the rest.
    pub fn main_mt(&mut self) {
        self.init();
        if self.is_init() {
            self.hard = true;
            self.sort_iroutes();
            self.assign();
            self.hard = false;
        }
        self.sort_iroutes();
        self.assign();
    }

    pub fn init(&mut self) {
        let n = self.tech().layers.len();
        let h = self.horizontal;
        self.shapes = (0..n).map(|_| TrackIndex::new(h)).collect();
        self.route_costs = (0..n).map(|_| TrackIndex::new(h)).collect();
        self.via_costs = (0..n).map(|_| TrackIndex::new(h)).collect();
        self.init_tracks();
        self.init_fixed_objs();
        self.init_iroutes();
        self.init_costs();
        for (l, t) in self.track_locs.clone().iter().enumerate() {
            if !t.is_empty() {
                self.emit(TaEvent::Tracks { layer: l, count: t.len(), first: t[0], last: *t.last().expect("a track") });
            }
        }
        for id in 0..self.iroutes.len() {
            let r = &self.iroutes[id];
            let (begin, end) = match r.figs[0] {
                Fig::Seg { begin, end, .. } => (begin, end),
                Fig::Via { .. } => unreachable!(),
            };
            let vias = r.figs[1..].iter().map(|f| if let Fig::Via { via, origin } = *f { (via, origin) } else { unreachable!() }).collect();
            let ev = TaEvent::Iroute { id, ext: !self.own.contains(&id), guide: r.guide, begin, end, next_dir: r.next_dir, pin: r.pin, cost: r.cost, vias };
            self.emit(ev);
        }
    }

    /// Each assigned iroute's wire, for its guide.
    pub fn save_to_guides(&self) -> Vec<(usize, (P, P))> {
        self.own
            .iter()
            .filter_map(|&id| {
                let r = &self.iroutes[id];
                r.figs.iter().find_map(|f| if let Fig::Seg { begin, end, .. } = *f { Some((r.guide, (begin, end))) } else { None })
            })
            .collect()
    }
}

/// The panels of one pass, direction `horizontal`, in batches.
fn panels(grid: &GCellGrid, size: i32, offset: i32, horizontal: bool) -> Vec<Rect> {
    let (xn, yn) = (grid.x.1, grid.y.1);
    let mut out = Vec::new();
    let n = if horizontal { yn } else { xn };
    let mut i = offset;
    while i < n {
        let (b, e) = if horizontal { (grid.gcell_box((0, i)), grid.gcell_box((xn - 1, (i + size - 1).min(yn - 1)))) } else { (grid.gcell_box((i, 0)), grid.gcell_box(((i + size - 1).min(xn - 1), yn - 1))) };
        out.push(Rect { xl: b.xl, yl: b.yl, xh: e.xh, yh: e.yh });
        i += size;
    }
    out
}

fn run_pass(st: &mut TaState<'_>, iter: usize, horizontal: bool, trace: &mut Option<Vec<TaEvent>>) {
    let size = 50;
    let grid = *st.input.grid;
    let boxes = panels(&grid, size, 0, horizontal);
    let half = if horizontal { grid.y.2 / 2 } else { grid.x.2 / 2 };
    let bs = st.input.cfg.batch_size;
    for (b, chunk) in boxes.chunks(bs).enumerate() {
        let mut saves: Vec<Vec<(usize, (P, P))>> = Vec::new();
        let mut logs: Vec<Vec<TaEvent>> = Vec::new();
        for (k, rb) in chunk.iter().enumerate() {
            let mut w = Worker {
                st,
                route_box: *rb,
                ext_box: bloat(rb, half),
                horizontal,
                iter,
                hard: false,
                track_locs: Vec::new(),
                iroutes: Vec::new(),
                own: Vec::new(),
                shapes: Vec::new(),
                route_costs: Vec::new(),
                via_costs: Vec::new(),
                reassign: BTreeSet::new(),
                trace: trace.as_ref().map(|_| vec![TaEvent::Worker { iter, horizontal, batch: b, index: k, route_box: *rb }]),
            };
            w.main_mt();
            let s = w.save_to_guides();
            let mut log = w.trace.take().unwrap_or_default();
            for &(g, (sb, se)) in &s {
                log.push(TaEvent::Save { guide: g, begin: sb, end: se });
            }
            log.push(TaEvent::EndWorker);
            saves.push(s);
            logs.push(log);
        }
        // After the batch, in batch order.
        for s in saves {
            for (g, r) in s {
                st.guides[g].route = Some(r);
            }
        }
        if let Some(t) = trace {
            for l in logs {
                t.extend(l);
            }
        }
    }
}

/// Track assignment: the initial pass and one repair pass; the guides' routes are the result.
pub fn track_assignment(st: &mut TaState<'_>, trace: &mut Option<Vec<TaEvent>>) {
    // ⚠️ "The bottom layer" is layer 0, or layer 1 when layer 0 is not a routing layer — the
    // placeholder cut layer, which is never horizontal: vertical panels go first.
    let tech = st.input.tech;
    let bottom = if tech.layers[0].kind == LayerKind::Routing { 0 } else { 1 };
    let h_first = tech.layers[bottom].is_horizontal();
    for iter in [0, 1] {
        for h in if h_first { [true, false] } else { [false, true] } {
            run_pass(st, iter, h, trace);
        }
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::tech::{Dir, Layer, SpacingTable, ViaDef};

    fn r(xl: i32, yl: i32, xh: i32, yh: i32) -> Rect {
        Rect { xl, yl, xh, yh }
    }

    /// 0–1 placeholders, 2 horizontal routing, 3 cut, 4 vertical routing (width 100, pitch 200;
    /// spacing 100, or 150 over runs above 200); one via whose lower metal is 300 wide.
    fn tech() -> Tech {
        let table = SpacingTable { widths: vec![0], prls: vec![0, 200], values: vec![vec![100, 150]] };
        let routing = |dir| Layer { kind: LayerKind::Routing, dir, width: 100, min_width: 100, pitch: 200, spacing: Some(table.clone()), ..Default::default() };
        Tech {
            layers: vec![Layer::default(), Layer::default(), routing(Dir::Horizontal), Layer { kind: LayerKind::Cut, cut_spacing: Some(100), ..Default::default() }, routing(Dir::Vertical)],
            via_defs: vec![ViaDef { name: "v".into(), is_default: true, layer1: 2, cut: 3, layer2: 4, layer1_figs: vec![r(-150, -50, 150, 50)], cut_figs: vec![r(-50, -50, 50, 50)], layer2_figs: vec![r(-50, -150, 50, 150)] }],
            ..Default::default()
        }
    }

    struct Fixture {
        tech: Tech,
        defaults: Vec<Option<usize>>,
        eol: Vec<EolTable>,
        grid: GCellGrid,
        tracks: Vec<TrackPattern>,
        nets: Vec<TaNet>,
        fixed: Vec<Vec<(Rect, Fixed)>>,
    }

    fn fixture() -> Fixture {
        Fixture {
            tech: tech(),
            defaults: vec![None, None, None, Some(0), None],
            eol: vec![EolTable::default(); 2],
            grid: GCellGrid { x: (0, 10, 1000), y: (0, 10, 1000), die: r(0, 0, 10000, 10000) },
            tracks: vec![TrackPattern { layer: 2, vertical_tracks: false, start: 100, num: 50, spacing: 200 }, TrackPattern { layer: 4, vertical_tracks: true, start: 100, num: 50, spacing: 200 }],
            nets: vec![TaNet { name: "n".into(), is_clock: false, ndr: None }],
            fixed: vec![Vec::new(); 5],
        }
    }

    fn state(f: &Fixture, guides: Vec<TaGuide>) -> TaState<'_> {
        let input = TaInput { tech: &f.tech, defaults: &f.defaults, eol: &f.eol, tables: &[], grid: &f.grid, die: f.grid.die, tracks: &f.tracks, nets: &f.nets, fixed: &f.fixed, terms: &[], gr_pins: &[], cfg: TaConfig::default() };
        TaState::new(input, guides)
    }

    fn worker<'s, 'a>(st: &'s TaState<'a>, iter: usize) -> Worker<'s, 'a> {
        let rb = st.input.grid.die;
        let mut w = Worker { st, route_box: rb, ext_box: rb, horizontal: true, iter, hard: false, track_locs: Vec::new(), iroutes: Vec::new(), own: Vec::new(), shapes: Vec::new(), route_costs: Vec::new(), via_costs: Vec::new(), reassign: BTreeSet::new(), trace: None };
        let n = st.input.tech.layers.len();
        w.shapes = (0..n).map(|_| TrackIndex::new(true)).collect();
        w.route_costs = (0..n).map(|_| TrackIndex::new(true)).collect();
        w.via_costs = (0..n).map(|_| TrackIndex::new(true)).collect();
        w.init_tracks();
        w
    }

    // Both ends are inclusive: a track exactly at `lo` or `hi` is inside.
    #[test]
    fn a_track_range_includes_both_ends() {
        let t = [0, 10, 20, 30];
        assert_eq!(track_range(&t, 10, 20), (1, 2));
        assert_eq!(track_range(&t, 11, 19), (2, 1));
        assert_eq!(track_range(&t, -5, 35), (0, 3));
    }

    // Neighbours' wires can arrive inverted (the begin side's wire starts beyond the end side's):
    // the span is swapped into order.
    #[test]
    fn an_inverted_span_is_swapped() {
        let f = fixture();
        let g = |layer, b, e, route| TaGuide { net: 0, layer, begin: b, end: e, route };
        let guides = vec![g(2, (500, 500), (2500, 500), None), g(4, (500, 500), (500, 500), Some(((2300, 500), (2300, 700)))), g(4, (2500, 500), (2500, 500), Some(((700, 500), (700, 700))))];
        let st = state(&f, guides);
        let w = worker(&st, 1);
        let (b, e, ..) = w.init_iroute_helper_generic(0);
        assert_eq!((b, e), (700, 2300));
    }

    // A segment's via spacing on the track beside it runs over the VIA's length (not the
    // shorter of the two): here 300, above the table's 200 column, so 150 is required and a
    // track 100 away is blocked.
    #[test]
    fn a_wire_to_via_run_is_the_vias_length() {
        let f = fixture();
        let st = state(&f, vec![TaGuide { net: 0, layer: 2, begin: (1000, 300), end: (1000, 300), route: None }]);
        let mut w = worker(&st, 0);
        let seg = r(950, 250, 1050, 350);
        w.mod_min_spacing_cost_via(&seg, 2, Owner::Net(0), 0, true, true, true, &mut None);
        assert!(w.via_costs[3].query(&r(-100000, 500, 100000, 500)).count() > 0);
    }

    /// Layer 2 rect-only (so unidirectional).
    fn rect_only(mut f: Fixture) -> Fixture {
        f.tech.layers[2].rect_only = true;
        f
    }

    // Rule: on a unidirectional layer a track is dropped where neither default via — the one
    // above (here 300 tall, centred on the track) nor the one below (none: counted as leaving)
    // — fits in the die. Track 100 goes (its via reaches y = −50); without rect-only it stays.
    #[test]
    fn a_unidirectional_layer_drops_tracks_whose_vias_leave_the_die() {
        let f = rect_only(fixture());
        let st = state(&f, vec![]);
        let w = worker(&st, 0);
        assert_eq!((w.track_locs[2][0], w.track_locs[2].last().copied()), (300, Some(9700)));
        let f = fixture();
        let st = state(&f, vec![]);
        let w = worker(&st, 0);
        assert_eq!((w.track_locs[2][0], w.track_locs[2].last().copied()), (100, Some(9900)));
    }

    // Rule: a guide's available tracks on a unidirectional layer stop half the via ABOVE's box
    // (300 tall) inside the die, whatever the via below: here a tiny via below keeps track 100
    // in the worker, yet the first gcell's range starts at 150, so its first track is 300.
    #[test]
    fn a_unidirectional_guide_keeps_its_tracks_half_a_via_inside_the_die() {
        let mut f = rect_only(fixture());
        f.tech.via_defs.push(ViaDef { name: "s".into(), is_default: true, layer1: 0, cut: 1, layer2: 2, layer1_figs: vec![r(-20, -20, 20, 20)], cut_figs: vec![r(-20, -20, 20, 20)], layer2_figs: vec![r(-20, -20, 20, 20)] });
        f.defaults[1] = Some(1);
        let st = state(&f, vec![TaGuide { net: 0, layer: 2, begin: (500, 500), end: (4500, 500), route: None }]);
        let mut w = worker(&st, 0);
        assert_eq!(w.track_locs[2][0], 100);
        w.init_iroutes();
        let (_, i1, _) = w.assign_iroute_avail_tracks(0);
        assert_eq!(w.track_locs[2][i1 as usize], 300);
        f.tech.layers[2].rect_only = false;
        let st = state(&f, vec![TaGuide { net: 0, layer: 2, begin: (500, 500), end: (4500, 500), route: None }]);
        let mut w = worker(&st, 0);
        w.init_iroutes();
        let (_, i1, _) = w.assign_iroute_avail_tracks(0);
        assert_eq!(w.track_locs[2][i1 as usize], 100);
    }

    // Rule: off a boundary pin's coordinate on a unidirectional layer, the DRC cost is added
    // when neither neighbour layer can bridge the offset: below is outside the routing layers
    // (bottom routing layer 2), above by the via-to-via forbidden length (two vias up, across a
    // horizontal panel's tracks = along y, the layer's OWN table): 1..=150 here. Offsets 0 and
    // 200 cost their distance; 100 costs 100 + 32.
    #[test]
    fn a_unidirectional_pin_offset_no_layer_can_bridge_costs_the_drc_cost() {
        let f = rect_only(fixture());
        let mut tables = vec![crate::dr::rules::LayerTables::default(); 2];
        tables[0].via2via[1] = vec![(1, 150)];
        let input = TaInput { tech: &f.tech, defaults: &f.defaults, eol: &f.eol, tables: &tables, grid: &f.grid, die: f.grid.die, tracks: &f.tracks, nets: &f.nets, fixed: &f.fixed, terms: &[], gr_pins: &[], cfg: TaConfig::default() };
        let st = TaState::new(input, vec![TaGuide { net: 0, layer: 2, begin: (500, 500), end: (4500, 500), route: None }]);
        let mut w = worker(&st, 0);
        w.init_iroutes();
        w.iroutes[0].pin = Some(500);
        assert_eq!((w.get_pin_cost(0, 500), w.get_pin_cost(0, 600), w.get_pin_cost(0, 700)), (0, 132, 200));
        let mut f2 = fixture();
        f2.tech.layers[2].rect_only = false;
        let input = TaInput { tech: &f2.tech, defaults: &f2.defaults, eol: &f2.eol, tables: &tables, grid: &f2.grid, die: f2.grid.die, tracks: &f2.tracks, nets: &f2.nets, fixed: &f2.fixed, terms: &[], gr_pins: &[], cfg: TaConfig::default() };
        let st = TaState::new(input, vec![TaGuide { net: 0, layer: 2, begin: (500, 500), end: (4500, 500), route: None }]);
        let mut w = worker(&st, 0);
        w.init_iroutes();
        w.iroutes[0].pin = Some(500);
        assert_eq!(w.get_pin_cost(0, 600), 100);
    }

    // Via costs count only within half a gcell of a wire's ends: one 700 in (beyond half of
    // 1000) does not.
    #[test]
    fn via_costs_count_near_the_wire_ends_only() {
        let f = fixture();
        let st = state(&f, vec![TaGuide { net: 0, layer: 2, begin: (500, 500), end: (4500, 500), route: None }]);
        let mut w = worker(&st, 0);
        w.init_iroutes();
        w.via_costs[2].push(r(700, 500, 700, 500), (Owner::None, Con::Short(2)));
        assert_eq!(w.get_drc_cost_helper(0, &r(0, 500, 5000, 500), 2), 0);
        w.via_costs[2].push(r(300, 500, 300, 500), (Owner::None, Con::Short(2)));
        assert_eq!(w.get_drc_cost_helper(0, &r(0, 500, 5000, 500), 2), 400);
    }
}
