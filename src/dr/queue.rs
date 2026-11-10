// SPDX-License-Identifier: Apache-2.0
//! A worker's search and repair: route the queue, check each routed net, and let the markers
//! requeue nets (to route again) and owners (to check again) and cost the grid where they sit.
//!
//! Rules:
//! - a route entry runs only if the net has not been rerouted since it was queued; the net's old
//!   shapes are lifted, it is routed, and the check runs on it;
//! - a check entry runs only if nothing was routed since that owner was last checked;
//! - a marker is the worker's when its box, or either shape it names, touches the route box. Its
//!   aggressor, when a signal net in the worker that may still be ripped up (fewer reroutes than
//!   the iteration allows), is rerouted and the marker's other owners checked; otherwise its other
//!   signal nets that may be ripped up are rerouted, the rest checked. With more than one owner
//!   involved, a net that may still avoid a ripup (clock nets up to 100 times, rule nets 3) is
//!   queued to check instead — unless another net just used up its avoids (⛔ such an entry names
//!   the worker net, which the check does not know: it checks nothing);
//! - each batch of new entries is sorted (routes, then checks; each by owner kind, name, id) and
//!   appended;
//! - after a route, the marker costs decay (× the iteration's decay, truncated; a node whose cost
//!   reaches 0 is forgotten); after a check, each marker adds 10 of marker cost to the route
//!   shapes under it (a wire: along it, at least two grid points either side; a via: once per
//!   node and per net per marker; a patch: every node it covers, and the vias there).

use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};

use crate::dr::cost::{CostWorker, DrFig, Ndr};
use crate::dr::drw::DrNet;
use crate::dr::maze::{Idx, MazeCfg, MazeState};
use crate::dr::route::{after_check, maze_net_end, reroute_net, NetCtx, Search};
use crate::gc::{Marker, Owner, PatchWire, Rule, Worker};
use crate::polygon90::Rect;
use crate::rtree::DynRTree;

/// What a queue entry names: a worker net (to route, or check), or another owner (to check).
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum Block {
    Net(usize),
    Owner(Owner),
}

#[derive(Debug, Clone)]
struct Entry {
    block: Block,
    num_reroute: i32,
    do_route: bool,
    checking: Option<Owner>,
}

/// One event of the queue, in order.
#[derive(Debug, Clone)]
pub enum Event {
    /// A net routed: its worker id, reroutes before, the searches, what it wrote, the check's
    /// markers.
    Route { net: usize, reroutes: u32, searches: Vec<Search>, figs: Vec<DrFig>, markers: Vec<Marker>, patches: Vec<PatchWire> },
    /// An owner checked: the markers.
    Check { owner: Owner, markers: Vec<Marker> },
    /// After the queue, the check over every owner: the worker's markers.
    Final { markers: Vec<Marker>, patches: Vec<PatchWire> },
    /// Area patches of one check dropped for want of a worker net (`!patch->hasNet()`: no worker
    /// net of the patch's design net meets the patch; divergence D4, OpenROAD issue #12191). Never
    /// zero.
    AreaPatchesDropped(usize),
    /// The check's view of an owner (`VYGD_GC`): `gcinit` / `gcupd` and its maximal rectangles.
    Gc(String),
    /// An entry pushed onto the queue (after its batch was sorted).
    Push { block: Block, num_reroute: i32, do_route: bool, checking: Option<Owner> },
    /// A check patch written to a worker net (`writeGCPatchesToDRWorker`): the net, the patch as
    /// written, and how many path points it could move to (0: not moved).
    Patch { net: usize, fig: DrFig, valid: usize },
    /// A via-swap entry queued at the start (`route_queue_init_queue`, VIASWAP): the worker net
    /// and the lonely via's def.
    SwapInit { net: usize, via: usize },
    /// A lonely via visited by a via-swap entry: the worker net, the entry's re-route count, the
    /// via's def and the def chosen for it, and whether the layer's secondaries ran out.
    Swap { net: usize, reroute: i32, old: usize, new: usize, no_solution: bool },
    /// A via-swap entry's figures after its swaps (the net's whole list, in order).
    SwapFigs { net: usize, figs: Vec<DrFig> },
    /// A via-swap entry's check: the worker net and the check's marker count; none: the net
    /// is kept (`setModified`).
    SwapCheck { net: usize, markers: usize },
}

/// What the queue reads beyond the costs.
pub struct QueueCtx<'a> {
    pub mcfg: &'a MazeCfg<'a>,
    pub route_box: Rect,
    /// The worker's check box (`getDrcBox`): the checker's `drc_box` (a minimum-area marker needs
    /// its polygon wholly inside).
    pub drc_box: Rect,
    /// Per worker net: its net's name, its routing context, its non-default rule.
    pub name: &'a dyn Fn(usize) -> String,
    pub net_ctx: &'a dyn Fn(usize) -> NetCtx<'a>,
    pub ndr: &'a dyn Fn(usize) -> Option<Ndr<'a>>,
    /// The ripups a net may avoid (100 for a clock net, 3 for a rule net, else 0).
    pub max_ripup_avoids: &'a dyn Fn(usize) -> u32,
    /// Whether a net (by name) is a supply net.
    pub is_supply: &'a dyn Fn(&str) -> bool,
    /// An instance's index (a check entry for its obstructions sorts by it).
    pub inst_index: &'a dyn Fn(&str) -> usize,
    /// The design's shapes in the extended box, owned and fixed.
    pub fixed: &'a [(Owner, usize, Rect)],
    /// Per worker net with a non-default rule: its spacing per z; the technology's largest.
    pub ndr_spacing: &'a dyn Fn(usize) -> Option<Vec<i32>>,
    pub max_ndr_spacing: &'a [i32],
    pub marker_decay: f32,
    pub maze_end_iter: u32,
    /// The worker's markers drive the queue (not everything ripped up): a reroute is dropped
    /// when its net and the owner that queued it were both last checked clean since the last
    /// route.
    pub markers_drive: bool,
    /// A worker net that may not be ripped up by a marker (an incremental run's nets routed
    /// before routing began).
    pub pinned: &'a dyn Fn(usize) -> bool,
}

/// The sort key of an owner or net: (kind, name, id), as the reroute queue orders them.
fn key(q: &QueueCtx<'_>, nets: &[DrNet], b: &Block) -> (i32, String, i64) {
    match b {
        Block::Net(i) => (28, (q.name)(*i), nets[*i].id as i64),
        Block::Owner(o) => owner_key(q, o),
    }
}

fn owner_key(q: &QueueCtx<'_>, o: &Owner) -> (i32, String, i64) {
    match o {
        Owner::Net(n) => (0, n.clone(), 0),
        Owner::FloatingGround => (0, "frFakeVSS".into(), -2),
        Owner::FloatingPower => (0, "frFakeVDD".into(), -1),
        Owner::BlockTerm(n) => (1, n.clone(), 0),
        Owner::Inst(n) => (3, String::new(), (q.inst_index)(n) as i64),
        // `frcInstBlockage` (19); its id is not set by io — the blockage's place stands in (no
        // corpus design carries one).
        Owner::InstBlockage(_, k, _, _) => (19, String::new(), *k as i64),
        Owner::InstTerm(i, t) => (7, format!("{i}/{t}"), 0),
        Owner::Blockage(b) => (14, String::new(), *b as i64),
    }
}

fn opt_key(q: &QueueCtx<'_>, o: &Option<Owner>) -> (i32, String, i64) {
    o.as_ref().map_or((-1, String::new(), -1), |o| owner_key(q, o))
}

struct NetState {
    reroutes: u32,
    ripup_avoids: u32,
    figs: Vec<DrFig>,
    /// The net's committed shapes around the route box.
    ext: Vec<DrFig>,
}

fn touches(a: &Rect, b: &Rect) -> bool {
    a.xh >= b.xl && a.xl <= b.xh && a.yh >= b.yl && a.yl <= b.yh
}

/// The worker's region query of its nets' shapes, which a marker's cost reads: bulk-loaded at the
/// start from every net (in order) — its route shapes (none when everything is ripped up), then
/// its committed shapes around the box — then each route's shapes inserted as it writes them and
/// removed, in the same order, when the net is ripped up. A via has an entry per rectangle of
/// each of its layers (below, above, cut). An entry names (net, committed?, index).
/// A region-query entry: (net, committed?, index in its list).
type RqRef = (usize, bool, usize);

#[derive(Default)]
struct RouteRq {
    trees: Vec<DynRTree<RqRef>>,
    ids: HashMap<usize, Vec<(usize, usize)>>,
}

impl RouteRq {
    /// A route shape's entries: a wire's box, a via's rectangles per layer, a patch's box.
    fn rects(tech: &crate::tech::Tech, f: &DrFig) -> Vec<(usize, Rect)> {
        let sh = |r: &Rect, o: (i32, i32)| Rect { xl: r.xl + o.0, yl: r.yl + o.1, xh: r.xh + o.0, yh: r.yh + o.1 };
        match *f {
            DrFig::Seg { layer, begin, end, width, begin_ext, end_ext, .. } => vec![(layer, DrFig::seg_box(begin, end, width, begin_ext, end_ext))],
            DrFig::Via { via, origin, .. } => {
                let vd = &tech.via_defs[via];
                let mut v: Vec<(usize, Rect)> = vd.layer1_figs.iter().map(|r| (vd.layer1, sh(r, origin))).collect();
                v.extend(vd.layer2_figs.iter().map(|r| (vd.layer2, sh(r, origin))));
                v.extend(vd.cut_figs.iter().map(|r| (vd.cut, sh(r, origin))));
                v
            }
            DrFig::Patch { layer, origin, offset } => vec![(layer, sh(&offset, origin))],
        }
    }

    /// The bulk load: per net, its route shapes, then its committed ones.
    fn new(tech: &crate::tech::Tech, state: &[NetState]) -> RouteRq {
        let mut per: Vec<Vec<(Rect, RqRef)>> = Vec::new();
        // A route shape's entries are ripped up with its net (a bulk-loaded entry's id is its place
        // in its layer's load).
        let mut ids: HashMap<usize, Vec<(usize, usize)>> = HashMap::new();
        for (ni, s) in state.iter().enumerate() {
            for (ext, figs) in [(false, &s.figs), (true, &s.ext)] {
                for (k, f) in figs.iter().enumerate() {
                    for (l, r) in Self::rects(tech, f) {
                        while per.len() <= l {
                            per.push(Vec::new());
                        }
                        if !ext {
                            ids.entry(ni).or_default().push((l, per[l].len()));
                        }
                        per[l].push((r, (ni, ext, k)));
                    }
                }
            }
        }
        let n = tech.layers.len().max(per.len());
        per.resize(n, Vec::new());
        RouteRq { trees: per.into_iter().map(DynRTree::new).collect(), ids }
    }

    fn add_net(&mut self, tech: &crate::tech::Tech, net: usize, figs: &[DrFig]) {
        for (k, f) in figs.iter().enumerate() {
            for (l, r) in Self::rects(tech, f) {
                while self.trees.len() <= l {
                    self.trees.push(DynRTree::new(Vec::new()));
                }
                let id = self.trees[l].insert(r, (net, false, k));
                self.ids.entry(net).or_default().push((l, id));
            }
        }
    }

    /// One shape added to a net's list at index `k` (a written check patch).
    fn add_fig(&mut self, tech: &crate::tech::Tech, net: usize, k: usize, f: &DrFig) {
        for (l, r) in Self::rects(tech, f) {
            while self.trees.len() <= l {
                self.trees.push(DynRTree::new(Vec::new()));
            }
            let id = self.trees[l].insert(r, (net, false, k));
            self.ids.entry(net).or_default().push((l, id));
        }
    }

    /// A via swap (`workerRegionQuery.remove(old)`, `net->removeShape(old)`, `net->addRoute(new)`,
    /// `workerRegionQuery.add(new)`): the shape at `pos` of the net's list taken out of the query
    /// (its rectangles in the order they went in), the list closing over it (later shapes move
    /// down one), the new shape added at the list's end `last`, its rectangles inserted.
    fn swap_fig(&mut self, tech: &crate::tech::Tech, net: usize, pos: usize, last: usize, new: &DrFig) {
        let ids = self.ids.remove(&net).unwrap_or_default();
        let mut kept = Vec::with_capacity(ids.len());
        for (l, id) in ids {
            let (n, ext, k) = self.trees[l].value(id).1;
            if k == pos {
                self.trees[l].remove(id);
                continue;
            }
            if k > pos {
                self.trees[l].set_payload(id, (n, ext, k - 1));
            }
            kept.push((l, id));
        }
        self.ids.insert(net, kept);
        self.add_fig(tech, net, last, new);
    }

    fn remove_net(&mut self, net: usize) {
        for (l, id) in self.ids.remove(&net).unwrap_or_default() {
            self.trees[l].remove(id);
        }
    }

    /// The route shapes on layer `l` touching `b`, in query order (a via as often as its
    /// rectangles touch).
    fn query(&self, l: usize, b: &Rect) -> Vec<RqRef> {
        self.trees.get(l).map_or(Vec::new(), |t| t.query(b).into_iter().map(|(_, v)| v.1).collect())
    }
}

/// The marker costs added so far (planar, via nodes), which decay after each route; and the
/// worker's region query the queue goes on with.
pub struct History {
    planar: BTreeSet<Idx>,
    via: BTreeSet<Idx>,
    rq: RouteRq,
}

/// Before the queue: the markers standing in the worker's check box cost the grid.
pub fn initial_marker_cost(w: &mut CostWorker<'_, '_>, q: &QueueCtx<'_>, nets: &[DrNet], markers: &[Marker]) -> History {
    let state: Vec<NetState> = nets.iter().map(|n| NetState { reroutes: 0, ripup_avoids: 0, figs: n.route.clone(), ext: n.ext.clone() }).collect();
    let mut h = History { planar: BTreeSet::new(), via: BTreeSet::new(), rq: RouteRq::new(q.mcfg.tech, &state) };
    for m in markers {
        add_marker_cost(w, q, &state, &h.rq, m, &mut h.planar, &mut h.via);
    }
    h
}

/// How a worker's queue starts: every net in a routing order (everything ripped up), or the
/// worker's markers (their victims and aggressors, as a check's markers requeue them).
pub enum Start<'a> {
    Nets(&'a [usize]),
    Markers(&'a [Marker]),
    /// `RipUpMode::VIASWAP` (the LEF58_MAXSPACING pass's workers): a check entry per lonely via
    /// of each worker net; per cut layer, its secondary via defs in order.
    ViaSwap(&'a [Vec<usize>]),
}

/// A worker whose check box holds a re-check marker: a check over everything it holds, before
/// the queue — its markers replace the worker's.
pub fn recheck_markers(q: &QueueCtx<'_>, nets: &[DrNet]) -> Vec<Marker> {
    let state: Vec<NetState> = nets.iter().map(|n| NetState { reroutes: 0, ripup_avoids: 0, figs: n.route.clone(), ext: n.ext.clone() }).collect();
    let mut gw = check_init(q, nets, &state, None);
    gw.target = None;
    gw.run().to_vec()
}

/// Run a worker's queue.
pub fn route_queue(w: &mut CostWorker<'_, '_>, st: &mut MazeState, q: &QueueCtx<'_>, nets: &[DrNet], start: Start<'_>, hist: History) -> Vec<Event> {
    let mut events = Vec::new();
    let mut state: Vec<NetState> = nets.iter().map(|n| NetState { reroutes: 0, ripup_avoids: 0, figs: n.route.clone(), ext: n.ext.clone() }).collect();
    let by_name: HashMap<String, Vec<usize>> = {
        let mut m: HashMap<String, Vec<usize>> = HashMap::new();
        for i in 0..nets.len() {
            m.entry((q.name)(i)).or_default().push(i);
        }
        m
    };
    let mut queue: VecDeque<Entry> = VecDeque::new();
    match start {
        Start::Nets(order) => queue.extend(order.iter().map(|&i| Entry { block: Block::Net(i), num_reroute: 0, do_route: true, checking: None })),
        Start::Markers(markers) => {
            update_queue(q, nets, &mut state, &by_name, markers, &mut queue, None);
            for e in &queue {
                events.push(Event::Push { block: e.block.clone(), num_reroute: e.num_reroute, do_route: e.do_route, checking: e.checking.clone() });
            }
        }
        Start::ViaSwap(_) => {
            for e in via_swap_init_queue(q, nets, &state, &mut events) {
                events.push(Event::Push { block: e.block.clone(), num_reroute: e.num_reroute, do_route: e.do_route, checking: e.checking.clone() });
                queue.push_back(e);
            }
        }
    }
    let secondary: Option<&[Vec<usize>]> = match start {
        Start::ViaSwap(s) => Some(s),
        _ => None,
    };
    // VIASWAP (upstream fix 0013, OpenROAD issue #12203, register D11): each swapped via's def before its first swap,
    // by (worker net, origin, cut layer) — upstream keys it by the via object, which a swap
    // replaces; and the worker nets whose check update is pending (`updateDRNet`, applied at the
    // next check).
    let mut swap_orig: HashMap<(usize, (i32, i32), usize), usize> = HashMap::new();
    let mut pending: Vec<usize> = Vec::new();
    let gc_dump = std::env::var("VYGD_GC").is_ok();
    let mut gw = check_init(q, nets, &state, if gc_dump { Some(&mut events) } else { None });
    let mut gc_version = 1i64;
    // Per owner checked (or net routed): the version it was checked at and its marker count; the
    // last check's marker count (what a check that runs nothing records).
    let mut checked: HashMap<Block, (i64, usize)> = HashMap::new();
    let mut last_markers = 0usize;
    let History { planar: mut planar_hist, via: mut via_hist, mut rq } = hist;
    while let Some(e) = queue.pop_front() {
        if let (Some(sec), Block::Net(i), false) = (secondary, &e.block, e.do_route) {
            let i = *i;
            let no_solution = swap_lonely_vias(q, sec, &mut state, &mut rq, &mut swap_orig, i, e.num_reroute, &mut events);
            events.push(Event::SwapFigs { net: i, figs: state[i].figs.clone() });
            pending.push(i);
            if no_solution {
                continue;
            }
            flush_pending(&mut gw, q, nets, &state, &mut pending);
            gw.target = Some(Owner::Net((q.name)(i)));
            // `setTargetNet(frNet)`: no worker net is the target (`targetDRNet_` stays null).
            let (m, pw, dropped) = gc_main_surgical(&mut gw, q, nets, &state, &rq, None, true);
            if dropped > 0 {
                events.push(Event::AreaPatchesDropped(dropped));
            }
            events.push(Event::SwapCheck { net: i, markers: m.len() });
            if m.is_empty() {
                events.extend(write_gc_patches(q, &mut state, &mut rq, &pw, &[]));
            } else {
                let next = Entry { block: Block::Net(i), num_reroute: e.num_reroute + 1, do_route: false, checking: None };
                events.push(Event::Push { block: next.block.clone(), num_reroute: next.num_reroute, do_route: false, checking: None });
                queue.push_back(next);
            }
            continue;
        }
        let mut did_route = false;
        let (markers, checking_obj): (Vec<Marker>, Owner) = match (&e.block, e.do_route) {
            (Block::Net(i), true) => {
                let i = *i;
                if e.num_reroute != state[i].reroutes as i32 {
                    continue;
                }
                if q.markers_drive {
                    if let Some(c) = &e.checking {
                        let clean = |b: &Block| checked.get(b) == Some(&(gc_version, 0));
                        if clean(&Block::Owner(Owner::Net((q.name)(i)))) && clean(&Block::Owner(c.clone())) {
                            continue;
                        }
                    }
                }
                let cx = (q.net_ctx)(i);
                let ndr = (q.ndr)(i);
                let old = std::mem::take(&mut state[i].figs);
                rq.remove_net(i);
                let siblings = sibling_metal(q.mcfg.tech, &state, &|j| (q.name)(j), i);
                let (searches, figs) = reroute_net(w, st, q.mcfg, &nets[i], ndr, &cx, state[i].reroutes, &old, &siblings);
                rq.add_net(q.mcfg.tech, i, &figs);
                maze_net_end(w, st, &nets[i], &cx);
                crate::dr::route::edge_watch(w, "after-maze-net-end");
                gc_version += 1;
                let reroutes = state[i].reroutes;
                state[i].reroutes += 1;
                state[i].figs = figs.clone();
                did_route = true;
                let owner = Owner::Net((q.name)(i));
                let (route, taper) = owner_route(q, nets, &state, &owner);
                gw.replace_route(&owner, &route, &taper);
                if gc_dump {
                    events.push(Event::Gc(format!("gcupd|{}", owner_tag(&owner)) + &gw.dump(&owner)));
                }
                gw.target = Some(owner.clone());
                let (m, pw, dropped) = gc_main_surgical(&mut gw, q, nets, &state, &rq, Some(i), false);
                if dropped > 0 {
                    events.push(Event::AreaPatchesDropped(dropped));
                }
                // `modEolCosts_poly` reads the check's net, which already holds the new patches.
                let mut with_patches = figs.clone();
                with_patches.extend(pw.iter().map(|(_, p)| DrFig::Patch { layer: p.layer, origin: p.origin, offset: p.offset }));
                after_check(w, &nets[i], &with_patches, cx.ext_box, &siblings);
                crate::dr::route::edge_watch(w, "after-check-eol");
                // The patches go to the net at the point of its paths nearest each.
                let points: Vec<(i32, i32)> = searches.iter().flat_map(|s| s.path.iter()).map(|m| (w.g.xs[m.0 as usize], w.g.ys[m.1 as usize])).collect();
                let written = write_gc_patches(q, &mut state, &mut rq, &pw, &points);
                crate::dr::route::edge_watch(w, "after-write-patches");
                checked.insert(Block::Owner(owner.clone()), (gc_version, m.len()));
                last_markers = m.len();
                events.push(Event::Route { net: i, reroutes, searches, figs, markers: m.clone(), patches: pw.iter().map(|(_, p)| p.clone()).collect() });
                events.extend(written);
                (m, owner)
            }
            (b, _) => {
                if checked.get(b).map(|v| v.0) == Some(gc_version) {
                    continue;
                }
                let owner = match b {
                    // ⛔ A worker net queued to check (it avoided a ripup) names the worker net,
                    // which the check does not know: nothing is checked.
                    Block::Net(_) => {
                        checked.insert(b.clone(), (gc_version, last_markers));
                        continue;
                    }
                    Block::Owner(o) => o.clone(),
                };
                let m = check(&mut gw, &owner);
                checked.insert(b.clone(), (gc_version, m.len()));
                last_markers = m.len();
                events.push(Event::Check { owner: owner.clone(), markers: m.clone() });
                (m, owner)
            }
        };
        let before = queue.len();
        update_queue(q, nets, &mut state, &by_name, &markers, &mut queue, Some(checking_obj));
        for e in queue.iter().skip(before) {
            events.push(Event::Push { block: e.block.clone(), num_reroute: e.num_reroute, do_route: e.do_route, checking: e.checking.clone() });
        }
        if did_route {
            marker_cost_decay(w, q.marker_decay, &mut planar_hist, &mut via_hist);
        }
        for mk in &markers {
            add_marker_cost(w, q, &state, &rq, mk, &mut planar_hist, &mut via_hist);
        }
        crate::dr::route::edge_watch(w, "after-marker-cost");
    }
    gw.target = None;
    flush_pending(&mut gw, q, nets, &state, &mut pending);
    let (m, pw, dropped) = gc_main_surgical(&mut gw, q, nets, &state, &rq, None, secondary.is_some());
    if dropped > 0 {
        events.push(Event::AreaPatchesDropped(dropped));
    }
    events.push(Event::Final { markers: m, patches: pw.iter().map(|(_, p)| p.clone()).collect() });
    events.extend(write_gc_patches(q, &mut state, &mut rq, &pw, &[]));
    events
}

/// `route_queue_init_queue`, VIASWAP: per worker net in worker order, per lonely via of its route
/// in list order, a check entry on the net (re-route 0, its checking object the net itself) —
/// one entry PER lonely via, so a net with k of them is queued k times; then
/// `mazeIterInit_sortRerouteQueue` and `route_queue_update_queue` sort them by the net's key
/// (equal entries are alike, so the sort's stability does not show).
fn via_swap_init_queue(q: &QueueCtx<'_>, nets: &[DrNet], state: &[NetState], events: &mut Vec<Event>) -> Vec<Entry> {
    let mut checks = Vec::new();
    for (i, s) in state.iter().enumerate() {
        for f in &s.figs {
            if let DrFig::Via { via, lonely: true, .. } = f {
                events.push(Event::SwapInit { net: i, via: *via });
                checks.push(Entry { block: Block::Net(i), num_reroute: 0, do_route: false, checking: None });
            }
        }
    }
    checks.sort_by_cached_key(|e| key(q, nets, &e.block));
    checks
}

/// `route_queue_main`, VIASWAP, the swap: each lonely via of the net's route list is visited and
/// given the layer's secondary via def at the entry's re-route count — or, when the layer has no
/// more (`size() <= numReroute`), the def it had before its first swap (upstream fix 0013, OpenROAD issue #12203: the
/// reference takes the layer's DEFAULT via def there and leaves the check on the last swap,
/// register D11). A changed via is erased from its place and appended to the list's end while the
/// list is being walked BY POSITION (a range-for over the `std::vector`, its end fixed): the
/// shape after it moves into its place and is skipped, and the appended via is visited again at
/// the end (its def unchanged then). Whether a layer's secondaries ran out.
#[allow(clippy::too_many_arguments)]
fn swap_lonely_vias(q: &QueueCtx<'_>, secondary: &[Vec<usize>], state: &mut [NetState], rq: &mut RouteRq, orig: &mut HashMap<(usize, (i32, i32), usize), usize>, i: usize, reroute: i32, events: &mut Vec<Event>) -> bool {
    let tech = q.mcfg.tech;
    let n = state[i].figs.len();
    let mut visits = Vec::new();
    let mut swaps = Vec::new();
    let no_solution = swap_walk(&mut state[i].figs, &|v| tech.via_defs[v].cut, secondary, orig, i, reroute, &mut visits, &mut swaps);
    for (old, new, ran_out) in visits {
        events.push(Event::Swap { net: i, reroute, old, new, no_solution: ran_out });
    }
    for (pos, fig) in swaps {
        rq.swap_fig(tech, i, pos, n - 1, &fig);
    }
    no_solution
}

/// The walk of [`swap_lonely_vias`] over one net's list: each visit (old def, new def, whether
/// the layer ran out) and each swap (the place it left, the via appended), in order.
#[allow(clippy::too_many_arguments)]
fn swap_walk(figs: &mut Vec<DrFig>, cut_of: &dyn Fn(usize) -> usize, secondary: &[Vec<usize>], orig: &mut HashMap<(usize, (i32, i32), usize), usize>, i: usize, reroute: i32, visits: &mut Vec<(usize, usize, bool)>, swaps: &mut Vec<(usize, DrFig)>) -> bool {
    let mut no_solution = false;
    for pos in 0..figs.len() {
        let DrFig::Via { via, origin, lonely: true, .. } = figs[pos] else { continue };
        let cut = cut_of(via);
        let sec = secondary.get(cut).map_or(&[][..], Vec::as_slice);
        let key = (i, origin, cut);
        let first = orig.get(&key).copied().unwrap_or(via);
        let (new, ran_out) = match usize::try_from(reroute).ok().and_then(|r| sec.get(r)) {
            Some(&v) => (v, false),
            None => (first, true),
        };
        no_solution |= ran_out;
        visits.push((via, new, ran_out));
        if new == via {
            continue;
        }
        orig.insert(key, first);
        let mut fig = figs.remove(pos);
        if let DrFig::Via { via, .. } = &mut fig {
            *via = new;
        }
        figs.push(fig.clone());
        swaps.push((pos, fig));
    }
    no_solution
}

/// The pending `updateDRNet`s applied, as the next check's `updateGCWorker` applies them: once
/// per design net, by id.
fn flush_pending(gw: &mut Worker<'_>, q: &QueueCtx<'_>, nets: &[DrNet], state: &[NetState], pending: &mut Vec<usize>) {
    let mut ids: Vec<(usize, Owner)> = Vec::new();
    for &i in pending.iter() {
        let o = Owner::Net((q.name)(i));
        if !ids.iter().any(|(_, x)| *x == o) {
            ids.push((nets[i].net, o));
        }
    }
    pending.clear();
    ids.sort_by_key(|(id, _)| *id);
    let batch: Vec<_> = ids
        .into_iter()
        .map(|(_, owner)| {
            let (route, taper) = owner_route(q, nets, state, &owner);
            (owner, route, taper)
        })
        .collect();
    if !batch.is_empty() {
        gw.replace_routes(&batch);
    }
}

/// `FlexGCWorker::main` with the surgical fix (the worker's own checks): `checkMetalShape(true)`
/// — with a target, its minimum-area and LEF58_AREA patches ([`Worker::area_patches`], each
/// given its worker net and kept when valid by [`area_patch_owner`]); then, when the technology
/// has corner spacing, `patchMetalShape` — `checkMetalCornerSpacing`, then
/// `patchMetalShape_cornerSpacing` — on the check as it stood before any patch; and the patched
/// nets updated with their patches (`updateGCWorker`); then the full check and `modifyMarkers`.
/// `target`: the worker net checked (`setTargetNet(drNet)`), none after the queue. The check's
/// markers and patches, and the count of area patches dropped for want of a worker net.
/// `patchMetalShape` starts with `patchMetalShape_minStep` ([`patch_min_step`]) on the
/// minimum-step markers of `checkMetalShape(true)`, when the technology keeps a MINSTEP rule
/// (`hasCornerSpacingConstraint()`, which only a MINSTEP rule sets, or `hasVia2ViaMinStep()`);
/// it patches only in a via-swap worker (`via_swap`).
#[allow(clippy::too_many_arguments)]
fn gc_main_surgical(gw: &mut Worker<'_>, q: &QueueCtx<'_>, nets: &[DrNet], state: &[NetState], rq: &RouteRq, target: Option<usize>, via_swap: bool) -> (Vec<Marker>, Vec<(usize, PatchWire)>, usize) {
    let (mut pw, dropped) = own_area_patches(gw.area_patches(), &|p| area_patch_owner(q, nets, state, p), &q.route_box);
    if q.mcfg.tech.layers.iter().any(|l| l.min_step.is_some()) {
        let markers = gw.min_step_markers();
        pw.extend(patch_min_step(q, state, rq, &markers, gw.target.as_ref(), via_swap));
    }
    if patches_corners(&q.mcfg.tech.layers) {
        let markers = gw.corner_markers();
        pw.extend(patch_corner_spacing(q, state, rq, &markers, target));
    }
    if !pw.is_empty() {
        update_patched(gw, q, nets, state, &pw);
    }
    let patches: Vec<PatchWire> = pw.iter().map(|(_, p)| p.clone()).collect();
    (gw.run_patched(&patches).to_vec(), pw, dropped)
}

/// `checkMetalShape_addPatch`, its end, per area patch in order: the worker net (`owner`), then
/// [`is_patch_valid`]. The patches kept with their worker nets, and how many were dropped for
/// want of a worker net — the reference drops them silently; the run reports the count (with the
/// absolute-box lookup of divergence D4 this should not happen; OpenROAD issue #12191). An
/// invalid patch is dropped too but not counted.
fn own_area_patches(patches: Vec<crate::gc::AreaPatch>, owner: &dyn Fn(&crate::gc::AreaPatch) -> Option<usize>, rb: &Rect) -> (Vec<(usize, PatchWire)>, usize) {
    let mut kept = Vec::new();
    let mut dropped = 0;
    for p in patches {
        let Some(ni) = owner(&p) else {
            dropped += 1;
            continue;
        };
        let pwire = PatchWire { layer: p.layer, origin: p.origin, offset: p.offset, owner: p.owner };
        if is_patch_valid(&pwire, rb) {
            kept.push((ni, pwire));
        }
    }
    (kept, dropped)
}

/// `gc_patch::isPatchValid`: a patch is kept when its box touches the route box (edges count) or
/// the route box holds its origin (boundary included).
fn is_patch_valid(p: &PatchWire, rb: &Rect) -> bool {
    let o = p.origin;
    touches(&p.bbox(), rb) || (rb.xl <= o.0 && o.0 <= rb.xh && rb.yl <= o.1 && o.1 <= rb.yh)
}

/// `checkMetalShape_addPatch`, its second half: the patch's worker net. The design net's only
/// worker net; with several, `checkMetalShape_patchOwner_helper` — the first (worker order) whose
/// pin box, else one of its boundary shapes, else one of its route shapes (each in order) meets
/// the patch's box.
///
/// ⛔ A DIVERGENCE from the reference by decision (drt divergence register D4; OpenROAD issue
/// #12191). Upstream rule: that box is `patch->getOffsetBox()`, the box RELATIVE to the patch's
/// origin, not where the patch is, and it must strictly overlap (`Rect::overlaps`), so a match
/// needs a shape near the origin of the design; without one the patch is dropped silently. Our
/// rule, the reference patched with vyges-openroad-patches upstream-fix 0007 v2: the patch's
/// absolute box (`getBBox()`), touching included (`intersects`), since a patch abuts the shape
/// it grows from. A patch still without a worker net is dropped and counted ([`own_area_patches`]).
fn area_patch_owner(q: &QueueCtx<'_>, nets: &[DrNet], state: &[NetState], p: &crate::gc::AreaPatch) -> Option<usize> {
    let Owner::Net(name) = &p.owner else { return None };
    let mine: Vec<usize> = (0..nets.len()).filter(|&i| (q.name)(i) == *name).collect();
    if mine.len() == 1 {
        return Some(mine[0]);
    }
    let tech = q.mcfg.tech;
    patch_owner_helper(patch_box(p), mine.into_iter().map(|i| (i, nets[i].pin_box, state[i].ext.iter().chain(&state[i].figs).map(|f| fig_bbox(tech, f)).collect())))
}

/// The patch's absolute box (`drPatchWire::getBBox()`): its offset box moved to its origin. The
/// unpatched reference compares `getOffsetBox()`, the box relative to the origin (D4).
fn patch_box(p: &crate::gc::AreaPatch) -> Rect {
    let o = p.origin;
    Rect::new(p.offset.xl + o.0, p.offset.yl + o.1, p.offset.xh + o.0, p.offset.yh + o.1)
}

/// `checkMetalShape_patchOwner_helper` on the patch's absolute box (divergence D4, upstream-fix
/// 0007 v2): the first candidate worker net, in order, whose pin box, then one of its shapes'
/// boxes (boundary shapes, then route shapes, in order) meets it (`Rect::intersects`: closed
/// boxes, so touching counts).
fn patch_owner_helper(b: Rect, cands: impl Iterator<Item = (usize, Rect, Vec<Rect>)>) -> Option<usize> {
    let meets = |r: &Rect| r.xh >= b.xl && r.xl <= b.xh && r.yh >= b.yl && r.yl <= b.yh;
    for (i, pin_box, figs) in cands {
        if meets(&pin_box) || figs.iter().any(meets) {
            return Some(i);
        }
    }
    None
}

/// A route shape's box (`drConnFig::getBBox`): a wire's, a via's over its three layers, a patch's.
fn fig_bbox(tech: &crate::tech::Tech, f: &DrFig) -> Rect {
    match *f {
        DrFig::Seg { begin, end, width, begin_ext, end_ext, .. } => DrFig::seg_box(begin, end, width, begin_ext, end_ext),
        DrFig::Via { via, origin, .. } => {
            let vd = &tech.via_defs[via];
            let all: Vec<&Rect> = vd.layer1_figs.iter().chain(&vd.cut_figs).chain(&vd.layer2_figs).collect();
            let b = all.iter().skip(1).fold(*all[0], |a, f| Rect::new(a.xl.min(f.xl), a.yl.min(f.yl), a.xh.max(f.xh), a.yh.max(f.yh)));
            Rect::new(b.xl + origin.0, b.yl + origin.1, b.xh + origin.0, b.yh + origin.1)
        }
        DrFig::Patch { origin, offset, .. } => Rect::new(offset.xl + origin.0, offset.yl + origin.1, offset.xh + origin.0, offset.yh + origin.1),
    }
}

/// The corner-spacing patch gate: the technology has corner spacing. DIVERGES from the reference
/// on purpose (register D12, upstream OpenROAD issue #12207, fix 0014): the reference's flag for this gate,
/// `hasCornerSpacingConstraint_`, is set by MINSTEP rules, not by corner spacing, so it corner-patches
/// only a technology that also has a MINSTEP rule. Every technology with both kinds of rule, the
/// only kind the reference's gate admits, patches the same.
fn patches_corners(layers: &[crate::tech::Layer]) -> bool {
    layers.iter().any(|l| !l.corner_spacing.is_empty())
}

/// `patchMetalShape_cornerSpacing`: per corner-spacing marker, the first route shape of a source
/// net in the worker's region query of the marker's layer (in query order; with a target, only
/// the target worker net's) whose point lies in the route box — a via (its origin; its box on the
/// layer), a wire (its end nearer the marker), or a patch (its origin; ⚠️ a patch does not end
/// the search, a later via or wire replaces it). The marker box, stretched across the layer to
/// the shape's box and one manufacturing grid further along it (away from the shape when the
/// shape starts at the marker's far side), becomes a patch there — kept when its box touches the
/// route box or the route box holds its origin (`isPatchValid`).
fn patch_corner_spacing(q: &QueueCtx<'_>, state: &[NetState], rq: &RouteRq, markers: &[Marker], target: Option<usize>) -> Vec<(usize, PatchWire)> {
    let tech = q.mcfg.tech;
    let rb = q.route_box;
    let in_rb = |p: (i32, i32)| rb.xl <= p.0 && p.0 <= rb.xh && rb.yl <= p.1 && p.1 <= rb.yh;
    let shift = |r: &Rect, o: (i32, i32)| Rect::new(r.xl + o.0, r.yl + o.1, r.xh + o.0, r.yh + o.1);
    let mut out = Vec::new();
    for m in markers.iter().filter(|m| m.rule == Rule::CornerSpacing) {
        let l = m.layer;
        let mb0 = m.bbox;
        let mut chosen: Option<(usize, (i32, i32), Rect)> = None;
        for (ni, ext, k) in rq.query(l, &mb0) {
            let name = (q.name)(ni);
            if !m.owners.contains(&Owner::Net(name.clone())) {
                continue;
            }
            if let Some(t) = target {
                if (q.name)(t) != name || ni != t {
                    continue;
                }
            }
            match *if ext { &state[ni].ext[k] } else { &state[ni].figs[k] } {
                DrFig::Via { via, origin, .. } => {
                    if in_rb(origin) {
                        let vd = &tech.via_defs[via];
                        let figs = if vd.layer1 == l { &vd.layer1_figs } else { &vd.layer2_figs };
                        let b = figs.iter().skip(1).fold(figs[0], |a, f| Rect::new(a.xl.min(f.xl), a.yl.min(f.yl), a.xh.max(f.xh), a.yh.max(f.yh)));
                        chosen = Some((ni, origin, shift(&b, origin)));
                        break;
                    }
                }
                DrFig::Seg { begin, end, width, begin_ext, end_ext, .. } => {
                    let dist = |p: (i32, i32)| (p.0.clamp(mb0.xl, mb0.xh) - p.0).abs() + (p.1.clamp(mb0.yl, mb0.yh) - p.1).abs();
                    let o = if dist(begin) < dist(end) { begin } else { end };
                    if in_rb(o) {
                        chosen = Some((ni, o, DrFig::seg_box(begin, end, width, begin_ext, end_ext)));
                        break;
                    }
                }
                DrFig::Patch { origin, offset, .. } => {
                    if in_rb(origin) {
                        chosen = Some((ni, origin, shift(&offset, origin)));
                    }
                }
            }
        }
        let Some((ni, origin, fb)) = chosen else { continue };
        let mb = stretch_corner_patch(mb0, fb, tech.layers[l].is_horizontal(), tech.manufacturing_grid);
        let p = PatchWire { layer: l, origin, offset: Rect::new(mb.xl - origin.0, mb.yl - origin.1, mb.xh - origin.0, mb.yh - origin.1), owner: Owner::Net((q.name)(ni)) };
        if is_patch_valid(&p, &rb) {
            out.push((ni, p));
        }
    }
    out
}

/// Per via origin (`std::map<odb::Point, std::vector<drVia*>>`), its vias: worker net, enclosure
/// box on the layer, lonely.
type ViasByOrigin = BTreeMap<(i32, i32), Vec<(usize, Rect, bool)>>;

/// `patchMetalShape_minStep`: per minimum-step marker, in order, on a routing layer beside a cut
/// layer that keeps a LEF58_MAXSPACING rule (the via-to-via half of its condition is refused
/// before routing, `min_step_via`) and at least the layer's width long on one side: the vias of
/// the marker's first source net (and the target's, with one) in the worker's region query of
/// the layer, cut just below or above it, grouped by origin (`std::map<Point>`: x, then y; each
/// group in query order). In a via-swap worker, a LONELY via whose enclosure box on the layer
/// spans the marker across (a horizontal box, `Rect::getDir`, dx >= dy: the marker's y range
/// equal to the box's; a vertical one: its x range) and shares one end of it along, takes the
/// patch: the marker box grown at that end by the MINSTEP length less its own length (never
/// less than nothing). ⚠️ Upstream rules kept as they are: `break` leaves the group only, so a
/// later origin's lonely via may take the marker again, from the box already grown; a layer with
/// no classic MINSTEP (`getMinStepConstraint()` null) skips the via (`continue`). The patch sits
/// at the via's origin on its worker net, kept when valid (`isPatchValid`).
fn patch_min_step(q: &QueueCtx<'_>, state: &[NetState], rq: &RouteRq, markers: &[Marker], target: Option<&Owner>, via_swap: bool) -> Vec<(usize, PatchWire)> {
    let tech = q.mcfg.tech;
    let max_spacing = |l: Option<usize>| l.and_then(|l| tech.layers.get(l)).is_some_and(|c| !c.lef58_max_spacing.is_empty());
    let mut out = Vec::new();
    for m in markers.iter().filter(|m| m.rule == Rule::MinStep) {
        let l = m.layer;
        let layer = &tech.layers[l];
        if !max_spacing(l.checked_sub(1)) && !max_spacing(Some(l + 1)) {
            continue;
        }
        let mut mb = m.bbox;
        if mb.dx().max(mb.dy()) < layer.width {
            continue;
        }
        let Some(Owner::Net(src)) = m.owners.first() else { continue };
        // `std::map<odb::Point, std::vector<drVia*>>`: per via, its worker net, enclosure, lonely.
        let mut vias: ViasByOrigin = BTreeMap::new();
        for (ni, ext, k) in rq.query(l, &mb) {
            let fig = if ext { &state[ni].ext[k] } else { &state[ni].figs[k] };
            let DrFig::Via { via, origin, lonely, .. } = *fig else { continue };
            let name = (q.name)(ni);
            if name != *src || target.is_some_and(|t| *t != Owner::Net(name.clone())) {
                continue;
            }
            let cut = tech.via_defs[via].cut;
            if cut == l + 1 || cut + 1 == l {
                vias.entry(origin).or_default().push((ni, via_enclosure(tech, via, l, origin), lonely));
            }
        }
        let min_step_length = layer.min_step.map(|ms| ms.min_step_length);
        let (grown, chosen) = min_step_take(mb, &vias, min_step_length, via_swap);
        let Some((ni, origin)) = chosen else { continue };
        mb = grown;
        let p = PatchWire { layer: l, origin, offset: Rect::new(mb.xl - origin.0, mb.yl - origin.1, mb.xh - origin.0, mb.yh - origin.1), owner: Owner::Net((q.name)(ni)) };
        if is_patch_valid(&p, &q.route_box) {
            out.push((ni, p));
        }
    }
    out
}

/// `patchMetalShape_minStep`'s walk over the vias of one marker, by origin (`std::map` order) and
/// in each group in order: the first lonely via in a via-swap worker that the marker box grows
/// against ([`min_step_bloat`]) ends its GROUP (`break`), not the walk; a later group's may take
/// the marker again from the box already grown, and the last one taken wins. Without a classic
/// MINSTEP length (`getMinStepConstraint()` null) no via takes it. The box as grown, and the
/// (worker net, origin) that took it last.
#[allow(clippy::type_complexity)]
fn min_step_take(mut mb: Rect, vias: &ViasByOrigin, min_step_length: Option<i32>, via_swap: bool) -> (Rect, Option<(usize, (i32, i32))>) {
    let mut chosen = None;
    for (&origin, objs) in vias {
        for &(ni, enc, lonely) in objs {
            if !(lonely && via_swap) {
                continue;
            }
            let Some(len) = min_step_length else { continue };
            let Some(grown) = min_step_bloat(mb, enc, len) else { continue };
            mb = grown;
            chosen = Some((ni, origin));
            break;
        }
    }
    (mb, chosen)
}

/// A via's enclosure box on routing layer `l` (`getLayer1BBox` when its bottom layer is `l`, else
/// `getLayer2BBox`), at its origin.
fn via_enclosure(tech: &crate::tech::Tech, via: usize, l: usize, origin: (i32, i32)) -> Rect {
    let vd = &tech.via_defs[via];
    let figs = if vd.layer1 == l { &vd.layer1_figs } else { &vd.layer2_figs };
    let b = figs.iter().skip(1).fold(figs[0], |a, f| Rect::new(a.xl.min(f.xl), a.yl.min(f.yl), a.xh.max(f.xh), a.yh.max(f.yh)));
    Rect::new(b.xl + origin.0, b.yl + origin.1, b.xh + origin.0, b.yh + origin.1)
}

/// `patchMetalShape_minStep`'s growth of a marker box against a lonely via's enclosure `enc`: a
/// horizontal enclosure (`getDir`: dx >= dy) whose y range the marker shares, grown to the right
/// when the marker ends at the enclosure's right end (and starts inside it), else to the left
/// when it starts at its left end (and ends inside it); a vertical one the same way in y. The
/// growth is MINSTEP less the marker's length along, at least 0. `None`: the via does not take
/// the marker.
fn min_step_bloat(mb: Rect, enc: Rect, min_step_length: i32) -> Option<Rect> {
    let mut r = mb;
    if enc.dx() >= enc.dy() && mb.yl == enc.yl && mb.yh == enc.yh {
        let grow = (min_step_length - mb.dx()).max(0);
        if mb.xl >= enc.xl && mb.xh == enc.xh {
            r.xh += grow;
        } else if mb.xl == enc.xl && mb.xh <= enc.xh {
            r.xl -= grow;
        } else {
            return None;
        }
    } else if enc.dx() < enc.dy() && mb.xl == enc.xl && mb.xh == enc.xh {
        let grow = (min_step_length - mb.dy()).max(0);
        if mb.yl >= enc.yl && mb.yh == enc.yh {
            r.yh += grow;
        } else if mb.yl == enc.yl && mb.yh <= enc.yh {
            r.yl -= grow;
        } else {
            return None;
        }
    } else {
        return None;
    }
    Some(r)
}

/// The corner patch's box: the marker box across the layer's direction as the shape's box, and one
/// manufacturing grid further along it — toward the low side when the shape starts exactly at the
/// marker's high side, else toward the high side.
fn stretch_corner_patch(marker: Rect, fig: Rect, horizontal: bool, grid: i32) -> Rect {
    let mut mb = marker;
    if horizontal {
        mb.yl = fig.yl;
        mb.yh = fig.yh;
        if fig.xl == mb.xh {
            mb.xl -= grid;
        } else {
            mb.xh += grid;
        }
    } else {
        mb.xl = fig.xl;
        mb.xh = fig.xh;
        if fig.yl == mb.yh {
            mb.yl -= grid;
        } else {
            mb.yh += grid;
        }
    }
    mb
}

/// A written patch's origin: the path point nearest its own (squared distance; the first of
/// equals), its box moved so it stays where it was. No points: unchanged.
fn move_to_nearest(origin: (i32, i32), offset: Rect, points: &[(i32, i32)]) -> ((i32, i32), Rect) {
    if points.is_empty() {
        return (origin, offset);
    }
    let d2 = |a: (i32, i32)| (i64::from(a.0) - i64::from(origin.0)).pow(2) + (i64::from(a.1) - i64::from(origin.1)).pow(2);
    let mut best = i64::MAX;
    let mut c = (0, 0);
    for &pt in points {
        if d2(pt) < best {
            best = d2(pt);
            c = pt;
        }
    }
    (c, Rect::new(origin.0 - c.0 + offset.xl, origin.1 - c.1 + offset.yl, origin.0 - c.0 + offset.xh, origin.1 - c.1 + offset.yh))
}

/// `updateGCWorker` after the patch pass: each patched net (by id) rebuilt from its worker nets'
/// shapes with its patches added.
fn update_patched(gw: &mut Worker<'_>, q: &QueueCtx<'_>, nets: &[DrNet], state: &[NetState], pw: &[(usize, PatchWire)]) {
    let mut ids: Vec<(usize, Owner)> = Vec::new();
    for (ni, p) in pw {
        if !ids.iter().any(|(_, o)| *o == p.owner) {
            ids.push((nets[*ni].net, p.owner.clone()));
        }
    }
    ids.sort_by_key(|(id, _)| *id);
    let batch: Vec<_> = ids
        .into_iter()
        .map(|(_, owner)| {
            let (mut route, taper) = owner_route(q, nets, state, &owner);
            route.extend(pw.iter().filter(|(_, p)| p.owner == owner).map(|(_, p)| (p.layer, p.bbox())));
            (owner, route, taper)
        })
        .collect();
    gw.replace_routes(&batch);
}

/// `writeGCPatchesToDRWorker`: each patch onto its worker net's route (and region query); with
/// path points, its origin moved to the nearest (the first of equals) and its box kept.
fn write_gc_patches(q: &QueueCtx<'_>, state: &mut [NetState], rq: &mut RouteRq, pw: &[(usize, PatchWire)], points: &[(i32, i32)]) -> Vec<Event> {
    let mut out = Vec::new();
    for (ni, p) in pw {
        let (origin, offset) = move_to_nearest(p.origin, p.offset, points);
        let fig = DrFig::Patch { layer: p.layer, origin, offset };
        rq.add_fig(q.mcfg.tech, *ni, state[*ni].figs.len(), &fig);
        state[*ni].figs.push(fig.clone());
        out.push(Event::Patch { net: *ni, fig, valid: points.len() });
    }
    out
}

/// The check on one owner, against everything the check holds.
fn check(gw: &mut Worker<'_>, target: &Owner) -> Vec<Marker> {
    gw.target = Some(target.clone());
    gw.run().to_vec()
}

/// The metal of the worker's other nets of net `i`'s design net (same name), in worker order:
/// each one's boundary shapes, then what it wrote. Upstream, the end-of-line cost of a routed net
/// is its check net's (one per design net: `modEolCosts_poly(gcNet)`), so a sibling's wire that
/// meets the pin closes the pin's end; ours merged only the routed net's own shapes.
fn sibling_metal(tech: &crate::tech::Tech, state: &[NetState], name: &dyn Fn(usize) -> String, i: usize) -> Vec<(usize, Rect)> {
    let me = name(i);
    let mut out = Vec::new();
    for (j, s) in state.iter().enumerate() {
        if j == i || name(j) != me {
            continue;
        }
        for f in s.ext.iter().chain(&s.figs) {
            out.extend(f.metal(tech));
        }
    }
    out
}

/// An owner's route shapes as the check holds them: every worker net of it, its committed
/// shapes and what it wrote (metal and cuts); for a rule net, which are tapered (a patch counts
/// untapered where it touches an untapered shape).
#[allow(clippy::type_complexity)]
fn owner_route(q: &QueueCtx<'_>, _nets: &[DrNet], state: &[NetState], owner: &Owner) -> (Vec<(usize, Rect)>, Vec<(usize, Rect, bool)>) {
    let tech = q.mcfg.tech;
    let (mut route, mut taper) = (Vec::new(), Vec::new());
    let Owner::Net(name) = owner else { return (route, taper) };
    for (i, s) in state.iter().enumerate() {
        if (q.name)(i) != *name {
            continue;
        }
        let ndr = (q.ndr_spacing)(i).is_some();
        let mut non_tapered: Vec<(usize, Rect)> = Vec::new();
        let mut patches: Vec<(usize, Rect)> = Vec::new();
        for f in s.ext.iter().chain(&s.figs) {
            for (l, b) in f.metal(tech) {
                route.push((l, b));
                if ndr {
                    match f {
                        DrFig::Seg { tapered, .. } | DrFig::Via { tapered, .. } => {
                            taper.push((l, b, *tapered));
                            if !tapered {
                                non_tapered.push((l, b));
                            }
                        }
                        DrFig::Patch { .. } => patches.push((l, b)),
                    }
                }
            }
            if let DrFig::Via { via, origin, .. } = f {
                let vd = &tech.via_defs[*via];
                for c in &vd.cut_figs {
                    route.push((vd.cut, Rect { xl: c.xl + origin.0, yl: c.yl + origin.1, xh: c.xh + origin.0, yh: c.yh + origin.1 }));
                }
            }
        }
        for (l, b) in patches {
            if non_tapered.iter().any(|(l2, b2)| *l2 == l && touches(b2, &b)) {
                taper.push((l, b, false));
            }
        }
    }
    (route, taper)
}

/// The worker's check as the reference builds it: the design's shapes in the extended box
/// (their owners created in the query's order), every worker net's committed shapes, packed;
/// then each owner (by net) updated once — the maze-cost pass's check update.
fn owner_tag(o: &Owner) -> String {
    match o {
        Owner::Net(n) => format!("0:{n}"),
        Owner::FloatingGround => "0:frFakeVSS".into(),
        Owner::FloatingPower => "0:frFakeVDD".into(),
        Owner::BlockTerm(n) => format!("1:{n}"),
        Owner::Inst(n) => format!("3:{n}"),
        Owner::InstBlockage(n, k, _, _) => format!("19:{n}#{k}"),
        Owner::InstTerm(i, t) => format!("7:{i}/{t}"),
        Owner::Blockage(b) => format!("14:#{b}"),
    }
}

fn check_init<'t>(q: &QueueCtx<'t>, nets: &[DrNet], state: &[NetState], mut dump: Option<&mut Vec<Event>>) -> Worker<'t> {
    let tech = q.mcfg.tech;
    let mut gw = Worker::new(tech);
    gw.drc_box = Some(q.drc_box);
    gw.top_routing_layer = q.mcfg.top_routing_layer;
    for (o, l, b) in q.fixed {
        gw.add(o, *l, *b, true);
    }
    gw.check_ndrs = true;
    gw.max_ndr_spacing = q.max_ndr_spacing.to_vec();
    let mut owners: Vec<(usize, Owner)> = Vec::new();
    for (i, n) in nets.iter().enumerate() {
        let owner = Owner::Net((q.name)(i));
        if !owners.iter().any(|(_, o)| *o == owner) {
            owners.push((n.net, owner.clone()));
        }
    }
    // Worker nets' shapes in worker order (their owners created after the design's).
    for (_, owner) in &owners {
        gw.ensure_owner(owner);
        let (route, taper) = owner_route(q, nets, state, owner);
        for (l, b) in route {
            gw.add(owner, l, b, false);
        }
        for (l, b, t) in taper {
            gw.add_taper(owner, l, b, t);
        }
    }
    for i in 0..nets.len() {
        if let Some(sp) = (q.ndr_spacing)(i) {
            gw.set_ndr_spacing(&Owner::Net((q.name)(i)), sp);
        }
    }
    gw.init();
    if let Some(ev) = dump.as_deref_mut() {
        for o in gw.owners() {
            ev.push(Event::Gc(format!("gcinit|{}", owner_tag(&o)) + &gw.dump(&o)));
        }
    }
    // Per net (by id), ONE update per worker net of it: a net split into k worker nets is removed
    // from the check's region query and re-inserted k times, and every re-insert reshapes the
    // tree — which decides the order a later query finds its rectangles in.
    owners.sort_by_key(|(id, _)| *id);
    for (net, owner) in &owners {
        let (route, taper) = owner_route(q, nets, state, owner);
        for _ in nets.iter().filter(|n| n.net == *net) {
            gw.replace_route(owner, &route, &taper);
            if let Some(ev) = dump.as_deref_mut() {
                ev.push(Event::Gc(format!("gcupd|{}", owner_tag(owner)) + &gw.dump(owner)));
            }
        }
    }
    gw
}

fn can_ripup(q: &QueueCtx<'_>, state: &[NetState], i: usize) -> bool {
    state[i].reroutes < q.maze_end_iter && !(q.pinned)(i)
}

/// The markers' consequences for the queue: sorted routes, then sorted checks, appended.
#[allow(clippy::too_many_arguments)]
fn update_queue(q: &QueueCtx<'_>, nets: &[DrNet], state: &mut [NetState], by_name: &HashMap<String, Vec<usize>>, markers: &[Marker], queue: &mut VecDeque<Entry>, checking: Option<Owner>) {
    let mut unique_victims: BTreeSet<(i32, String, i64)> = BTreeSet::new();
    let mut unique_aggressors: BTreeSet<(i32, String, i64)> = BTreeSet::new();
    let mut checks: Vec<Entry> = Vec::new();
    let mut routes: Vec<Entry> = Vec::new();
    for m in markers {
        update_from_marker(q, nets, state, by_name, m, &mut unique_victims, &mut unique_aggressors, &mut checks, &mut routes, &checking);
    }
    let sort = |v: &mut Vec<Entry>| v.sort_by_key(|a| (a.do_route, a.num_reroute, key(q, nets, &a.block), opt_key(q, &a.checking)));
    sort(&mut routes);
    sort(&mut checks);
    queue.extend(routes);
    queue.extend(checks);
}

/// `route_queue_update_from_marker`'s first test: a marker off the route box counts when a
/// side's shape reaches it — the aggressor, then every victim (a copied marker has none).
fn marker_reaches_route_box(m: &Marker, rb: &Rect) -> bool {
    touches(&m.bbox, rb) || m.aggressor.as_ref().is_some_and(|a| touches(&a.2, rb)) || m.victim.as_ref().is_some_and(|v| touches(&v.2, rb)) || m.second_victim.as_ref().is_some_and(|v| touches(&v.2, rb))
}

#[allow(clippy::too_many_arguments)]
fn update_from_marker(q: &QueueCtx<'_>, nets: &[DrNet], state: &mut [NetState], by_name: &HashMap<String, Vec<usize>>, m: &Marker, uv: &mut BTreeSet<(i32, String, i64)>, ua: &mut BTreeSet<(i32, String, i64)>, checks: &mut Vec<Entry>, routes: &mut Vec<Entry>, checking: &Option<Owner>) {
    if !marker_reaches_route_box(m, &q.route_box) {
        return;
    }
    let dr_nets = |o: &Owner| -> Vec<usize> {
        match o {
            Owner::Net(n) if !(q.is_supply)(n) => by_name.get(n).cloned().unwrap_or_default(),
            _ => Vec::new(),
        }
    };
    let mut victim_owners: Vec<Owner> = Vec::new();
    let mut aggressor_owners: Vec<Owner> = Vec::new();
    let mut movable_owners: BTreeSet<(i32, String, i64)> = BTreeSet::new();
    // The aggressor (a copied marker has none: every source is then a candidate).
    let agg: Option<&Owner> = m.aggressor.as_ref().map(|a| &a.0);
    let agg_nets = agg.map_or(Vec::new(), dr_nets);
    for &i in &agg_nets {
        if can_ripup(q, state, i) {
            movable_owners.insert(owner_key(q, agg.expect("an aggressor")));
        }
    }
    let srcs: Vec<Owner> = {
        let mut v = m.owners.clone();
        v.sort_by_key(|o| owner_key(q, o));
        v
    };
    let mut has_reroute = false;
    for &i in &agg_nets {
        if !can_ripup(q, state, i) {
            continue;
        }
        let agg = agg.expect("an aggressor");
        if ua.insert(owner_key(q, agg)) {
            aggressor_owners.push(agg.clone());
        }
        has_reroute = true;
    }
    if has_reroute {
        for s in &srcs {
            if !movable_owners.contains(&owner_key(q, s)) && uv.insert(owner_key(q, s)) {
                victim_owners.push(s.clone());
            }
        }
    } else {
        let others: Vec<&Owner> = srcs.iter().filter(|s| !movable_owners.contains(&owner_key(q, s))).collect();
        let mut route_owners: BTreeSet<(i32, String, i64)> = BTreeSet::new();
        for o in others {
            for i in dr_nets(o) {
                if !can_ripup(q, state, i) {
                    continue;
                }
                if ua.insert(owner_key(q, o)) {
                    aggressor_owners.push(o.clone());
                }
                route_owners.insert(owner_key(q, o));
                has_reroute = true;
            }
        }
        if has_reroute {
            for s in &srcs {
                if !route_owners.contains(&owner_key(q, s)) && uv.insert(owner_key(q, s)) {
                    victim_owners.push(s.clone());
                }
            }
        }
    }
    let mut avoid: Vec<usize> = Vec::new();
    let mut allow_avoid = false;
    let many = aggressor_owners.len() + victim_owners.len() > 1;
    for a in &aggressor_owners {
        for i in dr_nets(a) {
            if !can_ripup(q, state, i) {
                continue;
            }
            if many {
                if state[i].ripup_avoids < (q.max_ripup_avoids)(i) {
                    avoid.push(i);
                    continue;
                }
                allow_avoid = true;
                state[i].ripup_avoids = 0;
            }
            routes.push(Entry { block: Block::Net(i), num_reroute: state[i].reroutes as i32, do_route: true, checking: checking.clone() });
        }
    }
    for i in avoid {
        if nets[i].pins.len() <= 1 || allow_avoid {
            state[i].ripup_avoids += 1;
            checks.push(Entry { block: Block::Net(i), num_reroute: -1, do_route: false, checking: checking.clone() });
        } else {
            state[i].ripup_avoids = 0;
            routes.push(Entry { block: Block::Net(i), num_reroute: state[i].reroutes as i32, do_route: true, checking: checking.clone() });
        }
    }
    for v in victim_owners {
        checks.push(Entry { block: Block::Owner(v), num_reroute: -1, do_route: false, checking: checking.clone() });
    }
}

/// Marker costs decay after each route; a node whose cost reaches 0 is forgotten.
fn marker_cost_decay(w: &mut CostWorker<'_, '_>, d: f32, planar: &mut BTreeSet<Idx>, via: &mut BTreeSet<Idx>) {
    let decay = |v: &mut u8| -> bool {
        let c = (f32::from(*v) * d) as i32;
        *v = c.max(0) as u8;
        *v == 0
    };
    planar.retain(|m| {
        let k = w.g.idx(m.0 as usize, m.1 as usize, m.2 as usize);
        !decay(&mut w.g.nodes[k].marker_planar)
    });
    via.retain(|m| {
        let k = w.g.idx(m.0 as usize, m.1 as usize, m.2 as usize);
        !decay(&mut w.g.nodes[k].marker_via)
    });
}

fn add10(v: &mut u8) {
    *v = (u32::from(*v) + 10).min(255) as u8;
}

/// A marker's history cost on the route shapes under it (its box on its layer).
#[allow(clippy::too_many_arguments)]
fn add_marker_cost(w: &mut CostWorker<'_, '_>, q: &QueueCtx<'_>, state: &[NetState], rq: &RouteRq, m: &Marker, planar: &mut BTreeSet<Idx>, via: &mut BTreeSet<Idx>) {
    let rb = q.route_box;
    let in_rb = |p: (i32, i32)| p.0 >= rb.xl && p.0 <= rb.xh && p.1 >= rb.yl && p.1 <= rb.yh;
    let mut vio_nets: BTreeSet<usize> = BTreeSet::new();
    // The route shapes on the marker's layer touching its box, in the region query's order.
    for (ni, ext, k) in rq.query(m.layer, &m.bbox) {
        let f = if ext { &state[ni].ext[k] } else { &state[ni].figs[k] };
        {
            match *f {
                DrFig::Seg { begin, end, width, bi, ei, .. } => {
                    if !(in_rb(begin) && in_rb(end)) {
                        continue;
                    }
                    let mut bx = bloat(&m.bbox, width);
                    let (mut x1, mut y1, mut x2, mut y2) = w.g.idx_box(&bx);
                    let horz = bi.1 == ei.1;
                    for _ in 0..5 {
                        let short = if horz { bi.0.max(x1) >= ei.0.min(x2) } else { bi.1.max(y1) >= ei.1.min(y2) };
                        if !short {
                            break;
                        }
                        bx = bloat(&bx, width);
                        (x1, y1, x2, y2) = w.g.idx_box(&bx);
                    }
                    if horz {
                        for i in bi.0.max(x1)..=ei.0.min(x2) {
                            let k = w.g.idx(i, bi.1, bi.2);
                            add10(&mut w.g.nodes[k].marker_planar);
                            planar.insert((i as i32, bi.1 as i32, bi.2 as i32));
                        }
                    } else {
                        for j in bi.1.max(y1)..=ei.1.min(y2) {
                            let k = w.g.idx(bi.0, j, bi.2);
                            add10(&mut w.g.nodes[k].marker_planar);
                            planar.insert((bi.0 as i32, j as i32, bi.2 as i32));
                        }
                    }
                }
                DrFig::Via { origin, bi, .. } => {
                    if !in_rb(origin) || vio_nets.contains(&ni) {
                        continue;
                    }
                    let b = (bi.0 as i32, bi.1 as i32, bi.2 as i32);
                    if !via.contains(&b) {
                        let k = w.g.idx(bi.0, bi.1, bi.2);
                        add10(&mut w.g.nodes[k].marker_via);
                        via.insert(b);
                        vio_nets.insert(ni);
                    }
                }
                DrFig::Patch { layer, origin, offset } => {
                    if !in_rb(origin) {
                        continue;
                    }
                    let b = Rect { xl: offset.xl + origin.0, yl: offset.yl + origin.1, xh: offset.xh + origin.0, yh: offset.yh + origin.1 };
                    let Some(z) = w.g.z_of(layer) else { continue };
                    let (sx, sy) = (w.g.xs.partition_point(|&c| c < b.xl), w.g.ys.partition_point(|&c| c < b.yl));
                    let (ex, ey) = (w.g.xs.partition_point(|&c| c < b.xh), w.g.ys.partition_point(|&c| c < b.yh));
                    for x in sx..=ex {
                        for y in sy..=ey {
                            if x >= w.g.xs.len() || y >= w.g.ys.len() {
                                continue;
                            }
                            let k = w.g.idx(x, y, z);
                            add10(&mut w.g.nodes[k].marker_planar);
                            add10(&mut w.g.nodes[k].marker_via);
                            if z > 0 {
                                let kd = w.g.idx(x, y, z - 1);
                                add10(&mut w.g.nodes[kd].marker_via);
                            }
                            planar.insert((x as i32, y as i32, z as i32));
                        }
                    }
                }
            }
        }
    }
    let _ = Rule::Short;
}

fn bloat(r: &Rect, d: i32) -> Rect {
    Rect { xl: r.xl - d, yl: r.yl - d, xh: r.xh + d, yh: r.yh + d }
}

/// The routing order's first-iteration map (for callers building the queue).
pub fn order_map(order: &[usize]) -> BTreeMap<usize, usize> {
    order.iter().enumerate().map(|(k, &i)| (i, k)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// ⛔ Divergence D4 (register; OpenROAD issue #12191). Upstream rule
    /// (`checkMetalShape_patchOwner_helper`, as `checkMetalShape_addPatch` calls it for a design
    /// net with several worker nets): the box compared is the patch's OFFSET box (relative to its
    /// origin — the reference's trace shows (-70, -300, 70, 0) for a patch at (87210, 68320)), and
    /// it must strictly overlap, so a worker net is found only when a shape of it reaches the
    /// design's origin. Our rule, the reference patched with upstream-fix 0007 v2: the patch's
    /// ABSOLUTE box, and touching counts (a patch abuts the shape it grows from). First match in
    /// order; pin box before shapes.
    #[test]
    fn a_patch_owner_is_looked_up_by_where_the_patch_is_touching_included() {
        let b = Rect::new(87210 - 70, 68320 - 300, 87210 + 70, 68320);
        let far = |i| (i, Rect::new(1000, 1000, 2000, 2000), vec![Rect::new(-70, -300, 70, 0)]);
        assert_eq!(patch_owner_helper(b, [far(0), far(1)].into_iter()), None, "the origin-relative box does not count");
        let abutting = (1, Rect::new(5000, 5000, 6000, 6000), vec![Rect::new(9000, 9000, 9100, 9100), Rect::new(87140, 68320, 87280, 68600)]);
        assert_eq!(patch_owner_helper(b, [far(0), abutting.clone()].into_iter()), Some(1), "a shape sharing only an edge owns it");
        let on_pin = (2, Rect::new(87000, 68000, 88000, 69000), vec![]);
        assert_eq!(patch_owner_helper(b, [on_pin, abutting].into_iter()), Some(2), "the first in order, pin box first");
        let apart = (3, Rect::new(87281, 68000, 87300, 68100), vec![Rect::new(87000, 68321, 87100, 68400)]);
        assert_eq!(patch_owner_helper(b, [apart].into_iter()), None, "one unit apart");
        // The box looked up is the absolute one: the trace's offset box at the trace's origin.
        let p = crate::gc::AreaPatch { layer: 2, origin: (87210, 68320), offset: Rect::new(-70, -300, 70, 0), owner: Owner::Net("n".into()) };
        assert_eq!(patch_box(&p), b, "the offset box moved to the origin, not the offset box");
    }

    /// Rule (`checkMetalShape_addPatch`, its end): per area patch in order, the owner first — a
    /// patch without one is dropped and COUNTED (the reference drops it silently; OpenROAD issue
    /// #12191) — then validity: an invalid patch is dropped but not counted. Kept patches keep
    /// their order and worker nets.
    #[test]
    fn area_patches_without_a_worker_net_are_dropped_and_counted() {
        use crate::gc::AreaPatch;
        let rb = Rect::new(0, 0, 1000, 1000);
        let p = |x: i32, net: &str| AreaPatch { layer: 2, origin: (x, 500), offset: Rect::new(0, -10, 100, 10), owner: Owner::Net(net.into()) };
        let patches = vec![p(100, "a"), p(200, "lost"), p(5000, "a"), p(300, "b"), p(400, "lost")];
        let owner = |q: &AreaPatch| match &q.owner {
            Owner::Net(n) if n == "a" => Some(0),
            Owner::Net(n) if n == "b" => Some(1),
            _ => None,
        };
        let (kept, dropped) = own_area_patches(patches, &owner, &rb);
        assert_eq!(dropped, 2, "two patches found no worker net; the invalid one is not counted");
        assert_eq!(kept.iter().map(|(n, w)| (*n, w.origin.0)).collect::<Vec<_>>(), [(0, 100), (1, 300)]);
        assert_eq!(own_area_patches(Vec::new(), &owner, &rb).1, 0);
    }

    /// Rule (`gc_patch::isPatchValid`, both patch passes): a patch is kept when its box touches
    /// the route box — sharing only an edge is enough — or the route box holds its origin; a patch
    /// wholly outside, origin outside, is dropped.
    #[test]
    fn a_patch_is_valid_when_it_touches_the_route_box_or_holds_its_origin_there() {
        let rb = Rect::new(0, 0, 1000, 1000);
        let p = |origin: (i32, i32), offset: Rect| PatchWire { layer: 2, origin, offset, owner: Owner::Net("a".into()) };
        assert!(is_patch_valid(&p((500, 500), Rect::new(0, -10, 100, 10)), &rb), "inside");
        assert!(is_patch_valid(&p((1000, 500), Rect::new(0, -10, 100, 10)), &rb), "origin on the boundary, box outward");
        assert!(is_patch_valid(&p((1100, 500), Rect::new(-100, -10, 0, 10)), &rb), "box edge on the boundary");
        assert!(!is_patch_valid(&p((1100, 500), Rect::new(0, -10, 100, 10)), &rb), "wholly outside");
        assert!(!is_patch_valid(&p((1001, 500), Rect::new(0, -10, 100, 10)), &rb), "one unit outside");
    }

    // Rule (`patchMetalShape_cornerSpacing`): on a horizontal layer the patch spans the shape's y
    // and one grid past the marker in x — to the left when the shape starts at the marker's right
    // side, else to the right. Vertical: the same across.
    // D12 (register; upstream OpenROAD issue #12207, fix 0014): we corner-patch whenever the technology has corner
    // spacing. The reference's gate (`hasVia2ViaMinStep() || hasCornerSpacingConstraint()`, the
    // flag set only by MINSTEP / LEF58 MINSTEP in `frTechObject::addUConstraint`) corner-patches
    // only a technology that also has a MINSTEP rule; 0014 sets it from corner spacing.
    #[test]
    fn corner_patches_follow_corner_spacing_not_min_step() {
        use crate::tech::{CornerSpacing, Layer, MinStep};
        let corner = Layer { corner_spacing: vec![CornerSpacing::default()], ..Default::default() };
        let step = Layer { min_step: Some(MinStep { min_step_length: 100, kind: None, max_length: -1, max_edges: 1 }), ..Default::default() };
        assert!(patches_corners(std::slice::from_ref(&corner)), "corner spacing, no MINSTEP: patched (the reference does not; 0014 does)");
        assert!(!patches_corners(std::slice::from_ref(&step)), "MINSTEP, no corner spacing: nothing to patch");
        assert!(patches_corners(&[step, corner]), "both: patched, as the reference");
        assert!(!patches_corners(&[Layer::default()]));
    }

    #[test]
    fn a_corner_patch_takes_the_shape_across_and_one_grid_along() {
        let m = Rect::new(100, 10, 120, 30);
        assert_eq!(stretch_corner_patch(m, Rect::new(120, 0, 300, 40), true, 1), Rect::new(99, 0, 120, 40));
        assert_eq!(stretch_corner_patch(m, Rect::new(0, 0, 100, 40), true, 1), Rect::new(100, 0, 121, 40));
        assert_eq!(stretch_corner_patch(m, Rect::new(90, 30, 130, 200), false, 1), Rect::new(90, 9, 130, 30));
    }

    // Rule (`writeGCPatchesToDRWorker`): with path points, the origin moves to the nearest (strictly
    // nearer: the first of two equals stays) and the box keeps its place.
    #[test]
    fn a_written_patch_moves_to_the_nearest_path_point() {
        let (o, b) = move_to_nearest((100, 100), Rect::new(0, -5, 20, 5), &[(90, 100), (110, 100), (100, 130)]);
        assert_eq!((o, b), ((90, 100), Rect::new(10, -5, 30, 5)));
        assert_eq!(move_to_nearest((100, 100), Rect::new(0, -5, 20, 5), &[]), ((100, 100), Rect::new(0, -5, 20, 5)));
    }

    // Rule: a marker cost decays as an integer times a 32-bit float factor, truncated — 10 × 0.95
    // is 9 (9.4999…), and a cost of 1 decays to 0.
    #[test]
    fn marker_cost_decays_by_a_truncated_float_product() {
        let decay = |v: u8, d: f32| (f32::from(v) * d) as i32;
        assert_eq!(decay(10, 0.95), 9);
        assert_eq!(decay(1, 0.95), 0);
        assert_eq!(decay(255, 0.95), 242);
    }

    // Rule (`modEolCosts_poly(gcNet)`): a routed net's end-of-line cost is its CHECK net's, one
    // per design net — its pins, and every worker net of that design net with what each wrote and
    // its boundary shapes. So a net's end-of-line metal takes in the same name's other worker nets
    // (boundary shapes, then figures, in worker order), and nothing of its own or of other names.
    #[test]
    fn the_end_of_line_metal_takes_the_other_worker_nets_of_the_same_design_net() {
        let tech = crate::tech::Tech { layers: vec![Default::default(); 3], manufacturing_grid: 5, via_defs: vec![], obs_own_width: false };
        let patch = |x: i32| DrFig::Patch { layer: 2, origin: (x, 0), offset: Rect::new(0, 0, 10, 10) };
        let st = |ext: Vec<DrFig>, figs: Vec<DrFig>| NetState { reroutes: 0, ripup_avoids: 0, figs, ext };
        let state = vec![st(vec![patch(0)], vec![patch(10)]), st(vec![patch(100)], vec![patch(110)]), st(vec![patch(200)], vec![patch(210)]), st(vec![], vec![patch(300)])];
        let names = ["in_4", "in_3", "in_4", "in_4"];
        let name = |j: usize| names[j].to_string();
        let got: Vec<i32> = sibling_metal(&tech, &state, &name, 0).into_iter().map(|(_, r)| r.xl).collect();
        assert_eq!(got, [200, 210, 300]);
        assert!(sibling_metal(&tech, &state, &name, 1).is_empty());
    }

    /// Rule (`route_queue_update_from_marker`, its first test): a marker whose box misses the
    /// route box still counts when the aggressor's box or ANY victim's box reaches it
    /// (`intersects`: touching counts) — the second victim too, which only SPACINGTABLE
    /// ORTHOGONAL markers carry; a marker none of whose boxes reach it is skipped.
    #[test]
    fn a_marker_off_the_route_box_counts_when_any_side_reaches_it() {
        let rb = Rect::new(0, 0, 1000, 1000);
        let far = Rect::new(2000, 2000, 2040, 2040);
        let side = |r: Rect| Some((Owner::Net("a".into()), 3, r, false));
        let base = Marker { bbox: far, layer: 3, rule: Rule::SpacingTableOrth, owners: vec![], aggressor: side(far), victim: side(far), second_victim: side(far) };
        assert!(!marker_reaches_route_box(&base, &rb), "every box off the route box");
        assert!(marker_reaches_route_box(&Marker { bbox: Rect::new(1000, 0, 1040, 40), ..base.clone() }, &rb), "the box touches it");
        assert!(marker_reaches_route_box(&Marker { aggressor: side(Rect::new(960, 960, 1000, 1000)), ..base.clone() }, &rb), "the aggressor");
        assert!(marker_reaches_route_box(&Marker { victim: side(Rect::new(500, 500, 540, 540)), ..base.clone() }, &rb), "the first victim");
        assert!(marker_reaches_route_box(&Marker { second_victim: side(Rect::new(500, 500, 540, 540)), ..base.clone() }, &rb), "the second victim");
    }

    fn lonely(via: usize, x: i32) -> DrFig {
        DrFig::Via { via, origin: (x, 0), bi: (0, 0, 0), ei: (0, 0, 0), tapered: false, bottom_connected: false, top_connected: false, lonely: true }
    }

    fn defs(figs: &[DrFig]) -> Vec<(usize, i32)> {
        figs.iter()
            .map(|f| match f {
                DrFig::Via { via, origin, .. } => (*via, origin.0),
                _ => (usize::MAX, 0),
            })
            .collect()
    }

    /// Rule (`route_queue_main`, VIASWAP): the net's route list is walked by position while a
    /// swapped via is erased and appended (a range-for over the `std::vector`): the shape after
    /// a swapped one is skipped, the appended via is visited again at the end (unchanged). The
    /// reference's ispd18 trace shows it: of a net's two adjacent lonely VIA12 vias, one is
    /// swapped per entry. Re-route r takes the layer's secondary r.
    #[test]
    fn a_via_swap_skips_the_shape_after_a_swapped_via_and_revisits_it_at_the_end() {
        let mut figs = vec![lonely(0, 10), lonely(0, 20), lonely(1, 30)];
        let sec = vec![Vec::new(), vec![5, 6], vec![7]];
        let cut = |v: usize| if v == 0 || v == 5 || v == 6 { 1 } else { 2 };
        let (mut orig, mut visits, mut swaps) = (HashMap::new(), Vec::new(), Vec::new());
        let ran_out = swap_walk(&mut figs, &cut, &sec, &mut orig, 0, 0, &mut visits, &mut swaps);
        assert!(!ran_out);
        // Visited: the first (swapped to 5; the second moves into place 0, already passed), the
        // third (now at place 1, swapped to 7: the appended 5 moves into place 1, passed too),
        // then the appended 7 at place 2, unchanged.
        assert_eq!(visits, [(0, 5, false), (1, 7, false), (7, 7, false)]);
        assert_eq!(defs(&figs), [(0, 20), (5, 10), (7, 30)]);
        assert_eq!(swaps.iter().map(|(p, _)| *p).collect::<Vec<_>>(), [0, 1]);
    }

    /// Rule (upstream fix 0013, OpenROAD issue #12203, register D11): when a layer has no secondary left
    /// (`size() <= numReroute`) the via goes back to the def it had BEFORE ITS FIRST SWAP, and the
    /// entry runs no check. The clean reference takes the layer's default def there.
    #[test]
    fn a_via_whose_layer_ran_out_goes_back_to_its_first_def() {
        let mut figs = vec![lonely(0, 10)];
        let sec = vec![Vec::new(), vec![5]];
        let cut = |_: usize| 1;
        let (mut orig, mut visits, mut swaps) = (HashMap::new(), Vec::new(), Vec::new());
        assert!(!swap_walk(&mut figs, &cut, &sec, &mut orig, 0, 0, &mut visits, &mut swaps));
        assert_eq!(defs(&figs), [(5, 10)]);
        visits.clear();
        assert!(swap_walk(&mut figs, &cut, &sec, &mut orig, 0, 1, &mut visits, &mut swaps));
        assert_eq!(visits, [(5, 0, true)]);
        assert_eq!(defs(&figs), [(0, 10)]);
        // A via never swapped keeps its def when its layer has run out (nothing to restore).
        let mut figs = vec![lonely(0, 10)];
        visits.clear();
        assert!(swap_walk(&mut figs, &cut, &sec, &mut HashMap::new(), 0, 1, &mut visits, &mut swaps));
        assert_eq!(visits, [(0, 0, true)]);
    }

    /// Rule: only a via flagged lonely is swapped (a plain via, a wire are passed over).
    #[test]
    fn only_a_lonely_via_is_swapped() {
        let plain = DrFig::Via { via: 0, origin: (0, 0), bi: (0, 0, 0), ei: (0, 0, 0), tapered: false, bottom_connected: false, top_connected: false, lonely: false };
        let mut figs = vec![plain.clone()];
        let (mut visits, mut swaps) = (Vec::new(), Vec::new());
        swap_walk(&mut figs, &|_| 1, &[Vec::new(), vec![5]], &mut HashMap::new(), 0, 0, &mut visits, &mut swaps);
        assert!(visits.is_empty() && figs == [plain]);
    }

    // Rule: a marker adds 10, saturating at 255.
    #[test]
    fn marker_cost_saturates() {
        let mut v = 250u8;
        add10(&mut v);
        assert_eq!(v, 255);
    }

    /// Rule (`patchMetalShape_minStep`, the VIASWAP branch, seen in the reference's trace on the
    /// tiny `steppad` case): a marker that spans a lonely via's enclosure across (a vertical box,
    /// dx < dy: the same x range) and shares its top end grows up, its bottom end grows down, by
    /// MINSTEP less its own height; a horizontal box (`Rect::getDir`: dx >= dy, a square too) the
    /// same way in x (the high end tried first); never shrinks; a marker that touches neither end, or does not span the box
    /// across, is not the via's.
    #[test]
    fn a_min_step_marker_grows_from_the_end_of_the_lonely_via_pad_it_spans() {
        let r = Rect::new;
        let enc = r(2465, 2035, 2535, 2165);
        assert_eq!(min_step_bloat(r(2465, 2135, 2535, 2165), enc, 100), Some(r(2465, 2135, 2535, 2235)));
        assert_eq!(min_step_bloat(r(2465, 2035, 2535, 2065), enc, 100), Some(r(2465, 1965, 2535, 2065)));
        assert_eq!(min_step_bloat(r(2465, 2080, 2535, 2120), enc, 100), None, "neither end");
        assert_eq!(min_step_bloat(r(2470, 2135, 2535, 2165), enc, 100), None, "not across");
        let h = r(0, 0, 200, 70);
        assert_eq!(min_step_bloat(r(135, 0, 200, 70), h, 100), Some(r(135, 0, 235, 70)));
        assert_eq!(min_step_bloat(r(0, 0, 65, 70), h, 100), Some(r(-35, 0, 65, 70)));
        assert_eq!(min_step_bloat(r(0, 0, 150, 70), h, 100), Some(r(0, 0, 150, 70)), "never shrinks");
        assert_eq!(min_step_bloat(r(0, 0, 70, 70), r(0, 0, 70, 70), 100), Some(r(0, 0, 100, 70)), "a square is horizontal, and a marker over the whole box grows at the high end");
    }

    /// Rule (`patchMetalShape_minStep`): only a LONELY via in a via-swap worker on a layer with a
    /// classic MINSTEP takes the marker; `break` ends the via's origin group only, so a later
    /// origin (`std::map<Point>`: x, then y) takes it again from the box already grown, and the
    /// last taken wins; inside a group the first wins.
    #[test]
    fn a_later_origin_retakes_a_min_step_marker_from_the_grown_box() {
        let r = Rect::new;
        let mb = r(135, 0, 200, 70);
        let a = (0, r(0, 0, 200, 70), true);
        let b = (1, r(100, 0, 235, 70), true);
        type Via = (usize, Rect, bool);
        let group = |v: Vec<((i32, i32), Via)>| {
            let mut m: ViasByOrigin = BTreeMap::new();
            for (o, x) in v {
                m.entry(o).or_default().push(x);
            }
            m
        };
        assert_eq!(min_step_take(mb, &group(vec![((20, 0), b), ((5, 0), a)]), Some(100), true), (r(135, 0, 235, 70), Some((1, (20, 0)))));
        assert_eq!(min_step_take(mb, &group(vec![((5, 0), a), ((5, 0), b)]), Some(100), true), (r(135, 0, 235, 70), Some((0, (5, 0)))));
        assert_eq!(min_step_take(mb, &group(vec![((5, 0), a)]), Some(100), false).1, None, "not a via-swap worker");
        assert_eq!(min_step_take(mb, &group(vec![((5, 0), (0, a.1, false))]), Some(100), true).1, None, "not lonely");
        assert_eq!(min_step_take(mb, &group(vec![((5, 0), a)]), None, true).1, None, "no classic MINSTEP");
    }
}
