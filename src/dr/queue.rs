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
use crate::gc::{Marker, Owner, Rule, Worker};
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
    Route { net: usize, reroutes: u32, searches: Vec<Search>, figs: Vec<DrFig>, markers: Vec<Marker> },
    /// An owner checked: the markers.
    Check { owner: Owner, markers: Vec<Marker> },
    /// After the queue, the check over every owner: the worker's markers.
    Final { markers: Vec<Marker> },
    /// The check's view of an owner (`VYGD_GC`): `gcinit` / `gcupd` and its maximal rectangles.
    Gc(String),
    /// An entry pushed onto the queue (after its batch was sorted).
    Push { block: Block, num_reroute: i32, do_route: bool, checking: Option<Owner> },
}

/// What the queue reads beyond the costs.
pub struct QueueCtx<'a> {
    pub mcfg: &'a MazeCfg<'a>,
    pub route_box: Rect,
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
        for (ni, s) in state.iter().enumerate() {
            for (ext, figs) in [(false, &s.figs), (true, &s.ext)] {
                for (k, f) in figs.iter().enumerate() {
                    for (l, r) in Self::rects(tech, f) {
                        while per.len() <= l {
                            per.push(Vec::new());
                        }
                        per[l].push((r, (ni, ext, k)));
                    }
                }
            }
        }
        let n = tech.layers.len().max(per.len());
        per.resize(n, Vec::new());
        RouteRq { trees: per.into_iter().map(DynRTree::new).collect(), ids: HashMap::new() }
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
    }
    let gc_dump = std::env::var("VYGD_GC").is_ok();
    let mut gw = check_init(q, nets, &state, if gc_dump { Some(&mut events) } else { None });
    let mut gc_version = 1i64;
    let mut checked: HashMap<Block, i64> = HashMap::new();
    let History { planar: mut planar_hist, via: mut via_hist, mut rq } = hist;
    while let Some(e) = queue.pop_front() {
        let mut did_route = false;
        let (markers, checking_obj): (Vec<Marker>, Owner) = match (&e.block, e.do_route) {
            (Block::Net(i), true) => {
                let i = *i;
                if e.num_reroute != state[i].reroutes as i32 {
                    continue;
                }
                let cx = (q.net_ctx)(i);
                let ndr = (q.ndr)(i);
                let old = std::mem::take(&mut state[i].figs);
                rq.remove_net(i);
                let (searches, figs) = reroute_net(w, st, q.mcfg, &nets[i], ndr, &cx, state[i].reroutes, &old);
                rq.add_net(q.mcfg.tech, i, &figs);
                maze_net_end(w, st, &nets[i], &cx);
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
                let m = check(&mut gw, &owner);
                after_check(w, &nets[i], &figs, cx.ext_box);
                checked.insert(Block::Owner(owner.clone()), gc_version);
                events.push(Event::Route { net: i, reroutes, searches, figs, markers: m.clone() });
                (m, owner)
            }
            (b, _) => {
                if checked.get(b) == Some(&gc_version) {
                    continue;
                }
                let owner = match b {
                    // ⛔ A worker net queued to check (it avoided a ripup) names the worker net,
                    // which the check does not know: nothing is checked.
                    Block::Net(_) => {
                        checked.insert(b.clone(), gc_version);
                        continue;
                    }
                    Block::Owner(o) => o.clone(),
                };
                let m = check(&mut gw, &owner);
                checked.insert(b.clone(), gc_version);
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
    }
    gw.target = None;
    events.push(Event::Final { markers: gw.run().to_vec() });
    events
}

/// The check on one owner, against everything the check holds.
fn check(gw: &mut Worker<'_>, target: &Owner) -> Vec<Marker> {
    gw.target = Some(target.clone());
    gw.run().to_vec()
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
        Owner::InstTerm(i, t) => format!("7:{i}/{t}"),
        Owner::Blockage(b) => format!("14:#{b}"),
    }
}

fn check_init<'t>(q: &QueueCtx<'t>, nets: &[DrNet], state: &[NetState], mut dump: Option<&mut Vec<Event>>) -> Worker<'t> {
    let tech = q.mcfg.tech;
    let mut gw = Worker::new(tech);
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
    state[i].reroutes < q.maze_end_iter
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

#[allow(clippy::too_many_arguments)]
fn update_from_marker(q: &QueueCtx<'_>, nets: &[DrNet], state: &mut [NetState], by_name: &HashMap<String, Vec<usize>>, m: &Marker, uv: &mut BTreeSet<(i32, String, i64)>, ua: &mut BTreeSet<(i32, String, i64)>, checks: &mut Vec<Entry>, routes: &mut Vec<Entry>, checking: &Option<Owner>) {
    let rb = &q.route_box;
    if !touches(&m.bbox, rb) && !touches(&m.aggressor.2, rb) && !touches(&m.victim.2, rb) {
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
    let agg = &m.aggressor.0;
    let agg_nets = dr_nets(agg);
    for &i in &agg_nets {
        if can_ripup(q, state, i) {
            movable_owners.insert(owner_key(q, agg));
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

    // Rule: a marker cost decays as an integer times a 32-bit float factor, truncated — 10 × 0.95
    // is 9 (9.4999…), and a cost of 1 decays to 0.
    #[test]
    fn marker_cost_decays_by_a_truncated_float_product() {
        let decay = |v: u8, d: f32| (f32::from(v) * d) as i32;
        assert_eq!(decay(10, 0.95), 9);
        assert_eq!(decay(1, 0.95), 0);
        assert_eq!(decay(255, 0.95), 242);
    }

    // Rule: a marker adds 10, saturating at 255.
    #[test]
    fn marker_cost_saturates() {
        let mut v = 250u8;
        add10(&mut v);
        assert_eq!(v, 255);
    }
}
