// SPDX-License-Identifier: Apache-2.0
//! A worker's routing: the reroute queue and, per net, the maze search and what it writes.
//!
//! Stages, in order ([`init_queue`] first): the first iteration rips up every net and routes
//! them all; before any search each net reserves a via at its cheapest via access point (route
//! cost around it, so other nets keep clear) — in priority order: clock nets, then non-default-rule nets,
//! then fewest pins inside the worker, smallest pin box, lowest id — and the nets then route in
//! NAME order; a net lifts its own reservation when its turn comes.

use crate::dr::cost::{CostWorker, DrFig, ModCost, Ndr};
use crate::dr::drw::DrNet;

/// A net's absolute priority: 4 for a clock net, 2 for one with a non-default rule, else 0.
pub fn abs_priority(is_clock: bool, has_ndr: bool) -> i32 {
    if is_clock {
        4
    } else if has_ndr {
        2
    } else {
        0
    }
}

/// The first iteration's routing order: nets with more than one pin, by absolute priority
/// (highest first), then (pins inside the worker, pin-box area, id) — the marker priorities are
/// all equal before any marker.
pub fn sort_reroute_nets(nets: &[DrNet], abs: &dyn Fn(&DrNet) -> i32) -> Vec<usize> {
    let mut order: Vec<usize> = (0..nets.len()).filter(|&i| nets[i].pins.len() > 1).collect();
    let area = |n: &DrNet| i64::from(n.pin_box.dx()) * i64::from(n.pin_box.dy());
    order.sort_by_key(|&i| (std::cmp::Reverse(abs(&nets[i])), nets[i].num_pins_in, area(&nets[i]), nets[i].id));
    order
}

/// A net's via reservation: per pin with a terminal (not a macro's), the access point with a
/// via and the lowest pin cost (the first such on a tie; stop at a cost of 0) gets its best via,
/// costed as a route shape (end of line and cut spacing included).
///
/// `is_macro_term`: a terminal of a block, pad or ring instance.
pub fn init_maze_cost_via_helper(w: &mut CostWorker<'_, '_>, net: &DrNet, add: bool, ndr: Option<Ndr<'_>>, is_macro_term: &dyn Fn(usize) -> bool) {
    let tech = w.cx.tech;
    for pin in &net.pins {
        let Some(term) = pin.term else { continue };
        if is_macro_term(term) {
            continue;
        }
        let mut best: Option<usize> = None;
        for (k, p) in pin.patterns.iter().enumerate() {
            let Some(ap) = &p.ap else { continue };
            let has_via = (ap.has(crate::dr::cost::Dir6::U) || ap.has(crate::dr::cost::Dir6::D)) && !ap.vias.is_empty();
            if !has_via {
                continue;
            }
            match best {
                None => best = Some(k),
                Some(b) if p.pin_cost < pin.patterns[b].pin_cost => {
                    best = Some(k);
                    if p.pin_cost == 0 {
                        break;
                    }
                }
                _ => {}
            }
        }
        let Some(b) = best else { continue };
        let p = &pin.patterns[b];
        let ap = p.ap.as_ref().expect("an access record");
        let via = ap.vias[0];
        let vd = &tech.via_defs[via];
        let (Some(x), Some(y), Some(z)) = (w.g.xs.binary_search(&p.point.0).ok(), w.g.ys.binary_search(&p.point.1).ok(), w.g.z_of(vd.layer1)) else { continue };
        let fig = DrFig::Via { via, origin: p.point, bi: (x, y, z), ei: (x, y, z + 1), tapered: false };
        w.mod_path_cost(&fig, if add { ModCost::AddRoute } else { ModCost::SubRoute }, true, true, ndr);
    }
}

/// The first iteration's queue: every net's via reservation, made in the priority order, then
/// the ROUTING order — the queue re-sorted by net name (bytes), then worker net id.
pub fn init_queue<'n>(w: &mut CostWorker<'_, '_>, nets: &[DrNet], abs: &dyn Fn(&DrNet) -> i32, name: &dyn Fn(&DrNet) -> &'n str, ndr_of: &dyn Fn(&DrNet) -> Option<Ndr<'n>>, is_macro_term: &dyn Fn(usize) -> bool) -> Vec<usize> {
    let mut order = sort_reroute_nets(nets, abs);
    for &i in &order {
        init_maze_cost_via_helper(w, &nets[i], true, ndr_of(&nets[i]), is_macro_term);
    }
    order.sort_by(|&a, &b| name(&nets[a]).cmp(name(&nets[b])).then(nets[a].id.cmp(&nets[b].id)));
    order
}

use std::collections::{BTreeMap, BTreeSet};

use crate::dr::cost::{mod_term_cost, Dir6};
use crate::dr::maze::{Idx, Maze, MazeCfg, MazeState, TaperBox};
use crate::dr::ta::Fixed;
use crate::dr::write::{patch_min_area, write_path, WriteCtx};
use crate::polygon90::Rect;

/// What routing one net reads beyond the costs.
pub struct NetCtx<'a> {
    pub ext_box: &'a Rect,
    /// The net's input guides in the worker (layer, box).
    pub guides: &'a [(usize, Rect)],
    /// A pin's terminal record (for lifting its costs).
    pub term_fixed: &'a dyn Fn(usize) -> Fixed,
    pub is_macro_term: &'a dyn Fn(usize) -> bool,
    /// A pin's name as the reference prints it (`inst/term`, the port's name, "" at a boundary).
    pub pin_name: &'a dyn Fn(&crate::dr::drw::DrPin) -> String,
    /// The net's non-default rule: its tables and widths per z; and whether it tapers at pins.
    pub ndr: Option<(&'a crate::dr::rules::NdrTables, &'a [i32])>,
    pub auto_taper: bool,
    pub is_port_term: &'a dyn Fn(usize) -> bool,
    /// Whether a terminal of this net has an access point at the point on the layer.
    pub has_access_point: &'a dyn Fn((i32, i32), usize) -> bool,
    /// The net's non-default rule itself (widths, preferred vias).
    pub ndr_rule: Option<&'a crate::dr::rules::NdrRule>,
    pub route_box: Rect,
    /// The net's rule as the path costs read it (with its end-of-line rules).
    pub ndr_cost: Option<Ndr<'a>>,
}

/// One search's record: the destination pin, the path (dst first).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Search {
    pub dst: String,
    pub path: Vec<Idx>,
}

fn ap_idx(st: &MazeState, g: &crate::dr::drw::GridGraph, p: (i32, i32), layer: usize) -> Option<Idx> {
    let _ = st;
    Some((g.xs.binary_search(&p.0).ok()? as i32, g.ys.binary_search(&p.1).ok()? as i32, g.z_of(layer)? as i32))
}

fn ap_valid_up(p: &crate::dr::drw::DrAccessPattern) -> bool {
    p.ap.as_ref().is_none_or(|a| a.has(Dir6::U))
}

/// Whether an access point is on a track (x across a horizontal layer's wires, y a vertical's):
/// off when the grid costs its planar edges there, on its own layer or the one above it.
fn ap_on_track(w: &CostWorker<'_, '_>, p: &crate::dr::drw::DrAccessPattern) -> (bool, bool) {
    let tech = w.cx.tech;
    let (mut on_x, mut on_y) = (true, true);
    for l in [p.layer, p.layer + 2] {
        let Some(z) = w.g.z_of(l) else { continue };
        let (Ok(x), Ok(y)) = (w.g.xs.binary_search(&p.point.0), w.g.ys.binary_search(&p.point.1)) else { continue };
        let n = |x: usize, y: usize| w.g.nodes[w.g.idx(x, y, z)];
        if tech.layers[l].is_horizontal() {
            let west = x > 0 && n(x - 1, y).grid_cost_e;
            if west || n(x, y).grid_cost_e {
                on_x = false;
            }
        } else {
            let south = y > 0 && n(x, y - 1).grid_cost_n;
            if south || n(x, y).grid_cost_n {
                on_y = false;
            }
        }
    }
    (on_x, on_y)
}

fn set_ap_cost(g: &mut crate::dr::drw::GridGraph, i: Idx, d: Dir6, on: bool) {
    let (i, d) = match d {
        Dir6::W => ((i.0 - 1, i.1, i.2), Dir6::E),
        Dir6::S => ((i.0, i.1 - 1, i.2), Dir6::N),
        Dir6::D => ((i.0, i.1, i.2 - 1), Dir6::U),
        _ => (i, d),
    };
    if i.0 < 0 || i.1 < 0 || i.2 < 0 || i.0 as usize >= g.xs.len() || i.1 as usize >= g.ys.len() || i.2 as usize >= g.zs.len() {
        return;
    }
    let k = g.idx(i.0 as usize, i.1 as usize, i.2 as usize);
    match d {
        Dir6::E => g.nodes[k].ap_cost_e = on,
        Dir6::N => g.nodes[k].ap_cost_n = on,
        _ => g.nodes[k].ap_cost_u = on,
    }
}

/// From an access point along `d` up to `bloat` long: access-point cost on the edge and on the
/// vias up and down at each node.
fn ap_planar_grid(w: &mut CostWorker<'_, '_>, st: &MazeState, mi: Idx, d: Dir6, bloat: i32, add: bool) {
    let (mut x, mut y, z) = mi;
    let mut len = 0;
    loop {
        if len > bloat || !st.has_edge(w.g, (x, y, z), d) {
            break;
        }
        set_ap_cost(w.g, (x, y, z), d, add);
        set_ap_cost(w.g, (x, y, z), Dir6::D, add);
        set_ap_cost(w.g, (x, y, z), Dir6::U, add);
        match d {
            Dir6::W => x -= 1,
            Dir6::E => x += 1,
            Dir6::S => y -= 1,
            _ => y += 1,
        }
        len += st.edge_len(w.g, (x, y, z), d);
    }
}

/// A net's access-point costs: off (add = false) while it routes, on for everyone else after.
/// Block and port pins cost 10 widths of planar grid around each access point; an access point
/// that may go up has its via cost overridden while the net routes, and (unless a standard-cell
/// pin has an on-track way up) 10 upper widths along the upper layer where it is off track.
pub fn init_maze_cost_ap_helper(w: &mut CostWorker<'_, '_>, st: &MazeState, net: &DrNet, add: bool, is_macro_term: &dyn Fn(usize) -> bool, is_port_term: &dyn Fn(usize) -> bool) {
    let tech = w.cx.tech;
    let top = tech.layers.len() - 1;
    for pin in &net.pins {
        let Some(term) = pin.term else { continue };
        let std_cell = !is_port_term(term) && !is_macro_term(term);
        let mut upper_on_track = false;
        if std_cell {
            for p in &pin.patterns {
                if !ap_valid_up(p) || p.layer + 2 > top {
                    continue;
                }
                let (on_x, on_y) = ap_on_track(w, p);
                let up = &tech.layers[p.layer + 2];
                if (up.is_horizontal() && on_x) || (up.is_vertical() && on_y) {
                    upper_on_track = true;
                    break;
                }
            }
        }
        for p in &pin.patterns {
            let Some(mi) = ap_idx(st, w.g, p.point, p.layer) else { continue };
            let width = tech.layers[p.layer].width;
            if !std_cell {
                for d in [Dir6::S, Dir6::W, Dir6::E, Dir6::N] {
                    ap_planar_grid(w, st, mi, d, 10 * width, add);
                }
            }
            if !ap_valid_up(p) {
                continue;
            }
            let k = w.g.idx(mi.0 as usize, mi.1 as usize, mi.2 as usize);
            w.g.nodes[k].override_via = !add;
            if p.layer + 2 > top || upper_on_track {
                continue;
            }
            let up = &tech.layers[p.layer + 2];
            let bloat = 10 * up.width;
            let umi = (mi.0, mi.1, mi.2 + 1);
            let (on_x, on_y) = ap_on_track(w, p);
            if up.is_horizontal() && !on_x {
                ap_planar_grid(w, st, umi, Dir6::W, bloat, add);
                ap_planar_grid(w, st, umi, Dir6::E, bloat, add);
            }
            if up.is_vertical() && !on_y {
                ap_planar_grid(w, st, umi, Dir6::N, bloat, add);
                ap_planar_grid(w, st, umi, Dir6::S, bloat, add);
            }
        }
    }
}

/// A net's guides on the grid (every node of each guide's index box), or off.
pub fn init_maze_cost_guide_helper(w: &CostWorker<'_, '_>, st: &mut MazeState, guides: &[(usize, Rect)], add: bool) {
    for &(l, b) in guides {
        let Some(z) = w.g.z_of(l) else { continue };
        let (x1, y1, x2, y2) = w.g.idx_box(&b);
        if x2 < x1 || y2 < y1 {
            continue;
        }
        for x in x1..=x2 {
            for y in y1..=y2 {
                st.guide[w.g.idx(x, y, z)] = add;
            }
        }
    }
}

/// Route one net (the first iteration, from nothing): lift its own end-of-line and via
/// reservations and its pins' costs, mark its guides, then connect its pins one search at a
/// time. The searches, in order.
#[allow(clippy::too_many_arguments)]
pub fn route_net(w: &mut CostWorker<'_, '_>, st: &mut MazeState, mcfg: &MazeCfg<'_>, net: &DrNet, ndr: Option<Ndr<'_>>, cx: &NetCtx<'_>) -> (Vec<Search>, Vec<DrFig>) {
    reroute_net(w, st, mcfg, net, ndr, cx, 0, &[])
}

/// Route a net again (or the first time: `reroutes` 0, nothing written): lift what it wrote
/// (route cost, cut spacing included), its end-of-line cost from its pins and that metal, and —
/// the first time only — its via reservation; then as [`route_net`].
#[allow(clippy::too_many_arguments)]
pub fn reroute_net(w: &mut CostWorker<'_, '_>, st: &mut MazeState, mcfg: &MazeCfg<'_>, net: &DrNet, ndr: Option<Ndr<'_>>, cx: &NetCtx<'_>, reroutes: u32, old: &[DrFig]) -> (Vec<Search>, Vec<DrFig>) {
    for f in old {
        w.mod_path_cost(f, ModCost::SubRoute, false, true, ndr);
    }
    let tech = w.cx.tech;
    let metal: Vec<(usize, Rect)> = net.ext.iter().chain(old).flat_map(|f| f.metal(tech)).collect();
    crate::dr::cost::mod_eol_costs_poly_with(w, net.net, cx.ext_box, &metal, ModCost::SubRoute);
    if reroutes == 0 {
        init_maze_cost_via_helper(w, net, false, ndr, cx.is_macro_term);
    }
    maze_net_init(w, st, net, cx);
    let (searches, figs, ok) = route_net_search(w, st, mcfg, net, cx);
    if ok {
        // What the net wrote, costed for the nets after it (no end of line or cut spacing here).
        for f in &figs {
            w.mod_path_cost(f, ModCost::AddRoute, false, false, ndr);
        }
    }
    (searches, figs)
}

/// A net's end: its pins' costs back (vias kept), its guides off, its access points costed for
/// the nets after it.
pub fn maze_net_end(w: &mut CostWorker<'_, '_>, st: &mut MazeState, net: &DrNet, cx: &NetCtx<'_>) {
    let terms: BTreeSet<usize> = net.pins.iter().filter_map(|p| p.term).collect();
    for t in terms {
        mod_term_cost(w, &(cx.term_fixed)(t), true, true);
    }
    init_maze_cost_guide_helper(w, st, cx.guides, false);
    init_maze_cost_ap_helper(w, st, net, true, cx.is_macro_term, cx.is_port_term);
    for f in &net.ext {
        w.mod_path_cost(f, ModCost::AddRoute, false, false, cx.ndr_cost);
    }
}

/// After the net's check: its end-of-line route costs from its pins and what it wrote, merged.
pub fn after_check(w: &mut CostWorker<'_, '_>, net: &DrNet, figs: &[DrFig], ext_box: &Rect) {
    let tech = w.cx.tech;
    let metal: Vec<(usize, Rect)> = net.ext.iter().chain(figs).flat_map(|f| f.metal(tech)).collect();
    crate::dr::cost::mod_eol_costs_poly_with(w, net.net, ext_box, &metal, ModCost::AddRoute);
}

fn maze_net_init(w: &mut CostWorker<'_, '_>, st: &mut MazeState, net: &DrNet, cx: &NetCtx<'_>) {
    st.reset_status();
    let terms: BTreeSet<usize> = net.pins.iter().filter_map(|p| p.term).collect();
    for t in terms {
        mod_term_cost(w, &(cx.term_fixed)(t), false, true);
    }
    init_maze_cost_guide_helper(w, st, cx.guides, true);
    init_maze_cost_ap_helper(w, st, net, false, cx.is_macro_term, cx.is_port_term);
    // Its committed shapes' route cost off (same-net spacing to them is not a violation).
    for f in &net.ext {
        w.mod_path_cost(f, ModCost::SubRoute, false, false, cx.ndr_cost);
    }
}

fn route_net_search(w: &mut CostWorker<'_, '_>, st: &mut MazeState, mcfg: &MazeCfg<'_>, net: &DrNet, cx: &NetCtx<'_>) -> (Vec<Search>, Vec<DrFig>, bool) {
    let mut out = Vec::new();
    let mut figs = Vec::new();
    if net.pins.len() <= 1 {
        return (out, figs, true);
    }
    // Prep: every access point a destination; pins by id.
    let mut unconn: BTreeSet<usize> = BTreeSet::new();
    let mut at: BTreeMap<Idx, BTreeSet<usize>> = BTreeMap::new();
    let pin_by_id: BTreeMap<usize, usize> = net.pins.iter().enumerate().map(|(k, p)| (p.id, k)).collect();
    let aps = |k: usize| -> Vec<Idx> { net.pins[k].patterns.iter().filter_map(|p| ap_idx(st, w.g, p.point, p.layer)).collect() };
    for p in &net.pins {
        unconn.insert(p.id);
    }
    let all_aps: Vec<Vec<Idx>> = (0..net.pins.len()).map(aps).collect();
    for (k, p) in net.pins.iter().enumerate() {
        for &mi in &all_aps[k] {
            at.entry(mi).or_default().insert(p.id);
            st.set_dst_i(w.g, mi, true);
        }
    }
    // The source: the pin farthest from the centre of the pins' first access points.
    let (xd, yd, zd) = (w.g.xs.len() as i32, w.g.ys.len() as i32, w.g.zs.len() as i32);
    let (mut cc1, mut cc2): (Idx, Idx) = ((xd - 1, yd - 1, zd - 1), (0, 0, 0));
    let (mut tx, mut ty, mut tz, mut cnt) = (0i32, 0i32, 0i32, 0i32);
    for id in &unconn {
        let k = pin_by_id[id];
        if let (Some(p), Some(&mi)) = (net.pins[k].patterns.first(), all_aps[k].first()) {
            tx += p.point.0;
            ty += p.point.1;
            tz += st.z_height(mi.2);
            cnt += 1;
        }
    }
    let center = (tx / cnt, ty / cnt);
    let cz = tz / cnt;
    // Taper boxes: a non-default-rule net's instance pins, 3 pitches around their access points
    // (from the lowest layer to the highest, or to z 1 when the lowest is z 0).
    let mut tapers: Vec<TaperBox> = Vec::new();
    let mut taper_of_pin: BTreeMap<usize, usize> = BTreeMap::new();
    let mut taper_at: std::collections::HashMap<Idx, usize> = std::collections::HashMap::new();
    if cx.ndr.is_some() && cx.auto_taper {
        for (k, p) in net.pins.iter().enumerate() {
            let Some(t) = p.term else { continue };
            if (cx.is_port_term)(t) || all_aps[k].is_empty() {
                continue;
            }
            let lo = all_aps[k].iter().fold((i32::MAX, i32::MAX, i32::MAX), |a, m| (a.0.min(m.0), a.1.min(m.1), a.2.min(m.2)));
            let hi = all_aps[k].iter().fold((i32::MIN, i32::MIN, i32::MIN), |a, m| (a.0.max(m.0), a.1.max(m.1), a.2.max(m.2)));
            let pitch = w.cx.tech.layers[w.g.zs[lo.2 as usize]].pitch;
            let r = 3 * pitch;
            let mx = |c: i32| w.g.xs.partition_point(|&v| v < c) as i32;
            let my = |c: i32| w.g.ys.partition_point(|&v| v < c) as i32;
            let b = TaperBox {
                lo: (mx(w.g.xs[lo.0 as usize] - r), my(w.g.ys[lo.1 as usize] - r), lo.2),
                hi: (mx(w.g.xs[hi.0 as usize] + r), my(w.g.ys[hi.1 as usize] + r), if lo.2 == 0 { 1 } else { hi.2 }),
            };
            let id = tapers.len();
            tapers.push(b);
            taper_of_pin.insert(p.id, id);
            for z in b.lo.2..=b.hi.2 {
                for x in b.lo.0..=b.hi.0 {
                    for y in b.lo.1..=b.hi.1 {
                        taper_at.insert((x, y, z), id);
                    }
                }
            }
        }
    }
    let mut dst_taper: Option<usize> = None;
    // Real pins' access points, all access points, and each point's access area (the largest).
    let mut real_ap: BTreeSet<Idx> = BTreeSet::new();
    let mut any_ap: BTreeSet<Idx> = BTreeSet::new();
    let mut area_at: std::collections::HashMap<Idx, i64> = std::collections::HashMap::new();
    for (k, p) in net.pins.iter().enumerate() {
        for (a, &mi) in p.patterns.iter().zip(&all_aps[k]) {
            any_ap.insert(mi);
            if p.term.is_some() {
                real_ap.insert(mi);
            }
            let e = area_at.entry(mi).or_insert(a.begin_area);
            *e = (*e).max(a.begin_area);
        }
    }
    let mut src_pin = None;
    let mut best = 0;
    for id in &unconn {
        let k = pin_by_id[id];
        for (p, &mi) in net.pins[k].patterns.iter().zip(&all_aps[k]) {
            let d = (center.0 - p.point.0).abs() + (center.1 - p.point.1).abs() + (cz - st.z_height(mi.2)).abs();
            if d >= best {
                best = d;
                src_pin = Some(*id);
            }
        }
    }
    let src_pin = src_pin.expect("a source pin");
    unconn.remove(&src_pin);
    let mut conn: Vec<Idx> = Vec::new();
    for &mi in &all_aps[pin_by_id[&src_pin]] {
        conn.push(mi);
        cc1 = (cc1.0.min(mi.0), cc1.1.min(mi.1), cc1.2.min(mi.2));
        cc2 = (cc2.0.max(mi.0), cc2.1.max(mi.1), cc2.2.max(mi.2));
        let Some(set) = at.get_mut(&mi) else { continue };
        if !set.remove(&src_pin) {
            continue;
        }
        st.set_src_i(w.g, mi, true);
        if set.is_empty() {
            at.remove(&mi);
            st.set_dst_i(w.g, mi, false);
        }
    }
    while !unconn.is_empty() {
        st.reset_prev_dirs();
        // The next destination: the nearest access point to the connected box (first pin at it).
        let (ll, ur) = ((w.g.xs[cc1.0 as usize], w.g.ys[cc1.1 as usize]), (w.g.xs[cc2.0 as usize], w.g.ys[cc2.1 as usize]));
        let mut cur = i32::MAX;
        let mut next = None;
        for (mi, set) in &at {
            let p = (w.g.xs[mi.0 as usize], w.g.ys[mi.1 as usize]);
            let dx = (ll.0 - p.0).max(p.0 - ur.0).max(0);
            let dy = (ll.1 - p.1).max(p.1 - ur.1).max(0);
            let dz = (st.z_height(cc1.2) - st.z_height(mi.2)).max(st.z_height(mi.2) - st.z_height(cc2.2)).max(0);
            if dx + dy + dz < cur {
                cur = dx + dy + dz;
                next = set.iter().next().copied();
            }
            if cur == 0 {
                break;
            }
        }
        let next = next.expect("a destination");
        if cx.ndr.is_some() && cx.auto_taper {
            if let Some(&t) = taper_of_pin.get(&next) {
                dst_taper = Some(t);
            }
        }
        let dk = pin_by_id[&next];
        let mut path = Vec::new();
        let found = {
            let mut m = Maze { cfg: mcfg, g: w.g, st, ndr: cx.ndr.map(|n| n.0), ndr_widths: cx.ndr.map(|n| n.1), tapers: &tapers, taper_at: &taper_at, dst_taper };
            m.search(&mut conn, &all_aps[dk], &mut path, &mut cc1, &mut cc2, center)
        };
        if !found {
            out.push(Search { dst: (cx.pin_name)(&net.pins[dk]), path: Vec::new() });
            return (out, figs, false);
        }
        out.push(Search { dst: (cx.pin_name)(&net.pins[dk]), path: path.clone() });
        post_astar_update(w, st, &path, &mut conn, &mut unconn, &mut at);
        let wcx = WriteCtx { route_box: cx.route_box, real_ap: &real_ap, ap: &any_ap, has_access_point: cx.has_access_point, ndr: cx.ndr_rule, auto_taper: cx.auto_taper, tapers: &tapers, taper_at: &taper_at, area_at: &area_at };
        figs.extend(write_path(w, &wcx, &path));
        figs.extend(patch_min_area(w, st, &wcx, &path, mcfg.drc_cost, mcfg.fixed_cost, mcfg.marker_cost));
        add_cut_spc_cost(w, &path);
    }
    (out, figs, true)
}

/// After a search: the destination's pins are connected (their access points become sources),
/// every node on the path becomes a source and joins the connected set (the destination point
/// itself excepted).
fn post_astar_update(w: &CostWorker<'_, '_>, st: &mut MazeState, path: &[Idx], conn: &mut Vec<Idx>, unconn: &mut BTreeSet<usize>, at: &mut BTreeMap<Idx, BTreeSet<usize>>) {
    let mut local: BTreeSet<Idx> = BTreeSet::new();
    if let Some(&d) = path.first() {
        let pins: Vec<usize> = at.get(&d).map(|s| s.iter().copied().collect()).unwrap_or_default();
        for pid in pins {
            unconn.remove(&pid);
            let keys: Vec<Idx> = at.keys().copied().collect();
            // Its access points, in the pin's own order — any order gives the same sets.
            for mi in keys {
                let Some(set) = at.get_mut(&mi) else { continue };
                if !set.remove(&pid) {
                    continue;
                }
                if set.is_empty() {
                    at.remove(&mi);
                    st.set_dst_i(w.g, mi, false);
                }
                local.insert(mi);
                st.set_src_i(w.g, mi, true);
            }
        }
    }
    for win in path.windows(2) {
        let (a, b) = (win[0], win[1]);
        let pts: Vec<Idx> = if a.0 != b.0 && a.1 == b.1 && a.2 == b.2 {
            (a.0.min(b.0)..=a.0.max(b.0)).map(|x| (x, a.1, a.2)).collect()
        } else if a.0 == b.0 && a.1 != b.1 && a.2 == b.2 {
            (a.1.min(b.1)..=a.1.max(b.1)).map(|y| (a.0, y, a.2)).collect()
        } else if a.0 == b.0 && a.1 == b.1 && a.2 != b.2 {
            (a.2.min(b.2)..=a.2.max(b.2)).map(|z| (a.0, a.1, z)).collect()
        } else {
            Vec::new()
        };
        for p in pts {
            local.insert(p);
            st.set_src_i(w.g, p, true);
        }
    }
    for mi in local {
        if Some(&mi) != path.first() {
            conn.push(mi);
        }
    }
}

/// A path's vias: route cost for cut spacing around each via's default cut (not at its own node).
fn add_cut_spc_cost(w: &mut CostWorker<'_, '_>, path: &[Idx]) {
    let tech = w.cx.tech;
    for k in 1..path.len() {
        if path[k].2 == path[k - 1].2 {
            continue;
        }
        let z = path[k].2.min(path[k - 1].2) as usize;
        let cut = w.g.zs[z] + 1;
        let Some(v) = w.cx.defaults.get(cut).copied().flatten() else { continue };
        let origin = (w.g.xs[path[k].0 as usize], w.g.ys[path[k].1 as usize]);
        for f in tech.via_defs[v].cut_figs.clone() {
            let b = Rect { xl: f.xl + origin.0, yl: f.yl + origin.1, xh: f.xh + origin.0, yh: f.yh + origin.1 };
            w.mod_cut_spacing_cost(&b, z, ModCost::AddRoute, Some((path[k].0 as usize, path[k].1 as usize)));
        }
    }
}
