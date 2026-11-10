// SPDX-License-Identifier: Apache-2.0
//! Pin access re-run for the instances the LEF58_MAXSPACING pass made dirty
//! (`FlexPA::updateDirtyInsts`, from `TritonRoute::updateDirtyPAData`), in the reference's call
//! sequence: [`update_dirty_insts`] only calls the stages, each named after the reference's step.
//!
//! The pass marks instance terminals STUBBORN (a lonely via lands on them). A stubborn terminal
//! is part of its instance's unique-class key, so each dirty instance moves to a class of its own
//! kind; a new class gets its access points made on its first instance with the stubborn
//! terminals avoiding the avoided via defs, its patterns, and the rows of abutting instances
//! around its members choose their patterns again.
//!
//! Rules:
//! - dirty instances are visited by instance id (`frOrderedIdSet`), a class's members likewise;
//! - a dirty class is one an instance moved into that was new, or one whose terminals the
//!   instance routes but the class skips (`updateSkipInstTerm`); a class created earlier in the
//!   same loop still skips every terminal (its skip map starts all true), so a second instance
//!   moving into it makes it dirty that way;
//! - a dirty class not yet initialised takes the next pin-access index of its master;
//! - its skip map is AND-ed with the members' (`setSkipTerm`'s `&=`): a terminal routed once stays
//!   routed;
//! - its access points are made on its FIRST member (`getFirstInst`), REPLACING any it had;
//! - each pattern instance in id order not yet in a row, not skipped and a standard cell seeds a
//!   row: its run of ABUTTING instances (same origin y, one's right edge on the other's left) in
//!   placement order — ⚠️ the walk back from the seed stops BEFORE the first instance of the
//!   placement order (`prev_inst_it != insts_set_.begin()`), so that instance joins a row only as
//!   a seed or when walking forward;
//! - rows run in order (`-threads 1`), a later row's choice overwriting an earlier one's for an
//!   instance in both.

use std::cmp::Ordering;
use std::collections::{BTreeSet, HashMap};

use crate::pa::access::Config;
use crate::pa::candidates::Context;
use crate::pa::flow::{gen_inst_access_points, pattern_instance, row_inst, DesignInst, FlowError, MasterClass, PinAccess};
use crate::pa::pattern::{prep_pattern_inst, Instance, Trace};
use crate::pa::row::gen_inst_row_pattern;
use crate::pa::unique::{compute_pref_track_patterns, compute_unique_class_key, master_pin_layer_range, ClassKey, UniqueClass};
use crate::tech::{Master, Tech, TrackPattern};

/// What the re-run reads besides the pin access it updates.
pub struct DirtyIn<'a> {
    /// Pin access's technology and tracks (as the first run read them).
    pub tech: &'a Tech,
    pub tracks: &'a [TrackPattern],
    pub cfg: &'a Config,
    pub masters: &'a HashMap<String, Master>,
    pub insts: &'a [DesignInst],
    /// The via defs a stubborn terminal avoids (indices into `tech.via_defs`).
    pub avoid: &'a BTreeSet<usize>,
    /// The stubborn terminals, as (instance, master terminal).
    pub stubborn: &'a BTreeSet<(usize, usize)>,
}

/// Why the re-run stops.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DirtyError {
    /// A routed terminal of a dirty class's first instance with no access point: the reference
    /// aborts here (`DRT-0073`, OpenROAD issue #12199): `inst/term (master)`.
    NoAccessPoint { inst: String, term: String, master: String },
    /// A row with no pattern combination: the reference aborts (`DRT-0085`).
    NoRowPath(String),
    /// A case this transcription does not model.
    Unsupported(String),
}

/// A trace line sink (`VYC MSPA …`, MEASUREMENT ONLY).
pub type TraceSink<'a> = Option<&'a mut dyn FnMut(String)>;

fn emit(trace: &mut TraceSink<'_>, line: impl FnOnce() -> String) {
    if let Some(t) = trace {
        t(line());
    }
}

/// `FlexPA::updateDirtyInsts`: the sequence only.
pub fn update_dirty_insts(inp: &DirtyIn<'_>, pa: &mut PinAccess, dirty: &BTreeSet<usize>, trace: &mut TraceSink<'_>, warn: &mut dyn FnMut(String)) -> Result<(), DirtyError> {
    let mut pattern_insts: BTreeSet<usize> = BTreeSet::new();
    let dirty_classes = reclassify_dirty_insts(inp, pa, dirty, &mut pattern_insts, trace);
    init_dirty_classes(pa, &dirty_classes, &mut pattern_insts);
    for &c in &dirty_classes {
        emit(trace, || format!("VYC MSPA DIRTYCLASS first={} idx={} init=1", inp.insts[pa.classes[c].insts[0]].name, pa.pa_idx[c]));
    }
    for &i in dirty {
        emit(trace, || format!("VYC MSPA INSTIDX inst={} idx={}", inp.insts[i].name, pa.pa_idx[pa.class_of[i]]));
    }
    for &c in &dirty_classes {
        gen_dirty_class_access(inp, pa, c, trace, warn)?;
    }
    for &i in dirty {
        add_to_insts_set(inp, pa, i);
    }
    for &c in &dirty_classes {
        for i in pa.classes[c].insts.clone() {
            add_to_insts_set(inp, pa, i);
        }
    }
    for &i in &pattern_insts {
        emit(trace, || format!("VYC MSPA PINST inst={}", inp.insts[i].name));
    }
    let rows = adjacent_clusters(inp, pa, &pattern_insts)?;
    gen_inst_row_patterns(inp, pa, &rows)?;
    trace_rows(inp, pa, &rows, trace);
    Ok(())
}

/// The first loop of `updateDirtyInsts`: each dirty instance (by id) out of the placement set,
/// its class recomputed with its stubborn terminals in the key, and moved to it. The dirty
/// classes, in key order.
fn reclassify_dirty_insts(inp: &DirtyIn<'_>, pa: &mut PinAccess, dirty: &BTreeSet<usize>, pattern_insts: &mut BTreeSet<usize>, trace: &mut TraceSink<'_>) -> Vec<usize> {
    let pref = compute_pref_track_patterns(inp.tech, inp.tracks);
    let mut dirty_classes: Vec<usize> = Vec::new();
    for &i in dirty {
        remove_from_insts_set(pa, i);
        let old = pa.class_of[i];
        let key = unique_class_key(inp, &pref, i);
        let new = compute_unique_class(inp, pa, key);
        emit(trace, || format!("VYC MSPA DIRTY inst={} same={}", inp.insts[i].name, u8::from(old == new)));
        if old == new {
            // A moved cell (a changed transform) is not modelled: the pass moves none.
            if update_skip_inst_term(inp, pa, i) {
                insert_class(pa, &mut dirty_classes, old);
            }
        } else {
            pa.classes[old].insts.retain(|&x| x != i);
            let is_new = pa.classes[new].insts.is_empty();
            let at = pa.classes[new].insts.partition_point(|&x| x < i);
            pa.classes[new].insts.insert(at, i);
            pa.class_of[i] = new;
            if is_new || update_skip_inst_term(inp, pa, i) {
                insert_class(pa, &mut dirty_classes, new);
            } else {
                pattern_insts.insert(i);
            }
        }
    }
    dirty_classes
}

/// `UniqueInsts::computeUniqueClassKey` with the instance's stubborn terminals.
fn unique_class_key(inp: &DirtyIn<'_>, pref: &[TrackPattern], i: usize) -> ClassKey {
    let u = &inp.insts[i].unique;
    let mut key = compute_unique_class_key(pref, master_pin_layer_range(inp.tech, &inp.masters[&u.master]), u, i);
    key.stubborn = inp.stubborn.range((i, 0)..=(i, usize::MAX)).map(|&(_, t)| t).collect();
    key
}

/// `UniqueInsts::computeUniqueClass`: the class with this key, made (empty, every terminal
/// skipped, not initialised) when there is none.
fn compute_unique_class(inp: &DirtyIn<'_>, pa: &mut PinAccess, key: ClassKey) -> usize {
    if let Some(c) = pa.classes.iter().position(|c| c.key == key) {
        return c;
    }
    let n = inp.masters[&key.master].terms.len();
    pa.classes.push(UniqueClass { key, insts: Vec::new(), routes: vec![false; n] });
    pa.reps.push(usize::MAX);
    pa.pa_idx.push(u32::MAX);
    pa.class_aps.push(Vec::new());
    pa.patterns.push(Vec::new());
    pa.classes.len() - 1
}

/// `FlexPA::updateSkipInstTerm`: a terminal the instance routes that its class skips.
fn update_skip_inst_term(inp: &DirtyIn<'_>, pa: &PinAccess, i: usize) -> bool {
    let class = &pa.classes[pa.class_of[i]];
    inp.insts[i].unique.routes.iter().zip(&class.routes).any(|(&r, &cr)| r && !cr)
}

/// Into the dirty-class set, kept in `UniqueClassKey` order.
fn insert_class(pa: &PinAccess, set: &mut Vec<usize>, c: usize) {
    if set.contains(&c) {
        return;
    }
    let at = set.partition_point(|&x| key_cmp(&pa.classes[x].key, &pa.classes[c].key) == Ordering::Less);
    set.insert(at, c);
}

/// `UniqueClassKey::operator<`: (master, orientation, offsets, NDR instance, stubborn terminals).
/// ⚠️ The reference compares the master, the NDR instance and the stubborn terminals as
/// POINTERS; here by name, index and terminal index. The order decides only which class takes
/// which pin-access index (and the reference's processing order), never an access point.
fn key_cmp(a: &ClassKey, b: &ClassKey) -> Ordering {
    (a.master.as_str(), orient_rank(&a.orient), &a.offsets, a.ndr_inst, &a.stubborn).cmp(&(b.master.as_str(), orient_rank(&b.orient), &b.offsets, b.ndr_inst, &b.stubborn))
}

/// `dbOrientType`'s values: R0, R90, R180, R270, MY, MYR90, MX, MXR90.
fn orient_rank(o: &str) -> usize {
    ["R0", "R90", "R180", "R270", "MY", "MYR90", "MX", "MXR90"].iter().position(|x| *x == o).unwrap_or(usize::MAX)
}

/// The second loop: each dirty class (key order) not initialised takes its master's next
/// pin-access index; every member becomes a pattern instance.
fn init_dirty_classes(pa: &mut PinAccess, dirty_classes: &[usize], pattern_insts: &mut BTreeSet<usize>) {
    for &c in dirty_classes {
        if pa.pa_idx[c] == u32::MAX {
            let master = &pa.classes[c].key.master;
            let n = (0..pa.classes.len()).filter(|&k| pa.pa_idx[k] != u32::MAX && &pa.classes[k].key.master == master).count();
            pa.pa_idx[c] = n as u32;
        }
        pattern_insts.extend(pa.classes[c].insts.iter().copied());
    }
}

/// The parallel loop's body for one dirty class: `initSkipInstTerm`, `genInstAccessPoints` on its
/// first instance (stubborn terminals avoiding the avoid list; points replaced), and for a
/// standard cell `prepPatternInst`.
fn gen_dirty_class_access(inp: &DirtyIn<'_>, pa: &mut PinAccess, c: usize, trace: &mut TraceSink<'_>, warn: &mut dyn FnMut(String)) -> Result<(), DirtyError> {
    let rep = pa.classes[c].insts[0];
    let n = pa.classes[c].routes.len();
    for t in 0..n {
        let routed = pa.classes[c].insts.iter().any(|&i| inp.insts[i].unique.routes[t]);
        pa.classes[c].routes[t] |= routed;
    }
    let inst = &inp.insts[rep];
    let master = &inp.masters[&pa.classes[c].key.master];
    if inst.class == MasterClass::Other {
        return Err(DirtyError::Unsupported(format!("lef58_max_spacing: pin access re-run on {} ({}), neither a standard cell nor a block", inst.name, pa.classes[c].key.master)));
    }
    if (0..n).any(|t| pa.classes[c].routes[t] && inst.no_taper[t]) {
        return Err(DirtyError::Unsupported(format!("lef58_max_spacing: pin access re-run on {}: a terminal on a non-default-rule net without auto-taper (its via trials read the design mid-routing)", inst.name)));
    }
    let cx = Context::new(inp.tech, inp.tracks);
    let stubborn = |t: usize| inp.stubborn.contains(&(rep, t));
    let idx = pa.pa_idx[c];
    let routes = pa.classes[c].routes.clone();
    let master_name = pa.classes[c].key.master.clone();
    // DRT-355 (upstream fix 0008): `inst/term (master)`, the instance's master.
    let mut fell_back = |t: usize| warn(format!("No access point for {}/{} ({master_name}) without the via defs avoided for max spacing; using all via defs.", inst.name, master.terms[t].name));
    let aps = gen_inst_access_points(&cx, inp.cfg, master, inst, &routes, None, &stubborn, Some(inp.avoid), &mut |t, got| {
        emit(trace, || format!("VYC MSPA APS inst={} term={} idx={idx} n={got} stubborn={}", inst.name, master.terms[t].name, u8::from(stubborn(t))));
    }, &mut fell_back);
    let aps = aps.map_err(|e| match e {
        FlowError::NoAccessPoint(name) => {
            let (i, t) = name.rsplit_once('/').map_or((name.clone(), String::new()), |(i, t)| (i.to_string(), t.to_string()));
            DirtyError::NoAccessPoint { inst: i, term: t, master: pa.classes[c].key.master.clone() }
        }
        FlowError::Unsupported(u) => DirtyError::Unsupported(format!("lef58_max_spacing: the pin-access re-run for the stubborn terminals of {} ({}) reaches {}, which is not modelled", inst.name, pa.classes[c].key.master, u.0)),
        FlowError::NoRowPath(r) => DirtyError::NoRowPath(r),
    })?;
    trace_access_points(inp, &aps, rep, master, &pa.classes[c].routes, trace);
    pa.class_aps[c] = aps;
    pa.reps[c] = rep;
    pa.patterns[c] = if inst.class == MasterClass::StdCell {
        let view = pattern_instance(inp.tech, inst, master, &pa.classes[c], &pa.class_aps[c]);
        prep_pattern_inst(&view, &mut Trace(None))
    } else {
        Vec::new()
    };
    if inst.class == MasterClass::StdCell {
        emit(trace, || format!("VYC MSPA PAT inst={} n={}", inst.name, pa.patterns[c].len()));
    }
    Ok(())
}

fn trace_access_points(inp: &DirtyIn<'_>, aps: &[Vec<Vec<crate::pa::access::AccessPoint>>], rep: usize, master: &Master, routes: &[bool], trace: &mut TraceSink<'_>) {
    if trace.is_none() {
        return;
    }
    for (t, term) in master.terms.iter().enumerate() {
        if !routes[t] {
            continue;
        }
        for (p, pin) in aps[t].iter().enumerate() {
            for (k, a) in pin.iter().enumerate() {
                let acc: String = a.access.iter().map(|&b| if b { '1' } else { '0' }).collect();
                let vias: String = a.vias.iter().map(|&v| format!("{},", inp.tech.via_defs[v].name)).collect();
                emit(trace, || {
                    format!("VYC MSPA AP inst={} term={} pin={p} k={k} pt=({},{}) layer={} type=({},{}) acc={acc} vias={vias}", inp.insts[rep].name, term.name, a.point.0, a.point.1, a.layer, a.lower as i32, a.upper as i32)
                });
            }
        }
    }
}

/// `frInstLocationComp`: the placed box's lower-left, y then x, then the instance id.
fn location_key(inp: &DirtyIn<'_>, i: usize) -> (i32, i32, usize) {
    let b = inp.insts[i].unique.bbox;
    (b.yl, b.xl, i)
}

/// `FlexPA::removeFromInstsSet`.
fn remove_from_insts_set(pa: &mut PinAccess, i: usize) {
    pa.insts_set.retain(|&x| x != i);
}

/// `FlexPA::isSkipInst`: every terminal skipped by its class.
fn is_skip_inst(pa: &PinAccess, i: usize) -> bool {
    pa.classes[pa.class_of[i]].routes.iter().all(|&r| !r)
}

/// `FlexPA::addToInstsSet`: a standard cell its class does not skip whole, at its place.
fn add_to_insts_set(inp: &DirtyIn<'_>, pa: &mut PinAccess, i: usize) {
    if pa.insts_set.contains(&i) || inp.insts[i].class != MasterClass::StdCell || is_skip_inst(pa, i) {
        return;
    }
    let k = location_key(inp, i);
    let at = pa.insts_set.partition_point(|&x| location_key(inp, x) < k);
    pa.insts_set.insert(at, i);
}

/// `FlexPA::instancesAreAbuting`: the same origin y, and the left one's (smaller origin x; on a
/// tie the SECOND argument) right edge on the other's left edge.
fn instances_are_abutting(inp: &DirtyIn<'_>, a: usize, b: usize) -> bool {
    let (oa, ob) = (inp.insts[a].transform.origin, inp.insts[b].transform.origin);
    if oa.1 != ob.1 {
        return false;
    }
    let (left, right) = if oa.0 < ob.0 { (a, b) } else { (b, a) };
    inp.insts[left].unique.bbox.xh == inp.insts[right].unique.bbox.xl
}

/// `FlexPA::getAdjacentInstancesCluster`: the seed, the abutting run before it (never reaching
/// the set's FIRST instance), then the abutting run after it, in placement order.
pub fn get_adjacent_instances_cluster(set: &[usize], k: usize, abut: &dyn Fn(usize, usize) -> bool) -> Vec<usize> {
    let mut cluster = vec![set[k]];
    if k != 0 {
        let (mut cur, mut prev) = (k, k - 1);
        while prev != 0 && abut(set[cur], set[prev]) {
            cluster.push(set[prev]);
            cur -= 1;
            prev -= 1;
        }
    }
    cluster.reverse();
    let (mut cur, mut next) = (k, k + 1);
    while next < set.len() && abut(set[cur], set[next]) {
        cluster.push(set[next]);
        cur += 1;
        next += 1;
    }
    cluster
}

/// The rows: per pattern instance in id order, not yet in a row, not skipped and a standard
/// cell, its cluster. Err: a seed missing from the placement set (the reference's DRT-9419).
fn adjacent_clusters(inp: &DirtyIn<'_>, pa: &PinAccess, pattern_insts: &BTreeSet<usize>) -> Result<Vec<Vec<usize>>, DirtyError> {
    let mut processed: BTreeSet<usize> = BTreeSet::new();
    let mut rows = Vec::new();
    let abut = |a: usize, b: usize| instances_are_abutting(inp, a, b);
    for &i in pattern_insts {
        if processed.contains(&i) || is_skip_inst(pa, i) || inp.insts[i].class != MasterClass::StdCell {
            continue;
        }
        let k = pa.insts_set.iter().position(|&x| x == i).ok_or_else(|| DirtyError::Unsupported(format!("lef58_max_spacing: instance {} not in the placement set (DRT-9419)", inp.insts[i].name)))?;
        let row = get_adjacent_instances_cluster(&pa.insts_set, k, &abut);
        processed.extend(row.iter().copied());
        rows.push(row);
    }
    Ok(rows)
}

/// `genInstRowPattern` per row, in order; each instance's pick written.
fn gen_inst_row_patterns(inp: &DirtyIn<'_>, pa: &mut PinAccess, rows: &[Vec<usize>]) -> Result<(), DirtyError> {
    for row in rows {
        let views: Vec<(usize, Instance<'_>)> = row
            .iter()
            .map(|&i| {
                let c = pa.class_of[i];
                (c, pattern_instance(inp.tech, &inp.insts[pa.reps[c]], &inp.masters[&pa.classes[c].key.master], &pa.classes[c], &pa.class_aps[c]))
            })
            .collect();
        let row_insts: Vec<_> = row.iter().zip(&views).map(|(&i, (c, v))| row_inst(inp.insts, inp.masters, &pa.classes, v, &pa.patterns[*c], *c, i)).collect();
        let chosen = gen_inst_row_pattern(inp.tech, &row_insts, &mut None).ok_or_else(|| DirtyError::NoRowPath(row.iter().map(|&i| inp.insts[i].name.clone()).collect::<Vec<_>>().join(" ")))?;
        drop(row_insts);
        drop(views);
        for (&i, p) in row.iter().zip(chosen) {
            pa.picks[i] = Some(p);
        }
    }
    Ok(())
}

fn trace_rows(inp: &DirtyIn<'_>, pa: &PinAccess, rows: &[Vec<usize>], trace: &mut TraceSink<'_>) {
    if trace.is_none() {
        return;
    }
    for row in rows {
        let names: String = row.iter().map(|&i| format!("{},", inp.insts[i].name)).collect();
        emit(trace, || format!("VYC MSPA ROW insts={names}"));
        for &i in row {
            let master = &inp.masters[&inp.insts[i].unique.master];
            let class = &pa.classes[pa.class_of[i]];
            for (t, term) in master.terms.iter().enumerate() {
                if !class.routes[t] {
                    continue;
                }
                let prefs = pa.pref_access_points(inp.insts, inp.masters, i, t);
                for p in 0..term.pins.len() {
                    let loc = inp.insts[i].unique.location;
                    let line = match prefs.get(p).copied().flatten() {
                        Some((q, l)) => format!("VYC MSPA PICK inst={} term={} pin={p} pt=({},{}) layer={l}", inp.insts[i].name, term.name, q.0 + loc.0, q.1 + loc.1),
                        None => format!("VYC MSPA PICK inst={} term={} pin={p} none", inp.insts[i].name, term.name),
                    };
                    emit(trace, || line);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Rule (`FlexPA::getAdjacentInstancesCluster`): the walk back from the seed stops BEFORE the
    /// set's first instance (`prev_inst_it != insts_set_.begin()`), the walk forward reaches the
    /// last; a seed at the first instance takes the whole run.
    #[test]
    fn the_cluster_walk_back_never_reaches_the_first_instance() {
        let set = [10, 11, 12, 13];
        let all = |_: usize, _: usize| true;
        assert_eq!(get_adjacent_instances_cluster(&set, 2, &all), [11, 12, 13]);
        assert_eq!(get_adjacent_instances_cluster(&set, 3, &all), [11, 12, 13]);
        assert_eq!(get_adjacent_instances_cluster(&set, 1, &all), [11, 12, 13]);
        assert_eq!(get_adjacent_instances_cluster(&set, 0, &all), [10, 11, 12, 13]);
        // A break in the run stops either walk.
        let not_12_13 = |a: usize, b: usize| !((a == 12 && b == 13) || (a == 13 && b == 12));
        assert_eq!(get_adjacent_instances_cluster(&set, 0, &not_12_13), [10, 11, 12]);
        assert_eq!(get_adjacent_instances_cluster(&set, 3, &not_12_13), [13]);
    }

    use crate::gc::Owner;
    use crate::pa::flow::pin_access;
    use crate::pa::unique::UniqueInst;
    use crate::polygon90::Rect;
    use crate::tech::{MasterPin, MasterTerm, Transform, ViaDef};

    /// The gc test technology (l2 → c3 → l4) with two single-cut vias on c3: the default `v` of
    /// cut class 0, and `b` of class 1 — as small as `v`, or a bar wider than the cell. Each cell
    /// has one pin on l2 between two obstructions.
    fn fixture(bar: bool) -> (Tech, HashMap<String, Master>, Vec<DesignInst>) {
        let mut t = crate::gc::tests::tech();
        let sq = |h: i32| Rect::new(-h, -h, h, h);
        t.via_defs.push(ViaDef { name: "v".into(), is_default: true, layer1: 2, cut: 3, layer2: 4, layer1_figs: vec![sq(85)], cut_figs: vec![sq(85)], layer2_figs: vec![sq(70)], cut_class: Some(0) });
        let b1 = if bar { Rect::new(-1500, -85, 1500, 85) } else { sq(85) };
        t.via_defs.push(ViaDef { name: "b".into(), is_default: false, layer1: 2, cut: 3, layer2: 4, layer1_figs: vec![b1], cut_figs: vec![sq(60)], layer2_figs: vec![sq(70)], cut_class: Some(1) });
        let pin = Rect::new(800, 800, 1200, 1200);
        let master = Master { terms: vec![MasterTerm { name: "A".into(), sig: "SIGNAL".into(), pins: vec![MasterPin { shapes: vec![(2, pin)] }] }], blockages: vec![], blockage_rules: vec![] };
        let inst = |name: &str, x: i32| DesignInst {
            name: name.into(),
            unique: UniqueInst { master: "m".into(), orient: "R0".into(), location: (x, 0), bbox: Rect::new(x, 0, x + 2000, 2000), routes: vec![true], ndr_no_taper: false },
            class: MasterClass::StdCell,
            is_block: false,
            transform: Transform { orient: "R0".into(), origin: (x, 0) },
            nets: vec![Some("n".into())],
            // The pin, and an obstruction 200 to either side of it (the bar runs into them).
            target: vec![(Owner::Net("n".into()), 2, Rect::new(x + 800, 800, x + 1200, 1200)), (Owner::Inst(name.into()), 2, Rect::new(x + 1400, 800, x + 2000, 1200)), (Owner::Inst(name.into()), 2, Rect::new(x, 800, x + 600, 1200))],
            no_taper: vec![false],
        };
        (t, [("m".to_string(), master)].into(), vec![inst("u0", 0), inst("u1", 5000)])
    }

    fn cfg() -> Config {
        Config { via_access_layer: 2, via_in_pin: None, top_routing_layer: 4, min_std_cell_points: 3, min_macro_points: 3, use_nonpref_tracks: true }
    }

    fn rerun(bar: bool, stubborn: &[(usize, usize)]) -> (PinAccess, Result<(), DirtyError>) {
        let (pa, r, _) = rerun_warned(bar, stubborn);
        (pa, r)
    }

    fn rerun_warned(bar: bool, stubborn: &[(usize, usize)]) -> (PinAccess, Result<(), DirtyError>, Vec<String>) {
        let (t, masters, insts) = fixture(bar);
        let c = cfg();
        let mut pa = pin_access(&t, &[], &c, &masters, &insts, &[]).expect("first pin access");
        let stubborn: BTreeSet<(usize, usize)> = stubborn.iter().copied().collect();
        let dirty: BTreeSet<usize> = stubborn.iter().map(|&(i, _)| i).collect();
        let avoid = BTreeSet::from([0]);
        let inp = DirtyIn { tech: &t, tracks: &[], cfg: &c, masters: &masters, insts: &insts, avoid: &avoid, stubborn: &stubborn };
        let mut warned = Vec::new();
        let r = update_dirty_insts(&inp, &mut pa, &dirty, &mut None, &mut |w| warned.push(w));
        (pa, r, warned)
    }

    /// Rule (`FlexPA::updateDirtyInsts`, `computeUniqueClassKey` with `stubborn_terms`): a dirty
    /// instance leaves its class for a NEW one keyed by its stubborn terminals (next pin-access
    /// index of the master), whose points are made on it with the avoided via defs skipped; the
    /// old class keeps its other members and its points (still relative to the instance they were
    /// made on); the dirty instance gets a pick from its row.
    #[test]
    fn a_stubborn_instance_moves_to_a_class_of_its_own_avoiding_the_class_vias() {
        let (pa, r) = rerun(false, &[(0, 0)]);
        r.expect("the small via fits");
        assert_eq!(pa.classes.len(), 2);
        assert_eq!((pa.classes[0].insts.clone(), pa.reps[0], pa.class_of[1]), (vec![1], 0, 0));
        assert_eq!((pa.classes[1].insts.clone(), pa.classes[1].key.stubborn.clone(), pa.pa_idx[1], pa.class_of[0]), (vec![0], vec![0], 1, 1));
        let aps: Vec<&crate::pa::access::AccessPoint> = pa.class_aps[1][0][0].iter().collect();
        assert!(!aps.is_empty());
        assert!(aps.iter().all(|a| a.vias == [1]), "{aps:?}");
        assert!(pa.class_aps[0][0][0].iter().all(|a| a.vias.first() == Some(&0)));
        assert!(pa.picks[0].is_some() && !pa.patterns[1].is_empty());
    }

    /// Rule (the reclassification loop): two dirty instances with the same key share ONE new
    /// class — the second joins the class the first made (not new, but every terminal of it is
    /// still skipped, so it is dirty all the same); its points are made on the first by id.
    #[test]
    fn two_dirty_instances_with_one_key_share_one_new_class() {
        let (pa, r) = rerun(false, &[(0, 0), (1, 0)]);
        r.expect("the small via fits");
        assert_eq!(pa.classes.len(), 2);
        assert!(pa.classes[0].insts.is_empty());
        assert_eq!((pa.classes[1].insts.clone(), pa.reps[1]), (vec![0, 1], 0));
        assert!(pa.picks[0].is_some() && pa.picks[1].is_some());
    }

    /// Rule (`genInstAccessPoints` with upstream fix 0008; the clean reference aborts with DRT-0073,
    /// OpenROAD issue #12199): once the avoided via defs are dropped a stubborn pin whose only other
    /// via cannot be placed has no access point; it is made again with every via def, so it keeps
    /// the via it had, and DRT-355 names the terminal and the master once.
    #[test]
    fn a_stubborn_pin_with_no_other_via_falls_back_to_every_via_def() {
        let (pa, r, warned) = rerun_warned(true, &[(0, 0)]);
        r.expect("the fallback finds the avoided via");
        assert_eq!(warned, ["No access point for u0/A (m) without the via defs avoided for max spacing; using all via defs."]);
        let aps = &pa.class_aps[pa.class_of[0]][0][0];
        assert!(!aps.is_empty() && aps.iter().all(|a| a.vias.first() == Some(&0)), "{aps:?}");
        // A pin the avoided defs do not empty warns nothing.
        let (_, r, warned) = rerun_warned(false, &[(0, 0)]);
        r.expect("the small via fits");
        assert!(warned.is_empty(), "{warned:?}");
    }

    /// Rule (`UniqueClass::setSkipTerm`, `skip_term_[t] &= skip`, from `initSkipInstTerm`): the
    /// skip map is AND-ed in, never overwritten, so a terminal a class once routed stays routed
    /// for it even when no current member routes it (a member that routed it has left).
    #[test]
    fn a_terminal_routed_once_stays_unskipped_for_its_class() {
        let (t, masters, insts) = fixture(false);
        let c = cfg();
        let mut pa = pin_access(&t, &[], &c, &masters, &insts, &[]).expect("first pin access");
        let stubborn = BTreeSet::from([(0, 0)]);
        let avoid = BTreeSet::from([0]);
        let inp = DirtyIn { tech: &t, tracks: &[], cfg: &c, masters: &masters, insts: &insts, avoid: &avoid, stubborn: &stubborn };
        update_dirty_insts(&inp, &mut pa, &BTreeSet::from([0]), &mut None, &mut |_| {}).expect("the small via fits");
        assert_eq!(pa.classes[1].routes, [true]);
        // The member that routed the terminal is gone; the one left does not route it.
        let (_, _, mut later) = fixture(false);
        later[0].unique.routes = vec![false];
        let inp = DirtyIn { insts: &later, ..inp };
        gen_dirty_class_access(&inp, &mut pa, 1, &mut None, &mut |_| {}).expect("the small via fits");
        assert_eq!(pa.classes[1].routes, [true], "a routed terminal was skipped again");
        assert!(!pa.class_aps[1][0][0].is_empty());
    }

    /// Rule (the reclassification loop, `is_new || updateSkipInstTerm(inst)`): an instance that
    /// joins an EXISTING class makes it dirty when it routes a terminal the class skips, and the
    /// class's points are made again; joining as no new route would only add it to the rows.
    #[test]
    fn joining_a_class_that_skips_a_routed_terminal_makes_it_dirty() {
        let (t, masters, insts) = fixture(false);
        let c = cfg();
        let mut pa = pin_access(&t, &[], &c, &masters, &insts, &[]).expect("first pin access");
        let avoid = BTreeSet::from([0]);
        let first = BTreeSet::from([(0, 0)]);
        let inp = DirtyIn { tech: &t, tracks: &[], cfg: &c, masters: &masters, insts: &insts, avoid: &avoid, stubborn: &first };
        update_dirty_insts(&inp, &mut pa, &BTreeSet::from([0]), &mut None, &mut |_| {}).expect("the small via fits");
        // The stubborn class as if its terminal were still skipped, with no points.
        pa.classes[1].routes = vec![false];
        pa.class_aps[1] = Vec::new();
        let second = BTreeSet::from([(0, 0), (1, 0)]);
        let inp = DirtyIn { stubborn: &second, ..inp };
        update_dirty_insts(&inp, &mut pa, &BTreeSet::from([1]), &mut None, &mut |_| {}).expect("the small via fits");
        assert_eq!((pa.classes[1].insts.clone(), pa.classes[1].routes.clone()), (vec![0, 1], vec![true]));
        assert!(pa.class_aps[1].first().is_some_and(|p| !p[0].is_empty()), "the joined class was not made again");
    }

    /// Rule (`UniqueClassKey::operator<`, `dbOrientType`): orientation compares by the enum's
    /// value, so MX (6) sorts after R180 (2) and MY (4).
    #[test]
    fn orientations_order_by_the_database_enum() {
        let mut o = ["MX", "R0", "MY", "R180"];
        o.sort_by_key(|x| orient_rank(x));
        assert_eq!(o, ["R0", "R180", "MY", "MX"]);
    }
}
