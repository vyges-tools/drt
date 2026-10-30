// SPDX-License-Identifier: Apache-2.0
//! The run's log lines (tool `DRT`), built from what each stage computed, in the order the
//! reference logs them at its default verbosity. The stages hand them to a [`Sink`] as they go,
//! so a refusal still leaves the lines of the stages before it.
//!
//! Lines whose value is a time or a memory size are never written.

use crate::dr::cost::DrFig;
use crate::gc::Rule;
use crate::tech::{LayerKind, Tech};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Line {
    Info(u16, String),
    Warn(u16, String),
    /// `logger_->report`: text with no prefix.
    Report(String),
}

/// Where a stage writes its lines.
pub type Sink<'a> = &'a mut dyn FnMut(Line);

fn info(id: u16, text: String) -> Line {
    Line::Info(id, text)
}

/// What `readTechAndLibs` reports at its end.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct TechStats {
    pub dbu: i32,
    /// Every via the router holds: technology, block, generated.
    pub vias: usize,
    /// Every master of every library.
    pub masters: usize,
    pub via_rule_generates: usize,
}

/// The reader's opening lines (`readTechAndLibs`, then `readDesign`): `149`, the skipped rules'
/// warnings (`setLayers`, layer by layer), the generated vias' warnings (`initDefaultVias` runs
/// inside the tech read), the tech's counts between empty reports, then `150`. `first_generated`
/// is the via count before generation: every via from it on was generated, in layer order.
pub fn reading(tech: &Tech, first_generated: usize, skipped_rules: &[(u16, String)], stats: TechStats) -> Vec<Line> {
    let mut out = vec![info(149, "Reading tech and libs.".into())];
    out.extend(skipped_rules.iter().map(|(id, t)| Line::Warn(*id, t.clone())));
    out.extend(tech.via_defs[first_generated..].iter().map(|v| Line::Warn(160, format!("Warning: {} does not have viaDef aligned with layer direction, generating new viaDef {}.", tech.layers[v.layer1].name, v.name))));
    out.extend([
        Line::Report(String::new()),
        Line::Report(format!("Units:                {}", stats.dbu)),
        Line::Report(format!("Number of layers:     {}", tech.layers.len())),
        Line::Report(format!("Number of macros:     {}", stats.masters)),
        Line::Report(format!("Number of vias:       {}", stats.vias)),
        Line::Report(format!("Number of viarulegen: {}", stats.via_rule_generates)),
        Line::Report(String::new()),
    ]);
    out.push(info(150, "Reading design.".into()));
    out
}

/// What `readDesign` reports at its end.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct DesignStats {
    pub name: String,
    pub die: (i32, i32, i32, i32),
    pub track_patterns: usize,
    pub components: usize,
    pub terminals: usize,
    pub snets: usize,
    pub nets: usize,
}

/// `readDesign`'s counts between empty reports. "DEF vias" is the vias the DESIGN read added, and
/// nothing in it adds one (the block's vias are read with the technology), so it is always 0.
pub fn design_stats(d: &DesignStats) -> Vec<Line> {
    let (x0, y0, x1, y1) = d.die;
    [
        String::new(),
        format!("Design:                   {}", d.name),
        format!("Die area:                 ( {x0} {y0} ) ( {x1} {y1} )"),
        format!("Number of track patterns: {}", d.track_patterns),
        "Number of DEF vias:       0".into(),
        format!("Number of components:     {}", d.components),
        format!("Number of terminals:      {}", d.terminals),
        format!("Number of snets:          {}", d.snets),
        format!("Number of nets:           {}", d.nets),
        String::new(),
    ].into_iter().map(Line::Report).collect()
}

/// The default-via list (`printDefaultVias`), over the cut layers from the bottom routing layer up.
pub fn default_vias(tech: &Tech, defaults: &[Option<usize>], bottom_routing_layer: usize) -> Vec<Line> {
    let mut out = vec![info(167, "List of default vias:".into())];
    for (l, layer) in tech.layers.iter().enumerate() {
        if layer.kind != LayerKind::Cut || l < bottom_routing_layer {
            continue;
        }
        out.push(Line::Report(format!("  Layer {}", layer.name)));
        let name = defaults.get(l).copied().flatten().map_or("none", |v| tech.via_defs[v].name.as_str());
        out.push(Line::Report(format!("    default via: {name}")));
    }
    out
}

/// One `  Complete <layer><suffix>.` line per layer, in layer order (a region query's trees built).
pub fn layers_complete(tech: &Tech, id: u16, suffix: &str) -> Vec<Line> {
    tech.layers.iter().map(|l| info(id, format!("  Complete {}{suffix}.", l.name))).collect()
}

/// One `<layer> <what> region query size = N.` line per layer, in layer order. The layer name is
/// `frLayer::getName(out)`, which spells the fake cut layer `FR_VIA`; the by-value `getName()`
/// (the `Complete` lines, and our layer name) spells it `Fr_VIA`.
pub fn region_query_sizes(tech: &Tech, id: u16, what: &str, sizes: &[usize]) -> Vec<Line> {
    tech.layers.iter().enumerate().map(|(l, x)| {
        let name = if x.kind == LayerKind::Placeholder && x.name == "Fr_VIA" { "FR_VIA" } else { x.name.as_str() };
        info(id, format!("{name} {what} region query size = {}.", sizes.get(l).copied().unwrap_or(0)))
    }).collect()
}

/// A counter's progress lines: counting 1..=`n`, a line at each multiple of `small`, or of `big`
/// once the count is past `big`.
pub fn progress(id: u16, n: usize, small: usize, big: usize, text: impl Fn(usize) -> String) -> Vec<Line> {
    (1..=n).filter(|&c| c % if c > big { big } else { small } == 0).map(|c| info(id, text(c))).collect()
}

/// Pin access's lines after its header: points (`76`, `78`), patterns (`79`, `81`), rows (`82`,
/// `84`), then `166`. `pins` counts every terminal of each standard-cell or macro class's
/// representative; `patterns` the standard-cell classes; `groups` the instance rows.
pub fn pin_access(pins: usize, patterns: usize, groups: usize) -> Vec<Line> {
    let mut out = progress(76, pins, 1000, 10000, |c| format!("  Complete {c} pins."));
    out.push(info(78, format!("  Complete {pins} pins.")));
    out.extend(progress(79, patterns, 100, 1000, |c| format!("  Complete {c} unique inst patterns.")));
    out.push(info(81, format!("  Complete {patterns} unique inst patterns.")));
    out.extend(progress(82, groups, 10000, 100000, |c| format!("  Complete {c} groups.")));
    out.push(info(84, format!("  Complete {groups} groups.")));
    out.push(info(166, "Complete pin access.".into()));
    out
}

/// A guide counter's lines (`logGuidesRead`, `initOrigGuide`): every 100000th count below one
/// million, then every millionth — under `big_id` from one million on (`initOrigGuide` switches
/// 26 to 27; `logGuidesRead` keeps 156).
pub fn guide_progress(n: usize, small_id: u16, big_id: u16, text: impl Fn(usize) -> String) -> Vec<Line> {
    (1..=n).filter_map(|c| match c < 1_000_000 {
        true if c % 100_000 == 0 => Some(info(small_id, text(c))),
        false if c % 1_000_000 == 0 => Some(info(big_id, text(c))),
        _ => None,
    }).collect()
}

/// `readGuides`: a progress line per 100000 guides read, then the count between empty reports.
pub fn guides_read(n: usize) -> Vec<Line> {
    let mut out = guide_progress(n, 156, 156, |c| format!("guideIn read {c} guides."));
    out.extend([Line::Report(String::new()), info(157, format!("Number of guides:     {n}")), Line::Report(String::new())]);
    out
}

/// `initOrigGuide`: the same guides, counted again as they enter the region query.
pub fn orig_guides(n: usize) -> Vec<Line> {
    guide_progress(n, 26, 27, |c| format!("  Complete {c} origin guides."))
}

/// The gcell grid, per axis its first line, count and step.
pub fn gcell_grid(x: (i32, i32, i32), y: (i32, i32, i32)) -> Vec<Line> {
    vec![info(176, format!("GCELLGRID X {} DO {} STEP {} ;", x.0, x.1, x.2)), info(177, format!("GCELLGRID Y {} DO {} STEP {} ;", y.0, y.1, y.2))]
}

/// One track-assignment pass: wires assigned and panels, per direction in the order run. The
/// code and the order of the two halves follow the direction run first (`183`/`184` initial,
/// `268`/`186` repair; horizontal first takes the lower code of each pair).
pub fn ta_pass(iter: usize, h_first: bool, first: (usize, usize), second: (usize, usize)) -> Line {
    let id = match (iter, h_first) {
        (0, true) => 183,
        (0, false) => 184,
        (_, true) => 268,
        (_, false) => 186,
    };
    let (a, b) = if h_first { ("horizontal", "vertical") } else { ("vertical", "horizontal") };
    info(id, format!("Done with {} {a} wires in {} frboxes and {} {b} wires in {} frboxes.", first.0, first.1, second.0, second.1))
}

/// An iteration's lines: its header (`195`), then the markers standing after it (`199`); a
/// SKIPPED iteration logs `200` and returns before the count.
pub fn iteration(iter: usize, flow: &str, markers: usize) -> Vec<Line> {
    if flow == "skip" {
        return vec![info(200, format!("Skipping iteration {iter}"))];
    }
    vec![info(195, format!("Start {iter}{} {flow} iteration.", crate::dr::flow::iteration_suffix(iter))), info(199, format!("  Number of violations = {markers}."))]
}

/// `frConstraint::getViolName` for each rule the check reports, after `reportIterationViolations`'
/// relabels (`Lef58SpacingEndOfLine` → `EOL`, `Lef58CutSpacingTable` → `CutSpcTbl`,
/// `Lef58EolKeepOut` → `eolKeepOut`). Several constraint kinds share a name (every metal spacing
/// rule is `Metal Spacing`, both cut spacings `Cut Spacing`).
pub fn viol_name(rule: Rule) -> &'static str {
    match rule {
        Rule::Short => "Short",
        Rule::NonSufficientMetal => "NS Metal",
        Rule::MetalSpacing => "Metal Spacing",
        Rule::CutSpacing => "Cut Spacing",
        Rule::EolSpacing => "EOL Spacing",
        Rule::Recheck => "Recheck",
        Rule::MinWidth => "Min Width",
        Rule::RectOnly => "Rect Only",
        Rule::MinArea => "Min Area",
        Rule::MinEnclosedArea => "Min Hole",
        Rule::Lef58EolKeepOut => "eolKeepOut",
        Rule::Lef58SpacingEndOfLine => "EOL",
        Rule::CornerSpacing => "Corner Spacing",
        Rule::Lef58CutSpacingTable => "CutSpcTbl",
        Rule::MinStep => "Min Step",
        Rule::MinimumCut => "Minimum Cut",
        Rule::SpacingRange => "SpacingRange",
        Rule::Lef58Enclosure => "Lef58Enclosure",
    }
}

/// `reportIterationViolations`' table, after `199` when markers stand: a column per layer that has
/// one (layer order; a name of 7 or more characters shortened to its first two, `..`, its last two),
/// a row per violation name (byte order; 15 or more characters cut to 12 and `..`).
pub fn violation_table(tech: &Tech, markers: impl Iterator<Item = (Rule, usize)>) -> Vec<Line> {
    use std::collections::{BTreeMap, BTreeSet};
    let mut by_name: BTreeMap<&str, BTreeMap<usize, u32>> = BTreeMap::new();
    let mut layers = BTreeSet::new();
    for (rule, layer) in markers {
        *by_name.entry(viol_name(rule)).or_default().entry(layer).or_default() += 1;
        layers.insert(layer);
    }
    if by_name.is_empty() {
        return Vec::new();
    }
    let mut head = format!("{:<15}", "Viol/Layer");
    for &l in &layers {
        let n = &tech.layers[l].name;
        let n = if n.len() >= 7 { format!("{}..{}", &n[..2], &n[n.len() - 2..]) } else { n.clone() };
        head += &format!("{n:>7}");
    }
    let mut out = vec![Line::Report(head)];
    for (name, counts) in &by_name {
        let name = if name.len() >= 15 { format!("{}..", &name[..12]) } else { (*name).to_string() };
        let mut row = format!("{name:<15}");
        for l in &layers {
            row += &format!("{:>7}", counts.get(l).copied().unwrap_or(0));
        }
        out.push(Line::Report(row));
    }
    out
}

/// A double as a default `std::ostream` writes it: six significant digits, trailing zeros dropped
/// (`%g`). Only the percentages in `[0, 100]` reach it, so the exponent form never applies.
fn ostream_double(x: f64) -> String {
    if x == 0.0 {
        return "0".into();
    }
    let digits = 6 - 1 - x.abs().log10().floor() as i32;
    let s = format!("{:.*}", digits.max(0) as usize, x);
    if s.contains('.') { s.trim_end_matches('0').trim_end_matches('.').to_string() } else { s }
}

/// `FlexDR::end`'s report on the routes: wire length (path segments, end to end) in whole microns,
/// in total and per routing layer, the via count, and the up-via summary per cut layer, named by the
/// layer below it — single-cut only, or single/multi-cut columns once any via has more than one cut.
pub fn route_stats<'a>(tech: &Tech, dbu: i64, figs: impl Iterator<Item = &'a DrFig>) -> Vec<Line> {
    let n = tech.layers.len();
    let (mut wlen, mut s_cut, mut m_cut) = (vec![0u64; n], vec![0u64; n], vec![0u64; n]);
    for f in figs {
        match *f {
            DrFig::Seg { layer, begin, end, .. } => wlen[layer] += ((begin.0 - end.0).unsigned_abs() + (begin.1 - end.1).unsigned_abs()) as u64,
            DrFig::Via { via, .. } => {
                let v = &tech.via_defs[via];
                if v.cut_figs.len() > 1 { m_cut[v.cut] += 1 } else { s_cut[v.cut] += 1 }
            }
            DrFig::Patch { .. } => {}
        }
    }
    let dbu = dbu.max(1) as u64;
    let (tot_w, tot_s, tot_m): (u64, u64, u64) = (wlen.iter().sum(), s_cut.iter().sum(), m_cut.iter().sum());
    let tot = tot_s + tot_m;
    let mut out = vec![Line::Report(format!("Total wire length = {} um.", tot_w / dbu))];
    for (l, layer) in tech.layers.iter().enumerate() {
        if layer.kind == LayerKind::Routing {
            out.push(Line::Report(format!("Total wire length on LAYER {} = {} um.", layer.name, wlen[l] / dbu)));
        }
    }
    out.push(Line::Report(format!("Total number of vias = {tot}.")));
    if tot_m > 0 {
        out.push(Line::Report(format!("Total number of multi-cut vias = {tot_m} ({:5.1}%).", tot_m as f64 * 100.0 / tot as f64)));
        out.push(Line::Report(format!("Total number of single-cut vias = {tot_s} ({:5.1}%).", tot_s as f64 * 100.0 / tot as f64)));
    }
    out.push(Line::Report(format!("Up-via summary (total {tot}):")));
    let cuts: Vec<usize> = (1..n).filter(|&l| tech.layers[l].kind == LayerKind::Cut).collect();
    let name_len = cuts.iter().map(|&l| tech.layers[l - 1].name.len()).max().unwrap_or(0);
    let (ws, wm, wt) = (tot_s.to_string().len(), tot_m.to_string().len(), tot.to_string().len());
    let mut max_l = 1 + name_len + 4 + ws;
    if tot_m > 0 {
        max_l += 9 + 4 + wm + 9 + 4 + wt;
    }
    let pct = |a: u64, b: u64| ostream_double(if b > 0 { a as f64 * 100.0 / b as f64 } else { 0.0 });
    let mut msg = String::new();
    if tot_m > 0 {
        msg += &format!(" {:>w1$}{:>w2$}{:>w3$}", "single-cut", "multi-cut", "total", w1 = name_len + 4 + ws + 9, w2 = 4 + wm + 9, w3 = 4 + wt);
    }
    msg.push('\n');
    msg += &"-".repeat(max_l);
    msg.push('\n');
    for &l in &cuts {
        msg += &format!(" {:>name_len$}    {:>ws$}", tech.layers[l - 1].name, s_cut[l]);
        if tot_m > 0 {
            let all = s_cut[l] + m_cut[l];
            msg += &format!(" ({:>5}%)    {:>wm$} ({:>5}%)    {:>wt$}", pct(s_cut[l], all), m_cut[l], pct(m_cut[l], all), all);
        }
        msg.push('\n');
    }
    msg += &"-".repeat(max_l);
    msg.push('\n');
    msg += &format!(" {:>name_len$}{:>ws$}", "    ", tot_s);
    if tot_m > 0 {
        msg += &format!(" ({:>5}%)    {:>wm$} ({:>5}%)    {:>wt$}", pct(tot_s, tot), tot_m, pct(tot_m, tot), tot);
    }
    msg += "\n\n";
    out.extend(msg.split('\n').map(|l| Line::Report(l.to_string())));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::polygon90::Rect;
    use crate::tech::{Layer, ViaDef};

    fn text(l: &Line) -> String {
        match l {
            Line::Info(i, t) => format!("I{i} {t}"),
            Line::Warn(i, t) => format!("W{i} {t}"),
            Line::Report(t) => format!("R {t}"),
        }
    }

    /// Rule (readTechAndLibs, initDefaultVias, readDesign, printDefaultVias): the generated vias'
    /// warnings come inside the tech read, between `149` and `150`, and each names the layer
    /// BELOW its cut; the list skips cut layers under the bottom routing layer and says `none`
    /// for a cut layer without a default.
    #[test]
    fn default_vias_warn_then_list() {
        let l = |name: &str, kind| Layer { name: name.into(), kind, ..Layer::default() };
        let v = |name: &str, layer1| ViaDef { name: name.into(), is_default: false, layer1, cut: layer1 + 1, layer2: layer1 + 2, layer1_figs: vec![Rect::new(0, 0, 1, 1)], cut_figs: vec![], layer2_figs: vec![] };
        let tech = Tech {
            layers: vec![l("li", LayerKind::Routing), l("mcon", LayerKind::Cut), l("m1", LayerKind::Routing), l("via", LayerKind::Cut), l("m2", LayerKind::Routing), l("via2", LayerKind::Cut), l("m3", LayerKind::Routing)],
            via_defs: vec![v("V1", 2), v("via_FR", 2)],
            ..Tech::default()
        };
        let skipped = [(349, "LEF58_ENCLOSURE with no CUTCLASS is not supported. Skipping for layer mcon".to_string())];
        let stats = TechStats { dbu: 1000, vias: 2, masters: 437, via_rule_generates: 25 };
        let got: Vec<String> = reading(&tech, 1, &skipped, stats).iter().map(text).collect();
        assert_eq!(got, ["I149 Reading tech and libs.", "W349 LEF58_ENCLOSURE with no CUTCLASS is not supported. Skipping for layer mcon",
                         "W160 Warning: m1 does not have viaDef aligned with layer direction, generating new viaDef via_FR.",
                         "R ", "R Units:                1000", "R Number of layers:     7", "R Number of macros:     437", "R Number of vias:       2", "R Number of viarulegen: 25", "R ",
                         "I150 Reading design."]);
        let got: Vec<String> = default_vias(&tech, &[None, None, None, Some(1), None, None, None], 2).iter().map(text).collect();
        assert_eq!(got, ["I167 List of default vias:", "R   Layer via", "R     default via: via_FR", "R   Layer via2", "R     default via: none"]);
    }

    fn sky130_stack() -> Tech {
        let l = |name: &str, kind| Layer { name: name.into(), kind, ..Layer::default() };
        use LayerKind::*;
        Tech {
            layers: vec![l("FR_MASTERSLICE", Placeholder), l("licon", Cut), l("li1", Routing), l("mcon", Cut), l("met1", Routing), l("via", Cut),
                         l("met2", Routing), l("via2", Cut), l("met3", Routing), l("via3", Cut), l("met4", Routing), l("via4", Cut), l("met5", Routing)],
            ..Tech::default()
        }
    }

    /// Rule (reportIterationViolations): layers with a marker in layer order, names in byte order,
    /// relabelled. Witness: the reference on the ascon flow, iteration 0.
    #[test]
    fn violation_table_as_the_reference_prints_it() {
        let t = sky130_stack();
        let mut m = Vec::new();
        let mut add = |r: Rule, l: usize, n: usize| m.extend(std::iter::repeat_n((r, l), n));
        add(Rule::CutSpacing, 3, 10);
        add(Rule::MetalSpacing, 4, 2077); add(Rule::MetalSpacing, 6, 425); add(Rule::MetalSpacing, 8, 59); add(Rule::MetalSpacing, 10, 8);
        add(Rule::MinEnclosedArea, 4, 12);
        add(Rule::Recheck, 2, 9); add(Rule::Recheck, 4, 810); add(Rule::Recheck, 6, 487); add(Rule::Recheck, 8, 154); add(Rule::Recheck, 10, 90);
        add(Rule::Short, 4, 9278); add(Rule::Short, 5, 8); add(Rule::Short, 6, 1529); add(Rule::Short, 8, 102); add(Rule::Short, 10, 32);
        let got: Vec<String> = violation_table(&t, m.into_iter()).iter().map(text).collect();
        assert_eq!(got, ["R Viol/Layer         li1   mcon   met1    via   met2   met3   met4",
                         "R Cut Spacing          0     10      0      0      0      0      0",
                         "R Metal Spacing        0      0   2077      0    425     59      8",
                         "R Min Hole             0      0     12      0      0      0      0",
                         "R Recheck              9      0    810      0    487    154     90",
                         "R Short                0      0   9278      8   1529    102     32"]);
        assert!(violation_table(&t, std::iter::empty()).is_empty());
    }

    /// Rule (FlexDR::end): whole microns by integer division; the up-via summary names each cut by
    /// the layer below it, padded to the longest such name. Witness: the reference, iteration 0.
    #[test]
    fn route_stats_as_the_reference_prints_them() {
        let mut t = sky130_stack();
        let v = |name: &str, cut: usize, cuts: usize| ViaDef { name: name.into(), is_default: true, layer1: cut - 1, cut, layer2: cut + 1, layer1_figs: vec![], cut_figs: vec![Rect::new(0, 0, 1, 1); cuts], layer2_figs: vec![] };
        t.via_defs = vec![v("L1M1", 3, 1), v("M1M2", 5, 1), v("M2M3", 7, 1), v("M2M3x2", 7, 2)];
        let seg = |layer: usize, len: i32| DrFig::Seg { layer, begin: (0, 0), end: (len, 0), width: 1, begin_ext: 0, end_ext: 0, bi: (0, 0, 0), ei: (0, 0, 0), tapered: false, begin_trunc: false, end_trunc: false };
        let via = |via: usize| DrFig::Via { via, origin: (0, 0), bi: (0, 0, 0), ei: (0, 0, 0), tapered: false, bottom_connected: false, top_connected: false };
        let mut figs = vec![seg(4, 256_728_999), seg(6, 288_254_000)];
        figs.extend(std::iter::repeat_n(via(0), 36455));
        figs.extend(std::iter::repeat_n(via(1), 48766));
        let got: Vec<String> = route_stats(&t, 1000, figs.iter()).iter().map(text).collect();
        assert_eq!(got, ["R Total wire length = 544982 um.", "R Total wire length on LAYER li1 = 0 um.", "R Total wire length on LAYER met1 = 256728 um.",
                         "R Total wire length on LAYER met2 = 288254 um.", "R Total wire length on LAYER met3 = 0 um.", "R Total wire length on LAYER met4 = 0 um.",
                         "R Total wire length on LAYER met5 = 0 um.", "R Total number of vias = 85221.", "R Up-via summary (total 85221):", "R ",
                         "R ------------------------", "R  FR_MASTERSLICE        0", "R             li1    36455", "R            met1    48766",
                         "R            met2        0", "R            met3        0", "R            met4        0", "R ------------------------",
                         "R                85221", "R ", "R "]);
        // A multi-cut via adds the columns and the share lines; a percentage prints as `%g` does.
        figs.push(via(3));
        let got: Vec<String> = route_stats(&t, 1000, figs.iter()).iter().map(text).collect();
        assert_eq!(got[8], "R Total number of multi-cut vias = 1 (  0.0%).");
        assert_eq!(got[9], "R Total number of single-cut vias = 85221 (100.0%).");
        // Widths: name 14 + 4 + "85221" 5 + 9 = 32; 4 + "1" + 9 = 14; 4 + "85222" 5 = 9.
        assert_eq!(got[11], format!("R  {:>32}{:>14}{:>9}", "single-cut", "multi-cut", "total"));
        assert_eq!(got[16], "R            met2        0 (    0%)    1 (  100%)        1");
        assert_eq!(ostream_double(99.99882664), "99.9988");
    }

    /// Rule (readDesign): the counts between empty reports; the die as odb prints a Rect; DEF vias
    /// are those the design read itself added, never any (block vias load with the technology).
    #[test]
    fn design_stats_block() {
        let d = DesignStats { name: "ascon_core".into(), die: (0, 0, 520000, 520000), track_patterns: 12, components: 14124, terminals: 928, snets: 2, nets: 10225 };
        let got: Vec<String> = design_stats(&d).iter().map(text).collect();
        assert_eq!(got, ["R ", "R Design:                   ascon_core", "R Die area:                 ( 0 0 ) ( 520000 520000 )",
                         "R Number of track patterns: 12", "R Number of DEF vias:       0", "R Number of components:     14124",
                         "R Number of terminals:      928", "R Number of snets:          2", "R Number of nets:           10225", "R "]);
    }

    /// Rule (logGuidesRead, initOrigGuide): a line per 100000 guides below one million, per
    /// million from it on; the origin-guide count changes its code (26 -> 27) at one million.
    #[test]
    fn guide_progress_lines() {
        let got: Vec<String> = guides_read(203_880).iter().map(text).collect();
        assert_eq!(got, ["I156 guideIn read 100000 guides.", "I156 guideIn read 200000 guides.", "R ", "I157 Number of guides:     203880", "R "]);
        let got: Vec<String> = orig_guides(2_000_000).iter().map(text).collect();
        assert_eq!(got.len(), 9 + 2);
        assert_eq!(got[8], "I26   Complete 900000 origin guides.");
        assert_eq!(got[9], "I27   Complete 1000000 origin guides.");
        assert!(orig_guides(99_999).is_empty());
    }

    /// Rule (frRegionQuery::print, frLayer::getName): the size lines name the fake cut layer
    /// `FR_VIA` (the out-parameter overload); every other layer by its own name.
    #[test]
    fn region_query_names_the_fake_cut_fr_via() {
        let l = |name: &str, kind| Layer { name: name.into(), kind, ..Layer::default() };
        let tech = Tech { layers: vec![l("FR_MASTERSLICE", LayerKind::Placeholder), l("Fr_VIA", LayerKind::Placeholder), l("m1", LayerKind::Routing)], ..Tech::default() };
        let got: Vec<String> = region_query_sizes(&tech, 33, "shape", &[0, 0, 4]).iter().map(text).collect();
        assert_eq!(got, ["I33 FR_MASTERSLICE shape region query size = 0.", "I33 FR_VIA shape region query size = 0.", "I33 m1 shape region query size = 4."]);
        assert_eq!(text(&layers_complete(&tech, 24, "")[1]), "I24   Complete Fr_VIA.");
    }

    /// Rule (genAllAccessPoints, prepPattern, prepPatternInstRows): a progress line at every
    /// multiple of the small step, of the big step once past it; then the totals.
    #[test]
    fn pin_access_progress_steps() {
        let got: Vec<String> = pin_access(2000, 250, 3).iter().map(text).collect();
        assert_eq!(got, ["I76   Complete 1000 pins.", "I76   Complete 2000 pins.", "I78   Complete 2000 pins.", "I79   Complete 100 unique inst patterns.", "I79   Complete 200 unique inst patterns.", "I81   Complete 250 unique inst patterns.", "I84   Complete 3 groups.", "I166 Complete pin access."]);
        assert_eq!(progress(1, 30000, 1000, 10000, |c| c.to_string()).len(), 10 + 2);
    }

    /// Rule (initTA, searchRepair): the code and the halves' order follow the direction run first.
    #[test]
    fn ta_pass_lines() {
        assert_eq!(text(&ta_pass(0, false, (139, 1), (82, 1))), "I184 Done with 139 vertical wires in 1 frboxes and 82 horizontal wires in 1 frboxes.");
        assert_eq!(text(&ta_pass(1, true, (5, 2), (6, 3))), "I268 Done with 5 horizontal wires in 2 frboxes and 6 vertical wires in 3 frboxes.");
    }

    /// Rule (searchRepair): a skipped iteration returns before `reportIterationViolations`.
    #[test]
    fn a_skipped_iteration_has_no_violation_count() {
        assert_eq!(iteration(4, "skip", 7).iter().map(text).collect::<Vec<_>>(), ["I200 Skipping iteration 4"]);
        assert_eq!(iteration(21, "optimization", 7).iter().map(text).collect::<Vec<_>>(), ["I195 Start 21st optimization iteration.", "I199   Number of violations = 7."]);
    }
}
