// SPDX-License-Identifier: Apache-2.0
//! The run's log lines (tool `DRT`), built from what each stage computed, in the order the
//! reference logs them at its default verbosity. The stages hand them to a [`Sink`] as they go,
//! so a refusal still leaves the lines of the stages before it.
//!
//! Lines whose value is a time or a memory size are never written.

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

/// The reader's opening lines (`readTechAndLibs`, then `readDesign`): `149`, the generated vias'
/// warnings (`initDefaultVias` runs inside the tech read), then `150`. `first_generated` is the
/// via count before generation: every via from it on was generated, in layer order.
pub fn reading(tech: &Tech, first_generated: usize) -> Vec<Line> {
    let mut out = vec![info(149, "Reading tech and libs.".into())];
    out.extend(tech.via_defs[first_generated..].iter().map(|v| Line::Warn(160, format!("Warning: {} does not have viaDef aligned with layer direction, generating new viaDef {}.", tech.layers[v.layer1].name, v.name))));
    out.push(info(150, "Reading design.".into()));
    out
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

/// The guide count, between empty reports (`readGuides`).
pub fn guides_read(n: usize) -> Vec<Line> {
    vec![Line::Report(String::new()), info(157, format!("Number of guides:     {n}")), Line::Report(String::new())]
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
        let got: Vec<String> = reading(&tech, 1).iter().map(text).collect();
        assert_eq!(got, ["I149 Reading tech and libs.", "W160 Warning: m1 does not have viaDef aligned with layer direction, generating new viaDef via_FR.", "I150 Reading design."]);
        let got: Vec<String> = default_vias(&tech, &[None, None, None, Some(1), None, None, None], 2).iter().map(text).collect();
        assert_eq!(got, ["I167 List of default vias:", "R   Layer via", "R     default via: via_FR", "R   Layer via2", "R     default via: none"]);
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
