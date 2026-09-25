// SPDX-License-Identifier: Apache-2.0
//! Routing against the design database: what the rule tables do not model, refused up front.

use vyges_opendb::Db;

use crate::tech::{LayerKind, Tech};

/// The rule families [`crate::dr::rules`] does not model, as `layer: family` for each layer that
/// carries one — a technology with any must be refused, not routed with the rule ignored. A cut
/// layer may carry one plain spacing rule (more than one is refused too).
pub fn unmodelled_rules(db: &Db, tech: &Tech) -> Vec<String> {
    let mut out = Vec::new();
    for l in &tech.layers {
        if l.kind == LayerKind::Placeholder {
            continue;
        }
        for (family, n) in db.layer_rule_census(&l.name) {
            let modelled = match family.as_str() {
                "cut_spacing" => n <= 1,
                "v55_influence" => true,
                _ => false,
            };
            if !modelled {
                out.push(format!("{}: {family} ({n})", l.name));
            }
        }
    }
    out
}
