// SPDX-License-Identifier: Apache-2.0
//! `vyges-drt` — detailed routing, starting with pin access.
//!
//! ```text
//! vyges-drt pin_access (--db IN.odb | --lef A.lef [--lef …] --def D.def) [--max-routing-layer L] --out OUT.odb
//! ```
//!
//! `--max-routing-layer` sets the block's maximum routing layer first (as `set_routing_layers
//! -signal …-L` does); the vias tried stop at it.
//!
//! Computes every pin's access points and each routed instance terminal's preferred ones, writes
//! them into the database, and saves it. Exit 0 on success, 2 on a usage or input error, 3 when
//! the pin access cannot be completed (a step not modelled, a terminal without access points, a
//! row without a pattern combination).
use std::process::ExitCode;

use vyges_drt::pa::{db as padb, flow};
use vyges_drt::tech::read;
use vyges_opendb::Db;

fn usage() -> ExitCode {
    eprintln!("usage: vyges-drt pin_access (--db IN.odb | --lef A.lef [--lef …] --def D.def) [--max-routing-layer L] --out OUT.odb");
    ExitCode::from(2)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) != Some("pin_access") {
        return usage();
    }
    let (mut lefs, mut def, mut dbin, mut out, mut max_layer) = (Vec::new(), None, None, None, None::<String>);
    let mut it = args[1..].iter();
    while let Some(a) = it.next() {
        let v = it.next().cloned();
        match (a.as_str(), v) {
            ("--lef", Some(v)) => lefs.push(v),
            ("--def", Some(v)) => def = Some(v),
            ("--db", Some(v)) => dbin = Some(v),
            ("--out", Some(v)) => out = Some(v),
            ("--max-routing-layer", Some(v)) => max_layer = Some(v),
            _ => return usage(),
        }
    }
    let Some(out) = out else { return usage() };
    let mut db = match (dbin, def) {
        (Some(p), None) if lefs.is_empty() => match Db::open(&p) {
            Ok(d) => d,
            Err(e) => {
                eprintln!("vyges-drt: {e}");
                return ExitCode::from(2);
            }
        },
        (None, Some(d)) if !lefs.is_empty() => {
            let mut db = Db::new();
            for l in &lefs {
                if let Err(e) = db.read_lef(l) {
                    eprintln!("vyges-drt: {e}");
                    return ExitCode::from(2);
                }
            }
            if let Err(e) = db.read_def(&d, "default") {
                eprintln!("vyges-drt: {e}");
                return ExitCode::from(2);
            }
            db
        }
        _ => return usage(),
    };
    let mut run = || -> Result<(), (u8, String)> {
        if let Some(l) = &max_layer {
            let level = db.layer_get_routing_level(l);
            if level <= 0 {
                return Err((2, format!("{l} is not a routing layer")));
            }
            db.block_set_max_routing_layer(level).map_err(|e| (2, e.to_string()))?;
        }
        let tech = read::tech(&db).map_err(|e| (2, e.to_string()))?;
        let tracks = read::tracks(&db, &tech).map_err(|e| (2, e.to_string()))?;
        let (masters, insts, ports) = padb::read_design(&db, &tech).map_err(|e| (2, e))?;
        let cfg = padb::config(&db, &tech);
        let pa = flow::pin_access(&tech, &tracks, &cfg, &masters, &insts, &ports).map_err(|e| (3, format!("{e:?}")))?;
        let n_pref = pa.picks.iter().filter(|p| p.is_some()).count();
        padb::update_db(&mut db, &tech, &masters, &insts, &ports, &pa).map_err(|e| (2, e))?;
        eprintln!("vyges-drt pin_access: {} classes, {} instances in rows", pa.classes.len(), n_pref);
        Ok(())
    };
    if let Err((code, msg)) = run() {
        eprintln!("vyges-drt: {msg}");
        return ExitCode::from(code);
    }
    if let Err(e) = db.write(&out) {
        eprintln!("vyges-drt: {e}");
        return ExitCode::from(2);
    }
    ExitCode::SUCCESS
}
