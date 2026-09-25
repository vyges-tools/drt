// SPDX-License-Identifier: Apache-2.0
//! `vyges-drt` — detailed routing, starting with pin access. See [`USAGE`].
use std::process::ExitCode;

use vyges_drt::pa::{db as padb, flow};
use vyges_drt::tech::read;
use vyges_opendb::Db;

const USAGE: &str = "vyges-drt — detailed routing: pin access

USAGE:
  vyges-drt pin_access (--db IN.odb | --lef A.lef [--lef B.lef …] --def D.def)
                       [--max-routing-layer LAYER] --out OUT.odb [-o REPORT.json]
  vyges-drt --describe
  vyges-drt --help
  vyges-drt --version

pin_access groups the instances into unique classes, finds every pin's access points (cost rounds
of candidates, each planar direction and via checked against the design rules), chooses one
access pattern per class and one per instance along each row of abutting cells, and writes the
results into the database: every master pin's access points, each routed instance terminal's
preferred points, each single-pin port's points. The report (JSON) goes to stdout.

OPTIONS:
  --db IN.odb                 read the design from a database
  --lef FILE / --def FILE     or from LEF files and a DEF
  --max-routing-layer LAYER   set the block's maximum routing layer first (as a
                              signal-layer range ending at LAYER does)
  --out OUT.odb               write the database here
  -o FILE                     write the JSON report to FILE instead of stdout
  --describe                  print a machine-readable JSON description of the command

EXIT STATUS:
  0  written   access points computed and the database written
  2  vacuous   nothing to access: no routed instance terminal and no routed port. NOT a pass;
               nothing is written
  2  error     usage, unreadable input
  3  refused   a step this engine does not model, a terminal with no access point, or a row
               with no pattern combination — see `reason`
";

const DESCRIBE: &str = r#"{
  "schema": "vyges-tool-descriptor/1.1",
  "name": "drt",
  "summary": "pin access: every pin's access points and each routed instance terminal's preferred points, written into the design database",
  "maturity": "structured",
  "provenance_limitations": [
    "status is one of written, vacuous, refused or error. VACUOUS IS NOT WRITTEN: the design has no routed instance terminal and no routed port. Exit status is 0 for written, 2 for vacuous and for error, 3 for refused.",
    "Correlated stage by stage against an instrumented reference router, and end to end on the database it writes (preferred points, point counts, port points, pin-access indices) on sky130hs designs, and stage by stage on Nangate45 and GF180 designs, 2026-09-25. The correlation harness is not part of this repository.",
    "Design rules modelled in the access trials: shorts (metal and cut), non-sufficient metal, the parallel-run spacing table and cut spacing. A technology with end-of-line spacing is checked WITHOUT it — its trials may pass where they should not.",
    "REFUSED rather than approximated: a nearby-track cost round (a pin that no other round can reach).",
    "Taken as absent: a metal-width via map, unidirectional (multi-mask or rect-only) and right-way-on-grid-only layers, a net's non-default rule without auto-taper.",
    "The router settings are its defaults: via-access layer 2, no via-in-pin range, three sparse points per pin, non-preferred tracks allowed; the top routing layer is the block's maximum routing layer, else the topmost."
  ],
  "invocation": {
    "args_template": ["pin_access", "--db", "{db}", "--out", "{out}"],
    "optional": [ { "arg": "report", "flag": "-o" }, { "arg": "max_routing_layer", "flag": "--max-routing-layer" } ],
    "emits_json": true
  },
  "inputs": {
    "type": "object",
    "required": ["db", "out"],
    "properties": {
      "db": { "type": "string", "description": "the design database to read" },
      "out": { "type": "string", "description": "where to write the database with access points" },
      "max_routing_layer": { "type": "string", "description": "the block's maximum routing layer, set first" },
      "report": { "type": "string", "description": "write the JSON report to FILE instead of stdout" }
    }
  },
  "consumes": ["db"],
  "artifacts": [ { "role": "db", "field": "out" } ],
  "assertion": {
    "id": "pin-access-written",
    "field": "status",
    "equals": "written"
  },
  "exit_codes": { "0": "written", "2": "vacuous or error", "3": "refused" }
}"#;

struct Args {
    lefs: Vec<String>,
    def: Option<String>,
    db: Option<String>,
    out: Option<String>,
    report: Option<String>,
    max_layer: Option<String>,
}

fn parse(args: &[String]) -> Option<Args> {
    let mut a = Args { lefs: Vec::new(), def: None, db: None, out: None, report: None, max_layer: None };
    let mut it = args.iter();
    while let Some(k) = it.next() {
        let v = it.next()?.clone();
        match k.as_str() {
            "--lef" => a.lefs.push(v),
            "--def" => a.def = Some(v),
            "--db" => a.db = Some(v),
            "--out" => a.out = Some(v),
            "-o" => a.report = Some(v),
            "--max-routing-layer" => a.max_layer = Some(v),
            _ => return None,
        }
    }
    let one_source = a.db.is_some() != (a.def.is_some() || !a.lefs.is_empty());
    let def_with_lefs = a.def.is_some() == !a.lefs.is_empty();
    (a.out.is_some() && one_source && def_with_lefs).then_some(a)
}

fn json_str(s: &str) -> String {
    let mut o = String::from("\"");
    for c in s.chars() {
        match c {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            '\n' => o.push_str("\\n"),
            c if (c as u32) < 0x20 => o.push_str(&format!("\\u{:04x}", c as u32)),
            c => o.push(c),
        }
    }
    o.push('"');
    o
}

/// The outcome: status, exit code, and the report's other fields.
struct Outcome {
    status: &'static str,
    code: u8,
    fields: Vec<(&'static str, String)>,
}

fn fail(status: &'static str, code: u8, reason: String) -> Outcome {
    Outcome { status, code, fields: vec![("reason", json_str(&reason))] }
}

fn run(a: &Args) -> Outcome {
    let mut db = if let Some(p) = &a.db {
        match Db::open(p) {
            Ok(d) => d,
            Err(e) => return fail("error", 2, e.to_string()),
        }
    } else {
        let mut db = Db::new();
        for l in &a.lefs {
            if let Err(e) = db.read_lef(l) {
                return fail("error", 2, e.to_string());
            }
        }
        if let Err(e) = db.read_def(a.def.as_ref().expect("a DEF"), "default") {
            return fail("error", 2, e.to_string());
        }
        db
    };
    if let Some(l) = &a.max_layer {
        let level = db.layer_get_routing_level(l);
        if level <= 0 {
            return fail("error", 2, format!("{l} is not a routing layer"));
        }
        if let Err(e) = db.block_set_max_routing_layer(level) {
            return fail("error", 2, e.to_string());
        }
    }
    let tech = match read::tech(&db) {
        Ok(t) => t,
        Err(e) => return fail("error", 2, e.to_string()),
    };
    let tracks = match read::tracks(&db, &tech) {
        Ok(t) => t,
        Err(e) => return fail("error", 2, e.to_string()),
    };
    let (masters, insts, ports) = match padb::read_design(&db, &tech) {
        Ok(d) => d,
        Err(e) => return fail("error", 2, e),
    };
    if !insts.iter().any(|i| i.unique.routes.iter().any(|&r| r)) && !ports.iter().any(|p| p.routed) {
        return fail("vacuous", 2, "no routed instance terminal and no routed port".into());
    }
    let cfg = padb::config(&db, &tech);
    let pa = match flow::pin_access(&tech, &tracks, &cfg, &masters, &insts, &ports) {
        Ok(p) => p,
        Err(e) => return fail("refused", 3, format!("{e:?}")),
    };
    let mut pref = 0usize;
    for (i, inst) in insts.iter().enumerate() {
        for t in 0..masters[&inst.unique.master].terms.len() {
            pref += pa.pref_access_points(&insts, &masters, i, t).iter().flatten().count();
        }
    }
    let port_points: usize = ports.iter().zip(&pa.port_aps).filter(|(p, _)| p.routed && p.pins.len() == 1).map(|(_, a)| a[0].len()).sum();
    if let Err(e) = padb::update_db(&mut db, &tech, &masters, &insts, &ports, &pa) {
        return fail("error", 2, e);
    }
    let out = a.out.as_ref().expect("--out");
    if let Err(e) = db.write(out) {
        return fail("error", 2, e.to_string());
    }
    Outcome {
        status: "written",
        code: 0,
        fields: vec![
            ("out", json_str(out)),
            ("unique_classes", pa.classes.len().to_string()),
            ("instances_in_rows", pa.picks.iter().filter(|p| p.is_some()).count().to_string()),
            ("preferred_points", pref.to_string()),
            ("port_points", port_points.to_string()),
        ],
    }
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("--version") => {
            println!("vyges-drt {} ({})\nCopyright (c) Vyges. Apache-2.0.", env!("CARGO_PKG_VERSION"), env!("VYGES_GIT_SHA"));
            return ExitCode::SUCCESS;
        }
        Some("--describe") => {
            println!("{DESCRIBE}");
            return ExitCode::SUCCESS;
        }
        Some("--help") | Some("-h") => {
            print!("{USAGE}");
            return ExitCode::SUCCESS;
        }
        Some("pin_access") => {}
        _ => {
            eprint!("{USAGE}");
            return ExitCode::from(2);
        }
    }
    let Some(a) = parse(&args[1..]) else {
        eprint!("{USAGE}");
        return ExitCode::from(2);
    };
    let o = run(&a);
    let mut report = format!("{{\"tool\":\"drt\",\"status\":{}", json_str(o.status));
    for (k, v) in &o.fields {
        report.push_str(&format!(",\"{k}\":{v}"));
    }
    report.push_str("}\n");
    match &a.report {
        Some(p) => {
            if let Err(e) = std::fs::write(p, &report) {
                eprintln!("vyges-drt: {e}");
                return ExitCode::from(2);
            }
        }
        None => print!("{report}"),
    }
    ExitCode::from(o.code)
}
