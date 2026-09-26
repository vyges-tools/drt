// SPDX-License-Identifier: Apache-2.0
//! `vyges-drt` — detailed routing, starting with pin access. See [`USAGE`].
use std::process::ExitCode;

use vyges_drt::pa::{db as padb, flow};
use vyges_drt::tech::read;
use vyges_opendb::Db;

const USAGE: &str = "vyges-drt — detailed routing

USAGE:
  vyges-drt pin_access (--db IN.odb | --lef A.lef [--lef B.lef …] --def D.def)
                       [--max-routing-layer LAYER] --out OUT.odb [-o REPORT.json]
  vyges-drt detailed_route --db IN.odb [--via-access-layer LAYER] --out OUT.def|OUT.odb
                       [-o REPORT.json]
  vyges-drt --describe
  vyges-drt --help
  vyges-drt --version

pin_access groups the instances into unique classes, finds every pin's access points (cost rounds
of candidates, each planar direction and via checked against the design rules), chooses one
access pattern per class and one per instance along each row of abutting cells, and writes the
results into the database: every master pin's access points, each routed instance terminal's
preferred points, each single-pin port's points. The report (JSON) goes to stdout.

detailed_route routes the design on its route guides: pin access, guide processing, track
assignment, then search-and-repair iterations until no design-rule marker stands, and writes the
routes (a DEF or a database). The routing layers are the block's minimum and maximum routing
layers as the database holds them; the non-default rules are the database's. Nets already routed
in the database are kept: their wires are read, the first three iterations reroute only the other
nets, and the report counts both (routed_before, rerouted). Refused: FIXED wiring, a wire using
one of the design's own vias, a routed net on a non-default rule.

OPTIONS:
  --db IN.odb                 read the design from a database
  --lef FILE / --def FILE     or from LEF files and a DEF
  --max-routing-layer LAYER   set the block's maximum routing layer first (as a
                              signal-layer range ending at LAYER does)
  --via-access-layer LAYER    detailed_route: the via-access layer (default the second
                              routing layer)
  --out OUT.odb               write the database here
  -o FILE                     write the JSON report to FILE instead of stdout
  --json                      accepted; the report is JSON either way
  --describe                  print a machine-readable JSON description of the command

EXIT STATUS:
  0  written   access points computed (or the design routed clean) and the output written
  1  markers   detailed_route: routing ended with design-rule markers standing; written
  2  vacuous   nothing to access: no routed instance terminal and no routed port. NOT a pass;
               nothing is written
  2  error     usage, unreadable input
  3  refused   a step this engine does not model (among them a multi-patterned routing
               layer), a terminal with no access point, or a row with no pattern
               combination — see `reason`
";

/// The pin, inherited from the database crate this binary links.
const CRATE_PIN: &str = vyges_opendb::OPENROAD_PIN;

/// ⛔ The `openroad_pin` FIELD is this token, substituted at print time — a hand-typed pin
/// reports what was typed, not what the binary links. A correlation claim in the prose names the
/// date it was MEASURED and stays a literal.
const PIN_TOKEN: &str = "@OPENROAD_PIN@";

fn describe() -> String {
    DESCRIBE.replace(PIN_TOKEN, CRATE_PIN)
}

/// ⚠️ **Two commands, one descriptor.** `invocation` is `detailed_route` — what a caller that
/// reads only the primary invocation (the MCP registry) should run; `commands` lists both, each
/// with its own assertion, as `vyges-dpl` does. ⚠️ `maturity` is `structured`: the correlation
/// runs against the reference outside this repository, so the `workflow-validated` rung is not
/// claimed.
const DESCRIBE: &str = r#"{
  "schema": "vyges-tool-descriptor/1.1",
  "openroad_pin": "@OPENROAD_PIN@",
  "name": "drt",
  "summary": "detailed routing on the design's route guides: pin access, track assignment and search-and-repair until no design-rule marker stands, the routes written as a DEF or a database; pin access alone is its own command",
  "maturity": "structured",
  "provenance_limitations": [
    "detailed_route status is one of written, markers, refused or error: written is routed with no design-rule marker standing (exit 0); markers is routed and written with markers standing (exit 1, a finding, not a pass); error is exit 2; refused is exit 3.",
    "pin_access status is one of written, vacuous, refused or error. VACUOUS IS NOT WRITTEN: the design has no routed instance terminal and no routed port, and nothing is written. Exit status is 0 for written, 2 for vacuous and for error, 3 for refused.",
    "detailed_route is correlated end to end on the DEF it writes, compared WHOLE and byte for byte against a fresh reference run: 16 of 16 cases, 2026-09-25 -- 14 of the reference router's own regression scripts (the largest 16,880 nets over 5 search-and-repair iterations), its incremental-routing script, and one constructed incremental case. Each stage (guides, rule tables, track assignment, every worker of every iteration) is also correlated against an instrumented reference. The correlation harness is not part of this repository.",
    "An incremental run's DEF cannot tell incremental rip-up from rerouting everything, which writes the same DEF: the report's routed_before and rerouted counts say which ran.",
    "pin_access is correlated stage by stage against an instrumented reference router, and end to end on the database it writes (preferred points, point counts, port points, pin-access indices) on sky130hs designs, and stage by stage on Nangate45 and GF180 designs, 2026-09-25.",
    "Design rules modelled: shorts (metal and cut), non-sufficient metal, the parallel-run spacing table, cut spacing (one plain rule per cut layer), LEF 5.4 end-of-line spacing (with or without a parallel edge), minimum width, minimum area, and rect-only layers (which are also unidirectional).",
    "REFUSED rather than approximated, by detailed_route: any other rule family a layer carries (LEF58 end-of-line forms, minimum step, corner spacing and the rest, named in the reason); a multi-patterned routing layer; LEF 5.4 spacing limited to a width range; a non-default rule with hard spacing, via generate rules or wire extension; FIXED wiring, a via or patch with no wire, or a routed net on a non-default rule already in the database; congested input guides; and a run still carrying markers at iteration 7, where the reference widens the clip of congested workers.",
    "REFUSED rather than approximated, by pin_access: a nearby-track cost round (a pin that no other round can reach); a multi-patterned routing layer.",
    "Taken as absent: a metal-width via map, right-way-on-grid-only layers.",
    "The router settings are its defaults: via-access layer the second routing layer (detailed_route takes --via-access-layer), no via-in-pin range, three sparse points per pin, non-preferred tracks allowed. detailed_route routes between the block's minimum and maximum routing layers as the database holds them, with the database's non-default rules; pin_access's top routing layer is the block's maximum routing layer, else the topmost."
  ],
  "invocation": {
    "args_template": ["detailed_route", "--db", "{db}", "--out", "{out}"],
    "optional": [ { "arg": "report", "flag": "-o" }, { "arg": "via_access_layer", "flag": "--via-access-layer" } ],
    "emits_json": true
  },
  "commands": [
    {
      "name": "detailed_route",
      "summary": "route the design on its route guides and write the routes (a .def or .odb out)",
      "args_template": ["detailed_route", "--db", "{db}", "--out", "{out}"],
      "optional": [ { "arg": "report", "flag": "-o" }, { "arg": "via_access_layer", "flag": "--via-access-layer" } ],
      "assertion": { "id": "routed-clean", "field": "status", "pass_when": { "eq": "written" } }
    },
    {
      "name": "pin_access",
      "summary": "compute every pin's access points and write them into the database",
      "args_template": ["pin_access", "--db", "{db}", "--out", "{out}"],
      "optional": [ { "arg": "report", "flag": "-o" }, { "arg": "max_routing_layer", "flag": "--max-routing-layer" } ],
      "assertion": { "id": "pin-access-written", "field": "status", "pass_when": { "eq": "written" } }
    }
  ],
  "inputs": {
    "type": "object",
    "required": ["db", "out"],
    "properties": {
      "db": { "type": "string", "description": "the design database to read (detailed_route: placed, with route guides)" },
      "out": { "type": "string", "description": "where to write the result: detailed_route writes a DEF if the name ends in .def, else a database; pin_access writes a database" },
      "via_access_layer": { "type": "string", "description": "detailed_route: the via-access layer (default the second routing layer)" },
      "max_routing_layer": { "type": "string", "description": "pin_access: the block's maximum routing layer, set first" },
      "report": { "type": "string", "description": "write the JSON report to FILE instead of stdout" }
    }
  },
  "consumes": ["odb"],
  "artifacts": [ { "role": "routed_design", "field": "out" } ],
  "assertion": {
    "id": "routed-clean",
    "field": "status",
    "pass_when": { "eq": "written" }
  },
  "exit_codes": { "0": "written", "1": "markers (detailed_route)", "2": "vacuous or error", "3": "refused" }
}"#;

struct Args {
    lefs: Vec<String>,
    def: Option<String>,
    db: Option<String>,
    out: Option<String>,
    report: Option<String>,
    max_layer: Option<String>,
    via_access: Option<String>,
}

fn parse(args: &[String]) -> Option<Args> {
    let mut a = Args { lefs: Vec::new(), def: None, db: None, out: None, report: None, max_layer: None, via_access: None };
    let mut it = args.iter();
    while let Some(k) = it.next() {
        // ⚠️ Accepted and ignored — the report is JSON either way. The descriptor says
        // `emits_json`, so a registry caller appends `--json`; rejecting it failed every such call.
        if k == "--json" {
            continue;
        }
        let v = it.next()?.clone();
        match k.as_str() {
            "--lef" => a.lefs.push(v),
            "--def" => a.def = Some(v),
            "--db" => a.db = Some(v),
            "--out" => a.out = Some(v),
            "-o" => a.report = Some(v),
            "--max-routing-layer" => a.max_layer = Some(v),
            "--via-access-layer" => a.via_access = Some(v),
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

fn route(a: &Args) -> Outcome {
    let Some(p) = &a.db else { return fail("error", 2, "detailed_route reads a database (--db)".into()) };
    let mut db = match Db::open(p) {
        Ok(d) => d,
        Err(e) => return fail("error", 2, e.to_string()),
    };
    let tech = match read::tech(&db) {
        Ok(t) => t,
        Err(e) => return fail("error", 2, e.to_string()),
    };
    let mut opts = vyges_drt::dr::run::Options::default();
    if let Some(l) = &a.via_access {
        match tech.layer_num(l) {
            Some(n) => opts.via_access_layer = Some(n),
            None => return fail("error", 2, format!("no layer {l}")),
        }
    }
    let s = match vyges_drt::dr::run::detailed_route(&mut db, &tech, &opts) {
        Ok(s) => s,
        Err(e) => return fail("refused", 3, e),
    };
    let out = a.out.as_ref().expect("--out");
    let written = if out.ends_with(".def") { db.write_def(out) } else { db.write(out) };
    if let Err(e) = written {
        return fail("error", 2, e.to_string());
    }
    Outcome {
        status: if s.markers == 0 { "written" } else { "markers" },
        code: u8::from(s.markers > 0),
        fields: vec![("out", json_str(out)), ("iterations", s.iterations.to_string()), ("markers", s.markers.to_string()), ("nets_written", s.nets_written.to_string()), ("routed_before", s.routed_before.to_string()), ("rerouted", s.rerouted.to_string())],
    }
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
    // A multi-patterned routing layer is unidirectional AND coloured; pin access does not model
    // colouring.
    let unidirectional: Vec<String> = vyges_drt::dr::db::unmodelled_rules(&db, &tech).into_iter().filter(|r| r.ends_with(": multi-patterned")).collect();
    if !unidirectional.is_empty() {
        return fail("refused", 3, format!("not modelled: {}", unidirectional.join("; ")));
    }
    let cfg = padb::config(&db, &tech);
    let (masters, insts, ports) = match padb::read_design(&db, &tech, cfg.top_routing_layer) {
        Ok(d) => d,
        Err(e) => return fail("error", 2, e),
    };
    if !insts.iter().any(|i| i.unique.routes.iter().any(|&r| r)) && !ports.iter().any(|p| p.routed) {
        return fail("vacuous", 2, "no routed instance terminal and no routed port".into());
    }
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
            println!("{}", describe());
            return ExitCode::SUCCESS;
        }
        Some("--help") | Some("-h") => {
            print!("{USAGE}");
            return ExitCode::SUCCESS;
        }
        Some("pin_access") | Some("detailed_route") => {}
        _ => {
            eprint!("{USAGE}");
            return ExitCode::from(2);
        }
    }
    let Some(a) = parse(&args[1..]) else {
        eprint!("{USAGE}");
        return ExitCode::from(2);
    };
    let o = if args[0] == "detailed_route" { route(&a) } else { run(&a) };
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
