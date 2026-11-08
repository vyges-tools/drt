// SPDX-License-Identifier: Apache-2.0
//! The `vyges-drt` command's contract, on a tiny design (`tests/data`): two buffers in one unique
//! class, a port, two nets — and the same design without nets.
//!
//! ⚠️ The written points below are a REGRESSION pin (this engine's own output on this design), not
//! a correlation claim; correlation against the reference router runs outside this repository.
#![cfg(feature = "odb")]

use std::path::PathBuf;
use std::process::Command;

use vyges_opendb::Db;

fn bin() -> Command {
    Command::new(env!("CARGO_BIN_EXE_vyges-drt"))
}

fn data(f: &str) -> String {
    format!("{}/tests/data/{f}", env!("CARGO_MANIFEST_DIR"))
}

fn tmp(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!("vyges-drt-{}-{name}", std::process::id()))
}

#[test]
fn help_describe_and_version_exit_zero() {
    for a in ["--help", "--describe", "--version"] {
        assert!(bin().arg(a).output().unwrap().status.success(), "{a}");
    }
    let d = String::from_utf8(bin().arg("--describe").output().unwrap().stdout).unwrap();
    assert!(d.contains("\"name\": \"drt\"") && d.contains("\"schema\": \"vyges-tool-descriptor/1.1\""));
}

fn descriptor() -> serde_json::Value {
    serde_json::from_slice(&bin().arg("--describe").output().unwrap().stdout).expect("--describe must be valid JSON")
}

/// ⛔ The pin the binary LINKS, not a typed one: the placeholder must not survive into the output.
#[test]
fn the_descriptor_reports_the_pin_this_binary_was_built_against() {
    let d = descriptor();
    assert_eq!(d["openroad_pin"], vyges_opendb::OPENROAD_PIN);
    assert_eq!(d["openroad_pin"].as_str().unwrap().len(), 40, "a full commit SHA");
}

/// Every assertion is `field` + `pass_when` with ONE predicate — the form the registry's schema
/// accepts. ⚠️ An `equals` key (what this descriptor once carried) is not a usable assertion: the
/// registry drops it and the verdict resolves `unknown`.
#[test]
fn every_assertion_is_a_pass_when_predicate() {
    let d = descriptor();
    let mut all = vec![d["assertion"].clone()];
    all.extend(d["tools"].as_array().expect("tools").iter().map(|c| c["assertion"].clone()));
    for a in all {
        assert_eq!(a["field"], "status", "{a}");
        assert_eq!(a["pass_when"]["eq"], "written", "{a}");
        assert!(a.get("equals").is_none(), "{a}");
    }
}

/// Both commands are described, and the primary invocation is `detailed_route` — the one a caller
/// that reads only `invocation` runs. `pin_access` is a `tools[]` entry (`drt.pin_access`), the
/// list `vyges mcp` registers, and its template names the command itself.
#[test]
fn both_commands_are_described_and_route_is_primary() {
    let d = descriptor();
    assert_eq!(d["invocation"]["args_template"][0], "detailed_route");
    let names: Vec<&str> = d["tools"].as_array().unwrap().iter().map(|c| c["name"].as_str().unwrap()).collect();
    assert_eq!(names, ["drt.pin_access"]);
    for c in d["tools"].as_array().unwrap() {
        assert_eq!(c["invocation"]["args_template"][0], c["name"].as_str().unwrap().trim_start_matches("drt."), "{c}");
    }
    assert!(["discovered", "structured", "workflow-validated"].contains(&d["maturity"].as_str().unwrap()));
}

/// `--json` is accepted anywhere and changes nothing: a registry caller appends it because the
/// descriptor declares `emits_json`, and rejecting it failed every such call with a usage error.
#[test]
fn json_is_accepted_and_changes_nothing() {
    let out = tmp("json.odb");
    let o = bin().args(["pin_access", "--json", "--lef", &data("tiny.lef"), "--def", &data("tiny.def"), "--out", out.to_str().unwrap(), "--json"]).output().unwrap();
    assert_eq!(o.status.code(), Some(0), "{}", String::from_utf8_lossy(&o.stderr));
    assert!(String::from_utf8(o.stdout).unwrap().contains("\"status\":\"written\""));
}

#[test]
fn a_bad_invocation_exits_two() {
    assert_eq!(bin().output().unwrap().status.code(), Some(2));
    assert_eq!(bin().args(["pin_access", "--def", "x.def", "--out", "y.odb"]).output().unwrap().status.code(), Some(2));
    assert_eq!(bin().args(["pin_access", "--db", "missing.odb", "--out", "y.odb"]).output().unwrap().status.code(), Some(2));
}

/// Nothing routed: VACUOUS, exit 2, nothing written.
#[test]
fn a_design_without_nets_is_vacuous() {
    let out = tmp("vacuous.odb");
    let o = bin().args(["pin_access", "--lef", &data("tiny.lef"), "--def", &data("tiny-no-nets.def"), "--out", out.to_str().unwrap()]).output().unwrap();
    assert_eq!(o.status.code(), Some(2));
    assert!(String::from_utf8(o.stdout).unwrap().contains("\"status\":\"vacuous\""));
    assert!(!out.exists());
}

/// Written: each connected terminal of a routed class gets a preferred point — `u2/Y` is
/// unconnected, so it gets none — and the port its point.
#[test]
fn the_tiny_design_is_written() {
    let out = tmp("tiny.odb");
    let o = bin().args(["pin_access", "--lef", &data("tiny.lef"), "--def", &data("tiny.def"), "--out", out.to_str().unwrap()]).output().unwrap();
    assert_eq!(o.status.code(), Some(0), "{}", String::from_utf8_lossy(&o.stderr));
    let report = String::from_utf8(o.stdout).unwrap();
    assert!(report.contains("\"status\":\"written\"") && report.contains("\"preferred_points\":3") && report.contains("\"port_points\":1"), "{report}");
    let db = Db::open(&out).unwrap();
    let pref = |i: &str, t: &str| db.iterm_pref_access_points(i, t).unwrap();
    assert_eq!(pref("u1", "A").len(), 1);
    assert_eq!(pref("u1", "Y").len(), 1);
    assert_eq!(pref("u2", "A").len(), 1);
    assert!(pref("u2", "Y").is_empty());
    // One class: both instances hold the same relative points.
    assert_eq!(pref("u1", "A"), pref("u2", "A"));
    // The regression pin: (x, y) relative to the instance, routing level; the port's absolute.
    assert_eq!(pref("u1", "A"), vec![(100, 300, 1)]);
    assert_eq!(pref("u1", "Y"), vec![(700, 700, 1)]);
    assert_eq!(db.bpin_access_points("in", 0).unwrap(), vec![(1100, 2300, 2)]);
    let _ = std::fs::remove_file(&out);
}

/// A DEF whose cells no LEF defines fails with a reason a user can act on: the file, the generic
/// error, its cause (the cell and instance) and what to do — not libodb's bare `ODB-0421`.
#[test]
fn a_def_with_undefined_cells_names_the_file_the_cause_and_the_fix() {
    let lef = std::fs::read_to_string(data("tiny.lef")).unwrap();
    let tech = tmp("tech.lef");
    std::fs::write(&tech, format!("{}\nEND LIBRARY\n", &lef[..lef.find("\nMACRO").unwrap()])).unwrap();
    let out = tmp("undefined.odb");
    let o = bin().args(["pin_access", "--lef", tech.to_str().unwrap(), "--def", &data("tiny.def"), "--out", out.to_str().unwrap()]).output().unwrap();
    assert_eq!(o.status.code(), Some(2));
    let r: serde_json::Value = serde_json::from_slice(&o.stdout).expect("stdout is the JSON report");
    let reason = r["reason"].as_str().unwrap();
    assert!(reason.starts_with(&format!("{}: ODB-0421: DEF parser returns an error!", data("tiny.def"))), "{reason}");
    assert!(reason.contains("caused by ODB-0092: unknown library cell referenced (BUF) for instance (u1)"), "{reason}");
    assert!(reason.contains("a LEF defining that cell was not read"), "{reason}");
}

/// The tiny technology with `extra` added inside layer `layer`.
fn tiny_lef_with(name: &str, layer: &str, extra: &str) -> PathBuf {
    let lef = std::fs::read_to_string(data("tiny.lef")).unwrap();
    let end = format!("END {layer}\n");
    let at = lef.find(&end).unwrap();
    let path = tmp(name);
    std::fs::write(&path, format!("{}{extra}{}", &lef[..at], &lef[at..])).unwrap();
    path
}

/// Rule (ours): a LEF58 end-of-line rule with TOCONCAVECORNER is SKIPPED with a warning — the
/// run is not refused, the rule is not checked (the access points are the base technology's),
/// and the report lists it in `skipped_rules`. The reference checks it as a plain end-of-line
/// rule with WITHIN 0 (upstream OpenROAD issue #12174); that reading is not copied.
#[test]
fn a_to_concave_corner_rule_is_skipped_loudly_and_listed_in_the_report() {
    let lef = tiny_lef_with("tcc.lef", "M1", "  PROPERTY LEF58_SPACING \"SPACING 0.09 ENDOFLINE 0.07 TOCONCAVECORNER MINADJACENTLENGTH 0.1 0.12 ;\" ;\n");
    let out = tmp("tcc.odb");
    let o = bin().args(["pin_access", "--lef", lef.to_str().unwrap(), "--def", &data("tiny.def"), "--out", out.to_str().unwrap()]).output().unwrap();
    let err = String::from_utf8_lossy(&o.stderr);
    assert_eq!(o.status.code(), Some(0), "{err}");
    let r: serde_json::Value = serde_json::from_slice(&o.stdout).expect("stdout is the JSON report");
    assert_eq!(r["status"], "written");
    assert_eq!(r["preferred_points"], 3, "the skipped rule changes nothing");
    let s = r["skipped_rules"].as_array().expect("skipped_rules");
    assert_eq!(s.len(), 1, "{r}");
    assert_eq!((s[0]["layer"].as_str(), s[0]["clause"].as_str(), s[0]["checked"].as_bool()), (Some("M1"), Some("TOCONCAVECORNER"), Some(false)));
    assert!(err.contains("[WARNING vyges-drt] layer M1: LEF58_SPACING end-of-line rule 0 uses TOCONCAVECORNER") && err.contains("#12174"), "{err}");
    let _ = std::fs::remove_file(&out);
}

/// Without a skipped rule the list is present and empty: absence never stands for "none".
#[test]
fn the_report_lists_no_skipped_rule_when_none_is_skipped() {
    let out = tmp("tiny-skips.odb");
    let o = bin().args(["pin_access", "--lef", &data("tiny.lef"), "--def", &data("tiny.def"), "--out", out.to_str().unwrap()]).output().unwrap();
    let r: serde_json::Value = serde_json::from_slice(&o.stdout).expect("stdout is the JSON report");
    assert_eq!(r["skipped_rules"], serde_json::json!([]));
    assert_eq!(r["lef58_not_checked"], serde_json::json!([]));
    let _ = std::fs::remove_file(&out);
}

/// Every other refusal stands: a LEF58 end-of-line SAMEMASK rule still refuses the run.
#[test]
fn another_end_of_line_clause_is_still_refused() {
    let lef = tiny_lef_with("eolsm.lef", "M1", "  PROPERTY LEF58_SPACING \"SPACING 0.09 ENDOFLINE 0.07 WITHIN 0.025 SAMEMASK ;\" ;\n");
    let out = tmp("eolsm.odb");
    let o = bin().args(["pin_access", "--lef", lef.to_str().unwrap(), "--def", &data("tiny.def"), "--out", out.to_str().unwrap()]).output().unwrap();
    assert_eq!(o.status.code(), Some(3));
    let r: serde_json::Value = serde_json::from_slice(&o.stdout).expect("stdout is the JSON report");
    assert_eq!(r["reason"], "not modelled: M1: lef58_spacing_eol (1, SAMEMASK)");
    assert!(!out.exists());
}

/// Informational: a LEF58 property the database reader leaves unparsed (no rule reaches routing)
/// is listed per layer and kind in `lef58_not_checked`; the run is not refused and the access
/// points do not change.
#[test]
fn an_unparsed_lef58_property_is_listed_not_refused() {
    let lef = tiny_lef_with("ee.lef", "V1", "  PROPERTY LEF58_ENCLOSUREEDGE \"ENCLOSUREEDGE 0.01 ;\" ;\n");
    let out = tmp("ee.odb");
    let o = bin().args(["pin_access", "--lef", lef.to_str().unwrap(), "--def", &data("tiny.def"), "--out", out.to_str().unwrap()]).output().unwrap();
    let err = String::from_utf8_lossy(&o.stderr);
    assert_eq!(o.status.code(), Some(0), "{err}");
    let r: serde_json::Value = serde_json::from_slice(&o.stdout).expect("stdout is the JSON report");
    assert_eq!(r["preferred_points"], 3);
    assert_eq!(r["lef58_not_checked"], serde_json::json!([{"layer": "V1", "property": "LEF58_ENCLOSUREEDGE", "statements": 1}]));
    assert_eq!(r["skipped_rules"], serde_json::json!([]));
    assert!(err.contains("[INFO vyges-drt] layer V1: LEF58_ENCLOSUREEDGE (1 statement(s)) is not parsed"), "{err}");
    let _ = std::fs::remove_file(&out);
}
