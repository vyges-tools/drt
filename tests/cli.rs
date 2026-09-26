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
    all.extend(d["commands"].as_array().expect("commands").iter().map(|c| c["assertion"].clone()));
    for a in all {
        assert_eq!(a["field"], "status", "{a}");
        assert_eq!(a["pass_when"]["eq"], "written", "{a}");
        assert!(a.get("equals").is_none(), "{a}");
    }
}

/// Both commands are described, and the primary invocation is `detailed_route` — the one a caller
/// that reads only `invocation` runs. Each command's template names the command itself.
#[test]
fn both_commands_are_described_and_route_is_primary() {
    let d = descriptor();
    assert_eq!(d["invocation"]["args_template"][0], "detailed_route");
    let names: Vec<&str> = d["commands"].as_array().unwrap().iter().map(|c| c["name"].as_str().unwrap()).collect();
    assert_eq!(names, ["detailed_route", "pin_access"]);
    for c in d["commands"].as_array().unwrap() {
        assert_eq!(c["args_template"][0], c["name"], "{c}");
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
