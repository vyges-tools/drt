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
