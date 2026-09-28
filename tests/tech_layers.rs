// SPDX-License-Identifier: Apache-2.0
//! The router's layer list (`tech::read::tech`) as the reference's reader builds it
//! (`io::Parser::setLayers`): a placeholder masterslice (0), then the technology's routing and cut
//! layers in order — a placeholder cut layer (1) only when a routing layer is read first; a cut
//! layer read first is layer 1 itself; a cut layer right above another is not read.
#![cfg(feature = "odb")]

use vyges_opendb::Db;

fn layers(lef: &str) -> Vec<String> {
    let path = std::env::temp_dir().join(format!("vyges-drt-layers-{}-{}.lef", std::process::id(), lef.len()));
    std::fs::write(&path, lef).expect("write lef");
    let mut db = Db::new();
    db.read_lef(path.to_str().expect("utf-8 path")).expect("read lef");
    let _ = std::fs::remove_file(&path);
    vyges_drt::tech::read::tech(&db).expect("tech").layers.into_iter().map(|l| l.name).collect()
}

const HEAD: &str = "VERSION 5.8 ;\nUNITS\n  DATABASE MICRONS 1000 ;\nEND UNITS\nMANUFACTURINGGRID 0.005 ;\n";

fn routing(name: &str, dir: &str) -> String {
    format!("LAYER {name}\n  TYPE ROUTING ;\n  DIRECTION {dir} ;\n  PITCH 0.2 ;\n  WIDTH 0.1 ;\n  SPACING 0.1 ;\nEND {name}\n")
}

fn cut(name: &str) -> String {
    format!("LAYER {name}\n  TYPE CUT ;\n  SPACING 0.1 ;\n  WIDTH 0.1 ;\nEND {name}\n")
}

// Rule: a cut layer read before any routing layer (asap7's V0) is layer 1, the masterslice
// placeholder (named after the last non-well masterslice read, here `Active`) below it.
#[test]
fn a_cut_layer_read_first_is_layer_one() {
    let lef = format!("{HEAD}LAYER Active\n  TYPE MASTERSLICE ;\nEND Active\n{}{}{}{}END LIBRARY\n", cut("V0"), routing("M1", "VERTICAL"), cut("V1"), routing("M2", "HORIZONTAL"));
    assert_eq!(layers(&lef), ["Active", "V0", "M1", "V1", "M2"]);
}

// Rule: a routing layer read first gets both placeholders below it.
#[test]
fn a_routing_layer_read_first_gets_both_placeholders() {
    let lef = format!("{HEAD}{}{}{}END LIBRARY\n", routing("M1", "VERTICAL"), cut("V1"), routing("M2", "HORIZONTAL"));
    assert_eq!(layers(&lef), ["FR_MASTERSLICE", "Fr_VIA", "M1", "V1", "M2"]);
}

// Rule: a cut layer whose lower layer is a cut layer is not read.
#[test]
fn a_cut_layer_above_a_cut_layer_is_not_read() {
    let lef = format!("{HEAD}{}{}{}{}END LIBRARY\n", routing("M1", "VERTICAL"), cut("V1"), cut("V1b"), routing("M2", "HORIZONTAL"));
    assert_eq!(layers(&lef), ["FR_MASTERSLICE", "Fr_VIA", "M1", "V1", "M2"]);
}
