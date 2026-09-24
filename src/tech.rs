// SPDX-License-Identifier: Apache-2.0
//! The technology and design as the router reads them.
//!
//! Rules:
//! - layers are numbered from the first routing layer, with a placeholder masterslice (0) and a
//!   placeholder cut layer (1) below it; routing and cut layers then follow in technology order;
//! - a routing layer's min width is `min(min width, width)`;
//! - a via definition is a technology via: its shapes on the layer below, the cut and the layer
//!   above, and whether it is a default via;
//! - a routing layer's minimum spacing is its parallel-run spacing table; a plain SPACING value is
//!   a table of one entry (width 0, run length 0);
//! - a cut layer's spacing is its plain SPACING value, edge to edge.

use crate::polygon90::{Polygon90Set, Rect};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LayerKind {
    Routing,
    Cut,
    /// The placeholders below the first routing layer.
    #[default]
    Placeholder,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Dir {
    Horizontal,
    Vertical,
    #[default]
    None,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Layer {
    pub name: String,
    pub kind: LayerKind,
    pub dir: Dir,
    pub width: i32,
    pub min_width: i32,
    pub pitch: i32,
    /// The width of a wire run across the layer's direction.
    pub wrong_way_width: i32,
    /// A routing layer's minimum spacing.
    pub spacing: Option<SpacingTable>,
    /// A cut layer's minimum spacing, edge to edge.
    pub cut_spacing: Option<i32>,
}

/// A parallel-run spacing table: rows by width, columns by parallel run length.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SpacingTable {
    pub widths: Vec<i32>,
    pub prls: Vec<i32>,
    pub values: Vec<Vec<i32>>,
}

impl SpacingTable {
    /// The spacing for a width and a run length: the row of the LAST width strictly below
    /// `width` and the column of the last run length strictly below `prl` — the first row or
    /// column when there is none. ⛔ Strictly below: a width equal to a row's is the row before.
    pub fn find(&self, width: i32, prl: i32) -> i32 {
        let idx = |axis: &[i32], v: i32| axis.iter().filter(|&&a| a < v).count().saturating_sub(1);
        self.values[idx(&self.widths, width)][idx(&self.prls, prl)]
    }
    /// The first row's first value.
    pub fn find_min(&self) -> i32 {
        self.values[0][0]
    }
    /// The last row's last value.
    pub fn find_max(&self) -> i32 {
        *self.values.last().and_then(|r| r.last()).expect("a value")
    }
}

impl Layer {
    pub fn is_horizontal(&self) -> bool {
        self.dir == Dir::Horizontal
    }
    pub fn is_vertical(&self) -> bool {
        self.dir == Dir::Vertical
    }
    pub fn is_routable(&self) -> bool {
        self.kind == LayerKind::Routing
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ViaDef {
    pub name: String,
    pub is_default: bool,
    /// Layer numbers: below, cut, above.
    pub layer1: usize,
    pub cut: usize,
    pub layer2: usize,
    pub layer1_figs: Vec<Rect>,
    pub cut_figs: Vec<Rect>,
    pub layer2_figs: Vec<Rect>,
}

impl ViaDef {
    /// The bounding box of the shapes on the layer below, at the origin.
    pub fn layer1_bbox(&self) -> Rect {
        bbox(&self.layer1_figs)
    }
    pub fn layer2_bbox(&self) -> Rect {
        bbox(&self.layer2_figs)
    }
}

fn bbox(figs: &[Rect]) -> Rect {
    let mut b = figs[0];
    for f in &figs[1..] {
        b = Rect { xl: b.xl.min(f.xl), yl: b.yl.min(f.yl), xh: b.xh.max(f.xh), yh: b.yh.max(f.yh) };
    }
    b
}

/// A track pattern: `num` tracks from `start` every `spacing`, on `layer`; `vertical_tracks` when
/// the tracks are vertical lines (the pattern steps in x).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TrackPattern {
    pub layer: usize,
    pub vertical_tracks: bool,
    pub start: i32,
    pub num: i32,
    pub spacing: i32,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Tech {
    pub layers: Vec<Layer>,
    pub manufacturing_grid: i32,
    /// Technology vias, in technology order.
    pub via_defs: Vec<ViaDef>,
}

impl Tech {
    pub fn top_layer_num(&self) -> usize {
        self.layers.len() - 1
    }
    pub fn bottom_layer_num(&self) -> usize {
        0
    }
    /// The topmost routing layer (the default top routing layer).
    pub fn top_routing_layer(&self) -> usize {
        (0..self.layers.len()).rev().find(|&l| self.layers[l].kind == LayerKind::Routing).expect("a routing layer")
    }
    pub fn layer_num(&self, name: &str) -> Option<usize> {
        self.layers.iter().position(|l| l.name == name)
    }
}

/// A master's pin: its shapes by layer number (master coordinates).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MasterPin {
    pub shapes: Vec<(usize, Rect)>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MasterTerm {
    pub name: String,
    /// The signal type (`SIGNAL`, `POWER`, `GROUND`, …).
    pub sig: String,
    pub pins: Vec<MasterPin>,
}

/// A master as the router imports it: its terminals, and its obstructions as blockages.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Master {
    pub terms: Vec<MasterTerm>,
    /// Blockage rectangles by layer number (master coordinates).
    pub blockages: Vec<(usize, Rect)>,
}

impl Master {
    /// The import rules for obstructions:
    /// - an obstruction on a CUT layer that touches the shapes of exactly ONE pin on the layer
    ///   above becomes a shape of that pin (a contact drawn as an obstruction is the pin's own);
    /// - every other obstruction merges with the rest on its layer, and the merged shapes'
    ///   MAXIMAL rectangles are the blockages.
    pub fn import(tech: &Tech, mut terms: Vec<MasterTerm>, obstructions: &[(usize, Rect)]) -> Master {
        let touches = |a: &Rect, b: &Rect| a.xl <= b.xh && b.xl <= a.xh && a.yl <= b.yh && b.yl <= a.yh;
        let mut merged: Vec<Polygon90Set> = vec![Polygon90Set::new(); tech.layers.len()];
        for &(layer, r) in obstructions {
            if tech.layers[layer].kind == LayerKind::Cut {
                let mut owner: Option<(usize, usize)> = None;
                let mut many = false;
                for (t, term) in terms.iter().enumerate() {
                    for (p, pin) in term.pins.iter().enumerate() {
                        if pin.shapes.iter().any(|(l, s)| *l == layer + 1 && touches(s, &r)) {
                            match owner {
                                None => owner = Some((t, p)),
                                Some(o) if o != (t, p) => many = true,
                                _ => {}
                            }
                        }
                    }
                }
                if let (Some((t, p)), false) = (owner, many) {
                    terms[t].pins[p].shapes.push((layer, r));
                    continue;
                }
            }
            merged[layer].insert_rect(r);
        }
        let mut blockages = Vec::new();
        for (layer, set) in merged.iter_mut().enumerate() {
            for r in set.max_rectangles() {
                blockages.push((layer, r));
            }
        }
        Master { terms, blockages }
    }
}

/// A placed instance's transform: an orientation, then the origin.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Transform {
    pub orient: String,
    pub origin: (i32, i32),
}

impl Transform {
    /// The orientations as the database applies them to a point: `R90` → `(-y, x)`, `R180` →
    /// `(-x, -y)`, `R270` → `(y, -x)`, `MY` → `(-x, y)`, `MX` → `(x, -y)`, `MYR90` → `(-y, -x)`,
    /// `MXR90` → `(y, x)`; then the origin is added.
    pub fn apply(&self, r: Rect) -> Rect {
        let f = |(x, y): (i32, i32)| -> (i32, i32) {
            let (x, y) = match self.orient.as_str() {
                "R90" => (-y, x),
                "R180" => (-x, -y),
                "R270" => (y, -x),
                "MY" => (-x, y),
                "MYR90" => (-y, -x),
                "MX" => (x, -y),
                "MXR90" => (y, x),
                _ => (x, y),
            };
            (x + self.origin.0, y + self.origin.1)
        };
        let (a, b) = (f((r.xl, r.yl)), f((r.xh, r.yh)));
        Rect::new(a.0, a.1, b.0, b.1)
    }
}

#[cfg(feature = "odb")]
pub mod read {
    //! The technology, masters and tracks from the design database.
    use super::*;
    use vyges_opendb::Db;

    type Res<T> = Result<T, Box<dyn std::error::Error>>;

    pub fn tech(db: &Db) -> Res<Tech> {
        let mut layers: Vec<Layer> = Vec::new();
        let placeholder = |name: &str| Layer { name: name.into(), ..Layer::default() };
        for (name, dir) in db.layers_with_direction()? {
            let kind = db.layer_get_type(&name)?;
            match kind.as_str() {
                "ROUTING" if db.layer_get_routing_level(&name) > 0 => {
                    if layers.is_empty() {
                        layers.push(placeholder("FR_MASTERSLICE"));
                        layers.push(placeholder("Fr_VIA"));
                    }
                    let width = db.layer_get_width(&name) as i32;
                    let min_width = (db.layer_get_min_width(&name) as i32).min(width);
                    let dir = match dir.as_str() {
                        "HORIZONTAL" => Dir::Horizontal,
                        "VERTICAL" => Dir::Vertical,
                        _ => Dir::None,
                    };
                    let pitch = db.layer_get_pitch(&name) as i32;
                    let wrong_way_width = db.layer_get_wrong_way_width(&name) as i32;
                    let v55 = db.layer_v55_spacing_table(&name)?;
                    let spacing = match (v55.widths_and_lengths, v55.table) {
                        (Some((w, l)), Some(t)) => Some(SpacingTable {
                            widths: w.iter().map(|&v| v as i32).collect(),
                            prls: l.iter().map(|&v| v as i32).collect(),
                            values: t.iter().map(|r| r.iter().map(|&v| v as i32).collect()).collect(),
                        }),
                        _ => match db.layer_get_spacing(&name) {
                            s if s > 0 => Some(SpacingTable { widths: vec![0], prls: vec![0], values: vec![vec![s]] }),
                            _ => None,
                        },
                    };
                    layers.push(Layer { name, kind: LayerKind::Routing, dir, width, min_width, pitch, wrong_way_width, spacing, cut_spacing: None });
                }
                "CUT" if !layers.is_empty() => {
                    let width = db.layer_get_width(&name) as i32;
                    let cut_spacing = Some(db.layer_get_spacing(&name)).filter(|&s| s > 0);
                    layers.push(Layer { name, kind: LayerKind::Cut, width, cut_spacing, ..Layer::default() });
                }
                _ => {}
            }
        }
        let num_of = |n: i64| -> Option<usize> {
            let name = db.layer_name_by_number(n);
            layers.iter().position(|l| l.name == name)
        };
        let mut via_defs = Vec::new();
        for via in db.tech_get_vias() {
            let boxes = db.tech_via_boxes(&via)?;
            let mut nums: Vec<usize> = boxes.iter().filter_map(|b| num_of(b.0)).collect();
            nums.sort();
            nums.dedup();
            let [l1, cut, l2] = nums[..] else { continue };
            let figs = |layer: usize| -> Vec<Rect> { boxes.iter().filter(|b| num_of(b.0) == Some(layer)).map(|b| Rect::new(b.1, b.2, b.3, b.4)).collect() };
            via_defs.push(ViaDef { name: via.clone(), is_default: db.techvia_is_default(&via), layer1: l1, cut, layer2: l2, layer1_figs: figs(l1), cut_figs: figs(cut), layer2_figs: figs(l2) });
        }
        Ok(Tech { layers, manufacturing_grid: db.tech_get_manufacturing_grid(), via_defs })
    }

    /// Every track pattern of the block, per layer: x patterns (vertical tracks), then y.
    pub fn tracks(db: &Db, tech: &Tech) -> Res<Vec<TrackPattern>> {
        let mut out = Vec::new();
        for (layer, l) in tech.layers.iter().enumerate() {
            if l.kind != LayerKind::Routing {
                continue;
            }
            let Ok((x, y)) = db.track_patterns(&l.name) else { continue };
            for (start, num, spacing) in x {
                out.push(TrackPattern { layer, vertical_tracks: true, start, num, spacing });
            }
            for (start, num, spacing) in y {
                out.push(TrackPattern { layer, vertical_tracks: false, start, num, spacing });
            }
        }
        Ok(out)
    }

    /// A master's terminals and their pins, by layer number (routing and cut shapes only).
    pub fn master_terms(db: &Db, tech: &Tech, master: &str) -> Res<Vec<MasterTerm>> {
        let mut out = Vec::new();
        for (term, sig) in db.master_mterms(master)? {
            let mut pins = Vec::new();
            for p in 0..db.num_mpins(master, &term) {
                let shapes = db
                    .mpin_boxes(master, &term, p)?
                    .into_iter()
                    .filter_map(|(n, x0, y0, x1, y1)| tech.layer_num(&db.layer_name_by_number(n)).map(|l| (l, Rect::new(x0, y0, x1, y1))))
                    .collect();
                pins.push(MasterPin { shapes });
            }
            out.push(MasterTerm { name: term, sig, pins });
        }
        Ok(out)
    }

    /// A master's obstructions, by layer number (the technology's layers only).
    pub fn master_obstructions(db: &Db, tech: &Tech, master: &str) -> Res<Vec<(usize, Rect)>> {
        Ok(db
            .master_obstruction_boxes(master)?
            .into_iter()
            .filter_map(|(n, x0, y0, x1, y1)| tech.layer_num(&db.layer_name_by_number(n)).map(|l| (l, Rect::new(x0, y0, x1, y1))))
            .collect())
    }

    pub fn transform(db: &Db, inst: &str) -> Transform {
        Transform { orient: db.inst_get_orient(inst), origin: (db.inst_get_origin_x(inst), db.inst_get_origin_y(inst)) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn term(name: &str, pins: Vec<Vec<(usize, Rect)>>) -> MasterTerm {
        MasterTerm { name: name.into(), sig: "SIGNAL".into(), pins: pins.into_iter().map(|shapes| MasterPin { shapes }).collect() }
    }

    /// A cut obstruction touching ONE pin's shapes on the layer above becomes that pin's shape.
    #[test]
    fn a_cut_obstruction_on_one_pin_joins_it() {
        let t = crate::gc::tests::tech();
        let cut = Rect::new(0, 0, 170, 170);
        let m = Master::import(&t, vec![term("A", vec![vec![(4, Rect::new(-50, -50, 400, 220))]])], &[(3, cut)]);
        assert!(m.blockages.is_empty());
        assert!(m.terms[0].pins[0].shapes.contains(&(3, cut)));
    }

    /// Touching two pins it stays an obstruction.
    #[test]
    fn a_cut_obstruction_on_two_pins_stays_a_blockage() {
        let t = crate::gc::tests::tech();
        let cut = Rect::new(0, 0, 170, 170);
        let terms = vec![term("A", vec![vec![(4, Rect::new(-50, -50, 100, 220))]]), term("B", vec![vec![(4, Rect::new(100, -50, 400, 220))]])];
        let m = Master::import(&t, terms, &[(3, cut)]);
        assert_eq!(m.blockages, vec![(3, cut)]);
        assert!(m.terms.iter().all(|t| t.pins[0].shapes.len() == 1));
    }

    /// Obstructions on a layer merge; the blockages are the merge's MAXIMAL rectangles (an L gives
    /// both arms through the corner).
    #[test]
    fn obstructions_merge_into_maximal_rectangles() {
        let t = crate::gc::tests::tech();
        let m = Master::import(&t, Vec::new(), &[(4, Rect::new(0, 0, 300, 100)), (4, Rect::new(0, 0, 100, 300))]);
        let mut b = m.blockages;
        b.sort();
        assert_eq!(b, vec![(4, Rect::new(0, 0, 100, 300)), (4, Rect::new(0, 0, 300, 100))]);
    }
}
