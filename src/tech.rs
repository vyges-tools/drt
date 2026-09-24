// SPDX-License-Identifier: Apache-2.0
//! The technology and design as the router reads them.
//!
//! Rules:
//! - layers are numbered from the first routing layer, with a placeholder masterslice (0) and a
//!   placeholder cut layer (1) below it; routing and cut layers then follow in technology order;
//! - a routing layer's min width is `min(min width, width)`;
//! - a via definition is a technology via: its shapes on the layer below, the cut and the layer
//!   above, and whether it is a default via.

use crate::polygon90::Rect;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LayerKind {
    Routing,
    Cut,
    /// The placeholders below the first routing layer.
    Placeholder,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dir {
    Horizontal,
    Vertical,
    None,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Layer {
    pub name: String,
    pub kind: LayerKind,
    pub dir: Dir,
    pub width: i32,
    pub min_width: i32,
    pub pitch: i32,
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
    pub pins: Vec<MasterPin>,
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
        let placeholder = |name: &str| Layer { name: name.into(), kind: LayerKind::Placeholder, dir: Dir::None, width: 0, min_width: 0, pitch: 0 };
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
                    layers.push(Layer { name, kind: LayerKind::Routing, dir, width, min_width, pitch });
                }
                "CUT" if !layers.is_empty() => {
                    let width = db.layer_get_width(&name) as i32;
                    layers.push(Layer { name, kind: LayerKind::Cut, dir: Dir::None, width, min_width: 0, pitch: 0 });
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
        for (term, _sig) in db.master_mterms(master)? {
            let mut pins = Vec::new();
            for p in 0..db.num_mpins(master, &term) {
                let shapes = db
                    .mpin_boxes(master, &term, p)?
                    .into_iter()
                    .filter_map(|(n, x0, y0, x1, y1)| tech.layer_num(&db.layer_name_by_number(n)).map(|l| (l, Rect::new(x0, y0, x1, y1))))
                    .collect();
                pins.push(MasterPin { shapes });
            }
            out.push(MasterTerm { name: term, pins });
        }
        Ok(out)
    }

    pub fn transform(db: &Db, inst: &str) -> Transform {
        Transform { orient: db.inst_get_orient(inst), origin: (db.inst_get_origin_x(inst), db.inst_get_origin_y(inst)) }
    }
}
