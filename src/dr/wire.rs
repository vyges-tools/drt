// SPDX-License-Identifier: Apache-2.0
//! A net's routed wire as the router reads it from the database: the wire decoder's ops, walked
//! path by path into wires, vias and patches. Pure: the database accessor that yields the ops is
//! `dr::db::read_net_routing`.

use crate::polygon90::Rect;
use crate::tech::Tech;

/// A net's routing as the router reads it from the database: its wires, vias and patches, each
/// list in the wire's order.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct InitialRouting {
    pub segs: Vec<crate::dr::cost::DrFig>,
    pub vias: Vec<crate::dr::cost::DrFig>,
    pub patches: Vec<crate::dr::cost::DrFig>,
}

impl InitialRouting {
    /// Only a wire makes a net "routed" (a wire of vias or patches alone does not).
    pub fn has_wire(&self) -> bool {
        !self.segs.is_empty()
    }
}

/// The wire decoder's ops as [`Db::net_wire_decode`] gives them (the first record, the wire type,
/// left out).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WireOp {
    /// Path, junction, short or virtual wire (opcodes 0–3): the layer.
    Path(i32, String),
    Point(i32, i32),
    PointExt(i32, i32, i32),
    /// Block via (6) or tech via (7): opcode, name, bottom layer, top layer.
    Via(i32, String, String, String),
    Rect(i32, i32, i32, i32),
    Other(i32),
}

impl WireOp {
    fn code(&self) -> i32 {
        match self {
            WireOp::Path(c, _) | WireOp::Via(c, ..) | WireOp::Other(c) => *c,
            WireOp::Point(..) => 4,
            WireOp::PointExt(..) => 5,
            WireOp::Rect(..) => 8,
        }
    }
}

pub fn wire_ops(recs: &[String]) -> Result<Vec<WireOp>, String> {
    let mut out = Vec::new();
    for r in recs {
        let f: Vec<&str> = r.split('|').collect();
        let code: i32 = f[0].parse().map_err(|_| format!("wire op {r}"))?;
        let n = |i: usize| -> Result<i32, String> { f.get(i).and_then(|v| v.parse().ok()).ok_or_else(|| format!("wire op {r}")) };
        let st = |i: usize| -> Result<String, String> { f.get(i).map(|v| v.to_string()).ok_or_else(|| format!("wire op {r}")) };
        out.push(match code {
            0..=3 => WireOp::Path(code, st(1)?),
            4 => WireOp::Point(n(1)?, n(2)?),
            5 => WireOp::PointExt(n(1)?, n(2)?, n(3)?),
            6 | 7 => WireOp::Via(code, st(1)?, st(2)?, st(3)?),
            8 => WireOp::Rect(n(1)?, n(2)?, n(3)?, n(4)?),
            c => WireOp::Other(c),
        });
    }
    Ok(out)
}

/// The router's reading of a wire, path by path: a path gathers its layer, up to two points (the
/// second with the extension it carries), a via and a patch, and ends at the next path op, via or
/// the end — or where the next point turns: a point after a complete segment that runs the other
/// way ends the path there, and the next path begins at this one's end point (so no stretch is
/// lost at a bend). A via with no point before it in its path sits where the previous path ended
/// (its "next" point: the end point when there was one, else the begin point); the path's layer is
/// the via's layer that is not the previous path's. Per path, in order: a patch at the begin
/// point; a wire (its points ordered low to high with their extensions swapped along; width the
/// layer's, or its wrong-way width across the layer; an end with no extension EXTENDS by half the
/// layer's width, an extension of 0 TRUNCATES, another is kept as given); a via at the begin
/// point. (The reference places a via at the END point when the path had points before it — but
/// a via op always ends the running path, so a via is always its path's first op: that branch
/// never runs, and is kept here only as written.)
pub fn parse_wire(tech: &Tech, ops: &[WireOp]) -> Result<InitialRouting, String> {
    use crate::dr::cost::DrFig;
    let mut out = InitialRouting::default();
    let layer_of = |name: &str| tech.layer_num(name).ok_or_else(|| format!("layer {name} not read"));
    let (mut end, mut next) = ((-1, -1), (-1, -1));
    let mut layer_name = String::new();
    let mut prev_layer = String::new();
    let mut orthogonal = false;
    let mut i = 0;
    let at = |i: usize| ops.get(i).map_or(12, WireOp::code);
    while at(i) != 12 {
        // After a bend the path begins at the previous one's end, on its layer.
        let mut has_begin = orthogonal;
        let mut begin = if orthogonal { end } else { (-1, -1) };
        if !orthogonal {
            layer_name.clear();
        }
        let mut via: Option<(i32, String)> = None;
        let mut has_end = false;
        let mut begin_in_via = false;
        orthogonal = false;
        let (mut begin_ext, mut end_ext) = (-1, -1);
        end = (-1, -1);
        let mut rect: Option<Rect> = None;
        loop {
            match ops.get(i) {
                Some(WireOp::Path(_, l)) => {
                    layer_of(l)?;
                    prev_layer = l.clone();
                    layer_name = l.clone();
                }
                Some(WireOp::Point(x, y)) => {
                    if !has_begin {
                        begin = (*x, *y);
                        has_begin = true;
                    } else {
                        end = (*x, *y);
                        has_end = true;
                    }
                }
                Some(WireOp::PointExt(x, y, e)) => {
                    if !has_begin {
                        begin = (*x, *y);
                        begin_ext = *e;
                        has_begin = true;
                    } else {
                        end = (*x, *y);
                        end_ext = *e;
                        has_end = true;
                    }
                }
                Some(WireOp::Via(code, name, bottom, top)) => {
                    via = Some((*code, name.clone()));
                    layer_name = if prev_layer == *top { bottom.clone() } else { top.clone() };
                    if !has_begin {
                        begin = next;
                        has_begin = true;
                        begin_in_via = true;
                    }
                }
                Some(WireOp::Rect(l, b, r, t)) => rect = Some(Rect { xl: *l, yl: *b, xh: *r, yh: *t }),
                _ => {}
            }
            i += 1;
            if let (Some(WireOp::Point(x, _)), true) = (ops.get(i), has_end) {
                let curr_vertical = begin.0 == end.0;
                let next_vertical = end.0 == *x;
                orthogonal = curr_vertical != next_vertical;
            }
            let c = at(i);
            if c <= 3 || c == 6 || c == 7 || c == 12 || orthogonal {
                next = if has_end { end } else { begin };
                break;
            }
        }
        let layer = layer_of(&layer_name)?;
        if let Some(r) = rect {
            out.patches.push(DrFig::Patch { layer, origin: begin, offset: r });
        }
        if has_end {
            let (mut b, mut e) = (begin, end);
            if begin.0 > end.0 || begin.1 > end.1 {
                std::mem::swap(&mut b, &mut e);
                std::mem::swap(&mut begin_ext, &mut end_ext);
            }
            let l = &tech.layers[layer];
            let wrong = if l.is_horizontal() { begin.1 != end.1 } else { begin.0 != end.0 };
            let width = if wrong { l.wrong_way_width } else { l.width };
            let half = l.width / 2;
            let ext = |x: i32| if x == -1 { half } else { x };
            out.segs.push(DrFig::Seg { layer, begin: b, end: e, width, begin_ext: ext(begin_ext), end_ext: ext(end_ext), bi: (0, 0, 0), ei: (0, 0, 0), tapered: false, begin_trunc: begin_ext == 0, end_trunc: end_ext == 0 });
        }
        if let Some((code, name)) = via {
            if code == 6 {
                return Err(format!("block via {name} in a wire not modelled"));
            }
            let v = tech.via_defs.iter().position(|d| d.name == name).ok_or_else(|| format!("via {name} not read"))?;
            let origin = if has_end && !begin_in_via { end } else { begin };
            out.vias.push(DrFig::Via { via: v, origin, bi: (0, 0, 0), ei: (0, 0, 0), tapered: false, bottom_connected: false, top_connected: false });
        }
    }
    Ok(out)
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::dr::cost::DrFig;
    use crate::tech::{Dir, Layer, LayerKind, ViaDef};

    /// 2 M1 horizontal (width 100, wrong-way 120), 3 cut, 4 M2 vertical (140 / 160); via V12.
    fn tech() -> Tech {
        let r = |xl, yl, xh, yh| Rect { xl, yl, xh, yh };
        let routing = |name: &str, dir, width, wrong_way_width| Layer { name: name.into(), kind: LayerKind::Routing, dir, width, min_width: width, wrong_way_width, ..Default::default() };
        Tech {
            layers: vec![Layer::default(), Layer::default(), routing("M1", Dir::Horizontal, 100, 120), Layer { name: "V1".into(), kind: LayerKind::Cut, ..Default::default() }, routing("M2", Dir::Vertical, 140, 160)],
            via_defs: vec![ViaDef { name: "V12".into(), is_default: true, layer1: 2, cut: 3, layer2: 4, layer1_figs: vec![r(-50, -50, 50, 50)], cut_figs: vec![r(-20, -20, 20, 20)], layer2_figs: vec![r(-70, -70, 70, 70)] }],
            ..Default::default()
        }
    }

    fn ops(v: &[&str]) -> Vec<WireOp> {
        wire_ops(&v.iter().map(|s| s.to_string()).collect::<Vec<_>>()).expect("ops")
    }

    #[allow(clippy::too_many_arguments)]
    fn seg(layer: usize, begin: (i32, i32), end: (i32, i32), width: i32, be: i32, ee: i32, bt: bool, et: bool) -> DrFig {
        DrFig::Seg { layer, begin, end, width, begin_ext: be, end_ext: ee, bi: (0, 0, 0), ei: (0, 0, 0), tapered: false, begin_trunc: bt, end_trunc: et }
    }

    fn via(origin: (i32, i32)) -> DrFig {
        DrFig::Via { via: 0, origin, bi: (0, 0, 0), ei: (0, 0, 0), tapered: false, bottom_connected: false, top_connected: false }
    }

    // Rule: an end with no extension extends by half the layer's width; an extension of 0
    // truncates.
    #[test]
    fn a_wire_end_without_extension_extends_and_zero_truncates() {
        let r = parse_wire(&tech(), &ops(&["0|M1", "4|0|0", "5|1000|0|0", "12"])).unwrap();
        assert_eq!(r.segs, vec![seg(2, (0, 0), (1000, 0), 100, 50, 0, false, true)]);
    }

    // Rule: a point after a complete wire that turns ends the path there; the next path begins at
    // this one's end on the same layer (here a wrong-way stretch of M1: its wrong-way width).
    #[test]
    fn a_bend_continues_from_the_previous_end() {
        let r = parse_wire(&tech(), &ops(&["0|M1", "4|0|0", "4|1000|0", "4|1000|500", "12"])).unwrap();
        assert_eq!(r.segs, vec![seg(2, (0, 0), (1000, 0), 100, 50, 50, false, false), seg(2, (1000, 0), (1000, 500), 120, 50, 50, false, false)]);
    }

    // Rule: a via with no point before it in its path sits at the previous path's end and gives
    // the path the via's other layer; its wire starts there.
    #[test]
    fn a_via_that_begins_a_path_sits_at_the_previous_end() {
        let r = parse_wire(&tech(), &ops(&["0|M1", "4|0|0", "4|1000|0", "7|V12|M1|M2", "4|1000|800", "12"])).unwrap();
        assert_eq!(r.segs, vec![seg(2, (0, 0), (1000, 0), 100, 50, 50, false, false), seg(4, (1000, 0), (1000, 800), 140, 70, 70, false, false)]);
        assert_eq!(r.vias, vec![via((1000, 0))]);
    }

    // Rule: a via after a wire's points ends that path and begins the next, at the wire's end.
    #[test]
    fn a_via_after_a_wire_begins_the_next_path_at_its_end() {
        let r = parse_wire(&tech(), &ops(&["0|M2", "4|500|0", "4|500|900", "7|V12|M1|M2", "12"])).unwrap();
        assert_eq!(r.vias, vec![via((500, 900))]);
    }

    // Rule: points are ordered low to high with their extensions swapped along (a given
    // extension that is neither absent nor 0 is kept as given).
    #[test]
    fn points_are_ordered_with_their_extensions() {
        let r = parse_wire(&tech(), &ops(&["0|M1", "5|1000|0|30", "4|0|0", "12"])).unwrap();
        assert_eq!(r.segs, vec![seg(2, (0, 0), (1000, 0), 100, 50, 30, false, false)]);
    }

    // Rule: a patch sits at its path's begin point; a path with one point has no wire.
    #[test]
    fn a_patch_sits_at_the_begin_point() {
        let r = parse_wire(&tech(), &ops(&["0|M1", "4|500|0", "8|-50|-60|50|60", "12"])).unwrap();
        assert!(r.segs.is_empty());
        assert_eq!(r.patches, vec![DrFig::Patch { layer: 2, origin: (500, 0), offset: Rect { xl: -50, yl: -60, xh: 50, yh: 60 } }]);
    }

    // A block via (the design's own) is refused: the reader would add it to the technology.
    #[test]
    fn a_block_via_is_refused() {
        assert!(parse_wire(&tech(), &ops(&["0|M1", "4|0|0", "4|1000|0", "6|B12|M1|M2", "12"])).is_err());
    }
}
