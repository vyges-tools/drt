// SPDX-License-Identifier: Apache-2.0
//! Detailed routing, starting with pin access: for every pin, the points a route may reach it at,
//! chosen by fully specified rules so the result is reproducible bit for bit.
//!
//! - [`polygon90`]: Manhattan polygon sets — union, slicing, maximal rectangles.
//! - [`tech`]: the technology and design as the router reads them.
//! - [`gc`]: design-rule checks over a small window.
//! - [`polygon_formation`]: the polygons (with holes) of a rectangle set, each ring in the vertex
//!   order the design-rule check reads its edges in.
//! - [`pa`]: pin access — unique classes, access points, patterns, rows, the write-back.
//! - [`dr`]: detailed routing after pin access — the route guides first.

// The crate's own name inside it: the command-line entry ([`cli_main`]) is built into the
// library too, and its `vyges_drt::…` paths resolve here.
extern crate self as vyges_drt;

pub mod dr;
pub mod gc;
pub mod pa;
pub mod polygon90;
pub mod polygon_formation;
pub mod rtree;
pub mod tech;

/// The `vyges-drt` command-line entry (the binary's `main`), built into the library too so a
/// suite binary can link every engine and run the one it is invoked as.
#[cfg(feature = "suite")]
#[doc(hidden)]
#[path = "bin/vyges-drt.rs"]
pub mod cli_main;
