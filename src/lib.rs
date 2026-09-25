// SPDX-License-Identifier: Apache-2.0
//! Detailed routing, starting with pin access: for every pin, the points a route may reach it at,
//! chosen by fully specified rules so the result is reproducible bit for bit.
//!
//! - [`polygon90`]: Manhattan polygon sets — union, slicing, maximal rectangles.
//! - [`tech`]: the technology and design as the router reads them.
//! - [`gc`]: design-rule checks over a small window.
//! - [`pa`]: pin access — unique classes, access points, patterns, rows, the write-back.
//! - [`dr`]: detailed routing after pin access — the route guides first.

pub mod dr;
pub mod gc;
pub mod pa;
pub mod polygon90;
pub mod tech;
