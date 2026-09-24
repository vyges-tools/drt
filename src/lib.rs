// SPDX-License-Identifier: Apache-2.0
//! Detailed routing, starting with pin access: for every pin, the points a route may reach it at,
//! chosen by fully specified rules so the result is reproducible bit for bit.
//!
//! - [`polygon90`]: Manhattan polygon sets — union, slicing, maximal rectangles.
//! - [`tech`]: the technology and design as the router reads them.
//! - [`pa`]: pin access — candidates first.

pub mod pa;
pub mod polygon90;
pub mod tech;
