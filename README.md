# vyges-drt

Detailed routing, starting with pin access: for every pin of every cell, the points a route may
reach it at — chosen by fully specified rules, so the result is reproducible bit for bit.

**Stability: in development (v0.1.0).** Done: access-point candidates (per cost class: on-track,
half-track, centre, enclosed-boundary, nearby-track) from each pin's merged shapes and their
maximal rectangles; and the design-rule verdict of each access trial (a planar segment, or a via
and its segment) against the pin's own cell: shorts, non-sufficient metal, the parallel-run spacing
table and cut spacing. Next: the per-instance access patterns.

## Build

```sh
cargo test --release                    # geometry and rules, no C++
cargo build --release --features odb    # the design-database reader (vyges-opendb, C++)
```

Licensed under Apache-2.0; `src/polygon90.rs` also carries the Boost Software License notice.
