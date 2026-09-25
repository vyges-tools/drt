# vyges-drt

Detailed routing, starting with pin access: for every pin of every cell, the points a route may
reach it at, and for every routed instance terminal the point it will be reached at — chosen by
fully specified rules, so the result is reproducible bit for bit.

```sh
vyges-drt pin_access --lef tech.lef --lef cells.lef --def design.def --out design.odb
vyges-drt --describe
```

**Pin access** (`vyges-drt pin_access`), in stages:

1. **Unique classes** — instances that must have the same pin access (same master, orientation
   and track offsets) are grouped, and each class is analysed once, on a representative.
2. **Access points** — per pin, cost rounds of candidates (on-track, half-track, centre,
   enclosed-boundary), each planar direction and each via checked against the design rules
   (shorts, non-sufficient metal, the parallel-run spacing table, cut spacing), until enough
   sparse points are kept.
3. **Access patterns** — per class, a shortest path choosing one point per pin so that
   neighbouring pins' vias stand clear of each other.
4. **Row patterns** — per row of abutting instances, one pattern per instance so that facing
   boundary vias stand clear.
5. **Write-back** — every master pin's points, each routed instance terminal's preferred points
   and each single-pin port's points, into the design database, where global routing reads them.

Status: **in development (v0.1.0)**. See `--describe` for what is modelled and what is refused.
Documentation: [`docs/src/drt.md`](docs/src/drt.md) (an mdBook; `mdbook build docs`), with the
[CLI reference](docs/src/reference/vyges-drt.md).

## Build

The command reads and writes the design database through `vyges-opendb`, which builds its C++
library from source. Check out `vyges-tools/opendb` and `vyges-tools/opendb-lib` beside this
repository (see `.github/workflows/ci.yml`), then:

```sh
cargo build --release --features odb
cargo test  --release --features odb
```

Without `--features odb` the library builds with no C++ dependency: the geometry and every rule
(candidates, design-rule checks, patterns, rows, the write plan) are tested on their own.

Licensed under Apache-2.0; `src/polygon90.rs` also carries the Boost Software License notice.
