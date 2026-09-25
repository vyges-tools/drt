# vyges-drt

Detailed routing: the design's nets routed on their route guides into wires and vias that meet
the design rules — every step chosen by fully specified rules, so the result is reproducible bit
for bit. Pin access is also a command of its own.

```sh
vyges-drt detailed_route --db guided.odb --out routed.def
vyges-drt pin_access --lef tech.lef --lef cells.lef --def design.def --out design.odb
vyges-drt --describe
```

**Detailed routing** (`vyges-drt detailed_route`), in stages: pin access (below); guide
processing (each net's guides split per gcell, covered to its pins, and checked to connect them);
the rule tables; track assignment (each guide's wire on a track, with its costs); then
search-and-repair iterations — the design cut into worker boxes, each worker's nets routed by a
maze search on its grid, its design-rule markers checked and rerouted, the result kept only when
not worse — until no marker stands; and the write-out (wires, vias, patches, via stacks up to
ports above the top routing layer, the gcell grid).

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
