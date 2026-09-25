# vyges-drt — pin access

Before a net can be routed in detail, each of its pins needs a point a wire can reach it at, and a
direction or via to leave by. `vyges-drt pin_access` computes them for every pin of every cell and
writes them into the design database; global routing reads each terminal's preferred point from
there.

## What it does

**Unique classes.** Instances of the same master, in the same orientation and at the same offset
from each preferred-direction track pattern (within the layers the master's signal pins reach),
must have the same pin access. They form a class; its first instance in database order is the
representative the analysis runs on. A class routes a terminal when any of its instances has it on
a regular (not special) net.

**Access points.** For each routed pin of each representative, and each routed port, candidates
are generated in cost rounds — on-track, half-track, centre, enclosed-boundary — from the maximal
rectangles of the pin's merged shapes. Each candidate is given its allowed accesses (a standard
cell's pin at or below the via-access layer gets no planar access), then:

- each planar direction is tried with a segment three layer-widths long; an end still inside the
  pin drops the direction, otherwise the design-rule verdict decides;
- the cut layer's single-cut vias are tried in priority order (at most two kept), each with a
  segment leaving it on its other layer; a via whose shape leaves the cell is skipped, and so is
  one that sticks out of the pin where the via must stay in it;
- if no candidate got an access, every via is tried again.

The rounds stop once enough sparse points are kept (three per cell pin; one for a port). Each
point's vias are ordered by how far they stick out of the pin.

**Design rules** checked in these trials, against the pin's own cell (or port): shorts between
different nets (metal and cut), non-sufficient metal, the parallel-run spacing table, cut spacing,
and end-of-line spacing (a line end narrower than the rule's width needs its space to a facing
edge; long sides of a wire are skipped in via trials, not in planar ones).

**Access patterns.** Per class, the pins are sorted by the mean x of their points (relative to the
instance), and a shortest path picks one point per pin: an edge between neighbouring pins costs the
two points' costs, or a violation cost when their vias (with the previous pin's, on first
evaluation) break a rule. Up to ten rounds each mark the path found; a path whose vias break a
rule as a whole marks its offending points. Every pattern that passes is kept.

**Row patterns.** The routed standard cells, in placement order, form rows of abutting instances;
per row a shortest path picks one pattern per instance, checking each neighbour pair's facing
boundary vias. A lone instance takes its class's first pattern.

**Write-back.** Every master pin's points (per class), every instance's class index, each routed
instance terminal's preferred point per pin, and each single-pin port's points.

## Run it

```sh
vyges-drt pin_access --lef tech.lef --lef cells.lef --def design.def \
    --max-routing-layer met5 --out design.odb
```

The report is JSON on stdout (`-o FILE` to write it elsewhere):

```json
{"tool":"drt","status":"written","out":"design.odb","unique_classes":82,
 "instances_in_rows":375,"preferred_points":1210,"port_points":54}
```

`status` is `written`, `vacuous` (nothing to access — not a pass), `refused` or `error`; see the
[CLI reference](./reference/vyges-drt.md).

## Limits

Refused: a nearby-track round (a pin no other round reaches); a multi-patterned routing layer.
Rect-only layers are modelled: unidirectional in access, track assignment and routing, and checked
with the rect-only and minimum-width rules. Not modelled, so a technology that has them is checked
without them: LEF58 end-of-line forms, a metal-width via map, right-way-on-grid-only layers. The
router settings are its defaults (via-access layer 2, three sparse points per pin, non-preferred
tracks allowed); the top routing layer is the block's maximum routing layer.
