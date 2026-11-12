# vyges-drt — detailed routing

`vyges-drt` has two commands. `detailed_route` routes a placed design on its route guides and
writes the wires. `pin_access` is its first stage on its own: the access points global routing
reads. Pin access is described first because routing starts there.

## Pin access

Before a net can be routed in detail, each of its pins needs a point a wire can reach it at, and a
direction or via to leave by. `vyges-drt pin_access` computes them for every pin of every cell and
writes them into the design database; global routing reads each terminal's preferred point from
there.

### What it does

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
different nets (metal and cut), non-sufficient metal, the layer's spacing table (parallel-run or
two-widths), cut spacing, and end-of-line spacing (a line end narrower than the rule's width needs its space to a facing
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

### Run it

```sh
vyges-drt pin_access --lef tech.lef --lef cells.lef --def design.def \
    --max-routing-layer met5 --out design.odb
```

The report is JSON on stdout (`-o FILE` to write it elsewhere):

```json
{"tool":"drt","status":"written","out":"design.odb","unique_classes":82,
 "instances_in_rows":375,"preferred_points":1210,"port_points":54,"skipped_rules":[],
 "lef58_not_checked":[]}
```

`status` is `written`, `vacuous` (nothing to access — not a pass), `refused` or `error`; see the
[CLI reference](./reference/vyges-drt.md).

### Limits

Refused: a nearby-track access point that keeps an access (a pin no other round reaches, whose
nearby point would need its path to the pin added to the route), and every rule family or layer
property detailed routing refuses, since the access points are judged by the same design-rule
check. The nearby-track rounds themselves run; a point they make that has no access is dropped, as
the reference drops it.
Rect-only layers are modelled: unidirectional in access, track assignment and routing, and checked
with the rect-only and minimum-width rules; so are right-way-on-grid-only layers, which take on-track
access points only. A multi-patterned routing layer (LEF `MASK` above 1) is routed both ways at
once: unidirectional and right-way on grid; the router assigns no colour. The
router settings are its defaults (via-access layer 2, three sparse points per pin, non-preferred
tracks allowed); the top routing layer is the block's maximum routing layer.

## Detailed routing

`detailed_route` reads a placed database that carries route guides (what global routing writes)
and runs the router's stages in order: pin access, guide processing, track assignment, then
search-and-repair iterations. Each iteration routes the design in worker tiles, checks the result
against the design rules, and rips up and reroutes around the markers it finds, until none stand.
The routes are written as a DEF or a database.

```sh
vyges-drt detailed_route --db placed_with_guides.odb --out routed.def
```

The JSON report carries `status`, `out`, `iterations` (search-and-repair iterations run),
`markers` (design-rule markers standing at the end), `nets_written`, `routed_before`,
`rerouted`, `skipped_rules`, `lef58_not_checked` and `area_patches_dropped_no_owner` (below).

`status` is `written` (routed with no marker standing, exit 0), `markers` (routed and written, but
markers remain: exit 1, a finding rather than a pass), `vacuous`, `refused` (exit 3) or `error`
(exit 2). `vacuous` (exit 2, never a pass) is a design where no net has two or more terminals (two
instance terminals, two ports, or one of each, special nets included): the reference stops there,
after reading the design and before pin access, with warning DRT-0040, and so does this router. It
logs the same warning, routes nothing and writes the database back unchanged; the report carries
`reason`, `out`, `skipped_rules` and `lef58_not_checked`.

The routing layers are the block's minimum and maximum routing layers as the database holds them,
and the non-default rules are the database's. **Incremental routing:** nets that already have wires
in the database are kept. Their wires are read, the first three iterations reroute only the other
nets, and the report counts both (`routed_before`, `rerouted`). A DEF alone cannot show which mode
ran, because rerouting everything writes the same DEF; the counts can.

### Limits

Design rules modelled: shorts, non-sufficient metal, parallel-run spacing (with spacing limited to
a width range), the two-widths spacing table (`SPACINGTABLE TWOWIDTHS`, looked up on both widths;
kept over a parallel-run table on the same layer), cut spacing (one plain rule per cut layer), LEF 5.4 end-of-line spacing, minimum
width, minimum area (patched, or reported where a patch cannot fix it), minimum enclosed area
(`MINENCLOSEDAREA` without a width), minimum step, minimum cut (`MINIMUMCUT`, and
`LEF58_MINIMUMCUT` converted into the same rule as the reference converts it; one written per
cut class, `MINIMUMCUT CUTCLASS c n`, takes the count of the class of the default via below,
else above, as the reference fixed for upstream OpenROAD issue #12176 does, where the unpatched
reference drops it unread, and a run whose default via there has no cut class is refused), rect-only and
right-way-on-grid-only layers, and the LEF58 families end-of-line keep-out, end-of-line spacing
(its common clauses, with `MINLENGTH` / `MAXLENGTH`, `ENDTOEND … EXTENSION` (twice the extension
replaces `WITHIN` beside the line end, and a facing line end is measured with both ends widened by
the extension, as the reference's check does; the router's costs do not read it, nor do the
reference's), `OPPOSITEWIDTH` (the rule applies only when the wire beyond the line end spans
less than the opposite width, as the LEF defines it, in the check only; the reference stores the
clause and never reads it, upstream OpenROAD issue #12226) and `SAMEMASK`, the latter held on every pair whatever its masks, as the reference
holds it), corner spacing (convex corners, patched by the router whenever the technology has corner spacing; the reference patches them only when the technology also has a `MINSTEP` rule, and this engine follows the reference's own fix, upstream OpenROAD issue #12207), cut classes, the
different-net and `SAMENET` cut spacing tables of a cut layer, its default inter-layer table
(`SPACINGTABLE … LAYER l`: cuts of two nets on the two layers, checked from either layer, and
costed by the router around the other layer's vias, except where the other layer's vias fall
outside the routing grid, which the reference indexes past its grid, OpenROAD issue #12184; a
`SAMEMETAL` table, and a `SAMENET` or
`SAMEMETAL` inter-layer one, are refused), enclosure, and cut spacing to a concave corner or to a
non-end-of-line convex corner of the metal on another layer (`SPACING s LAYER l CUTCLASS c
CONCAVECORNER`, its plain form, and `SPACING s LAYER l CUTCLASS c NONEOLCONVEXCORNER w [MINLENGTH
m]`), and LEF58 cut spacing with adjacent cuts (`SPACING s [CENTERTOCENTER] ADJACENTCUTS n
[TWOCUTS t] WITHIN w [CUTCLASS c] [NOPRL]`; with `EXACTALIGNED`, `EXCEPTSAMEPGNET`, `TO ALL`,
`SIDEPARALLELOVERLAP` or `SAMEMASK` the rule is skipped with a warning, as the reference skips
it; with two `WITHIN` values (`WITHIN w1 w2`) it is skipped with the warning the reference's own
fix prints, DRT-359, where the unpatched reference keeps only `w1`, upstream OpenROAD issue
#12202; every other LEF58 cut spacing form is refused), and
`LEF58_AREA` on a routing layer (`AREA a` with `RECTWIDTH w` or `EXCEPTRECTANGLE`: checked, and
patched on the routed net as the reference patches it; a rule with `EXCEPTMINWIDTH`,
`EXCEPTEDGELENGTH`, `EXCEPTMINSIZE`, `EXCEPTSTEP`, `MASK` or `LAYER` is skipped with a warning, as
the reference skips it), `LEF58_MINSTEP` on a routing layer (`MINSTEP l … NOBETWEENEOL w`: a
line end narrower than `w` between two edges shorter than `l`; `MINSTEP l … MINADJACENTLENGTH m`,
with `EXCEPTRECTANGLE`: an edge shorter than `l` beside one shorter than `m`; checked per rule, a marker named `Min Step` as the
reference names it, and patched by the via-swap workers' minimum-step patch as the reference
patches it; a rule with `MAXEDGES` above 1 is skipped with the reference's warning, DRT-335; a
rule with `NOADJACENTEOL`, `EXCEPTRECTANGLE` with `NOBETWEENEOL`, `MINBETWEENLENGTH`, a second
`MINADJACENTLENGTH` value or its corner forms, or neither checked form, is refused, since the
reference's check reads none of them as the LEF defines them; upstream OpenROAD issue #12229
asks about the first two, which the reference's own fix checks),
`LEF58_WIDTHTABLE … ORTHOGONAL` on a routing layer (the first width of the last orthogonal table,
and of the last one with `WRONGDIRECTION` for the other axis, the width when there is none: two
concave corners of one polygon closer than both, across and along, are a marker named
`WidthTableOrth` as the reference names it; checked only, as the reference only checks it; a table
without `ORTHOGONAL` is not read, as the reference does not read it),
`LEF58_SPACING … WRONGDIRECTION` on a routing layer (`SPACING s WRONGDIRECTION [NONEOL w] [PRL p]`:
two wrong-direction edges, vertical on a horizontal layer and horizontal on a vertical one, of any
shapes, neither shorter than `w` when `NONEOL` is given, running parallel longer than `p` (0
without `PRL`), and closer than `s`, are a marker named `Lef58SpacingWrongDir` as the reference
names it, once per rule; checked only, as the reference only checks it; a rule with `LENGTH` or a
negative `PRL` is skipped with the reference's warning, DRT-333 or DRT-334), and the
orthogonal spacing table of a cut layer (`SPACINGTABLE ORTHOGONAL
WITHIN w SPACING s …`: per row, a cut within `w` of a pair of cuts that run parallel on one axis,
and closer than `s` across that run, is marked; the marker names the pair as its victims, each
box its cut, which the router reads when it picks the nets to rip up; the unpatched reference
takes each box's top from the cut's right edge, upstream OpenROAD issue #12198, and this router
follows the fixed reading, by decision). LEF58 max spacing (`LEF58_MAXSPACING max CUTCLASS c`) runs the router's repair pass as the reference
runs it, after an iteration that ends with no marker: its secondary via defs and its search for
vias of the class with no other cut within `max`; a rule with no `CUTCLASS` is skipped with the
reference's warning (DRT-350). Then, per such via, its marker, the three-by-three-gcell region
around it and the pin terminals it lands on, and the regions' merge, as the reference does them.
The pin-access re-run for those terminals follows: the instances they belong to are reclassified
with the terminals in their class key, their access points regenerated without the rule's vias,
and their rows' patterns chosen again. Then the via-swap workers, one per merged region: each
walks every lonely via in its region, gives it the layer's next secondary via def, checks the net,
and keeps the swap only when the check is clean; a worker that ends with more markers than it began
with is not written back. Three points differ from the reference on purpose, each the reference's own
upstream fix: a stubborn terminal that the avoided via defs leave with no access point has its
access made again with every via def, with warning DRT-355 (the reference stops with DRT-0073,
upstream OpenROAD issue #12199); when a via has tried every secondary def it goes back to the def it had and the
worker's check is told (the reference takes the layer's default via and checks the last one it
tried, which leaves markers for a via that is in neither the route nor the design: upstream OpenROAD
issue #12203); and a worker
that holds none of a re-run terminal's new access points leaves that terminal out with warning
DRT-356 (the reference stops with DRT-1231, upstream OpenROAD issue #12200). A swapped via
whose pad leaves a minimum-step violation on a `MINSTEP` layer beside the cut layer has its pad end
patched out to the `MINSTEP` length in the worker's check, as the reference patches it. Where a re-run terminal has no access point even with every via def, the
reference itself aborts (DRT-0073); the run is refused at the same point with exit 4, `reason` and
the report's `reference_aborts` name the error and the issue, and nothing is written. A re-run terminal whose nearby-track access rounds keep a point, which this engine does not
model, is refused with exit 3; when they keep none, the terminal falls back to every via def as
above. A cell obstruction with its own `DESIGNRULEWIDTH`
or `SPACING` is checked on its own, by that rule. A net the global router gave antenna jumpers is
kept to its guides: off them it costs ten times as much.
Refused rather than approximated: any other rule family or clause a layer carries (named in the
report's `reason`), among them LEF58 minimum step, `SPACINGTABLE INFLUENCE` and a
metal-width via map; corner spacing in a design with a block master; a layer whose via pads break
its minimum step; a non-default rule with hard spacing, via
generate rules or wire extension; FIXED wiring, or a routed net on a non-default rule, already in
the database; and congested input guides. Past iteration 7 the router widens the clip of
congested workers, and from iteration 23 rips up the nets near each marker (NEARDRC); both follow
the reference, though no correlated design exercises them yet.

Skipped with a warning, neither checked nor refused: a LEF58 end-of-line spacing rule with
`TOCONCAVECORNER` or `TONOTCHLENGTH`. The router does not model these forms, so it warns at tech
load and lists each rule in the report's `skipped_rules` (layer, rule index, clause, `checked`
false), in both commands, and the log carries the warning the reference's own fix for these
forms prints (DRT-357 for `TOCONCAVECORNER`, DRT-358 for `TONOTCHLENGTH`); the status does not
change. Sign-off DRC must cover these rules. The unpatched reference router reads both forms as a
plain end-of-line rule with `WITHIN 0` (upstream OpenROAD issue #12174); this router follows the
fixed reading instead, by decision. Skipped the same way: a `LEF58_MINIMUMCUT`
rule the reference routes without, one with `AREA`, `SAMEMETALOVERLAP` or `FULLYENCLOSED` (the
reference warns and skips it), and a LEF58 wrong-direction spacing rule with `LENGTH` or a negative
`PRL` (DRT-333, DRT-334, as the reference warns and skips it).

Every rule the reference itself skips at tech load is skipped here too, and the log carries the
reference's own coded warning for it, line for line and in the reference's order (layer by layer;
on a routing layer the end-of-line rules, then wrong-direction spacing, minimum step, `LEF58_AREA`
and `LEF58_MINIMUMCUT`): DRT-400, DRT-401 and DRT-403 for an end-of-line rule with
`EXCEPTEXACTWIDTH`, `FILLCONCAVECORNER` or `EQUALRECTWIDTH`; DRT-311 to DRT-316 for a `LEF58_AREA`
rule with `EXCEPTMINWIDTH`, `EXCEPTEDGELENGTH`, `EXCEPTMINSIZE`, `EXCEPTSTEP`, `MASK` or `LAYER`;
DRT-317 to DRT-319 for a `LEF58_MINIMUMCUT` rule with `AREA`, `SAMEMETALOVERLAP` or
`FULLYENCLOSED`; DRT-45, DRT-46, DRT-48, DRT-51 and DRT-52 for an `ADJACENTCUTS` rule with
`EXACTALIGNED`, `EXCEPTSAMEPGNET`, `TO ALL`, `SIDEPARALLELOVERLAP` or `SAMEMASK`. A rule with two of
these clauses gets one line, for the first the reference tests.

Skipped the same way: a LEF58 cut spacing `LAYER` rule with no cut class
(`SPACING s [CENTERTOCENTER] [SAMENET] LAYER l`), or with a cut class but neither `CONCAVECORNER`
nor `NONEOLCONVEXCORNER` (`SPACING s … LAYER l CUTCLASS c`), with the warning the reference's own
fix prints (DRT-353 with no class, DRT-354 with neither corner). The unpatched reference keeps
it, but its check acts only on a cut class with one of those corners, so it never checks it,
without a warning, and a `SAMENET` one makes every other LEF58 cut spacing rule of the layer
judge only cuts of one net (upstream OpenROAD issues #12187 and #12188); this router follows the
fixed reading, by decision, so a rule beside such a `SAMENET` rule judges every net.

### Where this router differs from the reference, by decision

Everywhere else the router is a reading of the reference and matches it byte for byte. Where the
reference misreads a rule, this router follows the rule as written instead. Each such divergence
is correlated against the reference patched with the fix (a named build), and on every case it does
not reach the routes still match the unpatched reference.

- **End-of-line `MINLENGTH` / `MAXLENGTH` `TWOSIDES`.** The reference takes TWOSIDES from
  PARALLELEDGE's `TWOEDGES`, so `MINLENGTH l TWOSIDES` is judged on one side unless the rule also
  says `TWOEDGES` (upstream OpenROAD issue #12181). This router reads TWOSIDES as written.
- **End-of-line `SAMEMASK` `WITHIN`.** The reference reads a `SAMEMASK` end-of-line rule's `WITHIN`
  as 0, so a line end just past a neighbour's corner is not held to the rule (upstream OpenROAD
  issue #12182). This router reads `WITHIN` as written. Like the reference, it holds the rule on
  every pair whatever its masks; the colour step checks it per mask.
- **End-of-line `TOCONCAVECORNER` / `TONOTCHLENGTH`.** The reference reads either form as a plain
  end-of-line rule with `WITHIN` 0 (upstream OpenROAD issue #12174). This router skips it with the
  fixed reference's warning (DRT-357, DRT-358).
- **`ADJACENTCUTS` with two `WITHIN` values.** The reference keeps only the first value (upstream
  OpenROAD issue #12202). This router skips the rule with the fixed reference's warning (DRT-359).
- **A cut spacing `LAYER` rule it never checks.** With no `CUTCLASS`, or a class but neither
  corner, the reference keeps the rule, never checks it, and a `SAMENET` one makes every other
  LEF58 cut spacing rule of the layer judge one net only (upstream OpenROAD issues #12187 and
  #12188). This router skips it with the fixed reference's warning (DRT-353, DRT-354).
- **Orthogonal spacing table victims.** The reference builds each victim box with its top taken
  from the cut's right edge (upstream OpenROAD issue #12198). This router's victims are the cuts.
- **Area patch owner.** See below (upstream OpenROAD issue #12191).
- **Per-cut-class `MINIMUMCUT`.** The reference's database keeps such a rule with a cut count of 0
  and drops it (upstream OpenROAD issue #12176). This router converts it as the fixed reference
  does, taking the count of the first default via, below then above, whose cut class is named; a
  default via with no cut class is refused.

Area patches, owned where they are: when a net has several pieces in one worker, the unpatched
reference looks up the piece a minimum-area or `LEF58_AREA` patch belongs to by comparing the
patch box relative to its origin, not where the patch is, so it usually finds none and drops the
patch silently (upstream OpenROAD issue #12191). This router follows the fixed reading, by
decision: the patch goes to the first piece that touches it where it is. A patch still without a
piece is dropped and counted: the report's `area_patches_dropped_no_owner`, with one
`DRT-PATCH-DROPPED` warning per run when it is not zero, since a minimum-area marker such a patch
would have fixed may remain and sign-off DRC must then cover minimum area.

Not checked, informational: a LEF58 layer property the database's LEF reader leaves unparsed
(it keeps the raw text only), such as `ENCLOSUREEDGE`, `JOINTCORNERSPACING`, `VIACLUSTER`,
`PRLTWOSIDESSPACING` or `EOLENCLOSURE`. No rule reaches routing, in this router as in the
reference; the report's `lef58_not_checked` lists them per layer and property kind, and the run
is never refused for them.
