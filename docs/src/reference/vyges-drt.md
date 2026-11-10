# vyges-drt — CLI reference

```text
vyges-drt — detailed routing

USAGE:
  vyges-drt pin_access (--db IN.odb | --lef A.lef [--lef B.lef …] --def D.def)
                       [--max-routing-layer LAYER] --out OUT.odb [-o REPORT.json]
  vyges-drt detailed_route --db IN.odb [--via-access-layer LAYER] [--end-iteration N]
                       --out OUT.def|OUT.odb [-o REPORT.json]
  vyges-drt --describe
  vyges-drt --help
  vyges-drt --version

pin_access groups the instances into unique classes, finds every pin's access points (cost rounds
of candidates, each planar direction and via checked against the design rules), chooses one
access pattern per class and one per instance along each row of abutting cells, and writes the
results into the database: every master pin's access points, each routed instance terminal's
preferred points, each single-pin port's points. The report (JSON) goes to stdout.

detailed_route routes the design on its route guides: pin access, guide processing, track
assignment, then search-and-repair iterations until no design-rule marker stands, and writes the
routes (a DEF or a database). The routing layers are the block's minimum and maximum routing
layers as the database holds them; the non-default rules are the database's. Nets already routed
in the database are kept: their wires are read, the first three iterations reroute only the other
nets, and the report counts both (routed_before, rerouted). Refused: FIXED wiring, a wire using
one of the design's own vias, a routed net on a non-default rule.

Skipped with a warning, neither checked nor refused (both commands): a LEF58 end-of-line spacing
rule with TOCONCAVECORNER or TONOTCHLENGTH, and a LEF58 minimum-cut rule the reference routes
without (AREA, SAMEMETALOVERLAP, FULLYENCLOSED). Each is warned about at
tech load and listed in the report's skipped_rules; the status does not change, so sign-off DRC
must cover these rules.
The report's lef58_not_checked lists, per layer and property kind, the LEF58 layer properties
the database reader left unparsed (no rule reaches routing): informational, never refused.

OPTIONS:
  --db IN.odb                 read the design from a database
  --lef FILE / --def FILE     or from LEF files and a DEF
  --max-routing-layer LAYER   set the block's maximum routing layer first (as a
                              signal-layer range ending at LAYER does)
  --via-access-layer LAYER    detailed_route: the via-access layer (default the second
                              routing layer)
  --end-iteration N           detailed_route: stop after search-and-repair iteration N
                              (counted from 0), whatever markers stand
  --out OUT.odb               write the database here
  -o FILE                     write the JSON report to FILE instead of stdout
  --threads N                 worker threads (also VYGES_THREADS); default a balanced count
                              from the machine's physical cores; 1 is serial. The routes are
                              the same at any count
  --json                      accepted; the report is JSON either way
  --describe                  print a machine-readable JSON description of the command

EXIT STATUS:
  0  written   access points computed (or the design routed clean) and the output written
  1  markers   detailed_route: routing ended with design-rule markers standing; written
  2  vacuous   nothing to access: no routed instance terminal and no routed port. NOT a pass;
               nothing is written
  2  error     usage, unreadable input
  3  refused   a step this engine does not model (a rule family named in `reason`), a
               terminal with no access point, or a row with no pattern combination — see
               `reason`
  4  refused   detailed_route: the reference itself aborts on this input (an upstream defect);
               refused at the same point, nothing written. `reason` and `reference_aborts`
               name the reference's error and the upstream issue
```

## `--describe`

```json
{
  "schema": "vyges-tool-descriptor/1.1",
  "openroad_pin": "4155ac31014923e0fa721b691d222511adc7d8c3",
  "name": "drt",
  "summary": "detailed routing on the design's route guides: pin access, track assignment and search-and-repair until no design-rule marker stands, the routes written as a DEF or a database; pin access alone is its own command",
  "maturity": "structured",
  "provenance_limitations": [
    "detailed_route status is one of written, markers, vacuous, refused or error: written is routed with no design-rule marker standing (exit 0); markers is routed and written with markers standing (exit 1, a finding, not a pass); vacuous is a design with no net of two or more terminals, where the reference stops before pin access with DRT-0040: nothing is routed and the database is written back unchanged (exit 2, never a pass); error is exit 2; refused is exit 3, or exit 4 where the reference itself aborts on the input (an upstream defect: the report's reference_aborts gives its error code and issue).",
    "pin_access status is one of written, vacuous, refused or error. VACUOUS IS NOT WRITTEN: the design has no routed instance terminal and no routed port, and nothing is written. Exit status is 0 for written, 2 for vacuous and for error, 3 for refused.",
    "detailed_route is correlated end to end on the DEF it writes, compared WHOLE and byte for byte against a fresh reference run: 16 of 16 cases, 2026-09-25 -- 14 of the reference router's own regression scripts (the largest 16,880 nets over 5 search-and-repair iterations), its incremental-routing script, and one constructed incremental case. Each stage (guides, rule tables, track assignment, every worker of every iteration) is also correlated against an instrumented reference. The correlation harness is not part of this repository.",
    "An incremental run's DEF cannot tell incremental rip-up from rerouting everything, which writes the same DEF: the report's routed_before and rerouted counts say which ran.",
    "pin_access is correlated stage by stage against an instrumented reference router, and end to end on the database it writes (preferred points, point counts, port points, pin-access indices) on sky130hs designs, and stage by stage on Nangate45 and GF180 designs, 2026-09-25.",
    "Design rules modelled: shorts (metal and cut), non-sufficient metal, the parallel-run spacing table, the two-widths spacing table (SPACINGTABLE TWOWIDTHS, looked up on both widths; kept over a parallel-run table on the same layer, as the reference keeps it), LEF 5.4 spacing limited to a width range (SPACING ... RANGE), cut spacing (one plain rule per cut layer), LEF 5.4 end-of-line spacing (with or without a parallel edge), minimum width, minimum area (patched where the router can, reported where it cannot), minimum enclosed area (MINENCLOSEDAREA without a WIDTH; a rule with one is skipped, as the reference skips it), minimum step (MINSTEP), minimum cut (MINIMUMCUT, and LEF58_MINIMUMCUT as the reference converts it into the same rule; LENGTH ... WITHIN also widens the worker margin; a rule written per cut class, MINIMUMCUT CUTCLASS c n, takes the count of the class of the default via below, else above, as the reference fixed for upstream OpenROAD issue #12176 does, where the unpatched reference drops it unread; refused where that default via has no cut class), rect-only layers (which are also unidirectional), right-way-on-grid-only layers, multi-patterned routing layers (LEF MASK above 1: unidirectional and right-way on grid, as the reference routes them; no colour is assigned), LEF58 end-of-line keep-out, LEF58 end-of-line spacing with SPACING, ENDOFLINE, WITHIN, ENDTOEND (with EXTENSION: twice the extension replaces WITHIN beside the line end, and a facing line end is measured with both ends widened by it, in the check only, as the reference), OPPOSITEWIDTH (the rule applies only when the wire beyond the line end spans less than the opposite width, as the LEF defines it, in the check only; the reference stores the clause and never reads it, upstream OpenROAD issue #12226), PARALLELEDGE, MINLENGTH or MAXLENGTH (with TWOSIDES) and SAMEMASK (checked on every pair whatever its masks, as the reference checks it) (a rule with EXCEPTEXACTWIDTH, FILLCONCAVECORNER or EQUALRECTWIDTH is skipped, as the reference skips it), LEF58 corner spacing (CONVEXCORNER, with CORNERONLY or CORNERTOCORNER; the router patches corners that break it whenever the technology has corner spacing, where the reference patches them only when the technology also has a MINSTEP rule: its own fix, upstream OpenROAD issue #12207), LEF58 cut classes, the LEF58 different-net and SAMENET cut spacing tables of a cut layer (the SAMENET table judges cuts of one net, which the different-net table then leaves to it, and is read first by the via-to-via prep), the LEF58 default inter-layer cut spacing table of a cut layer (SPACINGTABLE ... LAYER l, neither SAMENET nor SAMEMETAL: checked between cuts of two nets on the two layers, from either layer, with NOSTACK; the router's cost around a cut reads its default for the other layer's vias, except vias outside the routing grid, which the reference indexes past its grid: OpenROAD issue #12184), LEF58 cut spacing to a concave corner or to a non-end-of-line convex corner of the metal on another layer (SPACING s LAYER l CUTCLASS c CONCAVECORNER, its plain form, and SPACING s LAYER l CUTCLASS c NONEOLCONVEXCORNER w [MINLENGTH m]), LEF58 cut spacing with adjacent cuts (SPACING s [CENTERTOCENTER] ADJACENTCUTS n [TWOCUTS t] WITHIN w [CUTCLASS c] [NOPRL]; a rule with EXACTALIGNED, EXCEPTSAMEPGNET, TO ALL, SIDEPARALLELOVERLAP or SAMEMASK is skipped with a warning, as the reference skips it; one with two WITHIN values is skipped with DRT-359, as the reference fixed for upstream OpenROAD issue #12202 skips it, where the unpatched reference keeps only the first value), a LEF58 cut spacing LAYER rule with no CUTCLASS, or with a CUTCLASS but neither CONCAVECORNER nor NONEOLCONVEXCORNER (skipped with DRT-353 or DRT-354, as the reference fixed for OpenROAD issue #12187 skips it: the unpatched reference keeps it but its check acts only on a cut class with one of those corners, so it never checks it, and a SAMENET one makes the reference judge only cuts of one net in every other LEF58 cut spacing rule of the layer; sign-off DRC must cover it), LEF58 enclosure (only the rules the reference itself keeps), LEF58_AREA on a routing layer (AREA a with RECTWIDTH w or EXCEPTRECTANGLE: checked, and patched on the routed net as the reference patches it; a rule with EXCEPTMINWIDTH, EXCEPTEDGELENGTH, EXCEPTMINSIZE, EXCEPTSTEP, MASK or LAYER is skipped with a warning, as the reference skips it; a minimum-area or LEF58_AREA patch on a net with several pieces in one worker goes to the piece it touches, looked up where the patch is, as the reference fixed for OpenROAD issue #12191 does, where the unpatched reference compares the patch box relative to its origin and drops most such patches; one still without a piece is dropped and counted in the report's area_patches_dropped_no_owner, with one DRT-PATCH-DROPPED warning), the orthogonal spacing table of a cut layer (SPACINGTABLE ORTHOGONAL: per row, a cut within WITHIN of a pair of cuts that run parallel on one axis, and closer than SPACING across that run, is marked; each marker names the pair as its victims, each box its cut, as the reference fixed for OpenROAD issue #12198 gives them, where the unpatched reference takes each box's top from the cut's right edge), and a cell obstruction with its own DESIGNRULEWIDTH or SPACING (checked as a blockage of its own, by its own rule). LEF58 max spacing (LEF58_MAXSPACING max CUTCLASS c) runs the router's repair pass as the reference runs it, after an iteration that ends with no marker: its secondary via defs and its search for vias of the class with no other cut within max; a rule with no CUTCLASS is skipped with the reference's warning (DRT-350). Then, per such via, its marker, the three-by-three-gcell region around it and the pin terminals it lands on (marked stubborn), the pin-access re-run for those terminals' instances (a class of their own, the stubborn terminals avoiding every via def of the rule's class, the rows around them choosing their patterns again), and the regions' merge, and the via-swap workers (one per merged region, each trying the layer's secondary via defs on every lonely via in it and keeping a swap only when its check is clean), as the reference does them. Three places differ from the reference, by its own upstream fixes: a stubborn terminal the avoided via defs leave with no access point has its access made again with every via def, with warning DRT-355 (the reference stops with DRT-0073, OpenROAD issue #12199); a via whose secondary defs are all tried goes back to the def it had, and the worker's check is told (the reference takes the layer's default via there and checks the last one it tried, leaving markers for a via that is not in the route: OpenROAD issue #12203); and a worker that holds none of a re-run terminal's new access points leaves the terminal out with warning DRT-356 (the reference stops with DRT-1231, OpenROAD issue #12200). A swapped via whose pad leaves a minimum-step violation on a MINSTEP layer beside the cut layer has its pad end patched out to the MINSTEP length in the worker's check, as the reference patches it. Where the reference itself aborts, this router refuses at the same point with exit 4 (status refused, nothing written; the reason and the report's reference_aborts name the reference's error and the upstream issue): DRT-0073 when a re-run terminal has no access point even with every via def. A net carrying antenna jumpers (the database's hasJumpers, set by global routing's antenna repair) is routed at ten times the off-guide cost.",
    "DIVERGES from the reference BY DECISION where the reference misreads a rule: each divergence is correlated against the reference patched with the fix (named builds), and on every case it does not reach the routes still match the unpatched reference. (1) A LEF58 end-of-line rule's MINLENGTH or MAXLENGTH TWOSIDES is read as written; the reference takes it from PARALLELEDGE's TWOEDGES (upstream OpenROAD issue #12181). (2) A SAMEMASK end-of-line rule's WITHIN is read as written; the reference reads it as 0 (upstream OpenROAD issue #12182). (3) An end-of-line rule with TOCONCAVECORNER or TONOTCHLENGTH is skipped with DRT-357/358; the reference reads it as a plain rule with WITHIN 0 (#12174). (4) An ADJACENTCUTS rule with two WITHIN values is skipped with DRT-359; the reference keeps the first (#12202). (5) A cut spacing LAYER rule the reference never checks is skipped with DRT-353/354, so a SAMENET one no longer narrows the layer's other rules to one net (#12187, #12188). (6) An orthogonal spacing table marker's victims are the cuts (#12198). (7) An area patch's owner is looked up where the patch is (#12191). (8) A per-cut-class LEF58 minimum-cut rule is converted against the default vias; the reference drops it (#12176).",
    "REFUSED rather than approximated, by detailed_route: any other rule family a layer carries (other LEF58 end-of-line clauses, LEF58 minimum step, SPACINGTABLE INFLUENCE, two-wires forbidden spacing, orthogonal width tables, wrong-direction spacing, a metal-width via map and the rest, named in the reason; only the rules the reference itself keeps count); corner spacing with CONCAVECORNER, SAMEMASK or EXCEPTEOL, or in a design that instantiates a BLOCK master; any other LEF58 cut spacing between layers (LAYER), an ADJACENTCUTS rule with SAMENET, SAMEMETAL, SAMEVIA or TWOCUTS SAMECUT, a cut spacing table with SAMEMETAL, and an inter-layer (LAYER) cut spacing table with SAMENET or SAMEMETAL; more than one plain cut spacing rule on a cut layer; a layer whose via pads break its minimum step (the check's min-step patch is not modelled); a non-default rule with hard spacing, via generate rules or wire extension; FIXED wiring, a via or patch with no wire, or a routed net on a non-default rule already in the database; and congested input guides. From iteration 7 a congested worker widens the clip of later rows, and from iteration 23 NEARDRC rips up the nets near each marker; both are transcribed but no correlated design exercises them yet.",
    "SKIPPED WITH A WARNING, neither checked nor refused, by detailed_route and pin_access: a LEF58 end-of-line spacing rule with TOCONCAVECORNER or TONOTCHLENGTH, and a LEF58 minimum-cut rule the reference routes without (one with AREA, SAMEMETALOVERLAP or FULLYENCLOSED, which the reference warns about and skips). The unpatched reference reads either end-of-line form as a plain end-of-line rule with WITHIN 0 (upstream OpenROAD issue #12174); this router follows the reference's fix instead, by decision, and logs its warning (DRT-357 TOCONCAVECORNER, DRT-358 TONOTCHLENGTH). Each skipped rule is warned about at tech load (event DRT-SKIPPED) and listed in the report's skipped_rules (layer, rule, clause, checked false); the status does not change, so sign-off DRC must cover these rules.",
    "NOT CHECKED, informational, by detailed_route and pin_access: a LEF58 layer property the database's LEF reader leaves unparsed (it keeps only the raw text, with an unsupported-property INFO), e.g. ENCLOSUREEDGE, JOINTCORNERSPACING, VIACLUSTER, PRLTWOSIDESSPACING or EOLENCLOSURE. No rule reaches routing, as in the reference. Each is listed per layer and property kind in the report's lef58_not_checked (event DRT-NOT-CHECKED); never a refusal.",
    "REFUSED rather than approximated, by pin_access: a nearby-track access point that keeps an access (a pin that no other round can reach; the rounds themselves run, and a point with no access is dropped as the reference drops it); and every rule family or layer property detailed_route refuses (the access points are judged by the same design-rule check), named in the reason.",
    "The router settings are its defaults: via-access layer the second routing layer (detailed_route takes --via-access-layer), no via-in-pin range, three sparse points per pin, non-preferred tracks allowed. detailed_route routes between the block's minimum and maximum routing layers as the database holds them, with the database's non-default rules; pin_access's top routing layer is the block's maximum routing layer, else the topmost."
  ],
  "invocation": {
    "args_template": ["detailed_route", "--db", "{db}", "--out", "{out}"],
    "optional": [ { "arg": "report", "flag": "-o" }, { "arg": "via_access_layer", "flag": "--via-access-layer" }, { "arg": "end_iteration", "flag": "--end-iteration" }, { "arg": "threads", "flag": "--threads" } ],
    "emits_json": true,
    "findings_exit_codes": [1]
  },
  "inputs": {
    "type": "object",
    "required": ["db", "out"],
    "properties": {
      "db": { "type": "string", "description": "the design database to read (detailed_route: placed, with route guides)" },
      "out": { "type": "string", "description": "where to write the result: detailed_route writes a DEF if the name ends in .def, else a database; pin_access writes a database" },
      "via_access_layer": { "type": "string", "description": "detailed_route: the via-access layer (default the second routing layer)" },
      "end_iteration": { "type": "integer", "description": "detailed_route: stop after this search-and-repair iteration (counted from 0)" },
      "threads": { "type": "integer", "description": "bound the worker threads; leave unset (the results are identical at any count)" },
      "report": { "type": "string", "description": "write the JSON report to FILE instead of stdout" }
    }
  },
  "consumes": ["odb"],
  "threads": {
    "parallel": true,
    "flag": "--threads",
    "env": "VYGES_THREADS",
    "default": "balanced",
    "default_rule": "the machine's physical cores (within this process's CPU affinity), one fewer when there are more than four, at most 16",
    "zero": "the default",
    "serial": 1,
    "deterministic": true,
    "reported": "the run logs the count it chose and why",
    "guidance": "Leave it unset: the default is chosen from the machine and the results are identical at any count. Pass --threads N only to bound CPU or memory, for example when running several jobs at once (give each a share of the cores)."
  },
  "artifacts": [ { "role": "routed_design", "field": "out" } ],
  "assertion": {
    "id": "routed-clean",
    "field": "status",
    "pass_when": { "eq": "written" }
  },
  "exit_codes": { "0": "written", "1": "markers (detailed_route)", "2": "vacuous or error", "3": "refused", "4": "refused where the reference aborts (detailed_route)" },
  "tools": [
    {
      "name": "drt.pin_access",
      "summary": "compute every pin's access points and write them into the database",
      "invocation": {
        "args_template": ["pin_access", "--db", "{db}", "--out", "{out}"],
        "optional": [ { "arg": "report", "flag": "-o" }, { "arg": "max_routing_layer", "flag": "--max-routing-layer" }, { "arg": "threads", "flag": "--threads" } ],
        "emits_json": true
      },
      "inputs": {
        "type": "object",
        "required": ["db", "out"],
        "properties": {
          "db": { "type": "string", "description": "the placed design database to read" },
          "out": { "type": "string", "description": "the database to write, access points included" },
          "max_routing_layer": { "type": "string", "description": "the block's maximum routing layer, set first" },
          "threads": { "type": "integer", "description": "bound the worker threads; leave unset" },
          "report": { "type": "string", "description": "write the JSON report to FILE instead of stdout" }
        }
      },
      "assertion": { "id": "pin-access-written", "field": "status", "pass_when": { "eq": "written" } }
    }
  ]
}
```
