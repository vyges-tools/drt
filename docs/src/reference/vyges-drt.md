# vyges-drt — CLI reference

```text
vyges-drt — detailed routing

USAGE:
  vyges-drt pin_access (--db IN.odb | --lef A.lef [--lef B.lef …] --def D.def)
                       [--max-routing-layer LAYER] --out OUT.odb [-o REPORT.json]
  vyges-drt detailed_route --db IN.odb [--via-access-layer LAYER] --out OUT.def|OUT.odb
                       [-o REPORT.json]
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

OPTIONS:
  --db IN.odb                 read the design from a database
  --lef FILE / --def FILE     or from LEF files and a DEF
  --max-routing-layer LAYER   set the block's maximum routing layer first (as a
                              signal-layer range ending at LAYER does)
  --via-access-layer LAYER    detailed_route: the via-access layer (default the second
                              routing layer)
  --out OUT.odb               write the database here
  -o FILE                     write the JSON report to FILE instead of stdout
  --json                      accepted; the report is JSON either way
  --describe                  print a machine-readable JSON description of the command

EXIT STATUS:
  0  written   access points computed (or the design routed clean) and the output written
  1  markers   detailed_route: routing ended with design-rule markers standing; written
  2  vacuous   nothing to access: no routed instance terminal and no routed port. NOT a pass;
               nothing is written
  2  error     usage, unreadable input
  3  refused   a step this engine does not model (among them a multi-patterned routing
               layer), a terminal with no access point, or a row with no pattern
               combination — see `reason`
```

## `--describe`

```json
{
  "schema": "vyges-tool-descriptor/1.1",
  "openroad_pin": "da9f29f18b6487825aa880597176e0fa97110b31",
  "name": "drt",
  "summary": "detailed routing on the design's route guides: pin access, track assignment and search-and-repair until no design-rule marker stands, the routes written as a DEF or a database; pin access alone is its own command",
  "maturity": "structured",
  "provenance_limitations": [
    "detailed_route status is one of written, markers, refused or error: written is routed with no design-rule marker standing (exit 0); markers is routed and written with markers standing (exit 1, a finding, not a pass); error is exit 2; refused is exit 3.",
    "pin_access status is one of written, vacuous, refused or error. VACUOUS IS NOT WRITTEN: the design has no routed instance terminal and no routed port, and nothing is written. Exit status is 0 for written, 2 for vacuous and for error, 3 for refused.",
    "detailed_route is correlated end to end on the DEF it writes, compared WHOLE and byte for byte against a fresh reference run: 16 of 16 cases, 2026-09-25 -- 14 of the reference router's own regression scripts (the largest 16,880 nets over 5 search-and-repair iterations), its incremental-routing script, and one constructed incremental case. Each stage (guides, rule tables, track assignment, every worker of every iteration) is also correlated against an instrumented reference. The correlation harness is not part of this repository.",
    "An incremental run's DEF cannot tell incremental rip-up from rerouting everything, which writes the same DEF: the report's routed_before and rerouted counts say which ran.",
    "pin_access is correlated stage by stage against an instrumented reference router, and end to end on the database it writes (preferred points, point counts, port points, pin-access indices) on sky130hs designs, and stage by stage on Nangate45 and GF180 designs, 2026-09-25.",
    "Design rules modelled: shorts (metal and cut), non-sufficient metal, the parallel-run spacing table, cut spacing (one plain rule per cut layer), LEF 5.4 end-of-line spacing (with or without a parallel edge), minimum width, minimum area (patched where the router can, reported where it cannot), minimum enclosed area (MINENCLOSEDAREA without a WIDTH; a rule with one is skipped, as the reference skips it), rect-only layers (which are also unidirectional), LEF58 end-of-line keep-out, and LEF58 end-of-line spacing with SPACING, ENDOFLINE, WITHIN, ENDTOEND and PARALLELEDGE (a rule with EXCEPTEXACTWIDTH, FILLCONCAVECORNER or EQUALRECTWIDTH is skipped, as the reference skips it). A net carrying antenna jumpers (the database's hasJumpers, set by global routing's antenna repair) is routed at ten times the off-guide cost.",
    "REFUSED rather than approximated, by detailed_route: any other rule family a layer carries (other LEF58 end-of-line clauses, minimum step, corner spacing, LEF58 enclosure, right-way-on-grid-only layers and the rest, named in the reason; only the rules the reference itself keeps count); a multi-patterned routing layer; LEF 5.4 spacing limited to a width range; a cell whose obstructions carry DESIGNRULEWIDTH or SPACING; a non-default rule with hard spacing, via generate rules or wire extension; FIXED wiring, a via or patch with no wire, or a routed net on a non-default rule already in the database; and congested input guides. From iteration 7 a congested worker widens the clip of later rows, and from iteration 23 NEARDRC rips up the nets near each marker; both are transcribed but no correlated design exercises them yet.",
    "REFUSED rather than approximated, by pin_access: a nearby-track cost round (a pin that no other round can reach); and every rule family or layer property detailed_route refuses (the access points are judged by the same design-rule check), named in the reason.",
    "Taken as absent: a metal-width via map.",
    "The router settings are its defaults: via-access layer the second routing layer (detailed_route takes --via-access-layer), no via-in-pin range, three sparse points per pin, non-preferred tracks allowed. detailed_route routes between the block's minimum and maximum routing layers as the database holds them, with the database's non-default rules; pin_access's top routing layer is the block's maximum routing layer, else the topmost."
  ],
  "invocation": {
    "args_template": ["detailed_route", "--db", "{db}", "--out", "{out}"],
    "optional": [ { "arg": "report", "flag": "-o" }, { "arg": "via_access_layer", "flag": "--via-access-layer" } ],
    "emits_json": true
  },
  "commands": [
    {
      "name": "detailed_route",
      "summary": "route the design on its route guides and write the routes (a .def or .odb out)",
      "args_template": ["detailed_route", "--db", "{db}", "--out", "{out}"],
      "optional": [ { "arg": "report", "flag": "-o" }, { "arg": "via_access_layer", "flag": "--via-access-layer" } ],
      "assertion": { "id": "routed-clean", "field": "status", "pass_when": { "eq": "written" } }
    },
    {
      "name": "pin_access",
      "summary": "compute every pin's access points and write them into the database",
      "args_template": ["pin_access", "--db", "{db}", "--out", "{out}"],
      "optional": [ { "arg": "report", "flag": "-o" }, { "arg": "max_routing_layer", "flag": "--max-routing-layer" } ],
      "assertion": { "id": "pin-access-written", "field": "status", "pass_when": { "eq": "written" } }
    }
  ],
  "inputs": {
    "type": "object",
    "required": ["db", "out"],
    "properties": {
      "db": { "type": "string", "description": "the design database to read (detailed_route: placed, with route guides)" },
      "out": { "type": "string", "description": "where to write the result: detailed_route writes a DEF if the name ends in .def, else a database; pin_access writes a database" },
      "via_access_layer": { "type": "string", "description": "detailed_route: the via-access layer (default the second routing layer)" },
      "max_routing_layer": { "type": "string", "description": "pin_access: the block's maximum routing layer, set first" },
      "report": { "type": "string", "description": "write the JSON report to FILE instead of stdout" }
    }
  },
  "consumes": ["odb"],
  "artifacts": [ { "role": "routed_design", "field": "out" } ],
  "assertion": {
    "id": "routed-clean",
    "field": "status",
    "pass_when": { "eq": "written" }
  },
  "exit_codes": { "0": "written", "1": "markers (detailed_route)", "2": "vacuous or error", "3": "refused" }
}
```
