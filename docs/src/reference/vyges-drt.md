# vyges-drt — CLI reference

```text
vyges-drt — detailed routing: pin access

USAGE:
  vyges-drt pin_access (--db IN.odb | --lef A.lef [--lef B.lef …] --def D.def)
                       [--max-routing-layer LAYER] --out OUT.odb [-o REPORT.json]
  vyges-drt --describe
  vyges-drt --help
  vyges-drt --version

pin_access groups the instances into unique classes, finds every pin's access points (cost rounds
of candidates, each planar direction and via checked against the design rules), chooses one
access pattern per class and one per instance along each row of abutting cells, and writes the
results into the database: every master pin's access points, each routed instance terminal's
preferred points, each single-pin port's points. The report (JSON) goes to stdout.

OPTIONS:
  --db IN.odb                 read the design from a database
  --lef FILE / --def FILE     or from LEF files and a DEF
  --max-routing-layer LAYER   set the block's maximum routing layer first (as a
                              signal-layer range ending at LAYER does)
  --out OUT.odb               write the database here
  -o FILE                     write the JSON report to FILE instead of stdout
  --describe                  print a machine-readable JSON description of the command

EXIT STATUS:
  0  written   access points computed and the database written
  2  vacuous   nothing to access: no routed instance terminal and no routed port. NOT a pass;
               nothing is written
  2  error     usage, unreadable input
  3  refused   a step this engine does not model, a terminal with no access point, or a row
               with no pattern combination — see `reason`
```

## `--describe`

```json
{
  "schema": "vyges-tool-descriptor/1.1",
  "name": "drt",
  "summary": "pin access: every pin's access points and each routed instance terminal's preferred points, written into the design database",
  "maturity": "structured",
  "provenance_limitations": [
    "status is one of written, vacuous, refused or error. VACUOUS IS NOT WRITTEN: the design has no routed instance terminal and no routed port. Exit status is 0 for written, 2 for vacuous and for error, 3 for refused.",
    "Correlated stage by stage against an instrumented reference router, and end to end on the database it writes (preferred points, point counts, port points, pin-access indices) on sky130hs designs, and stage by stage on Nangate45 and GF180 designs, 2026-09-25. The correlation harness is not part of this repository.",
    "Design rules modelled in the access trials: shorts (metal and cut), non-sufficient metal, the parallel-run spacing table and cut spacing. A technology with end-of-line spacing is checked WITHOUT it — its trials may pass where they should not.",
    "REFUSED rather than approximated: a nearby-track cost round (a pin that no other round can reach).",
    "Taken as absent: a metal-width via map, unidirectional (multi-mask or rect-only) and right-way-on-grid-only layers, a net's non-default rule without auto-taper.",
    "The router settings are its defaults: via-access layer 2, no via-in-pin range, three sparse points per pin, non-preferred tracks allowed; the top routing layer is the block's maximum routing layer, else the topmost."
  ],
  "invocation": {
    "args_template": ["pin_access", "--db", "{db}", "--out", "{out}"],
    "optional": [ { "arg": "report", "flag": "-o" }, { "arg": "max_routing_layer", "flag": "--max-routing-layer" } ],
    "emits_json": true
  },
  "inputs": {
    "type": "object",
    "required": ["db", "out"],
    "properties": {
      "db": { "type": "string", "description": "the design database to read" },
      "out": { "type": "string", "description": "where to write the database with access points" },
      "max_routing_layer": { "type": "string", "description": "the block's maximum routing layer, set first" },
      "report": { "type": "string", "description": "write the JSON report to FILE instead of stdout" }
    }
  },
  "consumes": ["db"],
  "artifacts": [ { "role": "db", "field": "out" } ],
  "assertion": {
    "id": "pin-access-written",
    "field": "status",
    "equals": "written"
  },
  "exit_codes": { "0": "written", "2": "vacuous or error", "3": "refused" }
}
```
