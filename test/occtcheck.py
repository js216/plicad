# SPDX-License-Identifier: GPL-3.0-only
# occtcheck.py -- Check STEP files of PliCad with OCCT
# Copyright (c) 2026 Jakob Kastelic

# Run with FreeCADCmd: FreeCADCmd occtcheck.py OUT FILE.step...
# FreeCADCmd passes its own arguments too; the ones for this script
# follow its path.  OUT gets one line per file:
#   solid NAME valid|INVALID volume V area A solids K
# with the volume and area of OCCT over its K solids; valid means that
# OCCT finds each solid valid and its boolean check finds no self
# intersection.

import os
import sys

import Part


def bop_ok(s):
    """Whether the boolean operations of OCCT find s free of self
    intersections, touching faces among them."""
    try:
        s.check(True)
    except Exception:
        return False
    return True


def check(path):
    name = os.path.basename(path)[: -len(".step")]
    solids = Part.read(path).Solids
    if not solids:
        return f"solid {name} NOSOLID volume 0 area 0 solids 0"
    good = all(s.isValid() and bop_ok(s) for s in solids)
    ok = "valid" if good else "INVALID"
    vol = sum(s.Volume for s in solids)
    area = sum(s.Area for s in solids)
    return (f"solid {name} {ok} volume {vol:.4f} area {area:.4f} "
            f"solids {len(solids)}")


me = [i for i, a in enumerate(sys.argv) if a.endswith("occtcheck.py")]
with open(sys.argv[me[0] + 1], "w") as f:
    for a in sys.argv[me[0] + 2:]:
        f.write(check(a) + "\n")
