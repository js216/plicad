# SPDX-License-Identifier: GPL-3.0-only
# occtpart.py -- Judge the answer of occtworker.py on the solid of a part
# Copyright (c) 2026 Jakob Kastelic

# Usage: python3 occtpart.py [-n] MEASURE < answer
# The answer is one JSON object of test/occtworker.py, and MEASURE the
# output of measure on the same solid.  It fails with a message unless
# OCCT reads valid, closed solids, clean to its boolean check, with
# the volume and area of measure to 1e-6 of each, and the symmetric
# difference with the reference solid is under 1e-6 of the volume.  With
# -n the answer has no reference solid.  Where MEASURE holds several
# solids, OCCT must read as many, with their volumes and areas in sum;
# an empty solid, of no faces, has none in the STEP file.

import json
import sys

ans = json.loads(sys.stdin.read())
args = sys.argv[1:]
noref = args[0] == "-n"
want = {"volume": 0.0, "area": 0.0, "part": 0}
with open(args[-1]) as f:
    for line in f:
        w = line.split()
        if w and w[0] in ("volume", "area"):
            want[w[0]] += float(w[1])
        elif w and w[0] == "faces" and int(w[1]) > 0:
            want["part"] += 1


def fail(msg):
    sys.exit("%s: %s" % (ans.get("path"), msg))


if want["part"] == 0 and ans.get("solids") == 0:
    sys.exit(0)
if ans.get("solids") != want["part"] or not ans.get("valid") or \
        not ans.get("closed"):
    fail("not the valid closed solids of measure: %s" % json.dumps(ans))
if ans.get("bop") != "ok":
    fail("boolean check: %s" % ans.get("bop"))
for k in ("volume", "area"):
    if abs(ans[k] - want[k]) > 1e-6 * want[k] + 1e-4:
        fail("%s %.6f, measure %.4f" % (k, ans[k], want[k]))
ref = ans.get("ref", {})
if noref:
    sys.exit(0)
if "symdiff" not in ref or not ref.get("valid"):
    fail("no reference: %s" % json.dumps(ref))
if abs(ref["symdiff"]) > 1e-6 * ans["volume"]:
    fail("symmetric difference %.9f" % ref["symdiff"])
