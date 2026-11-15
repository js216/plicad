# SPDX-License-Identifier: GPL-3.0-only
# objcheck.py -- Check the meshes of an OBJ file against measure(1)
# Copyright (c) 2026 Jakob Kastelic

# Usage: python3 -I test/objcheck.py FILE.obj [MEASURE CHORD]
#
# For each object prints its triangles, whether its mesh is closed (each
# edge of a triangle is an edge of one other, the other way round), its
# volume and its area.  With the output of measure(1) and the chord of
# the mesh, it also checks that the volume and the area of each object
# match those of its part: the volume within the area times the chord,
# the area within one part in a hundred.  Exits 1 on any fault.  The
# fuzzer (fuzz.py) imports it for faults().

import sys


def objects(path):
    """The objects of an OBJ file: name and triangles of points."""
    verts = []
    objs = []
    cur = None
    with open(path) as f:
        for line in f:
            w = line.split()
            if not w or w[0].startswith("#"):
                continue
            if w[0] == "v":
                verts.append(tuple(float(x) for x in w[1:4]))
            elif w[0] in ("o", "g"):
                cur = [" ".join(w[1:]), []]
                objs.append(cur)
            elif w[0] == "f":
                if cur is None:
                    cur = ["", []]
                    objs.append(cur)
                idx = []
                for x in w[1:]:
                    i = int(x.split("/")[0])
                    idx.append(i - 1 if i > 0 else len(verts) + i)
                for k in range(1, len(idx) - 1):
                    cur[1].append((idx[0], idx[k], idx[k + 1]))
    return verts, objs


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def check(verts, tris):
    """Whether the mesh is closed, its volume and its area."""
    edges = {}
    vol = 0.0
    area = 0.0
    for t in tris:
        for k in range(3):
            e = (t[k], t[(k + 1) % 3])
            edges[e] = edges.get(e, 0) + 1
        a, b, c = (verts[i] for i in t)
        n = cross(sub(b, a), sub(c, a))
        area += 0.5 * sum(x * x for x in n) ** 0.5
        vol += sum(x * y for x, y in zip(a, cross(b, c))) / 6.0
    closed = all(n == 1 and edges.get((e[1], e[0]), 0) == 1
                 for e, n in edges.items())
    return closed, vol, area


def measured(path):
    """The volume and the area of each part named in measure output."""
    out = {}
    name = None
    with open(path) as f:
        for line in f:
            w = line.split()
            if not w:
                continue
            if w[0] == "part":
                name = w[1]
                out[name] = {}
            elif w[0] in ("volume", "area") and name is not None:
                out[name][w[0]] = float(w[1])
    return out


def faults(verts, objs, ref, chord, out=None):
    """The faults of the objects against measure output ref (or None),
    each a line; each object a line on out, if given."""
    bad = []
    for name, tris in objs:
        closed, vol, area = check(verts, tris)
        if out is not None:
            out.write("o %s tris %d closed %s volume %.4f area %.4f\n" %
                      (name, len(tris), "yes" if closed else "no", vol,
                       area))
        if not closed:
            bad.append("FAIL %s: the mesh is open" % name)
        if ref is None:
            continue
        r = ref.get(name)
        if r is None:
            bad.append("FAIL %s: not in the measure output" % name)
            continue
        if abs(vol - r["volume"]) > r["area"] * chord + 1e-6 * r["volume"]:
            bad.append("FAIL %s: volume %.4f, measured %.4f" %
                       (name, vol, r["volume"]))
        if abs(area - r["area"]) > 0.01 * r["area"]:
            bad.append("FAIL %s: area %.4f, measured %.4f" %
                       (name, area, r["area"]))
    return bad


def main():
    verts, objs = objects(sys.argv[1])
    ref = measured(sys.argv[2]) if len(sys.argv) > 3 else None
    chord = float(sys.argv[3]) if len(sys.argv) > 3 else 0
    bad = faults(verts, objs, ref, chord, sys.stdout)
    for line in bad:
        print(line)
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
