# SPDX-License-Identifier: GPL-3.0-only
# occtworker.py -- Check STEP files with OCCT, one request per input line
# Copyright (c) 2026 Jakob Kastelic

# Run with FreeCADCmd: FreeCADCmd occtworker.py < requests
#
# Each input line is a JSON request {"step": PATH, "spec": PATH or null},
# or the bare path of a STEP file.  For each one, a line
#   R {"path": ..., "solids": N, "valid": ..., "closed": ...,
#      "volume": V, "area": A, "bop": "ok" or a message, "ref": {...}}
# goes to stdout, or to file descriptor N if the environment holds
# OCCTWORKER_FD=N.  OCCT prints its own trace on stdout as well, so a
# reader of stdout keeps only the lines that start with "R ", and even
# these can follow a piece of trace without its newline.
#
# With a spec (the .sm file of the part), the worker builds the
# part a second time from the spec alone, with OCCT primitives, and
# compares the two solids.  The "ref" object holds the volume of this
# reference solid and the volume of the symmetric difference, as
# VA + VB - 2 V(A common B).  The construction covers a subset of each
# language, with literal numbers, names, + - * / and parentheses.  A spec
# outside the subset gives "ref": {"skip": REASON}.  The comparison
# needs a valid STEP solid, since a boolean operation on a broken one
# can run for ever.
#
# The sheet metal construction follows the README model.  The plate is a
# box in its flat frame, with z in [-t, 0], placed by its origin and
# axes.  The bend of a flange is the cross section of the sheet at the
# edge of its parent, revolved about the bend axis.  The axis lies R
# above the top face for a positive angle and R below the bottom face
# for a negative one.  The
# straight leg is a box at the end of the bend, turned by the same
# angle.  Holes, cuts and outlines take material from the solid made so
# far, and corner rounds take the square-minus-disc pieces at the end.

import json
import math
import os
import re
import sys

import FreeCAD
import Part
from FreeCAD import Matrix, Rotation
from FreeCAD import Vector as V

BIG = 1e4


class Skip(Exception):
    pass


# Expressions.

TOKEN = re.compile(r"\s*(?:(\d+\.?\d*(?:[eE][-+]?\d+)?|\.\d+(?:[eE][-+]?\d+)?)"
                   r"|([A-Za-z_]\w*)|(.))")


def evaluate(text, names):
    toks = []
    for num, name, op in TOKEN.findall(text):
        if num:
            toks.append(("n", float(num)))
        elif name:
            if name not in names:
                raise Skip("unknown name " + name)
            toks.append(("n", names[name]))
        elif op.strip():
            toks.append(("o", op))
    pos = [0]

    def peek():
        return toks[pos[0]] if pos[0] < len(toks) else ("e", None)

    def take():
        t = peek()
        pos[0] += 1
        return t

    def expr():
        v = term()
        while peek() in (("o", "+"), ("o", "-")):
            v = v + term() if take()[1] == "+" else v - term()
        return v

    def term():
        v = unary()
        while peek() in (("o", "*"), ("o", "/")):
            v = v * unary() if take()[1] == "*" else v / unary()
        return v

    def unary():
        t = take()
        if t == ("o", "-"):
            return -unary()
        if t == ("o", "+"):
            return unary()
        if t == ("o", "("):
            v = expr()
            if take() != ("o", ")"):
                raise Skip("bad parentheses")
            return v
        if t[0] == "n":
            return t[1]
        raise Skip("unsupported expression " + text)

    v = expr()
    if pos[0] != len(toks):
        raise Skip("unsupported expression " + text)
    return v


def statements(path):
    with open(path) as f:
        for line in f:
            words = line.split("#", 1)[0].split()
            if words:
                yield words


def axis_of(word):
    sign = -1.0 if word[0] == "-" else 1.0
    name = word.lstrip("+-")
    if name not in ("x", "y", "z"):
        raise Skip("bad axis " + word)
    return "xyz".index(name), sign


def unit(a, sign=1.0):
    v = [0.0, 0.0, 0.0]
    v[a] = sign
    return V(*v)


def comp(v, a):
    return (v.x, v.y, v.z)[a]


def frame(o, u, v):
    # The matrix whose columns are u, v and u x v, moved to o.
    n = u.cross(v)
    m = Matrix(u.x, v.x, n.x, o.x, u.y, v.y, n.y, o.y,
               u.z, v.z, n.z, o.z, 0, 0, 0, 1)
    return m


def moved(shape, m):
    s = shape.copy()
    s.transformShape(m)
    return s


def turn(o, d, deg):
    a = Matrix()
    a.move(o)
    b = Matrix()
    b.move(-o)
    return a * Rotation(d, deg).toMatrix() * b


def shift(v):
    m = Matrix()
    m.move(v)
    return m


def box(x0, y0, z0, x1, y1, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def fuse(shapes):
    if len(shapes) == 1:
        return shapes[0]
    return shapes[0].multiFuse(shapes[1:])


def uv_point(a, u, v, w):
    # The 3D point with (u, v) on the two other axes in alphabetical
    # order and w along axis a.
    au = 1 if a == 0 else 0
    av = 1 if a == 2 else 2
    p = [0.0, 0.0, 0.0]
    p[au] = u
    p[av] = v
    p[a] = w
    return V(*p)


def parse_where(words, i, names):
    # Parses "along A [at W depth L] [pattern AX N P [AX N P]]" from
    # words[i].
    if words[i] != "along":
        raise Skip("expected along")
    a = axis_of(words[i + 1])[0]
    lo, hi = None, None
    copies = [V(0, 0, 0)]
    i += 2
    while i < len(words):
        if words[i] == "at" and i + 3 < len(words) and \
                words[i + 2] == "depth":
            w = evaluate(words[i + 1], names)
            d = evaluate(words[i + 3], names)
            lo, hi = w - d / 2, w + d / 2
            i += 4
        elif words[i] == "pattern":
            i += 1
            while i < len(words) and words[i] != "at":
                ax, sg = axis_of(words[i])
                n = int(round(evaluate(words[i + 1], names)))
                p = sg * evaluate(words[i + 2], names)
                copies = [c + unit(ax, p * j) for c in copies
                          for j in range(n)]
                i += 3
        else:
            raise Skip("unknown option " + words[i])
    return a, lo, hi, copies


class SheetPart:
    def __init__(self):
        self.names = {}
        self.slabs = {}
        self.solids = []
        self.rounds = []
        self.t = self.r = self.k = None
        self.outline = None

    def ev(self, w):
        return evaluate(w, self.names)

    def build(self, path):
        for words in statements(path):
            if words[0] == "point":
                self.point(words)
                continue
            self.close_outline()
            if len(words) >= 3 and words[1] == "=":
                self.names[words[0]] = self.ev(" ".join(words[2:]))
            elif words[0] == "sheet":
                self.t, self.r, self.k = (self.ev(w) for w in words[1:4])
            elif words[0] == "plate":
                self.plate(words)
            elif words[0] == "flange":
                self.flange(words)
            elif words[0] in ("hole", "cut"):
                self.through(words)
            elif words[0] == "outline":
                self.outline = (words, [])
            elif words[0] == "round":
                self.rounds.append(words)
            else:
                raise Skip("unsupported statement " + words[0])
        self.close_outline()
        shape = fuse(self.solids)
        for words in self.rounds:
            shape = shape.cut(self.round(words))
        return shape

    def plate(self, words):
        x0, y0, x1, y1 = (self.ev(w) for w in words[2:6])
        o = V(*(self.ev(w) for w in words[7:10]))
        u = unit(*axis_of(words[11]))
        v = unit(*axis_of(words[12]))
        m = frame(o, u, v)
        self.slabs[words[1]] = {"m": m, "rect": (x0, y0, x1, y1),
                                "dir": None}
        self.solids.append(moved(box(x0, y0, -self.t, x1, y1, 0), m))

    def flange(self, words):
        name, parent = words[1], self.slabs[words[2]]
        ax, sg = axis_of(words[3])
        ang = self.ev(words[4])
        leg = self.ev(words[5])
        x0, y0, x1, y1 = parent["rect"]
        pa, pb = (y0, y1) if ax == 0 else (x0, x1)
        c = (x1 if sg > 0 else x0) if ax == 0 else (y1 if sg > 0 else y0)
        sa, sb = pa, pb
        za = zb = None
        rad = self.r
        i = 6
        while i < len(words):
            if words[i] == "at":
                c = self.ev(words[i + 1])
                i += 2
            elif words[i] == "span":
                sa, sb = self.ev(words[i + 1]), self.ev(words[i + 2])
                i += 3
            elif words[i] == "zone":
                za, zb = self.ev(words[i + 1]), self.ev(words[i + 2])
                i += 3
            elif words[i] == "radius":
                rad = self.ev(words[i + 1])
                i += 2
            else:
                raise Skip("unknown flange option " + words[i])
        if za is None:
            za, zb = max(sa, pa), min(sb, pb)
        t = self.t
        ba = (rad + self.k * t) * abs(ang) * math.pi / 180
        m = unit(ax, sg)
        zo = rad if ang > 0 else -t - rad
        o = V(c, 0, zo) if ax == 0 else V(0, c, zo)
        d = m.cross(V(0, 0, 1))

        def pt(across, along, z):
            return V(across, along, z) if ax == 0 else V(along, across, z)

        sec = Part.Face(Part.makePolygon(
            [pt(c, za, -t), pt(c, zb, -t), pt(c, zb, 0), pt(c, za, 0),
             pt(c, za, -t)]))
        zone = sec.revolve(o, d if ang > 0 else -d, abs(ang))
        self.solids.append(moved(zone, parent["m"]))
        c1 = c + sg * ba
        c2 = c1 + sg * leg
        p0 = pt(min(c1, c2), sa, -t)
        p1 = pt(max(c1, c2), sb, 0)
        ms = parent["m"] * turn(o, d, ang) * shift(m * -ba)
        self.solids.append(moved(box(p0.x, p0.y, p0.z, p1.x, p1.y, p1.z),
                                 ms))
        self.slabs[name] = {"m": ms, "dir": (ax, sg),
                            "rect": (p0.x, p0.y, p1.x, p1.y), "end": c2}

    def extent(self):
        b = fuse(self.solids).BoundBox
        return b

    def through(self, words):
        if words[0] == "hole":
            d, u, v = (self.ev(w) for w in words[1:4])
            a, lo, hi, copies = parse_where(words, 4, self.names)
        else:
            u0, v0, u1, v1 = (self.ev(w) for w in words[1:5])
            a, lo, hi, copies = parse_where(words, 5, self.names)
        if lo is None:
            lo, hi = -BIG, BIG
        tools = []
        for c in copies:
            if words[0] == "hole":
                base = uv_point(a, u, v, lo) + c
                tools.append(Part.makeCylinder(d / 2, hi - lo, base,
                                               unit(a)))
            else:
                p0 = uv_point(a, min(u0, u1), min(v0, v1), lo) + c
                p1 = uv_point(a, max(u0, u1), max(v0, v1), hi) + c
                tools.append(box(p0.x, p0.y, p0.z, p1.x, p1.y, p1.z))
        self.solids = [fuse(self.solids).cut(fuse(tools))]

    def point(self, words):
        if self.outline is None:
            raise Skip("point outside an outline")
        k = {"u": self.ev(words[1]), "v": self.ev(words[2]), "arc": None}
        if len(words) == 7:
            k["arc"] = (self.ev(words[4]), self.ev(words[5]),
                        words[6] == "ccw")
        self.outline[1].append(k)

    def close_outline(self):
        if self.outline is None:
            return
        words, pts = self.outline
        self.outline = None
        a, lo, hi, copies = parse_where(words, 1, self.names)
        if lo is None:
            lo, hi = -BIG, BIG
        edges = []
        n = len(pts)
        for i in range(n):
            p, q = pts[i], pts[(i + 1) % n]
            pa = uv_point(a, p["u"], p["v"], lo)
            qa = uv_point(a, q["u"], q["v"], lo)
            if p["arc"] is None:
                edges.append(Part.LineSegment(pa, qa).toShape())
                continue
            cu, cv, ccw = p["arc"]
            r = math.hypot(p["u"] - cu, p["v"] - cv)
            t0 = math.atan2(p["v"] - cv, p["u"] - cu)
            t1 = math.atan2(q["v"] - cv, q["u"] - cu)
            s = math.fmod(t1 - t0 + 4 * math.pi, 2 * math.pi)
            if s < 1e-7:
                s = 2 * math.pi
            if not ccw:
                s -= 2 * math.pi
            tm = t0 + s / 2
            ma = uv_point(a, cu + r * math.cos(tm), cv + r * math.sin(tm),
                          lo)
            edges.append(Part.Arc(pa, ma, qa).toShape())
        face = Part.Face(Part.Wire(edges))
        prism = face.extrude(unit(a, hi - lo))
        tools = [moved(prism, shift(c)) for c in copies]
        self.solids = [fuse(self.solids).cut(fuse(tools))]

    def corner_piece(self, slab, x, y, sx, sy, r):
        # The material of slab outside a round of radius r at its corner
        # (x, y), whose inside lies toward (sx, sy).
        t = self.t
        cx, cy = x + sx * r, y + sy * r
        e = 1e-3
        sq = box(min(x, cx), min(y, cy), -t - e, max(x, cx), max(y, cy), e)
        disc = Part.makeCylinder(r, t + 3 * e, V(cx, cy, -t - 2 * e))
        return moved(sq.cut(disc), slab["m"])

    def round(self, words):
        slab = self.slabs[words[1]]
        r = self.ev(words[2])
        x0, y0, x1, y1 = slab["rect"]
        if len(words) == 3:
            ax, sg = slab["dir"]
            e = slab["end"]
            pieces = []
            for along, s2 in ((y0 if ax == 0 else x0, 1),
                              (y1 if ax == 0 else x1, -1)):
                if ax == 0:
                    pieces.append(self.corner_piece(slab, e, along, -sg,
                                                    s2, r))
                else:
                    pieces.append(self.corner_piece(slab, along, e, s2,
                                                    -sg, r))
            return fuse(pieces)
        pt = V(*(self.ev(w) for w in words[4:7]))
        m = slab["m"]
        n = m.multVec(V(0, 0, 1)) - m.multVec(V(0, 0, 0))
        for x, sx in ((x0, 1), (x1, -1)):
            for y, sy in ((y0, 1), (y1, -1)):
                d = m.multVec(V(x, y, 0)) - pt
                if (d - n * d.dot(n)).Length < 1e-4:
                    return self.corner_piece(slab, x, y, sx, sy, r)
        raise Skip("round at no corner")


def reference(shape, spec):
    r = {}
    try:
        ref = SheetPart().build(spec)
    except Skip as e:
        return {"skip": str(e)}
    except Exception as e:
        return {"error": "reference: " + " ".join(str(e).split())[:200]}
    r["volume"] = ref.Volume
    r["valid"] = ref.isValid()
    try:
        common = shape.common(ref).Volume
        if common < 0.5 * min(shape.Volume, ref.Volume):
            common = shape.common(ref, 1e-6).Volume
        r["symdiff"] = shape.Volume + ref.Volume - 2 * common
    except Exception as e:
        r["error"] = "common: " + " ".join(str(e).split())[:200]
        return r
    # A common part of almost nothing between solids with one box and
    # one volume comes from a failed boolean operation.
    a, b = shape.BoundBox, ref.BoundBox
    same = all(abs(x - y) < 1e-3 for x, y in zip(
        (a.XMin, a.YMin, a.ZMin, a.XMax, a.YMax, a.ZMax),
        (b.XMin, b.YMin, b.ZMin, b.XMax, b.YMax, b.ZMax)))
    if common < 0.5 * min(shape.Volume, ref.Volume) and same and \
            abs(shape.Volume - ref.Volume) < 1e-6 * shape.Volume + 1e-3:
        return {"skip": "the boolean common failed", "volume": ref.Volume}
    return r


def check(path, spec):
    r = {"path": path}
    try:
        shape = Part.read(path)
    except Exception as e:
        r["error"] = "read: %s" % e
        return r
    solids = shape.Solids
    r["solids"] = len(solids)
    if not solids:
        return r
    s = solids[0]
    r["valid"] = all(x.isValid() for x in solids)
    r["closed"] = all(sh.isClosed() for x in solids for sh in x.Shells)
    r["volume"] = sum(x.Volume for x in solids)
    r["area"] = sum(x.Area for x in solids)
    try:
        s.check(True)
        r["bop"] = "ok"
    except Exception as e:
        r["bop"] = " ".join(str(e).split())[:300]
    if spec and len(solids) == 1 and r["valid"] and r["closed"] and \
            r["bop"] == "ok":
        r["ref"] = reference(s, spec)
    return r


OUT = sys.stdout
if os.environ.get("OCCTWORKER_FD"):
    OUT = os.fdopen(int(os.environ["OCCTWORKER_FD"]), "w")
while True:
    line = sys.stdin.readline()
    if not line:
        break
    line = line.strip()
    if not line:
        continue
    if line.startswith("{"):
        req = json.loads(line)
        out = check(req["step"], req.get("spec"))
    else:
        out = check(line, None)
    OUT.write("R " + json.dumps(out) + "\n")
    OUT.flush()
