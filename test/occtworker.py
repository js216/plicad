# SPDX-License-Identifier: GPL-3.0-only
# occtworker.py -- Check STEP files with OCCT, one request per input line
# Copyright (c) 2026 Jakob Kastelic

# Run with FreeCADCmd: FreeCADCmd occtworker.py < requests
#
# Each input line is a JSON request {"step": PATH, "spec": PATH or null},
# with "brep": PATH in place of the spec for a reference solid read from
# a BREP file, or the bare path of a STEP file.  A request with "view":
# [X, Y, Z], the direction toward the eye, and "lines": PATH, a view of
# smview, compares its visible lines with the hidden-line projection of
# OCCT instead (hlr below).  For each one, a line
#   R {"path": ..., "solids": N, "valid": ..., "closed": ...,
#      "volume": V, "area": A, "bop": "ok" or a message, "ref": {...}}
# goes to stdout, or to file descriptor N if the environment holds
# OCCTWORKER_FD=N.  OCCT prints its own trace on stdout as well, so a
# reader of stdout keeps only the lines that start with "R ", and even
# these can follow a piece of trace without its newline.
#
# With a spec (the .sm file of a sheet metal part, or the .part file of
# a solid), the worker builds the part a second time from the spec
# alone, with OCCT primitives, and
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


# Parts of solids (.part files, the extrude and bool programs), with
# literal numbers and names only.  An outline is a list of corners
# (u, v, r) in the plane across its axis, each rounded by a tangent arc
# of radius r.  A drafted circle is a cone.

def corner_wire(a, w, pts):
    n = len(pts)
    ends = []
    for i, (u, v, r) in enumerate(pts):
        p = uv_point(a, u, v, w)
        if r <= 1e-9:
            ends.append((p, None, p))
            continue
        pa = uv_point(a, pts[i - 1][0], pts[i - 1][1], w) - p
        pb = uv_point(a, pts[(i + 1) % n][0], pts[(i + 1) % n][1], w) - p
        pa.normalize()
        pb.normalize()
        h = math.acos(max(-1.0, min(1.0, pa.dot(pb)))) / 2
        d = r / math.tan(h)
        m = pa + pb
        m.normalize()
        c = p + m * (r / math.sin(h))
        ends.append((p + pa * d, c - m * r, p + pb * d))
    edges = []
    for i in range(n):
        t1, mid, t2 = ends[i]
        if mid is not None:
            edges.append(Part.Arc(t1, mid, t2).toShape())
        nxt = ends[(i + 1) % n][0]
        if (nxt - t2).Length > 1e-9:
            edges.append(Part.LineSegment(t2, nxt).toShape())
    return Part.Wire(edges)


def outline(pts):
    """The sides and corner arcs of a profile in (u, v), its region on the
    left: (arc, sharp, a, c, r, ccw) each, a the start, as extrude.c has
    them."""
    n = len(pts)
    area = sum(pts[i][0] * pts[(i + 1) % n][1] -
               pts[(i + 1) % n][0] * pts[i][1] for i in range(n))
    if area < 0:
        pts = pts[::-1]
    arcs = []
    for i, (u, v, r) in enumerate(pts):
        pa = (pts[i - 1][0] - u, pts[i - 1][1] - v)
        pb = (pts[(i + 1) % n][0] - u, pts[(i + 1) % n][1] - v)
        la, lb = math.hypot(*pa), math.hypot(*pb)
        pa, pb = (pa[0] / la, pa[1] / la), (pb[0] / lb, pb[1] / lb)
        if r <= 0:
            arcs.append(((u, v), (u, v), None, 0.0, True))
            continue
        h = math.acos(max(-1.0, min(1.0, pa[0] * pb[0] + pa[1] * pb[1]))) / 2
        d = r / math.tan(h)
        m = (pa[0] + pb[0], pa[1] + pb[1])
        lm = math.hypot(*m)
        c = (u + m[0] / lm * r / math.sin(h), v + m[1] / lm * r / math.sin(h))
        t1 = (u + pa[0] * d, v + pa[1] * d)
        t2 = (u + pb[0] * d, v + pb[1] * d)
        ccw = ((t1[0] - c[0]) * (t2[1] - c[1]) -
               (t1[1] - c[1]) * (t2[0] - c[0])) > 0
        arcs.append((t1, t2, c, r, ccw))
    segs = []
    for i in range(n):
        t1, t2, c, r, ccw = arcs[i]
        if c is not None:
            segs.append([True, False, t1, c, r, ccw])
        nxt = arcs[(i + 1) % n][0]
        if math.hypot(nxt[0] - t2[0], nxt[1] - t2[1]) > 1e-9:
            segs.append([False, c is None, t2, None, 0.0, True])
    return segs


def left_of(segs, i, at_end):
    """The unit normal to the left of side i, at its start or its end."""
    arc, _, a, c, r, ccw = segs[i]
    b = segs[(i + 1) % len(segs)][2]
    if not arc:
        d = (b[0] - a[0], b[1] - a[1])
        ld = math.hypot(*d)
        return (-d[1] / ld, d[0] / ld)
    q = b if at_end else a
    d = ((q[0] - c[0]) / r, (q[1] - c[1]) / r)
    return (-d[0], -d[1]) if ccw else d


def moved_in(segs, d):
    """The outline moved in by d, as far_outline of extrude.c does it."""
    n = len(segs)
    out = []
    for i in range(n):
        arc, sharp, a, c, r, ccw = segs[i]
        lf = left_of(segs, i, False)
        if arc:
            r += -d if ccw else d
        if not sharp or d <= 0:
            a = (a[0] + lf[0] * d, a[1] + lf[1] * d)
        else:
            lp = left_of(segs, (i - 1) % n, True)
            pv = segs[i - 1][2]
            o1 = (pv[0] + lp[0] * d, pv[1] + lp[1] * d)
            d1 = (segs[i][2][0] - pv[0], segs[i][2][1] - pv[1])
            o2 = (a[0] + lf[0] * d, a[1] + lf[1] * d)
            nx = segs[(i + 1) % n][2]
            d2 = (nx[0] - segs[i][2][0], nx[1] - segs[i][2][1])
            den = d1[0] * d2[1] - d1[1] * d2[0]
            t = ((o2[0] - o1[0]) * d2[1] - (o2[1] - o1[1]) * d2[0]) / den
            a = (o1[0] + d1[0] * t, o1[1] + d1[1] * t)
        out.append([arc, sharp, a, c, r, ccw])
    return out


def outline_wire(a, w, segs):
    edges = []
    n = len(segs)
    for i in range(n):
        arc, _, p, c, r, ccw = segs[i]
        q = segs[(i + 1) % n][2]
        if arc:
            m = (p[0] + q[0] - 2 * c[0], p[1] + q[1] - 2 * c[1])
            lm = math.hypot(*m)
            mid = (c[0] + m[0] / lm * r, c[1] + m[1] / lm * r)
            edges.append(Part.Arc(uv_point(a, p[0], p[1], w),
                                  uv_point(a, mid[0], mid[1], w),
                                  uv_point(a, q[0], q[1], w)).toShape())
        else:
            edges.append(Part.LineSegment(uv_point(a, p[0], p[1], w),
                                          uv_point(a, q[0], q[1], w)).toShape())
    return Part.Wire(edges)


def draft_solid(near, far):
    """The solid between two outlines with the same sides: a plane on
    each pair of lines and a ruled face, a cone, on each pair of arcs.
    A loft gives the planes as splines, which cost exactness."""
    faces = [Part.Face(near), Part.Face(far)]
    for e0, e1 in zip(near.Edges, far.Edges):
        if isinstance(e0.Curve, Part.Line):
            p = [e0.Vertexes[0].Point, e0.Vertexes[-1].Point,
                 e1.Vertexes[-1].Point, e1.Vertexes[0].Point]
            faces.append(Part.Face(Part.makePolygon(p + [p[0]])))
        else:
            faces.append(Part.makeRuledSurface(e0, e1))
    shell = Part.Shell(faces)
    shell.sewShape()
    s = Part.Solid(shell)
    if s.Volume < 0:
        s.reverse()
    return s


class SolidPart:
    def __init__(self):
        self.names = {}
        self.prof = {}
        self.solids = {}
        self.last = None

    def ev(self, w):
        return evaluate(w, self.names)

    def build(self, path):
        for words in statements(path):
            if len(words) >= 3 and words[1] == "=":
                self.names[words[0]] = self.ev(" ".join(words[2:]))
            elif words[0] == "profile":
                pts = self.corners(words[4:])
                self.prof[words[1]] = [axis_of(words[3])[0], "corners", pts]
                self.last = words[1]
            elif words[0] == "corner":
                self.prof[self.last][2] += self.corners(words[1:])
            elif words[0] == "circle":
                self.prof[words[1]] = [axis_of(words[3])[0], "circle",
                                       [self.ev(x) for x in words[4:7]]]
                self.last = None
            elif words[0] == "extrude":
                self.solids[words[1]] = self.extrude(words)
            elif words[0] in ("union", "cut", "common"):
                acc = self.solids.pop(words[2])
                for w in words[3:]:
                    b = self.solids.pop(w)
                    acc = {"union": acc.fuse, "cut": acc.cut,
                           "common": acc.common}[words[0]](b)
                self.solids[words[1]] = acc
            else:
                raise Skip("unknown statement " + words[0])
        if len(self.solids) != 1:
            raise Skip("%d solids" % len(self.solids))
        s = list(self.solids.values())[0]
        # Joining the faces on one surface is tidying, and OCCT gets it
        # wrong where a hole touches an edge of its face (fuzzer finding
        # 0021): the solid as cut stands then.
        joined = s.removeSplitter()
        return joined if joined.isValid() else s

    def corners(self, words):
        if len(words) % 3:
            raise Skip("corners come in threes")
        v = [self.ev(w) for w in words]
        return [tuple(v[i:i + 3]) for i in range(0, len(v), 3)]

    def extrude(self, words):
        w0, w1 = self.ev(words[3]), self.ev(words[5])
        a, kind, data = self.prof[words[6]]
        i = 7
        draft = 0.0
        copies = [V(0, 0, 0)]
        while i < len(words):
            if words[i] == "draft":
                draft = self.ev(words[i + 1])
                i += 2
            elif words[i] == "pattern":
                i += 1
                while i + 2 < len(words) and words[i] != "draft":
                    ax, sg = axis_of(words[i])
                    n = int(round(self.ev(words[i + 1])))
                    p = sg * self.ev(words[i + 2])
                    copies = [c + unit(ax, p * j) for c in copies
                              for j in range(n)]
                    i += 3
            else:
                raise Skip("unknown option " + words[i])
        h = abs(w1 - w0)
        sg = 1.0 if w1 > w0 else -1.0
        if kind != "circle":
            if draft:
                near = outline(data)
                far = moved_in(near, h * math.tan(math.radians(draft)))
                s = draft_solid(outline_wire(a, w0, near),
                                outline_wire(a, w1, far))
            else:
                s = Part.Face(corner_wire(a, w0, data)).extrude(
                    unit(a, w1 - w0))
        else:
            u, v, dia = data
            r1 = dia / 2 - h * math.tan(math.radians(draft))
            p = uv_point(a, u, v, w0)
            if draft:
                s = Part.makeCone(dia / 2, r1 if r1 > 1e-9 else 0.0, h, p,
                                  unit(a, sg))
            else:
                s = Part.makeCylinder(dia / 2, h, p, unit(a, sg))
        return fuse([moved(s, shift(c)) for c in copies])


def reference(shape, spec):
    r = {}
    try:
        if spec.endswith(".part"):
            ref = SolidPart().build(spec)
        else:
            ref = SheetPart().build(spec)
    except Skip as e:
        return {"skip": str(e)}
    except Exception as e:
        return {"error": "reference: " + " ".join(str(e).split())[:200]}
    r["volume"] = ref.Volume
    r["valid"] = ref.isValid()
    r["solids"] = len(ref.Solids)
    # OCCT gives the symmetric difference two ways: the sum of the two
    # cuts, and VA + VB - 2 V(A common B).  Each comes out too great at
    # times: the common part of two solids with faces at a tiny angle is
    # wrong, as for a bend of 179.999 degrees (fuzzer finding 0011), and
    # so is a cut of two equal solids where a hole touches an edge of
    # its face (finding 0021), by the whole volume.  The smaller of the
    # two counts: no failure of OCCT is known that makes one too small.
    try:
        common = shape.common(ref).Volume
        r["symdiff"] = min(shape.cut(ref).Volume + ref.cut(shape).Volume,
                           abs(shape.Volume + ref.Volume - 2 * common))
    except Exception as e:
        if spec.endswith(".part"):
            # OCCT fails on its own booleans of solids that touch at a
            # point; that judges nothing of PliCad.
            return {"skip": "common: " + " ".join(str(e).split())[:200]}
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


def brep_reference(shape, path):
    # The solid of the BREP file at path as the reference.
    ref = Part.Shape()
    ref.read(path)
    if len(ref.Solids) != 1:
        return {"error": "reference: %d solids" % len(ref.Solids)}
    ref = ref.Solids[0]
    return {"volume": ref.Volume, "valid": ref.isValid(),
            "symdiff": shape.cut(ref).Volume + ref.cut(shape).Volume}


def check(path, spec, brep=None):
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
    # The solids of a part of solids (.part) may be several; the
    # reference is the part as a whole.
    whole = bool(spec) and spec.endswith(".part")
    s = Part.makeCompound(solids) if whole and len(solids) > 1 else solids[0]
    r["valid"] = all(x.isValid() for x in solids)
    r["closed"] = all(sh.isClosed() for x in solids for sh in x.Shells)
    r["volume"] = sum(x.Volume for x in solids)
    r["area"] = sum(x.Area for x in solids)
    try:
        s.check(True)
        r["bop"] = "ok"
    except Exception as e:
        r["bop"] = " ".join(str(e).split())[:300]
    if spec and (len(solids) == 1 or whole) and r["valid"] and \
            r["closed"] and r["bop"] == "ok":
        r["ref"] = reference(s, spec)
    if brep and len(solids) == 1 and r["valid"]:
        r["ref"] = brep_reference(s, brep)
    return r


# Points along a curve, one each HLR_STEP mm, and the distance within
# which a point of one drawing must have a point of the other.
HLR_STEP = 0.1
HLR_TOL = 0.08


def hlr_points(shapes):
    pts = []
    for s in shapes:
        for e in s.Edges:
            n = max(2, int(e.Length / HLR_STEP) + 1)
            pts += [(v.x, v.y) for v in e.discretize(Number=n)]
    return pts


def view_points(path):
    # The points of the visible lines and arcs of a view of smview, but
    # its centre lines.
    pts = []
    with open(path) as f:
        for line in f:
            w = line.split()
            if len(w) < 7 or w[0] not in ("line", "arc") or w[2] != "v" \
                    or w[1] == "c":
                continue
            x = [float(t) for t in w[3:]]
            if w[0] == "line":
                n = max(2, int(math.hypot(x[2] - x[0], x[3] - x[1]) /
                               HLR_STEP) + 1)
                pts += [(x[0] + (x[2] - x[0]) * k / (n - 1),
                         x[1] + (x[3] - x[1]) * k / (n - 1))
                        for k in range(n)]
            else:
                a0, a1 = math.radians(x[3]), math.radians(x[4])
                n = max(2, int(x[2] * abs(a1 - a0) / HLR_STEP) + 1)
                pts += [(x[0] + x[2] * math.cos(a0 + (a1 - a0) * k / (n - 1)),
                         x[1] + x[2] * math.sin(a0 + (a1 - a0) * k / (n - 1)))
                        for k in range(n)]
    return pts


def lower_left(pts):
    x0 = min(p[0] for p in pts)
    y0 = min(p[1] for p in pts)
    return [(x - x0, y - y0) for x, y in pts]


def missing(a, b):
    # The points of a with no point of b within HLR_TOL.
    grid = {}
    for x, y in b:
        grid.setdefault((int(x // HLR_TOL), int(y // HLR_TOL)), []).append(
            (x, y))
    n = 0
    for x, y in a:
        i, j = int(x // HLR_TOL), int(y // HLR_TOL)
        n += not any((u - x) ** 2 + (v - y) ** 2 < HLR_TOL ** 2
                     for di in (-1, 0, 1) for dj in (-1, 0, 1)
                     for u, v in grid.get((i + di, j + dj), ()))
    return n


def hlr(path, view, lines):
    # The visible lines of the view in lines against those of OCCT, as
    # the points of each with no point of the other near them, in the
    # best of the eight ways to lay the axes of OCCT on those of the
    # view.
    import TechDraw
    r = {"path": path}
    try:
        shape = Part.read(path)
        res = TechDraw.projectEx(shape, FreeCAD.Vector(*view))
    except Exception as e:
        r["error"] = "hlr: %s" % e
        return r
    occt = hlr_points([res[0], res[1], res[3]])
    mine = view_points(lines)
    if not occt or not mine:
        r["hlr"] = {"mine": len(mine), "occt": len(occt), "lost": len(occt),
                    "extra": len(mine)}
        return r
    mine = lower_left(mine)
    best = None
    for sx in (1, -1):
        for sy in (1, -1):
            for swap in (0, 1):
                o = lower_left([((y if swap else x) * sx,
                                 (x if swap else y) * sy) for x, y in occt])
                k = (missing(mine, o), missing(o, mine))
                if best is None or sum(k) < sum(best):
                    best = k
    r["hlr"] = {"mine": len(mine), "occt": len(occt), "extra": best[0],
                "lost": best[1]}
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
        if "view" in req:
            out = hlr(req["step"], req["view"], req["lines"])
        else:
            out = check(req["step"], req.get("spec"), req.get("brep"))
    else:
        out = check(line, None)
    OUT.write("R " + json.dumps(out) + "\n")
    OUT.flush()
