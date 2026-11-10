# SPDX-License-Identifier: GPL-3.0-only
# fuzz.py -- Endless fuzzer for the PliCad programs
# Copyright (c) 2026 Jakob Kastelic

# Usage: python3 fuzz.py [--cases N] [--no-occt] [--modes M,M...]
#                        [--one MODE SEED] BIN WORK [SEED]
#
# BIN holds sanitized builds of the programs.  WORK receives the log
# (fuzz.log), the current case (case/), the OCCT queue (occt/) and one
# directory per new finding (findings/).  The fuzzer runs until it is
# killed, or for N cases.  --modes limits the modes.  With --one, it
# runs the single case of MODE and SEED from a log line, and keeps its
# files in WORK/case.
#
# Modes of a case:
#   sm    A random valid sheet metal part.  smfold folds it, and every
#         writer runs on the fold.  The outputs of measure must match
#         a model of the part (counts, holes, flat area and box)
#         and with the exact volume identity below.  OCCT checks the
#         STEP file: one valid, closed solid, BOP clean, with the volume
#         and area of measure.  OCCT also builds the part a second time,
#         from the .sm text alone, and the symmetric difference of the
#         two solids must have no volume (occtworker.py).
#   asm   An assembly of generated parts for place, measure and
#         interfere.  Placed parts keep their volume and area, their
#         boxes move with them, parts far apart do not collide, and two
#         copies at one place do.
#   mut   A damaged input file (.sm, .fold, .txt, .params, a
#         source header or a measure output) for its programs.
#   args  Random arguments for a random program.
# A program must either succeed or exit with a message, and never
# crash, hang or trip a sanitizer.
#
# The volume identity rests on two facts.  Holes, cuts, outlines and
# rounded corners take the same material out of the solid and out of
# the blank, as long as they stay clear of the bends.  A bend of angle
# a and length L holds a t (r + t/2) L of material but a t (r + K t) L
# of blank.  So volume = t * flat area + sum of a L t^2 (1/2 - K).
#
# Each log line holds the time, binary version, case number, seed,
# mode, program, verdict and details.  The verdicts are PASS, REJECT
# (smfold refused a generated part), MISMATCH, CRASH,
# SANITIZER, HANG, NOMSG (failure without a message), OCCT (invalid
# STEP or a disagreement with OCCT) and INVALID (an accepted damaged
# input gives an invalid solid).  A new
# finding is logged with NEW in front of its verdict, and a repeated
# one with DUP.  Each finding directory holds the input, the command,
# stderr, and a minimised input in min/ with its command in README.

import hashlib
import json
import math
import os
import random
import re
import select
import shutil
import subprocess
import sys
import time
import traceback

TIMEOUT = 20
OCCT_EVERY = 8
OCCT_TIMEOUT = 60
LOAD_PAUSE = 25
MIN_RUNS = 300
MIN_SECONDS = 600
MUT_OCCT = 0.25
LOG_MAX = 100 * 2 ** 20      # fuzz.log turns into fuzz.log.1 past this size
DISK_MIN = 2 * 2 ** 30       # the fuzzer pauses below this much free disk
FREECADCMD = "/home/jk/prog/freecad/build/bin/FreeCADCmd"
HERE = os.path.dirname(os.path.abspath(__file__))
SAN_ENV = {
    "ASAN_OPTIONS": "detect_leaks=1:abort_on_error=0:exitcode=99:"
                    "allocator_may_return_null=1:handle_abort=1:"
                    "fast_unwind_on_malloc=0:soft_rss_limit_mb=1000:"
                    "max_allocation_size_mb=800",
    "UBSAN_OPTIONS": "print_stacktrace=1:halt_on_error=1:exitcode=99",
}
SAN_RE = re.compile(r"AddressSanitizer|LeakSanitizer|runtime error:|"
                    r"UndefinedBehaviorSanitizer")
FRAME_RE = re.compile(r"#\d+ 0x[0-9a-f]+ in (\w+) \S*?(\w+\.c):(\d+)")
SUMMARY_RE = re.compile(r"SUMMARY: (\w+): (\S+)")
UB_RE = re.compile(r"(\w+\.c):(\d+):\d+: runtime error: ([a-z -]+)")
WRAPPERS = ("xmalloc", "xcalloc", "xrealloc", "xstrdup", "die_msg",
            "malloc", "calloc", "realloc", "strdup", "__interceptor")
TOL = ["-t", "volume=rel:1e-3", "-t", "area=rel:1e-3", "-t",
       "bbox=abs:0.01", "-t", "hole=abs:0.01", "-t", "d=abs:0.001",
       "-t", "axis=abs:0.001", "-t", "flat=abs:0.01"]
NUMBERS = ["0", "-0", "1", "-1", "2.5", "1e-308", "1e308", "-1e308",
           "nan", "inf", "-inf", "1e9", "99999999999999999999", "0x10",
           "1/0", "sqrt(-1)", "(" * 200 + "1" + ")" * 200, "", "--1",
           "1e", ".", "+", "9" * 400, "(" * 30000, ")", "1in", "2mm",
           "90deg", "atan2(1,0)", "min(1)", "max()", "hypot(3,4,5)",
           "1e-9", "-1e-9", "0.0000001", "179.9999999", "180", "-180",
           "100000", "100001", "4294967297", "1e-320", "t", "x*", "(1",
           "1)", "floor(-0.5)", "abs(-2)", "a b", "(1 2)", "1,2"]
WORDS = ["include", "sheet", "plate", "flange", "cut", "hole", "round",
         "origin", "axes", "at", "span", "zone", "radius", "along",
         "depth", "pattern", "outline", "point", "trim", "arc", "ccw",
         "cw", "+x", "-x", "+y", "-y", "+z", "-z", "x", "y", "z", "part",
         "end", "map", "face", "vertex", "edge", "rigid", "bend",
         "place", "rot", "t", "hole", "kind", "prism", "pmap", "ploop",
         "player", "pround", "pxhole", "solid", "loop", "corner",
         "layer", "xhole", "cbore", "rim", "floor", "far", "-", "#",
         "=", "(", ")", "\x00", "\xff", "a.fold", "b.fold", "nope.fold",
         "/", "..", "volume", "bbox", "flat", "holes", "bends", "d",
         "SPDX-License-Identifier:", "--", "//", "/*", "#!"]
AXES = ["+x", "-x", "+y", "-y", "+z", "-z"]
# The programs that print their findings on stdout.
CHECKERS = ("numdiff", "hdrcheck", "lint")
PIPES = {
    "sm": "smfold gen.sm > gen.fold, then measure, tostep (gen.step) "
          "and smflat -f dxf on gen.fold, then occtworker",
    "asm": "place asm.txt > asm.fold, then measure, interfere and tostep "
           "on asm.fold",
    "mut": "the producer of mut.* (smfold or place) > mut.fold, then "
           "measure, tostep (out.step), smflat and interfere on it, and "
           "occtworker for INVALID",
    "args": "the program with the lines of argv as its arguments",
}


def fmt(x):
    # A short exact text for x, rounded to 4 decimals.
    s = "%.4f" % x
    s = s.rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def q(x):
    return float(fmt(x))


# Results of program runs.

class Run:
    def __init__(self, verdict, detail, err, out, sig):
        self.verdict = verdict
        self.detail = detail
        self.err = err
        self.out = out
        self.sig = sig


def san_sig(err):
    # The name of a sanitizer report holds its kind and the first frame
    # in PliCad code below the allocation wrappers.
    m = UB_RE.search(err)
    if m:
        return "ubsan %s %s:%s" % (m.group(3).strip(), m.group(1),
                                   m.group(2))
    kind = "asan"
    m = SUMMARY_RE.search(err)
    if m:
        kind = m.group(2)
    if "LeakSanitizer" in err:
        kind = "leak"
    for fn, src, line in FRAME_RE.findall(err):
        if not fn.startswith(WRAPPERS):
            return "%s %s %s:%s" % (kind, fn, src, line)
    return kind


def run(argv, cwd=None, out=None, stdout_msg=False):
    # Runs one program.  The verdict is OK or FAIL for a clean run, else
    # CRASH, SANITIZER, HANG or NOMSG.
    env = dict(os.environ, **SAN_ENV)
    prog = os.path.basename(argv[0])
    try:
        p = subprocess.run(argv, stdin=subprocess.DEVNULL,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           timeout=TIMEOUT, env=env, cwd=cwd)
    except subprocess.TimeoutExpired as e:
        err = (e.stderr or b"").decode("latin-1")
        return Run("HANG", "timeout %d s" % TIMEOUT, err, b"",
                   "HANG %s" % prog)
    err = p.stderr.decode("latin-1")
    if out is not None:
        with open(out, "wb") as f:
            f.write(p.stdout)
    if SAN_RE.search(err):
        s = san_sig(err)
        return Run("SANITIZER", s, err, p.stdout,
                   "SANITIZER %s %s" % (prog, s))
    if p.returncode < 0:
        d = "signal %d" % -p.returncode
        return Run("CRASH", d, err, p.stdout, "CRASH %s %s" % (prog, d))
    if p.returncode == 0:
        return Run("OK", "", err, p.stdout, None)
    if not err.strip() and not (stdout_msg and p.stdout.strip()):
        d = "exit %d" % p.returncode
        return Run("NOMSG", d, err, p.stdout, "NOMSG %s %s" % (prog, d))
    first = err.strip().splitlines()[0][:300] if err.strip() else \
        "exit %d" % p.returncode
    first = re.sub(r"[^\s/]*/", "", first)
    return Run("FAIL", first[:150], err, p.stdout, None)


class Bad(Exception):
    # A finding in a pipeline, with its verdict, program, signature and
    # detail.
    def __init__(self, verdict, prog, sig, detail, argv=(), err=""):
        Exception.__init__(self, detail)
        self.verdict = verdict
        self.prog = prog
        self.sig = sig
        self.detail = detail
        self.argv = list(argv)
        self.err = err


class Refused(Exception):
    pass


def must(r, argv, gen):
    # Raises Bad for a broken run.  A clean failure of a generated case
    # is a mismatch, and of a damaged one a refusal.
    prog = os.path.basename(argv[0])
    if r.verdict == "OK":
        return r
    if r.verdict == "FAIL":
        if gen:
            raise Bad("MISMATCH", prog, "MISMATCH %s fails" % prog,
                      "fails on a generated part: " + r.detail, argv,
                      r.err)
        raise Refused(r.detail)
    raise Bad(r.verdict, prog, r.sig, r.detail, argv, r.err)


# Small 3D algebra.  A rigid motion is (R, t), and R is a tuple of rows.

def vadd(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vmul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def mat_vec(r, v):
    return tuple(dot(row, v) for row in r)


def mat_mul(a, b):
    cols = list(zip(*b))
    return tuple(tuple(dot(row, c) for c in cols) for row in a)


def rig_pt(m, p):
    return vadd(mat_vec(m[0], p), m[1])


def rig_mul(a, b):
    return (mat_mul(a[0], b[0]), rig_pt(a, b[1]))


def rig_inv(m):
    rt = tuple(zip(*m[0]))
    return (rt, vmul(mat_vec(rt, m[1]), -1))


def rig_rot(o, k, deg):
    n = math.sqrt(dot(k, k))
    x, y, z = vmul(k, 1 / n)
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    C = 1 - c
    r = ((c + x * x * C, x * y * C - z * s, x * z * C + y * s),
         (y * x * C + z * s, c + y * y * C, y * z * C - x * s),
         (z * x * C - y * s, z * y * C + x * s, c + z * z * C))
    return (r, vsub(o, mat_vec(r, o)))


IDENT = (((1, 0, 0), (0, 1, 0), (0, 0, 1)), (0, 0, 0))


def axis_vec(a, s=1.0):
    v = [0.0, 0.0, 0.0]
    v[a] = s
    return tuple(v)


def box_of(points):
    return (tuple(min(p[i] for p in points) for i in range(3)),
            tuple(max(p[i] for p in points) for i in range(3)))


def box_hit(a, b, margin):
    # Whether boxes a and b overlap by more than -margin on every axis.
    return all(min(a[1][i], b[1][i]) - max(a[0][i], b[0][i]) > -margin
               for i in range(3))


def grow(b, d):
    return (tuple(x - d for x in b[0]), tuple(x + d for x in b[1]))


# Expressions: numbers, names, + - * / and parentheses.

TOKEN = re.compile(r"\s*(?:(\d+\.?\d*(?:[eE][-+]?\d+)?|\.\d+(?:[eE][-+]?\d+)?)"
                   r"|([A-Za-z_]\w*)|(.))")


def evaluate(text, names):
    toks = []
    for num, name, op in TOKEN.findall(text):
        if num:
            toks.append(("n", float(num)))
        elif name:
            if name not in names:
                raise Reject("unknown name " + name)
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
            if take()[1] == "*":
                v = v * unary()
            else:
                d = unary()
                if d == 0:
                    raise Reject("division by zero")
                v = v / d
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
                raise Reject("bad parentheses")
            return v
        if t[0] == "n":
            return t[1]
        raise Reject("unsupported expression " + text)

    v = expr()
    if pos[0] != len(toks) or not math.isfinite(v):
        raise Reject("unsupported expression " + text)
    return v


# A model of a sheet metal part, from the text of the .sm file.

EPS = 1e-7


class Reject(Exception):
    pass


def axis_word(w):
    s = -1.0 if w[:1] == "-" else 1.0
    name = w[1:] if w[:1] in "+-" else w
    if name not in ("x", "y", "z") or not w:
        raise Reject("bad axis " + w)
    return "xyz".index(name), s


def rect_norm(x0, y0, x1, y1):
    return (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))


def set_rect(ax, c0, c1, a, b):
    if ax == 0:
        return rect_norm(c0, a, c1, b)
    return rect_norm(a, c0, b, c1)


def rect_gap(a, b):
    # The overlap of rectangles a and b along x and y.  A negative
    # overlap is a gap.
    return (min(a[2], b[2]) - max(a[0], b[0]),
            min(a[3], b[3]) - max(a[1], b[1]))


def arc_sweep(p, q, c, ccw):
    t0 = math.atan2(p[1] - c[1], p[0] - c[0])
    t1 = math.atan2(q[1] - c[1], q[0] - c[0])
    s = math.fmod(t1 - t0 + 4 * math.pi, 2 * math.pi)
    if s < EPS:
        s = 2 * math.pi
    return s if ccw else s - 2 * math.pi


def outline_area(pts):
    # Signed area of an outline of points with optional arcs, exact.
    a = 0.0
    n = len(pts)
    for i in range(n):
        p, nx = pts[i], pts[(i + 1) % n]
        if p[2] is None:
            a += 0.5 * (p[0] * nx[1] - nx[0] * p[1])
        else:
            c = p[2]
            r = math.hypot(p[0] - c[0], p[1] - c[1])
            s = arc_sweep(p, nx, c, p[3])
            a += 0.5 * (c[0] * (nx[1] - p[1]) - c[1] * (nx[0] - p[0]))
            a += 0.5 * r * r * s
    return a


class SmModel:
    def __init__(self):
        self.names = {}
        self.t = None
        self.bodies = []
        self.bends = []
        self.holes = []
        self.removed = 0.0
        self.outline = None
        self.last_hits = []

    def mark(self):
        return (len(self.bodies), len(self.bends), len(self.holes),
                self.removed, dict(self.names), self.outline,
                [len(b["kids"]) for b in self.bodies])

    def rollback(self, mk):
        nb, nbe, nh, rem, names, ol, kids = mk
        del self.bodies[nb:]
        del self.bends[nbe:]
        del self.holes[nh:]
        self.removed = rem
        self.names = names
        self.outline = ol
        for b, k in zip(self.bodies, kids):
            del b["kids"][k:]

    def ev(self, w):
        return evaluate(w, self.names)

    def feed_text(self, text):
        for line in text.split("\n"):
            self.feed(line)
        self.close_outline()

    def feed(self, line):
        words = line.split("#", 1)[0].split()
        self.last_hits = []
        if not words:
            return
        if words[0] == "point":
            if self.outline is None:
                raise Reject("point outside an outline")
            k = [self.ev(words[1]), self.ev(words[2]), None, True]
            if len(words) == 7:
                k[2] = (self.ev(words[4]), self.ev(words[5]))
                k[3] = words[6] == "ccw"
            self.outline[1].append(k)
            return
        self.close_outline()
        w = words[0]
        if len(words) >= 3 and words[1] == "=":
            self.names[w] = self.ev(" ".join(words[2:]))
        elif w == "sheet":
            self.t, self.r, self.k = (self.ev(x) for x in words[1:4])
        elif w == "plate":
            self.plate(words)
        elif w == "flange":
            self.flange(words)
        elif w in ("hole", "cut"):
            self.through(words)
        elif w == "outline":
            self.outline = (words, [])
        elif w == "round":
            self.round(words)
        else:
            raise Reject("unknown statement " + w)

    def body(self, **kw):
        b = {"bend": False, "parent": None, "kids": [], "dir": None}
        b.update(kw)
        self.bodies.append(b)
        b["box"] = self.plc_box(b)
        b["safe"] = self.safe_box(b)
        if b["parent"] is not None:
            self.bodies[b["parent"]]["kids"].append(len(self.bodies) - 1)
        return b

    def point(self, b, x, y, z):
        # The 3D point of flat (x, y, z) on body b, as in fold.c.
        if not b["bend"]:
            return rig_pt(b["m"], (x, y, z))
        qx, qy = b["q"]
        mx, my = b["mv"]
        u = (x - qx) * mx + (y - qy) * my
        o = (qx, qy, b["r"] if b["ang"] > 0 else -self.t - b["r"])
        rot = rig_rot(o, (my, -mx, 0), u / b["ba"] * b["ang"])
        return rig_pt(rig_mul(b["m"], rot), (x - u * mx, y - u * my, z))

    def samples(self, b, n):
        # Points of body b on n + 1 lines across the bend.  The map is
        # affine along the bend axis, so the two ends of each line hold
        # the extremes of the box.
        x0, y0, x1, y1 = b["rect"]
        mx, my = b.get("mv", (1, 0))
        out = []
        for i in range(n + 1):
            for j in (0, 1):
                if abs(mx) > 0.5:
                    x, y = x0 + (x1 - x0) * i / n, (y0, y1)[j]
                else:
                    x, y = (x0, x1)[j], y0 + (y1 - y0) * i / n
                for k in range(2):
                    out.append(self.point(b, x, y, -self.t * k))
        return out

    def plc_box(self, b):
        return box_of(self.samples(b, 8))

    def safe_box(self, b):
        if not b["bend"]:
            return b["box"]
        return grow(box_of(self.samples(b, 64)), 0.02)

    def plate(self, words):
        if self.t is None or self.bodies:
            raise Reject("plate order")
        x0, y0, x1, y1 = (self.ev(x) for x in words[2:6])
        o = tuple(self.ev(x) for x in words[7:10])
        u = axis_vec(*axis_word(words[11]))
        v = axis_vec(*axis_word(words[12]))
        n = cross(u, v)
        r = tuple(zip(u, v, n))
        if not (x1 - x0 > EPS and y1 - y0 > EPS) or dot(n, n) < 0.5:
            raise Reject("bad plate")
        self.body(name=words[1], rect=rect_norm(x0, y0, x1, y1), m=(r, o),
                  depth=0)

    def slab(self, name):
        for i, b in enumerate(self.bodies):
            if not b["bend"] and b["name"] == name:
                return i
        raise Reject("no slab " + name)

    def flange(self, words):
        name = words[1]
        if any(b["name"] == name for b in self.bodies):
            raise Reject("duplicate name")
        pi = self.slab(words[2])
        p = self.bodies[pi]
        ax, sg = axis_word(words[3])
        if ax > 1:
            raise Reject("bad direction")
        ang = self.ev(words[4])
        leg = self.ev(words[5])
        x0, y0, x1, y1 = p["rect"]
        pa, pb = (y0, y1) if ax == 0 else (x0, x1)
        c = (x1 if sg > 0 else x0) if ax == 0 else (y1 if sg > 0 else y0)
        sa, sb = pa, pb
        za = None
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
            else:
                raise Reject("flange option")
        if za is None:
            za, zb = max(sa, pa), min(sb, pb)
        if not (1e-9 < abs(ang) < 180) or not leg > 0 or rad < 0 or \
                not zb - za > EPS or not sb - sa > EPS:
            raise Reject("bad flange")
        t = self.t
        ba = (rad + self.k * t) * abs(ang) * math.pi / 180
        mv = (sg, 0.0) if ax == 0 else (0.0, sg)
        qv = (c, 0.0) if ax == 0 else (0.0, c)
        d = ax * 2 + (sg < 0)
        self.body(name=name, bend=True, parent=pi, m=p["m"], q=qv, mv=mv,
                  ba=ba, ang=ang, r=rad, dir=d,
                  rect=set_rect(ax, c, c + sg * ba, za, zb),
                  depth=p["depth"])
        o = (qv[0], qv[1], rad if ang > 0 else -t - rad)
        turn = rig_rot(o, cross((mv[0], mv[1], 0), (0, 0, 1)), ang)
        shift = (IDENT[0], (-mv[0] * ba, -mv[1] * ba, 0))
        c1 = c + sg * ba
        m = rig_mul(p["m"], rig_mul(turn, shift))
        self.body(name=name, parent=len(self.bodies) - 1, m=m, dir=d,
                  rect=set_rect(ax, c1, c1 + sg * leg, sa, sb), end=
                  c1 + sg * leg, depth=p["depth"] + 1)
        self.bends.append((ang, zb - za))

    def normal(self, b):
        return mat_vec(b["m"][0], (0, 0, 1))

    def hits(self, a, foot):
        # Returns the slabs in the way of a cut through foot, as cut_hits
        # in smfold.c.
        au = 1 if a == 0 else 0
        av = 1 if a == 2 else 2
        out = []
        for i, b in enumerate(self.bodies):
            lo, hi = b["box"]
            if hi[au] < foot[0] + EPS or lo[au] > foot[2] - EPS or \
                    hi[av] < foot[1] + EPS or lo[av] > foot[3] - EPS or \
                    hi[a] < foot[4] + EPS or lo[a] > foot[5] - EPS:
                continue
            if b["bend"]:
                raise Reject("the cut crosses a bend")
            n = self.normal(b)
            if abs(abs(n[a]) - 1) > 1e-9:
                raise Reject("the cut crosses a slab at an angle")
            s0 = rig_pt(b["m"], (0, 0, 0))[a]
            s1 = rig_pt(b["m"], (0, 0, -self.t))[a]
            if min(s0, s1) < foot[4] - EPS or max(s0, s1) > foot[5] + EPS:
                raise Reject("the cut depth ends inside a slab")
            out.append(i)
        return out

    def slab_flat(self, b, a, u, v):
        p = [0.0, 0.0, 0.0]
        p[1 if a == 0 else 0] = u
        p[1 if a == 2 else 2] = v
        p[a] = rig_pt(b["m"], (0, 0, 0))[a]
        return rig_pt(rig_inv(b["m"]), tuple(p))

    def where(self, words, i):
        # Parses "along A [at W depth L] [pattern ...]"; returns the axis,
        # the depth range and the (du, dv) of the copies.
        if i >= len(words) or words[i] != "along":
            raise Reject("expected along")
        a = axis_word(words[i + 1])[0]
        au = 1 if a == 0 else 0
        lo, hi = -1e300, 1e300
        dirs = []
        i += 2
        while i < len(words):
            if words[i] == "at" and i + 3 < len(words) and \
                    words[i + 2] == "depth":
                w, d = self.ev(words[i + 1]), self.ev(words[i + 3])
                lo, hi = w - d / 2, w + d / 2
                i += 4
            elif words[i] == "pattern":
                i += 1
                while i < len(words) and words[i] != "at":
                    if len(dirs) == 2 or i + 2 >= len(words):
                        raise Reject("bad pattern")
                    ax, sg = axis_word(words[i])
                    if ax == a:
                        raise Reject("pattern along the cut axis")
                    n = self.ev(words[i + 1])
                    if abs(n - round(n)) > 1e-9 or not 1 <= n <= 100000:
                        raise Reject("pattern count")
                    dirs.append((ax == au, int(round(n)),
                                 sg * self.ev(words[i + 2])))
                    i += 3
                if not dirs:
                    raise Reject("empty pattern")
            else:
                raise Reject("unknown option")
        copies = [(0.0, 0.0)]
        for is_u, n, p in dirs:
            copies = [(du + (j * p if is_u else 0),
                       dv + (0 if is_u else j * p))
                      for du, dv in copies for j in range(n)]
        if len(copies) > 2000:
            raise Reject("too many copies for the model")
        return a, lo, hi, copies

    def through(self, words):
        if words[0] == "hole":
            d, u, v = (self.ev(x) for x in words[1:4])
            if not d > 0:
                raise Reject("hole size")
            a, lo, hi, copies = self.where(words, 4)
            foot = (u - d / 2, v - d / 2, u + d / 2, v + d / 2)
        else:
            u0, v0, u1, v1 = (self.ev(x) for x in words[1:5])
            a, lo, hi, copies = self.where(words, 5)
            foot = (min(u0, u1), min(v0, v1), max(u0, u1), max(v0, v1))
        for du, dv in copies:
            f = (foot[0] + du, foot[1] + dv, foot[2] + du, foot[3] + dv,
                 lo, hi)
            hits = self.hits(a, f)
            self.last_hits.append(hits)
            for i in hits:
                b = self.bodies[i]
                if words[0] == "hole":
                    cu, cv = (f[0] + f[2]) / 2, (f[1] + f[3]) / 2
                    p = self.slab_flat(b, a, cu, cv)
                    self.holes.append((i, p[0], p[1], d / 2, a))
                else:
                    self.removed += self.cut_area(b, a, f)

    def cut_area(self, b, a, f):
        # The area of slab b inside the rectangle f of the (U, V) plane.
        # A rectangle inside the slab keeps its area.  A rectangle across
        # the edge keeps its overlap with the slab, for a slab with its
        # flat axes along U and V.
        ps = [self.slab_flat(b, a, u, v) for u in (f[0], f[2])
              for v in (f[1], f[3])]
        x0, y0, x1, y1 = b["rect"]
        if all(x0 <= p[0] <= x1 and y0 <= p[1] <= y1 for p in ps):
            return (f[2] - f[0]) * (f[3] - f[1])
        g = rect_gap(rect_norm(ps[0][0], ps[0][1], ps[3][0], ps[3][1]),
                     b["rect"])
        return max(g[0], 0) * max(g[1], 0)

    def close_outline(self):
        if self.outline is None:
            return
        words, pts = self.outline
        self.outline = None
        if len(pts) < 2:
            raise Reject("short outline")
        a, lo, hi, copies = self.where(words, 1)
        us = [p[0] for p in pts]
        vs = [p[1] for p in pts]
        for p in pts:
            if p[2] is not None:
                r = math.hypot(p[0] - p[2][0], p[1] - p[2][1])
                us += [p[2][0] - r, p[2][0] + r]
                vs += [p[2][1] - r, p[2][1] + r]
        area = abs(outline_area(pts))
        for du, dv in copies:
            f = (min(us) + du, min(vs) + dv, max(us) + du, max(vs) + dv,
                 lo, hi)
            hits = self.hits(a, f)
            self.last_hits.append(hits)
            self.removed += area * len(hits)

    def round(self, words):
        i = self.slab(words[1])
        r = self.ev(words[2])
        if not r > 0:
            raise Reject("round radius")
        b = self.bodies[i]
        if len(words) == 3:
            if b["parent"] is None:
                raise Reject("round works on flanges")
            self.removed += 2 * (1 - math.pi / 4) * r * r
        else:
            self.removed += (1 - math.pi / 4) * r * r

    # The expected outputs.

    def flat_area(self):
        a = sum((b["rect"][2] - b["rect"][0]) * (b["rect"][3] - b["rect"][1])
                for b in self.bodies)
        return a - self.removed - sum(math.pi * h[3] ** 2
                                      for h in self.holes)

    def flat_box(self):
        x0 = min(b["rect"][0] for b in self.bodies)
        y0 = min(b["rect"][1] for b in self.bodies)
        x1 = max(b["rect"][2] for b in self.bodies)
        y1 = max(b["rect"][3] for b in self.bodies)
        return max(x1 - x0, y1 - y0), min(x1 - x0, y1 - y0)

    def bend_sum(self):
        return sum(math.radians(abs(a)) * L * self.t ** 2 * (0.5 - self.k)
                   for a, L in self.bends)

    def holes3(self):
        out = []
        for i, x, y, r, a in self.holes:
            b = self.bodies[i]
            c = rig_pt(b["m"], (x, y, -self.t / 2))
            n = self.normal(b)
            out.append((round(abs(n[0])), round(abs(n[1])), round(abs(n[2])),
                        2 * r, c[0], c[1], c[2]))
        return sorted(out)


def parse_measure(data):
    parts = []
    for line in data.decode("latin-1").splitlines():
        w = line.split()
        if not w:
            continue
        if w[0] == "part":
            parts.append({"name": " ".join(w[1:]), "holes": []})
            continue
        if not parts:
            raise ValueError("no part line")
        m = parts[-1]
        if w[0] in ("volume", "area"):
            m[w[0]] = float(w[1])
        elif w[0] == "bbox":
            m["bbox"] = [float(x) for x in w[1:7]]
        elif w[0] == "hole":
            m["holes"].append((int(w[7]), int(w[8]), int(w[9]),
                               float(w[5]), float(w[1]), float(w[2]),
                               float(w[3])))
        elif w[0] == "flat":
            m["flat"] = (float(w[1]), float(w[2]))
            m["flat_area"] = float(w[w.index("area") + 1])
            m["nholes"] = int(w[w.index("holes") + 1])
            m["bends"] = int(w[w.index("bends") + 1])
    return parts


def close(a, b, tol):
    return abs(a - b) <= tol


def compare_holes(got, want):
    # Matches the hole lines of measure with the model, in any order.
    left = list(want)
    for h in got:
        for k, w in enumerate(left):
            if h[:3] == w[:3] and all(abs(h[i] - w[i]) < 2e-3
                                      for i in range(3, 7)):
                del left[k]
                break
        else:
            return "hole %s not in the model" % (h,)
    return "unmatched hole %s" % (left[0],) if left else ""


def check_sheet(model, m):
    # Compares the measure output m of one sheet part with the model.
    # Returns (check name, detail), or None.
    for key in ("volume", "area", "flat_area", "bends", "bbox"):
        if key not in m:
            return "format", "measure prints no " + key
    if m["bends"] != len(model.bends):
        return "bends", "bends %d, expected %d" % (m["bends"],
                                                   len(model.bends))
    if m["nholes"] != len(model.holes):
        return "holes", "holes %d, expected %d" % (m["nholes"],
                                                   len(model.holes))
    bad = compare_holes(m["holes"], model.holes3())
    if bad:
        return "hole lines", bad
    t = model.t
    tol = 6e-4 * t + 2e-4 + 1e-9 * m["volume"]
    want = t * m["flat_area"] + model.bend_sum()
    if not close(m["volume"], want, tol):
        return "identity", "volume %.4f, t * flat area + bends %.4f" % (
            m["volume"], want)
    fa = model.flat_area()
    if not close(m["flat_area"], fa, 1e-3 + 1e-9 * fa):
        return "flat area", "flat area %.3f, model %.4f" % (
            m["flat_area"], fa)
    fw, fh = model.flat_box()
    if not (close(m["flat"][0], fw, 2e-4) and close(m["flat"][1], fh, 2e-4)):
        return "flat box", "flat %.4f %.4f, model %.4f %.4f" % (
            m["flat"][0], m["flat"][1], fw, fh)
    return None


# The generator of sheet metal parts.

class SheetGen:
    def __init__(self, rng):
        self.rng = rng
        self.model = SmModel()
        self.lines = []
        self.taken = {}

    def num(self, x):
        # Writes x, now and then as a sum or through a name.
        s = fmt(x)
        r = self.rng.random()
        if r < 0.04:
            a = fmt(self.rng.uniform(-5, 5))
            return "(%s+%s)" % (a, fmt(float(s) - float(a)))
        if r < 0.06:
            return "(%s*2)/2" % s
        return s

    def add(self, line, ok=None):
        mk = self.model.mark()
        try:
            self.model.feed(line)
            if ok is not None and not ok():
                raise Reject("generator check")
        except (Reject, ZeroDivisionError, ValueError, IndexError):
            self.model.rollback(mk)
            return False
        self.lines.append(line)
        return True

    def gen(self):
        rng = self.rng
        t = rng.choice([0.5, 0.8, 1, 1.2, 1.5, 2, 2.286, 3])
        r = q(rng.uniform(0.2, 3) * t)
        k = q(rng.uniform(0.25, 0.5))
        if rng.random() < 0.15:
            self.add("T = %s" % fmt(t))
            self.add("R = %s" % fmt(r))
            self.add("sheet T R %s" % fmt(k))
        else:
            self.add("sheet %s %s %s" % (fmt(t), fmt(r), fmt(k)))
        w = q(rng.uniform(25, 200))
        h = q(rng.uniform(25, 200))
        x0 = q(rng.choice([0, 0, rng.uniform(-50, 50)]))
        y0 = q(rng.choice([0, 0, rng.uniform(-50, 50)]))
        u = rng.choice(AXES)
        v = rng.choice([a for a in AXES if a[1] != u[1]])
        o = [fmt(rng.choice([0, rng.uniform(-80, 80)])) for _ in range(3)]
        self.add("plate base %s %s %s %s origin %s %s %s axes %s %s" % (
            fmt(x0), fmt(y0), self.num(x0 + w), self.num(y0 + h),
            o[0], o[1], o[2], u, v))
        for i in range(rng.choice([0, 1, 2, 3, 4, 4, 5, 6, 8, 10])):
            self.flange("f%d" % i)
        self.rounds()
        feats = []
        for i, b in enumerate(self.model.bodies):
            if not b["bend"]:
                n = rng.randint(0, 4 if b["parent"] is None else 2)
                feats += [i] * n
        rng.shuffle(feats)
        for i in feats:
            self.feature(i)
        return "\n".join(self.lines) + "\n"

    def angle(self):
        rng = self.rng
        x = rng.random()
        if x < 0.5:
            a = 90
        elif x < 0.7:
            a = rng.choice([30, 45, 60, 120, 135, 150, 180 - 1e-3])
        else:
            a = q(rng.uniform(5, 175))
        return a * rng.choice([-1, 1])

    def flange(self, name):
        rng = self.rng
        m = self.model
        slabs = [i for i, b in enumerate(m.bodies)
                 if not b["bend"] and b["depth"] < 4]
        pi = 0 if rng.random() < 0.4 else rng.choice(slabs)
        p = m.bodies[pi]
        dirs = [0, 1, 2, 3]
        if p["dir"] is not None:
            dirs.remove(p["dir"] ^ 1)
        d = rng.choice(dirs)
        ax, sg = d // 2, (1 if d % 2 == 0 else -1)
        word = ("+" if sg > 0 else "-") + "xy"[ax]
        x0, y0, x1, y1 = p["rect"]
        pa, pb = (y0, y1) if ax == 0 else (x0, x1)
        e = pb - pa
        ang = self.angle()
        leg = q(rng.uniform(max(1.0, m.t), max(2.0, min(80, 0.8 * e))))
        opts = ""
        x = rng.random()
        if x < 0.25:
            pass
        elif x < 0.75 or e < 4:
            a = q(pa + rng.uniform(0, e / 2 - 0.5))
            b = q(pb - rng.uniform(0, e / 2 - 0.5))
            opts = " span %s %s" % (self.num(a), self.num(b))
        else:
            a = q(pa - rng.uniform(0, 10))
            b = q(pb + rng.uniform(0, 10))
            opts = " span %s %s" % (self.num(a), self.num(b))
            if rng.random() < 0.6:
                za = q(max(a, pa) + rng.uniform(0, e / 3))
                zb = q(min(b, pb) - rng.uniform(0, e / 3))
                opts += " zone %s %s" % (self.num(za), self.num(zb))
        line = "flange %s %s %s %s %s%s" % (name, p["name"], word,
                                           fmt(ang), self.num(leg), opts)
        self.add(line, lambda: self.clear(len(m.bodies) - 2))

    def clear(self, zi):
        # Whether the new zone zi and its slab zi + 1 keep clear of the
        # other bodies, in the flat and in 3D.
        m = self.model
        bs = m.bodies
        for ni in (zi, zi + 1):
            n = bs[ni]
            for j, b in enumerate(bs[:zi]):
                link = (ni == zi and j == n["parent"])
                grand = ni == zi + 1 and j == bs[zi]["parent"]
                gap = 1e-3 if grand else 0.5
                ox, oy = rect_gap(n["rect"], b["rect"])
                if link:
                    if ox > 1e-6 and oy > 1e-6:
                        return False
                elif ox > -gap and oy > -gap:
                    sib = (n["bend"] and b["bend"] and
                           b["parent"] == n["parent"] and
                           abs(ox) < 1e-9 and abs(oy) < 1e-9)
                    if not sib:
                        return False
                if link:
                    continue
                if grand:
                    margin = -1e-6
                elif n["bend"] and b["bend"] and \
                        b["parent"] == n["parent"]:
                    margin = -1e-6
                else:
                    margin = 0.05
                if box_hit(n["safe"], b["safe"], margin):
                    return False
        return True

    def rounds(self):
        rng = self.rng
        m = self.model
        for i, b in enumerate(m.bodies):
            if b["bend"]:
                continue
            x0, y0, x1, y1 = b["rect"]
            if b["parent"] is not None:
                if b["kids"] or rng.random() > 0.3:
                    continue
                ax = b["dir"] // 2
                leg = (x1 - x0) if ax == 0 else (y1 - y0)
                span = (y1 - y0) if ax == 0 else (x1 - x0)
                rr = q(rng.uniform(0.2, 0.45 * min(leg, span)))
                if rr <= 0.05:
                    continue
                if self.add("round %s %s" % (b["name"], fmt(rr))):
                    e = b["end"]
                    for al in ((y0, y1) if ax == 0 else (x0, x1)):
                        pt = (e, al) if ax == 0 else (al, e)
                        self.take(i, pt[0] - rr, pt[1] - rr, pt[0] + rr,
                                  pt[1] + rr)
                continue
            self.plate_rounds(i, b)

    def plate_rounds(self, i, b):
        rng = self.rng
        m = self.model
        x0, y0, x1, y1 = b["rect"]
        for cx in (x0, x1):
            for cy in (y0, y1):
                if rng.random() > 0.3:
                    continue
                lim = min(x1 - x0, y1 - y0) / 3
                for k in b["kids"]:
                    z = m.bodies[k]
                    zx0, zy0, zx1, zy1 = z["rect"]
                    if z["dir"] // 2 == 0:
                        if abs(zx0 - cx) < 1e-9 or abs(zx1 - cx) < 1e-9:
                            lim = min(lim, max(0, zy0 - cy, cy - zy1))
                    elif abs(zy0 - cy) < 1e-9 or abs(zy1 - cy) < 1e-9:
                        lim = min(lim, max(0, zx0 - cx, cx - zx1))
                if lim < 0.5:
                    continue
                rr = q(rng.uniform(0.2, 0.8 * lim))
                z = rng.choice([0, -m.t])
                p = rig_pt(b["m"], (cx, cy, z))
                line = "round %s %s at %s %s %s" % (
                    b["name"], fmt(rr), fmt(p[0]), fmt(p[1]), fmt(p[2]))
                if self.add(line):
                    self.take(i, cx - rr, cy - rr, cx + rr, cy + rr)

    def take(self, i, x0, y0, x1, y1):
        self.taken.setdefault(i, []).append((x0, y0, x1, y1))

    def free(self, i, box):
        b = self.model.bodies[i]["rect"]
        if box[0] < b[0] + 0.3 or box[1] < b[1] + 0.3 or \
                box[2] > b[2] - 0.3 or box[3] > b[3] - 0.3:
            return False
        for o in self.taken.get(i, []):
            ox, oy = rect_gap(box, o)
            if ox > -0.4 and oy > -0.4:
                return False
        return True

    def feature(self, i):
        # A hole, cut or outline on slab i, with an optional pattern.  It
        # is made in the (U, V) plane of the statement, and its flat box
        # on the slab must be free.
        rng = self.rng
        m = self.model
        b = m.bodies[i]
        n = m.normal(b)
        a = max(range(3), key=lambda k: abs(n[k]))
        # smfold takes a slab as across the axis while |cos| is within
        # 1e-9 of 1, a tilt of up to 0.0026 degrees, and cuts its holes
        # square to the slab.  The generator needs a slab square to the
        # axis.
        if any(abs(n[k]) > 1e-12 for k in range(3) if k != a):
            return
        au = 1 if a == 0 else 0
        av = 1 if a == 2 else 2
        r3 = b["m"]
        ex = mat_vec(r3[0], (1, 0, 0))
        turned = abs(abs(ex[au]) - 1) > 1e-9 and abs(abs(ex[av]) - 1) > 1e-9

        def uv(x, y):
            p = rig_pt(r3, (x, y, 0))
            return p[au], p[av]

        def flat(u, v):
            p = m.slab_flat(b, a, u, v)
            return p[0], p[1]

        x0, y0, x1, y1 = b["rect"]
        cs = [uv(x, y) for x in (x0, x1) for y in (y0, y1)]
        ulo, uhi = min(c[0] for c in cs), max(c[0] for c in cs)
        vlo, vhi = min(c[1] for c in cs), max(c[1] for c in cs)
        size = min(x1 - x0, y1 - y0)
        if size < 2:
            return
        kind = rng.choice(["hole"] * 5 + ["cut"] * 3 + ["outline"] * 3)
        if kind == "cut" and turned and rng.random() > 0.1:
            kind = "hole"
        lim = 0.35 * size if not turned else 0.2 * size
        if kind == "hole":
            fw = fh = q(rng.uniform(0.3, max(0.4, min(12, lim))))
        else:
            fw = q(rng.uniform(0.5, max(0.6, lim)))
            fh = q(rng.uniform(0.5, max(0.6, lim)))
        pats = []
        if rng.random() < 0.4:
            for dim in rng.sample([0, 1], rng.choice([1, 1, 2])):
                cnt = rng.randint(2, 5)
                pitch = q(((fw, fh)[dim] + rng.uniform(0.5, 6)))
                pats.append((dim, rng.choice([-1, 1]), cnt, pitch))
        span = [fw, fh]
        lo = [0.0, 0.0]
        for dim, sg, cnt, pitch in pats:
            span[dim] += pitch * (cnt - 1)
            if sg < 0:
                lo[dim] = -pitch * (cnt - 1)
        if span[0] > uhi - ulo - 1 or span[1] > vhi - vlo - 1:
            return
        bu = q(rng.uniform(ulo + 0.4, uhi - 0.4 - span[0]))
        bv = q(rng.uniform(vlo + 0.4, vhi - 0.4 - span[1]))
        fu, fv = bu - lo[0], bv - lo[1]
        offs = [(0.0, 0.0)]
        for dim, sg, cnt, pitch in pats:
            offs = [(du + (sg * j * pitch if dim == 0 else 0),
                     dv + (sg * j * pitch if dim == 1 else 0))
                    for du, dv in offs for j in range(cnt)]
        boxes = []
        for du, dv in offs:
            if kind == "hole":
                cx, cy = flat(fu + du + fw / 2, fv + dv + fh / 2)
                boxes.append((cx - fw / 2, cy - fw / 2, cx + fw / 2,
                              cy + fw / 2))
                continue
            ps = [flat(fu + du + su * fw, fv + dv + sv * fh)
                  for su in (0, 1) for sv in (0, 1)]
            boxes.append((min(p[0] for p in ps), min(p[1] for p in ps),
                          max(p[0] for p in ps), max(p[1] for p in ps)))
        if not all(self.free(i, bx) for bx in boxes):
            return
        words = []
        for dim, sg, cnt, pitch in pats:
            sign = "-" if sg < 0 else rng.choice(["", "+"])
            words.append("%s%s %d %s" % (sign, "xyz"[(au, av)[dim]], cnt,
                                         fmt(pitch)))
        pat = " pattern " + " ".join(words) if words else ""
        body = []
        if kind == "hole":
            head = "hole %s %s %s" % (fmt(fw), fmt(fu + fw / 2),
                                      fmt(fv + fh / 2))
        elif kind == "cut":
            cut = [fmt(fu), fmt(fv), fmt(fu + fw), fmt(fv + fh)]
            if rng.random() < 0.5:
                cut = [cut[2], cut[1], cut[0], cut[3]]
            head = "cut " + " ".join(cut)
        else:
            head = "outline"
            body = self.outline_points(fu, fv, fw, fh)
            if body is None:
                return
        w = rig_pt(r3, (0, 0, -m.t / 2))[a]
        foot = (bu, bv, bu + span[0], bv + span[1])
        depth = ""
        if rng.random() < 0.5 or not self.line_clear(i, a, foot, None):
            dl = q(m.t * (1 + rng.uniform(0.2, 1)))
            if not self.line_clear(i, a, foot, (w, dl)):
                return
            depth = " at %s depth %s" % (fmt(w), fmt(dl))
        line = "%s along %s%s%s" % (head, "xyz"[a], depth, pat)

        def one_hit():
            return all(h == [i] for h in m.last_hits)

        if not body:
            if self.add(line, one_hit):
                for bx in boxes:
                    self.take(i, *bx)
            return
        mk = m.mark()
        nl = len(self.lines)
        ok = self.add(line)
        for p in body:
            ok = ok and self.add(p)
        if ok:
            try:
                m.close_outline()
                ok = one_hit()
            except Reject:
                ok = False
        if not ok:
            m.rollback(mk)
            m.outline = None
            del self.lines[nl:]
            return
        for bx in boxes:
            self.take(i, *bx)

    def outline_points(self, fu, fv, fw, fh):
        # The point lines of a random outline in the box (fu, fv, fw, fh)
        # of the (U, V) plane.  The outline is a convex polygon or a slot
        # with round ends.
        rng = self.rng
        pts = []
        if rng.random() < 0.5:
            n = rng.randint(3, 6)
            angs = sorted(rng.uniform(0, 2 * math.pi) for _ in range(n))
            if any(b - a < 0.3 for a, b in zip(angs, angs[1:] +
                                               [angs[0] + 2 * math.pi])):
                return None
            cu, cv = fu + fw / 2, fv + fh / 2
            for t in angs:
                pts.append((cu + fw / 2 * math.cos(t),
                            cv + fh / 2 * math.sin(t), None))
        else:
            rad = q(min(fw, fh) / 2)
            if fw >= fh:
                c1 = (q(fu + rad), q(fv + rad))
                c2 = (q(fu + fw - rad), c1[1])
                pts = [(c1[0], c1[1] - rad, None), (c2[0], c2[1] - rad, c2),
                       (c2[0], c2[1] + rad, None), (c1[0], c1[1] + rad, c1)]
            else:
                c1 = (q(fu + rad), q(fv + rad))
                c2 = (c1[0], q(fv + fh - rad))
                pts = [(c1[0] + rad, c1[1], None), (c2[0] + rad, c2[1], c2),
                       (c2[0] - rad, c2[1], None), (c1[0] - rad, c1[1], c1)]
            if abs(c1[0] - c2[0]) + abs(c1[1] - c2[1]) < 1e-3:
                return None
        rev = rng.random() < 0.5
        # An arc runs from its point to the next one.  In reverse order
        # the arc of a point is the one of the point after it.
        arcs = [p[2] for p in pts]
        if rev:
            pts = pts[::-1]
            arcs = [pts[(k + 1) % len(pts)][2] for k in range(len(pts))]
        lines = []
        for (u, v, _), c in zip(pts, arcs):
            line = "point %s %s" % (fmt(u), fmt(v))
            if c is not None:
                line += " arc %s %s %s" % (fmt(c[0]), fmt(c[1]),
                                           "cw" if rev else "ccw")
            lines.append(line)
        # The written points are rounded, so the outline is dropped when
        # the ends of an arc sit at different radii.
        for k, line in enumerate(lines):
            w = line.split()
            if len(w) == 7:
                nx = lines[(k + 1) % len(lines)].split()
                r0 = math.hypot(float(w[1]) - float(w[4]),
                                float(w[2]) - float(w[5]))
                r1 = math.hypot(float(nx[1]) - float(w[4]),
                                float(nx[2]) - float(w[5]))
                if abs(r0 - r1) > 1e-7:
                    return None
        return lines

    def line_clear(self, i, a, foot, depth):
        # Whether the cut through foot, a (U, V) box, misses every other
        # body.
        m = self.model
        lo, hi = (-1e9, 1e9) if depth is None else \
            (depth[0] - depth[1] / 2, depth[0] + depth[1] / 2)
        au = 1 if a == 0 else 0
        av = 1 if a == 2 else 2
        p, r = [0.0] * 3, [0.0] * 3
        p[au], p[av], p[a] = foot[0], foot[1], lo
        r[au], r[av], r[a] = foot[2], foot[3], hi
        box = (tuple(p), tuple(r))
        for j, b in enumerate(m.bodies):
            if j != i and box_hit(b["safe"], box, 0.05):
                return False
        return True


# The generator of prism parts.

def ddmin(units, test, budget):
    # Shrinks the list of units while test(units) stays true.  Returns
    # the smallest list found and the number of runs used.
    runs = 0
    n = 2
    while len(units) >= 2 and runs < budget:
        chunk = max(1, len(units) // n)
        smaller = False
        for i in range(0, len(units), chunk):
            cand = units[:i] + units[i + chunk:]
            runs += 1
            if cand and test(cand):
                units = cand
                n = max(n - 1, 2)
                smaller = True
                break
            if runs >= budget:
                break
        if not smaller:
            if chunk == 1:
                break
            n = min(len(units), n * 2)
    return units, runs


def minimise(data, test, budget=MIN_RUNS):
    # Line-level, then token-level delta debugging of data (bytes).
    lines = data.split(b"\n")
    lines, used = ddmin(lines, lambda u: test(b"\n".join(u)), budget)
    toks = [(i, j) for i, ln in enumerate(lines)
            for j, _ in enumerate(ln.split(b" "))]
    words = [ln.split(b" ") for ln in lines]

    def build(keep):
        ks = set(keep)
        out = []
        for i, ws in enumerate(words):
            out.append(b" ".join(w for j, w in enumerate(ws) if (i, j) in ks))
        return b"\n".join(out)

    toks, used2 = ddmin(toks, lambda u: test(build(u)), budget - used)
    return build(toks), used + used2


# The fuzzer.

class Fuzzer:
    def __init__(self, bindir, work, seed, occt=True):
        self.bin = os.path.abspath(bindir)
        self.work = os.path.abspath(work)
        self.rng = random.Random(seed)
        self.seed = seed
        self.n = 0
        self.use_occt = occt
        self.version = self.bin_version()
        self.case = os.path.join(self.work, "case")
        self.fdir = os.path.join(self.work, "findings")
        self.qdir = os.path.join(self.work, "occt")
        for d in (self.case, self.fdir, self.qdir):
            os.makedirs(d, exist_ok=True)
        self.log = open(os.path.join(self.work, "fuzz.log"), "a")
        self.seen = self.load_seen()
        self.occt = None
        self.pending = []
        self.pool = []
        self.corpus = self.load_corpus()

    def bin_version(self):
        h = hashlib.sha1()
        for name in sorted(os.listdir(self.bin)):
            p = os.path.join(self.bin, name)
            if os.path.isfile(p) and os.access(p, os.X_OK):
                with open(p, "rb") as f:
                    h.update(f.read())
        return h.hexdigest()[:8]

    def prog(self, name):
        return os.path.join(self.bin, name)

    def load_seen(self):
        seen = set()
        for d in sorted(os.listdir(self.fdir)):
            p = os.path.join(self.fdir, d, "sig")
            if os.path.exists(p):
                with open(p) as f:
                    seen.add(f.read().strip())
        return seen

    def load_corpus(self):
        corpus = []
        for name in sorted(os.listdir(HERE)):
            ext = os.path.splitext(name)[1]
            if ext in (".sm", ".fold", ".txt", ".params", ".expected",
                       ".py"):
                with open(os.path.join(HERE, name), "rb") as f:
                    data = f.read()
                if ext == ".txt":
                    data = re.sub(rb"\S+\.fold", lambda m: b"a.fold"
                                  if b"bad" not in m.group(0)
                                  else m.group(0).split(b"/")[-1], data)
                corpus.append((ext, data))
        # Sources for the checkers of headers and text.
        src = os.path.join(os.path.dirname(HERE), "src")
        if os.path.isdir(src):
            for name in sorted(os.listdir(src))[:12]:
                if name.endswith((".c", ".h")):
                    with open(os.path.join(src, name), "rb") as f:
                        corpus.append((name[-2:], f.read()))
        # Folds of the test parts, for the programs that read folds.
        tmp = os.path.join(self.work, "seed")
        os.makedirs(tmp, exist_ok=True)
        for name in sorted(os.listdir(HERE)):
            base, ext = os.path.splitext(name)
            if ext != ".sm" or base.startswith("bad_"):
                continue
            for aux in os.listdir(HERE):
                if aux.endswith(".params"):
                    shutil.copy(os.path.join(HERE, aux), tmp)
            shutil.copy(os.path.join(HERE, name), tmp)
            out = os.path.join(tmp, base + ".fold")
            r = run([self.prog("smfold"), os.path.join(tmp, name)],
                    out=out)
            if r.verdict == "OK":
                with open(out, "rb") as f:
                    corpus.append((".fold", f.read()))
        return corpus

    def keep_corpus(self, ext, data):
        self.corpus.append((ext, data))
        if len(self.corpus) > 600:
            del self.corpus[self.rng.randrange(40, len(self.corpus))]

    # The log and the findings.

    def note(self, mode, prog, verdict, detail, seed):
        stamp = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        line = "%s v=%s n=%d s=%d m=%s p=%s %s %s" % (
            stamp, self.version, self.n, seed, mode, prog, verdict,
            " ".join(str(detail).split()))
        self.log.write(line.rstrip() + "\n")
        self.log.flush()
        if self.log.tell() > LOG_MAX:
            path = os.path.join(self.work, "fuzz.log")
            self.log.close()
            os.replace(path, path + ".1")
            self.log = open(path, "a")
        # New findings and errors of the fuzzer itself also go to
        # findings.log, one line each, for a watcher; it never rotates.
        if verdict.startswith("NEW ") or verdict == "ERROR":
            with open(os.path.join(self.work, "findings.log"), "a") as f:
                f.write(line.rstrip() + "\n")
                f.flush()
                os.fsync(f.fileno())

    def finding(self, mode, seed, bad, files, main, check):
        # Saves a new finding with its input; dedupes it by signature.
        # check(dir, data) reruns the case on data and returns its Bad,
        # or None.
        sig = bad.sig
        if sig in self.seen:
            self.note(mode, bad.prog, "DUP " + bad.verdict, bad.detail, seed)
            return
        self.seen.add(sig)
        num = len(os.listdir(self.fdir)) + 1
        d = os.path.join(self.fdir, "%04d-%s-%s" % (num, bad.verdict.lower(),
                                                    bad.prog))
        os.makedirs(d, exist_ok=True)
        for name, data in files.items():
            with open(os.path.join(d, name), "wb") as f:
                f.write(data)
        with open(os.path.join(d, "sig"), "w") as f:
            f.write(sig + "\n")
        mind = os.path.join(d, "min")
        small, runs, mbad = self.shrink(files, main, check, sig, mind)
        with open(os.path.join(d, "README"), "w") as f:
            f.write("seed %d case %d mode %s binaries %s\n" % (
                seed, self.n, mode, self.version))
            f.write("signature: %s\n%s %s: %s\n" % (sig, bad.verdict,
                                                   bad.prog, bad.detail))
            f.write("command: %s\n" % " ".join(self.show(bad.argv)))
            f.write("pipeline: %s\n" % PIPES.get(mode, "the command"))
            if mbad is not None:
                f.write("minimised in %d runs to min/%s (%d -> %d bytes)\n"
                        "min command: %s\n" % (
                            runs, main, len(files[main]), len(small),
                            " ".join(self.show(mbad.argv))))
            f.write("\nstderr:\n%s\n" % bad.err[-6000:])
        self.note(mode, bad.prog, "NEW " + bad.verdict, bad.detail + " " + d,
                  seed)

    def show(self, argv):
        out = []
        for a in argv:
            if a.startswith(self.bin):
                a = os.path.basename(a)
            for d in (self.case, os.path.join(self.work, "min")):
                a = a.replace(d + "/", "")
            out.append(a)
        return out

    def shrink(self, files, main, check, sig, mind):
        os.makedirs(mind, exist_ok=True)
        tmp = os.path.join(self.work, "min")
        last = [None]

        stop = time.time() + MIN_SECONDS

        def test(data):
            if time.time() > stop:
                return False
            if os.path.isdir(tmp):
                shutil.rmtree(tmp)
            os.makedirs(tmp)
            for name, d in files.items():
                if name != main:
                    with open(os.path.join(tmp, name), "wb") as f:
                        f.write(d)
            b = check(tmp, data)
            if b is not None and b.sig == sig:
                last[0] = b
                return True
            return False

        try:
            if not test(files[main]):
                return files[main], 1, None
            small, runs = minimise(files[main], test)
            test(small)
        except Exception as e:
            self.note("min", "fuzz", "ERROR", repr(e), 0)
            return files[main], 0, None
        for name, d in files.items():
            with open(os.path.join(mind, name), "wb") as f:
                f.write(small if name == main else d)
        return small, runs, last[0]

    # Generated sheet metal parts.

    def sm_check(self, d, data, occt=False):
        # Folds the part in data and runs every writer.  Returns a Bad,
        # or None; raises Refused if smfold rejects the part.
        sm = os.path.join(d, "gen.sm")
        with open(sm, "wb") as f:
            f.write(data)
        model = SmModel()
        try:
            model.feed_text(data.decode("latin-1"))
        except Exception as e:
            raise Refused("model: %r" % e)
        fold = os.path.join(d, "gen.fold")
        try:
            argv = [self.prog("smfold"), sm]
            r = run(argv, cwd=d, out=fold)
            if r.verdict == "FAIL":
                raise Refused(r.detail)
            must(r, argv, True)
            outs = {}
            for prog, args, ext in [
                    ("measure", [], "measure"), ("tostep", [], "step"),
                    ("smflat", ["-f", "dxf"], "dxf")]:
                out = os.path.join(d, "gen." + ext)
                argv = [self.prog(prog)] + args + [fold]
                must(run(argv, cwd=d, out=out), argv, True)
                outs[ext] = out
            with open(outs["measure"], "rb") as f:
                ms = parse_measure(f.read())
            argv = [self.prog("measure"), fold]
            if len(ms) != 1:
                raise Bad("MISMATCH", "measure", "MISMATCH measure parts",
                          "%d parts" % len(ms), argv)
            bad = check_sheet(model, ms[0])
            if bad:
                raise Bad("MISMATCH", "measure", "MISMATCH measure " + bad[0],
                          bad[1], argv)
            if occt:
                self.occt_judge(outs["step"], sm, ms[0], model, argv)
        except Bad as b:
            return b
        self.last = (fold, outs, ms[0], model)
        return None

    def gen_sm(self, seed):
        text = SheetGen(random.Random(seed)).gen()
        data = text.encode()
        d = self.fresh()
        try:
            bad = self.sm_check(d, data)
        except Refused as e:
            self.note("sm", "smfold", "REJECT", str(e)[:150], seed)
            self.keep_corpus(".sm", data)
            return
        if bad:
            self.finding("sm", seed, bad, {"gen.sm": data}, "gen.sm",
                         lambda dd, x: self.retry(self.sm_check, dd, x))
            return
        fold, outs, m, model = self.last
        self.queue(seed, "sm", outs["step"], data, m, model)
        self.note("sm", "all", "PASS", "flanges=%d holes=%d V=%.3f" % (
            len(model.bends), len(model.holes), m["volume"]), seed)
        with open(fold, "rb") as f:
            fd = f.read()
        self.keep_corpus(".sm", data)
        self.keep_corpus(".fold", fd)
        self.pool.append((fd, m))
        if len(self.pool) > 50:
            del self.pool[0]

    def retry(self, fn, d, data, **kw):
        try:
            return fn(d, data, **kw)
        except Refused:
            return None

    def fresh(self):
        if os.path.isdir(self.case):
            shutil.rmtree(self.case)
        os.makedirs(self.case)
        return self.case

    def occt_start(self):
        # The answers come on a pipe of their own, apart from the OCCT
        # trace on stdout.
        home = os.path.join(self.work, "fchome")
        os.makedirs(home, exist_ok=True)
        rd, wr = os.pipe()
        env = dict(os.environ, FREECAD_USER_HOME=home, HOME=home,
                   QT_QPA_PLATFORM="offscreen", OCCTWORKER_FD=str(wr))
        self.occt = subprocess.Popen(
            [FREECADCMD, os.path.join(HERE, "occtworker.py")],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, env=env, cwd=home, pass_fds=(wr,))
        os.close(wr)
        self.occt_fd = rd
        self.occt_buf = b""

    def occt_stop(self):
        if self.occt is not None:
            try:
                self.occt.stdin.close()
                self.occt.wait(timeout=30)
            except Exception:
                self.occt.kill()
                self.occt.wait()
            os.close(self.occt_fd)
            self.occt = None

    def occt_ask(self, step, spec):
        # One request; a worker that takes longer than OCCT_TIMEOUT is
        # killed, and the answer holds "timeout".
        if self.occt is None or self.occt.poll() is not None:
            self.occt_stop()
            self.occt_start()
        req = json.dumps({"step": step, "spec": spec}) + "\n"
        try:
            self.occt.stdin.write(req.encode())
            self.occt.stdin.flush()
        except OSError:
            self.occt_stop()
            return {"error": "the OCCT worker exited"}
        end = time.time() + OCCT_TIMEOUT
        while b"\n" not in self.occt_buf:
            left = end - time.time()
            ready = select.select([self.occt_fd], [], [], max(left, 0))[0]
            if not ready:
                self.occt.kill()
                self.occt_stop()
                return {"timeout": OCCT_TIMEOUT}
            chunk = os.read(self.occt_fd, 65536)
            if not chunk:
                self.occt_stop()
                return {"error": "the OCCT worker exited"}
            self.occt_buf += chunk
        line, self.occt_buf = self.occt_buf.split(b"\n", 1)
        return json.loads(line.decode()[2:])

    def occt_judge(self, step, spec, m, model, argv):
        # Raises Bad for an OCCT disagreement.
        r = self.occt_ask(step, spec)
        prog = "tostep"

        def bad(name, detail):
            raise Bad("OCCT", prog, "OCCT %s" % name, detail,
                      ["occtworker", step, spec], json.dumps(r))

        if "timeout" in r:
            raise Refused("the OCCT worker timed out")
        if "error" in r:
            bad("read", r["error"])
        if r.get("solids") != 1:
            bad("solids", "%s solids" % r.get("solids"))
        if not r["valid"] or not r["closed"]:
            bad("invalid", "valid %s closed %s" % (r["valid"], r["closed"]))
        if not close(r["volume"], m["volume"], 1e-6 * m["volume"] + 2e-4):
            bad("volume", "measure %.4f, OCCT %.6f" % (m["volume"],
                                                      r["volume"]))
        if not close(r["area"], m["area"], 1e-6 * m["area"] + 2e-4):
            bad("area", "measure %.4f, OCCT %.6f" % (m["area"], r["area"]))
        if r["bop"] != "ok":
            bad("bop", "BOP check: " + r["bop"][:160])
        ref = r.get("ref", {})
        if "error" in ref:
            prog = "occtworker"
            bad("reference", ref["error"])
        if "symdiff" in ref:
            v = r["volume"]
            if not ref.get("valid"):
                prog = "occtworker"
                bad("reference invalid", "the reference solid is invalid")
            sd = ref["symdiff"]
            if sd > 1e-6 * v + 1e-3 or sd < -1e-4 * v - 1e-3:
                bad("symdiff", "symmetric difference %.6f of volume %.4f, "
                    "reference %.4f" % (ref["symdiff"], v, ref["volume"]))
            if model is not None:
                want = model.t * model.flat_area() + model.bend_sum()
                if not close(ref["volume"], want, 1e-6 * v + 1e-3):
                    prog = "fuzz"
                    bad("identity", "reference volume %.6f, identity "
                        "%.6f" % (ref["volume"], want))
        return r

    def queue(self, seed, mode, step, data, m, model):
        if not self.use_occt:
            return
        ext = ".sm"
        keep = os.path.join(self.qdir, "%d" % seed)
        shutil.copy(step, keep + ".step")
        with open(keep + ext, "wb") as f:
            f.write(data)
        self.pending.append((seed, mode, keep, ext, m, model))

    def occt_batch(self):
        for seed, mode, keep, ext, m, model in self.pending:
            step, spec = keep + ".step", keep + ext
            try:
                r = self.occt_judge(step, spec, m, model, [])
            except Refused as e:
                self.note(mode, "occt", "SKIP", str(e), seed)
            except Bad as b:
                with open(spec, "rb") as f:
                    data = f.read()
                fn = self.sm_check
                self.finding(mode, seed, b, {"gen" + ext: data},
                             "gen" + ext,
                             lambda dd, x, fn=fn: self.retry(
                                 fn, dd, x, occt=True))
            else:
                ref = r.get("ref", {})
                extra = "symdiff=%.2e" % ref["symdiff"] if "symdiff" in ref \
                    else "ref=" + ref.get("skip", "none")
                self.note(mode, "occt", "PASS", "V=%.3f %s" % (r["volume"],
                                                               extra), seed)
            for e in (".step", ext):
                if os.path.exists(keep + e):
                    os.remove(keep + e)
        self.pending = []

    # Assemblies.

    def asm_check(self, d, data, expect=None):
        asm = os.path.join(d, "asm.txt")
        with open(asm, "wb") as f:
            f.write(data)
        fold = os.path.join(d, "asm.fold")
        try:
            argv = [self.prog("place"), asm]
            r = run(argv, cwd=d, out=fold)
            if r.verdict == "FAIL" and expect is None:
                raise Refused(r.detail)
            must(r, argv, expect is not None)
            outs = {}
            for prog, args, ext in [("measure", [], "measure"),
                                    ("interfere", [], "interfere"),
                                    ("tostep", [], "step")]:
                out = os.path.join(d, "asm." + ext)
                argv = [self.prog(prog)] + args + [fold]
                must(run(argv, cwd=d, out=out), argv, expect is not None)
                outs[ext] = out
            if expect is not None:
                self.asm_judge(outs, fold, expect)
        except Bad as b:
            return b
        return None

    def asm_judge(self, outs, fold, expect):
        with open(outs["measure"], "rb") as f:
            ms = parse_measure(f.read())
        with open(outs["interfere"], "rb") as f:
            pairs = f.read().decode("latin-1").split("\n")
        pairs = [p for p in pairs if p.strip()]
        argv = [self.prog("measure"), fold]
        if len(ms) != len(expect["parts"]):
            raise Bad("MISMATCH", "place", "MISMATCH place parts",
                      "%d parts, expected %d" % (len(ms),
                                                 len(expect["parts"])), argv)
        for m, (name, orig, box) in zip(ms, expect["parts"]):
            if m["name"] != name:
                raise Bad("MISMATCH", "place", "MISMATCH place name",
                          "part %s, expected %s" % (m["name"], name), argv)
            for key in ("volume", "area"):
                if not close(m[key], orig[key], 2e-4 + 1e-7 * orig[key]):
                    raise Bad("MISMATCH", "place", "MISMATCH place " + key,
                              "%s %s %.4f, alone %.4f" % (name, key, m[key],
                                                          orig[key]), argv)
            if box is not None and not all(close(a, b, 3e-4) for a, b in
                                           zip(m["bbox"], box)):
                raise Bad("MISMATCH", "place", "MISMATCH place bbox",
                          "%s bbox %s, expected %s" % (
                              name, m["bbox"], [round(x, 4) for x in box]),
                          argv)
        argv = [self.prog("interfere"), fold]
        if expect["mode"] == "apart" and pairs:
            raise Bad("MISMATCH", "interfere", "MISMATCH interfere apart",
                      "parts far apart collide: " + pairs[0], argv)
        if expect["mode"] == "dup":
            a, b = expect["parts"][0][0], expect["parts"][1][0]
            if not any(p.split()[:2] == [a, b] for p in pairs):
                raise Bad("MISMATCH", "interfere", "MISMATCH interfere dup",
                          "two copies at one place do not collide", argv)

    def gen_asm(self, seed):
        rng = random.Random(seed)
        if not self.pool:
            return
        d = self.fresh()
        mode = rng.choice(["apart", "apart", "dup"])
        k = 2 if mode == "dup" else rng.randint(1, 3)
        picks = [rng.choice(self.pool) for _ in range(k)]
        if mode == "dup":
            picks[1] = picks[0]
        lines, parts = [], []
        files = {}
        cursor = 0.0
        same = None
        for i, (fd, m) in enumerate(picks):
            fname = "p%d.fold" % (i if mode != "dup" else 0)
            files[fname] = fd
            with open(os.path.join(d, fname), "wb") as f:
                f.write(fd)
            rots = []
            square = True
            for _ in range(rng.randint(0, 3)):
                ax = rng.choice("xyz")
                if rng.random() < 0.8:
                    deg = rng.choice([90, 180, 270, -90])
                else:
                    deg = q(rng.uniform(-180, 180))
                    square = False
                rots.append((ax, deg))
            if mode == "dup" and same is not None:
                rots = same[0]
                square = same[2]
            rig = IDENT
            for ax, deg in rots:
                rig = rig_mul(rig_rot((0, 0, 0), axis_vec("xyz".index(ax)),
                                      deg), rig)
            b = m["bbox"]
            corners = [rig_pt(rig, (b[3 * (c & 1)], b[1 + 3 * (c >> 1 & 1)],
                                    b[2 + 3 * (c >> 2 & 1)]))
                       for c in range(8)]
            lo, hi = box_of(corners)
            if mode == "dup" and same is not None:
                at = same[1]
            else:
                at = (q(cursor - lo[0]), q(rng.uniform(-50, 50)),
                      q(rng.uniform(-50, 50)))
                same = (rots, at, square)
            cursor += hi[0] - lo[0] + 5 + rng.uniform(0, 20)
            box = list(vadd(lo, at)) + list(vadd(hi, at)) if square else None
            name = "a%d" % i
            words = [name, fname]
            for ax, deg in rots:
                words += ["rot", ax, fmt(deg)]
            if rng.random() < 0.5 and rots:
                words[2:2] = ["at"] + [fmt(x) for x in at]
            else:
                words += ["at"] + [fmt(x) for x in at]
            lines.append(" ".join(words))
            parts.append((name, m, box))
        data = ("\n".join(lines) + "\n").encode()
        files["asm.txt"] = data
        expect = {"mode": mode, "parts": parts}
        bad = self.asm_check(d, data, expect)
        if bad:
            self.finding("asm", seed, bad, files, "asm.txt",
                         lambda dd, x: self.retry(self.asm_check, dd, x,
                                                  expect=expect))
            return
        self.keep_corpus(".txt", data)
        self.note("asm", "all", "PASS", "%s parts=%d" % (mode, k), seed)

    # Damaged inputs.

    def mutate(self, rng, data):
        text = data.decode("latin-1")
        lines = text.split("\n")
        for _ in range(rng.randint(1, 4)):
            op = rng.randrange(13)
            i = rng.randrange(len(lines)) if lines else 0
            if op == 0 and lines:
                del lines[i]
            elif op == 1 and lines:
                lines.insert(i, lines[rng.randrange(len(lines))])
            elif op == 2 and len(lines) > 1:
                j = rng.randrange(len(lines))
                lines[i], lines[j] = lines[j], lines[i]
            elif op in (3, 4, 5, 6) and lines:
                w = lines[i].split(" ")
                j = rng.randrange(len(w))
                if op == 3:
                    w[j] = rng.choice(NUMBERS)
                elif op == 4:
                    w[j] = rng.choice(WORDS)
                elif op == 5:
                    del w[j]
                else:
                    w[j] = self.tweak(rng, w[j])
                lines[i] = " ".join(w)
            elif op == 7:
                lines = lines[:i]
            elif op == 8:
                lines.insert(i, " ".join(rng.choice(WORDS + NUMBERS)
                                         for _ in range(rng.randint(1, 30))))
            elif op == 9:
                lines.insert(i, rng.choice(["x", "1", "(", "#"]) *
                             rng.choice([1000, 5000, 70000]))
            elif op == 10 and lines:
                s = lines[i]
                if s:
                    j = rng.randrange(len(s))
                    lines[i] = s[:j] + chr(rng.randrange(256)) + s[j + 1:]
            elif op == 11 and lines:
                lines[i] = lines[i] + " " + rng.choice(WORDS + NUMBERS)
            elif op == 12:
                ext, other = rng.choice(self.corpus)
                ol = other.decode("latin-1").split("\n")
                j = rng.randrange(len(ol))
                lines[i:i] = ol[j:j + rng.randint(1, 5)]
        sep = "\r\n" if rng.random() < 0.05 else "\n"
        out = sep.join(lines)
        if rng.random() < 0.05:
            out = out.rstrip("\n")
        return out.encode("latin-1")

    def tweak(self, rng, word):
        # Nudges a number by a tiny change, a change of sign or scale, or
        # a jump to a value nearby.
        try:
            x = float(word)
        except ValueError:
            return word + rng.choice(["", "1", "x", "*0", "/0", "e5"])
        # Words such as "nan" and "inf" parse as floats but have no
        # nearby value, so they get the same damage as other words.
        if not math.isfinite(x):
            return word + rng.choice(["", "1", "x", "*0", "/0", "e5"])
        c = rng.randrange(6)
        if c == 0:
            return repr(-x)
        if c == 1:
            return repr(x * rng.choice([0, 1e-9, 1e9, 1000, 0.001]))
        if c == 2:
            return repr(x + rng.choice([1e-9, -1e-9, 1e-7, -1e-7, 1e-6]))
        if c == 3:
            return repr(x + rng.uniform(-10, 10))
        if c == 4:
            return repr(round(x))
        return repr(x * 2)

    def mut_files(self, d, data, ext):
        # Writes the fold files of a damaged assembly and the include
        # files of a damaged part.
        files = {}
        folds = [x for e, x in self.corpus if e == ".fold"]
        params = [x for e, x in self.corpus if e == ".params"]
        for m in re.finditer(rb"(?:^|\s)([\w.-]+\.fold)(?=\s|$)", data):
            name = m.group(1).decode("latin-1")
            if name in files or name.startswith(".") or not folds:
                continue
            p = os.path.join(HERE, name)
            if os.path.isfile(p):
                with open(p, "rb") as f:
                    files[name] = f.read()
            else:
                files[name] = self.rng.choice(folds)
        for m in re.finditer(rb"include\s+([\w.-]+)", data):
            name = m.group(1).decode("latin-1")
            if name not in files and params and not name.startswith("."):
                files[name] = self.rng.choice(params)
        # A mutated name can exceed the file system's limit of 255 bytes;
        # such a file can't exist, so the program sees it as missing.
        for name, x in list(files.items()):
            if len(name.encode("latin-1")) > 255:
                del files[name]
                continue
            with open(os.path.join(d, name), "wb") as f:
                f.write(x)
        return files

    def mut_check(self, d, data, ext, occt=False, files=None):
        # Runs the programs for one damaged input.  Returns a Bad or None.
        src = os.path.join(d, "mut" + ext)
        with open(src, "wb") as f:
            f.write(data)
        fold = src if ext == ".fold" else os.path.join(d, "mut.fold")
        try:
            for name in CHECKERS[1:]:
                if os.path.exists(self.prog(name)):
                    argv = [self.prog(name), src]
                    r = run(argv, cwd=d, stdout_msg=True)
                    if r.verdict not in ("OK", "FAIL"):
                        must(r, argv, False)
            if ext == ".expected":
                other = os.path.join(d, "other.expected")
                for argv in ([self.prog("numdiff")] + TOL + [src, other],
                             [self.prog("numdiff")] + TOL + [other, src],
                             [self.prog("numdiff"), src, src]):
                    r = run(argv, cwd=d, stdout_msg=True)
                    if r.verdict not in ("OK", "FAIL"):
                        must(r, argv, False)
                return None
            if ext in (".py", ".c", ".sh", ".h"):
                return None
            if ext == ".params":
                for name in ("channel.params", "inc.params"):
                    with open(os.path.join(d, name), "wb") as f:
                        f.write(data)
                with open(os.path.join(d, "inc.sm"), "wb") as f:
                    f.write(b"include inc.params\nsheet T R K\nplate p 0 0 "
                            b"W L origin 0 0 0 axes +x +y\nflange f p +x 90 "
                            b"H\n")
                src = os.path.join(d, "inc.sm")
                ext = ".sm"
            prod = {".sm": "smfold", ".txt": "place"}
            if ext in prod:
                argv = [self.prog(prod[ext])]
                if ext != ".txt" and self.rng.random() < 0.2:
                    argv += ["-M"]
                argv += [src]
                r = run(argv, cwd=d, out=fold)
                if r.verdict == "FAIL":
                    return None
                must(r, argv, False)
                if "-M" in argv:
                    return None
            for prog, args in [("measure", []), ("tostep", []),
                               ("smflat", ["-f", "dxf"]),
                               ("interfere", [])]:
                argv = [self.prog(prog)] + args + [fold]
                out = os.path.join(d, "out." + ("step" if prog == "tostep"
                                                else prog))
                r = run(argv, cwd=d, out=out)
                if r.verdict not in ("OK", "FAIL"):
                    must(r, argv, False)
                if prog == "tostep":
                    step = out if r.verdict == "OK" else None
                if prog == "measure":
                    meas = r.out if r.verdict == "OK" else None
            if ext == ".fold":
                asm = os.path.join(d, "wrap.txt")
                with open(asm, "wb") as f:
                    f.write(b"a mut.fold\nb mut.fold rot z 90 at 1 2 3\n")
                argv = [self.prog("place"), asm]
                r = run(argv, cwd=d)
                if r.verdict not in ("OK", "FAIL"):
                    must(r, argv, False)
            if occt and step and meas and ext == ".sm":
                self.mut_occt(step, meas, src)
        except Refused:
            return None
        except Bad as b:
            return b
        return None

    def mut_occt(self, step, meas, src):
        # An accepted damaged part must still give a valid solid.
        try:
            ms = parse_measure(meas)
        except (ValueError, IndexError):
            return
        if len(ms) != 1 or "volume" not in ms[0]:
            return
        r = self.occt_ask(step, None)
        bad = ""
        if "timeout" in r:
            return
        if "error" in r:
            bad = "read"
        elif r.get("solids") != 1:
            bad = "solids"
        elif not r["valid"] or not r["closed"]:
            bad = "invalid"
        elif not close(r["volume"], ms[0]["volume"],
                       1e-6 * abs(ms[0]["volume"]) + 2e-4):
            bad = "volume"
        elif r["bop"] != "ok":
            bad = "bop"
        if bad:
            raise Bad("INVALID", "smfold", "INVALID %s %s" % (
                          os.path.splitext(src)[1], bad),
                      "accepted damaged input: %s %s" % (bad, json.dumps(
                          {k: r.get(k) for k in ("solids", "valid", "volume",
                                                 "bop")})[:200]),
                      ["occtworker", step], json.dumps(r))

    def gen_mut(self, seed):
        rng = random.Random(seed)
        ext, data = rng.choice(self.corpus)
        if ext == ".py":
            ext = rng.choice([".py", ".c", ".sh"])
        data = self.mutate(rng, data)
        d = self.fresh()
        files = self.mut_files(d, data, ext)
        if ext == ".expected":
            other = self.mutate(rng, rng.choice(
                [x for e, x in self.corpus if e == ".expected"]))
            files["other.expected"] = other
            with open(os.path.join(d, "other.expected"), "wb") as f:
                f.write(other)
        occt = self.use_occt and ext == ".sm" and rng.random() < MUT_OCCT
        main = "mut" + ext
        files[main] = data
        bad = self.mut_check(d, data, ext, occt)
        if bad:
            self.finding("mut", seed, bad, files, main,
                         lambda dd, x: self.mut_check(dd, x, ext, occt))
            return
        self.note("mut", "all", "PASS", ext, seed)

    # Arguments.

    def gen_args(self, seed):
        rng = random.Random(seed)
        d = self.fresh()
        folds = [x for e, x in self.corpus if e == ".fold"]
        files = {"a.fold": rng.choice(folds), "empty": b"",
                 "bin": bytes(rng.randrange(256) for _ in range(300))}
        for e in (".sm", ".txt", ".params", ".expected"):
            pick = [x for k, x in self.corpus if k == e]
            if pick:
                files["a" + e] = rng.choice(pick)
        for name, x in files.items():
            with open(os.path.join(d, name), "wb") as f:
                f.write(x)
        os.makedirs(os.path.join(d, "dir"), exist_ok=True)
        progs = sorted(p for p in os.listdir(self.bin)
                       if os.access(self.prog(p), os.X_OK) and
                       not p.startswith("test_"))
        name = rng.choice(progs)
        pool = ["-D", "-M", "-f", "-v", "-t", "-a", "-", "--", "-Q", "-DX=1",
                "dxf", "svg", "x", "X=1", "T=0", "W=1/0", "K=-1",
                "volume=rel:1e-3", "a=abs:x", "=", "rel:", "1e400", "-90",
                "nan", "dir", "empty", "bin", "/nonexistent/x", "a" * 5000,
                "a.fold", "a.sm", "a.txt", "a.params", "a.expected",
                "params.d", "", "\x01"]
        argv = [self.prog(name)] + [rng.choice(pool) for _ in
                                    range(rng.randint(0, 6))]
        if rng.random() < 0.5:
            argv.append(rng.choice(["a.fold", "a.sm", "a.txt", "a.params",
                                    "a.expected"]))
        r = run(argv, cwd=d, stdout_msg=name in CHECKERS)
        if r.verdict in ("OK", "FAIL"):
            self.note("args", name, "PASS", r.verdict, seed)
            return
        bad = Bad(r.verdict, name, r.sig, r.detail, argv, r.err)
        files["argv"] = "\n".join(argv[1:]).encode("latin-1")

        def check(dd, x):
            for nm, data in files.items():
                with open(os.path.join(dd, nm), "wb") as f:
                    f.write(data)
            os.makedirs(os.path.join(dd, "dir"), exist_ok=True)
            av = [self.prog(name)] + x.decode("latin-1").split("\n") \
                if x else [self.prog(name)]
            rr = run(av, cwd=dd, stdout_msg=name in CHECKERS)
            if rr.verdict in ("OK", "FAIL"):
                return None
            return Bad(rr.verdict, name, rr.sig, rr.detail, av, rr.err)

        self.finding("args", seed, bad, files, "argv", check)

    # The main loop.

    MODES = ["sm"] * 9 + ["asm"] + ["mut"] * 8 + ["args"] * 2

    def one(self, mode, seed):
        {"sm": self.gen_sm, "asm": self.gen_asm,
         "mut": self.gen_mut, "args": self.gen_args}[mode](seed)

    def loop(self, cases=None):
        self.note("start", "fuzz", "START", "seed %d" % self.seed, self.seed)
        while cases is None or self.n < cases:
            while os.getloadavg()[0] > LOAD_PAUSE:
                self.note("pause", "fuzz", "PAUSE", "load %.1f" %
                          os.getloadavg()[0], 0)
                time.sleep(60)
            while shutil.disk_usage(self.work).free < DISK_MIN:
                self.note("pause", "fuzz", "PAUSE", "free disk %d MB" % (
                    shutil.disk_usage(self.work).free // 2 ** 20), 0)
                time.sleep(300)
            self.n += 1
            mode = self.rng.choice(self.MODES)
            seed = self.rng.randrange(2 ** 32)
            try:
                self.one(mode, seed)
            except Exception as e:
                where = " < ".join(
                    "%s:%d" % (f.name, f.lineno) for f in
                    traceback.extract_tb(e.__traceback__)[::-1][:4])
                self.note(mode, "fuzz", "ERROR", "%r at %s" % (e, where),
                          seed)
            if len(self.pending) >= OCCT_EVERY:
                self.occt_batch()
        self.occt_batch()
        self.occt_stop()


def main():
    args = sys.argv[1:]
    cases = None
    occt = True
    one = None
    while args and args[0].startswith("--"):
        if args[0] == "--cases":
            cases = int(args[1])
            args = args[2:]
        elif args[0] == "--no-occt":
            occt = False
            args = args[1:]
        elif args[0] == "--modes":
            Fuzzer.MODES = args[1].split(",")
            args = args[2:]
        elif args[0] == "--one":
            one = (args[1], int(args[2]))
            args = args[3:]
        else:
            sys.exit("fuzz.py: unknown option " + args[0])
    if len(args) < 2:
        sys.exit("usage: fuzz.py [--cases N] [--no-occt] [--one MODE SEED] "
                 "BIN WORK [SEED]")
    seed = int(args[2]) if len(args) > 2 else int(time.time())
    f = Fuzzer(args[0], args[1], seed, occt)
    if one:
        f.n = 1
        f.one(*one)
        f.occt_batch()
        f.occt_stop()
        return
    try:
        f.loop(cases)
    finally:
        f.occt_stop()


if __name__ == "__main__":
    main()
