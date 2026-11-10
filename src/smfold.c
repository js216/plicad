// SPDX-License-Identifier: GPL-3.0-only
// smfold.c -- Sheet metal part description to fold file
// Copyright (c) 2026 Jakob Kastelic

// Usage: smfold [-D NAME=EXPR]... part.sm > part.fold
//        smfold -M part.sm     prints the names of the included
//                              parameters used by part.sm
//
// Option -D pins a parameter to a value.
//
// Each line holds one statement, and # starts a comment.  An argument is
// one expression without spaces, or a (parenthesised) one.  Flat
// coordinates are x and y.  The sheet is z in [-t, 0], and its top face,
// z = 0, is the reference (outer) face.
//
//   include FILE          name = expr lines, FILE relative to this file
//   NAME = EXPR           a local definition
//   sheet T R K           thickness, inside bend radius, ANSI k-factor
//   plate NAME X0 Y0 X1 Y1 origin OX OY OZ axes U V
//       The root slab.  Flat x and y run along the 3D axes U and V (+x,
//       -z, ...), the top face faces U x V, and flat (0, 0) is at O.
//   flange NAME PARENT DIR ANGLE LEG [at C] [span A B] [zone A B]
//       A bend zone starts on the edge of PARENT in flat direction DIR
//       (+x -x +y -y), or on the line where that coordinate is C.  After
//       the zone is a straight leg LEG long.  ANGLE is in degrees, and a
//       positive one folds toward the top face.  The span sets the extent
//       of the flange along the edge, by default that of the parent.  The
//       zone sets the extent of the bend, by default the overlap of span
//       and parent.  The bend has the inside radius of the sheet.
//   cut U0 V0 U1 V1 along A [at W depth L]
//   hole D U V along A [at W depth L] [csink DC ANGLE near|far]
//        [pattern AX N P [AX N P] | pattern hex P within R]
//       A rectangle or a hole through every slab made so far that lies
//       across the 3D axis A (x, y or z).  U and V are the other two
//       axes in alphabetical order.  A depth L centred on W limits it.
//       A pattern repeats it N times, P apart, along the axis AX (with
//       its sign: -x runs toward smaller x).  A hex pattern puts holes
//       on a hexagonal grid of pitch P round (U, V), rows along U, where
//       they lie wholly inside the circle of radius R about (U, V).  A
//       countersink of diameter
//       DC and included angle ANGLE opens the hole at the top face of
//       the slab (near) or at the other face (far).
//   outline along A [at W depth L]
//   point U V [arc CU CV ccw|cw]
//       A hole of any outline through every slab across A, inside one
//       face of each, like a cut.  The points of the outline follow in
//       point statements.  From each point a line runs to the next one,
//       or an arc about (CU, CV), counterclockwise or clockwise in the
//       (U, V) plane.
//   trim along A [at W depth L]
//       An open curve of point statements, from one point on the
//       outline of a slab face to another, each at a corner or on a
//       straight edge.  It replaces the run of free outline edges
//       between them that cuts off the smaller piece.
//   round NAME R [at X Y Z]
//       Rounds the two free-end corners of flange NAME, or the one
//       corner of slab NAME at the 3D point (X, Y, Z): a round of a
//       convex corner, or a fillet of a concave one.  The point may lie
//       on either face of the sheet.
//
// Statements apply in order, like the features of a CAD model.

#include "contact.h"
#include "expr.h"
#include "fold.h"
#include "geom.h"
#include "util.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define EPS 1e-7
#define ON 1e-6 // Points closer than this to an edge lie on it, in mm.
#define DRIFT 1e-6 // How far a cut axis may drift across a slab, in mm.
#define NO_ANGLE 1e-9 // A smaller bend angle is none, in degrees.
#define WHOLE 1e-9 // How far a count may miss a whole number.
#define CORNER_AT 1e-4 // How far the point of a round may miss its corner.
#define HEX_RANGE 300.0 // The widest hex pattern, in pitches.
#define APART 1e-6 // Bodies closer than this touch, in mm.
#define SLICE (PI / 64) // Widest slice of a bend, to test contacts.

enum {
	FOOT_SIDES = 32, // Sides of the polygon round a hole, to test slabs.
	BOX_STEPS = 8, // Grid steps across a slab for its box.
	OUTLINE_STEPS = 64, // Points on each piece of an outline.
	DEF_LEN = 2048 // Room for the text of a definition.
};
// The words of the statements.
enum {
	PLATE_Y1 = 5,
	PLATE_ORIGIN,
	PLATE_OX,
	PLATE_OY,
	PLATE_OZ,
	PLATE_AXES,
	PLATE_U,
	PLATE_V,
	PLATE_WORDS
};
enum { FLANGE_ANGLE = 4, FLANGE_LEG, FLANGE_OPTIONS };
enum { HOLE_ALONG = 4, CUT_ALONG }; // Where "along A" starts.
enum { POINT_WORDS = 3, POINT_CU, POINT_CV, POINT_DIR, POINT_ARC_WORDS };
enum { ROUND_WORDS = 3, ROUND_X, ROUND_Y, ROUND_Z, ROUND_AT_WORDS };
enum {
	MAXARGS = 64,
	MAXREPEAT = 100000 // Most copies of one cut or hole.
};

struct body {
	char *name;
	int bend; // 0 for a slab, 1 for a bend zone.
	size_t parent; // Bend of a slab (NONE at the root); slab of a bend.
	double x0, y0, x1, y1; // Flat rectangle.
	struct rigid m; // Slab: flat to 3D.
	double qx, qy, mx, my, ba, ang, r; // Bend.
	int dir; // Flange direction: 0 +x, 1 -x, 2 +y, 3 -y.
	struct box box; // 3D box.
};

// The sheet: thickness, inside bend radius and k-factor.
struct sheet_dims {
	double t, r, k;
	bool set;
};

// A rectangle that paints cells with a body, or clears the cells of one.
struct op {
	double x0, y0, x1, y1;
	size_t body;
	int clear;
};

// A hole on a slab, and the file and line of its statement.
struct circ {
	size_t body;
	double x, y, r;
	int side; // Countersink at the top face (1), the bottom (-1), none (0).
	double rc, h;
	char *where;
};

// A corner rounding, and the file and line of its statement.
struct rnd {
	size_t body;
	double r;
	int at; // Whether the corner is the one at pt.
	struct vec pt;
	char *where;
};

// A cut or hole: its footprint in the U, V plane across axis a, an
// optional depth range along a, and for a hole its radius and centre.
struct through {
	int a;
	struct box foot; // U in x, V in y, the range along a in z.
	int hole;
	double rad, cu, cv;
	int side; // A countersink at the top or bottom face (1, -1), or 0.
	double rc, angle;
};

// A pattern: n[k] copies along axis[k], with pitch step[k] (signed), or
// if hex is above 0, the points of a hexagonal grid of pitch hex, with
// rows along U, whose holes lie inside radius rmax.
struct pattern {
	int axis[2];
	double step[2];
	long n[2];
	double hex, rmax;
};

// A corner of an outline in flat coordinates, with the arc to the next
// corner if it has one.
struct ocorner {
	double x, y;
	int arc, ccw;
	double cx, cy;
};

// An outline cut on slab body, and the file and line of its statement.
struct oline {
	size_t body;
	struct ocorner *c;
	size_t n;
	int trim; // An open curve that trims the outline of the face.
	char *where;
};

// The outline being read: its statement and its corners in (U, V).
struct open_outline {
	int open;
	struct through c;
	struct ocorner *uv;
	size_t n, cap;
	int trim;
	char *where;
};

// The part being made: its names, the statement being read, the bodies,
// paint ops, holes, rounds and outlines so far, the grid of cells and
// the part.
struct smf {
	struct names nm;
	const char *part_path;
	char *args[MAXARGS];
	size_t nargs;
	struct sheet_dims sheet;
	struct body *bodies;
	size_t nb, capb;
	struct op *ops;
	size_t nops, capops;
	struct circ *circs;
	size_t ncircs, capcircs;
	struct rnd *rnds;
	size_t nrnds, caprnds;
	struct oline *olines;
	size_t nolines, capolines;
	struct open_outline ol;
	double *xs, *ys;
	size_t nx, ny; // Number of coordinates.
	size_t *lab; // Body + 1, or 0 for empty.
	size_t *fid; // Face, or NONE for empty.
	size_t *vid; // Grid point to vertex.
	struct part out;
};

static double arg(struct smf *g, size_t i)
{
	if (i >= g->nargs) {
		expr_fail(&g->nm, "missing argument", NULL);
	}
	return eval(&g->nm, g->args[i]);
}

static int is(const struct smf *g, size_t i, const char *w)
{
	return i < g->nargs && strcmp(g->args[i], w) == 0;
}

static size_t find_slab(struct smf *g, const char *n)
{
	for (size_t i = 0; i < g->nb; i++) {
		if (!g->bodies[i].bend && strcmp(g->bodies[i].name, n) == 0) {
			return i;
		}
	}
	expr_fail(&g->nm, "no such plate or flange", n);
}

static void new_name(struct smf *g, const char *n)
{
	for (size_t i = 0; i < g->nb; i++) {
		if (strcmp(g->bodies[i].name, n) == 0) {
			expr_fail(&g->nm,
				  "a plate or flange has this name already", n);
		}
	}
}

// Parses an axis: "+x" gives axis 0 and sign +1.
static int axis_of(struct smf *g, const char *s, double *sign)
{
	*sign = 1;
	if (*s == '+' || *s == '-') {
		*sign = *s++ == '-' ? -1 : 1;
	}
	if (!s[0] || s[1] || !strchr("xyz", s[0])) {
		expr_fail(&g->nm,
			  "expected an axis: x, y or z, with an optional sign",
			  s);
	}
	return s[0] - 'x';
}

static void op(struct smf *g, double x0, double y0, double x1, double y1,
	       size_t body, int clear)
{
	struct op o = { fmin(x0, x1), fmin(y0, y1), fmax(x0, x1),
			fmax(y0, y1), body,	    clear };

	PUSH(g->ops, g->nops, g->capops, o);
}

// The fold map of a body; the parent slab of a bend is map 0.
static struct map body_map(const struct body *b, size_t parent)
{
	struct map m = { 0,	b->m,  NONE,  b->qx,  b->qy,
			 b->mx, b->my, b->ba, b->ang, b->r };

	m.bend = b->bend;
	if (b->bend) {
		m.parent = parent;
	}
	return m;
}

// Computes the 3D box of the last body; bends are sampled along the zone.
static void box_last(struct smf *g)
{
	struct body *b = &g->bodies[g->nb - 1];
	struct map maps[2];
	size_t face = 1;
	struct part tmp;

	maps[0] = body_map(b->bend ? &g->bodies[b->parent] : b, NONE);
	maps[1] = body_map(b, 0);
	memset(&tmp, 0, sizeof(tmp));
	tmp.t = g->sheet.t;
	tmp.place = rig_id();
	tmp.maps = maps;
	tmp.nmaps = 2;
	tmp.faces = &face;
	tmp.nfaces = 1;
	b->box = box_empty();
	for (int i = 0; i <= BOX_STEPS; i++) {
		for (int j = 0; j <= BOX_STEPS; j++) {
			for (int k = 0; k < 2; k++) {
				box_add(&b->box,
					fmap(&tmp, 0,
					     b->x0 + ((b->x1 - b->x0) * i /
						      (double)BOX_STEPS),
					     b->y0 + ((b->y1 - b->y0) * j /
						      (double)BOX_STEPS),
					     -g->sheet.t * k));
			}
		}
	}
}

static void add_body(struct smf *g, struct body b)
{
	double x0 = fmin(b.x0, b.x1);
	double x1 = fmax(b.x0, b.x1);
	double y0 = fmin(b.y0, b.y1);
	double y1 = fmax(b.y0, b.y1);

	b.x0 = x0;
	b.x1 = x1;
	b.y0 = y0;
	b.y1 = y1;
	PUSH(g->bodies, g->nb, g->capb, b);
	box_last(g);
	op(g, x0, y0, x1, y1, g->nb - 1, 0);
}

static struct body blank_body(const char *name)
{
	struct body b;

	memset(&b, 0, sizeof(b));
	b.name = xstrdup(name);
	b.parent = NONE;
	b.m = rig_id();
	return b;
}

static void st_sheet(struct smf *g)
{
	if (g->nb) {
		expr_fail(&g->nm, "sheet must come before plate", NULL);
	}
	if (g->nargs != 4) {
		expr_fail(&g->nm, "sheet T R K", NULL);
	}
	g->sheet.t = arg(g, 1);
	g->sheet.r = arg(g, 2);
	g->sheet.k = arg(g, 3);
	if (!(g->sheet.t > 0) || !(g->sheet.r >= 0) || !(g->sheet.k >= 0) ||
	    !(g->sheet.k <= 1)) {
		expr_fail(&g->nm, "need T > 0, R >= 0 and K in [0, 1]", NULL);
	}
	g->sheet.set = true;
}

// The frame of the root plate: flat x along u, flat y along v, at o.
struct plate_axes {
	struct vec o, u, v;
};

static struct rigid plate_map(struct smf *g, struct plate_axes ax)
{
	struct vec o = ax.o;
	struct vec u = ax.u;
	struct vec v = ax.v;
	struct vec n = cross(u, v);
	struct rigid m = rig_id();

	if (len(n) < MID) {
		expr_fail(&g->nm, "the plate axes must differ", NULL);
	}
	m.r[0][0] = u.x;
	m.r[1][0] = u.y;
	m.r[2][0] = u.z;
	m.r[0][1] = v.x;
	m.r[1][1] = v.y;
	m.r[2][1] = v.z;
	m.r[0][2] = n.x;
	m.r[1][2] = n.y;
	m.r[2][2] = n.z;
	m.t = o;
	return m;
}

static void st_plate(struct smf *g)
{
	struct plate_axes ax;
	struct body b;
	double su = 0;
	double sv = 0;
	int au = 0;
	int av = 0;

	if (!g->sheet.set) {
		expr_fail(&g->nm, "sheet must come before plate", NULL);
	}
	if (g->nb) {
		expr_fail(&g->nm, "there is only one plate (the root)", NULL);
	}
	if (g->nargs != PLATE_WORDS || !is(g, PLATE_ORIGIN, "origin") ||
	    !is(g, PLATE_AXES, "axes")) {
		expr_fail(&g->nm,
			  "plate NAME X0 Y0 X1 Y1 origin OX OY OZ axes U V",
			  NULL);
	}
	b = blank_body(g->args[1]);
	b.x0 = arg(g, 2);
	b.y0 = arg(g, 3);
	b.x1 = arg(g, 4);
	b.y1 = arg(g, PLATE_Y1);
	au = axis_of(g, g->args[PLATE_U], &su);
	av = axis_of(g, g->args[PLATE_V], &sv);
	ax.o = v3(arg(g, PLATE_OX), arg(g, PLATE_OY), arg(g, PLATE_OZ));
	ax.u = axis_vec(au, su);
	ax.v = axis_vec(av, sv);
	b.m = plate_map(g, ax);
	if (!(b.x1 - b.x0 > EPS) || !(b.y1 - b.y0 > EPS)) {
		expr_fail(&g->nm, "the plate must have X1 > X0 and Y1 > Y0",
			  NULL);
	}
	add_body(g, b);
}

// The values of a flange statement.  The edge line is at c on axis ax,
// the flange goes toward sign sg, and the zone and span run from a to b
// along the edge.
struct flange {
	size_t parent;
	int ax;
	double sg, ang, leg, r, c;
	double sa, sb, za, zb;
};

static void flange_options(struct smf *g, struct flange *f,
			   const struct body *p)
{
	bool zone_given = false;
	double pa = f->ax == 0 ? p->y0 : p->x0;
	double pb = f->ax == 0 ? p->y1 : p->x1;

	f->sa = pa;
	f->sb = pb;
	for (size_t i = FLANGE_OPTIONS; i < g->nargs; i++) {
		if (is(g, i, "at")) {
			f->c = arg(g, ++i);
		} else if (is(g, i, "span")) {
			f->sa = arg(g, i + 1);
			f->sb = arg(g, i + 2);
			i += 2;
		} else if (is(g, i, "zone")) {
			f->za = arg(g, i + 1);
			f->zb = arg(g, i + 2);
			zone_given = true;
			i += 2;
		} else {
			expr_fail(&g->nm, "unknown flange option", g->args[i]);
		}
	}
	if (!zone_given) {
		f->za = fmax(f->sa, pa);
		f->zb = fmin(f->sb, pb);
	}
}

static struct flange parse_flange(struct smf *g)
{
	struct flange f;
	const struct body *p = NULL;

	if (g->nargs < FLANGE_OPTIONS) {
		expr_fail(&g->nm, "flange NAME PARENT DIR ANGLE LEG ...", NULL);
	}
	new_name(g, g->args[1]);
	f.parent = find_slab(g, g->args[2]);
	p = &g->bodies[f.parent];
	f.ax = axis_of(g, g->args[3], &f.sg);
	if (f.ax > 1) {
		expr_fail(&g->nm,
			  "a flange direction is +x, -x, +y or -y (flat)",
			  NULL);
	}
	f.ang = arg(g, FLANGE_ANGLE);
	f.leg = arg(g, FLANGE_LEG);
	f.r = g->sheet.r;
	if (f.ax == 0) {
		f.c = f.sg > 0 ? p->x1 : p->x0;
	} else {
		f.c = f.sg > 0 ? p->y1 : p->y0;
	}
	flange_options(g, &f, p);
	if (!(fabs(f.ang) > NO_ANGLE) || !(fabs(f.ang) < HALF_TURN)) {
		expr_fail(&g->nm,
			  "the bend angle must be in (-180, 180) and not 0",
			  NULL);
	}
	if (!(f.leg > 0) || !(f.r > 0)) {
		expr_fail(&g->nm, "need LEG > 0 and a bend radius above 0",
			  NULL);
	}
	if (!(f.zb - f.za > EPS) || !(f.sb - f.sa > EPS)) {
		expr_fail(&g->nm, "the bend zone or the span is empty", NULL);
	}
	return f;
}

// A rectangle across an edge along axis ax, and along it.
struct strip {
	struct span across, along;
};

static void set_rect(struct body *b, int ax, struct strip st)
{
	double c0 = st.across.lo;
	double c1 = st.across.hi;
	double a = st.along.lo;
	double e = st.along.hi;

	b->x0 = ax == 0 ? c0 : a;
	b->x1 = ax == 0 ? c1 : e;
	b->y0 = ax == 0 ? a : c0;
	b->y1 = ax == 0 ? e : c1;
}

static void add_zone(struct smf *g, const struct flange *f, double ba)
{
	struct strip st;
	struct body z = blank_body(g->args[1]);

	z.bend = 1;
	z.parent = f->parent;
	z.ba = ba;
	z.ang = f->ang;
	z.r = f->r;
	z.mx = f->ax == 0 ? f->sg : 0;
	z.my = f->ax == 1 ? f->sg : 0;
	z.qx = f->ax == 0 ? f->c : 0;
	z.qy = f->ax == 1 ? f->c : 0;
	z.dir = f->ax * 2 + (f->sg < 0);
	st.across.lo = f->c;
	st.across.hi = f->c + (f->sg * ba);
	st.along.lo = f->za;
	st.along.hi = f->zb;
	set_rect(&z, f->ax, st);
	add_body(g, z);
}

// The flange slab maps like the parent, turned about the bend axis and
// shifted back by the bend allowance ba.
static void add_flange_slab(struct smf *g, const struct flange *f, double ba)
{
	struct strip st;
	const struct body *z = &g->bodies[g->nb - 1];
	struct body s = blank_body(g->args[1]);
	struct vec m = v3(z->mx, z->my, 0);
	struct vec o = v3(z->qx, z->qy, f->ang > 0 ? f->r : -g->sheet.t - f->r);
	struct rigid shift = rig_id();
	struct line3 ax = { o, cross(m, v3(0, 0, 1)) };
	struct rigid turn = rig_rot(ax, f->ang);
	double c1 = f->c + (f->sg * ba);

	s.parent = g->nb - 1;
	s.dir = z->dir;
	st.across.lo = c1;
	st.across.hi = c1 + (f->sg * f->leg);
	st.along.lo = f->sa;
	st.along.hi = f->sb;
	set_rect(&s, f->ax, st);
	shift.t = mul(m, -ba);
	s.m = rig_mul(g->bodies[f->parent].m, rig_mul(turn, shift));
	add_body(g, s);
}

static void st_flange(struct smf *g)
{
	struct flange f = parse_flange(g);
	double ba = bend_allowance(f.ang, f.r, g->sheet.t, g->sheet.k);

	add_zone(g, &f, ba);
	add_flange_slab(g, &f, ba);
}

// May the cut make a hole in slab b?  Dies if it crosses b at an angle,
// crosses a bend or ends inside the sheet.
// Whether a slab of normal n lies square to the axis a: a line along a
// through the sheet drifts by less than a micrometre in the slab.
static int square_to(struct smf *g, struct vec n, int a)
{
	struct vec e = axis_vec(a, 1);

	return len(cross(n, e)) * g->sheet.t < DRIFT * fabs(dot(n, e));
}

// A convex solid: its corners, and the directions of its faces and of
// its edges.
struct convex {
	struct vec pt[2 * FOOT_SIDES];
	size_t npt;
	struct vec dir[FOOT_SIDES + 1];
	size_t ndir;
};

// The point of the cut c at (u, v) across its axis and z along it.
static struct vec cut_point(const struct through *c, double u, double v,
			    double z)
{
	int au = c->a == 0 ? 1 : 0;
	int av = c->a == 2 ? 1 : 2;

	return add(add(axis_vec(au, u), axis_vec(av, v)), axis_vec(c->a, z));
}

// The solid of cut c from z0 to z1 along its axis: a rectangle, or the
// polygon round a hole and its countersink.
static void cut_solid(const struct through *c, double z0, double z1,
		      struct convex *k)
{
	double r = fmax(c->rad, c->rc) / cos(PI / (double)FOOT_SIDES);
	size_t n = c->hole ? FOOT_SIDES : 4;
	const struct box *f = &c->foot;

	k->npt = 0;
	k->ndir = 0;
	for (size_t i = 0; i < n; i++) {
		double t = 2 * PI * (double)i / (double)n;
		double u = 0;
		double v = 0;

		if (c->hole) {
			u = c->cu + (r * cos(t));
			v = c->cv + (r * sin(t));
		} else {
			u = i == 1 || i == 2 ? f->hi.x : f->lo.x;
			v = i >= 2 ? f->hi.y : f->lo.y;
		}
		k->pt[k->npt++] = cut_point(c, u, v, z0);
		k->pt[k->npt++] = cut_point(c, u, v, z1);
	}
	for (size_t i = 0; i < n; i++) {
		k->dir[k->ndir++] =
			sub(k->pt[(2 * ((i + 1) % n))], k->pt[2 * i]);
	}
	k->dir[k->ndir++] = axis_vec(c->a, 1);
}

// The parallelepiped round slab b, sheet thickness t.
static void slab_solid(const struct body *b, double t, struct convex *k)
{
	k->npt = 0;
	k->ndir = 0;
	for (unsigned i = 0; i < BOX_CORNERS; i++) {
		struct vec q = v3(i & 1U ? b->x1 : b->x0,
				  i & 2U ? b->y1 : b->y0, i & 4U ? -t : 0);

		k->pt[k->npt++] = rig_pt(b->m, q);
	}
	k->dir[k->ndir++] = rig_dir(b->m, v3(1, 0, 0));
	k->dir[k->ndir++] = rig_dir(b->m, v3(0, 1, 0));
	k->dir[k->ndir++] = rig_dir(b->m, v3(0, 0, 1));
}

// Whether the solids part along the direction n.
static int parts_along(const struct convex *a, const struct convex *b,
		       struct vec n)
{
	struct span sa = { HUGE_VAL, -HUGE_VAL };
	struct span sb = { HUGE_VAL, -HUGE_VAL };

	if (!(len(n) > NO_ANGLE)) {
		return 0;
	}
	n = unit(n);
	for (size_t i = 0; i < a->npt; i++) {
		sa.lo = fmin(sa.lo, dot(a->pt[i], n));
		sa.hi = fmax(sa.hi, dot(a->pt[i], n));
	}
	for (size_t i = 0; i < b->npt; i++) {
		sb.lo = fmin(sb.lo, dot(b->pt[i], n));
		sb.hi = fmax(sb.hi, dot(b->pt[i], n));
	}
	return sa.hi < sb.lo + EPS || sb.hi < sa.lo + EPS;
}

// Whether the convex solids a and b meet: no plane separates them, of
// those across the directions of either or across an edge of each.
static int solids_meet(const struct convex *a, const struct convex *b)
{
	for (size_t i = 0; i < a->ndir; i++) {
		if (parts_along(a, b, a->dir[i])) {
			return 0;
		}
		for (size_t j = 0; j < b->ndir; j++) {
			if (parts_along(a, b, b->dir[j]) ||
			    parts_along(a, b, cross(a->dir[i], b->dir[j]))) {
				return 0;
			}
		}
	}
	return 1;
}

// Whether the polygon of cut c, over its depth, meets slab b.
static int cut_meets(const struct smf *g, const struct through *c,
		     const struct body *b)
{
	struct convex cut;
	struct convex slab;
	double z0 = fmax(c->foot.lo.z, comp(b->box.lo, c->a) - 1);
	double z1 = fmin(c->foot.hi.z, comp(b->box.hi, c->a) + 1);

	slab_solid(b, g->sheet.t, &slab);
	cut_solid(c, z0, z1, &cut);
	return solids_meet(&cut, &slab);
}

static int cut_in_bend(const struct smf *g, const struct through *c,
		       const struct body *b);

static int cut_hits(struct smf *g, const struct through *c,
		    const struct body *b)
{
	int au = c->a == 0 ? 1 : 0;
	int av = c->a == 2 ? 1 : 2;
	struct box f = c->foot;
	struct vec n = rig_dir(b->m, v3(0, 0, 1));
	double s0 = comp(rig_pt(b->m, v3(0, 0, 0)), c->a);
	double s1 = comp(rig_pt(b->m, v3(0, 0, -g->sheet.t)), c->a);

	if (comp(b->box.hi, au) < f.lo.x + EPS ||
	    comp(b->box.lo, au) > f.hi.x - EPS ||
	    comp(b->box.hi, av) < f.lo.y + EPS ||
	    comp(b->box.lo, av) > f.hi.y - EPS ||
	    comp(b->box.hi, c->a) < f.lo.z + EPS ||
	    comp(b->box.lo, c->a) > f.hi.z - EPS) {
		return 0;
	}
	if (b->bend && cut_in_bend(g, c, b)) {
		expr_fail(&g->nm, "the cut crosses a bend", b->name);
	}
	if (b->bend) {
		return 0;
	}
	if (!square_to(g, n, c->a) && !cut_meets(g, c, b)) {
		return 0;
	}
	if (!square_to(g, n, c->a)) {
		expr_fail(&g->nm, "the cut crosses a slab at an angle",
			  b->name);
	}
	if (fmin(s0, s1) < f.lo.z - EPS || fmax(s0, s1) > f.hi.z + EPS) {
		expr_fail(&g->nm, "the cut depth ends inside a slab", b->name);
	}
	return 1;
}

// Maps the point (u, v) on the top of slab b to flat coordinates.
static struct vec slab_flat(const struct through *c, const struct body *b,
			    double u, double v)
{
	struct vec eu = axis_vec(c->a == 0 ? 1 : 0, 1);
	struct vec ev = axis_vec(c->a == 2 ? 1 : 2, 1);
	struct vec ea = axis_vec(c->a, 1);
	double s0 = comp(rig_pt(b->m, v3(0, 0, 0)), c->a);
	struct vec w = add(add(mul(eu, u), mul(ev, v)), mul(ea, s0));

	return rig_pt(rig_inv(b->m), w);
}

static void add_circ(struct smf *g, const struct through *c, size_t i)
{
	struct vec p = slab_flat(c, &g->bodies[i], c->cu, c->cv);
	double h = (c->rc - c->rad) / tan(c->angle * DEG / 2);
	struct circ ci = { i,	    p.x,   p.y, c->rad,
			   c->side, c->rc, h,	xstrdup(expr_where(&g->nm)) };

	PUSH(g->circs, g->ncircs, g->capcircs, ci);
}

// Adds the cut to slab i.  The flat axes of the slab must run along the
// U and V axes of the cut, or the rectangle would turn in the flat.
static void add_cut(struct smf *g, const struct through *c, size_t i)
{
	const struct box *f = &c->foot;
	struct vec p = slab_flat(c, &g->bodies[i], f->lo.x, f->lo.y);
	struct vec q = slab_flat(c, &g->bodies[i], f->hi.x, f->hi.y);
	struct vec pu = slab_flat(c, &g->bodies[i], f->hi.x, f->lo.y);

	if (fabs(pu.x - p.x) > EPS && fabs(pu.y - p.y) > EPS) {
		expr_fail(&g->nm,
			  "the flat axes of the slab are turned against the "
			  "cut; an outline can make the hole",
			  g->bodies[i].name);
	}
	op(g, p.x, p.y, q.x, q.y, i, 1);
}

static void through(struct smf *g, const struct through *c)
{
	int hit = 0;

	for (size_t i = 0; i < g->nb; i++) {
		if (!cut_hits(g, c, &g->bodies[i])) {
			continue;
		}
		hit = 1;
		if (c->hole) {
			add_circ(g, c, i);
		} else {
			add_cut(g, c, i);
		}
	}
	if (!hit) {
		warn("%s: warning: the cut misses the part",
		     expr_where(&g->nm));
	}
}

static long count_of(struct smf *g, size_t i)
{
	double v = arg(g, i);

	if (fabs(v - floor(v + MID)) > WHOLE || v < 1 ||
	    v > (double)MAXREPEAT) {
		expr_fail(&g->nm, "a pattern count is a whole number from 1 up",
			  g->args[i]);
	}
	return lround(v);
}

// Parses "hex P within R" at args[i]; returns the next index.
static size_t parse_hex(struct smf *g, size_t i, struct pattern *pt)
{
	if (!is(g, i + 2, "within") || i + 3 >= g->nargs) {
		expr_fail(&g->nm, "pattern hex P within R", NULL);
	}
	pt->hex = arg(g, i + 1);
	pt->rmax = arg(g, i + 3);
	if (!(pt->hex > 0) || !(pt->rmax > 0) ||
	    pt->rmax / pt->hex > HEX_RANGE) {
		expr_fail(&g->nm,
			  "a hex pattern needs P > 0, R > 0, and R under 300 P",
			  NULL);
	}
	return i + 4;
}

// Parses "pattern AX N P [AX N P]" or "pattern hex ..." at args[i] for
// cut c; returns the next index.
static size_t parse_pattern(struct smf *g, size_t i, const struct through *c,
			    struct pattern *pt)
{
	int a = c->a;
	size_t np = 0;
	double sg = 0;

	if (is(g, i + 1, "hex")) {
		return parse_hex(g, i + 1, pt);
	}
	for (i++; i + 2 < g->nargs + 1 && !is(g, i, "at"); i += 3) {
		if (np == 2) {
			expr_fail(&g->nm, "at most two pattern directions",
				  NULL);
		}
		if (i + 2 >= g->nargs) {
			expr_fail(&g->nm, "pattern AX N P", NULL);
		}
		pt->axis[np] = axis_of(g, g->args[i], &sg);
		if (pt->axis[np] == a) {
			expr_fail(&g->nm, "a pattern along the cut axis",
				  g->args[i]);
		}
		pt->n[np] = count_of(g, i + 1);
		pt->step[np] = sg * arg(g, i + 2);
		np++;
	}
	if (!np) {
		expr_fail(&g->nm, "pattern AX N P", NULL);
	}
	if (pt->n[0] * pt->n[1] > MAXREPEAT) {
		expr_fail(&g->nm, "the pattern makes too many copies", NULL);
	}
	return i;
}

// Parses "csink DC ANGLE near|far" at args[i] into hole c.
static void parse_csink(struct smf *g, size_t i, struct through *c)
{
	c->rc = arg(g, i + 1) / 2;
	c->angle = arg(g, i + 2);
	if (is(g, i + 3, "near")) {
		c->side = 1;
	} else if (is(g, i + 3, "far")) {
		c->side = -1;
	} else {
		expr_fail(&g->nm, "csink DC ANGLE near|far", g->args[i + 3]);
	}
	if (!(c->rc > c->rad) || !(c->angle > 0 && c->angle < HALF_TURN)) {
		expr_fail(&g->nm,
			  "a countersink needs DC above D and ANGLE in "
			  "(0, 180)",
			  NULL);
	}
}

// Parses "along A [at W depth L] [pattern ...]" from args[i] on; only a
// hole takes a pattern.
static void parse_where(struct smf *g, size_t i, struct through *c,
			struct pattern *pt)
{
	double sg = 0;

	if (!is(g, i, "along") || i + 1 >= g->nargs) {
		expr_fail(&g->nm, "expected: along x|y|z", NULL);
	}
	c->a = axis_of(g, g->args[i + 1], &sg);
	c->foot.lo.z = -HUGE_VAL;
	c->foot.hi.z = HUGE_VAL;
	for (i += 2; i < g->nargs;) {
		if (is(g, i, "at") && is(g, i + 2, "depth")) {
			c->foot.lo.z = arg(g, i + 1) - arg(g, i + 3) / 2;
			c->foot.hi.z = arg(g, i + 1) + arg(g, i + 3) / 2;
			i += 4;
		} else if (is(g, i, "pattern") && c->hole) {
			i = parse_pattern(g, i, c, pt);
		} else if (is(g, i, "csink") && c->hole && i + 3 < g->nargs) {
			parse_csink(g, i, c);
			i += 4;
		} else {
			expr_fail(&g->nm, "unknown option", g->args[i]);
		}
	}
}

// The cut moved by du along U and dv along V.
static struct through copy_off(const struct through *c, double du, double dv)
{
	struct through d = *c;

	d.foot.lo.x += du;
	d.foot.hi.x += du;
	d.foot.lo.y += dv;
	d.foot.hi.y += dv;
	d.cu += du;
	d.cv += dv;
	return d;
}

// The index of a copy of a pattern, along its first and second axis.
struct copy_index {
	long j, k;
};

// Moves the cut to copy at of the pattern.
static struct through copy_at(const struct through *c, const struct pattern *pt,
			      struct copy_index at)
{
	long j = at.j;
	long k = at.k;
	int au = c->a == 0 ? 1 : 0;
	double du = 0;
	double dv = 0;
	double s0 = (double)j * pt->step[0];
	double s1 = (double)k * pt->step[1];

	*(pt->axis[0] == au ? &du : &dv) += s0;
	*(pt->axis[1] == au ? &du : &dv) += s1;
	return copy_off(c, du, dv);
}

// Makes the holes of hex pattern pt about hole c.
static void hex_holes(struct smf *g, const struct through *c,
		      const struct pattern *pt)
{
	double h = pt->hex * sqrt(3) / 2;
	long nj = lround(floor(pt->rmax / h));
	long ni = lround(floor(pt->rmax / pt->hex)) + 1;

	for (long j = -nj; j <= nj; j++) {
		double dv = (double)j * h;
		double odd = (j % 2) ? pt->hex / 2 : 0;

		for (long i = -ni; i <= ni; i++) {
			double du = ((double)i * pt->hex) + odd;
			double r = hypot(du, dv);
			struct through d = copy_off(c, du, dv);

			if (r + c->rad <= pt->rmax + ON) {
				through(g, &d);
			}
		}
	}
}

static void st_cut(struct smf *g, int hole)
{
	struct through c;
	struct pattern pt = { { 0, 0 }, { 0, 0 }, { 1, 1 }, 0, 0 };
	size_t i = hole ? HOLE_ALONG : CUT_ALONG;

	memset(&c, 0, sizeof(c));
	c.hole = hole;
	if (hole) {
		c.rad = arg(g, 1) / 2;
		c.cu = arg(g, 2);
		c.cv = arg(g, 3);
		if (!(c.rad > 0)) {
			expr_fail(&g->nm, "the hole diameter must be above 0",
				  NULL);
		}
		c.foot.lo = v3(c.cu - c.rad, c.cv - c.rad, 0);
		c.foot.hi = v3(c.cu + c.rad, c.cv + c.rad, 0);
	} else {
		c.foot.lo = v3(fmin(arg(g, 1), arg(g, 3)),
			       fmin(arg(g, 2), arg(g, 4)), 0);
		c.foot.hi = v3(fmax(arg(g, 1), arg(g, 3)),
			       fmax(arg(g, 2), arg(g, 4)), 0);
	}
	parse_where(g, i, &c, &pt);
	if (pt.hex > 0) {
		hex_holes(g, &c, &pt);
		return;
	}
	for (long j = 0; j < pt.n[0]; j++) {
		for (long k = 0; k < pt.n[1]; k++) {
			struct copy_index at = { j, k };
			struct through d = copy_at(&c, &pt, at);

			through(g, &d);
		}
	}
}

static void st_outline(struct smf *g, int trim)
{
	struct pattern none = { { 0, 0 }, { 0, 0 }, { 1, 1 }, 0, 0 };

	if (g->ol.open) {
		expr_fail(&g->nm, "an outline is open", NULL);
	}
	memset(&g->ol.c, 0, sizeof(g->ol.c));
	parse_where(g, 1, &g->ol.c, &none);
	g->ol.open = 1;
	g->ol.trim = trim;
	g->ol.n = 0;
	g->ol.where = xstrdup(expr_where(&g->nm));
}

static void st_point(struct smf *g)
{
	struct ocorner k;

	if (!g->ol.open) {
		expr_fail(&g->nm, "point outside an outline", NULL);
	}
	if (g->nargs != POINT_WORDS &&
	    (g->nargs != POINT_ARC_WORDS ||
	     strcmp(g->args[POINT_WORDS], "arc") != 0 ||
	     (strcmp(g->args[POINT_DIR], "ccw") != 0 &&
	      strcmp(g->args[POINT_DIR], "cw") != 0))) {
		expr_fail(&g->nm, "point U V [arc CU CV ccw|cw]", NULL);
	}
	memset(&k, 0, sizeof(k));
	k.x = arg(g, 1);
	k.y = arg(g, 2);
	if (g->nargs == POINT_ARC_WORDS) {
		k.arc = 1;
		k.cx = arg(g, POINT_CU);
		k.cy = arg(g, POINT_CV);
		k.ccw = strcmp(g->args[POINT_DIR], "ccw") == 0;
	}
	PUSH(g->ol.uv, g->ol.n, g->ol.cap, k);
}

// The flat corner of the (U, V) corner k on slab b.
static struct ocorner flat_corner(const struct through *c, const struct body *b,
				  struct ocorner k, int flip)
{
	struct vec p = slab_flat(c, b, k.x, k.y);
	struct vec q = slab_flat(c, b, k.cx, k.cy);

	k.x = p.x;
	k.y = p.y;
	k.cx = q.x;
	k.cy = q.y;
	k.ccw = k.ccw != flip;
	return k;
}

// Whether the map from (U, V) to flat coordinates of slab b turns the
// sense of rotation round.
static int flips(const struct through *c, const struct body *b)
{
	struct vec o = slab_flat(c, b, 0, 0);
	struct vec u = sub(slab_flat(c, b, 1, 0), o);
	struct vec v = sub(slab_flat(c, b, 0, 1), o);

	return (u.x * v.y) - (u.y * v.x) < 0;
}

static void add_oline(struct smf *g, const struct through *c, size_t i)
{
	struct oline o;
	int flip = flips(c, &g->bodies[i]);

	o.body = i;
	o.n = g->ol.n;
	o.trim = g->ol.trim;
	o.c = xmalloc((g->ol.n + 1) * sizeof(*o.c));
	o.where = xstrdup(g->ol.where);
	for (size_t k = 0; k < g->ol.n; k++) {
		o.c[k] = flat_corner(c, &g->bodies[i], g->ol.uv[k], flip);
	}
	PUSH(g->olines, g->nolines, g->capolines, o);
}

// The box of the outline in (U, V): its corners and the circles of its
// arcs.
static struct box outline_foot(struct smf *g)
{
	struct box f = box_empty();

	for (size_t k = 0; k < g->ol.n; k++) {
		const struct ocorner *p = &g->ol.uv[k];

		box_add(&f, v3(p->x, p->y, 0));
		if (p->arc) {
			double r = hypot(p->x - p->cx, p->y - p->cy);

			box_add(&f, v3(p->cx - r, p->cy - r, 0));
			box_add(&f, v3(p->cx + r, p->cy + r, 0));
		}
	}
	return f;
}

// Cuts the outline through the slabs it crosses.
static void outline_cut(struct smf *g)
{
	int hit = 0;

	for (size_t i = 0; i < g->nb; i++) {
		if (!cut_hits(g, &g->ol.c, &g->bodies[i])) {
			continue;
		}
		hit = 1;
		add_oline(g, &g->ol.c, i);
	}
	if (!hit) {
		warn("%s: warning: the outline misses the part", g->ol.where);
	}
}

// Dies if two points of the open outline in a row coincide: the piece
// between them would have no length.
static void check_points(const struct smf *g)
{
	size_t n = g->ol.n;

	for (size_t k = 0; k < n; k++) {
		const struct ocorner *a = &g->ol.uv[k];
		const struct ocorner *b = &g->ol.uv[(k + 1) % n];

		if ((g->ol.trim && k + 1 == n) ||
		    hypot(b->x - a->x, b->y - a->y) > ON) {
			continue;
		}
		die("%s: two points in a row of the outline coincide",
		    g->ol.where);
	}
}

// Ends the open outline and cuts it through the slabs.
static void close_outline(struct smf *g)
{
	struct box f;

	if (!g->ol.open) {
		return;
	}
	g->ol.open = 0;
	if (g->ol.n < 2) {
		die("%s: an outline needs two points at least", g->ol.where);
	}
	check_points(g);
	f = outline_foot(g);
	g->ol.c.foot.lo.x = f.lo.x;
	g->ol.c.foot.lo.y = f.lo.y;
	g->ol.c.foot.hi.x = f.hi.x;
	g->ol.c.foot.hi.y = f.hi.y;
	outline_cut(g);
	free(g->ol.where);
	g->ol.where = NULL;
}

static void st_round(struct smf *g)
{
	struct rnd r;

	if (g->nargs != ROUND_WORDS &&
	    (g->nargs != ROUND_AT_WORDS ||
	     strcmp(g->args[ROUND_WORDS], "at") != 0)) {
		expr_fail(&g->nm, "round NAME R [at X Y Z]", NULL);
	}
	r.body = find_slab(g, g->args[1]);
	r.r = arg(g, 2);
	r.at = g->nargs == ROUND_AT_WORDS;
	r.pt = r.at ? v3(arg(g, ROUND_X), arg(g, ROUND_Y), arg(g, ROUND_Z)) :
		      v3(0, 0, 0);
	r.where = xstrdup(expr_where(&g->nm));
	if (!r.at && g->bodies[r.body].parent == NONE) {
		expr_fail(&g->nm, "round works on flanges", NULL);
	}
	if (!(r.r > 0)) {
		expr_fail(&g->nm, "the radius must be positive", NULL);
	}
	PUSH(g->rnds, g->nrnds, g->caprnds, r);
}

static void st_define(struct smf *g)
{
	char buf[DEF_LEN];
	size_t k = 0;

	for (size_t i = 2; i < g->nargs; i++) {
		size_t n = strlen(g->args[i]);

		if (k + n + 2 > sizeof(buf)) {
			expr_fail(&g->nm, "definition too long", NULL);
		}
		memcpy(buf + k, g->args[i], n);
		k += n;
		buf[k++] = ' ';
	}
	buf[k] = 0;
	define(&g->nm, g->args[0], eval(&g->nm, buf), 0);
}

static void st_include(struct smf *g)
{
	struct sibling sib;
	char *path = NULL;

	if (g->nargs != 2) {
		expr_fail(&g->nm, "include FILE", NULL);
	}
	sib.of = g->part_path;
	sib.name = g->args[1];
	path = sibling_path(sib);
	read_params(&g->nm, path);
	free(path);
}

static void statement(char *line, void *ctx)
{
	struct smf *g = ctx;
	const char *w = NULL;

	g->nargs = split_words(line, g->args, MAXARGS);
	if (g->nargs > MAXARGS) {
		expr_fail(&g->nm, "too many arguments", NULL);
	}
	if (!g->nargs) {
		return;
	}
	w = g->args[0];
	if (strcmp(w, "point") == 0) {
		st_point(g);
		return;
	}
	close_outline(g);
	if (g->nargs >= 3 && strcmp(g->args[1], "=") == 0) {
		st_define(g);
	} else if (strcmp(w, "include") == 0) {
		st_include(g);
	} else if (strcmp(w, "sheet") == 0) {
		st_sheet(g);
	} else if (strcmp(w, "plate") == 0) {
		st_plate(g);
	} else if (strcmp(w, "flange") == 0) {
		st_flange(g);
	} else if (strcmp(w, "cut") == 0) {
		st_cut(g, 0);
	} else if (strcmp(w, "hole") == 0) {
		st_cut(g, 1);
	} else if (strcmp(w, "round") == 0) {
		st_round(g);
	} else if (strcmp(w, "outline") == 0) {
		st_outline(g, 0);
	} else if (strcmp(w, "trim") == 0) {
		st_outline(g, 1);
	} else {
		expr_fail(&g->nm, "unknown statement", w);
	}
}

// The grid of cells lies between the sorted x and y coordinates of all
// ops.

static size_t cell(const struct smf *g, size_t i, size_t j)
{
	return (j * (g->nx - 1)) + i;
}

// Returns the face of cell (i - 1 + di, j - 1 + dj), NONE off the grid.
static size_t face_at(struct smf *g, size_t i, size_t j, int di, int dj)
{
	size_t a = 0;
	size_t b = 0;

	if ((i == 0 && di == 0) || (j == 0 && dj == 0)) {
		return NONE;
	}
	a = i - 1 + (size_t)di;
	b = j - 1 + (size_t)dj;
	if (a >= g->nx - 1 || b >= g->ny - 1) {
		return NONE;
	}
	return g->fid[cell(g, a, b)];
}

static size_t uniq(double *v, size_t n)
{
	size_t k = 0;

	sort_doubles(v, n);
	for (size_t i = 0; i < n; i++) {
		if (!k || v[i] - v[k - 1] > EPS) {
			v[k++] = v[i];
		}
	}
	return k;
}

// The sorted coordinates of the grid along one axis.
struct coords {
	const double *v;
	size_t n;
};

static size_t grid_index(struct coords cs, double x)
{
	const double *v = cs.v;
	size_t lo = 0;
	size_t hi = cs.n - 1;

	while (lo < hi) {
		size_t mid = (lo + hi) / 2;

		if (v[mid] < x - EPS) {
			lo = mid + 1;
		} else {
			hi = mid;
		}
	}
	if (fabs(v[lo] - x) > EPS) {
		die("internal error: coordinate %g is not on the grid", x);
	}
	return lo;
}

static size_t vertex_at(struct smf *g, size_t i, size_t j)
{
	size_t *v = &g->vid[(j * g->nx) + i];

	if (*v == NONE) {
		struct vec2 p = { g->xs[i], g->ys[j] };

		*v = g->out.nverts;
		PUSH(g->out.verts, g->out.nverts, g->out.capverts, p);
	}
	return *v;
}

// Does a vertex sit at grid point (i, j)?  Not where the four cells
// around it split in two along a straight line, or not at all.
static int is_vertex(struct smf *g, size_t i, size_t j)
{
	size_t a = face_at(g, i, j, 0, 0);
	size_t b = face_at(g, i, j, 1, 0);
	size_t c = face_at(g, i, j, 0, 1);
	size_t d = face_at(g, i, j, 1, 1);

	return !((a == b && c == d) || (a == c && b == d));
}

static void add_edge(struct smf *g, size_t v0, size_t v1, size_t lf, size_t rf)
{
	struct edge e = { v0, v1, lf, rf, 0, 0, 0 };

	PUSH(g->out.edges, g->out.nedges, g->out.capedges, e);
}

// Two bodies.
struct body_pair {
	size_t a, b;
};

// May material of the bodies touch across a cell side?  across_x is 1
// for a side across the x direction.
static int joint_ok(const struct smf *g, struct body_pair ab, int across_x)
{
	size_t a = ab.a;
	size_t b = ab.b;

	for (int k = 0; k < 2 && a != b; k++) {
		size_t z = k ? b : a;
		size_t s = k ? a : b;
		const struct body *zb = &g->bodies[z];
		int across = (fabs(zb->mx) > MID) == across_x;

		if (zb->bend && across &&
		    (zb->parent == s || g->bodies[s].parent == z)) {
			return 1;
		}
	}
	return a == b;
}

static void tear(const struct smf *g, size_t a, size_t b)
{
	die("%s: %s touches %s, which it is not bent from (a tear)",
	    g->part_path, g->bodies[a].name, g->bodies[b].name);
}

static void make_grid(struct smf *g)
{
	g->xs = xmalloc(2 * g->nops * sizeof(*g->xs));
	g->ys = xmalloc(2 * g->nops * sizeof(*g->ys));
	for (size_t i = 0; i < g->nops; i++) {
		g->xs[2 * i] = g->ops[i].x0;
		g->xs[(2 * i) + 1] = g->ops[i].x1;
		g->ys[2 * i] = g->ops[i].y0;
		g->ys[(2 * i) + 1] = g->ops[i].y1;
	}
	g->nx = uniq(g->xs, 2 * g->nops);
	g->ny = uniq(g->ys, 2 * g->nops);
	g->lab = xcalloc((g->nx - 1) * (g->ny - 1), sizeof(*g->lab));
}

// Paints the ops in order: a cut clears only the cells of its body.
static void paint(struct smf *g)
{
	for (size_t k = 0; k < g->nops; k++) {
		const struct op *o = &g->ops[k];
		struct coords xs = { g->xs, g->nx };
		struct coords ys = { g->ys, g->ny };
		size_t i0 = grid_index(xs, o->x0);
		size_t i1 = grid_index(xs, o->x1);
		size_t j0 = grid_index(ys, o->y0);
		size_t j1 = grid_index(ys, o->y1);

		for (size_t j = j0; j < j1; j++) {
			for (size_t i = i0; i < i1; i++) {
				size_t *c = &g->lab[cell(g, i, j)];

				if (!o->clear) {
					*c = o->body + 1;
				} else if (*c == o->body + 1) {
					*c = 0;
				}
			}
		}
	}
}

// Dies where material of two bodies touches without a bend between them.
static void check_tears(struct smf *g)
{
	for (size_t j = 0; j + 1 < g->ny; j++) {
		for (size_t i = 0; i + 1 < g->nx; i++) {
			size_t a = g->lab[cell(g, i, j)];
			size_t b = i + 2 < g->nx ? g->lab[cell(g, i + 1, j)] :
						   0;
			size_t c = j + 2 < g->ny ? g->lab[cell(g, i, j + 1)] :
						   0;

			struct body_pair ab = { a - 1, b - 1 };
			struct body_pair ac = { a - 1, c - 1 };

			if (a && b && !joint_ok(g, ab, 1)) {
				tear(g, a - 1, b - 1);
			}
			if (a && c && !joint_ok(g, ac, 0)) {
				tear(g, a - 1, c - 1);
			}
		}
	}
}

// A prism of the folded sheet: the quadrilateral pt[0..4) across axis
// and its copy moved by axis, with the normals of its faces and the
// directions of its edges.
enum {
	QUAD = 4, // Corners of the cross section of a prism.
	QUAD_PAIRS = 6, // Pairs of those corners.
	PRISM_AXES = QUAD_PAIRS + 1, // And the axis.
	PRISM_PTS = 2 * QUAD // Corners of a prism.
};

struct prism4 {
	struct vec pt[PRISM_PTS];
	struct vec nrm[PRISM_AXES];
	struct vec dir[PRISM_AXES];
};

// A rectangle of the flat cells of one body.
struct cells {
	size_t body;
	double x0, y0, x1, y1;
};

static struct vec2 v2(double x, double y)
{
	struct vec2 p = { x, y };

	return p;
}

// The point of body b at (x, y) in the flat, z below its top.
// A part of one face, body b alone: map 1, on its parent slab, map 0.
struct body_part {
	struct map maps[2];
	size_t face;
	struct part p;
};

static void body_part(const struct smf *g, size_t b, struct body_part *bp)
{
	const struct body *bd = &g->bodies[b];

	bp->maps[0] = body_map(bd->bend ? &g->bodies[bd->parent] : bd, NONE);
	bp->maps[1] = body_map(bd, 0);
	bp->face = 1;
	memset(&bp->p, 0, sizeof(bp->p));
	bp->p.t = g->sheet.t;
	bp->p.place = rig_id();
	bp->p.maps = bp->maps;
	bp->p.nmaps = 2;
	bp->p.faces = &bp->face;
	bp->p.nfaces = 1;
}

static struct vec body_pt(const struct smf *g, size_t b, struct vec2 p,
			  double z)
{
	struct body_part bp;

	body_part(g, b, &bp);
	return fmap(&bp.p, 0, p.x, p.y, z);
}

// The turn of the slice k of a bend, in radians.
static double slice_turn(const struct smf *g, const struct cells *k)
{
	const struct body *bd = &g->bodies[k->body];
	double run = fabs(bd->mx) > MID ? k->x1 - k->x0 : k->y1 - k->y0;

	return fabs(bd->ang) * DEG * run / bd->ba;
}

// Moves the corners pt[0 .. 3] of slice k of a bend on its outer face out
// from the axis, so that the chord between them stands clear of the arc:
// the prism of pt then holds the slice.
static void cover_slice(const struct smf *g, const struct cells *k,
			struct vec *pt)
{
	struct body_part bp;
	struct line3 ax;
	struct vec d;
	double grow = 1 / cos(slice_turn(g, k) / 2);

	body_part(g, k->body, &bp);
	ax = bend_axis(&bp.p, 1);
	d = unit(ax.d);
	for (int i = 0; i < 4; i++) {
		struct vec r = sub(pt[i], ax.o);
		struct vec foot = add(ax.o, mul(d, dot(r, d)));
		struct vec out = sub(pt[i], foot);
		struct vec other = sub(pt[3 - i], foot);

		if (len(out) > len(other)) {
			pt[i] = add(foot, mul(out, grow));
		}
	}
}

// The prism of the flat rectangle k of a body, at most a slice of a bend
// wide; the axis runs across the bend.
static void cell_prism(const struct smf *g, const struct cells *k,
		       struct prism4 *q)
{
	const struct body *bd = &g->bodies[k->body];
	int along_x = !bd->bend || fabs(bd->mx) > MID;
	struct vec2 c0 = v2(k->x0, k->y0);
	struct vec2 c1 = along_x ? v2(k->x1, k->y0) : v2(k->x0, k->y1);
	struct vec2 c2 = along_x ? v2(k->x0, k->y1) : v2(k->x1, k->y0);
	struct vec ax;
	size_t n = 0;

	q->pt[0] = body_pt(g, k->body, c0, 0);
	q->pt[1] = body_pt(g, k->body, c1, 0);
	q->pt[2] = body_pt(g, k->body, c1, -g->sheet.t);
	q->pt[3] = body_pt(g, k->body, c0, -g->sheet.t);
	if (bd->bend) {
		cover_slice(g, k, q->pt);
	}
	ax = sub(body_pt(g, k->body, c2, 0), q->pt[0]);
	for (size_t i = 0; i < QUAD; i++) {
		q->pt[i + QUAD] = add(q->pt[i], ax);
		for (size_t j = i + 1; j < QUAD; j++) {
			q->dir[n] = sub(q->pt[j], q->pt[i]);
			q->nrm[n] = cross(ax, q->dir[n]);
			n++;
		}
	}
	q->dir[QUAD_PAIRS] = ax;
	q->nrm[QUAD_PAIRS] = ax;
}

// Whether the prisms a and b lie more than APART apart along n.
static int apart_along(const struct prism4 *a, const struct prism4 *b,
		       struct vec n)
{
	struct span sa = { HUGE_VAL, -HUGE_VAL };
	struct span sb = { HUGE_VAL, -HUGE_VAL };

	if (!(len(n) > NO_ANGLE)) {
		return 0;
	}
	n = unit(n);
	for (size_t i = 0; i < PRISM_PTS; i++) {
		sa.lo = fmin(sa.lo, dot(a->pt[i], n));
		sa.hi = fmax(sa.hi, dot(a->pt[i], n));
		sb.lo = fmin(sb.lo, dot(b->pt[i], n));
		sb.hi = fmax(sb.hi, dot(b->pt[i], n));
	}
	return sa.hi < sb.lo - APART || sb.hi < sa.lo - APART;
}

// Whether the prisms a and b touch or overlap: no plane parts them, of
// those along the faces of either or across an edge of each.
static int prisms_touch(const struct prism4 *a, const struct prism4 *b)
{
	for (size_t i = 0; i < PRISM_AXES; i++) {
		if (apart_along(a, b, a->nrm[i]) ||
		    apart_along(a, b, b->nrm[i])) {
			return 0;
		}
		for (size_t j = 0; j < PRISM_AXES; j++) {
			if (apart_along(a, b, cross(a->dir[i], b->dir[j]))) {
				return 0;
			}
		}
	}
	return 1;
}

// The number of slices of a bend to cut the flat rectangle k into.
static size_t slice_count(const struct smf *g, const struct cells *k)
{
	const struct body *bd = &g->bodies[k->body];
	int along_x = fabs(bd->mx) > MID;
	double run = along_x ? k->x1 - k->x0 : k->y1 - k->y0;
	double slices = ceil(fabs(bd->ang) * DEG * run / bd->ba / SLICE);

	return bd->bend && slices > 1 ? (size_t)slices : 1;
}

// Adds the flat rectangle k of a run of cells of a body, cut into
// slices of a bend.
static void add_cells(const struct smf *g, struct cells k, struct cells **out,
		      size_t *n, size_t *cap)
{
	int along_x = fabs(g->bodies[k.body].mx) > MID;
	double *lo = along_x ? &k.x0 : &k.y0;
	double *hi = along_x ? &k.x1 : &k.y1;
	double a = *lo;
	double span = *hi - *lo;
	size_t m = slice_count(g, &k);

	for (size_t i = 0; i < m; i++) {
		*lo = a + (span * (double)i / (double)m);
		*hi = a + (span * (double)(i + 1) / (double)m);
		PUSH(*out, *n, *cap, k);
	}
}

// The rectangles of the cells of each body, row by row.
static size_t body_cells(const struct smf *g, struct cells **out)
{
	size_t n = 0;
	size_t cap = 0;

	*out = NULL;
	for (size_t j = 0; j + 1 < g->ny; j++) {
		for (size_t i = 0; i + 1 < g->nx;) {
			size_t a = g->lab[cell(g, i, j)];
			size_t e = i + 1;
			struct cells k;

			while (e + 1 < g->nx && g->lab[cell(g, e, j)] == a) {
				e++;
			}
			k.body = a - 1;
			k.x0 = g->xs[i];
			k.x1 = g->xs[e];
			k.y0 = g->ys[j];
			k.y1 = g->ys[j + 1];
			if (a) {
				add_cells(g, k, out, &n, &cap);
			}
			i = e;
		}
	}
	return n;
}

// Whether the bodies a and b join: one is bent from the other, or both
// are bent from one.
static int related(const struct smf *g, size_t a, size_t b)
{
	const struct body *x = &g->bodies[a];
	const struct body *y = &g->bodies[b];

	return a == b || x->parent == b || y->parent == a ||
	       (x->bend && y->bend && x->parent == y->parent);
}

// Whether the boxes a and b come within APART of each other.
static int boxes_near(struct box a, struct box b)
{
	return a.lo.x < b.hi.x + APART && b.lo.x < a.hi.x + APART &&
	       a.lo.y < b.hi.y + APART && b.lo.y < a.hi.y + APART &&
	       a.lo.z < b.hi.z + APART && b.lo.z < a.hi.z + APART;
}

// The solid of the material of cell k, whose prism is q, into s, and the
// part of its body into bp for a bend.
static void cell_solid(const struct smf *g, const struct cells *k,
		       const struct prism4 *q, struct body_part *bp,
		       struct csolid *s)
{
	const struct body *bd = &g->bodies[k->body];

	memset(s, 0, sizeof(*s));
	s->kind = bd->bend ? CS_BEND : CS_SLAB;
	s->x0 = k->x0;
	s->y0 = k->y0;
	s->x1 = k->x1;
	s->y1 = k->y1;
	csolid_prism(s, q->pt);
	if (bd->bend) {
		body_part(g, k->body, bp);
		s->p = &bp->p;
		s->mi = 1;
		s->fi = 0;
		s->axis = bend_axis(&bp->p, 1);
		s->r0 = bd->r;
		s->r1 = bd->r + g->sheet.t;
		return;
	}
	s->p = &g->out;
	s->mi = k->body;
	for (size_t f = 0; f < g->out.nfaces; f++) {
		if (g->out.faces[f] == k->body) {
			s->fi = f;
			break;
		}
	}
}

// The solid of cut c, from z0 to z1 along its axis.
static void cut_csolid(const struct through *c, double z0, double z1,
		       struct csolid *s)
{
	struct box b = { v3(0, 0, 0), v3(0, 0, 0) };
	int au = c->a == 0 ? 1 : 0;
	int av = c->a == 2 ? 1 : 2;
	double r = c->hole ? c->rad : 0;
	double u0 = c->hole ? c->cu - r : c->foot.lo.x;
	double u1 = c->hole ? c->cu + r : c->foot.hi.x;
	double v0 = c->hole ? c->cv - r : c->foot.lo.y;
	double v1 = c->hole ? c->cv + r : c->foot.hi.y;

	memset(s, 0, sizeof(*s));
	s->kind = c->hole ? CS_HOLE : CS_RECT;
	s->a = c->a;
	s->cu = c->cu;
	s->cv = c->cv;
	s->r = r;
	s->x0 = u0;
	s->y0 = v0;
	s->x1 = u1;
	s->y1 = v1;
	s->z0 = z0;
	s->z1 = z1;
	b.lo = add(add(axis_vec(au, u0), axis_vec(av, v0)), axis_vec(c->a, z0));
	b.hi = add(add(axis_vec(au, u1), axis_vec(av, v1)), axis_vec(c->a, z1));
	csolid_box(s, b);
}

// Whether cut c, a hole of its plain radius or a rectangle, meets the
// material of bend b: its cylindrical shell, slice by slice.
static int cut_in_bend(const struct smf *g, const struct through *c,
		       const struct body *b)
{
	struct cells k = { (size_t)(b - g->bodies), b->x0, b->y0, b->x1,
			   b->y1 };
	double z0 = fmax(c->foot.lo.z, comp(b->box.lo, c->a) - 1);
	double z1 = fmin(c->foot.hi.z, comp(b->box.hi, c->a) + 1);
	struct csolid cut;
	struct cells *sl = NULL;
	size_t n = 0;
	size_t cap = 0;
	int hit = 0;

	cut_csolid(c, z0, z1, &cut);
	add_cells(g, k, &sl, &n, &cap);
	for (size_t i = 0; i < n && !hit; i++) {
		struct prism4 q;
		struct body_part bp;
		struct csolid s;

		cell_prism(g, &sl[i], &q);
		cell_solid(g, &sl[i], &q, &bp, &s);
		hit = csolids_meet(&cut, &s);
	}
	free(sl);
	return hit;
}

// Whether the material of cells a and b, with prisms p and q, meets.
static int cells_meet(const struct smf *g, const struct cells *a,
		      const struct cells *b, const struct prism4 *p,
		      const struct prism4 *q)
{
	struct body_part ba;
	struct body_part bb;
	struct csolid sa;
	struct csolid sb;

	cell_solid(g, a, p, &ba, &sa);
	cell_solid(g, b, q, &bb, &sb);
	return csolids_meet(&sa, &sb);
}

// Dies if the cells a and b of two bodies that do not join touch in the
// folded part: one solid may not touch itself.
static void check_pair(const struct smf *g, const struct cells *a,
		       const struct cells *b)
{
	struct prism4 p;
	struct prism4 q;

	if (related(g, a->body, b->body) ||
	    !boxes_near(g->bodies[a->body].box, g->bodies[b->body].box)) {
		return;
	}
	cell_prism(g, a, &p);
	cell_prism(g, b, &q);
	if (prisms_touch(&p, &q) && cells_meet(g, a, b, &p, &q)) {
		die("%s: %s touches %s in the folded part; a part may not "
		    "touch itself",
		    g->part_path, g->bodies[a->body].name,
		    g->bodies[b->body].name);
	}
}

static void check_contacts(const struct smf *g)
{
	struct cells *k = NULL;
	size_t n = body_cells(g, &k);

	for (size_t i = 0; i < n; i++) {
		for (size_t j = i + 1; j < n; j++) {
			check_pair(g, &k[i], &k[j]);
		}
	}
	free(k);
}

// Floods face id from cell k over the cells of the same body.
static void flood(struct smf *g, size_t k, size_t id, size_t *stack)
{
	size_t sp = 0;

	stack[sp++] = k;
	g->fid[k] = id;
	while (sp) {
		size_t c = stack[--sp];
		size_t i = c % (g->nx - 1);
		size_t j = c / (g->nx - 1);
		const size_t nbr[4] = { i > 0 ? c - 1 : NONE,
					i + 2 < g->nx ? c + 1 : NONE,
					j > 0 ? c - (g->nx - 1) : NONE,
					j + 2 < g->ny ? c + (g->nx - 1) :
							NONE };

		for (int q = 0; q < 4; q++) {
			if (nbr[q] != NONE && g->fid[nbr[q]] == NONE &&
			    g->lab[nbr[q]] == g->lab[k]) {
				g->fid[nbr[q]] = id;
				stack[sp++] = nbr[q];
			}
		}
	}
}

static void find_faces(struct smf *g)
{
	size_t nc = (g->nx - 1) * (g->ny - 1);
	size_t *stack = xmalloc(nc * sizeof(*stack));

	g->fid = xmalloc(nc * sizeof(*g->fid));
	for (size_t k = 0; k < nc; k++) {
		g->fid[k] = NONE;
	}
	for (size_t k = 0; k < nc; k++) {
		size_t map = g->lab[k] - 1;

		if (!g->lab[k] || g->fid[k] != NONE) {
			continue;
		}
		PUSH(g->out.faces, g->out.nfaces, g->out.capfaces, map);
		flood(g, k, g->out.nfaces - 1, stack);
	}
	free(stack);
}

// Horizontal edges run along +x, with the cell above on the left.
static void row_edges(struct smf *g, size_t j)
{
	size_t start = NONE;

	for (size_t i = 0; i < g->nx; i++) {
		int has = i + 1 < g->nx && face_at(g, i + 1, j, 0, 1) !=
						   face_at(g, i + 1, j, 0, 0);

		if (start != NONE && (!has || is_vertex(g, i, j))) {
			add_edge(g, vertex_at(g, start, j), vertex_at(g, i, j),
				 face_at(g, i, j, 0, 1),
				 face_at(g, i, j, 0, 0));
			start = NONE;
		}
		if (has && start == NONE) {
			start = i;
		}
	}
}

// Vertical edges run along +y, with the cell at -x on the left.
static void column_edges(struct smf *g, size_t i)
{
	size_t start = NONE;

	for (size_t j = 0; j < g->ny; j++) {
		int has = j + 1 < g->ny && face_at(g, i, j + 1, 0, 0) !=
						   face_at(g, i, j + 1, 1, 0);

		if (start != NONE && (!has || is_vertex(g, i, j))) {
			add_edge(g, vertex_at(g, i, start), vertex_at(g, i, j),
				 face_at(g, i, j, 0, 0),
				 face_at(g, i, j, 1, 0));
			start = NONE;
		}
		if (has && start == NONE) {
			start = j;
		}
	}
}

static void find_edges(struct smf *g)
{
	g->vid = xmalloc(g->nx * g->ny * sizeof(*g->vid));
	for (size_t k = 0; k < g->nx * g->ny; k++) {
		g->vid[k] = NONE;
	}
	for (size_t j = 0; j < g->ny; j++) {
		row_edges(g, j);
	}
	for (size_t i = 0; i < g->nx; i++) {
		column_edges(g, i);
	}
}

static size_t vertex_find(const struct smf *g, double x, double y)
{
	for (size_t i = 0; i < g->out.nverts; i++) {
		if (fabs(g->out.verts[i].x - x) < ON &&
		    fabs(g->out.verts[i].y - y) < ON) {
			return i;
		}
	}
	return NONE;
}

// Finds the two outline edges at vertex v.  Returns 0, or -1 if v is no
// simple corner of the outline.
static int corner_edges(struct smf *g, size_t v, size_t *es)
{
	size_t n = 0;

	for (size_t i = 0; i < g->out.nedges; i++) {
		const struct edge *e = &g->out.edges[i];

		if (e->v0 != v && e->v1 != v) {
			continue;
		}
		if (n == 2 || e->arc || (e->lf == NONE) == (e->rf == NONE)) {
			return -1;
		}
		es[n++] = i;
	}
	return n == 2 ? 0 : -1;
}

// Adds the arc edge e, with its centre and faces set, between the new
// vertices nv at np, counterclockwise about its centre.
static void add_arc(struct smf *g, const size_t *nv, const struct vec2 *np,
		    struct edge e)
{
	double cr = ((np[0].x - e.cx) * (np[1].y - e.cy)) -
		    ((np[0].y - e.cy) * (np[1].x - e.cx));

	e.v0 = cr > 0 ? nv[0] : nv[1];
	e.v1 = cr > 0 ? nv[1] : nv[0];
	PUSH(g->out.edges, g->out.nedges, g->out.capedges, e);
}

// Moves the corner end of each edge es[k] to np[k], a new vertex.
static void cut_corner(struct smf *g, size_t v, const size_t *es,
		       const struct vec2 *np, size_t *nv)
{
	for (int k = 0; k < 2; k++) {
		struct edge *e = &g->out.edges[es[k]];

		nv[k] = g->out.nverts;
		PUSH(g->out.verts, g->out.nverts, g->out.capverts, np[k]);
		if (e->v0 == v) {
			e->v0 = nv[k];
		} else {
			e->v1 = nv[k];
		}
	}
}

// Replaces the corner at vertex v by an arc of the radius of round rd: a
// round of a convex corner, or a fillet of a concave one.
static void round_corner(struct smf *g, size_t v, const struct rnd *rd)
{
	size_t es[2];
	size_t nv[2];
	struct vec2 c = g->out.verts[v];
	struct vec2 np[2];
	size_t face = 0;
	double r = rd->r;
	const char *where = rd->where;
	struct edge arc = { 0, 0, NONE, NONE, 1, 0, 0 };

	if (corner_edges(g, v, es)) {
		die("%s: round: not a corner of the outline", where);
	}
	face = g->out.edges[es[0]].lf != NONE ? g->out.edges[es[0]].lf :
						g->out.edges[es[0]].rf;
	for (int k = 0; k < 2; k++) {
		const struct edge *e = &g->out.edges[es[k]];
		struct vec2 o = g->out.verts[e->v0 == v ? e->v1 : e->v0];
		double l = hypot(o.x - c.x, o.y - c.y);

		if (l < r + EPS) {
			die("%s: round: an edge is shorter than the radius",
			    where);
		}
		np[k].x = c.x + (o.x - c.x) / l * r;
		np[k].y = c.y + (o.y - c.y) / l * r;
	}
	arc.cx = np[0].x + np[1].x - c.x;
	arc.cy = np[0].y + np[1].y - c.y;
	if (point_in_face(&g->out, face, v2(arc.cx, arc.cy))) {
		arc.lf = face;
	} else {
		arc.rf = face;
	}
	cut_corner(g, v, es, np, nv);
	add_arc(g, nv, np, arc);
	index_faces(&g->out);
}

// Rounds the two corners at the free end of flange r->body.
static void round_flange(struct smf *g, const struct rnd *r)
{
	const struct body *b = &g->bodies[r->body];
	int across_x = b->dir < 2;
	const double ends[4] = { b->x1, b->x0, b->y1, b->y0 };
	double end = ends[b->dir];

	for (int k = 0; k < 2; k++) {
		double alongs[2][2] = { { b->x0, b->x1 }, { b->y0, b->y1 } };
		double along = alongs[across_x][k];
		size_t v = across_x ? vertex_find(g, end, along) :
				      vertex_find(g, along, end);

		if (v == NONE) {
			die("%s: round: no corner there", r->where);
		}
		round_corner(g, v, r);
	}
}

// Rounds the corner of slab r->body at the 3D point r->pt.
static void round_at(struct smf *g, const struct rnd *r)
{
	const struct body *b = &g->bodies[r->body];
	struct vec n = v3(b->m.r[0][2], b->m.r[1][2], b->m.r[2][2]);

	for (size_t v = 0; v < g->out.nverts; v++) {
		struct vec2 f = g->out.verts[v];
		struct vec d = sub(rig_pt(b->m, v3(f.x, f.y, 0)), r->pt);

		if (f.x < fmin(b->x0, b->x1) - EPS ||
		    f.x > fmax(b->x0, b->x1) + EPS ||
		    f.y < fmin(b->y0, b->y1) - EPS ||
		    f.y > fmax(b->y0, b->y1) + EPS) {
			continue;
		}
		if (len(sub(d, mul(n, dot(d, n)))) < CORNER_AT) {
			round_corner(g, v, r);
			return;
		}
	}
	die("%s: round: no corner of the slab there", r->where);
}

static double edge_dist(struct smf *g, const struct edge *e, double x, double y)
{
	struct vec2 s = g->out.verts[e->v0];
	struct vec2 t = g->out.verts[e->v1];
	double dx = t.x - s.x;
	double dy = t.y - s.y;
	double u = ((x - s.x) * dx + (y - s.y) * dy) / (dx * dx + dy * dy);

	if (e->arc) {
		struct arc_ang aa = arc_angles(&g->out, e);
		double a = atan2(y - e->cy, x - e->cx);

		if (fmod(a - aa.a0 + (4 * PI), 2 * PI) <= aa.da) {
			return fabs(hypot(x - e->cx, y - e->cy) - aa.r);
		}
		return fmin(hypot(x - s.x, y - s.y), hypot(x - t.x, y - t.y));
	}
	u = fmin(1, fmax(0, u));
	return hypot(x - s.x - (u * dx), y - s.y - (u * dy));
}

// The sweep of the arc from corner a to corner b about the centre of a,
// counterclockwise positive.
static double corner_sweep(const struct ocorner *a, const struct ocorner *b)
{
	double t0 = atan2(a->y - a->cy, a->x - a->cx);
	double t1 = atan2(b->y - a->cy, b->x - a->cx);
	double s = fmod(t1 - t0 + (4 * PI), 2 * PI);

	if (s < EPS) {
		s = 2 * PI;
	}
	return a->ccw ? s : s - (2 * PI);
}

// The point at s in [0, 1] from corner a of outline o to the next one.
static struct vec2 outline_at(const struct oline *o, const struct ocorner *a,
			      double s)
{
	const struct ocorner *b = &o->c[((size_t)(a - o->c) + 1) % o->n];
	struct vec2 p;
	double r = 0;
	double t = 0;

	if (!a->arc) {
		p.x = a->x + ((b->x - a->x) * s);
		p.y = a->y + ((b->y - a->y) * s);
		return p;
	}
	r = hypot(a->x - a->cx, a->y - a->cy);
	t = atan2(a->y - a->cy, a->x - a->cx) + (corner_sweep(a, b) * s);
	p.x = a->cx + (r * cos(t));
	p.y = a->cy + (r * sin(t));
	return p;
}

// Twice the signed area of outline o, from points on each piece.
static double outline_area(const struct oline *o)
{
	double a = 0;

	for (size_t k = 0; k < o->n; k++) {
		for (int i = 0; i < OUTLINE_STEPS; i++) {
			struct vec2 p = outline_at(o, &o->c[k],
						   i / (double)OUTLINE_STEPS);
			struct vec2 q = outline_at(
				o, &o->c[k], (i + 1) / (double)OUTLINE_STEPS);

			a += (p.x * q.y) - (p.y * q.x);
		}
	}
	return a;
}

// Dies unless every point of outline o is inside face f, clear of its
// edges and holes.
static void outline_clear(struct smf *g, const struct oline *o, size_t f)
{
	for (size_t k = 0; k < o->n; k++) {
		for (int i = 0; i < OUTLINE_STEPS; i++) {
			struct vec2 p = outline_at(o, &o->c[k],
						   i / (double)OUTLINE_STEPS);

			if (!point_in_face(&g->out, f, p)) {
				die("%s: the outline leaves its face",
				    o->where);
			}
			for (size_t j = 0; j < g->out.nedges; j++) {
				const struct edge *e = &g->out.edges[j];

				if ((e->lf == f || e->rf == f) &&
				    edge_dist(g, e, p.x, p.y) < EPS) {
					die("%s: the outline touches an edge",
					    o->where);
				}
			}
		}
	}
}

// The free outline edges of the face of slab body at vertex v, at most
// two.  Returns their number.
static size_t free_edges(struct smf *g, size_t v, size_t body, size_t *es)
{
	size_t n = 0;

	for (size_t i = 0; i < g->out.nedges; i++) {
		const struct edge *e = &g->out.edges[i];
		size_t f = e->lf != NONE ? e->lf : e->rf;

		if ((e->v0 != v && e->v1 != v) ||
		    (e->lf != NONE) == (e->rf != NONE) ||
		    g->out.faces[f] != body) {
			continue;
		}
		if (n == 2) {
			return 3;
		}
		es[n++] = i;
	}
	return n;
}

// The ends of a trim curve on the outline of slab body: vertices a and b.
struct trim_ends {
	size_t a, b, body;
};

// A run of free edges from vertex a to vertex b.
struct chain {
	size_t *e;
	size_t n;
	int left; // Whether the face is on the left, from a toward b.
	size_t face;
};

// Follows the free edges from t->a, first along edge first, to t->b.
// Returns 0, or -1 if the run ends somewhere else.
static int walk(struct smf *g, const struct trim_ends *t, size_t first,
		struct chain *c)
{
	size_t a = t->a;
	size_t b = t->b;
	size_t body = t->body;
	size_t v = a;
	size_t e = first;
	size_t cap = 0;

	c->e = NULL;
	c->n = 0;
	c->face = g->out.edges[e].lf != NONE ? g->out.edges[e].lf :
					       g->out.edges[e].rf;
	c->left = (g->out.edges[e].v0 == a) == (g->out.edges[e].lf != NONE);
	while (c->n <= g->out.nedges) {
		size_t es[2];

		PUSH(c->e, c->n, cap, e);
		v = g->out.edges[e].v0 == v ? g->out.edges[e].v1 :
					      g->out.edges[e].v0;
		if (v == b) {
			return 0;
		}
		if (free_edges(g, v, body, es) != 2) {
			return -1;
		}
		e = es[0] == e ? es[1] : es[0];
	}
	return -1;
}

// Twice the area enclosed by chain c and the trim curve o, unsigned.
static double chain_area(struct smf *g, const struct chain *c,
			 const struct oline *o, size_t a)
{
	double s = 0;
	size_t v = a;
	struct vec2 p = g->out.verts[a];

	for (size_t k = 0; k < c->n; k++) {
		const struct edge *e = &g->out.edges[c->e[k]];
		struct vec2 q;

		v = e->v0 == v ? e->v1 : e->v0;
		q = g->out.verts[v];
		s += (p.x * q.y) - (p.y * q.x);
		p = q;
	}
	for (size_t k = o->n - 1; k > 0; k--) {
		for (int i = OUTLINE_STEPS; i > 0; i--) {
			struct vec2 q =
				outline_at(o, &o->c[k - 1],
					   (i - 1) / (double)OUTLINE_STEPS);

			s += (p.x * q.y) - (p.y * q.x);
			p = q;
		}
	}
	return fabs(s);
}

// Drops the edges of chain c.
static void drop_chain(struct smf *g, const struct chain *c)
{
	size_t n = 0;

	for (size_t i = 0; i < g->out.nedges; i++) {
		size_t k = 0;

		while (k < c->n && c->e[k] != i) {
			k++;
		}
		if (k == c->n) {
			g->out.edges[n++] = g->out.edges[i];
		}
	}
	g->out.nedges = n;
}

// Two vertices, the ends of an edge.
struct vpair {
	size_t from, to;
};

// Adds the edge of piece p of an outline from vertex run.from to vertex
// run.to, with the face of c on the left of that run if c->left, else on
// its right.
static void push_piece(struct smf *g, const struct ocorner *p, struct vpair run,
		       const struct chain *c)
{
	struct edge e = { run.from, run.to, NONE, NONE, p->arc, p->cx, p->cy };
	int fwd = !p->arc || p->ccw;

	if (!fwd) {
		e.v0 = run.to;
		e.v1 = run.from;
	}
	if (fwd == c->left) {
		e.lf = c->face;
	} else {
		e.rf = c->face;
	}
	PUSH(g->out.edges, g->out.nedges, g->out.capedges, e);
}

// Dies unless both ends of every arc of outline o lie on its circle;
// the last corner of an open curve has no arc.
static void check_arcs(const struct oline *o)
{
	for (size_t k = 0; k < o->n; k++) {
		const struct ocorner *a = &o->c[k];
		const struct ocorner *n = &o->c[(k + 1) % o->n];

		if (a->arc && fabs(hypot(a->x - a->cx, a->y - a->cy) -
				   hypot(n->x - a->cx, n->y - a->cy)) > ON) {
			die("%s: the ends of an arc are not on one circle",
			    o->where);
		}
	}
}

// Adds the pieces of trim curve o from vertex t->a to vertex t->b, with
// the face of chain c on the same side.
static void add_trim(struct smf *g, const struct oline *o,
		     const struct trim_ends *t, const struct chain *c)
{
	size_t prev = t->a;

	for (size_t k = 0; k + 1 < o->n; k++) {
		size_t next = t->b;
		struct vpair run = { 0, 0 };

		if (k + 2 < o->n) {
			struct vec2 q = { o->c[k + 1].x, o->c[k + 1].y };

			next = g->out.nverts;
			PUSH(g->out.verts, g->out.nverts, g->out.capverts, q);
		}
		run.from = prev;
		run.to = next;
		push_piece(g, &o->c[k], run, c);
		prev = next;
	}
}

// The vertex at (x, y) on the outline of slab body: an existing one, or a
// new one that splits the free straight edge through the point.
static size_t trim_end(struct smf *g, double x, double y, size_t body)
{
	size_t v = vertex_find(g, x, y);

	for (size_t i = 0; i < g->out.nedges && v == NONE; i++) {
		struct edge *e = &g->out.edges[i];
		size_t f = e->lf != NONE ? e->lf : e->rf;
		struct vec2 p = { x, y };
		struct edge t;

		if (e->arc || (e->lf != NONE) == (e->rf != NONE) ||
		    g->out.faces[f] != body || edge_dist(g, e, x, y) > ON) {
			continue;
		}
		v = g->out.nverts;
		PUSH(g->out.verts, g->out.nverts, g->out.capverts, p);
		t = *e;
		t.v0 = v;
		e->v1 = v;
		PUSH(g->out.edges, g->out.nedges, g->out.capedges, t);
	}
	return v;
}

// Replaces the run of outline edges between the ends of trim curve o
// that cuts off the smaller piece of the face by the curve.
static void place_trim(struct smf *g, const struct oline *o)
{
	size_t a = trim_end(g, o->c[0].x, o->c[0].y, o->body);
	size_t b = trim_end(g, o->c[o->n - 1].x, o->c[o->n - 1].y, o->body);
	size_t es[2];
	struct chain c[2];
	struct trim_ends t = { a, b, o->body };
	int ok[2];
	int k = 0;

	if (a == NONE || b == NONE || a == b) {
		die("%s: a trim must start and end on the outline", o->where);
	}
	if (free_edges(g, a, o->body, es) != 2) {
		die("%s: the trim starts at no simple corner", o->where);
	}
	for (k = 0; k < 2; k++) {
		ok[k] = walk(g, &t, es[k], &c[k]) == 0;
	}
	if (!ok[0] && !ok[1]) {
		die("%s: no free outline runs between the trim ends", o->where);
	}
	k = !ok[0] ||
	    (ok[1] && chain_area(g, &c[1], o, a) < chain_area(g, &c[0], o, a));
	drop_chain(g, &c[k]);
	add_trim(g, o, &t, &c[k]);
	free(c[0].e);
	free(c[1].e);
	index_faces(&g->out);
}

// The face of slab body that holds the point at, or NONE.
static size_t slab_face(struct smf *g, size_t body, struct vec2 at)
{
	for (size_t i = 0; i < g->out.nfaces; i++) {
		if (g->out.faces[i] == body && point_in_face(&g->out, i, at)) {
			return i;
		}
	}
	return NONE;
}

// Adds the outline as a loop of edges round a hole in its face.
static void place_outline(struct smf *g, const struct oline *o)
{
	size_t f = slab_face(g, o->body, v2(o->c[0].x, o->c[0].y));
	size_t v0 = g->out.nverts;
	struct chain side = { NULL, 0, outline_area(o) <= 0, f };

	if (f == NONE) {
		die("%s: the outline is not on its slab", o->where);
	}
	outline_clear(g, o, f);
	check_arcs(o);
	for (size_t k = 0; k < o->n; k++) {
		struct vec2 p = { o->c[k].x, o->c[k].y };

		PUSH(g->out.verts, g->out.nverts, g->out.capverts, p);
	}
	for (size_t k = 0; k < o->n; k++) {
		struct vpair run = { v0 + k, v0 + ((k + 1) % o->n) };

		push_piece(g, &o->c[k], run, &side);
	}
	index_faces(&g->out);
}

static size_t hole_face(struct smf *g, const struct circ *c)
{
	for (size_t f = 0; f < g->out.nfaces; f++) {
		if (g->out.faces[f] == c->body &&
		    point_in_face(&g->out, f, v2(c->x, c->y))) {
			return f;
		}
	}
	die("%s: the hole is not on its slab %s", c->where,
	    g->bodies[c->body].name);
}

// Adds the hole to its face; it must stay clear of every edge of the
// face and of the other holes.
static void place_hole(struct smf *g, const struct circ *c)
{
	size_t face = hole_face(g, c);
	double rc = c->side ? c->rc : c->r;
	struct hole h = { face, c->x, c->y, c->r, c->side, rc, c->h };

	for (size_t i = 0; i < g->out.nedges; i++) {
		const struct edge *e = &g->out.edges[i];

		if ((e->lf == face || e->rf == face) &&
		    edge_dist(g, e, c->x, c->y) < rc + EPS) {
			die("%s: the hole touches an edge of %s", c->where,
			    g->bodies[c->body].name);
		}
	}
	for (size_t j = 0; j < g->out.nholes; j++) {
		const struct hole *o = &g->out.holes[j];

		if (o->face == face &&
		    hypot(o->cx - c->x, o->cy - c->y) < o->rc + rc + EPS) {
			die("%s: the hole overlaps another one", c->where);
		}
	}
	PUSH(g->out.holes, g->out.nholes, g->out.capholes, h);
}

// Drops vertices without edges, left over from rounded corners.
static void drop_unused(struct smf *g)
{
	size_t *ren = xmalloc((g->out.nverts + 1) * sizeof(*ren));
	size_t n = 0;

	for (size_t i = 0; i < g->out.nverts; i++) {
		ren[i] = NONE;
	}
	for (size_t i = 0; i < g->out.nedges; i++) {
		ren[g->out.edges[i].v0] = 0;
		ren[g->out.edges[i].v1] = 0;
	}
	for (size_t i = 0; i < g->out.nverts; i++) {
		if (ren[i] != NONE) {
			g->out.verts[n] = g->out.verts[i];
			ren[i] = n++;
		}
	}
	g->out.nverts = n;
	for (size_t i = 0; i < g->out.nedges; i++) {
		g->out.edges[i].v0 = ren[g->out.edges[i].v0];
		g->out.edges[i].v1 = ren[g->out.edges[i].v1];
	}
	free(ren);
}

static void finish(struct smf *g)
{
	index_faces(&g->out);
	for (size_t k = 0; k < g->nrnds; k++) {
		if (g->rnds[k].at) {
			round_at(g, &g->rnds[k]);
		} else {
			round_flange(g, &g->rnds[k]);
		}
	}
	for (size_t k = 0; k < g->nolines; k++) {
		if (g->olines[k].trim) {
			place_trim(g, &g->olines[k]);
		}
	}
	for (size_t k = 0; k < g->nolines; k++) {
		if (!g->olines[k].trim) {
			place_outline(g, &g->olines[k]);
		}
	}
	for (size_t k = 0; k < g->ncircs; k++) {
		place_hole(g, &g->circs[k]);
	}
	drop_unused(g);
	for (size_t i = 0; i < g->nb; i++) {
		struct map m = body_map(&g->bodies[i], g->bodies[i].parent);

		PUSH(g->out.maps, g->out.nmaps, g->out.capmaps, m);
	}
}

static void print_name(const char *name, void *ctx)
{
	(void)ctx;
	printf("%s\n", name);
}

static void print_deps(struct smf *g)
{
	each_param(&g->nm, 1, print_name, NULL);
}

// Reads the options and returns the index of the part file, or -1.  -D
// NAME=EXPR pins a parameter over every definition in the files.
static int options(struct smf *g, int argc, char **argv, int *deps)
{
	int i = 1;
	char *eq = NULL;

	*deps = 0;
	for (; i < argc - 1; i++) {
		if (strcmp(argv[i], "-M") == 0) {
			*deps = 1;
			continue;
		}
		if (strcmp(argv[i], "-D") != 0 || i + 2 >= argc) {
			break;
		}
		eq = strchr(argv[++i], '=');
		if (!eq) {
			break;
		}
		*eq = 0;
		g->nm.where = "command line";
		pin(&g->nm, argv[i], eval(&g->nm, eq + 1));
	}
	return i == argc - 1 ? i : -1;
}

static void free_smf(struct smf *g)
{
	for (size_t i = 0; i < g->nb; i++) {
		free(g->bodies[i].name);
	}
	for (size_t i = 0; i < g->ncircs; i++) {
		free(g->circs[i].where);
	}
	for (size_t i = 0; i < g->nrnds; i++) {
		free(g->rnds[i].where);
	}
	for (size_t i = 0; i < g->nolines; i++) {
		free(g->olines[i].c);
		free(g->olines[i].where);
	}
	free(g->bodies);
	free(g->ops);
	free(g->circs);
	free(g->rnds);
	free(g->olines);
	free(g->ol.uv);
	free(g->ol.where);
	free(g->xs);
	free(g->ys);
	free(g->lab);
	free(g->fid);
	free(g->vid);
	clear_part(&g->out);
	free_names(&g->nm);
}

int main(int argc, char **argv)
{
	struct smf g;
	int deps = 0;
	int i = 0;

	memset(&g, 0, sizeof(g));
	i = options(&g, argc, argv, &deps);
	if (i < 0) {
		die("usage: smfold [-M] [-D NAME=EXPR]... part.sm");
	}
	g.part_path = argv[i];
	each_line(&g.nm, g.part_path, statement, &g);
	close_outline(&g);
	if (deps) {
		print_deps(&g);
		free_smf(&g);
		close_stdout();
		return 0;
	}
	if (!g.nb) {
		die("%s: no plate", g.part_path);
	}
	make_grid(&g);
	paint(&g);
	check_tears(&g);
	find_faces(&g);
	find_edges(&g);
	finish(&g);
	g.out.name = base_name(g.part_path);
	g.out.t = g.sheet.t;
	g.out.place = rig_id();
	check_contacts(&g);
	check_part(&g.out, g.part_path);
	write_fold(&g.out);
	free_smf(&g);
	close_stdout();
	return 0;
}
