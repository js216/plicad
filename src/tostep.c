// SPDX-License-Identifier: GPL-3.0-only
// tostep.c -- Write parts as a STEP AP214 file of exact solids
// Copyright (c) 2026 Jakob Kastelic

// Usage: tostep STREAM|- > part.step
//
// Every part of the stream (stream.h) becomes a manifold solid in one
// shape representation, one solid for each shell; a shell that is a void
// is refused, and so is any source line no tool of a pipe took.  Each
// part is in world coordinates, in mm, at its place.  A part with a
// material gets an AP214 surface style in its color.  The faces are
// planes, cylinders, cones, spheres and tori, and pipes, which no STEP
// surface is: each goes out as a rational B-spline surface that is the
// pipe over its face, exactly (pipe_surf).  An ellipse, and an arc of
// a circle on a sphere, go out as several edges on the one curve
// (pieces_of).  A nodal section of a torus, which no STEP curve is, goes
// out as edges on cubic B-splines, which meet the curve and its tangents
// at their ends and keep within about 1e-10 mm of it between: their
// misfit grows with the tube of the torus and the fourth power of the
// piece, which is NODAL_PIECE over the fourth root of the tube in mm.
// Its faces stay exact.

#include "brep.h"
#include "bsolid.h"
#include "fold.h"
#include "geom.h"
#include "integ.h"
#include "isect.h"
#include "mesh.h"
#include "shape.h"
#include "stream.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
	REAL_LEN = 40, // Room for the text of one real.
	HEAD_IDS = 13 // The entities of the head, #1 to #13.
};
#define ZERO 1e-13 // Reals closer to 0 than this are 0.
#define ARC_MAX (PI / 16) // The longest piece of an edge OCCT must fit.
#define NODAL_PIECE (PI / 256) // A piece of a nodal section of a 1 mm tube.

// Prints a STEP real, which always has a decimal point.
static void real(double x)
{
	char b[REAL_LEN];
	char *e = NULL;
	double v = fabs(x) < ZERO ? 0.0 : x;

	xsnprintf(b, "%.12G", v);
	e = strchr(b, 'E');
	if (strchr(b, '.')) {
		printf("%s", b);
	} else if (e) {
		char tail[REAL_LEN];

		xsnprintf(tail, "%s", e);
		*e = 0;
		printf("%s.%s", b, tail);
	} else {
		printf("%s.", b);
	}
}

// Prints s as a STEP string: in quotes, with each quote doubled.
static void string(const char *s)
{
	printf("'");
	for (; *s; s++) {
		if (*s == '\'') {
			printf("'");
		}
		printf("%c", *s);
	}
	printf("'");
}

static unsigned long triple(unsigned long *id, const char *kind, struct vec p)
{
	printf("#%lu=%s('',(", ++*id, kind);
	real(p.x);
	printf(",");
	real(p.y);
	printf(",");
	real(p.z);
	printf("));\n");
	return *id;
}

// A placement: origin o, axis z and reference direction x.
struct axes {
	struct vec o, z, x;
};

static unsigned long placement(unsigned long *id, struct axes ax)
{
	unsigned long a = triple(id, "CARTESIAN_POINT", ax.o);
	unsigned long b = triple(id, "DIRECTION", unit(ax.z));
	unsigned long c = triple(id, "DIRECTION", unit(ax.x));

	printf("#%lu=AXIS2_PLACEMENT_3D('',#%lu,#%lu,#%lu);\n", ++*id, a, b, c);
	return *id;
}

static unsigned long line(unsigned long *id, const struct brep *b,
			  const struct bedge *e)
{
	struct vec d = sub(b->v[e->v1], b->v[e->v0]);
	unsigned long pt = triple(id, "CARTESIAN_POINT", b->v[e->v0]);
	unsigned long dir = triple(id, "DIRECTION", unit(d));

	printf("#%lu=VECTOR('',#%lu,", ++*id, dir);
	real(len(d));
	printf(");\n#%lu=LINE('',#%lu,#%lu);\n", *id + 1, pt, *id);
	return ++*id;
}

static unsigned long curve(unsigned long *id, const struct brep *b,
			   const struct bedge *e)
{
	struct axes pl;
	unsigned long ax = 0;

	if (e->type == C_LINE) {
		return line(id, b, e);
	}
	pl.o = e->c;
	pl.z = e->n;
	pl.x = sub(b->v[e->v0], e->c);
	ax = placement(id, pl);
	printf("#%lu=CIRCLE('',#%lu,", ++*id, ax);
	real(e->r);
	printf(");\n");
	return *id;
}

static unsigned long loop(unsigned long *id, const struct brep *b,
			  const struct bloop *lp, const unsigned long *eid)
{
	unsigned long first = *id + 1;

	for (size_t k = 0; k < lp->n; k++) {
		const struct buse *u = &b->u[lp->first + k];

		printf("#%lu=ORIENTED_EDGE('',*,*,#%lu,.%c.);\n", ++*id,
		       eid[u->e], u->fwd ? 'T' : 'F');
	}
	printf("#%lu=EDGE_LOOP('',(", ++*id);
	for (unsigned long k = 0; k < lp->n; k++) {
		printf("%s#%lu", k ? "," : "", first + k);
	}
	printf("));\n#%lu=FACE_BOUND('',#%lu,.T.);\n", *id + 1, *id);
	return ++*id;
}

static unsigned long face(unsigned long *id, const struct brep *b,
			  const struct bface *f, const unsigned long *eid)
{
	struct axes pl;
	unsigned long *bounds = xmalloc((f->nloops + 1) * sizeof(*bounds));
	unsigned long ax = 0;
	unsigned long s = 0;

	for (size_t l = 0; l < f->nloops; l++) {
		bounds[l] = loop(id, b, &b->l[f->loop0 + l], eid);
	}
	pl.o = f->o;
	pl.z = f->n;
	pl.x = f->x;
	ax = placement(id, pl);
	s = ++*id;
	if (f->type == S_PLANE) {
		printf("#%lu=PLANE('',#%lu);\n", s, ax);
	} else if (f->type == S_CYLINDER) {
		printf("#%lu=CYLINDRICAL_SURFACE('',#%lu,", s, ax);
		real(f->r);
		printf(");\n");
	} else {
		printf("#%lu=CONICAL_SURFACE('',#%lu,", s, ax);
		real(f->r);
		printf(",");
		real(atan(f->r2));
		printf(");\n");
	}
	printf("#%lu=ADVANCED_FACE('',(", ++*id);
	for (size_t l = 0; l < f->nloops; l++) {
		printf("%s#%lu", l ? "," : "", bounds[l]);
	}
	printf("),#%lu,.%c.);\n", s, f->sense > 0 ? 'T' : 'F');
	free(bounds);
	return *id;
}

// Prints the vertices and edges of b; eid[i] gets the id of edge i.
static void edges(unsigned long *id, const struct brep *b, unsigned long *eid)
{
	unsigned long *vid = xmalloc((b->nv + 1) * sizeof(*vid));

	for (size_t i = 0; i < b->nv; i++) {
		unsigned long pt = triple(id, "CARTESIAN_POINT", b->v[i]);

		printf("#%lu=VERTEX_POINT('',#%lu);\n", ++*id, pt);
		vid[i] = *id;
	}
	for (size_t i = 0; i < b->ne; i++) {
		unsigned long c = curve(id, b, &b->e[i]);

		printf("#%lu=EDGE_CURVE('',#%lu,#%lu,#%lu,.T.);\n", ++*id,
		       vid[b->e[i].v0], vid[b->e[i].v1], c);
		eid[i] = *id;
	}
	free(vid);
}

// A solid written, and its part.
struct solid_ref {
	unsigned long id;
	size_t part;
};

// The solids written so far.
struct solids {
	struct solid_ref *s;
	size_t n, cap;
};

// The faces of a brep: the shell of each, and the id it has in the file.
struct face_ids {
	const size_t *shell;
	const unsigned long *fid;
	size_t n;
};

// Prints the closed shell of the faces fs of shell sh, and returns its
// id.
static unsigned long shell_solid(unsigned long *id, const struct face_ids *fs,
				 size_t sh)
{
	int first = 1;

	printf("#%lu=CLOSED_SHELL('',(", ++*id);
	for (size_t i = 0; i < fs->n; i++) {
		if (fs->shell[i] == sh) {
			printf("%s#%lu", first ? "" : ",", fs->fid[i]);
			first = 0;
		}
	}
	printf("));\n");
	return *id;
}

static void shape_solid(unsigned long *id, const struct shape *sh,
			struct solids *sd, size_t part);

// Prints part i of ps as one solid for each of its pieces, and adds them
// to sd.
static void solid(unsigned long *id, const struct part *ps, size_t i,
		  struct solids *sd)
{
	struct brep b;
	unsigned long *eid = NULL;
	unsigned long *fid = NULL;
	size_t *shell = NULL;
	size_t n = 0;
	struct face_ids fs;

	if (ps[i].solid) {
		struct shape sh;

		part_shape(&ps[i], &sh);
		shape_solid(id, &sh, sd, i);
		shape_free(&sh);
		return;
	}
	brep_build(&ps[i], &b);
	eid = xmalloc((b.ne + 1) * sizeof(*eid));
	fid = xmalloc((b.nf + 1) * sizeof(*fid));
	shell = xmalloc((b.nf + 1) * sizeof(*shell));
	n = brep_shells(&b, shell);
	edges(id, &b, eid);
	for (size_t f = 0; f < b.nf; f++) {
		fid[f] = face(id, &b, &b.f[f], eid);
	}
	fs.shell = shell;
	fs.fid = fid;
	fs.n = b.nf;
	for (size_t k = 0; k < n; k++) {
		unsigned long sh = shell_solid(id, &fs, k);
		struct solid_ref r = { 0, i };

		printf("#%lu=MANIFOLD_SOLID_BREP(", ++*id);
		string(ps[i].name);
		printf(",#%lu);\n", sh);
		r.id = *id;
		PUSH(sd->s, sd->n, sd->cap, r);
	}
	free(eid);
	free(fid);
	free(shell);
	brep_free(&b);
}

// Prints the AP214 style of solid s in the color of part p, and returns
// the id of its STYLED_ITEM.
static unsigned long style(unsigned long *id, const struct part *p,
			   unsigned long s)
{
	printf("#%lu=COLOUR_RGB(", ++*id);
	string(p->base);
	for (int i = 0; i < 3; i++) {
		printf(",");
		real(p->rgb[i]);
	}
	printf(");\n#%lu=FILL_AREA_STYLE_COLOUR('',#%lu);\n", *id + 1, *id);
	(*id)++;
	printf("#%lu=FILL_AREA_STYLE('',(#%lu));\n", *id + 1, *id);
	(*id)++;
	printf("#%lu=SURFACE_STYLE_FILL_AREA(#%lu);\n", *id + 1, *id);
	(*id)++;
	printf("#%lu=SURFACE_SIDE_STYLE('',(#%lu));\n", *id + 1, *id);
	(*id)++;
	printf("#%lu=SURFACE_STYLE_USAGE(.BOTH.,#%lu);\n", *id + 1, *id);
	(*id)++;
	printf("#%lu=PRESENTATION_STYLE_ASSIGNMENT((#%lu));\n", *id + 1, *id);
	(*id)++;
	printf("#%lu=STYLED_ITEM('color',(#%lu),#%lu);\n", *id + 1, *id, s);
	return ++*id;
}

static void header(unsigned long *id, const char *name)
{
	printf("ISO-10303-21;\nHEADER;\n"
	       "FILE_DESCRIPTION(('folded sheet metal'),'2;1');\n"
	       "FILE_NAME(");
	string(name);
	printf(",'',(''),(''),'tostep','tostep','');\n"
	       "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));"
	       "\nENDSEC;\nDATA;\n"
	       "#1=APPLICATION_CONTEXT('automotive design');\n"
	       "#2=APPLICATION_PROTOCOL_DEFINITION('international standard',"
	       "'automotive_design',2000,#1);\n"
	       "#3=PRODUCT_CONTEXT('',#1,'mechanical');\n"
	       "#4=PRODUCT(");
	string(name);
	printf(",");
	string(name);
	printf(",'',(#3));\n"
	       "#5=PRODUCT_DEFINITION_FORMATION('','',#4);\n"
	       "#6=PRODUCT_DEFINITION_CONTEXT('part definition',#1,'design');\n"
	       "#7=PRODUCT_DEFINITION('design','',#5,#6);\n"
	       "#8=PRODUCT_DEFINITION_SHAPE('','',#7);\n"
	       "#9=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));\n"
	       "#10=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));\n"
	       "#11=(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT());\n"
	       "#12=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#9,"
	       "'distance_accuracy_value','');\n"
	       "#13=(GEOMETRIC_REPRESENTATION_CONTEXT(3)"
	       "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#12))"
	       "GLOBAL_UNIT_ASSIGNED_CONTEXT((#9,#10,#11))"
	       "REPRESENTATION_CONTEXT('',''));\n");
	*id = HEAD_IDS;
}

// Prints the colors of the parts that have one, as one presentation.
static void styles(unsigned long *id, const struct part *ps,
		   const struct solids *sd)
{
	unsigned long *items = xmalloc((sd->n + 1) * sizeof(*items));
	size_t k = 0;

	for (size_t i = 0; i < sd->n; i++) {
		const struct part *p = &ps[sd->s[i].part];

		if (p->base) {
			items[k++] = style(id, p, sd->s[i].id);
		}
	}
	if (k) {
		printf("#%lu=MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_"
		       "REPRESENTATION('',(",
		       ++*id);
		for (size_t i = 0; i < k; i++) {
			printf("%s#%lu", i ? "," : "", items[i]);
		}
		printf("),#13);\n");
	}
	free(items);
}

// Prints curve c as a STEP curve, and returns its id.
static unsigned long shape_curve(unsigned long *id, const struct curve *c)
{
	static const char *const conic[CV_KINDS] = { "", "CIRCLE", "ELLIPSE" };
	struct axes pl = { c->o, c->n, c->x };
	unsigned long ax = 0;

	if (c->kind == CV_LINE) {
		unsigned long pt = triple(id, "CARTESIAN_POINT", c->o);
		unsigned long dir = triple(id, "DIRECTION", c->x);

		printf("#%lu=VECTOR('',#%lu,1.);\n#%lu=LINE('',#%lu,#%lu);\n",
		       *id + 1, dir, *id + 2, pt, *id + 1);
		*id += 2;
		return *id;
	}
	ax = placement(id, pl);
	printf("#%lu=%s('',#%lu,", ++*id, conic[c->kind], ax);
	real(c->a);
	if (c->kind == CV_ELLIPSE) {
		printf(",");
		real(c->b);
	}
	printf(");\n");
	return *id;
}

// Prints surface s as a STEP surface, and returns its id.  A cone that
// narrows along its axis turns round, for a positive half angle.
static unsigned long shape_surf(unsigned long *id, const struct surf *s)
{
	static const char *const names[SF_KINDS] = { "PLANE",
						     "CYLINDRICAL_SURFACE",
						     "CONICAL_SURFACE",
						     "SPHERICAL_SURFACE",
						     "TOROIDAL_SURFACE",
						     "" };
	struct axes pl = { s->o, s->n, s->x };
	unsigned long ax = 0;

	if (s->kind == SF_CONE && s->r2 < 0) {
		pl.z = mul(s->n, -1);
	}
	ax = placement(id, pl);
	printf("#%lu=%s('',#%lu", ++*id, names[s->kind], ax);
	if (s->kind != SF_PLANE) {
		printf(",");
		real(s->r);
	}
	if (s->kind == SF_CONE) {
		printf(",");
		real(atan(fabs(s->r2)));
	} else if (s->kind == SF_TORUS) {
		printf(",");
		real(s->r2);
	}
	printf(");\n");
	return *id;
}

// A pipe goes out as a rational B-spline surface that holds its face
// exactly.  Over a piece of the ellipse from u0 to u0 + du, a rational
// quadratic Bezier curve is the ellipse, c(s) = N(s) / W(s), with the
// weights 1, cos(du / 2), 1, and its tangent runs along the quadratic
// D(s) = N' W - N W'.  The ring of radius rt about c(s) square to D is
//   q(s, g) = ((g^2 - |D|^2) m + 2 g (m x D)) / (g^2 + |D|^2),
// m the unit normal n of the ellipse or its opposite, which runs once
// round the ring as g runs from -inf to inf, from m at both ends; the
// angle f from -m toward m x D has g = |D| tan(f / 2).  So
//   S(s, g) = (N (g^2 + |D|^2) + rt W ((g^2 - |D|^2) m + 2 g (m x D)))
//             / (W (g^2 + |D|^2))
// is of degree 6 in s and 2 in g, and the pieces of equal du join, as D
// is the same at their ends.  m is the one whose end of g the face lies
// farthest from, and g spans the face, with room to spare, from where
// it is least to where it is greatest, split at 0 so that no weight
// falls below 0.
enum {
	PIPE_DEG = 6, // The degree of a pipe in u.
	PIPE_ROWS = PIPE_DEG + 1,
	PIPE_PTS = 32, // The points of each edge of a face of a pipe.
	PIPE_PIECES = 64, // The most pieces of a pipe along its ellipse.
	NET_ROWS = (PIPE_PIECES * PIPE_DEG) + 1, // The most rows of its net.
	NET_COLS = 5 // The most columns: two pieces of degree 2.
};
#define PIPE_PIECE (PI / 4) // The longest piece of a pipe along u.
#define PIPE_PAD 0.05 // The room along u past the face, in radians.
#define RING_PAD 0.1 // The room round the ring past the face, in radians.
#define SPEED_PAD 0.05 // The share of room on the speed of the pieces.

static double binom(int n, int k)
{
	double c = 1;

	for (int i = 1; i <= k; i++) {
		c = c * (n - k + i) / i;
	}
	return c;
}

// The product of the Bernstein forms f, of degree m, and g, of degree n,
// with dim numbers each, f a scalar if fdim is 1, into h of degree m + n.
struct bern {
	double c[PIPE_ROWS][4];
	int deg, dim;
};

static struct bern bern_mul(const struct bern *f, const struct bern *g)
{
	struct bern h;
	int dim = f->dim > g->dim ? f->dim : g->dim;

	memset(&h, 0, sizeof(h));
	h.deg = f->deg + g->deg;
	h.dim = dim;
	for (int i = 0; i <= f->deg; i++) {
		for (int j = 0; j <= g->deg && i + j < PIPE_ROWS; j++) {
			double w = binom(f->deg, i) * binom(g->deg, j) /
				   binom(h.deg, i + j);

			for (int k = 0; k < dim; k++) {
				double a = f->c[i][f->dim == 1 ? 0 : k];
				double b = g->c[j][g->dim == 1 ? 0 : k];

				h.c[i + j][k] += w * a * b;
			}
		}
	}
	return h;
}

static struct bern bern_sum(struct bern f, const struct bern *g, double k)
{
	struct bern h = f;

	for (int i = 0; i <= f.deg; i++) {
		for (int j = 0; j < f.dim; j++) {
			h.c[i][j] += k * g->c[i][j];
		}
	}
	return h;
}

// The Bernstein form of degree d, all of whose numbers are 1: a way to
// raise the degree of another by d in a product.
static struct bern bern_one(int d)
{
	struct bern h;

	memset(&h, 0, sizeof(h));
	h.deg = d;
	h.dim = 1;
	for (int i = 0; i <= d; i++) {
		h.c[i][0] = 1;
	}
	return h;
}

static struct vec bvec(const struct bern *f, int i)
{
	return v3(f->c[i][0], f->c[i][1], f->c[i][2]);
}

static void set_bvec(struct bern *f, int i, struct vec v)
{
	f->c[i][0] = v.x;
	f->c[i][1] = v.y;
	f->c[i][2] = v.z;
}

// A piece of the ellipse of pipe s from u0 to u0 + du: N (with its
// weight as the fourth number), W and D.
struct spine {
	struct bern n, w, d;
};

static struct spine spine_piece(const struct surf *s, double u0, double du)
{
	struct vec y = frame_y(s->n, s->x);
	double h = cos(du / 2);
	const double wt[3] = { 1, h, 1 };
	const double ang[3] = { u0, u0 + (du / 2), u0 + du };
	struct vec p[3];
	struct spine sp;

	memset(&sp, 0, sizeof(sp));
	sp.n.deg = sp.w.deg = sp.d.deg = 2;
	sp.n.dim = 4;
	sp.w.dim = 1;
	sp.d.dim = 3;
	for (int k = 0; k < 3; k++) {
		double r = k == 1 ? 1 / h : 1;

		p[k] = add(s->o, add(mul(s->x, s->r * r * cos(ang[k])),
				     mul(y, s->r3 * r * sin(ang[k]))));
		set_bvec(&sp.n, k, mul(p[k], wt[k]));
		sp.n.c[k][3] = wt[k];
		sp.w.c[k][0] = wt[k];
	}
	set_bvec(&sp.d, 0, mul(sub(p[1], p[0]), 2 * wt[0] * wt[1]));
	set_bvec(&sp.d, 1, mul(sub(p[2], p[0]), wt[0] * wt[2]));
	set_bvec(&sp.d, 2, mul(sub(p[2], p[1]), 2 * wt[1] * wt[2]));
	return sp;
}

// The value of the Bernstein form f at t, its first three numbers.
static struct vec bern_at(const struct bern *f, double t)
{
	struct vec v = v3(0, 0, 0);

	for (int i = 0; i <= f->deg; i++) {
		double b =
			binom(f->deg, i) * pow(t, i) * pow(1 - t, f->deg - i);

		v = add(v, mul(bvec(f, i), b));
	}
	return v;
}

// The homogeneous points of the piece sp of pipe s, its ring from m,
// for g from g0 to g1: row i of degree 6 in s, column j of degree 2.
static void pipe_patch(const struct surf *s, const struct spine *sp,
		       struct vec m, struct span g, struct bern *col)
{
	struct bern dd = bern_mul(&sp->d, &sp->d);
	struct bern md;
	struct bern one2 = bern_one(2);
	const double g2[3] = { g.lo * g.lo, g.lo * g.hi, g.hi * g.hi };
	const double g1[3] = { g.lo, (g.lo + g.hi) / 2, g.hi };

	// dd = D . D: bern_mul gives the products of the numbers one by
	// one; their sum is the dot.
	for (int i = 0; i <= dd.deg; i++) {
		dd.c[i][0] += dd.c[i][1] + dd.c[i][2];
		dd.c[i][1] = dd.c[i][2] = 0;
	}
	dd.dim = 1;
	md = sp->d;
	for (int i = 0; i <= md.deg; i++) {
		set_bvec(&md, i, cross(m, bvec(&sp->d, i)));
	}
	md = bern_mul(&md, &one2);
	for (int j = 0; j < 3; j++) {
		struct bern den;
		struct bern num;
		struct bern a;
		struct bern b;

		memset(&den, 0, sizeof(den));
		memset(&num, 0, sizeof(num));
		den.deg = num.deg = dd.deg;
		den.dim = 1;
		num.dim = 3;
		for (int i = 0; i <= dd.deg; i++) {
			den.c[i][0] = g2[j] + dd.c[i][0];
			set_bvec(&num, i,
				 add(mul(m, g2[j] - dd.c[i][0]),
				     mul(bvec(&md, i), 2 * g1[j])));
		}
		a = bern_mul(&sp->n, &den); // N den, and W den last.
		b = bern_mul(&sp->w, &num);
		col[j] = bern_sum(a, &b, s->r2);
		col[j].dim = 4;
	}
}

// A point of a pipe by its ring: the parameter u of the ring and the
// angle f round it.
struct ring_pt {
	double u, f;
};

// The parameter u near ref and the angle f round the ring from -m of
// the point p of pipe s.
static struct ring_pt ring_angle(const struct surf *s, struct vec p, double ref,
				 struct vec m)
{
	struct vec y = frame_y(s->n, s->x);
	struct line3 c;
	struct vec t;
	struct vec q;
	struct ring_pt r = { 0, 0 };

	r.u = angle_near(pipe_foot(s, p), ref);
	c = pipe_ring(s, r.u);
	t = unit(add(mul(s->x, -s->r * sin(r.u)), mul(y, s->r3 * cos(r.u))));
	q = sub(p, c.o);
	r.f = atan2(dot(q, cross(m, t)), -dot(q, m));
	return r;
}

// The reach of face fi of pipe sh along u and round its ring, from
// either end, at the points of its edges.
struct pipe_reach {
	struct span u, f[2];
};

static struct pipe_reach face_reach_pipe(const struct shape *sh, size_t fi)
{
	const struct surf *s = &sh->f[fi].s;
	struct facemap fm;
	struct urange r = face_uses(sh, fi);
	struct pipe_reach x = { { HUGE_VAL, -HUGE_VAL },
				{ { HUGE_VAL, -HUGE_VAL },
				  { HUGE_VAL, -HUGE_VAL } } };

	facemap_init(&fm, sh, fi);
	for (size_t k = r.first; k < r.end; k++) {
		const struct sedge *e = &sh->e[sh->u[k].e];

		for (int i = 0; i <= PIPE_PTS; i++) {
			struct vec p = edge_at(e, i / (double)PIPE_PTS);

			for (int side = 0; side < 2; side++) {
				struct ring_pt rp =
					ring_angle(s, p, fm.ref.u,
						   mul(s->n, side ? -1 : 1));
				double u = rp.u;
				double f = rp.f;
				x.u.lo = fmin(x.u.lo, u);
				x.u.hi = fmax(x.u.hi, u);
				x.f[side].lo = fmin(x.f[side].lo, f);
				x.f[side].hi = fmax(x.f[side].hi, f);
			}
		}
	}
	return x;
}

// The rows and columns of a net of control points.
struct net_size {
	size_t rows, cols;
};

// Prints the control points of the net and returns the id of the first;
// row i holds those of the s of point i, their weights in w.
static unsigned long net_points(unsigned long *id, double (*net)[4],
				struct net_size sz, double *w, const char *name)
{
	unsigned long first = *id + 1;
	size_t rows = sz.rows;
	size_t cols = sz.cols;

	for (size_t i = 0; i < rows; i++) {
		for (size_t j = 0; j < cols; j++) {
			const double *h = net[(j * NET_ROWS) + i];

			w[(i * cols) + j] = h[3];
			if (!(h[3] > 0)) {
				die("tostep: the pipe of face %s has a weight "
				    "not above 0",
				    name);
			}
			(void)triple(id, "CARTESIAN_POINT",
				     v3(h[0] / h[3], h[1] / h[3], h[2] / h[3]));
		}
	}
	return first;
}

// Prints the list of n knots k.
static void knot_list(const double *k, size_t n)
{
	printf("(");
	for (size_t i = 0; i < n; i++) {
		printf("%s", i ? "," : "");
		real(k[i]);
	}
	printf(")");
}

// Prints the list of n multiplicities k, whole numbers.
static void mult_list(const double *k, size_t n)
{
	printf("(");
	for (size_t i = 0; i < n; i++) {
		printf("%s%d", i ? "," : "", (int)k[i]);
	}
	printf(")");
}

// The net of a pipe face: its span of u and pieces along it, each du
// long; the way m the ring's angle is measured from; the span g of the
// tangent of half the angle, in ng pieces, cut between them; the rows
// and columns of control points.
struct pipe_net {
	struct span u;
	size_t np;
	double du;
	struct vec m;
	struct span g;
	size_t ng;
	double cut;
	struct net_size sz;
};

// The least and the most speed of the spine of pipe s along the pieces
// of pn, padded.
static struct span spine_speeds(const struct surf *s, const struct pipe_net *pn)
{
	struct span sp = { HUGE_VAL, 0 };

	for (size_t i = 0; i < pn->np; i++) {
		struct spine pc =
			spine_piece(s, pn->u.lo + (pn->du * (double)i), pn->du);

		for (int k = 0; k <= PIPE_PTS; k++) {
			double l = len(bern_at(&pc.d, k / (double)PIPE_PTS));

			sp.lo = fmin(sp.lo, l);
			sp.hi = fmax(sp.hi, l);
		}
	}
	sp.lo *= 1 - SPEED_PAD;
	sp.hi *= 1 + SPEED_PAD;
	return sp;
}

// The control points of the net pn of pipe s into net.
static void fill_net(const struct surf *s, const struct pipe_net *pn,
		     double (*net)[4])
{
	for (size_t i = 0; i < pn->np; i++) {
		struct spine sp =
			spine_piece(s, pn->u.lo + (pn->du * (double)i), pn->du);

		for (size_t k = 0; k < pn->ng; k++) {
			struct span gk = { k ? 0 : pn->g.lo,
					   pn->ng == 2 && !k ? 0 : pn->g.hi };
			struct bern col[3];

			pipe_patch(s, &sp, pn->m, gk, col);
			for (size_t j = 0; j < 3; j++) {
				for (size_t r = 0; r <= PIPE_DEG; r++) {
					size_t at = (((2 * k) + j) * NET_ROWS) +
						    (PIPE_DEG * i) + r;

					memcpy(net[at], col[j].c[r],
					       sizeof(net[at]));
				}
			}
		}
	}
}

// Prints the surface of net pn, its points from first, their weights
// w, and returns its id.
static unsigned long print_pipe(unsigned long *id, const struct pipe_net *pn,
				unsigned long first, const double *w)
{
	size_t np = pn->np;
	size_t ng = pn->ng;
	size_t rows = pn->sz.rows;
	size_t cols = pn->sz.cols;
	double uk[PIPE_PIECES + 1];
	double um[PIPE_PIECES + 1];
	double vk[3] = { 0, ng == 2 ? pn->cut : 1, 1 };
	double vm[3] = { 3, ng == 2 ? 2 : 3, 3 };

	for (size_t i = 0; i <= np; i++) {
		uk[i] = (double)i;
		um[i] = i == 0 || i == np ? PIPE_DEG + 1 : PIPE_DEG;
	}
	printf("#%lu=(BOUNDED_SURFACE()B_SPLINE_SURFACE(%d,2,(", ++*id,
	       PIPE_DEG);
	for (size_t i = 0; i < rows; i++) {
		printf("%s(", i ? "," : "");
		for (size_t j = 0; j < cols; j++) {
			printf("%s#%lu", j ? "," : "", first + (i * cols) + j);
		}
		printf(")");
	}
	printf("),.UNSPECIFIED.,.F.,.F.,.F.)B_SPLINE_SURFACE_WITH_KNOTS(");
	mult_list(um, np + 1);
	printf(",");
	mult_list(vm, ng + 1);
	printf(",");
	knot_list(uk, np + 1);
	printf(",");
	knot_list(vk, ng == 2 ? 3 : 2);
	printf(",.UNSPECIFIED.)GEOMETRIC_REPRESENTATION_ITEM()"
	       "RATIONAL_B_SPLINE_SURFACE((");
	for (size_t i = 0; i < rows; i++) {
		printf("%s(", i ? "," : "");
		for (size_t j = 0; j < cols; j++) {
			printf("%s", j ? "," : "");
			real(w[(i * cols) + j]);
		}
		printf(")");
	}
	printf("))REPRESENTATION_ITEM('')SURFACE());\n");
	return *id;
}

// The pieces of span u: as many as PIPE_PIECE long, 1 to PIPE_PIECES.
static size_t pipe_pieces(struct span u)
{
	double du = ceil((u.hi - u.lo) / PIPE_PIECE);

	if (du < 1) {
		return 1;
	}
	return du > PIPE_PIECES ? PIPE_PIECES : (size_t)du;
}

// The net of face fc of pipe s, of reach x, into pn.
static void pipe_net_of(const struct sface *fc, const struct pipe_reach *x,
			struct pipe_net *pn)
{
	const struct surf *s = &fc->s;
	int side = fmax(fabs(x->f[1].lo), fabs(x->f[1].hi)) <
		   fmax(fabs(x->f[0].lo), fabs(x->f[0].hi));
	struct span f = x->f[side];
	struct span sp = { 0, 0 };

	pn->u.lo = x->u.lo - PIPE_PAD;
	pn->u.hi = x->u.hi + PIPE_PAD;
	pn->m = mul(s->n, side ? -1 : 1);
	if (fmax(fabs(f.lo), fabs(f.hi)) > PI - (2 * RING_PAD)) {
		die("tostep: face %s of a pipe runs round its tube", fc->name);
	}
	if (pn->u.hi - pn->u.lo > 2 * PI) {
		pn->u.lo = ((x->u.lo + x->u.hi) / 2) - PI;
		pn->u.hi = pn->u.lo + (2 * PI);
	}
	pn->np = pipe_pieces(pn->u);
	pn->du = (pn->u.hi - pn->u.lo) / (double)pn->np;
	sp = spine_speeds(s, pn);
	f.lo = fmax(f.lo - RING_PAD, -PI + RING_PAD);
	f.hi = fmin(f.hi + RING_PAD, PI - RING_PAD);
	pn->g.lo = (f.lo < 0 ? sp.hi : sp.lo) * tan(f.lo / 2);
	pn->g.hi = (f.hi > 0 ? sp.hi : sp.lo) * tan(f.hi / 2);
	pn->ng = pn->g.lo < 0 && pn->g.hi > 0 ? 2 : 1;
	pn->cut = pn->ng == 2 ? -pn->g.lo / (pn->g.hi - pn->g.lo) : 1;
	pn->sz.rows = (PIPE_DEG * pn->np) + 1;
	pn->sz.cols = (2 * pn->ng) + 1;
}

// Prints the surface of face fi of sh, a pipe, as a rational B-spline
// surface that holds the face, and returns its id; *same gets whether
// its normal is the outward one of the face.  Its normal, s x g, runs
// toward the center of the ring, against that of the pipe.
static unsigned long pipe_surf(unsigned long *id, const struct shape *sh,
			       size_t fi, int *same)
{
	const struct sface *fc = &sh->f[fi];
	struct pipe_reach x = face_reach_pipe(sh, fi);
	struct pipe_net pn;
	double(*net)[4] = NULL;
	double *w = NULL;
	unsigned long first = 0;

	pipe_net_of(fc, &x, &pn);
	net = xcalloc((size_t)NET_ROWS * NET_COLS, sizeof(*net));
	w = xmalloc(((pn.sz.rows * pn.sz.cols) + 1) * sizeof(*w));
	fill_net(&fc->s, &pn, net);
	first = net_points(id, net, pn.sz, w, fc->name);
	print_pipe(id, &pn, first, w);
	*same = fc->sense < 0;
	free((void *)net);
	free(w);
	return *id;
}

// The edge curves of an edge of a solid in the file: n of them, with
// ids from first on, in the way of the edge.
struct edge_ids {
	unsigned long first, n;
};

// The number of pieces that edge e of sh goes out in.  OCCT 7.8 knows
// the exact curve in the parameters of a face for a line, for a circle
// on a plane, a cylinder, a cone or a torus, and for a conic on a plane.
// For an ellipse on a cylinder (where two rounds end on each other) and
// for an arc of a circle on a sphere it fits a B-spline to the whole
// edge, up to 4e-6 mm off for a quarter turn, and leaves the tolerance
// of the edge at 1e-7 mm, so that its own check finds the edge off the
// face.  The misfit falls with about the fourth power of the length:
// in pieces of ARC_MAX at most it stays near 1e-9 mm.  The curve and
// its ends are the same; only the file has more edges.
static unsigned long pieces_of(const struct shape *sh, size_t e,
			       const int *on_sphere)
{
	const struct sedge *ed = &sh->e[e];
	double n = 1;

	if (ed->c.kind == CV_ELLIPSE ||
	    (ed->c.kind == CV_CIRCLE && on_sphere[e])) {
		n = ceil((ed->t1 - ed->t0) / ARC_MAX);
	}
	if (ed->c.kind == CV_NODAL) {
		n = ceil((ed->t1 - ed->t0) * sqrt(sqrt(fmax(1, ed->c.b))) /
			 NODAL_PIECE);
	}
	return n < 1 ? 1 : (unsigned long)n;
}

// Prints the piece of nodal section c from t0 to t1 as a cubic Bezier
// curve, a B-spline, and returns its id: its ends and the tangents there
// are those of c.
static unsigned long nodal_piece(unsigned long *id, const struct curve *c,
				 double t0, double t1)
{
	double h = t1 - t0;
	struct vec p[4];
	unsigned long pid[4];

	p[0] = curve_at(c, t0);
	p[3] = curve_at(c, t1);
	p[1] = add(p[0], mul(curve_tan(c, t0), h / 3));
	p[2] = sub(p[3], mul(curve_tan(c, t1), h / 3));
	for (int k = 0; k < 4; k++) {
		pid[k] = triple(id, "CARTESIAN_POINT", p[k]);
	}
	printf("#%lu=B_SPLINE_CURVE_WITH_KNOTS('',3,(#%lu,#%lu,#%lu,#%lu),"
	       ".UNSPECIFIED.,.F.,.F.,(4,4),(0.,1.),.UNSPECIFIED.);\n",
	       ++*id, pid[0], pid[1], pid[2], pid[3]);
	return *id;
}

// Prints edge e of sh as its pieces, with new vertices between them,
// and returns their ids; vid holds the ids of the vertices.
static struct edge_ids shape_edge_out(unsigned long *id, const struct sedge *ed,
				      unsigned long n, const unsigned long *vid)
{
	unsigned long c = ed->c.kind == CV_NODAL ? 0 : shape_curve(id, &ed->c);
	unsigned long *cid = xmalloc((n + 1) * sizeof(*cid));
	unsigned long prev = vid[ed->v0];
	struct edge_ids r = { 0, n };
	unsigned long mid0 = *id + 1;

	for (unsigned long j = 1; j < n; j++) {
		double t = ed->t0 + ((ed->t1 - ed->t0) * (double)j / (double)n);
		unsigned long pt =
			triple(id, "CARTESIAN_POINT", curve_at(&ed->c, t));

		printf("#%lu=VERTEX_POINT('',#%lu);\n", ++*id, pt);
	}
	for (unsigned long j = 0; j < n && ed->c.kind == CV_NODAL; j++) {
		double h = (ed->t1 - ed->t0) / (double)n;

		cid[j] = nodal_piece(id, &ed->c, ed->t0 + (h * (double)j),
				     ed->t0 + (h * (double)(j + 1)));
	}
	r.first = *id + 1;
	for (unsigned long j = 0; j < n; j++) {
		unsigned long next = j + 1 < n ? mid0 + (2 * j) + 1 :
						 vid[ed->v1];

		printf("#%lu=EDGE_CURVE('',#%lu,#%lu,#%lu,.T.);\n", ++*id, prev,
		       next, ed->c.kind == CV_NODAL ? cid[j] : c);
		prev = next;
	}
	free(cid);
	return r;
}

static unsigned long shape_face_out(unsigned long *id, const struct shape *sh,
				    size_t fi, const struct edge_ids *eid)
{
	const struct sface *f = &sh->f[fi];
	unsigned long *bounds = xmalloc((f->nloops + 1) * sizeof(*bounds));
	unsigned long s = 0;
	int same = f->sense > 0;

	for (size_t l = 0; l < f->nloops; l++) {
		const struct sloop *lp = &sh->l[f->loop0 + l];
		unsigned long first = *id + 1;
		unsigned long last = 0;

		for (size_t k = 0; k < lp->n; k++) {
			const struct suse *u = &sh->u[lp->first + k];
			const struct edge_ids *e = &eid[u->e];

			for (unsigned long j = 0; j < e->n; j++) {
				printf("#%lu=ORIENTED_EDGE('',*,*,#%lu,.%c.);\n",
				       ++*id,
				       e->first + (u->fwd ? j : e->n - 1 - j),
				       u->fwd ? 'T' : 'F');
			}
		}
		last = *id;
		printf("#%lu=EDGE_LOOP('',(", ++*id);
		for (unsigned long k = first; k <= last; k++) {
			printf("%s#%lu", k > first ? "," : "", k);
		}
		printf("));\n#%lu=FACE_BOUND('',#%lu,.T.);\n", *id + 1, *id);
		bounds[l] = ++*id;
	}
	if (f->s.kind == SF_PIPE) {
		s = pipe_surf(id, sh, fi, &same);
	} else {
		s = shape_surf(id, &f->s);
	}
	printf("#%lu=ADVANCED_FACE(", ++*id);
	string(f->name);
	printf(",(");
	for (size_t l = 0; l < f->nloops; l++) {
		printf("%s#%lu", l ? "," : "", bounds[l]);
	}
	printf("),#%lu,.%c.);\n", s, same ? 'T' : 'F');
	free(bounds);
	return *id;
}

// Dies if a shell of sh, shell[f] the shell of face f, is a void: its
// volume is below 0.
static void refuse_voids(const struct shape *sh, const size_t *shell, size_t n)
{
	double *vol = xcalloc(n + 1, sizeof(*vol));

	for (size_t f = 0; f < sh->nf; f++) {
		vol[shell[f]] += face_mass(sh, f).volume;
	}
	for (size_t k = 0; k < n; k++) {
		if (vol[k] < 0) {
			die("tostep: solid %s has a void, which tostep does "
			    "not write",
			    sh->name);
		}
	}
	free(vol);
}

// Prints solid sh, one STEP solid for each shell, and adds them to sd
// as solids of part part.
static void shape_solid(unsigned long *id, const struct shape *sh,
			struct solids *sd, size_t part)
{
	unsigned long *vid = xmalloc((sh->nv + 1) * sizeof(*vid));
	struct edge_ids *eid = xmalloc((sh->ne + 1) * sizeof(*eid));
	unsigned long *fid = xmalloc((sh->nf + 1) * sizeof(*fid));
	size_t *shell = xmalloc((sh->nf + 1) * sizeof(*shell));
	int *on_sphere = xcalloc(sh->ne + 1, sizeof(*on_sphere));
	size_t n = shape_shells(sh, shell);
	struct face_ids fs = { shell, fid, sh->nf };

	refuse_voids(sh, shell, n);
	for (size_t f = 0; f < sh->nf; f++) {
		struct urange r = face_uses(sh, f);

		for (size_t i = r.first; i < r.end; i++) {
			if (sh->f[f].s.kind == SF_SPHERE ||
			    sh->f[f].s.kind == SF_PIPE) {
				on_sphere[sh->u[i].e] = 1;
			}
		}
	}
	for (size_t i = 0; i < sh->nv; i++) {
		unsigned long pt = triple(id, "CARTESIAN_POINT", sh->v[i]);

		printf("#%lu=VERTEX_POINT('',#%lu);\n", ++*id, pt);
		vid[i] = *id;
	}
	for (size_t i = 0; i < sh->ne; i++) {
		eid[i] = shape_edge_out(id, &sh->e[i],
					pieces_of(sh, i, on_sphere), vid);
	}
	for (size_t f = 0; f < sh->nf; f++) {
		fid[f] = shape_face_out(id, sh, f, eid);
	}
	for (size_t k = 0; k < n; k++) {
		unsigned long shl = shell_solid(id, &fs, k);
		struct solid_ref r = { 0, part };

		printf("#%lu=MANIFOLD_SOLID_BREP(", ++*id);
		string(sh->name);
		printf(",#%lu);\n", shl);
		r.id = *id;
		PUSH(sd->s, sd->n, sd->cap, r);
	}
	free(vid);
	free(eid);
	free(fid);
	free(shell);
	free(on_sphere);
}

// Prints the shape representation of the solids sd.
static void representation(unsigned long *id, const char *name,
			   const struct solids *sd)
{
	struct axes pl = { v3(0, 0, 0), v3(0, 0, 1), v3(1, 0, 0) };
	unsigned long origin = placement(id, pl);

	printf("#%lu=ADVANCED_BREP_SHAPE_REPRESENTATION(", ++*id);
	string(name);
	printf(",(#%lu", origin);
	for (size_t i = 0; i < sd->n; i++) {
		printf(",#%lu", sd->s[i].id);
	}
	printf("),#13);\n#%lu=SHAPE_DEFINITION_REPRESENTATION(#8,#%lu);\n",
	       *id + 1, *id);
	(*id)++;
}

int main(int argc, char **argv)
{
	struct part *ps = NULL;
	size_t n = 0;
	char *name = NULL;
	struct solids sd = { NULL, 0, 0 };
	unsigned long id = 0;

	if (argc != 2) {
		die("usage: tostep STREAM|- > part.step");
	}
	n = read_parts(argv[1], &ps);
	if (!n) {
		die("%s: no parts", argv[1]);
	}
	name = base_name(strcmp(argv[1], "-") == 0 ? "stdin" : argv[1]);
	header(&id, name);
	for (size_t i = 0; i < n; i++) {
		solid(&id, ps, i, &sd);
	}
	representation(&id, name, &sd);
	styles(&id, ps, &sd);
	printf("ENDSEC;\nEND-ISO-10303-21;\n");
	free(sd.s);
	free(name);
	free_parts(ps, n);
	close_stdout();
	return 0;
}
