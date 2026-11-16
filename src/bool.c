// SPDX-License-Identifier: GPL-3.0-only
// bool.c -- Unions, cuts and common parts of exact solids
// Copyright (c) 2026 Jakob Kastelic

// Usage: bool STREAM|- > stream
//
// Reads a stream (stream.h) and takes its statements
//   union NAME A B [C ...]
//   subtract NAME A B [C ...]     A less B, less C, ...
//   common NAME A B
// Each replaces the solids it names by the solid NAME, in turn.
//
// The solids have faces on planes, cylinders and cones.  Each face of
// one solid meets each face of the other along the curves where their
// surfaces meet: the line of two planes, the circle of a plane across a
// cylinder or a cone, the lines of a plane along a cylinder, the
// ellipse of a plane at a slant to one, the circles of two round faces
// on one axis; two faces on one surface share the stretches of their
// edges that lie inside each other.  Where the boxes of two faces meet
// and their surfaces meet on any other curve, bool refuses and names
// them, unless it can tell that the two faces stay clear of each other:
// they lie on the two sides of a plane across an axis; one lies on one
// side of the plane of the other; a plane face lies inside a round
// one; or two round faces with parallel axes stay apart, or one inside
// the other.  So it refuses a result with a void, one whose pieces
// touch, and one that touches itself along an edge.  Each face
// breaks into parts along those curves, and a point inside each part
// tells whether it lies inside the other solid, outside it, or on one of
// its faces, facing the same way or the other way.  That decides whether
// the part bounds the result, and which way it faces.  At the end, the
// faces of the result on one surface join.

#include "geom.h"
#include "integ.h"
#include "isect.h"
#include "merge.h"
#include "shape.h"
#include "split.h"
#include "stream.h"
#include "util.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The operations on two solids.

#define LINE_SPAN 2.0 // The slack of the stretch of a line, times the box.

enum { OP_UNION, OP_CUT, OP_COMMON };
enum {
	EDGE_STEPS = 256, // Samples of an edge against another.
	THIRDS = 100, // Steps of the search for the nearest point.
	TOUCH_STEPS = 4, // How near, in steps, a touch is looked for.
	MSG_LEN = 512 // Room for a message.
};

// The cuts of each face of a solid.
struct cuts {
	struct pedge *c;
	size_t n, cap;
};

struct bool_ctx {
	struct stream *st;
	const struct shape *a, *b;
	struct cuts *ca, *cb;
	struct box *boxa, *boxb;
};

// A growing list of parameters.
struct params {
	double *t;
	size_t n, cap;
};

static void add_param(struct params *p, double t)
{
	PUSH(p->t, p->n, p->cap, t);
}

static void add_cut(struct cuts *c, const struct curve *cv, struct span s)
{
	struct pedge e;

	e.p.c = *cv;
	e.p.t0 = s.lo;
	e.p.t1 = s.hi;
	e.a = curve_at(cv, s.lo);
	e.b = curve_at(cv, s.hi);
	PUSH(c->c, c->n, c->cap, e);
}

// Adds to t the parameters on curve c of the points where edge e meets
// surface s.
static void edge_mark(const struct sedge *e, const struct surf *s,
		      const struct curve *c, struct params *t)
{
	struct cpiece pc = { e->c, e->t0, e->t1 };
	double r[ROOTS_MAX + 2];
	size_t m = curve_surf(&pc, s, r);

	if (curve_on_surf(&pc, s)) {
		r[m++] = e->t0;
		r[m++] = e->t1;
	}
	for (size_t i = 0; i < m; i++) {
		struct vec p = curve_at(&e->c, r[i]);

		if (curve_dist(c, p) < SH_EPS) {
			add_param(t, curve_param(c, p));
		}
	}
}

// Adds to t the parameters on curve c of the points where the edges of
// face f of sh meet surface s.
static void edge_marks(const struct shape *sh, size_t f, const struct surf *s,
		       const struct curve *c, struct params *t)
{
	struct urange r = face_uses(sh, f);

	for (size_t i = r.first; i < r.end; i++) {
		edge_mark(&sh->e[sh->u[i].e], s, c, t);
	}
}

// The stretch of line c inside the box x, slack added.
static struct span line_span(const struct curve *c, struct box x)
{
	struct span s = { HUGE_VAL, -HUGE_VAL };
	double w = 0;

	for (unsigned k = 0; k < BOX_CORNERS; k++) {
		double t = dot(sub(box_corner(x, k), c->o), c->x);

		s.lo = fmin(s.lo, t);
		s.hi = fmax(s.hi, t);
	}
	w = (LINE_SPAN * (s.hi - s.lo)) + SH_EPS;
	s.lo -= w;
	s.hi += w;
	return s;
}

// The common box of the faces fa of a and fb of b.
static struct box pair_box(const struct bool_ctx *x, size_t fa, size_t fb)
{
	struct box p = x->boxa[fa];
	struct box q = x->boxb[fb];

	p.lo = v3(fmax(p.lo.x, q.lo.x), fmax(p.lo.y, q.lo.y),
		  fmax(p.lo.z, q.lo.z));
	p.hi = v3(fmin(p.hi.x, q.hi.x), fmin(p.hi.y, q.hi.y),
		  fmin(p.hi.z, q.hi.z));
	return p;
}

// The parameters of curve c where it may enter or leave face fa of a or
// fb of b, sorted, over one turn of an angle or the stretch of a line
// across the two faces.
static void cut_marks(const struct bool_ctx *x, size_t fa, size_t fb,
		      const struct curve *c, struct params *t)
{
	struct span sp = { 0, 0 };

	edge_marks(x->a, fa, &x->b->f[fb].s, c, t);
	edge_marks(x->b, fb, &x->a->f[fa].s, c, t);
	if (curve_turns(c)) {
		sp.lo = t->n ? t->t[0] : 0;
		for (size_t i = 0; i < t->n; i++) {
			t->t[i] = turn_from(t->t[i], sp.lo);
		}
		sp.hi = sp.lo + (2 * PI);
	} else {
		sp = line_span(c, pair_box(x, fa, fb));
	}
	add_param(t, sp.lo);
	add_param(t, sp.hi);
	sort_doubles(t->t, t->n);
}

// Adds the stretches of curve c that lie on face fa of a and fb of b to
// the cuts of both.
static void curve_cuts(struct bool_ctx *x, size_t fa, size_t fb,
		       const struct curve *c)
{
	struct params t = { NULL, 0, 0 };

	cut_marks(x, fa, fb, c, &t);
	for (size_t i = 0; i + 1 < t.n; i++) {
		struct span s = { t.t[i], t.t[i + 1] };
		struct vec p = curve_at(c, (s.lo + s.hi) / 2);

		if (s.hi - s.lo > SH_EPS && face_on(x->a, fa, p) &&
		    face_on(x->b, fb, p)) {
			add_cut(&x->ca[fa], c, s);
			add_cut(&x->cb[fb], c, s);
		}
	}
	free(t.t);
}

// The distance from edge d to the point of edge e at t.
static double dist_at(const struct sedge *e, const struct sedge *d, double t)
{
	return edge_dist(d, curve_at(&e->c, t));
}

// The parameter in s of the point of edge e nearest edge d, by thirds.
static double nearest(const struct sedge *e, const struct sedge *d,
		      struct span s)
{
	for (int j = 0; j < THIRDS; j++) {
		double m1 = s.lo + ((s.hi - s.lo) / 3);
		double m2 = s.hi - ((s.hi - s.lo) / 3);

		if (dist_at(e, d, m1) < dist_at(e, d, m2)) {
			s.hi = m2;
		} else {
			s.lo = m1;
		}
	}
	return (s.lo + s.hi) / 2;
}

// Adds to t the parameters on edge e of the ends of edge d that lie on
// it.
static void end_marks(const struct sedge *e, const struct sedge *d,
		      struct params *t)
{
	for (int k = 0; k < 2; k++) {
		struct vec q = curve_at(&d->c, k ? d->t1 : d->t0);

		if (edge_dist(e, q) < SH_EPS) {
			double x = curve_param(&e->c, q);

			add_param(t,
				  curve_turns(&e->c) ? turn_from(x, e->t0) : x);
		}
	}
}

// Adds to t the parameters of the points of edge e within SH_EPS of edge
// d: the ends of d on e, and where the two cross or touch.  Where the
// two run together, the stretch ends at an end of one of them, so only
// a sample with samples off d on both sides marks a crossing.
static void edge_edge(const struct sedge *e, const struct sedge *d,
		      struct params *t)
{
	double h = (e->t1 - e->t0) / (double)EDGE_STEPS;
	double f[EDGE_STEPS + 1];

	end_marks(e, d, t);
	for (int i = 0; i <= EDGE_STEPS; i++) {
		f[i] = dist_at(e, d, e->t0 + (h * i));
	}
	for (int i = 1; i < EDGE_STEPS; i++) {
		struct span s = { e->t0 + (h * (i - 1)),
				  e->t0 + (h * (i + 1)) };
		double m = 0;

		if (f[i] > f[i - 1] || f[i] > f[i + 1] ||
		    f[i] >= h * (double)TOUCH_STEPS ||
		    fmin(f[i - 1], f[i + 1]) <= SH_EPS) {
			continue;
		}
		m = nearest(e, d, s);
		if (dist_at(e, d, m) < SH_EPS) {
			add_param(t, m);
		}
	}
}

// Adds to the cuts c the stretches of edge e that lie inside face fo of
// o, on the same surface.
static void same_edge_cuts(const struct sedge *e, const struct shape *o,
			   size_t fo, struct cuts *c)
{
	struct urange r = face_uses(o, fo);
	struct params t = { NULL, 0, 0 };

	add_param(&t, e->t0);
	add_param(&t, e->t1);
	for (size_t j = r.first; j < r.end; j++) {
		edge_edge(e, &o->e[o->u[j].e], &t);
	}
	sort_doubles(t.t, t.n);
	for (size_t i = 0; i + 1 < t.n; i++) {
		struct span s = { t.t[i], t.t[i + 1] };
		struct vec p = curve_at(&e->c, (s.lo + s.hi) / 2);

		if (s.hi - s.lo > SH_EPS && s.lo >= e->t0 - SH_EPS &&
		    s.hi <= e->t1 + SH_EPS && face_has(o, fo, p)) {
			add_cut(c, &e->c, s);
		}
	}
	free(t.t);
}

// Adds to the cuts c the stretches of the edges of face fs of s that
// lie inside face fo of o, on the same surface.
static void same_cuts(const struct shape *s, size_t fs, const struct shape *o,
		      size_t fo, struct cuts *c)
{
	struct urange r = face_uses(s, fs);

	for (size_t k = r.first; k < r.end; k++) {
		same_edge_cuts(&s->e[s->u[k].e], o, fo, c);
	}
}

// Dies with the faces fa of a and fb of b, whose boxes meet and whose
// surfaces meet on a curve bool does not know.
static void refuse_pair(const struct bool_ctx *x, size_t fa, size_t fb)
{
	char msg[MSG_LEN];

	xsnprintf(msg,
		  "the surfaces of faces %s and %s meet on a curve that is "
		  "no line or circle",
		  x->a->f[fa].name, x->b->f[fb].name);
	stream_fail(x->st, msg, NULL);
}

static void pair_cuts(struct bool_ctx *x, size_t fa, size_t fb)
{
	struct curve c[2];
	size_t n = 0;
	int how = surf_surf(&x->a->f[fa].s, &x->b->f[fb].s, c, &n);

	if (how == SS_SAME) {
		same_cuts(x->b, fb, x->a, fa, &x->ca[fa]);
		same_cuts(x->a, fa, x->b, fb, &x->cb[fb]);
	} else if (how == SS_CURVES &&
		   (c[0].kind != CV_ELLIPSE || x->a->f[fa].s.kind == SF_PLANE ||
		    x->b->f[fb].s.kind == SF_PLANE)) {
		for (size_t i = 0; i < n; i++) {
			curve_cuts(x, fa, fb, &c[i]);
		}
	} else if (how != SS_NONE && !faces_clear(x->a, fa, x->b, fb)) {
		refuse_pair(x, fa, fb);
	}
}

// How a point of a face of one solid lies to the other solid.
enum { P_OUT, P_IN, P_SAME, P_OPPOSITE };

// The place to solid o of the point at.o of a face whose outward normal
// there is at.d.
static int place_of(const struct shape *o, struct line3 at)
{
	for (size_t f = 0; f < o->nf; f++) {
		if (face_on(o, f, at.o)) {
			struct vec n = mul(surf_normal(&o->f[f].s, at.o),
					   o->f[f].sense);

			return dot(n, at.d) > 0 ? P_SAME : P_OPPOSITE;
		}
	}
	return solid_has(o, at.o) ? P_IN : P_OUT;
}

// Whether a part of a face of the first solid (first) or the second, at
// place pl to the other, bounds the result of op; *flip gets 1 if it
// faces the other way there.
static int keep(int op, int first, int pl, int *flip)
{
	*flip = 0;
	if (op == OP_UNION) {
		return pl == P_OUT || (first && pl == P_SAME);
	}
	if (op == OP_COMMON) {
		return pl == P_IN || (first && pl == P_SAME);
	}
	if (first) {
		return pl == P_OUT || pl == P_OPPOSITE;
	}
	*flip = 1;
	return pl == P_IN;
}

// A part of a face that bounds the result: part ri of the parts pt of
// face f of solid s, turned round if flip.
struct kept {
	const struct shape *s;
	size_t f;
	const struct parts *pt;
	size_t ri;
	int flip;
};

// The result under construction, and the parts that go into it.
struct result {
	struct shape *sh;
	struct kept *k;
	size_t nk, capk;
};

// The parameters of piece pe where the vertices of sh lie inside it,
// with its ends, sorted.
static void inner_marks(const struct shape *sh, const struct pedge *pe,
			struct params *t)
{
	const struct cpiece *c = &pe->p;

	add_param(t, c->t0);
	for (size_t i = 0; i < sh->nv; i++) {
		struct vec v = sh->v[i];
		double x = curve_param(&c->c, v);

		if (curve_turns(&c->c)) {
			x = turn_from(x, c->t0);
		}
		if (len(sub(v, pe->a)) > SH_EPS &&
		    len(sub(v, pe->b)) > SH_EPS && x > c->t0 && x < c->t1 &&
		    len(sub(curve_at(&c->c, x), v)) < SH_EPS) {
			add_param(t, x);
		}
	}
	add_param(t, c->t1);
	sort_doubles(t->t, t->n);
}

// Adds to the loop under way the uses of piece pe, forward if fwd,
// broken at the vertices of the result inside it, so that two faces
// that meet there share each edge whole.
static void add_piece_uses(struct result *res, const struct pedge *pe, int fwd)
{
	struct params t = { NULL, 0, 0 };

	inner_marks(res->sh, pe, &t);
	for (size_t k = 0; k + 1 < t.n; k++) {
		size_t j = fwd ? k : t.n - 2 - k;
		struct cpiece c = { pe->p.c, t.t[j], t.t[j + 1] };
		int efwd = 1;
		size_t e = shape_piece(res->sh, &c.c,
				       (struct span){ c.t0, c.t1 }, &efwd);

		shape_use(res->sh, e, fwd == efwd);
	}
	free(t.t);
}

// Adds kept part k to the result.
static void add_part(struct result *res, const struct kept *k)
{
	const struct region *r = &k->pt->r[k->ri];
	const struct sface *fc = &k->s->f[k->f];
	int sense = k->flip ? -fc->sense : fc->sense;
	int rev = sense < 0;

	shape_face(res->sh, fc->s, sense, fc->name);
	for (size_t l = 0; l < r->nloops; l++) {
		size_t a = r->loop0[l];
		size_t b = l + 1 < r->nloops ? r->loop0[l + 1] : r->nu;
		size_t first = res->sh->nu;

		for (size_t i = 0; i < b - a; i++) {
			const struct puse *u = &r->u[rev ? b - 1 - i : a + i];

			add_piece_uses(res, &k->pt->e[u->i], u->fwd != rev);
		}
		shape_end_loop(res->sh, first);
	}
}

// One of the two solids of a boolean operation: s, the other one o, the
// cuts of the faces of s, their parts, and whether s comes first.
struct side {
	const struct shape *s, *o;
	const struct cuts *c;
	struct parts *pt;
	int first;
};

// Keeps part i of face f of side d if it bounds the result of op; its
// corners become vertices of the result.
static void keep_part(struct result *res, int op, const struct side *d,
		      size_t f, size_t i)
{
	const struct region *r = &d->pt[f].r[i];
	struct line3 at = { r->inside,
			    mul(surf_normal(&d->s->f[f].s, r->inside),
				d->s->f[f].sense) };
	struct kept k = { d->s, f, &d->pt[f], i, 0 };

	if (!keep(op, d->first, place_of(d->o, at), &k.flip)) {
		return;
	}
	PUSH(res->k, res->nk, res->capk, k);
	for (size_t j = 0; j < r->nu; j++) {
		const struct pedge *pe = &d->pt[f].e[r->u[j].i];

		(void)shape_vertex(res->sh, pe->a);
		(void)shape_vertex(res->sh, pe->b);
	}
}

// Cuts the faces of side d into their parts, and keeps those that bound
// the result of op.
static void keep_side(struct result *res, int op, const struct side *d)
{
	for (size_t f = 0; f < d->s->nf; f++) {
		split_face(d->s, f, d->c[f].c, d->c[f].n, &d->pt[f]);
		for (size_t i = 0; i < d->pt[f].nr; i++) {
			keep_part(res, op, d, f, i);
		}
	}
}

// The cuts of every pair of faces of x whose boxes meet.
static void all_cuts(struct bool_ctx *x)
{
	const struct shape *a = x->a;
	const struct shape *b = x->b;

	x->ca = xcalloc(a->nf + 1, sizeof(*x->ca));
	x->cb = xcalloc(b->nf + 1, sizeof(*x->cb));
	x->boxa = xmalloc((a->nf + 1) * sizeof(*x->boxa));
	x->boxb = xmalloc((b->nf + 1) * sizeof(*x->boxb));
	for (size_t i = 0; i < a->nf; i++) {
		x->boxa[i] = face_box(a, i);
	}
	for (size_t j = 0; j < b->nf; j++) {
		x->boxb[j] = face_box(b, j);
	}
	for (size_t i = 0; i < a->nf; i++) {
		for (size_t j = 0; j < b->nf; j++) {
			if (box_meet(x->boxa[i], x->boxb[j])) {
				pair_cuts(x, i, j);
			}
		}
	}
}

static void free_side(const struct side *d)
{
	for (size_t i = 0; i < d->s->nf; i++) {
		free(d->c[i].c);
		free_parts_of(&d->pt[i]);
	}
	free(d->pt);
}

// Dies unless every edge of the result bounds two faces: where the
// result touches itself along a line, an edge bounds four, and no
// manifold solid holds it.
static void refuse_unless_manifold(struct stream *st, const struct shape *sh)
{
	size_t *n = xcalloc(sh->ne + 1, sizeof(*n));

	for (size_t i = 0; i < sh->nu; i++) {
		n[sh->u[i].e]++;
	}
	for (size_t e = 0; e < sh->ne; e++) {
		if (n[e] > 2) {
			char msg[MSG_LEN];
			struct vec m = edge_at(&sh->e[e], MID);

			free(n);
			xsnprintf(
				msg,
				"the result touches itself along an edge "
				"through %.6g %.6g %.6g, which no solid holds",
				m.x, m.y, m.z);
			stream_fail(st, msg, NULL);
		}
	}
	free(n);
}

// The point where edge e of a void touches face f of sh, if it does,
// into *at.
static int edge_touches(const struct shape *sh, const struct sedge *e, size_t f,
			struct vec *at)
{
	struct cpiece pc = { e->c, e->t0, e->t1 };
	double t[ROOTS_MAX + 2];
	size_t m = curve_surf(&pc, &sh->f[f].s, t);

	if (curve_on_surf(&pc, &sh->f[f].s)) {
		t[m++] = (e->t0 + e->t1) / 2;
	}
	for (size_t i = 0; i < m; i++) {
		*at = curve_at(&e->c, t[i]);
		if (face_on(sh, f, *at)) {
			return 1;
		}
	}
	return 0;
}

// Two faces of a solid.
struct fpair {
	size_t f, g;
};

// Whether an edge of face f of sh touches face g, of another shell;
// *at gets where.
static int face_touches(const struct shape *sh, const size_t *shell,
			struct fpair p, struct vec *at)
{
	struct urange r = face_uses(sh, p.f);

	if (shell[p.f] == shell[p.g]) {
		return 0;
	}
	for (size_t i = r.first; i < r.end; i++) {
		if (edge_touches(sh, &sh->e[sh->u[i].e], p.g, at)) {
			return 1;
		}
	}
	return 0;
}

// Dies with what is wrong with the result at the point at.
static void result_fail(struct stream *st, const char *what, struct vec at)
{
	char msg[MSG_LEN];

	xsnprintf(msg, "%s at %.6g %.6g %.6g, which bool does not make", what,
		  at.x, at.y, at.z);
	stream_fail(st, msg, NULL);
}

// Dies unless the shells of the result, if more than one, are solids
// apart: a shell of a volume below 0 is a void inside a solid, and two
// shells that touch, at a point or along a line, are no solids either.
static void refuse_odd_shells(struct stream *st, const struct shape *sh)
{
	size_t *shell = xmalloc((sh->nf + 1) * sizeof(*shell));
	size_t n = shape_shells(sh, shell);
	double *vol = xcalloc(n + 1, sizeof(*vol));
	struct vec at;

	for (size_t f = 0; f < sh->nf && n > 1; f++) {
		vol[shell[f]] += face_mass(sh, f).volume;
	}
	for (size_t f = 0; f < sh->nf && n > 1; f++) {
		struct urange r = face_uses(sh, f);

		if (vol[shell[f]] < 0) {
			result_fail(st, "the result has a void",
				    edge_at(&sh->e[sh->u[r.first].e], 0));
		}
		for (size_t g = 0; g < sh->nf; g++) {
			struct fpair p = { f, g };

			if (face_touches(sh, shell, p, &at)) {
				result_fail(st,
					    "two pieces of the result touch",
					    at);
			}
		}
	}
	free(shell);
	free(vol);
}

static void boolean(struct stream *st, int op, const struct shape *a,
		    const struct shape *b, struct shape *out)
{
	struct bool_ctx x = { st, a, b, NULL, NULL, NULL, NULL };
	struct result res = { out, NULL, 0, 0 };
	struct side da = { a, b, NULL, xcalloc(a->nf + 1, sizeof(struct parts)),
			   1 };
	struct side db = { b, a, NULL, xcalloc(b->nf + 1, sizeof(struct parts)),
			   0 };

	all_cuts(&x);
	da.c = x.ca;
	db.c = x.cb;
	keep_side(&res, op, &da);
	keep_side(&res, op, &db);
	for (size_t i = 0; i < res.nk; i++) {
		add_part(&res, &res.k[i]);
	}
	free_side(&da);
	free_side(&db);
	free(x.ca);
	free(x.cb);
	free(x.boxa);
	free(x.boxb);
	free(res.k);
	refuse_unless_manifold(st, out);
	shape_check(out, st->where);
	shape_merge(out);
	shape_check(out, st->where);
	refuse_odd_shells(st, out);
}

// Dies unless every face of sh lies on a plane, a cylinder or a cone.
static void check_kinds(struct stream *st, const struct shape *sh)
{
	for (size_t f = 0; f < sh->nf; f++) {
		if (sh->f[f].s.kind > SF_CONE) {
			stream_fail(st,
				    "bool takes faces on planes, cylinders "
				    "and cones, and this one is none",
				    sh->f[f].name);
		}
	}
}

static void st_bool(struct stream *st, int op)
{
	struct shape acc;

	if (st->nw < 4 || (op == OP_COMMON && st->nw != 4)) {
		stream_fail(st,
			    "union|subtract NAME A B ..., or common NAME A B",
			    NULL);
	}
	stream_take(st, st->w[2], &acc);
	check_kinds(st, &acc);
	for (size_t i = 3; i < st->nw; i++) {
		struct shape b;
		struct shape r;

		stream_take(st, st->w[i], &b);
		check_kinds(st, &b);
		shape_init(&r, st->w[1]);
		boolean(st, op, &acc, &b, &r);
		shape_free(&acc);
		shape_free(&b);
		acc = r;
	}
	free(acc.name);
	acc.name = xstrdup(st->w[1]);
	stream_add(st, &acc);
}

int main(int argc, char **argv)
{
	struct stream st;

	if (argc != 2) {
		die("usage: bool STREAM|- > stream");
	}
	stream_read(&st, argv[1]);
	for (size_t i = 0; i < st.nln; i++) {
		if (!stream_line(&st, i)) {
			continue;
		}
		if (stream_is(&st, 0, "union")) {
			st_bool(&st, OP_UNION);
		} else if (stream_is(&st, 0, "subtract")) {
			st_bool(&st, OP_CUT);
		} else if (stream_is(&st, 0, "common")) {
			st_bool(&st, OP_COMMON);
		} else {
			continue;
		}
		st.ln[i].gone = 1;
	}
	stream_write(&st);
	stream_free(&st);
	close_stdout();
	return 0;
}
