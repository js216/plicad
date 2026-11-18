// SPDX-License-Identifier: GPL-3.0-only
// extrude.c -- Solids of outlines pushed along an axis, with a draft
// Copyright (c) 2026 Jakob Kastelic

// Usage: extrude PART|- > stream
//
// Reads a part source or a stream (stream.h) and adds the solids of its
// extrude statements:
//   profile NAME along A U V R [side SIDE] [U V R [side SIDE] ...]
//   corner U V R [side SIDE] [U V R [side SIDE] ...]
//       A closed outline in the plane across the world axis A (x, y or
//       z), U and V the other two axes in alphabetical order: its
//       corners in order, each rounded by a tangent arc of radius R, or
//       sharp for R = 0.  Corner lines go on with the profile above.
//       The side from a corner to the next takes the name SIDE of the
//       corner, if it has one.
//   circle NAME D U V along A
//       A circle of diameter D about (U, V).
//   extrude NAME from W0 to W1 P [draft DEG] [pattern AX N P [AX N P]]
//       The solid of the region of profile P, from A = W0 to A = W1.  A
//       draft of DEG degrees, from 0 up to 89, shrinks the region along
//       the way: each side leans in by that angle, so an arc becomes a
//       piece of a cone, and a circle a cone, which may end in a point.
//       A pattern makes N copies, P apart along the axis AX (with its
//       sign), as one solid.
// The faces are named NAME.start and NAME.end for the two ends, and
// NAME.side.SIDE for a side of its profile named SIDE, NAME.side for the
// others and the arcs of the corners.
//
// The stream goes out with the ranges of the faces (stream.h): for each
// parameter of a range that the solids lean on (expr.h), the solids are
// made again with the parameter moved by STEP_SHARE of its range, and
// each face that moves goes out with its surface and its surface at
// each end of the range, moved in proportion.

#include "expr.h"
#include "fold.h"
#include "geom.h"
#include "shape.h"
#include "stream.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FLAT 1e-12 // A cross product of unit vectors below this is none.
#define DRAFT_MAX 89.0 // The steepest draft, in degrees.
#define PATTERN_MAX 1000.0 // The most copies along a pattern axis.
#define STEP_SHARE 1e-4 // Of its range, the move of a parameter to rebuild.

enum {
	CORNER_WORDS = 3, // U V R
	PROFILE_HEAD = 4, // profile NAME along A
	CIRCLE_WORDS = 7, // circle NAME D U V along A
	EXTRUDE_HEAD = 6, // extrude NAME from W0 to W1
	EXTRUDE_W0 = 3, // The words of W0 and W1.
	EXTRUDE_W1 = 5,
	CIRCLE_D = 2, // The words of D, U, V and A.
	CIRCLE_U = 3,
	CIRCLE_V = 4,
	CIRCLE_A = 6,
	NAME_LEN = 256, // Room for the name of a face.
	SURF_ROOM = 512 // Room for the text of a surface.
};

struct p2 {
	double u, v;
};

struct corner {
	struct p2 p;
	double r;
	char *side; // The name of its faces, or NULL.
};

// A profile: its corners, or a circle.
struct prof {
	char *name;
	int axis;
	int circle;
	struct p2 c;
	double r;
	struct corner *k;
	size_t n, cap;
};

// A piece of an outline in (u, v): a line from a to b, or an arc about
// c of radius r, counterclockwise if ccw; the outline turns sharp
// before it if sharp.
struct seg {
	int arc, ccw, sharp;
	struct p2 a, b, c;
	double r;
	const char *side; // The name of its corner, or NULL.
};

struct outline {
	struct seg *s;
	size_t n, cap;
	const char *name;
};

struct ex {
	struct stream st;
	struct prof *pf;
	size_t npf, cappf;
};

static struct p2 pt2(double u, double v)
{
	struct p2 p = { u, v };

	return p;
}

static struct p2 sub2(struct p2 a, struct p2 b)
{
	return pt2(a.u - b.u, a.v - b.v);
}

static double len2(struct p2 a)
{
	return hypot(a.u, a.v);
}

static double cross2(struct p2 a, struct p2 b)
{
	return (a.u * b.v) - (a.v * b.u);
}

static int axis_of(struct ex *x, const char *w)
{
	if (strcmp(w, "x") == 0 || strcmp(w, "y") == 0 || strcmp(w, "z") == 0) {
		return w[0] - 'x';
	}
	stream_fail(&x->st, "an axis is x, y or z", w);
}

static struct prof *find_prof(struct ex *x, const char *name)
{
	for (size_t i = 0; i < x->npf; i++) {
		if (strcmp(x->pf[i].name, name) == 0) {
			return &x->pf[i];
		}
	}
	stream_fail(&x->st, "no such profile", name);
}

// Names the side after the last corner of p by word i of the line under
// work, side SIDE.
static void name_side(struct ex *x, struct prof *p, size_t i)
{
	struct stream *st = &x->st;

	if (i + 1 >= st->nw || p->n == 0 || p->k[p->n - 1].side) {
		stream_fail(st, "side SIDE follows a corner U V R", NULL);
	}
	p->k[p->n - 1].side = xstrdup(st->w[i + 1]);
}

static void add_corners(struct ex *x, struct prof *p, size_t from)
{
	struct stream *st = &x->st;

	for (size_t i = from; i < st->nw;) {
		struct corner k;

		if (stream_is(st, i, "side")) {
			name_side(x, p, i);
			i += 2;
			continue;
		}
		if (i + CORNER_WORDS > st->nw) {
			stream_fail(st, "corners come as U V R", NULL);
		}
		k.p = pt2(stream_num(st, i), stream_num(st, i + 1));
		k.r = stream_num(st, i + 2);
		k.side = NULL;
		if (!(k.r >= 0)) {
			stream_fail(st, "a corner radius is 0 or more", NULL);
		}
		PUSH(p->k, p->n, p->cap, k);
		i += CORNER_WORDS;
	}
}

// Dies if a profile has the name of the one of the line under work.
static void check_new_prof(struct ex *x)
{
	struct stream *st = &x->st;

	for (size_t i = 0; i < x->npf; i++) {
		if (strcmp(x->pf[i].name, st->w[1]) == 0) {
			stream_fail(st, "a profile of that name exists",
				    st->w[1]);
		}
	}
}

static void new_prof(struct ex *x, int circle)
{
	struct stream *st = &x->st;
	struct prof p;

	memset(&p, 0, sizeof(p));
	if (circle ? st->nw != CIRCLE_WORDS ||
			     !stream_is(st, CIRCLE_A - 1, "along") :
		     st->nw < PROFILE_HEAD || !stream_is(st, 2, "along")) {
		stream_fail(st,
			    circle ? "circle NAME D U V along A" :
				     "profile NAME along A U V R ...",
			    NULL);
	}
	check_new_prof(x);
	p.name = xstrdup(st->w[1]);
	p.axis = axis_of(x, st->w[circle ? CIRCLE_A : 3]);
	p.circle = circle;
	if (circle) {
		p.c = pt2(stream_num(st, CIRCLE_U), stream_num(st, CIRCLE_V));
		p.r = stream_num(st, CIRCLE_D) / 2;
		if (!(p.r > SH_EPS)) {
			stream_fail(st, "a circle needs D above 0", NULL);
		}
	}
	PUSH(x->pf, x->npf, x->cappf, p);
	if (!circle) {
		add_corners(x, &x->pf[x->npf - 1], PROFILE_HEAD);
	}
}

// The outline of a profile of corners, counterclockwise.
static double corner_area(const struct prof *p)
{
	double a = 0;

	for (size_t i = 0; i < p->n; i++) {
		a += cross2(p->k[i].p, p->k[(i + 1) % p->n].p);
	}
	return a / 2;
}

// The arc that rounds corner i of p, from where it leaves the side
// before to where it joins the side after, or a sharp corner.
static struct seg corner_arc(struct ex *x, const struct prof *p, size_t i,
			     int rev)
{
	size_t n = p->n;
	struct p2 c = p->k[i].p;
	struct p2 a = p->k[rev ? (i + 1) % n : (i + n - 1) % n].p;
	struct p2 b = p->k[rev ? (i + n - 1) % n : (i + 1) % n].p;
	struct p2 da = sub2(a, c);
	struct p2 db = sub2(b, c);
	double la = len2(da);
	double lb = len2(db);
	double r = p->k[i].r;
	struct seg s;
	double h = 0;
	double d = 0;
	struct p2 m;

	memset(&s, 0, sizeof(s));
	if (!(la > SH_EPS) || !(lb > SH_EPS)) {
		stream_fail(&x->st, "two corners of a profile coincide",
			    p->name);
	}
	da = pt2(da.u / la, da.v / la);
	db = pt2(db.u / lb, db.v / lb);
	s.a = c;
	s.b = c;
	if (!(r > 0)) {
		s.sharp = 1;
		return s;
	}
	h = acos(fmax(-1, fmin(1, (da.u * db.u) + (da.v * db.v)))) / 2;
	if (!(h > FLAT) || !(h < (PI / 2) - FLAT)) {
		stream_fail(&x->st, "a rounded corner of a profile is straight",
			    p->name);
	}
	d = r / tan(h);
	if (d > la + SH_EPS || d > lb + SH_EPS) {
		stream_fail(&x->st, "a corner round is longer than a side",
			    p->name);
	}
	m = pt2(da.u + db.u, da.v + db.v);
	m = pt2(m.u / len2(m), m.v / len2(m));
	s.arc = 1;
	s.r = r;
	s.a = pt2(c.u + (da.u * d), c.v + (da.v * d));
	s.b = pt2(c.u + (db.u * d), c.v + (db.v * d));
	s.c = pt2(c.u + (m.u * r / sin(h)), c.v + (m.v * r / sin(h)));
	s.ccw = cross2(sub2(s.a, s.c), sub2(s.b, s.c)) > 0;
	return s;
}

static void push_seg(struct outline *o, struct seg s)
{
	PUSH(o->s, o->n, o->cap, s);
}

// The outline of profile p, its region on the left: sides and corner
// arcs in turn.
static void corner_outline(struct ex *x, const struct prof *p,
			   struct outline *o)
{
	size_t n = p->n;
	int rev = !(corner_area(p) > 0);
	struct seg *arcs = xmalloc((n + 1) * sizeof(*arcs));

	if (n < 3 || !(fabs(corner_area(p)) > SH_EPS)) {
		stream_fail(&x->st,
			    "a profile needs three corners round an "
			    "area",
			    p->name);
	}
	for (size_t i = 0; i < n; i++) {
		arcs[i] = corner_arc(x, p, rev ? n - 1 - i : i, rev);
	}
	for (size_t i = 0; i < n; i++) {
		struct seg line;
		const struct seg *nx = &arcs[(i + 1) % n];

		memset(&line, 0, sizeof(line));
		// The side from the corner of arcs[i] to the next one, which
		// a reversed outline runs from the next one back.
		line.side = p->k[rev ? ((2 * n) - 2 - i) % n : i].side;
		if (arcs[i].arc) {
			push_seg(o, arcs[i]);
		}
		line.a = arcs[i].b;
		line.b = nx->a;
		line.sharp = !arcs[i].arc;
		if (len2(sub2(line.b, line.a)) > SH_EPS) {
			push_seg(o, line);
		} else if (!arcs[i].arc || !nx->arc) {
			stream_fail(&x->st, "a side of a profile vanishes",
				    p->name);
		}
	}
	free(arcs);
}

// The outline of a circle: two half circles, its region on the left.
static void circle_outline(const struct prof *p, struct outline *o)
{
	struct seg s;

	memset(&s, 0, sizeof(s));
	s.arc = 1;
	s.ccw = 1;
	s.c = p->c;
	s.r = p->r;
	s.a = pt2(p->c.u + p->r, p->c.v);
	s.b = pt2(p->c.u - p->r, p->c.v);
	push_seg(o, s);
	s.a = s.b;
	s.b = pt2(p->c.u + p->r, p->c.v);
	push_seg(o, s);
}

// The unit normal toward the left of s at its start, or end.
static struct p2 left_of(const struct seg *s, int at_end)
{
	struct p2 q = at_end ? s->b : s->a;
	struct p2 d;

	if (!s->arc) {
		d = sub2(s->b, s->a);
		d = pt2(-d.v / len2(d), d.u / len2(d));
		return d;
	}
	d = sub2(q, s->c);
	d = pt2(d.u / s->r, d.v / s->r);
	return s->ccw ? pt2(-d.u, -d.v) : d;
}

// A line in the plane, through o along d.
struct line2 {
	struct p2 o, d;
};

// The point where the lines a and b meet.
static struct p2 meet(struct line2 a, struct line2 b)
{
	double den = cross2(a.d, b.d);
	double t = cross2(sub2(b.o, a.o), b.d) / den;

	return pt2(a.o.u + (a.d.u * t), a.o.v + (a.d.v * t));
}

// The outline o moved in by d toward its region: each start point moves
// along the normal, or to the meeting of the moved lines at a sharp
// corner, which joins two lines; an arc that turns left loses d of its
// radius, and one that turns right gains it.
static void far_outline(const struct outline *o, double d, struct outline *out)
{
	size_t n = o->n;

	for (size_t i = 0; i < n; i++) {
		struct seg s = o->s[i];
		const struct seg *pv = &o->s[(i + n - 1) % n];
		struct p2 l = left_of(&s, 0);

		if (s.arc) {
			s.r += s.ccw ? -d : d;
		}
		if (!s.sharp || !(d > 0)) {
			s.a = pt2(s.a.u + (l.u * d), s.a.v + (l.v * d));
		} else {
			struct p2 lp = left_of(pv, 1);
			struct line2 before = { pt2(pv->a.u + (lp.u * d),
						    pv->a.v + (lp.v * d)),
						sub2(pv->b, pv->a) };
			struct line2 after = { pt2(s.a.u + (l.u * d),
						   s.a.v + (l.v * d)),
					       sub2(s.b, s.a) };

			s.a = meet(before, after);
		}
		PUSH(out->s, out->n, out->cap, s);
	}
	for (size_t i = 0; i < n; i++) {
		out->s[i].b = out->s[(i + 1) % n].a;
	}
}

// The frame of an extrusion: the world axes of u and v and of the
// way from W0 to W1.
struct frame {
	int au, av, aw;
	double w0, w1, dir, tana;
	struct vec shift;
};

static struct vec world(const struct frame *f, struct p2 p, double w)
{
	return add(add(add(axis_vec(f->au, p.u), axis_vec(f->av, p.v)),
		       axis_vec(f->aw, w)),
		   f->shift);
}

// A solid under construction: the shape, its frame, and the outlines.
struct build {
	struct stream *st;
	struct shape *sh;
	const struct frame *f;
	const char *name;
	char fname[NAME_LEN];
};

// The edge of segment s of an outline at height w, from vertex a to b;
// for an arc of radius 0, none.
static size_t seg_edge(struct build *bd, const struct seg *s, double w,
		       struct sedge e)
{
	const struct frame *f = bd->f;
	struct vec pa = world(f, s->a, w);
	struct vec pb = world(f, s->b, w);
	struct vec up = axis_vec(f->aw, 1);

	if (!s->arc) {
		e.c.kind = CV_LINE;
		e.c.o = pa;
		e.c.x = unit(sub(pb, pa));
		e.t0 = 0;
		e.t1 = len(sub(pb, pa));
		return shape_edge(bd->sh, e);
	}
	if (!(s->r > SH_EPS)) {
		return NONE;
	}
	{
		struct vec c = world(f, s->c, w);
		struct vec xa = unit(sub(pa, c));
		struct vec xb = unit(sub(pb, c));
		struct vec u3 = axis_vec(f->au, 1);
		struct vec v3d = axis_vec(f->av, 1);
		double hand = dot(cross(u3, v3d), up);
		struct vec n = mul(up, (s->ccw ? 1 : -1) * hand);
		double sw = atan2(dot(cross(xa, xb), n), dot(xa, xb));

		if (sw <= 0) {
			sw += 2 * PI;
		}
		e.c.kind = CV_CIRCLE;
		e.c.o = c;
		e.c.n = n;
		e.c.x = xa;
		e.c.a = len(sub(pa, c));
		e.t0 = 0;
		e.t1 = sw;
	}
	return shape_edge(bd->sh, e);
}

// The surface of side s, from its outline at w0 to its moved copy t at
// w1, and its sense.
static struct surf side_surf(const struct build *bd, const struct seg *s,
			     int *sense)
{
	const struct frame *f = bd->f;
	struct surf sf;
	struct vec d = axis_vec(f->aw, f->dir);
	struct p2 l = left_of(s, 0);
	struct vec out = mul(sub(world(f, l, 0), world(f, pt2(0, 0), 0)), -1);

	memset(&sf, 0, sizeof(sf));
	*sense = 1;
	if (!s->arc) {
		struct vec a = world(f, s->a, f->w0);
		struct vec t = unit(sub(world(f, s->b, f->w0), a));
		struct vec up = sub(d, mul(out, f->tana));
		struct vec n = unit(cross(t, up));

		if (dot(n, out) < 0) {
			n = mul(n, -1);
		}
		sf.kind = SF_PLANE;
		sf.o = a;
		sf.n = n;
		sf.x = t;
		return sf;
	}
	sf.kind = f->tana > 0 ? SF_CONE : SF_CYL;
	sf.o = world(f, s->c, f->w0);
	sf.n = d;
	sf.x = unit(sub(world(f, s->a, f->w0), sf.o));
	sf.r = s->r;
	sf.r2 = (s->ccw ? -1 : 1) * f->tana;
	*sense = s->ccw ? 1 : -1;
	return sf;
}

// Whether side t of the far outline, a line, keeps a length and the way
// of side b of the near one.
static int keeps_way(const struct seg *b, const struct seg *t)
{
	struct p2 db = sub2(b->b, b->a);
	struct p2 dt = sub2(t->b, t->a);

	return len2(dt) > SH_EPS && (db.u * dt.u) + (db.v * dt.v) > 0;
}

// The edges of one outline of an extrusion: bottom, top and up.
struct ring {
	size_t *bot, *top, *up;
};

static void make_ring(struct build *bd, const struct outline *b,
		      const struct outline *t, struct ring *r)
{
	const struct frame *f = bd->f;
	size_t n = b->n;
	struct sedge e;

	memset(&e, 0, sizeof(e));
	r->bot = xmalloc((n + 1) * sizeof(size_t));
	r->top = xmalloc((n + 1) * sizeof(size_t));
	r->up = xmalloc((n + 1) * sizeof(size_t));
	for (size_t i = 0; i < n; i++) {
		struct vec pb = world(f, b->s[i].a, f->w0);
		struct vec pt = world(f, t->s[i].a, f->w1);

		e.v0 = shape_vertex(bd->sh, pb);
		e.v1 = shape_vertex(bd->sh, world(f, b->s[i].b, f->w0));
		r->bot[i] = seg_edge(bd, &b->s[i], f->w0, e);
		e.v0 = shape_vertex(bd->sh, pt);
		e.v1 = shape_vertex(bd->sh, world(f, t->s[i].b, f->w1));
		r->top[i] = seg_edge(bd, &t->s[i], f->w1, e);
		if (!t->s[i].arc && !keeps_way(&b->s[i], &t->s[i])) {
			stream_fail(bd->st, "the draft closes a side", NULL);
		}
		e.v0 = shape_vertex(bd->sh, pb);
		e.v1 = shape_vertex(bd->sh, pt);
		e.c.kind = CV_LINE;
		e.c.o = pb;
		e.c.x = unit(sub(pt, pb));
		e.c.n = v3(0, 0, 0);
		e.t0 = 0;
		e.t1 = len(sub(pt, pb));
		r->up[i] = shape_edge(bd->sh, e);
	}
}

static void free_ring(struct ring *r)
{
	free(r->bot);
	free(r->top);
	free(r->up);
}

// The side faces of one outline.
static void side_faces(struct build *bd, const struct outline *b,
		       const struct ring *r)
{
	size_t n = b->n;

	for (size_t i = 0; i < n; i++) {
		int sense = 1;
		struct surf s = side_surf(bd, &b->s[i], &sense);
		size_t first = 0;

		if (b->s[i].side) {
			xsnprintf(bd->fname, "%s.side.%s", bd->name,
				  b->s[i].side);
		} else {
			xsnprintf(bd->fname, "%s.side", bd->name);
		}
		shape_face(bd->sh, s, sense, bd->fname);
		first = bd->sh->nu;
		shape_use(bd->sh, r->bot[i], 1);
		shape_use(bd->sh, r->up[(i + 1) % n], 1);
		if (r->top[i] != NONE) {
			shape_use(bd->sh, r->top[i], 0);
		}
		shape_use(bd->sh, r->up[i], 0);
		shape_end_loop(bd->sh, first);
	}
}

// Adds the loop of the edges e of one outline, forward or back.
static void cap_loop(struct shape *sh, const size_t *e, size_t n, int fwd)
{
	size_t first = sh->nu;

	for (size_t i = 0; i < n; i++) {
		size_t k = fwd ? i : n - 1 - i;

		if (e[k] != NONE) {
			shape_use(sh, e[k], fwd);
		}
	}
	if (sh->nu > first) {
		shape_end_loop(sh, first);
	}
}

// The plane of the far end if far, else of the near end, facing out.
static struct surf cap_surf(const struct frame *f, int far)
{
	struct surf s;
	double w = far ? f->w1 : f->w0;
	double sg = far ? 1 : -1;

	memset(&s, 0, sizeof(s));
	s.kind = SF_PLANE;
	s.o = world(f, pt2(0, 0), w);
	s.n = axis_vec(f->aw, sg * f->dir);
	s.x = axis_vec(f->au, 1);
	return s;
}

// Reverses every loop of the faces from f0 on.
static void flip_loops(struct shape *sh, size_t f0)
{
	for (size_t f = f0; f < sh->nf; f++) {
		for (size_t l = 0; l < sh->f[f].nloops; l++) {
			const struct sloop *lp = &sh->l[sh->f[f].loop0 + l];

			for (size_t i = 0, j = lp->n - 1; i < j; i++, j--) {
				struct suse t = sh->u[lp->first + i];

				sh->u[lp->first + i] = sh->u[lp->first + j];
				sh->u[lp->first + j] = t;
			}
			for (size_t i = 0; i < lp->n; i++) {
				sh->u[lp->first + i].fwd =
					!sh->u[lp->first + i].fwd;
			}
		}
	}
}

// Whether the frame u, v, way is left-handed: the loops built for a
// right-handed one then run the other way round.
static int left_handed(const struct frame *f)
{
	return dot(cross(axis_vec(f->au, 1), axis_vec(f->av, 1)),
		   axis_vec(f->aw, f->dir)) < 0;
}

// Builds one copy of the solid of the outline o and its moved copy t at
// the far end.
static void build_copy(struct build *bd, const struct outline *o,
		       const struct outline *t)
{
	struct ring ring;
	size_t f0 = bd->sh->nf;
	int apex = 1;

	make_ring(bd, o, t, &ring);
	side_faces(bd, o, &ring);
	for (size_t k = 0; k < t->n; k++) {
		apex = apex && ring.top[k] == NONE;
	}
	xsnprintf(bd->fname, "%s.start", bd->name);
	shape_face(bd->sh, cap_surf(bd->f, 0), 1, bd->fname);
	cap_loop(bd->sh, ring.bot, o->n, 0);
	if (!apex) {
		xsnprintf(bd->fname, "%s.end", bd->name);
		shape_face(bd->sh, cap_surf(bd->f, 1), 1, bd->fname);
		cap_loop(bd->sh, ring.top, t->n, 1);
	}
	if (left_handed(bd->f)) {
		flip_loops(bd->sh, f0);
	}
	free_ring(&ring);
}

// A pattern: n[k] copies along axis[k], step[k] apart.
struct pattern {
	int axis[2];
	long n[2];
	double step[2];
};

static size_t parse_options(struct ex *x, size_t i, double *draft,
			    struct pattern *pt)
{
	struct stream *st = &x->st;
	int npat = 0;

	while (i < st->nw) {
		if (stream_is(st, i, "draft")) {
			*draft = stream_num(st, i + 1);
			i += 2;
		} else if (stream_is(st, i, "pattern") && npat < 2 &&
			   i + 3 < st->nw) {
			int k = npat++;
			const char *a = st->w[i + 1];
			double sg = a[0] == '-' ? -1 : 1;
			double nn = stream_num(st, i + 2);

			pt->axis[k] =
				axis_of(x, a + (a[0] == '-' || a[0] == '+'));
			if (!(nn >= 1 && nn <= PATTERN_MAX &&
			      !(floor(nn) < nn))) {
				stream_fail(st,
					    "a pattern count is whole, 1 up",
					    NULL);
			}
			pt->n[k] = (long)nn;
			pt->step[k] = sg * stream_num(st, i + 3);
			i += 4;
		} else {
			stream_fail(st, "unknown option", st->w[i]);
		}
	}
	return i;
}

// Dies unless every arc of the far end t keeps a radius, or a circle
// ends in a point.
static void check_radii(struct ex *x, const struct outline *t)
{
	for (size_t k = 0; k < t->n; k++) {
		const struct seg *s = &t->s[k];
		int point = t->n == 2 && fabs(s->r) < SH_EPS;

		if (s->arc && !(s->r > SH_EPS) && !point) {
			stream_fail(&x->st, "the draft closes an arc", t->name);
		}
	}
}

static void st_extrude(struct ex *x)
{
	struct stream *st = &x->st;
	struct frame f;
	struct pattern pt = { { 0, 0 }, { 1, 1 }, { 0, 0 } };
	struct shape sh;
	struct build bd;
	struct outline o;
	struct outline t;
	double draft = 0;
	const struct prof *p = NULL;

	if (st->nw < EXTRUDE_HEAD + 1 || !stream_is(st, 2, "from") ||
	    !stream_is(st, 4, "to")) {
		stream_fail(st, "extrude NAME from W0 to W1 PROFILE ...", NULL);
	}
	memset(&f, 0, sizeof(f));
	memset(&o, 0, sizeof(o));
	memset(&t, 0, sizeof(t));
	f.w0 = stream_num(st, EXTRUDE_W0);
	f.w1 = stream_num(st, EXTRUDE_W1);
	(void)parse_options(x, EXTRUDE_HEAD + 1, &draft, &pt);
	p = find_prof(x, st->w[EXTRUDE_HEAD]);
	if (!(fabs(f.w1 - f.w0) > SH_EPS) || !(draft >= 0) ||
	    draft > DRAFT_MAX) {
		stream_fail(st,
			    "an extrusion needs a length and a draft from 0 "
			    "to 89",
			    NULL);
	}
	f.aw = p->axis;
	f.au = f.aw == 0 ? 1 : 0;
	f.av = f.aw == 2 ? 1 : 2;
	f.dir = f.w1 > f.w0 ? 1 : -1;
	f.tana = tan(draft * DEG);
	o.name = p->name;
	t.name = p->name;
	if (p->circle) {
		circle_outline(p, &o);
	} else {
		corner_outline(x, p, &o);
	}
	far_outline(&o, fabs(f.w1 - f.w0) * f.tana, &t);
	check_radii(x, &t);
	shape_init(&sh, st->w[1]);
	bd.st = st;
	bd.sh = &sh;
	bd.f = &f;
	bd.name = st->w[1];
	for (long j = 0; j < pt.n[0]; j++) {
		for (long k = 0; k < pt.n[1]; k++) {
			f.shift = add(
				axis_vec(pt.axis[0], (double)j * pt.step[0]),
				axis_vec(pt.axis[1], (double)k * pt.step[1]));
			build_copy(&bd, &o, &t);
		}
	}
	free(o.s);
	free(t.s);
	shape_check(&sh, st->where);
	stream_add(st, &sh);
}

static void statement(struct ex *x, size_t i)
{
	struct stream *st = &x->st;

	if (!stream_line(st, i)) {
		return;
	}
	if (stream_is(st, 0, "profile")) {
		new_prof(x, 0);
	} else if (stream_is(st, 0, "circle")) {
		new_prof(x, 1);
	} else if (stream_is(st, 0, "corner")) {
		if (!x->npf || x->pf[x->npf - 1].circle) {
			stream_fail(st, "corner lines follow a profile", NULL);
		}
		add_corners(x, &x->pf[x->npf - 1], 1);
	} else if (stream_is(st, 0, "extrude")) {
		st_extrude(x);
	} else {
		return;
	}
	st->ln[i].gone = 1;
}

static void free_ex(struct ex *x)
{
	for (size_t i = 0; i < x->npf; i++) {
		free(x->pf[i].name);
		for (size_t k = 0; k < x->pf[i].n; k++) {
			free(x->pf[i].k[k].side);
		}
		free(x->pf[i].k);
	}
	free(x->pf);
	stream_free(&x->st);
}

// A parameter of a range that the solids lean on, its index in the
// ranges of the stream, and the end of its range that a record is of,
// the high one if hi, as k times the move of a rebuild.
struct end_of {
	const struct sym *s;
	size_t tol;
	int hi;
	double k;
};

static struct vec lerp(struct vec a, struct vec b, double k)
{
	return add(a, mul(sub(b, a), k));
}

// Surface b with its axis turned the way of that of a, and its origin
// moved along it to the point nearest the origin of a: on a plane, or
// on the axis of a cylinder or a cone, which takes its radius there.
static struct surf near_origin(const struct surf *a, struct surf b)
{
	double t = 0;

	if (dot(a->n, b.n) < 0 && b.kind <= SF_CONE) {
		b.n = mul(b.n, -1);
		b.r2 = -b.r2;
	}
	t = dot(sub(a->o, b.o), b.n);
	if (b.kind == SF_PLANE) {
		b.o = sub(a->o, mul(b.n, t));
	} else if (b.kind == SF_CYL || b.kind == SF_CONE) {
		b.o = add(b.o, mul(b.n, t));
		b.r += b.kind == SF_CONE ? b.r2 * t : 0;
	}
	return b;
}

// The surface a, of the part, changed k times as much as to b, of a
// rebuild, its frame made two unit vectors at right angles again.
static struct surf surf_end(const struct surf *a, const struct surf *to,
			    double k)
{
	struct surf s = *a;
	struct surf nb = near_origin(a, *to);
	const struct surf *b = &nb;

	s.o = lerp(a->o, b->o, k);
	s.n = unit(lerp(a->n, b->n, k));
	s.x = lerp(a->x, b->x, k);
	s.x = unit(sub(s.x, mul(s.n, dot(s.x, s.n))));
	s.r = a->r + (k * (b->r - a->r));
	s.r2 = a->r2 + (k * (b->r2 - a->r2));
	s.r3 = a->r3 + (k * (b->r3 - a->r3));
	return s;
}

// Adds to the ranges of x the vary records of the faces of solid a at
// the end e, those that b, rebuilt, moves; dies unless b has the same
// faces.
static void vary_faces(struct ex *x, const struct shape *a,
		       const struct shape *b, struct end_of e)
{
	if (a->nf != b->nf) {
		die("%s: solid %s takes another form as %s (%s) moves",
		    x->st.src, a->name, e.s->name, e.s->where);
	}
	for (size_t f = 0; f < a->nf; f++) {
		char s0[SURF_ROOM];
		char s1[SURF_ROOM];
		char rec[3 * SURF_ROOM];
		struct surf u = surf_end(&a->f[f].s, &b->f[f].s, e.k);

		if (strcmp(a->f[f].name, b->f[f].name) != 0) {
			die("%s: the faces of solid %s change", x->st.src,
			    a->name);
		}
		if (surfs_same(&a->f[f].s, &b->f[f].s)) {
			continue;
		}
		surf_text(s0, sizeof(s0), &a->f[f].s);
		surf_text(s1, sizeof(s1), &u);
		xsnprintf(rec, "face %s %s %s", a->f[f].name, s0, s1);
		add_vary(&x->st.rg, e.tol, e.hi, rec);
	}
}

// The parameters of a range that the solids lean on, their indices
// among the names.
struct params {
	size_t *i;
	size_t n, cap;
};

static void note_param(const struct sym *s, size_t i, void *ctx)
{
	struct params *ps = ctx;

	(void)s;
	PUSH(ps->i, ps->n, ps->cap, i);
}

// The index of the parameter of the name in g, added if new.
static size_t tol_of(struct ranges *g, const struct sym *s)
{
	for (size_t i = 0; i < g->ntol; i++) {
		if (strcmp(g->tol[i].name, s->name) == 0) {
			return i;
		}
	}
	return add_tol(g, s->name, s->kind, s->lo - s->v, s->hi - s->v);
}

// Makes the solids of x again with each parameter that they lean on
// moved by STEP_SHARE of its range, the solids from n0 on, and adds to
// the ranges of the stream the parameters and what the ends of their
// ranges move, by the rebuild: the surfaces of the faces, each number
// in proportion.
static void vary_all(struct ex *x, size_t n0)
{
	struct params ps = { NULL, 0, 0 };

	each_tolerance(&x->st.nm, note_param, &ps);
	for (size_t k = 0; k < ps.n; k++) {
		const struct sym *s = &x->st.nm.syms[ps.i[k]];
		struct end_of e = { s, tol_of(&x->st.rg, s), 0, 0 };
		double by = STEP_SHARE * (s->hi - s->lo);
		struct ex y;

		memset(&y, 0, sizeof(y));
		stream_source(&y.st, &x->st);
		y.st.nm.vary = ps.i[k] + 1;
		y.st.nm.vary_by = by;
		for (size_t i = 0; i < y.st.nln; i++) {
			statement(&y, i);
		}
		for (e.hi = 0; e.hi < 2; e.hi++) {
			e.k = ((e.hi ? s->hi : s->lo) - s->v) / by;
			for (size_t j = 0; j < y.st.nsh; j++) {
				vary_faces(x, &x->st.sh[n0 + j], &y.st.sh[j],
					   e);
			}
		}
		free_ex(&y);
	}
	free(ps.i);
}

int main(int argc, char **argv)
{
	struct ex x;
	size_t n0 = 0;

	if (argc != 2) {
		die("usage: extrude PART|- > stream");
	}
	memset(&x, 0, sizeof(x));
	stream_read(&x.st, argv[1]);
	n0 = x.st.nsh;
	for (size_t i = 0; i < x.st.nln; i++) {
		statement(&x, i);
	}
	vary_all(&x, n0);
	stream_write(&x.st);
	free_ex(&x);
	close_stdout();
	return 0;
}
