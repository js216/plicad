// SPDX-License-Identifier: GPL-3.0-only
// fillet.c -- Round the edges of exact solids, as a ball rolls along them
// Copyright (c) 2026 Jakob Kastelic

// Usage: fillet [-l] STREAM|- > stream
//
// Reads a stream (stream.h) and takes its statements
//   round R convex
//   round R PATTERN PATTERN
//   sharp PATTERN PATTERN
// which give each edge of each solid a radius.  An edge lies between two
// faces, and a pattern matches the name of a face, a * standing for any
// run of characters.  Two patterns pick the edges between two such
// faces, and convex those that turn outward.  The last rule that matches
// an edge wins; sharp gives the radius 0.  A rule that matches no edge
// is an error.  Edges where two faces meet smoothly have no rule, and an
// edge between two faces that rounds made has only the rules that name
// faces.
//
// The round of an edge is the surface that a ball of the radius of the
// edge sweeps as it rolls along it, in touch with both faces, which are
// planes, cylinders or cones: a cylinder along a line, a torus along a
// circle.
// The rounds are made by radius, the greatest first, so that a smaller
// round runs along the end of a greater one.  Where rounds of one radius
// end at a corner:
//   - an edge that runs on smoothly carries the round on;
//   - a round whose edge ends at a face ends on that face, along the
//     line or circle where the two meet; if that takes all of the face,
//     the round goes on into the face beyond;
//   - three rounds meet in a part of a sphere;
//   - two rounds that meet at an angle, cylinders of one radius, end on
//     each other, along an ellipse.
// Any other corner fillet refuses, with its place and the radius.  So it
// does a radius that leaves no room for a neighbour face or edge, with
// the two faces of the edge.
//
// The face of a round has the names of its two faces joined by +, and a
// sphere those of its three.  With -l, fillet lists the edges instead:
//   edge NAME NAME convex|concave|smooth ANGLE LENGTH
// the angle between the two faces in degrees, 0 for smooth.

#include "geom.h"
#include "integ.h"
#include "isect.h"
#include "shape.h"
#include "stream.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SMOOTH 1e-6 // The sine of an angle under which two faces are one.
#define NEAR 1e-6 // Points of one corner this near are one, in mm.
#define REACH 4.0 // How far past a corner a round is looked for, in radii.
#define HALF_TURN 180.0 // In degrees.
#define INSIDE 1e-6 // The share of a curve of touch left out at each end.
#define HALF 0.5 // A stretch under this, with its ends at one point, is none.

enum { K_SMOOTH, K_CONVEX, K_CONCAVE };
enum {
	DEG_MAX = 3, // The edges of a corner where rounds meet.
	CORNER_MAX = 4, // The most edges of a corner where a round ends.
	NAME_MAX_LEN = 1024, // Room for the name of a face of a round.
	MSG_LEN = 2048, // Room for a message.
	LEN_PIECES = 64, // Pieces of an edge for its length.
	ROOM_PTS = 8 // Pieces of a curve of touch, checked at their ends.
};

static const char *const kind_names[] = { "smooth", "convex", "concave" };

// A rule: its radius, its two patterns (none for the edges that turn
// outward), its line in the stream and the edges it matched.
struct rule {
	double r;
	char *pat[2];
	size_t line;
	size_t hits;
};

struct rules {
	struct rule *r;
	size_t n, cap;
	struct stream *st;
};

// An edge of a solid: the face that runs along it and the one that runs
// against it, how it turns, and its radius by the rules, with the rule.
struct einfo {
	size_t fa, fb;
	int kind;
	double angle;
	double r;
	size_t rule;
	int in; // Whether it is rounded now.
};

// A stretch of a curve from one point to another: forward from t0 to
// t1, or back from t1 to t0; gone if a round took all of its edge.
struct seg {
	struct curve c;
	double t0, t1;
	int fwd;
	int gone;
};

// The round of an edge: the curve of the centre of the ball, in the way
// of the edge, the curves where the ball touches the two faces, the
// surface, and for each end of the edge the parameters where the two
// curves end, and the curve from the end of the first to the end of the
// second.
struct blend {
	size_t e;
	struct curve c, ca, cb;
	struct surf s;
	int sense;
	double tm; // The parameter of the middle of the edge.
	double ta[2], tb[2];
	struct seg end[2];
	int has_end[2];
};

// A part of a sphere at a corner: its three arcs, in order.
struct ball {
	struct surf s;
	int sense;
	size_t f[DEG_MAX];
	struct seg arc[DEG_MAX];
};

// The work of one radius on one solid.
struct stage {
	struct stream *st;
	const struct shape *sh;
	struct einfo *ei;
	double r;
	struct blend *b;
	size_t nb, capb;
	size_t *blend_of; // The round of each edge, or NONE.
	struct vec (*trim)[2]; // The new ends of each edge that stays.
	struct ball *ball;
	size_t nball, capball;
};

static int rule_fits(const struct rule *r, const char *na, const char *nb,
		     int kind)
{
	if (!r->pat[0]) {
		return kind == K_CONVEX &&
		       !(strchr(na, '+') && strchr(nb, '+'));
	}
	return (glob(r->pat[0], na) && glob(r->pat[1], nb)) ||
	       (glob(r->pat[0], nb) && glob(r->pat[1], na));
}

// The curve length of edge e.
static double edge_length(const struct sedge *e)
{
	double l = 0;
	struct vec p = edge_at(e, 0);

	if (e->c.kind == CV_LINE) {
		return e->t1 - e->t0;
	}
	if (e->c.kind == CV_CIRCLE) {
		return e->c.a * (e->t1 - e->t0);
	}
	for (int i = 1; i <= LEN_PIECES; i++) {
		struct vec q = edge_at(e, (double)i / (double)LEN_PIECES);

		l += len(sub(q, p));
		p = q;
	}
	return l;
}

// The outward normal of face f of sh at its point p.
static struct vec out_normal(const struct shape *sh, size_t f, struct vec p)
{
	return mul(surf_normal(&sh->f[f].s, p), sh->f[f].sense);
}

// The two faces of each edge of sh, and how each edge turns.
static struct einfo *edge_infos(const struct shape *sh)
{
	struct einfo *ei = xcalloc(sh->ne + 1, sizeof(*ei));

	for (size_t f = 0; f < sh->nf; f++) {
		struct urange r = face_uses(sh, f);

		for (size_t i = r.first; i < r.end; i++) {
			if (sh->u[i].fwd) {
				ei[sh->u[i].e].fa = f;
			} else {
				ei[sh->u[i].e].fb = f;
			}
		}
	}
	for (size_t e = 0; e < sh->ne; e++) {
		struct vec m = edge_at(&sh->e[e], MID);
		struct vec t = unit(curve_tan(&sh->e[e].c,
					      (sh->e[e].t0 + sh->e[e].t1) / 2));
		struct vec na = out_normal(sh, ei[e].fa, m);
		struct vec nb = out_normal(sh, ei[e].fb, m);
		double s = dot(nb, cross(na, t));
		double sn = len(cross(na, nb));

		ei[e].angle = atan2(sn, dot(na, nb)) * HALF_TURN / PI;
		ei[e].kind = K_SMOOTH;
		if (sn > SMOOTH || dot(na, nb) < 0) {
			ei[e].kind = s < 0 ? K_CONVEX : K_CONCAVE;
		}
		ei[e].rule = NONE;
	}
	return ei;
}

// Gives each edge that turns the radius of the last rule that fits it.
static void apply_rules(struct rules *rs, const struct shape *sh,
			struct einfo *ei)
{
	for (size_t e = 0; e < sh->ne; e++) {
		const char *na = sh->f[ei[e].fa].name;
		const char *nb = sh->f[ei[e].fb].name;

		for (size_t i = 0; i < rs->n && ei[e].kind != K_SMOOTH; i++) {
			if (rule_fits(&rs->r[i], na, nb, ei[e].kind)) {
				ei[e].rule = i;
				ei[e].r = rs->r[i].r;
			}
		}
	}
}

// Dies at the rule of edge e of the stage: what is wrong, and the two
// faces of the edge.
static void edge_fail(const struct stage *g, size_t e, const char *what)
{
	char msg[MSG_LEN];
	const struct einfo *x = &g->ei[e];

	xsnprintf(msg, "the round of %.6g of the edge of %s and %s %s", g->r,
		  g->sh->f[x->fa].name, g->sh->f[x->fb].name, what);
	stream_fail(g->st, msg, NULL);
}

// Dies at the rule of the stage: its rounds leave no room for edge e,
// which stays, between the two faces named.
static void room_fail(const struct stage *g, size_t e)
{
	char msg[MSG_LEN];
	const struct einfo *x = &g->ei[e];

	xsnprintf(msg,
		  "the rounds of %.6g leave no room for the edge of %s "
		  "and %s",
		  g->r, g->sh->f[x->fa].name, g->sh->f[x->fb].name);
	stream_fail(g->st, msg, NULL);
}

// Dies at corner v of the stage, which no round of the stage can end
// at: what is wrong, its place and the radius.
static void corner_fail(const struct stage *g, size_t v, const char *what)
{
	char msg[MSG_LEN];
	struct vec p = g->sh->v[v];

	xsnprintf(msg, "the rounds of %.6g at the corner %.6g %.6g %.6g %s",
		  g->r, p.x, p.y, p.z, what);
	stream_fail(g->st, msg, NULL);
}

// The parameter of p on curve c, an angle within half a turn of ref.
static double param_near(const struct curve *c, struct vec p, double ref)
{
	double t = curve_param(c, p);

	return curve_turns(c) ? angle_near(t, ref) : t;
}

// The stretch of curve c from a to b: the shorter way round a circle or
// an ellipse.
static struct seg seg_between(const struct curve *c, struct vec a, struct vec b)
{
	struct seg s = { *c, curve_param(c, a), curve_param(c, b), 1, 0 };

	if (curve_turns(c)) {
		s.t1 = angle_near(s.t1, s.t0);
	}
	if (s.t1 < s.t0) {
		double t = s.t0;

		s.t0 = s.t1;
		s.t1 = t;
		s.fwd = 0;
	}
	return s;
}

// The arc of the ball of radius r about o from a to b, both on it.
static struct seg ball_arc(struct vec o, double r, struct vec a, struct vec b)
{
	struct curve c;

	memset(&c, 0, sizeof(c));
	c.kind = CV_CIRCLE;
	c.o = o;
	c.x = unit(sub(a, o));
	c.n = unit(cross(sub(a, o), sub(b, o)));
	c.a = r;
	return seg_between(&c, a, b);
}

static struct vec seg_from(const struct seg *s)
{
	return curve_at(&s->c, s->fwd ? s->t0 : s->t1);
}

static struct vec seg_to(const struct seg *s)
{
	return curve_at(&s->c, s->fwd ? s->t1 : s->t0);
}

// Adds the use of stretch s to the loop under way of sh, the other way
// if back.
static void use_seg(struct shape *sh, const struct seg *s, int back)
{
	int efwd = 1;
	size_t e = 0;

	if (!(s->t1 - s->t0 > SH_EPS)) {
		return;
	}
	e = shape_piece(sh, &s->c, (struct span){ s->t0, s->t1 }, &efwd);
	shape_use(sh, e, (s->fwd != back) == efwd);
}

// The surface of face f, a plane, a cylinder or a cone, moved out by d
// along its outward normal; 0 if nothing is left of it, or it is none
// of the three.  A cone keeps its axis and its slope, and its radius at
// each height changes by d over the cosine of its half angle.
static int offset_surf(const struct shape *sh, size_t f, double d,
		       struct surf *out)
{
	const struct surf *s = &sh->f[f].s;
	double dn = d * sh->f[f].sense;

	*out = *s;
	if (s->kind == SF_PLANE) {
		out->o = add(s->o, mul(s->n, dn));
		return 1;
	}
	if (s->kind == SF_CONE) {
		out->r = s->r + (dn * hypot(1, s->r2));
		return 1;
	}
	out->r = s->r + dn;
	return s->kind == SF_CYL && out->r > SH_EPS;
}

// The point of surface s nearest p, which lies near it.
static struct vec foot(const struct surf *s, struct vec p)
{
	return sub(p, mul(surf_normal(s, p), surf_dist(s, p)));
}

// The curve where the ball with its centre on c, a line or a circle,
// touches surface s; 0 if it is no curve of the kind of c.
static int contact_curve(const struct curve *c, const struct surf *s,
			 struct curve *out)
{
	struct vec f0 = foot(s, curve_at(c, 0));

	*out = *c;
	if (c->kind == CV_LINE) {
		out->o = f0;
		return 1;
	}
	out->o = add(c->o, mul(c->n, dot(sub(f0, c->o), c->n)));
	out->a = len(sub(f0, out->o));
	return out->a > SH_EPS && dot(sub(f0, out->o), c->x) > 0 &&
	       len(sub(foot(s, curve_at(c, PI / 2)), curve_at(out, PI / 2))) <
		       NEAR;
}

// The round of edge e of the stage: its curves and its surface, without
// its ends.
static void make_blend(struct stage *g, size_t e)
{
	const struct shape *sh = g->sh;
	const struct einfo *x = &g->ei[e];
	const struct sedge *ed = &sh->e[e];
	double tmid = (ed->t0 + ed->t1) / 2;
	struct vec m = curve_at(&ed->c, tmid);
	struct vec na = out_normal(sh, x->fa, m);
	struct vec nb = out_normal(sh, x->fb, m);
	double d = x->kind == K_CONVEX ? -g->r : g->r;
	struct vec cm = add(m, mul(add(na, nb), d / (1 + dot(na, nb))));
	struct surf oa;
	struct surf ob;
	struct curve cand[2];
	size_t n = 0;
	size_t pick = NONE;
	struct blend b;

	memset(&b, 0, sizeof(b));
	b.e = e;
	if (!offset_surf(sh, x->fa, d, &oa) ||
	    !offset_surf(sh, x->fb, d, &ob)) {
		edge_fail(g, e,
			  "leaves nothing of a face, or a face is no plane, "
			  "cylinder or cone");
	}
	if (surf_surf(&oa, &ob, cand, &n) != SS_CURVES) {
		n = 0;
	}
	for (size_t i = 0; i < n; i++) {
		if (curve_dist(&cand[i], cm) < NEAR &&
		    (cand[i].kind == CV_LINE || cand[i].kind == CV_CIRCLE)) {
			pick = i;
		}
	}
	if (pick == NONE) {
		edge_fail(g, e, "follows no line or circle");
	}
	b.c = cand[pick];
	b.tm = curve_param(&b.c, cm);
	if (dot(curve_tan(&b.c, b.tm), curve_tan(&ed->c, tmid)) < 0) {
		if (b.c.kind == CV_LINE) {
			b.c.x = mul(b.c.x, -1);
		} else {
			b.c.n = mul(b.c.n, -1);
		}
		b.tm = curve_param(&b.c, cm);
	}
	if (!contact_curve(&b.c, &sh->f[x->fa].s, &b.ca) ||
	    !contact_curve(&b.c, &sh->f[x->fb].s, &b.cb)) {
		edge_fail(g, e, "touches a face on no line or circle");
	}
	b.s.o = b.c.o;
	b.sense = x->kind == K_CONVEX ? 1 : -1;
	if (b.c.kind == CV_LINE) {
		b.s.kind = SF_CYL;
		b.s.n = b.c.x;
		b.s.x = unit(sub(curve_at(&b.ca, 0), b.c.o));
		b.s.r = g->r;
	} else {
		b.s.kind = SF_TORUS;
		b.s.n = b.c.n;
		b.s.x = b.c.x;
		b.s.r = b.c.a;
		b.s.r2 = g->r;
	}
	g->blend_of[e] = g->nb;
	PUSH(g->b, g->nb, g->capb, b);
}

// The edges at each vertex.
struct corner {
	size_t e[CORNER_MAX];
	int end[CORNER_MAX]; // 0 where the edge starts at the vertex, 1 where
		// it ends.
	size_t n; // CORNER_MAX + 1 for more edges than CORNER_MAX.
};

static struct corner *corners(const struct shape *sh)
{
	struct corner *c = xcalloc(sh->nv + 1, sizeof(*c));

	for (size_t e = 0; e < sh->ne; e++) {
		for (int k = 0; k < 2; k++) {
			struct corner *x = &c[k ? sh->e[e].v1 : sh->e[e].v0];

			if (x->n < CORNER_MAX) {
				x->e[x->n] = e;
				x->end[x->n] = k;
			}
			x->n += x->n <= CORNER_MAX;
		}
	}
	return c;
}

// The parameter of the centre of round b nearest t where c, a curve
// that goes with the centre (the centre itself or a curve of touch),
// meets surface s; dies with what if it meets it nowhere near.
static double meet_param(const struct stage *g, const struct blend *b,
			 const struct curve *c, const struct surf *s, double t)
{
	double reach = REACH * g->r;
	struct cpiece pc = { *c, t - reach, t + reach };
	double root[ROOTS_MAX];
	size_t n = 0;
	double best = HUGE_VAL;

	if (curve_turns(c)) {
		double a = fmin(PI - SMOOTH, reach / c->a);

		pc.t0 = t - a;
		pc.t1 = t + a;
	}
	n = curve_surf(&pc, s, root);
	for (size_t i = 0; i < n; i++) {
		if (fabs(root[i] - t) < fabs(best - t)) {
			best = root[i];
		}
	}
	if (!(best < HUGE_VAL)) {
		edge_fail(g, b->e, "ends on no face at a corner");
	}
	return best;
}

// The face at corner c that is neither of the two faces of edge e.
static size_t third_face(const struct stage *g, const struct corner *c,
			 size_t e)
{
	for (size_t i = 0; i < c->n; i++) {
		const struct einfo *x = &g->ei[c->e[i]];

		if (x->fa != g->ei[e].fa && x->fa != g->ei[e].fb) {
			return x->fa;
		}
		if (x->fb != g->ei[e].fa && x->fb != g->ei[e].fb) {
			return x->fb;
		}
	}
	return NONE;
}

// Sets the new end of the edge at place i of corner c, which stays, to
// p, a point of its curve: the edge grows where a round bares more of a
// face.
static void set_trim(struct stage *g, const struct corner *c, size_t i,
		     struct vec p, size_t cause)
{
	const struct curve *cv = &g->sh->e[c->e[i]].c;

	if (curve_dist(cv, p) > NEAR) {
		edge_fail(g, cause, "leaves no room for a neighbour edge");
	}
	g->trim[c->e[i]][c->end[i]] = p;
}

// The curve where surfaces a and b meet that runs through p and q.
static int curve_through(const struct surf *a, const struct surf *b,
			 struct vec p, struct vec q, struct curve *out)
{
	struct curve cand[2];
	size_t n = 0;

	if (surf_surf(a, b, cand, &n) != SS_CURVES) {
		return 0;
	}
	for (size_t i = 0; i < n; i++) {
		if (curve_dist(&cand[i], p) < NEAR &&
		    curve_dist(&cand[i], q) < NEAR) {
			*out = cand[i];
			return 1;
		}
	}
	return 0;
}

// The place in corner c of the edge, other than the one at place i,
// that has face f on one side; *other gets the face on its other side.
static size_t edge_of_face(const struct stage *g, const struct corner *c,
			   size_t i, size_t f, size_t *other)
{
	for (size_t k = 0; k < c->n; k++) {
		const struct einfo *x = &g->ei[c->e[k]];

		if (k != i && (x->fa == f || x->fb == f)) {
			*other = x->fa == f ? x->fb : x->fa;
			return k;
		}
	}
	return NONE;
}

// The end of the round of the edge at place i of corner c on a face
// there: the round ends along the curve where it meets that face.  At a
// corner of three edges the face is the third one.  At a corner of four,
// one of the two faces of the round runs on smoothly into a fourth face,
// and the round ends on the other of the two that it does not touch; the
// two edges between the smooth pair and that face then end at one point.
static void run_out(struct stage *g, size_t v, const struct corner *c, size_t i)
{
	const struct shape *sh = g->sh;
	size_t e = c->e[i];
	int k = c->end[i];
	struct blend *b = &g->b[g->blend_of[e]];
	const struct einfo *x = &g->ei[e];
	double tv = param_near(&b->c, sh->v[v], b->tm);
	size_t fca = NONE;
	size_t fcb = NONE;
	size_t ga = edge_of_face(g, c, i, x->fa, &fca);
	size_t gb = edge_of_face(g, c, i, x->fb, &fcb);
	int smooth_a = ga != NONE && g->ei[c->e[ga]].kind == K_SMOOTH;
	int smooth_b = gb != NONE && g->ei[c->e[gb]].kind == K_SMOOTH;
	size_t fc = fca;
	struct vec pa;
	struct vec pb;
	struct curve end;

	if (ga == NONE || gb == NONE || ga == gb) {
		corner_fail(g, v, "have no face to end on");
	}
	if ((c->n == DEG_MAX && (fca != fcb || smooth_a || smooth_b)) ||
	    (c->n != DEG_MAX && (fca == fcb || smooth_a == smooth_b))) {
		corner_fail(g, v,
			    "end where the edge runs on smoothly with "
			    "another radius");
	}
	if (c->n != DEG_MAX && smooth_a) {
		fc = fcb;
	}
	b->ta[k] = meet_param(g, b, &b->ca, &sh->f[fc].s, tv);
	b->tb[k] = meet_param(g, b, &b->cb, &sh->f[fc].s, tv);
	pa = curve_at(&b->ca, b->ta[k]);
	pb = curve_at(&b->cb, b->tb[k]);
	set_trim(g, c, ga, pa, e);
	set_trim(g, c, gb, pb, e);
	if (c->n != DEG_MAX) {
		set_trim(g, c, CORNER_MAX + 2 - i - ga - gb, smooth_a ? pa : pb,
			 e);
	}
	if (!curve_through(&b->s, &sh->f[fc].s, pa, pb, &end)) {
		corner_fail(g, v,
			    "end on a face along no line, circle or conic");
	}
	b->end[k] = seg_between(&end, pa, pb);
	b->has_end[k] = 1;
}

// The end of the round at place i of corner c at the ball with its
// centre at o: both curves of touch end where that ball touches.
static void ball_end(struct stage *g, const struct corner *c, size_t i,
		     struct vec o)
{
	struct blend *b = &g->b[g->blend_of[c->e[i]]];
	int k = c->end[i];
	double t = param_near(&b->c, o, b->tm);

	b->ta[k] = t;
	b->tb[k] = t;
	b->end[k] = ball_arc(curve_at(&b->c, t), g->r, curve_at(&b->ca, t),
			     curve_at(&b->cb, t));
	b->has_end[k] = 1;
}

// A corner where two rounds run on into each other smoothly, at places
// i and j of c; the third edge, between two faces that meet smoothly,
// ends where the ball touches them.
static void run_on(struct stage *g, size_t v, const struct corner *c, size_t i,
		   size_t j)
{
	size_t s = DEG_MAX - i - j;
	const struct einfo *x = &g->ei[c->e[i]];
	const struct einfo *y = &g->ei[c->e[j]];
	const struct einfo *z = &g->ei[c->e[s]];
	const struct blend *b = &g->b[g->blend_of[c->e[i]]];
	int k = c->end[i];
	int on_a = x->fa == z->fa || x->fa == z->fb;

	if (x->kind != y->kind) {
		corner_fail(g, v, "turn two ways along one smooth run");
	}
	ball_end(g, c, i, g->sh->v[v]);
	ball_end(g, c, j, g->sh->v[v]);
	set_trim(g, c, s,
		 on_a ? curve_at(&b->ca, b->ta[k]) : curve_at(&b->cb, b->tb[k]),
		 c->e[i]);
}

// The centre of the ball of the stage that touches the three faces at
// corner c, on the centre curve of the round at place i.
static struct vec ball_centre(struct stage *g, size_t v, const struct corner *c,
			      size_t i)
{
	const struct blend *b = &g->b[g->blend_of[c->e[i]]];
	size_t fc = third_face(g, c, c->e[i]);
	double d = g->ei[c->e[i]].kind == K_CONVEX ? -g->r : g->r;
	struct surf oc;
	double tv = param_near(&b->c, g->sh->v[v], b->tm);

	if (fc == NONE || !offset_surf(g->sh, fc, d, &oc)) {
		corner_fail(g, v,
			    "leave nothing of a face, or a face is no plane, "
			    "cylinder or cone");
	}
	return curve_at(&b->c, meet_param(g, b, &b->c, &oc, tv));
}

// A corner of three rounds: a part of a sphere between them.
static void ball_corner(struct stage *g, size_t v, const struct corner *c)
{
	struct vec o = ball_centre(g, v, c, 0);
	struct ball bl;
	struct vec mid = v3(0, 0, 0);

	memset(&bl, 0, sizeof(bl));
	for (size_t i = 0; i < DEG_MAX; i++) {
		const struct blend *b = NULL;
		int k = c->end[i];

		if (g->ei[c->e[i]].kind != g->ei[c->e[0]].kind) {
			corner_fail(g, v, "turn both ways: no ball fits");
		}
		ball_end(g, c, i, o);
		b = &g->b[g->blend_of[c->e[i]]];
		if (len(sub(curve_at(&b->c, b->ta[k]), o)) > NEAR) {
			corner_fail(g, v, "have no common ball");
		}
		// The round runs along its end from its second curve to its
		// first at the end of its edge, the other way at the start;
		// the sphere runs against it.
		bl.arc[i] = b->end[k];
		bl.arc[i].fwd = k ? b->end[k].fwd : !b->end[k].fwd;
		mid = add(mid, sub(seg_from(&bl.arc[i]), o));
	}
	bl.s.kind = SF_SPHERE;
	bl.s.o = o;
	bl.s.r = g->r;
	bl.s.x = unit(mid);
	bl.s.n = unit(cross(bl.s.x, sub(seg_from(&bl.arc[0]), o)));
	bl.sense = g->ei[c->e[0]].kind == K_CONVEX ? 1 : -1;
	bl.f[0] = g->ei[c->e[0]].fa;
	bl.f[1] = g->ei[c->e[0]].fb;
	bl.f[2] = third_face(g, c, c->e[0]);
	PUSH(g->ball, g->nball, g->capball, bl);
}

// A corner where two rounds meet at an angle, at places i and j of c:
// each ends on the other, from the point of the third edge where both
// touch the far faces to the point of the common face where both touch
// it.
static void mitre(struct stage *g, size_t v, const struct corner *c, size_t i,
		  size_t j)
{
	const struct shape *sh = g->sh;
	size_t s = DEG_MAX - i - j;
	struct vec o = ball_centre(g, v, c, i);
	size_t place[2] = { i, j };
	struct vec far[2];
	struct vec common[2];
	struct curve m;

	if (g->ei[c->e[i]].kind != g->ei[c->e[j]].kind) {
		corner_fail(g, v, "turn both ways: no exact corner");
	}
	for (int n = 0; n < 2; n++) {
		size_t e = c->e[place[n]];
		const struct einfo *x = &g->ei[e];
		const struct einfo *y = &g->ei[c->e[place[!n]]];
		struct blend *b = &g->b[g->blend_of[e]];
		int k = c->end[place[n]];
		int a_common = x->fa == y->fa || x->fa == y->fb;
		size_t fcom = a_common ? x->fa : x->fb;
		size_t fo = y->fa == fcom ? y->fb : y->fa;
		double tv = param_near(&b->c, sh->v[v], b->tm);
		double tc = param_near(&b->c, o, b->tm);
		double tf = meet_param(g, b, a_common ? &b->cb : &b->ca,
				       &sh->f[fo].s, tv);

		b->ta[k] = a_common ? tc : tf;
		b->tb[k] = a_common ? tf : tc;
		far[n] = curve_at(a_common ? &b->cb : &b->ca, tf);
		common[n] = curve_at(a_common ? &b->ca : &b->cb, tc);
	}
	if (len(sub(far[0], far[1])) > NEAR ||
	    len(sub(common[0], common[1])) > NEAR) {
		corner_fail(g, v, "do not end at one point: no exact corner");
	}
	set_trim(g, c, s, far[0], c->e[i]);
	if (!curve_through(&g->b[g->blend_of[c->e[i]]].s,
			   &g->b[g->blend_of[c->e[j]]].s, far[0], common[0],
			   &m)) {
		corner_fail(g, v,
			    "meet on no line, circle or conic: no exact "
			    "corner");
	}
	for (int n = 0; n < 2; n++) {
		struct blend *b = &g->b[g->blend_of[c->e[place[n]]]];
		int k = c->end[place[n]];

		b->end[k] = seg_between(&m, curve_at(&b->ca, b->ta[k]),
					curve_at(&b->cb, b->tb[k]));
		b->has_end[k] = 1;
	}
}

// Whether the edges at the two places in of corner c run on into each
// other there: their ways are one line.
static int runs_on(const struct stage *g, const struct corner *c,
		   const size_t *in)
{
	size_t i = in[0];
	size_t j = in[1];
	const struct sedge *a = &g->sh->e[c->e[i]];
	const struct sedge *b = &g->sh->e[c->e[j]];
	struct vec ta = unit(curve_tan(&a->c, c->end[i] ? a->t1 : a->t0));
	struct vec tb = unit(curve_tan(&b->c, c->end[j] ? b->t1 : b->t0));

	return len(cross(ta, tb)) < SMOOTH;
}

// Ends the rounds of the stage at vertex v.
static void end_at_corner(struct stage *g, size_t v, const struct corner *c)
{
	size_t in[CORNER_MAX] = { 0, 0, 0, 0 };
	size_t n = 0;

	for (size_t i = 0; i < c->n && i < CORNER_MAX; i++) {
		if (g->ei[c->e[i]].in) {
			in[n++] = i;
		}
	}
	for (size_t e = 0; e < g->sh->ne && c->n > CORNER_MAX; e++) {
		n += g->ei[e].in &&
		     (g->sh->e[e].v0 == v || g->sh->e[e].v1 == v);
	}
	if (!n) {
		return;
	}
	if (n == 1 && c->n == CORNER_MAX) {
		run_out(g, v, c, in[0]);
		return;
	}
	if (c->n != DEG_MAX) {
		corner_fail(g, v, "meet where more than three edges do");
	}
	if (n == 1) {
		run_out(g, v, c, in[0]);
	} else if (n == DEG_MAX) {
		ball_corner(g, v, c);
	} else if (runs_on(g, c, in)) {
		if (g->ei[c->e[DEG_MAX - in[0] - in[1]]].kind != K_SMOOTH) {
			corner_fail(g, v, "run on past an edge that turns");
		}
		run_on(g, v, c, in[0], in[1]);
	} else {
		mitre(g, v, c, in[0], in[1]);
	}
}

// Whether p, a point of face f of the solid, lies in the strip of f
// that round b takes away: between the edge of b and the curve where
// its ball touches f.
static int in_strip(const struct stage *g, const struct blend *b, size_t f,
		    struct vec p)
{
	const struct einfo *x = &g->ei[b->e];
	int on_a = x->fa == f;
	double t = param_near(&b->c, p, b->tm);
	struct vec here = curve_at(on_a ? &b->ca : &b->cb, t);
	struct vec there = curve_at(on_a ? &b->cb : &b->ca, t);
	struct vec n = out_normal(g->sh, f, here);
	struct vec w = sub(there, here);
	double lo = on_a ? b->ta[0] : b->tb[0];
	double hi = on_a ? b->ta[1] : b->tb[1];
	double d = 0;

	if (t < lo || t > hi) {
		return 0;
	}
	w = unit(sub(w, mul(n, dot(w, n))));
	d = dot(sub(p, here), w);
	return d > NEAR && d < len(sub(there, here)) &&
	       len(sub(sub(p, here), mul(w, d))) < g->r;
}

// Whether the stretch pc of a curve on face f of the solid, a curve
// where a round touches f, meets an edge of f other than e, short of
// its own ends.
static int meets_edge(const struct stage *g, const struct cpiece *pc, size_t f,
		      size_t e)
{
	const struct shape *sh = g->sh;
	struct urange r = face_uses(sh, f);

	for (size_t i = r.first; i < r.end; i++) {
		size_t d = sh->u[i].e;
		size_t other = g->ei[d].fa == f ? g->ei[d].fb : g->ei[d].fa;
		double root[ROOTS_MAX];
		size_t n = d == e || other == f ?
				   0 :
				   curve_surf(pc, &sh->f[other].s, root);

		for (size_t k = 0; k < n; k++) {
			if (edge_dist(&sh->e[d], curve_at(&pc->c, root[k])) <
			    NEAR) {
				return 1;
			}
		}
	}
	return 0;
}

// Dies unless the curve where round b touches its face f, the first
// one if on_a, stays inside f and clear of the strips that the other
// rounds take of f.
static void check_touch(const struct stage *g, const struct blend *b, int on_a)
{
	const struct einfo *x = &g->ei[b->e];
	size_t f = on_a ? x->fa : x->fb;
	const struct curve *c = on_a ? &b->ca : &b->cb;
	double lo = on_a ? b->ta[0] : b->tb[0];
	double hi = on_a ? b->ta[1] : b->tb[1];
	double in = (hi - lo) * INSIDE;
	struct cpiece pc = { *c, lo + in, hi - in };

	if (meets_edge(g, &pc, f, b->e)) {
		edge_fail(g, b->e, "leaves no room on a face");
	}
	for (int i = 1; i < ROOM_PTS; i++) {
		struct vec p =
			curve_at(c, lo + ((hi - lo) * i / (double)ROOM_PTS));

		if (!face_has(g->sh, f, p)) {
			edge_fail(g, b->e, "leaves no room on a face");
		}
		for (size_t k = 0; k < g->nb; k++) {
			const struct einfo *y = &g->ei[g->b[k].e];

			if (&g->b[k] != b && (y->fa == f || y->fb == f) &&
			    in_strip(g, &g->b[k], f, p)) {
				edge_fail(g, b->e,
					  "leaves no room for another round");
			}
		}
	}
}

// Dies unless every round of the stage has room on its two faces.
static void check_room(const struct stage *g)
{
	for (size_t i = 0; i < g->nb; i++) {
		check_touch(g, &g->b[i], 1);
		check_touch(g, &g->b[i], 0);
	}
}

// The stretch of the edge e of the stage, which stays, between its new
// ends.
static struct seg kept_seg(const struct stage *g, size_t e)
{
	const struct sedge *ed = &g->sh->e[e];
	struct seg s = { ed->c, ed->t0, ed->t1, 1, 0 };
	double mid = (ed->t0 + ed->t1) / 2;

	if (len(sub(g->trim[e][0], g->sh->v[ed->v0])) > SH_EPS) {
		s.t0 = param_near(&ed->c, g->trim[e][0], mid);
	}
	if (len(sub(g->trim[e][1], g->sh->v[ed->v1])) > SH_EPS) {
		s.t1 = param_near(&ed->c, g->trim[e][1], mid);
	}
	return s;
}

// The stretch that takes the place of use u in face f: the edge between
// its new ends, or the curve where its round touches f.
static struct seg face_seg(const struct stage *g, const struct suse *u)
{
	const struct blend *b = NULL;
	struct seg s;

	if (g->blend_of[u->e] == NONE) {
		s = kept_seg(g, u->e);
		s.fwd = u->fwd;
		s.gone = len(sub(curve_at(&s.c, s.t0), curve_at(&s.c, s.t1))) <
				 NEAR &&
			 fabs(s.t1 - s.t0) < HALF;
		if (!s.gone && !(s.t1 - s.t0 > SH_EPS)) {
			room_fail(g, u->e);
		}
		return s;
	}
	b = &g->b[g->blend_of[u->e]];
	s.c = u->fwd ? b->ca : b->cb;
	s.t0 = u->fwd ? b->ta[0] : b->tb[0];
	s.t1 = u->fwd ? b->ta[1] : b->tb[1];
	s.fwd = u->fwd;
	s.gone = 0;
	if (!(s.t1 - s.t0 > SH_EPS)) {
		edge_fail(g, u->e, "leaves no room at its ends");
	}
	return s;
}

// The end of a round of the stage that runs from p to q or back; NULL
// if there is none.
static const struct seg *find_end(const struct stage *g, struct vec p,
				  struct vec q, int *back)
{
	for (size_t i = 0; i < g->nb; i++) {
		for (int k = 0; k < 2; k++) {
			const struct seg *s = &g->b[i].end[k];
			struct vec a = seg_from(s);
			struct vec z = seg_to(s);

			if (!g->b[i].has_end[k]) {
				continue;
			}
			if (len(sub(a, p)) < NEAR && len(sub(z, q)) < NEAR) {
				*back = 0;
				return s;
			}
			if (len(sub(a, q)) < NEAR && len(sub(z, p)) < NEAR) {
				*back = 1;
				return s;
			}
		}
	}
	return NULL;
}

// The stretches of loop lp of the old solid that stay, each edge
// replaced by its stretch; *n gets their number.
static struct seg *loop_segs(const struct stage *g, const struct sloop *lp,
			     size_t *n)
{
	struct seg *s = xmalloc((lp->n + 1) * sizeof(*s));

	*n = 0;
	for (size_t k = 0; k < lp->n; k++) {
		s[*n] = face_seg(g, &g->sh->u[lp->first + k]);
		*n += !s[*n].gone;
	}
	return s;
}

// The point half way along stretch s.
static struct vec seg_mid(const struct seg *s)
{
	return curve_at(&s->c, (s->t0 + s->t1) / 2);
}

// Whether the rounds take all of face f: of its one loop a single
// stretch stays, and the end of a round runs along that stretch.  The
// round then goes on into the face beyond, on the edge they share.
static int face_gone(const struct stage *g, size_t f)
{
	const struct sface *fc = &g->sh->f[f];
	size_t n = 0;
	struct seg *s = NULL;
	const struct seg *e = NULL;
	int back = 0;
	int gone = 0;

	if (fc->nloops != 1) {
		return 0;
	}
	s = loop_segs(g, &g->sh->l[fc->loop0], &n);
	if (n == 1) {
		e = find_end(g, seg_to(&s[0]), seg_from(&s[0]), &back);
		gone = e && len(sub(seg_mid(e), seg_mid(&s[0]))) < NEAR;
	}
	free(s);
	return gone;
}

// Dies if the rounds take all of edge e, which stays, and neither of
// its faces goes with it: the round then has no room on that face.
static void refuse_lost(const struct stage *g, size_t e)
{
	struct suse u = { e, 1 };

	if (g->blend_of[e] == NONE && face_seg(g, &u).gone &&
	    !face_gone(g, g->ei[e].fa) && !face_gone(g, g->ei[e].fb)) {
		room_fail(g, e);
	}
}

// Adds loop lp of the old solid to the face under way of out, each edge
// replaced by its stretch, and the ends of rounds put in where two
// stretches do not join.
static void new_loop(const struct stage *g, const struct sloop *lp,
		     struct shape *out)
{
	const struct shape *sh = g->sh;
	size_t first = out->nu;
	size_t n = 0;
	struct seg *s = loop_segs(g, lp, &n);

	for (size_t k = 0; k < lp->n; k++) {
		refuse_lost(g, sh->u[lp->first + k].e);
	}

	for (size_t k = 0; k < n; k++) {
		const struct seg *next = &s[(k + 1) % n];
		struct vec p = seg_to(&s[k]);
		struct vec q = seg_from(next);

		use_seg(out, &s[k], 0);
		if (len(sub(p, q)) > NEAR) {
			int back = 0;
			const struct seg *e = find_end(g, p, q, &back);

			if (e) {
				use_seg(out, e, back);
			} else {
				room_fail(g, sh->u[lp->first].e);
			}
		}
	}
	free(s);
	shape_end_loop(out, first);
}

// Adds the face of round b to out: back along its first curve of touch,
// across at the start of the edge, along the second curve, and across
// at the end.
static void blend_face(const struct stage *g, const struct blend *b,
		       struct shape *out)
{
	const struct einfo *x = &g->ei[b->e];
	char name[NAME_MAX_LEN];
	struct seg sa = { b->ca, b->ta[0], b->ta[1], 0, 0 };
	struct seg sb = { b->cb, b->tb[0], b->tb[1], 1, 0 };
	size_t first = 0;

	xsnprintf(name, "%s+%s", g->sh->f[x->fa].name, g->sh->f[x->fb].name);
	shape_face(out, b->s, b->sense, name);
	first = out->nu;
	use_seg(out, &sa, 0);
	if (b->has_end[0]) {
		use_seg(out, &b->end[0], 0);
	}
	use_seg(out, &sb, 0);
	if (b->has_end[1]) {
		use_seg(out, &b->end[1], 1);
	}
	shape_end_loop(out, first);
}

// Adds the part of a sphere bl to out: its arcs, each after the one it
// starts at the end of.
static void ball_face(const struct stage *g, const struct ball *bl,
		      struct shape *out)
{
	char name[NAME_MAX_LEN];
	size_t first = 0;
	size_t at = 0;

	xsnprintf(name, "%s+%s+%s", g->sh->f[bl->f[0]].name,
		  g->sh->f[bl->f[1]].name, g->sh->f[bl->f[2]].name);
	shape_face(out, bl->s, bl->sense, name);
	first = out->nu;
	for (size_t n = 0; n < DEG_MAX; n++) {
		struct vec to = seg_to(&bl->arc[at]);
		size_t next = at;

		use_seg(out, &bl->arc[at], 0);
		for (size_t i = 0; i < DEG_MAX; i++) {
			if (i != at &&
			    len(sub(seg_from(&bl->arc[i]), to)) < NEAR) {
				next = i;
			}
		}
		at = next;
	}
	shape_end_loop(out, first);
}

// Dies unless every face of out has an area above 0: a face turned
// inside out lost its room to the rounds.
static void check_faces(const struct stage *g, const struct shape *out)
{
	for (size_t f = 0; f < out->nf; f++) {
		if (!(face_mass(out, f).area > 0)) {
			char msg[MSG_LEN];

			xsnprintf(msg,
				  "the rounds of %.6g leave nothing of face "
				  "%s",
				  g->r, out->f[f].name);
			stream_fail(g->st, msg, NULL);
		}
	}
}

// Rounds the edges of sh marked in ei with the radius r, into out.
static void round_stage(struct stream *st, const struct shape *sh,
			struct einfo *ei, double r, struct shape *out)
{
	struct stage g = { st, sh, ei, r, NULL, 0, 0, NULL, NULL, NULL, 0, 0 };
	struct corner *c = corners(sh);

	g.blend_of = xmalloc((sh->ne + 1) * sizeof(*g.blend_of));
	g.trim = xmalloc((sh->ne + 1) * sizeof(*g.trim));
	for (size_t e = 0; e < sh->ne; e++) {
		g.blend_of[e] = NONE;
		g.trim[e][0] = sh->v[sh->e[e].v0];
		g.trim[e][1] = sh->v[sh->e[e].v1];
	}
	for (size_t e = 0; e < sh->ne; e++) {
		if (ei[e].in) {
			make_blend(&g, e);
		}
	}
	for (size_t v = 0; v < sh->nv; v++) {
		end_at_corner(&g, v, &c[v]);
	}
	check_room(&g);
	for (size_t f = 0; f < sh->nf; f++) {
		const struct sface *fc = &sh->f[f];

		if (face_gone(&g, f)) {
			continue;
		}
		shape_face(out, fc->s, fc->sense, fc->name);
		for (size_t l = 0; l < fc->nloops; l++) {
			new_loop(&g, &sh->l[fc->loop0 + l], out);
		}
	}
	for (size_t i = 0; i < g.nb; i++) {
		blend_face(&g, &g.b[i], out);
	}
	for (size_t i = 0; i < g.nball; i++) {
		ball_face(&g, &g.ball[i], out);
	}
	shape_check(out, st->where);
	check_faces(&g, out);
	free(c);
	free(g.b);
	free(g.blend_of);
	free(g.trim);
	free(g.ball);
}

// Prints the edges of sh, with their angles and lengths.
static void print_edges(const struct shape *sh, const struct einfo *ei)
{
	for (size_t e = 0; e < sh->ne; e++) {
		printf("edge %s %s %s %.4f %.4f\n", sh->f[ei[e].fa].name,
		       sh->f[ei[e].fb].name, kind_names[ei[e].kind],
		       ei[e].angle + 0.0, edge_length(&sh->e[e]));
	}
}

// Counts the edges of sh that each rule gave a radius, and finds a rule
// that gives the greatest radius below the last one; marks the edges of
// that radius.  Returns the rule, NONE for none.
static size_t next_rule(struct rules *rs, const struct shape *sh,
			struct einfo *ei, double below)
{
	size_t rule = NONE;
	double r = 0;

	for (size_t e = 0; e < sh->ne; e++) {
		if (ei[e].rule >= rs->n) {
			continue;
		}
		rs->r[ei[e].rule].hits++;
		if (ei[e].r < below && ei[e].r > r) {
			r = ei[e].r;
			rule = ei[e].rule;
		}
	}
	for (size_t e = 0; e < sh->ne; e++) {
		ei[e].in = r > 0 && ei[e].r <= r && ei[e].r >= r;
	}
	return rule;
}

// Rounds solid sh by the rules, the greatest radius first, in place;
// each round of a radius works on the solid that the greater ones left.
// If list, it prints the edges with what the rules need to know, and
// rounds nothing.
static void round_solid(struct rules *rs, struct shape *sh, int list)
{
	double below = HUGE_VAL;

	for (;;) {
		struct einfo *ei = edge_infos(sh);
		struct shape out;
		size_t rule = NONE;

		apply_rules(rs, sh, ei);
		if (list) {
			print_edges(sh, ei);
		} else {
			rule = next_rule(rs, sh, ei, below);
		}
		if (rule >= rs->n) {
			free(ei);
			return;
		}
		below = rs->r[rule].r;
		(void)stream_line(rs->st, rs->r[rule].line);
		shape_init(&out, sh->name);
		round_stage(rs->st, sh, ei, below, &out);
		shape_free(sh);
		*sh = out;
		free(ei);
	}
}

// Reads the rule of line i of the stream, the line under work.
static void read_rule(struct rules *rs, size_t line)
{
	struct stream *st = rs->st;
	struct rule r = { 0, { NULL, NULL }, line, 0 };
	int sharp = stream_is(st, 0, "sharp");
	size_t i = sharp ? 1 : 2;
	int convex = !sharp && st->nw == i + 1 && stream_is(st, i, "convex");

	if (!convex && st->nw != i + 2) {
		stream_fail(st,
			    "round R convex, round R PATTERN PATTERN, or "
			    "sharp PATTERN PATTERN",
			    NULL);
	}
	if (!sharp) {
		r.r = stream_num(st, 1);
		if (!(r.r > SH_EPS)) {
			stream_fail(st, "a round needs a radius above 0", NULL);
		}
	}
	if (!convex) {
		r.pat[0] = xstrdup(st->w[i]);
		r.pat[1] = xstrdup(st->w[i + 1]);
	}
	PUSH(rs->r, rs->n, rs->cap, r);
}

int main(int argc, char **argv)
{
	struct stream st;
	struct rules rs = { NULL, 0, 0, &st };
	int list = argc == 3 && strcmp(argv[1], "-l") == 0;

	if (argc != 2 + list || (!list && argv[1][0] == '-' && argv[1][1])) {
		die("usage: fillet [-l] STREAM|- > stream");
	}
	stream_read(&st, argv[argc - 1]);
	for (size_t i = 0; i < st.nln; i++) {
		if (stream_line(&st, i) && (stream_is(&st, 0, "sharp") ||
					    stream_is(&st, 0, "round"))) {
			read_rule(&rs, i);
			st.ln[i].gone = 1;
		}
	}
	for (size_t i = 0; i < st.nsh; i++) {
		round_solid(&rs, &st.sh[i], list);
	}
	for (size_t i = 0; i < rs.n && !list; i++) {
		if (!rs.r[i].hits) {
			(void)stream_line(&st, rs.r[i].line);
			stream_fail(&st, "this rule matches no edge", NULL);
		}
	}
	if (!list) {
		stream_write(&st);
	}
	for (size_t i = 0; i < rs.n; i++) {
		free(rs.r[i].pat[0]);
		free(rs.r[i].pat[1]);
	}
	free(rs.r);
	stream_free(&st);
	close_stdout();
	return 0;
}
