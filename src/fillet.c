// SPDX-License-Identifier: GPL-3.0-only
// fillet.c -- Round the edges of exact solids, as a ball rolls along them
// Copyright (c) 2026 Jakob Kastelic

// Usage: fillet [-l] STREAM|- > stream
//
// Reads a stream (stream.h) and takes its statements
//   round R convex
//   round R FACES FACES
//   sharp FACES FACES
// which give each edge of each solid a radius.  An edge lies between two
// faces, and FACES names faces by the names of the source (stream_face
// in stream.h): any, a set of a faces statement, a solid of extrude, or
// its start, end, sides or the side of a named corner.  Two of them pick
// the edges between two such faces, and convex those that turn outward.
// The last rule that matches an edge wins; sharp gives the radius 0.  A
// rule that matches no edge is an error.  Edges where two faces meet smoothly have no rule, and an
// edge between two faces that rounds made has only the rules that name
// faces.
//
// The round of an edge is the surface that a ball of the radius of the
// edge sweeps as it rolls along it, in touch with both faces, which are
// planes, cylinders or cones: a cylinder along a line, a torus along a
// circle, and a pipe along an ellipse, where a plane meets a cylinder
// at a slant.  The ball touches a plane or a cylinder along an ellipse
// too, the image of that of its center by an affine map: the plane its
// shadow along the normal, the cylinder its own scaled toward the axis.
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
// does a radius that leaves no room for a neighbor face or edge, with
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
// outward), its line in the stream, the edges it gave a radius, those it
// fits, and the last rule that took one of those from it, or NONE.
struct rule {
	double r;
	char *pat[2];
	size_t line;
	size_t hits;
	size_t fits;
	size_t over;
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

// The round of an edge: the curve of the center of the ball, in the way
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
	size_t split[2]; // The edges more than one that the end runs on,
	double ts[2][2]; // split at these parameters, the lower first.
	// At a miter past a crease (miter_past), the end goes on from the
	// miter along tail, toward the first curve if tail_at is TAIL_A,
	// toward the second if TAIL_B; and a curve of touch that runs past
	// the edge of its face into the sliver that the other round adds
	// to it, the first if past is TAIL_A, the second if TAIL_B, lies
	// on that face up to tpast.
	struct seg tail[2];
	int tail_at[2];
	int past[2];
	double tpast[2];
};

enum { TAIL_NONE, TAIL_A, TAIL_B };

// A part of a sphere at a corner: its three arcs, in order.
struct ball {
	struct surf s;
	int sense;
	size_t f[DEG_MAX];
	struct seg arc[DEG_MAX];
};

// A piece of the plane of a miter, where two rounds meet across an edge
// whose two faces do not meet the plane in one line: the two lines from
// the far ends of the rounds to the point where the edge meets the
// plane, and the stretch of the miter between those ends; or, if face
// is 0, no face, and the two sides are stretches of the miter itself.
struct patch {
	struct surf s;
	int sense;
	int face;
	size_t f[DEG_MAX];
	struct seg side[DEG_MAX];
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
	struct patch *patch;
	size_t npatch, cappatch;
};

static int rule_fits(const struct stream *st, const struct rule *r,
		     const char *na, const char *nb, int kind)
{
	struct face_ref a = { st, r->pat[0] };
	struct face_ref b = { st, r->pat[1] };

	if (!r->pat[0]) {
		return kind == K_CONVEX &&
		       !(strchr(na, '+') && strchr(nb, '+'));
	}
	return (stream_face(a, na) && stream_face(b, nb)) ||
	       (stream_face(a, nb) && stream_face(b, na));
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
			if (rule_fits(rs->st, &rs->r[i], na, nb, ei[e].kind)) {
				if (ei[e].rule < rs->n) {
					rs->r[ei[e].rule].over = i;
				}
				rs->r[i].fits++;
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

// The parameter of p on curve c, an angle within half a turn of ref: of
// the point of c whose normal plane holds p.
static double param_near(const struct curve *c, struct vec p, double ref)
{
	double t = curve_near(c, p);

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

// An affine map p -> o + l p, l by its columns.
struct affine {
	struct vec o, col[3];
};

static struct vec aff_dir(const struct affine *m, struct vec v)
{
	return add(add(mul(m->col[0], v.x), mul(m->col[1], v.y)),
		   mul(m->col[2], v.z));
}

// The map that takes the center of a ball to where it touches s, a plane
// or a cylinder, for the centers r from it: along the normal of the
// plane, or toward the axis of the cylinder by its radius over r.
static int touch_map(const struct surf *s, double r, struct affine *m)
{
	const struct vec ax[3] = { v3(1, 0, 0), v3(0, 1, 0), v3(0, 0, 1) };
	double k = r > SH_EPS ? s->r / r : 0;

	for (int i = 0; i < 3; i++) {
		struct vec along = mul(s->n, dot(ax[i], s->n));

		m->col[i] = s->kind == SF_PLANE ?
				    sub(ax[i], along) :
				    add(along, mul(sub(ax[i], along), k));
	}
	m->o = sub(s->o, aff_dir(m, s->o));
	return s->kind == SF_PLANE || (s->kind == SF_CYL && r > SH_EPS);
}

// The ellipse where the ball with its center on ellipse c touches
// surface s, a plane or a cylinder, the image of c by touch_map in the
// parameter of c: o' + a l x cos t + b l y sin t, whose two vectors must
// be its half axes, so that it has a frame; 0 if it has none.
static int ellipse_touch(const struct curve *c, const struct surf *s,
			 struct curve *out)
{
	struct affine m;
	struct vec y = frame_y(c->n, c->x);
	struct vec p0 = curve_at(c, 0);
	struct vec d0 = sub(p0, s->o);
	double r = len(sub(d0, mul(s->n, dot(d0, s->n))));
	struct vec a;
	struct vec b;
	double la = 0;
	double lb = 0;

	if (!touch_map(s, r, &m)) {
		return 0;
	}
	a = mul(aff_dir(&m, c->x), c->a);
	b = mul(aff_dir(&m, y), c->b);
	la = len(a);
	lb = len(b);
	if (!(lb > SH_EPS) || fabs(dot(a, b)) > SMOOTH * la * lb ||
	    la < lb * (1 - SMOOTH)) {
		return 0;
	}
	*out = *c;
	out->o = add(m.o, aff_dir(&m, c->o));
	out->x = mul(a, 1 / la);
	out->n = unit(cross(a, b));
	out->a = la;
	out->b = fmin(lb, la);
	return 1;
}

// The curve where the ball with its center on c, a line, a circle or an
// ellipse, touches surface s; 0 if it is no curve of the kind of c.
static int contact_curve(const struct curve *c, const struct surf *s,
			 struct curve *out)
{
	struct vec f0 = foot(s, curve_at(c, 0));

	*out = *c;
	if (c->kind == CV_ELLIPSE) {
		int ok = ellipse_touch(c, s, out);

		for (int i = 1; ok && i < ROOM_PTS; i++) {
			double t = 2 * PI * i / (double)ROOM_PTS;

			ok = len(sub(foot(s, curve_at(c, t)),
				     curve_at(out, t))) < NEAR;
		}
		return ok;
	}
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

// Whether the tube of the pipe of round b folds over itself where the
// ball runs between its two curves of touch: where rt times the cosine
// of v below 0 reaches the radius of curvature of the ellipse, w^3 /
// (a b), which leaves the nearest point of the ellipse to a point of the
// face another one (shape.h).  The arc from one curve to the other runs
// the shorter way; v there is the angle from the normal w of the
// ellipse toward its axis n.
static int pipe_folds(const struct blend *b)
{
	enum { FOLD_PTS = 64 };
	const struct surf *s = &b->s;

	for (int i = 0; i < FOLD_PTS; i++) {
		double u = 2 * PI * i / (double)FOLD_PTS;
		struct line3 ring = pipe_ring(s, u);
		struct vec da = sub(curve_at(&b->ca, u), ring.o);
		struct vec db = sub(curve_at(&b->cb, u), ring.o);
		double va = atan2(dot(da, s->n), dot(da, ring.d));
		double vb =
			angle_near(atan2(dot(db, s->n), dot(db, ring.d)), va);
		double lo = fmin(va, vb);
		double hi = fmax(va, vb);
		double low = fmin(cos(va), cos(vb));
		double su = sin(u);
		double cu = cos(u);
		double w2 = (s->r * s->r * su * su) + (s->r3 * s->r3 * cu * cu);

		double back = angle_near(PI, (lo + hi) / 2);

		if (back >= lo && back <= hi) {
			low = -1;
		}
		if (-s->r2 * low > (w2 * sqrt(w2) / (s->r * s->r3)) - NEAR) {
			return 1;
		}
	}
	return 0;
}

// The radius of face f, a cylinder or a cone, at the height of p along
// its axis.
static double face_radius(const struct sface *f, struct vec p)
{
	double h = dot(sub(p, f->s.o), f->s.n);

	return f->s.kind == SF_CONE ? f->s.r + (f->s.r2 * h) : f->s.r;
}

// Dies if face fc of edge e, a cylinder or a cone, moved out by d along
// its outward normal, has no radius left where the center of the ball
// runs, near cm: the round is larger than the face, as at a corner of a
// drafted profile that narrows toward one end.  m is the middle of the
// edge, where the radius of the face is told.
static void corner_too_small(const struct stage *g, size_t e,
			     const struct sface *fc, double d, struct vec m,
			     struct vec cm)
{
	double dn = d * fc->sense;
	double slant = fc->s.kind == SF_CONE ? hypot(1, fc->s.r2) : 1;
	char msg[MSG_LEN];

	if ((fc->s.kind != SF_CYL && fc->s.kind != SF_CONE) ||
	    face_radius(fc, cm) + (dn * slant) > SH_EPS) {
		return;
	}
	xsnprintf(msg,
		  "is larger than the corner of %s there, of radius %.6g at "
		  "%.6g %.6g %.6g",
		  fc->name, face_radius(fc, m), m.x + 0.0, m.y + 0.0,
		  m.z + 0.0);
	edge_fail(g, e, msg);
}

// The surface of round b of the stage, round its curve of the center:
// a pipe round an ellipse, a cylinder round a line, a torus round a
// circle.
static void blend_surf(const struct stage *g, struct blend *b)
{
	if (b->c.kind == CV_ELLIPSE) {
		b->s.kind = SF_PIPE;
		b->s.n = b->c.n;
		b->s.x = b->c.x;
		b->s.r = b->c.a;
		b->s.r3 = b->c.b;
		b->s.r2 = g->r;
		if (pipe_folds(b)) {
			edge_fail(g, b->e,
				  "folds its tube over itself: the radius "
				  "passes that of the curve of its center");
		}
	} else if (b->c.kind == CV_LINE) {
		b->s.kind = SF_CYL;
		b->s.n = b->c.x;
		b->s.x = unit(sub(curve_at(&b->ca, 0), b->c.o));
		b->s.r = g->r;
	} else {
		b->s.kind = SF_TORUS;
		b->s.n = b->c.n;
		b->s.x = b->c.x;
		b->s.r = b->c.a;
		b->s.r2 = g->r;
	}
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
		corner_too_small(g, e, &sh->f[x->fa], d, m, cm);
		corner_too_small(g, e, &sh->f[x->fb], d, m, cm);
		edge_fail(g, e,
			  "leaves nothing of a face, or a face is no plane, "
			  "cylinder or cone");
	}
	if (surf_surf(&oa, &ob, cand, &n) != SS_CURVES) {
		n = 0;
	}
	// cm lies on the curve of the center where both faces are planes,
	// and near it, to the second order in the radius over that of the
	// faces, where one is curved: the nearest curve within the radius.
	for (size_t i = 0; i < n; i++) {
		double gap = curve_dist(&cand[i], cm);

		if (cand[i].kind != CV_NODAL && gap < g->r &&
		    (pick == NONE || gap < curve_dist(&cand[pick], cm))) {
			pick = i;
		}
	}
	if (pick == NONE) {
		corner_too_small(g, e, &sh->f[x->fa], d, m, cm);
		corner_too_small(g, e, &sh->f[x->fb], d, m, cm);
		edge_fail(g, e, "follows no line, circle or ellipse");
	}
	b.c = cand[pick];
	b.tm = curve_near(&b.c, cm);
	if (dot(curve_tan(&b.c, b.tm), curve_tan(&ed->c, tmid)) < 0) {
		if (b.c.kind == CV_LINE) {
			b.c.x = mul(b.c.x, -1);
		} else {
			b.c.n = mul(b.c.n, -1);
		}
		b.tm = curve_near(&b.c, cm);
	}
	if (!contact_curve(&b.c, &sh->f[x->fa].s, &b.ca) ||
	    !contact_curve(&b.c, &sh->f[x->fb].s, &b.cb)) {
		edge_fail(g, e, "touches a face on no line, circle or ellipse");
	}
	b.s.o = b.c.o;
	b.sense = x->kind == K_CONVEX ? 1 : -1;
	blend_surf(g, &b);
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

// The parameter of the center of round b nearest t where c, a curve
// that goes with the center (the center itself or a curve of touch),
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

// The parameter of curve c of round b where it meets surface s, nearest
// t, the end of the round at a corner, of those along the round from
// there to its other end: the round ends short of its corner, not past
// it.
static double meet_inside(const struct stage *g, const struct blend *b,
			  const struct curve *c, const struct surf *s, double t)
{
	double reach = REACH * g->r;
	double lo = fmin(t, (2 * b->tm) - t);
	double hi = fmax(t, (2 * b->tm) - t);
	struct cpiece pc = { *c, lo - NEAR, hi + NEAR };
	double root[ROOTS_MAX];
	size_t n = 0;
	double best = HUGE_VAL;

	if (curve_turns(c)) {
		pc.t0 = fmax(pc.t0, t - fmin(PI - SMOOTH, reach / c->a));
		pc.t1 = fmin(pc.t1, t + fmin(PI - SMOOTH, reach / c->a));
	}
	n = curve_surf(&pc, s, root);
	for (size_t i = 0; i < n; i++) {
		if (fabs(root[i] - t) < fabs(best - t)) {
			best = root[i];
		}
	}
	return best < HUGE_VAL ? best : meet_param(g, b, c, s, t);
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
		edge_fail(g, cause, "leaves no room for a neighbor edge");
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
// The parameter of the point where the curve of touch t of round b,
// on a face that runs on smoothly into face f along the line or curve of
// the edge ed, touches f: where it meets the plane across f through the
// point of ed nearest the corner p.
static double touch_param(const struct stage *g, const struct blend *b,
			  const struct curve *t, const struct sedge *ed,
			  size_t f, struct vec p, double ref)
{
	double te = param_near(&ed->c, p, (ed->t0 + ed->t1) / 2);
	struct vec q = curve_at(&ed->c, te);
	struct surf across;

	memset(&across, 0, sizeof(across));
	across.kind = SF_PLANE;
	across.o = q;
	across.n = unit(cross(curve_tan(&ed->c, te), out_normal(g->sh, f, q)));
	across.x = unit(curve_tan(&ed->c, te));
	return meet_param(g, b, t, &across, ref);
}

// The nodal section where torus s meets plane p, if p runs along its
// axis and touches its inner circle, through points a and z; 0 if it is
// none.
static int nodal_through(const struct surf *s, const struct surf *p,
			 struct vec a, struct vec z, struct curve *out)
{
	double d = dot(sub(s->o, p->o), p->n);
	struct curve c;

	if (s->kind != SF_TORUS || p->kind != SF_PLANE ||
	    fabs(dot(s->n, p->n)) > SMOOTH ||
	    fabs(fabs(d) - (s->r - s->r2)) > NEAR) {
		return 0;
	}
	memset(&c, 0, sizeof(c));
	c.kind = CV_NODAL;
	c.o = s->o;
	c.n = s->n;
	c.x = mul(p->n, d > 0 ? -1 : 1);
	c.a = s->r;
	c.b = s->r2;
	if (curve_dist(&c, a) > NEAR || curve_dist(&c, z) > NEAR) {
		return 0;
	}
	*out = c;
	return 1;
}

// Of 0 and a whole turn either way, the one nearest d.
static double turn_shift(double d)
{
	double best = 0;

	for (int k = -1; k <= 1; k += 2) {
		double u = k * 2 * PI;

		if (fabs(u - d) < fabs(best - d)) {
			best = u;
		}
	}
	return best;
}

// The stretch of nodal section c from a to z: the parameter of the one
// nearer the node, where both halves meet, taken a turn on if that
// brings it nearer the other.
static struct seg nodal_seg(const struct curve *c, struct vec a, struct vec z)
{
	double ta = curve_param(c, a);
	double tz = curve_param(c, z);
	struct seg s = { *c, 0, 0, 1, 0 };

	if (fabs(ta) < fabs(tz)) {
		ta += turn_shift(tz - ta);
	} else {
		tz += turn_shift(ta - tz);
	}
	s.t0 = fmin(ta, tz);
	s.t1 = fmax(ta, tz);
	s.fwd = ta <= tz;
	return s;
}

// The end of the round at place i of corner c, where one of its faces
// runs on smoothly into the face f that the other face meets at an edge
// that stays, the face of the corner across both: the round ends on f,
// its curve of touch on the smooth face where that touches f, and the
// other where it meets f.
static void run_into(struct stage *g, size_t v, const struct corner *c,
		     size_t i)
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
	int smooth_a = g->ei[c->e[ga]].kind == K_SMOOTH;
	struct vec pa;
	struct vec pb;
	struct curve end;
	size_t f = fca;

	if (smooth_a) {
		b->ta[k] = touch_param(g, b, &b->ca, &sh->e[c->e[ga]], f,
				       sh->v[v], tv);
		b->tb[k] = meet_inside(g, b, &b->cb, &sh->f[f].s, tv);
	} else {
		b->ta[k] = meet_inside(g, b, &b->ca, &sh->f[f].s, tv);
		b->tb[k] = touch_param(g, b, &b->cb, &sh->e[c->e[gb]], f,
				       sh->v[v], tv);
	}
	pa = curve_at(&b->ca, b->ta[k]);
	pb = curve_at(&b->cb, b->tb[k]);
	set_trim(g, c, ga, pa, e);
	set_trim(g, c, gb, pb, e);
	if (curve_through(&b->s, &sh->f[f].s, pa, pb, &end)) {
		b->end[k] = seg_between(&end, pa, pb);
	} else if (nodal_through(&b->s, &sh->f[f].s, pa, pb, &end)) {
		b->end[k] = nodal_seg(&end, pa, pb);
	} else {
		corner_fail(g, v,
			    "end on a face along no line, circle, conic or "
			    "nodal section");
	}
	b->has_end[k] = 1;
}

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
	if (c->n == DEG_MAX && fca == fcb && smooth_a != smooth_b) {
		run_into(g, v, c, i);
		return;
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
// center at o: both curves of touch end where that ball touches.
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
// ends where the ball touches them.  The two rounds share the ball whose
// center lies where the curve of the center of the first crosses the
// plane of the third edge and the normal of its faces: there the ball
// touches both faces on the edge, which the plane square to the curve
// through the corner does not hold where the faces lean to it, as at a
// draft.
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
	double tv = param_near(&b->c, g->sh->v[v], b->tm);
	struct vec o =
		curve_at(&b->c, touch_param(g, b, &b->c, &g->sh->e[c->e[s]],
					    z->fa, g->sh->v[v], tv));

	if (x->kind != y->kind) {
		corner_fail(g, v, "turn two ways along one smooth run");
	}
	ball_end(g, c, i, o);
	ball_end(g, c, j, o);
	set_trim(g, c, s,
		 on_a ? curve_at(&b->ca, b->ta[k]) : curve_at(&b->cb, b->tb[k]),
		 c->e[i]);
}

// The center of the ball of the stage that touches the three faces at
// corner c, on the center curve of the round at place i.
static struct vec ball_center(struct stage *g, size_t v, const struct corner *c,
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
	struct vec o = ball_center(g, v, c, 0);
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

// The plane of conic c, a circle or an ellipse.
static struct surf conic_plane(const struct curve *c)
{
	struct surf p;

	memset(&p, 0, sizeof(p));
	p.kind = SF_PLANE;
	p.o = c->o;
	p.n = c->n;
	p.x = c->x;
	return p;
}

// The miter of the rounds ba and bb, which meet at the ball with its
// center at o: the curve where they meet through p, whose plane has the
// two rounds on its two sides; 0 if there is none.
static int miter_curve(const struct blend *ba, struct vec o,
		       const struct blend *bb, struct vec p, struct curve *out)
{
	struct curve cand[2];
	size_t n = 0;
	struct vec da = sub(curve_at(&ba->c, ba->tm), o);
	struct vec db = sub(curve_at(&bb->c, bb->tm), o);

	if (surf_surf(&ba->s, &bb->s, cand, &n) != SS_CURVES) {
		return 0;
	}
	for (size_t i = 0; i < n; i++) {
		if (cand[i].kind != CV_LINE && curve_dist(&cand[i], p) < NEAR &&
		    dot(cand[i].n, da) * dot(cand[i].n, db) < 0) {
			*out = cand[i];
			return 1;
		}
	}
	return 0;
}

// The point nearest p where the edge at place s of corner c meets plane
// m; dies if there is none.
static struct vec edge_meets(const struct stage *g, size_t v,
			     const struct corner *c, size_t s,
			     const struct surf *m)
{
	const struct sedge *ed = &g->sh->e[c->e[s]];
	struct cpiece pc = { ed->c, ed->t0, ed->t1 };
	double root[ROOTS_MAX];
	size_t n = curve_surf(&pc, m, root);
	struct vec best = g->sh->v[v];
	double d = HUGE_VAL;

	for (size_t i = 0; i < n; i++) {
		struct vec q = curve_at(&ed->c, root[i]);

		if (len(sub(q, g->sh->v[v])) < d) {
			d = len(sub(q, g->sh->v[v]));
			best = q;
		}
	}
	if (!(d < REACH * g->r)) {
		corner_fail(g, v, "meet an edge that misses the miter");
	}
	return best;
}

// The farthest that the stretch s of a miter strays from the lines from
// a to q and from q to z.
static double stray(const struct seg *s, struct vec a, struct vec q,
		    struct vec z)
{
	double most = 0;

	for (int i = 1; i < ROOM_PTS; i++) {
		struct vec p = curve_at(&s->c, s->t0 + ((s->t1 - s->t0) * i /
							(double)ROOM_PTS));
		struct curve l1 = { CV_LINE,	     a, v3(0, 0, 0),
				    unit(sub(q, a)), 0, 0 };
		struct curve l2 = { CV_LINE,	     q, v3(0, 0, 0),
				    unit(sub(z, q)), 0, 0 };

		most = fmax(most, fmin(curve_dist(&l1, p), curve_dist(&l2, p)));
	}
	return most;
}

// A side of a patch: the line from a to z.
static struct seg line_seg(struct vec a, struct vec z)
{
	struct curve ln;

	memset(&ln, 0, sizeof(ln));
	ln.kind = CV_LINE;
	ln.o = a;
	ln.x = unit(sub(z, a));
	return (struct seg){ ln, 0, len(sub(z, a)), 1, 0 };
}

// The gap at the miter m between the far ends of two rounds that do not
// meet there, at places i and j of corner c, whose far ends are far[0]
// and far[1] and whose common point is common.  The round with the far
// end farther along m from common, the long one, ends on the miter in
// edges split at the far end of the other.  The plane of m closes the
// gap up to the point q where the edge at place s, which stays, meets
// it; if the miter strays less than SH_EPS from the lines of that plane,
// the faces of those lines end on the miter itself, split at the point
// of it nearest q.
static void miter_patch(struct stage *g, size_t v, const struct corner *c,
			const size_t *place, const struct curve *m,
			const struct vec *far, struct vec common)
{
	size_t s = DEG_MAX - place[0] - place[1];
	struct patch pt;
	struct surf mp = conic_plane(m);
	struct vec q = edge_meets(g, v, c, s, &mp);
	double tc = curve_param(m, common);
	double tf[2];
	int lng = 0;
	struct blend *b = NULL;
	int k = 0;
	struct vec in;
	struct seg gap;

	for (int n = 0; n < 2; n++) {
		tf[n] = curve_turns(m) ?
				angle_near(curve_param(m, far[n]), tc) :
				curve_param(m, far[n]);
	}
	lng = fabs(tf[1] - tc) > fabs(tf[0] - tc);
	b = &g->b[g->blend_of[c->e[place[lng]]]];
	k = c->end[place[lng]];
	memset(&pt, 0, sizeof(pt));
	gap = seg_between(m, far[!lng], far[lng]);
	pt.face = stray(&gap, far[lng], q, far[!lng]) >= SH_EPS;
	if (!pt.face) {
		double tq = param_near(m, q, (gap.t0 + gap.t1) / 2);

		q = curve_at(m, tq);
		pt.side[0] = seg_between(m, far[lng], q);
		pt.side[1] = seg_between(m, q, far[!lng]);
		b->ts[k][1] = angle_near(tq, b->end[k].t0);
		b->split[k] = 2;
	} else {
		pt.side[0] = line_seg(far[lng], q);
		pt.side[1] = line_seg(q, far[!lng]);
		pt.side[2] = gap;
		b->split[k] = 1;
	}
	set_trim(g, c, s, q, c->e[place[0]]);
	b->ts[k][0] = curve_turns(m) ? angle_near(tf[!lng], b->end[k].t0) :
				       tf[!lng];
	if (b->split[k] == 2 && b->ts[k][1] < b->ts[k][0]) {
		double t = b->ts[k][0];

		b->ts[k][0] = b->ts[k][1];
		b->ts[k][1] = t;
	}
	pt.s = mp;
	in = sub(curve_at(&b->c, b->tm),
		 curve_at(&b->c, param_near(&b->c, q, b->tm)));
	pt.sense = dot(mp.n, in) > 0 ? 1 : -1;
	pt.f[0] = g->ei[c->e[place[0]]].fa;
	pt.f[1] = g->ei[c->e[place[0]]].fb;
	pt.f[2] = third_face(g, c, c->e[place[0]]);
	PUSH(g->patch, g->npatch, g->cappatch, pt);
}

// The gap at the miter m between the far ends of two rounds at places
// place of corner c whose far curves fc[n] meet the far face of the
// other, fo[n], at on[n], short of where they meet the plane of m, at
// far[n]: their far faces meet at a crease that turns the other way
// from the rounds, as where a drafted face runs past a step.  Of the
// two cylinders, each touches the far face of its round along its far
// curve and lies off it elsewhere, so m only touches each far plane, at
// the far point of its round.  The round whose far point comes first
// along m from the common face, the short one, ends on m there; its far
// curve runs on past the crease by a sliver of its far face.  The long
// one ends at its point on the crease, on[n]: along m to the far point
// of the short one, which lies on the far face of the short one, then
// along the curve where it meets that face.  That face then closes the
// sliver, below the crease, where the long round takes the solid away,
// and the crease ends where the long round meets it.
static void miter_past(struct stage *g, size_t v, const struct corner *c,
		       const size_t *place, const struct curve *m,
		       const struct vec *far, const double *on,
		       const struct curve **fc, const size_t *fo,
		       struct vec common)
{
	size_t s = DEG_MAX - place[0] - place[1];
	struct blend *bs = NULL;
	struct blend *bl = NULL;
	double tc = curve_param(m, common);
	double tf[2];
	int lng = 0;
	int ks = 0;
	int kl = 0;
	int far_b = 0;
	struct vec pl;
	struct vec pc;
	struct curve cv;

	for (int n = 0; n < 2; n++) {
		tf[n] = curve_turns(m) ?
				angle_near(curve_param(m, far[n]), tc) :
				curve_param(m, far[n]);
	}
	lng = fabs(tf[1] - tc) > fabs(tf[0] - tc);
	bs = &g->b[g->blend_of[c->e[place[!lng]]]];
	ks = c->end[place[!lng]];
	bs->past[ks] = fc[!lng] == &bs->cb ? TAIL_B : TAIL_A;
	bs->tpast[ks] = on[!lng];
	bl = &g->b[g->blend_of[c->e[place[lng]]]];
	kl = c->end[place[lng]];
	far_b = fc[lng] == &bl->cb;
	(far_b ? bl->tb : bl->ta)[kl] = on[lng];
	pl = curve_at(fc[lng], on[lng]);
	pc = far_b ? curve_at(&bl->ca, bl->ta[kl]) :
		     curve_at(&bl->cb, bl->tb[kl]);
	if (!curve_through(&bl->s, &g->sh->f[fo[lng]].s, far[!lng], pl, &cv)) {
		corner_fail(g, v,
			    "go past their crease along no line, circle or "
			    "conic: no exact corner");
	}
	if (far_b) {
		bl->end[kl] = seg_between(m, pc, far[!lng]);
		bl->tail[kl] = seg_between(&cv, far[!lng], pl);
		bl->tail_at[kl] = TAIL_B;
	} else {
		bl->end[kl] = seg_between(m, far[!lng], pc);
		bl->tail[kl] = seg_between(&cv, pl, far[!lng]);
		bl->tail_at[kl] = TAIL_A;
	}
	set_trim(g, c, s, pl, c->e[place[lng]]);
}

// A corner where two rounds meet at an angle, at places i and j of c:
// each ends on the other, along their miter, from the point of the
// common face where both touch it to the point where it meets its far
// face.  If those far points are one, the edge between the far faces
// ends there; else a piece of the plane of the miter fills the gap
// (miter_patch), or, past a crease that turns the other way, a sliver
// of a far face (miter_past).
// The first ends of the rounds at places place of corner c, which meet
// at the ball with its center at o: each from its point on the common
// face, common[n], to where its far curve, fc[n], meets the far face of
// the other round, fo[n].
static void miter_ends(struct stage *g, size_t v, const struct corner *c,
		       const size_t *place, struct vec o, struct vec *common,
		       const struct curve **fc, size_t *fo)
{
	const struct shape *sh = g->sh;

	for (int n = 0; n < 2; n++) {
		size_t e = c->e[place[n]];
		const struct einfo *x = &g->ei[e];
		const struct einfo *y = &g->ei[c->e[place[!n]]];
		struct blend *b = &g->b[g->blend_of[e]];
		int k = c->end[place[n]];
		int a_common = x->fa == y->fa || x->fa == y->fb;
		size_t fcom = a_common ? x->fa : x->fb;
		double tc = param_near(&b->c, o, b->tm);
		double tv = param_near(&b->c, sh->v[v], b->tm);

		fo[n] = y->fa == fcom ? y->fb : y->fa;
		fc[n] = a_common ? &b->cb : &b->ca;
		(a_common ? b->ta : b->tb)[k] = tc;
		(a_common ? b->tb : b->ta)[k] =
			meet_param(g, b, fc[n], &sh->f[fo[n]].s, tv);
		common[n] = curve_at(a_common ? &b->ca : &b->cb, tc);
	}
}

static void miter(struct stage *g, size_t v, const struct corner *c, size_t i,
		  size_t j)
{
	size_t s = DEG_MAX - i - j;
	struct vec o = ball_center(g, v, c, i);
	const size_t place[2] = { i, j };
	struct vec far[2];
	struct vec common[2];
	const struct curve *fc[2];
	size_t fo[2];
	double on[2];
	int past[2];
	struct curve m;
	struct surf mp;

	if (g->ei[c->e[i]].kind != g->ei[c->e[j]].kind) {
		corner_fail(g, v, "turn both ways: no exact corner");
	}
	miter_ends(g, v, c, place, o, common, fc, fo);
	if (len(sub(common[0], common[1])) > NEAR) {
		corner_fail(g, v, "do not end at one point: no exact corner");
	}
	if (!miter_curve(&g->b[g->blend_of[c->e[i]]], o,
			 &g->b[g->blend_of[c->e[j]]], common[0], &m)) {
		corner_fail(g, v,
			    "meet on no line, circle or conic: no exact "
			    "corner");
	}
	// Each round ends where its far curve meets the plane of the miter.
	// That point lies past the crease of the far faces, off its face,
	// where the far curve meets the far face of the other round nearer
	// the middle of its edge than the plane.
	mp = conic_plane(&m);
	for (int n = 0; n < 2; n++) {
		size_t e = c->e[place[n]];
		const struct einfo *x = &g->ei[e];
		const struct einfo *y = &g->ei[c->e[place[!n]]];
		struct blend *b = &g->b[g->blend_of[e]];
		int k = c->end[place[n]];
		int a_common = x->fa == y->fa || x->fa == y->fb;
		double *t = a_common ? b->tb : b->ta;

		on[n] = t[k];
		t[k] = meet_param(g, b, fc[n], &mp, t[k]);
		past[n] = fabs(on[n] - b->tm) < fabs(t[k] - b->tm);
		far[n] = curve_at(fc[n], t[k]);
		b->end[k] = seg_between(&m, curve_at(&b->ca, b->ta[k]),
					curve_at(&b->cb, b->tb[k]));
		b->has_end[k] = 1;
	}
	if (!(len(sub(far[0], far[1])) > NEAR)) {
		set_trim(g, c, s, far[0], c->e[i]);
	} else if (past[0] != past[1]) {
		corner_fail(g, v,
			    "cross their crease on one side of the miter only: "
			    "no exact corner");
	} else if (past[0]) {
		miter_past(g, v, c, place, &m, far, on, fc, fo, common[0]);
	} else {
		miter_patch(g, v, c, place, &m, far, common[0]);
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
		miter(g, v, c, in[0], in[1]);
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

	// A point at an end of the strip, where it meets another round at a
	// corner, lies on its edge, not in it.
	if (t < lo + NEAR || t > hi - NEAR) {
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
	int self = on_a ? TAIL_A : TAIL_B;
	// A curve that runs past the edge of f into a sliver at a miter
	// (miter_past) lies on f up to that edge.
	double lo = on_a ? b->ta[0] : b->tb[0];
	double hi = on_a ? b->ta[1] : b->tb[1];
	double in = 0;
	struct cpiece pc;

	if (b->past[0] == self) {
		lo = b->tpast[0];
	}
	if (b->past[1] == self) {
		hi = b->tpast[1];
	}
	in = (hi - lo) * INSIDE;
	pc = (struct cpiece){ *c, lo + in, hi - in };

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

// Whether stretch s runs from p to q, or back if *back.
static int joins(const struct seg *s, struct vec p, struct vec q, int *back)
{
	struct vec a = seg_from(s);
	struct vec z = seg_to(s);

	*back = len(sub(a, q)) < NEAR && len(sub(z, p)) < NEAR;
	return *back || (len(sub(a, p)) < NEAR && len(sub(z, q)) < NEAR);
}

// The end of a round of the stage, a piece of it past a miter, or a side
// of a patch, that runs from p to q or back; NULL if there is none.
static const struct seg *find_end(const struct stage *g, struct vec p,
				  struct vec q, int *back)
{
	for (size_t i = 0; i < g->npatch; i++) {
		for (int k = 0; k < 2; k++) {
			if (joins(&g->patch[i].side[k], p, q, back)) {
				return &g->patch[i].side[k];
			}
		}
	}
	for (size_t i = 0; i < g->nb; i++) {
		const struct blend *b = &g->b[i];

		for (int k = 0; k < 2; k++) {
			if (b->has_end[k] && joins(&b->end[k], p, q, back)) {
				return &b->end[k];
			}
			if (b->tail_at[k] != TAIL_NONE &&
			    joins(&b->tail[k], p, q, back)) {
				return &b->tail[k];
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

// Adds the use of end k of round b to out, the second end the other
// way: one stretch, or two split where the end meets another round and
// a patch, with the piece past a miter before or after them.
static void use_end(struct shape *out, const struct blend *b, int k)
{
	int back = k;
	struct seg p[DEG_MAX];
	struct seg run[DEG_MAX + 1];
	size_t n = b->split[k] + 1;
	size_t nr = 0;

	for (size_t i = 0; i < n && i < DEG_MAX; i++) {
		p[i] = b->end[k];
		p[i].t0 = i ? b->ts[k][i - 1] : b->end[k].t0;
		p[i].t1 = i + 1 < n ? b->ts[k][i] : b->end[k].t1;
	}
	// The pieces in order from the first curve of touch to the second.
	if (b->tail_at[k] == TAIL_A) {
		run[nr++] = b->tail[k];
	}
	for (size_t i = 0; i < n && i < DEG_MAX; i++) {
		run[nr++] = p[b->end[k].fwd ? i : n - 1 - i];
	}
	if (b->tail_at[k] == TAIL_B) {
		run[nr++] = b->tail[k];
	}
	for (size_t i = 0; i < nr; i++) {
		use_seg(out, &run[back ? nr - 1 - i : i], back);
	}
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
		use_end(out, b, 0);
	}
	use_seg(out, &sb, 0);
	if (b->has_end[1]) {
		use_end(out, b, 1);
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

// The way in which out uses the edge of stretch s already, the other way
// from that of the new face, and that edge in *e; -1 if out uses it not
// once.
static int used_way(const struct shape *out, const struct seg *s, size_t *e)
{
	struct vec a = curve_at(&s->c, s->t0);
	struct vec z = curve_at(&s->c, s->t1);
	struct vec m = curve_at(&s->c, (s->t0 + s->t1) / 2);
	int way = -1;

	for (size_t i = 0; i < out->ne; i++) {
		const struct sedge *x = &out->e[i];
		struct vec xa = out->v[x->v0];
		struct vec xz = out->v[x->v1];

		if (((len(sub(xa, a)) < NEAR && len(sub(xz, z)) < NEAR) ||
		     (len(sub(xa, z)) < NEAR && len(sub(xz, a)) < NEAR)) &&
		    len(sub(edge_at(x, MID), m)) < NEAR) {
			*e = i;
		}
	}
	for (size_t i = 0; i < out->nu && *e != NONE; i++) {
		if (out->u[i].e == *e) {
			way = way < 0 ? out->u[i].fwd : 2;
		}
	}
	return way > 1 ? -1 : way;
}

// Adds the piece of a miter pt to out: each of its three edges the other
// way from the face beside it, in a loop.
static void patch_face(const struct stage *g, const struct patch *pt,
		       struct shape *out)
{
	char name[NAME_MAX_LEN];
	size_t e[DEG_MAX];
	int way[DEG_MAX];
	size_t first = 0;
	size_t at = 0;

	for (size_t i = 0; i < DEG_MAX; i++) {
		e[i] = NONE;
		way[i] = used_way(out, &pt->side[i], &e[i]);
		if (way[i] < 0) {
			stream_fail(g->st, "a miter patch lost its edges",
				    NULL);
		}
	}
	xsnprintf(name, "%s+%s+%s", g->sh->f[pt->f[0]].name,
		  g->sh->f[pt->f[1]].name, g->sh->f[pt->f[2]].name);
	shape_face(out, pt->s, pt->sense, name);
	first = out->nu;
	for (size_t n = 0; n < DEG_MAX; n++) {
		const struct sedge *x = &out->e[e[at]];
		size_t to = way[at] ? x->v0 : x->v1;
		size_t next = at;

		shape_use(out, e[at], !way[at]);
		for (size_t i = 0; i < DEG_MAX; i++) {
			const struct sedge *y = &out->e[e[i]];

			if (i != at && (way[i] ? y->v1 : y->v0) == to) {
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
	struct stage g = { st,	 sh,   ei, r, NULL, 0, 0, NULL,
			   NULL, NULL, 0,  0, NULL, 0, 0 };
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
	for (size_t i = 0; i < g.npatch; i++) {
		if (g.patch[i].face) {
			patch_face(&g, &g.patch[i], out);
		}
	}
	shape_check(out, st->where);
	check_faces(&g, out);
	free(c);
	free(g.b);
	free(g.blend_of);
	free(g.trim);
	free(g.ball);
	free(g.patch);
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

// Marks the edges of sh of radius r that turn the way kind, or every
// way if kind is K_SMOOTH; returns the rule of one, or NONE for none.
static size_t mark(const struct shape *sh, struct einfo *ei, double r, int kind)
{
	size_t rule = NONE;

	for (size_t e = 0; e < sh->ne; e++) {
		ei[e].in = r > 0 && ei[e].r <= r && ei[e].r >= r &&
			   (kind == K_SMOOTH || ei[e].kind == kind);
		rule = ei[e].in ? ei[e].rule : rule;
	}
	return rule;
}

// Counts the edges of sh that each rule gave a radius, and finds the
// rule of the next stage: the concave edges of the greatest radius below
// the last one, then the convex ones of that radius, so that a convex
// round runs along a concave one of the same radius as along a greater
// one.  *below is the radius of the stage, *convex whether the convex
// ones of it are left; marks the edges of the stage.  Returns the rule,
// NONE for none.
static size_t next_rule(struct rules *rs, const struct shape *sh,
			struct einfo *ei, double *below, int *convex)
{
	double r = 0;
	size_t rule = NONE;

	for (size_t e = 0; e < sh->ne; e++) {
		if (ei[e].rule >= rs->n) {
			continue;
		}
		rs->r[ei[e].rule].hits++;
		if (ei[e].r < *below && ei[e].r > r) {
			r = ei[e].r;
		}
	}
	if (*convex) {
		*convex = 0;
		rule = mark(sh, ei, *below, K_CONVEX);
		if (rule != NONE) {
			return rule;
		}
	}
	rule = mark(sh, ei, r, K_CONCAVE);
	*below = r;
	*convex = rule != NONE;
	return rule != NONE ? rule : mark(sh, ei, r, K_CONVEX);
}

// Rounds solid sh by the rules, the greatest radius first, in place;
// each round of a radius works on the solid that the greater ones left.
// If list, it prints the edges with what the rules need to know, and
// rounds nothing.
static void round_solid(struct rules *rs, struct shape *sh, int list)
{
	double below = HUGE_VAL;
	int convex = 0;

	for (;;) {
		struct einfo *ei = edge_infos(sh);
		struct shape out;
		size_t rule = NONE;

		apply_rules(rs, sh, ei);
		if (list) {
			print_edges(sh, ei);
		} else {
			rule = next_rule(rs, sh, ei, &below, &convex);
		}
		if (rule >= rs->n) {
			free(ei);
			return;
		}
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
	struct rule r = { 0, { NULL, NULL }, line, 0, 0, NONE };
	int sharp = stream_is(st, 0, "sharp");
	size_t i = sharp ? 1 : 2;
	int convex = !sharp && st->nw == i + 1 && stream_is(st, i, "convex");

	if (!convex && st->nw != i + 2) {
		stream_fail(st,
			    "round R convex, round R FACES FACES, or "
			    "sharp FACES FACES",
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

// Refuses each rule of rs that matches no edge, and says so of the rule
// that takes every edge it fits.
static void check_hits(const struct rules *rs)
{
	struct stream *st = rs->st;

	for (size_t i = 0; i < rs->n; i++) {
		const struct rule *r = &rs->r[i];

		if (!r->hits && r->fits && r->over < rs->n) {
			char msg[MSG_LEN];

			xsnprintf(msg,
				  "this rule matches no edge: the rule of line "
				  "%u takes every edge it fits",
				  st->ln[rs->r[r->over].line].no);
			(void)stream_line(st, r->line);
			stream_fail(st, msg, NULL);
		}
		if (!r->hits) {
			(void)stream_line(st, r->line);
			stream_fail(st, "this rule matches no edge", NULL);
		}
	}
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
	if (!list) {
		check_hits(&rs);
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
