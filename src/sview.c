// SPDX-License-Identifier: GPL-3.0-only
// sview.c -- Project solid parts into the lines of one drawing view
// Copyright (c) 2026 Jakob Kastelic

// The parts go into one shape, each in shells of its own, and each face
// keeps the number of its part.  Each curve is cut into pieces no longer
// than STEP_MM or STEP_ANG, and the middle of each piece is seen or not
// as a ray from it toward the eye meets a face or none, by the exact
// hits of isect.h; where two pieces differ, halvings find the place
// between.  A run of a curve that is neither a line nor a circle seen
// face on goes out as the points of a line: pieces of SPLIT_ANG at the
// most, each halved until its middle lies within the chord of the view
// from the line.  A section leaves out the hits nearer the eye than its
// plane, and the pieces there.  The faces cut by the plane are the loops
// where it cuts the triangles of the mesh of the parts, within
// SLICE_CHORD, thinned to the chord of the view; the mesh is cut
// SLICE_SHIFT beyond the plane, where none of its points lies.  A ray
// down a hole, to tell whether it is blind, runs PROBE of the radius off
// the axis, clear of the point of a drill.

#include "sview.h"
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

#define STEP_MM 0.5 // The longest piece of a curve, in mm,
#define STEP_ANG (2 * DEG) // and of an arc, in radians.
#define SPLIT_ANG (PI / 8) // A piece of a poly record before halving.
#define LIFT 1e-4 // How far off its point a ray of sight starts, in mm.
#define FACE_ON 1e-9 // A sine below this is no angle at all.
#define ZERO 5e-7 // Numbers closer to 0 than this print as 0.
#define SLICE_CHORD 0.005 // The chord of the mesh of a section, in mm.
#define SLICE_ANGLE 2.0 // and its angle, in degrees.
#define JOIN 1e-6 // Ends of cuts of a section this near are one, in mm.
#define SLICE_SHIFT 1e-5 // The mesh is cut this far beyond the plane, in mm.
#define FACING 0.5 // The least cosine of a textured face to the eye.
#define THICK 1.0 // The wall that a view of a solid gives smdim, in mm.
#define SINK_FIT 1e-3 // A countersink meets its hole this near, in mm.
#define PROBE 0.5 // The share of its radius off the axis of a ray down a hole.
#define FLAT_PAIR 1e-9 // Below this, 1 - cos^2 of two normals is none.

enum {
	HALVINGS = 30, // To find where a curve goes behind material.
	SPLITS = 16, // The most halvings of a piece of a poly record.
	GEN_PTS = 64, // Points of an outline of a cylinder or a cone.
	FACE_PTS = 16, // Points of each edge of a face, for its extent.
	SEEN_GRID = 8, // The grid over a face where the eye may see it.
	LETTERS = 26, // The letters that may mark a texture.
	NAME_ROOM = 256, // Room for the name of a feature.
	RECORD_WORDS = 64 // The most words of a vary record of a face.
};

// What a piece of a curve is: seen, behind material, or gone with the
// part of the solid that a section takes away.
enum { SEEN, BEHIND, GONE };

struct moves;

// The parts, all in one shape, with the part of each face, and the
// parameters they lean on, with the moves of the faces at the two ends
// of the range of each, mv[2 k] and mv[2 k + 1] for parameter k.
struct sv {
	const struct part *ps;
	size_t np;
	struct shape sh;
	size_t *fpart;
	struct shape_cache c;
	struct sview v;
	double reach;
	const char **param;
	size_t nparam, capparam;
	struct moves *mv;
};

// Points of the paper.
struct pts {
	struct vec2 *p;
	size_t n, cap;
};

static double pr(double x)
{
	return fabs(x) < ZERO ? 0.0 : x;
}

static double px(const struct sv *s, struct vec q)
{
	return dot(q, s->v.x);
}

static double py(const struct sv *s, struct vec q)
{
	return dot(q, s->v.y);
}

static struct vec2 paper(const struct sv *s, struct vec q)
{
	struct vec2 p = { px(s, q), py(s, q) };

	return p;
}

// Whether point q is seen, behind material, or gone.  In a section,
// the faces cut hide what lies behind them: the ray toward the eye
// meets the plane inside a part, which has an odd number of its faces
// beyond the plane.  The faces of a part come one after another.
static int seen(const struct sv *s, struct vec q)
{
	struct ray_seg rs = { { add(q, mul(s->v.z, LIFT)), s->v.z }, s->reach };
	int odd = 0;

	if (s->v.section && dot(q, s->v.z) > s->v.cut) {
		return GONE;
	}
	for (size_t f = 0; f < s->sh.nf; f++) {
		double t[ROOTS_MAX];
		int unsafe = 0;
		size_t n = 0;

		if (f > 0 && s->fpart[f] != s->fpart[f - 1]) {
			if (odd) {
				return BEHIND;
			}
		}
		n = ray_face_hits(&s->c, f, rs, t, &unsafe);
		for (size_t k = 0; k < n; k++) {
			struct vec h = add(rs.l.o, mul(s->v.z, t[k]));

			if (!s->v.section || dot(h, s->v.z) <= s->v.cut) {
				return BEHIND;
			}
			odd = !odd;
		}
	}
	return odd ? BEHIND : SEEN;
}

// The place between a and b on curve c where what it is changes, from
// that at a.
static double change(const struct sv *s, const struct curve *c, double a,
		     double b)
{
	int at = seen(s, curve_at(c, a));

	for (int i = 0; i < HALVINGS; i++) {
		double m = (a + b) / 2;

		if (seen(s, curve_at(c, m)) == at) {
			a = m;
		} else {
			b = m;
		}
	}
	return (a + b) / 2;
}

// The distance from q to the segment from a to b.
static double off_seg(struct vec2 q, struct vec2 a, struct vec2 b)
{
	double dx = b.x - a.x;
	double dy = b.y - a.y;
	double l2 = (dx * dx) + (dy * dy);
	double t = 0;

	if (l2 > 0) {
		t = (((q.x - a.x) * dx) + ((q.y - a.y) * dy)) / l2;
	}
	t = fmin(fmax(t, 0), 1);
	return hypot(q.x - a.x - (t * dx), q.y - a.y - (t * dy));
}

// Adds to l the points of curve c on the paper after its point at t.lo
// up to that at t.hi: the end, or the points of the two halves if the
// middle lies further than the chord from the line, n more times at the
// most.
static void split_piece(const struct sv *s, const struct curve *c,
			struct span t, int n, struct pts *l)
{
	double m = (t.lo + t.hi) / 2;
	struct vec2 a = paper(s, curve_at(c, t.lo));
	struct vec2 b = paper(s, curve_at(c, t.hi));

	if (n > 0 && off_seg(paper(s, curve_at(c, m)), a, b) > s->v.chord) {
		split_piece(s, c, (struct span){ t.lo, m }, n - 1, l);
		split_piece(s, c, (struct span){ m, t.hi }, n - 1, l);
		return;
	}
	PUSH(l->p, l->n, l->cap, b);
}

// A parameter of the parts at one end of its range: the surface of each
// face that it moves, NULL for one it leaves, and whether the place of
// each face there is unknown, as of a folded part that takes another
// form.
struct moves {
	struct surf **s;
	unsigned char *unknown;
};

// The surface s moved by the rigid motion m.
static struct surf surf_moved(struct surf s, struct rigid m)
{
	s.o = rig_pt(m, s.o);
	s.n = rig_dir(m, s.n);
	s.x = rig_dir(m, s.x);
	return s;
}

// The index of parameter name among those of s, added if new.
static size_t param_index(struct sv *s, const char *name)
{
	for (size_t i = 0; i < s->nparam; i++) {
		if (strcmp(s->param[i], name) == 0) {
			return i;
		}
	}
	PUSH(s->param, s->nparam, s->capparam, name);
	return s->nparam - 1;
}

// Notes in mv the faces of solid part i, its faces from f0 on, that the
// record face FACE S0 S1 at text moves: the faces of that name on S0
// take S1.
static void solid_moves(struct sv *s, size_t i, size_t f0, const char *text,
			struct moves *mv)
{
	const struct part *p = &s->ps[i];
	char *copy = xstrdup(text);
	char *w[RECORD_WORDS + 1];
	size_t n = split_words(copy, w, RECORD_WORDS);
	size_t at = 2;
	struct surf a;
	struct surf b;

	if (n < 3 || n > RECORD_WORDS || strcmp(w[0], "face") != 0) {
		die("%s: part %s: bad vary record %s", s->v.where, p->name,
		    text);
	}
	a = surf_moved(surf_read(w, n, &at, p->name), p->place);
	b = surf_moved(surf_read(w, n, &at, p->name), p->place);
	for (size_t f = f0; f < s->sh.nf && s->fpart[f] == i; f++) {
		if (strcmp(s->sh.f[f].name, w[1]) == 0 &&
		    surfs_same(&s->sh.f[f].s, &a)) {
			if (!mv->s[f]) {
				mv->s[f] = xmalloc(sizeof(*mv->s[f]));
			}
			*mv->s[f] = b;
		}
	}
	free(copy);
}

// Notes in mv the faces of folded part i, its faces from f0 on, that
// the end hi of its parameter t moves: those of its blank there, or all
// unknown if the blank there has other faces.
static void folded_moves(struct sv *s, size_t i, size_t f0, size_t t, int hi,
			 struct moves *mv)
{
	struct part q;
	struct shape one;
	size_t n = 0;

	vary_part(&s->ps[i], t, hi, &q);
	part_shape(&q, &one);
	while (f0 + n < s->sh.nf && s->fpart[f0 + n] == i) {
		n++;
	}
	for (size_t k = 0; k < n; k++) {
		const struct surf *u = &one.f[k].s;

		if (one.nf != n) {
			mv->unknown[f0 + k] = 1;
		} else if (!surfs_same(&s->sh.f[f0 + k].s, u)) {
			mv->s[f0 + k] = xmalloc(sizeof(*mv->s[f0 + k]));
			*mv->s[f0 + k] = *u;
		}
	}
	shape_free(&one);
	free_variant(&q);
}

// Makes the moves of the faces of the parts at each end of the range of
// each parameter that they lean on, by name: of a solid, its vary
// records of faces, and of a folded part, its blank there.
static void make_moves(struct sv *s)
{
	size_t f0 = 0;

	for (size_t i = 0; i < s->np; i++) {
		for (size_t t = 0; t < s->ps[i].rg.ntol; t++) {
			(void)param_index(s, s->ps[i].rg.tol[t].name);
		}
	}
	s->mv = xcalloc((2 * s->nparam) + 1, sizeof(*s->mv));
	for (size_t k = 0; k < 2 * s->nparam; k++) {
		s->mv[k].s = xcalloc(s->sh.nf + 1, sizeof(*s->mv[k].s));
		s->mv[k].unknown = xcalloc(s->sh.nf + 1, 1);
	}
	for (size_t i = 0; i < s->np; i++) {
		const struct ranges *g = &s->ps[i].rg;

		while (f0 < s->sh.nf && s->fpart[f0] < i) {
			f0++;
		}
		for (size_t t = 0; t < g->ntol; t++) {
			size_t k = 2 * param_index(s, g->tol[t].name);

			for (int hi = 0; !s->ps[i].solid && hi < 2; hi++) {
				folded_moves(s, i, f0, t, hi,
					     &s->mv[k + (size_t)hi]);
			}
		}
		for (size_t v = 0; s->ps[i].solid && v < g->nvary; v++) {
			const struct part_vary *pv = &g->vary[v];
			size_t k = 2 * param_index(s, g->tol[pv->tol].name);

			solid_moves(s, i, f0, pv->rec,
				    &s->mv[k + (size_t)pv->hi]);
		}
	}
}

static void free_moves(struct sv *s)
{
	for (size_t k = 0; k < 2 * s->nparam; k++) {
		for (size_t f = 0; f < s->sh.nf; f++) {
			free(s->mv[k].s[f]);
		}
		free(s->mv[k].s);
		free(s->mv[k].unknown);
	}
	free(s->mv);
	free(s->param);
}

// What the end k of a parameter moves the point q of face f: along the
// normal of the face, to the moved surface.
static struct vec face_move(const struct sv *s, size_t k, size_t f,
			    struct vec q)
{
	const struct surf *m = s->mv[k].s[f];

	if (!m) {
		return v3(0, 0, 0);
	}
	return mul(surf_normal(m, q), -surf_dist(m, q));
}

// Where a point of the view lies, for its moves: on an edge between
// the faces on[0] and on[1], the second NONE for a line on one face; or
// with cut on the line where the plane of a section cuts face on[0]; or
// with axis at the center of the hole of cylinder on[0].
struct lies {
	size_t on[2];
	int cut, axis;
};

// What the end k of a parameter moves the point q of the view, as at
// says: the least move that keeps it on its faces, as they move, in the
// plane of a section with cut, or the move of the axis of a hole;
// returns 0 if it is unknown, as at the tip of a cone.
static int point_move(const struct sv *s, size_t k, struct lies at,
		      struct vec q, struct vec *d)
{
	const struct surf *m = NULL;
	struct vec da;
	struct vec na;

	if (at.on[0] == NONE || s->mv[k].unknown[at.on[0]] ||
	    (at.on[1] != NONE && s->mv[k].unknown[at.on[1]])) {
		return 0;
	}
	m = s->mv[k].s[at.on[0]];
	da = face_move(s, k, at.on[0], q);
	na = surf_normal(&s->sh.f[at.on[0]].s, q);
	if (at.axis) {
		struct vec o = m ? sub(m->o, s->sh.f[at.on[0]].s.o) : da;
		struct vec n = s->sh.f[at.on[0]].s.n;

		*d = sub(o, mul(n, dot(o, n)));
	} else if (at.cut) {
		struct vec np = sub(na, mul(s->v.z, dot(na, s->v.z)));
		double l2 = dot(np, np);

		*d = l2 > FLAT_PAIR ? mul(np, dot(na, da) / l2) : v3(0, 0, 0);
		if (!(l2 > FLAT_PAIR) && m) {
			return 0;
		}
	} else if (at.on[1] == NONE) {
		*d = da;
	} else {
		struct vec db = face_move(s, k, at.on[1], q);
		struct vec nb = surf_normal(&s->sh.f[at.on[1]].s, q);
		double c = dot(na, nb);
		double det = 1 - (c * c);
		double a = dot(na, da);
		double b = dot(nb, db);

		*d = det < FLAT_PAIR ? mul(add(da, db), 0.5) :
				       add(mul(na, (a - (c * b)) / det),
					   mul(nb, (b - (c * a)) / det));
	}
	return isfinite(d->x) && isfinite(d->y) && isfinite(d->z);
}

// Prints a record range NAME N X Y ... for each parameter that moves the
// n points q of a record of the view, which lie as at says: N, and the
// moves on the paper at the low end and then at the high one; or range
// NAME - where a move is unknown.
static void put_ranges(const struct sv *s, const struct vec *q, size_t n,
		       struct lies at)
{
	for (size_t g = 0; g < s->nparam; g++) {
		struct vec d[2][2];
		int ok = 1;
		double most = 0;

		for (size_t k = 0; k < 2 * n; k++) {
			ok = ok && point_move(s, (2 * g) + (k / n), at,
					      q[k % n], &d[k / n][k % n]);
			most = ok ? fmax(most, len(d[k / n][k % n])) : most;
		}
		if (!ok) {
			printf("range %s -\n", s->param[g]);
			continue;
		}
		if (!(most > ZERO)) {
			continue;
		}
		printf("range %s %zu", s->param[g], n);
		for (size_t k = 0; k < 2 * n; k++) {
			printf(" %.6f %.6f", pr(px(s, d[k / n][k % n])),
			       pr(py(s, d[k / n][k % n])));
		}
		printf("\n");
	}
}

// Prints the n points p as a poly record of the kind and visibility.
static void put_points(const struct vec2 *p, size_t n, char kind, char vis)
{
	printf("poly %c %c %zu", kind, vis, n);
	for (size_t i = 0; i < n; i++) {
		printf(" %.6f %.6f", pr(p[i].x), pr(p[i].y));
	}
	printf("\n");
}

// Prints the run of curve c from t.lo to t.hi, seen or not by vis, of
// the kind: a line, an arc seen face on, or else a poly record.
static void put_run(const struct sv *s, const struct curve *c, struct span t,
		    char kind, char vis, struct lies at)
{
	struct vec p0 = curve_at(c, t.lo);
	struct vec p1 = curve_at(c, t.hi);
	struct pts l = { NULL, 0, 0 };
	double pieces = 0;
	size_t k = 0;

	if (c->kind == CV_LINE) {
		const struct vec q[2] = { p0, p1 };

		printf("line %c %c %.6f %.6f %.6f %.6f\n", kind, vis,
		       pr(px(s, p0)), pr(py(s, p0)), pr(px(s, p1)),
		       pr(py(s, p1)));
		put_ranges(s, q, 2, at);
		return;
	}
	if (c->kind == CV_CIRCLE && len(cross(c->n, s->v.z)) < FACE_ON) {
		int ccw = dot(c->n, s->v.z) > 0;
		struct vec from = sub(ccw ? p0 : p1, c->o);
		double a0 = atan2(py(s, from), px(s, from)) / DEG;

		printf("arc %c %c %.6f %.6f %.6f %.6f %.6f\n", kind, vis,
		       pr(px(s, c->o)), pr(py(s, c->o)), c->a, pr(a0),
		       pr(a0 + ((t.hi - t.lo) / DEG)));
		return;
	}
	pieces = ceil((t.hi - t.lo) / SPLIT_ANG);
	k = pieces < 1 ? 1 : (size_t)pieces;
	PUSH(l.p, l.n, l.cap, paper(s, p0));
	for (size_t i = 0; i < k; i++) {
		struct span piece = {
			t.lo + ((t.hi - t.lo) * (double)i / (double)k),
			t.lo + ((t.hi - t.lo) * (double)(i + 1) / (double)k)
		};

		split_piece(s, c, piece, SPLITS, &l);
	}
	put_points(l.p, l.n, kind, vis);
	free(l.p);
}

// Prints curve c from t0 to t1, of the kind: its runs of what it is,
// those behind material only if the view shows hidden lines.
static void put_curve(const struct sv *s, const struct curve *c, struct span t,
		      char kind, struct lies at)
{
	double l = len(sub(curve_at(c, t.hi), curve_at(c, t.lo)));
	double n = ceil(l / STEP_MM);
	size_t steps = 0;
	double run = t.lo;
	int what = 0;

	if (c->kind != CV_LINE) {
		double turn = (t.hi - t.lo) / STEP_ANG;

		l = 0;
		for (int i = 0; i < GEN_PTS; i++) {
			l += len(sub(curve_at(c, t.lo + ((t.hi - t.lo) *
							 (i + 1) / GEN_PTS)),
				     curve_at(c, t.lo + ((t.hi - t.lo) * i /
							 GEN_PTS))));
		}
		n = fmax(ceil(turn), ceil(l / STEP_MM));
	}
	steps = n < 1 ? 1 : (size_t)n;
	n = (double)steps;
	what = seen(s, curve_at(c, t.lo + ((t.hi - t.lo) / (2 * n))));
	for (size_t i = 1; i <= steps; i++) {
		double a = t.lo + ((t.hi - t.lo) * ((double)i - 0.5) / n);
		double b = t.lo + ((t.hi - t.lo) * ((double)i + 0.5) / n);
		int next = i < steps ? seen(s, curve_at(c, b)) : -1;

		if (next == what) {
			continue;
		}
		{
			double end = next < 0 ? t.hi : change(s, c, a, b);
			struct span r = { run, end };

			if (what == SEEN || (what == BEHIND && s->v.hidden)) {
				put_run(s, c, r, kind, what == SEEN ? 'v' : 'h',
					at);
			}
			run = end;
			what = next;
		}
	}
}

// Whether face f is the wall of a hole: a cylinder that bounds the solid
// from outside.
static int hole_wall(const struct shape *sh, size_t f)
{
	return sh->f[f].s.kind == SF_CYL && sh->f[f].sense < 0;
}

// Prints the edges of the solid: smooth ones of the kind s, those of a
// hole o, and the others e.
static void put_edges(const struct sv *s)
{
	const struct shape *sh = &s->sh;
	size_t(*ef)[2] = xcalloc(sh->ne + 1, sizeof(*ef));

	for (size_t f = 0; f < sh->nf; f++) {
		struct urange r = face_uses(sh, f);

		for (size_t i = r.first; i < r.end; i++) {
			ef[sh->u[i].e][sh->u[i].fwd ? 0 : 1] = f;
		}
	}
	for (size_t e = 0; e < sh->ne; e++) {
		const struct sedge *ed = &sh->e[e];
		struct vec m = edge_at(ed, MID);
		struct vec na = surf_normal(&sh->f[ef[e][0]].s, m);
		struct vec nb = surf_normal(&sh->f[ef[e][1]].s, m);
		char kind = 'e';
		struct lies at = { { ef[e][0], ef[e][1] }, 0, 0 };

		if (len(cross(na, nb)) < FACE_ON) {
			kind = 's';
		} else if (hole_wall(sh, ef[e][0]) || hole_wall(sh, ef[e][1])) {
			kind = 'o';
		}
		put_curve(s, &ed->c, (struct span){ ed->t0, ed->t1 }, kind, at);
	}
	free(ef);
}

// The span of the parameter v of the points of the edges of face f.
static struct span face_v(const struct shape *sh, size_t f)
{
	struct urange r = face_uses(sh, f);
	struct span v = { HUGE_VAL, -HUGE_VAL };

	for (size_t i = r.first; i < r.end; i++) {
		for (int k = 0; k <= FACE_PTS; k++) {
			struct uv q = surf_param(&sh->f[f].s,
						 edge_at(&sh->e[sh->u[i].e],
							 k / (double)FACE_PTS));

			v.lo = fmin(v.lo, q.v);
			v.hi = fmax(v.hi, q.v);
		}
	}
	return v;
}

// Prints the outlines of cylinder or cone face f whose axis lies across
// the view: the lines of it along which its normal lies across the eye,
// where they run on the face.
static void put_outline(const struct sv *s, size_t f)
{
	const struct surf *u = &s->sh.f[f].s;
	struct vec y = frame_y(u->n, u->x);
	double a = dot(u->x, s->v.z);
	double b = dot(y, s->v.z);
	double rho = hypot(a, b);
	double k = u->kind == SF_CONE ? u->r2 * dot(u->n, s->v.z) : 0;
	struct span v = face_v(&s->sh, f);

	if (rho < FACE_ON || fabs(k) > rho) {
		return;
	}
	for (int side = -1; side <= 1; side += 2) {
		double ang = atan2(b, a) + (side * acos(k / rho));
		struct vec e = add(mul(u->x, cos(ang)), mul(y, sin(ang)));
		struct vec d =
			add(u->n, mul(e, u->kind == SF_CONE ? u->r2 : 0));
		struct curve g;
		int in = 0;
		double from = 0;

		memset(&g, 0, sizeof(g));
		g.kind = CV_LINE;
		g.o = surf_at(u, (struct uv){ ang, 0 });
		g.x = unit(d);
		for (int i = 0; i <= GEN_PTS; i++) {
			double t =
				(v.lo + ((v.hi - v.lo) * i / GEN_PTS)) * len(d);
			int now = i < GEN_PTS &&
				  face_has_c(&s->c, f, curve_at(&g, t));

			if (now && !in) {
				from = t;
			} else if (!now && in && t > from) {
				struct lies at = { { f, NONE }, 0, 0 };

				put_curve(s, &g, (struct span){ from, t },
					  hole_wall(&s->sh, f) ? 'o' : 'e', at);
			}
			in = now;
		}
	}
}

// The name of the feature of face f: its name up to the first dot.
static void feature(const struct shape *sh, size_t f, char *out)
{
	size_t n = strcspn(sh->f[f].name, ".");

	n = n < NAME_ROOM - 1 ? n : NAME_ROOM - 1;
	memcpy(out, sh->f[f].name, n);
	out[n] = 0;
}

// The countersink of a hole: its diameter, 0 for none, its included
// angle in degrees, whether it opens at the end of the hole nearer the
// eye, and the middle of its wide end.
struct sink {
	double d, ang;
	int near;
	struct vec wide;
};

// The countersink of the hole of cylinder face f, whose ends on its axis
// are a, nearer the eye, and b: a cone of its part on the same axis
// whose narrow end meets one of them, at the radius of the hole.
static struct sink find_sink(const struct sv *s, size_t f, struct vec a,
			     struct vec b)
{
	const struct shape *sh = &s->sh;
	const struct surf *u = &sh->f[f].s;
	struct sink k = { 0, 0, 0, { 0, 0, 0 } };

	for (size_t g = 0; g < sh->nf; g++) {
		const struct surf *w = &sh->f[g].s;
		struct span v = { 0, 0 };
		struct span r = { 0, 0 };
		struct vec thin;

		if (w->kind != SF_CONE || s->fpart[g] != s->fpart[f] ||
		    len(cross(w->n, u->n)) > FACE_ON ||
		    len(cross(sub(w->o, u->o), u->n)) > SINK_FIT) {
			continue;
		}
		v = face_v(sh, g);
		r.lo = w->r + (w->r2 * v.lo);
		r.hi = w->r + (w->r2 * v.hi);
		thin = add(w->o, mul(w->n, r.lo < r.hi ? v.lo : v.hi));
		if (fabs(fmin(r.lo, r.hi) - u->r) > SINK_FIT ||
		    (len(sub(thin, a)) > SINK_FIT &&
		     len(sub(thin, b)) > SINK_FIT)) {
			continue;
		}
		k.d = 2 * fmax(r.lo, r.hi);
		k.ang = 2 * atan(fabs(w->r2)) / DEG;
		k.near = len(sub(thin, a)) <= SINK_FIT;
		k.wide = add(w->o, mul(w->n, r.lo < r.hi ? v.hi : v.lo));
	}
	return k;
}

// Whether the hole of cylinder face f is blind: a ray down it, from its
// end a toward its other end b, meets a face of its part.
static int is_blind(const struct sv *s, size_t f, struct vec a, struct vec b)
{
	const struct surf *u = &s->sh.f[f].s;
	struct vec d = unit(sub(b, a));
	struct ray_seg rs = {
		{ add(add(a, mul(u->x, PROBE * u->r)), mul(d, LIFT)), d },
		s->reach
	};

	for (size_t g = 0; g < s->sh.nf; g++) {
		double t[ROOTS_MAX];
		int unsafe = 0;

		if (g != f && s->fpart[g] == s->fpart[f] &&
		    ray_face_hits(&s->c, g, rs, t, &unsafe) > 0) {
			return 1;
		}
	}
	return 0;
}

// Prints the thread of the hole of face f, if its part gives it one: to
// the depth the part gives, or else to the depth deep of a blind hole,
// or through.
static void put_thread(const struct sv *s, size_t f, double deep)
{
	const struct part *p = &s->ps[s->fpart[f]];
	const struct face_thread *t =
		thread_of(p->threads, p->nthreads, s->sh.f[f].name);

	if (!t) {
		return;
	}
	if (t->deep > 0 || deep > 0) {
		printf(" thread %.6f %s", t->deep > 0 ? t->deep : deep,
		       t->spec);
	} else {
		printf(" thread thru %s", t->spec);
	}
}

// Prints a hole record for each hole seen along its axis, at its end
// nearest the eye, if nothing covers that: with the depth of a blind
// one from its mouth, its countersink and its thread.
static void put_holes(const struct sv *s)
{
	const struct shape *sh = &s->sh;
	struct vec *at = xmalloc((sh->nf + 1) * sizeof(*at));
	size_t n = 0;

	for (size_t f = 0; f < sh->nf; f++) {
		const struct surf *u = &sh->f[f].s;
		int up = dot(u->n, s->v.z) > 0;
		struct span v = face_v(sh, f);
		struct vec a;
		struct vec b;
		struct sink k;
		char name[NAME_ROOM];
		int dup = 0;
		double deep = 0;

		if (!hole_wall(sh, f) || len(cross(u->n, s->v.z)) > FACE_ON) {
			continue;
		}
		a = add(u->o, mul(u->n, up ? v.hi : v.lo));
		b = add(u->o, mul(u->n, up ? v.lo : v.hi));
		for (size_t i = 0; i < n; i++) {
			dup = dup || len(sub(at[i], a)) < JOIN;
		}
		if (dup || seen(s, a) != SEEN) {
			continue;
		}
		at[n++] = a;
		k = find_sink(s, f, a, b);
		if (is_blind(s, f, a, b)) {
			deep = dot(k.d > 0 && k.near ? k.wide : a, s->v.z) -
			       dot(b, s->v.z);
		}
		feature(sh, f, name);
		printf("hole %.6f %.6f %.6f", pr(px(s, a)), pr(py(s, a)),
		       2 * u->r);
		if (deep > 0) {
			printf(" deep %.6f", deep);
		}
		if (k.d > 0) {
			printf(" csink %.6f %.3f %s", k.d, k.ang,
			       k.near ? "near" : "far");
		}
		put_thread(s, f, deep);
		printf(" by %s\n", name);
		{
			struct lies on = { { f, NONE }, 0, 1 };

			put_ranges(s, &a, 1, on);
		}
	}
	free(at);
}

// Whether face f lies all on the side of the plane of a section that it
// takes away.
static int gone(const struct sv *s, size_t f)
{
	struct urange r = face_uses(&s->sh, f);

	for (size_t i = r.first; s->v.section && i < r.end; i++) {
		for (int k = 0; k <= FACE_PTS; k++) {
			struct vec x = edge_at(&s->sh.e[s->sh.u[i].e],
					       k / (double)FACE_PTS);

			if (dot(x, s->v.z) < s->v.cut) {
				return 0;
			}
		}
	}
	return s->v.section;
}

// A texture of the faces of the parts: its material, the letter that
// marks it, or '-' for the one of all the faces it does not mark, and
// the area of its faces.
struct tex {
	const struct material *m;
	char mark;
	double area;
};

// The texture of face f, or NULL.
static const struct material *face_texture(const struct sv *s, size_t f)
{
	const struct part *p = &s->ps[s->fpart[f]];
	const char *name = finish_of(p->fins, p->nfins, s->sh.f[f].name);
	const struct material *m = name ? part_material(p, name) : NULL;

	return m && m->callout[0] ? m : NULL;
}

// The textures of the faces of the parts, into *out, in the order of
// their first faces; returns their number.  Where each face has one and
// there are two or more, the texture of the view's unmarked, or else of
// the most area, marks none: it is that of all the faces not marked.
// Each other gets its own mark, or else the first letter from T on,
// round the alphabet, that no texture has: no two share one, whatever
// the view, as the parts give them.
static size_t textures(const struct sv *s, struct tex **out)
{
	struct tex *t = NULL;
	size_t n = 0;
	size_t cap = 0;
	size_t most = 0;
	int all = 1;

	for (size_t f = 0; f < s->sh.nf; f++) {
		const struct material *m = face_texture(s, f);
		size_t k = 0;

		all = all && m;
		while (m && k < n && strcmp(t[k].m->name, m->name) != 0) {
			k++;
		}
		if (m && k == n) {
			struct tex x = { m, 0, 0 };

			PUSH(t, n, cap, x);
		}
		if (m) {
			t[k].area += face_mass(&s->sh, f).area;
		}
	}
	for (size_t k = 1; k < n; k++) {
		most = t[k].area > t[most].area ? k : most;
	}
	for (size_t k = 0; s->v.unmarked && k < n; k++) {
		if (strcmp(t[k].m->name, s->v.unmarked) == 0) {
			most = k;
			all = all ? 2 : 0;
		}
	}
	if (s->v.unmarked && all != 2) {
		die("%s: view %s: the texture %s is none of every face that the "
		    "others leave",
		    s->v.where, s->v.name, s->v.unmarked);
	}
	if (all && (n > 1 || s->v.unmarked)) {
		t[most].mark = '-';
	}
	for (int own = 1; own >= 0; own--) {
		for (size_t k = 0; k < n; k++) {
			char c = own ? t[k].m->mark : 'T';
			int taken = 1;

			for (int tries = 0;
			     !t[k].mark && c && taken && tries < LETTERS;
			     tries++) {
				taken = 0;
				for (size_t i = 0; i < n; i++) {
					taken = taken || t[i].mark == c;
				}
				if (!taken) {
					t[k].mark = c;
				} else if (own) {
					c = 0;
				} else {
					c = (char)(c == 'Z' ? 'A' : c + 1);
				}
			}
		}
	}
	*out = t;
	return n;
}

// Whether a point inside face f is seen: of a grid over the span of its
// parameters, the first that lies on the face and that the eye sees.
static int face_seen(const struct sv *s, size_t f)
{
	const struct shape *sh = &s->sh;
	struct urange r = face_uses(sh, f);
	struct span u = { HUGE_VAL, -HUGE_VAL };
	struct span v = { HUGE_VAL, -HUGE_VAL };

	for (size_t i = r.first; i < r.end; i++) {
		for (int k = 0; k < FACE_PTS; k++) {
			struct uv q = surf_param(&sh->f[f].s,
						 edge_at(&sh->e[sh->u[i].e],
							 k / (double)FACE_PTS));

			u.lo = fmin(u.lo, q.u);
			u.hi = fmax(u.hi, q.u);
			v.lo = fmin(v.lo, q.v);
			v.hi = fmax(v.hi, q.v);
		}
	}
	for (int i = 1; i < SEEN_GRID; i++) {
		for (int j = 1; j < SEEN_GRID; j++) {
			struct uv q = { u.lo + ((u.hi - u.lo) * i / SEEN_GRID),
					v.lo + ((v.hi - v.lo) * j /
						SEEN_GRID) };
			struct vec x = surf_at(&sh->f[f].s, q);

			if (face_has_c(&s->c, f, x) && seen(s, x) == SEEN) {
				return 1;
			}
		}
	}
	return 0;
}

// Prints the textures of the faces: first the one of all the faces not
// marked, if any, with the mark -; then each face of a marked texture
// that looks at the eye and that it sees in part, but those a section
// takes away, as a coat of it, after the texture of the first of them:
// the polygon of the points of its edges, and face.
static void put_textures(const struct sv *s)
{
	const struct shape *sh = &s->sh;
	struct tex *t = NULL;
	size_t nt = textures(s, &t);
	char *done = xcalloc(nt + 1, 1);

	for (size_t k = 0; k < nt; k++) {
		if (t[k].mark == '-') {
			printf("texture %s - %s\n", t[k].m->name,
			       t[k].m->callout);
		}
	}
	for (size_t f = 0; f < sh->nf; f++) {
		const struct material *m = face_texture(s, f);
		struct urange r = face_uses(sh, f);
		struct vec q = edge_at(&sh->e[sh->u[r.first].e], MID);
		size_t k = 0;

		while (m && k < nt && strcmp(t[k].m->name, m->name) != 0) {
			k++;
		}
		if (!m || t[k].mark == '-' || gone(s, f) ||
		    dot(mul(surf_normal(&sh->f[f].s, q), sh->f[f].sense),
			s->v.z) < FACING ||
		    !face_seen(s, f)) {
			continue;
		}
		if (!done[k]) {
			printf("texture %s %c %s\n", m->name, t[k].mark,
			       m->callout);
			done[k] = 1;
		}
		printf("coat %s near %zu", m->name,
		       (r.end - r.first) * FACE_PTS);
		for (size_t i = r.first; i < r.end; i++) {
			const struct sedge *e = &sh->e[sh->u[i].e];

			for (int j = 0; j < FACE_PTS; j++) {
				double w = sh->u[i].fwd ? j : FACE_PTS - j;
				struct vec x = edge_at(e, w / FACE_PTS);

				printf(" %.6f %.6f", pr(px(s, x)),
				       pr(py(s, x)));
			}
		}
		printf(" face\n");
	}
	free(done);
	free(t);
}

// A cut of a triangle by the plane of a section, on the paper, the
// part of the triangle, and its face if that is a plane, else NONE: the
// cuts of one plane face lie on one line; and its face.
struct cut {
	struct vec2 a, b;
	size_t part, flat, face;
	int used;
};

// A loop of cuts: its n points, and the plane face of the cut from each
// to the next, or NONE, and its face.
struct loop {
	struct vec2 *p;
	size_t *flat, *face;
	size_t n, cap, capf, capface;
};

// Thins the closed loop l, and the faces of its cuts, to the points that
// keep each point left out within the chord of the line through the
// points kept.  The ends of
// each straight run stay, the cuts of one plane face, so that a straight
// edge of a section is one line, as it is in the part, where an
// ordinate or a lin may stand.  Between two points kept, or from the
// first point and the one furthest from it if no face is a plane, each
// stretch gains the point furthest from its chord, if further than the
// chord.  Returns the number kept, first in l->p, in order, the face of
// the cut from each in l->flat: that of the side to the next point
// kept, where its cuts are of one plane face; and in l->face the face of
// all the cuts of that side, or NONE.
static size_t thin_loop(struct loop *l, double chord)
{
	struct vec2 *p = l->p;
	size_t n = l->n;
	char *keep = xcalloc(n + 1, 1);
	size_t(*st)[2] = xmalloc((2 * n + 2) * sizeof(*st));
	size_t *v = xmalloc((n + 1) * sizeof(*v));
	size_t nv = 0;
	size_t ns = 0;
	size_t far = 0;
	size_t m = 0;

	for (size_t k = 0; k < n; k++) {
		size_t before = l->flat[(k + n - 1) % n];

		if (before != l->flat[k] &&
		    (before != NONE || l->flat[k] != NONE)) {
			keep[k] = 1;
			v[nv++] = k;
		}
	}
	if (!nv) {
		for (size_t k = 1; k < n; k++) {
			if (hypot(p[k].x - p[0].x, p[k].y - p[0].y) >
			    hypot(p[far].x - p[0].x, p[far].y - p[0].y)) {
				far = k;
			}
		}
		keep[0] = keep[far] = 1;
		v[nv++] = 0;
		v[nv++] = far;
	}
	for (size_t i = 0; i < nv; i++) {
		st[ns][0] = v[i];
		st[ns++][1] = i + 1 < nv ? v[i + 1] : v[0] + n;
	}
	while (ns) {
		size_t i = st[--ns][0];
		size_t j = st[ns][1];
		size_t at = i;
		double most = chord;

		for (size_t k = i + 1; k < j; k++) {
			double d = off_seg(p[k % n], p[i % n], p[j % n]);

			if (d > most) {
				most = d;
				at = k;
			}
		}
		if (at > i) {
			keep[at % n] = 1;
			st[ns][0] = i;
			st[ns++][1] = at;
			st[ns][0] = at;
			st[ns++][1] = j;
		}
	}
	for (size_t k = 0; k < n; k++) {
		size_t j = (k + 1) % n;

		while (keep[k] && !keep[j] && l->face[k] != NONE) {
			l->face[k] = l->face[j] == l->face[k] ? l->face[k] :
								NONE;
			j = (j + 1) % n;
		}
	}
	for (size_t k = 0; k < n; k++) {
		if (keep[k]) {
			l->flat[m] = l->flat[k];
			l->face[m] = l->face[k];
			p[m++] = p[k];
		}
	}
	free(keep);
	free(st);
	free(v);
	return m;
}

// The loop of the cuts c of one part from cut i on, into l, the first
// point not again at the end; the last side closes it.
static void chain(struct cut *c, size_t n, size_t i, struct loop *l)
{
	struct vec2 at = c[i].b;
	size_t nf = 0;
	size_t nf2 = 0;
	size_t last = NONE;

	c[i].used = 1;
	PUSH(l->p, l->n, l->cap, c[i].a);
	PUSH(l->flat, nf, l->capf, c[i].flat);
	PUSH(l->face, nf2, l->capface, c[i].face);
	PUSH(l->p, l->n, l->cap, at);
	for (int more = 1; more;) {
		more = 0;
		for (size_t j = 0; j < n && !more; j++) {
			int fwd = hypot(c[j].a.x - at.x, c[j].a.y - at.y) <
				  JOIN;
			int back = hypot(c[j].b.x - at.x, c[j].b.y - at.y) <
				   JOIN;

			if (!c[j].used && c[j].part == c[i].part &&
			    (fwd || back)) {
				c[j].used = 1;
				at = fwd ? c[j].b : c[j].a;
				PUSH(l->flat, nf, l->capf, c[j].flat);
				PUSH(l->face, nf2, l->capface, c[j].face);
				PUSH(l->p, l->n, l->cap, at);
				more = 1;
			}
		}
	}
	if (l->n > 2 && hypot(at.x - l->p[0].x, at.y - l->p[0].y) < JOIN) {
		l->n--;
	} else {
		PUSH(l->flat, nf, l->capf, last);
		PUSH(l->face, nf2, l->capface, last);
	}
}

// The cuts of the triangles of the mesh of the parts by the plane of the
// section, on the paper, into *out; returns their number.
static size_t section_cuts(const struct sv *s, struct cut **out)
{
	struct mesh m;
	struct mesh_opts o = { SLICE_CHORD, SLICE_ANGLE };
	struct cut *c = NULL;
	size_t n = 0;
	size_t cap = 0;

	mesh_init(&m);
	mesh_shape(&s->sh, o, &m);
	for (size_t i = 0; i < m.nt; i++) {
		struct vec2 end[2];
		int k = 0;

		for (int j = 0; j < 3 && k < 2; j++) {
			struct vec a = m.v[m.t[i].v[j]];
			struct vec b = m.v[m.t[i].v[(j + 1) % 3]];
			double da = dot(a, s->v.z) - s->v.cut + SLICE_SHIFT;
			double db = dot(b, s->v.z) - s->v.cut + SLICE_SHIFT;

			if ((da < 0) != (db < 0)) {
				end[k++] =
					paper(s, add(a, mul(sub(b, a),
							    da / (da - db))));
			}
		}
		if (k == 2) {
			size_t f = m.t[i].face;
			struct cut x = {
				end[0],
				end[1],
				s->fpart[f],
				s->sh.f[f].s.kind == SF_PLANE ? f : NONE,
				f,
				0
			};

			PUSH(c, n, cap, x);
		}
	}
	mesh_free(&m);
	*out = c;
	return n;
}

// The point of the plane of the section at p on the paper.
static struct vec in_cut(const struct sv *s, struct vec2 p)
{
	return add(add(mul(s->v.x, p.x), mul(s->v.y, p.y)),
		   mul(s->v.z, s->v.cut));
}

// Prints the faces that the plane of the section cuts: for each part
// cut, a fill region of the loops of the cuts of the triangles of the
// mesh, thinned to the chord, that names the part; and their sides as
// lines of the kind e.  A loop of a cut shorter than JOIN, which the
// loop it lies on passes by, is none.
static void put_section(const struct sv *s)
{
	struct cut *c = NULL;
	size_t n = section_cuts(s, &c);
	struct pts all = { NULL, 0, 0 }; // The loops, one after another,
	size_t *ends = NULL; // and where each ends in all,
	size_t nends = 0;
	size_t capends = 0;
	size_t *fs = NULL; // and the face of the side from each point.
	size_t nfs = 0;
	size_t capfs = 0;

	for (size_t part = 0; part < s->np; part++) {
		int open = 0;

		for (size_t i = 0; i < n; i++) {
			struct loop l = { NULL, NULL, NULL, 0, 0, 0, 0 };
			size_t k = 0;

			if (c[i].used || c[i].part != part) {
				continue;
			}
			chain(c, n, i, &l);
			k = thin_loop(&l, s->v.chord);
			if (k < 3) {
				free(l.flat);
				free(l.face);
				free(l.p);
				continue;
			}
			if (!open) {
				printf("fill cut %zu\n", part);
				open = 1;
			}
			for (size_t j = 0; j < k; j++) {
				printf("%c %.6f %.6f\n", j ? 'l' : 'm',
				       pr(l.p[j].x), pr(l.p[j].y));
				PUSH(all.p, all.n, all.cap, l.p[j]);
				PUSH(fs, nfs, capfs, l.face[j]);
			}
			printf("z\n");
			PUSH(ends, nends, capends, all.n);
			free(l.flat);
			free(l.face);
			free(l.p);
		}
	}
	for (size_t i = 0, from = 0; i < nends; from = ends[i++]) {
		for (size_t j = from; j < ends[i]; j++) {
			struct vec2 a = all.p[j];
			struct vec2 b = all.p[j + 1 < ends[i] ? j + 1 : from];

			struct lies at = { { fs[j], NONE }, 1, 0 };
			const struct vec q[2] = { in_cut(s, a), in_cut(s, b) };

			if (hypot(b.x - a.x, b.y - a.y) > 0) {
				printf("line e v %.6f %.6f %.6f %.6f\n",
				       pr(a.x), pr(a.y), pr(b.x), pr(b.y));
				put_ranges(s, q, 2, at);
			}
		}
	}
	free(all.p);
	free(fs);
	free(ends);
	free(c);
}

// Prints the box of the solid on the paper.
static void put_box(const struct sv *s)
{
	struct box b = box_empty();

	for (size_t e = 0; e < s->sh.ne; e++) {
		for (int k = 0; k <= GEN_PTS; k++) {
			struct vec q =
				edge_at(&s->sh.e[e], k / (double)GEN_PTS);

			box_add(&b, v3(px(s, q), py(s, q), 0));
		}
	}
	printf("box %.6f %.6f %.6f %.6f\n", pr(b.lo.x), pr(b.lo.y), pr(b.hi.x),
	       pr(b.hi.y));
}

// Adds the vertices, edges, loops and faces of from to those of to, as
// shells of their own.
static void append_shape(struct shape *to, const struct shape *from)
{
	size_t v0 = to->nv;
	size_t e0 = to->ne;
	size_t u0 = to->nu;
	size_t l0 = to->nl;

	for (size_t i = 0; i < from->nv; i++) {
		PUSH(to->v, to->nv, to->capv, from->v[i]);
	}
	for (size_t i = 0; i < from->ne; i++) {
		struct sedge e = from->e[i];

		e.v0 += v0;
		e.v1 += v0;
		PUSH(to->e, to->ne, to->cape, e);
	}
	for (size_t i = 0; i < from->nu; i++) {
		struct suse u = from->u[i];

		u.e += e0;
		PUSH(to->u, to->nu, to->capu, u);
	}
	for (size_t i = 0; i < from->nl; i++) {
		struct sloop l = from->l[i];

		l.first += u0;
		PUSH(to->l, to->nl, to->capl, l);
	}
	for (size_t i = 0; i < from->nf; i++) {
		struct sface f = from->f[i];

		f.loop0 += l0;
		f.name = xstrdup(f.name);
		PUSH(to->f, to->nf, to->capf, f);
	}
}

void sview_print(const struct part *ps, size_t n, struct sview v)
{
	struct sv s;
	struct box b = box_empty();
	size_t nfp = 0;
	size_t capfp = 0;

	memset(&s, 0, sizeof(s));
	s.ps = ps;
	s.np = n;
	s.v = v;
	shape_init(&s.sh, ps[0].name);
	for (size_t i = 0; i < n; i++) {
		struct shape one;

		part_shape(&ps[i], &one);
		append_shape(&s.sh, &one);
		shape_free(&one);
		while (nfp < s.sh.nf) {
			PUSH(s.fpart, nfp, capfp, i);
		}
	}
	shape_cache_init(&s.c, &s.sh);
	make_moves(&s);
	for (size_t f = 0; f < s.sh.nf; f++) {
		box_add(&b, s.c.fbox[f].lo);
		box_add(&b, s.c.fbox[f].hi);
	}
	s.reach = 2 * len(sub(b.hi, b.lo)) + 1;
	printf("view %s %s %.9g\nthick %g\n", v.name, ps[0].name, v.scale,
	       THICK);
	put_box(&s);
	if (v.section) {
		put_section(&s);
	}
	put_edges(&s);
	for (size_t f = 0; f < s.sh.nf; f++) {
		if (s.sh.f[f].s.kind == SF_CYL || s.sh.f[f].s.kind == SF_CONE) {
			put_outline(&s, f);
		}
	}
	put_holes(&s);
	put_textures(&s);
	free_moves(&s);
	shape_cache_free(&s.c);
	shape_free(&s.sh);
	free(s.fpart);
}
