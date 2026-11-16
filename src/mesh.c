// SPDX-License-Identifier: GPL-3.0-only
// mesh.c -- Triangles of exact solids, for viewing
// Copyright (c) 2026 Jakob Kastelic

// Each face is meshed in the parameters (u, v) of its surface.  A grid
// of lines u = const and v = const cuts the face into cells, so small
// that the surface over a cell stays within the chord and the angle; a
// plane needs no lines but those that keep its cells small for speed,
// and every hole gets a line across it, so that no cell has a hole.
// Every edge is cut first into pieces within the chord, and then where
// it crosses a line of either of its faces, so that the two faces share
// all its points.  The loops of a face, counterclockwise in (u, v), are
// then cut along each line in turn, and each cell is cut into triangles
// by clipping ears.

#include "mesh.h"
#include "brep.h"
#include "fold.h"
#include "geom.h"
#include "shape.h"
#include "util.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define LINE_EPS 1e-9 // The nearest a point lies to a line of the grid.
#define PLANE_CELL 50.0 // The size of a cell on a plane, in mm.
#define NUDGE 1e-4 // A move of a line off a point, as a share of a cell.
#define DIFF 1e-6 // The step of a parameter for a derivative.
#define MERGE 0.1 // Edge points this near are one, as a share of the chord.
#define ELLIPSE_MIN 1e-12 // The least half axis of an ellipse to divide by.
#define POLE_NEAR 0.9 // A frame axis this near a pole takes another axis.
#define SPLIT_MIN 0.1 // The least share of a split direction off the axis.
// The direction of the splits of full circles, and the one they take when
// it lies near their axis: any two that no part favors.
#define SPLIT_1 0.6
#define SPLIT_2 0.48
#define SPLIT_3 0.64

#define DEG (PI / 180)
#define FLAT_EAR 1e-12 // The least area of an ear, at the last, times 2.
#define HASH_MUL 0x9e3779b97f4a7c15ULL // Spreads the bits of a key.

enum {
	BISECT_STEPS = 64, // Halvings of a piece of an edge at a line.
	NO_TAG = -1, // A point that lies on no line of a face.
	HASH_MIN = 64, // The least room of the table of grid corners.
	NAME_LEN = 512, // Room for the name of a face.
	HALF_PTS = 8, // The ends and middles of the four edges of a half face.
	NODAL_SPEED = 16 // Samples of a nodal section for its top speed.
};

void mesh_init(struct mesh *m)
{
	memset(m, 0, sizeof(*m));
}

void mesh_free(struct mesh *m)
{
	free(m->v);
	free(m->t);
	mesh_init(m);
}

// A point of an edge at parameter t; for each of the two faces of the
// edge and each axis, the index of the line of that face it lies on, or
// NO_TAG.
struct tpt {
	double t;
	int tag[2][2];
};

// The points of an edge, its faces, and the mesh vertex of each point.
struct ework {
	struct tpt *p;
	size_t n, cap;
	size_t face[2], use[2]; // The faces of the edge, and its uses there.
	int nface;
	size_t *vid;
};

// The line x = c along axis a of the parameters of a face, or the step
// c between lines along a.
struct line {
	int a;
	double c;
};

// The parameters of a face: its surface, framed for meshing; whether u
// and v are angles, and the angles they lie near; the sign of the turn
// from u to v about the outward normal; and the lines of the grid.
struct fpar {
	struct surf s;
	int angle[2];
	double center[2];
	double lo[2], hi[2];
	double sign;
	double *line[2];
	size_t nline[2], capline[2];
	size_t first; // The first use of the face.
	struct uv *ref; // The angles at the middle of each use, unwrapped.
};

// A point of a loop in the cut space: (u, sign v), its vertex, and
// whether it is the apex of a cone.
struct pt {
	double c[2];
	size_t id;
	int apex;
};

struct ploop {
	struct pt *p;
	size_t n, cap;
};

struct lset {
	struct ploop *l;
	size_t n, cap;
};

// A table of the grid corners of a face, made as the cuts need them.
struct corners {
	struct pt *slot;
	unsigned char *used;
	size_t cap, n;
};

// What one face needs while it is cut.
struct face_ctx {
	struct mesh *m;
	const struct fpar *fp;
	struct corners cn;
	const char *name;
};

static void face_fail(const char *name, const char *what)
{
	die("mesh: face %s: %s", name, what);
}

// The angle step that keeps a circle of radius r within the options.
static double step_of(double r, struct mesh_opts o)
{
	double a = o.angle * DEG;

	if (r > o.chord) {
		a = fmin(a, 2 * acos(1 - (o.chord / r)));
	}
	return a;
}

static size_t pieces(double span, double step)
{
	double n = ceil(span / step);

	return n < 1 ? 1 : (size_t)n;
}

static void push_tpt(struct ework *w, double t)
{
	struct tpt p = { t, { { NO_TAG, NO_TAG }, { NO_TAG, NO_TAG } } };

	PUSH(w->p, w->n, w->cap, p);
}

// The points of edge e within the chord and the angle.
static void base_points(const struct sedge *e, struct mesh_opts o,
			struct ework *w)
{
	size_t n = 1;
	double r = e->c.a;

	if (e->c.kind == CV_ELLIPSE) {
		double lo = fmin(e->c.a, e->c.b);

		r = e->c.a * e->c.a / (lo > 0 ? lo : 1);
		r = fmax(r, e->c.b * e->c.b / fmax(e->c.a, ELLIPSE_MIN));
	}
	if (e->c.kind == CV_NODAL) {
		// Its points run at most at speed v, round radii no less than
		// a quarter of the tube.
		double v = 0;

		r = e->c.b / 4;
		for (int i = 0; i <= NODAL_SPEED; i++) {
			v = fmax(v,
				 len(curve_tan(&e->c,
					       e->t0 + ((e->t1 - e->t0) *
							(double)i /
							(double)NODAL_SPEED))));
		}
		n = pieces((e->t1 - e->t0) * v / r, step_of(r, o));
	} else if (e->c.kind != CV_LINE) {
		n = pieces(e->t1 - e->t0, step_of(r, o));
	}
	for (size_t i = 0; i <= n; i++) {
		push_tpt(w, e->t0 + ((e->t1 - e->t0) * (double)i / (double)n));
	}
}

// The parameters of p, its angles near those of ref.
static struct uv param_near(const struct fpar *fp, struct vec p, struct uv ref)
{
	struct uv q = surf_param(&fp->s, p);

	if (fp->angle[0]) {
		q.u = angle_near(q.u, ref.u);
	}
	if (fp->angle[1]) {
		q.v = angle_near(q.v, ref.v);
	}
	return q;
}

// The parameters of the point p of use i of the face.  The angles run on
// from use to use round each loop, past a full turn if they must, so that
// a face that reaches round its axis lies in one piece of the parameters
// (fuzzer finding 0033).
static struct uv param(const struct fpar *fp, size_t i, struct vec p)
{
	return param_near(fp, p, fp->ref[i - fp->first]);
}

static double coord(struct uv q, int a)
{
	return a ? q.v : q.u;
}

// Whether p is the apex of a cone, where u has no value.
static int apex(const struct fpar *fp, struct vec p)
{
	struct vec d = sub(p, fp->s.o);

	return fp->s.kind == SF_CONE &&
	       len(sub(d, mul(fp->s.n, dot(d, fp->s.n)))) < SH_EPS;
}

// The point of edge ew at t, in the world.
static struct vec ept(const struct sedge *e, double t)
{
	return curve_at(&e->c, t);
}

// Sets the frame of a sphere so that the face, about the direction d
// from its center, lies about u = 0, v = 0, far from the poles.
static void frame_sphere(struct surf *s, struct vec d)
{
	struct vec a = fabs(d.x) < POLE_NEAR ? v3(1, 0, 0) : v3(0, 1, 0);

	s->x = unit(d);
	s->n = unit(cross(s->x, cross(a, s->x)));
}

// The direction from the center of a sphere to the middle of the points
// of face f.
static struct vec mid_dir(const struct shape *sh, size_t f,
			  const struct ework *w)
{
	struct urange r = face_uses(sh, f);
	struct vec d = v3(0, 0, 0);

	for (size_t i = r.first; i < r.end; i++) {
		const struct sedge *e = &sh->e[sh->u[i].e];
		const struct ework *x = &w[sh->u[i].e];

		for (size_t k = 0; k < x->n; k++) {
			d = add(d, unit(sub(ept(e, x->p[k].t), sh->f[f].s.o)));
		}
	}
	return len(d) > 0 ? d : v3(1, 0, 0);
}

// The sign of the turn from u to v about the outward normal of face f
// at its point p.
static double turn_sign(const struct fpar *fp, const struct sface *fc,
			struct vec p)
{
	struct uv q = param(fp, fp->first, p);
	struct uv qu = { q.u + DIFF, q.v };
	struct uv qv = { q.u, q.v + DIFF };
	struct vec pu = sub(surf_at(&fp->s, qu), surf_at(&fp->s, q));
	struct vec pv = sub(surf_at(&fp->s, qv), surf_at(&fp->s, q));
	struct vec out =
		mul(surf_normal(&fp->s, surf_at(&fp->s, q)), fc->sense);

	return dot(cross(pu, pv), out) < 0 ? -1 : 1;
}

// The radii that set the steps along u and v of a curved surface.
static void radii(const struct fpar *fp, double *ru, double *rv)
{
	const struct surf *s = &fp->s;

	*ru = s->r;
	*rv = 0;
	if (s->kind == SF_CONE) {
		*ru = fmax(fabs(s->r + (s->r2 * fp->lo[1])),
			   fabs(s->r + (s->r2 * fp->hi[1])));
	} else if (s->kind == SF_SPHERE) {
		*rv = s->r;
	} else if (s->kind == SF_TORUS) {
		*ru = s->r + s->r2;
		*rv = s->r2;
	} else if (s->kind == SF_PIPE) {
		// A step of u turns the rings by no more than a circle of the
		// radius a^2 (k + rt) / k^2 does, k the least radius of
		// curvature of the ellipse, a its larger half axis.
		double a = fmax(s->r, s->r3);
		double k = pipe_bend(s);

		*ru = a * a * (fmax(a * a / fmin(s->r, s->r3), k) + s->r2) /
		      (k * k);
		*rv = s->r2;
	}
}

// Whether the value x lies within LINE_EPS of a point of face f.
static int near_point(const struct shape *sh, size_t f, const struct ework *w,
		      const struct fpar *fp, int a, double x)
{
	struct urange r = face_uses(sh, f);

	for (size_t i = r.first; i < r.end; i++) {
		const struct sedge *e = &sh->e[sh->u[i].e];
		const struct ework *k = &w[sh->u[i].e];

		for (size_t j = 0; j < k->n; j++) {
			struct vec p = ept(e, k->p[j].t);
			struct uv q = param(fp, i, p);

			if (!apex(fp, p) && fabs(coord(q, a) - x) < LINE_EPS) {
				return 1;
			}
		}
	}
	return 0;
}

static void add_line(struct fpar *fp, int a, double x)
{
	PUSH(fp->line[a], fp->nline[a], fp->capline[a], x);
}

// The lines along axis a of face f that keep each cell within step.
static void grid_lines(const struct shape *sh, size_t f, const struct ework *w,
		       struct fpar *fp, struct line g)
{
	int a = g.a;
	double step = g.c;
	double span = fp->hi[a] - fp->lo[a];
	size_t n = pieces(span, step);

	for (size_t k = 1; k < n; k++) {
		double cell = span / (double)n;
		double x = fp->lo[a] + (cell * (double)k);

		while (near_point(sh, f, w, fp, a, x)) {
			x += cell * NUDGE;
		}
		add_line(fp, a, x);
	}
}

// The range along u of loop l of face f, and its signed area in (u, v)
// times sign.
struct lrange {
	double lo, hi, area;
};

static struct lrange loop_range(const struct shape *sh, const struct sloop *l,
				const struct ework *w, const struct fpar *fp)
{
	struct lrange r = { HUGE_VAL, -HUGE_VAL, 0 };
	struct uv prev = { 0, 0 };
	int first = 1;
	struct uv q0 = { 0, 0 };

	for (size_t i = 0; i < l->n; i++) {
		const struct suse *u = &sh->u[l->first + i];
		const struct sedge *e = &sh->e[u->e];
		const struct ework *k = &w[u->e];

		for (size_t j = 0; j + 1 < k->n; j++) {
			size_t jj = u->fwd ? j : k->n - 1 - j;
			struct vec p = ept(e, k->p[jj].t);
			struct uv q = param(fp, l->first + i, p);

			if (apex(fp, p)) {
				continue;
			}
			r.lo = fmin(r.lo, q.u);
			r.hi = fmax(r.hi, q.u);
			if (first) {
				q0 = q;
				first = 0;
			} else {
				r.area += (prev.u * q.v) - (q.u * prev.v);
			}
			prev = q;
		}
	}
	r.area += (prev.u * q0.v) - (q0.u * prev.v);
	r.area *= fp->sign / 2;
	return r;
}

// A line across each hole of face f that no line of the grid crosses.
static void hole_lines(const struct shape *sh, size_t f, const struct ework *w,
		       struct fpar *fp)
{
	const struct sface *fc = &sh->f[f];

	for (size_t l = 0; l < fc->nloops; l++) {
		struct lrange r = loop_range(sh, &sh->l[fc->loop0 + l], w, fp);
		int crossed = 0;
		double x = (r.lo + r.hi) / 2;

		if (r.area > 0) {
			continue;
		}
		for (size_t k = 0; k < fp->nline[0]; k++) {
			crossed = crossed || (fp->line[0][k] > r.lo &&
					      fp->line[0][k] < r.hi);
		}
		while (!crossed && near_point(sh, f, w, fp, 0, x)) {
			x += (r.hi - r.lo) * NUDGE;
		}
		if (!crossed) {
			add_line(fp, 0, x);
		}
	}
}

// The angles about which the points of face f lie, as the way of the
// sum of their directions.
static void mean_angles(const struct shape *sh, size_t f, const struct ework *w,
			struct fpar *fp)
{
	struct urange r = face_uses(sh, f);
	double c[2] = { 0, 0 };
	double s[2] = { 0, 0 };

	for (size_t i = r.first; i < r.end; i++) {
		const struct sedge *e = &sh->e[sh->u[i].e];
		const struct ework *k = &w[sh->u[i].e];

		for (size_t j = 0; j < k->n; j++) {
			struct vec p = ept(e, k->p[j].t);
			struct uv q = surf_param(&fp->s, p);

			if (apex(fp, p)) {
				continue;
			}
			c[0] += cos(q.u);
			s[0] += sin(q.u);
			c[1] += cos(q.v);
			s[1] += sin(q.v);
		}
	}
	fp->center[0] = atan2(s[0], c[0]);
	fp->center[1] = atan2(s[1], c[1]);
}

// The frame, the range, the sign and the lines of face f.
// Sets the angles of the uses of face f: each loop starts near the center
// of the face and runs on from point to point, and each use keeps the
// angles at its middle.
static void use_refs(const struct shape *sh, size_t f, const struct ework *w,
		     struct fpar *fp)
{
	const struct sface *fc = &sh->f[f];
	struct urange r = face_uses(sh, f);
	struct uv center = { fp->center[0], fp->center[1] };

	fp->first = r.first;
	fp->ref = xmalloc((r.end - r.first + 1) * sizeof(*fp->ref));
	for (size_t l = 0; l < fc->nloops; l++) {
		const struct sloop *lp = &sh->l[fc->loop0 + l];
		struct uv cur = center;

		for (size_t i = 0; i < lp->n; i++) {
			const struct suse *u = &sh->u[lp->first + i];
			const struct sedge *e = &sh->e[u->e];
			const struct ework *k = &w[u->e];
			struct uv start = cur;

			for (size_t j = 0; j < k->n; j++) {
				size_t jj = u->fwd ? j : k->n - 1 - j;
				struct vec p = ept(e, k->p[jj].t);

				if (!apex(fp, p)) {
					cur = param_near(fp, p, cur);
				}
				if (j == 0) {
					start = cur;
				}
			}
			fp->ref[lp->first + i - r.first] = param_near(
				fp, ept(e, (e->t0 + e->t1) / 2), start);
		}
	}
}

static void face_param(const struct shape *sh, size_t f, const struct ework *w,
		       struct mesh_opts o, struct fpar *fp)
{
	const struct sface *fc = &sh->f[f];
	struct urange r = face_uses(sh, f);
	int kind = fc->s.kind;
	double ru = 0;
	double rv = 0;
	const struct sedge *e0 = &sh->e[sh->u[r.first].e];
	struct vec p0 = ept(e0, (e0->t0 + e0->t1) / 2);

	memset(fp, 0, sizeof(*fp));
	fp->s = fc->s;
	if (kind == SF_SPHERE) {
		frame_sphere(&fp->s, mid_dir(sh, f, w));
	}
	fp->angle[0] = kind != SF_PLANE;
	fp->angle[1] = kind == SF_TORUS || kind == SF_PIPE;
	mean_angles(sh, f, w, fp);
	use_refs(sh, f, w, fp);
	fp->lo[0] = fp->lo[1] = HUGE_VAL;
	fp->hi[0] = fp->hi[1] = -HUGE_VAL;
	for (size_t i = r.first; i < r.end; i++) {
		const struct sedge *e = &sh->e[sh->u[i].e];
		const struct ework *k = &w[sh->u[i].e];

		for (size_t j = 0; j < k->n; j++) {
			struct vec p = ept(e, k->p[j].t);
			struct uv q = param(fp, i, p);

			if (apex(fp, p)) {
				continue;
			}
			fp->lo[0] = fmin(fp->lo[0], q.u);
			fp->hi[0] = fmax(fp->hi[0], q.u);
			fp->lo[1] = fmin(fp->lo[1], q.v);
			fp->hi[1] = fmax(fp->hi[1], q.v);
		}
	}
	fp->center[0] = (fp->lo[0] + fp->hi[0]) / 2;
	fp->center[1] = (fp->lo[1] + fp->hi[1]) / 2;
	fp->sign = turn_sign(fp, fc, p0);
	radii(fp, &ru, &rv);
	if (kind == SF_PLANE) {
		grid_lines(sh, f, w, fp, (struct line){ 0, PLANE_CELL });
		grid_lines(sh, f, w, fp, (struct line){ 1, PLANE_CELL });
	} else {
		grid_lines(sh, f, w, fp, (struct line){ 0, step_of(ru, o) });
		if (rv > 0) {
			grid_lines(sh, f, w, fp,
				   (struct line){ 1, step_of(rv, o) });
		}
	}
	hole_lines(sh, f, w, fp);
	for (int a = 0; a < 2; a++) {
		sort_doubles(fp->line[a], fp->nline[a]);
	}
}

// An axis of the parameters of a face, the side of an edge the face lies
// on, and the use of the edge in the face.
struct axis_side {
	int a, side;
	size_t use;
};

// The parameter of edge e between t0 and t1 where coordinate g.a of face
// fp reaches x.
static double bisect(const struct sedge *e, const struct fpar *fp,
		     struct axis_side g, double x, struct span t)
{
	int a = g.a;
	double f0 = coord(param(fp, g.use, ept(e, t.lo)), a) - x;

	for (int i = 0; i < BISECT_STEPS; i++) {
		double tm = (t.lo + t.hi) / 2;
		double fm = coord(param(fp, g.use, ept(e, tm)), a) - x;

		if ((fm < 0) == (f0 < 0)) {
			t.lo = tm;
			f0 = fm;
		} else {
			t.hi = tm;
		}
	}
	return (t.lo + t.hi) / 2;
}

// The points where edge e crosses the lines of face side s of it.

// A piece of an edge: its span of the parameter of the edge, and the
// span it runs along an axis of a face.
struct piece {
	struct span t, c;
};

// Adds to w the points where piece pc of edge e crosses the lines of the
// face on axis g.a, tagged for side g.side of the edge.
static void line_points(const struct sedge *e, const struct fpar *fp,
			struct axis_side g, struct piece pc, struct ework *w)
{
	int a = g.a;
	struct span t = pc.t;
	struct span c = pc.c;

	for (size_t k = 0; k < fp->nline[a]; k++) {
		double x = fp->line[a][k];
		struct tpt p = { 0,
				 { { NO_TAG, NO_TAG }, { NO_TAG, NO_TAG } } };

		if ((c.lo - x) * (c.hi - x) < 0) {
			p.t = bisect(e, fp, g, x, t);
			p.tag[g.side][a] = (int)k;
			PUSH(w->p, w->n, w->cap, p);
		}
	}
}

static void cross_points(const struct sedge *e, const struct fpar *fp, int s,
			 struct ework *w)
{
	size_t n = w->n;

	for (size_t i = 0; i + 1 < n; i++) {
		struct span t = { w->p[i].t, w->p[i + 1].t };
		struct uv q0 = param(fp, w->use[s], ept(e, t.lo));
		struct uv q1 = param(fp, w->use[s], ept(e, t.hi));
		int top = apex(fp, ept(e, t.lo)) || apex(fp, ept(e, t.hi));

		for (int a = top; a < 2; a++) {
			struct piece pc = { t, { coord(q0, a), coord(q1, a) } };

			line_points(e, fp,
				    (struct axis_side){ a, s, w->use[s] }, pc,
				    w);
		}
	}
}

static int cmp_tpt(const void *lhs, const void *rhs)
{
	const struct tpt *x = lhs;
	const struct tpt *y = rhs;

	return (x->t > y->t) - (x->t < y->t);
}

static size_t new_vertex(struct mesh *m, struct vec p)
{
	PUSH(m->v, m->nv, m->capv, p);
	return m->nv - 1;
}

// Whether points a and b lie on two lines of one axis of one face, which
// a merge would lose one of.
static int tags_clash(const struct tpt *a, const struct tpt *b)
{
	for (int s = 0; s < 4; s++) {
		int x = a->tag[s / 2][s % 2];
		int y = b->tag[s / 2][s % 2];

		if (x != NO_TAG && y != NO_TAG && x != y) {
			return 1;
		}
	}
	return 0;
}

// Joins the points of an edge that lie within merge of one another, the
// tags of both kept; the two ends stay apart from the rest, and so do two
// points on two lines of one axis of a face (fuzzer finding 0032: lines
// nearer than merge on a small hole).
static void merge_points(const struct sedge *e, double merge, struct ework *w)
{
	size_t n = 1;

	for (size_t i = 1; i < w->n; i++) {
		struct tpt *last = &w->p[n - 1];
		struct tpt *p = &w->p[i];

		if (n == 1 || i + 1 == w->n || tags_clash(last, p) ||
		    len(sub(ept(e, p->t), ept(e, last->t))) > merge) {
			w->p[n++] = *p;
			continue;
		}
		for (int s = 0; s < 4; s++) {
			int *lt = &last->tag[s / 2][s % 2];

			*lt = *lt != NO_TAG ? *lt : p->tag[s / 2][s % 2];
		}
	}
	w->n = n;
}

// Sorts the points of an edge and gives each its vertex.
static void edge_vertices(const struct sedge *e, const size_t *sv,
			  struct mesh_opts o, struct mesh *m, struct ework *w)
{
	qsort(w->p, w->n, sizeof(*w->p), cmp_tpt);
	merge_points(e, o.chord * MERGE, w);
	w->vid = xmalloc((w->n + 1) * sizeof(size_t));
	for (size_t i = 0; i < w->n; i++) {
		if (i == 0) {
			w->vid[i] = sv[e->v0];
		} else if (i + 1 == w->n) {
			w->vid[i] = sv[e->v1];
		} else {
			w->vid[i] = new_vertex(m, ept(e, w->p[i].t));
		}
	}
}

static uint64_t bits(double x)
{
	uint64_t b = 0;

	memcpy(&b, &x, sizeof(b));
	return b;
}

static size_t slot_of(const struct corners *c, double u, double v)
{
	uint64_t h = (bits(u) * HASH_MUL) ^ bits(v);

	return h % c->cap;
}

static void corners_grow(struct corners *c);

// The vertex of the grid corner (u, v) in the cut space of the face.
static size_t corner(struct face_ctx *x, double u, double v)
{
	struct corners *c = &x->cn;
	size_t i = 0;
	struct pt p = { { u, v }, 0, 0 };
	struct uv q = { u, v * x->fp->sign };

	if (2 * (c->n + 1) > c->cap) {
		corners_grow(c);
	}
	i = slot_of(c, u, v);
	while (c->used[i]) {
		if (bits(c->slot[i].c[0]) == bits(u) &&
		    bits(c->slot[i].c[1]) == bits(v)) {
			return c->slot[i].id;
		}
		i = (i + 1) % c->cap;
	}
	p.id = new_vertex(x->m, surf_at(&x->fp->s, q));
	c->slot[i] = p;
	c->used[i] = 1;
	c->n++;
	return p.id;
}

static void corners_grow(struct corners *c)
{
	struct corners d = { NULL, NULL, c->cap ? 2 * c->cap : HASH_MIN, 0 };

	d.slot = xcalloc(d.cap, sizeof(*d.slot));
	d.used = xcalloc(d.cap, 1);
	for (size_t i = 0; i < c->cap; i++) {
		if (c->used[i]) {
			size_t j =
				slot_of(&d, c->slot[i].c[0], c->slot[i].c[1]);

			while (d.used[j]) {
				j = (j + 1) % d.cap;
			}
			d.slot[j] = c->slot[i];
			d.used[j] = 1;
			d.n++;
		}
	}
	free(c->slot);
	free(c->used);
	*c = d;
}

static void push_pt(struct ploop *l, struct pt p)
{
	PUSH(l->p, l->n, l->cap, p);
}

static void push_loop(struct lset *s, struct ploop l)
{
	PUSH(s->l, s->n, s->cap, l);
}

// Puts in place of each apex of the loop two points, at the u of the
// points before and after it: the apex is a stretch of u in the cut
// space, one point in the world.
static void split_apex(struct ploop *l)
{
	struct ploop out = { NULL, 0, 0 };

	for (size_t i = 0; i < l->n; i++) {
		struct pt p = l->p[i];

		if (p.apex && l->n > 2) {
			struct pt a = p;

			a.c[0] = l->p[(i + l->n - 1) % l->n].c[0];
			p.c[0] = l->p[(i + 1) % l->n].c[0];
			a.apex = p.apex = 0;
			push_pt(&out, a);
		}
		push_pt(&out, p);
	}
	free(l->p);
	*l = out;
}

// The loops of face f in the cut space, each point at the coordinate of
// its line where it lies on one.
static void face_loops(const struct shape *sh, size_t f, const struct ework *w,
		       const struct fpar *fp, struct lset *out)
{
	const struct sface *fc = &sh->f[f];

	for (size_t l = 0; l < fc->nloops; l++) {
		const struct sloop *lp = &sh->l[fc->loop0 + l];
		struct ploop pl = { NULL, 0, 0 };

		for (size_t i = 0; i < lp->n; i++) {
			const struct suse *u = &sh->u[lp->first + i];
			const struct sedge *e = &sh->e[u->e];
			const struct ework *k = &w[u->e];
			int s = k->face[0] == f ? 0 : 1;

			for (size_t j = 0; j + 1 < k->n; j++) {
				size_t jj = u->fwd ? j : k->n - 1 - j;
				struct vec w3 = ept(e, k->p[jj].t);
				struct uv q = param(fp, lp->first + i, w3);
				struct pt p = { { q.u, q.v },
						k->vid[jj],
						apex(fp, w3) };

				for (int a = 0; a < 2; a++) {
					int tag = k->p[jj].tag[s][a];

					if (tag != NO_TAG) {
						p.c[a] = fp->line[a][tag];
					}
				}
				p.c[1] *= fp->sign;
				push_pt(&pl, p);
			}
		}
		split_apex(&pl);
		push_loop(out, pl);
	}
}

// Which side of line g the point p lies: -1, 0 or 1.
static int side_of(const struct pt *p, struct line g)
{
	return (p->c[g.a] > g.c) - (p->c[g.a] < g.c);
}

// Puts a point on line g into each piece of a loop that crosses it.
static void split_at(struct face_ctx *x, struct ploop *l, struct line g)
{
	struct ploop out = { NULL, 0, 0 };
	int a = g.a;
	double c = g.c;

	for (size_t i = 0; i < l->n; i++) {
		const struct pt *p = &l->p[i];
		const struct pt *q = &l->p[(i + 1) % l->n];
		int sp = side_of(p, g);
		int sq = side_of(q, g);

		push_pt(&out, *p);
		if (sp * sq < 0) {
			double s = (c - p->c[a]) / (q->c[a] - p->c[a]);
			struct pt n = { { 0, 0 }, 0, 0 };

			n.c[a] = c;
			n.c[!a] = p->c[!a] + (s * (q->c[!a] - p->c[!a]));
			n.id = p->id == q->id ? p->id :
						corner(x, n.c[0], n.c[1]);
			push_pt(&out, n);
		}
	}
	free(l->p);
	*l = out;
}

// A point of a loop on the line where the loop crosses it: its loop, its
// place, its way (1 from the low side to the high, -1 back), whether the
// face lies after it along the line (1) or before it (-1), its place
// along the line, and its partner in the pairs along the line.  Where two
// pieces of the face touch at a point of the line, the crossing that
// closes one piece sorts before the one that opens the next.
struct cross {
	size_t loop, at;
	int way, opens;
	double b;
	size_t partner;
};

static int cmp_cross(const void *lhs, const void *rhs)
{
	const struct cross *x = lhs;
	const struct cross *y = rhs;

	if (bits(x->b) != bits(y->b)) {
		return (x->b > y->b) - (x->b < y->b);
	}
	return x->opens - y->opens;
}

// The side of the nearest point of l off line g, going by step from i.
static int off_side(const struct ploop *l, size_t i, struct line g, int step)
{
	for (size_t k = 1; k < l->n; k++) {
		size_t j = (i + l->n + (size_t)((long)k * step)) % l->n;
		int s = side_of(&l->p[j], g);

		if (s) {
			return s;
		}
	}
	return 0;
}

// The index in the sorted crossings of the crossing at point at of loop.
struct place {
	size_t loop, at;
};

static size_t find_cross(const struct cross *cr, size_t n, struct place p)
{
	for (size_t i = 0; i < n; i++) {
		if (cr[i].loop == p.loop && cr[i].at == p.at) {
			return i;
		}
	}
	return n;
}

// Walks side s of the cut from crossing k into out: along the loop of k
// to the next crossing, which leaves side s, then across to its partner.
static void walk(const struct lset *in, struct cross *cr, size_t n,
		 unsigned char *done, size_t k, struct ploop *out,
		 const char *name)
{
	size_t start = k;

	do {
		const struct ploop *l = &in->l[cr[k].loop];
		size_t i = cr[k].at;
		size_t next = n;

		done[k] = 1;
		push_pt(out, l->p[i]);
		for (size_t step = 1; step <= l->n; step++) {
			size_t j = (i + step) % l->n;

			next = find_cross(cr, n,
					  (struct place){ cr[k].loop, j });
			if (next < n) {
				break;
			}
			push_pt(out, l->p[j]);
		}
		if (next >= n || cr[next].way == cr[k].way) {
			face_fail(name, "a loop crosses a line of the grid "
					"the same way twice");
		}
		push_pt(out, l->p[cr[next].at]);
		k = cr[next].partner;
		if (k >= n || cr[k].way != cr[start].way) {
			face_fail(name, "the crossings of a line do not pair");
		}
	} while (k != start);
}

// The crossings.
struct crossings {
	struct cross *v;
	size_t n, cap;
};

// Adds to cs the crossings of line g by loop l of in, or puts the loop,
// whole, into lo or hi if it does not cross.
static void crossings_of(struct face_ctx *x, struct lset *in, size_t l,
			 struct line g, struct lset *side[2],
			 struct crossings *cs)
{
	struct ploop *lp = &in->l[l];
	struct ploop cp = { NULL, 0, 0 };
	size_t n0 = cs->n;

	split_at(x, lp, g);
	for (size_t i = 0; i < lp->n; i++) {
		int before = off_side(lp, i, g, -1);
		int after = off_side(lp, i, g, 1);
		struct cross k = {
			l, i, after, g.a ? -after : after, lp->p[i].c[!g.a], 0
		};

		if (!side_of(&lp->p[i], g) && before != after) {
			PUSH(cs->v, cs->n, cs->cap, k);
		}
	}
	if (cs->n > n0) {
		return;
	}
	for (size_t i = 0; i < lp->n; i++) {
		push_pt(&cp, lp->p[i]);
	}
	push_loop(side[off_side(lp, 0, g, 1) > 0], cp);
}

// Cuts the loops in along line g into those below and above.
static void cut(struct face_ctx *x, struct lset *in, struct line g,
		struct lset *lo, struct lset *hi)
{
	struct crossings cs = { NULL, 0, 0 };
	struct lset *side[2] = { lo, hi };
	struct cross *cr = NULL;
	size_t n = 0;
	unsigned char *done = NULL;

	for (size_t l = 0; l < in->n; l++) {
		crossings_of(x, in, l, g, side, &cs);
	}
	cr = cs.v;
	n = cs.n;
	if (n % 2) {
		face_fail(x->name, "a line of the grid crosses the loops an "
				   "odd number of times");
	}
	if (n) {
		qsort(cr, n, sizeof(*cr), cmp_cross);
	}
	for (size_t i = 0; i < n; i++) {
		cr[i].partner = i ^ 1U;
	}
	done = xcalloc(n + 1, 1);
	for (size_t i = 0; i < n; i++) {
		struct ploop out = { NULL, 0, 0 };

		if (done[i]) {
			continue;
		}
		walk(in, cr, n, done, i, &out, x->name);
		push_loop(cr[i].way > 0 ? hi : lo, out);
	}
	free(done);
	free(cr);
}

static void free_lset(struct lset *s)
{
	for (size_t i = 0; i < s->n; i++) {
		free(s->l[i].p);
	}
	free(s->l);
	memset(s, 0, sizeof(*s));
}

// Cuts every loop of in along each line on axis a in turn, into out.
static void cut_all(struct face_ctx *x, struct lset *in, int a,
		    struct lset *out)
{
	const struct fpar *fp = x->fp;
	struct lset rest = *in;

	memset(in, 0, sizeof(*in));
	for (size_t k = 0; k < fp->nline[a]; k++) {
		double c = fp->line[a][k] * (a ? fp->sign : 1);
		struct lset lo = { NULL, 0, 0 };
		struct lset hi = { NULL, 0, 0 };

		cut(x, &rest, (struct line){ a, c }, &lo, &hi);
		free_lset(&rest);
		if (a && fp->sign < 0) {
			rest = lo;
			lo = hi;
		} else {
			rest = hi;
		}
		for (size_t i = 0; i < lo.n; i++) {
			push_loop(out, lo.l[i]);
		}
		free(lo.l);
	}
	for (size_t i = 0; i < rest.n; i++) {
		push_loop(out, rest.l[i]);
	}
	free(rest.l);
}

static double cross2(const struct pt *a, const struct pt *b, const struct pt *c)
{
	return ((b->c[0] - a->c[0]) * (c->c[1] - a->c[1])) -
	       ((b->c[1] - a->c[1]) * (c->c[0] - a->c[0]));
}

// Whether p lies inside or on the triangle a, b, c, counterclockwise.
static int in_tri(const struct pt *p, const struct pt *a, const struct pt *b,
		  const struct pt *c)
{
	return cross2(a, b, p) >= 0 && cross2(b, c, p) >= 0 &&
	       cross2(c, a, p) >= 0;
}

static void add_tri(struct mesh *m, size_t a, size_t b, size_t c, size_t f)
{
	struct mesh_tri t = { { a, b, c }, f };

	if (a != b && b != c && c != a) {
		PUSH(m->t, m->nt, m->capt, t);
	}
}

// Whether vertex i of the ring, with its neighbors, is an ear: convex
// past min, and no other point inside.
// A ring of the points of a loop, those left linked by prev and next,
// and the least turn of an ear.
struct ring {
	const struct ploop *l;
	size_t *prev, *next;
	double min;
};

static int is_ear(const struct ring *r, size_t i)
{
	double min = r->min;
	const struct ploop *l = r->l;
	const size_t *prev = r->prev;
	const size_t *next = r->next;
	const struct pt *a = &l->p[prev[i]];
	const struct pt *b = &l->p[i];
	const struct pt *c = &l->p[next[i]];

	if (!(cross2(a, b, c) > min)) {
		return 0;
	}
	for (size_t j = next[next[i]]; j != prev[i]; j = next[j]) {
		const struct pt *p = &l->p[j];

		if (p->id != a->id && p->id != b->id && p->id != c->id &&
		    in_tri(p, a, b, c)) {
			return 0;
		}
	}
	return 1;
}

// Cuts the loop, counterclockwise and simple, into triangles of face f.
static void ears(const struct ploop *l, struct mesh *m, size_t f)
{
	size_t n = l->n;
	size_t *prev = xmalloc((n + 1) * sizeof(size_t));
	size_t *next = xmalloc((n + 1) * sizeof(size_t));
	struct ring r = { l, prev, next, 0 };
	size_t i = 0;
	size_t miss = 0;

	for (size_t k = 0; k < n; k++) {
		prev[k] = (k + n - 1) % n;
		next[k] = (k + 1) % n;
	}
	while (n > 3) {
		r.min = miss < n ? 0 : -FLAT_EAR;
		if (miss > 2 * n || is_ear(&r, i)) {
			add_tri(m, l->p[prev[i]].id, l->p[i].id,
				l->p[next[i]].id, f);
			next[prev[i]] = next[i];
			prev[next[i]] = prev[i];
			i = prev[i];
			n--;
			miss = 0;
		} else {
			i = next[i];
			miss++;
		}
	}
	add_tri(m, l->p[prev[i]].id, l->p[i].id, l->p[next[i]].id, f);
	free(prev);
	free(next);
}

static void mesh_face(const struct shape *sh, size_t f, const struct ework *w,
		      const struct fpar *fp, struct mesh *m)
{
	struct face_ctx x = { m, fp, { NULL, NULL, 0, 0 }, sh->f[f].name };
	struct lset loops = { NULL, 0, 0 };
	struct lset strips = { NULL, 0, 0 };
	struct lset cells = { NULL, 0, 0 };

	face_loops(sh, f, w, fp, &loops);
	cut_all(&x, &loops, 0, &strips);
	cut_all(&x, &strips, 1, &cells);
	for (size_t i = 0; i < cells.n; i++) {
		if (cells.l[i].n >= 3) {
			ears(&cells.l[i], m, f);
		}
	}
	free_lset(&loops);
	free_lset(&strips);
	free_lset(&cells);
	free(x.cn.slot);
	free(x.cn.used);
}

void mesh_shape(const struct shape *sh, struct mesh_opts o, struct mesh *m)
{
	struct ework *w = xcalloc(sh->ne + 1, sizeof(*w));
	struct fpar *fp = xcalloc(sh->nf + 1, sizeof(*fp));
	size_t *sv = xmalloc((sh->nv + 1) * sizeof(size_t));

	for (size_t f = 0; f < sh->nf; f++) {
		struct urange r = face_uses(sh, f);

		for (size_t i = r.first; i < r.end; i++) {
			struct ework *k = &w[sh->u[i].e];

			if (k->nface == 2) {
				die("mesh: edge %zu of %s has more than two "
				    "uses",
				    sh->u[i].e, sh->name);
			}
			k->face[k->nface] = f;
			k->use[k->nface++] = i;
		}
	}
	for (size_t e = 0; e < sh->ne; e++) {
		base_points(&sh->e[e], o, &w[e]);
	}
	for (size_t f = 0; f < sh->nf; f++) {
		face_param(sh, f, w, o, &fp[f]);
	}
	for (size_t e = 0; e < sh->ne; e++) {
		for (int s = 0; s < w[e].nface; s++) {
			cross_points(&sh->e[e], &fp[w[e].face[s]], s, &w[e]);
		}
	}
	for (size_t i = 0; i < sh->nv; i++) {
		sv[i] = new_vertex(m, sh->v[i]);
	}
	for (size_t e = 0; e < sh->ne; e++) {
		edge_vertices(&sh->e[e], sv, o, m, &w[e]);
	}
	for (size_t f = 0; f < sh->nf; f++) {
		mesh_face(sh, f, w, &fp[f], m);
	}
	for (size_t e = 0; e < sh->ne; e++) {
		free(w[e].p);
		free(w[e].vid);
	}
	for (size_t f = 0; f < sh->nf; f++) {
		free(fp[f].line[0]);
		free(fp[f].line[1]);
		free(fp[f].ref);
	}
	free(w);
	free(fp);
	free(sv);
}

// The conversion of a folded part.  A full circle becomes two half
// circles, split along the way d made square to its axis, so that two
// circles on one axis split in line; a cylinder or a cone round a hole,
// between two full circles, becomes two halves with seams along it.

// An edge of the part as edges of the shape: one, or two halves.
struct emap {
	size_t e[2];
	size_t n;
};

// A use of an edge of the shape.
struct use2 {
	size_t e;
	int fwd;
};

static struct vec split_dir(struct vec n)
{
	struct vec d = v3(SPLIT_1, SPLIT_2, SPLIT_3);
	struct vec p = sub(d, mul(n, dot(d, n)));

	if (len(p) < SPLIT_MIN) {
		d = v3(-SPLIT_2, SPLIT_3, SPLIT_1);
		p = sub(d, mul(n, dot(d, n)));
	}
	return unit(p);
}

static int full_circle(const struct bedge *e)
{
	return e->type == C_CIRCLE && e->v0 == e->v1 && e->sweep > PI;
}

static struct emap map_edge(struct shape *sh, const struct brep *b, size_t i)
{
	const struct bedge *be = &b->e[i];
	struct sedge e;
	struct emap m = { { 0, 0 }, 1 };
	struct vec a = b->v[be->v0];
	struct vec z = b->v[be->v1];

	memset(&e, 0, sizeof(e));
	if (be->type == C_LINE) {
		e.c.kind = CV_LINE;
		e.c.o = a;
		e.c.x = unit(sub(z, a));
		e.t1 = len(sub(z, a));
	} else {
		e.c.kind = CV_CIRCLE;
		e.c.o = be->c;
		e.c.n = be->n;
		e.c.a = be->r;
		e.c.x = full_circle(be) ? split_dir(be->n) :
					  unit(sub(a, be->c));
		e.t1 = full_circle(be) ? PI : be->sweep;
	}
	// The ends of an arc are those of the part, not the ends of its
	// sweep: the two lie apart by the rounding of the fold file.
	e.v0 = shape_vertex(sh, full_circle(be) ? curve_at(&e.c, 0) : a);
	e.v1 = shape_vertex(sh, full_circle(be) ? curve_at(&e.c, PI) : z);
	m.e[0] = shape_edge(sh, e);
	if (full_circle(be)) {
		e.t0 = PI;
		e.t1 = 2 * PI;
		e.v0 = e.v1;
		e.v1 = shape_vertex(sh, curve_at(&e.c, e.t1));
		m.e[1] = shape_edge(sh, e);
		m.n = 2;
	}
	return m;
}

static void put_use(struct shape *sh, struct use2 u)
{
	shape_use(sh, u.e, u.fwd);
}

// The uses of edge i of the part, forward if fwd.
static void put_edge(struct shape *sh, const struct emap *em, int fwd)
{
	for (size_t k = 0; k < em->n; k++) {
		struct use2 u = { em->e[fwd ? k : em->n - 1 - k], fwd };

		put_use(sh, u);
	}
}

static void put_loop(struct shape *sh, const struct use2 *u, size_t n, int back)
{
	size_t first = sh->nu;

	for (size_t i = 0; i < n; i++) {
		struct use2 x = u[back ? n - 1 - i : i];

		x.fwd = back ? !x.fwd : x.fwd;
		put_use(sh, x);
	}
	shape_end_loop(sh, first);
}

// The area in (u, v) of surface s inside the loop of uses u, of four
// edges, by their ends and middles.
static double half_area(const struct shape *sh, const struct use2 *u,
			struct surf s)
{
	struct uv q[HALF_PTS];
	double ref = 0;
	double a = 0;

	for (size_t i = 0; i < HALF_PTS / 2; i++) {
		const struct sedge *e = &sh->e[u[i].e];
		struct vec p = curve_at(&e->c, u[i].fwd ? e->t0 : e->t1);
		struct vec mid = curve_at(&e->c, (e->t0 + e->t1) / 2);

		q[2 * i] = surf_param(&s, p);
		q[(2 * i) + 1] = surf_param(&s, mid);
	}
	ref = q[1].u;
	for (size_t i = 0; i < HALF_PTS; i++) {
		q[i].u = angle_near(q[i].u, ref);
	}
	for (size_t i = 0; i < HALF_PTS; i++) {
		struct uv b = q[(i + 1) % HALF_PTS];

		a += (q[i].u * b.v) - (b.u * q[i].v);
	}
	return a / 2;
}

// The two halves of a round face fc of the part between two full
// circles: half k runs along half k of the first circle, the seams and
// the half of the second circle on its side.
static void wrap_face(struct shape *sh, const struct brep *b,
		      const struct bface *bf, const struct emap *em,
		      struct surf s, const char *name)
{
	const struct bedge *c[2];
	const struct emap *m[2];
	struct sedge seam[2];
	size_t se[2];

	for (int k = 0; k < 2; k++) {
		const struct buse *u = &b->u[b->l[bf->loop0 + (size_t)k].first];

		c[k] = &b->e[u->e];
		m[k] = &em[u->e];
	}
	for (int k = 0; k < 2; k++) {
		const struct sedge *a = &sh->e[m[0]->e[0]];
		const struct sedge *z = &sh->e[m[1]->e[0]];
		struct vec p = sh->v[k ? a->v1 : a->v0];
		struct vec q = sh->v[k ? z->v1 : z->v0];

		memset(&seam[k], 0, sizeof(seam[k]));
		seam[k].c.kind = CV_LINE;
		seam[k].c.o = p;
		seam[k].c.x = unit(sub(q, p));
		seam[k].t1 = len(sub(q, p));
		seam[k].v0 = k ? a->v1 : a->v0;
		seam[k].v1 = k ? z->v1 : z->v0;
		se[k] = shape_edge(sh, seam[k]);
	}
	{
		struct vec d = split_dir(c[0]->n);
		int same = dot(cross(c[0]->n, d), cross(c[1]->n, d)) > 0;
		struct use2 h0[4] = { { m[0]->e[0], 1 },
				      { se[1], 1 },
				      { m[1]->e[same ? 0 : 1], !same },
				      { se[0], 0 } };
		struct use2 h1[4] = { { m[0]->e[1], 1 },
				      { se[0], 1 },
				      { m[1]->e[same ? 1 : 0], !same },
				      { se[1], 0 } };
		int back = half_area(sh, h0, s) * bf->sense < 0;

		shape_face(sh, s, bf->sense, name);
		put_loop(sh, h0, 4, back);
		shape_face(sh, s, bf->sense, name);
		put_loop(sh, h1, 4, back);
	}
}

static int wraps(const struct brep *b, const struct bface *f)
{
	if (f->type == S_PLANE || f->nloops != 2) {
		return 0;
	}
	for (size_t l = 0; l < 2; l++) {
		const struct bloop *lp = &b->l[f->loop0 + l];

		if (lp->n != 1 || !full_circle(&b->e[b->u[lp->first].e])) {
			return 0;
		}
	}
	return 1;
}

// Adds the loops of brep face bf to the last face of sh.
static void face_loops_of(struct shape *sh, const struct brep *b,
			  const struct bface *bf, const struct emap *em)
{
	for (size_t l = 0; l < bf->nloops; l++) {
		const struct bloop *lp = &b->l[bf->loop0 + l];
		size_t first = sh->nu;

		for (size_t i = 0; i < lp->n; i++) {
			const struct buse *u = &b->u[lp->first + i];

			put_edge(sh, &em[u->e], u->fwd);
		}
		shape_end_loop(sh, first);
	}
}

void part_shape(const struct part *p, struct shape *sh)
{
	part_shape_from(p, sh, NULL);
}

// The solid of solid part p, moved by its place, into sh.
static void solid_shape(const struct part *p, struct shape *sh)
{
	shape_copy(sh, p->solid);
	free(sh->name);
	sh->name = xstrdup(p->name);
	shape_move(sh, p->place);
}

// The surface of face bf of a blank.
static struct surf bface_surf(const struct bface *bf)
{
	struct surf s;

	memset(&s, 0, sizeof(s));
	s.kind = SF_CONE;
	if (bf->type == S_PLANE) {
		s.kind = SF_PLANE;
	} else if (bf->type == S_CYLINDER) {
		s.kind = SF_CYL;
	}
	s.o = bf->o;
	s.n = bf->n;
	s.x = bf->x;
	s.r = bf->r;
	s.r2 = bf->r2;
	return s;
}

// Adds the faces of face f of blank b to sh, named name, by the edges
// em of the solid.
static void add_bface(struct shape *sh, const struct brep *b, size_t f,
		      const struct emap *em, const char *name)
{
	const struct bface *bf = &b->f[f];
	struct surf s = bface_surf(bf);

	if (wraps(b, bf)) {
		wrap_face(sh, b, bf, em, s, name);
	} else {
		shape_face(sh, s, bf->sense, name);
		face_loops_of(sh, b, bf, em);
	}
}

void part_shape_from(const struct part *p, struct shape *sh,
		     struct face_from **from)
{
	size_t capf = 0;
	size_t nf = 0;
	struct brep b;
	struct emap *em = NULL;
	char name[NAME_LEN];

	if (p->solid) {
		solid_shape(p, sh);
		return;
	}
	memset(&b, 0, sizeof(b));
	brep_build(p, &b);
	shape_init(sh, p->name);
	em = xcalloc(b.ne + 1, sizeof(*em));
	for (size_t i = 0; i < b.ne; i++) {
		em[i] = map_edge(sh, &b, i);
	}
	for (size_t f = 0; f < b.nf; f++) {
		const struct bface *bf = &b.f[f];

		xsnprintf(name, "%s.%zu", p->name, f);
		add_bface(sh, &b, f, em, name);
		while (from && nf < sh->nf) {
			struct face_from x = { bf->from, bf->idx };

			PUSH(*from, nf, capf, x);
		}
	}
	free(em);
	brep_free(&b);
}
