// SPDX-License-Identifier: GPL-3.0-only
// isect.c -- Where curves, surfaces and faces of exact solids meet
// Copyright (c) 2026 Jakob Kastelic

// A curve meets a plane where a sum of cosines and sines, or a
// polynomial, vanishes: those roots come in closed form.  A curve meets
// a curved surface where the signed distance to the surface vanishes
// along it: samples bracket each root, or show a touch where the
// distance comes near zero, and bisection or a golden-section search
// pins it down to the last bits.

#include "isect.h"
#include "geom.h"
#include "shape.h"
#include "util.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define TINY 1e-12 // Cosines and lengths below this are 0.
#define GOLD 0.6180339887498949 // The golden section.
#define NEAR_EDGE (10 * SH_EPS) // Ray hits nearer an edge are unsafe.
#define NUDGE 1e-9 // A shift of a ray off a vertex, in mm.
#define TOUCH 1e-9 // A curve this near a surface touches it, in mm.
#define AT_VERTEX 1e-10 // A ray this near a vertex meets it, in mm.
#define GRAZE 1e-9 // A sine of a crossing below this is a touch.

enum {
	SAMPLES = 128, // Samples of a curve against a surface.
	BISECT = 80, // Halvings of a bracket.
	RAYS = 6, // Directions of rays for an inside test.
	EDGE_PTS = 64, // Points of an edge for its box, or its distance.
	NUDGES = 4 // Tries of a ray across a face.
};

// Adds root t to the n roots in r if it lies in [t0, t1], or a whole
// turn away for an angle.
static void keep_root(const struct cpiece *p, double t, double *r, size_t *n)
{
	if (curve_turns(&p->c)) {
		t = turn_from(t, p->t0 - TINY);
	}
	if (t >= p->t0 - TINY && t <= p->t1 + TINY && *n < ROOTS_MAX) {
		r[(*n)++] = fmin(p->t1, fmax(p->t0, t));
	}
}

// The roots of q2 t^2 + q1 t + q0 = 0, into r; returns their count.
static size_t quadratic(double q2, double q1, double q0, double *r)
{
	double disc = (q1 * q1) - (4 * q2 * q0);

	if (fabs(q2) < TINY) {
		if (fabs(q1) < TINY) {
			return 0;
		}
		r[0] = -q0 / q1;
		return 1;
	}
	if (disc < 0) {
		if (disc > -TINY * ((q1 * q1) + fabs(4 * q2 * q0))) {
			disc = 0;
		} else {
			return 0;
		}
	}
	disc = sqrt(disc);
	r[0] = (-q1 - disc) / (2 * q2);
	r[1] = (-q1 + disc) / (2 * q2);
	return disc > 0 ? 2 : 1;
}

size_t curve_plane(const struct cpiece *p, struct plane3 pl, double *t)
{
	const struct curve *c = &p->c;
	struct vec y = frame_y(c->n, c->x);
	double k0 = dot(pl.n, c->o) - pl.d;
	double ka = dot(pl.n, c->x) * c->a;
	double kb = dot(pl.n, y) * (c->kind == CV_CIRCLE ? c->a : c->b);
	double rr = hypot(ka, kb);
	double phi = atan2(kb, ka);
	double cs = 0;
	size_t n = 0;

	if (c->kind == CV_LINE) {
		ka = dot(pl.n, c->x);
		if (fabs(ka) > TINY) {
			keep_root(p, -k0 / ka, t, &n);
		}
		return n;
	}
	if (!(rr > TINY) || fabs(k0) > rr * (1 + TINY)) {
		return 0;
	}
	cs = acos(fmax(-1, fmin(1, -k0 / rr)));
	if (rr - fabs(k0) < TOUCH) {
		cs = k0 < 0 ? 0 : PI;
	}
	keep_root(p, phi + cs, t, &n);
	if (cs > TINY && cs < PI) {
		keep_root(p, phi - cs, t, &n);
	}
	return n;
}

static double dist_at(const struct cpiece *p, const struct surf *s, double t)
{
	return surf_dist(s, curve_at(&p->c, t));
}

// The root of the distance in the bracket [a, b], where it changes sign.
static double bisect(const struct cpiece *p, const struct surf *s, double a,
		     double b)
{
	double fa = dist_at(p, s, a);

	for (int i = 0; i < BISECT; i++) {
		double m = (a + b) / 2;
		double fm = dist_at(p, s, m);

		if ((fm < 0) == (fa < 0)) {
			a = m;
			fa = fm;
		} else {
			b = m;
		}
	}
	return (a + b) / 2;
}

// The place of least |distance| in [a, b], by golden sections.
static double least(const struct cpiece *p, const struct surf *s, double a,
		    double b)
{
	double x1 = b - (GOLD * (b - a));
	double x2 = a + (GOLD * (b - a));
	double f1 = fabs(dist_at(p, s, x1));
	double f2 = fabs(dist_at(p, s, x2));

	for (int i = 0; i < BISECT; i++) {
		if (f1 < f2) {
			b = x2;
			x2 = x1;
			f2 = f1;
			x1 = b - (GOLD * (b - a));
			f1 = fabs(dist_at(p, s, x1));
		} else {
			a = x1;
			x1 = x2;
			f1 = f2;
			x2 = a + (GOLD * (b - a));
			f2 = fabs(dist_at(p, s, x2));
		}
	}
	return (a + b) / 2;
}

static void add_root(double *t, size_t *n, double v)
{
	for (size_t i = 0; i < *n; i++) {
		if (fabs(t[i] - v) < SH_EPS) {
			return;
		}
	}
	if (*n < ROOTS_MAX) {
		t[(*n)++] = v;
	}
}

// The parameters t in [0, tmax] where o + t d meets surface s, into t;
// returns their count.  *tangent gets 1 if the line touches s there.
static size_t line_surf(struct line3 ray, double tmax, const struct surf *s,
			double *t, int *tangent);

// The roots of the distance from the piece p to surface s, by samples:
// a change of sign, a sample on s, or a least distance that reaches s.
static size_t sample_roots(const struct cpiece *p, const struct surf *s,
			   double *t)
{
	double f[SAMPLES + 1];
	double h = (p->t1 - p->t0) / (double)SAMPLES;
	size_t n = 0;

	for (int i = 0; i <= SAMPLES; i++) {
		f[i] = dist_at(p, s, p->t0 + (h * i));
	}
	for (int i = 0; i <= SAMPLES; i++) {
		double ti = p->t0 + (h * i);

		if (fabs(f[i]) < SH_EPS / 2) {
			add_root(t, &n, ti);
		} else if (i < SAMPLES && (f[i] < 0) != (f[i + 1] < 0) &&
			   fabs(f[i + 1]) >= SH_EPS / 2) {
			add_root(t, &n, bisect(p, s, ti, ti + h));
		} else if (i > 0 && i < SAMPLES &&
			   fabs(f[i]) <= fabs(f[i - 1]) &&
			   fabs(f[i]) <= fabs(f[i + 1]) &&
			   (f[i - 1] < 0) == (f[i + 1] < 0)) {
			double m = least(p, s, ti - h, ti + h);

			if (fabs(dist_at(p, s, m)) < SH_EPS) {
				add_root(t, &n, m);
			}
		}
	}
	return n;
}

size_t curve_surf(const struct cpiece *p, const struct surf *s, double *t)
{
	if (s->kind == SF_PLANE) {
		struct plane3 pl = { s->n, dot(s->n, s->o) };

		return curve_on_surf(p, s) ? 0 : curve_plane(p, pl, t);
	}
	if (curve_on_surf(p, s)) {
		return 0;
	}
	if (p->c.kind == CV_LINE && s->kind <= SF_CONE) {
		struct line3 ray = { curve_at(&p->c, p->t0), p->c.x };
		int tangent = 0;
		size_t n = line_surf(ray, p->t1 - p->t0, s, t, &tangent);

		for (size_t i = 0; i < n; i++) {
			t[i] += p->t0;
		}
		return n;
	}
	return sample_roots(p, s, t);
}

int curve_on_surf(const struct cpiece *p, const struct surf *s)
{
	enum { PROBES = 7 };

	for (int i = 0; i < PROBES; i++) {
		double t = p->t0 + ((p->t1 - p->t0) * i / (PROBES - 1));

		if (fabs(dist_at(p, s, t)) > SH_EPS) {
			return 0;
		}
	}
	return 1;
}

// A quadric along a line: q2 t^2 + q1 t + q0, the square of a distance
// less that of a radius r.
struct quadric {
	double q2, q1, q0, r;
};

// The hits of a line with a quadric, its roots in [0, tmax].  A line
// within TOUCH of the quadric touches it at one point.
static size_t line_quadric(struct quadric q, double tmax, double *t,
			   int *tangent)
{
	double rt[2];
	size_t m = 0;
	size_t n = 0;
	double q2 = q.q2;
	double q1 = q.q1;

	if (fabs(q2) > TINY &&
	    fabs(q.q0 - (q1 * q1 / (4 * q2))) < 2 * fmax(q.r, 1) * TOUCH) {
		m = 1;
		rt[0] = -q1 / (2 * q2);
	} else {
		m = quadratic(q2, q1, q.q0, rt);
	}

	*tangent = m == 1 && fabs(q2) > TINY;
	for (size_t i = 0; i < m; i++) {
		if (rt[i] >= 0 && rt[i] <= tmax) {
			t[n++] = rt[i];
		}
	}
	return n;
}

static size_t line_surf(struct line3 ray, double tmax, const struct surf *s,
			double *t, int *tangent)
{
	struct vec q = sub(ray.o, s->o);
	struct vec d = ray.d;
	double qn = dot(q, s->n);
	double dn = dot(d, s->n);
	struct vec qp = sub(q, mul(s->n, qn));
	struct vec dp = sub(d, mul(s->n, dn));

	*tangent = 0;
	if (s->kind > SF_CONE) {
		die("internal error: a line against a sphere or a torus");
	}
	if (s->kind == SF_PLANE) {
		if (fabs(dn) < TINY) {
			return 0;
		}
		t[0] = -qn / dn;
		return t[0] >= 0 && t[0] <= tmax;
	}
	if (s->kind == SF_CYL) {
		struct quadric k = { dot(dp, dp), 2 * dot(qp, dp),
				     dot(qp, qp) - (s->r * s->r), s->r };

		return line_quadric(k, tmax, t, tangent);
	}
	{
		double r0 = s->r + (s->r2 * qn);
		double r1 = s->r2 * dn;
		double tt[2];
		struct quadric k = { dot(dp, dp) - (r1 * r1),
				     2 * (dot(qp, dp) - (r0 * r1)),
				     dot(qp, qp) - (r0 * r0), fabs(r0) };
		size_t m = line_quadric(k, tmax, tt, tangent);
		size_t n = 0;

		for (size_t i = 0; i < m; i++) {
			if (r0 + (r1 * tt[i]) >= 0) {
				t[n++] = tt[i];
			}
		}
		return n;
	}
}

// The line where the planes a and b meet.
static int plane_plane(const struct surf *a, const struct surf *b,
		       struct curve *out, size_t *n)
{
	struct vec d = cross(a->n, b->n);
	double da = dot(a->n, a->o);
	double db = dot(b->n, b->o);
	double l = len(d);

	if (l < TINY) {
		return fabs(dot(a->n, sub(b->o, a->o))) < SH_EPS ? SS_SAME :
								   SS_NONE;
	}
	memset(out, 0, sizeof(*out));
	out->kind = CV_LINE;
	out->x = mul(d, 1 / l);
	out->o = mul(add(mul(cross(b->n, d), da), mul(cross(d, a->n), db)),
		     1 / (l * l));
	*n = 1;
	return SS_CURVES;
}

// The lines of cylinder c along plane p: where the plane cuts the circle
// across the axis.
static int plane_along(const struct surf *p, const struct surf *c,
		       struct curve *out, size_t *n)
{
	double s = dot(p->n, sub(c->o, p->o));
	struct vec w = cross(p->n, c->n);
	double r = c->r;
	double h = 0;

	if (fabs(s) > r + SH_EPS) {
		return SS_NONE;
	}
	h = r - fabs(s) < TOUCH ? 0 : sqrt(fmax(0, (r * r) - (s * s)));
	*n = 0;
	for (int k = -1; k <= 1; k += 2) {
		memset(&out[*n], 0, sizeof(*out));
		out[*n].kind = CV_LINE;
		out[*n].x = c->n;
		out[*n].o = add(sub(c->o, mul(p->n, s)), mul(w, k * h));
		(*n)++;
		if (h < SH_EPS) {
			break;
		}
	}
	return SS_CURVES;
}

// A circle about the axis ax, of radius r, into out.
static void circle_about(struct curve *out, struct line3 ax, double r)
{
	memset(out, 0, sizeof(*out));
	out->kind = CV_CIRCLE;
	out->o = ax.o;
	out->n = ax.d;
	out->x = unit(
		cross(ax.d, fabs(ax.d.x) < MID ? v3(1, 0, 0) : v3(0, 1, 0)));
	out->a = r;
}

// The circle where plane p cuts cylinder or cone c across its axis.
static int plane_round(const struct surf *p, const struct surf *c,
		       struct curve *out, size_t *n)
{
	double h = dot(p->n, sub(p->o, c->o)) / dot(p->n, c->n);
	struct line3 ax = { add(c->o, mul(c->n, h)), p->n };
	double r = c->kind == SF_CONE ? c->r + (c->r2 * h) : c->r;

	*n = 1;
	if (r < SH_EPS) {
		return SS_NONE;
	}
	circle_about(out, ax, r);
	return SS_CURVES;
}

// The ellipse where plane p cuts cylinder c at a slant.
static int plane_slant(const struct surf *p, const struct surf *c,
		       struct curve *out, size_t *n)
{
	double cs = dot(p->n, c->n);
	double h = dot(p->n, sub(p->o, c->o)) / cs;
	struct vec minor = unit(cross(p->n, c->n));

	memset(out, 0, sizeof(*out));
	*n = 1;
	out->kind = CV_ELLIPSE;
	out->o = add(c->o, mul(c->n, h));
	out->n = p->n;
	out->x = cross(minor, p->n);
	out->a = c->r / fabs(cs);
	out->b = c->r;
	return SS_CURVES;
}

// Plane p and a cylinder or a cone c: a circle across the axis, the
// lines of a cylinder along the plane, or the ellipse of a cylinder at
// a slant.  The conics of a cone at a slant have no user.
static int plane_curved(const struct surf *p, const struct surf *c,
			struct curve *out, size_t *n)
{
	double cs = dot(p->n, c->n);

	if (fabs(fabs(cs) - 1) < TINY) {
		return plane_round(p, c, out, n);
	}
	if (c->kind != SF_CYL) {
		return SS_NOT;
	}
	if (fabs(cs) < TINY) {
		return plane_along(p, c, out, n);
	}
	return plane_slant(p, c, out, n);
}

// Whether the axes of a and b are one line.
static int coaxial(const struct surf *a, const struct surf *b)
{
	struct vec d = sub(b->o, a->o);

	return len(cross(a->n, b->n)) < TINY &&
	       len(sub(d, mul(a->n, dot(d, a->n)))) < SH_EPS;
}

// The radius of a cylinder or a cone at height h along the axis of a.
static double radius_at(const struct surf *s, const struct surf *a, double h)
{
	double hs = (h - dot(sub(s->o, a->o), a->n)) * dot(s->n, a->n);

	return s->kind == SF_CONE ? s->r + (s->r2 * hs) : s->r;
}

// Two coaxial cylinders or cones: circles where their radii agree.
static int round_round(const struct surf *a, const struct surf *b,
		       struct curve *out, size_t *n)
{
	double r0 = radius_at(a, a, 0) - radius_at(b, a, 0);
	double r1 = radius_at(a, a, 1) - radius_at(b, a, 1);
	double slope = r1 - r0;
	double h = 0;

	if (fabs(slope) < TINY) {
		return fabs(r0) < SH_EPS ? SS_SAME : SS_NONE;
	}
	h = -r0 / slope;
	if (radius_at(a, a, h) < SH_EPS) {
		return SS_NONE;
	}
	memset(out, 0, sizeof(*out));
	out->kind = CV_CIRCLE;
	out->o = add(a->o, mul(a->n, h));
	out->n = a->n;
	out->x = a->x;
	out->a = radius_at(a, a, h);
	*n = 1;
	return SS_CURVES;
}

// Plane p and torus t, for a plane through the axis: the two circles of
// the tube.
static int plane_torus(const struct surf *p, const struct surf *t,
		       struct curve *out, size_t *n)
{
	struct vec e = unit(cross(t->n, p->n));

	if (fabs(dot(p->n, t->n)) >= TINY ||
	    fabs(dot(p->n, sub(t->o, p->o))) >= SH_EPS) {
		return SS_NOT;
	}
	for (int k = 0; k < 2; k++) {
		struct line3 ax = { add(t->o, mul(e, k ? -t->r : t->r)), p->n };

		circle_about(&out[k], ax, t->r2);
	}
	*n = 2;
	return SS_CURVES;
}

// Two cylinders of one radius whose axes cross: two ellipses, in the
// planes that halve the angles of the axes.
static int cyl_cross(const struct surf *a, const struct surf *b,
		     struct curve *out, size_t *n)
{
	struct vec w = cross(a->n, b->n);
	struct vec d = sub(b->o, a->o);
	double l2 = dot(w, w);
	double ta = dot(cross(d, b->n), w) / l2;
	struct vec x = add(a->o, mul(a->n, ta));

	if (fabs(a->r - b->r) > SH_EPS || fabs(dot(d, w)) > SH_EPS * sqrt(l2)) {
		return SS_NOT;
	}
	w = unit(w);
	for (int k = 0; k < 2; k++) {
		struct vec major = unit(add(a->n, mul(b->n, k ? -1 : 1)));
		double sn = len(cross(major, a->n));

		memset(&out[k], 0, sizeof(*out));
		out[k].kind = CV_ELLIPSE;
		out[k].o = x;
		out[k].x = major;
		out[k].n = cross(major, w);
		out[k].a = a->r / sn;
		out[k].b = a->r;
	}
	*n = 2;
	return SS_CURVES;
}

// Plane p and surface s.
static int plane_any(const struct surf *p, const struct surf *s,
		     struct curve *out, size_t *n)
{
	switch (s->kind) {
	case SF_PLANE:
		return plane_plane(p, s, out, n);
	case SF_CYL:
	case SF_CONE:
		return plane_curved(p, s, out, n);
	case SF_TORUS:
		return plane_torus(p, s, out, n);
	default:
		return SS_NOT;
	}
}

// Cylinder c and surface s, no plane.
static int cyl_any(const struct surf *c, const struct surf *s,
		   struct curve *out, size_t *n)
{
	if (s->kind == SF_CYL && len(cross(c->n, s->n)) >= TINY) {
		return cyl_cross(c, s, out, n);
	}
	if ((s->kind == SF_CYL || s->kind == SF_CONE) && coaxial(c, s)) {
		return round_round(c, s, out, n);
	}
	if (s->kind == SF_CYL) {
		// Along each other: apart, one inside the other, or they
		// meet on lines, which have no user.
		struct vec d = sub(s->o, c->o);
		double dd = len(sub(d, mul(c->n, dot(d, c->n))));

		if (dd > c->r + s->r + SH_EPS ||
		    dd < fabs(c->r - s->r) - SH_EPS) {
			return SS_NONE;
		}
	}
	return SS_NOT;
}

int surf_surf(const struct surf *a, const struct surf *b, struct curve *out,
	      size_t *n)
{
	*n = 0;
	if (a->kind > b->kind) {
		const struct surf *t = a;

		a = b;
		b = t;
	}
	if (a->kind == SF_PLANE) {
		return plane_any(a, b, out, n);
	}
	if (a->kind == SF_CYL) {
		return cyl_any(a, b, out, n);
	}
	if (a->kind == SF_CONE && b->kind == SF_CONE && coaxial(a, b)) {
		return round_round(a, b, out, n);
	}
	return SS_NOT;
}

// Whether the point of s at q lies where u has no meaning: the apex of
// a cone, or a pole of a sphere.
static int pole(const struct surf *s, struct uv q)
{
	if (s->kind == SF_CONE) {
		return fabs(s->r + (s->r2 * q.v)) < SH_EPS;
	}
	if (s->kind == SF_SPHERE) {
		return fabs(s->r * cos(q.v)) < SH_EPS;
	}
	return 0;
}

// The start point of the first loop of face f.
static struct vec first_point(const struct shape *sh, size_t f)
{
	const struct suse *u = &sh->u[sh->l[sh->f[f].loop0].first];
	const struct sedge *e = &sh->e[u->e];

	return sh->v[u->fwd ? e->v0 : e->v1];
}

// The middle of the arc of the circle of angles that holds the n angles
// a, shorter than a turn: across from the widest gap between them.  The
// angles get sorted.
static double arc_middle(double *a, size_t n)
{
	double gap = 0;
	double end = 0;

	for (size_t i = 0; i < n; i++) {
		a[i] -= 2 * PI * floor(a[i] / (2 * PI));
	}
	sort_doubles(a, n);
	for (size_t i = 0; i < n; i++) {
		double next = i + 1 < n ? a[i + 1] : a[0] + (2 * PI);

		if (next - a[i] > gap) {
			gap = next - a[i];
			end = next;
		}
	}
	return end + ((2 * PI - gap) / 2);
}

// The parameters of the points of the edges of a face, but at a pole.
struct samples {
	double *u, *v;
	size_t n, capu, capv;
	double lo, hi; // The least and the greatest v.
};

static void edge_samples(const struct surf *s, const struct sedge *e,
			 struct samples *sm)
{
	for (int i = 0; i <= EDGE_PTS; i++) {
		struct uv q =
			surf_param(s, edge_at(e, (double)i / (double)EDGE_PTS));
		size_t nv = sm->n;

		if (pole(s, q)) {
			continue;
		}
		PUSH(sm->u, sm->n, sm->capu, q.u);
		PUSH(sm->v, nv, sm->capv, q.v);
		sm->lo = fmin(sm->lo, q.v);
		sm->hi = fmax(sm->hi, q.v);
	}
}

void facemap_init(struct facemap *m, const struct shape *sh, size_t f)
{
	const struct surf *s = &sh->f[f].s;
	struct samples sm = { NULL, NULL, 0, 0, 0, HUGE_VAL, -HUGE_VAL };
	struct urange r = face_uses(sh, f);

	m->sh = sh;
	m->f = f;
	m->ref = surf_param(s, first_point(sh, f));
	if (s->kind == SF_PLANE) {
		return;
	}
	for (size_t k = r.first; k < r.end; k++) {
		edge_samples(s, &sh->e[sh->u[k].e], &sm);
	}
	if (sm.n) {
		m->ref.u = arc_middle(sm.u, sm.n);
		m->ref.v = s->kind == SF_TORUS ? arc_middle(sm.v, sm.n) :
						 (sm.lo + sm.hi) / 2;
	}
	free(sm.u);
	free(sm.v);
}

struct uv fm_uv(const struct facemap *m, struct vec p)
{
	const struct surf *s = &m->sh->f[m->f].s;
	struct uv q = surf_param(s, p);

	if (s->kind != SF_PLANE) {
		q.u = angle_near(q.u, m->ref.u);
	}
	if (s->kind == SF_SPHERE || s->kind == SF_TORUS) {
		q.v = angle_near(q.v, m->ref.v);
	}
	return q;
}

// The distance from p to the point of edge e at the share s of its way.
static double share_dist(const struct sedge *e, struct vec p, double s)
{
	return len(sub(p, edge_at(e, s)));
}

// The distance from p to edge e on a conic: the nearest of the samples,
// then the least between its neighbours, by the golden section.
static double conic_dist(const struct sedge *e, struct vec p)
{
	int best = 0;
	double d = HUGE_VAL;
	double lo = 0;
	double hi = 0;

	for (int i = 0; i <= EDGE_PTS; i++) {
		double di = share_dist(e, p, (double)i / (double)EDGE_PTS);

		if (di < d) {
			d = di;
			best = i;
		}
	}
	lo = fmax(0, (best - 1) / (double)EDGE_PTS);
	hi = fmin(1, (best + 1) / (double)EDGE_PTS);
	for (int k = 0; k < BISECT; k++) {
		double m1 = hi - (GOLD * (hi - lo));
		double m2 = lo + (GOLD * (hi - lo));

		if (share_dist(e, p, m1) < share_dist(e, p, m2)) {
			hi = m2;
		} else {
			lo = m1;
		}
	}
	return fmin(d, share_dist(e, p, (lo + hi) / 2));
}

double edge_dist(const struct sedge *e, struct vec p)
{
	const struct curve *c = &e->c;

	if (c->kind == CV_LINE) {
		double t = fmin(e->t1, fmax(e->t0, dot(sub(p, c->o), c->x)));

		return len(sub(p, curve_at(c, t)));
	}
	if (c->kind == CV_CIRCLE) {
		double t = curve_param(c, p);

		t = turn_from(t, e->t0);
		if (t <= e->t1) {
			return len(sub(p, curve_at(c, t)));
		}
		return fmin(len(sub(p, curve_at(c, e->t0))),
			    len(sub(p, curve_at(c, e->t1))));
	}
	return conic_dist(e, p);
}

struct plane3 iso_plane(const struct surf *s, struct vec p)
{
	struct plane3 pl = { s->n, dot(s->n, p) };

	if (s->kind == SF_PLANE) {
		pl.n = frame_y(s->n, s->x);
		pl.d = dot(pl.n, p);
	}
	return pl;
}

int cross_sign(const struct surf *s, struct plane3 pl, struct line3 at)
{
	double dv = surf_dv(s, at);

	if (fabs(dot(pl.n, at.d)) < GRAZE * len(at.d)) {
		return 0;
	}
	if (dv > 0) {
		return 1;
	}
	return dv < 0 ? -1 : 0;
}

// The winding number of the loops of face f about q, the parameters of
// p, by the crossings of the ray from q along +u; -1 if the ray meets an
// edge at its end.
static int winding(const struct facemap *m, struct vec p, struct uv q)
{
	const struct shape *sh = m->sh;
	const struct sface *f = &sh->f[m->f];
	struct plane3 pl = iso_plane(&f->s, p);
	int w = 0;

	for (size_t l = 0; l < f->nloops; l++) {
		const struct sloop *lp = &sh->l[f->loop0 + l];

		for (size_t k = 0; k < lp->n; k++) {
			const struct suse *u = &sh->u[lp->first + k];
			const struct sedge *e = &sh->e[u->e];
			struct cpiece pc = { e->c, e->t0, e->t1 };
			double t[ROOTS_MAX];
			size_t n = curve_plane(&pc, pl, t);

			for (size_t i = 0; i < n; i++) {
				struct vec r = curve_at(&e->c, t[i]);
				struct uv qr = fm_uv(m, r);
				struct line3 at = { r, curve_tan(&e->c, t[i]) };
				int dv = cross_sign(&f->s, pl, at) *
					 (u->fwd ? 1 : -1);

				if (fabs(qr.v - q.v) > SH_EPS || qr.u <= q.u) {
					continue;
				}
				if (len(sub(r, sh->v[e->v0])) < AT_VERTEX ||
				    len(sub(r, sh->v[e->v1])) < AT_VERTEX) {
					return -1;
				}
				w += dv;
			}
		}
	}
	return w != 0;
}

static int near_edges(const struct shape *sh, size_t f, struct vec p, double d)
{
	const struct sface *fc = &sh->f[f];

	for (size_t l = 0; l < fc->nloops; l++) {
		const struct sloop *lp = &sh->l[fc->loop0 + l];

		for (size_t k = 0; k < lp->n; k++) {
			if (edge_dist(&sh->e[sh->u[lp->first + k].e], p) < d) {
				return 1;
			}
		}
	}
	return 0;
}

int face_has(const struct shape *sh, size_t f, struct vec p)
{
	struct facemap m;

	if (near_edges(sh, f, p, SH_EPS)) {
		return 1;
	}
	facemap_init(&m, sh, f);
	for (int i = 0; i < NUDGES; i++) {
		struct vec pi = p;
		int w = 0;

		if (i) {
			pi = add(p,
				 mul(iso_plane(&sh->f[f].s, p).n, NUDGE * i));
		}
		w = winding(&m, pi, fm_uv(&m, pi));
		if (w >= 0) {
			return w;
		}
	}
	die("internal error: no ray across face %s misses its corners",
	    sh->f[f].name);
}

int face_on(const struct shape *sh, size_t f, struct vec p)
{
	return fabs(surf_dist(&sh->f[f].s, p)) < SH_EPS && face_has(sh, f, p);
}

// An angle where the height along d of curve c, a circle or an ellipse,
// stops growing or falling; the other one lies half a turn on.
static double turning(const struct curve *c, struct vec d)
{
	struct vec y = frame_y(c->n, c->x);
	double xa = dot(c->x, d);
	double ya = dot(y, d);

	return c->kind == CV_CIRCLE ? atan2(ya, xa) :
				      atan2(c->b * ya, c->a * xa);
}

// The least and the greatest height of edge e along the unit vector d.
static struct span edge_reach(const struct sedge *e, struct vec d)
{
	const struct curve *c = &e->c;
	double h0 = dot(curve_at(c, e->t0), d);
	double h1 = dot(curve_at(c, e->t1), d);
	struct span r = { fmin(h0, h1), fmax(h0, h1) };
	double t = 0;

	if (c->kind == CV_LINE) {
		return r;
	}
	t = turning(c, d);
	for (int k = 0; k < 2; k++) {
		double tk = turn_from(t + (k * PI), e->t0);

		if (tk > e->t0 && tk < e->t1) {
			double h = dot(curve_at(c, tk), d);

			r.lo = fmin(r.lo, h);
			r.hi = fmax(r.hi, h);
		}
	}
	return r;
}

struct box edge_box3(const struct sedge *e)
{
	struct span x = edge_reach(e, v3(1, 0, 0));
	struct span y = edge_reach(e, v3(0, 1, 0));
	struct span z = edge_reach(e, v3(0, 0, 1));
	struct box b = { v3(x.lo, y.lo, z.lo), v3(x.hi, y.hi, z.hi) };

	return b;
}

struct box face_box(const struct shape *sh, size_t f)
{
	const struct sface *fc = &sh->f[f];
	struct box x = box_empty();

	for (size_t l = 0; l < fc->nloops; l++) {
		const struct sloop *lp = &sh->l[fc->loop0 + l];

		for (size_t k = 0; k < lp->n; k++) {
			struct box b =
				edge_box3(&sh->e[sh->u[lp->first + k].e]);

			box_add(&x, b.lo);
			box_add(&x, b.hi);
		}
	}
	box_pad(&x, SH_EPS);
	return x;
}

static const double ray_dirs[RAYS][3] = {
	{ 0.5734, 0.4176, 0.7049 },   { -0.3218, 0.8371, 0.4422 },
	{ 0.7963, -0.2511, -0.5503 }, { -0.6142, -0.5233, 0.5907 },
	{ 0.1307, -0.9214, 0.3659 },  { -0.8812, 0.2049, -0.4262 }
};

// The crossings of the ray with the faces of sh, or -1 if the ray is
// unsafe: it touches a surface, or passes near an edge.
static int ray_count(const struct shape *sh, struct line3 ray, double tmax)
{
	int count = 0;

	for (size_t f = 0; f < sh->nf; f++) {
		double t[ROOTS_MAX];
		int tangent = 0;
		size_t n = line_surf(ray, tmax, &sh->f[f].s, t, &tangent);

		for (size_t i = 0; i < n; i++) {
			struct vec h = add(ray.o, mul(ray.d, t[i]));

			if (!face_has(sh, f, h)) {
				continue;
			}
			if (tangent || t[i] < NEAR_EDGE ||
			    near_edges(sh, f, h, NEAR_EDGE)) {
				return -1;
			}
			count++;
		}
	}
	return count;
}

int solid_has(const struct shape *sh, struct vec p)
{
	struct box x = box_empty();
	double tmax = 0;

	for (size_t i = 0; i < sh->nv; i++) {
		box_add(&x, sh->v[i]);
	}
	for (size_t i = 0; i < sh->ne; i++) {
		struct box b = edge_box3(&sh->e[i]);

		box_add(&x, b.lo);
		box_add(&x, b.hi);
	}
	if (!box_has(x, p)) {
		return 0;
	}
	tmax = len(sub(x.hi, x.lo)) * 2;
	for (int i = 0; i < RAYS; i++) {
		struct line3 ray = { p, unit(v3(ray_dirs[i][0], ray_dirs[i][1],
						ray_dirs[i][2])) };
		int c = ray_count(sh, ray, tmax);

		if (c >= 0) {
			return c % 2;
		}
	}
	die("internal error: no ray from a point misses the edges of %s",
	    sh->name);
}

#define PIECE (PI / 8) // The longest piece of an arc held in a triangle.

// Whether every point of face f at the height of a plane across the
// axis d that bounds its box lies on its edges: so for a plane not
// across the axis, and for a cylinder or a cone along it.
static int ends_on_edges(const struct sface *f, struct vec d)
{
	const struct surf *s = &f->s;
	double along = fabs(dot(s->n, d));

	if (s->kind == SF_PLANE) {
		return along < 1 - TINY;
	}
	return (s->kind == SF_CYL || s->kind == SF_CONE) && along > 1 - TINY;
}

// Whether the faces fa of sa and fb of sb lie on the two sides of a
// plane across an axis and reach it on their edges alone: they then
// share points of their edges at most, and no curve cuts either.
static int only_touch(const struct shape *sa, size_t fa, const struct shape *sb,
		      size_t fb)
{
	struct box a = face_box(sa, fa);
	struct box b = face_box(sb, fb);
	struct vec over = v3(fmin(a.hi.x, b.hi.x) - fmax(a.lo.x, b.lo.x),
			     fmin(a.hi.y, b.hi.y) - fmax(a.lo.y, b.lo.y),
			     fmin(a.hi.z, b.hi.z) - fmax(a.lo.z, b.lo.z));
	const struct vec axes[3] = { v3(1, 0, 0), v3(0, 1, 0), v3(0, 0, 1) };

	for (int k = 0; k < 3; k++) {
		if (dot(over, axes[k]) < 3 * SH_EPS &&
		    ends_on_edges(&sa->f[fa], axes[k]) &&
		    ends_on_edges(&sb->f[fb], axes[k])) {
			return 1;
		}
	}
	return 0;
}

// The least and the greatest height of the edges of face f of sh along
// d, above the point o.
static struct span face_reach(const struct shape *sh, size_t f, struct vec o,
			      struct vec d)
{
	const struct sface *fc = &sh->f[f];
	struct span r = { HUGE_VAL, -HUGE_VAL };

	for (size_t l = 0; l < fc->nloops; l++) {
		const struct sloop *lp = &sh->l[fc->loop0 + l];

		for (size_t k = 0; k < lp->n; k++) {
			struct span e =
				edge_reach(&sh->e[sh->u[lp->first + k].e], d);

			r.lo = fmin(r.lo, e.lo - dot(o, d));
			r.hi = fmax(r.hi, e.hi - dot(o, d));
		}
	}
	return r;
}

// Whether a face is made of straight lines: a plane, a cylinder or a
// cone.
static int ruled(const struct sface *f)
{
	return f->s.kind == SF_PLANE || f->s.kind == SF_CYL ||
	       f->s.kind == SF_CONE;
}

// Whether face fp of p, on a plane, has face fo of o, made of straight
// lines, wholly on one side.  The height of a point of fo above the
// plane is greatest and least on its edges, since it changes evenly
// along each line of the face.
static int plane_clear(const struct shape *p, size_t fp, const struct shape *o,
		       size_t fo)
{
	const struct surf *s = &p->f[fp].s;
	struct span r;

	if (s->kind != SF_PLANE || !ruled(&o->f[fo])) {
		return 0;
	}
	r = face_reach(o, fo, s->o, s->n);
	return r.lo > SH_EPS || r.hi < -SH_EPS;
}

// The radius of s, a cylinder or a cone, at the height h along its axis
// of the point p.
static double radius_by(const struct surf *s, struct vec p)
{
	return s->kind == SF_CONE ? s->r + (s->r2 * dot(sub(p, s->o), s->n)) :
				    s->r;
}

// Whether the circles of radii ra and rb, their centres d apart, are
// clear of each other, and which way: 1 apart, 2 the second inside the
// first, 3 the first inside the second; 0 if they may meet.
static int circles_clear(double ra, double rb, double d)
{
	if (d > ra + rb + SH_EPS) {
		return 1;
	}
	if (ra - rb > d + SH_EPS) {
		return 2;
	}
	return rb - ra > d + SH_EPS ? 3 : 0;
}

// Whether the faces fa of a and fb of b, on cylinders or cones with
// parallel axes, are clear of each other: at both ends of the stretch
// of the axis that both reach, the circles of the two surfaces are
// clear the same way.  The radii change evenly between.
static int rounds_clear(const struct shape *sa, size_t fa,
			const struct shape *sb, size_t fb)
{
	const struct surf *a = &sa->f[fa].s;
	const struct surf *b = &sb->f[fb].s;
	struct vec off = sub(b->o, a->o);
	double d = len(sub(off, mul(a->n, dot(off, a->n))));
	struct span ra;
	struct span rb;
	struct span both;
	struct vec lo;
	struct vec hi;
	int how = 0;

	if (a->kind == SF_PLANE || b->kind == SF_PLANE || !ruled(&sa->f[fa]) ||
	    !ruled(&sb->f[fb]) || len(cross(a->n, b->n)) > TINY) {
		return 0;
	}
	ra = face_reach(sa, fa, a->o, a->n);
	rb = face_reach(sb, fb, a->o, a->n);
	both.lo = fmax(ra.lo, rb.lo) - SH_EPS;
	both.hi = fmin(ra.hi, rb.hi) + SH_EPS;
	if (both.lo > both.hi) {
		return 1;
	}
	lo = add(a->o, mul(a->n, both.lo));
	hi = add(a->o, mul(a->n, both.hi));
	how = circles_clear(radius_by(a, lo), radius_by(b, lo), d);
	return how &&
	       how == circles_clear(radius_by(a, hi), radius_by(b, hi), d);
}

// How far p lies outside s, a cylinder or a cone: its distance from the
// axis less the radius at its height.  Along any straight line this
// distance turns one way, as that from the axis does.
static double outside(const struct surf *s, struct vec p)
{
	struct vec d = sub(p, s->o);

	return len(sub(d, mul(s->n, dot(d, s->n)))) - radius_by(s, p);
}

// Whether edge e lies inside s, a cylinder or a cone, by more than the
// tolerance: a line by its ends; a circle or an ellipse by the ends of
// its pieces of at most PIECE and the points where the tangents at
// those ends cross, since each piece lies within that triangle.
static int edge_inside(const struct sedge *e, const struct surf *s)
{
	const struct curve *c = &e->c;
	size_t n = 1;
	double step = 0;

	while (c->kind != CV_LINE && (e->t1 - e->t0) / (double)n > PIECE) {
		n++;
	}
	step = (e->t1 - e->t0) / (double)n;

	for (size_t i = 0; i <= n; i++) {
		double t = e->t0 + (step * (double)i);
		struct vec p = curve_at(c, t);

		if (c->kind != CV_LINE && i < n) {
			struct vec m = sub(curve_at(c, t + (step / 2)), c->o);
			struct vec q = add(c->o, mul(m, 1 / cos(step / 2)));

			if (outside(s, q) > -SH_EPS) {
				return 0;
			}
		}
		if (outside(s, p) > -SH_EPS) {
			return 0;
		}
	}
	return 1;
}

// Whether face fp of p, on a plane, lies wholly inside the cylinder or
// the cone of face fo of o: a face on a plane lies between its edges.
static int plane_inside(const struct shape *p, size_t fp, const struct shape *o,
			size_t fo)
{
	const struct sface *fc = &p->f[fp];
	const struct surf *s = &o->f[fo].s;

	if (fc->s.kind != SF_PLANE ||
	    (s->kind != SF_CYL && s->kind != SF_CONE)) {
		return 0;
	}
	for (size_t l = 0; l < fc->nloops; l++) {
		const struct sloop *lp = &p->l[fc->loop0 + l];

		for (size_t k = 0; k < lp->n; k++) {
			if (!edge_inside(&p->e[p->u[lp->first + k].e], s)) {
				return 0;
			}
		}
	}
	return 1;
}

int faces_clear(const struct shape *a, size_t fa, const struct shape *b,
		size_t fb)
{
	return only_touch(a, fa, b, fb) || plane_clear(a, fa, b, fb) ||
	       plane_clear(b, fb, a, fa) || rounds_clear(a, fa, b, fb) ||
	       plane_inside(a, fa, b, fb) || plane_inside(b, fb, a, fa);
}
