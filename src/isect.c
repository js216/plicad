// SPDX-License-Identifier: GPL-3.0-only
// isect.c -- Where curves, surfaces and faces of exact solids meet
// Copyright (c) 2026 Jakob Kastelic

// A curve meets a plane where a sum of cosines and sines, or a
// polynomial, vanishes: those roots come in closed form.  A curve meets
// a curved surface where the signed distance to the surface vanishes
// along it: samples bracket each root, or show a touch where the
// distance comes near zero, and bisection or a golden-section search
// pins it down to the last bits.  A line meets a quadric or a torus at
// the roots of a polynomial in its parameter, and a pipe at those of
// one of degree 8 in tan(u / 2) of its ellipse (pipe_poly), on two
// halves of it; the roots and the turning points of a polynomial come
// apart between those of its derivatives.

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
#define HALF 0.5
#define OVERLAP 1.1 // The reach of a half of an ellipse in tan(u / 2).
#define INDEX_PAD 1e-5 // How far the index of a face reaches past an edge.

enum {
	SAMPLES = 128, // Samples of a curve against a surface.
	BISECT = 80, // Halvings of a bracket.
	RAYS = 6, // Directions of rays for an inside test.
	EDGE_PTS = 64, // Points of an edge for its box, or its distance.
	NODAL_PTS = 256, // Points of a nodal section held inside a surface.
	NUDGES = 4, // Tries of a ray across a face.
	QUARTIC = 4, // The degree of the implicit torus along a line.
	POLY_MAX = 8, // The highest degree of a polynomial of a root search.
	POLISH = 3, // Newton steps from a root of a polynomial to the surface.
	INDEX_BINS = 256 // The most bins of the index of a face.
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
	if (s->kind == SF_PLANE && p->c.kind != CV_NODAL) {
		struct plane3 pl = { s->n, dot(s->n, s->o) };

		return curve_on_surf(p, s) ? 0 : curve_plane(p, pl, t);
	}
	if (curve_on_surf(p, s)) {
		return 0;
	}
	if (p->c.kind == CV_LINE) {
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

// A polynomial of degree at most POLY_MAX in t: c[k] t^k.
struct poly {
	double c[POLY_MAX + 1];
	int deg;
};

static double poly_at(const struct poly *p, double t)
{
	double v = 0;

	for (int k = p->deg; k >= 0; k--) {
		v = (v * t) + p->c[k];
	}
	return v;
}

// The root of p in [a, b], where p changes sign, by halvings.
static double poly_bisect(const struct poly *p, double a, double b)
{
	double fa = poly_at(p, a);

	for (int i = 0; i < BISECT; i++) {
		double m = (a + b) / 2;
		double fm = poly_at(p, m);

		if (m <= a || m >= b) {
			break;
		}
		if ((fm < 0) == (fa < 0)) {
			a = m;
			fa = fm;
		} else {
			b = m;
		}
	}
	return (a + b) / 2;
}

// The points of [lo, hi] where p changes sign, and its turning points
// there, cut[1 .. nc - 2], into r in order; marks in turn[i] those that
// are turning points.  Between two turning points p is monotone and has
// one root at most.  Returns their count.
static size_t poly_pieces(const struct poly *p, const double *cut, size_t nc,
			  double *r, int *turn)
{
	size_t n = 0;

	for (size_t i = 0; i + 1 < nc; i++) {
		double fa = poly_at(p, cut[i]);
		double fb = poly_at(p, cut[i + 1]);

		if (i > 0) {
			turn[n] = 1;
			r[n++] = cut[i];
		}
		if ((fa < 0 && fb > 0) || (fa > 0 && fb < 0)) {
			turn[n] = 0;
			r[n++] = poly_bisect(p, cut[i], cut[i + 1]);
		}
	}
	return n;
}

// The roots and turning points of p in the span in, as poly_pieces gives
// them.  The turning points are the roots of the derivative, which come
// the same way from those of the next derivative, from the line up.
static size_t poly_points(const struct poly *p, struct span in, double *r,
			  int *turn)
{
	struct poly dv[POLY_MAX + 1];
	double cut[POLY_MAX + 2];
	size_t nc = 0;
	size_t n = 0;

	memset(dv, 0, sizeof(dv));
	dv[0] = *p;
	for (int k = 1; k < p->deg && k <= POLY_MAX; k++) {
		dv[k].deg = dv[k - 1].deg - 1;
		for (int j = 1; j <= dv[k - 1].deg; j++) {
			dv[k].c[j - 1] = j * dv[k - 1].c[j];
		}
	}
	cut[nc++] = in.lo;
	cut[nc++] = in.hi;
	for (int k = p->deg - 1; k >= 0; k--) {
		n = poly_pieces(&dv[k], cut, nc, r, turn);
		if (k == 0) {
			break;
		}
		nc = 0;
		cut[nc++] = in.lo;
		for (size_t i = 0; i < n; i++) {
			if (!turn[i] && r[i] > cut[nc - 1] && r[i] < in.hi) {
				cut[nc++] = r[i];
			}
		}
		cut[nc++] = in.hi;
	}
	return n;
}

// The distance of point p, from the center of torus s, to the torus.
static double torus_gap(const struct surf *s, struct vec p)
{
	double h = dot(p, s->n);
	double rho = len(sub(p, mul(s->n, h)));

	return fabs(hypot(rho - s->r, h) - s->r2);
}

// The parameters t in [0, tmax] where the line meets torus s, the roots
// of the quartic of its implicit form along the line,
//   (|p|^2 + R^2 - rt^2)^2 = 4 R^2 (|p|^2 - (p . n)^2),
// with p measured from the center of the torus and t from the point of
// the line nearest the center, where p is small, so that no large terms
// cancel.  A turning point of the quartic that lies within TOUCH of the
// torus is a touch.
static size_t line_torus(struct line3 ray, double tmax, const struct surf *s,
			 double *t, int *tangent)
{
	struct vec d = ray.d;
	double a = dot(d, d);
	double t0 = -dot(sub(ray.o, s->o), d) / a;
	struct vec q = add(sub(ray.o, s->o), mul(d, t0));
	double big = s->r * s->r;
	double qn = dot(q, s->n);
	double dn = dot(d, s->n);
	double b = 2 * dot(q, d);
	double c = dot(q, q) + big - (s->r2 * s->r2);
	double a2 = a - (dn * dn);
	double a1 = 2 * (dot(q, d) - (qn * dn));
	double a0 = dot(q, q) - (qn * qn);
	struct poly p;
	double r[(2 * POLY_MAX) + 2];
	int turn[(2 * POLY_MAX) + 2];
	size_t m = 0;
	size_t n = 0;

	if (len(q) > s->r + s->r2 + TOUCH) {
		return 0;
	}
	memset(&p, 0, sizeof(p));
	p.deg = QUARTIC;
	p.c[4] = a * a;
	p.c[3] = 2 * a * b;
	p.c[2] = (b * b) + (2 * a * c) - (4 * big * a2);
	p.c[1] = (2 * b * c) - (4 * big * a1);
	p.c[0] = (c * c) - (4 * big * a0);
	m = poly_points(&p, (struct span){ -t0, tmax - t0 }, r, turn);
	for (size_t i = 0; i < m && n < ROOTS_MAX; i++) {
		if (turn[i]) {
			if (torus_gap(s, add(q, mul(d, r[i]))) > TOUCH) {
				continue;
			}
			*tangent = 1;
		}
		t[n++] = r[i] + t0;
	}
	return n;
}

// The product of polynomials a and b, whose degrees sum to POLY_MAX at
// most, and the sum a + k b.
static struct poly poly_mul(const struct poly *a, const struct poly *b)
{
	struct poly r;

	memset(&r, 0, sizeof(r));
	r.deg = a->deg + b->deg;
	for (int i = 0; i <= a->deg; i++) {
		for (int j = 0; j <= b->deg && i + j <= POLY_MAX; j++) {
			r.c[i + j] += a->c[i] * b->c[j];
		}
	}
	r.deg = r.deg > POLY_MAX ? POLY_MAX : r.deg;
	return r;
}

static struct poly poly_sum(const struct poly *a, const struct poly *b,
			    double k)
{
	struct poly r = *a;

	r.deg = a->deg > b->deg ? a->deg : b->deg;
	for (int i = a->deg + 1; i <= r.deg; i++) {
		r.c[i] = 0;
	}
	for (int i = 0; i <= b->deg; i++) {
		r.c[i] += k * b->c[i];
	}
	return r;
}

// A vector of polynomials, in the frame of a pipe.
struct vpoly {
	struct poly c[3];
};

static struct poly vp_dot(const struct vpoly *a, const struct vpoly *b)
{
	struct poly r = poly_mul(&a->c[0], &b->c[0]);

	for (int k = 1; k < 3; k++) {
		struct poly m = poly_mul(&a->c[k], &b->c[k]);

		r = poly_sum(&r, &m, 1);
	}
	return r;
}

static struct vpoly vp_cross(const struct vpoly *a, const struct vpoly *b)
{
	struct vpoly r;

	for (int k = 0; k < 3; k++) {
		int i = (k + 1) % 3;
		int j = (k + 2) % 3;
		struct poly p = poly_mul(&a->c[i], &b->c[j]);
		struct poly q = poly_mul(&a->c[j], &b->c[i]);

		r.c[k] = poly_sum(&p, &q, -1);
	}
	return r;
}

// The polynomial of degree 0 to 2 of the coefficients k0, k1, k2.
static struct poly quad_poly(double k0, double k1, double k2)
{
	struct poly r;

	memset(&r, 0, sizeof(r));
	r.deg = 2;
	r.c[0] = k0;
	r.c[1] = k1;
	r.c[2] = k2;
	return r;
}

// The ray q + t d, q and d in the frame of a pipe s, from the point
// nearest its center: the parameters t from lo to hi.
struct pipe_ray {
	const struct surf *s;
	struct vec q, d;
	struct vec qw, dw; // The same in the world.
	struct span in;
};

// The polynomial H in s, for u = us + 2 atan s, whose roots are the
// angles u of the points c(u) of the ellipse of the pipe at whose ring
// the ray meets the pipe.  The ray meets the ball about c(u) along its
// normal plane where
//   (q + t d - c) . c' = 0,  |q + t d - c|^2 = rt^2;
// the first gives t, and the second then holds where
//   |c' x ((q - c) x d)|^2 - rt^2 (d . c')^2 = 0,
// which times (1 + s^2)^4 is H, of degree 8 in s.
static struct poly pipe_poly(const struct pipe_ray *pr, double us)
{
	const struct surf *s = pr->s;
	double cs = cos(us);
	double ss = sin(us);
	struct poly cn = quad_poly(cs, -2 * ss, -cs);
	struct poly sn = quad_poly(ss, 2 * cs, -ss);
	struct poly wd = quad_poly(1, 0, 1);
	struct vpoly o;
	struct vpoly dd;
	struct vpoly dc;
	struct vpoly od;
	struct vpoly v;
	struct poly h;
	struct poly k;
	struct poly w2;
	const double q[3] = { pr->q.x, pr->q.y, pr->q.z };
	const double d[3] = { pr->d.x, pr->d.y, pr->d.z };

	for (int i = 0; i < 3; i++) {
		double ax = i == 0 ? s->r : s->r3;
		const struct poly *c = i == 0 ? &cn : &sn;

		o.c[i] = quad_poly(q[i], 0, q[i]);
		if (i < 2) {
			o.c[i] = poly_sum(&o.c[i], c, -ax);
		}
		memset(&dd.c[i], 0, sizeof(dd.c[i]));
		dd.c[i].c[0] = d[i];
	}
	dc.c[0] = quad_poly(-s->r * sn.c[0], -s->r * sn.c[1], -s->r * sn.c[2]);
	dc.c[1] = quad_poly(s->r3 * cn.c[0], s->r3 * cn.c[1], s->r3 * cn.c[2]);
	dc.c[2] = quad_poly(0, 0, 0);
	od = vp_cross(&o, &dd);
	v = vp_cross(&dc, &od);
	h = vp_dot(&v, &v);
	k = vp_dot(&dd, &dc);
	k = poly_mul(&k, &k);
	w2 = poly_mul(&wd, &wd);
	k = poly_mul(&k, &w2);
	return poly_sum(&h, &k, -(s->r2 * s->r2));
}

// Adds the parameter t of the ray to the n of r, moved onto the pipe by
// Newton steps, if it lies on the pipe in the span of the ray: within
// TOUCH, or within SH_EPS for a touch, which sets *tangent.
static void pipe_hit(const struct pipe_ray *pr, double t, int turn, double *r,
		     size_t *n, int *tangent)
{
	const struct surf *s = pr->s;
	double f = 0;

	for (int i = 0; i < POLISH; i++) {
		struct vec p = add(pr->qw, mul(pr->dw, t));
		double slope = dot(pr->dw, surf_normal(s, p));

		f = surf_dist(s, p);
		if (fabs(slope) < GRAZE * len(pr->dw)) {
			break;
		}
		t -= f / slope;
	}
	f = surf_dist(s, add(pr->qw, mul(pr->dw, t)));
	if (t < pr->in.lo || t > pr->in.hi ||
	    fabs(f) > (turn ? SH_EPS : TOUCH)) {
		return;
	}
	for (size_t i = 0; i < *n; i++) {
		if (fabs(r[i] - t) < SH_EPS) {
			return;
		}
	}
	*tangent = *tangent || turn ||
		   fabs(dot(unit(pr->dw),
			    surf_normal(s, add(pr->qw, mul(pr->dw, t))))) <
			   GRAZE;
	if (*n < ROOTS_MAX) {
		r[(*n)++] = t;
	}
}

// The points of the ray for the root at u, the angle of the ring.
static void pipe_ring_hits(const struct pipe_ray *pr, double u, int turn,
			   double *r, size_t *n, int *tangent)
{
	const struct surf *s = pr->s;
	struct vec c = v3(s->r * cos(u), s->r3 * sin(u), 0);
	struct vec c1 = v3(-s->r * sin(u), s->r3 * cos(u), 0);
	struct vec qc = sub(pr->q, c);
	double dc = dot(pr->d, c1);
	double t[2];
	size_t m = 0;

	if (fabs(dc) > TINY * len(c1)) {
		t[0] = -dot(qc, c1) / dc;
		m = 1;
	} else {
		// The ray lies in the normal plane of the ring: where it
		// meets the ring itself.
		m = quadratic(dot(pr->d, pr->d), 2 * dot(qc, pr->d),
			      dot(qc, qc) - (s->r2 * s->r2), t);
		*tangent = 1;
	}
	for (size_t i = 0; i < m; i++) {
		pipe_hit(pr, t[i], turn, r, n, tangent);
	}
}

// The parameters t in [0, tmax] where the line meets pipe s: the roots
// of H (pipe_poly) on two halves of the ellipse, about u = 0 and u = pi,
// each with s in [-OVERLAP, OVERLAP] so that they overlap, give the
// rings of the hits, and so the hits, one each.
// A turning point of H whose point lies within SH_EPS of the pipe is a
// touch, and so is a hit where the line lies in the plane of a ring;
// the points where the line meets the pipe where it folds, off the
// nearest ring, are none.
static size_t line_pipe(struct line3 ray, double tmax, const struct surf *s,
			double *t, int *tangent)
{
	struct vec d = ray.d;
	double a = dot(d, d);
	double t0 = -dot(sub(ray.o, s->o), d) / a;
	struct vec qw = add(ray.o, mul(d, t0));
	struct vec y = frame_y(s->n, s->x);
	struct vec dq = sub(qw, s->o);
	struct pipe_ray pr;
	size_t n = 0;

	if (len(dq) > fmax(s->r, s->r3) + s->r2 + TOUCH) {
		return 0;
	}
	pr.s = s;
	pr.q = v3(dot(dq, s->x), dot(dq, y), dot(dq, s->n));
	pr.d = v3(dot(d, s->x), dot(d, y), dot(d, s->n));
	pr.qw = qw;
	pr.dw = d;
	pr.in = (struct span){ -t0, tmax - t0 };
	for (int half = 0; half < 2; half++) {
		double us = half * PI;
		struct poly p = pipe_poly(&pr, us);
		double r[(2 * POLY_MAX) + 2];
		int turn[(2 * POLY_MAX) + 2];
		size_t m = poly_points(&p, (struct span){ -OVERLAP, OVERLAP },
				       r, turn);

		for (size_t i = 0; i < m; i++) {
			pipe_ring_hits(&pr, us + (2 * atan(r[i])), turn[i], t,
				       &n, tangent);
		}
	}
	for (size_t i = 0; i < n; i++) {
		t[i] += t0;
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
	if (s->kind == SF_SPHERE) {
		struct quadric k = { dot(d, d), 2 * dot(q, d),
				     dot(q, q) - (s->r * s->r), s->r };

		return line_quadric(k, tmax, t, tangent);
	}
	if (s->kind == SF_TORUS) {
		return line_torus(ray, tmax, s, t, tangent);
	}
	if (s->kind == SF_PIPE) {
		return line_pipe(ray, tmax, s, t, tangent);
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

// Plane p and pipe s, for a plane square to the ellipse of the pipe where
// it holds a point of it: the ring there, the circle of the tube in the
// plane.  The tangent of the ellipse, (-a sin u, b cos u) in its frame,
// lies along the normal (mx, my) of the plane at u and u + pi, where
// tan u = -b mx / (a my).  The rest of where they meet has no user.
static int plane_pipe(const struct surf *p, const struct surf *s,
		      struct curve *out, size_t *n)
{
	struct vec y = frame_y(s->n, s->x);
	double mx = dot(p->n, s->x);
	double my = dot(p->n, y);
	double u = atan2(-s->r3 * mx, s->r * my);

	if (fabs(dot(p->n, s->n)) >= TINY) {
		return SS_NOT;
	}
	for (int k = 0; k < 2; k++) {
		struct line3 c = pipe_ring(s, u + (k * PI));

		if (fabs(dot(p->n, sub(c.o, p->o))) < SH_EPS) {
			memset(&out[*n], 0, sizeof(*out));
			out[*n].kind = CV_CIRCLE;
			out[*n].o = c.o;
			out[*n].n = p->n;
			out[*n].x = c.d;
			out[*n].a = s->r2;
			(*n)++;
		}
	}
	return *n ? SS_CURVES : SS_NOT;
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
	case SF_PIPE:
		return plane_pipe(p, s, out, n);
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
	// A line of a cylinder or a cone is a ruling, of one u: its ends
	// tell all of it.
	int n = e->c.kind == CV_LINE &&
				(s->kind == SF_CYL || s->kind == SF_CONE) ?
			1 :
			EDGE_PTS;

	for (int i = 0; i <= n; i++) {
		struct uv q = surf_param(s, edge_at(e, (double)i / (double)n));
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
	m->ebox = NULL;
	m->index = NULL;
	m->ref = surf_param(s, first_point(sh, f));
	if (s->kind == SF_PLANE) {
		return;
	}
	for (size_t k = r.first; k < r.end; k++) {
		edge_samples(s, &sh->e[sh->u[k].e], &sm);
	}
	if (sm.n) {
		m->ref.u = arc_middle(sm.u, sm.n);
		m->ref.v = s->kind == SF_TORUS || s->kind == SF_PIPE ?
				   arc_middle(sm.v, sm.n) :
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
	if (s->kind == SF_SPHERE || s->kind == SF_TORUS || s->kind == SF_PIPE) {
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
// then the least between its neighbors, by the golden section.
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

// Whether box b lies wholly on one side of the plane, SH_EPS or more off.
static int box_misses(struct box b, struct plane3 pl)
{
	struct vec c = mul(add(b.lo, b.hi), HALF);
	struct vec h = mul(sub(b.hi, b.lo), HALF);
	double r = (fabs(pl.n.x) * h.x) + (fabs(pl.n.y) * h.y) +
		   (fabs(pl.n.z) * h.z);

	return fabs(dot(pl.n, c) - pl.d) > r + SH_EPS;
}

// The bin of the index x that holds the height d, clamped.
static size_t bin_of(const struct face_index *x, double d)
{
	double k = x->step > 0 ? (d - x->lo) / x->step : 0;
	size_t b = 0;

	while ((double)(b + 1) <= k && b + 1 < x->nb) {
		b++;
	}
	return b;
}

// The uses of the edges of a face whose spans along the normal of its
// iso-planes hold the height d, into *out; returns their count.
static size_t index_uses(const struct face_index *x, double d,
			 const size_t **out)
{
	double k = x->step > 0 ? (d - x->lo) / x->step : 0;
	size_t b = 0;

	if (!(k >= 0) || !(k < (double)x->nb)) {
		*out = x->uses;
		return 0;
	}
	b = bin_of(x, d);
	*out = x->uses + x->start[b];
	return x->start[b + 1] - x->start[b];
}

// The winding number of the loops of face f about q, the parameters of
// p, by the crossings of the ray from q along +u; -1 if the ray meets an
// edge at its end.
static int winding(const struct facemap *m, struct vec p, struct uv q)
{
	const struct shape *sh = m->sh;
	const struct sface *f = &sh->f[m->f];
	struct plane3 pl = iso_plane(&f->s, p);
	struct urange all = face_uses(sh, m->f);
	const size_t *list = NULL;
	size_t nl = 0;
	int w = 0;

	if (m->index) {
		nl = index_uses(m->index, pl.d, &list);
	}
	for (size_t j = 0; j < (list ? nl : all.end - all.first); j++) {
		{
			const struct suse *u =
				&sh->u[list ? list[j] : all.first + j];
			const struct sedge *e = &sh->e[u->e];
			struct cpiece pc = { e->c, e->t0, e->t1 };
			double t[ROOTS_MAX];
			size_t n = 0;

			if (m->ebox && box_misses(m->ebox[u->e], pl)) {
				continue;
			}
			n = curve_plane(&pc, pl, t);

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

// Whether p lies within d of an edge of face f, the edges of boxes ebox
// (each grown by d), or of any box if ebox is NULL; with the index x of
// the face, for d no more than INDEX_PAD, only the edges it names at p.
static int near_edges_in(const struct shape *sh, const struct box *ebox,
			 const struct face_index *x, size_t f, struct vec p,
			 double d)
{
	struct urange all = face_uses(sh, f);
	const size_t *list = NULL;
	size_t nl = 0;

	if (x && d <= INDEX_PAD) {
		nl = index_uses(x, dot(x->n, p), &list);
	}
	for (size_t j = 0; j < (list ? nl : all.end - all.first); j++) {
		size_t e = sh->u[list ? list[j] : all.first + j].e;

		if (ebox) {
			struct box b = ebox[e];

			box_pad(&b, d);
			if (!box_has(b, p)) {
				continue;
			}
		}
		if (edge_dist(&sh->e[e], p) < d) {
			return 1;
		}
	}
	return 0;
}

static int near_edges(const struct shape *sh, size_t f, struct vec p, double d)
{
	return near_edges_in(sh, NULL, NULL, f, p, d);
}

// As face_has, with the facemap m of the face.
static int face_has_m(const struct facemap *m, const struct box *ebox,
		      struct vec p)
{
	const struct shape *sh = m->sh;
	size_t f = m->f;

	if (near_edges_in(sh, ebox, m->index, f, p, SH_EPS)) {
		return 1;
	}
	for (int i = 0; i < NUDGES; i++) {
		struct vec pi = p;
		int w = 0;

		if (i) {
			pi = add(p,
				 mul(iso_plane(&sh->f[f].s, p).n, NUDGE * i));
		}
		w = winding(m, pi, fm_uv(m, pi));
		if (w >= 0) {
			return w;
		}
	}
	die("internal error: no ray across face %s misses its corners",
	    sh->f[f].name);
}

int face_has(const struct shape *sh, size_t f, struct vec p)
{
	struct facemap m;

	if (near_edges(sh, f, p, SH_EPS)) {
		return 1;
	}
	facemap_init(&m, sh, f);
	return face_has_m(&m, NULL, p);
}

// Builds the index of the uses of the edges of face f of sh, by their
// spans along the normal n of the iso-planes of the face, each the span
// of the box of its edge, grown by INDEX_PAD.
static void index_face(struct face_index *x, const struct shape *sh,
		       const struct box *ebox, size_t f)
{
	struct urange r = face_uses(sh, f);
	size_t n = r.end - r.first;
	double *lo = xmalloc((n + 1) * sizeof(*lo));
	double *hi = xmalloc((n + 1) * sizeof(*hi));
	double top = -HUGE_VAL;

	x->n = iso_plane(&sh->f[f].s, sh->v[0]).n;
	x->lo = HUGE_VAL;
	for (size_t i = 0; i < n; i++) {
		struct box b = ebox[sh->u[r.first + i].e];
		struct vec c = mul(add(b.lo, b.hi), HALF);
		struct vec h = mul(sub(b.hi, b.lo), HALF);
		double w = (fabs(x->n.x) * h.x) + (fabs(x->n.y) * h.y) +
			   (fabs(x->n.z) * h.z) + INDEX_PAD;

		lo[i] = dot(x->n, c) - w;
		hi[i] = dot(x->n, c) + w;
		x->lo = fmin(x->lo, lo[i]);
		top = fmax(top, hi[i]);
	}
	x->nb = n < INDEX_BINS ? n : INDEX_BINS;
	x->nb = x->nb ? x->nb : 1;
	x->step = n ? (top - x->lo) / (double)x->nb : 0;
	x->start = xcalloc(x->nb + 2, sizeof(*x->start));
	for (int pass = 0; pass < 2; pass++) {
		size_t *fill = pass ? xcalloc(x->nb + 1, sizeof(*fill)) : NULL;

		for (size_t i = 0; i < n; i++) {
			size_t b0 = bin_of(x, lo[i]);
			size_t b1 = bin_of(x, hi[i]);

			for (size_t b = b0; b <= b1; b++) {
				if (pass) {
					x->uses[x->start[b] + fill[b]++] =
						r.first + i;
				} else {
					x->start[b + 1]++;
				}
			}
		}
		if (!pass) {
			for (size_t b = 0; b < x->nb; b++) {
				x->start[b + 1] += x->start[b];
			}
			x->uses = xmalloc((x->start[x->nb] + 1) *
					  sizeof(*x->uses));
		}
		free(fill);
	}
	free(lo);
	free(hi);
}

void shape_cache_init(struct shape_cache *c, const struct shape *sh)
{
	c->sh = sh;
	c->quick = NULL;
	c->qctx = NULL;
	c->fm = xmalloc((sh->nf + 1) * sizeof(*c->fm));
	c->ebox = xmalloc((sh->ne + 1) * sizeof(*c->ebox));
	c->fbox = xmalloc((sh->nf + 1) * sizeof(*c->fbox));
	for (size_t e = 0; e < sh->ne; e++) {
		c->ebox[e] = edge_box3(&sh->e[e]);
	}
	c->index = xcalloc(sh->nf + 1, sizeof(*c->index));
	for (size_t f = 0; f < sh->nf; f++) {
		facemap_init(&c->fm[f], sh, f);
		index_face(&c->index[f], sh, c->ebox, f);
		c->fm[f].ebox = c->ebox;
		c->fm[f].index = &c->index[f];
		c->fbox[f] = face_box(sh, f);
	}
}

void shape_cache_free(struct shape_cache *c)
{
	for (size_t f = 0; c->index && f < c->sh->nf; f++) {
		free(c->index[f].start);
		free(c->index[f].uses);
	}
	free(c->index);
	free(c->fm);
	free(c->ebox);
	free(c->fbox);
	memset(c, 0, sizeof(*c));
}

int face_has_c(const struct shape_cache *c, size_t f, struct vec p)
{
	int k = c->quick ? c->quick(c->qctx, f, p) : -1;

	return k >= 0 ? k : face_has_m(&c->fm[f], c->ebox, p);
}

int face_on_c(const struct shape_cache *c, size_t f, struct vec p)
{
	return fabs(surf_dist(&c->sh->f[f].s, p)) < SH_EPS &&
	       face_has_c(c, f, p);
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

// The height along d of curve c at t.
static double height(const struct curve *c, struct vec d, double t)
{
	return dot(curve_at(c, t), d);
}

// The least and the greatest height of edge e on a nodal section along
// d: the samples, and between the neighbors of each one that tops its
// neighbors either way, the turning point by golden sections.
static struct span nodal_reach(const struct sedge *e, struct vec d)
{
	const struct curve *c = &e->c;
	double h = (e->t1 - e->t0) / (double)EDGE_PTS;
	struct span r = { HUGE_VAL, -HUGE_VAL };

	for (int i = 0; i <= EDGE_PTS; i++) {
		double ti = e->t0 + (h * i);
		double v = height(c, d, ti);

		r.lo = fmin(r.lo, v);
		r.hi = fmax(r.hi, v);
		for (int up = 0; up < 2 && i > 0 && i < EDGE_PTS; up++) {
			double s = up ? 1 : -1;
			double lo = ti - h;
			double hi = ti + h;

			if (s * v < s * height(c, d, lo) ||
			    s * v < s * height(c, d, hi)) {
				continue;
			}
			for (int k = 0; k < BISECT; k++) {
				double m1 = hi - (GOLD * (hi - lo));
				double m2 = lo + (GOLD * (hi - lo));

				if (s * height(c, d, m1) >
				    s * height(c, d, m2)) {
					hi = m2;
				} else {
					lo = m1;
				}
			}
			v = height(c, d, (lo + hi) / 2);
			r.lo = fmin(r.lo, v);
			r.hi = fmax(r.hi, v);
		}
	}
	return r;
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
	if (c->kind == CV_NODAL) {
		return nodal_reach(e, d);
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
// As ray_face_hits, with the cache c of sh, or none if c is NULL.
static size_t face_hits(const struct shape *sh, const struct shape_cache *c,
			size_t f, struct ray_seg rs, double *t, int *unsafe)
{
	double r[ROOTS_MAX];
	int tangent = 0;
	size_t m = line_surf(rs.l, rs.tmax, &sh->f[f].s, r, &tangent);
	size_t n = 0;

	for (size_t i = 0; i < m; i++) {
		struct vec h = add(rs.l.o, mul(rs.l.d, r[i]));

		if (c && !box_has(c->fbox[f], h)) {
			continue;
		}
		if (!(c ? face_has_c(c, f, h) : face_has(sh, f, h))) {
			continue;
		}
		if (tangent || r[i] < NEAR_EDGE ||
		    near_edges_in(sh, c ? c->ebox : NULL,
				  c ? c->fm[f].index : NULL, f, h, NEAR_EDGE)) {
			*unsafe = 1;
		}
		t[n++] = r[i];
	}
	return n;
}

size_t ray_face_hits(const struct shape_cache *c, size_t f, struct ray_seg rs,
		     double *t, int *unsafe)
{
	return face_hits(c->sh, c, f, rs, t, unsafe);
}

// The number of times the ray crosses the faces of the list fs of sh, or
// -1 if it meets one where the count is unsafe: at a touch or near an
// edge.
static int ray_count(const struct shape *sh, const struct shape_cache *c,
		     struct ray_seg rs, const struct face_list *fs)
{
	int count = 0;

	for (size_t i = 0; i < fs->n; i++) {
		double t[ROOTS_MAX];
		int unsafe = 0;
		size_t n = face_hits(sh, c, fs->f[i], rs, t, &unsafe);

		if (unsafe) {
			return -1;
		}
		count += (int)n;
	}
	return count;
}

// The faces of sh that the ray rs may cross, into fs: by rf, or all.
static void ray_list(const struct shape *sh, const struct ray_faces *rf,
		     struct ray_seg rs, struct face_list *fs)
{
	fs->n = 0;
	if (rf) {
		rf->fn(rf->ctx, rs.l, rs.tmax, fs);
		return;
	}
	for (size_t f = 0; f < sh->nf; f++) {
		PUSH(fs->f, fs->n, fs->cap, f);
	}
}

int solid_has_with(const struct shape *sh, struct vec p, struct box x,
		   const struct ray_faces *rf)
{
	struct face_list fs = { NULL, 0, 0 };
	double tmax = 0;

	if (!box_has(x, p)) {
		return 0;
	}
	tmax = len(sub(x.hi, x.lo)) * 2;
	for (int i = 0; i < RAYS; i++) {
		struct line3 ray = { p, unit(v3(ray_dirs[i][0], ray_dirs[i][1],
						ray_dirs[i][2])) };
		int c = 0;

		ray_list(sh, rf, (struct ray_seg){ ray, tmax }, &fs);
		c = ray_count(sh, rf ? rf->cache : NULL,
			      (struct ray_seg){ ray, tmax }, &fs);
		if (c >= 0) {
			free(fs.f);
			return c % 2;
		}
	}
	die("internal error: no ray from a point misses the edges of %s",
	    sh->name);
}

int solid_has(const struct shape *sh, struct vec p)
{
	struct box x = box_empty();

	for (size_t i = 0; i < sh->nv; i++) {
		box_add(&x, sh->v[i]);
	}
	for (size_t i = 0; i < sh->ne; i++) {
		struct box b = edge_box3(&sh->e[i]);

		box_add(&x, b.lo);
		box_add(&x, b.hi);
	}
	return solid_has_with(sh, p, x, NULL);
}

enum { WAYS = 13 }; // The lines that may part two faces.

// The axes, the diagonals of their planes and those of the cube, one
// way each.
static const double ways[WAYS][3] = { { 1, 0, 0 },  { 0, 1, 0 },  { 0, 0, 1 },
				      { 1, 1, 0 },  { 1, -1, 0 }, { 1, 0, 1 },
				      { 1, 0, -1 }, { 0, 1, 1 },  { 0, 1, -1 },
				      { 1, 1, 1 },  { 1, 1, -1 }, { 1, -1, 1 },
				      { 1, -1, -1 } };

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

// Whether the circles of radii ra and rb, their centers d apart, are
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

	if (c->kind == CV_NODAL) {
		// No triangle holds a piece of it: its points, closely.
		for (int i = 0; i <= NODAL_PTS; i++) {
			struct vec p = edge_at(e, i / (double)NODAL_PTS);

			if (outside(s, p) > -SH_EPS) {
				return 0;
			}
		}
		return 1;
	}

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

// Whether a line along one of the axes, the diagonals of their planes
// or those of the cube parts the faces fa of a and fb of b, both made of
// straight lines: the heights along it of the one lie all above those of
// the other.  The heights over a face made of straight lines are
// greatest and least on its edges.
static int parted(const struct shape *a, size_t fa, const struct shape *b,
		  size_t fb)
{
	if (!ruled(&a->f[fa]) || !ruled(&b->f[fb])) {
		return 0;
	}
	for (int k = 0; k < WAYS; k++) {
		struct vec d = unit(v3(ways[k][0], ways[k][1], ways[k][2]));
		struct span ra = face_reach(a, fa, v3(0, 0, 0), d);
		struct span rb = face_reach(b, fb, v3(0, 0, 0), d);

		if (ra.hi < rb.lo - SH_EPS || rb.hi < ra.lo - SH_EPS) {
			return 1;
		}
	}
	return 0;
}

int faces_clear(const struct shape *a, size_t fa, const struct shape *b,
		size_t fb)
{
	return only_touch(a, fa, b, fb) || parted(a, fa, b, fb) ||
	       plane_clear(a, fa, b, fb) || plane_clear(b, fb, a, fa) ||
	       rounds_clear(a, fa, b, fb) || plane_inside(a, fa, b, fb) ||
	       plane_inside(b, fb, a, fa);
}
