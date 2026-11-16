// SPDX-License-Identifier: GPL-3.0-only
// integ.c -- Volume, area and center of exact solids, by quadrature
// Copyright (c) 2026 Jakob Kastelic

// The divergence theorem turns the volume into a sum over the faces of
// integrals of p . n / 3, and the moments of the volume into integrals of
// x^2 n_x / 2 and so on.  Over a face, each is an integral of g(u, v) over
// the domain of its parameters, and Green's theorem turns that into the
// loop integral of G dv, with G(u, v) the integral of g from the middle
// angle u0 of the face to u.  Where the form of the face and of its
// edges makes the integrands polynomials, or sums of cos k t and sin k t
// of known degree, the rules are exact (rule_of): the inner integral on a
// plane, a cylinder, a cone, a sphere and a torus; the loop along a line
// of a plane, a cylinder or a cone, a conic of a plane, a circle round
// the axis of a cylinder or a cone, an ellipse of a cylinder through its
// axis, a parallel or a meridian of a sphere or a torus.  Along a ring of
// a pipe the integral over u is by Gauss's rule, with the bound of the
// ellipse of Bernstein (ring_sum).  Any other edge takes Gauss's rule on
// short pieces with no bound, but a face of a pipe or a sphere of three
// or four conics takes a mesh with a bound instead (mesh_sum).

#include "integ.h"
#include "geom.h"
#include "isect.h"
#include "shape.h"
#include "util.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PIECE (PI / 16) // The longest piece of an angle, in radians.
#define AXIS_TINY 1e-12 // Sine below which a circle is round an axis.
#define COARSE (PI / 8) // The first piece of a ring in shape_mass_to.
#define ROUNDING 1e-12 // The rounding of a sum, of the sizes of its terms.
#define GEO 1e-9 // Of a size, the nearest that edges prove their form.
#define FACE_REL 1e-10 // The accuracy of a ring in shape_mass.
#define DELTA 2e-3 // The most distance from a mesh to its face, in mm.
#define STEP 0.9 // Of the step of a mesh that makes DELTA, the share it takes.
#define AWAY 0.5 // Of x, the least size of an axis not near it.

enum {
	GAUSS = 8, // Points of the rule.
	LINE_PIECES = 4, // Pieces of a line.
	NQ = 5, // The integrands: volume, area, three moments.
	TRIG = 3, // The highest multiple of u in the integrands of a
	TRIG_N = 7, // cylinder or a cone, and the values that find them.
	EDGE_TRIG = 4, // The same of G dv along a conic on a plane,
	EDGE_TRIG_ELLIPSE = 6, // along an ellipse of a cylinder,
	EDGE_TRIG_MAX = 6,
	RING_MOST = 1 << 12, // The most pieces of a ring.
	MESH_MOST = 1 << 20 // The most points of a mesh.
};

static const double gx[GAUSS] = { -0.9602898564975363, -0.7966664774136267,
				  -0.5255324099163290, -0.1834346424956498,
				  0.1834346424956498,  0.5255324099163290,
				  0.7966664774136267,  0.9602898564975363 };
static const double gw[GAUSS] = { 0.1012285362903763, 0.2223810344533745,
				  0.3137066458778873, 0.3626837833783620,
				  0.3626837833783620, 0.3137066458778873,
				  0.2223810344533745, 0.1012285362903763 };

// The integrands at (u, v) of surface s: p . N / 3, |N|, and the three
// moments, N = r_u x r_v.
static void integrands(const struct surf *s, struct uv q, double *g)
{
	struct vec p = surf_at(s, q);
	struct vec y = frame_y(s->n, s->x);
	struct vec e = add(mul(s->x, cos(q.u)), mul(y, sin(q.u)));
	struct vec nn = s->n;
	double w = 1;

	switch (s->kind) {
	case SF_PLANE:
		break;
	case SF_CYL:
		nn = e;
		w = s->r;
		break;
	case SF_CONE:
		nn = sub(e, mul(s->n, s->r2));
		w = s->r + (s->r2 * q.v);
		break;
	case SF_SPHERE:
		nn = add(mul(e, cos(q.v)), mul(s->n, sin(q.v)));
		w = s->r * s->r * cos(q.v);
		break;
	case SF_PIPE: {
		// |r_u| is the speed of the ellipse, sqrt(w2), times 1 + rt
		// cos v k for its curvature k = a b / w2^(3/2).
		struct line3 c = pipe_ring(s, q.u);
		double su = sin(q.u);
		double cu = cos(q.u);
		double w2 = (s->r * s->r * su * su) + (s->r3 * s->r3 * cu * cu);

		nn = add(mul(c.d, cos(q.v)), mul(s->n, sin(q.v)));
		w = (sqrt(w2) + (s->r2 * cos(q.v) * s->r * s->r3 / w2)) * s->r2;
		break;
	}
	default:
		nn = add(mul(e, cos(q.v)), mul(s->n, sin(q.v)));
		w = (s->r + (s->r2 * cos(q.v))) * s->r2;
		break;
	}
	nn = mul(nn, w);
	g[0] = dot(p, nn) / 3;
	g[1] = len(nn);
	g[2] = p.x * p.x * nn.x / 2;
	g[3] = p.y * p.y * nn.y / 2;
	g[4] = p.z * p.z * nn.z / 2;
}

// G of a plane, whose integrands are of degree 2 in u: the two-point
// rule is exact.
static void inner_plane(const struct surf *s, double u0, struct uv q, double *G)
{
	double h = q.u - u0;

	for (int k = 0; k < NQ; k++) {
		G[k] = 0;
	}
	for (int j = 0; j < 2; j++) {
		double g[NQ];
		struct uv p = { u0 + (h * (1 + (j ? 1 : -1) / sqrt(3)) / 2),
				q.v };

		integrands(s, p, g);
		for (int k = 0; k < NQ; k++) {
			G[k] += g[k] * h / 2;
		}
	}
}

// The coefficients of the integrands of a cylinder or a cone at v, from
// u0: of cos k (u - u0) in a, of sin k (u - u0) in b.  The last are
// kept, for the points of an edge at one v, as a circle is.
struct trig {
	const struct surf *s;
	double u0, v;
	double a[NQ][TRIG + 1], b[NQ][TRIG + 1];
};

static void trig_of(const struct surf *s, double u0, double v, struct trig *c)
{
	if (c->s == s && c->u0 == u0 && c->v == v) {
		return;
	}
	c->s = s;
	c->u0 = u0;
	c->v = v;
	for (int k = 0; k < NQ; k++) {
		for (int m = 0; m <= TRIG; m++) {
			c->a[k][m] = c->b[k][m] = 0;
		}
	}
	for (int j = 0; j < TRIG_N; j++) {
		double th = 2 * PI * j / TRIG_N, g[NQ];
		struct uv p = { u0 + th, v };

		integrands(s, p, g);
		for (int m = 0; m <= TRIG; m++) {
			double cm = cos(m * th) / TRIG_N;
			double sm = sin(m * th) / TRIG_N;

			for (int k = 0; k < NQ; k++) {
				c->a[k][m] += g[k] * cm;
				c->b[k][m] += g[k] * sm;
			}
		}
	}
}

// G of a cylinder or a cone, whose integrands are sums of cos k u and
// sin k u for k up to TRIG: its coefficients from TRIG_N values round
// the turn (trig_of), and their integrals exact.
static void inner_trig(const struct surf *s, double u0, struct uv q, double *G,
		       struct trig *c)
{
	double t = q.u - u0, ct = cos(t), st = sin(t), cm = 1, sm = 0;

	trig_of(s, u0, q.v, c);
	for (int k = 0; k < NQ; k++) {
		G[k] = c->a[k][0] * t;
	}
	for (int m = 1; m <= TRIG; m++) {
		double c2 = (cm * ct) - (sm * st);

		sm = (sm * ct) + (cm * st);
		cm = c2;
		for (int k = 0; k < NQ; k++) {
			G[k] += 2 *
				((c->a[k][m] * sm) + (c->b[k][m] * (1 - cm))) /
				m;
		}
	}
}

// A sum under way over the loops of a face: the longest piece of an
// angle the rules take, the accuracy asked of a rule with a bound (a ring
// of a pipe), the sums, the sums of the sizes of their terms (for the
// bound of their rounding), the bounds of the errors of the rules that
// are not exact, and whether some rule has no bound.
struct acc {
	double piece, rel;
	double sum[NQ], mag[NQ], bound[NQ];
	int unproven;
};

// G: the integrals of g over u from u0 to q.u at q.v.  Of a plane, a
// cylinder, a cone, a sphere and a torus it is exact: their integrands
// are of degree 2 in u, or sums of cos k u and sin k u, k up to 3; of a
// pipe, a sum by Gauss's rule on pieces.
static void inner(const struct surf *s, double u0, struct uv q, double *G,
		  struct trig *c, struct acc *z)
{
	double span = q.u - u0;
	double m = ceil(fabs(span) / z->piece);
	size_t n = (size_t)m;
	double h = 0;

	if (s->kind == SF_PLANE) {
		inner_plane(s, u0, q, G);
		return;
	}
	if (s->kind != SF_PIPE) {
		inner_trig(s, u0, q, G, c);
		return;
	}
	for (int k = 0; k < NQ; k++) {
		G[k] = 0;
	}
	if (!n) {
		return;
	}
	h = span / (double)n;
	for (size_t i = 0; i < n; i++) {
		double a = u0 + (h * (double)i);

		for (int j = 0; j < GAUSS; j++) {
			double g[NQ];
			struct uv p = { a + (h * (1 + gx[j]) / 2), q.v };

			integrands(s, p, g);
			for (int k = 0; k < NQ; k++) {
				G[k] += g[k] * gw[j] * h / 2;
			}
		}
	}
}

// The number of pieces of the edge from t0 to t1 of curve c.
static size_t pieces(const struct curve *c, double t0, double t1, double piece)
{
	double n = c->kind == CV_LINE ? (double)LINE_PIECES :
					ceil(fabs(t1 - t0) / piece);

	return n < 1 ? 1 : (size_t)n;
}

// Adds term x of integrand k.
static void add_term(struct acc *z, int k, double x)
{
	z->sum[k] += x;
	z->mag[k] += fabs(x);
}

// The integrals from w0 to w0 + th of the NQ sums of cos k w and sin k w,
// k up to deg, whose values at w0 + 2 pi j / (2 deg + 1) are f[j].
static void trig_integral(double (*f)[NQ], int deg, double th, double *out)
{
	int n = (2 * deg) + 1;
	double ct = cos(th), st = sin(th), cm = 1, sm = 0;

	for (int k = 0; k < NQ; k++) {
		out[k] = 0;
		for (int j = 0; j < n; j++) {
			out[k] += f[j][k] / n * th;
		}
	}
	for (int m = 1; m <= deg; m++) {
		double c2 = (cm * ct) - (sm * st);

		sm = (sm * ct) + (cm * st);
		cm = c2;
		for (int j = 0; j < n; j++) {
			double w = 2 * PI * j / n;
			double a = cos(m * w) / n, b = sin(m * w) / n;

			for (int k = 0; k < NQ; k++) {
				out[k] += 2 * f[j][k] *
					  ((a * sm) + (b * (1 - cm))) / m;
			}
		}
	}
}

// The integrals from w0 to w0 + th of w - w0 times the NQ sums of
// cos k w and sin k w, k up to deg, whose values are f[j] as in
// trig_integral.
static void trig_moment(double (*f)[NQ], int deg, double th, double *out)
{
	int n = (2 * deg) + 1;

	for (int k = 0; k < NQ; k++) {
		out[k] = 0;
		for (int j = 0; j < n; j++) {
			out[k] += f[j][k] / n * th * th / 2;
		}
	}
	for (int m = 1; m <= deg; m++) {
		double cm = cos(m * th), sm = sin(m * th);
		double ic = ((cm - 1) / (m * m)) + (th * sm / m);
		double is = (sm / (m * m)) - (th * cm / m);

		for (int j = 0; j < n; j++) {
			double w = 2 * PI * j / n;
			double a = cos(m * w) / n, b = sin(m * w) / n;

			for (int k = 0; k < NQ; k++) {
				out[k] += 2 * f[j][k] * ((a * ic) + (b * is));
			}
		}
	}
}

// Adds the loop integral of G dv along an ellipse of a cylinder from a
// to b, in a plane through the axis: u - u0 runs as sign t, and v as a
// sum of cos t and sin t, so G = a0(v) (u - u0) + P with P a sum of cos
// k t and sin k t: the integral of P dv, k up to EDGE_TRIG_ELLIPSE, and
// of a0 dv times t, k up to 3, both exact (trig_integral, trig_moment).
static void ellipse_sum(const struct facemap *m, const struct sedge *e,
			double a, double b, int fwd, int sign, struct acc *z)
{
	const struct surf *s = &m->sh->f[m->f].s;
	double ft[2 * EDGE_TRIG_MAX + 1][NQ], fs[2 * EDGE_TRIG_MAX + 1][NQ];
	double lo = fmin(a, b), th = fabs(b - a), o1[NQ], o2[NQ], o3[NQ];
	double u_lo = fm_uv(m, curve_at(&e->c, lo)).u - m->ref.u;
	int n = (2 * EDGE_TRIG_ELLIPSE) + 1;
	struct trig c;

	c.s = NULL;
	for (int j = 0; j < n; j++) {
		double t = lo + (2 * PI * j / n), gu[NQ];
		struct vec p = curve_at(&e->c, t);
		struct vec d = mul(curve_tan(&e->c, t), fwd ? 1 : -1);
		double dv = surf_dv(s, (struct line3){ p, d });
		struct uv q = fm_uv(m, p);

		inner(s, m->ref.u, q, gu, &c, z);
		for (int k = 0; k < NQ; k++) {
			double a0 = c.a[k][0] * dv;

			fs[j][k] = a0;
			ft[j][k] = (gu[k] * dv) - (a0 * (q.u - m->ref.u));
			z->mag[k] += fabs(gu[k] * dv) * 2 * PI / n;
		}
	}
	trig_integral(ft, EDGE_TRIG_ELLIPSE, th, o1);
	trig_integral(fs, EDGE_TRIG_ELLIPSE, th, o2);
	trig_moment(fs, EDGE_TRIG_ELLIPSE, th, o3);
	for (int k = 0; k < NQ; k++) {
		z->sum[k] += o1[k] + (u_lo * o2[k]) + (sign * o3[k]);
	}
}

// Adds the loop integral of G dv along an edge from a to b where G dv
// is a sum of cos k t and sin k t for k up to deg: exact from its values
// at 2 deg + 1 points round the curve.
static void trig_sum(const struct facemap *m, const struct sedge *e, double a,
		     double b, int fwd, int deg, struct acc *z)
{
	const struct surf *s = &m->sh->f[m->f].s;
	double f[2 * EDGE_TRIG_MAX + 1][NQ], out[NQ];
	double lo = fmin(a, b);
	int n = (2 * deg) + 1;
	struct trig c;

	c.s = NULL;
	for (int j = 0; j < n; j++) {
		double t = lo + (2 * PI * j / n), gu[NQ];
		struct vec p = curve_at(&e->c, t);
		struct vec d = mul(curve_tan(&e->c, t), fwd ? 1 : -1);
		double dv = surf_dv(s, (struct line3){ p, d });

		inner(s, m->ref.u, fm_uv(m, p), gu, &c, z);
		for (int k = 0; k < NQ; k++) {
			f[j][k] = gu[k] * dv;
			z->mag[k] += fabs(f[j][k]) * 2 * PI / n;
		}
	}
	trig_integral(f, deg, fabs(b - a), out);
	for (int k = 0; k < NQ; k++) {
		z->sum[k] += out[k];
	}
}

// The rules of an edge on a face.
enum { R_GAUSS, R_POLY, R_TRIG, R_ELLIPSE, R_ZERO, R_RING, R_MERIDIAN };

// Adds the loop integral of G dv along a meridian of a sphere or a
// torus, from a to b: at the u of its middle, v running with t, so G dv
// is a sum of cos k t and sin k t, k up to EDGE_TRIG, in v; its values
// are taken at (u, v(t)) round the turn, past the poles as v goes on.
static void meridian_sum(const struct facemap *m, const struct sedge *e,
			 double a, double b, int fwd, struct acc *z)
{
	const struct surf *s = &m->sh->f[m->f].s;
	double mid = (a + b) / 2, lo = fmin(a, b), out[NQ];
	double f[2 * EDGE_TRIG_MAX + 1][NQ];
	struct vec p = curve_at(&e->c, mid);
	struct uv q = fm_uv(m, p);
	double dv = surf_dv(s, (struct line3){ p, curve_tan(&e->c, mid) });
	double sg = (dv > 0 ? 1 : -1) * (fwd ? 1 : -1);
	int n = (2 * EDGE_TRIG) + 1;
	struct trig c;

	c.s = NULL;
	if (fabs(fabs(dv) - 1) > GEO) {
		z->unproven = 1;
		return;
	}
	for (int j = 0; j < n; j++) {
		double t = lo + (2 * PI * j / n), gu[NQ];
		struct uv r = { q.u, q.v + ((dv > 0 ? 1 : -1) * (t - mid)) };

		inner(s, m->ref.u, r, gu, &c, z);
		for (int k = 0; k < NQ; k++) {
			f[j][k] = gu[k] * sg;
			z->mag[k] += fabs(f[j][k]) * 2 * PI / n;
		}
	}
	trig_integral(f, EDGE_TRIG, fabs(b - a), out);
	for (int k = 0; k < NQ; k++) {
		z->sum[k] += out[k];
	}
}

// The size of surface s, for the tests of its edges.
static double near_of(const struct surf *s)
{
	return GEO * (1 + fabs(s->r) + fabs(s->r2) + fabs(s->r3));
}

static int parallel(struct vec a, struct vec b)
{
	return len(cross(unit(a), unit(b))) < GEO;
}

static int square(struct vec a, struct vec b)
{
	return fabs(dot(unit(a), unit(b))) < GEO;
}

// Whether point p lies on the axis of surface s.
static int on_axis(const struct surf *s, struct vec p)
{
	return len(cross(sub(p, s->o), s->n)) < near_of(s);
}

// Whether the arc of circle c from t0 to t1 keeps to one side of the
// line through its center along a: its angles of a and -a not inside.
static int one_side(const struct curve *c, double t0, double t1, struct vec a)
{
	struct vec y = frame_y(c->n, c->x);
	double t = atan2(dot(a, y), dot(a, c->x));

	for (int k = 0; k < 2; k++, t += PI) {
		double w = t0 + fmod(fmod(t - t0, 2 * PI) + (2 * PI), 2 * PI);

		if (w > t0 + 1e-12 && w < t1 - 1e-12) {
			return 0;
		}
	}
	return 1;
}

// Whether curve c is a ring of pipe s: a circle of its tube at one u.
static int is_ring(const struct surf *s, const struct curve *c)
{
	double near = near_of(s);
	struct line3 r;

	if (c->kind != CV_CIRCLE || fabs(c->a - s->r2) >= near) {
		return 0;
	}
	r = pipe_ring(s, pipe_foot(s, c->o));
	return len(sub(r.o, c->o)) < near && square(c->n, s->n) &&
	       square(c->n, r.d);
}

// The rule of edge e on surface s, and in *deg the degree of a sum of
// cos k t and sin k t: proven exact by the form of the edge on the
// surface, or R_RING for a ring of a pipe, of the bounded rule, or
// R_GAUSS, of no bound.
static int rule_of(const struct surf *s, const struct sedge *e, int *deg)
{
	const struct curve *c = &e->c;
	double near = near_of(s);
	int ruled = s->kind == SF_CYL || s->kind == SF_CONE;

	*deg = EDGE_TRIG;
	if (c->kind == CV_LINE && (s->kind == SF_PLANE || ruled)) {
		return R_POLY;
	}
	if (s->kind == SF_PLANE &&
	    (c->kind == CV_CIRCLE || c->kind == CV_ELLIPSE)) {
		return R_TRIG;
	}
	if (ruled && c->kind == CV_CIRCLE &&
	    len(cross(c->n, s->n)) < AXIS_TINY) {
		return R_TRIG;
	}
	if (s->kind == SF_CYL && c->kind == CV_ELLIPSE && on_axis(s, c->o)) {
		// A plane through the axis's point: seen along the axis, the
		// ellipse is the circle of the cylinder at angle t.
		struct vec y = frame_y(c->n, c->x);
		struct vec px =
			mul(sub(c->x, mul(s->n, dot(c->x, s->n))), c->a);
		struct vec py = mul(sub(y, mul(s->n, dot(y, s->n))), c->b);

		if (fabs(len(px) - s->r) < near &&
		    fabs(len(py) - s->r) < near &&
		    fabs(dot(px, py)) < near * s->r) {
			// u turns with t, or against it: deg says which.
			*deg = dot(cross(px, py), s->n) > 0 ? 1 : -1;
			return R_ELLIPSE;
		}
	}
	if ((s->kind == SF_SPHERE || s->kind == SF_TORUS) &&
	    c->kind == CV_CIRCLE) {
		struct vec d = sub(c->o, s->o);
		double rc = s->kind == SF_SPHERE ? s->r : s->r2;

		if (parallel(c->n, s->n) && on_axis(s, c->o)) {
			return R_ZERO; // At one v: no dv.
		}
		// A meridian: in a plane of the axis, at one u.
		if (square(c->n, s->n) && fabs(c->a - rc) < near &&
		    (s->kind == SF_SPHERE ?
			     len(d) < near && one_side(c, e->t0, e->t1, s->n) :
			     s->r > s->r2 && fabs(dot(d, s->n)) < near &&
				     fabs(len(d) - s->r) < near &&
				     square(c->n, d))) {
			return R_MERIDIAN;
		}
	}
	if (s->kind == SF_PIPE && c->kind == CV_ELLIPSE &&
	    parallel(c->n, s->n) &&
	    fabs(fabs(dot(sub(c->o, s->o), s->n)) - s->r2) < near &&
	    on_axis(s, c->o)) {
		return R_ZERO; // A rail at v of a quarter turn: no dv.
	}
	if (s->kind == SF_PIPE && s->r2 < pipe_bend(s) && is_ring(s, c)) {
		return R_RING;
	}
	return R_GAUSS;
}

// The bounds, at u + i y for any u, of the integrands of pipe s, with
// v real: of their sizes, the volume, area and moment ones, or 0 if the
// ellipse's speed may be 0 there.
static int pipe_bounds(const struct surf *s, double y, double *b)
{
	double ch = cosh(y), ex = exp(2 * y) - 1;
	double lo = fmin(s->r, s->r3), hi = fmax(s->r, s->r3);
	double half = fabs((s->r3 * s->r3) - (s->r * s->r)) / 2;
	double m2 = (lo * lo) - (half * ex);
	double M2 = (hi * hi) + (half * ex);
	double d = 0, p = 0, w = 0, nn = 0;

	if (!(m2 > 0)) {
		return 0;
	}
	d = (s->r + s->r3) * ch / sqrt(m2);
	p = len(s->o) + ((s->r + s->r3) * ch) + (s->r2 * (d + 1));
	w = s->r2 * (sqrt(M2) + (s->r2 * s->r * s->r3 / m2));
	nn = (d + 1) * w;
	b[0] = p * nn / 3;
	b[1] = w;
	b[2] = b[3] = b[4] = p * p * nn / 2;
	return 1;
}

// The height off the real line, of the ellipses of Bernstein of the
// pieces of a ring of pipe s, where the speed of its ellipse keeps half
// its least: |w2(u + i y) - w2(u)| <= |B| (e^{2y} - 1), B half the
// difference of the squares of its radii.
static double ring_height(const struct surf *s)
{
	double lo = fmin(s->r, s->r3);
	double half = fabs((s->r3 * s->r3) - (s->r * s->r)) / 2;

	return half > 0 ? fmin(1, log(1 + (lo * lo / (2 * half))) / 2) : 1;
}

// Adds the loop integral of G dv along a ring of a pipe, at u = u1, from
// a to b: the integral over u from u0 to u1 of H(u), the integral of g
// along the ring, exact in v (its integrands are sums of cos k v and
// sin k v, k up to EDGE_TRIG); over u by Gauss's rule on pieces, each
// with its bound by the ellipse of Bernstein of the piece (RHO), where
// H is analytic: pieces halved until the bound is within the accuracy.
static void ring_sum(const struct facemap *m, const struct sedge *e, double a,
		     double b, struct acc *z)
{
	const struct surf *s = &m->sh->f[m->f].s;
	struct vec p = curve_at(&e->c, a);
	struct uv q = fm_uv(m, p);
	double dv = surf_dv(s, (struct line3){ p, curve_tan(&e->c, a) });
	double u0 = m->ref.u, u1 = q.u, th = dv > 0 ? b - a : a - b;
	double sum[NQ], mag[NQ], bound[NQ];

	double y = ring_height(s), mb[NQ];

	if (fabs(fabs(dv) - 1) > GEO || !pipe_bounds(s, y, mb)) {
		z->unproven = 1;
		return;
	}
	for (size_t n = (size_t)fmax(1, ceil(fabs(u1 - u0) / z->piece));;
	     n *= 2) {
		double h = (u1 - u0) / (double)n, hh = fabs(h) / 2;
		double rho = (y + sqrt((y * y) + (hh * hh))) / hh;
		double f = 64 / (15 * ((rho * rho) - 1) * pow(rho, 2 * GAUSS));
		int ok = 1;

		for (int k = 0; k < NQ; k++) {
			sum[k] = mag[k] = 0;
			bound[k] = (double)n * f * fabs(th) * mb[k] * hh;
		}
		for (size_t i = 0; i < n; i++) {
			for (int j = 0; j < GAUSS; j++) {
				double u = u0 + (h * ((double)i +
						      ((1 + gx[j]) / 2)));
				double vals[2 * EDGE_TRIG + 1][NQ], hv[NQ];

				for (int t = 0; t <= 2 * EDGE_TRIG; t++) {
					struct uv r = {
						u, q.v + (2 * PI * t /
							  (2 * EDGE_TRIG + 1))
					};

					integrands(s, r, vals[t]);
				}
				trig_integral(vals, EDGE_TRIG, th, hv);
				for (int k = 0; k < NQ; k++) {
					double x = hv[k] * gw[j] * h / 2;

					sum[k] += x;
					mag[k] += fabs(x);
				}
			}
		}
		ok = ok && n < RING_MOST;
		for (int k = 0; ok && k < NQ; k++) {
			ok = bound[k] <= z->rel * mag[k];
		}
		if (ok || n >= RING_MOST) {
			break;
		}
	}
	for (int k = 0; k < NQ; k++) {
		z->sum[k] += sum[k];
		z->mag[k] += mag[k];
		z->bound[k] += bound[k];
	}
}

// Adds the loop integral of G dv along use u of edge e of the face of
// m, the angles kept within half a turn of the middle of the face, G
// from there, by the rule of the edge (rule_of).
static void use_sum(const struct facemap *m, const struct sedge *e,
		    const struct suse *u, struct acc *z)
{
	static const double g2x[2] = { -0.5773502691896258,
				       0.5773502691896258 };
	static const double g2w[2] = { 1, 1 };
	const struct surf *s = &m->sh->f[m->f].s;
	double a = u->fwd ? e->t0 : e->t1;
	double b = u->fwd ? e->t1 : e->t0;
	size_t n = pieces(&e->c, e->t0, e->t1, z->piece);
	const double *rx = gx, *rw = gw;
	int np = GAUSS, deg = 0, r = rule_of(s, e, &deg);
	double h = 0;
	struct trig c;

	c.s = NULL;
	if (r == R_ZERO) {
		return;
	}
	if (r == R_TRIG) {
		trig_sum(m, e, a, b, u->fwd, deg, z);
		return;
	}
	if (r == R_MERIDIAN) {
		meridian_sum(m, e, a, b, u->fwd, z);
		return;
	}
	if (r == R_ELLIPSE) {
		ellipse_sum(m, e, a, b, u->fwd, deg, z);
		return;
	}
	if (r == R_RING) {
		ring_sum(m, e, a, b, z);
		return;
	}
	if (r == R_POLY) {
		// G along the line is of degree 3 at most, and dv constant.
		n = 1;
		rx = g2x;
		rw = g2w;
		np = 2;
	} else {
		z->unproven = 1;
	}
	h = (b - a) / (double)n;
	for (size_t i = 0; i < n; i++) {
		for (int j = 0; j < np; j++) {
			double t = a + (h * ((double)i + ((1 + rx[j]) / 2)));
			struct vec p = curve_at(&e->c, t);
			struct vec d =
				mul(curve_tan(&e->c, t), u->fwd ? 1 : -1);
			struct uv q = fm_uv(m, p);
			double gu[NQ];
			double dv = surf_dv(s, (struct line3){ p, d });

			inner(s, m->ref.u, q, gu, &c, z);
			for (int k = 0; k < NQ; k++) {
				add_term(z, k,
					 gu[k] * dv * rw[j] * fabs(h) / 2);
			}
		}
	}
}

// The sums of the face of m, with pieces of angle no longer than piece,
// a ring of a pipe to rel.
static struct acc face_sum(const struct facemap *m, double piece, double rel)
{
	const struct shape *sh = m->sh;
	const struct sface *fc = &sh->f[m->f];
	struct acc z;

	memset(&z, 0, sizeof(z));
	z.piece = piece;
	z.rel = rel;
	for (size_t l = 0; l < fc->nloops; l++) {
		const struct sloop *lp = &sh->l[fc->loop0 + l];

		for (size_t k = 0; k < lp->n; k++) {
			const struct suse *u = &sh->u[lp->first + k];

			use_sum(m, &sh->e[u->e], u, &z);
		}
	}
	return z;
}

// The share of face f in a mass, from the sums q.
static struct solid_mass mass_of(const struct shape *sh, size_t f,
				 const double *q)
{
	struct solid_mass m;

	m.volume = q[0];
	m.area = sh->f[f].sense * q[1];
	m.moment = v3(q[2], q[3], q[4]);
	return m;
}

struct solid_mass face_mass(const struct shape *sh, size_t f)
{
	struct facemap m;
	struct acc z;

	facemap_init(&m, sh, f);
	z = face_sum(&m, PIECE, FACE_REL);
	return mass_of(sh, f, z.sum);
}

struct solid_mass shape_mass(const struct shape *sh)
{
	struct solid_mass m = { 0, 0, { 0, 0, 0 } };

	for (size_t i = 0; i < sh->nf; i++) {
		struct solid_mass f = face_mass(sh, i);

		m.volume += f.volume;
		m.area += f.area;
		m.moment = add(m.moment, f.moment);
	}
	return m;
}

// The sums of sphere face f of sh in the frame of the axis of one of its
// circles, the first in which every edge is a parallel or a meridian;
// else z, as they were.  The integrals do not depend on the frame.
static struct acc sphere_sum(const struct shape *sh, size_t f, double rel,
			     struct acc z)
{
	const struct sface *fc = &sh->f[f];
	struct shape one = *sh;
	struct sface *faces = xmalloc(sh->nf * sizeof(*faces));

	memcpy(faces, sh->f, sh->nf * sizeof(*faces));
	one.f = faces;
	for (size_t l = 0; l < fc->nloops && z.unproven; l++) {
		const struct sloop *lp = &sh->l[fc->loop0 + l];

		for (size_t k = 0; k < lp->n && z.unproven; k++) {
			const struct curve *c =
				&sh->e[sh->u[lp->first + k].e].c;
			struct facemap fm;
			struct acc y;

			if (c->kind != CV_CIRCLE) {
				continue;
			}
			faces[f].s.n = unit(c->n);
			faces[f].s.x = unit(c->x);
			facemap_init(&fm, &one, f);
			y = face_sum(&fm, COARSE, rel);
			if (!y.unproven) {
				z = y;
			}
		}
	}
	free(faces);
	return z;
}

// The integral of the outward normal over face f of sh, half that of
// p x dp round its loop: o x x dt along a line, o x dq + a b x x y dt
// along a conic q = a cos t x + b sin t y about o; 0 on other edges.
static int face_normal(const struct shape *sh, size_t f, struct vec *nf)
{
	struct urange r = face_uses(sh, f);

	*nf = v3(0, 0, 0);
	for (size_t i = r.first; i < r.end; i++) {
		const struct sedge *e = &sh->e[sh->u[i].e];
		const struct curve *c = &e->c;
		double ta = sh->u[i].fwd ? e->t0 : e->t1;
		double tb = sh->u[i].fwd ? e->t1 : e->t0;
		double ab = c->a * (c->kind == CV_CIRCLE ? c->a : c->b);
		struct vec d;

		if (c->kind == CV_LINE) {
			d = mul(cross(c->o, c->x), tb - ta);
		} else if (c->kind == CV_CIRCLE || c->kind == CV_ELLIPSE) {
			d = add(cross(c->o,
				      sub(curve_at(c, tb), curve_at(c, ta))),
				mul(cross(c->x, frame_y(c->n, c->x)),
				    ab * (tb - ta)));
		} else {
			return 0;
		}
		*nf = add(*nf, mul(d, 1.0 / 2));
	}
	return 1;
}

// A face of a pipe or a sphere of one loop of three or four lines or
// conics, where its edges have no rule with a bound, takes a mesh
// (mesh_sum): the bilinear blend in (u, v) of its sides (of three, the
// first corner for a fourth), its points on the face, cut into flat
// triangles, whose sums about the center o of the surface are exact.  By
// Taylor a triangle lies within M l^2 / 6 of the face, l its longest
// side in (u, v) and M the most second derivative of S along a unit
// (du, dv); the step makes that, with the tolerance of the edges and the
// rounding, at most DELTA.  The nearest point on the
// face, of curvature at most K, maps the mesh onto it, scaling area by
// (1 - K DELTA)^-2 at most and by cos t (1 + K DELTA)^-2 at least, t the
// angle of the normals: the face has area A <= A_L (1 - K DELTA)^-2 + W,
// W the area between the chords of its edges and their arcs, l^3 k / 4
// at most for a curvature k, and the space between it and the mesh a
// volume B <= A DELTA (1 + K DELTA)^2.  The volume and the moments are
// fluxes of p / 3 and x^2 / 2 along x, about o: of the constant part
// exact from the edges (face_normal), of the rest f within max |div f| B
// + max |f| W.  The area errs by A_L ((1 - K DELTA)^-2 - 1) + W + sum A_L
// t^2 / 2, t at most pi / 2 M (|d1|^2 |e2| + |d2|^2 |e1|) / (2 |e1 x
// e2|), d and e the sides at a corner in (u, v) and in space, plus N max
// |d| + 4 K DELTA, N the most turn of the normal per unit (du, dv).  Of
// radii a, b and rt, lo and hi the least and the greatest of a and b,
// the ellipse c(u) has |c''| <= hi, and its normal turns at a rate q <=
// k = a b / lo^2 that changes at most at e = k |a^2 - b^2| / lo^2; a
// sphere has a = b = 0, k = 1 and rt = r.  So M = max(hi + rt (e + k^2),
// rt) + rt k, N = sqrt(k^2 + 1), and K = max(1 / rt, k c / (lo - rt k
// c)), c the most -cos v of the face.
struct mpt {
	double q[2], p[3]; // (u, v), and the place less o.
};

// A mesh under way: M, N, K and the radius about o of a ball that holds
// the face (bounds); its surface, the sense of the face, o, the sums,
// those of A_L t and A_L t^2 / 2, the longest side squared, W, and the
// most -cos v.
struct mesh {
	double b[4], sense, o[3], t1, t2, l2, w, cn;
	double f[3][3], ra, rb, rt; // Axes x, y and n, and radii a, b and rt.
	struct surf s;
	struct acc z;
};

// The bounds of the mesh x for a most -cos v of cn; 0 if the pipe folds
// there.
static int bounds(struct mesh *x, double cn)
{
	const struct surf *s = &x->s;
	int pipe = s->kind == SF_PIPE;
	double a = pipe ? s->r : 0, c = pipe ? s->r3 : 0;
	double rt = pipe ? s->r2 : s->r, lo = fmin(a, c), hi = fmax(a, c);
	double d = fabs((a * a) - (c * c)), k = d > 0 ? a * c / (lo * lo) : 1;
	double e = d > 0 ? k * d / (lo * lo) : 0, f = rt * k * cn;

	x->b[0] = fmax(hi + (rt * (e + (k * k))), rt) + (rt * k);
	x->b[1] = hypot(k, 1);
	x->b[2] = pipe ? fmax(1 / rt, k * cn / (lo - f)) : 1 / rt;
	x->b[3] = hi + rt;
	return (!pipe || lo > f) && x->b[2] * DELTA < 1;
}

// Point j of n of side i of the loop of the face of m, on its edge, or
// of side 3 of a loop of three, its first corner; the most curvature of
// the edge into *kc.
static struct mpt side_at(const struct facemap *m, const struct mesh *x,
			  size_t i, size_t j, size_t n, double *kc)
{
	const struct sloop *lp = &m->sh->l[m->sh->f[m->f].loop0];
	const struct suse *u = &m->sh->u[lp->first + (i < lp->n ? i : 0)];
	const struct sedge *e = &m->sh->e[u->e];
	const struct curve *c = &e->c;
	double a = u->fwd ? e->t0 : e->t1, b = u->fwd ? e->t1 : e->t0;
	double rb = c->kind == CV_ELLIPSE ? c->b : c->a, lo = fmin(c->a, rb);
	struct vec p = curve_at(
		c, i < lp->n ? a + ((b - a) * (double)j / (double)n) : a);
	struct vec d = sub(p, x->s.o);
	struct uv q =
		x->s.kind == SF_PIPE ?
			fm_uv(m, p) :
			(struct uv){
				atan2(dot(d, frame_y(x->s.n, x->s.x)),
				      dot(d, x->s.x)),
				asin(fmax(-1, fmin(1, dot(d, x->s.n) / x->s.r)))
			};

	*kc = c->kind == CV_LINE ? 0 : fmax(c->a, rb) / (lo * lo);
	return (struct mpt){ { q.u, q.v }, { d.x, d.y, d.z } };
}

static double dist2(const struct mpt *a, const struct mpt *b)
{
	double du = b->q[0] - a->q[0], dv = b->q[1] - a->q[1];

	return (du * du) + (dv * dv);
}

static double dist3(const double *a, const double *b)
{
	double dx = b[0] - a[0], dy = b[1] - a[1], dz = b[2] - a[2];

	return (dx * dx) + (dy * dy) + (dz * dz);
}

// The place of point p of the mesh x, less o, from its (u, v): c(u) + rt
// (cos v w(u) + sin v n).
static void point(const struct mesh *x, struct mpt *p)
{
	double cu = cos(p->q[0]), su = sin(p->q[0]);
	double cv = cos(p->q[1]) * x->rt, sv = sin(p->q[1]) * x->rt;
	double wx = x->ra == x->rb ? cu : x->rb * cu;
	double wy = x->ra == x->rb ? su : x->ra * su, wl = hypot(wx, wy);

	for (int i = 0; i < 3; i++) {
		p->p[i] = (((x->ra * cu) + (cv * wx / wl)) * x->f[0][i]) +
			  (((x->rb * su) + (cv * wy / wl)) * x->f[1][i]) +
			  (sv * x->f[2][i]);
	}
}

// The points of the mesh of the face of m into g, n + 1 along sides 0
// and 2 and k + 1 along sides 1 and 3, row j at g + j (n + 1); its W
// and -cos v into x.
static void grid(const struct facemap *m, size_t n, size_t k, struct mpt *g,
		 struct mesh *x)
{
	size_t w = n + 1;

	for (size_t i = 0; i < 4; i++) {
		const struct mpt *was = NULL;
		double kc = 0;

		for (size_t j = 0; j <= (i % 2 ? k : n); j++) {
			size_t at[4] = { j, (j * w) + n, (k * w) + n - j,
					 (k - j) * w };
			struct mpt *p = &g[at[i]];
			double l = 0;

			*p = side_at(m, x, i, j, i % 2 ? k : n, &kc);
			l = was ? sqrt(dist3(p->p, was->p)) : 0;
			x->w += kc * l > 1 ? HUGE_VAL : l * l * l * kc / 4;
			x->cn = fmax(x->cn, -cos(p->q[1]));
			was = p;
		}
	}
	for (size_t j = 1; j < k; j++) {
		for (size_t i = 1; i < n; i++) {
			double a = (double)i / (double)n,
			       b = (double)j / (double)k;
			const struct mpt *c = g + (k * w), *l = g + (j * w);
			struct mpt *p = &g[(j * w) + i];

			for (int t = 0; t < 2; t++) {
				p->q[t] = ((1 - b) * g[i].q[t]) +
					  (b * c[i].q[t]) +
					  ((1 - a) * l[0].q[t]) +
					  (a * l[n].q[t]) -
					  ((1 - a) * (1 - b) * g[0].q[t]) -
					  (a * (1 - b) * g[n].q[t]) -
					  ((1 - a) * b * c[0].q[t]) -
					  (a * b * c[n].q[t]);
			}
			point(x, p);
			x->cn = fmax(x->cn, -cos(p->q[1]));
		}
	}
}

// Adds the moment along axis k of a flat triangle whose normal, twice
// its area, has n along k, and whose points less o are p0, p1 and p2
// along k: about o, of p^2 n / 2 and of o p n.
static void moment(struct mesh *x, int k, double n, double p0, double p1,
		   double p2)
{
	add_term(&x->z, 2 + k,
		 n *
			 ((p0 * p0) + (p1 * p1) + (p2 * p2) + (p0 * p1) +
			  (p1 * p2) + (p2 * p0)) /
			 24);
	add_term(&x->z, 2 + k, x->o[k] * n * (p0 + p1 + p2) / 6);
}

// Adds the triangle a, b, c of a mesh, t at its corner a (mesh_sum).
static void tri(struct mesh *x, const struct mpt *a, const struct mpt *b,
		const struct mpt *c)
{
	const double *p0 = a->p, *p1 = b->p, *p2 = c->p;
	double ax = p1[0] - p0[0], ay = p1[1] - p0[1], az = p1[2] - p0[2];
	double bx = p2[0] - p0[0], by = p2[1] - p0[1], bz = p2[2] - p0[2];
	double det = ((b->q[0] - a->q[0]) * (c->q[1] - a->q[1])) -
		     ((b->q[1] - a->q[1]) * (c->q[0] - a->q[0]));
	double sg = (det > 0 ? 1 : -1) * x->sense;
	double nx = sg * ((ay * bz) - (az * by));
	double ny = sg * ((az * bx) - (ax * bz));
	double nz = sg * ((ax * by) - (ay * bx));
	double w = sqrt((nx * nx) + (ny * ny) + (nz * nz));
	double d1 = dist2(a, b), d2 = dist2(a, c), sn = 0, t = 0;

	x->l2 = fmax(x->l2, fmax(fmax(d1, d2), dist2(b, c)));
	if (!(w > 0)) {
		return;
	}
	add_term(&x->z, 0,
		 (((p0[0] + p1[0] + p2[0]) * nx) +
		  ((p0[1] + p1[1] + p2[1]) * ny) +
		  ((p0[2] + p1[2] + p2[2]) * nz)) /
			 18);
	add_term(&x->z, 1, x->sense * w / 2);
	moment(x, 0, nx, p0[0], p1[0], p2[0]);
	moment(x, 1, ny, p0[1], p1[1], p2[1]);
	moment(x, 2, nz, p0[2], p1[2], p2[2]);
	sn = x->b[0] *
	     ((d1 * sqrt(dist3(p2, p0))) + (d2 * sqrt(dist3(p1, p0)))) /
	     (2 * w);
	t = (sn < 1 ? PI / 2 * sn : PI) + (x->b[1] * sqrt(fmax(d1, d2)));
	x->t1 += w / 2 * t;
	x->t2 += w / 2 * t * t / 2;
}

// The sums of the face of m from a mesh, and their bounds, into *z; 0
// and z as it was if the face is not of three or four lines or conics,
// or a bound fails.
static int mesh_sum(const struct facemap *m, struct acc *z)
{
	const struct sface *fc = &m->sh->f[m->f];
	struct urange r = face_uses(m->sh, m->f);
	size_t ne = r.end - r.first, n = 0, k = 0, w = 0;
	struct mpt c[4], *g = NULL;
	struct mesh x;
	struct vec nf, y = v3(0, 0, 0);
	double h = 0, kd = 0, area = 0, vb = 0, ek = 0, rr = 0, kc = 0;

	memset(&x, 0, sizeof(x));
	x.s = fc->s;
	if (fc->nloops != 1 || ne < 3 || ne > 4 || !bounds(&x, 0) ||
	    !face_normal(m->sh, m->f, &nf)) {
		return 0;
	}
	// A sphere takes the frame whose axis x points to the middle of the
	// corners, far from its poles.
	for (size_t i = r.first; i < r.end; i++) {
		const struct sedge *e = &m->sh->e[m->sh->u[i].e];

		y = add(y,
			sub(m->sh->v[m->sh->u[i].fwd ? e->v0 : e->v1], x.s.o));
	}
	if (x.s.kind == SF_SPHERE) {
		x.s.x = unit(y);
		x.s.n = unit(cross(x.s.x, fabs(x.s.x.x) < AWAY ? v3(1, 0, 0) :
								 v3(0, 1, 0)));
	}
	x.sense = fc->sense;
	x.ra = x.s.kind == SF_PIPE ? x.s.r : 0;
	x.rb = x.s.kind == SF_PIPE ? x.s.r3 : 0;
	x.rt = x.s.kind == SF_PIPE ? x.s.r2 : x.s.r;
	for (int i = 0; i < 3; i++) {
		x.o[i] = comp(x.s.o, i);
		x.f[0][i] = comp(x.s.x, i);
		x.f[1][i] = comp(frame_y(x.s.n, x.s.x), i);
		x.f[2][i] = comp(x.s.n, i);
	}
	for (size_t i = 0; i < 4; i++) {
		c[i] = side_at(m, &x, i, 0, 1, &kc);
	}
	h = STEP * sqrt(3 * DELTA / x.b[0]);
	n = (size_t)ceil(sqrt(fmax(dist2(c, c + 1), dist2(c + 2, c + 3))) / h);
	k = (size_t)ceil(sqrt(fmax(dist2(c + 1, c + 2), dist2(c + 3, c))) / h);
	for (n += !n, k += !k;; n += (n / 4) + 1, k += (k / 4) + 1) {
		w = n + 1;
		if (w * (k + 1) > MESH_MOST) {
			free(g);
			return 0;
		}
		g = xrealloc(g, w * (k + 1) * sizeof(*g));
		x.t1 = x.t2 = x.l2 = x.w = x.cn = 0;
		memset(&x.z, 0, sizeof(x.z));
		grid(m, n, k, g, &x);
		for (size_t j = 0; j < k; j++) {
			for (size_t i = 0; i < n; i++) {
				struct mpt *a = &g[(j * w) + i];

				tri(&x, a + 1, a + w + 1, a);
				tri(&x, a + w, a, a + w + 1);
			}
		}
		if ((x.b[0] * x.l2 / 6) + near_of(&x.s) +
			    (ROUNDING * (len(x.s.o) + x.b[3])) <=
		    DELTA) {
			break;
		}
	}
	free(g);
	if (!bounds(&x, fmin(1, x.cn + sqrt(x.l2)))) {
		return 0;
	}
	kd = 1 / ((1 - (x.b[2] * DELTA)) * (1 - (x.b[2] * DELTA)));
	ek = 4 * x.b[2] * DELTA;
	rr = x.b[3];
	area = fabs(x.z.sum[1]);
	vb = ((area * kd) + x.w) * DELTA * (1 + (x.b[2] * DELTA)) *
	     (1 + (x.b[2] * DELTA));
	add_term(&x.z, 0, dot(x.s.o, nf) / 3);
	x.z.bound[0] = vb + (rr / 3 * x.w);
	x.z.bound[1] = x.t2 + (ek * x.t1) + (ek * ek * area / 2) +
		       (area * (kd - 1)) + x.w;
	for (int i = 0; i < 3; i++) {
		add_term(&x.z, 2 + i, x.o[i] * x.o[i] / 2 * comp(nf, i));
		x.z.bound[2 + i] =
			((fabs(x.o[i]) + rr) * vb) +
			(((fabs(x.o[i]) * rr) + (rr * rr / 2)) * x.w);
	}
	*z = x.z;
	return 1;
}

// Whether every edge of the face of m has a rule with a bound.
static int rules_bound(const struct facemap *m)
{
	struct urange r = face_uses(m->sh, m->f);
	int deg = 0;

	for (size_t i = r.first; i < r.end; i++) {
		if (rule_of(&m->sh->f[m->f].s, &m->sh->e[m->sh->u[i].e],
			    &deg) == R_GAUSS) {
			return 0;
		}
	}
	return 1;
}

struct solid_mass shape_mass_to(const struct shape *sh, double rel,
				struct solid_mass *err)
{
	struct solid_mass m = { 0, 0, { 0, 0, 0 } };
	double e[NQ] = { 0, 0, 0, 0, 0 };

	for (size_t i = 0; i < sh->nf; i++) {
		struct facemap fm;
		struct acc z;
		struct solid_mass f;

		facemap_init(&fm, sh, i);
		if (sh->f[i].s.kind != SF_PIPE || rules_bound(&fm) ||
		    !mesh_sum(&fm, &z)) {
			z = face_sum(&fm, COARSE, rel);
		}
		if (z.unproven && sh->f[i].s.kind == SF_SPHERE) {
			z = sphere_sum(sh, i, rel, z);
		}
		if (z.unproven && sh->f[i].s.kind == SF_SPHERE) {
			mesh_sum(&fm, &z);
		}
		for (int k = 0; k < NQ; k++) {
			e[k] += z.unproven ? HUGE_VAL :
					     z.bound[k] + (ROUNDING * z.mag[k]);
		}
		f = mass_of(sh, i, z.sum);
		m.volume += f.volume;
		m.area += f.area;
		m.moment = add(m.moment, f.moment);
	}
	if (err) {
		err->volume = e[0];
		err->area = e[1];
		err->moment = v3(e[2], e[3], e[4]);
	}
	return m;
}
