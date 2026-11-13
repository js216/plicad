// SPDX-License-Identifier: GPL-3.0-only
// integ.c -- Volume, area and centre of exact solids, by quadrature
// Copyright (c) 2026 Jakob Kastelic

// The divergence theorem turns the volume into a sum over the faces of
// integrals of p . n / 3, and the moments of the volume into integrals of
// x^2 n_x / 2 and so on.  Over a face, each is an integral of g(u, v) over
// the domain of its parameters, and Green's theorem turns that into the
// loop integral of G dv, with G(u, v) the integral of g from the middle
// angle u0 of the face to u.  Gauss-Legendre rules on short pieces give
// both integrals; the integrands are smooth there, so the sums agree
// with the exact values to about 1e-12 of the size.

#include "integ.h"
#include "geom.h"
#include "isect.h"
#include "shape.h"
#include <math.h>
#include <stdlib.h>

#define PIECE (PI / 16) // The longest piece of an angle, in radians.

enum {
	GAUSS = 8, // Points of the rule.
	LINE_PIECES = 4, // Pieces of a line.
	NQ = 5 // The integrands: volume, area, three moments.
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

// G: the integrals of g over u from u0 to q.u at q.v.
static void inner(const struct surf *s, double u0, struct uv q, double *G)
{
	double span = q.u - u0;
	double m = s->kind == SF_PLANE ? 1 : ceil(fabs(span) / PIECE);
	size_t n = (size_t)m;
	double h = 0;

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
static size_t pieces(const struct curve *c, double t0, double t1)
{
	double n = c->kind == CV_LINE ? (double)LINE_PIECES :
					ceil(fabs(t1 - t0) / PIECE);

	return n < 1 ? 1 : (size_t)n;
}

// Adds the loop integral of G dv along use u of edge e of the face of
// m, the angles kept within half a turn of the middle of the face, G
// from there.
static void use_sum(const struct facemap *m, const struct sedge *e,
		    const struct suse *u, double *sum)
{
	const struct surf *s = &m->sh->f[m->f].s;
	double a = u->fwd ? e->t0 : e->t1;
	double b = u->fwd ? e->t1 : e->t0;
	size_t n = pieces(&e->c, e->t0, e->t1);
	double h = (b - a) / (double)n;

	for (size_t i = 0; i < n; i++) {
		for (int j = 0; j < GAUSS; j++) {
			double t = a + (h * ((double)i + ((1 + gx[j]) / 2)));
			struct vec p = curve_at(&e->c, t);
			struct vec d =
				mul(curve_tan(&e->c, t), u->fwd ? 1 : -1);
			struct uv q = fm_uv(m, p);
			double gu[NQ];
			double dv = surf_dv(s, (struct line3){ p, d });

			inner(s, m->ref.u, q, gu);
			for (int k = 0; k < NQ; k++) {
				sum[k] += gu[k] * dv * gw[j] * fabs(h) / 2;
			}
		}
	}
}

static void face_sum(const struct shape *sh, size_t f, double *sum)
{
	const struct sface *fc = &sh->f[f];
	struct facemap m;

	facemap_init(&m, sh, f);
	for (size_t l = 0; l < fc->nloops; l++) {
		const struct sloop *lp = &sh->l[fc->loop0 + l];

		for (size_t k = 0; k < lp->n; k++) {
			const struct suse *u = &sh->u[lp->first + k];

			use_sum(&m, &sh->e[u->e], u, sum);
		}
	}
}

struct solid_mass face_mass(const struct shape *sh, size_t f)
{
	struct solid_mass m = { 0, 0, { 0, 0, 0 } };
	double sum[NQ] = { 0, 0, 0, 0, 0 };

	face_sum(sh, f, sum);
	m.volume = sum[0];
	m.area = sh->f[f].sense * sum[1];
	m.moment = v3(sum[2], sum[3], sum[4]);
	return m;
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
