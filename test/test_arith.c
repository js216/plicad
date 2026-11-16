// SPDX-License-Identifier: GPL-3.0-only
// test_arith.c -- Tests of expressions, bend arithmetic and line hits
// Copyright (c) 2026 Jakob Kastelic

// The flat sizes are the FreeCAD SheetMetal unfold results for a set of
// covers and chassis 8 in deep.

#include "expr.h"
#include "geom.h"
#include "isect.h"
#include "shape.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define IN 25.4
#define K 0.44 // The ANSI k-factor of the FreeCAD model.
#define EXACT 1e-12 // The tolerance of the exact results,
#define FLAT_TOL 1e-4 // of the flat sizes,
#define REAR_TOL 1e-3 // and of the rear chassis, from rounded sizes.
#define TORUS_R 10.0 // The radius of the torus of test_torus,
#define TUBE_R 1.5 // its tube,
#define LINE_LEN 40.0 // and the length of the lines across it.

enum { SAMPLES = 100000 }; // Steps along a line for its crossings.

static const double cov = 0.06 * IN; // Cover sheet.
static const double ch = 0.09 * IN; // Chassis sheet.
static const double r = 0.03 * IN; // Inside bend radius.
static const double width = 13.35 * IN;
static const double height = 6.72 * IN;
static const double depth = 8 * IN;
static const double pinned = 100; // A value pinned over a definition.

// The FreeCAD sizes: the flats of the top cover, the front chassis and
// its flange, and the rear chassis; the flange leg of the front chassis;
// the pieces of a rear chassis wall, and the jog offset.
static const double cover_flat = 448.2385;
static const double front_flat = 558.0038;
static const double flange_flat = 254.9818;
static const double rear_flat = 558.5856;
static const double flange_leg = 43.942;
static const double wall_a = 200.152;
static const double wall_b = 121.944;
static const double wall_c = 114.929;
static const double wall_d = 88.9;
static const double jog_full = 115.062;
static const double jog_off = 0.25 * IN;

struct expr_case {
	const char *what, *text;
	double want, tol;
};

static const struct expr_case cases[] = {
	{ "units", "1 in + 2mm", 27.4, EXACT },
	{ "precedence", "2 + 3 * 4 - -1", 15, EXACT },
	{ "parentheses", "(2 + 3) * 4", 20, EXACT },
	{ "name", "Height / 3", 56.896, 1e-9 },
	{ "tan", "tan(40 deg)", 0.83909963117728001, 1e-15 },
	{ "atan2", "atan2(1, 1)", 45, EXACT },
	{ "floor", "floor((158.75 - 126.492) / 5.08) + 1", 7, 0 },
	{ "max", "max(1, max(3, 2))", 3, 0 },
	{ "min", "min(4, min(3, 5))", 3, 0 },
	{ "abs", "abs(2 - 5) + abs(1)", 4, 0 },
	{ "ceil", "ceil((158.75 - 126.492) / 5.08)", 7, 0 },
};

// Returns 1, and says so, if got misses want by more than tol.
static int near(const char *what, double got, double want, double tol)
{
	if (!(fabs(got - want) <= tol)) {
		printf("FAIL %s: %.9g, want %.9g\n", what, got, want);
		return 1;
	}
	return 0;
}

static int test_expr(void)
{
	struct names nm = { NULL, 0, 0, 0, NULL, NULL, 0, 0, NULL, NULL, 0 };
	int n = 0;

	define(&nm, "Height", height, 1);
	for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		n += near(cases[i].what, eval(&nm, cases[i].text),
			  cases[i].want, cases[i].tol);
	}
	pin(&nm, "Height", pinned);
	n += near("pin", eval(&nm, "Height"), pinned, 0);
	free_names(&nm);
	return n;
}

static int test_allowance(void)
{
	double ba = bend_allowance(RIGHT, r, ch, K);
	int n = near("ba 90", ba, PI / 2 * (r + K * ch), EXACT);

	return n + near("ba sign", bend_allowance(-RIGHT, r, ch, K), ba, 0);
}

// The top cover has two legs CovLeg high and a plate Width wide, both
// outside sizes.
static int test_cover(void)
{
	double legs = 2 * (height / 3 - cov - r);
	double plate = width - (2 * (cov + r));

	return near("cover flat",
		    legs + plate + (2 * bend_allowance(RIGHT, r, cov, K)),
		    cover_flat, FLAT_TOL);
}

// The front chassis has a panel, two walls WallD deep and two flanges.
static int test_front(void)
{
	double tr = ch + r;
	double ba = bend_allowance(RIGHT, r, ch, K);
	double wall = (depth / 2 + IN / 2) - tr;
	double panel = width - (2 * cov) - (2 * tr);
	int n = near("front flat", panel + (2 * (ba + wall)), front_flat,
		     FLAT_TOL);

	return n + near("flange",
			(2 * (flange_leg + ba)) + (height - 2 * cov - 2 * tr),
			flange_flat, FLAT_TOL);
}

// Each wall of the rear chassis has a corner bend, a wall, a jog of two
// bends of phi and the overlap.
static int test_rear(void)
{
	double ba = bend_allowance(RIGHT, r, ch, K);
	double panel = width - (2 * cov) - (2 * (ch + r));
	double phi = atan2(ch, jog_off) / DEG;
	double so = tan(phi * DEG / 2);
	double jog = hypot(ch, jog_off) - ((r + ch) * so) - (r * so);
	double walls = wall_a - wall_b + wall_c - wall_d;
	int n = near("jog setback", jog_full - (r * so), wall_c, REAR_TOL);

	return n + near("rear flat",
			panel + (2 * (ba + walls + jog +
				      2 * bend_allowance(phi, r, ch, K))),
			rear_flat, REAR_TOL);
}

// The number of changes of sign of the distance to surface s, at n
// steps along the piece p: the crossings, for a line that touches s
// nowhere.
static size_t sign_changes(const struct cpiece *p, const struct surf *s, int n)
{
	size_t c = 0;
	double last = surf_dist(s, curve_at(&p->c, p->t0));

	for (int i = 1; i <= n; i++) {
		double t = p->t0 + ((p->t1 - p->t0) * i / n);
		double d = surf_dist(s, curve_at(&p->c, t));

		c += (d < 0) != (last < 0);
		last = d;
	}
	return c;
}

// Lines across a torus meet it where their distance to it changes sign,
// at points on it: two lines through the hole and the tube, four times
// each, one through the tube twice, and one past it.
static int test_torus(void)
{
	struct surf s;
	static const struct vec at[] = { { -20, 0.3, 0.2 },
					 { -20, -0.5, 0.7 },
					 { 0, 10, -5 },
					 { -20, 20, 0 } };
	static const struct vec dir[] = {
		{ 1, 0, 0 }, { 1, 0.02, 0.01 }, { 0, 0, 1 }, { 1, 0.01, 0 }
	};
	static const size_t want[] = { 4, 4, 2, 0 };
	int n = 0;

	memset(&s, 0, sizeof(s));
	s.kind = SF_TORUS;
	s.o = v3(0, 0, 0);
	s.n = v3(0, 0, 1);
	s.x = v3(1, 0, 0);
	s.r = TORUS_R;
	s.r2 = TUBE_R;
	for (size_t i = 0; i < sizeof(want) / sizeof(want[0]); i++) {
		struct cpiece p;
		double t[ROOTS_MAX];
		size_t m = 0;

		memset(&p, 0, sizeof(p));
		p.c.kind = CV_LINE;
		p.c.o = at[i];
		p.c.x = unit(dir[i]);
		p.t0 = 0;
		p.t1 = LINE_LEN;
		m = curve_surf(&p, &s, t);
		n += m != want[i] || sign_changes(&p, &s, SAMPLES) != m;
		for (size_t k = 0; k < m; k++) {
			n += fabs(surf_dist(&s, curve_at(&p.c, t[k]))) > EXACT;
		}
	}
	if (n) {
		printf("FAIL torus: %d\n", n);
	}
	return n;
}

// A pipe about the ellipse of half axes 12 and 10, of tube 3, which
// folds nowhere, as its least radius of curvature is 100 / 12.
static void pipe_of(struct surf *s)
{
	memset(s, 0, sizeof(*s));
	s->kind = SF_PIPE;
	s->n = v3(0, 0, 1);
	s->x = v3(1, 0, 0);
	s->r = 12;
	s->r3 = 10;
	s->r2 = 3;
}

// The parameters of the points of a pipe come back as they went in, the
// points lie on it, its normal is that of the ring, the point of the
// ellipse nearest a point on its normal is the foot, and lines across it
// meet it where their distance to it changes sign: through the hole and
// the tube, four times each, along x and along y, through the tube
// twice, and past it not at all.
static int test_pipe(void)
{
	struct surf s;
	static const struct vec at[] = { { -20, 0.3, 0.2 },
					 { 0.4, -20, 0.5 },
					 { 12, 0, -5 },
					 { -20, 20, 0 } };
	static const struct vec dir[] = {
		{ 1, 0, 0 }, { 0.01, 1, 0.02 }, { 0, 0, 1 }, { 1, 0.01, 0 }
	};
	static const size_t want[] = { 4, 4, 2, 0 };
	int n = 0;

	pipe_of(&s);
	for (int i = -6; i <= 6; i++) {
		for (int j = -6; j <= 6; j++) {
			struct uv q = { i * 0.5, j * 0.5 };
			struct vec p = surf_at(&s, q);
			struct uv b = surf_param(&s, p);
			struct line3 ring = pipe_ring(&s, q.u);
			struct curve c;

			n += fabs(angle_near(b.u, q.u) - q.u) > 1e-9 ||
			     fabs(angle_near(b.v, q.v) - q.v) > 1e-9 ||
			     fabs(surf_dist(&s, p)) > 1e-9 ||
			     len(sub(surf_normal(&s, p),
				     unit(sub(p, ring.o)))) > 1e-9;
			memset(&c, 0, sizeof(c));
			c.kind = CV_ELLIPSE;
			c.n = s.n;
			c.x = s.x;
			c.a = s.r;
			c.b = s.r3;
			n += fabs(angle_near(curve_near(&c, add(ring.o,
								mul(ring.d,
								    j * 0.5))),
					     q.u) -
				  q.u) > 1e-9;
		}
	}
	for (size_t i = 0; i < sizeof(want) / sizeof(want[0]); i++) {
		struct cpiece p;
		double t[ROOTS_MAX];
		size_t m = 0;

		memset(&p, 0, sizeof(p));
		p.c.kind = CV_LINE;
		p.c.o = at[i];
		p.c.x = unit(dir[i]);
		p.t0 = 0;
		p.t1 = LINE_LEN;
		m = curve_surf(&p, &s, t);
		n += m != want[i] || sign_changes(&p, &s, SAMPLES) != m;
		for (size_t k = 0; k < m; k++) {
			n += fabs(surf_dist(&s, curve_at(&p.c, t[k]))) > EXACT;
		}
	}
	if (n) {
		printf("FAIL pipe: %d\n", n);
	}
	return n;
}

int main(void)
{
	int failures = test_expr() + test_allowance() + test_cover() +
		       test_front() + test_rear() + test_torus() + test_pipe();

	printf("test_arith: %d failures\n", failures);
	close_stdout();
	return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
