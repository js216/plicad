// SPDX-License-Identifier: GPL-3.0-only
// test_arith.c -- Tests of the expression language and the bend arithmetic
// Copyright (c) 2026 Jakob Kastelic

// The flat sizes are the FreeCAD SheetMetal unfold results for the SR792
// covers and chassis at Depth 8 in.

#include "expr.h"
#include "geom.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define IN 25.4
#define K 0.44 // The ANSI k-factor of the FreeCAD model.
#define EXACT 1e-12 // The tolerance of the exact results,
#define FLAT_TOL 1e-4 // of the flat sizes,
#define REAR_TOL 1e-3 // and of the rear chassis, from rounded sizes.

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
	struct names nm = { NULL, 0, 0, 0, NULL, NULL, 0, 0 };
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

int main(void)
{
	int failures = test_expr() + test_allowance() + test_cover() +
		       test_front() + test_rear();

	printf("test_arith: %d failures\n", failures);
	close_stdout();
	return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
