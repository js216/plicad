// SPDX-License-Identifier: GPL-3.0-only
// brep.h -- Faces, edges and vertices of a part
// Copyright (c) 2026 Jakob Kastelic

#ifndef BREP_H
#define BREP_H

#include "fold.h"
#include <stddef.h>

enum { C_LINE, C_CIRCLE };
// A cone has the radius r at o, and widens by r2 per unit along n.
enum { S_PLANE, S_CYLINDER, S_CONE };

// A circle runs counterclockwise about n from v0 to v1 through sweep
// radians (2 pi and v0 == v1 for a full circle).
struct bedge {
	size_t v0, v1;
	int type;
	struct vec c, n;
	double r, sweep;
};

struct buse {
	size_t e;
	int fwd;
};

// A loop of n uses, from use first on.
struct bloop {
	size_t first, n;
};

// The origin of a face: the top or bottom of 2D face idx, the side along
// 2D edge idx, or the wall of hole idx of a folded part.
enum { F_TOP, F_BOTTOM, F_SIDE, F_HOLE };

// A plane has point o, normal n and in-plane direction x.  A cylinder or
// a cone has axis point o, axis n, reference direction x and radius r.
// The sense is +1 if the outward normal of the face is the natural normal
// of the surface (n for a plane, away from the axis otherwise), and -1 if
// not.
struct bface {
	int type;
	struct vec o, n, x;
	double r, r2;
	int sense;
	size_t loop0, nloops;
	int from;
	size_t idx;
};

struct brep {
	struct vec *v;
	size_t nv, capv;
	struct bedge *e;
	size_t ne, cape;
	struct buse *u;
	size_t nu, capu;
	struct bloop *l;
	size_t nl, capl;
	struct bface *f;
	size_t nf, capf;
};

void brep_build(const struct part *p, struct brep *b);
void brep_free(struct brep *b);

#endif
