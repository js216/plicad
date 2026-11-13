// SPDX-License-Identifier: GPL-3.0-only
// split.h -- A face of an exact solid cut into pieces along curves on it
// Copyright (c) 2026 Jakob Kastelic

#ifndef SPLIT_H
#define SPLIT_H

#include "geom.h"
#include "isect.h"
#include "shape.h"
#include <stddef.h>

// A piece of a curve on the face, from a at t0 to b at t1.
struct pedge {
	struct cpiece p;
	struct vec a, b;
};

// A use of a piece in a loop: piece i, forward from a to b if fwd.
struct puse {
	size_t i;
	int fwd;
};

// A part of the face: its loops, counterclockwise about the natural
// normal of the surface, the first the outer one, and a point inside.
struct region {
	struct puse *u;
	size_t nu, capu;
	size_t *loop0; // Where each loop starts in u.
	size_t nloops, caploops;
	struct vec inside;
};

// The face cut along the cuts: its pieces of curves, and its parts.
struct parts {
	struct pedge *e;
	size_t ne, cape;
	struct region *r;
	size_t nr, capr;
};

// Cuts face f of sh along the n pieces of curves cut, which lie on its
// surface, each from a face boundary or another cut to another, into
// its parts.
void split_face(const struct shape *sh, size_t f, const struct pedge *cut,
		size_t n, struct parts *out);
void free_parts_of(struct parts *p);

#endif
