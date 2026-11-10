// SPDX-License-Identifier: GPL-3.0-only
// contact.h -- Whether two solids of a folded part meet, exactly
// Copyright (c) 2026 Jakob Kastelic

// A solid here is a convex cover, a few half-spaces, with the exact
// material inside it: the material of a flat cell of a slab, with the
// holes and the outline of its faces; a slice of a bend, its cylindrical
// shell; or a cut, a hole or a rectangle along a world axis.  Two solids
// meet if their materials come within CS_TOUCH of each other.

#ifndef CONTACT_H
#define CONTACT_H

#include "fold.h"
#include "geom.h"
#include <stddef.h>

#define CS_TOUCH 2e-6 // Solids closer than this meet, in mm.

// A half-space n . p <= d.
struct halfspace {
	struct vec n;
	double d;
};

enum { CS_SLAB, CS_BEND, CS_HOLE, CS_RECT };
enum { COVER_MAX = 8 };

// A solid.  A slab or a bend is the material of map mi of part p in the
// flat rectangle [x0, x1] x [y0, y1], through face fi of that map; a bend
// also has its axis and its inner and outer radius.  A cut runs along
// the world axis a, from z0 to z1 along it: a hole of radius r about
// (cu, cv) in the other two axes, in alphabetical order, or the
// rectangle [x0, x1] x [y0, y1] there.
struct csolid {
	int kind;
	struct halfspace h[COVER_MAX];
	size_t nh;
	const struct part *p;
	size_t mi, fi;
	double x0, y0, x1, y1;
	struct line3 axis;
	double r0, r1;
	int a;
	double cu, cv, r, z0, z1;
};

// Sets the cover of s to the prism of the quadrilateral pt[0 .. 3] and
// its copy pt[4 .. 7], moved along an axis across it; grows it by
// CS_TOUCH.
void csolid_prism(struct csolid *s, const struct vec *pt);
// Sets the cover of s to the box b, grown by CS_TOUCH.
void csolid_box(struct csolid *s, struct box b);
// Whether the materials of a and b meet.
int csolids_meet(const struct csolid *a, const struct csolid *b);

#endif
