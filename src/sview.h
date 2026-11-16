// SPDX-License-Identifier: GPL-3.0-only
// sview.h -- Project solid parts into the lines of one drawing view
// Copyright (c) 2026 Jakob Kastelic

// The view of solid parts for smview(1), in its records (smview.c): of
// one solid, or of the parts of an assembly in place, each seen as a
// solid, a folded one too.  Each edge, cut where it goes behind
// material, seen along the axis z of the view; the outlines of the
// cylinders and the cones whose axes lie across it; the holes seen along
// their axes, with the depth of a blind one, the countersink at a mouth
// and the thread that the part gives them; and the textured faces that
// look at the eye, as coats of their finish.  A line stays a line and a
// circle seen face on an arc; any other curve goes out as one poly
// record, the points of a line that keeps within the chord of the
// curve.  The edges between two faces that meet smoothly, as a round
// meets its faces, are of the kind s; those of holes, o.  A section cuts
// the parts by the plane at depth cut along z and leaves out what lies
// nearer the eye: the faces cut of each part show filled, as fill
// regions that name the part, edged by lines of the kind e.  The
// outlines of spheres, tori and pipes are left out: rounds show by their
// edges.

#ifndef SVIEW_H
#define SVIEW_H

#include "fold.h"
#include "geom.h"

// A view: its name, x to the right and y up on the paper and z toward
// the eye; whether it shows the hidden lines; whether it is a section,
// by the plane at the depth cut along z; the chord, in mm, that a poly
// record keeps within; the texture of the faces not marked, or NULL
// for that of the most area; the scale of the view, for its view
// record; and what the messages name it by, before the name of the
// view.
struct sview {
	const char *name;
	struct vec x, y, z;
	int hidden, section;
	double cut, chord, scale;
	const char *unmarked, *where;
};

// Prints the records of view v of the n parts ps on stdout.
void sview_print(const struct part *ps, size_t n, struct sview v);

#endif
