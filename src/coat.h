// SPDX-License-Identifier: GPL-3.0-only
// coat.h -- The triangles of a folded part split along its coats
// Copyright (c) 2026 Jakob Kastelic

// A coat is a region of the flat blank on one side of the sheet (fold.h).
// Every triangle of the mesh of the part is cut along the lines of the
// sides of every coat, in the flat blank, so that each piece lies wholly
// in a coat or out of it; then each piece takes the material of the last
// coat that holds its middle, on its side: the near face for near, the
// far face for far, and with edges the faces of the cut edges and
// the holes.  The cuts across a bend follow the bend.  Two triangles
// that share an edge cut it at the same points, so the mesh stays
// closed.

#ifndef COAT_H
#define COAT_H

#include "fold.h"
#include "mesh.h"
#include "shape.h"
#include <stddef.h>

// Cuts the triangles of m, the mesh of folded part p whose solid sh has
// the faces of the origins from (mesh.h), along the coats of p, and sets
// (*fin)[triangle] to the index of the coat whose material it takes, or
// NONE.
void coat_mesh(const struct part *p, const struct face_from *from,
	       struct mesh *m, size_t **fin);

#endif
