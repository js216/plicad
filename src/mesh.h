// SPDX-License-Identifier: GPL-3.0-only
// mesh.h -- Triangles of exact solids, for viewing
// Copyright (c) 2026 Jakob Kastelic

// A mesh holds the triangles of solids, each triangle three vertices
// counterclockwise about the outward normal, and the face it lies on.
// Two faces that meet share the points of their edge, so that the mesh
// of a closed solid is closed: each edge of a triangle is an edge of one
// other, the other way round.  No point of a triangle lies further from
// its face than the chord, nor does a triangle turn by more than the
// angle across its face.

#ifndef MESH_H
#define MESH_H

#include "fold.h"
#include "geom.h"
#include "shape.h"
#include <stddef.h>

struct mesh_tri {
	size_t v[3];
	size_t face;
};

struct mesh {
	struct vec *v;
	size_t nv, capv;
	struct mesh_tri *t;
	size_t nt, capt;
};

// The chord in mm and the angle in degrees of a mesh.
struct mesh_opts {
	double chord, angle;
};

void mesh_init(struct mesh *m);
void mesh_free(struct mesh *m);
// Adds the triangles of solid sh to m, its vertices after those there.
void mesh_shape(const struct shape *sh, struct mesh_opts o, struct mesh *m);
// The exact solid of a folded part, its full circles split in two.
void part_shape(const struct part *p, struct shape *sh);

#endif
