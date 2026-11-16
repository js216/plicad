// SPDX-License-Identifier: GPL-3.0-only
// clash.h -- Whether two exact solids overlap, by samples of their faces
// Copyright (c) 2026 Jakob Kastelic

// Two solids overlap where one holds material of the other.  The faces
// and the edges of each solid are sampled on the exact surfaces and
// curves, and each sample moves CLASH_MARGIN into its own solid: a
// sample of one that lies in the other is a clash.  Solids that only
// touch, face on face or edge on face, have none.
//
// The samples lie at most h apart, h the lesser of CLASH_GRID and half
// the thinner of the two solids (clash_init), but only where the faces
// of the other solid come within REACH (clash.c): a piece of a face, or
// of an edge, that no face of the other comes near lies wholly inside
// the other or wholly outside it, and one sample tells which; so does
// one for a flat piece that only flat faces of the other come near, all
// on one side of it or in its plane, and no edge.  A piece near the
// other is halved until no side is longer than h, and sampled at its
// middle, which no point of the piece lies more than 0.58 h from.  So an
// overlap is found wherever an edge of one solid runs inside the other
// for more than h, or the part of a face of one inside the other holds
// a disk of diameter 1.2 h: wherever one solid goes into or through the
// other, however thin, as the part of a face inside the other is then at
// least as wide as the thinner solid, 2 h.  The step stops at STEP_MIN,
// and an overlap thinner than about twice CLASH_MARGIN can pass unseen.

#ifndef CLASH_H
#define CLASH_H

#include "geom.h"
#include "isect.h"
#include "mesh.h"
#include "shape.h"
#include <stddef.h>

#define CLASH_GRID 0.5 // The most room between two samples, in mm.
#define CLASH_MARGIN 1e-3 // How far a sample lies inside its solid, in mm.
#define CLASH_CHORD 0.05 // The chord of the triangles sampled, in mm.

// A tree of boxes, of the faces or the triangles of a solid: node k holds
// the items idx[lo .. hi) in its box, and its children, if any, are left
// and left + 1.
struct clash_node {
	struct box b;
	size_t lo, hi, left;
};

struct clash_tree {
	struct clash_node *node;
	size_t nnode, capnode;
	size_t *idx;
	const struct box *items;
};

// A solid in its own frame: its box, the cache of its face tests and
// the boxes of its faces in it, their tree, the triangles of its faces,
// the boxes of their pieces and the tree of those, the tree of the boxes
// of its edges, the two faces of each edge, and the thinnest wall: the
// thickness of a sheet, or the least distance through the solid along
// the normal of a face to a face turned away from it.
struct clash_body {
	const struct shape *sh;
	struct box box;
	struct shape_cache cache;
	const struct box *fbox; // Those of the cache.
	struct clash_tree ftree;
	struct mesh m;
	struct box *tbox; // The box of each piece of a triangle (clash.c).
	size_t *tface; // The face of each piece.
	struct vec (*tpt)[3]; // The corners of each piece.
	size_t ntbox;
	struct clash_tree ttree;
	struct clash_tree etree; // That of the boxes of the edges.
	size_t (*ef)[2];
	double thin;
};

// Makes the body of solid sh, its thinnest wall thin, or if thin is 0
// that found along the normals at the middles of its triangles.
void clash_init(struct clash_body *b, const struct shape *sh, double thin);
void clash_free(struct clash_body *b);

// A solid where it stands: its body and the motion from its frame to
// the world.
struct clash_pose {
	const struct clash_body *b;
	struct rigid m;
};

// The number of samples of a and of b that lie in the other, at most
// max: 0 when they only touch or stand apart.
size_t clash_count(struct clash_pose a, struct clash_pose b, size_t max);
// The least distance between a and b, up to limit, by the points of the
// mesh of each against the triangles of the other: within about
// CLASH_CHORD of the true one; limit if they stand farther apart.
double clash_gap(struct clash_pose a, struct clash_pose b, double limit);

#endif
