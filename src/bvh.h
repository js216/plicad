// SPDX-License-Identifier: GPL-3.0-only
// bvh.h -- A tree of boxes over the triangles of a mesh, for rays
// Copyright (c) 2026 Jakob Kastelic

// Each node holds the box of its triangles; a leaf holds a few, and an
// inner node splits them in two halves along the longest side of its
// box, by the middles of the triangles.  A ray visits only the nodes
// whose boxes it passes through.

#ifndef BVH_H
#define BVH_H

#include "geom.h"
#include <stddef.h>

struct bvh_node {
	struct box b;
	size_t lo, hi; // The triangles idx[lo .. hi - 1] of the node.
	size_t left; // The first child, the second after it; 0 for a leaf.
};

struct bvh {
	struct bvh_node *n;
	size_t nn, capn;
	size_t *idx; // The triangles, in the order of the leaves.
};

// Builds the tree of the nt triangles of pos, nine coordinates each.
void bvh_build(struct bvh *t, const float *pos, size_t nt);
void bvh_free(struct bvh *t);

// The distance along the ray r (from r.o along the unit r.d) at which it
// enters box b, if it meets b before tmax; or HUGE_VAL.
double ray_box(struct line3 r, struct box b, double tmax);

#endif
