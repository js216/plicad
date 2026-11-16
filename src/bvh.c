// SPDX-License-Identifier: GPL-3.0-only
// bvh.c -- A tree of boxes over the triangles of a mesh, for rays
// Copyright (c) 2026 Jakob Kastelic

#include "bvh.h"
#include "geom.h"
#include "obj.h"
#include "util.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

enum { LEAF_MAX = 8 }; // The most triangles of a leaf.

#define DIR_MIN 1e-300 // A smaller part of a direction is none.

// The work of a build: the triangles and the middle of each.
struct build {
	struct bvh *t;
	const float *pos;
	struct vec *mid;
};

static struct vec corner_of(const float *pos, size_t t, size_t k)
{
	const float *p = &pos[(TRI_FLOATS * t) + (3 * k)];

	return v3((double)p[0], (double)p[1], (double)p[2]);
}

// A range of the triangles of the tree, idx[lo .. hi - 1].
struct range {
	size_t lo, hi;
};

// Puts the triangles of r in order along axis a about the middle one,
// idx[(r.lo + r.hi) / 2]: those before it lie no further along a, those
// after it no nearer.
static void select_mid(struct build *w, struct range r, int a)
{
	size_t *idx = w->t->idx;
	size_t k = (r.lo + r.hi) / 2;
	size_t lo = r.lo;
	size_t hi = r.hi;

	while (hi - lo > 1) {
		double pivot = comp(w->mid[idx[(lo + hi) / 2]], a);
		size_t i = lo;
		size_t j = hi - 1;

		while (i <= j) {
			while (comp(w->mid[idx[i]], a) < pivot) {
				i++;
			}
			while (comp(w->mid[idx[j]], a) > pivot) {
				j--;
			}
			if (i <= j) {
				size_t x = idx[i];

				idx[i] = idx[j];
				idx[j] = x;
				i++;
				if (j == 0) {
					break;
				}
				j--;
			}
		}
		if (k <= j) {
			hi = j + 1;
		} else if (k >= i) {
			lo = i;
		} else {
			return;
		}
	}
}

// Splits node n in two children, or leaves it a leaf if it holds few
// triangles or they all have one middle.
static void split_node(struct build *w, size_t n)
{
	struct bvh *t = w->t;
	struct range r = { t->n[n].lo, t->n[n].hi };
	struct box m = box_empty();
	struct vec d;
	int a = 0;
	size_t left = t->nn;

	for (size_t i = r.lo; i < r.hi; i++) {
		box_add(&m, w->mid[t->idx[i]]);
	}
	d = sub(m.hi, m.lo);
	if (r.hi - r.lo <= LEAF_MAX || !(len(d) > 0)) {
		return;
	}
	if (d.y > d.x && d.y >= d.z) {
		a = 1;
	} else if (d.z > d.x && d.z > d.y) {
		a = 2;
	}
	select_mid(w, r, a);
	t->nn += 2;
	t->n[n].left = left;
	t->n[left].lo = r.lo;
	t->n[left].hi = (r.lo + r.hi) / 2;
	t->n[left].left = 0;
	t->n[left + 1].lo = (r.lo + r.hi) / 2;
	t->n[left + 1].hi = r.hi;
	t->n[left + 1].left = 0;
}

// Sets the box of node n: that of its triangles for a leaf, of its
// children, which come after it, for an inner node.
static void node_box(struct build *w, size_t n)
{
	struct bvh *t = w->t;
	struct bvh_node *x = &t->n[n];
	struct box b = box_empty();

	if (x->left) {
		const struct box *l = &t->n[x->left].b;
		const struct box *r = &t->n[x->left + 1].b;

		box_add(&b, l->lo);
		box_add(&b, l->hi);
		box_add(&b, r->lo);
		box_add(&b, r->hi);
	} else {
		for (size_t i = x->lo; i < x->hi; i++) {
			for (size_t k = 0; k < 3; k++) {
				box_add(&b, corner_of(w->pos, t->idx[i], k));
			}
		}
	}
	x->b = b;
}

void bvh_build(struct bvh *t, const float *pos, size_t nt)
{
	struct build w = { t, pos, NULL };

	memset(t, 0, sizeof(*t));
	t->capn = (2 * nt) + 1;
	t->n = xmalloc(t->capn * sizeof(*t->n));
	t->idx = xmalloc((nt + 1) * sizeof(*t->idx));
	w.mid = xmalloc((nt + 1) * sizeof(*w.mid));
	for (size_t i = 0; i < nt; i++) {
		t->idx[i] = i;
		w.mid[i] =
			mul(add(add(corner_of(pos, i, 0), corner_of(pos, i, 1)),
				corner_of(pos, i, 2)),
			    1.0 / 3);
	}
	t->n[0].lo = 0;
	t->n[0].hi = nt;
	t->n[0].left = 0;
	t->nn = 1;
	// The nodes split in the order they come; each child comes after
	// its parent, so the boxes go from the last node back.
	for (size_t n = 0; n < t->nn; n++) {
		split_node(&w, n);
	}
	for (size_t n = t->nn; n-- > 0;) {
		node_box(&w, n);
	}
	free(w.mid);
}

void bvh_free(struct bvh *t)
{
	free(t->n);
	free(t->idx);
	memset(t, 0, sizeof(*t));
}

double ray_box(struct line3 r, struct box b, double tmax)
{
	double lo = 0;
	double hi = tmax;

	for (int a = 0; a < 3; a++) {
		double o = comp(r.o, a);
		double d = comp(r.d, a);
		double b0 = comp(b.lo, a);
		double b1 = comp(b.hi, a);

		if (fabs(d) < DIR_MIN) {
			if (o < b0 || o > b1) {
				return HUGE_VAL;
			}
			continue;
		}
		{
			double t0 = (b0 - o) / d;
			double t1 = (b1 - o) / d;

			lo = fmax(lo, fmin(t0, t1));
			hi = fmin(hi, fmax(t0, t1));
		}
		if (lo > hi) {
			return HUGE_VAL;
		}
	}
	return lo;
}
