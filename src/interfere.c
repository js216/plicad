// SPDX-License-Identifier: GPL-3.0-only
// interfere.c -- Print the pairs of placed parts whose material overlaps
// Copyright (c) 2026 Jakob Kastelic

// Usage: interfere assembly.fold
//
// It prints one line per colliding pair: "NAME NAME points N".  The
// material of each part is sampled on a 1 mm grid in its flat faces at
// three depths, and N counts the samples that collide, each once, up to
// POINTS_MAX: past that the count tells nothing more, and parts that lie
// wholly in each other would take long to count.  A sample collides if
// it lies inside the other part, 1 um or more from its faces.  Parts
// that only touch do not collide.  The sampling can miss an overlap
// thinner than the grid.

#include "fold.h"
#include "geom.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define GRID 1.0
#define MARGIN 1e-3

enum { POINTS_MAX = 10000 }; // The most samples counted for a pair.

struct body {
	struct part *p;
	struct box *flat; // Flat box of each face, z unused.
	struct box *world; // World box of each face.
	struct box box; // World box of the part.
};

static void flat_boxes(struct body *b)
{
	const struct part *p = b->p;

	b->flat = xmalloc((p->nfaces + 1) * sizeof(*b->flat));
	for (size_t f = 0; f < p->nfaces; f++) {
		b->flat[f] = box_empty();
	}
	for (size_t i = 0; i < p->nedges; i++) {
		const struct edge *e = &p->edges[i];
		struct vec a = v3(p->verts[e->v0].x, p->verts[e->v0].y, 0);
		struct vec c = v3(p->verts[e->v1].x, p->verts[e->v1].y, 0);

		if (e->lf != NONE) {
			box_add(&b->flat[e->lf], a);
			box_add(&b->flat[e->lf], c);
		}
		if (e->rf != NONE) {
			box_add(&b->flat[e->rf], a);
			box_add(&b->flat[e->rf], c);
		}
	}
}

// Finds the world boxes of the faces and of the part.  The corners of a
// bend zone are inside the bulge of its arc, so the box is larger by the
// bend radius and the thickness.
static void world_boxes(struct body *b)
{
	const struct part *p = b->p;

	b->world = xmalloc((p->nfaces + 1) * sizeof(*b->world));
	b->box = box_empty();
	for (size_t f = 0; f < p->nfaces; f++) {
		const struct box *e = &b->flat[f];
		const struct map *m = &p->maps[p->faces[f]];
		struct box x = box_empty();
		struct box s = *e;

		s.lo.z = 0;
		s.hi.z = -p->t;
		for (unsigned k = 0; k < BOX_CORNERS; k++) {
			struct vec c = box_corner(s, k);

			box_add(&x, fmap(p, f, c.x, c.y, c.z));
		}
		box_pad(&x, m->bend ? m->r + p->t : 0);
		b->world[f] = x;
		box_add(&b->box, x.lo);
		box_add(&b->box, x.hi);
	}
}

// Is the flat point q inside face f, MARGIN or more from its edges and
// holes?
static int deep_in_face(const struct part *p, size_t f, struct vec q)
{
	for (int a = 0; a < 3; a++) {
		for (int k = -1; k <= 1; k += 2) {
			if (!point_in_slab(p, f,
					   add(q, axis_vec(a, k * MARGIN)))) {
				return 0;
			}
		}
	}
	return point_in_slab(p, f, q);
}

// The faces of b whose boxes meet the box of one face of another part,
// and the box of the common parts.
struct near {
	size_t *g;
	struct box *c; // The common box of face g[i] and the other face.
	size_t n;
};

// Is w inside the material of b, MARGIN or more from its faces?  Only
// the faces of nb can hold it.
static int inside(const struct body *b, const struct near *nb, struct vec w)
{
	const struct part *p = b->p;

	for (size_t i = 0; i < nb->n; i++) {
		size_t f = nb->g[i];
		struct vec q;

		if (!box_has(b->world[f], w)) {
			continue;
		}
		if (!finv(p, f, w, &q) || q.z > -MARGIN ||
		    q.z < -p->t + MARGIN) {
			continue;
		}
		if (deep_in_face(p, f, q)) {
			return 1;
		}
	}
	return 0;
}

// Returns the part of the flat box of face f with an image that can lie
// in the box w.
static struct box sample_box(const struct body *a, size_t f, struct box w)
{
	const struct part *p = a->p;
	struct box e = a->flat[f];
	struct box s = box_empty();

	if (p->maps[p->faces[f]].bend) {
		return e;
	}
	for (unsigned k = 0; k < BOX_CORNERS; k++) {
		struct vec q;

		finv(p, f, box_corner(w, k), &q);
		box_add(&s, v3(q.x, q.y, 0));
	}
	e.lo = v3(fmax(e.lo.x, s.lo.x), fmax(e.lo.y, s.lo.y), 0);
	e.hi = v3(fmin(e.hi.x, s.hi.x), fmin(e.hi.y, s.hi.y), 0);
	return e;
}

// The number of grid points lo + GRID (i + 1/2) below hi.
static size_t grid_n(double lo, double hi)
{
	double c = ceil(((hi - lo) / GRID) - MID);

	return c > 0 ? (size_t)c : 0;
}

// Point i of the grid from lo.
static double grid_at(double lo, size_t i)
{
	return lo + (GRID * ((double)i + MID));
}

// The first grid point from lo at or above v, and its count n.
static size_t grid_from(double lo, double v, size_t n)
{
	double c = fmin(floor(((v - lo) / GRID) - MID), (double)n);

	return c > 0 ? (size_t)c : 0;
}

// A face f of a sampled: its map, and the box of the grid of samples
// against all of b.
struct sampled {
	const struct body *a;
	size_t f;
	struct box e;
	struct rigid m;
	int bend;
};

// The grid points of a sampled face, on its grid, that can lie in a
// world box.
struct span2 {
	size_t i0, i1, j0, j1, nx;
};

static struct span2 grid_span(const struct sampled *sf, struct box nb)
{
	struct box e = sf->e;
	struct box r = sample_box(sf->a, sf->f, nb);
	struct span2 s = { 0, 0, 0, 0, grid_n(e.lo.x, e.hi.x) };
	size_t ny = grid_n(e.lo.y, e.hi.y);

	s.i0 = grid_from(e.lo.x, r.lo.x, s.nx);
	s.i1 = grid_from(e.lo.x, r.hi.x + (2 * GRID), s.nx);
	s.j0 = grid_from(e.lo.y, r.lo.y, ny);
	s.j1 = grid_from(e.lo.y, r.hi.y + (2 * GRID), ny);
	return s;
}

// Whether w lies in the common box of a face of nb before face i.
static int seen_before(const struct near *nb, size_t i, struct vec w)
{
	for (size_t h = 0; h < i; h++) {
		if (box_has(nb->c[h], w)) {
			return 1;
		}
	}
	return 0;
}

// The world point of the sample of s at the flat point q.
static struct vec sample_at(const struct sampled *s, struct vec q)
{
	return s->bend ? fmap(s->a->p, s->f, q.x, q.y, q.z) : rig_pt(s->m, q);
}

// Adds to *n the samples of s in the common box of face nb->g[c] that no
// earlier face of nb holds, inside b, up to POINTS_MAX.
static void box_overlap(const struct sampled *s, const struct body *b,
			const struct near *nb, size_t c, unsigned long *n)
{
	const struct part *p = s->a->p;
	struct span2 sp = grid_span(s, nb->c[c]);

	for (size_t j = sp.j0; j < sp.j1 && *n < POINTS_MAX; j++) {
		for (size_t i = sp.i0; i < sp.i1 && *n < POINTS_MAX; i++) {
			for (int k = 1; k <= 3; k++) {
				struct vec q = v3(grid_at(s->e.lo.x, i),
						  grid_at(s->e.lo.y, j),
						  -p->t * k / 4);
				struct vec w = sample_at(s, q);

				if (box_has(nb->c[c], w) &&
				    box_has(b->box, w) &&
				    !seen_before(nb, c, w) &&
				    inside(b, nb, w) &&
				    point_in_slab(p, s->f, q)) {
					(*n)++;
				}
			}
		}
	}
}

// Counts the samples of face f of a inside b, from n on up to
// POINTS_MAX.  The samples lie on the grid of the flat box of f that can
// meet b; each counts once, in the first common box of f with a face of
// nb that holds it.
static unsigned long face_overlap(const struct body *a, size_t f,
				  const struct body *b, const struct near *nb,
				  unsigned long n)
{
	const struct part *p = a->p;
	struct sampled s = { a, f, sample_box(a, f, b->box),
			     map_rig(p, p->faces[f]),
			     p->maps[p->faces[f]].bend };

	for (size_t c = 0; c < nb->n && n < POINTS_MAX; c++) {
		box_overlap(&s, b, nb, c, &n);
	}
	return n;
}

// The common box of a and b, empty if they do not meet.
static int box_common(struct box a, struct box b, struct box *out)
{
	out->lo = v3(fmax(a.lo.x, b.lo.x), fmax(a.lo.y, b.lo.y),
		     fmax(a.lo.z, b.lo.z));
	out->hi = v3(fmin(a.hi.x, b.hi.x), fmin(a.hi.y, b.hi.y),
		     fmin(a.hi.z, b.hi.z));
	return out->lo.x <= out->hi.x && out->lo.y <= out->hi.y &&
	       out->lo.z <= out->hi.z;
}

// The faces of b near the box x, into nb.
static void near_faces(const struct body *b, struct box x, struct near *nb)
{
	nb->n = 0;
	for (size_t g = 0; g < b->p->nfaces; g++) {
		if (box_common(x, b->world[g], &nb->c[nb->n])) {
			nb->g[nb->n++] = g;
		}
	}
}

// Counts the samples of the material of a inside b, each once, from n
// on up to POINTS_MAX.
static unsigned long overlap(const struct body *a, const struct body *b,
			     unsigned long n)
{
	struct near nb = { xmalloc((b->p->nfaces + 1) * sizeof(size_t)),
			   xmalloc((b->p->nfaces + 1) * sizeof(struct box)),
			   0 };

	for (size_t f = 0; f < a->p->nfaces && n < POINTS_MAX; f++) {
		if (!box_meet(a->world[f], b->box)) {
			continue;
		}
		near_faces(b, a->world[f], &nb);
		if (nb.n) {
			n = face_overlap(a, f, b, &nb, n);
		}
	}
	free(nb.g);
	free(nb.c);
	return n;
}

static void report_pairs(const struct body *bs, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		for (size_t j = i + 1; j < n; j++) {
			unsigned long k = 0;

			if (!box_meet(bs[i].box, bs[j].box)) {
				continue;
			}
			k = overlap(&bs[j], &bs[i], overlap(&bs[i], &bs[j], 0));
			k = k < POINTS_MAX ? k : POINTS_MAX;
			if (k) {
				printf("%s %s points %lu\n", bs[i].p->name,
				       bs[j].p->name, k);
			}
		}
	}
}

int main(int argc, char **argv)
{
	struct part *ps = NULL;
	struct body *bs = NULL;
	size_t n = 0;

	if (argc != 2) {
		die("usage: interfere assembly.fold");
	}
	n = read_fold(argv[1], &ps);
	bs = xcalloc(n + 1, sizeof(*bs));
	for (size_t i = 0; i < n; i++) {
		bs[i].p = &ps[i];
		flat_boxes(&bs[i]);
		world_boxes(&bs[i]);
	}
	report_pairs(bs, n);
	for (size_t i = 0; i < n; i++) {
		free(bs[i].flat);
		free(bs[i].world);
	}
	free(bs);
	free_parts(ps, n);
	close_stdout();
	return 0;
}
