// SPDX-License-Identifier: GPL-3.0-only
// measure.c -- Print the volume, area, box, holes and flat size of parts
// Copyright (c) 2026 Jakob Kastelic

// Usage: measure part.fold
//
// For each part it prints one item per line:
//   part NAME
//   volume V
//   area A
//   bbox XMIN YMIN ZMIN XMAX YMAX ZMAX
//   faces N plane P cylinder C [cone K]
//   hole X Y Z d D axis AX AY AZ      (axis components without sign)
//   flat W H area A holes N bends B  (W >= H, the box of the blank)
// The faces line counts the cones only if there are any.

#include "brep.h"
#include "bsolid.h"
#include "fold.h"
#include "geom.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// The keys of a hole: its axis, diameter and centre.
enum { K_AX, K_AY, K_AZ, K_D, K_CX, K_CY, K_CZ, K_N };
#define ROUND_TO 1000.0 // Holes sort by keys rounded to 1 / ROUND_TO.
#define ALONG 1e-9 // Sine below which a side runs along a bend axis.

struct hole_out {
	double k[K_N];
};

static double r3(double x)
{
	return floor((x * ROUND_TO) + MID) / ROUND_TO;
}

// Orders holes by axis, diameter and centre, each rounded to 0.001.
static int cmp_hole(const void *lhs, const void *rhs)
{
	const struct hole_out *x = lhs;
	const struct hole_out *y = rhs;

	for (int i = 0; i < K_N; i++) {
		if (r3(x->k[i]) < r3(y->k[i])) {
			return -1;
		}
		if (r3(x->k[i]) > r3(y->k[i])) {
			return 1;
		}
	}
	return 0;
}

// Grows the box by the extreme points of a circular edge on each axis.
static void circle_box(const struct brep *b, const struct bedge *e,
		       struct box *x)
{
	struct vec u = unit(sub(b->v[e->v0], e->c));
	struct vec w = cross(e->n, u);

	for (int ax = 0; ax < 3; ax++) {
		double th = atan2(comp(w, ax), comp(u, ax));

		for (int k = 0; k < 2; k++) {
			double a = fmod(th + (k * PI) + (4 * PI), 2 * PI);
			struct vec d = add(mul(u, cos(a)), mul(w, sin(a)));

			if (a <= e->sweep) {
				box_add(x, add(e->c, mul(d, e->r)));
			}
		}
	}
}

// Adds the volume and the cap areas of 2D face f.  A bend zone is a
// full-width rectangle, ext long along the bend.
static void face_size(const struct part *p, size_t f, struct solid_size *sz)
{
	const struct map *m = &p->maps[p->faces[f]];
	double a2 = face_area2(p, f);
	double ext = m->bend ? a2 / m->ba : 0;
	double a = fabs(m->ang) * DEG;
	double r1 = m->r + p->t;

	if (!m->bend) {
		sz->vol += a2 * p->t;
		sz->area += 2 * a2;
		return;
	}
	sz->vol += ext * a / 2 * (r1 * r1 - m->r * m->r);
	sz->area += ext * a * (m->r + r1);
}

// Adds the area of the side along outline edge e.  On a bend face, a
// side across the bend is an end cap of the bend, and a side along its
// axis is a flat strip, a free edge of the zone.
static void side_size(const struct part *p, const struct edge *e, double *area)
{
	size_t f = e->lf != NONE ? e->lf : e->rf;
	const struct map *m = &p->maps[p->faces[f]];
	double r1 = m->r + p->t;
	struct vec2 a = p->verts[e->v0];
	struct vec2 b = p->verts[e->v1];
	double l = edge_len2(p, e);

	if (!m->bend ||
	    fabs(((b.x - a.x) * m->my) - ((b.y - a.y) * m->mx)) > ALONG * l) {
		*area += l * p->t;
	} else {
		*area += fabs(m->ang) * DEG / 2 * (r1 * r1 - m->r * m->r);
	}
}

// Adds the wall area of hole h, and takes off what its countersink adds
// to the hole: the volume beyond the cylinder, and the ring of the face.
static void hole_size(const struct part *p, const struct hole *h,
		      struct solid_size *sz)
{
	double r = h->r;
	double rc = h->rc;

	if (!h->side) {
		sz->area += 2 * PI * r * p->t;
		return;
	}
	sz->area += (2 * PI * r * (p->t - h->h)) +
		    (PI * (r + rc) * hypot(h->h, rc - r)) -
		    (PI * ((rc * rc) - (r * r)));
	sz->vol -= (PI * h->h * ((rc * rc) + (rc * r) + (r * r)) / 3) -
		   (PI * r * r * h->h);
}

static void print_size(const struct part *p)
{
	struct solid_size sz = { 0, 0 };

	for (size_t f = 0; f < p->nfaces; f++) {
		face_size(p, f, &sz);
	}
	for (size_t i = 0; i < p->nedges; i++) {
		if ((p->edges[i].lf == NONE) != (p->edges[i].rf == NONE)) {
			side_size(p, &p->edges[i], &sz.area);
		}
	}
	for (size_t i = 0; i < p->nholes; i++) {
		hole_size(p, &p->holes[i], &sz);
	}
	printf("volume %.4f\narea %.4f\n", sz.vol, sz.area);
}

// Prints the bounding box and the face counts of the solid.
static void print_box(const struct part *p)
{
	struct brep b;
	struct box x = box_empty();
	size_t planes = 0;
	size_t cones = 0;

	brep_build(p, &b);
	for (size_t i = 0; i < b.nv; i++) {
		box_add(&x, b.v[i]);
	}
	for (size_t i = 0; i < b.ne; i++) {
		if (b.e[i].type == C_CIRCLE) {
			circle_box(&b, &b.e[i], &x);
		}
	}
	for (size_t i = 0; i < b.nf; i++) {
		planes += b.f[i].type == S_PLANE;
		cones += b.f[i].type == S_CONE;
	}
	printf("bbox %.4f %.4f %.4f %.4f %.4f %.4f\n", x.lo.x + 0.0,
	       x.lo.y + 0.0, x.lo.z + 0.0, x.hi.x + 0.0, x.hi.y + 0.0,
	       x.hi.z + 0.0);
	printf("faces %zu plane %zu cylinder %zu", b.nf, planes,
	       b.nf - planes - cones);
	if (cones) {
		printf(" cone %zu", cones);
	}
	printf("\n");
	brep_free(&b);
}

static size_t sheet_holes(const struct part *p, struct hole_out **out)
{
	struct hole_out *hs = xmalloc((p->nholes + 1) * sizeof(*hs));

	for (size_t i = 0; i < p->nholes; i++) {
		const struct hole *h = &p->holes[i];
		struct vec c = fmap(p, h->face, h->cx, h->cy, -p->t / 2);
		struct vec n = fnormal(p, h->face);
		struct hole_out o = { { fabs(n.x), fabs(n.y), fabs(n.z),
					2 * h->r, c.x, c.y, c.z } };

		hs[i] = o;
	}
	*out = hs;
	return p->nholes;
}

static void print_holes(const struct part *p)
{
	struct hole_out *hs = NULL;
	size_t n = sheet_holes(p, &hs);

	qsort(hs, n, sizeof(*hs), cmp_hole);
	for (size_t i = 0; i < n; i++) {
		const double *k = hs[i].k;

		printf("hole %.4f %.4f %.4f d %.4f axis %.0f %.0f %.0f\n",
		       k[K_CX] + 0.0, k[K_CY] + 0.0, k[K_CZ] + 0.0, k[K_D],
		       k[K_AX], k[K_AY], k[K_AZ]);
	}
	free(hs);
}

static void print_flat(const struct part *p)
{
	struct box x = box_empty();
	double area = 0;
	size_t bends = 0;
	double w = 0;
	double h = 0;

	for (size_t i = 0; i < p->nverts; i++) {
		box_add(&x, v3(p->verts[i].x, p->verts[i].y, 0));
	}
	for (size_t f = 0; f < p->nfaces; f++) {
		area += face_area2(p, f);
	}
	for (size_t i = 0; i < p->nmaps; i++) {
		bends += p->maps[i].bend != 0;
	}
	w = x.hi.x - x.lo.x;
	h = x.hi.y - x.lo.y;
	printf("flat %.4f %.4f area %.3f holes %zu bends %zu\n", fmax(w, h),
	       fmin(w, h), area, p->nholes, bends);
}

int main(int argc, char **argv)
{
	struct part *ps = NULL;
	size_t n = 0;

	if (argc != 2) {
		die("usage: measure part.fold");
	}
	n = read_fold(argv[1], &ps);
	for (size_t i = 0; i < n; i++) {
		printf("part %s\n", ps[i].name);
		print_size(&ps[i]);
		print_box(&ps[i]);
		print_holes(&ps[i]);
		print_flat(&ps[i]);
	}
	free_parts(ps, n);
	close_stdout();
	return 0;
}
