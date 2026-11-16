// SPDX-License-Identifier: GPL-3.0-only
// smflat.c -- Write the flat pattern of a folded part as DXF
// Copyright (c) 2026 Jakob Kastelic

// Usage: smflat -f dxf part.st > out.dxf
//
// The stream holds one part (smfold).  Layer OUTLINE holds the cut
// contour with true arcs and circles, and layer BEND holds one center
// line per bend zone.  Each coat (fold.h) draws its region, cut to the
// box of the blank, on the layer COAT_NEAR_MATERIAL or COAT_FAR_MATERIAL
// of its side and material.  Units are mm.

#include "fold.h"
#include "stream.h"
#include "geom.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { LAYER_LEN = 600 }; // Room for the name of a layer.

// A line x = c (a = 0) or y = c (a = 1), and the side of it kept: that
// where sign (x - c) is not below 0.
struct clip {
	int a, sign;
	double c;
};

static void line(const char *layer, double ax, double ay, double bx, double by)
{
	printf("0\nLINE\n8\n%s\n10\n%.6f\n20\n%.6f\n11\n%.6f\n21\n%.6f\n",
	       layer, ax, ay, bx, by);
}

static void arc(const struct part *p, const struct edge *e)
{
	struct arc_ang a = arc_angles(p, e);

	printf("0\nARC\n8\nOUTLINE\n10\n%.6f\n20\n%.6f\n40\n%.6f\n"
	       "50\n%.6f\n51\n%.6f\n",
	       e->cx, e->cy, a.r, a.a0 / DEG, (a.a0 + a.da) / DEG);
}

static void circle(const struct hole *h)
{
	printf("0\nCIRCLE\n8\nOUTLINE\n10\n%.6f\n20\n%.6f\n40\n%.6f\n", h->cx,
	       h->cy, h->r);
}

// Draws the center line of bend face f, across the face along the bend.
static void bend_line(const struct part *p, size_t f)
{
	const struct map *m = &p->maps[p->faces[f]];
	double dx = m->my;
	double dy = -m->mx;
	double cx = m->qx + (m->mx * m->ba / 2);
	double cy = m->qy + (m->my * m->ba / 2);
	double lo = HUGE_VAL;
	double hi = -HUGE_VAL;

	for (size_t k = p->fe_start[f]; k < p->fe_start[f + 1]; k++) {
		const struct edge *e = &p->edges[p->fe[k]];
		const size_t v[2] = { e->v0, e->v1 };

		for (int j = 0; j < 2; j++) {
			double s = ((p->verts[v[j]].x - m->qx) * dx) +
				   ((p->verts[v[j]].y - m->qy) * dy);

			lo = fmin(lo, s);
			hi = fmax(hi, s);
		}
	}
	line("BEND", cx + (dx * lo), cy + (dy * lo), cx + (dx * hi),
	     cy + (dy * hi));
}

// The part of polygon p of n corners where sign (x - c) along axis a
// (0 for x, 1 for y) is not below 0, into out; returns its corners.
static size_t clip_half(const struct vec2 *p, size_t n, struct clip h,
			struct vec2 *out)
{
	size_t m = 0;

	for (size_t i = 0; i < n; i++) {
		struct vec2 a = p[i];
		struct vec2 b = p[(i + 1) % n];
		double da = h.sign * ((h.a ? a.y : a.x) - h.c);
		double db = h.sign * ((h.a ? b.y : b.x) - h.c);

		if (da >= 0) {
			out[m++] = a;
		}
		if ((da < 0) != (db < 0)) {
			double s = da / (da - db);

			out[m].x = a.x + (s * (b.x - a.x));
			out[m].y = a.y + (s * (b.y - a.y));
			m++;
		}
	}
	return m;
}

// Draws the region of coat c, cut to the box lo, hi of the blank.
static void coat(const struct coat *c, struct vec2 lo, struct vec2 hi)
{
	struct vec2 *buf[2];
	const double side[4] = { lo.x, hi.x, lo.y, hi.y };
	size_t n = c->n;
	char layer[LAYER_LEN];

	xsnprintf(layer, "COAT_%s_%s", c->side > 0 ? "NEAR" : "FAR",
		  c->material);
	buf[0] = xmalloc((c->n + 4 + 1) * sizeof(**buf));
	buf[1] = xmalloc((c->n + 4 + 1) * sizeof(**buf));
	memcpy(buf[0], c->pt, n * sizeof(*c->pt));
	// After four halves the polygon is back in buf[0].
	for (int k = 0; k < 4 && n; k++) {
		struct clip h = { k / 2, k % 2 ? -1 : 1, side[k] };

		n = clip_half(buf[k % 2], n, h, buf[(k + 1) % 2]);
	}
	for (size_t i = 0; n >= 3 && i < n; i++) {
		struct vec2 a = buf[0][i];
		struct vec2 b = buf[0][(i + 1) % n];

		line(layer, a.x, a.y, b.x, b.y);
	}
	free(buf[0]);
	free(buf[1]);
}

static void flat(const struct part *p)
{
	struct vec2 lo = { HUGE_VAL, HUGE_VAL };
	struct vec2 hi = { -HUGE_VAL, -HUGE_VAL };

	printf("0\nSECTION\n2\nENTITIES\n");
	for (size_t i = 0; i < p->nedges; i++) {
		const struct edge *e = &p->edges[i];
		struct vec2 a = p->verts[e->v0];
		struct vec2 b = p->verts[e->v1];

		if ((e->lf == NONE) == (e->rf == NONE)) {
			continue;
		}
		if (e->arc) {
			arc(p, e);
		} else {
			line("OUTLINE", a.x, a.y, b.x, b.y);
		}
	}
	// A blind hole is no cut of the blank.
	for (size_t i = 0; i < p->nholes; i++) {
		if (!(p->holes[i].blind > 0)) {
			circle(&p->holes[i]);
		}
	}
	for (size_t f = 0; f < p->nfaces; f++) {
		if (p->maps[p->faces[f]].bend) {
			bend_line(p, f);
		}
	}
	for (size_t i = 0; i < p->nverts; i++) {
		lo.x = fmin(lo.x, p->verts[i].x);
		lo.y = fmin(lo.y, p->verts[i].y);
		hi.x = fmax(hi.x, p->verts[i].x);
		hi.y = fmax(hi.y, p->verts[i].y);
	}
	for (size_t i = 0; i < p->ncoats; i++) {
		coat(&p->coats[i], lo, hi);
	}
	printf("0\nENDSEC\n0\nEOF\n");
}

int main(int argc, char **argv)
{
	struct part *ps = NULL;
	size_t n = 0;

	if (argc != 4 || strcmp(argv[1], "-f") != 0 ||
	    strcmp(argv[2], "dxf") != 0) {
		die("usage: smflat -f dxf part.st > out.dxf");
	}
	n = read_parts(argv[3], &ps);
	if (!n || n != 1 + ps[0].nins || ps[0].solid) {
		die("%s: smflat takes a file with one folded part", argv[3]);
	}
	flat(&ps[0]);
	free_parts(ps, n);
	close_stdout();
	return 0;
}
