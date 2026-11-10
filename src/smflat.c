// SPDX-License-Identifier: GPL-3.0-only
// smflat.c -- Write the flat pattern of a folded part as DXF
// Copyright (c) 2026 Jakob Kastelic

// Usage: smflat -f dxf part.fold > out.dxf
//
// Layer OUTLINE holds the cut contour with true arcs and circles, and
// layer BEND holds one centre line per bend zone.  Units are mm.

#include "fold.h"
#include "geom.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

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

// Draws the centre line of bend face f, across the face along the bend.
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

static void flat(const struct part *p)
{
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
	for (size_t i = 0; i < p->nholes; i++) {
		circle(&p->holes[i]);
	}
	for (size_t f = 0; f < p->nfaces; f++) {
		if (p->maps[p->faces[f]].bend) {
			bend_line(p, f);
		}
	}
	printf("0\nENDSEC\n0\nEOF\n");
}

int main(int argc, char **argv)
{
	struct part *ps = NULL;
	size_t n = 0;

	if (argc != 4 || strcmp(argv[1], "-f") != 0 ||
	    strcmp(argv[2], "dxf") != 0) {
		die("usage: smflat -f dxf part.fold > out.dxf");
	}
	n = read_fold(argv[3], &ps);
	if (n != 1) {
		die("%s: smflat takes a file with one part", argv[3]);
	}
	flat(&ps[0]);
	free_parts(ps, n);
	close_stdout();
	return 0;
}
