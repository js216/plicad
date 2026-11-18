// SPDX-License-Identifier: GPL-3.0-only
// pcbgerber.c -- Write the Gerber and drill files of a board
// Copyright (c) 2026 Jakob Kastelic

// Usage: pcbgerber [-L LIB]... -o DIR board.pcb < board.lock
//
// Reads the source (pcb(5)), its netlist and footprints, and the lock,
// and writes into DIR one RS-274X file for each copper layer, named
// after the layer (top.gbr, in1.gbr, ..., bottom.gbr), the outline
// (edge.gbr), the solder mask and paste of the top (topmask.gbr,
// toppaste.gbr), the courtyards and silkscreen of each side (topcourt,
// bottomcourt, topsilk, bottomsilk) and the Excellon drill file
// (drill.drl), in mm with six decimals; what each layer draws is
// pcbplot's.  Each pad carries the name
// of its part (TO.C, the name from the schematic) and each object its
// net (TO.N), so a viewer can name what is under the mouse.

#include "pcbplot.h"
#include "pcbsrc.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UNITS 1e6 // The units of a coordinate in a mm: format 4.6.
#define SAME 1e-9 // Sizes closer than this are one, in mm.
#define OUTLINE_W 0.05 // The width a line of width 0 is drawn, in mm.
enum {
	D_FIRST = 10, // The number of the first aperture.
	TOOLS_MAX = 64, // The most drill sizes.
	NAME_LEN = 300 // Room for the name of a file.
};

// An aperture: its shape (0 a circle, 1 a rectangle) and size.
struct ap {
	int shape;
	double w, h;
};

// The Gerbers under way: of the board b, into the directory dir; the
// apertures of the file out, and whether object attributes are set.
struct gerber_out {
	const struct pcb *b;
	const char *dir;
	struct ap *aps;
	size_t naps;
	FILE *out;
	int attrs;
};

// The drill sizes of a board.
struct tools {
	double size[TOOLS_MAX];
	size_t n;
};

static FILE *open_out(const struct gerber_out *g, const char *name)
{
	size_t room = strlen(g->dir) + strlen(name) + 2;
	char *p = pcb_alloc(room);
	FILE *f = NULL;

	(void)snprintf(p, room, "%s/%s", g->dir, name);
	f = fopen(p, "w");
	if (!f) {
		pcb_die("%s: cannot write", p);
	}
	free(p);
	return f;
}

// Closes the file f of g, or dies if it was not all written.
static void close_out(const struct gerber_out *g, FILE *f)
{
	int failed = ferror(f);

	if (fclose(f) || failed) {
		pcb_die("%s: cannot write", g->dir);
	}
}

static void g_begin(void *ctx, struct plot_file f)
{
	struct gerber_out *g = ctx;
	char file[NAME_LEN];
	int n = snprintf(file, sizeof file, "%s.gbr", f.name);

	if (n < 0 || (size_t)n >= sizeof file) {
		pcb_die("%s: the name of a layer is too long", f.name);
	}
	g->out = open_out(g, file);
	free(g->aps);
	g->aps = NULL;
	g->naps = 0;
	(void)fprintf(g->out, "%%TF.GenerationSoftware,PliCad,pcbgerber*%%\n");
	(void)fprintf(g->out, "%%TF.FileFunction,%s*%%\n", f.function);
	(void)fprintf(g->out, "%%FSLAX46Y46*%%\n%%MOMM*%%\n%%LPD*%%\nG01*\n");
	g->attrs = 0;
}

static void g_end(void *ctx)
{
	struct gerber_out *g = ctx;

	(void)fprintf(g->out, "M02*\n");
	close_out(g, g->out);
}

static long mm(double v)
{
	return lround(v * UNITS);
}

// Selects the aperture a, defining it the first time.
static void aperture(struct gerber_out *g, struct ap a)
{
	size_t i = 0;

	for (i = 0; g->aps && i < g->naps; i++) {
		if (g->aps[i].shape == a.shape &&
		    fabs(g->aps[i].w - a.w) < SAME &&
		    fabs(g->aps[i].h - a.h) < SAME) {
			break;
		}
	}
	if (i == g->naps) {
		g->aps = pcb_grow(g->aps, g->naps, sizeof *g->aps);
		g->aps[g->naps++] = a;
		if (a.shape) {
			(void)fprintf(g->out, "%%ADD%zuR,%.6fX%.6f*%%\n",
				      i + D_FIRST, a.w, a.h);
		} else {
			(void)fprintf(g->out, "%%ADD%zuC,%.6f*%%\n",
				      i + D_FIRST, a.w);
		}
	}
	(void)fprintf(g->out, "D%zu*\n", i + D_FIRST);
}

static void polarity(void *ctx, int dark)
{
	struct gerber_out *g = ctx;

	(void)fprintf(g->out, "%%LP%c*%%\n", dark ? 'D' : 'C');
}

static void g_attr(struct gerber_out *g, const char *key, const char *value)
{
	if (value) {
		(void)fprintf(g->out, "%%TO.%s,%s*%%\n", key, value);
		g->attrs = 1;
	} else if (g->attrs) {
		(void)fprintf(g->out, "%%TD*%%\n");
		g->attrs = 0;
	}
}

static void g_part(void *ctx, const char *name)
{
	g_attr(ctx, "C", name);
}

static void g_net(void *ctx, const char *name)
{
	g_attr(ctx, "N", name);
}

static void g_flash(void *ctx, struct plot_flash f)
{
	struct gerber_out *g = ctx;
	struct ap a = { f.rect, f.w, f.rect ? f.h : 0 };

	aperture(g, a);
	(void)fprintf(g->out, "X%ldY%ldD03*\n", mm(f.x), mm(f.y));
}

// A line of width 0, an outline, is drawn OUTLINE_W wide.
static void g_line(void *ctx, double w, const double *xy, size_t n)
{
	struct gerber_out *g = ctx;
	struct ap a = { 0, w > 0 ? w : OUTLINE_W, 0 };

	aperture(g, a);
	(void)fprintf(g->out, "X%ldY%ldD02*\n", mm(xy[0]), mm(xy[1]));
	for (size_t j = 1; j < n; j++) {
		(void)fprintf(g->out, "X%ldY%ldD01*\n", mm(xy[2 * j]),
			      mm(xy[(2 * j) + 1]));
	}
}

static void g_region(void *ctx, const double *xy, size_t n)
{
	struct gerber_out *g = ctx;

	(void)fprintf(g->out, "G36*\nX%ldY%ldD02*\n", mm(xy[0]), mm(xy[1]));
	for (size_t j = 1; j < n; j++) {
		(void)fprintf(g->out, "X%ldY%ldD01*\n", mm(xy[2 * j]),
			      mm(xy[(2 * j) + 1]));
	}
	(void)fprintf(g->out, "G37*\n");
}

// Adds drill size d to t, if new and there is room.
static void add_size(struct tools *t, double d)
{
	size_t k = 0;

	while (k < t->n && fabs(t->size[k] - d) > SAME) {
		k++;
	}
	if (k == t->n && t->n < TOOLS_MAX) {
		t->size[t->n++] = d;
	}
}

// The drill sizes of b: of the vias, then of the plated holes of each
// placed part.
static void drill_sizes(const struct pcb *b, struct tools *t)
{
	t->n = 0;
	for (size_t i = 0; i < b->nvia; i++) {
		add_size(t, b->via[i].drill);
	}
	for (size_t q = 0; q < b->npart; q++) {
		const struct pcb_part *p = &b->part[q];
		size_t nd = 0;

		for (size_t k = 0;
		     p->placed && k < p->fp->npad && nd < TOOLS_MAX; k++) {
			if (p->fp->pad[k].hole) {
				add_size(t, p->fp->pad[k].drill);
				nd++;
			}
		}
	}
}

// The holes of b of drill size d, into f.
static void drill_holes(FILE *f, const struct pcb *b, double d)
{
	for (size_t i = 0; i < b->nvia; i++) {
		if (fabs(b->via[i].drill - d) < SAME) {
			(void)fprintf(f, "X%.3fY%.3f\n", b->via[i].x,
				      b->via[i].y);
		}
	}
	for (size_t q = 0; q < b->npart; q++) {
		for (size_t k = 0; b->part[q].placed && k < b->part[q].fp->npad;
		     k++) {
			struct pcb_spot s;

			pcb_spot(b, &b->part[q], k, &s);
			if (s.layer < 0 && fabs(s.drill - d) < SAME) {
				(void)fprintf(f, "X%.3fY%.3f\n", s.x, s.y);
			}
		}
	}
}

// The Excellon file: a tool for each size, then its holes.
static void drill(const struct gerber_out *g)
{
	FILE *f = open_out(g, "drill.drl");
	struct tools t;

	drill_sizes(g->b, &t);
	(void)fprintf(f, "M48\nMETRIC,TZ\n");
	for (size_t k = 0; k < t.n; k++) {
		(void)fprintf(f, "T%zuC%.3f\n", k + 1, t.size[k]);
	}
	(void)fprintf(f, "%%\nG90\nG05\n");
	for (size_t k = 0; k < t.n; k++) {
		(void)fprintf(f, "T%zu\n", k + 1);
		drill_holes(f, g->b, t.size[k]);
	}
	(void)fprintf(f, "T0\nM30\n");
	close_out(g, f);
}

static const struct pcb_plot gerber = { g_begin, g_end,	   g_part,
					g_net,	 polarity, g_flash,
					g_line,	 g_region, NULL };

int main(int argc, char **argv)
{
	static const char usage[] =
		"usage: pcbgerber [-L LIB]... -o DIR board.pcb < board.lock";
	struct pcb b;
	struct gerber_out g = { &b, NULL, NULL, 0, NULL, 0 };
	struct pcb_lib lib = { NULL, 0, NULL, 0 };
	int i = 1;

	for (; i < argc && argv[i][0] == '-'; i++) {
		if (!strcmp(argv[i], "-L") && i + 1 < argc) {
			pcb_add_lib(&lib, argv[++i]);
		} else if (!strcmp(argv[i], "-o") && i + 1 < argc) {
			g.dir = argv[++i];
		} else {
			pcb_die("%s", usage);
		}
	}
	if (i != argc - 1 || !g.dir) {
		pcb_die("%s", usage);
	}
	memset(&b, 0, sizeof b);
	pcb_read(&b, argv[i], &lib, stdin, NULL);
	pcb_plot(&b, &gerber, &g);
	drill(&g);
	return b.failed;
}
