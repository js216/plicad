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
#include <math.h>
#include <string.h>

static struct pcb B;
static const char *dir;

// The apertures of a file: their sizes and shapes.
struct ap {
	int shape; // 0 a circle, 1 a rectangle.
	double w, h;
};

static struct ap *aps;
static size_t naps;
static FILE *out;

static FILE *open_out(const char *name)
{
	char *p = pcb_alloc(strlen(dir) + strlen(name) + 2);
	FILE *f;

	sprintf(p, "%s/%s", dir, name);
	f = fopen(p, "w");
	if (!f) {
		pcb_die("%s: cannot write", p);
	}
	free(p);
	return f;
}

static void begin(const char *name, const char *function)
{
	out = open_out(name);
	free(aps);
	aps = NULL;
	naps = 0;
	fprintf(out, "%%TF.GenerationSoftware,PliCad,pcbgerber*%%\n");
	fprintf(out, "%%TF.FileFunction,%s*%%\n", function);
	fprintf(out, "%%FSLAX46Y46*%%\n%%MOMM*%%\n%%LPD*%%\nG01*\n");
}

static void end(void)
{
	fprintf(out, "M02*\n");
	if (fclose(out)) {
		pcb_die("%s: cannot write", dir);
	}
}

static long mm(double v)
{
	return lround(v * 1e6);
}

// Selects the aperture of a shape and size, defining it the first time.
static void aperture(int shape, double w, double h)
{
	size_t i;

	for (i = 0; aps && i < naps; i++) {
		if (aps[i].shape == shape && fabs(aps[i].w - w) < 1e-9 &&
		    fabs(aps[i].h - h) < 1e-9) {
			break;
		}
	}
	if (i == naps) {
		aps = pcb_grow(aps, naps, sizeof *aps);
		aps[naps].shape = shape;
		aps[naps].w = w;
		aps[naps++].h = h;
		if (shape) {
			fprintf(out, "%%ADD%zuR,%.6fX%.6f*%%\n", i + 10, w, h);
		} else {
			fprintf(out, "%%ADD%zuC,%.6f*%%\n", i + 10, w);
		}
	}
	fprintf(out, "D%zu*\n", i + 10);
}

static void flash(double x, double y)
{
	fprintf(out, "X%ldY%ldD03*\n", mm(x), mm(y));
}

static void polarity(int dark)
{
	fprintf(out, "%%LP%c*%%\n", dark ? 'D' : 'C');
}

static int attrs; // Object attributes are set.

static void g_begin(const char *name, const char *function)
{
	char file[300];

	snprintf(file, sizeof file, "%s.gbr", name);
	begin(file, function);
	attrs = 0;
}

static void g_attr(const char *key, const char *value)
{
	if (value) {
		fprintf(out, "%%TO.%s,%s*%%\n", key, value);
		attrs = 1;
	} else if (attrs) {
		fprintf(out, "%%TD*%%\n");
		attrs = 0;
	}
}

static void g_part(const char *name)
{
	g_attr("C", name);
}

static void g_net(const char *name)
{
	g_attr("N", name);
}

static void g_flash(int rect, double x, double y, double w, double h)
{
	aperture(rect, w, rect ? h : 0);
	flash(x, y);
}

// A line of width 0, an outline, is drawn 0.05 mm wide.
static void g_line(double w, const double *xy, size_t n)
{
	aperture(0, w > 0 ? w : 0.05, 0);
	fprintf(out, "X%ldY%ldD02*\n", mm(xy[0]), mm(xy[1]));
	for (size_t j = 1; j < n; j++) {
		fprintf(out, "X%ldY%ldD01*\n", mm(xy[2 * j]),
			mm(xy[2 * j + 1]));
	}
}

static void g_region(const double *xy, size_t n)
{
	fprintf(out, "G36*\nX%ldY%ldD02*\n", mm(xy[0]), mm(xy[1]));
	for (size_t j = 1; j < n; j++) {
		fprintf(out, "X%ldY%ldD01*\n", mm(xy[2 * j]),
			mm(xy[2 * j + 1]));
	}
	fprintf(out, "G37*\n");
}

// The Excellon file: a tool for each size, then its holes.
static void drill(void)
{
	FILE *f = open_out("drill.drl");
	double sizes[64];
	size_t ns = 0;

	for (int pass = 0; pass < 2; pass++) {
		if (pass) {
			fprintf(f, "M48\nMETRIC,TZ\n");
			for (size_t t = 0; t < ns; t++) {
				fprintf(f, "T%zuC%.3f\n", t + 1, sizes[t]);
			}
			fprintf(f, "%%\nG90\nG05\n");
		}
		for (size_t t = 0; pass && t < ns; t++) {
			fprintf(f, "T%zu\n", t + 1);
			for (size_t i = 0; i < B.nvia; i++) {
				if (fabs(B.via[i].drill - sizes[t]) < 1e-9) {
					fprintf(f, "X%.3fY%.3f\n", B.via[i].x,
						B.via[i].y);
				}
			}
			for (size_t q = 0; q < B.npart; q++) {
				for (size_t k = 0;
				     B.part[q].placed && k < B.part[q].fp->npad;
				     k++) {
					struct pcb_spot s;

					pcb_spot(&B, &B.part[q], k, &s);
					if (s.layer < 0 &&
					    fabs(s.drill - sizes[t]) < 1e-9) {
						fprintf(f, "X%.3fY%.3f\n", s.x,
							s.y);
					}
				}
			}
		}
		for (size_t i = 0; !pass && i < B.nvia + B.npart; i++) {
			double d[64];
			size_t nd = 0;

			if (i < B.nvia) {
				d[nd++] = B.via[i].drill;
			} else {
				const struct pcb_part *p = &B.part[i - B.nvia];

				for (size_t k = 0;
				     p->placed && k < p->fp->npad && nd < 64;
				     k++) {
					if (p->fp->pad[k].hole) {
						d[nd++] = p->fp->pad[k].drill;
					}
				}
			}
			for (size_t j = 0; j < nd; j++) {
				size_t t = 0;

				while (t < ns && fabs(sizes[t] - d[j]) > 1e-9) {
					t++;
				}
				if (t == ns && ns < 64) {
					sizes[ns++] = d[j];
				}
			}
		}
	}
	fprintf(f, "T0\nM30\n");
	if (fclose(f)) {
		pcb_die("%s: cannot write", dir);
	}
}

static const struct pcb_plot gerber = { g_begin,  end,	   g_part, g_net,
					polarity, g_flash, g_line, g_region,
					NULL };

int main(int argc, char **argv)
{
	char **lib = NULL;
	size_t nlib = 0;
	int i = 1;

	for (; i < argc && argv[i][0] == '-'; i++) {
		if (!strcmp(argv[i], "-L") && i + 1 < argc) {
			lib = pcb_grow(lib, nlib, sizeof *lib);
			lib[nlib] = pcb_alloc(strlen(argv[i + 1]) + 16);
			sprintf(lib[nlib++], "%s/footprints", argv[++i]);
		} else if (!strcmp(argv[i], "-o") && i + 1 < argc) {
			dir = argv[++i];
		} else {
			pcb_die("usage: pcbgerber [-L LIB]... -o DIR board.pcb < board.lock");
		}
	}
	if (i != argc - 1 || !dir) {
		pcb_die("usage: pcbgerber [-L LIB]... -o DIR board.pcb < board.lock");
	}
	pcb_read(&B, argv[i], lib, nlib, stdin, NULL);
	pcb_plot(&B, &gerber);
	drill();
	return B.failed;
}
