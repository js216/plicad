// SPDX-License-Identifier: GPL-3.0-only
// pcbvec.c -- Write the layers of a board as plain vectors for a viewer
// Copyright (c) 2026 Jakob Kastelic

// Usage: pcbvec [-L LIB]... -o DIR board.pcb < board.lock
//
// Reads the source (pcb(5)), its netlist and footprints, and the lock,
// and writes into DIR a vec(5) file for each layer that pcbgerber writes
// a Gerber for, named alike (top.vec, ..., topsilk.vec), and drill.vec,
// the holes as disks; what each layer draws is pcbplot's.  Each file
// says first what it is, by KiCad's name of the layer (layer F.Cu); a
// part's records are bracketed by part, a net's by net (with its class,
// if the netlist gives it one), and a plane is a zone of its net; each part's box is given by name, so pcbview names
// and selects them.

#include "pcbplot.h"
#include <math.h>
#include <string.h>

static struct pcb B;
static const char *dir;
static FILE *out;
static int dark = 1, in_zone;
static const char *part, *net; // Set, and their brackets maybe not open.
static int part_open, net_open;

// The boxes of the parts in the file being written, by part.
static double (*box)[4];

// KiCad's name of the layer of a file function of Gerber X2: F.Cu,
// In1.Cu, ..., B.Cu, F.Mask, F.Paste, F.Silkscreen, F.Courtyard (B. on
// the bottom), Edge.Cuts, and Drill for the holes.
static void layer_name(const char *function, char *name, size_t size)
{
	int n = 0;
	const char *side = strstr(function, ",Bot") ? "B" : "F";

	if (sscanf(function, "Copper,L%d", &n) == 1) {
		if (strstr(function, ",Top")) {
			snprintf(name, size, "F.Cu");
		} else if (strstr(function, ",Bot")) {
			snprintf(name, size, "B.Cu");
		} else {
			snprintf(name, size, "In%d.Cu", n - 1);
		}
	} else if (!strncmp(function, "Soldermask,", 11)) {
		snprintf(name, size, "%s.Mask", side);
	} else if (!strncmp(function, "Paste,", 6)) {
		snprintf(name, size, "%s.Paste", side);
	} else if (!strncmp(function, "Legend,", 7)) {
		snprintf(name, size, "%s.Silkscreen", side);
	} else if (strstr(function, "Courtyard")) {
		snprintf(name, size, "%s.Courtyard", side);
	} else if (!strncmp(function, "Profile", 7)) {
		snprintf(name, size, "Edge.Cuts");
	} else {
		snprintf(name, size, "Drill");
	}
}

static void v_begin(const char *name, const char *function)
{
	char *p = pcb_alloc(strlen(dir) + strlen(name) + 6), layer[64];

	sprintf(p, "%s/%s.vec", dir, name);
	out = fopen(p, "w");
	if (!out) {
		pcb_die("%s: cannot write", p);
	}
	free(p);
	layer_name(function, layer, sizeof layer);
	fprintf(out, "# vec of %s, by pcbvec\nlayer %s\n", name, layer);
	dark = 1;
	in_zone = part_open = net_open = 0;
	part = net = NULL;
	free(box);
	box = pcb_alloc((B.npart + 1) * sizeof *box);
	for (size_t q = 0; q < B.npart; q++) {
		box[q][0] = box[q][1] = 1e300;
		box[q][2] = box[q][3] = -1e300;
	}
}

static int by_name(const void *a, const void *b)
{
	return strcmp(B.part[*(const size_t *)a].name,
		      B.part[*(const size_t *)b].name);
}

static void v_end(void)
{
	size_t *order = pcb_alloc((B.npart + 1) * sizeof *order);

	for (size_t q = 0; q < B.npart; q++) {
		order[q] = q;
	}
	qsort(order, B.npart, sizeof *order, by_name);
	for (size_t i = 0; i < B.npart; i++) {
		const double *b = box[order[i]];

		if (b[2] >= b[0]) {
			fprintf(out, "name %s %.4f %.4f %.4f %.4f\n",
				B.part[order[i]].name, b[0], b[1], b[2], b[3]);
		}
	}
	free(order);
	if (fclose(out)) {
		pcb_die("%s: cannot write", dir);
	}
}

// Opens the brackets of what is set before a drawing, and takes its
// points into the box of its part.
static void open_brackets(const double *xy, size_t n, double r)
{
	if (part && !part_open) {
		const char *ref = part;

		for (size_t q = 0; q < B.npart; q++) {
			if (!strcmp(B.part[q].name, part)) {
				ref = pcb_ref(&B.part[q]);
			}
		}
		fprintf(out, "part %s %s\n", part, ref);
		part_open = 1;
	}
	if (net && !net_open && !in_zone) {
		const char *c = NULL;

		// The class the netlist gives the net, for the viewer.
		for (size_t k = 0; k < B.nnet; k++) {
			if (!strcmp(B.net[k].name, net)) {
				c = B.net[k].klass;
			}
		}
		fprintf(out, "net %s%s%s\n", net, c ? " class " : "",
			c ? c : "");
		net_open = 1;
	}
	for (size_t q = 0; part && q < B.npart; q++) {
		if (B.part[q].name == part || !strcmp(B.part[q].name, part)) {
			for (size_t i = 0; i < n; i++) {
				box[q][0] = fmin(box[q][0], xy[2 * i] - r);
				box[q][1] = fmin(box[q][1], xy[2 * i + 1] - r);
				box[q][2] = fmax(box[q][2], xy[2 * i] + r);
				box[q][3] = fmax(box[q][3], xy[2 * i + 1] + r);
			}
			break;
		}
	}
}

static void v_part(const char *name)
{
	if (part_open && (!name || strcmp(name, part))) {
		fprintf(out, "part\n");
		part_open = 0;
	}
	part = name;
}

static void v_net(const char *name)
{
	if (!name && in_zone) {
		fprintf(out, "end\n");
		in_zone = 0;
	}
	if (net_open && (!name || strcmp(name, net))) {
		fprintf(out, "net\n");
		net_open = 0;
	}
	net = name;
}

static void v_dark(int d)
{
	if (d != dark) {
		fprintf(out, d ? "dark\n" : "clear\n");
		dark = d;
	}
}

static void points(const double *xy, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		fprintf(out, " %.4f %.4f", xy[2 * i], xy[2 * i + 1]);
	}
	fprintf(out, "\n");
}

static void v_flash(int rect, double x, double y, double w, double h)
{
	const double xy[8] = { x - w / 2, y - h / 2, x + w / 2, y - h / 2,
			       x + w / 2, y + h / 2, x - w / 2, y + h / 2 };

	if (!rect) {
		const double c[2] = { x, y };

		open_brackets(c, 1, w / 2);
		fprintf(out, "disk %.4f %.4f %.4f\n", x, y, w / 2);
		return;
	}
	open_brackets(xy, 4, 0);
	fprintf(out, "poly");
	points(xy, 4);
}

static void v_line(double w, const double *xy, size_t n)
{
	open_brackets(xy, n, w / 2);
	fprintf(out, "line %.4f", w);
	points(xy, n);
}

// A region under a net is a plane: a zone of its net.
static void v_region(const double *xy, size_t n)
{
	if (net && !in_zone) {
		fprintf(out, "zone %s\n", net);
		in_zone = 1;
	}
	open_brackets(xy, n, 0);
	fprintf(out, "poly");
	points(xy, n);
}

// Where a pad is, by its pin: for the numbers a viewer shows on pads.
static void v_pin(const char *name, double x, double y)
{
	const double c[2] = { x, y };

	open_brackets(c, 1, 0);
	fprintf(out, "pin %s %.4f %.4f\n", name, x, y);
}

static const struct pcb_plot vec = { v_begin, v_end,  v_part,	v_net, v_dark,
				     v_flash, v_line, v_region, v_pin };

// The holes: the vias' and the parts' plated holes, as disks.
static void holes(void)
{
	v_begin("drill", "Plated,1,2,PTH");
	for (size_t q = 0; q < B.npart; q++) {
		const struct pcb_part *p = &B.part[q];

		for (size_t k = 0; p->placed && p->fp && k < p->fp->npad; k++) {
			struct pcb_spot s;

			pcb_spot(&B, p, k, &s);
			if (s.layer < 0) {
				v_part(p->name);
				v_flash(0, s.x, s.y, s.drill, 0);
			}
		}
	}
	v_part(NULL);
	for (size_t i = 0; i < B.nvia; i++) {
		v_flash(0, B.via[i].x, B.via[i].y, B.via[i].drill, 0);
	}
	v_end();
}

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
			pcb_die("usage: pcbvec [-L LIB]... -o DIR board.pcb < board.lock");
		}
	}
	if (i != argc - 1 || !dir) {
		pcb_die("usage: pcbvec [-L LIB]... -o DIR board.pcb < board.lock");
	}
	pcb_read(&B, argv[i], lib, nlib, stdin, NULL);
	pcb_plot(&B, &vec);
	holes();
	return B.failed;
}
