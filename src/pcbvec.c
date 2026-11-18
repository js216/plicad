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
#include "pcbsrc.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { DECIMAL = 10, LAYER_LEN = 64 };
#define FAR 1e300 // Beyond any box, for a box of nothing yet.

// The vectors under way: of the board b, into the directory dir, the file
// out; its polarity, whether a zone is open, the part and the net set
// (their brackets maybe not open), whether they are open, and the boxes
// of the parts in the file, by part.
struct vec_out {
	const struct pcb *b;
	const char *dir;
	FILE *out;
	int dark;
	int in_zone;
	const char *part;
	const char *net;
	int part_open;
	int net_open;
	double (*box)[4];
};

// Whether s starts with prefix.
static int starts(const char *s, const char *prefix)
{
	return strncmp(s, prefix, strlen(prefix)) == 0;
}

// KiCad's name of the layer of a file function of Gerber X2: F.Cu,
// In1.Cu, ..., B.Cu, F.Mask, F.Paste, F.Silkscreen, F.Courtyard (B. on
// the bottom), Edge.Cuts, and Drill for the holes.
static void layer_name(const char *function, char *name, size_t size)
{
	static const char copper[] = "Copper,L";
	const char *side = strstr(function, ",Bot") ? "B" : "F";
	const char *num = function + sizeof(copper) - 1;
	char *end = NULL;
	long n = starts(function, copper) ? strtol(num, &end, DECIMAL) : 0;

	if (end && end != num) {
		if (strstr(function, ",Top")) {
			(void)snprintf(name, size, "F.Cu");
		} else if (strstr(function, ",Bot")) {
			(void)snprintf(name, size, "B.Cu");
		} else {
			(void)snprintf(name, size, "In%d.Cu", (int)n - 1);
		}
	} else if (starts(function, "Soldermask,")) {
		(void)snprintf(name, size, "%s.Mask", side);
	} else if (starts(function, "Paste,")) {
		(void)snprintf(name, size, "%s.Paste", side);
	} else if (starts(function, "Legend,")) {
		(void)snprintf(name, size, "%s.Silkscreen", side);
	} else if (strstr(function, "Courtyard")) {
		(void)snprintf(name, size, "%s.Courtyard", side);
	} else if (starts(function, "Profile")) {
		(void)snprintf(name, size, "Edge.Cuts");
	} else {
		(void)snprintf(name, size, "Drill");
	}
}

static void v_begin(void *ctx, struct plot_file f)
{
	struct vec_out *v = ctx;
	const struct pcb *b = v->b;
	size_t room = strlen(v->dir) + strlen(f.name) + sizeof("/.vec");
	char *p = pcb_alloc(room);
	char layer[LAYER_LEN];

	(void)snprintf(p, room, "%s/%s.vec", v->dir, f.name);
	v->out = fopen(p, "w");
	if (!v->out) {
		pcb_die("%s: cannot write", p);
	}
	free(p);
	layer_name(f.function, layer, sizeof layer);
	(void)fprintf(v->out, "# vec of %s, by pcbvec\nlayer %s\n", f.name,
		      layer);
	v->dark = 1;
	v->in_zone = v->part_open = v->net_open = 0;
	v->part = v->net = NULL;
	free((void *)v->box);
	v->box = pcb_alloc((b->npart + 1) * sizeof *v->box);
	for (size_t q = 0; q < b->npart; q++) {
		v->box[q][0] = v->box[q][1] = FAR;
		v->box[q][2] = v->box[q][3] = -FAR;
	}
}

// A part by its name, to sort.
struct named {
	const char *name;
	size_t q;
};

static int by_name(const void *lhs, const void *rhs)
{
	const struct named *x = lhs;
	const struct named *y = rhs;
	int c = strcmp(x->name, y->name);

	if (c) {
		return c;
	}
	return (x->q > y->q) - (x->q < y->q);
}

static void v_end(void *ctx)
{
	struct vec_out *v = ctx;
	const struct pcb *b = v->b;
	struct named *order = pcb_alloc((b->npart + 1) * sizeof *order);
	int failed = 0;

	for (size_t q = 0; q < b->npart; q++) {
		order[q].name = b->part[q].name;
		order[q].q = q;
	}
	qsort(order, b->npart, sizeof *order, by_name);
	for (size_t i = 0; i < b->npart; i++) {
		const double *bx = v->box[order[i].q];

		if (bx[2] >= bx[0]) {
			(void)fprintf(v->out, "name %s %.4f %.4f %.4f %.4f\n",
				      order[i].name, bx[0], bx[1], bx[2],
				      bx[3]);
		}
	}
	free(order);
	failed = ferror(v->out);
	if (fclose(v->out) || failed) {
		pcb_die("%s: cannot write", v->dir);
	}
}

// Opens the bracket of the part set, if not open.
static void open_part(struct vec_out *v)
{
	const struct pcb *b = v->b;
	const char *ref = v->part;

	if (!v->part || v->part_open) {
		return;
	}
	for (size_t q = 0; q < b->npart; q++) {
		if (!strcmp(b->part[q].name, v->part)) {
			ref = pcb_ref(&b->part[q]);
		}
	}
	(void)fprintf(v->out, "part %s %s\n", v->part, ref);
	v->part_open = 1;
}

// Opens the bracket of the net set, if not open and not in a zone.
static void open_net(struct vec_out *v)
{
	const struct pcb *b = v->b;
	const char *c = NULL;

	if (!v->net || v->net_open || v->in_zone) {
		return;
	}
	// The class the netlist gives the net, for the viewer.
	for (size_t k = 0; k < b->nnet; k++) {
		if (!strcmp(b->net[k].name, v->net)) {
			c = b->net[k].klass;
		}
	}
	(void)fprintf(v->out, "net %s%s%s\n", v->net, c ? " class " : "",
		      c ? c : "");
	v->net_open = 1;
}

// Takes the n points xy, grown by r, into the box of the part set.
static void grow_box(struct vec_out *v, double r, const double *xy, size_t n)
{
	const struct pcb *b = v->b;

	for (size_t q = 0; v->part && q < b->npart; q++) {
		if (b->part[q].name == v->part ||
		    !strcmp(b->part[q].name, v->part)) {
			double *bx = v->box[q];

			for (size_t i = 0; i < n; i++) {
				bx[0] = fmin(bx[0], xy[2 * i] - r);
				bx[1] = fmin(bx[1], xy[(2 * i) + 1] - r);
				bx[2] = fmax(bx[2], xy[2 * i] + r);
				bx[3] = fmax(bx[3], xy[(2 * i) + 1] + r);
			}
			break;
		}
	}
}

// Opens the brackets of what is set before a drawing, and takes its
// n points xy, grown by r, into the box of its part.
static void open_brackets(struct vec_out *v, double r, const double *xy,
			  size_t n)
{
	open_part(v);
	open_net(v);
	grow_box(v, r, xy, n);
}

static void v_part(void *ctx, const char *name)
{
	struct vec_out *v = ctx;

	if (v->part_open && (!name || strcmp(name, v->part) != 0)) {
		(void)fprintf(v->out, "part\n");
		v->part_open = 0;
	}
	v->part = name;
}

static void v_net(void *ctx, const char *name)
{
	struct vec_out *v = ctx;

	if (!name && v->in_zone) {
		(void)fprintf(v->out, "end\n");
		v->in_zone = 0;
	}
	if (v->net_open && (!name || strcmp(name, v->net) != 0)) {
		(void)fprintf(v->out, "net\n");
		v->net_open = 0;
	}
	v->net = name;
}

static void v_dark(void *ctx, int d)
{
	struct vec_out *v = ctx;

	if (d != v->dark) {
		(void)fprintf(v->out, d ? "dark\n" : "clear\n");
		v->dark = d;
	}
}

static void points(const struct vec_out *v, const double *xy, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		(void)fprintf(v->out, " %.4f %.4f", xy[2 * i], xy[(2 * i) + 1]);
	}
	(void)fprintf(v->out, "\n");
}

static void v_flash(void *ctx, struct plot_flash f)
{
	struct vec_out *v = ctx;
	const double x = f.x;
	const double y = f.y;
	const double w = f.w;
	const double h = f.h;
	const double xy[8] = { x - (w / 2), y - (h / 2), x + (w / 2),
			       y - (h / 2), x + (w / 2), y + (h / 2),
			       x - (w / 2), y + (h / 2) };

	if (!f.rect) {
		const double c[2] = { x, y };

		open_brackets(v, w / 2, c, 1);
		(void)fprintf(v->out, "disk %.4f %.4f %.4f\n", x, y, w / 2);
		return;
	}
	open_brackets(v, 0, xy, 4);
	(void)fprintf(v->out, "poly");
	points(v, xy, 4);
}

static void v_line(void *ctx, double w, const double *xy, size_t n)
{
	struct vec_out *v = ctx;

	open_brackets(v, w / 2, xy, n);
	(void)fprintf(v->out, "line %.4f", w);
	points(v, xy, n);
}

// A region under a net is a plane: a zone of its net.
static void v_region(void *ctx, const double *xy, size_t n)
{
	struct vec_out *v = ctx;

	if (v->net && !v->in_zone) {
		(void)fprintf(v->out, "zone %s\n", v->net);
		v->in_zone = 1;
	}
	open_brackets(v, 0, xy, n);
	(void)fprintf(v->out, "poly");
	points(v, xy, n);
}

// Where a pad is, by its pin: for the numbers a viewer shows on pads.
static void v_pin(void *ctx, struct plot_pin p)
{
	struct vec_out *v = ctx;
	const double c[2] = { p.x, p.y };

	open_brackets(v, 0, c, 1);
	(void)fprintf(v->out, "pin %s %.4f %.4f\n", p.name, p.x, p.y);
}

static const struct pcb_plot vec = { v_begin, v_end,  v_part,	v_net, v_dark,
				     v_flash, v_line, v_region, v_pin };

// The holes: the vias' and the parts' plated holes, as disks.
static void holes(struct vec_out *v)
{
	const struct pcb *b = v->b;

	v_begin(v, (struct plot_file){ "drill", "Plated,1,2,PTH" });
	for (size_t q = 0; q < b->npart; q++) {
		const struct pcb_part *p = &b->part[q];

		for (size_t k = 0; p->placed && p->fp && k < p->fp->npad; k++) {
			struct pcb_spot s;

			pcb_spot(b, p, k, &s);
			if (s.layer < 0) {
				v_part(v, p->name);
				v_flash(v, (struct plot_flash){ 0, s.x, s.y,
								s.drill, 0 });
			}
		}
	}
	v_part(v, NULL);
	for (size_t i = 0; i < b->nvia; i++) {
		v_flash(v, (struct plot_flash){ 0, b->via[i].x, b->via[i].y,
						b->via[i].drill, 0 });
	}
	v_end(v);
}

int main(int argc, char **argv)
{
	static const char usage[] =
		"usage: pcbvec [-L LIB]... -o DIR board.pcb < board.lock";
	struct pcb b;
	struct vec_out v = { &b, NULL, NULL, 1, 0, NULL, NULL, 0, 0, NULL };
	struct pcb_lib lib = { NULL, 0, NULL, 0 };
	int i = 1;

	for (; i < argc && argv[i][0] == '-'; i++) {
		if (!strcmp(argv[i], "-L") && i + 1 < argc) {
			pcb_add_lib(&lib, argv[++i]);
		} else if (!strcmp(argv[i], "-o") && i + 1 < argc) {
			v.dir = argv[++i];
		} else {
			pcb_die("%s", usage);
		}
	}
	if (i != argc - 1 || !v.dir) {
		pcb_die("%s", usage);
	}
	memset(&b, 0, sizeof b);
	pcb_read(&b, argv[i], &lib, stdin, NULL);
	pcb_plot(&b, &vec, &v);
	holes(&v);
	return b.failed;
}
