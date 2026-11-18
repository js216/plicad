// SPDX-License-Identifier: GPL-3.0-only
// pcbplot.c -- What each layer of a board draws, for pcbgerber and pcbvec
// Copyright (c) 2026 Jakob Kastelic

// See pcbplot.h.

#include "pcbplot.h"
#include "pcbsrc.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.14159265358979323846
#define PLANE_INSET 0.3 // How far a plane keeps in from the outline, in mm.
#define MASK_GROW 0.05 // How far the mask opens round a pad, in mm.
enum {
	RECT_PTS = 5, // The points of the outline of a rectangle, closed.
	FN_LEN = 300, // Room for the function of a copper layer.
	PIN_LEN = 1200 // Room for a pin, REF.PIN.
};

// A plot under way: of the board b, by the primitives p with ctx.
struct plotter {
	const struct pcb *b;
	const struct pcb_plot *p;
	void *ctx;
};

// A layer of the pads of the top: its file, how far each pad grows on
// each side, and whether it takes only pads of the top and only those
// that take paste.
struct pad_layer {
	struct plot_file f;
	double grow;
	int smd_only;
};

static const struct pad_layer top_mask = { { "topmask", "Soldermask,Top" },
					   MASK_GROW,
					   0 };
static const struct pad_layer top_paste = { { "toppaste", "Paste,Top" }, 0, 1 };

// Flashes pad s, grown by g on each side.
static void pad(const struct plotter *pl, const struct pcb_spot *s, double g)
{
	struct plot_flash f = { s->shape != PAD_CIRCLE, s->x, s->y,
				s->w + (2 * g), s->h + (2 * g) };

	pl->p->flash(pl->ctx, f);
}

// Flashes a circle of diameter d at the via v.
static void via_disk(const struct plotter *pl, const struct pcb_via *v,
		     double d)
{
	struct plot_flash f = { 0, v->x, v->y, d, 0 };

	pl->p->flash(pl->ctx, f);
}

static void track(const struct plotter *pl, const struct pcb_track *t, double w)
{
	pl->p->line(pl->ctx, w, t->xy, t->n);
}

// The outline of a rectangle with its corners rounded to radius r, the
// arcs as short chords (within 0.005 mm for 1 mm).
static void round_rect(const struct plotter *pl, double w, const double c[4],
		       double r)
{
	enum { ARC = 16 };
	double xy[2 * ((4 * (ARC + 1)) + 1)];
	const double cx[4] = { c[2] - r, c[2] - r, c[0] + r, c[0] + r };
	const double cy[4] = { c[1] + r, c[3] - r, c[3] - r, c[1] + r };
	size_t n = 0;

	for (int k = 0; k < 4; k++) {
		for (int i = 0; i <= ARC; i++) {
			double a = (k - 1 + ((double)i / ARC)) * PI / 2;

			xy[n++] = cx[k] + (r * cos(a));
			xy[n++] = cy[k] + (r * sin(a));
		}
	}
	xy[n++] = xy[0];
	xy[n++] = xy[1];
	pl->p->line(pl->ctx, w, xy, n / 2);
}

// The outline of the box c, x0 y0 x1 y1, as a closed polygon into xy.
static void box_outline(const double c[4], double xy[2 * RECT_PTS])
{
	const double pts[2 * RECT_PTS] = { c[0], c[1], c[2], c[1], c[2],
					   c[3], c[0], c[3], c[0], c[1] };

	memcpy(xy, pts, sizeof(pts));
}

static void rect(const struct plotter *pl, double w, const double c[4])
{
	double xy[2 * RECT_PTS];

	box_outline(c, xy);
	pl->p->line(pl->ctx, w, xy, RECT_PTS);
}

// The position of copper layer l in the terms of Gerber X2.
static const char *copper_pos(const struct pcb *b, int l)
{
	if (l == 0) {
		return "Top";
	}
	return l == (int)b->nlayer - 1 ? "Bot" : "Inr";
}

// Clears the plane of the net of layer l round the pads of other nets,
// and round its own pads that join it by a neck (a thermal relief).
static void plane_pads(const struct plotter *pl, int l, const char *net)
{
	const struct pcb *b = pl->b;

	for (size_t q = 0; q < b->npart; q++) {
		for (size_t k = 0; b->part[q].placed && k < b->part[q].fp->npad;
		     k++) {
			struct pcb_spot s;
			int n = pcb_net_of(b, q, k);

			pcb_spot(b, &b->part[q], k, &s);
			if ((s.layer == l || s.layer < 0) &&
			    (n < 0 || strcmp(b->net[n].name, net) != 0 ||
			     pcb_relieved(b, q, k))) {
				pad(pl, &s, b->clearance);
			}
		}
	}
}

// The plane of layer l, of the net: the outline PLANE_INSET in, cleared
// round the cutouts by as much and round what is not of its net.
static void plane(const struct plotter *pl, int l, const char *net)
{
	const struct pcb *b = pl->b;
	const struct pcb_plot *p = pl->p;
	const double inset[4] = { b->board[0] + PLANE_INSET,
				  b->board[1] + PLANE_INSET,
				  b->board[2] - PLANE_INSET,
				  b->board[3] - PLANE_INSET };
	double xy[2 * RECT_PTS];

	box_outline(inset, xy);
	p->net(pl->ctx, net);
	p->region(pl->ctx, xy, RECT_PTS);
	p->dark(pl->ctx, 0);
	// The cutouts cleared, the gap round them as round the board.
	for (size_t i = 0; i < b->ncut; i++) {
		const double *c = b->cut[i];
		const double grown[4] = { c[0] - PLANE_INSET,
					  c[1] - PLANE_INSET,
					  c[2] + PLANE_INSET,
					  c[3] + PLANE_INSET };
		double cx[2 * RECT_PTS];

		box_outline(grown, cx);
		p->region(pl->ctx, cx, RECT_PTS);
	}
	plane_pads(pl, l, net);
	for (size_t i = 0; i < b->nvia; i++) {
		const struct pcb_via *v = &b->via[i];

		if (strcmp(b->net[v->net].name, net) != 0) {
			via_disk(pl, v, v->dia + (2 * b->clearance));
		}
	}
	for (size_t i = 0; i < b->ntrack; i++) {
		if (b->track_[i].layer == l &&
		    strcmp(b->net[b->track_[i].net].name, net) != 0) {
			track(pl, &b->track_[i],
			      b->track_[i].w + (2 * b->clearance));
		}
	}
	p->net(pl->ctx, NULL);
	p->dark(pl->ctx, 1);
}

// Names pad k of part p at s, for a writer that names pins.
static void pad_pin(const struct plotter *pl, const struct pcb_part *p,
		    size_t k, const struct pcb_spot *s)
{
	char pin[PIN_LEN];
	int len = snprintf(pin, sizeof pin, "%s.%s", pcb_ref(p),
			   p->fp->pad[k].name);
	struct plot_pin at = { pin, s->x, s->y };

	if (len < 0 || (size_t)len >= sizeof pin) {
		pcb_die("%s: the name of a pin is too long", pcb_ref(p));
	}
	pl->p->pin(pl->ctx, at);
}

// The pads of layer l, each with its part and net.
static void layer_pads(const struct plotter *pl, int l)
{
	const struct pcb *b = pl->b;
	const struct pcb_plot *p = pl->p;

	for (size_t q = 0; q < b->npart; q++) {
		const struct pcb_part *pt = &b->part[q];

		for (size_t k = 0; pt->placed && pt->fp && k < pt->fp->npad;
		     k++) {
			struct pcb_spot s;
			int n = pcb_net_of(b, q, k);

			pcb_spot(b, pt, k, &s);
			if (s.layer != l && s.layer >= 0) {
				continue;
			}
			p->part(pl->ctx, pt->name);
			p->net(pl->ctx, n >= 0 ? b->net[n].name : NULL);
			pad(pl, &s, 0);
			if (p->pin) {
				pad_pin(pl, pt, k, &s);
			}
			p->net(pl->ctx, NULL);
			p->part(pl->ctx, NULL);
		}
	}
}

// The copper of layer l: on a plane, the plane cleared round what is not
// of its net, and round its own pads that join it by a neck; then the
// pads, tracks and vias.
static void copper(const struct plotter *pl, int l)
{
	const struct pcb *b = pl->b;
	const struct pcb_plot *p = pl->p;
	const struct pcb_layer *ly = &b->layer[l];
	char fn[FN_LEN];

	(void)snprintf(fn, sizeof fn, "Copper,L%d,%s", l + 1, copper_pos(b, l));
	p->begin(pl->ctx, (struct plot_file){ ly->name, fn });
	if (ly->kind == LAYER_PLANE) {
		plane(pl, l, ly->net ? ly->net : "");
	}
	layer_pads(pl, l);
	for (size_t i = 0; i < b->ntrack; i++) {
		const struct pcb_track *t = &b->track_[i];

		if (t->layer == l) {
			p->net(pl->ctx, b->net[t->net].name);
			track(pl, t, t->w);
			p->net(pl->ctx, NULL);
		}
	}
	for (size_t i = 0; i < b->nvia; i++) {
		const struct pcb_via *v = &b->via[i];

		p->net(pl->ctx, b->net[v->net].name);
		via_disk(pl, v, v->dia);
		p->net(pl->ctx, NULL);
	}
	p->end(pl->ctx);
}

// The solder mask or paste of the top, of layer t: its pads.
static void top_pads(const struct plotter *pl, const struct pad_layer *t)
{
	const struct pcb *b = pl->b;
	const struct pcb_plot *p = pl->p;

	p->begin(pl->ctx, t->f);
	for (size_t q = 0; q < b->npart; q++) {
		const struct pcb_part *pt = &b->part[q];

		for (size_t k = 0; pt->placed && pt->fp && k < pt->fp->npad;
		     k++) {
			struct pcb_spot s;

			pcb_spot(b, pt, k, &s);
			if ((s.layer == 0 || (s.layer < 0 && !t->smd_only)) &&
			    !(t->smd_only && pt->fp->pad[k].nopaste)) {
				p->part(pl->ctx, pt->name);
				pad(pl, &s, t->grow);
				p->part(pl->ctx, NULL);
			}
		}
	}
	p->end(pl->ctx);
}

// The outlines of the courtyards of one side, a drawing only: no maker
// needs them, but a viewer shows what keeps the parts apart.
static void courtyards(const struct plotter *pl, int bottom)
{
	const struct pcb *b = pl->b;
	const struct pcb_plot *p = pl->p;
	struct plot_file f = { bottom ? "bottomcourt" : "topcourt",
			       bottom ? "Other,Courtyard,Bot" :
					"Other,Courtyard,Top" };

	p->begin(pl->ctx, f);
	for (size_t q = 0; q < b->npart; q++) {
		const struct pcb_part *pt = &b->part[q];

		if (pt->placed && pt->bottom == bottom) {
			double c[4];

			pcb_court(pt, c);
			p->part(pl->ctx, pt->name);
			rect(pl, 0, c);
			p->part(pl->ctx, NULL);
		}
	}
	p->end(pl->ctx);
}

// The silkscreen of one side, cut where the mask opens.
static void silk(const struct plotter *pl, int bottom)
{
	const struct pcb *b = pl->b;
	const struct pcb_plot *p = pl->p;
	struct pcb_strokes ss = pcb_silk(b);
	const struct pcb_stroke *s = ss.s;
	size_t n = ss.n;
	struct plot_file f = { bottom ? "bottomsilk" : "topsilk",
			       bottom ? "Legend,Bot" : "Legend,Top" };

	p->begin(pl->ctx, f);
	for (size_t i = 0; i < n; i++) {
		if (s[i].bottom == bottom) {
			const double xy[4] = { s[i].x0, s[i].y0, s[i].x1,
					       s[i].y1 };

			p->part(pl->ctx, s[i].part < b->npart ?
						 b->part[s[i].part].name :
						 NULL);
			p->line(pl->ctx, s[i].w, xy, 2);
			p->part(pl->ctx, NULL);
		}
	}
	p->dark(pl->ctx, 0);
	for (size_t q = 0; q < b->npart; q++) {
		for (size_t k = 0; b->part[q].placed && k < b->part[q].fp->npad;
		     k++) {
			double o[4];

			struct pcb_end at = { q, k };

			if (pcb_opening(b, at, bottom, o)) {
				struct plot_flash fl = { 1, (o[0] + o[2]) / 2,
							 (o[1] + o[3]) / 2,
							 o[2] - o[0],
							 o[3] - o[1] };

				p->flash(pl->ctx, fl);
			}
		}
	}
	p->dark(pl->ctx, 1);
	free(ss.s);
	p->end(pl->ctx);
}

void pcb_plot(const struct pcb *b, const struct pcb_plot *p, void *ctx)
{
	struct plotter pl = { b, p, ctx };

	for (size_t l = 0; l < b->nlayer; l++) {
		copper(&pl, (int)l);
	}
	top_pads(&pl, &top_mask);
	top_pads(&pl, &top_paste);
	p->begin(ctx, (struct plot_file){ "edge", "Profile,NP" });
	rect(&pl, 0, b->board);
	for (size_t i = 0; i < b->ncut; i++) {
		round_rect(&pl, 0, b->cut[i], b->cutr[i]);
	}
	p->end(ctx);
	courtyards(&pl, 0);
	courtyards(&pl, 1);
	silk(&pl, 0);
	silk(&pl, 1);
}
