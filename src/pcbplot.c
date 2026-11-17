// SPDX-License-Identifier: GPL-3.0-only
// pcbplot.c -- What each layer of a board draws, for pcbgerber and pcbvec
// Copyright (c) 2026 Jakob Kastelic

// See pcbplot.h.

#include "pcbplot.h"
#include <math.h>
#include <string.h>

static const struct pcb *B;
static const struct pcb_plot *P;

// Flashes pad s, grown by g on each side.
static void pad(const struct pcb_spot *s, double g)
{
	P->flash(s->shape != PAD_CIRCLE, s->x, s->y, s->w + 2 * g,
		 s->h + 2 * g);
}

static void track(const struct pcb_track *t, double w)
{
	P->line(w, t->xy, t->n);
}

// The outline of a rectangle with its corners rounded to radius r, the
// arcs as short chords (within 0.005 mm for 1 mm).
static void round_rect(double w, const double c[4], double r)
{
	enum { ARC = 16 };
	double xy[2 * (4 * (ARC + 1) + 1)];
	const double cx[4] = { c[2] - r, c[2] - r, c[0] + r, c[0] + r };
	const double cy[4] = { c[1] + r, c[3] - r, c[3] - r, c[1] + r };
	size_t n = 0;

	for (int k = 0; k < 4; k++) {
		for (int i = 0; i <= ARC; i++) {
			double a = (k - 1 + (double)i / ARC) *
				   3.14159265358979323846 / 2;

			xy[n++] = cx[k] + r * cos(a);
			xy[n++] = cy[k] + r * sin(a);
		}
	}
	xy[n++] = xy[0];
	xy[n++] = xy[1];
	P->line(w, xy, n / 2);
}

static void rect(double w, const double c[4])
{
	double xy[10] = { c[0], c[1], c[2], c[1], c[2],
			  c[3], c[0], c[3], c[0], c[1] };

	P->line(w, xy, 5);
}

// The copper of layer l: on a plane, the plane cleared round what is not
// of its net, and round its own pads that join it by a neck; then the
// pads, tracks and vias.
static void copper(int l)
{
	const struct pcb_layer *ly = &B->layer[l];
	const char *net = ly->net ? ly->net : "";
	char fn[300];
	const char *pos = l == 0		  ? "Top" :
			  l == (int)B->nlayer - 1 ? "Bot" :
						    "Inr";

	snprintf(fn, sizeof fn, "Copper,L%d,%s", l + 1, pos);
	P->begin(ly->name, fn);
	if (ly->kind == LAYER_PLANE) {
		double e = 0.3;
		double xy[10] = { B->board[0] + e, B->board[1] + e,
				  B->board[2] - e, B->board[1] + e,
				  B->board[2] - e, B->board[3] - e,
				  B->board[0] + e, B->board[3] - e,
				  B->board[0] + e, B->board[1] + e };

		P->net(net);
		P->region(xy, 5);
		P->dark(0);
		// The cutouts cleared, the gap round them as round the board.
		for (size_t i = 0; i < B->ncut; i++) {
			const double *c = B->cut[i];
			double cx[10] = { c[0] - e, c[1] - e, c[2] + e,
					  c[1] - e, c[2] + e, c[3] + e,
					  c[0] - e, c[3] + e, c[0] - e,
					  c[1] - e };

			P->region(cx, 5);
		}
		for (size_t q = 0; q < B->npart; q++) {
			for (size_t k = 0;
			     B->part[q].placed && k < B->part[q].fp->npad;
			     k++) {
				struct pcb_spot s;
				int n = pcb_net_of(B, q, k);

				pcb_spot(B, &B->part[q], k, &s);
				// A pad of another net, or one of its own that
				// joins by a neck (a thermal relief).
				if ((s.layer == l || s.layer < 0) &&
				    (n < 0 || strcmp(B->net[n].name, net) ||
				     pcb_relieved(B, q, k))) {
					pad(&s, B->clearance);
				}
			}
		}
		for (size_t i = 0; i < B->nvia; i++) {
			const struct pcb_via *v = &B->via[i];

			if (strcmp(B->net[v->net].name, net)) {
				P->flash(0, v->x, v->y,
					 v->dia + 2 * B->clearance, 0);
			}
		}
		for (size_t i = 0; i < B->ntrack; i++) {
			if (B->track_[i].layer == l &&
			    strcmp(B->net[B->track_[i].net].name, net)) {
				track(&B->track_[i],
				      B->track_[i].w + 2 * B->clearance);
			}
		}
		P->net(NULL);
		P->dark(1);
	}
	for (size_t q = 0; q < B->npart; q++) {
		const struct pcb_part *p = &B->part[q];

		for (size_t k = 0; p->placed && p->fp && k < p->fp->npad; k++) {
			struct pcb_spot s;
			int n = pcb_net_of(B, q, k);

			pcb_spot(B, p, k, &s);
			if (s.layer != l && s.layer >= 0) {
				continue;
			}
			P->part(p->name);
			P->net(n >= 0 ? B->net[n].name : NULL);
			pad(&s, 0);
			if (P->pin) {
				char pin[1200];

				snprintf(pin, sizeof pin, "%s.%s", pcb_ref(p),
					 p->fp->pad[k].name);
				P->pin(pin, s.x, s.y);
			}
			P->net(NULL);
			P->part(NULL);
		}
	}
	for (size_t i = 0; i < B->ntrack; i++) {
		const struct pcb_track *t = &B->track_[i];

		if (t->layer == l) {
			P->net(B->net[t->net].name);
			track(t, t->w);
			P->net(NULL);
		}
	}
	for (size_t i = 0; i < B->nvia; i++) {
		const struct pcb_via *v = &B->via[i];

		P->net(B->net[v->net].name);
		P->flash(0, v->x, v->y, v->dia, 0);
		P->net(NULL);
	}
	P->end();
}

// The solder mask (grown by 0.05 mm) or paste of the top: its pads.
static void top_pads(const char *name, const char *function, double g,
		     int smd_only)
{
	P->begin(name, function);
	for (size_t q = 0; q < B->npart; q++) {
		const struct pcb_part *p = &B->part[q];

		for (size_t k = 0; p->placed && p->fp && k < p->fp->npad; k++) {
			struct pcb_spot s;

			pcb_spot(B, p, k, &s);
			if ((s.layer == 0 || (s.layer < 0 && !smd_only)) &&
			    !(smd_only && p->fp->pad[k].nopaste)) {
				P->part(p->name);
				pad(&s, g);
				P->part(NULL);
			}
		}
	}
	P->end();
}

// The outlines of the courtyards of one side, a drawing only: no maker
// needs them, but a viewer shows what keeps the parts apart.
static void courtyards(int bottom)
{
	P->begin(bottom ? "bottomcourt" : "topcourt",
		 bottom ? "Other,Courtyard,Bot" : "Other,Courtyard,Top");
	for (size_t q = 0; q < B->npart; q++) {
		const struct pcb_part *p = &B->part[q];

		if (p->placed && p->bottom == bottom) {
			double c[4];

			pcb_court(p, c);
			P->part(p->name);
			rect(0, c);
			P->part(NULL);
		}
	}
	P->end();
}

// The silkscreen of one side, cut where the mask opens.
static void silk(int bottom)
{
	struct pcb_stroke *s;
	size_t n = pcb_silk(B, &s);

	P->begin(bottom ? "bottomsilk" : "topsilk",
		 bottom ? "Legend,Bot" : "Legend,Top");
	for (size_t i = 0; i < n; i++) {
		if (s[i].bottom == bottom) {
			const double xy[4] = { s[i].x0, s[i].y0, s[i].x1,
					       s[i].y1 };

			P->part(s[i].part < B->npart ? B->part[s[i].part].name :
						       NULL);
			P->line(s[i].w, xy, 2);
			P->part(NULL);
		}
	}
	P->dark(0);
	for (size_t q = 0; q < B->npart; q++) {
		for (size_t k = 0; B->part[q].placed && k < B->part[q].fp->npad;
		     k++) {
			double o[4];

			if (pcb_opening(B, q, k, bottom, o)) {
				P->flash(1, (o[0] + o[2]) / 2,
					 (o[1] + o[3]) / 2, o[2] - o[0],
					 o[3] - o[1]);
			}
		}
	}
	P->dark(1);
	free(s);
	P->end();
}

void pcb_plot(const struct pcb *b, const struct pcb_plot *p)
{
	B = b;
	P = p;
	for (size_t l = 0; l < B->nlayer; l++) {
		copper((int)l);
	}
	top_pads("topmask", "Soldermask,Top", 0.05, 0);
	top_pads("toppaste", "Paste,Top", 0, 1);
	P->begin("edge", "Profile,NP");
	rect(0, B->board);
	for (size_t i = 0; i < B->ncut; i++) {
		round_rect(0, B->cut[i], B->cutr[i]);
	}
	P->end();
	courtyards(0);
	courtyards(1);
	silk(0);
	silk(1);
}
