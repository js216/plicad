// SPDX-License-Identifier: GPL-3.0-only
// pcbcheck.c -- Check a board: its copper against its netlist and rules
// Copyright (c) 2026 Jakob Kastelic

// Usage: pcbcheck [-l] [-L LIB]... board.pcb < board.lock
//
// Reads the source (pcb(5)), its netlist and footprints, and the lock,
// and checks the whole board from its copper alone.  Pads, tracks and
// vias join where they touch on a layer (a via and a hole on every
// layer), and a via or hole, or a pad on a plane layer, of the plane's
// net joins the plane.  Then:
//   each net must be one piece of copper, and no piece may hold pads of
//   two nets (a short);
//   copper of two nets keeps the clearance apart, and the edge rule from
//   the outline and the cutouts, measured from the copper itself to the
//   round corners (but for the parts it excepts);
//   each track ends at the center of a pad of its net (or an end of a
//   long pad), at a via, or on the copper a fork of its net feeds;
//   the copper between two pins of a width statement, and each
//   connection of every net, is no narrower than its width (the width
//   statements, else its class's width, else rule track; a
//   neck to a via or plane, pcb_neck_width);
//   every via keeps the via gap from the pads of its net, so a neck
//   joins them; the solder mask keeps its web between two openings and
//   between an opening and a via; a small part of two pins with one pad
//   on a plane joins it by a neck no wider than a track, with no via by
//   it, so its pads heat alike;
//   no silkscreen (a footprint's lines, a designator) runs over an
//   opening of the solder mask, where it would be cut;
//   no two courtyards on one side overlap, and no courtyard on either
//   side, nor any via, comes into the keep-out of a hole (pcbfp(5)),
//   whatever the nets;
//   every part is placed by a current statement, with the nets of its
//   pads as they are, and each net's copper was made by its current
//   statements for its pads where they stand; the lock tells these in
//   plain words, so the fault says what changed.
// Each fault is one line, in the names of the source, with the place it
// is at, and printed once; nothing is printed when all is well.  With
// -l each net's track length and vias are printed too, one net a line.

#define _POSIX_C_SOURCE 200809L // For strtok_r.

#include "pcbsrc.h"
#include <math.h>
#include <string.h>

static struct pcb B;

// A piece of copper: a segment (a point for a pad or via) of a radius,
// or a rectangle; on a layer (-1 all).
struct item {
	int net, layer, rect, via;
	double ax, ay, bx, by, r; // A segment and its half width, or the box.
	const char *what; // The part, or the net, for the message.
	long part, pad; // A pad's, or -1.
};

static struct item *items;
static size_t nitems;
static size_t *up;

static void add(struct item it)
{
	items = pcb_grow(items, nitems, sizeof *items);
	items[nitems++] = it;
}

static size_t find(size_t i)
{
	while (up[i] != i) {
		up[i] = up[up[i]];
		i = up[i];
	}
	return i;
}

static double seg_seg(double ax, double ay, double bx, double by, double cx,
		      double cy, double dx, double dy)
{
	double d = fmin(fmin(pcb_seg_dist(ax, ay, cx, cy, dx, dy),
			     pcb_seg_dist(bx, by, cx, cy, dx, dy)),
			fmin(pcb_seg_dist(cx, cy, ax, ay, bx, by),
			     pcb_seg_dist(dx, dy, ax, ay, bx, by)));
	double d1 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
	double d2 = (bx - ax) * (dy - ay) - (by - ay) * (dx - ax);
	double d3 = (dx - cx) * (ay - cy) - (dy - cy) * (ax - cx);
	double d4 = (dx - cx) * (by - cy) - (dy - cy) * (bx - cx);

	return d1 * d2 < 0 && d3 * d4 < 0 ? 0 : d;
}

// The gap between two pieces of copper, 0 where they touch or overlap.
static double gap(const struct item *a, const struct item *b)
{
	double d;

	if (a->rect && b->rect) {
		double dx = fmax(fmax(a->ax - b->bx, b->ax - a->bx), 0);
		double dy = fmax(fmax(a->ay - b->by, b->ay - a->by), 0);

		return hypot(dx, dy);
	}
	if (a->rect || b->rect) {
		const struct item *r = a->rect ? a : b, *s = a->rect ? b : a;
		const double c[8] = { r->ax, r->ay, r->bx, r->ay,
				      r->bx, r->by, r->ax, r->by };

		if ((s->ax >= r->ax && s->ax <= r->bx && s->ay >= r->ay &&
		     s->ay <= r->by) ||
		    (s->bx >= r->ax && s->bx <= r->bx && s->by >= r->ay &&
		     s->by <= r->by)) {
			return 0;
		}
		d = 1e300;
		for (int i = 0; i < 4; i++) {
			d = fmin(d,
				 seg_seg(s->ax, s->ay, s->bx, s->by, c[2 * i],
					 c[2 * i + 1], c[(2 * i + 2) % 8],
					 c[(2 * i + 3) % 8]));
		}
		return fmax(d - s->r, 0);
	}
	d = seg_seg(a->ax, a->ay, a->bx, a->by, b->ax, b->ay, b->bx, b->by);
	return fmax(d - a->r - b->r, 0);
}

static int same_layer(const struct item *a, const struct item *b)
{
	return a->layer < 0 || b->layer < 0 || a->layer == b->layer;
}

// The name of each part for the messages, NAME (REF), made once.
static char **labels;

// A part for a message: NAME (REF), kept till the end.
static const char *label(const struct pcb_part *p)
{
	size_t k = (size_t)(p - B.part);

	if (!labels) {
		labels = pcb_alloc((B.npart + 1) * sizeof *labels);
	}
	if (!labels[k]) {
		labels[k] = pcb_alloc(strlen(p->name) + strlen(pcb_ref(p)) + 4);
		sprintf(labels[k], "%s (%s)", p->name, pcb_ref(p));
	}
	return labels[k];
}

// The name of pad k of part q for a message: PART.PAD (REF).
static const char *pad_name(size_t q, size_t k)
{
	static char buf[4][1200];
	static int n;
	char *s = buf[n++ % 4];

	snprintf(s, sizeof buf[0], "%s.%s (%s)", B.part[q].name,
		 B.part[q].fp->pad[k].name, pcb_ref(&B.part[q]));
	return s;
}

// The pieces of copper of the board, one for each pad, track segment
// and via.
static void gather(void)
{
	for (size_t q = 0; q < B.npart; q++) {
		const struct pcb_part *p = &B.part[q];

		for (size_t k = 0; p->placed && p->fp && k < p->fp->npad; k++) {
			struct pcb_spot s;
			struct item it;

			pcb_spot(&B, p, k, &s);
			memset(&it, 0, sizeof it);
			it.net = pcb_net_of(&B, q, k);
			it.layer = s.layer;
			it.what = label(p);
			it.part = (long)q;
			it.pad = (long)k;
			if (s.shape == PAD_CIRCLE) {
				it.ax = it.bx = s.x;
				it.ay = it.by = s.y;
				it.r = s.w / 2;
			} else {
				it.rect = 1;
				it.ax = s.x - s.w / 2;
				it.ay = s.y - s.h / 2;
				it.bx = s.x + s.w / 2;
				it.by = s.y + s.h / 2;
			}
			add(it);
		}
	}
	for (size_t i = 0; i < B.ntrack; i++) {
		const struct pcb_track *t = &B.track_[i];

		for (size_t j = 1; j < t->n; j++) {
			struct item it;

			memset(&it, 0, sizeof it);
			it.net = t->net;
			it.layer = t->layer;
			it.ax = t->xy[2 * j - 2];
			it.ay = t->xy[2 * j - 1];
			it.bx = t->xy[2 * j];
			it.by = t->xy[2 * j + 1];
			it.r = t->w / 2;
			it.what = B.net[t->net].name;
			it.part = it.pad = -1;
			add(it);
		}
	}
	for (size_t i = 0; i < B.nvia; i++) {
		struct item it;

		memset(&it, 0, sizeof it);
		it.net = B.via[i].net;
		it.layer = -1;
		it.ax = it.bx = B.via[i].x;
		it.ay = it.by = B.via[i].y;
		it.r = B.via[i].dia / 2;
		it.what = "a via";
		it.via = 1;
		it.part = it.pad = -1;
		add(it);
	}
}

static int is_except(const char *name)
{
	for (size_t i = 0; i < B.nexcept; i++) {
		if (!strcmp(B.edge_except[i], name)) {
			return 1;
		}
	}
	return 0;
}

static int faults;
static char **told; // The faults printed, each once.
static size_t ntold;

// Prints a fault, made by the caller, the place it is at and a hint
// after it; but not a second time word for word.
static void fault_hint(const char *what, double x, double y, const char *hint)
{
	char line[2800];

	snprintf(line, sizeof line, "%s: %s at %.3f %.3f%s", B.file, what, x, y,
		 hint);
	faults = 1;
	for (size_t i = 0; i < ntold; i++) {
		if (!strcmp(told[i], line)) {
			return;
		}
	}
	told = pcb_grow(told, ntold, sizeof *told);
	told[ntold++] = pcb_strdup(line);
	printf("%s\n", line);
}

static void fault(const char *what, double x, double y)
{
	fault_hint(what, x, y, "");
}

// Whether a stroke runs into the box, by points along it 0.05 mm apart.
static int stroke_in(const struct pcb_stroke *s, const double o[4])
{
	double len = hypot(s->x1 - s->x0, s->y1 - s->y0), r = s->w / 2;
	int n = 1 + (int)(len / 0.05);

	for (int i = 0; i <= n; i++) {
		double x = s->x0 + (s->x1 - s->x0) * i / n;
		double y = s->y0 + (s->y1 - s->y0) * i / n;

		if (x > o[0] - r && x < o[2] + r && y > o[1] - r &&
		    y < o[3] + r) {
			return 1;
		}
	}
	return 0;
}

// The parts against each other: no two courtyards on one side overlap,
// and no courtyard on either side, nor any via, comes into the keep-out
// of a hole a screw goes through, whatever the nets.
static void check_parts(void)
{
	char why[1024];

	for (size_t q = 0; q < B.npart; q++) {
		for (size_t o = q + 1; o < B.npart; o++) {
			if (pcb_clash(&B, &B.part[q], &B.part[o], why,
				      sizeof why)) {
				fault(why, B.part[q].x, B.part[q].y);
			}
		}
	}
	for (size_t v = 0; v < B.nvia; v++) {
		char msg[1200];

		if (pcb_in_keepout(&B, B.via[v].x, B.via[v].y, B.via[v].dia / 2,
				   why, sizeof why)) {
			snprintf(msg, sizeof msg, "a via of %s in %s",
				 B.net[B.via[v].net].name, why);
			fault(msg, B.via[v].x, B.via[v].y);
		}
	}
}

// The silkscreen against the openings of the mask: one fault a part.
// Whether the designator of part q found no room.
static int no_room(const struct pcb_stroke *s, size_t n, size_t q)
{
	for (size_t i = 0; i < n; i++) {
		if (s[i].part == q && s[i].ref == 2) {
			return 1;
		}
	}
	return 0;
}

// No two designators of one side overlap.
static void check_refs(const struct pcb_stroke *s, size_t n)
{
	char msg[2048], hint[600];

	for (size_t q = 0; q < B.npart; q++) {
		double a[4];

		if (!pcb_ref_box(s, n, q, a)) {
			continue;
		}
		for (size_t o = q + 1; o < B.npart; o++) {
			double c[4];

			if (B.part[o].bottom == B.part[q].bottom &&
			    pcb_ref_box(s, n, o, c) && a[0] < c[2] &&
			    c[0] < a[2] && a[1] < c[3] && c[1] < a[3]) {
				// The hint for the one with no room, else the
				// later.
				size_t h = no_room(s, n, q) &&
							   !no_room(s, n, o) ?
						   q :
						   o;

				pcb_ref_hint(&B, s, n, h, hint, sizeof hint);
				snprintf(msg, sizeof msg,
					 "the designators of %s and %s overlap",
					 label(&B.part[q]), label(&B.part[o]));
				fault_hint(msg, a[0], a[1], hint);
			}
		}
	}
}

// The silkscreen of stroke s, for a message: its part, or a text
// statement's text.
static const char *silk_of(const struct pcb_stroke *s)
{
	static char buf[1200];

	if (s->part < B.npart) {
		return label(&B.part[s->part]);
	}
	{
		const struct pcb_stmt *st = &B.stmt[s->part - B.npart];
		char *t = pcb_words(st->w + 1, st->n - 1);

		snprintf(buf, sizeof buf, "the text %s", t);
		free(t);
	}
	return buf;
}

static void check_silk(void)
{
	struct pcb_stroke *s;
	size_t n = pcb_silk(&B, &s), off = (size_t)-1;
	char msg[2048], hint[600];

	check_refs(s, n);
	for (size_t i = 0; i < n; i++) {
		int bottom = s[i].bottom;

		// A text statement's on the board, off its cutouts: once.
		if (s[i].part >= B.npart && s[i].part != off &&
		    pcb_edge_dist(&B, 0,
				  (const double[4]){ s[i].x0, s[i].y0, s[i].x1,
						     s[i].y1 },
				  s[i].w / 2, NULL) < 0) {
			snprintf(msg, sizeof msg,
				 "the silkscreen of %s is off the board or "
				 "over a cutout",
				 silk_of(&s[i]));
			fault(msg, s[i].x0, s[i].y0);
			off = s[i].part;
		}
		for (size_t q = 0; q < B.npart; q++) {
			for (size_t k = 0;
			     B.part[q].placed && k < B.part[q].fp->npad; k++) {
				double o[4];

				if (pcb_opening(&B, q, k, bottom, o) &&
				    stroke_in(&s[i], o)) {
					*hint = 0;
					if (s[i].ref && s[i].part < B.npart) {
						pcb_ref_hint(&B, s, n,
							     s[i].part, hint,
							     sizeof hint);
					}
					snprintf(
						msg, sizeof msg,
						"the silkscreen of %s runs over pad %s",
						silk_of(&s[i]), pad_name(q, k));
					fault_hint(msg, s[i].x0, s[i].y0, hint);
					while (i + 1 < n &&
					       s[i + 1].part == s[i].part) {
						i++;
					}
					q = B.npart;
					break;
				}
			}
		}
	}
	free(s);
}

static void check_copper(void)
{
	size_t nplane = B.nlayer, base;
	char msg[2048];

	// The items, then one node for each plane.
	base = nitems;
	up = pcb_alloc((nitems + nplane + 1) * sizeof *up);
	for (size_t i = 0; i < nitems + nplane; i++) {
		up[i] = i;
	}
	for (size_t i = 0; i < nitems; i++) {
		const struct item *a = &items[i];

		for (size_t j = i + 1; j < nitems; j++) {
			const struct item *b = &items[j];
			double g;

			if (!same_layer(a, b)) {
				continue;
			}
			g = gap(a, b);
			if (g <= 1e-9) {
				up[find(i)] = find(j);
			} else if (a->net != b->net && g < B.clearance - 1e-6) {
				snprintf(msg, sizeof msg,
					 "%s and %s are %.3f mm apart", a->what,
					 b->what, g);
				fault(msg, (a->ax + b->ax) / 2,
				      (a->ay + b->ay) / 2);
			}
		}
		// Joined to a plane: of its net, through a hole or a via, or a
		// pad or track on the plane's layer; but a pad the plane
		// relieves joins it by its neck alone.
		for (size_t l = 0; l < B.nlayer; l++) {
			const struct pcb_layer *ly = &B.layer[l];

			if (ly->kind == LAYER_PLANE && ly->net && a->net >= 0 &&
			    !strcmp(B.net[a->net].name, ly->net) &&
			    (a->layer < 0 || a->layer == (int)l) &&
			    !(a->part >= 0 && pcb_relieved(&B, (size_t)a->part,
							   (size_t)a->pad))) {
				up[find(i)] = find(base + l);
			}
		}
		// The edge: the outline and the cutouts, but for the parts
		// it excepts.
		if ((a->layer >= 0 || a->r > 0) &&
		    !(a->part >= 0 && is_except(B.part[a->part].name))) {
			double at[2];
			double d = pcb_edge_dist(
				&B, a->rect,
				(const double[4]){ a->ax, a->ay, a->bx, a->by },
				a->rect ? 0 : a->r, at);

			if (d < -1e-6) {
				snprintf(msg, sizeof msg,
					 "%s runs past the edge", a->what);
				fault(msg, at[0], at[1]);
			} else if (d < B.edge - 1e-6) {
				snprintf(msg, sizeof msg,
					 "%s is nearer the edge than %.3f mm",
					 a->what, B.edge);
				fault(msg, at[0], at[1]);
			}
		}
	}
	// Each net one piece, and no piece of two nets; a net that waits
	// whole for parts not placed (of no plane) may be in pieces.
	for (size_t k = 0; k < B.nnet; k++) {
		size_t root = (size_t)-1;

		if (B.net[k].waits && B.net[k].plane < 0) {
			continue;
		}
		for (size_t i = 0; i < nitems; i++) {
			if (items[i].net != (int)k ||
			    items[i].rect + (items[i].r > 0) == 0) {
				continue;
			}
			if (root == (size_t)-1) {
				root = find(i);
			} else if (find(i) != root) {
				snprintf(msg, sizeof msg,
					 "net %s: %s is not joined to the rest",
					 B.net[k].name, items[i].what);
				fault(msg, items[i].ax, items[i].ay);
				break;
			}
		}
	}
	for (size_t i = 0; i < nitems; i++) {
		for (size_t j = i + 1; j < nitems; j++) {
			if (items[i].net != items[j].net && items[i].net >= 0 &&
			    items[j].net >= 0 && find(i) == find(j)) {
				snprintf(msg, sizeof msg,
					 "nets %s and %s are shorted",
					 B.net[items[i].net].name,
					 B.net[items[j].net].name);
				fault(msg, items[i].ax, items[i].ay);
				return;
			}
		}
	}
}

// Why a part's lock is stale, from its words now (by) and the lock's
// (was): "nets PAD NET ... by STATEMENT".
static void stale_part(const struct pcb_part *p, const char *by,
		       const char *was)
{
	const char *a = strstr(by, " by "), *b = strstr(was, " by ");
	char *x, *y, *sx = NULL, *sy = NULL;

	if (!a || !b || strcmp(a, b)) {
		printf("%s: %s (%s) is placed by a statement since changed\n",
		       B.file, p->name, pcb_ref(p));
		faults = 1;
		return;
	}
	x = pcb_strdup(by);
	y = pcb_strdup(was);
	x[a - by] = 0;
	y[b - was] = 0;
	for (char *u = strtok_r(x + 4, " ", &sx),
		  *v = strtok_r(y + 4, " ", &sy);
	     u && v;
	     u = strtok_r(NULL, " ", &sx), v = strtok_r(NULL, " ", &sy)) {
		char *nu = strtok_r(NULL, " ", &sx),
		     *nv = strtok_r(NULL, " ", &sy);

		if (nu && nv && strcmp(nu, nv)) {
			printf("%s: %s (%s) was placed with its pad %s on %s, "
			       "now on %s\n",
			       B.file, p->name, pcb_ref(p), u, nv, nu);
			faults = 1;
		}
	}
	free(x);
	free(y);
}

// Each net's copper made by its statements now, for its pads where they
// stand.
static void check_nets(void)
{
	for (size_t k = 0; k < B.nnet; k++) {
		char *w, **pads;
		size_t n;

		if (!B.netby[k]) {
			continue;
		}
		w = pcb_net_words(&B, k);
		if (strcmp(w, B.netby[k])) {
			printf("%s: net %s: its copper was made by statements "
			       "since changed\n",
			       B.file, B.net[k].name);
			faults = 1;
		}
		free(w);
		pads = pcb_net_pads(&B, k, &n);
		for (size_t i = 0; i < B.nnetpad[k]; i++) {
			const char *was = B.netpad[k][i];
			size_t len = strcspn(was, " ");
			int same = 0;

			for (size_t j = 0; !same && j < n; j++) {
				same = !strcmp(pads[j], was);
			}
			for (size_t j = 0; !same && j < n; j++) {
				if (!strncmp(pads[j], was, len) &&
				    pads[j][len] == ' ') {
					printf("%s: net %s: its copper was made "
					       "for %.*s at%s, now at%s\n",
					       B.file, B.net[k].name, (int)len,
					       was, was + len, pads[j] + len);
					same = 1;
					faults = 1;
				}
			}
			if (!same) {
				printf("%s: net %s: its copper was made for %s, "
				       "no pad of it now\n",
				       B.file, B.net[k].name, was);
				faults = 1;
			}
		}
		for (size_t j = 0; j < n; j++) {
			free(pads[j]);
		}
		free(pads);
	}
}

// The via gap: each via's ring that far from the edge of every pad of
// its net that is soldered, joined by a neck track, not standing on or
// by the pad; a via may stand in a contact pad.
static void check_viagap(void)
{
	char msg[2048];

	for (size_t i = 0; i < nitems; i++) {
		const struct item *v = &items[i];

		for (size_t j = 0; v->via && j < nitems; j++) {
			const struct item *p = &items[j];
			double g;

			if (p->part < 0 || p->net != v->net ||
			    !pcb_soldered(&B.part[p->part], (size_t)p->pad)) {
				continue;
			}
			g = gap(v, p);
			if (g < B.viagap - 1e-6) {
				snprintf(msg, sizeof msg,
					 "a via of %s is %.3f mm from pad %s, "
					 "under the via gap %.3f",
					 B.net[v->net].name, g,
					 pad_name((size_t)p->part,
						  (size_t)p->pad),
					 B.viagap);
				fault(msg, v->ax, v->ay);
			}
		}
	}
}

// The distance between two boxes, x0 y0 x1 y1; 0 if they overlap.
static double box_gap(const double *a, const double *b)
{
	return hypot(fmax(0, fmax(b[0] - a[2], a[0] - b[2])),
		     fmax(0, fmax(b[1] - a[3], a[1] - b[3])));
}

// The web of solder mask: between two openings of a side, of two nets
// (where none would be a bridge) or of one net but apart (a sliver);
// and between an opening and a via, which the mask covers.
static void check_maskweb(void)
{
	char msg[2048];

	for (int side = 0; side < 2; side++) {
		for (size_t q = 0; q < B.npart; q++) {
			for (size_t k = 0; B.part[q].placed && B.part[q].fp &&
					   k < B.part[q].fp->npad;
			     k++) {
				double a[4];
				int na = pcb_net_of(&B, q, k);

				if (!pcb_opening(&B, q, k, side, a)) {
					continue;
				}
				for (size_t r = q; r < B.npart; r++) {
					for (size_t m = r == q ? k + 1 : 0;
					     B.part[r].placed && B.part[r].fp &&
					     m < B.part[r].fp->npad;
					     m++) {
						double c[4], g;
						int nc = pcb_net_of(&B, r, m);

						if (!pcb_opening(&B, r, m, side,
								 c)) {
							continue;
						}
						g = box_gap(a, c);
						if (g >= B.maskweb - 1e-6 ||
						    (na == nc && na >= 0 &&
						     g <= 1e-9)) {
							continue;
						}
						snprintf(msg, sizeof msg,
							 "the mask web between "
							 "pad %s and pad %s is "
							 "%.3f mm, under %.3f",
							 pad_name(q, k),
							 pad_name(r, m), g,
							 B.maskweb);
						fault(msg, (a[0] + c[2]) / 2,
						      (a[1] + c[3]) / 2);
					}
				}
				for (size_t i = 0;
				     pcb_soldered(&B.part[q], k) && i < B.nvia;
				     i++) {
					const struct pcb_via *v = &B.via[i];
					double vb[4] = { v->x - v->dia / 2,
							 v->y - v->dia / 2,
							 v->x + v->dia / 2,
							 v->y + v->dia / 2 };
					double g = box_gap(a, vb);

					if (g < B.maskweb - 1e-6) {
						snprintf(msg, sizeof msg,
							 "the mask web between "
							 "pad %s and a via of "
							 "%s is %.3f mm, under "
							 "%.3f",
							 pad_name(q, k),
							 B.net[v->net].name, g,
							 B.maskweb);
						fault(msg, v->x, v->y);
					}
				}
			}
		}
	}
}

// The heat of soldering: a small part of two pins with one pad on a
// plane's net and the other not must join the plane by a neck no wider
// than a track, and no via by it, so both pads heat alike.
static void check_thermal(void)
{
	char msg[2048];

	for (size_t q = 0; q < B.npart; q++) {
		const struct pcb_part *p = &B.part[q];
		int n0, n1;

		if (!p->placed || !pcb_small_two_pin(p)) {
			continue;
		}
		n0 = pcb_net_of(&B, q, 0);
		n1 = pcb_net_of(&B, q, 1);
		for (size_t k = 0; k < 2; k++) {
			int n = k ? n1 : n0, o = k ? n0 : n1;
			const struct item *pad = NULL;

			if (n < 0 || B.net[n].plane < 0 ||
			    (o >= 0 && B.net[o].plane >= 0)) {
				continue;
			}
			for (size_t i = 0; i < nitems && !pad; i++) {
				if (items[i].part == (long)q &&
				    items[i].pad == (long)k) {
					pad = &items[i];
				}
			}
			for (size_t i = 0; pad && i < nitems; i++) {
				const struct item *t = &items[i];
				const char *why = NULL;
				double g;

				if (t->net != n || t->part >= 0) {
					continue;
				}
				g = gap(t, pad);
				if (t->via && g < B.viagap - 1e-6) {
					why = "a via by it";
				} else if (!t->via && same_layer(t, pad) &&
					   g <= 1e-9 &&
					   2 * t->r > B.track + 1e-6) {
					why = "a track wider than a neck";
				}
				if (why) {
					snprintf(
						msg, sizeof msg,
						"thermal imbalance: pad %s joins "
						"the plane %s with %s, but pad "
						"%s by a track",
						pad_name(q, k), B.net[n].name,
						why, pad_name(q, 1 - k));
					fault(msg, pad->ax, pad->ay);
				}
			}
		}
	}
}

// Whether (x, y) is on the copper between the two last pins of a fork
// statement of net n (fork NET PIN to PIN PIN), on its layer: where the
// fork's first pin is fed.
static int on_fork(int n, int layer, double x, double y)
{
	for (size_t i = 0; i < B.nstmt; i++) {
		const struct pcb_stmt *st = &B.stmt[i];
		size_t q[2], p[2];
		char *ka, *kb;
		int on = 0;

		if (strcmp(st->w[0], "fork") || st->n != 6 ||
		    strcmp(st->w[1], B.net[n].name) ||
		    !pcb_pin(&B, st->w[4], &q[0], &p[0]) ||
		    !pcb_pin(&B, st->w[5], &q[1], &p[1])) {
			continue;
		}
		ka = pcb_pad_key(&B, q[0], p[0]);
		kb = pcb_pad_key(&B, q[1], p[1]);
		for (size_t j = 0; !on && j < B.ntrack; j++) {
			const struct pcb_track *t = &B.track_[j];
			const struct pcb_link *l =
				t->link >= 0 ? &B.link[t->link] : NULL;

			if (t->net != n || t->layer != layer || !l ||
			    !((!strcmp(l->a, ka) && !strcmp(l->b, kb)) ||
			      (!strcmp(l->a, kb) && !strcmp(l->b, ka)))) {
				continue;
			}
			for (size_t e = 1; !on && e < t->n; e++) {
				on = pcb_seg_dist(x, y, t->xy[2 * e - 2],
						  t->xy[2 * e - 1],
						  t->xy[2 * e],
						  t->xy[2 * e + 1]) < 1e-3;
			}
		}
		free(ka);
		free(kb);
		if (on) {
			return 1;
		}
	}
	return 0;
}

// Each track ends at the center of a pad of its net on its layer (of a
// long pad, on the line between its ends), or at a via of its net, or in its own plane on the plane's layer (a neck),
// or on the copper between the two last pins of a fork of its net: a
// route never ends on the side of a pad, or in the middle of another
// track but a fork's.
static void check_ends(void)
{
	char msg[2048];

	for (size_t i = 0; i < B.ntrack; i++) {
		const struct pcb_track *t = &B.track_[i];

		for (size_t e = 0; e < 2 && t->n > 0; e++) {
			double x = t->xy[e ? 2 * t->n - 2 : 0];
			double y = t->xy[e ? 2 * t->n - 1 : 1];
			int ok = B.layer[t->layer].kind == LAYER_PLANE &&
				 B.net[t->net].plane == t->layer;

			for (size_t v = 0; !ok && v < B.nvia; v++) {
				ok = B.via[v].net == t->net &&
				     hypot(B.via[v].x - x, B.via[v].y - y) <
					     1e-3;
			}
			for (size_t q = 0; !ok && q < B.npart; q++) {
				for (size_t k = 0;
				     !ok && B.part[q].placed && B.part[q].fp &&
				     k < B.part[q].fp->npad;
				     k++) {
					struct pcb_spot s;
					double p[2][2];
					int np;

					pcb_spot(&B, &B.part[q], k, &s);
					np = pcb_pad_ends(&s, p);
					ok = pcb_net_of(&B, q, k) == t->net &&
					     (s.layer < 0 ||
					      s.layer == t->layer) &&
					     hypot(s.x - x, s.y - y) < 1e-3;
					// A long pad: any point of the line
					// between its ends.
					if (!ok && np == 2 &&
					    pcb_net_of(&B, q, k) == t->net &&
					    (s.layer < 0 ||
					     s.layer == t->layer)) {
						ok = pcb_seg_dist(
							     x, y, p[0][0],
							     p[0][1], p[1][0],
							     p[1][1]) < 1e-3;
					}
				}
			}
			ok = ok || on_fork(t->net, t->layer, x, y);
			if (!ok) {
				snprintf(
					msg, sizeof msg,
					"a track of %s ends off a pad's center "
					"or line and off a via",
					B.net[t->net].name);
				fault(msg, x, y);
			}
		}
	}
}

// The item of pad k of part q, or nitems.
static size_t pad_item(size_t q, size_t k)
{
	for (size_t i = 0; i < nitems; i++) {
		if (items[i].part == (long)q && items[i].pad == (long)k) {
			return i;
		}
	}
	return nitems;
}

// The pad of net n that key (PART.PAD) names, to *q and *k; 0 if none.
static int link_pad(int n, const char *key, long *q, long *k)
{
	for (size_t j = 0; j < B.net[n].nend; j++) {
		const struct pcb_end *e = &B.net[n].end[j];
		char *s = pcb_pad_key(&B, e->part, e->pad);
		int same = !strcmp(s, key);

		free(s);
		if (same) {
			*q = (long)e->part;
			*k = (long)e->pad;
			return 1;
		}
	}
	return 0;
}

// Whether a statement width PIN PIN W names pads (qa, ka) and (qb, kb).
static int stated(long qa, long ka, long qb, long kb)
{
	for (size_t s = 0; s < B.nstmt; s++) {
		const struct pcb_stmt *st = &B.stmt[s];
		size_t q[2], p[2];

		if (!strcmp(st->w[0], "width") && st->n == 4 &&
		    pcb_pin(&B, st->w[1], &q[0], &p[0]) &&
		    pcb_pin(&B, st->w[2], &q[1], &p[1]) &&
		    (((long)q[0] == qa && (long)p[0] == ka &&
		      (long)q[1] == qb && (long)p[1] == kb) ||
		     ((long)q[0] == qb && (long)p[0] == kb &&
		      (long)q[1] == qa && (long)p[1] == ka))) {
			return 1;
		}
	}
	return 0;
}

// The width of each connection as stated: between the two pins of each
// width PIN PIN W, the copper from one to the other (the fewest pieces
// that join them) no narrower than W; and each connection of the lock,
// of every net, no narrower than its width now (pcb_link_width), a neck
// to a via or a plane no narrower than pcb_neck_width.  One fault a
// connection: one of a width PIN PIN W is told by the first.
static void check_widths(void)
{
	char msg[2800], *done;
	size_t *from = pcb_alloc((nitems + 1) * sizeof *from);
	size_t *queue = pcb_alloc((nitems + 1) * sizeof *queue);

	for (size_t s = 0; s < B.nstmt; s++) {
		const struct pcb_stmt *st = &B.stmt[s];
		size_t q[2], p[2], a, b, head = 0, tail = 0, at;
		double w, least = 1e300, lx = 0, ly = 0;

		if (strcmp(st->w[0], "width") || st->n != 4 ||
		    !pcb_pin(&B, st->w[1], &q[0], &p[0]) ||
		    !pcb_pin(&B, st->w[2], &q[1], &p[1])) {
			continue;
		}
		w = atof(st->w[3]);
		a = pad_item(q[0], p[0]);
		b = pad_item(q[1], p[1]);
		if (a == nitems || b == nitems) {
			continue;
		}
		for (size_t i = 0; i < nitems; i++) {
			from[i] = nitems;
		}
		from[a] = a;
		queue[tail++] = a;
		while (head < tail && from[b] == nitems) {
			size_t i = queue[head++];

			for (size_t j = 0; j < nitems; j++) {
				if (from[j] == nitems &&
				    items[j].net == items[a].net &&
				    same_layer(&items[i], &items[j]) &&
				    gap(&items[i], &items[j]) <= 1e-9) {
					from[j] = i;
					queue[tail++] = j;
				}
			}
		}
		if (from[b] == nitems) {
			continue; // Not joined: told as such.
		}
		for (at = from[b]; at != a; at = from[at]) {
			const struct item *t = &items[at];

			if (t->part < 0 && !t->via && 2 * t->r < least) {
				least = 2 * t->r;
				lx = (t->ax + t->bx) / 2;
				ly = (t->ay + t->by) / 2;
			}
		}
		if (least < w - 1e-6) {
			snprintf(msg, sizeof msg,
				 "the copper from pad %s to pad %s is %.3f mm "
				 "wide, narrower than its width %.3f",
				 pad_name(q[0], p[0]), pad_name(q[1], p[1]),
				 least, w);
			fault(msg, lx, ly);
		}
	}
	free(from);
	free(queue);
	done = pcb_alloc(B.nlink + 1);
	memset(done, 0, B.nlink + 1);
	for (size_t i = 0; i < B.ntrack; i++) {
		const struct pcb_track *t = &B.track_[i];
		const struct pcb_link *l = t->link >= 0 ? &B.link[t->link] :
							  NULL;
		int neck;
		long qa, ka, qb = -1, kb = -1;
		double w;

		if (!l || l->net < 0 || done[t->link] ||
		    !link_pad(l->net, l->a, &qa, &ka)) {
			continue;
		}
		neck = !strcmp(l->b, "via") || !strcmp(l->b, "plane");
		if (!neck && !link_pad(l->net, l->b, &qb, &kb)) {
			continue;
		}
		w = neck ? pcb_neck_width(&B, l->net, (size_t)qa) :
			   pcb_link_width(&B, l->net, l->a, l->b);
		if (t->w >= w - 1e-6 || (!neck && stated(qa, ka, qb, kb))) {
			continue;
		}
		done[t->link] = 1;
		snprintf(msg, sizeof msg,
			 "net %.200s: the copper from pad %s to %s%s is %.3f "
			 "mm wide, narrower than its width %.3f",
			 B.net[l->net].name, pad_name((size_t)qa, (size_t)ka),
			 neck ? "its " : "pad ",
			 neck ? l->b : pad_name((size_t)qb, (size_t)kb), t->w,
			 w);
		fault(msg, t->xy[0], t->xy[1]);
	}
	free(done);
}

// Every part placed by a current statement.
static void check_lock(void)
{
	for (size_t q = 0; q < B.npart; q++) {
		const struct pcb_part *p = &B.part[q];
		// The part of a group placed whole that no statement places
		// is placed by the group's (pcb(5)).
		const struct pcb_stmt *st = pcb_placed_by(&B, q);
		char *by = st ? pcb_by(&B, q, st) : NULL;

		if (!p->fp) {
			continue;
		}
		if (!by) {
			printf("%s: %s (%s) is placed by no statement\n",
			       B.file, p->name, pcb_ref(p));
			faults = 1;
		} else if (!p->placed) {
			printf("%s: %s (%s) is not placed\n", B.file, p->name,
			       pcb_ref(p));
			faults = 1;
		} else if (strcmp(by, p->by ? p->by : "")) {
			stale_part(p, by, p->by ? p->by : "");
		}
		free(by);
	}
}

// The copper of a net as a graph: the points of its tracks, the centers
// of its pads and vias; joined along the tracks (a track split where a
// point of the net lies on it) and into each pad or via a point lies in.
struct node {
	double x, y;
	long part, pad; // A pad's center, or -1.
	int via;
};

static struct node *nodes;
static size_t nnodes;
static size_t (*edge)[2];
static double *elen;
static size_t nedges;

static size_t node_at(double x, double y, long part, long pad, int via)
{
	for (size_t i = 0; i < nnodes; i++) {
		if (hypot(nodes[i].x - x, nodes[i].y - y) < 1e-6 &&
		    nodes[i].part == part && nodes[i].pad == pad &&
		    nodes[i].via == via) {
			return i;
		}
	}
	nodes = pcb_grow(nodes, nnodes, sizeof *nodes);
	nodes[nnodes].x = x;
	nodes[nnodes].y = y;
	nodes[nnodes].part = part;
	nodes[nnodes].pad = pad;
	nodes[nnodes].via = via;
	return nnodes++;
}

static void join(size_t a, size_t b)
{
	edge = pcb_grow(edge, nedges, sizeof *edge);
	elen = pcb_grow(elen, nedges, sizeof *elen);
	edge[nedges][0] = a;
	edge[nedges][1] = b;
	elen[nedges++] =
		hypot(nodes[a].x - nodes[b].x, nodes[a].y - nodes[b].y);
}

static int by_t(const void *x, const void *y)
{
	const double *a = x, *b = y;

	return (a[0] > b[0]) - (a[0] < b[0]);
}

static void net_graph(int n)
{
	size_t first;

	nnodes = nedges = 0;
	for (size_t i = 0; i < B.ntrack; i++) {
		const struct pcb_track *t = &B.track_[i];

		for (size_t j = 0; t->net == n && j < t->n; j++) {
			node_at(t->xy[2 * j], t->xy[2 * j + 1], -1, -1, 0);
		}
	}
	first = nnodes; // The points of the tracks are the nodes before.
	for (size_t i = 0; i < B.ntrack; i++) {
		const struct pcb_track *t = &B.track_[i];

		for (size_t j = 1; t->net == n && j < t->n; j++) {
			const double *a = &t->xy[2 * j - 2], *b = &t->xy[2 * j];
			double(*on)[2] = pcb_alloc((first + 1) * sizeof *on);
			size_t m = 0;

			// The points on the segment, in order along it.
			for (size_t k = 0; k < first; k++) {
				double dx = b[0] - a[0], dy = b[1] - a[1];
				double l = dx * dx + dy * dy, u;

				u = l > 0 ? ((nodes[k].x - a[0]) * dx +
					     (nodes[k].y - a[1]) * dy) /
						    l :
					    0;
				if (u >= -1e-9 && u <= 1 + 1e-9 &&
				    pcb_seg_dist(nodes[k].x, nodes[k].y, a[0],
						 a[1], b[0],
						 b[1]) <= t->w / 2) {
					on[m][0] = u;
					on[m++][1] = (double)k;
				}
			}
			qsort(on, m, sizeof *on, by_t);
			for (size_t k = 1; k < m; k++) {
				join((size_t)on[k - 1][1], (size_t)on[k][1]);
			}
			free(on);
		}
	}
	for (size_t q = 0; q < B.npart; q++) {
		for (size_t k = 0;
		     B.part[q].placed && B.part[q].fp && k < B.part[q].fp->npad;
		     k++) {
			struct pcb_spot s;
			size_t c;

			if (pcb_net_of(&B, q, k) != n) {
				continue;
			}
			pcb_spot(&B, &B.part[q], k, &s);
			c = node_at(s.x, s.y, (long)q, (long)k, 0);
			for (size_t i = 0; i < first; i++) {
				if (pcb_spot_dist(&s, nodes[i].x, nodes[i].y) <=
				    1e-6) {
					join(c, i);
				}
			}
		}
	}
	for (size_t v = 0; v < B.nvia; v++) {
		size_t c;

		if (B.via[v].net != n) {
			continue;
		}
		c = node_at(B.via[v].x, B.via[v].y, -1, -1, 1);
		for (size_t i = 0; i < first; i++) {
			if (hypot(nodes[i].x - B.via[v].x,
				  nodes[i].y - B.via[v].y) <=
			    B.via[v].dia / 2 + 1e-6) {
				join(c, i);
			}
		}
	}
}

// The shortest copper of net n from pad k of part q to the nearest pad
// of another part or via of the net: its points, from the far end to the
// pad, into *path; their number, 0 if there is none.
static size_t reach(int n, size_t q, size_t k, double **path)
{
	double *d;
	size_t *prev, src = (size_t)-1, goal = (size_t)-1, m = 0;
	char *done;

	net_graph(n);
	for (size_t i = 0; i < nnodes; i++) {
		src = nodes[i].part == (long)q && nodes[i].pad == (long)k ? i :
									    src;
	}
	if (src == (size_t)-1) {
		return 0;
	}
	d = pcb_alloc(nnodes * sizeof *d);
	prev = pcb_alloc(nnodes * sizeof *prev);
	done = pcb_alloc(nnodes);
	for (size_t i = 0; i < nnodes; i++) {
		d[i] = 1e300;
		prev[i] = (size_t)-1;
	}
	d[src] = 0;
	for (;;) {
		size_t u = (size_t)-1;

		for (size_t i = 0; i < nnodes; i++) {
			if (!done[i] && d[i] < 1e299 &&
			    (u == (size_t)-1 || d[i] < d[u])) {
				u = i;
			}
		}
		if (u == (size_t)-1) {
			break;
		}
		done[u] = 1;
		if (u != src && (nodes[u].via || (nodes[u].part >= 0 &&
						  nodes[u].part != (long)q))) {
			goal = u;
			break;
		}
		for (size_t e = 0; e < nedges; e++) {
			size_t a = edge[e][0], b = edge[e][1], v;

			if (a != u && b != u) {
				continue;
			}
			v = a == u ? b : a;
			if (d[u] + elen[e] < d[v]) {
				d[v] = d[u] + elen[e];
				prev[v] = u;
			}
		}
	}
	if (goal != (size_t)-1) {
		*path = pcb_alloc(2 * (nnodes + 1) * sizeof **path);
		for (size_t u = goal; u != (size_t)-1; u = prev[u]) {
			(*path)[2 * m] = nodes[u].x;
			(*path)[2 * m + 1] = nodes[u].y;
			m++;
		}
	}
	free(d);
	free(prev);
	free(done);
	return m;
}

// The loop through each part of two pins on two nets: from each pad the
// shortest copper of its net to the nearest pad of another part or via
// (a decoupling capacitor to its pin and its ground via, a feedback
// resistor to the amplifier's two pins), closed straight between those
// ends; its area, by the shoelace.
static void loops(void)
{
	for (size_t q = 0; q < B.npart; q++) {
		const struct pcb_part *p = &B.part[q];
		int n0, n1;
		double *a = NULL, *b = NULL, area = 0;
		size_t na, nb, m;

		if (!p->placed || !p->fp || p->fp->npad != 2) {
			continue;
		}
		n0 = pcb_net_of(&B, q, 0);
		n1 = pcb_net_of(&B, q, 1);
		if (n0 < 0 || n1 < 0 || n0 == n1) {
			continue;
		}
		na = reach(n0, q, 0, &a);
		nb = reach(n1, q, 1, &b);
		if (na && nb) {
			// a runs from its far end to pad 0, b from its far end to
			// pad 1: a, then b backward, closes the loop.
			double *v = pcb_alloc(2 * (na + nb) * sizeof *v);

			memcpy(v, a, 2 * na * sizeof *v);
			for (size_t i = 0; i < nb; i++) {
				v[2 * (na + i)] = b[2 * (nb - 1 - i)];
				v[2 * (na + i) + 1] = b[2 * (nb - 1 - i) + 1];
			}
			m = na + nb;
			for (size_t i = 0; i < m; i++) {
				size_t j = (i + 1) % m;

				area += v[2 * i] * v[2 * j + 1] -
					v[2 * j] * v[2 * i + 1];
			}
			printf("loop %s %s %s %s %.3f\n", p->name, pcb_ref(p),
			       B.net[n0].name, B.net[n1].name, fabs(area) / 2);
			free(v);
		}
		free(a);
		free(b);
	}
}

// The free room of a side of courtyard c out to box o: how far o is, if
// it faces that side (0 if it overlaps c), else room as it was.
static double room_to(const double c[4], const double o[4], int side,
		      double room)
{
	int across = side == 0 || side == 1; // above, below: along x
	double d;

	if (across ? o[2] <= c[0] || o[0] >= c[2] :
		     o[3] <= c[1] || o[1] >= c[3]) {
		return room;
	}
	if (o[0] < c[2] && o[2] > c[0] && o[1] < c[3] && o[3] > c[1]) {
		return 0;
	}
	d = side == 0 ? o[1] - c[3] :
	    side == 1 ? c[1] - o[3] :
	    side == 2 ? c[0] - o[2] :
			o[0] - c[2];
	return d >= 0 && d < room ? d : room;
}

// Prints, for each part placed, the free room on each side of its
// courtyard, in mm out to 5: to the courtyards and the openings of the
// mask of the other parts of its side, the keep-outs of the holes and
// the board's edge rule.
static void rooms(void)
{
	static const char *const sides[] = { "above", "below", "left",
					     "right" };

	for (size_t q = 0; q < B.npart; q++) {
		const struct pcb_part *p = &B.part[q];
		double c[4], r[4] = { 5, 5, 5, 5 };

		if (!p->placed || !p->fp) {
			continue;
		}
		pcb_court(p, c);
		for (int s = 0; s < 4; s++) {
			double e = B.edge;
			double rim = s == 0 ? B.board[3] - e - c[3] :
				     s == 1 ? c[1] - B.board[1] - e :
				     s == 2 ? c[0] - B.board[0] - e :
					      B.board[2] - e - c[2];

			r[s] = fmax(0, fmin(r[s], rim));
		}
		for (size_t k = 0; k < B.ncut; k++) {
			const double *ct = B.cut[k];
			double cb[4] = { ct[0] - B.edge, ct[1] - B.edge,
					 ct[2] + B.edge, ct[3] + B.edge };

			for (int s = 0; s < 4; s++) {
				r[s] = room_to(c, cb, s, r[s]);
			}
		}
		for (size_t o = 0; o < B.npart; o++) {
			const struct pcb_part *op = &B.part[o];
			double oc[4], x, y, rad;

			if (o == q || !op->placed || !op->fp) {
				continue;
			}
			pcb_court(op, oc);
			for (int s = 0; op->bottom == p->bottom && s < 4; s++) {
				r[s] = room_to(c, oc, s, r[s]);
			}
			for (size_t k = 0; k < op->fp->npad; k++) {
				double ob[4];

				if (pcb_opening(&B, o, k, p->bottom, ob)) {
					for (int s = 0; s < 4; s++) {
						r[s] = room_to(c, ob, s, r[s]);
					}
				}
				if (pcb_keepout(&B, op, k, &x, &y, &rad)) {
					double kb[4] = { x - rad, y - rad,
							 x + rad, y + rad };

					for (int s = 0; s < 4; s++) {
						r[s] = room_to(c, kb, s, r[s]);
					}
				}
			}
		}
		printf("room %s %s", p->name, pcb_ref(p));
		for (int s = 0; s < 4; s++) {
			printf(" %s %.2f", sides[s], r[s]);
		}
		printf("\n");
	}
}

// Notes, not faults: each group not placed yet, and each net that waits
// for it: a plane net checked so far among its placed parts alone, any
// other net not routed till the group is placed.
static void notes(void)
{
	for (size_t s = 0; s < B.nstmt; s++) {
		if (!strcmp(B.stmt[s].w[0], "unplaced") && B.stmt[s].n == 2) {
			printf("%s:%d: note: %s is not placed yet\n", B.file,
			       B.stmt[s].line, B.stmt[s].w[1]);
		}
	}
	for (size_t k = 0; k < B.nnet; k++) {
		if (B.net[k].waits && B.net[k].nend) {
			printf("%s: note: net %s waits for parts not placed\n",
			       B.file, B.net[k].name);
		}
	}
}

int main(int argc, char **argv)
{
	char **lib = NULL;
	size_t nlib = 0;
	int i = 1, lengths = 0, room = 0;

	for (; i < argc && argv[i][0] == '-'; i++) {
		if (!strcmp(argv[i], "-l")) {
			lengths = 1;
		} else if (!strcmp(argv[i], "-r")) {
			room = 1;
		} else if (!strcmp(argv[i], "-L") && i + 1 < argc) {
			lib = pcb_grow(lib, nlib, sizeof *lib);
			lib[nlib] = pcb_alloc(strlen(argv[i + 1]) + 16);
			sprintf(lib[nlib++], "%s/footprints", argv[++i]);
		} else {
			pcb_die("usage: pcbcheck [-l] [-r] [-L LIB]... board.pcb < board.lock");
		}
	}
	if (i != argc - 1) {
		pcb_die("usage: pcbcheck [-l] [-r] [-L LIB]... board.pcb < board.lock");
	}
	pcb_read(&B, argv[i], lib, nlib, stdin, NULL);
	pcb_tell_copies(&B);
	gather();
	check_copper();
	check_viagap();
	check_maskweb();
	check_thermal();
	check_ends();
	check_widths();
	check_silk();
	check_parts();
	check_lock();
	check_nets();
	notes();
	if (room) {
		rooms();
	}
	if (lengths) {
		for (size_t k = 0; k < B.nnet; k++) {
			double len = 0;
			int nv = 0;

			for (size_t t = 0; t < B.ntrack; t++) {
				for (size_t j = 1; B.track_[t].net == (int)k &&
						   j < B.track_[t].n;
				     j++) {
					const double *xy = B.track_[t].xy;

					len += hypot(xy[2 * j] - xy[2 * j - 2],
						     xy[2 * j + 1] -
							     xy[2 * j - 1]);
				}
			}
			for (size_t v = 0; v < B.nvia; v++) {
				nv += B.via[v].net == (int)k;
			}
			printf("length %s %.3f vias %d\n", B.net[k].name, len,
			       nv);
		}
		loops();
	}
	for (size_t k = 0; k < nlib; k++) {
		free(lib[k]);
	}
	free(lib);
	for (size_t k = 0; labels && k < B.npart; k++) {
		free(labels[k]);
	}
	free(labels);
	for (size_t k = 0; k < ntold; k++) {
		free(told[k]);
	}
	free(told);
	return faults || B.failed;
}
