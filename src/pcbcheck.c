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

#include "pcbsrc.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TOUCH 1e-9 // Copper this near touches, in mm.
#define SLACK 1e-6 // What a rule lets go of rounding, in mm.
#define ON 1e-3 // A point this near copper is on it, in mm.
#define STROKE_STEP 0.05 // The step along a stroke against an opening.
#define FAR 1e300 // Beyond any length.
#define NEAR_FAR 1e299 // A length below this is reached.
#define ROOM_MOST 5.0 // The most room told of a side, in mm.
enum {
	BOX = 4, // The corners of a box,
	BOX_XY = 2 * BOX, // and their coordinates.
	FORK_WORDS = 6, // The words of fork NET PIN to PIN PIN.
	NAME_LEN = 1200, // Room for a name in a message,
	HINT_LEN = 600, // for a hint,
	MSG_LEN = 2048, // for a message,
	LONG_MSG_LEN = 2800, // and for one of two pads and a net.
	WHY_LEN = 1024 // Room for why parts clash.
};

// The check under way: the board; its pieces of copper, their sets
// (up) of what touches; the labels of the parts for the messages; the
// faults told; and the graph of the copper of the net under way.
struct check {
	struct pcb b;
	struct item *items;
	size_t nitems;
	size_t *up;
	char **labels;
	int faults;
	char **told; // The faults printed, each once.
	size_t ntold;
	struct node *nodes;
	size_t nnodes;
	size_t (*edge)[2];
	double *elen;
	size_t nedges;
};

// A piece of copper: a segment (a point for a pad or via) of a radius,
// or a rectangle; on a layer (-1 all).
struct item {
	int net, layer, rect, via;
	double ax, ay, bx, by, r; // A segment and its half width, or the box.
	const char *what; // The part, or the net, for the message.
	long part, pad; // A pad's, or -1.
};

static void add(struct check *ck, struct item it)
{
	ck->items = pcb_grow(ck->items, ck->nitems, sizeof *ck->items);
	ck->items[ck->nitems++] = it;
}

static size_t find(struct check *ck, size_t i)
{
	while (ck->up[i] != i) {
		ck->up[i] = ck->up[ck->up[i]];
		i = ck->up[i];
	}
	return i;
}

// The distance between the segments st[0], a-b, and st[1], c-d.
static double seg_seg(const struct pcb_seg st[2])
{
	double ax = st[0].a.x;
	double ay = st[0].a.y;
	double bx = st[0].b.x;
	double by = st[0].b.y;
	double cx = st[1].a.x;
	double cy = st[1].a.y;
	double dx = st[1].b.x;
	double dy = st[1].b.y;
	double d = fmin(fmin(pcb_seg_dist(st[0].a, st[1]),
			     pcb_seg_dist(st[0].b, st[1])),
			fmin(pcb_seg_dist(st[1].a, st[0]),
			     pcb_seg_dist(st[1].b, st[0])));
	double d1 = ((bx - ax) * (cy - ay)) - ((by - ay) * (cx - ax));
	double d2 = ((bx - ax) * (dy - ay)) - ((by - ay) * (dx - ax));
	double d3 = ((dx - cx) * (ay - cy)) - ((dy - cy) * (ax - cx));
	double d4 = ((dx - cx) * (by - cy)) - ((dy - cy) * (bx - cx));

	return d1 * d2 < 0 && d3 * d4 < 0 ? 0 : d;
}

// The gap between two pieces of copper, 0 where they touch or overlap.
static double gap(const struct item *a, const struct item *b)
{
	double d = NAN;

	if (a->rect && b->rect) {
		double dx = fmax(fmax(a->ax - b->bx, b->ax - a->bx), 0);
		double dy = fmax(fmax(a->ay - b->by, b->ay - a->by), 0);

		return hypot(dx, dy);
	}
	if (a->rect || b->rect) {
		const struct item *r = a->rect ? a : b;
		const struct item *s = a->rect ? b : a;
		const double c[BOX_XY] = { r->ax, r->ay, r->bx, r->ay,
					   r->bx, r->by, r->ax, r->by };

		if ((s->ax >= r->ax && s->ax <= r->bx && s->ay >= r->ay &&
		     s->ay <= r->by) ||
		    (s->bx >= r->ax && s->bx <= r->bx && s->by >= r->ay &&
		     s->by <= r->by)) {
			return 0;
		}
		d = FAR;
		for (size_t i = 0; i < BOX; i++) {
			struct pcb_seg st[2] = {
				{ { s->ax, s->ay }, { s->bx, s->by } },
				{ { c[2 * i], c[(2 * i) + 1] },
				  { c[((2 * i) + 2) % BOX_XY],
				    c[((2 * i) + 3) % BOX_XY] } }
			};

			d = fmin(d, seg_seg(st));
		}
		return fmax(d - s->r, 0);
	}
	{
		const struct pcb_seg st[2] = {
			{ { a->ax, a->ay }, { a->bx, a->by } },
			{ { b->ax, b->ay }, { b->bx, b->by } }
		};

		d = seg_seg(st);
	}
	return fmax(d - a->r - b->r, 0);
}

static int same_layer(const struct item *a, const struct item *b)
{
	return a->layer < 0 || b->layer < 0 || a->layer == b->layer;
}

// A part for a message: NAME (REF), kept till the end.
static const char *label(struct check *ck, const struct pcb_part *p)
{
	size_t k = (size_t)(p - ck->b.part);

	if (!ck->labels) {
		ck->labels = (char **)pcb_alloc((ck->b.npart + 1) *
						sizeof *ck->labels);
	}
	if (!ck->labels[k]) {
		size_t room = strlen(p->name) + strlen(pcb_ref(p)) + 4;

		ck->labels[k] = pcb_alloc(room);
		(void)snprintf(ck->labels[k], room, "%s (%s)", p->name,
			       pcb_ref(p));
	}
	return ck->labels[k];
}

// The name of pad k of part q for a message: PART.PAD (REF).
static const char *pad_name(struct check *ck, size_t q, size_t k)
{
	static char buf[4][NAME_LEN];
	static int n;
	char *s = buf[n++ % 4];

	(void)snprintf(s, sizeof buf[0], "%s.%s (%s)", ck->b.part[q].name,
		       ck->b.part[q].fp->pad[k].name, pcb_ref(&ck->b.part[q]));
	return s;
}

// The pieces of copper of the board, one for each pad, track segment
// and via.
static void gather(struct check *ck)
{
	for (size_t q = 0; q < ck->b.npart; q++) {
		const struct pcb_part *p = &ck->b.part[q];

		for (size_t k = 0; p->placed && p->fp && k < p->fp->npad; k++) {
			struct pcb_spot s;
			struct item it;

			pcb_spot(&ck->b, p, k, &s);
			memset(&it, 0, sizeof it);
			it.net = pcb_net_of(&ck->b, q, k);
			it.layer = s.layer;
			it.what = label(ck, p);
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
			add(ck, it);
		}
	}
	for (size_t i = 0; i < ck->b.ntrack; i++) {
		const struct pcb_track *t = &ck->b.track_[i];

		for (size_t j = 1; j < t->n; j++) {
			struct item it;

			memset(&it, 0, sizeof it);
			it.net = t->net;
			it.layer = t->layer;
			it.ax = t->xy[(2 * j) - 2];
			it.ay = t->xy[(2 * j) - 1];
			it.bx = t->xy[2 * j];
			it.by = t->xy[(2 * j) + 1];
			it.r = t->w / 2;
			it.what = ck->b.net[t->net].name;
			it.part = it.pad = -1;
			add(ck, it);
		}
	}
	for (size_t i = 0; i < ck->b.nvia; i++) {
		struct item it;

		memset(&it, 0, sizeof it);
		it.net = ck->b.via[i].net;
		it.layer = -1;
		it.ax = it.bx = ck->b.via[i].x;
		it.ay = it.by = ck->b.via[i].y;
		it.r = ck->b.via[i].dia / 2;
		it.what = "a via";
		it.via = 1;
		it.part = it.pad = -1;
		add(ck, it);
	}
}

static int is_except(struct check *ck, const char *name)
{
	for (size_t i = 0; i < ck->b.nexcept; i++) {
		if (!strcmp(ck->b.edge_except[i], name)) {
			return 1;
		}
	}
	return 0;
}

// Prints a fault, made by the caller, the place it is at and a hint
// after it; but not a second time word for word.
static void fault_hint(struct check *ck, const char *what, double x, double y,
		       const char *hint)
{
	char line[LONG_MSG_LEN];

	(void)snprintf(line, sizeof line, "%s: %s at %.3f %.3f%s", ck->b.file,
		       what, x, y, hint);
	ck->faults = 1;
	for (size_t i = 0; i < ck->ntold; i++) {
		if (!strcmp(ck->told[i], line)) {
			return;
		}
	}
	ck->told = (char **)pcb_grow((void *)ck->told, ck->ntold,
				     sizeof *ck->told);
	ck->told[ck->ntold++] = pcb_strdup(line);
	printf("%s\n", line);
}

static void fault(struct check *ck, const char *what, double x, double y)
{
	fault_hint(ck, what, x, y, "");
}

// Whether a stroke runs into the box, by points along it STROKE_STEP
// apart.
static int stroke_in(const struct pcb_stroke *s, const double o[4])
{
	double len = hypot(s->x1 - s->x0, s->y1 - s->y0);
	double r = s->w / 2;
	int n = 1 + (int)(len / STROKE_STEP);

	for (int i = 0; i <= n; i++) {
		double x = s->x0 + ((s->x1 - s->x0) * i / n);
		double y = s->y0 + ((s->y1 - s->y0) * i / n);

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
static void check_parts(struct check *ck)
{
	char why[WHY_LEN];

	for (size_t q = 0; q < ck->b.npart; q++) {
		for (size_t o = q + 1; o < ck->b.npart; o++) {
			if (pcb_clash(&ck->b, &ck->b.part[q], &ck->b.part[o],
				      why, sizeof why)) {
				fault(ck, why, ck->b.part[q].x,
				      ck->b.part[q].y);
			}
		}
	}
	for (size_t v = 0; v < ck->b.nvia; v++) {
		char msg[NAME_LEN];

		if (pcb_in_keepout(&ck->b, ck->b.via[v].x, ck->b.via[v].y,
				   ck->b.via[v].dia / 2, why, sizeof why)) {
			(void)snprintf(msg, sizeof msg, "a via of %s in %s",
				       ck->b.net[ck->b.via[v].net].name, why);
			fault(ck, msg, ck->b.via[v].x, ck->b.via[v].y);
		}
	}
}

// Whether the designator of part q found no room.
static int no_room(const struct pcb_strokes *ss, size_t q)
{
	for (size_t i = 0; i < ss->n; i++) {
		if (ss->s[i].part == q && ss->s[i].ref == 2) {
			return 1;
		}
	}
	return 0;
}

// Tells that the designators of parts q and o, of boxes a and c,
// overlap, with the hint for the one with no room, else the later.
static void refs_overlap(struct check *ck, const struct pcb_strokes *ss,
			 const size_t qo[2], const double a[4])
{
	char msg[MSG_LEN];
	char hint[HINT_LEN];
	size_t h = no_room(ss, qo[0]) && !no_room(ss, qo[1]) ? qo[0] : qo[1];

	pcb_ref_hint(&ck->b, ss, h, hint, sizeof hint);
	(void)snprintf(msg, sizeof msg, "the designators of %s and %s overlap",
		       label(ck, &ck->b.part[qo[0]]),
		       label(ck, &ck->b.part[qo[1]]));
	fault_hint(ck, msg, a[0], a[1], hint);
}

// No two designators of one side overlap.
static void check_refs(struct check *ck, const struct pcb_strokes *ss)
{
	for (size_t q = 0; q < ck->b.npart; q++) {
		double a[4];

		if (!pcb_ref_box(ss, q, a)) {
			continue;
		}
		for (size_t o = q + 1; o < ck->b.npart; o++) {
			double c[4];

			if (ck->b.part[o].bottom == ck->b.part[q].bottom &&
			    pcb_ref_box(ss, o, c) && a[0] < c[2] &&
			    c[0] < a[2] && a[1] < c[3] && c[1] < a[3]) {
				const size_t qo[2] = { q, o };

				refs_overlap(ck, ss, qo, a);
			}
		}
	}
}

// The silkscreen of stroke s, for a message: its part, or a text
// statement's text.
static const char *silk_of(struct check *ck, const struct pcb_stroke *s)
{
	static char buf[NAME_LEN];
	const struct pcb_stmt *st = NULL;
	char *t = NULL;

	if (s->part < ck->b.npart) {
		return label(ck, &ck->b.part[s->part]);
	}
	st = &ck->b.stmt[s->part - ck->b.npart];
	t = pcb_words(st->w + 1, st->n - 1);
	(void)snprintf(buf, sizeof buf, "the text %s", t);
	free(t);
	return buf;
}

// Whether stroke i of ss runs over a pad of its side; if so, tells it,
// once for its part.
static int silk_over_pad(struct check *ck, const struct pcb_strokes *ss,
			 size_t i)
{
	const struct pcb_stroke *s = &ss->s[i];

	for (size_t q = 0; q < ck->b.npart; q++) {
		for (size_t k = 0;
		     ck->b.part[q].placed && k < ck->b.part[q].fp->npad; k++) {
			struct pcb_end at = { q, k };
			char msg[MSG_LEN];
			char hint[HINT_LEN] = "";
			double o[4];

			if (!pcb_opening(&ck->b, at, s->bottom, o) ||
			    !stroke_in(s, o)) {
				continue;
			}
			if (s->ref && s->part < ck->b.npart) {
				pcb_ref_hint(&ck->b, ss, s->part, hint,
					     sizeof hint);
			}
			(void)snprintf(msg, sizeof msg,
				       "the silkscreen of %s runs over pad %s",
				       silk_of(ck, s), pad_name(ck, q, k));
			fault_hint(ck, msg, s->x0, s->y0, hint);
			return 1;
		}
	}
	return 0;
}

// The silkscreen against the openings of the mask: one fault a part.
static void check_silk(struct check *ck)
{
	struct pcb_strokes ss = pcb_silk(&ck->b);
	const struct pcb_stroke *s = ss.s;
	size_t off = (size_t)-1;

	check_refs(ck, &ss);
	for (size_t i = 0; i < ss.n; i++) {
		const double seg[4] = { s[i].x0, s[i].y0, s[i].x1, s[i].y1 };

		// A text statement's on the board, off its cutouts: once.
		if (s[i].part >= ck->b.npart && s[i].part != off &&
		    pcb_edge_dist(&ck->b, 0, seg, s[i].w / 2, NULL) < 0) {
			char msg[MSG_LEN];

			(void)snprintf(msg, sizeof msg,
				       "the silkscreen of %s is off the board "
				       "or over a cutout",
				       silk_of(ck, &s[i]));
			fault(ck, msg, s[i].x0, s[i].y0);
			off = s[i].part;
		}
		if (silk_over_pad(ck, &ss, i)) {
			while (i + 1 < ss.n && s[i + 1].part == s[i].part) {
				i++;
			}
		}
	}
	free(ss.s);
}

// Joins pieces i and j of copper that touch; tells them if they are of
// two nets and nearer than the clearance.
static void pair_gap(struct check *ck, size_t i, size_t j)
{
	const struct item *a = &ck->items[i];
	const struct item *b = &ck->items[j];
	double g = NAN;

	if (!same_layer(a, b)) {
		return;
	}
	g = gap(a, b);
	if (g <= TOUCH) {
		ck->up[find(ck, i)] = find(ck, j);
	} else if (a->net != b->net && g < ck->b.clearance - SLACK) {
		char msg[MSG_LEN];

		(void)snprintf(msg, sizeof msg, "%s and %s are %.3f mm apart",
			       a->what, b->what, g);
		fault(ck, msg, (a->ax + b->ax) / 2, (a->ay + b->ay) / 2);
	}
}

// Joins piece i of copper to the planes it joins (their nodes from
// base): of its net, through a hole or a via, or a pad or track on the
// plane's layer; but a pad the plane relieves joins it by its neck
// alone.
static void join_planes(struct check *ck, size_t i, size_t base)
{
	const struct item *a = &ck->items[i];

	for (size_t l = 0; l < ck->b.nlayer; l++) {
		const struct pcb_layer *ly = &ck->b.layer[l];

		if (ly->kind == LAYER_PLANE && ly->net && a->net >= 0 &&
		    !strcmp(ck->b.net[a->net].name, ly->net) &&
		    (a->layer < 0 || a->layer == (int)l) &&
		    !(a->part >= 0 &&
		      pcb_relieved(&ck->b, (size_t)a->part, (size_t)a->pad))) {
			ck->up[find(ck, i)] = find(ck, base + l);
		}
	}
}

// Piece a of copper against the edge: the outline and the cutouts, but
// for the parts it excepts.
static void edge_gap(struct check *ck, const struct item *a)
{
	const double seg[4] = { a->ax, a->ay, a->bx, a->by };
	char msg[MSG_LEN];
	double at[2];
	double d = NAN;

	if ((a->layer < 0 && !(a->r > 0)) ||
	    (a->part >= 0 && is_except(ck, ck->b.part[a->part].name))) {
		return;
	}
	d = pcb_edge_dist(&ck->b, a->rect, seg, a->rect ? 0 : a->r, at);
	if (d < -SLACK) {
		(void)snprintf(msg, sizeof msg, "%s runs past the edge",
			       a->what);
		fault(ck, msg, at[0], at[1]);
	} else if (d < ck->b.edge - SLACK) {
		(void)snprintf(msg, sizeof msg,
			       "%s is nearer the edge than %.3f mm", a->what,
			       ck->b.edge);
		fault(ck, msg, at[0], at[1]);
	}
}

// Each net one piece; a net that waits whole for parts not placed (of no
// plane) may be in pieces.
static void check_pieces(struct check *ck)
{
	for (size_t k = 0; k < ck->b.nnet; k++) {
		size_t root = (size_t)-1;

		if (ck->b.net[k].waits && ck->b.net[k].plane < 0) {
			continue;
		}
		for (size_t i = 0; i < ck->nitems; i++) {
			const struct item *it = &ck->items[i];

			if (it->net != (int)k || it->rect + (it->r > 0) == 0) {
				continue;
			}
			if (root == (size_t)-1) {
				root = find(ck, i);
			} else if (find(ck, i) != root) {
				char msg[MSG_LEN];

				(void)snprintf(msg, sizeof msg,
					       "net %s: %s is not joined to "
					       "the rest",
					       ck->b.net[k].name, it->what);
				fault(ck, msg, it->ax, it->ay);
				break;
			}
		}
	}
}

// No piece of two nets: the first short told.
static void check_shorts(struct check *ck)
{
	for (size_t i = 0; i < ck->nitems; i++) {
		const struct item *a = &ck->items[i];

		for (size_t j = i + 1; j < ck->nitems; j++) {
			const struct item *b = &ck->items[j];

			if (a->net != b->net && a->net >= 0 && b->net >= 0 &&
			    find(ck, i) == find(ck, j)) {
				char msg[MSG_LEN];

				(void)snprintf(msg, sizeof msg,
					       "nets %s and %s are shorted",
					       ck->b.net[a->net].name,
					       ck->b.net[b->net].name);
				fault(ck, msg, a->ax, a->ay);
				return;
			}
		}
	}
}

static void check_copper(struct check *ck)
{
	size_t nplane = ck->b.nlayer;
	size_t base = ck->nitems;

	// The items, then one node for each plane.
	ck->up = pcb_alloc((ck->nitems + nplane + 1) * sizeof *ck->up);
	for (size_t i = 0; i < ck->nitems + nplane; i++) {
		ck->up[i] = i;
	}
	for (size_t i = 0; i < ck->nitems; i++) {
		for (size_t j = i + 1; j < ck->nitems; j++) {
			pair_gap(ck, i, j);
		}
		join_planes(ck, i, base);
		edge_gap(ck, &ck->items[i]);
	}
	check_pieces(ck);
	check_shorts(ck);
}

// Why a part's lock is stale, from its words now (by) and the lock's
// (was): "nets PAD NET ... by STATEMENT".
static void stale_part(struct check *ck, const struct pcb_part *p,
		       const char *by, const char *was)
{
	const char *a = strstr(by, " by ");
	const char *b = strstr(was, " by ");
	char *x = NULL;
	char *y = NULL;
	char *sx = NULL;
	char *sy = NULL;

	if (!a || !b || strcmp(a, b) != 0) {
		printf("%s: %s (%s) is placed by a statement since changed\n",
		       ck->b.file, p->name, pcb_ref(p));
		ck->faults = 1;
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
		char *nu = strtok_r(NULL, " ", &sx);
		char *nv = strtok_r(NULL, " ", &sy);

		if (nu && nv && strcmp(nu, nv) != 0) {
			printf("%s: %s (%s) was placed with its pad %s on %s, "
			       "now on %s\n",
			       ck->b.file, p->name, pcb_ref(p), u, nv, nu);
			ck->faults = 1;
		}
	}
	free(x);
	free(y);
}

// Each net's copper made by its statements now, for its pads where they
// stand.
static void check_nets(struct check *ck)
{
	for (size_t k = 0; k < ck->b.nnet; k++) {
		char *w = NULL;
		char **pads = NULL;
		size_t n = 0;

		if (!ck->b.netby[k]) {
			continue;
		}
		w = pcb_net_words(&ck->b, k);
		if (strcmp(w, ck->b.netby[k]) != 0) {
			printf("%s: net %s: its copper was made by statements "
			       "since changed\n",
			       ck->b.file, ck->b.net[k].name);
			ck->faults = 1;
		}
		free(w);
		pads = pcb_net_pads(&ck->b, k, &n);
		for (size_t i = 0; i < ck->b.nnetpad[k]; i++) {
			const char *was = ck->b.netpad[k][i];
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
					       ck->b.file, ck->b.net[k].name,
					       (int)len, was, was + len,
					       pads[j] + len);
					same = 1;
					ck->faults = 1;
				}
			}
			if (!same) {
				printf("%s: net %s: its copper was made for %s, "
				       "no pad of it now\n",
				       ck->b.file, ck->b.net[k].name, was);
				ck->faults = 1;
			}
		}
		for (size_t j = 0; j < n; j++) {
			free(pads[j]);
		}
		free((void *)pads);
	}
}

// The via gap: each via's ring that far from the edge of every pad of
// its net that is soldered, joined by a neck track, not standing on or
// by the pad; a via may stand in a contact pad.
static void check_viagap(struct check *ck)
{
	char msg[MSG_LEN];

	for (size_t i = 0; i < ck->nitems; i++) {
		const struct item *v = &ck->items[i];

		for (size_t j = 0; v->via && j < ck->nitems; j++) {
			const struct item *p = &ck->items[j];
			double g = NAN;

			if (p->part < 0 || p->net != v->net ||
			    !pcb_soldered(&ck->b.part[p->part],
					  (size_t)p->pad)) {
				continue;
			}
			g = gap(v, p);
			if (g < ck->b.viagap - SLACK) {
				(void)snprintf(
					msg, sizeof msg,
					"a via of %s is %.3f mm from pad %s, "
					"under the via gap %.3f",
					ck->b.net[v->net].name, g,
					pad_name(ck, (size_t)p->part,
						 (size_t)p->pad),
					ck->b.viagap);
				fault(ck, msg, v->ax, v->ay);
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
// The mask web between the opening a of pad at, on a side, and those of
// the pads after it.
static void pad_webs(struct check *ck, int side, struct pcb_end at,
		     const double a[4])
{
	int na = pcb_net_of(&ck->b, at.part, at.pad);

	for (size_t r = at.part; r < ck->b.npart; r++) {
		const struct pcb_part *pr = &ck->b.part[r];

		for (size_t m = r == at.part ? at.pad + 1 : 0;
		     pr->placed && pr->fp && m < pr->fp->npad; m++) {
			struct pcb_end to = { r, m };
			char msg[MSG_LEN];
			double c[4];
			double g = NAN;
			int nc = pcb_net_of(&ck->b, r, m);

			if (!pcb_opening(&ck->b, to, side, c)) {
				continue;
			}
			g = box_gap(a, c);
			if (g >= ck->b.maskweb - SLACK ||
			    (na == nc && na >= 0 && g <= TOUCH)) {
				continue;
			}
			(void)snprintf(msg, sizeof msg,
				       "the mask web between pad %s and pad %s "
				       "is %.3f mm, under %.3f",
				       pad_name(ck, at.part, at.pad),
				       pad_name(ck, r, m), g, ck->b.maskweb);
			fault(ck, msg, (a[0] + c[2]) / 2, (a[1] + c[3]) / 2);
		}
	}
}

// The mask web between the opening a of pad at, soldered, and the vias,
// which are tented.
static void via_webs(struct check *ck, struct pcb_end at, const double a[4])
{
	for (size_t i = 0;
	     pcb_soldered(&ck->b.part[at.part], at.pad) && i < ck->b.nvia;
	     i++) {
		const struct pcb_via *v = &ck->b.via[i];
		const double vb[4] = { v->x - (v->dia / 2), v->y - (v->dia / 2),
				       v->x + (v->dia / 2),
				       v->y + (v->dia / 2) };
		double g = box_gap(a, vb);

		if (g < ck->b.maskweb - SLACK) {
			char msg[MSG_LEN];

			(void)snprintf(msg, sizeof msg,
				       "the mask web between pad %s and a via "
				       "of %s is %.3f mm, under %.3f",
				       pad_name(ck, at.part, at.pad),
				       ck->b.net[v->net].name, g,
				       ck->b.maskweb);
			fault(ck, msg, v->x, v->y);
		}
	}
}

static void check_maskweb(struct check *ck)
{
	for (int side = 0; side < 2; side++) {
		for (size_t q = 0; q < ck->b.npart; q++) {
			const struct pcb_part *p = &ck->b.part[q];

			for (size_t k = 0;
			     p->placed && p->fp && k < p->fp->npad; k++) {
				struct pcb_end at = { q, k };
				double a[4];

				if (!pcb_opening(&ck->b, at, side, a)) {
					continue;
				}
				pad_webs(ck, side, at, a);
				via_webs(ck, at, a);
			}
		}
	}
}

// The heat of soldering: a small part of two pins with one pad on a
// plane's net and the other not must join the plane by a neck no wider
// than a track, and no via by it, so both pads heat alike.
// Pad at of a small part of two pins, on the plane net n, whose other
// pad is on no plane: no via by it, and no track wider than a neck.
static void thermal_pad(struct check *ck, struct pcb_end at, int n)
{
	const struct item *pad = NULL;

	for (size_t i = 0; i < ck->nitems && !pad; i++) {
		if (ck->items[i].part == (long)at.part &&
		    ck->items[i].pad == (long)at.pad) {
			pad = &ck->items[i];
		}
	}
	for (size_t i = 0; pad && i < ck->nitems; i++) {
		const struct item *t = &ck->items[i];
		const char *why = NULL;
		double g = NAN;

		if (t->net != n || t->part >= 0) {
			continue;
		}
		g = gap(t, pad);
		if (t->via && g < ck->b.viagap - SLACK) {
			why = "a via by it";
		} else if (!t->via && same_layer(t, pad) && g <= TOUCH &&
			   2 * t->r > ck->b.track + SLACK) {
			why = "a track wider than a neck";
		}
		if (why) {
			char msg[MSG_LEN];

			(void)snprintf(
				msg, sizeof msg,
				"thermal imbalance: pad %s joins the "
				"plane %s with %s, but pad %s by a track",
				pad_name(ck, at.part, at.pad),
				ck->b.net[n].name, why,
				pad_name(ck, at.part, 1 - at.pad));
			fault(ck, msg, pad->ax, pad->ay);
		}
	}
}

static void check_thermal(struct check *ck)
{
	for (size_t q = 0; q < ck->b.npart; q++) {
		const struct pcb_part *p = &ck->b.part[q];
		int n0 = 0;
		int n1 = 0;

		if (!p->placed || !pcb_small_two_pin(p)) {
			continue;
		}
		n0 = pcb_net_of(&ck->b, q, 0);
		n1 = pcb_net_of(&ck->b, q, 1);
		for (size_t k = 0; k < 2; k++) {
			int n = k ? n1 : n0;
			int o = k ? n0 : n1;
			struct pcb_end at = { q, k };

			if (n < 0 || ck->b.net[n].plane < 0 ||
			    (o >= 0 && ck->b.net[o].plane >= 0)) {
				continue;
			}
			thermal_pad(ck, at, n);
		}
	}
}

// An end of a track: its net, its layer and its point.
struct track_end {
	int net;
	int layer;
	struct pcb_pt p;
};

// Whether the end te is on the copper of net n's connection between the
// pads of keys k[0] and k[1], on its layer.
static int on_link(const struct pcb *b, const struct track_end *te,
		   char *const k[2])
{
	for (size_t j = 0; j < b->ntrack; j++) {
		const struct pcb_track *t = &b->track_[j];
		const struct pcb_link *l = t->link >= 0 ? &b->link[t->link] :
							  NULL;

		if (t->net != te->net || t->layer != te->layer || !l ||
		    !((!strcmp(l->a, k[0]) && !strcmp(l->b, k[1])) ||
		      (!strcmp(l->a, k[1]) && !strcmp(l->b, k[0])))) {
			continue;
		}
		for (size_t e = 1; e < t->n; e++) {
			struct pcb_seg s = {
				{ t->xy[(2 * e) - 2], t->xy[(2 * e) - 1] },
				{ t->xy[2 * e], t->xy[(2 * e) + 1] }
			};

			if (pcb_seg_dist(te->p, s) < ON) {
				return 1;
			}
		}
	}
	return 0;
}

// Whether the end te is on the copper between the two last pins of a
// fork statement of its net (fork NET PIN to PIN PIN), on its layer:
// where the fork's first pin is fed.
static int on_fork(const struct check *ck, const struct track_end *te)
{
	const struct pcb *b = &ck->b;

	for (size_t i = 0; i < b->nstmt; i++) {
		const struct pcb_stmt *st = &b->stmt[i];
		struct pcb_end p[2] = { { 0, 0 }, { 0, 0 } };
		char *k[2] = { NULL, NULL };
		int on = 0;

		if (strcmp(st->w[0], "fork") != 0 || st->n != FORK_WORDS ||
		    strcmp(st->w[1], b->net[te->net].name) != 0 ||
		    !pcb_pin(b, st->w[FORK_WORDS - 2], &p[0]) ||
		    !pcb_pin(b, st->w[FORK_WORDS - 1], &p[1])) {
			continue;
		}
		k[0] = pcb_pad_key(b, p[0].part, p[0].pad);
		k[1] = pcb_pad_key(b, p[1].part, p[1].pad);
		on = on_link(b, te, k);
		free(k[0]);
		free(k[1]);
		if (on) {
			return 1;
		}
	}
	return 0;
}

// Whether the end te is at the center of a pad of its net on its layer,
// or of a long pad on the line between its ends.
static int on_pad(const struct pcb *b, const struct track_end *te)
{
	for (size_t q = 0; q < b->npart; q++) {
		const struct pcb_part *pt = &b->part[q];

		for (size_t k = 0; pt->placed && pt->fp && k < pt->fp->npad;
		     k++) {
			struct pcb_spot s;
			double p[2][2];
			int np = 0;

			pcb_spot(b, pt, k, &s);
			if (pcb_net_of(b, q, k) != te->net ||
			    (s.layer >= 0 && s.layer != te->layer)) {
				continue;
			}
			np = pcb_pad_ends(&s, p);
			if (hypot(s.x - te->p.x, s.y - te->p.y) < ON) {
				return 1;
			}
			// A long pad: any point of the line between its ends.
			if (np == 2) {
				struct pcb_seg line = { { p[0][0], p[0][1] },
							{ p[1][0], p[1][1] } };

				if (pcb_seg_dist(te->p, line) < ON) {
					return 1;
				}
			}
		}
	}
	return 0;
}

// Whether the end te of a track ends where a route may: in its own plane
// on the plane's layer (a neck), at a via of its net, on a pad
// (on_pad), or on a fork (on_fork).
static int end_ok(const struct check *ck, const struct track_end *te)
{
	const struct pcb *b = &ck->b;

	if (b->layer[te->layer].kind == LAYER_PLANE &&
	    b->net[te->net].plane == te->layer) {
		return 1;
	}
	for (size_t v = 0; v < b->nvia; v++) {
		if (b->via[v].net == te->net &&
		    hypot(b->via[v].x - te->p.x, b->via[v].y - te->p.y) < ON) {
			return 1;
		}
	}
	return on_pad(b, te) || on_fork(ck, te);
}

// Each track ends at the center of a pad of its net on its layer (of a
// long pad, on the line between its ends), or at a via of its net, or
// in its own plane on the plane's layer (a neck), or on the copper
// between the two last pins of a fork of its net: a route never ends on
// the side of a pad, or in the middle of another track but a fork's.
static void check_ends(struct check *ck)
{
	for (size_t i = 0; i < ck->b.ntrack; i++) {
		const struct pcb_track *t = &ck->b.track_[i];

		for (size_t e = 0; e < 2 && t->n > 0; e++) {
			const double *xy = t->xy + (e ? (2 * t->n) - 2 : 0);
			struct track_end te = { t->net,
						t->layer,
						{ xy[0], xy[1] } };

			if (!end_ok(ck, &te)) {
				char msg[MSG_LEN];

				(void)snprintf(msg, sizeof msg,
					       "a track of %s ends off a pad's "
					       "center or line and off a via",
					       ck->b.net[t->net].name);
				fault(ck, msg, xy[0], xy[1]);
			}
		}
	}
}

// The item of pad at, or nitems.
static size_t pad_item(const struct check *ck, struct pcb_end at)
{
	for (size_t i = 0; i < ck->nitems; i++) {
		if (ck->items[i].part == (long)at.part &&
		    ck->items[i].pad == (long)at.pad) {
			return i;
		}
	}
	return ck->nitems;
}

// The pad of net n that key (PART.PAD) names, to *at; 0 if none.
static int link_pad(const struct check *ck, int n, const char *key,
		    struct pcb_end *at)
{
	for (size_t j = 0; j < ck->b.net[n].nend; j++) {
		const struct pcb_end *e = &ck->b.net[n].end[j];
		char *s = pcb_pad_key(&ck->b, e->part, e->pad);
		int same = !strcmp(s, key);

		free(s);
		if (same) {
			*at = *e;
			return 1;
		}
	}
	return 0;
}

// Whether two pads are the same.
static int same_pad(struct pcb_end a, struct pcb_end b)
{
	return a.part == b.part && a.pad == b.pad;
}

// Whether a statement width PIN PIN W names the pads ab, either way.
static int stated(const struct check *ck, const struct pcb_end ab[2])
{
	for (size_t s = 0; s < ck->b.nstmt; s++) {
		const struct pcb_stmt *st = &ck->b.stmt[s];
		struct pcb_end p[2] = { { 0, 0 }, { 0, 0 } };

		if (!strcmp(st->w[0], "width") && st->n == 4 &&
		    pcb_pin(&ck->b, st->w[1], &p[0]) &&
		    pcb_pin(&ck->b, st->w[2], &p[1]) &&
		    ((same_pad(p[0], ab[0]) && same_pad(p[1], ab[1])) ||
		     (same_pad(p[0], ab[1]) && same_pad(p[1], ab[0])))) {
			return 1;
		}
	}
	return 0;
}

// Room for a search of the copper: of each item, the one it was reached
// from, and the queue.
struct search {
	size_t *from;
	size_t *queue;
};

// Searches the copper from item ab[0] to item ab[1] by the fewest
// pieces, of its net, each touching the one before, into sr; returns
// whether it reached ab[1].
static int search_path(const struct check *ck, const size_t ab[2],
		       struct search *sr)
{
	size_t a = ab[0];
	size_t head = 0;
	size_t tail = 0;

	for (size_t i = 0; i < ck->nitems; i++) {
		sr->from[i] = ck->nitems;
	}
	sr->from[a] = a;
	sr->queue[tail++] = a;
	while (head < tail && sr->from[ab[1]] == ck->nitems) {
		size_t i = sr->queue[head++];

		for (size_t j = 0; j < ck->nitems; j++) {
			if (sr->from[j] == ck->nitems &&
			    ck->items[j].net == ck->items[a].net &&
			    same_layer(&ck->items[i], &ck->items[j]) &&
			    gap(&ck->items[i], &ck->items[j]) <= TOUCH) {
				sr->from[j] = i;
				sr->queue[tail++] = j;
			}
		}
	}
	return sr->from[ab[1]] != ck->nitems;
}

// The statement st, width PIN PIN W: the copper from one pin to the
// other no narrower than W.
static void width_stmt(struct check *ck, const struct pcb_stmt *st,
		       struct search *sr)
{
	struct pcb_end p[2] = { { 0, 0 }, { 0, 0 } };
	size_t ab[2] = { 0, 0 };
	double w = NAN;
	double least = FAR;
	double lx = 0;
	double ly = 0;

	if (strcmp(st->w[0], "width") != 0 || st->n != 4 ||
	    !pcb_pin(&ck->b, st->w[1], &p[0]) ||
	    !pcb_pin(&ck->b, st->w[2], &p[1])) {
		return;
	}
	w = strtod(st->w[3], NULL);
	ab[0] = pad_item(ck, p[0]);
	ab[1] = pad_item(ck, p[1]);
	if (ab[0] == ck->nitems || ab[1] == ck->nitems ||
	    !search_path(ck, ab, sr)) {
		return; // Not joined: told as such.
	}
	for (size_t at = sr->from[ab[1]]; at != ab[0]; at = sr->from[at]) {
		const struct item *t = &ck->items[at];

		if (t->part < 0 && !t->via && 2 * t->r < least) {
			least = 2 * t->r;
			lx = (t->ax + t->bx) / 2;
			ly = (t->ay + t->by) / 2;
		}
	}
	if (least < w - SLACK) {
		char msg[LONG_MSG_LEN];

		(void)snprintf(msg, sizeof msg,
			       "the copper from pad %s to pad %s is %.3f mm "
			       "wide, narrower than its width %.3f",
			       pad_name(ck, p[0].part, p[0].pad),
			       pad_name(ck, p[1].part, p[1].pad), least, w);
		fault(ck, msg, lx, ly);
	}
}

// Track t of a connection of the lock against its width now: one fault
// a connection, done.
static void link_width(struct check *ck, const struct pcb_track *t, char *done)
{
	const struct pcb_link *l = t->link >= 0 ? &ck->b.link[t->link] : NULL;
	struct pcb_end ab[2] = { { 0, 0 }, { 0, 0 } };
	char msg[LONG_MSG_LEN];
	int neck = 0;
	double w = NAN;

	if (!l || l->net < 0 || done[t->link] ||
	    !link_pad(ck, l->net, l->a, &ab[0])) {
		return;
	}
	neck = !strcmp(l->b, "via") || !strcmp(l->b, "plane");
	if (!neck && !link_pad(ck, l->net, l->b, &ab[1])) {
		return;
	}
	w = neck ? pcb_neck_width(&ck->b, &ck->b.part[ab[0].part], l->net) :
		   pcb_link_width(&ck->b, l->net, l->a, l->b);
	if (t->w >= w - SLACK || (!neck && stated(ck, ab))) {
		return;
	}
	done[t->link] = 1;
	(void)snprintf(msg, sizeof msg,
		       "net %.200s: the copper from pad %s to %s%s is %.3f "
		       "mm wide, narrower than its width %.3f",
		       ck->b.net[l->net].name,
		       pad_name(ck, ab[0].part, ab[0].pad),
		       neck ? "its " : "pad ",
		       neck ? l->b : pad_name(ck, ab[1].part, ab[1].pad), t->w,
		       w);
	fault(ck, msg, t->xy[0], t->xy[1]);
}

// The width of each connection as stated: between the two pins of each
// width PIN PIN W, the copper from one to the other (the fewest pieces
// that join them) no narrower than W; and each connection of the lock,
// of every net, no narrower than its width now (pcb_link_width), a neck
// to a via or a plane no narrower than pcb_neck_width.  One fault a
// connection: one of a width PIN PIN W is told by the first.
static void check_widths(struct check *ck)
{
	struct search sr = { NULL, NULL };
	char *done = NULL;

	sr.from = pcb_alloc((ck->nitems + 1) * sizeof *sr.from);
	sr.queue = pcb_alloc((ck->nitems + 1) * sizeof *sr.queue);
	for (size_t s = 0; s < ck->b.nstmt; s++) {
		width_stmt(ck, &ck->b.stmt[s], &sr);
	}
	free(sr.from);
	free(sr.queue);
	done = pcb_alloc(ck->b.nlink + 1);
	memset(done, 0, ck->b.nlink + 1);
	for (size_t i = 0; i < ck->b.ntrack; i++) {
		link_width(ck, &ck->b.track_[i], done);
	}
	free(done);
}

// Every part placed by a current statement.
static void check_lock(struct check *ck)
{
	for (size_t q = 0; q < ck->b.npart; q++) {
		const struct pcb_part *p = &ck->b.part[q];
		// The part of a group placed whole that no statement places
		// is placed by the group's (pcb(5)).
		const struct pcb_stmt *st = pcb_placed_by(&ck->b, q);
		char *by = st ? pcb_by(&ck->b, q, st) : NULL;

		if (!p->fp) {
			continue;
		}
		if (!by) {
			printf("%s: %s (%s) is placed by no statement\n",
			       ck->b.file, p->name, pcb_ref(p));
			ck->faults = 1;
		} else if (!p->placed) {
			printf("%s: %s (%s) is not placed\n", ck->b.file,
			       p->name, pcb_ref(p));
			ck->faults = 1;
		} else if (strcmp(by, p->by ? p->by : "") != 0) {
			stale_part(ck, p, by, p->by ? p->by : "");
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

static size_t node_at(struct check *ck, double x, double y, long part, long pad,
		      int via)
{
	for (size_t i = 0; i < ck->nnodes; i++) {
		if (hypot(ck->nodes[i].x - x, ck->nodes[i].y - y) < SLACK &&
		    ck->nodes[i].part == part && ck->nodes[i].pad == pad &&
		    ck->nodes[i].via == via) {
			return i;
		}
	}
	ck->nodes = pcb_grow(ck->nodes, ck->nnodes, sizeof *ck->nodes);
	ck->nodes[ck->nnodes].x = x;
	ck->nodes[ck->nnodes].y = y;
	ck->nodes[ck->nnodes].part = part;
	ck->nodes[ck->nnodes].pad = pad;
	ck->nodes[ck->nnodes].via = via;
	return ck->nnodes++;
}

static void join(struct check *ck, size_t a, size_t b)
{
	ck->edge = pcb_grow(ck->edge, ck->nedges, sizeof *ck->edge);
	ck->elen = pcb_grow(ck->elen, ck->nedges, sizeof *ck->elen);
	ck->edge[ck->nedges][0] = a;
	ck->edge[ck->nedges][1] = b;
	ck->elen[ck->nedges++] = hypot(ck->nodes[a].x - ck->nodes[b].x,
				       ck->nodes[a].y - ck->nodes[b].y);
}

static int by_t(const void *lhs, const void *rhs)
{
	const double *a = lhs;
	const double *b = rhs;

	return (a[0] > b[0]) - (a[0] < b[0]);
}

// The points among the first nodes on the segment a-b of a track of
// width w, in order along it, joined in turn.
static void split_segment(struct check *ck, size_t first, const double *a,
			  const double *b, double w)
{
	double(*on)[2] = pcb_alloc((first + 1) * sizeof *on);
	struct pcb_seg s = { { a[0], a[1] }, { b[0], b[1] } };
	double dx = b[0] - a[0];
	double dy = b[1] - a[1];
	double l = (dx * dx) + (dy * dy);
	size_t m = 0;

	for (size_t k = 0; k < first; k++) {
		const struct node *nd = &ck->nodes[k];
		struct pcb_pt p = { nd->x, nd->y };
		double u = 0;

		if (l > 0) {
			u = (((nd->x - a[0]) * dx) + ((nd->y - a[1]) * dy)) / l;
		}
		if (u >= -TOUCH && u <= 1 + TOUCH &&
		    pcb_seg_dist(p, s) <= w / 2) {
			on[m][0] = u;
			on[m++][1] = (double)k;
		}
	}
	qsort(on, m, sizeof *on, by_t);
	for (size_t k = 1; k < m; k++) {
		join(ck, (size_t)on[k - 1][1], (size_t)on[k][1]);
	}
	free((void *)on);
}

// Joins the pads of net n to the first nodes, the points of its tracks,
// that lie in them.
static void pad_nodes(struct check *ck, int n, size_t first)
{
	for (size_t q = 0; q < ck->b.npart; q++) {
		const struct pcb_part *p = &ck->b.part[q];

		for (size_t k = 0; p->placed && p->fp && k < p->fp->npad; k++) {
			struct pcb_spot s;
			size_t c = 0;

			if (pcb_net_of(&ck->b, q, k) != n) {
				continue;
			}
			pcb_spot(&ck->b, p, k, &s);
			c = node_at(ck, s.x, s.y, (long)q, (long)k, 0);
			for (size_t i = 0; i < first; i++) {
				if (pcb_spot_dist(&s, ck->nodes[i].x,
						  ck->nodes[i].y) <= SLACK) {
					join(ck, c, i);
				}
			}
		}
	}
}

// Joins the vias of net n to the first nodes that lie in them.
static void via_nodes(struct check *ck, int n, size_t first)
{
	for (size_t v = 0; v < ck->b.nvia; v++) {
		const struct pcb_via *vi = &ck->b.via[v];
		size_t c = 0;

		if (vi->net != n) {
			continue;
		}
		c = node_at(ck, vi->x, vi->y, -1, -1, 1);
		for (size_t i = 0; i < first; i++) {
			if (hypot(ck->nodes[i].x - vi->x,
				  ck->nodes[i].y - vi->y) <=
			    (vi->dia / 2) + SLACK) {
				join(ck, c, i);
			}
		}
	}
}

static void net_graph(struct check *ck, int n)
{
	size_t first = 0;

	ck->nnodes = ck->nedges = 0;
	for (size_t i = 0; i < ck->b.ntrack; i++) {
		const struct pcb_track *t = &ck->b.track_[i];

		for (size_t j = 0; t->net == n && j < t->n; j++) {
			node_at(ck, t->xy[2 * j], t->xy[(2 * j) + 1], -1, -1,
				0);
		}
	}
	first = ck->nnodes; // The points of the tracks are the nodes before.
	for (size_t i = 0; i < ck->b.ntrack; i++) {
		const struct pcb_track *t = &ck->b.track_[i];

		for (size_t j = 1; t->net == n && j < t->n; j++) {
			split_segment(ck, first, &t->xy[(2 * j) - 2],
				      &t->xy[2 * j], t->w);
		}
	}
	pad_nodes(ck, n, first);
	via_nodes(ck, n, first);
}

// The nearest node not done of the graph by d, or -1.
static size_t nearest(const struct check *ck, const double *d, const char *done)
{
	size_t u = (size_t)-1;

	for (size_t i = 0; i < ck->nnodes; i++) {
		if (!done[i] && d[i] < NEAR_FAR &&
		    (u == (size_t)-1 || d[i] < d[u])) {
			u = i;
		}
	}
	return u;
}

// Relaxes the edges of node u of the graph, d its distances and prev the
// node before each.
static void relax(const struct check *ck, size_t u, double *d, size_t *prev)
{
	for (size_t e = 0; e < ck->nedges; e++) {
		size_t a = ck->edge[e][0];
		size_t b = ck->edge[e][1];
		size_t v = 0;

		if (a != u && b != u) {
			continue;
		}
		v = a == u ? b : a;
		if (d[u] + ck->elen[e] < d[v]) {
			d[v] = d[u] + ck->elen[e];
			prev[v] = u;
		}
	}
}

// The node of the graph nearest node src, by Dijkstra's method, that is
// a via or a pad of another part than q, or -1; prev the node before
// each.
static size_t nearest_end(const struct check *ck, size_t src, size_t q,
			  size_t *prev)
{
	double *d = pcb_alloc(ck->nnodes * sizeof *d);
	char *done = pcb_alloc(ck->nnodes);
	size_t goal = (size_t)-1;

	for (size_t i = 0; i < ck->nnodes; i++) {
		d[i] = FAR;
		prev[i] = (size_t)-1;
	}
	d[src] = 0;
	for (;;) {
		size_t u = nearest(ck, d, done);

		if (u == (size_t)-1) {
			break;
		}
		done[u] = 1;
		if (u != src &&
		    (ck->nodes[u].via || (ck->nodes[u].part >= 0 &&
					  ck->nodes[u].part != (long)q))) {
			goal = u;
			break;
		}
		relax(ck, u, d, prev);
	}
	free(d);
	free(done);
	return goal;
}

// The shortest copper of net n from pad at to the nearest pad of another
// part or via of the net: its points, from the far end to the pad, into
// *path; their number, 0 if there is none.
static size_t reach(struct check *ck, int n, struct pcb_end at, double **path)
{
	size_t *prev = NULL;
	size_t src = (size_t)-1;
	size_t goal = (size_t)-1;
	size_t m = 0;

	net_graph(ck, n);
	for (size_t i = 0; i < ck->nnodes; i++) {
		if (ck->nodes[i].part == (long)at.part &&
		    ck->nodes[i].pad == (long)at.pad) {
			src = i;
		}
	}
	if (src == (size_t)-1) {
		return 0;
	}
	prev = pcb_alloc(ck->nnodes * sizeof *prev);
	goal = nearest_end(ck, src, at.part, prev);
	if (goal != (size_t)-1) {
		*path = pcb_alloc(2 * (ck->nnodes + 1) * sizeof **path);
		for (size_t u = goal; u != (size_t)-1; u = prev[u]) {
			(*path)[2 * m] = ck->nodes[u].x;
			(*path)[(2 * m) + 1] = ck->nodes[u].y;
			m++;
		}
	}
	free(prev);
	return m;
}

// The loop through each part of two pins on two nets: from each pad the
// shortest copper of its net to the nearest pad of another part or via
// (a decoupling capacitor to its pin and its ground via, a feedback
// resistor to the amplifier's two pins), closed straight between those
// ends; its area, by the shoelace.
static void loops(struct check *ck)
{
	for (size_t q = 0; q < ck->b.npart; q++) {
		const struct pcb_part *p = &ck->b.part[q];
		int n0 = 0;
		int n1 = 0;
		double *a = NULL;
		double *b = NULL;
		double area = 0;
		size_t na = 0;
		size_t nb = 0;
		size_t m = 0;

		if (!p->placed || !p->fp || p->fp->npad != 2) {
			continue;
		}
		n0 = pcb_net_of(&ck->b, q, 0);
		n1 = pcb_net_of(&ck->b, q, 1);
		if (n0 < 0 || n1 < 0 || n0 == n1) {
			continue;
		}
		na = reach(ck, n0, (struct pcb_end){ q, 0 }, &a);
		nb = reach(ck, n1, (struct pcb_end){ q, 1 }, &b);
		if (na && nb) {
			// a runs from its far end to pad 0, b from its far end to
			// pad 1: a, then b backward, closes the loop.
			double *v = pcb_alloc(2 * (na + nb) * sizeof *v);

			memcpy(v, a, 2 * na * sizeof *v);
			for (size_t i = 0; i < nb; i++) {
				v[2 * (na + i)] = b[2 * (nb - 1 - i)];
				v[(2 * (na + i)) + 1] =
					b[(2 * (nb - 1 - i)) + 1];
			}
			m = na + nb;
			for (size_t i = 0; i < m; i++) {
				size_t j = (i + 1) % m;

				area += (v[2 * i] * v[(2 * j) + 1]) -
					(v[2 * j] * v[(2 * i) + 1]);
			}
			printf("loop %s %s %s %s %.3f\n", p->name, pcb_ref(p),
			       ck->b.net[n0].name, ck->b.net[n1].name,
			       fabs(area) / 2);
			free(v);
		}
		free(a);
		free(b);
	}
}

// How far box o is out from courtyard c on side s (0 above, 1 below, 2
// left, 3 right).
static double side_gap(const double c[4], const double o[4], int s)
{
	switch (s) {
	case 0:
		return o[1] - c[3];
	case 1:
		return c[1] - o[3];
	case 2:
		return c[0] - o[2];
	default:
		return o[0] - c[2];
	}
}

// The free room of a side of courtyard c out to box o: how far o is, if
// it faces that side (0 if it overlaps c), else *room as it was.
static void room_to(const double c[4], const double o[4], int side,
		    double *room)
{
	int across = side == 0 || side == 1; // above, below: along x
	int faces = across ? !(o[2] <= c[0] || o[0] >= c[2]) :
			     !(o[3] <= c[1] || o[1] >= c[3]);
	double d = NAN;

	if (!faces) {
		return;
	}
	if (o[0] < c[2] && o[2] > c[0] && o[1] < c[3] && o[3] > c[1]) {
		*room = 0;
		return;
	}
	d = side_gap(c, o, side);
	if (d >= 0 && d < *room) {
		*room = d;
	}
}

// The room of each side r of courtyard c out to box o.
static void rooms_to(const double c[4], const double o[4], double r[4])
{
	for (int s = 0; s < 4; s++) {
		room_to(c, o, s, &r[s]);
	}
}

// The room of each side r of courtyard c of a part on a side (bottom or
// not) out to part op: its courtyard on that side, its openings, and the
// keep-outs of its holes.
static void part_rooms(const struct pcb *b, const struct pcb_part *op,
		       int bottom, const double c[4], double r[4])
{
	double oc[4];

	pcb_court(op, oc);
	if (op->bottom == bottom) {
		rooms_to(c, oc, r);
	}
	for (size_t k = 0; k < op->fp->npad; k++) {
		struct pcb_end at = { (size_t)(op - b->part), k };
		struct pcb_disk d = { NAN, NAN, NAN };
		double ob[4];

		if (pcb_opening(b, at, bottom, ob)) {
			rooms_to(c, ob, r);
		}
		if (pcb_keepout(b, op, k, &d)) {
			const double kb[4] = { d.x - d.r, d.y - d.r, d.x + d.r,
					       d.y + d.r };

			rooms_to(c, kb, r);
		}
	}
}

// The room of each side r of courtyard c out to the board's edge rule:
// its outline and its cutouts.
static void edge_rooms(const struct pcb *b, const double c[4], double r[4])
{
	const double e = b->edge;
	const double rim[4] = { b->board[3] - e - c[3], c[1] - b->board[1] - e,
				c[0] - b->board[0] - e,
				b->board[2] - e - c[2] };

	for (int s = 0; s < 4; s++) {
		r[s] = fmax(0, fmin(r[s], rim[s]));
	}
	for (size_t k = 0; k < b->ncut; k++) {
		const double *ct = b->cut[k];
		const double cb[4] = { ct[0] - e, ct[1] - e, ct[2] + e,
				       ct[3] + e };

		rooms_to(c, cb, r);
	}
}

// Prints, for each part placed, the free room on each side of its
// courtyard, in mm out to ROOM_MOST: to the courtyards and the openings
// of the mask of the other parts of its side, the keep-outs of the holes
// and the board's edge rule.
static void rooms(const struct check *ck)
{
	static const char *const sides[] = { "above", "below", "left",
					     "right" };
	const struct pcb *b = &ck->b;

	for (size_t q = 0; q < b->npart; q++) {
		const struct pcb_part *p = &b->part[q];
		double c[4];
		double r[4] = { ROOM_MOST, ROOM_MOST, ROOM_MOST, ROOM_MOST };

		if (!p->placed || !p->fp) {
			continue;
		}
		pcb_court(p, c);
		edge_rooms(b, c, r);
		for (size_t o = 0; o < b->npart; o++) {
			const struct pcb_part *op = &b->part[o];

			if (o != q && op->placed && op->fp) {
				part_rooms(b, op, p->bottom, c, r);
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
static void notes(struct check *ck)
{
	for (size_t s = 0; s < ck->b.nstmt; s++) {
		if (!strcmp(ck->b.stmt[s].w[0], "unplaced") &&
		    ck->b.stmt[s].n == 2) {
			printf("%s:%d: note: %s is not placed yet\n",
			       ck->b.file, ck->b.stmt[s].line,
			       ck->b.stmt[s].w[1]);
		}
	}
	for (size_t k = 0; k < ck->b.nnet; k++) {
		if (ck->b.net[k].waits && ck->b.net[k].nend) {
			printf("%s: note: net %s waits for parts not placed\n",
			       ck->b.file, ck->b.net[k].name);
		}
	}
}

// Prints the length of the tracks of each net and its vias.
static void print_lengths(const struct pcb *b)
{
	for (size_t k = 0; k < b->nnet; k++) {
		double len = 0;
		int nv = 0;

		for (size_t t = 0; t < b->ntrack; t++) {
			const double *xy = b->track_[t].xy;

			for (size_t j = 1;
			     b->track_[t].net == (int)k && j < b->track_[t].n;
			     j++) {
				len += hypot(xy[2 * j] - xy[(2 * j) - 2],
					     xy[(2 * j) + 1] - xy[(2 * j) - 1]);
			}
		}
		for (size_t v = 0; v < b->nvia; v++) {
			nv += b->via[v].net == (int)k;
		}
		printf("length %s %.3f vias %d\n", b->net[k].name, len, nv);
	}
}

// Frees the labels and the faults told of ck.
static void free_check(struct check *ck)
{
	for (size_t k = 0; ck->labels && k < ck->b.npart; k++) {
		free(ck->labels[k]);
	}
	free((void *)ck->labels);
	for (size_t k = 0; k < ck->ntold; k++) {
		free(ck->told[k]);
	}
	free((void *)ck->told);
}

int main(int argc, char **argv)
{
	static const char usage[] =
		"usage: pcbcheck [-l] [-r] [-L LIB]... board.pcb < board.lock";
	struct check ctx;
	struct pcb_lib lib = { NULL, 0, NULL, 0 };
	int i = 1;
	int lengths = 0;
	int room = 0;

	for (; i < argc && argv[i][0] == '-'; i++) {
		if (!strcmp(argv[i], "-l")) {
			lengths = 1;
		} else if (!strcmp(argv[i], "-r")) {
			room = 1;
		} else if (!strcmp(argv[i], "-L") && i + 1 < argc) {
			pcb_add_lib(&lib, argv[++i]);
		} else {
			pcb_die("%s", usage);
		}
	}
	if (i != argc - 1) {
		pcb_die("%s", usage);
	}
	memset(&ctx, 0, sizeof ctx);
	pcb_read(&ctx.b, argv[i], &lib, stdin, NULL);
	pcb_tell_copies(&ctx.b);
	gather(&ctx);
	check_copper(&ctx);
	check_viagap(&ctx);
	check_maskweb(&ctx);
	check_thermal(&ctx);
	check_ends(&ctx);
	check_widths(&ctx);
	check_silk(&ctx);
	check_parts(&ctx);
	check_lock(&ctx);
	check_nets(&ctx);
	notes(&ctx);
	if (room) {
		rooms(&ctx);
	}
	if (lengths) {
		print_lengths(&ctx.b);
		loops(&ctx);
	}
	pcb_free_lib(&lib);
	free_check(&ctx);
	return ctx.faults || ctx.b.failed;
}
