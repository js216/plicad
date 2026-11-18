// SPDX-License-Identifier: GPL-3.0-only
// pcbplace.c -- Place the parts of a board as its source says
// Copyright (c) 2026 Jakob Kastelic

// Usage: pcbplace [-v] [-g GROUP] [-L LIB]... board.pcb [LOCK] > new.lock
//
// Reads the source (pcb(5)), its netlist and footprints, and the lock,
// and writes the lock with every part placed.  A part keeps its place
// in the lock if the statement that placed it is unchanged and the
// parts it is placed against kept theirs; any other part is placed
// anew, among the others as they stand, which nothing moves.  So an
// edit of one statement moves that part and those placed against it,
// and nothing else.  Placement is best effort: the same source and
// lock need not give the same places.  The copper of the lock is
// passed on as it is; pcbroute tells what has gone stale.
//
// The statements, in mm and degrees:
//   fix PART at X Y [rot R] [bottom]     where the mechanics put it
//   place PART WAY of THING [GAP] [rot R] [by PIN]
//                                        next to THING (a part, or a
//                                        group: its box), its courtyard
//                                        GAP off (0), centered on it, or
//                                        by PIN its pad of its pin
//   place PART WAY from PIN [rot R]     its pad of the pin's net next
//                                        to the pin, lying out WAY
//   place GROUP WAY from PIN            a block whole, as one part of two
//                                        sides: its box centered on the
//                                        pin, its pads of the pin's net
//                                        toward it
//   place PART|GROUP near THING [WAY] [GAP] [rot R]
//                                        the nearest room by THING (a part,
//                                        group, pin, cutout [N] or edge)
// A place statement of no such form, or a statement whose rot is not
// 0, 90, 180 or 270, places nothing and is a fault, told once, naming
// the word that is wrong.
// No part moves for a designator (pcb_silk).  The fixed parts are
// placed first, the rest in the order their anchors allow.  A part
// without rot is turned the way, of four, that brings its pads nearest
// to the pads of their nets already placed; then it is moved out the
// way it lies until its courtyard is clear of all placed, of the
// keep-outs of the holes (pcbfp(5)) and inside the board.  A group
// laid out alone whose root is fixed keeps clear of the parts fixed
// outside it, where they stand from its root.  What is not moved, a
// part fixed or kept, or a block fixed by its root, and comes on
// another part or a keep-out is a fault.  With -v each placement is
// told on stderr, with the line and the rule it came from, and the
// difficulty of the group laid out: the number of its parts, the parts
// alike on the same nets counted once, and each block one.
//
// A layout is made by groups (blocks): a run lays out every group of a
// block statement alone, the deepest first, then the board, each
// section of the lock in turn (the lock from LOCK, or stdin; a file not
// there is nothing locked yet).  With -g GROUP, the parts of the group
// alone are placed, its root (the part placed by no part of the
// group: fixed, or placed against a part outside it) at the origin, so
// the lock is the group's, relative to its origin; the root placed from
// a pin outside lies with its pad of the pin's net toward it, the way
// back, so the port faces its neighbor.  A statement
//   block GROUP        the group is laid out alone, its section in the lock
//   block GROUP as SRC a copy of block SRC: its sections are SRC's,
//                      renamed, and the statements of SRC's parts hold
// makes the group one rigid body in any layout round it: its parts stand
// as its section has them, the whole moved, unturned, to where its root's
// statement puts its root, then pushed out its way until clear.

#include "pcbsrc.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { DECIMAL = 10 }; // The base of decimal numbers.

#define STEP 0.05 // How far a part moves at a time to find room.
#define TOUCH 1e-9 // Lengths closer than this are one, in mm.
#define SLACK 1e-6 // What the edge lets go of rounding, in mm.
#define FAR 1e300 // Beyond any length.
#define RATS_SHARE 1e-3 // What the ratsnest weighs against a pad's reach.
enum {
	GAP = 0, // Between courtyards, by default: they may touch.
	STEPS_MOST = 4000, // The most steps out a group laid out alone takes.
	TURN = 360, // A whole turn, and a quarter, in degrees.
	QUARTER = 90,
	FIX_WORDS = 5, // The words of fix PART at X Y,
	PLACE_OF_WORDS = 5, // of place PART WAY of|from THING,
	PLACE_GAP_WORDS = 6, // and of place PART WAY of THING GAP.
	OWN_SPARE = 8, // Room in a fault beyond its part and why.
	WHY_LEN = 1024, // Room for why a part is not clear,
	NAME_LEN = 1200, // and why a block is not.
	LINE_LEN = 1100 // Room for a fault of a statement's form.
};
#define LOCK_UNITS 1e4 // The steps of the lock in a mm: 0.1 um.

// What lasts over all passes: whether to tell what is done, and the
// lines of the faults told once.
struct across {
	int verbose;
	int *told;
	size_t ntold;
};

// A block composed here: its group, its lock (laid out alone), its root.
struct block {
	char *group;
	struct pcb lock;
	long root;
	int whole; // Placed whole by place GROUP WAY from PIN.
};

// A pass under way: what lasts over the passes; the board and the
// library of its footprints; the group laid out alone, or NULL; the
// blocks and the block of each part (-1 for none); the statement that
// places each part, and the part it is placed against (-1 for none);
// and the parts the mechanics fix outside a group laid out alone whose
// root they fix too, where they stand from the group's origin: a group
// laid out alone keeps clear of them as the board will.
struct placer {
	struct across *all;
	struct pcb b;
	struct pcb_lib *lib;
	const char *scope;
	struct block *blocks;
	size_t nblocks;
	long *block_of;
	long *stmt_of;
	long *anchor;
	struct pcb_part *fixed_out;
	size_t nfixed_out;
};

// Whether a name is of a group: GROUP.REST.
static int of_group(const char *name, const char *group)
{
	size_t n = strlen(group);

	return !strncmp(name, group, n) && name[n] == '.';
}

static int in_scope(struct placer *pl, size_t k)
{
	return !pl->scope || of_group(pl->b.part[k].name, pl->scope);
}
// A way: the step it moves in x and in y; right, up, left, down.
struct dir {
	int dx, dy;
};
static const struct dir dirs[4] = { { 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, -1 } };

static int way_word(const char *s)
{
	static const char *const w[] = { "right", "up", "left", "down" };

	for (int i = 0; i < 4; i++) {
		if (!strcmp(s, w[i])) {
			return i;
		}
	}
	return -1;
}

static long part_named(struct placer *pl, const char *name)
{
	for (size_t i = 0; i < pl->b.npart; i++) {
		if (!strcmp(pl->b.part[i].name, name)) {
			return (long)i;
		}
	}
	return -1;
}

static int overlap(const double a[4], const double b[4])
{
	return a[0] < b[2] - TOUCH && b[0] < a[2] - TOUCH &&
	       a[1] < b[3] - TOUCH && b[1] < a[3] - TOUCH;
}

// Whether courtyard c of part k comes on the copper of a plated hole
// of part o, which is copper on both sides.
static int on_hole(const struct placer *pl, const double c[4],
		   const struct pcb_part *o)
{
	for (size_t p = 0; o->placed && o->fp && p < o->fp->npad; p++) {
		struct pcb_spot s;
		double h[4];

		if (!o->fp->pad[p].hole) {
			continue;
		}
		pcb_spot(&pl->b, o, p, &s);
		h[0] = s.x - (s.w / 2) - pl->b.clearance;
		h[1] = s.y - (s.h / 2) - pl->b.clearance;
		h[2] = s.x + (s.w / 2) + pl->b.clearance;
		h[3] = s.y + (s.h / 2) + pl->b.clearance;
		if (overlap(c, h)) {
			return 1;
		}
	}
	return 0;
}

// Whether part k stands off the board, or in a cutout (of a group laid
// out alone, never); the fault, if so, goes to why.
static int off_board(const struct placer *pl, size_t k, char *why, size_t n)
{
	double c[4];

	if (pl->scope) {
		return 0;
	}
	pcb_court(&pl->b.part[k], c);
	if (c[0] < pl->b.board[0] || c[1] < pl->b.board[1] ||
	    c[2] > pl->b.board[2] || c[3] > pl->b.board[3]) {
		(void)snprintf(why, n, "off the board");
		return 1;
	}
	{
		const double in[4] = { c[0] + SLACK, c[1] + SLACK, c[2] - SLACK,
				       c[3] - SLACK };

		if (pcb_edge_dist(&pl->b, 1, in, 0, NULL) <= 0) {
			(void)snprintf(why, n, "in a cutout");
			return 1;
		}
	}
	return 0;
}

// Whether part k stands clear of the other parts: its courtyard off
// every other placed part's on its side, off the keep-out of every hole
// and off every plated hole; the fault, if not, goes to why.
static int clear_on(const struct placer *pl, size_t k, char *why, size_t n)
{
	double c[4];

	pcb_court(&pl->b.part[k], c);
	for (size_t i = 0; i < pl->b.npart; i++) {
		const struct pcb_part *o = &pl->b.part[i];

		if (i == k || !o->placed || !o->fp ||
		    (pl->block_of[k] >= 0 &&
		     pl->block_of[i] == pl->block_of[k])) {
			continue;
		}
		if (pcb_clash(&pl->b, &pl->b.part[k], o, why, n)) {
			return 0;
		}
		if (on_hole(pl, c, o)) {
			(void)snprintf(why, n, "on a plated hole of %s (%s)",
				       o->name, pcb_ref(o));
			return 0;
		}
	}
	for (size_t i = 0; i < pl->nfixed_out; i++) {
		if (pcb_clash(&pl->b, &pl->b.part[k], &pl->fixed_out[i], why,
			      n)) {
			return 0;
		}
		if (on_hole(pl, c, &pl->fixed_out[i])) {
			(void)snprintf(why, n, "on a plated hole of %s (%s)",
				       pl->fixed_out[i].name,
				       pcb_ref(&pl->fixed_out[i]));
			return 0;
		}
	}
	return 1;
}

static int clear(struct placer *pl, size_t k)
{
	char why[WHY_LEN];

	return !off_board(pl, k, why, sizeof why) &&
	       clear_on(pl, k, why, sizeof why);
}

// Whether part k, which is not moved, comes on no other part or
// keep-out: the board's edge is pcbcheck's, by its rule.
static int clear_why(struct placer *pl, size_t k, char *why, size_t n)
{
	return clear_on(pl, k, why, n);
}

// The length of the ratsnest of part k as it stands: from each of its
// pads to the nearest placed pad of the same net on another part.
static double ratsnest(struct placer *pl, size_t k)
{
	double sum = 0;

	for (size_t p = 0; p < pl->b.part[k].fp->npad; p++) {
		int n = pcb_net_of(&pl->b, k, p);
		double best = -1;
		struct pcb_spot s;

		if (n < 0 || pl->b.net[n].plane >= 0) {
			continue;
		}
		pcb_spot(&pl->b, &pl->b.part[k], p, &s);
		for (size_t j = 0; j < pl->b.net[n].nend; j++) {
			const struct pcb_end *e = &pl->b.net[n].end[j];
			struct pcb_spot o;
			double d = NAN;

			if (e->part == k || !pl->b.part[e->part].placed) {
				continue;
			}
			pcb_spot(&pl->b, &pl->b.part[e->part], e->pad, &o);
			d = hypot(o.x - s.x, o.y - s.y);
			best = best < 0 || d < best ? d : best;
		}
		sum += best > 0 ? best : 0;
	}
	return sum;
}

// Moves part k out the way until it stands clear; returns 0 if it
// leaves the board first.
static int push(struct placer *pl, size_t k, const struct dir *d)
{
	struct pcb_part *q = &pl->b.part[k];
	double c[4];

	for (int steps = 0; !clear(pl, k); steps++) {
		int out = 0;

		q->x += d->dx * STEP;
		q->y += d->dy * STEP;
		pcb_court(q, c);
		if (pl->scope) {
			out = steps > STEPS_MOST;
		} else {
			out = c[2] < pl->b.board[0] || c[0] > pl->b.board[2] ||
			      c[3] < pl->b.board[1] || c[1] > pl->b.board[3];
		}
		if (out) {
			return 0;
		}
	}
	return 1;
}

// Whether a pad of block b comes within the clearance of a via of
// another block placed, as that block's lock has the via, moved with it;
// the fault, if so, goes to why.
static int on_via(struct placer *pl, size_t b, char *why, size_t n)
{
	for (size_t o = 0; o < pl->nblocks; o++) {
		const struct block *ob = &pl->blocks[o];
		long r = ob->root;
		double dx = NAN;
		double dy = NAN;

		if (o == b || r < 0 || !pl->b.part[r].placed ||
		    !ob->lock.part[r].placed) {
			continue;
		}
		dx = pl->b.part[r].x - ob->lock.part[r].x;
		dy = pl->b.part[r].y - ob->lock.part[r].y;
		for (size_t v = 0; v < ob->lock.nvia; v++) {
			const struct pcb_via *w = &ob->lock.via[v];

			for (size_t i = 0; i < pl->b.npart; i++) {
				for (size_t p = 0; pl->block_of[i] == (long)b &&
						   p < pl->b.part[i].fp->npad;
				     p++) {
					struct pcb_spot s;

					pcb_spot(&pl->b, &pl->b.part[i], p, &s);
					if (pcb_net_of(&pl->b, i, p) !=
						    w->net &&
					    pcb_spot_dist(&s, w->x + dx,
							  w->y + dy) <
						    w->dia / 2 +
							    pl->b.clearance) {
						(void)snprintf(
							why, n,
							"%s (%s): on a via of block %s",
							pl->b.part[i].name,
							pcb_ref(&pl->b.part[i]),
							ob->group);
						return 1;
					}
				}
			}
		}
	}
	return 0;
}

// Whether part i of block b stands clear, of the board too if board is
// set; the fault, if not, goes to why, after the part.
static int block_part_clear(const struct placer *pl, size_t i, int board,
			    char *why, size_t n)
{
	size_t room = 0;
	char *own = NULL;

	if ((!board || !off_board(pl, i, why, n)) && clear_on(pl, i, why, n)) {
		return 1;
	}
	room = strlen(pl->b.part[i].name) + strlen(pcb_ref(&pl->b.part[i])) +
	       strlen(why) + OWN_SPARE;
	own = pcb_alloc(room);
	(void)snprintf(own, room, "%s (%s): %s", pl->b.part[i].name,
		       pcb_ref(&pl->b.part[i]), why);
	(void)snprintf(why, n, "%s", own);
	free(own);
	return 0;
}

// Whether every part of block b stands clear of all outside it (the
// board's edge too, if board is set); the fault, if not, goes to why.
static int block_clear_on(struct placer *pl, size_t b, int board, char *why,
			  size_t n)
{
	if (on_via(pl, b, why, n)) {
		return 0;
	}
	for (size_t i = 0; i < pl->b.npart; i++) {
		if (pl->block_of[i] == (long)b &&
		    !block_part_clear(pl, i, board, why, n)) {
			return 0;
		}
	}
	return 1;
}

static int block_clear(struct placer *pl, size_t b)
{
	char why[NAME_LEN];

	return block_clear_on(pl, b, 1, why, sizeof why);
}

static int block_clear_why(struct placer *pl, size_t b, char *why, size_t n)
{
	return block_clear_on(pl, b, 0, why, n);
}

// Moves block b out the way, rigidly, until it stands clear; returns 0
// if it leaves the board (200 mm out of a group).
static int push_block(struct placer *pl, size_t b, const struct dir *d)
{
	for (int steps = 0; !block_clear(pl, b); steps++) {
		if (!d || steps > STEPS_MOST) {
			return 0;
		}
		for (size_t i = 0; i < pl->b.npart; i++) {
			if (pl->block_of[i] == (long)b) {
				pl->b.part[i].x += d->dx * STEP;
				pl->b.part[i].y += d->dy * STEP;
			}
		}
	}
	return 1;
}

// The box of a thing placed against: the courtyard of the part, else
// the box of the courtyards of the group's parts placed.
static void thing_box(struct placer *pl, const char *th, double d[4])
{
	long a = part_named(pl, th);

	if (a >= 0) {
		pcb_court(&pl->b.part[a], d);
		return;
	}
	d[0] = d[1] = FAR;
	d[2] = d[3] = -FAR;
	for (size_t i = 0; i < pl->b.npart; i++) {
		double e[4];

		if (!of_group(pl->b.part[i].name, th) || !pl->b.part[i].fp ||
		    !pl->b.part[i].placed) {
			continue;
		}
		pcb_court(&pl->b.part[i], e);
		d[0] = fmin(d[0], e[0]);
		d[1] = fmin(d[1], e[1]);
		d[2] = fmax(d[2], e[2]);
		d[3] = fmax(d[3], e[3]);
	}
}

// What a part is placed against: the thing th, anchored on part a; the
// way of it, the gap off it, the turn, and the pad of the part centered
// across the way (-1 for its origin).
struct against {
	const char *th;
	size_t a;
	int way;
	double gap;
	int rot;
	long by;
};

// Places part k against ag, centered across the way.
static void place_of(struct placer *pl, size_t k, const struct against *ag)
{
	struct pcb_part *q = &pl->b.part[k];
	struct pcb_spot own = { 0 };
	double c[4];
	double d[4];

	q->rot = ag->rot;
	q->bottom = pl->b.part[ag->a].bottom;
	q->x = q->y = 0;
	pcb_court(q, c);
	thing_box(pl, ag->th, d);
	if (ag->by >= 0) {
		pcb_spot(&pl->b, q, (size_t)ag->by, &own);
	}
	q->x = ((d[0] + d[2]) / 2) - own.x;
	q->y = ((d[1] + d[3]) / 2) - own.y;
	if (ag->way == 0) {
		q->x = d[2] + ag->gap - c[0];
	} else if (ag->way == 2) {
		q->x = d[0] - ag->gap - c[2];
	} else if (ag->way == 1) {
		q->y = d[3] + ag->gap - c[1];
	} else if (ag->way == 3) {
		q->y = d[1] - ag->gap - c[3];
	}
}

// A way out and a turn.
struct lay {
	int way;
	int rot;
};

// Places part k with its pad of the net of the pin next to the pin,
// lying out the way of ly, turned as it says.
static void place_at(struct placer *pl, size_t k, struct pcb_end pin_at,
		     struct lay ly)
{
	struct pcb_part *q = &pl->b.part[k];
	const struct pcb_part *anchor = &pl->b.part[pin_at.part];
	struct pcb_spot pin;
	struct pcb_spot own;
	int n = pcb_net_of(&pl->b, pin_at.part, pin_at.pad);
	int way = ly.way;
	size_t mine = 0;
	double c[4];
	double d[4];

	for (size_t p = 0; p < q->fp->npad; p++) {
		if (n >= 0 && pcb_net_of(&pl->b, k, p) == n) {
			mine = p;
			break;
		}
	}
	q->rot = ly.rot;
	q->bottom = anchor->bottom;
	q->x = q->y = 0;
	pcb_spot(&pl->b, q, mine, &own);
	pcb_spot(&pl->b, anchor, pin_at.pad, &pin);
	pcb_court(q, c);
	pcb_court(anchor, d);
	// The pad lines up with the pin, and the courtyard starts at the
	// anchor's.
	if (way == 1 || way == 3) {
		q->x = pin.x - own.x;
		q->y = way == 1 ? d[3] + GAP - c[1] : d[1] - GAP - c[3];
	} else {
		q->y = pin.y - own.y;
		q->x = way == 0 ? d[2] + GAP - c[0] : d[0] - GAP - c[2];
	}
}

struct placing;

// The turn of the root k of the group laid out alone, placed from a pin
// outside the way way: its pad of the pin's net the way back, toward the
// pin (the port facing its neighbor); -1 if it has no pad of that net.
static int root_turn(struct placer *pl, size_t k, struct pcb_end pin, int way)
{
	struct pcb_part *q = &pl->b.part[k];
	int net = pcb_net_of(&pl->b, pin.part, pin.pad);
	double best = -FAR;
	int rot = -1;

	for (int r = 0; r < TURN; r += QUARTER) {
		q->rot = r;
		for (size_t p = 0; p < q->fp->npad; p++) {
			struct pcb_spot own;
			double back = NAN;

			if (pcb_net_of(&pl->b, k, p) != net) {
				continue;
			}
			pcb_spot(&pl->b, q, p, &own);
			back = -((own.x * dirs[way].dx) +
				 (own.y * dirs[way].dy));
			if (back > best + TOUCH) {
				best = back;
				rot = r;
			}
			break;
		}
	}
	return rot;
}

// The root of the group laid out alone, at its origin: fixed, as said;
// placed from a pin outside, turned so its pad of the pin's net lies the
// way back, toward the pin (the port facing its neighbor); else as said,
// or unturned.
static void place_root(struct placer *pl, const struct placing *pg);

#define NEAR_STEP 0.1 // The step of the search of near.
enum {
	NEAR_FAR = 1000 // Its steps out from what it is near, at most.
};

// What a near statement is near: its box, the way (-1 any) and the
// gap; inside for the edge; and the word after what it names.
struct near_of {
	double t[4];
	int way;
	double gap;
	int inside;
	int i;
};

static int is_near(const struct pcb_stmt *st)
{
	return !strcmp(st->w[0], "place") && st->n >= 4 &&
	       !strcmp(st->w[2], "near");
}

// Tells a fault of a statement once over all passes, the first only.
static void fault_once(struct placer *pl, const struct pcb_stmt *st,
		       const char *what)
{
	pl->b.failed = 1;
	for (size_t i = 0; i < pl->all->ntold; i++) {
		if (pl->all->told[i] == st->line) {
			return;
		}
	}
	pl->all->told =
		pcb_grow(pl->all->told, pl->all->ntold, sizeof *pl->all->told);
	pl->all->told[pl->all->ntold++] = st->line;
	pcb_error(&pl->b, st->line, "%s", what);
}

// A fault of the statement's form, after its first word.
static void form_fault(struct placer *pl, const struct pcb_stmt *st,
		       const char *what)
{
	char line[NAME_LEN];

	(void)snprintf(line, sizeof line, "%s: %s", st->w[0], what);
	fault_once(pl, st, line);
}

// The fault of a pin that is not there.
static void no_pin(struct placer *pl, const struct pcb_stmt *st,
		   const char *pin)
{
	char line[LINE_LEN];

	(void)snprintf(line, sizeof line, "no pin %s", pin);
	fault_once(pl, st, line);
}

static int number(const char *s)
{
	char *e = NULL;

	(void)strtod(s, &e);
	return e != s && !*e;
}

// Whether a turn is one of the four: 0, 90, 180 or 270.
static int turn_ok(const char *s)
{
	return !strcmp(s, "0") || !strcmp(s, "90") || !strcmp(s, "180") ||
	       !strcmp(s, "270");
}

// The pad of part k that the by PIN of its place statement names, or
// -1 if it names none of k's pins.
static long by_pad(struct placer *pl, const struct pcb_stmt *st, size_t k)
{
	struct pcb_end pa = { 0, 0 };

	for (int i = 0; i + 1 < st->n; i++) {
		if (!strcmp(st->w[i], "by") &&
		    pcb_pin(&pl->b, st->w[i + 1], &pa) && pa.part == k) {
			return (long)pa.pad;
		}
	}
	return -1;
}

// Whether each turn of statement st is one of the four; if not, the
// word that is wrong is told.
static int turns_ok(struct placer *pl, const struct pcb_stmt *st)
{
	for (int i = 0; i + 1 < st->n; i++) {
		if (!strcmp(st->w[i], "rot") && !turn_ok(st->w[i + 1])) {
			char what[LINE_LEN];

			(void)snprintf(what, sizeof what,
				       "'%s' is not a turn (0, 90, 180, 270)",
				       st->w[i + 1]);
			form_fault(pl, st, what);
			return 0;
		}
	}
	return 1;
}

// Whether the pin after by, word by of place statement st, is none of
// its part's.
static int by_wrong(struct placer *pl, const struct pcb_stmt *st)
{
	long k = part_named(pl, st->w[1]);

	return k >= 0 && pl->b.part[k].fp && by_pad(pl, st, (size_t)k) < 0;
}

// The word by of statement st, or -1.
static int by_word(const struct pcb_stmt *st)
{
	int by = -1;

	for (int i = 0; i < st->n; i++) {
		by = !strcmp(st->w[i], "by") ? i : by;
	}
	return by;
}

// Whether a place statement of n words before rot and by has a form of
// pcb(5); if not, the word that is wrong is told.
static int place_form_ok(struct placer *pl, const struct pcb_stmt *st, int n)
{
	int by = by_word(st);
	const char *of = st->n > 3 ? st->w[3] : "";
	int is_of = *of == 'o';
	int most = is_of ? PLACE_GAP_WORDS : PLACE_OF_WORDS;
	char what[LINE_LEN];

	if (n < 2) {
		form_fault(pl, st, "missing the part");
	} else if (n < 3) {
		form_fault(pl, st, "missing a way, or 'near'");
	} else if (!strcmp(st->w[2], "near")) {
		form_fault(pl, st, "missing the thing after 'near'");
	} else if (way_word(st->w[2]) < 0) {
		(void)snprintf(what, sizeof what,
			       "'%s' is not a way (right, left, up, down)",
			       st->w[2]);
		form_fault(pl, st, what);
	} else if (strcmp(of, "of") != 0 && strcmp(of, "from") != 0) {
		form_fault(pl, st, "missing 'of', 'from' or 'near'");
	} else if (n < PLACE_OF_WORDS) {
		form_fault(pl, st,
			   is_of ? "missing the part after 'of'" :
				   "missing the pin after 'from'");
	} else if (is_of && n > PLACE_OF_WORDS &&
		   !number(st->w[PLACE_OF_WORDS])) {
		(void)snprintf(what, sizeof what, "'%s' is not a gap",
			       st->w[PLACE_OF_WORDS]);
		form_fault(pl, st, what);
	} else if (n > most) {
		(void)snprintf(what, sizeof what, "'%s' is one word too many",
			       st->w[most]);
		form_fault(pl, st, what);
	} else if (by >= 0 && by_wrong(pl, st)) {
		(void)snprintf(what, sizeof what, "'%s' is not a pin of %s",
			       st->w[by + 1], st->w[1]);
		form_fault(pl, st, what);
	} else {
		return 1;
	}
	return 0;
}

// Whether a fix or place statement turns its part one of the four ways
// and, a place statement, has a form of pcb(5); if not, the word that
// is wrong is told.  Of near, near_thing tells the rest; of fix, place.
static int form_ok(struct placer *pl, const struct pcb_stmt *st)
{
	int n = st->n;
	int by = by_word(st);
	const char *of = n > 3 ? st->w[3] : "";

	if (!turns_ok(pl, st)) {
		return 0;
	}
	if (by >= 0 &&
	    (strcmp(st->w[0], "place") != 0 || strcmp(of, "of") != 0)) {
		form_fault(pl, st, "'by' goes with 'of' only");
		return 0;
	}
	if (strcmp(st->w[0], "place") != 0 || is_near(st)) {
		return 1;
	}
	for (int i = 0; i < 2; i++) {
		if (n >= 2 && (!strcmp(st->w[n - 2], "rot") ||
			       !strcmp(st->w[n - 2], "by"))) {
			n -= 2;
		}
	}
	return place_form_ok(pl, st, n);
}

// The parts of a group, a dot after its name.
static int group_has(struct placer *pl, const char *g)
{
	for (size_t i = 0; i < pl->b.npart; i++) {
		if (of_group(pl->b.part[i].name, g) && pl->b.part[i].fp) {
			return 1;
		}
	}
	return 0;
}

// The box t of the group th: of its parts' courtyards placed; 0, the
// fault told, if none is.
static int group_box(struct placer *pl, const struct pcb_stmt *st,
		     const char *th, double t[4])
{
	t[0] = t[1] = FAR;
	t[2] = t[3] = -FAR;
	for (size_t q = 0; q < pl->b.npart; q++) {
		double c[4];

		if (!of_group(pl->b.part[q].name, th) ||
		    !pl->b.part[q].placed || !pl->b.part[q].fp) {
			continue;
		}
		pcb_court(&pl->b.part[q], c);
		t[0] = fmin(t[0], c[0]);
		t[1] = fmin(t[1], c[1]);
		t[2] = fmax(t[2], c[2]);
		t[3] = fmax(t[3], c[3]);
	}
	if (t[2] < t[0]) {
		pcb_error(&pl->b, st->line, "no part of %s is placed", th);
		return 0;
	}
	return 1;
}

// The box t of what a near statement of n words is near: the edge (its
// room inside), cutout [N], a part, a pin or a group; the words after
// it start at the word i of no.  Returns 0, the fault told, if it is none.
static int near_box(struct placer *pl, const struct pcb_stmt *st, int n,
		    struct near_of *no)
{
	const char *th = st->w[3];
	double *t = no->t;
	long k = part_named(pl, th);
	struct pcb_end pa = { 0, 0 };

	t[0] = t[1] = FAR;
	t[2] = t[3] = -FAR;
	if (!strcmp(th, "edge")) {
		memcpy(t, pl->b.board, sizeof pl->b.board);
		no->inside = 1;
	} else if (!strcmp(th, "cutout")) {
		int c = 1;

		if (no->i < n && strspn(st->w[no->i], "0123456789") ==
					 strlen(st->w[no->i])) {
			c = (int)strtol(st->w[no->i++], NULL, DECIMAL);
		}
		if (c < 1 || (size_t)c > pl->b.ncut) {
			pcb_error(&pl->b, st->line, "no cutout %d", c);
			return 0;
		}
		memcpy(t, pl->b.cut[c - 1], sizeof pl->b.cut[0]);
	} else if (k >= 0 && pl->b.part[k].fp) {
		if (!pl->b.part[k].placed) {
			pcb_error(&pl->b, st->line, "%s is not placed", th);
			return 0;
		}
		pcb_court(&pl->b.part[k], t);
	} else if (pcb_pin(&pl->b, th, &pa)) {
		struct pcb_spot sp;

		if (!pl->b.part[pa.part].placed) {
			pcb_error(&pl->b, st->line, "%s is not placed", th);
			return 0;
		}
		pcb_spot(&pl->b, &pl->b.part[pa.part], pa.pad, &sp);
		t[0] = sp.x - (sp.w / 2);
		t[1] = sp.y - (sp.h / 2);
		t[2] = sp.x + (sp.w / 2);
		t[3] = sp.y + (sp.h / 2);
	} else if (group_has(pl, th)) {
		return group_box(pl, st, th, t);
	} else {
		pcb_error(&pl->b, st->line,
			  "place PART|GROUP near PART|GROUP|PIN|cutout "
			  "[N]|edge [WAY] [GAP]");
		return 0;
	}
	return 1;
}

// What a near statement is near, as a box, and the words after it: the
// way (-1 any) and the gap; inside is set for the edge, whose room is
// inside it.  Returns 0, the fault told, if it is not so.
static int near_thing(struct placer *pl, const struct pcb_stmt *st, int n,
		      struct near_of *no)
{
	no->way = -1;
	no->gap = 0;
	no->inside = 0;
	no->i = 4;
	if (!near_box(pl, st, n, no)) {
		return 0;
	}
	if (no->i < n && way_word(st->w[no->i]) >= 0) {
		no->way = way_word(st->w[no->i++]);
	}
	if (no->i < n) {
		char *e = NULL;

		no->gap = strtod(st->w[no->i], &e);
		no->i += *e || !(no->gap >= 0) ? 0 : 1;
	}
	if (no->i != n) {
		pcb_error(&pl->b, st->line,
			  "place PART|GROUP near PART|GROUP|PIN|cutout "
			  "[N]|edge [WAY] [GAP]");
		return 0;
	}
	return 1;
}

// What moves: part k, or block b if b is 0 or more (its root k).
struct mover {
	size_t k;
	long b;
};

static int set_clear(struct placer *pl, struct mover m)
{
	return m.b >= 0 ? block_clear(pl, (size_t)m.b) : clear(pl, m.k);
}

// Moves part k, or block b, of m by (dx, dy).
static void set_move(struct placer *pl, struct mover m, double dx, double dy)
{
	for (size_t i = 0; i < pl->b.npart; i++) {
		if (m.b >= 0 ? pl->block_of[i] == m.b : i == m.k) {
			pl->b.part[i].x += dx;
			pl->b.part[i].y += dy;
		}
	}
}

// The box of the courtyards of what m moves, into c.
static void mover_box(const struct placer *pl, struct mover m, double c[4])
{
	c[0] = c[1] = FAR;
	c[2] = c[3] = -FAR;
	for (size_t i = 0; i < pl->b.npart; i++) {
		double e[4];

		if (m.b >= 0 ? pl->block_of[i] != m.b : i != m.k) {
			continue;
		}
		pcb_court(&pl->b.part[i], e);
		c[0] = fmin(c[0], e[0]);
		c[1] = fmin(c[1], e[1]);
		c[2] = fmax(c[2], e[2]);
		c[3] = fmax(c[3], e[3]);
	}
}

// A spot a near statement tries for a box w by h: on side sd of the
// thing, d out from it (in, of the edge), along it by along.
struct near_try {
	int sd;
	double d;
	double along;
	int inside;
	double w, h;
};

// The lower left of the box of nt where it would stand at its spot by
// the box t of the thing: past the side, or inside the edge.
static void near_spot(const double t[4], const struct near_try *nt,
		      double xy[2])
{
	double w = nt->w;
	double h = nt->h;
	double d = nt->d;

	xy[0] = ((t[0] + t[2] - w) / 2) + nt->along;
	xy[1] = ((t[1] + t[3] - h) / 2) + nt->along;
	if (nt->sd == 0) {
		xy[0] = nt->inside ? t[2] - d - w : t[2] + d;
	} else if (nt->sd == 2) {
		xy[0] = nt->inside ? t[0] + d : t[0] - d - w;
	} else if (nt->sd == 1) {
		xy[1] = nt->inside ? t[3] - d - h : t[3] + d;
	} else {
		xy[1] = nt->inside ? t[1] + d : t[1] - d - h;
	}
}

// Tries what m moves, of box c, at the spots along the sides of the
// thing of box t that the way says (any, if -1); keeps the first clear.
// Returns whether one is.
static int near_along(struct placer *pl, struct mover m, const double t[4],
		      const double c[4], struct near_try *nt, int way)
{
	double reach = (fmax(t[2] - t[0], t[3] - t[1]) / 2) +
		       fmax(c[2] - c[0], c[3] - c[1]);

	for (int ti = 0; ti * NEAR_STEP / 2 <= reach; ti++) {
		int half = (ti + 1) / 2;

		nt->along = (ti % 2 ? 1 : -1) * half * NEAR_STEP;
		for (int sd = 0; sd < 4; sd++) {
			double xy[2];

			if (way >= 0 && sd != way) {
				continue;
			}
			nt->sd = sd;
			near_spot(t, nt, xy);
			set_move(pl, m, xy[0] - c[0], xy[1] - c[1]);
			if (set_clear(pl, m)) {
				return 1;
			}
			set_move(pl, m, c[0] - xy[0], c[1] - xy[1]);
		}
	}
	return 0;
}

// Carries out a near statement for what m moves: the nearest spot where
// the courtyard (of a block, the box of its parts' courtyards) is
// clear, searching outward from what it is near, at the gap from it at
// least, on the side the way says or any; along the side, from its
// middle out.  Of the edge, the spot is inside it.  Returns 0 if none
// is.
static int near_search(struct placer *pl, struct mover m,
		       const struct pcb_stmt *st, int n)
{
	struct near_of no;
	double c[4];
	struct near_try nt = { 0, 0, 0, 0, 0, 0 };

	if (!near_thing(pl, st, n, &no)) {
		return 0;
	}
	nt.inside = no.inside;
	mover_box(pl, m, c);
	nt.w = c[2] - c[0];
	nt.h = c[3] - c[1];
	for (int di = 0; di <= NEAR_FAR; di++) {
		nt.d = no.gap + (di * NEAR_STEP);
		if (near_along(pl, m, no.t, c, &nt, no.way)) {
			return 1;
		}
	}
	return 0;
}

// A part being placed by its statement st: the turn it says (-1 for
// none), the words before rot or by, its way (-1 for none), and how it
// was placed, for -v.
struct placing {
	size_t k;
	const struct pcb_stmt *st;
	int rot;
	int n;
	int way;
	const char *how;
};

static void place_root(struct placer *pl, const struct placing *pg)
{
	const struct pcb_stmt *st = pg->st;
	struct pcb_part *q = &pl->b.part[pg->k];
	struct pcb_end pin = { 0, 0 };
	int rot = pg->rot;

	q->x = q->y = 0;
	q->rot = rot < 0 ? 0 : rot;
	q->bottom = !strcmp(st->w[0], "fix") &&
		    !strcmp(st->w[st->n - 1], "bottom");
	if (rot < 0 && pg->n >= PLACE_OF_WORDS && !strcmp(st->w[3], "from") &&
	    pcb_pin(&pl->b, st->w[4], &pin) && way_word(st->w[2]) >= 0) {
		rot = root_turn(pl, pg->k, pin, way_word(st->w[2]));
		q->rot = rot < 0 ? 0 : rot;
	}
}

// fix PART at X Y [rot R] [bottom].
static int place_fix(struct placer *pl, struct placing *pg)
{
	const struct pcb_stmt *st = pg->st;
	struct pcb_part *q = &pl->b.part[pg->k];

	if (pg->n < FIX_WORDS || strcmp(st->w[2], "at") != 0) {
		pcb_error(&pl->b, st->line, "fix PART at X Y [rot R] [bottom]");
		return 0;
	}
	q->x = strtod(st->w[3], NULL);
	q->y = strtod(st->w[4], NULL);
	q->rot = pg->rot < 0 ? 0 : pg->rot;
	q->bottom = !strcmp(st->w[st->n - 1], "bottom");
	pg->how = "fixed";
	return 1;
}

// place PART near THING: the nearest room; a block's root (block set)
// is left for the block to search whole.
static int place_near(struct placer *pl, struct placing *pg, int block)
{
	struct pcb_part *q = &pl->b.part[pg->k];
	struct mover m = { pg->k, -1 };

	q->rot = pg->rot < 0 ? 0 : pg->rot;
	q->bottom = 0;
	q->x = q->y = 0;
	q->placed = 1;
	if (block) {
		return 1;
	}
	if (!near_search(pl, m, pg->st, pg->n)) {
		q->placed = 0;
		pcb_error(&pl->b, pg->st->line, "no room for %s (%s) near %s",
			  q->name, pcb_ref(q), pg->st->w[3]);
		return 0;
	}
	pg->how = "nearest room";
	return 1;
}

// place PART WAY of THING [GAP]: as said, or turned the way of the
// shortest ratsnest.
static int place_of_stmt(struct placer *pl, struct placing *pg)
{
	const struct pcb_stmt *st = pg->st;
	size_t k = pg->k;
	struct against ag = { st->w[4], (size_t)pl->anchor[k], 0, GAP,
			      0,	by_pad(pl, st, k) };
	double best = FAR;
	int br = 0;

	pg->way = way_word(st->w[2]);
	ag.way = pg->way;
	if (pg->way < 0) {
		pcb_error(&pl->b, st->line, "place PART WAY of THING [GAP]");
		return 0;
	}
	if (pg->n == PLACE_GAP_WORDS) {
		ag.gap = strtod(st->w[PLACE_GAP_WORDS - 1], NULL);
	}
	if (pg->rot >= 0) {
		ag.rot = pg->rot;
		place_of(pl, k, &ag);
		pg->how = "said";
		return 1;
	}
	for (int r = 0; r < TURN; r += QUARTER) {
		double len = NAN;

		ag.rot = r;
		place_of(pl, k, &ag);
		len = push(pl, k, &dirs[pg->way]) ? ratsnest(pl, k) : FAR;
		if (len < best - TOUCH) {
			best = len;
			br = r;
		}
	}
	ag.rot = br;
	place_of(pl, k, &ag);
	pg->how = "shortest ratsnest";
	return 1;
}

// The turn of part k placed from pin the way way that lays its pad of
// the pin's net nearest the pin, the shorter ratsnest after.
static int nearest_turn(struct placer *pl, size_t k, struct pcb_end pin,
			int way)
{
	struct pcb_part *q = &pl->b.part[k];
	int net = pcb_net_of(&pl->b, pin.part, pin.pad);
	double best = FAR;
	int br = 0;

	for (int r = 0; r < TURN; r += QUARTER) {
		struct pcb_spot ps;
		struct pcb_spot own;
		double len = 0;

		place_at(pl, k, pin, (struct lay){ way, r });
		pcb_spot(&pl->b, &pl->b.part[pin.part], pin.pad, &ps);
		for (size_t p = 0; p < q->fp->npad; p++) {
			if (pcb_net_of(&pl->b, k, p) == net) {
				pcb_spot(&pl->b, q, p, &own);
				len = hypot(own.x - ps.x, own.y - ps.y);
				break;
			}
		}
		len += ratsnest(pl, k) * RATS_SHARE;
		if (len < best - TOUCH) {
			best = len;
			br = r;
		}
	}
	return br;
}

// place PART WAY from PIN: lying along the way, the pad of the net
// toward the pin, unless turned as said.
static int place_from_stmt(struct placer *pl, struct placing *pg)
{
	const struct pcb_stmt *st = pg->st;
	struct pcb_end pin = { 0, 0 };

	if (!pcb_pin(&pl->b, st->w[4], &pin)) {
		pcb_error(&pl->b, st->line, "no pin %s", st->w[4]);
		return 0;
	}
	pg->way = way_word(st->w[2]);
	if (pg->way < 0) {
		pcb_error(&pl->b, st->line, "place PART WAY from PIN");
		return 0;
	}
	if (pg->rot < 0) {
		pg->rot = nearest_turn(pl, pg->k, pin, pg->way);
		pg->how = "its pad nearest the pin";
	} else {
		pg->how = "said";
	}
	place_at(pl, pg->k, pin, (struct lay){ pg->way, pg->rot });
	return 1;
}

// The statement of part k: its turn, and the words before rot or by.
static struct placing placing_of(const struct placer *pl, size_t k)
{
	const struct pcb_stmt *st = &pl->b.stmt[pl->stmt_of[k]];
	struct placing pg = { k, st, -1, st->n, -1, "" };

	for (int i = 0; i + 1 < st->n; i++) {
		if (!strcmp(st->w[i], "rot")) {
			pg.rot = (int)strtol(st->w[i + 1], NULL, DECIMAL);
		}
		if (!strcmp(st->w[i], "rot") || !strcmp(st->w[i], "by")) {
			pg.n = i < pg.n ? i : pg.n;
		}
	}
	return pg;
}

// Carries out the placing pg by its kind of statement; returns 0 if it
// cannot be.
static int place_by(struct placer *pl, struct placing *pg, int block)
{
	const struct pcb_stmt *st = pg->st;
	size_t k = pg->k;

	if (pl->scope &&
	    (pl->anchor[k] < 0 || !in_scope(pl, (size_t)pl->anchor[k]))) {
		place_root(pl, pg);
		pg->how = "the root of the group, at its origin";
		return 1;
	}
	if (!strcmp(st->w[0], "fix")) {
		return place_fix(pl, pg);
	}
	if (is_near(st)) {
		return place_near(pl, pg, block);
	}
	if (pg->n >= PLACE_OF_WORDS && !strcmp(st->w[3], "of")) {
		return place_of_stmt(pl, pg);
	}
	if (pg->n >= PLACE_OF_WORDS && !strcmp(st->w[3], "from")) {
		return place_from_stmt(pl, pg);
	}
	pcb_error(&pl->b, st->line, "place PART WAY of THING, or WAY from PIN");
	return 0;
}

// Carries out the statement of part k, turned as *block_rot says if it
// is given (the root of a block, whose turn its lock has); a block's
// root is not pushed here (the block is, whole), and its way goes to
// *way_out.  Returns 0 if it cannot be.
static int place(struct placer *pl, size_t k, const int *block_rot,
		 int *way_out)
{
	static const char *const ways[] = { "right", "up", "left", "down" };
	struct placing pg = placing_of(pl, k);
	struct pcb_part *q = &pl->b.part[k];
	const struct pcb_stmt *st = pg.st;

	if (block_rot) {
		pg.rot = *block_rot;
	}
	if (!place_by(pl, &pg, way_out != NULL)) {
		return 0;
	}
	q->placed = 1;
	if (way_out) {
		*way_out = pg.way;
		return 1;
	}
	if (pg.way >= 0 && !push(pl, k, &dirs[pg.way])) {
		q->placed = 0;
		pcb_error(&pl->b, st->line,
			  "no room for %s (%s) %s of its anchor", q->name,
			  pcb_ref(q), ways[pg.way]);
		return 0;
	}
	free(q->by);
	q->by = pcb_by(&pl->b, k, st);
	if (pl->all->verbose) {
		(void)fprintf(stderr,
			      "%s:%d: %s (%s) at %.3f %.3f rot %d: %s\n",
			      pl->b.file, st->line, q->name, pcb_ref(q), q->x,
			      q->y, q->rot, pg.how);
	}
	return 1;
}

// Moves block b, placed whole from a pin by statement st the way way,
// to stand as one part of two sides would: the box of its courtyards
// centered on the pin across the way, starting at the anchor's
// courtyard along it.
static void center_block(struct placer *pl, size_t b, const struct pcb_stmt *st,
			 int way)
{
	struct pcb_end pa = { 0, 0 };
	struct pcb_spot pin;
	double c[4] = { FAR, FAR, -FAR, -FAR };
	double d[4];
	double dx = NAN;
	double dy = NAN;

	if (!pcb_pin(&pl->b, st->w[4], &pa) || way < 0) {
		return;
	}
	pcb_spot(&pl->b, &pl->b.part[pa.part], pa.pad, &pin);
	pcb_court(&pl->b.part[pa.part], d);
	for (size_t i = 0; i < pl->b.npart; i++) {
		double e[4];

		if (pl->block_of[i] != (long)b) {
			continue;
		}
		pcb_court(&pl->b.part[i], e);
		c[0] = fmin(c[0], e[0]);
		c[1] = fmin(c[1], e[1]);
		c[2] = fmax(c[2], e[2]);
		c[3] = fmax(c[3], e[3]);
	}
	if (c[0] > c[2]) {
		return;
	}
	if (way == 1 || way == 3) {
		dx = pin.x - (c[0] + c[2]) / 2;
		dy = way == 1 ? d[3] + GAP - c[1] : d[1] - GAP - c[3];
	} else {
		dy = pin.y - (c[1] + c[3]) / 2;
		dx = way == 0 ? d[2] + GAP - c[0] : d[0] - GAP - c[2];
	}
	// By whole steps of the lock's 0.1 um, as the block moves.
	dx = round(dx * LOCK_UNITS) / LOCK_UNITS;
	dy = round(dy * LOCK_UNITS) / LOCK_UNITS;
	for (size_t i = 0; i < pl->b.npart; i++) {
		if (pl->block_of[i] == (long)b) {
			pl->b.part[i].x += dx;
			pl->b.part[i].y += dy;
		}
	}
}

// Places block b: its root by its statement, turned as its lock has it;
// its parts as its lock has them, moved with the root, unturned; then
// pushed out the root's way until it stands clear.
static int place_block(struct placer *pl, size_t b)
{
	const struct block *bl = &pl->blocks[b];
	size_t r = (size_t)bl->root;
	const struct pcb_part *lr = &bl->lock.part[r];
	double tx = NAN;
	double ty = NAN;
	int way = -1;

	if (!lr->placed) {
		pcb_error(&pl->b, pl->b.stmt[pl->stmt_of[r]].line,
			  "block %s: its lock does not place %s (%s)",
			  bl->group, pl->b.part[r].name,
			  pcb_ref(&pl->b.part[r]));
		return 0;
	}
	if (!place(pl, r, &lr->rot, &way)) {
		return 0;
	}
	// A move by whole steps of the lock's 0.1 um, so the parts stand
	// to the digit as the block's lock has them, relative to each other.
	tx = round((pl->b.part[r].x - lr->x) * LOCK_UNITS) / LOCK_UNITS;
	ty = round((pl->b.part[r].y - lr->y) * LOCK_UNITS) / LOCK_UNITS;
	for (size_t i = 0; i < pl->b.npart; i++) {
		const struct pcb_part *l = &bl->lock.part[i];

		if (pl->block_of[i] != (long)b) {
			continue;
		}
		if (!l->placed) {
			pcb_error(&pl->b, 0,
				  "block %s: its lock does not place %s",
				  bl->group, pl->b.part[i].name);
			continue;
		}
		pl->b.part[i].x = l->x + tx;
		pl->b.part[i].y = l->y + ty;
		pl->b.part[i].rot = l->rot;
		pl->b.part[i].bottom = l->bottom;
		pl->b.part[i].placed = 1;
		free(pl->b.part[i].by);
		pl->b.part[i].by = pcb_strdup(l->by ? l->by : "by");
	}
	if (bl->whole) {
		center_block(pl, b, &pl->b.stmt[pl->stmt_of[r]], way);
	}
	if (is_near(&pl->b.stmt[pl->stmt_of[r]])) {
		const struct pcb_stmt *st = &pl->b.stmt[pl->stmt_of[r]];
		int n = st->n;

		for (int i = 0; i + 1 < st->n; i++) {
			n = !strcmp(st->w[i], "rot") ? i : n;
		}
		if (!near_search(pl, (struct mover){ r, (long)b }, st, n)) {
			pcb_error(&pl->b, st->line,
				  "no room for block %s near %s", bl->group,
				  st->w[3]);
		}
		return 1;
	}
	// A fixed root is the mechanics': the block stays where it puts it,
	// and what it comes on is a fault.
	if (way >= 0 && !block_clear(pl, b) && !push_block(pl, b, &dirs[way])) {
		pcb_error(&pl->b, pl->b.stmt[pl->stmt_of[r]].line,
			  "no room for block %s", bl->group);
	} else if (way < 0) {
		char why[NAME_LEN];

		if (!block_clear_why(pl, b, why, sizeof why)) {
			pcb_error(
				&pl->b, pl->b.stmt[pl->stmt_of[r]].line,
				"block %s, fixed by its root, is not clear: %s",
				bl->group, why);
		}
	}
	if (pl->all->verbose) {
		(void)fprintf(
			stderr,
			"%s:%d: block %s at %.4f %.4f by its root %s (%s)\n",
			pl->b.file, pl->b.stmt[pl->stmt_of[r]].line, bl->group,
			pl->b.part[r].x - lr->x, pl->b.part[r].y - lr->y,
			pl->b.part[r].name, pcb_ref(&pl->b.part[r]));
	}
	return 1;
}

// Reads the blocks the layout is composed of: each group of a block
// statement within the scope that no other such group holds, and its
// lock; and its root.
// Whether block statement s is held by no larger block (of the group
// laid out, if any).
static int top_block(const struct placer *pl, size_t s)
{
	const struct pcb_stmt *st = &pl->b.stmt[s];

	for (size_t o = 0; o < pl->b.nstmt; o++) {
		const struct pcb_stmt *ot = &pl->b.stmt[o];

		if (o != s && !strcmp(ot->w[0], "block") && ot->n == 2 &&
		    of_group(st->w[1], ot->w[1]) &&
		    (!pl->scope || of_group(ot->w[1], pl->scope))) {
			return 0;
		}
	}
	return 1;
}

// Reads the block of statement st from its section of the lock, laid
// out alone.
static void read_block(struct placer *pl, const struct pcb_stmt *st)
{
	char *text = pcb_section(&pl->b, st->w[1]);
	FILE *f = text ? fmemopen(text, strlen(text), "r") : NULL;
	struct block bl;

	if (!f) {
		pcb_error(&pl->b, st->line,
			  "block %s: no section of it in the lock; lay it "
			  "out first",
			  st->w[1]);
		return;
	}
	memset(&bl, 0, sizeof bl);
	bl.group = pcb_strdup(st->w[1]);
	bl.root = -1;
	pcb_read(&bl.lock, pl->b.file, pl->lib, f, NULL);
	(void)fclose(f);
	pl->blocks = pcb_grow(pl->blocks, pl->nblocks, sizeof *pl->blocks);
	pl->blocks[pl->nblocks++] = bl;
}

static void read_blocks(struct placer *pl)
{
	for (size_t s = 0; s < pl->b.nstmt; s++) {
		const struct pcb_stmt *st = &pl->b.stmt[s];

		if (strcmp(st->w[0], "block") != 0) {
			continue;
		}
		if (st->n != 2) {
			pcb_error(&pl->b, st->line, "block GROUP [as GROUP]");
			continue;
		}
		if ((pl->scope && !of_group(st->w[1], pl->scope)) ||
		    !top_block(pl, s)) {
			continue; // Held by a larger block.
		}
		read_block(pl, st);
	}
	for (size_t k = 0; k < pl->b.npart; k++) {
		pl->block_of[k] = -1;
		for (size_t b = 0; b < pl->nblocks; b++) {
			if (of_group(pl->b.part[k].name, pl->blocks[b].group)) {
				pl->block_of[k] = (long)b;
			}
		}
	}
}

static int by_text(const void *lhs, const void *rhs)
{
	return strcmp(*(char *const *)lhs, *(char *const *)rhs);
}

// The nets on the pads of part k, sorted and joined: alike parts on the
// same nets have the same.
static char *nets_of(struct placer *pl, size_t k)
{
	const struct fp *f = pl->b.part[k].fp;
	char **w = (char **)pcb_alloc((f->npad + 1) * sizeof *w);
	char *s = NULL;

	for (size_t p = 0; p < f->npad; p++) {
		int n = pcb_net_of(&pl->b, k, p);

		w[p] = pcb_strdup(n >= 0 ? pl->b.net[n].name : "-");
	}
	qsort((void *)w, f->npad, sizeof *w, by_text);
	s = pcb_words(w, (int)f->npad);
	for (size_t p = 0; p < f->npad; p++) {
		free(w[p]);
	}
	free((void *)w);
	return s;
}

// The difficulty of placing the group laid out (the board, without -g):
// its parts, each block one and the parts alike on the same nets (the
// same nets on their pads) once.
static size_t difficulty(struct placer *pl)
{
	char **seen = (char **)pcb_alloc((pl->b.npart + 1) * sizeof *seen);
	size_t n = 0;
	size_t nblock = 0;

	for (size_t k = 0; k < pl->b.npart; k++) {
		char *sig = NULL;
		int dup = 0;

		if (!in_scope(pl, k) || !pl->b.part[k].fp ||
		    pl->block_of[k] >= 0) {
			continue;
		}
		sig = nets_of(pl, k);
		for (size_t i = 0; i < n && !dup; i++) {
			dup = !strcmp(seen[i], sig);
		}
		if (dup) {
			free(sig);
		} else {
			seen[n++] = sig;
		}
	}
	for (size_t i = 0; i < n; i++) {
		free(seen[i]);
	}
	free((void *)seen);
	for (size_t b = 0; b < pl->nblocks; b++) {
		nblock += pl->blocks[b].root >= 0;
	}
	return n + nblock;
}

// One pass: the parts of the group (the board, for NULL) placed, from
// its section of the lock read from in; the whole lock written to out.
// Whether a block statement names the group.
static int is_block(struct placer *pl, const char *group)
{
	for (size_t s = 0; s < pl->b.nstmt; s++) {
		if (!strcmp(pl->b.stmt[s].w[0], "block") &&
		    pl->b.stmt[s].n == 2 &&
		    !strcmp(pl->b.stmt[s].w[1], group)) {
			return 1;
		}
	}
	return 0;
}

// The part named, else the first part of the group; or -1.
static long part_or_group(struct placer *pl, const char *th)
{
	long k = part_named(pl, th);

	for (size_t i = 0; k < 0 && i < pl->b.npart; i++) {
		if (of_group(pl->b.part[i].name, th) && pl->b.part[i].fp) {
			k = (long)i;
		}
	}
	return k;
}

// The part a near statement is placed against, for the order and the
// lock: the part, the pin's part, the first part of the group; or -1
// (a cutout, the edge).
static long near_anchor(struct placer *pl, const struct pcb_stmt *st)
{
	char name[WHY_LEN];
	char *dot = NULL;
	long k = part_named(pl, st->w[3]);

	if (k >= 0) {
		return k;
	}
	if (strlen(st->w[3]) >= sizeof name) {
		return part_or_group(pl, st->w[3]);
	}
	(void)snprintf(name, sizeof name, "%s", st->w[3]);
	dot = strrchr(name, '.');
	if (dot) {
		*dot = 0;
		k = part_named(pl, name);
		if (k >= 0) {
			return k;
		}
	}
	return part_or_group(pl, st->w[3]);
}

// Whether the group a near or of statement of part k names is placed
// whole, so k may be placed against it.
static int group_ready(struct placer *pl, size_t k, const int *done)
{
	const struct pcb_stmt *st = &pl->b.stmt[pl->stmt_of[k]];
	const char *th = NULL;

	if (is_near(st)) {
		th = st->w[3];
	} else if (st->n >= PLACE_OF_WORDS && !strcmp(st->w[3], "of")) {
		th = st->w[4];
	}

	for (size_t i = 0; th && i < pl->b.npart; i++) {
		if (i != k && of_group(pl->b.part[i].name, th) &&
		    pl->b.part[i].fp && in_scope(pl, i) && !done[i]) {
			return 0;
		}
	}
	return 1;
}

// The block of the group, or -1.
static long block_named(const struct placer *pl, const char *group)
{
	long b = -1;

	for (size_t i = 0; i < pl->nblocks; i++) {
		if (!strcmp(pl->blocks[i].group, group)) {
			b = (long)i;
		}
	}
	return b;
}

// The parts of the group placed by no statement: their number, and the
// last of them in *root.
static size_t group_root(const struct placer *pl, const char *group, long *root)
{
	size_t n = 0;

	for (size_t k = 0; k < pl->b.npart; k++) {
		if (of_group(pl->b.part[k].name, group) && pl->stmt_of[k] < 0 &&
		    pl->b.part[k].fp) {
			*root = (long)k;
			n++;
		}
	}
	return n;
}

// Carries out each statement place GROUP WAY from PIN that this pass
// lays out: of the group laid out alone, or of a block in it.  The
// group's part that no statement places is its root, placed by the
// statement: alone, at the origin with its pad of the pin's net toward
// the pin; as a block, the block is centered on the pin (place_block).
static void place_groups(struct placer *pl)
{
	for (size_t s = 0; s < pl->b.nstmt; s++) {
		const struct pcb_stmt *st = &pl->b.stmt[s];
		long b = -1;
		long root = -1;
		struct pcb_end pa = { 0, 0 };
		size_t n = 0;

		if (strcmp(st->w[0], "place") != 0 || !form_ok(pl, st) ||
		    !(is_near(st) || !strcmp(st->w[3], "from")) ||
		    part_named(pl, st->w[1]) >= 0 || !is_block(pl, st->w[1])) {
			continue;
		}
		b = block_named(pl, st->w[1]);
		if (b < 0 && (!pl->scope || strcmp(pl->scope, st->w[1]) != 0)) {
			continue; // Not laid out in this pass.
		}
		n = group_root(pl, st->w[1], &root);
		if (is_near(st) && n == 1) {
			pl->stmt_of[root] = (long)s;
			pl->anchor[root] = near_anchor(pl, st);
			continue;
		}
		// A near statement names no pin, and may be of fewer words.
		if (!is_near(st) && !pcb_pin(&pl->b, st->w[4], &pa)) {
			no_pin(pl, st, st->w[4]);
			continue;
		}
		if (n != 1) {
			pcb_error(
				&pl->b, st->line,
				"place GROUP WAY from PIN, or near THING: the "
				"group needs one part placed by no statement");
			continue;
		}
		pl->stmt_of[root] = (long)s;
		pl->anchor[root] = (long)pa.part;
		if (b >= 0) {
			pl->blocks[b].whole = 1;
		}
	}
}

// In a group laid out alone whose root is fixed, the parts fixed
// outside it, where they stand from the root: the group's origin.
static void fixed_outside(struct placer *pl)
{
	const struct pcb_stmt *root = NULL;

	for (size_t k = 0; pl->scope && k < pl->b.npart; k++) {
		if (in_scope(pl, k) && pl->stmt_of[k] >= 0 &&
		    (pl->anchor[k] < 0 ||
		     !in_scope(pl, (size_t)pl->anchor[k]))) {
			root = &pl->b.stmt[pl->stmt_of[k]];
		}
	}
	if (!root || strcmp(root->w[0], "fix") != 0 || root->n < FIX_WORDS) {
		return;
	}
	for (size_t s = 0; s < pl->b.nstmt; s++) {
		const struct pcb_stmt *st = &pl->b.stmt[s];
		long k = -1;
		struct pcb_part o;

		if (!strcmp(st->w[0], "fix") && st->n >= FIX_WORDS) {
			k = part_named(pl, st->w[1]);
		}
		if (k < 0 || in_scope(pl, (size_t)k) || !pl->b.part[k].fp) {
			continue;
		}
		o = pl->b.part[k];
		o.x = strtod(st->w[3], NULL) - strtod(root->w[3], NULL);
		o.y = strtod(st->w[4], NULL) - strtod(root->w[4], NULL);
		o.rot = 0;
		for (int i = FIX_WORDS; i + 1 < st->n; i++) {
			if (!strcmp(st->w[i], "rot")) {
				o.rot = (int)strtol(st->w[i + 1], NULL,
						    DECIMAL);
			}
		}
		o.bottom = !strcmp(st->w[st->n - 1], "bottom");
		o.placed = 1;
		pl->fixed_out = pcb_grow(pl->fixed_out, pl->nfixed_out,
					 sizeof *pl->fixed_out);
		pl->fixed_out[pl->nfixed_out++] = o;
	}
}

// Takes place statement s, st, of part k: what places it, and the part
// it is placed against.
static void take_place(struct placer *pl, size_t s, long k)
{
	const struct pcb_stmt *st = &pl->b.stmt[s];

	if (pl->stmt_of[k] >= 0) {
		pcb_error(&pl->b, st->line, "%s (%s) is placed twice", st->w[1],
			  pcb_ref(&pl->b.part[k]));
		return;
	}
	pl->stmt_of[k] = (long)s;
	if (is_near(st)) {
		pl->anchor[k] = near_anchor(pl, st);
	} else if (st->n >= PLACE_OF_WORDS && !strcmp(st->w[3], "of")) {
		pl->anchor[k] = part_or_group(pl, st->w[4]);
		if (pl->anchor[k] < 0) {
			pcb_error(&pl->b, st->line, "no part or group %s",
				  st->w[4]);
			pl->stmt_of[k] = -1;
		}
	} else if (!strcmp(st->w[0], "place")) {
		struct pcb_end at = { 0, 0 };

		if (pcb_pin(&pl->b, st->w[4], &at)) {
			pl->anchor[k] = (long)at.part;
		} else {
			no_pin(pl, st, st->w[4]);
			pl->stmt_of[k] = -1;
		}
	}
}

// The place and fix statements: what places each part, and what it is
// placed against.
static void read_places(struct placer *pl)
{
	for (size_t k = 0; k < pl->b.npart; k++) {
		pl->stmt_of[k] = pl->anchor[k] = -1;
	}
	for (size_t s = 0; s < pl->b.nstmt; s++) {
		const struct pcb_stmt *st = &pl->b.stmt[s];
		long k = 0;

		if ((strcmp(st->w[0], "fix") != 0 &&
		     strcmp(st->w[0], "place") != 0) ||
		    !form_ok(pl, st)) {
			continue;
		}
		k = st->n >= 2 ? part_named(pl, st->w[1]) : -1;
		if (k < 0 && is_block(pl, st->w[1]) &&
		    !strcmp(st->w[0], "place") &&
		    (is_near(st) ||
		     (st->n >= PLACE_OF_WORDS && !strcmp(st->w[3], "from")))) {
			continue; // A block placed whole: place_groups.
		}
		if (k < 0 || !pl->b.part[k].fp) {
			pcb_error(&pl->b, st->line, "no part %s",
				  st->n >= 2 ? st->w[1] : "");
			continue;
		}
		if (in_scope(pl, (size_t)k)) {
			take_place(pl, s, k);
		}
	}
}

// The root of each block: its part placed by no part of it.
static void block_roots(struct placer *pl)
{
	for (size_t k = 0; k < pl->b.npart; k++) {
		long b = pl->block_of[k];

		if (b >= 0 && pl->stmt_of[k] >= 0 && pl->blocks[b].root < 0 &&
		    (pl->anchor[k] < 0 || pl->block_of[pl->anchor[k]] != b)) {
			pl->blocks[b].root = (long)k;
		}
	}
	for (size_t b = 0; b < pl->nblocks; b++) {
		if (pl->blocks[b].root < 0) {
			pcb_error(&pl->b, 0,
				  "block %s has no part placed from outside it",
				  pl->blocks[b].group);
		}
	}
}

// Which parts keep their place, into kept: the lock's, by the same
// statement and the same nets on its pads, against parts that kept
// theirs.  A block is placed anew each time, from its own lock; the rest
// are placed anew.  Returns how many.
static size_t keep_parts(struct placer *pl, int *kept, int *done)
{
	size_t nfixed = 0;

	for (size_t k = 0; k < pl->b.npart; k++) {
		char *by =
			pl->stmt_of[k] >= 0 ?
				pcb_by(&pl->b, k, &pl->b.stmt[pl->stmt_of[k]]) :
				NULL;

		kept[k] = pl->b.part[k].placed && by && pl->b.part[k].by &&
			  !strcmp(by, pl->b.part[k].by) &&
			  pl->block_of[k] < 0 && in_scope(pl, k);
		free(by);
	}
	for (int changed = 1; changed;) {
		changed = 0;
		for (size_t k = 0; k < pl->b.npart; k++) {
			if (kept[k] && pl->anchor[k] >= 0 &&
			    !kept[pl->anchor[k]] &&
			    in_scope(pl, (size_t)pl->anchor[k])) {
				kept[k] = 0;
				changed = 1;
			}
		}
	}
	for (size_t k = 0; k < pl->b.npart; k++) {
		if (!kept[k]) {
			pl->b.part[k].placed = 0;
		}
		// What is out of the group laid out is never placed here.
		done[k] = kept[k] || !in_scope(pl, k);
		nfixed += kept[k] ? 1 : 0;
	}
	return nfixed;
}

// The part to place next of those not done: what the mechanics fix
// first, then the rest as their anchors allow; a part of a block comes
// with its root.  -1 for none.
static long next_part(struct placer *pl, const int *done)
{
	for (int round = 0; round < 2; round++) {
		for (size_t k = 0; k < pl->b.npart; k++) {
			long b = pl->block_of[k];
			int ready = 0;

			if (done[k] || pl->stmt_of[k] < 0 ||
			    (b >= 0 && pl->blocks[b].root != (long)k)) {
				continue;
			}
			if (round) {
				ready = (pl->anchor[k] < 0 ||
					 done[pl->anchor[k]]) &&
					group_ready(pl, k, done);
			} else {
				ready = !strcmp(pl->b.stmt[pl->stmt_of[k]].w[0],
						"fix");
			}
			if (ready) {
				return (long)k;
			}
		}
	}
	return -1;
}

// Places the parts not done, each in its turn.
static void place_all(struct placer *pl, int *done)
{
	for (;;) {
		long next = next_part(pl, done);

		if (next < 0) {
			break;
		}
		done[next] = 1;
		if (pl->block_of[next] >= 0) {
			place_block(pl, (size_t)pl->block_of[next]);
			for (size_t k = 0; k < pl->b.npart; k++) {
				done[k] = done[k] ||
					  pl->block_of[k] == pl->block_of[next];
			}
		} else {
			place(pl, (size_t)next, NULL, NULL);
		}
	}
}

// Tells each part not placed, and each that stays as it was, kept or
// fixed, and is not clear: it is not moved.
static void tell_unplaced(struct placer *pl, const int *kept)
{
	for (size_t k = 0; k < pl->b.npart; k++) {
		const struct pcb_part *p = &pl->b.part[k];
		char why[WHY_LEN];

		if (p->fp && !p->placed && in_scope(pl, k)) {
			pcb_error(&pl->b, 0, "%s (%s) is not placed", p->name,
				  pcb_ref(p));
		}
		if (p->placed && in_scope(pl, k) && pl->block_of[k] < 0 &&
		    pl->stmt_of[k] >= 0 &&
		    (kept[k] ||
		     !strcmp(pl->b.stmt[pl->stmt_of[k]].w[0], "fix")) &&
		    !clear_why(pl, k, why, sizeof why)) {
			pcb_error(&pl->b, pl->b.stmt[pl->stmt_of[k]].line,
				  "%s (%s), %s, is not clear: %s", p->name,
				  pcb_ref(p), kept[k] ? "kept" : "fixed", why);
		}
	}
}

// Frees what a pass made.
static void free_pass(struct placer *pl)
{
	for (size_t b = 0; b < pl->nblocks; b++) {
		free(pl->blocks[b].group);
		pcb_free(&pl->blocks[b].lock);
	}
	free(pl->blocks);
	free(pl->stmt_of);
	free(pl->anchor);
	free(pl->block_of);
	free(pl->fixed_out);
	pcb_free(&pl->b);
}

static int pass(const struct pcb_run *r)
{
	struct placer pls;
	struct placer *pl = &pls;
	size_t nfixed = 0;
	int *kept = NULL;
	int *done = NULL;
	int failed = 0;

	memset(pl, 0, sizeof *pl);
	pl->all = r->ctx;
	pl->lib = r->lib;
	pl->scope = r->group;
	pcb_read(&pl->b, r->file, r->lib, r->in, pl->scope);
	if (!pl->scope) {
		pcb_tell_copies(&pl->b);
	}
	pl->stmt_of = pcb_alloc((pl->b.npart + 1) * sizeof *pl->stmt_of);
	pl->anchor = pcb_alloc((pl->b.npart + 1) * sizeof *pl->anchor);
	kept = pcb_alloc((pl->b.npart + 1) * sizeof *kept);
	done = pcb_alloc((pl->b.npart + 1) * sizeof *done);
	pl->block_of = pcb_alloc((pl->b.npart + 1) * sizeof *pl->block_of);
	read_blocks(pl);
	read_places(pl);
	place_groups(pl);
	block_roots(pl);
	fixed_outside(pl);
	nfixed = keep_parts(pl, kept, done);
	place_all(pl, done);
	tell_unplaced(pl, kept);
	free(pl->fixed_out);
	pl->fixed_out = NULL;
	pl->nfixed_out = 0;
	if (pl->all->verbose) {
		(void)fprintf(
			stderr,
			"%s: %zu parts kept from the lock; %s: difficulty %zu\n",
			pl->b.file, nfixed, pl->scope ? pl->scope : "the board",
			difficulty(pl));
	}
	pcb_write_lock(&pl->b, r->out);
	failed = pl->b.failed;
	free(kept);
	free(done);
	free_pass(pl);
	return failed;
}

int main(int argc, char **argv)
{
	struct pcb_lib lib = { NULL, 0, NULL, 0 };
	struct across all = { 0, NULL, 0 };
	struct pcb_run r = { NULL, &lib, NULL, NULL, stdout, &all };
	int i = 1;
	const char *usage = "usage: pcbplace [-v] [-g GROUP] [-L LIB]... "
			    "board.pcb [LOCK] > new.lock";
	int failed = 0;

	for (; i < argc && argv[i][0] == '-'; i++) {
		if (!strcmp(argv[i], "-v")) {
			all.verbose = 1;
		} else if (!strcmp(argv[i], "-g") && i + 1 < argc) {
			r.group = argv[++i];
		} else if (!strcmp(argv[i], "-L") && i + 1 < argc) {
			pcb_add_lib(&lib, argv[++i]);
		} else {
			pcb_die("%s", usage);
		}
	}
	if (i != argc - 1 && i != argc - 2) {
		pcb_die("%s", usage);
	}
	r.file = argv[i];
	r.in = pcb_open_lock(i + 1 < argc ? argv[i + 1] : NULL);
	failed = pcb_passes(&r, pass);
	if (r.in && r.in != stdin) {
		(void)fclose(r.in);
	}
	pcb_free_lib(&lib);
	free(all.told);
	return failed;
}
