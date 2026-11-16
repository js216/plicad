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
// and nothing else.  The copper of the lock is passed on as it is;
// pcbroute tells what has gone stale.
//
// The statements, in mm and degrees:
//   fix PART at X Y [rot R] [bottom]     where the mechanics put it
//   place PART WAY of PART [GAP] [rot R] next to it, its courtyard GAP
//                                        off (0), centered on it
//   place PART WAY from PIN [rot R]     its pad of the pin's net next
//                                        to the pin, lying out WAY
//   place GROUP WAY from PIN            a block whole, as one part of two
//                                        sides: its pads of the pin's net
//                                        centered on the pin, toward it
// The fixed parts are placed first, the rest in the order their
// anchors allow.  A part
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
// makes the group one rigid body in any layout round it: its parts stand
// as its section has them, the whole moved, unturned, to where its root's
// statement puts its root, then pushed out its way until clear.

#define _POSIX_C_SOURCE 200809L // For fmemopen.

#include "pcbsrc.h"
#include <math.h>
#include <string.h>

#define STEP 0.05 // How far a part moves at a time to find room.
#define GAP 0 // Between courtyards, by default: they may touch.

static struct pcb B;
static int verbose;
static const char *scope; // The group laid out alone, or NULL.

// A block composed here: its group, its lock (laid out alone), its root.
struct block {
	char *group;
	struct pcb lock;
	long root;
	int whole; // Placed whole by place GROUP WAY from PIN.
};

static struct block *blocks;
static size_t nblocks;
static long *block_of; // The block of each part, or -1.

// Whether a name is of a group: GROUP.REST.
static int of_group(const char *name, const char *group)
{
	size_t n = strlen(group);

	return !strncmp(name, group, n) && name[n] == '.';
}

static int in_scope(size_t k)
{
	return !scope || of_group(B.part[k].name, scope);
}
static const int DX[4] = { 1, 0, -1, 0 }, DY[4] = { 0, 1, 0, -1 };

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

static long part_named(const char *name)
{
	for (size_t i = 0; i < B.npart; i++) {
		if (!strcmp(B.part[i].name, name)) {
			return (long)i;
		}
	}
	return -1;
}

// The statement that places each part, and the part it is placed
// against (-1 for none).
static long *stmt_of, *anchor;

static int overlap(const double a[4], const double b[4])
{
	return a[0] < b[2] - 1e-9 && b[0] < a[2] - 1e-9 && a[1] < b[3] - 1e-9 &&
	       b[1] < a[3] - 1e-9;
}

// The parts the mechanics fix outside a group laid out alone whose root
// they fix too, where they stand from the group's origin: a group laid
// out alone keeps clear of them as the board will.
static struct pcb_part *fixed_out;
static size_t nfixed_out;

// Whether courtyard c of part k comes on the copper of a plated hole
// of part o, which is copper on both sides.
static int on_hole(const double c[4], const struct pcb_part *o)
{
	for (size_t p = 0; o->placed && o->fp && p < o->fp->npad; p++) {
		struct pcb_spot s;
		double h[4];

		if (!o->fp->pad[p].hole) {
			continue;
		}
		pcb_spot(&B, o, p, &s);
		h[0] = s.x - s.w / 2 - B.clearance;
		h[1] = s.y - s.h / 2 - B.clearance;
		h[2] = s.x + s.w / 2 + B.clearance;
		h[3] = s.y + s.h / 2 + B.clearance;
		if (overlap(c, h)) {
			return 1;
		}
	}
	return 0;
}

// Whether part k stands clear: its courtyard off every other placed
// part's on its side, off the keep-out of every hole and off every
// plated hole, and inside the board; the fault, if not, goes to why.
static int clear_on(size_t k, int board, char *why, size_t n)
{
	double c[4];

	pcb_court(&B.part[k], c);
	if (board && !scope && (c[0] < B.board[0] || c[1] < B.board[1] ||
		       c[2] > B.board[2] || c[3] > B.board[3])) {
		snprintf(why, n, "off the board");
		return 0;
	}
	for (size_t i = 0; i < B.npart; i++) {
		const struct pcb_part *o = &B.part[i];

		if (i == k || !o->placed || !o->fp ||
		    (block_of[k] >= 0 && block_of[i] == block_of[k])) {
			continue;
		}
		if (pcb_clash(&B, &B.part[k], o, why, n)) {
			return 0;
		}
		if (on_hole(c, o)) {
			snprintf(why, n, "on a plated hole of %s (%s)", o->name,
				 pcb_ref(o));
			return 0;
		}
	}
	for (size_t i = 0; i < nfixed_out; i++) {
		if (pcb_clash(&B, &B.part[k], &fixed_out[i], why, n)) {
			return 0;
		}
		if (on_hole(c, &fixed_out[i])) {
			snprintf(why, n, "on a plated hole of %s (%s)",
				 fixed_out[i].name, pcb_ref(&fixed_out[i]));
			return 0;
		}
	}
	return 1;
}

static int clear(size_t k)
{
	char why[1024];

	return clear_on(k, 1, why, sizeof why);
}

// Whether part k, which is not moved, comes on no other part or
// keep-out: the board's edge is pcbcheck's, by its rule.
static int clear_why(size_t k, char *why, size_t n)
{
	return clear_on(k, 0, why, n);
}

// The length of the ratsnest of part k as it stands: from each of its
// pads to the nearest placed pad of the same net on another part.
static double ratsnest(size_t k)
{
	double sum = 0;

	for (size_t p = 0; p < B.part[k].fp->npad; p++) {
		int n = pcb_net_of(&B, k, p);
		double best = -1;
		struct pcb_spot s;

		if (n < 0 || B.net[n].plane >= 0) {
			continue;
		}
		pcb_spot(&B, &B.part[k], p, &s);
		for (size_t j = 0; j < B.net[n].nend; j++) {
			const struct pcb_end *e = &B.net[n].end[j];
			struct pcb_spot o;
			double d;

			if (e->part == k || !B.part[e->part].placed) {
				continue;
			}
			pcb_spot(&B, &B.part[e->part], e->pad, &o);
			d = hypot(o.x - s.x, o.y - s.y);
			best = best < 0 || d < best ? d : best;
		}
		sum += best > 0 ? best : 0;
	}
	return sum;
}

// Moves part k out the way until it stands clear; returns 0 if it
// leaves the board first.
static int push(size_t k, int way)
{
	struct pcb_part *q = &B.part[k];
	double c[4];

	for (int steps = 0; !clear(k); steps++) {
		q->x += DX[way] * STEP;
		q->y += DY[way] * STEP;
		pcb_court(q, c);
		if (scope ? steps > 4000 :
			    c[2] < B.board[0] || c[0] > B.board[2] ||
				    c[3] < B.board[1] || c[1] > B.board[3]) {
			return 0;
		}
	}
	return 1;
}

// Whether every part of block b stands clear of all outside it (the
// board's edge too, if board is set); the fault, if not, goes to why.
// Whether a pad of block b comes within the clearance of a via of
// another block placed, as that block's lock has the via, moved with it;
// the fault, if so, goes to why.
static int on_via(size_t b, char *why, size_t n)
{
	for (size_t o = 0; o < nblocks; o++) {
		const struct block *ob = &blocks[o];
		long r = ob->root;
		double dx, dy;

		if (o == b || r < 0 || !B.part[r].placed ||
		    !ob->lock.part[r].placed) {
			continue;
		}
		dx = B.part[r].x - ob->lock.part[r].x;
		dy = B.part[r].y - ob->lock.part[r].y;
		for (size_t v = 0; v < ob->lock.nvia; v++) {
			const struct pcb_via *w = &ob->lock.via[v];

			for (size_t i = 0; i < B.npart; i++) {
				for (size_t p = 0;
				     block_of[i] == (long)b && p < B.part[i].fp->npad;
				     p++) {
					struct pcb_spot s;

					pcb_spot(&B, &B.part[i], p, &s);
					if (pcb_net_of(&B, i, p) != w->net &&
					    pcb_spot_dist(&s, w->x + dx, w->y + dy) <
						    w->dia / 2 + B.clearance) {
						snprintf(why, n,
							 "%s (%s): on a via of block %s",
							 B.part[i].name,
							 pcb_ref(&B.part[i]),
							 ob->group);
						return 1;
					}
				}
			}
		}
	}
	return 0;
}

static int block_clear_on(size_t b, int board, char *why, size_t n)
{
	if (on_via(b, why, n)) {
		return 0;
	}
	for (size_t i = 0; i < B.npart; i++) {
		if (block_of[i] == (long)b && !clear_on(i, board, why, n)) {
			char *own = pcb_alloc(strlen(B.part[i].name) +
					      strlen(pcb_ref(&B.part[i])) +
					      strlen(why) + 8);

			sprintf(own, "%s (%s): %s", B.part[i].name,
				pcb_ref(&B.part[i]), why);
			snprintf(why, n, "%s", own);
			free(own);
			return 0;
		}
	}
	return 1;
}

static int block_clear(size_t b)
{
	char why[1200];

	return block_clear_on(b, 1, why, sizeof why);
}

static int block_clear_why(size_t b, char *why, size_t n)
{
	return block_clear_on(b, 0, why, n);
}

// Moves block b out the way, rigidly, until it stands clear; returns 0
// if it leaves the board (200 mm out of a group).
static int push_block(size_t b, int way)
{
	for (int steps = 0; !block_clear(b); steps++) {
		if (way < 0 || steps > 4000) {
			return 0;
		}
		for (size_t i = 0; i < B.npart; i++) {
			if (block_of[i] == (long)b) {
				B.part[i].x += DX[way] * STEP;
				B.part[i].y += DY[way] * STEP;
			}
		}
	}
	return 1;
}

// Places part k against part a, the way of it, gap off, centered.
static void place_of(size_t k, size_t a, int way, double gap, int rot)
{
	struct pcb_part *q = &B.part[k];
	double c[4], d[4];

	q->rot = rot;
	q->bottom = B.part[a].bottom;
	q->x = q->y = 0;
	pcb_court(q, c);
	pcb_court(&B.part[a], d);
	q->x = way == 0 ? d[2] + gap - c[0] :
	       way == 2 ? d[0] - gap - c[2] :
			  (d[0] + d[2]) / 2;
	q->y = way == 1 ? d[3] + gap - c[1] :
	       way == 3 ? d[1] - gap - c[3] :
			  (d[1] + d[3]) / 2;
}

// Places part k with its pad of the net of pin (a, ap) next to the pin,
// lying out the way.
static void place_at(size_t k, size_t a, size_t ap, int way, int rot)
{
	struct pcb_part *q = &B.part[k];
	struct pcb_spot pin, own;
	int n = pcb_net_of(&B, a, ap);
	size_t mine = 0;
	double c[4], d[4];

	for (size_t p = 0; p < q->fp->npad; p++) {
		if (n >= 0 && pcb_net_of(&B, k, p) == n) {
			mine = p;
			break;
		}
	}
	q->rot = rot;
	q->bottom = B.part[a].bottom;
	q->x = q->y = 0;
	pcb_spot(&B, q, mine, &own);
	pcb_spot(&B, &B.part[a], ap, &pin);
	pcb_court(q, c);
	pcb_court(&B.part[a], d);
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

// The root of the group laid out alone, at its origin: fixed, as said;
// placed from a pin outside, turned so its pad of the pin's net lies the
// way back, toward the pin (the port facing its neighbor); else as said,
// or unturned.
static void place_root(size_t k, const struct pcb_stmt *st, int rot, int n)
{
	struct pcb_part *q = &B.part[k];
	size_t a, ap;

	q->x = q->y = 0;
	q->rot = rot < 0 ? 0 : rot;
	q->bottom = !strcmp(st->w[0], "fix") &&
		    !strcmp(st->w[st->n - 1], "bottom");
	if (rot < 0 && n >= 5 && !strcmp(st->w[3], "from") &&
	    pcb_pin(&B, st->w[4], &a, &ap) && way_word(st->w[2]) >= 0) {
		int way = way_word(st->w[2]), net = pcb_net_of(&B, a, ap);
		double best = -1e300;

		for (int r = 0; r < 360; r += 90) {
			q->rot = r;
			for (size_t p = 0; p < q->fp->npad; p++) {
				struct pcb_spot own;
				double back;

				if (pcb_net_of(&B, k, p) != net) {
					continue;
				}
				pcb_spot(&B, q, p, &own);
				back = -(own.x * DX[way] + own.y * DY[way]);
				if (back > best + 1e-9) {
					best = back;
					rot = r;
				}
				break;
			}
		}
		q->rot = rot < 0 ? 0 : rot;
	}
}

// Carries out the statement of part k, turned as block_rot says if it
// is 0 or more (the root of a block, whose turn its lock has); a block's
// root is not pushed here (the block is, whole), and its way goes to
// *way_out.  Returns 0 if it cannot be.
static int place(size_t k, int block_rot, int *way_out)
{
	const struct pcb_stmt *st = &B.stmt[stmt_of[k]];
	struct pcb_part *q = &B.part[k];
	int rot = -1, way = -1, n = st->n;
	size_t a = 0, ap = 0;
	const char *how = "";

	for (int i = 0; i + 1 < st->n; i++) {
		if (!strcmp(st->w[i], "rot")) {
			rot = atoi(st->w[i + 1]);
			n = i;
		}
	}
	rot = block_rot >= 0 ? block_rot : rot;
	if (scope && (anchor[k] < 0 || !in_scope((size_t)anchor[k]))) {
		place_root(k, st, rot, n);
		how = "the root of the group, at its origin";
	} else if (!strcmp(st->w[0], "fix")) {
		if (n < 5 || strcmp(st->w[2], "at")) {
			pcb_error(&B, st->line,
				  "fix PART at X Y [rot R] [bottom]");
			return 0;
		}
		q->x = atof(st->w[3]);
		q->y = atof(st->w[4]);
		q->rot = rot < 0 ? 0 : rot;
		q->bottom = !strcmp(st->w[st->n - 1], "bottom");
		how = "fixed";
	} else if (n >= 5 && !strcmp(st->w[3], "of")) {
		double gap = GAP;

		way = way_word(st->w[2]);
		a = (size_t)anchor[k];
		if (way < 0) {
			pcb_error(&B, st->line, "place PART WAY of PART [GAP]");
			return 0;
		}
		if (n == 6) {
			gap = atof(st->w[5]);
		}
		if (rot >= 0) {
			place_of(k, a, way, gap, rot);
			how = "said";
		} else {
			double best = 1e300;
			int br = 0;

			for (int r = 0; r < 360; r += 90) {
				double len;

				place_of(k, a, way, gap, r);
				len = push(k, way) ? ratsnest(k) : 1e300;
				if (len < best - 1e-9) {
					best = len;
					br = r;
				}
			}
			place_of(k, a, way, gap, br);
			how = "shortest ratsnest";
		}
	} else if (n >= 5 && !strcmp(st->w[3], "from")) {
		if (!pcb_pin(&B, st->w[4], &a, &ap)) {
			pcb_error(&B, st->line, "no pin %s", st->w[4]);
			return 0;
		}
		way = way_word(st->w[2]);
		if (way < 0) {
			pcb_error(&B, st->line, "place PART WAY from PIN");
			return 0;
		}
		if (rot < 0) {
			// Lying along the way, the pad of the net toward the pin.
			double best = 1e300;
			int br = 0;

			for (int r = 0; r < 360; r += 90) {
				struct pcb_spot pin, own;
				int net = pcb_net_of(&B, a, ap);
				double len = 0;

				place_at(k, a, ap, way, r);
				pcb_spot(&B, &B.part[a], ap, &pin);
				for (size_t p = 0; p < q->fp->npad; p++) {
					if (pcb_net_of(&B, k, p) == net) {
						pcb_spot(&B, q, p, &own);
						len = hypot(own.x - pin.x,
							    own.y - pin.y);
						break;
					}
				}
				len += ratsnest(k) * 1e-3;
				if (len < best - 1e-9) {
					best = len;
					br = r;
				}
			}
			rot = br;
			how = "its pad nearest the pin";
		} else {
			how = "said";
		}
		place_at(k, a, ap, way, rot);
	} else {
		pcb_error(&B, st->line,
			  "place PART WAY of PART, or WAY from PIN");
		return 0;
	}
	q->placed = 1;
	if (way_out) {
		*way_out = way;
		return 1;
	}
	if (way >= 0 && !push(k, way)) {
		static const char *const ways[] = { "right", "up", "left",
						    "down" };

		q->placed = 0;
		pcb_error(&B, st->line, "no room for %s (%s) %s of its anchor",
			  q->name, pcb_ref(q), ways[way]);
		return 0;
	}
	free(q->by);
	q->by = pcb_by(&B, k, st);
	if (verbose) {
		fprintf(stderr, "%s:%d: %s (%s) at %.3f %.3f rot %d: %s\n",
			B.file, st->line, q->name, pcb_ref(q), q->x, q->y, q->rot,
			how);
	}
	return 1;
}

// Moves block b, placed whole from a pin by statement st the way way,
// to stand as one part of two sides would: its pads of the pin's net
// centered on the pin across the way, its courtyards starting at the
// anchor's along it.
static void center_block(size_t b, const struct pcb_stmt *st, int way)
{
	size_t a, ap;
	struct pcb_spot pin;
	double c[4] = { 1e300, 1e300, -1e300, -1e300 }, d[4];
	double sx = 0, sy = 0, dx, dy;
	int net, n = 0;

	if (!pcb_pin(&B, st->w[4], &a, &ap) || way < 0) {
		return;
	}
	net = pcb_net_of(&B, a, ap);
	pcb_spot(&B, &B.part[a], ap, &pin);
	pcb_court(&B.part[a], d);
	for (size_t i = 0; i < B.npart; i++) {
		double e[4];

		if (block_of[i] != (long)b) {
			continue;
		}
		pcb_court(&B.part[i], e);
		c[0] = fmin(c[0], e[0]);
		c[1] = fmin(c[1], e[1]);
		c[2] = fmax(c[2], e[2]);
		c[3] = fmax(c[3], e[3]);
		for (size_t p = 0; p < B.part[i].fp->npad; p++) {
			struct pcb_spot o;

			if (net >= 0 && pcb_net_of(&B, i, p) == net) {
				pcb_spot(&B, &B.part[i], p, &o);
				sx += o.x;
				sy += o.y;
				n++;
			}
		}
	}
	if (!n) {
		return;
	}
	if (way == 1 || way == 3) {
		dx = pin.x - sx / n;
		dy = way == 1 ? d[3] + GAP - c[1] : d[1] - GAP - c[3];
	} else {
		dy = pin.y - sy / n;
		dx = way == 0 ? d[2] + GAP - c[0] : d[0] - GAP - c[2];
	}
	// By whole steps of the lock's 0.1 um, as the block moves.
	dx = round(dx * 1e4) / 1e4;
	dy = round(dy * 1e4) / 1e4;
	for (size_t i = 0; i < B.npart; i++) {
		if (block_of[i] == (long)b) {
			B.part[i].x += dx;
			B.part[i].y += dy;
		}
	}
}

// Places block b: its root by its statement, turned as its lock has it;
// its parts as its lock has them, moved with the root, unturned; then
// pushed out the root's way until it stands clear.
static int place_block(size_t b)
{
	const struct block *bl = &blocks[b];
	size_t r = (size_t)bl->root;
	const struct pcb_part *lr = &bl->lock.part[r];
	double tx, ty;
	int way = -1;

	if (!lr->placed) {
		pcb_error(&B, B.stmt[stmt_of[r]].line,
			  "block %s: its lock does not place %s (%s)", bl->group,
			  B.part[r].name, pcb_ref(&B.part[r]));
		return 0;
	}
	if (!place(r, lr->rot, &way)) {
		return 0;
	}
	// A move by whole steps of the lock's 0.1 µm, so the parts stand
	// to the digit as the block's lock has them, relative to each other.
	tx = round((B.part[r].x - lr->x) * 1e4) / 1e4;
	ty = round((B.part[r].y - lr->y) * 1e4) / 1e4;
	for (size_t i = 0; i < B.npart; i++) {
		const struct pcb_part *l = &bl->lock.part[i];

		if (block_of[i] != (long)b) {
			continue;
		}
		if (!l->placed) {
			pcb_error(&B, 0, "block %s: its lock does not place %s",
				  bl->group, B.part[i].name);
			continue;
		}
		B.part[i].x = l->x + tx;
		B.part[i].y = l->y + ty;
		B.part[i].rot = l->rot;
		B.part[i].bottom = l->bottom;
		B.part[i].placed = 1;
		free(B.part[i].by);
		B.part[i].by = pcb_strdup(l->by ? l->by : "by");
	}
	if (bl->whole) {
		center_block(b, &B.stmt[stmt_of[r]], way);
	}
	// A fixed root is the mechanics': the block stays where it puts it,
	// and what it comes on is a fault.
	if (way >= 0 && !block_clear(b) && !push_block(b, way)) {
		pcb_error(&B, B.stmt[stmt_of[r]].line, "no room for block %s",
			  bl->group);
	} else if (way < 0) {
		char why[1200];

		if (!block_clear_why(b, why, sizeof why)) {
			pcb_error(&B, B.stmt[stmt_of[r]].line,
				  "block %s, fixed by its root, is not clear: %s",
				  bl->group, why);
		}
	}
	if (verbose) {
		fprintf(stderr, "%s:%d: block %s at %.4f %.4f by its root %s (%s)\n",
			B.file, B.stmt[stmt_of[r]].line, bl->group,
			B.part[r].x - lr->x, B.part[r].y - lr->y, B.part[r].name,
			pcb_ref(&B.part[r]));
	}
	return 1;
}

// Reads the blocks the layout is composed of: each group of a block
// statement within the scope that no other such group holds, and its
// lock; and its root.
static void read_blocks(char **lib, size_t nlib)
{
	for (size_t s = 0; s < B.nstmt; s++) {
		const struct pcb_stmt *st = &B.stmt[s];
		int top = 1;
		char *text;
		FILE *f;
		struct block bl;

		if (strcmp(st->w[0], "block")) {
			continue;
		}
		if (st->n != 2) {
			pcb_error(&B, st->line, "block GROUP");
			continue;
		}
		if (scope && !of_group(st->w[1], scope)) {
			continue;
		}
		for (size_t o = 0; o < B.nstmt; o++) {
			const struct pcb_stmt *ot = &B.stmt[o];

			if (o != s && !strcmp(ot->w[0], "block") && ot->n == 2 &&
			    of_group(st->w[1], ot->w[1]) &&
			    (!scope || of_group(ot->w[1], scope))) {
				top = 0; // Held by a larger block.
			}
		}
		if (!top) {
			continue;
		}
		text = pcb_section(&B, st->w[1]);
		f = text ? fmemopen(text, strlen(text), "r") : NULL;
		if (!f) {
			pcb_error(&B, st->line,
				  "block %s: no section of it in the lock; lay it "
				  "out first",
				  st->w[1]);
			continue;
		}
		memset(&bl, 0, sizeof bl);
		bl.group = pcb_strdup(st->w[1]);
		bl.root = -1;
		pcb_read(&bl.lock, B.file, lib, nlib, f, NULL);
		fclose(f);
		blocks = pcb_grow(blocks, nblocks, sizeof *blocks);
		blocks[nblocks++] = bl;
	}
	for (size_t k = 0; k < B.npart; k++) {
		block_of[k] = -1;
		for (size_t b = 0; b < nblocks; b++) {
			if (of_group(B.part[k].name, blocks[b].group)) {
				block_of[k] = (long)b;
			}
		}
	}
}

static int by_text(const void *a, const void *b)
{
	return strcmp(*(char *const *)a, *(char *const *)b);
}

// The nets on the pads of part k, sorted and joined: alike parts on the
// same nets have the same.
static char *nets_of(size_t k)
{
	const struct fp *f = B.part[k].fp;
	char **w = pcb_alloc((f->npad + 1) * sizeof *w), *s;

	for (size_t p = 0; p < f->npad; p++) {
		int n = pcb_net_of(&B, k, p);

		w[p] = pcb_strdup(n >= 0 ? B.net[n].name : "-");
	}
	qsort(w, f->npad, sizeof *w, by_text);
	s = pcb_words(w, (int)f->npad);
	for (size_t p = 0; p < f->npad; p++) {
		free(w[p]);
	}
	free(w);
	return s;
}

// The difficulty of placing the group laid out (the board, without -g):
// its parts, each block one and the parts alike on the same nets (the
// same nets on their pads) once.
static size_t difficulty(void)
{
	char **seen = pcb_alloc((B.npart + 1) * sizeof *seen);
	size_t n = 0, nblock = 0;

	for (size_t k = 0; k < B.npart; k++) {
		char *sig;
		int dup = 0;

		if (!in_scope(k) || !B.part[k].fp || block_of[k] >= 0) {
			continue;
		}
		sig = nets_of(k);
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
	free(seen);
	for (size_t b = 0; b < nblocks; b++) {
		nblock += blocks[b].root >= 0;
	}
	return n + nblock;
}

// One pass: the parts of the group (the board, for NULL) placed, from
// its section of the lock read from in; the whole lock written to out.
// Whether a block statement names the group.
static int is_block(const char *group)
{
	for (size_t s = 0; s < B.nstmt; s++) {
		if (!strcmp(B.stmt[s].w[0], "block") && B.stmt[s].n == 2 &&
		    !strcmp(B.stmt[s].w[1], group)) {
			return 1;
		}
	}
	return 0;
}

// Carries out each statement place GROUP WAY from PIN that this pass
// lays out: of the group laid out alone, or of a block in it.  The
// group's part that no statement places is its root, placed by the
// statement: alone, at the origin with its pad of the pin's net toward
// the pin; as a block, the block is centered on the pin (place_block).
static void place_groups(void)
{
	for (size_t s = 0; s < B.nstmt; s++) {
		const struct pcb_stmt *st = &B.stmt[s];
		long b = -1, root = -1;
		size_t a, ap, n = 0;

		if (strcmp(st->w[0], "place") || st->n < 5 ||
		    strcmp(st->w[3], "from") || part_named(st->w[1]) >= 0 ||
		    !is_block(st->w[1])) {
			continue;
		}
		for (size_t i = 0; i < nblocks; i++) {
			b = strcmp(blocks[i].group, st->w[1]) ? b : (long)i;
		}
		if (b < 0 && (!scope || strcmp(scope, st->w[1]))) {
			continue; // Not laid out in this pass.
		}
		for (size_t k = 0; k < B.npart; k++) {
			if (of_group(B.part[k].name, st->w[1]) &&
			    stmt_of[k] < 0 && B.part[k].fp) {
				root = (long)k;
				n++;
			}
		}
		if (n != 1 || !pcb_pin(&B, st->w[4], &a, &ap)) {
			pcb_error(&B, st->line,
				  "place GROUP WAY from PIN: the group needs one "
				  "part placed by no statement, and the pin");
			continue;
		}
		stmt_of[root] = (long)s;
		anchor[root] = (long)a;
		if (b >= 0) {
			blocks[b].whole = 1;
		}
	}
}

// In a group laid out alone whose root is fixed, the parts fixed
// outside it, where they stand from the root: the group's origin.
static void fixed_outside(void)
{
	const struct pcb_stmt *root = NULL;

	for (size_t k = 0; scope && k < B.npart; k++) {
		if (in_scope(k) && stmt_of[k] >= 0 &&
		    (anchor[k] < 0 || !in_scope((size_t)anchor[k]))) {
			root = &B.stmt[stmt_of[k]];
		}
	}
	if (!root || strcmp(root->w[0], "fix") || root->n < 5) {
		return;
	}
	for (size_t s = 0; s < B.nstmt; s++) {
		const struct pcb_stmt *st = &B.stmt[s];
		long k;
		struct pcb_part o;

		if (strcmp(st->w[0], "fix") || st->n < 5 ||
		    (k = part_named(st->w[1])) < 0 || in_scope((size_t)k) ||
		    !B.part[k].fp) {
			continue;
		}
		o = B.part[k];
		o.x = atof(st->w[3]) - atof(root->w[3]);
		o.y = atof(st->w[4]) - atof(root->w[4]);
		o.rot = 0;
		for (int i = 5; i + 1 < st->n; i++) {
			if (!strcmp(st->w[i], "rot")) {
				o.rot = atoi(st->w[i + 1]);
			}
		}
		o.bottom = !strcmp(st->w[st->n - 1], "bottom");
		o.placed = 1;
		fixed_out = pcb_grow(fixed_out, nfixed_out, sizeof *fixed_out);
		fixed_out[nfixed_out++] = o;
	}
}

static int pass(const char *file, char **lib, size_t nlib, const char *group,
		FILE *in, FILE *out)
{
	size_t nfixed = 0;
	int *kept, *done, failed;

	scope = group;
	blocks = NULL;
	nblocks = 0;
	pcb_read(&B, file, lib, nlib, in, scope);
	stmt_of = pcb_alloc((B.npart + 1) * sizeof *stmt_of);
	anchor = pcb_alloc((B.npart + 1) * sizeof *anchor);
	kept = pcb_alloc((B.npart + 1) * sizeof *kept);
	done = pcb_alloc((B.npart + 1) * sizeof *done);
	block_of = pcb_alloc((B.npart + 1) * sizeof *block_of);
	read_blocks(lib, nlib);
	for (size_t k = 0; k < B.npart; k++) {
		stmt_of[k] = anchor[k] = -1;
	}
	for (size_t s = 0; s < B.nstmt; s++) {
		const struct pcb_stmt *st = &B.stmt[s];
		long k;

		if (strcmp(st->w[0], "fix") && strcmp(st->w[0], "place")) {
			continue;
		}
		k = st->n >= 2 ? part_named(st->w[1]) : -1;
		if (k < 0 && is_block(st->w[1]) && st->n >= 5 &&
		    !strcmp(st->w[0], "place") && !strcmp(st->w[3], "from")) {
			continue; // A block placed whole: place_groups.
		}
		if (k < 0 || !B.part[k].fp) {
			pcb_error(&B, st->line, "no part %s",
				  st->n >= 2 ? st->w[1] : "");
			continue;
		}
		if (!in_scope((size_t)k)) {
			continue;
		}
		if (stmt_of[k] >= 0) {
			pcb_error(&B, st->line, "%s (%s) is placed twice", st->w[1],
				  pcb_ref(&B.part[k]));
			continue;
		}
		stmt_of[k] = (long)s;
		if (st->n >= 5 && !strcmp(st->w[3], "of")) {
			anchor[k] = part_named(st->w[4]);
			if (anchor[k] < 0) {
				pcb_error(&B, st->line, "no part %s", st->w[4]);
				stmt_of[k] = -1;
			}
		} else if (st->n >= 5 && !strcmp(st->w[0], "place") &&
			   !strcmp(st->w[3], "from")) {
			char name[1024], *dot;

			snprintf(name, sizeof name, "%s", st->w[4]);
			dot = strrchr(name, '.');
			if (dot) {
				*dot = 0;
			}
			anchor[k] = part_named(name);
			if (anchor[k] < 0) {
				pcb_error(&B, st->line, "no part %s", name);
				stmt_of[k] = -1;
			}
		}
	}
	place_groups();
	// The root of each block: its part placed by no part of it.
	for (size_t k = 0; k < B.npart; k++) {
		long b = block_of[k];

		if (b >= 0 && stmt_of[k] >= 0 && blocks[b].root < 0 &&
		    (anchor[k] < 0 || block_of[anchor[k]] != b)) {
			blocks[b].root = (long)k;
		}
	}
	for (size_t b = 0; b < nblocks; b++) {
		if (blocks[b].root < 0) {
			pcb_error(&B, 0, "block %s has no part placed from outside it",
				  blocks[b].group);
		}
	}
	fixed_outside();
	// Which parts keep their place: the lock's, by the same statement
	// and the same nets on its pads, against parts that kept theirs.  A
	// block is placed anew each time, from its own lock; the rest are
	// placed anew.
	for (size_t k = 0; k < B.npart; k++) {
		char *by = stmt_of[k] >= 0 ?
				   pcb_by(&B, k, &B.stmt[stmt_of[k]]) :
				   NULL;

		kept[k] = B.part[k].placed && by && B.part[k].by &&
			  !strcmp(by, B.part[k].by) && block_of[k] < 0 &&
			  in_scope(k);
		free(by);
	}
	for (int changed = 1; changed;) {
		changed = 0;
		for (size_t k = 0; k < B.npart; k++) {
			if (kept[k] && anchor[k] >= 0 && !kept[anchor[k]] &&
			    in_scope((size_t)anchor[k])) {
				kept[k] = 0;
				changed = 1;
			}
		}
	}
	for (size_t k = 0; k < B.npart; k++) {
		if (!kept[k]) {
			B.part[k].placed = 0;
		}
		// What is out of the group laid out is never placed here.
		done[k] = kept[k] || !in_scope(k);
		nfixed += kept[k] ? 1 : 0;
	}
	for (;;) {
		long next = -1;

		// What the mechanics fix comes first, then the rest as their
		// anchors allow; a part of a block comes with its root.
		for (int pass = 0; pass < 2 && next < 0; pass++) {
			for (size_t k = 0; k < B.npart && next < 0; k++) {
				long b = block_of[k];

				if (done[k] || stmt_of[k] < 0 ||
				    (b >= 0 && blocks[b].root != (long)k)) {
					continue;
				}
				if (pass ? anchor[k] < 0 || done[anchor[k]] :
					   !strcmp(B.stmt[stmt_of[k]].w[0],
						   "fix")) {
					next = (long)k;
				}
			}
		}
		if (next < 0) {
			break;
		}
		done[next] = 1;
		if (block_of[next] >= 0) {
			place_block((size_t)block_of[next]);
			for (size_t k = 0; k < B.npart; k++) {
				done[k] |= block_of[k] == block_of[next];
			}
		} else {
			place((size_t)next, -1, NULL);
		}
	}
	for (size_t k = 0; k < B.npart; k++) {
		char why[1200];

		if (B.part[k].fp && !B.part[k].placed && in_scope(k)) {
			pcb_error(&B, 0, "%s (%s) is not placed", B.part[k].name,
				  pcb_ref(&B.part[k]));
		}
		// What stays as it was, kept or fixed, is not moved: what it
		// comes on is a fault.
		if (B.part[k].placed && in_scope(k) && block_of[k] < 0 &&
		    stmt_of[k] >= 0 &&
		    (kept[k] || !strcmp(B.stmt[stmt_of[k]].w[0], "fix")) &&
		    !clear_why(k, why, sizeof why)) {
			pcb_error(&B, B.stmt[stmt_of[k]].line,
				  "%s (%s), %s, is not clear: %s", B.part[k].name,
				  pcb_ref(&B.part[k]), kept[k] ? "kept" : "fixed",
				  why);
		}
	}
	free(fixed_out);
	fixed_out = NULL;
	nfixed_out = 0;
	if (verbose) {
		fprintf(stderr,
			"%s: %zu parts kept from the lock; %s: difficulty %zu\n",
			B.file, nfixed, scope ? scope : "the board", difficulty());
	}
	pcb_write_lock(&B, out);
	failed = B.failed;
	for (size_t b = 0; b < nblocks; b++) {
		free(blocks[b].group);
		pcb_free(&blocks[b].lock);
	}
	free(blocks);
	free(stmt_of);
	free(anchor);
	free(kept);
	free(done);
	free(block_of);
	pcb_free(&B);
	return failed;
}

int main(int argc, char **argv)
{
	char **lib = NULL;
	size_t nlib = 0;
	int i = 1;
	const char *group = NULL;
	const char *usage = "usage: pcbplace [-v] [-g GROUP] [-L LIB]... "
			    "board.pcb [LOCK] > new.lock";
	FILE *in;
	int failed;

	for (; i < argc && argv[i][0] == '-'; i++) {
		if (!strcmp(argv[i], "-v")) {
			verbose = 1;
		} else if (!strcmp(argv[i], "-g") && i + 1 < argc) {
			group = argv[++i];
		} else if (!strcmp(argv[i], "-L") && i + 1 < argc) {
			lib = pcb_grow(lib, nlib, sizeof *lib);
			lib[nlib] = pcb_alloc(strlen(argv[i + 1]) + 16);
			sprintf(lib[nlib++], "%s/footprints", argv[++i]);
		} else {
			pcb_die("%s", usage);
		}
	}
	if (i != argc - 1 && i != argc - 2) {
		pcb_die("%s", usage);
	}
	in = pcb_open_lock(i + 1 < argc ? argv[i + 1] : NULL);
	failed = pcb_passes(argv[i], lib, nlib, group, in, stdout, pass);
	if (in && in != stdin) {
		fclose(in);
	}
	for (size_t k = 0; k < nlib; k++) {
		free(lib[k]);
	}
	free(lib);
	return failed;
}
