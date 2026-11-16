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
// The fixed parts are placed first, the rest in the order their
// anchors allow.  A part
// without rot is turned the way, of four, that brings its pads nearest
// to the pads of their nets already placed; then it is moved out the
// way it lies until its courtyard is clear of all placed and inside
// the board.  With -v each placement is told on stderr, with the line
// and the rule it came from, and the difficulty of the group laid out:
// the number of its parts, the parts alike on the same nets counted
// once, and each block one.
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

// Whether part k stands clear: its courtyard off every other placed
// part's on its side and off every plated hole, and inside the board.
static int clear(size_t k)
{
	double c[4];

	pcb_court(&B.part[k], c);
	if (!scope && (c[0] < B.board[0] || c[1] < B.board[1] ||
		       c[2] > B.board[2] || c[3] > B.board[3])) {
		return 0;
	}
	for (size_t i = 0; i < B.npart; i++) {
		double d[4];

		if (i == k || !B.part[i].placed || !B.part[i].fp ||
		    (block_of[k] >= 0 && block_of[i] == block_of[k])) {
			continue;
		}
		pcb_court(&B.part[i], d);
		if (B.part[i].bottom == B.part[k].bottom && overlap(c, d)) {
			return 0;
		}
		// A plated hole is copper on both sides.
		for (size_t p = 0; p < B.part[i].fp->npad; p++) {
			struct pcb_spot s;
			double h[4];

			if (!B.part[i].fp->pad[p].hole) {
				continue;
			}
			pcb_spot(&B, &B.part[i], p, &s);
			h[0] = s.x - s.w / 2 - B.clearance;
			h[1] = s.y - s.h / 2 - B.clearance;
			h[2] = s.x + s.w / 2 + B.clearance;
			h[3] = s.y + s.h / 2 + B.clearance;
			if (overlap(c, h)) {
				return 0;
			}
		}
	}
	return 1;
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

// Whether every part of block b stands clear of all outside it.
static int block_clear(size_t b)
{
	for (size_t i = 0; i < B.npart; i++) {
		if (block_of[i] == (long)b && !clear(i)) {
			return 0;
		}
	}
	return 1;
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
	q->by = pcb_by(&B, k, st);
	if (verbose) {
		fprintf(stderr, "%s:%d: %s (%s) at %.3f %.3f rot %d: %s\n",
			B.file, st->line, q->name, pcb_ref(q), q->x, q->y, q->rot,
			how);
	}
	return 1;
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
	// A fixed root is the mechanics': the block stays where it puts it.
	if (way >= 0 && !block_clear(b) && !push_block(b, way)) {
		pcb_error(&B, B.stmt[stmt_of[r]].line, "no room for block %s",
			  bl->group);
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
static int pass(const char *file, char **lib, size_t nlib, const char *group,
		FILE *in, FILE *out)
{
	size_t nfixed = 0;
	int *kept, *done;

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
		if (B.part[k].fp && !B.part[k].placed && in_scope(k)) {
			pcb_error(&B, 0, "%s (%s) is not placed", B.part[k].name,
				  pcb_ref(&B.part[k]));
		}
	}
	if (verbose) {
		fprintf(stderr,
			"%s: %zu parts kept from the lock; %s: difficulty %zu\n",
			B.file, nfixed, scope ? scope : "the board", difficulty());
	}
	pcb_write_lock(&B, out);
	return B.failed;
}

int main(int argc, char **argv)
{
	char **lib = NULL;
	size_t nlib = 0;
	int i = 1;
	const char *group = NULL;
	const char *usage = "usage: pcbplace [-v] [-g GROUP] [-L LIB]... "
			    "board.pcb [LOCK] > new.lock";

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
	return pcb_passes(argv[i], lib, nlib, group,
			  pcb_open_lock(i + 1 < argc ? argv[i + 1] : NULL),
			  stdout, pass);
}
