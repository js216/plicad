// SPDX-License-Identifier: GPL-3.0-only
// pcbroute.c -- Route the copper of a board as its source says
// Copyright (c) 2026 Jakob Kastelic

// Usage: pcbroute [-v] [-g GROUP] [-L LIB]... board.pcb [LOCK] > new.lock
//
// Reads the source (pcb(5)), its netlist and footprints, and a lock with
// the parts placed (from pcbplace), and writes the lock with the copper.
// The copper is kept by connection, from a pad to a pad or to its via:
// a connection keeps its copper if the statements about its net, its
// pads and its width are as the lock says they were made from, and its
// copper is clear of the pads as they now stand; any other is made anew,
// among the copper kept, which nothing moves.  So an edit makes anew
// only the connections it touches.  When a net finds no way, the pad it
// cannot reach is named.
//
// The statements:
//   width NET W            its tracks W mm wide (rule track by default)
//   width PIN PIN W        the copper between two pins of one net
//   path NET PIN PIN...    its pins wired in this order, one to the next
//   via PIN WAY            the via of a pin of a plane net goes that way
// A net of a plane layer gets a via by each of its pads, on a neck
// track, the nearest place clear of all else; the via joins the pad to
// the plane.  Holes join every layer.  Every other net is routed on the
// top layer as a tree of connections from pad center to pad center: the
// pins of its paths first, then the pins of its widths, then the nearest
// pad not yet joined to the nearest joined, each by the cheapest path
// over a grid of 0.05 mm in eight ways laid through the center of its
// first pad, by A* (Dijkstra's method led by the octile distance), which
// leaves and enters each pad along its longer side and turns only
// outside it, keeping its clearance from all copper of other nets and
// from the edge; a bend costs a little more than its length.  So pads in
// line are joined by one straight track through their centers.  The
// nets are routed with the fewest and nearest pads first; a net that
// finds no way is put first and all that was made anew is made again.
// With -v each connection and via is told on stderr.
//
// A run routes every group of a block statement alone, the deepest
// first, then the board, each section of the lock in turn (the lock from
// LOCK, or stdin; a file not there is nothing locked yet).  With -g
// GROUP the group alone is routed, as pcbplace -g laid it out:
// its nets whose pads are all of it (a net to a port waits for the
// layout round it), and the vias of its pads, with no outline.  The
// copper of a block (block GROUP) is its section's, moved with its
// parts, and kept as it is.

#define _POSIX_C_SOURCE 200809L // For fmemopen.

#include "pcbsrc.h"
#include <math.h>
#include <string.h>

#define CELL 0.05 // The grid of the router.
#define BEND 0.5 // The cost of a turn of 45 degrees, in cells.
#define ROOM 3.0 // Round the parts, the extent of the grid.
#define TRIES 20
#define PI 3.14159265358979323846

static struct pcb B;
static int verbose;
static int **padnet; // The net of each pad of each part, worked out once.
// The grid: cell (i, j) is at ((X0 + i) CELL + OX, (Y0 + j) CELL + OY),
// laid through the center of the pad a connection leaves.
static int X0, Y0, W, H;
static double OX, OY;
static unsigned char *bad; // Cells the net now routed may not take.
static double *cost, *prio; // The cost of a state, and with the estimate.
static int tb[4]; // The target's cell, twice, for the estimate.
static int *from, *heap;
static unsigned char *inp, *inq; // The cells in the pads left and entered.
static size_t nheap, capheap;
static int *kept; // Of each connection: read from the lock and kept.
static size_t nkept;
static const char *scope; // The group routed alone, or NULL.

static int of_group(const char *name, const char *group)
{
	size_t n = strlen(group);

	return !strncmp(name, group, n) && name[n] == '.';
}

// Whether net n is routed here: all its pads of the group routed.
static int net_in_scope(size_t n)
{
	for (size_t j = 0; scope && j < B.net[n].nend; j++) {
		if (!of_group(B.part[B.net[n].end[j].part].name, scope)) {
			return 0;
		}
	}
	return 1;
}

// Whether net n has a pad here.
static int net_here(size_t n)
{
	for (size_t j = 0; j < B.net[n].nend; j++) {
		if (!scope || of_group(B.part[B.net[n].end[j].part].name, scope)) {
			return 1;
		}
	}
	return 0;
}

static const int DI[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
static const int DJ[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };

// The whole number at or under, and at or over, v.
static int down_int(double v)
{
	double f = floor(v);

	return (int)f;
}

static int up_int(double v)
{
	double c = ceil(v);

	return (int)c;
}

static double cx(int i)
{
	return (X0 + i) * CELL + OX;
}

static double cy(int j)
{
	return (Y0 + j) * CELL + OY;
}

// A part for a message: NAME (REF).
static const char *part_label(const struct pcb_part *q)
{
	static char buf[1100];

	snprintf(buf, sizeof buf, "%s (%s)", q->name, pcb_ref(q));
	return buf;
}

static int inside_spot(const struct pcb_spot *s, double x, double y,
		       double grow)
{
	if (s->shape == PAD_CIRCLE) {
		return hypot(x - s->x, y - s->y) <= s->w / 2 + grow;
	}
	return fabs(x - s->x) <= s->w / 2 + grow &&
	       fabs(y - s->y) <= s->h / 2 + grow;
}

// Whether a disk at (x, y) of radius r on the layer (-1 all) keeps its
// clearance from all copper not of net n; sets *who to what is hit.
static int free_at(double x, double y, double r, int layer, int n,
		   const char **who)
{
	double c = B.clearance;

	for (size_t q = 0; q < B.npart; q++) {
		const struct pcb_part *p = &B.part[q];

		if (!p->placed || !p->fp) {
			continue;
		}
		for (size_t k = 0; k < p->fp->npad; k++) {
			struct pcb_spot s;

			pcb_spot(&B, p, k, &s);
			if (padnet[q][k] == n && n >= 0) {
				continue;
			}
			if (s.layer >= 0 && layer >= 0 && s.layer != layer) {
				continue;
			}
			if (inside_spot(&s, x, y, r + c)) {
				*who = part_label(p);
				return 0;
			}
		}
	}
	for (size_t i = 0; i < B.ntrack; i++) {
		const struct pcb_track *t = &B.track_[i];

		if (t->net == n || (layer >= 0 && t->layer != layer)) {
			continue;
		}
		for (size_t j = 1; j < t->n; j++) {
			if (pcb_seg_dist(x, y, t->xy[2 * j - 2],
					 t->xy[2 * j - 1], t->xy[2 * j],
					 t->xy[2 * j + 1]) < r + c + t->w / 2) {
				*who = B.net[t->net].name;
				return 0;
			}
		}
	}
	for (size_t i = 0; i < B.nvia; i++) {
		const struct pcb_via *v = &B.via[i];

		if (v->net != n &&
		    hypot(x - v->x, y - v->y) < r + c + v->dia / 2) {
			*who = B.net[v->net].name;
			return 0;
		}
	}
	if (!scope &&
	    (x - r < B.board[0] + B.edge || x + r > B.board[2] - B.edge ||
	     y - r < B.board[1] + B.edge || y + r > B.board[3] - B.edge)) {
		*who = "the edge";
		return 0;
	}
	return 1;
}

// Whether a disk at (x, y) of radius r on the layer (-1 all) keeps its
// clearance from the pads not of net n; sets *who to the part hit.
static int pads_clear(double x, double y, double r, int layer, int n,
		      const char **who)
{
	for (size_t q = 0; q < B.npart; q++) {
		const struct pcb_part *p = &B.part[q];

		for (size_t k = 0; p->placed && p->fp && k < p->fp->npad; k++) {
			struct pcb_spot s;

			pcb_spot(&B, p, k, &s);
			if ((padnet[q][k] == n && n >= 0) ||
			    (s.layer >= 0 && layer >= 0 && s.layer != layer)) {
				continue;
			}
			if (inside_spot(&s, x, y, r + B.clearance)) {
				*who = part_label(p);
				return 0;
			}
		}
	}
	return 1;
}

// Marks bad the cells within d of a box (a rectangle or, with round,
// a disk in it) or of a segment.
static void mark_box(double x0, double y0, double x1, double y1, int round,
		     double d)
{
	int i0 = down_int((x0 - d - OX) / CELL) - X0,
	    i1 = up_int((x1 + d - OX) / CELL) - X0;
	int j0 = down_int((y0 - d - OY) / CELL) - Y0,
	    j1 = up_int((y1 + d - OY) / CELL) - Y0;
	double mx = (x0 + x1) / 2, my = (y0 + y1) / 2, r = (x1 - x0) / 2;

	for (int j = j0 < 0 ? 0 : j0; j <= j1 && j < H; j++) {
		for (int i = i0 < 0 ? 0 : i0; i <= i1 && i < W; i++) {
			double x = cx(i), y = cy(j), dx, dy;

			if (round) {
				if (hypot(x - mx, y - my) < r + d) {
					bad[j * W + i] = 1;
				}
				continue;
			}
			dx = fmax(fmax(x0 - x, x - x1), 0);
			dy = fmax(fmax(y0 - y, y - y1), 0);
			if (hypot(dx, dy) < d) {
				bad[j * W + i] = 1;
			}
		}
	}
}

static void mark_seg(double ax, double ay, double bx, double by, double d)
{
	int i0 = down_int((fmin(ax, bx) - d - OX) / CELL) - X0;
	int i1 = up_int((fmax(ax, bx) + d - OX) / CELL) - X0;
	int j0 = down_int((fmin(ay, by) - d - OY) / CELL) - Y0;
	int j1 = up_int((fmax(ay, by) + d - OY) / CELL) - Y0;

	for (int j = j0 < 0 ? 0 : j0; j <= j1 && j < H; j++) {
		for (int i = i0 < 0 ? 0 : i0; i <= i1 && i < W; i++) {
			if (pcb_seg_dist(cx(i), cy(j), ax, ay, bx, by) < d) {
				bad[j * W + i] = 1;
			}
		}
	}
}

// Sets up the grid round the placed parts for net n of width w, through
// (ox, oy): the cells its track may not take, each obstacle marked in
// the cells round it.
static void grid(int n, double w, double ox, double oy)
{
	double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9,
	       d = w / 2 + B.clearance;

	for (size_t q = 0; q < B.npart; q++) {
		double c[4];

		if (!B.part[q].placed || !B.part[q].fp) {
			continue;
		}
		pcb_court(&B.part[q], c);
		if (c[2] - c[0] > 50) {
			continue; // A board-wide part, as a block, spans nothing here.
		}
		x0 = fmin(x0, c[0]);
		y0 = fmin(y0, c[1]);
		x1 = fmax(x1, c[2]);
		y1 = fmax(y1, c[3]);
	}
	// Round the parts, but never past the board (a group has none).
	x0 = scope ? x0 - ROOM : fmax(x0 - ROOM, B.board[0]);
	y0 = scope ? y0 - ROOM : fmax(y0 - ROOM, B.board[1]);
	x1 = scope ? x1 + ROOM : fmin(x1 + ROOM, B.board[2]);
	y1 = scope ? y1 + ROOM : fmin(y1 + ROOM, B.board[3]);
	OX = fmod(ox, CELL);
	OY = fmod(oy, CELL);
	X0 = down_int((x0 - OX) / CELL);
	Y0 = down_int((y0 - OY) / CELL);
	W = up_int((x1 - x0) / CELL) + 1;
	H = up_int((y1 - y0) / CELL) + 1;
	free(bad);
	bad = pcb_alloc((size_t)W * (size_t)H);
	for (size_t q = 0; q < B.npart; q++) {
		const struct pcb_part *p = &B.part[q];

		for (size_t k = 0; p->placed && p->fp && k < p->fp->npad; k++) {
			struct pcb_spot sp;

			pcb_spot(&B, p, k, &sp);
			if ((padnet[q][k] == n && n >= 0) || sp.layer > 0) {
				continue;
			}
			mark_box(sp.x - sp.w / 2, sp.y - sp.h / 2,
				 sp.x + sp.w / 2, sp.y + sp.h / 2,
				 sp.shape == PAD_CIRCLE, d);
		}
	}
	for (size_t i = 0; i < B.ntrack; i++) {
		const struct pcb_track *t = &B.track_[i];

		if (t->net == n || t->layer != 0) {
			continue;
		}
		for (size_t j = 1; j < t->n; j++) {
			mark_seg(t->xy[2 * j - 2], t->xy[2 * j - 1],
				 t->xy[2 * j], t->xy[2 * j + 1], d + t->w / 2);
		}
	}
	for (size_t i = 0; i < B.nvia; i++) {
		const struct pcb_via *v = &B.via[i];

		if (v->net != n) {
			mark_box(v->x - v->dia / 2, v->y - v->dia / 2,
				 v->x + v->dia / 2, v->y + v->dia / 2, 1, d);
		}
	}
	for (int j = 0; !scope && j < H; j++) {
		for (int i = 0; i < W; i++) {
			double x = cx(i), y = cy(j), e = B.edge + w / 2;

			if (x < B.board[0] + e || x > B.board[2] - e ||
			    y < B.board[1] + e || y > B.board[3] - e) {
				bad[j * W + i] = 1;
			}
		}
	}
}

static void push(int s)
{
	size_t i;

	if (nheap == capheap) {
		capheap = capheap ? 2 * capheap : 1 << 16;
		heap = pcb_grow(heap, capheap - 1, sizeof *heap);
	}
	i = nheap++;
	heap[i] = s;
	while (i > 0 && prio[heap[(i - 1) / 2]] > prio[heap[i]]) {
		int t = heap[(i - 1) / 2];

		heap[(i - 1) / 2] = heap[i];
		heap[i] = t;
		i = (i - 1) / 2;
	}
}

static int pop(void)
{
	int top = heap[0];
	size_t i = 0;

	heap[0] = heap[--nheap];
	for (;;) {
		size_t l = 2 * i + 1, r = l + 1, m = i;
		int t;

		if (l < nheap && prio[heap[l]] < prio[heap[m]]) {
			m = l;
		}
		if (r < nheap && prio[heap[r]] < prio[heap[m]]) {
			m = r;
		}
		if (m == i) {
			return top;
		}
		t = heap[m];
		heap[m] = heap[i];
		heap[i] = t;
		i = m;
	}
}

// The ways along the longer side of a pad, as a mask of the eight
// ways: right and left, or up and down; any of the four for a pad round
// or about as long as it is wide (within a tenth).
static unsigned axis(const struct pcb_spot *s)
{
	if (s->shape == PAD_CIRCLE ||
	    fabs(s->w - s->h) < 0.1 * fmax(s->w, s->h)) {
		return 0x55;
	}
	return s->w > s->h ? 0x11 : 0x44;
}

// The least the path from cell c to the target can cost: the octile
// distance to its cell (A*).
static double estimate(int c)
{
	int i = c % W, j = c / W;
	int dx = abs(tb[0] - i), dy = abs(tb[1] - j);
	int lo = dx < dy ? dx : dy, hi = dx < dy ? dy : dx;

	return hi - lo + lo * 1.41421356;
}

// The cell nearest (x, y), or -1 off the grid.
static int cell_at(double x, double y)
{
	long i = lround((x - OX) / CELL) - X0, j = lround((y - OY) / CELL) - Y0;

	return i < 0 || j < 0 || i >= W || j >= H ? -1 : (int)(j * W + i);
}

// The connections of the -v trace, and of the messages: PART.PAD (REF).
static const char *pad_label(const struct pcb_end *e)
{
	static char buf[2][1200];
	static int k;
	const struct pcb_part *q = &B.part[e->part];
	char *s = buf[k++ % 2];

	snprintf(s, sizeof buf[0], "%s.%s (%s)", q->name,
		 q->fp->pad[e->pad].name, pcb_ref(q));
	return s;
}

// Joins pad a of net n to pad b: a connection of width w from the center
// of a to the center of b, leaving and entering each along its longer
// side.  Returns 0 if there is no way.
static int connect(int n, const struct pcb_end *a, const struct pcb_end *b,
		   double w)
{
	struct pcb_spot sa, sb;
	unsigned ma, mb;
	int ns, goal = -1, ca, cb, l;
	char *ka, *kb;

	pcb_spot(&B, &B.part[a->part], a->pad, &sa);
	pcb_spot(&B, &B.part[b->part], b->pad, &sb);
	ma = axis(&sa);
	mb = axis(&sb);
	grid(n, w, sa.x, sa.y);
	ns = W * H * 8;
	free(cost);
	free(prio);
	free(from);
	free(inp);
	free(inq);
	cost = pcb_alloc((size_t)ns * sizeof *cost);
	prio = pcb_alloc((size_t)ns * sizeof *prio);
	from = pcb_alloc((size_t)ns * sizeof *from);
	inp = pcb_alloc((size_t)W * (size_t)H);
	inq = pcb_alloc((size_t)W * (size_t)H);
	ca = cell_at(sa.x, sa.y);
	cb = cell_at(sb.x, sb.y);
	if (ca < 0 || cb < 0) {
		return 0;
	}
	for (int s = 0; s < ns; s++) {
		cost[s] = 1e300;
		from[s] = -1;
	}
	for (int c = 0; c < W * H; c++) {
		inp[c] = (unsigned char)inside_spot(&sa, cx(c % W), cy(c / W), 0);
		inq[c] = (unsigned char)inside_spot(&sb, cx(c % W), cy(c / W), 0);
	}
	tb[0] = cb % W;
	tb[1] = cb / W;
	nheap = 0;
	// Pads in line: one straight track through their centers, if clear.
	if (fabs(sa.x - sb.x) < 1e-6 || fabs(sa.y - sb.y) < 1e-6) {
		int di = (cb % W > ca % W) - (cb % W < ca % W);
		int dj = (cb / W > ca / W) - (cb / W < ca / W), d;
		int clear = 1;

		for (int c = ca; clear && c != cb; c += dj * W + di) {
			clear = !bad[c] || inp[c] || inq[c];
		}
		for (d = 0; d < 8 && (DI[d] != di || DJ[d] != dj); d++) {
		}
		if (clear && d < 8) {
			cost[cb * 8 + d] = 1;
			from[cb * 8 + d] = ca * 8 + d;
			cost[ca * 8 + d] = 0;
			goal = cb * 8 + d;
		}
	}
	for (int d = 0; goal < 0 && d < 8; d++) {
		if (ma >> d & 1) {
			cost[ca * 8 + d] = 0;
			prio[ca * 8 + d] = estimate(ca);
			push(ca * 8 + d);
		}
	}
	while (goal < 0 && nheap) {
		int s = pop(), c = s / 8, d = s % 8, i = c % W, j = c / W;

		if (c == cb && cost[s] > 0) {
			goal = s;
			break;
		}
		for (int nd = 0; nd < 8; nd++) {
			int ni = i + DI[nd], nj = j + DJ[nd], nc, turn;
			double step;

			// Straight while in a pad: out of the first along its
			// side, into the second along its side.
			if ((inp[c] || inq[c]) && nd != d) {
				continue;
			}
			if (ni < 0 || nj < 0 || ni >= W || nj >= H) {
				continue;
			}
			nc = nj * W + ni;
			if (inq[nc] && !(mb >> nd & 1)) {
				continue;
			}
			if (bad[nc] && !inq[nc] && !inp[nc]) {
				continue;
			}
			turn = abs(nd - d);
			turn = turn > 4 ? 8 - turn : turn;
			if (turn > 2) {
				continue; // No sharp turns.
			}
			step = (nd % 2 ? 1.41421356 : 1) + turn * BEND;
			if (cost[s] + step < cost[nc * 8 + nd]) {
				cost[nc * 8 + nd] = cost[s] + step;
				prio[nc * 8 + nd] = cost[s] + step + estimate(nc);
				from[nc * 8 + nd] = s;
				push(nc * 8 + nd);
			}
		}
	}
	if (goal < 0) {
		return 0;
	}
	ka = pcb_pad_key(&B, a->part, a->pad);
	kb = pcb_pad_key(&B, b->part, b->pad);
	l = pcb_add_link(&B, n, ka, kb, w);
	free(ka);
	free(kb);
	{
		// The path, as a polyline of its corners, from b back to a;
		// its ends at the pads' centers, its last run on b's line.
		struct pcb_track t;
		size_t cap = 16;
		int last = -1, fd = goal % 8;

		t.net = n;
		t.layer = 0;
		t.link = l;
		t.w = w;
		t.n = 0;
		t.xy = pcb_alloc(2 * cap * sizeof *t.xy);
		for (int s = goal; s >= 0; s = from[s]) {
			int d = s % 8;

			if (t.n + 1 >= cap) {
				cap *= 2;
				t.xy = realloc(t.xy, 2 * cap * sizeof *t.xy);
				if (!t.xy) {
					pcb_die("out of memory");
				}
			}
			if (last < 0 || d != last || from[s] < 0) {
				t.xy[2 * t.n] = cx(s / 8 % W);
				t.xy[2 * t.n + 1] = cy(s / 8 / W);
				t.n++;
			}
			last = d;
		}
		if (t.n >= 3) {
			// The corner before b on b's line.
			if (fd == 0 || fd == 4) {
				t.xy[3] = sb.y;
			} else {
				t.xy[2] = sb.x;
			}
		}
		t.xy[0] = sb.x;
		t.xy[1] = sb.y;
		t.xy[2 * t.n - 2] = sa.x;
		t.xy[2 * t.n - 1] = sa.y;
		B.track_ = pcb_grow(B.track_, B.ntrack, sizeof *B.track_);
		B.track_[B.ntrack++] = t;
	}
	if (verbose) {
		fprintf(stderr, "%s: %s to %s, %.3f wide\n", B.net[n].name,
			pad_label(a), pad_label(b), w);
	}
	return 1;
}

// The ends of net n to route on the top: its pads on the top.
static size_t top_ends(int n, struct pcb_end **ends)
{
	size_t m = 0;

	*ends = pcb_alloc((B.net[n].nend + 1) * sizeof **ends);
	for (size_t j = 0; j < B.net[n].nend; j++) {
		struct pcb_spot s;
		const struct pcb_end *e = &B.net[n].end[j];

		if (!B.part[e->part].placed) {
			continue;
		}
		pcb_spot(&B, &B.part[e->part], e->pad, &s);
		if (s.layer <= 0) {
			(*ends)[m++] = *e;
		}
	}
	return m;
}

// The group of each end, joined by the connections there are: the end
// of the least index of its group.
static void groups(int n, const struct pcb_end *ends, size_t m, size_t *g)
{
	char **key = pcb_alloc((m + 1) * sizeof *key);

	for (size_t j = 0; j < m; j++) {
		g[j] = j;
		key[j] = pcb_pad_key(&B, ends[j].part, ends[j].pad);
	}
	for (int changed = 1; changed;) {
		changed = 0;
		for (size_t l = 0; l < B.nlink; l++) {
			size_t x = m, y = m;

			if (B.link[l].net != n) {
				continue;
			}
			for (size_t j = 0; j < m; j++) {
				x = !strcmp(key[j], B.link[l].a) ? j : x;
				y = !strcmp(key[j], B.link[l].b) ? j : y;
			}
			if (x < m && y < m && g[x] != g[y]) {
				size_t lo = g[x] < g[y] ? g[x] : g[y];
				size_t hi = g[x] < g[y] ? g[y] : g[x];

				for (size_t j = 0; j < m; j++) {
					g[j] = g[j] == hi ? lo : g[j];
				}
				changed = 1;
			}
		}
	}
	for (size_t j = 0; j < m; j++) {
		free(key[j]);
	}
	free(key);
}

// The end of a pin among the ends, or m.
static size_t end_of(const char *pin, const struct pcb_end *ends, size_t m)
{
	size_t q, p;

	for (size_t j = 0; pcb_pin(&B, pin, &q, &p) && j < m; j++) {
		if (ends[j].part == q && ends[j].pad == p) {
			return j;
		}
	}
	return m;
}

// Joins ends i and j of net n, if not joined yet.
static int join(int n, const struct pcb_end *ends, size_t m, size_t *g,
		size_t i, size_t j)
{
	char *ka, *kb;
	double w;
	int ok;

	if (i >= m || j >= m || g[i] == g[j]) {
		return 1;
	}
	ka = pcb_pad_key(&B, ends[i].part, ends[i].pad);
	kb = pcb_pad_key(&B, ends[j].part, ends[j].pad);
	w = pcb_link_width(&B, n, ka, kb);
	free(ka);
	free(kb);
	ok = connect(n, &ends[i], &ends[j], w);
	if (ok) {
		groups(n, ends, m, g);
	}
	return ok;
}

// Routes signal net n: joins its groups of pads by connections, those
// its paths and widths name first, then the nearest pad not joined to
// the nearest joined.  Returns 0 if a pad finds no way, and names it in
// *who.
static int route_net(int n, const char **who)
{
	struct pcb_end *ends;
	size_t m = top_ends(n, &ends), *g = pcb_alloc((m + 1) * sizeof *g);
	int ok = 1;

	groups(n, ends, m, g);
	for (size_t i = 0; ok && i < B.nstmt; i++) {
		const struct pcb_stmt *st = &B.stmt[i];

		if (!strcmp(st->w[0], "path") && st->n >= 4 &&
		    !strcmp(st->w[1], B.net[n].name)) {
			for (int k = 3; ok && k < st->n; k++) {
				size_t a = end_of(st->w[k - 1], ends, m);
				size_t b = end_of(st->w[k], ends, m);

				if (a == m || b == m) {
					pcb_error(&B, st->line, "no pin %s on the top",
						  st->w[a == m ? k - 1 : k]);
					continue;
				}
				ok = join(n, ends, m, g, a, b);
				*who = ok ? *who : pad_label(&ends[b]);
			}
		} else if (!strcmp(st->w[0], "width") && st->n == 4) {
			size_t a = end_of(st->w[1], ends, m);
			size_t b = end_of(st->w[2], ends, m);

			if (a < m && b < m) {
				ok = join(n, ends, m, g, a, b);
				*who = ok ? *who : pad_label(&ends[b]);
			}
		}
	}
	while (ok) {
		size_t bi = m, bj = m;
		double bd = 1e300;

		for (size_t i = 0; i < m; i++) {
			struct pcb_spot a;

			if (g[i] == g[0]) {
				continue;
			}
			pcb_spot(&B, &B.part[ends[i].part], ends[i].pad, &a);
			for (size_t j = 0; j < m; j++) {
				struct pcb_spot o;
				double d;

				if (g[j] != g[0]) {
					continue;
				}
				pcb_spot(&B, &B.part[ends[j].part], ends[j].pad,
					 &o);
				d = hypot(a.x - o.x, a.y - o.y);
				if (d < bd - 1e-9) {
					bd = d;
					bi = i;
					bj = j;
				}
			}
		}
		if (bi == m) {
			break;
		}
		ok = join(n, ends, m, g, bi, bj);
		if (!ok) {
			*who = pad_label(&ends[bi]);
		}
	}
	free(ends);
	free(g);
	return ok;
}

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

// Whether a via of net n at (x, y), of radius r, keeps the via gap from
// every pad of its net (its ring that far from their edges, so it joins
// them by a neck track, not standing on them) and the mask web from
// every opening of the mask, on either side, for the via is tented.
static int via_room(int n, double x, double y, double r, const char **who)
{
	for (size_t q = 0; q < B.npart; q++) {
		const struct pcb_part *p = &B.part[q];

		for (size_t k = 0; p->placed && p->fp && k < p->fp->npad; k++) {
			struct pcb_spot s;
			double o[4];

			pcb_spot(&B, p, k, &s);
			if (padnet[q][k] == n && pcb_soldered(p, k) &&
			    pcb_spot_dist(&s, x, y) < r + B.viagap - 1e-9) {
				*who = part_label(p);
				return 0;
			}
			// A contact pad's opening: bare copper, no solder to
			// bridge to the via.
			for (int side = 0; pcb_soldered(p, k) && side < 2; side++) {
				if (pcb_opening(&B, q, k, side, o) &&
				    hypot(fmax(fmax(o[0] - x, x - o[2]), 0),
					  fmax(fmax(o[1] - y, y - o[3]), 0)) <
					    r + B.maskweb - 1e-9) {
					*who = part_label(p);
					return 0;
				}
			}
		}
	}
	return 1;
}

// Adds a track of net n, of connection link, on a layer from (ax, ay)
// to (bx, by).
static void add_track(int n, int link, int layer, double w, double ax,
		      double ay, double bx, double by)
{
	struct pcb_track t;

	t.net = n;
	t.link = link;
	t.layer = layer;
	t.w = w;
	t.n = 2;
	t.xy = pcb_alloc(4 * sizeof *t.xy);
	t.xy[0] = ax;
	t.xy[1] = ay;
	t.xy[2] = bx;
	t.xy[3] = by;
	B.track_ = pcb_grow(B.track_, B.ntrack, sizeof *B.track_);
	B.track_[B.ntrack++] = t;
}

// The neck of pad s of part q on its plane's layer: out of the pad along
// its longer side, away from the middle of the part, past the clearance
// of the pour, so the pad joins its plane by the neck alone.
static void neck(int n, int link, const struct pcb_part *q,
		 const struct pcb_spot *s, double w)
{
	int along_x = s->w >= s->h;
	double ox = s->x - q->x, oy = s->y - q->y;
	double ux = along_x ? (ox < 0 ? -1 : 1) : 0;
	double uy = along_x ? 0 : (oy < 0 ? -1 : 1);
	double len = (along_x ? s->w : s->h) / 2 + B.clearance + 2 * w;

	add_track(n, link, s->layer, w, s->x, s->y, s->x + ux * len,
		  s->y + uy * len);
	if (verbose) {
		fprintf(stderr, "%s: neck of %s (%s) on its plane\n",
			B.net[n].name, q->name, pcb_ref(q));
	}
}

// The width of the neck of pad k of part q, of plane net n: the net's,
// but no wider than a track on a small part of two pins, which takes
// heat alike at its two pads.
static double neck_width(int n, size_t q)
{
	double w = B.net[n].width;

	return pcb_small_two_pin(&B.part[q]) ? fmin(w, B.track) : w;
}

// Whether a connection of net n from pad key (to its via or plane) is
// there.
static int has_link(int n, const char *key)
{
	for (size_t l = 0; l < B.nlink; l++) {
		if (B.link[l].net == n && !strcmp(B.link[l].a, key)) {
			return 1;
		}
	}
	return 0;
}

// Gives each pad of plane net n a via, on a neck track: the nearest
// place clear of all else, the via gap from the pads of its net and the
// mask web from every opening, out the way the source says or, else,
// away from the middle of its part.  A pad on its plane's own layer
// needs none: it is in the pour, or, a pad of a small part of two pins,
// joined to it by a neck.
static int vias(int n)
{
	int ok = 1;

	for (size_t j = 0; j < B.net[n].nend; j++) {
		const struct pcb_end *e = &B.net[n].end[j];
		const struct pcb_part *q = &B.part[e->part];
		struct pcb_spot s;
		double best = 1e300, bx = 0, by = 0, r = B.via_dia / 2;
		double w;
		int want = -1, have;
		const char *who = "";
		char *key;

		if (!q->placed) {
			continue;
		}
		pcb_spot(&B, q, e->pad, &s);
		if (s.layer < 0) {
			continue; // A hole is on the plane already.
		}
		key = pcb_pad_key(&B, e->part, e->pad);
		have = has_link(n, key);
		w = neck_width(n, e->part);
		if (s.layer == B.net[n].plane) {
			if (!have && pcb_relieved(&B, e->part, e->pad)) {
				neck(n, pcb_add_link(&B, n, key, "plane", w), q, &s,
				     w);
			}
			free(key);
			continue;
		}
		if (have) {
			free(key);
			continue; // Kept.
		}
		{
			int holed = 0;

			// Nor does a pad of a pin that has a hole too.
			for (size_t p = 0; p < q->fp->npad; p++) {
				holed |= q->fp->pad[p].hole &&
					 !strcmp(q->fp->pad[p].name,
						 q->fp->pad[e->pad].name);
			}
			if (holed) {
				continue;
			}
		}
		for (size_t i = 0; i < B.nstmt; i++) {
			const struct pcb_stmt *st = &B.stmt[i];
			size_t pq, pp;

			if (!strcmp(st->w[0], "via") && st->n == 3 &&
			    pcb_pin(&B, st->w[1], &pq, &pp) && pq == e->part &&
			    !strcmp(q->fp->pad[pp].name,
				    q->fp->pad[e->pad].name)) {
				want = way_word(st->w[2]);
			}
		}
		for (int d = 0; d < 16; d++) {
			double a = d * PI / 8, ux = cos(a), uy = sin(a);
			double ox = s.x - q->x, oy = s.y - q->y, out;
			int dir = (int)lround(a / (PI / 2)) % 4;

			if (want >= 0 && (d % 4 || dir != want)) {
				continue;
			}
			// Out from the part, rather than over it.
			out = hypot(ox, oy) > 1e-9 ?
				      (ux * ox + uy * oy) / hypot(ox, oy) :
				      1;
			for (double t = 0.3; t < 3; t += CELL) {
				double x = s.x + ux * t, y = s.y + uy * t,
				       score;

				if (!via_room(n, x, y, r, &who) ||
				    !free_at(x, y, r, -1, n, &who)) {
					continue;
				}
				// The neck from the pad keeps clear too.
				{
					int clear = 1;

					for (double f = 0; f <= 1 && clear;
					     f += 0.05) {
						clear = free_at(
							s.x + (x - s.x) * f,
							s.y + (y - s.y) * f,
							w / 2, s.layer, n, &who);
					}
					if (!clear) {
						continue;
					}
				}
				score = t - out * 0.3;
				if (score < best) {
					best = score;
					bx = x;
					by = y;
				}
				break;
			}
		}
		if (best > 1e299) {
			pcb_error(
				&B, 0,
				"net %s: no room for the via of %s.%s (%s); in the way: %s",
				B.net[n].name, q->name, q->fp->pad[e->pad].name,
				pcb_ref(q), who);
			ok = 0;
			free(key);
			continue;
		}
		{
			int l = pcb_add_link(&B, n, key, "via", w);
			struct pcb_via v = { n, l, bx, by, B.via_dia,
					     B.via_drill };

			free(key);
			B.via = pcb_grow(B.via, B.nvia, sizeof *B.via);
			B.via[B.nvia++] = v;
			add_track(n, l, s.layer, w, s.x, s.y, bx, by);
			if (verbose) {
				fprintf(stderr,
					"%s: via of %s.%s (%s) at %.3f %.3f\n",
					B.net[n].name, q->name,
					q->fp->pad[e->pad].name, pcb_ref(q), bx,
					by);
			}
		}
	}
	return ok;
}

// Takes the copper of connection l off the board, or, l -1, the copper
// of net n of no connection; a connection taken off is of no net.
static void rip(int n, long l)
{
	size_t k = 0;

	for (size_t i = 0; i < B.ntrack; i++) {
		if (B.track_[i].net == n && B.track_[i].link == l) {
			free(B.track_[i].xy);
		} else {
			B.track_[k++] = B.track_[i];
		}
	}
	B.ntrack = k;
	k = 0;
	for (size_t i = 0; i < B.nvia; i++) {
		if (B.via[i].net != n || B.via[i].link != l) {
			B.via[k++] = B.via[i];
		}
	}
	B.nvia = k;
	if (l >= 0) {
		B.link[l].net = -1;
	}
}

// The line of a pad by its key among lines (the lock's, or the pads'
// as they stand), or NULL.
static const char *pad_line(char **lines, size_t n, const char *key)
{
	size_t len = strlen(key);

	for (size_t i = 0; i < n; i++) {
		if (!strncmp(lines[i], key, len) && lines[i][len] == ' ') {
			return lines[i];
		}
	}
	return NULL;
}

// The part of a pad's key on net n, or -1.
static long part_of_key(int n, const char *key)
{
	for (size_t j = 0; j < B.net[n].nend; j++) {
		char *k = pcb_pad_key(&B, B.net[n].end[j].part,
				      B.net[n].end[j].pad);
		int same = !strcmp(k, key);

		free(k);
		if (same) {
			return (long)B.net[n].end[j].part;
		}
	}
	return -1;
}

// Whether the copper of connection l keeps clear of the pads of other
// nets as they now stand.
static int link_clear(long l)
{
	int n = B.link[l].net;
	const char *who;

	for (size_t i = 0; i < B.ntrack; i++) {
		const struct pcb_track *t = &B.track_[i];

		for (size_t j = 1; t->link == l && j < t->n; j++) {
			double len = hypot(t->xy[2 * j] - t->xy[2 * j - 2],
					   t->xy[2 * j + 1] - t->xy[2 * j - 1]);

			for (double f = 0; f <= len; f += CELL) {
				double u = len > 0 ? f / len : 0;
				double x = t->xy[2 * j - 2] +
					   (t->xy[2 * j] - t->xy[2 * j - 2]) * u;
				double y = t->xy[2 * j - 1] +
					   (t->xy[2 * j + 1] - t->xy[2 * j - 1]) * u;

				if (!pads_clear(x, y, t->w / 2 - 1e-6, t->layer, n,
						&who)) {
					return 0;
				}
			}
		}
	}
	for (size_t i = 0; i < B.nvia; i++) {
		if (B.via[i].link == l &&
		    !pads_clear(B.via[i].x, B.via[i].y, B.via[i].dia / 2 - 1e-6,
				-1, n, &who)) {
			return 0;
		}
	}
	return 1;
}

// Whether connection l of net n may be kept: its pads where they stood
// (now, as the lock tells them), its width what the statements give now,
// and its copper clear of the pads as they stand.
static int keep_link(long l, char **now, size_t nnow)
{
	const struct pcb_link *k = &B.link[l];
	int n = k->net, to_pad = strcmp(k->b, "via") && strcmp(k->b, "plane");
	const char *a0 = pad_line(B.netpad[n], B.nnetpad[n], k->a);
	const char *a1 = pad_line(now, nnow, k->a);
	long q = part_of_key(n, k->a);
	double w;

	if (!a0 || !a1 || strcmp(a0, a1) || q < 0) {
		return 0;
	}
	if (to_pad) {
		const char *b0 = pad_line(B.netpad[n], B.nnetpad[n], k->b);
		const char *b1 = pad_line(now, nnow, k->b);

		if (!b0 || !b1 || strcmp(b0, b1)) {
			return 0;
		}
		w = pcb_link_width(&B, n, k->a, k->b);
	} else {
		w = neck_width(n, (size_t)q);
	}
	return fabs(w - k->w) < 1e-6 && link_clear(l);
}

// The size of a net, to order them: its pads and their spread.
static double size_of(int n)
{
	double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;

	for (size_t j = 0; j < B.net[n].nend; j++) {
		struct pcb_spot s;

		pcb_spot(&B, &B.part[B.net[n].end[j].part], B.net[n].end[j].pad,
			 &s);
		x0 = fmin(x0, s.x);
		y0 = fmin(y0, s.y);
		x1 = fmax(x1, s.x);
		y1 = fmax(y1, s.y);
	}
	return (x1 - x0) + (y1 - y0) + (double)B.net[n].nend;
}

// Takes in the copper of each block the layout is made of (the groups
// of block statements in scope that no other holds): its lock's, moved
// as pcbplace moved its parts (by its first part placed), in place of
// what the lock had of it; kept as it is.  Returns the first connection
// of the blocks': they are all those from it on.
static size_t block_copper(char **lib, size_t nlib)
{
	size_t first = B.nlink;

	for (size_t s = 0; s < B.nstmt; s++) {
		const struct pcb_stmt *st = &B.stmt[s];
		struct pcb L;
		char *text;
		FILE *f;
		long r = -1;
		int top = st->n == 2 && !strcmp(st->w[0], "block") &&
			  (!scope || of_group(st->w[1], scope));
		double tx, ty;

		for (size_t o = 0; top && o < B.nstmt; o++) {
			const struct pcb_stmt *ot = &B.stmt[o];

			top = !(o != s && !strcmp(ot->w[0], "block") &&
				ot->n == 2 && of_group(st->w[1], ot->w[1]) &&
				(!scope || of_group(ot->w[1], scope)));
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
		pcb_read(&L, B.file, lib, nlib, f, NULL);
		fclose(f);
		for (size_t q = 0; q < B.npart && r < 0; q++) {
			if (of_group(B.part[q].name, st->w[1]) && B.part[q].placed &&
			    L.part[q].placed) {
				r = (long)q;
			}
		}
		if (r < 0) {
			continue;
		}
		tx = round((B.part[r].x - L.part[r].x) * 1e4) / 1e4;
		ty = round((B.part[r].y - L.part[r].y) * 1e4) / 1e4;
		// What the lock had of the block's copper goes.
		for (size_t l = 0; l < first; l++) {
			long qa = B.link[l].net >= 0 ?
					  part_of_key(B.link[l].net, B.link[l].a) :
					  -1;

			if (qa >= 0 &&
			    of_group(B.part[qa].name, st->w[1])) {
				rip(B.link[l].net, (long)l);
			}
		}
		for (size_t l = 0; l < L.nlink; l++) {
			long n = L.link[l].net, nl;

			if (n < 0) {
				continue;
			}
			nl = pcb_add_link(&B, (int)n, L.link[l].a, L.link[l].b,
					  L.link[l].w);
			for (size_t i = 0; i < L.ntrack; i++) {
				struct pcb_track t = L.track_[i];

				if (t.link != (int)l) {
					continue;
				}
				t.link = (int)nl;
				t.xy = pcb_alloc(2 * t.n * sizeof *t.xy);
				for (size_t j = 0; j < t.n; j++) {
					t.xy[2 * j] = L.track_[i].xy[2 * j] + tx;
					t.xy[2 * j + 1] = L.track_[i].xy[2 * j + 1] + ty;
				}
				B.track_ = pcb_grow(B.track_, B.ntrack,
						    sizeof *B.track_);
				B.track_[B.ntrack++] = t;
			}
			for (size_t i = 0; i < L.nvia; i++) {
				struct pcb_via v = L.via[i];

				if (v.link != (int)l) {
					continue;
				}
				v.link = (int)nl;
				v.x += tx;
				v.y += ty;
				B.via = pcb_grow(B.via, B.nvia, sizeof *B.via);
				B.via[B.nvia++] = v;
			}
		}
		if (verbose) {
			fprintf(stderr, "%s: the copper of block %s, moved %.4f %.4f\n",
				B.file, st->w[1], tx, ty);
		}
	}
	return first;
}

// One pass: the copper of the group (the board, for NULL), from its
// section of the lock read from in; the whole lock written to out.
static int pass(const char *file, char **lib, size_t nlib, const char *group,
		FILE *in, FILE *out)
{
	char **words;
	size_t nfresh = 0, ours;
	int *fresh, *order;

	scope = group;
	pcb_read(&B, file, lib, nlib, in, scope);
	padnet = pcb_alloc((B.npart + 1) * sizeof *padnet);
	ours = block_copper(lib, nlib);
	for (size_t q = 0; q < B.npart; q++) {
		size_t np = B.part[q].fp ? B.part[q].fp->npad : 0;

		padnet[q] = pcb_alloc((np + 1) * sizeof **padnet);
		for (size_t k = 0; k < np; k++) {
			padnet[q][k] = pcb_net_of(&B, q, k);
		}
	}
	fresh = pcb_alloc((B.nnet + 1) * sizeof *fresh);
	order = pcb_alloc((B.nnet + 1) * sizeof *order);
	words = pcb_alloc((B.nnet + 1) * sizeof *words);
	// Which connections keep their copper: of a net made by the same
	// statements, their pads where they were, their widths as they were
	// and their copper clear of the pads.
	for (size_t k = 0; k < B.nnet; k++) {
		words[k] = pcb_net_words(&B, k);
		fresh[k] = !B.netby[k] || strcmp(B.netby[k], words[k]);
	}
	nkept = B.nlink;
	kept = pcb_alloc((nkept + 1) * sizeof *kept);
	for (size_t l = 0; l < B.nlink; l++) {
		int k = B.link[l].net;
		size_t np;
		char **now;

		if (k < 0) {
			continue;
		}
		now = pcb_net_pads(&B, (size_t)k, &np);
		// A block's copper is kept as its lock has it.
		kept[l] = l >= ours ||
			  (!fresh[k] && now && keep_link((long)l, now, np));
		for (size_t j = 0; j < np; j++) {
			free(now[j]);
		}
		free(now);
	}
	for (size_t l = 0; l < B.nlink; l++) {
		if (!kept[l] && B.link[l].net >= 0) {
			rip(B.link[l].net, (long)l);
		}
	}
	// Copper of no connection (of a lock not made here, or by hand) is
	// made anew, by connection, for every net routed here.
	for (size_t k = 0; k < B.nnet; k++) {
		if (net_in_scope(k)) {
			rip((int)k, -1);
		}
	}
	for (size_t k = 0; k < B.nnet; k++) {
		order[k] = (int)k;
	}
	for (size_t a = 1; a < B.nnet; a++) {
		for (size_t b = a;
		     b > 0 && size_of(order[b]) < size_of(order[b - 1]); b--) {
			int t = order[b];

			order[b] = order[b - 1];
			order[b - 1] = t;
		}
	}
	for (int tries = 0; tries < TRIES; tries++) {
		const char *who = "";
		int failed = -1;

		// What was made anew goes, to be made again in the new order.
		for (size_t l = 0; tries && l < B.nlink; l++) {
			if ((l >= nkept || !kept[l]) && B.link[l].net >= 0 &&
			    B.net[B.link[l].net].plane < 0) {
				rip(B.link[l].net, (long)l);
			}
		}
		for (size_t o = 0; o < B.nnet; o++) {
			int k = order[o];

			if (B.net[k].plane >= 0 || !net_in_scope((size_t)k)) {
				continue;
			}
			if (!route_net(k, &who)) {
				failed = k;
				break;
			}
		}
		if (failed < 0) {
			break;
		}
		if (tries == TRIES - 1) {
			pcb_error(&B, 0, "net %s: no way to %s",
				  B.net[failed].name, who);
			break;
		}
		for (size_t o = 0; o < B.nnet; o++) {
			if (order[o] == failed) {
				memmove(order + 1, order, o * sizeof *order);
				order[0] = failed;
				break;
			}
		}
	}
	for (size_t o = 0; o < B.nnet; o++) {
		int k = order[o];

		if (B.net[k].plane >= 0) {
			vias(k);
		}
	}
	for (size_t l = nkept; l < B.nlink; l++) {
		nfresh += B.link[l].net >= 0;
	}
	// The lock tells what the copper is made from now, for the nets
	// with a pad here.
	for (size_t k = 0; k < B.nnet; k++) {
		for (size_t j = 0; j < B.nnetpad[k]; j++) {
			free(B.netpad[k][j]);
		}
		free(B.netpad[k]);
		free(B.netby[k]);
		B.netby[k] = net_here(k) ? words[k] : NULL;
		B.netpad[k] = pcb_net_pads(&B, k, &B.nnetpad[k]);
	}
	if (verbose) {
		fprintf(stderr, "%s: %zu connections made anew\n", B.file,
			nfresh);
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
	const char *usage = "usage: pcbroute [-v] [-g GROUP] [-L LIB]... "
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
