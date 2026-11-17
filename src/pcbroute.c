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
// copper is clear of the pads as they now stand and keeps the edge rule;
// any other is made anew, among the copper kept, which nothing moves.
// So an edit makes anew only the connections it touches.  Routing is
// best effort: the same source and lock need not give the same copper.
// When a net finds no way, its pads and where it got stuck are named.
//
// The statements:
//   width NET W            its tracks W mm wide (rule track by default)
//   width PIN PIN W        the copper between two pins of one net
//   path NET PIN PIN...    its pins wired in this order, one to the next
//   fork NET PIN to PIN PIN  the two last joined, the first to the
//                          middle of their copper
//   via PIN WAY            the via of a pin of a plane net goes that way
// A net of a plane layer gets a via by each of its pads, on a neck
// track, the nearest place clear of all else; the via joins the pad to
// the plane.  Holes join every layer.  A net with pins on parts not
// placed yet (unplaced GROUP) waits whole: none of it is routed, and its
// copper leaves the lock; a plane net still gets its vias.  Every other
// net is routed on the top layer as a tree of connections from pad
// center to pad center: the pins of its forks first (the two last pins
// joined, then the first pin to the middle of that copper, a point met
// any way), then those of its paths, then the pins of its widths, then
// the nearest pad not yet joined to the nearest joined, each by the
// cheapest path over a grid of 0.05 mm in eight ways laid through the
// center of its first pad, by A* (Dijkstra's method led by the octile
// distance), which leaves and enters each pad along its longer side and
// turns only outside it, keeping its clearance from all copper of other
// nets, a run between two cells as well as the cells, and from the
// edge; a bend costs a little more than its length.  So pads in line are
// joined by one straight track through their centers.  A long pad
// (twice as long as wide or more) is joined at any point of the line
// between its ends (pcb_pad_ends), from its side or its ends, the
// shortest way: straight in from its side where the other pad is beside
// it; never at a point where the track would break the edge rule.  The
// search keeps to one fixed window of 256 by 256 cells (12.8 mm) laid
// about the middle of what it joins; a connection whose ends do not fit
// is routed in sections along the line between its pads, each in the
// window, each joint slid sideways off copper if need be.  The nets
// are routed with the fewest and nearest pads first; a net that finds no
// way is put first and all that was made anew is made again.
// With -v each connection and via is told on stderr.
//
// A run routes every group of a block statement alone, the deepest
// first, then the board, each section of the lock in turn (the lock from
// LOCK, or stdin; a file not there is nothing locked yet).  With -g
// GROUP the group alone is routed, as pcbplace -g laid it out:
// its nets whose pads are all of it (a net to a port waits for the
// layout round it), and the vias of its pads, with no outline.  The
// copper of a block (block GROUP) is its section's, moved with its
// parts, and kept as it is, but for a connection that breaks the edge
// rule on the board (a group alone has no outline): that is made anew.
// A copy (block GROUP as SRC) is not routed: its sections are SRC's,
// renamed part for part and net for net (pcb_passes).

#define _POSIX_C_SOURCE 200809L // For fmemopen.

#include "pcbsrc.h"
#include <math.h>
#include <string.h>

#define CELL 0.05 // The grid of the router.
#define BEND 0.5 // The cost of a turn of 45 degrees, in cells.
#define WIN 256 // The side of the window of the router, in cells: 12.8 mm.
#define MARGIN 24 // Cells between a section's ends and the window's sides.
#define SLIDE 30 // How far a section's end slides sideways, in cells.
#define NS (WIN * WIN * 8) // The states: a cell and the way it is entered.
#define TRIES 20
// What a cell keeps from copper beyond the clearance: a run between two
// cells passes a corner nearer than either by up to half a diagonal
// step, and the lock rounds each point to 0.1 um.
#define SNAP 2e-4
#define PI 3.14159265358979323846

static struct pcb B;
static int verbose;
static int **padnet; // The net of each pad of each part, worked out once.
// The window: cell (i, j) is at ((X0 + i) CELL + OX, (Y0 + j) CELL + OY),
// the grid laid through the center of the pad a connection leaves, the
// window about the middle of the section routed; it is allocated once,
// and every search stays inside it.
static int X0, Y0;
static double OX, OY;
// Cells the net now routed may not take: 1 by copper, which its own pads
// at the ends may override, 2 by the edge, which nothing overrides.
static unsigned char bad[WIN * WIN];
static unsigned char inp[WIN * WIN], inq[WIN * WIN]; // In the pads at the ends.
// The cells a connection may leave its first end at and end at its
// second: a point, or along a long pad.
static unsigned char ga[WIN * WIN], gb[WIN * WIN];
// Of each state: its cost, the way the state before it was entered, its
// place in the heap (-1 once taken off), valid while its stamp is the
// search's.
static float cost[NS];
static unsigned char back[NS];
static int place[NS], heap[NS], nheap;
static unsigned char stamp[NS], search; // Set anew when search wraps.
static int tb[4]; // The box of the target's cells, for the estimate.
static char stuck[2600]; // Where the last connection found no way.
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

// Whether net n waits whole for parts not placed: of no plane, whose
// vias each join it on their own.
static int net_waits(size_t n)
{
	return B.net[n].waits && B.net[n].plane < 0;
}

// Whether net n has a pad here.
static int net_here(size_t n)
{
	for (size_t j = 0; j < B.net[n].nend; j++) {
		if (!scope ||
		    of_group(B.part[B.net[n].end[j].part].name, scope)) {
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
	if (!scope && pcb_edge_dist(&B, 0, (const double[4]){ x, y, x, y }, r,
				    NULL) < B.edge - 1e-9) {
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

	for (int j = j0 < 0 ? 0 : j0; j <= j1 && j < WIN; j++) {
		for (int i = i0 < 0 ? 0 : i0; i <= i1 && i < WIN; i++) {
			double x = cx(i), y = cy(j), dx, dy;

			if (round) {
				if (hypot(x - mx, y - my) < r + d) {
					bad[j * WIN + i] = 1;
				}
				continue;
			}
			dx = fmax(fmax(x0 - x, x - x1), 0);
			dy = fmax(fmax(y0 - y, y - y1), 0);
			if (hypot(dx, dy) < d) {
				bad[j * WIN + i] = 1;
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

	for (int j = j0 < 0 ? 0 : j0; j <= j1 && j < WIN; j++) {
		for (int i = i0 < 0 ? 0 : i0; i <= i1 && i < WIN; i++) {
			if (pcb_seg_dist(cx(i), cy(j), ax, ay, bx, by) < d) {
				bad[j * WIN + i] = 1;
			}
		}
	}
}

// Sets up the window for net n of width w about (mx, my), its grid
// through (ox, oy): the cells its track may not take, each obstacle in
// the window marked in the cells round it.
static void window(int n, double w, double mx, double my, double ox, double oy)
{
	double d = w / 2 + B.clearance, r = WIN * CELL / 2 + d + 1;
	double h = CELL * 0.70710678; // Half a diagonal step.

	OX = fmod(ox, CELL);
	OY = fmod(oy, CELL);
	X0 = (int)lround((mx - OX) / CELL) - WIN / 2;
	Y0 = (int)lround((my - OY) / CELL) - WIN / 2;
	memset(bad, 0, sizeof bad);
	for (size_t q = 0; q < B.npart; q++) {
		const struct pcb_part *p = &B.part[q];

		for (size_t k = 0; p->placed && p->fp && k < p->fp->npad; k++) {
			struct pcb_spot sp;

			pcb_spot(&B, p, k, &sp);
			if ((padnet[q][k] == n && n >= 0) || sp.layer > 0 ||
			    fabs(sp.x - mx) > r + sp.w / 2 ||
			    fabs(sp.y - my) > r + sp.h / 2) {
				continue;
			}
			mark_box(sp.x - sp.w / 2, sp.y - sp.h / 2,
				 sp.x + sp.w / 2, sp.y + sp.h / 2,
				 sp.shape == PAD_CIRCLE, hypot(d, h) + SNAP);
		}
	}
	for (size_t i = 0; i < B.ntrack; i++) {
		const struct pcb_track *t = &B.track_[i];

		if (t->net == n || t->layer != 0) {
			continue;
		}
		for (size_t j = 1; j < t->n; j++) {
			const double *e = &t->xy[2 * j - 2];

			if (fmin(e[0], e[2]) > mx + r ||
			    fmax(e[0], e[2]) < mx - r ||
			    fmin(e[1], e[3]) > my + r ||
			    fmax(e[1], e[3]) < my - r) {
				continue;
			}
			mark_seg(e[0], e[1], e[2], e[3],
				 hypot(d + t->w / 2, h) + SNAP);
		}
	}
	for (size_t i = 0; i < B.nvia; i++) {
		const struct pcb_via *v = &B.via[i];

		if (v->net != n && fabs(v->x - mx) < r + v->dia &&
		    fabs(v->y - my) < r + v->dia) {
			mark_box(v->x - v->dia / 2, v->y - v->dia / 2,
				 v->x + v->dia / 2, v->y + v->dia / 2, 1,
				 hypot(d, h) + SNAP);
		}
	}
	// The edge, cell by cell, where the window comes near it.
	if (!scope &&
	    pcb_edge_dist(&B, 1,
			  (const double[4]){ cx(0), cy(0), cx(WIN - 1),
					     cy(WIN - 1) },
			  0, NULL) > B.edge + w) {
		return;
	}
	for (int j = 0; !scope && j < WIN; j++) {
		for (int i = 0; i < WIN; i++) {
			double x = cx(i), y = cy(j);

			if (pcb_edge_dist(&B, 0,
					  (const double[4]){ x, y, x, y },
					  w / 2, NULL) < B.edge) {
				bad[j * WIN + i] |= 2;
			}
		}
	}
}

// The ways a track may leave or enter a pad by, as a mask of the eight
// ways: along its longer side, right and left or up and down; any of
// the four for a pad round or about as long as it is wide (within a
// tenth), and for a long pad, met along its side as well as at its ends.
static unsigned axis(const struct pcb_spot *s)
{
	double e[2][2];

	if (s->shape == PAD_CIRCLE ||
	    fabs(s->w - s->h) < 0.1 * fmax(s->w, s->h) ||
	    pcb_pad_ends(s, e) == 2) {
		return 0x55;
	}
	return s->w > s->h ? 0x11 : 0x44;
}

// The least the path from cell c to the target can cost: the octile
// distance to the box of its cells (A*).
static double estimate(int c)
{
	int i = c % WIN, j = c / WIN;
	int dx = tb[0] > i ? tb[0] - i : i > tb[2] ? i - tb[2] : 0;
	int dy = tb[1] > j ? tb[1] - j : j > tb[3] ? j - tb[3] : 0;
	int lo = dx < dy ? dx : dy, hi = dx < dy ? dy : dx;

	return hi - lo + lo * 1.41421356;
}

// The order of the heap: the cost and the estimate.
static double key(int s)
{
	return cost[s] + estimate(s / 8);
}

static void swap_at(int i, int j)
{
	int t = heap[i];

	heap[i] = heap[j];
	heap[j] = t;
	place[heap[i]] = i;
	place[heap[j]] = j;
}

static void sift_up(int i)
{
	while (i > 0 && key(heap[(i - 1) / 2]) > key(heap[i])) {
		swap_at(i, (i - 1) / 2);
		i = (i - 1) / 2;
	}
}

// Takes the state of least key off the heap.
static int pop(void)
{
	int top = heap[0], i = 0;

	place[top] = -1;
	if (--nheap > 0) {
		heap[0] = heap[nheap];
		place[heap[0]] = 0;
	}
	for (;;) {
		int l = 2 * i + 1, r = l + 1, m = i;

		if (l < nheap && key(heap[l]) < key(heap[m])) {
			m = l;
		}
		if (r < nheap && key(heap[r]) < key(heap[m])) {
			m = r;
		}
		if (m == i) {
			return top;
		}
		swap_at(i, m);
		i = m;
	}
}

// Reaches state s at cost c from a state entered the way d: new, or
// cheaper than it was and not yet taken off the heap.
static void reach(int s, double c, int d)
{
	if (stamp[s] != search) {
		stamp[s] = search;
		cost[s] = (float)c;
		back[s] = (unsigned char)d;
		place[s] = nheap;
		heap[nheap++] = s;
		sift_up(place[s]);
	} else if (place[s] >= 0 && c < cost[s]) {
		cost[s] = (float)c;
		back[s] = (unsigned char)d;
		sift_up(place[s]);
	}
}

// The cell nearest (x, y), or -1 off the window.
static int cell_at(double x, double y)
{
	long i = lround((x - OX) / CELL) - X0, j = lround((y - OY) / CELL) - Y0;

	return i < 0 || j < 0 || i >= WIN || j >= WIN ? -1 : (int)(j * WIN + i);
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

// Whether a track of width w ending at p keeps the edge rule.
static int off_edge(const double p[2], double w)
{
	return scope ||
	       pcb_edge_dist(&B, 0, (const double[4]){ p[0], p[1], p[0], p[1] },
			     w / 2, NULL) >= B.edge;
}

// The point of the line between the ends of long pad s (pcb_pad_ends)
// nearest (x, y), to p.
static void on_line(const struct pcb_spot *s, double x, double y, double p[2])
{
	double e[2][2], dx, dy, l, t;

	pcb_pad_ends(s, e);
	dx = e[1][0] - e[0][0];
	dy = e[1][1] - e[0][1];
	l = dx * dx + dy * dy;
	t = ((x - e[0][0]) * dx + (y - e[0][1]) * dy) / l;
	t = t < 0 ? 0 : t > 1 ? 1 : t;
	p[0] = e[0][0] + t * dx;
	p[1] = e[0][1] + t * dy;
}

// The point of pad s a track of width w joins it at, toward (x, y): its
// center, or of a long pad the point of the line between its ends
// nearest (x, y), so a pad met from its side is joined there; but of
// the points of that line where the track keeps the edge rule, the
// nearest to that one, if any does.
static void pad_point(const struct pcb_spot *s, double x, double y, double w,
		      double p[2])
{
	double e[2][2], q[2], l;

	if (pcb_pad_ends(s, e) < 2) {
		memcpy(p, e[0], 2 * sizeof *p);
		return;
	}
	on_line(s, x, y, p);
	l = hypot(e[1][0] - e[0][0], e[1][1] - e[0][1]);
	for (int k = 0; k <= 2 * (int)ceil(l / CELL); k++) {
		double t = (k % 2 ? 1 : -1) * (double)((k + 1) / 2) * CELL / l;

		on_line(s, p[0] + (e[1][0] - e[0][0]) * t,
			p[1] + (e[1][1] - e[0][1]) * t, q);
		if (off_edge(q, w)) {
			memcpy(p, q, sizeof q);
			return;
		}
	}
}

// Marks the cells of the window in pad s (NULL: none).
static void mark_pad(unsigned char *in, const struct pcb_spot *s)
{
	int i0, i1, j0, j1;

	if (!s) {
		return;
	}
	i0 = down_int((s->x - s->w / 2 - OX) / CELL) - X0;
	i1 = up_int((s->x + s->w / 2 - OX) / CELL) - X0;
	j0 = down_int((s->y - s->h / 2 - OY) / CELL) - Y0;
	j1 = up_int((s->y + s->h / 2 - OY) / CELL) - Y0;
	for (int j = j0 < 0 ? 0 : j0; j <= j1 && j < WIN; j++) {
		for (int i = i0 < 0 ? 0 : i0; i <= i1 && i < WIN; i++) {
			in[j * WIN + i] =
				(unsigned char)inside_spot(s, cx(i), cy(j), 0);
		}
	}
}

// Marks in g the cells of the window a track of width w may end at on
// pad s (NULL: none) for point p: 2 the cells along a long pad whose
// nearest point of the line between its ends keeps the edge rule, then
// 1 the cell of p, if p keeps it.  The box of the cells grows box
// (NULL: none).  Returns the number of cells.
static int mark_ends(unsigned char *g, const struct pcb_spot *s,
		     const double p[2], double w, int *box)
{
	double e[2][2], l;
	int c, count = 0;

	if (s && pcb_pad_ends(s, e) == 2) {
		l = hypot(e[1][0] - e[0][0], e[1][1] - e[0][1]);
		for (double t = 0; t <= l + 1e-9; t += CELL / 2) {
			double q[2], u;

			c = cell_at(e[0][0] + (e[1][0] - e[0][0]) * t / l,
				    e[0][1] + (e[1][1] - e[0][1]) * t / l);
			if (c < 0 || g[c]) {
				continue;
			}
			// The cell's own point of the line, not past its ends.
			u = ((cx(c % WIN) - e[0][0]) * (e[1][0] - e[0][0]) +
			     (cy(c / WIN) - e[0][1]) * (e[1][1] - e[0][1])) /
			    l;
			on_line(s, cx(c % WIN), cy(c / WIN), q);
			if (u >= -1e-9 && u <= l + 1e-9 && off_edge(q, w)) {
				g[c] = 2;
			}
		}
	}
	c = cell_at(p[0], p[1]);
	if (c >= 0 && off_edge(p, w)) {
		g[c] = 1;
	}
	for (c = 0; c < WIN * WIN; c++) {
		if (!g[c]) {
			continue;
		}
		count++;
		if (box) {
			box[0] = c % WIN < box[0] ? c % WIN : box[0];
			box[1] = c / WIN < box[1] ? c / WIN : box[1];
			box[2] = c % WIN > box[2] ? c % WIN : box[2];
			box[3] = c / WIN > box[3] ? c / WIN : box[3];
		}
	}
	return count;
}

// The point a track ending in cell c of g (mark_ends) joins at: p, or
// the point of the line of long pad s nearest the cell.
static void end_at(const unsigned char *g, const struct pcb_spot *s,
		   const double p[2], int c, double q[2])
{
	if (g[c] == 2) {
		on_line(s, cx(c % WIN), cy(c / WIN), q);
	} else {
		memcpy(q, p, 2 * sizeof *q);
	}
}

// Routes a section of net n of width w from point a to point b, both in
// the window laid about their middle: out of a the ways of mask ma, into
// b the ways of mb, straight while in the pad of either (sa, sb, or
// NULL for a point between); a long pad is left or met at any point of
// the line between its ends.  Its corners, from a to b, go to *xy (their
// number to *n); returns 0 if there is no way, and where it got nearest
// goes to near.
static int section(int n, double w, const double *a, unsigned ma,
		   const struct pcb_spot *sa, const double *b, unsigned mb,
		   const struct pcb_spot *sb, double **xy, size_t *np,
		   double near[2])
{
	int ca, cb, goal = -1, best, start = -1;
	size_t m = 0;
	double qa[2], qb[2];

	window(n, w, (a[0] + b[0]) / 2, (a[1] + b[1]) / 2, a[0], a[1]);
	ca = cell_at(a[0], a[1]);
	cb = cell_at(b[0], b[1]);
	near[0] = a[0];
	near[1] = a[1];
	memset(inp, 0, sizeof inp);
	memset(inq, 0, sizeof inq);
	memset(ga, 0, sizeof ga);
	memset(gb, 0, sizeof gb);
	tb[0] = tb[1] = WIN;
	tb[2] = tb[3] = -1;
	// Nothing overrides the edge, not even at the ends.
	if (ca < 0 || cb < 0 || !mark_ends(ga, sa, a, w, NULL) ||
	    !mark_ends(gb, sb, b, w, tb)) {
		return 0;
	}
	mark_pad(inp, sa);
	mark_pad(inq, sb);
	if (++search == 0) {
		memset(stamp, 0, sizeof stamp);
		search = 1;
	}
	nheap = 0;
	best = ca;
	// In line: one straight track, if clear.
	if (ga[ca] == 1 && gb[cb] == 1 &&
	    (fabs(a[0] - b[0]) < 1e-6 || fabs(a[1] - b[1]) < 1e-6)) {
		int di = (cb % WIN > ca % WIN) - (cb % WIN < ca % WIN);
		int dj = (cb / WIN > ca / WIN) - (cb / WIN < ca / WIN), d;
		int clear = 1;

		for (int c = ca; clear && c != cb; c += dj * WIN + di) {
			clear = !(bad[c] & 2) && (!bad[c] || inp[c] || inq[c]);
		}
		for (d = 0; d < 8 && (DI[d] != di || DJ[d] != dj); d++) {
		}
		if (clear && d < 8) {
			*xy = pcb_alloc(4 * sizeof **xy);
			memcpy(*xy, a, 2 * sizeof **xy);
			memcpy(*xy + 2, b, 2 * sizeof **xy);
			*np = 2;
			return 1;
		}
	}
	for (int c = 0; c < WIN * WIN; c++) {
		for (int d = 0; ga[c] && d < 8; d++) {
			inp[c] = 1;
			if (ma >> d & 1) {
				reach(c * 8 + d, 0, 8);
			}
		}
	}
	while (nheap) {
		int s = pop(), c = s / 8, d = s % 8, i = c % WIN, j = c / WIN;

		if (gb[c] && cost[s] > 0) {
			goal = s;
			break;
		}
		if (estimate(c) < estimate(best)) {
			best = c;
		}
		for (int nd = 0; nd < 8; nd++) {
			int ni = i + DI[nd], nj = j + DJ[nd], nc, turn;

			// Straight while in a pad: out of the first along its
			// side, into the second along its side.
			if ((inp[c] || inq[c]) && nd != d) {
				continue;
			}
			if (ni < 0 || nj < 0 || ni >= WIN || nj >= WIN) {
				continue;
			}
			nc = nj * WIN + ni;
			if ((inq[nc] || gb[nc]) && !(mb >> nd & 1)) {
				continue;
			}
			if (!gb[nc] && ((bad[nc] & 2) ||
					(bad[nc] && !inq[nc] && !inp[nc]))) {
				continue;
			}
			turn = abs(nd - d);
			turn = turn > 4 ? 8 - turn : turn;
			if (turn > 2) {
				continue; // No sharp turns.
			}
			reach(nc * 8 + nd,
			      cost[s] + (nd % 2 ? 1.41421356 : 1) + turn * BEND,
			      d);
		}
	}
	if (goal < 0) {
		near[0] = cx(best % WIN);
		near[1] = cy(best / WIN);
		return 0;
	}
	// The corners, from b back to a, then turned round.
	*xy = pcb_alloc(2 * 16 * sizeof **xy);
	for (int st = goal, last = -1, cap = 16;;) {
		int d = st % 8, c = st / 8, pd = back[st];

		if (m + 1 >= (size_t)cap) {
			cap *= 2;
			*xy = realloc(*xy, 2 * (size_t)cap * sizeof **xy);
			if (!*xy) {
				pcb_die("out of memory");
			}
		}
		if (last < 0 || d != last || pd == 8) {
			(*xy)[2 * m] = cx(c % WIN);
			(*xy)[2 * m + 1] = cy(c / WIN);
			m++;
		}
		if (pd == 8) {
			start = c;
			break;
		}
		last = d;
		st = (c - DJ[d] * WIN - DI[d]) * 8 + pd;
	}
	end_at(gb, sb, b, goal / 8, qb);
	end_at(ga, sa, a, start, qa);
	// The corner before b on b's line; the ends exact.
	if (m >= 3 && goal % 2 == 0) {
		if (goal % 8 == 0 || goal % 8 == 4) {
			(*xy)[3] = qb[1];
		} else {
			(*xy)[2] = qb[0];
		}
	}
	(*xy)[0] = qb[0];
	(*xy)[1] = qb[1];
	(*xy)[2 * m - 2] = qa[0];
	(*xy)[2 * m - 1] = qa[1];
	for (size_t i = 0; i < m / 2; i++) {
		for (int k = 0; k < 2; k++) {
			double t = (*xy)[2 * i + (size_t)k];

			(*xy)[2 * i + (size_t)k] =
				(*xy)[2 * (m - 1 - i) + (size_t)k];
			(*xy)[2 * (m - 1 - i) + (size_t)k] = t;
		}
	}
	*np = m;
	return 1;
}

// A point between the sections of a connection of net n, width w: on
// the grid of the first pad, the straight line's point p, or slid
// sideways (u the way across) off copper and out of cutouts, as near as
// it is clear; returns 0 if none is.
static int midpoint(int n, double w, const double *p, const double *u,
		    double q[2])
{
	const char *who;

	for (int k = 0; k <= 2 * SLIDE; k++) {
		double t = (k % 2 ? 1 : -1) * (double)((k + 1) / 2) * CELL;

		q[0] = OX +
		       (double)lround((p[0] + u[0] * t - OX) / CELL) * CELL;
		q[1] = OY +
		       (double)lround((p[1] + u[1] * t - OY) / CELL) * CELL;
		if (free_at(q[0], q[1], w / 2, 0, n, &who)) {
			return 1;
		}
	}
	return 0;
}

// Joins pad a of net n to pad b: a connection of width w from the center
// of a to the center of b (of a long pad, from any point of the line
// between its ends, the nearest the other pad first), leaving and
// entering each along its longer side (a long pad any of four ways), in
// sections along the straight line between them, each within the
// window; or, with at, from the point at on copper of the net, left any
// way, to a, on a grid laid through at, the connection named by a and b
// all the same.  Returns 0 if there is no way, and stuck says where.
static int connect(int n, const struct pcb_end *a, const struct pcb_end *b,
		   const double *at, double w)
{
	struct pcb_spot sa, sb, *s0 = &sa, *s1 = &sb;
	double pa[2], pb[2], len, u[2], *all = NULL, near[2] = { 0, 0 };
	double span = (WIN - 2 * MARGIN - 2 * SLIDE) * CELL;
	size_t nall = 0;
	int k, ok = 1, l;
	char *ka, *kb;

	pcb_spot(&B, &B.part[a->part], a->pad, &sa);
	pcb_spot(&B, &B.part[b->part], b->pad, &sb);
	pad_point(&sa, sb.x, sb.y, w, pa);
	pad_point(&sb, pa[0], pa[1], w, pb);
	pad_point(&sa, pb[0], pb[1], w, pa);
	if (at) {
		// From the point to the pad, so the point is on the grid.
		pad_point(&sa, at[0], at[1], w, pb);
		memcpy(pa, at, sizeof pa);
		s0 = NULL;
		s1 = &sa;
	}
	len = hypot(pb[0] - pa[0], pb[1] - pa[1]);
	u[0] = len > 0 ? -(pb[1] - pa[1]) / len : 0;
	u[1] = len > 0 ? (pb[0] - pa[0]) / len : 1;
	k = (int)ceil(fmax(fabs(pb[0] - pa[0]), fabs(pb[1] - pa[1])) / span);
	k = k < 1 ? 1 : k;
	OX = fmod(pa[0], CELL);
	OY = fmod(pa[1], CELL);
	for (int i = 0; ok && i < k; i++) {
		double from[2], to[2], *xy = NULL;
		size_t m = 0;

		if (!i) {
			memcpy(from, pa, sizeof from);
		} else {
			memcpy(from, all + 2 * nall - 2, sizeof from);
		}
		if (i == k - 1) {
			memcpy(to, pb, sizeof to);
		} else {
			const double p[2] = {
				pa[0] + (pb[0] - pa[0]) * (i + 1) / k,
				pa[1] + (pb[1] - pa[1]) * (i + 1) / k
			};

			if (!midpoint(n, w, p, u, to)) {
				near[0] = p[0];
				near[1] = p[1];
				ok = 0;
				break;
			}
		}
		ok = section(n, w, from, i || !s0 ? 0xFFu : axis(s0),
			     i ? NULL : s0, to, i == k - 1 ? axis(s1) : 0xFFu,
			     i == k - 1 ? s1 : NULL, &xy, &m, near);
		// The sections end to end, each joint once.
		for (size_t j = nall ? 1 : 0; ok && j < m; j++) {
			all = pcb_grow(all, 2 * nall + 1, sizeof *all);
			all[2 * nall] = xy[2 * j];
			all[2 * nall + 1] = xy[2 * j + 1];
			nall++;
		}
		free(xy);
	}
	if (!ok) {
		snprintf(stuck, sizeof stuck, "%s to %s, stuck at %.3f %.3f",
			 pad_label(a), pad_label(b), near[0], near[1]);
		free(all);
		return 0;
	}
	ka = pcb_pad_key(&B, a->part, a->pad);
	kb = pcb_pad_key(&B, b->part, b->pad);
	l = pcb_add_link(&B, n, ka, kb, w);
	free(ka);
	free(kb);
	{
		struct pcb_track t;

		t.net = n;
		t.layer = 0;
		t.link = l;
		t.w = w;
		t.n = nall;
		t.xy = all;
		B.track_ = pcb_grow(B.track_, B.ntrack, sizeof *B.track_);
		B.track_[B.ntrack++] = t;
	}
	if (verbose) {
		fprintf(stderr, "%s: %s to %s%s, %.3f wide, %d sections\n",
			B.net[n].name, pad_label(a), at ? "the middle of " : "",
			pad_label(b), w, k);
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
	ok = connect(n, &ends[i], &ends[j], NULL, w);
	if (ok) {
		groups(n, ends, m, g);
	}
	return ok;
}

// Whether statement st is a fork of net n: fork NET PIN to PIN PIN.
static int is_fork(const struct pcb_stmt *st, int n)
{
	return !strcmp(st->w[0], "fork") && st->n == 6 &&
	       !strcmp(st->w[3], "to") && !strcmp(st->w[1], B.net[n].name);
}

// The connection of net n between the pads of keys ka and kb, named
// either way, or -1.
static long link_of(int n, const char *ka, const char *kb)
{
	for (size_t l = 0; l < B.nlink; l++) {
		const struct pcb_link *k = &B.link[l];

		if (k->net == n && ((!strcmp(k->a, ka) && !strcmp(k->b, kb)) ||
				    (!strcmp(k->a, kb) && !strcmp(k->b, ka)))) {
			return (long)l;
		}
	}
	return -1;
}

// The key of the pad of a pin (PART.PIN), or NULL.
static char *pin_key(const char *pin)
{
	size_t q, p;

	return pcb_pin(&B, pin, &q, &p) ? pcb_pad_key(&B, q, p) : NULL;
}

// The middle of the copper of connection l, by its length.
static void middle(long l, double p[2])
{
	const struct pcb_track *t = NULL;
	double len = 0, f;

	for (size_t i = 0; !t && i < B.ntrack; i++) {
		t = B.track_[i].link == l && B.track_[i].n ? &B.track_[i] :
							     NULL;
	}
	if (!t) {
		p[0] = p[1] = 0;
		return;
	}
	for (size_t j = 1; j < t->n; j++) {
		len += hypot(t->xy[2 * j] - t->xy[2 * j - 2],
			     t->xy[2 * j + 1] - t->xy[2 * j - 1]);
	}
	f = len / 2;
	p[0] = t->xy[2 * t->n - 2];
	p[1] = t->xy[2 * t->n - 1];
	for (size_t j = 1; j < t->n; j++) {
		const double *e = &t->xy[2 * j - 2];
		double d = hypot(e[2] - e[0], e[3] - e[1]);

		if (f <= d) {
			p[0] = d > 0 ? e[0] + (e[2] - e[0]) * f / d : e[0];
			p[1] = d > 0 ? e[1] + (e[3] - e[1]) * f / d : e[1];
			return;
		}
		f -= d;
	}
}

// Carries out statement st, a fork of net n: joins its two last pins by
// one connection, then its first pin to the middle of that copper, a
// connection named by the first pin and the second.
static int fork_net(int n, const struct pcb_stmt *st,
		    const struct pcb_end *ends, size_t m, size_t *g)
{
	size_t p = end_of(st->w[2], ends, m), a = end_of(st->w[4], ends, m);
	size_t b = end_of(st->w[5], ends, m);
	char *ka, *kb, *kp;
	double mid[2];
	long l;
	int ok;

	if (p == m || a == m || b == m || p == a || p == b || a == b) {
		pcb_error(&B, st->line,
			  "fork NET PIN to PIN PIN, three pins of the net "
			  "on the top");
		return 1;
	}
	if (!join(n, ends, m, g, a, b)) {
		return 0;
	}
	ka = pin_key(st->w[4]);
	kb = pin_key(st->w[5]);
	kp = pin_key(st->w[2]);
	l = link_of(n, ka, kb);
	ok = 1;
	if (l < 0) {
		pcb_error(&B, st->line, "fork: %s and %s joined otherwise",
			  st->w[4], st->w[5]);
	} else if (g[p] != g[a]) {
		middle(l, mid);
		ok = connect(n, &ends[p], &ends[a], mid,
			     pcb_link_width(&B, n, kp, ka));
		if (ok) {
			groups(n, ends, m, g);
		}
	}
	free(ka);
	free(kb);
	free(kp);
	return ok;
}

// Routes signal net n: joins its groups of pads by connections, those
// its forks name first, then those of its paths and widths, then the
// nearest pad not joined to the nearest joined.  Returns 0 if a pad finds no way, and *who says
// which, from which, and where it got stuck.
static int route_net(int n, const char **who)
{
	struct pcb_end *ends;
	size_t m = top_ends(n, &ends), *g = pcb_alloc((m + 1) * sizeof *g);
	int ok = 1;

	groups(n, ends, m, g);
	for (size_t i = 0; ok && i < B.nstmt; i++) {
		if (is_fork(&B.stmt[i], n)) {
			ok = fork_net(n, &B.stmt[i], ends, m, g);
			*who = ok ? *who : stuck;
		}
	}
	for (size_t i = 0; ok && i < B.nstmt; i++) {
		const struct pcb_stmt *st = &B.stmt[i];

		if (!strcmp(st->w[0], "path") && st->n >= 4 &&
		    !strcmp(st->w[1], B.net[n].name)) {
			for (int k = 3; ok && k < st->n; k++) {
				size_t a = end_of(st->w[k - 1], ends, m);
				size_t b = end_of(st->w[k], ends, m);

				if (a == m || b == m) {
					pcb_error(&B, st->line,
						  "no pin %s on the top",
						  st->w[a == m ? k - 1 : k]);
					continue;
				}
				ok = join(n, ends, m, g, a, b);
				*who = ok ? *who : stuck;
			}
		} else if (!strcmp(st->w[0], "width") && st->n == 4) {
			size_t a = end_of(st->w[1], ends, m);
			size_t b = end_of(st->w[2], ends, m);

			if (a < m && b < m) {
				ok = join(n, ends, m, g, a, b);
				*who = ok ? *who : stuck;
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
			*who = stuck;
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
// every opening of the mask, on either side, for the via is tented,
// and stands off the keep-out of every hole (pcbfp(5)).
static int via_room(int n, double x, double y, double r, const char **who)
{
	for (size_t q = 0; q < B.npart; q++) {
		const struct pcb_part *p = &B.part[q];
		char why[1024];

		if (pcb_keeps(&B, p, x, y, r, why, sizeof why)) {
			*who = part_label(p);
			return 0;
		}

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
			for (int side = 0; pcb_soldered(p, k) && side < 2;
			     side++) {
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
				neck(n, pcb_add_link(&B, n, key, "plane", w), q,
				     &s, w);
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
				free(key);
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
							w / 2, s.layer, n,
							&who);
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
			struct pcb_via v = { n,	 l,	    bx,
					     by, B.via_dia, B.via_drill };

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
				double x =
					t->xy[2 * j - 2] +
					(t->xy[2 * j] - t->xy[2 * j - 2]) * u;
				double y =
					t->xy[2 * j - 1] +
					(t->xy[2 * j + 1] - t->xy[2 * j - 1]) *
						u;

				if (!pads_clear(x, y, t->w / 2 - 1e-6, t->layer,
						n, &who)) {
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

// Whether the copper of connection l keeps the edge rule from the
// outline and the cutouts, as pcbcheck(1) measures it; a group laid out
// alone has no outline.
static int link_off_edge(long l)
{
	for (size_t i = 0; !scope && i < B.ntrack; i++) {
		const struct pcb_track *t = &B.track_[i];

		for (size_t j = 1; t->link == l && j < t->n; j++) {
			if (pcb_edge_dist(&B, 0, &t->xy[2 * j - 2], t->w / 2,
					  NULL) < B.edge - 1e-6) {
				return 0;
			}
		}
	}
	for (size_t i = 0; !scope && i < B.nvia; i++) {
		const struct pcb_via *v = &B.via[i];

		if (v->link == l &&
		    pcb_edge_dist(&B, 0,
				  (const double[4]){ v->x, v->y, v->x, v->y },
				  v->dia / 2, NULL) < B.edge - 1e-6) {
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
			pcb_error(
				&B, st->line,
				"block %s: no section of it in the lock; lay it "
				"out first",
				st->w[1]);
			continue;
		}
		pcb_read(&L, B.file, lib, nlib, f, NULL);
		fclose(f);
		for (size_t q = 0; q < B.npart && r < 0; q++) {
			if (of_group(B.part[q].name, st->w[1]) &&
			    B.part[q].placed && L.part[q].placed) {
				r = (long)q;
			}
		}
		if (r < 0) {
			pcb_free(&L);
			continue;
		}
		tx = round((B.part[r].x - L.part[r].x) * 1e4) / 1e4;
		ty = round((B.part[r].y - L.part[r].y) * 1e4) / 1e4;
		// What the lock had of the block's copper goes.
		for (size_t l = 0; l < first; l++) {
			long qa = B.link[l].net >= 0 ?
					  part_of_key(B.link[l].net,
						      B.link[l].a) :
					  -1;

			if (qa >= 0 && of_group(B.part[qa].name, st->w[1])) {
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
					t.xy[2 * j] =
						L.track_[i].xy[2 * j] + tx;
					t.xy[2 * j + 1] =
						L.track_[i].xy[2 * j + 1] + ty;
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
			fprintf(stderr,
				"%s: the copper of block %s, moved %.4f %.4f\n",
				B.file, st->w[1], tx, ty);
		}
		pcb_free(&L);
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
	int *fresh, *order, status;

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
		// A block's copper is kept as its lock has it, but for what
		// breaks the edge rule, which a group alone does not know; a
		// net that waits keeps none.
		kept[l] = !net_waits((size_t)k) &&
			  (l >= ours ||
			   (!fresh[k] && now && keep_link((long)l, now, np))) &&
			  link_off_edge((long)l);
		for (size_t j = 0; j < np; j++) {
			free(now[j]);
		}
		free(now);
	}
	// A fork's first connection, to the middle of its second, is kept
	// only with it.
	for (size_t k = 0; k < B.nnet; k++) {
		for (size_t s = 0; s < B.nstmt; s++) {
			const struct pcb_stmt *st = &B.stmt[s];
			char *kp, *ka, *kb;
			long stem, trunk;

			if (!is_fork(st, (int)k)) {
				continue;
			}
			kp = pin_key(st->w[2]);
			ka = pin_key(st->w[4]);
			kb = pin_key(st->w[5]);
			stem = kp && ka ? link_of((int)k, kp, ka) : -1;
			trunk = ka && kb ? link_of((int)k, ka, kb) : -1;
			if (stem >= 0 && (trunk < 0 || !kept[trunk])) {
				kept[stem] = 0;
			}
			free(kp);
			free(ka);
			free(kb);
		}
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

			if (B.net[k].plane >= 0 || net_waits((size_t)k) ||
			    !net_in_scope((size_t)k)) {
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
			pcb_error(&B, 0, "net %s: no way from %s",
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
		if (!net_here(k)) {
			free(words[k]);
		}
		B.netpad[k] = pcb_net_pads(&B, k, &B.nnetpad[k]);
	}
	if (verbose) {
		fprintf(stderr, "%s: %zu connections made anew\n", B.file,
			nfresh);
	}
	pcb_write_lock(&B, out);
	status = B.failed;
	for (size_t q = 0; q < B.npart; q++) {
		free(padnet[q]);
	}
	free(padnet);
	padnet = NULL;
	free(fresh);
	free(order);
	free(words);
	free(kept);
	kept = NULL;
	pcb_free(&B);
	return status;
}

int main(int argc, char **argv)
{
	char **lib = NULL;
	size_t nlib = 0;
	int i = 1;
	const char *group = NULL;
	const char *usage = "usage: pcbroute [-v] [-g GROUP] [-L LIB]... "
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
