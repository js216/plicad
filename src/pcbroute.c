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
//   width NET W            its tracks W mm wide (its class's width, else
//                          rule track, by default)
//   width PIN PIN W        the copper between two pins of one net
//   class CLASS width W    the tracks of the nets of the class W mm wide
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
// distance), which leaves and enters each pad any of the eight ways and
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

#include "pcbsrc.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CELL 0.05 // The grid of the router.
#define BEND 0.5 // The cost of a turn of 45 degrees, in cells.
enum {
	WIN = 256, // The side of the window of the router, in cells: 12.8 mm.
	MARGIN = 24, // Cells between a section's ends and the window's sides.
	SLIDE = 30 // How far a section's end slides sideways, in cells.
};
enum {
	WAYS = 8, // The ways a cell is entered.
	FORK_WORDS = 6, // The words of fork NET PIN to PIN PIN.
	VIA_WAYS = 16, // The ways a via is looked for from its pad,
	VIA_HALF = VIA_WAYS / 2, // of a half turn.
	WHY_LEN = 1024, // Room for why a via is in a keep-out.
	TRIES = 20, // The most times the nets are routed in a new order.
	PATH_START = 16 // The first room for the corners of a section.
};
#define NS (WIN * WIN * WAYS) // The states: a cell and the way it is entered.
#define SAME 1e-6 // Points this near are one, in mm.
#define FAR 1e300 // Beyond any length.
#define STEP_SLACK 1e-9 // What a count of steps lets go of rounding.
#define BIG 1e9 // Beyond the board, in mm.
#define LOCK_UNITS 1e4 // The steps of the lock in a mm: 0.1 um.
#define NEAR_FAR 1e299 // A score below this is of a place found.
#define VIA_FROM 0.3 // How far from its pad a via is looked for first,
#define VIA_TO 3.0 // and last, in mm.
#define NECK_STEP 0.05 // The step along a neck checked clear.
#define OUT_SHARE 0.3 // What going out from the part weighs, in mm.
// What a cell keeps from copper beyond the clearance: a run between two
// cells passes a corner nearer than either by up to half a diagonal
// step, and the lock rounds each point to 0.1 um.
#define SNAP 2e-4
#define PI 3.14159265358979323846
#define HALF_DIAG 0.70710678 // Half the diagonal of a cell, in cells.
#define DIAG 1.41421356 // The diagonal of a cell, in cells.
#define TOUCH 1e-9 // Lengths closer than this are one, in mm.
enum {
	LABEL_LEN = 1100, // Room for a part's label,
	PAD_LABEL_LEN = 1200, // for a pad's,
	STUCK_LEN = 2600 // and for where a connection got stuck.
};

// What lasts over all passes: whether to tell what is done.
struct across {
	int verbose;
};

// A pass under way: what lasts over the passes; the board and the
// library of its footprints; the group routed alone, or NULL; the net of
// each pad of each part, worked out once; and the search.
struct router {
	struct across *all;
	struct pcb b;
	struct pcb_lib *lib;
	const char *scope;
	int **padnet;
	// The window: cell (i, j) is at ((x0 + i) CELL + ox, (y0 + j) CELL +
	// oy), the grid laid through the center of the pad a connection
	// leaves, the window about the middle of the section routed; every
	// search stays inside it.
	int x0, y0;
	double ox, oy;
	// Cells the net now routed may not take: 1 by copper, which its own
	// pads at the ends may override, 2 by the edge, which nothing
	// overrides.
	unsigned char bad[WIN * WIN];
	unsigned char inp[WIN * WIN]; // In the pads at the ends.
	unsigned char inq[WIN * WIN];
	// The cells a connection may leave its first end at and end at its
	// second: a point, or along a long pad.
	unsigned char ga[WIN * WIN];
	unsigned char gb[WIN * WIN];
	// Of each state: its cost, the way the state before it was entered,
	// its place in the heap (-1 once taken off), valid while its stamp
	// is the search's.
	float cost[NS];
	unsigned char back[NS];
	int place[NS];
	int heap[NS];
	int nheap;
	unsigned char stamp[NS];
	unsigned char search; // Set anew when search wraps.
	int tb[4]; // The box of the target's cells, for the estimate.
	char stuck[STUCK_LEN]; // Where the last connection found no way.
	int *kept; // Of each connection: read from the lock and kept.
	size_t nkept;
};

static int of_group(const char *name, const char *group)
{
	size_t n = strlen(group);

	return !strncmp(name, group, n) && name[n] == '.';
}

// Whether net n is routed here: all its pads of the group routed.
static int net_in_scope(struct router *rt, size_t n)
{
	for (size_t j = 0; rt->scope && j < rt->b.net[n].nend; j++) {
		if (!of_group(rt->b.part[rt->b.net[n].end[j].part].name,
			      rt->scope)) {
			return 0;
		}
	}
	return 1;
}

// Whether net n waits whole for parts not placed: of no plane, whose
// vias each join it on their own.
static int net_waits(struct router *rt, size_t n)
{
	return rt->b.net[n].waits && rt->b.net[n].plane < 0;
}

// Whether net n has a pad here.
static int net_here(struct router *rt, size_t n)
{
	for (size_t j = 0; j < rt->b.net[n].nend; j++) {
		if (!rt->scope ||
		    of_group(rt->b.part[rt->b.net[n].end[j].part].name,
			     rt->scope)) {
			return 1;
		}
	}
	return 0;
}

static const int way_di[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
static const int way_dj[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };

// The number of steps from 0 by step to len, len itself if it is one
// (to within STEP_SLACK of a step).
static int steps_upto(double len, double step)
{
	return (int)floor((len / step) + STEP_SLACK) + 1;
}

// The number of steps from 0 by step short of len.
static int steps_below(double len, double step)
{
	return (int)ceil((len / step) - STEP_SLACK);
}

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

static double cx(const struct router *rt, int i)
{
	return ((rt->x0 + i) * CELL) + rt->ox;
}

static double cy(const struct router *rt, int j)
{
	return ((rt->y0 + j) * CELL) + rt->oy;
}

// A part for a message: NAME (REF).
static const char *part_label(const struct pcb_part *q)
{
	static char buf[LABEL_LEN];

	(void)snprintf(buf, sizeof buf, "%s (%s)", q->name, pcb_ref(q));
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

// A disk of copper: its center and radius, its layer (-1 all) and its
// net.
struct disk_on {
	double x, y, r;
	int layer;
	int net;
};

// Whether disk d keeps its clearance from the pads not of its net, grown
// by c; sets *who to the part hit.
static int pads_clear_by(const struct router *rt, const struct disk_on *d,
			 double c, const char **who)
{
	for (size_t q = 0; q < rt->b.npart; q++) {
		const struct pcb_part *p = &rt->b.part[q];

		for (size_t k = 0; p->placed && p->fp && k < p->fp->npad; k++) {
			struct pcb_spot s;

			pcb_spot(&rt->b, p, k, &s);
			if ((rt->padnet[q][k] == d->net && d->net >= 0) ||
			    (s.layer >= 0 && d->layer >= 0 &&
			     s.layer != d->layer)) {
				continue;
			}
			if (inside_spot(&s, d->x, d->y, d->r + c)) {
				*who = part_label(p);
				return 0;
			}
		}
	}
	return 1;
}

// Whether disk d keeps its clearance from the pads not of its net; sets
// *who to the part hit.
static int pads_clear(const struct router *rt, const struct disk_on *d,
		      const char **who)
{
	return pads_clear_by(rt, d, rt->b.clearance, who);
}

// Whether disk d keeps its clearance from the tracks and vias not of its
// net; sets *who to the net hit.
static int copper_clear(const struct router *rt, const struct disk_on *d,
			const char **who)
{
	double c = rt->b.clearance;
	struct pcb_pt p = { d->x, d->y };

	for (size_t i = 0; i < rt->b.ntrack; i++) {
		const struct pcb_track *t = &rt->b.track_[i];

		if (t->net == d->net ||
		    (d->layer >= 0 && t->layer != d->layer)) {
			continue;
		}
		for (size_t j = 1; j < t->n; j++) {
			struct pcb_seg s = {
				{ t->xy[(2 * j) - 2], t->xy[(2 * j) - 1] },
				{ t->xy[2 * j], t->xy[(2 * j) + 1] }
			};

			if (pcb_seg_dist(p, s) < d->r + c + (t->w / 2)) {
				*who = rt->b.net[t->net].name;
				return 0;
			}
		}
	}
	for (size_t i = 0; i < rt->b.nvia; i++) {
		const struct pcb_via *v = &rt->b.via[i];

		if (v->net != d->net &&
		    hypot(d->x - v->x, d->y - v->y) < d->r + c + (v->dia / 2)) {
			*who = rt->b.net[v->net].name;
			return 0;
		}
	}
	return 1;
}

// Whether disk d keeps its clearance from all copper not of its net, and
// the edge rule; sets *who to what is hit.
static int free_at(const struct router *rt, const struct disk_on *d,
		   const char **who)
{
	const double at[4] = { d->x, d->y, d->x, d->y };

	if (!pads_clear(rt, d, who) || !copper_clear(rt, d, who)) {
		return 0;
	}
	if (!rt->scope &&
	    pcb_edge_dist(&rt->b, 0, at, d->r, NULL) < rt->b.edge - TOUCH) {
		*who = "the edge";
		return 0;
	}
	return 1;
}

// What to mark bad: the cells within d of a box (a rectangle or, with
// round, a disk in it).
struct mark {
	double box[4];
	int round;
	double d;
};

// Marks bad the cells of mk.
static void mark_box(struct router *rt, const struct mark *mk)
{
	const double *b = mk->box;
	double d = mk->d;
	int i0 = down_int((b[0] - d - rt->ox) / CELL) - rt->x0;
	int i1 = up_int((b[2] + d - rt->ox) / CELL) - rt->x0;
	int j0 = down_int((b[1] - d - rt->oy) / CELL) - rt->y0;
	int j1 = up_int((b[3] + d - rt->oy) / CELL) - rt->y0;
	double mx = (b[0] + b[2]) / 2;
	double my = (b[1] + b[3]) / 2;
	double r = (b[2] - b[0]) / 2;

	for (int j = j0 < 0 ? 0 : j0; j <= j1 && j < WIN; j++) {
		for (int i = i0 < 0 ? 0 : i0; i <= i1 && i < WIN; i++) {
			double x = cx(rt, i);
			double y = cy(rt, j);
			double dx = NAN;
			double dy = NAN;

			if (mk->round) {
				if (hypot(x - mx, y - my) < r + d) {
					rt->bad[(j * WIN) + i] = 1;
				}
				continue;
			}
			dx = fmax(fmax(b[0] - x, x - b[2]), 0);
			dy = fmax(fmax(b[1] - y, y - b[3]), 0);
			if (hypot(dx, dy) < d) {
				rt->bad[(j * WIN) + i] = 1;
			}
		}
	}
}

// Marks bad the cells within d of the segment s.
static void mark_seg(struct router *rt, struct pcb_seg s, double d)
{
	int i0 = down_int((fmin(s.a.x, s.b.x) - d - rt->ox) / CELL) - rt->x0;
	int i1 = up_int((fmax(s.a.x, s.b.x) + d - rt->ox) / CELL) - rt->x0;
	int j0 = down_int((fmin(s.a.y, s.b.y) - d - rt->oy) / CELL) - rt->y0;
	int j1 = up_int((fmax(s.a.y, s.b.y) + d - rt->oy) / CELL) - rt->y0;

	for (int j = j0 < 0 ? 0 : j0; j <= j1 && j < WIN; j++) {
		for (int i = i0 < 0 ? 0 : i0; i <= i1 && i < WIN; i++) {
			struct pcb_pt p = { cx(rt, i), cy(rt, j) };

			if (pcb_seg_dist(p, s) < d) {
				rt->bad[(j * WIN) + i] = 1;
			}
		}
	}
}

// A connection under way: its net and the width of its track.
struct conn {
	int net;
	double w;
};

// The window of a connection about the point mid: the reach of what is
// marked in it, the clearance round the track, and half a diagonal step.
struct win_of {
	struct pcb_pt mid;
	double r, d, h;
};

// Marks the pads not of the net of cn in the window wo.
static void window_pads(struct router *rt, const struct conn *cn,
			const struct win_of *wo)
{
	for (size_t q = 0; q < rt->b.npart; q++) {
		const struct pcb_part *p = &rt->b.part[q];

		for (size_t k = 0; p->placed && p->fp && k < p->fp->npad; k++) {
			struct pcb_spot sp;
			struct mark mk;

			pcb_spot(&rt->b, p, k, &sp);
			if ((rt->padnet[q][k] == cn->net && cn->net >= 0) ||
			    sp.layer > 0 ||
			    fabs(sp.x - wo->mid.x) > wo->r + (sp.w / 2) ||
			    fabs(sp.y - wo->mid.y) > wo->r + (sp.h / 2)) {
				continue;
			}
			mk.box[0] = sp.x - (sp.w / 2);
			mk.box[1] = sp.y - (sp.h / 2);
			mk.box[2] = sp.x + (sp.w / 2);
			mk.box[3] = sp.y + (sp.h / 2);
			mk.round = sp.shape == PAD_CIRCLE;
			mk.d = hypot(wo->d, wo->h) + SNAP;
			mark_box(rt, &mk);
		}
	}
}

// Marks the tracks of the top and the vias not of the net of cn in the
// window wo.
static void window_copper(struct router *rt, const struct conn *cn,
			  const struct win_of *wo)
{
	double mx = wo->mid.x;
	double my = wo->mid.y;
	double r = wo->r;

	for (size_t i = 0; i < rt->b.ntrack; i++) {
		const struct pcb_track *t = &rt->b.track_[i];

		if (t->net == cn->net || t->layer != 0) {
			continue;
		}
		for (size_t j = 1; j < t->n; j++) {
			const double *e = &t->xy[(2 * j) - 2];
			struct pcb_seg s = { { e[0], e[1] }, { e[2], e[3] } };

			if (fmin(e[0], e[2]) > mx + r ||
			    fmax(e[0], e[2]) < mx - r ||
			    fmin(e[1], e[3]) > my + r ||
			    fmax(e[1], e[3]) < my - r) {
				continue;
			}
			mark_seg(rt, s,
				 hypot(wo->d + (t->w / 2), wo->h) + SNAP);
		}
	}
	for (size_t i = 0; i < rt->b.nvia; i++) {
		const struct pcb_via *v = &rt->b.via[i];

		if (v->net != cn->net && fabs(v->x - mx) < r + v->dia &&
		    fabs(v->y - my) < r + v->dia) {
			struct mark mk = {
				{ v->x - (v->dia / 2), v->y - (v->dia / 2),
				  v->x + (v->dia / 2), v->y + (v->dia / 2) },
				1,
				hypot(wo->d, wo->h) + SNAP
			};

			mark_box(rt, &mk);
		}
	}
}

// Marks the edge, cell by cell, where the window comes near it, for a
// track of width w.
static void window_edge(struct router *rt, double w)
{
	const double win[4] = { cx(rt, 0), cy(rt, 0), cx(rt, WIN - 1),
				cy(rt, WIN - 1) };

	if (rt->scope ||
	    pcb_edge_dist(&rt->b, 1, win, 0, NULL) > rt->b.edge + w) {
		return;
	}
	for (int j = 0; j < WIN; j++) {
		for (int i = 0; i < WIN; i++) {
			double x = cx(rt, i);
			double y = cy(rt, j);
			const double at[4] = { x, y, x, y };

			if (pcb_edge_dist(&rt->b, 0, at, w / 2, NULL) <
			    rt->b.edge) {
				rt->bad[(j * WIN) + i] |= 2U;
			}
		}
	}
}

// Sets up the window for the connection cn about mid, its grid through
// grid: the cells its track may not take, each obstacle in the window
// marked in the cells round it.
static void window(struct router *rt, const struct conn *cn, struct pcb_pt mid,
		   struct pcb_pt grid)
{
	struct win_of wo;

	wo.mid = mid;
	wo.d = (cn->w / 2) + rt->b.clearance;
	wo.r = (WIN * CELL / 2) + wo.d + 1;
	wo.h = CELL * HALF_DIAG; // Half a diagonal step.
	rt->ox = fmod(grid.x, CELL);
	rt->oy = fmod(grid.y, CELL);
	rt->x0 = (int)lround((mid.x - rt->ox) / CELL) - (WIN / 2);
	rt->y0 = (int)lround((mid.y - rt->oy) / CELL) - (WIN / 2);
	memset(rt->bad, 0, sizeof rt->bad);
	window_pads(rt, cn, &wo);
	window_copper(rt, cn, &wo);
	window_edge(rt, cn->w);
}

// A span of cells, from lo to hi.
struct span {
	int lo, hi;
};

// How far v is outside the span s, 0 inside.
static int off_span(int v, struct span s)
{
	if (s.lo > v) {
		return s.lo - v;
	}
	return v > s.hi ? v - s.hi : 0;
}

// The least the path from cell c to the target can cost: the octile
// distance to the box of its cells (A*).
static double estimate(struct router *rt, int c)
{
	int i = c % WIN;
	int j = c / WIN;
	int dx = off_span(i, (struct span){ rt->tb[0], rt->tb[2] });
	int dy = off_span(j, (struct span){ rt->tb[1], rt->tb[3] });
	int lo = dx < dy ? dx : dy;
	int hi = dx < dy ? dy : dx;

	return hi - lo + (lo * DIAG);
}

// The order of the heap: the cost and the estimate.
static double key(struct router *rt, int s)
{
	return rt->cost[s] + estimate(rt, s / WAYS);
}

static void swap_at(struct router *rt, int i, int j)
{
	int t = rt->heap[i];

	rt->heap[i] = rt->heap[j];
	rt->heap[j] = t;
	rt->place[rt->heap[i]] = i;
	rt->place[rt->heap[j]] = j;
}

static void sift_up(struct router *rt, int i)
{
	while (i > 0 && key(rt, rt->heap[(i - 1) / 2]) > key(rt, rt->heap[i])) {
		swap_at(rt, i, (i - 1) / 2);
		i = (i - 1) / 2;
	}
}

// Takes the state of least key off the heap.
static int pop(struct router *rt)
{
	int top = rt->heap[0];
	int i = 0;

	rt->place[top] = -1;
	if (--rt->nheap > 0) {
		rt->heap[0] = rt->heap[rt->nheap];
		rt->place[rt->heap[0]] = 0;
	}
	for (;;) {
		int l = (2 * i) + 1;
		int r = l + 1;
		int m = i;

		if (l < rt->nheap &&
		    key(rt, rt->heap[l]) < key(rt, rt->heap[m])) {
			m = l;
		}
		if (r < rt->nheap &&
		    key(rt, rt->heap[r]) < key(rt, rt->heap[m])) {
			m = r;
		}
		if (m == i) {
			return top;
		}
		swap_at(rt, i, m);
		i = m;
	}
}

// A step of the search: to state s, at its cost, from a state entered
// the way d.
struct step {
	int s;
	double c;
	int d;
};

// Reaches the state of st: new, or cheaper than it was and not yet taken
// off the heap.
static void reach(struct router *rt, struct step st)
{
	int s = st.s;
	double c = st.c;
	int d = st.d;

	if (rt->stamp[s] != rt->search) {
		rt->stamp[s] = rt->search;
		rt->cost[s] = (float)c;
		rt->back[s] = (unsigned char)d;
		rt->place[s] = rt->nheap;
		rt->heap[rt->nheap++] = s;
		sift_up(rt, rt->place[s]);
	} else if (rt->place[s] >= 0 && c < rt->cost[s]) {
		rt->cost[s] = (float)c;
		rt->back[s] = (unsigned char)d;
		sift_up(rt, rt->place[s]);
	}
}

// The cell nearest p, or -1 off the window.
static int cell_at(const struct router *rt, const double p[2])
{
	long i = lround((p[0] - rt->ox) / CELL) - rt->x0;
	long j = lround((p[1] - rt->oy) / CELL) - rt->y0;

	if (i < 0 || j < 0 || i >= WIN || j >= WIN) {
		return -1;
	}
	return (int)((j * WIN) + i);
}

// The connections of the -v trace, and of the messages: PART.PAD (REF).
static const char *pad_label(struct router *rt, const struct pcb_end *e)
{
	static char buf[2][PAD_LABEL_LEN];
	static int k;
	const struct pcb_part *q = &rt->b.part[e->part];
	char *s = buf[k++ % 2];

	(void)snprintf(s, sizeof buf[0], "%s.%s (%s)", q->name,
		       q->fp->pad[e->pad].name, pcb_ref(q));
	return s;
}

// Whether a track of width w ending at p keeps the edge rule.
static int off_edge(const struct router *rt, const double p[2], double w)
{
	const double at[4] = { p[0], p[1], p[0], p[1] };

	return rt->scope ||
	       pcb_edge_dist(&rt->b, 0, at, w / 2, NULL) >= rt->b.edge;
}

// The point of the line between the ends of long pad s (pcb_pad_ends)
// nearest to, to p.
static void on_line(const struct pcb_spot *s, const double to[2], double p[2])
{
	double x = to[0];
	double y = to[1];
	double e[2][2];
	double dx = NAN;
	double dy = NAN;
	double l = NAN;
	double t = NAN;

	pcb_pad_ends(s, e);
	dx = e[1][0] - e[0][0];
	dy = e[1][1] - e[0][1];
	l = (dx * dx) + (dy * dy);
	t = (((x - e[0][0]) * dx) + ((y - e[0][1]) * dy)) / l;
	if (t < 0) {
		t = 0;
	} else if (t > 1) {
		t = 1;
	}
	p[0] = e[0][0] + (t * dx);
	p[1] = e[0][1] + (t * dy);
}

// The point of pad s a track of width w joins it at, toward (x, y): its
// center, or of a long pad the point of the line between its ends
// nearest (x, y), so a pad met from its side is joined there; but of
// the points of that line where the track keeps the edge rule, the
// nearest to that one, if any does.
static void pad_point(const struct router *rt, const struct pcb_spot *s,
		      const double to[2], double w, double p[2])
{
	double e[2][2];
	double q[2];
	double l = NAN;

	if (pcb_pad_ends(s, e) < 2) {
		memcpy(p, e[0], 2 * sizeof *p);
		return;
	}
	on_line(s, to, p);
	l = hypot(e[1][0] - e[0][0], e[1][1] - e[0][1]);
	for (int k = 0; k <= 2 * (int)ceil(l / CELL); k++) {
		int half = (k + 1) / 2;
		double t = (k % 2 ? 1 : -1) * (double)half * CELL / l;
		const double at[2] = { p[0] + ((e[1][0] - e[0][0]) * t),
				       p[1] + ((e[1][1] - e[0][1]) * t) };

		on_line(s, at, q);
		if (off_edge(rt, q, w)) {
			memcpy(p, q, sizeof q);
			return;
		}
	}
}

// Marks the cells of the window in pad s (NULL: none).
static void mark_pad(struct router *rt, unsigned char *in,
		     const struct pcb_spot *s)
{
	int i0 = 0;
	int i1 = 0;
	int j0 = 0;
	int j1 = 0;

	if (!s) {
		return;
	}
	i0 = down_int((s->x - s->w / 2 - rt->ox) / CELL) - rt->x0;
	i1 = up_int((s->x + s->w / 2 - rt->ox) / CELL) - rt->x0;
	j0 = down_int((s->y - s->h / 2 - rt->oy) / CELL) - rt->y0;
	j1 = up_int((s->y + s->h / 2 - rt->oy) / CELL) - rt->y0;
	for (int j = j0 < 0 ? 0 : j0; j <= j1 && j < WIN; j++) {
		for (int i = i0 < 0 ? 0 : i0; i <= i1 && i < WIN; i++) {
			in[(j * WIN) + i] = (unsigned char)inside_spot(
				s, cx(rt, i), cy(rt, j), 0);
		}
	}
}

// Marks 2 in g the cells along long pad s whose nearest point of the
// line between its ends keeps the edge rule for a track of width w.
static void mark_line(const struct router *rt, unsigned char *g,
		      const struct pcb_spot *s, double w)
{
	double e[2][2];
	double l = NAN;

	if (!s || pcb_pad_ends(s, e) != 2) {
		return;
	}
	l = hypot(e[1][0] - e[0][0], e[1][1] - e[0][1]);
	int n = steps_upto(l, CELL / 2);

	for (int k = 0; k < n; k++) {
		double t = k * (CELL / 2);
		const double at[2] = { e[0][0] + ((e[1][0] - e[0][0]) * t / l),
				       e[0][1] +
					       ((e[1][1] - e[0][1]) * t / l) };
		int c = cell_at(rt, at);
		double q[2];
		double u = NAN;

		if (c < 0 || g[c]) {
			continue;
		}
		// The cell's own point of the line, not past its ends.
		{
			const double cp[2] = { cx(rt, c % WIN),
					       cy(rt, c / WIN) };

			u = (((cp[0] - e[0][0]) * (e[1][0] - e[0][0])) +
			     ((cp[1] - e[0][1]) * (e[1][1] - e[0][1]))) /
			    l;
			on_line(s, cp, q);
		}
		if (u >= -TOUCH && u <= l + TOUCH && off_edge(rt, q, w)) {
			g[c] = 2;
		}
	}
}

// Marks in g the cells of the window a track of width w may end at on
// pad s (NULL: none) for point p: 2 the cells along a long pad whose
// nearest point of the line between its ends keeps the edge rule, then
// 1 the cell of p, if p keeps it.  The box of the cells grows box
// (NULL: none).  Returns the number of cells.
static int mark_ends(struct router *rt, unsigned char *g,
		     const struct pcb_spot *s, const double p[2], double w,
		     int *box)
{
	int c = 0;
	int count = 0;

	mark_line(rt, g, s, w);
	c = cell_at(rt, p);
	if (c >= 0 && off_edge(rt, p, w)) {
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
static void end_at(const struct router *rt, const unsigned char *g,
		   const struct pcb_spot *s, const double p[2], int c,
		   double q[2])
{
	if (g[c] == 2) {
		const double cp[2] = { cx(rt, c % WIN), cy(rt, c / WIN) };

		on_line(s, cp, q);
	} else {
		memcpy(q, p, 2 * sizeof *q);
	}
}

// The ends of a section: points a and b, each in its pad (sa, sb), or
// NULL for a point between.
struct sect {
	const double *a;
	const struct pcb_spot *sa;
	const double *b;
	const struct pcb_spot *sb;
};

// The corners of a track: n points at xy.
struct path {
	double *xy;
	size_t n;
};

// Lays the window for the section sc of the connection cn and marks its
// ends; returns 0 if an end is off it or breaks the edge rule.
static int section_window(struct router *rt, const struct conn *cn,
			  const struct sect *sc, int ce[2])
{
	const double *a = sc->a;
	const double *b = sc->b;
	struct pcb_pt mid = { (a[0] + b[0]) / 2, (a[1] + b[1]) / 2 };
	struct pcb_pt grid = { a[0], a[1] };

	window(rt, cn, mid, grid);
	ce[0] = cell_at(rt, a);
	ce[1] = cell_at(rt, b);
	memset(rt->inp, 0, sizeof rt->inp);
	memset(rt->inq, 0, sizeof rt->inq);
	memset(rt->ga, 0, sizeof rt->ga);
	memset(rt->gb, 0, sizeof rt->gb);
	rt->tb[0] = rt->tb[1] = WIN;
	rt->tb[2] = rt->tb[3] = -1;
	// Nothing overrides the edge, not even at the ends.
	if (ce[0] < 0 || ce[1] < 0 ||
	    !mark_ends(rt, rt->ga, sc->sa, a, cn->w, NULL) ||
	    !mark_ends(rt, rt->gb, sc->sb, b, cn->w, rt->tb)) {
		return 0;
	}
	mark_pad(rt, rt->inp, sc->sa);
	mark_pad(rt, rt->inq, sc->sb);
	if (++rt->search == 0) {
		memset(rt->stamp, 0, sizeof rt->stamp);
		rt->search = 1;
	}
	rt->nheap = 0;
	return 1;
}

// In line: the section sc from cell ca to cell cb as one straight track,
// into out, if it is clear; returns whether it is.
static int straight(struct router *rt, const struct sect *sc, const int ce[2],
		    struct path *out)
{
	int ca = ce[0];
	int cb = ce[1];
	int di = (cb % WIN > ca % WIN) - (cb % WIN < ca % WIN);
	int dj = (cb / WIN > ca / WIN) - (cb / WIN < ca / WIN);
	int d = 0;
	int clear = 1;

	if (rt->ga[ca] != 1 || rt->gb[cb] != 1 ||
	    (fabs(sc->a[0] - sc->b[0]) >= SAME &&
	     fabs(sc->a[1] - sc->b[1]) >= SAME)) {
		return 0;
	}
	for (int c = ca; clear && c != cb; c += (dj * WIN) + di) {
		clear = !(rt->bad[c] & 2U) &&
			(!rt->bad[c] || rt->inp[c] || rt->inq[c]);
	}
	while (d < WAYS && (way_di[d] != di || way_dj[d] != dj)) {
		d++;
	}
	if (!clear || d == WAYS) {
		return 0;
	}
	out->xy = pcb_alloc(4 * sizeof *out->xy);
	memcpy(out->xy, sc->a, 2 * sizeof *out->xy);
	memcpy(out->xy + 2, sc->b, 2 * sizeof *out->xy);
	out->n = 2;
	return 1;
}

// Reaches from state s the states round its cell: straight while in a
// pad, never into a cell bad unless in a pad of an end (or the target),
// and no sharp turns.
static void expand(struct router *rt, int s)
{
	int c = s / WAYS;
	int d = s % WAYS;
	int i = c % WIN;
	int j = c / WIN;

	for (int nd = 0; nd < WAYS; nd++) {
		int ni = i + way_di[nd];
		int nj = j + way_dj[nd];
		int nc = 0;
		int turn = 0;
		struct step to;

		// Straight while in a pad.
		if ((rt->inp[c] || rt->inq[c]) && nd != d) {
			continue;
		}
		if (ni < 0 || nj < 0 || ni >= WIN || nj >= WIN) {
			continue;
		}
		nc = (nj * WIN) + ni;
		if (!rt->gb[nc] &&
		    ((rt->bad[nc] & 2U) ||
		     (rt->bad[nc] && !rt->inq[nc] && !rt->inp[nc]))) {
			continue;
		}
		turn = abs(nd - d);
		turn = turn > 4 ? WAYS - turn : turn;
		if (turn > 2) {
			continue; // No sharp turns.
		}
		to.s = (nc * WAYS) + nd;
		to.c = rt->cost[s] + (nd % 2 ? DIAG : 1) + (turn * BEND);
		to.d = d;
		reach(rt, to);
	}
}

// The search from the start cells to the target's: the state reached
// there, or -1, the cell nearest the target in *best.
static int search_to(struct router *rt, int *best)
{
	for (int c = 0; c < WIN * WIN; c++) {
		for (int d = 0; rt->ga[c] && d < WAYS; d++) {
			struct step to = { (c * WAYS) + d, 0, WAYS };

			rt->inp[c] = 1;
			reach(rt, to);
		}
	}
	while (rt->nheap) {
		int s = pop(rt);
		int c = s / WAYS;

		if (rt->gb[c] && rt->cost[s] > 0) {
			return s;
		}
		if (estimate(rt, c) < estimate(rt, *best)) {
			*best = c;
		}
		expand(rt, s);
	}
	return -1;
}

// The corners of the way found from state goal back to its start, into
// out, from b to a; returns the start cell.
static int trace(struct router *rt, int goal, struct path *out)
{
	size_t m = 0;
	size_t cap = PATH_START;
	int start = -1;

	out->xy = pcb_alloc(2 * cap * sizeof *out->xy);
	for (int st = goal, last = -1;;) {
		int d = st % WAYS;
		int c = st / WAYS;
		int pd = rt->back[st];

		if (m + 1 >= cap) {
			double *more = NULL;

			cap *= 2;
			more = realloc(out->xy, 2 * cap * sizeof *out->xy);
			if (!more) {
				pcb_die("out of memory");
			}
			out->xy = more;
		}
		if (last < 0 || d != last || pd == WAYS) {
			out->xy[2 * m] = cx(rt, c % WIN);
			out->xy[(2 * m) + 1] = cy(rt, c / WIN);
			m++;
		}
		if (pd == WAYS) {
			start = c;
			break;
		}
		last = d;
		st = ((c - (way_dj[d] * WIN) - way_di[d]) * WAYS) + pd;
	}
	out->n = m;
	return start;
}

// Puts the ends of the corners of out, from b to a, where they join
// (qb, qa), the corner before b on b's line for the way goal enters,
// then turns them round, from a to b.
static void exact_ends(struct path *out, int goal, const double qa[2],
		       const double qb[2])
{
	double *xy = out->xy;
	size_t m = out->n;

	if (m >= 3 && goal % 2 == 0) {
		if (goal % WAYS == 0 || goal % WAYS == 4) {
			xy[3] = qb[1];
		} else {
			xy[2] = qb[0];
		}
	}
	xy[0] = qb[0];
	xy[1] = qb[1];
	xy[(2 * m) - 2] = qa[0];
	xy[(2 * m) - 1] = qa[1];
	for (size_t i = 0; i < m / 2; i++) {
		for (size_t k = 0; k < 2; k++) {
			double t = xy[(2 * i) + k];

			xy[(2 * i) + k] = xy[(2 * (m - 1 - i)) + k];
			xy[(2 * (m - 1 - i)) + k] = t;
		}
	}
}

// Routes the section sc of the connection cn, both ends in the window
// laid about their middle, straight while in the pad of either; a long
// pad is left or met at any point of the line between its ends.  Its
// corners, from a to b, go to out; returns 0 if there is no way, and
// where it got nearest goes to near.
static int section(struct router *rt, const struct conn *cn,
		   const struct sect *sc, struct path *out, double near[2])
{
	int ce[2] = { 0, 0 };
	int goal = -1;
	int best = 0;
	int start = -1;
	double qa[2];
	double qb[2];

	near[0] = sc->a[0];
	near[1] = sc->a[1];
	if (!section_window(rt, cn, sc, ce)) {
		return 0;
	}
	best = ce[0];
	if (straight(rt, sc, ce, out)) {
		return 1;
	}
	goal = search_to(rt, &best);
	if (goal < 0) {
		near[0] = cx(rt, best % WIN);
		near[1] = cy(rt, best / WIN);
		return 0;
	}
	start = trace(rt, goal, out);
	end_at(rt, rt->gb, sc->sb, sc->b, goal / WAYS, qb);
	end_at(rt, rt->ga, sc->sa, sc->a, start, qa);
	exact_ends(out, goal, qa, qb);
	return 1;
}

// A point between the sections of the connection cn: on the grid of
// the first pad, the straight line's point p, or slid sideways (u the
// way across) off copper and out of cutouts, as near as it is clear;
// returns 0 if none is.
static int midpoint(const struct router *rt, const struct conn *cn,
		    const double *p, const double *u, double q[2])
{
	const char *who = NULL;

	for (int k = 0; k <= 2 * SLIDE; k++) {
		int half = (k + 1) / 2;
		double t = (k % 2 ? 1 : -1) * (double)half * CELL;
		struct disk_on d = { 0, 0, cn->w / 2, 0, cn->net };

		q[0] = rt->ox +
		       ((double)lround((p[0] + (u[0] * t) - rt->ox) / CELL) *
			CELL);
		q[1] = rt->oy +
		       ((double)lround((p[1] + (u[1] * t) - rt->oy) / CELL) *
			CELL);
		d.x = q[0];
		d.y = q[1];
		if (free_at(rt, &d, &who)) {
			return 1;
		}
	}
	return 0;
}

// A connection being routed in sections: from pa to pb, in its pads s0
// and s1 (NULL for a point), the way across u, in k sections; the
// corners so far, and where it got stuck.
struct sections {
	double pa[2];
	double pb[2];
	const struct pcb_spot *s0;
	const struct pcb_spot *s1;
	double u[2];
	int k;
	struct path all;
	double near[2];
};

// Routes section i of sn of the connection cn, and adds its corners;
// returns 0 if there is no way.
static int add_section(struct router *rt, const struct conn *cn,
		       struct sections *sn, int i)
{
	double from[2];
	double to[2];
	struct path out = { NULL, 0 };
	struct sect sc;
	size_t nall = sn->all.n;
	int ok = 0;

	if (!i) {
		memcpy(from, sn->pa, sizeof from);
	} else {
		memcpy(from, sn->all.xy + (2 * nall) - 2, sizeof from);
	}
	if (i == sn->k - 1) {
		memcpy(to, sn->pb, sizeof to);
	} else {
		const double p[2] = {
			sn->pa[0] + ((sn->pb[0] - sn->pa[0]) * (i + 1) / sn->k),
			sn->pa[1] + ((sn->pb[1] - sn->pa[1]) * (i + 1) / sn->k)
		};

		if (!midpoint(rt, cn, p, sn->u, to)) {
			sn->near[0] = p[0];
			sn->near[1] = p[1];
			return 0;
		}
	}
	sc.a = from;
	sc.sa = i ? NULL : sn->s0;
	sc.b = to;
	sc.sb = i == sn->k - 1 ? sn->s1 : NULL;
	ok = section(rt, cn, &sc, &out, sn->near);
	// The sections end to end, each joint once.
	for (size_t j = nall ? 1 : 0; ok && j < out.n; j++) {
		sn->all.xy = pcb_grow(sn->all.xy, (2 * sn->all.n) + 1,
				      sizeof *sn->all.xy);
		sn->all.xy[2 * sn->all.n] = out.xy[2 * j];
		sn->all.xy[(2 * sn->all.n) + 1] = out.xy[(2 * j) + 1];
		sn->all.n++;
	}
	free(out.xy);
	return ok;
}

// The ends of the connection cn from pad a to pad b into sn: of a long
// pad, the point of the line between its ends nearest the other pad;
// or, with at, from the point at to a.
static void connect_ends(const struct router *rt, const struct conn *cn,
			 const struct pcb_spot sp[2], const double *at,
			 struct sections *sn)
{
	pad_point(rt, &sp[0], (const double[2]){ sp[1].x, sp[1].y }, cn->w,
		  sn->pa);
	pad_point(rt, &sp[1], sn->pa, cn->w, sn->pb);
	pad_point(rt, &sp[0], sn->pb, cn->w, sn->pa);
	sn->s0 = &sp[0];
	sn->s1 = &sp[1];
	if (at) {
		// From the point to the pad, so the point is on the grid.
		pad_point(rt, &sp[0], at, cn->w, sn->pb);
		memcpy(sn->pa, at, sizeof sn->pa);
		sn->s0 = NULL;
		sn->s1 = &sp[0];
	}
}

// Adds the track of corners all of the connection cn from pad a to pad
// b, and the connection; returns it.
static int add_link_track(struct router *rt, const struct conn *cn,
			  const struct pcb_end *ab[2], struct path all)
{
	char *ka = pcb_pad_key(&rt->b, ab[0]->part, ab[0]->pad);
	char *kb = pcb_pad_key(&rt->b, ab[1]->part, ab[1]->pad);
	int l = pcb_add_link(&rt->b, cn->net, ka, kb, cn->w);
	struct pcb_track t;

	free(ka);
	free(kb);
	t.net = cn->net;
	t.layer = 0;
	t.link = l;
	t.w = cn->w;
	t.n = all.n;
	t.xy = all.xy;
	rt->b.track_ =
		pcb_grow(rt->b.track_, rt->b.ntrack, sizeof *rt->b.track_);
	rt->b.track_[rt->b.ntrack++] = t;
	return l;
}

// Joins pad a to pad b by the connection cn: from the center of a to
// the center of b (of a long pad, from any point of the line between its
// ends, the nearest the other pad first), in sections along the
// straight line between them, each within the window; or, with at,
// from the point at on copper of the net, left any way, to a, on a grid
// laid through at, the connection named by a and b all the same.
// Returns 0 if there is no way, and stuck says where.
static int connect(struct router *rt, const struct conn *cn,
		   const struct pcb_end *a, const struct pcb_end *b,
		   const double *at)
{
	struct pcb_spot sp[2];
	struct sections sn;
	const struct pcb_end *ab[2] = { a, b };
	double len = NAN;
	double span = (WIN - (2 * MARGIN) - (2 * SLIDE)) * CELL;
	int ok = 1;

	memset(&sn, 0, sizeof sn);
	pcb_spot(&rt->b, &rt->b.part[a->part], a->pad, &sp[0]);
	pcb_spot(&rt->b, &rt->b.part[b->part], b->pad, &sp[1]);
	connect_ends(rt, cn, sp, at, &sn);
	len = hypot(sn.pb[0] - sn.pa[0], sn.pb[1] - sn.pa[1]);
	sn.u[0] = len > 0 ? -(sn.pb[1] - sn.pa[1]) / len : 0;
	sn.u[1] = len > 0 ? (sn.pb[0] - sn.pa[0]) / len : 1;
	sn.k = (int)ceil(
		fmax(fabs(sn.pb[0] - sn.pa[0]), fabs(sn.pb[1] - sn.pa[1])) /
		span);
	sn.k = sn.k < 1 ? 1 : sn.k;
	rt->ox = fmod(sn.pa[0], CELL);
	rt->oy = fmod(sn.pa[1], CELL);
	for (int i = 0; ok && i < sn.k; i++) {
		ok = add_section(rt, cn, &sn, i);
	}
	if (!ok) {
		(void)snprintf(rt->stuck, sizeof rt->stuck,
			       "%s to %s, stuck at %.3f %.3f", pad_label(rt, a),
			       pad_label(rt, b), sn.near[0], sn.near[1]);
		free(sn.all.xy);
		return 0;
	}
	add_link_track(rt, cn, ab, sn.all);
	if (rt->all->verbose) {
		(void)fprintf(stderr,
			      "%s: %s to %s%s, %.3f wide, %d sections\n",
			      rt->b.net[cn->net].name, pad_label(rt, a),
			      at ? "the middle of " : "", pad_label(rt, b),
			      cn->w, sn.k);
	}
	return 1;
}

// The ends of net n to route on the top: its pads on the top.
static size_t top_ends(struct router *rt, int n, struct pcb_end **ends)
{
	size_t m = 0;

	*ends = pcb_alloc((rt->b.net[n].nend + 1) * sizeof **ends);
	for (size_t j = 0; j < rt->b.net[n].nend; j++) {
		struct pcb_spot s;
		const struct pcb_end *e = &rt->b.net[n].end[j];

		if (!rt->b.part[e->part].placed) {
			continue;
		}
		pcb_spot(&rt->b, &rt->b.part[e->part], e->pad, &s);
		if (s.layer <= 0) {
			(*ends)[m++] = *e;
		}
	}
	return m;
}

// The ends of a net routed: its net, its m pads on the top, and the
// group of each.
struct net_ends {
	int net;
	struct pcb_end *e;
	size_t m;
	size_t *g;
};

// Merges the groups of ends x and y of ne, joined by a connection.
static int merge(struct net_ends *ne, size_t x, size_t y)
{
	size_t *g = ne->g;
	size_t lo = g[x] < g[y] ? g[x] : g[y];
	size_t hi = g[x] < g[y] ? g[y] : g[x];

	if (g[x] == g[y]) {
		return 0;
	}
	for (size_t j = 0; j < ne->m; j++) {
		g[j] = g[j] == hi ? lo : g[j];
	}
	return 1;
}

// The group of each end of ne, joined by the connections there are: the
// end of the least index of its group.
static void groups(const struct router *rt, struct net_ends *ne)
{
	size_t m = ne->m;
	char **key = (char **)pcb_alloc((m + 1) * sizeof *key);

	for (size_t j = 0; j < m; j++) {
		ne->g[j] = j;
		key[j] = pcb_pad_key(&rt->b, ne->e[j].part, ne->e[j].pad);
	}
	for (int changed = 1; changed;) {
		changed = 0;
		for (size_t l = 0; l < rt->b.nlink; l++) {
			size_t x = m;
			size_t y = m;

			if (rt->b.link[l].net != ne->net) {
				continue;
			}
			for (size_t j = 0; j < m; j++) {
				x = !strcmp(key[j], rt->b.link[l].a) ? j : x;
				y = !strcmp(key[j], rt->b.link[l].b) ? j : y;
			}
			if (x < m && y < m && merge(ne, x, y)) {
				changed = 1;
			}
		}
	}
	for (size_t j = 0; j < m; j++) {
		free(key[j]);
	}
	free((void *)key);
}

// The end of a pin among the ends of ne, or m.
static size_t end_of(const struct router *rt, const char *pin,
		     const struct net_ends *ne)
{
	struct pcb_end at = { 0, 0 };

	for (size_t j = 0; pcb_pin(&rt->b, pin, &at) && j < ne->m; j++) {
		if (ne->e[j].part == at.part && ne->e[j].pad == at.pad) {
			return j;
		}
	}
	return ne->m;
}

// Joins ends i and j of ne, if not joined yet.
static int join(struct router *rt, struct net_ends *ne, size_t i, size_t j)
{
	char *ka = NULL;
	char *kb = NULL;
	struct conn cn = { ne->net, NAN };
	int ok = 0;

	if (i >= ne->m || j >= ne->m || ne->g[i] == ne->g[j]) {
		return 1;
	}
	ka = pcb_pad_key(&rt->b, ne->e[i].part, ne->e[i].pad);
	kb = pcb_pad_key(&rt->b, ne->e[j].part, ne->e[j].pad);
	cn.w = pcb_link_width(&rt->b, ne->net, ka, kb);
	free(ka);
	free(kb);
	ok = connect(rt, &cn, &ne->e[i], &ne->e[j], NULL);
	if (ok) {
		groups(rt, ne);
	}
	return ok;
}

// Whether statement st is a fork of net n: fork NET PIN to PIN PIN.
static int is_fork(const struct router *rt, const struct pcb_stmt *st, int n)
{
	return !strcmp(st->w[0], "fork") && st->n == FORK_WORDS &&
	       !strcmp(st->w[3], "to") && !strcmp(st->w[1], rt->b.net[n].name);
}

// The connection of net n between the pads of keys ka and kb, named
// either way, or -1.
static long link_of(const struct router *rt, int n, const char *ka,
		    const char *kb)
{
	for (size_t l = 0; l < rt->b.nlink; l++) {
		const struct pcb_link *k = &rt->b.link[l];

		if (k->net == n && ((!strcmp(k->a, ka) && !strcmp(k->b, kb)) ||
				    (!strcmp(k->a, kb) && !strcmp(k->b, ka)))) {
			return (long)l;
		}
	}
	return -1;
}

// The key of the pad of a pin (PART.PIN), or NULL.
static char *pin_key(const struct router *rt, const char *pin)
{
	struct pcb_end at = { 0, 0 };

	return pcb_pin(&rt->b, pin, &at) ?
		       pcb_pad_key(&rt->b, at.part, at.pad) :
		       NULL;
}

// The middle of the copper of connection l, by its length.
static void middle(const struct router *rt, long l, double p[2])
{
	const struct pcb_track *t = NULL;
	double len = 0;
	double f = NAN;

	for (size_t i = 0; !t && i < rt->b.ntrack; i++) {
		if (rt->b.track_[i].link == l && rt->b.track_[i].n) {
			t = &rt->b.track_[i];
		}
	}
	if (!t) {
		p[0] = p[1] = 0;
		return;
	}
	for (size_t j = 1; j < t->n; j++) {
		len += hypot(t->xy[2 * j] - t->xy[(2 * j) - 2],
			     t->xy[(2 * j) + 1] - t->xy[(2 * j) - 1]);
	}
	f = len / 2;
	p[0] = t->xy[(2 * t->n) - 2];
	p[1] = t->xy[(2 * t->n) - 1];
	for (size_t j = 1; j < t->n; j++) {
		const double *e = &t->xy[(2 * j) - 2];
		double d = hypot(e[2] - e[0], e[3] - e[1]);

		if (f <= d) {
			p[0] = d > 0 ? e[0] + ((e[2] - e[0]) * f / d) : e[0];
			p[1] = d > 0 ? e[1] + ((e[3] - e[1]) * f / d) : e[1];
			return;
		}
		f -= d;
	}
}

// Joins the first pin p of the fork st to the middle of the copper
// joining its pin a to the next, a connection named by p and a.
static int fork_stem(struct router *rt, struct net_ends *ne,
		     const struct pcb_stmt *st, const size_t pa[2])
{
	char *ka = pin_key(rt, st->w[FORK_WORDS - 2]);
	char *kb = pin_key(rt, st->w[FORK_WORDS - 1]);
	char *kp = pin_key(rt, st->w[2]);
	long l = link_of(rt, ne->net, ka, kb);
	int ok = 1;

	if (l < 0) {
		pcb_error(&rt->b, st->line, "fork: %s and %s joined otherwise",
			  st->w[FORK_WORDS - 2], st->w[FORK_WORDS - 1]);
	} else if (ne->g[pa[0]] != ne->g[pa[1]]) {
		double mid[2];
		struct conn cn = { ne->net,
				   pcb_link_width(&rt->b, ne->net, kp, ka) };

		middle(rt, l, mid);
		ok = connect(rt, &cn, &ne->e[pa[0]], &ne->e[pa[1]], mid);
		if (ok) {
			groups(rt, ne);
		}
	}
	free(ka);
	free(kb);
	free(kp);
	return ok;
}

// Carries out statement st, a fork of the net of ne: joins its two last
// pins by one connection, then its first pin to the middle of that
// copper, a connection named by the first pin and the second.
static int fork_net(struct router *rt, struct net_ends *ne,
		    const struct pcb_stmt *st)
{
	size_t p = end_of(rt, st->w[2], ne);
	size_t a = end_of(rt, st->w[FORK_WORDS - 2], ne);
	size_t b = end_of(rt, st->w[FORK_WORDS - 1], ne);
	size_t m = ne->m;
	const size_t pa[2] = { p, a };

	if (p == m || a == m || b == m || p == a || p == b || a == b) {
		pcb_error(&rt->b, st->line,
			  "fork NET PIN to PIN PIN, three pins of the net "
			  "on the top");
		return 1;
	}
	if (!join(rt, ne, a, b)) {
		return 0;
	}
	return fork_stem(rt, ne, st, pa);
}

// Carries out statement st of the net of ne, a path or a width: joins
// the pins it names.  Returns 0 if a pad finds no way.
static int path_net(struct router *rt, struct net_ends *ne,
		    const struct pcb_stmt *st)
{
	size_t m = ne->m;

	if (!strcmp(st->w[0], "path") && st->n >= 4 &&
	    !strcmp(st->w[1], rt->b.net[ne->net].name)) {
		for (int k = 3; k < st->n; k++) {
			size_t a = end_of(rt, st->w[k - 1], ne);
			size_t b = end_of(rt, st->w[k], ne);

			if (a == m || b == m) {
				pcb_error(&rt->b, st->line,
					  "no pin %s on the top",
					  st->w[a == m ? k - 1 : k]);
				continue;
			}
			if (!join(rt, ne, a, b)) {
				return 0;
			}
		}
	} else if (!strcmp(st->w[0], "width") && st->n == 4) {
		size_t a = end_of(rt, st->w[1], ne);
		size_t b = end_of(rt, st->w[2], ne);

		if (a < m && b < m) {
			return join(rt, ne, a, b);
		}
	}
	return 1;
}

// The nearest pad of ne not joined to the group of the first and the
// nearest joined, into ij; returns 0 if all are joined.
static int nearest_pair(const struct router *rt, const struct net_ends *ne,
			size_t ij[2])
{
	size_t m = ne->m;
	double bd = FAR;

	ij[0] = ij[1] = m;
	for (size_t i = 0; i < m; i++) {
		struct pcb_spot a;

		if (ne->g[i] == ne->g[0]) {
			continue;
		}
		pcb_spot(&rt->b, &rt->b.part[ne->e[i].part], ne->e[i].pad, &a);
		for (size_t j = 0; j < m; j++) {
			struct pcb_spot o;
			double d = NAN;

			if (ne->g[j] != ne->g[0]) {
				continue;
			}
			pcb_spot(&rt->b, &rt->b.part[ne->e[j].part],
				 ne->e[j].pad, &o);
			d = hypot(a.x - o.x, a.y - o.y);
			if (d < bd - TOUCH) {
				bd = d;
				ij[0] = i;
				ij[1] = j;
			}
		}
	}
	return ij[0] != m;
}

// Routes signal net n: joins its groups of pads by connections, those
// its forks name first, then those of its paths and widths, then the
// nearest pad not joined to the nearest joined.  Returns 0 if a pad
// finds no way, and *who says which, from which, and where it got stuck.
static int route_net(struct router *rt, int n, const char **who)
{
	struct net_ends ne = { n, NULL, 0, NULL };
	size_t ij[2] = { 0, 0 };
	int ok = 1;

	ne.m = top_ends(rt, n, &ne.e);
	ne.g = pcb_alloc((ne.m + 1) * sizeof *ne.g);
	groups(rt, &ne);
	for (size_t i = 0; ok && i < rt->b.nstmt; i++) {
		if (is_fork(rt, &rt->b.stmt[i], n)) {
			ok = fork_net(rt, &ne, &rt->b.stmt[i]);
		}
	}
	for (size_t i = 0; ok && i < rt->b.nstmt; i++) {
		ok = path_net(rt, &ne, &rt->b.stmt[i]);
	}
	while (ok && nearest_pair(rt, &ne, ij)) {
		ok = join(rt, &ne, ij[0], ij[1]);
	}
	if (!ok) {
		*who = rt->stuck;
	}
	free(ne.e);
	free(ne.g);
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

// Whether the via d, of its net, keeps the via gap from every pad of its
// net (its ring that far from their edges, so it joins them by a neck
// track, not standing on them) and the mask web from every opening of
// the mask, on either side, for the via is tented, and stands off the
// keep-out of every hole (pcbfp(5)).
static int via_room(const struct router *rt, const struct disk_on *d,
		    const char **who)
{
	for (size_t q = 0; q < rt->b.npart; q++) {
		const struct pcb_part *p = &rt->b.part[q];
		char why[WHY_LEN];

		if (pcb_keeps(&rt->b, p, d->x, d->y, d->r, why, sizeof why)) {
			*who = part_label(p);
			return 0;
		}
		for (size_t k = 0; p->placed && p->fp && k < p->fp->npad; k++) {
			struct pcb_spot s;

			pcb_spot(&rt->b, p, k, &s);
			if (rt->padnet[q][k] == d->net && pcb_soldered(p, k) &&
			    pcb_spot_dist(&s, d->x, d->y) <
				    d->r + rt->b.viagap - TOUCH) {
				*who = part_label(p);
				return 0;
			}
			// A contact pad's opening: bare copper, no solder to
			// bridge to the via.
			for (int side = 0; pcb_soldered(p, k) && side < 2;
			     side++) {
				struct pcb_end at = { q, k };
				double o[4];

				if (pcb_opening(&rt->b, at, side, o) &&
				    hypot(fmax(fmax(o[0] - d->x, d->x - o[2]),
					       0),
					  fmax(fmax(o[1] - d->y, d->y - o[3]),
					       0)) <
					    d->r + rt->b.maskweb - TOUCH) {
					*who = part_label(p);
					return 0;
				}
			}
		}
	}
	return 1;
}

// Adds a track like t (its net, connection, layer and width) along s.
static void add_track(struct router *rt, struct pcb_track t, struct pcb_seg s)
{
	t.n = 2;
	t.xy = pcb_alloc(4 * sizeof *t.xy);
	t.xy[0] = s.a.x;
	t.xy[1] = s.a.y;
	t.xy[2] = s.b.x;
	t.xy[3] = s.b.y;
	rt->b.track_ =
		pcb_grow(rt->b.track_, rt->b.ntrack, sizeof *rt->b.track_);
	rt->b.track_[rt->b.ntrack++] = t;
}

// The neck of pad s of part q on its plane's layer, a track like t: out
// of the pad along its longer side, away from the middle of the part,
// past the clearance of the pour, so the pad joins its plane by the neck
// alone.
static void neck(struct router *rt, struct pcb_track t,
		 const struct pcb_part *q, const struct pcb_spot *s)
{
	int along_x = s->w >= s->h;
	double ox = s->x - q->x;
	double oy = s->y - q->y;
	double ux = 0;
	double uy = 0;
	double len =
		((along_x ? s->w : s->h) / 2) + rt->b.clearance + (2 * t.w);
	struct pcb_seg sg;

	if (along_x) {
		ux = ox < 0 ? -1 : 1;
	} else {
		uy = oy < 0 ? -1 : 1;
	}
	sg.a = (struct pcb_pt){ s->x, s->y };
	sg.b = (struct pcb_pt){ s->x + (ux * len), s->y + (uy * len) };
	t.layer = s->layer;
	add_track(rt, t, sg);
	if (rt->all->verbose) {
		(void)fprintf(stderr, "%s: neck of %s (%s) on its plane\n",
			      rt->b.net[t.net].name, q->name, pcb_ref(q));
	}
}

// Whether a connection of net n from pad key (to its via or plane) is
// there.
static int has_link(const struct router *rt, int n, const char *key)
{
	for (size_t l = 0; l < rt->b.nlink; l++) {
		if (rt->b.link[l].net == n && !strcmp(rt->b.link[l].a, key)) {
			return 1;
		}
	}
	return 0;
}

// A pad of a plane net that wants a via: its end, its spot, its part,
// the width of its neck, and the way the source says (-1 for none).
struct via_pad {
	const struct pcb_end *e;
	struct pcb_spot s;
	const struct pcb_part *q;
	double w;
	int want;
	int net;
};

// Whether the pad of vp has a pin with a hole too, which needs no via.
static int holed(const struct via_pad *vp)
{
	const struct fp *f = vp->q->fp;

	for (size_t p = 0; p < f->npad; p++) {
		if (f->pad[p].hole &&
		    !strcmp(f->pad[p].name, f->pad[vp->e->pad].name)) {
			return 1;
		}
	}
	return 0;
}

// The way a via statement says the via of the pad of vp goes, or -1.
static int via_way(const struct router *rt, const struct via_pad *vp)
{
	int want = -1;

	for (size_t i = 0; i < rt->b.nstmt; i++) {
		const struct pcb_stmt *st = &rt->b.stmt[i];
		struct pcb_end at = { 0, 0 };

		if (!strcmp(st->w[0], "via") && st->n == 3 &&
		    pcb_pin(&rt->b, st->w[1], &at) && at.part == vp->e->part &&
		    !strcmp(vp->q->fp->pad[at.pad].name,
			    vp->q->fp->pad[vp->e->pad].name)) {
			want = way_word(st->w[2]);
		}
	}
	return want;
}

// Whether the neck from the pad of vp to a via at (x, y) keeps clear.
static int neck_clear(const struct router *rt, const struct via_pad *vp,
		      const double at[2], const char **who)
{
	const struct pcb_spot *s = &vp->s;
	int clear = 1;

	int n = steps_upto(1, NECK_STEP);

	for (int k = 0; k < n && clear; k++) {
		double f = k * NECK_STEP;
		struct disk_on d = { s->x + ((at[0] - s->x) * f),
				     s->y + ((at[1] - s->y) * f), vp->w / 2,
				     s->layer, vp->net };

		clear = free_at(rt, &d, who);
	}
	return clear;
}

// The score of the first place clear along the way d of the 16 from the
// pad of vp (out from the part rather than over it), its place into at;
// FAR if none is.
static double via_along(const struct router *rt, const struct via_pad *vp,
			int d, double at[2], const char **who)
{
	const struct pcb_spot *s = &vp->s;
	double a = d * PI / VIA_HALF;
	double ux = cos(a);
	double uy = sin(a);
	double ox = s->x - vp->q->x;
	double oy = s->y - vp->q->y;
	double out = 1;

	if (hypot(ox, oy) > TOUCH) {
		out = ((ux * ox) + (uy * oy)) / hypot(ox, oy);
	}
	int n = steps_below(VIA_TO - VIA_FROM, CELL);

	for (int k = 0; k < n; k++) {
		double t = VIA_FROM + (k * CELL);
		struct disk_on v = { s->x + (ux * t), s->y + (uy * t),
				     rt->b.via_dia / 2, -1, vp->net };

		at[0] = v.x;
		at[1] = v.y;
		if (via_room(rt, &v, who) && free_at(rt, &v, who) &&
		    neck_clear(rt, vp, at, who)) {
			return t - (out * OUT_SHARE);
		}
	}
	return FAR;
}

// The best place for the via of the pad of vp, into at; 0 if none is,
// what is in the way to *who.
static int via_spot(const struct router *rt, const struct via_pad *vp,
		    double at[2], const char **who)
{
	double best = FAR;

	for (int d = 0; d < VIA_WAYS; d++) {
		int dir = (int)lround(d * PI / VIA_HALF / (PI / 2)) % 4;
		double here[2];
		double score = NAN;

		if (vp->want >= 0 && (d % 4 || dir != vp->want)) {
			continue;
		}
		score = via_along(rt, vp, d, here, who);
		if (score < best) {
			best = score;
			at[0] = here[0];
			at[1] = here[1];
		}
	}
	return best <= NEAR_FAR;
}

// Adds the via at to the pad of vp, of key, on a neck track.
static void add_via(struct router *rt, const struct via_pad *vp,
		    const char *key, const double at[2])
{
	int n = vp->net;
	int l = pcb_add_link(&rt->b, n, key, "via", vp->w);
	struct pcb_via v = {
		n, l, at[0], at[1], rt->b.via_dia, rt->b.via_drill
	};
	struct pcb_track t = { 0 };
	struct pcb_seg sg = { { vp->s.x, vp->s.y }, { at[0], at[1] } };

	rt->b.via = pcb_grow(rt->b.via, rt->b.nvia, sizeof *rt->b.via);
	rt->b.via[rt->b.nvia++] = v;
	t.net = n;
	t.link = l;
	t.layer = vp->s.layer;
	t.w = vp->w;
	add_track(rt, t, sg);
	if (rt->all->verbose) {
		(void)fprintf(stderr, "%s: via of %s.%s (%s) at %.3f %.3f\n",
			      rt->b.net[n].name, vp->q->name,
			      vp->q->fp->pad[vp->e->pad].name, pcb_ref(vp->q),
			      at[0], at[1]);
	}
}

// Gives pad e of plane net n its via, or on the plane's own layer its
// neck if it needs one; returns 0 if there is no room for it.
static int pad_via(struct router *rt, int n, const struct pcb_end *e)
{
	struct via_pad vp;
	const char *who = "";
	char *key = NULL;
	double at[2] = { 0, 0 };
	int ok = 1;

	vp.e = e;
	vp.q = &rt->b.part[e->part];
	vp.net = n;
	if (!vp.q->placed) {
		return 1;
	}
	pcb_spot(&rt->b, vp.q, e->pad, &vp.s);
	if (vp.s.layer < 0) {
		return 1; // A hole is on the plane already.
	}
	key = pcb_pad_key(&rt->b, e->part, e->pad);
	vp.w = pcb_neck_width(&rt->b, vp.q, n);
	if (has_link(rt, n, key)) {
		free(key);
		return 1; // Kept.
	}
	if (vp.s.layer == rt->b.net[n].plane) {
		if (pcb_relieved(&rt->b, e->part, e->pad)) {
			struct pcb_track t = { 0 };

			t.net = n;
			t.link = pcb_add_link(&rt->b, n, key, "plane", vp.w);
			t.w = vp.w;
			neck(rt, t, vp.q, &vp.s);
		}
	} else if (!holed(&vp)) {
		vp.want = via_way(rt, &vp);
		if (via_spot(rt, &vp, at, &who)) {
			add_via(rt, &vp, key, at);
		} else {
			pcb_error(&rt->b, 0,
				  "net %s: no room for the via of %s.%s (%s); "
				  "in the way: %s",
				  rt->b.net[n].name, vp.q->name,
				  vp.q->fp->pad[e->pad].name, pcb_ref(vp.q),
				  who);
			ok = 0;
		}
	}
	free(key);
	return ok;
}

// Gives each pad of plane net n a via, on a neck track: the nearest
// place clear of all else, the via gap from the pads of its net and the
// mask web from every opening, out the way the source says or, else,
// away from the middle of its part.  A pad on its plane's own layer
// needs none: it is in the pour, or, a pad of a small part of two pins,
// joined to it by a neck.
static int vias(struct router *rt, int n)
{
	int ok = 1;

	for (size_t j = 0; j < rt->b.net[n].nend; j++) {
		ok = pad_via(rt, n, &rt->b.net[n].end[j]) && ok;
	}
	return ok;
}

// Takes the copper of connection l off the board, or, l -1, the copper
// of net n of no connection; a connection taken off is of no net.
static void rip(struct router *rt, int n, long l)
{
	size_t k = 0;

	for (size_t i = 0; i < rt->b.ntrack; i++) {
		if (rt->b.track_[i].net == n && rt->b.track_[i].link == l) {
			free(rt->b.track_[i].xy);
		} else {
			rt->b.track_[k++] = rt->b.track_[i];
		}
	}
	rt->b.ntrack = k;
	k = 0;
	for (size_t i = 0; i < rt->b.nvia; i++) {
		if (rt->b.via[i].net != n || rt->b.via[i].link != l) {
			rt->b.via[k++] = rt->b.via[i];
		}
	}
	rt->b.nvia = k;
	if (l >= 0) {
		rt->b.link[l].net = -1;
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
static long part_of_key(struct router *rt, int n, const char *key)
{
	for (size_t j = 0; j < rt->b.net[n].nend; j++) {
		char *k = pcb_pad_key(&rt->b, rt->b.net[n].end[j].part,
				      rt->b.net[n].end[j].pad);
		int same = !strcmp(k, key);

		free(k);
		if (same) {
			return (long)rt->b.net[n].end[j].part;
		}
	}
	return -1;
}

// Whether the copper of connection l keeps clear of the pads of other
// nets as they now stand.
static int link_clear(struct router *rt, long l)
{
	int n = rt->b.link[l].net;
	const char *who = NULL;

	for (size_t i = 0; i < rt->b.ntrack; i++) {
		const struct pcb_track *t = &rt->b.track_[i];

		for (size_t j = 1; t->link == l && j < t->n; j++) {
			double len =
				hypot(t->xy[2 * j] - t->xy[(2 * j) - 2],
				      t->xy[(2 * j) + 1] - t->xy[(2 * j) - 1]);

			int nf = steps_upto(len, CELL);

			for (int k = 0; k < nf; k++) {
				double f = k * CELL;
				double u = len > 0 ? f / len : 0;
				double x =
					t->xy[(2 * j) - 2] +
					((t->xy[2 * j] - t->xy[(2 * j) - 2]) *
					 u);
				double y = t->xy[(2 * j) - 1] +
					   ((t->xy[(2 * j) + 1] -
					     t->xy[(2 * j) - 1]) *
					    u);

				struct disk_on d = { x, y, (t->w / 2) - SAME,
						     t->layer, n };

				if (!pads_clear(rt, &d, &who)) {
					return 0;
				}
			}
		}
	}
	for (size_t i = 0; i < rt->b.nvia; i++) {
		const struct pcb_via *v = &rt->b.via[i];
		struct disk_on d = { v->x, v->y, (v->dia / 2) - SAME, -1, n };

		if (v->link == l && !pads_clear(rt, &d, &who)) {
			return 0;
		}
	}
	return 1;
}

// Whether the copper of connection l keeps the edge rule from the
// outline and the cutouts, as pcbcheck(1) measures it; a group laid out
// alone has no outline.
static int link_off_edge(struct router *rt, long l)
{
	for (size_t i = 0; !rt->scope && i < rt->b.ntrack; i++) {
		const struct pcb_track *t = &rt->b.track_[i];

		for (size_t j = 1; t->link == l && j < t->n; j++) {
			if (pcb_edge_dist(&rt->b, 0, &t->xy[(2 * j) - 2],
					  t->w / 2, NULL) < rt->b.edge - SAME) {
				return 0;
			}
		}
	}
	for (size_t i = 0; !rt->scope && i < rt->b.nvia; i++) {
		const struct pcb_via *v = &rt->b.via[i];

		if (v->link == l &&
		    pcb_edge_dist(&rt->b, 0,
				  (const double[4]){ v->x, v->y, v->x, v->y },
				  v->dia / 2, NULL) < rt->b.edge - SAME) {
			return 0;
		}
	}
	return 1;
}

// Whether connection l of net n may be kept: its pads where they stood
// (now, as the lock tells them), its width what the statements give now,
// and its copper clear of the pads as they stand.
static int keep_link(struct router *rt, long l, char **now, size_t nnow)
{
	const struct pcb_link *k = &rt->b.link[l];
	int n = k->net;
	int to_pad = strcmp(k->b, "via") != 0 && strcmp(k->b, "plane") != 0;
	const char *a0 = pad_line(rt->b.netpad[n], rt->b.nnetpad[n], k->a);
	const char *a1 = pad_line(now, nnow, k->a);
	long q = part_of_key(rt, n, k->a);
	double w = NAN;

	if (!a0 || !a1 || strcmp(a0, a1) != 0 || q < 0) {
		return 0;
	}
	if (to_pad) {
		const char *b0 =
			pad_line(rt->b.netpad[n], rt->b.nnetpad[n], k->b);
		const char *b1 = pad_line(now, nnow, k->b);

		if (!b0 || !b1 || strcmp(b0, b1) != 0) {
			return 0;
		}
		w = pcb_link_width(&rt->b, n, k->a, k->b);
	} else {
		w = pcb_neck_width(&rt->b, &rt->b.part[q], n);
	}
	return fabs(w - k->w) < SAME && link_clear(rt, l);
}

// The size of a net, to order them: its pads and their spread.
static double size_of(const struct router *rt, int n)
{
	double x0 = BIG;
	double y0 = BIG;
	double x1 = -BIG;
	double y1 = -BIG;

	for (size_t j = 0; j < rt->b.net[n].nend; j++) {
		struct pcb_spot s;

		pcb_spot(&rt->b, &rt->b.part[rt->b.net[n].end[j].part],
			 rt->b.net[n].end[j].pad, &s);
		x0 = fmin(x0, s.x);
		y0 = fmin(y0, s.y);
		x1 = fmax(x1, s.x);
		y1 = fmax(y1, s.y);
	}
	return (x1 - x0) + (y1 - y0) + (double)rt->b.net[n].nend;
}

// Whether block statement s is in scope and held by no other block.
static int top_block(const struct router *rt, size_t s)
{
	const struct pcb_stmt *st = &rt->b.stmt[s];

	if (st->n != 2 || strcmp(st->w[0], "block") != 0 ||
	    (rt->scope && !of_group(st->w[1], rt->scope))) {
		return 0;
	}
	for (size_t o = 0; o < rt->b.nstmt; o++) {
		const struct pcb_stmt *ot = &rt->b.stmt[o];

		if (o != s && !strcmp(ot->w[0], "block") && ot->n == 2 &&
		    of_group(st->w[1], ot->w[1]) &&
		    (!rt->scope || of_group(ot->w[1], rt->scope))) {
			return 0;
		}
	}
	return 1;
}

// Takes the connections before first that the lock had of the block of
// the group off the board.
static void rip_block(struct router *rt, const char *group, size_t first)
{
	for (size_t l = 0; l < first; l++) {
		long qa = rt->b.link[l].net >= 0 ?
				  part_of_key(rt, rt->b.link[l].net,
					      rt->b.link[l].a) :
				  -1;

		if (qa >= 0 && of_group(rt->b.part[qa].name, group)) {
			rip(rt, rt->b.link[l].net, (long)l);
		}
	}
}

// Takes in the copper of connection l of the block's lock lk, as
// connection nl, moved by mv.
static void copy_link(struct router *rt, const struct pcb *lk, size_t l,
		      struct pcb_pt mv)
{
	long nl = pcb_add_link(&rt->b, lk->link[l].net, lk->link[l].a,
			       lk->link[l].b, lk->link[l].w);

	for (size_t i = 0; i < lk->ntrack; i++) {
		struct pcb_track t = lk->track_[i];

		if (t.link != (int)l) {
			continue;
		}
		t.link = (int)nl;
		t.xy = pcb_alloc(2 * t.n * sizeof *t.xy);
		for (size_t j = 0; j < t.n; j++) {
			t.xy[2 * j] = lk->track_[i].xy[2 * j] + mv.x;
			t.xy[(2 * j) + 1] =
				lk->track_[i].xy[(2 * j) + 1] + mv.y;
		}
		rt->b.track_ = pcb_grow(rt->b.track_, rt->b.ntrack,
					sizeof *rt->b.track_);
		rt->b.track_[rt->b.ntrack++] = t;
	}
	for (size_t i = 0; i < lk->nvia; i++) {
		struct pcb_via v = lk->via[i];

		if (v.link != (int)l) {
			continue;
		}
		v.link = (int)nl;
		v.x += mv.x;
		v.y += mv.y;
		rt->b.via = pcb_grow(rt->b.via, rt->b.nvia, sizeof *rt->b.via);
		rt->b.via[rt->b.nvia++] = v;
	}
}

// Takes in the copper of the block of statement st, before first in
// the lock; its own lock read from its section.
static void take_block(struct router *rt, const struct pcb_stmt *st,
		       size_t first)
{
	struct pcb lk;
	char *text = pcb_section(&rt->b, st->w[1]);
	FILE *f = text ? fmemopen(text, strlen(text), "r") : NULL;
	long r = -1;
	struct pcb_pt mv = { NAN, NAN };

	if (!f) {
		pcb_error(&rt->b, st->line,
			  "block %s: no section of it in the lock; lay it "
			  "out first",
			  st->w[1]);
		return;
	}
	pcb_read(&lk, rt->b.file, rt->lib, f, NULL);
	(void)fclose(f);
	for (size_t q = 0; q < rt->b.npart && r < 0; q++) {
		if (of_group(rt->b.part[q].name, st->w[1]) &&
		    rt->b.part[q].placed && lk.part[q].placed) {
			r = (long)q;
		}
	}
	if (r < 0) {
		pcb_free(&lk);
		return;
	}
	mv.x = round((rt->b.part[r].x - lk.part[r].x) * LOCK_UNITS) /
	       LOCK_UNITS;
	mv.y = round((rt->b.part[r].y - lk.part[r].y) * LOCK_UNITS) /
	       LOCK_UNITS;
	// What the lock had of the block's copper goes.
	rip_block(rt, st->w[1], first);
	for (size_t l = 0; l < lk.nlink; l++) {
		if (lk.link[l].net >= 0) {
			copy_link(rt, &lk, l, mv);
		}
	}
	if (rt->all->verbose) {
		(void)fprintf(stderr,
			      "%s: the copper of block %s, moved %.4f %.4f\n",
			      rt->b.file, st->w[1], mv.x, mv.y);
	}
	pcb_free(&lk);
}

// Takes in the copper of each block the layout is made of (the groups
// of block statements in scope that no other holds): its lock's, moved
// as pcbplace moved its parts (by its first part placed), in place of
// what the lock had of it; kept as it is.  Returns the first connection
// of the blocks': they are all those from it on.
static size_t block_copper(struct router *rt)
{
	size_t first = rt->b.nlink;

	for (size_t s = 0; s < rt->b.nstmt; s++) {
		if (top_block(rt, s)) {
			take_block(rt, &rt->b.stmt[s], first);
		}
	}
	return first;
}

// One pass: the copper of the group (the board, for NULL), from its
// section of the lock read from in; the whole lock written to out.
// Which connections keep their copper, into kept: of a net made by the
// same statements (not fresh), their pads where they were, their widths
// as they were and their copper clear of the pads; the connections from
// ours on, a block's, kept as its lock has them.
static void keep_links(struct router *rt, const int *fresh, size_t ours)
{
	for (size_t l = 0; l < rt->b.nlink; l++) {
		int k = rt->b.link[l].net;
		size_t np = 0;
		char **now = NULL;

		if (k < 0) {
			continue;
		}
		now = pcb_net_pads(&rt->b, (size_t)k, &np);
		// A block's copper is kept as its lock has it, but for what
		// breaks the edge rule, which a group alone does not know; a
		// net that waits keeps none.
		rt->kept[l] =
			!net_waits(rt, (size_t)k) &&
			(l >= ours || (!fresh[k] && now &&
				       keep_link(rt, (long)l, now, np))) &&
			link_off_edge(rt, (long)l);
		for (size_t j = 0; now && j < np; j++) {
			free(now[j]);
		}
		free((void *)now);
	}
}

static void keep_forks(struct router *rt)
{
	// A fork's first connection, to the middle of its second, is kept
	// only with it.
	for (size_t k = 0; k < rt->b.nnet; k++) {
		for (size_t s = 0; s < rt->b.nstmt; s++) {
			const struct pcb_stmt *st = &rt->b.stmt[s];
			char *kp = NULL;
			char *ka = NULL;
			char *kb = NULL;
			long stem = 0;
			long trunk = 0;

			if (!is_fork(rt, st, (int)k)) {
				continue;
			}
			kp = pin_key(rt, st->w[2]);
			ka = pin_key(rt, st->w[FORK_WORDS - 2]);
			kb = pin_key(rt, st->w[FORK_WORDS - 1]);
			stem = kp && ka ? link_of(rt, (int)k, kp, ka) : -1;
			trunk = ka && kb ? link_of(rt, (int)k, ka, kb) : -1;
			if (stem >= 0 && (trunk < 0 || !rt->kept[trunk])) {
				rt->kept[stem] = 0;
			}
			free(kp);
			free(ka);
			free(kb);
		}
	}
}

// Rips the copper not kept, and the copper of no connection.
static void rip_fresh(struct router *rt)
{
	for (size_t l = 0; l < rt->b.nlink; l++) {
		if (!rt->kept[l] && rt->b.link[l].net >= 0) {
			rip(rt, rt->b.link[l].net, (long)l);
		}
	}
	// Copper of no connection (of a lock not made here, or by hand) is
	// made anew, by connection, for every net routed here.
	for (size_t k = 0; k < rt->b.nnet; k++) {
		if (net_in_scope(rt, k)) {
			rip(rt, (int)k, -1);
		}
	}
}

// The nets in the order routed, the fewest and nearest pads first.
static void net_order(const struct router *rt, int *order)
{
	for (size_t k = 0; k < rt->b.nnet; k++) {
		order[k] = (int)k;
	}
	for (size_t a = 1; a < rt->b.nnet; a++) {
		for (size_t b = a;
		     b > 0 && size_of(rt, order[b]) < size_of(rt, order[b - 1]);
		     b--) {
			int t = order[b];

			order[b] = order[b - 1];
			order[b - 1] = t;
		}
	}
}

// Routes the nets in order; a net that finds no way is put first and
// all that was made anew is made again, TRIES times at most.
// Takes what was made anew off the board, to be made again in a new
// order.
static void rip_made(struct router *rt)
{
	for (size_t l = 0; l < rt->b.nlink; l++) {
		if ((l >= rt->nkept || !rt->kept[l]) &&
		    rt->b.link[l].net >= 0 &&
		    rt->b.net[rt->b.link[l].net].plane < 0) {
			rip(rt, rt->b.link[l].net, (long)l);
		}
	}
}

// Routes the signal nets routed here in order; returns the first that
// finds no way, and whom from in *who, or -1.
static int route_order(struct router *rt, const int *order, const char **who)
{
	for (size_t o = 0; o < rt->b.nnet; o++) {
		int k = order[o];

		if (rt->b.net[k].plane >= 0 || net_waits(rt, (size_t)k) ||
		    !net_in_scope(rt, (size_t)k)) {
			continue;
		}
		if (!route_net(rt, k, who)) {
			return k;
		}
	}
	return -1;
}

static void route_all(struct router *rt, int *order)
{
	for (int tries = 0; tries < TRIES; tries++) {
		const char *who = "";
		int failed = -1;

		if (tries) {
			rip_made(rt);
		}
		failed = route_order(rt, order, &who);
		if (failed < 0) {
			break;
		}
		if (tries == TRIES - 1) {
			pcb_error(&rt->b, 0, "net %s: no way from %s",
				  rt->b.net[failed].name, who);
			break;
		}
		for (size_t o = 0; o < rt->b.nnet; o++) {
			if (order[o] == failed) {
				memmove(order + 1, order, o * sizeof *order);
				order[0] = failed;
				break;
			}
		}
	}
}

// The vias of the plane nets, in order.
static void plane_vias(struct router *rt, const int *order)
{
	for (size_t o = 0; o < rt->b.nnet; o++) {
		int k = order[o];

		if (rt->b.net[k].plane >= 0) {
			vias(rt, k);
		}
	}
}

// Tells in the lock what the copper of each net is made from now, words,
// for the nets with a pad here.
static void tell_lock(struct router *rt, char **words)
{
	// The lock tells what the copper is made from now, for the nets
	// with a pad here.
	for (size_t k = 0; k < rt->b.nnet; k++) {
		for (size_t j = 0; j < rt->b.nnetpad[k]; j++) {
			free(rt->b.netpad[k][j]);
		}
		free((void *)rt->b.netpad[k]);
		free(rt->b.netby[k]);
		rt->b.netby[k] = net_here(rt, k) ? words[k] : NULL;
		if (!net_here(rt, k)) {
			free(words[k]);
		}
		rt->b.netpad[k] = pcb_net_pads(&rt->b, k, &rt->b.nnetpad[k]);
	}
}

// Frees the net of each pad.
static void free_padnet(struct router *rt)
{
	for (size_t q = 0; q < rt->b.npart; q++) {
		free(rt->padnet[q]);
	}
	free((void *)rt->padnet);
	rt->padnet = NULL;
}

static int pass(const struct pcb_run *r)
{
	struct router *rt = pcb_alloc(sizeof *rt);
	char **words = NULL;
	size_t nfresh = 0;
	size_t ours = 0;
	int *fresh = NULL;
	int *order = NULL;
	int status = 0;

	rt->all = r->ctx;
	rt->lib = r->lib;
	rt->scope = r->group;
	pcb_read(&rt->b, r->file, r->lib, r->in, rt->scope);
	rt->padnet = (int **)pcb_alloc((rt->b.npart + 1) * sizeof *rt->padnet);
	ours = block_copper(rt);
	for (size_t q = 0; q < rt->b.npart; q++) {
		size_t np = rt->b.part[q].fp ? rt->b.part[q].fp->npad : 0;

		rt->padnet[q] = pcb_alloc((np + 1) * sizeof **rt->padnet);
		for (size_t k = 0; k < np; k++) {
			rt->padnet[q][k] = pcb_net_of(&rt->b, q, k);
		}
	}
	fresh = pcb_alloc((rt->b.nnet + 1) * sizeof *fresh);
	order = pcb_alloc((rt->b.nnet + 1) * sizeof *order);
	words = (char **)pcb_alloc((rt->b.nnet + 1) * sizeof *words);
	for (size_t k = 0; k < rt->b.nnet; k++) {
		words[k] = pcb_net_words(&rt->b, k);
		fresh[k] = !rt->b.netby[k] ||
			   strcmp(rt->b.netby[k], words[k]) != 0;
	}
	rt->nkept = rt->b.nlink;
	rt->kept = pcb_alloc((rt->nkept + 1) * sizeof *rt->kept);
	keep_links(rt, fresh, ours);
	keep_forks(rt);
	rip_fresh(rt);
	net_order(rt, order);
	route_all(rt, order);
	plane_vias(rt, order);
	for (size_t l = rt->nkept; l < rt->b.nlink; l++) {
		nfresh += rt->b.link[l].net >= 0;
	}
	tell_lock(rt, words);
	if (rt->all->verbose) {
		(void)fprintf(stderr, "%s: %zu connections made anew\n",
			      rt->b.file, nfresh);
	}
	pcb_write_lock(&rt->b, r->out);
	status = rt->b.failed;
	free_padnet(rt);
	free(fresh);
	free(order);
	free((void *)words);
	free(rt->kept);
	pcb_free(&rt->b);
	free(rt);
	return status;
}

int main(int argc, char **argv)
{
	struct pcb_lib lib = { NULL, 0, NULL, 0 };
	struct across all = { 0 };
	struct pcb_run r = { NULL, &lib, NULL, NULL, stdout, &all };
	int i = 1;
	const char *usage = "usage: pcbroute [-v] [-g GROUP] [-L LIB]... "
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
	return failed;
}
