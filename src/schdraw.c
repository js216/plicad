// SPDX-License-Identifier: GPL-3.0-only
// schdraw.c -- Draw a schematic sheet as plain vectors
// Copyright (c) 2026 Jakob Kastelic

// Usage: schdraw [-v] [-L LIB]... sheet.sch > sheet.vec
//
// Reads the source (sch(5)), page by page, which the reader places, and
// the symbols it names from the project (symbols and sheets next to the
// source) and the library directories (LIB/symbols, LIB/sheets; by
// default ../lib next to the program); routes the wires; and writes the
// pages as vec(5), each from its page record: the template of the sheet,
// each net's wires and dots, and each part's symbol and text, the parts
// and nets named, the text as text, and where each pin and flag meets
// its wire, so a test (schconn) can tell from the drawing alone that it
// joins what the netlist joins.  Each port of the sheet names the other
// pages it is on, as BUF(2,5).  vecps prints it.
//
// The wires run on the grid of the source, 0.1 in at scale 1.  The pins
// of a net are wired in groups: each pin with a flag of its own and the
// flag are a group, and the other pins of the net are one group with
// the first of them.  A group is routed as a tree: first the wires the
// source names, then the two pins that face each other on one line
// farthest apart, so the line of the signal is straight; then each
// further pin, the nearest first, takes the cheapest path to the tree,
// by Dijkstra's method over the points of the grid and the way a wire
// comes into each.  A path leaves a pin and comes into one along the
// way the pin points; it runs through no body and no other pin, along
// no wire of another group, and may cross one at right angles; it joins
// its own tree where that makes a T and nowhere else, so there are no
// four-way junctions and no junctions on pins.  Bends, crossings, running
// next to a body or a foreign wire, a junction next to another or next
// to a pin and, for
// a wire the source gives a side, straying off that side cost more than
// length.  The smallest groups are routed first.  A group that finds no
// way is put first and all are routed again (rip-up); once all are
// routed, each is ripped up and routed again among the others, and its
// new path kept if it costs no more.
//
// Blocks that nothing joins, side by side across the sheet or one over
// another, are spread so the margins and the gaps between them come
// out alike (spread).
//
// The room the drawing keeps, round bodies and text, is the sheet
// template's (its clear records); each pair of things too close is
// printed, and the exit status is 1.  With -v each placement is told on
// stderr, with its line and the rule it came from.

#include "schsrc.h"
#include <limits.h>
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GRID SCH_GRID
#define LW SCH_LW
#define PI 3.14159265358979323846
#define DOT_R 1.64 // The radius of a junction dot.
#define TEXT SCH_TEXT
#define LEAD SCH_LEAD
#define DESCENT SCH_DESCENT
#define PINNUM 3.25 // The pin numbers of an integrated circuit.
#define NOTE_BOX 10.0 // The side of the box of a note, in points of the page.
#define FLAG_TEXT_OFF 2.5 // The text of a flag beyond its end, in points,
#define PORT_TIP 1.7 // and the tip of a port before it,
#define GLOBAL_TIP 1.2 // and of a global.
#define PINNUM_IN \
	0.45 // A pin number in from the end of its pin, in grid units,
#define PINNUM_UP 1.2 // above a pin across, in points,
#define PINNUM_RIGHT 1.5 // right of one up or down,
#define PINNUM_PAD 0.5 // and the room round it,
#define PINNUM_RIGHT_W 3.5 // to its right.
#define COVER_ANY 10.0 // What any overlap costs, beyond its area.
#define BEST_NONE 1e300 // Worse than any place.
#define LABEL_OUT 3.0 // The steps a label moves out from its side,
#define LABEL_SLIDE 1.5 // and slides along it, in points,
#define LABEL_ABOVE 1.5 // and the room above a part, past the gap.
#define CLASH_RULE 1e6 // The cost of a place too near to something,
#define SIDE_COST 8.0 // of each side past the label's own,
#define SLIDE_COST 0.4 // and of a point of slide.
#define STEP_SLACK 1e-9 // What a count of steps lets go of rounding.
#define FAR_GRID 1e9 // Beyond any page, in grid units.
#define NC_R 1.8 // Half the cross of a pin left unconnected, in points.
#define ROOM_SLACK 0.01 // What the room the drawing keeps lets go, in points.
enum {
	MARGIN = 8, // Grid units round everything, for the router.
	LABEL_OUTS = 4, // The steps a label moves out.
	RUNS_MOST = 256, // The most runs of the text of a flag,
	PIECE_LEN = 8, // and room for a piece of its list of pages.
	LINE = 4096, // The longest line of a sheet.
	TEXT_NUMBERS = 4, // The numbers of a text record but its alignment.
	DECIMAL = 10,
	CIRCLE_STEPS = 128, // The pieces of a circle.
	PAGE_LEN = 16, // Room for a page number.
	NOTE_STEPS = 400, // The most steps a note box goes out.
	SPREAD_MIN = 2 // The least gap between blocks spread, in grid units.
};

enum {
	STEP = 10,
	BEND = 30,
	CROSS = 20,
	NEAR = 4,
	CROWD = 60,
	SIDE = 40,
	TRIES = 40,
	PASSES = 2
};
enum {
	WAYS_X = 0x5, // The ways of a wire along x: right and left,
	WAYS_Y = 0xA // and along y: up and down.
};

static const int way_dx[4] = { 1, 0, -1, 0 };
static const int way_dy[4] = { 0, 1, 0, -1 };

// A run of text in the stroke font: its size, the capital height.
struct run {
	char *s;
	double size;
};

// A block of text: lines of runs, placed by its box in points, and the
// side of its part it stands on (its lines align to the part).
struct label {
	struct run run[3][4];
	int nrun[3], nline, side;
	double w, h;
	double x, y; // The lower left corner of the box.
};

// The polylines of the drawing, in grid units, with their nets.
struct poly {
	int *xy;
	size_t n;
	size_t net;
};

// A group of pins of a net that are wired together, with the edges of
// the grid its wires take and what they cost.
struct group {
	size_t net, *end, nend;
	int *edge, cost;
	size_t nedge;
};

// The drawing under way.
struct draw {
	struct sch sch; // The page drawn,
	struct sch *pages; // and all pages read.
	struct poly *polys;
	size_t npolys;
	struct label *labels; // One for each part, if it has one.
	int failed;
	int verbose; // Whether -v tells the routes too.
	// The grid of the router: the bodies, the pins (the net of each, -1
	// none, -2 a pin on no net) and the edges (the group whose wire
	// takes each, -1 none): edge 2a runs right from point a, edge 2a+1
	// up.
	int x0, y0, w, h;
	unsigned char *blocked;
	int *pinnet;
	int *eown;
	struct group *groups;
	size_t ngroups;
	// The heap of Dijkstra's method: states are point * 4 + the way in.
	int *heap;
	int *cost;
	int *from;
	size_t nheap;
	// The transform of the page: the point of the drawing at the origin
	// of the grid, and the scale.
	double ox, oy, sc;
	// The note boxes placed so far, in points of the drawing.
	double (*note_box)[4];
	size_t nnote_box;
};

static void *xrealloc(void *p, size_t n)
{
	void *q = realloc(p, n ? n : 1);

	if (!q) {
		sch_die("out of memory");
	}
	return q;
}

static int idx(const struct draw *dr, int x, int y)
{
	return ((y - dr->y0) * dr->w) + (x - dr->x0);
}

static int inside(const struct draw *dr, int x, int y)
{
	return x >= dr->x0 && x < dr->x0 + dr->w && y >= dr->y0 &&
	       y < dr->y0 + dr->h;
}

// The net of pin p of part i, or -1.
static int net_of(struct draw *dr, size_t i, size_t p)
{
	for (size_t k = 0; k < dr->sch.nnet; k++) {
		for (size_t j = 0; j < dr->sch.net[k].nend; j++) {
			if (dr->sch.net[k].end[j].part == i &&
			    dr->sch.net[k].end[j].pin == p) {
				return (int)k;
			}
		}
	}
	return -1;
}

// Adds a copy of the polyline o.
static void add_poly(struct draw *dr, const struct poly *o)
{
	struct poly p = *o;

	p.xy = sch_alloc(2 * p.n * sizeof *p.xy);
	memcpy(p.xy, o->xy, 2 * p.n * sizeof *o->xy);
	dr->polys = xrealloc(dr->polys, (dr->npolys + 1) * sizeof *dr->polys);
	dr->polys[dr->npolys++] = p;
}

// The number of steps from 0 by step to len, len itself if it is one
// (to within STEP_SLACK of a step).
static int steps_upto(double len, double step)
{
	return (int)floor((len / step) + STEP_SLACK) + 1;
}

// The grid lines at or under, and at or over, a coordinate.
static int lo(double v)
{
	double f = floor(v);

	return (int)f;
}

static int hi(double v)
{
	double c = ceil(v);

	return (int)c;
}

// The extent of the parts and their pins, in grid units: x0, y0, x1, y1
// into e.
static void parts_extent(const struct draw *dr, int e[4])
{
	e[0] = e[1] = INT_MAX;
	e[2] = e[3] = INT_MIN;
	for (size_t i = 0; i < dr->sch.npart; i++) {
		double b[4];

		sch_box(&dr->sch.part[i], b);
		e[0] = e[0] < lo(b[0]) ? e[0] : lo(b[0]);
		e[1] = e[1] < lo(b[1]) ? e[1] : lo(b[1]);
		e[2] = e[2] > hi(b[2]) ? e[2] : hi(b[2]);
		e[3] = e[3] > hi(b[3]) ? e[3] : hi(b[3]);
		for (size_t p = 0; p < dr->sch.part[i].dev->shape->npin; p++) {
			struct sch_at pin_at = sch_pin_at(&dr->sch.part[i], p);
			int x = pin_at.x;
			int y = pin_at.y;

			e[0] = x < e[0] ? x : e[0];
			e[1] = y < e[1] ? y : e[1];
			e[2] = x > e[2] ? x : e[2];
			e[3] = y > e[3] ? y : e[3];
		}
	}
}

// Blocks the points of the grid in the body of part i and the room
// round it: no place for a wire.
static void block_body(struct draw *dr, size_t i)
{
	double b[4];
	double m = dr->sch.part[i].dev->shape->flag ? 0 :
						      dr->sch.clear_body / GRID;

	sch_box(&dr->sch.part[i], b);
	for (int y = hi(b[1] - m); y <= lo(b[3] + m); y++) {
		for (int x = hi(b[0] - m); x <= lo(b[2] + m); x++) {
			if (x > b[0] - m && x < b[2] + m && y > b[1] - m &&
			    y < b[3] + m) {
				dr->blocked[idx(dr, x, y)] = 1;
			}
		}
	}
}

// Marks the pins of part i on the grid, by their nets.
static void mark_pins(struct draw *dr, size_t i)
{
	for (size_t p = 0; p < dr->sch.part[i].dev->shape->npin; p++) {
		struct sch_at pin_at = sch_pin_at(&dr->sch.part[i], p);
		int n = net_of(dr, i, p);
		int a = idx(dr, pin_at.x, pin_at.y);

		if (dr->pinnet[a] != -1) {
			sch_die("%s:%d: two pins meet at %d %d", dr->sch.file,
				dr->sch.part[i].line, pin_at.x, pin_at.y);
		}
		dr->pinnet[a] = n < 0 ? -2 : n;
		dr->blocked[a] = 0;
	}
}

// Sets up the grid: its extent, the bodies and the pins.
static void grid(struct draw *dr)
{
	int e[4];
	size_t n = 0;

	if (!dr->sch.npart) {
		sch_die("%s: nothing to draw", dr->sch.file);
	}
	parts_extent(dr, e);
	dr->x0 = e[0] - MARGIN;
	dr->y0 = e[1] - MARGIN;
	dr->w = e[2] - e[0] + 2 * MARGIN + 1;
	dr->h = e[3] - e[1] + 2 * MARGIN + 1;
	n = (size_t)dr->w * (size_t)dr->h;
	dr->blocked = sch_alloc(n);
	dr->pinnet = sch_alloc(n * sizeof *dr->pinnet);
	dr->eown = sch_alloc(2 * n * sizeof *dr->eown);
	for (size_t i = 0; i < n; i++) {
		dr->pinnet[i] = -1;
		dr->eown[2 * i] = dr->eown[(2 * i) + 1] = -1;
	}
	for (size_t i = 0; i < dr->sch.npart; i++) {
		block_body(dr, i);
	}
	for (size_t i = 0; i < dr->sch.npart; i++) {
		mark_pins(dr, i);
	}
}

// A point of the grid.
struct gpt {
	int x, y;
};

// A step of the grid: from point a, the way d.
struct gstep {
	int a, d;
};

// The edge of step st, or -1 off the grid.
static int edge(const struct draw *dr, struct gstep st)
{
	int a = st.a;
	int x = a % dr->w;
	int y = a / dr->w;

	switch (st.d) {
	case SCH_R:
		return x + 1 < dr->w ? 2 * a : -1;
	case SCH_U:
		return y + 1 < dr->h ? (2 * a) + 1 : -1;
	case SCH_L:
		return x > 0 ? 2 * (a - 1) : -1;
	default:
		return y > 0 ? (2 * (a - dr->w)) + 1 : -1;
	}
}

// The bit of the way d among the ways of a point.
static unsigned way_bit(int d)
{
	return 1U << (unsigned)d;
}

// The ways of a wire straight across the way d.
static unsigned across(int d)
{
	return d % 2 ? (unsigned)WAYS_X : (unsigned)WAYS_Y;
}

// The ways from point a whose edges group gr takes (NULL: any group).
static unsigned ways(const struct draw *dr, int a, const struct group *gr)
{
	unsigned m = 0;

	for (int d = 0; d < 4; d++) {
		int e = edge(dr, (struct gstep){ a, d });

		if (e >= 0 && dr->eown[e] >= 0 &&
		    (!gr || dr->eown[e] == (int)(gr - dr->groups))) {
			m |= way_bit(d);
		}
	}
	return m;
}

static int bits(unsigned m)
{
	int n = 0;

	for (; m; m >>= 1U) {
		n += (int)(m & 1U);
	}
	return n;
}

static void push(struct draw *dr, int s)
{
	size_t i = dr->nheap++;

	dr->heap[i] = s;
	while (i > 0) {
		size_t up = (i - 1) / 2;
		int t = 0;

		if (dr->cost[dr->heap[up]] <= dr->cost[dr->heap[i]]) {
			break;
		}
		t = dr->heap[up];
		dr->heap[up] = dr->heap[i];
		dr->heap[i] = t;
		i = up;
	}
}

static int pop(struct draw *dr)
{
	int top = dr->heap[0];
	size_t i = 0;

	dr->heap[0] = dr->heap[--dr->nheap];
	for (;;) {
		size_t l = (2 * i) + 1;
		size_t r = l + 1;
		size_t m = i;
		int t = 0;

		if (l < dr->nheap &&
		    dr->cost[dr->heap[l]] < dr->cost[dr->heap[m]]) {
			m = l;
		}
		if (r < dr->nheap &&
		    dr->cost[dr->heap[r]] < dr->cost[dr->heap[m]]) {
			m = r;
		}
		if (m == i) {
			return top;
		}
		t = dr->heap[m];
		dr->heap[m] = dr->heap[i];
		dr->heap[i] = t;
		i = m;
	}
}

// The extra cost of a point for group gr: next to a body or another
// group's wire.
static int near_cost(const struct draw *dr, struct gpt p,
		     const struct group *gr)
{
	int c = 0;

	for (int d = 0; d < 4; d++) {
		int nx = p.x + way_dx[d];
		int ny = p.y + way_dy[d];
		int a = 0;

		if (!inside(dr, nx, ny)) {
			continue;
		}
		a = idx(dr, nx, ny);
		if (dr->blocked[a] || (ways(dr, a, NULL) & ~ways(dr, a, gr))) {
			c += NEAR;
		}
	}
	return c;
}

// Whether a junction, or a point that would become one, stands next to
// p: two dots side by side read as a smudge.
static int next_to_dot(const struct draw *dr, struct gpt p)
{
	for (int d = 0; d < 4; d++) {
		int nx = p.x + way_dx[d];
		int ny = p.y + way_dy[d];

		if (inside(dr, nx, ny) &&
		    bits(ways(dr, idx(dr, nx, ny), NULL)) >= 3) {
			return 1;
		}
	}
	return 0;
}

// Whether a pin stands next to p: a junction there would sit on its
// lead, as if the branch came from the pin.
static int next_to_pin(const struct draw *dr, struct gpt p)
{
	for (int d = 0; d < 4; d++) {
		int nx = p.x + way_dx[d];
		int ny = p.y + way_dy[d];

		if (inside(dr, nx, ny) && dr->pinnet[idx(dr, nx, ny)] != -1) {
			return 1;
		}
	}
	return 0;
}

// A side a path keeps to: the way, and the two pins it runs between.
struct keep {
	int side, x[2], y[2];
};

// The cost of straying off the side: a point past the line of the pins
// on the wrong side, but for the points next to the pins.
static int stray(const struct keep *k, struct gpt p)
{
	int near = 0;
	int off = 0;
	int x = p.x;
	int y = p.y;

	if (!k || k->side < 0) {
		return 0;
	}
	for (int i = 0; i < 2; i++) {
		near = near || abs(x - k->x[i]) + abs(y - k->y[i]) <= 1;
	}
	switch (k->side) {
	case SCH_U:
		off = y < (k->y[0] > k->y[1] ? k->y[0] : k->y[1]);
		break;
	case SCH_D:
		off = y > (k->y[0] < k->y[1] ? k->y[0] : k->y[1]);
		break;
	case SCH_R:
		off = x < (k->x[0] > k->x[1] ? k->x[0] : k->x[1]);
		break;
	default:
		off = x > (k->x[0] < k->x[1] ? k->x[0] : k->x[1]);
		break;
	}
	return off && !near ? SIDE : 0;
}

// A path to route: of group g, from the pin s to its tree: a point of
// its wires where a T may be made, or, if the tree has no wire yet (t.x
// not INT_MIN), the pin t; kept to the side k, if any.
struct leg {
	int g;
	struct sch_at s, t;
	const struct keep *k;
};

// Starts the search of the leg lg: the point out from its pin, if a
// wire may start there.  Returns 0 if not.
static int route_start(struct draw *dr, const struct leg *lg)
{
	const struct group *gr = &dr->groups[lg->g];
	int sd = lg->s.dir;
	int s0x = lg->s.x + way_dx[sd];
	int s0y = lg->s.y + way_dy[sd];
	int at_t = s0x == lg->t.x && s0y == lg->t.y;
	int a = 0;
	unsigned other = 0;

	if (!inside(dr, s0x, s0y)) {
		return 0;
	}
	a = idx(dr, s0x, s0y);
	other = ways(dr, a, NULL) & ~ways(dr, a, gr);
	if ((dr->blocked[a] && !at_t) || (dr->pinnet[a] != -1 && !at_t) ||
	    (other && (other != across(sd) || ways(dr, a, gr)))) {
		return 0;
	}
	dr->cost[(a * 4) + sd] = STEP;
	push(dr, (a * 4) + sd);
	return 1;
}

// The cost of the step of the leg lg from state s, at the way d, the
// way nd to the point b at p; or -1 if the step may not be taken.
static int step_cost(const struct draw *dr, const struct leg *lg, int s,
		     struct gstep to, struct gpt p)
{
	const struct group *gr = &dr->groups[lg->g];
	int d = s % 4;
	int nd = to.d;
	int b = to.a;
	unsigned bm = 0;
	unsigned bo = 0;
	int crossing = 0;
	int c = 0;

	if (p.x == lg->t.x && p.y == lg->t.y) {
		if (nd != (lg->t.dir + 2) % 4) {
			return -1;
		}
	} else if (dr->blocked[b] || dr->pinnet[b] != -1) {
		return -1;
	}
	bm = ways(dr, b, gr);
	bo = ways(dr, b, NULL) & ~bm;
	crossing = bo != 0;
	if (crossing && (bm || bo != across(nd))) {
		return -1;
	}
	if (bm && (bits(bm) > 2 || (bm & way_bit((nd + 2) % 4)))) {
		return -1;
	}
	c = dr->cost[s] + STEP + (nd != d ? BEND : 0) + (crossing ? CROSS : 0) +
	    near_cost(dr, p, gr) + stray(lg->k, p);
	if (bm && (next_to_dot(dr, p) || next_to_pin(dr, p))) {
		c += CROWD;
	}
	return c;
}

// Takes the steps of the leg lg from state s at point x: each way on but
// back, to a point it may go to, at what it costs.  Through a wire of
// another group only straight across.
static void expand(struct draw *dr, const struct leg *lg, int s, struct gpt x)
{
	int a = s / 4;
	int d = s % 4;
	unsigned other = ways(dr, a, NULL) & ~ways(dr, a, &dr->groups[lg->g]);

	for (int nd = 0; nd < 4; nd++) {
		struct gpt p = { x.x + way_dx[nd], x.y + way_dy[nd] };
		int e = 0;
		int c = 0;
		int ns = 0;

		if (nd == (d + 2) % 4 || !inside(dr, p.x, p.y) ||
		    (other && nd != d)) {
			continue;
		}
		e = edge(dr, (struct gstep){ a, nd });
		if (e < 0 || dr->eown[e] >= 0) {
			continue;
		}
		c = step_cost(dr, lg, s,
			      (struct gstep){ idx(dr, p.x, p.y), nd }, p);
		ns = (idx(dr, p.x, p.y) * 4) + nd;
		if (c >= 0 && c < dr->cost[ns]) {
			dr->cost[ns] = c;
			dr->from[ns] = s;
			push(dr, ns);
		}
	}
}

// Searches the way of the leg lg; returns the state it ends in, or -1.
static int route_search(struct draw *dr, const struct leg *lg)
{
	const struct group *gr = &dr->groups[lg->g];

	while (dr->nheap) {
		int s = pop(dr);
		int a = s / 4;
		int d = s % 4;
		struct gpt x = { (a % dr->w) + dr->x0, (a / dr->w) + dr->y0 };
		unsigned mine = ways(dr, a, gr);
		unsigned other = ways(dr, a, NULL) & ~mine;

		if (x.x == lg->t.x && x.y == lg->t.y) {
			if (d == (lg->t.dir + 2) % 4) {
				return s;
			}
			continue;
		}
		if (mine && dr->pinnet[a] == -1) {
			if (bits(mine) <= 2 && !(mine & way_bit((d + 2) % 4)) &&
			    !other) {
				return s;
			}
			continue;
		}
		expand(dr, lg, s, x);
	}
	return -1;
}

// Routes the leg lg.  Lays the path and adds its cost; returns 0 if
// there is none.
static int route(struct draw *dr, const struct leg *lg)
{
	int n = dr->w * dr->h * 4;
	int goal = -1;
	struct group *gr = &dr->groups[lg->g];

	for (int i = 0; i < n; i++) {
		dr->cost[i] = INT_MAX;
		dr->from[i] = -1;
	}
	dr->nheap = 0;
	if (!route_start(dr, lg)) {
		return 0;
	}
	goal = route_search(dr, lg);
	if (goal < 0) {
		return 0;
	}
	gr->cost += dr->cost[goal];
	for (int s = goal; s >= 0; s = dr->from[s]) {
		int e = edge(dr, (struct gstep){ s / 4, ((s % 4) + 2) % 4 });

		dr->eown[e] = lg->g;
		gr->edge =
			xrealloc(gr->edge, (gr->nedge + 1) * sizeof *gr->edge);
		gr->edge[gr->nedge++] = e;
	}
	return 1;
}

// Takes the wires of group g off the grid.
static void rip(struct draw *dr, size_t g)
{
	for (size_t i = 0; i < dr->groups[g].nedge; i++) {
		dr->eown[dr->groups[g].edge[i]] = -1;
	}
	dr->groups[g].nedge = 0;
	dr->groups[g].cost = 0;
}

static struct sch_at end_at(struct draw *dr, const struct sch_end *e)
{
	return sch_pin_at(&dr->sch.part[e->part], e->pin);
}

// The pins of a group being routed: where each stands, and whether it
// is wired.
struct group_pins {
	size_t m;
	struct sch_at *p;
	int *done;
};

// The tree of a leg, not a pin.
static const struct sch_at to_tree = { INT_MIN, INT_MIN, 0 };

// The index among the pins gp of group gr of pin i of hint hn, or -1.
static long hint_pin(const struct draw *dr, const struct group *gr,
		     const struct sch_hint *hn, int i)
{
	const struct sch_net *t = &dr->sch.net[gr->net];
	long at = -1;

	for (size_t j = 0; j < gr->nend; j++) {
		const struct sch_end *e = &t->end[gr->end[j]];

		if (e->part == hn->part[i] && e->pin == hn->pin[i]) {
			at = (long)j;
		}
	}
	return at;
}

// Routes the wires of group g that the source names, with their sides,
// first; *wired is set once a wire is laid.  Returns 0 if one finds no
// way.
static int route_hints(struct draw *dr, size_t g, struct group_pins *gp,
		       int *wired)
{
	const struct group *gr = &dr->groups[g];
	const struct sch_at *p = gp->p;

	for (size_t h = 0; h < dr->sch.nhint; h++) {
		const struct sch_hint *hn = &dr->sch.hint[h];
		long a = hint_pin(dr, gr, hn, 0);
		long b = hint_pin(dr, gr, hn, 1);
		struct keep k = { 0, { 0, 0 }, { 0, 0 } };
		struct leg lg = { (int)g, to_tree, to_tree, &k };

		if (a < 0 || b < 0 || (gp->done[a] && gp->done[b])) {
			continue;
		}
		k = (struct keep){ hn->side,
				   { p[a].x, p[b].x },
				   { p[a].y, p[b].y } };
		lg.s = p[b];
		if (!*wired) {
			lg.t = p[a];
		} else if (gp->done[b]) {
			lg.s = p[a];
		}
		if (!route(dr, &lg)) {
			return 0;
		}
		gp->done[a] = gp->done[b] = 1;
		*wired = 1;
	}
	return 1;
}

// Routes the spine of group g, if nothing is wired: the two pins that
// face each other on one line farthest apart.  Else the first pin is
// where the tree starts.
static void route_spine(struct draw *dr, size_t g, struct group_pins *gp,
			int *wired)
{
	const struct sch_at *p = gp->p;
	int far = -1;
	size_t si = 0;
	size_t sj = 0;

	for (size_t i = 0; i < gp->m; i++) {
		for (size_t j = 0; j < gp->m; j++) {
			int len = abs(p[j].x - p[i].x) + abs(p[j].y - p[i].y);

			if (i != j && p[j].dir == (p[i].dir + 2) % 4 &&
			    p[i].x + way_dx[p[i].dir] * len == p[j].x &&
			    p[i].y + way_dy[p[i].dir] * len == p[j].y &&
			    len > far) {
				far = len;
				si = i;
				sj = j;
			}
		}
	}
	if (far > 0) {
		struct leg lg = { (int)g, p[sj], p[si], NULL };

		if (route(dr, &lg)) {
			gp->done[si] = gp->done[sj] = 1;
			*wired = 1;
			return;
		}
	}
	gp->done[0] = 1;
}

// The pin of gp not wired nearest a pin wired, or gp->m if none.
static size_t nearest_pin(const struct group_pins *gp)
{
	const struct sch_at *p = gp->p;
	size_t best = gp->m;
	int bd = INT_MAX;

	for (size_t j = 0; j < gp->m; j++) {
		for (size_t i = 0; !gp->done[j] && i < gp->m; i++) {
			int dd = abs(p[i].x - p[j].x) + abs(p[i].y - p[j].y);

			if (gp->done[i] && dd < bd) {
				bd = dd;
				best = j;
			}
		}
	}
	return best;
}

// Routes the rest of the pins of group g to its tree, the nearest
// first.  Returns 0 if one finds no way.
static int route_rest(struct draw *dr, size_t g, struct group_pins *gp,
		      int wired)
{
	for (;;) {
		size_t best = nearest_pin(gp);
		struct leg lg = { (int)g, to_tree, to_tree, NULL };

		if (best == gp->m) {
			return 1;
		}
		lg.s = gp->p[best];
		if (!wired) {
			size_t first = 0;

			while (!gp->done[first]) {
				first++;
			}
			lg.t = gp->p[first];
		}
		if (!route(dr, &lg)) {
			return 0;
		}
		wired = 1;
		gp->done[best] = 1;
	}
}

// Routes group g as a tree.  Returns 0 if a pin finds no way.
static int route_group(struct draw *dr, size_t g)
{
	struct group *gr = &dr->groups[g];
	const struct sch_net *t = &dr->sch.net[gr->net];
	struct group_pins gp = { gr->nend, NULL, NULL };
	int wired = 0;
	int ok = 1;

	gp.p = sch_alloc(gp.m * sizeof *gp.p);
	gp.done = sch_alloc(gp.m * sizeof *gp.done);
	for (size_t j = 0; j < gp.m; j++) {
		gp.p[j] = end_at(dr, &t->end[gr->end[j]]);
	}
	ok = route_hints(dr, g, &gp, &wired);
	if (ok && !wired) {
		route_spine(dr, g, &gp, &wired);
	}
	if (ok) {
		ok = route_rest(dr, g, &gp, wired);
	}
	free(gp.p);
	free(gp.done);
	return ok;
}

// The groups to wire: the pins of one group number of a net.
static void make_groups(struct draw *dr)
{
	size_t n = 0;

	// At most one group for each pin of each net.
	for (size_t k = 0; k < dr->sch.nnet; k++) {
		n += dr->sch.net[k].nend;
	}
	dr->groups = sch_alloc((n + 1) * sizeof *dr->groups);
	for (size_t k = 0; k < dr->sch.nnet; k++) {
		const struct sch_net *t = &dr->sch.net[k];

		for (size_t j = 0; j < t->nend; j++) {
			int seen = 0;
			struct group *gr = &dr->groups[dr->ngroups];

			for (size_t i = 0; i < j; i++) {
				seen = seen ||
				       t->end[i].group == t->end[j].group;
			}
			if (seen) {
				continue;
			}
			gr->net = k;
			for (size_t i = j; i < t->nend; i++) {
				gr->nend += t->end[i].group == t->end[j].group;
			}
			if (gr->nend < 2) {
				gr->nend = 0;
				continue;
			}
			gr->end = sch_alloc(gr->nend * sizeof *gr->end);
			gr->nend = 0;
			for (size_t i = j; i < t->nend; i++) {
				if (t->end[i].group == t->end[j].group) {
					gr->end[gr->nend++] = i;
				}
			}
			dr->ngroups++;
		}
	}
}

// The size of a group, to order them: its pins and its extent.
static int group_size(struct draw *dr, size_t g)
{
	int x0 = INT_MAX;
	int y0 = INT_MAX;
	int x1 = INT_MIN;
	int y1 = INT_MIN;

	if (!dr->groups[g].nend) {
		return 0;
	}
	for (size_t j = 0; j < dr->groups[g].nend; j++) {
		struct sch_at at =
			end_at(dr, &dr->sch.net[dr->groups[g].net]
					    .end[dr->groups[g].end[j]]);
		int x = at.x;
		int y = at.y;

		x0 = x < x0 ? x : x0;
		y0 = y < y0 ? y : y0;
		x1 = x > x1 ? x : x1;
		y1 = y > y1 ? y : y1;
	}
	return (x1 - x0) + (y1 - y0) + (4 * (int)dr->groups[g].nend);
}

// Sorts the groups of order, the smallest first.
static void sort_groups(struct draw *dr, size_t *order)
{
	for (size_t g = 0; g < dr->ngroups; g++) {
		order[g] = g;
	}
	for (size_t i = 1; i < dr->ngroups; i++) {
		for (size_t j = i;
		     j > 0 &&
		     group_size(dr, order[j]) < group_size(dr, order[j - 1]);
		     j--) {
			size_t t = order[j];

			order[j] = order[j - 1];
			order[j - 1] = t;
		}
	}
}

// Routes the groups in order, the one that fails put first and all
// again, TRIES times at the most; returns whether all found a way.
static int route_tries(struct draw *dr, size_t *order)
{
	int ok = 0;

	for (int tries = 0; tries < TRIES && !ok; tries++) {
		ok = 1;
		for (size_t g = 0; g < dr->ngroups; g++) {
			rip(dr, g);
		}
		for (size_t o = 0; o < dr->ngroups; o++) {
			if (!route_group(dr, order[o])) {
				size_t bad = order[o];

				memmove(order + 1, order, o * sizeof *order);
				order[0] = bad;
				ok = 0;
				break;
			}
		}
	}
	return ok;
}

// Routes the groups in order, those with no way left without wires,
// and told.
static void route_what_goes(struct draw *dr, const size_t *order)
{
	for (size_t g = 0; g < dr->ngroups; g++) {
		rip(dr, g);
	}
	for (size_t o = 0; o < dr->ngroups; o++) {
		if (!route_group(dr, order[o])) {
			const struct sch_net *t =
				&dr->sch.net[dr->groups[order[o]].net];

			(void)fprintf(stderr, "%s:%d: no way for net %s\n",
				      dr->sch.file, t->line, t->name);
			rip(dr, order[o]);
			dr->failed = 1;
		}
	}
}

// Rips up group g and routes it again among the rest; the old path
// comes back if the new one is no better.
static void reroute(struct draw *dr, size_t g)
{
	struct group *gr = &dr->groups[g];
	int *old = gr->edge;
	int was = gr->cost;
	size_t n_old = gr->nedge;

	for (size_t i = 0; i < n_old; i++) {
		dr->eown[old[i]] = -1;
	}
	gr->edge = NULL;
	gr->nedge = 0;
	gr->cost = 0;
	if (!route_group(dr, g) || gr->cost > was) {
		rip(dr, g);
		free(gr->edge);
		gr->edge = old;
		gr->nedge = n_old;
		gr->cost = was;
		for (size_t i = 0; i < n_old; i++) {
			dr->eown[old[i]] = (int)g;
		}
	} else {
		free(old);
	}
}

// Routes all groups: in order, the one that fails put first and all
// again; then each ripped up and routed again among the rest.
static void route_all(struct draw *dr)
{
	size_t *order = sch_alloc((dr->ngroups + 1) * sizeof *order);
	int n = dr->w * dr->h * 4;

	dr->heap = sch_alloc((size_t)n * 4 * sizeof *dr->heap);
	dr->cost = sch_alloc((size_t)n * sizeof *dr->cost);
	dr->from = sch_alloc((size_t)n * sizeof *dr->from);
	sort_groups(dr, order);
	if (!route_tries(dr, order)) {
		// The rest is drawn, the groups with no way without wires.
		route_what_goes(dr, order);
	}
	for (int pass = 0; pass < PASSES; pass++) {
		for (size_t o = 0; o < dr->ngroups; o++) {
			reroute(dr, order[o]);
		}
	}
	free(order);
}

// Turns the wires of each group into polylines, from node to node: a
// node is a pin, an end, or a junction.
// Room for a polyline being traced, the edges traced, and the group.
struct tracer {
	int *xy;
	unsigned char *seen;
	size_t g;
};

// Traces the wire of group tr->g from the step st on, up to the next
// node, into tr->xy; returns its number of points.
static size_t trace_from(struct draw *dr, struct tracer *tr, struct gstep st)
{
	const struct group *gr = &dr->groups[tr->g];
	int *xy = tr->xy;
	int a = st.a;
	int b = a;
	int dd = st.d;
	size_t n = 1;

	xy[0] = (a % dr->w) + dr->x0;
	xy[1] = (a / dr->w) + dr->y0;
	for (;;) {
		unsigned bm = 0;
		int nd = 0;

		tr->seen[edge(dr, (struct gstep){ b, dd })] = 1;
		b += way_dx[dd] + (way_dy[dd] * dr->w);
		bm = ways(dr, b, gr);
		if (bits(bm) != 2 || dr->pinnet[b] != -1 || b == a) {
			break;
		}
		while (!(bm & way_bit(nd)) || nd == (dd + 2) % 4) {
			nd++;
		}
		if (nd != dd) {
			xy[2 * n] = (b % dr->w) + dr->x0;
			xy[(2 * n) + 1] = (b / dr->w) + dr->y0;
			n++;
		}
		dd = nd;
	}
	xy[2 * n] = (b % dr->w) + dr->x0;
	xy[(2 * n) + 1] = (b / dr->w) + dr->y0;
	return n + 1;
}

// Traces the wires of group tr->g from the nodes, those of a pin or an
// end or a junction on the first pass, any left (loops) on the second.
static void trace_group(struct draw *dr, struct tracer *tr)
{
	const struct group *gr = &dr->groups[tr->g];

	for (int pass = 0; pass < 2; pass++) {
		for (int a = 0; a < dr->w * dr->h; a++) {
			unsigned m = ways(dr, a, gr);

			if (!m || (pass == 0 && bits(m) == 2 &&
				   dr->pinnet[a] == -1)) {
				continue;
			}
			for (int d = 0; d < 4; d++) {
				struct gstep st = { a, d };
				struct poly p = { tr->xy, 0, gr->net };

				if (!(m & way_bit(d)) ||
				    tr->seen[edge(dr, st)]) {
					continue;
				}
				p.n = trace_from(dr, tr, st);
				add_poly(dr, &p);
			}
		}
	}
}

static void trace(struct draw *dr)
{
	struct tracer tr = { NULL, NULL, 0 };

	tr.xy = sch_alloc(2 * ((size_t)dr->w * (size_t)dr->h + 2) *
			  sizeof *tr.xy);
	tr.seen = sch_alloc(2 * (size_t)dr->w * (size_t)dr->h);
	for (size_t g = 0; g < dr->ngroups; g++) {
		tr.g = g;
		trace_group(dr, &tr);
	}
	free(tr.seen);
	free(tr.xy);
}

// Whether (x, y) lies on the segment from (ax, ay) to (bx, by).
static int on_seg(int x, int y, int ax, int ay, int bx, int by)
{
	if (ax == bx) {
		return x == ax && y >= (ay < by ? ay : by) &&
		       y <= (ay > by ? ay : by);
	}
	return y == ay && x >= (ax < bx ? ax : bx) && x <= (ax > bx ? ax : bx);
}

// Whether (x, y) lies on polyline i; on an end counts.
static int on_poly(const struct poly *p, int x, int y)
{
	for (size_t j = 1; j < p->n; j++) {
		if (on_seg(x, y, p->xy[(2 * j) - 2], p->xy[(2 * j) - 1],
			   p->xy[2 * j], p->xy[(2 * j) + 1])) {
			return 1;
		}
	}
	return 0;
}

// The number of wire arms at (x, y): one for each wire ending there, two
// for each running through.
static int arms(struct draw *dr, int x, int y)
{
	int n = 0;

	for (size_t i = 0; i < dr->npolys; i++) {
		const struct poly *p = &dr->polys[i];
		int ends = (p->xy[0] == x && p->xy[1] == y) +
			   (p->xy[(2 * p->n) - 2] == x &&
			    p->xy[(2 * p->n) - 1] == y);

		if (ends) {
			n += ends;
		} else if (on_poly(p, x, y)) {
			n += 2;
		}
	}
	return n;
}

// Whether a dot belongs at (x, y): three arms or more.
static int dot_at(struct draw *dr, int x, int y)
{
	return arms(dr, x, y) >= 3;
}

// Adds a run to line l of the label.
static void add_run(struct label *b, int l, const char *s, double size)
{
	struct run *r = &b->run[l][b->nrun[l]++];

	r->s = sch_strdup(s);
	r->size = size;
}

static void measure(struct label *b)
{
	b->w = 0;
	for (int l = 0; l < b->nline; l++) {
		double w = 0;

		for (int r = 0; r < b->nrun[l]; r++) {
			w += sch_text_width(b->run[l][r].s, b->run[l][r].size);
		}
		b->w = fmax(b->w, w);
	}
	b->h = DESCENT + TEXT + (b->nline - 1) * LEAD;
}

// Where a text goes: its point, and its alignment, left (0), centered
// (1) or right (2).
struct text_at {
	double x, y;
	int align;
};

// How far beyond the end of a flag its text goes: past the tip of a
// port or a global, in points.
static double flag_off(const char *fl)
{
	if (!strcmp(fl, "port")) {
		return PORT_TIP + FLAG_TEXT_OFF;
	}
	if (!strcmp(fl, "global")) {
		return GLOBAL_TIP + FLAG_TEXT_OFF;
	}
	return FLAG_TEXT_OFF;
}

// The box of the text of a flag, in points, and where its text goes,
// into *ta: beyond its end, the way out from the part.  Returns 0 for a
// flag without text.
static int flag_box(const struct sch_part *q, double b[4], struct text_at *ta)
{
	struct sch_at pin_at = sch_pin_at(q, 0);
	int out = (pin_at.dir + 2) % 4;
	double px = NAN;
	double py = NAN;
	double off = NAN;
	double w = NAN;
	const char *fl = q->dev->shape->flag;

	if (strcmp(fl, "supply") != 0 && strcmp(fl, "port") != 0 &&
	    strcmp(fl, "global") != 0) {
		return 0;
	}
	off = flag_off(fl);
	px = pin_at.x * GRID + way_dx[out] * off;
	py = pin_at.y * GRID + way_dy[out] * off;
	w = sch_text_width(q->text, TEXT);
	ta->x = px;
	if (out == SCH_U || out == SCH_D) {
		ta->align = 1;
		ta->y = out == SCH_U ? py : py - TEXT;
		b[0] = px - w / 2;
	} else {
		ta->align = out == SCH_L ? 2 : 0;
		ta->y = py - TEXT / 2;
		b[0] = out == SCH_L ? px - w : px;
	}
	b[1] = ta->y - DESCENT;
	b[2] = b[0] + w;
	b[3] = ta->y + TEXT;
	return 1;
}

// Where the number of pin p of a part with a pin table goes, and its
// alignment, into *ta; returns 0 for a part without a table.
static int pin_number_at(const struct sch_part *q, size_t p, struct text_at *ta)
{
	struct sch_at pin_at = sch_pin_at(q, p);
	int d = pin_at.dir;

	if (q->dev->pnum[0] == q->dev->shape->pin[0].name) {
		return 0;
	}
	ta->x = pin_at.x * GRID - way_dx[d] * PINNUM_IN * GRID;
	ta->y = pin_at.y * GRID - way_dy[d] * PINNUM_IN * GRID;
	if (d == SCH_L || d == SCH_R) {
		ta->y += PINNUM_UP;
		ta->align = 1;
	} else {
		ta->x += PINNUM_RIGHT;
		ta->y -= PINNUM / 2;
		ta->align = 0;
	}
	return 1;
}

// How much the boxes a and b overlap: their common area and COVER_ANY,
// or 0.
static double cover(const double a[4], const double b[4])
{
	double ox = fmin(a[2], b[2]) - fmax(a[0], b[0]);
	double oy = fmin(a[3], b[3]) - fmax(a[1], b[1]);

	return ox > 0 && oy > 0 ? (ox * oy) + COVER_ANY : 0;
}

// How much of the box a (points) the wires take.
static double wires_clash(const struct draw *dr, const double a[4])
{
	double c = 0;
	double b[4];

	for (size_t i = 0; i < dr->npolys; i++) {
		const struct poly *p = &dr->polys[i];

		for (size_t j = 1; j < p->n; j++) {
			double ax = p->xy[(2 * j) - 2] * GRID;
			double ay = p->xy[(2 * j) - 1] * GRID;
			double bx = p->xy[2 * j] * GRID;
			double by = p->xy[(2 * j) + 1] * GRID;

			b[0] = fmin(ax, bx) - dr->sch.clear_text;
			b[1] = fmin(ay, by) - dr->sch.clear_text;
			b[2] = fmax(ax, bx) + dr->sch.clear_text;
			b[3] = fmax(ay, by) + dr->sch.clear_text;
			c += cover(a, b);
		}
	}
	return c;
}

// The clash c and how much of the box a the pin numbers of part q take.
static double numbers_clash(const struct sch_part *q, const double a[4],
			    double c)
{
	struct text_at ta = { NAN, NAN, 0 };

	for (size_t p = 0; p < q->dev->shape->npin; p++) {
		if (pin_number_at(q, p, &ta)) {
			const double b[4] = {
				ta.x - (ta.align ? 3 : PINNUM_PAD),
				ta.y - PINNUM_PAD, ta.x + PINNUM_RIGHT_W,
				ta.y + PINNUM
			};

			c += cover(a, b);
		}
	}
	return c;
}

// How much of the box a (points) the drawing already takes: wires,
// bodies, the text of flags, pin numbers and the labels placed before,
// but that of part self.
static double clash(const struct draw *dr, const double a[4], size_t self)
{
	double c = wires_clash(dr, a);
	double b[4];

	for (size_t i = 0; i < dr->sch.npart; i++) {
		const struct sch_part *q = &dr->sch.part[i];
		struct text_at ta = { NAN, NAN, 0 };
		double m = q->dev->shape->flag ? dr->sch.clear_flag :
						 dr->sch.clear_text;

		sch_box(q, b);
		for (int k = 0; k < 4; k++) {
			b[k] = b[k] * GRID + (k < 2 ? -m : m);
		}
		c += cover(a, b);
		if (q->dev->shape->flag) {
			if (flag_box(q, b, &ta)) {
				c += cover(a, b);
			}
			continue;
		}
		c = numbers_clash(q, a, c);
		if (i != self && dr->labels[i].nline &&
		    dr->labels[i].side >= 0) {
			const struct label *l = &dr->labels[i];

			b[0] = l->x - dr->sch.clear_text;
			b[1] = l->y - dr->sch.clear_text;
			b[2] = l->x + l->w + dr->sch.clear_text;
			b[3] = l->y + l->h + dr->sch.clear_text;
			c += cover(a, b);
		}
	}
	return c;
}

// A place for a label to try: on side s, the r-th in order, out from
// the side and slid t along it.
struct label_try {
	int s, r;
	double out, t;
};

// Where the label b of part q, its box bx in points, stands at the try
// lt, into x and y.
static void label_spot(const struct draw *dr, const struct sch_part *q,
		       const double bx[4], const struct label_try *lt,
		       double *xy)
{
	const struct label *b = &dr->labels[q - dr->sch.part];
	double gap = dr->sch.clear_label;
	int s = lt->s;

	if (s == 0 || s == 2) {
		xy[0] = s == 0 ? bx[2] + gap + lt->out :
				 bx[0] - gap - lt->out - b->w;
		xy[1] = q->y * GRID - b->h / 2 + lt->t;
	} else {
		xy[0] = q->x * GRID - b->w / 2 + lt->t;
		xy[1] = s == 1 ? bx[3] + gap + LABEL_ABOVE + lt->out :
				 bx[1] - gap - lt->out - b->h - 1;
	}
}

// Places the label of part i: on each side of the part, its own side
// first, slid along the side and moved out from it, at the place of
// least clash, near its middle and its own side rather than not.
static void place_label(struct draw *dr, size_t i)
{
	const struct sch_part *q = &dr->sch.part[i];
	struct label *b = &dr->labels[i];
	double bx[4];
	double best = BEST_NONE;
	int side = q->side >= 0 ? q->side : q->dev->shape->side;
	int order[4];

	if (q->side < 0) {
		if (q->mirror && (side == 0 || side == 2)) {
			side = 2 - side;
		}
		side = (side + q->rot) % 4;
	}
	order[0] = side;
	order[1] = (side + 2) % 4;
	order[2] = (side + 1) % 4;
	order[3] = (side + 3) % 4;
	sch_box(q, bx);
	for (int k = 0; k < 4; k++) {
		bx[k] *= GRID;
	}
	for (int r = 0; r < 4; r++) {
		int s = order[r];
		double span = s % 2 ? (bx[2] - bx[0] + b->w) / 2 :
				      (bx[3] - bx[1] + b->h) / 2;
		int nslide = steps_upto(2 * span, LABEL_SLIDE);

		for (int k = 0; k <= LABEL_OUTS; k++) {
			for (int j = 0; j < nslide; j++) {
				struct label_try lt = { s, r, LABEL_OUT * k,
							-span + (LABEL_SLIDE *
								 j) };
				double xy[2];
				double box[4];
				double c = NAN;

				label_spot(dr, q, bx, &lt, xy);
				box[0] = xy[0];
				box[1] = xy[1];
				box[2] = xy[0] + b->w;
				box[3] = xy[1] + b->h;
				// Room round text is a rule: a place that
				// keeps it wins over any that does not.
				c = clash(dr, box, i);
				c = (c > 0 ? CLASH_RULE + c : 0) +
				    r * SIDE_COST + fabs(lt.t) * SLIDE_COST +
				    lt.out;
				if (c < best) {
					best = c;
					b->x = xy[0];
					b->y = xy[1];
					b->side = s;
				}
			}
		}
	}
}

static void labels_all(struct draw *dr)
{
	dr->labels = sch_alloc(dr->sch.npart * sizeof *dr->labels);
	for (size_t i = 0; i < dr->sch.npart; i++) {
		const struct sch_part *q = &dr->sch.part[i];
		struct label *b = &dr->labels[i];

		b->side = -1;
		if (q->dev->shape->flag) {
			continue;
		}
		add_run(b, b->nline++, q->ref, TEXT);
		if ((q->value || q->rating) && !q->novalue) {
			const char *value = q->value ? q->value : "";
			const char *rating = q->rating ? q->rating : "";
			size_t n = strlen(value) + strlen(rating) + 2;
			char *v = sch_alloc(n);

			(void)snprintf(v, n, "%s%s%s", value,
				       q->value && q->rating ? " " : "",
				       rating);
			add_run(b, b->nline++, v, TEXT);
			free(v);
		}
		measure(b);
	}
	for (size_t i = 0; i < dr->sch.npart; i++) {
		if (dr->labels[i].nline) {
			place_label(dr, i);
		}
	}
}

// Writes a text record of a run, from (x, y) of the drawing, flush left.
static void text_run(struct draw *dr, const struct run *r, double x, double y)
{
	printf("text %.3f 0 0 %.3f %.3f %s\n", r->size * dr->sc,
	       dr->ox + (x * dr->sc), dr->oy + (y * dr->sc), r->s);
}

// Writes runs from the point of at, by its alignment.
static void text_runs(struct draw *dr, const struct run *r, int n,
		      struct text_at at)
{
	double w = 0;
	double x = at.x;
	double y = at.y;
	int align = at.align;

	for (int i = 0; i < n; i++) {
		w += sch_text_width(r[i].s, r[i].size);
	}
	x -= w * align / 2;
	for (int i = 0; i < n; i++) {
		text_run(dr, &r[i], x, y);
		x += sch_text_width(r[i].s, r[i].size);
	}
}

// Writes the points of a line or polygon, from the drawing, as the page.
static void points(struct draw *dr, const double *xy, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		printf(" %.3f %.3f", dr->ox + (xy[2 * i] * dr->sc),
		       dr->oy + (xy[(2 * i) + 1] * dr->sc));
	}
	printf("\n");
}

// The text of a field, by its name after a $.
static const char *field(struct draw *dr, const char *name);

// Reads up to n numbers from s into v as sscanf's %lf would, each
// after the blanks before it; returns how many, *end past the last.
static int read_numbers(const char *s, double *v, int n, const char **end)
{
	int k = 0;

	for (; k < n; k++) {
		char *e = NULL;
		double d = strtod(s, &e);

		if (e == s) {
			break;
		}
		v[k] = d;
		s = e;
	}
	*end = s;
	return k;
}

// Writes the record text SIZE ANGLE ALIGN X Y TEXT of a sheet, if it is
// one, its field filled in for a text $NAME.
static void sheet_text(struct draw *dr, const char *buf)
{
	double v[TEXT_NUMBERS];
	const char *t = buf + strlen("text");
	char *e = NULL;
	long tj = 0;

	if (strncmp(buf, "text", strlen("text")) != 0 ||
	    read_numbers(t, v, 2, &t) != 2) {
		return;
	}
	tj = strtol(t, &e, DECIMAL);
	if (e == t || read_numbers(e, v + 2, 2, &t) != 2) {
		return;
	}
	while (isspace((unsigned char)*t)) {
		t++;
	}
	if (*t == '$') {
		t = field(dr, t + 1);
	}
	if (*t) {
		printf("text %.3f %g %d %g %g %s\n", v[0], v[1], (int)tj, v[2],
		       v[3], t);
	}
}

// Reads the sheet TEMPLATE.sheet and, if print is set, writes it, its
// fields filled in: the paper and the room for the drawing come back.
static void sheet(struct draw *dr, double room[4], double paper[2], int print)
{
	char *path = sch_sheet_path(&dr->sch);
	char buf[LINE];
	FILE *f = NULL;

	room[0] = room[1] = room[2] = room[3] = 0;
	paper[0] = paper[1] = 0;
	if (!path) {
		sch_die("%s: no sheet %s in the library", dr->sch.file,
			dr->sch.sheet);
	}
	f = fopen(path, "r");
	if (!f) {
		sch_die("%s: cannot read", path);
	}
	while (fgets(buf, sizeof buf, f)) {
		const char *end = NULL;

		buf[strcspn(buf, "\n")] = 0;
		if (!strncmp(buf, "paper", strlen("paper")) &&
		    read_numbers(buf + strlen("paper"), paper, 2, &end) == 2) {
			if (print) {
				printf("page %.0f %.0f %d\n", paper[0],
				       paper[1], dr->sch.page);
			}
		} else if ((!strncmp(buf, "room", strlen("room")) &&
			    read_numbers(buf + strlen("room"), room, 4, &end) ==
				    4) ||
			   !print) {
		} else if (!strncmp(buf, "line ", strlen("line "))) {
			printf("line %.2f%s\n", LW, buf + 4);
		} else {
			sheet_text(dr, buf);
		}
	}
	(void)fclose(f);
	if (!(paper[0] > 0) || !(room[2] > room[0])) {
		sch_die("%s: no paper or room", path);
	}
	free(path);
}

// Writes the circle d, its center at c, as a line.
static void circle(struct draw *dr, const struct sch_draw *d, const double *c)
{
	double r = d->xy[2];
	double *xy = sch_alloc(2 * (size_t)(CIRCLE_STEPS + 1) * sizeof *xy);

	for (int k = 0; k <= CIRCLE_STEPS; k++) {
		size_t i = (size_t)k;

		xy[2 * i] = c[0] + r * cos(2 * PI * k / CIRCLE_STEPS);
		xy[(2 * i) + 1] = c[1] + r * sin(2 * PI * k / CIRCLE_STEPS);
	}
	printf("line %.3f", d->w * dr->sc);
	points(dr, xy, CIRCLE_STEPS + 1);
	free(xy);
}

// The points of the drawing d of part q, turned and placed as the part,
// into xy.
static void symbol_points(const struct sch_part *q, const struct sch_draw *d,
			  size_t np, double *xy)
{
	double c = (double)(q->rot == 0) - (double)(q->rot == 2);
	double sn = (double)(q->rot == 1) - (double)(q->rot == 3);

	for (size_t k = 0; k < np; k++) {
		double x = q->mirror ? -d->xy[2 * k] : d->xy[2 * k];
		double y = d->xy[(2 * k) + 1];

		xy[2 * k] = q->x * GRID + x * c - y * sn;
		xy[(2 * k) + 1] = q->y * GRID + x * sn + y * c;
	}
}

// Writes the records of a part's shape, turned and placed as the part.
static void symbol(struct draw *dr, const struct sch_part *q)
{
	const struct sch_shape *sh = q->dev->shape;

	for (size_t i = 0; i < sh->ndraw; i++) {
		const struct sch_draw *d = &sh->draw[i];
		size_t np = d->kind >= SCH_CIRCLE ? 1 : d->n / 2;
		double *xy = sch_alloc(2 * (np + 1) * sizeof *xy);

		if ((d->when == 1 && !q->flip) || (d->when == 2 && q->flip)) {
			free(xy);
			continue;
		}
		symbol_points(q, d, np, xy);
		if (d->kind == SCH_LINE || d->kind == SCH_LOOP) {
			if (d->kind == SCH_LOOP) {
				xy[2 * np] = xy[0];
				xy[(2 * np) + 1] = xy[1];
				np++;
			}
			printf("line %.3f", d->w * dr->sc);
			points(dr, xy, np);
		} else if (d->kind == SCH_FILL || d->kind == SCH_BLANK) {
			printf("%spoly", d->kind == SCH_BLANK ? "clear\n" : "");
			points(dr, xy, np);
			if (d->kind == SCH_BLANK) {
				printf("dark\n");
			}
		} else if (d->kind == SCH_DISK) {
			printf("disk %.3f %.3f %.3f\n",
			       dr->ox + (xy[0] * dr->sc),
			       dr->oy + (xy[1] * dr->sc), d->xy[2] * dr->sc);
		} else {
			circle(dr, d, xy);
		}
		free(xy);
	}
}

// The text of a field: from the source, or made.
static const char *field(struct draw *dr, const char *name)
{
	for (size_t i = 0; i < dr->sch.nfield; i++) {
		if (strcmp(dr->sch.field[i][0], name) == 0) {
			return dr->sch.field[i][1];
		}
	}
	if (strcmp(name, "Title") == 0) {
		return dr->sch.name;
	}
	if (strcmp(name, "Page") == 0) {
		static char page[PAGE_LEN];

		(void)snprintf(page, sizeof page, "%d", dr->sch.page);
		return page;
	}
	if (strcmp(name, "File") == 0) {
		const char *base = strrchr(dr->sch.file, '/');

		return base ? base + 1 : dr->sch.file;
	}
	return "";
}

// The extent of the drawing in grid units.
static void extent(struct draw *dr, double e[4])
{
	e[0] = e[1] = FAR_GRID;
	e[2] = e[3] = -FAR_GRID;
	for (size_t i = 0; i < dr->sch.npart; i++) {
		double b[4];

		sch_box(&dr->sch.part[i], b);
		e[0] = fmin(e[0], b[0]);
		e[1] = fmin(e[1], b[1]);
		e[2] = fmax(e[2], b[2]);
		e[3] = fmax(e[3], b[3]);
		if (dr->labels[i].nline) {
			e[0] = fmin(e[0], dr->labels[i].x / GRID);
			e[1] = fmin(e[1], dr->labels[i].y / GRID);
			e[2] = fmax(e[2],
				    (dr->labels[i].x + dr->labels[i].w) / GRID);
			e[3] = fmax(e[3],
				    (dr->labels[i].y + dr->labels[i].h) / GRID);
		}
	}
	for (size_t i = 0; i < dr->npolys; i++) {
		for (size_t j = 0; j < dr->polys[i].n; j++) {
			e[0] = fmin(e[0], dr->polys[i].xy[2 * j]);
			e[1] = fmin(e[1], dr->polys[i].xy[(2 * j) + 1]);
			e[2] = fmax(e[2], dr->polys[i].xy[2 * j]);
			e[3] = fmax(e[3], dr->polys[i].xy[(2 * j) + 1]);
		}
	}
	e[0] -= 3;
	e[2] += 3;
	e[1] -= 2;
	e[3] += 2;
}

// The text of a flag: the name of its net beyond its end; of a port,
// its name and each piece of its list of pages a text of its own (BUF,
// (, 1, ",", 2, )), so a reader finds where each page number stands.
static void flag_text(struct draw *dr, const struct sch_part *q)
{
	double b[4];
	struct text_at ta = { NAN, NAN, 0 };
	int n = 1;
	size_t len = strlen(q->name);
	struct run r[RUNS_MOST];
	char piece[RUNS_MOST][PIECE_LEN];
	const char *t = q->text + (strncmp(q->text, q->name, len) ? 0 : len);

	if (!flag_box(q, b, &ta)) {
		return;
	}
	r[0].s = q->name;
	r[0].size = TEXT;
	if (t == q->text) {
		r[0].s = q->text;
		t = "";
	}
	while (*t && n < RUNS_MOST) {
		size_t k = isdigit((unsigned char)*t) ?
				   strspn(t, "0123456789") :
				   1;

		k = k < sizeof piece[0] ? k : sizeof piece[0] - 1;
		memcpy(piece[n], t, k);
		piece[n][k] = 0;
		r[n] = r[0];
		r[n].s = piece[n];
		n++;
		t += k;
	}
	text_runs(dr, r, n, ta);
}

// The pin numbers of a part with a pin table, by its leads.
static void pin_numbers(struct draw *dr, const struct sch_part *q)
{
	for (size_t p = 0; p < q->dev->shape->npin; p++) {
		struct text_at ta = { NAN, NAN, 0 };
		struct run r = { q->dev->pnum[p], PINNUM };

		if (pin_number_at(q, p, &ta)) {
			text_runs(dr, &r, 1, ta);
		}
	}
}

// Where each pin of a part meets its wire, by its designator and number
// (pin R401.2 X Y), or a flag's, by its net and whether it is a port of
// the sheet or a global (a ground, a supply, a global): flag GND X Y
// global.
static void pin_ends(struct draw *dr, const struct sch_part *q)
{
	for (size_t p = 0; p < q->dev->shape->npin; p++) {
		int x = 0;
		int y = 0;

		{
			struct sch_at pin_at = sch_pin_at(q, p);
			x = pin_at.x;
			y = pin_at.y;
		}
		if (q->dev->shape->flag) {
			printf("flag %s", q->name);
			printf(" %.3f %.3f %s\n", dr->ox + (x * GRID * dr->sc),
			       dr->oy + (y * GRID * dr->sc),
			       !strcmp(q->dev->shape->flag, "port") ? "port" :
								      "global");
			continue;
		}
		printf("pin %s.%s", q->ref, sch_pnum(q->dev, p));

		printf(" %.3f %.3f\n", dr->ox + (x * GRID * dr->sc),
		       dr->oy + (y * GRID * dr->sc));
	}
}

static int body(struct draw *dr, size_t i, double b[4]);

static int boxes_meet(const double *a, const double *b)
{
	return a[0] < b[2] && b[0] < a[2] && a[1] < b[3] && b[1] < a[3];
}

// Whether the box b (points of the drawing) meets a wire or a dot.
static int meets_wires(const struct draw *dr, const double b[4])
{
	double c[4];

	for (size_t i = 0; i < dr->npolys; i++) {
		const struct poly *p = &dr->polys[i];

		for (size_t j = 0; j < p->n; j++) {
			size_t k = j + 1 < p->n ? j + 1 : j;

			c[0] = fmin(p->xy[2 * j], p->xy[2 * k]) * GRID - DOT_R;
			c[1] = fmin(p->xy[(2 * j) + 1], p->xy[(2 * k) + 1]) *
				       GRID -
			       DOT_R;
			c[2] = fmax(p->xy[2 * j], p->xy[2 * k]) * GRID + DOT_R;
			c[3] = fmax(p->xy[(2 * j) + 1], p->xy[(2 * k) + 1]) *
				       GRID +
			       DOT_R;
			if (boxes_meet(b, c)) {
				return 1;
			}
		}
	}
	return 0;
}

// Whether the box b meets part i: its body, its flag's text, its label
// or its pin numbers.
static int meets_part(struct draw *dr, size_t i, const double b[4])
{
	const struct sch_part *q = &dr->sch.part[i];
	const struct label *l = &dr->labels[i];
	struct text_at ta = { NAN, NAN, 0 };
	double c[4];

	body(dr, i, c);
	if (boxes_meet(b, c)) {
		return 1;
	}
	if (q->dev->shape->flag && flag_box(q, c, &ta) && boxes_meet(b, c)) {
		return 1;
	}
	if (l->nline) {
		const double t[4] = { l->x, l->y, l->x + l->w, l->y + l->h };

		if (boxes_meet(b, t)) {
			return 1;
		}
	}
	for (size_t p = 0; p < q->dev->shape->npin; p++) {
		if (pin_number_at(q, p, &ta)) {
			const double t[4] = { ta.x - 4, ta.y - 1, ta.x + 4,
					      ta.y + PINNUM + 1 };

			if (boxes_meet(b, t)) {
				return 1;
			}
		}
	}
	return 0;
}

// Whether a note's box, in points of the drawing, is clear of all that
// is drawn, a point apart, and inside the room (r, in points of the
// drawing): the wires and dots, the bodies of the parts and the flags,
// all their text, and the other notes' boxes.
static int note_clear(struct draw *dr, const double nb[4], const double r[4])
{
	double b[4] = { nb[0] - 1, nb[1] - 1, nb[2] + 1, nb[3] + 1 };

	if (nb[0] < r[0] || nb[1] < r[1] || nb[2] > r[2] || nb[3] > r[3]) {
		return 0;
	}
	if (meets_wires(dr, b)) {
		return 0;
	}
	for (size_t i = 0; i < dr->sch.npart; i++) {
		if (meets_part(dr, i, b)) {
			return 0;
		}
	}
	for (size_t i = 0; i < dr->nnote_box; i++) {
		if (boxes_meet(b, dr->note_box[i])) {
			return 0;
		}
	}
	return 1;
}

// Whether the group h of a note is g (both NULL outside any group).
static int same_group(const char *g, const char *h)
{
	if (g) {
		return h && !strcmp(g, h);
	}
	return !h;
}

// The box of the bodies of the parts of group g (of all parts for NULL)
// into e; the middle of the room r if none.
static void group_box(struct draw *dr, const char *g, const double r[4],
		      double e[4])
{
	size_t gl = g ? strlen(g) : 0;
	double c[4];

	e[0] = e[1] = BEST_NONE;
	e[2] = e[3] = -BEST_NONE;
	for (size_t k = 0; k < dr->sch.npart; k++) {
		if (dr->sch.part[k].dev->shape->flag ||
		    (g && (strncmp(dr->sch.part[k].name, g, gl) != 0 ||
			   dr->sch.part[k].name[gl] != '.'))) {
			continue;
		}
		body(dr, k, c);
		e[0] = fmin(e[0], c[0]);
		e[1] = fmin(e[1], c[1]);
		e[2] = fmax(e[2], c[2]);
		e[3] = fmax(e[3], c[3]);
	}
	if (e[2] < e[0]) {
		e[0] = e[2] = (r[0] + r[2]) / 2;
		e[1] = e[3] = (r[1] + r[3]) / 2;
	}
}

// A row of note boxes being placed: the room, the box of the group, the
// first box of the row among those placed, the boxes, and their side.
struct note_row {
	const double *r;
	double e[4];
	size_t first, n;
	double side;
};

// A corner of the box of a group, k, and the steps out along its
// diagonal.
struct corner {
	int k, step;
};

// Places the row of note boxes at the corner co of its group's box;
// returns whether all are clear there.
static int row_at(struct draw *dr, const struct note_row *nr, struct corner co)
{
	int k = co.k;
	int step = co.step;
	int right = k % 2;
	int up = k < 2;
	double side = nr->side;
	double d = step * GRID / 2;
	double x = right ? nr->e[2] + d : nr->e[0] - d - side;
	double y = up ? nr->e[3] + d : nr->e[1] - d - side;
	int ok = 1;

	dr->nnote_box = nr->first;
	for (size_t m = 0; ok && m < nr->n; m++) {
		double bx = x + ((right ? 1 : -1) * (double)m * (side + 2));
		double *nb = dr->note_box[dr->nnote_box];

		nb[0] = bx;
		nb[1] = y;
		nb[2] = bx + side;
		nb[3] = y + side;
		ok = note_clear(dr, nb, nr->r);
		dr->nnote_box += ok ? 1 : 0;
	}
	return ok;
}

// Places the row of note boxes: each corner in turn, a step further out
// each round: the upper left, the upper right, the lower left, the
// lower right, the row going out from the box.  Returns 0 if there is
// no room.
static int place_row(struct draw *dr, const struct note_row *nr)
{
	for (int step = 0; step < NOTE_STEPS; step++) {
		for (int k = 0; k < 4; k++) {
			if (row_at(dr, nr, (struct corner){ k, step })) {
				return 1;
			}
		}
	}
	return 0;
}

// Tells that the notes of group g from note i on have no room, and
// marks them done.
static void no_room(struct draw *dr, size_t i, const char *g, char *done)
{
	for (size_t j = i; j < dr->sch.nnote; j++) {
		if (!done[j] && same_group(g, dr->sch.note[j].group)) {
			done[j] = 1;
			(void)fprintf(stderr,
				      "%s:%d: warning: no room for the "
				      "note box of %s\n",
				      dr->sch.file, dr->sch.note[j].line,
				      g ? g : "-");
		}
	}
}

// Writes the note records of the notes of group g from note i on, at
// the boxes from first on, and marks them done.
static void put_notes(struct draw *dr, size_t i, size_t first, char *done)
{
	const char *g = dr->sch.note[i].group;

	for (size_t j = i, m = first; j < dr->sch.nnote; j++) {
		const double *nb = NULL;

		if (done[j] || !same_group(g, dr->sch.note[j].group)) {
			continue;
		}
		done[j] = 1;
		nb = dr->note_box[m++];
		printf("note %.3f %.3f %s %s\n",
		       dr->ox + ((nb[0] + nb[2]) / 2 * dr->sc),
		       dr->oy + ((nb[1] + nb[3]) / 2 * dr->sc), g ? g : "-",
		       dr->sch.note[j].text);
	}
}

// Writes a note record for each note of the page, its box placed by its
// group: at a corner of the box of the bodies of the group's parts (of
// all parts, for a note outside any group), stepped out along the
// diagonal until it is clear of all the drawing, the frame and the
// title block (the room, in points of the page); the notes of one group
// side by side.  The record gives the box's center; it is NOTE_BOX
// units square, for the viewers alone.  A group's notes without room
// are left out of the view.
static void write_notes(struct draw *dr, const double room[4])
{
	const double r[4] = { (room[0] - dr->ox) / dr->sc,
			      (room[1] - dr->oy) / dr->sc,
			      (room[2] - dr->ox) / dr->sc,
			      (room[3] - dr->oy) / dr->sc };
	char *done = sch_alloc(dr->sch.nnote + 1);

	dr->nnote_box = 0;
	dr->note_box = sch_alloc((dr->sch.nnote + 1) * sizeof *dr->note_box);
	for (size_t i = 0; i < dr->sch.nnote; i++) {
		const char *g = dr->sch.note[i].group;
		struct note_row nr = {
			r, { 0, 0, 0, 0 }, dr->nnote_box, 0, NOTE_BOX / dr->sc
		};

		if (done[i]) {
			continue;
		}
		group_box(dr, g, r, nr.e);
		for (size_t j = i; j < dr->sch.nnote; j++) {
			nr.n += !done[j] &&
				same_group(g, dr->sch.note[j].group);
		}
		if (!place_row(dr, &nr)) {
			dr->nnote_box = nr.first;
			no_room(dr, i, g, done);
			continue;
		}
		put_notes(dr, i, nr.first, done);
	}
	free(done);
	free(dr->note_box);
	dr->note_box = NULL;
}

// Opens the records of the net of the name: net NAME, and the class the
// page gives it, if any.
static void net_record(struct draw *dr, const char *name)
{
	for (size_t k = 0; k < dr->sch.nnet; k++) {
		if (!strcmp(dr->sch.net[k].name, name) &&
		    dr->sch.net[k].klass) {
			printf("net %s class %s\n", name, dr->sch.net[k].klass);
			return;
		}
	}
	printf("net %s\n", name);
}

// Writes the wires of net k, as lines, opening its records if it has
// any; returns whether it did.
static int net_wires(struct draw *dr, size_t k)
{
	int any = 0;

	for (size_t i = 0; i < dr->npolys; i++) {
		const struct poly *p = &dr->polys[i];
		double *xy = NULL;

		if (p->net != k || p->n < 2) {
			continue;
		}
		if (!any) {
			net_record(dr, dr->sch.net[k].name);
			any = 1;
		}
		xy = sch_alloc(2 * p->n * sizeof *xy);
		for (size_t j = 0; j < 2 * p->n; j++) {
			xy[j] = p->xy[j] * GRID;
		}
		printf("line %.3f", LW * dr->sc);
		points(dr, xy, p->n);
		free(xy);
	}
	return any;
}

// An end of a polyline: the polyline and its point.
struct poly_end {
	size_t i, j;
};

// Whether the end pe is an end of a polyline before its own, or the last
// end of it its first.
static int end_told(const struct draw *dr, struct poly_end pe)
{
	size_t i = pe.i;
	size_t j = pe.j;
	const struct poly *p = &dr->polys[i];
	int x = p->xy[2 * j];
	int y = p->xy[(2 * j) + 1];

	for (size_t m = 0; m < i; m++) {
		const struct poly *o = &dr->polys[m];

		if ((o->xy[0] == x && o->xy[1] == y) ||
		    (o->xy[(2 * o->n) - 2] == x &&
		     o->xy[(2 * o->n) - 1] == y)) {
			return 1;
		}
	}
	return j > 0 && p->n > 1 && p->xy[0] == x && p->xy[1] == y;
}

// Writes the junction dots of net k, once each, at the ends of its
// polylines.
static void net_dots(struct draw *dr, size_t k)
{
	for (size_t i = 0; i < dr->npolys; i++) {
		const struct poly *p = &dr->polys[i];

		for (size_t j = 0; p->net == k && j < p->n; j += p->n - 1) {
			int x = p->xy[2 * j];
			int y = p->xy[(2 * j) + 1];

			if (!end_told(dr, (struct poly_end){ i, j }) &&
			    dot_at(dr, x, y)) {
				printf("disk %.3f %.3f %.3f\n",
				       dr->ox + (x * GRID * dr->sc),
				       dr->oy + (y * GRID * dr->sc),
				       DOT_R * dr->sc);
			}
			if (p->n == 1) {
				break;
			}
		}
	}
}

// Writes the small cross at the end of each pin of q left unconnected on
// purpose.
static void nc_crosses(struct draw *dr, const struct sch_part *q)
{
	for (size_t p = 0; q->nc && p < q->dev->shape->npin; p++) {
		struct sch_at pin_at = sch_pin_at(q, p);
		double cx = dr->ox + (pin_at.x * GRID * dr->sc);
		double cy = dr->oy + (pin_at.y * GRID * dr->sc);
		double r = NC_R;

		if (!q->nc[p]) {
			continue;
		}
		printf("line %.3f %.3f %.3f %.3f %.3f\n", LW * dr->sc,
		       cx - (r * dr->sc), cy - (r * dr->sc), cx + (r * dr->sc),
		       cy + (r * dr->sc));
		printf("line %.3f %.3f %.3f %.3f %.3f\n", LW * dr->sc,
		       cx - (r * dr->sc), cy + (r * dr->sc), cx + (r * dr->sc),
		       cy - (r * dr->sc));
	}
}

// Writes the lines of the label b, aligned to the part: to the left of
// it flush right, above or below it centered, else flush left.
static void label_lines(struct draw *dr, const struct label *b)
{
	for (int l = 0; l < b->nline; l++) {
		struct text_at at = { b->x, b->y + b->h - TEXT - (l * LEAD),
				      0 };

		if (b->side == SCH_U || b->side == SCH_D) {
			at.x = b->x + (b->w / 2);
			at.align = 1;
		} else if (b->side == SCH_L) {
			at.x = b->x + b->w;
			at.align = 2;
		}
		text_runs(dr, b->run[l], b->nrun[l], at);
	}
}

// Writes part q: a flag, of its net, drawn lit with it; else its name,
// symbol, pins and label b.
static void write_part(struct draw *dr, const struct sch_part *q,
		       const struct label *b)
{
	double bx[4];

	if (q->dev->shape->flag) {
		net_record(dr, q->name);
		symbol(dr, q);
		flag_text(dr, q);
		pin_ends(dr, q);
		printf("net\n");
		return;
	}
	sch_box(q, bx);
	printf("part %s %s\nname %s %.3f %.3f %.3f %.3f\n", q->name, q->ref,
	       q->name, dr->ox + ((bx[0] * GRID - 2) * dr->sc),
	       dr->oy + ((bx[1] * GRID - 2) * dr->sc),
	       dr->ox + ((bx[2] * GRID + 2) * dr->sc),
	       dr->oy + ((bx[3] * GRID + 2) * dr->sc));
	symbol(dr, q);
	pin_numbers(dr, q);
	pin_ends(dr, q);
	nc_crosses(dr, q);
	label_lines(dr, b);
	printf("part\n");
}

// Writes the sheet as vec(5): the template, then each net's wires and
// dots, then each part, its symbol and its text, and the flags.
static void write_vec(struct draw *dr)
{
	double room[4];
	double paper[2];
	double e[4];

	sheet(dr, room, paper, 1);
	extent(dr, e);
	dr->sc = dr->sch.scale;
	if ((e[2] - e[0]) * GRID * dr->sc > room[2] - room[0] ||
	    (e[3] - e[1]) * GRID * dr->sc > room[3] - room[1]) {
		(void)fprintf(stderr,
			      "%s: the drawing is larger than the room of "
			      "the sheet\n",
			      dr->sch.file);
	}
	dr->ox = (room[0] + room[2]) / 2 - (e[0] + e[2]) / 2 * GRID * dr->sc;
	dr->oy = (room[1] + room[3]) / 2 - (e[1] + e[3]) / 2 * GRID * dr->sc;
	for (size_t k = 0; k < dr->sch.nnet; k++) {
		int any = net_wires(dr, k);

		net_dots(dr, k);
		if (any) {
			printf("net\n");
		}
	}
	for (size_t i = 0; i < dr->sch.npart; i++) {
		write_part(dr, &dr->sch.part[i], &dr->labels[i]);
	}
	write_notes(dr, room);
}

// The distance between two boxes, x0 y0 x1 y1.
static double apart(const double *a, const double *b)
{
	return hypot(fmax(0, fmax(b[0] - a[2], a[0] - b[2])),
		     fmax(0, fmax(b[1] - a[3], a[1] - b[3])));
}

// The box of a part's body, or of its label, in points.
static int body(struct draw *dr, size_t i, double b[4])
{
	sch_box(&dr->sch.part[i], b);
	for (int k = 0; k < 4; k++) {
		b[k] *= GRID;
	}
	return !dr->sch.part[i].dev->shape->flag;
}

static int label(struct draw *dr, size_t i, double b[4])
{
	const struct label *l = &dr->labels[i];

	b[0] = l->x;
	b[1] = l->y;
	b[2] = l->x + l->w;
	b[3] = l->y + l->h;
	return l->nline && l->side >= 0;
}

// The designator of part k for a message, or the kind of a flag.
static const char *ref_of(struct draw *dr, size_t k)
{
	return dr->sch.part[k].ref ? dr->sch.part[k].ref :
				     dr->sch.part[k].dev->shape->name;
}

// Whether a pin of part k is on net.
static int part_on_net(struct draw *dr, size_t k, size_t net)
{
	for (size_t r = 0; r < dr->sch.part[k].dev->shape->npin; r++) {
		if (net_of(dr, k, r) == (int)net) {
			return 1;
		}
	}
	return 0;
}

// Measures the room the wire w (a box in points) of polyline p keeps
// from the bodies of parts not on its net and from the labels.
static void check_wire(struct draw *dr, const struct poly *p, const double w[4])
{
	double b[4];

	for (size_t k = 0; k < dr->sch.npart; k++) {
		if (!part_on_net(dr, k, p->net) && body(dr, k, b) &&
		    apart(w, b) < dr->sch.clear_body - ROOM_SLACK) {
			(void)fprintf(stderr,
				      "%s:%d: a wire of %s runs %.1f pt from "
				      "%s (%s)\n",
				      dr->sch.file, dr->sch.part[k].line,
				      dr->sch.net[p->net].name, apart(w, b),
				      dr->sch.part[k].name, ref_of(dr, k));
			dr->failed = 1;
		}
		if (label(dr, k, b) &&
		    apart(w, b) < dr->sch.clear_text - ROOM_SLACK) {
			(void)fprintf(stderr,
				      "%s:%d: a wire of %s runs %.1f pt from "
				      "the label of %s (%s)\n",
				      dr->sch.file, dr->sch.part[k].line,
				      dr->sch.net[p->net].name, apart(w, b),
				      dr->sch.part[k].name, ref_of(dr, k));
			dr->failed = 1;
		}
	}
}

// Measures the room parts i and k keep: body from body, label from
// body, label from label; tells a pair too close.
static void check_pair(struct draw *dr, size_t i, size_t k)
{
	double b[4];
	double c[4];
	double e = ROOM_SLACK;
	const char *what = NULL;
	double room = dr->sch.clear_text;

	if (k > i && body(dr, i, b) && body(dr, k, c) &&
	    apart(b, c) < dr->sch.clear_body - e) {
		what = "";
		room = apart(b, c);
	} else if (k != i && label(dr, i, b) && body(dr, k, c) &&
		   apart(b, c) < dr->sch.clear_text - e) {
		what = "the label of ";
		room = apart(b, c);
	} else if (k > i && label(dr, i, b) && label(dr, k, c) &&
		   apart(b, c) < dr->sch.clear_text - e) {
		what = "the labels of ";
		room = apart(b, c);
	}
	if (what) {
		(void)fprintf(stderr,
			      "%s:%d: %s%s (%s) is %.1f pt from %s (%s)\n",
			      dr->sch.file, dr->sch.part[i].line, what,
			      dr->sch.part[i].name, ref_of(dr, i), room,
			      dr->sch.part[k].name, ref_of(dr, k));
		dr->failed = 1;
	}
}

// Measures the room the drawing keeps: a wire from the bodies of parts
// not on its net, a body from a body, and a label from every wire, body
// and other label; each pair too close is a fault.
static void check_room(struct draw *dr)
{
	for (size_t i = 0; i < dr->npolys; i++) {
		const struct poly *p = &dr->polys[i];

		for (size_t j = 1; j < p->n; j++) {
			double w[4] = {
				fmin(p->xy[(2 * j) - 2], p->xy[2 * j]) * GRID,
				fmin(p->xy[(2 * j) - 1], p->xy[(2 * j) + 1]) *
					GRID,
				fmax(p->xy[(2 * j) - 2], p->xy[2 * j]) * GRID,
				fmax(p->xy[(2 * j) - 1], p->xy[(2 * j) + 1]) *
					GRID
			};

			check_wire(dr, p, w);
		}
	}
	for (size_t i = 0; i < dr->sch.npart; i++) {
		for (size_t k = 0; k < dr->sch.npart; k++) {
			check_pair(dr, i, k);
		}
	}
}

// A thing of the drawing on one axis: its span, in grid units, the
// part (with its label and the text of its flag) or the wire it is,
// whether it is a flag, and the block it falls in.
struct span {
	double lo, hi;
	size_t i;
	int wire, flag, block;
};

static int by_lo(const void *lhs, const void *rhs)
{
	const struct span *p = lhs;
	const struct span *q = rhs;
	int f = (p->flag > q->flag) - (p->flag < q->flag);

	return f ? f : (p->lo > q->lo) - (p->lo < q->lo);
}

// The spans of the parts, with their pins, and of the wires of the
// drawing on axis a, the flags last; their number comes back.
static size_t spans(struct draw *dr, size_t a, struct span *sp)
{
	size_t n = 0;

	for (size_t i = 0; i < dr->sch.npart; i++) {
		double b[4];
		double t[4];
		struct text_at ta = { NAN, NAN, 0 };
		int flag = dr->sch.part[i].dev->shape->flag != NULL;

		body(dr, i, b);
		for (size_t p = 0; p < dr->sch.part[i].dev->shape->npin; p++) {
			int px = 0;
			int py = 0;

			{
				struct sch_at pin_at =
					sch_pin_at(&dr->sch.part[i], p);
				px = pin_at.x;
				py = pin_at.y;
			}
			b[0] = fmin(b[0], px * GRID);
			b[1] = fmin(b[1], py * GRID);
			b[2] = fmax(b[2], px * GRID);
			b[3] = fmax(b[3], py * GRID);
		}
		if (label(dr, i, t) ||
		    (flag && flag_box(&dr->sch.part[i], t, &ta))) {
			b[0] = fmin(b[0], t[0]);
			b[1] = fmin(b[1], t[1]);
			b[2] = fmax(b[2], t[2]);
			b[3] = fmax(b[3], t[3]);
		}
		sp[n++] = (struct span){ b[a] / GRID, b[a + 2] / GRID,
					 i,	      0,
					 flag,	      0 };
	}
	for (size_t i = 0; i < dr->npolys; i++) {
		struct span w = { FAR_GRID, -FAR_GRID, i, 1, 0, 0 };

		for (size_t j = 0; j < dr->polys[i].n; j++) {
			w.lo = fmin(w.lo, dr->polys[i].xy[(2 * j) + a]);
			w.hi = fmax(w.hi, dr->polys[i].xy[(2 * j) + a]);
		}
		sp[n++] = w;
	}
	qsort(sp, n, sizeof *sp, by_lo);
	return n;
}

// Moves thing s by d grid units along axis a.
static void move_span(struct draw *dr, size_t a, const struct span *s, int d)
{
	if (s->wire) {
		for (size_t j = 0; j < dr->polys[s->i].n; j++) {
			dr->polys[s->i].xy[(2 * j) + a] += d;
		}
	} else {
		*(a ? &dr->sch.part[s->i].y : &dr->sch.part[s->i].x) += d;
		*(a ? &dr->labels[s->i].y : &dr->labels[s->i].x) += d * GRID;
	}
}

// Spreads the blocks of the drawing along axis a (0 across, 1 up): the
// runs of parts and wires that nothing else of the drawing overlaps on
// that axis.  A flag (a ground, a supply or a port) makes no block of
// its own: it goes with the block it overlaps, or else the nearest.  The
// free length of the room of the sheet is shared equally by the two
// margins and the gaps between the blocks, a gap wider than its share
// closing up to it, if the share is SPREAD_MIN or more.  No wire runs
// across a gap, so the blocks move whole.  The drawing is centered in
// the room later, so the margins come out as wide as the gaps.
// The blocks of the spans: where each starts and ends, and their number.
struct blocks {
	double *lo, *end;
	size_t n;
};

// Gathers the spans sp[0] .. sp[n-1] but the flags (which come last, as
// they are sorted) into blocks that overlap none of the others.
static void make_blocks(struct span *sp, size_t n, struct blocks *bl)
{
	double hi = -FAR_GRID;

	for (size_t k = 0; k < n && !sp[k].flag; k++) {
		if (sp[k].lo > hi) {
			bl->lo[bl->n++] = sp[k].lo;
		}
		sp[k].block = (int)bl->n - 1;
		hi = fmax(hi, sp[k].hi);
		bl->end[bl->n - 1] = hi;
	}
}

// How far the flag span f lies outside block m, -1 if it overlaps it.
static double block_dist(const struct span *f, const struct blocks *bl,
			 size_t m)
{
	double mid = (f->lo + f->hi) / 2;

	if (f->hi < bl->lo[m]) {
		return bl->lo[m] - mid;
	}
	if (f->lo > bl->end[m]) {
		return mid - bl->end[m];
	}
	return -1;
}

// Puts each flag span in the block nearest it, and widens the blocks to
// their spans.
static void place_flags(struct span *sp, size_t n, struct blocks *bl)
{
	for (size_t k = 0; k < n; k++) {
		double best = FAR_GRID;

		for (size_t m = 0; sp[k].flag && m < bl->n; m++) {
			double d = block_dist(&sp[k], bl, m);

			if (d < best) {
				best = d;
				sp[k].block = (int)m;
			}
		}
	}
	for (size_t k = 0; k < n; k++) {
		bl->lo[sp[k].block] = fmin(bl->lo[sp[k].block], sp[k].lo);
		bl->end[sp[k].block] = fmax(bl->end[sp[k].block], sp[k].hi);
	}
}

// Moves the blocks of the spans so that the gaps between them are gap.
static void move_blocks(struct draw *dr, size_t a, const struct span *sp,
			size_t n, const struct blocks *bl, double gap)
{
	int *shift = sch_alloc((bl->n + 1) * sizeof *shift);

	for (size_t m = 1; m < bl->n; m++) {
		shift[m] = shift[m - 1] +
			   (int)floor(gap - (bl->lo[m] - bl->end[m - 1]));
	}
	for (size_t k = 0; k < n; k++) {
		move_span(dr, a, &sp[k], shift[sp[k].block]);
	}
	free(shift);
}

static void spread(struct draw *dr, size_t a)
{
	double room[4];
	double paper[2];
	double sum = 0;
	double gap = NAN;
	size_t n = 0;
	struct span *sp =
		sch_alloc((dr->sch.npart + dr->npolys + 1) * sizeof *sp);
	struct blocks bl = { NULL, NULL, 0 };
	int ok = 1;

	sheet(dr, room, paper, 0);
	n = spans(dr, a, sp);
	bl.lo = sch_alloc((n + 1) * sizeof *bl.lo);
	bl.end = sch_alloc((n + 1) * sizeof *bl.end);
	make_blocks(sp, n, &bl);
	if (bl.n) {
		place_flags(sp, n, &bl);
	}
	for (size_t m = 0; m < bl.n; m++) {
		sum += bl.end[m] - bl.lo[m];
		ok = ok && (m == 0 || bl.lo[m] > bl.end[m - 1]);
	}
	gap = ((room[a + 2] - room[a]) / (GRID * dr->sch.scale) - sum) /
	      (double)(bl.n + 1);
	if (ok && bl.n > 1 && gap >= SPREAD_MIN) {
		move_blocks(dr, a, sp, n, &bl, gap);
		if (dr->verbose) {
			(void)fprintf(stderr,
				      "%s: %zu blocks %s, %.0f grid apart\n",
				      dr->sch.file, bl.n,
				      a ? "up the sheet" : "across the sheet",
				      gap);
		}
	}
	free(sp);
	free(bl.lo);
	free(bl.end);
}

// Leaves out of the drawing the parts that found no place (the reader
// told why), and their pins out of the nets and the wires.
static void drop_unplaced(struct draw *dr)
{
	size_t *to = sch_alloc((dr->sch.npart + 1) * sizeof *to);
	size_t n = 0;
	size_t m = 0;

	for (size_t i = 0; i < dr->sch.npart; i++) {
		to[i] = dr->sch.part[i].placed ? n : (size_t)-1;
		if (dr->sch.part[i].placed) {
			dr->sch.part[n++] = dr->sch.part[i];
		}
	}
	dr->sch.npart = n;
	for (size_t k = 0; k < dr->sch.nnet; k++) {
		struct sch_net *t = &dr->sch.net[k];

		m = 0;
		for (size_t j = 0; j < t->nend; j++) {
			if (to[t->end[j].part] != (size_t)-1) {
				t->end[m] = t->end[j];
				t->end[m++].part = to[t->end[j].part];
			}
		}
		t->nend = m;
	}
	m = 0;
	for (size_t h = 0; h < dr->sch.nhint; h++) {
		struct sch_hint hn = dr->sch.hint[h];

		if (to[hn.part[0]] != (size_t)-1 &&
		    to[hn.part[1]] != (size_t)-1) {
			hn.part[0] = to[hn.part[0]];
			hn.part[1] = to[hn.part[1]];
			dr->sch.hint[m++] = hn;
		}
	}
	dr->sch.nhint = m;
	free(to);
}

// Whether page pg has the port of the name.
static int page_has_port(const struct sch_page *pg, const char *name)
{
	for (size_t j = 0; j < pg->nport; j++) {
		if (!strcmp(pg->port[j], name)) {
			return 1;
		}
	}
	return 0;
}

// Gives each port of the sheet the pages it is on, its own too, by
// number, after its name, in brackets: BUF(1,2); a port on one page
// alone has none.
static void cross_refs(struct draw *dr)
{
	for (size_t i = 0; i < dr->sch.npart; i++) {
		struct sch_part *q = &dr->sch.part[i];
		size_t name_len = strlen(q->name);
		size_t size = name_len + 2 + (PAGE_LEN * dr->sch.npages);
		size_t len = name_len;
		char *t = NULL;
		int any = 0;

		if (!q->dev->shape->flag ||
		    strcmp(q->dev->shape->flag, "port") != 0) {
			continue;
		}
		t = sch_alloc(size);
		memcpy(t, q->name, name_len + 1);
		for (size_t p = 0; p < dr->sch.npages; p++) {
			const struct sch_page *pg = &dr->sch.pages[p];

			if (page_has_port(pg, q->name)) {
				int k = snprintf(t + len, size - len, "%c%d",
						 any ? ',' : '(', pg->num);

				any++;
				len += k > 0 ? (size_t)k : 0;
			}
		}
		if (any > 1) {
			t[len++] = ')';
			t[len] = 0;
		} else {
			t[name_len] = 0;
		}
		q->text = t;
	}
}

// Frees the drawing of a page, for the next.
static void clear_page(struct draw *dr)
{
	for (size_t i = 0; i < dr->npolys; i++) {
		free(dr->polys[i].xy);
	}
	for (size_t g = 0; g < dr->ngroups; g++) {
		free(dr->groups[g].end);
		free(dr->groups[g].edge);
	}
	for (size_t i = 0; i < dr->sch.npart; i++) {
		for (int l = 0; l < dr->labels[i].nline; l++) {
			for (int r = 0; r < dr->labels[i].nrun[l]; r++) {
				free(dr->labels[i].run[l][r].s);
			}
		}
	}
	free(dr->polys);
	free(dr->groups);
	free(dr->labels);
	free(dr->blocked);
	free(dr->pinnet);
	free(dr->eown);
	free(dr->heap);
	free(dr->cost);
	free(dr->from);
	dr->polys = NULL;
	dr->groups = NULL;
	dr->labels = NULL;
	dr->blocked = NULL;
	dr->pinnet = dr->eown = NULL;
	dr->heap = dr->cost = dr->from = NULL;
	dr->npolys = dr->ngroups = dr->nheap = 0;
}

int main(int argc, char **argv)
{
	struct sch_lib sl = { NULL, 0, NULL, 0, NULL, 0, 0 };
	struct draw ctx;
	char **lib = NULL;
	size_t nlib = 0;
	size_t npage = 0;
	int i = 1;
	int faults = 0;

	memset(&ctx, 0, sizeof ctx);
	for (; i < argc && argv[i][0] == '-'; i++) {
		if (strcmp(argv[i], "-v") == 0) {
			ctx.verbose = 1;
		} else if (strcmp(argv[i], "-L") == 0 && i + 1 < argc) {
			size_t n = strlen(argv[i + 1]) + sizeof "/symbols";

			lib = (char **)xrealloc((void *)lib,
						(nlib + 1) * sizeof *lib);
			lib[nlib] = sch_alloc(n);
			(void)snprintf(lib[nlib++], n, "%s/symbols", argv[++i]);
		} else {
			sch_die("usage: schdraw [-v] [-L LIB]... sheet.sch > sheet.vec");
		}
	}
	if (i != argc - 1) {
		sch_die("usage: schdraw [-v] [-L LIB]... sheet.sch > sheet.vec");
	}
	if (!nlib) {
		lib = (char **)sch_alloc(sizeof *lib);
		lib[0] = sch_default_lib(argv[0]);
		nlib = 1;
	}
	ctx.pages = sch_alloc(sizeof *ctx.pages);
	sl.dir = lib;
	sl.ndir = nlib;
	sl.verbose = ctx.verbose;
	faults = sch_read(&ctx.pages[0], argv[i], 0, &sl);
	npage = ctx.pages[0].npages ? ctx.pages[0].npages : 1;
	ctx.pages = xrealloc(ctx.pages, npage * sizeof *ctx.pages);
	printf("# vec of %s, by schdraw\n", argv[i]);
	for (size_t k = 0; k < npage; k++) {
		if (k) {
			faults += sch_read(&ctx.pages[k], argv[i], k, &sl);
		}
		ctx.sch = ctx.pages[k];
		cross_refs(&ctx);
		drop_unplaced(&ctx);
		grid(&ctx);
		make_groups(&ctx);
		route_all(&ctx);
		trace(&ctx);
		labels_all(&ctx);
		spread(&ctx, 0);
		spread(&ctx, 1);
		check_room(&ctx);
		write_vec(&ctx);
		clear_page(&ctx);
	}
	for (size_t k = 0; k < nlib; k++) {
		free(lib[k]);
	}
	free((void *)lib);
	return ctx.failed || faults;
}
