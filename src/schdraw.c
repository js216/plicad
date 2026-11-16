// SPDX-License-Identifier: GPL-3.0-only
// schdraw.c -- Draw a schematic sheet as plain vectors
// Copyright (c) 2026 Jakob Kastelic

// Usage: schdraw [-v] [-L LIB]... sheet.sch > sheet.vec
//
// Reads the source (sch(5)), which the reader places, and the symbols
// it names from the project (symbols and sheets next to the source) and
// the library directories (LIB/symbols, LIB/sheets; by default ../lib
// next to the program); routes the wires; and writes the page as
// vec(5): the template of the sheet, each net's wires and dots, and each
// part's symbol and text, the parts and nets named, the text as text,
// and where each pin and flag meets its wire, so a test (schconn) can
// tell from the drawing alone that it joins what the netlist joins.
// vecps prints it.
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
// next to a body or a foreign wire, a junction next to another and, for
// a wire the source gives a side, straying off that side cost more than
// length.  The smallest groups are routed first.  A group that finds no
// way is put first and all are routed again (rip-up); once all are
// routed, each is ripped up and routed again among the others, and its
// new path kept if it costs no more.
//
// The room the drawing keeps, round bodies and text, is the sheet
// template's (its clear records); each pair of things too close is
// printed, and the exit status is 1.  With -v each placement is told on
// stderr, with its line and the rule it came from.

#include "schsrc.h"
#include <limits.h>
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
#define SUB SCH_SUB
#define SUB_DROP 1.6 // How far a subscript drops.
#define LEAD SCH_LEAD
#define DESCENT SCH_DESCENT
#define PINNUM 5.0 // The pin numbers of an integrated circuit.
#define MARGIN 8 // Grid units round everything, for the router.

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

static const int DX[4] = { 1, 0, -1, 0 }, DY[4] = { 0, 1, 0, -1 };

// A run of text in one font: italic or not, its size and how far it
// drops below the baseline.
struct run {
	char *s;
	int italic;
	double size, drop;
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

static struct sch S;
static struct poly *polys;
static size_t npolys;
static struct label *labels; // One for each part, if it has one.
static int failed;

// The grid of the router: the bodies, the pins (the net of each, -1
// none, -2 a pin on no net) and the edges (the group whose wire takes
// each, -1 none): edge 2a runs right from point a, edge 2a+1 up.
static int X0, Y0, W, H;
static unsigned char *blocked;
static int *pinnet, *eown;
static struct group *groups;
static size_t ngroups;

static void *xrealloc(void *p, size_t n)
{
	p = realloc(p, n ? n : 1);
	if (!p) {
		sch_die("out of memory");
	}
	return p;
}

static int idx(int x, int y)
{
	return (y - Y0) * W + (x - X0);
}

static int inside(int x, int y)
{
	return x >= X0 && x < X0 + W && y >= Y0 && y < Y0 + H;
}

// The net of pin p of part i, or -1.
static int net_of(size_t i, size_t p)
{
	for (size_t k = 0; k < S.nnet; k++) {
		for (size_t j = 0; j < S.net[k].nend; j++) {
			if (S.net[k].end[j].part == i &&
			    S.net[k].end[j].pin == p) {
				return (int)k;
			}
		}
	}
	return -1;
}

static void add_poly(const int *xy, size_t n, size_t net)
{
	struct poly p;

	p.xy = sch_alloc(2 * n * sizeof *p.xy);
	memcpy(p.xy, xy, 2 * n * sizeof *xy);
	p.n = n;
	p.net = net;
	polys = xrealloc(polys, (npolys + 1) * sizeof *polys);
	polys[npolys++] = p;
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

// Sets up the grid: its extent, the bodies and the pins.
static void grid(void)
{
	int x0 = INT_MAX, y0 = INT_MAX, x1 = INT_MIN, y1 = INT_MIN;

	if (!S.npart) {
		sch_die("%s: nothing to draw", S.file);
	}
	for (size_t i = 0; i < S.npart; i++) {
		double b[4];

		sch_box(&S.part[i], b);
		x0 = x0 < lo(b[0]) ? x0 : lo(b[0]);
		y0 = y0 < lo(b[1]) ? y0 : lo(b[1]);
		x1 = x1 > hi(b[2]) ? x1 : hi(b[2]);
		y1 = y1 > hi(b[3]) ? y1 : hi(b[3]);
		for (size_t p = 0; p < S.part[i].dev->shape->npin; p++) {
			int x, y, d;

			sch_pin_at(&S.part[i], p, &x, &y, &d);
			x0 = x < x0 ? x : x0;
			y0 = y < y0 ? y : y0;
			x1 = x > x1 ? x : x1;
			y1 = y > y1 ? y : y1;
		}
	}
	X0 = x0 - MARGIN;
	Y0 = y0 - MARGIN;
	W = x1 - x0 + 2 * MARGIN + 1;
	H = y1 - y0 + 2 * MARGIN + 1;
	blocked = sch_alloc((size_t)W * (size_t)H);
	pinnet = sch_alloc((size_t)W * (size_t)H * sizeof *pinnet);
	eown = sch_alloc(2 * (size_t)W * (size_t)H * sizeof *eown);
	for (int i = 0; i < W * H; i++) {
		pinnet[i] = -1;
		eown[2 * i] = eown[2 * i + 1] = -1;
	}
	// A body and the room round it are no place for a wire.
	for (size_t i = 0; i < S.npart; i++) {
		double b[4], m = S.part[i].dev->shape->flag ?
					 0 :
					 S.clear_body / GRID;

		sch_box(&S.part[i], b);
		for (int y = hi(b[1] - m); y <= lo(b[3] + m); y++) {
			for (int x = hi(b[0] - m); x <= lo(b[2] + m); x++) {
				if (x > b[0] - m && x < b[2] + m &&
				    y > b[1] - m && y < b[3] + m) {
					blocked[idx(x, y)] = 1;
				}
			}
		}
	}
	for (size_t i = 0; i < S.npart; i++) {
		for (size_t p = 0; p < S.part[i].dev->shape->npin; p++) {
			int x, y, d, n = net_of(i, p), a;

			sch_pin_at(&S.part[i], p, &x, &y, &d);
			a = idx(x, y);
			if (pinnet[a] != -1) {
				sch_die("%s:%d: two pins meet at %d %d", S.file,
					S.part[i].line, x, y);
			}
			pinnet[a] = n < 0 ? -2 : n;
			blocked[a] = 0;
		}
	}
}

// The edge from point a the way d, or -1 off the grid.
static int edge(int a, int d)
{
	int x = a % W, y = a / W;

	switch (d) {
	case SCH_R:
		return x + 1 < W ? 2 * a : -1;
	case SCH_U:
		return y + 1 < H ? 2 * a + 1 : -1;
	case SCH_L:
		return x > 0 ? 2 * (a - 1) : -1;
	default:
		return y > 0 ? 2 * (a - W) + 1 : -1;
	}
}

// The ways from point a whose edges group g takes (g < 0: any group).
static unsigned ways(int a, int g)
{
	unsigned m = 0;

	for (int d = 0; d < 4; d++) {
		int e = edge(a, d);

		if (e >= 0 && eown[e] >= 0 && (g < 0 || eown[e] == g)) {
			m |= 1u << d;
		}
	}
	return m;
}

static int bits(unsigned m)
{
	int n = 0;

	for (; m; m >>= 1) {
		n += (int)(m & 1);
	}
	return n;
}

// The heap of Dijkstra's method: states are point * 4 + the way in.
static int *heap, *cost, *from;
static size_t nheap;

static void push(int s)
{
	size_t i = nheap++;

	heap[i] = s;
	while (i > 0) {
		size_t up = (i - 1) / 2;
		int t;

		if (cost[heap[up]] <= cost[heap[i]]) {
			break;
		}
		t = heap[up];
		heap[up] = heap[i];
		heap[i] = t;
		i = up;
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

		if (l < nheap && cost[heap[l]] < cost[heap[m]]) {
			m = l;
		}
		if (r < nheap && cost[heap[r]] < cost[heap[m]]) {
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

// The extra cost of a point for group g: next to a body or another
// group's wire.
static int near_cost(int x, int y, int g)
{
	int c = 0;

	for (int d = 0; d < 4; d++) {
		int nx = x + DX[d], ny = y + DY[d], a;

		if (!inside(nx, ny)) {
			continue;
		}
		a = idx(nx, ny);
		if (blocked[a] || (ways(a, -1) & ~ways(a, g))) {
			c += NEAR;
		}
	}
	return c;
}

// Whether a junction, or a point that would become one, stands next to
// (x, y): two dots side by side read as a smudge.
static int next_to_dot(int x, int y)
{
	for (int d = 0; d < 4; d++) {
		int nx = x + DX[d], ny = y + DY[d];

		if (inside(nx, ny) && bits(ways(idx(nx, ny), -1)) >= 3) {
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
static int stray(const struct keep *k, int x, int y)
{
	int near = 0, off;

	if (!k || k->side < 0) {
		return 0;
	}
	for (int i = 0; i < 2; i++) {
		near |= abs(x - k->x[i]) + abs(y - k->y[i]) <= 1;
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

// Routes group g from the pin at (sx, sy), which points sd, to its tree:
// a point of its wires where a T may be made, or, if the tree has no
// wire yet, the pin at (tx, ty), which points td.  Lays the path and
// adds its cost; returns 0 if there is none.
static int route(int g, int sx, int sy, int sd, int tx, int ty, int td,
		 const struct keep *k)
{
	int n = W * H * 4, goal = -1, s0x = sx + DX[sd], s0y = sy + DY[sd];

	for (int i = 0; i < n; i++) {
		cost[i] = INT_MAX;
		from[i] = -1;
	}
	nheap = 0;
	if (!inside(s0x, s0y)) {
		return 0;
	}
	{
		int a = idx(s0x, s0y);
		unsigned other = ways(a, -1) & ~ways(a, g);

		if ((blocked[a] && !(s0x == tx && s0y == ty)) ||
		    (pinnet[a] != -1 && !(s0x == tx && s0y == ty)) ||
		    (other &&
		     (other != ((sd % 2) ? 0x5u : 0xAu) || ways(a, g)))) {
			return 0;
		}
		cost[a * 4 + sd] = STEP;
		push(a * 4 + sd);
	}
	while (nheap) {
		int s = pop(), a = s / 4, d = s % 4;
		int x = a % W + X0, y = a / W + Y0;
		unsigned mine = ways(a, g), other = ways(a, -1) & ~mine;

		if (x == tx && y == ty) {
			if (d == (td + 2) % 4) {
				goal = s;
				break;
			}
			continue;
		}
		if (mine && pinnet[a] == -1) {
			if (bits(mine) <= 2 &&
			    !(mine & (1u << ((d + 2) % 4))) && !other) {
				goal = s;
				break;
			}
			continue;
		}
		for (int nd = 0; nd < 4; nd++) {
			int nx = x + DX[nd], ny = y + DY[nd], b, ns, c, e;
			unsigned bm, bo;
			int crossing;

			if (nd == (d + 2) % 4 || !inside(nx, ny)) {
				continue;
			}
			// Through a wire of another group only straight across.
			if (other && nd != d) {
				continue;
			}
			e = edge(a, nd);
			if (e < 0 || eown[e] >= 0) {
				continue;
			}
			b = idx(nx, ny);
			if (nx == tx && ny == ty) {
				if (nd != (td + 2) % 4) {
					continue;
				}
			} else if (blocked[b] || pinnet[b] != -1) {
				continue;
			}
			bm = ways(b, g);
			bo = ways(b, -1) & ~bm;
			crossing = bo != 0;
			if (crossing &&
			    (bm || bo != ((nd % 2) ? 0x5u : 0xAu))) {
				continue;
			}
			if (bm &&
			    (bits(bm) > 2 || (bm & (1u << ((nd + 2) % 4))))) {
				continue;
			}
			c = cost[s] + STEP + (nd != d ? BEND : 0) +
			    (crossing ? CROSS : 0) + near_cost(nx, ny, g) +
			    stray(k, nx, ny);
			if (bm && next_to_dot(nx, ny)) {
				c += CROWD;
			}
			ns = b * 4 + nd;
			if (c < cost[ns]) {
				cost[ns] = c;
				from[ns] = s;
				push(ns);
			}
		}
	}
	if (goal < 0) {
		return 0;
	}
	groups[g].cost += cost[goal];
	for (int s = goal; s >= 0; s = from[s]) {
		int a = s / 4, d = s % 4, e = edge(a, (d + 2) % 4);
		struct group *gr = &groups[g];

		eown[e] = g;
		gr->edge =
			xrealloc(gr->edge, (gr->nedge + 1) * sizeof *gr->edge);
		gr->edge[gr->nedge++] = e;
	}
	return 1;
}

// Takes the wires of group g off the grid.
static void rip(size_t g)
{
	for (size_t i = 0; i < groups[g].nedge; i++) {
		eown[groups[g].edge[i]] = -1;
	}
	groups[g].nedge = 0;
	groups[g].cost = 0;
}

static void end_at(const struct sch_end *e, int *x, int *y, int *d)
{
	sch_pin_at(&S.part[e->part], e->pin, x, y, d);
}

// Routes group g as a tree.  Returns 0 if a pin finds no way.
static int route_group(size_t g)
{
	struct group *gr = &groups[g];
	const struct sch_net *t = &S.net[gr->net];
	size_t m = gr->nend;
	int *px = sch_alloc(m * sizeof *px), *py = sch_alloc(m * sizeof *py);
	int *pd = sch_alloc(m * sizeof *pd),
	    *done = sch_alloc(m * sizeof *done);
	int wired = 0, ok = 1;

	for (size_t j = 0; j < m; j++) {
		end_at(&t->end[gr->end[j]], &px[j], &py[j], &pd[j]);
	}
	// The wires the source names, with their sides, first.
	for (size_t h = 0; h < S.nhint; h++) {
		const struct sch_hint *hn = &S.hint[h];
		long a = -1, b = -1;

		for (size_t j = 0; j < m; j++) {
			const struct sch_end *e = &t->end[gr->end[j]];

			for (int i = 0; i < 2; i++) {
				if (e->part == hn->part[i] &&
				    e->pin == hn->pin[i]) {
					*(i ? &b : &a) = (long)j;
				}
			}
		}
		if (a < 0 || b < 0 || (done[a] && done[b])) {
			continue;
		}
		{
			struct keep k = { hn->side,
					  { px[a], px[b] },
					  { py[a], py[b] } };

			if (!wired) {
				ok = route((int)g, px[b], py[b], pd[b], px[a],
					   py[a], pd[a], &k);
			} else if (!done[b]) {
				ok = route((int)g, px[b], py[b], pd[b], INT_MIN,
					   INT_MIN, 0, &k);
			} else {
				ok = route((int)g, px[a], py[a], pd[a], INT_MIN,
					   INT_MIN, 0, &k);
			}
		}
		if (!ok) {
			goto out;
		}
		done[a] = done[b] = 1;
		wired = 1;
	}
	// The spine: the two pins that face each other on one line
	// farthest apart.
	if (!wired) {
		int far = -1;
		size_t si = 0, sj = 0;

		for (size_t i = 0; i < m; i++) {
			for (size_t j = 0; j < m; j++) {
				int len =
					abs(px[j] - px[i]) + abs(py[j] - py[i]);

				if (i != j && pd[j] == (pd[i] + 2) % 4 &&
				    px[i] + DX[pd[i]] * len == px[j] &&
				    py[i] + DY[pd[i]] * len == py[j] &&
				    len > far) {
					far = len;
					si = i;
					sj = j;
				}
			}
		}
		if (far > 0 && route((int)g, px[sj], py[sj], pd[sj], px[si],
				     py[si], pd[si], NULL)) {
			done[si] = done[sj] = 1;
			wired = 1;
		} else {
			done[0] = 1;
		}
	}
	for (;;) {
		size_t best = m;
		int bd = INT_MAX;

		for (size_t j = 0; j < m; j++) {
			for (size_t i = 0; !done[j] && i < m; i++) {
				int dd =
					abs(px[i] - px[j]) + abs(py[i] - py[j]);

				if (done[i] && dd < bd) {
					bd = dd;
					best = j;
				}
			}
		}
		if (best == m) {
			break;
		}
		if (!wired) {
			size_t first = 0;

			while (!done[first]) {
				first++;
			}
			ok = route((int)g, px[best], py[best], pd[best],
				   px[first], py[first], pd[first], NULL);
		} else {
			ok = route((int)g, px[best], py[best], pd[best],
				   INT_MIN, INT_MIN, 0, NULL);
		}
		if (!ok) {
			break;
		}
		wired = 1;
		done[best] = 1;
	}
out:
	free(px);
	free(py);
	free(pd);
	free(done);
	return ok;
}

// The groups to wire: the pins of one group number of a net.
static void make_groups(void)
{
	size_t n = 0;

	// At most one group for each pin of each net.
	for (size_t k = 0; k < S.nnet; k++) {
		n += S.net[k].nend;
	}
	groups = sch_alloc((n + 1) * sizeof *groups);
	for (size_t k = 0; k < S.nnet; k++) {
		const struct sch_net *t = &S.net[k];

		for (size_t j = 0; j < t->nend; j++) {
			int seen = 0;
			struct group *gr = &groups[ngroups];

			for (size_t i = 0; i < j; i++) {
				seen |= t->end[i].group == t->end[j].group;
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
			ngroups++;
		}
	}
}

// The size of a group, to order them: its pins and its extent.
static int group_size(size_t g)
{
	int x0 = INT_MAX, y0 = INT_MAX, x1 = INT_MIN, y1 = INT_MIN;

	if (!groups[g].nend) {
		return 0;
	}
	for (size_t j = 0; j < groups[g].nend; j++) {
		int x, y, d;

		end_at(&S.net[groups[g].net].end[groups[g].end[j]], &x, &y, &d);
		x0 = x < x0 ? x : x0;
		y0 = y < y0 ? y : y0;
		x1 = x > x1 ? x : x1;
		y1 = y > y1 ? y : y1;
	}
	return (x1 - x0) + (y1 - y0) + 4 * (int)groups[g].nend;
}

// Routes all groups: in order, the one that fails put first and all
// again; then each ripped up and routed again among the rest.
static void route_all(void)
{
	size_t *order = sch_alloc((ngroups + 1) * sizeof *order);
	int n = W * H * 4, ok = 0;

	heap = sch_alloc((size_t)n * 4 * sizeof *heap);
	cost = sch_alloc((size_t)n * sizeof *cost);
	from = sch_alloc((size_t)n * sizeof *from);
	for (size_t g = 0; g < ngroups; g++) {
		order[g] = g;
	}
	for (size_t i = 1; i < ngroups; i++) {
		for (size_t j = i;
		     j > 0 && group_size(order[j]) < group_size(order[j - 1]);
		     j--) {
			size_t t = order[j];

			order[j] = order[j - 1];
			order[j - 1] = t;
		}
	}
	for (int tries = 0; tries < TRIES && !ok; tries++) {
		ok = 1;
		for (size_t g = 0; g < ngroups; g++) {
			rip(g);
		}
		for (size_t o = 0; o < ngroups; o++) {
			if (!route_group(order[o])) {
				size_t bad = order[o];

				memmove(order + 1, order, o * sizeof *order);
				order[0] = bad;
				ok = 0;
				break;
			}
		}
	}
	if (!ok) {
		// The rest is drawn, the groups with no way without wires.
		for (size_t g = 0; g < ngroups; g++) {
			rip(g);
		}
		for (size_t o = 0; o < ngroups; o++) {
			if (!route_group(order[o])) {
				const struct sch_net *t =
					&S.net[groups[order[o]].net];

				fprintf(stderr, "%s:%d: no way for net %s\n", S.file,
					t->line, t->name);
				rip(order[o]);
				failed = 1;
			}
		}
	}
	for (int pass = 0; pass < PASSES; pass++) {
		for (size_t o = 0; o < ngroups; o++) {
			size_t g = order[o];
			struct group *gr = &groups[g];
			int *old = gr->edge, was = gr->cost;
			size_t n_old = gr->nedge;

			for (size_t i = 0; i < n_old; i++) {
				eown[old[i]] = -1;
			}
			gr->edge = NULL;
			gr->nedge = 0;
			gr->cost = 0;
			if (!route_group(g) || gr->cost > was) {
				// The new path is no better: the old one comes back.
				rip(g);
				free(gr->edge);
				gr->edge = old;
				gr->nedge = n_old;
				gr->cost = was;
				for (size_t i = 0; i < n_old; i++) {
					eown[old[i]] = (int)g;
				}
			} else {
				free(old);
			}
		}
	}
	free(order);
}

// Turns the wires of each group into polylines, from node to node: a
// node is a pin, an end, or a junction.
static void trace(void)
{
	int *xy = sch_alloc(2 * ((size_t)W * (size_t)H + 2) * sizeof *xy);
	unsigned char *seen = sch_alloc(2 * (size_t)W * (size_t)H);

	for (size_t g = 0; g < ngroups; g++) {
		for (int pass = 0; pass < 2; pass++) {
			for (int a = 0; a < W * H; a++) {
				unsigned m = ways(a, (int)g);

				if (!m || (pass == 0 && bits(m) == 2 &&
					   pinnet[a] == -1)) {
					continue;
				}
				for (int d = 0; d < 4; d++) {
					int n = 1, b = a, dd = d;

					if (!(m & (1u << d)) ||
					    seen[edge(a, d)]) {
						continue;
					}
					xy[0] = a % W + X0;
					xy[1] = a / W + Y0;
					for (;;) {
						unsigned bm;
						int nd = 0;

						seen[edge(b, dd)] = 1;
						b += DX[dd] + DY[dd] * W;
						bm = ways(b, (int)g);
						if (bits(bm) != 2 ||
						    pinnet[b] != -1 || b == a) {
							break;
						}
						while (!(bm & (1u << nd)) ||
						       nd == (dd + 2) % 4) {
							nd++;
						}
						if (nd != dd) {
							xy[2 * n] = b % W + X0;
							xy[2 * n + 1] =
								b / W + Y0;
							n++;
						}
						dd = nd;
					}
					xy[2 * n] = b % W + X0;
					xy[2 * n + 1] = b / W + Y0;
					n++;
					add_poly(xy, (size_t)n, groups[g].net);
				}
			}
		}
	}
	free(seen);
	free(xy);
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
		if (on_seg(x, y, p->xy[2 * j - 2], p->xy[2 * j - 1],
			   p->xy[2 * j], p->xy[2 * j + 1])) {
			return 1;
		}
	}
	return 0;
}

// The number of wire arms at (x, y): one for each wire ending there, two
// for each running through.
static int arms(int x, int y)
{
	int n = 0;

	for (size_t i = 0; i < npolys; i++) {
		const struct poly *p = &polys[i];
		int ends =
			(p->xy[0] == x && p->xy[1] == y) +
			(p->xy[2 * p->n - 2] == x && p->xy[2 * p->n - 1] == y);

		if (ends) {
			n += ends;
		} else if (on_poly(p, x, y)) {
			n += 2;
		}
	}
	return n;
}

// Whether a dot belongs at (x, y): three arms or more.
static int dot_at(int x, int y)
{
	return arms(x, y) >= 3;
}

// Adds a run to line l of the label.
static void add_run(struct label *b, int l, const char *s, int italic,
		    double size, double drop)
{
	struct run *r = &b->run[l][b->nrun[l]++];

	r->s = sch_strdup(s);
	r->italic = italic;
	r->size = size;
	r->drop = drop;
}

// The designator as a name with a subscript: R401 as R and 401.
static void add_ref(struct label *b, int l, const char *ref)
{
	char head[64], tail[64];
	size_t n = strcspn(ref, "0123456789");

	if (n >= sizeof head || strlen(ref + n) >= sizeof tail) {
		sch_die("%s: designator %s too long", S.file, ref);
	}
	memcpy(head, ref, n);
	head[n] = 0;
	strcpy(tail, ref + n);
	add_run(b, l, head, 1, TEXT, 0);
	if (*tail) {
		add_run(b, l, tail, 0, SUB, SUB_DROP);
	}
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
	b->h = DESCENT + TEXT * 0.72 + (b->nline - 1) * LEAD;
}

// The box of the text of a flag, in points, and where its text goes:
// beyond its end, the way out from the part.  Returns 0 for a flag
// without text.
static int flag_box(const struct sch_part *q, double b[4], double *tx,
		    double *ty, int *align)
{
	int x, y, d, out;
	double px, py, off, w;
	const char *fl = q->dev->shape->flag;

	if (strcmp(fl, "supply") && strcmp(fl, "port")) {
		return 0;
	}
	sch_pin_at(q, 0, &x, &y, &d);
	out = (d + 2) % 4;
	off = strcmp(fl, "port") == 0 ? 1.7 + 2.5 : 2.5;
	px = x * GRID + DX[out] * off;
	py = y * GRID + DY[out] * off;
	w = sch_text_width(q->text, TEXT);
	if (out == SCH_U || out == SCH_D) {
		*align = 1;
		*tx = px;
		*ty = out == SCH_U ? py : py - TEXT * 0.72;
		b[0] = px - w / 2;
	} else {
		*align = out == SCH_L ? 2 : 0;
		*tx = px;
		*ty = py - TEXT * 0.36;
		b[0] = out == SCH_L ? px - w : px;
	}
	b[1] = *ty - DESCENT;
	b[2] = b[0] + w;
	b[3] = *ty + TEXT * 0.72;
	return 1;
}

// Where the number of pin p of a part with a pin table goes, and its
// alignment; returns 0 for a part without a table.
static int pin_number_at(const struct sch_part *q, size_t p, double *x,
			 double *y, int *align)
{
	int px, py, d;

	if (q->dev->pnum[0] == q->dev->shape->pin[0].name) {
		return 0;
	}
	sch_pin_at(q, p, &px, &py, &d);
	*x = px * GRID - DX[d] * 0.45 * GRID;
	*y = py * GRID - DY[d] * 0.45 * GRID;
	if (d == SCH_L || d == SCH_R) {
		*y += 1.2;
		*align = 1;
	} else {
		*x += 1.5;
		*y -= PINNUM * 0.36;
		*align = 0;
	}
	return 1;
}

static double cover(double x0, double y0, double x1, double y1,
		    const double b[4])
{
	double ox = fmin(x1, b[2]) - fmax(x0, b[0]);
	double oy = fmin(y1, b[3]) - fmax(y0, b[1]);

	return ox > 0 && oy > 0 ? ox * oy + 10 : 0;
}

// How much of the box (points) the drawing already takes: wires,
// bodies, the text of flags, pin numbers and the labels placed before.
static double clash(double x0, double y0, double x1, double y1, size_t self)
{
	double c = 0, b[4];

	for (size_t i = 0; i < npolys; i++) {
		const struct poly *p = &polys[i];

		for (size_t j = 1; j < p->n; j++) {
			double ax = p->xy[2 * j - 2] * GRID,
			       ay = p->xy[2 * j - 1] * GRID;
			double bx = p->xy[2 * j] * GRID,
			       by = p->xy[2 * j + 1] * GRID;

			b[0] = fmin(ax, bx) - S.clear_text;
			b[1] = fmin(ay, by) - S.clear_text;
			b[2] = fmax(ax, bx) + S.clear_text;
			b[3] = fmax(ay, by) + S.clear_text;
			c += cover(x0, y0, x1, y1, b);
		}
	}
	for (size_t i = 0; i < S.npart; i++) {
		const struct sch_part *q = &S.part[i];
		double tx, ty;
		int al;

		sch_box(q, b);
		for (int k = 0; k < 4; k++) {
			double m = q->dev->shape->flag ? S.clear_flag :
							 S.clear_text;

			b[k] = b[k] * GRID + (k < 2 ? -m : m);
		}
		c += cover(x0, y0, x1, y1, b);
		if (q->dev->shape->flag) {
			if (flag_box(q, b, &tx, &ty, &al)) {
				c += cover(x0, y0, x1, y1, b);
			}
			continue;
		}
		for (size_t p = 0; p < q->dev->shape->npin; p++) {
			if (pin_number_at(q, p, &tx, &ty, &al)) {
				b[0] = tx - (al ? 3 : 0.5);
				b[1] = ty - 0.5;
				b[2] = tx + 3.5;
				b[3] = ty + PINNUM * 0.75;
				c += cover(x0, y0, x1, y1, b);
			}
		}
		if (i != self && labels[i].nline && labels[i].side >= 0) {
			const struct label *l = &labels[i];

			b[0] = l->x - S.clear_text;
			b[1] = l->y - S.clear_text;
			b[2] = l->x + l->w + S.clear_text;
			b[3] = l->y + l->h + S.clear_text;
			c += cover(x0, y0, x1, y1, b);
		}
	}
	return c;
}

// Places the label of part i: on each side of the part, its own side
// first, slid along the side and moved out from it, at the place of
// least clash, near its middle and its own side rather than not.
static void place_label(size_t i)
{
	const struct sch_part *q = &S.part[i];
	struct label *b = &labels[i];
	double bx[4], best = 1e300, gap = S.clear_label;
	int side = q->side >= 0 ? q->side : q->dev->shape->side, order[4];

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

		for (double out = 0; out <= 12; out += 3) {
			for (double t = -span; t <= span; t += 1.5) {
				double x, y, c;

				if (s == 0 || s == 2) {
					x = s == 0 ? bx[2] + gap + out :
						     bx[0] - gap - out - b->w;
					y = q->y * GRID - b->h / 2 + t;
				} else {
					x = q->x * GRID - b->w / 2 + t;
					y = s == 1 ? bx[3] + gap + 1.5 + out :
						     bx[1] - gap - out - b->h -
							     1;
				}
				// Room round text is a rule: a place that
				// keeps it wins over any that does not.
				c = clash(x, y, x + b->w, y + b->h, i);
				c = (c > 0 ? 1e6 + c : 0) + r * 8 +
				    fabs(t) * 0.4 + out;
				if (c < best) {
					best = c;
					b->x = x;
					b->y = y;
					b->side = s;
				}
			}
		}
	}
}

static void labels_all(void)
{
	labels = sch_alloc(S.npart * sizeof *labels);
	for (size_t i = 0; i < S.npart; i++) {
		const struct sch_part *q = &S.part[i];
		struct label *b = &labels[i];

		b->side = -1;
		if (q->dev->shape->flag) {
			continue;
		}
		add_ref(b, b->nline++, q->ref);
		if ((q->value || q->rating) && !q->novalue) {
			char v[512];

			snprintf(v, sizeof v, "%s%s%s",
				 q->value ? q->value : "",
				 q->value && q->rating ? " " : "",
				 q->rating ? q->rating : "");
			add_run(b, b->nline++, v, 0, TEXT, 0);
		}
		measure(b);
	}
	for (size_t i = 0; i < S.npart; i++) {
		if (labels[i].nline) {
			place_label(i);
		}
	}
}

// The transform of the page: the point of the drawing at the origin of
// the grid, and the scale.
static double OX, OY, SC;

// Writes a text record of a run, from (x, y) of the drawing, flush left.
static void text_run(const struct run *r, double x, double y)
{
	printf("text %c %.3f 0 0 %.3f %.3f %s\n", r->italic ? 'I' : 'R',
	       r->size * SC, OX + x * SC, OY + (y - r->drop) * SC, r->s);
}

// Writes runs from (x, y), aligned left (0), centered (1) or right (2).
static void text_runs(const struct run *r, int n, double x, double y, int align)
{
	double w = 0;

	for (int i = 0; i < n; i++) {
		w += sch_text_width(r[i].s, r[i].size);
	}
	x -= w * align / 2;
	for (int i = 0; i < n; i++) {
		text_run(&r[i], x, y);
		x += sch_text_width(r[i].s, r[i].size);
	}
}

// Writes the points of a line or polygon, from the drawing, as the page.
static void points(const double *xy, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		printf(" %.3f %.3f", OX + xy[2 * i] * SC,
		       OY + xy[2 * i + 1] * SC);
	}
	printf("\n");
}

// The text of a field, by its name after a $.
static const char *field(const char *name);

// Reads the sheet TEMPLATE.sheet and writes it, its fields filled in:
// the paper and the room for the drawing come back.
static void sheet(double room[4], double paper[2])
{
	char *path = sch_sheet_path(&S), buf[4096];
	FILE *f;

	room[0] = room[1] = room[2] = room[3] = 0;
	paper[0] = paper[1] = 0;
	if (!path) {
		sch_die("%s: no sheet %s in the library", S.file, S.sheet);
	}
	f = fopen(path, "r");
	if (!f) {
		sch_die("%s: cannot read", path);
	}
	while (fgets(buf, sizeof buf, f)) {
		const char *t;

		buf[strcspn(buf, "\n")] = 0;
		if (sscanf(buf, "paper %lf %lf", &paper[0], &paper[1]) == 2) {
			printf("page %.0f %.0f\n", paper[0], paper[1]);
		} else if (sscanf(buf, "room %lf %lf %lf %lf", &room[0],
				  &room[1], &room[2], &room[3]) == 4) {
		} else if (!strncmp(buf, "line ", 5)) {
			printf("line %.2f%s\n", LW, buf + 4);
		} else if (!strncmp(buf, "text ", 5) &&
			   (t = strchr(buf, '$')) != NULL) {
			if (*field(t + 1)) {
				printf("%.*s%s\n", (int)(t - buf), buf,
				       field(t + 1));
			}
		} else if (!strncmp(buf, "text ", 5)) {
			printf("%s\n", buf);
		}
	}
	fclose(f);
	if (!(paper[0] > 0) || !(room[2] > room[0])) {
		sch_die("%s: no paper or room", path);
	}
	free(path);
}

// Writes the records of a part's shape, turned and placed as the part.
static void symbol(const struct sch_part *q)
{
	const struct sch_shape *sh = q->dev->shape;
	double c = (double)(q->rot == 0) - (double)(q->rot == 2);
	double sn = (double)(q->rot == 1) - (double)(q->rot == 3);

	for (size_t i = 0; i < sh->ndraw; i++) {
		const struct sch_draw *d = &sh->draw[i];
		size_t np = d->kind >= SCH_CIRCLE ? 1 : d->n / 2;
		double *xy = sch_alloc(2 * (np + 1) * sizeof *xy);

		if ((d->when == 1 && !q->flip) || (d->when == 2 && q->flip)) {
			free(xy);
			continue;
		}
		for (size_t k = 0; k < np; k++) {
			double x = q->mirror ? -d->xy[2 * k] : d->xy[2 * k];
			double y = d->xy[2 * k + 1];

			xy[2 * k] = q->x * GRID + x * c - y * sn;
			xy[2 * k + 1] = q->y * GRID + x * sn + y * c;
		}
		if (d->kind == SCH_LINE || d->kind == SCH_LOOP) {
			if (d->kind == SCH_LOOP) {
				xy[2 * np] = xy[0];
				xy[2 * np + 1] = xy[1];
				np++;
			}
			printf("line %.3f", d->w * SC);
			points(xy, np);
		} else if (d->kind == SCH_FILL || d->kind == SCH_BLANK) {
			printf("%spoly", d->kind == SCH_BLANK ? "clear\n" : "");
			points(xy, np);
			if (d->kind == SCH_BLANK) {
				printf("dark\n");
			}
		} else if (d->kind == SCH_DISK) {
			printf("disk %.3f %.3f %.3f\n", OX + xy[0] * SC,
			       OY + xy[1] * SC, d->xy[2] * SC);
		} else {
			double r = d->xy[2], cx = xy[0], cy = xy[1];

			free(xy);
			xy = sch_alloc(2 * 129 * sizeof *xy);
			for (int k = 0; k <= 128; k++) {
				xy[2 * k] = cx + r * cos(2 * PI * k / 128);
				xy[2 * k + 1] = cy + r * sin(2 * PI * k / 128);
			}
			printf("line %.3f", d->w * SC);
			points(xy, 129);
		}
		free(xy);
	}
}

// The text of a field: from the source, or made.
static const char *field(const char *name)
{
	for (size_t i = 0; i < S.nfield; i++) {
		if (strcmp(S.field[i][0], name) == 0) {
			return S.field[i][1];
		}
	}
	if (strcmp(name, "Page") == 0) {
		static char page[16];

		snprintf(page, sizeof page, "%d", S.page);
		return page;
	}
	if (strcmp(name, "File") == 0) {
		const char *base = strrchr(S.file, '/');

		return base ? base + 1 : S.file;
	}
	return "";
}

// The extent of the drawing in grid units.
static void extent(double e[4])
{
	e[0] = e[1] = 1e9;
	e[2] = e[3] = -1e9;
	for (size_t i = 0; i < S.npart; i++) {
		double b[4];

		sch_box(&S.part[i], b);
		e[0] = fmin(e[0], b[0]);
		e[1] = fmin(e[1], b[1]);
		e[2] = fmax(e[2], b[2]);
		e[3] = fmax(e[3], b[3]);
		if (labels[i].nline) {
			e[0] = fmin(e[0], labels[i].x / GRID);
			e[1] = fmin(e[1], labels[i].y / GRID);
			e[2] = fmax(e[2], (labels[i].x + labels[i].w) / GRID);
			e[3] = fmax(e[3], (labels[i].y + labels[i].h) / GRID);
		}
	}
	for (size_t i = 0; i < npolys; i++) {
		for (size_t j = 0; j < polys[i].n; j++) {
			e[0] = fmin(e[0], polys[i].xy[2 * j]);
			e[1] = fmin(e[1], polys[i].xy[2 * j + 1]);
			e[2] = fmax(e[2], polys[i].xy[2 * j]);
			e[3] = fmax(e[3], polys[i].xy[2 * j + 1]);
		}
	}
	e[0] -= 3;
	e[2] += 3;
	e[1] -= 2;
	e[3] += 2;
}

// The text of a flag: the name of its net beyond its end.
static void flag_text(const struct sch_part *q)
{
	double b[4], x, y;
	int align;
	struct run r = { q->text, 0, TEXT, 0 };

	if (flag_box(q, b, &x, &y, &align)) {
		text_runs(&r, 1, x, y, align);
	}
}

// The pin numbers of a part with a pin table, by its leads.
static void pin_numbers(const struct sch_part *q)
{
	for (size_t p = 0; p < q->dev->shape->npin; p++) {
		double x, y;
		int align;
		struct run r = { q->dev->pnum[p], 0, PINNUM, 0 };

		if (pin_number_at(q, p, &x, &y, &align)) {
			text_runs(&r, 1, x, y, align);
		}
	}
}

// Where each pin of a part meets its wire, by its designator and number
// (pin R401.2 X Y), or a flag's, by its net (flag GND X Y).
static void pin_ends(const struct sch_part *q)
{
	for (size_t p = 0; p < q->dev->shape->npin; p++) {
		int x, y, d;

		sch_pin_at(q, p, &x, &y, &d);
		if (q->dev->shape->flag) {
			printf("flag %s", q->text);
		} else {
			printf("pin %s.%s", q->ref, sch_pnum(q->dev, p));
		}
		printf(" %.3f %.3f\n", OX + x * GRID * SC, OY + y * GRID * SC);
	}
}

// Writes the sheet as vec(5): the template, then each net's wires and
// dots, then each part, its symbol and its text, and the flags.
static void write_vec(void)
{
	double room[4], paper[2], e[4];

	printf("# vec of %s, by schdraw\n", S.file);
	sheet(room, paper);
	extent(e);
	SC = S.scale;
	if ((e[2] - e[0]) * GRID * SC > room[2] - room[0] ||
	    (e[3] - e[1]) * GRID * SC > room[3] - room[1]) {
		fprintf(stderr,
			"%s: the drawing is larger than the room of "
			"the sheet\n",
			S.file);
	}
	OX = (room[0] + room[2]) / 2 - (e[0] + e[2]) / 2 * GRID * SC;
	OY = (room[1] + room[3]) / 2 - (e[1] + e[3]) / 2 * GRID * SC;
	for (size_t k = 0; k < S.nnet; k++) {
		int any = 0;

		for (size_t i = 0; i < npolys; i++) {
			const struct poly *p = &polys[i];
			double *xy;

			if (p->net != k || p->n < 2) {
				continue;
			}
			if (!any) {
				printf("net %s\n", S.net[k].name);
				any = 1;
			}
			xy = sch_alloc(2 * p->n * sizeof *xy);
			for (size_t j = 0; j < 2 * p->n; j++) {
				xy[j] = p->xy[j] * GRID;
			}
			printf("line %.3f", LW * SC);
			points(xy, p->n);
			free(xy);
		}
		for (size_t i = 0; i < npolys; i++) {
			const struct poly *p = &polys[i];

			for (size_t j = 0; p->net == k && j < p->n;
			     j += p->n - 1) {
				int x = p->xy[2 * j], y = p->xy[2 * j + 1];
				int dup = 0;

				for (size_t m = 0; m < i && !dup; m++) {
					const struct poly *o = &polys[m];

					dup = (o->xy[0] == x &&
					       o->xy[1] == y) ||
					      (o->xy[2 * o->n - 2] == x &&
					       o->xy[2 * o->n - 1] == y);
				}
				if (!dup && j > 0 && p->n > 1) {
					dup = p->xy[0] == x && p->xy[1] == y;
				}
				if (!dup && dot_at(x, y)) {
					printf("disk %.3f %.3f %.3f\n",
					       OX + x * GRID * SC,
					       OY + y * GRID * SC, DOT_R * SC);
				}
				if (p->n == 1) {
					break;
				}
			}
		}
		if (any) {
			printf("net\n");
		}
	}
	for (size_t i = 0; i < S.npart; i++) {
		const struct sch_part *q = &S.part[i];
		const struct label *b = &labels[i];
		double bx[4];

		if (q->dev->shape->flag) {
			symbol(q);
			flag_text(q);
			pin_ends(q);
			continue;
		}
		sch_box(q, bx);
		printf("part %s %s\nname %s %.3f %.3f %.3f %.3f\n", q->name,
		       q->ref, q->name, OX + (bx[0] * GRID - 2) * SC,
		       OY + (bx[1] * GRID - 2) * SC,
		       OX + (bx[2] * GRID + 2) * SC,
		       OY + (bx[3] * GRID + 2) * SC);
		symbol(q);
		pin_numbers(q);
		pin_ends(q);
		// The lines align to the part: to the left of it flush right,
		// above or below it centered, else flush left.
		for (int l = 0; l < b->nline; l++) {
			double y = b->y + b->h - TEXT * 0.72 - l * LEAD;

			if (b->side == SCH_U || b->side == SCH_D) {
				text_runs(b->run[l], b->nrun[l],
					  b->x + b->w / 2, y, 1);
			} else if (b->side == SCH_L) {
				text_runs(b->run[l], b->nrun[l], b->x + b->w, y,
					  2);
			} else {
				text_runs(b->run[l], b->nrun[l], b->x, y, 0);
			}
		}
		printf("part\n");
	}
}

// The distance between two boxes, x0 y0 x1 y1.
static double apart(const double *a, const double *b)
{
	return hypot(fmax(0, fmax(b[0] - a[2], a[0] - b[2])),
		     fmax(0, fmax(b[1] - a[3], a[1] - b[3])));
}

// The box of a part's body, or of its label, in points.
static int body(size_t i, double b[4])
{
	sch_box(&S.part[i], b);
	for (int k = 0; k < 4; k++) {
		b[k] *= GRID;
	}
	return !S.part[i].dev->shape->flag;
}

static int label(size_t i, double b[4])
{
	const struct label *l = &labels[i];

	b[0] = l->x;
	b[1] = l->y;
	b[2] = l->x + l->w;
	b[3] = l->y + l->h;
	return l->nline && l->side >= 0;
}

// The designator of part k for a message, or the kind of a flag.
static const char *ref_of(size_t k)
{
	return S.part[k].ref ? S.part[k].ref : S.part[k].dev->shape->name;
}

// Measures the room the drawing keeps: a wire from the bodies of parts
// not on its net, a body from a body, and a label from every wire, body
// and other label; each pair too close is a fault.
static void check_room(void)
{
	double b[4], c[4], e = 0.01;

	for (size_t i = 0; i < npolys; i++) {
		const struct poly *p = &polys[i];

		for (size_t j = 1; j < p->n; j++) {
			double w[4] = {
				fmin(p->xy[2 * j - 2], p->xy[2 * j]) * GRID,
				fmin(p->xy[2 * j - 1], p->xy[2 * j + 1]) * GRID,
				fmax(p->xy[2 * j - 2], p->xy[2 * j]) * GRID,
				fmax(p->xy[2 * j - 1], p->xy[2 * j + 1]) * GRID
			};

			for (size_t k = 0; k < S.npart; k++) {
				int on = 0;

				for (size_t r = 0;
				     r < S.part[k].dev->shape->npin; r++) {
					on |= net_of(k, r) == (int)p->net;
				}
				if (!on && body(k, b) &&
				    apart(w, b) < S.clear_body - e) {
					fprintf(stderr,
						"%s:%d: a wire of %s runs %.1f pt from %s (%s)\n",
						S.file, S.part[k].line,
						S.net[p->net].name, apart(w, b),
						S.part[k].name, ref_of(k));
					failed = 1;
				}
				if (label(k, b) &&
				    apart(w, b) < S.clear_text - e) {
					fprintf(stderr,
						"%s:%d: a wire of %s runs %.1f pt from "
						"the label of %s (%s)\n",
						S.file, S.part[k].line,
						S.net[p->net].name, apart(w, b),
						S.part[k].name, ref_of(k));
					failed = 1;
				}
			}
		}
	}
	for (size_t i = 0; i < S.npart; i++) {
		for (size_t k = 0; k < S.npart; k++) {
			const char *what = NULL;
			double room = S.clear_text;

			if (k > i && body(i, b) && body(k, c) &&
			    apart(b, c) < S.clear_body - e) {
				what = "";
				room = apart(b, c);
			} else if (k != i && label(i, b) && body(k, c) &&
				   apart(b, c) < S.clear_text - e) {
				what = "the label of ";
				room = apart(b, c);
			} else if (k > i && label(i, b) && label(k, c) &&
				   apart(b, c) < S.clear_text - e) {
				what = "the labels of ";
				room = apart(b, c);
			}
			if (what) {
				fprintf(stderr,
					"%s:%d: %s%s (%s) is %.1f pt from %s (%s)\n",
					S.file, S.part[i].line, what,
					S.part[i].name, ref_of(i), room,
					S.part[k].name, ref_of(k));
				failed = 1;
			}
		}
	}
}

// Leaves out of the drawing the parts that found no place (the reader
// told why), and their pins out of the nets and the wires.
static void drop_unplaced(void)
{
	size_t *to = sch_alloc((S.npart + 1) * sizeof *to), n = 0, m;

	for (size_t i = 0; i < S.npart; i++) {
		to[i] = S.part[i].placed ? n : (size_t)-1;
		if (S.part[i].placed) {
			S.part[n++] = S.part[i];
		}
	}
	S.npart = n;
	for (size_t k = 0; k < S.nnet; k++) {
		struct sch_net *t = &S.net[k];

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
	for (size_t h = 0; h < S.nhint; h++) {
		struct sch_hint hn = S.hint[h];

		if (to[hn.part[0]] != (size_t)-1 &&
		    to[hn.part[1]] != (size_t)-1) {
			hn.part[0] = to[hn.part[0]];
			hn.part[1] = to[hn.part[1]];
			S.hint[m++] = hn;
		}
	}
	S.nhint = m;
	free(to);
}

int main(int argc, char **argv)
{
	char **lib = NULL;
	size_t nlib = 0;
	int i = 1, faults;

	for (; i < argc && argv[i][0] == '-'; i++) {
		if (strcmp(argv[i], "-v") == 0) {
			sch_verbose = 1;
		} else if (strcmp(argv[i], "-L") == 0 && i + 1 < argc) {
			lib = xrealloc(lib, (nlib + 1) * sizeof *lib);
			lib[nlib] = sch_alloc(strlen(argv[i + 1]) + 16);
			sprintf(lib[nlib++], "%s/symbols", argv[++i]);
		} else {
			sch_die("usage: schdraw [-v] [-L LIB]... sheet.sch > sheet.vec");
		}
	}
	if (i != argc - 1) {
		sch_die("usage: schdraw [-v] [-L LIB]... sheet.sch > sheet.vec");
	}
	if (!nlib) {
		lib = sch_alloc(sizeof *lib);
		lib[0] = sch_default_lib(argv[0]);
		nlib = 1;
	}
	faults = sch_read(&S, argv[i], lib, nlib);
	drop_unplaced();
	grid();
	make_groups();
	route_all();
	trace();
	labels_all();
	check_room();
	write_vec();
	return failed || faults;
}
